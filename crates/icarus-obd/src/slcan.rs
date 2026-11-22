//! SLCAN, the serial CAN protocol of Lawicel's CANUSB that the CANable's slcan firmware and many cheap USB-CAN sticks
//! speak: for watching the bus with no `slcand` and no root, Icarus talks to the adapter over its serial port itself.
//!
//! **Listen-only.** The channel is opened with `L` (listen-only: the adapter takes no part in the bus, so it acknowledges
//! nothing), never with `O` (normal mode, in which it would acknowledge every frame), and no frame is ever sent. The only
//! commands that go to the adapter are closing the channel (`C`), the bit rate (`S0`-`S8`) and `L`: `command` refuses
//! anything else. An adapter that refuses `L` is not watched through at all.
//!
//! Commands end with a carriage return; the adapter answers a carriage return for OK and BEL (07) for an error. Frames
//! come as lines: `t` + 3 hex digits of 11-bit ID + length + data, `T` + 8 digits of 29-bit ID + length + data, with or
//! without a 4-digit time stamp after the data. Remote frames (`r`, `R`) carry no data and are skipped.

use crate::bus::Frame;
use crate::elm::ObdError;
use crate::port::Port;
use std::io;
use std::time::{Duration, Instant};

/// The bit rates `S0` to `S8` set, in bit/s.
pub const BITRATES: [u32; 9] = [
    10_000, 20_000, 50_000, 100_000, 125_000, 250_000, 500_000, 800_000, 1_000_000,
];

/// How long the adapter has to answer a command.
const ANSWER: Duration = Duration::from_secs(1);

/// Whether Icarus may send `cmd` to an SLCAN adapter: `C`, `S0`-`S8` and `L`, nothing else (never `O`, never a frame).
pub fn allowed(cmd: &str) -> bool {
    matches!(cmd, "C" | "L")
        || (cmd.len() == 2 && cmd.starts_with('S') && matches!(cmd.as_bytes()[1], b'0'..=b'8'))
}

/// An SLCAN adapter whose channel Icarus opened listen-only.
pub struct Slcan {
    port: Box<dyn Port>,
    buf: Vec<u8>,
    open: bool,
}

impl Slcan {
    /// Opens the channel listen-only at `bitrate` (one of `BITRATES`): `C` first (a channel another program left open is
    /// closed; an adapter whose channel is closed already answers it with an error, which is fine), then `S<n>`, then `L`.
    pub fn listen(port: Box<dyn Port>, bitrate: u32) -> Result<Slcan, ObdError> {
        let n = BITRATES.iter().position(|b| *b == bitrate).ok_or_else(|| {
            ObdError::Adapter(format!(
                "SLCAN adapters take these bit rates: {}",
                BITRATES.map(|b| b.to_string()).join(", ")
            ))
        })?;
        let mut s = Slcan {
            port,
            buf: Vec::new(),
            open: false,
        };
        match s.command("C") {
            Ok(()) | Err(ObdError::Adapter(_)) => {}
            Err(e) => return Err(e),
        }
        s.command(&format!("S{n}")).map_err(|e| match e {
            ObdError::Adapter(_) => {
                ObdError::Adapter(format!("the adapter refused the bit rate {bitrate} (S{n})"))
            }
            other => other,
        })?;
        s.command("L").map_err(|e| match e {
            ObdError::Adapter(_) => ObdError::Adapter(
                "the adapter would not open the bus listen-only (L), so Icarus does not watch through it (it never \
                 opens the bus in normal mode, which would acknowledge frames)"
                    .into(),
            ),
            other => other,
        })?;
        s.open = true;
        Ok(s)
    }

    /// Sends one allowed command and waits for its answer: OK (a bare carriage return) or the adapter's error (BEL).
    /// Frame lines that arrive meanwhile are passed over.
    fn command(&mut self, cmd: &str) -> Result<(), ObdError> {
        if !allowed(cmd) {
            return Err(ObdError::Adapter(format!(
                "Icarus sends an SLCAN adapter C, S0-S8 and L only, not {cmd:?}"
            )));
        }
        self.buf.clear();
        self.port.send(format!("{cmd}\r").as_bytes())?;
        let deadline = Instant::now() + ANSWER;
        let mut tmp = [0u8; 256];
        loop {
            while let Some(i) = self.buf.iter().position(|b| *b == b'\r' || *b == 0x07) {
                let (bel, empty) = (self.buf[i] == 0x07, i == 0);
                self.buf.drain(..=i);
                if bel {
                    return Err(ObdError::Adapter(format!("the adapter refused {cmd}")));
                }
                if empty {
                    return Ok(());
                }
                // a frame line that came before the answer
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Err(ObdError::Timeout(format!("{cmd} (SLCAN)")));
            }
            let n = self
                .port
                .recv(&mut tmp, left.min(Duration::from_millis(100)))?;
            self.buf.extend_from_slice(&tmp[..n]);
        }
    }

    /// The next data frame, waiting up to `timeout`, `at` seconds into the watch (None: none came in time). Remote frames
    /// and lines that are not frames are passed over.
    pub fn read(&mut self, timeout: Duration, at: f64) -> io::Result<Option<Frame>> {
        let deadline = Instant::now() + timeout;
        let mut tmp = [0u8; 1024];
        loop {
            while let Some(i) = self.buf.iter().position(|b| *b == b'\r' || *b == 0x07) {
                let line = String::from_utf8_lossy(&self.buf[..i]).to_string();
                self.buf.drain(..=i);
                if let Some(f) = parse(line.trim(), at) {
                    return Ok(Some(f));
                }
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Ok(None);
            }
            let n = self.port.recv(&mut tmp, left)?;
            self.buf.extend_from_slice(&tmp[..n]);
            if n == 0 && Instant::now() >= deadline {
                return Ok(None);
            }
        }
    }

    /// Closes the channel (`C`).
    pub fn close(&mut self) -> Result<(), ObdError> {
        if std::mem::take(&mut self.open) {
            self.command("C")?;
        }
        Ok(())
    }
}

impl Drop for Slcan {
    fn drop(&mut self) {
        let _ = self.close();
    }
}

/// One SLCAN line as a data frame: `tIIILDD..` (11-bit) or `TIIIIIIIILDD..` (29-bit), the data `L` bytes long (0-8), with
/// or without a 4-digit time stamp after it. Remote frames (`r`, `R`) and anything else are not data frames.
pub fn parse(line: &str, at: f64) -> Option<Frame> {
    let (ext, digits) = match line.as_bytes().first()? {
        b't' => (false, 3),
        b'T' => (true, 8),
        _ => return None,
    };
    let rest = line.get(1..)?;
    if !rest.chars().all(|c| c.is_ascii_hexdigit()) || rest.len() < digits + 1 {
        return None;
    }
    let id = u32::from_str_radix(&rest[..digits], 16).ok()?;
    if id > if ext { 0x1FFF_FFFF } else { 0x7FF } {
        return None;
    }
    let len = usize::from_str_radix(&rest[digits..digits + 1], 16).ok()?;
    let hex = &rest[digits + 1..];
    if len > 8 || (hex.len() != 2 * len && hex.len() != 2 * len + 4) {
        return None;
    }
    let data = (0..len)
        .map(|i| u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).ok())
        .collect::<Option<Vec<u8>>>()?;
    Some(Frame { id, ext, data, at })
}

/// A recorded SLCAN session played back (for the tests): an adapter that answers C, S0-S8 and L (and anything else) OK,
/// and once its channel is open streams the recorded frame lines; everything sent to it is kept (one entry per send).
pub mod recorded {
    use crate::port::Port;
    use std::collections::VecDeque;
    use std::io;
    use std::sync::{Arc, Mutex};
    use std::time::Duration;

    pub struct Session {
        stream: VecDeque<u8>,
        out: VecDeque<u8>,
        listening: bool,
        pub sent: Arc<Mutex<Vec<String>>>,
        /// Commands the adapter answers with BEL.
        pub refuses: Vec<&'static str>,
    }

    impl Session {
        /// A session whose adapter streams `lines` once its channel is open, and the log of what was sent to it.
        pub fn new(lines: &[&str]) -> (Session, Arc<Mutex<Vec<String>>>) {
            let sent = Arc::new(Mutex::new(Vec::new()));
            let stream = lines
                .iter()
                .flat_map(|l| format!("{l}\r").into_bytes())
                .collect();
            (
                Session {
                    stream,
                    out: VecDeque::new(),
                    listening: false,
                    sent: sent.clone(),
                    refuses: Vec::new(),
                },
                sent,
            )
        }
    }

    impl Port for Session {
        fn send(&mut self, data: &[u8]) -> io::Result<()> {
            let cmd = String::from_utf8_lossy(data).to_string();
            self.sent.lock().unwrap().push(cmd.clone());
            let c = cmd.trim_end_matches('\r');
            if self.refuses.contains(&c) {
                self.out.push_back(0x07);
                return Ok(());
            }
            match c {
                "L" | "O" => self.listening = true,
                "C" => self.listening = false,
                _ => {}
            }
            self.out.push_back(b'\r');
            Ok(())
        }

        fn recv(&mut self, buf: &mut [u8], _timeout: Duration) -> io::Result<usize> {
            if self.out.is_empty() && self.listening {
                // the stream, a few bytes at a time, as a serial line delivers it
                let n = self.stream.len().min(7);
                self.out.extend(self.stream.drain(..n));
            }
            if self.out.is_empty() {
                std::thread::sleep(Duration::from_millis(2));
            }
            let n = buf.len().min(self.out.len());
            for (i, b) in self.out.drain(..n).enumerate() {
                buf[i] = b;
            }
            Ok(n)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::recorded::Session;
    use super::*;

    #[test]
    fn frame_lines() {
        let f = parse(
            "t2C4803 1A F8 00 1C 00 00 00".replace(' ', "").as_str(),
            1.5,
        )
        .unwrap();
        assert_eq!(
            (f.id, f.ext, f.data.clone(), f.at),
            (
                0x2C4,
                false,
                vec![0x03, 0x1A, 0xF8, 0x00, 0x1C, 0x00, 0x00, 0x00],
                1.5
            )
        );
        let e = parse("T18DAF1103410D00", 0.0).unwrap();
        assert_eq!(
            (e.id, e.ext, e.data),
            (0x18DA_F110, true, vec![0x41, 0x0D, 0x00])
        );
        // a time stamp after the data is passed over; an empty frame is a frame
        assert_eq!(parse("t3B7201001A2B", 0.0).unwrap().data, vec![0x01, 0x00]);
        assert_eq!(parse("t1230", 0.0).unwrap().data, Vec::<u8>::new());
        for bad in [
            "r1230",
            "R18DAF1100",
            "t12",
            "t1239",
            "t123211",
            "t8001AA",
            "T200000001AA",
            "tXYZ1AA",
            "z",
            "",
        ] {
            assert!(parse(bad, 0.0).is_none(), "{bad}");
        }
    }

    #[test]
    fn only_close_bit_rate_and_listen_only_are_ever_sent() {
        for ok in ["C", "L", "S0", "S6", "S8"] {
            assert!(allowed(ok), "{ok}");
        }
        for never in [
            "O",
            "S9",
            "t1230",
            "T000000000",
            "r1230",
            "V",
            "Q1",
            "X1",
            "Z1",
            "W2",
            "M00000000",
            "",
        ] {
            assert!(!allowed(never), "{never}");
        }
        let (port, sent) = Session::new(&[]);
        let mut s = Slcan::listen(Box::new(port), 500_000).unwrap();
        assert!(s.command("O").is_err(), "refused before it is sent");
        s.close().unwrap();
        assert_eq!(*sent.lock().unwrap(), ["C\r", "S6\r", "L\r", "C\r"]);
    }

    #[test]
    fn an_adapter_that_refuses_listen_only_is_not_watched_and_never_opened() {
        let (mut port, sent) = Session::new(&["t1230"]);
        port.refuses.push("L");
        let e = Slcan::listen(Box::new(port), 500_000)
            .err()
            .unwrap()
            .to_string();
        assert!(e.contains("listen-only"), "{e}");
        let log = sent.lock().unwrap().clone();
        assert!(!log.iter().any(|c| c.starts_with('O')), "{log:?}");
        // a bit rate SLCAN has no command for is refused before anything is sent
        let (port, sent) = Session::new(&[]);
        assert!(Slcan::listen(Box::new(port), 33_300).is_err());
        assert!(sent.lock().unwrap().is_empty());
        // an adapter whose channel was closed already answers the first C with an error: that is fine
        let (mut port, _) = Session::new(&[]);
        port.refuses.push("C");
        assert!(Slcan::listen(Box::new(port), 125_000).is_ok());
    }
}
