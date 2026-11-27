//! The Tactrix OpenPort 2.0 (`0403:cc4d`), a J2534 cable, driven by Icarus itself: no Windows library and no libusb. Linux's
//! own `cdc_acm` driver makes it a serial port (`/dev/ttyACM0`), and the cable's command language goes over that port: short
//! text commands it answers with `ar...` lines, and binary packets for what it receives from the vehicle. `OpenPort` turns
//! that into the `PassThru` operations, so `PassThruLink` reaches OBD-II and each module through it as through any J2534
//! device (`--adapter openport`).
//!
//! The command words and the packet layout were learned by reading NikolaKozina's and Dale Schultz's j2534 library for Linux
//! and macOS (github.com/NikolaKozina/j2534, BSD 3-Clause, with thanks; its notice is in docs/THIRD_PARTY.md). That library is
//! C over libusb; the code here is Icarus's own. It has not met a cable in hand yet: the tests use a stand-in that answers the
//! way that library expects the cable to.
//!
//! | Sent (each ends with CR LF) | The cable's answer | J2534 |
//! | --- | --- | --- |
//! | `ati` | `ari ... : <firmware version>` | PassThruReadVersion |
//! | `ata`, `atz` | `aro` | PassThruOpen, PassThruClose |
//! | `ato<protocol> <flags> <baud> 0` | `aro` | PassThruConnect (the channel is the protocol's number) |
//! | `atc<channel>` | `aro` | PassThruDisconnect |
//! | `att<channel> <size> <tx flags>`, then the message | `aro` | PassThruWriteMsgs |
//! | `atf<channel> <type> <tx flags> <size>`, then mask, pattern and flow control | `arf... <filter>` | PassThruStartMsgFilter |
//! | `atr 16` | `arr 16 <millivolts>` | PassThruIoctl READ_VBATT (the OBD port's pin 16) |
//! | anything it refuses | `are <J2534 error number>` | |
//!
//! What the cable receives comes as packets: `a` `r`, the channel as a digit (`6` is ISO 15765), the length of the rest as a
//! byte, the packet's kind, then on CAN a four-byte timestamp, the four-byte CAN ID and the data (big-endian). Kind 0x00 is a
//! message, or on ISO 15765 and the K-line a piece of one that 0x40 (0x44 with extended addressing) ends; on the K-line a
//! piece is the bytes alone. Loopback (0x20, 0x60), "sent" (0x10) and "a long message is starting" (0x80, 0xA0) are not
//! answers and are passed over.

use crate::elm::ObdError;
use crate::passthru::{J2534Protocol, PassThru, PassThruMsg, Versions};
use crate::port::{FdPort, Port};
use std::collections::VecDeque;
use std::path::Path;
use std::time::{Duration, Instant};

/// Its USB IDs and name.
pub const VENDOR: u16 = 0x0403;
pub const PRODUCT: u16 = 0xcc4d;
pub const NAME: &str = "Tactrix OpenPort 2.0";

/// How long the cable has to answer a command.
const ANSWER: Duration = Duration::from_secs(2);
/// J2534-1's TxFlags and connect flags.
const ISO15765_FRAME_PAD: u32 = 0x40;
const CAN_29BIT_ID: u32 = 0x100;
const CAN_ID_BOTH: u32 = 0x800;
/// J2534-1's FLOW_CONTROL_FILTER.
const FLOW_CONTROL_FILTER: u32 = 3;
/// The OBD port's battery pin, which READ_VBATT measures.
const BATTERY_PIN: u32 = 16;
/// The longest ISO 15765 message.
const LONGEST: usize = 4095;
/// Messages kept for `read` at most (a raw CAN channel nobody reads would otherwise grow without end).
const INBOX: usize = 4096;

/// The channel number the cable uses for a protocol: J2534's own protocol ID.
fn number(p: J2534Protocol) -> u32 {
    match p {
        J2534Protocol::Iso9141 => 3,
        J2534Protocol::Iso14230 => 4,
        J2534Protocol::Can => 5,
        J2534Protocol::Iso15765 => 6,
    }
}

fn is_can(channel: u32) -> bool {
    channel == 5 || channel == 6
}

/// The TxFlags for a message or filter on `channel` to or from `id`: ISO 15765 frames padded to eight bytes (as OBD-II
/// modules expect), and 29-bit IDs said as such.
fn tx_flags(channel: u32, id: u32) -> u32 {
    let pad = if channel == 6 { ISO15765_FRAME_PAD } else { 0 };
    let long = if is_can(channel) && id > 0x7FF {
        CAN_29BIT_ID
    } else {
        0
    };
    pad | long
}

/// J2534-1's error numbers, as `are` gives them.
fn error_name(n: u32) -> &'static str {
    match n {
        0x01 => "not supported",
        0x02 => "invalid channel ID",
        0x03 => "invalid protocol ID",
        0x04 => "null parameter",
        0x05 => "invalid ioctl value",
        0x06 => "invalid flags",
        0x07 => "failed",
        0x08 => "device not connected",
        0x09 => "timeout",
        0x0A => "invalid message",
        0x0B => "invalid time interval",
        0x0C => "exceeded limit",
        0x0D => "invalid message ID",
        0x0E => "device in use",
        0x0F => "invalid ioctl ID",
        0x10 => "buffer empty",
        0x11 => "buffer full",
        0x12 => "buffer overflow",
        0x13 => "pin invalid",
        0x14 => "channel in use",
        0x15 => "message protocol ID",
        0x16 => "invalid filter ID",
        0x17 => "no flow control",
        0x18 => "not unique",
        0x19 => "invalid baud rate",
        0x1A => "invalid device ID",
        _ => "an error J2534 does not name",
    }
}

fn refusal(what: &str, line: &str) -> ObdError {
    let n = line
        .trim_start_matches("are")
        .split_whitespace()
        .next()
        .and_then(|w| w.parse::<u32>().ok());
    ObdError::Adapter(match n {
        Some(n) => format!(
            "the {NAME} refused {what}: J2534 error {n}, {}",
            error_name(n)
        ),
        None => format!("the {NAME} refused {what}: {line}"),
    })
}

/// The version in `ati`'s answer: the word after its last colon ("ari main code version : 1.17.4877"), or the first word.
fn firmware(line: &str) -> String {
    let rest = line.strip_prefix("ari").unwrap_or(line);
    rest.rsplit(':')
        .next()
        .and_then(|t| t.split_whitespace().next())
        .unwrap_or_default()
        .to_string()
}

/// One thing the cable sent.
#[derive(Debug, PartialEq, Eq)]
enum Piece {
    /// A text answer (`aro`, `ari ...`, `arf...`, `arr ...`, `are ...`), without its line end.
    Text(String),
    /// A packet: its channel, its kind and what follows the kind.
    Packet {
        channel: u8,
        kind: u8,
        body: Vec<u8>,
    },
}

/// The next whole piece at the front of `buf`, taken out of it; None until one has come in full. Bytes before an `ar` (left
/// over from before Icarus opened the port) are dropped.
fn next_piece(buf: &mut Vec<u8>) -> Option<Piece> {
    loop {
        match buf.windows(2).position(|w| w == b"ar") {
            Some(0) => {}
            Some(i) => {
                buf.drain(..i);
            }
            None => {
                // a last `a` stays: its `r` may be on the way
                let keep = usize::from(buf.last() == Some(&b'a'));
                buf.drain(..buf.len() - keep);
                return None;
            }
        }
        let &c = buf.get(2)?;
        if c.is_ascii_digit() {
            let &len = buf.get(3)?;
            let total = 4 + len as usize;
            if buf.len() < total {
                return None;
            }
            let p: Vec<u8> = buf.drain(..total).collect();
            if len == 0 {
                continue;
            }
            return Some(Piece::Packet {
                channel: c - b'0',
                kind: p[4],
                body: p[5..].to_vec(),
            });
        }
        match buf.windows(2).position(|w| w == b"\r\n") {
            Some(end) => {
                let line: Vec<u8> = buf.drain(..end + 2).collect();
                return Some(Piece::Text(
                    String::from_utf8_lossy(&line[..end]).trim().to_string(),
                ));
            }
            // a line that never ends: noise that happened to start with "ar"; look past it
            None if buf.len() > 256 => {
                buf.drain(..2);
            }
            None => return None,
        }
    }
}

/// An OpenPort 2.0 behind a byte stream: its serial port (`open_tty`), or a stand-in in tests.
pub struct OpenPort<P: Port> {
    port: P,
    buf: Vec<u8>,
    firmware: String,
    /// Open channels, by the cable's numbers.
    open: Vec<u32>,
    /// Messages received and not read yet, with their channel.
    inbox: VecDeque<(u32, PassThruMsg)>,
    /// ISO 15765 and K-line messages still coming in, by channel.
    partial: Vec<(u32, Vec<u8>)>,
    closed: bool,
}

/// Opens the cable on its serial port (`/dev/ttyACM0`). The speed is the USB link's, so any will do.
pub fn open_tty(path: &str) -> Result<OpenPort<FdPort>, ObdError> {
    OpenPort::open(crate::port::open_serial(path, 115_200).map_err(ObdError::Io)?)
}

/// The serial port of the first OpenPort 2.0 plugged in, from `root` (`/sys/bus/usb/devices`).
pub fn find(root: &Path) -> Result<String, String> {
    let found = crate::devices::scan(root);
    let cable = found
        .iter()
        .find(|f| f.vendor == VENDOR && f.product == PRODUCT)
        .ok_or("no Tactrix OpenPort 2.0 is plugged in (icarus devices lists what is)")?;
    cable
        .ports
        .first()
        .map(|p| format!("/dev/{p}"))
        .ok_or_else(|| {
            "the OpenPort 2.0 is plugged in but has no serial port: Linux's cdc_acm driver did not take it".into()
        })
}

impl<P: Port> OpenPort<P> {
    /// Wakes the cable and opens it (PassThruOpen): its firmware version (`ati`), then `ata`. Anything it had kept from
    /// before is dropped.
    pub fn open(port: P) -> Result<Self, ObdError> {
        let mut op = OpenPort {
            port,
            buf: Vec::new(),
            firmware: String::new(),
            open: Vec::new(),
            inbox: VecDeque::new(),
            partial: Vec::new(),
            closed: false,
        };
        op.send(b"\r\n\r\nati\r\n")?;
        let info = op.answer("ari", "ati")?;
        op.firmware = firmware(&info);
        op.command("ata")?;
        op.inbox.clear();
        op.partial.clear();
        Ok(op)
    }

    /// Closes it (PassThruClose, `atz`): its channels and filters end. Dropping it does the same without waiting.
    pub fn close(mut self) -> Result<(), ObdError> {
        self.closed = true;
        self.command("atz")
    }

    fn send(&mut self, data: &[u8]) -> Result<(), ObdError> {
        self.port.send(data).map_err(ObdError::Io)
    }

    /// Sends a command and waits for its `aro`.
    fn command(&mut self, cmd: &str) -> Result<(), ObdError> {
        self.send(format!("{cmd}\r\n").as_bytes())?;
        self.answer("aro", cmd).map(|_| ())
    }

    /// More bytes from the cable, waiting up to `timeout`.
    fn fill(&mut self, timeout: Duration) -> Result<(), ObdError> {
        let mut b = [0u8; 1024];
        let n = self.port.recv(&mut b, timeout).map_err(ObdError::Io)?;
        self.buf.extend_from_slice(&b[..n]);
        Ok(())
    }

    /// The answer to `cmd`: the next text line that starts with `want` (`are` is a refusal). Messages that come meanwhile are
    /// kept for `read`; other lines (a late `aro`) are passed over.
    fn answer(&mut self, want: &str, cmd: &str) -> Result<String, ObdError> {
        let deadline = Instant::now() + ANSWER;
        loop {
            while let Some(p) = next_piece(&mut self.buf) {
                match p {
                    Piece::Packet {
                        channel,
                        kind,
                        body,
                    } => self.take(channel.into(), kind, &body),
                    Piece::Text(t) if t.starts_with("are") => return Err(refusal(cmd, &t)),
                    Piece::Text(t) if t.starts_with(want) => return Ok(t),
                    Piece::Text(_) => {}
                }
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Err(ObdError::Io(std::io::Error::new(
                    std::io::ErrorKind::TimedOut,
                    format!("the {NAME} did not answer {cmd} in time (is it plugged in, and is this its port?)"),
                )));
            }
            self.fill(left)?;
        }
    }

    fn check(&self, channel: u32) -> Result<(), ObdError> {
        if self.open.contains(&channel) {
            Ok(())
        } else {
            Err(ObdError::Adapter(format!(
                "channel {channel} of the {NAME} is not open"
            )))
        }
    }

    /// A packet from the cable: a message, a piece of one, or an indication.
    fn take(&mut self, channel: u32, kind: u8, body: &[u8]) {
        // on CAN the four-byte timestamp comes first; a K-line piece is the bytes alone
        let bytes = if is_can(channel) {
            body.get(4..).unwrap_or_default()
        } else {
            body
        };
        match kind {
            0x00 if channel == 5 => self.deliver(channel, bytes.to_vec()),
            0x00 => {
                let p = self.piece(channel);
                if p.len() + bytes.len() <= LONGEST + 4 {
                    p.extend_from_slice(bytes);
                }
            }
            0x40 | 0x44 => {
                let mut m = self.take_piece(channel);
                if is_can(channel) {
                    m.extend_from_slice(bytes);
                }
                self.deliver(channel, m);
            }
            _ => {}
        }
    }

    fn piece(&mut self, channel: u32) -> &mut Vec<u8> {
        let i = match self.partial.iter().position(|(c, _)| *c == channel) {
            Some(i) => i,
            None => {
                self.partial.push((channel, Vec::new()));
                self.partial.len() - 1
            }
        };
        &mut self.partial[i].1
    }

    fn take_piece(&mut self, channel: u32) -> Vec<u8> {
        match self.partial.iter().position(|(c, _)| *c == channel) {
            Some(i) => self.partial.swap_remove(i).1,
            None => Vec::new(),
        }
    }

    /// A whole message: on CAN its ID and data, on the K-line its bytes.
    fn deliver(&mut self, channel: u32, bytes: Vec<u8>) {
        if !self.open.contains(&channel) {
            return;
        }
        let msg = if is_can(channel) {
            if bytes.len() < 4 {
                return;
            }
            PassThruMsg {
                id: u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]) & 0x1FFF_FFFF,
                data: bytes[4..].to_vec(),
            }
        } else {
            PassThruMsg { id: 0, data: bytes }
        };
        if self.inbox.len() >= INBOX {
            self.inbox.pop_front();
        }
        self.inbox.push_back((channel, msg));
    }
}

impl<P: Port> Drop for OpenPort<P> {
    fn drop(&mut self) {
        if !self.closed {
            let _ = self.port.send(b"atz\r\n");
        }
    }
}

impl<P: Port> PassThru for OpenPort<P> {
    fn version(&mut self) -> Result<Versions, ObdError> {
        Ok(Versions {
            device: NAME.into(),
            firmware: self.firmware.clone(),
            api: "04.04".into(),
        })
    }

    fn connect(&mut self, protocol: J2534Protocol, baud: u32) -> Result<u32, ObdError> {
        let ch = number(protocol);
        // raw CAN sees 11- and 29-bit frames alike; ISO 15765 (11-bit here) and the K-line need no flags
        let flags = if protocol == J2534Protocol::Can {
            CAN_ID_BOTH
        } else {
            0
        };
        self.command(&format!("ato{ch} {flags} {baud} 0"))?;
        if !self.open.contains(&ch) {
            self.open.push(ch);
        }
        Ok(ch)
    }

    fn disconnect(&mut self, channel: u32) -> Result<(), ObdError> {
        self.check(channel)?;
        self.command(&format!("atc{channel}"))?;
        self.open.retain(|c| *c != channel);
        self.inbox.retain(|(c, _)| *c != channel);
        self.take_piece(channel);
        Ok(())
    }

    fn flow_control(&mut self, channel: u32, tx: u32, rx: u32) -> Result<u32, ObdError> {
        self.check(channel)?;
        let cmd = format!(
            "atf{channel} {FLOW_CONTROL_FILTER} {} 4",
            tx_flags(channel, tx.max(rx))
        );
        let mut out = format!("{cmd}\r\n").into_bytes();
        out.extend_from_slice(&[0xFF; 4]); // the mask: the whole ID
        out.extend_from_slice(&rx.to_be_bytes()); // the pattern: answers from rx
        out.extend_from_slice(&tx.to_be_bytes()); // flow control goes to tx
        self.send(&out)?;
        let a = self.answer("arf", &cmd)?;
        a.split_whitespace()
            .nth(1)
            .and_then(|w| w.parse().ok())
            .ok_or_else(|| ObdError::Garbled(format!("{cmd}: {a}")))
    }

    fn write(
        &mut self,
        channel: u32,
        msg: &PassThruMsg,
        _timeout: Duration,
    ) -> Result<(), ObdError> {
        self.check(channel)?;
        if msg.data.is_empty() || msg.data.len() > LONGEST {
            return Err(ObdError::Adapter(format!(
                "a message of {} bytes is not one J2534 sends",
                msg.data.len()
            )));
        }
        let mut body = Vec::with_capacity(4 + msg.data.len());
        if is_can(channel) {
            body.extend_from_slice(&msg.id.to_be_bytes());
        }
        body.extend_from_slice(&msg.data);
        let cmd = format!("att{channel} {} {}", body.len(), tx_flags(channel, msg.id));
        let mut out = format!("{cmd}\r\n").into_bytes();
        out.extend_from_slice(&body);
        self.send(&out)?;
        self.answer("aro", &cmd).map(|_| ())
    }

    fn read(&mut self, channel: u32, timeout: Duration) -> Result<Option<PassThruMsg>, ObdError> {
        self.check(channel)?;
        let deadline = Instant::now() + timeout;
        loop {
            while let Some(p) = next_piece(&mut self.buf) {
                match p {
                    Piece::Packet {
                        channel,
                        kind,
                        body,
                    } => self.take(channel.into(), kind, &body),
                    Piece::Text(t) if t.starts_with("are") => return Err(refusal("a message", &t)),
                    Piece::Text(_) => {}
                }
            }
            if let Some(i) = self.inbox.iter().position(|(c, _)| *c == channel) {
                return Ok(self.inbox.remove(i).map(|(_, m)| m));
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Ok(None);
            }
            self.fill(left)?;
        }
    }

    fn battery_voltage(&mut self) -> Result<Option<f64>, ObdError> {
        let cmd = format!("atr {BATTERY_PIN}");
        self.send(format!("{cmd}\r\n").as_bytes())?;
        let a = self.answer("arr", &cmd)?;
        let w: Vec<&str> = a.split_whitespace().collect();
        match (w.get(1), w.get(2).and_then(|m| m.parse::<u32>().ok())) {
            (Some(pin), Some(mv)) if *pin == BATTERY_PIN.to_string() => {
                Ok(Some(f64::from(mv) / 1000.0))
            }
            _ => Err(ObdError::Garbled(format!("{cmd}: {a}"))),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::link::Link;
    use crate::passthru::PassThruLink;
    use crate::sim::SimCar;
    use std::io;
    use std::sync::{Arc, Mutex};

    /// A packet as the cable frames it.
    fn packet(channel: u8, kind: u8, body: &[u8]) -> Vec<u8> {
        let mut p = vec![b'a', b'r', b'0' + channel, (body.len() + 1) as u8, kind];
        p.extend_from_slice(body);
        p
    }

    fn ts() -> [u8; 4] {
        [0x00, 0x01, 0x86, 0xA0]
    }

    /// A stand-in OpenPort 2.0 in front of the simulated car, answering as the j2534 library expects the cable to: an ISO
    /// 15765 answer comes as kind 0x00 pieces (the first after the CAN ID, at most twelve data bytes each) and a 0x40 end,
    /// after a "sent" indication and, for a long one, a "starting" one. What it sends back is handed over a few bytes at a
    /// time, as a serial line does.
    struct Cable {
        car: SimCar,
        out: VecDeque<u8>,
        sent: Arc<Mutex<Vec<String>>>,
        filters: Vec<u32>,
        millivolts: u32,
    }

    impl Cable {
        fn new(car: SimCar) -> (Cable, Arc<Mutex<Vec<String>>>) {
            let sent = Arc::new(Mutex::new(Vec::new()));
            // left over from an earlier program: dropped when the cable is opened
            let out = b"leftover\r\n".iter().copied().collect();
            (
                Cable {
                    car,
                    out,
                    sent: sent.clone(),
                    filters: Vec::new(),
                    millivolts: 12_600,
                },
                sent,
            )
        }

        fn say(&mut self, line: &str) {
            self.out.extend(format!("{line}\r\n").bytes());
        }

        fn answer(&mut self, id: u32, data: &[u8]) {
            if !self.filters.contains(&id) {
                return;
            }
            let id = id.to_be_bytes();
            if data.len() > 7 {
                self.out
                    .extend(packet(6, 0x80, &[&ts()[..], &id[..]].concat()));
            }
            let mut chunks = data.chunks(12);
            let first = chunks.next().unwrap_or_default();
            self.out
                .extend(packet(6, 0x00, &[&ts()[..], &id[..], first].concat()));
            for c in chunks {
                self.out.extend(packet(6, 0x00, &[&ts()[..], c].concat()));
            }
            self.out.extend(packet(6, 0x40, &ts()));
        }
    }

    impl Port for Cable {
        fn send(&mut self, data: &[u8]) -> io::Result<()> {
            let mut rest = data;
            while let Some(end) = rest.windows(2).position(|w| w == b"\r\n") {
                let line = String::from_utf8_lossy(&rest[..end]).to_string();
                rest = &rest[end + 2..];
                if line.is_empty() {
                    continue;
                }
                self.sent.lock().unwrap().push(line.clone());
                let w: Vec<u32> = line[3.min(line.len())..]
                    .split_whitespace()
                    .filter_map(|x| x.parse().ok())
                    .collect();
                match &line[..3.min(line.len())] {
                    "ati" => self.say("ari main code version : 1.17.4877"),
                    "ata" | "atz" | "atc" => self.say("aro"),
                    "ato" if w == [6, 0, 500_000, 0] => self.say("aro"),
                    "ato" => self.say("are 3"),
                    "atr" if w == [16] => {
                        let mv = self.millivolts;
                        self.say(&format!("arr 16 {mv}"));
                    }
                    "atf" => {
                        let size = w[3] as usize;
                        let rx = u32::from_be_bytes(rest[size..size + 4].try_into().unwrap());
                        rest = &rest[3 * size..];
                        self.filters.push(rx);
                        let n = self.filters.len();
                        self.say(&format!("arf6 {n}"));
                    }
                    "att" => {
                        let size = w[1] as usize;
                        let msg = &rest[..size];
                        rest = &rest[size..];
                        self.say("aro");
                        let id = u32::from_be_bytes(msg[..4].try_into().unwrap());
                        let req = msg[4..].to_vec();
                        self.out
                            .extend(packet(6, 0x10, &[&ts()[..], &msg[..4]].concat()));
                        let answers: Vec<(u32, Vec<u8>)> = if id == 0x7DF {
                            self.car
                                .broadcast_answers(&req)
                                .into_iter()
                                .filter_map(|(h, d)| {
                                    u32::from_str_radix(h, 16).ok().map(|i| (i, d))
                                })
                                .collect()
                        } else {
                            self.car
                                .module_answers(id, &req)
                                .into_iter()
                                .map(|d| (id + 8, d))
                                .collect()
                        };
                        for (from, d) in answers {
                            self.answer(from, &d);
                        }
                    }
                    _ => self.say("are 1"),
                }
            }
            Ok(())
        }

        fn recv(&mut self, buf: &mut [u8], timeout: Duration) -> io::Result<usize> {
            if self.out.is_empty() {
                std::thread::sleep(timeout.min(Duration::from_millis(2)));
                return Ok(0);
            }
            let n = self.out.len().min(5).min(buf.len());
            for (i, b) in self.out.drain(..n).enumerate() {
                buf[i] = b;
            }
            Ok(n)
        }
    }

    #[test]
    fn the_cable_framing_is_read_piece_by_piece() {
        let mut buf = b"noise".to_vec();
        buf.extend(b"aro\r\nari main code version : 1.17\r\n");
        buf.extend(packet(
            6,
            0x00,
            &[1, 2, 3, 4, 0, 0, 7, 0xE8, 0x41, 0x0C, 0x1A, 0xF8],
        ));
        buf.extend(b"ar"); // the start of something still coming
        assert_eq!(next_piece(&mut buf), Some(Piece::Text("aro".into())));
        assert_eq!(
            next_piece(&mut buf).map(|p| match p {
                Piece::Text(t) => firmware(&t),
                p => format!("{p:?}"),
            }),
            Some("1.17".into())
        );
        assert_eq!(
            next_piece(&mut buf),
            Some(Piece::Packet {
                channel: 6,
                kind: 0x00,
                body: vec![1, 2, 3, 4, 0, 0, 7, 0xE8, 0x41, 0x0C, 0x1A, 0xF8],
            })
        );
        assert_eq!(next_piece(&mut buf), None);
        assert_eq!(buf, b"ar", "a half-come piece waits");
        buf.extend(b"r 16 12650\r\n");
        assert_eq!(
            next_piece(&mut buf),
            Some(Piece::Text("arr 16 12650".into()))
        );
        // a packet can hold the bytes of a line end
        let mut buf = packet(5, 0x00, &[0, 0, 0, 0, 0, 0, 0x01, 0x23, b'\r', b'\n']);
        assert!(matches!(
            next_piece(&mut buf),
            Some(Piece::Packet { channel: 5, .. })
        ));
        assert!(buf.is_empty());
    }

    #[test]
    fn obd_and_one_module_over_an_openport() {
        let (cable, sent) = Cable::new(SimCar::new());
        let mut l = PassThruLink::connect(OpenPort::open(cable).unwrap()).unwrap();
        assert_eq!(l.adapter(), "Tactrix OpenPort 2.0 1.17.4877");
        assert_eq!(l.voltage(), Some(12.6));
        // a long answer in three pieces: the VIN
        assert_eq!(
            crate::j1979::vin(&mut l).unwrap().as_deref(),
            Some(crate::sim::VIN)
        );
        // the instrument cluster says "response pending" first: waited out
        let m = l.request_to(0x7C0, 0x7C8, &[0x19, 0x02, 0xFF]).unwrap();
        assert_eq!(m[0].data[0], 0x59, "{m:?}");
        let rpm = l.request(&[0x01, 0x0C]).unwrap();
        assert_eq!(rpm[0].from, "7E8");
        assert_eq!(&rpm[0].data[..2], &[0x41, 0x0C]);
        let log = sent.lock().unwrap().clone();
        assert_eq!(&log[..3], ["ati", "ata", "ato6 0 500000 0"]);
        assert!(log.contains(&"atf6 3 64 4".to_string()), "{log:?}");
        assert!(log.contains(&"att6 6 64".to_string()), "{log:?}");
        assert!(log.contains(&"atr 16".to_string()), "{log:?}");
    }

    #[test]
    fn a_refusal_is_said_with_its_j2534_name() {
        let (cable, _) = Cable::new(SimCar::new());
        let mut op = OpenPort::open(cable).unwrap();
        let e = op.connect(J2534Protocol::Iso15765, 250_000).unwrap_err();
        assert_eq!(
            e.to_string(),
            "the adapter said: the Tactrix OpenPort 2.0 refused ato6 0 250000 0: J2534 error 3, invalid protocol ID"
        );
        assert!(
            op.read(6, Duration::ZERO).is_err(),
            "a channel that did not open is not read"
        );
    }

    #[test]
    fn raw_can_frames_and_what_is_not_an_answer() {
        let (cable, _) = Cable::new(SimCar::new());
        let mut op = OpenPort::open(cable).unwrap();
        op.open.push(5);
        op.open.push(6);
        let id = |i: u32| i.to_be_bytes();
        let mut stream = Vec::new();
        // raw CAN: each 0x00 packet is a frame, 11- or 29-bit
        stream.extend(packet(
            5,
            0x00,
            &[&ts()[..], &id(0x2C4), &[1, 2, 3]].concat(),
        ));
        stream.extend(packet(
            5,
            0x00,
            &[&ts()[..], &id(0x18DA_F110), &[4]].concat(),
        ));
        // ISO 15765: loopback, "sent" and "starting" are passed over; the message is the pieces up to its end
        stream.extend(packet(
            6,
            0x20,
            &[&ts()[..], &id(0x7E0), &[0x01, 0x0C]].concat(),
        ));
        stream.extend(packet(6, 0x60, &ts()));
        stream.extend(packet(6, 0x10, &[&ts()[..], &id(0x7E0)].concat()));
        stream.extend(packet(6, 0x80, &[&ts()[..], &id(0x7E8)].concat()));
        stream.extend(packet(
            6,
            0x00,
            &[&ts()[..], &id(0x7E8), &[0x41, 0x0C]].concat(),
        ));
        stream.extend(packet(6, 0x00, &[&ts()[..], &[0x1A, 0xF8]].concat()));
        stream.extend(packet(6, 0x40, &ts()));
        op.buf = stream;
        let r = |op: &mut OpenPort<Cable>, ch| op.read(ch, Duration::ZERO).unwrap();
        assert_eq!(
            r(&mut op, 6),
            Some(PassThruMsg {
                id: 0x7E8,
                data: vec![0x41, 0x0C, 0x1A, 0xF8]
            })
        );
        assert_eq!(r(&mut op, 6), None, "nothing else on ISO 15765");
        assert_eq!(
            r(&mut op, 5),
            Some(PassThruMsg {
                id: 0x2C4,
                data: vec![1, 2, 3]
            })
        );
        assert_eq!(r(&mut op, 5).unwrap().id, 0x18DA_F110);
        assert_eq!(r(&mut op, 5), None);
    }

    #[test]
    fn a_read_only_vehicle_is_never_changed_over_an_openport() {
        crate::lock::switch_for_test(&[crate::sim::VIN]);
        let (cable, sent) = Cable::new(SimCar::new());
        let mut l = PassThruLink::connect(OpenPort::open(cable).unwrap()).unwrap();
        assert!(matches!(l.request(&[0x04]), Err(ObdError::ReadOnly(_))));
        crate::lock::switch_for_test(&[]);
        assert!(
            !sent.lock().unwrap().iter().any(|c| c.starts_with("att6 5")),
            "the clear request never reached the cable"
        );
    }

    #[test]
    fn the_cable_is_found_by_its_ids() {
        let root = std::env::temp_dir().join(format!("icarus-openport-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        assert!(find(&root)
            .unwrap_err()
            .starts_with("no Tactrix OpenPort 2.0"));
        let d = root.join("1-2");
        std::fs::create_dir_all(&d).unwrap();
        for (f, v) in [
            ("idVendor", "0403"),
            ("idProduct", "cc4d"),
            ("busnum", "1"),
            ("devnum", "4"),
        ] {
            std::fs::write(d.join(f), format!("{v}\n")).unwrap();
        }
        assert!(find(&root).unwrap_err().contains("cdc_acm"));
        std::fs::create_dir_all(root.join("1-2:1.0/tty/ttyACM0")).unwrap();
        assert_eq!(find(&root).unwrap(), "/dev/ttyACM0");
        let _ = std::fs::remove_dir_all(&root);
    }
}
