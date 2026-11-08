//! The ELM327 command language, which almost every OBD adapter speaks (the OBDLink MX+ speaks it on its STN chip, plus STN
//! extras). Text commands end with a carriage return; the adapter answers in hex lines and ends with a `>` prompt.
//!
//! Icarus switches headers on (`ATH1`) so every answer says which module sent it (several answer an OBD request: engine,
//! transmission...), and puts multi-frame CAN messages (ISO 15765-2, "ISO-TP") back together itself.

use crate::port::Port;
use std::fmt;
use std::io;
use std::time::{Duration, Instant};

#[derive(Debug)]
pub enum ObdError {
    Io(io::Error),
    /// No `>` prompt in time: the adapter is busy, asleep or not an ELM327.
    Timeout(String),
    /// The adapter said something went wrong (`?`, `CAN ERROR`, `BUFFER FULL`...).
    Adapter(String),
    /// The adapter could not reach any module (ignition off, or no OBD-II on this vehicle).
    NoConnection(String),
    /// Nothing answered this request (often: a value this vehicle does not offer).
    NoData,
    /// An answer that could not be read.
    Garbled(String),
}

impl fmt::Display for ObdError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            ObdError::Io(e) => write!(f, "adapter link: {e}"),
            ObdError::Timeout(c) => write!(
                f,
                "no answer to {c} in time (is the adapter on and an ELM327?)"
            ),
            ObdError::Adapter(m) => write!(f, "the adapter said: {m}"),
            ObdError::NoConnection(m) => write!(
                f,
                "{m}: is the ignition on (engine off is fine) and the adapter fully in the port?"
            ),
            ObdError::NoData => f.write_str("no module answered (the vehicle may not offer this)"),
            ObdError::Garbled(l) => write!(f, "could not read the answer {l:?}"),
        }
    }
}

impl std::error::Error for ObdError {}

impl From<io::Error> for ObdError {
    fn from(e: io::Error) -> Self {
        ObdError::Io(e)
    }
}

/// The OBD-II signalling the vehicle uses, as the adapter numbers them (`ATDPN`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Protocol {
    J1850Pwm,
    J1850Vpw,
    Iso9141,
    Kwp5Baud,
    KwpFast,
    Can11At500,
    Can29At500,
    Can11At250,
    Can29At250,
    J1939,
    Other(u8),
}

impl Protocol {
    pub fn from_number(n: u8) -> Protocol {
        match n {
            1 => Protocol::J1850Pwm,
            2 => Protocol::J1850Vpw,
            3 => Protocol::Iso9141,
            4 => Protocol::Kwp5Baud,
            5 => Protocol::KwpFast,
            6 => Protocol::Can11At500,
            7 => Protocol::Can29At500,
            8 => Protocol::Can11At250,
            9 => Protocol::Can29At250,
            10 => Protocol::J1939,
            n => Protocol::Other(n),
        }
    }

    pub fn is_can(self) -> bool {
        matches!(
            self,
            Protocol::Can11At500
                | Protocol::Can29At500
                | Protocol::Can11At250
                | Protocol::Can29At250
                | Protocol::J1939
        )
    }

    /// Hex characters of the header in front of each answer line.
    fn header_chars(self) -> usize {
        match self {
            Protocol::Can11At500 | Protocol::Can11At250 => 3,
            Protocol::Can29At500 | Protocol::Can29At250 | Protocol::J1939 => 8,
            _ => 6,
        }
    }

    pub fn describe(self) -> &'static str {
        match self {
            Protocol::J1850Pwm => "SAE J1850 PWM (41.6 kbit/s; older Fords)",
            Protocol::J1850Vpw => "SAE J1850 VPW (10.4 kbit/s; older GM and Chrysler)",
            Protocol::Iso9141 => "ISO 9141-2 (K-line; older Chrysler, European and Asian cars)",
            Protocol::Kwp5Baud => "ISO 14230-4 KWP2000, 5-baud start (K-line)",
            Protocol::KwpFast => "ISO 14230-4 KWP2000, fast start (K-line)",
            Protocol::Can11At500 => {
                "ISO 15765-4 CAN, 11-bit IDs, 500 kbit/s (most vehicles from 2008)"
            }
            Protocol::Can29At500 => "ISO 15765-4 CAN, 29-bit IDs, 500 kbit/s",
            Protocol::Can11At250 => "ISO 15765-4 CAN, 11-bit IDs, 250 kbit/s",
            Protocol::Can29At250 => "ISO 15765-4 CAN, 29-bit IDs, 250 kbit/s",
            Protocol::J1939 => "SAE J1939 CAN (heavy trucks)",
            Protocol::Other(_) => "a protocol Icarus does not name",
        }
    }
}

/// One module's answer: who sent it (the header) and the message (starting with the service byte + 0x40, or 0x7F for a
/// refusal).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Answer {
    pub from: String,
    pub data: Vec<u8>,
}

impl Answer {
    /// What the sending module most likely is (the OBD-II standard's usual addresses).
    pub fn module(&self) -> &'static str {
        let last = self
            .from
            .get(self.from.len().saturating_sub(2)..)
            .unwrap_or("");
        match (self.from.len(), self.from.as_str(), last) {
            (3, "7E8", _) | (8, _, "10") | (6, _, "10") => "engine (ECM/PCM)",
            (3, "7E9", _) | (8, _, "18") | (6, _, "18") => "transmission (TCM)",
            (3, "7EA", _) | (8, _, "28") | (6, _, "28") => "ABS / chassis",
            _ => "another module",
        }
    }

    /// The refusal code when the module said no (0x7F service code).
    pub fn refusal(&self) -> Option<(u8, u8)> {
        (self.data.first() == Some(&0x7F) && self.data.len() >= 3)
            .then(|| (self.data[1], self.data[2]))
    }
}

pub struct Elm {
    port: Box<dyn Port>,
    /// The adapter's own name (`ATI`), e.g. "ELM327 v1.5".
    pub id: String,
    /// An STN chip's name (`STI`) when the adapter has one (OBDLink).
    pub stn: Option<String>,
    pub protocol: Protocol,
    /// The voltage at the OBD port (`ATRV`): the vehicle's battery.
    pub voltage: Option<f64>,
}

const LINE_ERRORS: [&str; 12] = [
    "CAN ERROR",
    "BUS ERROR",
    "BUS BUSY",
    "DATA ERROR",
    "BUFFER FULL",
    "<RX ERROR",
    "FB ERROR",
    "STOPPED",
    "ACT ALERT",
    "LV RESET",
    "ERROR",
    "?",
];

impl Elm {
    /// Resets the adapter, sets it up, and finds the vehicle's protocol (the vehicle's ignition must be on).
    pub fn connect(port: Box<dyn Port>) -> Result<Elm, ObdError> {
        let mut elm = Elm {
            port,
            id: String::new(),
            stn: None,
            protocol: Protocol::Other(0),
            voltage: None,
        };
        let reset = elm.command("ATZ", Duration::from_secs(4))?;
        elm.id = reset
            .iter()
            .rev()
            .find(|l| l.to_uppercase().contains("ELM"))
            .cloned()
            .unwrap_or_else(|| reset.join(" "));
        for c in ["ATE0", "ATL0", "ATS1", "ATH1", "ATSP0"] {
            let r = elm.command(c, Duration::from_secs(2))?;
            if !r.iter().any(|l| l == "OK") {
                return Err(ObdError::Adapter(format!("{c} gave {r:?}")));
            }
        }
        elm.stn = elm
            .command("STI", Duration::from_secs(2))
            .ok()
            .and_then(|r| r.into_iter().find(|l| l.starts_with("STN")));
        elm.voltage = elm
            .command("ATRV", Duration::from_secs(2))
            .ok()
            .and_then(|r| r.first().and_then(|v| v.trim_end_matches('V').parse().ok()));
        // the first request makes the adapter try each protocol until one answers
        elm.protocol = Protocol::Other(0);
        let first = elm.raw_request("0100", Duration::from_secs(15))?;
        check_errors(&first)?;
        let dpn = elm.command("ATDPN", Duration::from_secs(2))?;
        let n = dpn
            .first()
            .map(|s| s.trim_start_matches('A'))
            .and_then(|s| u8::from_str_radix(s, 16).ok());
        elm.protocol = Protocol::from_number(n.ok_or_else(|| ObdError::Garbled(dpn.join(" ")))?);
        Ok(elm)
    }

    /// Sends one command and returns the answer's lines (without the echo, "SEARCHING..." and blank lines).
    pub fn command(&mut self, cmd: &str, timeout: Duration) -> Result<Vec<String>, ObdError> {
        let mut junk = [0u8; 256];
        while self.port.recv(&mut junk, Duration::ZERO)? > 0 {}
        self.port.send(format!("{cmd}\r").as_bytes())?;
        let deadline = Instant::now() + timeout;
        let mut text = Vec::new();
        let mut buf = [0u8; 512];
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Err(ObdError::Timeout(cmd.to_string()));
            }
            let n = self
                .port
                .recv(&mut buf, left.min(Duration::from_millis(200)))?;
            text.extend_from_slice(&buf[..n]);
            if text.contains(&b'>') {
                break;
            }
        }
        let text = String::from_utf8_lossy(&text);
        let squash = |s: &str| {
            s.chars()
                .filter(|c| !c.is_whitespace())
                .collect::<String>()
                .to_uppercase()
        };
        let echo = squash(cmd);
        Ok(text
            .split(['\r', '\n', '>'])
            .map(|l| l.trim().to_string())
            .filter(|l| {
                !l.is_empty()
                    && squash(l) != echo
                    && !l.starts_with("SEARCHING")
                    && !l.starts_with("BUS INIT")
            })
            .collect())
    }

    fn raw_request(&mut self, hex: &str, timeout: Duration) -> Result<Vec<String>, ObdError> {
        self.command(hex, timeout)
    }

    /// Sends an OBD request (e.g. `[0x01, 0x0C]`, engine speed) and returns every module's answer.
    pub fn request(&mut self, data: &[u8]) -> Result<Vec<Answer>, ObdError> {
        let hex: String = data.iter().map(|b| format!("{b:02X}")).collect();
        let lines = self.raw_request(&hex, Duration::from_secs(5))?;
        check_errors(&lines)?;
        parse_lines(&lines, self.protocol)
    }
}

fn check_errors(lines: &[String]) -> Result<(), ObdError> {
    for l in lines {
        let u = l.to_uppercase();
        if u.contains("NO DATA") {
            return Err(ObdError::NoData);
        }
        if u.contains("UNABLE TO CONNECT") {
            return Err(ObdError::NoConnection(
                "the adapter found no vehicle".into(),
            ));
        }
        if LINE_ERRORS
            .iter()
            .any(|e| u == *e || (e.len() > 1 && u.contains(e)))
        {
            return Err(ObdError::Adapter(l.clone()));
        }
    }
    if lines.is_empty() {
        return Err(ObdError::NoData);
    }
    Ok(())
}

/// Lines with headers turned into answers: CAN frames put back together per sender, other protocols one answer per line.
pub fn parse_lines(lines: &[String], protocol: Protocol) -> Result<Vec<Answer>, ObdError> {
    let hc = protocol.header_chars();
    let mut frames: Vec<(String, Vec<u8>)> = Vec::new();
    for l in lines {
        let hex: String = l.chars().filter(|c| !c.is_whitespace()).collect();
        if hex.len() <= hc
            || !hex.chars().all(|c| c.is_ascii_hexdigit())
            || !(hex.len() - hc).is_multiple_of(2)
        {
            return Err(ObdError::Garbled(l.clone()));
        }
        let bytes = (hc..hex.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap())
            .collect();
        frames.push((hex[..hc].to_uppercase(), bytes));
    }
    if !protocol.is_can() {
        // header (priority, target, sender) then data then a checksum byte
        return frames
            .into_iter()
            .map(|(h, b)| {
                if b.len() < 2 {
                    return Err(ObdError::Garbled(h));
                }
                Ok(Answer {
                    from: h,
                    data: b[..b.len() - 1].to_vec(),
                })
            })
            .collect();
    }
    // ISO-TP: single frames (0x0L), first frames (0x1L LL) and consecutive frames (0x2N), per sender
    let mut out: Vec<(Answer, usize)> = Vec::new();
    for (from, b) in frames {
        let Some(&pci) = b.first() else { continue };
        match pci >> 4 {
            0 => {
                let len = (pci & 0x0F) as usize;
                if len == 0 || b.len() < 1 + len {
                    return Err(ObdError::Garbled(format!(
                        "{from} single frame of {len} bytes"
                    )));
                }
                out.push((
                    Answer {
                        from,
                        data: b[1..1 + len].to_vec(),
                    },
                    len,
                ));
            }
            1 => {
                if b.len() < 2 {
                    return Err(ObdError::Garbled(from));
                }
                let len = (((pci & 0x0F) as usize) << 8) | b[1] as usize;
                out.push((
                    Answer {
                        from,
                        data: b[2..].to_vec(),
                    },
                    len,
                ));
            }
            2 => {
                let Some((a, len)) = out
                    .iter_mut()
                    .rev()
                    .find(|(a, len)| a.from == from && a.data.len() < *len)
                else {
                    return Err(ObdError::Garbled(format!(
                        "{from} continued a message that never started"
                    )));
                };
                let take = (*len - a.data.len()).min(b.len() - 1);
                a.data.extend_from_slice(&b[1..1 + take]);
            }
            _ => {} // flow control: the adapter's business
        }
    }
    Ok(out
        .into_iter()
        .map(|(mut a, len)| {
            a.data.truncate(len);
            a
        })
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::SimCar;

    fn lines(s: &[&str]) -> Vec<String> {
        s.iter().map(|l| l.to_string()).collect()
    }

    #[test]
    fn can_answers_from_two_modules_and_a_long_message() {
        let a = parse_lines(
            &lines(&["7E8 06 41 00 BE 3E B8 13", "7E9 06 41 00 98 18 80 11"]),
            Protocol::Can11At500,
        )
        .unwrap();
        assert_eq!(a.len(), 2);
        assert_eq!(
            (a[0].module(), a[1].module()),
            ("engine (ECM/PCM)", "transmission (TCM)")
        );
        assert_eq!(a[0].data, vec![0x41, 0x00, 0xBE, 0x3E, 0xB8, 0x13]);
        // a VIN: 20 bytes in a first frame and two consecutive frames, padding dropped
        let vin = parse_lines(
            &lines(&[
                "7E8 10 14 49 02 01 31 41 42",
                "7E8 21 43 44 45 46 47 48 49",
                "7E8 22 4A 4B 4C 4D 4E 4F 50",
            ]),
            Protocol::Can11At500,
        )
        .unwrap();
        assert_eq!(vin.len(), 1);
        assert_eq!(vin[0].data.len(), 0x14);
        assert_eq!(&vin[0].data[..3], &[0x49, 0x02, 0x01]);
        assert_eq!(vin[0].data.last(), Some(&0x50));
        // no spaces (ATS0) reads the same; 29-bit headers
        let a = parse_lines(&lines(&["18DAF110034110AB"]), Protocol::Can29At500).unwrap();
        assert_eq!(
            (a[0].from.as_str(), a[0].data.clone(), a[0].module()),
            ("18DAF110", vec![0x41, 0x10, 0xAB], "engine (ECM/PCM)")
        );
    }

    #[test]
    fn k_line_answers_drop_the_checksum() {
        let a = parse_lines(&lines(&["48 6B 10 41 0C 1A F8 C2"]), Protocol::Iso9141).unwrap();
        assert_eq!(
            (a[0].from.as_str(), a[0].data.clone()),
            ("486B10", vec![0x41, 0x0C, 0x1A, 0xF8])
        );
    }

    #[test]
    fn garbage_is_an_error_not_a_reading() {
        for bad in [
            &["7E8 06 41"][..],
            &["hello there"],
            &["7E8 21 00 00"],
            &["7E8 6"],
        ] {
            assert!(
                parse_lines(&lines(bad), Protocol::Can11At500).is_err(),
                "{bad:?}"
            );
        }
        assert!(matches!(
            check_errors(&lines(&["NO DATA"])),
            Err(ObdError::NoData)
        ));
        assert!(matches!(
            check_errors(&lines(&["CAN ERROR"])),
            Err(ObdError::Adapter(_))
        ));
        assert!(matches!(
            check_errors(&lines(&["UNABLE TO CONNECT"])),
            Err(ObdError::NoConnection(_))
        ));
    }

    #[test]
    fn connects_to_the_simulated_car() {
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        assert_eq!(elm.protocol, Protocol::Can11At500);
        assert!(elm.id.contains("ELM327"));
        assert_eq!(elm.voltage, Some(12.6));
        let a = elm.request(&[0x01, 0x0C]).unwrap();
        assert_eq!(a[0].data[..2], [0x41, 0x0C]);
        assert!(matches!(elm.request(&[0x01, 0x99]), Err(ObdError::NoData)));
    }
}
