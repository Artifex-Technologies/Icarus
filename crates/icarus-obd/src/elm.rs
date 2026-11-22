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
    /// A request that would change the vehicle, refused because its Read-Only switch is on (`lock`).
    ReadOnly(String),
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
            ObdError::ReadOnly(why) => write!(f, "{why}"),
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
    /// The one module requests go to (request ID, answer ID), or None: OBD's broadcast to every module (7DF, or 18DB33F1
    /// on 29-bit CAN).
    target: Option<(u32, u32)>,
    /// The extended address byte `request_to` uses from now on (`set_ext`: a module behind a gateway, such as Toyota's
    /// body modules behind 750), and the one the adapter is set to (`ATCEA`), which follows it before each request.
    want_ext: Option<u8>,
    ext: Option<u8>,
    /// CAN auto-formatting is off (`ATCAF0`, for watching the bus): it goes back on (`ATCAF1`) before the next request,
    /// which needs the adapter to add and strip ISO-TP's length bytes.
    raw_frames: bool,
    /// The senders left out of the last broadcast's answers, with why (a message that did not add up): the others' answers
    /// were kept (`parse_broadcast`).
    pub dropped: Vec<(String, String)>,
    /// Every sender left out of a broadcast on this connection, as a note, not yet taken (`take_dropped`): each (sender,
    /// why) once, so a loop of broadcasts does not repeat it.
    unseen: Vec<String>,
    seen: std::collections::BTreeSet<(String, String)>,
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
            target: None,
            want_ext: None,
            ext: None,
            raw_frames: false,
            dropped: Vec::new(),
            unseen: Vec::new(),
            seen: Default::default(),
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

    /// The backstop under every write path: a request that would change the vehicle (`lock::changes_vehicle`) is sent
    /// only when its Read-Only switch is off, whichever part of Icarus asks. Reads, and the requests that hand control
    /// back, always go.
    fn read_only_check(&mut self, data: &[u8]) -> Result<(), ObdError> {
        if crate::lock::changes_vehicle(data) {
            crate::lock::writes_allowed(self).map_err(ObdError::ReadOnly)?;
        }
        Ok(())
    }

    /// Sends an OBD request (e.g. `[0x01, 0x0C]`, engine speed) to every module and returns each one's answer. A sender
    /// whose message does not add up is left out and named in `dropped` (and once a connection in `take_dropped`'s notes);
    /// the other senders' answers are kept.
    pub fn request(&mut self, data: &[u8]) -> Result<Vec<Answer>, ObdError> {
        self.read_only_check(data)?;
        self.format_frames()?;
        self.ext_to(None)?;
        self.point_at_everyone()?;
        self.dropped.clear();
        let lines = self.request_lines(data)?;
        let (answers, dropped) = parse_broadcast(&lines, self.protocol)?;
        self.keep_dropped(data, dropped);
        Ok(answers)
    }

    /// Notes the senders a broadcast of `data` left out: in `dropped`, and once each in the notes `take_dropped` gives.
    fn keep_dropped(&mut self, data: &[u8], dropped: Vec<(String, String)>) {
        for (from, why) in &dropped {
            if self.seen.insert((from.clone(), why.clone())) {
                let req: Vec<String> = data.iter().map(|b| format!("{b:02X}")).collect();
                self.unseen.push(format!(
                    "{from}'s answer to {} (asked of every module) was left out: {why}; the other modules' answers \
                     were kept",
                    req.join(" ")
                ));
            }
        }
        self.dropped = dropped;
    }

    /// The notes on senders left out of broadcasts since the last call, each once a connection: whoever shows a
    /// broadcast's answers shows these too, so a module is never lost silently.
    pub fn take_dropped(&mut self) -> Vec<String> {
        std::mem::take(&mut self.unseen)
    }

    /// Points the adapter back at every module (OBD's broadcast) when a request to one module moved it.
    fn point_at_everyone(&mut self) -> Result<(), ObdError> {
        if self.target.is_some() {
            let cmds = header_commands(None, self.reach().can29);
            self.ok_commands(&cmds.iter().map(String::as_str).collect::<Vec<_>>())?;
            self.target = None;
        }
        Ok(())
    }

    /// What this adapter reaches on this vehicle: the ID width of the vehicle's CAN protocol.
    pub fn reach(&self) -> crate::link::Reach {
        crate::link::Reach::of(self.protocol)
    }

    /// Sends a request to one module on CAN: to request ID `tx`, listening only to answer ID `rx`. 11-bit IDs (up to 7FF;
    /// the answer usually tx + 8) on a vehicle whose OBD-II uses 11-bit CAN, 29-bit IDs (18DAxxF1 answered from 18DAF1xx)
    /// on one that uses 29-bit CAN (`header_commands`). Flow control for long answers goes to `tx` too, which modules
    /// outside OBD's own 7E0-7E7 need.
    pub fn request_to(&mut self, tx: u32, rx: u32, data: &[u8]) -> Result<Vec<Answer>, ObdError> {
        self.read_only_check(data)?;
        if tx > 0x1FFF_FFFF || rx > 0x1FFF_FFFF || (tx > 0x7FF) != (rx > 0x7FF) {
            return Err(ObdError::Adapter(format!(
                "{tx:X}/{rx:X} are not both 11-bit or both 29-bit CAN IDs"
            )));
        }
        let (wide, reach) = (tx > 0x7FF, self.reach());
        if (!wide && !reach.can11) || (wide && !reach.can29) {
            return Err(ObdError::Adapter(format!(
                "talking to the module at {tx:X} needs {}-bit CAN; this vehicle uses {}",
                if wide { 29 } else { 11 },
                self.protocol.describe()
            )));
        }
        self.format_frames()?;
        self.ext_to(self.want_ext)?;
        if self.target != Some((tx, rx)) {
            let cmds = header_commands(Some((tx, rx)), wide);
            self.ok_commands(&cmds.iter().map(String::as_str).collect::<Vec<_>>())?;
            self.target = Some((tx, rx));
        }
        match self.ext {
            None => self.send_request(data),
            Some(a) => {
                let lines = self.request_lines(data)?;
                parse_lines_ext(&strip_ext(&lines, a, self.protocol), self.protocol)
            }
        }
    }

    /// Sets the extended address byte for the requests `request_to` sends from now on (None: normal addressing). Kept
    /// until changed; broadcasts always go without one. `uds::at` sets it for one module's requests and puts it back.
    pub fn set_ext(&mut self, ext: Option<u8>) {
        self.want_ext = ext;
    }

    /// Puts the adapter's extended addressing (`ATCEA hh`, or `ATCEA` alone for off) where a request needs it.
    fn ext_to(&mut self, want: Option<u8>) -> Result<(), ObdError> {
        if self.ext != want {
            let cmd = match want {
                Some(a) => format!("ATCEA{a:02X}"),
                None => "ATCEA".to_string(),
            };
            self.ok_commands(&[&cmd])?;
            self.ext = want;
        }
        Ok(())
    }

    /// Watches every frame on the bus (STMA on an STN chip, else ATMA), handing each line to `each` until it returns false or
    /// the adapter stops by itself (its buffer full, for one). Returns how it ended. Nothing is sent onto the bus.
    ///
    /// Settings change for the watch, and auto-formatting is put back afterwards:
    /// - `ATCSM1`, silent monitoring: the adapter neither acknowledges frames nor sends anything while it watches. It is the
    ///   default on ELM327 v1.4 and later and on STN chips (OBDLink), and nothing in Icarus turns it off; it is said again
    ///   so a setting left by another program cannot make the adapter take part in the bus. An adapter that does not know
    ///   the command (an ELM327 before v1.4, many clones) may acknowledge frames while it watches, so Icarus does not watch
    ///   through it.
    /// - `STCMM0`, on an STN chip only: the STN's own CAN monitoring mode, receive-only (1 would make it a normal node that
    ///   acknowledges frames). STMA is the STN's monitor command, and it is not certain that `ATCSM1` alone governs it, so
    ///   both are said, and both must answer OK before anything is watched.
    /// - `ATCAF0`, CAN auto-formatting off: with it on, the adapter reads every frame as an ISO-TP message and cuts any
    ///   whose first byte is 01-07 to that length, so most of the bus's ordinary frames (an engine-speed frame "2C4 03 1A
    ///   F8 00 1C 00 00 00") lost their last bytes. Off, each frame shows whole. It goes back on (`ATCAF1`) when the watch
    ///   ends, and, if that fails, before the next request, since requests need it.
    ///
    /// An OK to `ATCSM1` or `STCMM0` proves only that the adapter accepted the command, not that it obeys it: a clone that
    /// answers OK to everything can still acknowledge frames while it watches.
    ///
    /// Only the adapter refusing `ATCSM1` or `STCMM0` is reported as an adapter that cannot stay silent; a timeout or a lost
    /// link comes back as itself. If the watch went well but `ATCAF1` failed, the watch's result comes back, saying that
    /// auto-formatting goes back on before the next request (it stays pending until it does).
    pub fn monitor(&mut self, each: impl FnMut(&str) -> bool) -> Result<String, ObdError> {
        self.point_at_everyone()?;
        let silent: &[&str] = if self.stn.is_some() {
            &["ATCSM1", "STCMM0"]
        } else {
            &["ATCSM1"]
        };
        self.ok_commands(silent).map_err(|e| match e {
            ObdError::Adapter(why) => ObdError::Adapter(format!(
                "this adapter cannot promise to stay silent on the bus while it watches ({why}); \
                 an ELM327 v1.4 or later, or an OBDLink, can"
            )),
            other => other,
        })?;
        self.raw_frames = true;
        let watched = self
            .ok_commands(&["ATCAF0"])
            .and_then(|()| self.watch(each));
        // put back whatever happened to the watch; on failure `raw_frames` stays set and the next request tries again
        match (watched, self.format_frames()) {
            (Err(e), _) => Err(e),
            (Ok(how), Ok(())) => Ok(how),
            (Ok(how), Err(e)) => Ok(format!(
                "{how}; CAN auto-formatting could not be turned back on yet ({e}), so it goes back on before the next request"
            )),
        }
    }

    /// Turns CAN auto-formatting back on after a watch (`ATCAF1`), when it is off.
    fn format_frames(&mut self) -> Result<(), ObdError> {
        if self.raw_frames {
            self.ok_commands(&["ATCAF1"])?;
            self.raw_frames = false;
        }
        Ok(())
    }

    /// Monitor mode itself: the command, then lines until `each` says stop or the adapter stops by itself. The only byte
    /// sent after the command is the one that ends monitoring, which the adapter does not put on the bus.
    fn watch(&mut self, mut each: impl FnMut(&str) -> bool) -> Result<String, ObdError> {
        let cmd = if self.stn.is_some() { "STMA" } else { "ATMA" };
        let mut junk = [0u8; 256];
        while self.port.recv(&mut junk, Duration::ZERO)? > 0 {}
        self.port.send(format!("{cmd}\r").as_bytes())?;
        let mut buf: Vec<u8> = Vec::new();
        let mut tmp = [0u8; 2048];
        let mut last = String::new();
        loop {
            let n = self.port.recv(&mut tmp, Duration::from_millis(100))?;
            buf.extend_from_slice(&tmp[..n]);
            if n == 0 && !each("") {
                break;
            }
            while let Some(i) = buf.iter().position(|b| *b == b'\r' || *b == b'>') {
                let prompt = buf[i] == b'>';
                let line = String::from_utf8_lossy(&buf[..i]).trim().to_string();
                buf.drain(..=i);
                if prompt {
                    return Ok(format!(
                        "the adapter stopped by itself{}",
                        if last.is_empty() {
                            String::new()
                        } else {
                            format!(" ({last})")
                        }
                    ));
                }
                if line.is_empty() || line.eq_ignore_ascii_case(cmd) {
                    continue;
                }
                last = line.clone();
                if !each(&line) {
                    // any character ends monitoring; then the prompt comes back
                    self.port.send(b"\r")?;
                    let deadline = Instant::now() + Duration::from_secs(3);
                    while Instant::now() < deadline {
                        let n = self.port.recv(&mut tmp, Duration::from_millis(100))?;
                        if tmp[..n].contains(&b'>') {
                            break;
                        }
                    }
                    return Ok("stopped".into());
                }
            }
        }
        self.port.send(b"\r")?;
        let deadline = Instant::now() + Duration::from_secs(3);
        while Instant::now() < deadline {
            let n = self.port.recv(&mut tmp, Duration::from_millis(100))?;
            if tmp[..n].contains(&b'>') {
                break;
            }
        }
        Ok("stopped".into())
    }

    fn ok_commands(&mut self, cmds: &[&str]) -> Result<(), ObdError> {
        for c in cmds {
            let r = self.command(c, Duration::from_secs(2))?;
            if !r.iter().any(|l| l == "OK") {
                return Err(ObdError::Adapter(format!("{c} gave {r:?}")));
            }
        }
        Ok(())
    }

    fn send_request(&mut self, data: &[u8]) -> Result<Vec<Answer>, ObdError> {
        let lines = self.request_lines(data)?;
        parse_lines(&lines, self.protocol)
    }

    fn request_lines(&mut self, data: &[u8]) -> Result<Vec<String>, ObdError> {
        let hex: String = data.iter().map(|b| format!("{b:02X}")).collect();
        let lines = self.raw_request(&hex, Duration::from_secs(5))?;
        check_errors(&lines)?;
        Ok(lines)
    }
}

/// The commands that point the adapter at one module, `Some((tx, rx))`, or back at every module (OBD's broadcast), for
/// 11-bit or 29-bit (`can29`) CAN. One module: the header requests go out under (`ATSH`), the only answer ID shown
/// (`ATCRA`), and flow control for long answers sent to `tx` (`ATFCSH`, `ATFCSD300000`: go on, no pause; `ATFCSM1`: these
/// settings, not the adapter's own guess). On 29-bit CAN, `ATSH` takes the header's lower three bytes and `ATCP` its top
/// one, the priority (18 in 18DAxxF1); the broadcast is 18DB33F1.
pub fn header_commands(target: Option<(u32, u32)>, can29: bool) -> Vec<String> {
    let v = |s: &[&str]| s.iter().map(|c| c.to_string()).collect::<Vec<_>>();
    match (target, can29) {
        (None, false) => v(&["ATSH7DF", "ATCRA", "ATFCSM0"]),
        (None, true) => v(&["ATCP18", "ATSHDB33F1", "ATCRA", "ATFCSM0"]),
        (Some((tx, rx)), false) => vec![
            format!("ATSH{tx:03X}"),
            format!("ATCRA{rx:03X}"),
            format!("ATFCSH{tx:03X}"),
            "ATFCSD300000".into(),
            "ATFCSM1".into(),
        ],
        (Some((tx, rx)), true) => vec![
            format!("ATCP{:02X}", (tx >> 24) & 0x1F),
            format!("ATSH{:06X}", tx & 0xFF_FFFF),
            format!("ATCRA{rx:08X}"),
            format!("ATFCSH{tx:08X}"),
            "ATFCSD300000".into(),
            "ATFCSM1".into(),
        ],
    }
}

/// Answers from a module behind a gateway carry its address byte first in each frame (Toyota: the same byte the request
/// carried, `758 2A 03 62 ...`). Some adapters strip it, some show it: which, is read from the first frame, whose next byte
/// must be a single or first frame's PCI (0x0_ or 0x1_), and the byte is taken off every frame that has it then.
pub fn strip_ext(lines: &[String], addr: u8, protocol: Protocol) -> Vec<String> {
    let hc = protocol.header_chars();
    let a = format!("{addr:02X}");
    let compact: Vec<String> = lines
        .iter()
        .map(|l| l.chars().filter(|c| !c.is_whitespace()).collect())
        .collect();
    let shown = compact.iter().find(|h| h.len() > hc + 2).is_some_and(|h| {
        h[hc..].starts_with(&a) && matches!(h.as_bytes().get(hc + 2), Some(b'0' | b'1'))
    });
    if !shown {
        return lines.to_vec();
    }
    compact
        .into_iter()
        .map(|h| {
            if h.len() > hc + 2 && h[hc..].starts_with(&a) {
                format!("{}{}", &h[..hc], &h[hc + 2..])
            } else {
                h
            }
        })
        .collect()
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
/// Any message that does not add up is an error: a request to one module reads live values from these bytes.
pub fn parse_lines(lines: &[String], protocol: Protocol) -> Result<Vec<Answer>, ObdError> {
    let (answers, bad) = parse_each(lines, protocol)?;
    match bad.into_iter().next() {
        Some((_, why)) => Err(ObdError::Garbled(why)),
        None => Ok(answers),
    }
}

/// Every sender's answers, and (sender, why) for each sender left out because its message did not add up.
pub type Parsed = (Vec<Answer>, Vec<(String, String)>);

/// `parse_lines` for a broadcast (7DF), which many modules answer: a sender whose message does not add up is left out and
/// named, with why, in the second part, and every other sender's answer is kept (one module's lost frame does not lose the
/// engine's VIN). An error only when a line cannot be read at all, or no sender's answer adds up.
pub fn parse_broadcast(lines: &[String], protocol: Protocol) -> Result<Parsed, ObdError> {
    let (answers, bad) = parse_each(lines, protocol)?;
    if answers.is_empty() {
        if let Some((_, why)) = bad.first() {
            return Err(ObdError::Garbled(why.clone()));
        }
    }
    Ok((answers, bad))
}

/// Every sender's answers, and (sender, why) for each sender whose message did not add up (its answers left out, in the
/// order the problems were found). Err only for a line that is not a frame at all.
fn parse_each(lines: &[String], protocol: Protocol) -> Result<Parsed, ObdError> {
    parse_sized(lines, protocol, 8)
}

/// `parse_lines` for a module behind a gateway (extended addressing, after `strip_ext`): each frame carries 7 bytes of
/// ISO-TP, its address byte having taken the eighth.
pub fn parse_lines_ext(lines: &[String], protocol: Protocol) -> Result<Vec<Answer>, ObdError> {
    let (answers, bad) = parse_sized(lines, protocol, 7)?;
    match bad.into_iter().next() {
        Some((_, why)) => Err(ObdError::Garbled(why)),
        None => Ok(answers),
    }
}

/// `parse_each` for frames of `frame` ISO-TP bytes (8 on classic CAN, 7 behind a gateway).
fn parse_sized(lines: &[String], protocol: Protocol, frame: usize) -> Result<Parsed, ObdError> {
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
    let mut bad: Vec<(String, String)> = Vec::new();
    if !protocol.is_can() {
        // header (priority, target, sender) then data then a checksum byte
        let mut out = Vec::new();
        for (h, b) in frames {
            if b.len() < 2 {
                bad.push((h.clone(), h));
            } else {
                out.push(Answer {
                    from: h,
                    data: b[..b.len() - 1].to_vec(),
                });
            }
        }
        let gone: Vec<String> = bad.iter().map(|b| b.0.clone()).collect();
        out.retain(|a| !gone.contains(&a.from));
        return Ok((out, bad));
    }
    // ISO-TP: single frames (0x0L), first frames (0x1L LL) and consecutive frames (0x2N), per sender. A message that does
    // not add up (a frame lost or repeated, one missing at the end, lengths classic CAN cannot carry) is an error for that
    // sender, never a short or wrong answer: live values are read from these bytes.
    struct Message {
        a: Answer,
        len: usize,
        /// The sequence number the next consecutive frame must carry: 1 after the first frame, then 2..15, 0, 1...
        next: u8,
    }
    let mut out: Vec<Message> = Vec::new();
    for (from, b) in frames {
        if bad.iter().any(|x| x.0 == from) {
            continue; // the rest of a sender whose message already did not add up
        }
        let Some(&pci) = b.first() else { continue };
        let open = |out: &Vec<Message>| {
            out.iter()
                .any(|m| m.a.from == from && m.a.data.len() < m.len)
        };
        let problem = match pci >> 4 {
            0 => {
                let len = (pci & 0x0F) as usize;
                if len == 0 || len >= frame || b.len() < 1 + len {
                    Some(format!("{from} single frame of {len} bytes"))
                } else {
                    out.push(Message {
                        a: Answer {
                            from: from.clone(),
                            data: b[1..1 + len].to_vec(),
                        },
                        len,
                        next: 0,
                    });
                    None
                }
            }
            1 => {
                let len = if b.len() >= 2 {
                    (((pci & 0x0F) as usize) << 8) | b[1] as usize
                } else {
                    0
                };
                if b.len() < frame {
                    Some(format!(
                        "{from} first frame without its {} data bytes",
                        frame - 2
                    ))
                } else if len == 0 {
                    Some(format!("{from} message longer than classic ISO-TP carries"))
                } else if len < frame {
                    Some(format!(
                        "{from} first frame of a {len}-byte message (that is a single frame's)"
                    ))
                } else if open(&out) {
                    Some(format!(
                        "{from} started a new message before the last one ended"
                    ))
                } else {
                    out.push(Message {
                        a: Answer {
                            from: from.clone(),
                            data: b[2..frame].to_vec(),
                        },
                        len,
                        next: 1,
                    });
                    None
                }
            }
            2 => match out
                .iter_mut()
                .rev()
                .find(|m| m.a.from == from && m.a.data.len() < m.len)
            {
                None => Some(format!("{from} continued a message that never started")),
                Some(m) => {
                    let seq = pci & 0x0F;
                    if seq != m.next {
                        Some(format!(
                            "{from} frame {seq} came after frame {}: a frame was lost or repeated",
                            (m.next + 15) & 0x0F
                        ))
                    } else {
                        m.next = (m.next + 1) & 0x0F;
                        let take = (m.len - m.a.data.len()).min(b.len() - 1);
                        m.a.data.extend_from_slice(&b[1..1 + take]);
                        None
                    }
                }
            },
            _ => None, // flow control: the adapter's business
        };
        if let Some(why) = problem {
            out.retain(|m| m.a.from != from);
            bad.push((from, why));
        }
    }
    let unfinished: Vec<(String, String)> = out
        .iter()
        .filter(|m| m.a.data.len() < m.len)
        .map(|m| {
            (
                m.a.from.clone(),
                format!(
                    "{} message of {} bytes ended after {}",
                    m.a.from,
                    m.len,
                    m.a.data.len()
                ),
            )
        })
        .collect();
    for (from, why) in unfinished {
        if !bad.iter().any(|b| b.0 == from) {
            out.retain(|m| m.a.from != from);
            bad.push((from, why));
        }
    }
    Ok((
        out.into_iter()
            .map(|mut m| {
                m.a.data.truncate(m.len);
                m.a
            })
            .collect(),
        bad,
    ))
}

#[cfg(test)]
mod tests {

    #[test]
    fn address_bytes_behind_a_gateway() {
        let l = |v: &[&str]| v.iter().map(|x| x.to_string()).collect::<Vec<_>>();
        // shown by the adapter: taken off every frame that has it
        let shown = l(&["758 2A 10 0B 62 F1 87 38 39", "758 2A 21 32 32 31 2D 49 43"]);
        assert_eq!(
            strip_ext(&shown, 0x2A, Protocol::Can11At500),
            l(&["758100B62F1873839", "758213232312D4943"])
        );
        // stripped by the adapter: a first frame's PCI comes first, and a consecutive frame 2A is not mistaken for it
        let stripped = l(&["758 10 0B 62 F1 87 38 39 32", "758 2A 32 31 2D 49 43"]);
        assert_eq!(strip_ext(&stripped, 0x2A, Protocol::Can11At500), stripped);
        let a = parse_lines_ext(
            &strip_ext(&shown, 0x2A, Protocol::Can11At500),
            Protocol::Can11At500,
        )
        .unwrap();
        assert_eq!(
            a[0].data,
            [0x62, 0xF1, 0x87, 0x38, 0x39, 0x32, 0x32, 0x31, 0x2D, 0x49, 0x43]
        );
    }
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

    /// CAN lines from 7E8 carrying `payload` as ISO-TP frames (sequence numbers as given by `seq`, or the right ones).
    fn isotp(payload: &[u8], seq: Option<&[u8]>) -> Vec<String> {
        let hex = |b: &[u8]| {
            b.iter()
                .map(|x| format!("{x:02X}"))
                .collect::<Vec<_>>()
                .join(" ")
        };
        let mut out = vec![format!(
            "7E8 {:02X} {:02X} {}",
            0x10 | (payload.len() >> 8) as u8,
            payload.len() as u8,
            hex(&payload[..6])
        )];
        for (i, c) in payload[6..].chunks(7).enumerate() {
            let n = seq.map_or(((i + 1) & 0x0F) as u8, |s| s[i]);
            let mut f = c.to_vec();
            f.resize(7, 0xAA);
            out.push(format!("7E8 {:02X} {}", 0x20 | n, hex(&f)));
        }
        out
    }

    #[test]
    fn long_answers_come_back_whole_or_not_at_all() {
        let p40: Vec<u8> = (0..40).collect();
        let a = parse_lines(&isotp(&p40, None), Protocol::Can11At500).unwrap();
        assert_eq!(a[0].data, p40, "40 bytes over 6 frames, padding dropped");
        let p120: Vec<u8> = (0..120).collect();
        let lines = isotp(&p120, None);
        assert!(lines.len() > 16, "the sequence wraps past 15");
        assert_eq!(
            parse_lines(&lines, Protocol::Can11At500).unwrap()[0].data,
            p120
        );
        let err = |l: Vec<String>| {
            parse_lines(&l, Protocol::Can11At500)
                .unwrap_err()
                .to_string()
        };
        // a lost frame, a repeated one, the last one missing
        let mut lost = isotp(&p40, None);
        lost.remove(2);
        assert!(err(lost).contains("frame was lost"));
        assert!(err(isotp(&p40, Some(&[1, 1, 2, 3, 4]))).contains("lost or repeated"));
        let mut short = isotp(&p40, None);
        short.pop();
        assert!(err(short).contains("message of 40 bytes ended after 34"));
        // lengths classic CAN cannot carry
        assert!(err(lines_of(&["7E8 10 00 62 1F 01 00 00 00"])).contains("longer than classic"));
        assert!(err(lines_of(&["7E8 10 05 62 1F 01 00 00 00"])).contains("single frame's"));
        assert!(err(lines_of(&["7E8 09 62 1F 01 00 00 00 00"])).contains("single frame of 9"));
        assert!(err(lines_of(&["7E8 10 14 49 02"])).contains("6 data bytes"));
        // two first frames from one sender
        let mut two = isotp(&p40, None);
        two.insert(2, two[0].clone());
        assert!(err(two).contains("before the last one ended"));
        // two senders interleaved are still fine
        let mut both = isotp(&p40, None);
        let other: Vec<String> = both.iter().map(|l| l.replacen("7E8", "7E9", 1)).collect();
        both = both
            .into_iter()
            .zip(other)
            .flat_map(|(a, b)| [a, b])
            .collect();
        let a = parse_lines(&both, Protocol::Can11At500).unwrap();
        assert_eq!((a.len(), &a[0].data, &a[1].data), (2, &p40, &p40));
    }

    fn lines_of(s: &[&str]) -> Vec<String> {
        lines(s)
    }

    #[test]
    fn a_broadcast_drops_only_the_sender_whose_message_does_not_add_up() {
        // 7E8's VIN whole, 7E9's missing a consecutive frame (a cheap adapter dropped it)
        let vin: Vec<u8> = [0x49, 0x02, 0x01]
            .into_iter()
            .chain(b"1ABCDEFGHJKLMNPRS".iter().copied())
            .collect();
        let good = isotp(&vin, None);
        let mut broken: Vec<String> = good.iter().map(|l| l.replacen("7E8", "7E9", 1)).collect();
        broken.remove(1);
        // interleaved as they come off the bus: 7E8's first frame, 7E9's, then the rest
        let mut window = vec![good[0].clone(), broken[0].clone()];
        window.extend(good[1..].iter().cloned());
        window.extend(broken[1..].iter().cloned());
        window.push("7EA 03 41 0D 00".into());
        // asked of one module, any message that does not add up is an error
        assert!(parse_lines(&window, Protocol::Can11At500).is_err());
        // a broadcast keeps the others' answers, and names the one left out
        let (answers, dropped) = parse_broadcast(&window, Protocol::Can11At500).unwrap();
        let from: Vec<&str> = answers.iter().map(|a| a.from.as_str()).collect();
        assert_eq!(from, vec!["7E8", "7EA"]);
        assert_eq!(answers[0].data, vin);
        assert_eq!(dropped.len(), 1);
        assert_eq!(dropped[0].0, "7E9");
        assert!(dropped[0].1.contains("frame was lost"), "{dropped:?}");
        // a restarted message, and one that ends early, are left out the same way
        let mut restart = isotp(&vin, None);
        restart.insert(1, "7E9 10 14 49 02 01 31 41 42".into());
        restart.insert(2, "7E9 10 14 49 02 01 31 41 42".into());
        let (a, d) = parse_broadcast(&restart, Protocol::Can11At500).unwrap();
        assert_eq!((a.len(), d[0].0.as_str()), (1, "7E9"));
        assert!(d[0].1.contains("before the last one ended"));
        let mut early = isotp(&vin, None);
        early.push("7E9 10 14 49 02 01 31 41 42".into());
        let (a, d) = parse_broadcast(&early, Protocol::Can11At500).unwrap();
        assert_eq!(
            (a.len(), d[0].1.as_str()),
            (1, "7E9 message of 20 bytes ended after 6")
        );
        // nothing whole at all is still an error, and a line that is not a frame fails the window
        assert!(parse_broadcast(&lines_of(&["7E9 21 00 00"]), Protocol::Can11At500).is_err());
        assert!(parse_broadcast(
            &lines_of(&["7E8 06 41 00 BE 3E B8 13", "hello"]),
            Protocol::Can11At500
        )
        .is_err());
    }

    #[test]
    fn a_sender_left_out_is_noted_once_a_connection() {
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        assert!(
            elm.take_dropped().is_empty(),
            "a whole broadcast drops no one"
        );
        let lost = || vec![("7E9".to_string(), "a frame was lost".to_string())];
        elm.keep_dropped(&[0x01, 0x0C], lost());
        // a loop of broadcasts (live values): the same sender for the same reason is noted once
        elm.keep_dropped(&[0x01, 0x0C], lost());
        assert_eq!(elm.dropped, lost(), "the last broadcast's");
        let notes = elm.take_dropped();
        assert_eq!(notes.len(), 1, "{notes:?}");
        assert!(
            notes[0].starts_with("7E9's answer to 01 0C") && notes[0].contains("a frame was lost"),
            "{}",
            notes[0]
        );
        assert!(elm.take_dropped().is_empty(), "taken");
        elm.keep_dropped(&[0x01, 0x0C], lost());
        assert!(elm.take_dropped().is_empty(), "noted already");
        // another reason is another note; a whole broadcast clears `dropped` but not the notes
        elm.keep_dropped(&[0x09, 0x02], vec![("7E9".into(), "ended early".into())]);
        elm.request(&[0x01, 0x0C]).unwrap();
        assert!(elm.dropped.is_empty());
        assert_eq!(elm.take_dropped().len(), 1);
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
