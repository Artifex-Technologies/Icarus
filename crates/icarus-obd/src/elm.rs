//! The ELM327 command language, which almost every OBD adapter speaks (the OBDLink MX+ speaks it on its STN chip, plus STN
//! extras). Text commands end with a carriage return; the adapter answers in hex lines and ends with a `>` prompt.
//!
//! Icarus switches headers on (`ATH1`) so every answer says which module sent it (several answer an OBD request: engine,
//! transmission...), and puts multi-frame CAN messages (ISO 15765-2, "ISO-TP") back together itself.

use crate::link::Bus;
use crate::port::Port;
use std::fmt;
use std::io;
use std::time::{Duration, Instant};

/// Serial speeds an ELM327 or STN adapter is found at, most likely first: OBDLink's USB cables (EX, SX) at 115200, the
/// ELM327's own 38400, then the faster and slower ones some clones use.
pub const SERIAL_SPEEDS: [u32; 5] = [115_200, 38_400, 500_000, 230_400, 9_600];

/// Opens the adapter `target` names and speaks to it (`Elm::connect`). A serial adapter that does not answer, or answers
/// nonsense, at its speed is tried at the other usual ones (`SERIAL_SPEEDS`): only the adapter is spoken to, never the
/// vehicle. The note says which speed answered when it was not the one given (an OBDLink EX at 115200 behind a `/dev/ttyUSB0`
/// that meant 38400).
pub fn connect_target(target: &crate::port::Target) -> Result<(Elm, Option<String>), String> {
    use crate::port::Target;
    match target {
        Target::Serial { path, baud } => connect_serial(path, *baud, &mut |b| {
            Target::Serial {
                path: path.clone(),
                baud: b,
            }
            .open()
        }),
        t => {
            let port = t.open().map_err(|e| e.to_string())?;
            Elm::connect(port)
                .map(|e| (e, None))
                .map_err(|e| e.to_string())
        }
    }
}

/// Opens the adapter `target` names to listen only (ICA-80): `Elm::listen` on `protocol`, with `connect_target`'s search for
/// a serial adapter's speed (which speaks only to the adapter). Nothing goes onto the bus.
pub fn listen_target(
    target: &crate::port::Target,
    protocol: u8,
) -> Result<(Elm, Option<String>), String> {
    use crate::port::Target;
    let how = |p: Box<dyn Port>| Elm::listen(p, protocol);
    match target {
        Target::Serial { path, baud } => connect_serial_with(
            path,
            *baud,
            &mut |b| {
                Target::Serial {
                    path: path.clone(),
                    baud: b,
                }
                .open()
            },
            &how,
        ),
        Target::Can(_) | Target::Slcan { .. } => {
            Err("a CAN interface or an SLCAN adapter is watched with icarus bus watch, which is listen-only already".into())
        }
        t => {
            let port = t.open().map_err(|e| e.to_string())?;
            how(port).map(|e| (e, None)).map_err(|e| e.to_string())
        }
    }
}

/// `connect_target` for a serial port, opened at a speed by `open` (a missing port or no permission is said at once).
fn connect_serial(
    path: &str,
    first: u32,
    open: &mut dyn FnMut(u32) -> io::Result<Box<dyn Port>>,
) -> Result<(Elm, Option<String>), String> {
    connect_serial_with(path, first, open, &Elm::connect)
}

/// `connect_serial` with the adapter set up by `how` (`Elm::connect`, or `Elm::listen` for a listener).
fn connect_serial_with(
    path: &str,
    first: u32,
    open: &mut dyn FnMut(u32) -> io::Result<Box<dyn Port>>,
    how: &dyn Fn(Box<dyn Port>) -> Result<Elm, ObdError>,
) -> Result<(Elm, Option<String>), String> {
    let mut tried: Vec<u32> = vec![];
    let mut first_err: Option<ObdError> = None;
    for baud in std::iter::once(first).chain(SERIAL_SPEEDS.into_iter().filter(|b| *b != first)) {
        let port = open(baud).map_err(|e| format!("{path}: {e}"))?;
        match how(port) {
            Ok(elm) => {
                let note = (baud != first).then(|| {
                    format!("{path} answered at {baud} baud, not {first} ({path}@{baud} names it)")
                });
                return Ok((elm, note));
            }
            Err(e @ (ObdError::Timeout(_) | ObdError::Adapter(_))) => {
                tried.push(baud);
                first_err.get_or_insert(e);
            }
            Err(e) => return Err(e.to_string()),
        }
    }
    let speeds: Vec<String> = tried.iter().map(|b| b.to_string()).collect();
    Err(format!(
        "{}; {path} was tried at {} baud. An OBDLink EX or SX takes its power from the vehicle: it answers only when plugged \
         into the OBD port",
        first_err.map(|e| e.to_string()).unwrap_or_default(),
        speeds.join(", ")
    ))
}

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

/// An ELM327 or STN adapter, set up and on the vehicle's protocol. Requests go onto the bus only through `request` and
/// `request_to`, which make the Read-Only switch's check first (`read_only_check`); the adapter's own command line, which
/// sends a command of hex digits onto the bus as it is, is not handed out:
///
/// ```
/// use icarus_obd::elm::Elm;
/// use icarus_obd::sim::SimCar;
/// let mut elm = Elm::connect(Box::new(SimCar::engine_off())).unwrap();
/// assert!(elm.request(&[0x01, 0x00]).is_ok());
/// ```
///
/// Saying a command to the adapter from outside this module does not build:
///
/// ```compile_fail,E0624
/// use icarus_obd::elm::Elm;
/// use icarus_obd::sim::SimCar;
/// let mut elm = Elm::connect(Box::new(SimCar::engine_off())).unwrap();
/// elm.command("2EA00100", std::time::Duration::from_secs(1)).unwrap();
/// ```
pub struct Elm {
    port: Box<dyn Port>,
    /// The adapter's own name (`ATI`), e.g. "ELM327 v1.5".
    pub id: String,
    /// An STN chip's name (`STI`) when the adapter has one (OBDLink).
    pub stn: Option<String>,
    pub protocol: Protocol,
    /// The voltage at the OBD port (`ATRV`, the vehicle's battery) as read when the adapter connected, for showing. A
    /// write judges the battery measured now (`measure_voltage`), never this.
    pub voltage: Option<f64>,
    /// The one module requests go to (request ID, answer ID), or None: OBD's broadcast to every module (7DF, or 18DB33F1
    /// on 29-bit CAN).
    target: Option<(u32, u32)>,
    /// The extended address byte `request_to` uses from now on (`set_ext`: a module behind a gateway, such as Toyota's
    /// body modules behind 750), and the one the adapter is set to (`ATCEA`), which follows it before each request (None:
    /// not known, after a switch of bus, so it is said again).
    want_ext: Option<u8>,
    ext: Option<Option<u8>>,
    /// The bus `request_to` uses from now on (`set_bus`), and the one the adapter is on (None: not known, after a switch
    /// that failed), which follows it before each request; broadcasts and watching always go on the main bus.
    want_bus: Bus,
    bus: Option<Bus>,
    /// Why this adapter cannot switch to the medium-speed CAN, once it refused to.
    no_ms_can: Option<String>,
    /// Why this adapter cannot open its receive filter wide for a sweep's probe, once it refused to or reported trouble
    /// under it (`probe`, which then listens to the guessed answer ID alone for the rest of the connection).
    no_wide: Option<String>,
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
    /// The requests this adapter sent onto the bus lately (ICA-80: shown beside what a listening adapter saw): (when, the
    /// request ID, the request's bytes), the last `SENT_KEEP`.
    sent: std::collections::VecDeque<(Instant, u32, Vec<u8>)>,
    /// What each request to one module brought back, not yet taken (`take_outcomes`, ICA-70: the vehicle's record).
    outcomes: Vec<crate::record::Outcome>,
    /// Reads this vehicle refused for good on an earlier connection (`remember`, ICA-71): (request ID, address byte,
    /// request) -> the refusal's code. Not sent again: the refusal comes back as before.
    refused: std::collections::HashMap<crate::record::Asked, u8>,
    /// How many requests the record spared on this connection.
    pub spared: usize,
}

/// How many sent requests an adapter keeps for `take_sent`.
const SENT_KEEP: usize = 256;

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
        let mut elm = Elm::setup(port, "ATSP0")?;
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

    /// Sets the adapter up to listen only (ICA-80): reset, its formatting and the protocol given (an ELM327 protocol number:
    /// 6 is ISO 15765-4 CAN, 11-bit, 500 kbit/s, what most vehicles since 2008 use), and nothing else. No request goes onto
    /// the bus, not even the protocol search `connect` makes with its first request. Watch with `monitor` or `monitor_ids`,
    /// which make the adapter silent first (it acknowledges nothing).
    pub fn listen(port: Box<dyn Port>, protocol: u8) -> Result<Elm, ObdError> {
        if !(1..=0xC).contains(&protocol) {
            return Err(ObdError::Adapter(format!(
                "no protocol {protocol:X} to listen on"
            )));
        }
        let mut elm = Elm::setup(port, &format!("ATSP{protocol:X}"))?;
        elm.protocol = Protocol::from_number(protocol);
        Ok(elm)
    }

    /// The requests sent onto the bus since the last call (ICA-80): (when, request ID, bytes), oldest first.
    pub fn take_sent(&mut self) -> Vec<(Instant, u32, Vec<u8>)> {
        self.sent.drain(..).collect()
    }

    /// Reset and setup: echo, line feeds and spaces as Icarus reads them, headers on, the protocol command `sp`; then the
    /// adapter's chip (STI) and the battery's voltage. Only the adapter is spoken to.
    fn setup(port: Box<dyn Port>, sp: &str) -> Result<Elm, ObdError> {
        let mut elm = Elm {
            port,
            id: String::new(),
            stn: None,
            protocol: Protocol::Other(0),
            voltage: None,
            target: None,
            want_ext: None,
            ext: Some(None),
            want_bus: Bus::Main,
            bus: Some(Bus::Main),
            no_ms_can: None,
            no_wide: None,
            raw_frames: false,
            dropped: Vec::new(),
            unseen: Vec::new(),
            seen: Default::default(),
            sent: Default::default(),
            outcomes: Vec::new(),
            refused: Default::default(),
            spared: 0,
        };
        let reset = elm.command("ATZ", Duration::from_secs(4))?;
        elm.id = reset
            .iter()
            .rev()
            .find(|l| l.to_uppercase().contains("ELM"))
            .cloned()
            .unwrap_or_else(|| reset.join(" "));
        for c in ["ATE0", "ATL0", "ATS1", "ATH1", sp] {
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
        Ok(elm)
    }

    /// Whatever is left from before (a late answer) goes before a command, for half a second at most: an adapter still in
    /// monitor mode streams until it is sent a character, and a port that never went quiet held Icarus up for good.
    fn drain_left_over(&mut self) -> Result<(), ObdError> {
        let mut junk = [0u8; 256];
        let quiet_by = Instant::now() + Duration::from_millis(500);
        while self.port.recv(&mut junk, Duration::ZERO)? > 0 && Instant::now() < quiet_by {}
        Ok(())
    }

    /// The voltage at the OBD port measured now (`ATRV`: the adapter answers, nothing goes onto the bus); None when the
    /// adapter does not answer with a number this time (it timed out, said `?`, or a line left from a bus watch came
    /// first). Never the reading from when it connected (`voltage`, for showing): the write paths judge the battery by
    /// this, and an hour-old 12.6 V says nothing about the battery now.
    pub fn measure_voltage(&mut self) -> Option<f64> {
        self.command("ATRV", Duration::from_secs(2))
            .ok()
            .and_then(|r| {
                r.first()
                    .and_then(|v| v.trim().trim_end_matches('V').parse::<f64>().ok())
            })
    }

    /// Whether the port this adapter was opened on is a cable all the way (`Port::wired`). Judged by the open port
    /// itself, so a symlink moved after connecting (udev giving `/dev/obdlink` to another adapter) does not change it.
    pub fn wired(&self) -> bool {
        self.port.wired()
    }

    /// Sends one command and returns the answer's lines (without the echo, "SEARCHING..." and blank lines). Private: a
    /// command of hex digits, or an STN's STPX, goes onto the bus as it is, so only this module's own requests say one,
    /// after the Read-Only switch's check (`read_only_check`).
    fn command(&mut self, cmd: &str, timeout: Duration) -> Result<Vec<String>, ObdError> {
        self.drain_left_over()?;
        // a request (hex bytes, not an AT or ST command) goes onto the bus: kept for take_sent
        if !cmd.is_empty()
            && cmd.len().is_multiple_of(2)
            && cmd.bytes().all(|b| b.is_ascii_hexdigit())
        {
            let bytes: Vec<u8> = (0..cmd.len())
                .step_by(2)
                .filter_map(|i| u8::from_str_radix(&cmd[i..i + 2], 16).ok())
                .collect();
            let id = match self.target {
                Some((tx, _)) => tx,
                None if matches!(self.protocol, Protocol::Can29At500 | Protocol::Can29At250) => {
                    0x18DB33F1
                }
                None => 0x7DF,
            };
            self.sent.push_back((Instant::now(), id, bytes));
            if self.sent.len() > SENT_KEEP {
                self.sent.pop_front();
            }
        }
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
    /// only when the session is not read-only and the vehicle's Read-Only switch is off, whichever part of Icarus asks.
    /// Reads, and the requests that hand control back, always go.
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
        if self.protocol.is_can() && data.len() > 7 {
            return Err(ObdError::Adapter(format!(
                "a request of {} bytes to every module does not fit one CAN frame, and Icarus sends no longer one to \
                 every module",
                data.len()
            )));
        }
        self.bus_to(Bus::Main)?;
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

    /// What this adapter reaches on this vehicle: the ID width of the vehicle's CAN protocol, and Ford's medium-speed CAN
    /// when the adapter has an STN chip (an OBDLink, which switches buses: `STN_UNVERIFIED`), the vehicle's OBD-II runs on
    /// 11-bit CAN at 500 kbit/s (Ford's high-speed CAN), and the adapter has not refused the switch.
    pub fn reach(&self) -> crate::link::Reach {
        crate::link::Reach {
            ms_can: self.stn.is_some()
                && self.no_ms_can.is_none()
                && self.protocol == Protocol::Can11At500,
            ..crate::link::Reach::of(self.protocol)
        }
    }

    /// Moves the `request_to` calls that follow to `bus` (the switch is made now; `uds::at_module` moves them back).
    /// Broadcasts and watching always go on the main bus, switching back first. A switch that fails is tried again before
    /// the next request, so no request goes out on a bus other than its own.
    pub fn set_bus(&mut self, bus: Bus) -> Result<(), ObdError> {
        if bus == Bus::MsCan && !self.reach().ms_can {
            return Err(ObdError::Adapter(self.no_ms_can.clone().unwrap_or_else(|| {
                "switching to Ford's medium-speed CAN needs an OBDLink (an STN chip) on a vehicle whose OBD-II runs on \
                 11-bit CAN at 500 kbit/s; this adapter or vehicle is not one"
                    .into()
            })));
        }
        self.want_bus = bus;
        self.bus_to(bus)
    }

    /// Puts the adapter on `bus` when it is not there (or not known to be). After a switch the adapter's header, filters
    /// and extended addressing are said again before the next request.
    fn bus_to(&mut self, bus: Bus) -> Result<(), ObdError> {
        if self.bus == Some(bus) {
            return Ok(());
        }
        let cmd = match bus {
            Bus::MsCan => STN_UNVERIFIED.ms_can,
            Bus::Main => STN_UNVERIFIED.main_bus,
        };
        self.bus = None;
        (self.target, self.ext) = (Some(UNKNOWN_TARGET), None);
        match self.ok_commands(&[cmd]) {
            Ok(()) => {
                self.bus = Some(bus);
                Ok(())
            }
            Err(ObdError::Adapter(why)) if bus == Bus::MsCan => {
                let why =
                    format!("this adapter refused to switch to Ford's medium-speed CAN ({why})");
                self.no_ms_can = Some(why.clone());
                self.want_bus = Bus::Main;
                Err(ObdError::Adapter(why))
            }
            Err(e) => Err(e),
        }
    }

    /// Sends a request to one module on CAN: to request ID `tx`, listening only to answer ID `rx`. 11-bit IDs (up to 7FF;
    /// the answer usually tx + 8) on a vehicle whose OBD-II uses 11-bit CAN, 29-bit IDs (18DAxxF1 answered from 18DAF1xx)
    /// on one that uses 29-bit CAN (`header_commands`). Flow control for long answers goes to `tx` too, which modules
    /// outside OBD's own 7E0-7E7 need.
    pub fn request_to(&mut self, tx: u32, rx: u32, data: &[u8]) -> Result<Vec<Answer>, ObdError> {
        // a read this vehicle refused for good before (ICA-71): not asked again, the refusal comes back as it did
        if let Some(&code) = self.refused.get(&(tx, self.want_ext, data.to_vec())) {
            if let Some(&sid) = data.first() {
                self.spared += 1;
                let from = if rx > 0x7FF {
                    format!("{rx:08X}")
                } else {
                    format!("{rx:03X}")
                };
                return Ok(vec![Answer {
                    from,
                    data: vec![0x7F, sid, code],
                }]);
            }
        }
        let r = self.request_to_inner(tx, rx, data);
        // what the module did with it, for the vehicle's record (ICA-70): data, a refusal and its reason, or silence; an
        // adapter's own trouble says nothing about the module and is left out
        let got = match &r {
            Ok(answers) => {
                let mine: Vec<&Answer> = answers
                    .iter()
                    .filter(|a| {
                        u32::from_str_radix(&a.from, 16).ok() == Some(rx) || answers.len() == 1
                    })
                    .collect();
                match mine.iter().find_map(|a| a.refusal()) {
                    Some((_, code)) => Some(crate::record::Got::Refused(code)),
                    None => mine.first().map(|a| crate::record::Got::Data(a.data.len())),
                }
            }
            Err(ObdError::NoData) | Err(ObdError::Timeout(_)) => Some(crate::record::Got::Silent),
            Err(_) => None,
        };
        if let Some(got) = got {
            if self.outcomes.len() < 10_000 {
                self.outcomes.push(crate::record::Outcome {
                    tx,
                    ext: self.want_ext,
                    request: data.to_vec(),
                    got,
                });
            }
        }
        r
    }

    /// Learns from the vehicle's record (ICA-71): the reads it refused for good are not sent again on this connection.
    pub fn remember(&mut self, r: &crate::record::Record) {
        self.refused = r.refused_for_good().into_iter().collect();
    }

    /// What the requests to single modules brought back since the last call (ICA-70).
    pub fn take_outcomes(&mut self) -> Vec<crate::record::Outcome> {
        std::mem::take(&mut self.outcomes)
    }

    /// A sweep's probe (`Link::probe`, ICA-103): tester present to `tx`, with the answers shown from every ID a module could
    /// answer from, not only `rx`. The header and flow control are said as for one module, with `ATCRA` alone in place of
    /// its answer ID, so no earlier request's answer ID stays among the adapter's filters whichever of them it goes by; then
    /// the receive filter is opened to 700-7FF (`ATCF700`, `ATCM700`), or on 29-bit CAN to 18DAF1xx (`ATCF18DAF100`,
    /// `ATCM1FFFFF00`); an STN chip gets its own pass filter instead (`STFCP`, then `STFAP`), as for watching
    /// (`monitor_ids`). Only the probe goes out under it. Afterwards the filter goes back to automatic (`ATCRA`, or `STFCP`
    /// on an STN), and the next request says its own header and answer ID again whatever happened. Behind a gateway, on the
    /// medium-speed CAN, or where the wide filter would not pass `rx` (an 11-bit ID below 700), it listens to `rx` alone,
    /// as `request_to` does.
    ///
    /// It hears at least what `request_to(tx, rx)` hears, so no module is missed that the probe at `rx` alone found before:
    /// when nothing from `rx` answered the probe through the wide filter, the probe is sent once more to `rx` alone and what
    /// that hears is added. An adapter that refuses the wide filter (`?` to ATCF, ATCM, STFCP or STFAP), or reports trouble
    /// under it, is probed at `rx` alone for the rest of the connection (`no_wide`); the scan goes on.
    ///
    /// What the module did with the probe goes in the vehicle's record (ICA-70) only from `rx`'s answer: another sender heard
    /// in the window says nothing about the module at `tx`. When `rx` did not answer, the probe at `rx` alone records it, as
    /// a request to one module does; a module found answering elsewhere is recorded when the scan asks it there again.
    ///
    /// UNVERIFIED ON HARDWARE: the filter commands are the ELM327 and STN references' as I understand them, tested only
    /// against the simulated adapter, like `STN_UNVERIFIED`. The probe at `rx` alone is the floor for that reason.
    pub fn probe(&mut self, tx: u32, rx: u32, data: &[u8]) -> Result<Vec<Answer>, ObdError> {
        crate::uds::check_probe(data)?;
        let can29 = tx > 0x7FF;
        let (f, m) = if can29 { WIDE29 } else { WIDE11 };
        if self.no_wide.is_some()
            || self.want_ext.is_some()
            || self.want_bus != Bus::Main
            || rx & m != f
            || (can29 && !crate::vehicles::is_fixed29_request(tx))
        {
            return self.request_to(tx, rx, data);
        }
        let Some(mut heard) = self.probe_wide(tx, rx, data)? else {
            // the adapter would not listen wide (remembered): the probe at rx alone
            return self.request_to(tx, rx, data);
        };
        let from_rx = heard.iter().find(|a| {
            u32::from_str_radix(&a.from, 16).ok() == Some(rx) && crate::uds::answers_probe(data, a)
        });
        if let Some(a) = from_rx {
            let got = match a.refusal() {
                Some((_, code)) => crate::record::Got::Refused(code),
                None => crate::record::Got::Data(a.data.len()),
            };
            if self.outcomes.len() < 10_000 {
                self.outcomes.push(crate::record::Outcome {
                    tx,
                    ext: None,
                    request: data.to_vec(),
                    got,
                });
            }
            return Ok(heard);
        }
        // the floor: the probe at rx alone, as before the wide probe (its outcome kept by `request_to`)
        match self.request_to(tx, rx, data) {
            Ok(a) => heard.extend(a),
            Err(ObdError::NoData) => {}
            Err(e) => return Err(e),
        }
        Ok(heard)
    }

    /// Why this adapter's probes listen to the guessed answer ID alone, when it refused the wide filter (`probe`).
    pub fn no_wide(&self) -> Option<&str> {
        self.no_wide.as_deref()
    }

    /// `probe`'s wide window: what the adapter showed through the wide filter, or None when it refused the filter or
    /// reported trouble under it (then remembered in `no_wide`). A link's own trouble (no prompt, the port lost) is an error.
    fn probe_wide(
        &mut self,
        tx: u32,
        rx: u32,
        data: &[u8],
    ) -> Result<Option<Vec<Answer>>, ObdError> {
        self.read_only_check(data)?;
        let can29 = self.sendable(tx, rx, data)?.0;
        self.bus_to(Bus::Main)?;
        self.format_frames()?;
        self.ext_to(None)?;
        // whatever happens from here, the next request says its header and answer ID again
        self.target = Some(UNKNOWN_TARGET);
        let header: Vec<String> = header_commands(Some((tx, rx)), can29)
            .into_iter()
            .map(|c| {
                if c.starts_with("ATCRA") {
                    "ATCRA".to_string()
                } else {
                    c
                }
            })
            .collect();
        self.ok_commands(&header.iter().map(String::as_str).collect::<Vec<_>>())?;
        let (f, m) = if can29 { WIDE29 } else { WIDE11 };
        let hex = |id: u32| {
            if can29 {
                format!("{id:08X}")
            } else {
                format!("{id:03X}")
            }
        };
        let wide: Vec<String> = if self.stn.is_some() {
            vec!["STFCP".into(), format!("STFAP {},{}", hex(f), hex(m))]
        } else {
            vec![format!("ATCF{}", hex(f)), format!("ATCM{}", hex(m))]
        };
        let heard = self
            .ok_commands(&wide.iter().map(String::as_str).collect::<Vec<_>>())
            .and_then(|()| self.request_lines(data));
        // back to automatic receive filtering; a failure here is put right by the next request, which sets its own filter
        // (ATCRA with its answer ID, or ATCRA alone before a broadcast)
        let back: &[&str] = if self.stn.is_some() {
            &["STFCP"]
        } else {
            &["ATCRA"]
        };
        let _ = self.ok_commands(back);
        match heard {
            Ok(lines) => Ok(Some(probe_heard(&lines, self.protocol))),
            Err(ObdError::NoData) => Ok(Some(Vec::new())),
            Err(ObdError::Adapter(why)) => {
                self.no_wide = Some(format!(
                    "the adapter would not listen to every answer ID ({why}), so each probe listens at the guessed one"
                ));
                Ok(None)
            }
            Err(e) => Err(e),
        }
    }

    /// Whether a request of `data` can go to `tx`, answered from `rx`, on this vehicle and adapter: (29-bit, more than one
    /// frame), or why not.
    fn sendable(&self, tx: u32, rx: u32, data: &[u8]) -> Result<(bool, bool), ObdError> {
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
        if wide && self.want_bus == Bus::MsCan {
            return Err(ObdError::Adapter(
                "on the medium-speed CAN Icarus asks on 11-bit IDs only".into(),
            ));
        }
        // a single frame carries 7 bytes (6 behind a gateway); longer requests go as several frames, which only an STN
        // chip's STPX sends
        let long = data.len() > if self.want_ext.is_some() { 6 } else { 7 };
        if long && (self.stn.is_none() || self.want_ext.is_some()) {
            return Err(ObdError::Adapter(format!(
                "a request of {} bytes needs several CAN frames, which {}",
                data.len(),
                if self.stn.is_none() {
                    "only an adapter with an STN chip (an OBDLink, with STPX) sends; this ELM327 sends one frame"
                } else {
                    "Icarus does not send to a module behind a gateway"
                }
            )));
        }
        Ok((wide, long))
    }

    fn request_to_inner(&mut self, tx: u32, rx: u32, data: &[u8]) -> Result<Vec<Answer>, ObdError> {
        self.read_only_check(data)?;
        let (wide, long) = self.sendable(tx, rx, data)?;
        self.bus_to(self.want_bus)?;
        self.format_frames()?;
        self.ext_to(self.want_ext)?;
        if self.target != Some((tx, rx)) {
            let cmds = header_commands(Some((tx, rx)), wide);
            self.ok_commands(&cmds.iter().map(String::as_str).collect::<Vec<_>>())?;
            self.target = Some((tx, rx));
        }
        if long {
            let cmd = stn_long_request(tx, data);
            let lines = self.command(&cmd, Duration::from_secs(5))?;
            check_errors(&lines)?;
            return parse_lines(&lines, self.protocol);
        }
        match self.want_ext {
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
        if self.ext != Some(want) {
            let cmd = match want {
                Some(a) => format!("ATCEA{a:02X}"),
                None => "ATCEA".to_string(),
            };
            self.ok_commands(&[&cmd])?;
            self.ext = Some(want);
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
    /// - On a serial line whose speed can change (`Port::speed`: a USB cable, not Bluetooth or Wi-Fi), the line is sped up
    ///   for the watch (`raise_speed`: `STBR` on an STN, `ATBRD` on an ELM327, by the adapter's own handshake) and put
    ///   back as soon as it ends, so a busy bus does not fill the adapter's buffer (ICA-213); an adapter that refuses or
    ///   does not confirm is watched at the speed it was at. The result says the speed watched at when it changed.
    ///
    /// An OK to `ATCSM1` or `STCMM0` proves only that the adapter accepted the command, not that it obeys it: a clone that
    /// answers OK to everything can still acknowledge frames while it watches.
    ///
    /// Only the adapter refusing `ATCSM1` or `STCMM0` is reported as an adapter that cannot stay silent; a timeout or a lost
    /// link comes back as itself. If the watch went well but `ATCAF1` failed, the watch's result comes back, saying that
    /// auto-formatting goes back on before the next request (it stays pending until it does).
    pub fn monitor(&mut self, each: impl FnMut(&str) -> bool) -> Result<String, ObdError> {
        self.monitor_ids(None, each)
    }

    /// `monitor`, the adapter passing only IDs that match `only` (filter, mask) when given: `(0x700, 0x700)` keeps the
    /// diagnostic messages (700-7FF), what another tool asks and the modules answer, when the whole bus is more than the
    /// adapter's serial link carries (it stops with BUFFER FULL). The filter goes back to automatic afterwards (ATCRA).
    pub fn monitor_ids(
        &mut self,
        only: Option<(u32, u32)>,
        each: impl FnMut(&str) -> bool,
    ) -> Result<String, ObdError> {
        self.bus_to(Bus::Main)?;
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
        // the STN's own monitor (STMA) follows its pass filters (STFAP), not the ELM's ATCF/ATCM: with those only, an OBDLink
        // EX watched the whole bus and stopped with BUFFER FULL (the truck, 2026-11-22)
        let stn = self.stn.is_some();
        let filter: Vec<String> = match only {
            None => vec![],
            Some((f, m)) if stn => vec!["STFCP".into(), format!("STFAP {f:03X},{m:03X}")],
            Some((f, m)) => vec![format!("ATCF{f:03X}"), format!("ATCM{m:03X}")],
        };
        let filter: Vec<&str> = filter.iter().map(String::as_str).collect();
        // the serial link sped up for the watch (ICA-213), and put back as soon as it ends
        let mut raised = None;
        let watched = self
            .ok_commands(&["ATCAF0"])
            .and_then(|()| self.ok_commands(&filter))
            .and_then(|()| {
                raised = self.raise_speed()?;
                self.watch(only.is_some(), each)
            });
        let watched = match raised {
            None => watched,
            Some((from, to)) => {
                let back = self.switch_speed(from);
                watched.map(|how| match back {
                    Ok(true) => format!(
                        "{how}; watched at {to} baud, the adapter's serial link back at {from} afterwards"
                    ),
                    Ok(false) => format!(
                        "{how}; watched at {to} baud, and the adapter's serial link did not go back to {from} (it stays \
                         at {to} until the adapter is unplugged or reset)"
                    ),
                    Err(e) => format!("{how}; watched at {to} baud, and then {e}"),
                })
            }
        };
        if only.is_some() {
            // back to no filter (STN) or automatic receive filtering; the next request sets its own (ATCRA with its address)
            let _ = self.ok_commands(if stn { &["STFCP"] } else { &["ATCRA"] });
        }
        // put back whatever happened to the watch; on failure `raw_frames` stays set and the next request tries again
        match (watched, self.format_frames()) {
            (Err(e), _) => Err(e),
            (Ok(how), Ok(())) => Ok(how),
            (Ok(how), Err(e)) => Ok(format!(
                "{how}; CAN auto-formatting could not be turned back on yet ({e}), so it goes back on before the next request"
            )),
        }
    }

    /// Speeds the adapter's serial link up for a watch (ICA-213: an OBDLink EX watching the truck's whole bus at 115200
    /// stopped with BUFFER FULL after about 200 frames), when the port is a serial line whose speed can change and the
    /// speed it runs at now can be asked back afterwards. The speeds `WATCH_SPEEDS` names are tried fastest first, each
    /// with the adapter's own handshake (`switch_speed`); Some((the speed before, the one now)) when one took, None when
    /// none did (a clone that does not know the command answers `?`, and the watch goes on at the speed it was at).
    fn raise_speed(&mut self) -> Result<Option<(u32, u32)>, ObdError> {
        let Some(from) = self.port.speed() else {
            return Ok(None);
        };
        if self.speed_command(from).is_none() {
            return Ok(None);
        }
        let speeds: &[u32] = if self.stn.is_some() {
            &WATCH_SPEEDS.stn
        } else {
            &WATCH_SPEEDS.elm
        };
        for &to in speeds.iter().filter(|&&s| s > from) {
            if self.switch_speed(to)? {
                return Ok(Some((from, to)));
            }
        }
        Ok(None)
    }

    /// The command that asks this adapter to change its serial speed to `baud` by its handshake, or None when it cannot be
    /// said: an STN's `STBR` takes the speed itself in decimal; an ELM327's `ATBRD hh` a divisor, the speed being
    /// 4000 kbps / hh (hh 01-FF), so only a speed that comes within 3% of one is said (38400 is hh 68, 115200 hh 23:
    /// 114.3 kbps, 500000 hh 08).
    fn speed_command(&self, baud: u32) -> Option<String> {
        if baud == 0 {
            return None;
        }
        if self.stn.is_some() {
            return Some(format!("STBR {baud}"));
        }
        let div = (4_000_000 + baud / 2) / baud;
        if !(1..=0xFF).contains(&div) {
            return None;
        }
        let made = 4_000_000 / div;
        (made.abs_diff(baud) * 100 <= baud * 3).then(|| format!("ATBRD {div:02X}"))
    }

    /// Changes the adapter's serial speed, and this computer's side of the line with it, to `to`, by the handshake the
    /// ELM327 datasheet gives for `AT BRD` ("Using Higher RS232 Baud Rates") and the STN reference gives `STBR` the same:
    /// the adapter answers OK at the old speed (no prompt after it), switches, and after its wait (75 ms unless
    /// `ATBRT`/`STBRT` says otherwise) sends its ID (`ATI`'s string; an STN's `STI` string) at the new speed; when a
    /// carriage return comes back at the new speed in that wait it answers OK and stays there, and otherwise goes back to
    /// the old speed by itself. Ok(true) when both sides now run at `to`; Ok(false) when the adapter refused (`?`: a clone
    /// that does not know the command, or a speed it cannot make) or did not confirm, both sides at the old speed again;
    /// an error only when the adapter answers at neither speed.
    fn switch_speed(&mut self, to: u32) -> Result<bool, ObdError> {
        let (Some(from), Some(cmd)) = (self.port.speed(), self.speed_command(to)) else {
            return Ok(false);
        };
        let id = self.stn.clone().unwrap_or_else(|| self.id.clone());
        if id.is_empty() {
            // nothing to know the adapter's answer at the new speed by
            return Ok(false);
        }
        self.drain_left_over()?;
        self.port.send(format!("{cmd}\r").as_bytes())?;
        let (said, _) = self.read_until(Duration::from_secs(2), |t| {
            has(t, b"OK\r") || t.contains(&b'>')
        })?;
        if !has(&said, b"OK\r") && said.contains(&b'>') {
            return Ok(false);
        }
        if has(&said, b"OK\r") {
            self.port.set_speed(to)?;
            let (seen, _) = self.read_until(Duration::from_millis(500), |t| {
                find(t, id.as_bytes()).is_some_and(|i| t[i..].contains(&b'\r'))
            })?;
            if has(&seen, id.as_bytes()) {
                self.port.send(b"\r")?;
                let (ok, _) = self.read_until(Duration::from_millis(500), |t| t.contains(&b'>'))?;
                if has(&ok, b"OK") {
                    return Ok(true);
                }
            }
        }
        self.find_speed(from, to)
    }

    /// Where the adapter is after a speed change it did not confirm: at `from` again (it goes back by itself once its
    /// wait has passed), or at `to` after all (it took the change but its OK was lost). Asked with `ATI`, which only the
    /// adapter answers; Ok(true) when it is at `to`. This computer's side is left at the speed that answered.
    fn find_speed(&mut self, from: u32, to: u32) -> Result<bool, ObdError> {
        self.port.set_speed(from)?;
        // past the adapter's own wait (75 ms, twice, by default), so it has gone back
        std::thread::sleep(Duration::from_millis(250));
        for speed in [from, to] {
            self.port.set_speed(speed)?;
            // twice: the first may only end a handshake still waiting, its characters taken for a wrong carriage return
            for _ in 0..2 {
                match self.command("ATI", Duration::from_secs(1)) {
                    Ok(lines) if lines.iter().any(|l| l.contains("ELM")) => return Ok(speed == to),
                    Ok(_) | Err(ObdError::Timeout(_)) => {}
                    Err(e) => return Err(e),
                }
            }
        }
        self.port.set_speed(from)?;
        Err(ObdError::Adapter(format!(
            "the adapter stopped answering while its serial speed was changed from {from} to {to} baud: unplug it and plug \
             it back in"
        )))
    }

    /// The adapter's bytes until `done` says what came is complete (true), or `limit` passed (false).
    fn read_until(
        &mut self,
        limit: Duration,
        done: impl Fn(&[u8]) -> bool,
    ) -> Result<(Vec<u8>, bool), ObdError> {
        let deadline = Instant::now() + limit;
        let mut text = Vec::new();
        let mut buf = [0u8; 256];
        loop {
            if done(&text) {
                return Ok((text, true));
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Ok((text, false));
            }
            let n = self
                .port
                .recv(&mut buf, left.min(Duration::from_millis(50)))?;
            text.extend_from_slice(&buf[..n]);
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
    /// `filtered`: an STN watches with STM, which follows its pass filters; its STMA shows everything whatever they say (the
    /// truck, 2026-11-22: with STMA the filter changed nothing). An ELM327's ATMA follows ATCF/ATCM.
    fn watch(
        &mut self,
        filtered: bool,
        mut each: impl FnMut(&str) -> bool,
    ) -> Result<String, ObdError> {
        let cmd = match (self.stn.is_some(), filtered) {
            (true, true) => "STM",
            (true, false) => "STMA",
            (false, _) => "ATMA",
        };
        self.drain_left_over()?;
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

/// Where `part` starts in `text`.
fn find(text: &[u8], part: &[u8]) -> Option<usize> {
    if part.is_empty() {
        return Some(0);
    }
    text.windows(part.len()).position(|w| w == part)
}

fn has(text: &[u8], part: &[u8]) -> bool {
    find(text, part).is_some()
}

/// The serial speeds a watch asks the adapter for (`Elm::raise_speed`), fastest first.
pub struct WatchSpeeds {
    /// An STN chip (an OBDLink): `STBR` takes these. The STN21xx/STN22xx datasheet gives its UART up to 8 Mbps
    /// (theoretical); the OBDLink EX and SX reach the computer through an FTDI FT231X, which makes 3,000,000 / (n + k/8)
    /// baud (n from 2, k 0-7) and 2 Mbaud as a special case (FTDI's FT231X datasheet and application note AN232B-05), so
    /// these three come out exact on both sides.
    pub stn: [u32; 3],
    /// A plain ELM327: `ATBRD 08`. The ELM327 datasheet's revision history says v1.2, which brought `AT BRD`, made the
    /// serial speed adjustable to 500 kbps, and no more is asked of an ELM327 or a clone; one that does not know the
    /// command answers `?`.
    pub elm: [u32; 1],
}

/// UNVERIFIED ON HARDWARE: the speed change follows the OBDLink Family Reference and Programming Manual (`STBR`, which
/// "operates identically to ATBRD" with the speed in decimal, `?` for one it cannot make within 3%, and the STI string as
/// its ID) and the ELM327 datasheet (`AT BRD`'s handshake), tested only against the simulated adapter.
pub const WATCH_SPEEDS: WatchSpeeds = WatchSpeeds {
    stn: [2_000_000, 1_000_000, 500_000],
    elm: [500_000],
};

/// The adapter's header and filters are not known (after a switch of bus): any request says them again.
const UNKNOWN_TARGET: (u32, u32) = (u32::MAX, u32::MAX);

/// A sweep's probe listens through these receive filters (filter, mask: an ID passes when its bits under the mask equal
/// the filter's): 11-bit IDs 700-7FF, where diagnostic answers are, and on 29-bit CAN 18DAF1xx, every module's answer
/// to the tester at F1 (ISO 15765-4's normal fixed addressing).
pub const WIDE11: (u32, u32) = (0x700, 0x700);
pub const WIDE29: (u32, u32) = (0x18DA_F100, 0x1FFF_FF00);

/// The STN-only commands Icarus sends (an OBDLink's STN chip; a plain ELM327 is never sent them), and the ELM327 command
/// that goes back from them.
pub struct StnCommands {
    /// Switches to Ford's medium-speed CAN: ISO 15765, 11-bit IDs, 125 kbit/s, on OBD pins 3 and 11 (`STP 53`).
    pub ms_can: &'static str,
    /// Back to the vehicle's own OBD-II protocol on pins 6 and 14: ISO 15765-4 CAN, 11-bit, 500 kbit/s (`ATSP 6`, the
    /// ELM327's own command, which the STN takes). Switching to MS-CAN is offered only on that protocol.
    pub main_bus: &'static str,
    /// One request longer than a CAN frame, which the STN sends as ISO-TP frames itself: the header, the data, one answer
    /// expected (`STPX`). `{H}` and `{D}` are the header's and the data's hex digits.
    pub long_request: &'static str,
}

/// UNVERIFIED ON HARDWARE: these are the STN11xx/STN22xx family reference's commands as I understand them (STP 53 for the
/// MS-CAN on an OBDLink MX+ or EX, STPX for long messages), tested only against the simulated adapter. Until each has been
/// seen to work on a real OBDLink, a scan's "no answer" from a medium-speed module, or a refused long request, may be this
/// table's fault rather than the vehicle's. Everything Icarus sends to switch buses or send long requests is here.
pub const STN_UNVERIFIED: StnCommands = StnCommands {
    ms_can: "STP53",
    main_bus: "ATSP6",
    long_request: "STPX H:{H},D:{D},R:1",
};

/// The `STPX` command for one long request to `tx`.
pub fn stn_long_request(tx: u32, data: &[u8]) -> String {
    let header = if tx > 0x7FF {
        format!("{tx:08X}")
    } else {
        format!("{tx:03X}")
    };
    let hex: String = data.iter().map(|b| format!("{b:02X}")).collect();
    STN_UNVERIFIED
        .long_request
        .replace("{H}", &header)
        .replace("{D}", &hex)
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

/// What a sweep's probe heard through the wide filter (`Elm::probe`): every sender's whole message, and for a sender whose
/// message did not add up (another conversation's long message, its frames cut off) the start of it as its first frame
/// gives it, so the scan's guard still sees what kind of message it was (`uds::is_response`). A probe's own answer is one
/// frame. Lines that are not frames at all are nothing heard.
fn probe_heard(lines: &[String], protocol: Protocol) -> Vec<Answer> {
    let Ok((mut answers, bad)) = parse_each(lines, protocol) else {
        return Vec::new();
    };
    let frames = frames_of(lines, protocol).unwrap_or_default();
    for (from, _) in bad {
        let start = frames
            .iter()
            .find(|(h, b)| *h == from && b.len() >= 8 && b[0] >> 4 == 1);
        if let Some((_, b)) = start {
            answers.push(Answer {
                from,
                data: b[2..8].to_vec(),
            });
        }
    }
    answers
}

/// Lines with headers as frames: (header, bytes), or the first line that is not a frame.
fn frames_of(lines: &[String], protocol: Protocol) -> Result<Vec<(String, Vec<u8>)>, ObdError> {
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
    Ok(frames)
}

/// `parse_each` for frames of `frame` ISO-TP bytes (8 on classic CAN, 7 behind a gateway).
fn parse_sized(lines: &[String], protocol: Protocol, frame: usize) -> Result<Parsed, ObdError> {
    let frames = frames_of(lines, protocol)?;
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
    /// ICA-103: what a probe heard through the wide filter keeps the start of a long message whose frames were cut off, as
    /// its first frame gives it, so the scan sees it was an answer; a whole message stays whole, and lines that are not
    /// frames are nothing heard.
    #[test]
    fn a_probe_keeps_the_start_of_a_message_cut_off() {
        use super::{probe_heard, Answer, Protocol};
        let lines: Vec<String> = ["7D6 10 0A 62 F1 90 01 02 03", "7C8 02 7E 00 AA AA AA AA AA"]
            .iter()
            .map(|s| s.to_string())
            .collect();
        assert_eq!(
            probe_heard(&lines, Protocol::Can11At500),
            vec![
                Answer {
                    from: "7C8".into(),
                    data: vec![0x7E, 0x00]
                },
                Answer {
                    from: "7D6".into(),
                    data: vec![0x62, 0xF1, 0x90, 0x01, 0x02, 0x03]
                },
            ]
        );
        assert!(probe_heard(&["SEARCHING".to_string()], Protocol::Can11At500).is_empty());
    }

    /// A serial port at one speed: the simulated car where the adapter answers; elsewhere a garbled reply to each command
    /// (a wrong speed), or bytes that never stop (an adapter left in monitor mode).
    struct AtSpeed {
        car: Option<crate::sim::SimCar>,
        reply: Vec<u8>,
        streams: bool,
    }
    fn at_speed(answers: bool) -> Box<dyn crate::port::Port> {
        Box::new(AtSpeed {
            car: answers.then(crate::sim::SimCar::new),
            reply: vec![],
            streams: false,
        })
    }
    impl crate::port::Port for AtSpeed {
        fn send(&mut self, data: &[u8]) -> io::Result<()> {
            match &mut self.car {
                Some(c) => c.send(data),
                None => {
                    self.reply = b"?\r\r>".to_vec();
                    Ok(())
                }
            }
        }
        fn recv(&mut self, buf: &mut [u8], timeout: Duration) -> io::Result<usize> {
            if let Some(c) = &mut self.car {
                return c.recv(buf, timeout);
            }
            if self.streams {
                buf[..8].copy_from_slice(b"7E8 03 4");
                return Ok(8);
            }
            if self.reply.is_empty() {
                std::thread::sleep(timeout.min(Duration::from_millis(20)));
                return Ok(0);
            }
            let n = self.reply.len().min(buf.len());
            buf[..n].copy_from_slice(&self.reply[..n]);
            self.reply.drain(..n);
            Ok(n)
        }
    }

    /// Whether an adapter is wired is asked of the port it was opened on (`Port::wired`), not of a path: the simulation
    /// is; the same simulated adapter behind a port that cannot tell is not.
    #[test]
    fn wired_is_asked_of_the_open_port() {
        assert!(Elm::connect(Box::new(crate::sim::SimCar::new()))
            .unwrap()
            .wired());
        assert!(!Elm::connect(at_speed(true)).unwrap().wired());
    }

    #[test]
    fn a_port_that_never_goes_quiet_does_not_hold_icarus_up() {
        let t = Instant::now();
        let streaming = Box::new(AtSpeed {
            car: None,
            reply: vec![],
            streams: true,
        });
        assert!(super::Elm::connect(streaming).is_err());
        assert!(t.elapsed() < Duration::from_secs(8), "{:?}", t.elapsed());
    }

    #[test]
    fn a_serial_adapter_is_found_at_another_usual_speed() {
        // an OBDLink EX answers at 115200 only; asked for 38400 first (a bare /dev/ttyUSB0), it is found and said so
        let mut asked = vec![];
        let (elm, note) = super::connect_serial("/dev/ttyUSB0", 38_400, &mut |b| {
            asked.push(b);
            Ok(at_speed(b == 115_200))
        })
        .unwrap();
        assert_eq!(asked, [38_400, 115_200]);
        assert!(note.unwrap().contains("answered at 115200 baud, not 38400"));
        assert!(!crate::link::Link::adapter(&elm).is_empty());
        // at its own speed: no note, nothing else tried
        let mut asked = vec![];
        let (_, note) = super::connect_serial("/dev/ttyUSB0", 115_200, &mut |b| {
            asked.push(b);
            Ok(at_speed(b == 115_200))
        })
        .unwrap();
        assert_eq!((asked, note), (vec![115_200], None));
        // nothing answers at any speed (an OBDLink EX off the vehicle): every speed tried, and why it may be silent
        let e = super::connect_serial("/dev/ttyUSB0", 38_400, &mut |_| Ok(at_speed(false)))
            .err()
            .unwrap();
        assert!(
            e.contains("38400, 115200, 500000, 230400, 9600")
                && e.contains("power from the vehicle"),
            "{e}"
        );
        // a port that cannot be opened is said at once
        let e = super::connect_serial("/dev/ttyUSB9", 38_400, &mut |_| {
            Err(io::Error::from(io::ErrorKind::NotFound))
        })
        .err()
        .unwrap();
        assert!(e.starts_with("/dev/ttyUSB9:"), "{e}");
    }

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
    fn long_requests_go_as_stpx_on_an_obdlink_only() {
        use crate::sim::recording::Recorder;
        use crate::sim::FORD_VIN;
        // four identifiers in one read: nine bytes, more than a CAN frame carries
        let req = [0x22, 0xF1, 0x87, 0xF1, 0x88, 0xF1, 0x89, 0xF1, 0x90];
        let (port, sent) = Recorder::new(SimCar::ford(true));
        let mut elm = Elm::connect(Box::new(port)).unwrap();
        sent.lock().unwrap().clear();
        let a = elm.request_to(0x7E0, 0x7E8, &req).unwrap();
        let d = &a[0].data;
        assert_eq!(&d[..3], &[0x62, 0xF1, 0x87]);
        let text = String::from_utf8_lossy(d);
        assert!(
            text.contains("ICARUS-FORD-PCM") && text.contains(FORD_VIN),
            "{text}"
        );
        let log = sent.lock().unwrap().clone();
        let stpx = log
            .iter()
            .position(|l| l == "STPX H:7E0,D:22F187F188F189F190,R:1\r");
        let sh = log.iter().position(|l| l == "ATSH7E0\r");
        assert!(sh.is_some() && stpx > sh, "{log:?}");
        assert!(!log.iter().any(|l| l == "22F187F188F189F190\r"), "{log:?}");
        assert_eq!(
            stn_long_request(0x18DA_30F1, &[0x22, 0xF1, 0x87]),
            "STPX H:18DA30F1,D:22F187,R:1"
        );
        // a plain ELM327 sends single frames only: refused before anything is sent
        let (port, sent) = Recorder::new(SimCar::ford(false));
        let mut elm = Elm::connect(Box::new(port)).unwrap();
        sent.lock().unwrap().clear();
        let e = elm.request_to(0x7E0, 0x7E8, &req).unwrap_err().to_string();
        assert!(e.contains("STN") && e.contains("9 bytes"), "{e}");
        assert!(sent.lock().unwrap().is_empty());
        assert!(elm
            .request(&[0x01, 0x0C, 0x0D, 0x05, 0x0B, 0x0F, 0x11, 0x04])
            .is_err());
        // the Read-Only backstop holds for long requests too: a write to a switched vehicle never reaches STPX
        crate::lock::switch_for_test(&[FORD_VIN]);
        let (port, sent) = Recorder::new(SimCar::ford(true));
        let mut elm = Elm::connect(Box::new(port)).unwrap();
        sent.lock().unwrap().clear();
        let write = [0x2E, 0xF1, 0x90, 0x31, 0x46, 0x41, 0x49, 0x43, 0x41];
        let e = elm.request_to(0x7E0, 0x7E8, &write).unwrap_err();
        assert!(matches!(e, ObdError::ReadOnly(_)), "{e}");
        let log = sent.lock().unwrap().clone();
        assert!(!log.iter().any(|l| l.starts_with("STPX")), "{log:?}");
        crate::lock::switch_for_test(&[]);
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
