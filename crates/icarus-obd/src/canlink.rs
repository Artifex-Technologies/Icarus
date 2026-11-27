//! Diagnostics through a Linux CAN interface (SocketCAN: a CANable, or any adapter Linux shows as `can0`), as a `Link`.
//! Icarus speaks ISO 15765-2 (ISO-TP) over raw CAN frames itself (no kernel `can-isotp` module): single frames, first and
//! consecutive frames with their sequence numbers, flow control both ways (block size, gap, "wait"), and ISO 15765's
//! waits. OBD-II's broadcast goes to 7DF (18DB33F1 on 29-bit CAN) and every module's answer is gathered; a request to one
//! module goes to its request ID and only its answer ID is read.
//!
//! The link is written against `Frames`, raw frames in and out, so the tests drive it with the simulated vehicle on an
//! in-memory bus (`SimBus`); `socketcan::CanSocket` is the real one. As on every link to a vehicle, a request that would
//! change a vehicle switched to read-only is refused before any frame is sent (`lock`).

use crate::bus::Frame;
use crate::elm::{Answer, ObdError, Protocol};
use crate::link::{Link, Reach};
use crate::sim::SimCar;
use std::collections::{BTreeSet, VecDeque};
use std::io;
use std::time::{Duration, Instant};

/// Raw CAN frames in and out: a SocketCAN socket (`socketcan::CanSocket`) or the simulated bus (`SimBus`).
pub trait Frames {
    /// Puts one frame on the bus.
    fn send(&mut self, f: &Frame) -> io::Result<()>;
    /// The next frame from the bus, waiting up to `timeout`; None when none came in that time.
    fn recv(&mut self, timeout: Duration) -> io::Result<Option<Frame>>;
    /// The voltage at the OBD port (the vehicle's battery) measured now, when the frame source can measure it: a raw-CAN
    /// source on a cable that reads pin 16 (the OpenPort does) could. None by default: a SocketCAN interface and the
    /// Mongoose driver measure nothing, and the write paths refuse a link whose battery is not known.
    fn voltage(&mut self) -> Option<f64> {
        None
    }
}

/// ISO-TP's waits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Timing {
    /// How long a module has to start answering, and on a broadcast how long after the last answer the link still listens
    /// for another module's (an ELM327's default wait).
    pub answer: Duration,
    /// How much longer a module that said "response pending" (7F xx 78) may take each time (ISO 14229's P2*).
    pub pending: Duration,
    /// The longest gap inside one message: to the next consecutive frame (ISO 15765-2's N_Cr), or to the flow control
    /// after a first frame Icarus sent (N_Bs).
    pub frame: Duration,
}

impl Default for Timing {
    fn default() -> Self {
        Timing {
            answer: Duration::from_millis(200),
            pending: Duration::from_secs(5),
            frame: Duration::from_secs(1),
        }
    }
}

/// What a frame is filled up to eight bytes with (ISO 15765-4 sends every diagnostic frame eight bytes long).
const PAD: u8 = 0x00;
/// The most "wait" flow controls Icarus takes before giving up on a module (ISO 15765-2's N_WFTmax).
const MAX_WAITS: usize = 10;
/// OBD-II's broadcast on 11-bit and on 29-bit CAN.
const FUNCTIONAL11: u32 = 0x7DF;
const FUNCTIONAL29: u32 = 0x18DB_33F1;

/// The other side of a 29-bit normal fixed address: 18DA<a><b> to 18DA<b><a>.
fn swapped(id: u32) -> u32 {
    (id & 0xFFFF_0000) | ((id & 0xFF) << 8) | ((id >> 8) & 0xFF)
}

/// How an answer's sender is written: three digits, or eight for a 29-bit ID (as the ELM327 shows them).
fn sender(id: u32) -> String {
    if id > 0x7FF {
        format!("{id:08X}")
    } else {
        format!("{id:03X}")
    }
}

/// The gap a flow control asks for between consecutive frames (ISO 15765-2's STmin byte).
fn gap(st: u8) -> Duration {
    match st {
        0x00..=0x7F => Duration::from_millis(st as u64),
        0xF1..=0xF9 => Duration::from_micros((st as u64 - 0xF0) * 100),
        _ => Duration::from_millis(0x7F),
    }
}

/// Whole messages, and (sender, why) for each sender whose message did not add up.
type Gathered = (Vec<Answer>, Vec<(String, String)>);

/// A Linux CAN interface (or any `Frames`) as Icarus's link to the vehicle.
pub struct CanLink<F: Frames> {
    bus: F,
    name: String,
    protocol: Protocol,
    /// The address byte in front of each frame to one module (`set_ext`), or None.
    ext: Option<u8>,
    pub timing: Timing,
    /// Notes on senders left out of broadcasts, not yet taken (`take_dropped`), each (sender, why) once.
    unseen: Vec<String>,
    seen: BTreeSet<(String, String)>,
    /// The interface's bit rate when known (`with_bitrate`; SocketCAN: `socketcan::bitrate`).
    bitrate: Option<u32>,
    /// What the frames go through, when it is not a SocketCAN interface (`named`: a Mongoose cable).
    label: Option<String>,
}

impl<F: Frames> CanLink<F> {
    /// Finds the vehicle on the bus `name` ("can0"): OBD-II's first request (01 00) on 11-bit CAN, then on 29-bit CAN, as
    /// ISO 15765-4 does. The interface's bit rate is whatever it was set up with (`ip link`): the protocol says 500 kbit/s,
    /// the rate of every OBD-II CAN vehicle, until `with_bitrate` tells the real one.
    pub fn connect(bus: F, name: &str) -> Result<Self, ObdError> {
        Self::with_timing(bus, name, Timing::default())
    }

    /// The frame source underneath, for a test that looks at what went out on it.
    #[cfg(test)]
    pub(crate) fn frames_mut(&mut self) -> &mut F {
        &mut self.bus
    }

    /// `connect` with other waits.
    pub fn with_timing(bus: F, name: &str, timing: Timing) -> Result<Self, ObdError> {
        let mut link = CanLink {
            bus,
            name: name.to_string(),
            protocol: Protocol::Can11At500,
            ext: None,
            timing,
            unseen: Vec::new(),
            seen: BTreeSet::new(),
            bitrate: None,
            label: None,
        };
        for p in [Protocol::Can11At500, Protocol::Can29At500] {
            link.protocol = p;
            match link.request(&[0x01, 0x00]) {
                Ok(a) if !a.is_empty() => return Ok(link),
                Ok(_) | Err(ObdError::NoData) => {}
                Err(e) => return Err(e),
            }
        }
        Err(ObdError::NoConnection(format!(
            "nothing answered OBD-II on {name}, on 11-bit or 29-bit CAN (is the interface up, at the vehicle's bit rate, \
             usually 500 kbit/s?)"
        )))
    }

    /// What the adapter is called, for a `Frames` that is not a SocketCAN interface ("Drewtech Mongoose 1.2.4.0").
    pub fn named(mut self, label: impl Into<String>) -> Self {
        self.label = Some(label.into());
        self
    }

    /// The interface's real bit rate (ICA-68): at 250 kbit/s the protocol is ISO 15765-4's 250 kbit/s one; any rate is named
    /// in the adapter's description. None leaves it as it is.
    pub fn with_bitrate(mut self, rate: Option<u32>) -> Self {
        self.bitrate = rate;
        if rate == Some(250_000) {
            self.protocol = match self.protocol {
                Protocol::Can29At500 => Protocol::Can29At250,
                Protocol::Can11At500 => Protocol::Can11At250,
                p => p,
            };
        }
        self
    }

    /// The frames underneath (the socket, or the simulated bus), to look at: what a simulated bus was sent. Read only: a
    /// frame put on the bus past `request` and `request_to` would skip the Read-Only switch's check (`lock`), so nothing
    /// outside this module gets the frames to send on.
    ///
    /// ```
    /// use icarus_obd::canlink::{CanLink, Frames, SimBus};
    /// use icarus_obd::sim::SimCar;
    /// let mut l = CanLink::connect(SimBus::new(SimCar::new()), "vcan0").unwrap();
    /// let f = l.frames().sent[0].clone();
    /// assert_eq!(f.id, 0x7DF, "connecting asked every module");
    /// ```
    ///
    /// Sending on the bus through it does not build:
    ///
    /// ```compile_fail,E0596
    /// use icarus_obd::canlink::{CanLink, Frames, SimBus};
    /// use icarus_obd::sim::SimCar;
    /// let mut l = CanLink::connect(SimBus::new(SimCar::new()), "vcan0").unwrap();
    /// let f = l.frames().sent[0].clone();
    /// l.frames().send(&f).unwrap();
    /// ```
    pub fn frames(&self) -> &F {
        &self.bus
    }

    /// The backstop under every write path, as on the ELM327 link: a request that would change the vehicle goes only when
    /// its Read-Only switch is off.
    fn read_only_check(&mut self, data: &[u8]) -> Result<(), ObdError> {
        if crate::lock::changes_vehicle(data) {
            crate::lock::writes_allowed(self).map_err(ObdError::ReadOnly)?;
        }
        Ok(())
    }

    /// One frame on `id`: the address byte first when given, then the ISO-TP bytes, filled to eight.
    fn out(&mut self, id: u32, ext: Option<u8>, payload: &[u8]) -> Result<(), ObdError> {
        let mut data: Vec<u8> = ext.into_iter().chain(payload.iter().copied()).collect();
        data.resize(8, PAD);
        self.bus.send(&Frame {
            id,
            ext: id > 0x7FF,
            data,
            at: 0.0,
        })?;
        Ok(())
    }

    /// The ISO-TP bytes of a frame from `from` (its address byte checked and taken off when `ext` is given).
    fn payload(f: &Frame, from: u32, ext: Option<u8>) -> Option<Vec<u8>> {
        if f.id != from {
            return None;
        }
        match ext {
            None => Some(f.data.clone()),
            Some(a) => (f.data.first() == Some(&a)).then(|| f.data[1..].to_vec()),
        }
    }

    /// Sends one message to `to`: a single frame, or (to one module, whose flow control comes from `fc_from`) a first
    /// frame and consecutive frames paced by that flow control.
    fn send_message(
        &mut self,
        to: u32,
        fc_from: Option<u32>,
        ext: Option<u8>,
        data: &[u8],
    ) -> Result<(), ObdError> {
        let room = if ext.is_some() { 6 } else { 7 };
        if data.is_empty() {
            return Err(ObdError::Adapter("an empty request".into()));
        }
        if data.len() <= room {
            return self.out(to, ext, &[&[data.len() as u8][..], data].concat());
        }
        let Some(fc_from) = fc_from else {
            return Err(ObdError::Adapter(format!(
                "a request of {} bytes to every module does not fit one CAN frame, and Icarus sends no longer one to \
                 every module",
                data.len()
            )));
        };
        if data.len() > 4095 {
            return Err(ObdError::Adapter(format!(
                "a request of {} bytes is longer than ISO-TP carries",
                data.len()
            )));
        }
        let first = room - 1;
        let ff = [
            &[0x10 | (data.len() >> 8) as u8, data.len() as u8][..],
            &data[..first],
        ]
        .concat();
        self.out(to, ext, &ff)?;
        let (mut rest, mut seq) = (&data[first..], 1u8);
        while !rest.is_empty() {
            let (block, pause) = self.flow_control(fc_from, ext)?;
            let mut n = 0;
            while !rest.is_empty() && (block == 0 || n < block) {
                let take = rest.len().min(room);
                self.out(to, ext, &[&[0x20 | seq][..], &rest[..take]].concat())?;
                (rest, seq, n) = (&rest[take..], (seq + 1) & 0x0F, n + 1);
                if !rest.is_empty() && !pause.is_zero() {
                    std::thread::sleep(pause);
                }
            }
        }
        Ok(())
    }

    /// The module's flow control after a first frame Icarus sent: (block size, gap). "Wait" is waited out, up to
    /// `MAX_WAITS` times; "overflow" and silence are errors.
    fn flow_control(&mut self, from: u32, ext: Option<u8>) -> Result<(usize, Duration), ObdError> {
        let (mut waits, mut deadline) = (0, Instant::now() + self.timing.frame);
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            let f = if left.is_zero() {
                None
            } else {
                self.bus.recv(left)?
            };
            let Some(f) = f else {
                return Err(ObdError::Adapter(format!(
                    "{} sent no flow control in time for a long request",
                    sender(from)
                )));
            };
            let Some(d) = Self::payload(&f, from, ext) else {
                continue;
            };
            match d.first() {
                Some(0x30) => {
                    let (bs, st) = (d.get(1).copied(), d.get(2).copied());
                    return Ok((bs.unwrap_or(0) as usize, gap(st.unwrap_or(0))));
                }
                Some(0x31) if waits < MAX_WAITS => {
                    waits += 1;
                    deadline = Instant::now() + self.timing.frame;
                }
                Some(0x31) => {
                    return Err(ObdError::Adapter(format!(
                        "{} kept asking to wait for a long request",
                        sender(from)
                    )))
                }
                Some(0x32) => {
                    return Err(ObdError::Adapter(format!(
                        "{} cannot take a request that long (flow control: overflow)",
                        sender(from)
                    )))
                }
                _ => {} // not a flow control
            }
        }
    }

    /// Gathers the messages from the IDs `from` accepts, frames put back together per sender (flow control sent to
    /// `fc_to(sender)` after each first frame), "response pending" waited out, until the bus has been quiet for
    /// `timing.answer` (inside a message: `timing.frame`), or with `one` until the first whole message. A sender whose
    /// message does not add up (a frame lost or repeated, one missing at the end, lengths classic CAN cannot carry) is
    /// named with why and its messages left out, never kept short or wrong.
    ///
    /// With no `fc_to` (a sweep's probe, whose answer is one frame) a first frame is taken as it is, the start of a longer
    /// message (its bytes after the length, so the scan's guard sees what kind of message it is), with no flow control, so
    /// nothing is sent into another sender's conversation; its consecutive frames are passed over.
    fn gather(
        &mut self,
        from: &dyn Fn(u32) -> bool,
        fc_to: Option<&dyn Fn(u32) -> u32>,
        ext: Option<u8>,
        one: bool,
    ) -> Result<Gathered, ObdError> {
        struct Open {
            from: u32,
            buf: Vec<u8>,
            len: usize,
            next: u8,
        }
        let room = if ext.is_some() { 7 } else { 8 };
        let (mut done, mut bad, mut open): (Vec<Answer>, Vec<(String, String)>, Vec<Open>) =
            (Vec::new(), Vec::new(), Vec::new());
        let mut deadline = Instant::now() + self.timing.answer;
        loop {
            let wait = deadline.saturating_duration_since(Instant::now());
            if wait.is_zero() {
                break;
            }
            let Some(f) = self.bus.recv(wait)? else {
                break;
            };
            if !from(f.id) || bad.iter().any(|b| b.0 == sender(f.id)) {
                continue;
            }
            let id = f.id;
            let Some(d) = Self::payload(&f, id, ext) else {
                continue;
            };
            let Some(&pci) = d.first() else { continue };
            let mut whole: Option<Vec<u8>> = None;
            let problem = match pci >> 4 {
                1 if fc_to.is_none() => {
                    if d.len() >= room && !done.iter().any(|a| a.from == sender(id)) {
                        whole = Some(d[2..room].to_vec());
                    }
                    None
                }
                2 if fc_to.is_none() => None,
                0 => {
                    let n = (pci & 0x0F) as usize;
                    if n == 0 || n >= room || d.len() < 1 + n {
                        Some(format!("{} single frame of {n} bytes", sender(id)))
                    } else {
                        open.retain(|o| o.from != id);
                        whole = Some(d[1..1 + n].to_vec());
                        None
                    }
                }
                1 => {
                    let n = d
                        .get(1)
                        .map_or(0, |b| ((pci as usize & 0x0F) << 8) | *b as usize);
                    if n < room || d.len() < room {
                        Some(format!("{} first frame of a {n}-byte message", sender(id)))
                    } else if open.iter().any(|o| o.from == id) {
                        Some(format!(
                            "{} started a new message before the last one ended",
                            sender(id)
                        ))
                    } else {
                        open.push(Open {
                            from: id,
                            buf: d[2..room].to_vec(),
                            len: n,
                            next: 1,
                        });
                        if let Some(fc_to) = fc_to {
                            self.out(fc_to(id), ext, &[0x30, 0x00, 0x00])?;
                        }
                        None
                    }
                }
                2 => match open.iter_mut().find(|o| o.from == id) {
                    None => Some(format!(
                        "{} continued a message that never started",
                        sender(id)
                    )),
                    Some(o) if pci & 0x0F != o.next => Some(format!(
                        "{} frame {} came after frame {}: a frame was lost or repeated",
                        sender(id),
                        pci & 0x0F,
                        (o.next + 15) & 0x0F
                    )),
                    Some(o) => {
                        o.next = (o.next + 1) & 0x0F;
                        let take = (o.len - o.buf.len()).min(d.len() - 1);
                        o.buf.extend_from_slice(&d[1..1 + take]);
                        if o.buf.len() >= o.len {
                            whole = Some(std::mem::take(&mut o.buf));
                            open.retain(|o| o.from != id);
                        }
                        None
                    }
                },
                _ => None, // flow control, or not ISO-TP: not an answer
            };
            let now = Instant::now();
            if let Some(why) = problem {
                open.retain(|o| o.from != id);
                done.retain(|a| a.from != sender(id));
                bad.push((sender(id), why));
                if one {
                    break;
                }
            } else if let Some(msg) = whole {
                if msg.len() >= 3 && msg[0] == 0x7F && msg[2] == 0x78 {
                    deadline = now + self.timing.pending;
                    continue;
                }
                done.push(Answer {
                    from: sender(id),
                    data: msg,
                });
                if one {
                    break;
                }
            }
            let quiet = if open.is_empty() {
                self.timing.answer
            } else {
                self.timing.frame
            };
            // a probe listens for its one answer's time only (pending answers aside): other traffic in the range it hears
            // does not keep it listening
            if fc_to.is_some() {
                deadline = deadline.max(now + quiet);
            }
        }
        for o in open {
            done.retain(|a| a.from != sender(o.from));
            bad.push((
                sender(o.from),
                format!(
                    "{} message of {} bytes ended after {}",
                    sender(o.from),
                    o.len,
                    o.buf.len()
                ),
            ));
        }
        Ok((done, bad))
    }

    /// Notes the senders a broadcast of `data` left out, once each a connection.
    fn note(&mut self, data: &[u8], bad: &[(String, String)]) {
        for (from, why) in bad {
            if self.seen.insert((from.clone(), why.clone())) {
                let req: Vec<String> = data.iter().map(|b| format!("{b:02X}")).collect();
                self.unseen.push(format!(
                    "{from}'s answer to {} (asked of every module) was left out: {why}; the other modules' answers \
                     were kept",
                    req.join(" ")
                ));
            }
        }
    }
}

impl<F: Frames> Link for CanLink<F> {
    fn request(&mut self, data: &[u8]) -> Result<Vec<Answer>, ObdError> {
        self.read_only_check(data)?;
        let can29 = matches!(self.protocol, Protocol::Can29At500 | Protocol::Can29At250);
        let to = if can29 { FUNCTIONAL29 } else { FUNCTIONAL11 };
        self.send_message(to, None, None, data)?;
        let from = move |id: u32| {
            if can29 {
                id & 0xFFFF_FF00 == 0x18DA_F100
            } else {
                (0x7E8..=0x7EF).contains(&id)
            }
        };
        // flow control for a long answer goes to the sender's own request ID: 7E8's to 7E0, 18DAF110's to 18DA10F1
        let fc_to = |id: u32| if id > 0x7FF { swapped(id) } else { id - 8 };
        let (answers, bad) = self.gather(&from, Some(&fc_to), None, false)?;
        self.note(data, &bad);
        if answers.is_empty() {
            return Err(match bad.into_iter().next() {
                Some((_, why)) => ObdError::Garbled(why),
                None => ObdError::NoData,
            });
        }
        Ok(answers)
    }

    fn request_to(&mut self, tx: u32, rx: u32, data: &[u8]) -> Result<Vec<Answer>, ObdError> {
        self.read_only_check(data)?;
        if tx > 0x1FFF_FFFF || rx > 0x1FFF_FFFF || (tx > 0x7FF) != (rx > 0x7FF) || tx == rx {
            return Err(ObdError::Adapter(format!(
                "{tx:X}/{rx:X} are not a request and an answer ID, both 11-bit or both 29-bit"
            )));
        }
        let ext = self.ext;
        self.send_message(tx, Some(rx), ext, data)?;
        let (answers, bad) = self.gather(&|id| id == rx, Some(&|_| tx), ext, true)?;
        if let Some((_, why)) = bad.into_iter().next() {
            return Err(ObdError::Garbled(why));
        }
        if answers.is_empty() {
            return Err(ObdError::NoData);
        }
        Ok(answers)
    }

    /// A sweep's probe (ICA-103): tester present to `tx`, and every frame heard from `rx` and from 700-7FF (but `tx`
    /// itself), or on 29-bit CAN from 18DAF1xx, in the answer's time (`timing.answer`, longer after a "response pending"):
    /// raw frames carry every ID, so the module is heard wherever it answers from, and `rx` as `request_to` hears it. A
    /// first frame of a longer message is taken as its start, with no flow control (`gather`). Behind a gateway, or for a
    /// 29-bit ID that is not ISO 15765-4's 18DAxxF1, the probe listens to `rx` alone, as `request_to` does.
    fn probe(&mut self, tx: u32, rx: u32, data: &[u8]) -> Result<Vec<Answer>, ObdError> {
        crate::uds::check_probe(data)?;
        let can29 = tx > 0x7FF;
        if self.ext.is_some() || (can29 && !crate::vehicles::is_fixed29_request(tx)) {
            return self.request_to(tx, rx, data);
        }
        self.read_only_check(data)?;
        if tx > 0x1FFF_FFFF || rx > 0x1FFF_FFFF || can29 != (rx > 0x7FF) || tx == rx {
            return Err(ObdError::Adapter(format!(
                "{tx:X}/{rx:X} are not a request and an answer ID, both 11-bit or both 29-bit"
            )));
        }
        self.send_message(tx, None, None, data)?;
        let from = move |id: u32| {
            id != tx
                && (id == rx
                    || if can29 {
                        id & 0xFFFF_FF00 == 0x18DA_F100
                    } else {
                        (0x700..=0x7FF).contains(&id)
                    })
        };
        // a sender whose frames do not add up is not an answer to the probe
        let (answers, _) = self.gather(&from, None, None, false)?;
        Ok(answers)
    }

    fn protocol(&self) -> Protocol {
        self.protocol
    }

    fn adapter(&self) -> String {
        let what = self
            .label
            .clone()
            .unwrap_or_else(|| format!("SocketCAN {}", self.name));
        match self.bitrate {
            Some(r) if r.is_multiple_of(1000) => format!("{what} ({} kbit/s)", r / 1000),
            Some(r) => format!("{what} ({r} bit/s)"),
            None => what,
        }
    }

    /// Whatever the frames underneath measure (`Frames::voltage`): a SocketCAN interface measures nothing, so the write
    /// paths refuse it.
    fn voltage(&mut self) -> Option<f64> {
        self.bus.voltage()
    }

    fn set_ext(&mut self, ext: Option<u8>) -> Result<(), ObdError> {
        self.ext = ext;
        Ok(())
    }

    /// Any frame goes out as given: 11-bit and 29-bit IDs both, on the one bus the interface is wired to.
    fn reach(&self) -> Reach {
        Reach {
            can11: true,
            can29: true,
            ms_can: false,
        }
    }

    fn take_dropped(&mut self) -> Vec<String> {
        std::mem::take(&mut self.unseen)
    }
}

/// A long request coming in to the simulated vehicle.
struct Incoming {
    to: u32,
    ext: Option<u8>,
    buf: Vec<u8>,
    len: usize,
    /// The sequence number the next consecutive frame must carry.
    next: u8,
    /// Consecutive frames since the last flow control.
    since: u8,
}

/// The simulated vehicle on a CAN bus, frame by frame: requests come in as ISO-TP frames (a long one paced by the
/// module's flow control), answers go out as ISO-TP frames, a long answer's consecutive frames only after the tester's
/// flow control. Every frame the tester sent is kept in `sent`. For the tests, and the other end of a real `vcan0` in the
/// test that uses one.
pub struct SimBus {
    car: SimCar,
    out: VecDeque<Frame>,
    /// A module's consecutive frames waiting for the tester's flow control: (the ID it comes on, address byte, frames).
    held: Vec<(u32, Option<u8>, VecDeque<Frame>)>,
    /// A long request coming in.
    incoming: Option<Incoming>,
    pub sent: Vec<Frame>,
    /// The block size and gap a module answers a long request's first frame with ((0, 0): all at once, no gap).
    pub flow: (u8, u8),
    /// The "wait" flow controls a module sends before its "go on", to each first frame.
    pub waits: u8,
    /// A module's long answers stop after their first frame (it went quiet in the middle of a message).
    pub stall: bool,
    /// The battery voltage the bus says it measures (`Frames::voltage`): None, as on a SocketCAN interface, unless a test
    /// stands in for a frame source that measures it.
    pub voltage: Option<f64>,
}

impl SimBus {
    pub fn new(car: SimCar) -> SimBus {
        SimBus {
            car,
            out: VecDeque::new(),
            held: Vec::new(),
            incoming: None,
            sent: Vec::new(),
            flow: (0, 0),
            waits: 0,
            stall: false,
            voltage: None,
        }
    }

    /// A message as the module sends it: ISO-TP frames on `id`, the address byte first when given.
    fn frames(id: u32, ext: Option<u8>, msg: &[u8]) -> Vec<Frame> {
        let room = if ext.is_some() { 7 } else { 8 };
        let frame = |b: Vec<u8>| {
            let mut data: Vec<u8> = ext.into_iter().chain(b).collect();
            data.resize(8, 0xAA);
            Frame {
                id,
                ext: id > 0x7FF,
                data,
                at: 0.0,
            }
        };
        if msg.len() < room {
            return vec![frame([&[msg.len() as u8][..], msg].concat())];
        }
        let mut v = vec![frame(
            [
                &[0x10 | (msg.len() >> 8) as u8, msg.len() as u8][..],
                &msg[..room - 2],
            ]
            .concat(),
        )];
        for (i, c) in msg[room - 2..].chunks(room - 1).enumerate() {
            v.push(frame([&[0x20 | ((i + 1) & 0x0F) as u8][..], c].concat()));
        }
        v
    }

    /// The module's answers to a whole request on `to`.
    fn answer(&mut self, to: u32, ext: Option<u8>, req: &[u8]) {
        let functional = to == self.car.functional() && ext.is_none();
        // other nodes on the bus while a tester present is answered (`SimCar::with_chatter`, `with_chatter_once`): a raw
        // interface hears them all
        if !functional && req.first() == Some(&0x3E) {
            for (from, msg) in self.car.chatter_at(to) {
                self.out.extend(Self::frames(from, None, &msg));
            }
        }
        for (from, x, msg) in self.car.can_answers(to, ext, req) {
            let mut frames: VecDeque<Frame> = Self::frames(from, x, &msg).into();
            self.out.extend(frames.pop_front());
            if !frames.is_empty() && !self.stall {
                // the tester's flow control comes on the module's request ID
                let fc_on = if !functional {
                    to
                } else if from > 0x7FF {
                    swapped(from)
                } else {
                    from - 8
                };
                self.held.push((fc_on, x, frames));
            }
        }
    }

    /// The module's flow control to a long request from the tester.
    fn flow_control(&mut self, to: u32, ext: Option<u8>) {
        let from = self.car.answer_id_of(to);
        for _ in 0..self.waits {
            self.out.extend(Self::control(from, ext, 0x31, 0, 0));
        }
        let (bs, st) = self.flow;
        self.out.extend(Self::control(from, ext, 0x30, bs, st));
    }

    fn control(id: u32, ext: Option<u8>, kind: u8, bs: u8, st: u8) -> Option<Frame> {
        let mut data: Vec<u8> = ext.into_iter().chain([kind, bs, st]).collect();
        data.resize(8, 0xAA);
        Some(Frame {
            id,
            ext: id > 0x7FF,
            data,
            at: 0.0,
        })
    }
}

/// Plays the simulated vehicle `sim` on a real bus (the second socket on a `vcan0`) until `stop` says so: every frame
/// that comes in is answered on the bus as the simulation answers it.
pub fn serve(bus: &mut impl Frames, sim: &mut SimBus, stop: impl Fn() -> bool) -> io::Result<()> {
    while !stop() {
        if let Some(f) = bus.recv(Duration::from_millis(20))? {
            sim.send(&f)?;
            while let Some(a) = sim.recv(Duration::ZERO)? {
                bus.send(&a)?;
            }
        }
    }
    Ok(())
}

impl Frames for SimBus {
    fn send(&mut self, f: &Frame) -> io::Result<()> {
        self.sent.push(f.clone());
        let ext = match self.car.gateway() {
            Some(g) if g == f.id => f.data.first().copied(),
            _ => None,
        };
        let d = &f.data[usize::from(ext.is_some())..];
        let Some(&pci) = d.first() else {
            return Ok(());
        };
        match pci >> 4 {
            0 => {
                let n = (pci & 0x0F) as usize;
                if n > 0 && d.len() > n {
                    self.answer(f.id, ext, &d[1..=n]);
                }
            }
            1 if d.len() >= 2 => {
                self.incoming = Some(Incoming {
                    to: f.id,
                    ext,
                    buf: d[2..].to_vec(),
                    len: ((pci as usize & 0x0F) << 8) | d[1] as usize,
                    next: 1,
                    since: 0,
                });
                self.flow_control(f.id, ext);
            }
            2 => {
                let Some(mut r) = self.incoming.take() else {
                    return Ok(());
                };
                if r.to != f.id || pci & 0x0F != r.next {
                    return Ok(()); // the request is dropped: a frame was lost
                }
                r.buf.extend_from_slice(&d[1..]);
                if r.buf.len() >= r.len {
                    r.buf.truncate(r.len);
                    self.answer(r.to, r.ext, &r.buf);
                } else {
                    (r.next, r.since) = ((r.next + 1) & 0x0F, r.since + 1);
                    if self.flow.0 != 0 && r.since == self.flow.0 {
                        r.since = 0;
                        self.flow_control(r.to, r.ext);
                    }
                    self.incoming = Some(r);
                }
            }
            3 if pci == 0x30 => {
                // the tester's "go on": a held answer's frames (all at once; the tester asks for no blocks)
                if let Some(i) = self.held.iter().position(|h| h.0 == f.id && h.1 == ext) {
                    let (_, _, frames) = self.held.remove(i);
                    self.out.extend(frames);
                }
            }
            _ => {}
        }
        Ok(())
    }

    fn recv(&mut self, _timeout: Duration) -> io::Result<Option<Frame>> {
        Ok(self.out.pop_front())
    }

    fn voltage(&mut self) -> Option<f64> {
        self.voltage
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::j1979;
    use crate::sim::VIN;
    use crate::uds;

    fn link(car: SimCar) -> CanLink<SimBus> {
        CanLink::connect(SimBus::new(car), "vcan0").unwrap()
    }

    /// The frames the tester sent since the last call, and forgotten.
    fn sent(l: &mut CanLink<SimBus>) -> Vec<Frame> {
        std::mem::take(&mut l.bus.sent)
    }

    #[test]
    fn obd_ii_over_raw_frames() {
        let mut l = link(SimCar::new());
        assert_eq!(l.protocol(), Protocol::Can11At500);
        assert_eq!(
            (l.adapter().as_str(), l.voltage()),
            ("SocketCAN vcan0", None)
        );
        sent(&mut l);
        // the VIN comes in three frames: Icarus's flow control goes to the engine's request ID, 7E0
        assert_eq!(j1979::vin(&mut l).unwrap().as_deref(), Some(VIN));
        let frames = sent(&mut l);
        assert_eq!(
            (frames[0].id, &frames[0].data[..3]),
            (0x7DF, &[0x02, 0x09, 0x02][..])
        );
        assert_eq!(frames[0].data.len(), 8, "every frame eight bytes long");
        assert_eq!(
            (frames[1].id, &frames[1].data[..3]),
            (0x7E0, &[0x30, 0x00, 0x00][..])
        );
        assert_eq!(frames.len(), 2);
        let sup = j1979::supported(&mut l).unwrap();
        assert_eq!(sup.len(), 2, "engine and transmission: {sup:?}");
        let rpm = j1979::read(&mut l, 0x0C).unwrap();
        assert!((700.0..900.0).contains(&rpm[0].value));
        assert!(l.take_dropped().is_empty());
    }

    #[test]
    fn modules_one_by_one_pending_answers_and_the_gateway() {
        let mut l = link(SimCar::new());
        let id = uds::module_identity(&mut l, 0x7E0, 0x7E8).unwrap();
        assert!(
            id.contains(&("part number", "89661-ICARUS".to_string())),
            "{id:?}"
        );
        // the cluster says "response pending" first: waited out, then its codes
        let (d, codes) = uds::module_dtcs(&mut l, 0x7C0, 0x7C8).unwrap().unwrap();
        assert_eq!((d, codes.len()), (uds::Dialect::Uds, 0));
        // nothing at 702: no answer, not an error
        assert!(!uds::present(&mut l, 0x702, 0x70A).unwrap());
        // behind the gateway at 750: every frame carries the module's address byte first, the answers too
        sent(&mut l);
        let part = uds::at(&mut l, Some(0x40), |l| {
            uds::module_identity(l, 0x750, 0x758)
        })
        .unwrap();
        assert!(
            part.contains(&("part number", "89221-ICARUS".to_string())),
            "{part:?}"
        );
        let frames = sent(&mut l);
        assert!(
            frames.iter().all(|f| f.id == 0x750 && f.data[0] == 0x40),
            "{frames:?}"
        );
        // and normal addressing afterwards
        assert!(!uds::present(&mut l, 0x750, 0x758).unwrap());
        assert!(uds::present(&mut l, 0x7B0, 0x7B8).unwrap());
    }

    /// ICA-68: the interface's bit rate as the kernel tells it: 250 kbit/s is ISO 15765-4's 250 kbit/s protocol (and a
    /// 29-bit vehicle there still speaks 29-bit), any rate is named in the adapter's description, none leaves both alone.
    #[test]
    fn the_interfaces_bit_rate_is_the_one_shown() {
        let mut l = link(SimCar::honda()).with_bitrate(Some(250_000));
        assert_eq!(l.protocol(), Protocol::Can29At250);
        assert_eq!(l.adapter(), "SocketCAN vcan0 (250 kbit/s)");
        sent(&mut l);
        assert_eq!(
            j1979::vin(&mut l).unwrap().as_deref(),
            Some(crate::sim::HONDA_VIN)
        );
        let frames = sent(&mut l);
        assert!(
            !frames.is_empty() && frames.iter().all(|f| f.ext),
            "still 29-bit: {frames:?}"
        );
        let l = link(SimCar::new()).with_bitrate(Some(250_000));
        assert_eq!(l.protocol(), Protocol::Can11At250);
        let l = link(SimCar::new()).with_bitrate(Some(500_000));
        assert_eq!(
            (l.protocol(), l.adapter().as_str()),
            (Protocol::Can11At500, "SocketCAN vcan0 (500 kbit/s)")
        );
        let l = link(SimCar::new()).with_bitrate(Some(83_333));
        assert_eq!(l.adapter(), "SocketCAN vcan0 (83333 bit/s)");
        let l = link(SimCar::new()).with_bitrate(None);
        assert_eq!(
            (l.protocol(), l.adapter().as_str()),
            (Protocol::Can11At500, "SocketCAN vcan0")
        );
    }

    #[test]
    fn a_29_bit_vehicle_and_its_modules() {
        let mut l = link(SimCar::honda());
        assert_eq!(l.protocol(), Protocol::Can29At500);
        // connecting asked on 11-bit CAN first, then on 29-bit
        let ids: Vec<u32> = sent(&mut l).iter().map(|f| f.id).collect();
        assert_eq!(ids, [0x7DF, 0x18DB_33F1]);
        assert_eq!(
            j1979::vin(&mut l).unwrap().as_deref(),
            Some(crate::sim::HONDA_VIN)
        );
        let frames = sent(&mut l);
        assert!(frames.iter().all(|f| f.ext), "{frames:?}");
        assert!(frames.iter().any(|f| f.id == 0x18DB_33F1));
        assert!(
            frames
                .iter()
                .any(|f| f.id == 0x18DA_10F1 && f.data[0] == 0x30),
            "flow control to the engine"
        );
        // a scan through the guard finds the Honda-like modules by the 29-bit sweep
        let all = crate::vehicles::builtin();
        let mut guard = uds::ScanGuard::default();
        let found: Vec<u32> = uds::sweep_on(&all, l.protocol())
            .iter()
            .filter(|m| guard.present(&mut l, m).unwrap())
            .map(|m| m.tx)
            .collect();
        assert_eq!(found, crate::sim::HONDA_MODULES);
        let (_, codes) = uds::module_dtcs(&mut l, 0x18DA_28F1, 0x18DA_F128)
            .unwrap()
            .unwrap();
        assert_eq!(codes[0].code, "C0210");
    }

    /// ICA-103: on raw CAN the sweep's probe hears a module wherever it answers from: one at 710 answering from 77A (not
    /// 718) is found with that answer ID and read there, and the rest of the sweep sends nothing on 77A. The probe sends
    /// tester present only, and no flow control.
    #[test]
    fn the_sweep_finds_a_module_answering_far_from_its_request_id() {
        let mut l = link(SimCar::new().with_module(0x710, 0x77A));
        let all = crate::vehicles::builtin();
        let list = uds::sweep(&all);
        assert!(list.iter().any(|m| m.tx == 0x77A), "77A is in the list");
        sent(&mut l);
        let mut guard = uds::ScanGuard::new(&all);
        let found: Vec<crate::vehicles::ModuleAddr> = list
            .iter()
            .filter_map(|m| guard.find(&mut l, m).unwrap())
            .collect();
        assert_eq!(
            found.iter().map(|m| m.tx).collect::<Vec<_>>(),
            vec![0x700, 0x701, 0x710, 0x780, 0x7B0, 0x7C0, 0x7C4, 0x7E0, 0x7E1]
        );
        let odd = found.iter().find(|m| m.tx == 0x710).unwrap();
        assert_eq!(odd.rx(), 0x77A);
        assert_eq!(guard.answer_of(0x77A), Some(0x710));
        let frames = sent(&mut l);
        let on: BTreeSet<u32> = frames.iter().map(|f| f.id).collect();
        assert!(!on.contains(&0x77A), "a frame went out on 77A");
        assert!(on.contains(&0x779) && on.contains(&0x77B) && on.contains(&0x718));
        assert!(
            frames.iter().all(|f| f.data[1] == 0x3E),
            "tester present only, no flow control: {frames:?}"
        );
        // read at the answer ID it was heard from; at 718, the old guess, nothing answers
        let id = uds::module_identity(&mut l, odd.tx, odd.rx()).unwrap();
        assert!(
            id.contains(&("part number", "ICARUS-ODD-ID".to_string())),
            "{id:?}"
        );
        assert!(!uds::present(&mut l, 0x710, 0x718).unwrap());
    }

    /// ICA-103: another node's long message in the probe's window (7D6's 62 F1 90 and seven bytes more: a first frame and
    /// its consecutive frames, while each tester present is answered) gets no flow control from the probe, so every frame
    /// the sweep sends is a tester present. 7D6 is not taken for a module, the probe hears its message's start, and since
    /// that is an answer the sweep never sends on 7D6.
    #[test]
    fn the_probe_sends_no_flow_control_into_another_conversation() {
        let mut l =
            link(SimCar::new().with_chatter(0x7D6, &[0x62, 0xF1, 0x90, 1, 2, 3, 4, 5, 6, 7]));
        let all = crate::vehicles::builtin();
        let list = uds::sweep(&all);
        assert!(list.iter().any(|m| m.tx == 0x7D6), "7D6 is in the list");
        sent(&mut l);
        let mut guard = uds::ScanGuard::new(&all);
        let found: Vec<u32> = list
            .iter()
            .filter_map(|m| guard.find(&mut l, m).unwrap())
            .map(|m| m.tx)
            .collect();
        assert_eq!(
            found,
            vec![0x700, 0x701, 0x780, 0x7B0, 0x7C0, 0x7C4, 0x7E0, 0x7E1]
        );
        let frames = sent(&mut l);
        assert!(
            !frames.iter().any(|f| f.data[0] & 0xF0 == 0x30),
            "flow control went out: {:?}",
            frames.iter().find(|f| f.data[0] & 0xF0 == 0x30)
        );
        assert!(frames.iter().all(|f| f.data[..2] == [0x02, 0x3E]));
        assert!(guard.never_sends_on(0x7D6));
        assert!(!frames.iter().any(|f| f.id == 0x7D6));
        let heard = l.probe(0x702, 0x70A, &[0x3E, 0x00]).unwrap();
        assert!(
            heard
                .iter()
                .any(|a| a.from == "7D6" && a.data == [0x62, 0xF1, 0x90, 1, 2, 3]),
            "{heard:?}"
        );
        assert!(sent(&mut l).iter().all(|f| f.data[0] & 0xF0 != 0x30));
    }

    /// ICA-103: a probe hears at least what a request to the guessed answer ID alone hears, there too where the wide range
    /// does not reach: a module a profile names at 6F0 with no answer ID, answering at 6F8, is found there, on raw CAN as
    /// through an ELM327 (which probes it at 6F8 alone).
    #[test]
    fn the_probe_hears_a_guess_below_the_wide_range() {
        let m = crate::vehicles::ModuleAddr {
            tx: 0x6F0,
            rx: None,
            name: "a module below 700".into(),
            ext: None,
            bus: None,
            checked: false,
        };
        let car = || SimCar::new().with_module(0x6F0, 0x6F8);
        let mut l = link(car());
        let mut elm = crate::elm::Elm::connect(Box::new(car())).unwrap();
        for link in [&mut l as &mut dyn Link, &mut elm] {
            let found = uds::ScanGuard::new(&[]).find(link, &m).unwrap();
            assert_eq!(found.map(|f| f.rx()), Some(0x6F8), "{}", link.adapter());
        }
    }

    #[test]
    fn long_requests_follow_the_modules_flow_control() {
        // seven identifiers in one read: 15 bytes, a first frame and two consecutive frames
        let req: Vec<u8> = [0x22]
            .into_iter()
            .chain(
                [0xF187u16, 0xF188, 0xF190, 0xF187, 0xF188, 0xF190, 0xF187]
                    .iter()
                    .flat_map(|d| d.to_be_bytes()),
            )
            .collect();
        let mut bus = SimBus::new(SimCar::new());
        // the engine asks to wait twice, then takes one frame per flow control
        (bus.flow, bus.waits) = ((1, 0), 2);
        let mut l = CanLink::connect(bus, "vcan0").unwrap();
        sent(&mut l);
        let a = l.request_to(0x7E0, 0x7E8, &req).unwrap();
        assert_eq!(&a[0].data[..3], &[0x62, 0xF1, 0x87]);
        assert!(String::from_utf8_lossy(&a[0].data).contains(VIN));
        let pcis: Vec<u8> = sent(&mut l)
            .iter()
            .filter(|f| f.id == 0x7E0)
            .map(|f| f.data[0])
            .collect();
        assert_eq!(
            &pcis[..3],
            &[0x10, 0x21, 0x22],
            "first frame, then frames 1 and 2: {pcis:02X?}"
        );
        // a module that cannot take it: overflow is an error, not a wait
        let mut l = CanLink::connect(SimBus::new(SimCar::new()), "vcan0").unwrap();
        l.bus.flow = (0, 0);
        l.bus.waits = 11;
        let e = l.request_to(0x7E0, 0x7E8, &req).unwrap_err().to_string();
        assert!(e.contains("wait"), "{e}");
        // a long request to every module is refused before anything is sent
        sent(&mut l);
        assert!(l
            .request(&[0x01, 0x0C, 0x0D, 0x05, 0x0B, 0x0F, 0x11, 0x04])
            .is_err());
        assert!(sent(&mut l).is_empty());
    }

    #[test]
    fn a_message_cut_short_is_an_error_not_a_short_answer() {
        let mut bus = SimBus::new(SimCar::new());
        bus.stall = true;
        let mut l = CanLink::with_timing(
            bus,
            "vcan0",
            Timing {
                answer: Duration::from_millis(20),
                pending: Duration::from_millis(50),
                frame: Duration::from_millis(20),
            },
        )
        .unwrap();
        // the engine's part number comes in a first frame only
        let e = l
            .request_to(0x7E0, 0x7E8, &[0x22, 0xF1, 0x87])
            .unwrap_err()
            .to_string();
        assert!(e.contains("message of 15 bytes ended after 6"), "{e}");
        // the VIN to every module: no whole answer at all is an error, and the sender is noted
        assert!(l.request(&[0x09, 0x02]).is_err());
        let notes = l.take_dropped();
        assert!(
            notes.len() == 1 && notes[0].starts_with("7E8's answer to 09 02"),
            "{notes:?}"
        );
        // short answers still read
        assert!(uds::present(&mut l, 0x7E0, 0x7E8).unwrap());
    }

    #[test]
    fn the_read_only_backstop_holds_before_any_frame() {
        crate::lock::switch_for_test(&[VIN]);
        let mut l = link(SimCar::engine_off());
        sent(&mut l);
        let e = l.request(&[0x04]).unwrap_err();
        assert!(matches!(e, ObdError::ReadOnly(_)), "{e}");
        let e = l
            .request_to(0x7C0, 0x7C8, &[0x2E, 0xA0, 0x01, 0x00])
            .unwrap_err();
        assert!(matches!(e, ObdError::ReadOnly(_)), "{e}");
        // only the VIN reads that checked the switch went out: no clear, no write
        let frames = sent(&mut l);
        assert!(!frames.iter().any(|f| f.id == 0x7C0), "{frames:?}");
        assert!(
            !frames.iter().any(|f| f.data[..2] == [0x01, 0x04]),
            "{frames:?}"
        );
        assert!(l.request(&[0x01, 0x0C]).is_ok(), "reads go");
        crate::lock::switch_for_test(&[]);
    }

    #[test]
    fn nothing_on_the_bus_is_no_vehicle() {
        struct Quiet;
        impl Frames for Quiet {
            fn send(&mut self, _: &Frame) -> io::Result<()> {
                Ok(())
            }
            fn recv(&mut self, _: Duration) -> io::Result<Option<Frame>> {
                Ok(None)
            }
        }
        let e = CanLink::connect(Quiet, "can0").err().unwrap();
        assert!(matches!(e, ObdError::NoConnection(_)), "{e}");
        assert!(e.to_string().contains("can0"));
    }

    /// The same over a real virtual CAN interface, when one exists (it needs root once to make:
    /// `sudo ip link add vcan0 type vcan && sudo ip link set vcan0 up`): a thread plays the simulated vehicle on a second
    /// socket on vcan0.
    #[test]
    #[ignore = "needs a vcan0 interface"]
    fn obd_ii_over_a_real_vcan0() {
        use crate::socketcan;
        let (Ok(tester), Ok(mut car_side)) = (socketcan::open("vcan0"), socketcan::open("vcan0"))
        else {
            eprintln!("no vcan0 here: skipped");
            return;
        };
        let stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let done = stop.clone();
        let car = std::thread::spawn(move || {
            let mut sim = SimBus::new(SimCar::new());
            serve(&mut car_side, &mut sim, || {
                done.load(std::sync::atomic::Ordering::Relaxed)
            })
        });
        let mut l = CanLink::connect(tester, "vcan0").unwrap();
        assert_eq!(j1979::vin(&mut l).unwrap().as_deref(), Some(VIN));
        let id = uds::module_identity(&mut l, 0x7E0, 0x7E8).unwrap();
        assert!(id.contains(&("part number", "89661-ICARUS".to_string())));
        stop.store(true, std::sync::atomic::Ordering::Relaxed);
        car.join().unwrap().unwrap();
    }
}
