//! Watching the CAN bus: every frame the modules send each other, from the adapter's monitor mode (ATMA/STMA) or a Linux
//! CAN interface (`socketcan`). Icarus only listens here; nothing is sent onto the bus.
//!
//! The `Analyzer` keeps a table of IDs (how often each comes, its last bytes, which bytes just changed: press a pedal or a
//! switch and watch which byte follows), puts diagnostic messages (IDs 700-7FF, ISO-TP) back together and says what they
//! ask, and learns actuator tests from another tool: an output-control request (UDS `2F ... 03`, KWP2000 `30 .. 07`) becomes
//! a `[[test]]` entry, unchecked, for the vehicle's profile. A write of one data identifier that the module accepts (UDS
//! `2E`, KWP2000 `3B`) becomes a `[[setting]]` entry the same way, with the value read just before it (`22`/`21`) as "was"
//! and the bits the write changed as its mask (ICA-102). A routine another tool starts (`31 01`) becomes a `[[service]]`.
//!
//! What is never learned (ICA-101), only noted: anything to or from an immobilizer, smart-key or certification module
//! (`fence`, by the frame's CAN ID): its messages are not even put in words in the log, so no learning session, and so no
//! assistant, sees them; and on any module, a write, routine or output test the other tool sent after it unlocked the
//! module with security access (`27`), since it may need access Icarus never uses. The module counts as unlocked until it
//! confirms it is back in its default session (`50 01`, KWP2000's `50 81`) or reset (`51`).
//! The VIN and a module's serial number are masked in what the log says (`capture::mask_message`), and a write of
//! either is never learned.

use crate::actuator::{Condition, Control, Protocol, TestDef};
use crate::elm::Protocol as Bus;
use crate::settings::SettingDef;
use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};

#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub id: u32,
    /// A 29-bit ID.
    pub ext: bool,
    pub data: Vec<u8>,
    /// Seconds since watching began.
    pub at: f64,
}

/// One line of the adapter's monitor output (headers on): "2C4 03 1A F8" (11-bit) or "18 DA F1 10 03 41 0D 00" (29-bit).
pub fn parse_line(line: &str, bus: Bus, at: f64) -> Option<Frame> {
    let toks: Vec<&str> = line.split_whitespace().collect();
    if toks.is_empty()
        || !toks
            .iter()
            .all(|t| t.chars().all(|c| c.is_ascii_hexdigit()))
    {
        return None;
    }
    let ext = matches!(bus, Bus::Can29At500 | Bus::Can29At250 | Bus::J1939);
    let (id, rest) = if ext {
        if toks.len() < 4 || toks[..4].iter().any(|t| t.len() != 2) {
            return None;
        }
        (
            u32::from_str_radix(&toks[..4].concat(), 16).ok()?,
            &toks[4..],
        )
    } else {
        if toks[0].len() != 3 {
            return None;
        }
        (u32::from_str_radix(toks[0], 16).ok()?, &toks[1..])
    };
    if rest.len() > 8 || rest.iter().any(|t| t.len() != 2) {
        return None;
    }
    let data = rest
        .iter()
        .map(|t| u8::from_str_radix(t, 16).unwrap())
        .collect();
    Some(Frame { id, ext, data, at })
}

#[derive(Clone, Debug, Default)]
pub struct IdStats {
    pub count: u64,
    pub first: f64,
    pub last: f64,
    pub data: Vec<u8>,
    /// When each byte last changed (seconds), for highlighting.
    pub changed_at: Vec<f64>,
}

impl IdStats {
    pub fn hz(&self) -> f64 {
        let span = self.last - self.first;
        if self.count > 1 && span > 0.0 {
            (self.count - 1) as f64 / span
        } else {
            0.0
        }
    }
}

/// Service names for what a diagnostic message asks (shared with the capture importers).
fn service(sid: u8) -> &'static str {
    icarus_core::diag::service_name(sid)
}

fn hex(b: &[u8]) -> String {
    b.iter()
        .map(|x| format!("{x:02X}"))
        .collect::<Vec<_>>()
        .join(" ")
}

/// What a whole diagnostic message says, in words.
pub fn describe(msg: &[u8]) -> String {
    let Some(&sid) = msg.first() else {
        return String::new();
    };
    if sid == 0x7F && msg.len() >= 3 {
        return format!("refused {}: {}", service(msg[1]), crate::uds::nrc(msg[2]));
    }
    if (0x40..0x80).contains(&sid) || sid >= 0xC0 {
        let req = sid.wrapping_sub(0x40);
        return format!("answer to {}: {}", service(req), hex(&msg[1..]));
    }
    let detail = match (sid, &msg[1..]) {
        (0x01, [pid, ..]) => crate::j1979::pid(*pid)
            .map(|p| format!("PID {pid:02X} ({})", p.name))
            .unwrap_or(format!("PID {pid:02X}")),
        (0x10, [s, ..]) => match s {
            0x01 => "default".into(),
            0x02 => "programming".into(),
            0x03 => "extended".into(),
            s => format!("{s:02X}"),
        },
        (0x22, [hi, lo, ..]) => format!(
            "{:02X}{:02X}{}",
            hi,
            lo,
            crate::uds::IDENTITY
                .iter()
                .find(|(d, _)| *d == u16::from_be_bytes([*hi, *lo]))
                .map(|(_, n)| format!(" ({n})"))
                .unwrap_or_default()
        ),
        (0x2F, [hi, lo, p, rest @ ..]) => format!(
            "{:02X}{:02X}, {}{}",
            hi,
            lo,
            match p {
                0x00 => "return control to the module",
                0x01 => "reset to default",
                0x02 => "freeze current state",
                0x03 => "short-term adjustment",
                _ => "?",
            },
            if rest.is_empty() {
                String::new()
            } else {
                format!(" to {}", hex(rest))
            }
        ),
        (0x2E, [hi, lo, rest @ ..]) => format!("{hi:02X}{lo:02X} to {}", hex(rest)),
        (0x3B, [lid, rest @ ..]) => format!("{lid:02X} to {}", hex(rest)),
        (0x21, [lid, ..]) => format!("{lid:02X}"),
        (0x30, [lid, p, rest @ ..]) => format!(
            "{lid:02X}, {}{}",
            if *p == 0 { "return control" } else { "adjust" },
            if rest.is_empty() {
                String::new()
            } else {
                format!(" to {}", hex(rest))
            }
        ),
        (_, rest) => hex(rest),
    };
    format!(
        "{}{}",
        service(sid),
        if detail.is_empty() {
            String::new()
        } else {
            format!(": {detail}")
        }
    )
}

/// How long a request waits for its module's answer, in seconds, from the request or the module's last "response
/// pending" (`7F .. 78`): UDS's longest wait after a pending answer (P2*, 5 s; ISO 14229-2).
pub const ANSWER_WAIT: f64 = 5.0;

/// What `ANSWER_WAIT` is given on top: frame times are when Icarus read each frame, not when it was on the bus, and an
/// ELM327/STN adapter's serial link buffers lines (a request can be read late and its answer early, or the other way
/// round), so an answer on time on the bus may look a fraction of a second late here.
pub const ANSWER_SLACK: f64 = 1.0;

/// A request waiting for its module's answer: the entry it becomes, when it was sent (or last said pending), and the mark
/// that was naming what is learned next when it was sent (by its place in `marks`).
struct Waiting<T> {
    def: T,
    at: f64,
    mark: Option<usize>,
}

#[derive(Default)]
pub struct Analyzer {
    pub ids: BTreeMap<u32, IdStats>,
    /// ISO-TP messages being put together, per ID (`icarus_core::diag`, shared with the capture importers).
    partial: icarus_core::diag::Reassembler<u32>,
    /// What the diagnostic messages said, newest last (at most 500).
    pub log: VecDeque<String>,
    /// Actuator tests seen run by another tool.
    pub learned: Vec<TestDef>,
    /// Service routines seen started by another tool (`31 01`), unchecked.
    pub services: Vec<crate::service::ServiceDef>,
    /// Module settings seen changed by another tool (the module accepted the write).
    pub settings: Vec<SettingDef>,
    /// The last value each (module, identifier, KWP2000?) was read as, and writes waiting for the module's answer.
    read_values: HashMap<(u16, u16, bool), Vec<u8>>,
    pending_writes: HashMap<(u16, u16, bool), Vec<u8>>,
    /// Routine starts (`31 01`) waiting for the module's answer, by (module, routine), as the entry each would be, with
    /// when it was sent (or last said pending): one is learned when the module says it started (`71 01`), noted when it
    /// refuses (`7F 31`) or gives no answer in time (`ANSWER_WAIT`).
    pending_routines: HashMap<(u16, u16), Waiting<crate::service::ServiceDef>>,
    /// Output tests (`2F .. 03`, `30 .. 07`) waiting for the module's answer, by (module, output, KWP2000?), the same way:
    /// learned on `6F` / `70`, noted on a refusal or no answer (ICA-221).
    pending_tests: HashMap<(u16, u16, bool), Waiting<TestDef>>,
    /// Answer IDs the profiles give that are not their request ID + 8 (VW's cluster: 714 answered on 77E), each with the
    /// request IDs answered there (`answering`).
    answer_ids: HashMap<u16, Vec<u16>>,
    /// The time of the frame being read.
    now: f64,
    /// Modules another tool has in the extended diagnostic session.
    extended: HashSet<u16>,
    /// What the person said they were about to do in the other tool (`mark`), with the time: newest last.
    pub marks: Vec<(f64, String)>,
    /// The mark that names what is learned next, until it has named one.
    naming: Option<String>,
    /// What was seen but not learned, in words (security access, routines, refused writes...): for the person, or for
    /// Sarina, to work out.
    pub notes: Vec<String>,
    pub frames: u64,
    /// Diagnostic messages described so far (the log keeps only the last 500).
    pub logged: u64,
    /// J1939 (heavy-duty vehicles, ICA-94): the 29-bit frames read as parameter groups, when the bus is J1939; a change in
    /// a sender's active faults goes into the log.
    pub j1939: Option<crate::j1939::Decoder>,
    /// The key modules (`fence`): nothing to or from them is learned or put in words.
    pub fence: crate::fence::Fence,
    /// Modules (by request ID) the other tool sent security access (27) to, until they are back in their default session.
    unlocked: HashSet<u16>,
    /// Learned settings (by their place in `settings`) whose first write changed nothing the read before it showed, so
    /// their mask is not known yet: the next write on them that changes known bits gives them theirs (ICA-102).
    unmasked: HashSet<usize>,
}

impl Analyzer {
    /// An analyzer that keeps away from the key modules of `fence` (`fence::current()` for a real watch).
    pub fn watching(fence: crate::fence::Fence) -> Analyzer {
        Analyzer {
            fence,
            ..Default::default()
        }
    }

    /// The analyzer, pairing each module's answers with its requests as `profiles` give them: an answer ID that is not
    /// its request's + 8 (a `[[module]]` with `rx`). Without it, an answer pairs with the request on its ID - 8 only.
    pub fn answering(mut self, profiles: &[crate::vehicles::Profile]) -> Analyzer {
        for m in profiles.iter().flat_map(|p| &p.modules) {
            let rx = m.rx();
            if m.ext.is_none() && m.tx <= 0x7FF && rx <= 0x7FF && rx != m.tx + 8 {
                let list = self.answer_ids.entry(rx as u16).or_default();
                if !list.contains(&(m.tx as u16)) {
                    list.push(m.tx as u16);
                }
            }
        }
        self
    }

    /// The request IDs an answer on `answer` can be to: the profiles' pairing first, then `answer` - 8, then the
    /// functional request 7DF for the engine-side answers 7E8-7EF.
    fn requests_for(&self, answer: u16) -> Vec<u16> {
        let mut v = self.answer_ids.get(&answer).cloned().unwrap_or_default();
        let plus8 = answer.wrapping_sub(8);
        if !v.contains(&plus8) {
            v.push(plus8);
        }
        if (0x7E8..=0x7EF).contains(&answer) {
            v.push(0x7DF);
        }
        v
    }

    /// The request on `answer` that `waiting` finds: (the ID it was sent on, the module's own request ID). A request sent
    /// to every module (7DF) and answered on 7E8-7EF belongs to that module's physical request ID, 7E0-7E7.
    fn waiting_request(&self, answer: u16, waiting: impl Fn(u16) -> bool) -> Option<(u16, u16)> {
        self.requests_for(answer)
            .into_iter()
            .find(|&c| waiting(c))
            .map(|c| (c, Self::module_of(c, answer)))
    }

    /// The module a request on `sent` answered on `answer` is: `sent` itself, or for 7DF the answer's physical request ID.
    fn module_of(sent: u16, answer: u16) -> u16 {
        if sent == 0x7DF {
            answer.wrapping_sub(8)
        } else {
            sent
        }
    }

    /// The `rx` a learned entry for `module` answered on `answer` needs: none when it is the usual + 8.
    fn rx_of(module: u16, answer: u16) -> Option<u16> {
        (answer != module.wrapping_add(8)).then_some(answer)
    }

    /// The mark of `mark` (a place in `marks`), used up, if it is still the one naming what is learned next: an item
    /// waiting for its answer takes the mark that was current when it was sent, and only if no other mark came since.
    fn take_mark(&mut self, mark: Option<usize>) -> Option<String> {
        if mark.is_some() && self.naming.is_some() && mark == Some(self.marks.len()) {
            self.naming.take()
        } else {
            None
        }
    }

    /// The mark naming what is learned next, by its place in `marks`, for a request now waiting for its answer.
    fn current_mark(&self) -> Option<usize> {
        self.naming.as_ref().map(|_| self.marks.len())
    }

    /// The adapter was lost and the watch ended early: said in the notes, so the saved session says so.
    pub fn lost(&mut self, why: &str) {
        self.note(format!(
            "the watch ended early: the adapter was lost ({why}); nothing after this point is in the session"
        ));
    }

    /// The person is about to do `label` in the other tool: it goes into the log, and names the next test or setting
    /// learned (one change per mark; what follows it gets the usual "Learned: ..." name). An empty label names nothing.
    pub fn mark(&mut self, label: &str, at: f64) {
        let label = label.trim();
        self.log.push_back(format!("{at:8.3}  -- mark: {label}"));
        self.logged += 1;
        self.marks.push((at, label.to_string()));
        self.naming = (!label.is_empty()).then(|| label.to_string());
    }

    /// The name the next learned item gets: the mark's, else `default`. `take` uses the mark up.
    fn name_for(&mut self, default: String, take: bool) -> String {
        if take {
            return self.naming.take().unwrap_or(default);
        }
        self.naming.clone().unwrap_or(default)
    }

    fn note(&mut self, what: String) {
        if !self.notes.contains(&what) && self.notes.len() < 200 {
            self.log
                .push_back(format!("{:>8}  -- not learned: {what}", ""));
            self.logged += 1;
            self.notes.push(what);
        }
    }

    pub fn add(&mut self, f: &Frame) {
        self.frames += 1;
        self.now = f.at;
        self.expire(|at| f.at - at > ANSWER_WAIT + ANSWER_SLACK);
        let s = self.ids.entry(f.id).or_insert_with(|| IdStats {
            first: f.at,
            ..Default::default()
        });
        if s.changed_at.len() < f.data.len() {
            s.changed_at.resize(f.data.len(), f.at);
        }
        for (i, b) in f.data.iter().enumerate() {
            if s.data.get(i) != Some(b) && s.count > 0 {
                s.changed_at[i] = f.at;
            }
        }
        s.data = f.data.clone();
        s.count += 1;
        s.last = f.at;
        // a key module's frame: counted in the table, never put together, put in words or learned from
        if let Some(k) = self.fence.on_id(f.id) {
            let what = format!(
                "{} is fenced: nothing to or from it is learned or written down here",
                k.describe()
            );
            self.note(what);
            return;
        }
        if !f.ext && (0x700..=0x7FF).contains(&f.id) {
            if let Some(msg) = self.isotp(f) {
                self.diagnostic(f.id, &msg, f.at);
            }
        }
        if f.ext {
            if let Some(line) = self.j1939.as_mut().and_then(|d| d.add(f.id, &f.data, f.at)) {
                self.log.push_back(line);
                self.logged += 1;
                if self.log.len() > 500 {
                    self.log.pop_front();
                }
            }
        }
    }

    fn isotp(&mut self, f: &Frame) -> Option<Vec<u8>> {
        match self.partial.feed(f.id, &f.data) {
            icarus_core::diag::Fed::Message(m) => Some(m),
            _ => None, // part of a message, flow control, or a frame that does not fit
        }
    }

    fn diagnostic(&mut self, id: u32, msg: &[u8], at: f64) {
        let Some(&sid) = msg.first() else { return };
        let answer = sid >= 0x40;
        let arrow = if id == 0x7DF {
            "7DF to every module".to_string()
        } else if answer {
            format!("{id:03X} answer")
        } else {
            format!("{id:03X} request")
        };
        // the VIN and a module's serial number never reach the log (ICA-106): it is shown, saved with a learning session
        // and handed to `icarus mcp`'s learning_sessions, so to a model that may be in the cloud. Learning sees the bytes.
        let (shown, hidden) = icarus_core::moddata::capture::mask_message(msg);
        let mut said = describe(&shown);
        if hidden.vin {
            said += " [VIN masked]";
        }
        if hidden.serial {
            said += " [serial number masked]";
        }
        self.log.push_back(format!("{at:8.3}  {arrow:<22} {said}"));
        self.logged += 1;
        while self.log.len() > 500 {
            self.log.pop_front();
        }
        self.learn(id as u16, msg);
    }

    /// A data identifier another tool read and then changed (the module said yes) becomes a setting: the value before is
    /// "was", each value written "set", "set-2", ... (unchecked, parked conditions). An answer is paired with its request
    /// by `requests_for` (the profiles' answer IDs, else ID - 8).
    fn learn_setting(&mut self, id: u16, msg: &[u8]) {
        match msg {
            [0x10, 0x03] => {
                self.extended.insert(id);
            }
            [0x10, 0x01] => {
                self.extended.remove(&id);
            }
            [0x62, hi, lo, v @ ..] if !v.is_empty() => {
                for tx in self.requests_for(id) {
                    let tx = Self::module_of(tx, id);
                    self.read_values
                        .insert((tx, u16::from_be_bytes([*hi, *lo]), false), v.to_vec());
                }
            }
            [0x61, lid, v @ ..] if !v.is_empty() => {
                for tx in self.requests_for(id) {
                    let tx = Self::module_of(tx, id);
                    self.read_values
                        .insert((tx, *lid as u16, true), v.to_vec());
                }
            }
            // the VIN (2E F190, KWP2000's 3B 90) and a module's serial number (2E F18C) are identity, not a setting: a
            // setting learned from one would carry it into the session and the vehicle's profile
            [0x2E, 0xF1, 0x90 | 0x8C, ..] | [0x3B, 0x90, ..] => self.note(format!(
                "{id:03X}: a write of the VIN or of the module's serial number: identity, never learned"
            )),
            [0x2E, hi, lo, v @ ..] if !v.is_empty() && v.len() <= 16 => {
                self.pending_writes
                    .insert((id, u16::from_be_bytes([*hi, *lo]), false), v.to_vec());
            }
            [0x3B, lid, v @ ..] if !v.is_empty() && v.len() <= 16 => {
                self.pending_writes
                    .insert((id, *lid as u16, true), v.to_vec());
            }
            [0x2E, hi, lo, v @ ..] => self.note(format!(
                "{id:03X}: a write of {} bytes to {hi:02X}{lo:02X}, too long for a setting (coding, or a block of several settings)",
                v.len()
            )),
            [0x3B, lid, v @ ..] => self.note(format!(
                "{id:03X}: a write of {} bytes to local identifier {lid:02X}, too long for a setting (coding, or a block of several settings)",
                v.len()
            )),
            [0x6E, hi, lo, ..] => {
                self.setting_written(id, u16::from_be_bytes([*hi, *lo]), Protocol::Uds)
            }
            [0x7B, lid, ..] => self.setting_written(id, *lid as u16, Protocol::Kwp),
            // 78 is "response pending", not a refusal: the answer is still to come
            [0x7F, sid @ 0x2F..=0x31, 0x78] => self.still_pending(id, *sid),
            [0x7F, sid @ 0x2F..=0x31, code] => self.refused_pending(id, *sid, *code),
            [0x7F, sid @ (0x2E | 0x3B), code] if *code != 0x78 => {
                let kwp = *sid == 0x3B;
                let sent = self.requests_for(id);
                let who = sent
                    .iter()
                    .copied()
                    .find(|&c| self.pending_writes.keys().any(|k| k.0 == c && k.2 == kwp))
                    .map(|c| Self::module_of(c, id))
                    .unwrap_or(sent[0]);
                self.pending_writes
                    .retain(|k, _| !(sent.contains(&k.0) && k.2 == kwp));
                self.note(format!(
                    "{who:03X} refused the tool's {}: {}",
                    service(*sid),
                    crate::uds::nrc(*code)
                ));
            }
            // a routine's start is learned as a service entry (`learn`); its stop and results are only noted
            [0x31, kind @ (0x02 | 0x03), hi, lo, ..] => self.note(format!(
                "{id:03X}: routine {hi:02X}{lo:02X} ({}): part of a service function the tool ran",
                if *kind == 0x02 { "stop" } else { "result" }
            )),
            [0x3D, ..] => self.note(format!("{id:03X}: a write to memory by address: never learned")),
            [0x34 | 0x36, ..] => self.note(format!("{id:03X}: programming (a download to the module): flashing, never learned")),
            [0x2F, hi, lo, p, ..] if *p != 0x00 && *p != 0x03 => self.note(format!(
                "{id:03X}: output control {hi:02X}{lo:02X} with control type {p:02X} (only short-term adjustment, 03, is learned)"
            )),
            _ => {}
        }
    }

    /// The module answering on `answer` took a write of `did`: the write waiting for it becomes a setting.
    fn setting_written(&mut self, answer: u16, did: u16, protocol: Protocol) {
        let kwp = protocol == Protocol::Kwp;
        let Some((sent, tx)) =
            self.waiting_request(answer, |c| self.pending_writes.contains_key(&(c, did, kwp)))
        else {
            return;
        };
        let Some(value) = self.pending_writes.remove(&(sent, did, kwp)) else {
            return;
        };
        let key = (tx, did, kwp);
        let rx = Self::rx_of(tx, answer);
        let before = self
            .read_values
            .get(&key)
            .filter(|b| b.len() == value.len())
            .cloned();
        // what was written is what the module now holds
        self.read_values.insert(key, value.clone());
        let extended = protocol == Protocol::Uds
            && (self.extended.contains(&tx) || self.extended.contains(&sent));
        // the bits the tool changed (ICA-102): not known without a read before the write, nor when nothing changed
        let changed: Option<Vec<u8>> = before
            .as_ref()
            .map(|b| {
                b.iter()
                    .zip(&value)
                    .map(|(x, y)| x ^ y)
                    .collect::<Vec<u8>>()
            })
            .filter(|m| m.iter().any(|&b| b != 0));
        // a setting learned on this identifier before whose bits this write touches: another value of the same setting
        // (its mask grows to the bits seen changing); a write of other bits only is another setting packed in beside it
        let overlaps = |s: &SettingDef| match (&s.mask, &changed) {
            (Some(m), Some(c)) => m.iter().zip(c).any(|(m, c)| m & c != 0),
            _ => true,
        };
        let same_id = |s: &SettingDef| {
            s.module == tx
                && s.rx == rx
                && s.id == did
                && s.protocol == protocol
                && s.values.values().next().map_or(0, Vec::len) == value.len()
        };
        if let Some(i) = self.settings.iter().position(|s| same_id(s) && overlaps(s)) {
            // a setting first learned from a write that changed nothing takes the bits of the first write that does
            let first_bits = self.unmasked.contains(&i) && changed.is_some();
            if first_bits {
                self.unmasked.remove(&i);
            }
            let s = &mut self.settings[i];
            s.extended |= extended;
            match (&mut s.mask, &changed) {
                (Some(m), Some(c)) => {
                    for (m, c) in m.iter_mut().zip(c) {
                        *m |= c;
                    }
                }
                (None, Some(c)) if first_bits => s.mask = Some(c.clone()),
                // without a read before its first write, which bits it covers is not known: the whole value
                _ => {}
            }
            if s.name_of(&value).is_none() {
                let n = (2..)
                    .find(|n| !s.values.contains_key(&format!("set-{n}")))
                    .unwrap_or(2);
                s.values.insert(format!("set-{n}"), value);
            }
            return;
        }
        let beside = self.settings.iter().any(same_id);
        let nothing_changed = before.as_ref() == Some(&value);
        let mut values = BTreeMap::new();
        if let Some(b) = before.filter(|b| *b != value) {
            values.insert("was".to_string(), b);
        }
        values.insert("set".to_string(), value);
        let mut default = format!(
            "Learned: setting {did:0w$X} on {tx:03X}",
            w = if protocol == Protocol::Uds { 4 } else { 2 }
        );
        if let Some(c) = changed.as_ref().filter(|_| beside) {
            default += &format!(", bits {}", hex(c).replace(' ', ""));
        }
        let s = SettingDef {
            name: self.name_for(default, false),
            module: tx,
            rx,
            protocol,
            id: did,
            values,
            mask: changed,
            extended,
            conditions: vec![Condition::EngineOff, Condition::Stationary],
            note: format!("Learned by watching another tool ({}): find out what it changes before relying on it.", crate::report::now()),
            checked: false,
        };
        match s
            .refused()
            .or_else(|| self.fence.refuses(tx.into(), s.rx().into()))
        {
            None => {
                self.name_for(String::new(), true);
                if beside {
                    self.note(format!(
                        "{tx:03X} {did:X}: the tool changed other bits of an identifier a setting was learned on, so \
                         \"{}\" is learned as a setting of its own beside it (its own mask); if they are one setting, \
                         give them one entry",
                        s.name
                    ));
                }
                if nothing_changed {
                    self.unmasked.insert(self.settings.len());
                }
                self.settings.push(s);
            }
            Some(why) => self.note(format!("{tx:03X} {did:X}: {why}")),
        }
    }

    /// Security access (ICA-101): a module the tool sends `27` to counts as unlocked until it says it is back in its
    /// default session (UDS `50 01`, KWP2000 `50 81`) or was reset (`51`). Returns whether `msg`, sent to `tx`, is a write,
    /// routine or output test the tool sent to an unlocked module: those are noted, never learned (they may need access
    /// Icarus never uses).
    fn after_unlock(&mut self, tx: u16, msg: &[u8]) -> bool {
        match msg {
            [0x27, level, ..] => {
                // a write sent before the unlock and still waiting for its answer is not learned from an answer that
                // comes after it (that answer could be to a write sent unlocked); a routine start the same
                self.pending_writes.retain(|k, _| k.0 != tx);
                self.pending_routines.retain(|k, _| k.0 != tx);
                self.pending_tests.retain(|k, _| k.0 != tx);
                if self.unlocked.insert(tx) {
                    self.note(format!(
                        "{tx:03X}: security access (level {level:02X}): the tool unlocked the module. Icarus never does \
                         this, so no write, routine or output test the tool sends {tx:03X} from here on is learned, until \
                         the module is back in its default session"
                    ));
                }
                return false;
            }
            [0x50, 0x01 | 0x81, ..] | [0x51, ..] => {
                for module in self.requests_for(tx) {
                    if self.unlocked.remove(&module) {
                        self.note(format!(
                            "{module:03X} is back in its default session (locked again): what the tool does on it is \
                             learned again"
                        ));
                    }
                }
                return false;
            }
            // the module's answer to a write the tool sent it unlocked: nothing to learn from either (the write was noted)
            [0x6E | 0x7B | 0x71 | 0x6F | 0x70, ..]
                if self
                    .requests_for(tx)
                    .iter()
                    .any(|c| self.unlocked.contains(c)) =>
            {
                return true
            }
            _ => {}
        }
        if !self.unlocked.contains(&tx) {
            return false;
        }
        // what an unlocked write leaves in the module is not known (it may or may not have taken): no later write is
        // learned with the value read before the unlock as its "was"
        match msg {
            [0x2E, hi, lo, ..] => {
                self.read_values
                    .remove(&(tx, u16::from_be_bytes([*hi, *lo]), false));
            }
            [0x3B, lid, ..] => {
                self.read_values.remove(&(tx, *lid as u16, true));
            }
            _ => {}
        }
        let what = match msg {
            [0x2E, hi, lo, ..] => format!("a write to {hi:02X}{lo:02X}"),
            [0x3B, lid, ..] => format!("a write to local identifier {lid:02X}"),
            [0x31, 0x01, hi, lo, ..] => format!("routine {hi:02X}{lo:02X}"),
            [0x2F, hi, lo, 0x03, ..] => format!("output control {hi:02X}{lo:02X}"),
            [0x30, lid, 0x07, ..] => format!("output control {lid:02X}"),
            _ => return false,
        };
        self.note(format!(
            "{tx:03X}: {what} after security access (27): it may need access Icarus never uses, so it is not learned"
        ));
        true
    }

    /// The module answering on `answer` said "response pending" (`7F sid 78`): the requests of that service waiting on it
    /// wait on from now.
    fn still_pending(&mut self, answer: u16, sid: u8) {
        let now = self.now;
        let sent = self.requests_for(answer);
        match sid {
            0x31 => self
                .pending_routines
                .iter_mut()
                .filter(|(k, _)| sent.contains(&k.0))
                .for_each(|(_, w)| w.at = now),
            _ => self
                .pending_tests
                .iter_mut()
                .filter(|(k, _)| sent.contains(&k.0) && k.2 == (sid == 0x30))
                .for_each(|(_, w)| w.at = now),
        }
    }

    /// The module answering on `answer` refused the tool's routine control (`7F 31 xx`) or output control (`7F 2F xx`,
    /// `7F 30 xx`): what of that service was waiting on it is noted with the refusal in words, not learned.
    fn refused_pending(&mut self, answer: u16, sid: u8, code: u8) {
        let sent = self.requests_for(answer);
        let (mut what, first): (Vec<String>, Option<u16>) = match sid {
            0x31 => {
                let keys: Vec<(u16, u16)> = self
                    .pending_routines
                    .keys()
                    .filter(|k| sent.contains(&k.0))
                    .copied()
                    .collect();
                self.pending_routines.retain(|k, _| !sent.contains(&k.0));
                (
                    keys.iter()
                        .map(|k| format!("start of routine {:04X}", k.1))
                        .collect(),
                    keys.first().map(|k| k.0),
                )
            }
            _ => {
                let kwp = sid == 0x30;
                let mut first = None;
                let w = self
                    .pending_tests
                    .iter()
                    .filter(|(k, _)| sent.contains(&k.0) && k.2 == kwp)
                    .map(|(k, w)| {
                        first = Some(k.0);
                        format!(
                            "output control {:0n$X} (to {})",
                            k.1,
                            hex(&w.def.on),
                            n = if kwp { 2 } else { 4 }
                        )
                    })
                    .collect();
                self.pending_tests
                    .retain(|k, _| !(sent.contains(&k.0) && k.2 == kwp));
                (w, first)
            }
        };
        what.sort();
        let who = first.map(|c| Self::module_of(c, answer)).unwrap_or(sent[0]);
        let what = if what.is_empty() {
            service(sid).to_string()
        } else {
            what.join(", ")
        };
        self.note(format!(
            "{who:03X} refused the tool's {what}: {}",
            crate::uds::nrc(code)
        ));
    }

    /// Requests waiting for an answer for which `gone(sent)` holds are noted as unanswered and dropped.
    fn expire(&mut self, gone: impl Fn(f64) -> bool) {
        if self.pending_routines.is_empty() && self.pending_tests.is_empty() {
            return;
        }
        let mut what: Vec<String> = Vec::new();
        self.pending_routines.retain(|(tx, r), w| {
            let keep = !gone(w.at);
            if !keep {
                what.push(format!("{tx:03X}: routine {r:04X} started"));
            }
            keep
        });
        self.pending_tests.retain(|(tx, out, kwp), w| {
            let keep = !gone(w.at);
            if !keep {
                what.push(format!(
                    "{tx:03X}: output control {out:0n$X} (to {})",
                    hex(&w.def.on),
                    n = if *kwp { 2 } else { 4 }
                ));
            }
            keep
        });
        what.sort();
        for w in what {
            self.note(format!("{w}: no answer seen, not learned"));
        }
    }

    /// The watch has ended: what still waits for an answer is noted as unanswered (call before `learning::session_text`).
    pub fn finish(&mut self) {
        self.expire(|_| true);
    }

    /// The module answering on `answer` said it started `routine` (`71 01`): the start the tool sent it, if one is waiting,
    /// becomes a service entry for that module (with its answer ID when it is not the usual + 8), checked again against the
    /// airbag modules and the fence on both IDs. A start the module refused (`7F 31`) is noted, not learned; one still
    /// pending (`7F 31 78`) waits.
    fn routine_started(&mut self, answer: u16, routine: u16) {
        let Some((sent, tx)) = self.waiting_request(answer, |c| {
            self.pending_routines.contains_key(&(c, routine))
        }) else {
            return;
        };
        let Some(w) = self.pending_routines.remove(&(sent, routine)) else {
            return;
        };
        if self
            .services
            .iter()
            .any(|x| x.module == tx && x.routine == routine)
        {
            return;
        }
        let mut s = w.def;
        s.module = tx;
        s.rx = Self::rx_of(tx, answer);
        if let Some(name) = self.take_mark(w.mark) {
            s.name = name;
        } else if sent != tx {
            s.name = format!("Learned: routine {routine:04X} on {tx:03X}");
        }
        match s
            .refused()
            .or_else(|| self.fence.refuses(tx.into(), s.rx().into()))
        {
            None => self.services.push(s),
            Some(why) => self.note(format!("{tx:03X} routine {routine:04X}: {why}")),
        }
    }

    /// What another tool sent or a module answered: an output test or a routine start waits for its answer, a setting for
    /// its write's (the most cautious conditions, unchecked).
    fn learn(&mut self, tx: u16, msg: &[u8]) {
        if self.after_unlock(tx, msg) {
            return;
        }
        self.learn_setting(tx, msg);
        // a routine another tool started: kept until the module answers; a service entry, unchecked, with the most
        // cautious conditions, once it says it started (`routine_started`)
        if let [0x31, 0x01, hi, lo, params @ ..] = msg {
            let routine = u16::from_be_bytes([*hi, *lo]);
            if params.len() > 8 {
                self.note(format!(
                    "{tx:03X}: routine {routine:04X} started with {} bytes of parameters, too long to learn (a service \
                     entry takes at most 8)",
                    params.len()
                ));
            } else if !self
                .services
                .iter()
                .any(|s| s.module == tx && s.routine == routine)
            {
                let default = format!("Learned: routine {routine:04X} on {tx:03X}");
                let mut s = crate::service::ServiceDef {
                    // the name it would get now, for the airbag check; it is named when the module answers
                    name: self.name_for(default.clone(), false),
                    module: tx,
                    rx: None,
                    routine,
                    params: params.to_vec(),
                    extended: true,
                    results: true,
                    conditions: vec![Condition::EngineOff, Condition::Stationary],
                    max_seconds: 30,
                    // what an unknown routine writes is not known (it could be coding): a cable (ICA-220)
                    wired: true,
                    warning: format!(
                        "Learned by watching another tool ({}): find out what it does before relying on it.",
                        crate::report::now()
                    ),
                    checked: false,
                };
                // a request to every module (7DF) is checked once its answer says which module took it
                let why = (tx != 0x7DF)
                    .then(|| {
                        s.refused()
                            .or_else(|| self.fence.refuses(tx.into(), s.rx().into()))
                    })
                    .flatten();
                match why {
                    None => {
                        s.name = default;
                        let w = Waiting {
                            def: s,
                            at: self.now,
                            mark: self.current_mark(),
                        };
                        self.pending_routines.insert((tx, routine), w);
                    }
                    Some(why) => self.note(format!("{tx:03X} routine {routine:04X}: {why}")),
                }
            }
            return;
        }
        if let [0x71, 0x01, hi, lo, ..] = msg {
            self.routine_started(tx, u16::from_be_bytes([*hi, *lo]));
            return;
        }
        // an output test another tool ran: kept until the module answers, learned when it takes it (`test_accepted`)
        match msg {
            [0x6F, hi, lo, 0x03, ..] => {
                return self.test_accepted(tx, u16::from_be_bytes([*hi, *lo]), false)
            }
            [0x70, lid, 0x07, ..] => return self.test_accepted(tx, *lid as u16, true),
            _ => {}
        }
        let (protocol, out, state) = match msg {
            [0x2F, hi, lo, 0x03, state @ ..] if !state.is_empty() && state.len() <= 4 => {
                (Protocol::Uds, u16::from_be_bytes([*hi, *lo]), state)
            }
            [0x30, lid, 0x07, state @ ..] if !state.is_empty() && state.len() <= 4 => {
                (Protocol::Kwp, *lid as u16, state)
            }
            _ => return,
        };
        let default = Self::test_name(out, tx, protocol);
        let mut t = TestDef {
            // the name it would get now, for the airbag check; it is named when the module answers
            name: self.name_for(default.clone(), false),
            module: tx,
            rx: None,
            protocol,
            id: out,
            control: Control::Onoff,
            on: state.to_vec(),
            scale: 2.55,
            conditions: vec![Condition::EngineOff, Condition::Stationary],
            max_seconds: 10,
            warning: format!("Learned by watching another tool ({}): find out what it drives before relying on it.", crate::report::now()),
            watch: None,
            checked: false,
        };
        // a request to every module (7DF) is checked once its answer says which module took it
        let why = (tx != 0x7DF)
            .then(|| {
                t.refused()
                    .or_else(|| self.fence.refuses(tx.into(), t.rx().into()))
            })
            .flatten();
        match why {
            None => {
                t.name = default;
                let w = Waiting {
                    def: t,
                    at: self.now,
                    mark: self.current_mark(),
                };
                self.pending_tests
                    .insert((tx, out, protocol == Protocol::Kwp), w);
            }
            Some(why) => self.note(format!("{tx:03X} output {out:X}: {why}")),
        }
    }

    fn test_name(out: u16, tx: u16, protocol: Protocol) -> String {
        format!(
            "Learned: output {out:0w$X} on {tx:03X}",
            w = if protocol == Protocol::Uds { 4 } else { 2 }
        )
    }

    /// The module answering on `answer` took the tool's output control of `out` (`6F .. 03`, `70 .. 07`): the request
    /// waiting for it becomes a test for that module (with its answer ID when it is not the usual + 8), checked again
    /// against the airbag modules and the fence on both IDs, or, on a test learned before, another setting of it.
    fn test_accepted(&mut self, answer: u16, out: u16, kwp: bool) {
        let Some((sent, tx)) =
            self.waiting_request(answer, |c| self.pending_tests.contains_key(&(c, out, kwp)))
        else {
            return;
        };
        let Some(w) = self.pending_tests.remove(&(sent, out, kwp)) else {
            return;
        };
        let mut t = w.def;
        if let Some(l) = self
            .learned
            .iter_mut()
            .find(|l| l.module == tx && l.id == out && l.protocol == t.protocol)
        {
            // several different one-byte settings: a setting with a range rather than on/off
            if t.on.len() == 1 && l.on.len() == 1 && l.on[0] != t.on[0] {
                l.control = Control::Percent;
                l.warning = format!(
                    "{} Seen with several settings; check its scaling (now: percent x {}).",
                    l.warning.split(" Seen with").next().unwrap_or(""),
                    l.scale
                );
            }
            return;
        }
        t.module = tx;
        t.rx = Self::rx_of(tx, answer);
        if let Some(name) = self.take_mark(w.mark) {
            t.name = name;
        } else if sent != tx {
            t.name = Self::test_name(out, tx, t.protocol);
        }
        match t
            .refused()
            .or_else(|| self.fence.refuses(tx.into(), t.rx().into()))
        {
            None => self.learned.push(t),
            Some(why) => self.note(format!("{tx:03X} output {out:X}: {why}")),
        }
    }
}

/// A `[[test]]` entry for a vehicle profile.
pub fn to_toml(t: &TestDef) -> String {
    let mut s = String::from("[[test]]\n");
    s += &format!("name = {:?}\nmodule = 0x{:03X}\n", t.name, t.module);
    if let Some(rx) = t.rx {
        s += &format!("rx = 0x{rx:03X}\n");
    }
    s += &format!(
        "protocol = \"{}\"\nid = 0x{:X}\n",
        if t.protocol == Protocol::Uds {
            "uds"
        } else {
            "kwp"
        },
        t.id
    );
    s += &format!(
        "control = \"{}\"\n",
        if t.control == Control::Percent {
            "percent"
        } else {
            "onoff"
        }
    );
    if t.control == Control::Percent {
        s += &format!("scale = {}\n", t.scale);
    } else {
        s += &format!(
            "on = [{}]\n",
            t.on.iter()
                .map(|b| format!("0x{b:02X}"))
                .collect::<Vec<_>>()
                .join(", ")
        );
    }
    let conds: Vec<&str> = t
        .conditions
        .iter()
        .map(|c| match c {
            Condition::EngineOff => "\"engine-off\"",
            Condition::EngineRunning => "\"engine-running\"",
            Condition::Stationary => "\"stationary\"",
        })
        .collect();
    s += &format!(
        "conditions = [{}]\nmax_seconds = {}\nwarning = {:?}\n",
        conds.join(", "),
        t.max_seconds,
        t.warning
    );
    if let Some(w) = t.watch {
        s += &format!("watch = 0x{w:02X}\n");
    }
    s += &format!("checked = {}\n", t.checked);
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    fn f(id: u32, data: &[u8], at: f64) -> Frame {
        Frame {
            id,
            ext: false,
            data: data.to_vec(),
            at,
        }
    }

    #[test]
    fn lines_from_the_adapter() {
        assert_eq!(
            parse_line("2C4 03 1A F8", Bus::Can11At500, 1.0).unwrap(),
            f(0x2C4, &[0x03, 0x1A, 0xF8], 1.0)
        );
        let e = parse_line("18 DA F1 10 03 41 0D 00", Bus::Can29At500, 0.0).unwrap();
        assert_eq!((e.id, e.ext, e.data.len()), (0x18DAF110, true, 4));
        for bad in [
            "BUFFER FULL",
            "<DATA ERROR",
            "2C4 03 1A F8 00 00 00 00 00 00",
            "12 34",
            "",
        ] {
            assert!(parse_line(bad, Bus::Can11At500, 0.0).is_none(), "{bad}");
        }
    }

    #[test]
    fn the_table_marks_changing_bytes() {
        let mut a = Analyzer::default();
        a.add(&f(0x2C4, &[0x03, 0x10, 0x00], 0.0));
        a.add(&f(0x2C4, &[0x03, 0x14, 0x00], 0.5));
        a.add(&f(0x2C4, &[0x03, 0x14, 0x00], 1.0));
        let s = &a.ids[&0x2C4];
        assert_eq!((s.count, s.hz()), (3, 2.0));
        assert_eq!(s.changed_at, vec![0.0, 0.5, 0.0], "byte 1 changed at 0.5 s");
    }

    /// ICA-200: a setting, a routine and an output test another tool ran on a module a GM car's profile does not name
    /// (7A4; the learner does not know the make) are learned as entries, but on GM they stay unusable: not added to the
    /// user's GM file, left out when a GM file holding them loads, and refused at the moment of the write. The same entries
    /// on Toyota's cluster (7C0, which Toyota's profile names beside its airbag module) load as before.
    #[test]
    fn what_is_learned_on_a_module_of_unknown_kind_stays_unusable_where_the_profile_names_no_airbag_module(
    ) {
        let mut a = Analyzer::default();
        let convo: &[(u32, &[u8])] = &[
            (0x7A4, &[0x03, 0x22, 0xA0, 0x01]),
            (0x7AC, &[0x04, 0x62, 0xA0, 0x01, 0x01]),
            (0x7A4, &[0x04, 0x2E, 0xA0, 0x01, 0x00]),
            (0x7AC, &[0x03, 0x6E, 0xA0, 0x01]),
            (0x7A4, &[0x04, 0x31, 0x01, 0x02, 0x03]),
            (0x7AC, &[0x04, 0x71, 0x01, 0x02, 0x03]),
            (0x7A4, &[0x05, 0x2F, 0x00, 0x10, 0x03, 0x01]),
            (0x7AC, &[0x04, 0x6F, 0x00, 0x10, 0x03]),
        ];
        for (i, (id, d)) in convo.iter().enumerate() {
            a.add(&f(*id, d, i as f64));
        }
        assert_eq!(
            (a.settings.len(), a.services.len(), a.learned.len()),
            (1, 1, 1),
            "{:?}",
            a.log
        );
        let entries = [
            crate::settings::to_toml(&a.settings[0]),
            crate::service::to_toml(&a.services[0]),
            to_toml(&a.learned[0]),
        ];
        let dir = std::env::temp_dir().join(format!("icarus-bus-ica200-{}", std::process::id()));
        for e in &entries {
            let why = crate::vehicles::add_test_to_user_file("gm", e).unwrap_err();
            assert!(why.contains("could be the airbag module"), "{why}");
            // a new make's file holding it, as a person might paste it: left out when it loads
            let _ = std::fs::remove_dir_all(&dir);
            std::fs::create_dir_all(&dir).unwrap();
            std::fs::write(
                dir.join("kart.toml"),
                format!("name = \"Kart\"\nwmi = [\"ZZK\"]\n{e}"),
            )
            .unwrap();
            let (all, problems) = crate::vehicles::load(Some(&dir));
            let kart = crate::vehicles::find(&all, "kart").unwrap();
            assert!(
                kart.tests.is_empty() && kart.settings.is_empty() && kart.services.is_empty(),
                "{e}"
            );
            assert!(
                problems
                    .iter()
                    .any(|p| p.contains("could be the airbag module") && p.ends_with("(left out)")),
                "{problems:?}"
            );
            // on Toyota's cluster, the same entry loads
            let on_cluster = e.replace("module = 0x7A4", "module = 0x7C0");
            std::fs::write(dir.join("kart.toml"), "name = \"Kart\"\nwmi = [\"ZZK\"]\n").unwrap();
            std::fs::write(dir.join("toyota.toml"), &on_cluster).unwrap();
            let (all, problems) = crate::vehicles::load(Some(&dir));
            let t = crate::vehicles::find(&all, "toyota").unwrap();
            assert_eq!(
                t.tests.len() + t.settings.len() + t.services.len(),
                1,
                "{on_cluster} {problems:?}"
            );
            std::fs::remove_file(dir.join("toyota.toml")).unwrap();
        }
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_setting_the_module_accepted_is_learned() {
        let mut a = Analyzer::default();
        let convo: &[(u32, &[u8])] = &[
            (0x7C0, &[0x02, 0x10, 0x03]),
            (0x7C8, &[0x02, 0x50, 0x03]),
            (0x7C0, &[0x03, 0x22, 0xA0, 0x01]),
            (0x7C8, &[0x04, 0x62, 0xA0, 0x01, 0x01]),
            (0x7C0, &[0x04, 0x2E, 0xA0, 0x01, 0x00]),
            (0x7C8, &[0x03, 0x6E, 0xA0, 0x01]),
            // a second write that changes the same bit and the one beside it: the same setting, its mask grows
            (0x7C0, &[0x04, 0x2E, 0xA0, 0x01, 0x03]),
            (0x7C8, &[0x03, 0x6E, 0xA0, 0x01]),
            // refused: not learned
            (0x7B0, &[0x03, 0x3B, 0x30, 0x01]),
            (0x7B8, &[0x03, 0x7F, 0x3B, 0x22]),
            // the airbag module: never learned
            (0x780, &[0x04, 0x2E, 0x12, 0x34, 0x01]),
            (0x788, &[0x03, 0x6E, 0x12, 0x34]),
        ];
        for (i, (id, d)) in convo.iter().enumerate() {
            a.add(&f(*id, d, i as f64));
        }
        assert!(
            a.log[4].contains("write data by identifier: A001 to 00"),
            "{:?}",
            a.log
        );
        assert_eq!(a.settings.len(), 1, "{:?}", a.settings);
        let s = &a.settings[0];
        assert_eq!(
            (s.module, s.id, s.protocol, s.extended, s.checked),
            (0x7C0, 0xA001, Protocol::Uds, true, false)
        );
        let v: Vec<(&str, &[u8])> = s
            .values
            .iter()
            .map(|(k, v)| (k.as_str(), v.as_slice()))
            .collect();
        assert_eq!(
            v,
            vec![
                ("set", &[0x00][..]),
                ("set-2", &[0x03][..]),
                ("was", &[0x01][..])
            ]
        );
        assert_eq!(
            s.mask,
            Some(vec![0x03]),
            "01 to 00 changed bit 0, 00 to 03 bits 0 and 1"
        );
        let entry = crate::settings::to_toml(s);
        let p = crate::vehicles::parse("x", "test", &format!("name = \"x\"\n{entry}")).unwrap();
        assert_eq!(&p.settings[0], s, "{entry}");
    }

    #[test]
    fn marks_name_what_is_learned_and_the_rest_is_noted() {
        let mut a = Analyzer::default();
        a.mark("Seat belt chime off", 1.0);
        let convo: &[(u32, &[u8])] = &[
            (0x7C0, &[0x04, 0x2E, 0xA0, 0x01, 0x00]),
            (0x7C8, &[0x03, 0x7F, 0x2E, 0x78]), // pending, then yes
            (0x7C8, &[0x03, 0x6E, 0xA0, 0x01]),
            (0x7C0, &[0x04, 0x2E, 0xA0, 0x02, 0x01]),
            (0x7C8, &[0x03, 0x6E, 0xA0, 0x02]),
            (0x7E0, &[0x04, 0x31, 0x01, 0xFF, 0x00]),
            (0x7E8, &[0x04, 0x71, 0x01, 0xFF, 0x00]),
            (0x7E0, &[0x02, 0x27, 0x01]),
            (0x7B0, &[0x03, 0x3B, 0x30, 0x01]),
            (0x7B8, &[0x03, 0x7F, 0x3B, 0x22]),
        ];
        for (i, (id, d)) in convo.iter().enumerate() {
            a.add(&f(*id, d, 2.0 + i as f64));
        }
        a.mark("", 20.0);
        a.add(&f(0x7C0, &[0x04, 0x2E, 0xA0, 0x03, 0x01], 21.0));
        a.add(&f(0x7C8, &[0x03, 0x6E, 0xA0, 0x03], 21.1));
        let names: Vec<&str> = a.settings.iter().map(|s| s.name.as_str()).collect();
        assert_eq!(
            names,
            [
                "Seat belt chime off",
                "Learned: setting A002 on 7C0",
                "Learned: setting A003 on 7C0"
            ]
        );
        assert!(a.log[0].contains("mark: Seat belt chime off"));
        assert_eq!(a.notes.len(), 2, "{:?}", a.notes);
        assert!(a.notes[0].contains("security access"));
        assert!(
            a.notes[1].contains("7B0 refused the tool's write data (KWP2000)"),
            "{:?}",
            a.notes
        );
        // the routine the tool started (before any security access) is learned as a service entry, unchecked, with the
        // most cautious conditions
        assert_eq!(a.services.len(), 1);
        let r = &a.services[0];
        assert_eq!(
            (r.module, r.routine, r.name.as_str(), r.checked),
            (0x7E0, 0xFF00, "Learned: routine FF00 on 7E0", false)
        );
        assert!(r.conditions.contains(&Condition::EngineOff) && r.refused().is_none());
        assert_eq!(
            a.marks,
            vec![
                (1.0, "Seat belt chime off".to_string()),
                (20.0, String::new())
            ]
        );
        // a mark that names a refused kind of setting does not get it learned
        let mut b = Analyzer::default();
        b.mark("immobilizer: add a key", 0.0);
        b.add(&f(0x750, &[0x04, 0x2E, 0x10, 0x01, 0x01], 1.0));
        b.add(&f(0x758, &[0x03, 0x6E, 0x10, 0x01], 1.1));
        assert!(
            b.settings.is_empty() && b.notes[0].contains("immobilizer"),
            "{:?}",
            b.notes
        );
    }

    /// A key registration as a dealer tool might run it (ICA-101): the extended session, security access (27 01 seed,
    /// 27 02 key), then a routine, a write and an output test. On a simulated smart-key module (fenced by address, its
    /// name innocent) and on a module of no kind at all, nothing is learned, and the notes say why.
    fn key_registration(module: u32) -> Vec<(u32, Vec<u8>)> {
        let answer = module + 8;
        vec![
            (module, vec![0x02, 0x10, 0x03]),
            (answer, vec![0x02, 0x50, 0x03]),
            (module, vec![0x02, 0x27, 0x01]),
            (answer, vec![0x04, 0x67, 0x01, 0x12, 0x34]),
            (module, vec![0x04, 0x27, 0x02, 0x56, 0x78]),
            (answer, vec![0x02, 0x67, 0x02]),
            (module, vec![0x04, 0x31, 0x01, 0x12, 0x34]),
            (answer, vec![0x04, 0x71, 0x01, 0x12, 0x34]),
            (module, vec![0x03, 0x22, 0x10, 0x01]),
            (answer, vec![0x04, 0x62, 0x10, 0x01, 0x00]),
            (module, vec![0x04, 0x2E, 0x10, 0x01, 0x01]),
            (answer, vec![0x03, 0x6E, 0x10, 0x01]),
            (module, vec![0x05, 0x2F, 0x20, 0x01, 0x03, 0x01]),
            (answer, vec![0x05, 0x6F, 0x20, 0x01, 0x03, 0x01]),
        ]
    }

    fn watch(a: &mut Analyzer, frames: &[(u32, Vec<u8>)], from: f64) {
        for (i, (id, d)) in frames.iter().enumerate() {
            a.add(&f(*id, d, from + i as f64));
        }
    }

    #[test]
    fn nothing_is_learned_from_a_key_module_or_after_security_access() {
        // a smart-key module, by address: its name says nothing about keys
        let fence = crate::fence::one_for_test(0x7D5, crate::fence::Kind::SmartKey);
        let mut a = Analyzer::watching(fence.clone());
        a.mark("Door lock buzzer", 0.0);
        watch(&mut a, &key_registration(0x7D5), 1.0);
        assert!(
            a.services.is_empty() && a.settings.is_empty() && a.learned.is_empty(),
            "{:?} {:?} {:?}",
            a.services,
            a.settings,
            a.learned
        );
        assert_eq!(a.notes.len(), 1, "{:?}", a.notes);
        assert!(
            a.notes[0].contains("7D5 (the smart-key module) is fenced"),
            "{:?}",
            a.notes
        );
        // its messages are not put in words: the log has the mark and the note, nothing the tool or the module said
        assert!(
            a.log
                .iter()
                .all(|l| !l.contains("request") && !l.contains("answer") && !l.contains("1234")),
            "{:?}",
            a.log
        );
        assert_eq!((a.frames, a.ids.len()), (14, 2), "counted in the table");
        // a module of no kind at all: the same session teaches nothing either, and each thing is noted
        let mut b = Analyzer::watching(fence.clone());
        watch(&mut b, &key_registration(0x7A4), 1.0);
        assert!(
            b.services.is_empty() && b.settings.is_empty() && b.learned.is_empty(),
            "{:?} {:?} {:?}",
            b.services,
            b.settings,
            b.learned
        );
        for why in [
            "7A4: security access (level 01)",
            "7A4: routine 1234 after security access",
            "7A4: a write to 1001 after security access",
            "7A4: output control 2001 after security access",
        ] {
            assert!(
                b.notes.iter().any(|n| n.contains(why)),
                "{why}: {:?}",
                b.notes
            );
        }
        assert_eq!(b.notes.len(), 4, "27 02 is the same unlock: {:?}", b.notes);
        // the same routine with no 27 before it is learned as before
        let mut c = Analyzer::watching(fence.clone());
        let plain: Vec<(u32, Vec<u8>)> = key_registration(0x7A4)
            .into_iter()
            .filter(|(_, d)| d[1] != 0x27 && d[1] != 0x67)
            .collect();
        watch(&mut c, &plain, 1.0);
        assert_eq!(
            c.services
                .iter()
                .map(|s| (s.module, s.routine, s.name.as_str()))
                .collect::<Vec<_>>(),
            [(0x7A4, 0x1234, "Learned: routine 1234 on 7A4")]
        );
        assert_eq!((c.settings.len(), c.learned.len()), (1, 1), "{:?}", c.notes);
        // once the module says it is back in its default session it is locked again, and learned from again
        let mut d = Analyzer::watching(fence);
        watch(&mut d, &key_registration(0x7A4), 1.0);
        watch(
            &mut d,
            &[
                (0x7A4, vec![0x02, 0x10, 0x01]),
                (0x7AC, vec![0x02, 0x50, 0x01]),
                (0x7A4, vec![0x04, 0x31, 0x01, 0x56, 0x78]),
                (0x7AC, vec![0x04, 0x71, 0x01, 0x56, 0x78]),
            ],
            20.0,
        );
        assert_eq!(
            d.services.iter().map(|s| s.routine).collect::<Vec<_>>(),
            [0x5678],
            "{:?}",
            d.notes
        );
        assert!(d
            .notes
            .iter()
            .any(|n| n.contains("7A4 is back in its default session")));
    }

    /// ICA-102: a write that changes some bits of an identifier is learned with those bits as its mask; a later write that
    /// changes only other bits of it is another setting packed beside it, with its own mask and name.
    #[test]
    fn a_learned_setting_carries_the_bits_the_tool_changed() {
        let mut a = Analyzer::default();
        watch(
            &mut a,
            &[
                (0x7C0, vec![0x02, 0x10, 0x03]),
                (0x7C8, vec![0x02, 0x50, 0x03]),
                (0x7C0, vec![0x03, 0x22, 0xA0, 0x02]),
                (0x7C8, vec![0x04, 0x62, 0xA0, 0x02, 0x05]),
                // the auto door lock off: bit 0
                (0x7C0, vec![0x04, 0x2E, 0xA0, 0x02, 0x04]),
                (0x7C8, vec![0x03, 0x6E, 0xA0, 0x02]),
                // the interior light's fade long: bits 2 and 3 (01 to 10)
                (0x7C0, vec![0x04, 0x2E, 0xA0, 0x02, 0x08]),
                (0x7C8, vec![0x03, 0x6E, 0xA0, 0x02]),
            ],
            0.0,
        );
        assert_eq!(a.settings.len(), 2, "{:?}", a.settings);
        let (lock, fade) = (&a.settings[0], &a.settings[1]);
        assert_eq!(lock.mask, Some(vec![0x01]));
        assert_eq!(
            (lock.values["was"].clone(), lock.values["set"].clone()),
            (vec![0x05], vec![0x04])
        );
        assert_eq!(fade.mask, Some(vec![0x0C]));
        assert_eq!(fade.name, "Learned: setting A002 on 7C0, bits 0C");
        assert_eq!(
            (fade.values["was"].clone(), fade.values["set"].clone()),
            (vec![0x04], vec![0x08])
        );
        assert!(
            a.notes
                .iter()
                .any(|n| n.contains("a setting of its own beside it")),
            "{:?}",
            a.notes
        );
        // the entries round-trip through a profile, the masks with them
        for s in &a.settings {
            let entry = crate::settings::to_toml(s);
            assert!(entry.contains("mask = ["), "{entry}");
            let p = crate::vehicles::parse("x", "test", &format!("name = \"x\"\n{entry}")).unwrap();
            assert_eq!(&p.settings[0], s, "{entry}");
        }
    }

    /// A write sent before the unlock whose answer never came (the refusal dropped by the monitor, or the module silent),
    /// then security access and the same write again: the module's answer to the unlocked write learns nothing, and
    /// neither does a later answer while the module is unlocked. Once it is back in its default session a write is learned
    /// again, but not with the value read before the unlock as its "was": what the unlocked write left is not known.
    #[test]
    fn a_write_waiting_before_an_unlock_is_not_learned_from_after_it() {
        let mut a = Analyzer::default();
        watch(
            &mut a,
            &[
                (0x7C0, vec![0x03, 0x22, 0xA0, 0x01]),
                (0x7C8, vec![0x04, 0x62, 0xA0, 0x01, 0x01]),
                // its answer (7F 2E 33) is not seen
                (0x7C0, vec![0x04, 0x2E, 0xA0, 0x01, 0x00]),
                (0x7C0, vec![0x02, 0x27, 0x01]),
                (0x7C8, vec![0x04, 0x67, 0x01, 0x12, 0x34]),
                (0x7C0, vec![0x04, 0x27, 0x02, 0x56, 0x78]),
                (0x7C8, vec![0x02, 0x67, 0x02]),
                (0x7C0, vec![0x04, 0x2E, 0xA0, 0x01, 0x00]),
                (0x7C8, vec![0x03, 0x6E, 0xA0, 0x01]),
                (0x7C8, vec![0x03, 0x6E, 0xA0, 0x01]),
            ],
            0.0,
        );
        assert!(a.settings.is_empty(), "{:?} {:?}", a.settings, a.notes);
        assert!(
            a.notes
                .iter()
                .any(|n| n.contains("7C0: a write to A001 after security access")),
            "{:?}",
            a.notes
        );
        // locked again: a write is learned again, with no "was" (nothing was read since the unlocked write)
        watch(
            &mut a,
            &[
                (0x7C0, vec![0x02, 0x10, 0x01]),
                (0x7C8, vec![0x02, 0x50, 0x01]),
                (0x7C0, vec![0x04, 0x2E, 0xA0, 0x01, 0x01]),
                (0x7C8, vec![0x03, 0x6E, 0xA0, 0x01]),
            ],
            20.0,
        );
        assert_eq!(a.settings.len(), 1, "{:?}", a.notes);
        let s = &a.settings[0];
        assert!(
            !s.values.contains_key("was") && s.mask.is_none(),
            "no read since the unlocked write: the value before is not known ({s:?})"
        );
    }

    /// ICA-102: a first write that changes nothing (the tool wrote the value the module had) gives no mask yet; the next
    /// write that changes known bits gives the setting those bits, so it never writes the whole value. A setting whose
    /// first write had no read before it stays a whole-value one: which bits that write changed is not known.
    #[test]
    fn a_setting_first_written_unchanged_takes_the_bits_of_the_next_write() {
        let mut a = Analyzer::default();
        watch(
            &mut a,
            &[
                (0x7C0, vec![0x03, 0x22, 0xA0, 0x02]),
                (0x7C8, vec![0x04, 0x62, 0xA0, 0x02, 0x05]),
                // written as it was
                (0x7C0, vec![0x04, 0x2E, 0xA0, 0x02, 0x05]),
                (0x7C8, vec![0x03, 0x6E, 0xA0, 0x02]),
                // bit 0
                (0x7C0, vec![0x04, 0x2E, 0xA0, 0x02, 0x04]),
                (0x7C8, vec![0x03, 0x6E, 0xA0, 0x02]),
                // bits 2 and 3: another setting beside it
                (0x7C0, vec![0x04, 0x2E, 0xA0, 0x02, 0x08]),
                (0x7C8, vec![0x03, 0x6E, 0xA0, 0x02]),
            ],
            0.0,
        );
        assert_eq!(a.settings.len(), 2, "{:?}", a.settings);
        let (lock, fade) = (&a.settings[0], &a.settings[1]);
        assert_eq!(lock.mask, Some(vec![0x01]), "{lock:?}");
        assert_eq!(
            lock.values.values().cloned().collect::<Vec<_>>(),
            [vec![0x05], vec![0x04]]
        );
        assert_eq!(fade.mask, Some(vec![0x0C]), "{fade:?}");
        // no read before the first write: the whole value, whatever later writes change
        let mut b = Analyzer::default();
        watch(
            &mut b,
            &[
                (0x7C0, vec![0x04, 0x2E, 0xA0, 0x02, 0x04]),
                (0x7C8, vec![0x03, 0x6E, 0xA0, 0x02]),
                (0x7C0, vec![0x04, 0x2E, 0xA0, 0x02, 0x05]),
                (0x7C8, vec![0x03, 0x6E, 0xA0, 0x02]),
            ],
            0.0,
        );
        assert_eq!(b.settings.len(), 1, "{:?}", b.settings);
        assert_eq!(b.settings[0].mask, None);
    }

    /// ICA-219: a routine start is learned only once the module says it started it (`71 01`, after any number of `7F 31 78`
    /// "response pending"); one the module refused (`7F 31 xx`) is noted with the refusal in words, and a start with more
    /// than 8 bytes of parameters is noted as too long to learn. Each answer comes after its request.
    #[test]
    fn a_routine_is_learned_only_when_the_module_starts_it() {
        let mut a = Analyzer::default();
        a.mark("Throttle relearn", 0.0);
        watch(
            &mut a,
            &[
                (0x7E0, vec![0x02, 0x10, 0x03]),
                (0x7E8, vec![0x02, 0x50, 0x03]),
                // refused: conditions not correct
                (0x7E0, vec![0x05, 0x31, 0x01, 0x02, 0x01, 0x05]),
                (0x7E8, vec![0x03, 0x7F, 0x31, 0x22]),
                // pending twice, then started
                (0x7E0, vec![0x04, 0x31, 0x01, 0x02, 0x02]),
                (0x7E8, vec![0x03, 0x7F, 0x31, 0x78]),
                (0x7E8, vec![0x03, 0x7F, 0x31, 0x78]),
                (0x7E8, vec![0x05, 0x71, 0x01, 0x02, 0x02, 0x00]),
                // 9 bytes of parameters (13 bytes: a first frame, the tool's flow control, a consecutive frame)
                (0x7C0, vec![0x10, 0x0D, 0x31, 0x01, 0x03, 0x01, 0x01, 0x02]),
                (0x7C8, vec![0x30, 0x00, 0x00]),
                (0x7C0, vec![0x21, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09]),
                (0x7C8, vec![0x04, 0x71, 0x01, 0x03, 0x01]),
                // a start nothing answered is not learned either
                (0x7B0, vec![0x04, 0x31, 0x01, 0x04, 0x01]),
            ],
            1.0,
        );
        assert_eq!(
            a.services
                .iter()
                .map(|s| (s.module, s.routine, s.name.as_str()))
                .collect::<Vec<_>>(),
            [(0x7E0, 0x0202, "Throttle relearn")],
            "{:?}",
            a.notes
        );
        for want in [
            "7E0 refused the tool's start of routine 0201: conditions not correct",
            "7C0: routine 0301 started with 9 bytes of parameters, too long to learn",
        ] {
            assert!(
                a.notes.iter().any(|n| n.contains(want)),
                "{want}: {:?}",
                a.notes
            );
        }
        assert_eq!(a.notes.len(), 2, "{:?}", a.notes);
        // the refused start does not stay waiting: a later 71 01 with no start of its own teaches nothing, and the same
        // routine started again and accepted is learned with the parameters of the start that worked
        let mut b = Analyzer::default();
        watch(
            &mut b,
            &[
                (0x7E0, vec![0x05, 0x31, 0x01, 0x02, 0x01, 0x05]),
                (0x7E8, vec![0x03, 0x7F, 0x31, 0x22]),
                (0x7E8, vec![0x04, 0x71, 0x01, 0x02, 0x01]),
            ],
            1.0,
        );
        assert!(b.services.is_empty(), "{:?}", b.services);
        watch(
            &mut b,
            &[
                (0x7E0, vec![0x05, 0x31, 0x01, 0x02, 0x01, 0x06]),
                (0x7E8, vec![0x04, 0x71, 0x01, 0x02, 0x01]),
            ],
            10.0,
        );
        assert_eq!(
            b.services
                .iter()
                .map(|s| (s.routine, s.params.clone()))
                .collect::<Vec<_>>(),
            [(0x0201, vec![0x06])]
        );
    }

    /// A shop tool's throttle and KWP2000 tests as the simulated bus plays them (sim.rs's monitor script), each answer
    /// after its request.
    #[test]
    fn a_shop_tool_running_a_throttle_test_is_learned() {
        let mut a = Analyzer::default();
        let convo: &[(u32, &[u8])] = &[
            (0x7E0, &[0x02, 0x10, 0x03]),
            (0x7E8, &[0x02, 0x50, 0x03]),
            (0x7E0, &[0x05, 0x2F, 0xF0, 0x02, 0x03, 0x4C]),
            (0x7E8, &[0x05, 0x6F, 0xF0, 0x02, 0x03, 0x4C]),
            (0x7E0, &[0x05, 0x2F, 0xF0, 0x02, 0x03, 0x80]),
            (0x7E8, &[0x05, 0x6F, 0xF0, 0x02, 0x03, 0x80]),
            (0x7E0, &[0x04, 0x2F, 0xF0, 0x02, 0x00]),
            (0x7E8, &[0x04, 0x6F, 0xF0, 0x02, 0x00]),
            (0x7B0, &[0x04, 0x30, 0x10, 0x07, 0x01]),
            (0x7B8, &[0x03, 0x70, 0x10, 0x07]),
            (0x780, &[0x05, 0x2F, 0x12, 0x34, 0x03, 0x01]),
            (0x788, &[0x05, 0x6F, 0x12, 0x34, 0x03, 0x01]),
        ];
        for (i, (id, d)) in convo.iter().enumerate() {
            a.add(&f(*id, d, i as f64));
        }
        assert!(
            a.log[0].contains("diagnostic session: extended"),
            "{:?}",
            a.log
        );
        assert!(a.log[2].contains("output control: F002, short-term adjustment to 4C"));
        assert!(a.log[6].contains("return control to the module"));
        assert_eq!(
            a.learned.len(),
            2,
            "the airbag module's output is never learned: {:?}",
            a.learned
        );
        let t = &a.learned[0];
        assert_eq!(
            (t.module, t.id, t.protocol, t.control),
            (0x7E0, 0xF002, Protocol::Uds, Control::Percent),
            "two settings: a percentage"
        );
        assert_eq!(
            (a.learned[1].protocol, a.learned[1].on.clone()),
            (Protocol::Kwp, vec![0x01])
        );
        // the entry reads back as a profile test, unchecked
        let toml = format!("name = \"x\"\n{}", to_toml(&a.learned[1]));
        let p = crate::vehicles::parse("x", "test", &toml).unwrap();
        assert_eq!((p.tests[0].id, p.tests[0].checked), (0x10, false));
    }

    /// ICA-221: an output test is learned only once the module takes it (`6F` / `70`, after any number of `7F .. 78`
    /// "response pending"); one the module refused is noted with the refusal in words, and one that gets no answer within
    /// `ANSWER_WAIT` (or by the end of the watch) is noted as unanswered. Each answer comes after its request.
    #[test]
    fn an_output_test_is_learned_only_when_the_module_takes_it() {
        let mut a = Analyzer::default();
        a.mark("Radiator fan", 0.0);
        watch(
            &mut a,
            &[
                (0x7E0, vec![0x02, 0x10, 0x03]),
                (0x7E8, vec![0x02, 0x50, 0x03]),
                // refused: conditions not correct
                (0x7E0, vec![0x05, 0x2F, 0xF0, 0x01, 0x03, 0x01]),
                (0x7E8, vec![0x03, 0x7F, 0x2F, 0x22]),
                // pending twice, then taken
                (0x7E0, vec![0x05, 0x2F, 0xF0, 0x03, 0x03, 0x01]),
                (0x7E8, vec![0x03, 0x7F, 0x2F, 0x78]),
                (0x7E8, vec![0x03, 0x7F, 0x2F, 0x78]),
                (0x7E8, vec![0x05, 0x6F, 0xF0, 0x03, 0x03, 0x01]),
                // KWP2000, refused: request out of range
                (0x7B0, vec![0x04, 0x30, 0x20, 0x07, 0x01]),
                (0x7B8, vec![0x03, 0x7F, 0x30, 0x31]),
            ],
            1.0,
        );
        // no answer: noted once the wait has passed and more of the bus has been read
        watch(
            &mut a,
            &[(0x7C0, vec![0x05, 0x2F, 0xA0, 0x10, 0x03, 0x01])],
            20.0,
        );
        watch(&mut a, &[(0x7E0, vec![0x02, 0x3E, 0x00])], 30.0);
        // and one still waiting when the watch ends
        watch(&mut a, &[(0x7B0, vec![0x04, 0x30, 0x21, 0x07, 0x02])], 40.0);
        a.finish();
        assert_eq!(
            a.learned
                .iter()
                .map(|t| (t.module, t.id, t.name.as_str(), t.on.clone()))
                .collect::<Vec<_>>(),
            [(0x7E0, 0xF003, "Radiator fan", vec![0x01])],
            "{:?}",
            a.notes
        );
        for want in [
            "7E0 refused the tool's output control F001 (to 01): conditions not correct",
            "7B0 refused the tool's output control 20 (to 01): request out of range",
            "7C0: output control A010 (to 01): no answer seen, not learned",
            "7B0: output control 21 (to 02): no answer seen, not learned",
        ] {
            assert!(
                a.notes.iter().any(|n| n.contains(want)),
                "{want}: {:?}",
                a.notes
            );
        }
        assert_eq!(a.notes.len(), 4, "{:?}", a.notes);
        // a refusal of an output test takes no write waiting on the same module with it
        let mut b = Analyzer::default();
        watch(
            &mut b,
            &[
                (0x7C0, vec![0x04, 0x2E, 0xA0, 0x01, 0x00]),
                (0x7C0, vec![0x05, 0x2F, 0xF0, 0x01, 0x03, 0x01]),
                (0x7C8, vec![0x03, 0x7F, 0x2F, 0x22]),
                (0x7C8, vec![0x03, 0x6E, 0xA0, 0x01]),
            ],
            1.0,
        );
        assert_eq!((b.settings.len(), b.learned.len()), (1, 0), "{:?}", b.notes);
    }

    /// Review of ICA-219/221: an answer is paired with its request by the profiles' answer IDs, not only ID - 8. VW's
    /// cluster (714, answering on 77E) has a setting, an output test and a routine learned for 714 with `rx = 0x77E`;
    /// without the profiles nothing pairs, and each is noted as unanswered. A request to every module (7DF) answered on
    /// 7E8 is learned for 7E0. A module answering on an airbag module's ID is refused when its answer shows it.
    #[test]
    fn answers_pair_with_their_requests_by_the_profiles_answer_ids() {
        let cluster: Vec<(u32, Vec<u8>)> = vec![
            (0x714, vec![0x02, 0x10, 0x03]),
            (0x77E, vec![0x02, 0x50, 0x03]),
            (0x714, vec![0x03, 0x22, 0x06, 0x01]),
            (0x77E, vec![0x04, 0x62, 0x06, 0x01, 0x01]),
            (0x714, vec![0x04, 0x2E, 0x06, 0x01, 0x00]),
            (0x77E, vec![0x03, 0x6E, 0x06, 0x01]),
            (0x714, vec![0x05, 0x2F, 0x02, 0x10, 0x03, 0x01]),
            (0x77E, vec![0x05, 0x6F, 0x02, 0x10, 0x03, 0x01]),
            (0x714, vec![0x04, 0x31, 0x01, 0x03, 0x01]),
            (0x77E, vec![0x04, 0x71, 0x01, 0x03, 0x01]),
        ];
        let mut a = Analyzer::default().answering(&crate::vehicles::builtin());
        watch(&mut a, &cluster, 1.0);
        a.finish();
        assert_eq!(
            (
                a.settings
                    .iter()
                    .map(|s| (s.module, s.rx))
                    .collect::<Vec<_>>(),
                a.learned
                    .iter()
                    .map(|t| (t.module, t.rx))
                    .collect::<Vec<_>>(),
                a.services
                    .iter()
                    .map(|s| (s.module, s.rx))
                    .collect::<Vec<_>>(),
            ),
            (
                vec![(0x714, Some(0x77E))],
                vec![(0x714, Some(0x77E))],
                vec![(0x714, Some(0x77E))]
            ),
            "{:?}",
            a.notes
        );
        assert!(a.notes.is_empty(), "{:?}", a.notes);
        let s = &a.settings[0];
        assert_eq!(
            (s.values["was"].clone(), s.extended),
            (vec![0x01], true),
            "the read before it and the extended session, through 77E"
        );
        assert!(to_toml(&a.learned[0]).contains("rx = 0x77E"));
        // the entries read back as profile entries that ask 714 and listen on 77E
        let back = crate::vehicles::parse(
            "x",
            "test",
            &format!(
                "name = \"x\"\n{}\n{}\n{}",
                crate::settings::to_toml(s),
                to_toml(&a.learned[0]),
                crate::service::to_toml(&a.services[0])
            ),
        )
        .unwrap();
        assert_eq!(
            (
                back.settings[0].rx(),
                back.tests[0].rx(),
                back.services[0].rx()
            ),
            (0x77E, 0x77E, 0x77E)
        );
        // without the profiles' pairing nothing answers 714, and that is what the notes say
        let mut b = Analyzer::default();
        watch(&mut b, &cluster, 1.0);
        b.finish();
        assert!(
            b.settings.is_empty() && b.learned.is_empty() && b.services.is_empty(),
            "{:?}",
            b.notes
        );
        for want in [
            "714: output control 0210 (to 01): no answer seen",
            "714: routine 0301 started: no answer seen",
        ] {
            assert!(
                b.notes.iter().any(|n| n.contains(want)),
                "{want}: {:?}",
                b.notes
            );
        }
        // to every module: 7DF, answered by the engine on 7E8
        let mut c = Analyzer::default();
        c.mark("EVAP test", 0.0);
        watch(
            &mut c,
            &[
                (0x7DF, vec![0x04, 0x31, 0x01, 0x02, 0x05]),
                (0x7E8, vec![0x03, 0x7F, 0x31, 0x78]),
                (0x7E8, vec![0x04, 0x71, 0x01, 0x02, 0x05]),
                (0x7DF, vec![0x05, 0x2F, 0xF0, 0x02, 0x03, 0x01]),
                (0x7E9, vec![0x05, 0x6F, 0xF0, 0x02, 0x03, 0x01]),
            ],
            1.0,
        );
        assert_eq!(
            (
                c.services
                    .iter()
                    .map(|s| (s.module, s.rx, s.name.as_str()))
                    .collect::<Vec<_>>(),
                c.learned
                    .iter()
                    .map(|t| (t.module, t.rx, t.name.as_str()))
                    .collect::<Vec<_>>()
            ),
            (
                vec![(0x7E0, None, "EVAP test")],
                vec![(0x7E1, None, "Learned: output F002 on 7E1")]
            ),
            "{:?}",
            c.notes
        );
        // a module answering on an airbag module's ID (788): its request ID passed, its answer shows what it is
        let p = crate::vehicles::parse(
            "test",
            "a test",
            "name = \"test\"\n[[module]]\ntx = 0x740\nrx = 0x788\nname = \"body\"\n",
        )
        .unwrap();
        let mut d = Analyzer::default().answering(&[p]);
        watch(
            &mut d,
            &[
                (0x740, vec![0x05, 0x2F, 0x01, 0x10, 0x03, 0x01]),
                (0x788, vec![0x05, 0x6F, 0x01, 0x10, 0x03, 0x01]),
                (0x740, vec![0x04, 0x31, 0x01, 0x03, 0x01]),
                (0x788, vec![0x04, 0x71, 0x01, 0x03, 0x01]),
            ],
            1.0,
        );
        assert!(
            d.learned.is_empty() && d.services.is_empty(),
            "{:?}",
            d.learned
        );
        assert_eq!(
            d.notes.iter().filter(|n| n.contains("airbag")).count(),
            2,
            "{:?}",
            d.notes
        );
    }

    /// Review of ICA-221: the wait is checked as frames come, not only at the end. A 6F 7 s after its request with no
    /// "response pending" between is too late (5 s and the 1 s of slack): noted before `finish`, not learned. 78s 4 s
    /// apart keep a request waiting, and its 6F at 12 s is learned.
    #[test]
    fn the_answer_wait_runs_as_frames_come_and_response_pending_extends_it() {
        let mut a = Analyzer::default();
        a.add(&f(0x7E0, &[0x05, 0x2F, 0xF0, 0x01, 0x03, 0x01], 0.0));
        a.add(&f(0x7E8, &[0x05, 0x6F, 0xF0, 0x01, 0x03, 0x01], 7.0));
        assert!(a.learned.is_empty(), "{:?}", a.learned);
        assert!(
            a.notes
                .iter()
                .any(|n| n.contains("7E0: output control F001 (to 01): no answer seen")),
            "noted before finish: {:?}",
            a.notes
        );
        // within the wait and its slack: learned
        let mut b = Analyzer::default();
        b.add(&f(0x7E0, &[0x05, 0x2F, 0xF0, 0x01, 0x03, 0x01], 0.0));
        b.add(&f(0x7E8, &[0x05, 0x6F, 0xF0, 0x01, 0x03, 0x01], 5.8));
        assert_eq!(b.learned.len(), 1, "{:?}", b.notes);
        // response pending every 4 s, then the answer at 12 s
        let mut c = Analyzer::default();
        c.add(&f(0x7E0, &[0x05, 0x2F, 0xF0, 0x01, 0x03, 0x01], 0.0));
        for at in [4.0, 8.0] {
            c.add(&f(0x7E8, &[0x03, 0x7F, 0x2F, 0x78], at));
        }
        c.add(&f(0x7E8, &[0x05, 0x6F, 0xF0, 0x01, 0x03, 0x01], 12.0));
        assert_eq!(c.learned.len(), 1, "{:?}", c.notes);
        assert!(c.notes.is_empty(), "{:?}", c.notes);
        // the same for a routine
        let mut d = Analyzer::default();
        d.add(&f(0x7E0, &[0x04, 0x31, 0x01, 0x02, 0x01], 0.0));
        for at in [4.0, 8.0] {
            d.add(&f(0x7E8, &[0x03, 0x7F, 0x31, 0x78], at));
        }
        d.add(&f(0x7E8, &[0x04, 0x71, 0x01, 0x02, 0x01], 12.0));
        assert_eq!(d.services.len(), 1, "{:?}", d.notes);
    }

    /// Review of ICA-221: a mark names one item, the first waiting one the module takes, and only if no other mark came
    /// after it was sent: two requests waiting after "Fan" do not both get it, and a mark typed while a routine waits
    /// names the next thing, not that routine.
    #[test]
    fn a_mark_names_one_waiting_item_and_only_its_own() {
        let mut a = Analyzer::default();
        a.mark("Fan", 0.0);
        watch(
            &mut a,
            &[
                (0x7E0, vec![0x05, 0x2F, 0xF0, 0x01, 0x03, 0x01]),
                (0x7E0, vec![0x05, 0x2F, 0xF0, 0x02, 0x03, 0x01]),
                (0x7E8, vec![0x05, 0x6F, 0xF0, 0x01, 0x03, 0x01]),
                (0x7E8, vec![0x05, 0x6F, 0xF0, 0x02, 0x03, 0x01]),
            ],
            1.0,
        );
        assert_eq!(
            a.learned
                .iter()
                .map(|t| t.name.as_str())
                .collect::<Vec<_>>(),
            ["Fan", "Learned: output F002 on 7E0"]
        );
        let mut b = Analyzer::default();
        b.add(&f(0x7E0, &[0x04, 0x31, 0x01, 0x02, 0x01], 1.0));
        b.add(&f(0x7E8, &[0x03, 0x7F, 0x31, 0x78], 2.0));
        b.mark("Door chime off", 3.0);
        b.add(&f(0x7E8, &[0x04, 0x71, 0x01, 0x02, 0x01], 4.0));
        b.add(&f(0x7C0, &[0x04, 0x2E, 0xA0, 0x01, 0x00], 5.0));
        b.add(&f(0x7C8, &[0x03, 0x6E, 0xA0, 0x01], 5.1));
        assert_eq!(
            (b.services[0].name.as_str(), b.settings[0].name.as_str()),
            ("Learned: routine 0201 on 7E0", "Door chime off")
        );
    }

    #[test]
    fn long_messages_are_put_together() {
        let mut a = Analyzer::default();
        a.add(&f(
            0x7E8,
            &[0x10, 0x14, 0x49, 0x02, 0x01, 0x31, 0x41, 0x42],
            0.0,
        ));
        a.add(&f(0x7E0, &[0x30, 0x00, 0x00], 0.0));
        a.add(&f(
            0x7E8,
            &[0x21, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49],
            0.0,
        ));
        a.add(&f(
            0x7E8,
            &[0x22, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50],
            0.0,
        ));
        assert_eq!(a.log.len(), 1);
        assert!(a.log[0].contains("answer to vehicle information"));
    }
}
