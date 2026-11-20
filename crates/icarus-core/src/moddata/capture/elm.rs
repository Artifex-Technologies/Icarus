//! An app's conversation with an ELM327 or STN adapter (the OBDLink family), read back from the bytes each side sent: the
//! app's commands end at a carriage return, the adapter's answers at its `>` prompt. The adapter's settings that change
//! what a request means or how its answer looks are followed as they go by, each only once the adapter said OK:
//!
//! | command | what is followed |
//! | --- | --- |
//! | `ATZ`, `ATD`, `ATWS` | back to the defaults (headers off, CAN formatting on, no filters, the default header) |
//! | `ATSP n`, `ATTP n`, `ATDPN` (its answer) | the protocol: 11-bit or 29-bit CAN, or K-line/J1850 |
//! | `ATSH xyz`, `ATSH xxyyzz`, `ATSH xxxxxxxx`, `ATCP` | the request ID (11-bit, 29-bit with the priority byte) |
//! | `ATCRA`, `ATAR`, `ATCF` with `ATCM` | the answer ID the adapter listens to |
//! | `ATCEA hh` | an extended address byte in front of each message |
//! | `ATH0/1`, `ATCAF0/1`, `ATS0/1`, `ATD0/1` | headers shown, CAN formatting (ISO-TP done by the adapter), spaces, the length digit |
//! | `STPX H:hdr, D:data, R:n` (STN) | a one-shot request on its own header |
//!
//! Answers are read with and without headers: CAN frames put back together per sender (`crate::diag`), the adapter's own
//! multi-frame layout without headers (a length line, then `0:`, `1:` ... lines), "response pending" (7F xx 78) skipped,
//! the adapter's words (NO DATA, CAN ERROR, STOPPED...) kept as the exchange's problem. An empty command repeats the last
//! one, as the adapter does. Adapter commands are counted and dropped: they are not requests to the vehicle.

use super::{Exchange, Reply};
use crate::diag::{self, Fed, Reassembler};
use crate::moddata::Ids;
use std::collections::VecDeque;

/// Bytes one side sent at one time.
#[derive(Clone, Debug, PartialEq)]
pub struct Chunk {
    /// Seconds from the start of the capture.
    pub at: f64,
    /// Seconds since 1970 (UTC), when the capture holds it.
    pub clock: Option<f64>,
    /// Sent by the adapter (else by the app).
    pub adapter: bool,
    pub bytes: Vec<u8>,
}

/// The longest command and answer kept (an answer of 4095 bytes with headers is about 20 kB of text).
const MAX_COMMAND: usize = 512;
const MAX_ANSWER: usize = 256 * 1024;

/// What a conversation held.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Conversation {
    pub exchanges: Vec<Exchange>,
    /// AT and ST commands (adapter settings, its identity, the battery voltage...): counted, never kept.
    pub adapter_commands: usize,
    /// Commands that were neither adapter commands nor hex requests.
    pub unreadable: usize,
}

/// A command and the text that answered it.
struct Said {
    at: f64,
    clock: Option<f64>,
    command: String,
    answer: String,
    /// The adapter's prompt came (the answer is whole).
    prompt: bool,
}

/// Commands paired with their answers: each answer ends at the adapter's prompt and answers the oldest command waiting.
fn pair(chunks: &[Chunk]) -> Vec<Said> {
    let mut out = Vec::new();
    let mut command = String::new();
    let mut waiting: VecDeque<(f64, Option<f64>, String)> = VecDeque::new();
    let mut answer = String::new();
    let mut last = String::new();
    for c in chunks {
        for &b in &c.bytes {
            if c.adapter {
                match b {
                    b'>' => match waiting.pop_front() {
                        Some((at, clock, command)) => out.push(Said {
                            at,
                            clock,
                            command,
                            answer: std::mem::take(&mut answer),
                            prompt: true,
                        }),
                        None => answer.clear(), // the prompt after start-up, or text nobody asked for
                    },
                    b'\r' | b'\n' => answer.push('\r'),
                    0x20..=0x7E if answer.len() < MAX_ANSWER => answer.push(b as char),
                    _ => {}
                }
            } else {
                match b {
                    b'\r' => {
                        let t = command.trim().to_string();
                        command.clear();
                        // an empty command repeats the last one
                        let t = if t.is_empty() { last.clone() } else { t };
                        if !t.is_empty() {
                            last = t.clone();
                            waiting.push_back((c.at, c.clock, t));
                        }
                    }
                    0x20..=0x7E if command.len() < MAX_COMMAND => command.push(b as char),
                    _ => {}
                }
            }
        }
    }
    for (at, clock, command) in waiting {
        out.push(Said {
            at,
            clock,
            command,
            answer: std::mem::take(&mut answer),
            prompt: false,
        });
    }
    out
}

/// The adapter's settings that change what a request means or how its answer looks.
struct State {
    headers: bool,
    caf: bool,
    spaces: bool,
    dlc: bool,
    /// ATSP's protocol number; None: automatic or not known.
    protocol: Option<u8>,
    /// ATSH: (kind, ID).
    header: Option<(Ids, u32)>,
    priority: u8,
    cra: Option<u32>,
    cf: Option<u32>,
    cm: Option<u32>,
    cea: Option<u8>,
    /// Frames put back together across exchanges (CAN formatting off: the app sends flow control itself).
    isotp: Reassembler<Option<u32>>,
}

impl Default for State {
    fn default() -> Self {
        State {
            headers: false,
            caf: true,
            spaces: true,
            dlc: false,
            protocol: None,
            header: None,
            priority: 0x18,
            cra: None,
            cf: None,
            cm: None,
            cea: None,
            isotp: Reassembler::default(),
        }
    }
}

fn hex_digits(s: &str) -> Option<u32> {
    (!s.is_empty() && s.len() <= 8 && s.chars().all(|c| c.is_ascii_hexdigit()))
        .then(|| u32::from_str_radix(s, 16).ok())
        .flatten()
}

impl State {
    fn kline(&self) -> bool {
        matches!(self.protocol, Some(1..=5))
    }

    fn can29(&self) -> bool {
        match self.protocol {
            Some(7 | 9) => true,
            Some(6 | 8) => false,
            _ => matches!(self.header, Some((Ids::Can29, _))),
        }
    }

    fn ids(&self) -> Ids {
        if self.kline() {
            Ids::Kline
        } else if self.can29() {
            Ids::Can29
        } else {
            Ids::Can11
        }
    }

    /// The request ID now: ATSH's, else the protocol's default (the functional ID every module hears).
    fn tx(&self) -> (Ids, Option<u32>) {
        if let Some((ids, h)) = self.header {
            return (ids, Some(h));
        }
        match self.ids() {
            Ids::Kline => (Ids::Kline, None),
            Ids::Can29 => (Ids::Can29, Some(0x18DB_33F1)),
            Ids::Can11 => (Ids::Can11, Some(0x7DF)),
        }
    }

    /// The answer ID the adapter listens to, when it was told one.
    fn rx(&self) -> Option<u32> {
        self.cra.or(match (self.cf, self.cm) {
            (Some(f), Some(0x7FF | 0x1FFF_FFFF)) => Some(f),
            _ => None,
        })
    }

    /// A header in hex digits: 3 (11-bit), 6 (11-bit, or 29-bit after the priority byte) or 8 (29-bit).
    fn header_of(&self, h: &str) -> Option<(Ids, u32)> {
        let v = hex_digits(h)?;
        if self.kline() && h.len() == 6 {
            return Some((Ids::Kline, v));
        }
        match h.len() {
            3 => Some((Ids::Can11, v & 0x7FF)),
            8 => Some((Ids::Can29, v & 0x1FFF_FFFF)),
            6 if self.can29() || v > 0x7FF => Some((
                Ids::Can29,
                ((u32::from(self.priority) << 24) | v) & 0x1FFF_FFFF,
            )),
            6 => Some((Ids::Can11, v & 0x7FF)),
            _ => None,
        }
    }

    /// An AT command (without "AT", capitals, no spaces) the adapter answered with `lines`.
    fn at(&mut self, c: &str, lines: &[String]) {
        if matches!(c, "Z" | "D" | "WS") {
            let protocol = self.protocol;
            *self = State::default();
            // the protocol set with ATSP is kept over a reset
            self.protocol = protocol;
            return;
        }
        if c == "DPN" {
            if let Some(p) = lines
                .iter()
                .find_map(|l| l.trim().trim_start_matches('A').chars().next())
                .and_then(|d| d.to_digit(16))
            {
                self.protocol = (p != 0).then_some(p as u8);
            }
            return;
        }
        if !lines.iter().any(|l| l.trim() == "OK") {
            return; // "?" or nothing: the setting did not change
        }
        let protocol = |s: &str| u8::from_str_radix(s, 16).ok().filter(|p| *p <= 0x0C);
        match c {
            "H0" => self.headers = false,
            "H1" => self.headers = true,
            "CAF0" => self.caf = false,
            "CAF1" => self.caf = true,
            "S0" => self.spaces = false,
            "S1" => self.spaces = true,
            "D0" => self.dlc = false,
            "D1" => self.dlc = true,
            "AR" | "CRA" => self.cra = None,
            "CEA" => self.cea = None,
            _ if c.starts_with("SPA") || c.starts_with("TPA") => {
                if let Some(p) = protocol(&c[3..]) {
                    self.protocol = (p != 0).then_some(p);
                }
            }
            _ if c.starts_with("SP") || c.starts_with("TP") => {
                if let Some(p) = protocol(&c[2..]) {
                    self.protocol = (p != 0).then_some(p);
                }
            }
            _ if c.starts_with("SH") => {
                if let Some(h) = self.header_of(&c[2..]) {
                    self.header = Some(h);
                }
            }
            _ if c.starts_with("CRA") => {
                self.cra = hex_digits(&c[3..]).filter(|_| matches!(c.len() - 3, 3 | 8));
            }
            _ if c.starts_with("CEA") => {
                self.cea = hex_digits(&c[3..]).and_then(|v| u8::try_from(v).ok())
            }
            _ if c.starts_with("CP") => {
                if let Some(p) = hex_digits(&c[2..]).and_then(|v| u8::try_from(v).ok()) {
                    self.priority = p & 0x1F;
                }
            }
            // CAN flow control on or off (ATCFC0/1), not the filter
            "CFC0" | "CFC1" => {}
            _ if c.starts_with("CF") => {
                self.cf = hex_digits(&c[2..]).filter(|_| matches!(c.len() - 2, 3 | 8));
            }
            _ if c.starts_with("CM") => {
                self.cm = hex_digits(&c[2..]).filter(|_| matches!(c.len() - 2, 3 | 8));
            }
            _ => {}
        }
    }

    /// One answer line with headers shown: (sender, the frame's bytes).
    fn split_header(&self, line: &str) -> Option<(u32, Vec<u8>)> {
        let toks: Vec<&str> = line.split_whitespace().collect();
        let bytes = |toks: &[&str]| -> Option<Vec<u8>> {
            toks.iter()
                .map(|t| {
                    (t.len() == 2)
                        .then(|| u8::from_str_radix(t, 16).ok())
                        .flatten()
                })
                .collect()
        };
        let (id, rest): (u32, Vec<u8>) = if toks.len() > 1 {
            let (id, mut rest) = if matches!(toks[0].len(), 3 | 8) {
                (hex_digits(toks[0])?, &toks[1..])
            } else if toks.len() >= 5
                && toks[..4].iter().all(|t| t.len() == 2)
                && (self.can29() || toks[0].starts_with('1'))
            {
                (hex_digits(&toks[..4].concat())?, &toks[4..])
            } else {
                return None;
            };
            if self.dlc && rest.first().is_some_and(|t| t.len() == 1) {
                rest = &rest[1..];
            }
            (id, bytes(rest)?)
        } else {
            let s = toks.first()?;
            let head = if self.can29() {
                8
            } else if s.len() % 2 == 1 {
                3
            } else {
                return None;
            };
            let skip = head + usize::from(self.dlc);
            if s.len() < skip || (s.len() - skip) % 2 != 0 {
                return None;
            }
            let id = hex_digits(&s[..head])?;
            let rest = (skip..s.len())
                .step_by(2)
                .map(|i| u8::from_str_radix(&s[i..i + 2], 16).ok())
                .collect::<Option<Vec<u8>>>()?;
            (id, rest)
        };
        Some((id, rest))
    }

    /// The messages in an answer's lines, for a request with service `sid`; and whether any line could not be read.
    fn replies(&mut self, lines: &[String], sid: u8) -> (Vec<Reply>, bool) {
        let mut out = Vec::new();
        let mut unreadable = false;
        let framed = self.headers || !self.caf;
        if self.kline() && self.headers {
            // header (priority, target, sender), the data, a checksum byte
            for l in lines {
                match self.split_line_bytes(l) {
                    Some(b) if b.len() >= 5 => out.push(Reply {
                        from: Some(u32::from(b[0]) << 16 | u32::from(b[1]) << 8 | u32::from(b[2])),
                        data: b[3..b.len() - 1].to_vec(),
                    }),
                    _ => unreadable = true,
                }
            }
        } else if framed {
            for l in lines {
                let (from, mut frame) = if self.headers {
                    match self.split_header(l) {
                        Some((id, b)) => (Some(id), b),
                        None => {
                            unreadable = true;
                            continue;
                        }
                    }
                } else {
                    match self.split_line_bytes(l) {
                        Some(b) => (None, b),
                        None => {
                            unreadable = true;
                            continue;
                        }
                    }
                };
                if self.cea.is_some() && frame.len() > 1 {
                    frame.remove(0); // the extended address byte
                }
                match self.isotp.feed(from, &frame) {
                    Fed::Message(data) => out.push(Reply { from, data }),
                    Fed::Broken(_) => unreadable = true,
                    Fed::Partial | Fed::FlowControl => {}
                }
            }
        } else {
            // headers off, CAN formatting on: one message per line, or the adapter's own layout of a long one
            let mut open: Option<(Option<usize>, Vec<u8>)> = None;
            let close = |open: &mut Option<(Option<usize>, Vec<u8>)>, out: &mut Vec<Reply>| {
                if let Some((len, mut data)) = open.take() {
                    if let Some(n) = len {
                        data.truncate(n);
                    }
                    if !data.is_empty() {
                        out.push(Reply { from: None, data });
                    }
                }
            };
            for l in lines {
                let t = l.trim();
                let compact: String = t.chars().filter(|c| !c.is_whitespace()).collect();
                if compact.len() == 3 && compact.chars().all(|c| c.is_ascii_hexdigit()) {
                    close(&mut open, &mut out);
                    open = Some((usize::from_str_radix(&compact, 16).ok(), Vec::new()));
                    continue;
                }
                if let Some((idx, rest)) = t.split_once(':') {
                    if idx.trim().len() == 1 && idx.trim().chars().all(|c| c.is_ascii_hexdigit()) {
                        match self.split_line_bytes(rest) {
                            Some(b) => match open.as_mut() {
                                Some((_, data)) => data.extend(b),
                                None => open = Some((None, b)),
                            },
                            None => unreadable = true,
                        }
                        continue;
                    }
                }
                close(&mut open, &mut out);
                match self.split_line_bytes(t) {
                    Some(b) => out.push(Reply {
                        from: None,
                        data: b,
                    }),
                    None => unreadable = true,
                }
            }
            close(&mut open, &mut out);
        }
        for r in out.iter_mut().filter(|r| r.from.is_none()) {
            // an extended address in front of an answer without headers: the answer starts one byte later
            if self.cea.is_some()
                && r.data.len() > 1
                && diag::answers(&r.data) != Some(sid)
                && diag::answers(&r.data[1..]) == Some(sid)
            {
                r.data.remove(0);
            }
        }
        if out.is_empty() && lines.is_empty() {
            return (out, false);
        }
        // "response pending" says the answer is still to come
        out.retain(|r| !(r.data.len() >= 3 && r.data[0] == 0x7F && r.data[2] == 0x78));
        (out, unreadable)
    }

    /// A line of hex bytes, with or without spaces.
    fn split_line_bytes(&self, l: &str) -> Option<Vec<u8>> {
        let compact: String = l.chars().filter(|c| !c.is_whitespace()).collect();
        if compact.is_empty() || !compact.len().is_multiple_of(2) {
            return None;
        }
        (0..compact.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(compact.get(i..i + 2)?, 16).ok())
            .collect()
    }
}

/// What the adapter says when there is no answer to give.
const WORDS: [&str; 16] = [
    "NO DATA",
    "CAN ERROR",
    "BUS ERROR",
    "BUS BUSY",
    "DATA ERROR",
    "<DATA ERROR",
    "<RX ERROR",
    "RX ERROR",
    "UNABLE TO CONNECT",
    "STOPPED",
    "BUFFER FULL",
    "FB ERROR",
    "LV RESET",
    "ACT ALERT",
    "?",
    "ERR",
];

/// An answer's lines: the echo of the command, empty lines and the adapter's progress words left out.
fn answer_lines(answer: &str, compact_command: &str) -> Vec<String> {
    let mut lines: Vec<String> = answer
        .split('\r')
        .map(|l| l.trim().to_string())
        .filter(|l| !l.is_empty())
        .filter(|l| !l.starts_with("SEARCHING") && !l.starts_with("BUS INIT"))
        .collect();
    if lines.first().is_some_and(|l| {
        l.chars()
            .filter(|c| !c.is_whitespace())
            .collect::<String>()
            .eq_ignore_ascii_case(compact_command)
    }) {
        lines.remove(0);
    }
    lines
}

/// A request in hex ("22 1234", "221234", "0100 1": a last odd digit is how many answers to wait for).
fn request_bytes(compact: &str) -> Option<Vec<u8>> {
    if compact.is_empty() || !compact.chars().all(|c| c.is_ascii_hexdigit()) {
        return None;
    }
    let even = &compact[..compact.len() & !1];
    if even.is_empty() {
        return None;
    }
    (0..even.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&even[i..i + 2], 16).ok())
        .collect()
}

/// `STPX H:7E0, D:22 12 34, R:1`: (header, data); None without data (a long send whose data follows on its own).
fn stpx(command: &str) -> Option<(Option<String>, Vec<u8>)> {
    let rest = command.get(4..)?;
    let (mut header, mut data) = (None, None);
    for part in rest.split(',') {
        let Some((k, v)) = part.split_once(':') else {
            continue;
        };
        let v: String = v.chars().filter(|c| !c.is_whitespace()).collect();
        match k.trim().to_ascii_uppercase().as_str() {
            "H" => header = Some(v.to_ascii_uppercase()),
            "D" => {
                data = request_bytes(&v.to_ascii_uppercase()).filter(|_| v.len().is_multiple_of(2))
            }
            _ => {}
        }
    }
    Some((header, data?))
}

/// The exchanges in a conversation, in order.
pub fn conversation(chunks: &[Chunk]) -> Conversation {
    let mut st = State::default();
    let mut out = Conversation::default();
    for s in pair(chunks) {
        let upper = s.command.to_ascii_uppercase();
        let compact: String = upper.chars().filter(|c| !c.is_whitespace()).collect();
        let lines = answer_lines(&s.answer, &compact);
        if let Some(c) = compact.strip_prefix("AT") {
            st.at(c, &lines);
            out.adapter_commands += 1;
            continue;
        }
        let (header, mut request) = if compact.starts_with("STPX") {
            match stpx(&upper) {
                Some((h, d)) => (h, d),
                None => {
                    out.adapter_commands += 1;
                    continue;
                }
            }
        } else if compact.starts_with("ST") {
            out.adapter_commands += 1;
            continue;
        } else {
            match request_bytes(&compact) {
                Some(b) => (None, b),
                None => {
                    out.unreadable += 1;
                    continue;
                }
            }
        };
        let (ids, tx) = match header.as_deref().and_then(|h| st.header_of(h)) {
            Some((ids, h)) => (ids, Some(h)),
            None => st.tx(),
        };
        let mut continuation = false;
        if !st.caf && ids != Ids::Kline {
            // the app sends the frame itself: a single frame is the request, flow control asks for the rest
            match diag::frame(&request) {
                Some(diag::Frame::Single(d)) => request = d.to_vec(),
                Some(diag::Frame::FlowControl) => continuation = true,
                _ => {
                    out.unreadable += 1;
                    continue;
                }
            }
        }
        let sid = request.first().copied().unwrap_or(0);
        let (replies, garbled) = st.replies(&lines, sid);
        let problem = lines
            .iter()
            .find(|l| WORDS.iter().any(|w| l.starts_with(w)))
            .cloned()
            .or_else(|| (!s.prompt).then(|| "no answer (the capture ends first)".to_string()))
            .or_else(|| lines.is_empty().then(|| "no answer".to_string()))
            .or_else(|| {
                (garbled && replies.is_empty())
                    .then(|| "an answer Icarus could not read".to_string())
            });
        if continuation {
            if let Some(last) = out.exchanges.last_mut() {
                last.replies.extend(replies);
            }
            continue;
        }
        out.exchanges.push(Exchange {
            at: s.at,
            clock: s.clock,
            ids,
            tx,
            ext: st.cea,
            rx: st.rx(),
            request,
            replies,
            problem,
            masked: false,
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A conversation from (command, answer) pairs, one second apart; the answer ends with the prompt.
    fn talk(pairs: &[(&str, &str)]) -> Conversation {
        let mut chunks = vec![Chunk {
            at: 0.0,
            clock: None,
            adapter: true,
            bytes: b"ELM327 v1.4b\r\r>".to_vec(),
        }];
        for (i, (c, a)) in pairs.iter().enumerate() {
            chunks.push(Chunk {
                at: i as f64,
                clock: None,
                adapter: false,
                bytes: format!("{c}\r").into_bytes(),
            });
            chunks.push(Chunk {
                at: i as f64 + 0.1,
                clock: None,
                adapter: true,
                bytes: format!("{a}\r\r>").into_bytes(),
            });
        }
        conversation(&chunks)
    }

    #[test]
    fn headers_on_and_off_single_and_long_answers() {
        let c = talk(&[
            ("ATZ", "ELM327 v1.4b"),
            ("ATE0", "ATE0\rOK"),
            ("ATH1", "OK"),
            ("ATSH7E0", "OK"),
            (
                "22 1234",
                "7E8 10 0B 62 12 34 01 02 03\r7E8 21 04 05 06 07 08 AA AA",
            ),
            ("221235", "7E8 05 62 12 35 AA BB"),
            ("ATH0", "OK"),
            ("2112", "00B\r0: 61 12 01 02 03 04\r1: 05 06 07 08 09 AA AA"),
            ("21 13", "61 13 7F"),
            ("ATCRA 7E9", "OK"),
            ("22 0001", "NO DATA"),
            ("22 0002", "7F 22 78\r62 00 02 05"),
            ("ATSH 7B0", "?"),
            ("22 0003", "7F 22 31"),
        ]);
        assert_eq!(c.adapter_commands, 7);
        let e = &c.exchanges;
        assert_eq!(e.len(), 7);
        assert_eq!((e[0].tx, e[0].ids), (Some(0x7E0), Ids::Can11));
        assert_eq!(e[0].request, vec![0x22, 0x12, 0x34]);
        assert_eq!(
            e[0].replies,
            vec![Reply {
                from: Some(0x7E8),
                data: vec![0x62, 0x12, 0x34, 1, 2, 3, 4, 5, 6, 7, 8]
            }]
        );
        assert_eq!(e[1].replies[0].data, vec![0x62, 0x12, 0x35, 0xAA, 0xBB]);
        assert_eq!(
            e[2].replies,
            vec![Reply {
                from: None,
                data: vec![0x61, 0x12, 1, 2, 3, 4, 5, 6, 7, 8, 9]
            }]
        );
        assert_eq!(e[3].replies[0].data, vec![0x61, 0x13, 0x7F]);
        assert_eq!(
            (e[4].replies.len(), e[4].problem.as_deref(), e[4].rx),
            (0, Some("NO DATA"), Some(0x7E9))
        );
        assert_eq!(
            e[5].replies,
            vec![Reply {
                from: None,
                data: vec![0x62, 0x00, 0x02, 0x05]
            }],
            "pending skipped"
        );
        assert_eq!(
            e[6].tx,
            Some(0x7E0),
            "ATSH answered ? did not change the header"
        );
        assert_eq!(e[6].replies[0].data, vec![0x7F, 0x22, 0x31]);
    }

    #[test]
    fn the_default_header_is_every_module_and_echo_is_dropped() {
        let c = talk(&[(
            "0902",
            "0902\r014\r0: 49 02 01 31 32 33\r1: 34 35 36 37 38 39 30\r2: 31 32 33 34 35 36 37",
        )]);
        let e = &c.exchanges[0];
        assert_eq!(e.tx, Some(0x7DF));
        assert_eq!(e.replies[0].data.len(), 0x14);
        assert_eq!(&e.replies[0].data[..3], &[0x49, 0x02, 0x01]);
    }

    #[test]
    fn stn_one_shot_requests_use_their_own_header() {
        let c = talk(&[
            ("ATH1", "OK"),
            ("STPX H:7B0, D:22 AB CD, R:1", "7B8 04 62 AB CD 01"),
            ("STPX h:18DA28F1,d:2101", "18 DA F1 28 03 61 01 07"),
            ("22 0101", "7E8 04 62 01 01 09"),
            ("STPX L:100", "DATA"),
            ("STDI", "OBDLink MX+ r1.2"),
        ]);
        assert_eq!(c.adapter_commands, 3);
        let e = &c.exchanges;
        assert_eq!(
            (e[0].tx, e[0].request.clone()),
            (Some(0x7B0), vec![0x22, 0xAB, 0xCD])
        );
        assert_eq!(e[0].replies[0].from, Some(0x7B8));
        assert_eq!((e[1].ids, e[1].tx), (Ids::Can29, Some(0x18DA28F1)));
        assert_eq!(
            e[1].replies[0],
            Reply {
                from: Some(0x18DAF128),
                data: vec![0x61, 0x01, 0x07]
            }
        );
        assert_eq!(
            e[2].tx,
            Some(0x7DF),
            "STPX leaves the adapter's header alone"
        );
    }

    #[test]
    fn twenty_nine_bit_extended_addresses_and_no_spaces() {
        let c = talk(&[
            ("ATSP7", "OK"),
            ("ATH1", "OK"),
            ("ATS0", "OK"),
            ("ATSH DA10F1", "OK"),
            ("221234", "18DAF110046212340100"),
            ("ATSP6", "OK"),
            ("ATSH750", "OK"),
            ("ATCEA40", "OK"),
            ("2101", "758F10361010A"),
            ("ATCEA", "OK"),
        ]);
        let e = &c.exchanges;
        assert_eq!(
            (e[0].ids, e[0].tx),
            (Ids::Can29, Some(0x18DA10F1)),
            "priority 18 in front"
        );
        assert_eq!(
            e[0].replies[0],
            Reply {
                from: Some(0x18DAF110),
                data: vec![0x62, 0x12, 0x34, 0x01]
            }
        );
        assert_eq!((e[1].tx, e[1].ext), (Some(0x750), Some(0x40)));
        assert_eq!(
            e[1].replies[0],
            Reply {
                from: Some(0x758),
                data: vec![0x61, 0x01, 0x0A]
            }
        );
    }

    #[test]
    fn can_formatting_off_the_app_does_flow_control() {
        let c = talk(&[
            ("ATCAF0", "OK"),
            ("ATH1", "OK"),
            ("ATSH7E0", "OK"),
            ("03 22 12 34 00 00 00 00", "7E8 10 09 62 12 34 01 02 03"),
            ("30 00 00 00 00 00 00 00", "7E8 21 04 05 06 AA AA AA AA"),
            ("02 21 05", "7E8 03 61 05 09"),
        ]);
        let e = &c.exchanges;
        assert_eq!(e.len(), 2);
        assert_eq!(e[0].request, vec![0x22, 0x12, 0x34]);
        assert_eq!(
            e[0].replies[0].data,
            vec![0x62, 0x12, 0x34, 1, 2, 3, 4, 5, 6]
        );
        assert_eq!(e[1].replies[0].data, vec![0x61, 0x05, 0x09]);
    }

    #[test]
    fn flow_control_on_and_off_is_not_the_filter() {
        let c = talk(&[
            ("ATSH7E0", "OK"),
            ("ATCF 7E8", "OK"),
            ("ATCM 7FF", "OK"),
            ("ATCFC1", "OK"),
            ("22 1234", "62 12 34 01"),
            ("ATCFC0", "OK"),
            ("22 1235", "62 12 35 01"),
            // a filter of the wrong length leaves no answer ID
            ("ATCF 7E", "OK"),
            ("22 1236", "62 12 36 01"),
        ]);
        let e = &c.exchanges;
        assert_eq!(e[0].rx, Some(0x7E8), "ATCFC1 did not replace the filter");
        assert_eq!(e[1].rx, Some(0x7E8), "ATCFC0 did not replace the filter");
        assert_eq!(e[2].rx, None);
    }

    #[test]
    fn an_empty_command_repeats_the_last_and_a_cut_capture_is_noted() {
        let mut chunks = vec![
            Chunk {
                at: 0.0,
                clock: None,
                adapter: false,
                bytes: b"22 1234\r".to_vec(),
            },
            Chunk {
                at: 0.1,
                clock: None,
                adapter: true,
                bytes: b"62 12 34 01\r\r>".to_vec(),
            },
            Chunk {
                at: 0.2,
                clock: None,
                adapter: false,
                bytes: b"\r".to_vec(),
            },
            Chunk {
                at: 0.3,
                clock: None,
                adapter: true,
                bytes: b"62 12 34 02\r\r>".to_vec(),
            },
            Chunk {
                at: 0.4,
                clock: None,
                adapter: false,
                bytes: b"22 5678\r".to_vec(),
            },
        ];
        chunks.push(Chunk {
            at: 0.5,
            clock: None,
            adapter: true,
            bytes: b"62 56".to_vec(),
        });
        let c = conversation(&chunks);
        assert_eq!(c.exchanges.len(), 3);
        assert_eq!(c.exchanges[1].request, vec![0x22, 0x12, 0x34]);
        assert_eq!(c.exchanges[1].replies[0].data, vec![0x62, 0x12, 0x34, 2]);
        assert!(c.exchanges[2]
            .problem
            .as_deref()
            .unwrap()
            .contains("capture ends"));
    }

    #[test]
    fn junk_is_never_a_panic() {
        let c = talk(&[
            ("ATH1", "OK"),
            (
                "22 12",
                "7E8 \r7E8 1\r12\r7E8 21 00\rZZZ 01 02\r7E8 10 FF 62",
            ),
            ("ATSHZZZ", "OK"),
            ("hello", "?"),
            ("ATH0", "OK"),
            ("22 12", "0: 62\r1:\r:\rFFF\r0:"),
        ]);
        assert_eq!(c.unreadable, 1);
        assert_eq!(c.exchanges.len(), 2);
    }
}
