//! Module data learned from a capture of another tool at work: a phone app talking to its Bluetooth adapter (`btlog`, the
//! phone's Bluetooth log) or any tool on the bus, recorded by `icarus bus watch --raw` through a Y-splitter (`buslog`).
//! Both become the same thing: a timed list of **exchanges** (a request a module was sent, and what came back), which
//! gives a transcript (to match against a screen recording of the app) and a module-data set: every identifier a module
//! answered on a data read (21, 22; a 22 read of several identifiers at once counts as a read of each; K-line and J1850
//! modules are known by their address) becomes a request with one placeholder value covering its answer's bytes, unnamed
//! and not verified, with the answers seen kept as samples. Identity reads (1A) are listed as seen, not imported (a
//! module's identity is read in a later build).
//!
//! **Privacy.** A Bluetooth capture holds everything the phone sent and received over Bluetooth. Only the diagnostic
//! conversation is kept: the readers never read Bluetooth addresses, device names or link keys (HCI commands and events
//! are skipped whole), and a channel that is not an adapter's conversation is dropped as it is found, never written or
//! quoted in a message. VINs and VIN-like text (17 or more characters a VIN may hold) are masked in the transcript and
//! the set unless the caller keeps them (`--keep-vin`): in requests (a VIN written with 2E F190 or 3B 90) and answers
//! (22 F190, 1A 90, 09 02), also where the VIN is split over messages (09 02 on K-line and J1850); adapter commands (AT, ST) are counted, never
//! written (their answers hold the adapter's serial number and the battery voltage).

pub mod btlog;
pub mod buslog;
pub mod elm;
#[cfg(test)]
pub(crate) mod testkit;

use super::{
    sha256_hex, validate, Ids, Imported, ModuleDef, ModuleHead, RequestDef, SetInfo, Source,
    ValueDef, ValueKind, FORMAT,
};
use crate::diag;
use std::collections::{BTreeMap, BTreeSet};

/// What every set learned from a capture says about itself (shown wherever its values appear).
pub const ATTRIBUTION: &str =
    "learned from a capture of another tool: values unnamed and not verified";

/// The attribution of a set learned from a capture as its values get named by hand (`icarus data name`, ICA-61): `named`
/// of the `total` values a capture made (its placeholders and the values named from them). Naming is not checking: the set
/// still says its values are not verified.
pub fn attribution_named(named: usize, total: usize) -> String {
    if named == 0 {
        ATTRIBUTION.into()
    } else if named >= total {
        "learned from a capture of another tool: values named by hand, not verified".into()
    } else {
        format!(
            "learned from a capture of another tool: {named} of {total} values named by hand, the rest unnamed; not verified"
        )
    }
}

/// Whether a set's attribution is one a capture wrote (and so may be brought up to date): a hand-written one is kept.
pub fn is_capture_attribution(a: &str) -> bool {
    a.starts_with("learned from a capture of another tool:")
}

/// A capture's values in these modules: (named by hand, all of them). A placeholder's note says "not named, not checked";
/// a named one's "named by hand".
pub fn naming_counts(modules: &[ModuleDef]) -> (usize, usize) {
    let mut named = 0;
    let mut total = 0;
    for v in modules
        .iter()
        .flat_map(|m| &m.requests)
        .flat_map(|r| &r.values)
    {
        if v.note.contains("named by hand") {
            named += 1;
            total += 1;
        } else if v.note.contains("not named, not checked") {
            total += 1;
        }
    }
    (named, total)
}

/// Different answers kept per request as samples.
pub const MAX_SAMPLES: usize = 8;

/// The year range that means "every year" (a capture's requests are marked unverified for all of them).
const EVERY_YEAR: [u16; 2] = [0, 9999];

/// One message that came back.
#[derive(Clone, Debug, PartialEq)]
pub struct Reply {
    /// The CAN ID it came from, when the capture shows it.
    pub from: Option<u32>,
    pub data: Vec<u8>,
}

/// A request and what came back.
#[derive(Clone, Debug, PartialEq)]
pub struct Exchange {
    /// Seconds from the start of the capture.
    pub at: f64,
    /// Wall-clock time (seconds since 1970, UTC) when the capture holds it (Bluetooth logs and `candump -l` do;
    /// `icarus bus watch` counts from when watching began).
    pub clock: Option<f64>,
    pub ids: Ids,
    /// The request ID; None when the adapter was left on its default header (every module).
    pub tx: Option<u32>,
    /// An extended address byte in front of each message (Toyota's modules behind 750, BMW).
    pub ext: Option<u8>,
    /// The answer ID the adapter was told to listen to (ATCRA), for answers that do not name their sender.
    pub rx: Option<u32>,
    pub request: Vec<u8>,
    pub replies: Vec<Reply>,
    /// What came instead of an answer ("NO DATA", "CAN ERROR", "no answer"...).
    pub problem: Option<String>,
    /// A VIN or VIN-like text in the request or the replies was masked.
    pub masked: bool,
}

impl Exchange {
    /// "7E0", "750.40", "18DA10F1", "7DF (every module)".
    pub fn module(&self) -> String {
        match self.tx {
            None => "adapter default (every module)".into(),
            Some(t) if is_functional(self.ids, t, self.ext) => {
                format!("{} (every module)", id_label(self.ids, t, self.ext))
            }
            Some(t) => id_label(self.ids, t, self.ext),
        }
    }
}

/// BMW's tester ID: requests go out on 6F1 with the module's address first, and come back on 600 + that address with the
/// tester's address F1 first (DF first: every module).
pub const BMW_TESTER: u32 = 0x6F1;

/// Whether a request is BMW-style: on 6F1 with a module's address first.
pub fn is_bmw(ids: Ids, tx: u32, ext: Option<u8>) -> bool {
    ids == Ids::Can11 && tx == BMW_TESTER && ext.is_some()
}

/// An ID as Icarus writes it: three hex digits for 11-bit, eight for 29-bit, ".EE" for an extended address.
pub fn id_label(ids: Ids, id: u32, ext: Option<u8>) -> String {
    let id = match ids {
        Ids::Can29 => format!("{id:08X}"),
        _ => format!("{id:03X}"),
    };
    match ext {
        Some(e) => format!("{id}.{e:02X}"),
        None => id,
    }
}

/// Whether a request (its ID and extended address) is the one every module hears: 7DF, BMW's 6F1 to DF, a 29-bit ID with
/// the functional format DB (18DB33F1), or K-line and J1850 header bytes to a functional address (33: ISO 14230; 6A:
/// ISO 9141 and J1850) or with KWP2000's functional format (C0-FF; a J1850 header of priority 6 or 7 is taken for one
/// too: more is left out rather than less).
pub fn is_functional(ids: Ids, tx: u32, ext: Option<u8>) -> bool {
    match ids {
        Ids::Can11 => tx == 0x7DF || (is_bmw(ids, tx, ext) && ext == Some(0xDF)),
        Ids::Can29 => (tx >> 16) & 0xFF == 0xDB,
        Ids::Kline => (tx >> 16) & 0xFF >= 0xC0 || matches!((tx >> 8) & 0xFF, 0x33 | 0x6A),
    }
}

/// The answer ID a request (its ID and extended address) usually has: request + 8 (11-bit), GM's 24x answered on 64x,
/// BMW's 6F1 answered on 600 + the module's address, or the two address bytes swapped (29-bit, 18DA); a K-line or J1850
/// module, known by its address (one byte), answers from that address.
pub fn usual_rx(ids: Ids, tx: u32, ext: Option<u8>) -> Option<u32> {
    match ids {
        _ if is_bmw(ids, tx, ext) => ext.map(|a| 0x600 | u32::from(a)),
        Ids::Can11 if (0x240..=0x24F).contains(&tx) => Some(tx + 0x400),
        Ids::Can11 if tx <= 0x7F7 => Some(tx + 8),
        Ids::Can29 if (tx >> 16) & 0xFF == 0xDA => {
            Some((tx & 0xFFFF_0000) | ((tx & 0xFF) << 8) | ((tx >> 8) & 0xFF))
        }
        Ids::Kline if tx <= 0xFF => Some(tx),
        _ => None,
    }
}

/// Bytes as "62 12 34".
pub fn hex(b: &[u8]) -> String {
    b.iter()
        .map(|x| format!("{x:02X}"))
        .collect::<Vec<_>>()
        .join(" ")
}

/// A character a VIN may hold: digits and capital letters but I, O and Q.
fn vin_char(b: u8) -> bool {
    b.is_ascii_digit() || (b.is_ascii_uppercase() && !matches!(b, b'I' | b'O' | b'Q'))
}

/// The runs of 17 or more VIN characters in `data`, as (start, length).
fn vin_runs(data: &[u8]) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < data.len() {
        if vin_char(data[i]) {
            let start = i;
            while i < data.len() && vin_char(data[i]) {
                i += 1;
            }
            if i - start >= 17 {
                out.push((start, i - start));
            }
        } else {
            i += 1;
        }
    }
    out
}

/// Where a byte of an exchange is: the request (None) or a reply (Some(index)), and the byte's position there.
type At = (Option<usize>, usize);

/// The texts a VIN may be read across in an exchange, each as the positions of its bytes in order: the request; each
/// sender's replies run together (a VIN can be split over messages); and for 09 02 each sender's replies after their
/// `49 02 NN` prefix run together (K-line and J1850 give the VIN as five messages of four bytes).
fn vin_texts(e: &Exchange) -> Vec<Vec<At>> {
    let mut out = vec![(0..e.request.len()).map(|j| (None, j)).collect::<Vec<At>>()];
    let mut senders: Vec<Option<u32>> = Vec::new();
    for r in &e.replies {
        if !senders.contains(&r.from) {
            senders.push(r.from);
        }
    }
    let vin_request = e.request.starts_with(&[0x09, 0x02]);
    for from in senders {
        let mine = || {
            e.replies
                .iter()
                .enumerate()
                .filter(move |(_, r)| r.from == from)
        };
        out.push(
            mine()
                .flat_map(|(i, r)| (0..r.data.len()).map(move |j| (Some(i), j)))
                .collect(),
        );
        if vin_request {
            out.push(
                mine()
                    .filter(|(_, r)| r.data.starts_with(&[0x49, 0x02]))
                    .flat_map(|(i, r)| {
                        (3.min(r.data.len())..r.data.len()).map(move |j| (Some(i), j))
                    })
                    .collect(),
            );
        }
    }
    out
}

fn byte_at(e: &Exchange, (part, j): At) -> u8 {
    match part {
        None => e.request[j],
        Some(i) => e.replies[i].data[j],
    }
}

/// The bytes the VIN itself fills, wherever the exchange is a VIN's own identifier: after `22 F1 90`/`62 F1 90` and
/// `2E F1 90` (UDS), `1A 90`/`5A 90` and `3B 90` (KWP), and `49 02 NN` (OBD-II 09 02); for a `22` read of several
/// identifiers with F190 among them, everything after F190's echo in the answer.
fn vin_fields(e: &Exchange) -> Vec<At> {
    let mut out = Vec::new();
    let req = &e.request;
    for (prefix, skip) in [(&[0x2E, 0xF1, 0x90][..], 3), (&[0x3B, 0x90][..], 2)] {
        if req.starts_with(prefix) {
            out.extend((skip..req.len()).map(|j| (None, j)));
        }
    }
    if req.len() > 3
        && !req.starts_with(&[0x22, 0xF1, 0x90])
        && multi_read(req).is_some_and(|ids| ids.contains(&0xF190))
    {
        for (i, r) in e.replies.iter().enumerate() {
            if r.data.first() != Some(&0x62) {
                continue;
            }
            if let Some(p) = r.data.windows(2).skip(1).position(|w| w == [0xF1, 0x90]) {
                out.extend((p + 3..r.data.len()).map(|j| (Some(i), j)));
            }
        }
    }
    let answer: Option<(&[u8], usize)> = if req.starts_with(&[0x22, 0xF1, 0x90]) {
        Some((&[0x62, 0xF1, 0x90], 3))
    } else if req.starts_with(&[0x1A, 0x90]) {
        Some((&[0x5A, 0x90], 2))
    } else if req.starts_with(&[0x09, 0x02]) {
        Some((&[0x49, 0x02], 3))
    } else {
        None
    };
    if let Some((prefix, skip)) = answer {
        for (i, r) in e.replies.iter().enumerate() {
            if r.data.starts_with(prefix) {
                out.extend((skip.min(r.data.len())..r.data.len()).map(|j| (Some(i), j)));
            }
        }
    }
    out
}

/// The value a VIN character counts for in the check digit (49 CFR 565).
fn vin_value(c: u8) -> u32 {
    match c {
        b'0'..=b'9' => u32::from(c - b'0'),
        b'A' | b'J' => 1,
        b'B' | b'K' | b'S' => 2,
        b'C' | b'L' | b'T' => 3,
        b'D' | b'M' | b'U' => 4,
        b'E' | b'N' | b'V' => 5,
        b'F' | b'W' => 6,
        b'G' | b'P' | b'X' => 7,
        b'H' | b'Y' => 8,
        _ => 9, // R, Z
    }
}

/// Whether 17 VIN characters read as a VIN: its check digit adds up, or position 10 holds a model-year code (a VIN from
/// outside North America need not use the check digit). Calibration IDs and other text that only looks like one fail.
fn plausible_vin(v: &[u8]) -> bool {
    const WEIGHTS: [u32; 17] = [8, 7, 6, 5, 4, 3, 2, 10, 0, 9, 8, 7, 6, 5, 4, 3, 2];
    if v.len() != 17 || !v.iter().all(|c| vin_char(*c)) {
        return false;
    }
    let sum: u32 = v.iter().zip(WEIGHTS).map(|(c, w)| vin_value(*c) * w).sum();
    let check = match sum % 11 {
        10 => b'X',
        r => b'0' + r as u8,
    };
    let year = v[9];
    check == v[8]
        || (year.is_ascii_uppercase() && !matches!(year, b'U' | b'Z'))
        || matches!(year, b'1'..=b'9')
}

/// The first VIN in the capture (to choose the vehicle's make; never written anywhere): the first 17 characters of a VIN
/// identifier's field (after any padding), or a run of exactly 17 VIN characters in a request or across an exchange's
/// replies, that reads as a VIN.
pub fn find_vin(ex: &[Exchange]) -> Option<String> {
    ex.iter().find_map(|e| {
        let field: Vec<u8> = vin_fields(e)
            .into_iter()
            .map(|at| byte_at(e, at))
            .skip_while(|b| matches!(b, 0x00 | 0x20 | 0xFF))
            .take(17)
            .collect();
        if plausible_vin(&field) {
            return Some(String::from_utf8_lossy(&field).into_owned());
        }
        vin_texts(e).iter().find_map(|t| {
            let bytes: Vec<u8> = t.iter().map(|at| byte_at(e, *at)).collect();
            vin_runs(&bytes)
                .into_iter()
                .map(|(s, len)| &bytes[s..s + len])
                .find(|v| plausible_vin(v))
                .map(|v| String::from_utf8_lossy(v).into_owned())
        })
    })
}

/// Masks every VIN and VIN-like run (each byte becomes `*`, 2A) in the requests and the replies: every run of 17 or more
/// VIN characters, also where it is split over replies, and the whole field of a VIN's own identifier (22 F190, 1A 90,
/// 09 02, and the writes 2E F190 and 3B 90) whatever it holds. More is masked than need be rather than less. Returns how
/// many exchanges had one.
pub fn mask_vins(ex: &mut [Exchange]) -> usize {
    let mut n = 0;
    for e in ex.iter_mut() {
        let mut hide: Vec<At> = vin_fields(e);
        for t in vin_texts(e) {
            let bytes: Vec<u8> = t.iter().map(|at| byte_at(e, *at)).collect();
            for (s, len) in vin_runs(&bytes) {
                hide.extend_from_slice(&t[s..s + len]);
            }
        }
        for (part, j) in hide {
            let b = match part {
                None => &mut e.request[j],
                Some(i) => &mut e.replies[i].data[j],
            };
            if *b != b'*' {
                *b = b'*';
                e.masked = true;
            }
        }
        n += usize::from(e.masked);
    }
    n
}

/// What the command line knows that the capture does not.
#[derive(Clone, Debug, Default)]
pub struct Options {
    /// The capture's file name (without its folder).
    pub file_name: String,
    /// `--name`: the set's name (else from the file name).
    pub name: Option<String>,
    /// The vehicle profile it is for ("" when not known).
    pub make: String,
    /// Model years it is for (from the VIN in the capture, when there is one).
    pub years: Vec<[u16; 2]>,
    /// Today (YYYY-MM-DD).
    pub today: String,
    /// "Bluetooth capture" or "bus capture".
    pub from: String,
    /// The capture's bytes, for its sha256.
    pub sha256: String,
}

/// A set name from a file name: lowercase letters, digits and dashes ("OBDLink 2026-11-19.pklg" -> "obdlink-2026-11-19").
pub fn name_from_file(file: &str) -> String {
    let stem = file.rsplit_once('.').map(|(a, _)| a).unwrap_or(file);
    let mut out = String::new();
    for c in stem.chars() {
        if c.is_ascii_alphanumeric() {
            out.push(c.to_ascii_lowercase());
        } else if !out.ends_with('-') {
            out.push('-');
        }
    }
    let out = out.trim_matches('-').to_string();
    if out.is_empty() {
        "capture".into()
    } else {
        out
    }
}

/// The bytes of a read request's identifier: two for 22, one for 21.
fn ident_len(sid: u8) -> usize {
    if sid == 0x22 {
        2
    } else {
        1
    }
}

/// The identifiers of a `22` read of several different ones at once (`22 1234 5678`), if `req` is one.
fn multi_read(req: &[u8]) -> Option<Vec<u16>> {
    if req.len() < 5 || req[0] != 0x22 || req.len().is_multiple_of(2) {
        return None;
    }
    let ids: Vec<u16> = req[1..]
        .chunks(2)
        .map(|c| u16::from_be_bytes([c[0], c[1]]))
        .collect();
    let different: BTreeSet<&u16> = ids.iter().collect();
    (different.len() == ids.len()).then_some(ids)
}

/// How a positive answer to a read of several identifiers splits into them: (identifier, data length) in order.
type Split = Vec<(u16, usize)>;

/// The most ways one answer is tried for splitting: more, and it is taken as not to be split.
const MAX_SPLITS: usize = 64;

/// Every way `a` (an answer after its 62) splits into parts of `ids`, in order: each part an identifier's echo and at
/// least one data byte, running to the next identifier's echo or the end; an identifier may be missing (a module leaves
/// out the ones it does not have). None: more ways than `MAX_SPLITS`.
fn splits(a: &[u8], ids: &[u16]) -> Option<Vec<Split>> {
    fn walk(a: &[u8], p: usize, ids: &[u16], next: usize, cur: &mut Split, out: &mut Vec<Split>) {
        if out.len() > MAX_SPLITS {
            return;
        }
        if p == a.len() {
            if !cur.is_empty() {
                out.push(cur.clone());
            }
            return;
        }
        let Some(echo) = a.get(p..p + 2).map(|b| u16::from_be_bytes([b[0], b[1]])) else {
            return;
        };
        for j in (next..ids.len()).filter(|j| ids[*j] == echo) {
            for q in p + 3..=a.len() {
                let at_next = a
                    .get(q..q + 2)
                    .is_some_and(|b| ids[j + 1..].contains(&u16::from_be_bytes([b[0], b[1]])));
                if q == a.len() || at_next {
                    cur.push((ids[j], q - p - 2));
                    walk(a, q, ids, j + 1, cur, out);
                    cur.pop();
                }
            }
        }
    }
    let mut out = Vec::new();
    walk(a, 0, ids, 0, &mut Vec::new(), &mut out);
    (out.len() <= MAX_SPLITS).then_some(out)
}

/// Whether `a` (an answer after its 62) is split `s`: each identifier's echo where the one before ends, up to the end.
fn fits_answer(a: &[u8], s: &Split) -> bool {
    let mut p = 0;
    for (id, len) in s {
        if a.get(p..p + 2) != Some(&id.to_be_bytes()[..]) {
            return false;
        }
        p += 2 + len;
    }
    p == a.len()
}

/// The split among `c` with the most identifiers in it, when only one has that many.
fn fullest(c: &BTreeSet<Split>) -> Option<&Split> {
    let most = c.iter().map(Vec::len).max()?;
    let mut best = c.iter().filter(|s| s.len() == most);
    let one = best.next()?;
    best.next().is_none().then_some(one)
}

/// One request per identifier for each `22` read of several identifiers at once (`22 1234 5678`) to one module, by the
/// exchange's index; Err(why) where its answer could not be split. The answer gives each identifier's echo, then its
/// data (`62 1234 .. 5678 ..`), and leaves out an identifier the module does not have; how long each one's data is, is
/// not in it. The split taken is the one that fits every answer to that same request and the data lengths single reads
/// of those identifiers gave, the one with the most identifiers in it when there are several; an answer that still
/// splits more than one way (or none) is not imported.
fn split_reads(ex: &[Exchange]) -> BTreeMap<usize, Result<Vec<Exchange>, String>> {
    // data lengths single reads gave: (kind of ID, request ID, extended address, identifier)
    let mut known: BTreeMap<(u8, u32, Option<u8>, u16), BTreeSet<usize>> = BTreeMap::new();
    for e in ex
        .iter()
        .filter(|e| e.request.len() == 3 && e.request[0] == 0x22)
    {
        let Some(tx) = e.tx else { continue };
        let ident = u16::from_be_bytes([e.request[1], e.request[2]]);
        for r in &e.replies {
            if r.data.len() > 3 && r.data[0] == 0x62 && r.data[1..3] == e.request[1..3] {
                known
                    .entry((e.ids as u8, tx, e.ext, ident))
                    .or_default()
                    .insert(r.data.len() - 3);
            }
        }
    }
    // the reads of several identifiers, by module and request, with the answer each got
    type Group<'a> = Vec<(usize, Option<&'a Reply>)>;
    let mut groups: BTreeMap<(u8, u32, Option<u8>, Vec<u8>), Group> = BTreeMap::new();
    for (i, e) in ex.iter().enumerate() {
        let (Some(ids), Some(tx)) = (multi_read(&e.request), e.tx) else {
            continue;
        };
        if is_functional(e.ids, tx, e.ext) {
            continue;
        }
        let usual = e.rx.or(usual_rx(e.ids, tx, e.ext));
        let positive: Vec<&Reply> = e
            .replies
            .iter()
            .filter(|r| {
                r.data.len() > 3
                    && r.data[0] == 0x62
                    && ids.contains(&u16::from_be_bytes([r.data[1], r.data[2]]))
            })
            .collect();
        let answer = positive
            .iter()
            .find(|r| r.from.is_some() && r.from == usual)
            .or(positive.first())
            .copied();
        groups
            .entry((e.ids as u8, tx, e.ext, e.request.clone()))
            .or_default()
            .push((i, answer));
    }
    let mut out = BTreeMap::new();
    for ((kind, tx, ext, request), members) in groups {
        let ids = multi_read(&request).unwrap_or_default();
        let fits = |s: &Split| {
            s.iter().all(|(id, len)| {
                known
                    .get(&(kind, tx, ext, *id))
                    .is_none_or(|lens| lens.contains(len))
            })
        };
        // each answer's splits (those that fit the single reads' lengths, when any do); None: too many to tell
        let ways: Vec<Option<BTreeSet<Split>>> = members
            .iter()
            .map(|(_, r)| {
                let all = splits(&(*r)?.data[1..], &ids)?;
                let fitting: BTreeSet<Split> = all.iter().filter(|s| fits(s)).cloned().collect();
                Some(if fitting.is_empty() {
                    all.into_iter().collect()
                } else {
                    fitting
                })
            })
            .collect();
        let mut common: Option<BTreeSet<Split>> = None;
        for w in ways.iter().flatten() {
            common = Some(match common {
                None => w.clone(),
                Some(c) => c.intersection(w).cloned().collect(),
            });
        }
        let common = common.filter(|c| !c.is_empty());
        for ((i, answer), own) in members.into_iter().zip(&ways) {
            let e = &ex[i];
            let part = |ident: u16, replies: Vec<Reply>, problem: Option<String>| Exchange {
                request: vec![0x22, (ident >> 8) as u8, ident as u8],
                replies,
                problem,
                ..e.clone()
            };
            let Some(r) = answer else {
                // refused or unanswered: so is each identifier
                let parts = ids
                    .iter()
                    .map(|id| part(*id, e.replies.clone(), e.problem.clone()))
                    .collect();
                out.insert(i, Ok(parts));
                continue;
            };
            let chosen = common
                .as_ref()
                .and_then(fullest)
                .filter(|s| {
                    own.as_ref()
                        .map_or_else(|| fits_answer(&r.data[1..], s), |w| w.contains(*s))
                })
                .or_else(|| own.as_ref().and_then(fullest));
            let Some(split) = chosen else {
                let how = match own {
                    Some(w) if w.is_empty() => "does not split into them",
                    _ => "splits into them more than one way",
                };
                let why =
                    format!("a read of several identifiers whose answer {how}: seen, not imported");
                out.insert(i, Err(why));
                continue;
            };
            let mut at = 1;
            let mut parts = Vec::new();
            for id in &ids {
                match split.iter().find(|(x, _)| x == id) {
                    Some((_, len)) => {
                        let data = r.data[at..at + 2 + len].to_vec();
                        at += 2 + len;
                        let mut whole = vec![0x62];
                        whole.extend_from_slice(&data);
                        parts.push(part(
                            *id,
                            vec![Reply {
                                from: r.from,
                                data: whole,
                            }],
                            None,
                        ));
                    }
                    None => parts.push(part(
                        *id,
                        vec![],
                        Some("left out of the module's answer to several identifiers".into()),
                    )),
                }
            }
            out.insert(i, Ok(parts));
        }
    }
    out
}

/// A request as (kind of ID, request ID, extended address, service, identifier).
type RequestKey = (u8, u32, Option<u8>, u8, u16);

/// One identifier a module answered, gathered over the whole capture.
struct Answered {
    ids: Ids,
    tx: u32,
    rx: u32,
    ext: Option<u8>,
    service: u8,
    ident: u16,
    /// Different whole positive answers, first seen first.
    answers: Vec<Vec<u8>>,
    times: usize,
    shortest: usize,
    longest: usize,
    masked: bool,
    rx_assumed: bool,
    /// The answers came from another ID than the usual one (and the adapter was given no answer ID).
    rx_odd: bool,
}

/// The set a capture's exchanges give: every module that answered a data read (21, 22), one request per identifier it
/// answered (a 22 read of several identifiers at once split into one per identifier, `split_reads`), each with one
/// placeholder value covering the answer's bytes. Everything else seen (1A identity reads among it) is listed in
/// `left_out` with why. The caller applies the vehicle gates (`icarus-obd`'s `moddata::gate_imported`) before writing it.
pub fn build(ex: &[Exchange], o: &Options) -> Result<Imported, String> {
    let mut out = Imported::default();
    let mut left: Vec<(String, String)> = Vec::new();
    let mut leave = |what: String, why: String| {
        if !left.iter().any(|(w, y)| *w == what && *y == why) {
            left.push((what, why));
        }
    };
    let mut got: Vec<Answered> = Vec::new();
    // (ids, tx, ext, service, ident) -> (what, why) for requests never answered with data
    let mut unanswered: BTreeMap<RequestKey, (String, String)> = BTreeMap::new();
    let split = split_reads(ex);
    for (i, whole) in ex.iter().enumerate() {
        // a read of several identifiers counts as one read of each
        let parts = match split.get(&i) {
            Some(Ok(parts)) => &parts[..],
            Some(Err(why)) => {
                leave(
                    format!("{} {}", whole.module(), hex(&whole.request)),
                    why.clone(),
                );
                continue;
            }
            None => std::slice::from_ref(whole),
        };
        for e in parts {
            let Some(&sid) = e.request.first() else {
                continue;
            };
            let what = format!(
                "{} {}",
                e.module(),
                hex(&e.request[..e.request.len().min(3)])
            );
            if sid == 0x1A {
                leave(
                    what,
                    "service 1A (a module's identity): seen, not imported (identity is read in a later build)".into(),
                );
                continue;
            }
            if !matches!(sid, 0x21 | 0x22) {
                leave(
                    what,
                    format!(
                        "service {sid:02X} ({}): seen, not imported (only data reads are: 21, 22)",
                        diag::service_name(sid)
                    ),
                );
                continue;
            }
            let Some(tx) = e.tx.filter(|t| !is_functional(e.ids, *t, e.ext)) else {
                leave(
                    what,
                    "sent to every module (7DF, a functional address or the adapter's default): seen, not imported (one module at a time only)"
                        .into(),
                );
                continue;
            };
            // a K-line or J1850 module is known by its address: its header's target byte (`81 10 F1`: 10)
            let tx = if e.ids == Ids::Kline {
                (tx >> 8) & 0xFF
            } else {
                tx
            };
            let n = 1 + ident_len(sid);
            if e.request.len() != n {
                leave(
                    what,
                    "not one identifier (more bytes than a single read): seen, not imported".into(),
                );
                continue;
            }
            let ident = e.request[1..n]
                .iter()
                .fold(0u16, |a, b| (a << 8) | u16::from(*b));
            let key = (e.ids as u8, tx, e.ext, sid, ident);
            let positive: Vec<&Reply> = e
                .replies
                .iter()
                .filter(|r| {
                    r.data.len() >= n && r.data[0] == sid + 0x40 && r.data[1..n] == e.request[1..n]
                })
                .collect();
            let usual = e.rx.or(usual_rx(e.ids, tx, e.ext));
            let Some(r) = positive
                .iter()
                .find(|r| r.from.is_some() && r.from == usual)
                .or(positive.first())
            else {
                let why = match e
                    .replies
                    .iter()
                    .find(|r| r.data.first() == Some(&0x7F) && r.data.get(1) == Some(&sid))
                {
                    Some(r) => format!(
                        "refused by the module (7F {sid:02X} {:02X}): not imported",
                        r.data.get(2).copied().unwrap_or(0)
                    ),
                    None => match &e.problem {
                        Some(p) => format!("no answer ({p}): not imported"),
                        None => "no answer that fits it: not imported".into(),
                    },
                };
                unanswered.entry(key).or_insert((what, why));
                continue;
            };
            let (rx, rx_assumed) = match r.from.or(e.rx) {
                Some(x) => (x, false),
                None => match usual {
                    Some(x) => (x, true),
                    None => {
                        leave(what, "the answer's ID is not known: not imported".into());
                        continue;
                    }
                },
            };
            let len = r.data.len() - n;
            match got.iter_mut().find(|a| {
                (a.ids, a.tx, a.rx, a.ext, a.service, a.ident) == (e.ids, tx, rx, e.ext, sid, ident)
            }) {
                Some(a) => {
                    a.times += 1;
                    a.shortest = a.shortest.min(len);
                    a.longest = a.longest.max(len);
                    a.masked |= e.masked;
                    if a.answers.len() < MAX_SAMPLES && !a.answers.contains(&r.data) {
                        a.answers.push(r.data.clone());
                    }
                }
                None => got.push(Answered {
                    ids: e.ids,
                    tx,
                    rx,
                    ext: e.ext,
                    service: sid,
                    ident,
                    answers: vec![r.data.clone()],
                    times: 1,
                    shortest: len,
                    longest: len,
                    masked: e.masked,
                    rx_assumed,
                    rx_odd: e.rx.is_none() && usual.is_some_and(|u| u != rx),
                }),
            }
        }
    }
    // a request refused or unanswered at one time but answered at another counts as answered
    for (k, (what, why)) in unanswered {
        if !got
            .iter()
            .any(|a| (a.ids as u8, a.tx, a.ext, a.service, a.ident) == k)
        {
            leave(what, why);
        }
    }
    let mut modules: Vec<ModuleDef> = Vec::new();
    for a in &got {
        let label = if a.service == 0x22 {
            format!("22 {:04X}", a.ident)
        } else {
            format!("{:02X} {:02X}", a.service, a.ident)
        };
        let what = format!("{} {label}", id_label(a.ids, a.tx, a.ext));
        if a.shortest == 0 {
            leave(
                what,
                "the answer carries no data after the identifier: not imported".into(),
            );
            continue;
        }
        let Ok(bits) = u16::try_from(a.shortest * 8) else {
            leave(
                what,
                format!(
                    "the answer is too long for one value ({} bytes): not imported",
                    a.shortest
                ),
            );
            continue;
        };
        let mut v = ValueDef::new(
            &format!("RAW_{}", label.replace(' ', "_")),
            &format!("{label} ({} bytes)", a.shortest),
            0,
            bits,
        );
        v.kind = ValueKind::Hex;
        let mut note = format!(
            "learned from a capture ({}): not named, not checked; answered {} time(s), {}",
            o.from,
            a.times,
            if a.shortest == a.longest {
                format!("{} bytes", a.shortest)
            } else {
                format!("{} to {} bytes", a.shortest, a.longest)
            }
        );
        if a.masked {
            note += "; VIN masked in the samples";
        }
        v.note = note;
        let r = RequestDef {
            service: a.service,
            ident: a.ident,
            every: None,
            min_len: None,
            timeout_ms: None,
            years: vec![],
            unverified: vec![EVERY_YEAR],
            needs_session: None,
            sample: a.answers.first().map(|b| hex(b)),
            samples: a.answers.iter().skip(1).map(|b| hex(b)).collect(),
            values: vec![v],
        };
        let idx = match modules.iter().position(|m| {
            let h = &m.module;
            (h.ids, h.tx, h.rx, h.ext) == (a.ids, a.tx, Some(a.rx), a.ext)
        }) {
            Some(i) => i,
            None => {
                if a.rx_odd {
                    out.notes.push(format!(
                        "{}: answered from {}, not its usual answer ID {}: check that the answer is this module's",
                        id_label(a.ids, a.tx, a.ext),
                        id_label(a.ids, a.rx, None),
                        usual_rx(a.ids, a.tx, a.ext)
                            .map(|u| id_label(a.ids, u, None))
                            .unwrap_or_default()
                    ));
                }
                if a.rx_assumed {
                    out.notes.push(format!(
                        "{}: the answers do not name their sender; {} assumed (the usual answer ID)",
                        id_label(a.ids, a.tx, a.ext),
                        id_label(a.ids, a.rx, None)
                    ));
                }
                modules.push(ModuleDef {
                    format: FORMAT,
                    module: ModuleHead {
                        name: match a.ids {
                            Ids::Kline => format!("module (K-line {:02X})", a.tx),
                            _ => format!("module ({})", id_label(a.ids, a.tx, a.ext)),
                        },
                        tx: a.tx,
                        rx: Some(a.rx),
                        ext: a.ext,
                        // BMW's answers carry the tester's address, not the module's
                        tester: is_bmw(a.ids, a.tx, a.ext).then_some(0xF1),
                        ids: a.ids,
                        bus: None,
                        dialect: None,
                        kind: String::new(),
                        identity: vec![],
                    },
                    requests: vec![],
                });
                modules.len() - 1
            }
        };
        let mut trial = modules[idx].clone();
        trial.requests.push(r);
        match validate(&mut trial) {
            Ok(()) => modules[idx] = trial,
            Err(e) => leave(what, e),
        }
    }
    modules.retain(|m| !m.requests.is_empty());
    let set = SetInfo {
        format: FORMAT,
        name: o
            .name
            .clone()
            .unwrap_or_else(|| name_from_file(&o.file_name)),
        make: o.make.clone(),
        models: vec![],
        years: o.years.clone(),
        source: Source {
            from: o.from.clone(),
            file: o.file_name.clone(),
            sha256: o.sha256.clone(),
            imported: o.today.clone(),
            license: String::new(),
            attribution: ATTRIBUTION.into(),
        },
    };
    super::check_set(&set)?;
    out.set = Some(set);
    out.modules = modules;
    out.left_out = left;
    Ok(out)
}

/// "2026-11-19T14:03:05.123Z" from seconds since 1970 (UTC), to the nearest millisecond (a log's `.100000` reads back
/// as a hair under it).
pub fn utc(secs: f64) -> String {
    if !secs.is_finite() || secs < 0.0 {
        return String::new();
    }
    let total_ms = (secs * 1000.0).round() as i64;
    let (whole, ms) = (total_ms.div_euclid(1000), total_ms.rem_euclid(1000));
    let (days, rest) = (whole.div_euclid(86_400), whole.rem_euclid(86_400));
    // days since 1970-01-01 to a civil date (the well-known era arithmetic)
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!(
        "{y:04}-{m:02}-{d:02}T{:02}:{:02}:{:02}.{ms:03}Z",
        rest / 3600,
        rest % 3600 / 60,
        rest % 60
    )
}

fn csv_field(s: &str) -> String {
    if s.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", s.replace('"', "\"\""))
    } else {
        s.to_string()
    }
}

/// The transcript as CSV: `time_s` (from the start of the capture), `clock_utc` (when the capture holds the time of day),
/// `module` (request ID, and answer ID when known), `service`, `request`, `answer`. `nrc` names a refusal's code.
pub fn transcript_csv(ex: &[Exchange], nrc: &dyn Fn(u8) -> String) -> String {
    let mut o = String::from("time_s,clock_utc,module,service,request,answer\n");
    for e in ex {
        let rx: Vec<String> = {
            let mut v: Vec<String> = e
                .replies
                .iter()
                .filter_map(|r| r.from)
                .map(|f| id_label(e.ids, f, None))
                .collect();
            v.dedup();
            if v.is_empty() {
                v.extend(e.rx.map(|r| id_label(e.ids, r, None)));
            }
            v
        };
        let module = if rx.is_empty() {
            e.module()
        } else {
            format!("{}/{}", e.module(), rx.join("+"))
        };
        let answer = if e.replies.is_empty() {
            e.problem.clone().unwrap_or_else(|| "no answer".into())
        } else {
            let mut parts: Vec<String> = e
                .replies
                .iter()
                .map(|r| {
                    let mut s = String::new();
                    if let Some(f) = r.from {
                        if e.replies.iter().any(|q| q.from != r.from) {
                            s += &format!("{}: ", id_label(e.ids, f, None));
                        }
                    }
                    s += &hex(&r.data);
                    if r.data.first() == Some(&0x7F) && r.data.len() >= 3 {
                        s += &format!(" (refused: {})", nrc(r.data[2]));
                    }
                    s
                })
                .collect();
            if e.masked {
                parts.push("[VIN masked]".into());
            }
            parts.join("; ")
        };
        let service = e
            .request
            .first()
            .map(|s| diag::service_name(*s))
            .unwrap_or("");
        let line = [
            format!("{:.3}", e.at),
            e.clock.map(utc).unwrap_or_default(),
            module,
            service.to_string(),
            hex(&e.request),
            answer,
        ];
        o += &line
            .iter()
            .map(|f| csv_field(f))
            .collect::<Vec<_>>()
            .join(",");
        o.push('\n');
    }
    o
}

/// What a capture file held: which reader read it, the exchanges in time order, and counts of what was dropped (never its
/// content).
#[derive(Clone, Debug, Default)]
pub struct Read {
    /// "Bluetooth capture" or "bus capture".
    pub from: &'static str,
    /// The file's format in words.
    pub format: String,
    pub exchanges: Vec<Exchange>,
    /// What was seen and dropped, in counts and words (no content).
    pub notes: Vec<String>,
}

/// Reads a capture: a Bluetooth log (pcap with HCI H4 frames, or Apple PacketLogger) or a bus log (`icarus bus watch
/// --raw`, candump), told apart by their first bytes.
pub fn read(bytes: &[u8]) -> Result<Read, String> {
    if btlog::looks_binary(bytes) {
        btlog::read(bytes)
    } else {
        let text = std::str::from_utf8(bytes)
            .map_err(|_| "not a capture Icarus reads: neither a Bluetooth log (pcap, PacketLogger .pklg) nor a bus log (text)".to_string())?;
        buslog::read(text)
    }
}

/// The sha256 of a capture (kept in `set.toml`, so the set can be traced to its file).
pub fn digest(bytes: &[u8]) -> String {
    sha256_hex(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ex(tx: u32, req: &[u8], replies: &[(u32, &[u8])]) -> Exchange {
        Exchange {
            at: 1.0,
            clock: None,
            ids: Ids::Can11,
            tx: Some(tx),
            ext: None,
            rx: None,
            request: req.to_vec(),
            replies: replies
                .iter()
                .map(|(f, d)| Reply {
                    from: Some(*f),
                    data: d.to_vec(),
                })
                .collect(),
            problem: None,
            masked: false,
        }
    }

    fn opts() -> Options {
        Options {
            file_name: "Truck run.pklg".into(),
            today: "2026-11-19".into(),
            from: "Bluetooth capture".into(),
            make: "toyota".into(),
            ..Default::default()
        }
    }

    #[test]
    fn answered_reads_become_unnamed_unverified_values_with_their_samples() {
        let list = vec![
            ex(
                0x7E0,
                &[0x22, 0x12, 0x34],
                &[(0x7E8, &[0x62, 0x12, 0x34, 1, 2, 3])],
            ),
            ex(
                0x7E0,
                &[0x22, 0x12, 0x34],
                &[(0x7E8, &[0x62, 0x12, 0x34, 1, 2, 4])],
            ),
            ex(
                0x7E0,
                &[0x22, 0x12, 0x34],
                &[(0x7E8, &[0x62, 0x12, 0x34, 1, 2, 4, 9])],
            ),
            ex(0x7E0, &[0x21, 0x05], &[(0x7E8, &[0x61, 0x05, 0xAA])]),
            ex(
                0x7E0,
                &[0x2E, 0x12, 0x34, 0x01],
                &[(0x7E8, &[0x6E, 0x12, 0x34])],
            ),
            ex(0x7E0, &[0x22, 0x56, 0x78], &[(0x7E8, &[0x7F, 0x22, 0x31])]),
            ex(
                0x7DF,
                &[0x22, 0xF1, 0x90],
                &[(0x7E8, &[0x62, 0xF1, 0x90, 0x41])],
            ),
            ex(0x7E0, &[0x22, 0x9A, 0xBC], &[(0x7E8, &[0x62, 0x9A, 0xBC])]),
        ];
        let imp = build(&list, &opts()).unwrap();
        let s = imp.set.as_ref().unwrap();
        assert_eq!(s.name, "truck-run");
        assert_eq!(
            (s.source.from.as_str(), s.make.as_str()),
            ("Bluetooth capture", "toyota")
        );
        assert_eq!(imp.modules.len(), 1);
        let m = &imp.modules[0];
        assert_eq!((m.module.tx, m.module.rx), (0x7E0, Some(0x7E8)));
        assert_eq!(m.requests.len(), 2);
        let r = &m.requests[0];
        assert_eq!((r.service, r.ident), (0x22, 0x1234));
        assert_eq!(r.unverified, vec![[0, 9999]]);
        assert_eq!(r.sample.as_deref(), Some("62 12 34 01 02 03"));
        assert_eq!(r.samples, vec!["62 12 34 01 02 04", "62 12 34 01 02 04 09"]);
        let v = &r.values[0];
        assert_eq!(
            (v.id.as_str(), v.name.as_str()),
            ("RAW_22_1234", "22 1234 (3 bytes)")
        );
        assert_eq!((v.kind, v.bits, v.checked), (ValueKind::Hex, 24, false));
        assert!(v.note.contains("3 to 4 bytes"), "{}", v.note);
        assert_eq!(m.requests[1].label(), "21 05");
        let why = |w: &str| {
            imp.left_out
                .iter()
                .find(|(x, _)| x.contains(w))
                .map(|x| x.1.clone())
                .unwrap_or_else(|| panic!("{w}: {:?}", imp.left_out))
        };
        assert!(why("2E 12 34").contains("seen, not imported"));
        assert!(why("22 56 78").contains("refused"));
        assert!(why("7DF").contains("every module"));
        assert!(why("22 9ABC").contains("no data"));
        // the files written read back
        for (name, text) in super::super::set_files(s, &imp.modules).iter().skip(1) {
            assert_eq!(&super::super::parse_module(text, name).unwrap(), m);
        }
    }

    #[test]
    fn a_refusal_at_one_time_and_an_answer_at_another_is_answered() {
        let list = vec![
            ex(0x7E0, &[0x22, 0x12, 0x34], &[(0x7E8, &[0x7F, 0x22, 0x22])]),
            ex(
                0x7E0,
                &[0x22, 0x12, 0x34],
                &[(0x7E8, &[0x62, 0x12, 0x34, 1])],
            ),
        ];
        let imp = build(&list, &opts()).unwrap();
        assert!(imp.left_out.is_empty(), "{:?}", imp.left_out);
        assert_eq!(imp.modules[0].requests.len(), 1);
    }

    #[test]
    fn vins_are_found_and_masked() {
        let mut list = vec![ex(
            0x7E0,
            &[0x22, 0xF1, 0x90],
            &[(0x7E8, b"\x62\xF1\x90JTEST0000H0000001")],
        )];
        assert_eq!(find_vin(&list).as_deref(), Some("JTEST0000H0000001"));
        assert_eq!(mask_vins(&mut list), 1);
        assert!(list[0].masked);
        assert_eq!(&list[0].replies[0].data[3..], &[b'*'; 17]);
        let csv = transcript_csv(&list, &|c| format!("code {c:02X}"));
        assert!(
            !csv.contains("4A 54") && csv.contains("[VIN masked]"),
            "{csv}"
        );
        let imp = build(&list, &opts()).unwrap();
        let r = &imp.modules[0].requests[0];
        assert!(r.sample.as_deref().unwrap().ends_with("2A 2A"));
        assert!(r.values[0].note.contains("VIN masked"));
        // a short run (a part number) is not a VIN
        let mut short = vec![ex(0x7E0, &[0x1A, 0x87], &[(0x7E8, b"\x5A\x8789661-04A10")])];
        assert_eq!(mask_vins(&mut short), 0);
    }

    /// A made-up VIN (position 10 a model-year code) and its bytes as hex.
    const VIN: &[u8; 17] = b"JTEST0000H0000001";
    const VIN_HEX: &str = "4A 54 45 53 54 30 30 30 30 48 30 30 30 30 30 30 31";

    fn assert_no_vin(list: &[Exchange]) {
        let csv = transcript_csv(list, &|_| String::new());
        // any 4 bytes of it in a row would be too much
        let parts: Vec<&str> = VIN_HEX.split(' ').collect();
        for w in parts.windows(4) {
            assert!(!csv.contains(&w.join(" ")), "{} in {csv}", w.join(" "));
        }
        assert!(csv.contains("[VIN masked]"), "{csv}");
    }

    #[test]
    fn a_vin_in_a_request_is_masked() {
        let mut uds = vec![0x2E, 0xF1, 0x90];
        uds.extend_from_slice(VIN);
        let mut kwp = vec![0x3B, 0x90];
        kwp.extend_from_slice(VIN);
        let mut list = vec![
            ex(0x7E0, &uds, &[(0x7E8, &[0x6E, 0xF1, 0x90])]),
            ex(0x7E0, &kwp, &[(0x7E8, &[0x7B, 0x90])]),
        ];
        assert_eq!(find_vin(&list).as_deref(), Some("JTEST0000H0000001"));
        assert_eq!(mask_vins(&mut list), 2);
        assert_eq!(&list[0].request[3..], &[b'*'; 17]);
        assert_eq!(&list[1].request[2..], &[b'*'; 17]);
        assert_no_vin(&list);
        // a VIN write cut short (fewer than 17 characters) is masked too
        let mut cut = vec![ex(0x7E0, &uds[..12], &[])];
        assert_eq!(mask_vins(&mut cut), 1);
        assert_eq!(&cut[0].request[3..], &[b'*'; 9]);
    }

    #[test]
    fn a_vin_over_several_messages_is_found_and_masked() {
        // 09 02 on K-line or J1850: five messages of four bytes, the first padded with zeros
        let mut msgs: Vec<Vec<u8>> = Vec::new();
        let mut padded = vec![0u8; 3];
        padded.extend_from_slice(VIN);
        for (i, part) in padded.chunks(4).enumerate() {
            let mut m = vec![0x49, 0x02, i as u8 + 1];
            m.extend_from_slice(part);
            msgs.push(m);
        }
        let mut e = ex(0, &[0x09, 0x02], &[]);
        e.ids = Ids::Kline;
        e.tx = None;
        e.replies = msgs
            .iter()
            .map(|m| Reply {
                from: None,
                data: m.clone(),
            })
            .collect();
        // and the same VIN from a CAN module, but split over two replies with no prefix to go by
        let split = ex(
            0x7E0,
            &[0x21, 0x99],
            &[(0x7E8, b"\x61\x99JTEST000"), (0x7E8, b"0H0000001")],
        );
        let mut list = vec![e, split];
        assert_eq!(find_vin(&list).as_deref(), Some("JTEST0000H0000001"));
        assert_eq!(mask_vins(&mut list), 2);
        for r in &list[0].replies {
            assert!(r.data[3..].iter().all(|b| *b == b'*'), "{:?}", r.data);
            assert_eq!(&r.data[..2], &[0x49, 0x02], "the prefix stays");
        }
        assert_no_vin(&list);
    }

    #[test]
    fn the_vin_is_not_taken_from_calibration_ids() {
        // 09 04 first: two 16-character calibration IDs run together, then the VIN
        let mut cal = vec![0x49, 0x04, 0x02];
        cal.extend_from_slice(b"A0B1C2D3E4F5G6H7J8K9L0M1N2P3R4S5");
        let mut vin = vec![0x49, 0x02, 0x01];
        vin.extend_from_slice(VIN);
        let list = vec![
            ex(0x7DF, &[0x09, 0x04], &[(0x7E8, &cal)]),
            ex(0x7DF, &[0x09, 0x02], &[(0x7E8, &vin)]),
        ];
        assert_eq!(find_vin(&list).as_deref(), Some("JTEST0000H0000001"));
        // 17 characters that do not read as a VIN are not one
        let mut odd = vec![0x61, 0x10];
        odd.extend_from_slice(b"ABCDEFGHJ0LMNPRST");
        assert_eq!(
            find_vin(&[ex(0x7E0, &[0x21, 0x10], &[(0x7E8, &odd)])]),
            None
        );
    }

    #[test]
    fn several_identifiers_at_once_and_k_line_modules_are_imported() {
        use super::testkit::{pklg_file, Session};
        let cap = Session::talk(&[
            ("ATZ", "ELM327 v1.4b"),
            ("ATH1", "OK"),
            ("ATSH7E0", "OK"),
            // three identifiers at once: 1234 (3 bytes, the first time with 5678's echo inside), 5678 (1), 9ABC (3)
            (
                "22123456789ABC",
                "7E8 10 0E 62 12 34 01 56 78\r7E8 21 56 78 03 9A BC 04 05\r7E8 22 06 AA AA AA AA AA AA",
            ),
            (
                "22 1234 5678 9ABC",
                "7E8 10 0E 62 12 34 02 11 22\r7E8 21 56 78 04 9A BC 04 05\r7E8 22 07 AA AA AA AA AA AA",
            ),
            // an answer that splits more than one way, seen once
            (
                "2211112222",
                "7E8 10 09 62 11 11 01 22 22\r7E8 21 22 22 05 AA AA AA AA",
            ),
            ("22 3333 4444", "7E8 03 7F 22 31"),
            // K-line (ISO 14230 fast init) to the module at address 10, with headers and without
            ("ATSP5", "OK"),
            ("ATSH8110F1", "OK"),
            ("2101", "83 F1 10 61 01 05 EB"),
            ("ATH0", "OK"),
            ("2102", "61 02 06 07"),
            // to every module (KWP2000's functional address)
            ("ATSHC133F1", "OK"),
            ("2103", "61 03 01"),
        ]);
        let r = read(&pklg_file(&cap.packets, true)).unwrap();
        let imp = build(&r.exchanges, &opts()).unwrap();
        assert_eq!(imp.modules.len(), 2, "{:?}", imp.left_out);
        let ecm = &imp.modules[0];
        let labels: Vec<String> = ecm.requests.iter().map(|q| q.label()).collect();
        assert_eq!(labels, vec!["22 1234", "22 5678", "22 9ABC"]);
        let first = &ecm.requests[0];
        assert_eq!(first.sample.as_deref(), Some("62 12 34 01 56 78"));
        assert_eq!(first.samples, vec!["62 12 34 02 11 22"]);
        assert_eq!(first.values[0].bits, 24);
        assert_eq!(ecm.requests[1].sample.as_deref(), Some("62 56 78 03"));
        assert_eq!(ecm.requests[2].sample.as_deref(), Some("62 9A BC 04 05 06"));
        let k = &imp.modules[1];
        assert_eq!(
            (k.module.ids, k.module.tx, k.module.rx),
            (Ids::Kline, 0x10, Some(0x10))
        );
        assert_eq!(k.module.name, "module (K-line 10)");
        let labels: Vec<String> = k.requests.iter().map(|q| q.label()).collect();
        assert_eq!(labels, vec!["21 01", "21 02"]);
        assert_eq!(k.requests[1].sample.as_deref(), Some("61 02 06 07"));
        let why = |w: &str| {
            imp.left_out
                .iter()
                .find(|(x, _)| x.contains(w))
                .map(|x| x.1.clone())
                .unwrap_or_else(|| panic!("{w}: {:?}", imp.left_out))
        };
        assert!(why("22 11 11 22 22").contains("more than one way"));
        assert!(why("7E0 22 33 33").contains("refused"));
        assert!(why("7E0 22 44 44").contains("refused"));
        assert!(why("C133F1 (every module) 21 03").contains("every module"));
        // the files read back
        let s = imp.set.as_ref().unwrap();
        for ((_, text), m) in super::super::set_files(s, &imp.modules)
            .iter()
            .skip(1)
            .zip(&imp.modules)
        {
            assert_eq!(&super::super::parse_module(text, "test").unwrap(), m);
        }
    }

    #[test]
    fn identity_reads_are_seen_not_imported() {
        let list = vec![
            ex(0x7E0, &[0x1A, 0x87], &[(0x7E8, b"\x5A\x8789661-04A10")]),
            ex(0x7E0, &[0x21, 0x01], &[(0x7E8, &[0x61, 0x01, 0x05])]),
        ];
        let imp = build(&list, &opts()).unwrap();
        assert_eq!(imp.modules[0].requests.len(), 1);
        assert_eq!(imp.modules[0].requests[0].service, 0x21);
        let (w, why) = &imp.left_out[0];
        assert!(
            w.contains("1A 87") && why.contains("identity"),
            "{:?}",
            imp.left_out
        );
    }

    #[test]
    fn an_answer_too_long_for_one_value_is_left_out_not_the_import() {
        let mut long = vec![0x62, 0x12, 0x34];
        long.resize(3 + 9000, 0x01);
        let list = vec![
            ex(0x7E0, &[0x22, 0x12, 0x34], &[(0x7E8, &long)]),
            ex(
                0x7E0,
                &[0x22, 0x56, 0x78],
                &[(0x7E8, &[0x62, 0x56, 0x78, 0x02])],
            ),
        ];
        let imp = build(&list, &opts()).unwrap();
        assert_eq!(imp.modules[0].requests.len(), 1);
        assert_eq!(imp.modules[0].requests[0].ident, 0x5678);
        assert!(
            imp.left_out
                .iter()
                .any(|(w, y)| w.contains("22 1234") && y.contains("too long")),
            "{:?}",
            imp.left_out
        );
    }

    #[test]
    fn the_transcript_is_csv_with_times_modules_and_answers() {
        let mut a = ex(0x7E0, &[0x22, 0x12, 0x34], &[(0x7E8, &[0x7F, 0x22, 0x31])]);
        a.clock = Some(1_763_560_985.25);
        let mut b = ex(0x7E0, &[0x21, 0x01], &[]);
        b.problem = Some("NO DATA".into());
        let csv = transcript_csv(&[a, b], &|_| "request out of range".into());
        let lines: Vec<&str> = csv.lines().collect();
        assert_eq!(lines[0], "time_s,clock_utc,module,service,request,answer");
        assert_eq!(
            lines[1],
            "1.000,2025-11-19T14:03:05.250Z,7E0/7E8,read data by identifier,22 12 34,7F 22 31 (refused: request out of range)"
        );
        assert!(lines[2].ends_with(",21 01,NO DATA"), "{}", lines[2]);
    }

    #[test]
    fn ids_and_names() {
        assert_eq!(usual_rx(Ids::Can11, 0x7E0, None), Some(0x7E8));
        assert_eq!(usual_rx(Ids::Can29, 0x18DA10F1, None), Some(0x18DAF110));
        assert_eq!(usual_rx(Ids::Can29, 0x18DB33F1, None), None);
        assert!(
            is_functional(Ids::Can29, 0x18DB33F1, None) && is_functional(Ids::Can11, 0x7DF, None)
        );
        assert_eq!(
            name_from_file("OBDLink 2026-11-19.pklg"),
            "obdlink-2026-11-19"
        );
        assert_eq!(name_from_file("...pcap"), "capture");
        assert_eq!(utc(0.0), "1970-01-01T00:00:00.000Z");
        assert_eq!(utc(951_782_400.0), "2000-02-29T00:00:00.000Z");
    }
}
