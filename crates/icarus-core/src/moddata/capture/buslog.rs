//! A bus log: the CAN frames another tool and the modules sent each other, recorded through a Y-splitter by
//! `icarus bus watch --raw` (one frame a line: the time in seconds, the ID in hex, the data bytes in hex; the watch's own
//! words between them are skipped) or by Linux's `candump` (`(1700000000.123456) can0 7E0#0322123400000000`, or its
//! default `can0  7E0   [8]  03 22 12 34 00 00 00 00`).
//!
//! Diagnostic frames (11-bit IDs 700-7FF, 29-bit IDs 18DAxxxx and 18DBxxxx, and where the log shows them used that way
//! BMW's 6F1 and 6xx and GM's 24x and 64x, `make_ids`) are put back together per sender with ISO-TP
//! (`crate::diag`, the same code `icarus bus watch` uses); flow control frames carry no message. An ID whose frames make
//! more whole messages with an address byte in front (extended addressing: Toyota's modules behind 750, BMW) is read
//! that way, told by how its frames read over the whole log (an ID that reads as well both ways follows the ID it talks
//! to). Each request is then paired with its answers: the answer's service is the request's + 40 (or 7F naming it), it
//! comes from the usual answer ID (request + 8; the two address bytes swapped on 29-bit; 7E8-7EF for 7DF; behind a
//! gateway with the request's address byte first) or, failing that, from the module the request went to most recently
//! with that service, within five seconds (and among the last 64 requests still waiting). Every other frame on the bus is
//! ignored.

use super::{is_bmw, is_functional, usual_rx, Exchange, Read, Reply, BMW_TESTER};
use crate::diag::{self, Fed, Reassembler};
use crate::moddata::Ids;
use std::collections::{BTreeMap, BTreeSet};

/// How long a request waits for its answers (seconds).
const WAIT: f64 = 5.0;

/// The most requests waiting for answers at once: older ones stop waiting (a log without times, as candump's default
/// format, never lets five seconds pass).
const MAX_OPEN: usize = 64;

/// One CAN frame from the log.
#[derive(Clone, Debug, PartialEq)]
pub struct BusFrame {
    pub at: f64,
    pub ids: Ids,
    pub id: u32,
    pub data: Vec<u8>,
}

fn is_time(t: &str) -> Option<f64> {
    let t = t.trim_start_matches('(').trim_end_matches(')');
    (t.contains('.') && t.chars().all(|c| c.is_ascii_digit() || c == '.'))
        .then(|| t.parse().ok())
        .flatten()
}

fn hex_byte(t: &str) -> Option<u8> {
    (t.len() == 2)
        .then(|| u8::from_str_radix(t, 16).ok())
        .flatten()
}

fn id_of(t: &str) -> Option<(Ids, u32)> {
    if t.is_empty() || t.len() > 8 || !t.chars().all(|c| c.is_ascii_hexdigit()) {
        return None;
    }
    let v = u32::from_str_radix(t, 16).ok()?;
    if v > 0x1FFF_FFFF {
        return None;
    }
    Some(if t.len() == 8 || v > 0x7FF {
        (Ids::Can29, v)
    } else {
        (Ids::Can11, v)
    })
}

/// One line as a frame, if it is one.
pub fn parse_line(line: &str, last: f64) -> Option<BusFrame> {
    let toks: Vec<&str> = line.split_whitespace().collect();
    let mut i = 0;
    let mut at = None;
    while let Some(t) = toks.get(i).and_then(|t| is_time(t)) {
        at.get_or_insert(t);
        i += 1;
    }
    // an interface name (can0, vcan1, slcan0)
    if toks.get(i).is_some_and(|t| {
        t.chars().next().is_some_and(|c| c.is_ascii_alphabetic())
            && !t.chars().all(|c| c.is_ascii_hexdigit())
            && !t.contains('#')
    }) {
        i += 1;
    }
    let tok = toks.get(i)?;
    let (ids, id, data) = if let Some((id, data)) = tok.split_once('#') {
        if toks.len() != i + 1 || data.len() % 2 != 0 || data.len() > 128 {
            return None;
        }
        let (ids, id) = id_of(id)?;
        let data = (0..data.len())
            .step_by(2)
            .map(|k| hex_byte(&data[k..k + 2]))
            .collect::<Option<Vec<u8>>>()?;
        (ids, id, data)
    } else {
        let (ids, id) = id_of(tok)?;
        let mut rest = &toks[i + 1..];
        if rest
            .first()
            .is_some_and(|t| t.starts_with('[') && t.ends_with(']'))
        {
            rest = &rest[1..];
        }
        if rest.len() > 64 {
            return None;
        }
        let data = rest
            .iter()
            .map(|t| hex_byte(t))
            .collect::<Option<Vec<u8>>>()?;
        (ids, id, data)
    };
    Some(BusFrame {
        at: at.unwrap_or(last),
        ids,
        id,
        data,
    })
}

/// Whether an ID carries diagnostic messages on every make.
fn diagnostic(ids: Ids, id: u32) -> bool {
    match ids {
        Ids::Can11 => (0x700..=0x7FF).contains(&id),
        Ids::Can29 => matches!((id >> 16) & 0xFF, 0xDA | 0xDB),
        Ids::Kline => false,
    }
}

/// The 11-bit IDs outside 700-7FF read as diagnostic ones because the log shows them used that way: (BMW's, GM's).
/// BMW's tester ID 6F1 (each frame a module's address, then ISO-TP) and the answer IDs 600 + each address it was sent to
/// (each frame the tester's F1 first); GM's request IDs 240-24F with their answer IDs 640-64F, both in the log. An ID
/// counts only when its frames make whole messages read that way and nearly all fit (at most one in ten left over), so
/// other traffic on those IDs (a body module's frames) is not taken for diagnostics.
fn make_ids(frames: &[BusFrame]) -> (BTreeSet<u32>, BTreeSet<u32>) {
    let mut by: BTreeMap<u32, Vec<&[u8]>> = BTreeMap::new();
    for f in frames.iter().filter(|f| {
        f.ids == Ids::Can11 && ((0x600..=0x6FF).contains(&f.id) || (0x240..=0x24F).contains(&f.id))
    }) {
        by.entry(f.id).or_default().push(&f.data);
    }
    let mostly = |n: usize, of: usize| n * 10 >= of * 9;
    let reads = |id: u32, ext: bool| {
        by.get(&id).is_some_and(|d| {
            let (good, broken) = fit(d, ext);
            good > 0 && mostly(d.len() - broken.min(d.len()), d.len())
        })
    };
    let mut bmw = BTreeSet::new();
    if reads(BMW_TESTER, true) {
        bmw.insert(BMW_TESTER);
        let asked: BTreeSet<u8> = by[&BMW_TESTER]
            .iter()
            .filter_map(|d| d.first().copied())
            .collect();
        for a in asked {
            let rx = 0x600 | u32::from(a);
            let f1_first = by.get(&rx).is_some_and(|d| {
                mostly(
                    d.iter().filter(|f| f.first() == Some(&0xF1)).count(),
                    d.len(),
                )
            });
            if rx != BMW_TESTER && f1_first && reads(rx, true) {
                bmw.insert(rx);
            }
        }
    }
    let mut gm = BTreeSet::new();
    for tx in 0x240..=0x24F {
        let rx = tx + 0x400;
        if !bmw.contains(&rx) && reads(tx, false) && reads(rx, false) {
            gm.extend([tx, rx]);
        }
    }
    (bmw, gm)
}

/// Whether a message is a known service's request or answer (or a refusal).
fn known(m: &[u8]) -> bool {
    match m.first() {
        Some(0x7F) => m.len() >= 3,
        Some(&s) => {
            diag::service_name(if diag::is_request(s) {
                s
            } else {
                s.wrapping_sub(0x40)
            }) != "another service"
        }
        None => false,
    }
}

/// How one ID's frames read with an address byte in front of each (`ext`) or without: (whole messages of a known
/// service, frames that fit no message). Each address byte is its own sender.
fn fit(frames: &[&[u8]], ext: bool) -> (usize, usize) {
    let mut isotp: Reassembler<Option<u8>> = Reassembler::default();
    let (mut good, mut broken) = (0, 0);
    for b in frames {
        let (addr, body) = match (ext, b.split_first()) {
            (false, _) => (None, *b),
            (true, Some((a, rest))) => (Some(*a), rest),
            (true, None) => {
                broken += 1;
                continue;
            }
        };
        match isotp.feed(addr, body) {
            Fed::Message(m) if known(&m) => good += 1,
            Fed::Broken(_) => broken += 1,
            _ => {}
        }
    }
    (good, broken)
}

/// The ID answers to `id` come from, or the requests it answers go to (request + 8; on 29-bit the two address bytes
/// swapped).
fn partner(ids: Ids, id: u32) -> Option<u32> {
    match ids {
        Ids::Can11 if (0x700..=0x7F7).contains(&id) && id & 0x8 == 0 => Some(id + 8),
        Ids::Can11 if (0x708..=0x7FF).contains(&id) => Some(id - 8),
        Ids::Can29 => Some((id & 0xFFFF_0000) | ((id & 0xFF) << 8) | ((id >> 8) & 0xFF)),
        _ => None,
    }
}

/// Which IDs use extended addressing (an address byte in front of each frame: Toyota's modules behind 750, BMW): those
/// whose frames make more whole messages that way (or as many, with fewer frames left over). An ID that reads as well
/// both ways (its sub-addresses 10-3F look like the start or the next part of a message) follows the ID it talks to;
/// failing that, it reads without.
fn extended(frames: &[BusFrame]) -> BTreeSet<(u8, u32)> {
    let mut by: BTreeMap<(u8, u32), Vec<&[u8]>> = BTreeMap::new();
    for f in frames {
        by.entry((f.ids as u8, f.id)).or_default().push(&f.data);
    }
    let mut ext = BTreeSet::new();
    let mut tied = Vec::new();
    for (k, data) in &by {
        let ((ng, nb), (xg, xb)) = (fit(data, false), fit(data, true));
        match (xg, nb).cmp(&(ng, xb)) {
            std::cmp::Ordering::Greater => {
                ext.insert(*k);
            }
            std::cmp::Ordering::Equal => tied.push(*k),
            std::cmp::Ordering::Less => {}
        }
    }
    for (kind, id) in tied {
        let ids = if kind == Ids::Can29 as u8 {
            Ids::Can29
        } else {
            Ids::Can11
        };
        if partner(ids, id).is_some_and(|p| ext.contains(&(kind, p))) {
            ext.insert((kind, id));
        }
    }
    ext
}

/// A diagnostic message from the log.
struct Msg {
    at: f64,
    ids: Ids,
    id: u32,
    ext: Option<u8>,
    data: Vec<u8>,
}

/// Whether a message from `rx` (with address byte `rx_ext`) comes from where answers to a request to `tx` (with
/// `ext`) usually come from: the usual answer ID, and behind a gateway the module's own address byte first, as the
/// request had it (Toyota's 750), or for BMW's 6F1 the tester's F1 first on 600 + the module's address.
fn related(ids: Ids, (tx, ext): (u32, Option<u8>), (rx, rx_ext): (u32, Option<u8>)) -> bool {
    if is_bmw(ids, tx, ext) {
        let from_module = rx_ext == Some(0xF1) && (0x600..=0x6FF).contains(&rx);
        return from_module && (ext == Some(0xDF) || usual_rx(ids, tx, ext) == Some(rx));
    }
    if ext != rx_ext {
        return false;
    }
    if usual_rx(ids, tx, ext) == Some(rx) {
        return true;
    }
    match ids {
        Ids::Can11 => tx == 0x7DF && (0x7E8..=0x7EF).contains(&rx),
        // a functional 29-bit request (18DB33F1): answers to its sender (F1) on 18DA F1 xx
        Ids::Can29 => {
            is_functional(ids, tx, ext)
                && (rx >> 16) & 0xFF == 0xDA
                && (rx >> 8) & 0xFF == tx & 0xFF
        }
        Ids::Kline => false,
    }
}

/// Reads a bus log down to its diagnostic exchanges.
pub fn read(text: &str) -> Result<Read, String> {
    let mut frames = Vec::new();
    let (mut lines, mut last) = (0usize, 0.0);
    for l in text.lines() {
        lines += 1;
        if let Some(f) = parse_line(l, last) {
            last = f.at;
            frames.push(f);
        }
    }
    if frames.is_empty() {
        return Err(
            "no CAN frames in this file: is it a bus log (icarus bus watch --raw, candump)?".into(),
        );
    }
    let total = frames.len();
    let (bmw, gm) = make_ids(&frames);
    let made = |f: &BusFrame| f.ids == Ids::Can11 && (bmw.contains(&f.id) || gm.contains(&f.id));
    frames.retain(|f| diagnostic(f.ids, f.id) || made(f));
    // extended addressing, per ID: frames that make sense only with an address byte in front (always on BMW's IDs,
    // never on GM's)
    let mut ext_ids = extended(&frames);
    let can11 = |id: &u32| (Ids::Can11 as u8, *id);
    ext_ids.extend(bmw.iter().map(can11));
    for k in gm.iter().map(can11) {
        ext_ids.remove(&k);
    }
    let mut isotp: Reassembler<(u32, Option<u8>)> = Reassembler::default();
    let mut msgs = Vec::new();
    let mut broken = 0;
    for f in &frames {
        let (addr, body) = if ext_ids.contains(&(f.ids as u8, f.id)) {
            match f.data.split_first() {
                Some((a, rest)) => (Some(*a), rest),
                None => continue,
            }
        } else {
            (None, &f.data[..])
        };
        match isotp.feed((f.id, addr), body) {
            Fed::Message(data) if !data.is_empty() => msgs.push(Msg {
                at: f.at,
                ids: f.ids,
                id: f.id,
                ext: addr,
                data,
            }),
            Fed::Broken(_) => broken += 1,
            _ => {}
        }
    }
    // requests and their answers
    let mut ex: Vec<Exchange> = Vec::new();
    // per exchange: (service, still waiting)
    let mut open: Vec<(usize, u8, bool)> = Vec::new();
    let mut stray = 0;
    for m in msgs {
        let sid = m.data[0];
        if diag::is_request(sid) {
            // a new request from the same sender ends its last one's wait
            for o in open.iter_mut() {
                let e = &ex[o.0];
                if e.tx == Some(m.id) && e.ext == m.ext && !is_functional(e.ids, m.id, m.ext) {
                    o.2 = false;
                }
            }
            open.retain(|o| o.2 && m.at - ex[o.0].at <= WAIT);
            if open.len() >= MAX_OPEN {
                open.drain(..=open.len() - MAX_OPEN);
            }
            open.push((ex.len(), sid, true));
            ex.push(Exchange {
                at: m.at,
                clock: None,
                ids: m.ids,
                tx: Some(m.id),
                ext: m.ext,
                rx: None,
                request: m.data,
                replies: vec![],
                problem: None,
                masked: false,
            });
            continue;
        }
        let Some(asked) = diag::answers(&m.data) else {
            stray += 1;
            continue;
        };
        let fits = |o: &&(usize, u8, bool), strict: bool| {
            let e = &ex[o.0];
            let echo = match asked {
                0x22 => e.request.get(1..3) == m.data.get(1..3) || m.data[0] == 0x7F,
                0x21 | 0x1A => e.request.get(1) == m.data.get(1) || m.data[0] == 0x7F,
                _ => true,
            };
            o.2 && o.1 == asked
                && e.ids == m.ids
                && m.at - e.at <= WAIT
                && echo
                && (!strict || related(e.ids, (e.tx.unwrap_or(0), e.ext), (m.id, m.ext)))
        };
        let pick = open
            .iter()
            .rev()
            .find(|o| fits(o, true))
            .or_else(|| open.iter().rev().find(|o| fits(o, false)))
            .map(|o| o.0);
        let Some(i) = pick else {
            stray += 1;
            continue;
        };
        let pending = m.data.len() >= 3 && m.data[0] == 0x7F && m.data[2] == 0x78;
        if !pending {
            let functional = ex[i]
                .tx
                .is_some_and(|t| is_functional(ex[i].ids, t, ex[i].ext));
            ex[i].replies.push(Reply {
                from: Some(m.id),
                data: m.data,
            });
            if !functional {
                if let Some(o) = open.iter_mut().find(|o| o.0 == i) {
                    o.2 = false;
                }
            }
        }
        open.retain(|o| o.2 && m.at - ex[o.0].at <= WAIT);
    }
    for e in ex.iter_mut().filter(|e| e.replies.is_empty()) {
        e.problem = Some("no answer on the bus".into());
    }
    let mut notes = vec![format!(
        "{lines} line(s), {total} CAN frame(s), {} on diagnostic IDs; {} request(s) found",
        frames.len(),
        ex.len()
    )];
    if broken > 0 {
        notes.push(format!(
            "{broken} diagnostic frame(s) that did not fit a message (a part lost or repeated)"
        ));
    }
    if stray > 0 {
        notes.push(format!("{stray} answer(s) without a request in the log"));
    }
    for (make, set) in [("BMW", &bmw), ("GM", &gm)] {
        if !set.is_empty() {
            let list: Vec<String> = set.iter().map(|id| format!("{id:03X}")).collect();
            notes.push(format!(
                "read as {make}'s diagnostic IDs too: {}",
                list.join(", ")
            ));
        }
    }
    Ok(Read {
        from: "bus capture",
        format: "bus log (icarus bus watch --raw or candump)".into(),
        exchanges: ex,
        notes,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A bus log as `icarus bus watch --raw` prints it, with the watch's own words in between (made-up identifiers).
    const LOG: &str = "watching for 60 s (Ctrl+C stops); nothing is sent onto the bus...
   0.010       2C4  03 1A F8 00
   0.100       7E0  03 22 12 34 00 00 00 00
   0.100  7E0 request            read data by identifier: 1234
   0.110       7E8  10 0B 62 12 34 01 02 03
   0.111       7E0  30 00 00 00 00 00 00 00
   0.120       7E8  21 04 05 06 07 08 AA AA
   0.200       7E0  03 22 56 78 00 00 00 00
   0.210       7E8  03 7F 22 78 00 00 00 00
   0.260       7E8  03 7F 22 31 00 00 00 00
   0.300       7DF  02 09 02 00 00 00 00 00
   0.310       7E8  10 14 49 02 01 4A 54 45
   0.311       7DF  30 00 00 00 00 00 00 00
   0.312       7E8  21 53 54 30 30 30 30 30
   0.313       7E8  22 54 30 30 30 30 30 31
   0.320       7EA  03 7F 09 11 00 00 00 00
   0.400       750  40 02 21 01 00 00 00 00
   0.410       758  F1 03 61 01 0A 00 00 00
   0.500       7E0  04 2E 12 34 01 00 00 00
   0.510       7E8  03 6E 12 34 00 00 00 00
   0.600  18DA10F1  03 22 AB CD 00 00 00 00
   0.610  18DAF110  04 62 AB CD 09 00 00 00
   0.700       7B0  02 21 07 00 00 00 00 00
   0.800       7E0  02 10 03
   0.810       7E8  02 50 03
  -- mark: lights
   1.000       2C4  03 1A F9 00

10 frames, 3 IDs (stopped). Bytes that changed in the last second are marked *:
       7E0     10.0 Hz        5  02 21 01
";

    #[test]
    fn requests_and_answers_are_paired_on_the_bus() {
        // a raw log's first line names the adapter (icarus bus watch --raw): not a frame
        assert!(parse_line("# adapter: ELM327 v1.4b (STN2256 v5.10.3)", 0.0).is_none());
        let r = read(LOG).unwrap();
        let e = &r.exchanges;
        let got: Vec<(String, String)> = e
            .iter()
            .map(|x| (x.module(), super::super::hex(&x.request)))
            .collect();
        assert_eq!(
            got,
            vec![
                ("7E0".into(), "22 12 34".into()),
                ("7E0".into(), "22 56 78".into()),
                ("7DF (every module)".into(), "09 02".into()),
                ("750.40".into(), "21 01".into()),
                ("7E0".into(), "2E 12 34 01".into()),
                ("18DA10F1".into(), "22 AB CD".into()),
                ("7B0".into(), "21 07".into()),
                ("7E0".into(), "10 03".into()),
            ],
            "{:?}",
            r.notes
        );
        assert_eq!(
            e[0].replies[0].data,
            vec![0x62, 0x12, 0x34, 1, 2, 3, 4, 5, 6, 7, 8]
        );
        assert_eq!(e[0].replies[0].from, Some(0x7E8));
        assert_eq!(
            e[1].replies,
            vec![Reply {
                from: Some(0x7E8),
                data: vec![0x7F, 0x22, 0x31]
            }],
            "pending skipped"
        );
        assert_eq!(e[2].replies.len(), 2, "two modules answered 7DF");
        assert_eq!(e[2].replies[0].data.len(), 20);
        assert_eq!(
            e[3].replies[0],
            Reply {
                from: Some(0x758),
                data: vec![0x61, 0x01, 0x0A]
            }
        );
        assert_eq!(e[5].replies[0].from, Some(0x18DAF110));
        assert_eq!(e[6].problem.as_deref(), Some("no answer on the bus"));
        assert!(r.notes[0].contains("CAN frame(s)"));
    }

    #[test]
    fn two_modules_behind_a_gateway_are_paired_by_their_address_bytes() {
        // behind 750 at sub-addresses 10 and 29 (which read as a message's start and its next part), each answering
        // from 758 with its own address byte first in two frames; the second asked before the first answered
        let log = "\
   0.400       750  10 02 21 01 00 00 00 00
   0.401       750  29 02 21 01 00 00 00 00
   0.410       758  10 10 0B 61 01 01 02 03
   0.411       750  10 30 00 00 00 00 00 00
   0.412       758  10 21 04 05 06 07 08 09
   0.420       758  29 10 0B 61 01 0A 0B 0C
   0.421       750  29 30 00 00 00 00 00 00
   0.422       758  29 21 0D 0E 0F 10 11 12
";
        let r = read(log).unwrap();
        let e = &r.exchanges;
        assert_eq!(e.len(), 2, "{:?}", r.notes);
        assert_eq!(
            (e[0].module(), e[1].module()),
            ("750.10".into(), "750.29".into())
        );
        assert_eq!(
            e[0].replies,
            vec![Reply {
                from: Some(0x758),
                data: vec![0x61, 0x01, 1, 2, 3, 4, 5, 6, 7, 8, 9]
            }]
        );
        assert_eq!(
            e[1].replies,
            vec![Reply {
                from: Some(0x758),
                data: vec![0x61, 0x01, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12]
            }]
        );
        assert_eq!(r.notes.len(), 1, "nothing broken or stray: {:?}", r.notes);
    }

    #[test]
    fn bmw_and_gm_diagnostic_ids_are_paired() {
        // BMW: requests on 6F1 with the module's address first, answers on 600 + that address with F1 first (made-up
        // identifiers); 6A0 and 6B0 are other traffic, never asked on 6F1
        let bmw = "\
   1.000       6F1  12 03 22 40 01 00 00 00
   1.010       612  F1 05 62 40 01 0A 0B 00
   1.100       6F1  18 03 22 40 02 00 00 00
   1.110       618  F1 10 09 62 40 02 01 02
   1.111       6F1  18 30 00 00 00 00 00 00
   1.120       618  F1 21 03 04 05 06 00 00
   1.200       6F1  40 02 10 03 00 00 00 00
   1.210       640  F1 02 50 03 00 00 00 00
   1.300       6F1  DF 02 3E 80 00 00 00 00
   1.400       6A0  02 10 03 00 00 00 00 00
   1.500       6B0  F1 03 62 40 01 00 00 00
";
        let r = read(bmw).unwrap();
        let e = &r.exchanges;
        let got: Vec<(String, String)> = e
            .iter()
            .map(|x| (x.module(), super::super::hex(&x.request)))
            .collect();
        assert_eq!(
            got,
            vec![
                ("6F1.12".into(), "22 40 01".into()),
                ("6F1.18".into(), "22 40 02".into()),
                ("6F1.40".into(), "10 03".into()),
                ("6F1.DF (every module)".into(), "3E 80".into()),
            ],
            "{:?}",
            r.notes
        );
        let reply = |from: u32, data: &[u8]| Reply {
            from: Some(from),
            data: data.to_vec(),
        };
        assert_eq!(
            e[0].replies,
            vec![reply(0x612, &[0x62, 0x40, 0x01, 0x0A, 0x0B])]
        );
        assert_eq!(
            e[1].replies,
            vec![reply(0x618, &[0x62, 0x40, 0x02, 1, 2, 3, 4, 5, 6])]
        );
        assert_eq!(e[2].replies, vec![reply(0x640, &[0x50, 0x03])]);
        assert!(
            r.notes
                .iter()
                .any(|n| n == "read as BMW's diagnostic IDs too: 612, 618, 640, 6F1"),
            "{:?}",
            r.notes
        );
        // as a set: answered from the usual ID, nothing assumed; the answers carry the tester's address
        let o = super::super::Options {
            file_name: "bmw.log".into(),
            from: "bus capture".into(),
            ..Default::default()
        };
        let imp = super::super::build(e, &o).unwrap();
        assert!(imp.notes.is_empty(), "{:?}", imp.notes);
        let m = &imp.modules[0].module;
        assert_eq!(
            (m.label(), m.rx, m.tester),
            ("6F1.12".into(), Some(0x612), Some(0xF1))
        );
        // GM: requests on 24x, answers on 64x; 24A is never answered and 245/645 carry other traffic
        let gm = "\
   0.100       241  03 22 00 0C 00 00 00 00
   0.110       641  05 62 00 0C 1A F8 00 00
   0.200       242  03 22 19 40 00 00 00 00
   0.210       642  04 62 19 40 05 00 00 00
   0.300       24A  03 22 12 34 00 00 00 00
   0.400       245  11 22 33 44 55 66 77 88
   0.410       645  99 88 77 66 55 44 33 22
";
        let r = read(gm).unwrap();
        let e = &r.exchanges;
        assert_eq!(e.len(), 2, "{:?}", r.notes);
        assert_eq!((e[0].module(), e[1].module()), ("241".into(), "242".into()));
        assert_eq!(
            e[0].replies,
            vec![reply(0x641, &[0x62, 0x00, 0x0C, 0x1A, 0xF8])]
        );
        assert_eq!(e[1].replies, vec![reply(0x642, &[0x62, 0x19, 0x40, 0x05])]);
        let imp = super::super::build(e, &o).unwrap();
        assert!(imp.notes.is_empty(), "{:?}", imp.notes);
    }

    #[test]
    fn candump_lines_are_read_too() {
        let f = parse_line("(1763560000.250000) can0 7E0#0322123400000000", 0.0).unwrap();
        assert_eq!((f.id, f.at, f.data.len()), (0x7E0, 1_763_560_000.25, 8));
        let f = parse_line("  can0  18DAF110   [8]  04 62 AB CD 09 00 00 00", 3.0).unwrap();
        assert_eq!((f.ids, f.id, f.at), (Ids::Can29, 0x18DAF110, 3.0));
        for junk in [
            "",
            "   0.100  7E0 request            read data",
            "watching for 60 s",
            "7E0#0",
            "  7E0  10.0 Hz  5  02",
            "FFFFFFFFF 00",
            "1.5 1.6",
        ] {
            assert!(parse_line(junk, 0.0).is_none(), "{junk}");
        }
        assert!(read("no frames here\n").is_err());
    }

    #[test]
    fn requests_without_times_stop_waiting_by_count() {
        // candump's default format: no times, so five seconds never pass
        let line = |id: &str, data: &str| format!("  can0  {id}   [8]  {data}\n");
        let mut log = line("7DF", "03 22 12 34 00 00 00 00");
        for _ in 0..MAX_OPEN {
            log += &line("7DF", "02 21 01 00 00 00 00 00");
        }
        log += &line("7E8", "04 62 12 34 01 00 00 00");
        log += &line("7DF", "03 22 56 78 00 00 00 00");
        log += &line("7E8", "04 62 56 78 02 00 00 00");
        let r = read(&log).unwrap();
        assert!(r.exchanges[0].replies.is_empty(), "too old to be answered");
        assert_eq!(
            r.exchanges.last().unwrap().replies[0].data,
            vec![0x62, 0x56, 0x78, 0x02]
        );
        assert!(
            r.notes
                .iter()
                .any(|n| n.contains("1 answer(s) without a request")),
            "{:?}",
            r.notes
        );
        // a long log of unanswered requests to every module is read in a moment, not in quadratic time
        let many = line("7DF", "02 3E 80 00 00 00 00 00").repeat(50_000);
        let t = std::time::Instant::now();
        assert_eq!(read(&many).unwrap().exchanges.len(), 50_000);
        assert!(t.elapsed().as_secs() < 20, "{:?}", t.elapsed());
    }
}
