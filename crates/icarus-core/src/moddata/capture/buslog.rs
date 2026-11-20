//! A bus log: the CAN frames another tool and the modules sent each other, recorded through a Y-splitter by
//! `icarus bus watch --raw` (one frame a line: the time in seconds, the ID in hex, the data bytes in hex; the watch's own
//! words between them are skipped) or by Linux's `candump` (`(1700000000.123456) can0 7E0#0322123400000000`, or its
//! default `can0  7E0   [8]  03 22 12 34 00 00 00 00`).
//!
//! Diagnostic frames (11-bit IDs 700-7FF, 29-bit IDs 18DAxxxx and 18DBxxxx) are put back together per sender with ISO-TP
//! (`crate::diag`, the same code `icarus bus watch` uses); flow control frames carry no message. An ID whose frames make
//! sense only with an address byte in front (extended addressing: Toyota's modules behind 750, BMW) is read that way,
//! told by how its frames read over the whole log. Each request is then paired with its answers: the answer's service is
//! the request's + 40 (or 7F naming it), it comes from the usual answer ID (request + 8; the two address bytes swapped on
//! 29-bit; 7E8-7EF for 7DF) or, failing that, from the module the request went to most recently with that service, within
//! five seconds (and among the last 64 requests still waiting). Every other frame on the bus is ignored.

use super::{is_functional, usual_rx, Exchange, Read, Reply};
use crate::diag::{self, Fed, Reassembler};
use crate::moddata::Ids;
use std::collections::BTreeMap;

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

/// Whether an ID carries diagnostic messages.
fn diagnostic(ids: Ids, id: u32) -> bool {
    match ids {
        Ids::Can11 => (0x700..=0x7FF).contains(&id),
        Ids::Can29 => matches!((id >> 16) & 0xFF, 0xDA | 0xDB),
        Ids::Kline => false,
    }
}

/// Whether a frame reads as ISO-TP (a single frame carrying a known service, or a part of a longer message).
fn reads(b: &[u8]) -> bool {
    match diag::frame(b) {
        Some(diag::Frame::Single(d)) => {
            let s = d[0];
            s == 0x7F
                || diag::service_name(if diag::is_request(s) {
                    s
                } else {
                    s.wrapping_sub(0x40)
                }) != "another service"
        }
        Some(_) => true,
        None => false,
    }
}

/// A diagnostic message from the log.
struct Msg {
    at: f64,
    ids: Ids,
    id: u32,
    ext: Option<u8>,
    data: Vec<u8>,
}

/// Whether `rx` is where answers to `tx` usually come from.
fn related(ids: Ids, tx: u32, rx: u32) -> bool {
    if usual_rx(ids, tx) == Some(rx) {
        return true;
    }
    match ids {
        Ids::Can11 => tx == 0x7DF && (0x7E8..=0x7EF).contains(&rx),
        // a functional 29-bit request (18DB33F1): answers to its sender (F1) on 18DA F1 xx
        Ids::Can29 => {
            is_functional(ids, tx) && (rx >> 16) & 0xFF == 0xDA && (rx >> 8) & 0xFF == tx & 0xFF
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
    frames.retain(|f| diagnostic(f.ids, f.id));
    // extended addressing, per ID: frames that make sense only with an address byte in front
    let mut votes: BTreeMap<(u8, u32), (usize, usize)> = BTreeMap::new();
    for f in &frames {
        let v = votes.entry((f.ids as u8, f.id)).or_default();
        v.0 += usize::from(reads(&f.data));
        v.1 += usize::from(f.data.len() > 1 && reads(&f.data[1..]));
    }
    let mut isotp: Reassembler<(u32, Option<u8>)> = Reassembler::default();
    let mut msgs = Vec::new();
    let mut broken = 0;
    for f in &frames {
        let (normal, ext) = votes[&(f.ids as u8, f.id)];
        let (addr, body) = if ext > normal {
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
                if e.tx == Some(m.id) && e.ext == m.ext && !is_functional(e.ids, m.id) {
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
                && (!strict || related(e.ids, e.tx.unwrap_or(0), m.id))
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
            let functional = ex[i].tx.is_some_and(|t| is_functional(ex[i].ids, t));
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
