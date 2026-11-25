//! Naming a module's unknown values by matching them against the standard values (ICA-7). While Icarus reads OBD-II's
//! standard values and a module's identifiers in turn, every byte and every pair of bytes (high byte first) of the answers
//! that no named value covers is compared with every standard value: one that rises and falls with it (Pearson's r of at
//! least `MIN_R` over at least `MIN_READS` reads, both taking at least three different values) is suggested as that value,
//! with the scale and offset that turn the raw number into the standard value's units. A capture's placeholders (a value
//! whose note says "not named") count as unknown.
//!
//! The reads are not simultaneous: each answer is compared with the standard value as it stood at that moment, drawn
//! between the reads before and after it. A suggestion is a guess to check on the vehicle (and in another tool), never
//! written into a definition by itself.
//!
//! Values that change with something done (a door, the brake pedal, the lights) are found by marks (ICA-89): the person
//! says what they are doing as they do it, and a bit (or a whole byte) that changed near every mark of one name
//! (`MARK_BEFORE` seconds before it to `MARK_AFTER` after) and at no time no mark explains is suggested as that.
//!
//! Every module request goes through module data's gates and pacer (`moddata::fetch_request`): reading services only, no
//! airbag module, no session, at least 50 ms between requests.

use crate::j1979;
use crate::link::Link;
use crate::moddata::{self, Gates, Outcome, Pacer, Scope};
use icarus_core::moddata::{ModuleDef, RequestDef};
use std::time::Instant;

/// The standard values compared: the ones that move while a vehicle runs (load, coolant, fuel trims, manifold pressure,
/// engine speed, road speed, timing, intake air, air flow, throttle, fuel level, barometric pressure, voltage, ambient air,
/// oil temperature). Only those the vehicle gives are read.
pub const PIDS: &[u8] = &[
    0x04, 0x05, 0x06, 0x07, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x2F, 0x33, 0x42, 0x46, 0x5C,
];
/// The fewest reads compared for a suggestion.
pub const MIN_READS: usize = 20;
/// The least correlation (either way) for a suggestion.
pub const MIN_R: f64 = 0.98;

/// One standard value's reads: seconds from the start, and the value.
#[derive(Clone, Debug, Default)]
pub struct Standard {
    pub pid: u8,
    pub name: String,
    pub unit: String,
    pub reads: Vec<(f64, f64)>,
}

/// One request's answers (seconds from the start, the data after the echo), and the bytes its named values cover.
#[derive(Clone, Debug, Default)]
pub struct Unknown {
    /// The module's label ("700", "750.2A").
    pub module: String,
    /// The request ("22 1F06").
    pub request: String,
    /// (first byte, byte count) of each named value.
    pub named: Vec<(usize, usize)>,
    pub reads: Vec<(f64, Vec<u8>)>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Suggestion {
    pub module: String,
    pub request: String,
    /// The field: its first byte in the data after the echo, and 8 or 16 bits (high byte first).
    pub byte: usize,
    pub bits: u8,
    pub pid: u8,
    pub name: String,
    pub unit: String,
    pub r: f64,
    /// standard value = raw x mul + add
    pub mul: f64,
    pub add: f64,
    pub reads: usize,
    /// Both only rose (or only fell) while compared: a counter or a slow drift fits that as well.
    pub one_way: bool,
}

impl Suggestion {
    pub fn text(&self) -> String {
        let field = if self.bits == 8 {
            format!("byte {}", self.byte)
        } else {
            format!("bytes {}-{} (16 bits)", self.byte, self.byte + 1)
        };
        let sign = if self.add < 0.0 { '-' } else { '+' };
        // "Engine speed" mid-sentence, an acronym ("MAF ...") as it is
        let mut c = self.name.chars();
        let name = match (c.next(), c.next()) {
            (Some(a), Some(b)) if a.is_uppercase() && b.is_lowercase() => {
                a.to_lowercase().chain(self.name.chars().skip(1)).collect()
            }
            _ => self.name.clone(),
        };
        format!(
            "{} {}, {field}: {} {} (r {:.3} over {} reads): {} = raw x {} {sign} {} {}{}",
            self.module,
            self.request,
            if self.r < 0.0 { "falls as" } else { "follows" },
            name,
            self.r,
            self.reads,
            name,
            round(self.mul),
            round(self.add.abs()),
            self.unit,
            if self.one_way {
                "; both only went one way, so a counter or a slow drift fits too: compare again while it rises and falls"
            } else {
                ""
            }
        )
    }
}

/// Four significant digits, without trailing zeros.
fn round(x: f64) -> String {
    if x == 0.0 {
        return "0".into();
    }
    let digits = (3 - x.abs().log10().floor() as i32).max(0) as usize;
    let s = format!("{x:.digits$}");
    if s.contains('.') {
        s.trim_end_matches('0').trim_end_matches('.').to_string()
    } else {
        s
    }
}

/// The bytes a request's named values cover: (first byte, count). A capture's placeholder (its note says "not named") names
/// nothing.
pub fn named_bytes(r: &RequestDef) -> Vec<(usize, usize)> {
    r.values
        .iter()
        .filter(|v| !v.note.contains("not named"))
        .map(|v| {
            let bits = v.bit as usize + v.bits as usize;
            (v.byte, bits.div_ceil(8).max(1))
        })
        .collect()
}

/// The value as it stood at `t`, drawn between the reads before and after it; None outside the reads.
fn at(reads: &[(f64, f64)], t: f64) -> Option<f64> {
    let i = reads.partition_point(|(rt, _)| *rt < t);
    if i < reads.len() && reads[i].0 == t {
        return Some(reads[i].1);
    }
    if i == 0 || i == reads.len() {
        return None;
    }
    let ((t0, v0), (t1, v1)) = (reads[i - 1], reads[i]);
    Some(v0 + (v1 - v0) * (t - t0) / (t1 - t0))
}

/// At least three different values.
fn moves(values: impl Iterator<Item = f64>) -> bool {
    let mut seen: Vec<f64> = Vec::new();
    for v in values {
        if !seen.contains(&v) {
            seen.push(v);
            if seen.len() >= 3 {
                return true;
            }
        }
    }
    false
}

fn one_way(v: impl Iterator<Item = f64> + Clone) -> bool {
    let pairs = || v.clone().zip(v.clone().skip(1));
    pairs().all(|(a, b)| b >= a) || pairs().all(|(a, b)| b <= a)
}

/// Pearson's r over (x, y) pairs; None when either does not vary.
fn pearson(p: &[(f64, f64)]) -> Option<f64> {
    let n = p.len() as f64;
    let (mx, my) = (
        p.iter().map(|q| q.0).sum::<f64>() / n,
        p.iter().map(|q| q.1).sum::<f64>() / n,
    );
    let (mut sxy, mut sxx, mut syy) = (0.0, 0.0, 0.0);
    for (x, y) in p {
        sxy += (x - mx) * (y - my);
        sxx += (x - mx) * (x - mx);
        syy += (y - my) * (y - my);
    }
    (sxx > 0.0 && syy > 0.0).then(|| sxy / (sxx * syy).sqrt())
}

/// y = mul * x + add, least squares.
fn fit(p: &[(f64, f64)]) -> (f64, f64) {
    let n = p.len() as f64;
    let (mx, my) = (
        p.iter().map(|q| q.0).sum::<f64>() / n,
        p.iter().map(|q| q.1).sum::<f64>() / n,
    );
    let sxy: f64 = p.iter().map(|(x, y)| (x - mx) * (y - my)).sum();
    let sxx: f64 = p.iter().map(|(x, _)| (x - mx) * (x - mx)).sum();
    let mul = sxy / sxx;
    (mul, my - mul * mx)
}

/// Every suggestion, the best first within each request; a field is suggested once (as the value it fits best), and a
/// field overlapping a better one is left out (a byte next to a constant one fits as well alone: the narrower is kept).
pub fn suggest(standard: &[Standard], unknown: &[Unknown]) -> Vec<Suggestion> {
    let mut out = Vec::new();
    for u in unknown {
        let len = u.reads.iter().map(|(_, d)| d.len()).min().unwrap_or(0);
        let free = |b: usize| b < len && !u.named.iter().any(|(s, n)| (*s..s + n).contains(&b));
        let mut cands = Vec::new();
        for byte in 0..len {
            for bits in [8u8, 16] {
                if !free(byte) || (bits == 16 && !free(byte + 1)) {
                    continue;
                }
                let raw = |d: &[u8]| {
                    if bits == 8 {
                        d[byte] as f64
                    } else {
                        u16::from_be_bytes([d[byte], d[byte + 1]]) as f64
                    }
                };
                let xs: Vec<(f64, f64)> = u.reads.iter().map(|(t, d)| (*t, raw(d))).collect();
                if !moves(xs.iter().map(|x| x.1)) {
                    continue;
                }
                for s in standard {
                    if !moves(s.reads.iter().map(|x| x.1)) {
                        continue;
                    }
                    let pairs: Vec<(f64, f64)> = xs
                        .iter()
                        .filter_map(|(t, x)| at(&s.reads, *t).map(|y| (*x, y)))
                        .collect();
                    if pairs.len() < MIN_READS {
                        continue;
                    }
                    let Some(r) = pearson(&pairs).filter(|r| r.abs() >= MIN_R) else {
                        continue;
                    };
                    let (mul, add) = fit(&pairs);
                    cands.push(Suggestion {
                        module: u.module.clone(),
                        request: u.request.clone(),
                        byte,
                        bits,
                        pid: s.pid,
                        name: s.name.clone(),
                        unit: s.unit.clone(),
                        r,
                        mul,
                        add,
                        reads: pairs.len(),
                        one_way: one_way(pairs.iter().map(|p| p.0))
                            && one_way(pairs.iter().map(|p| p.1)),
                    });
                }
            }
        }
        // equal fits (to 1e-9) keep the narrower field
        let key = |s: &Suggestion| ((s.r.abs() * 1e9).round() as i64, std::cmp::Reverse(s.bits));
        cands.sort_by_key(|s| std::cmp::Reverse(key(s)));
        let mut taken: Vec<(usize, usize)> = Vec::new();
        for c in cands {
            let (a, b) = (c.byte, c.byte + c.bits as usize / 8);
            if taken.iter().any(|(x, y)| a < *y && *x < b) {
                continue;
            }
            taken.push((a, b));
            out.push(c);
        }
    }
    out
}

/// A mark's window: a change counts from this long before the mark (said a little late)...
pub const MARK_BEFORE: f64 = 2.0;
/// ...to this long after it (said first, then done).
pub const MARK_AFTER: f64 = 5.0;

/// (name, how many marks, the value before and after the change nearest its first mark), for each name every one of whose
/// marks had a change near it.
pub type MarkHits = Vec<(String, usize, u8, u8)>;

/// A bit (or a whole byte) of an unnamed byte that changed near every mark of one name or more, and at no time no mark
/// explains.
#[derive(Clone, Debug, PartialEq)]
pub struct MarkSuggestion {
    pub module: String,
    pub request: String,
    pub byte: usize,
    /// None: the whole byte (several of its bits changed, differently from mark to mark).
    pub bit: Option<u8>,
    /// Each name every one of whose marks had a change near it: (name, how many marks, the value before and after the
    /// change near its first mark).
    pub marks: MarkHits,
}

impl MarkSuggestion {
    pub fn text(&self) -> String {
        let field = match self.bit {
            Some(b) => format!("byte {} bit {b}", self.byte),
            None => format!("byte {}", self.byte),
        };
        let what: Vec<String> = self
            .marks
            .iter()
            .map(|(l, n, a, b)| match self.bit {
                Some(_) => format!("\"{l}\" ({n} of {n}, {a} to {b})"),
                None => format!("\"{l}\" ({n} of {n}, {a:02X} to {b:02X})"),
            })
            .collect();
        let once = self.marks.iter().all(|m| m.1 == 1);
        format!(
            "{} {}, {field}: changed with {} and at no other time{}",
            self.module,
            self.request,
            what.join(", "),
            if once {
                "; marked once: do it and mark it again to be sure"
            } else {
                ""
            }
        )
    }
}

/// Every bit or byte that changed with marks (`marks`: seconds from the start, the name said). A byte of which one bit fits
/// is suggested as that bit (a switch); one of which several bits fit, or none alone, as the whole byte when it fits. The
/// bytes `matched` already gives to a standard value are left out: a slow value can cross a bit's edge near a mark.
pub fn suggest_marks(
    unknown: &[Unknown],
    marks: &[(f64, String)],
    matched: &[Suggestion],
) -> Vec<MarkSuggestion> {
    let mut out = Vec::new();
    let mut labels: Vec<&str> = Vec::new();
    for (_, l) in marks {
        if !labels.contains(&l.as_str()) {
            labels.push(l);
        }
    }
    let near = |t: f64, m: f64| t >= m - MARK_BEFORE && t <= m + MARK_AFTER;
    for u in unknown {
        let len = u.reads.iter().map(|(_, d)| d.len()).min().unwrap_or(0);
        let free = |b: usize| {
            b < len
                && !u.named.iter().any(|(s, n)| (*s..s + n).contains(&b))
                && !matched.iter().any(|m| {
                    m.module == u.module
                        && m.request == u.request
                        && (m.byte..m.byte + m.bits as usize / 8).contains(&b)
                })
        };
        let check = |val: &dyn Fn(&[u8]) -> u8| -> Option<MarkHits> {
            // (when, before, after) of each change between two reads in a row
            let changes: Vec<(f64, u8, u8)> = u
                .reads
                .windows(2)
                .filter_map(|w| {
                    let (a, b) = (val(&w[0].1), val(&w[1].1));
                    (a != b).then_some((w[1].0, a, b))
                })
                .collect();
            // a switch changes once a mark (twice, if it bounces); a value that keeps changing is not one, however many
            // marks there are around it
            if changes.is_empty()
                || changes.len() > 2 * marks.len()
                || !changes
                    .iter()
                    .all(|c| marks.iter().any(|(m, _)| near(c.0, *m)))
            {
                return None;
            }
            let mut got = Vec::new();
            for l in &labels {
                let ms: Vec<f64> = marks
                    .iter()
                    .filter(|(_, x)| x == l)
                    .map(|(t, _)| *t)
                    .collect();
                // the change nearest each mark (marks close together share their windows)
                let hits: Vec<&(f64, u8, u8)> = ms
                    .iter()
                    .filter_map(|m| {
                        changes
                            .iter()
                            .filter(|c| near(c.0, *m))
                            .min_by(|a, b| (a.0 - m).abs().total_cmp(&(b.0 - m).abs()))
                    })
                    .collect();
                if hits.len() == ms.len() {
                    got.push((l.to_string(), ms.len(), hits[0].1, hits[0].2));
                }
            }
            (!got.is_empty()).then_some(got)
        };
        for byte in (0..len).filter(|b| free(*b)) {
            let bits: Vec<(u8, MarkHits)> = (0..8u8)
                .filter_map(|k| check(&|d: &[u8]| (d[byte] >> k) & 1).map(|g| (k, g)))
                .collect();
            let at = |bit, marks| MarkSuggestion {
                module: u.module.clone(),
                request: u.request.clone(),
                byte,
                bit,
                marks,
            };
            // one bit: a switch; several, or none alone: a value of several bits (a gear, a position), whole when it fits
            let whole = if bits.len() == 1 {
                None
            } else {
                check(&|d: &[u8]| d[byte])
            };
            match whole {
                Some(g) => out.push(at(None, g)),
                None => out.extend(bits.into_iter().map(|(k, g)| at(Some(k), g))),
            }
        }
    }
    out
}

/// What `sample` read, the marks given while it read, and the requests it could not use (with why).
#[derive(Debug, Default)]
pub struct Sampled {
    pub standard: Vec<Standard>,
    pub unknown: Vec<Unknown>,
    /// Seconds from the start, and the name said.
    pub marks: Vec<(f64, String)>,
    pub left_out: Vec<(String, String)>,
    pub passes: usize,
}

/// Reads the standard values (`pids`, those the vehicle gives) and every request of `modules`, in turn, until `done` says
/// to stop (it is given the seconds since `start`), taking the marks `marks` gives at the start of each pass (when each
/// was said, and its name). A request whose first answer has no byte left unnamed is not asked again; one the gates or the
/// vehicle refuse is listed once in `left_out`.
#[allow(clippy::too_many_arguments)]
pub fn sample(
    link: &mut dyn Link,
    pacer: &mut Pacer,
    gates: &Gates,
    modules: &[ModuleDef],
    scope: &Scope,
    pids: &[u8],
    now: &dyn Fn() -> Instant,
    done: &mut dyn FnMut(f64) -> bool,
    marks: &mut dyn FnMut() -> Vec<(Instant, String)>,
) -> Sampled {
    let start = now();
    let secs = |t: Instant| t.duration_since(start).as_secs_f64();
    let mut out = Sampled::default();
    // the vehicle's own list of what it gives, when it says
    let given: Vec<u8> = match j1979::supported(link) {
        Ok(per) => {
            let all: Vec<u8> = per.into_iter().flat_map(|(_, _, p)| p).collect();
            pids.iter().copied().filter(|p| all.contains(p)).collect()
        }
        Err(_) => pids.to_vec(),
    };
    for &p in &given {
        if let Some(d) = j1979::pid(p) {
            out.standard.push(Standard {
                pid: p,
                name: d.name.into(),
                unit: d.unit.into(),
                reads: Vec::new(),
            });
        }
    }
    let mut asks: Vec<(&ModuleDef, &RequestDef, usize)> = Vec::new();
    for m in modules {
        for r in &m.requests {
            out.unknown.push(Unknown {
                module: m.module.label(),
                request: format!(
                    "{:02X} {:0w$X}",
                    r.service,
                    r.ident,
                    w = if r.service == 0x22 { 4 } else { 2 }
                ),
                named: named_bytes(r),
                reads: Vec::new(),
            });
            asks.push((m, r, out.unknown.len() - 1));
        }
    }
    let mut dropped = vec![false; asks.len()];
    loop {
        if done(secs(now())) {
            break;
        }
        out.passes += 1;
        for (at, name) in marks() {
            out.marks.push((secs(at.max(start)), name));
        }
        if let Ok(read) = j1979::read_many(link, &given) {
            let t = secs(now());
            for (p, readings) in read {
                // the engine module's (7E8) when several answer
                let r = readings
                    .iter()
                    .find(|r| r.from.eq_ignore_ascii_case("7E8"))
                    .or(readings.first());
                if let (Some(r), Some(s)) = (r, out.standard.iter_mut().find(|s| s.pid == p)) {
                    s.reads.push((t, r.value));
                }
            }
        }
        for (k, (m, r, u)) in asks.iter().enumerate() {
            if dropped[k] {
                continue;
            }
            match moddata::fetch_request(link, pacer, gates, m, r, scope) {
                Ok(d) => {
                    let t = secs(now());
                    let unk = &mut out.unknown[*u];
                    let covered =
                        |b: usize| unk.named.iter().any(|(s, n)| (*s..s + n).contains(&b));
                    if unk.reads.is_empty() && (0..d.len()).all(covered) {
                        dropped[k] = true;
                        continue;
                    }
                    unk.reads.push((t, d));
                }
                Err(Outcome::NotDue) => {}
                Err(o) => {
                    dropped[k] = true;
                    out.left_out.push((
                        format!("{} {}", out.unknown[*u].module, out.unknown[*u].request),
                        o.describe(),
                    ));
                }
            }
            if done(secs(now())) {
                break;
            }
        }
    }
    out.unknown.retain(|u| !u.reads.is_empty());
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn standard(name: &str, f: impl Fn(f64) -> f64) -> Standard {
        Standard {
            pid: 0x0C,
            name: name.into(),
            unit: "rpm".into(),
            reads: (0..60).map(|i| i as f64 * 0.5).map(|t| (t, f(t))).collect(),
        }
    }

    /// The made-up drive: engine speed rises and falls; an answer carries it as a 16-bit number (x 2), a byte of road
    /// speed's double, a constant byte and a counter. Engine speed and only that is suggested for the pair, the counter
    /// for nothing, and the named byte is left alone.
    #[test]
    fn a_pair_that_follows_engine_speed_is_suggested_as_engine_speed() {
        let rpm = |t: f64| 800.0 + 600.0 * (t * 0.3).sin();
        let std = [standard("engine speed", rpm)];
        let reads = (0..58)
            .map(|i| {
                let t = 0.25 + i as f64 * 0.5;
                let raw = (rpm(t) * 2.0) as u16;
                (t, vec![0x5A, (raw >> 8) as u8, raw as u8, i as u8, 7])
            })
            .collect();
        let u = Unknown {
            module: "700".into(),
            request: "22 1F06".into(),
            named: vec![(4, 1)],
            reads,
        };
        let got = suggest(&std, &[u]);
        assert_eq!(got.len(), 1, "{got:#?}");
        let s = &got[0];
        assert_eq!((s.byte, s.bits, s.name.as_str()), (1, 16, "engine speed"));
        assert!(
            s.r > 0.999 && (s.mul - 0.5).abs() < 0.01 && s.add.abs() < 5.0 && !s.one_way,
            "{s:?}"
        );
        assert!(
            s.text().starts_with(
                "700 22 1F06, bytes 1-2 (16 bits): follows engine speed (r 1.000 over 58 reads)"
            ),
            "{}",
            s.text()
        );
    }

    /// A byte next to a constant one fits as well alone, and is suggested alone; a value that only rose is marked.
    #[test]
    fn the_narrower_field_and_a_one_way_value() {
        let warm = |t: f64| 20.0 + t * 2.0;
        let std = [standard("coolant temperature", warm)];
        let reads = (0..40)
            .map(|i| (i as f64 * 0.5, vec![0, (warm(i as f64 * 0.5) + 40.0) as u8]))
            .collect();
        let u = Unknown {
            module: "7E0".into(),
            request: "21 A7".into(),
            named: vec![],
            reads,
        };
        let got = suggest(&std, &[u]);
        assert_eq!(got.len(), 1, "{got:#?}");
        assert_eq!((got[0].byte, got[0].bits), (1, 8));
        assert!(
            got[0].one_way && (got[0].add + 40.0).abs() < 0.5,
            "{:?}",
            got[0]
        );
    }

    thread_local! {
        static CLOCK: std::cell::Cell<std::time::Duration> = const { std::cell::Cell::new(std::time::Duration::ZERO) };
    }
    fn fake_now() -> Instant {
        static B: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
        *B.get_or_init(Instant::now) + CLOCK.with(|c| c.get())
    }
    fn fake_sleep(d: std::time::Duration) {
        CLOCK.with(|c| c.set(c.get() + d));
    }
    fn never() -> Option<String> {
        None
    }

    /// ICA-7's check: on the simulated vehicle, read in turn with the standard values for 40 (paced, simulated) seconds,
    /// module 700's unnamed 22 1F06 is suggested as engine speed (bytes 1-2, x 0.5), and nothing else is; the requests the
    /// vehicle refuses or the gates skip are listed.
    #[test]
    fn on_the_simulated_vehicle_22_1f06_is_suggested_as_engine_speed() {
        let set = moddata::load_set("sim", None, &crate::vehicles::builtin()).unwrap();
        let mut elm = crate::elm::Elm::connect(Box::new(crate::sim::SimCar::new())).unwrap();
        let mut pacer = Pacer::new(fake_now, fake_sleep, never);
        let scope = Scope {
            year: Some(2025),
            ..Default::default()
        };
        let engine = set.module(0x700).unwrap();
        let mut looks = 0;
        let got = sample(
            &mut elm,
            &mut pacer,
            &set.gates,
            std::slice::from_ref(engine),
            &scope,
            PIDS,
            &fake_now,
            &mut |t| {
                looks += 1;
                t > 40.0 || looks > 100_000
            },
            &mut Vec::new,
        );
        let s = suggest(&got.standard, &got.unknown);
        assert_eq!(s.len(), 1, "{s:#?}");
        let s = &s[0];
        assert_eq!(
            (s.module.as_str(), s.request.as_str(), s.byte, s.bits, s.pid),
            ("700", "22 1F06", 1, 16, 0x0C),
            "{s:?}"
        );
        assert!(
            s.r > 0.99 && (s.mul - 0.5).abs() < 0.02 && s.reads >= MIN_READS,
            "{s:?}"
        );
        let left: Vec<&str> = got.left_out.iter().map(|(w, _)| w.as_str()).collect();
        assert!(
            left.contains(&"700 22 1F03") && left.contains(&"700 22 1F05"),
            "{left:?}"
        );
    }

    /// ICA-89: a bit that flips near every mark of "door open" and "door closed", and at no other time, is suggested with
    /// both; a byte that also changes once with no mark near is not; a byte that changes only with the marks but through
    /// several bits is suggested whole.
    #[test]
    fn a_bit_that_flips_with_the_marks() {
        let marks: Vec<(f64, String)> = [
            (10.0, "door open"),
            (20.0, "door closed"),
            (30.0, "door open"),
            (40.0, "door closed"),
        ]
        .map(|(t, l)| (t, l.to_string()))
        .to_vec();
        let reads = (0..100)
            .map(|i| {
                let t = i as f64 * 0.5;
                let open = (11.0..21.0).contains(&t) || (31.5..41.0).contains(&t);
                let gear = if t < 11.0 {
                    1
                } else if t < 21.0 {
                    2
                } else if t < 31.0 {
                    3
                } else {
                    4
                };
                (
                    t,
                    vec![0x80 | open as u8, gear, if t > 25.0 { 9 } else { 8 }],
                )
            })
            .collect();
        let u = Unknown {
            module: "750".into(),
            request: "21 04".into(),
            named: vec![],
            reads,
        };
        let got = suggest_marks(&[u], &marks, &[]);
        assert_eq!(got.len(), 2, "{got:#?}");
        assert_eq!((got[0].byte, got[0].bit), (0, Some(0)));
        assert_eq!(
            got[0].marks,
            [
                ("door open".to_string(), 2, 0, 1),
                ("door closed".to_string(), 2, 1, 0)
            ]
        );
        assert_eq!(
            got[0].text(),
            "750 21 04, byte 0 bit 0: changed with \"door open\" (2 of 2, 0 to 1), \"door closed\" (2 of 2, 1 to 0) and at no other time"
        );
        // the gear byte changes at every mark through different bits: whole
        assert_eq!((got[1].byte, got[1].bit), (1, None));
        assert!(suggest_marks(&[], &marks, &[]).is_empty() && got.iter().all(|g| g.byte != 2));
    }

    /// ICA-89's check on the simulated vehicle: the brake pedal pressed twice and marked each time, while module 700 is read
    /// with the standard values, is suggested as byte 1 bit 0 of its unnamed 22 1F07, and nothing else is.
    #[test]
    fn on_the_simulated_vehicle_the_brake_is_found_by_marks() {
        let set = moddata::load_set("sim", None, &crate::vehicles::builtin()).unwrap();
        let mut elm = crate::elm::Elm::connect(Box::new(crate::sim::SimCar::new())).unwrap();
        let mut pacer = Pacer::new(fake_now, fake_sleep, never);
        let scope = Scope {
            year: Some(2025),
            ..Default::default()
        };
        let engine = set.module(0x700).unwrap();
        // the pedal goes down with the 30th and 90th answer of 22 1F07 and up with the 60th and 120th; one pass reads it once
        let mut pass = 0;
        let mut marks = || {
            pass += 1;
            match pass {
                30 | 90 => vec![(fake_now(), "brake pressed".to_string())],
                60 | 120 => vec![(fake_now(), "brake released".to_string())],
                _ => vec![],
            }
        };
        let mut looks = 0;
        let got = sample(
            &mut elm,
            &mut pacer,
            &set.gates,
            std::slice::from_ref(engine),
            &scope,
            PIDS,
            &fake_now,
            &mut |t| {
                looks += 1;
                t > 40.0 || looks > 100_000
            },
            &mut marks,
        );
        assert!(got.passes > 130, "{}", got.passes);
        assert_eq!(got.marks.len(), 4);
        let found = suggest_marks(
            &got.unknown,
            &got.marks,
            &suggest(&got.standard, &got.unknown),
        );
        assert_eq!(found.len(), 1, "{found:#?}");
        assert_eq!(
            (found[0].request.as_str(), found[0].byte, found[0].bit),
            ("22 1F07", 1, Some(0)),
            "{found:?}"
        );
        assert_eq!(
            found[0]
                .marks
                .iter()
                .map(|m| (m.0.as_str(), m.1, m.2, m.3))
                .collect::<Vec<_>>(),
            [("brake pressed", 2, 0, 1), ("brake released", 2, 1, 0)]
        );
    }

    #[test]
    fn rounding() {
        assert_eq!(
            [
                round(0.5),
                round(0.250_000_1),
                round(-40.0),
                round(12345.6),
                round(0.0)
            ],
            ["0.5", "0.25", "-40", "12346", "0"]
        );
    }
}
