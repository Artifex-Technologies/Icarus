//! A health check (ICA-214): one answer to "how does everything look?", each part beside this vehicle's own earlier
//! readings, with drift said in plain words ("bank 2 total trim (warm idle) moved from -2 to +4 since 2026-11-21: 6 points
//! leaner-correcting").
//!
//! What it reads, all through the read paths that exist (nothing cleared, nothing written to the vehicle):
//!
//! - codes: OBD-II's stored, pending and permanent codes (03, 07, 0A), and each of the profile's modules' own codes, failed
//!   ones only, as `uds::ModuleCodes` keeps them (a code marked only "not tested" is not a finding);
//! - the readiness monitors (01 PID 01): which have not run;
//! - misfire counts: Mode 06's per-cylinder counts (MIDs A1-AD) and, where the vehicle's module-data set has them, its
//!   misfire counts (`is_misfire_count`: not a misfire's engine speed, load or margin), only those values' requests sent
//!   (`vehicledata::read_values`);
//! - fuel trims: a short sample of short- and long-term trim per bank (01 PIDs 06-09) with coolant, engine speed, road
//!   speed and the fuel system's loop (01 PID 03), the total (short + long) kept for a warm idle and, when the sample
//!   catches a steady one (within 5 mph), a cruise, from closed-loop rows only (open loop: warm-up, enrichment, fuel cut);
//!   and the gap between the banks (bank 1 minus bank 2, compared signed).
//!
//! The history is kept per vehicle, in its folder's `Logs/health-history.jsonl` (the simulated vehicle's, or one with no
//! folder, in Icarus's state folder; none for a blank VIN, which every bench ECU would share): one JSON object a line, one line a health check, oldest first; a line that does not
//! read is skipped. Its fields (every one but `when` may be missing, read as empty):
//!
//! - `when`: "2026-11-21 14:02", local time;
//! - `codes`: ["P0171 stored (Engine 7E8)", "C1241 (ABS 7B0)"];
//! - `monitors_incomplete`, `monitors_complete`: monitor names ("evaporative system");
//! - `misfires`: [{"name": "cylinder 3, average of the last 10 drive cycles", "count": 3}];
//! - `trims`: [{"bank": 1, "at": "warm-idle" or "cruise", "short": 2.3, "long": 11.7, "samples": 18}], in percent.
//!
//! A total trim (or the bank-to-bank gap) is compared with the earliest of the last ten checks that has it: a move of
//! `DRIFT_POINTS` or more is drift; less is steady.

use crate::elm::ObdError;
use crate::link::Link;
use crate::vehicles::Profile;
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

/// The history file in the vehicle's Logs folder.
pub const FILE: &str = "health-history.jsonl";

/// How far a total trim (or the gap between banks) must move, in percentage points, to be called drift.
pub const DRIFT_POINTS: f64 = 5.0;

/// How far a misfire count must rise to be called drift.
pub const MISFIRE_RISE: f64 = 5.0;

/// How many of the latest checks the comparison looks back over.
pub const LOOK_BACK: usize = 10;

/// Rounds of the trim sample (each one request for every value, about a tenth of a second on a vehicle).
const SAMPLES: usize = 20;
/// Samples a condition needs before its trims are kept.
const AT_LEAST: usize = 3;
/// Warm: coolant at 70 °C (158 °F) or more.
const WARM_C: f64 = 70.0;
/// Idle: under 2 km/h (1 mph) and under 1,200 rpm.
const IDLE_KMH: f64 = 2.0;
const IDLE_RPM: f64 = 1200.0;
/// Cruise: 48 km/h (30 mph) or more, steady within 8 km/h (5 mph) over the sample.
const CRUISE_KMH: f64 = 48.0;
const STEADY_KMH: f64 = 8.0;

/// Where a trim was sampled.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Condition {
    WarmIdle,
    Cruise,
}

impl Condition {
    pub fn words(self) -> &'static str {
        match self {
            Condition::WarmIdle => "warm idle",
            Condition::Cruise => "cruise",
        }
    }
}

/// One bank's fuel trims at one condition, averaged over the sample (percent).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Trim {
    pub bank: u8,
    pub at: Condition,
    pub short: f64,
    pub long: f64,
    #[serde(default)]
    pub samples: usize,
}

impl Trim {
    /// Short + long: what the engine computer is correcting in all.
    pub fn total(&self) -> f64 {
        round1(self.short + self.long)
    }
}

/// One misfire count.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Misfire {
    pub name: String,
    pub count: f64,
}

/// One health check, as the history keeps it.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Entry {
    pub when: String,
    #[serde(default)]
    pub codes: Vec<String>,
    #[serde(default)]
    pub monitors_incomplete: Vec<String>,
    #[serde(default)]
    pub monitors_complete: Vec<String>,
    #[serde(default)]
    pub misfires: Vec<Misfire>,
    #[serde(default)]
    pub trims: Vec<Trim>,
}

impl Entry {
    /// The date part of `when`: "2026-11-21".
    fn date(&self) -> &str {
        self.when.get(..10).unwrap_or(&self.when)
    }

    fn trim(&self, bank: u8, at: Condition) -> Option<&Trim> {
        self.trims.iter().find(|t| t.bank == bank && t.at == at)
    }

    /// Bank 1's total minus bank 2's, at `at`, when both were sampled.
    fn gap(&self, at: Condition) -> Option<f64> {
        Some(round1(
            self.trim(1, at)?.total() - self.trim(2, at)?.total(),
        ))
    }
}

/// A health check as read: what it found, and what could not be read.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Check {
    pub vin: Option<String>,
    pub entry: Entry,
    /// What could not be read, or was left out (engine not warm, no trims offered...).
    pub notes: Vec<String>,
}

/// What kind of thing a line of the comparison says.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// Moved enough since the earlier readings to be worth a look.
    Drift,
    /// Something new since the last check: a code, a monitor no longer complete.
    Changed,
    /// Much as before.
    Steady,
    /// No earlier reading to compare with, or just what it is.
    Info,
}

/// One line of the comparison.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Said {
    pub kind: Kind,
    pub text: String,
}

fn said(kind: Kind, text: String) -> Said {
    Said { kind, text }
}

fn round1(x: f64) -> f64 {
    (x * 10.0).round() / 10.0
}

/// "+4", "-2", "+3.5": signed, a decimal only when there is one.
fn signed(x: f64) -> String {
    let x = round1(x);
    if x.fract() == 0.0 {
        format!("{:+}", x as i64)
    } else {
        format!("{x:+.1}")
    }
}

/// "6", "4.5".
fn plain(x: f64) -> String {
    let x = round1(x);
    if x.fract() == 0.0 {
        format!("{}", x as i64)
    } else {
        format!("{x:.1}")
    }
}

/// Temperature in °F first, °C after (the readings come in °C).
fn temp(c: f64) -> String {
    format!("{:.0} °F ({c:.0} °C)", c * 9.0 / 5.0 + 32.0)
}

/// The engine module's reading of a value (7E8, else the first that answered).
fn engine(r: &[crate::j1979::Reading]) -> Option<f64> {
    r.iter()
        .find(|x| x.from == "7E8")
        .or(r.first())
        .map(|x| x.value)
}

/// Whether a module-data value is a misfire count: its name says misfire, and it is a count (its name says count, or its
/// unit is none or counts), not a misfire's engine speed, load, margin or temperature.
pub fn is_misfire_count(name: &str, unit: &str) -> bool {
    let n = name.to_lowercase();
    let u = unit.trim().to_lowercase();
    n.contains("misfire")
        && ![
            "rpm",
            "load",
            "margin",
            "speed",
            "temp",
            "time",
            "threshold",
            "limit",
        ]
        .iter()
        .any(|w| n.contains(w))
        && (n.contains("count") || u.is_empty() || u.starts_with("count"))
}

/// The module-data values a health check reads (`vehicledata::read_values`): the misfire counts.
pub fn misfire_value(v: &icarus_core::moddata::ValueDef) -> bool {
    is_misfire_count(&v.name, &v.unit)
}

/// Reads a health check from the vehicle on `link`, on `profile` (its modules' codes are read; None: OBD-II's range),
/// with the vehicle's module data as already read (`module_values`, for its misfire counts, `misfire_value`; None:
/// none). `progress` is told what is being read. Reads only.
pub fn check(
    link: &mut dyn Link,
    profile: Option<&Profile>,
    module_values: Option<&crate::vehicledata::VehicleValues>,
    progress: &mut dyn FnMut(&str),
) -> Result<Check, ObdError> {
    use crate::j1979::{self, DtcKind};
    let mut c = Check {
        entry: Entry {
            when: crate::report::now(),
            ..Default::default()
        },
        ..Default::default()
    };
    c.vin = j1979::vin(link)?;
    // codes: OBD-II's, then each module's failed ones
    for (kind, label) in [
        (DtcKind::Stored, "stored"),
        (DtcKind::Pending, "pending"),
        (DtcKind::Permanent, "permanent"),
    ] {
        for (from, module, list) in j1979::dtcs(link, kind)? {
            for code in list {
                c.entry
                    .codes
                    .push(format!("{code} {label} ({module} {from})"));
            }
        }
    }
    let fallback;
    let profile = match profile {
        Some(p) => Some(p),
        None => {
            fallback = crate::vehicles::builtin()
                .into_iter()
                .find(|p| p.id == "generic");
            fallback.as_ref()
        }
    };
    let protocol = link.protocol();
    let modules = profile
        .map(|p| crate::vehicles::candidates_on(p, protocol))
        .unwrap_or_default();
    let n = modules.len();
    for (i, m) in modules.into_iter().enumerate() {
        if m.unreachable(link.reach()).is_some() {
            continue;
        }
        let (tx, rx) = (m.tx, m.rx());
        progress(&format!(
            "Health check: module {} of {n}, {} ({tx:03X}): its codes...",
            i + 1,
            m.name
        ));
        let read = crate::uds::at_module(link, &m, |l| {
            if !crate::uds::present(l, tx, rx)? {
                return Ok(None);
            }
            crate::uds::module_dtcs(l, tx, rx).map(Some)
        });
        match read {
            Ok(Some(Ok(codes))) => {
                for d in codes.codes {
                    c.entry
                        .codes
                        .push(format!("{} ({} {tx:03X})", d.code, m.name));
                }
            }
            Ok(Some(Err(why))) => c.notes.push(format!(
                "{tx:03X} {} did not give its codes ({why})",
                m.name
            )),
            Ok(None) => {}
            Err(ObdError::Adapter(why)) => c.notes.push(why),
            Err(e) => return Err(e),
        }
    }
    // readiness monitors (the first module that gives them: the engine's, on most vehicles)
    progress("Health check: readiness monitors and misfire counts...");
    if let Ok(a) = link.request(&[0x01, 0x01]) {
        if let Some(m) = a
            .iter()
            .filter(|x| x.data.len() >= 6 && x.data[..2] == [0x41, 0x01])
            .find_map(|x| j1979::monitors(&x.data[2..]))
        {
            for (name, done) in m.tests {
                if done {
                    c.entry.monitors_complete.push(name.to_string());
                } else {
                    c.entry.monitors_incomplete.push(name.to_string());
                }
            }
        }
    }
    // misfires: Mode 06's counts, then the module data's
    match crate::mode06::read(link) {
        Ok(results) => {
            for t in results
                .iter()
                .filter(|t| (0xA1..=0xAD).contains(&t.mid) && matches!(t.tid, 0x0B | 0x0C))
            {
                let who = if t.mid == 0xA1 {
                    "all cylinders".to_string()
                } else {
                    format!("cylinder {}", t.mid - 0xA1)
                };
                let when = if t.tid == 0x0B {
                    "average of the last 10 drive cycles"
                } else {
                    "this or the last drive cycle"
                };
                c.entry.misfires.push(Misfire {
                    name: format!("{who}, {when}"),
                    count: t.value as f64,
                });
            }
        }
        Err(e) => c
            .notes
            .push(format!("no on-board test results (Mode 06): {e}")),
    }
    if let Some(v) = module_values {
        for m in &v.modules {
            for (name, value, _) in &m.values {
                let v = value.trim();
                let n: String = v
                    .chars()
                    .take_while(|ch| ch.is_ascii_digit() || *ch == '.' || *ch == '-')
                    .collect();
                // what follows the number is its unit
                if !is_misfire_count(name, &v[n.len()..]) {
                    continue;
                }
                if let Ok(count) = n.parse::<f64>() {
                    c.entry.misfires.push(Misfire {
                        name: format!("{} {name}", m.label),
                        count,
                    });
                }
            }
        }
    }
    // fuel trims: a short sample
    let offered: Vec<u8> = j1979::supported(link)?
        .into_iter()
        .flat_map(|s| s.2)
        .collect();
    let banks: Vec<(u8, u8, u8)> = [(1, 0x06, 0x07), (2, 0x08, 0x09)]
        .into_iter()
        .filter(|(_, s, l)| offered.contains(s) && offered.contains(l))
        .collect();
    if banks.is_empty() {
        c.notes
            .push("the engine offers no fuel trims (01 PIDs 06-09)".into());
    } else {
        progress("Health check: sampling the fuel trims (a few seconds)...");
        sample_trims(link, &banks, &mut c)?;
    }
    Ok(c)
}

/// Whether the fuel system is in closed loop now (01 PID 03: each fuel system's status byte 02, or 00 where there is no
/// second one): open loop (warm-up, enrichment, fuel cut while coasting) leaves the trims meaning nothing.
fn closed_loop(link: &mut dyn Link) -> Result<bool, ObdError> {
    let a = match link.request(&[0x01, 0x03]) {
        Ok(a) => a,
        Err(ObdError::NoData) => return Ok(false),
        Err(e) => return Err(e),
    };
    let ok = |x: &crate::elm::Answer| x.data.len() >= 4 && x.data[..2] == [0x41, 0x03];
    Ok(a.iter()
        .filter(|x| ok(x))
        .find(|x| x.from == "7E8")
        .or(a.iter().find(|x| ok(x)))
        .is_some_and(|x| x.data[2] == 0x02 && matches!(x.data[3], 0x00 | 0x02)))
}

/// Samples the trims of `banks` ((bank, short PID, long PID)) with coolant, engine speed and road speed, and keeps each
/// condition's average that had enough samples.
fn sample_trims(
    link: &mut dyn Link,
    banks: &[(u8, u8, u8)],
    c: &mut Check,
) -> Result<(), ObdError> {
    let mut pids = vec![0x05, 0x0C, 0x0D];
    for (_, s, l) in banks {
        pids.extend([*s, *l]);
    }
    // (condition, bank) -> (short sum, long sum, n)
    let mut sums: std::collections::BTreeMap<(Condition, u8), (f64, f64, usize)> =
        Default::default();
    let (mut off, mut cold, mut between, mut open) = (0usize, None::<f64>, 0usize, 0usize);
    // the cruise samples' road speeds: a cruise is kept only when steady
    let (mut slowest, mut fastest) = (f64::INFINITY, f64::NEG_INFINITY);
    for _ in 0..SAMPLES {
        let r = crate::j1979::read_many(link, &pids)?;
        let v = |p: u8| r.iter().find(|x| x.0 == p).and_then(|x| engine(&x.1));
        let (Some(coolant), Some(rpm), Some(kmh)) = (v(0x05), v(0x0C), v(0x0D)) else {
            continue;
        };
        if rpm < 300.0 {
            off += 1;
            continue;
        }
        if coolant < WARM_C {
            cold = Some(coolant);
            continue;
        }
        let at = if kmh < IDLE_KMH && rpm < IDLE_RPM {
            Condition::WarmIdle
        } else if kmh >= CRUISE_KMH {
            Condition::Cruise
        } else {
            between += 1;
            continue;
        };
        // open loop (enrichment, fuel cut while coasting): its trims are not the engine's correction
        if !closed_loop(link)? {
            open += 1;
            continue;
        }
        if at == Condition::Cruise {
            slowest = slowest.min(kmh);
            fastest = fastest.max(kmh);
        }
        for (bank, s, l) in banks {
            if let (Some(s), Some(l)) = (v(*s), v(*l)) {
                let e = sums.entry((at, *bank)).or_default();
                e.0 += s;
                e.1 += l;
                e.2 += 1;
            }
        }
    }
    let unsteady = fastest - slowest > STEADY_KMH;
    if unsteady {
        c.notes.push(format!(
            "the road speed changed by {:.0} mph while sampling: no cruise trims kept (hold a steady speed)",
            (fastest - slowest) / 1.609_344
        ));
    }
    if open > 0 {
        c.notes.push(format!(
            "{open} sample(s) left out: the fuel system was in open loop (warm-up, enrichment or fuel cut)"
        ));
    }
    for ((at, bank), (s, l, n)) in sums {
        if at == Condition::Cruise && unsteady {
            continue;
        }
        if n >= AT_LEAST {
            c.entry.trims.push(Trim {
                bank,
                at,
                short: round1(s / n as f64),
                long: round1(l / n as f64),
                samples: n,
            });
        }
    }
    if off > 0 && c.entry.trims.is_empty() {
        c.notes.push(
            "the engine is not running: start it and let it warm up to idle for the fuel trims"
                .into(),
        );
    } else if let (Some(t), true) = (cold, c.entry.trims.is_empty()) {
        c.notes.push(format!(
            "the engine is not warm yet (coolant {}): the fuel trims are kept from {} up",
            temp(t),
            temp(WARM_C)
        ));
    } else if between > 0 && c.entry.trims.is_empty() {
        c.notes.push(
            "neither at idle nor at a cruise (30 mph or more) while sampling: no fuel trims kept"
                .into(),
        );
    }
    if !c.entry.trims.iter().any(|t| t.at == Condition::Cruise) {
        c.notes.push(
            "no cruise in the sample: a check while driving at a steady 30 mph or more adds the cruise trims".into(),
        );
    }
    Ok(())
}

/// The earliest of the last `LOOK_BACK` entries of `history` (oldest first) for which `has` gives a value: (its date,
/// the value).
fn baseline<T>(history: &[Entry], has: impl Fn(&Entry) -> Option<T>) -> Option<(&str, T)> {
    let from = history.len().saturating_sub(LOOK_BACK);
    history[from..]
        .iter()
        .find_map(|e| has(e).map(|v| (e.date(), v)))
}

/// What a health check says beside this vehicle's earlier checks (`history`, oldest first; the check itself not in it).
pub fn compare(now: &Entry, history: &[Entry]) -> Vec<Said> {
    let mut out = Vec::new();
    let last = history.last();
    // codes
    if now.codes.is_empty() {
        out.push(said(Kind::Info, "no failed trouble codes".into()));
    }
    match last {
        Some(l) => {
            let new: Vec<&String> = now.codes.iter().filter(|c| !l.codes.contains(c)).collect();
            let gone: Vec<&String> = l.codes.iter().filter(|c| !now.codes.contains(c)).collect();
            if !new.is_empty() {
                out.push(said(
                    Kind::Changed,
                    format!(
                        "new since {}: {}",
                        l.date(),
                        new.iter()
                            .map(|s| s.as_str())
                            .collect::<Vec<_>>()
                            .join(", ")
                    ),
                ));
            }
            let kept: Vec<&String> = now.codes.iter().filter(|c| l.codes.contains(c)).collect();
            if !kept.is_empty() {
                out.push(said(
                    Kind::Steady,
                    format!(
                        "still there since {}: {}",
                        l.date(),
                        kept.iter()
                            .map(|s| s.as_str())
                            .collect::<Vec<_>>()
                            .join(", ")
                    ),
                ));
            }
            if !gone.is_empty() {
                out.push(said(
                    Kind::Info,
                    format!(
                        "gone since {}: {}",
                        l.date(),
                        gone.iter()
                            .map(|s| s.as_str())
                            .collect::<Vec<_>>()
                            .join(", ")
                    ),
                ));
            }
        }
        None if !now.codes.is_empty() => out.push(said(
            Kind::Info,
            format!("trouble codes: {}", now.codes.join(", ")),
        )),
        None => {}
    }
    // monitors
    if !now.monitors_incomplete.is_empty() {
        out.push(said(
            Kind::Info,
            format!(
                "monitors not yet run: {}",
                now.monitors_incomplete.join(", ")
            ),
        ));
    } else if !now.monitors_complete.is_empty() {
        out.push(said(Kind::Info, "every readiness monitor complete".into()));
    }
    if let Some(l) = last {
        let undone: Vec<&String> = now
            .monitors_incomplete
            .iter()
            .filter(|m| l.monitors_complete.contains(m))
            .collect();
        if !undone.is_empty() {
            out.push(said(
                Kind::Changed,
                format!(
                    "complete on {}, not now: {} (codes cleared or the battery disconnected since?)",
                    l.date(),
                    undone.iter().map(|s| s.as_str()).collect::<Vec<_>>().join(", ")
                ),
            ));
        }
    }
    // misfires
    for m in &now.misfires {
        let before = baseline(history, |e| {
            e.misfires
                .iter()
                .find(|x| x.name == m.name)
                .map(|x| x.count)
        });
        let text = match before {
            Some((date, b)) if m.count - b >= MISFIRE_RISE => {
                out.push(said(
                    Kind::Drift,
                    format!(
                        "misfires, {}: rose from {} to {} since {date}",
                        m.name,
                        plain(b),
                        plain(m.count)
                    ),
                ));
                continue;
            }
            Some((date, b)) => format!(
                "misfires, {}: {} ({} on {date})",
                m.name,
                plain(m.count),
                plain(b)
            ),
            None => format!("misfires, {}: {}", m.name, plain(m.count)),
        };
        let kind = if before.is_some() {
            Kind::Steady
        } else {
            Kind::Info
        };
        // a cylinder with none, steady, needs no line of its own
        if m.count == 0.0 && before.is_none_or(|(_, b)| b == 0.0) {
            continue;
        }
        out.push(said(kind, text));
    }
    if !now.misfires.is_empty() && now.misfires.iter().all(|m| m.count == 0.0) {
        out.push(said(Kind::Info, "no misfires counted".into()));
    }
    // trims, each bank at each condition, and the gap between the banks
    for t in &now.trims {
        let what = format!("bank {} total trim ({})", t.bank, t.at.words());
        let total = t.total();
        match baseline(history, |e| e.trim(t.bank, t.at).map(Trim::total)) {
            Some((date, b)) if round1((total - b).abs()) >= DRIFT_POINTS => out.push(said(
                Kind::Drift,
                format!(
                    "{what} moved from {} to {} since {date}: {} points {}",
                    signed(b),
                    signed(total),
                    plain((total - b).abs()),
                    if total > b {
                        "leaner-correcting (more fuel added)"
                    } else {
                        "richer-correcting (fuel taken away)"
                    }
                ),
            )),
            Some((date, b)) => out.push(said(
                Kind::Steady,
                format!("{what} {}: steady ({} on {date})", signed(total), signed(b)),
            )),
            None => out.push(said(
                Kind::Info,
                format!(
                    "{what} {} (short {}, long {}): the first reading kept",
                    signed(total),
                    signed(t.short),
                    signed(t.long)
                ),
            )),
        }
        if total.abs() > 10.0 {
            out.push(said(
                Kind::Info,
                format!(
                    "{what} is beyond 10 points either way: worth a look whatever its history ({})",
                    if total > 0.0 {
                        "adding fuel: a vacuum leak, a weak pump or a dirty mass airflow sensor are common"
                    } else {
                        "taking fuel away: a leaking injector or high fuel pressure are common"
                    }
                ),
            ));
        }
    }
    // the gap is bank 1's total minus bank 2's, compared signed: +3 to -3 is a move of 6, not steady
    for at in [Condition::WarmIdle, Condition::Cruise] {
        let Some(gap) = now.gap(at) else { continue };
        let what = format!("bank-to-bank gap ({}, bank 1 minus bank 2)", at.words());
        match baseline(history, |e| e.gap(at)) {
            Some((date, b)) if round1((gap - b).abs()) >= DRIFT_POINTS => out.push(said(
                Kind::Drift,
                format!(
                    "{what} moved from {} to {} since {date}: {} points",
                    signed(b),
                    signed(gap),
                    plain((gap - b).abs())
                ),
            )),
            Some((date, b)) => out.push(said(
                Kind::Steady,
                format!("{what} {}: steady ({} on {date})", signed(gap), signed(b)),
            )),
            None => out.push(said(Kind::Info, format!("{what} {}", signed(gap)))),
        }
    }
    out
}

/// The vehicle's history file: its folder's Logs (the simulated vehicle's in Icarus's state folder).
pub fn path(vin: &str) -> Result<PathBuf, String> {
    crate::garage::logs_dir(Some(vin)).map(|d| d.join(FILE))
}

/// The history in `file`, oldest first (none when it does not exist; a line that does not read is skipped).
pub fn load_in(file: &Path) -> Vec<Entry> {
    std::fs::read_to_string(file)
        .unwrap_or_default()
        .lines()
        .filter(|l| !l.trim().is_empty())
        .filter_map(|l| serde_json::from_str(l).ok())
        .collect()
}

/// Adds one check to the history in `file` (the folder made if needed).
pub fn append_in(file: &Path, e: &Entry) -> Result<(), String> {
    use std::io::Write;
    if let Some(d) = file.parent() {
        std::fs::create_dir_all(d).map_err(|x| format!("{}: {x}", d.display()))?;
    }
    let line = serde_json::to_string(e).map_err(|x| x.to_string())?;
    std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(file)
        .and_then(|mut f| writeln!(f, "{line}"))
        .map_err(|x| format!("{}: {x}", file.display()))
}

/// A health check beside the history in `file`: the comparison with what was there, then the check added to it.
pub fn keep_in(file: &Path, c: &Check) -> Result<Vec<Said>, String> {
    let history = load_in(file);
    let said = compare(&c.entry, &history);
    append_in(file, &c.entry)?;
    Ok(said)
}

/// What a health check says and where it was kept, for the command line, the window and plain reports: the comparison
/// with the vehicle's own history (kept in its folder), or, with no VIN, the check alone (it could be any vehicle's, so
/// it is not kept).
#[derive(Clone, Debug, PartialEq)]
pub struct Health {
    pub check: Check,
    pub said: Vec<Said>,
    /// Where the history is, or why the check was not kept.
    pub kept: Result<PathBuf, String>,
}

/// `check`'s result kept in the vehicle's history (`path`) and compared with it.
pub fn keep(c: Check) -> Health {
    let unkept = |c: Check, why: &str| Health {
        said: compare(&c.entry, &[]),
        check: c,
        kept: Err(why.into()),
    };
    let Some(vin) = c.vin.clone() else {
        return unkept(
            c,
            "the VIN could not be read, so the check was not kept (it could be any vehicle's)",
        );
    };
    // every blank bench ECU would share one history (and a folder named 00000...)
    if crate::benchmake::blank(&vin) {
        return unkept(
            c,
            "blank VIN, so the check was not kept (it could be any bench ECU's)",
        );
    }
    match path(&vin).and_then(|p| keep_in(&p, &c).map(|s| (p, s))) {
        Ok((p, said)) => Health {
            check: c,
            said,
            kept: Ok(p),
        },
        Err(why) => Health {
            said: compare(&c.entry, &[]),
            check: c,
            kept: Err(why),
        },
    }
}

impl Health {
    /// As text: each line with a mark (! drift, * new, = steady, - else), the notes, and where it was kept.
    pub fn text(&self) -> String {
        let mut o = format!("Health check, {}\n", self.check.entry.when);
        for s in &self.said {
            let mark = match s.kind {
                Kind::Drift => "!",
                Kind::Changed => "*",
                Kind::Steady => "=",
                Kind::Info => "-",
            };
            o += &format!("  {mark} {}\n", s.text);
        }
        for n in &self.check.notes {
            o += &format!("  note: {n}\n");
        }
        o += &match &self.kept {
            Ok(p) => format!("Kept in {}.\n", p.display()),
            Err(why) => format!("Not kept: {why}.\n"),
        };
        o
    }

    /// How many lines are drift.
    pub fn drifts(&self) -> usize {
        self.said.iter().filter(|s| s.kind == Kind::Drift).count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elm::Elm;
    use crate::sim::SimCar;

    fn sim() -> Elm {
        Elm::connect(Box::new(SimCar::new())).unwrap()
    }

    fn toyota() -> Profile {
        crate::vehicles::builtin()
            .into_iter()
            .find(|p| p.id == "toyota")
            .unwrap()
    }

    fn earlier(when: &str, trims: Vec<Trim>) -> Entry {
        Entry {
            when: when.into(),
            trims,
            ..Default::default()
        }
    }

    fn trim(bank: u8, short: f64, long: f64) -> Trim {
        Trim {
            bank,
            at: Condition::WarmIdle,
            short,
            long,
            samples: 20,
        }
    }

    /// What the simulated vehicle gives: its stored code, the evaporative monitor not yet run, cylinder 3's misfires, and
    /// bank 1's trims at a warm idle (it has one bank, and is parked).
    #[test]
    fn a_check_on_the_simulated_vehicle_reads_only() {
        let mut e = sim();
        e.take_sent();
        let c = check(&mut e, Some(&toyota()), None, &mut |_| {}).unwrap();
        assert_eq!(c.vin.as_deref(), Some(crate::sim::VIN));
        assert!(
            c.entry.codes.iter().any(|x| x.starts_with("P0171 stored")),
            "{:?}",
            c.entry.codes
        );
        assert!(c
            .entry
            .monitors_incomplete
            .iter()
            .any(|m| m == "evaporative system"));
        assert!(c
            .entry
            .misfires
            .iter()
            .any(|m| m.name == "cylinder 3, this or the last drive cycle" && m.count == 7.0));
        assert_eq!(c.entry.trims.len(), 1, "{:?}", c.entry.trims);
        let t = &c.entry.trims[0];
        assert_eq!((t.bank, t.at), (1, Condition::WarmIdle));
        assert!((t.long - 11.7).abs() < 0.1 && t.total() > 11.0, "{t:?}");
        assert!(c.notes.iter().any(|n| n.contains("no cruise")));
        // nothing but reads went out: services 01, 03, 06, 07, 09, 0A, and to the modules 3E, 19, 18, 13
        for (_, id, req) in e.take_sent() {
            assert!(
                matches!(
                    req.first(),
                    Some(0x01 | 0x03 | 0x06 | 0x07 | 0x09 | 0x0A | 0x3E | 0x19 | 0x18 | 0x13)
                ),
                "{id:03X} {req:02X?}"
            );
            assert!(!crate::lock::changes_vehicle(&req), "{req:02X?}");
        }
    }

    /// ICA-214's acceptance: on the simulated vehicle with a stored history, a total trim that moved 5 points is named as
    /// drift, and one that moved 1 point is steady, not drift. The history goes through its file.
    #[test]
    fn a_trim_that_moved_5_points_is_drift_and_a_steady_one_is_not() {
        let now = check(&mut sim(), Some(&toyota()), None, &mut |_| {}).unwrap();
        let total = now.entry.trims[0].total();
        let dir = std::env::temp_dir().join(format!("icarus-health-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        // moved 5 points since 2026-11-21 (the long trim was 5 lower)
        let drifted = dir.join("drifted").join(FILE);
        append_in(
            &drifted,
            &earlier("2026-11-21 09:00", vec![trim(1, total - 5.0 - 11.7, 11.7)]),
        )
        .unwrap();
        append_in(
            &drifted,
            &earlier("2026-11-24 09:00", vec![trim(1, total - 3.0 - 11.7, 11.7)]),
        )
        .unwrap();
        // a fresh simulated vehicle answers just as the first did
        let c = check(&mut sim(), Some(&toyota()), None, &mut |_| {}).unwrap();
        assert_eq!(c.entry.trims[0].total(), total);
        let said = keep_in(&drifted, &c).unwrap();
        let drift: Vec<&Said> = said.iter().filter(|s| s.kind == Kind::Drift).collect();
        assert_eq!(drift.len(), 1, "{said:?}");
        assert!(
            drift[0]
                .text
                .starts_with("bank 1 total trim (warm idle) moved from")
                && drift[0]
                    .text
                    .contains("since 2026-11-21: 5 points leaner-correcting"),
            "{}",
            drift[0].text
        );
        assert_eq!(load_in(&drifted).len(), 3, "the check was kept");
        // steady: 1 point since 2026-11-21
        let steady = dir.join("steady").join(FILE);
        append_in(
            &steady,
            &earlier("2026-11-21 09:00", vec![trim(1, total - 1.0 - 11.7, 11.7)]),
        )
        .unwrap();
        let said = keep_in(&steady, &c).unwrap();
        assert!(said.iter().all(|s| s.kind != Kind::Drift), "{said:?}");
        assert!(
            said.iter().any(|s| s.kind == Kind::Steady
                && s.text.starts_with("bank 1 total trim (warm idle)")
                && s.text.contains("steady")),
            "{said:?}"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The owner's example, two banks: bank 2 from -2 to +4 is drift, leaner-correcting, the gap's growth too; bank 1 steady.
    /// A move the other way is richer-correcting. Lines that do not read in the file are skipped.
    #[test]
    fn two_banks_the_gap_and_the_words() {
        let before = earlier(
            "2026-11-21 09:00",
            vec![trim(1, 0.0, -1.0), trim(2, -1.0, -1.0)],
        );
        let now = earlier(
            "2026-11-28 09:00",
            vec![trim(1, 0.5, -1.0), trim(2, 1.0, 3.0)],
        );
        let said = compare(&now, std::slice::from_ref(&before));
        let text: Vec<&str> = said
            .iter()
            .filter(|s| s.kind == Kind::Drift)
            .map(|s| s.text.as_str())
            .collect();
        // the gap (bank 1 minus bank 2) went from +1 to -4.5: 5.5 points, compared signed
        assert_eq!(
            text,
            [
                "bank 2 total trim (warm idle) moved from -2 to +4 since 2026-11-21: 6 points leaner-correcting (more fuel added)",
                "bank-to-bank gap (warm idle, bank 1 minus bank 2) moved from +1 to -4.5 since 2026-11-21: 5.5 points",
            ]
        );
        assert!(said.iter().any(|s| s.kind == Kind::Steady
            && s.text == "bank 1 total trim (warm idle) -0.5: steady (-1 on 2026-11-21)"));
        let richer = compare(&before, &[now]);
        assert!(richer
            .iter()
            .any(|s| s.kind == Kind::Drift && s.text.contains("6 points richer-correcting")));
        // the file: one JSON object a line; a broken line is skipped
        let dir = std::env::temp_dir().join(format!("icarus-health-file-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let f = dir.join(FILE);
        append_in(&f, &before).unwrap();
        std::fs::write(
            &f,
            format!("{}not json\n", std::fs::read_to_string(&f).unwrap()),
        )
        .unwrap();
        assert_eq!(load_in(&f), vec![before]);
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Review of ICA-214: a gap from +3 to -3 (the same size, the other bank richer now) is a move of 6, not steady; a
    /// gap that kept its size and side is steady.
    #[test]
    fn the_gap_is_compared_signed() {
        let before = earlier(
            "2026-11-21 09:00",
            vec![trim(1, 0.0, 3.0), trim(2, 0.0, 0.0)],
        );
        let flipped = earlier(
            "2026-11-28 09:00",
            vec![trim(1, 0.0, 0.0), trim(2, 0.0, 3.0)],
        );
        let said = compare(&flipped, std::slice::from_ref(&before));
        assert!(
            said.iter().any(|s| s.kind == Kind::Drift
                && s.text.starts_with("bank-to-bank gap")
                && s.text.contains("from +3 to -3")
                && s.text.ends_with("6 points")),
            "{said:?}"
        );
        let same = earlier(
            "2026-11-28 09:00",
            vec![trim(1, 1.0, 3.0), trim(2, 1.0, 0.0)],
        );
        let said = compare(&same, &[before]);
        assert!(said.iter().all(|s| s.kind != Kind::Drift), "{said:?}");
    }

    /// Review of ICA-214: only misfire counts are counted, not a misfire's engine speed, load or margin.
    #[test]
    fn only_misfire_counts_count() {
        assert!(is_misfire_count("Cylinder 3 misfire count", "rpm"));
        assert!(is_misfire_count("Misfire cylinder 1", ""));
        assert!(is_misfire_count("Misfires, cylinder 2", "counts"));
        for (name, unit) in [
            ("Misfire RPM", "rpm"),
            ("Misfire RPM", ""),
            ("Misfire Load", "%"),
            ("Misfire Margin", ""),
            ("Misfire cylinder 1", "%"),
            ("Knock count", ""),
        ] {
            assert!(!is_misfire_count(name, unit), "{name} ({unit})");
        }
        let values = crate::vehicledata::VehicleValues {
            set: "t".into(),
            attribution: String::new(),
            notes: vec![],
            modules: vec![crate::vehicledata::ModuleValues {
                label: "7E0".into(),
                name: "engine".into(),
                values: vec![
                    ("Misfire RPM".into(), "850 rpm".into(), false),
                    ("Misfire Load".into(), "31 %".into(), false),
                    ("Cylinder 2 misfire count".into(), "4".into(), false),
                ],
                notes: vec![],
            }],
        };
        let c = check(&mut sim(), Some(&toyota()), Some(&values), &mut |_| {}).unwrap();
        let from_data: Vec<&Misfire> = c
            .entry
            .misfires
            .iter()
            .filter(|m| m.name.starts_with("7E0"))
            .collect();
        assert_eq!(
            from_data,
            [&Misfire {
                name: "7E0 Cylinder 2 misfire count".into(),
                count: 4.0
            }]
        );
    }

    /// Review of ICA-214: in open loop (warm-up, enrichment, fuel cut) no trim is kept, and that is said. A check is told
    /// what is being read as it goes.
    #[test]
    fn open_loop_keeps_no_trims() {
        let mut e = Elm::connect(Box::new(SimCar::new().in_open_loop())).unwrap();
        let mut said = Vec::new();
        let c = check(&mut e, Some(&toyota()), None, &mut |s| {
            said.push(s.to_string())
        })
        .unwrap();
        assert!(c.entry.trims.is_empty(), "{:?}", c.entry.trims);
        assert!(
            c.notes.iter().any(|n| n.contains("open loop")),
            "{:?}",
            c.notes
        );
        assert!(said.iter().any(|s| s.contains("module 1 of")), "{said:?}");
        assert!(
            said.iter().any(|s| s.contains("sampling the fuel trims")),
            "{said:?}"
        );
    }

    /// Review of ICA-214: a check of a blank bench ECU (no VIN, 17 zeros) is shown, never kept: every such ECU would share
    /// one history and a folder named 00000...
    #[test]
    fn a_blank_vin_is_not_kept() {
        for vin in [None, Some("00000000000000000".to_string())] {
            let h = keep(Check {
                vin: vin.clone(),
                ..Default::default()
            });
            assert!(h.kept.is_err(), "{vin:?}");
        }
        let h = keep(Check {
            vin: Some("00000000000000000".into()),
            ..Default::default()
        });
        assert!(h.kept.unwrap_err().starts_with("blank VIN"));
    }
}
