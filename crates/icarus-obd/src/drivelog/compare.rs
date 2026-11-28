//! Two drives or more compared at matching conditions (ICA-210): only the rows where the engine is warm, in closed loop and
//! running, grouped by air flow (each bank's total fuel trim, short + long term) and by engine speed and load (timing
//! advance), side by side with each group's row count. A group with fewer rows than the minimum is marked thin and never
//! summarized, and the difference from the first recording is given only where neither group is thin. The recordings may
//! hold different columns in a different order: each is found by its name. Offline: only the files are read.

use super::{adapter_of, cells, series, Cell, Cut, Series};
use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

/// Warm: coolant at this temperature (°C; 176 °F) or more. Higher than the summary's `WARM_C`: comparing drives wants the
/// engine fully warm, its trims settled.
pub const WARM_C: f64 = 80.0;
/// Closed loop: commanded lambda in this range (both ends included). Outside it the engine is aiming rich or lean on purpose
/// (power enrichment, warm-up, fuel cut) and the trims and timing are not comparable.
pub const CLOSED_LOOP: (f64, f64) = (0.98, 1.02);
/// The engine running: over this engine speed (rpm).
pub const RUNNING_RPM: f64 = 400.0;
/// A group with fewer rows than this is thin: shown with its count, never summarized (`--min-rows` changes it).
pub const MIN_ROWS: usize = 10;
/// The air flow bands (g/s) the fuel trims are grouped by.
pub const AIR_FLOW: &[f64] = &[0.0, 4.0, 8.0, 15.0, 30.0, 60.0, 200.0];
/// The engine speed (rpm) and engine load (%) bands timing is grouped by; the top load band reaches 101 so 100 % is in it.
pub const RPM: &[f64] = &[1000.0, 1800.0, 2600.0, 3600.0, 5500.0];
pub const LOAD: &[f64] = &[20.0, 40.0, 60.0, 80.0, 101.0];

/// A carried-forward coolant reading older than this (seconds) is not used: coolant read in turn is there every few rows,
/// and it changes slowly, but not across a stop with the engine off.
const COOLANT_HOLD_S: f64 = 60.0;

/// One recording in a comparison.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Recording {
    /// The file's name.
    pub name: String,
    /// The adapter its first line names (`# adapter: ...`), when it does.
    pub adapter: Option<String>,
    /// The fuel or calibration it was driven on (91 E0, 87, a tune's name), shown under the name when there is one. Not
    /// recorded yet (ICA-113 and ICA-211 will set it); a side of drives pooled (`compare_pooled`) says how many it holds.
    pub tag: Option<String>,
    /// Its rows, and how many of them are at the conditions compared.
    pub rows: usize,
    pub kept: usize,
    /// What the comparison needs that it lacks, in words ("commanded lambda").
    pub lacking: Vec<&'static str>,
}

/// One group (a band of air flow, or of engine speed and load): its name, and one cell per recording, in their order.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Group {
    pub label: String,
    pub cells: Vec<Cell>,
}

/// What a table shows: total fuel trim of each bank by air flow, or timing by engine speed and load.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Table {
    Trims,
    Timing,
}

impl Table {
    fn values(self) -> usize {
        match self {
            Table::Trims => 2,
            Table::Timing => 1,
        }
    }

    pub fn title(self) -> &'static str {
        match self {
            Table::Trims => "Total fuel trim (short + long term), bank 1 / bank 2, %, by air flow",
            Table::Timing => {
                "Timing advance (degrees before top dead center) by engine speed and load"
            }
        }
    }

    /// The first column's heading.
    pub fn by(self) -> &'static str {
        match self {
            Table::Trims => "Air flow",
            Table::Timing => "Engine speed, load",
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Comparison {
    pub recordings: Vec<Recording>,
    /// Groups under this many rows are thin.
    pub min_rows: usize,
    pub trims: Vec<Group>,
    pub timing: Vec<Group>,
}

/// Column `c` at row `i`, when the recording has both.
fn at(s: &Series, c: Option<usize>, i: usize) -> Option<f64> {
    s.cols.get(c?)?.get(i).copied().flatten()
}

/// One recording's cells: (the recording, trims by air flow band, timing by [rpm band, load band]).
type Cells = BTreeMap<Vec<usize>, Cell>;

fn one(name: &str, text: &str) -> Result<(Recording, Cells, Cells), String> {
    let s = series(text).map_err(|e| format!("{name}: {e}"))?;
    let rpm = s.find(&["engine speed"]);
    let load = s.find(&["engine load"]);
    let coolant = s.find(&["coolant"]);
    let lambda = s.find(&["commanded lambda"]);
    let maf = s.find(&["mass air flow"]);
    let timing = s.find(&["timing advance"]);
    let trims = [
        s.find(&["short-term fuel trim", "bank 1"]),
        s.find(&["long-term fuel trim", "bank 1"]),
        s.find(&["short-term fuel trim", "bank 2"]),
        s.find(&["long-term fuel trim", "bank 2"]),
    ];
    let mut lacking = vec![];
    for (c, what) in [
        (rpm, "engine speed"),
        (coolant, "coolant temperature"),
        (lambda, "commanded lambda"),
        (maf, "mass air flow"),
        (trims[0].and(trims[1]), "fuel trims"),
        (load, "engine load"),
        (timing, "timing advance"),
    ] {
        if c.is_none() {
            lacking.push(what);
        }
    }
    // coolant read in turn shows every few rows: the last reading holds until the next (for a minute at most)
    let mut warm_at: Vec<Option<f64>> = Vec::with_capacity(s.t.len());
    let mut last: Option<(f64, f64)> = None;
    for i in 0..s.t.len() {
        if let Some(v) = at(&s, coolant, i) {
            last = Some((s.t[i], v));
        }
        warm_at.push(
            last.filter(|(t, _)| s.t[i] - t <= COOLANT_HOLD_S)
                .map(|l| l.1),
        );
    }
    let base = |i: usize| {
        warm_at[i].is_some_and(|c| c >= WARM_C)
            && at(&s, lambda, i).is_some_and(|l| (CLOSED_LOOP.0..=CLOSED_LOOP.1).contains(&l))
            && at(&s, rpm, i).is_some_and(|r| r > RUNNING_RPM)
    };
    let total =
        |st: Option<usize>, lt: Option<usize>, i: usize| Some(at(&s, st, i)? + at(&s, lt, i)?);
    let bank1 = |i: usize| total(trims[0], trims[1], i);
    let bank2 = |i: usize| total(trims[2], trims[3], i);
    let advance = |i: usize| at(&s, timing, i);
    let kept = (0..s.t.len()).filter(|i| base(*i)).count();
    let by_air = match maf {
        Some(m) => cells(
            &s,
            &[(m, Cut::Edges(AIR_FLOW))],
            |i| base(i) && (bank1(i).is_some() || bank2(i).is_some()),
            &[&bank1, &bank2],
        ),
        None => Cells::new(),
    };
    let by_speed = match (rpm, load) {
        (Some(r), Some(l)) => cells(
            &s,
            &[(r, Cut::Edges(RPM)), (l, Cut::Edges(LOAD))],
            |i| base(i) && advance(i).is_some(),
            &[&advance],
        ),
        _ => Cells::new(),
    };
    Ok((
        Recording {
            name: name.to_string(),
            adapter: adapter_of(text),
            tag: None,
            rows: s.t.len(),
            kept,
            lacking,
        },
        by_air,
        by_speed,
    ))
}

fn band(edges: &[f64], k: usize, cap: f64) -> String {
    format!("{:.0}-{:.0}", edges[k], edges[k + 1].min(cap))
}

/// The groups any recording has rows in, in band order, one cell per recording.
fn groups(per: &[Cells], label: impl Fn(&[usize]) -> String) -> Vec<Group> {
    let keys: BTreeSet<&Vec<usize>> = per.iter().flat_map(|m| m.keys()).collect();
    keys.into_iter()
        .map(|k| Group {
            label: label(k),
            cells: per
                .iter()
                .map(|m| m.get(k).cloned().unwrap_or_default())
                .collect(),
        })
        .collect()
}

/// Compares recordings given as (name, text), the first the one the others are measured against.
pub fn compare(recordings: &[(String, String)], min_rows: usize) -> Result<Comparison, String> {
    if recordings.len() < 2 {
        return Err("comparing needs two recordings or more".into());
    }
    let parts = recordings
        .iter()
        .map(|(name, text)| one(name, text))
        .collect::<Result<Vec<_>, String>>()?;
    Ok(assemble(parts, min_rows))
}

/// Several recordings as one (a side of a before-and-after, ICA-215): their rows pooled into the same groups, named `name`.
/// What every one of them lacks is said; the adapters they name are listed once each.
fn pool(name: &str, recordings: &[(String, String)]) -> Result<(Recording, Cells, Cells), String> {
    if recordings.is_empty() {
        return Err(format!("{name}: no recordings"));
    }
    let mut r = Recording {
        name: name.to_string(),
        tag: Some(format!(
            "{} drive{}",
            recordings.len(),
            if recordings.len() == 1 { "" } else { "s" }
        )),
        ..Default::default()
    };
    let (mut air, mut speed) = (Cells::new(), Cells::new());
    let mut adapters: Vec<String> = vec![];
    for (i, (n, text)) in recordings.iter().enumerate() {
        let (one_r, a, s) = one(n, text)?;
        r.rows += one_r.rows;
        r.kept += one_r.kept;
        r.lacking = if i == 0 {
            one_r.lacking
        } else {
            r.lacking
                .into_iter()
                .filter(|l| one_r.lacking.contains(l))
                .collect()
        };
        if let Some(ad) = one_r.adapter.filter(|ad| !adapters.contains(ad)) {
            adapters.push(ad);
        }
        for (to, from) in [(&mut air, a), (&mut speed, s)] {
            for (k, c) in from {
                to.entry(k).or_default().add(&c);
            }
        }
    }
    r.adapter = (!adapters.is_empty()).then(|| adapters.join(", "));
    Ok((r, air, speed))
}

/// Compares sides of recordings pooled, each given as (its name, its recordings as (name, text)): the first side is the one
/// the others are measured against (ICA-215: the drives before a service, then the drives after it).
pub fn compare_pooled(
    sides: &[(String, Vec<(String, String)>)],
    min_rows: usize,
) -> Result<Comparison, String> {
    if sides.len() < 2 {
        return Err("comparing needs two sides or more".into());
    }
    let parts = sides
        .iter()
        .map(|(name, recs)| pool(name, recs))
        .collect::<Result<Vec<_>, String>>()?;
    Ok(assemble(parts, min_rows))
}

/// The comparison of recordings already sorted into their groups.
fn assemble(parts: Vec<(Recording, Cells, Cells)>, min_rows: usize) -> Comparison {
    let mut c = Comparison {
        min_rows,
        ..Default::default()
    };
    let (mut air, mut speed) = (vec![], vec![]);
    for (r, a, s) in parts {
        c.recordings.push(r);
        air.push(a);
        speed.push(s);
    }
    c.trims = groups(&air, |k| format!("{} g/s", band(AIR_FLOW, k[0], f64::MAX)));
    c.timing = groups(&speed, |k| {
        format!(
            "{} rpm, {} %",
            band(RPM, k[0], f64::MAX),
            band(LOAD, k[1], 100.0)
        )
    });
    c
}

/// The misfire values a recording carries (columns whose name says misfire, such as a module's "Cylinder #1 misfire
/// count"; the standard values have none) and how much each rose over the recording: every step up between one reading
/// and the next added, so a counter that starts again (a new drive cycle) is still counted. With the recording's length
/// in seconds. No such column: an empty list.
pub fn misfires(text: &str) -> Result<(Vec<(String, f64)>, f64), String> {
    let s = series(text)?;
    let seconds = s.t.last().copied().unwrap_or(0.0) - s.t.first().copied().unwrap_or(0.0);
    let mut out = vec![];
    for (name, col) in s.names.iter().zip(&s.cols) {
        if !name.to_lowercase().contains("misfire") || name.ends_with("[hex]") {
            continue;
        }
        let mut rise = 0.0;
        let mut last: Option<f64> = None;
        for v in col.iter().flatten().filter(|v| v.is_finite()) {
            if let Some(l) = last {
                if *v > l {
                    rise += v - l;
                }
            }
            last = Some(*v);
        }
        out.push((name.clone(), rise));
    }
    Ok((out, seconds.max(0.0)))
}

/// Compares recording files, the first the one the others are measured against.
pub fn compare_files(paths: &[&Path], min_rows: usize) -> Result<Comparison, String> {
    let mut v = vec![];
    for p in paths {
        let text = std::fs::read_to_string(p).map_err(|e| format!("{}: {e}", p.display()))?;
        let name = p
            .file_name()
            .map(|f| f.to_string_lossy().into_owned())
            .unwrap_or_else(|| p.display().to_string());
        v.push((name, text));
    }
    compare(&v, min_rows)
}

impl Comparison {
    pub fn groups(&self, t: Table) -> &[Group] {
        match t {
            Table::Trims => &self.trims,
            Table::Timing => &self.timing,
        }
    }

    /// Whether recording `f`'s cell in a group is thin (fewer rows than the minimum).
    pub fn thin(&self, g: &Group, f: usize) -> bool {
        g.cells.get(f).is_none_or(|c| c.rows < self.min_rows)
    }

    /// Value `k` of recording `f` in a group: its mean, only when the group is not thin and enough of its rows had it.
    pub fn value(&self, g: &Group, f: usize, k: usize) -> Option<f64> {
        let c = g.cells.get(f)?;
        if self.thin(g, f) || c.count(k) < self.min_rows {
            return None;
        }
        c.mean(k)
    }

    /// Recording `f` minus the first, for value `k`: only where neither group is thin.
    pub fn difference(&self, g: &Group, f: usize, k: usize) -> Option<f64> {
        Some(self.value(g, f, k)? - self.value(g, 0, k)?)
    }

    /// Recording `f`'s cell as text: "-9.2 / -5.3 (388 rows)", "26.3° (234 rows)", "thin (5 rows)", or "-" with none.
    pub fn cell_text(&self, t: Table, g: &Group, f: usize) -> String {
        let rows = g.cells.get(f).map_or(0, |c| c.rows);
        if rows == 0 {
            return "-".into();
        }
        let count = format!("{rows} row{}", if rows == 1 { "" } else { "s" });
        if self.thin(g, f) {
            return format!("thin ({count})");
        }
        let v = |k| self.value(g, f, k);
        let s = match t {
            Table::Trims => format!("{} / {}", signed(v(0)), signed(v(1))),
            Table::Timing => v(0).map_or("-".into(), |x| format!("{x:.1}°")),
        };
        format!("{s} ({count})")
    }

    /// Recording `f` minus the first as text ("+3.1 / +1.1", "-5.0°"), "-" where a group is thin or empty.
    pub fn difference_text(&self, t: Table, g: &Group, f: usize) -> String {
        let d = |k| self.difference(g, f, k);
        if (0..t.values()).all(|k| d(k).is_none()) {
            return "-".into();
        }
        match t {
            Table::Trims => format!("{} / {}", signed(d(0)), signed(d(1))),
            Table::Timing => format!("{}°", signed(d(0))),
        }
    }

    /// The conditions compared, in words; `warm` is the coolant temperature as shown ("176 °F (80 °C)").
    pub fn conditions(&self, warm: &str) -> String {
        format!(
            "Only rows at matching conditions: warm (coolant {warm} or more), closed loop (commanded lambda {:.2} to {:.2}) \
             and the engine running (over {RUNNING_RPM:.0} rpm). A group with fewer than {} rows is thin: its count is \
             shown, never a value. Each band holds its lower edge and not its upper. Differences are each recording minus \
             the first, only where neither group is thin.",
            CLOSED_LOOP.0, CLOSED_LOOP.1, self.min_rows
        )
    }

    /// Recording `f` in a line: its name, how many of its rows were compared, and what it lacks.
    pub fn recording_line(&self, f: usize) -> String {
        let r = &self.recordings[f];
        let mut s = format!(
            "{}. {}: {} of {} rows at those conditions",
            f + 1,
            r.name,
            r.kept,
            r.rows
        );
        if let Some(a) = &r.adapter {
            s.push_str(&format!(", recorded through {a}"));
        }
        if let Some(t) = &r.tag {
            s.push_str(&format!(", {t}"));
        }
        if !r.lacking.is_empty() {
            s.push_str(&format!(
                "; it has no {} column, so what needs it is left out",
                r.lacking.join(", ")
            ));
        }
        s
    }

    /// A table's heading lines: the first column's, each recording's (its number and name, then its adapter and its tag
    /// when any recording has one), and each difference's.
    pub fn heading(&self, t: Table) -> Vec<Vec<String>> {
        let n = self.recordings.len();
        let mut lines = vec![std::iter::once(t.by().to_string())
            .chain(
                self.recordings
                    .iter()
                    .enumerate()
                    .map(|(i, r)| format!("{}. {}", i + 1, r.name)),
            )
            .chain((2..=n).map(|i| format!("Difference {i} - 1")))
            .collect::<Vec<_>>()];
        let extra: [fn(&Recording) -> Option<&String>; 2] =
            [|r| r.adapter.as_ref(), |r| r.tag.as_ref()];
        for get in extra {
            if self.recordings.iter().any(|r| get(r).is_some()) {
                lines.push(
                    std::iter::once(String::new())
                        .chain(
                            self.recordings
                                .iter()
                                .map(|r| get(r).cloned().unwrap_or_default()),
                        )
                        .chain((2..=n).map(|_| String::new()))
                        .collect(),
                );
            }
        }
        lines
    }

    /// A table's rows as text: the group, each recording's cell, each difference.
    pub fn rows(&self, t: Table) -> Vec<Vec<String>> {
        let n = self.recordings.len();
        self.groups(t)
            .iter()
            .map(|g| {
                std::iter::once(g.label.clone())
                    .chain((0..n).map(|f| self.cell_text(t, g, f)))
                    .chain((1..n).map(|f| self.difference_text(t, g, f)))
                    .collect()
            })
            .collect()
    }
}

fn signed(v: Option<f64>) -> String {
    v.map_or("-".into(), |x| format!("{x:+.1}"))
}

/// Lines of cells in columns, each as wide as its widest cell.
fn columns(lines: &[Vec<String>]) -> String {
    let n = lines.iter().map(Vec::len).max().unwrap_or(0);
    let w: Vec<usize> = (0..n)
        .map(|c| {
            lines
                .iter()
                .filter_map(|l| l.get(c))
                .map(|s| s.chars().count())
                .max()
                .unwrap_or(0)
        })
        .collect();
    let mut out = String::new();
    for l in lines {
        let cells: Vec<String> = l
            .iter()
            .enumerate()
            .map(|(c, s)| format!("{s:<width$}", width = w[c]))
            .collect();
        out.push_str(cells.join("   ").trim_end());
        out.push('\n');
    }
    out
}

/// The comparison in words and two tables; `warm` is the coolant temperature as shown ("176 °F (80 °C)").
pub fn words(c: &Comparison, warm: &str) -> String {
    let mut o = format!(
        "Comparing {} recordings.\n{}\n\n",
        c.recordings.len(),
        c.conditions(warm)
    );
    for f in 0..c.recordings.len() {
        o.push_str(&c.recording_line(f));
        o.push('\n');
    }
    for t in [Table::Trims, Table::Timing] {
        o.push_str(&format!("\n{}:\n\n", t.title()));
        if c.groups(t).is_empty() {
            o.push_str("No rows at those conditions.\n");
            continue;
        }
        let mut lines = c.heading(t);
        lines.extend(c.rows(t));
        o.push_str(&columns(&lines));
    }
    o
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A made-up drive: `n` rows at each (rpm, load %, timing, air flow g/s), warm and in closed loop, with trims bank 1
    /// -4 + st1 and bank 2 -2 + 0; the columns in the order `head` gives.
    fn drive(head: &[&str], rows: &[(usize, f64, f64, f64, f64, f64)]) -> String {
        let mut s = format!(
            "time_s,{}\n",
            head.iter()
                .map(|h| format!("\"{h}\""))
                .collect::<Vec<_>>()
                .join(",")
        );
        let mut t = 0.0;
        for &(n, rpm, load, timing, maf, st1) in rows {
            for _ in 0..n {
                let v = |h: &str| -> String {
                    let h = h.to_lowercase();
                    let x = if h.starts_with("engine speed") {
                        rpm
                    } else if h.starts_with("engine load") {
                        load
                    } else if h.starts_with("timing") {
                        timing
                    } else if h.starts_with("mass air") {
                        maf
                    } else if h.starts_with("coolant") {
                        90.0
                    } else if h.starts_with("commanded lambda") {
                        1.0
                    } else if h.starts_with("short-term fuel trim, bank 1") {
                        st1
                    } else if h.starts_with("long-term fuel trim, bank 1") {
                        -4.0
                    } else if h.starts_with("long-term fuel trim, bank 2") {
                        -2.0
                    } else {
                        0.0
                    };
                    format!("{x:.2}")
                };
                s.push_str(&format!(
                    "{t:.1},{}\n",
                    head.iter().map(|h| v(h)).collect::<Vec<_>>().join(",")
                ));
                t += 0.5;
            }
        }
        s
    }

    const HEAD_A: &[&str] = &[
        "Engine speed [rpm]",
        "Vehicle speed [km/h]",
        "Engine load [%]",
        "Short-term fuel trim, bank 1 [%]",
        "Long-term fuel trim, bank 1 [%]",
        "Short-term fuel trim, bank 2 [%]",
        "Long-term fuel trim, bank 2 [%]",
        "Timing advance [° before TDC]",
        "Mass air flow [g/s]",
        "Commanded lambda [λ]",
        "Coolant temperature [°C]",
    ];
    /// The same values in another order, with one more column: matched by name.
    const HEAD_B: &[&str] = &[
        "Engine speed [rpm]",
        "Coolant temperature [°C]",
        "Mass air flow [g/s]",
        "Timing advance [° before TDC]",
        "Long-term fuel trim, bank 2 [%]",
        "Short-term fuel trim, bank 2 [%]",
        "Intake air temperature [°C]",
        "Long-term fuel trim, bank 1 [%]",
        "Short-term fuel trim, bank 1 [%]",
        "Commanded lambda [λ]",
        "Engine load [%]",
    ];

    #[test]
    fn a_band_holds_its_lower_edge_only() {
        let c = Cut::Edges(LOAD);
        assert_eq!(c.band(60.0), Some(2), "60.0 % is in 60-80");
        assert_eq!(c.band(79.99), Some(2));
        assert_eq!(
            c.band(80.0),
            Some(3),
            "80.0 % is in 80-100, not counted twice"
        );
        assert_eq!(c.band(100.0), Some(3));
        assert_eq!((c.band(19.9), c.band(f64::NAN)), (None, None));
        assert_eq!(Cut::Step(500.0).band(1499.0), Some(2));
        assert_eq!(Cut::Step(500.0).band(-1.0), None);
    }

    #[test]
    fn five_degrees_less_in_one_load_band_and_a_thin_group() {
        // A: 12 rows at 2200 rpm, 70 % load, 25°, 20 g/s; 5 rows at 3000 rpm, 50 %; 11 rows at 2200 rpm, 50 %
        let a = drive(
            HEAD_A,
            &[
                (12, 2200.0, 70.0, 25.0, 20.0, 1.0),
                (5, 3000.0, 50.0, 30.0, 40.0, 0.0),
                (11, 2200.0, 50.0, 28.0, 18.0, 1.0),
            ],
        );
        // B: the 60-80 % band 5 degrees less, the rest alike; a cold row and an open-loop row that would drag it if
        // counted; a row on the 60 % edge counted once, in 60-80
        let mut b = drive(
            HEAD_B,
            &[
                (11, 2200.0, 70.0, 20.0, 20.0, 3.0),
                (1, 2200.0, 60.0, 20.0, 20.0, 3.0),
                (5, 3000.0, 50.0, 30.0, 40.0, 0.0),
                (11, 2200.0, 50.0, 28.0, 18.0, 3.0),
            ],
        );
        b = format!("# adapter: ELM327 v1.4b\n{b}");
        let cold = drive(HEAD_B, &[(1, 2200.0, 70.0, 5.0, 20.0, 3.0)]);
        let cold_row = cold.lines().nth(1).unwrap().replace(",90.00,", ",60.00,");
        let rich = cold.lines().nth(1).unwrap().replace(",1.00,", ",0.85,");
        b.push_str(&format!("{cold_row}\n{rich}\n"));
        let c = compare(&[("a.csv".into(), a), ("b.csv".into(), b)], MIN_ROWS).unwrap();
        assert_eq!(
            (c.recordings[0].kept, c.recordings[1].kept),
            (28, 28),
            "the cold and the open-loop rows are left out"
        );
        assert_eq!(c.recordings[1].adapter.as_deref(), Some("ELM327 v1.4b"));
        let band = |label: &str| c.timing.iter().find(|g| g.label == label).unwrap();
        let hi = band("1800-2600 rpm, 60-80 %");
        assert_eq!((hi.cells[0].rows, hi.cells[1].rows), (12, 12));
        assert_eq!(c.value(hi, 0, 0), Some(25.0));
        assert_eq!(c.value(hi, 1, 0), Some(20.0));
        assert_eq!(c.difference(hi, 1, 0), Some(-5.0));
        let lo = band("1800-2600 rpm, 40-60 %");
        assert_eq!(c.difference(lo, 1, 0), Some(0.0), "the other band alike");
        let thin = band("2600-3600 rpm, 40-60 %");
        assert_eq!(thin.cells[0].rows, 5);
        assert!(c.thin(thin, 0) && c.thin(thin, 1));
        assert_eq!(
            (c.value(thin, 0, 0), c.difference(thin, 1, 0)),
            (None, None)
        );
        assert_eq!(c.cell_text(Table::Timing, thin, 0), "thin (5 rows)");
        assert_eq!(c.difference_text(Table::Timing, thin, 1), "-");
        assert_eq!(c.cell_text(Table::Timing, hi, 1), "20.0° (12 rows)");
        assert_eq!(c.difference_text(Table::Timing, hi, 1), "-5.0°");
        // trims by air flow: 15-30 g/s holds both 2200 rpm groups (23 rows), bank 1 -3 in A and -1 in B
        let air = c.trims.iter().find(|g| g.label == "15-30 g/s").unwrap();
        assert_eq!(air.cells[0].rows, 23);
        assert_eq!(c.value(air, 0, 0), Some(-3.0));
        assert_eq!(c.value(air, 0, 1), Some(-2.0));
        assert_eq!(c.difference_text(Table::Trims, air, 1), "+2.0 / +0.0");
        assert_eq!(c.cell_text(Table::Trims, air, 0), "-3.0 / -2.0 (23 rows)");
        let w = words(&c, "176 °F (80 °C)");
        assert!(w.contains("thin (5 rows)") && w.contains("-5.0°"), "{w}");
        assert!(w.contains("1. a.csv") && w.contains("2. b.csv") && w.contains("Difference 2 - 1"));
        assert!(
            w.contains("ELM327 v1.4b"),
            "the adapter under its file: {w}"
        );
        assert!(w.contains("coolant 176 °F (80 °C) or more"));
        // a lower minimum summarizes the 5-row group
        let c5 = Comparison { min_rows: 5, ..c };
        let thin = c5
            .timing
            .iter()
            .find(|g| g.label == "2600-3600 rpm, 40-60 %")
            .unwrap();
        assert_eq!(c5.difference(thin, 1, 0), Some(0.0));
    }

    #[test]
    fn what_a_recording_lacks_is_said() {
        let a = drive(HEAD_A, &[(12, 2200.0, 70.0, 25.0, 20.0, 1.0)]);
        let head: Vec<&str> = HEAD_A
            .iter()
            .copied()
            .filter(|h| !h.starts_with("Commanded"))
            .collect();
        let b = drive(&head, &[(12, 2200.0, 70.0, 25.0, 20.0, 1.0)]);
        let c = compare(
            &[("a.csv".into(), a.clone()), ("b.csv".into(), b)],
            MIN_ROWS,
        )
        .unwrap();
        assert_eq!(c.recordings[1].lacking, ["commanded lambda"]);
        assert_eq!(
            c.recordings[1].kept, 0,
            "closed loop cannot be told without it"
        );
        assert!(c.recording_line(1).contains("no commanded lambda column"));
        assert!(
            compare(&[("a.csv".into(), a)], MIN_ROWS).is_err(),
            "one is no comparison"
        );
    }
}
