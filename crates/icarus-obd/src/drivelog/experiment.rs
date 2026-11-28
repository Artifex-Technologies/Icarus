//! An A/B question and what it still needs (ICA-212): two sides (a fuel tag such as "91 E0" against "87 E10", matched on
//! each recording's estimated blend, or lists of recordings), and for each engine speed and load group of the comparison
//! (`compare`'s bands and its conditions: warm, closed loop, running) how many seconds each side has. What driving is still
//! missing is said in plain words ("40 s more at 60-80 % load, 1,800-2,600 rpm, on 87 E10"), and "done" once every group
//! that needs it has the minimum on both sides.
//!
//! A group needs both sides once both sides have reached it (each has at least `compare::MIN_ROWS` rows there, so a
//! comparison would show both) or once one side has the minimum there (that driving happens, so the other side needs it
//! too). A group only one side touched briefly is listed as seen, not needed.
//!
//! An experiment's definition is kept in the vehicle's folder, `experiments.toml`, so it can be run again by its name:
//!
//! ```toml
//! ["91 vs 87"]
//! a = "91 E0"
//! b = "87 E10"
//! min_seconds = 60
//!
//! ["new plugs"]
//! a = ["drive-2026-11-01_080000.csv", "drive-2026-11-02_080000.csv"]
//! b = ["drive-2026-11-20_080000.csv"]
//! ```

use super::compare::{self, Time};
use crate::fuel;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::path::Path;

/// The experiments' file in a vehicle's folder.
pub const FILE: &str = "experiments.toml";
/// Seconds of qualifying driving each side needs in a group unless the experiment says otherwise.
pub const MIN_SECONDS: f64 = 60.0;

/// One side of an experiment: a fuel tag ("87 E10"; "87" for any ethanol) or recordings by file name.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Side {
    Tag(String),
    Files(Vec<String>),
}

impl Side {
    /// A side as typed: a fuel tag, else recordings' names or paths separated by commas.
    pub fn parse(s: &str) -> Result<Side, String> {
        if fuel::Want::parse(s).is_some() {
            return Ok(Side::Tag(
                s.split_whitespace().collect::<Vec<_>>().join(" "),
            ));
        }
        let files: Vec<String> = s
            .split(',')
            .map(str::trim)
            .filter(|f| !f.is_empty())
            .map(str::to_string)
            .collect();
        if files.is_empty() {
            return Err(format!(
                "{s:?}: a fuel tag (87 E10, 91 E0, 87) or recordings separated by commas"
            ));
        }
        Ok(Side::Files(files))
    }

    /// How the words name it: the tag, else "A" or "B".
    pub fn name(&self, letter: &str) -> String {
        match self {
            Side::Tag(t) => t.clone(),
            Side::Files(_) => letter.to_string(),
        }
    }
}

/// An experiment as kept: its two sides and the seconds each needs in a group.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Experiment {
    pub a: Side,
    pub b: Side,
    #[serde(default = "min_seconds")]
    pub min_seconds: f64,
}

fn min_seconds() -> f64 {
    MIN_SECONDS
}

/// The experiments kept in a vehicle's folder, by name (none: empty; a file that does not read is an error).
pub fn load(dir: &Path) -> Result<BTreeMap<String, Experiment>, String> {
    let f = dir.join(FILE);
    match std::fs::read_to_string(&f) {
        Ok(t) => toml::from_str(&t).map_err(|e| format!("{}: {e}", f.display())),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(BTreeMap::new()),
        Err(e) => Err(format!("{}: {e}", f.display())),
    }
}

/// Keeps an experiment in a vehicle's folder under its name (one of that name replaced).
pub fn save(dir: &Path, name: &str, e: &Experiment) -> Result<std::path::PathBuf, String> {
    let mut all = load(dir)?;
    all.insert(name.to_string(), e.clone());
    let body = toml::to_string(&all).map_err(|e| e.to_string())?;
    std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let f = dir.join(FILE);
    let tmp = dir.join(format!("{FILE}.tmp"));
    std::fs::write(
        &tmp,
        format!(
            "# Icarus: A/B experiments on this vehicle (icarus logs experiment NAME A B). Each side is a fuel tag\n\
             # (\"87 E10\", matched on a recording's estimated blend) or a list of recordings' file names; min_seconds is\n\
             # what each side needs in an engine speed and load group. Run one again: icarus logs experiment NAME\n{body}"
        ),
    )
    .and_then(|_| std::fs::rename(&tmp, &f))
    .map_err(|e| format!("{}: {e}", f.display()))?;
    Ok(f)
}

/// The recordings among `all` (name, text) whose estimated blend matches a tag.
pub fn on_tag(tag: &str, all: &[(String, String)]) -> Vec<(String, String)> {
    let Some(want) = fuel::Want::parse(tag) else {
        return vec![];
    };
    all.iter()
        .filter(|(_, t)| fuel::of_recording(t).is_some_and(|g| g.matches(&want)))
        .cloned()
        .collect()
}

/// One engine speed and load group: each side's seconds and rows there.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct GroupTime {
    pub key: Vec<usize>,
    pub seconds: [f64; 2],
    pub rows: [usize; 2],
}

/// A side short of the minimum in a group that needs it.
#[derive(Clone, Debug, PartialEq)]
pub struct Need {
    pub key: Vec<usize>,
    /// 0 for A, 1 for B.
    pub side: usize,
    pub have: f64,
    pub more: f64,
}

/// Where an experiment stands.
#[derive(Clone, Debug, PartialEq)]
pub struct Progress {
    pub name: String,
    /// How the words name each side.
    pub sides: [String; 2],
    /// Each side's recordings.
    pub files: [Vec<String>; 2],
    pub min_seconds: f64,
    pub groups: Vec<GroupTime>,
    /// What each side's recordings lack that the conditions need ("commanded lambda"), and which recordings.
    pub lacking: [Vec<String>; 2],
}

/// Where an experiment stands, from each side's recordings as (name, text).
pub fn progress(
    name: &str,
    e: &Experiment,
    a: &[(String, String)],
    b: &[(String, String)],
) -> Result<Progress, String> {
    let mut groups: BTreeMap<Vec<usize>, GroupTime> = BTreeMap::new();
    let mut lacking: [Vec<String>; 2] = [vec![], vec![]];
    for (side, recs) in [a, b].into_iter().enumerate() {
        for (n, text) in recs {
            let (time, lack): (Time, _) = compare::time_by_speed_load(n, text)?;
            if !lack.is_empty() {
                lacking[side].push(format!("{n} has no {} column", lack.join(", ")));
            }
            for (k, (secs, rows)) in time {
                let g = groups.entry(k.clone()).or_insert_with(|| GroupTime {
                    key: k,
                    ..Default::default()
                });
                g.seconds[side] += secs;
                g.rows[side] += rows;
            }
        }
    }
    Ok(Progress {
        name: name.to_string(),
        sides: [e.a.name("A"), e.b.name("B")],
        files: [
            a.iter().map(|r| r.0.clone()).collect(),
            b.iter().map(|r| r.0.clone()).collect(),
        ],
        min_seconds: e.min_seconds,
        groups: groups.into_values().collect(),
        lacking,
    })
}

impl Progress {
    fn reached(&self, g: &GroupTime, side: usize) -> bool {
        g.rows[side] >= compare::MIN_ROWS
    }

    /// Whether a group needs the minimum on both sides: both reached it, or one already has the minimum there.
    pub fn needed(&self, g: &GroupTime) -> bool {
        (self.reached(g, 0) && self.reached(g, 1))
            || g.seconds.iter().any(|s| *s >= self.min_seconds)
    }

    /// Each side short of the minimum in a group that needs it, in group order (A before B).
    pub fn needs(&self) -> Vec<Need> {
        let mut v = vec![];
        for g in self.groups.iter().filter(|g| self.needed(g)) {
            for side in 0..2 {
                if g.seconds[side] < self.min_seconds {
                    v.push(Need {
                        key: g.key.clone(),
                        side,
                        have: g.seconds[side],
                        more: (self.min_seconds - g.seconds[side]).ceil(),
                    });
                }
            }
        }
        v
    }

    /// Done: both sides have recordings, some group needs both, and every such group has the minimum on both sides.
    pub fn done(&self) -> bool {
        self.files.iter().all(|f| !f.is_empty())
            && self.groups.iter().any(|g| self.needed(g))
            && self.needs().is_empty()
    }
}

fn secs(v: f64) -> String {
    let n = v.round() as u64;
    if n >= 1000 {
        format!("{},{:03} s", n / 1000, n % 1000)
    } else {
        format!("{n} s")
    }
}

/// Where an experiment stands, in words.
pub fn words(p: &Progress) -> String {
    let count = |n: usize| format!("{n} drive{}", if n == 1 { "" } else { "s" });
    let mut o = format!(
        "Experiment {:?}: {} ({}) against {} ({}). Each engine speed and load group needs {} on each side at the \
         conditions compared (warm, closed loop, the engine running).\n",
        p.name,
        p.sides[0],
        count(p.files[0].len()),
        p.sides[1],
        count(p.files[1].len()),
        secs(p.min_seconds)
    );
    for side in 0..2 {
        if p.files[side].is_empty() {
            o.push_str(&format!(
                "No recordings on {} yet: drive on it (a recording's fuel comes from the fuel log, icarus fuel add).\n",
                p.sides[side]
            ));
        }
        for l in &p.lacking[side] {
            o.push_str(&format!(
                "On {}: {l}, so its rows cannot be judged.\n",
                p.sides[side]
            ));
        }
    }
    let needs = p.needs();
    if p.done() {
        o.push_str(&format!(
            "\nDone: every group both sides reached has {} on each side; icarus logs compare shows them side by side.\n",
            secs(p.min_seconds)
        ));
    } else if !needs.is_empty() {
        o.push_str("\nStill missing:\n");
        for n in &needs {
            let have = if n.have > 0.0 {
                format!("has {}", secs(n.have))
            } else {
                "none yet".into()
            };
            o.push_str(&format!(
                "  {} more at {}, on {} ({have})\n",
                secs(n.more),
                compare::speed_load_words(&n.key),
                p.sides[n.side]
            ));
        }
    } else if p.files.iter().all(|f| !f.is_empty()) {
        o.push_str(
            "\nNo group has enough on either side yet, and none was reached by both: drive more on each.\n",
        );
    }
    let enough: Vec<&GroupTime> = p
        .groups
        .iter()
        .filter(|g| p.needed(g) && g.seconds.iter().all(|s| *s >= p.min_seconds))
        .collect();
    if !enough.is_empty() {
        o.push_str("\nEnough on both sides:\n");
        for g in enough {
            o.push_str(&format!(
                "  {}: {} {}, {} {}\n",
                compare::speed_load_words(&g.key),
                p.sides[0],
                secs(g.seconds[0]),
                p.sides[1],
                secs(g.seconds[1])
            ));
        }
    }
    let brief: Vec<&GroupTime> = p.groups.iter().filter(|g| !p.needed(g)).collect();
    if !brief.is_empty() {
        o.push_str("\nSeen briefly, not needed:\n");
        for g in brief {
            o.push_str(&format!(
                "  {}: {} {}, {} {}\n",
                compare::speed_load_words(&g.key),
                p.sides[0],
                secs(g.seconds[0]),
                p.sides[1],
                secs(g.seconds[1])
            ));
        }
    }
    o
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A made-up drive, warm and in closed loop, one row every half second: `n` rows at each (rpm, load %).
    fn drive(fuel_line: &str, blocks: &[(usize, f64, f64)]) -> String {
        let mut s = format!(
            "# adapter: test\n{}time_s,\"Engine speed [rpm]\",\"Engine load [%]\",\"Coolant temperature [°C]\",\"Commanded lambda [λ]\"\n",
            if fuel_line.is_empty() {
                String::new()
            } else {
                format!("{fuel_line}\n")
            }
        );
        let mut t = 0.0;
        for &(n, rpm, load) in blocks {
            for _ in 0..n {
                s.push_str(&format!("{t:.1},{rpm:.0},{load:.1},90,1.00\n"));
                t += 0.5;
            }
        }
        s
    }

    /// A `# fuel:` line for a tank that is all of one fuel.
    fn fuel_line(grade: f64, ethanol: f64) -> String {
        let log = fuel::FuelLog {
            tank_gallons: Some(21.1),
            fills: vec![fuel::FillUp {
                date: "2026-11-01".into(),
                grade,
                ethanol,
                gallons: 21.1,
                odometer: None,
                note: String::new(),
                before: None,
            }],
        };
        fuel::tanks(&log, log.tank_gallons)[0].line()
    }

    #[test]
    fn what_each_side_lacks_is_listed_and_then_done() {
        let e = Experiment {
            a: Side::parse("91 E0").unwrap(),
            b: Side::parse("87  E10").unwrap(),
            min_seconds: 60.0,
        };
        assert_eq!(e.b, Side::Tag("87 E10".into()));
        let (on91, on87) = (fuel_line(91.0, 0.0), fuel_line(87.0, 10.0));
        let mut all = vec![
            // 91: 100 s at 2200 rpm and 70 %, 80 s at 2200 rpm and 50 %, a moment at 4000 rpm
            (
                "a1.csv".to_string(),
                drive(&on91, &[(200, 2200.0, 70.0), (4, 4000.0, 90.0)]),
            ),
            ("a2.csv".to_string(), drive(&on91, &[(160, 2200.0, 50.0)])),
            // 87: 40 s at 2200 rpm and 70 %, and 15 s at 3000 rpm and 30 % (where 91 never went)
            (
                "b1.csv".to_string(),
                drive(&on87, &[(80, 2200.0, 70.0), (30, 3000.0, 30.0)]),
            ),
            // no fuel line: on neither side
            ("other.csv".to_string(), drive("", &[(400, 2200.0, 70.0)])),
        ];
        let side = |s: &Side, all: &[(String, String)]| match s {
            Side::Tag(t) => on_tag(t, all),
            Side::Files(_) => unreachable!(),
        };
        let p = progress("91 vs 87", &e, &side(&e.a, &all), &side(&e.b, &all)).unwrap();
        assert_eq!(p.files, [vec!["a1.csv", "a2.csv"], vec!["b1.csv"]]);
        let g = p.groups.iter().find(|g| g.key == vec![1, 2]).unwrap();
        assert_eq!((g.seconds, g.rows), ([100.0, 40.0], [200, 80]));
        let needs: Vec<(Vec<usize>, usize, f64)> = p
            .needs()
            .into_iter()
            .map(|n| (n.key, n.side, n.more))
            .collect();
        // exactly what 87 lacks: 20 s at 60-80 %, 60 s at 40-60 %; not 4000 rpm (91 only a moment there), not 3000 rpm
        // and 30 % (87 alone, under the minimum)
        assert_eq!(needs, [(vec![1, 1], 1, 60.0), (vec![1, 2], 1, 20.0)]);
        assert!(!p.done());
        let w = words(&p);
        assert!(
            w.contains("20 s more at 60-80 % load, 1,800-2,600 rpm, on 87 E10 (has 40 s)"),
            "{w}"
        );
        assert!(w.contains("60 s more at 40-60 % load, 1,800-2,600 rpm, on 87 E10 (none yet)"));
        assert!(w.contains("Seen briefly, not needed:\n  20-40 % load, 2,600-3,600 rpm"));
        assert!(!w.contains("Done"));

        // another drive on 87 fills both: done, though 91 still never went to 3000 rpm and 30 %
        all.push((
            "b2.csv".to_string(),
            drive(&on87, &[(40, 2200.0, 70.0), (130, 2200.0, 50.0)]),
        ));
        let p = progress("91 vs 87", &e, &side(&e.a, &all), &side(&e.b, &all)).unwrap();
        assert_eq!(p.needs(), vec![]);
        assert!(p.done());
        let w = words(&p);
        assert!(
            w.contains("Done: every group both sides reached has 60 s on each side"),
            "{w}"
        );
        assert!(w.contains(
            "Enough on both sides:\n  40-60 % load, 1,800-2,600 rpm: 91 E0 80 s, 87 E10 65 s"
        ));

        // a side with no recordings is never done
        let p = progress("91 vs 87", &e, &side(&e.a, &all), &[]).unwrap();
        assert!(!p.done());
        assert!(words(&p).contains("No recordings on 87 E10 yet"));
    }

    #[test]
    fn sides_and_the_file_read_back() {
        assert_eq!(Side::parse("87").unwrap(), Side::Tag("87".into()));
        assert_eq!(
            Side::parse("a.csv, b.csv").unwrap(),
            Side::Files(vec!["a.csv".into(), "b.csv".into()])
        );
        assert!(Side::parse(" , ").is_err());
        let dir = std::env::temp_dir().join(format!("icarus-experiment-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        assert!(load(&dir).unwrap().is_empty());
        let e = Experiment {
            a: Side::Tag("91 E0".into()),
            b: Side::Files(vec!["drive-b.csv".into()]),
            min_seconds: 45.0,
        };
        save(&dir, "91 vs 87", &e).unwrap();
        save(
            &dir,
            "other",
            &Experiment {
                min_seconds: MIN_SECONDS,
                ..e.clone()
            },
        )
        .unwrap();
        let back = load(&dir).unwrap();
        assert_eq!(back["91 vs 87"], e);
        assert_eq!(back.len(), 2);
        // by hand, min_seconds left out: the default
        std::fs::write(dir.join(FILE), "[x]\na = \"91 E0\"\nb = \"87 E10\"\n").unwrap();
        assert_eq!(load(&dir).unwrap()["x"].min_seconds, MIN_SECONDS);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
