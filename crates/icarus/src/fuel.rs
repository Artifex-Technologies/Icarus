//! `icarus fuel` (a vehicle's fill-ups, ICA-211), and the fuel side of `icarus logs`: `compare --by-fuel` (drives pooled by
//! their estimated blend) and `experiment` (what an A/B question still needs, ICA-212). Only files are read and written: the
//! vehicle's `fuel.toml` and `experiments.toml` in its folder, and the recordings.

use crate::parse_args;
use icarus_obd::drivelog::{self, compare, experiment};
use icarus_obd::fuel::{self, FillUp, FuelLog};
use icarus_obd::{garage, maintenance, vin};
use std::path::{Path, PathBuf};

const USAGE: &str = "usage: icarus fuel add [--vin VIN] --grade 87 --ethanol 10 --gallons G [--odometer MI] [--date \"YYYY-MM-DD HH:MM\"] [--note TEXT] [--tank GALLONS] [--before \"91 E0\"]
       icarus fuel list [--vin VIN]";

/// The vehicle asked for: `--vin`, checked, else the one vehicle with a folder.
fn vehicle(v: Option<&str>) -> Result<(String, PathBuf), String> {
    let v = v.map(|v| vin::decode(v).map(|i| i.vin)).transpose()?;
    fuel::which_vehicle(v.as_deref())
}

fn number(args: &crate::Args, name: &str, what: &str) -> Result<Option<f64>, String> {
    args.opt(name)
        .map(|v| {
            v.trim()
                .trim_start_matches(['E', 'e'])
                .parse::<f64>()
                .map_err(|_| format!("--{name} needs {what}"))
        })
        .transpose()
}

pub fn cmd_fuel(a: &[String]) -> Result<(), String> {
    let args = parse_args(
        a,
        &[
            "vin", "grade", "ethanol", "gallons", "odometer", "date", "note", "tank", "before",
        ],
    )?;
    match args.pos.as_slice() {
        [w] if w == "add" => add(&args),
        [w] if w == "list" => list(&args),
        _ => Err(USAGE.into()),
    }
}

fn add(args: &crate::Args) -> Result<(), String> {
    let (vin, dir) = vehicle(args.opt("vin"))?;
    let grade = number(args, "grade", "the octane the pump shows (AKI), like 87")?
        .ok_or("--grade is needed: the octane the pump shows (AKI), like 87 or 91")?;
    let gallons = number(args, "gallons", "the gallons added, like 14.2")?
        .ok_or("--gallons is needed: the gallons added")?;
    let ethanol = number(args, "ethanol", "the ethanol percent, like 10 for E10")?.ok_or(
        "--ethanol is needed: the ethanol percent the pump shows (10 for E10, 0 for ethanol-free)",
    )?;
    let odometer = args
        .opt("odometer")
        .map(|v| {
            v.replace([',', ' '], "")
                .parse::<u32>()
                .map_err(|_| "--odometer needs the miles, a whole number")
        })
        .transpose()?;
    let tank = number(args, "tank", "the tank's size in gallons, like 21.1")?;
    if tank.is_some_and(|t| !(t > 0.0 && t < 500.0)) {
        return Err("--tank needs the tank's size in gallons, like 21.1".into());
    }
    let described = garage::get(&vin);
    let fill = FillUp {
        date: args
            .opt("date")
            .map(str::to_string)
            .unwrap_or_else(|| fuel::now(false)),
        grade,
        ethanol,
        gallons,
        odometer,
        note: args.opt("note").unwrap_or("").to_string(),
        before: args.opt("before").map(str::to_string),
    };
    // the change, made on the log as it is on disk, with the folder locked (another program may add one meanwhile)
    let change = |log: &mut FuelLog| -> Result<(f64, &'static str), String> {
        if tank.is_some() {
            log.tank_gallons = tank;
        }
        let size = fuel::tank_size(log, described.as_ref()).ok_or(
            "how much does the tank hold? Give --tank GALLONS once (it is kept in the fuel log): the blend in the \
             tank is estimated from it",
        )?;
        log.add(fill.clone())?;
        Ok(size)
    };
    // checked first on a copy, so a refused fill-up makes no vehicle folder
    change(&mut FuelLog::load(&dir)?)?;
    let ((size, from), log, f) = FuelLog::update(&ensure(&vin)?, change)?;
    let tanks = fuel::tanks(&log, Some(size));
    println!(
        "Added to {} (the tank {} gal, {from}).",
        f.display(),
        num(size)
    );
    let added = tanks.iter().position(|t| Some(&t.fill) == log.fills.last());
    if let Some(i) = added {
        println!("{}", fill_line(&tanks[i]));
    }
    // a fill-up dated before the last one changes the estimates after it
    if added.is_some_and(|i| i + 1 < tanks.len()) {
        if let Some(last) = tanks.last() {
            println!("The tank now: {}", last.estimate_words());
        }
    }
    Ok(())
}

fn num(v: f64) -> String {
    let s = format!("{v:.1}");
    s.strip_suffix(".0").map(str::to_string).unwrap_or(s)
}

fn miles(m: u32) -> String {
    if m >= 1000 {
        format!("{},{:03}", m / 1000, m % 1000)
    } else {
        m.to_string()
    }
}

/// A fill-up in a line, and the tank after it on the next.
fn fill_line(t: &fuel::Tank) -> String {
    let f = &t.fill;
    let mut s = format!("  {}  {}  {} gal", f.date, f.label(), num(f.gallons));
    if let Some(m) = f.odometer {
        s.push_str(&format!("  odometer {} mi", miles(m)));
    }
    if !f.note.is_empty() {
        s.push_str(&format!("  {}", f.note));
    }
    format!("{s}\n      the tank: {}", t.estimate_words())
}

fn list(args: &crate::Args) -> Result<(), String> {
    let (vin, dir) = vehicle(args.opt("vin"))?;
    let log = FuelLog::load(&dir)?;
    let described = garage::get(&vin);
    let name = described
        .as_ref()
        .map(|d| d.name())
        .filter(|n| !n.is_empty())
        .unwrap_or(vin.clone());
    if log.fills.is_empty() {
        println!("No fill-ups logged for {name} yet: icarus fuel add --grade 87 --ethanol 10 --gallons 14.2");
        return Ok(());
    }
    let size = fuel::tank_size(&log, described.as_ref());
    println!(
        "Fill-ups of {name} ({}), {}:",
        dir.join(fuel::FILE).display(),
        match size {
            Some((t, from)) => format!("the tank {} gal ({from})", num(t)),
            None => "the tank's size not known (icarus fuel add --tank GALLONS)".into(),
        }
    );
    for t in fuel::tanks(&log, size.map(|s| s.0)) {
        println!("{}", fill_line(&t));
    }
    println!(
        "Each fill-up is taken to fill the tank; a recording names the tank it ran on (# fuel: at its top)."
    );
    Ok(())
}

/// A recording by its path, or its file name in the logs folders.
pub fn resolve(f: &str, dirs: &[PathBuf]) -> Result<PathBuf, String> {
    let p = PathBuf::from(f);
    if p.is_file() {
        return Ok(p);
    }
    drivelog::by_name(dirs, f).ok_or(format!(
        "{f}: no such file, and no recording by that name in the logs folders"
    ))
}

/// A recording as (name, text), its `# fuel:` line worked out from its vehicle's fuel log as it is now
/// (`fuel::with_line_at_read`; the line it was recorded with when that cannot be).
fn read(p: &Path) -> Result<(String, String), String> {
    let text = std::fs::read_to_string(p).map_err(|e| format!("{}: {e}", p.display()))?;
    let name = p
        .file_name()
        .map(|f| f.to_string_lossy().into_owned())
        .unwrap_or_else(|| p.display().to_string());
    Ok((
        name,
        fuel::with_line_at_read(&garage::library(), Some(p), text),
    ))
}

/// The recordings a scan reads: the vehicle's (`maintenance::recordings_for`: its own Logs and the shared folder, another
/// vehicle's left out by its `# vehicle:` line, one copy of each drive), and what that says; with no vehicle folders at all,
/// every recording in the logs folders, one copy of each drive.
fn scan_paths(vin: Option<&str>) -> Result<(Vec<PathBuf>, Option<String>), String> {
    let dirs = drivelog::folders();
    match vehicle(vin) {
        Ok((_, dir)) => Ok(vehicle_recordings(&dir, &dirs)),
        Err(_) if vin.is_none() && garage::vehicles().is_empty() => {
            Ok((maintenance::one_copy_each(&drivelog::list(&dirs)), None))
        }
        Err(e) => Err(e),
    }
}

fn vehicle_recordings(dir: &Path, dirs: &[PathBuf]) -> (Vec<PathBuf>, Option<String>) {
    let shared = dirs.first().cloned().unwrap_or_default();
    let r = maintenance::recordings_for(&garage::library(), dir, &shared);
    (r.paths, r.note)
}

/// Each recording that reads and holds rows, as (name, text), and the names of those that do not.
fn read_all(paths: &[PathBuf]) -> (Vec<(String, String)>, Vec<String>) {
    let (mut out, mut unread) = (vec![], vec![]);
    for p in paths {
        match read(p) {
            Ok((name, text)) if drivelog::series(&text).is_ok() => out.push((name, text)),
            Ok((name, _)) => unread.push(name),
            Err(_) => unread.push(p.file_name().map_or(p.display().to_string(), |f| {
                f.to_string_lossy().into_owned()
            })),
        }
    }
    (out, unread)
}

/// What scanning left out, in lines (none: nothing).
fn left_out(unread: &[String], note: Option<&str>) -> String {
    let mut o = String::new();
    if !unread.is_empty() {
        o.push_str(&format!(
            "\nSkipped, not readable or with no rows: {}\n",
            unread.join(", ")
        ));
    }
    if let Some(n) = note {
        o.push_str(&format!("\nNote: {n}.\n"));
    }
    o
}

/// `icarus logs compare --by-fuel [A B...]`: the recordings given (else the vehicle's, `--vin` or the one vehicle with a
/// folder) pooled by their estimated blend, the highest octane first.
pub fn compare_by_fuel(
    files: &[String],
    vin: Option<&str>,
    min_rows: usize,
    warm: &str,
) -> Result<(), String> {
    let dirs = drivelog::folders();
    let (recs, unread, note) = if files.is_empty() {
        let (paths, note) = scan_paths(vin)?;
        let (recs, unread) = read_all(&paths);
        (recs, unread, note)
    } else {
        let given = files
            .iter()
            .map(|f| resolve(f, &dirs).and_then(|p| read(&p)))
            .collect::<Result<_, _>>()?;
        (given, vec![], None)
    };
    let skipped = left_out(&unread, note.as_deref());
    let (pools, unknown) = compare::by_fuel(recs);
    if pools.len() < 2 {
        return Err(format!(
            "comparing by fuel needs drives on two blends or more; found {}{}",
            if pools.is_empty() {
                "none with a known blend".to_string()
            } else {
                pools
                    .iter()
                    .map(|p| p.0.clone())
                    .collect::<Vec<_>>()
                    .join(", ")
            },
            if unknown.is_empty() {
                String::new()
            } else {
                format!(
                    " ({} recording{} with no known blend: no fuel log then, or the first tank logged)",
                    unknown.len(),
                    if unknown.len() == 1 { "" } else { "s" }
                )
            }
        ) + skipped.trim_end());
    }
    let c = compare::compare_pooled(&pools, min_rows)?;
    print!("{}", compare::words(&c, warm));
    if !unknown.is_empty() {
        println!("\nLeft out, their blend not known: {}", unknown.join(", "));
    }
    print!("{skipped}");
    Ok(())
}

const EXPERIMENT_USAGE: &str = "usage: icarus logs experiment NAME A B [--min-seconds 60] [--vin VIN]   (A and B each a fuel tag such as \"91 E0\" or \"87 E10\", or recordings separated by commas)
       icarus logs experiment NAME [--min-seconds S] [--vin VIN]   (run a kept one again)
       icarus logs experiment [--vin VIN]   (the ones kept)";

/// `icarus logs experiment`: what an A/B question still needs, kept in the vehicle's folder to run again.
pub fn cmd_experiment(args: &crate::Args) -> Result<(), String> {
    let (vin, dir) = vehicle(args.opt("vin"))?;
    let min = args
        .opt("min-seconds")
        .map(|v| {
            v.parse::<f64>()
                .ok()
                .filter(|s| *s > 0.0)
                .ok_or("--min-seconds needs a number of seconds over 0")
        })
        .transpose()?;
    let kept = experiment::load(&dir)?;
    let (name, e) = match &args.pos[1..] {
        [] => {
            if kept.is_empty() {
                println!("No experiments kept for {vin} yet.\n{EXPERIMENT_USAGE}");
            }
            for (n, e) in &kept {
                let side = |s: &experiment::Side| match s {
                    experiment::Side::Tag(t) => t.clone(),
                    experiment::Side::Files(f) => f.join(", "),
                };
                println!(
                    "{n}: {} against {}, {} s a group",
                    side(&e.a),
                    side(&e.b),
                    num(e.min_seconds)
                );
            }
            return Ok(());
        }
        [name] => {
            let mut e = kept.get(name).cloned().ok_or(format!(
                "no experiment {name:?} kept for this vehicle\n{EXPERIMENT_USAGE}"
            ))?;
            if let Some(m) = min {
                e.min_seconds = m;
                experiment::save(&ensure(&vin)?, name, &e)?;
            }
            (name.clone(), e)
        }
        [name, a, b] => {
            let e = experiment::Experiment {
                a: experiment::Side::parse(a)?,
                b: experiment::Side::parse(b)?,
                min_seconds: min.unwrap_or(experiment::MIN_SECONDS),
            };
            let f = experiment::save(&ensure(&vin)?, name, &e)?;
            println!(
                "Kept in {} (icarus logs experiment {name:?} runs it again).\n",
                f.display()
            );
            (name.clone(), e)
        }
        _ => return Err(EXPERIMENT_USAGE.into()),
    };
    let dirs = drivelog::folders();
    // a tag side: this vehicle's recordings (another vehicle's left out by its # vehicle: line), read once
    type Scan = (Vec<(String, String)>, Vec<String>, Option<String>);
    let mut all: Option<Scan> = None;
    let mut side = |s: &experiment::Side| -> Result<Vec<(String, String)>, String> {
        match s {
            experiment::Side::Tag(t) => Ok(experiment::on_tag(
                t,
                &all.get_or_insert_with(|| {
                    let (paths, note) = vehicle_recordings(&dir, &dirs);
                    let (recs, unread) = read_all(&paths);
                    (recs, unread, note)
                })
                .0,
            )),
            experiment::Side::Files(f) => f
                .iter()
                .map(|f| resolve(f, &dirs).and_then(|p| read(&p)))
                .collect(),
        }
    };
    let (a, b) = (side(&e.a)?, side(&e.b)?);
    let p = experiment::progress(&name, &e, &a, &b)?;
    print!("{}", experiment::words(&p));
    if let Some((_, unread, note)) = &all {
        print!("{}", left_out(unread, note.as_deref()));
    }
    Ok(())
}

fn ensure(vin: &str) -> Result<PathBuf, String> {
    garage::ensure(vin).map_err(|e| format!("the vehicle's folder: {e}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_fill_up_is_listed_with_the_tank_after_it() {
        let mut log = FuelLog::default();
        for (date, grade, ethanol, gallons, odometer, note) in [
            ("2026-11-01 09:00", 91.0, 0.0, 21.1, Some(84_210), "full"),
            ("2026-11-10 17:30", 87.0, 10.0, 15.0, None, ""),
        ] {
            log.add(FillUp {
                date: date.into(),
                grade,
                ethanol,
                gallons,
                odometer,
                note: note.into(),
                before: None,
            })
            .unwrap();
        }
        let t = fuel::tanks(&log, Some(21.1));
        assert_eq!(
            fill_line(&t[0]),
            "  2026-11-01 09:00  91 E0  21.1 gal  odometer 84,210 mi  full\n      the tank: blend 91 E0 (estimated 91.0 AKI, 0.0 % ethanol)"
        );
        assert_eq!(
            fill_line(&t[1]),
            "  2026-11-10 17:30  87 E10  15 gal\n      the tank: blend 88 E7 (estimated 88.2 AKI, 7.1 % ethanol)"
        );
    }

    /// A recording with no rows, or one that does not read, is skipped and named.
    #[test]
    fn only_recordings_that_read_are_scanned() {
        let root = std::env::temp_dir().join(format!("icarus-scan-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).unwrap();
        let rows = "# adapter: test\ntime_s,\"Engine speed [rpm]\"\n0,800\n0.5,810\n";
        std::fs::write(root.join("drive-1.csv"), rows).unwrap();
        std::fs::write(
            root.join("drive-3.csv"),
            "# adapter: test\ntime_s,\"Engine speed [rpm]\"\n",
        )
        .unwrap();
        let paths = [
            root.join("drive-1.csv"),
            root.join("drive-3.csv"),
            root.join("gone.csv"),
        ];
        let (recs, unread) = read_all(&paths);
        assert_eq!(recs.len(), 1);
        assert_eq!(recs[0].0, "drive-1.csv");
        assert_eq!(unread, ["drive-3.csv", "gone.csv"]);
        let w = left_out(&unread, Some("2 recordings of other vehicles left out"));
        assert!(w.contains("Skipped, not readable or with no rows: drive-3.csv, gone.csv"));
        assert!(w.contains("Note: 2 recordings of other vehicles left out."));
        let _ = std::fs::remove_dir_all(&root);
    }

    /// The leftover stated on the first fill-up (--before): the blend named, and said as stated.
    #[test]
    fn the_leftover_can_be_stated_on_the_first_fill_up() {
        let mut log = FuelLog::default();
        log.add(FillUp {
            date: "2026-11-01".into(),
            grade: 87.0,
            ethanol: 10.0,
            gallons: 10.0,
            before: Some("91 e0".into()),
            ..Default::default()
        })
        .unwrap();
        assert_eq!(log.fills[0].before.as_deref(), Some("91 E0"));
        let t = &fuel::tanks(&log, Some(20.0))[0];
        assert_eq!(t.blend().as_deref(), Some("89 E5"));
        assert!(
            fill_line(t).contains("the 10 gal left before it taken as 91 E0, as stated"),
            "{}",
            fill_line(t)
        );
    }
}
