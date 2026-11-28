//! A vehicle's fuel log (ICA-211): each fill-up (grade, ethanol, gallons, odometer, a note), so a recording can say what
//! fuel it ran on. Kept in the vehicle's folder as `fuel.toml`, a small plain file I can read and edit by hand:
//!
//! ```toml
//! # Icarus: this vehicle's fill-ups ...
//! tank_gallons = 21.1        # how much the tank holds (a Tacoma 2016-2023 is 21.1 without it)
//!
//! [[fill]]
//! date = "2026-11-20 14:05"  # local time; a date alone counts as the start of that day
//! grade = 91                 # octane as the pump shows it (AKI, (R+M)/2)
//! ethanol = 0                # percent by volume (E10 is 10)
//! gallons = 14.2
//! odometer = 84210           # miles, optional
//! note = "after the E15 tank" # optional
//! before = "91 E0"           # optional, the first fill-up only: what was in the tank before it
//! ```
//!
//! Each fill-up is taken to fill the tank: what was left (the tank's size less the gallons added) is the blend before it,
//! and the gallons added mix in by volume, octane and ethanol alike (a linear estimate: ethanol blends a little above it).
//! Before the first logged fill-up the leftover is not known, and it is said, never guessed: the estimate is over the part
//! of the tank that is known, with the unknown share beside it, and no blend is named while that share is over
//! `UNKNOWN_MAX`, unless the first fill-up says what was in the tank (`before`), which the line then says it took as
//! stated. A key the file does not know (a typo: `ethonal`) is an error, never dropped. A recording names the tank it ran on (the last fill-up at or before its start) and the estimated blend in
//! a `# fuel:` line at its top (`line`), which `logs compare` shows beside its name and groups by (`--by-fuel`).

use crate::garage::Described;
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

/// The fuel log's file in a vehicle's folder.
pub const FILE: &str = "fuel.toml";
/// A 2016-2023 Toyota Tacoma's tank (gallons): the size used for one only when its description says so and the log gives
/// none.
pub const TACOMA_TANK_GAL: f64 = 21.1;
/// No blend is named for a tank whose unknown share (fuel from before the first logged fill-up) is over this.
pub const UNKNOWN_MAX: f64 = 0.10;
/// A recording matches a tag ("87 E10") when its estimated blend is within this much octane and ethanol (percentage
/// points) of it: a blend settles toward the fuel added over a few tanks, and 87 E9.6 is the 87 E10 test.
pub const TAG_AKI_TOLERANCE: f64 = 0.5;
pub const TAG_ETHANOL_TOLERANCE: f64 = 2.0;

/// One fill-up.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FillUp {
    /// Local time, "YYYY-MM-DD HH:MM" (or "YYYY-MM-DD": the start of that day).
    pub date: String,
    /// Octane as the pump shows it (AKI).
    pub grade: f64,
    /// Ethanol, percent by volume.
    #[serde(default)]
    pub ethanol: f64,
    pub gallons: f64,
    /// The odometer, miles.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub odometer: Option<u32>,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub note: String,
    /// What was in the tank before it ("91 E0"), as I know it: used on the first fill-up logged only, where the leftover
    /// is otherwise not known.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub before: Option<String>,
}

impl FillUp {
    /// The fuel added, as a tag: "87 E10", "91 E0", "88.5 E15".
    pub fn label(&self) -> String {
        format!("{} E{}", num(self.grade), num(self.ethanol))
    }
}

/// A vehicle's fuel log: the tank's size when it was given, and the fill-ups (in the file's order; `tanks` sorts them).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FuelLog {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tank_gallons: Option<f64>,
    #[serde(default, rename = "fill")]
    pub fills: Vec<FillUp>,
}

/// A blend as typed ("91 E0", "87 e10"): octane and ethanol, both needed; as the log writes it ("91 E0").
pub fn parse_blend(s: &str) -> Result<(f64, f64, String), String> {
    let bad = || format!("{s:?}: a blend like 91 E0 or 87 E10 (octane, then ethanol)");
    let w = Want::parse(s).ok_or_else(bad)?;
    let e = w.ethanol.ok_or_else(bad)?;
    if !(70.0..=120.0).contains(&w.aki) || !(0.0..=100.0).contains(&e) {
        return Err(bad());
    }
    Ok((w.aki, e, format!("{} E{}", num(w.aki), num(e))))
}

/// A number as short as it is exact to a tenth: 87, 88.5, 14.2.
fn num(v: f64) -> String {
    let r = (v * 10.0).round() / 10.0;
    if r.fract() == 0.0 {
        format!("{r:.0}")
    } else {
        format!("{r:.1}")
    }
}

/// A date as written in the log ("2026-11-20 14:05", "2026-11-20T14:05", "2026-11-20 14:05:30", "2026-11-20"), in the one
/// form that sorts by time: "2026-11-20 14:05" (seconds kept when given), or the date alone.
pub fn normal_date(s: &str) -> Result<String, String> {
    let bad = || format!("{s:?}: a date like 2026-11-20 or 2026-11-20 14:05");
    let s = s.trim();
    let (d, t) = match s.split_once([' ', 'T']) {
        Some((d, t)) => (d, Some(t.trim())),
        None => (s, None),
    };
    let p: Vec<&str> = d.split('-').collect();
    let [y, m, dd] = p.as_slice() else {
        return Err(bad());
    };
    let n = |x: &str, lo: u32, hi: u32| {
        x.parse::<u32>()
            .ok()
            .filter(|v| (lo..=hi).contains(v))
            .ok_or_else(bad)
    };
    let (y, m, dd) = (n(y, 1950, 2100)?, n(m, 1, 12)?, n(dd, 1, 31)?);
    let mut out = format!("{y:04}-{m:02}-{dd:02}");
    if let Some(t) = t {
        let q: Vec<&str> = t.split(':').collect();
        match q.as_slice() {
            [h, mi] => out.push_str(&format!(" {:02}:{:02}", n(h, 0, 23)?, n(mi, 0, 59)?)),
            [h, mi, sec] => out.push_str(&format!(
                " {:02}:{:02}:{:02}",
                n(h, 0, 23)?,
                n(mi, 0, 59)?,
                n(sec, 0, 59)?
            )),
            _ => return Err(bad()),
        }
    }
    Ok(out)
}

/// Now, local time, as the log writes it ("2026-11-28 09:41"), or with seconds for a recording's start.
pub fn now(seconds: bool) -> String {
    unsafe {
        let t = libc::time(std::ptr::null_mut());
        let mut tm: libc::tm = std::mem::zeroed();
        libc::localtime_r(&t, &mut tm);
        let mut s = format!(
            "{:04}-{:02}-{:02} {:02}:{:02}",
            tm.tm_year + 1900,
            tm.tm_mon + 1,
            tm.tm_mday,
            tm.tm_hour,
            tm.tm_min
        );
        if seconds {
            s.push_str(&format!(":{:02}", tm.tm_sec));
        }
        s
    }
}

impl FuelLog {
    /// The log in a vehicle's folder (none there: empty). A file that does not read is an error, said, not taken as empty:
    /// a fill-up added over it would lose the others.
    pub fn load(dir: &Path) -> Result<FuelLog, String> {
        let f = dir.join(FILE);
        let text = match std::fs::read_to_string(&f) {
            Ok(t) => t,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(FuelLog::default()),
            Err(e) => return Err(format!("{}: {e}", f.display())),
        };
        FuelLog::parse(&text).map_err(|e| format!("{}: {e}", f.display()))
    }

    /// The log from its text, every date and number checked.
    pub fn parse(text: &str) -> Result<FuelLog, String> {
        let mut log: FuelLog = toml::from_str(text).map_err(|e| e.to_string())?;
        if let Some(t) = log.tank_gallons {
            if !(t > 0.0 && t < 500.0) {
                return Err(format!("tank_gallons = {t}: the tank's size in gallons"));
            }
        }
        for f in &mut log.fills {
            f.date = normal_date(&f.date)?;
            check(f)?;
        }
        Ok(log)
    }

    /// The file's text: a comment saying what it is, the tank's size, then each fill-up in date order.
    pub fn text(&self) -> String {
        let mut o = String::from(
            "# Icarus: this vehicle's fill-ups (icarus fuel add, or the Garage page's Fuel log). Edit by hand if you like:\n\
             # date is local time (\"YYYY-MM-DD HH:MM\", or a date alone for the start of that day), grade the octane the\n\
             # pump shows (AKI), ethanol percent by volume, gallons added; odometer (miles) and note are optional.\n\
             # Each fill-up is taken to fill the tank.\n",
        );
        if let Some(t) = self.tank_gallons {
            o.push_str(&format!("tank_gallons = {}\n", num(t)));
        }
        for f in self.sorted() {
            o.push_str(&format!(
                "\n[[fill]]\ndate = {}\ngrade = {}\nethanol = {}\ngallons = {}\n",
                quote(&f.date),
                num(f.grade),
                num(f.ethanol),
                f.gallons
            ));
            if let Some(m) = f.odometer {
                o.push_str(&format!("odometer = {m}\n"));
            }
            if !f.note.is_empty() {
                o.push_str(&format!("note = {}\n", quote(&f.note)));
            }
            if let Some(b) = &f.before {
                o.push_str(&format!("before = {}\n", quote(b)));
            }
        }
        o
    }

    /// Writes the log into a vehicle's folder: a temporary file of this process's own, synced, then renamed over the old
    /// one, so a crash or another program writing at the same moment never leaves half a file.
    pub fn save(&self, dir: &Path) -> Result<PathBuf, String> {
        std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
        let f = dir.join(FILE);
        crate::vehicles::write_replacing(&f, &self.text())?;
        Ok(f)
    }

    /// Reads the log in a vehicle's folder, changes it and writes it back, with the folder locked (flock) throughout, so a
    /// fill-up added meanwhile by the window, `icarus fuel add` or a hand edit is never lost. A log that does not read is
    /// refused, not replaced. The change's result, the log as saved and its file.
    pub fn update<T>(
        dir: &Path,
        change: impl FnOnce(&mut FuelLog) -> Result<T, String>,
    ) -> Result<(T, FuelLog, PathBuf), String> {
        use std::os::unix::io::AsRawFd;
        std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
        let held = std::fs::File::open(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
        if unsafe { libc::flock(held.as_raw_fd(), libc::LOCK_EX) } != 0 {
            return Err(format!(
                "{}: {}",
                dir.display(),
                std::io::Error::last_os_error()
            ));
        }
        let mut log = FuelLog::load(dir)?;
        let out = change(&mut log)?;
        let path = log.save(dir)?;
        drop(held);
        Ok((out, log, path))
    }

    /// Whether a fill-up is logged before `date` (as `normal_date` writes it).
    pub fn has_fill_before(&self, date: &str) -> bool {
        self.fills.iter().any(|f| f.date.as_str() < date)
    }

    /// Adds a fill-up (its date made normal and its numbers checked). What was in the tank before it (`before`) is taken
    /// only for the first fill-up: with an earlier one logged, the leftover is worked out from that.
    pub fn add(&mut self, mut f: FillUp) -> Result<(), String> {
        f.date = normal_date(&f.date)?;
        f.note = f.note.split_whitespace().collect::<Vec<_>>().join(" ");
        if let Some(b) = f.before.take().filter(|b| !b.trim().is_empty()) {
            if self.has_fill_before(&f.date) {
                return Err(format!(
                    "a fill-up is logged before {}: what was in the tank is worked out from it, so leave what was \
                     before out",
                    f.date
                ));
            }
            f.before = Some(parse_blend(&b)?.2);
        }
        check(&f)?;
        self.fills.push(f);
        Ok(())
    }

    /// The fill-ups by date (two on one date in the file's order).
    pub fn sorted(&self) -> Vec<&FillUp> {
        let mut v: Vec<&FillUp> = self.fills.iter().collect();
        v.sort_by(|a, b| a.date.cmp(&b.date));
        v
    }
}

fn check(f: &FillUp) -> Result<(), String> {
    if !(f.grade >= 70.0 && f.grade <= 120.0) {
        return Err(format!(
            "grade {}: the octane the pump shows (AKI), like 87 or 91",
            f.grade
        ));
    }
    if !(0.0..=100.0).contains(&f.ethanol) {
        return Err(format!(
            "ethanol {}: percent by volume, 0 to 100 (E10 is 10)",
            f.ethanol
        ));
    }
    if !(f.gallons > 0.0 && f.gallons < 500.0) {
        return Err(format!("gallons {}: the gallons added", f.gallons));
    }
    if let Some(b) = &f.before {
        parse_blend(b).map_err(|e| format!("before: {e}"))?;
    }
    Ok(())
}

fn quote(s: &str) -> String {
    toml::Value::String(s.to_string()).to_string()
}

/// How much the tank holds, and where that came from: the log's `tank_gallons`, else a 2016-2023 Tacoma's 21.1 when the
/// vehicle's description says it is one. None: not known (`icarus fuel add --tank GALLONS` once).
pub fn tank_size(log: &FuelLog, described: Option<&Described>) -> Option<(f64, &'static str)> {
    if let Some(t) = log.tank_gallons {
        return Some((t, "as set"));
    }
    let d = described?;
    (d.model.eq_ignore_ascii_case("tacoma") && d.year.is_some_and(|y| (2016..=2023).contains(&y)))
        .then_some((TACOMA_TANK_GAL, "a 2016-2023 Tacoma's"))
}

/// The tank after one fill-up: the fuel added and the blend estimated in the tank.
#[derive(Clone, Debug, PartialEq)]
pub struct Tank {
    pub fill: FillUp,
    /// The share of the tank (0 to 1) from before the first logged fill-up, whose blend is not known; None when the tank's
    /// size is not known (then nothing is estimated).
    pub unknown: Option<f64>,
    /// Octane and ethanol estimated over the known part of the tank.
    pub aki: Option<f64>,
    pub ethanol: Option<f64>,
    /// More gallons added than the tank holds (the tank's size is likely wrong): counted as a full tank of it.
    pub over_full: bool,
    /// The leftover taken as the blend the fill-up says was in the tank (`before`): (gallons, blend).
    pub before_used: Option<(f64, String)>,
    /// The fill-up says what was before it, but an earlier fill-up is logged: not used.
    pub before_ignored: bool,
}

impl Tank {
    /// The estimated blend named ("88 E8": octane and ethanol to the nearest whole), when the unknown share is at most
    /// `UNKNOWN_MAX`.
    pub fn blend(&self) -> Option<String> {
        let (a, e) = self.estimate()?;
        Some(blend_label(a, e))
    }

    /// The estimated octane and ethanol, when the unknown share is at most `UNKNOWN_MAX`.
    pub fn estimate(&self) -> Option<(f64, f64)> {
        (self.unknown? <= UNKNOWN_MAX).then_some(())?;
        Some((self.aki?, self.ethanol?))
    }

    /// The estimate in words: "about 88.0 AKI and 7.6 % ethanol (88 E8)", with the unknown share when there is one.
    pub fn estimate_words(&self) -> String {
        let Some(u) = self.unknown else {
            return "blend unknown (the tank's size is not known: icarus fuel add --tank GALLONS, or the Garage page)"
                .into();
        };
        let known = match (self.aki, self.ethanol) {
            (Some(a), Some(e)) => format!("{a:.1} AKI, {e:.1} % ethanol"),
            _ => String::new(),
        };
        let pct = (u * 100.0).round();
        let mut over = String::new();
        if let Some((g, b)) = &self.before_used {
            over.push_str(&format!(
                "; the {} gal left before it taken as {b}, as stated",
                num(*g)
            ));
        }
        if self.before_ignored {
            over.push_str(
                "; what it says was in the tank is not used: an earlier fill-up is logged",
            );
        }
        if self.over_full {
            over.push_str(
                "; more gallons than the tank holds were added, so its size may be wrong",
            );
        }
        match self.blend() {
            Some(b) if u < 0.005 => format!("blend {b} (estimated {known}{over})"),
            Some(b) => format!(
                "blend {b} (estimated {known} over the {:.0} % of the tank known; {pct:.0} % is from before the first logged fill-up{over})",
                100.0 - pct
            ),
            None if known.is_empty() => {
                "blend unknown (all of the tank is from before the first logged fill-up)".into()
            }
            None => format!(
                "blend unknown ({pct:.0} % of the tank is from before the first logged fill-up; the rest is {known}{over})"
            ),
        }
    }

    /// The recording's `# fuel:` line: the tank it ran on (the fuel added, when, how much) and the estimated blend.
    pub fn line(&self) -> String {
        let mut filled = format!("{}, {} gal", self.fill.date, num(self.fill.gallons));
        if let Some(m) = self.fill.odometer {
            filled.push_str(&format!(", odometer {m} mi"));
        }
        format!(
            "{PREFIX}tank {}; filled {filled}; {}",
            self.fill.label(),
            self.estimate_words()
        )
    }
}

/// "88 E8": octane and ethanol, each to the nearest whole.
pub fn blend_label(aki: f64, ethanol: f64) -> String {
    format!("{:.0} E{:.0}", aki.round(), ethanol.round())
}

const PREFIX: &str = "# fuel: ";

/// The tank after each fill-up, in date order: the mixing model (each fill-up fills the tank).
pub fn tanks(log: &FuelLog, tank: Option<f64>) -> Vec<Tank> {
    // the share from before the first fill-up (unknown), and the known part's octane and ethanol
    let mut unknown = 1.0_f64;
    let (mut aki, mut eth) = (0.0_f64, 0.0_f64);
    log.sorted()
        .into_iter()
        .map(|f| {
            let Some(t) = tank else {
                return Tank {
                    fill: f.clone(),
                    unknown: None,
                    aki: None,
                    ethanol: None,
                    over_full: false,
                    before_used: None,
                    before_ignored: false,
                };
            };
            let added = f.gallons.min(t);
            let left = t - added;
            // the first fill-up may say what was in the tank: then nothing is unknown
            let first = unknown >= 1.0;
            let stated = f
                .before
                .as_deref()
                .filter(|_| first)
                .and_then(|b| parse_blend(b).ok());
            let mut before_used = None;
            if let Some((a, e, name)) = stated {
                unknown = 0.0;
                aki = a;
                eth = e;
                before_used = Some((left, name));
            }
            let known_left = (1.0 - unknown) * left;
            let known = known_left + added;
            aki = (known_left * aki + added * f.grade) / known;
            eth = (known_left * eth + added * f.ethanol) / known;
            unknown = unknown * left / t;
            Tank {
                fill: f.clone(),
                unknown: Some(unknown),
                aki: Some(aki),
                ethanol: Some(eth),
                over_full: f.gallons > t + 0.05,
                before_used,
                before_ignored: f.before.is_some() && !first,
            }
        })
        .collect()
}

/// The tank a drive that started at `at` (as `now(true)` writes it) ran on: the last fill-up at or before it.
pub fn tank_at(tanks: &[Tank], at: &str) -> Option<Tank> {
    tanks
        .iter()
        .rev()
        .find(|t| t.fill.date.as_str() <= at)
        .cloned()
}

/// The `# fuel:` line for a drive starting at `at`, from the log in a vehicle's folder: None when the folder has no log
/// (nothing logged, nothing said); a log with no fill-up before the drive says so.
pub fn line_in(dir: &Path, described: Option<&Described>, at: &str) -> Option<String> {
    if !dir.join(FILE).is_file() {
        return None;
    }
    // the error said without the file's path: the folder may be named by the VIN, which stays out of the recording
    let log = match std::fs::read_to_string(dir.join(FILE))
        .map_err(|e| e.to_string())
        .and_then(|t| FuelLog::parse(&t))
    {
        Ok(l) => l,
        Err(e) => {
            return Some(format!(
                "{PREFIX}not known ({FILE} does not read: {})",
                e.lines().next().unwrap_or("").trim()
            ))
        }
    };
    let tank = tank_size(&log, described).map(|t| t.0);
    Some(match tank_on(&tanks(&log, tank), at) {
        On::Tank(t) => t.line(),
        On::SameDay(day) => format!(
            "{PREFIX}not known (a fill-up on {day} is logged with no time, so a drive that day may have run on the tank \
             before it or after it)"
        ),
        On::Nothing => format!("{PREFIX}not known (no fill-up logged before this drive)"),
    })
}

/// The tank a drive ran on, as `tank_on` tells it.
#[derive(Clone, Debug, PartialEq)]
pub enum On {
    Tank(Tank),
    /// The last fill-up before it is logged with a date only, the drive's own day: it may have been before or after it.
    SameDay(String),
    Nothing,
}

/// The tank a drive that began at `at` ran on: the last fill-up at or before it; but when that fill-up has a date and no
/// time, and the drive began that same day, which tank it ran on is not known (said, not guessed).
pub fn tank_on(tanks: &[Tank], at: &str) -> On {
    match tank_at(tanks, at) {
        Some(t) if t.fill.date.len() == 10 && at.len() > 10 && at.starts_with(&t.fill.date) => {
            On::SameDay(t.fill.date.clone())
        }
        Some(t) => On::Tank(t),
        None => On::Nothing,
    }
}

/// The `# fuel:` line a recording should carry, worked out now from its vehicle's fuel log at the time it began (ICA-211):
/// the vehicle's folder in `lib` from its `# vehicle:` line, or the vehicle folder whose Logs it sits in (`path`); when it
/// began from its `# began:` line, else its name or file (`maintenance::recording_start`). None when its vehicle or its
/// fuel log is not found: the line it was recorded with (if any) stands.
pub fn line_at_read(lib: &Path, path: Option<&Path>, text: &str) -> Option<String> {
    let hashes: Vec<String> = text
        .lines()
        .take_while(|l| l.starts_with('#'))
        .map(str::to_string)
        .collect();
    let dir = match crate::maintenance::vehicle_of(&hashes) {
        Some(label) if !label.is_empty() && !label.contains('/') && !label.contains("..") => {
            lib.join(label)
        }
        _ => {
            // a vehicle's own Logs folder: <lib>/<vehicle>/Logs/drive.csv
            let logs = path?.parent()?;
            let dir = logs.parent()?;
            (logs.file_name()? == "Logs" && dir.parent()? == lib).then(|| dir.to_path_buf())?
        }
    };
    if !dir.join(FILE).is_file() {
        return None;
    }
    let began = crate::maintenance::began_of(&hashes)
        .or_else(|| path.and_then(crate::maintenance::recording_start))?;
    let described = std::fs::read_to_string(dir.join("vehicle.toml"))
        .ok()
        .and_then(|t| toml::from_str::<toml::Table>(&t).ok())
        .and_then(|v| v.get("name").and_then(|n| n.as_str()).map(Described::parse));
    line_in(&dir, described.as_ref(), &began.text())
}

/// A recording's text with its `# fuel:` line as worked out now (`line_at_read`): the line replaced, or added after its
/// other `#` lines; unchanged when it cannot be worked out.
pub fn with_line_at_read(lib: &Path, path: Option<&Path>, text: String) -> String {
    let Some(line) = line_at_read(lib, path, &text) else {
        return text;
    };
    let mut out = String::with_capacity(text.len() + line.len() + 1);
    let mut head = true;
    let mut put = false;
    for l in text.split_inclusive('\n') {
        if head && !l.starts_with('#') {
            head = false;
            if !put {
                out.push_str(&line);
                out.push('\n');
                put = true;
            }
        }
        if head && l.starts_with(PREFIX) {
            if !put {
                out.push_str(&line);
                out.push('\n');
                put = true;
            }
            continue;
        }
        out.push_str(l);
    }
    out
}

/// The `# fuel:` line for a drive starting now on the vehicle with this VIN (its folder's log); None for the simulated
/// vehicle, an unknown VIN or a vehicle with no fuel log.
pub fn line_for(vin: Option<&str>) -> Option<String> {
    let vin = vin?.trim().to_uppercase();
    if vin == crate::sim::VIN || vin.is_empty() {
        return None;
    }
    let dir = folder(&vin);
    line_in(&dir, crate::garage::get(&vin).as_ref(), &now(true))
}

/// Gives a recording's `# fuel:` line when each of its part files begins (`drive::Plan::fuel`).
pub type FuelSource = Box<dyn Fn() -> Option<String> + Send>;

/// The `# fuel:` line of the vehicle with this VIN, worked out again each time a part file begins (`line_for`): a fill-up
/// logged during a stop for fuel names the tank the next part runs on.
pub fn source(vin: Option<String>) -> FuelSource {
    Box::new(move || line_for(vin.as_deref()))
}

/// A vehicle's folder by its VIN (made or not).
pub fn folder(vin: &str) -> PathBuf {
    let vin = vin.trim().to_uppercase();
    let names = crate::garage::load();
    crate::garage::folder_in(
        &crate::garage::library(),
        &vin,
        names.get(&vin).map(String::as_str),
    )
}

/// The vehicle a fuel or experiment command is about: the VIN given, else the one vehicle with a folder. (VIN, folder).
pub fn which_vehicle(vin: Option<&str>) -> Result<(String, PathBuf), String> {
    if let Some(v) = vin {
        let v = v.trim().to_uppercase();
        if v == crate::sim::VIN {
            return Err("the simulated vehicle has no folder, so no fuel log".into());
        }
        return Ok((v.clone(), folder(&v)));
    }
    let all = crate::garage::vehicles();
    match all.as_slice() {
        [(v, _, d)] => Ok((v.clone(), d.clone())),
        [] => {
            Err("no vehicle folders yet (each vehicle connected gets one): give --vin VIN".into())
        }
        _ => Err(format!(
            "which vehicle? --vin VIN, one of: {}",
            all.iter()
                .map(|(v, n, _)| match n {
                    Some(n) => format!("{v} ({n})"),
                    None => v.clone(),
                })
                .collect::<Vec<_>>()
                .join(", ")
        )),
    }
}

/// What a recording's `# fuel:` line says: the tank's fuel ("87 E10") and the estimated blend, when one was named.
#[derive(Clone, Debug, PartialEq)]
pub struct Tag {
    pub tank: String,
    /// "88 E8", with the octane and ethanol it was rounded from; None when the blend is not known.
    pub blend: Option<String>,
    pub aki: Option<f64>,
    pub ethanol: Option<f64>,
}

impl Tag {
    /// As a comparison shows it beside the drive's name: "87 E10 tank, blend 88 E8", "87 E10 tank, blend unknown".
    pub fn short(&self) -> String {
        format!(
            "{} tank, blend {}",
            self.tank,
            self.blend.as_deref().unwrap_or("unknown")
        )
    }

    /// Whether this drive's blend is within the tolerance of a tag ("87 E10", or "87" for any ethanol).
    pub fn matches(&self, want: &Want) -> bool {
        let (Some(a), Some(e)) = (self.aki, self.ethanol) else {
            return false;
        };
        (a - want.aki).abs() <= TAG_AKI_TOLERANCE
            && want
                .ethanol
                .is_none_or(|w| (e - w).abs() <= TAG_ETHANOL_TOLERANCE)
    }
}

/// A fuel asked for by its tag: "87 E10" (octane, ethanol), "87" (any ethanol).
#[derive(Clone, Debug, PartialEq)]
pub struct Want {
    pub aki: f64,
    pub ethanol: Option<f64>,
}

impl Want {
    pub fn parse(s: &str) -> Option<Want> {
        let mut w = s.split_whitespace();
        let aki = w.next()?.parse::<f64>().ok().filter(|a| *a > 50.0)?;
        let ethanol = match w.next() {
            None => None,
            Some(e) => Some(
                e.strip_prefix(['E', 'e'])?
                    .trim_end_matches('%')
                    .parse::<f64>()
                    .ok()?,
            ),
        };
        w.next().is_none().then_some(Want { aki, ethanol })
    }
}

/// The fuel a recording names in its `# fuel:` line, when it names a tank.
pub fn of_recording(text: &str) -> Option<Tag> {
    let line = text
        .lines()
        .take_while(|l| l.starts_with('#'))
        .find_map(|l| l.strip_prefix(PREFIX))?;
    let mut tag = Tag {
        tank: String::new(),
        blend: None,
        aki: None,
        ethanol: None,
    };
    for part in line.split("; ") {
        if let Some(t) = part.strip_prefix("tank ") {
            tag.tank = t.trim().to_string();
        } else if let Some(b) = part.strip_prefix("blend ") {
            let (name, rest) = b.split_once(" (").unwrap_or((b, ""));
            if name != "unknown" {
                tag.blend = Some(name.trim().to_string());
                let est = rest.strip_prefix("estimated ").unwrap_or("");
                let mut nums = est.split(|c: char| !(c.is_ascii_digit() || c == '.'));
                let mut next = || {
                    nums.by_ref()
                        .find(|s| !s.is_empty())
                        .and_then(|s| s.parse::<f64>().ok())
                };
                tag.aki = next();
                tag.ethanol = next();
            }
        }
    }
    (!tag.tank.is_empty()).then_some(tag)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fill(date: &str, grade: f64, ethanol: f64, gallons: f64) -> FillUp {
        FillUp {
            date: date.into(),
            grade,
            ethanol,
            gallons,
            odometer: None,
            note: String::new(),
            before: None,
        }
    }

    #[test]
    fn the_log_reads_back_as_written_and_by_hand() {
        let by_hand = r#"
tank_gallons = 21.1
[[fill]]
date = "2026-11-20T14:05"
grade = 91
ethanol = 0
gallons = 14.2
odometer = 84210
note = "the usual"

[[fill]]
date = "2026-11-01"
grade = 87
gallons = 12.5
"#;
        let log = FuelLog::parse(by_hand).unwrap();
        assert_eq!(log.tank_gallons, Some(21.1));
        assert_eq!(log.fills[0].date, "2026-11-20 14:05", "made normal");
        assert_eq!(
            log.fills[1].ethanol, 0.0,
            "ethanol left out is none: written so"
        );
        let sorted: Vec<&str> = log.sorted().iter().map(|f| f.date.as_str()).collect();
        assert_eq!(sorted, ["2026-11-01", "2026-11-20 14:05"]);
        // written back: whole numbers as whole numbers, by date, and it reads the same
        let text = log.text();
        assert!(text.starts_with("# Icarus: this vehicle's fill-ups"));
        assert!(text.contains("grade = 91\n") && text.contains("note = \"the usual\""));
        assert!(text.find("2026-11-01").unwrap() < text.find("2026-11-20").unwrap());
        let back = FuelLog::parse(&text).unwrap();
        assert_eq!(back.sorted(), log.sorted());

        // through the folder
        let dir = std::env::temp_dir().join(format!("icarus-fuel-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        assert_eq!(FuelLog::load(&dir).unwrap(), FuelLog::default());
        let mut l = FuelLog::default();
        l.add(fill("2026-11-02 7:05", 87.0, 10.0, 15.0)).unwrap();
        l.save(&dir).unwrap();
        assert_eq!(
            FuelLog::load(&dir).unwrap().fills[0].date,
            "2026-11-02 07:05"
        );
        std::fs::write(dir.join(FILE), "[[fill]]\ngrade = \"high\"\n").unwrap();
        assert!(
            FuelLog::load(&dir).is_err(),
            "a log that does not read is said"
        );
        let _ = std::fs::remove_dir_all(&dir);

        // what is refused
        for bad in [
            fill("2026-13-01", 87.0, 10.0, 10.0),
            fill("yesterday", 87.0, 10.0, 10.0),
            fill("2026-11-01 25:00", 87.0, 10.0, 10.0),
            fill("2026-11-01", 8.7, 10.0, 10.0),
            fill("2026-11-01", 87.0, 110.0, 10.0),
            fill("2026-11-01", 87.0, 10.0, 0.0),
        ] {
            assert!(FuelLog::default().add(bad.clone()).is_err(), "{bad:?}");
        }
    }

    #[test]
    fn the_tank_mixes_and_the_unknown_leftover_is_said() {
        let mut log = FuelLog::default();
        // 16 of 21.1 gallons of 91 E0: 5.1 gallons from before are not known
        log.add(fill("2026-11-01 10:00", 91.0, 0.0, 16.0)).unwrap();
        // then 15 gallons of 87 E10 over 6.1 left: (6.1 * 91 + 15 * 87) / 21.1
        log.add(fill("2026-11-10 10:00", 87.0, 10.0, 15.0)).unwrap();
        let t = tanks(&log, Some(21.1));
        assert_eq!(t.len(), 2);
        let first = &t[0];
        assert!((first.unknown.unwrap() - 5.1 / 21.1).abs() < 1e-9);
        assert_eq!(first.blend(), None, "24 % unknown: no blend named");
        assert!(first
            .estimate_words()
            .contains("24 % of the tank is from before the first logged fill-up"));
        let second = &t[1];
        let u = 5.1 / 21.1 * 6.1 / 21.1;
        assert!((second.unknown.unwrap() - u).abs() < 1e-9, "7 % unknown");
        // the known part: 6.1 * (1 - 0.2417) = 4.626 gallons of 91 E0 and 15 of 87 E10
        let k = 6.1 * (1.0 - 5.1 / 21.1);
        let aki = (k * 91.0 + 15.0 * 87.0) / (k + 15.0);
        let eth = (15.0 * 10.0) / (k + 15.0);
        assert!((second.aki.unwrap() - aki).abs() < 1e-9);
        assert!((second.ethanol.unwrap() - eth).abs() < 1e-9);
        assert_eq!(second.blend().as_deref(), Some("88 E8"));
        // a full tank of the new fuel is all of it
        let mut full = FuelLog::default();
        full.add(fill("2026-11-01", 87.0, 15.0, 22.0)).unwrap();
        let f = &tanks(&full, Some(21.1))[0];
        assert_eq!(
            (f.unknown, f.blend().as_deref()),
            (Some(0.0), Some("87 E15"))
        );
        assert!(f.over_full && f.line().contains("more gallons than the tank holds"));
        // no tank size: nothing estimated, said
        let n = &tanks(&log, None)[1];
        assert_eq!((n.unknown, n.blend()), (None, None));
        assert!(n.line().contains("the tank's size is not known"));
        // the tank a drive ran on: the last fill-up at or before its start
        assert_eq!(tank_at(&t, "2026-10-31 09:00:00"), None);
        assert_eq!(
            tank_at(&t, "2026-11-10 10:00:00").unwrap().fill.grade,
            87.0,
            "a fill-up the minute the drive starts counts"
        );
        assert_eq!(tank_at(&t, "2026-11-09 23:59:59").unwrap().fill.grade, 91.0);
    }

    #[test]
    fn the_tank_size_comes_from_the_log_or_a_tacoma() {
        let tacoma = Described::parse("2017 Toyota Tacoma 3.5L V6");
        let log = FuelLog::default();
        assert_eq!(tank_size(&log, Some(&tacoma)).unwrap().0, 21.1);
        assert_eq!(
            tank_size(&log, Some(&Described::parse("2010 Toyota Tacoma"))),
            None,
            "only a 2016-2023 Tacoma"
        );
        assert_eq!(
            tank_size(&log, Some(&Described::parse("2017 Ford F-150"))),
            None
        );
        assert_eq!(tank_size(&log, None), None);
        let set = FuelLog {
            tank_gallons: Some(18.5),
            ..FuelLog::default()
        };
        assert_eq!(
            tank_size(&set, Some(&tacoma)).unwrap().0,
            18.5,
            "the setting first"
        );
    }

    #[test]
    fn a_recordings_fuel_line_reads_back() {
        let mut log = FuelLog::default();
        log.add(fill("2026-11-01", 91.0, 0.0, 21.1)).unwrap();
        let mut second = fill("2026-11-10 10:00", 87.0, 10.0, 15.0);
        second.odometer = Some(84_500);
        log.add(second).unwrap();
        let t = tank_at(&tanks(&log, Some(21.1)), "2026-11-12 08:00:00").unwrap();
        let line = t.line();
        assert!(
            line.starts_with("# fuel: tank 87 E10; filled 2026-11-10 10:00, 15 gal, odometer 84500 mi; blend 88 E7 (estimated 88.2 AKI, 7.1 % ethanol)"),
            "{line}"
        );
        let tag = of_recording(&format!("# adapter: x\n{line}\ntime_s,a\n0,1\n")).unwrap();
        assert_eq!(
            (tag.tank.as_str(), tag.blend.as_deref()),
            ("87 E10", Some("88 E7"))
        );
        assert!(
            (tag.aki.unwrap() - 88.2).abs() < 0.01 && (tag.ethanol.unwrap() - 7.1).abs() < 0.01
        );
        assert_eq!(tag.short(), "87 E10 tank, blend 88 E7");
        assert!(tag.matches(&Want::parse("88 E8").unwrap()));
        assert!(tag.matches(&Want::parse("88").unwrap()));
        assert!(
            !tag.matches(&Want::parse("87 E10").unwrap()),
            "0.5 octane at most"
        );
        assert!(!tag.matches(&Want::parse("91 E0").unwrap()));
        assert_eq!(Want::parse("87 e10").unwrap().ethanol, Some(10.0));
        assert_eq!(Want::parse("drive-a.csv"), None);
        assert_eq!(Want::parse("87 E10 extra"), None);
        // an unknown blend is read as unknown, and matches nothing
        let first = tank_at(
            &tanks(
                &FuelLog {
                    fills: vec![fill("2026-11-01", 91.0, 0.0, 10.0)],
                    ..FuelLog::default()
                },
                Some(21.1),
            ),
            "2026-11-02",
        )
        .unwrap();
        let tag = of_recording(&first.line()).unwrap();
        assert_eq!(
            (tag.tank.as_str(), tag.blend.as_deref(), tag.aki),
            ("91 E0", None, None)
        );
        assert!(!tag.matches(&Want::parse("91 E0").unwrap()));
        assert_eq!(of_recording("time_s,a\n0,1\n"), None);
    }

    #[test]
    fn a_typo_is_said_and_a_broken_log_never_replaced() {
        // a key it does not know is an error, never read as nothing (ethonal = 10 is not E0)
        let e = FuelLog::parse(
            "[[fill]]\ndate = \"2026-11-01\"\ngrade = 87\nethonal = 10\ngallons = 10\n",
        )
        .unwrap_err();
        assert!(e.contains("ethonal"), "{e}");
        assert!(FuelLog::parse("tank_galons = 21\n").is_err());
        let dir = std::env::temp_dir().join(format!(
            "icarus-fuel-VINFOLDERX1234567-{}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join(FILE), "[[fill]]\nethonal = 10\n").unwrap();
        // the recording's line names neither the folder (it may be the VIN) nor the path
        let line = line_in(&dir, None, "2026-11-02").unwrap();
        assert!(
            line.starts_with("# fuel: not known (fuel.toml does not read: ")
                && !line.contains("VINFOLDER"),
            "{line}"
        );
        assert!(!line.contains('\n'));
        // an update over it is refused, the file left as it was
        let r = FuelLog::update(&dir, |l| l.add(fill("2026-11-02", 87.0, 10.0, 10.0)));
        assert!(r.is_err());
        assert_eq!(
            std::fs::read_to_string(dir.join(FILE)).unwrap(),
            "[[fill]]\nethonal = 10\n"
        );
        // a good one: added to what is on disk, under the lock, no temporary file left
        std::fs::write(dir.join(FILE), FuelLog::default().text()).unwrap();
        let (_, log, _) =
            FuelLog::update(&dir, |l| l.add(fill("2026-11-02", 87.0, 10.0, 10.0))).unwrap();
        let (_, log2, _) =
            FuelLog::update(&dir, |l| l.add(fill("2026-11-03", 91.0, 0.0, 9.0))).unwrap();
        assert_eq!((log.fills.len(), log2.fills.len()), (1, 2));
        assert_eq!(FuelLog::load(&dir).unwrap(), log2);
        let left: Vec<_> = std::fs::read_dir(&dir).unwrap().flatten().collect();
        assert_eq!(left.len(), 1, "{left:?}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn what_was_in_the_tank_before_the_first_fill_up() {
        let mut first = fill("2026-11-01 10:00", 87.0, 10.0, 15.0);
        first.before = Some("91 e0".into());
        let mut log = FuelLog::default();
        log.add(first).unwrap();
        assert_eq!(
            log.fills[0].before.as_deref(),
            Some("91 E0"),
            "written as the log writes it"
        );
        let t = &tanks(&log, Some(21.1))[0];
        // 6.1 gallons of 91 E0 and 15 of 87 E10: nothing unknown
        assert_eq!(t.unknown, Some(0.0));
        assert_eq!(t.blend().as_deref(), Some("88 E7"));
        assert!(
            t.line()
                .contains("the 6.1 gal left before it taken as 91 E0, as stated"),
            "{}",
            t.line()
        );
        let back = FuelLog::parse(&log.text()).unwrap();
        assert_eq!(back.fills[0].before.as_deref(), Some("91 E0"));
        // a later fill-up may not say it: the leftover is worked out from the one before
        let mut later = fill("2026-11-10", 87.0, 10.0, 15.0);
        later.before = Some("91 E0".into());
        assert!(log.add(later).unwrap_err().contains("is logged before"));
        // a blend needs both
        let mut bad = fill("2026-10-01", 87.0, 10.0, 15.0);
        bad.before = Some("91".into());
        assert!(log.add(bad).is_err());
        // a fill-up dated earlier added by hand afterwards: the stated leftover is no longer used, and that is said
        log.fills.push(fill("2026-10-20", 91.0, 0.0, 21.1));
        let t = &tanks(&log, Some(21.1))[1];
        assert!(t.before_ignored && t.before_used.is_none());
        assert!(t.line().contains("an earlier fill-up is logged"));
    }

    /// The tank is worked out when a recording is read, from its vehicle's fuel log and when it began: a stale line is
    /// replaced, a missing one added; a fill-up logged with a date only names no tank for that day's drives; a recording
    /// whose vehicle or log is not found keeps the line it has.
    #[test]
    fn a_recordings_tank_is_worked_out_when_it_is_read() {
        let lib = std::env::temp_dir().join(format!("icarus-fuel-read-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&lib);
        let truck = lib.join("Made-up Truck");
        std::fs::create_dir_all(truck.join("Logs")).unwrap();
        std::fs::create_dir_all(lib.join("Other Car")).unwrap();
        std::fs::write(
            truck.join("vehicle.toml"),
            "vin = \"MADEUP00000000001\"\nname = \"2017 Toyota Tacoma 3.5L V6\"\n",
        )
        .unwrap();
        let mut log = FuelLog::default();
        log.add(fill("2026-11-01 09:00", 91.0, 0.0, 21.1)).unwrap();
        log.add(fill("2026-11-10", 87.0, 10.0, 15.0)).unwrap();
        log.save(&truck).unwrap();
        let rec = |head: &str| format!("# adapter: x\n{head}time_s,a\n0,1\n0.5,1\n");
        let line_of = |t: &str| {
            let v: Vec<&str> = t.lines().filter(|l| l.starts_with("# fuel: ")).collect();
            assert!(v.len() <= 1, "one fuel line at most: {t}");
            v.first().map(|l| l.to_string())
        };
        // a stale line replaced (the Tacoma's 21.1 gallons from its description), in its place
        let stale = rec("# vehicle: Made-up Truck\n# began: 2026-11-05 08:00:00\n# fuel: tank 87 E15; filled 2026-10-01, 5 gal; blend unknown (all of the tank is from before the first logged fill-up)\n");
        let t = with_line_at_read(&lib, None, stale);
        assert!(
            line_of(&t)
                .unwrap()
                .starts_with("# fuel: tank 91 E0; filled 2026-11-01 09:00, 21.1 gal; blend 91 E0"),
            "{t}"
        );
        assert!(t.find("# fuel:").unwrap() < t.find("time_s").unwrap());
        assert_eq!(of_recording(&t).unwrap().blend.as_deref(), Some("91 E0"));
        // the day of a fill-up logged with no time: not known, said
        let t = with_line_at_read(
            &lib,
            None,
            rec("# vehicle: Made-up Truck\n# began: 2026-11-10 07:00:00\n"),
        );
        assert_eq!(
            line_of(&t).as_deref(),
            Some("# fuel: not known (a fill-up on 2026-11-10 is logged with no time, so a drive that day may have run on the tank before it or after it)")
        );
        assert_eq!(of_recording(&t), None);
        // the next day: the new tank, added where there was no line
        let t = with_line_at_read(
            &lib,
            None,
            rec("# vehicle: Made-up Truck\n# began: 2026-11-11 07:00:00\n"),
        );
        assert!(line_of(&t)
            .unwrap()
            .starts_with("# fuel: tank 87 E10; filled 2026-11-10, 15 gal; blend 88 E7"));
        // no vehicle line, but in the vehicle's own Logs folder
        let own = truck.join("Logs/drive-x.csv");
        let t = with_line_at_read(&lib, Some(&own), rec("# began: 2026-11-05 08:00:00\n"));
        assert!(line_of(&t).unwrap().contains("tank 91 E0"));
        // not found: the line it has stands (another vehicle with no log, or no vehicle at all)
        let keep = rec("# vehicle: Other Car\n# began: 2026-11-05 08:00:00\n# fuel: tank 87 E10; filled 2026-11-01, 13 gal; blend 87 E10 (estimated 87.0 AKI, 10.0 % ethanol)\n");
        assert_eq!(with_line_at_read(&lib, None, keep.clone()), keep);
        let bare = rec("# began: 2026-11-05 08:00:00\n");
        assert_eq!(
            with_line_at_read(&lib, Some(&lib.join("x.csv")), bare.clone()),
            bare
        );
        assert_eq!(
            with_line_at_read(&lib, None, rec("# vehicle: ../x\n")),
            rec("# vehicle: ../x\n")
        );
        let _ = std::fs::remove_dir_all(&lib);
    }
}
