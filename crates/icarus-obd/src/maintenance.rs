//! Each vehicle's maintenance log (ICA-185): the services done (oil and filter, spark plugs, a sensor, a fuel change...),
//! each with its date and the odometer then; the interval I set for each kind (miles, months or both); and what is due by
//! miles or by months. Kept in the vehicle's folder as `maintenance.toml`, a plain file meant to be read and edited by hand
//! as well (its format is written at its top, and in the README):
//!
//! ```toml
//! [intervals.oil]
//! miles = 5000
//! months = 6
//!
//! [[service]]
//! kind = "spark-plugs"
//! date = "2026-11-29 14:30"   # or only the date: "2026-11-29"
//! miles = 84213               # the odometer then, in miles (optional)
//! note = "NGK, gapped 1.1 mm" # optional; "other" needs one
//! ```
//!
//! The odometer is typed in. Where the vehicle gives a reading (standard value A6, or the vehicle's module-data set's
//! odometer, read live or found in a recording) it is offered (`Offer`), always shown for the owner to confirm and never
//! saved by itself. Icarus only reads an odometer; nothing here or anywhere in Icarus changes one.
//!
//! A service also marks a moment to compare the drives around (ICA-215, `before_after`): the last drives recorded before
//! it against the first ones after, by ICA-210's comparison (`drivelog::compare`), with misfire counts when the recordings
//! carry them. Offline: only files are read.

use crate::drivelog::{self, compare};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// The log's file in the vehicle's folder.
pub const FILE: &str = "maintenance.toml";

/// Due soon: within this many miles of the interval's end...
pub const SOON_MILES: i64 = 500;
/// ... or this many days.
pub const SOON_DAYS: i64 = 30;

/// The drives on each side of a service compared unless told otherwise.
pub const DRIVES: usize = 3;

/// A kind of service.
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
#[serde(rename_all = "kebab-case")]
pub enum Kind {
    Oil,
    SparkPlugs,
    AirFilter,
    Fluids,
    Brakes,
    TireRotation,
    Sensor,
    Fuel,
    Other,
}

impl Kind {
    pub const ALL: [Kind; 9] = [
        Kind::Oil,
        Kind::SparkPlugs,
        Kind::AirFilter,
        Kind::Fluids,
        Kind::Brakes,
        Kind::TireRotation,
        Kind::Sensor,
        Kind::Fuel,
        Kind::Other,
    ];

    /// As the file and the command line write it: "spark-plugs".
    pub fn key(self) -> &'static str {
        match self {
            Kind::Oil => "oil",
            Kind::SparkPlugs => "spark-plugs",
            Kind::AirFilter => "air-filter",
            Kind::Fluids => "fluids",
            Kind::Brakes => "brakes",
            Kind::TireRotation => "tire-rotation",
            Kind::Sensor => "sensor",
            Kind::Fuel => "fuel",
            Kind::Other => "other",
        }
    }

    /// In words: "spark plugs".
    pub fn name(self) -> &'static str {
        match self {
            Kind::Oil => "oil and filter",
            Kind::SparkPlugs => "spark plugs",
            Kind::AirFilter => "air filter",
            Kind::Fluids => "fluids",
            Kind::Brakes => "brakes",
            Kind::TireRotation => "tire rotation",
            Kind::Sensor => "a sensor",
            Kind::Fuel => "a fuel change",
            Kind::Other => "other",
        }
    }

    /// A kind as typed: its key, or a few plain words for it ("oil-and-filter", "plugs", "spark plugs", "rotation").
    pub fn parse(s: &str) -> Result<Kind, String> {
        let w: String = s
            .trim()
            .to_lowercase()
            .split(|c: char| c.is_whitespace() || c == '_' || c == '-')
            .filter(|p| !p.is_empty())
            .collect::<Vec<_>>()
            .join("-");
        let w = w.strip_prefix("a-").unwrap_or(&w);
        Ok(match w {
            "oil" | "oil-and-filter" | "oil-filter" | "oil-change" => Kind::Oil,
            "spark-plugs" | "spark-plug" | "plugs" | "plug" => Kind::SparkPlugs,
            "air-filter" | "engine-air-filter" => Kind::AirFilter,
            "fluids" | "fluid" => Kind::Fluids,
            "brakes" | "brake" | "brake-pads" => Kind::Brakes,
            "tire-rotation" | "rotation" | "tires-rotated" => Kind::TireRotation,
            "sensor" => Kind::Sensor,
            "fuel" | "fuel-change" => Kind::Fuel,
            "other" => Kind::Other,
            _ => {
                return Err(format!(
                    "{s:?} is not a kind of service: {} (other with a note)",
                    Kind::ALL.map(Kind::key).join(", ")
                ))
            }
        })
    }
}

/// When a service was done: a day (days since 1970-01-01), and the minute of that day when the time was written.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct When {
    pub day: i64,
    pub minute: Option<u32>,
}

/// Days since 1970-01-01 of a date (the proleptic Gregorian calendar).
pub fn days_from_civil(y: i64, m: u32, d: u32) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let mp = (m as i64 + 9) % 12;
    let doy = (153 * mp + 2) / 5 + d as i64 - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

/// The date of a day since 1970-01-01: (year, month, day).
pub fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    (
        if m <= 2 {
            yoe + era * 400 + 1
        } else {
            yoe + era * 400
        },
        m,
        d,
    )
}

fn month_len(y: i64, m: u32) -> u32 {
    match m {
        2 if (y % 4 == 0 && y % 100 != 0) || y % 400 == 0 => 29,
        2 => 28,
        4 | 6 | 9 | 11 => 30,
        _ => 31,
    }
}

/// A day `months` months later: the same day of the month, or the month's last day when it has fewer (Aug 31 + 6 months is
/// Feb 28, or 29).
pub fn add_months(day: i64, months: u32) -> i64 {
    let (y, m, d) = civil_from_days(day);
    let total = m as i64 - 1 + months as i64;
    let (y, m) = (y + total.div_euclid(12), (total.rem_euclid(12) + 1) as u32);
    days_from_civil(y, m, d.min(month_len(y, m)))
}

/// "2026-11-29".
pub fn date_text(day: i64) -> String {
    let (y, m, d) = civil_from_days(day);
    format!("{y:04}-{m:02}-{d:02}")
}

impl When {
    /// "2026-11-29" or "2026-11-29 14:30" (a T between them too).
    pub fn parse(s: &str) -> Result<When, String> {
        let bad = || {
            format!(
                "{s:?} is not a date: write it as 2026-11-29, or 2026-11-29 14:30 with the time"
            )
        };
        let s = s.trim();
        let (date, time) = match s.split_once([' ', 'T']) {
            Some((d, t)) => (d, Some(t.trim())),
            None => (s, None),
        };
        let p: Vec<&str> = date.split('-').collect();
        let [y, m, d] = p.as_slice() else {
            return Err(bad());
        };
        if y.len() != 4 {
            return Err(bad());
        }
        let (y, m, d) = (
            y.parse::<i64>().map_err(|_| bad())?,
            m.parse::<u32>().map_err(|_| bad())?,
            d.parse::<u32>().map_err(|_| bad())?,
        );
        if !(1..=12).contains(&m) || d == 0 || d > month_len(y, m) {
            return Err(bad());
        }
        let minute = match time {
            None => None,
            Some(t) => {
                let (h, mi) = t.split_once(':').ok_or_else(bad)?;
                let (h, mi) = (
                    h.parse::<u32>().map_err(|_| bad())?,
                    mi.get(..2)
                        .unwrap_or(mi)
                        .parse::<u32>()
                        .map_err(|_| bad())?,
                );
                if h > 23 || mi > 59 {
                    return Err(bad());
                }
                Some(h * 60 + mi)
            }
        };
        Ok(When {
            day: days_from_civil(y, m, d),
            minute,
        })
    }

    /// As the file writes it.
    pub fn text(&self) -> String {
        match self.minute {
            Some(m) => format!("{} {:02}:{:02}", date_text(self.day), m / 60, m % 60),
            None => date_text(self.day),
        }
    }

    /// Now, local time.
    pub fn now() -> When {
        When::parse(&crate::report::now()).unwrap_or(When {
            day: 0,
            minute: None,
        })
    }
}

/// One service done. Keys Icarus does not know (written by hand: `shop = "..."`, `cost = 45`) are kept in `extra` and
/// written back as they were.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Service {
    pub kind: Kind,
    /// "2026-11-29", or with the time: "2026-11-29 14:30".
    pub date: String,
    /// The odometer then, in miles.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub miles: Option<u32>,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub note: String,
    #[serde(flatten)]
    pub extra: toml::Table,
}

impl Service {
    pub fn new(kind: Kind, date: &str, miles: Option<u32>, note: &str) -> Service {
        Service {
            kind,
            date: date.to_string(),
            miles,
            note: note.to_string(),
            extra: toml::Table::new(),
        }
    }

    pub fn when(&self) -> Option<When> {
        When::parse(&self.date).ok()
    }

    /// "spark plugs (NGK)" or "other: PCV valve".
    pub fn what(&self) -> String {
        match (self.kind, self.note.trim()) {
            (_, "") => self.kind.name().to_string(),
            (Kind::Other, n) => n.to_string(),
            (k, n) => format!("{} ({n})", k.name()),
        }
    }

    /// "2026-11-29 at 84,213 mi: spark plugs (NGK)".
    pub fn line(&self) -> String {
        format!(
            "{}{}: {}",
            self.date,
            self.miles
                .map(|m| format!(" at {}", miles_text(m.into())))
                .unwrap_or_default(),
            self.what()
        )
    }
}

/// A kind's interval: whichever comes first of its miles and its months. Keys Icarus does not know are kept.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Interval {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub miles: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub months: Option<u32>,
    #[serde(flatten)]
    pub extra: toml::Table,
}

impl Interval {
    pub fn new(miles: Option<u32>, months: Option<u32>) -> Interval {
        Interval {
            miles,
            months,
            extra: toml::Table::new(),
        }
    }

    /// "every 5,000 mi or 6 months".
    pub fn text(&self) -> String {
        match (self.miles, self.months) {
            (Some(mi), Some(mo)) => {
                format!("every {} or {}", miles_text(mi.into()), months_text(mo))
            }
            (Some(mi), None) => format!("every {}", miles_text(mi.into())),
            (None, Some(mo)) => format!("every {}", months_text(mo)),
            (None, None) => "no interval".into(),
        }
    }
}

fn months_text(m: u32) -> String {
    format!("{m} month{}", if m == 1 { "" } else { "s" })
}

/// "84,213 mi".
pub fn miles_text(m: u64) -> String {
    let s = m.to_string();
    let mut out = String::new();
    for (i, c) in s.chars().enumerate() {
        if i > 0 && (s.len() - i).is_multiple_of(3) {
            out.push(',');
        }
        out.push(c);
    }
    format!("{out} mi")
}

/// A vehicle's log: the intervals (by kind's key) and the services, oldest first; top-level keys Icarus does not know are
/// kept in `extra`.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Log {
    #[serde(default)]
    pub intervals: BTreeMap<String, Interval>,
    #[serde(default, rename = "service")]
    pub services: Vec<Service>,
    #[serde(flatten)]
    pub extra: toml::Table,
}

const HEAD: &str = "# Icarus: this vehicle's maintenance log, on this computer only. Edit it by hand as you like:\n\
# [intervals.KIND] miles = N and/or months = M (whichever comes first); each [[service]] has kind, date\n\
# (\"2026-11-29\" or \"2026-11-29 14:30\"), miles (the odometer then, optional) and note (optional; other needs one).\n\
# Kinds: oil, spark-plugs, air-filter, fluids, brakes, tire-rotation, sensor, fuel, other.\n\
# Keys of your own (shop = \"...\", cost = 45) are kept when Icarus saves; comments other than these lines are not.\n";

impl Log {
    /// The interval set for a kind.
    pub fn interval(&self, k: Kind) -> Option<Interval> {
        self.intervals
            .get(k.key())
            .filter(|i| i.miles.is_some() || i.months.is_some())
            .cloned()
    }

    /// The latest service of a kind (by its date; the odometer breaks a tie).
    pub fn last(&self, k: Kind) -> Option<&Service> {
        self.services
            .iter()
            .filter(|s| s.kind == k)
            .max_by_key(|s| (s.when(), s.miles))
    }

    /// The highest odometer the log holds, and the service it came with.
    pub fn highest_miles(&self) -> Option<(u32, &Service)> {
        self.services
            .iter()
            .filter_map(|s| s.miles.map(|m| (m, s)))
            .max_by_key(|x| x.0)
    }

    /// The log as read from a file edited by hand: every date must read, and each interval's kind, written its one way
    /// ("spark_plugs" is "spark-plugs").
    fn checked(self) -> Result<Log, String> {
        for s in &self.services {
            When::parse(&s.date).map_err(|e| format!("a {} service: {e}", s.kind.key()))?;
        }
        let mut intervals = BTreeMap::new();
        for (k, v) in self.intervals {
            let kind = Kind::parse(&k).map_err(|e| format!("[intervals.{k}]: {e}"))?;
            intervals.insert(kind.key().to_string(), v);
        }
        Ok(Log { intervals, ..self })
    }
}

/// The log's file in a vehicle's folder.
pub fn path(dir: &Path) -> PathBuf {
    dir.join(FILE)
}

/// The log in a vehicle's folder (none yet: empty). A file that does not read says where and why, never taken as empty.
pub fn load(dir: &Path) -> Result<Log, String> {
    let p = path(dir);
    let text = match std::fs::read_to_string(&p) {
        Ok(t) => t,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(Log::default()),
        Err(e) => return Err(format!("{}: {e}", p.display())),
    };
    let log: Log = toml::from_str(&text).map_err(|e| format!("{}: {e}", p.display()))?;
    log.checked().map_err(|e| format!("{}: {e}", p.display()))
}

/// Writes `text` to `p` whole or not at all: into a new file of this process in the same folder, flushed to the disk,
/// then renamed over `p`, so a crash or a full disk midway leaves the old file as it was. (The pattern of vehicles.rs's
/// `write_replacing` on main; one shared helper once both are on one branch.)
fn write_replacing(p: &Path, text: &str) -> Result<(), String> {
    use std::io::Write;
    let name = p.file_name().and_then(|n| n.to_str()).unwrap_or(FILE);
    let tmp = p.with_file_name(format!(".{name}.new-{}", std::process::id()));
    let written = (|| {
        let mut f = std::fs::File::create(&tmp)?;
        f.write_all(text.as_bytes())?;
        f.sync_all()?;
        std::fs::rename(&tmp, p)
    })();
    if let Err(e) = written {
        let _ = std::fs::remove_file(&tmp);
        return Err(format!("{}: {e}", p.display()));
    }
    if let Some(d) = p.parent().and_then(|d| std::fs::File::open(d).ok()) {
        let _ = d.sync_all();
    }
    Ok(())
}

/// Runs `f` holding the log's lock (an flock on `.maintenance.lock` in the vehicle's folder), so the window and the
/// command line changing the log at once do not lose one another's change.
fn locked<T>(dir: &Path, f: impl FnOnce() -> Result<T, String>) -> Result<T, String> {
    use std::os::fd::AsRawFd;
    std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let lp = dir.join(".maintenance.lock");
    let lock = std::fs::OpenOptions::new()
        .create(true)
        .truncate(false)
        .write(true)
        .open(&lp)
        .map_err(|e| format!("{}: {e}", lp.display()))?;
    if unsafe { libc::flock(lock.as_raw_fd(), libc::LOCK_EX) } != 0 {
        return Err(format!(
            "{}: {}",
            lp.display(),
            std::io::Error::last_os_error()
        ));
    }
    let r = f();
    unsafe { libc::flock(lock.as_raw_fd(), libc::LOCK_UN) };
    r
}

/// Saves the log in a vehicle's folder, its services in date order.
pub fn save(dir: &Path, log: &Log) -> Result<(), String> {
    let mut log = log.clone();
    log.services.sort_by_key(|s| s.when());
    log.intervals
        .retain(|_, i| i.miles.is_some() || i.months.is_some() || !i.extra.is_empty());
    let body = toml::to_string(&log).map_err(|e| e.to_string())?;
    std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    write_replacing(&path(dir), &format!("{HEAD}\n{body}"))
}

/// Adds a service to the log in `dir`: its date checked (and written the one way), "other" with a note. Says what looks
/// wrong without refusing it: an odometer lower than one logged earlier (a typo, or the miles of another vehicle).
pub fn add(dir: &Path, s: Service) -> Result<(Service, Vec<String>), String> {
    let when = When::parse(&s.date)?;
    if s.kind == Kind::Other && s.note.trim().is_empty() {
        return Err("other needs a note saying what was done (--note \"PCV valve\")".into());
    }
    if s.miles.is_some_and(|m| m > 2_000_000) {
        return Err("that odometer is more than 2,000,000 miles: check it".into());
    }
    let s = Service {
        date: when.text(),
        note: s.note.split_whitespace().collect::<Vec<_>>().join(" "),
        ..s
    };
    locked(dir, || {
        let mut log = load(dir)?;
        let mut warnings = vec![];
        if let Some(m) = s.miles {
            if let Some(e) = log
                .services
                .iter()
                .filter(|e| e.when().is_some_and(|w| w.day < when.day))
                .filter(|e| e.miles.is_some_and(|em| em > m))
                .max_by_key(|e| e.miles)
            {
                warnings.push(format!(
                    "{} is less than the {} logged on {} ({}): check the odometer",
                    miles_text(m.into()),
                    miles_text(e.miles.unwrap_or(0).into()),
                    e.date,
                    e.kind.name()
                ));
            }
        }
        log.services.push(s.clone());
        save(dir, &log)?;
        Ok((s, warnings))
    })
}

/// Sets (or, with neither, clears) a kind's interval in the log in `dir`; keys of the owner's own under it are kept.
pub fn set_interval(
    dir: &Path,
    k: Kind,
    miles: Option<u32>,
    months: Option<u32>,
) -> Result<(), String> {
    if miles == Some(0) || months == Some(0) {
        return Err("an interval of 0 is no interval: leave it out, or clear it with off".into());
    }
    locked(dir, || {
        let mut log = load(dir)?;
        let i = log.intervals.entry(k.key().to_string()).or_default();
        i.miles = miles;
        i.months = months;
        save(dir, &log)
    })
}

/// Where a kind stands against its interval.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// An interval, but no service of the kind logged yet.
    NeverLogged,
    Overdue,
    /// Within `SOON_MILES` or `SOON_DAYS`.
    Soon,
    Later,
    /// By miles only, and the miles then or now are not known.
    Unknown,
}

/// One kind's due line.
#[derive(Clone, Debug, PartialEq)]
pub struct Due {
    pub kind: Kind,
    pub interval: Interval,
    pub last: Option<Service>,
    /// Due at this odometer (when the interval has miles and the last service has its odometer).
    pub at_miles: Option<u64>,
    /// Due by this day (when the interval has months).
    pub by_day: Option<i64>,
    /// Miles left (negative: past), when the odometer now is known too.
    pub miles_left: Option<i64>,
    /// Days left (negative: past).
    pub days_left: Option<i64>,
    pub state: State,
}

/// What is due, for every kind with an interval: by miles against `odometer` (the miles now, when known) and by months
/// against `today` (days since 1970-01-01). Most pressing first.
pub fn due(log: &Log, today: i64, odometer: Option<u32>) -> Vec<Due> {
    let mut out: Vec<Due> = Kind::ALL
        .iter()
        .filter_map(|&k| {
            let interval = log.interval(k)?;
            let last = log.last(k).cloned();
            let Some(l) = &last else {
                return Some(Due {
                    kind: k,
                    interval,
                    last: None,
                    at_miles: None,
                    by_day: None,
                    miles_left: None,
                    days_left: None,
                    state: State::NeverLogged,
                });
            };
            let at_miles = interval
                .miles
                .zip(l.miles)
                .map(|(i, m)| u64::from(m) + u64::from(i));
            let by_day = interval
                .months
                .zip(l.when())
                .map(|(mo, w)| add_months(w.day, mo));
            let miles_left = at_miles
                .zip(odometer)
                .map(|(at, now)| at as i64 - i64::from(now));
            let days_left = by_day.map(|d| d - today);
            let state = if miles_left.is_some_and(|m| m <= 0) || days_left.is_some_and(|d| d <= 0) {
                State::Overdue
            } else if miles_left.is_some_and(|m| m <= SOON_MILES)
                || days_left.is_some_and(|d| d <= SOON_DAYS)
            {
                State::Soon
            } else if miles_left.is_none() && days_left.is_none() {
                State::Unknown
            } else {
                State::Later
            };
            Some(Due {
                kind: k,
                interval,
                last,
                at_miles,
                by_day,
                miles_left,
                days_left,
                state,
            })
        })
        .collect();
    let rank = |s: State| match s {
        State::Overdue => 0,
        State::Soon => 1,
        State::NeverLogged => 2,
        State::Unknown => 3,
        State::Later => 4,
    };
    out.sort_by_key(|d| (rank(d.state), d.kind));
    out
}

fn days_text(d: i64) -> String {
    format!("{d} day{}", if d.abs() == 1 { "" } else { "s" })
}

impl Due {
    /// "oil and filter: overdue by 300 mi (due at 89,213 mi), last 2026-05-01 at 84,213 mi; every 5,000 mi or 6 months".
    pub fn text(&self) -> String {
        let name = self.kind.name();
        let every = self.interval.text();
        let Some(l) = &self.last else {
            return format!(
                "{name}: none logged yet ({every}): log the last one to know when the next is due"
            );
        };
        let mut parts = vec![];
        match (self.at_miles, self.miles_left) {
            (Some(at), Some(left)) if left <= 0 => parts.push(format!(
                "overdue by {} (due at {})",
                miles_text(left.unsigned_abs()),
                miles_text(at)
            )),
            (Some(at), Some(left)) => parts.push(format!(
                "in {} (at {})",
                miles_text(left.unsigned_abs()),
                miles_text(at)
            )),
            (Some(at), None) => parts.push(format!(
                "at {} (the odometer now is not known)",
                miles_text(at)
            )),
            (None, _) if self.interval.miles.is_some() => {
                parts.push("by miles not known (no odometer logged with the last one)".into())
            }
            _ => {}
        }
        if let (Some(by), Some(left)) = (self.by_day, self.days_left) {
            parts.push(if left <= 0 {
                format!("overdue since {} ({} ago)", date_text(by), days_text(-left))
            } else {
                format!("by {} (in {})", date_text(by), days_text(left))
            });
        }
        let state = match self.state {
            State::Overdue => "OVERDUE",
            State::Soon => "due soon",
            State::Later => "ok",
            State::Unknown => "not known",
            State::NeverLogged => "none logged",
        };
        format!(
            "{name} [{state}]: {}; last {}{}; {every}",
            parts.join(", "),
            l.date,
            l.miles
                .map(|m| format!(" at {}", miles_text(m.into())))
                .unwrap_or_default()
        )
    }
}

/// An odometer reading offered for a service: the miles and where they came from, for the owner to confirm; with a doubt
/// when the reading may be another vehicle's (a shared-folder recording that does not say which vehicle it is from).
#[derive(Clone, Debug, PartialEq)]
pub struct Offer {
    pub miles: u32,
    pub from: String,
    pub doubt: Option<String>,
}

impl Offer {
    /// "84,213 mi from ..." and its doubt, in words.
    pub fn text(&self) -> String {
        let mut t = format!("{} from {}", miles_text(self.miles.into()), self.from);
        if let Some(d) = &self.doubt {
            t += &format!(" (but {d})");
        }
        t
    }
}

/// A reading in `unit` as miles: km converted, mi as they are; any other unit (or none: an unchecked value) is not taken.
pub fn miles_of(v: f64, unit: &str) -> Option<u32> {
    if !v.is_finite() || v < 0.0 {
        return None;
    }
    let m = match unit.trim() {
        "km" => v / 1.609_344,
        "mi" | "miles" => v,
        _ => return None,
    };
    (m < u32::MAX as f64).then(|| m.round() as u32)
}

/// A column's unit, from the `[unit]` its name ends with.
fn unit_of(name: &str) -> &str {
    name.rsplit_once('[')
        .and_then(|(_, u)| u.strip_suffix(']'))
        .unwrap_or("")
}

/// The odometer a recording read last: a column named odometer (standard value A6, `Odometer [km]`, or a module-data
/// value, `Odometer (7E0 22 ...) [mi]`) whose unit is known.
pub fn odometer_in(text: &str) -> Option<(u32, String)> {
    let s = drivelog::series(text).ok()?;
    s.names
        .iter()
        .zip(&s.cols)
        .filter(|(n, _)| n.to_lowercase().contains("odometer") && !n.ends_with("[hex]"))
        .filter_map(|(n, c)| {
            let v = c.iter().rev().flatten().next()?;
            Some((miles_of(*v, unit_of(n))?, n.clone()))
        })
        .max_by_key(|x| x.0)
}

/// How much of a recording's end is read for its last odometer reading (the odometer is read in turn, so it is in the last
/// rows of any drive).
const TAIL_BYTES: u64 = 256 * 1024;

/// A recording's `#` lines and header, and its last `tail` bytes from the first whole line in them: what its start, its
/// vehicle and its last readings are told from, without reading the rows between.
fn head_and_tail(p: &Path, tail: u64) -> Option<(Vec<String>, Option<String>, String)> {
    use std::io::{BufRead, Read, Seek};
    let f = std::fs::File::open(p).ok()?;
    let len = f.metadata().ok()?.len();
    let mut r = std::io::BufReader::new(f);
    let (mut hashes, mut header, mut taken) = (vec![], None, 0u64);
    for _ in 0..64 {
        let mut l = String::new();
        let n = r.read_line(&mut l).ok()?;
        if n == 0 {
            break;
        }
        taken += n as u64;
        let l = l.trim_end_matches(['\n', '\r']).to_string();
        if l.starts_with('#') {
            hashes.push(l);
        } else {
            header = Some(l);
            break;
        }
    }
    if tail == 0 {
        return Some((hashes, header, String::new()));
    }
    let from = len.saturating_sub(tail).max(taken);
    r.seek(std::io::SeekFrom::Start(from)).ok()?;
    let mut buf = vec![];
    r.read_to_end(&mut buf).ok()?;
    let text = String::from_utf8_lossy(&buf).into_owned();
    // from the first whole line, unless the tail starts right after the header
    let text = if from == taken {
        text
    } else {
        text.split_once('\n')
            .map(|x| x.1.to_string())
            .unwrap_or_default()
    };
    Some((hashes, header, text))
}

/// The odometer a recording file read last, from its header and its end only.
pub fn odometer_in_file(p: &Path) -> Option<(u32, String)> {
    let (_, header, tail) = head_and_tail(p, TAIL_BYTES)?;
    odometer_in(&format!("{}\n{tail}", header?))
}

/// The newest odometer reading among `recs` (newest first; the first few that have one are looked at), with a doubt when
/// it comes from a recording that does not say which vehicle it is from while others are in the garage.
pub fn offer_from(recs: &Recordings) -> Option<Offer> {
    recs.paths.iter().take(20).find_map(|p| {
        let (miles, col) = odometer_in_file(p)?;
        Some(Offer {
            miles,
            from: format!(
                "the newest recording that read it, {} ({col})",
                file_name(p)
            ),
            doubt: recs.unmarked.contains(p).then(|| {
                "that recording does not say which vehicle it is from, and other vehicles are in the garage".into()
            }),
        })
    })
}

/// The odometer readings in module data as read (`vehicledata::read`): each value named odometer, from its number and
/// the definition's own unit (km or mi; a value of another unit, or none, is said and not offered). `read_where` is added
/// to where each came from (", read on the Modules page").
pub fn offers_in(
    vv: &crate::vehicledata::VehicleValues,
    read_where: &str,
) -> (Vec<Offer>, Vec<String>) {
    let (mut offers, mut notes) = (vec![], vec![]);
    for mv in &vv.modules {
        for (name, v, unit) in &mv.numbers {
            if !name.to_lowercase().contains("odometer") {
                continue;
            }
            match miles_of(*v, unit) {
                Some(miles) => offers.push(Offer {
                    miles,
                    from: format!("{name} ({}, the set {}{read_where})", mv.label, vv.set),
                    doubt: None,
                }),
                None => notes.push(format!(
                    "{name} ({}) read {v} {unit}: its unit is not known as km or mi, so it is not offered",
                    mv.label
                )),
            }
        }
    }
    (offers, notes)
}

fn file_name(p: &Path) -> String {
    p.file_name()
        .map(|f| f.to_string_lossy().into_owned())
        .unwrap_or_else(|| p.display().to_string())
}

/// A recording's `# vehicle:` line: the name of the garage folder of the vehicle it was recorded on.
pub fn vehicle_of(hashes: &[String]) -> Option<String> {
    hashes
        .iter()
        .find_map(|l| l.strip_prefix("# vehicle: "))
        .map(|v| v.trim().to_string())
}

/// A recording's `# began:` line: when its part began, local time.
pub fn began_of(hashes: &[String]) -> Option<When> {
    hashes
        .iter()
        .find_map(|l| l.strip_prefix("# began: "))
        .and_then(|v| When::parse(v).ok())
}

/// What a drive is, for telling two copies of it apart: its file name without `-redecoded` (`icarus record redecode`
/// writes `drive-STAMP-redecoded.csv` beside the original), and whether it is the decoded-again copy.
fn drive_key(p: &Path) -> (String, bool) {
    let stem = p
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_default();
    match stem.strip_suffix("-redecoded") {
        Some(s) => (s.to_string(), true),
        None => (stem, false),
    }
}

/// One copy of each drive, in the order given: a file whose name was already listed (the same drive in two folders) is
/// left out, and a drive decoded again (`-redecoded`) stands in for its original, where the original was.
pub fn one_copy_each(paths: &[PathBuf]) -> Vec<PathBuf> {
    let mut out: Vec<PathBuf> = vec![];
    let mut at: BTreeMap<String, (usize, bool)> = BTreeMap::new();
    for p in paths {
        let (key, redecoded) = drive_key(p);
        match at.get(&key) {
            Some(&(i, false)) if redecoded => {
                out[i] = p.clone();
                at.insert(key, (i, true));
            }
            Some(_) => {}
            None => {
                at.insert(key, (out.len(), redecoded));
                out.push(p.clone());
            }
        }
    }
    out
}

/// The recordings that may be one vehicle's (`recordings_for`).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Recordings {
    /// Newest first, one copy of each drive.
    pub paths: Vec<PathBuf>,
    /// Those in the shared folder that name no vehicle (older recordings) while other vehicles are in the garage: counted,
    /// but they could be another vehicle's.
    pub unmarked: Vec<PathBuf>,
    /// How many were left out as another vehicle's (their `# vehicle:` line names another vehicle's folder).
    pub others: usize,
    pub note: Option<String>,
}

/// The recordings that may be this vehicle's: its folder's Logs, and the shared folder `shared` (~/Documents/Icarus/logs,
/// where the recorder writes). A recording whose `# vehicle:` line names another vehicle's folder in `lib` is left out; one
/// with no such line (recorded before the line was written) is counted, and when other vehicles are in the garage the note
/// says so. One copy of each drive (`one_copy_each`, this vehicle's own folder first), newest first.
pub fn recordings_for(lib: &Path, dir: &Path, shared: &Path) -> Recordings {
    let other_folders: Vec<String> = crate::garage::vehicles_in(lib)
        .iter()
        .filter(|(_, _, d)| d != dir)
        .filter_map(|(_, _, d)| d.file_name().map(|f| f.to_string_lossy().into_owned()))
        .collect();
    let own = drivelog::list(&[dir.join("Logs")]);
    let common = drivelog::list(&[shared.to_path_buf()]);
    let mut recs = Recordings::default();
    let mut kept = vec![];
    for (p, in_shared) in own
        .iter()
        .map(|p| (p, false))
        .chain(common.iter().map(|p| (p, true)))
    {
        let hashes = head_and_tail(p, 0).map(|h| h.0).unwrap_or_default();
        match vehicle_of(&hashes) {
            Some(v) if other_folders.contains(&v) => {
                recs.others += 1;
                continue;
            }
            Some(_) => {}
            None if in_shared && !other_folders.is_empty() => recs.unmarked.push(p.clone()),
            None => {}
        }
        kept.push(p.clone());
    }
    let mut paths = one_copy_each(&kept);
    let modified = |p: &PathBuf| std::fs::metadata(p).and_then(|m| m.modified()).ok();
    paths.sort_by_key(|p| std::cmp::Reverse(modified(p)));
    recs.unmarked.retain(|p| paths.contains(p));
    let mut notes = vec![];
    if !recs.unmarked.is_empty() {
        let n = other_folders.len();
        notes.push(format!(
            "{} of the recordings in {} do not say which vehicle they are from, and {n} other vehicle{} {} a folder in \
             the garage: they are counted here (move another vehicle's drives into its own Logs folder to keep them out)",
            recs.unmarked.len(),
            shared.display(),
            if n == 1 { "" } else { "s" },
            if n == 1 { "has" } else { "have" }
        ));
    }
    if recs.others > 0 {
        notes.push(format!(
            "{} recording{} of other vehicles left out",
            recs.others,
            if recs.others == 1 { "" } else { "s" }
        ));
    }
    recs.paths = paths;
    recs.note = (!notes.is_empty()).then(|| notes.join("; "));
    recs
}

/// Local time `secs` seconds after 1970 as a `When`.
fn local(secs: i64) -> When {
    let t = secs as libc::time_t;
    let tm = unsafe {
        let mut tm: libc::tm = std::mem::zeroed();
        libc::localtime_r(&t, &mut tm);
        tm
    };
    When {
        day: days_from_civil(
            tm.tm_year as i64 + 1900,
            (tm.tm_mon + 1) as u32,
            tm.tm_mday as u32,
        ),
        minute: Some((tm.tm_hour * 60 + tm.tm_min) as u32),
    }
}

/// The recorder's stamp in a file name (`drive-2026-11-27_183005.csv`), when the file is a drive's first part: a later
/// part (`-part2`) carries the first part's stamp, not its own beginning.
fn stamp_in_name(name: &str) -> Option<When> {
    let b = name.as_bytes();
    (0..b.len().saturating_sub(16)).find_map(|i| {
        let s = name.get(i..i + 17)?;
        let (d, t) = (s.get(..10)?, s.get(11..)?);
        if !matches!(b[i + 10], b'_' | b'T' | b' ') || !t.bytes().all(|c| c.is_ascii_digit()) {
            return None;
        }
        let after = name.get(i + 17..).unwrap_or("");
        if after
            .strip_prefix("-part")
            .is_some_and(|n| n.starts_with(|c: char| c.is_ascii_digit()))
        {
            return None;
        }
        let w = When::parse(d).ok()?;
        let (h, m) = (t[..2].parse::<u32>().ok()?, t[2..4].parse::<u32>().ok()?);
        (h < 24 && m < 60).then_some(When {
            day: w.day,
            minute: Some(h * 60 + m),
        })
    })
}

/// When a recording began, read from its first lines and its end only: its `# began:` line (the recorder writes one in
/// every part), else the stamp in its name when it is a drive's first part, else when the file was last written less its
/// last row's time (the file is written to its end, and `time_s` counts from its start).
pub fn recording_start(p: &Path) -> Option<When> {
    let (hashes, _, tail) = head_and_tail(p, 4096).unwrap_or_default();
    if let Some(w) = began_of(&hashes) {
        return Some(w);
    }
    if let Some(w) = stamp_in_name(&file_name(p)) {
        return Some(w);
    }
    let modified = std::fs::metadata(p).ok()?.modified().ok()?;
    let end = modified
        .duration_since(std::time::UNIX_EPOCH)
        .ok()?
        .as_secs_f64();
    let last_t = tail
        .lines()
        .rev()
        .find_map(|l| l.split(',').next()?.trim().parse::<f64>().ok())
        .filter(|t| t.is_finite() && *t >= 0.0)
        .unwrap_or(0.0);
    Some(local((end - last_t) as i64))
}

/// What the time of a service logged at `when` says about the drives recorded that day (`recordings`): with a time, how
/// many began after it (they count as after) and before it; with none, that the day's drives are on neither side.
pub fn day_note(when: When, recordings: &[PathBuf]) -> Option<String> {
    let starts: Vec<When> = one_copy_each(recordings)
        .iter()
        .filter_map(|p| recording_start(p))
        .filter(|s| s.day == when.day)
        .collect();
    if starts.is_empty() {
        return None;
    }
    let date = date_text(when.day);
    Some(match when.minute {
        Some(_) => {
            let after = starts.iter().filter(|s| **s >= when).count();
            let before = starts.len() - after;
            let t = when.text();
            let time = t.rsplit(' ').next().unwrap_or("");
            format!(
                "{} on {date} began after {time}: {} as after it; {} began before it",
                drives_text(after),
                if after == 1 { "it counts" } else { "they count" },
                before
            )
        }
        None => format!(
            "{} recorded on {date}: with no time given, a before and after leaves them out (give the time: \
             --date \"{date} 14:30\")",
            drives_text(starts.len())
        ),
    })
}

/// A moment in the log to compare around: its name ("spark plugs"), when, and the services done then.
#[derive(Clone, Debug, PartialEq)]
pub struct Event {
    pub name: String,
    pub when: When,
    pub services: Vec<Service>,
}

impl Event {
    /// "spark plugs, 2026-11-29 14:30 at 84,213 mi (NGK)".
    pub fn title(&self) -> String {
        if self.services.is_empty() {
            return self.when.text();
        }
        let s: Vec<String> = self.services.iter().map(Service::line).collect();
        s.join("; ")
    }
}

/// The event `what` names in the log: a kind (its latest service), or a date ("2026-11-29", the services done that day;
/// a date with no service is compared around too, named by the date).
pub fn event(log: &Log, what: &str) -> Result<Event, String> {
    if let Ok(k) = Kind::parse(what) {
        let s = log.last(k).ok_or_else(|| {
            format!(
                "no {} logged for this vehicle yet: icarus garage service add {} --miles N --date YYYY-MM-DD",
                k.name(),
                k.key()
            )
        })?;
        return Ok(Event {
            name: k.name().to_string(),
            when: s.when().ok_or("its date does not read")?,
            services: vec![s.clone()],
        });
    }
    let w = When::parse(what)
        .map_err(|_| format!("{what:?} is neither a kind of service nor a date (2026-11-29)"))?;
    let services: Vec<Service> = log
        .services
        .iter()
        .filter(|s| {
            s.when()
                .is_some_and(|sw| sw.day == w.day && (w.minute.is_none() || sw.minute == w.minute))
        })
        .cloned()
        .collect();
    // the time the day's service was given, when one was and none was typed here
    let when = match w.minute {
        Some(_) => w,
        None => services
            .iter()
            .filter_map(Service::when)
            .find(|sw| sw.minute.is_some())
            .unwrap_or(w),
    };
    let name = if services.is_empty() {
        w.text()
    } else {
        let mut kinds: Vec<&str> = services.iter().map(|s| s.kind.name()).collect();
        kinds.dedup();
        kinds.join(", ")
    };
    Ok(Event {
        name,
        when,
        services,
    })
}

/// One misfire count on one side: how much it rose over the drives that carry it, and how long those drives were.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rise {
    pub count: f64,
    pub seconds: f64,
    pub drives: usize,
}

/// One misfire count before and after (None: that side's drives do not carry it).
#[derive(Clone, Debug, PartialEq)]
pub struct Misfire {
    pub name: String,
    pub before: Option<Rise>,
    pub after: Option<Rise>,
}

/// The drives before a service against the drives after it (ICA-215).
#[derive(Clone, Debug, PartialEq)]
pub struct BeforeAfter {
    pub event: Event,
    /// The drives on each side, in time order.
    pub before: Vec<PathBuf>,
    pub after: Vec<PathBuf>,
    /// How many drives each side was to have.
    pub drives: usize,
    /// Drives on the service's day itself, left out: the service has no time to place them by.
    pub same_day: Vec<PathBuf>,
    /// Files left out, and why (not a recording, could not be read).
    pub skipped: Vec<String>,
    /// The comparison: recording 1 is the drives before, 2 the drives after.
    pub comparison: compare::Comparison,
    pub misfires: Vec<Misfire>,
    /// Seconds recorded on each side.
    pub seconds: (f64, f64),
    /// A note on where the recordings came from (`recordings_for`).
    pub note: Option<String>,
}

/// Compares the last `drives` recordings before `event` with the first `drives` after it, by when each recording began
/// (`recording_start`, from its first lines and its end only) against the event's date (and time, when it has one:
/// without it, the drives on its day itself are left out, since they could be on either side). Only the drives chosen are
/// read whole; one that does not read is left out, said, and the next one taken. One copy of each drive
/// (`one_copy_each`). A side with no drive is an error saying so.
pub fn before_after(
    event: &Event,
    recordings: &[PathBuf],
    drives: usize,
    min_rows: usize,
) -> Result<BeforeAfter, String> {
    let drives = drives.max(1);
    let mut before: Vec<(When, PathBuf)> = vec![];
    let mut after: Vec<(When, PathBuf)> = vec![];
    let (mut same_day, mut skipped) = (vec![], vec![]);
    for p in one_copy_each(recordings) {
        let Some(start) = recording_start(&p) else {
            skipped.push(format!("{}: when it began is not known", file_name(&p)));
            continue;
        };
        let side = match event.when.minute {
            Some(_) => Some(start < event.when),
            None if start.day == event.when.day => None,
            None => Some(start.day < event.when.day),
        };
        match side {
            Some(true) => before.push((start, p)),
            Some(false) => after.push((start, p)),
            None => same_day.push(p),
        }
    }
    // nearest first on each side; only as many read as are kept
    before.sort_by_key(|a| std::cmp::Reverse(a.0));
    after.sort_by_key(|a| a.0);
    let mut take = |v: Vec<(When, PathBuf)>| -> Vec<(When, PathBuf, String)> {
        let mut out = vec![];
        for (w, p) in v {
            if out.len() == drives {
                break;
            }
            match std::fs::read_to_string(&p)
                .map_err(|e| e.to_string())
                .and_then(|t| drivelog::series(&t).map(|_| t))
            {
                Ok(t) => out.push((w, p, t)),
                Err(e) => skipped.push(format!("{}: {e}", file_name(&p))),
            }
        }
        out
    };
    let mut before = take(before);
    before.reverse();
    let after = take(after);
    same_day.sort();
    let when = event.when.text();
    let day_note = if same_day.is_empty() {
        String::new()
    } else {
        format!(
            " ({} on {} itself left out: give the service its time to place them)",
            same_day.len(),
            date_text(event.when.day)
        )
    };
    if before.is_empty() {
        return Err(format!(
            "no recordings before {} ({when}) to compare with{day_note}",
            event.name
        ));
    }
    if after.is_empty() {
        return Err(format!(
            "no recordings after {} ({when}) yet: record a drive or two, then compare again{day_note}",
            event.name
        ));
    }
    let side = |v: &[(When, PathBuf, String)]| -> Vec<(String, String)> {
        v.iter()
            .map(|(_, p, t)| (file_name(p), t.clone()))
            .collect()
    };
    let comparison = compare::compare_pooled(
        &[
            (format!("Before {}", event.name), side(&before)),
            (format!("After {}", event.name), side(&after)),
        ],
        min_rows,
    )?;
    // each misfire count over the drives on its side that carry it, with their own length
    let mut counts: BTreeMap<String, [Option<Rise>; 2]> = BTreeMap::new();
    let mut seconds = [0.0, 0.0];
    for (i, v) in [&before, &after].into_iter().enumerate() {
        for (_, _, t) in v {
            let (m, secs) = compare::misfires(t)?;
            seconds[i] += secs;
            for (name, rise) in m {
                let slot = &mut counts.entry(name).or_default()[i];
                let r = slot.get_or_insert(Rise {
                    count: 0.0,
                    seconds: 0.0,
                    drives: 0,
                });
                r.count += rise;
                r.seconds += secs;
                r.drives += 1;
            }
        }
    }
    Ok(BeforeAfter {
        event: event.clone(),
        before: before.into_iter().map(|x| x.1).collect(),
        after: after.into_iter().map(|x| x.1).collect(),
        drives,
        same_day,
        skipped,
        comparison,
        misfires: counts
            .into_iter()
            .map(|(name, [before, after])| Misfire {
                name,
                before,
                after,
            })
            .collect(),
        seconds: (seconds[0], seconds[1]),
        note: None,
    })
}

fn hours(s: f64) -> String {
    if s < 3600.0 {
        format!("{:.0} min", s / 60.0)
    } else {
        format!("{:.1} h", s / 3600.0)
    }
}

impl BeforeAfter {
    /// Which drives each side holds, in words.
    pub fn sides(&self) -> Vec<String> {
        let names = |v: &[PathBuf]| {
            v.iter()
                .map(|p| file_name(p))
                .collect::<Vec<_>>()
                .join(", ")
        };
        let short = |n: usize| {
            if n < self.drives {
                format!(" (only {n} of the {} asked for)", self.drives)
            } else {
                String::new()
            }
        };
        let mut v = vec![
            format!(
                "Before: the last {} before it{}: {}",
                drives_text(self.before.len()),
                short(self.before.len()),
                names(&self.before)
            ),
            format!(
                "After: the first {} after it{}: {}",
                drives_text(self.after.len()),
                short(self.after.len()),
                names(&self.after)
            ),
        ];
        if !self.same_day.is_empty() {
            v.push(format!(
                "{} on {} itself left out ({}): give the service its time (date = \"{} 14:30\" in {FILE}) to place them",
                drives_text(self.same_day.len()),
                date_text(self.event.when.day),
                names(&self.same_day),
                date_text(self.event.when.day)
            ));
        }
        for s in &self.skipped {
            v.push(format!("Left out: {s}"));
        }
        if let Some(n) = &self.note {
            v.push(format!("Note: {n}"));
        }
        v
    }

    /// A side with fewer rows at matching conditions than the minimum, said plainly (nothing on it can be summarized).
    pub fn thin_sides(&self) -> Vec<String> {
        let c = &self.comparison;
        c.recordings
            .iter()
            .filter(|r| r.kept < c.min_rows)
            .map(|r| {
                format!(
                    "{} has only {} row{} at matching conditions, fewer than the minimum of {}: nothing on that side \
                     can be summarized (record more warm driving, or ask for more drives)",
                    r.name,
                    r.kept,
                    if r.kept == 1 { "" } else { "s" },
                    c.min_rows
                )
            })
            .collect()
    }

    /// The misfire counts before and after, in lines (each over the drives that carry it, and their length); one line
    /// saying so when the recordings carry none.
    pub fn misfire_lines(&self) -> Vec<String> {
        if self.misfires.is_empty() {
            return vec!["The recordings carry no misfire counts (a module-data misfire count, recorded with the \
                         vehicle's module data, would be compared here)."
                .into()];
        }
        let side = |v: Option<Rise>| match v {
            Some(r) => format!(
                "{:.0} over {} ({})",
                r.count,
                hours(r.seconds),
                drives_text(r.drives)
            ),
            None => "not recorded".into(),
        };
        self.misfires
            .iter()
            .map(|m| {
                format!(
                    "{}: before {}, after {}",
                    m.name,
                    side(m.before),
                    side(m.after)
                )
            })
            .collect()
    }

    /// The whole of it in words, the table under the event's name; `warm` is the coolant temperature as shown.
    pub fn words(&self, warm: &str) -> String {
        let mut o = format!(
            "Before and after {}: {}\n",
            self.event.name,
            self.event.title()
        );
        for l in self.sides() {
            o += &format!("{l}\n");
        }
        for l in self.thin_sides() {
            o += &format!("{l}\n");
        }
        o += "\n";
        o += &compare::words(&self.comparison, warm);
        o += "\nMisfire counts (how much each misfire count rose over the drives that carry it):\n";
        for l in self.misfire_lines() {
            o += &format!("  {l}\n");
        }
        o
    }
}

fn drives_text(n: usize) -> String {
    format!("{n} drive{}", if n == 1 { "" } else { "s" })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(tag: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!(
            "icarus-maintenance-{tag}-{}-{:?}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    fn day(s: &str) -> i64 {
        When::parse(s).unwrap().day
    }

    #[test]
    fn dates_and_months() {
        assert_eq!(days_from_civil(1970, 1, 1), 0);
        assert_eq!(
            civil_from_days(days_from_civil(2026, 11, 29)),
            (2026, 11, 29)
        );
        assert_eq!(date_text(add_months(day("2026-08-31"), 6)), "2027-02-28");
        assert_eq!(date_text(add_months(day("2027-08-31"), 6)), "2028-02-29");
        assert_eq!(date_text(add_months(day("2026-11-29"), 14)), "2028-01-29");
        let w = When::parse("2026-11-29 14:30").unwrap();
        assert_eq!((w.minute, w.text()), (Some(870), "2026-11-29 14:30".into()));
        assert_eq!(
            When::parse("2026-11-29T07:05").unwrap().text(),
            "2026-11-29 07:05"
        );
        for bad in [
            "2026-02-30",
            "11/29/2026",
            "2026-13-01",
            "2026-11-29 25:00",
            "",
        ] {
            assert!(When::parse(bad).is_err(), "{bad}");
        }
        assert_eq!(miles_text(84_213), "84,213 mi");
        assert_eq!(miles_text(500), "500 mi");
        assert_eq!(miles_text(1_000_000), "1,000,000 mi");
    }

    #[test]
    fn kinds_as_typed() {
        for (s, k) in [
            ("oil", Kind::Oil),
            ("Oil and filter", Kind::Oil),
            ("spark plugs", Kind::SparkPlugs),
            ("spark_plugs", Kind::SparkPlugs),
            ("plugs", Kind::SparkPlugs),
            ("a sensor", Kind::Sensor),
            ("fuel-change", Kind::Fuel),
            ("rotation", Kind::TireRotation),
        ] {
            assert_eq!(Kind::parse(s), Ok(k), "{s}");
        }
        for k in Kind::ALL {
            assert_eq!(Kind::parse(k.key()), Ok(k));
        }
        assert!(Kind::parse("wiper blades")
            .unwrap_err()
            .contains("spark-plugs"));
    }

    #[test]
    fn services_kept_in_a_plain_file_and_read_back() {
        let dir = scratch("file");
        assert_eq!(load(&dir).unwrap(), Log::default(), "no file yet: empty");
        let (s, w) = add(
            &dir,
            Service::new(
                Kind::Oil,
                "2026-05-01",
                Some(80_000),
                "  0W-20   synthetic ",
            ),
        )
        .unwrap();
        assert!(w.is_empty());
        assert_eq!(s.note, "0W-20 synthetic");
        add(
            &dir,
            Service::new(Kind::SparkPlugs, "2026-11-29T14:30", Some(84_213), ""),
        )
        .unwrap();
        // logged out of order: the file keeps date order; a lower odometer than an earlier one is said
        let (_, w) = add(
            &dir,
            Service::new(Kind::AirFilter, "2026-06-01", Some(8_100), ""),
        )
        .unwrap();
        assert!(w[0].contains("less than the 80,000 mi"), "{w:?}");
        assert!(
            add(&dir, Service::new(Kind::Other, "2026-06-01", None, " "),)
                .unwrap_err()
                .contains("note")
        );
        assert!(add(&dir, Service::new(Kind::Oil, "June 1", None, ""),).is_err());
        set_interval(&dir, Kind::Oil, Some(5000), Some(6)).unwrap();
        set_interval(&dir, Kind::Brakes, Some(30_000), None).unwrap();
        set_interval(&dir, Kind::Brakes, None, None).unwrap();
        let text = std::fs::read_to_string(path(&dir)).unwrap();
        assert!(
            text.starts_with("# Icarus: this vehicle's maintenance log"),
            "{text}"
        );
        assert!(
            text.contains("kind = \"spark-plugs\"") && text.contains("date = \"2026-11-29 14:30\"")
        );
        assert!(
            text.contains("[intervals.oil]") && !text.contains("[intervals.brakes]"),
            "{text}"
        );
        let log = load(&dir).unwrap();
        let dates: Vec<&str> = log.services.iter().map(|s| s.date.as_str()).collect();
        assert_eq!(dates, ["2026-05-01", "2026-06-01", "2026-11-29 14:30"]);
        assert_eq!(
            log.interval(Kind::Oil),
            Some(Interval::new(Some(5000), Some(6)))
        );
        // edited by hand: read as written; a mistake in it is said, never taken as an empty log
        std::fs::write(
            path(&dir),
            "[intervals.spark-plugs]\nmiles = 60000\n\n[[service]]\nkind = \"fuel\"\ndate = \"2026-10-01\"\nnote = \"91 E0\"\n",
        )
        .unwrap();
        let log = load(&dir).unwrap();
        assert_eq!(log.services[0].what(), "a fuel change (91 E0)");
        assert_eq!(log.interval(Kind::SparkPlugs).unwrap().miles, Some(60_000));
        std::fs::write(
            path(&dir),
            "[[service]]\nkind = \"wipers\"\ndate = \"2026-10-01\"\n",
        )
        .unwrap();
        assert!(load(&dir).unwrap_err().contains(FILE));
        std::fs::write(
            path(&dir),
            "[[service]]\nkind = \"oil\"\ndate = \"10/01/2026\"\n",
        )
        .unwrap();
        assert!(load(&dir).unwrap_err().contains("not a date"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn due_by_miles_and_by_months() {
        let mut log = Log::default();
        let svc = |k, date: &str, miles| Service::new(k, date, miles, "");
        log.services = vec![
            svc(Kind::Oil, "2026-01-10", Some(70_000)),
            svc(Kind::Oil, "2026-05-01", Some(80_000)),
            svc(Kind::AirFilter, "2025-12-15", Some(60_000)),
            svc(Kind::SparkPlugs, "2026-11-29", None),
            svc(Kind::Brakes, "2026-01-01", None),
        ];
        let iv = Interval::new;
        log.intervals.insert("oil".into(), iv(Some(5000), Some(6)));
        log.intervals
            .insert("air-filter".into(), iv(None, Some(12)));
        log.intervals
            .insert("spark-plugs".into(), iv(Some(120_000), None));
        log.intervals
            .insert("tire-rotation".into(), iv(Some(5000), None));
        log.intervals
            .insert("brakes".into(), iv(Some(30_000), Some(24)));
        let find = |d: &[Due], k| d.iter().find(|x| x.kind == k).unwrap().clone();

        // 2026-08-01 at 84,600 mi: oil due at 85,000 (400 left: soon) and by 2026-11-01; the air filter by 2026-12-15
        let d = due(&log, day("2026-08-01"), Some(84_600));
        let oil = find(&d, Kind::Oil);
        assert_eq!(
            oil.last.as_ref().unwrap().date,
            "2026-05-01",
            "the latest oil change counts"
        );
        assert_eq!((oil.at_miles, oil.miles_left), (Some(85_000), Some(400)));
        assert_eq!(oil.by_day.map(date_text).as_deref(), Some("2026-11-01"));
        assert_eq!(oil.days_left, Some(92));
        assert_eq!(oil.state, State::Soon, "400 mi is within {SOON_MILES}");
        let air = find(&d, Kind::AirFilter);
        assert_eq!((air.at_miles, air.state), (None, State::Later));
        assert_eq!(air.by_day.map(date_text).as_deref(), Some("2026-12-15"));
        assert_eq!(find(&d, Kind::TireRotation).state, State::NeverLogged);
        let plugs = find(&d, Kind::SparkPlugs);
        assert_eq!(
            plugs.state,
            State::Unknown,
            "by miles only, none logged with it"
        );
        assert!(
            plugs.text().contains("by miles not known"),
            "{}",
            plugs.text()
        );
        assert_eq!(d[0].kind, Kind::Oil, "the most pressing first");
        assert!(
            oil.text().contains("in 400 mi (at 85,000 mi)"),
            "{}",
            oil.text()
        );

        // by miles: past 85,000 is overdue, whatever the date
        let oil = find(&due(&log, day("2026-08-01"), Some(85_300)), Kind::Oil);
        assert_eq!((oil.state, oil.miles_left), (State::Overdue, Some(-300)));
        assert!(oil.text().contains("overdue by 300 mi"), "{}", oil.text());
        // by months: few miles driven, but six months gone (on the day itself it is due)
        let d = due(&log, day("2026-11-01"), Some(81_000));
        let oil = find(&d, Kind::Oil);
        assert_eq!((oil.state, oil.days_left), (State::Overdue, Some(0)));
        let d = due(&log, day("2026-10-15"), Some(81_000));
        assert_eq!(find(&d, Kind::Oil).state, State::Soon, "17 days left");
        assert_eq!(find(&d, Kind::Oil).days_left, Some(17));
        let d = due(&log, day("2026-06-01"), Some(81_000));
        assert_eq!(find(&d, Kind::Oil).state, State::Later);
        // the odometer now not known: due by months still, the miles it is due at given
        let oil = find(&due(&log, day("2026-06-01"), None), Kind::Oil);
        assert_eq!((oil.miles_left, oil.state), (None, State::Later));
        assert!(oil
            .text()
            .contains("at 85,000 mi (the odometer now is not known)"));
        // a year past the air filter, two past the brakes
        let d = due(&log, day("2028-01-02"), Some(90_000));
        assert_eq!(find(&d, Kind::AirFilter).state, State::Overdue);
        assert_eq!(find(&d, Kind::Brakes).state, State::Overdue);
        assert!(find(&d, Kind::AirFilter)
            .text()
            .contains("overdue since 2026-12-15"));
        // no interval: not listed
        log.intervals.clear();
        assert!(due(&log, day("2026-08-01"), Some(84_600)).is_empty());
    }

    #[test]
    fn an_odometer_is_offered_only_in_a_known_unit() {
        assert_eq!(miles_of(135_528.0, "km"), Some(84_213));
        assert_eq!(miles_of(84_213.0, "mi"), Some(84_213));
        assert_eq!(
            miles_of(84_213.0, ""),
            None,
            "an unchecked value is never taken"
        );
        let rec = "# adapter: made-up\ntime_s,\"Engine speed [rpm]\",\"Odometer [km]\",\"Odometer (7E0 22 0001) [mi]\",\"Odometer raw (7E0 22 0002)\"\n\
                   0,700,,84210,99\n1,700,135528.0,,99\n2,700,,84213,99\n";
        let (m, col) = odometer_in(rec).unwrap();
        assert_eq!((m, col.as_str()), (84_213, "Odometer (7E0 22 0001) [mi]"));
        assert_eq!(odometer_in("time_s,\"Engine speed [rpm]\"\n0,700\n"), None);
    }

    /// A made-up recording: `n` warm, closed-loop rows at 2200 rpm, 70 % load, 20 g/s, with this timing and bank 1's
    /// short-term trim, and a misfire counter rising by `misfires` over it when given.
    fn drive(n: usize, timing: f64, st1: f64, misfires: Option<f64>) -> String {
        let mut s = String::from(
            "# adapter: made-up adapter\ntime_s,\"Engine speed [rpm]\",\"Engine load [%]\",\"Timing advance [° before TDC]\",\
             \"Mass air flow [g/s]\",\"Short-term fuel trim, bank 1 [%]\",\"Long-term fuel trim, bank 1 [%]\",\
             \"Coolant temperature [°C]\",\"Commanded lambda [λ]\"",
        );
        if misfires.is_some() {
            s.push_str(",\"Cylinder #3 misfire count (7E0 21 01) []\"");
        }
        s.push('\n');
        for i in 0..n {
            s.push_str(&format!(
                "{:.1},2200,70,{timing:.1},20,{st1:.1},-4.0,90,1.00",
                i as f64 * 0.5
            ));
            if let Some(m) = misfires {
                s.push_str(&format!(",{:.0}", (m * i as f64 / (n - 1) as f64).floor()));
            }
            s.push('\n');
        }
        s
    }

    #[test]
    fn a_service_between_two_recordings_compares_them_under_its_name() {
        let home = scratch("before-after");
        let logs = home.join("Logs");
        std::fs::create_dir_all(&logs).unwrap();
        // two drives before the plugs (the older one left out with --drives 1), one the same day, two after
        let files = [
            (
                "drive-2026-11-20_080000.csv",
                drive(12, 21.0, 4.0, Some(6.0)),
            ),
            (
                "drive-2026-11-27_183005.csv",
                drive(12, 20.0, 3.0, Some(4.0)),
            ),
            ("drive-2026-11-29_090000.csv", drive(12, 99.0, 9.0, None)),
            (
                "drive-2026-11-30_071500.csv",
                drive(12, 25.0, 1.0, Some(0.0)),
            ),
            (
                "drive-2026-12-01_071500.csv",
                drive(4, 25.0, 1.0, Some(0.0)),
            ),
            (
                "drive-2026-11-28_120000.csv",
                "not a recording\n".to_string(),
            ),
        ];
        for (n, t) in &files {
            std::fs::write(logs.join(n), t).unwrap();
        }
        let dir = home.clone();
        add(
            &dir,
            Service::new(Kind::SparkPlugs, "2026-11-29", Some(84_213), "NGK"),
        )
        .unwrap();
        let log = load(&dir).unwrap();
        let ev = event(&log, "spark-plugs").unwrap();
        assert_eq!(ev.name, "spark plugs");
        assert_eq!(event(&log, "2026-11-29").unwrap().name, "spark plugs");
        assert_eq!(
            event(&log, "2026-10-01").unwrap().name,
            "2026-10-01",
            "a bare date"
        );
        assert!(event(&log, "oil")
            .unwrap_err()
            .contains("no oil and filter logged"));
        let recs = drivelog::list(std::slice::from_ref(&logs));
        let ba = before_after(&ev, &recs, 1, compare::MIN_ROWS).unwrap();
        let names = |v: &[PathBuf]| v.iter().map(|p| file_name(p)).collect::<Vec<_>>();
        assert_eq!(names(&ba.before), ["drive-2026-11-27_183005.csv"]);
        assert_eq!(names(&ba.after), ["drive-2026-11-30_071500.csv"]);
        assert_eq!(
            names(&ba.same_day),
            ["drive-2026-11-29_090000.csv"],
            "no time given: on neither side"
        );
        assert!(
            ba.skipped
                .iter()
                .any(|s| s.starts_with("drive-2026-11-28_120000.csv")),
            "{:?}",
            ba.skipped
        );
        let c = &ba.comparison;
        assert_eq!(c.recordings[0].name, "Before spark plugs");
        assert_eq!(c.recordings[1].name, "After spark plugs");
        let g = c
            .timing
            .iter()
            .find(|g| g.label == "1800-2600 rpm, 60-80 %")
            .unwrap();
        assert_eq!(
            c.difference(g, 1, 0),
            Some(5.0),
            "5 degrees more timing after"
        );
        let air = c.trims.iter().find(|g| g.label == "15-30 g/s").unwrap();
        assert_eq!(
            c.difference(air, 1, 0),
            Some(-2.0),
            "bank 1 2 % leaner after"
        );
        assert!(ba.thin_sides().is_empty());
        assert_eq!(
            ba.misfires,
            [Misfire {
                name: "Cylinder #3 misfire count (7E0 21 01) []".into(),
                before: Some(Rise {
                    count: 4.0,
                    seconds: 5.5,
                    drives: 1
                }),
                after: Some(Rise {
                    count: 0.0,
                    seconds: 5.5,
                    drives: 1
                })
            }]
        );
        let w = ba.words("176 °F (80 °C)");
        assert!(
            w.starts_with(
                "Before and after spark plugs: 2026-11-29 at 84,213 mi: spark plugs (NGK)"
            ),
            "{w}"
        );
        assert!(
            w.contains("1. Before spark plugs") && w.contains("2. After spark plugs"),
            "{w}"
        );
        assert!(w.contains("+5.0°") && w.contains("before 4 over"), "{w}");
        assert!(w.contains("1 drive on 2026-11-29 itself left out"), "{w}");

        // three drives a side: both before pooled; after, only the two there are (one of 4 rows), said
        let ba = before_after(&ev, &recs, 3, compare::MIN_ROWS).unwrap();
        assert_eq!(ba.before.len(), 2);
        assert_eq!(ba.comparison.recordings[0].kept, 24);
        assert_eq!(ba.comparison.recordings[0].tag.as_deref(), Some("2 drives"));
        let g = ba
            .comparison
            .timing
            .iter()
            .find(|g| g.label == "1800-2600 rpm, 60-80 %")
            .unwrap();
        assert_eq!(
            ba.comparison.value(g, 0, 0),
            Some(20.5),
            "both drives before, pooled"
        );
        assert_eq!(
            ba.misfires[0].before,
            Some(Rise {
                count: 10.0,
                seconds: 11.0,
                drives: 2
            })
        );
        assert!(
            ba.sides()[1].contains("only 2 of the 3 asked for"),
            "{:?}",
            ba.sides()
        );

        // a minimum above what a side has: said plainly
        let ba = before_after(&ev, &recs, 1, 20).unwrap();
        let thin = ba.thin_sides();
        assert_eq!(thin.len(), 2);
        assert!(thin[0].starts_with("Before spark plugs has only 12 rows at matching conditions, fewer than the minimum of 20"), "{thin:?}");

        // with the time given, the morning's drive (09:00) falls before a 14:30 service
        let timed = Event {
            when: When::parse("2026-11-29 14:30").unwrap(),
            ..ev.clone()
        };
        let ba = before_after(&timed, &recs, 1, compare::MIN_ROWS).unwrap();
        assert_eq!(names(&ba.before), ["drive-2026-11-29_090000.csv"]);
        assert!(ba.same_day.is_empty());
        assert!(
            ba.misfire_lines()[0].contains("before not recorded"),
            "{:?}",
            ba.misfire_lines()
        );
        let none = BeforeAfter {
            misfires: vec![],
            ..ba
        };
        assert!(none.misfire_lines()[0].contains("carry no misfire counts"));

        // nothing after yet: said, not compared
        let later = Event {
            when: When::parse("2026-12-05").unwrap(),
            ..ev.clone()
        };
        assert!(before_after(&later, &recs, 3, compare::MIN_ROWS)
            .unwrap_err()
            .contains("no recordings after spark plugs"));
        let _ = std::fs::remove_dir_all(&home);
    }

    #[test]
    fn a_recording_begins_when_its_began_line_says() {
        // the name's stamp only for a drive's first part (and its copies); a later part's is the first part's
        assert_eq!(
            recording_start(Path::new("/x/drive-2026-11-27_183005.csv"))
                .unwrap()
                .text(),
            "2026-11-27 18:30"
        );
        assert_eq!(
            recording_start(Path::new("/x/drive-2026-11-27_183005-2.csv"))
                .unwrap()
                .text(),
            "2026-11-27 18:30"
        );
        assert_eq!(
            recording_start(Path::new("/x/drive-2026-11-27_183005-redecoded.csv"))
                .unwrap()
                .text(),
            "2026-11-27 18:30"
        );
        assert_eq!(
            recording_start(Path::new("/x/drive-2026-11-27_183005-part2.csv")),
            None,
            "a later part with no began line and no file: not known"
        );
        assert_eq!(recording_start(Path::new("/nowhere/at-all.csv")), None);
        let dir = scratch("began");
        // the began line first, whatever the name says
        let part2 = dir.join("drive-2026-11-27_183005-part2.csv");
        std::fs::write(
            &part2,
            format!(
                "# began: 2026-11-27 21:05:41\n{}",
                drive(12, 20.0, 1.0, None)
            ),
        )
        .unwrap();
        assert_eq!(recording_start(&part2).unwrap().text(), "2026-11-27 21:05");
        // no line, no stamp: when it was last written less its last row's time (5.5 s here, 600 s below)
        let plain = dir.join("live.csv");
        let mut text = drive(12, 20.0, 1.0, None);
        text.push_str("600.0,2200,70,20.0,20,1.0,-4.0,90,1.00\n");
        std::fs::write(&plain, text).unwrap();
        let end = 1_795_000_000i64;
        std::fs::File::options()
            .write(true)
            .open(&plain)
            .unwrap()
            .set_modified(std::time::UNIX_EPOCH + std::time::Duration::from_secs(end as u64))
            .unwrap();
        assert_eq!(recording_start(&plain), Some(local(end - 600)));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_later_part_recorded_after_the_service_is_after_it() {
        let dir = scratch("part2");
        let logs = dir.join("Logs");
        std::fs::create_dir_all(&logs).unwrap();
        // the drive began in the morning, before the plugs; it stopped for the plugs and went on at 15:10 (part 2)
        std::fs::write(
            logs.join("drive-2026-11-29_080000.csv"),
            format!(
                "# began: 2026-11-29 08:00:00\n{}",
                drive(12, 20.0, 3.0, None)
            ),
        )
        .unwrap();
        std::fs::write(
            logs.join("drive-2026-11-29_080000-part2.csv"),
            format!("# began: 2026-11-29 15:10:00\n# part 2 of the drive that began in drive-2026-11-29_080000.csv\n{}", drive(12, 25.0, 1.0, None)),
        )
        .unwrap();
        let ev = Event {
            name: "spark plugs".into(),
            when: When::parse("2026-11-29 14:30").unwrap(),
            services: vec![],
        };
        let recs = drivelog::list(std::slice::from_ref(&logs));
        let ba = before_after(&ev, &recs, 3, compare::MIN_ROWS).unwrap();
        let names = |v: &[PathBuf]| v.iter().map(|p| file_name(p)).collect::<Vec<_>>();
        assert_eq!(names(&ba.before), ["drive-2026-11-29_080000.csv"]);
        assert_eq!(names(&ba.after), ["drive-2026-11-29_080000-part2.csv"]);
        // logging it at 14:30 says which of that day's drives count after it
        let n = day_note(ev.when, &recs).unwrap();
        assert!(
            n.starts_with(
                "1 drive on 2026-11-29 began after 14:30: it counts as after it; 1 began before it"
            ),
            "{n}"
        );
        let n = day_note(When::parse("2026-11-29").unwrap(), &recs).unwrap();
        assert!(
            n.contains("2 drives recorded on 2026-11-29: with no time given"),
            "{n}"
        );
        assert_eq!(day_note(When::parse("2026-11-30").unwrap(), &recs), None);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn pooled_drives_count_each_row_once() {
        let dir = scratch("pooled");
        let logs = dir.join("Logs");
        std::fs::create_dir_all(&logs).unwrap();
        // before: 12 rows at 20° and 24 rows at 26°: the rows' mean is 24.0, not the drives' mean of 23.0
        for (n, t) in [
            (
                "drive-2026-11-20_080000.csv",
                drive(12, 20.0, 1.0, Some(2.0)),
            ),
            ("drive-2026-11-21_080000.csv", drive(24, 26.0, 1.0, None)),
            (
                "drive-2026-11-30_080000.csv",
                drive(12, 25.0, 1.0, Some(1.0)),
            ),
        ] {
            std::fs::write(logs.join(n), t).unwrap();
        }
        let ev = Event {
            name: "spark plugs".into(),
            when: When::parse("2026-11-29").unwrap(),
            services: vec![],
        };
        let ba = before_after(
            &ev,
            &drivelog::list(std::slice::from_ref(&logs)),
            3,
            compare::MIN_ROWS,
        )
        .unwrap();
        let c = &ba.comparison;
        let g = c
            .timing
            .iter()
            .find(|g| g.label == "1800-2600 rpm, 60-80 %")
            .unwrap();
        assert_eq!(c.value(g, 0, 0), Some(24.0), "row-weighted");
        assert_eq!(c.recordings[0].kept, 36);
        // the misfire count over the one drive before that carries it (5.5 s), not over both (17 s)
        let m = &ba.misfires[0];
        assert_eq!(
            m.before,
            Some(Rise {
                count: 2.0,
                seconds: 5.5,
                drives: 1
            })
        );
        assert_eq!(ba.seconds.0, 17.0, "both drives before, 5.5 + 11.5 s");
        assert!(
            ba.misfire_lines()[0].contains("before 2 over 0 min (1 drive)"),
            "{:?}",
            ba.misfire_lines()
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn one_copy_of_each_drive() {
        let p = |s: &str| PathBuf::from(s);
        let got = one_copy_each(&[
            p("/own/drive-A.csv"),
            p("/shared/drive-A.csv"),
            p("/shared/drive-A-redecoded.csv"),
            p("/shared/drive-B-part2.csv"),
            p("/own/drive-B-part2-redecoded.csv"),
            p("/shared/drive-B-part2-redecoded.csv"),
            p("/shared/drive-C.csv"),
        ]);
        assert_eq!(
            got,
            [
                p("/shared/drive-A-redecoded.csv"),
                p("/own/drive-B-part2-redecoded.csv"),
                p("/shared/drive-C.csv"),
            ]
        );
    }

    #[test]
    fn another_vehicles_drives_are_left_out_by_their_vehicle_line() {
        let home = scratch("vehicles");
        let lib = home.join("Vehicles");
        let shared = home.join("logs");
        std::fs::create_dir_all(&shared).unwrap();
        let mine = crate::garage::ensure_in(&lib, "1M8GDM9AXKP042788", Some("Old Blue")).unwrap();
        crate::garage::ensure_in(&lib, "1M8GDM9AXKP042789", Some("Grandma's Camry")).unwrap();
        let odo = |km: f64| {
            format!(
                "time_s,\"Engine speed [rpm]\",\"Odometer [km]\"\n0,700,{km:.1}\n1,700,{km:.1}\n"
            )
        };
        std::fs::write(
            shared.join("drive-2026-11-20_080000.csv"),
            format!("# vehicle: Grandma's Camry\n{}", odo(50_000.0)),
        )
        .unwrap();
        std::fs::write(
            shared.join("drive-2026-11-21_080000.csv"),
            format!("# vehicle: Old Blue\n{}", odo(135_000.0)),
        )
        .unwrap();
        std::fs::write(shared.join("drive-2026-11-22_080000.csv"), odo(135_100.0)).unwrap();
        // the same drive copied into my own Logs folder too: one copy
        std::fs::copy(
            shared.join("drive-2026-11-21_080000.csv"),
            mine.join("Logs/drive-2026-11-21_080000.csv"),
        )
        .unwrap();
        let recs = recordings_for(&lib, &mine, &shared);
        let names: Vec<String> = recs.paths.iter().map(|p| file_name(p)).collect();
        assert_eq!(names.len(), 2, "{names:?}");
        assert!(!names.contains(&"drive-2026-11-20_080000.csv".to_string()));
        assert_eq!(recs.others, 1);
        assert_eq!(recs.unmarked.len(), 1);
        assert!(recs
            .paths
            .contains(&mine.join("Logs/drive-2026-11-21_080000.csv")));
        let note = recs.note.clone().unwrap();
        assert!(
            note.contains("1 of the recordings")
                && note.contains("1 recording of other vehicles left out"),
            "{note}"
        );
        // the newest reading is from the unmarked drive: offered with its doubt
        std::fs::File::options()
            .write(true)
            .open(shared.join("drive-2026-11-22_080000.csv"))
            .unwrap()
            .set_modified(std::time::SystemTime::now() + std::time::Duration::from_secs(60))
            .unwrap();
        let recs = recordings_for(&lib, &mine, &shared);
        let o = offer_from(&recs).unwrap();
        assert_eq!(o.miles, miles_of(135_100.0, "km").unwrap());
        assert!(
            o.text().contains("does not say which vehicle"),
            "{}",
            o.text()
        );
        // with no other vehicle in the garage, nothing is in doubt
        let alone = home.join("Alone");
        let only = crate::garage::ensure_in(&alone, "1M8GDM9AXKP042788", Some("Old Blue")).unwrap();
        let recs = recordings_for(&alone, &only, &shared);
        assert_eq!(
            (recs.paths.len(), recs.unmarked.len(), recs.note.clone()),
            (3, 0, None)
        );
        assert_eq!(offer_from(&recs).unwrap().doubt, None);
        let _ = std::fs::remove_dir_all(&home);
    }

    #[test]
    fn the_owners_own_keys_are_kept() {
        let dir = scratch("keys");
        std::fs::write(
            path(&dir),
            "# my own comment\nowner = \"me\"\n\n[intervals.oil]\nmiles = 5000\nreminder = true\n\n\
             [[service]]\nkind = \"oil\"\ndate = \"2026-05-01\"\nmiles = 80000\nshop = \"Joe's\"\ncost = 45\n",
        )
        .unwrap();
        add(&dir, Service::new(Kind::Brakes, "2026-06-01", None, "pads")).unwrap();
        set_interval(&dir, Kind::Oil, Some(6000), None).unwrap();
        let text = std::fs::read_to_string(path(&dir)).unwrap();
        for k in [
            "owner = \"me\"",
            "reminder = true",
            "shop = \"Joe's\"",
            "cost = 45",
            "miles = 6000",
            "note = \"pads\"",
        ] {
            assert!(text.contains(k), "{k} in {text}");
        }
        assert!(
            !text.contains("my own comment"),
            "comments are not kept, as the file says"
        );
        assert!(text.contains("comments other than these lines are not"));
        let log = load(&dir).unwrap();
        assert_eq!(
            log.services[0]
                .extra
                .get("cost")
                .and_then(|v| v.as_integer()),
            Some(45)
        );
        // no temp file left behind, only the lock
        let left: Vec<String> = std::fs::read_dir(&dir)
            .unwrap()
            .flatten()
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .collect();
        assert!(
            left.iter().all(|n| n == FILE || n == ".maintenance.lock"),
            "{left:?}"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_due_odometer_far_past_u32_does_not_overflow() {
        let mut log = Log {
            services: vec![Service::new(
                Kind::Oil,
                "2026-05-01",
                Some(u32::MAX - 10),
                "",
            )],
            ..Log::default()
        };
        log.intervals
            .insert("oil".into(), Interval::new(Some(5000), None));
        let d = due(&log, day("2026-06-01"), Some(u32::MAX));
        assert_eq!(d[0].at_miles, Some(u32::MAX as u64 + 4990));
        assert_eq!(d[0].miles_left, Some(4990));
    }
}
