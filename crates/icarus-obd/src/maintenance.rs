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

/// One service done.
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
}

impl Service {
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
                .map(|m| format!(" at {}", miles_text(m)))
                .unwrap_or_default(),
            self.what()
        )
    }
}

/// A kind's interval: whichever comes first of its miles and its months.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct Interval {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub miles: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub months: Option<u32>,
}

impl Interval {
    /// "every 5,000 mi or 6 months".
    pub fn text(&self) -> String {
        match (self.miles, self.months) {
            (Some(mi), Some(mo)) => format!("every {} or {}", miles_text(mi), months_text(mo)),
            (Some(mi), None) => format!("every {}", miles_text(mi)),
            (None, Some(mo)) => format!("every {}", months_text(mo)),
            (None, None) => "no interval".into(),
        }
    }
}

fn months_text(m: u32) -> String {
    format!("{m} month{}", if m == 1 { "" } else { "s" })
}

/// "84,213 mi".
pub fn miles_text(m: u32) -> String {
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

/// A vehicle's log: the intervals (by kind's key) and the services, oldest first.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Log {
    #[serde(default)]
    pub intervals: BTreeMap<String, Interval>,
    #[serde(default, rename = "service")]
    pub services: Vec<Service>,
}

const HEAD: &str = "# Icarus: this vehicle's maintenance log, on this computer only. Edit it by hand as you like:\n\
# [intervals.KIND] miles = N and/or months = M (whichever comes first); each [[service]] has kind, date\n\
# (\"2026-11-29\" or \"2026-11-29 14:30\"), miles (the odometer then, optional) and note (optional; other needs one).\n\
# Kinds: oil, spark-plugs, air-filter, fluids, brakes, tire-rotation, sensor, fuel, other.\n";

impl Log {
    /// The interval set for a kind.
    pub fn interval(&self, k: Kind) -> Option<Interval> {
        self.intervals
            .get(k.key())
            .copied()
            .filter(|i| i.miles.is_some() || i.months.is_some())
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

/// Saves the log in a vehicle's folder, its services in date order.
pub fn save(dir: &Path, log: &Log) -> Result<(), String> {
    let mut log = log.clone();
    log.services.sort_by_key(|s| s.when());
    log.intervals
        .retain(|_, i| i.miles.is_some() || i.months.is_some());
    let body = toml::to_string(&log).map_err(|e| e.to_string())?;
    std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let p = path(dir);
    let tmp = p.with_extension("tmp");
    std::fs::write(&tmp, format!("{HEAD}\n{body}"))
        .and_then(|_| std::fs::rename(&tmp, &p))
        .map_err(|e| format!("{}: {e}", p.display()))
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
                miles_text(m),
                miles_text(e.miles.unwrap_or(0)),
                e.date,
                e.kind.name()
            ));
        }
    }
    log.services.push(s.clone());
    save(dir, &log)?;
    Ok((s, warnings))
}

/// Sets (or, with neither, clears) a kind's interval in the log in `dir`.
pub fn set_interval(
    dir: &Path,
    k: Kind,
    miles: Option<u32>,
    months: Option<u32>,
) -> Result<(), String> {
    if miles == Some(0) || months == Some(0) {
        return Err("an interval of 0 is no interval: leave it out, or clear it with off".into());
    }
    let mut log = load(dir)?;
    if miles.is_none() && months.is_none() {
        log.intervals.remove(k.key());
    } else {
        log.intervals
            .insert(k.key().to_string(), Interval { miles, months });
    }
    save(dir, &log)
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
    pub at_miles: Option<u32>,
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
            let at_miles = interval.miles.zip(l.miles).map(|(i, m)| m + i);
            let by_day = interval
                .months
                .zip(l.when())
                .map(|(mo, w)| add_months(w.day, mo));
            let miles_left = at_miles
                .zip(odometer)
                .map(|(at, now)| at as i64 - now as i64);
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
                miles_text((-left) as u32),
                miles_text(at)
            )),
            (Some(at), Some(left)) => parts.push(format!(
                "in {} (at {})",
                miles_text(left as u32),
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
                .map(|m| format!(" at {}", miles_text(m)))
                .unwrap_or_default()
        )
    }
}

/// An odometer reading offered for a service: the miles and where they came from, for the owner to confirm.
#[derive(Clone, Debug, PartialEq)]
pub struct Offer {
    pub miles: u32,
    pub from: String,
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
    Some(m.round() as u32)
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

/// The newest odometer reading among `recordings` (newest first; the first few that have one are looked at).
pub fn offer_from_recordings(recordings: &[PathBuf]) -> Option<Offer> {
    recordings.iter().take(20).find_map(|p| {
        let text = std::fs::read_to_string(p).ok()?;
        let (miles, col) = odometer_in(&text)?;
        Some(Offer {
            miles,
            from: format!(
                "the newest recording that read it, {} ({col})",
                file_name(p)
            ),
        })
    })
}

fn file_name(p: &Path) -> String {
    p.file_name()
        .map(|f| f.to_string_lossy().into_owned())
        .unwrap_or_else(|| p.display().to_string())
}

/// The recordings that may be this vehicle's: its folder's Logs, and the shared folder `shared` (~/Documents/Icarus/logs,
/// where the recorder writes; its files do not say which vehicle they are from). Newest first. With a note when other
/// vehicles have folders in `lib` too, since the shared folder's drives could be theirs.
pub fn recordings_for(lib: &Path, dir: &Path, shared: &Path) -> (Vec<PathBuf>, Option<String>) {
    let list = drivelog::list(&[dir.join("Logs"), shared.to_path_buf()]);
    let others = crate::garage::vehicles_in(lib)
        .iter()
        .filter(|(_, _, d)| d != dir)
        .count();
    let note = (others > 0).then(|| {
        format!(
            "the recordings in {} do not say which vehicle they are from, and {others} other vehicle{} {} a folder in \
             the garage: move another vehicle's drives into its own Logs folder to keep them out",
            shared.display(),
            if others == 1 { "" } else { "s" },
            if others == 1 { "has" } else { "have" }
        )
    });
    (list, note)
}

/// When a recording began: from its name (`drive-2026-11-27_183005.csv`, as the recorder names it), else when the file was
/// last written.
pub fn recording_start(p: &Path) -> Option<When> {
    let name = file_name(p);
    let b = name.as_bytes();
    let from_name = (0..b.len().saturating_sub(16)).find_map(|i| {
        let s = name.get(i..i + 17)?;
        let (d, t) = (s.get(..10)?, s.get(11..)?);
        if !matches!(b[i + 10], b'_' | b'T' | b' ') || !t.bytes().all(|c| c.is_ascii_digit()) {
            return None;
        }
        let w = When::parse(d).ok()?;
        let (h, m) = (t[..2].parse::<u32>().ok()?, t[2..4].parse::<u32>().ok()?);
        (h < 24 && m < 60).then_some(When {
            day: w.day,
            minute: Some(h * 60 + m),
        })
    });
    from_name.or_else(|| {
        let t = std::fs::metadata(p).ok()?.modified().ok()?;
        let secs = t.duration_since(std::time::UNIX_EPOCH).ok()?.as_secs() as libc::time_t;
        let tm = unsafe {
            let mut tm: libc::tm = std::mem::zeroed();
            libc::localtime_r(&secs, &mut tm);
            tm
        };
        Some(When {
            day: days_from_civil(
                tm.tm_year as i64 + 1900,
                (tm.tm_mon + 1) as u32,
                tm.tm_mday as u32,
            ),
            minute: Some((tm.tm_hour * 60 + tm.tm_min) as u32),
        })
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

/// One misfire value before and after: how much it rose over each side's drives (None: that side's drives do not carry
/// it).
#[derive(Clone, Debug, PartialEq)]
pub struct Misfire {
    pub name: String,
    pub before: Option<f64>,
    pub after: Option<f64>,
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
/// (`recording_start`) against the event's date (and time, when it has one: without it, the drives on its day itself are
/// left out, since they could be on either side). A side with no drive is an error saying so.
pub fn before_after(
    event: &Event,
    recordings: &[PathBuf],
    drives: usize,
    min_rows: usize,
) -> Result<BeforeAfter, String> {
    let drives = drives.max(1);
    let mut before: Vec<(When, PathBuf, String)> = vec![];
    let mut after: Vec<(When, PathBuf, String)> = vec![];
    let (mut same_day, mut skipped) = (vec![], vec![]);
    for p in recordings {
        let Some(start) = recording_start(p) else {
            skipped.push(format!("{}: when it began is not known", file_name(p)));
            continue;
        };
        let text = match std::fs::read_to_string(p) {
            Ok(t) => t,
            Err(e) => {
                skipped.push(format!("{}: {e}", file_name(p)));
                continue;
            }
        };
        if let Err(e) = drivelog::series(&text) {
            skipped.push(format!("{}: {e}", file_name(p)));
            continue;
        }
        let side = match event.when.minute {
            Some(_) => Some(start < event.when),
            None if start.day == event.when.day => None,
            None => Some(start.day < event.when.day),
        };
        match side {
            Some(true) => before.push((start, p.clone(), text)),
            Some(false) => after.push((start, p.clone(), text)),
            None => same_day.push(p.clone()),
        }
    }
    before.sort_by_key(|a| std::cmp::Reverse(a.0));
    before.truncate(drives);
    before.reverse();
    after.sort_by_key(|a| a.0);
    after.truncate(drives);
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
    let mut counts: BTreeMap<String, (Option<f64>, Option<f64>)> = BTreeMap::new();
    let mut seconds = (0.0, 0.0);
    for (i, v) in [&before, &after].into_iter().enumerate() {
        for (_, _, t) in v {
            let (m, secs) = compare::misfires(t)?;
            if i == 0 {
                seconds.0 += secs;
            } else {
                seconds.1 += secs;
            }
            for (name, rise) in m {
                let e = counts.entry(name).or_default();
                let slot = if i == 0 { &mut e.0 } else { &mut e.1 };
                *slot = Some(slot.unwrap_or(0.0) + rise);
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
            .map(|(name, (before, after))| Misfire {
                name,
                before,
                after,
            })
            .collect(),
        seconds,
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

    /// The misfire counts before and after, in lines; one line saying so when the recordings carry none.
    pub fn misfire_lines(&self) -> Vec<String> {
        if self.misfires.is_empty() {
            return vec!["The recordings carry no misfire counts (a module-data value named misfire, recorded with the \
                         vehicle's module data, would be compared here)."
                .into()];
        }
        let side = |v: Option<f64>, s: f64| match v {
            Some(x) => format!("{x:.0} over {}", hours(s)),
            None => "not recorded".into(),
        };
        self.misfires
            .iter()
            .map(|m| {
                format!(
                    "{}: before {}, after {}",
                    m.name,
                    side(m.before, self.seconds.0),
                    side(m.after, self.seconds.1)
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
        o += "\nMisfire counts (how much each misfire value rose over the drives):\n";
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
            Service {
                kind: Kind::Oil,
                date: "2026-05-01".into(),
                miles: Some(80_000),
                note: "  0W-20   synthetic ".into(),
            },
        )
        .unwrap();
        assert!(w.is_empty());
        assert_eq!(s.note, "0W-20 synthetic");
        add(
            &dir,
            Service {
                kind: Kind::SparkPlugs,
                date: "2026-11-29T14:30".into(),
                miles: Some(84_213),
                note: String::new(),
            },
        )
        .unwrap();
        // logged out of order: the file keeps date order; a lower odometer than an earlier one is said
        let (_, w) = add(
            &dir,
            Service {
                kind: Kind::AirFilter,
                date: "2026-06-01".into(),
                miles: Some(8_100),
                note: String::new(),
            },
        )
        .unwrap();
        assert!(w[0].contains("less than the 80,000 mi"), "{w:?}");
        assert!(add(
            &dir,
            Service {
                kind: Kind::Other,
                date: "2026-06-01".into(),
                miles: None,
                note: " ".into(),
            },
        )
        .unwrap_err()
        .contains("note"));
        assert!(add(
            &dir,
            Service {
                kind: Kind::Oil,
                date: "June 1".into(),
                miles: None,
                note: String::new(),
            },
        )
        .is_err());
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
            Some(Interval {
                miles: Some(5000),
                months: Some(6)
            })
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
        let svc = |k, date: &str, miles| Service {
            kind: k,
            date: date.into(),
            miles,
            note: String::new(),
        };
        log.services = vec![
            svc(Kind::Oil, "2026-01-10", Some(70_000)),
            svc(Kind::Oil, "2026-05-01", Some(80_000)),
            svc(Kind::AirFilter, "2025-12-15", Some(60_000)),
            svc(Kind::SparkPlugs, "2026-11-29", None),
            svc(Kind::Brakes, "2026-01-01", None),
        ];
        let iv = |miles, months| Interval { miles, months };
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
            ("notes.csv", "not a recording\n".to_string()),
        ];
        for (n, t) in &files {
            std::fs::write(logs.join(n), t).unwrap();
        }
        let dir = home.clone();
        add(
            &dir,
            Service {
                kind: Kind::SparkPlugs,
                date: "2026-11-29".into(),
                miles: Some(84_213),
                note: "NGK".into(),
            },
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
            ba.skipped.iter().any(|s| s.starts_with("notes.csv")),
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
                before: Some(4.0),
                after: Some(0.0)
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
        assert_eq!(ba.misfires[0].before, Some(10.0));
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
    fn a_recording_begins_when_its_name_says() {
        let w = recording_start(Path::new("/x/drive-2026-11-27_183005-part2.csv")).unwrap();
        assert_eq!(w.text(), "2026-11-27 18:30");
        assert_eq!(
            recording_start(Path::new("/x/drive-2026-11-27_183005-2.csv"))
                .unwrap()
                .text(),
            "2026-11-27 18:30"
        );
        assert_eq!(
            recording_start(Path::new("/nowhere/at-all.csv")),
            None,
            "no name, no file"
        );
    }
}
