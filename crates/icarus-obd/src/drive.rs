//! Recording a drive: one recorder for the window's Record drive and for `icarus record` (ICA-208), so both write the same
//! files the same way.
//!
//! A recording is a CSV in ~/Documents/Icarus/logs (drivelog reads it): a `# adapter:` line, a `# module data:` line when
//! module values are in it, the header, then one row per sweep. A sweep reads the fast standard values (`FAST`: what fuel
//! trims, timing, air flow, the throttle and the mixture are read from) every time, two of the other values the vehicle
//! offers in turn, and, no more often than `MODULE_STEP`, one request of the vehicle's module data (ICA-44: transmission
//! fluid temperature, knock correction... from the set `icarus obd data` would choose), through module data's gates and
//! pacer (`moddata::fetch_request`: services 21 and 22 only, never an airbag or key module, refusals dropped for the
//! session). Each module request has its decoded values in columns named `"knock correction (700 22 1F01) [°]"` and its
//! raw answer beside them (`"700 22 1F01 answer [hex]"`), so a set improved later decodes old drives again
//! (`redecode`). Values stay in the vehicle's own metric units in the file; screens convert them.
//!
//! In turn, each module value comes only every half minute or so on a set of a hundred values, too seldom to tie knock or
//! a shift to a moment. So the requests carrying a watched value (ICA-218: `WATCH_DEFAULT`, `--watch NAME`, the window's
//! setting in `watch_path`) are read every sweep besides the one in turn, at most `PRIORITY_PER_SWEEP` of them a sweep,
//! through the same gates and pacer; what is watched is said at the start, and the rows a second after `RATE_AFTER_S`.
//!
//! When the engine stops and starts again during one recording (a stop for fuel), the next rows go into a new part file
//! (`drive-...-part2.csv`): one file per ignition cycle. A row is flushed as it is written, and a part is closed (flushed
//! and synced) when it ends, so a recording cut off by a crash keeps every row before it.
//!
//! `run` is the recording loop of `icarus record`: it waits for the adapter (trying again while the vehicle sleeps), with
//! `--auto` waits for the engine to run and ends the drive once the engine has been off or the adapter gone for a while,
//! and goes on through adapter glitches (ICA-112: a failed sweep is tried again; several in a row, the adapter is opened
//! again and the same recording goes on).

use crate::elm::ObdError;
use crate::link::Link;
use crate::moddata::{self, Loaded, Outcome, Pacer, Scope};
use crate::notify::Notifier;
use crate::{drivelog, gps, j1979, prune};
use icarus_core::moddata::{ModuleDef, RequestDef, Shown};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

/// The values read every sweep while recording a drive, when the vehicle offers them. Engine speed comes first.
pub const FAST: &[u8] = &[
    0x0C, 0x0D, 0x04, 0x06, 0x07, 0x08, 0x09, 0x0E, 0x10, 0x11, 0x49, 0x15, 0x19, 0x44, 0x24, 0x28,
    0x34, 0x38,
];

/// A drive's values from what the vehicle offers: (read every sweep, the rest read in turn).
pub fn drive_set(offered: &[u8]) -> (Vec<u8>, Vec<u8>) {
    let fast: Vec<u8> = FAST
        .iter()
        .copied()
        .filter(|p| offered.contains(p))
        .collect();
    let rest = offered
        .iter()
        .copied()
        .filter(|p| !fast.contains(p))
        .collect();
    (fast, rest)
}

/// Engine speed at or above this is the engine running (a cranking engine turns at 150-300 rpm).
pub const RUNNING_RPM: f64 = 400.0;

/// The least time between two module-data requests of a recording: a sweep of the standard values keeps its pace, and
/// module values (which change slowly: temperatures, learned corrections) come every few seconds each.
pub const MODULE_STEP: Duration = Duration::from_millis(250);

/// The least time between two sweeps of `run`, as the window's worker paces its own: at most 50 rows a second.
pub const MIN_SWEEP: Duration = Duration::from_millis(20);

/// Now, local time, for a file's name: 2026-11-27_183005.
pub fn stamp() -> String {
    unsafe {
        let t = libc::time(std::ptr::null_mut());
        let mut tm: libc::tm = std::mem::zeroed();
        libc::localtime_r(&t, &mut tm);
        format!(
            "{:04}-{:02}-{:02}_{:02}{:02}{:02}",
            tm.tm_year + 1900,
            tm.tm_mon + 1,
            tm.tm_mday,
            tm.tm_hour,
            tm.tm_min,
            tm.tm_sec
        )
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Module data in the rotation (ICA-44)

/// The module values a recording reads every sweep unless told otherwise (ICA-218): knock, the gear, lock-up, the shift
/// lever and sport mode, what has to be tied to a moment of the drive. Each is part of a value's name, matched ignoring
/// case, spaces and punctuation ("lock up" finds "Lock up state" and "torque converter lock-up").
pub const WATCH_DEFAULT: &[&str] = &[
    "knock feedback",
    "knock correction",
    "gear",
    "lock up",
    "lockup",
    "shift sw status",
    "sports mode",
    "sports shift",
    "neutral",
];

/// The most watched requests one sweep reads (ICA-218), the bound on what watching costs a row. Each is one more module
/// request in every sweep: its round trip, the switch of the adapter's header when it is another module's (five commands
/// there, three back), and the pacer's least gap (`moddata::MIN_GAP`, from the end of one answer to the next request),
/// so on a Bluetooth adapter each adds roughly a tenth of a second to the row. With more watched requests than this they
/// take turns (each every second or third sweep): a long `--watch` list slows its own values, not the whole recording.
/// The standard values and the module values in turn keep their pace either way (one request in turn no more often than
/// `MODULE_STEP`). The pacer is not bypassed: a watched request goes through the same gates and is never asked more often
/// than the pacer's floor (`moddata::MIN_EVERY`, four times a second), whatever the sweep's speed.
pub const PRIORITY_PER_SWEEP: usize = 4;

/// The rows-per-second line comes once the recording has run this long (seconds).
pub const RATE_AFTER_S: f64 = 10.0;

/// The share of its rows a second a recording keeps with the default watch, against one that watches nothing (ICA-218's
/// done, tested on the simulated vehicle at an adapter's pace: 8 ms an AT command, 40 ms a request). The trade-off in
/// numbers there: two watched requests on two modules take a row from about 5 a second to about 2 (0.39), and the
/// watched values go from once every few seconds to every row (0.4 s). Most of a watched request's cost is the header
/// switch to its module, so the watched requests go by module, beginning with the module the request in turn left the
/// header on.
pub const RATE_KEPT: f64 = 1.0 / 3.0;

/// A name as it is matched: lower case, letters and digits only.
fn fold(s: &str) -> String {
    s.chars()
        .filter(|c| c.is_alphanumeric())
        .flat_map(char::to_lowercase)
        .collect()
}

/// Whether a value's name holds a watched name (`WATCH_DEFAULT`'s way of matching).
pub fn watched(name: &str, watch: &str) -> bool {
    let w = fold(watch);
    !w.is_empty() && fold(name).contains(&w)
}

/// The module requests a recording reads: those of the chosen set that its gates let through for this vehicle. Those
/// carrying a watched value are read every sweep (`watching`, ICA-218), the rest in turn.
pub struct Modules {
    set: Loaded,
    scope: Scope,
    /// (module, request) by index into the set.
    items: Vec<(usize, usize)>,
    /// Still asked (a refusal, silence or repeated failures drop one for the recording).
    asked: Vec<bool>,
    at: usize,
    pacer: Pacer,
    last: Option<Instant>,
    /// The watched requests (indices into `items`, by module so the adapter switches header as little as possible), and
    /// where the next sweep's turn starts when there are more than `PRIORITY_PER_SWEEP`.
    priority: Vec<usize>,
    prio_at: usize,
    /// Each item's answers so far (for the rows-per-second line).
    answers: Vec<u32>,
    /// What is watched, in words, and the watched names no request of the set carries.
    watching: Vec<String>,
    unmatched: Vec<String>,
    /// What the gates kept out, in words (said once when the recording starts).
    pub left_out: Vec<String>,
}

impl Modules {
    /// The set as used on the vehicle (`vehicledata::choose_for`, or `Loaded::on_vehicle`), what the vehicle may be asked
    /// (`scope`), and the pacer every request goes through. The default values are watched (`WATCH_DEFAULT`).
    pub fn new(set: Loaded, scope: Scope, pacer: Pacer) -> Modules {
        let (mut items, mut left_out) = (Vec::new(), Vec::new());
        for (mi, m) in set.modules.iter().enumerate() {
            for (ri, r) in m.requests.iter().enumerate() {
                if r.values.is_empty() {
                    continue;
                }
                match moddata::may_send(&set.gates, m, r, &scope) {
                    Ok(()) => items.push((mi, ri)),
                    Err(o) => left_out.push(format!(
                        "{} {}: {}",
                        m.module.label(),
                        r.label(),
                        o.describe()
                    )),
                }
            }
        }
        Modules {
            asked: vec![true; items.len()],
            answers: vec![0; items.len()],
            set,
            scope,
            items,
            at: 0,
            pacer,
            last: None,
            priority: Vec::new(),
            prio_at: 0,
            watching: Vec::new(),
            unmatched: Vec::new(),
            left_out,
        }
        .watching(WATCH_DEFAULT)
    }

    /// Reads the requests carrying a value whose name holds one of `names` (`watched`) every sweep (ICA-218), the others
    /// in turn; an empty list reads everything in turn, as before ICA-218.
    pub fn watching<S: AsRef<str>>(mut self, watch: &[S]) -> Modules {
        let mut names: Vec<&str> = Vec::new();
        for n in watch.iter().map(|n| n.as_ref().trim()) {
            if !fold(n).is_empty() && !names.iter().any(|x| fold(x) == fold(n)) {
                names.push(n);
            }
        }
        let mut used = vec![false; names.len()];
        let mut found: Vec<((u32, Option<u8>), usize, String)> = Vec::new();
        for i in 0..self.items.len() {
            let (m, r) = self.item(i);
            let mut hits: Vec<&str> = Vec::new();
            for v in &r.values {
                let mut hit = false;
                for (k, w) in names.iter().enumerate() {
                    if watched(&v.name, w) {
                        used[k] = true;
                        hit = true;
                    }
                }
                if hit {
                    hits.push(&v.name);
                }
            }
            if !hits.is_empty() {
                let line = format!("{} ({} {})", hits.join(", "), m.module.label(), r.label());
                found.push(((m.module.tx, m.module.ext), i, line));
            }
        }
        // by module (a set may name one module in more than one file): the header switched once for its requests
        found.sort_by_key(|f| f.0);
        self.priority = found.iter().map(|f| f.1).collect();
        self.watching = found.into_iter().map(|f| f.2).collect();
        self.prio_at = 0;
        self.unmatched = names
            .iter()
            .zip(&used)
            .filter(|(_, u)| !**u)
            .map(|(n, _)| n.to_string())
            .collect();
        self
    }

    pub fn set_name(&self) -> &str {
        &self.set.set.name
    }

    /// The watched requests, in words: `"Knock correction (700 22 1F01)"` (the watched values of each, and where).
    pub fn watched_requests(&self) -> &[String] {
        &self.watching
    }

    /// What the recording says about the watched values when it starts: which are read every sweep and how, and the
    /// names that found nothing.
    pub fn watch_lines(&self) -> Vec<String> {
        let mut out = Vec::new();
        let rest = self.items.len() - self.priority.len();
        let n = self.priority.len();
        if n == 0 {
            out.push(format!(
                "Module data: nothing watched; all {rest} request(s) read in turn."
            ));
        } else {
            let pace = if n <= PRIORITY_PER_SWEEP {
                "every sweep".to_string()
            } else {
                format!(
                    "{PRIORITY_PER_SWEEP} a sweep in turn, each about every {} sweeps",
                    n.div_ceil(PRIORITY_PER_SWEEP)
                )
            };
            out.push(format!(
                "Module data watched, read {pace} ({n} request(s); the other {rest} in turn): {}.",
                self.watching.join("; ")
            ));
        }
        if !self.unmatched.is_empty() {
            out.push(format!(
                "Module data: no value of the set {} is named {}, so it is not watched.",
                self.set_name(),
                self.unmatched
                    .iter()
                    .map(|n| format!("\"{n}\""))
                    .collect::<Vec<_>>()
                    .join(", ")
            ));
        }
        out
    }

    fn item(&self, i: usize) -> (&ModuleDef, &RequestDef) {
        let (mi, ri) = self.items[i];
        let m = &self.set.modules[mi];
        (m, &m.requests[ri])
    }

    /// Item `i` asked, the same way in turn and watched: None when it was not due (its `every`, or for a watched one the
    /// pacer's floor) or the gates or the pacer dropped it (said in `notes`; not asked again).
    fn ask(
        &mut self,
        i: usize,
        link: &mut dyn Link,
        notes: &mut Vec<String>,
    ) -> Option<Result<Vec<u8>, Outcome>> {
        let (mi, ri) = self.items[i];
        let m = &self.set.modules[mi];
        let def = &m.requests[ri];
        // a watched request every sweep: its definition's `every` (OBDb's polling hint, often 1 s) gives way to the
        // pacer's own floor, which still holds
        let paced;
        let r = if self.priority.contains(&i) {
            let mut w = def.clone();
            w.every = Some(moddata::MIN_EVERY.as_secs_f64());
            paced = w;
            &paced
        } else {
            def
        };
        match moddata::fetch_request(link, &mut self.pacer, &self.set.gates, m, r, &self.scope) {
            Err(Outcome::NotDue) => None,
            Err(Outcome::Skipped(why)) => {
                self.asked[i] = false;
                notes.push(format!(
                    "Module data {} {} is no longer read: {why}",
                    m.module.label(),
                    r.label()
                ));
                None
            }
            other => {
                if other.is_ok() {
                    self.answers[i] += 1;
                }
                Some(other)
            }
        }
    }

    /// Where item `i` is asked: (module request ID, address byte).
    fn module_of(&self, i: usize) -> (u32, Option<u8>) {
        let m = &self.set.modules[self.items[i].0].module;
        (m.tx, m.ext)
    }

    /// The watched requests due this sweep, sent: all of them, or `PRIORITY_PER_SWEEP` in turn when there are more. (index,
    /// its answer's data or what it did) for each sent. `after`: the request just asked in turn, whose module's watched
    /// requests then go first (the adapter's header is already there: five commands fewer).
    fn watched_step(
        &mut self,
        link: &mut dyn Link,
        notes: &mut Vec<String>,
        after: Option<usize>,
    ) -> Vec<(usize, Result<Vec<u8>, Outcome>)> {
        let n = self.priority.len();
        if n <= PRIORITY_PER_SWEEP {
            // every one each sweep, by module, beginning with the module the header is on
            let here = after.map(|i| self.module_of(i));
            self.prio_at = self
                .priority
                .iter()
                .position(|i| Some(self.module_of(*i)) == here)
                .unwrap_or(0);
        }
        let mut out = Vec::new();
        for _ in 0..n {
            if out.len() >= PRIORITY_PER_SWEEP {
                break;
            }
            let i = self.priority[self.prio_at % n];
            self.prio_at += 1;
            if !self.asked[i] {
                continue;
            }
            if let Some(o) = self.ask(i, link, notes) {
                out.push((i, o));
            }
        }
        out
    }

    /// The next request in turn due, sent: (its index, its answer's data or what it did); None when none was due (one no
    /// more often than `MODULE_STEP`). The watched requests are not in the turn.
    fn step(
        &mut self,
        link: &mut dyn Link,
        notes: &mut Vec<String>,
    ) -> Option<(usize, Result<Vec<u8>, Outcome>)> {
        if self.items.is_empty() || self.last.is_some_and(|t| t.elapsed() < MODULE_STEP) {
            return None;
        }
        for _ in 0..self.items.len() {
            let i = self.at % self.items.len();
            self.at += 1;
            if !self.asked[i] || self.priority.contains(&i) {
                continue;
            }
            if let Some(o) = self.ask(i, link, notes) {
                self.last = Some(Instant::now());
                return Some((i, o));
            }
        }
        None
    }

    /// Seconds between answers of the watched requests still asked over `secs` of recording: (most often, least often);
    /// None when nothing is watched, and `f64::INFINITY` as the least often when one never answered.
    fn watched_every(&self, secs: f64) -> Option<(f64, f64)> {
        let mut out: Option<(f64, f64)> = None;
        for i in self.priority.iter().filter(|i| self.asked[**i]) {
            let n = self.answers[*i];
            let every = if n == 0 {
                f64::INFINITY
            } else {
                secs / n as f64
            };
            out = Some(out.map_or((every, every), |(a, b)| (a.min(every), b.max(every))));
        }
        out
    }
}

/// A module request's raw-answer column: `"700 22 1F01 answer [hex]"`.
fn raw_name(m: &ModuleDef, r: &RequestDef) -> String {
    format!("{} {} answer [hex]", m.module.label(), r.label())
}

/// A module value's column: `"knock correction (700 22 1F01) [°]"` (no brackets for a value with no unit).
fn value_name(m: &ModuleDef, r: &RequestDef, name: &str, unit: &str) -> String {
    let base = format!("{name} ({} {})", m.module.label(), r.label());
    if unit.is_empty() {
        base
    } else {
        format!("{base} [{unit}]")
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// The file

/// One cell of a row.
#[derive(Clone, Debug, PartialEq)]
enum Cell {
    Empty,
    Num(f64),
    Text(String),
}

fn cell_text(c: &Cell) -> String {
    match c {
        Cell::Empty => String::new(),
        Cell::Num(x) => format!("{x:.2}"),
        // a text cell is quoted when it holds a comma; quotes inside are dropped
        Cell::Text(t) => {
            let t = t.replace(['"', '\r', '\n'], "");
            if t.contains(',') {
                format!("\"{t}\"")
            } else {
                t
            }
        }
    }
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02X}")).collect()
}

/// A part being written.
struct Part {
    path: PathBuf,
    out: std::io::BufWriter<std::fs::File>,
    t0: Option<f64>,
}

impl Part {
    fn close(mut self) -> Result<PathBuf, String> {
        self.out
            .flush()
            .map_err(|e| format!("{}: {e}", self.path.display()))?;
        self.out
            .get_ref()
            .sync_all()
            .map_err(|e| format!("{}: {e}", self.path.display()))?;
        Ok(self.path)
    }
}

/// What a recording is made from.
pub struct Plan {
    /// The standard values the vehicle offers (`j1979::supported`): the fast ones every sweep, the rest in turn.
    pub offered: Vec<u8>,
    /// The adapter as it names itself (`Link::adapter`), for the file's first line.
    pub adapter: String,
    /// The vehicle's module data in the rotation, when a set was chosen for it.
    pub modules: Option<Modules>,
    /// A GPS receiver followed, its columns after the values (only in the file).
    pub gps: Option<String>,
    /// A GPS that does not open is gone without (said in `gps_missing`) rather than stopping the recording: a recording
    /// by itself must not wait on a receiver left at home.
    pub gps_optional: bool,
    /// Where the files go (~/Documents/Icarus/logs: `drivelog::folders`).
    pub dir: PathBuf,
    /// Keep the computer from sleeping (the lid closed included) while recording (false in tests).
    pub keep_awake: bool,
    /// The VIN read when it started: connecting again to another vehicle ends the recording.
    pub vin: Option<String>,
}

/// What one sweep did.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Sweep {
    /// Every standard value read, for the live view: (PID, value).
    pub values: Vec<(u8, f64)>,
    /// A row was written.
    pub row: bool,
    /// Engine speed, when it was read.
    pub rpm: Option<f64>,
    /// A new part file began with this sweep (the engine started again).
    pub new_part: Option<PathBuf>,
    /// What to say: a value no longer read, a module request dropped.
    pub notes: Vec<String>,
    /// How fast it records, said once after the first `RATE_AFTER_S` (ICA-218): the rows a second and how often the watched
    /// module values came.
    pub rate: Option<String>,
}

impl Sweep {
    /// Whether the engine runs, when engine speed was read.
    pub fn running(&self) -> Option<bool> {
        self.rpm.map(|r| r >= RUNNING_RPM)
    }
}

/// A drive being recorded.
pub struct Recorder {
    /// Read every sweep, and the rest two at a time in turn.
    pub fast: Vec<u8>,
    pub rest: Vec<u8>,
    rest_at: usize,
    unanswered: prune::Unanswered,
    modules: Option<Modules>,
    gps: Option<gps::Follower>,
    /// The standard columns, in order.
    pids: Vec<u8>,
    /// The module columns: (item, value id), then the raw answers (one per item).
    value_cols: Vec<(usize, String)>,
    names: Vec<String>,
    head: Vec<String>,
    dir: PathBuf,
    stamp: String,
    part_no: u32,
    part: Option<Part>,
    /// Rows written, every part.
    pub rows: usize,
    /// The parts closed so far.
    pub closed: Vec<PathBuf>,
    /// The engine ran in this part, and stopped since.
    ran: bool,
    stopped: bool,
    pub vin: Option<String>,
    /// Kept while recording: the computer does not sleep.
    _awake: Option<crate::power_guard::Inhibit>,
    /// Why the computer could not be kept awake, when it could not (said once, when the recording starts).
    pub awake_missing: Option<String>,
    /// Why the GPS asked for is not followed (`Plan::gps_optional`), when it is not.
    pub gps_missing: Option<String>,
    /// The first row's time and the rows before it, and whether the rows-per-second line was said (ICA-218).
    rate_from: Option<(f64, usize)>,
    rate_said: bool,
    /// After how long (`RATE_AFTER_S`; shorter in tests).
    rate_after: f64,
}

impl Recorder {
    /// Starts a recording: its first part is created at once (`dir` made if needed). A GPS source that does not open stops
    /// it before a file is made, unless the plan lets it go on without (`gps_optional`).
    pub fn start(plan: Plan) -> Result<Recorder, String> {
        Recorder::start_pruning(plan, prune::WATCH_S)
    }

    /// `start`, with a value read in turn that never answers in its first `prune_after` seconds no longer asked (ICA-87).
    pub fn start_pruning(plan: Plan, prune_after: f64) -> Result<Recorder, String> {
        let mut gps_missing = None;
        let gps = match plan.gps.as_deref().map(gps::Follower::start).transpose() {
            Ok(g) => g,
            Err(e) if plan.gps_optional => {
                gps_missing = Some(format!(
                    "the GPS ({}) did not open ({e}): recording without positions",
                    plan.gps.as_deref().unwrap_or("")
                ));
                None
            }
            Err(e) => return Err(e),
        };
        let (fast, rest) = drive_set(&plan.offered);
        let pids: Vec<u8> = fast
            .iter()
            .chain(&rest)
            .copied()
            .filter(|p| j1979::pid(*p).is_some())
            .collect();
        let mut names: Vec<String> = pids
            .iter()
            .filter_map(|p| j1979::pid(*p))
            .map(|d| format!("{} [{}]", d.name, d.unit))
            .collect();
        let mut head = vec![drivelog::adapter_line(&plan.adapter)];
        let mut value_cols = Vec::new();
        if let Some(md) = &plan.modules {
            for i in 0..md.items.len() {
                let (m, r) = md.item(i);
                for v in &r.values {
                    value_cols.push((i, v.id.clone()));
                    names.push(value_name(m, r, &v.name, &v.unit));
                }
            }
            head.push(module_line(md.set_name(), md.scope.year));
        }
        if gps.is_some() {
            names.extend(gps::COLUMNS.iter().map(|c| c.to_string()));
        }
        if let Some(md) = &plan.modules {
            for i in 0..md.items.len() {
                let (m, r) = md.item(i);
                names.push(raw_name(m, r));
            }
        }
        let (awake, awake_missing) = if plan.keep_awake {
            match crate::power_guard::Inhibit::recording("Recording a drive") {
                Ok(i) => (Some(i), None),
                Err(e) => (
                    None,
                    Some(format!(
                        "{e}: recording anyway, but the computer may sleep (the lid closed) and stop it"
                    )),
                ),
            }
        } else {
            (None, None)
        };
        let mut r = Recorder {
            fast,
            rest,
            rest_at: 0,
            unanswered: prune::Unanswered::new(prune_after),
            modules: plan.modules,
            gps,
            pids,
            value_cols,
            names,
            head,
            dir: plan.dir,
            stamp: stamp(),
            part_no: 0,
            part: None,
            rows: 0,
            closed: Vec::new(),
            ran: false,
            stopped: false,
            vin: plan.vin,
            _awake: awake,
            awake_missing,
            gps_missing,
            rate_from: None,
            rate_said: false,
            rate_after: RATE_AFTER_S,
        };
        r.next_part()?;
        Ok(r)
    }

    /// What to say about the watched module values when the recording starts (`Modules::watch_lines`); nothing without
    /// module data.
    pub fn watch_lines(&self) -> Vec<String> {
        self.modules
            .as_ref()
            .map(Modules::watch_lines)
            .unwrap_or_default()
    }

    /// How many values each row holds (standard and module values).
    pub fn values(&self) -> usize {
        self.pids.len() + self.value_cols.len()
    }

    /// What the gates kept out of the module data, in words.
    pub fn left_out(&self) -> &[String] {
        self.modules.as_ref().map_or(&[][..], |m| &m.left_out)
    }

    /// The part being written.
    pub fn path(&self) -> Option<&Path> {
        self.part.as_ref().map(|p| p.path.as_path())
    }

    /// The file's name, for messages.
    pub fn name(&self) -> String {
        self.path()
            .or(self.closed.last().map(|p| p.as_path()))
            .and_then(|p| p.file_name())
            .map(|f| f.to_string_lossy().into_owned())
            .unwrap_or_default()
    }

    /// Ends the part being written (if any) and begins the next.
    fn next_part(&mut self) -> Result<PathBuf, String> {
        if let Some(p) = self.part.take() {
            self.closed.push(p.close()?);
        }
        self.part_no += 1;
        std::fs::create_dir_all(&self.dir).map_err(|e| format!("{}: {e}", self.dir.display()))?;
        let base = if self.part_no == 1 {
            format!("drive-{}", self.stamp)
        } else {
            format!("drive-{}-part{}", self.stamp, self.part_no)
        };
        // never over another file: a second recording in the same second gets a number
        let mut n = 1;
        let (path, file) = loop {
            let name = if n == 1 {
                format!("{base}.csv")
            } else {
                format!("{base}-{n}.csv")
            };
            let path = self.dir.join(name);
            match std::fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(&path)
            {
                Ok(f) => break (path, f),
                Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists && n < 100 => n += 1,
                Err(e) => return Err(format!("{}: {e}", path.display())),
            }
        };
        let mut out = std::io::BufWriter::new(file);
        let mut head = self.head.clone();
        if self.part_no > 1 {
            head.push(format!(
                "# part {} of the drive that began in drive-{}.csv (the engine started again)",
                self.part_no, self.stamp
            ));
        }
        writeln!(
            out,
            "{}\n{}",
            head.join("\n"),
            drivelog::csv_header(&self.names)
        )
        .and_then(|_| out.flush())
        .map_err(|e| format!("{}: {e}", path.display()))?;
        self.part = Some(Part {
            path: path.clone(),
            out,
            t0: None,
        });
        self.ran = false;
        self.stopped = false;
        Ok(path)
    }

    /// Closes the recording: the part being written flushed and synced. Every part's path, oldest first.
    pub fn close(mut self) -> Result<Vec<PathBuf>, String> {
        if let Some(p) = self.part.take() {
            self.closed.push(p.close()?);
        }
        Ok(std::mem::take(&mut self.closed))
    }

    /// One sweep at `t` seconds (any clock that goes on through the recording): the fast values and `extra` (values the
    /// live view wants besides), two of the rest in turn, a module request when one is due; a row written when anything
    /// answered. An adapter error is returned as it is (the caller tries again, or opens the adapter again): nothing is
    /// written for that sweep.
    pub fn sweep(&mut self, link: &mut dyn Link, extra: &[u8], t: f64) -> Result<Sweep, ObdError> {
        let mut ask = self.fast.clone();
        for p in extra {
            if !ask.contains(p) {
                ask.push(*p);
            }
        }
        let mut turn = Vec::new();
        for _ in 0..self.rest.len().min(2) {
            let p = self.rest[self.rest_at % self.rest.len()];
            self.rest_at += 1;
            if !ask.contains(&p) {
                ask.push(p);
                turn.push(p);
            }
        }
        let mut s = Sweep::default();
        match j1979::read_many(link, &ask) {
            Ok(all) => {
                for (pid, r) in all {
                    if let Some(x) = r.iter().find(|x| x.from == "7E8").or(r.first()) {
                        s.values.push((pid, x.value));
                    }
                }
            }
            Err(ObdError::NoData) => {}
            Err(e) => return Err(e),
        }
        s.rpm = s.values.iter().find(|v| v.0 == 0x0C).map(|v| v.1);
        // a value read in turn that never answered in the first minute is no longer asked (ICA-87)
        if !turn.is_empty() {
            let answered: Vec<u8> = s.values.iter().map(|v| v.0).collect();
            self.unanswered.see(t, &turn, &answered);
            if let Some(gone) = self.unanswered.decide(t).filter(|g| !g.is_empty()) {
                self.rest.retain(|p| !gone.contains(p));
                let names: Vec<String> = gone
                    .iter()
                    .map(|p| match j1979::pid(*p) {
                        Some(d) => format!("PID {p:02X} ({})", d.name),
                        None => format!("PID {p:02X}"),
                    })
                    .collect();
                s.notes.push(format!(
                    "Recording: {} never answered in the first minute and is no longer read (its column stays empty)",
                    names.join(", ")
                ));
            }
        }
        // the engine started again after it stopped in this part: a new part, one per ignition cycle
        match s.running() {
            Some(true) if self.ran && self.stopped => {
                let p = self.next_part().map_err(ObdError::Garbled)?;
                s.new_part = Some(p);
                self.ran = true;
            }
            Some(true) => self.ran = true,
            Some(false) if self.ran => self.stopped = true,
            _ => {}
        }
        // one module request in turn when one is due, then the watched ones (ICA-218), that module's first (only while the
        // vehicle answers)
        let mut module = Vec::new();
        if let Some(m) = self.modules.as_mut().filter(|_| !s.values.is_empty()) {
            module.extend(m.step(link, &mut s.notes));
            let after = module.first().map(|x| x.0);
            module.extend(m.watched_step(link, &mut s.notes, after));
        }
        let mut cells = vec![Cell::Empty; self.names.len()];
        for (pid, v) in &s.values {
            if let Some(i) = self.pids.iter().position(|p| p == pid) {
                cells[i] = Cell::Num(*v);
            }
        }
        let mut wrote_module = false;
        for (item, data) in module
            .iter()
            .filter_map(|(i, o)| Some((i, o.as_ref().ok()?)))
        {
            let Some(md) = &self.modules else { break };
            let (_, r) = md.item(*item);
            if let Outcome::Values(vs) = moddata::decode_all(r, data, md.scope.year) {
                for (c, (it, id)) in self.value_cols.iter().enumerate() {
                    if it != item {
                        continue;
                    }
                    if let Some(v) = vs.iter().find(|v| &v.id == id) {
                        cells[self.pids.len() + c] = match &v.shown {
                            Shown::Number(x) => Cell::Num(*x),
                            Shown::State(t) | Shown::Text(t) | Shown::Hex(t) => {
                                Cell::Text(t.clone())
                            }
                            Shown::NoReading => Cell::Empty,
                        };
                    }
                }
            }
            // the answer as it came, whether or not it decodes: a set improved later reads it again
            let raw_at = self.names.len() - md.items.len() + item;
            cells[raw_at] = Cell::Text(hex(data));
            wrote_module = true;
        }
        if s.values.is_empty() && !wrote_module {
            return Ok(s);
        }
        if let Some(g) = &self.gps {
            let at = self.pids.len() + self.value_cols.len();
            for (k, v) in g.cells().iter().enumerate() {
                if let Some(v) = v {
                    cells[at + k] = Cell::Text(format!("{v:.6}"));
                }
            }
        }
        let Some(part) = self.part.as_mut() else {
            return Ok(s);
        };
        let t0 = *part.t0.get_or_insert(t);
        let line: Vec<String> = cells.iter().map(cell_text).collect();
        writeln!(part.out, "{:.2},{}", t - t0, line.join(","))
            .and_then(|_| part.out.flush())
            .map_err(|e| ObdError::Garbled(format!("{}: {e}", part.path.display())))?;
        self.rows += 1;
        s.row = true;
        // once, after the first RATE_AFTER_S: how fast it records, and how often the watched values came (ICA-218)
        let from = *self.rate_from.get_or_insert((t, self.rows - 1));
        if !self.rate_said && t - from.0 >= self.rate_after {
            self.rate_said = true;
            s.rate = Some(self.rate_line(t - from.0, self.rows - from.1));
        }
        Ok(s)
    }

    /// The rows-per-second line: `rows` in `secs`, and how often the watched module values answered.
    fn rate_line(&self, secs: f64, rows: usize) -> String {
        let rate = rows as f64 / secs;
        let watched = match self.modules.as_ref().and_then(|m| m.watched_every(secs)) {
            None => String::new(),
            Some((_, b)) if b.is_infinite() => {
                "; a watched module value has not answered yet".to_string()
            }
            Some((a, b)) if b - a < 0.05 => format!("; the watched module values every {a:.1} s"),
            Some((a, b)) => format!("; the watched module values every {a:.1} to {b:.1} s"),
        };
        format!("Recording at {rate:.1} rows a second (the first {secs:.0} s){watched}.")
    }
}

/// The `# module data:` line: the set and the model year its answers were decoded for.
fn module_line(set: &str, year: Option<u16>) -> String {
    format!(
        "# module data: set {set}{}; each request's answer is kept in its \"answer [hex]\" column (icarus record redecode)",
        year.map(|y| format!(", model year {y}")).unwrap_or_default()
    )
}

/// The set and model year a recording's `# module data:` line names.
pub fn module_data_of(text: &str) -> Option<(String, Option<u16>)> {
    let l = text
        .lines()
        .take_while(|l| l.starts_with('#'))
        .find_map(|l| l.strip_prefix("# module data: set "))?;
    let head = l.split(';').next()?;
    let (set, year) = match head.split_once(", model year ") {
        Some((s, y)) => (s, y.trim().parse().ok()),
        None => (head, None),
    };
    Some((set.trim().to_string(), year))
}

/// Splits a CSV line on commas outside double quotes (quotes dropped), as drivelog reads it.
fn split(line: &str) -> Vec<String> {
    let mut out = vec![];
    let (mut cur, mut quoted) = (String::new(), false);
    for c in line.chars() {
        match c {
            '"' => quoted = !quoted,
            ',' if !quoted => out.push(std::mem::take(&mut cur)),
            _ => cur.push(c),
        }
    }
    out.push(cur);
    out
}

/// A recording decoded again with `modules` (an improved set) for model year `year` (None: the year its `# module data:`
/// line names): each module request's columns rebuilt from its raw answers, everything else kept as it was. A request
/// the new set no longer has keeps its old columns. The new file's text; the old file is not touched.
pub fn redecode(text: &str, modules: &[ModuleDef], year: Option<u16>) -> Result<String, String> {
    let year = year.or_else(|| module_data_of(text).and_then(|m| m.1));
    let mut lines = text.lines();
    let mut head: Vec<String> = Vec::new();
    let header = loop {
        let l = lines.next().ok_or("the file has no header")?;
        if l.starts_with('#') {
            head.push(l.to_string());
        } else {
            break split(l);
        }
    };
    if header.first().map(|h| h.trim()) != Some("time_s") {
        return Err("not a drive recording (no time_s column)".into());
    }
    // each raw column, and the request of the new set it belongs to
    struct Raw<'a> {
        col: usize,
        key: String,
        req: Option<&'a RequestDef>,
    }
    let mut raws: Vec<Raw> = Vec::new();
    for (i, h) in header.iter().enumerate() {
        let Some(key) = h.trim().strip_suffix(" answer [hex]") else {
            continue;
        };
        let req = modules.iter().find_map(|m| {
            let label = m.module.label();
            let rest = key.strip_prefix(&label)?.strip_prefix(' ')?;
            m.requests.iter().find(|r| {
                r.label() == rest
                    && match year {
                        Some(y) if !r.years.is_empty() => {
                            icarus_core::moddata::year_in(&r.years, y)
                        }
                        _ => true,
                    }
            })
        });
        raws.push(Raw {
            col: i,
            key: key.to_string(),
            req,
        });
    }
    if raws.is_empty() {
        return Err("this recording kept no module answers (it was made without module data, or before ICA-44)".into());
    }
    let label_of = |key: &str| -> Option<(String, String)> {
        let (m, r) = key.split_once(' ')?;
        Some((m.to_string(), r.to_string()))
    };
    // the old value columns of each request the new set decodes are left out; the new ones go where the first was
    let decoded: Vec<&Raw> = raws.iter().filter(|r| r.req.is_some()).collect();
    let is_old_value = |h: &str| decoded.iter().any(|r| h.contains(&format!("({})", r.key)));
    let mut keep: Vec<usize> = Vec::new();
    for (i, h) in header.iter().enumerate() {
        if i == 0 || raws.iter().any(|r| r.col == i) || is_old_value(h.trim()) {
            continue;
        }
        keep.push(i);
    }
    let mut names: Vec<String> = keep.iter().map(|i| header[*i].trim().to_string()).collect();
    for r in &decoded {
        let (label, _) = label_of(&r.key).unwrap_or_default();
        let req = r.req.unwrap();
        for v in &req.values {
            let base = format!("{} ({label} {})", v.name, req.label());
            names.push(if v.unit.is_empty() {
                base
            } else {
                format!("{base} [{}]", v.unit)
            });
        }
    }
    names.extend(raws.iter().map(|r| header[r.col].trim().to_string()));
    let mut out = head;
    out.push("# decoded again from its answers with an updated module-data set".into());
    out.push(drivelog::csv_header(&names));
    for l in lines {
        if l.trim().is_empty() {
            continue;
        }
        let f = split(l);
        let get = |i: usize| f.get(i).map(|x| x.trim()).unwrap_or("");
        let mut cells: Vec<String> = vec![get(0).to_string()];
        cells.extend(keep.iter().map(|i| cell_text(&Cell::Text(get(*i).into()))));
        for r in &decoded {
            let req = r.req.unwrap();
            let data = icarus_core::moddata::parse_hex(get(r.col))
                .ok()
                .filter(|d| !d.is_empty());
            let vs = match data.map(|d| moddata::decode_all(req, &d, year)) {
                Some(Outcome::Values(vs)) => vs,
                _ => Vec::new(),
            };
            for v in &req.values {
                let c = match vs.iter().find(|x| x.id == v.id).map(|x| &x.shown) {
                    Some(Shown::Number(x)) => Cell::Num(*x),
                    Some(Shown::State(t) | Shown::Text(t) | Shown::Hex(t)) => Cell::Text(t.clone()),
                    _ => Cell::Empty,
                };
                cells.push(cell_text(&c));
            }
        }
        cells.extend(raws.iter().map(|r| get(r.col).to_string()));
        out.push(cells.join(","));
    }
    Ok(out.join("\n") + "\n")
}

// ---------------------------------------------------------------------------------------------------------------------
// The recording loop (`icarus record`)

/// How `run` records.
#[derive(Clone, Debug)]
pub struct Config {
    /// Wait for the engine to run before recording, and end the drive once the engine has been off (or the adapter gone)
    /// for `off_after`; otherwise record from now until stopped.
    pub auto: bool,
    pub off_after: Duration,
    /// Between tries to reach the adapter.
    pub retry: Duration,
    /// Between looks at the engine while waiting for it to start.
    pub idle_every: Duration,
    /// Failed sweeps in a row before the adapter is opened again.
    pub glitches: u32,
    /// The value-pruning watch (`prune::WATCH_S`; shorter in tests).
    pub prune_after: f64,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            auto: false,
            off_after: Duration::from_secs(120),
            retry: Duration::from_secs(5),
            idle_every: Duration::from_secs(1),
            glitches: 3,
            prune_after: prune::WATCH_S,
        }
    }
}

/// What the loop needs from outside: the command line gives the real adapter, set and messages; a test its own.
pub trait Driver {
    /// Opens the adapter and sets it up (the vehicle answering).
    fn open(&mut self) -> Result<Box<dyn Link>, String>;
    /// What a recording on this connection is made from.
    fn plan(&mut self, link: &mut dyn Link) -> Result<Plan, String>;
    /// Why recording must end now (Ctrl+C, SIGTERM), if it must.
    fn stop(&self) -> Option<String>;
    /// A line for the person (the terminal, or the journal for the service).
    fn say(&mut self, line: &str);
    /// What the modules did with each request since the last call (the vehicle's record, ICA-70).
    fn outcomes(&mut self, _vin: Option<&str>, _link: &dyn Link, _o: Vec<crate::record::Outcome>) {}
}

/// Records until `d.stop()` says to end (and, with `auto`, drive after drive); every file written, oldest first. The
/// recording in progress is closed however the loop ends.
pub fn run(cfg: &Config, d: &mut dyn Driver, n: &mut Notifier) -> Result<Vec<PathBuf>, String> {
    let clock = Instant::now();
    let secs = || clock.elapsed().as_secs_f64();
    let mut link: Option<Box<dyn Link>> = None;
    let mut rec: Option<Recorder> = None;
    let mut written: Vec<PathBuf> = Vec::new();
    let mut last_try: Option<Instant> = None;
    let mut last_err: Option<String> = None;
    let mut last_look: Option<Instant> = None;
    let mut glitches = 0u32;
    let mut last_sweep: Option<Instant> = None;
    // when the engine was last seen running (or the drive began)
    let mut alive = 0.0f64;
    fn finish(
        rec: Option<Recorder>,
        d: &mut dyn Driver,
        n: &mut Notifier,
        written: &mut Vec<PathBuf>,
    ) -> Result<(), String> {
        let Some(r) = rec else { return Ok(()) };
        let (rows, name) = (r.rows, r.name());
        let paths = r.close()?;
        n.stopped(rows, &name);
        d.say(&format!(
            "Recording stopped: {rows} rows in {}.",
            paths
                .iter()
                .map(|p| p.display().to_string())
                .collect::<Vec<_>>()
                .join(", ")
        ));
        written.extend(paths);
        Ok(())
    }
    loop {
        if let Some(why) = d.stop() {
            d.say(&format!("Ending: {why}."));
            break;
        }
        let now = secs();
        if rec.is_some() {
            n.tick(now);
            if cfg.auto && now - alive >= cfg.off_after.as_secs_f64() {
                d.say(&format!(
                    "The engine has not run for {} s (or the adapter is gone): the drive is over.",
                    cfg.off_after.as_secs()
                ));
                finish(rec.take(), d, n, &mut written)?;
            }
        }
        let Some(l) = link.as_mut() else {
            if last_try.is_none_or(|t| t.elapsed() >= cfg.retry) {
                last_try = Some(Instant::now());
                match d.open() {
                    Ok(mut l) => {
                        d.say(&format!("Connected: {}.", l.adapter()));
                        // the same recording goes on only with the same vehicle: another VIN ends it (a new one starts)
                        if let Some(r) = rec.as_ref() {
                            if let Ok(now_vin) = j1979::vin(&mut *l) {
                                if now_vin != r.vin {
                                    d.say(&format!(
                                        "The adapter came back on another vehicle ({}): this recording ends here.",
                                        now_vin.as_deref().unwrap_or("no VIN")
                                    ));
                                    finish(rec.take(), d, n, &mut written)?;
                                }
                            }
                        }
                        link = Some(l);
                        glitches = 0;
                        last_err = None;
                        last_look = None;
                    }
                    Err(e) => {
                        if last_err.as_ref() != Some(&e) {
                            d.say(&format!(
                                "Waiting for the adapter ({e}); trying again every {} s.",
                                cfg.retry.as_secs()
                            ));
                            last_err = Some(e);
                        }
                    }
                }
            }
            std::thread::sleep(Duration::from_millis(100));
            continue;
        };
        if rec.is_none() {
            if cfg.auto {
                // waiting for the engine: a look now and then
                if last_look.is_some_and(|t| t.elapsed() < cfg.idle_every) {
                    std::thread::sleep(Duration::from_millis(50));
                    continue;
                }
                last_look = Some(Instant::now());
                match j1979::read_many(&mut **l, &[0x0C]) {
                    Ok(v) => {
                        glitches = 0;
                        let rpm = v
                            .iter()
                            .find(|x| x.0 == 0x0C)
                            .and_then(|x| x.1.iter().find(|r| r.from == "7E8").or(x.1.first()))
                            .map(|r| r.value);
                        if rpm.is_none_or(|r| r < RUNNING_RPM) {
                            continue;
                        }
                        d.say(&format!(
                            "The engine runs ({:.0} rpm): recording.",
                            rpm.unwrap_or(0.0)
                        ));
                    }
                    Err(ObdError::NoData) => continue,
                    Err(e) => {
                        glitches += 1;
                        if glitches >= cfg.glitches {
                            d.say(&format!("The adapter stopped answering ({e})."));
                            link = None;
                        }
                        continue;
                    }
                }
            }
            match d
                .plan(&mut **l)
                .and_then(|p| Recorder::start_pruning(p, cfg.prune_after))
            {
                Ok(r) => {
                    alive = secs();
                    n.started(alive, &r.name());
                    d.say(&format!(
                        "Recording {} values to {}.",
                        r.values(),
                        r.path()
                            .map(|p| p.display().to_string())
                            .unwrap_or_default()
                    ));
                    for w in r.left_out() {
                        d.say(&format!("Module data left out: {w}"));
                    }
                    for w in r.watch_lines() {
                        d.say(&w);
                    }
                    if let Some(w) = &r.awake_missing {
                        d.say(&format!("Note: {w}."));
                    }
                    if let Some(w) = &r.gps_missing {
                        d.say(&format!("Note: {w}."));
                        n.notice("Recording without GPS", &format!("{w}."));
                    }
                    rec = Some(r);
                }
                Err(e) => {
                    d.say(&format!("Could not start recording: {e}"));
                    if !cfg.auto {
                        return Err(e);
                    }
                    link = None;
                    last_try = Some(Instant::now());
                }
            }
            continue;
        }
        // no faster than a sweep each MIN_SWEEP (a real adapter is slower anyway; the simulated one is not)
        if let Some(w) = last_sweep.and_then(|t: Instant| MIN_SWEEP.checked_sub(t.elapsed())) {
            std::thread::sleep(w);
        }
        last_sweep = Some(Instant::now());
        let r = rec.as_mut().expect("recording");
        let now = secs();
        match r.sweep(&mut **l, &[], now) {
            Ok(s) => {
                glitches = 0;
                for note in &s.notes {
                    d.say(note);
                }
                if let Some(rate) = &s.rate {
                    d.say(rate);
                }
                if let Some(p) = &s.new_part {
                    d.say(&format!(
                        "The engine started again: the next rows go to {}.",
                        p.display()
                    ));
                }
                if s.row {
                    n.row(now, s.running() == Some(false));
                }
                if s.running() == Some(true) {
                    alive = now;
                }
                if !s.row {
                    // nothing answered (the ignition is off): no need to ask at full speed
                    std::thread::sleep(Duration::from_millis(200));
                }
            }
            Err(e) => {
                glitches += 1;
                if matches!(e, ObdError::Io(_)) || glitches >= cfg.glitches {
                    d.say(&format!(
                        "The adapter stopped answering ({e}): the recording goes on when it is back."
                    ));
                    link = None;
                } else {
                    std::thread::sleep(Duration::from_millis(200));
                }
            }
        }
        if let (Some(l), Some(r)) = (link.as_mut(), rec.as_ref()) {
            for note in l.take_dropped() {
                d.say(&format!("Note: {note}"));
            }
            let o = l.take_outcomes();
            if !o.is_empty() {
                d.outcomes(r.vin.as_deref(), &**l, o);
            }
        }
    }
    finish(rec.take(), d, n, &mut written)?;
    Ok(written)
}

/// Ctrl+C, SIGTERM (what `systemctl stop` sends) and SIGHUP (the terminal closed) only mark the process interrupted
/// (`actuator::interrupted`), so the recording ends between two sweeps and its file is closed.
pub fn catch_stop_signals() {
    crate::actuator::catch_interrupt();
    crate::actuator::catch_terminate();
    crate::actuator::catch_hangup();
}

/// The adapter used last (`~/.config/icarus/adapter`, kept by the window), when there is one: `icarus record --auto`'s
/// known adapter when none is named.
pub fn remembered_adapter() -> Option<String> {
    let base = std::env::var_os("XDG_CONFIG_HOME")
        .filter(|v| !v.is_empty())
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
    let t = std::fs::read_to_string(base.join("icarus/adapter")).ok()?;
    let a = t.lines().next()?.trim().to_string();
    (!a.is_empty() && a != "sim").then_some(a)
}

/// Where the window keeps the module values a recording watches (ICA-218): `$XDG_CONFIG_HOME/icarus/record-watch`, else
/// `~/.config/icarus/record-watch`. `icarus record` without `--watch` takes it too, so both record the same way.
pub fn watch_path() -> Option<PathBuf> {
    let base = std::env::var_os("XDG_CONFIG_HOME")
        .filter(|v| !v.is_empty())
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
    Some(base.join("icarus/record-watch"))
}

/// The watched names a `record-watch` file gives: one a line (`#` starts a comment); `none` on a line of its own watches
/// nothing. A file naming nothing at all gives the defaults (`WATCH_DEFAULT`).
pub fn parse_watch(text: &str) -> Vec<String> {
    let names: Vec<String> = text
        .lines()
        .map(|l| l.split('#').next().unwrap_or("").trim().to_string())
        .filter(|l| !l.is_empty())
        .collect();
    if names.iter().any(|n| n.eq_ignore_ascii_case("none")) {
        return Vec::new();
    }
    if names.is_empty() {
        return WATCH_DEFAULT.iter().map(|w| w.to_string()).collect();
    }
    names
}

/// A `record-watch` file's text for `names` (an empty list is written as `none`).
pub fn watch_text(names: &[String]) -> String {
    let mut t = String::from(
        "# Module values a recording reads every sweep (part of a value's name, one a line; none: nothing watched)\n",
    );
    if names.is_empty() {
        t.push_str("none\n");
    }
    for n in names {
        t.push_str(n.trim());
        t.push('\n');
    }
    t
}

/// The watched names in the file at `path`; the defaults when there is none.
pub fn load_watch(path: Option<&Path>) -> Vec<String> {
    path.and_then(|p| std::fs::read_to_string(p).ok())
        .map(|t| parse_watch(&t))
        .unwrap_or_else(|| WATCH_DEFAULT.iter().map(|w| w.to_string()).collect())
}

/// Keeps the watched names in the file at `path` (its folder made if needed).
pub fn save_watch(path: &Path, names: &[String]) -> Result<(), String> {
    if let Some(d) = path.parent() {
        std::fs::create_dir_all(d).map_err(|e| format!("{}: {e}", d.display()))?;
    }
    std::fs::write(path, watch_text(names)).map_err(|e| format!("{}: {e}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elm::Elm;
    use crate::notify::tests::StandIn;
    use crate::sim::SimCar;
    use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
    use std::sync::{Arc, Mutex};

    fn scratch(tag: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!(
            "icarus-drive-{tag}-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    /// The simulated vehicle's built-in set, as used on it (no user folder, no home).
    fn sim_modules() -> Modules {
        let profiles = crate::vehicles::builtin();
        let set = moddata::load_set(moddata::SIM_SET, None, &profiles)
            .unwrap()
            .on_vehicle(&profiles, Some("toyota"));
        let year = crate::vin::decode(crate::sim::VIN)
            .ok()
            .and_then(|d| d.year);
        let scope = Scope {
            year,
            ..Default::default()
        };
        fn never() -> Option<String> {
            None
        }
        Modules::new(
            set,
            scope,
            Pacer::new(Instant::now, std::thread::sleep, never),
        )
    }

    fn offered(link: &mut dyn Link) -> Vec<u8> {
        let mut v: Vec<u8> = j1979::supported(link)
            .unwrap()
            .into_iter()
            .flat_map(|s| s.2)
            .filter(|p| j1979::pid(*p).is_some())
            .collect();
        v.sort();
        v.dedup();
        v
    }

    fn plan(link: &mut dyn Link, dir: &Path, modules: bool) -> Plan {
        Plan {
            offered: offered(link),
            adapter: link.adapter(),
            modules: modules.then(sim_modules),
            gps: None,
            gps_optional: false,
            dir: dir.to_path_buf(),
            keep_awake: false,
            vin: j1979::vin(link).ok().flatten(),
        }
    }

    /// ICA-44: standard and module columns side by side, the raw answers kept, and the file decoded again with a changed
    /// definition gives the new values from the old answers.
    #[test]
    fn a_drive_has_standard_and_module_columns_and_decodes_again() {
        let dir = scratch("modules");
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        let mut r = Recorder::start(plan(&mut elm, &dir, true)).unwrap();
        assert!(
            r.left_out()
                .iter()
                .any(|w| w.contains("700 22 1F05") && w.contains("session")),
            "{:?}",
            r.left_out()
        );
        let t0 = Instant::now();
        // long enough for every module request to come round (one each MODULE_STEP)
        while t0.elapsed() < Duration::from_secs(4) {
            r.sweep(&mut elm, &[], t0.elapsed().as_secs_f64()).unwrap();
        }
        let rows = r.rows;
        let paths = r.close().unwrap();
        assert_eq!(paths.len(), 1);
        let text = std::fs::read_to_string(&paths[0]).unwrap();
        let year = crate::vin::decode(crate::sim::VIN)
            .ok()
            .and_then(|d| d.year);
        assert!(year.is_some());
        assert_eq!(module_data_of(&text), Some(("sim".into(), year)));
        let s = drivelog::series(&text).unwrap();
        assert_eq!(s.t.len(), rows, "every row held");
        let rpm = s.find(&["engine speed [rpm]"]).unwrap();
        assert_eq!(
            s.names[rpm], "Engine speed [rpm]",
            "the standard columns first"
        );
        assert!(s.cols[rpm].iter().all(|v| v.is_some_and(|x| x > 700.0)));
        let knock = s
            .find(&["knock correction (700 22 1f01)"])
            .expect("a module column");
        let atf = s
            .find(&["transmission fluid temperature (701 22 1f11)"])
            .unwrap();
        let raw = s.find(&["700 22 1f01 answer [hex]"]).unwrap();
        assert!(
            s.cols[knock].iter().any(|v| *v == Some(-2.0)),
            "{:?}",
            s.cols[knock]
        );
        assert!(s.cols[atf].iter().any(|v| v.is_some()));
        assert!(
            s.cols[raw].iter().all(Option::is_none),
            "a raw answer is never read as a number"
        );
        assert!(
            !s.names
                .iter()
                .any(|n| n.contains("1F05") || n.contains("1F04")),
            "a request behind a session, or for other years, has no column: {:?}",
            s.names
        );
        assert!(
            drivelog::summarize(&text).is_ok(),
            "the summary reads it as any drive"
        );
        // the module values come round less often than every row (in turn; a watched one, knock correction, no more often
        // than the pacer's floor), the fast ones every row
        let trim = s
            .find(&["short-term fuel trim, bank 1 (700 22 1f02)"])
            .unwrap();
        let knocks = s.cols[knock].iter().filter(|v| v.is_some()).count();
        let trims = s.cols[trim].iter().filter(|v| v.is_some()).count();
        assert!(knocks > 1 && knocks < rows, "{knocks} of {rows}");
        assert!(trims > 1 && trims < knocks, "{trims} of {rows}");

        // the set improved: knock correction is in quarter degrees, and a value is added from the same answer
        let mut files: Vec<(String, String)> = crate::sim::data()
            .iter()
            .map(|(a, b)| (a.to_string(), b.to_string()))
            .collect();
        for f in files.iter_mut().filter(|f| f.0 == "700.toml") {
            f.1 = f.1.replacen(
                "  signed = true\n  div = 2.0",
                "  signed = true\n  div = 4.0",
                1,
            );
            f.1 = f.1.replacen(
                "[[request.value]]\n  id = \"SIM_BATT\"",
                "[[request.value]]\n  id = \"SIM_B5\"\n  name = \"byte five\"\n  byte = 5\n  bits = 8\n\n  [[request.value]]\n  id = \"SIM_BATT\"",
                1,
            );
        }
        let (_, modules, problems) = moddata::read_files(&files, "test").unwrap();
        assert!(problems.is_empty(), "{problems:?}");
        let again = redecode(&text, &modules, None).unwrap();
        let s2 = drivelog::series(&again).unwrap();
        assert_eq!(s2.t, s.t, "the same rows");
        let k2 = s2.find(&["knock correction (700 22 1f01)"]).unwrap();
        for (a, b) in s.cols[knock].iter().zip(&s2.cols[k2]) {
            assert_eq!(
                a.map(|x| x / 2.0),
                *b,
                "the new scale, from the old answers"
            );
        }
        let b5 = s2.find(&["byte five (700 22 1f01)"]).unwrap();
        assert!(s2.cols[b5].contains(&Some(252.0)), "-4 as a byte");
        let rpm2 = s2.find(&["engine speed [rpm]"]).unwrap();
        assert_eq!(s2.cols[rpm2], s.cols[rpm], "standard values as they were");
        assert_eq!(
            s2.names
                .iter()
                .filter(|n| n.contains("knock correction"))
                .count(),
            1,
            "the old column replaced, not kept beside the new"
        );
        assert!(redecode("time_s,\"Engine speed [rpm]\"\n0,700\n", &modules, None).is_err());
        std::fs::remove_dir_all(&dir).unwrap();
    }

    /// The engine's state, switched by the test, and a count of opens (the adapter "gone" for a while: opens refused).
    struct Bench {
        engine: Arc<AtomicBool>,
        dir: PathBuf,
        opens: Arc<AtomicU32>,
        refuse: Arc<AtomicBool>,
        /// The adapter's first connection dies after this many commands (a glitch: ICA-112).
        die_after: Option<u32>,
        /// The adapter comes back on a vehicle with this VIN (connections after the first).
        later_vin: Option<&'static str>,
        /// A GPS asked for, allowed to be missing (as `--auto` allows it).
        gps: Option<String>,
        stop_at: Instant,
        said: Arc<Mutex<Vec<String>>>,
    }

    impl Driver for Bench {
        fn open(&mut self) -> Result<Box<dyn Link>, String> {
            if self.refuse.load(Ordering::SeqCst) {
                return Err("no adapter".into());
            }
            let n = self.opens.fetch_add(1, Ordering::SeqCst);
            let e = self.engine.clone();
            let mut car = SimCar::new().with_engine(Arc::new(move || e.load(Ordering::SeqCst)));
            if n == 0 {
                if let Some(k) = self.die_after {
                    car = car.dying_after(k);
                }
            } else if let Some(v) = self.later_vin {
                car = car.with_vin(v);
            }
            Elm::connect(Box::new(car))
                .map(|e| Box::new(e) as Box<dyn Link>)
                .map_err(|e| e.to_string())
        }
        fn plan(&mut self, link: &mut dyn Link) -> Result<Plan, String> {
            let mut p = plan(link, &self.dir, true);
            p.gps = self.gps.clone();
            p.gps_optional = true;
            Ok(p)
        }
        fn stop(&self) -> Option<String> {
            (Instant::now() >= self.stop_at).then(|| "the test is over".into())
        }
        fn say(&mut self, line: &str) {
            self.said.lock().unwrap().push(line.to_string());
        }
    }

    fn quick(auto: bool) -> Config {
        Config {
            auto,
            off_after: Duration::from_millis(1500),
            retry: Duration::from_millis(200),
            idle_every: Duration::from_millis(100),
            glitches: 3,
            prune_after: 60.0,
        }
    }

    /// ICA-208's done: on the simulated vehicle, engine off, then running, then off: the recording starts by itself once
    /// the engine runs, holds every row, and ends with its file closed once the engine has been off for the set time; the
    /// notifications say started and stopped (with the rows) once each.
    #[test]
    fn a_drive_records_itself_from_engine_start_to_engine_off() {
        let dir = scratch("auto");
        let engine = Arc::new(AtomicBool::new(false));
        let said = Arc::new(Mutex::new(Vec::new()));
        let mut bench = Bench {
            engine: engine.clone(),
            dir: dir.clone(),
            opens: Default::default(),
            refuse: Default::default(),
            die_after: None,
            later_vin: None,
            gps: None,
            stop_at: Instant::now() + Duration::from_secs(8),
            said: said.clone(),
        };
        let desk = StandIn::every();
        let mut n = Notifier::new(false, Box::new(desk.clone()));
        let e = engine.clone();
        let script = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(1200));
            e.store(true, Ordering::SeqCst);
            std::thread::sleep(Duration::from_millis(2500));
            e.store(false, Ordering::SeqCst);
        });
        let files = run(&quick(true), &mut bench, &mut n).unwrap();
        script.join().unwrap();
        assert_eq!(files.len(), 1, "one drive, one part: {files:?}");
        let text = std::fs::read_to_string(&files[0]).unwrap();
        let s = drivelog::series(&text).unwrap();
        let rpm = s.find(&["engine speed [rpm]"]).unwrap();
        assert!(
            s.cols[rpm][0].is_some_and(|r| r >= RUNNING_RPM),
            "it began with the engine running, not before"
        );
        assert!(
            s.cols[rpm].iter().rev().take(2).all(|r| *r == Some(0.0)),
            "and holds the engine stopping"
        );
        let shown = desk.shown();
        let titles: Vec<&str> = shown.iter().map(|x| x.0.as_str()).collect();
        assert_eq!(
            titles,
            ["Recording started", "Recording stopped"],
            "{shown:?}"
        );
        assert_eq!(
            shown[1].1,
            format!(
                "{} rows saved ({}).",
                s.t.len(),
                files[0].file_name().unwrap().to_string_lossy()
            ),
            "every row held"
        );
        let said = said.lock().unwrap();
        assert!(
            said.iter().any(|l| l.contains("the drive is over")),
            "{said:?}"
        );
        // closed: nothing is added after the loop ended
        let len = std::fs::metadata(&files[0]).unwrap().len();
        std::thread::sleep(Duration::from_millis(100));
        assert_eq!(std::fs::metadata(&files[0]).unwrap().len(), len);
        std::fs::remove_dir_all(&dir).unwrap();
    }

    /// The engine stops and starts again within the drive: the rows after it go to a part file of their own.
    #[test]
    fn an_ignition_cycle_within_a_drive_is_a_part_file() {
        let dir = scratch("parts");
        let engine = Arc::new(AtomicBool::new(true));
        let mut bench = Bench {
            engine: engine.clone(),
            dir: dir.clone(),
            opens: Default::default(),
            refuse: Default::default(),
            die_after: None,
            later_vin: None,
            gps: None,
            stop_at: Instant::now() + Duration::from_secs(3),
            said: Default::default(),
        };
        let mut n = Notifier::new(true, Box::new(StandIn::every()));
        let e = engine.clone();
        let script = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(800));
            e.store(false, Ordering::SeqCst);
            std::thread::sleep(Duration::from_millis(600));
            e.store(true, Ordering::SeqCst);
        });
        let mut cfg = quick(true);
        cfg.off_after = Duration::from_secs(10);
        let files = run(&cfg, &mut bench, &mut n).unwrap();
        script.join().unwrap();
        assert_eq!(files.len(), 2, "{files:?}");
        assert!(files[1].to_string_lossy().ends_with("-part2.csv"));
        let second = std::fs::read_to_string(&files[1]).unwrap();
        assert!(second.contains("# part 2 of the drive"));
        let s = drivelog::series(&second).unwrap();
        let rpm = s.find(&["engine speed [rpm]"]).unwrap();
        assert!(s.cols[rpm][0].is_some_and(|r| r >= RUNNING_RPM));
        assert_eq!(s.t[0], 0.0, "each part's time from its start");
        std::fs::remove_dir_all(&dir).unwrap();
    }

    /// ICA-112 and the manual form: a recording started now goes on through the adapter dying (opened again, the same
    /// file), the stall and the resumption are said once each, and the end (as Ctrl+C or SIGTERM gives it) closes the file.
    #[test]
    fn a_recording_goes_on_through_an_adapter_that_drops() {
        let dir = scratch("glitch");
        let refuse = Arc::new(AtomicBool::new(false));
        let opens = Arc::new(AtomicU32::new(0));
        let said = Arc::new(Mutex::new(Vec::new()));
        let mut bench = Bench {
            engine: Arc::new(AtomicBool::new(true)),
            dir: dir.clone(),
            opens: opens.clone(),
            refuse: refuse.clone(),
            die_after: Some(120),
            later_vin: None,
            gps: None,
            stop_at: Instant::now() + Duration::from_secs(6),
            said: said.clone(),
        };
        let desk = StandIn::every();
        let mut n = Notifier::new(false, Box::new(desk.clone()));
        n.stall_after = 1.0;
        // the adapter stays away for a while after it died (the opens refused), then comes back
        let (r2, o2) = (refuse.clone(), opens.clone());
        let watcher = std::thread::spawn(move || {
            let end = Instant::now() + Duration::from_secs(5);
            while Instant::now() < end {
                if o2.load(Ordering::SeqCst) == 1 {
                    // connected once: refuse the next opens for a while (from the first connection: long enough past its
                    // death, after 120 commands, for a stall)
                    r2.store(true, Ordering::SeqCst);
                    std::thread::sleep(Duration::from_millis(2500));
                    r2.store(false, Ordering::SeqCst);
                    return;
                }
                std::thread::sleep(Duration::from_millis(10));
            }
        });
        let files = run(&quick(false), &mut bench, &mut n).unwrap();
        watcher.join().unwrap();
        assert_eq!(files.len(), 1, "the same recording: {files:?}");
        assert!(opens.load(Ordering::SeqCst) >= 2, "opened again");
        let titles: Vec<String> = desk.shown().into_iter().map(|x| x.0).collect();
        assert_eq!(
            titles,
            [
                "Recording started",
                "Recording stalled",
                "Recording resumed",
                "Recording stopped"
            ]
        );
        let said = said.lock().unwrap();
        assert!(
            said.iter()
                .any(|l| l.contains("the recording goes on when it is back")),
            "{said:?}"
        );
        let s = drivelog::series(&std::fs::read_to_string(&files[0]).unwrap()).unwrap();
        let gap = s.t.windows(2).map(|w| w[1] - w[0]).fold(0.0f64, f64::max);
        assert!(
            gap > 1.0,
            "rows before and after the gap, in one file: {gap}"
        );
        std::fs::remove_dir_all(&dir).unwrap();
    }

    /// Review of ICA-208: the adapter comes back on another vehicle (its VIN differs) after dropping mid-recording: the
    /// recording ends there, and the next one (a file of its own) is that vehicle's.
    #[test]
    fn another_vehicle_after_the_adapter_drops_ends_the_recording() {
        let dir = scratch("vin");
        let said = Arc::new(Mutex::new(Vec::new()));
        let mut bench = Bench {
            engine: Arc::new(AtomicBool::new(true)),
            dir: dir.clone(),
            opens: Default::default(),
            refuse: Default::default(),
            die_after: Some(120),
            later_vin: Some("5TFICARUSDONOR001"),
            gps: None,
            stop_at: Instant::now() + Duration::from_secs(3),
            said: said.clone(),
        };
        let desk = StandIn::every();
        let mut n = Notifier::new(false, Box::new(desk.clone()));
        let files = run(&quick(false), &mut bench, &mut n).unwrap();
        assert_eq!(files.len(), 2, "{files:?}");
        let said = said.lock().unwrap();
        assert!(
            said.iter()
                .any(|l| l.contains("another vehicle (5TFICARUSDONOR001)")),
            "{said:?}"
        );
        let titles: Vec<String> = desk.shown().into_iter().map(|x| x.0).collect();
        assert_eq!(
            titles.iter().filter(|t| *t == "Recording stopped").count(),
            2,
            "{titles:?}"
        );
        std::fs::remove_dir_all(&dir).unwrap();
    }

    /// Review of ICA-208: by itself (`--auto`), a GPS that does not open is gone without: the drive is recorded with no
    /// position columns, and that is said and shown once.
    #[test]
    fn a_missing_gps_does_not_hold_up_a_drive() {
        let dir = scratch("gps");
        let said = Arc::new(Mutex::new(Vec::new()));
        let mut bench = Bench {
            engine: Arc::new(AtomicBool::new(true)),
            dir: dir.clone(),
            opens: Default::default(),
            refuse: Default::default(),
            die_after: None,
            later_vin: None,
            gps: Some("/nonexistent-icarus-test/ttyGPS".into()),
            stop_at: Instant::now() + Duration::from_millis(1500),
            said: said.clone(),
        };
        let desk = StandIn::every();
        let mut n = Notifier::new(false, Box::new(desk.clone()));
        let files = run(&quick(true), &mut bench, &mut n).unwrap();
        assert_eq!(files.len(), 1);
        let s = drivelog::series(&std::fs::read_to_string(&files[0]).unwrap()).unwrap();
        assert!(!s.t.is_empty() && s.find(&["gps latitude"]).is_none());
        let said = said.lock().unwrap();
        assert!(
            said.iter()
                .any(|l| l.contains("the GPS (/nonexistent-icarus-test/ttyGPS) did not open")),
            "{said:?}"
        );
        let titles: Vec<String> = desk.shown().into_iter().map(|x| x.0).collect();
        assert_eq!(
            titles,
            [
                "Recording started",
                "Recording without GPS",
                "Recording stopped"
            ]
        );
        // asked for by hand (the window's tick), it stops the recording before a file is made
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        let mut p = plan(&mut elm, &dir, false);
        p.gps = Some("/nonexistent-icarus-test/ttyGPS".into());
        assert!(Recorder::start(p).is_err());
        std::fs::remove_dir_all(&dir).unwrap();
    }

    /// ICA-87 in the shared recorder: a value read in turn that never answers is no longer asked after the watch.
    #[test]
    fn a_value_that_never_answers_stops_being_read() {
        let dir = scratch("prune");
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        let mut p = plan(&mut elm, &dir, false);
        p.offered.push(0x3D);
        let mut r = Recorder::start_pruning(p, 1.0).unwrap();
        assert!(r.rest.contains(&0x3D));
        let t0 = Instant::now();
        let mut notes = Vec::new();
        while t0.elapsed() < Duration::from_millis(2500) {
            notes.extend(
                r.sweep(&mut elm, &[], t0.elapsed().as_secs_f64())
                    .unwrap()
                    .notes,
            );
        }
        assert!(
            !r.rest.contains(&0x3D) && r.rest.contains(&0x46),
            "{:?}",
            r.rest
        );
        assert!(notes.iter().any(|n| n.contains("PID 3D")), "{notes:?}");
        r.close().unwrap();
        std::fs::remove_dir_all(&dir).unwrap();
    }

    /// One program per adapter: a recording that cannot have the port says so, naming it in use, and keeps waiting.
    #[test]
    fn a_port_in_use_is_said() {
        let (_master, path) = crate::port::tests::pty();
        let held = crate::port::open_serial(&path, 115200).unwrap();
        struct Busy(String, Vec<String>);
        impl Driver for Busy {
            fn open(&mut self) -> Result<Box<dyn Link>, String> {
                crate::elm::connect_target(&crate::port::Target::Serial {
                    path: self.0.clone(),
                    baud: 115200,
                })
                .map(|(e, _)| Box::new(e) as Box<dyn Link>)
            }
            fn plan(&mut self, _: &mut dyn Link) -> Result<Plan, String> {
                unreachable!()
            }
            fn stop(&self) -> Option<String> {
                (!self.1.is_empty()).then(|| "enough".into())
            }
            fn say(&mut self, line: &str) {
                self.1.push(line.into());
            }
        }
        let mut b = Busy(path, Vec::new());
        let mut n = Notifier::new(true, Box::new(StandIn::default()));
        assert!(run(&quick(true), &mut b, &mut n).unwrap().is_empty());
        assert!(
            b.1[0].starts_with("Waiting for the adapter") && b.1[0].contains("open"),
            "{:?}",
            b.1
        );
        assert!(
            b.1[0].contains("one connection at a time") || b.1[0].contains("one program at a time")
        );
        drop(held);
    }

    /// The simulated vehicle at an adapter's pace (ICA-218): each AT command takes `at` and each request to the vehicle
    /// `req`, roughly as an ELM327 over Bluetooth does; the commands are counted.
    struct Paced {
        car: SimCar,
        at: Duration,
        req: Duration,
        sent: Arc<AtomicU32>,
    }

    impl crate::port::Port for Paced {
        fn send(&mut self, data: &[u8]) -> std::io::Result<()> {
            let cmd = String::from_utf8_lossy(data).trim().to_uppercase();
            if !cmd.is_empty() {
                self.sent.fetch_add(1, Ordering::SeqCst);
                let at = cmd.starts_with("AT") || cmd.starts_with("ST");
                std::thread::sleep(if at { self.at } else { self.req });
            }
            self.car.send(data)
        }
        fn recv(&mut self, buf: &mut [u8], timeout: Duration) -> std::io::Result<usize> {
            self.car.recv(buf, timeout)
        }
    }

    fn paced() -> (Elm, Arc<AtomicU32>) {
        let sent = Arc::new(AtomicU32::new(0));
        let port = Paced {
            car: SimCar::new(),
            at: Duration::from_millis(8),
            req: Duration::from_millis(40),
            sent: sent.clone(),
        };
        (Elm::connect(Box::new(port)).unwrap(), sent)
    }

    /// A recording of `secs` on the paced vehicle watching `watch` (None: the defaults): its rows a second, the file's
    /// series, what it said when it started and the notes of its sweeps.
    fn paced_drive(
        tag: &str,
        watch: Option<&[&str]>,
        secs: f64,
    ) -> (f64, drivelog::Series, Vec<String>, Vec<String>) {
        let dir = scratch(tag);
        let (mut elm, _) = paced();
        let mut p = plan(&mut elm, &dir, true);
        if let Some(w) = watch {
            p.modules = p.modules.map(|m| m.watching(w));
        }
        let mut r = Recorder::start(p).unwrap();
        r.rate_after = secs / 2.0;
        let said = r.watch_lines();
        let mut notes = Vec::new();
        let t0 = Instant::now();
        while t0.elapsed().as_secs_f64() < secs {
            let s = r.sweep(&mut elm, &[], t0.elapsed().as_secs_f64()).unwrap();
            notes.extend(s.notes.into_iter().chain(s.rate));
        }
        let rate = r.rows as f64 / t0.elapsed().as_secs_f64();
        let paths = r.close().unwrap();
        let s = drivelog::series(&std::fs::read_to_string(&paths[0]).unwrap()).unwrap();
        std::fs::remove_dir_all(&dir).unwrap();
        (rate, s, said, notes)
    }

    /// ICA-218's done on the simulated vehicle: the watched module values (by default knock correction, and the gear and
    /// lock-up in the transmission's request) are read in every sweep while the others rotate, which is said at the start,
    /// and the rows a second (said too) stay within `RATE_KEPT` of a recording that watches nothing (as before ICA-218).
    #[test]
    fn watched_module_values_come_every_sweep_and_the_rest_in_turn() {
        let (before, old, said_before, _) = paced_drive("unwatched", Some(&[]), 4.0);
        assert!(
            said_before[0].contains("nothing watched"),
            "{said_before:?}"
        );
        let (rate, s, said, notes) = paced_drive("watched", None, 4.0);
        assert!(
            said[0].contains("every sweep")
                && said[0].contains("knock correction (700 22 1F01)")
                && said[0].contains("gear, torque converter lock-up (701 22 1F11)"),
            "{said:?}"
        );
        let rows = s.t.len();
        assert!(rows >= 8, "{rows} rows");
        let col = |name: &str| s.find(&[name]).unwrap_or_else(|| panic!("{name}"));
        let count = |c: usize| s.cols[c].iter().filter(|v| v.is_some()).count();
        // watched: in every row (both requests, each carrying a watched value)
        for name in [
            "knock correction (700 22 1f01)",
            "transmission fluid temperature (701 22 1f11)",
        ] {
            assert_eq!(count(col(name)), rows, "{name}: {:?}", s.cols[col(name)]);
        }
        // in turn: they still come, less often
        for name in [
            "short-term fuel trim, bank 1 (700 22 1f02)",
            "idle speed control (7e0 21 a7)",
        ] {
            let n = count(col(name));
            assert!(n >= 1 && n < rows, "{name}: {n} of {rows}");
        }
        // before ICA-218 knock correction came only in its turn
        let k_old = old.find(&["knock correction (700 22 1f01)"]).unwrap();
        let old_knocks = old.cols[k_old].iter().filter(|v| v.is_some()).count();
        assert!(old_knocks < old.t.len(), "{old_knocks} of {}", old.t.len());
        // the rate kept, and said
        assert!(
            rate >= RATE_KEPT * before,
            "{rate:.2} rows a second watching, {before:.2} before"
        );
        let line = notes
            .iter()
            .find(|n| n.starts_with("Recording at "))
            .expect("the rate said");
        assert!(
            line.contains("rows a second") && line.contains("the watched module values every"),
            "{line}"
        );
        eprintln!("ICA-218 on the paced simulated vehicle: {before:.2} rows a second before, {rate:.2} watching ({line})");
    }

    /// More watched requests than `PRIORITY_PER_SWEEP`: no sweep reads more than that many, and they take turns, so each
    /// still comes; a name nothing carries is said; the watch matches ignoring case, spaces and punctuation.
    #[test]
    fn a_long_watch_list_takes_turns_and_a_name_nothing_carries_is_said() {
        assert!(
            watched("Lock up state", "lockup") && watched("torque converter lock-up", "lock up")
        );
        assert!(watched("Shift SW status, parking", "shift sw status"));
        assert!(watched(
            "Knock correction, learned value",
            "KNOCK CORRECTION"
        ));
        assert!(!watched("Knock feedback value", "knock correction") && !watched("Gear", ""));
        let mut m = sim_modules().watching(&["e", "a value nobody has"]);
        let n = m.priority.len();
        assert!(n > PRIORITY_PER_SWEEP, "{n}");
        let said = m.watch_lines();
        assert!(said[0].contains("4 a sweep in turn"), "{said:?}");
        assert!(
            said[1].contains("\"a value nobody has\"") && said[1].contains("set sim"),
            "{said:?}"
        );
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        let mut seen = std::collections::BTreeSet::new();
        let mut notes = Vec::new();
        for _ in 0..4 {
            let sent = m.watched_step(&mut elm, &mut notes, None);
            assert!(
                !sent.is_empty() && sent.len() <= PRIORITY_PER_SWEEP,
                "{sent:?}"
            );
            seen.extend(sent.iter().map(|x| x.0));
            // a sweep's time on a real adapter: each watched request is due again (the pacer's floor)
            std::thread::sleep(moddata::MIN_EVERY);
        }
        let still: Vec<usize> = m.priority.iter().copied().filter(|i| m.asked[*i]).collect();
        assert!(still.iter().all(|i| seen.contains(i)), "{still:?} {seen:?}");
        // nothing in turn is watched; watching nothing puts every request back in turn
        assert!(m
            .step(&mut elm, &mut notes)
            .is_none_or(|x| !m.priority.contains(&x.0)));
        let m = m.watching::<&str>(&[]);
        assert!(m.priority.is_empty() && m.watch_lines()[0].contains("nothing watched"));
    }

    /// The window's setting and `icarus record`'s default: a name a line, `none` for nothing, the defaults when empty.
    #[test]
    fn the_watch_file_reads_back() {
        let dir = scratch("watch");
        let path = dir.join("icarus/record-watch");
        let all: Vec<String> = WATCH_DEFAULT.iter().map(|w| w.to_string()).collect();
        assert_eq!(load_watch(Some(&path)), all, "no file: the defaults");
        let mine = vec!["Knock feedback".to_string(), "gear".to_string()];
        save_watch(&path, &mine).unwrap();
        assert_eq!(load_watch(Some(&path)), mine);
        save_watch(&path, &[]).unwrap();
        assert!(load_watch(Some(&path)).is_empty(), "none: nothing watched");
        assert_eq!(parse_watch("# only a comment\n\n"), all);
        assert_eq!(parse_watch(" lock up # the converter\n"), vec!["lock up"]);
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn the_module_line_and_cells() {
        assert_eq!(
            module_data_of(&format!(
                "# adapter: x\n{}\ntime_s\n",
                module_line("toyota-tacoma", Some(2017))
            )),
            Some(("toyota-tacoma".into(), Some(2017)))
        );
        assert_eq!(
            module_data_of(&format!("{}\n", module_line("s", None))),
            Some(("s".into(), None))
        );
        assert_eq!(module_data_of("time_s\n"), None);
        assert_eq!(cell_text(&Cell::Text("a, \"b\"".into())), "\"a, b\"");
        assert_eq!(cell_text(&Cell::Num(1.234)), "1.23");
        let (fast, rest) = drive_set(&[0x05, 0x06, 0x0C, 0x2F, 0x34]);
        assert_eq!(
            (fast, rest),
            (vec![0x0C, 0x06, 0x34], vec![0x05, 0x2F]),
            "engine speed first, the rest in turn"
        );
    }
}
