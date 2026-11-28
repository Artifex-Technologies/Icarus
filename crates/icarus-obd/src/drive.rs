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

/// The module values a recording reads every sweep unless told otherwise (ICA-218): knock feedback, the gear, lock-up,
/// the shift lever's switches (on a Toyota they carry neutral and sport mode too) and sport mode, what has to be tied to a
/// moment of the drive. A learned value (knock correction, learned) changes slowly and stays in turn. Each name is
/// matched against a value's name as whole words (`watched`).
pub const WATCH_DEFAULT: &[&str] = &[
    "knock feedback",
    "gear",
    "lock up",
    "shift sw status",
    "sports mode",
    "sports shift",
];

/// The most watched requests one sweep reads (ICA-218), one of the bounds on what watching costs a row (`MODULE_BUDGET`
/// is the other). Each is one more module request in the sweep: its round trip, the switch of the adapter's header when
/// it is another module's (five commands there, three back), and the pacer's least gap (`moddata::MIN_GAP`, from the end
/// of one answer to the next request), so on a Bluetooth adapter each adds roughly a tenth of a second to the row. With
/// more watched requests than this they take turns: a long `--watch` list slows its own values, not the whole recording.
/// The pacer is not bypassed: a watched request goes through the same gates and is never asked more often than the
/// pacer's floor (`moddata::MIN_EVERY`, four times a second), and never waits for a busy module (`Pacer::would_wait`).
pub const PRIORITY_PER_SWEEP: usize = 2;

/// The time budget of a sweep's module requests (ICA-218 review: the standard values keep their pace): the request in
/// turn (when due) and the first watched one always go, a second watched one only while they have taken less than this.
/// A request that fails (the link lost or timing out) ends them for the sweep.
pub const MODULE_BUDGET: Duration = Duration::from_millis(150);

/// How long a recording leaves a module alone after it answered busy (7F xx 21), doubling with each busy answer in a row
/// up to `BUSY_REST_MOST` (ICA-218 review). One busy read takes `uds::WINDOW_SENDS` sends 300 ms apart, near a second of
/// the sweep, so it is tried again only now and then; the pacer's own slowdown is never waited out.
pub const BUSY_REST: Duration = Duration::from_secs(5);
pub const BUSY_REST_MOST: Duration = Duration::from_secs(60);

/// A request whose `every` is this many seconds or more (the source says its values change over minutes) is never
/// watched; it stays in turn.
pub const WATCH_SLOWEST_EVERY: f64 = 60.0;

/// The rows-per-second line comes once the recording has run this long (seconds).
pub const RATE_AFTER_S: f64 = 10.0;

/// The share of its rows a second a recording keeps with the default watch, against one that watches nothing (ICA-218's
/// done, tested on the simulated vehicle at an adapter's pace, 15 ms an AT command and 53 ms a request, where a
/// recording that watches nothing makes about 3.7 rows a second with a request in turn every other row, as the truck's
/// did at 3). Most of a watched request's cost is the switch of the adapter's header to its module (five commands); the
/// first build, which also put a request in turn in every row, kept under half. Here: about 0.6, the watched values in
/// every row.
pub const RATE_KEPT: f64 = 0.55;

/// A name's words as they are matched: lower case, letters and digits, punctuation as a space, and "lock up" as one
/// word ("lock-up", "lockup").
fn words(s: &str) -> Vec<String> {
    let raw: Vec<String> = s
        .split(|c: char| !c.is_alphanumeric())
        .filter(|w| !w.is_empty())
        .map(str::to_lowercase)
        .collect();
    let mut out = Vec::new();
    let mut k = 0;
    while k < raw.len() {
        if raw[k] == "lock" && raw.get(k + 1).is_some_and(|w| w == "up") {
            out.push("lockup".to_string());
            k += 2;
        } else {
            out.push(raw[k].clone());
            k += 1;
        }
    }
    out
}

/// Whether a value's name holds a watched name as whole words, in order ("lock up" finds "Lock up state" and "torque
/// converter lock-up", not "Clock update"; "gear" finds "Current gear" and "Gear ratio", not "Gearbox oil").
pub fn watched(name: &str, watch: &str) -> bool {
    let w = words(watch);
    let n = words(name);
    !w.is_empty() && n.windows(w.len()).any(|x| x == w.as_slice())
}

/// Whether a watched name is `none` (nothing watched, when it is the only one).
fn is_none(name: &str) -> bool {
    words(name) == ["none"]
}

/// Watched names as given (`--watch`, the file): `none` alone, or only `none`, is nothing watched; among other names it
/// is kept, and `Modules::watching` says it was ignored.
pub fn watch_names(names: Vec<String>) -> Vec<String> {
    if !names.is_empty() && names.iter().all(|n| is_none(n)) {
        Vec::new()
    } else {
        names
    }
}

/// Where a module is asked: (request ID, address byte).
type Where = (u32, Option<u8>);

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
    /// where the next sweep's turn starts.
    priority: Vec<usize>,
    prio_at: usize,
    /// Each item's answers so far (for the rows-per-second line).
    answers: Vec<u32>,
    /// What is watched, in words; why each watched name that found no request did not (in words); whether a `none`
    /// among other names was ignored.
    watching: Vec<String>,
    unmatched: Vec<String>,
    none_ignored: bool,
    /// Modules that answered busy (7F xx 21), left alone until then: (until, busy answers in a row).
    resting: std::collections::HashMap<u32, (Instant, u32)>,
    /// The last sweep sent a request in turn.
    turn_last: bool,
    /// The requests the gates kept out: (where, its values' names), so a watched name only they carry is said as such.
    gated: Vec<(String, Vec<String>)>,
    /// What the gates kept out, in words (said once when the recording starts).
    pub left_out: Vec<String>,
}

impl Modules {
    /// The set as used on the vehicle (`vehicledata::choose_for`, or `Loaded::on_vehicle`), what the vehicle may be asked
    /// (`scope`), and the pacer every request goes through. The default values are watched (`WATCH_DEFAULT`).
    pub fn new(set: Loaded, scope: Scope, pacer: Pacer) -> Modules {
        let (mut items, mut left_out, mut gated) = (Vec::new(), Vec::new(), Vec::new());
        for (mi, m) in set.modules.iter().enumerate() {
            for (ri, r) in m.requests.iter().enumerate() {
                if r.values.is_empty() {
                    continue;
                }
                match moddata::may_send(&set.gates, m, r, &scope) {
                    Ok(()) => items.push((mi, ri)),
                    Err(o) => {
                        let at = format!("{} {}", m.module.label(), r.label());
                        left_out.push(format!("{at}: {}", o.describe()));
                        gated.push((at, r.values.iter().map(|v| v.name.clone()).collect()));
                    }
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
            none_ignored: false,
            resting: Default::default(),
            turn_last: false,
            gated,
            left_out,
        }
        .watching(WATCH_DEFAULT)
    }

    /// Reads requests carrying a value named by one of `watch` (`watched`) every sweep (ICA-218), the others in turn; an
    /// empty list reads everything in turn, as before ICA-218. For each name, one request a module: the one asked most
    /// often by its definition (`every`), then the shortest answer, so a long record carrying the name among twenty other
    /// values is not read every sweep; a request whose `every` is `WATCH_SLOWEST_EVERY` or more is never watched.
    pub fn watching<S: AsRef<str>>(mut self, watch: &[S]) -> Modules {
        let mut names: Vec<&str> = Vec::new();
        let mut nones = 0;
        for n in watch.iter().map(|n| n.as_ref().trim()) {
            if words(n).is_empty() {
                continue;
            }
            if is_none(n) {
                nones += 1;
            } else if !names.iter().any(|x| words(x) == words(n)) {
                names.push(n);
            }
        }
        // `none` turns watching off only as the only name (`watch_names` makes that an empty list)
        self.none_ignored = nones > 0 && !names.is_empty();
        let set = self.set_name().to_string();
        let mut chosen: Vec<(Where, usize, Vec<&str>)> = Vec::new();
        let mut unmatched = Vec::new();
        for w in &names {
            // the best request carrying it on each module: (module, item, (every, answer length))
            let mut best: Vec<(Where, usize, (f64, usize))> = Vec::new();
            let mut slow: Vec<String> = Vec::new();
            for i in 0..self.items.len() {
                let (m, r) = self.item(i);
                if !r.values.iter().any(|v| watched(&v.name, w)) {
                    continue;
                }
                if r.every.is_some_and(|e| e >= WATCH_SLOWEST_EVERY) {
                    slow.push(format!("{} {}", m.module.label(), r.label()));
                    continue;
                }
                let rank = (r.every.unwrap_or(f64::MAX), r.min_len.unwrap_or(usize::MAX));
                let at = (m.module.tx, m.module.ext);
                match best.iter_mut().find(|b| b.0 == at) {
                    Some(b) if rank < b.2 => (b.1, b.2) = (i, rank),
                    Some(_) => {}
                    None => best.push((at, i, rank)),
                }
            }
            if best.is_empty() {
                let gated: Vec<&str> = self
                    .gated
                    .iter()
                    .filter(|g| g.1.iter().any(|v| watched(v, w)))
                    .map(|g| g.0.as_str())
                    .collect();
                unmatched.push(if !slow.is_empty() {
                    format!(
                        "\"{w}\" is only in request(s) read every {WATCH_SLOWEST_EVERY:.0} s or less often ({}), left in turn",
                        slow.join(", ")
                    )
                } else if !gated.is_empty() {
                    format!(
                        "\"{w}\" is only in request(s) the gates leave out ({}), not read",
                        gated.join(", ")
                    )
                } else {
                    format!("no value of the set {set} is named \"{w}\"")
                });
            }
            for (at, i, _) in best {
                match chosen.iter_mut().find(|c| c.1 == i) {
                    Some(c) => c.2.push(w),
                    None => chosen.push((at, i, vec![w])),
                }
            }
        }
        // by module (a set may name one module in more than one file): the header switched once for its requests
        chosen.sort_by_key(|c| (c.0, c.1));
        self.watching = chosen
            .iter()
            .map(|(_, i, ws)| {
                let (m, r) = self.item(*i);
                let hits: Vec<&str> = r
                    .values
                    .iter()
                    .filter(|v| ws.iter().any(|w| watched(&v.name, w)))
                    .map(|v| v.name.as_str())
                    .collect();
                format!("{} ({} {})", hits.join(", "), m.module.label(), r.label())
            })
            .collect();
        self.priority = chosen.iter().map(|c| c.1).collect();
        self.prio_at = 0;
        self.unmatched = unmatched;
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
        for u in &self.unmatched {
            out.push(format!("Module data: {u}: not watched."));
        }
        if self.none_ignored {
            out.push(
                "Module data: \"none\" was ignored: it turns watching off only as the only name."
                    .into(),
            );
        }
        out
    }

    fn item(&self, i: usize) -> (&ModuleDef, &RequestDef) {
        let (mi, ri) = self.items[i];
        let m = &self.set.modules[mi];
        (m, &m.requests[ri])
    }

    /// Item `i` asked, the same way in turn and watched: None when it was not due (its `every`, or for a watched one the
    /// pacer's floor), when its module is slowed for being busy (left for a later sweep: the sweep never waits it out), or
    /// when the gates or the pacer dropped it (said in `notes`; not asked again).
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
        // a busy module's slowdown (up to `moddata::MAX_SLOW`) is not waited out here, only the least gap is; and a module
        // that answered busy is left alone for a while (`BUSY_REST`): one busy read costs the sweep its retries
        let tx = m.module.tx;
        if self.pacer.would_wait(moddata::key(m, r)) > self.pacer.gap()
            || self.resting.get(&tx).is_some_and(|r| Instant::now() < r.0)
        {
            return None;
        }
        let out = moddata::fetch_request(link, &mut self.pacer, &self.set.gates, m, r, &self.scope);
        match out {
            Err(Outcome::NotDue) => None,
            // the recording is ending (Ctrl+C, SIGTERM): not a request that stopped being read
            Err(Outcome::Skipped(_)) if self.pacer.stopped().is_some() => None,
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
                if matches!(other, Err(Outcome::Refused(0x21))) {
                    // each busy answer in a row rests the module twice as long, up to BUSY_REST_MOST
                    let n = self.resting.get(&tx).map_or(0, |r| r.1) + 1;
                    let rest = (BUSY_REST * 2u32.pow((n - 1).min(8))).min(BUSY_REST_MOST);
                    self.resting.insert(tx, (Instant::now() + rest, n));
                } else {
                    self.resting.remove(&tx);
                }
                if other.is_ok() {
                    self.answers[i] += 1;
                }
                Some(other)
            }
        }
    }

    /// The module requests of one sweep, sent: the one in turn when one is due, then the watched ones in turn (at most
    /// `PRIORITY_PER_SWEEP`): the first due always, each after it only while the sweep's module requests have taken less
    /// than `MODULE_BUDGET`.
    /// A request that failed (the link lost or timing out, a garbled answer) ends them for this sweep, so the recording
    /// learns of a lost adapter from its next standard read at once. (index, its answer's data or what it did) for each.
    fn phase(
        &mut self,
        link: &mut dyn Link,
        notes: &mut Vec<String>,
    ) -> Vec<(usize, Result<Vec<u8>, Outcome>)> {
        let start = Instant::now();
        let failed = |o: &Result<Vec<u8>, Outcome>| matches!(o, Err(Outcome::Failed(_)));
        let mut out: Vec<(usize, Result<Vec<u8>, Outcome>)> = Vec::new();
        // while values are watched, a request in turn every other sweep at most: as often a row as before watching (on a
        // Bluetooth adapter at about 3 rows a second the request in turn went every second sweep), so the watched request
        // is what a row gains, not a request in turn in every row besides
        let turn = self.priority.is_empty() || !self.turn_last;
        if turn {
            out.extend(self.step(link, notes));
        }
        self.turn_last = turn && !out.is_empty();
        if out.iter().any(|x| failed(&x.1)) {
            return out;
        }
        let n = self.priority.len();
        let mut sent = 0;
        for _ in 0..n {
            if sent >= PRIORITY_PER_SWEEP || (sent > 0 && start.elapsed() >= MODULE_BUDGET) {
                break;
            }
            let i = self.priority[self.prio_at % n];
            self.prio_at += 1;
            if !self.asked[i] {
                continue;
            }
            if let Some(o) = self.ask(i, link, notes) {
                let stop = failed(&o);
                out.push((i, o));
                sent += 1;
                if stop {
                    break;
                }
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
        // the module requests: one in turn when due, then the watched ones (ICA-218), within the sweep's budget (only while
        // the vehicle answers)
        let module = match self.modules.as_mut() {
            Some(m) if !s.values.is_empty() => m.phase(link, &mut s.notes),
            _ => Vec::new(),
        };
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

/// The watched names a `record-watch` file gives: one a line, a line beginning with `#` a comment (a `#` within a name is
/// part of it: "Cylinder #1 misfire count"); `none` as the only name watches nothing (`watch_names`). A file naming
/// nothing at all gives the defaults (`WATCH_DEFAULT`).
pub fn parse_watch(text: &str) -> Vec<String> {
    let names: Vec<String> = text
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .map(str::to_string)
        .collect();
    if names.is_empty() {
        return WATCH_DEFAULT.iter().map(|w| w.to_string()).collect();
    }
    watch_names(names)
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
        // the module values in turn come round less often than every row, the fast ones every row
        let trim = s
            .find(&["short-term fuel trim, bank 1 (700 22 1f02)"])
            .unwrap();
        let knocks = s.cols[knock].iter().filter(|v| v.is_some()).count();
        let trims = s.cols[trim].iter().filter(|v| v.is_some()).count();
        assert!(knocks > 1 && knocks < rows, "{knocks} of {rows}");
        assert!(trims > 1 && trims < rows, "{trims} of {rows}");

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
    /// `req`, roughly as an ELM327 over Bluetooth does; every command is logged. From the first command starting with
    /// `mute_on` the adapter says nothing more (a link that times out rather than breaks).
    struct Paced {
        car: SimCar,
        at: Duration,
        req: Duration,
        log: Arc<Mutex<Vec<String>>>,
        mute_on: Option<&'static str>,
        muted: bool,
    }

    impl crate::port::Port for Paced {
        fn send(&mut self, data: &[u8]) -> std::io::Result<()> {
            let cmd = String::from_utf8_lossy(data).trim().to_uppercase();
            if !cmd.is_empty() {
                self.log.lock().unwrap().push(cmd.clone());
                let at = cmd.starts_with("AT") || cmd.starts_with("ST");
                std::thread::sleep(if at { self.at } else { self.req });
                if self.mute_on.is_some_and(|m| cmd.starts_with(m)) {
                    self.muted = true;
                }
            }
            if self.muted {
                return Ok(());
            }
            self.car.send(data)
        }
        fn recv(&mut self, buf: &mut [u8], timeout: Duration) -> std::io::Result<usize> {
            if self.muted {
                std::thread::sleep(timeout.min(Duration::from_millis(20)));
                return Ok(0);
            }
            self.car.recv(buf, timeout)
        }
    }

    fn paced(car: SimCar, mute_on: Option<&'static str>) -> (Elm, Arc<Mutex<Vec<String>>>) {
        let log = Arc::new(Mutex::new(Vec::new()));
        let port = Paced {
            car,
            at: Duration::from_millis(15),
            req: Duration::from_millis(53),
            log: log.clone(),
            mute_on,
            muted: false,
        };
        (Elm::connect(Box::new(port)).unwrap(), log)
    }

    /// A recording of `secs` on the paced vehicle `car` watching `watch` (None: the defaults): its rows a second, the
    /// file's series, what it said when it started and the notes of its sweeps.
    fn paced_drive(
        tag: &str,
        car: SimCar,
        watch: Option<&[&str]>,
        secs: f64,
    ) -> (f64, drivelog::Series, Vec<String>, Vec<String>) {
        let dir = scratch(tag);
        let (mut elm, _) = paced(car, None);
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
        let paths = r.close().unwrap();
        let s = drivelog::series(&std::fs::read_to_string(&paths[0]).unwrap()).unwrap();
        std::fs::remove_dir_all(&dir).unwrap();
        // the rows a second once settled: after the first 2 s (one answer pending, one refusal asked twice, at the start)
        let late: Vec<f64> = s.t.iter().copied().filter(|t| *t >= 2.0).collect();
        let rate = match (late.first(), late.last()) {
            (Some(a), Some(b)) if b > a => (late.len() - 1) as f64 / (b - a),
            _ => 0.0,
        };
        (rate, s, said, notes)
    }

    /// ICA-218's done on the simulated vehicle: the watched module values (by default the gear and lock-up, in the
    /// transmission's request) are read in every sweep while the others rotate, which is said at the start, and the rows
    /// a second (said too) stay at `RATE_KEPT` or more of a recording that watches nothing (as before ICA-218). The first
    /// build kept 0.39 here.
    #[test]
    fn watched_module_values_come_every_sweep_and_the_rest_in_turn() {
        let (before, old, said_before, _) = paced_drive("unwatched", SimCar::new(), Some(&[]), 6.0);
        assert!(
            said_before[0].contains("nothing watched"),
            "{said_before:?}"
        );
        let (rate, s, said, notes) = paced_drive("watched", SimCar::new(), None, 6.0);
        assert!(
            said[0].contains("every sweep")
                && said[0].contains("gear, torque converter lock-up (701 22 1F11)")
                && !said[0].contains("1F01"),
            "{said:?}"
        );
        let rows = s.t.len();
        assert!(rows >= 8, "{rows} rows");
        let col = |name: &str| s.find(&[name]).unwrap_or_else(|| panic!("{name}"));
        let count = |c: usize| s.cols[c].iter().filter(|v| v.is_some()).count();
        // watched: in every row
        let atf = col("transmission fluid temperature (701 22 1f11)");
        assert_eq!(count(atf), rows, "{:?}", s.cols[atf]);
        // in turn: they still come, less often
        for name in [
            "knock correction (700 22 1f01)",
            "short-term fuel trim, bank 1 (700 22 1f02)",
            "idle speed control (7e0 21 a7)",
        ] {
            let n = count(col(name));
            assert!(n >= 1 && n < rows, "{name}: {n} of {rows}");
        }
        // before ICA-218 the transmission's request came only in its turn
        let a_old = old
            .find(&["transmission fluid temperature (701 22 1f11)"])
            .unwrap();
        let old_atf = old.cols[a_old].iter().filter(|v| v.is_some()).count();
        assert!(old_atf < old.t.len(), "{old_atf} of {}", old.t.len());
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

    /// Review of ICA-218: a module busy for good (7F 22 21 to every read) does not hold the sweep up: its slowdown is
    /// never waited out, so the standard values keep their pace (no gap of several seconds between rows).
    #[test]
    fn a_busy_module_does_not_hold_the_standard_values_up() {
        let (rate, s, _, _) = paced_drive("busy", SimCar::new().busy_module(0x701), None, 5.0);
        let gap = s.t.windows(2).map(|w| w[1] - w[0]).fold(0.0f64, f64::max);
        // one busy read (four sends, 300 ms apart) can still be in a sweep; waiting out the slowdown (up to 2 s) is not
        assert!(gap < 2.0, "a gap of {gap:.2} s between rows");
        assert!(rate >= 1.5, "{rate:.2} rows a second");
        let atf = s
            .find(&["transmission fluid temperature (701 22 1f11)"])
            .unwrap();
        assert!(s.cols[atf].iter().all(Option::is_none), "busy: no answer");
        eprintln!(
            "ICA-218 review, a busy module: {rate:.2} rows a second, the longest gap {gap:.2} s"
        );
    }

    /// Review of ICA-218: a link that stops answering during the module requests ends them for that sweep (one time-out,
    /// not one a request), so the next standard read finds the adapter gone at once.
    #[test]
    fn a_link_that_times_out_ends_the_module_requests() {
        let dir = scratch("mute");
        let (mut elm, log) = paced(SimCar::new(), Some("221F01"));
        let mut p = plan(&mut elm, &dir, true);
        // two watched requests, the engine's first (by module): its time-out must end the sweep's module requests
        p.modules = p.modules.map(|m| m.watching(&["knock correction", "gear"]));
        assert_eq!(p.modules.as_ref().unwrap().priority.len(), 2);
        let mut r = Recorder::start(p).unwrap();
        let sent = |log: &Arc<Mutex<Vec<String>>>| {
            log.lock().unwrap().iter().any(|c| c.starts_with("221F01"))
        };
        let mut took = Duration::ZERO;
        for k in 0..5 {
            let t0 = Instant::now();
            let s = r.sweep(&mut elm, &[], k as f64).unwrap();
            took = t0.elapsed();
            assert!(s.row, "the standard values written");
            if sent(&log) {
                break;
            }
            std::thread::sleep(moddata::MIN_EVERY);
        }
        let log = log.lock().unwrap();
        let at = log.iter().position(|c| c.starts_with("221F01")).unwrap();
        assert!(
            !log[at + 1..]
                .iter()
                .any(|c| c.starts_with("22") || c.starts_with("21")),
            "nothing asked after the time-out: {:?}",
            &log[at..]
        );
        assert!(took < Duration::from_secs(8), "one time-out: {took:?}");
        drop(log);
        r.close().unwrap();
        std::fs::remove_dir_all(&dir).unwrap();
    }

    /// Review of ICA-218: matching by whole words, one request a module and name (the most often asked, then the shortest),
    /// never a request asked every minute or less often, at most `PRIORITY_PER_SWEEP` a sweep taking turns, nothing
    /// re-sent within the pacer's floor, and what found nothing said in the right words.
    #[test]
    fn the_watch_picks_its_requests_and_says_what_it_could_not() {
        assert!(
            watched("Lock up state", "lockup") && watched("torque converter lock-up", "lock up")
        );
        assert!(watched("Torque converter lockup", "lock-up"));
        assert!(!watched("Clock update", "lock up") && !watched("Gearbox oil", "gear"));
        assert!(
            watched("Current gear", "gear")
                && watched("Shift SW status, parking", "shift sw status")
        );
        assert!(!watched("Knock feedback value", "knock correction") && !watched("Gear", ""));
        assert!(watched("Cylinder #1 misfire count", "cylinder 1"));
        // one request a module: 22 1F06 and 22 1F07 both carry "bytes"; the shorter answer is the one watched
        let m = sim_modules().watching(&["bytes"]);
        assert_eq!(m.watching, vec!["22 1F07 (3 bytes) (700 22 1F07)"]);
        // a request asked every minute or less often is left in turn
        let mut m = sim_modules();
        for r in m.set.modules.iter_mut().flat_map(|m| m.requests.iter_mut()) {
            if r.ident == 0x1F01 {
                r.every = Some(3600.0);
            }
        }
        let m = m.watching(&["knock correction"]);
        assert!(m.priority.is_empty());
        assert!(
            m.watch_lines()[1]
                .contains("only in request(s) read every 60 s or less often (700 22 1F01)"),
            "{:?}",
            m.watch_lines()
        );
        // what found nothing: a name no value has, one only a request the gates leave out carries, a none among names
        let mut m = sim_modules().watching(&[
            "knock correction",
            "fuel trim",
            "idle speed control",
            "gear",
            "a value nobody has",
            "a value behind a session",
            "none",
        ]);
        let said = m.watch_lines();
        assert!(said[0].contains("2 a sweep in turn"), "{said:?}");
        assert!(
            said.iter()
                .any(|l| l.contains("no value of the set sim is named \"a value nobody has\"")),
            "{said:?}"
        );
        assert!(
            said.iter().any(|l| l.contains(
                "\"a value behind a session\" is only in request(s) the gates leave out (700 22 1F05)"
            )),
            "{said:?}"
        );
        assert!(
            said.iter().any(|l| l.contains("\"none\" was ignored")),
            "{said:?}"
        );
        assert_eq!(m.priority.len(), 4);
        // two a sweep at most, taking turns: within four sweeps a real adapter's time apart, each comes
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        let mut seen = std::collections::BTreeSet::new();
        let mut notes = Vec::new();
        for _ in 0..4 {
            let sent: Vec<usize> = m
                .phase(&mut elm, &mut notes)
                .into_iter()
                .map(|x| x.0)
                .filter(|i| m.priority.contains(i))
                .collect();
            assert!(
                !sent.is_empty() && sent.len() <= PRIORITY_PER_SWEEP,
                "{sent:?}"
            );
            seen.extend(sent);
            std::thread::sleep(moddata::MIN_EVERY);
        }
        let still: Vec<usize> = m.priority.iter().copied().filter(|i| m.asked[*i]).collect();
        assert!(still.iter().all(|i| seen.contains(i)), "{still:?} {seen:?}");
        // watching nothing puts every request back in turn
        let m = m.watching::<&str>(&[]);
        assert!(m.priority.is_empty() && m.watch_lines()[0].contains("nothing watched"));
    }

    /// Review of ICA-218: two sweeps in quick succession (a fast adapter) do not send a watched request twice within the
    /// pacer's floor.
    #[test]
    fn a_watched_request_is_not_sent_again_within_the_floor() {
        let mut m = sim_modules();
        let watched = m.priority.clone();
        assert_eq!(watched.len(), 1, "the transmission's request");
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        let mut notes = Vec::new();
        let first = m.phase(&mut elm, &mut notes);
        assert!(first.iter().any(|x| x.0 == watched[0]), "{first:?}");
        let t = Instant::now();
        let second = m.phase(&mut elm, &mut notes);
        assert!(t.elapsed() < moddata::MIN_EVERY);
        assert!(!second.iter().any(|x| x.0 == watched[0]), "{second:?}");
        std::thread::sleep(moddata::MIN_EVERY);
        assert!(m
            .phase(&mut elm, &mut notes)
            .iter()
            .any(|x| x.0 == watched[0]));
    }

    /// The window's setting and `icarus record`'s default: a name a line, a line beginning with # a comment (a # inside
    /// a name is part of it), `none` alone for nothing, the defaults when empty.
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
        assert_eq!(
            parse_watch("Cylinder #1 misfire count\n   # a comment\nnone\ngear\n"),
            vec!["Cylinder #1 misfire count", "none", "gear"],
            "none among names is kept (and said ignored)"
        );
        assert!(parse_watch("none\n# nothing watched\nNONE\n").is_empty());
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
