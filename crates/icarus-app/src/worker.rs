//! The vehicle side of the window: one thread owns the adapter (its reads block for up to seconds), takes commands from the
//! window and sends back what it found. The window never waits on the vehicle.

use icarus_obd::elm::{Elm, ObdError};
use icarus_obd::garage::{self, Described};
use icarus_obd::j1979::{self, DtcKind, FreezeFrame, Monitors};
use icarus_obd::port::Target;
use icarus_obd::settings::{self, SettingDef};
use icarus_obd::uds::{self, Dialect};
use icarus_obd::vehicles::{self, ModuleAddr, Profile};
use icarus_obd::vin::{self, VinInfo};
use icarus_obd::{actuator, clear, learning, report};
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender};
use std::time::{Duration, Instant};

pub enum Cmd {
    Connect(String),
    /// The make named for a bench ECU (ICA-222; None: none), for the next connection only: it is kept on that connection
    /// when its VIN reads blank (`benchmake::bind`), and goes with it.
    BenchMake(Option<String>),
    Disconnect,
    /// Scan modules: the profile to use (None: from the VIN), and whether to sweep every ID.
    Scan {
        profile: Option<String>,
        sweep: bool,
    },
    /// Poll these live values (empty stops). While a drive is recorded, the recording's sweep reads them besides its own.
    Live(Vec<u8>),
    /// Record a drive (`icarus_obd::drive`, the recorder `icarus record` uses): every value the vehicle offers, the fast ones
    /// every sweep and the rest in turn, and each module's own values; None stops it and closes the file. It goes on through
    /// adapter glitches (ICA-112: the adapter is opened again by the worker).
    Record(Option<RecordStart>),
    /// The recording's sounds and notifications silenced (true) or not (ICA-209).
    RecordQuiet(bool),
    /// Read everything again and save a report (with every module when true).
    Report(bool),
    /// Read each module's own values once (icarus_obd::vehicledata): every module of the set that fits, read only.
    ModuleData,
    /// Run a service routine (icarus_obd::service: its checks first), confirmed on the connection `on`.
    Service {
        def: icarus_obd::service::ServiceDef,
        on: Connection,
    },
    /// Start an actuator test (value for percent tests); it runs while TestHold keeps coming (at least every 0.5 s).
    TestStart {
        def: actuator::TestDef,
        value: f64,
    },
    /// Read the on-board test results (service 06).
    SelfTests,
    /// A health check (ICA-214: `icarus_obd::health`): codes, monitors, misfires and fuel trims beside this vehicle's
    /// earlier checks, kept in its folder. Reads only.
    Health,
    /// Measure battery and charging voltage for some seconds.
    Charging(u64, icarus_obd::charging::Battery),
    /// Watch the bus: through the connected adapter (None) or a Linux CAN interface (its name). Ends with BusStop or any
    /// other command. A learning watch (true) is saved as a session in the vehicle's Logs folder when it ends.
    BusStart(Option<String>, bool),
    BusStop,
    /// While watching: the person is about to do this in the other tool (names what is learned next).
    BusMark(String),
    /// Read every setting's value now.
    SettingsRead,
    /// Change a setting to these bytes (one of its values), or put back the value before its last change (None); confirmed on
    /// the connection `on`.
    SettingChange {
        def: SettingDef,
        value: Option<Vec<u8>>,
        on: Connection,
    },
    /// Clear codes: every OBD-II module (None) or one module (request ID, answer ID, its dialect), confirmed on the connection
    /// `on`. A report is saved first.
    Clear {
        target: Option<(u16, u16, Dialect)>,
        on: Connection,
    },
    TestHold,
    TestStop,
    /// The window is closing: hand back a running test, stop a recording (its file closed), and end the worker.
    Quit,
}

/// How a recording is started from the window.
#[derive(Clone, Debug, Default)]
pub struct RecordStart {
    /// A GPS receiver followed (the Tools page's source), its columns in the file only.
    pub gps: Option<String>,
    /// No sound or notification (the window's settings).
    pub quiet: bool,
    /// The module values read every sweep (the window's settings, ICA-218; empty: all in turn).
    pub watch: Vec<String>,
}

/// What the recording is doing, for the Live data page.
#[derive(Clone, Debug, PartialEq)]
pub enum Rec {
    /// Started into this file, with this many values a row.
    Started {
        path: std::path::PathBuf,
        values: usize,
    },
    /// Rows written so far.
    Rows(usize),
    /// What the recording says of its module values: the watched ones when it starts, its rows a second after the first
    /// 10 s (ICA-218).
    Said(String),
    /// Stopped (asked, or the vehicle changed): the rows and every part's file.
    Stopped {
        rows: usize,
        paths: Vec<std::path::PathBuf>,
    },
    /// Could not start, or could not close cleanly: why.
    Failed(String),
}

/// One connection to a vehicle, as a write's confirmation names it (ICA-105): its number (the worker counts one each time it
/// connects, from 1) and the VIN the vehicle gave then. A clear, a setting change or a service routine is carried out only
/// on the connection it was confirmed on; one confirmed before the adapter connected again (another vehicle, maybe) is
/// refused with nothing sent.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Connection {
    pub number: u64,
    pub vin: Option<String>,
}

/// The session floor's answer (ICA-192): `lock::session_allows_changes` in use; a test gives one of its own, since the floor
/// for the whole run would be every test's.
type Floor<'a> = &'a dyn Fn() -> Result<(), String>;

/// Whether a write confirmed on `on` may go to the connection open now (`now`), and the session floor (`floor`) allows it
/// (ICA-192); why not otherwise. Nothing is sent to ask.
fn may_write(on: &Connection, now: Option<&Connection>, floor: Floor) -> Result<(), String> {
    match now {
        None => Err("the connection it was confirmed on has ended; nothing was sent".into()),
        Some(n) if n.number != on.number => Err(
            "it was confirmed on an earlier connection (the adapter connected again since, maybe to another vehicle); \
             nothing was sent: ask again"
                .into(),
        ),
        Some(n) if n.vin != on.vin => {
            Err("it was confirmed for another vehicle (its VIN differs); nothing was sent".into())
        }
        Some(_) => floor(),
    }
}

pub struct Info {
    /// The connection this was read on (a confirmation given while it shows carries it).
    pub connection: Connection,
    pub adapter: String,
    /// The connection's port is a cable all the way (`Elm::wired`, judged by the open port): changing a setting needs one.
    pub wired: bool,
    pub voltage: Option<f64>,
    pub protocol: String,
    pub vin: Option<String>,
    pub vin_info: Option<VinInfo>,
    pub profile: Option<Profile>,
    /// The make writes follow (ICA-222): the VIN's, or the bench make named for this session where the VIN gives none;
    /// or why the two disagree.
    pub make: Result<icarus_obd::benchmake::WriteMake, String>,
    /// (module, monitors) from service 01 PID 01.
    pub monitors: Vec<(String, Monitors)>,
    pub fuel_system: Option<&'static str>,
    /// Live values the engine offers that Icarus can read.
    pub live: Vec<u8>,
    /// (kind, module, codes).
    pub codes: Vec<(&'static str, String, Vec<String>)>,
    pub freeze: Vec<FreezeFrame>,
    /// The name given to this vehicle (garage), for searches and parts links.
    pub vehicle: Option<Described>,
    /// Actuator tests known for this vehicle.
    pub tests: Vec<actuator::TestDef>,
    /// Module settings known for this vehicle.
    pub settings: Vec<SettingDef>,
    /// Service routines known for this vehicle.
    pub services: Vec<icarus_obd::service::ServiceDef>,
    /// The vehicle's code history (its folder), this reading included; or why it could not be kept.
    pub history: Result<Vec<icarus_obd::history::Span>, String>,
}

/// A setting as the Settings page shows it.
pub struct SettingView {
    pub name: String,
    /// Its value now, in words (or why it could not be read).
    pub now: String,
    /// What undo would put back, when its last change is in the vehicle's log.
    pub undo_to: Option<String>,
}

/// One ID in the bus table: (ID, count, Hz, last bytes, seconds since each byte changed).
pub type BusRow = (u32, u64, f64, Vec<u8>, Vec<f64>);

/// What the bus monitor shows, sent a few times a second.
#[derive(Default)]
pub struct BusView {
    /// (ID, count, Hz, last bytes, seconds since each byte changed).
    pub rows: Vec<BusRow>,
    pub log: Vec<String>,
    pub learned: Vec<actuator::TestDef>,
    pub settings: Vec<icarus_obd::settings::SettingDef>,
    /// Service routines the other tool started and the module took (ICA-220: named and added on the Learning page).
    pub services: Vec<icarus_obd::service::ServiceDef>,
    pub notes: Vec<String>,
    /// Where a finished learning session was saved (or why it was not).
    pub saved: Option<Result<String, String>>,
    pub learning: bool,
    pub frames: u64,
    pub running: bool,
    pub ended: Option<String>,
}

fn view(a: &icarus_obd::bus::Analyzer, now: f64, running: bool, ended: Option<String>) -> BusView {
    BusView {
        rows: a
            .ids
            .iter()
            .map(|(id, s)| {
                (
                    *id,
                    s.count,
                    s.hz(),
                    s.data.clone(),
                    s.changed_at
                        .iter()
                        .map(|t| if s.count > 1 { now - t } else { f64::INFINITY })
                        .collect(),
                )
            })
            .collect(),
        log: a.log.iter().rev().take(200).rev().cloned().collect(),
        learned: a.learned.clone(),
        settings: a.settings.clone(),
        services: a.services.clone(),
        notes: a.notes.clone(),
        saved: None,
        learning: false,
        frames: a.frames,
        running,
        ended,
    }
}

pub struct ModuleView {
    pub addr: ModuleAddr,
    pub identity: Vec<(&'static str, String)>,
    /// The identification values it did not give: refused (the request and why) or no answer (Advanced shows them).
    pub notes: Vec<String>,
    pub dtcs: uds::CodesRead,
}

pub enum Event {
    Status(String),
    /// A module's answer to a request to every module was left out (it did not add up); the others' were kept.
    Dropped(String),
    Connected(Box<Info>),
    Failed(String),
    Disconnected,
    ScanStarted(String, usize),
    Module(ModuleView),
    ScanDone {
        found: usize,
        unreachable: Vec<String>,
    },
    ReportSaved(String),
    /// Each module's own values, or why they could not be read.
    ModuleData(Result<Box<icarus_obd::vehicledata::VehicleValues>, String>),
    SelfTests(Vec<icarus_obd::mode06::TestResult>),
    /// A health check as kept and compared (ICA-214), or why it could not be read.
    Health(Result<Box<icarus_obd::health::Health>, String>),
    Charging(icarus_obd::charging::Check),
    Bus(Box<BusView>),
    /// Codes cleared (what was said), and the Vehicle page's information read again.
    Cleared(String, Box<Info>),
    /// A clear that was refused or did not run to the end (why): shown as it is, never taken for the adapter asleep.
    ClearFailed(String),
    Settings(Vec<SettingView>),
    /// A setting change (or undo) ended: what was done, or why not.
    SettingDone(Result<String, String>),
    /// A service routine ran (what the module answered), or why not.
    ServiceDone(Result<String, String>),
    TestStarted(String),
    /// A test ended: its name and why.
    TestStopped {
        name: String,
        why: String,
    },
    /// The requests the talking adapter sent onto the bus since the last of these (ICA-80): (seconds since the connection,
    /// request ID, bytes).
    Sent(Vec<(f64, u32, Vec<u8>)>),
    /// The watched value while a test runs.
    TestReading(f64),
    Reading {
        pid: u8,
        value: f64,
        at: f64,
    },
    Recording(Rec),
}

/// How long a recording's values read in turn are watched before one never answered stops being read (ICA-87).
const PRUNE_AFTER_S: f64 = icarus_obd::prune::WATCH_S;

/// Failed sweeps in a row before a recording opens the adapter again (ICA-112), and how often it tries while it is gone.
const GLITCHES: u32 = 3;
const REOPEN: Duration = Duration::from_secs(3);

/// The worker: commands from `rx`, what it found to `tx`. `fence` gives the key modules' fence when a bus watch starts
/// (`icarus_obd::fence::current`; a test gives its own, so no test reads the home folder through it).
pub fn run(
    rx: Receiver<Cmd>,
    tx: Sender<Event>,
    fence: fn() -> icarus_obd::fence::Fence,
    repaint: impl Fn() + Send + 'static,
) {
    run_with(
        rx,
        tx,
        fence,
        repaint,
        &open,
        &icarus_obd::lock::session_allows_changes,
    )
}

/// The adapter `spec` names, opened and set up (`Target::parse`, `connect_target`).
fn open(spec: &str) -> Result<Elm, String> {
    let target = Target::parse(spec)?;
    icarus_obd::elm::connect_target(&target)
        .map(|(e, _)| e)
        .map_err(|e| format!("{spec}: {e}"))
}

/// `run`, with the adapters opened by `open` and the session floor asked through `floor` (a test gives simulated vehicles
/// and a floor of its own).
pub(crate) fn run_with(
    rx: Receiver<Cmd>,
    tx: Sender<Event>,
    fence: fn() -> icarus_obd::fence::Fence,
    repaint: impl Fn() + Send + 'static,
    open: &dyn Fn(&str) -> Result<Elm, String>,
    floor: Floor,
) {
    let send = |e: Event| {
        let _ = tx.send(e);
        repaint();
    };
    let mut elm: Option<Elm> = None;
    // the connection's adapter (named in messages) and its vehicle's VIN (where its logs go); whether it is wired is asked
    // of the open port itself (`Elm::wired`), never of what the adapter's path leads to now
    let mut spec = String::new();
    let mut vin: Option<String> = None;
    // the connection open now, and how many there have been: a write runs only on the one it was confirmed on (ICA-105)
    let mut connection: Option<Connection> = None;
    let mut connections: u64 = 0;
    let mut live: Vec<u8> = Vec::new();
    // the values the vehicle offers (as connected), a drive being recorded, and what it tells the driver (ICA-209)
    let mut offered: Vec<u8> = Vec::new();
    let mut recorder: Option<icarus_obd::drive::Recorder> = None;
    let mut notifier = icarus_obd::notify::Notifier::system(false);
    let mut glitches = 0u32;
    let mut reopened = Instant::now();
    // the adapter dropped while recording and has not come back: when the recording stops, the window is told the
    // connection is gone, so it connects again by itself rather than show a link that is not there
    let mut lost = false;
    let mut test: Option<actuator::Running> = None;
    let mut last_hold = Instant::now();
    let mut last_watch = Instant::now();
    let start = Instant::now();
    let profiles = vehicles::load(vehicles::user_dir().as_deref()).0;
    // a command that arrived while the bus was watched, handled next
    let mut queued: Option<Cmd> = None;
    // the bench make named for the next connection (ICA-222): taken by it, never by a later one
    let mut bench: Option<String> = None;
    // what the modules did with each request, kept in the vehicle's record (ICA-70)
    let mut outcomes: Vec<icarus_obd::record::Outcome> = Vec::new();
    let mut last_kept = Instant::now();
    loop {
        // a module left out of a broadcast's answers is shown, never lost silently (each once a connection)
        for n in elm.as_mut().map(Elm::take_dropped).unwrap_or_default() {
            send(Event::Dropped(n));
        }
        // what went onto the bus, for the comparison with a listening device (ICA-80)
        let sent = elm.as_mut().map(Elm::take_sent).unwrap_or_default();
        if !sent.is_empty() {
            send(Event::Sent(
                sent.into_iter()
                    .map(|(at, id, b)| (at.duration_since(start).as_secs_f64(), id, b))
                    .collect(),
            ));
        }
        let busy = test.is_some() || recorder.is_some() || (!live.is_empty() && elm.is_some());
        // the vehicle's record: written every few seconds while busy, and whenever the worker is about to wait
        if let Some(e) = elm.as_mut() {
            outcomes.extend(e.take_outcomes());
        }
        if !outcomes.is_empty()
            && (!busy || last_kept.elapsed() >= Duration::from_secs(5) || outcomes.len() >= 500)
        {
            if let (Some(v), Some(e)) = (vin.as_deref(), elm.as_ref()) {
                let adapter = icarus_obd::link::Link::adapter(e);
                if let Err(x) =
                    icarus_obd::record::merge(v, &outcomes, &adapter, e.protocol.describe())
                {
                    send(Event::Status(format!(
                        "The vehicle's record was not kept: {x}"
                    )));
                }
            }
            outcomes.clear();
            last_kept = Instant::now();
        }
        let cmd = if let Some(c) = queued.take() {
            Some(c)
        } else if !busy {
            match rx.recv() {
                Ok(c) => Some(c),
                Err(_) => return,
            }
        } else {
            match rx.recv_timeout(Duration::from_millis(if test.is_some() { 50 } else { 20 })) {
                Ok(c) => Some(c),
                Err(RecvTimeoutError::Timeout) => None,
                Err(RecvTimeoutError::Disconnected) => {
                    // the window is gone: hand control back before the thread ends, and close the recording's file
                    end_test(&mut elm, &mut test, "the window closed", &send);
                    stop_recording(&mut recorder, &mut notifier, &send);
                    return;
                }
            }
        };
        match cmd {
            Some(Cmd::Quit) => {
                end_test(&mut elm, &mut test, "the window closed", &send);
                stop_recording(&mut recorder, &mut notifier, &send);
                return;
            }
            Some(Cmd::TestHold) => {
                last_hold = Instant::now();
                continue;
            }
            Some(Cmd::TestStop) => {
                end_test(&mut elm, &mut test, "released", &send);
                continue;
            }
            // anything else ends a running test first
            Some(_) => end_test(&mut elm, &mut test, "another action", &send),
            None => {}
        }
        match cmd {
            Some(Cmd::TestStart { def, value }) => {
                // a read-only session starts no test, before anything goes out (actuator::start asks it too)
                if let Err(why) = floor() {
                    send(Event::TestStopped {
                        name: def.name.clone(),
                        why: format!("not started: {why}"),
                    });
                    continue;
                }
                let Some(e) = elm.as_mut() else { continue };
                live.clear();
                match actuator::start(e, &def, value) {
                    Ok(r) => {
                        last_hold = Instant::now();
                        send(Event::TestStarted(def.name.clone()));
                        test = Some(r);
                    }
                    Err(why) => send(Event::TestStopped {
                        name: def.name.clone(),
                        why: format!("not started: {why}"),
                    }),
                }
            }
            Some(Cmd::BenchMake(m)) => bench = m,
            Some(Cmd::Connect(s)) => {
                spec = s.clone();
                let spec = s;
                elm = None;
                connection = None;
                live.clear();
                let bench = bench.take();
                // a device set to listen only is never talked through, whoever asks (ICA-80)
                if let Err(why) = crate::devices::Roles::load().may_talk(&spec) {
                    send(Event::Failed(why));
                    continue;
                }
                send(Event::Status(format!("Connecting to {spec}...")));
                match connect(open, &spec, &profiles, bench.as_deref()) {
                    Ok((mut e, mut info)) => {
                        match &info.make {
                            Err(why) => send(Event::Status(format!("Bench make not used: {why}"))),
                            Ok(m) if bench.is_some() && !m.chosen => {
                                send(Event::Status(format!("Bench make not used: {}", m.words())))
                            }
                            Ok(_) => {}
                        }
                        offered = info.live.clone();
                        // a recording goes on through a new connection to the same vehicle, never into another's
                        if recorder.as_ref().is_some_and(|r| r.vin != info.vin) {
                            stop_recording(&mut recorder, &mut notifier, &send);
                        }
                        glitches = 0;
                        vin = info.vin.clone();
                        connections += 1;
                        info.connection = Connection {
                            number: connections,
                            vin: vin.clone(),
                        };
                        connection = Some(info.connection.clone());
                        // what this vehicle refused for good before is not asked again (ICA-71)
                        if let Some(v) = vin.as_deref() {
                            if let Ok(p) = icarus_obd::record::path(v) {
                                e.remember(&icarus_obd::record::load_in(&p));
                            }
                        }
                        elm = Some(e);
                        lost = false;
                        send(Event::Connected(Box::new(info)));
                    }
                    Err(m) => send(Event::Failed(m)),
                }
            }
            Some(Cmd::Disconnect) => {
                stop_recording(&mut recorder, &mut notifier, &send);
                bench = None;
                elm = None;
                vin = None;
                connection = None;
                live.clear();
                send(Event::Disconnected);
            }
            Some(Cmd::Scan { profile, sweep }) => {
                let Some(e) = elm.as_mut() else { continue };
                live.clear();
                if sweep {
                    // every request ID: everything is asked again, the record's refusals too
                    e.remember(&icarus_obd::record::Record::default());
                }
                if let Err(m) = scan(e, &profiles, profile.as_deref(), sweep, &send) {
                    send(Event::Failed(m));
                }
            }
            Some(Cmd::Live(pids)) => {
                live = pids;
            }
            Some(Cmd::RecordQuiet(q)) => notifier.quiet = q,
            Some(Cmd::Record(None)) => {
                if std::mem::take(&mut lost) && elm.is_none() {
                    vin = None;
                    live.clear();
                    send(Event::Disconnected);
                }
                stop_recording(&mut recorder, &mut notifier, &send);
            }
            Some(Cmd::Record(Some(how))) => {
                if recorder.is_some() {
                    continue;
                }
                let Some(e) = elm.as_mut() else {
                    send(Event::Recording(Rec::Failed(
                        "connect first: a recording starts once the vehicle answers".into(),
                    )));
                    continue;
                };
                notifier.quiet = how.quiet;
                match start_recording(e, &offered, &profiles, &how, vin.clone()) {
                    Ok((r, notes)) => {
                        for n in notes {
                            send(Event::Status(n));
                        }
                        notifier.started(start.elapsed().as_secs_f64(), &r.name());
                        send(Event::Recording(Rec::Started {
                            path: r.path().map(|p| p.to_path_buf()).unwrap_or_default(),
                            values: r.values(),
                        }));
                        for w in r.watch_lines() {
                            send(Event::Recording(Rec::Said(w)));
                        }
                        glitches = 0;
                        recorder = Some(r);
                    }
                    Err(why) => send(Event::Recording(Rec::Failed(why))),
                }
            }
            Some(Cmd::Report(scan)) => {
                let Some(e) = elm.as_mut() else { continue };
                live.clear();
                send(Event::Status("Reading everything for the report...".into()));
                match report::collect(e, &profiles, scan)
                    .map_err(|x| x.to_string())
                    .and_then(|d| report::save(&d, None).map_err(|x| x.to_string()))
                {
                    Ok(p) => send(Event::ReportSaved(p.display().to_string())),
                    Err(m) => send(Event::Failed(m)),
                }
            }
            Some(Cmd::ModuleData) => {
                let Some(e) = elm.as_mut() else { continue };
                live.clear();
                send(Event::Status("Reading each module's own values...".into()));
                let us = icarus_core::moddata::units::us_customary();
                send(Event::ModuleData(
                    icarus_obd::vehicledata::read(e, &profiles, None, us).map(Box::new),
                ));
            }
            Some(Cmd::Clear { target, on }) => {
                // before anything goes out, the report's reads included
                if let Err(why) = may_write(&on, connection.as_ref(), floor) {
                    send(Event::ClearFailed(format!("not cleared: {why}")));
                    continue;
                }
                // one module: on the make the VIN read now picks (it must be the one confirmed), or the bench make kept
                // on this connection where its VIN reads blank (ICA-222); where its profile names no airbag module a
                // module of unknown kind could be it (ICA-200); the airbag modules by ID or by the profile. Only the
                // VIN is read before a refusal.
                let Some(e) = elm.as_mut() else { continue };
                let make = match clear_make(e, &profiles, &on) {
                    Ok(m) => m,
                    Err(why) => {
                        send(Event::ClearFailed(format!("not cleared: {why}")));
                        continue;
                    }
                };
                if let Some(why) = target.as_ref().and_then(|&(tx, rx, _)| {
                    clear::refused_on(&profiles, make.make.as_deref(), tx.into(), rx.into(), "")
                }) {
                    send(Event::ClearFailed(format!("not cleared: {why}")));
                    continue;
                }
                if make.chosen {
                    icarus_obd::audit::log(&format!("clear codes: {}", make.words()));
                }
                let make = make.make;
                live.clear();
                send(Event::Status("Saving a report before clearing...".into()));
                let saved = report::collect(e, &profiles, target.is_some())
                    .map_err(|x| x.to_string())
                    .and_then(|d| report::save(&d, None).map_err(|x| x.to_string()));
                let saved = match saved {
                    Ok(p) => p,
                    Err(m) => {
                        send(Event::ClearFailed(format!(
                            "not cleared: the report before clearing could not be saved ({m})"
                        )));
                        continue;
                    }
                };
                let result = match target {
                    None => clear::clear_obd(e).map(|ok| format!("Cleared: {}", ok.join(", "))),
                    Some((tx, rx, d)) => {
                        clear::clear_module(e, tx.into(), rx.into(), d, &profiles, make.as_deref())
                            .map(|_| format!("Cleared module {tx:03X}"))
                    }
                };
                let msg = match result {
                    Ok(msg) => msg,
                    // the clear's own checks and request: refused (the Read-Only switch, the battery, the engine) or failed
                    Err(m) => {
                        send(Event::ClearFailed(format!(
                            "{m} (the report from before is {})",
                            saved.display()
                        )));
                        continue;
                    }
                };
                match gather(e, &profiles) {
                    Ok(mut info) => send(Event::Cleared(
                        format!("{msg}. The report from before is {}.", saved.display()),
                        Box::new({
                            info.connection = on;
                            info
                        }),
                    )),
                    Err(m) => send(Event::Failed(format!(
                        "{m} (the report from before is {})",
                        saved.display()
                    ))),
                }
            }
            Some(Cmd::Charging(secs, battery)) => {
                let Some(e) = elm.as_mut() else { continue };
                live.clear();
                send(Event::Status(format!(
                    "Measuring the battery and charging for {secs} s..."
                )));
                match icarus_obd::charging::check(e, secs, battery) {
                    Ok(c) => send(Event::Charging(c)),
                    Err(m) => send(Event::Failed(m)),
                }
            }
            Some(Cmd::SelfTests) => {
                let Some(e) = elm.as_mut() else { continue };
                live.clear();
                send(Event::Status("Reading the on-board test results...".into()));
                match icarus_obd::mode06::read(e) {
                    Ok(r) => send(Event::SelfTests(r)),
                    Err(x) => send(Event::Failed(x.to_string())),
                }
            }
            Some(Cmd::Health) => {
                let Some(e) = elm.as_mut() else { continue };
                live.clear();
                send(Event::Status(
                    "Health check: the module data's misfire counts...".into(),
                ));
                let profile = write_profile(&profiles, vin.as_deref(), e.bench_make().as_deref());
                // only the misfire counts' requests of the vehicle's module-data set, not the whole set
                let us = icarus_core::moddata::units::us_customary();
                let values = icarus_obd::vehicledata::read_values(
                    e,
                    &profiles,
                    &icarus_obd::health::misfire_value,
                    us,
                )
                .ok();
                let mut said = |t: &str| send(Event::Status(t.to_string()));
                send(Event::Health(
                    icarus_obd::health::check(e, profile, values.as_ref(), &mut said)
                        .map(|c| Box::new(icarus_obd::health::keep(c)))
                        .map_err(|x| x.to_string()),
                ));
            }
            Some(Cmd::SettingsRead) => {
                let Some(e) = elm.as_mut() else { continue };
                live.clear();
                send(Event::Status("Reading the settings...".into()));
                let defs = vehicles::settings_for(
                    write_profile(&profiles, vin.as_deref(), e.bench_make().as_deref()),
                    vin.as_deref(),
                );
                send(Event::Settings(read_settings(e, &defs, vin.as_deref())));
                send(Event::Status(format!("{} setting(s) read.", defs.len())));
            }
            Some(Cmd::Service { def, on }) => {
                if let Err(why) = may_write(&on, connection.as_ref(), floor) {
                    send(Event::ServiceDone(Err(format!(
                        "{} did not run: {why}",
                        def.name
                    ))));
                    continue;
                }
                let Some(e) = elm.as_mut() else { continue };
                live.clear();
                let wired = e.wired();
                send(Event::Status(format!("Running {}...", def.name)));
                send(Event::ServiceDone(
                    icarus_obd::service::run(e, &def, wired).map(|status| {
                        if status.is_empty() {
                            format!("{}: done.", def.name)
                        } else {
                            format!("{}: done; the module answered {status}.", def.name)
                        }
                    }),
                ));
            }
            Some(Cmd::SettingChange { def, value, on }) => {
                // refused before the change and before the settings are read again: nothing goes out
                if let Err(why) = may_write(&on, connection.as_ref(), floor) {
                    send(Event::SettingDone(Err(format!(
                        "{} was not changed: {why}",
                        def.name
                    ))));
                    continue;
                }
                let Some(e) = elm.as_mut() else { continue };
                live.clear();
                // the port opened at connect, judged by its own device: a symlink moved since cannot make it wired
                let result = if !e.wired() {
                    Err("changing a setting writes the module's memory: connect through a wired adapter (a USB cable); this connection is not one Linux puts on USB (Bluetooth, Wi-Fi and built-in serial ports are not)".to_string())
                } else {
                    settings::log_for(vin.as_deref()).and_then(|log| match &value {
                        Some(v) => settings::change(e, &def, v, &log),
                        None => settings::undo(e, &def, &log),
                    })
                };
                let said = result.map(|c| match c {
                    None => format!("{}: already so, nothing written.", def.name),
                    Some(c) => format!(
                        "{}: {} -> {} (read back from the module). The value before is kept in the vehicle's Logs folder.",
                        def.name,
                        def.show(&c.before),
                        def.show(&c.after)
                    ),
                });
                send(Event::SettingDone(said));
                let defs = vehicles::settings_for(
                    write_profile(&profiles, vin.as_deref(), e.bench_make().as_deref()),
                    vin.as_deref(),
                );
                send(Event::Settings(read_settings(e, &defs, vin.as_deref())));
            }
            Some(Cmd::BusMark(_)) => {}
            Some(Cmd::BusStart(source, learning)) => {
                live.clear();
                // answers paired with requests by the profiles' answer IDs (VW's 714 answers on 77E)
                let mut a = icarus_obd::bus::Analyzer::watching(fence()).answering(&profiles);
                let t0 = Instant::now();
                let mut last_view = Instant::now();
                let mut pending: Option<Cmd> = None;
                // keep watching? (a stop or any other command ends it; another command is handled afterwards)
                let mut go_on =
                    |a: &mut icarus_obd::bus::Analyzer, pending: &mut Option<Cmd>| -> bool {
                        match rx.try_recv() {
                            Ok(Cmd::BusStop) => return false,
                            Ok(Cmd::TestHold) => {}
                            Ok(Cmd::BusMark(label)) => a.mark(&label, t0.elapsed().as_secs_f64()),
                            Ok(other) => {
                                *pending = Some(other);
                                return false;
                            }
                            Err(_) => {}
                        }
                        if last_view.elapsed() >= Duration::from_millis(250) {
                            last_view = Instant::now();
                            let mut v = view(a, t0.elapsed().as_secs_f64(), true, None);
                            v.learning = learning;
                            send(Event::Bus(Box::new(v)));
                        }
                        true
                    };
                let ended = match &source {
                    Some(ifname) => match icarus_obd::socketcan::open(ifname) {
                        Err(x) => Err(x.to_string()),
                        Ok(sock) => loop {
                            match sock.read(Duration::from_millis(50), t0.elapsed().as_secs_f64()) {
                                Ok(Some(f)) => a.add(&f),
                                Ok(None) => {}
                                Err(x) => break Err(x.to_string()),
                            }
                            if !go_on(&mut a, &mut pending) {
                                break Ok("stopped".to_string());
                            }
                        },
                    },
                    None => match elm.as_mut() {
                        None => Err("connect first, or choose a Linux CAN interface".into()),
                        Some(e) => {
                            let proto = e.protocol;
                            e.monitor(|line| {
                                if let Some(f) = icarus_obd::bus::parse_line(
                                    line,
                                    proto,
                                    t0.elapsed().as_secs_f64(),
                                ) {
                                    a.add(&f);
                                }
                                go_on(&mut a, &mut pending)
                            })
                            .map_err(|x| x.to_string())
                        }
                    },
                };
                let ended = match ended {
                    Ok(how) => how,
                    Err(m) => {
                        // the session saved below says the watch ended early (when it had begun: not for "connect
                        // first" or an interface that would not open)
                        if a.frames > 0 {
                            a.lost(&m);
                        }
                        format!("could not watch: {m}")
                    }
                };
                // what still waits for its module's answer is noted as unanswered, not learned (ICA-221)
                a.finish();
                let mut last = view(&a, t0.elapsed().as_secs_f64(), false, Some(ended));
                last.learning = learning;
                if learning {
                    let vehicle = match vin.as_deref() {
                        Some(v) if v == icarus_obd::sim::VIN => "the simulated vehicle".to_string(),
                        Some(v) => garage::get(v)
                            .map(|d| d.name())
                            .or_else(|| vehicles::for_vin(&profiles, v).map(|p| p.name.clone()))
                            .unwrap_or_else(|| "a vehicle".into()),
                        None => "a vehicle that gave no VIN".into(),
                    };
                    let what = match &source {
                        Some(ifn) => format!("the CAN interface {ifn}"),
                        None => format!("the adapter {spec}"),
                    };
                    let modules = vin
                        .as_deref()
                        .and_then(|v| vehicles::for_vin(&profiles, v))
                        .map(vehicles::candidates)
                        .unwrap_or_default();
                    let text = learning::session_text(&a, &what, &vehicle, &modules);
                    last.saved = Some(
                        garage::logs_dir(vin.as_deref())
                            .and_then(|d| learning::save(&d, &text))
                            .map(|p| p.display().to_string()),
                    );
                }
                send(Event::Bus(Box::new(last)));
                if let Some(c) = pending {
                    // the command that ended the watch is handled next
                    queued = Some(c);
                }
            }
            Some(Cmd::BusStop) => {}
            Some(Cmd::TestHold) | Some(Cmd::TestStop) | Some(Cmd::Quit) => {}
            None if test.is_some() => {
                if last_hold.elapsed() > Duration::from_millis(500) {
                    end_test(&mut elm, &mut test, "the button was let go", &send);
                    continue;
                }
                let (Some(e), Some(r)) = (elm.as_mut(), test.as_mut()) else {
                    continue;
                };
                if let Err(why) = actuator::keep(e, r) {
                    end_test(&mut elm, &mut test, &why, &send);
                    continue;
                }
                if let Some(pid) = r
                    .def
                    .watch
                    .filter(|_| last_watch.elapsed() >= Duration::from_millis(250))
                {
                    last_watch = Instant::now();
                    if let Ok(v) = j1979::read(e, pid) {
                        if let Some(x) = v.iter().find(|x| x.from == "7E8").or(v.first()) {
                            send(Event::TestReading(x.value));
                        }
                    }
                }
            }
            None if recorder.is_some() => {
                let t = start.elapsed().as_secs_f64();
                notifier.tick(t);
                let Some(e) = elm.as_mut() else {
                    // ICA-112: the adapter dropped while recording: opened again every few seconds, the same recording
                    // going on once it answers (a new connection, so no earlier confirmation carries over)
                    if reopened.elapsed() >= REOPEN && !spec.is_empty() {
                        reopened = Instant::now();
                        // a new connection: no bench make carries over (ICA-222)
                        if let Ok((mut e, mut info)) = connect(open, &spec, &profiles, None) {
                            if recorder.as_ref().is_some_and(|r| r.vin != info.vin) {
                                stop_recording(&mut recorder, &mut notifier, &send);
                            }
                            offered = info.live.clone();
                            vin = info.vin.clone();
                            connections += 1;
                            info.connection = Connection {
                                number: connections,
                                vin: vin.clone(),
                            };
                            connection = Some(info.connection.clone());
                            if let Some(v) = vin.as_deref() {
                                if let Ok(p) = icarus_obd::record::path(v) {
                                    e.remember(&icarus_obd::record::load_in(&p));
                                }
                            }
                            glitches = 0;
                            elm = Some(e);
                            lost = false;
                            send(Event::Connected(Box::new(info)));
                        }
                    } else {
                        std::thread::sleep(Duration::from_millis(80));
                    }
                    continue;
                };
                let r = recorder.as_mut().expect("recording");
                match r.sweep(e, &live, t) {
                    Ok(mut s) => {
                        glitches = 0;
                        for (pid, value) in &s.values {
                            send(Event::Reading {
                                pid: *pid,
                                value: *value,
                                at: t,
                            });
                        }
                        for n in std::mem::take(&mut s.notes) {
                            send(Event::Status(n));
                        }
                        if let Some(rate) = s.rate.take() {
                            send(Event::Status(rate.clone()));
                            send(Event::Recording(Rec::Said(rate)));
                        }
                        if let Some(p) = s.new_part.take() {
                            send(Event::Status(format!(
                                "The engine started again: recording goes on in {}.",
                                p.file_name()
                                    .map(|f| f.to_string_lossy().into_owned())
                                    .unwrap_or_default()
                            )));
                            send(Event::Recording(Rec::Started {
                                path: p,
                                values: r.values(),
                            }));
                        }
                        if s.row {
                            notifier.row(t, s.running() == Some(false));
                            send(Event::Recording(Rec::Rows(r.rows)));
                        } else {
                            // nothing answered (the ignition off?): not at full speed
                            std::thread::sleep(Duration::from_millis(150));
                        }
                    }
                    Err(err) => {
                        glitches += 1;
                        if matches!(err, ObdError::Io(_)) || glitches >= GLITCHES {
                            elm = None;
                            connection = None;
                            lost = true;
                            reopened = Instant::now();
                            send(Event::Status(format!(
                                "The adapter stopped answering ({err}): the recording goes on once it is back."
                            )));
                        } else {
                            std::thread::sleep(Duration::from_millis(150));
                        }
                    }
                }
            }
            None => {
                let Some(e) = elm.as_mut() else { continue };
                // up to six values a request on CAN (j1979::read_many)
                match j1979::read_many(e, &live) {
                    Ok(all) => {
                        for (pid, r) in all {
                            if let Some(x) = r.iter().find(|x| x.from == "7E8").or(r.first()) {
                                send(Event::Reading {
                                    pid,
                                    value: x.value,
                                    at: start.elapsed().as_secs_f64(),
                                });
                            }
                        }
                    }
                    Err(ObdError::NoData) => {}
                    Err(err) => {
                        live.clear();
                        send(Event::Failed(err.to_string()));
                    }
                }
            }
        }
    }
}

/// Starts a recording on the open connection: the values the vehicle offers, its module data (the set chosen as `icarus obd
/// data` chooses it; none when no set fits, said), the GPS when asked; and what to say about it.
fn start_recording(
    e: &mut Elm,
    offered: &[u8],
    profiles: &[Profile],
    how: &RecordStart,
    vin: Option<String>,
) -> Result<(icarus_obd::drive::Recorder, Vec<String>), String> {
    use icarus_obd::moddata::{self, Pacer};
    let mut notes = Vec::new();
    // module data's pacer, stopping for nothing but the recording's end (this recording owns the adapter)
    fn never() -> Option<String> {
        None
    }
    let modules = match icarus_obd::vehicledata::choose_for(e, profiles) {
        Ok(c) => {
            notes.push(format!(
                "Recording module data from the set {} too (the watched values every sweep, the rest in turn, its answers kept).",
                c.set.set.name
            ));
            Some(
                icarus_obd::drive::Modules::new(
                    c.set,
                    c.scope,
                    Pacer::new(Instant::now, std::thread::sleep, never).with_gap(moddata::MIN_GAP),
                )
                .watching(&how.watch),
            )
        }
        Err(why) => {
            notes.push(format!("Recording standard values only: {why}"));
            None
        }
    };
    let dir = icarus_obd::drivelog::folders()
        .into_iter()
        .next()
        .ok_or("no home folder")?;
    let plan = icarus_obd::drive::Plan {
        offered: offered.to_vec(),
        adapter: icarus_obd::link::Link::adapter(e),
        modules,
        gps: how.gps.clone(),
        // asked for by hand: a GPS that does not open stops the recording before it starts, and says why
        gps_optional: false,
        dir,
        keep_awake: !cfg!(test),
        // the garage folder's name for the file's vehicle line (a test reads no garage of the home's)
        vehicle: vin
            .as_deref()
            .filter(|_| !cfg!(test))
            .and_then(icarus_obd::garage::folder_label),
        // the tank it runs on, from the vehicle's fuel log as each part begins (ICA-211)
        fuel: Some(icarus_obd::fuel::source(vin.clone())),
        vin,
    };
    let r = icarus_obd::drive::Recorder::start_pruning(plan, PRUNE_AFTER_S)?;
    Ok((r, notes))
}

/// Ends a recording, its file closed; says so (and how many rows), or why it could not be closed.
fn stop_recording(
    recorder: &mut Option<icarus_obd::drive::Recorder>,
    notifier: &mut icarus_obd::notify::Notifier,
    send: &impl Fn(Event),
) {
    let Some(r) = recorder.take() else { return };
    let (rows, name) = (r.rows, r.name());
    notifier.stopped(rows, &name);
    send(Event::Recording(match r.close() {
        Ok(paths) => Rec::Stopped { rows, paths },
        Err(e) => Rec::Failed(format!("the recording's file was not closed cleanly: {e}")),
    }));
}

/// Ends a running test: control handed back, the session left; says so either way.
fn end_test(
    elm: &mut Option<Elm>,
    test: &mut Option<actuator::Running>,
    why: &str,
    send: &impl Fn(Event),
) {
    let Some(mut r) = test.take() else { return };
    let name = r.def.name.clone();
    let res = match elm.as_mut() {
        Some(e) => actuator::stop(e, &mut r, why),
        None => Err("the adapter is gone".into()),
    };
    let why = match res {
        Ok(()) => format!("{why}; control is back with the module"),
        Err(m) => format!("{why}; handing control back failed ({m}): the module takes it back by itself within seconds"),
    };
    send(Event::TestStopped { name, why });
}

/// The adapter `spec` opened (`open`), with the bench make `bench` named for it (ICA-222: kept only when its VIN reads
/// blank; otherwise the Info's make says why not), and the Vehicle page's information.
fn connect(
    open: &dyn Fn(&str) -> Result<Elm, String>,
    spec: &str,
    profiles: &[Profile],
    bench: Option<&str>,
) -> Result<(Elm, Info), String> {
    let mut elm = open(spec)?;
    let refused = bench.and_then(|m| icarus_obd::benchmake::bind(&mut elm, profiles, m).err());
    let mut info = gather(&mut elm, profiles)?;
    if let Some(why) = refused {
        info.make = Err(why);
    }
    // the link keeps the make only when what the window says (gather's own VIN read) uses it: a VIN that read blank for
    // the binding and then something else must not leave a make on the link the window says it is not using
    if !matches!(&info.make, Ok(m) if m.chosen) {
        elm.keep_bench_make(None);
    }
    Ok((elm, info))
}

/// The make a clear confirmed on the connection `on` follows (ICA-222 review), read on the live link now: the VIN read
/// again must be the one the clear was confirmed for, and the make is the one that VIN picks, or the bench make kept on
/// this connection where it reads blank (`benchmake::for_vin`, as every other write asks it). Only the VIN is read.
fn clear_make(
    e: &mut Elm,
    profiles: &[Profile],
    on: &Connection,
) -> Result<icarus_obd::benchmake::WriteMake, String> {
    let now = j1979::vin(e).map_err(|x| x.to_string());
    if now.as_ref().is_ok_and(|v| *v != on.vin) {
        icarus_obd::audit::log(
            "clear refused: the VIN read differently than when it was confirmed",
        );
        return Err(
            "the vehicle's VIN reads differently now than when the clear was confirmed; nothing was sent: ask again"
                .into(),
        );
    }
    let m = icarus_obd::benchmake::for_vin(profiles, now, e.bench_make().as_deref())?;
    Ok(m)
}

/// The profile a vehicle with `vin` is written (and its tests, settings and routines listed) as: its VIN's, or the bench
/// make kept on its connection (`bench`) where its VIN reads blank (ICA-222); none when the two disagree or neither names
/// one.
fn write_profile<'a>(
    profiles: &'a [Profile],
    vin: Option<&str>,
    bench: Option<&str>,
) -> Option<&'a Profile> {
    let m = icarus_obd::benchmake::for_vin(profiles, Ok(vin.map(str::to_string)), bench).ok()?;
    profiles
        .iter()
        .find(|p| Some(p.id.as_str()) == m.make.as_deref())
}

/// Everything the Vehicle page shows, read again (its connection left for the caller to say).
fn gather(elm: &mut Elm, profiles: &[Profile]) -> Result<Info, String> {
    let e = |x: ObdError| x.to_string();
    let vin = j1979::vin(elm).map_err(e)?;
    let vin_info = vin.as_deref().and_then(|v| vin::decode(v).ok());
    let bench = elm.bench_make();
    let make = icarus_obd::benchmake::for_vin(profiles, Ok(vin.clone()), bench.as_deref());
    let profile = write_profile(profiles, vin.as_deref(), bench.as_deref()).cloned();
    let mut monitors = Vec::new();
    if let Ok(a) = elm.request(&[0x01, 0x01]) {
        for x in a
            .iter()
            .filter(|x| x.data.len() >= 6 && x.data[..2] == [0x41, 0x01])
        {
            if let Some(m) = j1979::monitors(&x.data[2..]) {
                monitors.push((format!("{} ({})", x.module(), x.from), m));
            }
        }
    }
    let fuel_system = elm
        .request(&[0x01, 0x03])
        .ok()
        .and_then(|a| {
            a.into_iter()
                .find(|x| x.data.len() >= 3 && x.data[..2] == [0x41, 0x03])
        })
        .map(|x| j1979::fuel_system(x.data[2]));
    let mut live: Vec<u8> = j1979::supported(elm)
        .map_err(e)?
        .into_iter()
        .flat_map(|s| s.2)
        .filter(|p| j1979::pid(*p).is_some())
        .collect();
    live.sort();
    live.dedup();
    let mut codes = Vec::new();
    let mut readings: Vec<icarus_obd::history::Reading> = Vec::new();
    for (kind, label) in [
        (DtcKind::Stored, "stored"),
        (DtcKind::Pending, "pending"),
        (DtcKind::Permanent, "permanent"),
    ] {
        let mut this = Vec::new();
        for (from, module, list) in j1979::dtcs(elm, kind).map_err(e)? {
            for c in &list {
                this.push((format!("{module} ({from})"), c.clone()));
            }
            if !list.is_empty() {
                codes.push((label, format!("{module} ({from})"), list));
            }
        }
        readings.push((label.to_string(), this));
    }
    // the vehicle's code history (its folder; nothing without a VIN)
    let history = icarus_obd::history::record(vin.as_deref(), &readings).map(|()| {
        vin.as_deref()
            .map(icarus_obd::history::load)
            .unwrap_or_default()
    });
    let freeze = j1979::freeze_frame(elm).map_err(e)?;
    let vehicle = vin.as_deref().and_then(garage::get);
    let tests = vehicles::tests_for(profile.as_ref(), vin.as_deref());
    let services = vehicles::services_for(profile.as_ref(), vin.as_deref());
    let settings = vehicles::settings_for(profile.as_ref(), vin.as_deref());
    let info = Info {
        connection: Connection::default(),
        adapter: format!(
            "{}{}",
            elm.id,
            elm.stn
                .as_ref()
                .map(|s| format!(" ({s})"))
                .unwrap_or_default()
        ),
        wired: elm.wired(),
        voltage: elm.voltage,
        protocol: elm.protocol.describe().to_string(),
        vin,
        vin_info,
        profile,
        make,
        monitors,
        fuel_system,
        live,
        codes,
        freeze,
        tests,
        settings,
        services,
        vehicle,
        history,
    };
    Ok(info)
}

/// Every setting's value now, and what undo would put back. Reads only.
fn read_settings(elm: &mut Elm, defs: &[SettingDef], vin: Option<&str>) -> Vec<SettingView> {
    let log = settings::log_for(vin).ok();
    defs.iter()
        .map(|d| SettingView {
            name: d.name.clone(),
            now: match settings::read(elm, d) {
                Ok((v, _)) => d.show(&v),
                Err(x) => format!("could not be read ({x})"),
            },
            undo_to: log
                .as_deref()
                .and_then(|l| settings::last_change(l, d))
                .map(|c| d.show(&c.before)),
        })
        .collect()
}

fn scan(
    elm: &mut Elm,
    profiles: &[Profile],
    wanted: Option<&str>,
    sweep: bool,
    send: &impl Fn(Event),
) -> Result<(), String> {
    let e = |x: ObdError| x.to_string();
    let profile = match wanted {
        Some(w) => vehicles::find(profiles, w)?.clone(),
        None => {
            let vin = j1979::vin(elm).map_err(e)?;
            match vin.as_deref().and_then(|v| vehicles::for_vin(profiles, v)) {
                Some(p) => p.clone(),
                None => vehicles::find(profiles, "generic")?.clone(),
            }
        }
    };
    let list = if sweep {
        uds::sweep_on(profiles, elm.protocol)
    } else {
        vehicles::candidates_on(&profile, elm.protocol)
    };
    send(Event::ScanStarted(profile.name.clone(), list.len()));
    let (mut found, mut unreachable) = (0, Vec::new());
    let mut guard = uds::ScanGuard::new(profiles);
    for (i, m) in list.iter().enumerate() {
        if let Some(why) = m.unreachable(elm.reach()) {
            unreachable.push(format!("{:03X}  {}: {why}", m.tx, m.name));
            continue;
        }
        send(Event::Status(format!(
            "Looking at {:03X} ({} of {})...",
            m.tx,
            i + 1,
            list.len()
        )));
        // the module as found, read at the answer ID it was heard from (not always its request ID + 8); a bus the adapter
        // refused to switch to is listed as not reachable, it and the bus from now on
        match guard.scan(elm, m).map_err(e)? {
            uds::Step::Unreachable(why) => {
                unreachable.push(format!("{:03X}  {}: {why}", m.tx, m.name));
            }
            uds::Step::Absent => {}
            uds::Step::Found(s) => {
                found += 1;
                let uds::Scanned {
                    addr,
                    identity,
                    notes,
                    dtcs,
                } = *s;
                send(Event::Module(ModuleView {
                    addr,
                    identity,
                    notes,
                    dtcs,
                }));
            }
        }
    }
    send(Event::ScanDone { found, unreachable });
    Ok(())
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use std::sync::mpsc;
    use std::time::Duration;

    /// ICA-207: the Modules page gets the engine and transmission at 700/701 (which list every code they can set, none
    /// failed) as clean, with no code rows, and the engine's failed P0171 at 7E0 still as a code.
    #[test]
    fn the_modules_page_shows_a_module_listing_only_codes_it_can_set_as_clean() {
        let mut elm = Elm::connect(Box::new(icarus_obd::sim::SimCar::new())).unwrap();
        let got = std::sync::Mutex::new(Vec::new());
        scan(&mut elm, &vehicles::builtin(), None, false, &|e| {
            if let Event::Module(m) = e {
                got.lock().unwrap().push(m);
            }
        })
        .unwrap();
        let got = got.into_inner().unwrap();
        let at = |tx: u32| {
            let m = got.iter().find(|m| m.addr.tx == tx).unwrap();
            m.dtcs.as_ref().unwrap().clone()
        };
        for (tx, n) in [(0x700, 6), (0x701, 4)] {
            let read = at(tx);
            assert!(read.codes.is_empty(), "{tx:03X}: {:?}", read.codes);
            assert_eq!(
                crate::clean_codes_label(&read, "UDS").as_deref(),
                Some(format!("No trouble codes ({n} it can set, none failed) (UDS)").as_str())
            );
        }
        let engine = at(0x7E0);
        assert_eq!(engine.codes[0].code, "P0171");
        assert_eq!(crate::clean_codes_label(&engine, "UDS"), None);
        let cluster = at(0x7C0);
        assert_eq!(
            crate::clean_codes_label(&cluster, "UDS").as_deref(),
            Some("No trouble codes (UDS)"),
            "a module that lists none"
        );
    }

    /// ICA-211 through the window's worker: a recording on a vehicle with a fuel log names the tank it runs on.
    #[test]
    fn a_recording_through_the_window_names_the_tank() {
        let home = crate::TestHome::new("worker-fuel");
        let vin = "1ABCDEFGH12345678";
        let dir = icarus_obd::garage::ensure(vin).unwrap();
        assert!(dir.starts_with(&home.dir));
        let mut log = icarus_obd::fuel::FuelLog {
            tank_gallons: Some(20.0),
            ..Default::default()
        };
        log.add(icarus_obd::fuel::FillUp {
            date: "2026-01-01".into(),
            grade: 87.0,
            ethanol: 10.0,
            gallons: 20.0,
            ..Default::default()
        })
        .unwrap();
        log.save(&dir).unwrap();
        let mut elm = Elm::connect(Box::new(icarus_obd::sim::SimCar::new())).unwrap();
        let offered: Vec<u8> = j1979::supported(&mut elm)
            .unwrap()
            .into_iter()
            .flat_map(|s| s.2)
            .filter(|p| j1979::pid(*p).is_some())
            .collect();
        let how = RecordStart {
            gps: None,
            quiet: true,
            watch: vec![],
        };
        let (r, _) = start_recording(
            &mut elm,
            &offered,
            &vehicles::builtin(),
            &how,
            Some(vin.into()),
        )
        .unwrap();
        assert!(r.follows_fuel_log());
        let path = r.close().unwrap().remove(0);
        assert!(path.starts_with(&home.dir));
        let text = std::fs::read_to_string(&path).unwrap();
        assert!(
            text.contains("\n# fuel: tank 87 E10; filled 2026-01-01, 20 gal; blend 87 E10"),
            "{text}"
        );
        drop(home);
    }

    /// ICA-208/112 through the window's worker: Record drive writes rows with standard and module columns, goes on when the
    /// adapter dies (opened again by the worker: the same file), and Stop closes it with its row count.
    #[test]
    fn a_recording_through_the_window_goes_on_after_the_adapter_drops() {
        let home = crate::TestHome::new("worker-record");
        let (ctx, crx) = mpsc::channel();
        let (etx, erx) = mpsc::channel();
        let opens = std::sync::Arc::new(std::sync::atomic::AtomicU32::new(0));
        let o = opens.clone();
        let worker = std::thread::spawn(move || {
            let open = move |spec: &str| -> Result<Elm, String> {
                assert_eq!(spec, "sim");
                let n = o.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
                // the first connection dies a little into the recording
                let car = if n == 0 {
                    icarus_obd::sim::SimCar::new().dying_after(400)
                } else {
                    icarus_obd::sim::SimCar::new()
                };
                Elm::connect(Box::new(car)).map_err(|e| e.to_string())
            };
            run_with(
                crx,
                etx,
                icarus_obd::fence::Fence::default,
                || {},
                &open,
                &|| Ok(()),
            )
        });
        ctx.send(Cmd::Connect("sim".into())).unwrap();
        until(&erx, "connection", &|e| matches!(e, Event::Connected(_)));
        ctx.send(Cmd::Record(Some(RecordStart {
            gps: None,
            quiet: true,
            watch: icarus_obd::drive::WATCH_DEFAULT
                .iter()
                .map(|w| w.to_string())
                .collect(),
        })))
        .unwrap();
        let seen = until(&erx, "the start", &|e| {
            matches!(
                e,
                Event::Recording(Rec::Started { .. }) | Event::Recording(Rec::Failed(_))
            )
        });
        let Some(Event::Recording(Rec::Started { path, values })) = seen.last() else {
            panic!("not started");
        };
        let path = path.clone();
        assert!(path.starts_with(&home.dir), "{path:?}");
        assert!(*values > 20, "{values}");
        // ICA-218: what is watched is said as it starts
        until(
            &erx,
            "the watched values said",
            &|e| matches!(e, Event::Recording(Rec::Said(l)) if l.contains("torque converter lock-up (701 22 1F11)")),
        );
        until(&erx, "a new connection", &|e| {
            matches!(e, Event::Connected(_))
        });
        until(
            &erx,
            "rows after it",
            &|e| matches!(e, Event::Recording(Rec::Rows(n)) if *n > 100),
        );
        ctx.send(Cmd::Record(None)).unwrap();
        let seen = until(&erx, "the end", &|e| {
            matches!(e, Event::Recording(Rec::Stopped { .. }))
        });
        let Some(Event::Recording(Rec::Stopped { rows, paths })) = seen.last() else {
            unreachable!()
        };
        assert_eq!(paths, &vec![path.clone()], "one file through the drop");
        let text = std::fs::read_to_string(&path).unwrap();
        let s = icarus_obd::drivelog::series(&text).unwrap();
        assert_eq!(s.t.len(), *rows);
        assert!(s.find(&["engine speed [rpm]"]).is_some());
        assert!(
            s.find(&["knock correction (700 22 1f01)"]).is_some()
                && s.find(&["700 22 1f01 answer [hex]"]).is_some(),
            "module values and their answers: {:?}",
            s.names
        );
        assert!(opens.load(std::sync::atomic::Ordering::SeqCst) >= 2);
        drop(ctx);
        worker.join().unwrap();
        drop(home);
    }

    /// Review of ICA-208: the adapter drops while recording and stays away; Stop then ends the recording and tells the window
    /// the connection is gone (Disconnected, before the rows are said), so it connects again by itself instead of showing
    /// a link with frozen values. Quit (the window closing) ends the worker.
    #[test]
    fn stopping_a_recording_whose_adapter_is_gone_says_disconnected() {
        let home = crate::TestHome::new("worker-lost");
        let (ctx, crx) = mpsc::channel();
        let (etx, erx) = mpsc::channel();
        let opens = std::sync::Arc::new(std::sync::atomic::AtomicU32::new(0));
        let o = opens.clone();
        let worker = std::thread::spawn(move || {
            let open = move |_: &str| -> Result<Elm, String> {
                if o.fetch_add(1, std::sync::atomic::Ordering::SeqCst) > 0 {
                    return Err("no adapter".into());
                }
                Elm::connect(Box::new(icarus_obd::sim::SimCar::new().dying_after(400)))
                    .map_err(|e| e.to_string())
            };
            run_with(
                crx,
                etx,
                icarus_obd::fence::Fence::default,
                || {},
                &open,
                &|| Ok(()),
            )
        });
        ctx.send(Cmd::Connect("sim".into())).unwrap();
        until(&erx, "connection", &|e| matches!(e, Event::Connected(_)));
        ctx.send(Cmd::Record(Some(RecordStart {
            gps: None,
            quiet: true,
            watch: icarus_obd::drive::WATCH_DEFAULT
                .iter()
                .map(|w| w.to_string())
                .collect(),
        })))
        .unwrap();
        until(
            &erx,
            "the drop",
            &|e| matches!(e, Event::Status(s) if s.contains("stopped answering")),
        );
        ctx.send(Cmd::Record(None)).unwrap();
        let seen = until(&erx, "the end", &|e| {
            matches!(e, Event::Recording(Rec::Stopped { .. }))
        });
        let at = |f: &dyn Fn(&Event) -> bool| seen.iter().position(f);
        let gone = at(&|e| matches!(e, Event::Disconnected)).expect("disconnected");
        let stopped = at(&|e| matches!(e, Event::Recording(Rec::Stopped { .. }))).unwrap();
        assert!(gone < stopped, "the link said gone first, then the rows");
        ctx.send(Cmd::Quit).unwrap();
        let end = Instant::now() + Duration::from_secs(5);
        while !worker.is_finished() {
            assert!(Instant::now() < end, "Quit ends the worker");
            std::thread::sleep(Duration::from_millis(20));
        }
        worker.join().unwrap();
        drop(ctx);
        drop(home);
    }

    /// Another vehicle's VIN, made up: the adapter comes back on a different vehicle.
    pub(crate) const OTHER: &str = "5TFICARUSDONOR001";

    /// The adapters this worker test opens: the simulated vehicle (`sim`, engine off) and the same one giving another VIN
    /// (`other`).
    pub(crate) fn two_vehicles(spec: &str) -> Result<Elm, String> {
        let car = match spec {
            "sim" => icarus_obd::sim::SimCar::engine_off(),
            "other" => icarus_obd::sim::SimCar::engine_off().with_vin(OTHER),
            s => return Err(format!("no adapter {s} in this test")),
        };
        Elm::connect(Box::new(car)).map_err(|e| e.to_string())
    }

    /// The events up to the first one `ok` takes, that one last; panics after 30 s.
    fn until(erx: &mpsc::Receiver<Event>, what: &str, ok: &dyn Fn(&Event) -> bool) -> Vec<Event> {
        let end = Instant::now() + Duration::from_secs(30);
        let mut seen = Vec::new();
        while Instant::now() < end {
            if let Ok(e) = erx.recv_timeout(Duration::from_millis(200)) {
                let done = ok(&e);
                seen.push(e);
                if done {
                    return seen;
                }
            }
        }
        panic!("no {what}");
    }

    /// ICA-105: a clear (and a routine and a setting change) confirmed on the simulated vehicle, then the adapter connects
    /// again and the vehicle there gives another VIN. The window's dialogs close on that Connected event; the stale
    /// commands are each refused, with no request on the bus (not even the report's reads before a clear, or the settings
    /// read again after a change). The same change confirmed on the new connection goes.
    #[test]
    fn a_write_confirmed_on_an_earlier_connection_is_refused_with_nothing_sent() {
        use crate::confirm::{Asked, Confirms};
        let home = crate::TestHome::new("worker-ica105");
        let (ctx, crx) = mpsc::channel();
        let (etx, erx) = mpsc::channel();
        let worker = std::thread::spawn(move || {
            run_with(
                crx,
                etx,
                icarus_obd::fence::Fence::default,
                || {},
                &two_vehicles,
                &icarus_obd::lock::session_allows_changes,
            )
        });
        let connected = |erx: &mpsc::Receiver<Event>| -> Connection {
            let seen = until(erx, "connection", &|e| matches!(e, Event::Connected(_)));
            let Some(Event::Connected(info)) = seen.last() else {
                unreachable!()
            };
            let c = info.connection.clone();
            // what the connection read went out: reported once, before anything else is asked
            until(erx, "the connection's requests", &|e| {
                matches!(e, Event::Sent(_))
            });
            c
        };
        ctx.send(Cmd::Connect("sim".into())).unwrap();
        let first = connected(&erx);
        assert_eq!(first.number, 1);
        assert_eq!(first.vin.as_deref(), Some(icarus_obd::sim::VIN));
        // the person says yes on the first connection: the commands as the window builds them, and its dialogs open
        let chime = icarus_obd::sim::settings().remove(0);
        let off = chime.values["off"].clone();
        let stale = || {
            [
                Cmd::Clear {
                    target: None,
                    on: first.clone(),
                },
                Cmd::Service {
                    def: icarus_obd::sim::services().remove(1),
                    on: first.clone(),
                },
                Cmd::SettingChange {
                    def: chime.clone(),
                    value: Some(off.clone()),
                    on: first.clone(),
                },
            ]
        };
        let mut dialogs = Confirms {
            clear: Some(Asked {
                what: (None, "every module".into()),
                on: first.clone(),
            }),
            ..Default::default()
        };
        // the adapter connects again, to another vehicle
        ctx.send(Cmd::Connect("other".into())).unwrap();
        let seen = until(&erx, "the second connection", &|e| {
            matches!(e, Event::Connected(_))
        });
        for e in &seen {
            dialogs.on_event(e);
        }
        assert!(dialogs.clear.is_none(), "the dialog closed on Connected");
        let Some(Event::Connected(info)) = seen.last() else {
            unreachable!()
        };
        let second = info.connection.clone();
        assert_eq!(second.number, 2);
        assert_eq!(second.vin.as_deref(), Some(OTHER));
        until(&erx, "the connection's requests", &|e| {
            matches!(e, Event::Sent(_))
        });
        // the stale yeses arrive: each refused, nothing sent
        for c in stale() {
            ctx.send(c).unwrap();
        }
        ctx.send(Cmd::Disconnect).unwrap();
        let after = until(&erx, "the disconnect", &|e| {
            matches!(e, Event::Disconnected)
        });
        let earlier = |m: &str| m.contains("earlier connection") && m.contains("nothing was sent");
        assert!(after.iter().any(
            |e| matches!(e, Event::ClearFailed(m) if m.starts_with("not cleared") && earlier(m))
        ));
        assert!(after
            .iter()
            .any(|e| matches!(e, Event::ServiceDone(Err(m)) if earlier(m))));
        assert!(after
            .iter()
            .any(|e| matches!(e, Event::SettingDone(Err(m)) if earlier(m))));
        for e in &after {
            assert!(
                !matches!(e, Event::Sent(_) | Event::Settings(_) | Event::Cleared(..)),
                "something went onto the bus for a stale yes"
            );
        }
        // once disconnected, a yes for the connection that was last open is refused too
        ctx.send(Cmd::SettingChange {
            def: chime.clone(),
            value: Some(off.clone()),
            on: second.clone(),
        })
        .unwrap();
        let ended = until(&erx, "the refusal", &|e| matches!(e, Event::SettingDone(_)));
        assert!(
            matches!(ended.last(), Some(Event::SettingDone(Err(m))) if m.contains("has ended")),
            "{:?}",
            ended.len()
        );
        // a change confirmed on the connection open now goes to that vehicle
        ctx.send(Cmd::Connect("other".into())).unwrap();
        let third = connected(&erx);
        assert_eq!(third.number, 3);
        ctx.send(Cmd::SettingChange {
            def: chime,
            value: Some(off),
            on: third,
        })
        .unwrap();
        let done = until(&erx, "the change", &|e| matches!(e, Event::SettingDone(_)));
        assert!(
            matches!(done.last(), Some(Event::SettingDone(Ok(_)))),
            "the change on the connection open now went"
        );
        let sent = until(&erx, "the change's requests", &|e| {
            matches!(e, Event::Sent(_))
        });
        let Some(Event::Sent(v)) = sent.last() else {
            unreachable!()
        };
        assert!(
            v.iter()
                .any(|(_, id, b)| *id == 0x7C0 && b.first() == Some(&0x2E)),
            "the write reached the cluster"
        );
        drop(ctx);
        worker.join().unwrap();
        drop(home);
    }

    /// ICA-200: on a simulated vehicle whose profile (a user file written for the test, picked by the VIN) names no airbag
    /// module, the window's clear of a module the sweep found and the profile does not name (the ABS module at 7B0) is
    /// refused with module data's reason, and nothing is sent (not even the report's reads); a listed airbag ID is refused
    /// too; the engine, which the profile names with a kind, still clears.
    #[test]
    fn a_bench_make_goes_with_its_connection_and_never_reaches_a_friends_car() {
        // ICA-222 review: toyota named for a bench ECM (blank VIN), then the adapter disconnected and a friend's car of a
        // make with no profile connected: it gets no bench make (no Toyota profile, its tests or settings), and a clear of
        // its engine is refused by the caution with nothing sent. A reconnect of the bench ECM starts without it too, and
        // a bench make named for a well-formed VIN is not used (and said).
        const FRIEND: &str = "ZZFICARUSFRND0001";
        let _home = crate::TestHome::new("worker-ica222");
        let (ctx, crx) = mpsc::channel();
        let (etx, erx) = mpsc::channel();
        let worker = std::thread::spawn(move || {
            run_with(
                crx,
                etx,
                icarus_obd::fence::Fence::default,
                || {},
                &|spec: &str| {
                    let vin = match spec {
                        "bench" => "",
                        "friend" => FRIEND,
                        s => return Err(format!("no adapter {s} in this test")),
                    };
                    Elm::connect(Box::new(
                        icarus_obd::sim::SimCar::engine_off().with_vin(vin),
                    ))
                    .map_err(|e| e.to_string())
                },
                &icarus_obd::lock::session_allows_changes,
            )
        });
        let connect = |spec: &str| {
            ctx.send(Cmd::Connect(spec.into())).unwrap();
            let seen = until(&erx, "the connection", &|e| {
                matches!(e, Event::Connected(_))
            });
            let Some(Event::Connected(info)) = seen.into_iter().last() else {
                unreachable!()
            };
            info
        };
        ctx.send(Cmd::BenchMake(Some("toyota".into()))).unwrap();
        let bench = connect("bench");
        let made = bench.make.as_ref().unwrap();
        assert!(made.chosen, "{made:?}");
        assert_eq!(made.words(), "make chosen by me: toyota, VIN blank");
        assert_eq!(
            bench.profile.as_ref().map(|p| p.id.as_str()),
            Some("toyota")
        );
        // the same bench ECM connected again (as an auto-connect or a reconnect does): no make
        let again = connect("bench");
        assert!(!again.make.as_ref().unwrap().chosen);
        assert!(again.profile.is_none());
        // named again, disconnected before connecting: forgotten
        ctx.send(Cmd::BenchMake(Some("toyota".into()))).unwrap();
        ctx.send(Cmd::Disconnect).unwrap();
        until(&erx, "the disconnect", &|e| {
            matches!(e, Event::Disconnected)
        });
        let friend = connect("friend");
        let made = friend.make.as_ref().unwrap();
        assert_eq!((made.make.as_deref(), made.chosen), (None, false));
        assert!(friend.profile.is_none());
        assert!(
            friend.tests.is_empty() && friend.settings.is_empty() && friend.services.is_empty()
        );
        until(&erx, "the connection's requests", &|e| {
            matches!(e, Event::Sent(_))
        });
        ctx.send(Cmd::Clear {
            target: Some((0x7E0, 0x7E8, Dialect::Uds)),
            on: friend.connection.clone(),
        })
        .unwrap();
        // a bench make named for the friend's car itself: not used, and why said
        ctx.send(Cmd::BenchMake(Some("toyota".into()))).unwrap();
        ctx.send(Cmd::Connect("friend".into())).unwrap();
        let seen = until(&erx, "the second connection", &|e| {
            matches!(e, Event::Connected(_))
        });
        let refused = seen.iter().find_map(|e| match e {
            Event::ClearFailed(m) => Some(m.clone()),
            _ => None,
        });
        let refused = refused.expect("the clear was refused");
        assert!(refused.contains("could be the airbag module"), "{refused}");
        assert!(
            !seen
                .iter()
                .any(|e| matches!(e, Event::Cleared(..) | Event::ReportSaved(_))),
            "nothing cleared"
        );
        assert!(seen.iter().any(|e| matches!(e, Event::Status(s) if s.starts_with("Bench make not used") && s.contains("names no profile"))));
        let Some(Event::Connected(info)) = seen.last() else {
            unreachable!()
        };
        assert!(info.make.is_err());
        ctx.send(Cmd::Quit).unwrap();
        worker.join().unwrap();
    }

    /// ICA-222, second review. (1) The window's clear reads the VIN again on the live link: a bench ECM kept toyota on a
    /// blank VIN, then reads a well-formed VIN of a make with no profile; its engine's clear is refused, the VIN read
    /// again the only thing sent. (2) The link keeps a bench make only when the window uses it: blank for the binding,
    /// Toyota's VIN for the Vehicle page, blank again at a test's start; the window said the make was not used, and the
    /// test is refused by the caution (the make is not known), not let through on a make the link kept.
    #[test]
    fn a_bench_make_follows_the_vin_on_the_live_link() {
        const FRIEND: &str = "ZZFICARUSFRND0001";
        let _home = crate::TestHome::new("worker-ica222-live");
        let (ctx, crx) = mpsc::channel();
        let (etx, erx) = mpsc::channel();
        let worker = std::thread::spawn(move || {
            run_with(
                crx,
                etx,
                icarus_obd::fence::Fence::default,
                || {},
                &|spec: &str| {
                    // each VIN read in turn: the binding's, the Vehicle page's, then the write's
                    let vins: &[&str] = match spec {
                        "later-friend" => &["", "", FRIEND],
                        "flapping" => &["", icarus_obd::sim::VIN, ""],
                        s => return Err(format!("no adapter {s} in this test")),
                    };
                    Elm::connect(Box::new(
                        icarus_obd::sim::SimCar::engine_off().with_vins(vins),
                    ))
                    .map_err(|e| e.to_string())
                },
                &icarus_obd::lock::session_allows_changes,
            )
        });
        ctx.send(Cmd::BenchMake(Some("toyota".into()))).unwrap();
        ctx.send(Cmd::Connect("later-friend".into())).unwrap();
        let seen = until(&erx, "the connection", &|e| {
            matches!(e, Event::Connected(_))
        });
        let Some(Event::Connected(info)) = seen.last() else {
            unreachable!()
        };
        assert!(info.make.as_ref().unwrap().chosen);
        until(&erx, "the connection's requests", &|e| {
            matches!(e, Event::Sent(_))
        });
        ctx.send(Cmd::Clear {
            target: Some((0x7E0, 0x7E8, Dialect::Uds)),
            on: info.connection.clone(),
        })
        .unwrap();
        let seen = until(&erx, "the refusal", &|e| matches!(e, Event::ClearFailed(_)));
        let Some(Event::ClearFailed(why)) = seen.last() else {
            unreachable!()
        };
        assert!(why.contains("reads differently now"), "{why}");
        // (2) a VIN that flaps between the binding and the Vehicle page's read
        ctx.send(Cmd::BenchMake(Some("toyota".into()))).unwrap();
        ctx.send(Cmd::Connect("flapping".into())).unwrap();
        let seen = until(&erx, "the second connection", &|e| {
            matches!(e, Event::Connected(_))
        });
        assert!(
            seen.iter()
                .any(|e| matches!(e, Event::Status(s) if s.starts_with("Bench make not used"))),
            "the window says it is not used"
        );
        let Some(Event::Connected(info)) = seen.last() else {
            unreachable!()
        };
        assert!(!info.make.as_ref().unwrap().chosen);
        let def = icarus_obd::sim::tests().remove(0);
        ctx.send(Cmd::TestStart { def, value: 0.0 }).unwrap();
        let seen = until(&erx, "the test's end", &|e| {
            matches!(e, Event::TestStopped { .. } | Event::TestStarted(_))
        });
        let Some(Event::TestStopped { why, .. }) = seen.last() else {
            panic!("the test started on a make the window said it was not using")
        };
        assert!(why.contains("could be the airbag module"), "{why}");
        ctx.send(Cmd::Quit).unwrap();
        worker.join().unwrap();
    }

    #[test]
    fn a_clear_of_a_module_of_unknown_kind_is_refused_where_the_profile_names_no_airbag_module() {
        const KART: &str = "ZZKICARUSKART0001";
        let home = crate::TestHome::new("worker-ica200");
        let vehicles = home.dir.join("config/icarus/vehicles");
        std::fs::create_dir_all(&vehicles).unwrap();
        std::fs::write(
            vehicles.join("kart.toml"),
            "name = \"Kart\"\nwmi = [\"ZZK\"]\n[[module]]\ntx = 0x7E0\nname = \"engine\"\nkind = \"engine\"\n",
        )
        .unwrap();
        let (ctx, crx) = mpsc::channel();
        let (etx, erx) = mpsc::channel();
        let worker = std::thread::spawn(move || {
            run_with(
                crx,
                etx,
                icarus_obd::fence::Fence::default,
                || {},
                &|spec: &str| match spec {
                    "kart" => Elm::connect(Box::new(
                        icarus_obd::sim::SimCar::engine_off().with_vin(KART),
                    ))
                    .map_err(|e| e.to_string()),
                    s => Err(format!("no adapter {s} in this test")),
                },
                &icarus_obd::lock::session_allows_changes,
            )
        });
        ctx.send(Cmd::Connect("kart".into())).unwrap();
        let seen = until(&erx, "the connection", &|e| {
            matches!(e, Event::Connected(_))
        });
        let Some(Event::Connected(info)) = seen.last() else {
            unreachable!()
        };
        assert_eq!(info.profile.as_ref().map(|p| p.id.as_str()), Some("kart"));
        let on = info.connection.clone();
        until(&erx, "the connection's requests", &|e| {
            matches!(e, Event::Sent(_))
        });
        for target in [(0x7B0, 0x7B8, Dialect::Kwp), (0x780, 0x788, Dialect::Uds)] {
            ctx.send(Cmd::Clear {
                target: Some(target),
                on: on.clone(),
            })
            .unwrap();
        }
        ctx.send(Cmd::Disconnect).unwrap();
        let after = until(&erx, "the disconnect", &|e| {
            matches!(e, Event::Disconnected)
        });
        let refusals: Vec<&String> = after
            .iter()
            .filter_map(|e| match e {
                Event::ClearFailed(m) => Some(m),
                _ => None,
            })
            .collect();
        assert_eq!(refusals.len(), 2, "{refusals:?}");
        assert!(
            refusals[0].starts_with("not cleared")
                && refusals[0].contains("what 7B0 is, is not known")
                && refusals[0].contains("could be the airbag module"),
            "{}",
            refusals[0]
        );
        assert!(refusals[1].contains("is an airbag"), "{}", refusals[1]);
        for e in &after {
            // only the VIN read again (ICA-222 review: the make is the one the VIN on the link picks now)
            let only_vin = |sent: &[(f64, u32, Vec<u8>)]| sent.iter().all(|s| s.2 == [0x09, 0x02]);
            assert!(
                !matches!(e, Event::Cleared(..) | Event::ReportSaved(_))
                    && !matches!(e, Event::Sent(s) if !only_vin(s)),
                "something went onto the bus for a refused clear"
            );
        }
        // the engine, named with a kind in the profile: cleared
        ctx.send(Cmd::Connect("kart".into())).unwrap();
        let seen = until(&erx, "the connection", &|e| {
            matches!(e, Event::Connected(_))
        });
        let Some(Event::Connected(info)) = seen.last() else {
            unreachable!()
        };
        ctx.send(Cmd::Clear {
            target: Some((0x7E0, 0x7E8, Dialect::Uds)),
            on: info.connection.clone(),
        })
        .unwrap();
        let done = until(&erx, "the clear", &|e| {
            matches!(e, Event::Cleared(..) | Event::ClearFailed(_))
        });
        assert!(
            matches!(done.last(), Some(Event::Cleared(m, _)) if m.starts_with("Cleared module 7E0")),
            "{:?}",
            done.iter()
                .filter_map(|e| match e {
                    Event::ClearFailed(m) => Some(m.as_str()),
                    _ => None,
                })
                .collect::<Vec<_>>()
        );
        drop(ctx);
        worker.join().unwrap();
        drop(home);
    }

    /// A session floor of the test's own that refuses, as `lock::session_allows_changes` does in a read-only run (the floor
    /// for the whole run would be every test's in this process, so it is not turned on here).
    fn read_only_floor() -> Result<(), String> {
        Err("session read-only: this test's own floor".into())
    }

    /// ICA-192 in the window's worker: the session floor on, a donor VIN on the port and no vehicle switched to Read-Only.
    /// A clear (every module and one), a service routine, a setting change and its undo, each confirmed on the connection
    /// open now, and an actuator test are refused with "session read-only" before anything goes out: not the clear's
    /// report reads, not the settings read again after a change. Reads still go, and only reads went onto the bus.
    #[test]
    fn the_session_floor_stops_the_windows_writes_before_anything_is_sent() {
        let home = crate::TestHome::new("worker-ica192");
        let (ctx, crx) = mpsc::channel();
        let (etx, erx) = mpsc::channel();
        let worker = std::thread::spawn(move || {
            run_with(
                crx,
                etx,
                icarus_obd::fence::Fence::default,
                || {},
                &two_vehicles,
                &read_only_floor,
            )
        });
        ctx.send(Cmd::Connect("other".into())).unwrap();
        let seen = until(&erx, "the connection", &|e| {
            matches!(e, Event::Connected(_))
        });
        let Some(Event::Connected(info)) = seen.last() else {
            unreachable!()
        };
        let on = info.connection.clone();
        assert_eq!(on.vin.as_deref(), Some(OTHER));
        until(&erx, "the connection's requests", &|e| {
            matches!(e, Event::Sent(_))
        });
        let chime = icarus_obd::sim::settings().remove(0);
        let off = chime.values["off"].clone();
        for c in [
            Cmd::Clear {
                target: None,
                on: on.clone(),
            },
            Cmd::Clear {
                target: Some((0x7E0, 0x7E8, Dialect::Uds)),
                on: on.clone(),
            },
            Cmd::Service {
                def: icarus_obd::sim::services().remove(1),
                on: on.clone(),
            },
            Cmd::SettingChange {
                def: chime.clone(),
                value: Some(off),
                on: on.clone(),
            },
            Cmd::SettingChange {
                def: chime,
                value: None,
                on,
            },
            Cmd::TestStart {
                def: icarus_obd::sim::tests().remove(0),
                value: 30.0,
            },
            // reads: they go (this VIN has no settings to read; the on-board test results are asked)
            Cmd::SettingsRead,
            Cmd::SelfTests,
            Cmd::Disconnect,
        ] {
            ctx.send(c).unwrap();
        }
        let after = until(&erx, "the disconnect", &|e| {
            matches!(e, Event::Disconnected)
        });
        let floor = |m: &str| m.contains("session read-only");
        let count = |f: &dyn Fn(&Event) -> bool| after.iter().filter(|e| f(e)).count();
        assert_eq!(
            count(
                &|e| matches!(e, Event::ClearFailed(m) if m.starts_with("not cleared") && floor(m))
            ),
            2,
            "both clears refused"
        );
        assert_eq!(
            count(&|e| matches!(e, Event::ServiceDone(Err(m)) if floor(m))),
            1
        );
        assert_eq!(
            count(&|e| matches!(e, Event::SettingDone(Err(m)) if floor(m))),
            2,
            "the change and the undo refused"
        );
        assert_eq!(
            count(
                &|e| matches!(e, Event::TestStopped { why, .. } if why.starts_with("not started") && floor(why))
            ),
            1
        );
        assert_eq!(
            count(&|e| matches!(
                e,
                Event::Cleared(..)
                    | Event::TestStarted(_)
                    | Event::SettingDone(Ok(_))
                    | Event::ServiceDone(Ok(_))
            )),
            0,
            "nothing was carried out"
        );
        // the settings were read once, for SettingsRead: never again after a refused change
        assert_eq!(count(&|e| matches!(e, Event::Settings(_))), 1);
        assert_eq!(
            count(&|e| matches!(e, Event::SelfTests(r) if !r.is_empty())),
            1,
            "the on-board test results were read"
        );
        let sent: Vec<&Vec<u8>> = after
            .iter()
            .filter_map(|e| match e {
                Event::Sent(v) => Some(v.iter().map(|(_, _, b)| b)),
                _ => None,
            })
            .flatten()
            .collect();
        assert!(!sent.is_empty(), "the reads went out");
        for b in sent {
            assert!(
                !icarus_obd::lock::changes_vehicle(b),
                "{b:02X?} went onto the bus"
            );
        }
        drop(ctx);
        worker.join().unwrap();
        drop(home);
    }
}
