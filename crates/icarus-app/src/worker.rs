//! The vehicle side of the window: one thread owns the adapter (its reads block for up to seconds), takes commands from the
//! window and sends back what it found. The window never waits on the vehicle.

use icarus_obd::elm::{Elm, ObdError};
use icarus_obd::garage::{self, Described};
use icarus_obd::j1979::{self, DtcKind, FreezeFrame, Monitors};
use icarus_obd::port::Target;
use icarus_obd::uds::{self, Dialect, ModuleDtc};
use icarus_obd::vehicles::{self, ModuleAddr, Profile};
use icarus_obd::vin::{self, VinInfo};
use icarus_obd::{actuator, clear, report};
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender};
use std::time::{Duration, Instant};

pub enum Cmd {
    Connect(String),
    Disconnect,
    /// Scan modules: the profile to use (None: from the VIN), and whether to sweep every ID.
    Scan {
        profile: Option<String>,
        sweep: bool,
    },
    /// Poll these live values (empty stops).
    Live(Vec<u8>),
    /// Read everything again and save a report (with every module when true).
    Report(bool),
    /// Start an actuator test (value for percent tests); it runs while TestHold keeps coming (at least every 0.5 s).
    TestStart {
        def: actuator::TestDef,
        value: f64,
    },
    /// Read the on-board test results (service 06).
    SelfTests,
    /// Measure battery and charging voltage for some seconds.
    Charging(u64, icarus_obd::charging::Battery),
    /// Watch the bus: through the connected adapter (None) or a Linux CAN interface (its name). Ends with BusStop or any
    /// other command.
    BusStart(Option<String>),
    BusStop,
    /// Clear codes: every OBD-II module (None) or one module (request ID, answer ID, its dialect). A report is saved first.
    Clear(Option<(u16, u16, Dialect)>),
    TestHold,
    TestStop,
}

pub struct Info {
    pub adapter: String,
    pub voltage: Option<f64>,
    pub protocol: String,
    pub vin: Option<String>,
    pub vin_info: Option<VinInfo>,
    pub profile: Option<Profile>,
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
        frames: a.frames,
        running,
        ended,
    }
}

pub struct ModuleView {
    pub addr: ModuleAddr,
    pub identity: Vec<(&'static str, String)>,
    pub dtcs: Option<(Dialect, Vec<ModuleDtc>)>,
}

pub enum Event {
    Status(String),
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
    SelfTests(Vec<icarus_obd::mode06::TestResult>),
    Charging(icarus_obd::charging::Check),
    Bus(Box<BusView>),
    /// Codes cleared (what was said), and the Vehicle page's information read again.
    Cleared(String, Box<Info>),
    TestStarted(String),
    /// A test ended: its name and why.
    TestStopped {
        name: String,
        why: String,
    },
    /// The watched value while a test runs.
    TestReading(f64),
    Reading {
        pid: u8,
        value: f64,
        at: f64,
    },
}

pub fn run(rx: Receiver<Cmd>, tx: Sender<Event>, repaint: impl Fn() + Send + 'static) {
    let send = |e: Event| {
        let _ = tx.send(e);
        repaint();
    };
    let mut elm: Option<Elm> = None;
    let mut live: Vec<u8> = Vec::new();
    let mut test: Option<actuator::Running> = None;
    let mut last_hold = Instant::now();
    let mut last_watch = Instant::now();
    let start = Instant::now();
    let profiles = vehicles::load(vehicles::user_dir().as_deref()).0;
    // a command that arrived while the bus was watched, handled next
    let mut queued: Option<Cmd> = None;
    loop {
        let busy = test.is_some() || (!live.is_empty() && elm.is_some());
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
                    // the window is gone: hand control back before the thread ends
                    end_test(&mut elm, &mut test, "the window closed", &send);
                    return;
                }
            }
        };
        match cmd {
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
            Some(Cmd::Connect(spec)) => {
                elm = None;
                live.clear();
                send(Event::Status(format!("Connecting to {spec}...")));
                match connect(&spec, &profiles) {
                    Ok((e, info)) => {
                        elm = Some(e);
                        send(Event::Connected(Box::new(info)));
                    }
                    Err(m) => send(Event::Failed(m)),
                }
            }
            Some(Cmd::Disconnect) => {
                elm = None;
                live.clear();
                send(Event::Disconnected);
            }
            Some(Cmd::Scan { profile, sweep }) => {
                let Some(e) = elm.as_mut() else { continue };
                live.clear();
                if let Err(m) = scan(e, &profiles, profile.as_deref(), sweep, &send) {
                    send(Event::Failed(m));
                }
            }
            Some(Cmd::Live(pids)) => live = pids,
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
            Some(Cmd::Clear(target)) => {
                let Some(e) = elm.as_mut() else { continue };
                live.clear();
                send(Event::Status("Saving a report before clearing...".into()));
                let saved = report::collect(e, &profiles, target.is_some())
                    .map_err(|x| x.to_string())
                    .and_then(|d| report::save(&d, None).map_err(|x| x.to_string()));
                let saved = match saved {
                    Ok(p) => p,
                    Err(m) => {
                        send(Event::Failed(format!(
                            "not cleared: the report before clearing could not be saved ({m})"
                        )));
                        continue;
                    }
                };
                let result = match target {
                    None => clear::clear_obd(e).map(|ok| format!("Cleared: {}", ok.join(", "))),
                    Some((tx, rx, d)) => clear::clear_module(e, tx, rx, d)
                        .map(|_| format!("Cleared module {tx:03X}")),
                };
                match result.and_then(|msg| gather(e, &profiles).map(|i| (msg, i))) {
                    Ok((msg, info)) => send(Event::Cleared(
                        format!("{msg}. The report from before is {}.", saved.display()),
                        Box::new(info),
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
            Some(Cmd::BusStart(source)) => {
                live.clear();
                let mut a = icarus_obd::bus::Analyzer::default();
                let t0 = Instant::now();
                let mut last_view = Instant::now();
                let mut pending: Option<Cmd> = None;
                // keep watching? (a stop or any other command ends it; another command is handled afterwards)
                let mut go_on =
                    |a: &icarus_obd::bus::Analyzer, pending: &mut Option<Cmd>| -> bool {
                        match rx.try_recv() {
                            Ok(Cmd::BusStop) => return false,
                            Ok(Cmd::TestHold) => {}
                            Ok(other) => {
                                *pending = Some(other);
                                return false;
                            }
                            Err(_) => {}
                        }
                        if last_view.elapsed() >= Duration::from_millis(250) {
                            last_view = Instant::now();
                            send(Event::Bus(Box::new(view(
                                a,
                                t0.elapsed().as_secs_f64(),
                                true,
                                None,
                            ))));
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
                            if !go_on(&a, &mut pending) {
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
                                go_on(&a, &mut pending)
                            })
                            .map_err(|x| x.to_string())
                        }
                    },
                };
                let ended = match ended {
                    Ok(how) => how,
                    Err(m) => format!("could not watch: {m}"),
                };
                send(Event::Bus(Box::new(view(
                    &a,
                    t0.elapsed().as_secs_f64(),
                    false,
                    Some(ended),
                ))));
                if let Some(c) = pending {
                    // the command that ended the watch is handled next
                    queued = Some(c);
                }
            }
            Some(Cmd::BusStop) => {}
            Some(Cmd::TestHold) | Some(Cmd::TestStop) => {}
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
            None => {
                let Some(e) = elm.as_mut() else { continue };
                for &pid in &live {
                    match j1979::read(e, pid) {
                        Ok(r) => {
                            if let Some(x) = r.iter().find(|x| x.from == "7E8").or(r.first()) {
                                send(Event::Reading {
                                    pid,
                                    value: x.value,
                                    at: start.elapsed().as_secs_f64(),
                                });
                            }
                        }
                        Err(ObdError::NoData) => {}
                        Err(err) => {
                            live.clear();
                            send(Event::Failed(err.to_string()));
                            break;
                        }
                    }
                }
            }
        }
    }
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

fn connect(spec: &str, profiles: &[Profile]) -> Result<(Elm, Info), String> {
    let target = Target::parse(spec)?;
    let port = target.open().map_err(|e| format!("{spec}: {e}"))?;
    let mut elm = Elm::connect(port).map_err(|e| e.to_string())?;
    let info = gather(&mut elm, profiles)?;
    Ok((elm, info))
}

/// Everything the Vehicle page shows, read again.
fn gather(elm: &mut Elm, profiles: &[Profile]) -> Result<Info, String> {
    let e = |x: ObdError| x.to_string();
    let vin = j1979::vin(elm).map_err(e)?;
    let vin_info = vin.as_deref().and_then(|v| vin::decode(v).ok());
    let profile = vin
        .as_deref()
        .and_then(|v| vehicles::for_vin(profiles, v))
        .cloned();
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
    for (kind, label) in [
        (DtcKind::Stored, "stored"),
        (DtcKind::Pending, "pending"),
        (DtcKind::Permanent, "permanent"),
    ] {
        for (from, module, list) in j1979::dtcs(elm, kind).map_err(e)? {
            if !list.is_empty() {
                codes.push((label, format!("{module} ({from})"), list));
            }
        }
    }
    let freeze = j1979::freeze_frame(elm).map_err(e)?;
    let vehicle = vin.as_deref().and_then(garage::get);
    let tests = vehicles::tests_for(profile.as_ref(), vin.as_deref());
    let info = Info {
        adapter: format!(
            "{}{}",
            elm.id,
            elm.stn
                .as_ref()
                .map(|s| format!(" ({s})"))
                .unwrap_or_default()
        ),
        voltage: elm.voltage,
        protocol: elm.protocol.describe().to_string(),
        vin,
        vin_info,
        profile,
        monitors,
        fuel_system,
        live,
        codes,
        freeze,
        tests,
        vehicle,
    };
    Ok(info)
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
        uds::sweep()
    } else {
        vehicles::candidates(&profile)
    };
    send(Event::ScanStarted(profile.name.clone(), list.len()));
    let (mut found, mut unreachable) = (0, Vec::new());
    for (i, m) in list.iter().enumerate() {
        if let Some(why) = m.unreachable() {
            unreachable.push(format!("{:03X}  {}: {why}", m.tx, m.name));
            continue;
        }
        send(Event::Status(format!(
            "Looking at {:03X} ({} of {})...",
            m.tx,
            i + 1,
            list.len()
        )));
        if uds::present(elm, m.tx, m.rx()).map_err(e)? {
            found += 1;
            let identity = uds::module_identity(elm, m.tx, m.rx()).map_err(e)?;
            let dtcs = uds::module_dtcs(elm, m.tx, m.rx()).map_err(e)?;
            send(Event::Module(ModuleView {
                addr: m.clone(),
                identity,
                dtcs,
            }));
        }
    }
    send(Event::ScanDone { found, unreachable });
    Ok(())
}
