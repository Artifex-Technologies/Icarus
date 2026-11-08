//! The vehicle side of the window: one thread owns the adapter (its reads block for up to seconds), takes commands from the
//! window and sends back what it found. The window never waits on the vehicle.

use icarus_obd::elm::{Elm, ObdError};
use icarus_obd::garage::{self, Described};
use icarus_obd::j1979::{self, DtcKind, FreezeFrame, Monitors};
use icarus_obd::port::Target;
use icarus_obd::report;
use icarus_obd::uds::{self, Dialect, ModuleDtc};
use icarus_obd::vehicles::{self, ModuleAddr, Profile};
use icarus_obd::vin::{self, VinInfo};
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
    let start = Instant::now();
    let profiles = vehicles::load(vehicles::user_dir().as_deref()).0;
    loop {
        let cmd = if live.is_empty() || elm.is_none() {
            match rx.recv() {
                Ok(c) => Some(c),
                Err(_) => return,
            }
        } else {
            match rx.recv_timeout(Duration::from_millis(20)) {
                Ok(c) => Some(c),
                Err(RecvTimeoutError::Timeout) => None,
                Err(RecvTimeoutError::Disconnected) => return,
            }
        };
        match cmd {
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

fn connect(spec: &str, profiles: &[Profile]) -> Result<(Elm, Info), String> {
    let target = Target::parse(spec)?;
    let port = target.open().map_err(|e| format!("{spec}: {e}"))?;
    let mut elm = Elm::connect(port).map_err(|e| e.to_string())?;
    let e = |x: ObdError| x.to_string();
    let vin = j1979::vin(&mut elm).map_err(e)?;
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
    let mut live: Vec<u8> = j1979::supported(&mut elm)
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
        for (from, module, list) in j1979::dtcs(&mut elm, kind).map_err(e)? {
            if !list.is_empty() {
                codes.push((label, format!("{module} ({from})"), list));
            }
        }
    }
    let freeze = j1979::freeze_frame(&mut elm).map_err(e)?;
    let vehicle = vin.as_deref().and_then(garage::get);
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
        vehicle,
    };
    Ok((elm, info))
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
