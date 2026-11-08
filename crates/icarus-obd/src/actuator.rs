//! Actuator tests ("active tests", "bidirectional controls"): Icarus asks a module to drive one of its outputs for a short
//! while (open the throttle to 30 %, run the cooling fan, cycle the ABS pump) so the part can be checked. This is the first
//! thing in Icarus that changes what the vehicle does, so it is fenced in, in code, not by convention:
//!
//! - Only output control goes out: UDS `2F` (with "short-term adjustment" or "return control to the ECU"), KWP2000 `30`
//!   (likewise), the diagnostic session it needs (`10 03` and back to `10 01`), tester present, and OBD-II reads for the
//!   conditions. Security access, routines, writing data, clearing codes and programming are refused (`check`).
//! - Airbag (SRS) and occupant-detection modules are never tested, whatever a definition says.
//! - A test has conditions (engine off, engine running, vehicle stationary) checked before it starts and every second while it
//!   runs, needs at least 11.5 V at the port, and stops by itself after its time limit (at most 30 s).
//! - Every stop path hands control back to the module (`stop`). If Icarus dies mid-test, the module's own diagnostic-session
//!   timer (about 5 s without tester present) does the same.
//! - Every start and stop is written to `~/.local/state/icarus/tests.log`.
//!
//! The tests themselves are data: `[[test]]` entries in a vehicle profile. Their IDs are each maker's own; a profile carries
//! only IDs from a source, `checked = false` until seen working on a vehicle.

use crate::elm::{Answer, Elm, ObdError};
use crate::j1979;
use serde::Deserialize;
use std::time::{Duration, Instant};

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Protocol {
    /// ISO 14229 InputOutputControlByIdentifier (2F), a 2-byte identifier.
    Uds,
    /// ISO 14230 InputOutputControlByLocalIdentifier (30), a 1-byte identifier.
    Kwp,
}

impl Protocol {
    pub fn label(self) -> &'static str {
        match self {
            Protocol::Uds => "UDS",
            Protocol::Kwp => "KWP2000",
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Control {
    /// On while held (the `on` bytes).
    #[default]
    Onoff,
    /// A setting from 0 to 100 %, sent as one byte: value x `scale`.
    Percent,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "kebab-case")]
pub enum Condition {
    EngineOff,
    EngineRunning,
    Stationary,
}

impl Condition {
    pub fn describe(self) -> &'static str {
        match self {
            Condition::EngineOff => "engine off (ignition on)",
            Condition::EngineRunning => "engine running",
            Condition::Stationary => "vehicle not moving",
        }
    }
}

fn ten() -> u32 {
    10
}
fn on_default() -> Vec<u8> {
    vec![0x01]
}
fn scale_default() -> f64 {
    2.55
}

#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct TestDef {
    pub name: String,
    /// The module's request ID; answers from `rx` (module + 8 unless given).
    pub module: u16,
    #[serde(default)]
    pub rx: Option<u16>,
    pub protocol: Protocol,
    /// The output's identifier (2 bytes for UDS, 1 for KWP2000).
    pub id: u16,
    #[serde(default)]
    pub control: Control,
    /// The control state bytes for "on".
    #[serde(default = "on_default")]
    pub on: Vec<u8>,
    #[serde(default = "scale_default")]
    pub scale: f64,
    #[serde(default)]
    pub conditions: Vec<Condition>,
    #[serde(default = "ten")]
    pub max_seconds: u32,
    #[serde(default)]
    pub warning: String,
    /// An OBD-II live value (PID) that shows the test working, read while it runs (e.g. 0x11, throttle position).
    #[serde(default)]
    pub watch: Option<u8>,
    #[serde(default)]
    pub checked: bool,
}

impl TestDef {
    pub fn rx(&self) -> u16 {
        self.rx.unwrap_or(self.module + 8)
    }

    /// Why this definition is refused, whatever it says.
    pub fn refused(&self) -> Option<String> {
        let words = self.name.to_lowercase();
        if SRS_IDS.contains(&self.module)
            || [
                "airbag",
                "srs",
                "restraint",
                "occupant",
                "pretension",
                "squib",
            ]
            .iter()
            .any(|w| words.contains(w))
        {
            return Some("airbag and occupant-detection modules are never tested".into());
        }
        if self.protocol == Protocol::Kwp && self.id > 0xFF {
            return Some("a KWP2000 output identifier is one byte".into());
        }
        if self.module > 0x7FF || self.rx() > 0x7FF {
            return Some("not an 11-bit CAN ID".into());
        }
        if self.on.is_empty() || self.on.len() > 4 {
            return Some("the on state is 1 to 4 bytes".into());
        }
        if self.max_seconds == 0 || self.max_seconds > MAX_SECONDS {
            return Some(format!("the time limit is 1 to {MAX_SECONDS} seconds"));
        }
        if self.conditions.contains(&Condition::EngineOff)
            && self.conditions.contains(&Condition::EngineRunning)
        {
            return Some("engine off and engine running cannot both be required".into());
        }
        None
    }

    fn state(&self, value: f64) -> Vec<u8> {
        match self.control {
            Control::Onoff => self.on.clone(),
            Control::Percent => vec![(value.clamp(0.0, 100.0) * self.scale)
                .round()
                .clamp(0.0, 255.0) as u8],
        }
    }

    fn control_request(&self, value: f64) -> Vec<u8> {
        let mut r = match self.protocol {
            Protocol::Uds => vec![0x2F, (self.id >> 8) as u8, self.id as u8, 0x03],
            Protocol::Kwp => vec![0x30, self.id as u8, 0x07],
        };
        r.extend(self.state(value));
        r
    }

    fn return_request(&self) -> Vec<u8> {
        match self.protocol {
            Protocol::Uds => vec![0x2F, (self.id >> 8) as u8, self.id as u8, 0x00],
            Protocol::Kwp => vec![0x30, self.id as u8, 0x00],
        }
    }
}

pub const MAX_SECONDS: u32 = 30;
/// Airbag and occupant-detection request IDs in the built-in profiles (Toyota/Subaru/Mazda 780 and 781, Ford RCM 737, VW 715,
/// Hyundai/Kia 7D2).
const SRS_IDS: &[u16] = &[0x780, 0x781, 0x737, 0x715, 0x7D2];
const MIN_VOLTS: f64 = 11.5;

/// The only requests the actuator path sends.
pub fn check(data: &[u8]) -> Result<(), String> {
    let ok = match data {
        [0x10, 0x01 | 0x03] => true,
        [0x3E, 0x00 | 0x80] => true,
        [0x2F, _, _, 0x00] | [0x2F, _, _, 0x03, ..] => data.len() <= 8,
        [0x30, _, 0x00] | [0x30, _, 0x07, ..] => data.len() <= 7,
        [0x01, _] => true,
        _ => false,
    };
    if ok {
        Ok(())
    } else {
        Err(format!(
            "refused: {:02X?} is not an output-control request Icarus sends",
            data
        ))
    }
}

fn send(elm: &mut Elm, t: &TestDef, data: &[u8]) -> Result<Answer, String> {
    check(data)?;
    for _ in 0..4 {
        let a = elm
            .request_to(t.module, t.rx(), data)
            .map_err(|e| e.to_string())?;
        let Some(a) = a.into_iter().last() else {
            return Err("no answer".into());
        };
        match a.refusal() {
            Some((_, 0x78)) | Some((_, 0x21)) => std::thread::sleep(Duration::from_millis(300)),
            Some((_, code)) => {
                return Err(format!("the module refused: {}", crate::uds::nrc(code)))
            }
            None => return Ok(a),
        }
    }
    Err("the module stayed busy".into())
}

/// Whether the conditions hold now: engine speed and road speed from the standard OBD-II values.
pub fn conditions_hold(elm: &mut Elm, conds: &[Condition]) -> Result<(), String> {
    let engine = |r: Vec<j1979::Reading>| {
        r.iter()
            .find(|x| x.from == "7E8")
            .or(r.first())
            .map(|x| x.value)
    };
    let need_rpm = conds
        .iter()
        .any(|c| matches!(c, Condition::EngineOff | Condition::EngineRunning));
    let rpm = if need_rpm {
        match j1979::read(elm, 0x0C) {
            Ok(r) => engine(r),
            Err(ObdError::NoData) => None,
            Err(e) => return Err(e.to_string()),
        }
    } else {
        None
    };
    for c in conds {
        match c {
            Condition::EngineOff => match rpm {
                Some(v) if v < 50.0 => {}
                Some(v) => {
                    return Err(format!(
                        "the engine must be off (engine speed is {v:.0} rpm)"
                    ))
                }
                None => return Err("could not read engine speed to check the engine is off".into()),
            },
            Condition::EngineRunning => match rpm {
                Some(v) if v >= 400.0 => {}
                _ => return Err("the engine must be running".into()),
            },
            Condition::Stationary => match j1979::read(elm, 0x0D).map(engine) {
                Ok(Some(v)) if v < 1.0 => {}
                Ok(Some(v)) => {
                    return Err(format!("the vehicle must be stopped (speed {v:.0} km/h)"))
                }
                _ => return Err("could not read the vehicle speed to check it is stopped".into()),
            },
        }
    }
    Ok(())
}

fn log(line: &str) {
    if cfg!(test) {
        return; // tests never write into the person's own log
    }
    let dir = std::env::var_os("XDG_STATE_HOME")
        .map(std::path::PathBuf::from)
        .or_else(|| {
            std::env::var_os("HOME").map(|h| std::path::PathBuf::from(h).join(".local/state"))
        })
        .map(|d| d.join("icarus"));
    if let Some(d) = dir {
        let _ = std::fs::create_dir_all(&d);
        use std::io::Write;
        if let Ok(mut f) = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(d.join("tests.log"))
        {
            let _ = writeln!(f, "{} {line}", crate::report::now());
        }
    }
}

static INTERRUPTED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

extern "C" fn on_interrupt(_: libc::c_int) {
    INTERRUPTED.store(true, std::sync::atomic::Ordering::SeqCst);
}

/// From now on Ctrl+C (SIGINT) only marks the process interrupted, so a running test can hand control back before exiting.
pub fn catch_interrupt() {
    unsafe {
        libc::signal(
            libc::SIGINT,
            on_interrupt as *const () as libc::sighandler_t,
        );
    }
}

pub fn interrupted() -> bool {
    INTERRUPTED.load(std::sync::atomic::Ordering::SeqCst)
}

/// A test that is running: it must be kept alive with `keep` and ended with `stop`.
pub struct Running {
    pub def: TestDef,
    pub value: f64,
    started: Instant,
    last_present: Instant,
    last_check: Instant,
    session: bool,
}

impl Running {
    pub fn elapsed(&self) -> Duration {
        self.started.elapsed()
    }

    pub fn left(&self) -> Duration {
        Duration::from_secs(self.def.max_seconds as u64).saturating_sub(self.started.elapsed())
    }
}

/// Starts a test: checks the definition, the battery and the conditions, opens the session the protocol needs, and sends the
/// control. On any failure after the session opened, control is handed back before the error returns.
pub fn start(elm: &mut Elm, def: &TestDef, value: f64) -> Result<Running, String> {
    if let Some(why) = def.refused() {
        return Err(why);
    }
    if let Some(v) = elm
        .command("ATRV", Duration::from_secs(2))
        .ok()
        .and_then(|r| {
            r.first()
                .and_then(|v| v.trim_end_matches('V').parse::<f64>().ok())
        })
    {
        if v < MIN_VOLTS {
            return Err(format!(
                "the battery is at {v:.1} V; tests need at least {MIN_VOLTS} V (connect a charger)"
            ));
        }
    }
    conditions_hold(elm, &def.conditions)?;
    let session = def.protocol == Protocol::Uds;
    if session {
        send(elm, def, &[0x10, 0x03])?;
    }
    let mut run = Running {
        def: def.clone(),
        value,
        started: Instant::now(),
        last_present: Instant::now(),
        last_check: Instant::now(),
        session,
    };
    if let Err(e) = send(elm, def, &def.control_request(value)) {
        let _ = stop(elm, &mut run, "the start failed");
        return Err(e);
    }
    log(&format!(
        "start {:03X} \"{}\" value {value}",
        def.module, def.name
    ));
    Ok(run)
}

/// Keeps a running test alive (tester present every 2 s, conditions every second). An error means the test must stop
/// (call `stop`): the time is up, or a condition no longer holds.
pub fn keep(elm: &mut Elm, run: &mut Running) -> Result<(), String> {
    if run.left().is_zero() {
        return Err(format!("time limit ({} s) reached", run.def.max_seconds));
    }
    if run.last_check.elapsed() >= Duration::from_secs(1) {
        run.last_check = Instant::now();
        conditions_hold(elm, &run.def.conditions)?;
    }
    if run.session && run.last_present.elapsed() >= Duration::from_secs(2) {
        run.last_present = Instant::now();
        send(elm, &run.def, &[0x3E, 0x00])?;
    }
    Ok(())
}

/// Hands control back to the module and leaves the diagnostic session. Tries every step even if one fails.
pub fn stop(elm: &mut Elm, run: &mut Running, why: &str) -> Result<(), String> {
    let r1 = send(elm, &run.def, &run.def.return_request());
    let r2 = if run.session {
        send(elm, &run.def, &[0x10, 0x01]).map(|_| ())
    } else {
        Ok(())
    };
    run.session = false;
    log(&format!(
        "stop {:03X} \"{}\" after {:.1} s: {why}{}",
        run.def.module,
        run.def.name,
        run.elapsed().as_secs_f64(),
        r1.as_ref()
            .err()
            .map(|e| format!(" (return of control failed: {e})"))
            .unwrap_or_default()
    ));
    r1.map(|_| ())?;
    r2
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::SimCar;

    #[test]
    fn only_output_control_goes_out() {
        for ok in [
            &[0x10, 0x03][..],
            &[0x10, 0x01],
            &[0x3E, 0x00],
            &[0x2F, 0xF0, 0x02, 0x03, 0x4C],
            &[0x2F, 0xF0, 0x02, 0x00],
            &[0x30, 0x10, 0x07, 0x01],
            &[0x30, 0x10, 0x00],
        ] {
            assert!(check(ok).is_ok(), "{ok:02X?}");
        }
        for bad in [
            &[0x27, 0x01][..],
            &[0x31, 0x01, 0xFF, 0x00],
            &[0x2E, 0xF1, 0x90, 0x00],
            &[0x14, 0xFF, 0xFF, 0xFF],
            &[0x10, 0x02],
            &[0x2F, 0xF0, 0x02, 0x02],
            &[0x34, 0x00],
            &[0x11, 0x01],
            &[0x04],
        ] {
            assert!(check(bad).is_err(), "{bad:02X?}");
        }
    }

    #[test]
    fn airbag_modules_are_never_tested() {
        let mut t = crate::sim::tests().remove(0);
        assert!(t.refused().is_none());
        t.module = 0x780;
        assert!(t.refused().unwrap().contains("airbag"));
        let mut t = crate::sim::tests().remove(0);
        t.name = "Driver airbag squib".into();
        assert!(t.refused().is_some());
        let mut t = crate::sim::tests().remove(0);
        t.max_seconds = 120;
        assert!(t.refused().is_some());
    }

    #[test]
    fn a_throttle_test_on_the_simulated_vehicle() {
        let throttle = crate::sim::tests()
            .into_iter()
            .find(|t| t.name.starts_with("Throttle"))
            .unwrap();
        // engine running: refused before anything is sent
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        let e = start(&mut elm, &throttle, 30.0).err().unwrap();
        assert!(e.contains("engine must be off"), "{e}");
        // ignition on, engine off: the throttle follows the command, and comes back when control returns
        let mut elm = Elm::connect(Box::new(SimCar::engine_off())).unwrap();
        let tp = |elm: &mut Elm| j1979::read(elm, 0x11).unwrap()[0].value;
        let rest = tp(&mut elm);
        let mut run = start(&mut elm, &throttle, 30.0).unwrap();
        assert!(
            (tp(&mut elm) - 30.0).abs() < 1.0,
            "the throttle opened to 30 %"
        );
        keep(&mut elm, &mut run).unwrap();
        stop(&mut elm, &mut run, "test").unwrap();
        assert!(
            (tp(&mut elm) - rest).abs() < 0.5,
            "control returned to the engine computer"
        );
    }

    #[test]
    fn a_kwp_test_and_a_module_that_needs_its_session() {
        let pump = crate::sim::tests()
            .into_iter()
            .find(|t| t.protocol == Protocol::Kwp)
            .unwrap();
        let mut elm = Elm::connect(Box::new(SimCar::engine_off())).unwrap();
        let mut run = start(&mut elm, &pump, 0.0).unwrap();
        stop(&mut elm, &mut run, "test").unwrap();
        // a UDS output asked for without the extended session is refused by the module, and Icarus says why
        let fan = crate::sim::tests()
            .into_iter()
            .find(|t| t.name.starts_with("Cooling fan"))
            .unwrap();
        let e = send(&mut elm, &fan, &fan.control_request(0.0))
            .err()
            .unwrap();
        assert!(e.contains("conditions not correct"), "{e}");
    }
}
