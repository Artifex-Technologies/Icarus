//! Actuator tests ("active tests", "bidirectional controls"): Icarus asks a module to drive one of its outputs for a short
//! while (open the throttle to 30 %, run the cooling fan, cycle the ABS pump) so the part can be checked. This is the first
//! thing in Icarus that changes what the vehicle does, so it is fenced in, in code, not by convention:
//!
//! - Only output control goes out: UDS `2F` (with "short-term adjustment" or "return control to the ECU"), KWP2000 `30`
//!   (likewise), the diagnostic session it needs (`10 03` and back to `10 01`), tester present, and OBD-II reads for the
//!   conditions. Security access, routines, writing data, clearing codes and programming are refused (`check`).
//! - Airbag (SRS) and occupant-detection modules are never tested, whatever a definition says, nor immobilizer, smart-key
//!   and certification modules (`fence`, by address).
//! - A test has conditions (engine off, engine running, vehicle stationary) checked before it starts and every second while it
//!   runs, needs at least 11.5 V measured at the port (an adapter that cannot measure it is refused), and stops by itself
//!   after its time limit (at most 30 s).
//! - Every stop path hands control back to the module (`stop`). If Icarus dies mid-test, the module's own diagnostic-session
//!   timer (about 5 s without tester present) does the same.
//! - Every start and stop is written to `~/.local/state/icarus/actions.log` (see `audit`).
//!
//! The tests themselves are data: `[[test]]` entries in a vehicle profile. Their IDs are each maker's own; a profile carries
//! only IDs from a source, `checked = false` until seen working on a vehicle.

use crate::elm::{Answer, ObdError};
use crate::j1979;
use crate::link::Link;
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
        if let Some(why) = not_one_module(self.module) {
            return Some(why);
        }
        if is_airbag(self.module, &self.name) || is_airbag(self.rx(), "") {
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

/// Why an ID is not one module's request ID: 7DF is the broadcast every module hears (an action sent there reaches the
/// airbag module too), and module request IDs are 700-7F7 on 11-bit CAN.
pub fn not_one_module(id: u16) -> Option<String> {
    if id == 0x7DF {
        Some("refused: 7DF is the broadcast every module hears, not one module".into())
    } else if !(0x700..=0x7F7).contains(&id) {
        Some(format!(
            "refused: {id:03X} is not a module request ID (700-7F7)"
        ))
    } else {
        None
    }
}

/// Airbag and occupant-detection modules, by ID (a request ID in the built-in profiles, or its answer ID) or by name: Icarus
/// never tests them, clears their codes or writes to them ("the airbag modules never"). One check for every path that acts.
///
/// By name: the words anywhere ("airbags", "restraints", "occupant classification", "pretensioner", "squib", "SRS"), and
/// the makers' acronyms as whole words only, so "ODS" matches but "methods" does not: ORC (occupant restraint controller),
/// ACU (airbag control unit), SDM (sensing and diagnostic module), SIR, OCS/ODS/OCM (occupant classification and
/// detection), RCM (restraints control module), ACSM and MRS (BMW), and "air bag" written as two words.
pub fn is_airbag(id: u16, name: &str) -> bool {
    let lower = name.to_lowercase();
    let words: Vec<&str> = lower
        .split(|c: char| !c.is_ascii_alphanumeric())
        .filter(|w| !w.is_empty())
        .collect();
    SRS_IDS.iter().any(|&s| id == s || id == s + 8)
        || AIRBAG_WORDS.iter().any(|w| lower.contains(w))
        || words.iter().any(|w| AIRBAG_ACRONYMS.contains(w))
        || words
            .windows(2)
            .any(|p| p[0] == "air" && p[1].starts_with("bag"))
}

/// Airbag and occupant-module words, matched anywhere in a name.
const AIRBAG_WORDS: &[&str] = &[
    "airbag",
    "srs",
    "restraint",
    "occupant",
    "pretension",
    "squib",
];
/// Airbag and occupant-module acronyms, matched as whole words.
const AIRBAG_ACRONYMS: &[&str] = &[
    "orc", "acu", "sdm", "sir", "ocs", "ods", "ocm", "rcm", "acsm", "mrs",
];
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

fn send(elm: &mut dyn Link, t: &TestDef, data: &[u8]) -> Result<Answer, String> {
    check(data)?;
    for _ in 0..4 {
        let a = elm
            .request_to(t.module.into(), t.rx().into(), data)
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

/// The battery check every write path makes (actuator tests, clearing codes, settings, service routines): the voltage at
/// the OBD port measured now, and at least `min`. A link that cannot measure it (a SocketCAN interface, the Mongoose), or
/// cannot this time (an ELM327 whose `ATRV` does not answer with a number), is refused, not let through, and a reading
/// from when the link connected is never used instead: a write cut short by a sagging battery is how a module is left
/// half-written. `what` names the write in the reason ("changing a setting"). Returns the voltage measured.
pub fn battery_holds(link: &mut dyn Link, min: f64, what: &str) -> Result<f64, String> {
    match link.voltage() {
        Some(v) if v.is_finite() && v >= min => Ok(v),
        Some(v) if v.is_finite() => Err(format!(
            "the battery is at {v:.1} V; {what} needs at least {min} V (connect a charger)"
        )),
        _ => Err(format!(
            "the adapter cannot measure the battery, so {what} is refused; use an adapter that reads the OBD port's \
             voltage"
        )),
    }
}

/// Whether the conditions hold now: engine speed and road speed from the standard OBD-II values.
pub fn conditions_hold(elm: &mut dyn Link, conds: &[Condition]) -> Result<(), String> {
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
    crate::audit::log(&format!("test {line}"));
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

/// The same for SIGTERM (what `systemctl stop` and a shutdown send): a recording closes its file before the program ends.
pub fn catch_terminate() {
    unsafe {
        libc::signal(
            libc::SIGTERM,
            on_interrupt as *const () as libc::sighandler_t,
        );
    }
}

/// The same for SIGHUP (the terminal a recording runs in closed).
pub fn catch_hangup() {
    unsafe {
        libc::signal(
            libc::SIGHUP,
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
    /// The computer stays awake while the test runs (dropped with the test).
    _awake: Option<crate::power_guard::Inhibit>,
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
pub fn start(elm: &mut dyn Link, def: &TestDef, value: f64) -> Result<Running, String> {
    if let Some(why) = def.refused() {
        return Err(why);
    }
    if let Some(why) = crate::fence::current().refuses(def.module.into(), def.rx().into()) {
        log(&format!(
            "refused {:03X} \"{}\": a key module",
            def.module, def.name
        ));
        return Err(why);
    }
    crate::lock::writes_allowed(elm)?;
    battery_holds(elm, MIN_VOLTS, "an actuator test")?;
    conditions_hold(elm, &def.conditions)?;
    let session = def.protocol == Protocol::Uds;
    if session {
        send(elm, def, &[0x10, 0x03])?;
    }
    let awake = crate::power_guard::Inhibit::take(&format!("Actuator test: {}", def.name)).ok();
    let mut run = Running {
        def: def.clone(),
        value,
        started: Instant::now(),
        last_present: Instant::now(),
        last_check: Instant::now(),
        session,
        _awake: awake,
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
pub fn keep(elm: &mut dyn Link, run: &mut Running) -> Result<(), String> {
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
pub fn stop(elm: &mut dyn Link, run: &mut Running, why: &str) -> Result<(), String> {
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
    use crate::elm::Elm;
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
    fn airbag_names_by_their_acronyms_as_whole_words() {
        for name in [
            "ORC",
            "module ACU (7A2)",
            "SDM",
            "SIR",
            "OCS",
            "ods",
            "RCM (airbags)",
            "ACSM",
            "air bag control",
            "Air-Bags",
            "SRS",
            "occupant classification",
        ] {
            assert!(is_airbag(0x7A2, name), "{name}");
        }
        for name in [
            "methods",
            "accumulator",
            "body (BCM)",
            "ACM (audio)",
            "orchestra",
            "sirius radio",
            "airflow",
            "",
        ] {
            assert!(!is_airbag(0x7A2, name), "{name}");
        }
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

    /// A test over a link that cannot measure the battery (a SocketCAN interface, or a reading that is not a
    /// number) is refused before anything is sent; measured above 11.5 V it runs as before.
    #[test]
    fn a_test_starts_only_with_the_battery_measured() {
        use crate::canlink::{CanLink, SimBus};
        crate::lock::switch_for_test(&[]);
        let throttle = crate::sim::tests()
            .into_iter()
            .find(|t| t.name.starts_with("Throttle"))
            .unwrap();
        let on_bus = |volts: Option<f64>| {
            let mut bus = SimBus::new(SimCar::engine_off());
            bus.voltage = volts;
            CanLink::connect(bus, "vcan0").unwrap()
        };
        for volts in [None, Some(f64::NAN)] {
            let mut l = on_bus(volts);
            let sent = l.frames().sent.len();
            let e = start(&mut l, &throttle, 30.0).err().unwrap();
            assert!(
                e.contains("cannot measure the battery") && e.contains("an actuator test"),
                "{e}"
            );
            assert_eq!(l.frames().sent.len(), sent, "nothing was sent");
        }
        let e = start(&mut on_bus(Some(11.0)), &throttle, 30.0)
            .err()
            .unwrap();
        assert!(e.contains("11.0 V") && e.contains("at least 11.5 V"), "{e}");
        let mut l = on_bus(Some(12.6));
        let mut run = start(&mut l, &throttle, 30.0).unwrap();
        assert!(
            (j1979::read(&mut l, 0x11).unwrap()[0].value - 30.0).abs() < 1.0,
            "the throttle opened to 30 %"
        );
        stop(&mut l, &mut run, "test").unwrap();
    }

    /// An ELM327 that read the battery when it connected and cannot now: its `ATRV` after that answers `answer` (`?`, or
    /// a line left from a bus watch) instead of a number. The simulated vehicle, engine off, is behind it.
    struct Unmeasured {
        car: SimCar,
        answer: &'static str,
        atrv: u32,
        said: std::collections::VecDeque<u8>,
    }

    impl crate::port::Port for Unmeasured {
        fn send(&mut self, data: &[u8]) -> std::io::Result<()> {
            if data == b"ATRV\r" {
                self.atrv += 1;
                if self.atrv > 1 {
                    self.said.extend(format!("{}\r\r>", self.answer).bytes());
                    return Ok(());
                }
            }
            self.car.send(data)
        }
        fn recv(&mut self, buf: &mut [u8], timeout: Duration) -> std::io::Result<usize> {
            if self.said.is_empty() {
                return self.car.recv(buf, timeout);
            }
            let n = buf.len().min(self.said.len());
            for (b, s) in buf.iter_mut().zip(self.said.drain(..n)) {
                *b = s;
            }
            Ok(n)
        }
    }

    /// The battery is judged by a reading taken at the moment of the write, never by the one from when the adapter
    /// connected: an adapter that read 12.6 V then and gives no number now has every write path refused (an actuator
    /// test, clearing codes, a setting, a service routine), and nothing that changes the vehicle goes out.
    #[test]
    fn a_write_needs_the_battery_measured_now_not_when_the_adapter_connected() {
        crate::lock::switch_for_test(&[]);
        let throttle = crate::sim::tests()
            .into_iter()
            .find(|t| t.name.starts_with("Throttle"))
            .unwrap();
        let setting = crate::sim::settings().remove(0);
        let off = setting.values["off"].clone();
        let oil = crate::sim::services()
            .into_iter()
            .find(|s| s.name.starts_with("Oil"))
            .unwrap();
        let log = std::env::temp_dir().join(format!("icarus-unmeasured-{}", std::process::id()));
        for answer in ["?", "7E8 03 7F 22 78"] {
            let connect = || {
                let mut e = Elm::connect(Box::new(Unmeasured {
                    car: SimCar::engine_off(),
                    answer,
                    atrv: 0,
                    said: Default::default(),
                }))
                .unwrap();
                assert_eq!(e.voltage, Some(12.6), "measured when it connected");
                e.take_sent();
                e
            };
            let mut e = connect();
            assert_eq!(Link::voltage(&mut e), None, "{answer:?}: not measured now");
            let refused = |e: &mut Elm, r: Result<(), String>, what: &str| {
                let why = r.unwrap_err();
                assert!(
                    why.contains("cannot measure the battery") && why.contains(what),
                    "{answer:?}: {why}"
                );
                let sent = e.take_sent();
                assert!(
                    !sent.iter().any(|(_, _, d)| crate::lock::changes_vehicle(d)),
                    "{answer:?}: {sent:02X?}"
                );
            };
            let r = start(&mut e, &throttle, 30.0).map(|_| ());
            refused(&mut e, r, "an actuator test");
            let mut e = connect();
            let r = crate::clear::clear_obd(&mut e).map(|_| ());
            refused(&mut e, r, "clearing codes");
            let mut e = connect();
            let r = crate::settings::change(&mut e, &setting, &off, &log).map(|_| ());
            refused(&mut e, r, "changing a setting");
            assert!(!log.exists(), "nothing was logged as changed");
            let mut e = connect();
            let r = crate::service::run(&mut e, &oil, true).map(|_| ());
            refused(&mut e, r, "a service routine");
        }
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

    /// ICA-109: a test the module refuses for a condition only it knows says what to do: 7F 2F 90 "put it in Park",
    /// 7F 2F 83 "turn the engine off"; control is handed back and the session left as on any failed start.
    #[test]
    fn a_refused_test_says_what_to_do() {
        crate::lock::switch_for_test(&[]);
        for (code, name, want) in [
            (
                0x90,
                "Cooling fan",
                "shift lever not in Park: put it in Park",
            ),
            (
                0x83,
                "Throttle",
                "the engine is running: turn the engine off",
            ),
        ] {
            let def = crate::sim::tests()
                .into_iter()
                .find(|t| t.name.starts_with(name))
                .unwrap();
            let mut elm =
                Elm::connect(Box::new(SimCar::engine_off().refusing_outputs(code))).unwrap();
            let e = start(&mut elm, &def, 30.0).err().unwrap();
            assert!(e.contains(want), "{code:02X}: {e}");
            let sent = elm.take_sent();
            assert!(
                sent.iter().any(|(_, _, d)| d.as_slice() == [0x10, 0x01]),
                "back to the default session: {sent:02X?}"
            );
        }
    }
}
