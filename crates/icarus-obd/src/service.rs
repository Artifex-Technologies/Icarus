//! Service functions: a maker's routines run in a module with UDS routine control (`31`): relearns (throttle, idle, steering
//! angle), resets (oil life, battery registration), TPMS sensor IDs, injector coding. Each is a `[[service]]` entry in a
//! vehicle profile, learned by watching another tool on my own vehicle (`bus`) or verified for it; never guessed into a real
//! vehicle's profile, since an unverified routine ID could start anything. A routine changes the vehicle, so it runs under
//! the write rules: the Read-Only switch (`lock`), the battery measured at 12 V or more (an adapter that cannot measure it
//! is refused), its conditions, never an airbag module nor an immobilizer, smart-key or certification module (`fence`, by
//! address, whatever the entry is called), a cable for an entry marked `wired` (coding), every run in the
//! action log. Only this module's requests go out: the session (10 01 / 10 03), tester present (3E 00), and routine
//! control (31 01 start, 31 02 stop, 31 03 results) of its own routine with its own parameters (`check`).

use crate::actuator::{battery_holds, conditions_hold, is_airbag, not_one_module, Condition};
use crate::elm::Answer;
use crate::link::Link;
use serde::Deserialize;
use std::time::{Duration, Instant};

fn thirty() -> u32 {
    30
}

#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ServiceDef {
    pub name: String,
    /// The module's request ID; answers from `rx` (module + 8 unless given).
    pub module: u16,
    #[serde(default)]
    pub rx: Option<u16>,
    /// The routine's identifier (UDS `31 01 <hi> <lo>`).
    pub routine: u16,
    /// The routine's start parameters (up to 8 bytes).
    #[serde(default)]
    pub params: Vec<u8>,
    /// The module runs routines only in the extended diagnostic session (10 03).
    #[serde(default)]
    pub extended: bool,
    /// Ask for the routine's results (31 03) until the module gives them, up to `max_seconds`.
    #[serde(default)]
    pub results: bool,
    #[serde(default)]
    pub conditions: Vec<Condition>,
    #[serde(default = "thirty")]
    pub max_seconds: u32,
    /// Writes memory (coding): only over a cable.
    #[serde(default)]
    pub wired: bool,
    #[serde(default)]
    pub warning: String,
    #[serde(default)]
    pub checked: bool,
}

pub const MAX_SECONDS: u32 = 120;
const MIN_VOLTS: f64 = 12.0;

impl ServiceDef {
    pub fn rx(&self) -> u16 {
        self.rx.unwrap_or(self.module + 8)
    }

    /// Why this definition is refused, whatever it says.
    pub fn refused(&self) -> Option<String> {
        if let Some(why) = not_one_module(self.module) {
            return Some(why);
        }
        if is_airbag(self.module, &self.name) || is_airbag(self.rx(), "") {
            return Some(
                "airbag and occupant-detection modules never run service routines from Icarus"
                    .into(),
            );
        }
        if self.module > 0x7FF || self.rx() > 0x7FF {
            return Some("not an 11-bit CAN ID".into());
        }
        if self.params.len() > 8 {
            return Some("a routine's parameters are at most 8 bytes".into());
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

    fn routine_request(&self, sub: u8) -> Vec<u8> {
        let mut r = vec![0x31, sub, (self.routine >> 8) as u8, self.routine as u8];
        if sub == 0x01 {
            r.extend(&self.params);
        }
        r
    }
}

/// The only requests the service path sends.
pub fn check(data: &[u8]) -> Result<(), String> {
    let ok = match data {
        [0x10, 0x01 | 0x03] | [0x3E, 0x00] => true,
        [0x31, 0x01, _, _, params @ ..] => params.len() <= 8,
        [0x31, 0x02 | 0x03, _, _] => true,
        _ => false,
    };
    if ok {
        Ok(())
    } else {
        Err(format!(
            "refused: {data:02X?} is not a request the service path sends"
        ))
    }
}

fn send(link: &mut dyn Link, s: &ServiceDef, data: &[u8]) -> Result<Answer, String> {
    check(data)?;
    for _ in 0..8 {
        let a = link
            .request_to(s.module.into(), s.rx().into(), data)
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

fn log(line: &str) {
    crate::audit::log(&format!("service {line}"));
}

fn hex(b: &[u8]) -> String {
    b.iter()
        .map(|x| format!("{x:02X}"))
        .collect::<Vec<_>>()
        .join(" ")
}

/// Runs the routine: the definition, the Read-Only switch, the link (`wired` for an entry that needs a cable), the
/// battery and the conditions first; the session it needs; start; its results when asked for; the default session again
/// whatever happened. Returns what the module answered (the routine's status bytes, in hex).
pub fn run(link: &mut dyn Link, s: &ServiceDef, wired: bool) -> Result<String, String> {
    if let Some(why) = s.refused() {
        return Err(why);
    }
    if let Some(why) = crate::fence::current().refuses(s.module.into(), s.rx().into()) {
        log(&format!(
            "refused {:03X} {:04X} \"{}\": a key module",
            s.module, s.routine, s.name
        ));
        return Err(why);
    }
    crate::lock::writes_allowed(link)?;
    if s.wired && !wired {
        return Err(format!(
            "{} writes a module's memory: only over a cable, never Bluetooth or Wi-Fi",
            s.name
        ));
    }
    battery_holds(link, MIN_VOLTS, "a service routine")?;
    conditions_hold(link, &s.conditions)?;
    let _awake = crate::power_guard::Inhibit::take(&format!("Service: {}", s.name)).ok();
    if s.extended {
        send(link, s, &[0x10, 0x03])?;
    }
    let result = (|| -> Result<String, String> {
        let started = send(link, s, &s.routine_request(0x01))?;
        let mut status = started.data.get(4..).map(hex).unwrap_or_default();
        log(&format!(
            "start {:03X} {:04X} \"{}\": {status}",
            s.module, s.routine, s.name
        ));
        if s.results {
            let until = Instant::now() + Duration::from_secs(s.max_seconds as u64);
            let mut last_present = Instant::now();
            loop {
                if Instant::now() >= until {
                    let _ = send(link, s, &s.routine_request(0x02));
                    return Err(format!(
                        "no result in {} s: the routine was asked to stop",
                        s.max_seconds
                    ));
                }
                std::thread::sleep(Duration::from_millis(500));
                if s.extended && last_present.elapsed() >= Duration::from_secs(2) {
                    last_present = Instant::now();
                    send(link, s, &[0x3E, 0x00])?;
                }
                match send(link, s, &s.routine_request(0x03)) {
                    Ok(a) => {
                        status = a.data.get(4..).map(hex).unwrap_or_default();
                        break;
                    }
                    // still running ("request sequence error" or "conditions not correct yet"): ask again
                    Err(e) if e.contains("sequence") || e.contains("conditions") => {}
                    Err(e) => return Err(e),
                }
            }
        }
        log(&format!(
            "done {:03X} {:04X} \"{}\": {status}",
            s.module, s.routine, s.name
        ));
        Ok(status)
    })();
    if s.extended {
        let _ = send(link, s, &[0x10, 0x01]);
    }
    if let Err(e) = &result {
        log(&format!(
            "failed {:03X} {:04X} \"{}\": {e}",
            s.module, s.routine, s.name
        ));
    }
    result
}

/// A `[[service]]` entry for a vehicle profile.
pub fn to_toml(s: &ServiceDef) -> String {
    let mut t = format!(
        "[[service]]\nname = {:?}\nmodule = 0x{:03X}\nroutine = 0x{:04X}\n",
        s.name, s.module, s.routine
    );
    if let Some(rx) = s.rx {
        t += &format!("rx = 0x{rx:03X}\n");
    }
    if !s.params.is_empty() {
        t += &format!(
            "params = [{}]\n",
            s.params
                .iter()
                .map(|b| format!("0x{b:02X}"))
                .collect::<Vec<_>>()
                .join(", ")
        );
    }
    t += &format!(
        "extended = {}\nresults = {}\nmax_seconds = {}\n",
        s.extended, s.results, s.max_seconds
    );
    if !s.conditions.is_empty() {
        t += &format!(
            "conditions = [{}]\n",
            s.conditions
                .iter()
                .map(|c| match c {
                    Condition::EngineOff => "\"engine-off\"",
                    Condition::EngineRunning => "\"engine-running\"",
                    Condition::Stationary => "\"stationary\"",
                })
                .collect::<Vec<_>>()
                .join(", ")
        );
    }
    if s.wired {
        t += "wired = true\n";
    }
    if !s.warning.is_empty() {
        t += &format!("warning = {:?}\n", s.warning);
    }
    t += &format!("checked = {}\n", s.checked);
    t
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elm::Elm;
    use crate::sim::SimCar;

    #[test]
    fn only_the_service_requests_go_out() {
        for ok in [
            &[0x10, 0x03][..],
            &[0x3E, 0x00],
            &[0x31, 0x01, 0xFF, 0x00],
            &[0x31, 0x03, 0x02, 0x03],
        ] {
            assert!(check(ok).is_ok(), "{ok:02X?}");
        }
        for bad in [
            &[0x2E, 0xA0, 0x01, 0x00][..],
            &[0x14, 0xFF, 0xFF, 0xFF],
            &[0x34, 0x00],
            &[0x27, 0x01],
            &[0x31, 0x01, 0xFF, 0x00, 1, 2, 3, 4, 5, 6, 7, 8, 9],
            &[0x31, 0x04, 0xFF, 0x00],
        ] {
            assert!(check(bad).is_err(), "{bad:02X?}");
        }
    }

    #[test]
    fn routines_on_the_simulated_vehicle() {
        let all = crate::sim::services();
        let relearn = all.iter().find(|s| s.name.starts_with("Throttle")).unwrap();
        // engine running: refused before anything is sent
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        assert!(run(&mut elm, relearn, true)
            .unwrap_err()
            .contains("engine must be off"));
        let mut elm = Elm::connect(Box::new(SimCar::engine_off())).unwrap();
        assert_eq!(
            run(&mut elm, relearn, true).unwrap(),
            "02",
            "the result: complete"
        );
        let reset = all.iter().find(|s| s.name.starts_with("Oil")).unwrap();
        assert!(run(&mut elm, reset, true).is_ok());
        // coding over a wireless link: refused
        let coding = ServiceDef {
            wired: true,
            ..reset.clone()
        };
        assert!(run(&mut elm, &coding, false).unwrap_err().contains("cable"));
        // an airbag module never
        let airbag = ServiceDef {
            module: 0x780,
            ..reset.clone()
        };
        assert!(run(&mut elm, &airbag, true).unwrap_err().contains("airbag"));
        // a routine the module does not know: its refusal
        let unknown = ServiceDef {
            routine: 0x1234,
            ..reset.clone()
        };
        assert!(run(&mut elm, &unknown, true)
            .unwrap_err()
            .contains("refused"));
        // every entry round-trips through its TOML
        for s in &all {
            let t: toml::Table = toml::from_str(&to_toml(s)).unwrap();
            let back: ServiceDef = t["service"].as_array().unwrap()[0]
                .clone()
                .try_into()
                .unwrap();
            assert_eq!(&back, s);
        }
    }

    /// A routine over a link that cannot measure the battery (a SocketCAN interface) is refused before anything is
    /// sent; with the battery measured above 12 V it runs as before.
    #[test]
    fn a_routine_runs_only_with_the_battery_measured() {
        use crate::canlink::{CanLink, SimBus};
        crate::lock::switch_for_test(&[]);
        let all = crate::sim::services();
        let reset = all.iter().find(|s| s.name.starts_with("Oil")).unwrap();
        let on_bus = |volts: Option<f64>| {
            let mut bus = SimBus::new(SimCar::engine_off());
            bus.voltage = volts;
            CanLink::connect(bus, "vcan0").unwrap()
        };
        let mut l = on_bus(None);
        let sent = l.frames().sent.len();
        let e = run(&mut l, reset, true).unwrap_err();
        assert!(
            e.contains("cannot measure the battery") && e.contains("a service routine"),
            "{e}"
        );
        assert_eq!(l.frames().sent.len(), sent, "nothing was sent");
        let e = run(&mut on_bus(Some(11.9)), reset, true).unwrap_err();
        assert!(e.contains("11.9 V"), "{e}");
        let mut l = on_bus(Some(12.6));
        assert!(run(&mut l, reset, true).is_ok());
        assert!(
            l.frames()
                .sent
                .iter()
                .any(|f| f.id == 0x7C0 && f.data[..5] == [0x04, 0x31, 0x01, 0x02, 0x03]),
            "the routine was started"
        );
    }
}
