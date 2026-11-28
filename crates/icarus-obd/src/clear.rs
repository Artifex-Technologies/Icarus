//! Clearing trouble codes. Small, but it erases evidence: the freeze frame goes with the codes, and every readiness monitor
//! starts over (the vehicle will not pass an emissions inspection until its drive cycles have run them again). So:
//!
//! - the engine must be off (ignition on), as the OBD-II standard expects;
//! - the battery must be measured at 11.5 V or more, as for actuator tests: an adapter that cannot measure it is refused;
//! - the caller saves a report first (`report`), so the codes and freeze frame are kept;
//! - only the clearing requests go out: OBD-II service `04` (every emissions module), UDS `14 FF FF FF` and KWP2000
//!   `14 FF 00` (one module, all its codes);
//! - permanent codes are not cleared by any of these: they go when the monitor that set them passes again;
//! - one module's codes never on an airbag module (by its ID, 11- or 29-bit, its name, or a profile's name for its
//!   address), nor, where the vehicle's profile names no airbag module or the make is not known, on a module of unknown
//!   kind, which could be it (`refused_on`, as module data asks it: ICA-200), nor on a key module (`fence`);
//! - every clear is written to the action log.

use crate::actuator::{battery_holds, conditions_hold, Condition};
use crate::link::Link;
use crate::uds::Dialect;
use crate::vehicles::Profile;
use icarus_core::moddata::{Ids, ModuleHead};

/// The battery a clear needs, measured at the port: as for an actuator test (one short request, engine off).
const MIN_VOLTS: f64 = 11.5;

pub fn check(data: &[u8]) -> Result<(), String> {
    match data {
        [0x04] | [0x14, 0xFF, 0xFF, 0xFF] | [0x14, 0xFF, 0x00] => Ok(()),
        _ => Err(format!(
            "refused: {data:02X?} is not a clearing request Icarus sends"
        )),
    }
}

/// What clearing loses, for the confirmation.
pub const WARNING: &str = "Clearing erases the stored and pending codes and the freeze frame (Icarus saves a report first), \
and resets every readiness monitor: the vehicle will not pass an emissions inspection until it has been driven enough to \
run them again. Permanent codes stay until their monitor passes. If the fault is still there, the code comes back.";

/// OBD-II service 04 to every module: returns the modules that confirmed.
pub fn clear_obd(elm: &mut dyn Link) -> Result<Vec<String>, String> {
    crate::lock::writes_allowed(elm)?;
    battery_holds(elm, MIN_VOLTS, "clearing codes")?;
    conditions_hold(elm, &[Condition::EngineOff])?;
    let _awake = crate::power_guard::Inhibit::take("Clearing trouble codes").ok();
    check(&[0x04])?;
    let answers = elm.request(&[0x04]).map_err(|e| e.to_string())?;
    if let Some((_, code)) = answers.iter().find_map(|a| a.refusal()) {
        crate::audit::log(&format!(
            "clear OBD-II codes refused: {}",
            crate::uds::nrc(code)
        ));
        return Err(format!("a module refused: {}", crate::uds::nrc(code)));
    }
    let ok: Vec<String> = answers
        .iter()
        .filter(|a| a.data.first() == Some(&0x44))
        .map(|a| format!("{} ({})", a.module(), a.from))
        .collect();
    crate::audit::log(&format!("clear OBD-II codes: {}", ok.join(", ")));
    if ok.is_empty() {
        return Err("no module confirmed".into());
    }
    Ok(ok)
}

/// "7B0", "18DA53F1".
fn id_text(id: u32) -> String {
    if id > 0x7FF {
        format!("{id:08X}")
    } else {
        format!("{id:03X}")
    }
}

/// Why Icarus will not clear one module's codes, whatever asks: airbag and occupant-detection modules never (their codes
/// stay for a dealer or an airbag specialist, after the repair), by ID (11- or 29-bit, request or answer:
/// `actuator::is_airbag`) and by name where the caller knows it; and only one module's request ID on 11-bit CAN
/// (`actuator::not_one_module`). The vehicle's make is not asked here: `refused_on` adds what its profile says.
pub fn refused(tx: u32, rx: u32, name: &str) -> Option<String> {
    let hit = if crate::actuator::is_airbag(tx, name) {
        Some(tx)
    } else if crate::actuator::is_airbag(rx, "") {
        Some(rx)
    } else {
        None
    };
    if let Some(hit) = hit {
        return Some(format!(
            "refused: {} is an airbag or occupant-detection module, and Icarus never clears its codes \
             (a dealer or an airbag specialist does, after the repair)",
            id_text(hit)
        ));
    }
    match u16::try_from(tx) {
        Ok(t) => crate::actuator::not_one_module(t),
        Err(_) => Some(format!(
            "refused: {} is not a module request ID (700-7F7): Icarus clears one module's codes on 11-bit CAN only",
            id_text(tx)
        )),
    }
}

/// `refused`, and what the vehicle's profile says (ICA-200), as module data asks it (`moddata::Gates`): on a vehicle of
/// `make` (a profile's id; None: not known), a module a profile names as an airbag module is refused at its address
/// whatever it is called, and where the make's profile names no airbag module (or the make is not known) a module of
/// unknown kind is refused too, since it could be the airbag module: one the sweep found, or one the profile does not
/// name. A module the make's profile names (with its name, or a `kind`) is of known kind.
pub fn refused_on(
    profiles: &[Profile],
    make: Option<&str>,
    tx: u32,
    rx: u32,
    name: &str,
) -> Option<String> {
    let profile = make.and_then(|m| profiles.iter().find(|p| p.id.eq_ignore_ascii_case(m)));
    let named = profile.and_then(|p| p.modules.iter().find(|a| a.tx == tx && a.ext.is_none()));
    let name = named.map_or(name, |a| a.name.as_str());
    if let Some(why) = refused(tx, rx, name) {
        return Some(why);
    }
    let head = ModuleHead {
        name: name.to_string(),
        tx,
        rx: Some(rx),
        ext: None,
        tester: None,
        ids: if tx > 0x7FF { Ids::Can29 } else { Ids::Can11 },
        bus: None,
        dialect: None,
        kind: named.map(|a| a.kind.clone()).unwrap_or_default(),
        identity: Vec::new(),
        protocol: None,
        header_byte: None,
    };
    let gates = crate::moddata::Gates::for_make(profiles, make);
    if gates.is_airbag(&head) {
        return Some(format!(
            "refused: a vehicle profile names {} as an airbag or occupant-detection module, and Icarus never clears \
             its codes (a dealer or an airbag specialist does, after the repair)",
            id_text(tx)
        ));
    }
    gates.unknown_kind(&head).map(|why| {
        format!(
            "{why}; Icarus clears the codes of a module of unknown kind only once the vehicle profile names it \
             (a [[module]] in ~/.config/icarus/vehicles/)"
        )
    })
}

/// All of one module's codes (UDS or KWP2000, as the module spoke when its codes were read), on a vehicle of `make` (a
/// profile's id; None: not known) with `profiles`. Never an airbag module, nor a module of unknown kind where the make's
/// profile names no airbag module (`refused_on`), nor an immobilizer, smart-key or certification module (`fence`): those
/// get their identity and codes read, and nothing else.
pub fn clear_module(
    elm: &mut dyn Link,
    tx: u32,
    rx: u32,
    dialect: Dialect,
    profiles: &[Profile],
    make: Option<&str>,
) -> Result<(), String> {
    if let Some(why) = refused_on(profiles, make, tx, rx, "") {
        crate::audit::log(&format!("clear {} refused: {why}", id_text(tx)));
        return Err(why);
    }
    if let Some(why) = crate::fence::current().refuses(tx, rx) {
        crate::audit::log(&format!("clear {tx:03X} refused: a key module"));
        return Err(why);
    }
    crate::lock::writes_allowed(elm)?;
    battery_holds(elm, MIN_VOLTS, "clearing codes")?;
    conditions_hold(elm, &[Condition::EngineOff])?;
    let _awake = crate::power_guard::Inhibit::take("Clearing trouble codes").ok();
    let req: &[u8] = match dialect {
        Dialect::Uds => &[0x14, 0xFF, 0xFF, 0xFF],
        Dialect::Kwp => &[0x14, 0xFF, 0x00],
    };
    check(req)?;
    for _ in 0..4 {
        let a = elm.request_to(tx, rx, req).map_err(|e| e.to_string())?;
        let Some(a) = a.into_iter().last() else {
            return Err("no answer".into());
        };
        match a.refusal() {
            Some((_, 0x78)) | Some((_, 0x21)) => {
                std::thread::sleep(std::time::Duration::from_millis(300))
            }
            Some((_, code)) => {
                crate::audit::log(&format!(
                    "clear {tx:03X} refused: {}",
                    crate::uds::nrc(code)
                ));
                return Err(format!("the module refused: {}", crate::uds::nrc(code)));
            }
            None if a.data.first() == Some(&0x54) => {
                crate::audit::log(&format!("clear {tx:03X} codes"));
                return Ok(());
            }
            None => return Err(format!("unexpected answer {:02X?}", a.data)),
        }
    }
    Err("the module stayed busy".into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elm::Elm;
    use crate::j1979::{self, DtcKind};
    use crate::sim::SimCar;
    use crate::uds;

    /// The built-in profiles: the simulated vehicle is Toyota-like, and the Toyota profile names its airbag module.
    fn toyota() -> Vec<Profile> {
        crate::vehicles::builtin()
    }

    /// The simulated vehicle, engine off, with what connecting sent taken out.
    fn connect() -> Elm {
        let mut e = Elm::connect(Box::new(SimCar::engine_off())).unwrap();
        e.take_sent();
        e
    }

    /// ICA-108: the leads' airbag modules, Nissan's 752 (answering from 772) and Honda's 29-bit 18DA53F1 (answering from
    /// 18DAF153), by request ID or by answer ID and on any make, are never cleared, and nothing is sent.
    #[test]
    fn the_nissan_and_honda_airbag_modules_are_never_cleared() {
        crate::lock::switch_for_test(&[]);
        let ps = toyota();
        for (tx, rx) in [(0x752, 0x772), (0x18DA_53F1, 0x18DA_F153)] {
            assert!(crate::actuator::is_airbag(tx, "") && crate::actuator::is_airbag(rx, ""));
            for make in [Some("nissan"), Some("honda"), Some("toyota"), None] {
                for (t, r, hit) in [(tx, rx, tx), (0x7A0, rx, rx)] {
                    let mut e = connect();
                    let why = clear_module(&mut e, t, r, Dialect::Uds, &ps, make).unwrap_err();
                    assert!(
                        why.contains("airbag") && why.contains(&id_text(hit)),
                        "{make:?} {t:X}/{r:X}: {why}"
                    );
                    assert!(e.take_sent().is_empty(), "{make:?} {t:X}: nothing sent");
                }
            }
        }
    }

    /// ICA-200: on a vehicle whose profile names no airbag module (a profile written for the test, GM's) or whose make is
    /// not known, a clear aimed at a module of unknown kind (the simulated ABS module at 7B0, as a sweep finds it: the
    /// profile does not name it) is refused with module data's reason and nothing sent; the engine, which the profile names
    /// with a kind, still clears; the listed airbag IDs stay refused. On Toyota, whose profile names its airbag module and
    /// 7B0, it clears as before.
    #[test]
    fn a_module_of_unknown_kind_is_not_cleared_where_the_profile_names_no_airbag_module() {
        crate::lock::switch_for_test(&[]);
        let mut ps = toyota();
        ps.push(
            crate::vehicles::parse(
                "kart",
                "a test",
                "name = \"Kart\"\nwmi = [\"ZZK\"]\n[[module]]\ntx = 0x7E0\nname = \"engine\"\nkind = \"engine\"\n",
            )
            .unwrap(),
        );
        for make in [Some("kart"), Some("gm"), None] {
            let mut e = connect();
            let why = clear_module(&mut e, 0x7B0, 0x7B8, Dialect::Kwp, &ps, make).unwrap_err();
            assert!(
                why.contains("what 7B0 is, is not known")
                    && why.contains("could be the airbag module"),
                "{make:?}: {why}"
            );
            assert!(
                refused_on(&ps, make, 0x7B0, 0x7B8, "").as_deref() == Some(why.as_str()),
                "the same reason before anything is asked"
            );
            for (tx, rx) in [(0x780, 0x788), (0x781, 0x789), (0x752, 0x772)] {
                let why = clear_module(&mut e, tx, rx, Dialect::Uds, &ps, make).unwrap_err();
                assert!(why.contains("is an airbag"), "{make:?} {tx:X}: {why}");
            }
            assert!(e.take_sent().is_empty(), "{make:?}: nothing sent");
        }
        // named with a kind: cleared, the one request going to it
        let mut e = connect();
        clear_module(&mut e, 0x7E0, 0x7E8, Dialect::Uds, &ps, Some("kart")).unwrap();
        assert!(
            e.take_sent()
                .iter()
                .any(|(_, to, d)| *to == 0x7E0 && d[..] == [0x14, 0xFF, 0xFF, 0xFF]),
            "cleared"
        );
        // the Toyota profile names its airbag module, and 7B0
        clear_module(
            &mut connect(),
            0x7B0,
            0x7B8,
            Dialect::Kwp,
            &ps,
            Some("toyota"),
        )
        .unwrap();
        assert_eq!(refused_on(&ps, Some("toyota"), 0x7B0, 0x7B8, ""), None);
    }

    #[test]
    fn only_clearing_requests() {
        assert!(
            check(&[0x04]).is_ok()
                && check(&[0x14, 0xFF, 0xFF, 0xFF]).is_ok()
                && check(&[0x14, 0xFF, 0x00]).is_ok()
        );
        for bad in [
            &[0x14, 0x01, 0x71, 0x00][..],
            &[0x2E, 0x00],
            &[0x04, 0x00],
            &[0x31, 0x01],
        ] {
            assert!(check(bad).is_err(), "{bad:02X?}");
        }
    }

    #[test]
    fn airbag_modules_are_never_cleared() {
        // refused before anything is sent, by request ID, answer ID or name
        let mut elm = Elm::connect(Box::new(SimCar::engine_off())).unwrap();
        for (tx, rx) in [(0x780, 0x788), (0x781, 0x789)] {
            assert!(
                clear_module(&mut elm, tx, rx, Dialect::Uds, &toyota(), Some("toyota"))
                    .unwrap_err()
                    .contains("airbag")
            );
        }
        assert!(refused(0x7A0, 0x788, "").is_some(), "by its answer ID");
        assert!(
            refused(0x7A0, 0x7A8, "Front airbag sensor").is_some(),
            "by name"
        );
        assert!(refused(0x7B0, 0x7B8, "ABS / VSC / TRC (skid control)").is_none());
        assert!(refused(0x7E0, 0x7E8, "engine").is_none());
        assert!(
            refused(0x7DF, 0x7E8, "").unwrap().contains("broadcast"),
            "the broadcast reaches the airbag module too"
        );
        assert!(refused(0x123, 0x12B, "").is_some(), "not a module ID");
        assert!(
            refused(0x7A0, 0x788, "").unwrap().contains("788"),
            "names the ID that matched"
        );
    }

    /// A clear over a link that cannot measure the battery (a SocketCAN interface), or with the battery low, is refused
    /// before anything is sent; measured above 11.5 V the same clears go as before.
    #[test]
    fn codes_are_cleared_only_with_the_battery_measured() {
        use crate::canlink::{CanLink, SimBus};
        crate::lock::switch_for_test(&[]);
        let on_bus = |volts: Option<f64>| {
            let mut bus = SimBus::new(SimCar::engine_off());
            bus.voltage = volts;
            CanLink::connect(bus, "vcan0").unwrap()
        };
        for (volts, why) in [
            (None, "cannot measure the battery"),
            (Some(f64::NAN), "cannot measure the battery"),
            (Some(11.0), "at least 11.5 V"),
        ] {
            let mut l = on_bus(volts);
            let sent = l.frames().sent.len();
            for e in [
                clear_obd(&mut l).unwrap_err(),
                clear_module(
                    &mut l,
                    0x7B0,
                    0x7B8,
                    Dialect::Kwp,
                    &toyota(),
                    Some("toyota"),
                )
                .unwrap_err(),
            ] {
                assert!(e.contains(why) && e.contains("clearing codes"), "{e}");
            }
            assert_eq!(l.frames().sent.len(), sent, "nothing was sent");
        }
        let mut l = on_bus(Some(12.6));
        assert!(clear_obd(&mut l).unwrap()[0].starts_with("engine"));
        clear_module(
            &mut l,
            0x7B0,
            0x7B8,
            Dialect::Kwp,
            &toyota(),
            Some("toyota"),
        )
        .unwrap();
    }

    #[test]
    fn clearing_the_simulated_vehicle() {
        // engine running: refused, nothing sent
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        assert!(clear_obd(&mut elm)
            .unwrap_err()
            .contains("engine must be off"));
        let mut elm = Elm::connect(Box::new(SimCar::engine_off())).unwrap();
        assert_eq!(
            j1979::dtcs(&mut elm, DtcKind::Stored).unwrap()[0].2,
            vec!["P0171".to_string()]
        );
        let ok = clear_obd(&mut elm).unwrap();
        assert!(ok[0].starts_with("engine"));
        assert!(
            j1979::dtcs(&mut elm, DtcKind::Stored)
                .unwrap()
                .iter()
                .all(|d| d.2.is_empty()),
            "codes gone"
        );
        assert!(
            j1979::freeze_frame(&mut elm).unwrap().is_empty(),
            "freeze frame gone"
        );
        let a = elm.request(&[0x01, 0x01]).unwrap();
        let m = j1979::monitors(&a.iter().find(|a| a.from == "7E8").unwrap().data[2..]).unwrap();
        assert!(
            !m.mil && m.tests.iter().any(|t| !t.1),
            "light off, monitors start over"
        );
        // one module, KWP2000
        assert!(!uds::module_dtcs(&mut elm, 0x7B0, 0x7B8)
            .unwrap()
            .unwrap()
            .codes
            .is_empty());
        clear_module(
            &mut elm,
            0x7B0,
            0x7B8,
            Dialect::Kwp,
            &toyota(),
            Some("toyota"),
        )
        .unwrap();
        assert!(uds::module_dtcs(&mut elm, 0x7B0, 0x7B8)
            .unwrap()
            .unwrap()
            .codes
            .is_empty());
    }
}
