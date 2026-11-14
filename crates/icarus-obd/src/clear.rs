//! Clearing trouble codes. Small, but it erases evidence: the freeze frame goes with the codes, and every readiness monitor
//! starts over (the vehicle will not pass an emissions inspection until its drive cycles have run them again). So:
//!
//! - the engine must be off (ignition on), as the OBD-II standard expects;
//! - the caller saves a report first (`report`), so the codes and freeze frame are kept;
//! - only the clearing requests go out: OBD-II service `04` (every emissions module), UDS `14 FF FF FF` and KWP2000
//!   `14 FF 00` (one module, all its codes);
//! - permanent codes are not cleared by any of these: they go when the monitor that set them passes again;
//! - every clear is written to the action log.

use crate::actuator::{conditions_hold, Condition};
use crate::link::Link;
use crate::uds::Dialect;

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

/// Why Icarus will not clear one module's codes, whatever asks: airbag and occupant-detection modules never (their codes
/// stay for a dealer or an airbag specialist, after the repair). By ID, and by name where the caller knows it.
pub fn refused(tx: u16, rx: u16, name: &str) -> Option<String> {
    if let Some(why) = crate::actuator::not_one_module(tx) {
        return Some(why);
    }
    let hit = if crate::actuator::is_airbag(tx, name) {
        tx
    } else if crate::actuator::is_airbag(rx, "") {
        rx
    } else {
        return None;
    };
    Some(format!(
        "refused: {hit:03X} is an airbag or occupant-detection module, and Icarus never clears its codes \
         (a dealer or an airbag specialist does, after the repair)"
    ))
}

/// All of one module's codes (UDS or KWP2000, as the module spoke when its codes were read). Never an airbag module.
pub fn clear_module(elm: &mut dyn Link, tx: u16, rx: u16, dialect: Dialect) -> Result<(), String> {
    if let Some(why) = refused(tx, rx, "") {
        crate::audit::log(&format!("clear {tx:03X} refused: an airbag module"));
        return Err(why);
    }
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
            assert!(clear_module(&mut elm, tx, rx, Dialect::Uds)
                .unwrap_err()
                .contains("airbag"));
        }
        assert!(refused(0x7A0, 0x788, "").is_some(), "by its answer ID");
        assert!(refused(0x7A0, 0x7A8, "Front airbag sensor").is_some(), "by name");
        assert!(refused(0x7B0, 0x7B8, "ABS / VSC / TRC (skid control)").is_none());
        assert!(refused(0x7E0, 0x7E8, "engine").is_none());
        assert!(refused(0x7DF, 0x7E8, "").unwrap().contains("broadcast"), "the broadcast reaches the airbag module too");
        assert!(refused(0x123, 0x12B, "").is_some(), "not a module ID");
        assert!(refused(0x7A0, 0x788, "").unwrap().contains("788"), "names the ID that matched");
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
            .1
            .is_empty());
        clear_module(&mut elm, 0x7B0, 0x7B8, Dialect::Kwp).unwrap();
        assert!(uds::module_dtcs(&mut elm, 0x7B0, 0x7B8)
            .unwrap()
            .unwrap()
            .1
            .is_empty());
    }
}
