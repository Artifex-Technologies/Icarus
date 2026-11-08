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
use crate::elm::Elm;
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
pub fn clear_obd(elm: &mut Elm) -> Result<Vec<String>, String> {
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

/// All of one module's codes (UDS or KWP2000, as the module spoke when its codes were read).
pub fn clear_module(elm: &mut Elm, tx: u16, rx: u16, dialect: Dialect) -> Result<(), String> {
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
