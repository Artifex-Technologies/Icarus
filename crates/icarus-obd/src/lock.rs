//! The Read-Only switch: a safety switch per vehicle (the Garage page, or `icarus garage read-only VIN on|off`), kept by VIN
//! in `~/.config/icarus/read-only.toml` on this computer only. With it on, Icarus reads that vehicle (codes, live data,
//! module values, reports, recordings) and clears, tests and changes nothing on it. Off, which is every vehicle's default,
//! the write paths work as they always have, each behind its own checks.
//!
//! Each write path (clearing in `clear.rs`, actuator tests in `actuator.rs`, settings in `settings.rs`, and any later one)
//! asks `writes_allowed` first. When any vehicle has the switch on, that reads the VIN from the vehicle on the port at that
//! moment, so the switch follows the vehicle, not the one the window connected to earlier; a VIN that cannot be read then
//! counts as switched on (it could be that vehicle). With no vehicle switched on, nothing is asked of the vehicle.

use crate::link::Link;
use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

pub fn path() -> Option<PathBuf> {
    crate::vehicles::user_dir().and_then(|d| d.parent().map(|p| p.join("read-only.toml")))
}

fn normal(vin: &str) -> String {
    vin.trim().to_uppercase()
}

/// The VINs switched to read-only in `file` (none when it is missing or cannot be read).
pub fn list_in(file: &Path) -> BTreeSet<String> {
    #[derive(serde::Deserialize)]
    struct List {
        #[serde(default)]
        vins: Vec<String>,
    }
    std::fs::read_to_string(file)
        .ok()
        .and_then(|t| toml::from_str::<List>(&t).ok())
        .map(|m| m.vins.iter().map(|v| normal(v)).collect())
        .unwrap_or_default()
}

#[cfg(test)]
thread_local! {
    /// A test's own list, instead of the file in the real config folder.
    static FOR_TEST: std::cell::RefCell<Option<BTreeSet<String>>> = const { std::cell::RefCell::new(None) };
}

/// The VINs switched to read-only.
pub fn read_only() -> BTreeSet<String> {
    #[cfg(test)]
    if let Some(l) = FOR_TEST.with(|t| t.borrow().clone()) {
        return l;
    }
    path().map(|p| list_in(&p)).unwrap_or_default()
}

/// Turns a vehicle's Read-Only switch on or off.
pub fn set_in(file: &Path, vin: &str, on: bool) -> Result<(), String> {
    let mut all = list_in(file);
    if on {
        all.insert(normal(vin));
    } else {
        all.remove(&normal(vin));
    }
    let list: Vec<String> = all.iter().map(|v| format!("  {v:?},")).collect();
    let body = format!(
        "# Icarus: vehicles switched to read-only, by VIN (the Garage page's Read-Only switch). Icarus clears, tests and\n\
         # changes nothing on these. On this computer only.\nvins = [\n{}\n]\n",
        list.join("\n")
    );
    if let Some(d) = file.parent() {
        std::fs::create_dir_all(d).map_err(|e| format!("{}: {e}", d.display()))?;
    }
    let tmp = file.with_extension("tmp");
    std::fs::write(&tmp, body)
        .and_then(|_| std::fs::rename(&tmp, file))
        .map_err(|e| format!("{}: {e}", file.display()))
}

pub fn set(vin: &str, on: bool) -> Result<(), String> {
    set_in(&path().ok_or("no config folder")?, vin, on)
}

/// Whether a request would change the vehicle: anything but a read (OBD-II services 01 02 03 05 06 07 09 0A; KWP2000
/// and UDS 13 18 19 1A 21 22), tester present (3E), and the requests that hand things back: the default session (10 01)
/// and returning an output to the module (UDS 2F .. .. 00, KWP2000 30 .. 00). Unknown requests count as changes.
pub fn changes_vehicle(data: &[u8]) -> bool {
    !matches!(
        data,
        [0x01 | 0x02 | 0x03 | 0x05 | 0x06 | 0x07 | 0x09 | 0x0A, ..]
            | [0x13 | 0x18 | 0x19 | 0x1A | 0x21 | 0x22 | 0x3E, ..]
            | [0x10, 0x01]
            | [0x2F, _, _, 0x00]
            | [0x30, _, 0x00]
    )
}

/// The end of a VIN, enough to tell vehicles apart in a message without writing the whole number out.
fn tail(vin: &str) -> &str {
    &vin[vin.len().saturating_sub(6)..]
}

/// Whether the vehicle on `link` may be written to now; the reason when its Read-Only switch is on.
pub fn writes_allowed(link: &mut dyn Link) -> Result<(), String> {
    let list = read_only();
    if list.is_empty() {
        return Ok(());
    }
    match crate::j1979::vin(link) {
        Ok(Some(v)) if !list.contains(&normal(&v)) => Ok(()),
        Ok(Some(v)) => {
            crate::audit::log(&format!(
                "write refused: vehicle ...{} is switched to read-only",
                tail(&v)
            ));
            Err(format!(
                "read-only: this vehicle's Read-Only switch is on (VIN ending {}), so Icarus clears, tests and changes \
                 nothing on it (Garage page, or icarus garage read-only VIN off)",
                tail(&v)
            ))
        }
        Ok(None) | Err(_) => {
            crate::audit::log(
                "write refused: the VIN could not be read to check the Read-Only switch",
            );
            Err("read-only: the vehicle's VIN could not be read, so Icarus cannot tell whether its Read-Only switch is \
                 on, and writes nothing"
                .into())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elm::Elm;
    use crate::sim::SimCar;

    const SWITCHED: &str = "1ABCDEFGH12345678";

    fn switched(vins: &[&str]) {
        FOR_TEST.with(|t| *t.borrow_mut() = Some(vins.iter().map(|v| v.to_string()).collect()));
    }

    #[test]
    fn the_list() {
        let dir = std::env::temp_dir().join(format!("icarus-lock-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let f = dir.join("read-only.toml");
        assert!(list_in(&f).is_empty(), "no file: no vehicle is switched");
        set_in(&f, " 1abcdefgh12345678 ", true).unwrap();
        set_in(&f, "2ABCDEFGH12345678", true).unwrap();
        assert_eq!(
            list_in(&f).into_iter().collect::<Vec<_>>(),
            [SWITCHED, "2ABCDEFGH12345678"]
        );
        set_in(&f, SWITCHED, false).unwrap();
        assert_eq!(list_in(&f).len(), 1);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn the_switch_follows_the_vehicle_on_the_port() {
        switched(&[]);
        let mut switched_car = Elm::connect(Box::new(SimCar::new().with_vin(SWITCHED))).unwrap();
        assert!(writes_allowed(&mut switched_car).is_ok(), "off by default");
        switched(&[SWITCHED]);
        let why = writes_allowed(&mut switched_car).unwrap_err();
        assert!(
            why.starts_with("read-only") && why.contains("345678"),
            "{why}"
        );
        let mut other = Elm::connect(Box::new(SimCar::new())).unwrap();
        assert!(
            writes_allowed(&mut other).is_ok(),
            "another vehicle keeps its own switch"
        );
    }

    #[test]
    fn what_changes_a_vehicle() {
        for read in [
            &[0x01, 0x0C][..],
            &[0x03],
            &[0x09, 0x02],
            &[0x22, 0xF1, 0x90],
            &[0x21, 0x01],
            &[0x19, 0x02, 0xFF],
            &[0x3E, 0x00],
            &[0x10, 0x01],
            &[0x2F, 0x01, 0x23, 0x00],
            &[0x30, 0x05, 0x00],
        ] {
            assert!(!changes_vehicle(read), "{read:02X?}");
        }
        for change in [
            &[0x04][..],
            &[0x08, 0x01],
            &[0x10, 0x03],
            &[0x14, 0xFF, 0xFF, 0xFF],
            &[0x2E, 0xA0, 0x01, 0x00],
            &[0x2F, 0x01, 0x23, 0x03, 0x50],
            &[0x30, 0x05, 0x07, 0x50],
            &[0x31, 0x01, 0xFF, 0x00],
            &[0x3B, 0x20, 0x01],
            &[0x27, 0x01],
            &[0x11, 0x01],
            &[0x34],
            &[0x85, 0x02],
            &[],
            &[0x99],
        ] {
            assert!(changes_vehicle(change), "{change:02X?}");
        }
    }

    /// Below every write path: whatever sends a change to a switched vehicle through the adapter is refused there, and its
    /// reads and hand-backs still go.
    #[test]
    fn the_adapter_itself_refuses_changes_to_a_switched_vehicle() {
        switched(&[SWITCHED]);
        let mut car = Elm::connect(Box::new(SimCar::engine_off().with_vin(SWITCHED))).unwrap();
        let e = car.request(&[0x04]).unwrap_err();
        assert!(matches!(e, crate::elm::ObdError::ReadOnly(_)), "{e}");
        let e = car
            .request_to(0x7C0, 0x7C8, &[0x2E, 0xA0, 0x01, 0x00])
            .unwrap_err();
        assert!(e.to_string().starts_with("read-only"), "{e}");
        assert!(car.request(&[0x01, 0x0C]).is_ok(), "reads go");
        assert!(
            car.request_to(0x7E0, 0x7E8, &[0x10, 0x01]).is_ok(),
            "the default session goes"
        );
        // the switch off: the same change reaches the vehicle
        switched(&[]);
        assert!(!matches!(
            car.request(&[0x04]),
            Err(crate::elm::ObdError::ReadOnly(_))
        ));
    }

    /// A vehicle switched to read-only, ignition on and engine off, which every write path would otherwise accept: each
    /// one refuses before it sends anything that changes the vehicle.
    #[test]
    fn every_write_path_refuses_a_vehicle_switched_to_read_only() {
        switched(&[SWITCHED]);
        let car = || Elm::connect(Box::new(SimCar::engine_off().with_vin(SWITCHED))).unwrap();
        let read_only = |r: Result<(), String>| {
            let e = r.unwrap_err();
            assert!(e.starts_with("read-only"), "{e}");
        };
        read_only(crate::clear::clear_obd(&mut car()).map(|_| ()));
        read_only(crate::clear::clear_module(
            &mut car(),
            0x7E1,
            0x7E9,
            crate::uds::Dialect::Uds,
        ));
        let t = crate::sim::tests().remove(0);
        read_only(crate::actuator::start(&mut car(), &t, 30.0).map(|_| ()));
        let s = crate::sim::settings().remove(0);
        let v = s.values.values().next().unwrap().clone();
        let log = std::env::temp_dir().join(format!("icarus-lock-log-{}", std::process::id()));
        read_only(crate::settings::change(&mut car(), &s, &v, &log).map(|_| ()));
        assert!(!log.exists(), "nothing was logged as changed");
    }
}
