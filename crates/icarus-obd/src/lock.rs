//! Writes go only to my own vehicles. Every vehicle is read-only to Icarus until its VIN is on the list of mine
//! (`~/.config/icarus/mine.toml`, on this computer only; `icarus garage mine VIN`): a friend's car can be scanned, its codes
//! read and explained, its live data recorded, and nothing on it cleared, tested or changed. Each write path (clearing in
//! `clear.rs`, actuator tests in `actuator.rs`, settings in `settings.rs`, and any later one) asks `writes_allowed` first,
//! and it reads the VIN from the vehicle itself at that moment, so the answer is about the vehicle on the port now, not
//! the one the window connected to earlier. No VIN read means no write. The simulated vehicle is always writable.

use crate::link::Link;
use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

pub fn path() -> Option<PathBuf> {
    crate::vehicles::user_dir().and_then(|d| d.parent().map(|p| p.join("mine.toml")))
}

fn normal(vin: &str) -> String {
    vin.trim().to_uppercase()
}

/// The VINs of my vehicles in `file` (none when it is missing or cannot be read: fail closed).
pub fn mine_in(file: &Path) -> BTreeSet<String> {
    #[derive(serde::Deserialize)]
    struct Mine {
        #[serde(default)]
        vins: Vec<String>,
    }
    std::fs::read_to_string(file)
        .ok()
        .and_then(|t| toml::from_str::<Mine>(&t).ok())
        .map(|m| m.vins.iter().map(|v| normal(v)).collect())
        .unwrap_or_default()
}

pub fn mine() -> BTreeSet<String> {
    path().map(|p| mine_in(&p)).unwrap_or_default()
}

pub fn is_mine(vin: &str) -> bool {
    normal(vin) == crate::sim::VIN || mine().contains(&normal(vin))
}

/// Marks a vehicle as mine (writes allowed) or not (read-only again).
pub fn set_in(file: &Path, vin: &str, yes: bool) -> Result<(), String> {
    let mut all = mine_in(file);
    if yes {
        all.insert(normal(vin));
    } else {
        all.remove(&normal(vin));
    }
    let list: Vec<String> = all.iter().map(|v| format!("  {v:?},")).collect();
    let body = format!(
        "# Icarus: my own vehicles, by VIN. Only these can have codes cleared, tests run or settings changed;\n\
         # every other vehicle is read-only. On this computer only.\nvins = [\n{}\n]\n",
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

pub fn set(vin: &str, yes: bool) -> Result<(), String> {
    set_in(&path().ok_or("no config folder")?, vin, yes)
}

/// The end of a VIN, enough to tell vehicles apart in a message without writing the whole number out.
fn tail(vin: &str) -> &str {
    &vin[vin.len().saturating_sub(6)..]
}

/// Whether the vehicle on `link` may be written to now; the reason when not.
pub fn writes_allowed(link: &mut dyn Link) -> Result<(), String> {
    match crate::j1979::vin(link) {
        Ok(Some(v)) if is_mine(&v) => Ok(()),
        Ok(Some(v)) => {
            crate::audit::log(&format!(
                "write refused: vehicle ...{} is not marked as mine",
                tail(&v)
            ));
            Err(format!(
                "read-only: this vehicle (VIN ending {}) is not one of yours, so Icarus clears, tests and changes nothing \
                 on it. If it is yours: icarus garage mine VIN",
                tail(&v)
            ))
        }
        Ok(None) | Err(_) => {
            crate::audit::log("write refused: the vehicle's VIN could not be read");
            Err("read-only: the vehicle's VIN could not be read, so Icarus cannot tell it is yours and writes nothing".into())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_list_of_mine() {
        let dir = std::env::temp_dir().join(format!("icarus-lock-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let f = dir.join("mine.toml");
        assert!(mine_in(&f).is_empty(), "no file: nothing is mine");
        set_in(&f, " 1abcdefgh12345678 ", true).unwrap();
        set_in(&f, "2ABCDEFGH12345678", true).unwrap();
        assert_eq!(
            mine_in(&f).into_iter().collect::<Vec<_>>(),
            ["1ABCDEFGH12345678", "2ABCDEFGH12345678"]
        );
        set_in(&f, "1ABCDEFGH12345678", false).unwrap();
        assert_eq!(mine_in(&f).len(), 1);
        std::fs::write(&f, "vins = 3").unwrap();
        assert!(mine_in(&f).is_empty(), "a broken file: nothing is mine");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn the_simulated_vehicle_is_writable_and_others_are_not() {
        use crate::elm::Elm;
        use crate::sim::SimCar;
        let mut sim = Elm::connect(Box::new(SimCar::new())).unwrap();
        assert!(writes_allowed(&mut sim).is_ok());
        let mut other =
            Elm::connect(Box::new(SimCar::new().with_vin("1ABCDEFGH12345678"))).unwrap();
        let why = writes_allowed(&mut other).unwrap_err();
        assert!(
            why.starts_with("read-only") && why.contains("345678"),
            "{why}"
        );
    }

    /// Someone else's vehicle, ignition on and engine off, which every write path would otherwise accept: each one refuses
    /// before it sends anything that changes the vehicle.
    #[test]
    fn every_write_path_refuses_a_vehicle_that_is_not_mine() {
        use crate::elm::Elm;
        use crate::sim::SimCar;
        let other =
            || Elm::connect(Box::new(SimCar::engine_off().with_vin("1ABCDEFGH12345678"))).unwrap();
        let read_only = |r: Result<(), String>| {
            let e = r.unwrap_err();
            assert!(e.starts_with("read-only"), "{e}");
        };
        read_only(crate::clear::clear_obd(&mut other()).map(|_| ()));
        read_only(crate::clear::clear_module(
            &mut other(),
            0x7E1,
            0x7E9,
            crate::uds::Dialect::Uds,
        ));
        let t = crate::sim::tests().remove(0);
        read_only(crate::actuator::start(&mut other(), &t, 30.0).map(|_| ()));
        let s = crate::sim::settings().remove(0);
        let v = s.values.values().next().unwrap().clone();
        let log = std::env::temp_dir().join(format!("icarus-lock-log-{}", std::process::id()));
        read_only(crate::settings::change(&mut other(), &s, &v, &log).map(|_| ()));
        assert!(!log.exists(), "nothing was logged as changed");
    }
}
