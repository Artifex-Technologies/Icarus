//! Module settings: what a dealer tool calls customizations, a feature switched on or off, a value a module keeps (a chime,
//! a lock behaviour, a light). Icarus learns them by watching such a tool on my own vehicle's bus (`bus`), keeps them as
//! `[[setting]]` entries in the vehicle's profile, and changes them itself. A setting is written to the module's memory, so it
//! is fenced in, in code:
//!
//! - Only these requests go out: reading and writing one data identifier (UDS `22`/`2E`, KWP2000 `21`/`3B`), the diagnostic
//!   session the module needs (`10 03` and back to `10 01`), tester present, and OBD-II reads for the conditions. Security
//!   access, routines, programming and anything else are refused (`check`).
//! - Airbag (SRS) and occupant-detection modules, immobiliser and key settings and odometer values are never changed, whatever
//!   a definition says.
//! - Conditions are checked first (by default: engine off, vehicle not moving) and the battery must be at 12 V or more.
//! - The value there before is read and written to the vehicle's change log *before* anything is written; after the write the
//!   value is read back, and if it is not what was written, the old one is written back. `undo` restores the last change.
//! - Every change goes to `~/.local/state/icarus/actions.log` (see `audit`).

use crate::actuator::{conditions_hold, Condition, Protocol};
use crate::elm::Answer;
use crate::link::Link;
use serde::Deserialize;
use std::collections::BTreeMap;
use std::path::Path;
use std::time::Duration;

/// A module setting: its identifier and the values it can take, by name (`values = { off = [0x00], on = [0x01] }`).
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct SettingDef {
    pub name: String,
    /// The module's request ID; answers from `rx` (module + 8 unless given).
    pub module: u16,
    #[serde(default)]
    pub rx: Option<u16>,
    pub protocol: Protocol,
    /// The data identifier (2 bytes for UDS, 1 for KWP2000).
    pub id: u16,
    pub values: BTreeMap<String, Vec<u8>>,
    /// Whether the module needs the extended diagnostic session (10 03) to change it (UDS).
    #[serde(default)]
    pub extended: bool,
    #[serde(default = "parked")]
    pub conditions: Vec<Condition>,
    #[serde(default)]
    pub note: String,
    /// Seen working on a vehicle (a learned setting starts unchecked).
    #[serde(default)]
    pub checked: bool,
}

fn parked() -> Vec<Condition> {
    vec![Condition::EngineOff, Condition::Stationary]
}

/// Below this the module may not finish writing its memory.
const MIN_VOLTS: f64 = 12.0;
/// A setting is short: longer data is not a setting.
const MAX_LEN: usize = 16;

impl SettingDef {
    pub fn rx(&self) -> u16 {
        self.rx.unwrap_or(self.module + 8)
    }

    /// Why this definition is refused, whatever it says.
    pub fn refused(&self) -> Option<String> {
        let words = format!("{} {}", self.name, self.note).to_lowercase();
        if let Some(why) = crate::actuator::not_one_module(self.module) {
            return Some(why);
        }
        // the one airbag check (actuator::is_airbag), by request ID, answer ID and name
        if crate::actuator::is_airbag(self.module, &words) || crate::actuator::is_airbag(self.rx(), "") {
            return Some("airbag and occupant-detection modules are never changed".into());
        }
        if [
            "immobil",
            "transponder",
            "smart key",
            "key code",
            "pats",
            "anti-theft",
            "antitheft",
        ]
        .iter()
        .any(|w| words.contains(w))
        {
            return Some("immobiliser and key settings are never changed".into());
        }
        if ["odometer", "mileage", "odo "]
            .iter()
            .any(|w| words.contains(w))
        {
            return Some("odometer values are never changed".into());
        }
        if self.protocol == Protocol::Kwp && self.id > 0xFF {
            return Some("a KWP2000 local identifier is one byte".into());
        }
        if self.module > 0x7FF || self.rx() > 0x7FF {
            return Some("not an 11-bit CAN ID".into());
        }
        let mut lens = self.values.values().map(Vec::len);
        let Some(len) = lens.next() else {
            return Some("a setting needs at least one value".into());
        };
        if len == 0 || len > MAX_LEN || lens.any(|l| l != len) {
            return Some(format!(
                "every value is the same length, 1 to {MAX_LEN} bytes"
            ));
        }
        None
    }

    fn read_request(&self) -> Vec<u8> {
        match self.protocol {
            Protocol::Uds => vec![0x22, (self.id >> 8) as u8, self.id as u8],
            Protocol::Kwp => vec![0x21, self.id as u8],
        }
    }

    fn write_request(&self, value: &[u8]) -> Vec<u8> {
        let mut r = match self.protocol {
            Protocol::Uds => vec![0x2E, (self.id >> 8) as u8, self.id as u8],
            Protocol::Kwp => vec![0x3B, self.id as u8],
        };
        r.extend_from_slice(value);
        r
    }

    /// The value's bytes from a read answer (62 hi lo ... / 61 lid ...).
    fn value_in(&self, a: &Answer) -> Option<Vec<u8>> {
        match (self.protocol, a.data.as_slice()) {
            (Protocol::Uds, [0x62, hi, lo, v @ ..])
                if u16::from_be_bytes([*hi, *lo]) == self.id && !v.is_empty() =>
            {
                Some(v.to_vec())
            }
            (Protocol::Kwp, [0x61, lid, v @ ..]) if *lid as u16 == self.id && !v.is_empty() => {
                Some(v.to_vec())
            }
            _ => None,
        }
    }

    /// A value in words: its name, else its bytes.
    pub fn show(&self, value: &[u8]) -> String {
        match self.name_of(value) {
            Some(n) => n.to_string(),
            None => format!("{} (not one of its values)", hexs(value)),
        }
    }

    /// The name of a value, when it is one of the setting's.
    pub fn name_of(&self, value: &[u8]) -> Option<&str> {
        self.values
            .iter()
            .find(|(_, v)| v.as_slice() == value)
            .map(|(n, _)| n.as_str())
    }
}

/// The only requests the settings path sends.
pub fn check(data: &[u8]) -> Result<(), String> {
    let ok = match data {
        [0x10, 0x01 | 0x03] => true,
        [0x3E, 0x00 | 0x80] => true,
        [0x22, _, _] | [0x21, _] => true,
        [0x2E, _, _, v @ ..] => !v.is_empty() && v.len() <= MAX_LEN,
        [0x3B, _, v @ ..] => !v.is_empty() && v.len() <= MAX_LEN,
        [0x01, _] => true,
        _ => false,
    };
    if ok {
        Ok(())
    } else {
        Err(format!(
            "refused: {data:02X?} is not a request the settings path sends"
        ))
    }
}

fn send(link: &mut dyn Link, s: &SettingDef, data: &[u8]) -> Result<Answer, String> {
    check(data)?;
    for _ in 0..4 {
        let a = link
            .request_to(s.module, s.rx(), data)
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

/// The setting's value now (and its name, when it is one of the setting's values). Reads only.
pub fn read(link: &mut dyn Link, s: &SettingDef) -> Result<(Vec<u8>, Option<String>), String> {
    if let Some(why) = s.refused() {
        return Err(why);
    }
    let a = send(link, s, &s.read_request())?;
    let v = s
        .value_in(&a)
        .ok_or_else(|| format!("an answer that is not the value: {:02X?}", a.data))?;
    let name = s.name_of(&v).map(str::to_string);
    Ok((v, name))
}

/// One change, as the vehicle's change log keeps it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Change {
    pub setting: String,
    pub module: u16,
    pub id: u16,
    pub before: Vec<u8>,
    pub after: Vec<u8>,
}

impl Change {
    fn line(&self, when: &str) -> String {
        format!(
            "{when}\t{}\t{:03X}\t{:04X}\t{}\t{}",
            self.setting,
            self.module,
            self.id,
            hexs(&self.before),
            hexs(&self.after)
        )
    }
}

fn hexs(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02X}")).collect()
}

fn unhex(s: &str) -> Option<Vec<u8>> {
    if s.is_empty() || !s.len().is_multiple_of(2) || !s.is_ascii() {
        return None;
    }
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).ok())
        .collect()
}

/// The vehicle's change log (`Logs/settings-changes.tsv` in its folder): when, setting, module, identifier, before, after.
pub fn log_path(vehicle_dir: &Path) -> std::path::PathBuf {
    vehicle_dir.join("Logs").join("settings-changes.tsv")
}

/// The change log for the vehicle with this VIN (`garage::logs_dir`).
pub fn log_for(vin: Option<&str>) -> Result<std::path::PathBuf, String> {
    Ok(crate::garage::logs_dir(vin)?.join("settings-changes.tsv"))
}

fn append(log: &Path, c: &Change) -> Result<(), String> {
    if let Some(d) = log.parent() {
        std::fs::create_dir_all(d).map_err(|e| e.to_string())?;
    }
    use std::io::Write;
    let mut f = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(log)
        .map_err(|e| format!("{}: {e}", log.display()))?;
    writeln!(f, "{}", c.line(&crate::report::now())).map_err(|e| e.to_string())
}

/// The last change of this setting in the log.
pub fn last_change(log: &Path, s: &SettingDef) -> Option<Change> {
    let text = std::fs::read_to_string(log).ok()?;
    text.lines().rev().find_map(|l| {
        let f: Vec<&str> = l.split('\t').collect();
        let [_, name, module, id, before, after] = f.as_slice() else {
            return None;
        };
        let (module, id) = (
            u16::from_str_radix(module, 16).ok()?,
            u16::from_str_radix(id, 16).ok()?,
        );
        if module != s.module || id != s.id {
            return None;
        }
        Some(Change {
            setting: name.to_string(),
            module,
            id,
            before: unhex(before)?,
            after: unhex(after)?,
        })
    })
}

/// Whether the setting can be changed now: the battery at 12 V or more and the setting's conditions (checked again by
/// `change`).
pub fn ready(link: &mut dyn Link, s: &SettingDef) -> Result<(), String> {
    if let Some(why) = s.refused() {
        return Err(why);
    }
    match link.voltage() {
        Some(v) if v < MIN_VOLTS => {
            return Err(format!("the battery is at {v:.1} V; changing a setting needs at least {MIN_VOLTS} V (connect a charger)"))
        }
        _ => {}
    }
    conditions_hold(link, &s.conditions)
}

/// Changes the setting to `value` (bytes): conditions and battery first, the value there before read and written to `log`,
/// the new one written and read back (the old one written back if it did not take). Returns the change, or None when the
/// setting already had that value (nothing written).
pub fn change(
    link: &mut dyn Link,
    s: &SettingDef,
    value: &[u8],
    log: &Path,
) -> Result<Option<Change>, String> {
    if let Some(why) = s.refused() {
        return Err(why);
    }
    let len = s.values.values().next().map(Vec::len).unwrap_or(0);
    if value.len() != len {
        return Err(format!("{} takes {len} byte(s)", s.name));
    }
    ready(link, s)?;
    let session = s.extended && s.protocol == Protocol::Uds;
    if session {
        send(link, s, &[0x10, 0x03])?;
    }
    let result = (|| {
        let (before, _) = read(link, s)?;
        if before == value {
            return Ok(None);
        }
        let c = Change {
            setting: s.name.clone(),
            module: s.module,
            id: s.id,
            before: before.clone(),
            after: value.to_vec(),
        };
        // the way back is on disk before anything is written
        append(log, &c)
            .map_err(|e| format!("not changed: the change log could not be written ({e})"))?;
        send(link, s, &s.write_request(value))?;
        let (now, _) = read(link, s)?;
        if now != value {
            let back = send(link, s, &s.write_request(&before));
            return Err(format!(
                "the module kept {} instead of {}; {}",
                hexs(&now),
                hexs(value),
                if back.is_ok() {
                    format!("{} written back", hexs(&before))
                } else {
                    "writing the old value back failed too".into()
                }
            ));
        }
        crate::audit::log(&format!(
            "setting {:?} on {:03X} {:04X}: {} -> {}",
            s.name,
            s.module,
            s.id,
            hexs(&before),
            hexs(value)
        ));
        Ok(Some(c))
    })();
    if session {
        let _ = send(link, s, &[0x10, 0x01]);
    }
    result
}

/// Puts the setting back to what it was before its last change in `log`.
pub fn undo(link: &mut dyn Link, s: &SettingDef, log: &Path) -> Result<Option<Change>, String> {
    let last = last_change(log, s)
        .ok_or_else(|| format!("no change of {} in {}", s.name, log.display()))?;
    change(link, s, &last.before, log)
}

/// A `[[setting]]` entry for a vehicle profile.
pub fn to_toml(s: &SettingDef) -> String {
    let mut t = format!(
        "[[setting]]\nname = {:?}\nmodule = 0x{:03X}\n",
        s.name, s.module
    );
    if let Some(rx) = s.rx {
        t += &format!("rx = 0x{rx:03X}\n");
    }
    t += &format!(
        "protocol = \"{}\"\nid = 0x{:X}\n",
        if s.protocol == Protocol::Uds {
            "uds"
        } else {
            "kwp"
        },
        s.id
    );
    let vals: Vec<String> = s
        .values
        .iter()
        .map(|(n, v)| {
            format!(
                "{} = [{}]",
                toml_key(n),
                v.iter()
                    .map(|b| format!("0x{b:02X}"))
                    .collect::<Vec<_>>()
                    .join(", ")
            )
        })
        .collect();
    t += &format!("values = {{ {} }}\n", vals.join(", "));
    if s.extended {
        t += "extended = true\n";
    }
    if s.conditions != parked() {
        let c: Vec<&str> = s
            .conditions
            .iter()
            .map(|c| match c {
                Condition::EngineOff => "\"engine-off\"",
                Condition::EngineRunning => "\"engine-running\"",
                Condition::Stationary => "\"stationary\"",
            })
            .collect();
        t += &format!("conditions = [{}]\n", c.join(", "));
    }
    if !s.note.is_empty() {
        t += &format!("note = {:?}\n", s.note);
    }
    t += &format!("checked = {}\n", s.checked);
    t
}

fn toml_key(k: &str) -> String {
    if !k.is_empty()
        && k.chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_')
    {
        k.to_string()
    } else {
        format!("{k:?}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elm::Elm;
    use crate::sim::SimCar;

    fn chime() -> SettingDef {
        crate::sim::settings()
            .into_iter()
            .find(|s| s.module == 0x7C0)
            .unwrap()
    }

    fn scratch(name: &str) -> std::path::PathBuf {
        let d = std::env::temp_dir().join(format!("icarus-settings-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        d
    }

    #[test]
    fn a_setting_is_changed_kept_and_undone() {
        let mut elm = Elm::connect(Box::new(SimCar::engine_off())).unwrap();
        let s = chime();
        let dir = scratch("change");
        let log = log_path(&dir);
        assert_eq!(read(&mut elm, &s).unwrap(), (vec![0x01], Some("on".into())));
        let c = change(&mut elm, &s, &s.values["off"], &log)
            .unwrap()
            .unwrap();
        assert_eq!(
            (c.before.clone(), c.after.clone()),
            (vec![0x01], vec![0x00])
        );
        assert_eq!(read(&mut elm, &s).unwrap().1.as_deref(), Some("off"));
        assert_eq!(
            last_change(&log, &s),
            Some(c),
            "the way back is in the vehicle's log"
        );
        assert!(
            change(&mut elm, &s, &s.values["off"], &log)
                .unwrap()
                .is_none(),
            "already so: nothing written"
        );
        undo(&mut elm, &s, &log).unwrap();
        assert_eq!(read(&mut elm, &s).unwrap().1.as_deref(), Some("on"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn kwp_settings_and_the_conditions() {
        let s = crate::sim::settings()
            .into_iter()
            .find(|s| s.protocol == Protocol::Kwp)
            .unwrap();
        let dir = scratch("kwp");
        let mut running = Elm::connect(Box::new(SimCar::new())).unwrap();
        assert!(
            change(&mut running, &s, &s.values["on"], &log_path(&dir))
                .unwrap_err()
                .contains("engine must be off"),
            "parked, engine off"
        );
        let mut off = Elm::connect(Box::new(SimCar::engine_off())).unwrap();
        change(&mut off, &s, &s.values["on"], &log_path(&dir)).unwrap();
        assert_eq!(read(&mut off, &s).unwrap().1.as_deref(), Some("on"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn the_same_through_a_j2534_device() {
        use crate::passthru::{PassThruLink, SimPassThru};
        let mut link = PassThruLink::connect(SimPassThru::new(SimCar::engine_off())).unwrap();
        let s = chime();
        let dir = scratch("j2534");
        change(&mut link, &s, &s.values["off"], &log_path(&dir))
            .unwrap()
            .unwrap();
        assert_eq!(read(&mut link, &s).unwrap().1.as_deref(), Some("off"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_value_the_module_does_not_take_is_put_back() {
        let mut elm = Elm::connect(Box::new(SimCar::engine_off())).unwrap();
        let mut s = chime();
        s.values.insert("two".into(), vec![0x02]); // the simulated cluster refuses 02
        let dir = scratch("refused");
        let e = change(&mut elm, &s, &[0x02], &log_path(&dir)).unwrap_err();
        assert!(e.contains("refused"), "{e}");
        assert_eq!(
            read(&mut elm, &s).unwrap().1.as_deref(),
            Some("on"),
            "unchanged"
        );
        let mut no_session = chime();
        no_session.extended = false;
        assert!(change(&mut elm, &no_session, &[0x00], &log_path(&dir))
            .unwrap_err()
            .contains("refused"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn what_is_never_changed() {
        let mut s = chime();
        s.module = 0x780;
        assert!(s.refused().unwrap().contains("airbag"));
        let mut s = chime();
        s.name = "Smart key: register".into();
        assert!(s.refused().unwrap().contains("immobiliser"));
        let mut s = chime();
        s.note = "odometer offset".into();
        assert!(s.refused().unwrap().contains("odometer"));
        let mut s = chime();
        s.values.insert("long".into(), vec![0, 0]);
        assert!(s.refused().is_some(), "values of different lengths");
        assert!(
            check(&[0x27, 0x01]).is_err()
                && check(&[0x31, 0x01, 0xFF, 0x00]).is_err()
                && check(&[0x34, 0, 0x44]).is_err()
        );
        assert!(check(&[0x2E, 0xA0, 0x01, 0x00]).is_ok() && check(&[0x2E, 0xA0, 0x01]).is_err());
    }

    #[test]
    fn a_profile_entry_round_trips() {
        let s = chime();
        let text = format!("name = \"x\"\n{}", to_toml(&s));
        let p = crate::vehicles::parse("x", "test", &text).unwrap();
        assert_eq!(p.settings, vec![s]);
    }
}
