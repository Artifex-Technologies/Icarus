//! Module settings: what a dealer tool calls customizations, a feature switched on or off, a value a module keeps (a chime,
//! a lock behavior, a light). Icarus learns them by watching such a tool on my own vehicle's bus (`bus`), keeps them as
//! `[[setting]]` entries in the vehicle's profile, and changes them itself. A setting is written to the module's memory, so it
//! is fenced in, in code:
//!
//! - Only these requests go out: reading and writing one data identifier (UDS `22`/`2E`, KWP2000 `21`/`3B`), the diagnostic
//!   session the module needs (`10 03` and back to `10 01`), tester present, and OBD-II reads for the conditions. Security
//!   access, routines, programming and anything else are refused (`check`).
//! - Airbag (SRS) and occupant-detection modules, immobilizer and key settings and odometer values are never changed, whatever
//!   a definition says; nor is anything in an immobilizer, smart-key or certification module, by address (`fence`), whatever
//!   the setting is called. Such a module's settings are not even read.
//! - Conditions are checked first (by default: engine off, vehicle not moving) and the battery must be measured at 12 V or
//!   more: an adapter that cannot measure it is refused.
//! - The value there before is read and written to the vehicle's change log *before* anything is written; after the write the
//!   value is read back, and if it is not what was written, the old one is written back. `undo` restores the last change.
//! - Every change goes to `~/.local/state/icarus/actions.log` (see `audit`).
//!
//! **Settings that share an identifier** (ICA-102). A module may pack several settings into one data identifier, each in its
//! own bits. Such a setting has a `mask` (as long as its values): a change writes the value read just before with only the
//! masked bits replaced (`(before AND NOT mask) OR (new AND mask)`), the read-back and the put-back look at and touch only
//! those bits, and undo puts back only them. So changing or undoing one setting never puts another back to what it was. A
//! setting without a mask is the whole value, as before. The bus learner gives a learned setting the bits the other tool
//! changed (before XOR after).

use crate::actuator::{battery_holds, conditions_hold, Condition, Protocol};
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
    /// The bits of the identifier's value that are this setting's, as long as its values: only they are compared, written
    /// and put back (the rest belong to other settings packed into the same identifier). None: the whole value.
    #[serde(default)]
    pub mask: Option<Vec<u8>>,
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
        if crate::actuator::is_airbag(self.module, &words)
            || crate::actuator::is_airbag(self.rx(), "")
        {
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
            return Some("immobilizer and key settings are never changed".into());
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
        match &self.mask {
            Some(m) if m.len() != len => Some(format!(
                "the mask is as long as the values ({len} byte(s)), not {}",
                m.len()
            )),
            Some(m) if m.iter().all(|&b| b == 0) => {
                Some("a mask of no bits leaves nothing to change".into())
            }
            _ => None,
        }
    }

    /// Why Icarus will not touch this setting now: `refused`, or a key module at its address (`fence`).
    fn allowed(&self) -> Result<(), String> {
        if let Some(why) = self.refused() {
            return Err(why);
        }
        match crate::fence::current().refuses(self.module.into(), self.rx().into()) {
            Some(why) => Err(why),
            None => Ok(()),
        }
    }

    /// `value` with only this setting's bits kept (the whole value without a mask).
    pub fn masked(&self, value: &[u8]) -> Vec<u8> {
        match &self.mask {
            Some(m) => value.iter().zip(m).map(|(v, m)| v & m).collect(),
            None => value.to_vec(),
        }
    }

    /// Whether two values are the same as far as this setting goes (its masked bits).
    pub fn same(&self, a: &[u8], b: &[u8]) -> bool {
        a.len() == b.len() && self.masked(a) == self.masked(b)
    }

    /// The identifier's value with this setting's bits taken from `new` and every other bit from `now` (the value read
    /// just before): what a change writes. Without a mask, `new`.
    pub fn apply(&self, now: &[u8], new: &[u8]) -> Vec<u8> {
        match &self.mask {
            Some(m) if now.len() == m.len() && new.len() == m.len() => now
                .iter()
                .zip(new)
                .zip(m)
                .map(|((n, v), m)| (n & !m) | (v & m))
                .collect(),
            _ => new.to_vec(),
        }
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

    /// A value in words: its name, else its bytes (with a mask, only the setting's bits and the mask).
    pub fn show(&self, value: &[u8]) -> String {
        match (self.name_of(value), &self.mask) {
            (Some(n), _) => n.to_string(),
            (None, None) => format!("{} (not one of its values)", hexs(value)),
            (None, Some(m)) => format!(
                "{} in the bits {} (not one of its values)",
                hexs(&self.masked(value)),
                hexs(m)
            ),
        }
    }

    /// The name of a value, when it is one of the setting's (compared on its masked bits).
    pub fn name_of(&self, value: &[u8]) -> Option<&str> {
        self.values
            .iter()
            .find(|(_, v)| self.same(v, value))
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

/// The setting's value now (and its name, when it is one of the setting's values). Reads only.
pub fn read(link: &mut dyn Link, s: &SettingDef) -> Result<(Vec<u8>, Option<String>), String> {
    s.allowed()?;
    let a = send(link, s, &s.read_request())?;
    let v = s
        .value_in(&a)
        .ok_or_else(|| format!("an answer that is not the value: {:02X?}", a.data))?;
    let name = s.name_of(&v).map(str::to_string);
    Ok((v, name))
}

/// One change, as the vehicle's change log keeps it: the identifier's whole value before and after, and the setting's
/// mask when it has one (the bits that changed are only those).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Change {
    pub setting: String,
    pub module: u16,
    pub id: u16,
    pub before: Vec<u8>,
    pub after: Vec<u8>,
    pub mask: Option<Vec<u8>>,
}

impl Change {
    fn line(&self, when: &str) -> String {
        let mut l = format!(
            "{when}\t{}\t{:03X}\t{:04X}\t{}\t{}",
            self.setting,
            self.module,
            self.id,
            hexs(&self.before),
            hexs(&self.after)
        );
        if let Some(m) = &self.mask {
            l += &format!("\t{}", hexs(m));
        }
        l
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

/// The vehicle's change log (`Logs/settings-changes.tsv` in its folder): when, setting, module, identifier, before, after,
/// and the mask of a setting that has one.
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

/// The last change of this setting in the log: the same module and identifier, and the same mask (a line without one is a
/// setting without one), so the last change of another setting packed into the same identifier is not taken for it.
pub fn last_change(log: &Path, s: &SettingDef) -> Option<Change> {
    let text = std::fs::read_to_string(log).ok()?;
    text.lines().rev().find_map(|l| {
        let f: Vec<&str> = l.split('\t').collect();
        let (name, module, id, before, after, mask) = match f.as_slice() {
            [_, name, module, id, before, after] => (name, module, id, before, after, None),
            [_, name, module, id, before, after, mask] => {
                (name, module, id, before, after, Some(unhex(mask)?))
            }
            _ => return None,
        };
        let (module, id) = (
            u16::from_str_radix(module, 16).ok()?,
            u16::from_str_radix(id, 16).ok()?,
        );
        if module != s.module || id != s.id || mask != s.mask {
            return None;
        }
        Some(Change {
            setting: name.to_string(),
            module,
            id,
            before: unhex(before)?,
            after: unhex(after)?,
            mask,
        })
    })
}

/// Whether the setting can be changed now: the battery measured at 12 V or more (an adapter that cannot measure it is
/// refused) and the setting's conditions (checked again by `change`).
pub fn ready(link: &mut dyn Link, s: &SettingDef) -> Result<(), String> {
    s.allowed()?;
    battery_holds(link, MIN_VOLTS, "changing a setting")?;
    conditions_hold(link, &s.conditions)
}

/// Changes the setting to `value` (bytes): conditions and battery first, the value there before read and written to `log`,
/// the new one written and read back (the old one written back if it did not take). With a mask, only the setting's bits
/// are taken from `value`, and the read-back and the put-back look at and touch only them. Returns the change, or None
/// when the setting already had that value (nothing written).
pub fn change(
    link: &mut dyn Link,
    s: &SettingDef,
    value: &[u8],
    log: &Path,
) -> Result<Option<Change>, String> {
    s.allowed()?;
    crate::lock::writes_allowed(link)?;
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
        if s.mask.is_some() && before.len() != len {
            return Err(format!(
                "not changed: the module's value is {} byte(s), and {} covers {len}, so its other bits cannot be kept",
                before.len(),
                s.name
            ));
        }
        // the identifier's whole value as it will be: this setting's bits from `value`, every other bit as it is now
        let target = s.apply(&before, value);
        if target == before {
            return Ok(None);
        }
        let c = Change {
            setting: s.name.clone(),
            module: s.module,
            id: s.id,
            before: before.clone(),
            after: target.clone(),
            mask: s.mask.clone(),
        };
        // the way back is on disk before anything is written
        append(log, &c)
            .map_err(|e| format!("not changed: the change log could not be written ({e})"))?;
        send(link, s, &s.write_request(&target))?;
        let (now, _) = read(link, s)?;
        if !s.same(&now, &target) {
            // only this setting's bits go back: whatever the module did with the others stays as it is
            let back = s.apply(&now, &before);
            let sent = send(link, s, &s.write_request(&back));
            return Err(format!(
                "the module kept {} instead of {}; {}",
                hexs(&now),
                hexs(&target),
                if sent.is_ok() {
                    format!("{} written back", hexs(&back))
                } else {
                    "writing the old value back failed too".into()
                }
            ));
        }
        crate::audit::log(&format!(
            "setting {:?} on {:03X} {:04X}: {} -> {}{}",
            s.name,
            s.module,
            s.id,
            hexs(&before),
            hexs(&target),
            s.mask
                .as_ref()
                .map(|m| format!(" (bits {})", hexs(m)))
                .unwrap_or_default()
        ));
        Ok(Some(c))
    })();
    if session {
        let _ = send(link, s, &[0x10, 0x01]);
    }
    result
}

/// Puts the setting back to what it was before its last change in `log` (with a mask, only its own bits: a setting
/// packed into the same identifier and changed since keeps its value).
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
    if let Some(m) = &s.mask {
        t += &format!(
            "mask = [{}]\n",
            m.iter()
                .map(|b| format!("0x{b:02X}"))
                .collect::<Vec<_>>()
                .join(", ")
        );
    }
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

    /// ICA-102: two settings share the simulated cluster's A002 (the auto door lock in bit 0, the interior light's fade in
    /// bits 2-3; 05 to start: lock on, fade short). Changing the lock, then the fade, then undoing the lock leaves the fade's
    /// bits as they were set at each step, and the log keeps each setting's own way back.
    #[test]
    fn settings_that_share_an_identifier_keep_each_others_bits() {
        crate::lock::switch_for_test(&[]);
        let all = crate::sim::settings();
        let find = |n: &str| all.iter().find(|s| s.name.contains(n)).unwrap().clone();
        let (lock, fade) = (find("auto door lock"), find("interior light"));
        let mut elm = Elm::connect(Box::new(SimCar::engine_off())).unwrap();
        let dir = scratch("packed");
        let log = log_path(&dir);
        let raw = |elm: &mut Elm| read(elm, &lock).unwrap().0;
        assert_eq!(raw(&mut elm), vec![0x05]);
        assert_eq!(read(&mut elm, &lock).unwrap().1.as_deref(), Some("on"));
        assert_eq!(read(&mut elm, &fade).unwrap().1.as_deref(), Some("short"));
        // A: the lock off; the fade's bits stay
        let a = change(&mut elm, &lock, &lock.values["off"], &log)
            .unwrap()
            .unwrap();
        assert_eq!((a.before, a.after), (vec![0x05], vec![0x04]));
        assert_eq!(raw(&mut elm), vec![0x04]);
        assert_eq!(read(&mut elm, &fade).unwrap().1.as_deref(), Some("short"));
        // B: the fade long; the lock's bit stays off
        let b = change(&mut elm, &fade, &fade.values["long"], &log)
            .unwrap()
            .unwrap();
        assert_eq!((b.before, b.after), (vec![0x04], vec![0x08]));
        assert_eq!(read(&mut elm, &lock).unwrap().1.as_deref(), Some("off"));
        // undo A: the lock on again, and the fade stays long (a whole-value undo would have put 05 back: fade short)
        assert_eq!(last_change(&log, &lock).unwrap().after, vec![0x04]);
        assert_eq!(last_change(&log, &fade).unwrap().after, vec![0x08]);
        let u = undo(&mut elm, &lock, &log).unwrap().unwrap();
        assert_eq!((u.before, u.after), (vec![0x08], vec![0x09]));
        assert_eq!(raw(&mut elm), vec![0x09]);
        assert_eq!(read(&mut elm, &lock).unwrap().1.as_deref(), Some("on"));
        assert_eq!(read(&mut elm, &fade).unwrap().1.as_deref(), Some("long"));
        // already so, on its own bits: nothing written
        assert!(change(&mut elm, &fade, &[0x0B], &log).unwrap().is_none());
        // the log line names the mask, and an unmasked setting on the same identifier never takes a masked line
        let text = std::fs::read_to_string(&log).unwrap();
        assert!(
            text.lines()
                .all(|l| l.ends_with("\t01") || l.ends_with("\t0C")),
            "{text}"
        );
        let whole = SettingDef {
            mask: None,
            ..lock.clone()
        };
        assert_eq!(last_change(&log, &whole), None);
        assert_eq!(lock.show(&[0x09]), "on");
        assert_eq!(
            fade.show(&[0x0D]),
            "0C in the bits 0C (not one of its values)"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ICA-102: the read-back and the put-back of a setting with a mask look at and touch only its bits. The simulated
    /// cluster takes a write to A002 but keeps it with some bits flipped: a bit outside the auto door lock's (the module's
    /// own business) leaves the change done, with nothing written back; its own bit as well makes the change fail, and
    /// what is written back is (the value the module kept AND NOT the mask) OR (the value before AND the mask), so the
    /// other bits stay as the module left them.
    #[test]
    fn the_read_back_and_the_put_back_of_a_masked_setting_touch_only_its_bits() {
        crate::lock::switch_for_test(&[]);
        let lock = crate::sim::settings()
            .into_iter()
            .find(|s| s.name.contains("auto door lock"))
            .unwrap();
        assert_eq!(lock.mask, Some(vec![0x01]));
        let dir = scratch("putback");
        let log = log_path(&dir);
        let writes = |sent: Vec<(std::time::Instant, u32, Vec<u8>)>| -> Vec<Vec<u8>> {
            sent.into_iter()
                .map(|(_, _, d)| d)
                .filter(|d| d.first() == Some(&0x2E))
                .collect()
        };
        // bit 1 flipped (no setting's): 05 to 04 is kept as 06, the lock's bit took
        let mut elm =
            Elm::connect(Box::new(SimCar::engine_off().keeping_a002_flipped(0x02))).unwrap();
        elm.take_sent();
        let c = change(&mut elm, &lock, &lock.values["off"], &log)
            .unwrap()
            .unwrap();
        assert_eq!((c.before, c.after), (vec![0x05], vec![0x04]));
        assert_eq!(
            read(&mut elm, &lock).unwrap(),
            (vec![0x06], Some("off".into()))
        );
        assert_eq!(
            writes(elm.take_sent()),
            [vec![0x2E, 0xA0, 0x02, 0x04]],
            "one write, nothing put back"
        );
        // bits 0 and 1 flipped: 05 to 04 is kept as 07, the lock's bit did not take
        let mut elm =
            Elm::connect(Box::new(SimCar::engine_off().keeping_a002_flipped(0x03))).unwrap();
        elm.take_sent();
        let e = change(&mut elm, &lock, &lock.values["off"], &log).unwrap_err();
        // put back: (07 AND NOT 01) OR (05 AND 01) = 07 (a whole-value put-back would have written 05)
        assert!(
            e.contains("kept 07 instead of 04") && e.contains("07 written back"),
            "{e}"
        );
        assert_eq!(
            writes(elm.take_sent()),
            [vec![0x2E, 0xA0, 0x02, 0x04], vec![0x2E, 0xA0, 0x02, 0x07]]
        );
        assert_eq!(
            read(&mut elm, &lock).unwrap(),
            (vec![0x07], Some("on".into()))
        );
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

    /// A link that cannot measure the battery (a SocketCAN interface) is refused before anything is sent, and a
    /// low reading is refused too; with the battery measured above 12 V the same change goes as before.
    #[test]
    fn a_setting_is_changed_only_with_the_battery_measured() {
        use crate::canlink::{CanLink, SimBus};
        crate::lock::switch_for_test(&[]);
        let s = chime();
        let dir = scratch("battery");
        let log = log_path(&dir);
        let on_bus = |volts: Option<f64>| {
            let mut bus = SimBus::new(SimCar::engine_off());
            bus.voltage = volts;
            CanLink::connect(bus, "vcan0").unwrap()
        };
        let mut l = on_bus(None);
        let sent = l.frames().sent.len();
        let e = change(&mut l, &s, &s.values["off"], &log).unwrap_err();
        assert!(
            e.contains("cannot measure the battery") && e.contains("changing a setting"),
            "{e}"
        );
        assert_eq!(l.frames().sent.len(), sent, "nothing was sent");
        assert!(!log.exists(), "nothing was logged as changed");
        let mut l = on_bus(Some(11.2));
        let e = change(&mut l, &s, &s.values["off"], &log).unwrap_err();
        assert!(e.contains("11.2 V") && e.contains("at least 12 V"), "{e}");
        assert!(!log.exists());
        let mut l = on_bus(Some(12.6));
        let c = change(&mut l, &s, &s.values["off"], &log).unwrap().unwrap();
        assert_eq!((c.before, c.after), (vec![0x01], vec![0x00]));
        assert_eq!(read(&mut l, &s).unwrap().1.as_deref(), Some("off"));
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
        assert!(s.refused().unwrap().contains("immobilizer"));
        let mut s = chime();
        s.mask = Some(vec![0x01, 0x00]);
        assert!(s.refused().unwrap().contains("as long as the values"));
        s.mask = Some(vec![0x00]);
        assert!(s.refused().unwrap().contains("no bits"));
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
