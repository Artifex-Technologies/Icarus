//! Vehicle profiles: what Icarus knows about each make, kept as data (one TOML file per make in `crates/icarus-obd/vehicles`,
//! built in) so supporting another vehicle is a file, not code. Files in `~/.config/icarus/vehicles/` add makes or replace a
//! built-in one of the same name, so a module found on a vehicle can be written down without rebuilding Icarus.
//!
//! A profile names the VIN prefixes (world manufacturer identifiers) it covers and where its modules answer. The standard
//! OBD-II range (7E0-7E7) is in every profile.

use serde::Deserialize;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Profile {
    pub name: String,
    #[serde(default)]
    pub wmi: Vec<String>,
    #[serde(default)]
    pub notes: String,
    #[serde(default, rename = "module")]
    pub modules: Vec<ModuleAddr>,
    /// The maker's own trouble codes: code = meaning (`[codes]` table).
    #[serde(default)]
    pub codes: BTreeMap<String, String>,
    /// Actuator tests (`[[test]]` entries; see `actuator`).
    #[serde(default, rename = "test")]
    pub tests: Vec<crate::actuator::TestDef>,
    /// The file's name without `.toml` (how `--make` picks it).
    #[serde(skip)]
    pub id: String,
    /// Built in, or the user's file.
    #[serde(skip)]
    pub source: String,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct ModuleAddr {
    /// The ID requests go to.
    pub tx: u16,
    /// The ID answers come from (tx + 8 unless given).
    #[serde(default)]
    pub rx: Option<u16>,
    pub name: String,
    /// An extended address byte in front of each message (Toyota body modules behind 750, BMW).
    #[serde(default)]
    pub ext: Option<u8>,
    /// "hs-can" (the default, OBD pins 6 and 14) or another bus the adapter has to switch to ("ms-can" on older Fords).
    #[serde(default)]
    pub bus: Option<String>,
    /// True once Icarus has seen the module answer there on a real vehicle.
    #[serde(default)]
    pub checked: bool,
}

impl ModuleAddr {
    pub fn rx(&self) -> u16 {
        self.rx.unwrap_or(self.tx + 8)
    }

    /// Why Icarus cannot reach this module yet, if it cannot.
    pub fn unreachable(&self) -> Option<&'static str> {
        if self.ext.is_some() {
            Some("extended addressing, not supported yet")
        } else if self
            .bus
            .as_deref()
            .is_some_and(|b| !b.eq_ignore_ascii_case("hs-can"))
        {
            Some("on a second CAN bus the adapter has to switch to, not supported yet")
        } else {
            None
        }
    }
}

const BUILTIN: &[(&str, &str)] = &[
    ("generic", include_str!("../vehicles/generic.toml")),
    ("toyota", include_str!("../vehicles/toyota.toml")),
    ("subaru", include_str!("../vehicles/subaru.toml")),
    ("ford", include_str!("../vehicles/ford.toml")),
    ("gm", include_str!("../vehicles/gm.toml")),
    ("stellantis", include_str!("../vehicles/stellantis.toml")),
    ("honda", include_str!("../vehicles/honda.toml")),
    ("nissan", include_str!("../vehicles/nissan.toml")),
    ("hyundai-kia", include_str!("../vehicles/hyundai-kia.toml")),
    ("mazda", include_str!("../vehicles/mazda.toml")),
    ("vw-audi", include_str!("../vehicles/vw-audi.toml")),
    ("bmw", include_str!("../vehicles/bmw.toml")),
    ("mitsubishi", include_str!("../vehicles/mitsubishi.toml")),
];

/// One profile from TOML text, checked: IDs must be 11-bit CAN IDs, VIN prefixes 2-3 characters.
pub fn parse(id: &str, source: &str, text: &str) -> Result<Profile, String> {
    let mut p: Profile = toml::from_str(text).map_err(|e| format!("{source}: {e}"))?;
    for m in &p.modules {
        if m.tx > 0x7FF || m.rx() > 0x7FF {
            return Err(format!(
                "{source}: {} uses {:X}/{:X}, not 11-bit CAN IDs",
                m.name,
                m.tx,
                m.rx()
            ));
        }
    }
    for t in &p.tests {
        if let Some(why) = t.refused() {
            return Err(format!("{source}: test \"{}\" refused: {why}", t.name));
        }
    }
    for w in &p.wmi {
        if !(2..=3).contains(&w.len()) || !w.chars().all(|c| c.is_ascii_alphanumeric()) {
            return Err(format!(
                "{source}: VIN prefix {w:?} should be 2 or 3 letters or digits"
            ));
        }
    }
    p.wmi = p.wmi.iter().map(|w| w.to_uppercase()).collect();
    p.codes = std::mem::take(&mut p.codes)
        .into_iter()
        .map(|(k, v)| (k.trim().to_uppercase(), v))
        .collect();
    p.id = id.to_string();
    p.source = source.to_string();
    Ok(p)
}

pub fn builtin() -> Vec<Profile> {
    BUILTIN
        .iter()
        .map(|(id, text)| {
            parse(id, "built in", text).expect("built-in vehicle profiles parse (tested)")
        })
        .collect()
}

/// `$XDG_CONFIG_HOME/icarus/vehicles` (or `~/.config/...`).
pub fn user_dir() -> Option<PathBuf> {
    std::env::var_os("XDG_CONFIG_HOME")
        .map(PathBuf::from)
        .filter(|p| p.is_absolute())
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))
        .map(|c| c.join("icarus/vehicles"))
}

/// The built-in profiles plus the user's files (a user file replaces the built-in one of the same name). A broken user file
/// is reported and left out; it never stops Icarus.
pub fn load(dir: Option<&Path>) -> (Vec<Profile>, Vec<String>) {
    let mut all = builtin();
    let mut problems = Vec::new();
    let Some(dir) = dir else {
        return (all, problems);
    };
    let Ok(entries) = std::fs::read_dir(dir) else {
        return (all, problems);
    };
    let mut files: Vec<PathBuf> = entries
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "toml"))
        .collect();
    files.sort();
    for f in files {
        let id = f
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("")
            .to_string();
        let parsed = std::fs::read_to_string(&f)
            .map_err(|e| format!("{}: {e}", f.display()))
            .and_then(|t| parse(&id, &f.display().to_string(), &t));
        match parsed {
            Ok(p) => match all.iter_mut().find(|q| q.id == p.id) {
                Some(q) => *q = p,
                None => all.push(p),
            },
            Err(e) => problems.push(e),
        }
    }
    (all, problems)
}

/// The profile for a VIN: the longest matching manufacturer prefix wins (5TF before 5T).
pub fn for_vin<'a>(profiles: &'a [Profile], vin: &str) -> Option<&'a Profile> {
    let v = vin.trim().to_uppercase();
    profiles
        .iter()
        .flat_map(|p| p.wmi.iter().map(move |w| (w, p)))
        .filter(|(w, _)| v.starts_with(w.as_str()))
        .max_by_key(|(w, _)| w.len())
        .map(|(_, p)| p)
}

/// A profile by its file name, else the only one whose name contains `text`.
pub fn find<'a>(profiles: &'a [Profile], text: &str) -> Result<&'a Profile, String> {
    let t = text.to_lowercase();
    if let Some(p) = profiles.iter().find(|p| p.id == t) {
        return Ok(p);
    }
    let hits: Vec<&Profile> = profiles
        .iter()
        .filter(|p| p.name.to_lowercase().contains(&t))
        .collect();
    match hits.len() {
        1 => Ok(hits[0]),
        0 => Err(format!(
            "no vehicle profile matches {text:?} (icarus vehicles lists them)"
        )),
        _ => Err(format!(
            "{text:?} matches {}; use the file name",
            hits.iter()
                .map(|p| p.id.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        )),
    }
}

/// The actuator tests for a vehicle: its profile's, and the simulated vehicle's own when it is the simulation.
pub fn tests_for(profile: Option<&Profile>, vin: Option<&str>) -> Vec<crate::actuator::TestDef> {
    let mut v: Vec<_> = profile.map(|p| p.tests.clone()).unwrap_or_default();
    if vin == Some(crate::sim::VIN) {
        v.extend(crate::sim::tests());
    }
    v
}

/// Where to look: the OBD-II range, then the profile's own modules (a profile entry for an OBD ID renames it).
pub fn candidates(p: &Profile) -> Vec<ModuleAddr> {
    let mut v: Vec<ModuleAddr> = (0..8u16)
        .map(|i| ModuleAddr {
            tx: 0x7E0 + i,
            rx: None,
            name: match i {
                0 => "engine (ECM/PCM)".into(),
                1 => "transmission (TCM)".into(),
                n => format!("OBD module {}", n + 1),
            },
            ext: None,
            bus: None,
            checked: false,
        })
        .collect();
    for m in &p.modules {
        match v
            .iter_mut()
            .find(|c| c.tx == m.tx && c.ext.is_none() && m.ext.is_none())
        {
            Some(c) => *c = m.clone(),
            None => v.push(m.clone()),
        }
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_built_in_profile_parses_and_prefixes_do_not_clash() {
        let all = builtin();
        assert_eq!(all.len(), BUILTIN.len());
        let mut seen = std::collections::HashMap::new();
        for p in &all {
            for w in &p.wmi {
                assert!(
                    seen.insert(w.clone(), p.id.clone()).is_none(),
                    "VIN prefix {w} is in two profiles"
                );
            }
        }
    }

    #[test]
    fn the_vin_picks_the_make() {
        let all = builtin();
        let make = |vin: &str| for_vin(&all, vin).map(|p| p.id.as_str());
        assert_eq!(
            make("3TMCZ5AN0HM000000"),
            Some("toyota"),
            "a Mexico-built Tacoma"
        );
        assert_eq!(make("JF1VA2E60K9800000"), Some("subaru"));
        assert_eq!(make("1FTFW1E50JFA00000"), Some("ford"));
        assert_eq!(make("5NPE34AF0JH000000"), Some("hyundai-kia"));
        assert_eq!(make("5N1AT2MV0JC000000"), Some("nissan"));
        assert_eq!(make("WVWZZZ1KZ6W000000"), Some("vw-audi"));
        assert_eq!(make("ZZZ00000000000000"), None);
        assert_eq!(find(&all, "toyota").unwrap().id, "toyota");
        assert_eq!(find(&all, "Lexus").unwrap().id, "toyota");
        assert!(find(&all, "zeppelin").is_err());
    }

    #[test]
    fn candidates_merge_with_the_obd_range() {
        let all = builtin();
        let t = candidates(find(&all, "toyota").unwrap());
        assert_eq!(t[0].name, "engine (ECM)", "the profile's name for 7E0");
        assert!(t.iter().any(|m| m.tx == 0x7B0));
        let body = t.iter().find(|m| m.ext == Some(0x40)).unwrap();
        assert_eq!(
            (body.tx, body.rx(), body.unreachable().is_some()),
            (0x750, 0x758, true)
        );
        let vw = candidates(find(&all, "vw-audi").unwrap());
        assert_eq!(vw.iter().find(|m| m.tx == 0x713).unwrap().rx(), 0x77D);
        assert_eq!(candidates(find(&all, "generic").unwrap()).len(), 8);
    }

    #[test]
    fn user_files_add_and_replace_and_bad_ones_are_reported() {
        let dir = std::env::temp_dir().join(format!("icarus-vehicles-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("toyota.toml"), "name = \"My Toyota notes\"\nwmi = [\"3TM\"]\n[[module]]\ntx = 0x7B0\nname = \"ABS\"\nchecked = true\n").unwrap();
        std::fs::write(dir.join("kart.toml"), "name = \"Kart\"\nwmi = [\"KRT\"]\n").unwrap();
        std::fs::write(
            dir.join("broken.toml"),
            "name = \"x\"\n[[module]]\ntx = 0x9000\nname = \"too big\"\n",
        )
        .unwrap();
        std::fs::write(dir.join("typo.toml"), "name = \"y\"\nmodul = 1\n").unwrap();
        std::fs::write(dir.join("sneaky.toml"), "name = \"z\"\n[[test]]\nname = \"Seat belt pretensioner\"\nmodule = 0x7E0\nprotocol = \"uds\"\nid = 0x1234\n").unwrap();
        std::fs::write(dir.join("ok-test.toml"), "name = \"w\"\n[[test]]\nname = \"Radiator fan\"\nmodule = 0x7E0\nprotocol = \"uds\"\nid = 0x1234\nconditions = [\"stationary\"]\n").unwrap();
        let (all, problems) = load(Some(&dir));
        assert_eq!(find(&all, "toyota").unwrap().name, "My Toyota notes");
        assert!(find(&all, "kart").is_ok());
        assert_eq!(problems.len(), 3, "{problems:?}");
        assert!(
            problems.iter().any(|p| p.contains("never tested")),
            "an airbag-type test in a user file is refused"
        );
        assert_eq!(
            find(&all, "ok-test").unwrap().tests[0].conditions,
            vec![crate::actuator::Condition::Stationary]
        );
        assert!(problems.iter().any(|p| p.contains("not 11-bit")));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
