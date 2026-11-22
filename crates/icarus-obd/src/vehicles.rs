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
    /// Required for a make of its own; a user file that adds to a built-in profile may leave it out.
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub wmi: Vec<String>,
    #[serde(default)]
    pub notes: String,
    #[serde(default, rename = "module")]
    pub modules: Vec<ModuleAddr>,
    /// Ranges of module request IDs (`[[range]]` entries), asked like modules.
    #[serde(default, rename = "range")]
    pub ranges: Vec<ModuleRange>,
    /// The maker's own trouble codes: code = meaning (`[codes]` table).
    #[serde(default)]
    pub codes: BTreeMap<String, String>,
    /// Actuator tests (`[[test]]` entries; see `actuator`).
    #[serde(default, rename = "test")]
    pub tests: Vec<crate::actuator::TestDef>,
    /// Module settings (`[[setting]]` entries; see `settings`).
    #[serde(default, rename = "setting")]
    pub settings: Vec<crate::settings::SettingDef>,
    /// Service routines (`[[service]]` entries; see `service`).
    #[serde(default, rename = "service")]
    pub services: Vec<crate::service::ServiceDef>,
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
    /// The ID requests go to: an 11-bit CAN ID (up to 7FF), or a 29-bit one written with all eight digits (18DA30F1).
    pub tx: u32,
    /// The ID answers come from: tx + 8 on 11-bit CAN, and on 29-bit CAN 18DAF1xx for a request to 18DAxxF1 (ISO
    /// 15765-4's normal fixed addressing, the tester at F1), unless given.
    #[serde(default)]
    pub rx: Option<u32>,
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

/// ISO 15765-4's 29-bit physical request IDs (normal fixed addressing): 18DA, the module's address, the tester's (F1).
pub const FIXED29: u32 = 0x18DA_0000;

/// Whether `id` is a 29-bit request in ISO 15765-4's normal fixed form, 18DAxxF1 (and not to the tester's own address).
pub fn is_fixed29_request(id: u32) -> bool {
    id & 0xFFFF_00FF == FIXED29 | 0xF1 && (id >> 8) & 0xFF != 0xF1
}

impl ModuleAddr {
    pub fn rx(&self) -> u32 {
        self.rx.unwrap_or(if self.tx > 0x7FF {
            // 18DA<module><tester> is answered from 18DA<tester><module>
            (self.tx & 0xFFFF_0000) | ((self.tx & 0xFF) << 8) | ((self.tx >> 8) & 0xFF)
        } else {
            self.tx + 8
        })
    }

    /// A 29-bit module (its request ID above 7FF).
    pub fn is_29bit(&self) -> bool {
        self.tx > 0x7FF
    }

    /// On Ford's medium-speed CAN (OBD pins 3 and 11), which the adapter has to switch to.
    pub fn on_ms_can(&self) -> bool {
        self.bus
            .as_deref()
            .is_some_and(|b| b.eq_ignore_ascii_case("ms-can"))
    }

    /// Why a link that reaches `r` cannot reach this module, if it cannot: a bus it cannot switch to, or an ID width the
    /// vehicle's CAN does not carry to it.
    pub fn unreachable(&self, r: crate::link::Reach) -> Option<&'static str> {
        let other_bus = self
            .bus
            .as_deref()
            .is_some_and(|b| !b.eq_ignore_ascii_case("hs-can"));
        if other_bus && !self.on_ms_can() {
            Some("on a CAN bus Icarus does not know how to switch to")
        } else if self.on_ms_can() && !r.ms_can {
            Some(
                "on Ford's medium-speed CAN (OBD pins 3 and 11): it needs an adapter that switches to that bus (an \
                 OBDLink, with an STN chip), and this one cannot",
            )
        } else if self.on_ms_can() && self.is_29bit() {
            Some("a 29-bit ID on the medium-speed CAN, where Icarus asks on 11-bit IDs only")
        } else if !self.on_ms_can() && self.is_29bit() && !r.can29 {
            Some("a 29-bit ID, and this vehicle's CAN carries 11-bit IDs to this adapter")
        } else if !self.on_ms_can() && !self.is_29bit() && !r.can11 {
            Some("an 11-bit ID, and this vehicle's CAN carries 29-bit IDs to this adapter (or it is not CAN)")
        } else {
            None
        }
    }
}

/// A range of request IDs a make uses for its modules, each answered a fixed distance away (GM's body modules: requests
/// 240-24F, answers 400 higher). A scan of that make asks each one; they count as unchecked addresses.
#[derive(Clone, Debug, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct ModuleRange {
    /// The first and last request IDs (11-bit).
    pub from: u32,
    pub to: u32,
    /// How far above its request ID each module answers (8 unless given).
    #[serde(default = "eight")]
    pub answer_offset: u32,
    /// What the modules there are ("body module"); each is named this and its ID.
    pub name: String,
}

fn eight() -> u32 {
    8
}

impl ModuleRange {
    /// Each module address the range covers.
    pub fn modules(&self) -> Vec<ModuleAddr> {
        (self.from..=self.to)
            .map(|tx| ModuleAddr {
                tx,
                rx: Some(tx + self.answer_offset),
                name: format!("{} {tx:03X}", self.name),
                ext: None,
                bus: None,
                checked: false,
            })
            .collect()
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

/// Why `tx`/`rx` are not a module's CAN IDs, if they are not: both 11-bit (up to 7FF), or both 29-bit written with all
/// eight digits (01000000-1FFFFFFF: a 29-bit ID's top byte is its priority, 18 in 18DAxxF1, so a shorter number is a
/// mistyped 11-bit ID), and a 29-bit request outside ISO 15765-4's 18DAxxF1 form names its answer ID.
fn ids_refused(m: &ModuleAddr) -> Option<String> {
    let wide = |id: u32| (0x0100_0000..=0x1FFF_FFFF).contains(&id);
    let ok = match (m.tx, m.rx()) {
        (t, r) if t <= 0x7FF && r <= 0x7FF => true,
        (t, r) if wide(t) && wide(r) => m.rx.is_some() || is_fixed29_request(t),
        _ => false,
    };
    (!ok).then(|| {
        format!(
            "{} uses {:X}/{:X}, not 11-bit CAN IDs (up to 7FF) or 29-bit ones (eight digits, such as 18DA30F1; one outside \
             the 18DAxxF1 form gives its rx)",
            m.name,
            m.tx,
            m.rx()
        )
    })
}

/// One profile from TOML text, checked: IDs must be 11-bit or 29-bit CAN IDs (`ids_refused`), a range's 11-bit and at
/// most 64 long, VIN prefixes 2-3 characters.
pub fn parse(id: &str, source: &str, text: &str) -> Result<Profile, String> {
    let mut p: Profile = toml::from_str(text).map_err(|e| format!("{source}: {e}"))?;
    for m in &p.modules {
        if let Some(why) = ids_refused(m) {
            return Err(format!("{source}: {why}"));
        }
    }
    for r in &p.ranges {
        if r.from > r.to || r.to - r.from >= 64 || r.to + r.answer_offset > 0x7FF {
            return Err(format!(
                "{source}: the range \"{}\" ({:X}-{:X}, answers {:X} higher) should be at most 64 11-bit IDs, answered \
                 on 11-bit IDs",
                r.name, r.from, r.to, r.answer_offset
            ));
        }
    }
    for t in &p.tests {
        if let Some(why) = t.refused() {
            return Err(format!("{source}: test \"{}\" refused: {why}", t.name));
        }
    }
    for s in &p.settings {
        if let Some(why) = s.refused() {
            return Err(format!("{source}: setting \"{}\" refused: {why}", s.name));
        }
    }
    for s in &p.services {
        if let Some(why) = s.refused() {
            return Err(format!("{source}: service \"{}\" refused: {why}", s.name));
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
                Some(q) => merge(q, p),
                None if p.name.is_empty() => {
                    problems.push(format!("{}: a new make needs a name", f.display()))
                }
                None => all.push(p),
            },
            Err(e) => problems.push(e),
        }
    }
    (all, problems)
}

/// A user file adds to the built-in profile of the same name: its name and notes (when given) replace, VIN prefixes and
/// codes add, a module at the same address or a test of the same name replaces, others add.
fn merge(base: &mut Profile, user: Profile) {
    if !user.name.is_empty() {
        base.name = user.name;
    }
    if !user.notes.is_empty() {
        base.notes = user.notes;
    }
    for w in user.wmi {
        if !base.wmi.contains(&w) {
            base.wmi.push(w);
        }
    }
    for m in user.modules {
        match base
            .modules
            .iter_mut()
            .find(|b| b.tx == m.tx && b.ext == m.ext && b.on_ms_can() == m.on_ms_can())
        {
            Some(b) => *b = m,
            None => base.modules.push(m),
        }
    }
    for r in user.ranges {
        match base.ranges.iter_mut().find(|b| b.from == r.from) {
            Some(b) => *b = r,
            None => base.ranges.push(r),
        }
    }
    base.codes.extend(user.codes);
    for t in user.tests {
        match base.tests.iter_mut().find(|b| b.name == t.name) {
            Some(b) => *b = t,
            None => base.tests.push(t),
        }
    }
    for t in user.settings {
        match base.settings.iter_mut().find(|b| b.name == t.name) {
            Some(b) => *b = t,
            None => base.settings.push(t),
        }
    }
    base.source = user.source;
}

/// Adds a setting (a `[[setting]]` entry) to the user's file for this make, as `add_test_to_user_file` does a test.
pub fn add_setting_to_user_file(profile_id: &str, entry: &str) -> Result<PathBuf, String> {
    add_test_to_user_file(profile_id, entry)
}

/// Adds a test (a `[[test]]` entry; any entry, checked the same way) to the user's file for this make, creating the file if
/// needed: it then adds to the built-in profile of the same name. The entry is checked before anything is written.
pub fn add_test_to_user_file(profile_id: &str, entry: &str) -> Result<PathBuf, String> {
    if profile_id.is_empty()
        || !profile_id
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '-')
    {
        return Err(format!("bad profile name {profile_id:?}"));
    }
    parse(
        profile_id,
        "the new entry",
        &format!("name = \"check\"\n{entry}"),
    )?;
    let dir = user_dir().ok_or("no config folder")?;
    std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    let p = dir.join(format!("{profile_id}.toml"));
    let mut text = std::fs::read_to_string(&p).unwrap_or_else(|_| {
        format!("# My additions to Icarus's {profile_id} profile (merged into the built-in one).\n")
    });
    if !text.ends_with('\n') {
        text.push('\n');
    }
    text.push('\n');
    text.push_str(entry);
    parse(profile_id, &p.display().to_string(), &text)?;
    std::fs::write(&p, text).map_err(|e| format!("{}: {e}", p.display()))?;
    Ok(p)
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

/// The service routines for a vehicle: its profile's, and the simulated vehicle's own when it is the simulation.
pub fn services_for(
    profile: Option<&Profile>,
    vin: Option<&str>,
) -> Vec<crate::service::ServiceDef> {
    let mut v: Vec<_> = profile.map(|p| p.services.clone()).unwrap_or_default();
    if vin == Some(crate::sim::VIN) {
        v.extend(crate::sim::services());
    }
    v
}

/// The module settings for a vehicle: its profile's, and the simulated vehicle's own when it is the simulation.
pub fn settings_for(
    profile: Option<&Profile>,
    vin: Option<&str>,
) -> Vec<crate::settings::SettingDef> {
    let mut v: Vec<_> = profile.map(|p| p.settings.clone()).unwrap_or_default();
    if vin == Some(crate::sim::VIN) {
        v.extend(crate::sim::settings());
    }
    v
}

/// Why `tx` may not be used as one module's request ID, if it may not: 7DF and IDs outside 700-7F7
/// (`actuator::not_one_module`), and any answer ID the profiles know (7E8-7EF and every module's `rx`, `uds::sweep_skips`):
/// a request sent there would look like that module's own answer. `obd module`, `obd data` and the module-data loader all
/// ask here.
pub fn request_id_refused(tx: u32, profiles: &[Profile]) -> Option<String> {
    if tx > 0x7FF {
        // 29-bit: only ISO 15765-4's physical requests, 18DAxxF1; 18DB33F1 is the broadcast every module hears
        if !is_fixed29_request(tx) {
            return Some(format!(
                "refused: {tx:08X} is not a 29-bit module request ID (18DAxxF1)"
            ));
        }
    } else if let Some(why) = crate::actuator::not_one_module(tx as u16) {
        return Some(why);
    }
    if crate::uds::sweep_skips(profiles).contains(&tx) {
        return Some(format!(
            "refused: {tx:03X} is a module's answer ID; a request sent there would look like that module's own answer"
        ));
    }
    None
}

/// Every module address a profile names: its modules and the ones its ranges cover.
pub fn named(p: &Profile) -> Vec<ModuleAddr> {
    p.modules
        .iter()
        .cloned()
        .chain(p.ranges.iter().flat_map(ModuleRange::modules))
        .collect()
}

/// OBD-II's own modules on 11-bit CAN: 7E0-7E7.
fn obd_range() -> Vec<ModuleAddr> {
    (0..8u32)
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
        .collect()
}

/// Where to look: the OBD-II range (7E0-7E7), then the profile's own modules and ranges (a profile entry for an OBD ID
/// renames it).
pub fn candidates(p: &Profile) -> Vec<ModuleAddr> {
    with_profile(obd_range(), p)
}

/// `candidates` for a vehicle whose OBD-II uses `protocol`: on 29-bit CAN, OBD-II's own modules are the engine and the
/// transmission at their usual 29-bit addresses (18DA10F1 and 18DA18F1, answered from 18DAF110 and 18DAF118) instead of
/// 7E0-7E7.
pub fn candidates_on(p: &Profile, protocol: crate::elm::Protocol) -> Vec<ModuleAddr> {
    if !crate::link::Reach::of(protocol).can29 {
        return candidates(p);
    }
    let obd = [(0x10, "engine (ECM/PCM)"), (0x18, "transmission (TCM)")]
        .into_iter()
        .map(|(a, name): (u32, &str)| ModuleAddr {
            tx: FIXED29 | a << 8 | 0xF1,
            rx: None,
            name: name.into(),
            ext: None,
            bus: None,
            checked: false,
        })
        .collect();
    with_profile(obd, p)
}

fn with_profile(mut v: Vec<ModuleAddr>, p: &Profile) -> Vec<ModuleAddr> {
    for m in named(p) {
        match v.iter_mut().find(|c| {
            c.tx == m.tx && c.ext.is_none() && m.ext.is_none() && c.on_ms_can() == m.on_ms_can()
        }) {
            Some(c) => *c = m,
            None => v.push(m),
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
    fn answer_ids_and_the_broadcast_are_never_request_ids() {
        let all = builtin();
        for id in [0x7DF, 0x7E8, 0x7EF, 0x788, 0x758, 0x6FF, 0x7F8] {
            assert!(request_id_refused(id, &all).is_some(), "{id:03X}");
        }
        for id in [0x7E0, 0x700, 0x701, 0x7B0] {
            assert_eq!(request_id_refused(id, &all), None, "{id:03X}");
        }
    }

    #[test]
    fn candidates_merge_with_the_obd_range() {
        let all = builtin();
        let t = candidates(find(&all, "toyota").unwrap());
        assert_eq!(t[0].name, "engine (ECM)", "the profile's name for 7E0");
        assert!(t.iter().any(|m| m.tx == 0x7B0));
        let body = t.iter().find(|m| m.ext == Some(0x40)).unwrap();
        let hs = crate::link::Reach::of(crate::elm::Protocol::Can11At500);
        assert_eq!(
            (body.tx, body.rx(), body.unreachable(hs).is_some()),
            (0x750, 0x758, false)
        );
        let vw = candidates(find(&all, "vw-audi").unwrap());
        assert_eq!(vw.iter().find(|m| m.tx == 0x713).unwrap().rx(), 0x77D);
        assert_eq!(candidates(find(&all, "generic").unwrap()).len(), 8);
    }

    #[test]
    fn twenty_nine_bit_ids_and_ranges_in_a_profile() {
        let p = |t: &str| parse("x", "a file", &format!("name = \"x\"\n{t}"));
        let m = |t: &str| p(t).map(|p| p.modules[0].clone());
        // 18DAxxF1 answers from 18DAF1xx unless given; other 29-bit forms name their answer
        let honda = m("[[module]]\ntx = 0x18DA30F1\nname = \"EPS\"\n").unwrap();
        assert_eq!((honda.rx(), honda.is_29bit()), (0x18DA_F130, true));
        assert!(m("[[module]]\ntx = 0x17FC0076\nname = \"VW\"\n").is_err());
        let vw = m("[[module]]\ntx = 0x17FC0076\nrx = 0x17FE0076\nname = \"VW\"\n").unwrap();
        assert_eq!(vw.rx(), 0x17FE_0076);
        // a short number above 7FF is a mistyped 11-bit ID, and widths do not mix
        for bad in [
            "tx = 0x9000",
            "tx = 0x7E0\nrx = 0x18DAF110",
            "tx = 0x18DA10F1\nrx = 0x7E8",
            "tx = 0x20000000\nrx = 0x20000001",
        ] {
            let e = m(&format!("[[module]]\n{bad}\nname = \"m\"\n")).unwrap_err();
            assert!(e.contains("not 11-bit CAN IDs"), "{bad}: {e}");
        }
        // a range: each ID asked, answered at its offset, unchecked
        let r = p(
            "[[range]]\nfrom = 0x240\nto = 0x242\nanswer_offset = 0x400\nname = \"body module\"\n",
        )
        .unwrap();
        let c = candidates(&r);
        let body: Vec<(u32, u32, &str, bool)> = c[8..]
            .iter()
            .map(|m| (m.tx, m.rx(), m.name.as_str(), m.checked))
            .collect();
        assert_eq!(
            body,
            vec![
                (0x240, 0x640, "body module 240", false),
                (0x241, 0x641, "body module 241", false),
                (0x242, 0x642, "body module 242", false)
            ]
        );
        assert!(crate::uds::sweep_skips(&[r]).contains(&0x641));
        for bad in [
            "from = 0x700\nto = 0x6FF",
            "from = 0x100\nto = 0x200",
            "from = 0x7F0\nto = 0x7F8",
        ] {
            assert!(
                p(&format!("[[range]]\n{bad}\nname = \"r\"\n")).is_err(),
                "{bad}"
            );
        }
        // what a link reaches decides, and the reason says which way it fails
        use crate::elm::Protocol::*;
        use crate::link::Reach;
        let eleven = m("[[module]]\ntx = 0x7B0\nname = \"ABS\"\n").unwrap();
        assert!(eleven.unreachable(Reach::of(Can11At500)).is_none());
        assert!(honda.unreachable(Reach::of(Can29At500)).is_none());
        assert!(honda
            .unreachable(Reach::of(Can11At500))
            .unwrap()
            .contains("29-bit ID"));
        assert!(eleven
            .unreachable(Reach::of(Can29At500))
            .unwrap()
            .contains("11-bit ID"));
        assert!(
            eleven.unreachable(Reach::of(Iso9141)).is_some(),
            "K-line reaches no module"
        );
        // on 29-bit CAN the OBD-II range is the engine's and the transmission's 29-bit addresses
        let on29 = candidates_on(find(&builtin(), "honda").unwrap(), Can29At500);
        assert_eq!(
            on29.iter().map(|m| (m.tx, m.rx())).collect::<Vec<_>>(),
            vec![(0x18DA_10F1, 0x18DA_F110), (0x18DA_18F1, 0x18DA_F118)]
        );
        // a 29-bit module request ID must be 18DAxxF1 (the broadcast 18DB33F1 and the answers 18DAF1xx are not)
        let all = builtin();
        assert_eq!(request_id_refused(0x18DA_30F1, &all), None);
        for id in [0x18DB_33F1, 0x18DA_F130, 0x18DA_F1F1, 0x17FC_0076] {
            assert!(request_id_refused(id, &all).is_some(), "{id:08X}");
        }
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
        let t = find(&all, "toyota").unwrap();
        assert_eq!(t.name, "My Toyota notes");
        assert!(
            t.wmi.contains(&"JT".to_string()) && t.wmi.contains(&"3TM".to_string()),
            "prefixes add"
        );
        assert!(
            t.modules.iter().any(|m| m.tx == 0x7B0 && m.checked)
                && t.modules.iter().any(|m| m.tx == 0x780),
            "modules merge"
        );
        assert!(t.codes.contains_key("C1241"), "built-in codes stay");
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
