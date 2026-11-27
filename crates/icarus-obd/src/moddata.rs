//! Module data on a vehicle: each module's own live values (a dealer tool's data list), read with read-only requests from
//! definition sets (`icarus_core::moddata`: the format, the importers and the decoder).
//!
//! Where sets come from:
//! - built in (`crates/icarus-obd/data`): only sets Icarus may ship, `source.from = "Icarus"`. Today the simulated vehicle's,
//!   applied only to its VIN.
//! - the user's library, `~/.config/icarus/data/<set>/`: every import (`icarus import obdb`) and hand-written sets. A user
//!   set with a built-in set's name replaces it.
//! - per vehicle, `<vehicle folder>/Definitions/module-data.toml`: which set the vehicle uses and what each request did on it
//!   (answered, refused, too short), and the values the owner checked.
//!
//! **The gates**, checked when a set is loaded and again before every request (a hand-edited file cannot get past them):
//! 1. services 21, 22 and 1A only, built by Icarus from the identifier, and `uds::check_read_only` on the bytes; 1A is kept
//!    for identity (a later build), so no data request uses it yet;
//! 2. the request ID is one module's (`vehicles::request_id_refused`: never 7DF, never 7E8-7EF or another answer ID the
//!    profiles know), not the answer ID of any module in the set, and not its own answer ID;
//! 3. airbag and occupant modules (`actuator::is_airbag`, a source's "airbag" type, or an address any vehicle profile
//!    names as its airbag module): identity only, no data requests; and where the vehicle's make (or the set's, when they
//!    differ: the stricter holds; `Gates::for_vehicle`) has no profile naming its airbag module (GM, Honda, Nissan,
//!    Stellantis, BMW... today), or is not known, a module whose kind is not known (no `kind`, not in the profile) gets
//!    none either: it could be the airbag module;
//!    one request ID is one module: a second module file on the same ID gets no data requests;
//!    a sweep (reads of identifiers no file lists: the as-built block, discovery later) passes `Gates::sweep` besides
//!    (ICA-100): chassis and driver-assist modules (ABS and stability control, brakes, steering, radar, cameras, cruise,
//!    lane and parking assistance; by their kind, their name, or the vehicle profile's name for their address) get
//!    identity only, unless their own module file lists identifiers of the sweep, which are then the only ones read. A
//!    module file's own requests are listed by definition, so module data reads them as before;
//! 4. reachable: 11-bit CAN, no extended address, the main bus;
//! 5. no request that needs a diagnostic session is sent (Icarus opens none);
//! 6. the request's model years include the vehicle's;
//! 7. paced (`Pacer`): at least 50 ms between requests (counted from the end of the last one, and between the resends of
//!    one request too: `uds::read_data` waits through the pacer's clock), each no more often than its `every`, a busy
//!    module slowed down; a refused request (not there, security, a session Icarus does not open: asked once more first,
//!    since a refusal names no identifier and could be a late one to an earlier request), one that keeps failing or a
//!    silent module dropped for the session.
//!
//! A refused or skipped request is always listed with its reason, never left out silently.

use crate::link::Link;
use crate::uds::{self, DataAnswer};
use crate::vehicles::Profile;
use icarus_core::moddata::{
    self as md, capture, units, Ids, ModuleDef, RequestDef, SetInfo, Shown, ValueDef,
};
use serde::Deserialize;
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

pub use icarus_core::moddata::{Imported, ModuleHead};

/// A set as loaded: its modules with every refused request taken out (and listed in `refused`).
#[derive(Clone, Debug)]
pub struct Loaded {
    pub set: SetInfo,
    pub modules: Vec<ModuleDef>,
    /// (what, why) for every request the gates refused, and every file that could not be read.
    pub refused: Vec<(String, String)>,
    /// "built in", or the set's folder.
    pub source: String,
    pub gates: Gates,
}

impl Loaded {
    pub fn module(&self, tx: u32) -> Option<&ModuleDef> {
        self.modules
            .iter()
            .find(|m| m.module.tx == tx && m.module.ext.is_none())
    }

    /// The set as used on a vehicle of make `vehicle` (None: not known): its gates rebuilt for the vehicle
    /// (`Gates::for_vehicle`), and the requests they refuse taken out and listed in `refused`.
    pub fn on_vehicle(&self, profiles: &[Profile], vehicle: Option<&str>) -> Loaded {
        let mut s = self.clone();
        s.gates = Gates::for_vehicle(profiles, &s.modules, &s.set.make, vehicle);
        let refused = gate(&mut s.modules, &s.gates);
        s.refused.extend(refused);
        s
    }
}

/// `$XDG_CONFIG_HOME/icarus/data` (or `~/.config/icarus/data`).
pub fn user_dir() -> Option<PathBuf> {
    crate::vehicles::user_dir().and_then(|v| v.parent().map(|p| p.join("data")))
}

/// The built-in sets: (name, files).
fn builtin() -> Vec<(&'static str, &'static [(&'static str, &'static str)])> {
    vec![("sim", crate::sim::data())]
}

/// Why a module may not be sent data requests at all, if it may not: an airbag or occupant module (identity only), or a
/// request ID that is not one module's.
#[derive(Clone, Debug, Default)]
pub struct Gates {
    /// IDs never sent on: 7DF, every answer ID the profiles know, every answer ID in the set.
    skips: BTreeSet<u32>,
    profiles: Vec<Profile>,
    /// Set when the make's profile names no airbag module (or there is no profile for it): why a module of unknown kind
    /// gets no data request.
    caution: Option<String>,
    /// The request IDs the make's profile names (a module there is of known kind).
    named: BTreeSet<u16>,
    /// Every address a vehicle profile (any make's) names as its airbag or occupant module: (request ID, extended
    /// address), and the answer IDs of those without one. A module there is the airbag module whatever its own file says.
    airbag_tx: BTreeSet<(u16, Option<u8>)>,
    airbag_rx: BTreeSet<u16>,
    /// Addresses (request ID, extended address) the vehicle's profile names as a chassis or driver-assist module (with
    /// the make unknown, every profile's): (the profile's id, the name it gives, what kind of module that is).
    chassis_at: BTreeMap<(u16, Option<u8>), (String, String, &'static str)>,
}

/// What makes a module a chassis or driver-assist one (ICA-100): a word of its kind or name, in upper or lower case, whole
/// or as a part of a word run together (`ParkAssist`, `FrontCamera`). Brakes and stability control (ABS, the makers' names
/// for stability and traction control such as the Toyota profile's VSC and TRC and the Subaru profile's VDC, electric
/// parking brakes, suspension, yaw rate), steering (EPS, the Ford profile's PSCM, the steering angle sensor, the steering
/// column module), and driver assistance (radar, cameras, cruise and adaptive cruise, lane keeping and departure,
/// collision warning and braking, parking aids, blind-spot and cross-traffic warning, around-view cameras), with the
/// abbreviations module names commonly use for them. A word too many only keeps a sweep off a module; it never lets
/// anything be sent, so the list leans to too many.
const CHASSIS: &[(&str, &[&str])] = &[
    (
        "brake or stability control",
        &[
            "abs",
            "atrac",
            "brake",
            "brakes",
            "braking",
            "chassis",
            "dsc",
            "ebcm",
            "ebd",
            "ebs",
            "epb",
            "esc",
            "esp",
            "hdc",
            "kdss",
            "skid",
            "stability",
            "suspension",
            "tcs",
            "traction",
            "trc",
            "vdc",
            "vdim",
            "vsa",
            "vsc",
            "yaw",
        ],
    ),
    (
        "steering",
        &[
            "afs", "ehps", "epas", "eps", "mdps", "pscm", "sas", "sasm", "sccm", "steering",
        ],
    ),
    (
        "driver-assist",
        &[
            "acc",
            "adas",
            "aeb",
            "assist",
            "avm",
            "blind",
            "bsd",
            "bsm",
            "bsw",
            "camera",
            "cmbs",
            "collision",
            "cruise",
            "distronic",
            "drcc",
            "eyesight",
            "fcw",
            "icc",
            "ipma",
            "ipmb",
            "kafas",
            "lane",
            "lda",
            "ldw",
            "lidar",
            "lka",
            "lkas",
            "lta",
            "pam",
            "parking",
            "pcs",
            "pdc",
            "pksb",
            "radar",
            "rcta",
            "scc",
            "sodl",
            "sodr",
            "sonar",
            "spas",
            "ultrasonic",
        ],
    ),
];

/// The words of `text` as `chassis_kind` matches them, lower case: each run of letters and digits whole, and its parts
/// where it changes case or between letters and digits (`ParkAssist`: parkassist, park, assist; `ABSModule`: absmodule,
/// abs, module; `EPS2`: eps2, eps, 2).
fn words_of(text: &str) -> Vec<String> {
    let mut out = Vec::new();
    for token in text
        .split(|c: char| !c.is_alphanumeric())
        .filter(|t| !t.is_empty())
    {
        out.push(token.to_lowercase());
        let c: Vec<char> = token.chars().collect();
        let mut start = 0;
        for i in 1..c.len() {
            let (a, b) = (c[i - 1], c[i]);
            let split = (a.is_lowercase() && b.is_uppercase())
                || (a.is_uppercase()
                    && b.is_uppercase()
                    && c.get(i + 1).is_some_and(|n| n.is_lowercase()))
                || (a.is_ascii_digit() != b.is_ascii_digit());
            if split {
                out.push(c[start..i].iter().collect::<String>().to_lowercase());
                start = i;
            }
        }
        if start > 0 {
            out.push(c[start..].iter().collect::<String>().to_lowercase());
        }
    }
    out
}

/// What kind of chassis or driver-assist module `words` (a module's kind or name) names, if any (`CHASSIS`).
pub fn chassis_kind(words: &str) -> Option<&'static str> {
    let w = words_of(words);
    CHASSIS
        .iter()
        .find(|(_, list)| w.iter().any(|x| list.contains(&x.as_str())))
        .map(|(what, _)| *what)
}

/// The chassis and driver-assist addresses `profiles` name (`Gates::chassis_at`).
fn chassis_named<'p>(
    profiles: impl IntoIterator<Item = &'p Profile>,
) -> BTreeMap<(u16, Option<u8>), (String, String, &'static str)> {
    let mut out = BTreeMap::new();
    for p in profiles {
        for a in &p.modules {
            let (Some(what), Ok(tx)) = (chassis_kind(&a.name), u16::try_from(a.tx)) else {
                continue;
            };
            if tx <= 0x7FF {
                out.entry((tx, a.ext))
                    .or_insert((p.id.clone(), a.name.clone(), what));
            }
        }
    }
    out
}

/// What a sweep may read on one module (ICA-100): a sweep reads identifiers no module-data file lists (the as-built
/// block's DE00-DEFF, and discovery's, ICA-126), so it passes `Gates::sweep` first.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Sweep {
    /// Every identifier the sweep asks.
    Whole,
    /// These only, the ones the module's own file lists: a chassis or driver-assist module.
    Listed(Vec<u16>),
}

/// Airbag and occupant modules: identity only.
pub fn is_airbag(m: &ModuleHead) -> bool {
    let words = format!("{} {}", m.name, m.kind);
    let small = |id: u32| u16::try_from(id).ok().filter(|i| *i <= 0x7FF);
    m.kind.eq_ignore_ascii_case("airbag")
        || crate::actuator::is_airbag(small(m.tx).unwrap_or(0), &words)
        || small(m.rx()).is_some_and(|rx| crate::actuator::is_airbag(rx, ""))
}

impl Gates {
    /// The gates for a set of `make` (a vehicle profile's id; "" when the set names none).
    pub fn new(profiles: &[Profile], modules: &[ModuleDef], make: &str) -> Gates {
        let mut skips = uds::sweep_skips(profiles);
        skips.extend(
            modules
                .iter()
                .filter(|m| m.module.ids == Ids::Can11)
                .map(|m| m.module.rx()),
        );
        let profile = profiles
            .iter()
            .find(|p| !make.is_empty() && p.id.eq_ignore_ascii_case(make));
        // 11-bit IDs only: the ones module data reaches and the airbag list names (a 29-bit module counts by its name)
        let small = |id: u32| u16::try_from(id).ok().filter(|i| *i <= 0x7FF);
        let airbag = |a: &crate::vehicles::ModuleAddr| {
            crate::actuator::is_airbag(small(a.tx).unwrap_or(0), &a.name)
        };
        let caution = match profile {
            None => Some(if make.is_empty() {
                "the set names no make".to_string()
            } else {
                format!("no vehicle profile is named {make:?}")
            }),
            Some(p) if !p.modules.iter().any(airbag) => {
                Some(format!("the {} profile names no airbag module", p.id))
            }
            Some(_) => None,
        };
        let airbags: Vec<&crate::vehicles::ModuleAddr> = profiles
            .iter()
            .flat_map(|p| &p.modules)
            .filter(|a| airbag(a))
            .collect();
        // the chassis names of the make's profile, or with no profile for it every profile's: which one applies is not known
        let chassis_at = match profile {
            Some(p) => chassis_named([p]),
            None => chassis_named(profiles),
        };
        Gates {
            skips,
            chassis_at,
            profiles: profiles.to_vec(),
            airbag_tx: airbags
                .iter()
                .filter_map(|a| small(a.tx).map(|t| (t, a.ext)))
                .collect(),
            airbag_rx: airbags
                .iter()
                .filter(|a| a.ext.is_none())
                .filter_map(|a| small(a.rx()))
                .collect(),
            caution,
            named: profile
                .map(|p| {
                    p.modules
                        .iter()
                        .filter(|a| a.ext.is_none())
                        .filter_map(|a| small(a.tx))
                        .collect()
                })
                .unwrap_or_default(),
        }
    }

    /// The gates for a set of `set_make` used on a vehicle of make `vehicle` (a profile's id; None when not known).
    /// The airbag caution follows the vehicle, not only the set: where the two makes differ, the stricter of their gates
    /// holds (a module of unknown kind gets no request where either make's profile names no airbag module, and counts as
    /// known only where both profiles name it); with the vehicle's make not known, a module of unknown kind gets none.
    /// The chassis and driver-assist addresses (`chassis`) are both makes' too, and every profile's with the vehicle's make
    /// not known.
    pub fn for_vehicle(
        profiles: &[Profile],
        modules: &[ModuleDef],
        set_make: &str,
        vehicle: Option<&str>,
    ) -> Gates {
        let mut g = Gates::new(profiles, modules, set_make);
        match vehicle {
            Some(v) if v.eq_ignore_ascii_case(set_make) => {}
            Some(v) => {
                let other = Gates::new(profiles, modules, v);
                if g.caution.is_none() {
                    g.caution = other.caution.map(|why| {
                        format!("the set is for {set_make:?} but this vehicle is {v:?}, and {why}")
                    });
                }
                g.named = g.named.intersection(&other.named).copied().collect();
                for (k, v) in other.chassis_at {
                    g.chassis_at.entry(k).or_insert(v);
                }
            }
            None => {
                if g.caution.is_none() {
                    g.caution = Some(format!(
                        "this vehicle's make is not known (the set is for {set_make:?}; --make NAME)"
                    ));
                }
                g.named.clear();
                for (k, v) in chassis_named(profiles) {
                    g.chassis_at.entry(k).or_insert(v);
                }
            }
        }
        g
    }

    /// Why `h` counts as a chassis or driver-assist module (ICA-100), if it does: its kind or its name in its own file
    /// (`chassis_kind`), or the name the vehicle's profile gives its address.
    pub fn chassis(&self, h: &ModuleHead) -> Option<String> {
        if let Some(what) = chassis_kind(&h.kind) {
            return Some(format!("a {what} module (its kind is {:?})", h.kind));
        }
        if let Some(what) = chassis_kind(&h.name) {
            return Some(format!("a {what} module (its file names it {:?})", h.name));
        }
        if h.ids != Ids::Can11 {
            return None;
        }
        let tx = u16::try_from(h.tx).ok()?;
        self.chassis_at
            .get(&(tx, h.ext))
            .map(|(p, name, what)| format!("a {what} module (the {p} profile names it {name:?})"))
    }

    /// What a sweep of `service` identifiers in `range` may read on `m` (ICA-100; the module as its module-data file
    /// gives it, or a bare head when it has none), or why it may read nothing:
    /// - the module-data gates first (`module_refused`): never an airbag or occupant module, nor one of unknown kind where
    ///   the make names no airbag module, nor an ID that is not one module's;
    /// - then a chassis or driver-assist module (`chassis`: ABS and stability control, brakes, steering, radar, cameras,
    ///   cruise, lane and parking assistance) gets identity only, unless its own file lists identifiers in the range, which
    ///   are then the only ones read;
    /// - any other module, the whole sweep.
    pub fn sweep(
        &self,
        m: &ModuleDef,
        service: u8,
        range: std::ops::RangeInclusive<u16>,
    ) -> Result<Sweep, String> {
        if let Some(why) = self.module_refused(m) {
            return Err(why);
        }
        let Some(what) = self.chassis(&m.module) else {
            return Ok(Sweep::Whole);
        };
        let mut listed: Vec<u16> = m
            .requests
            .iter()
            .filter(|r| r.service == service && range.contains(&r.ident))
            .map(|r| r.ident)
            .collect();
        listed.sort_unstable();
        listed.dedup();
        if listed.is_empty() {
            return Err(format!(
                "{what}: chassis and driver-assist modules get identity only unless their own module-data file lists the \
                 identifiers to read, and none of {service:02X} {:04X}-{:04X} is listed for this one",
                range.start(),
                range.end()
            ));
        }
        Ok(Sweep::Listed(listed))
    }

    /// Whether `h` is an airbag or occupant module: by its own file (`is_airbag`), or at an address a vehicle profile
    /// names as one.
    pub fn is_airbag(&self, h: &ModuleHead) -> bool {
        if is_airbag(h) {
            return true;
        }
        if h.ids != Ids::Can11 {
            return false;
        }
        let small = |id: u32| u16::try_from(id).ok();
        small(h.tx).is_some_and(|t| self.airbag_tx.contains(&(t, h.ext)))
            || (h.ext.is_none() && small(h.rx()).is_some_and(|r| self.airbag_rx.contains(&r)))
    }

    /// Why no data request may go to this module, if none may.
    pub fn module_refused(&self, m: &ModuleDef) -> Option<String> {
        let h = &m.module;
        if self.is_airbag(h) {
            return Some("airbag module: identity only".into());
        }
        if h.ids != Ids::Can11 {
            return None; // not reachable yet, which `reachable` says; its IDs are not 11-bit request IDs
        }
        let (Ok(tx), Ok(rx)) = (u16::try_from(h.tx), u16::try_from(h.rx())) else {
            return Some("not an 11-bit CAN ID".into());
        };
        if let Some(why) = &self.caution {
            if h.kind.trim().is_empty() && !self.named.contains(&tx) {
                return Some(format!(
                    "refused: what {tx:03X} is, is not known, and {why}, so it could be the airbag module (give it a \
                     kind in its file, e.g. kind = \"body\", once you know what it is)"
                ));
            }
        }
        if tx == rx {
            return Some(format!(
                "refused: {tx:03X} would answer on its own request ID"
            ));
        }
        if let Some(why) = data_request_id_refused(tx.into(), h.ext, &self.profiles) {
            return Some(why);
        }
        if self.skips.contains(&tx.into()) {
            return Some(format!(
                "refused: {tx:03X} is the answer ID of another module in this set"
            ));
        }
        None
    }

    /// Why this request may not be sent, if it may not.
    pub fn request_refused(&self, m: &ModuleDef, r: &RequestDef) -> Option<String> {
        if let Some(why) = self.module_refused(m) {
            return Some(why);
        }
        if !matches!(r.service, 0x21 | 0x22) {
            return Some(if r.service == 0x1A {
                "service 1A is read for a module's identity only (a later build)".into()
            } else {
                format!("service {:02X} is not a data read", r.service)
            });
        }
        if r.service == 0x21 && r.ident > 0xFF {
            return Some("21 takes a one-byte identifier".into());
        }
        uds::check_read_only(&r.bytes())
            .err()
            .map(|e| e.to_string())
    }
}

/// Why a module-data read may not go to `tx` (with the module's extended address), if it may not: as any request ID
/// (`vehicles::request_id_refused`: 700-7F7, not 7DF, not an answer ID the profiles know), and besides those BMW's modules
/// on the tester ID 6F1 with their own address first (not DF or EF, which every module hears) and GM's body modules on
/// 240-24F (answered 400 higher), which bus logs bring in (ICA-60). Reads only: this gate lets nothing but 21 and 22 through,
/// and the tests, settings, services and clearing keep 700-7F7 (`actuator::not_one_module`).
fn data_request_id_refused(tx: u32, ext: Option<u8>, profiles: &[Profile]) -> Option<String> {
    let bmw = tx == capture::BMW_TESTER && ext.is_some();
    match ext {
        Some(e) if bmw && capture::bmw_every_module(e) => {
            return Some(format!(
                "refused: 6F1.{e:02X} is the address every BMW module hears, not one module"
            ))
        }
        _ if bmw || (0x240..=0x24F).contains(&tx) => {}
        _ => return crate::vehicles::request_id_refused(tx, profiles),
    }
    if uds::sweep_skips(profiles).contains(&tx) {
        return Some(format!(
            "refused: {tx:03X} is a module's answer ID; a request sent there would look like that module's own answer"
        ));
    }
    None
}

/// Why Icarus cannot reach this module yet, if it cannot.
pub fn reachable(m: &ModuleDef) -> Result<(), &'static str> {
    let h = &m.module;
    match h.ids {
        Ids::Can29 => return Err("not reachable yet: 29-bit CAN IDs"),
        Ids::Kline => return Err("not reachable yet: K-line"),
        Ids::Can11 => {}
    }
    // behind a gateway: reached when the module answers with its own address byte first (Toyota's 750), not when the
    // answers carry another one (BMW's tester address F1)
    if h.ext.is_some() && h.tester.is_some_and(|t| Some(t) != h.ext) {
        return Err(
            "not reachable yet: a gateway whose answers carry another address byte than the request's",
        );
    }
    if h.bus
        .as_deref()
        .is_some_and(|b| !b.eq_ignore_ascii_case("hs-can"))
    {
        return Err("not reachable yet: on a second CAN bus the adapter has to switch to");
    }
    Ok(())
}

/// Takes every refused request out of `modules` (the module stays, for its identity), returning (what, why) for each.
/// One request ID is one module: a second module on the same request ID (another answer ID) gets no data requests, since
/// a vehicle's state and the pacer know a module by its request ID.
pub fn gate(modules: &mut [ModuleDef], gates: &Gates) -> Vec<(String, String)> {
    let mut out = Vec::new();
    // a request ID one file calls the airbag module's is the airbag module's in every file
    let airbag: BTreeSet<(&str, u32)> = modules
        .iter()
        .filter(|m| m.module.ext.is_none() && gates.is_airbag(&m.module))
        .map(|m| (m.module.ids.word(), m.module.tx))
        .collect();
    let mut first: HashMap<(&str, u32), u32> = HashMap::new();
    for m in modules.iter_mut() {
        let label = m.module.label();
        let reqs = std::mem::take(&mut m.requests);
        let id = (m.module.ids.word(), m.module.tx);
        let whole = if m.module.ext.is_some() || reqs.is_empty() {
            None
        } else if airbag.contains(&id) {
            Some("airbag module: identity only".to_string())
        } else {
            match first.get(&id) {
                Some(rx) => Some(format!(
                    "refused: a second module on request ID {:03X} (answering from {:X}; the first answers from {rx:X}): \
                     one module per request ID",
                    m.module.tx,
                    m.module.rx()
                )),
                None => {
                    first.insert(id, m.module.rx());
                    None
                }
            }
        };
        for r in reqs {
            match whole.clone().or_else(|| gates.request_refused(m, &r)) {
                Some(why) => out.push((format!("{label} {}", r.label()), why)),
                None => m.requests.push(r),
            }
        }
    }
    out
}

/// The gates applied to an import before it is written: refused requests move to `left_out`.
pub fn gate_imported(imp: &mut Imported, profiles: &[Profile]) {
    let make = imp.set.as_ref().map(|s| s.make.clone()).unwrap_or_default();
    let gates = Gates::new(profiles, &imp.modules, &make);
    let refused = gate(&mut imp.modules, &gates);
    imp.left_out.extend(refused);
}

/// A set as read from its files: its information, its modules, and (file, why) for each file that could not be read.
pub type SetFiles = (SetInfo, Vec<ModuleDef>, Vec<(String, String)>);

/// A set from its files: (file name, text). `set.toml` must be there; a module file that cannot be read is listed in the
/// third part and left out.
pub fn read_files(files: &[(String, String)], place: &str) -> Result<SetFiles, String> {
    let set_text = files
        .iter()
        .find(|f| f.0 == "set.toml")
        .ok_or_else(|| format!("{place}: no set.toml"))?;
    let set = md::parse_set(&set_text.1).map_err(|e| format!("{place}: {e}"))?;
    let (mut modules, mut problems) = (Vec::new(), Vec::new());
    for (name, text) in files
        .iter()
        .filter(|f| f.0 != "set.toml" && f.0.ends_with(".toml"))
    {
        match md::parse_module(text, name) {
            Ok(m) => modules.push(m),
            Err(e) => problems.push((format!("{place}/{name}"), e)),
        }
    }
    Ok((set, modules, problems))
}

/// A set's folder read from disk.
pub fn read_dir(dir: &Path) -> Result<SetFiles, String> {
    let mut paths: Vec<PathBuf> = std::fs::read_dir(dir)
        .map_err(|e| format!("{}: {e}", dir.display()))?
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "toml"))
        .collect();
    paths.sort();
    let mut files = Vec::new();
    for p in paths {
        let name = p
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("")
            .to_string();
        let text = std::fs::read_to_string(&p).map_err(|e| format!("{}: {e}", p.display()))?;
        files.push((name, text));
    }
    read_files(&files, &dir.display().to_string())
}

/// A set from its files (`read_files`), gates applied, as `load_set` loads one: for files that are not in a folder (tests).
pub fn load_files(
    files: &[(String, String)],
    place: &str,
    profiles: &[Profile],
) -> Result<Loaded, String> {
    Ok(loaded(read_files(files, place)?, place.into(), profiles))
}

fn loaded(
    (set, mut modules, mut refused): SetFiles,
    source: String,
    profiles: &[Profile],
) -> Loaded {
    let gates = Gates::new(profiles, &modules, &set.make);
    refused.extend(gate(&mut modules, &gates));
    Loaded {
        set,
        modules,
        refused,
        source,
        gates,
    }
}

/// The names of every set: the user's and the built-in ones (a user set replaces a built-in one of its name).
pub fn set_names(user: Option<&Path>) -> Vec<(String, String)> {
    let mut out: Vec<(String, String)> = Vec::new();
    if let Some(dir) = user {
        let mut names: Vec<String> = std::fs::read_dir(dir)
            .into_iter()
            .flatten()
            .flatten()
            .filter(|e| e.path().join("set.toml").is_file())
            .filter_map(|e| e.file_name().to_str().map(str::to_string))
            .collect();
        names.sort();
        out.extend(names.into_iter().map(|n| {
            let p = dir.join(&n).display().to_string();
            (n, p)
        }));
    }
    for (n, _) in builtin() {
        if !out.iter().any(|(m, _)| m == n) {
            out.push((n.to_string(), "built in".into()));
        }
    }
    out
}

/// One set by name, gates applied: the user's folder first, then the built-in sets.
pub fn load_set(name: &str, user: Option<&Path>, profiles: &[Profile]) -> Result<Loaded, String> {
    md::check_set_name(name)?;
    if let Some(dir) = user
        .map(|u| u.join(name))
        .filter(|d| d.join("set.toml").is_file())
    {
        let read = read_dir(&dir)?;
        if read.0.name != name {
            return Err(format!(
                "{}: set.toml names the set {:?}, but its folder is {name:?}",
                dir.display(),
                read.0.name
            ));
        }
        return Ok(loaded(read, dir.display().to_string(), profiles));
    }
    let (_, files) = builtin()
        .into_iter()
        .find(|(n, _)| *n == name)
        .ok_or_else(|| format!("no module-data set named {name:?} (icarus data lists them)"))?;
    let files: Vec<(String, String)> = files
        .iter()
        .map(|(a, b)| (a.to_string(), b.to_string()))
        .collect();
    let read = read_files(&files, "built in")?;
    Ok(loaded(read, "built in".into(), profiles))
}

/// Every set (problems reading one are listed, not fatal).
pub fn load_all(user: Option<&Path>, profiles: &[Profile]) -> (Vec<Loaded>, Vec<String>) {
    let (mut sets, mut problems) = (Vec::new(), Vec::new());
    for (n, _) in set_names(user) {
        match load_set(&n, user, profiles) {
            Ok(s) => sets.push(s),
            Err(e) => problems.push(e),
        }
    }
    (sets, problems)
}

/// The sets that fit a vehicle: its profile's make, its model year and its model. Nothing is guessed: a set that names a
/// make, years or models fits only a vehicle whose make, year or model is known and matches (otherwise it is chosen with
/// `--set`). The simulated vehicle gets only the built-in "sim" set, and no real vehicle gets it.
pub fn sets_for<'a>(
    sets: &'a [Loaded],
    make: Option<&str>,
    year: Option<u16>,
    model: Option<&str>,
    vin: Option<&str>,
) -> Vec<&'a Loaded> {
    let sim = vin == Some(crate::sim::VIN);
    sets.iter()
        .filter(|s| (s.set.name == SIM_SET) == sim)
        .filter(|s| {
            sim || make
                .is_some_and(|m| !s.set.make.is_empty() && s.set.make.eq_ignore_ascii_case(m))
        })
        .filter(|s| s.set.years.is_empty() || year.is_some_and(|y| md::year_in(&s.set.years, y)))
        .filter(|s| {
            s.set.models.is_empty()
                || model.is_some_and(|m| s.set.models.iter().any(|p| model_fits(p, m)))
        })
        .collect()
}

/// The built-in set for the simulated vehicle: never used on any other.
pub const SIM_SET: &str = "sim";

/// Why `set` may not be used on this vehicle even when named with `--set`, if it may not: the simulated vehicle's set
/// (made-up identifiers) on anything but the simulated vehicle's VIN, an unread VIN included.
pub fn set_refused(set: &str, vin: Option<&str>) -> Option<String> {
    (set == SIM_SET && vin != Some(crate::sim::VIN)).then(|| {
        format!(
            "the {SIM_SET:?} set is the simulated vehicle's (made-up identifiers): it is never sent to another vehicle{}",
            if vin.is_none() {
                " (this one's VIN could not be read)"
            } else {
                ""
            }
        )
    })
}

/// Words of a model name, lowercase: letters and digits, everything else a break ("F-150" is "f", "150").
fn model_words(s: &str) -> Vec<String> {
    s.split(|c: char| !c.is_alphanumeric())
        .filter(|w| !w.is_empty())
        .map(str::to_lowercase)
        .collect()
}

/// Whether a set's model phrase ("F 150", "Tacoma") is in the vehicle's model name as whole words in a row ("2017 Ford
/// F-150 XLT" and "F150" fit "F 150"; "Focus" and "F-250" do not).
pub fn model_fits(phrase: &str, model: &str) -> bool {
    let p = model_words(phrase);
    let m = model_words(model);
    if p.is_empty() {
        return false;
    }
    let joined = p.concat();
    m.windows(p.len()).any(|w| w == p.as_slice()) || m.contains(&joined)
}

/// Writes an import as a set in `root` (the user's library): refuses to replace a set that is there unless `replace`.
pub fn write_set(root: &Path, imp: &Imported, replace: bool) -> Result<PathBuf, String> {
    let set = imp
        .set
        .as_ref()
        .ok_or("the import has no set information")?;
    md::check_set(set)?;
    let dir = root.join(&set.name);
    if dir.exists() {
        if !replace {
            return Err(format!(
                "{} is there already: --replace replaces it",
                dir.display()
            ));
        }
        std::fs::remove_dir_all(&dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    }
    std::fs::create_dir_all(&dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    for (name, text) in md::set_files(set, &imp.modules) {
        let p = dir.join(&name);
        std::fs::write(&p, text).map_err(|e| format!("{}: {e}", p.display()))?;
    }
    Ok(dir)
}

// ---------------------------------------------------------------------------------------------------------------------
// Reading

/// One value as read.
#[derive(Clone, Debug, PartialEq)]
pub struct Reading {
    pub id: String,
    pub name: String,
    pub path: String,
    /// The definition's own unit (converted for display by `shown`).
    pub unit: String,
    pub decimals: Option<u8>,
    pub shown: Shown,
    /// The source marks this value not yet verified for the vehicle's model year.
    pub unverified: bool,
    pub at_limit: bool,
}

impl Reading {
    /// The value as text: US units when `us`, then the marks ("at limit", "unverified").
    pub fn text(&self, us: bool) -> String {
        let mut t = match &self.shown {
            Shown::Number(x) => {
                let (v, u) = units::shown(&self.unit, *x, us);
                let n = match self.decimals {
                    Some(d) => format!("{v:.*}", d as usize),
                    // up to two decimals, without trailing zeros
                    None => {
                        let t = format!("{v:.2}");
                        let t = t.trim_end_matches('0').trim_end_matches('.');
                        if t == "-0" {
                            "0".into()
                        } else {
                            t.to_string()
                        }
                    }
                };
                if u.is_empty() {
                    n
                } else {
                    format!("{n} {u}")
                }
            }
            Shown::State(s) | Shown::Text(s) | Shown::Hex(s) => s.clone(),
            Shown::NoReading => "no reading".into(),
        };
        if self.at_limit {
            t += " (at limit)";
        }
        if self.unverified {
            t += " (unverified)";
        }
        t
    }

    /// The value as a number in the units `text` shows, for a CSV column.
    pub fn number(&self, us: bool) -> Option<f64> {
        match self.shown {
            Shown::Number(x) => Some(units::shown(&self.unit, x, us).0),
            _ => None,
        }
    }

    /// The unit `text` shows.
    pub fn shown_unit(&self, us: bool) -> &str {
        units::shown(&self.unit, 0.0, us).1
    }
}

/// What one request did.
#[derive(Clone, Debug, PartialEq)]
pub enum Outcome {
    Values(Vec<Reading>),
    /// `7F <service> <code>`.
    Refused(u8),
    /// No answer.
    Silent,
    /// An answer with fewer data bytes than the definition needs (another variant of the module).
    Short(usize),
    /// Not sent, and why.
    Skipped(String),
    /// Asked too recently (its `every`): not sent this pass.
    NotDue,
    /// The adapter or the answer failed.
    Failed(String),
}

impl Outcome {
    /// In words, for every outcome but values.
    pub fn describe(&self) -> String {
        match self {
            Outcome::Values(v) => format!("{} value(s)", v.len()),
            Outcome::Refused(c) => format!("refused (7F {c:02X}: {})", uds::nrc(*c)),
            Outcome::Silent => "no answer".into(),
            Outcome::Short(n) => {
                format!("answer too short ({n} data bytes): another variant of the module?")
            }
            Outcome::Skipped(why) => format!("not sent: {why}"),
            Outcome::NotDue => "not due yet".into(),
            Outcome::Failed(e) => format!("failed: {e}"),
        }
    }
}

/// What a read covers.
#[derive(Clone, Debug, Default)]
pub struct Scope {
    /// The vehicle's model year, when known.
    pub year: Option<u16>,
    /// Send requests limited to some years even though the year is unknown.
    pub all_years: bool,
    /// Requests this vehicle refused before (from its state): skipped, unless `retry`.
    pub refused_before: BTreeSet<Key>,
    pub retry: bool,
}

/// (module request ID, service, identifier).
pub type Key = (u32, u8, u16);

pub fn key(m: &ModuleDef, r: &RequestDef) -> Key {
    (m.module.tx, r.service, r.ident)
}

/// Whether the source marks `r` unverified for this year (with the year unknown: when it marks any year).
fn unverified(r: &RequestDef, year: Option<u16>) -> bool {
    match year {
        Some(y) => !r.unverified.is_empty() && md::year_in(&r.unverified, y),
        None => !r.unverified.is_empty(),
    }
}

/// One request: every gate checked again, paced, sent, and its answer decoded.
pub fn read_request(
    link: &mut dyn Link,
    pacer: &mut Pacer,
    gates: &Gates,
    m: &ModuleDef,
    r: &RequestDef,
    scope: &Scope,
) -> Outcome {
    match fetch_request(link, pacer, gates, m, r, scope) {
        Ok(d) => decode_all(r, &d, scope.year),
        Err(o) => o,
    }
}

/// `read_request` up to the answer: every gate checked again, paced and sent; the answer's data (after the echo), or the
/// outcome when there is none. Naming unknown values (`naming`) works on the bytes themselves.
pub fn fetch_request(
    link: &mut dyn Link,
    pacer: &mut Pacer,
    gates: &Gates,
    m: &ModuleDef,
    r: &RequestDef,
    scope: &Scope,
) -> Result<Vec<u8>, Outcome> {
    if let Err(why) = reachable(m) {
        return Err(Outcome::Skipped(why.into()));
    }
    if let Some(why) = gates.request_refused(m, r) {
        return Err(Outcome::Skipped(why));
    }
    if r.needs_session.is_some() {
        return Err(Outcome::Skipped(
            "needs a diagnostic session (Icarus does not open one)".into(),
        ));
    }
    if !r.years.is_empty() {
        match scope.year {
            Some(y) if !md::year_in(&r.years, y) => {
                return Err(Outcome::Skipped(format!("not for {y}")))
            }
            None if !scope.all_years => {
                return Err(Outcome::Skipped(
                    "for some model years only, and the year is not known (--year or --all-years)"
                        .into(),
                ))
            }
            _ => {}
        }
    }
    let k = key(m, r);
    if scope.refused_before.contains(&k) && !scope.retry {
        return Err(Outcome::Skipped(
            "this vehicle refused it before (--all asks again)".into(),
        ));
    }
    pacer.before(k, r.every)?;
    // `reachable` and the gates hold both to 11 bits
    let (tx, rx) = (m.module.tx, m.module.rx());
    let nap = pacer.sleep;
    let ext = m.module.ext;
    let ask = |link: &mut dyn Link| match uds::at(link, ext, |l| {
        uds::read_data_with(l, tx, rx, r.service, r.ident, &mut |d| nap(d))
    }) {
        Ok(DataAnswer::Data(d)) => Ok(d),
        Ok(DataAnswer::Refused(c)) => Err(Outcome::Refused(c)),
        Ok(DataAnswer::Silent) => Err(Outcome::Silent),
        Err(e) => Err(Outcome::Failed(e.to_string())),
    };
    let mut out = ask(link);
    // A negative answer names no identifier, so it could be a late refusal of an earlier request: one that would drop
    // this request for the session (or be kept for the vehicle as "not there") is asked once more, paced, and believed
    // only when the module refuses again.
    if matches!(out, Err(Outcome::Refused(c)) if lasting_refusal(c)) {
        pacer.again(k);
        out = ask(link);
    }
    // an answer is paced as any answer with data is, whether or not its values decode
    let data = Outcome::Values(Vec::new());
    pacer.after(
        k,
        match &out {
            Ok(_) => &data,
            Err(o) => o,
        },
    );
    out
}

/// A refusal asking again cannot change in this session: not there (11, 12, 31), security access (33), or a session
/// Icarus does not open (7E, 7F). It drops the request for the session, and 11/12/31 are kept for the vehicle.
pub fn lasting_refusal(code: u8) -> bool {
    matches!(code, 0x11 | 0x12 | 0x31 | 0x33 | 0x7E | 0x7F)
}

/// Every value of a request from its answer's data (after the echo).
pub fn decode_all(r: &RequestDef, data: &[u8], year: Option<u16>) -> Outcome {
    if data.len() < r.min_len.unwrap_or(0) {
        return Outcome::Short(data.len());
    }
    let unv = unverified(r, year);
    let mut out = Vec::new();
    for v in &r.values {
        match md::decode(v, data) {
            Ok(d) => out.push(Reading {
                id: v.id.clone(),
                name: v.name.clone(),
                path: v.path.clone(),
                unit: v.unit.clone(),
                decimals: v.decimals,
                shown: d.shown,
                unverified: unv,
                at_limit: d.at_limit,
            }),
            Err(_) => return Outcome::Short(data.len()),
        }
    }
    Outcome::Values(out)
}

/// Every request of a module whose values pass `only` (all of them when None), in order, each with what it did. Values
/// not passing `only` are left out of the readings; a request none of whose values pass is not sent. The request itself
/// is returned, not its label: one module may hold the same `22 xxxx` several times, one layout for each range of
/// model years, each with its own value ids.
pub fn read_module<'m>(
    link: &mut dyn Link,
    pacer: &mut Pacer,
    gates: &Gates,
    m: &'m ModuleDef,
    scope: &Scope,
    only: Option<&dyn Fn(&ValueDef) -> bool>,
) -> Vec<(&'m RequestDef, Outcome)> {
    let mut out = Vec::new();
    for r in &m.requests {
        let wanted: Vec<&ValueDef> = r
            .values
            .iter()
            .filter(|v| only.is_none_or(|f| f(v)))
            .collect();
        if wanted.is_empty() {
            continue;
        }
        let mut o = read_request(link, pacer, gates, m, r, scope);
        if let Outcome::Values(v) = &mut o {
            v.retain(|x| wanted.iter().any(|w| w.id == x.id));
        }
        out.push((r, o));
    }
    out
}

// ---------------------------------------------------------------------------------------------------------------------
// Pacing

/// The least time between any two data requests.
pub const MIN_GAP: Duration = Duration::from_millis(50);
/// No request is asked more often than this, whatever its `every`.
pub const MIN_EVERY: Duration = Duration::from_millis(250);
/// The most a busy module is slowed to.
pub const MAX_SLOW: Duration = Duration::from_secs(2);

/// How many failed answers in a row (garbled, the adapter's error) drop a request for the session.
pub const MAX_FAILED: u32 = 3;

/// The one rate limit every data request goes through (one per link). The clock is passed in, so tests check the pacing
/// without sleeping; `uds::read_data` waits through the same clock between the sends of one request (see there: at most
/// 8 sends a call, none closer than 50 ms; a lasting refusal is asked once more, after the gap, so 16 at the very most),
/// and the gap to the next request is counted from the end of the last.
pub struct Pacer {
    now: fn() -> Instant,
    sleep: fn(Duration),
    /// Why reading must stop now (Ctrl+C, a recording that owns the adapter), if it must.
    stop: fn() -> Option<String>,
    gap: Duration,
    last: Option<Instant>,
    asked: HashMap<Key, Instant>,
    /// A busy module's own gap, and the clean answers since it was last raised.
    slow: HashMap<u32, (Duration, u32)>,
    last_module: HashMap<u32, Instant>,
    silent: HashMap<u32, u32>,
    failed: HashMap<Key, u32>,
    dropped: HashMap<Key, String>,
    gone: HashMap<u32, String>,
    /// Requests sent so far.
    pub sent: u64,
}

impl Pacer {
    pub fn new(now: fn() -> Instant, sleep: fn(Duration), stop: fn() -> Option<String>) -> Pacer {
        Pacer {
            now,
            sleep,
            stop,
            gap: MIN_GAP,
            last: None,
            asked: HashMap::new(),
            slow: HashMap::new(),
            last_module: HashMap::new(),
            silent: HashMap::new(),
            failed: HashMap::new(),
            dropped: HashMap::new(),
            gone: HashMap::new(),
            sent: 0,
        }
    }

    /// The real clock; the gap raised by `ICARUS_DATA_GAP_MS` (never lowered below 50 ms); stops on Ctrl+C (when
    /// `actuator::catch_interrupt` was called) and when a drive is being recorded.
    pub fn real() -> Pacer {
        fn stop() -> Option<String> {
            if crate::actuator::interrupted() {
                return Some("stopped (Ctrl+C)".into());
            }
            crate::drivelog::recording_now()
                .map(|p| format!("a recording owns the adapter ({})", p.display()))
        }
        let gap = std::env::var("ICARUS_DATA_GAP_MS")
            .ok()
            .and_then(|v| v.parse::<u64>().ok())
            .map(Duration::from_millis)
            .unwrap_or(MIN_GAP);
        Pacer::new(Instant::now, std::thread::sleep, stop).with_gap(gap)
    }

    /// A longer gap between requests (a shorter one than 50 ms is not taken).
    pub fn with_gap(mut self, gap: Duration) -> Pacer {
        self.gap = gap.max(MIN_GAP);
        self
    }

    pub fn gap(&self) -> Duration {
        self.gap
    }

    /// Why reading must stop now, if it must.
    pub fn stopped(&self) -> Option<String> {
        (self.stop)()
    }

    /// Before a request: waits out the gap, or says why it must not go now.
    fn before(&mut self, k: Key, every: Option<f64>) -> Result<(), Outcome> {
        if let Some(why) = (self.stop)() {
            return Err(Outcome::Skipped(why));
        }
        if let Some(why) = self.gone.get(&k.0) {
            return Err(Outcome::Skipped(why.clone()));
        }
        if let Some(why) = self.dropped.get(&k) {
            return Err(Outcome::Skipped(why.clone()));
        }
        let now = (self.now)();
        if let (Some(e), Some(t)) = (every, self.asked.get(&k)) {
            let period = Duration::from_secs_f64(e).max(MIN_EVERY);
            if now.duration_since(*t) < period {
                return Err(Outcome::NotDue);
            }
        }
        let mut until = self.last.map(|l| l + self.gap);
        if let (Some((slow, _)), Some(t)) = (self.slow.get(&k.0), self.last_module.get(&k.0)) {
            let u = *t + *slow;
            until = Some(until.map_or(u, |x| x.max(u)));
        }
        if let Some(u) = until.filter(|u| *u > now) {
            (self.sleep)(u - now);
        }
        let at = (self.now)();
        self.last = Some(at);
        self.last_module.insert(k.0, at);
        self.asked.insert(k, at);
        self.sent += 1;
        Ok(())
    }

    /// Before the same request is asked once more within one read (to confirm a refusal): the gap, and a busy module's
    /// own, waited from the end of the answer; it counts as a request sent.
    fn again(&mut self, k: Key) {
        let mut wait = self.gap;
        if let Some((slow, _)) = self.slow.get(&k.0) {
            wait = wait.max(*slow);
        }
        (self.sleep)(wait);
        let at = (self.now)();
        self.last = Some(at);
        self.last_module.insert(k.0, at);
        self.sent += 1;
    }

    /// After an answer: a busy module is slowed; a refused request, one that failed `MAX_FAILED` times in a row, or a silent
    /// module dropped for the session. The gap to the next request starts now.
    fn after(&mut self, k: Key, o: &Outcome) {
        let at = (self.now)();
        self.last = Some(at);
        self.last_module.insert(k.0, at);
        if !matches!(o, Outcome::Failed(_)) {
            self.failed.remove(&k);
        }
        match o {
            Outcome::Refused(0x21) => {
                let s = self.slow.entry(k.0).or_insert((Duration::ZERO, 0));
                s.0 = (s.0 * 2).max(self.gap * 2).min(MAX_SLOW);
                s.1 = 0;
            }
            Outcome::Refused(c @ (0x11 | 0x12 | 0x31)) => {
                self.dropped.insert(
                    k,
                    format!("refused this session (7F {c:02X}: {})", uds::nrc(*c)),
                );
            }
            // security access or another session: Icarus unlocks nothing and opens no session, so asking again changes
            // nothing
            Outcome::Refused(c @ (0x33 | 0x7E | 0x7F)) => {
                self.dropped.insert(
                    k,
                    format!(
                        "refused this session (7F {c:02X}: {}; Icarus opens no session and unlocks nothing)",
                        uds::nrc(*c)
                    ),
                );
            }
            Outcome::Failed(e) => {
                let n = self.failed.entry(k).or_insert(0);
                *n += 1;
                if *n >= MAX_FAILED {
                    self.dropped.insert(
                        k,
                        format!("failed {MAX_FAILED} times in a row this session (last: {e})"),
                    );
                }
            }
            Outcome::Silent => {
                let n = self.silent.entry(k.0).or_insert(0);
                *n += 1;
                if *n >= 3 {
                    self.gone.insert(
                        k.0,
                        format!("{:03X} stopped answering (ignition off?)", k.0),
                    );
                }
            }
            Outcome::Values(_) | Outcome::Short(_) | Outcome::Refused(_) => {
                self.silent.insert(k.0, 0);
                if let Some(s) = self.slow.get_mut(&k.0) {
                    s.1 += 1;
                    if s.1 >= 10 {
                        s.0 /= 2;
                        s.1 = 0;
                        if s.0 < self.gap {
                            s.0 = Duration::ZERO;
                        }
                    }
                }
            }
            _ => {}
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Per-vehicle state

/// `module-data.toml` in a vehicle's `Definitions` folder: the set it uses, what each request did, the values checked.
#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct State {
    #[serde(default)]
    pub set: Option<String>,
    #[serde(default, rename = "request")]
    pub requests: Vec<RequestState>,
    #[serde(default, rename = "value")]
    pub values: Vec<ValueState>,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct RequestState {
    pub module: u32,
    pub service: u8,
    pub ident: u16,
    /// The first positive answer of the right length.
    #[serde(default)]
    pub answered: Option<String>,
    /// "7F 22 31": not on this vehicle.
    #[serde(default)]
    pub refused: Option<String>,
    /// Answered with this many data bytes, too few.
    #[serde(default)]
    pub short: Option<usize>,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ValueState {
    pub module: u32,
    pub id: String,
    /// When the owner confirmed it reads right.
    pub checked: String,
    #[serde(default)]
    pub note: Option<String>,
}

/// Where a vehicle's state goes: its folder's `Definitions/module-data.toml`; the simulated vehicle's in
/// `~/.local/state/icarus/sim/`. None for a vehicle whose VIN could not be read: its state is kept for the run only, never
/// shared with the simulation's or another unknown vehicle's.
pub fn state_path(vin: Option<&str>) -> Result<Option<PathBuf>, String> {
    match vin {
        None => Ok(None),
        Some(v) if v == crate::sim::VIN => crate::audit::path()
            .and_then(|p| p.parent().map(|d| d.join("sim/module-data.toml")))
            .map(Some)
            .ok_or("no state folder".into()),
        Some(v) => Ok(Some(
            crate::garage::ensure(v)
                .map_err(|x| format!("the vehicle's folder could not be made: {x}"))?
                .join("Definitions/module-data.toml"),
        )),
    }
}

impl State {
    pub fn load(path: &Path) -> Result<State, String> {
        match std::fs::read_to_string(path) {
            Ok(t) => toml::from_str(&t).map_err(|e| format!("{}: {e}", path.display())),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(State::default()),
            Err(e) => Err(format!("{}: {e}", path.display())),
        }
    }

    pub fn save(&self, path: &Path) -> Result<(), String> {
        if let Some(d) = path.parent() {
            std::fs::create_dir_all(d).map_err(|e| format!("{}: {e}", d.display()))?;
        }
        std::fs::write(path, self.to_toml()).map_err(|e| format!("{}: {e}", path.display()))
    }

    pub fn to_toml(&self) -> String {
        let q = |s: &str| toml::Value::String(s.to_string()).to_string();
        let mut o = String::from(
            "# Icarus: what each module-data request did on this vehicle, and the values checked on it.\n",
        );
        if let Some(s) = &self.set {
            o += &format!("set = {}\n", q(s));
        }
        for r in &self.requests {
            o += &format!(
                "\n[[request]]\nmodule = 0x{:03X}\nservice = 0x{:02X}\nident = 0x{:0w$X}\n",
                r.module,
                r.service,
                r.ident,
                w = if r.service == 0x22 { 4 } else { 2 }
            );
            if let Some(a) = &r.answered {
                o += &format!("answered = {}\n", q(a));
            }
            if let Some(a) = &r.refused {
                o += &format!("refused = {}\n", q(a));
            }
            if let Some(n) = r.short {
                o += &format!("short = {n}\n");
            }
        }
        for v in &self.values {
            o += &format!(
                "\n[[value]]\nmodule = 0x{:03X}\nid = {}\nchecked = {}\n",
                v.module,
                q(&v.id),
                q(&v.checked)
            );
            if let Some(n) = &v.note {
                o += &format!("note = {}\n", q(n));
            }
        }
        o
    }

    fn entry(&mut self, k: Key) -> &mut RequestState {
        let i = match self
            .requests
            .iter()
            .position(|r| (r.module, r.service, r.ident) == k)
        {
            Some(i) => i,
            None => {
                self.requests.push(RequestState {
                    module: k.0,
                    service: k.1,
                    ident: k.2,
                    ..Default::default()
                });
                self.requests.len() - 1
            }
        };
        &mut self.requests[i]
    }

    /// Notes what a request did (the first right-length answer, a refusal that says "not here", a short answer). True when
    /// something changed.
    pub fn record(&mut self, k: Key, o: &Outcome, today: &str) -> bool {
        let before = self.clone();
        match o {
            Outcome::Values(_) => {
                let e = self.entry(k);
                if e.answered.is_none() {
                    e.answered = Some(today.to_string());
                }
                (e.refused, e.short) = (None, None);
            }
            Outcome::Refused(c @ (0x11 | 0x12 | 0x31)) => {
                self.entry(k).refused = Some(format!("7F {:02X} {c:02X}", k.1));
            }
            Outcome::Short(n) => self.entry(k).short = Some(*n),
            _ => {}
        }
        *self != before
    }

    /// The requests this vehicle refused as not there.
    pub fn refused_keys(&self) -> BTreeSet<Key> {
        self.requests
            .iter()
            .filter(|r| r.refused.is_some())
            .map(|r| (r.module, r.service, r.ident))
            .collect()
    }

    /// Marks a value as checked by the owner.
    pub fn check(&mut self, module: u32, id: &str, note: Option<&str>, today: &str) {
        self.values.retain(|v| !(v.module == module && v.id == id));
        self.values.push(ValueState {
            module,
            id: id.to_string(),
            checked: today.to_string(),
            note: note.map(str::to_string),
        });
    }

    pub fn checked(&self, module: u32, id: &str) -> Option<&ValueState> {
        self.values
            .iter()
            .find(|v| v.module == module && v.id == id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elm::Elm;
    use crate::sim::SimCar;
    use std::cell::Cell;

    thread_local! {
        static CLOCK: Cell<Duration> = const { Cell::new(Duration::ZERO) };
        static NAPS: Cell<Duration> = const { Cell::new(Duration::ZERO) };
    }
    fn base() -> Instant {
        static B: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
        *B.get_or_init(Instant::now)
    }
    fn fake_now() -> Instant {
        base() + CLOCK.with(|c| c.get())
    }
    fn fake_sleep(d: Duration) {
        CLOCK.with(|c| c.set(c.get() + d));
        NAPS.with(|n| n.set(n.get() + d));
    }
    fn advance(d: Duration) {
        CLOCK.with(|c| c.set(c.get() + d));
    }
    fn never() -> Option<String> {
        None
    }
    fn pacer() -> Pacer {
        Pacer::new(fake_now, fake_sleep, never)
    }

    fn sim_set() -> Loaded {
        load_set("sim", None, &crate::vehicles::builtin()).unwrap()
    }

    fn sim_scope() -> Scope {
        let year = crate::vin::decode(crate::sim::VIN).unwrap().year;
        assert_eq!(year, Some(2025), "the simulated VIN's model year");
        Scope {
            year,
            ..Default::default()
        }
    }

    fn values(o: &Outcome) -> &[Reading] {
        match o {
            Outcome::Values(v) => v,
            other => panic!("{other:?}"),
        }
    }

    fn find<'a>(v: &'a [Reading], id: &str) -> &'a Reading {
        v.iter().find(|r| r.id == id).unwrap()
    }

    #[test]
    fn every_built_in_set_is_icarus_own_and_loads_with_no_refusals() {
        for (n, _) in builtin() {
            let s = load_set(n, None, &crate::vehicles::builtin()).unwrap();
            assert_eq!(
                s.set.source.from, "Icarus",
                "{n}: built-in sets are Icarus's own facts only"
            );
            assert!(s.refused.is_empty(), "{n}: {:?}", s.refused);
            assert!(!s.modules.is_empty());
        }
    }

    #[test]
    fn reading_the_simulated_engine_and_transmission() {
        let set = sim_set();
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        let mut p = pacer();
        let scope = sim_scope();
        let engine = set.module(0x700).unwrap();
        let got = read_module(&mut elm, &mut p, &set.gates, engine, &scope, None);
        let labels: Vec<String> = got.iter().map(|g| g.0.label()).collect();
        assert_eq!(
            labels,
            vec!["22 1F01", "22 1F02", "22 1F03", "22 1F04", "22 1F05", "22 1F06", "22 1F07"]
        );
        // a 20-byte answer over several frames
        let v = values(&got[0].1);
        assert_eq!(find(v, "SIM_ECT").shown, Shown::Number(90.0));
        assert_eq!(find(v, "SIM_ECT").text(true), "194 °F");
        assert_eq!(find(v, "SIM_ECT").text(false), "90 °C");
        assert_eq!(find(v, "SIM_RPM").shown, Shown::Number(700.0));
        assert_eq!(find(v, "SIM_KNOCK").text(true), "-2.0 °", "signed");
        assert_eq!(find(v, "SIM_BATT").text(true), "14.1 V");
        assert_eq!(find(v, "SIM_VSS").text(true), "0 mph");
        assert_eq!(find(v, "SIM_TAG").shown, Shown::Text("ICARUS".into()));
        // "response pending" first, then the answer
        let trims = values(&got[1].1);
        assert_eq!(find(trims, "SIM_STFT").text(true), "2.3 %");
        assert_eq!(find(trims, "SIM_LTFT").text(true), "-10.2 %");
        assert_eq!(got[2].1, Outcome::Refused(0x31));
        assert_eq!(got[3].1, Outcome::Skipped("not for 2025".into()));
        assert!(matches!(&got[4].1, Outcome::Skipped(w) if w.contains("diagnostic session")));
        // the transmission: unverified for this year by its source
        let t = read_module(
            &mut elm,
            &mut p,
            &set.gates,
            set.module(0x701).unwrap(),
            &scope,
            None,
        );
        let v = values(&t[0].1);
        assert_eq!(find(v, "SIM_ATF").text(true), "179.6 °F (unverified)");
        assert_eq!(find(v, "SIM_LOCKUP").shown, Shown::State("on".into()));
        assert_eq!(find(v, "SIM_GEAR").shown, Shown::Number(3.0));
        // a local identifier (21) on 7E0, and `only` choosing one value
        let only = |v: &ValueDef| v.id == "SIM_ISC";
        let k = read_module(
            &mut elm,
            &mut p,
            &set.gates,
            set.module(0x7E0).unwrap(),
            &scope,
            Some(&only),
        );
        assert_eq!(
            values(&k[0].1),
            &[Reading {
                id: "SIM_ISC".into(),
                name: "idle speed control".into(),
                path: "Engine.Idle".into(),
                unit: "%".into(),
                decimals: None,
                shown: Shown::Number(30.0),
                unverified: false,
                at_limit: false,
            }]
        );
        // the refused request is dropped for the session
        let again = read_module(&mut elm, &mut p, &set.gates, engine, &scope, None);
        assert!(matches!(&again[2].1, Outcome::Skipped(w) if w.contains("refused this session")));
        // each gap between sends is checked in `every_send_is_at_least_the_gap_apart`
    }

    #[test]
    fn a_definition_on_the_broadcast_or_an_answer_id_sends_nothing() {
        use crate::sim::recording::Recorder;
        let profiles = crate::vehicles::builtin();
        let set = sim_set();
        for tx in ["0x7DF", "0x7E8", "0x708", "0x7B8"] {
            let m = module(tx, "");
            let gates = Gates::new(&profiles, &set.modules, "toyota");
            let (port, log) = Recorder::new(SimCar::new());
            let mut elm = Elm::connect(Box::new(port)).unwrap();
            log.lock().unwrap().clear();
            let o = read_request(
                &mut elm,
                &mut pacer(),
                &gates,
                &m,
                &m.requests[0],
                &Scope::default(),
            );
            assert!(
                matches!(&o, Outcome::Skipped(w) if w.starts_with("refused")),
                "{tx}: {o:?}"
            );
            assert!(
                sent(&log.lock().unwrap()).is_empty(),
                "{tx}: nothing was sent"
            );
        }
    }

    /// The (header, request) pairs sent, from a log of what went to the adapter.
    fn sent(log: &[String]) -> Vec<(u16, String)> {
        let (mut header, mut out) = (0x7DF, Vec::new());
        for l in log {
            let c = l.trim_end_matches('\r');
            if let Some(h) = c.strip_prefix("ATSH") {
                header = u16::from_str_radix(h, 16).unwrap();
            } else if !c.is_empty() && c.chars().all(|ch| ch.is_ascii_hexdigit()) {
                out.push((header, c.to_string()));
            }
        }
        out
    }

    /// A module file written for the gate tests (of kind "body" unless `extra` gives one).
    fn module(tx: &str, extra: &str) -> ModuleDef {
        let kind = if extra.contains("kind =") {
            ""
        } else {
            "kind = \"body\""
        };
        md::parse_module(
            &format!(
                "format = 1\n[module]\nname = \"test module\"\ntx = {tx}\n{kind}\n{extra}\n[[request]]\nservice = 0x22\nident = 0x1F21\n\
                 [[request.value]]\nid = \"X\"\nname = \"x\"\nbyte = 0\nbits = 8\n"
            ),
            "test",
        )
        .unwrap()
    }

    #[test]
    fn the_airbag_module_gets_no_data_request() {
        use crate::sim::recording::Recorder;
        let profiles = crate::vehicles::builtin();
        for (tx, extra) in [
            ("0x780", ""),
            ("0x7A0", "kind = \"airbag\""),
            ("0x7A0", "rx = 0x788"),
            ("0x7A1", ""),
        ] {
            let mut ms = vec![module(tx, extra)];
            if tx == "0x7A1" {
                ms[0].module.name = "occupant classification".into();
            }
            let gates = Gates::new(&profiles, &ms, "toyota");
            let refused = gate(&mut ms, &gates);
            assert_eq!(refused.len(), 1, "{tx} {extra}");
            assert_eq!(refused[0].1, "airbag module: identity only");
            assert!(
                ms[0].requests.is_empty(),
                "kept for its identity, without data requests"
            );
        }
        // and at send time, for a definition that never went through the loader
        let m = module("0x780", "");
        let gates = Gates::new(&profiles, std::slice::from_ref(&m), "toyota");
        let (port, log) = Recorder::new(SimCar::new());
        let mut elm = Elm::connect(Box::new(port)).unwrap();
        log.lock().unwrap().clear();
        let o = read_request(
            &mut elm,
            &mut pacer(),
            &gates,
            &m,
            &m.requests[0],
            &Scope::default(),
        );
        assert_eq!(o, Outcome::Skipped("airbag module: identity only".into()));
        assert!(sent(&log.lock().unwrap()).is_empty(), "nothing was sent");
    }

    /// ICA-100: a chassis or driver-assist module, by its kind, its name, or the vehicle profile's name for its address
    /// (the stricter of the file and the profile), gets nothing from a sweep unless its own file lists the sweep's
    /// identifiers, which are then the only ones; its own requests stay module data's to read, and any other module gets
    /// the whole sweep. With the make not known every profile's names count.
    #[test]
    fn chassis_and_driver_assist_modules_are_swept_only_by_what_their_file_lists() {
        let profiles = crate::vehicles::builtin();
        let range = || 0xDE00..=0xDEFF;
        for (tx, extra, name, words) in [
            ("0x7D4", "kind = \"radar\"", None, "\"radar\""),
            ("0x7D4", "kind = \"ADAS\"", None, "\"ADAS\""),
            ("0x7D4", "", Some("front camera"), "\"front camera\""),
            ("0x7D4", "", Some("EPS (steering)"), "steering"),
            ("0x7B0", "", None, "\"ABS / VSC / TRC (skid control)\""),
            ("0x7A1", "", None, "\"electric power steering\""),
        ] {
            let mut ms = vec![module(tx, extra)];
            if let Some(n) = name {
                ms[0].module.name = n.into();
            }
            let gates = Gates::new(&profiles, &ms, "toyota");
            let e = gates.sweep(&ms[0], 0x22, range()).unwrap_err();
            assert!(
                e.contains("chassis and driver-assist")
                    && e.contains(words)
                    && e.contains("22 DE00-DEFF"),
                "{tx} {extra}: {e}"
            );
            assert!(gates.chassis(&ms[0].module).is_some());
            // its own file's requests are listed, so module data reads them as before
            assert!(gate(&mut ms, &gates).is_empty(), "{tx} {extra}");
        }
        // the identifiers its file lists, and only those, in order, each once (a file may list one for several years)
        let mut abs = module("0x7B0", "");
        for ident in [0xDE05, 0xDE01, 0xDE05] {
            let mut r = abs.requests[0].clone();
            r.ident = ident;
            abs.requests.push(r);
        }
        let gates = Gates::new(&profiles, std::slice::from_ref(&abs), "toyota");
        assert_eq!(
            gates.sweep(&abs, 0x22, range()),
            Ok(Sweep::Listed(vec![0xDE01, 0xDE05]))
        );
        // not a chassis module: the whole sweep; an airbag module: still nothing
        let cluster = module("0x7C0", "");
        assert_eq!(gates.sweep(&cluster, 0x22, range()), Ok(Sweep::Whole));
        assert!(gates
            .sweep(&module("0x780", ""), 0x22, range())
            .unwrap_err()
            .contains("airbag"));
        // the vehicle's profile decides: on a Ford, 7B0 named nothing there is a body module by its file, and Ford's ABS
        // (760) and parking aid (736) are chassis and driver-assist modules
        let ford = |tx: &str| {
            let m = module(tx, "");
            let g = Gates::for_vehicle(&profiles, std::slice::from_ref(&m), "ford", Some("ford"));
            g.sweep(&m, 0x22, range())
        };
        assert_eq!(ford("0x7B0"), Ok(Sweep::Whole));
        assert!(ford("0x760").unwrap_err().contains("\"ABS\""));
        assert!(ford("0x736")
            .unwrap_err()
            .contains("driver-assist module (the ford profile"));
        // with the make not known, any profile naming the address a chassis module's counts
        let m = module("0x7B0", "");
        let g = Gates::for_vehicle(&profiles, std::slice::from_ref(&m), "ford", None);
        assert!(g.sweep(&m, 0x22, range()).is_err());
        assert_eq!(chassis_kind("instrument cluster (combination meter)"), None);
        assert_eq!(chassis_kind("air conditioning"), None);
        assert_eq!(chassis_kind("tire pressure"), None);
        // the abbreviations module names use for them, and words run together, count as their words do
        for (name, what) in [
            ("MDPS", "steering"),
            ("SCC (7D0)", "driver-assist"),
            ("ACC", "driver-assist"),
            ("SAS", "steering"),
            ("yaw rate sensor", "brake or stability control"),
            ("AVM", "driver-assist"),
            ("SPAS", "driver-assist"),
            ("EyeSight", "driver-assist"),
            ("PCS", "driver-assist"),
            ("FCW", "driver-assist"),
            ("AEB", "driver-assist"),
            ("LDW", "driver-assist"),
            ("RCTA", "driver-assist"),
            ("IPMA (image processing)", "driver-assist"),
            ("SCCM", "steering"),
            ("ParkAssist", "driver-assist"),
            ("FrontCamera", "driver-assist"),
            ("BlindSpot", "driver-assist"),
            ("ABSModule", "brake or stability control"),
            ("EPS2", "steering"),
        ] {
            assert_eq!(chassis_kind(name), Some(what), "{name}");
        }
        for name in [
            "BodyControl",
            "keyless access",
            "gateway (GWM)",
            "ACM (audio)",
            "HVAC",
            "main body",
        ] {
            assert_eq!(chassis_kind(name), None, "{name}");
        }
    }

    /// ICA-100: a set made for one make, used on a vehicle of another, still keeps a sweep off the address the vehicle's own
    /// profile names a chassis module, though the set's make names nothing there: a Ford set on the Toyota at 7B0 (the
    /// Toyota profile's ABS), and a Toyota set on a Ford at 760 (the Ford profile's ABS).
    #[test]
    fn a_set_for_another_make_keeps_the_vehicles_chassis_names() {
        let profiles = crate::vehicles::builtin();
        let range = || 0xDE00..=0xDEFF;
        for (tx, set_make, vehicle, named) in [
            (
                "0x7B0",
                "ford",
                "toyota",
                "the toyota profile names it \"ABS / VSC / TRC",
            ),
            (
                "0x760",
                "toyota",
                "ford",
                "the ford profile names it \"ABS\"",
            ),
        ] {
            let m = module(tx, "");
            let alone = Gates::for_vehicle(
                &profiles,
                std::slice::from_ref(&m),
                set_make,
                Some(set_make),
            );
            assert_eq!(
                alone.sweep(&m, 0x22, range()),
                Ok(Sweep::Whole),
                "{tx} on a {set_make}"
            );
            let g =
                Gates::for_vehicle(&profiles, std::slice::from_ref(&m), set_make, Some(vehicle));
            let e = g.sweep(&m, 0x22, range()).unwrap_err();
            assert!(
                e.contains(named),
                "{tx}, a {set_make} set on a {vehicle}: {e}"
            );
        }
    }

    #[test]
    fn never_on_the_broadcast_or_an_answer_id() {
        let profiles = crate::vehicles::builtin();
        let set = sim_set();
        for (tx, why) in [
            ("0x7DF", "broadcast"),
            ("0x7E8", "answer ID"),
            ("0x708", "answer ID"),
            ("0x6F0", "not a module request ID"),
        ] {
            let m = module(tx, "");
            let mut all = set.modules.clone();
            all.push(m.clone());
            let gates = Gates::new(&profiles, &all, "toyota");
            let w = gates.module_refused(&m).unwrap_or_else(|| panic!("{tx}"));
            assert!(w.contains(why), "{tx}: {w}");
        }
        // 708 is refused because the set's engine module answers from it, even with no profile naming it
        let gates = Gates::new(&[], &set.modules, "");
        assert!(gates
            .module_refused(&module("0x708", ""))
            .unwrap()
            .contains("another module in this set"));
        assert!(gates
            .module_refused(&module("0x7A0", "rx = 0x7A0"))
            .unwrap()
            .contains("own request ID"));
        assert_eq!(gates.module_refused(&module("0x7A0", "")), None);
    }

    #[test]
    fn module_data_through_the_gateway() {
        // OBDb's Tacoma layout: the tire pressure module at 750 with address byte 2A, answering with 2A first
        let m = md::parse_module(
            "format = 1\n[module]\nname = \"tire pressure\"\ntx = 0x750\nrx = 0x758\next = 0x2A\ntester = 0x2A\nkind = \"tpms\"\n\
             [[request]]\nservice = 0x22\nident = 0x2021\n\
             [[request.value]]\nid = \"TID_1\"\nname = \"Tire 1 position\"\nbyte = 0\nbits = 8\n",
            "t",
        )
        .unwrap();
        assert!(reachable(&m).is_ok());
        let profiles = crate::vehicles::builtin();
        let gates = Gates::new(&profiles, std::slice::from_ref(&m), "toyota");
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        let o = read_request(
            &mut elm,
            &mut pacer(),
            &gates,
            &m,
            &m.requests[0],
            &Scope::default(),
        );
        let Outcome::Values(v) = o else {
            panic!("{o:?}")
        };
        assert_eq!(v[0].shown, Shown::Number(1.0));
        // normal addressing afterwards: the engine's own data still reads
        assert!(crate::uds::module_dtcs(&mut elm, 0x7E0, 0x7E8)
            .unwrap()
            .is_ok());
    }

    #[test]
    fn only_reading_services_and_reachable_modules() {
        let gates = Gates::new(&[], &[], "");
        let mut m = module("0x7A0", "");
        m.requests[0].service = 0x2E;
        assert!(gates
            .request_refused(&m, &m.requests[0])
            .unwrap()
            .contains("not a data read"));
        m.requests[0].service = 0x1A;
        m.requests[0].ident = 0x80;
        assert!(gates
            .request_refused(&m, &m.requests[0])
            .unwrap()
            .contains("identity only"));
        for (extra, why) in [
            ("ext = 0x40\ntester = 0xF1", "another address byte"),
            ("bus = \"ms-can\"", "second CAN bus"),
            ("ids = \"kline\"", "K-line"),
        ] {
            let m = module("0x750", extra);
            assert!(reachable(&m).unwrap_err().contains(why), "{extra}");
            let o = read_request(
                &mut Elm::connect(Box::new(SimCar::new())).unwrap(),
                &mut pacer(),
                &gates,
                &m,
                &m.requests[0],
                &Scope::default(),
            );
            assert!(
                matches!(&o, Outcome::Skipped(w) if w.starts_with("not reachable yet")),
                "{o:?}"
            );
        }
        let can29 = md::parse_module(
            "format = 1\n[module]\nname = \"x\"\ntx = 0x18DA10F1\nrx = 0x18DAF110\nids = \"can29\"\n",
            "t",
        )
        .unwrap();
        assert!(reachable(&can29).unwrap_err().contains("29-bit"));
    }

    #[test]
    fn pacing_with_a_fake_clock() {
        let mut p = pacer();
        let k = (0x700, 0x22, 0x1F01);
        let start = fake_now();
        p.before(k, Some(1.0)).unwrap();
        p.after(k, &Outcome::Values(vec![]));
        // another request right away waits out the gap
        p.before((0x700, 0x22, 0x1F02), None).unwrap();
        assert_eq!(fake_now() - start, MIN_GAP);
        // the first is not due again until its `every` has passed
        assert_eq!(p.before(k, Some(1.0)), Err(Outcome::NotDue));
        advance(Duration::from_secs(1));
        assert!(p.before(k, Some(1.0)).is_ok());
        // an `every` shorter than 0.25 s is held to 0.25 s
        let fast = (0x700, 0x22, 0x1F09);
        p.before(fast, Some(0.01)).unwrap();
        advance(Duration::from_millis(100));
        assert_eq!(p.before(fast, Some(0.01)), Err(Outcome::NotDue));
        // busy: the module's gap doubles, up to 2 s
        let busy = (0x701, 0x22, 0x1F11);
        for _ in 0..8 {
            p.after(busy, &Outcome::Refused(0x21));
        }
        assert_eq!(p.slow[&0x701].0, MAX_SLOW);
        advance(Duration::from_secs(5));
        p.before(busy, None).unwrap();
        let t0 = fake_now();
        p.before(busy, None).unwrap();
        assert_eq!(fake_now() - t0, MAX_SLOW);
        for _ in 0..10 {
            p.after(busy, &Outcome::Values(vec![]));
        }
        assert_eq!(
            p.slow[&0x701].0,
            MAX_SLOW / 2,
            "halved after 10 clean answers"
        );
        // refused as not there: dropped for the session
        let gone = (0x700, 0x22, 0x1F03);
        p.after(gone, &Outcome::Refused(0x31));
        assert!(
            matches!(p.before(gone, None), Err(Outcome::Skipped(w)) if w.contains("refused this session"))
        );
        // three silent answers: the module is dropped
        for _ in 0..3 {
            p.after((0x7C4, 0x22, 1), &Outcome::Silent);
        }
        assert!(
            matches!(p.before((0x7C4, 0x22, 2), None), Err(Outcome::Skipped(w)) if w.contains("stopped answering"))
        );
        // the gap can be raised, never lowered
        assert_eq!(pacer().with_gap(Duration::from_millis(10)).gap(), MIN_GAP);
        assert_eq!(
            pacer().with_gap(Duration::from_millis(200)).gap(),
            Duration::from_millis(200)
        );
        // stopping (Ctrl+C, a recording) sends nothing
        fn stop() -> Option<String> {
            Some("a recording owns the adapter".into())
        }
        let mut s = Pacer::new(fake_now, fake_sleep, stop);
        assert!(matches!(s.before(k, None), Err(Outcome::Skipped(w)) if w.contains("recording")));
        assert_eq!(s.sent, 0);
    }

    /// A link that answers from a script: each request takes the next window. It notes where each went, and never takes a
    /// broadcast.
    struct Script(Vec<Vec<crate::elm::Answer>>, usize);
    impl Link for Script {
        fn request(&mut self, _: &[u8]) -> Result<Vec<crate::elm::Answer>, crate::elm::ObdError> {
            unreachable!()
        }
        fn request_to(
            &mut self,
            _: u32,
            _: u32,
            _: &[u8],
        ) -> Result<Vec<crate::elm::Answer>, crate::elm::ObdError> {
            self.1 += 1;
            Ok(self.0.get(self.1 - 1).cloned().unwrap_or_default())
        }
        fn protocol(&self) -> crate::elm::Protocol {
            crate::elm::Protocol::Can11At500
        }
        fn adapter(&self) -> String {
            "script".into()
        }
        fn voltage(&mut self) -> Option<f64> {
            None
        }
    }

    #[test]
    fn a_late_answer_to_another_identifier_is_not_taken() {
        let a = |d: &[u8]| crate::elm::Answer {
            from: "708".into(),
            data: d.to_vec(),
        };
        // the window holds a late answer to 1F01 and then the right one
        let mut l = Script(
            vec![vec![a(&[0x62, 0x1F, 0x01, 9]), a(&[0x62, 0x1F, 0x02, 7])]],
            0,
        );
        assert_eq!(
            uds::read_data(&mut l, 0x700, 0x708, 0x22, 0x1F02).unwrap(),
            DataAnswer::Data(vec![7])
        );
        // only the late one: asked once more, and the second window has the right answer
        let mut l = Script(
            vec![
                vec![a(&[0x62, 0x1F, 0x01, 9])],
                vec![a(&[0x62, 0x1F, 0x02, 7])],
            ],
            0,
        );
        assert_eq!(
            uds::read_data(&mut l, 0x700, 0x708, 0x22, 0x1F02).unwrap(),
            DataAnswer::Data(vec![7])
        );
        assert_eq!(l.1, 2);
        // and the second window is paced: it waits RESEND_WAIT (on the pacer's clock), at least the least gap
        assert!(uds::RESEND_WAIT >= MIN_GAP);
        let mut l = Script(
            vec![
                vec![a(&[0x62, 0x1F, 0x01, 9])],
                vec![a(&[0x62, 0x1F, 0x02, 7])],
            ],
            0,
        );
        let before = NAPS.with(|n| n.get());
        assert_eq!(
            uds::read_data_with(&mut l, 0x700, 0x708, 0x22, 0x1F02, &mut fake_sleep).unwrap(),
            DataAnswer::Data(vec![7])
        );
        assert_eq!(NAPS.with(|n| n.get()) - before, uds::RESEND_WAIT);
        // late twice: silent, never the wrong bytes
        let mut l = Script(
            vec![
                vec![a(&[0x62, 0x1F, 0x01, 9])],
                vec![a(&[0x62, 0x1F, 0x01, 9])],
            ],
            0,
        );
        assert_eq!(
            uds::read_data(&mut l, 0x700, 0x708, 0x22, 0x1F02).unwrap(),
            DataAnswer::Silent
        );
        // refusals; and the bytes Icarus builds are only reads
        let mut l = Script(vec![vec![a(&[0x7F, 0x22, 0x31])]], 0);
        assert_eq!(
            uds::read_data(&mut l, 0x700, 0x708, 0x22, 0x1F02).unwrap(),
            DataAnswer::Refused(0x31)
        );
        assert!(uds::read_data(&mut l, 0x700, 0x708, 0x2E, 0x1F02).is_err());
        assert!(uds::read_data(&mut l, 0x700, 0x708, 0x21, 0x1F02).is_err());
        // a short answer
        let set = sim_set();
        let r = &set.module(0x700).unwrap().requests[0];
        assert_eq!(decode_all(r, &[1, 2, 3], None), Outcome::Short(3));
    }

    #[test]
    fn sets_match_the_vehicle_and_the_simulation_keeps_its_own() {
        let dir = std::env::temp_dir().join(format!("icarus-moddata-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let profiles = crate::vehicles::builtin();
        let imp = Imported {
            set: Some(SetInfo {
                format: 1,
                name: "toy-truck".into(),
                make: "toyota".into(),
                models: vec!["Truck".into()],
                years: vec![[2016, 2023]],
                source: md::Source {
                    from: "hand".into(),
                    ..Default::default()
                },
            }),
            modules: vec![module("0x7A0", ""), module("0x780", "")],
            ..Default::default()
        };
        let mut gated = imp.clone();
        gate_imported(&mut gated, &profiles);
        assert!(gated
            .left_out
            .iter()
            .any(|(w, why)| w.starts_with("780") && why.contains("airbag")));
        write_set(&dir, &gated, false).unwrap();
        assert!(write_set(&dir, &gated, false)
            .unwrap_err()
            .contains("--replace"));
        write_set(&dir, &gated, true).unwrap();
        let (sets, problems) = load_all(Some(&dir), &profiles);
        assert!(problems.is_empty(), "{problems:?}");
        assert_eq!(sets.len(), 2, "the user's set and the built-in one");
        let names = |v: Vec<&Loaded>| v.iter().map(|s| s.set.name.clone()).collect::<Vec<_>>();
        assert_eq!(
            names(sets_for(
                &sets,
                Some("toyota"),
                Some(2017),
                Some("2017 Toyota Truck"),
                Some("X")
            )),
            vec!["toy-truck"]
        );
        assert!(sets_for(&sets, Some("toyota"), Some(2030), None, None).is_empty());
        assert!(sets_for(
            &sets,
            Some("toyota"),
            Some(2017),
            Some("2017 Toyota Car"),
            None
        )
        .is_empty());
        assert!(sets_for(&sets, Some("subaru"), Some(2017), Some("Truck"), None).is_empty());
        // nothing guessed: the model, the year or the make unknown fits no set that names one
        assert!(
            sets_for(&sets, Some("toyota"), Some(2017), None, Some("X")).is_empty(),
            "the model unknown: a Tacoma's set is not put on a Prius"
        );
        assert!(
            sets_for(&sets, Some("toyota"), None, Some("Truck"), None).is_empty(),
            "the year unknown"
        );
        assert!(
            sets_for(&sets, None, Some(2017), Some("Truck"), None).is_empty(),
            "the make unknown"
        );
        // and an unread VIN never gets the simulated vehicle's set
        assert!(sets_for(&sets, Some("toyota"), Some(2025), None, None).is_empty());
        assert_eq!(
            names(sets_for(
                &sets,
                Some("toyota"),
                Some(2025),
                None,
                Some(crate::sim::VIN)
            )),
            vec!["sim"]
        );
        // a hand-edited file that names a write is refused when the set is read, and named
        std::fs::write(dir.join("toy-truck/7A0.toml"), "format = 1\n[module]\nname = \"x\"\ntx = 0x7A0\n[[request]]\nservice = 0x2E\nident = 1\n").unwrap();
        let s = load_set("toy-truck", Some(&dir), &profiles).unwrap();
        assert!(s
            .refused
            .iter()
            .any(|(w, why)| w.ends_with("7A0.toml") && why.contains("not one Icarus reads with")));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_model_fits_by_its_whole_phrase() {
        for model in ["F-150", "2017 Ford F-150 XLT", "f150", "F 150 Raptor"] {
            assert!(model_fits("F 150", model), "{model}");
        }
        for model in ["Focus", "Fusion", "F-250", "Fiesta", "F-1500", "150", ""] {
            assert!(!model_fits("F 150", model), "{model}");
        }
        assert!(model_fits("Tacoma", "2017 Toyota Tacoma TRD"));
        assert!(!model_fits("Tacoma", "Prius"));
        assert!(!model_fits("", "Prius"), "an empty phrase fits nothing");
        assert!(model_fits("Range Rover", "Land Rover Range Rover Sport"));
        assert!(!model_fits("Range Rover", "Range-Sport Rover"));
    }

    #[test]
    fn the_simulated_set_is_never_used_on_another_vehicle() {
        assert_eq!(set_refused(SIM_SET, Some(crate::sim::VIN)), None);
        assert!(set_refused(SIM_SET, Some("JTEXAMPLE00000000"))
            .unwrap()
            .contains("never sent to another vehicle"));
        assert!(set_refused(SIM_SET, None)
            .unwrap()
            .contains("VIN could not be read"));
        assert_eq!(set_refused("toy-truck", None), None);
        // an unread VIN's state is not the simulation's file, nor kept anywhere another vehicle would read it
        assert_eq!(state_path(None).unwrap(), None);
    }

    #[test]
    fn a_module_of_unknown_kind_gets_nothing_where_the_profile_names_no_airbag_module() {
        let profiles = crate::vehicles::builtin();
        let mut unknown = module("0x7A2", "");
        unknown.module.kind.clear();
        unknown.module.name = "module (7A2)".into();
        // GM's profile names no airbag module: a module that is only an address could be it
        for make in [
            "gm",
            "honda",
            "nissan",
            "stellantis",
            "bmw",
            "",
            "nosuchmake",
        ] {
            let gates = Gates::new(&profiles, std::slice::from_ref(&unknown), make);
            let why = gates
                .module_refused(&unknown)
                .unwrap_or_else(|| panic!("{make}"));
            assert!(why.contains("could be the airbag module"), "{make}: {why}");
        }
        // with a kind, or named by the profile (GM's 7E0 engine), or on a make whose profile knows its airbag module
        let gates = Gates::new(&profiles, std::slice::from_ref(&unknown), "gm");
        assert_eq!(
            gates.module_refused(&module("0x7A2", "")),
            None,
            "kind = body"
        );
        let mut engine = unknown.clone();
        engine.module.tx = 0x7E0;
        assert_eq!(gates.module_refused(&engine), None);
        let gates = Gates::new(&profiles, std::slice::from_ref(&unknown), "toyota");
        assert_eq!(gates.module_refused(&unknown), None);
        // an acronym in the name is enough, whatever the make
        let mut sdm = unknown.clone();
        sdm.module.name = "SDM".into();
        assert_eq!(
            gates.module_refused(&sdm).as_deref(),
            Some("airbag module: identity only")
        );
        // and at load: the requests come out, listed
        let mut ms = vec![unknown];
        let gates = Gates::new(&profiles, &ms, "gm");
        let refused = gate(&mut ms, &gates);
        assert!(
            ms[0].requests.is_empty() && refused.len() == 1,
            "{refused:?}"
        );
    }

    #[test]
    fn the_airbag_caution_follows_the_vehicle_not_the_set() {
        let profiles = crate::vehicles::builtin();
        let mut unknown = module("0x7A2", "");
        unknown.module.kind.clear();
        unknown.module.name = "module (7A2)".into();
        let set = Loaded {
            set: SetInfo {
                format: 1,
                name: "toy-truck".into(),
                make: "toyota".into(),
                models: vec![],
                years: vec![],
                source: md::Source::default(),
            },
            modules: vec![unknown.clone()],
            refused: vec![],
            source: "test".into(),
            gates: Gates::new(&profiles, std::slice::from_ref(&unknown), "toyota"),
        };
        assert_eq!(
            set.gates.module_refused(&unknown),
            None,
            "on a Toyota the airbag module's address is known"
        );
        // a Toyota set on a GM, or on a vehicle whose make is not known: the module could be the airbag module
        for vehicle in [Some("gm"), Some("honda"), None] {
            let on = set.on_vehicle(&profiles, vehicle);
            let why = on
                .gates
                .module_refused(&unknown)
                .unwrap_or_else(|| panic!("{vehicle:?}"));
            assert!(
                why.contains("could be the airbag module"),
                "{vehicle:?}: {why}"
            );
            assert!(
                on.modules[0].requests.is_empty(),
                "{vehicle:?}: no request kept"
            );
            assert!(
                on.refused.iter().any(|(w, _)| w.starts_with("7A2")),
                "{vehicle:?}"
            );
            // and at send time, nothing goes out
            let (port, log) = crate::sim::recording::Recorder::new(SimCar::new());
            let mut elm = Elm::connect(Box::new(port)).unwrap();
            log.lock().unwrap().clear();
            let o = read_request(
                &mut elm,
                &mut pacer(),
                &on.gates,
                &unknown,
                &unknown.requests[0],
                &Scope::default(),
            );
            assert!(
                matches!(&o, Outcome::Skipped(w) if w.contains("airbag")),
                "{o:?}"
            );
            assert!(
                sent(&log.lock().unwrap()).is_empty(),
                "{vehicle:?}: nothing was sent"
            );
        }
        // the vehicle's make the set's: as loaded
        let same = set.on_vehicle(&profiles, Some("toyota"));
        assert_eq!(same.gates.module_refused(&unknown), None);
        assert_eq!(same.modules[0].requests.len(), 1);
        // a GM set on a Toyota keeps the GM caution (the stricter of the two)
        let gm = Gates::for_vehicle(
            &profiles,
            std::slice::from_ref(&unknown),
            "gm",
            Some("toyota"),
        );
        assert!(gm.module_refused(&unknown).is_some());
        // a module only one of the two profiles names is not of known kind on the other make's vehicle (GM names 7E2)
        let mut tcm = unknown.clone();
        tcm.module.tx = 0x7E2;
        let on_gm = Gates::for_vehicle(&profiles, std::slice::from_ref(&tcm), "gm", Some("honda"));
        assert!(on_gm.module_refused(&tcm).is_some());
        assert_eq!(
            Gates::for_vehicle(&profiles, std::slice::from_ref(&tcm), "gm", Some("gm"))
                .module_refused(&tcm),
            None
        );
    }

    #[test]
    fn an_address_a_profile_names_as_the_airbag_module_is_refused() {
        // a user's profile naming its airbag module at an address Icarus does not know as one
        let mut profiles = crate::vehicles::builtin();
        profiles.push(
            crate::vehicles::parse(
                "myvan",
                "test",
                "name = \"My van\"\n[[module]]\ntx = 0x7A4\nname = \"SDM\"\n",
            )
            .unwrap(),
        );
        assert!(
            !crate::actuator::is_airbag(0x7A4, ""),
            "not one of the built-in airbag IDs"
        );
        // a set file that calls it only by its address, or even gives it another kind
        let mut by_address = module("0x7A4", "");
        by_address.module.kind.clear();
        by_address.module.name = "module (7A4)".into();
        let body = module("0x7A4", "");
        // answering from the airbag module's answer ID
        let answers = module("0x7A6", "rx = 0x7AC");
        for m in [&by_address, &body, &answers] {
            for make in ["myvan", "toyota", "gm", ""] {
                let gates = Gates::new(&profiles, std::slice::from_ref(m), make);
                assert_eq!(
                    gates.module_refused(m).as_deref(),
                    Some("airbag module: identity only"),
                    "{} on {make:?}",
                    m.module.tx
                );
            }
        }
        // at load: its requests come out
        let mut ms = vec![body.clone(), module("0x7A0", "")];
        let gates = Gates::new(&profiles, &ms, "myvan");
        let refused = gate(&mut ms, &gates);
        assert!(
            ms[0].requests.is_empty() && ms[1].requests.len() == 1,
            "{refused:?}"
        );
        assert_eq!(refused[0].1, "airbag module: identity only");
        // the van's profile names its airbag module: another module of unknown kind is not refused for that
        let mut other = by_address.clone();
        other.module.tx = 0x7A0;
        assert_eq!(gates.module_refused(&other), None);
    }

    #[test]
    fn a_refusal_is_believed_only_when_the_module_refuses_again() {
        let profiles = crate::vehicles::builtin();
        let m = module("0x7A0", "");
        let r = &m.requests[0];
        let gates = Gates::new(&profiles, std::slice::from_ref(&m), "toyota");
        let a = |d: &[u8]| crate::elm::Answer {
            from: "7A8".into(),
            data: d.to_vec(),
        };
        let k = key(&m, r);
        // a late "out of range" from an earlier request, then the module's own answer when asked again
        let mut l = Script(
            vec![
                vec![a(&[0x7F, 0x22, 0x31])],
                vec![a(&[0x62, 0x1F, 0x21, 5])],
            ],
            0,
        );
        let mut p = pacer();
        let before = NAPS.with(|n| n.get());
        let o = read_request(&mut l, &mut p, &gates, &m, r, &Scope::default());
        assert!(
            matches!(&o, Outcome::Values(v) if v[0].shown == Shown::Number(5.0)),
            "{o:?}"
        );
        assert_eq!(l.1, 2, "asked twice");
        assert!(
            NAPS.with(|n| n.get()) - before >= MIN_GAP,
            "the second ask is paced"
        );
        assert_eq!(p.sent, 2);
        let mut state = State::default();
        state.record(k, &o, "2026-11-14");
        assert!(state.refused_keys().is_empty(), "not kept as refused");
        advance(Duration::from_secs(1));
        assert!(p.before(k, None).is_ok(), "nor dropped for the session");
        // a late refusal and then silence: not believed either
        let mut l = Script(vec![vec![a(&[0x7F, 0x22, 0x31])], vec![]], 0);
        let o = read_request(&mut l, &mut pacer(), &gates, &m, r, &Scope::default());
        assert_eq!(o, Outcome::Silent);
        assert!(!State::default().record(k, &o, "2026-11-14"));
        // refused twice: believed, dropped for the session and kept for the vehicle
        let mut l = Script(
            vec![vec![a(&[0x7F, 0x22, 0x31])], vec![a(&[0x7F, 0x22, 0x31])]],
            0,
        );
        let mut p = pacer();
        let o = read_request(&mut l, &mut p, &gates, &m, r, &Scope::default());
        assert_eq!(o, Outcome::Refused(0x31));
        assert_eq!(l.1, 2);
        assert!(
            matches!(p.before(k, None), Err(Outcome::Skipped(w)) if w.contains("refused this session"))
        );
        let mut state = State::default();
        assert!(state.record(k, &o, "2026-11-14"));
        assert!(state.refused_keys().contains(&k));
        // a refusal that may change ("conditions not correct") is not asked again within the read
        let mut l = Script(
            vec![
                vec![a(&[0x7F, 0x22, 0x22])],
                vec![a(&[0x62, 0x1F, 0x21, 5])],
            ],
            0,
        );
        let o = read_request(&mut l, &mut pacer(), &gates, &m, r, &Scope::default());
        assert_eq!((o, l.1), (Outcome::Refused(0x22), 1));
    }

    #[test]
    fn one_module_per_request_id() {
        let profiles = crate::vehicles::builtin();
        let mut ms = vec![module("0x7A0", ""), module("0x7A0", "rx = 0x7AA")];
        let gates = Gates::new(&profiles, &ms, "toyota");
        let refused = gate(&mut ms, &gates);
        assert_eq!(ms[0].requests.len(), 1, "the first keeps its requests");
        assert!(ms[1].requests.is_empty());
        assert!(
            refused[0].1.contains("second module on request ID 7A0"),
            "{refused:?}"
        );
        // a request ID one file calls the airbag module's is the airbag module's in the other file too
        let mut ms = vec![
            module("0x7A0", "kind = \"airbag\""),
            module("0x7A0", "rx = 0x7AA"),
        ];
        let gates = Gates::new(&profiles, &ms, "toyota");
        let refused = gate(&mut ms, &gates);
        assert!(ms.iter().all(|m| m.requests.is_empty()));
        assert!(refused
            .iter()
            .all(|r| r.1 == "airbag module: identity only"));
    }

    #[test]
    fn a_set_name_cannot_leave_the_library() {
        let dir = std::env::temp_dir().join(format!("icarus-setname-{}", std::process::id()));
        for name in ["../x", "a/b", "/etc", "..", "Sim"] {
            assert!(
                load_set(name, Some(&dir), &[])
                    .unwrap_err()
                    .contains("lowercase letters, digits and dashes"),
                "{name}"
            );
        }
    }

    #[test]
    fn failures_and_refusals_that_cannot_change_are_dropped_for_the_session() {
        let mut p = pacer();
        let k = (0x700, 0x22, 0x1F01);
        // two failures, then an answer: counting starts again
        for _ in 0..2 {
            p.after(k, &Outcome::Failed("garbled".into()));
        }
        p.after(k, &Outcome::Values(vec![]));
        p.after(k, &Outcome::Failed("garbled".into()));
        assert!(p.before(k, None).is_ok());
        // three in a row: dropped
        for _ in 0..2 {
            p.after(k, &Outcome::Failed("garbled".into()));
        }
        assert!(
            matches!(p.before(k, None), Err(Outcome::Skipped(w)) if w.contains("failed 3 times in a row") && w.contains("garbled"))
        );
        for (c, word) in [(0x33, "security"), (0x7F, "session"), (0x7E, "session")] {
            let k = (0x701, 0x22, c as u16);
            p.after(k, &Outcome::Refused(c));
            assert!(
                matches!(p.before(k, None), Err(Outcome::Skipped(w)) if w.contains("refused this session") && w.contains(word)),
                "{c:02X}"
            );
        }
        // "conditions not correct" may change (the engine stopped): asked again
        let k = (0x701, 0x22, 0x22);
        p.after(k, &Outcome::Refused(0x22));
        assert!(p.before(k, None).is_ok());
    }

    /// A link that notes the (fake) time of every request it passes on.
    struct Timed<'a>(&'a mut dyn Link, Vec<Instant>);
    impl Link for Timed<'_> {
        fn request(&mut self, d: &[u8]) -> Result<Vec<crate::elm::Answer>, crate::elm::ObdError> {
            self.1.push(fake_now());
            self.0.request(d)
        }
        fn request_to(
            &mut self,
            tx: u32,
            rx: u32,
            d: &[u8],
        ) -> Result<Vec<crate::elm::Answer>, crate::elm::ObdError> {
            self.1.push(fake_now());
            self.0.request_to(tx, rx, d)
        }
        fn protocol(&self) -> crate::elm::Protocol {
            self.0.protocol()
        }
        fn adapter(&self) -> String {
            self.0.adapter()
        }
        fn voltage(&mut self) -> Option<f64> {
            self.0.voltage()
        }
    }

    #[test]
    fn every_send_is_at_least_the_gap_apart() {
        let set = sim_set();
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        let mut link = Timed(&mut elm, vec![]);
        let mut p = pacer();
        let scope = sim_scope();
        for _ in 0..3 {
            for tx in [0x700, 0x701, 0x7E0] {
                read_module(
                    &mut link,
                    &mut p,
                    &set.gates,
                    set.module(tx).unwrap(),
                    &scope,
                    None,
                );
            }
            advance(Duration::from_secs(2));
        }
        let times = link.1;
        assert!(
            times.len() as u64 > p.sent,
            "a pending answer was asked again within one request ({} sends, {} requests)",
            times.len(),
            p.sent
        );
        for (i, w) in times.windows(2).enumerate() {
            assert!(
                w[1] - w[0] >= MIN_GAP,
                "sends {i} and {} were {:?} apart",
                i + 1,
                w[1] - w[0]
            );
        }
    }

    #[test]
    fn the_vehicle_remembers_what_each_request_did() {
        let mut s = State {
            set: Some("sim".into()),
            ..Default::default()
        };
        let k = (0x701, 0x22, 0x1F11);
        assert!(s.record(k, &Outcome::Values(vec![]), "2026-11-14"));
        assert!(
            !s.record(k, &Outcome::Values(vec![]), "2026-11-15"),
            "the first answer is kept"
        );
        assert!(s.record((0x700, 0x22, 0x1F03), &Outcome::Refused(0x31), "2026-11-14"));
        assert!(
            !s.record((0x700, 0x22, 0x1F09), &Outcome::Refused(0x22), "2026-11-14"),
            "conditions: not 'not here'"
        );
        assert!(s.record((0x700, 0x21, 0x04), &Outcome::Short(2), "2026-11-14"));
        s.check(0x701, "SIM_ATF", Some("matched the gauge"), "2026-11-14");
        assert_eq!(s.refused_keys(), BTreeSet::from([(0x700, 0x22, 0x1F03)]));
        let text = s.to_toml();
        assert!(
            text.contains("module = 0x701") && text.contains("refused = \"7F 22 31\""),
            "{text}"
        );
        let back: State = toml::from_str(&text).unwrap();
        assert_eq!(back, s);
        assert_eq!(
            back.checked(0x701, "SIM_ATF").unwrap().note.as_deref(),
            Some("matched the gauge")
        );
        // a refused request is skipped next time, unless asked again
        let set = sim_set();
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        let mut scope = sim_scope();
        scope.refused_before = s.refused_keys();
        let m = set.module(0x700).unwrap();
        let o = read_request(
            &mut elm,
            &mut pacer(),
            &set.gates,
            m,
            &m.requests[2],
            &scope,
        );
        assert!(matches!(&o, Outcome::Skipped(w) if w.contains("--all")));
        scope.retry = true;
        assert_eq!(
            read_request(
                &mut elm,
                &mut pacer(),
                &set.gates,
                m,
                &m.requests[2],
                &scope
            ),
            Outcome::Refused(0x31)
        );
    }

    #[test]
    fn bmw_and_gm_module_ids_take_data_reads() {
        let profiles = crate::vehicles::builtin();
        let refused = |tx: u32, ext: Option<u8>| data_request_id_refused(tx, ext, &profiles);
        // BMW's modules by their address on 6F1; DF and EF are every module, and 6F1 alone is no module
        assert_eq!(refused(0x6F1, Some(0x12)), None);
        for every in [0xDF, 0xEF] {
            assert!(refused(0x6F1, Some(every))
                .unwrap()
                .contains("every BMW module"));
        }
        assert!(refused(0x6F1, None).unwrap().contains("700-7F7"));
        // GM's body modules, 240-24F; 250 is none of them
        assert_eq!(refused(0x241, None), None);
        assert_eq!(refused(0x24F, None), None);
        assert!(refused(0x250, None).unwrap().contains("700-7F7"));
        // the rest as before: a module request ID, never an answer ID or the broadcast
        assert_eq!(refused(0x7E0, None), None);
        assert!(refused(0x7E8, None).unwrap().contains("answer ID"));
        assert!(refused(0x7DF, None).unwrap().contains("broadcast"));
        // the actions keep 700-7F7
        assert!(crate::actuator::not_one_module(0x241).is_some());
    }

    #[test]
    fn bmw_and_gm_modules_from_a_bus_log_are_kept() {
        use icarus_core::moddata::capture::{build, buslog, Options};
        let profiles = crate::vehicles::builtin();
        // made-up identifiers. BMW: 12 and 18 read on 6F1, answered on 612 and 618; a read sent to EF (every module on
        // the E-series) is answered by 612 too
        let bmw = "\
   1.000       6F1  12 03 22 40 01 00 00 00
   1.010       612  F1 05 62 40 01 0A 0B 00
   1.100       6F1  18 03 22 40 02 00 00 00
   1.110       618  F1 04 62 40 02 01 00 00
   1.200       6F1  EF 03 22 40 03 00 00 00
   1.210       612  F1 04 62 40 03 07 00 00
";
        // GM: 241 and 242 read, answered on 641 and 642
        let gm = "\
   0.100       241  03 22 00 0C 00 00 00 00
   0.110       641  05 62 00 0C 1A F8 00 00
   0.200       242  03 22 19 40 00 00 00 00
   0.210       642  04 62 19 40 05 00 00 00
";
        for (make, log, want) in [
            ("bmw", bmw, ["6F1.12", "6F1.18"]),
            ("gm", gm, ["241", "242"]),
        ] {
            let o = Options {
                file_name: format!("{make}.log"),
                make: make.into(),
                from: "bus capture".into(),
                ..Default::default()
            };
            let imp = build(&buslog::read(log).unwrap().exchanges, &o).unwrap();
            let labels: Vec<String> = imp.modules.iter().map(|m| m.module.label()).collect();
            assert_eq!(labels, want, "{:?}", imp.left_out);
            if make == "bmw" {
                assert!(
                    imp.left_out
                        .iter()
                        .any(|(w, why)| w.starts_with("6F1.EF") && why.contains("every module")),
                    "{:?}",
                    imp.left_out
                );
            }
            // neither profile names an airbag module, so a module of unknown kind gets no request: refused for that, and not
            // for its ID
            let mut gated = imp.clone();
            gate_imported(&mut gated, &profiles);
            assert!(gated.modules.iter().all(|m| m.requests.is_empty()));
            assert!(
                gated
                    .left_out
                    .iter()
                    .all(|(_, why)| !why.contains("700-7F7")),
                "{:?}",
                gated.left_out
            );
            assert!(gated.left_out.iter().any(|(_, why)| why.contains("airbag")));
            // given a kind (as the module's file would be once known), every module keeps its read
            let mut known = imp.clone();
            for m in &mut known.modules {
                m.module.kind = "body".into();
            }
            gate_imported(&mut known, &profiles);
            assert!(
                known.modules.iter().all(|m| m.requests.len() == 1),
                "{:?}",
                known.left_out
            );
        }
    }
}
