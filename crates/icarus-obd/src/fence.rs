//! The key modules' fence (ICA-101): immobilizer, smart-key and key-certification modules. They hold what lets the vehicle
//! start, so Icarus writes nothing to them, runs no test or routine in them, clears none of their codes, and learns nothing
//! from them: a dealer tool's key registration watched on the bus never becomes a setting, a test or a service entry. They
//! get the identity reads Icarus already makes and nothing else (module data and the as-built sweep send them nothing,
//! whichever file names them: a key module one set's file names is fenced in every set, and the sweep asks `current`),
//! and nothing from them goes to an assistant (`icarus mcp`) or into a learning session.
//!
//! They are fenced by address, whatever a definition calls itself: a vehicle profile's `[[module]]` or a module-data file's
//! `[module]` gives the kind (`kind = "immobilizer"`, `"smart-key"` or `"certification"`; a name in those words counts too),
//! and every write path (`actuator`, `settings`, `service`, `clear`), the bus learner (`bus`), the module-data gates
//! (`moddata::Gates`, with the other sets' key modules added by `moddata::load_all`) and the as-built sweep (`asbuilt`)
//! look the address up here. IDs are u32 from the start, so a 29-bit module fits beside an 11-bit one.
//! A module behind a gateway shares the gateway's CAN IDs, so where Icarus cannot tell the modules behind it apart (a write
//! path, the bus learner) the whole gateway ID is fenced.
//!
//! No maker's addresses are built in: a vehicle's are written down from its own identity reads once I have confirmed them,
//! never guessed (an unconfirmed address could fence the wrong module, or miss the right one).

use crate::vehicles::Profile;
use icarus_core::moddata::ModuleHead;

/// The kinds of key module.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Kind {
    /// The immobilizer (a transponder key's reader and its code), also called anti-theft or PATS.
    Immobilizer,
    /// The smart key (keyless entry and start) module.
    SmartKey,
    /// The key-certification module (Toyota's certification ECU), which checks a key before the engine may start.
    Certification,
}

impl Kind {
    pub const ALL: [Kind; 3] = [Kind::Immobilizer, Kind::SmartKey, Kind::Certification];

    /// The word a file gives it.
    pub fn word(self) -> &'static str {
        match self {
            Kind::Immobilizer => "immobilizer",
            Kind::SmartKey => "smart-key",
            Kind::Certification => "certification",
        }
    }

    /// The kind a module's file gives it (`kind`), else the one its name says, when it is a key module. Matched by words,
    /// so British spelling, "smart key" written apart and a maker's plural all count: "immobilizer", "immobiliser",
    /// "transponder", "anti-theft", "theft deterrent" and Ford's "PATS" are an immobilizer; "smart key", "keyless" and
    /// "key" on its own a smart-key module; "certification" a certification module.
    pub fn of(kind: &str, name: &str) -> Option<Kind> {
        Kind::in_words(kind).or_else(|| Kind::in_words(name))
    }

    fn in_words(text: &str) -> Option<Kind> {
        let lower = text.to_lowercase();
        let w: Vec<&str> = lower
            .split(|c: char| !c.is_ascii_alphanumeric())
            .filter(|w| !w.is_empty())
            .collect();
        let starts = |p: &str| w.iter().any(|x| x.starts_with(p));
        let word = |x: &str| w.contains(&x);
        let pair = |a: &str, b: &str| w.windows(2).any(|p| p[0] == a && p[1].starts_with(b));
        if starts("certif") {
            Some(Kind::Certification)
        } else if starts("immobil")
            || starts("transponder")
            || starts("antitheft")
            || starts("theft")
            || word("pats")
            || pair("anti", "theft")
        {
            Some(Kind::Immobilizer)
        } else if starts("smartkey") || starts("keyless") || pair("smart", "key") || word("key") {
            Some(Kind::SmartKey)
        } else {
            None
        }
    }
}

/// One fenced module: where it is and what it is.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Fenced {
    pub tx: u32,
    pub rx: u32,
    /// The extended address byte of a module behind a gateway.
    pub ext: Option<u8>,
    pub kind: Kind,
    pub name: String,
}

impl Fenced {
    /// "7C0", "750.2A", "18DA40F1", with its kind: "7C0 (the smart-key module)".
    pub fn describe(&self) -> String {
        let id = if self.tx > 0x7FF {
            format!("{:08X}", self.tx)
        } else {
            format!("{:03X}", self.tx)
        };
        let at = match self.ext {
            Some(e) => format!("{id}.{e:02X}"),
            None => id,
        };
        format!("{at} (the {} module)", self.kind.word())
    }
}

/// Every key module Icarus knows the address of.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Fence {
    modules: Vec<Fenced>,
}

impl Fence {
    /// The key modules every profile names (any make's, as for the airbag modules: an address one make gives a key module
    /// is fenced on every vehicle, since a write path does not ask which make it is talking to).
    pub fn of(profiles: &[Profile]) -> Fence {
        let mut f = Fence::default();
        for m in profiles.iter().flat_map(crate::vehicles::named) {
            if let Some(kind) = Kind::of(&m.kind, &m.name) {
                f.push(Fenced {
                    tx: m.tx,
                    rx: m.rx(),
                    ext: m.ext,
                    kind,
                    name: m.name.clone(),
                });
            }
        }
        f
    }

    /// The key modules the built-in profiles name (none yet: see the top of this file).
    pub fn builtin() -> Fence {
        static BUILT_IN: std::sync::OnceLock<Fence> = std::sync::OnceLock::new();
        BUILT_IN
            .get_or_init(|| Fence::of(&crate::vehicles::builtin()))
            .clone()
    }

    /// Adds the key modules among module-data files' modules (by their `kind`, or their name).
    pub fn add_modules<'a>(&mut self, heads: impl IntoIterator<Item = &'a ModuleHead>) {
        for h in heads {
            if let Some(kind) = Kind::of(&h.kind, &h.name) {
                self.push(Fenced {
                    tx: h.tx,
                    rx: h.rx(),
                    ext: h.ext,
                    kind,
                    name: h.name.clone(),
                });
            }
        }
    }

    pub fn push(&mut self, f: Fenced) {
        if !self.modules.contains(&f) {
            self.modules.push(f);
        }
    }

    /// Adds every module of `other`.
    pub fn extend(&mut self, other: &Fence) {
        for f in &other.modules {
            self.push(f.clone());
        }
    }

    pub fn modules(&self) -> &[Fenced] {
        &self.modules
    }

    pub fn is_empty(&self) -> bool {
        self.modules.is_empty()
    }

    /// The key module a CAN ID belongs to, if any: its request or its answer ID. A module behind a gateway counts for the
    /// gateway's IDs, whatever the address byte, since a frame's ID alone cannot say which module behind it is meant.
    pub fn on_id(&self, id: u32) -> Option<&Fenced> {
        self.modules.iter().find(|f| f.tx == id || f.rx == id)
    }

    /// The key module at this module address, if it is one: the same request ID and address byte (a key module named at a
    /// gateway's ID without an address byte covers every module behind it), or, without an address byte, the same answer ID.
    pub fn at(&self, tx: u32, ext: Option<u8>, rx: u32) -> Option<&Fenced> {
        self.modules.iter().find(|f| {
            (f.tx == tx && (f.ext.is_none() || f.ext == ext))
                || (ext.is_none() && f.ext.is_none() && f.rx == rx)
        })
    }

    /// Why a write path may not send to the module at `tx` answering from `rx`, if it may not: either ID is a key module's
    /// (`on_id`).
    pub fn refuses(&self, tx: u32, rx: u32) -> Option<String> {
        let f = self.on_id(tx).or_else(|| self.on_id(rx))?;
        Some(format!(
            "refused: {} is fenced: Icarus never writes to a key module, tests it, runs a routine in it or clears its \
             codes, whatever a definition calls itself",
            f.describe()
        ))
    }
}

/// The fence the profiles in `vehicles` and the module-data sets in `data` give, with the built-in ones: every profile's
/// key modules, and every set's module file of a key kind or name (`add_modules`). `current` reads it from the home
/// folder's; tests give folders of their own. A file that cannot be read is left out, as when it is loaded for anything
/// else.
pub fn current_in(vehicles: Option<&std::path::Path>, data: Option<&std::path::Path>) -> Fence {
    let profiles = crate::vehicles::load(vehicles).0;
    let mut f = Fence::of(&profiles);
    for set in crate::moddata::load_all(data, &profiles).0 {
        f.add_modules(set.modules.iter().map(|m| &m.module));
    }
    f
}

/// The fence as it stands: `current_in` the home folder's profiles (`~/.config/icarus/vehicles`) and module-data sets
/// (`~/.config/icarus/data`), read again when the last reading is more than a few seconds old (a settings page reads each
/// setting in turn). The bus watch, `icarus learn` and the window's worker take their fence as a parameter, so their tests
/// give one of their own; this crate's tests get theirs below. Neither reads the home folder.
#[cfg(not(test))]
pub fn current() -> Fence {
    use std::sync::Mutex;
    use std::time::{Duration, Instant};
    static LAST: Mutex<Option<(Instant, Fence)>> = Mutex::new(None);
    let mut last = LAST.lock().unwrap_or_else(|e| e.into_inner());
    if let Some((at, f)) = last.as_ref() {
        if at.elapsed() < Duration::from_secs(5) {
            return f.clone();
        }
    }
    let f = current_in(
        crate::vehicles::user_dir().as_deref(),
        crate::moddata::user_dir().as_deref(),
    );
    *last = Some((Instant::now(), f.clone()));
    f
}

/// In tests: the test's own fence (`fence_for_test`), else the built-in profiles' (tests never read the home folder).
#[cfg(test)]
pub fn current() -> Fence {
    FOR_TEST
        .with(|t| t.borrow().clone())
        .unwrap_or_else(Fence::builtin)
}

#[cfg(test)]
thread_local! {
    static FOR_TEST: std::cell::RefCell<Option<Fence>> = const { std::cell::RefCell::new(None) };
}

/// Sets this test thread's fence (None: the built-in profiles' again).
#[cfg(test)]
pub(crate) fn fence_for_test(f: Option<Fence>) {
    FOR_TEST.with(|t| *t.borrow_mut() = f);
}

/// A fence with one module of `kind` at `tx` (answering tx + 8), from a profile written for a test.
#[cfg(test)]
pub(crate) fn one_for_test(tx: u32, kind: Kind) -> Fence {
    let p = crate::vehicles::parse(
        "test",
        "a test",
        &format!(
            "name = \"test\"\n[[module]]\ntx = 0x{tx:03X}\nname = \"test module\"\nkind = \"{}\"\n",
            kind.word()
        ),
    )
    .unwrap();
    Fence::of(&[p])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kinds_by_their_words() {
        for (text, kind) in [
            ("immobilizer", Kind::Immobilizer),
            ("Immobiliser ECU", Kind::Immobilizer),
            ("transponder key amplifier", Kind::Immobilizer),
            ("anti-theft", Kind::Immobilizer),
            ("theft deterrent module", Kind::Immobilizer),
            ("PATS", Kind::Immobilizer),
            ("smart-key", Kind::SmartKey),
            ("Smart Key ECU", Kind::SmartKey),
            ("smartkey", Kind::SmartKey),
            ("keyless entry and start", Kind::SmartKey),
            ("key module", Kind::SmartKey),
            ("certification", Kind::Certification),
            ("Certification ECU (smart key)", Kind::Certification),
        ] {
            assert_eq!(Kind::of(text, ""), Some(kind), "{text}");
            assert_eq!(Kind::of("", text), Some(kind), "{text} as a name");
        }
        for text in [
            "",
            "body",
            "engine (ECM)",
            "keypad lighting",
            "monkey",
            "patsy",
            "combination meter",
            "airbags (SRS)",
        ] {
            assert_eq!(Kind::of(text, text), None, "{text}");
        }
        // the file's kind wins over its name
        assert_eq!(
            Kind::of("certification", "smart key"),
            Some(Kind::Certification)
        );
        for k in Kind::ALL {
            assert_eq!(Kind::of(k.word(), ""), Some(k));
        }
    }

    #[test]
    fn a_fence_from_profiles_by_address() {
        let p = crate::vehicles::parse(
            "t",
            "a test",
            "name = \"t\"\n\
             [[module]]\ntx = 0x7D5\nname = \"key ECU\"\nkind = \"smart-key\"\n\
             [[module]]\ntx = 0x750\nrx = 0x758\next = 0x5C\nname = \"behind the gateway\"\nkind = \"certification\"\n\
             [[module]]\ntx = 0x18DA40F1\nname = \"29-bit\"\nkind = \"immobilizer\"\n\
             [[module]]\ntx = 0x7C0\nname = \"cluster\"\nkind = \"body\"\n",
        )
        .unwrap();
        let f = Fence::of(&[p]);
        assert_eq!(f.modules().len(), 3, "{f:?}");
        assert_eq!(f.on_id(0x7D5).unwrap().kind, Kind::SmartKey);
        assert_eq!(
            f.on_id(0x7DD).unwrap().kind,
            Kind::SmartKey,
            "its answer ID"
        );
        assert_eq!(
            f.on_id(0x18DA_F140).unwrap().kind,
            Kind::Immobilizer,
            "29-bit, answered from 18DAF140"
        );
        assert!(f.on_id(0x7C0).is_none() && f.on_id(0x7C8).is_none());
        // behind a gateway: exact by address byte for module data, the whole gateway ID for a write path
        assert_eq!(
            f.at(0x750, Some(0x5C), 0x758).unwrap().kind,
            Kind::Certification
        );
        assert!(f.at(0x750, Some(0x40), 0x758).is_none());
        assert!(f.refuses(0x750, 0x758).unwrap().contains("750.5C"));
        assert!(f.refuses(0x7D5, 0x7DD).unwrap().contains("smart-key"));
        assert!(f.refuses(0x7C0, 0x7C8).is_none());
        // the built-in profiles name no key module yet
        assert!(Fence::builtin().is_empty());
    }

    /// ICA-101: a setting, a service routine and an actuator test aimed at a module of each key kind are refused by its
    /// address, whatever they are called (the simulated cluster's chime and oil life reset, the engine's throttle test),
    /// and nothing is sent. Undo too: the chime was changed while its module was not fenced, so the log holds its way back
    /// and undo gets as far as the fence. With the module not fenced the same ones go through.
    #[test]
    fn writes_to_a_key_module_are_refused_by_address() {
        use crate::elm::Elm;
        use crate::sim::SimCar;
        crate::lock::switch_for_test(&[]);
        let chime = crate::sim::settings().remove(0);
        let oil = crate::sim::services()
            .into_iter()
            .find(|s| s.name.starts_with("Oil"))
            .unwrap();
        let throttle = crate::sim::tests()
            .into_iter()
            .find(|t| t.name.starts_with("Throttle"))
            .unwrap();
        for name in [&chime.name, &oil.name, &throttle.name] {
            assert_eq!(Kind::of("", name), None, "an innocent name: {name}");
        }
        let dir = std::env::temp_dir().join(format!("icarus-fence-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let log = crate::settings::log_path(&dir);
        let connect = || {
            let mut e = Elm::connect(Box::new(SimCar::engine_off())).unwrap();
            e.take_sent();
            e
        };
        // changed before its module was classified: the log has a line for the chime
        fence_for_test(Some(Fence::default()));
        let mut changed = connect();
        assert!(
            crate::settings::change(&mut changed, &chime, &chime.values["off"], &log)
                .unwrap()
                .is_some()
        );
        changed.take_sent();
        let logged = std::fs::read_to_string(&log).unwrap();
        assert!(crate::settings::last_change(&log, &chime).is_some());
        for kind in Kind::ALL {
            let refused = |e: &mut Elm, r: Result<(), String>, what: &str| {
                let why = r.unwrap_err();
                assert!(
                    why.contains("fenced") && why.contains(kind.word()),
                    "{what}, {kind:?}: {why}"
                );
                assert!(e.take_sent().is_empty(), "{what}, {kind:?}: nothing sent");
            };
            fence_for_test(Some(one_for_test(0x7C0, kind)));
            let mut e = connect();
            let r = crate::settings::change(&mut e, &chime, &chime.values["off"], &log).map(|_| ());
            refused(&mut e, r, "a setting");
            let r = crate::settings::read(&mut e, &chime).map(|_| ());
            refused(&mut e, r, "reading a setting");
            let r = crate::settings::undo(&mut changed, &chime, &log).map(|_| ());
            refused(&mut changed, r, "undoing a setting");
            let r = crate::service::run(&mut e, &oil, true).map(|_| ());
            refused(&mut e, r, "a service routine");
            let r = crate::clear::clear_module(
                &mut e,
                0x7C0,
                0x7C8,
                crate::uds::Dialect::Uds,
                &crate::vehicles::builtin(),
                Some("toyota"),
            );
            refused(&mut e, r, "clearing its codes");
            fence_for_test(Some(one_for_test(0x7E0, kind)));
            let mut e = connect();
            let r = crate::actuator::start(&mut e, &throttle, 30.0).map(|_| ());
            refused(&mut e, r, "an actuator test");
            assert_eq!(
                std::fs::read_to_string(&log).unwrap(),
                logged,
                "nothing more was logged as changed"
            );
        }
        // the same, not fenced
        fence_for_test(Some(Fence::default()));
        let u = crate::settings::undo(&mut changed, &chime, &log)
            .unwrap()
            .unwrap();
        assert_eq!((u.before, u.after), (vec![0x00], vec![0x01]), "undone");
        let mut e = connect();
        assert!(
            crate::settings::change(&mut e, &chime, &chime.values["off"], &log)
                .unwrap()
                .is_some()
        );
        assert!(crate::service::run(&mut e, &oil, true).is_ok());
        let mut run = crate::actuator::start(&mut e, &throttle, 30.0).unwrap();
        crate::actuator::stop(&mut e, &mut run, "test").unwrap();
        fence_for_test(None);
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A key module that only a module-data file names (no profile does) is fenced for the write paths too: the fence
    /// built from the folders, as `current` builds it from the home folder's, refuses a setting, a routine and clearing at
    /// its address, and nothing is sent. The same file without the key kind fences nothing.
    #[test]
    fn a_key_module_named_only_in_module_data_is_fenced() {
        use crate::elm::Elm;
        use crate::sim::SimCar;
        crate::lock::switch_for_test(&[]);
        let root =
            std::env::temp_dir().join(format!("icarus-fence-moddata-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        let (vehicles, data) = (root.join("vehicles"), root.join("data"));
        let write = |kind: &str| {
            let set = data.join("keys");
            std::fs::create_dir_all(&set).unwrap();
            std::fs::create_dir_all(&vehicles).unwrap();
            std::fs::write(
                set.join("set.toml"),
                "format = 1\nname = \"keys\"\nmake = \"toyota\"\n[source]\nfrom = \"hand\"\n",
            )
            .unwrap();
            std::fs::write(
                set.join("7C0.toml"),
                format!("format = 1\n[module]\nname = \"cluster\"\ntx = 0x7C0\n{kind}\n"),
            )
            .unwrap();
        };
        write("kind = \"smart-key\"");
        let f = current_in(Some(vehicles.as_path()), Some(data.as_path()));
        assert!(Fence::builtin().is_empty(), "no profile names it");
        assert_eq!(
            f.modules()
                .iter()
                .map(|m| (m.tx, m.kind))
                .collect::<Vec<_>>(),
            [(0x7C0, Kind::SmartKey)]
        );
        let chime = crate::sim::settings().remove(0);
        let oil = crate::sim::services()
            .into_iter()
            .find(|s| s.name.starts_with("Oil"))
            .unwrap();
        fence_for_test(Some(f));
        let mut e = Elm::connect(Box::new(SimCar::engine_off())).unwrap();
        e.take_sent();
        let log = crate::settings::log_path(&root);
        for (what, r) in [
            (
                "a setting",
                crate::settings::change(&mut e, &chime, &chime.values["off"], &log).map(|_| ()),
            ),
            (
                "a service routine",
                crate::service::run(&mut e, &oil, true).map(|_| ()),
            ),
            (
                "clearing its codes",
                crate::clear::clear_module(
                    &mut e,
                    0x7C0,
                    0x7C8,
                    crate::uds::Dialect::Uds,
                    &crate::vehicles::builtin(),
                    Some("toyota"),
                ),
            ),
        ] {
            let why = r.unwrap_err();
            assert!(
                why.contains("7C0 (the smart-key module) is fenced"),
                "{what}: {why}"
            );
        }
        assert!(e.take_sent().is_empty(), "nothing sent");
        assert!(!log.exists(), "nothing logged as changed");
        // the same set, its module of no key kind
        write("kind = \"body\"");
        assert!(current_in(Some(vehicles.as_path()), Some(data.as_path())).is_empty());
        fence_for_test(None);
        let _ = std::fs::remove_dir_all(&root);
    }

    /// Profiles with the key kinds load; a test, setting or routine another file aims at one of them is left out when the
    /// profiles are loaded, and said.
    #[test]
    fn profiles_with_key_modules_load_and_entries_on_them_are_left_out() {
        let dir =
            std::env::temp_dir().join(format!("icarus-fence-profiles-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(
            dir.join("toyota.toml"),
            "[[module]]\ntx = 0x7D5\nname = \"keys\"\nkind = \"smart-key\"\n\
             [[module]]\ntx = 0x7D6\nname = \"start\"\nkind = \"immobilizer\"\n\
             [[module]]\ntx = 0x750\nrx = 0x758\next = 0x5C\nname = \"behind the gateway\"\nkind = \"certification\"\n",
        )
        .unwrap();
        std::fs::write(
            dir.join("kart.toml"),
            "name = \"Kart\"\n[[module]]\ntx = 0x7E0\nname = \"engine\"\n\
             [[setting]]\nname = \"Mirror fold\"\nmodule = 0x7D5\nprotocol = \"uds\"\nid = 0x1001\nvalues = { on = [1], off = [0] }\n\
             [[test]]\nname = \"Horn\"\nmodule = 0x7D6\nprotocol = \"uds\"\nid = 0x2001\n\
             [[service]]\nname = \"Window relearn\"\nmodule = 0x750\nroutine = 0x3001\n\
             [[test]]\nname = \"Fan\"\nmodule = 0x7E0\nprotocol = \"uds\"\nid = 0x2002\n",
        )
        .unwrap();
        let (all, problems) = crate::vehicles::load(Some(&dir));
        let t = crate::vehicles::find(&all, "toyota").unwrap();
        let kinds: Vec<&str> = t
            .modules
            .iter()
            .filter(|m| !m.kind.is_empty())
            .map(|m| m.kind.as_str())
            .collect();
        assert_eq!(kinds, ["smart-key", "immobilizer", "certification"]);
        assert_eq!(Fence::of(&all).modules().len(), 3);
        let kart = crate::vehicles::find(&all, "kart").unwrap();
        assert!(kart.settings.is_empty() && kart.services.is_empty());
        assert_eq!(
            kart.tests
                .iter()
                .map(|t| t.name.as_str())
                .collect::<Vec<_>>(),
            ["Fan"]
        );
        assert_eq!(problems.len(), 3, "{problems:?}");
        for (what, kind) in [
            ("setting \"Mirror fold\"", "smart-key"),
            ("test \"Horn\"", "immobilizer"),
            ("service \"Window relearn\"", "certification"),
        ] {
            assert!(
                problems
                    .iter()
                    .any(|p| p.contains(what) && p.contains(kind) && p.contains("left out")),
                "{what}: {problems:?}"
            );
        }
        let _ = std::fs::remove_dir_all(&dir);
    }
}
