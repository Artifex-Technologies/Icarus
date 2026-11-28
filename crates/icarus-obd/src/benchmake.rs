//! The make a bench session names (ICA-222). The airbag caution (`vehicles::caution_holds`, `clear::refused_on`) takes the
//! vehicle's make from the VIN it reads before every write; a spare ECM on the bench with a blank or foreign VIN gives no
//! make, so every module write to it would be refused (mode 04 still works). So a session may name its make once: `--make
//! toyota` on the commands that write (`obd clear`, `test`, `service`, `setting`, `setting-undo`) and the window's "Bench
//! make" in its connection row. It is never guessed:
//!
//! - it is used only when the VIN is blank, unreadable or decodes to no profile;
//! - when the VIN names a make (a profile's VIN prefix), the named make must agree, or the write is refused and the
//!   mismatch said;
//! - it must be a profile's own id (`icarus vehicles` lists them), nothing looser;
//! - it is shown in every confirmation and written to the action log ("make chosen by me: toyota, VIN blank");
//! - it only picks the profile the caution follows: it never lifts the airbag list or the key modules' fence, which do
//!   not depend on the make.

use crate::vehicles::Profile;

#[cfg(not(test))]
static CHOSEN: std::sync::Mutex<Option<String>> = std::sync::Mutex::new(None);

#[cfg(test)]
thread_local! {
    /// A test's own bench make (tests run side by side in one process).
    static CHOSEN_FOR_TEST: std::cell::RefCell<Option<String>> = const { std::cell::RefCell::new(None) };
}

/// Names this session's bench make (a profile's id), or none (None: the make comes from the VIN alone). The command line
/// names it once, before it opens the adapter; the window when it connects.
pub fn choose(make: Option<&str>) {
    let make = make
        .map(|m| m.trim().to_ascii_lowercase())
        .filter(|m| !m.is_empty());
    #[cfg(test)]
    CHOSEN_FOR_TEST.with(|c| *c.borrow_mut() = make);
    #[cfg(not(test))]
    {
        *CHOSEN.lock().unwrap_or_else(|e| e.into_inner()) = make;
    }
}

/// This session's bench make, if one was named.
pub fn chosen() -> Option<String> {
    #[cfg(test)]
    return CHOSEN_FOR_TEST.with(|c| c.borrow().clone());
    #[cfg(not(test))]
    CHOSEN.lock().unwrap_or_else(|e| e.into_inner()).clone()
}

/// The profile id `make` names exactly (any case), or why it names none. Never a guess: no part of a name matches.
pub fn profile_id(profiles: &[Profile], make: &str) -> Result<String, String> {
    let m = make.trim();
    profiles
        .iter()
        .find(|p| p.id.eq_ignore_ascii_case(m))
        .map(|p| p.id.clone())
        .ok_or_else(|| {
            format!(
                "--make {m:?} names no vehicle profile; it must be a profile's own name: {} (icarus vehicles)",
                profiles
                    .iter()
                    .map(|p| p.id.as_str())
                    .collect::<Vec<_>>()
                    .join(", ")
            )
        })
}

/// What the VIN said when the make was asked.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum VinSaid {
    /// It names this profile's make (a profile's VIN prefix).
    Make(String),
    /// None given, or only filler (spaces, zeros, F's): a blank bench ECU.
    Blank,
    /// It could not be read (why).
    Unreadable(String),
    /// Read (masked here), but no profile's VIN prefix fits it: another make's donor, or one Icarus has no profile for.
    NoProfile(String),
}

/// The make a write follows, and where it came from.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WriteMake {
    /// A profile's id; None: not known, so a module of unknown kind is refused (it could be the airbag module).
    pub make: Option<String>,
    /// The bench make named for this session was used (the VIN gave none).
    pub chosen: bool,
    pub vin: VinSaid,
}

impl WriteMake {
    /// What the VIN said, in a few words.
    fn vin_words(&self) -> String {
        match &self.vin {
            VinSaid::Make(m) => format!("the VIN names {m}"),
            VinSaid::Blank => "VIN blank".into(),
            VinSaid::Unreadable(why) => format!("VIN unreadable ({why})"),
            VinSaid::NoProfile(v) => format!("VIN {v} names no profile"),
        }
    }

    /// As every confirmation shows it and the action log keeps it: "make chosen by me: toyota, VIN blank", "make from the
    /// VIN: toyota", or "make not known (VIN blank): ...".
    pub fn words(&self) -> String {
        match (&self.make, self.chosen) {
            (Some(m), true) => format!("make chosen by me: {m}, {}", self.vin_words()),
            (Some(m), false) => format!("make from the VIN: {m}"),
            (None, _) => format!(
                "make not known ({}): a module the profile does not name is refused; on a bench ECU, name its make \
                 (--make, or the window's Bench make)",
                self.vin_words()
            ),
        }
    }
}

/// Whether a VIN as read is only filler: empty, or one character repeated (spaces, zeros, F's, asterisks).
fn blank(vin: &str) -> bool {
    let v = vin.trim();
    let mut c = v.chars();
    match c.next() {
        None => true,
        Some(first) => {
            matches!(first, '0' | 'F' | 'f' | '*' | '-' | '.' | '\0' | ' ') && c.all(|x| x == first)
        }
    }
}

/// The make a write follows, from the VIN as read (`vin`) and the bench make named for this session (`chosen`): the VIN's
/// make when it names one, and then a named make must agree (else refused, the mismatch said); the named make only when
/// the VIN is blank, unreadable or decodes to no profile; else not known. A named make that is no profile's id is
/// refused.
pub fn for_vin(
    profiles: &[Profile],
    vin: Result<Option<String>, String>,
    chosen: Option<&str>,
) -> Result<WriteMake, String> {
    let chosen = chosen.map(|m| profile_id(profiles, m)).transpose()?;
    let said = match vin {
        Err(why) => VinSaid::Unreadable(why),
        Ok(None) => VinSaid::Blank,
        Ok(Some(v)) if blank(&v) => VinSaid::Blank,
        Ok(Some(v)) => match crate::vehicles::for_vin(profiles, &v) {
            Some(p) => VinSaid::Make(p.id.clone()),
            None => VinSaid::NoProfile(crate::vin::masked(v.trim())),
        },
    };
    if let VinSaid::Make(m) = &said {
        if let Some(c) = chosen.as_deref().filter(|c| !c.eq_ignore_ascii_case(m)) {
            return Err(format!(
                "refused: the VIN names {m}, and the make named for this session is {c}: they must agree (name {m}, or \
                 none, since the VIN gives it)"
            ));
        }
        return Ok(WriteMake {
            make: Some(m.clone()),
            chosen: false,
            vin: said,
        });
    }
    Ok(WriteMake {
        chosen: chosen.is_some(),
        make: chosen,
        vin: said,
    })
}

/// The make a write to the vehicle on `link` follows, now: its VIN read (nothing else is sent) and this session's bench
/// make (`chosen`). A mismatch is refused and logged; a bench make used is logged ("make chosen by me: ...").
pub fn for_write(
    link: &mut dyn crate::link::Link,
    profiles: &[Profile],
    what: &str,
) -> Result<WriteMake, String> {
    let vin = crate::j1979::vin(link).map_err(|e| e.to_string());
    match for_vin(profiles, vin, chosen().as_deref()) {
        Ok(m) => {
            if m.chosen {
                crate::audit::log(&format!("{what}: {}", m.words()));
            }
            Ok(m)
        }
        Err(why) => {
            crate::audit::log(&format!("{what} refused: {why}"));
            Err(why)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clear;
    use crate::elm::Elm;
    use crate::sim::SimCar;
    use crate::uds::Dialect;

    /// A simulated ECM on the bench, engine off, with this VIN ("" for a blank one).
    fn bench(vin: &str) -> Elm {
        let mut e = Elm::connect(Box::new(SimCar::engine_off().with_vin(vin))).unwrap();
        e.take_sent();
        e
    }

    #[test]
    fn what_the_vin_says_and_a_named_make() {
        let ps = crate::vehicles::builtin();
        let blank = for_vin(&ps, Ok(None), None).unwrap();
        assert_eq!((blank.make.as_deref(), blank.chosen), (None, false));
        assert!(blank.words().starts_with("make not known (VIN blank)"));
        let named = for_vin(&ps, Ok(None), Some("Toyota")).unwrap();
        assert_eq!(named.make.as_deref(), Some("toyota"));
        assert_eq!(named.words(), "make chosen by me: toyota, VIN blank");
        // zeros, spaces, F's: blank too
        for v in [
            "00000000000000000",
            "FFFFFFFFFFFFFFFFF",
            "                 ",
        ] {
            assert_eq!(
                for_vin(&ps, Ok(Some(v.into())), None).unwrap().vin,
                VinSaid::Blank
            );
        }
        let unread = for_vin(&ps, Err("no answer".into()), Some("toyota")).unwrap();
        assert_eq!(
            unread.words(),
            "make chosen by me: toyota, VIN unreadable (no answer)"
        );
        // a VIN no profile's prefix fits: the named make is used, the VIN masked in what is said
        let foreign = for_vin(&ps, Ok(Some("9ZZICARUSSIM00009".into())), Some("toyota")).unwrap();
        assert!(foreign.chosen);
        assert!(
            foreign.words().contains("names no profile"),
            "{}",
            foreign.words()
        );
        assert!(!foreign.words().contains("00009"));
        // the VIN names the make: it is used, a named one must agree
        let from_vin = for_vin(&ps, Ok(Some(crate::sim::VIN.into())), Some("toyota")).unwrap();
        assert_eq!(
            (from_vin.make.as_deref(), from_vin.chosen),
            (Some("toyota"), false)
        );
        let why = for_vin(&ps, Ok(Some(crate::sim::VIN.into())), Some("gm")).unwrap_err();
        assert!(
            why.contains("the VIN names toyota") && why.contains("gm"),
            "{why}"
        );
        // never a guess: a part of a name, or no profile, is refused
        assert!(for_vin(&ps, Ok(None), Some("toy")).is_err());
        assert!(for_vin(&ps, Ok(None), Some("tesla")).is_err());
    }

    /// ICA-222's acceptance: a simulated ECM with a blank VIN refuses a module write without a bench make, takes one with
    /// --make toyota on 7E0, still refuses 780 (the airbag module) and a key module, and a VIN that decodes to toyota
    /// refuses --make gm. Checked at the moment of the write (`caution_holds`, as a test, a setting and a routine ask it)
    /// and for one module's clear (`clear_module`).
    #[test]
    fn a_blank_vin_bench_ecm_takes_a_named_make_and_nothing_more() {
        crate::vehicles::profiles_for_test(None);
        // this test's own Read-Only list (none switched), not the config folder's
        crate::lock::switch_for_test(&[]);
        crate::fence::fence_for_test(Some(crate::fence::one_for_test(
            0x7D5,
            crate::fence::Kind::SmartKey,
        )));
        let ps = crate::vehicles::builtin();
        let hold =
            |e: &mut Elm, tx: u16| crate::vehicles::caution_holds(e, tx, tx + 8, "a setting");
        // without a make: refused, nothing sent but the VIN read
        choose(None);
        let mut e = bench("");
        let why = hold(&mut e, 0x7E0).unwrap_err();
        assert!(why.contains("could be the airbag module"), "{why}");
        assert!(
            why.contains("Bench make") || why.contains("--make"),
            "{why}"
        );
        assert!(e.take_sent().iter().all(|s| s.2 == [0x09, 0x02]));
        let make = for_write(&mut e, &ps, "clear 7E0").unwrap();
        let why = clear::clear_module(
            &mut e,
            0x7E0,
            0x7E8,
            Dialect::Uds,
            &ps,
            make.make.as_deref(),
        )
        .unwrap_err();
        assert!(why.contains("could be the airbag module"), "{why}");
        // --make toyota: 7E0 goes
        choose(Some("toyota"));
        hold(&mut e, 0x7E0).unwrap();
        let make = for_write(&mut e, &ps, "clear 7E0").unwrap();
        assert_eq!(make.words(), "make chosen by me: toyota, VIN blank");
        clear::clear_module(
            &mut e,
            0x7E0,
            0x7E8,
            Dialect::Uds,
            &ps,
            make.make.as_deref(),
        )
        .unwrap();
        // ...780 (the airbag module) never, and a key module never
        let why = hold(&mut e, 0x780).unwrap_err();
        assert!(why.contains("airbag"), "{why}");
        let why = clear::clear_module(&mut e, 0x780, 0x788, Dialect::Uds, &ps, Some("toyota"))
            .unwrap_err();
        assert!(why.contains("airbag"), "{why}");
        e.take_sent();
        let why = clear::clear_module(&mut e, 0x7D5, 0x7DD, Dialect::Uds, &ps, Some("toyota"))
            .unwrap_err();
        assert!(why.contains("key") || why.contains("smart"), "{why}");
        assert!(
            e.take_sent().iter().all(|s| s.1 != 0x7D5),
            "nothing to the key module"
        );
        // a VIN that decodes to toyota refuses --make gm, at the write and for a clear
        choose(Some("gm"));
        let mut t = bench(crate::sim::VIN);
        let why = hold(&mut t, 0x7E0).unwrap_err();
        assert!(
            why.contains("the VIN names toyota") && why.contains("gm"),
            "{why}"
        );
        assert!(for_write(&mut t, &ps, "clear 7E0").is_err());
        choose(None);
        crate::fence::fence_for_test(None);
    }
}
