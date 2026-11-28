//! The make a bench ECU's session names (ICA-222). The airbag caution (`vehicles::caution_holds`, `clear::refused_on`)
//! takes the vehicle's make from the VIN it reads before every write; a spare ECM on the bench with a blank VIN gives no
//! make, so every module write to it would be refused (mode 04 still works). So a connection may be given its make once:
//! `--make toyota` on the commands that write (`obd clear`, `test`, `service`, `setting`, `setting-undo`), and the
//! window's "Bench make" in its connection row. It fails closed:
//!
//! - it is kept only for a connection whose VIN read blank when it was named (`bind`): no VIN, all filler (spaces, 0s,
//!   F's) or not 17 characters. A well-formed VIN, a VIN that could not be read, or one naming no profile is not blank:
//!   the make is refused then, and the caution stays as it is;
//! - it goes with the connection (`Link::bench_make`): a reconnect, another adapter, another vehicle start without it;
//! - at each write the VIN is read again (`for_vin`): blank, or unreadable on a connection that was blank, takes the
//!   bench make; a VIN that names a make must agree with it, or the write is refused, the mismatch said; one that names
//!   no profile is refused;
//! - it must be a profile's own id (`icarus vehicles` lists them), nothing looser;
//! - it is shown in every confirmation and written to the action log ("make chosen by me: toyota, VIN blank");
//! - it only picks the profile the caution follows: it never lifts the airbag list or the key modules' fence, which do
//!   not depend on the make.

use crate::link::Link;
use crate::vehicles::Profile;

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
    /// None given, only filler (spaces, zeros, F's), or not 17 characters: a blank bench ECU.
    Blank,
    /// It could not be read (why).
    Unreadable(String),
    /// A well-formed VIN (masked here) that no profile's prefix fits: another make's, or one Icarus has no profile for.
    NoProfile(String),
}

/// The make a write follows, and where it came from.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WriteMake {
    /// A profile's id; None: not known, so a module of unknown kind is refused (it could be the airbag module).
    pub make: Option<String>,
    /// The bench make kept for this connection was used (the VIN gave none).
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
            (None, _) if matches!(self.vin, VinSaid::Blank) => format!(
                "make not known ({}): a module the profile does not name is refused; on a bench ECU, name its make \
                 (--make, or the window's Bench make before connecting)",
                self.vin_words()
            ),
            (None, _) => format!(
                "make not known ({}): a module the profile does not name is refused",
                self.vin_words()
            ),
        }
    }
}

/// Whether a VIN as read is blank: not 17 characters (none, or a partial one), or one character repeated (spaces, zeros,
/// F's, asterisks).
pub fn blank(vin: &str) -> bool {
    let v = vin.trim();
    if v.chars().count() != 17 {
        return true;
    }
    let mut c = v.chars();
    let first = c.next().unwrap_or(' ');
    matches!(first, '0' | 'F' | 'f' | '*' | '-' | '.' | '\0') && c.all(|x| x == first)
}

/// What a VIN as read says (`vin`: the read's result).
fn said(profiles: &[Profile], vin: Result<Option<String>, String>) -> VinSaid {
    match vin {
        Err(why) => VinSaid::Unreadable(why),
        Ok(None) => VinSaid::Blank,
        Ok(Some(v)) if blank(&v) => VinSaid::Blank,
        Ok(Some(v)) => match crate::vehicles::for_vin(profiles, &v) {
            Some(p) => VinSaid::Make(p.id.clone()),
            None => VinSaid::NoProfile(crate::vin::masked(v.trim())),
        },
    }
}

/// Why a bench make is refused for a VIN that names no profile.
fn no_profile(v: &str) -> String {
    format!(
        "refused: this VIN ({v}) names no profile; a bench make (--make) is only for an ECM with a blank VIN, so the \
         caution stays (a module the profile does not name could be the airbag module)"
    )
}

/// The make a write follows, from the VIN as read now (`vin`) and the bench make kept for this connection (`bench`, kept
/// only when its VIN was blank: `bind`): the VIN's make when it names one, and then the bench make must agree (else
/// refused, the mismatch said); the bench make when the VIN is blank or could not be read; a VIN that names no profile
/// refuses a bench make (and without one, the make is not known).
pub fn for_vin(
    profiles: &[Profile],
    vin: Result<Option<String>, String>,
    bench: Option<&str>,
) -> Result<WriteMake, String> {
    let vin = said(profiles, vin);
    match (&vin, bench) {
        (VinSaid::Make(m), Some(b)) if !b.eq_ignore_ascii_case(m) => Err(format!(
            "refused: the VIN names {m}, and the make named for this bench ECU is {b}: they must agree"
        )),
        (VinSaid::Make(m), _) => Ok(WriteMake {
            make: Some(m.clone()),
            chosen: false,
            vin,
        }),
        (VinSaid::NoProfile(v), Some(_)) => Err(no_profile(v)),
        (VinSaid::Blank | VinSaid::Unreadable(_), Some(b)) => Ok(WriteMake {
            make: Some(b.to_string()),
            chosen: true,
            vin,
        }),
        (_, None) => Ok(WriteMake {
            make: None,
            chosen: false,
            vin,
        }),
    }
}

/// Names `make` (a profile's own id) for the bench ECU on `link`, for this connection only: its VIN is read now and must
/// be blank (ICA-222). A VIN that names the same make needs none (it is not kept, the VIN gives it); any other VIN, or
/// one that cannot be read, refuses it. Returns the make as writes will follow it.
pub fn bind(link: &mut dyn Link, profiles: &[Profile], make: &str) -> Result<WriteMake, String> {
    let id = profile_id(profiles, make)?;
    let vin = said(profiles, crate::j1979::vin(link).map_err(|e| e.to_string()));
    let refused = |why: String| {
        crate::audit::log(&format!("bench make {id} refused: {why}"));
        Err(why)
    };
    match vin {
        VinSaid::Blank => {
            if !link.keep_bench_make(Some(id.clone())) {
                return refused(
                    "refused: this kind of link does not keep a bench make; a bench ECU's module writes need an \
                     ELM327 or OBDLink adapter"
                        .into(),
                );
            }
            let m = WriteMake {
                make: Some(id),
                chosen: true,
                vin,
            };
            crate::audit::log(&m.words());
            Ok(m)
        }
        VinSaid::Make(m) if m.eq_ignore_ascii_case(&id) => Ok(WriteMake {
            make: Some(m),
            chosen: false,
            vin: VinSaid::Make(id),
        }),
        VinSaid::Make(m) => refused(format!(
            "refused: the VIN names {m}, and --make says {id}: they must agree (a bench make is only for an ECM with a \
             blank VIN)"
        )),
        VinSaid::NoProfile(v) => refused(no_profile(&v)),
        VinSaid::Unreadable(why) => refused(format!(
            "refused: the VIN could not be read ({why}); a bench make is only for an ECM whose VIN reads blank"
        )),
    }
}

/// The make a write to the vehicle on `link` follows, now: its VIN read (nothing else is sent) and the bench make kept for
/// this connection. A refusal is logged; a bench make used is logged ("make chosen by me: ...").
pub fn for_write(
    link: &mut dyn Link,
    profiles: &[Profile],
    what: &str,
) -> Result<WriteMake, String> {
    let vin = crate::j1979::vin(link).map_err(|e| e.to_string());
    match for_vin(profiles, vin, link.bench_make().as_deref()) {
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

    /// A well-formed VIN of a make no built-in profile has (ZZF: no real maker's prefix; the I makes it no real VIN).
    const FRIEND: &str = "ZZFICARUSFRND0001";

    #[test]
    fn what_the_vin_says_and_a_kept_make() {
        let ps = crate::vehicles::builtin();
        let blank = for_vin(&ps, Ok(None), None).unwrap();
        assert_eq!((blank.make.as_deref(), blank.chosen), (None, false));
        assert!(blank.words().starts_with("make not known (VIN blank)"));
        let named = for_vin(&ps, Ok(None), Some("toyota")).unwrap();
        assert_eq!(named.words(), "make chosen by me: toyota, VIN blank");
        // zeros, F's, a partial VIN: blank too
        for v in [
            "00000000000000000",
            "FFFFFFFFFFFFFFFFF",
            "5TFICAR",
            "                 ",
        ] {
            assert!(blank_vin(&ps, v), "{v:?}");
        }
        assert!(!blank_vin(&ps, crate::sim::VIN) && !blank_vin(&ps, FRIEND));
        // unreadable on a connection that read blank (the make is kept only then): the kept make
        let unread = for_vin(&ps, Err("no answer".into()), Some("toyota")).unwrap();
        assert_eq!(
            unread.words(),
            "make chosen by me: toyota, VIN unreadable (no answer)"
        );
        // a well-formed VIN no profile fits: not blank, so the make is refused, the VIN masked in what is said
        let why = for_vin(&ps, Ok(Some(FRIEND.into())), Some("toyota")).unwrap_err();
        assert!(
            why.contains("names no profile") && !why.contains("0001"),
            "{why}"
        );
        let none = for_vin(&ps, Ok(Some(FRIEND.into())), None).unwrap();
        assert_eq!(none.make, None);
        // the VIN names the make: it is used, a kept one must agree
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
        // never a guess
        assert!(profile_id(&ps, "toy").is_err() && profile_id(&ps, "tesla").is_err());
        assert_eq!(profile_id(&ps, "Toyota").unwrap(), "toyota");
    }

    fn blank_vin(ps: &[Profile], v: &str) -> bool {
        for_vin(ps, Ok(Some(v.into())), None).unwrap().vin == VinSaid::Blank
    }

    /// Binding: kept only on a connection whose VIN reads blank; a well-formed VIN (with or without a profile), or one
    /// that disagrees, refuses it; a new connection starts without it.
    #[test]
    fn a_bench_make_is_kept_only_for_a_blank_vin_and_goes_with_the_connection() {
        let ps = crate::vehicles::builtin();
        let mut e = bench("");
        assert!(bind(&mut e, &ps, "toy").is_err(), "never a guess");
        assert_eq!(e.bench_make(), None);
        assert!(bind(&mut e, &ps, "toyota").unwrap().chosen);
        assert_eq!(e.bench_make().as_deref(), Some("toyota"));
        // the same bench ECU connected again: a new connection, no make until it is named again
        assert_eq!(bench("").bench_make(), None);
        // a partial VIN is blank
        assert!(bind(&mut bench("5TFICAR"), &ps, "toyota").unwrap().chosen);
        // a friend's car of a make with no profile: refused, nothing kept
        let mut f = bench(FRIEND);
        let why = bind(&mut f, &ps, "toyota").unwrap_err();
        assert!(
            why.contains("names no profile") && why.contains("blank VIN"),
            "{why}"
        );
        assert_eq!(f.bench_make(), None);
        // a VIN that names toyota: --make gm refused, --make toyota not kept (the VIN gives it)
        let mut t = bench(crate::sim::VIN);
        assert!(bind(&mut t, &ps, "gm")
            .unwrap_err()
            .contains("the VIN names toyota"));
        assert!(!bind(&mut t, &ps, "toyota").unwrap().chosen);
        assert_eq!(t.bench_make(), None);
    }

    /// ICA-222's acceptance: a simulated ECM with a blank VIN refuses a module write without a bench make, takes one with
    /// --make toyota on 7E0, still refuses 780 (the airbag module) and a key module, and a VIN that decodes to toyota
    /// refuses --make gm. Checked at the moment of the write (`caution_holds`, as a test, a setting and a routine ask it)
    /// and for one module's clear (`clear_module`). A friend's car with a well-formed VIN of a make with no profile, on
    /// a link that somehow kept the make, still has the caution.
    #[test]
    fn a_blank_vin_bench_ecm_takes_a_named_make_and_nothing_more() {
        crate::vehicles::profiles_for_test(None);
        crate::lock::switch_for_test(&[]);
        crate::fence::fence_for_test(Some(crate::fence::one_for_test(
            0x7D5,
            crate::fence::Kind::SmartKey,
        )));
        let ps = crate::vehicles::builtin();
        let hold =
            |e: &mut Elm, tx: u16| crate::vehicles::caution_holds(e, tx, tx + 8, "a setting");
        // without a make: refused, nothing sent but the VIN read
        let mut e = bench("");
        let why = hold(&mut e, 0x7E0).unwrap_err();
        assert!(why.contains("could be the airbag module"), "{why}");
        assert!(why.contains("Bench make"), "{why}");
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
        bind(&mut e, &ps, "toyota").unwrap();
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
        // a VIN that decodes to toyota refuses --make gm
        let mut t = bench(crate::sim::VIN);
        assert!(bind(&mut t, &ps, "gm").is_err());
        // even with gm kept on its link (as if the VIN had changed under it), the write is refused
        t.keep_bench_make(Some("gm".into()));
        let why = hold(&mut t, 0x7E0).unwrap_err();
        assert!(
            why.contains("the VIN names toyota") && why.contains("gm"),
            "{why}"
        );
        // a friend's car with a well-formed VIN of a make with no profile: toyota kept on the link is refused at the
        // write, and without it the caution holds; nothing is sent to the module either way
        let mut f = bench(FRIEND);
        let why = hold(&mut f, 0x7E0).unwrap_err();
        assert!(why.contains("could be the airbag module"), "{why}");
        f.keep_bench_make(Some("toyota".into()));
        let why = hold(&mut f, 0x7E0).unwrap_err();
        assert!(why.contains("names no profile"), "{why}");
        assert!(f.take_sent().iter().all(|s| s.2 == [0x09, 0x02]));
        crate::fence::fence_for_test(None);
    }
}
