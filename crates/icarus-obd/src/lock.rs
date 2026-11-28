//! The Read-Only switch: a safety switch per vehicle (the Garage page, or `icarus garage read-only VIN on|off`), kept by VIN
//! in `~/.config/icarus/read-only.toml` on this computer only. With it on, Icarus reads that vehicle (codes, live data,
//! module values, reports, recordings) and clears, tests and changes nothing on it. Off, which is every vehicle's default,
//! the write paths work as they always have, each behind its own checks.
//!
//! Each write path (clearing in `clear.rs`, actuator tests in `actuator.rs`, settings in `settings.rs`, and any later one)
//! asks `writes_allowed` first. When any vehicle has the switch on, that reads the VIN from the vehicle on the port at that
//! moment, so the switch follows the vehicle, not the one the window connected to earlier; a VIN that cannot be read then
//! counts as switched on (it could be that vehicle). With no vehicle switched on, nothing is asked of the vehicle.
//!
//! Under the switch, the session floor (ICA-192), which does not depend on the VIN: a command or the window started with
//! `--read-only`, or with ICARUS_READ_ONLY=1 in its environment, clears, tests and changes nothing on any vehicle for the
//! whole run, whatever VIN it gives and whatever `read-only.toml` says. A donor ECM on the bench gives its donor's VIN, so
//! the switch alone does not cover it. `writes_allowed` asks the floor first, before the VIN is read, so the write paths
//! refuse with nothing sent, and the links (`Elm`, `CanLink`, `PassThruLink`, so the OpenPort and the Mongoose too) refuse
//! the same requests `changes_vehicle` names that they refuse for a switched vehicle. The floor is turned on, never off.

use crate::link::Link;
use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

/// The environment variable that turns the session floor on (`asks_read_only` says which values do).
pub const SESSION_ENV: &str = "ICARUS_READ_ONLY";

/// The flag that turns the session floor on, on the command line and in the window.
pub const SESSION_FLAG: &str = "--read-only";

#[cfg(not(test))]
static SESSION: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

#[cfg(test)]
thread_local! {
    /// A test's own session floor, instead of the one for the whole run (tests run side by side in one process).
    static SESSION_FOR_TEST: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Whether a value of ICARUS_READ_ONLY asks for the session floor: anything but empty, 0, no, false or off, so a value
/// mistyped asks for it too.
pub fn asks_read_only(value: &str) -> bool {
    !matches!(
        value.trim().to_ascii_lowercase().as_str(),
        "" | "0" | "no" | "false" | "off"
    )
}

/// Whether this run's environment asks for the session floor (ICARUS_READ_ONLY).
pub fn env_asks_read_only() -> bool {
    std::env::var_os(SESSION_ENV).is_some_and(|v| asks_read_only(&v.to_string_lossy()))
}

/// Turns the session floor on for the rest of this run (`--read-only`). Nothing turns it off.
pub fn hold_session_read_only() {
    #[cfg(test)]
    SESSION_FOR_TEST.with(|s| s.set(true));
    #[cfg(not(test))]
    SESSION.store(true, std::sync::atomic::Ordering::SeqCst);
}

/// Whether this run is read-only: `hold_session_read_only` was called, or ICARUS_READ_ONLY asks for it (kept once seen,
/// so the variable unset later does not turn the floor off).
#[cfg(not(test))]
pub fn session_read_only() -> bool {
    use std::sync::atomic::Ordering;
    if SESSION.load(Ordering::SeqCst) {
        return true;
    }
    if env_asks_read_only() {
        SESSION.store(true, Ordering::SeqCst);
        return true;
    }
    false
}

/// Whether this test's session is read-only (`hold_session_read_only` on its own thread; the environment is not asked).
#[cfg(test)]
pub fn session_read_only() -> bool {
    SESSION_FOR_TEST.with(|s| s.get())
}

/// The session floor's answer, asked before anything is sent to a vehicle: the reason when this run is read-only.
pub fn session_allows_changes() -> Result<(), String> {
    if !session_read_only() {
        return Ok(());
    }
    crate::audit::log("write refused: the session is read-only");
    Err(
        "session read-only: Icarus was started read-only this time (--read-only or ICARUS_READ_ONLY=1), so it clears, \
         tests and changes nothing on any vehicle, whatever VIN it gives"
            .into(),
    )
}

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

/// Switches `vins` to read-only for this test thread (instead of the file in the real config folder).
#[cfg(test)]
pub(crate) fn switch_for_test(vins: &[&str]) {
    FOR_TEST.with(|t| *t.borrow_mut() = Some(vins.iter().map(|v| v.to_string()).collect()));
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

/// Whether the vehicle on `link` may be written to now; the reason when the session is read-only (asked first, before
/// anything is sent) or the vehicle's Read-Only switch is on.
pub fn writes_allowed(link: &mut dyn Link) -> Result<(), String> {
    session_allows_changes()?;
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
        switch_for_test(vins);
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

    /// A raw write (2E write data, 31 routine control, 34 request download) to a switched vehicle, through every
    /// public way onto the bus each kind of link has (`request` and `request_to`, the link boxed as the command line holds
    /// it), is refused before it reaches the bus. None of them hands out what is under it to send on: what each one sent
    /// is looked at through its read-only accessor.
    #[test]
    fn a_raw_write_never_reaches_a_switched_vehicle_through_any_link() {
        use crate::canlink::{CanLink, SimBus};
        use crate::elm::ObdError;
        use crate::passthru::{PassThruLink, SimPassThru};
        switched(&[crate::sim::VIN]);
        let writes: [&[u8]; 3] = [
            &[0x2E, 0xA0, 0x01, 0x00],
            &[0x31, 0x01, 0xFF, 0x00],
            &[0x34, 0x00, 0x44, 0x00, 0x01, 0x00, 0x00],
        ];
        let is_write = |d: &[u8]| matches!(d.first(), Some(0x2E | 0x31 | 0x34));
        let refuses = |l: &mut dyn Link| {
            for w in writes {
                let to_all = l.request(w).unwrap_err();
                let to_one = l.request_to(0x7E0, 0x7E8, w).unwrap_err();
                for e in [to_all, to_one] {
                    assert!(matches!(e, ObdError::ReadOnly(_)), "{w:02X?}: {e}");
                }
            }
        };
        let mut elm = Elm::connect(Box::new(SimCar::engine_off())).unwrap();
        refuses(&mut elm);
        let sent = elm.take_sent();
        assert!(!sent.iter().any(|(_, _, d)| is_write(d)), "{sent:02X?}");
        let mut can = CanLink::connect(SimBus::new(SimCar::engine_off()), "vcan0").unwrap();
        refuses(&mut can);
        // single frames: the length, then the request
        let frames = &can.frames().sent;
        assert!(!frames.iter().any(|f| is_write(&f.data[1..])), "{frames:?}");
        let mut j2534 = PassThruLink::connect(SimPassThru::new(SimCar::engine_off())).unwrap();
        refuses(&mut j2534);
        let msgs = &j2534.device().sent;
        assert!(!msgs.iter().any(|m| is_write(&m.data)), "{msgs:?}");
        let mut boxed: Box<dyn Link> =
            Box::new(CanLink::connect(SimBus::new(SimCar::engine_off()), "vcan0").unwrap());
        refuses(&mut boxed);
        // the switch off: the same write reaches the vehicle, and is seen through the accessor
        switched(&[]);
        let _ = can.request_to(0x7C0, 0x7C8, &[0x2E, 0xA0, 0x01, 0x00]);
        assert!(can
            .frames()
            .sent
            .iter()
            .any(|f| f.id == 0x7C0 && is_write(&f.data[1..])));
    }

    /// A made-up VIN no vehicle is switched for: a donor ECM on the bench gives its donor's.
    const DONOR: &str = "5TFICARUSDONOR001";

    #[test]
    fn what_icarus_read_only_asks_for() {
        for on in ["1", "yes", "TRUE", "on", " 1 ", "y", "2"] {
            assert!(asks_read_only(on), "{on:?}");
        }
        for off in ["", "0", "no", "False", "OFF", "  "] {
            assert!(!asks_read_only(off), "{off:?}");
        }
    }

    /// ICA-192: with no vehicle switched and a donor VIN on the port, the session floor refuses a clear (every module and
    /// one), an actuator test, a setting change and a service routine, each with 'session read-only' and before any frame
    /// goes out (not even the VIN read the switch would make).
    #[test]
    fn the_session_floor_refuses_every_write_path_whatever_the_vin() {
        switched(&[]);
        hold_session_read_only();
        let car = || {
            let mut c = Elm::connect(Box::new(SimCar::engine_off().with_vin(DONOR))).unwrap();
            c.take_sent();
            c
        };
        let refused = |c: &mut Elm, r: Result<(), String>| {
            let e = r.unwrap_err();
            assert!(e.starts_with("session read-only"), "{e}");
            assert!(c.take_sent().is_empty(), "{e}: nothing went out");
        };
        let mut c = car();
        let r = crate::clear::clear_obd(&mut c).map(|_| ());
        refused(&mut c, r);
        let mut c = car();
        let r = crate::clear::clear_module(&mut c, 0x7E0, 0x7E8, crate::uds::Dialect::Uds);
        refused(&mut c, r);
        let mut c = car();
        let t = crate::sim::tests().remove(0);
        let r = crate::actuator::start(&mut c, &t, 30.0).map(|_| ());
        refused(&mut c, r);
        let mut c = car();
        let s = crate::sim::settings().remove(0);
        let v = s.values.values().next().unwrap().clone();
        let log = std::env::temp_dir().join(format!("icarus-session-log-{}", std::process::id()));
        let r = crate::settings::change(&mut c, &s, &v, &log).map(|_| ());
        refused(&mut c, r);
        assert!(!log.exists(), "nothing was logged as changed");
        let mut c = car();
        let srv = crate::sim::services().remove(0);
        let r = crate::service::run(&mut c, &srv, true).map(|_| ());
        refused(&mut c, r);
    }

    /// ICA-192 on one link, the session floor on: each request `changes_vehicle` names is refused with 'session
    /// read-only' through `request` and `request_to` and puts nothing on the bus (`went_out` counts what went out since it
    /// was last asked); the reads, tester present and the default session still go.
    fn the_floor_holds_on<L: Link>(l: &mut L, went_out: &mut dyn FnMut(&mut L) -> usize) {
        use crate::elm::ObdError;
        use crate::j1979::{self, DtcKind};
        let changes: [&[u8]; 8] = [
            &[0x04],
            &[0x14, 0xFF, 0xFF, 0xFF],
            &[0x10, 0x03],
            &[0x27, 0x01],
            &[0x2E, 0xA0, 0x01, 0x00],
            &[0x2F, 0x01, 0x23, 0x03, 0x50],
            &[0x31, 0x01, 0xFF, 0x00],
            &[0x11, 0x01],
        ];
        went_out(l);
        for c in changes {
            assert!(changes_vehicle(c), "{c:02X?}");
            for e in [
                l.request(c).unwrap_err(),
                l.request_to(0x7E0, 0x7E8, c).unwrap_err(),
            ] {
                assert!(
                    matches!(&e, ObdError::ReadOnly(m) if m.starts_with("session read-only")),
                    "{c:02X?}: {e}"
                );
            }
        }
        assert_eq!(went_out(l), 0, "nothing went onto the bus");
        assert_eq!(
            j1979::vin(l).unwrap().as_deref(),
            Some(DONOR),
            "the VIN reads"
        );
        assert!(j1979::read(l, 0x0C).is_ok(), "live data reads");
        assert!(j1979::supported(l).is_ok());
        for kind in [DtcKind::Stored, DtcKind::Pending, DtcKind::Permanent] {
            assert!(j1979::dtcs(l, kind).is_ok(), "{kind:?} codes read");
        }
        assert!(j1979::calibration_ids(l).is_ok());
        assert!(
            crate::uds::module_dtcs(l, 0x7E0, 0x7E8).is_ok(),
            "a module's codes read"
        );
        assert!(crate::uds::module_identity_notes(l, 0x7E0, 0x7E8).is_ok());
        assert!(
            l.request_to(0x7E0, 0x7E8, &[0x3E, 0x00]).is_ok(),
            "tester present"
        );
        assert!(
            l.request_to(0x7E0, 0x7E8, &[0x10, 0x01]).is_ok(),
            "the default session"
        );
        assert!(went_out(l) > 0, "the reads went out");
    }

    /// ICA-192 on every kind of link (the ELM327, SocketCAN and the Mongoose's CanLink, J2534 and so the OpenPort, and a
    /// link boxed as the command line holds it), the donor VIN on the port and no vehicle switched.
    #[test]
    fn the_session_floor_holds_on_every_link_and_reads_still_go() {
        use crate::canlink::{CanLink, SimBus};
        use crate::passthru::{PassThruLink, SimPassThru};
        switched(&[]);
        hold_session_read_only();
        let donor = || SimCar::engine_off().with_vin(DONOR);
        let mut elm = Elm::connect(Box::new(donor())).unwrap();
        the_floor_holds_on(&mut elm, &mut |e| e.take_sent().len());
        let mut seen = 0;
        let mut can = CanLink::connect(SimBus::new(donor()), "vcan0").unwrap();
        the_floor_holds_on(&mut can, &mut |l| {
            let n = l.frames().sent.len();
            std::mem::replace(&mut seen, n).abs_diff(n)
        });
        let mut seen = 0;
        let mut j2534 = PassThruLink::connect(SimPassThru::new(donor())).unwrap();
        the_floor_holds_on(&mut j2534, &mut |l| {
            let n = l.device().sent.len();
            std::mem::replace(&mut seen, n).abs_diff(n)
        });
        let mut seen = 0;
        let mut boxed = Box::new(CanLink::connect(SimBus::new(donor()), "vcan0").unwrap());
        the_floor_holds_on(&mut boxed, &mut |l| {
            let n = l.frames().sent.len();
            std::mem::replace(&mut seen, n).abs_diff(n)
        });
    }

    /// Without the floor (every test's start), the same donor vehicle takes a change as before: the floor alone refused it.
    #[test]
    fn without_the_floor_nothing_changes() {
        switched(&[]);
        assert!(!session_read_only());
        assert!(session_allows_changes().is_ok());
        let mut car = Elm::connect(Box::new(SimCar::engine_off().with_vin(DONOR))).unwrap();
        assert!(writes_allowed(&mut car).is_ok());
        assert!(!matches!(
            car.request(&[0x04]),
            Err(crate::elm::ObdError::ReadOnly(_))
        ));
        hold_session_read_only();
        assert!(writes_allowed(&mut car)
            .unwrap_err()
            .starts_with("session read-only"));
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
        let r = crate::sim::services().remove(0);
        read_only(crate::service::run(&mut car(), &r, true).map(|_| ()));
        assert!(!log.exists(), "nothing was logged as changed");
    }
}
