//! A module's as-built configuration block (FORScan's name for Ford's per-module configuration: data identifiers
//! DE00-DEFF, read with UDS `22`): read whole and saved before anything in it could ever be changed, so there is always a
//! way back. Read-only: only `22` requests go out, through `uds::read_data` (its read-only guard, the Read-Only backstop,
//! the pacing of a busy module). Reading stops at the first identifier the module does not have after the last one it had
//! (blocks are numbered from DE00 without gaps), or at DEFF.
//!
//! A sweep of up to 256 identifiers no file lists, so it passes module data's sweep gate (`moddata::Gates::sweep`) before
//! anything is sent:
//! - an airbag or occupant module (by its ID, its name, or as any vehicle profile names it) is never swept, nor a module of
//!   unknown kind where the vehicle's make names no airbag module;
//! - a chassis or driver-assist module (ABS and stability control, brakes, steering, radar, cameras, cruise, lane and
//!   parking assistance; by its kind or name in its module-data file, or the vehicle profile's name for its address: ICA-100)
//!   gets identity only, unless its file in the module-data set chosen for the vehicle lists identifiers DE00-DEFF, which
//!   are then the only ones read.

use crate::elm::ObdError;
use crate::link::Link;
use crate::moddata::{Gates, Loaded, Sweep};
use crate::uds::{self, DataAnswer};
use crate::vehicles::Profile;
use icarus_core::moddata::{Ids, ModuleDef, ModuleHead};
use std::path::{Path, PathBuf};

pub const FIRST: u16 = 0xDE00;
pub const LAST: u16 = 0xDEFF;

/// The block as read: (identifier, its bytes) in order.
pub type Block = Vec<(u16, Vec<u8>)>;

/// What of the block of the module at `tx`/`rx` may be read on a vehicle of `make` (a profile's id; None: not known), with
/// `set`, the module-data set chosen for the vehicle (`Loaded::on_vehicle`), when there is one: its file for this module
/// is what a chassis or driver-assist module may be read by. Every identifier from DE00 on, only those the file lists, or
/// why nothing.
pub fn plan(
    profiles: &[Profile],
    make: Option<&str>,
    tx: u32,
    rx: u32,
    set: Option<&Loaded>,
) -> Result<Sweep, String> {
    let file = set
        .and_then(|s| s.module(tx))
        .filter(|m| m.module.ids == Ids::Can11);
    if let (Some(s), Some(f)) = (set, file) {
        if f.module.rx() != rx {
            return Err(format!(
                "the module-data set {:?} has {tx:03X} answering from {:03X}, not {rx:03X}",
                s.set.name,
                f.module.rx()
            ));
        }
    }
    let bare = ModuleDef {
        format: 1,
        module: ModuleHead {
            name: String::new(),
            tx,
            rx: Some(rx),
            ext: None,
            tester: None,
            ids: Ids::Can11,
            bus: None,
            dialect: None,
            kind: String::new(),
            identity: Vec::new(),
            protocol: None,
            header_byte: None,
        },
        requests: Vec::new(),
    };
    let gates = match set {
        Some(s) => Gates::for_vehicle(profiles, &s.modules, &s.set.make, make),
        None => Gates::for_vehicle(
            profiles,
            std::slice::from_ref(&bare),
            make.unwrap_or(""),
            make,
        ),
    };
    gates.sweep(file.unwrap_or(&bare), 0x22, FIRST..=LAST)
}

/// Reads the module's as-built block: from DE00 on until the module refuses or stays silent (after at least one answer),
/// or DEFF; or, for a chassis or driver-assist module, exactly the identifiers its module-data file lists (`plan`). A
/// module `plan` refuses is sent nothing.
pub fn read(
    link: &mut dyn Link,
    tx: u32,
    rx: u32,
    profiles: &[Profile],
    make: Option<&str>,
    set: Option<&Loaded>,
) -> Result<Block, String> {
    match plan(profiles, make, tx, rx, set) {
        Err(why) => Err(format!(
            "{tx:03X}'s as-built block is not read ({why}); nothing was sent to it"
        )),
        Ok(Sweep::Whole) => read_whole(link, tx, rx),
        Ok(Sweep::Listed(ids)) => read_listed(link, tx, rx, &ids),
    }
}

fn read_whole(link: &mut dyn Link, tx: u32, rx: u32) -> Result<Block, String> {
    let mut out = Vec::new();
    for did in FIRST..=LAST {
        match uds::read_data(link, tx, rx, 0x22, did) {
            Ok(DataAnswer::Data(d)) => out.push((did, d)),
            Ok(DataAnswer::Refused(c)) if out.is_empty() => {
                return Err(format!(
                    "{tx:03X} has no as-built block (DE00: {})",
                    uds::nrc(c)
                ))
            }
            Ok(DataAnswer::Silent) if out.is_empty() => {
                return Err(format!("{tx:03X} did not answer DE00"))
            }
            Ok(_) => break,
            Err(ObdError::NoData) if !out.is_empty() => break,
            Err(e) => return Err(e.to_string()),
        }
    }
    Ok(out)
}

/// Each identifier of `ids` asked once, in order: one the module does not have or does not answer is passed over, and a
/// module that refuses `22` altogether (not supported, or not in this session) is asked no more.
fn read_listed(link: &mut dyn Link, tx: u32, rx: u32, ids: &[u16]) -> Result<Block, String> {
    let (mut out, mut first) = (Vec::new(), None);
    for &did in ids {
        match uds::read_data(link, tx, rx, 0x22, did) {
            Ok(DataAnswer::Data(d)) => out.push((did, d)),
            Ok(DataAnswer::Refused(c)) => {
                first.get_or_insert(format!("{did:04X}: {}", uds::nrc(c)));
                if matches!(c, 0x11 | 0x7F) {
                    break;
                }
            }
            Ok(DataAnswer::Silent) | Err(ObdError::NoData) => {
                first.get_or_insert(format!("{did:04X}: no answer"));
            }
            Err(e) => return Err(e.to_string()),
        }
    }
    if out.is_empty() {
        return Err(format!(
            "{tx:03X} answered none of the {} identifiers its module-data file lists ({})",
            ids.len(),
            first.unwrap_or_default()
        ));
    }
    Ok(out)
}

/// The block as text, one identifier a line: `DE00: 0A 1B 2C`.
pub fn text(module: &str, when: &str, b: &Block) -> String {
    let mut o = format!(
        "# Icarus: as-built block of {module}, read {when}. The way back: keep this file.\n"
    );
    for (did, bytes) in b {
        let hex: Vec<String> = bytes.iter().map(|x| format!("{x:02X}")).collect();
        o += &format!("{did:04X}: {}\n", hex.join(" "));
    }
    o
}

/// Reads a saved block back (the lines `DE00: 0A 1B`); lines that are not one are skipped.
pub fn parse(t: &str) -> Block {
    t.lines()
        .filter_map(|l| {
            let (id, rest) = l.split_once(':')?;
            let did = u16::from_str_radix(id.trim(), 16).ok()?;
            let bytes = rest
                .split_whitespace()
                .map(|h| u8::from_str_radix(h, 16).ok())
                .collect::<Option<Vec<u8>>>()?;
            Some((did, bytes))
        })
        .collect()
}

/// Saves the block in `dir` (the vehicle's `Definitions`) as `as-built-<module>-<when>.txt`, never over an existing file.
pub fn save_in(dir: &Path, tx: u32, when: &str, b: &Block) -> Result<PathBuf, String> {
    std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let stamp: String = when
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() { c } else { '-' })
        .collect();
    let base = format!("as-built-{tx:03X}-{stamp}");
    let path = std::iter::once(dir.join(format!("{base}.txt")))
        .chain((2..).map(|n| dir.join(format!("{base} ({n}).txt"))))
        .find(|p| !p.exists())
        .unwrap();
    std::fs::write(&path, text(&format!("{tx:03X}"), when, b))
        .map_err(|e| format!("{}: {e}", path.display()))?;
    Ok(path)
}

/// Saves the block in the vehicle's folder (`Definitions`), or Icarus's state folder for the simulated vehicle.
pub fn save(vin: Option<&str>, tx: u32, b: &Block) -> Result<PathBuf, String> {
    let dir = match vin {
        Some(v) if v != crate::sim::VIN => crate::garage::ensure(v)
            .map_err(|e| format!("the vehicle's folder could not be made: {e}"))?
            .join("Definitions"),
        _ => crate::garage::logs_dir(vin)?,
    };
    save_in(&dir, tx, &crate::report::now(), b)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elm::Elm;
    use crate::sim::recording::Recorder;
    use crate::sim::SimCar;
    use std::sync::{Arc, Mutex};

    /// The simulated vehicle behind a recorded adapter: the link, and what was sent to the adapter (cleared after
    /// connecting).
    fn recorded(car: SimCar) -> (Elm, Arc<Mutex<Vec<String>>>) {
        let (rec, sent) = Recorder::new(car);
        let elm = Elm::connect(Box::new(rec)).unwrap();
        sent.lock().unwrap().clear();
        (elm, sent)
    }

    /// The requests (hex lines, not adapter commands) sent since the last call, and forgotten.
    fn requests(sent: &Mutex<Vec<String>>) -> Vec<String> {
        std::mem::take(&mut *sent.lock().unwrap())
            .into_iter()
            .map(|l| l.trim_end_matches('\r').to_string())
            .filter(|l| !l.is_empty() && l.chars().all(|c| c.is_ascii_hexdigit()))
            .collect()
    }

    /// A module-data set for the simulated Toyota with one module file for the ABS at 7B0 (the text after `[module]`'s
    /// lines: its requests), made up for the test.
    fn set_with_abs(requests: &str) -> Loaded {
        let files = vec![
            (
                "set.toml".to_string(),
                "format = 1\nname = \"test\"\nmake = \"toyota\"\n\n[source]\nfrom = \"hand\"\n"
                    .to_string(),
            ),
            (
                "7B0.toml".to_string(),
                format!("format = 1\n\n[module]\nname = \"ABS\"\ntx = 0x7B0\n\n{requests}"),
            ),
        ];
        let profiles = crate::vehicles::builtin();
        let set = crate::moddata::load_files(&files, "test", &profiles).unwrap();
        assert!(set.refused.is_empty(), "{:?}", set.refused);
        set.on_vehicle(&profiles, Some("toyota"))
    }

    /// The airbag module is never swept, by its ID or as the profile names it, and a module of unknown kind is not swept
    /// where the make is not known: nothing reaches the link.
    #[test]
    fn the_airbag_module_is_never_swept() {
        let profiles = crate::vehicles::builtin();
        let (mut elm, sent) = recorded(SimCar::new());
        let e = read(&mut elm, 0x780, 0x788, &profiles, Some("toyota"), None).unwrap_err();
        assert!(
            e.contains("airbag") && e.contains("nothing was sent"),
            "{e}"
        );
        let e = read(&mut elm, 0x7C0, 0x7C8, &profiles, None, None).unwrap_err();
        assert!(e.contains("could be the airbag module"), "{e}");
        assert!(sent.lock().unwrap().is_empty(), "no request went out");
        assert_eq!(
            plan(&profiles, Some("toyota"), 0x7C0, 0x7C8, None),
            Ok(Sweep::Whole)
        );
    }

    /// ICA-100: the ABS/VSC module (7B0) and the power steering (7A1) are chassis modules, by the Toyota profile's names
    /// for them: with no module-data file listing their identifiers they are sent nothing, and the reason says so. The
    /// airbag module is still sent nothing, and the cluster (7C0) is still read whole.
    #[test]
    fn chassis_modules_are_not_swept_and_say_why() {
        let profiles = crate::vehicles::builtin();
        let (mut elm, sent) = recorded(SimCar::new().with_block(0x7B0, 0x1F));
        for (tx, words) in [(0x7B0, "ABS / VSC / TRC"), (0x7A1, "power steering")] {
            let e = read(&mut elm, tx, tx + 8, &profiles, Some("toyota"), None).unwrap_err();
            assert!(
                e.contains("chassis and driver-assist")
                    && e.contains(words)
                    && e.contains("identity only")
                    && e.contains("nothing was sent"),
                "{e}"
            );
        }
        let e = read(&mut elm, 0x780, 0x788, &profiles, Some("toyota"), None).unwrap_err();
        assert!(e.contains("airbag"), "{e}");
        assert!(
            sent.lock().unwrap().is_empty(),
            "nothing reached the adapter"
        );
        // a set chosen for the vehicle that has no file for the ABS changes nothing, and one whose file for it lists only
        // other identifiers neither
        let set = set_with_abs("[[request]]\nservice = 0x21\nident = 0x20\n");
        let e = read(
            &mut elm,
            0x7B0,
            0x7B8,
            &profiles,
            Some("toyota"),
            Some(&set),
        )
        .unwrap_err();
        assert!(e.contains("none of 22 DE00-DEFF is listed"), "{e}");
        assert!(sent.lock().unwrap().is_empty());
        let b = read(
            &mut elm,
            0x7C0,
            0x7C8,
            &profiles,
            Some("toyota"),
            Some(&set),
        )
        .unwrap();
        assert_eq!(b.len(), 3, "the cluster is no chassis module: {b:?}");
    }

    /// ICA-100: a module file that lists DE00-DE0F for the ABS gets exactly those read, though the module keeps 32.
    #[test]
    fn a_chassis_module_gets_the_identifiers_its_file_lists() {
        let profiles = crate::vehicles::builtin();
        let listed: String = (0..=0x0Fu16)
            .map(|n| {
                format!(
                    "[[request]]\nservice = 0x22\nident = 0x{:04X}\n\n",
                    FIRST + n
                )
            })
            .collect();
        let set = set_with_abs(&listed);
        assert_eq!(
            plan(&profiles, Some("toyota"), 0x7B0, 0x7B8, Some(&set)),
            Ok(Sweep::Listed((FIRST..=FIRST + 0x0F).collect()))
        );
        let (mut elm, sent) = recorded(SimCar::new().with_block(0x7B0, 0x1F));
        let b = read(
            &mut elm,
            0x7B0,
            0x7B8,
            &profiles,
            Some("toyota"),
            Some(&set),
        )
        .unwrap();
        assert_eq!(
            b.iter().map(|x| x.0).collect::<Vec<_>>(),
            (FIRST..=FIRST + 0x0F).collect::<Vec<_>>()
        );
        let asked = requests(&sent);
        assert_eq!(
            asked,
            (0..=0x0Fu16)
                .map(|n| format!("22{:04X}", FIRST + n))
                .collect::<Vec<_>>(),
            "each listed identifier once, nothing else"
        );
        // a file that has the module answering elsewhere than asked is not taken for it
        let e = read(
            &mut elm,
            0x7B0,
            0x7B9,
            &profiles,
            Some("toyota"),
            Some(&set),
        )
        .unwrap_err();
        assert!(e.contains("answering from 7B8, not 7B9"), "{e}");
        assert!(requests(&sent).is_empty());
    }

    #[test]
    fn a_block_read_saved_and_read_back() {
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        let profiles = crate::vehicles::builtin();
        let b = read(&mut elm, 0x7C0, 0x7C8, &profiles, Some("toyota"), None).unwrap();
        assert_eq!(
            b.iter().map(|x| x.0).collect::<Vec<_>>(),
            [0xDE00, 0xDE01, 0xDE02],
            "until the first identifier the module does not have"
        );
        let dir = std::env::temp_dir().join(format!("icarus-asbuilt-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let p1 = save_in(&dir, 0x7C0, "2026-11-21 20:00", &b).unwrap();
        let p2 = save_in(&dir, 0x7C0, "2026-11-21 20:00", &b).unwrap();
        assert_ne!(p1, p2, "never over an earlier backup");
        assert_eq!(parse(&std::fs::read_to_string(&p1).unwrap()), b);
        assert!(
            read(&mut elm, 0x7E1, 0x7E9, &profiles, Some("toyota"), None)
                .unwrap_err()
                .contains("no as-built block")
        );
        let _ = std::fs::remove_dir_all(&dir);
    }
}
