//! A module's as-built configuration block (FORScan's name for Ford's per-module configuration: data identifiers
//! DE00-DEFF, read with UDS `22`): read whole and saved before anything in it could ever be changed, so there is always a
//! way back. Read-only: only `22` requests go out, through `uds::read_data` (its read-only guard, the Read-Only backstop,
//! the pacing of a busy module). Reading stops at the first identifier the module does not have after the last one it had
//! (blocks are numbered from DE00 without gaps), or at DEFF.
//!
//! A sweep of up to 256 identifiers is a module-data read, so it passes the same gates (`moddata::Gates`): an airbag or
//! occupant module (by its ID, its name, or as any vehicle profile names it) is never swept, nor a module of unknown kind
//! where the vehicle's make names no airbag module.

use crate::elm::ObdError;
use crate::link::Link;
use crate::moddata::Gates;
use crate::uds::{self, DataAnswer};
use crate::vehicles::Profile;
use icarus_core::moddata::{Ids, ModuleDef, ModuleHead};
use std::path::{Path, PathBuf};

pub const FIRST: u16 = 0xDE00;
pub const LAST: u16 = 0xDEFF;

/// The block as read: (identifier, its bytes) in order.
pub type Block = Vec<(u16, Vec<u8>)>;

/// Why the module at `tx`/`rx` may not be swept, if it may not, on a vehicle of `make` (a profile's id; None: not known).
pub fn refused(profiles: &[Profile], make: Option<&str>, tx: u32, rx: u32) -> Option<String> {
    let m = ModuleDef {
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
    let gates = Gates::for_vehicle(profiles, std::slice::from_ref(&m), make.unwrap_or(""), make);
    gates.module_refused(&m)
}

/// Reads the module's as-built block: from DE00 on until the module refuses or stays silent (after at least one answer),
/// or DEFF. A module the gates refuse (`refused`) is sent nothing.
pub fn read(
    link: &mut dyn Link,
    tx: u32,
    rx: u32,
    profiles: &[Profile],
    make: Option<&str>,
) -> Result<Block, String> {
    if let Some(why) = refused(profiles, make, tx, rx) {
        return Err(format!(
            "{tx:03X}'s as-built block is not read ({why}); nothing was sent to it"
        ));
    }
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
    use crate::sim::SimCar;

    /// The airbag module is never swept, by its ID or as the profile names it, and a module of unknown kind is not swept
    /// where the make is not known: nothing reaches the link.
    #[test]
    fn the_airbag_module_is_never_swept() {
        use crate::sim::recording::Recorder;
        let profiles = crate::vehicles::builtin();
        let (rec, sent) = Recorder::new(SimCar::new());
        let mut elm = Elm::connect(Box::new(rec)).unwrap();
        let before = sent.lock().unwrap().len();
        let e = read(&mut elm, 0x780, 0x788, &profiles, Some("toyota")).unwrap_err();
        assert!(
            e.contains("airbag") && e.contains("nothing was sent"),
            "{e}"
        );
        let e = read(&mut elm, 0x7C0, 0x7C8, &profiles, None).unwrap_err();
        assert!(e.contains("could be the airbag module"), "{e}");
        assert_eq!(sent.lock().unwrap().len(), before, "no request went out");
        assert!(refused(&profiles, Some("toyota"), 0x7C0, 0x7C8).is_none());
    }

    #[test]
    fn a_block_read_saved_and_read_back() {
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        let profiles = crate::vehicles::builtin();
        let b = read(&mut elm, 0x7C0, 0x7C8, &profiles, Some("toyota")).unwrap();
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
        assert!(read(&mut elm, 0x7E1, 0x7E9, &profiles, Some("toyota"))
            .unwrap_err()
            .contains("no as-built block"));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
