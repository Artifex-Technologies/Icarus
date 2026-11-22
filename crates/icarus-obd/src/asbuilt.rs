//! A module's as-built configuration block (FORScan's name for Ford's per-module configuration: data identifiers
//! DE00-DEFF, read with UDS `22`): read whole and saved before anything in it could ever be changed, so there is always a
//! way back. Read-only: only `22` requests go out, through `uds::read_data` (its read-only guard, the Read-Only backstop,
//! the pacing of a busy module). Reading stops at the first identifier the module does not have after the last one it had
//! (blocks are numbered from DE00 without gaps), or at DEFF.

use crate::elm::ObdError;
use crate::link::Link;
use crate::uds::{self, DataAnswer};
use std::path::{Path, PathBuf};

pub const FIRST: u16 = 0xDE00;
pub const LAST: u16 = 0xDEFF;

/// The block as read: (identifier, its bytes) in order.
pub type Block = Vec<(u16, Vec<u8>)>;

/// Reads the module's as-built block: from DE00 on until the module refuses or stays silent (after at least one answer),
/// or DEFF.
pub fn read(link: &mut dyn Link, tx: u32, rx: u32) -> Result<Block, String> {
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

    #[test]
    fn a_block_read_saved_and_read_back() {
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        let b = read(&mut elm, 0x7C0, 0x7C8).unwrap();
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
        assert!(read(&mut elm, 0x7E1, 0x7E9)
            .unwrap_err()
            .contains("no as-built block"));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
