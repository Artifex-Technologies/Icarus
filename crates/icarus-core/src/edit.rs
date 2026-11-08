//! Saving an edited image: always as a new file (never over the original or anything else), with every checksum Icarus can
//! trust corrected first (checksum.rs). What could not be corrected is said, so nobody mistakes the file for one that is
//! ready for a module.

use crate::checksum::{self, Spec};
use crate::rom::Rom;
use std::io::Write;
use std::path::{Path, PathBuf};

#[derive(Clone, Debug, PartialEq)]
pub struct Saved {
    pub path: PathBuf,
    /// Bytes that differ from the image as it was read (checksum corrections included).
    pub changed_bytes: usize,
    /// (name, old, new) for each checksum corrected.
    pub fixed: Vec<(String, u64, u64)>,
    /// Checksums whose description did not match the image as read: left as they were.
    pub untrusted: Vec<String>,
    /// No checksum description at all for this image.
    pub unknown: bool,
}

impl Saved {
    /// One line for people: what was saved and whether it could ever go to a module.
    pub fn summary(&self) -> String {
        let mut s = format!(
            "Saved {} ({} bytes changed)",
            self.path.display(),
            self.changed_bytes
        );
        if !self.fixed.is_empty() {
            s += &format!(
                "; checksums corrected: {}",
                self.fixed
                    .iter()
                    .map(|f| f.0.as_str())
                    .collect::<Vec<_>>()
                    .join(", ")
            );
        }
        if self.unknown {
            s += ". Its checksums are unknown (no description for this definition): a file to study, not to write to a module";
        } else if !self.untrusted.is_empty() {
            s += &format!(
                ". Not corrected (the description does not match the stock image): {}",
                self.untrusted.join(", ")
            );
        }
        s
    }
}

/// Where an edit of `image` goes by default: beside it as "<name>-edited.bin", or into "Custom tunes" when the image is in a
/// vehicle's "OEM tunes" folder; a free name either way.
pub fn default_path(image: &Path) -> PathBuf {
    let dir = image.parent().map(Path::to_path_buf).unwrap_or_default();
    let dir = if dir.file_name().is_some_and(|n| n == "OEM tunes") {
        dir.with_file_name("Custom tunes")
    } else {
        dir
    };
    let stem = image
        .file_stem()
        .map(|s| s.to_string_lossy().to_string())
        .unwrap_or_else(|| "image".into());
    let ext = image
        .extension()
        .map(|e| format!(".{}", e.to_string_lossy()))
        .unwrap_or_else(|| ".bin".into());
    let mut p = dir.join(format!("{stem}-edited{ext}"));
    let mut n = 2;
    while p.exists() {
        p = dir.join(format!("{stem}-edited ({n}){ext}"));
        n += 1;
    }
    p
}

/// Writes `rom` (checksums corrected) to `out`, which must not exist yet.
pub fn save(rom: &Rom, specs: &[Spec], out: &Path) -> Result<Saved, String> {
    let mut bytes = rom.bytes().to_vec();
    let fixed = checksum::fix(specs, rom.original(), &mut bytes);
    let status = checksum::check(specs, rom.original(), &bytes);
    let untrusted = status
        .iter()
        .filter(|s| !s.trusted)
        .map(|s| s.name.clone())
        .collect();
    if let Some(dir) = out.parent().filter(|d| !d.as_os_str().is_empty()) {
        std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    }
    // create_new: never replaces a file (the original, an OEM read included)
    let mut f = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(out)
        .map_err(|e| {
            if e.kind() == std::io::ErrorKind::AlreadyExists {
                format!(
                    "{} exists already: an edit is always saved as a new file",
                    out.display()
                )
            } else {
                format!("{}: {e}", out.display())
            }
        })?;
    f.write_all(&bytes).map_err(|e| e.to_string())?;
    let changed_bytes = rom
        .original()
        .iter()
        .zip(&bytes)
        .filter(|(a, b)| a != b)
        .count();
    Ok(Saved {
        path: out.to_path_buf(),
        changed_bytes,
        fixed,
        untrusted,
        unknown: specs.is_empty(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rom::Cell;

    fn tmp(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("icarus-edit-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    #[test]
    fn saves_new_files_only_with_checksums_corrected() {
        let d = tmp("save");
        let mut bytes = vec![0u8; 32];
        bytes[0..4].copy_from_slice(&[1, 2, 3, 4]);
        bytes[16] = 1 + 2 + 3 + 4; // a sum8 over 0..16 kept at 16, as the maker wrote it
        let specs = checksum::parse(
            "[[checksum]]\nname=\"cal\"\nalgorithm=\"sum8\"\nranges=[\"0-15\"]\nstore=\"16\"\n",
            32,
        )
        .unwrap();
        let mut rom = Rom::new(bytes);
        rom.write(
            2,
            Cell {
                bits: 8,
                signed: false,
                little_endian: false,
                float: false,
            },
            9.0,
        )
        .unwrap();
        let out = d.join("tune.bin");
        let saved = save(&rom, &specs, &out).unwrap();
        let written = std::fs::read(&out).unwrap();
        assert_eq!(written[16], 1 + 2 + 9 + 4, "the checksum follows the edit");
        assert_eq!(saved.fixed, vec![("cal".to_string(), 10, 16)]);
        assert_eq!(saved.changed_bytes, 2);
        assert!(!saved.unknown && saved.untrusted.is_empty());
        assert!(
            save(&rom, &specs, &out)
                .unwrap_err()
                .contains("exists already"),
            "never over a file"
        );
        let s = save(&rom, &[], &d.join("b.bin")).unwrap();
        assert!(s.unknown && s.summary().contains("not to write to a module"));
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn default_names() {
        let d = tmp("names");
        let oem = d.join("Truck").join("OEM tunes");
        std::fs::create_dir_all(&oem).unwrap();
        let img = oem.join("stock.bin");
        assert_eq!(
            default_path(&img),
            d.join("Truck")
                .join("Custom tunes")
                .join("stock-edited.bin")
        );
        let plain = d.join("ecu.bin");
        assert_eq!(default_path(&plain), d.join("ecu-edited.bin"));
        std::fs::write(d.join("ecu-edited.bin"), b"x").unwrap();
        assert_eq!(default_path(&plain), d.join("ecu-edited (2).bin"));
        let _ = std::fs::remove_dir_all(&d);
    }
}
