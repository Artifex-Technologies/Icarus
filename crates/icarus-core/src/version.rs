//! Versions of a tune: "stock" is the original image, never changed; every other version is a name and a set of changes
//! against that original, kept as data, not as another image.
//!
//! A version is a small TOML file, `<image> - <version>.version.toml`, in the folder edits of the image go into (the
//! vehicle's "Custom tunes" when the image is in its "OEM tunes", else beside the image; `edit::save_dir`), and is never
//! written over another file. It names the original it belongs to by size and CRC-32, and holds each changed run of bytes:
//! where it starts, the original's bytes there and the version's. The full image is the original with those runs put in
//! (`apply`), each run's original bytes checked first, so a version never lands on another image. Checksums are not part of
//! a version: they are corrected when an image is made from it (edit.rs), as for any edit.
//!
//! ```toml
//! # Icarus tune version: changes against the original image, which stays as it is
//! name = "Cold start"
//! note = "richer below 20 degC"
//! original = "stock.bin"           # the file it was made from (for people: the size and CRC-32 identify it)
//! original_size = 524288
//! original_crc32 = "0x1A2B3C4D"
//!
//! [[change]]
//! at = "0x1F40"                    # file offset of the first changed byte
//! was = "0A0B"                     # the original's bytes there (hex)
//! now = "0C0D"                     # the version's
//! elements = ["Warm-up enrichment"] # what the definition calls those bytes (for people; not used to apply)
//! ```

use crate::checksum::crc32;
use crate::def::Definition;
use crate::rom::Rom;
use serde::{Deserialize, Serialize};
use std::io::Write;
use std::path::{Path, PathBuf};

/// The end of a version file's name.
pub const SUFFIX: &str = ".version.toml";

#[derive(Clone, Debug, PartialEq)]
pub struct Version {
    pub name: String,
    pub note: String,
    /// The original's file name when the version was made (for people).
    pub original: String,
    pub original_size: usize,
    pub original_crc32: u32,
    /// In order, none overlapping.
    pub changes: Vec<Change>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Change {
    pub offset: usize,
    pub was: Vec<u8>,
    pub now: Vec<u8>,
    /// What the definition called these bytes when the version was saved.
    pub elements: Vec<String>,
}

#[derive(Serialize, Deserialize)]
struct File {
    name: String,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    note: String,
    #[serde(default)]
    original: String,
    original_size: usize,
    original_crc32: String,
    #[serde(default, rename = "change")]
    changes: Vec<FileChange>,
}

#[derive(Serialize, Deserialize)]
struct FileChange {
    at: String,
    was: String,
    now: String,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    elements: Vec<String>,
}

/// The CRC-32 that names an original image.
pub fn image_crc(bytes: &[u8]) -> u32 {
    crc32(bytes.iter().copied()) as u32
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02X}")).collect()
}

fn unhex(s: &str) -> Result<Vec<u8>, String> {
    let s = s.trim();
    if !s.len().is_multiple_of(2) || !s.chars().all(|c| c.is_ascii_hexdigit()) {
        return Err(format!("\"{s}\" is not bytes in hex"));
    }
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).map_err(|e| e.to_string()))
        .collect()
}

fn number(s: &str) -> Option<u64> {
    let s = s.trim();
    match s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        Some(h) => u64::from_str_radix(h, 16).ok(),
        None => s.parse().ok(),
    }
}

/// A version's name as part of a file name: no slashes or control characters, at most 80 characters.
fn checked_name(name: &str) -> Result<String, String> {
    let n = name.trim();
    if n.is_empty() {
        return Err("a version needs a name".into());
    }
    if n.chars().any(|c| c == '/' || c == '\\' || c.is_control()) || n.starts_with('.') {
        return Err(format!(
            "\"{n}\": a version's name cannot hold slashes or start with a dot"
        ));
    }
    if n.chars().count() > 80 {
        return Err("a version's name is at most 80 characters".into());
    }
    Ok(n.to_string())
}

impl Version {
    /// The version that turns `original` into `edited` (the same size): each run of changed bytes, named by `def` if given.
    pub fn between(
        name: &str,
        note: &str,
        original_name: &str,
        original: &[u8],
        edited: &[u8],
        def: Option<&Definition>,
    ) -> Result<Version, String> {
        let name = checked_name(name)?;
        if original.len() != edited.len() {
            return Err(format!(
                "the images differ in size (0x{:X} and 0x{:X} bytes): a version only changes bytes",
                original.len(),
                edited.len()
            ));
        }
        let changes = crate::rom::diff(original, edited)
            .into_iter()
            .map(|r| Change {
                offset: r.start,
                was: original[r.clone()].to_vec(),
                now: edited[r.clone()].to_vec(),
                elements: def
                    .map(|d| d.covering(r).iter().map(|e| e.title.clone()).collect())
                    .unwrap_or_default(),
            })
            .collect();
        Ok(Version {
            name,
            note: note.trim().to_string(),
            original: original_name.to_string(),
            original_size: original.len(),
            original_crc32: image_crc(original),
            changes,
        })
    }

    pub fn parse(text: &str) -> Result<Version, String> {
        let f: File = toml::from_str(text).map_err(|e| format!("not a version file: {e}"))?;
        let crc = number(&f.original_crc32)
            .and_then(|c| u32::try_from(c).ok())
            .ok_or_else(|| format!("original_crc32 \"{}\" is not a CRC-32", f.original_crc32))?;
        let mut changes = Vec::with_capacity(f.changes.len());
        let mut end = 0usize;
        for c in f.changes {
            let offset = number(&c.at)
                .and_then(|a| usize::try_from(a).ok())
                .ok_or_else(|| format!("change at \"{}\": not an offset", c.at))?;
            let (was, now) = (unhex(&c.was)?, unhex(&c.now)?);
            if was.is_empty() || was.len() != now.len() {
                return Err(format!(
                    "change at 0x{offset:X}: was and now must be the same number of bytes, at least one"
                ));
            }
            if offset < end || offset + was.len() > f.original_size {
                return Err(format!(
                    "change at 0x{offset:X} overlaps the one before it or runs past the image"
                ));
            }
            end = offset + was.len();
            changes.push(Change {
                offset,
                was,
                now,
                elements: c.elements,
            });
        }
        Ok(Version {
            name: checked_name(&f.name)?,
            note: f.note,
            original: f.original,
            original_size: f.original_size,
            original_crc32: crc,
            changes,
        })
    }

    pub fn to_toml(&self) -> String {
        let f = File {
            name: self.name.clone(),
            note: self.note.clone(),
            original: self.original.clone(),
            original_size: self.original_size,
            original_crc32: format!("0x{:08X}", self.original_crc32),
            changes: self
                .changes
                .iter()
                .map(|c| FileChange {
                    at: format!("0x{:X}", c.offset),
                    was: hex(&c.was),
                    now: hex(&c.now),
                    elements: c.elements.clone(),
                })
                .collect(),
        };
        format!(
            "# Icarus tune version: changes against the original image, which stays as it is\n{}",
            toml::to_string(&f).unwrap_or_default()
        )
    }

    /// Bytes changed in all.
    pub fn changed_bytes(&self) -> usize {
        self.changes.iter().map(|c| c.now.len()).sum()
    }

    /// Whether this version was made from `original` (its size, CRC-32 and every changed run's original bytes).
    pub fn fits(&self, original: &[u8]) -> Result<(), String> {
        if original.len() != self.original_size {
            return Err(format!(
                "version \"{}\" belongs to an image of 0x{:X} bytes; this one has 0x{:X}",
                self.name,
                self.original_size,
                original.len()
            ));
        }
        if image_crc(original) != self.original_crc32 {
            return Err(format!(
                "version \"{}\" was made from another image ({}, CRC-32 0x{:08X})",
                self.name, self.original, self.original_crc32
            ));
        }
        for c in &self.changes {
            if original.get(c.offset..c.offset + c.was.len()) != Some(c.was.as_slice()) {
                return Err(format!(
                    "version \"{}\": the original's bytes at 0x{:X} are not what the version says they were",
                    self.name, c.offset
                ));
            }
        }
        Ok(())
    }

    /// The full image of this version: `original` with the version's changes put in (the original itself is not touched).
    pub fn apply(&self, original: &[u8]) -> Result<Vec<u8>, String> {
        self.fits(original)?;
        let mut out = original.to_vec();
        for c in &self.changes {
            out[c.offset..c.offset + c.now.len()].copy_from_slice(&c.now);
        }
        Ok(out)
    }
}

/// Whether `path` is a version file (by its name).
pub fn is_version_file(path: &Path) -> bool {
    path.file_name()
        .is_some_and(|n| n.to_string_lossy().to_lowercase().ends_with(SUFFIX))
}

/// Where version `name` of `image` is kept.
pub fn path_for(image: &Path, name: &str) -> Result<PathBuf, String> {
    let name = checked_name(name)?;
    let stem = image
        .file_stem()
        .map(|s| s.to_string_lossy().to_string())
        .unwrap_or_else(|| "image".into());
    Ok(crate::edit::save_dir(image).join(format!("{stem} - {name}{SUFFIX}")))
}

pub fn load(path: &Path) -> Result<Version, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    Version::parse(&text).map_err(|e| format!("{}: {e}", path.display()))
}

/// Saves `v` as version `v.name` of `image`, as a new file (never over one: a name in use is refused). Returns the path.
pub fn save(v: &Version, image: &Path) -> Result<PathBuf, String> {
    if v.changes.is_empty() {
        return Err("nothing is changed: a version needs at least one change".into());
    }
    let path = path_for(image, &v.name)?;
    if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
        std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    }
    let mut f = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&path)
        .map_err(|e| {
            if e.kind() == std::io::ErrorKind::AlreadyExists {
                format!(
                    "a version named \"{}\" exists already ({}): choose another name",
                    v.name,
                    path.display()
                )
            } else {
                format!("{}: {e}", path.display())
            }
        })?;
    f.write_all(v.to_toml().as_bytes())
        .map_err(|e| format!("{}: {e}", path.display()))?;
    Ok(path)
}

/// The Maps page's "Save as version": the edits in `rom` (read from `image`) kept as version `name`, named by `def`.
pub fn save_edits(
    rom: &Rom,
    def: Option<&Definition>,
    image: &Path,
    name: &str,
    note: &str,
) -> Result<(PathBuf, Version), String> {
    let file_name = image
        .file_name()
        .map(|n| n.to_string_lossy().to_string())
        .unwrap_or_default();
    let v = Version::between(name, note, &file_name, rom.original(), rom.bytes(), def)?;
    Ok((save(&v, image)?, v))
}

/// The versions of `image` (its bytes `original`) kept where `save` puts them, by name; files of other images are left out.
pub fn list(image: &Path, original: &[u8]) -> Vec<(PathBuf, Version)> {
    let mut out: Vec<(PathBuf, Version)> = std::fs::read_dir(crate::edit::save_dir(image))
        .into_iter()
        .flatten()
        .flatten()
        .map(|e| e.path())
        .filter(|p| is_version_file(p))
        .filter_map(|p| Some((p.clone(), load(&p).ok()?)))
        .filter(|(_, v)| v.fits(original).is_ok())
        .collect();
    out.sort_by_key(|v| v.1.name.to_lowercase());
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmp(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("icarus-version-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    #[test]
    fn a_version_is_its_changes_and_makes_the_full_image_again() {
        let original: Vec<u8> = (0..64u8).collect();
        let mut edited = original.clone();
        edited[3] = 0xAA;
        edited[4] = 0xBB;
        edited[40] = 0;
        let v = Version::between(
            "Cold start",
            "richer",
            "stock.bin",
            &original,
            &edited,
            None,
        )
        .unwrap();
        assert_eq!(v.changes.len(), 2);
        assert_eq!(
            (v.changes[0].offset, v.changes[0].was.clone()),
            (3, vec![3, 4])
        );
        assert_eq!(v.changed_bytes(), 3);
        let text = v.to_toml();
        assert!(
            text.contains("at = \"0x3\"") && text.contains("now = \"AABB\""),
            "{text}"
        );
        let back = Version::parse(&text).unwrap();
        assert_eq!(back, v, "the file reads back as it was written");
        assert_eq!(
            back.apply(&original).unwrap(),
            edited,
            "reproducible into the full image"
        );

        let mut other = original.clone();
        other[63] = 1;
        assert!(v.apply(&other).unwrap_err().contains("another image"));
        assert!(v.apply(&original[..32]).unwrap_err().contains("0x40 bytes"));
        assert!(Version::between("a/b", "", "", &original, &edited, None).is_err());
        assert!(Version::parse(&text.replace("was = \"0304\"", "was = \"03\"")).is_err());
    }

    #[test]
    fn versions_are_kept_beside_the_vehicle_files_never_over_one() {
        let d = tmp("keep");
        let oem = d.join("Truck").join("OEM tunes");
        std::fs::create_dir_all(&oem).unwrap();
        let image = oem.join("stock.bin");
        let original = vec![7u8; 32];
        std::fs::write(&image, &original).unwrap();
        let mut edited = original.clone();
        edited[5] = 9;
        let v = Version::between("E85", "", "stock.bin", &original, &edited, None).unwrap();
        let p = save(&v, &image).unwrap();
        assert_eq!(
            p,
            d.join("Truck")
                .join("Custom tunes")
                .join("stock - E85.version.toml")
        );
        assert!(save(&v, &image).unwrap_err().contains("exists already"));
        assert!(save(
            &Version::between("Same", "", "", &original, &original, None).unwrap(),
            &image
        )
        .unwrap_err()
        .contains("nothing is changed"));
        // a version of another image in the same folder is not listed
        let other = Version::between("Other", "", "x.bin", &[1u8; 32], &[2u8; 32], None).unwrap();
        std::fs::write(
            d.join("Truck/Custom tunes/stock - Other.version.toml"),
            other.to_toml(),
        )
        .unwrap();
        let found = list(&image, &original);
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].1, v);
        assert_eq!(
            std::fs::read(&image).unwrap(),
            original,
            "the original is untouched"
        );
        let _ = std::fs::remove_dir_all(&d);
    }
}
