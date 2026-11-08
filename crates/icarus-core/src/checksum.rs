//! Checksums in an ECU image: a module checks its calibration against numbers stored in the image and refuses (or runs badly
//! on) one that does not add up, so an edited image must have them corrected before it could ever be written. How each ECU
//! family computes them is its own; Icarus knows only what a checksum file describes (one per definition, beside it as
//! `<definition>.checksums.toml`), with the algorithms below.
//!
//! The safety rule: a description is trusted only if it matches the image as it was read (the stock image's stored numbers
//! must equal what the description computes). A description that does not is reported and never used to "fix" anything,
//! since it would write wrong numbers into the image. No description at all means the checksums are unknown: the image can
//! be edited and saved as a file, never written to a module (docs/PLAN.md, "Never brick a module", rule 3).
//!
//! ```toml
//! [[checksum]]
//! name = "Calibration"
//! algorithm = "sum16-be"      # sum8, sum16-be, sum16-le, sum32-be, sum32-le, xor8, crc16-ccitt, crc32
//! ranges = ["0x8000-0xFFFD"]  # inclusive; several allowed; must not include the stored number
//! store = "0xFFFE"            # where the number is kept (big-endian for -be and the CRCs, little-endian for -le)
//! stored_as = "value"         # value (the number itself), or complement (what makes ranges + stored add up to target)
//! target = "0x0"              # complement only
//! source = "where this description comes from"
//! checked = false             # true once seen right on a real image of this family
//! ```

use serde::Deserialize;
use std::ops::RangeInclusive;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Algorithm {
    Sum8,
    Sum16Be,
    Sum16Le,
    Sum32Be,
    Sum32Le,
    Xor8,
    Crc16Ccitt,
    Crc32,
}

impl Algorithm {
    fn parse(s: &str) -> Result<Algorithm, String> {
        Ok(match s.to_lowercase().as_str() {
            "sum8" => Algorithm::Sum8,
            "sum16-be" => Algorithm::Sum16Be,
            "sum16-le" => Algorithm::Sum16Le,
            "sum32-be" => Algorithm::Sum32Be,
            "sum32-le" => Algorithm::Sum32Le,
            "xor8" => Algorithm::Xor8,
            "crc16-ccitt" => Algorithm::Crc16Ccitt,
            "crc32" => Algorithm::Crc32,
            other => return Err(format!("unknown checksum algorithm {other:?}")),
        })
    }

    /// Bytes the stored number takes.
    pub fn width(self) -> usize {
        match self {
            Algorithm::Sum8 | Algorithm::Xor8 => 1,
            Algorithm::Sum16Be | Algorithm::Sum16Le | Algorithm::Crc16Ccitt => 2,
            Algorithm::Sum32Be | Algorithm::Sum32Le | Algorithm::Crc32 => 4,
        }
    }

    fn little_endian(self) -> bool {
        matches!(self, Algorithm::Sum16Le | Algorithm::Sum32Le)
    }

    fn mask(self) -> u64 {
        (1u64 << (8 * self.width())) - 1
    }

    /// Only additive sums can be balanced with a complement.
    fn additive(self) -> bool {
        matches!(
            self,
            Algorithm::Sum8
                | Algorithm::Sum16Be
                | Algorithm::Sum16Le
                | Algorithm::Sum32Be
                | Algorithm::Sum32Le
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StoredAs {
    Value,
    Complement,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Spec {
    pub name: String,
    pub algorithm: Algorithm,
    pub ranges: Vec<RangeInclusive<usize>>,
    pub store: usize,
    pub stored_as: StoredAs,
    pub target: u64,
    pub source: String,
    pub checked: bool,
}

#[derive(Deserialize)]
struct File {
    #[serde(default)]
    checksum: Vec<RawSpec>,
}

#[derive(Deserialize)]
struct RawSpec {
    name: String,
    algorithm: String,
    ranges: Vec<String>,
    store: String,
    #[serde(default)]
    stored_as: Option<String>,
    #[serde(default)]
    target: Option<String>,
    #[serde(default)]
    source: String,
    #[serde(default)]
    checked: bool,
}

fn number(s: &str) -> Result<usize, String> {
    let t = s.trim();
    let r = match t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
        Some(h) => usize::from_str_radix(h, 16),
        None => t.parse(),
    };
    r.map_err(|_| format!("{s:?} is not a number (decimal or 0x hex)"))
}

/// Reads a checksum file; `size` is the image's, so a description that reaches past its end is refused here.
pub fn parse(text: &str, size: usize) -> Result<Vec<Spec>, String> {
    let f: File = toml::from_str(text).map_err(|e| e.to_string())?;
    let mut out = vec![];
    for r in f.checksum {
        let algorithm = Algorithm::parse(&r.algorithm)?;
        let stored_as = match r.stored_as.as_deref().unwrap_or("value") {
            "value" => StoredAs::Value,
            "complement" => StoredAs::Complement,
            other => {
                return Err(format!(
                    "{}: stored_as is value or complement, not {other:?}",
                    r.name
                ))
            }
        };
        if stored_as == StoredAs::Complement && !algorithm.additive() {
            return Err(format!(
                "{}: only a sum can be stored as a complement",
                r.name
            ));
        }
        let store = number(&r.store)?;
        if store + algorithm.width() > size {
            return Err(format!(
                "{}: the stored number at 0x{store:X} is past the image's end (0x{size:X} bytes)",
                r.name
            ));
        }
        let mut ranges = vec![];
        for s in &r.ranges {
            let (a, b) = s
                .split_once('-')
                .ok_or_else(|| format!("{}: range {s:?} is not START-END", r.name))?;
            let (a, b) = (number(a)?, number(b)?);
            if a > b || b >= size {
                return Err(format!(
                    "{}: range {s} is empty or past the image's end",
                    r.name
                ));
            }
            let w = algorithm.width();
            let word_sum = matches!(
                algorithm,
                Algorithm::Sum16Be | Algorithm::Sum16Le | Algorithm::Sum32Be | Algorithm::Sum32Le
            );
            if word_sum && (b + 1 - a) % w != 0 {
                return Err(format!(
                    "{}: range {s} is not a whole number of {w}-byte words",
                    r.name
                ));
            }
            if store <= b && a < store + w {
                return Err(format!(
                    "{}: range {s} includes the stored number at 0x{store:X}",
                    r.name
                ));
            }
            ranges.push(a..=b);
        }
        if ranges.is_empty() {
            return Err(format!("{}: no ranges", r.name));
        }
        let target = match &r.target {
            Some(t) => number(t)? as u64,
            None => 0,
        };
        out.push(Spec {
            name: r.name,
            algorithm,
            ranges,
            store,
            stored_as,
            target,
            source: r.source,
            checked: r.checked,
        });
    }
    Ok(out)
}

fn crc16_ccitt(data: impl Iterator<Item = u8>) -> u64 {
    let mut crc: u16 = 0xFFFF;
    for b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc as u64
}

fn crc32(data: impl Iterator<Item = u8>) -> u64 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xEDB8_8320
            } else {
                crc >> 1
            };
        }
    }
    (!crc) as u64
}

/// The algorithm over the ranges.
pub fn compute(s: &Spec, image: &[u8]) -> u64 {
    let bytes = || {
        s.ranges
            .iter()
            .flat_map(|r| image[r.clone()].iter().copied())
    };
    let words = |w: usize, le: bool| -> u64 {
        let mut sum: u64 = 0;
        for r in &s.ranges {
            for chunk in image[r.clone()].chunks(w) {
                let mut v: u64 = 0;
                for (k, b) in chunk.iter().enumerate() {
                    let shift = if le { 8 * k } else { 8 * (w - 1 - k) };
                    v |= (*b as u64) << shift;
                }
                sum = sum.wrapping_add(v);
            }
        }
        sum
    };
    let raw = match s.algorithm {
        Algorithm::Sum8 => bytes().fold(0u64, |a, b| a.wrapping_add(b as u64)),
        Algorithm::Sum16Be => words(2, false),
        Algorithm::Sum16Le => words(2, true),
        Algorithm::Sum32Be => words(4, false),
        Algorithm::Sum32Le => words(4, true),
        Algorithm::Xor8 => bytes().fold(0u64, |a, b| a ^ b as u64),
        Algorithm::Crc16Ccitt => crc16_ccitt(bytes()),
        Algorithm::Crc32 => crc32(bytes()),
    };
    let m = s.algorithm.mask();
    match s.stored_as {
        StoredAs::Value => raw & m,
        // the number that makes the ranges plus it add up to target
        StoredAs::Complement => s.target.wrapping_sub(raw) & m,
    }
}

/// The number stored in the image.
pub fn stored(s: &Spec, image: &[u8]) -> u64 {
    let w = s.algorithm.width();
    let le = s.algorithm.little_endian();
    (0..w).fold(0u64, |v, k| {
        let shift = if le { 8 * k } else { 8 * (w - 1 - k) };
        v | (image[s.store + k] as u64) << shift
    })
}

fn put(s: &Spec, image: &mut [u8], v: u64) {
    let w = s.algorithm.width();
    let le = s.algorithm.little_endian();
    for k in 0..w {
        let shift = if le { 8 * k } else { 8 * (w - 1 - k) };
        image[s.store + k] = (v >> shift) as u8;
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Status {
    pub name: String,
    pub stored: u64,
    pub computed: u64,
    /// The description matched the image as it was read: it may be used.
    pub trusted: bool,
}

impl Status {
    pub fn ok(&self) -> bool {
        self.stored == self.computed
    }
}

/// Each checksum in `image`, and whether its description is trusted (it matched `original`).
pub fn check(specs: &[Spec], original: &[u8], image: &[u8]) -> Vec<Status> {
    specs
        .iter()
        .map(|s| Status {
            name: s.name.clone(),
            stored: stored(s, image),
            computed: compute(s, image),
            trusted: stored(s, original) == compute(s, original),
        })
        .collect()
}

/// Corrects every trusted checksum in `image`; returns (name, old, new) for each one it changed. Untrusted ones are left as
/// they are (fixing them could write wrong numbers).
pub fn fix(specs: &[Spec], original: &[u8], image: &mut [u8]) -> Vec<(String, u64, u64)> {
    let mut changed = vec![];
    for s in specs {
        if stored(s, original) != compute(s, original) {
            continue;
        }
        let (old, new) = (stored(s, image), compute(s, image));
        if old != new {
            put(s, image, new);
            changed.push((s.name.clone(), old, new));
        }
    }
    changed
}

/// The checksum file that goes with a definition: `<definition minus .xdf>.checksums.toml`.
pub fn file_for(definition: &std::path::Path) -> std::path::PathBuf {
    definition.with_extension("checksums.toml")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn image() -> Vec<u8> {
        (0..64u32).map(|i| (i * 7 % 251) as u8).collect()
    }

    fn spec(algo: &str, stored_as: &str) -> Vec<Spec> {
        parse(&format!("[[checksum]]\nname = \"c\"\nalgorithm = \"{algo}\"\nranges = [\"0x0-0x2F\"]\nstore = \"0x30\"\nstored_as = \"{stored_as}\"\ntarget = \"0\"\n"), 64).unwrap()
    }

    #[test]
    fn known_values() {
        // CRC-32 of "123456789" is 0xCBF43926, CRC-16/CCITT-FALSE 0x29B1 (the standard check values)
        let mut img = b"123456789".to_vec();
        img.extend([0u8; 4]);
        let s = parse(
            "[[checksum]]\nname=\"a\"\nalgorithm=\"crc32\"\nranges=[\"0-8\"]\nstore=\"9\"\n",
            img.len(),
        )
        .unwrap();
        assert_eq!(compute(&s[0], &img), 0xCBF4_3926);
        let s = parse(
            "[[checksum]]\nname=\"a\"\nalgorithm=\"crc16-ccitt\"\nranges=[\"0-8\"]\nstore=\"9\"\n",
            img.len(),
        )
        .unwrap();
        assert_eq!(compute(&s[0], &img), 0x29B1);
        let s = parse(
            "[[checksum]]\nname=\"a\"\nalgorithm=\"sum8\"\nranges=[\"0-8\"]\nstore=\"9\"\n",
            img.len(),
        )
        .unwrap();
        assert_eq!(
            compute(&s[0], &img),
            (b'1' as u64..=b'9' as u64).sum::<u64>() & 0xFF
        );
    }

    #[test]
    fn an_edit_breaks_the_checksum_and_fix_mends_it() {
        for (algo, st) in [
            ("sum16-be", "value"),
            ("sum16-le", "complement"),
            ("sum32-be", "complement"),
            ("crc32", "value"),
            ("xor8", "value"),
        ] {
            let specs = spec(algo, st);
            let mut original = image();
            fix_fresh(&specs, &mut original);
            let st0 = check(&specs, &original, &original);
            assert!(
                st0[0].ok() && st0[0].trusted,
                "{algo}: a stock image adds up"
            );
            let mut edited = original.clone();
            edited[10] = edited[10].wrapping_add(3);
            assert!(
                !check(&specs, &original, &edited)[0].ok(),
                "{algo}: the edit shows"
            );
            let changed = fix(&specs, &original, &mut edited);
            assert_eq!(changed.len(), 1);
            assert!(
                check(&specs, &original, &edited)[0].ok(),
                "{algo}: corrected"
            );
        }
    }

    /// Writes correct checksums into a test image (as the ECU maker did).
    fn fix_fresh(specs: &[Spec], img: &mut [u8]) {
        for s in specs {
            let v = compute(s, img);
            put(s, img, v);
        }
    }

    #[test]
    fn a_description_that_does_not_match_the_stock_image_is_never_used() {
        let specs = spec("sum16-be", "value");
        let original = image(); // its stored number was never made right: the description does not match it
        let st = check(&specs, &original, &original);
        assert!(!st[0].trusted);
        let mut edited = original.clone();
        edited[3] ^= 1;
        assert!(
            fix(&specs, &original, &mut edited).is_empty(),
            "nothing is written from an untrusted description"
        );
        assert_eq!(&edited[0x30..0x32], &original[0x30..0x32]);
    }

    #[test]
    fn bad_descriptions_are_refused() {
        let bad = |t: &str| parse(t, 64).unwrap_err();
        assert!(bad(
            "[[checksum]]\nname=\"a\"\nalgorithm=\"md5\"\nranges=[\"0-3\"]\nstore=\"8\"\n"
        )
        .contains("unknown checksum algorithm"));
        assert!(bad(
            "[[checksum]]\nname=\"a\"\nalgorithm=\"sum8\"\nranges=[\"0-10\"]\nstore=\"8\"\n"
        )
        .contains("includes the stored number"));
        assert!(bad(
            "[[checksum]]\nname=\"a\"\nalgorithm=\"sum8\"\nranges=[\"0-3\"]\nstore=\"0x40\"\n"
        )
        .contains("past the image's end"));
        assert!(bad(
            "[[checksum]]\nname=\"a\"\nalgorithm=\"sum16-be\"\nranges=[\"0-2\"]\nstore=\"8\"\n"
        )
        .contains("whole number"));
        assert!(bad("[[checksum]]\nname=\"a\"\nalgorithm=\"crc32\"\nranges=[\"0-3\"]\nstore=\"8\"\nstored_as=\"complement\"\n").contains("only a sum"));
        assert!(parse("", 64).unwrap().is_empty());
        assert_eq!(
            file_for(std::path::Path::new("/x/ecu.xdf")),
            std::path::PathBuf::from("/x/ecu.checksums.toml")
        );
    }
}
