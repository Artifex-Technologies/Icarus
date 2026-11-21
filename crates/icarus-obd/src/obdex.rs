//! The standard trouble codes in depth, from OBDex (github.com/foerbsnavi/OBDex, CC0 1.0: free to use, no conditions): for
//! each of SAE J2012's 9,533 generic codes, what the module checks and when it sets the code, the usual causes with how
//! likely each is, the symptoms, whether it lights the check-engine lamp or can put the vehicle in limp mode, how hard the
//! repair is, related codes and the sources. English only, in `data/obdex/codes.jsonl.zz` (one code per line, sorted,
//! compressed), made by `cargo run -p icarus-obd --example obdex_import`; `data/obdex/README.md` says which OBDex it is.
//! Unpacked the first time a code is looked up, and kept: about 12 MB.

use serde::{Deserialize, Serialize};
use std::sync::OnceLock;

static PACKED: &[u8] = include_bytes!("../data/obdex/codes.jsonl.zz");

/// The credit shown with OBDex's text (CC0 asks for none; it is fair to say where words come from).
pub const CREDIT: &str = "OBDex (github.com/foerbsnavi/OBDex), CC0";

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Entry {
    pub code: String,
    pub title: String,
    /// What the module checks, when it sets the code, which makers use it how.
    pub about: String,
    /// (likelihood: high, medium or low; the cause).
    #[serde(default)]
    pub causes: Vec<(String, String)>,
    #[serde(default)]
    pub symptoms: Vec<String>,
    /// It lights the check-engine lamp.
    #[serde(default)]
    pub mil: bool,
    /// It matters for an emissions inspection.
    #[serde(default)]
    pub emissions: bool,
    /// Clearing it needs a drive cycle before the readiness monitors are ready again.
    #[serde(default)]
    pub drive_cycle: bool,
    /// The module can limit power (limp mode).
    #[serde(default)]
    pub limp: bool,
    /// "easy", "medium", "hard" or "shop_only".
    #[serde(default)]
    pub difficulty: String,
    #[serde(default)]
    pub diy: bool,
    /// Hours of work, from and to.
    #[serde(default)]
    pub hours: Option<(f64, f64)>,
    #[serde(default)]
    pub related: Vec<String>,
    #[serde(default)]
    pub sources: Vec<String>,
}

fn unpacked() -> &'static str {
    static TEXT: OnceLock<String> = OnceLock::new();
    TEXT.get_or_init(|| {
        miniz_oxide::inflate::decompress_to_vec_zlib(PACKED)
            .ok()
            .and_then(|b| String::from_utf8(b).ok())
            .unwrap_or_default()
    })
}

/// The lines, sorted by code; each starts `{"code":"P0420",`.
fn lines() -> &'static [&'static str] {
    static LINES: OnceLock<Vec<&'static str>> = OnceLock::new();
    LINES.get_or_init(|| unpacked().lines().filter(|l| !l.is_empty()).collect())
}

fn code_of(line: &str) -> &str {
    line.get(9..14).unwrap_or("")
}

/// The entry for one code (upper case, five characters), if OBDex has it.
pub fn get(code: &str) -> Option<Entry> {
    let c = code.trim().to_uppercase();
    let all = lines();
    let i = all.binary_search_by(|l| code_of(l).cmp(c.as_str())).ok()?;
    serde_json::from_str(all[i]).ok()
}

/// How many codes there are.
pub fn count() -> usize {
    lines().len()
}

/// Codes whose title has every word of `query` (any case), for searching by words ("oxygen heater"); at most `max`.
pub fn search(query: &str, max: usize) -> Vec<(String, String)> {
    #[derive(Deserialize)]
    struct Head {
        code: String,
        title: String,
    }
    let words: Vec<String> = query.split_whitespace().map(str::to_lowercase).collect();
    if words.is_empty() {
        return Vec::new();
    }
    lines()
        .iter()
        .filter_map(|l| serde_json::from_str::<Head>(l).ok())
        .filter(|h| {
            let t = h.title.to_lowercase();
            words.iter().all(|w| t.contains(w.as_str()))
        })
        .take(max)
        .map(|h| (h.code, h.title))
        .collect()
}

impl Entry {
    /// The flags in words: "lights the check-engine lamp; emissions inspection; ...".
    pub fn flags(&self) -> String {
        let mut f = Vec::new();
        if self.mil {
            f.push("lights the check-engine lamp");
        }
        if self.limp {
            f.push("can limit power (limp mode)");
        }
        if self.emissions {
            f.push("fails an emissions inspection while set");
        }
        if self.drive_cycle {
            f.push("after a fix, needs a drive cycle before the monitors are ready");
        }
        f.join("; ")
    }

    /// The repair in words: "medium, doable at home, 0.5-3 h".
    pub fn repair(&self) -> String {
        let d = match self.difficulty.as_str() {
            "shop_only" => "a job for a shop",
            "" => "",
            other => other,
        };
        let mut r = vec![d.to_string()];
        if self.diy && self.difficulty != "shop_only" {
            r.push("doable at home".into());
        }
        if let Some((a, b)) = self.hours {
            r.push(format!("{a}-{b} h of work"));
        }
        r.retain(|s| !s.is_empty());
        r.join(", ")
    }

    /// The causes, most likely first: "likely: ...; possible: ...".
    pub fn causes_text(&self) -> String {
        let by = |want: &str, word: &str| {
            let v: Vec<&str> = self
                .causes
                .iter()
                .filter(|(l, _)| l == want)
                .map(|(_, c)| c.as_str())
                .collect();
            (!v.is_empty()).then(|| format!("{word}: {}", v.join("; ")))
        };
        [
            by("high", "likely"),
            by("medium", "possible"),
            by("low", "rarely"),
        ]
        .into_iter()
        .flatten()
        .collect::<Vec<_>>()
        .join(". ")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_generic_code_is_there_sorted_and_reads() {
        let all = lines();
        assert!(count() > 9000, "{} codes", count());
        assert!(
            all.windows(2).all(|w| code_of(w[0]) < code_of(w[1])),
            "sorted, no code twice"
        );
        for l in all.iter().step_by(97) {
            let e: Entry = serde_json::from_str(l).unwrap_or_else(|x| panic!("{x}: {l}"));
            assert_eq!(e.code, code_of(l));
            assert!(!e.title.is_empty() && !e.about.is_empty(), "{}", e.code);
        }
    }

    #[test]
    fn a_catalyst_code_in_depth() {
        let e = get(" p0420 ").unwrap();
        assert!(
            e.title.contains("Catalyst") && e.title.contains("Bank 1"),
            "{}",
            e.title
        );
        assert!(e.mil && e.emissions);
        assert!(
            e.causes_text().starts_with("likely: "),
            "{}",
            e.causes_text()
        );
        assert!(e.related.iter().any(|r| r == "P0430"));
        assert!(get("P1234").is_none(), "maker codes are not in OBDex");
        assert!(get("X").is_none());
    }

    #[test]
    fn searching_by_words() {
        let hits = search("catalyst efficiency bank 2", 50);
        assert!(hits.iter().any(|(c, _)| c == "P0430"), "{hits:?}");
        assert!(search("  ", 5).is_empty());
    }
}
