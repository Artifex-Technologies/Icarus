//! Each vehicle's trouble-code history: every reading of its OBD-II codes (stored, pending, permanent) is appended to
//! `codes-history.tsv` in the vehicle's `Logs` folder (on this computer only), and read back as when each code came and
//! went. One line per kind per reading: `<when>\t<kind>\t<module>|<code>;<module>|<code>...` (an empty list is kept too:
//! it is how a code is known to have gone).

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

pub const FILE: &str = "codes-history.tsv";

/// The history file of the vehicle with this VIN (its folder made if needed); the simulated vehicle's goes to Icarus's
/// state folder.
pub fn path(vin: &str) -> Result<PathBuf, String> {
    crate::garage::logs_dir(Some(vin)).map(|d| d.join(FILE))
}

/// One kind's codes at one reading: (kind, [(module, code)]).
pub type Reading = (String, Vec<(String, String)>);

/// One reading of one kind as read back: (when, [(module, code)]).
type Snapshot = (String, Vec<(String, String)>);

fn clean(s: &str) -> String {
    s.chars()
        .map(|c| {
            if matches!(c, '\t' | '\n' | '\r' | ';' | '|') {
                ' '
            } else {
                c
            }
        })
        .collect()
}

/// Appends one reading (every kind read, each with its codes, an empty list included) at `when`.
pub fn append(file: &Path, when: &str, readings: &[Reading]) -> Result<(), String> {
    use std::io::Write;
    if let Some(d) = file.parent() {
        std::fs::create_dir_all(d).map_err(|e| format!("{}: {e}", d.display()))?;
    }
    let mut f = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(file)
        .map_err(|e| format!("{}: {e}", file.display()))?;
    let mut out = String::new();
    for (kind, codes) in readings {
        let list: Vec<String> = codes
            .iter()
            .map(|(m, c)| format!("{}|{}", clean(m), clean(c)))
            .collect();
        out += &format!("{}\t{}\t{}\n", clean(when), clean(kind), list.join(";"));
    }
    f.write_all(out.as_bytes())
        .map_err(|e| format!("{}: {e}", file.display()))
}

/// Appends a reading to the history of the vehicle with this VIN; nothing without a VIN (the codes could belong to any
/// vehicle).
pub fn record(vin: Option<&str>, readings: &[Reading]) -> Result<(), String> {
    match vin {
        Some(v) => append(&path(v)?, &crate::report::now(), readings),
        None => Ok(()),
    }
}

/// When one code came and went.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Span {
    pub code: String,
    pub module: String,
    pub kind: String,
    /// The first and the last reading that had it.
    pub first: String,
    pub last: String,
    /// In how many readings of its kind it was.
    pub seen: usize,
    /// The first reading of its kind after `last` without it: the code has gone (cleared, or its monitor passed) since
    /// then. None: it was there in the newest reading.
    pub gone: Option<String>,
}

/// Every code in a history, the ones there now first, then the gone ones, newest first.
pub fn spans(text: &str) -> Vec<Span> {
    // per kind, the readings in order: (when, codes)
    let mut by_kind: BTreeMap<String, Vec<Snapshot>> = BTreeMap::new();
    for l in text.lines() {
        let mut f = l.splitn(3, '\t');
        let (Some(when), Some(kind)) = (f.next(), f.next()) else {
            continue;
        };
        let codes = f
            .next()
            .unwrap_or("")
            .split(';')
            .filter_map(|x| x.split_once('|'))
            .map(|(m, c)| (m.to_string(), c.to_string()))
            .collect();
        by_kind
            .entry(kind.to_string())
            .or_default()
            .push((when.to_string(), codes));
    }
    let mut out = Vec::new();
    for (kind, readings) in &by_kind {
        let mut seen: BTreeMap<(String, String), Span> = BTreeMap::new();
        for (when, codes) in readings {
            for (m, c) in codes {
                let s = seen.entry((m.clone(), c.clone())).or_insert_with(|| Span {
                    code: c.clone(),
                    module: m.clone(),
                    kind: kind.clone(),
                    first: when.clone(),
                    last: when.clone(),
                    seen: 0,
                    gone: None,
                });
                s.last = when.clone();
                s.seen += 1;
            }
        }
        for s in seen.values_mut() {
            let last_i = readings.iter().rposition(|(w, codes)| {
                *w == s.last && codes.iter().any(|(m, c)| *m == s.module && *c == s.code)
            });
            s.gone = last_i
                .and_then(|i| readings.get(i + 1))
                .map(|(w, _)| w.clone());
        }
        out.extend(seen.into_values());
    }
    out.sort_by(|a, b| {
        (a.gone.is_some(), std::cmp::Reverse(&a.last), &a.code).cmp(&(
            b.gone.is_some(),
            std::cmp::Reverse(&b.last),
            &b.code,
        ))
    });
    out
}

/// A span in words: "P0171 (engine (7E8), stored): first 2026-11-13 10:01, last 2026-11-14 17:56, in 3 readings; gone
/// since 2026-11-16 04:52".
pub fn describe(s: &Span) -> String {
    let when = if s.first == s.last {
        format!("seen {}", s.first)
    } else {
        format!("first {}, last {}, in {} readings", s.first, s.last, s.seen)
    };
    format!(
        "{} ({}, {}): {when}{}",
        s.code,
        s.module,
        s.kind,
        match &s.gone {
            Some(g) => format!("; gone since {g}"),
            None => "; there now".to_string(),
        }
    )
}

/// The history of the vehicle with this VIN, read from its folder (empty when there is none yet).
pub fn load(vin: &str) -> Vec<Span> {
    path(vin)
        .ok()
        .and_then(|p| std::fs::read_to_string(p).ok())
        .map(|t| spans(&t))
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn r(kind: &str, codes: &[(&str, &str)]) -> Reading {
        (
            kind.into(),
            codes
                .iter()
                .map(|(m, c)| (m.to_string(), c.to_string()))
                .collect(),
        )
    }

    #[test]
    fn codes_come_and_go() {
        let dir = std::env::temp_dir().join(format!("icarus-history-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let f = dir.join(FILE);
        append(
            &f,
            "2026-11-13 10:01",
            &[r("stored", &[("engine (7E8)", "P0171")]), r("pending", &[])],
        )
        .unwrap();
        append(
            &f,
            "2026-11-14 17:56",
            &[
                r(
                    "stored",
                    &[("engine (7E8)", "P0171"), ("engine (7E8)", "P0300")],
                ),
                r("pending", &[]),
            ],
        )
        .unwrap();
        append(
            &f,
            "2026-11-16 04:52",
            &[r("stored", &[("engine (7E8)", "P0300")]), r("pending", &[])],
        )
        .unwrap();
        let s = spans(&std::fs::read_to_string(&f).unwrap());
        assert_eq!(s.len(), 2);
        assert_eq!(
            (s[0].code.as_str(), s[0].gone.as_deref(), s[0].seen),
            ("P0300", None, 2),
            "there now, first"
        );
        assert_eq!(
            (
                s[1].code.as_str(),
                s[1].first.as_str(),
                s[1].last.as_str(),
                s[1].gone.as_deref()
            ),
            (
                "P0171",
                "2026-11-13 10:01",
                "2026-11-14 17:56",
                Some("2026-11-16 04:52")
            )
        );
        assert!(
            describe(&s[1]).ends_with("gone since 2026-11-16 04:52"),
            "{}",
            describe(&s[1])
        );
        assert!(describe(&s[0]).contains("there now"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn hostile_text_stays_in_its_field() {
        let dir = std::env::temp_dir().join(format!("icarus-history-h-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let f = dir.join(FILE);
        append(&f, "now", &[r("stored", &[("a\tb;c|d\ne", "P0171")])]).unwrap();
        let s = spans(&std::fs::read_to_string(&f).unwrap());
        assert_eq!((s.len(), s[0].module.as_str()), (1, "a b c d e"));
        assert!(spans("garbage\n\tno kind\n").is_empty());
        let _ = std::fs::remove_dir_all(&dir);
    }
}
