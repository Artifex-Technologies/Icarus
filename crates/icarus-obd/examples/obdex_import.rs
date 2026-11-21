//! Makes `data/obdex/codes.jsonl.zz` (`icarus_obd::obdex`) from OBDex's published file:
//!
//!     curl -LO https://foerbsnavi.github.io/OBDex/generic.min.json
//!     cargo run -p icarus-obd --example obdex_import -- generic.min.json
//!
//! English only, at most three sources per code; one code per line, sorted, compressed. Then note OBDex's date and commit in
//! `data/obdex/README.md`.

use icarus_obd::obdex::Entry;
use serde_json::Value;

fn text(v: &Value) -> String {
    v.get("en")
        .and_then(Value::as_str)
        .unwrap_or("")
        .trim()
        .to_string()
}

fn entry(c: &Value) -> Option<Entry> {
    let code = c.get("code")?.as_str()?.trim().to_uppercase();
    if code.len() != 5 || !code.is_ascii() {
        return None;
    }
    let list = |k: &str| {
        c.get(k)
            .and_then(Value::as_array)
            .cloned()
            .unwrap_or_default()
    };
    let flag = |k: &str| {
        c.pointer(&format!("/flags/{k}"))
            .and_then(Value::as_bool)
            .unwrap_or(false)
    };
    let hours = c
        .pointer("/repair/estimated_hours")
        .and_then(Value::as_array)
        .and_then(|h| Some((h.first()?.as_f64()?, h.get(1)?.as_f64()?)));
    Some(Entry {
        title: text(c.get("title")?),
        about: c.get("description").map(text).unwrap_or_default(),
        causes: list("common_causes")
            .iter()
            .filter_map(|x| {
                let l = x.get("likelihood")?.as_str()?.to_string();
                Some((l, text(x.get("label")?)))
            })
            .collect(),
        symptoms: list("symptoms")
            .iter()
            .map(text)
            .filter(|s| !s.is_empty())
            .collect(),
        mil: flag("mil"),
        emissions: flag("emissions_relevant"),
        drive_cycle: flag("drive_cycle_required"),
        limp: flag("limp_mode_possible"),
        difficulty: c
            .pointer("/repair/difficulty")
            .and_then(Value::as_str)
            .unwrap_or("")
            .to_string(),
        diy: c
            .pointer("/repair/diy_possible")
            .and_then(Value::as_bool)
            .unwrap_or(false),
        hours,
        related: list("related_codes")
            .iter()
            .filter_map(|r| r.as_str().map(str::to_string))
            .collect(),
        sources: list("sources")
            .iter()
            .filter_map(|s| {
                s.as_str()
                    .filter(|u| u.starts_with("https://"))
                    .map(str::to_string)
            })
            .take(3)
            .collect(),
        code,
    })
}

fn main() -> Result<(), String> {
    let path = std::env::args()
        .nth(1)
        .ok_or("usage: obdex_import generic.min.json")?;
    let raw = std::fs::read_to_string(&path).map_err(|e| format!("{path}: {e}"))?;
    let all: Vec<Value> = serde_json::from_str(&raw).map_err(|e| e.to_string())?;
    let mut entries: Vec<Entry> = all
        .iter()
        .filter_map(entry)
        .filter(|e| !e.title.is_empty())
        .collect();
    entries.sort_by(|a, b| a.code.cmp(&b.code));
    entries.dedup_by(|a, b| a.code == b.code);
    let mut out = String::new();
    for e in &entries {
        out += &serde_json::to_string(e).map_err(|x| x.to_string())?;
        out.push('\n');
    }
    let packed = miniz_oxide::deflate::compress_to_vec_zlib(out.as_bytes(), 10);
    let dest = concat!(env!("CARGO_MANIFEST_DIR"), "/data/obdex/codes.jsonl.zz");
    std::fs::write(dest, &packed).map_err(|e| format!("{dest}: {e}"))?;
    println!(
        "{} codes ({} skipped), {:.1} MB unpacked, {:.1} MB in {dest}",
        entries.len(),
        all.len() - entries.len(),
        out.len() as f64 / 1e6,
        packed.len() as f64 / 1e6
    );
    Ok(())
}
