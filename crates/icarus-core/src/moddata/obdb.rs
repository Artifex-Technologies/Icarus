//! OBDb signal sets (github.com/OBDb: `signalsets/v3/default.json` in a vehicle's repository), imported from a file the
//! user downloaded. OBDb's data is CC BY-SA 4.0: Icarus ships none of it, only reads the user's copy, and the attribution
//! goes with every value imported.
//!
//! This reads the JSON loosely (fields it does not know are counted, not refused) and keeps only commands on services 21
//! and 22, never on 7DF. `icarus-obd` checks every request again when the set is loaded. Each value is checked as it is
//! added (`check_value`), so a bad signal leaves out only itself; a bad command (its header, identifier, years) leaves out
//! only that command. Everything left out is listed with the reason.
//!
//! How OBDb's fields become Icarus's (checked against OBDb's own reference decoder):
//!
//! | OBDb | Icarus |
//! | --- | --- |
//! | the repository folder `Make-Model` | `set.make` (the first word, or two for makes such as Mercedes-Benz) and `set.models` (the rest, as one phrase: `Ford-F-150` is the model "F 150") |
//! | `hdr`, `rax`, `eax`, `tst` | `module.tx`, `module.rx` (tx + 8 when not given, noted), `module.ext`, `module.tester` |
//! | `cmd {"22": "1F01"}` / `{"21": "05"}` | `request.service`, `request.ident`; any other service, and 7DF, are left out |
//! | `freq`, `tmo` (units of 4 ms), `din` | `every`, `timeout_ms`, `needs_session` ("10 <din>": never sent) |
//! | `filter`, `dbgfilter`, `dbg` | `years`, `unverified` (`dbg`: every year) |
//! | `fmt.bix`, `len`, `blsb`, `sign` | `byte` = bix / 8, `bit` = bix % 8, `bits`, `order = "lsb-bytes"`, `signed` |
//! | `fmt.mul`, `div`, `add`, `min`, `max`, `nullmin`, `nullmax` | the same names (`min` is 0 when only `max` is given), `null_below`, `null_above` |
//! | `fmt.unit`, `fmt.map` | `unit` (OBDb's word in Icarus's terms), or `kind = "state"` with `states` |
//!
//! A year filter `{from, to, years}`: from < to is the range from-to; from >= to wraps (every year up to `to`, and every
//! year from `from` on); `from` or `to` alone is open on the other side; `years` adds single years. A year that is not a
//! year (above 9999) leaves the command out rather than widening its range.

use super::{
    sha256_hex, validate, Ids, Imported, ModuleDef, ModuleHead, Order, RequestDef, SetInfo, Source,
    ValueDef, ValueKind, Years, FORMAT,
};
use serde_json::{Map, Value};
use std::collections::BTreeMap;

pub const LICENSE: &str = "CC-BY-SA-4.0";
pub const ATTRIBUTION: &str = "signal data: OBDb (github.com/OBDb), CC BY-SA 4.0";

/// What the command line knows that the file does not.
#[derive(Clone, Debug, Default)]
pub struct Options {
    /// The path the file was read from (it names the vehicle: `.../Toyota-Tacoma/signalsets/v3/default.json`).
    pub path: String,
    /// `--name`: the set's name.
    pub name: Option<String>,
    /// `--make`: the vehicle profile it is for.
    pub make: Option<String>,
    /// Today, for `source.imported` (YYYY-MM-DD).
    pub today: String,
}

/// The repository folder in a path (the one before `signalsets`), and the path from it on.
fn repo_of(path: &str) -> Option<(String, String)> {
    let parts: Vec<&str> = path.split(['/', '\\']).filter(|p| !p.is_empty()).collect();
    let i = parts.iter().position(|p| *p == "signalsets")?;
    let repo = parts.get(i.checked_sub(1)?)?;
    Some((repo.to_string(), parts[i - 1..].join("/")))
}

/// `2016-2023.json` -> [[2016, 2023]]; `default.json` -> every year.
fn file_years(path: &str) -> Years {
    let base = path.rsplit(['/', '\\']).next().unwrap_or("");
    let stem = base.strip_suffix(".json").unwrap_or(base);
    match stem.split_once('-') {
        Some((a, b)) => match (a.parse::<u16>(), b.parse::<u16>()) {
            (Ok(a), Ok(b)) if a <= b && a > 1900 => vec![[a, b]],
            _ => vec![],
        },
        None => vec![],
    }
}

fn hex_of(v: Option<&Value>) -> Option<Result<u32, String>> {
    let s = v?.as_str()?;
    Some(u32::from_str_radix(s.trim(), 16).map_err(|_| format!("{s:?} is not hex")))
}

/// The most a year may be (9999 stands for "on", open-ended).
const LAST_YEAR: u16 = 9999;

/// A year from a filter, if it is one: Err when it is given but is not a year.
fn year_of(v: Option<&Value>, what: &str) -> Result<Option<u16>, String> {
    let Some(v) = v else { return Ok(None) };
    v.as_u64()
        .and_then(|y| u16::try_from(y).ok())
        .filter(|y| *y <= LAST_YEAR)
        .map(Some)
        .ok_or_else(|| format!("year filter: {what} {v} is not a year"))
}

/// An OBDb year filter `{from, to, years[]}` as inclusive ranges (a from at or after to wraps: see the module notes).
/// Err when a year in it is not one: dropping it would widen the range, so the command is left out instead.
pub fn filter_years(f: &Map<String, Value>) -> Result<Years, String> {
    let from = year_of(f.get("from"), "from")?;
    let to = year_of(f.get("to"), "to")?;
    let mut out = match (from, to) {
        (Some(a), Some(b)) if a < b => vec![[a, b]],
        (Some(a), Some(b)) => vec![[0, b], [a, 9999]],
        (Some(a), None) => vec![[a, 9999]],
        (None, Some(b)) => vec![[0, b]],
        (None, None) => vec![],
    };
    if let Some(ys) = f.get("years").and_then(|y| y.as_array()) {
        for y in ys {
            let y = year_of(Some(y), "years")?.unwrap_or(0);
            out.push([y, y]);
        }
    }
    Ok(out)
}

/// What an OBDb unit word becomes: (Icarus unit, kind, states, note).
type UnitOf = (
    String,
    Option<ValueKind>,
    Option<BTreeMap<String, String>>,
    Option<String>,
);

/// OBDb's unit word in Icarus's terms.
fn unit(word: &str) -> UnitOf {
    let two = |a: &str, b: &str| {
        Some(BTreeMap::from([
            ("0".to_string(), a.to_string()),
            ("1".to_string(), b.to_string()),
        ]))
    };
    let assumed = Some("order of on/off assumed: the first word read as 0".to_string());
    let u = match word {
        "celsius" => "°C",
        "fahrenheit" => "°F",
        "kilometersPerHour" => "km/h",
        "kilometers" => "km",
        "kilopascal" => "kPa",
        "psi" => "psi",
        "bars" => "bar",
        "rpm" => "rpm",
        "percent" => "%",
        "volts" => "V",
        "amps" => "A",
        "milliamps" => "mA",
        "ampereHours" => "Ah",
        "degrees" => "°",
        "seconds" => "s",
        "milliseconds" => "ms",
        "minutes" => "min",
        "hours" => "h",
        "liters" => "L",
        "gramsPerSecond" => "g/s",
        "newtonMeters" => "Nm",
        "metersPerSecondSquared" => "m/s²",
        "gravity" => "g",
        "millimeters" => "mm",
        "scalar" | "unknown" | "normal" | "" => "",
        "hex" => return (String::new(), Some(ValueKind::Hex), None, None),
        "ascii" => return (String::new(), Some(ValueKind::Text), None, None),
        "offon" => {
            return (
                String::new(),
                Some(ValueKind::State),
                two("off", "on"),
                assumed,
            )
        }
        "noyes" => {
            return (
                String::new(),
                Some(ValueKind::State),
                two("no", "yes"),
                assumed,
            )
        }
        "onoff" => {
            return (
                String::new(),
                Some(ValueKind::State),
                two("on", "off"),
                assumed,
            )
        }
        "yesno" => {
            return (
                String::new(),
                Some(ValueKind::State),
                two("yes", "no"),
                assumed,
            )
        }
        other => {
            return (
                other.to_string(),
                None,
                None,
                Some(format!("unit {other:?} kept as given")),
            )
        }
    };
    (u.to_string(), None, None, None)
}

/// One signal as a value; Err(reason) when it cannot be imported.
fn signal(s: &Value) -> Result<ValueDef, String> {
    let id = s.get("id").and_then(|v| v.as_str()).ok_or("no id")?;
    let fmt = s.get("fmt").and_then(|v| v.as_object()).ok_or("no fmt")?;
    let num = |k: &str| fmt.get(k).and_then(|v| v.as_f64());
    let bix = match fmt.get("bix") {
        None => 0,
        Some(b) => b
            .as_u64()
            .and_then(|b| usize::try_from(b).ok())
            .filter(|b| *b / 8 < super::MAX_BYTES)
            .ok_or_else(|| {
                format!(
                    "bix {b} is past the {} bytes an answer can hold",
                    super::MAX_BYTES
                )
            })?,
    };
    let len = fmt.get("len").and_then(|v| v.as_u64()).ok_or("no len")?;
    let mut v = ValueDef::new(
        id,
        s.get("name").and_then(|v| v.as_str()).unwrap_or(id),
        bix / 8,
        u16::try_from(len).map_err(|_| "len too large")?,
    );
    v.bit = (bix % 8) as u8;
    v.path = s.get("path").and_then(|v| v.as_str()).unwrap_or("").into();
    v.about = s
        .get("description")
        .and_then(|v| v.as_str())
        .unwrap_or("")
        .into();
    v.hidden = s.get("hidden").and_then(|v| v.as_bool()).unwrap_or(false);
    if fmt.get("blsb").and_then(|v| v.as_bool()).unwrap_or(false) {
        v.order = Order::LsbBytes;
    }
    v.signed = fmt.get("sign").and_then(|v| v.as_bool()).unwrap_or(false);
    v.mul = num("mul").unwrap_or(1.0);
    v.div = num("div").unwrap_or(1.0);
    v.add = num("add").unwrap_or(0.0);
    v.max = num("max");
    v.min = num("min").or(v.max.map(|_| 0.0));
    v.null_below = num("nullmin");
    v.null_above = num("nullmax");
    let mut notes = Vec::new();
    if let Some(map) = fmt.get("map").and_then(|m| m.as_object()) {
        v.kind = ValueKind::State;
        for (k, e) in map {
            let label = match e {
                Value::Object(o) => o
                    .get("description")
                    .and_then(|d| d.as_str())
                    .filter(|d| !d.is_empty())
                    .or(o.get("value").and_then(|d| d.as_str()))
                    .unwrap_or(k)
                    .to_string(),
                Value::String(s) => s.clone(),
                other => other.to_string(),
            };
            v.states.insert(k.clone(), label);
        }
        // a key the bits can never hold (a wider map than the signal) is left out, not the whole signal
        let (lo, hi) = super::state_range(&v);
        let never: Vec<String> = v
            .states
            .keys()
            .filter(|k| {
                k.trim()
                    .parse::<i128>()
                    .is_ok_and(|n| !(lo..=hi).contains(&n))
            })
            .cloned()
            .collect();
        if !never.is_empty() {
            for k in &never {
                v.states.remove(k);
            }
            notes.push(format!(
                "map key(s) {} left out: {} bit(s) never hold them",
                never.join(", "),
                v.bits
            ));
        }
    } else if let Some(w) = fmt.get("unit").and_then(|u| u.as_str()) {
        let (u, kind, states, note) = unit(w);
        v.unit = u;
        if let Some(k) = kind {
            v.kind = k;
        }
        if let Some(st) = states {
            v.states = st;
        }
        notes.extend(note);
    }
    if matches!(v.kind, ValueKind::Text | ValueKind::Hex)
        && (v.bit != 0 || !v.bits.is_multiple_of(8))
    {
        notes.push(format!(
            "{} is not whole bytes: read as a number",
            v.kind.word()
        ));
        v.kind = ValueKind::Number;
    }
    v.note = notes.join("; ");
    Ok(v)
}

/// Which `ecu[]` entry describes a module: the most specific one that matches (hdr, then eax and rax when given).
fn ecu_kind(ecus: &[Value], tx: u32, rx: u32, ext: Option<u8>) -> String {
    let mut best: Option<(usize, String)> = None;
    for e in ecus {
        if hex_of(e.get("hdr")).and_then(|r| r.ok()) != Some(tx) {
            continue;
        }
        let mut score = 1;
        match hex_of(e.get("eax")).and_then(|r| r.ok()) {
            Some(x) if u8::try_from(x).ok() != ext => continue,
            Some(_) => score += 1,
            None => {}
        }
        match hex_of(e.get("rax")).and_then(|r| r.ok()) {
            Some(x) if x != rx => continue,
            Some(_) => score += 1,
            None => {}
        }
        if best.as_ref().is_none_or(|(s, _)| score > *s) {
            let kind = e
                .get("type")
                .and_then(|t| t.as_str())
                .unwrap_or("")
                .to_string();
            best = Some((score, kind));
        }
    }
    best.map(|b| b.1).unwrap_or_default()
}

/// Makes whose name is two words in a repository folder's name (`Mercedes-Benz-Sprinter`, `Land-Rover-Defender`).
const TWO_WORD_MAKES: [&str; 5] = [
    "mercedes-benz",
    "land-rover",
    "alfa-romeo",
    "aston-martin",
    "rolls-royce",
];

/// A repository folder's name as (make, model): `Toyota-Tacoma` -> ("toyota", "Tacoma"), `Ford-F-150` -> ("ford",
/// "F 150"), `Mercedes-Benz-Sprinter` -> ("mercedes-benz", "Sprinter"). The model is one phrase (empty when the folder
/// names only a make).
pub fn make_and_model(repo: &str) -> (String, String) {
    let words: Vec<&str> = repo.split('-').filter(|w| !w.is_empty()).collect();
    let take = match words.as_slice() {
        [a, b, ..] if TWO_WORD_MAKES.contains(&format!("{a}-{b}").to_lowercase().as_str()) => 2,
        [] => 0,
        _ => 1,
    };
    (
        words[..take].join("-").to_lowercase(),
        words[take..].join(" "),
    )
}

/// A signal set's JSON as a set of modules.
pub fn parse(text: &str, opts: &Options) -> Result<Imported, String> {
    let root: Value = serde_json::from_str(text).map_err(|e| format!("not OBDb JSON: {e}"))?;
    let commands = root
        .get("commands")
        .and_then(|c| c.as_array())
        .ok_or("no \"commands\" list: is this an OBDb signal set (signalsets/v3/*.json)?")?;
    let ecus: Vec<Value> = root
        .get("ecu")
        .and_then(|e| e.as_array())
        .cloned()
        .unwrap_or_default();
    let repo = repo_of(&opts.path);
    let name = match (&opts.name, &repo) {
        (Some(n), _) => n.clone(),
        (None, Some((r, _))) => r.to_lowercase(),
        (None, None) => return Err(
            "the set needs a name: --name SET (or import from the repository's own folder, e.g. \
                 Toyota-Tacoma/signalsets/v3/default.json)"
                .into(),
        ),
    };
    let (repo_make, repo_model) = repo
        .as_ref()
        .map(|(r, _)| make_and_model(r))
        .unwrap_or_default();
    let make = opts.make.clone().unwrap_or(repo_make);
    let mut out = Imported::default();
    for k in ["signalGroups", "synthetics", "diagnosticLevel"] {
        if let Some(v) = root.get(k) {
            let n = v.as_array().map(|a| a.len()).unwrap_or(1);
            out.left_out
                .push((format!("{n} {k}"), "not part of Icarus's format yet".into()));
        }
    }
    // (ids, tx, rx, ext, tester) -> module
    let mut modules: Vec<ModuleDef> = Vec::new();
    for (i, c) in commands.iter().enumerate() {
        let hdr_s = c.get("hdr").and_then(|h| h.as_str()).unwrap_or("");
        let cmd = c.get("cmd").and_then(|v| v.as_object());
        let what = format!(
            "{hdr_s} {}",
            cmd.and_then(|m| m.iter().next())
                .map(|(s, v)| format!("{s} {}", v.as_str().unwrap_or("")))
                .unwrap_or_else(|| format!("command {}", i + 1))
        );
        let mut leave = |why: &str| out.left_out.push((what.clone(), why.to_string()));
        let Some(Ok(tx)) = hex_of(c.get("hdr")) else {
            leave("no header (hdr)");
            continue;
        };
        if tx == 0x7DF {
            leave(
                "sent on 7DF, the broadcast every module hears: Icarus asks one module at a time",
            );
            continue;
        }
        let Some((service, ident)) = cmd.and_then(|m| {
            if m.len() != 1 {
                return None;
            }
            let (s, v) = m.iter().next()?;
            Some((s.clone(), v.as_str().unwrap_or("").to_string()))
        }) else {
            leave("no single command (cmd)");
            continue;
        };
        let (service, ident) = match (service.as_str(), ident.len()) {
            ("01", _) => {
                leave("standard OBD-II value: already in Icarus's live data");
                continue;
            }
            ("22", 4) | ("21", 2) => match u16::from_str_radix(&ident, 16) {
                Ok(id) => (if service == "22" { 0x22u8 } else { 0x21 }, id),
                Err(_) => {
                    leave("its identifier is not hex");
                    continue;
                }
            },
            ("22" | "21", _) => {
                leave("its identifier has the wrong length");
                continue;
            }
            (other, _) => {
                leave(&format!(
                    "service {other} is not one Icarus reads with (21, 22)"
                ));
                continue;
            }
        };
        let proto = c.get("proto").and_then(|p| p.as_str()).unwrap_or("");
        let ids = match proto {
            "9141-2" | "14230" => Ids::Kline,
            "15765-4-29bit" => Ids::Can29,
            _ if hdr_s.len() >= 4 => Ids::Can29,
            _ => Ids::Can11,
        };
        let top = super::id_top(ids);
        if tx > top {
            leave(&format!("header {tx:X} is not a {} ID", ids.word()));
            continue;
        }
        let rx_given = match hex_of(c.get("rax")) {
            Some(Ok(r)) if r <= top => Some(r),
            Some(Ok(r)) => {
                leave(&format!("answer ID (rax) {r:X} is not a {} ID", ids.word()));
                continue;
            }
            Some(Err(e)) => {
                leave(&format!("answer ID (rax): {e}"));
                continue;
            }
            None => None,
        };
        // an address byte that is not one byte is not truncated into another address
        let byte_of = |k: &str| match hex_of(c.get(k)) {
            None => Ok(None),
            Some(Ok(x)) => u8::try_from(x)
                .map(Some)
                .map_err(|_| format!("{k} {x:X} is not one byte")),
            Some(Err(e)) => Err(format!("{k}: {e}")),
        };
        let (ext, tester) = match (byte_of("eax"), byte_of("tst")) {
            (Ok(e), Ok(t)) => (e, t),
            (Err(e), _) | (_, Err(e)) => {
                leave(&e);
                continue;
            }
        };
        let years = match c
            .get("filter")
            .and_then(|f| f.as_object())
            .map(filter_years)
        {
            None => vec![],
            Some(Ok(y)) => y,
            Some(Err(e)) => {
                leave(&e);
                continue;
            }
        };
        let unverified = if c.get("dbg").and_then(|d| d.as_bool()).unwrap_or(false) {
            vec![[0, LAST_YEAR]]
        } else {
            match c
                .get("dbgfilter")
                .and_then(|f| f.as_object())
                .map(filter_years)
            {
                None => vec![],
                Some(Ok(y)) => y,
                Some(Err(e)) => {
                    leave(&format!("debug {e}"));
                    continue;
                }
            }
        };
        // tx is at most 1FFFFFFF here, so tx + 8 does not overflow
        let rx = rx_given.unwrap_or(tx + 8);
        let mut values: Vec<ValueDef> = Vec::new();
        for s in c
            .get("signals")
            .and_then(|s| s.as_array())
            .into_iter()
            .flatten()
        {
            let sid = s
                .get("id")
                .and_then(|v| v.as_str())
                .unwrap_or("?")
                .to_string();
            // each value is checked as it is added: a bad one leaves out only itself
            match signal(s).and_then(|v| super::check_value(&v).map(|()| v)) {
                Ok(v) if values.iter().any(|q| q.id == v.id) => out.left_out.push((
                    format!("{what}, signal {sid}"),
                    "its id is used twice in the command".into(),
                )),
                Ok(v) => {
                    if !v.note.is_empty() {
                        out.notes.push(format!("{what}, {sid}: {}", v.note));
                    }
                    values.push(v);
                }
                Err(e) => out.left_out.push((format!("{what}, signal {sid}"), e)),
            }
        }
        if values.is_empty() {
            out.left_out.push((what.clone(), "no values".into()));
            continue;
        }
        let mut r = RequestDef {
            service,
            ident,
            every: c.get("freq").and_then(|f| f.as_f64()).filter(|f| *f > 0.0),
            min_len: None,
            timeout_ms: c
                .get("tmo")
                .and_then(|t| t.as_str())
                .and_then(|t| u32::from_str_radix(t, 16).ok())
                .and_then(|t| t.checked_mul(4)),
            years,
            unverified,
            needs_session: c
                .get("din")
                .and_then(|d| d.as_str())
                .map(|d| format!("10 {d}")),
            sample: None,
            values,
        };
        if c.get("fcm1").and_then(|f| f.as_bool()).unwrap_or(false) {
            out.notes.push(format!(
                "{what}: asks for flow control from the request ID (fcm1), as Icarus always does"
            ));
        }
        let idx = match modules.iter().position(|m| {
            let h = &m.module;
            (h.ids, h.tx, h.rx(), h.ext, h.tester) == (ids, tx, rx, ext, tester)
        }) {
            Some(i) => i,
            None => {
                let kind = ecu_kind(&ecus, tx, rx, ext);
                if rx_given.is_none() && ids == Ids::Can11 {
                    out.notes.push(format!(
                        "{}: no answer ID given (rax); {:03X} assumed (request ID + 8)",
                        hdr_s,
                        tx + 8
                    ));
                }
                let label = if kind.is_empty() {
                    "module".to_string()
                } else {
                    kind.clone()
                };
                modules.push(ModuleDef {
                    format: FORMAT,
                    module: ModuleHead {
                        name: format!(
                            "{label} ({hdr_s}{})",
                            ext.map(|e| format!("+{e:02X}")).unwrap_or_default()
                        ),
                        tx,
                        rx: rx_given,
                        ext,
                        tester,
                        ids,
                        bus: None,
                        dialect: None,
                        kind,
                        identity: vec![],
                    },
                    requests: vec![],
                });
                modules.len() - 1
            }
        };
        let m = &mut modules[idx];
        if m.module.kind == "airbag" {
            out.left_out
                .push((what, "airbag module: identity only".into()));
            continue;
        }
        // keep value ids unique in the module: a later layout of the same request for other years renames its own
        // (ID_22_1F01, then ID_22_1F01_2... when there are more)
        let tag = r.label().replace(' ', "_");
        for v in &mut r.values {
            let taken = |id: &str| {
                m.requests
                    .iter()
                    .flat_map(|q| &q.values)
                    .any(|q| q.id == id)
            };
            if taken(&v.id) {
                let base = format!("{}_{tag}", v.id);
                let mut id = base.clone();
                let mut n = 2;
                while taken(&id) {
                    id = format!("{base}_{n}");
                    n += 1;
                }
                v.id = id;
            }
        }
        if m.requests.iter().any(|q| {
            (q.service, q.ident) == (r.service, r.ident) && super::years_overlap(&q.years, &r.years)
        }) {
            out.left_out
                .push((what, "the same request again for the same years".into()));
            continue;
        }
        let mut trial = m.clone();
        trial.requests.push(r);
        match validate(&mut trial) {
            Ok(()) => *m = trial,
            Err(e) => out.left_out.push((what, e)),
        }
    }
    // one phrase, matched as whole words (`F 150` fits an F-150, never a Focus or an F-250)
    let models: Vec<String> = if repo_model.is_empty() {
        vec![]
    } else {
        vec![repo_model]
    };
    let set = SetInfo {
        format: FORMAT,
        name,
        make,
        models,
        years: file_years(&opts.path),
        source: Source {
            from: "OBDb".into(),
            file: repo.map(|r| r.1).unwrap_or_else(|| {
                opts.path
                    .rsplit(['/', '\\'])
                    .next()
                    .unwrap_or("")
                    .to_string()
            }),
            sha256: sha256_hex(text.as_bytes()),
            imported: opts.today.clone(),
            license: LICENSE.into(),
            attribution: ATTRIBUTION.into(),
        },
    };
    super::check_set(&set)?;
    out.set = Some(set);
    out.modules = modules;
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::moddata::{decode, Shown};

    /// A signal set of a few lines written for these tests, with made-up identifiers in the 7A0 range (never OBDb's).
    const SET: &str = r#"{
  "ecu": [
    {"hdr": "7A0", "type": "transmission"},
    {"hdr": "7A4", "type": "airbag"},
    {"hdr": "7A6", "eax": "3C", "type": "tpms"},
    {"hdr": "7A6", "type": "body"}
  ],
  "commands": [
    {"hdr": "7A0", "rax": "7A8", "cmd": {"22": "4A01"}, "freq": 1, "filter": {"from": 2016, "to": 2023},
     "dbgfilter": {"from": 2020, "to": 2016},
     "signals": [
       {"id": "TOY_TEMP", "name": "Fluid temperature", "path": "Transmission", "description": "made up",
        "fmt": {"len": 8, "add": -40, "max": 215, "unit": "celsius", "nullmin": -40}},
       {"id": "TOY_SPD", "name": "Output speed", "fmt": {"bix": 8, "len": 16, "blsb": true, "unit": "rpm"}},
       {"id": "TOY_LOCK", "name": "Lock-up", "fmt": {"bix": 31, "len": 1, "unit": "offon"}},
       {"id": "TOY_MODE", "name": "Mode", "fmt": {"bix": 32, "len": 8,
          "map": {"1": {"description": "Normal", "value": "NORMAL"}, "2": {"description": "", "value": "SPORT"}}}},
       {"id": "TOY_FINE", "name": "Fine", "fmt": {"bix": 13, "len": 3}}
     ]},
    {"hdr": "7A0", "rax": "7A8", "cmd": {"21": "05"}, "freq": 0.5, "din": "03", "filter": {"years": [2017, 2019]},
     "signals": [{"id": "TOY_K", "name": "K", "fmt": {"len": 8, "mul": 1, "div": 4, "sign": true}}]},
    {"hdr": "7A2", "cmd": {"22": "4A02"}, "freq": 2, "dbg": true, "filter": {"from": 2018},
     "signals": [{"id": "TOY_V", "name": "Volts", "fmt": {"len": 8, "div": 10, "unit": "volts"}}]},
    {"hdr": "7A2", "cmd": {"22": "4A03"}, "freq": 2, "filter": {"to": 2017},
     "signals": [{"id": "TOY_W", "name": "W", "fmt": {"len": 8, "unit": "furlongs"}}]},
    {"hdr": "7DF", "cmd": {"22": "4A04"}, "freq": 1, "signals": [{"id": "A", "name": "A", "fmt": {"len": 8}}]},
    {"hdr": "7A0", "rax": "7A8", "cmd": {"01": "0C"}, "freq": 1, "signals": [{"id": "B", "name": "B", "fmt": {"len": 8}}]},
    {"hdr": "7A0", "rax": "7A8", "cmd": {"2F": "0101"}, "freq": 1, "signals": [{"id": "C", "name": "C", "fmt": {"len": 8}}]},
    {"hdr": "7A4", "rax": "7AC", "cmd": {"22": "4A05"}, "freq": 1, "signals": [{"id": "D", "name": "Belt", "fmt": {"len": 1}}]},
    {"hdr": "7A6", "rax": "7AE", "eax": "3C", "tst": "3C", "tmo": "96", "fcm1": true, "cmd": {"21": "30"}, "freq": 5,
     "signals": [{"id": "E", "name": "Tire", "fmt": {"len": 8, "unit": "kilopascal"}}]},
    {"hdr": "18DA", "cmd": {"22": "4A06"}, "freq": 1, "signals": [{"id": "F", "name": "F", "fmt": {"len": 8}}]},
    {"hdr": "7A3", "proto": "14230", "cmd": {"21": "01"}, "freq": 1, "signals": [{"id": "G", "name": "G", "fmt": {"len": 8, "unit": "ascii"}}]}
  ]
}"#;

    fn import() -> Imported {
        parse(
            SET,
            &Options {
                path: "/home/someone/Downloads/Toy-Truck/signalsets/v3/default.json".into(),
                today: "2026-11-14".into(),
                ..Default::default()
            },
        )
        .unwrap()
    }

    fn left_out(i: &Imported, what: &str) -> String {
        i.left_out
            .iter()
            .find(|(w, _)| w.contains(what))
            .map(|(_, why)| why.clone())
            .unwrap_or_else(|| panic!("{what} not left out: {:?}", i.left_out))
    }

    #[test]
    fn the_set_is_named_from_the_folder_and_attributed() {
        let i = import();
        let s = i.set.unwrap();
        assert_eq!((s.name.as_str(), s.make.as_str()), ("toy-truck", "toy"));
        assert_eq!(s.models, vec!["Truck"]);
        assert_eq!(s.source.file, "Toy-Truck/signalsets/v3/default.json");
        assert_eq!(
            (s.source.from.as_str(), s.source.license.as_str()),
            ("OBDb", LICENSE)
        );
        assert!(s.source.attribution.contains("OBDb"));
        assert_eq!(s.source.sha256.len(), 64);
        assert!(s.years.is_empty());
        assert!(parse(
            SET,
            &Options {
                path: "x.json".into(),
                ..Default::default()
            }
        )
        .unwrap_err()
        .contains("--name"));
        let named = parse(
            SET,
            &Options {
                path: "2016-2023.json".into(),
                name: Some("mine".into()),
                ..Default::default()
            },
        )
        .unwrap();
        assert_eq!(named.set.unwrap().years, vec![[2016, 2023]]);
    }

    #[test]
    fn a_22_and_a_21_command_become_requests_and_values() {
        let i = import();
        let m = i.modules.iter().find(|m| m.module.tx == 0x7A0).unwrap();
        assert_eq!(
            (m.module.rx(), m.module.kind.as_str()),
            (0x7A8, "transmission")
        );
        assert_eq!(m.requests.len(), 2);
        let r = &m.requests[0];
        assert_eq!((r.service, r.ident, r.every), (0x22, 0x4A01, Some(1.0)));
        assert_eq!(r.years, vec![[2016, 2023]]);
        assert_eq!(
            r.unverified,
            vec![[0, 2016], [2020, 9999]],
            "a wrapping filter"
        );
        assert_eq!(r.min_len, Some(5));
        let t = &r.values[0];
        assert_eq!(
            (t.unit.as_str(), t.add, t.min, t.max),
            ("°C", -40.0, Some(0.0), Some(215.0)),
            "min 0 with max only"
        );
        assert_eq!(
            (t.path.as_str(), t.about.as_str()),
            ("Transmission", "made up")
        );
        assert_eq!(r.values[1].order, Order::LsbBytes);
        let lock = &r.values[2];
        assert_eq!((lock.kind, lock.byte, lock.bit), (ValueKind::State, 3, 7));
        assert_eq!(lock.states["1"], "on");
        assert!(lock.note.contains("assumed"));
        let mode = &r.values[3];
        assert_eq!(
            (mode.states["1"].as_str(), mode.states["2"].as_str()),
            ("Normal", "SPORT")
        );
        let fine = &r.values[4];
        assert_eq!(
            (fine.byte, fine.bit, fine.bits),
            (1, 5, 3),
            "bix 13 is byte 1, bit 5"
        );
        let k = &m.requests[1];
        assert_eq!(
            (k.service, k.ident, k.needs_session.as_deref()),
            (0x21, 0x05, Some("10 03"))
        );
        assert_eq!(k.years, vec![[2017, 2017], [2019, 2019]]);
        assert!(k.values[0].signed && k.values[0].div == 4.0);
    }

    #[test]
    fn a_hand_made_answer_decodes() {
        let i = import();
        let r = &i
            .modules
            .iter()
            .find(|m| m.module.tx == 0x7A0)
            .unwrap()
            .requests[0];
        // after "62 4A 01": 90 °C, output speed E8 03 (blsb: 0x03E8 = 1000), byte 2, lock-up bit set, mode 2
        let data = [130, 0xE8, 0x03, 0x01, 0x02];
        let got: Vec<Shown> = r
            .values
            .iter()
            .map(|v| decode(v, &data).unwrap().shown)
            .collect();
        assert_eq!(
            got,
            vec![
                Shown::Number(90.0),
                Shown::Number(1000.0),
                Shown::State("on".into()),
                Shown::State("SPORT".into()),
                Shown::Number(0.0),
            ]
        );
        // as OBDb reads it: min defaults to 0 with only a max, so -40 is clamped to 0 (marked) before the null bound
        let cold = decode(&r.values[0], &[0, 0, 0, 0, 0]).unwrap();
        assert_eq!((cold.shown, cold.at_limit), (Shown::Number(0.0), true));
    }

    #[test]
    fn what_is_left_out_says_why() {
        let i = import();
        assert!(left_out(&i, "7DF").contains("broadcast"));
        assert!(left_out(&i, "7A0 01 0C").contains("standard OBD-II"));
        assert!(left_out(&i, "7A0 2F").contains("not one Icarus reads with"));
        assert_eq!(left_out(&i, "7A4 22 4A05"), "airbag module: identity only");
        let bag = i.modules.iter().find(|m| m.module.tx == 0x7A4).unwrap();
        assert!(
            bag.requests.is_empty() && bag.module.kind == "airbag",
            "kept for its identity, no data requests"
        );
    }

    #[test]
    fn modules_icarus_cannot_reach_yet_are_kept_and_marked() {
        let i = import();
        let tpms = i.modules.iter().find(|m| m.module.tx == 0x7A6).unwrap();
        assert_eq!(
            (
                tpms.module.ext,
                tpms.module.tester,
                tpms.module.kind.as_str()
            ),
            (Some(0x3C), Some(0x3C), "tpms")
        );
        assert_eq!(tpms.requests[0].timeout_ms, Some(600));
        assert!(i.notes.iter().any(|n| n.contains("fcm1")));
        let can29 = i.modules.iter().find(|m| m.module.tx == 0x18DA).unwrap();
        assert_eq!(can29.module.ids, Ids::Can29);
        let k = i.modules.iter().find(|m| m.module.tx == 0x7A3).unwrap();
        assert_eq!(k.module.ids, Ids::Kline);
        assert_eq!(k.requests[0].values[0].kind, ValueKind::Text);
        // no rax: tx + 8, noted
        let v = i.modules.iter().find(|m| m.module.tx == 0x7A2).unwrap();
        assert_eq!((v.module.rx, v.module.rx()), (None, 0x7AA));
        assert!(i.notes.iter().any(|n| n.contains("7AA assumed")));
        assert_eq!(v.requests[0].years, vec![[2018, 9999]], "from only");
        assert_eq!(v.requests[0].unverified, vec![[0, 9999]], "dbg: every year");
        assert_eq!(v.requests[1].years, vec![[0, 2017]], "to only");
        assert_eq!(v.requests[1].values[0].unit, "furlongs");
        assert!(i.notes.iter().any(|n| n.contains("furlongs")));
        assert_eq!(v.requests[0].values[0].unit, "V");
    }

    #[test]
    fn filters() {
        let r = |j: &str| {
            filter_years(
                serde_json::from_str::<Value>(j)
                    .unwrap()
                    .as_object()
                    .unwrap(),
            )
        };
        let f = |j: &str| r(j).unwrap();
        assert_eq!(f(r#"{"from": 2016, "to": 2023}"#), vec![[2016, 2023]]);
        assert_eq!(
            f(r#"{"from": 2024, "to": 2015}"#),
            vec![[0, 2015], [2024, 9999]]
        );
        assert_eq!(f(r#"{"from": 2016}"#), vec![[2016, 9999]]);
        assert_eq!(f(r#"{"to": 2016}"#), vec![[0, 2016]]);
        assert_eq!(f(r#"{"years": [2017]}"#), vec![[2017, 2017]]);
        assert_eq!(f(r#"{}"#), Vec::<[u16; 2]>::new());
        // years that are not years: refused, never truncated (70000 as u16 is 4464) or dropped (which would widen it)
        for bad in [
            r#"{"from": 70000, "to": 2020}"#,
            r#"{"to": 65536}"#,
            r#"{"from": 10000}"#,
            r#"{"from": -1}"#,
            r#"{"years": [2017, 70000]}"#,
            r#"{"years": ["2017"]}"#,
        ] {
            assert!(r(bad).unwrap_err().contains("is not a year"), "{bad}");
        }
    }

    /// A one-command signal set around `signals` (JSON), on 7A0.
    fn one(cmd: &str) -> Imported {
        parse(
            &format!(r#"{{"commands": [{cmd}]}}"#),
            &Options {
                path: "Toy-Truck/signalsets/v3/default.json".into(),
                ..Default::default()
            },
        )
        .unwrap()
    }

    #[test]
    fn malformed_commands_are_left_out_not_a_panic() {
        let sig = r#""signals": [{"id": "A", "name": "A", "fmt": {"len": 8}}]"#;
        for (cmd, why) in [
            // tx + 8 would overflow
            (
                format!(r#"{{"hdr": "FFFFFFFF", "cmd": {{"22": "4A01"}}, {sig}}}"#),
                "not a can29 ID",
            ),
            (
                format!(r#"{{"hdr": "7A0", "rax": "FFF", "cmd": {{"22": "4A01"}}, {sig}}}"#),
                "not a can11 ID",
            ),
            (
                format!(r#"{{"hdr": "7A0", "eax": "1FF", "cmd": {{"22": "4A01"}}, {sig}}}"#),
                "not one byte",
            ),
            (
                format!(r#"{{"hdr": "7A0", "tst": "100", "cmd": {{"22": "4A01"}}, {sig}}}"#),
                "not one byte",
            ),
            (
                format!(
                    r#"{{"hdr": "7A0", "cmd": {{"22": "4A01"}}, "filter": {{"from": 70000}}, {sig}}}"#
                ),
                "not a year",
            ),
            (
                format!(
                    r#"{{"hdr": "7A0", "cmd": {{"22": "4A01"}}, "dbgfilter": {{"to": 99999}}, {sig}}}"#
                ),
                "not a year",
            ),
        ] {
            let i = one(&cmd);
            assert!(i.modules.iter().all(|m| m.requests.is_empty()), "{cmd}");
            assert!(
                left_out(&i, "4A01").contains(why),
                "{cmd}: {:?}",
                i.left_out
            );
        }
        // a timeout too large to multiply by 4 is not a panic either
        let i = one(&format!(
            r#"{{"hdr": "7A0", "tmo": "FFFFFFFF", "cmd": {{"22": "4A01"}}, {sig}}}"#
        ));
        assert_eq!(i.modules[0].requests[0].timeout_ms, None);
    }

    #[test]
    fn one_bad_signal_leaves_out_only_itself() {
        let i = one(r#"{"hdr": "7A0", "cmd": {"22": "4A01"}, "signals": [
              {"id": "GOOD", "name": "good", "fmt": {"len": 8}},
              {"id": "WIDE", "name": "128 bits", "fmt": {"bix": 8, "len": 128}},
              {"id": "GOOD", "name": "the same id again", "fmt": {"bix": 8, "len": 8}},
              {"id": "FAR", "name": "far", "fmt": {"bix": 18446744073709551615, "len": 8}},
              {"id": "FAR2", "name": "far", "fmt": {"bix": 2305843009213693944, "len": 8}},
              {"id": "MAP", "name": "map", "fmt": {"bix": 16, "len": 1, "map": {"0": "off", "1": "on", "7": "never"}}},
              {"id": "OK2", "name": "fine", "fmt": {"bix": 24, "len": 8}}
            ]}"#);
        let r = &i.modules[0].requests[0];
        let ids: Vec<&str> = r.values.iter().map(|v| v.id.as_str()).collect();
        assert_eq!(ids, vec!["GOOD", "MAP", "OK2"]);
        assert!(left_out(&i, "signal WIDE").contains("1-64"));
        assert!(left_out(&i, "signal GOOD").contains("used twice"));
        assert!(left_out(&i, "signal FAR").contains("past the 4095"));
        assert!(left_out(&i, "signal FAR2").contains("past the 4095"));
        let map = &r.values[1];
        assert_eq!(map.states.len(), 2);
        assert!(map.note.contains("left out"), "{}", map.note);
    }

    #[test]
    fn three_layouts_of_one_request_keep_their_values_apart() {
        let cmd = |y: &str| {
            format!(
                r#"{{"hdr": "7A0", "cmd": {{"22": "4A01"}}, "filter": {y}, "signals": [{{"id": "A", "name": "A", "fmt": {{"len": 8}}}}]}}"#
            )
        };
        let i = one(&[
            cmd(r#"{"from": 2010, "to": 2012}"#),
            cmd(r#"{"from": 2013, "to": 2015}"#),
            cmd(r#"{"from": 2016, "to": 2018}"#),
        ]
        .join(","));
        let ids: Vec<&str> = i.modules[0]
            .requests
            .iter()
            .flat_map(|r| &r.values)
            .map(|v| v.id.as_str())
            .collect();
        assert_eq!(ids, vec!["A", "A_22_4A01", "A_22_4A01_2"]);
    }

    #[test]
    fn make_and_model_come_from_the_folder_as_whole_phrases() {
        assert_eq!(
            make_and_model("Toyota-Tacoma"),
            ("toyota".into(), "Tacoma".into())
        );
        assert_eq!(
            make_and_model("Ford-F-150"),
            ("ford".into(), "F 150".into())
        );
        assert_eq!(
            make_and_model("Mercedes-Benz-Sprinter"),
            ("mercedes-benz".into(), "Sprinter".into())
        );
        assert_eq!(
            make_and_model("Land-Rover-Range-Rover"),
            ("land-rover".into(), "Range Rover".into())
        );
        assert_eq!(make_and_model("Toyota"), ("toyota".into(), String::new()));
        let i = parse(
            SET,
            &Options {
                path: "Ford-F-150/signalsets/v3/default.json".into(),
                ..Default::default()
            },
        )
        .unwrap();
        let s = i.set.unwrap();
        assert_eq!(
            (s.make.as_str(), s.models),
            ("ford", vec!["F 150".to_string()])
        );
    }

    #[test]
    fn the_import_writes_files_icarus_reads_back() {
        let i = import();
        let files = crate::moddata::set_files(i.set.as_ref().unwrap(), &i.modules);
        assert_eq!(
            crate::moddata::parse_set(&files[0].1).unwrap(),
            i.set.clone().unwrap()
        );
        for (name, text) in &files[1..] {
            let m = crate::moddata::parse_module(text, name).unwrap();
            assert!(i.modules.contains(&m), "{name}");
        }
        assert!(files.iter().any(|f| f.0 == "7A6.3C.toml"));
    }
}
