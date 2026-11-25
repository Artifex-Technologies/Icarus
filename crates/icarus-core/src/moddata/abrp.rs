//! A Better Routeplanner's EV PID lists (github.com/Iternio-Planning-AB/ev-obd-pids, Apache-2.0 unless a folder's own licence
//! says otherwise), imported from a file the user downloaded: battery current, voltage, state of charge and health,
//! temperatures of electric and hybrid vehicles (Hyundai/Kia, GM, Ford, VW, Renault, ...). Icarus ships none of the lists.
//!
//! Each entry is `{"equation": "((A*256)+B)/100", "command": "222885", "ecu": "7E1", ...}` with Torque Pro's byte letters
//! (A is the first byte after the identifier's echo; AA follows Z). Icarus keeps an equation only when it is one of its own
//! values, which it finds out by evaluating the equation: a number from consecutive bytes (unsigned or signed, most
//! significant first) scaled by `x * mul + add`, checked at many raw values, or one bit (`{A:2}`). Anything else (a
//! yes/no test over several values, an equation that is not linear) is left out with the reason, as are commands that are
//! not `21`/`22` requests to one 11-bit module (29-bit IDs and raw CAN frames are not reachable yet).
//!
//! The module: the entry's `ecu` when it is a request ID (7E0-7E7, any 11-bit ID below 7E8); an answer ID 7E8-7EF becomes
//! its request ID (minus 8, noted); no `ecu`: the last `ATSH` before the command in `data_commands`.

use super::{
    sha256_hex, validate, Imported, ModuleDef, ModuleHead, RequestDef, SetInfo, Source, ValueDef,
    FORMAT,
};
use serde_json::Value;
use std::collections::BTreeMap;

pub const LICENSE: &str = "Apache-2.0";
pub const ATTRIBUTION: &str =
    "PID list: A Better Routeplanner's ev-obd-pids (github.com/Iternio-Planning-AB/ev-obd-pids), Apache-2.0";

#[derive(Clone, Debug, Default)]
pub struct Options {
    /// The path the file was read from (`.../hkmc/Ioniq5.json`).
    pub path: String,
    pub name: Option<String>,
    /// The vehicle profile it is for (`--make`); else guessed from the folder.
    pub make: Option<String>,
    /// Words matched against the vehicle's model (`--model "Ioniq 5"`); none: every model of the make.
    pub model: Option<String>,
    pub today: String,
}

/// ABRP's folder -> Icarus's vehicle profile.
fn make_of_folder(folder: &str) -> Option<&'static str> {
    Some(match folder.to_lowercase().as_str() {
        "hkmc" => "hyundai-kia",
        "gmc" => "gm",
        "volkswagen" => "vw-audi",
        "ford" => "ford",
        "mazda" => "mazda",
        "honda" => "honda",
        _ => return None,
    })
}

/// ABRP's names for the values it reads, in words.
fn name_of(key: &str) -> (String, &'static str) {
    let (n, u) = match key {
        "soc" => ("State of charge", "%"),
        "soh" => ("State of health", "%"),
        "soe" => ("Energy left", "kWh"),
        "current" => ("Battery current", "A"),
        "voltage" => ("Battery voltage", "V"),
        "batt_temp" => ("Battery temperature", "°C"),
        "ext_temp" => ("Outside temperature", "°C"),
        "capacity" => ("Battery capacity", "kWh"),
        "kwh_charged" => ("Energy charged (total)", "kWh"),
        "odometer" => ("Odometer", "km"),
        "speed" | "vehicle_reported_speed" => ("Vehicle speed", "km/h"),
        _ => ("", ""),
    };
    if n.is_empty() {
        let words = key.trim_matches('_').replace('_', " ");
        let mut c = words.chars();
        let name = c
            .next()
            .map(|f| f.to_uppercase().chain(c).collect())
            .unwrap_or_default();
        (name, "")
    } else {
        (n.to_string(), u)
    }
}

// ---- Torque-style equations ----

#[derive(Clone, Debug, PartialEq)]
enum Tok {
    Num(f64),
    Var(usize),
    Bit(usize, u8),
    Func(String),
    Op(&'static str),
    Open,
    Close,
    Comma,
}

/// A, B ... Z, then AA ... AZ, BA ...: the byte's index after the echo (any case).
fn var_index(s: &str) -> Option<usize> {
    let b: Vec<u8> = s.bytes().map(|c| c.to_ascii_uppercase()).collect();
    if b.is_empty() || b.len() > 3 || !b.iter().all(u8::is_ascii_uppercase) {
        return None;
    }
    Some(
        b.iter()
            .fold(0usize, |acc, c| acc * 26 + (c - b'A' + 1) as usize)
            - 1,
    )
}

fn lex(eq: &str) -> Result<Vec<Tok>, String> {
    let c: Vec<char> = eq.chars().collect();
    let mut i = 0;
    let mut out = Vec::new();
    while i < c.len() {
        let ch = c[i];
        if ch.is_whitespace() {
            i += 1;
        } else if ch.is_ascii_digit() || ch == '.' {
            let s = i;
            while i < c.len() && (c[i].is_ascii_digit() || c[i] == '.') {
                i += 1;
            }
            let t: String = c[s..i].iter().collect();
            out.push(Tok::Num(
                t.parse().map_err(|_| format!("{t:?} is not a number"))?,
            ));
        } else if ch.is_ascii_alphabetic() {
            let s = i;
            while i < c.len() && c[i].is_ascii_alphanumeric() {
                i += 1;
            }
            let w: String = c[s..i].iter().collect();
            if c.get(i) == Some(&'(') {
                out.push(Tok::Func(w.to_lowercase()));
            } else {
                out.push(Tok::Var(
                    var_index(&w).ok_or_else(|| format!("{w:?} is not a byte letter"))?,
                ));
            }
        } else if ch == '{' {
            let end = c[i..]
                .iter()
                .position(|x| *x == '}')
                .ok_or("a { without }")?
                + i;
            let inner: String = c[i + 1..end].iter().collect();
            let (v, n) = inner.split_once(':').ok_or("{A:n} needs a bit number")?;
            let bit: u8 = n
                .trim()
                .parse()
                .map_err(|_| format!("{n:?} is not a bit number"))?;
            if bit > 7 {
                return Err(format!("bit {bit} of a byte"));
            }
            out.push(Tok::Bit(
                var_index(v.trim()).ok_or_else(|| format!("{v:?} is not a byte letter"))?,
                bit,
            ));
            i = end + 1;
        } else {
            let two: String = c[i..(i + 2).min(c.len())].iter().collect();
            let op = ["<<", ">>", "&&", "||", "==", "<=", ">=", "!="]
                .into_iter()
                .find(|o| *o == two);
            if let Some(o) = op {
                out.push(Tok::Op(o));
                i += 2;
                continue;
            }
            out.push(match ch {
                '(' => Tok::Open,
                ')' => Tok::Close,
                // Int16(A:B), as some lists write it
                ',' | ':' => Tok::Comma,
                '+' => Tok::Op("+"),
                '-' => Tok::Op("-"),
                '*' => Tok::Op("*"),
                '/' => Tok::Op("/"),
                '<' => Tok::Op("<"),
                '>' => Tok::Op(">"),
                '!' => Tok::Op("!"),
                _ => return Err(format!("{ch:?} in an equation")),
            });
            i += 1;
        }
    }
    Ok(out)
}

#[derive(Clone, Debug)]
enum Ex {
    Num(f64),
    Var(usize),
    Bit(usize, u8),
    Neg(Box<Ex>),
    Not(Box<Ex>),
    Bin(&'static str, Box<Ex>, Box<Ex>),
    Call(String, Vec<Ex>),
}

/// C's precedence, loosest first (Torque's equations are written for it).
const LEVELS: &[&[&str]] = &[
    &["||"],
    &["&&"],
    &["==", "!="],
    &["<", ">", "<=", ">="],
    &["<<", ">>"],
    &["+", "-"],
    &["*", "/"],
];

struct Parser {
    t: Vec<Tok>,
    i: usize,
}

impl Parser {
    fn peek(&self) -> Option<&Tok> {
        self.t.get(self.i)
    }
    fn level(&mut self, n: usize) -> Result<Ex, String> {
        if n == LEVELS.len() {
            return self.unary();
        }
        let mut l = self.level(n + 1)?;
        while let Some(Tok::Op(o)) = self.peek().cloned() {
            if !LEVELS[n].contains(&o) {
                break;
            }
            self.i += 1;
            let r = self.level(n + 1)?;
            l = Ex::Bin(o, Box::new(l), Box::new(r));
        }
        Ok(l)
    }
    fn unary(&mut self) -> Result<Ex, String> {
        if self.peek() == Some(&Tok::Op("-")) {
            self.i += 1;
            return Ok(Ex::Neg(Box::new(self.unary()?)));
        }
        if self.peek() == Some(&Tok::Op("!")) {
            self.i += 1;
            return Ok(Ex::Not(Box::new(self.unary()?)));
        }
        let t = self.peek().cloned().ok_or("the equation ends too soon")?;
        self.i += 1;
        Ok(match t {
            Tok::Num(v) => Ex::Num(v),
            Tok::Var(v) => Ex::Var(v),
            Tok::Bit(v, b) => Ex::Bit(v, b),
            Tok::Open => {
                let e = self.level(0)?;
                if self.peek() != Some(&Tok::Close) {
                    return Err("a ( without )".into());
                }
                self.i += 1;
                e
            }
            Tok::Func(f) => {
                self.i += 1; // its (
                let mut args = vec![self.level(0)?];
                while self.peek() == Some(&Tok::Comma) {
                    self.i += 1;
                    args.push(self.level(0)?);
                }
                if self.peek() != Some(&Tok::Close) {
                    return Err(format!("{f}( without )"));
                }
                self.i += 1;
                Ex::Call(f, args)
            }
            other => return Err(format!("{other:?} where a value belongs")),
        })
    }
}

fn parse_eq(eq: &str) -> Result<Ex, String> {
    let mut p = Parser { t: lex(eq)?, i: 0 };
    let e = p.level(0)?;
    if p.i != p.t.len() {
        return Err("more after the end of the equation".into());
    }
    Ok(e)
}

fn eval(e: &Ex, d: &[u8]) -> Result<f64, String> {
    let byte = |i: usize| d.get(i).map(|b| *b as f64).ok_or("past the answer's end");
    Ok(match e {
        Ex::Num(v) => *v,
        Ex::Var(i) => byte(*i)?,
        Ex::Bit(i, b) => ((byte(*i)? as u8 >> b) & 1) as f64,
        Ex::Neg(x) => -eval(x, d)?,
        Ex::Not(x) => {
            if eval(x, d)? == 0.0 {
                1.0
            } else {
                0.0
            }
        }
        Ex::Bin(o, a, b) => {
            let (x, y) = (eval(a, d)?, eval(b, d)?);
            let t = |c: bool| if c { 1.0 } else { 0.0 };
            match *o {
                "+" => x + y,
                "-" => x - y,
                "*" => x * y,
                "/" => {
                    if y == 0.0 {
                        return Err("divides by 0".into());
                    }
                    x / y
                }
                "<<" => ((x as i64) << (y as u32 & 63)) as f64,
                ">>" => ((x as i64) >> (y as u32 & 63)) as f64,
                "&&" => t(x != 0.0 && y != 0.0),
                "||" => t(x != 0.0 || y != 0.0),
                "==" => t(x == y),
                "!=" => t(x != y),
                "<" => t(x < y),
                ">" => t(x > y),
                "<=" => t(x <= y),
                _ => t(x >= y),
            }
        }
        Ex::Call(f, a) => {
            let v: Vec<f64> = a.iter().map(|x| eval(x, d)).collect::<Result<_, _>>()?;
            match (f.as_str(), v.as_slice()) {
                ("signed", [x]) => (*x as i64 as u8 as i8) as f64,
                ("int16", [a, b]) => a * 256.0 + b,
                ("int24", [a, b, c]) => a * 65536.0 + b * 256.0 + c,
                _ => {
                    return Err(format!(
                        "{f}() with {} argument(s) is not one Icarus knows",
                        v.len()
                    ))
                }
            }
        }
    })
}

fn vars(e: &Ex, out: &mut Vec<usize>) {
    match e {
        Ex::Var(i) | Ex::Bit(i, _) => out.push(*i),
        Ex::Neg(x) | Ex::Not(x) => vars(x, out),
        Ex::Bin(_, a, b) => {
            vars(a, out);
            vars(b, out);
        }
        Ex::Call(_, a) => a.iter().for_each(|x| vars(x, out)),
        Ex::Num(_) => {}
    }
}

/// What an equation is in Icarus's terms: a value over bytes `lo ..` (a number, or one bit), or why it is not one.
fn as_value(eq: &str) -> Result<ValueDef, String> {
    let e = parse_eq(eq)?;
    let mut used = Vec::new();
    vars(&e, &mut used);
    used.sort_unstable();
    used.dedup();
    let (Some(&lo), Some(&hi)) = (used.first(), used.last()) else {
        return Err("reads no byte".into());
    };
    let n = hi - lo + 1;
    if n > 4 || used.len() != n {
        return Err("not one number from consecutive bytes".into());
    }
    let with = |raw: u64| -> Result<f64, String> {
        let mut d = vec![0u8; hi + 1];
        for k in 0..n {
            d[lo + k] = (raw >> (8 * (n - 1 - k))) as u8;
        }
        eval(&e, &d)
    };
    let bits = 8 * n as u32;
    let top = (1u64 << bits) - 1;
    let probes: Vec<u64> = if n == 1 {
        (0..=255).collect()
    } else {
        let mut p = vec![
            0,
            1,
            2,
            255,
            256,
            1000,
            top / 3,
            top / 2,
            top / 2 + 1,
            top - 1,
            top,
        ];
        p.extend((1..40).map(|k| top / 40 * k + k));
        p
    };
    let ys: Vec<f64> = probes.iter().map(|r| with(*r)).collect::<Result<_, _>>()?;
    // one bit: the answer is that bit of the one byte, at every raw value
    if n == 1 {
        for b in 0..8u8 {
            if probes
                .iter()
                .zip(&ys)
                .all(|(r, y)| *y == ((r >> b) & 1) as f64)
            {
                let mut v = ValueDef::new("", "", lo, 1);
                v.bit = 7 - b;
                v.kind = super::ValueKind::State;
                v.states = [
                    ("0".to_string(), "no".to_string()),
                    ("1".to_string(), "yes".to_string()),
                ]
                .into();
                return Ok(v);
            }
        }
    }
    let close = |a: f64, b: f64| (a - b).abs() <= 1e-6 * (1.0 + a.abs().max(b.abs()));
    for signed in [false, true] {
        let x = |r: u64| -> f64 {
            if signed && r >> (bits - 1) == 1 {
                r as f64 - (1u64 << bits) as f64
            } else {
                r as f64
            }
        };
        // two points that differ in x give the line; every probe must lie on it
        let (x0, x1) = (x(0), x(1));
        let (y0, y1) = (with(0)?, with(1)?);
        let mul = (y1 - y0) / (x1 - x0);
        let add = y0 - mul * x0;
        if mul != 0.0
            && probes
                .iter()
                .zip(&ys)
                .all(|(r, y)| close(*y, mul * x(*r) + add))
        {
            let mut v = ValueDef::new("", "", lo, bits as u16);
            v.signed = signed;
            v.mul = mul;
            v.add = add;
            return Ok(v);
        }
    }
    // a yes/no test over one byte (`(A==1)||(A==2)`, `!{A:1}&&{A:2}`): a state value naming every raw value yes or no,
    // read off the equation at all 256 (every probe of a one-byte equation is one of them)
    if n == 1 && ys.iter().all(|y| *y == 0.0 || *y == 1.0) && ys.contains(&1.0) && ys.contains(&0.0)
    {
        let mut v = ValueDef::new("", "", lo, 8);
        v.kind = super::ValueKind::State;
        v.states = probes
            .iter()
            .zip(&ys)
            .map(|(r, y)| {
                (
                    r.to_string(),
                    if *y == 1.0 { "yes" } else { "no" }.to_string(),
                )
            })
            .collect();
        return Ok(v);
    }
    if ys.iter().all(|y| *y == 0.0 || *y == 1.0) {
        return Err("a yes/no test over several bytes: not imported yet".into());
    }
    Err("not a scaled number Icarus reads (not linear in its bytes)".into())
}

// ---- the file ----

/// 7E0-7E7 are the request IDs OBD-II gives emissions-related modules (ISO 15765-4: engine, transmission, the hybrid or
/// battery system), never an airbag module; any other ID's kind is left unknown, for the airbag gate to judge.
fn obd_kind(tx: u32) -> String {
    if (0x7E0..=0x7E7).contains(&tx) {
        "powertrain".into()
    } else {
        String::new()
    }
}

fn hex_u32(s: &str) -> Option<u32> {
    u32::from_str_radix(s.trim(), 16).ok()
}

/// The request ID each data command goes to: the last `ATSH` before it.
fn headers(root: &Value) -> BTreeMap<String, String> {
    let mut out = BTreeMap::new();
    let mut hdr = String::new();
    // an ATSH among the start-up commands holds until data_commands sets another
    let init = root
        .pointer("/init_commands/command")
        .and_then(Value::as_array);
    let cmds = root
        .pointer("/data_commands/command")
        .and_then(Value::as_array);
    for c in init.into_iter().flatten().filter_map(Value::as_str) {
        let t: String = c
            .chars()
            .filter(|x| !x.is_whitespace())
            .collect::<String>()
            .to_uppercase();
        if let Some(h) = t.strip_prefix("ATSH") {
            hdr = h.to_string();
        }
    }
    for c in cmds.into_iter().flatten().filter_map(Value::as_str) {
        let t: String = c
            .chars()
            .filter(|x| !x.is_whitespace())
            .collect::<String>()
            .to_uppercase();
        if let Some(h) = t.strip_prefix("ATSH") {
            hdr = h.to_string();
        } else if !t.starts_with("AT") {
            out.insert(t, hdr.clone());
        }
    }
    out
}

pub fn parse(text: &str, opts: &Options) -> Result<Imported, String> {
    let root: Value = serde_json::from_str(text).map_err(|e| format!("not JSON: {e}"))?;
    let obj = root
        .as_object()
        .ok_or("not an ABRP PID list (an object of values)")?;
    let parts: Vec<&str> = opts
        .path
        .split(['/', '\\'])
        .filter(|p| !p.is_empty())
        .collect();
    let stem = parts
        .last()
        .map(|f| f.trim_end_matches(".json"))
        .unwrap_or("");
    let folder = parts.len().checked_sub(2).map(|i| parts[i]).unwrap_or("");
    let name = opts.name.clone().unwrap_or_else(|| {
        let words: Vec<String> = stem
            .to_lowercase()
            .split(|c: char| !c.is_ascii_alphanumeric())
            .filter(|w| !w.is_empty())
            .map(str::to_string)
            .collect();
        format!("abrp-{}", words.join("-"))
    });
    let make = opts
        .make
        .clone()
        .or_else(|| make_of_folder(folder).map(str::to_string))
        .unwrap_or_default();
    let mut out = Imported::default();
    if make.is_empty() {
        out.notes.push(format!(
            "no vehicle profile for the folder {folder:?}: the set is used only with --set (or import with --make NAME)"
        ));
    }
    if opts.model.is_none() {
        out.notes
            .push("no --model: the set is offered for every model of the make".into());
    }
    let sh = headers(&root);
    let mut modules: Vec<ModuleDef> = Vec::new();
    for (key, v) in obj {
        let Some(eq) = v.get("equation").and_then(Value::as_str) else {
            continue; // init_commands, obd_protocol, data_commands
        };
        let cmd: String = v
            .get("command")
            .and_then(Value::as_str)
            .unwrap_or("")
            .chars()
            .filter(|c| !c.is_whitespace())
            .collect::<String>()
            .to_uppercase();
        let what = format!("{key} ({cmd})");
        let mut leave = |why: String| out.left_out.push((what.clone(), why));
        let (service, ident) = match (cmd.get(..2), cmd.len()) {
            (Some("22"), 6) => (0x22u8, hex_u32(&cmd[2..])),
            (Some("21"), 4) => (0x21, hex_u32(&cmd[2..])),
            _ => {
                leave("not a 21 or 22 request (a raw CAN frame, or another service)".into());
                continue;
            }
        };
        let Some(ident) = ident.map(|i| i as u16) else {
            leave("its identifier is not hex".into());
            continue;
        };
        let ecu = v
            .get("ecu")
            .and_then(Value::as_str)
            .unwrap_or("")
            .trim()
            .to_uppercase();
        let ecu = if ecu.is_empty() || ecu == "NONE" {
            sh.get(&cmd).cloned().unwrap_or_default()
        } else {
            ecu
        };
        let tx = match hex_u32(&ecu) {
            Some(id) if ecu.len() <= 3 && (0x7E8..=0x7EF).contains(&id) => {
                out.notes.push(format!(
                    "{what}: ecu {ecu} is an answer ID; request ID {:03X} used",
                    id - 8
                ));
                id - 8
            }
            Some(id) if ecu.len() <= 3 && id < 0x7E8 && id != 0x7DF => id,
            _ => {
                leave(format!(
                    "module {ecu:?}: Icarus reaches one 11-bit module at a time (29-bit IDs not yet)"
                ));
                continue;
            }
        };
        let mut val = match as_value(eq) {
            Ok(x) => x,
            Err(why) => {
                leave(format!("{eq:?}: {why}"));
                continue;
            }
        };
        let (label, unit) = name_of(key);
        val.id = key.trim_matches('_').to_uppercase();
        val.name = label;
        val.unit = unit.to_string();
        val.about = format!("ABRP's {key}: {eq}");
        if val.kind == super::ValueKind::Number {
            let num = |k: &str| {
                v.get(k)
                    .and_then(Value::as_str)
                    .and_then(|s| s.trim().parse::<f64>().ok())
            };
            if let (Some(a), Some(b)) = (num("minValue"), num("maxValue")) {
                if b > a {
                    // ABRP's range is what is plausible, not the raw limits: kept as a note, not a clamp
                    val.note = format!("plausible range {a} to {b}");
                }
            }
        }
        if let Err(e) = super::check_value(&val) {
            leave(e);
            continue;
        }
        let idx = match modules.iter().position(|m| m.module.tx == tx) {
            Some(i) => i,
            None => {
                modules.push(ModuleDef {
                    format: FORMAT,
                    module: ModuleHead {
                        name: format!("module ({tx:03X})"),
                        tx,
                        rx: None,
                        ext: None,
                        tester: None,
                        ids: Default::default(),
                        bus: None,
                        dialect: None,
                        kind: obd_kind(tx),
                        identity: vec![],
                        protocol: None,
                        header_byte: None,
                    },
                    requests: vec![],
                });
                modules.len() - 1
            }
        };
        let m = &mut modules[idx];
        if m.requests
            .iter()
            .flat_map(|r| &r.values)
            .any(|q| q.id == val.id)
        {
            out.left_out
                .push((what, "the same value twice in the module".into()));
            continue;
        }
        let mut trial = m.clone();
        match trial
            .requests
            .iter_mut()
            .find(|r| (r.service, r.ident) == (service, ident))
        {
            Some(r) => {
                r.values.push(val);
                r.min_len = None; // filled in again from all its values
            }
            None => trial.requests.push(RequestDef {
                service,
                ident,
                every: None,
                min_len: None,
                timeout_ms: None,
                years: vec![],
                unverified: vec![],
                needs_session: None,
                sample: None,
                samples: vec![],
                values: vec![val],
            }),
        }
        match validate(&mut trial) {
            Ok(()) => *m = trial,
            Err(e) => out.left_out.push((what, e)),
        }
    }
    for m in modules.iter().filter(|m| m.requests.is_empty()) {
        out.notes.push(format!(
            "{}: none of its values could be imported",
            m.module.label()
        ));
    }
    modules.retain(|m| !m.requests.is_empty());
    let set = SetInfo {
        format: FORMAT,
        name,
        make,
        models: opts.model.iter().cloned().collect(),
        years: vec![],
        source: Source {
            from: "ABRP".into(),
            file: parts[parts.len().saturating_sub(2)..].join("/"),
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

    /// A list written for these tests in ABRP's layout, with made-up identifiers (never ABRP's).
    const LIST: &str = r#"{
  "init_commands": {"command": ["ATZ", "ATSP6"]},
  "obd_protocol": "6",
  "data_commands": {"command": ["ATSH7E4", "22AA01", "ATSH7E2", "22AA02"]},
  "voltage": {"equation": "((A*256)+B)/100", "minValue": "300", "maxValue": "500", "type": "Number", "command": "22AA01", "ecu": "7E4"},
  "current": {"equation": "((Signed(C)*256)+D)/10", "type": "Number", "command": "22AA01", "ecu": "7E4"},
  "soc": {"equation": "A/2", "type": "Number", "command": "22AA02", "ecu": ""},
  "is_charging": {"equation": "{B:2}", "type": "Boolean", "command": "22AA02"},
  "is_dcfc": {"equation": "(A==3)||(A==4)", "type": "Boolean", "command": "22AA02", "ecu": "7E2"},
  "batt_temp": {"equation": "E-40", "type": "Number", "command": "22AA03", "ecu": "7EC"},
  "_Odd_Sum": {"equation": "A+B", "type": "Number", "command": "22AA03", "ecu": "7E4"},
  "kwh": {"equation": "A<<32+B<<16", "type": "Number", "command": "22AA03", "ecu": "7E4"},
  "frame": {"equation": "A", "type": "Number", "command": "03221E3D55555555", "ecu": ""},
  "far": {"equation": "A", "type": "Number", "command": "22AA04", "ecu": "DADAF1"}
}"#;

    fn import() -> Imported {
        parse(
            LIST,
            &Options {
                path: "/home/me/Downloads/ev-obd-pids/hkmc/TestCar.json".into(),
                model: Some("Test".into()),
                today: "2026-11-21".into(),
                ..Default::default()
            },
        )
        .unwrap()
    }

    fn value<'a>(i: &'a Imported, id: &str) -> (&'a RequestDef, &'a ValueDef) {
        i.modules
            .iter()
            .flat_map(|m| &m.requests)
            .flat_map(|r| r.values.iter().map(move |v| (r, v)))
            .find(|(_, v)| v.id == id)
            .unwrap_or_else(|| panic!("{id} not imported"))
    }

    #[test]
    fn the_set_and_its_modules() {
        let i = import();
        let set = i.set.as_ref().unwrap();
        assert_eq!(
            (set.name.as_str(), set.make.as_str()),
            ("abrp-testcar", "hyundai-kia")
        );
        assert_eq!(set.models, ["Test"]);
        assert_eq!(set.source.file, "hkmc/TestCar.json");
        let tx: Vec<u32> = i.modules.iter().map(|m| m.module.tx).collect();
        assert_eq!(
            tx,
            [0x7E4, 0x7E2],
            "7EC is the answer ID of 7E4; an empty ecu takes the ATSH before it"
        );
        assert!(i.notes.iter().any(|n| n.contains("answer ID")));
        assert!(
            i.modules.iter().all(|m| m.module.kind == "powertrain"),
            "7E0-7E7 are OBD-II's powertrain addresses"
        );
        let (r, _) = value(&i, "CURRENT");
        assert_eq!(
            r.min_len,
            Some(4),
            "the second value of a request widens what it needs"
        );
    }

    #[test]
    fn numbers_are_found_by_evaluating_them() {
        let i = import();
        let (r, v) = value(&i, "VOLTAGE");
        assert_eq!((r.service, r.ident), (0x22, 0xAA01));
        assert_eq!(
            (v.byte, v.bits, v.signed, v.unit.as_str()),
            (0, 16, false, "V")
        );
        assert_eq!(v.name, "Battery voltage");
        let shown = |v: &ValueDef, d: &[u8]| decode::decode(v, d).unwrap().shown;
        assert_eq!(shown(v, &[0x9C, 0x40]), Shown::Number(400.0));
        let (_, c) = value(&i, "CURRENT");
        assert_eq!((c.byte, c.bits, c.signed), (2, 16, true));
        assert_eq!(shown(c, &[0, 0, 0xFF, 0x9C]), Shown::Number(-10.0));
        let (_, s) = value(&i, "SOC");
        assert_eq!(shown(s, &[150]), Shown::Number(75.0));
        let (_, t) = value(&i, "BATT_TEMP");
        assert_eq!(
            (t.byte, shown(t, &[0, 0, 0, 0, 65])),
            (4, Shown::Number(25.0))
        );
        let (_, b) = value(&i, "IS_CHARGING");
        assert_eq!(
            (b.byte, b.bit, b.bits),
            (1, 5, 1),
            "bit 2 from the right is bit 5 from the left"
        );
        assert_eq!(shown(b, &[0, 0b0000_0100]), Shown::State("yes".into()));
    }

    #[test]
    fn what_is_not_one_value_is_left_out_with_the_reason() {
        let i = import();
        let why = |k: &str| {
            i.left_out
                .iter()
                .find(|(w, _)| w.starts_with(k))
                .map(|(_, y)| y.clone())
                .unwrap_or_else(|| panic!("{k} not left out"))
        };
        assert!(
            !i.left_out.iter().any(|(w, _)| w.starts_with("is_dcfc")),
            "a yes/no test over one byte is imported"
        );
        assert!(
            why("_Odd_Sum").contains("not linear"),
            "A+B is not one number"
        );
        assert!(
            why("kwh").contains("not linear"),
            "C precedence: A << (32+B) << ..."
        );
        assert!(why("frame").contains("raw CAN frame"));
        assert!(why("far").contains("29-bit"));
    }

    #[test]
    fn equations() {
        assert_eq!(var_index("A"), Some(0));
        assert_eq!(var_index("z"), Some(25));
        assert_eq!(var_index("AA"), Some(26));
        assert_eq!(var_index("an"), Some(39));
        let v = as_value("Int16(A,B)*0.0032").unwrap();
        assert_eq!((v.bits, v.signed), (16, false));
        assert!((v.mul - 0.0032).abs() < 1e-12);
        let v = as_value("((N<<8)+O)/10").unwrap();
        assert_eq!((v.byte, v.bits), (13, 16));
        let v = as_value("1.12*A/2.5-7.16").unwrap();
        assert!((v.add + 7.16).abs() < 1e-9 && (v.mul - 0.448).abs() < 1e-9);
        assert!(as_value("42").is_err(), "reads no byte");
        assert!(as_value("A/(B-B)").is_err());
        assert!(as_value("A +").is_err());
        // a yes/no test over one byte: yes for exactly the raw values the equation is true for
        let v = as_value("(A==1)||(A==2)").unwrap();
        assert_eq!(v.kind, crate::moddata::ValueKind::State);
        assert_eq!(
            (
                v.states["1"].as_str(),
                v.states["2"].as_str(),
                v.states["0"].as_str()
            ),
            ("yes", "yes", "no")
        );
        assert_eq!((v.states.len(), v.states["3"].as_str()), (256, "no"));
        let shown = |d: &[u8]| crate::moddata::decode::decode(&v, d).unwrap().shown;
        assert_eq!(shown(&[2]), crate::moddata::Shown::State("yes".into()));
        assert_eq!(shown(&[7]), crate::moddata::Shown::State("no".into()));
        assert_eq!(as_value("!{A:1}&&{A:2}").unwrap().states["4"], "yes");
        assert!(as_value("(A==1)&&(B==2)")
            .unwrap_err()
            .contains("several bytes"));
        assert_eq!(as_value("INT16(A:B)*0.01").unwrap().bits, 16);
    }
}
