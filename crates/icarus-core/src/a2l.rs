//! ASAM MCD-2 MC definitions (`.a2l`, also called ASAP2): the format calibration tools use to describe an ECU's maps.
//! Written from the format's public descriptions, not from any program's source. Read into the same model as TunerPro's
//! files (def.rs), so the Maps page, `icarus show` and `icarus diff` work the same with either.
//!
//! What is read (enough for maps and values):
//! - `CHARACTERISTIC` of type `VALUE`, `CURVE` (its axis down the side, as TunerPro shows curves), `MAP` (the first
//!   `AXIS_DESCR` across the top, the second down the side) and `VAL_BLK` (`NUMBER n`, or `MATRIX_DIM x y`: a table with
//!   numbered rows and columns), at its address, laid out by its `RECORD_LAYOUT`;
//! - `RECORD_LAYOUT`: `FNC_VALUES` (data type; `ROW_DIR` stores the map row by row, `COLUMN_DIR` column by column; direct
//!   addressing), `AXIS_PTS_X` / `AXIS_PTS_Y` (rising, direct), `RESERVED`, `FIX_NO_AXIS_PTS_*` and the `ALIGNMENT_*` of each
//!   data type. The parts follow one another in the order of their positions, each starting on its data type's alignment
//!   (1, 2, 4 and 8 bytes unless `MOD_COMMON` or the layout says otherwise);
//! - `COMPU_METHOD`: `IDENTICAL`; `LINEAR` (`COEFFS_LINEAR a b`: real = a*stored + b); `RAT_FUNC` (`COEFFS a b c d e f`, which
//!   give the stored number from the real value v: stored = (a*v^2 + b*v + c) / (d*v^2 + e*v + f)), read when a and d are
//!   0, so real = (f*stored - c) / (b - e*stored), the form with one real value for each stored number; `FORM` when its
//!   `FORMULA` is plain arithmetic in `X1` (numbers, + - * /, parentheses; Icarus inverts it itself); `NO_COMPU_METHOD`;
//! - `AXIS_DESCR`: `FIX_AXIS` (`FIX_AXIS_PAR`, `FIX_AXIS_PAR_DIST`, `FIX_AXIS_PAR_LIST`; the points converted by the axis's
//!   method), `COM_AXIS` (an `AXIS_PTS` named by `AXIS_PTS_REF`) and `STD_AXIS` (points stored with the values);
//! - `AXIS_PTS`, also listed as elements of their own (category "Axis points"), so shared breakpoints can be seen and edited;
//! - `BYTE_ORDER` (`MSB_FIRST` / `BIG_ENDIAN`: most significant byte first; `MSB_LAST` / `LITTLE_ENDIAN`: least) from
//!   `MOD_COMMON` or the element, `FORMAT` (decimals), `PHYS_UNIT`, the limits, and `FUNCTION`s (`DEF_CHARACTERISTIC`, a
//!   characteristic's `FUNCTION_LIST`) as categories;
//! - addresses: the image is taken to begin at the lowest `MEMORY_SEGMENT` of `MOD_PAR` that is not RAM, a register or
//!   outside the ECU (a dump of the ECU's memory from there); with no segments, addresses are file offsets.
//!
//! Anything else an element needs (other characteristic types, conversion tables and formulas, bit masks, axis point
//! counts stored in the image, rescale axes, keywords not listed here) leaves that element out with the reason: nothing is
//! guessed. Measurements (live values) are not maps and are not read.

use crate::def::{fmt_num, Axis, Data, DefError, Definition, Element, Kind, Parsed, Table};
use crate::expr::{Expr, Op};
use crate::rom::Cell;
use std::collections::HashMap;

/// Whether `text` looks like an A2L file (ASAP2's blocks).
pub fn looks_like(text: &str) -> bool {
    text.contains("/begin") && text.contains("PROJECT")
}

pub fn parse(text: &str) -> Result<Parsed, DefError> {
    let toks = tokens(text)?;
    let mut skipped = Vec::new();
    let items = parse_items(&toks, &mut 0, None, &mut skipped)?;
    let project = items
        .iter()
        .find_map(|it| match it {
            Item::Block(b) if b.kind == "PROJECT" => Some(b),
            _ => None,
        })
        .ok_or_else(|| DefError::Bad("not an ASAM definition (no /begin PROJECT)".into()))?;
    let modules: Vec<&Block> = project.blocks("MODULE").collect();
    let module = *modules
        .first()
        .ok_or_else(|| DefError::Bad("the PROJECT has no MODULE".into()))?;
    let name_of = |b: &Block| {
        b.toks()
            .next()
            .map(|t| t.text().to_string())
            .unwrap_or_default()
    };
    if modules.len() > 1 {
        skipped.push(format!(
            "the file describes {} modules; only the first ({}) is read",
            modules.len(),
            name_of(module)
        ));
    }
    let long_of = |b: &Block| {
        b.toks()
            .nth(1)
            .map(|t| t.text().to_string())
            .unwrap_or_default()
    };
    let mut def = Definition {
        title: Some(long_of(project))
            .filter(|t| !t.is_empty())
            .unwrap_or_else(|| name_of(project)),
        description: long_of(module),
        ..Default::default()
    };

    let mut ctx = Ctx {
        order: None,
        align: Align::default(),
        difference: false,
        layouts: HashMap::new(),
        compus: HashMap::new(),
        axis_pts: HashMap::new(),
    };
    if let Some(mc) = module.block("MOD_COMMON") {
        let t: Vec<&Tok> = mc.toks().collect();
        for (i, k) in t.iter().enumerate() {
            let next = t.get(i + 1).map(|v| v.text()).unwrap_or("");
            match k.text() {
                "BYTE_ORDER" => ctx.order = Some(next.to_string()),
                "DEPOSIT" => ctx.difference = next == "DIFFERENCE",
                a if a.starts_with("ALIGNMENT_") => {
                    if let Some(v) = num(next) {
                        ctx.align.set(a, v as u64);
                    }
                }
                _ => {}
            }
        }
    }
    if let Some(mp) = module.block("MOD_PAR") {
        let segs: Vec<(u64, u64)> = mp
            .blocks("MEMORY_SEGMENT")
            .filter_map(|s| {
                let a = s.args(7).ok()?;
                if matches!(a[3].text(), "RAM" | "REGISTER" | "NOT_IN_ECU") {
                    return None;
                }
                Some((addr(a[5]).ok()?, addr(a[6]).ok()?))
            })
            .collect();
        if let Some(lo) = segs.iter().map(|s| s.0).min() {
            def.base_offset = lo;
            def.base_subtract = true;
            if let [one] = segs.as_slice() {
                def.image_size = usize::try_from(one.1).ok();
            }
        }
    }
    for (kind, map) in [
        ("RECORD_LAYOUT", &mut ctx.layouts),
        ("COMPU_METHOD", &mut ctx.compus),
        ("AXIS_PTS", &mut ctx.axis_pts),
    ] {
        for b in module.blocks(kind) {
            map.insert(name_of(b), b);
        }
    }
    let mut functions: HashMap<String, Vec<String>> = HashMap::new();
    for f in module.blocks("FUNCTION") {
        for dc in f.blocks("DEF_CHARACTERISTIC") {
            for t in dc.toks() {
                functions
                    .entry(t.text().to_string())
                    .or_default()
                    .push(name_of(f));
            }
        }
    }

    for b in module.items.iter().filter_map(|it| match it {
        Item::Block(b) => Some(b),
        _ => None,
    }) {
        let kind = match b.kind.as_str() {
            "CHARACTERISTIC" => ctx.characteristic(b),
            "AXIS_PTS" => ctx.axis_pts_data(b).map(|d| {
                Kind::Table(Table {
                    x: Axis::Index(1),
                    y: Axis::Index(d.cols),
                    z: Data {
                        rows: d.cols,
                        cols: 1,
                        ..d
                    },
                })
            }),
            _ => continue,
        };
        let name = name_of(b);
        match kind {
            Ok(kind) => {
                let mut categories = functions.remove(&name).unwrap_or_default();
                if b.kind == "AXIS_PTS" {
                    categories.push("Axis points".into());
                }
                for fl in b.blocks("FUNCTION_LIST") {
                    for t in fl.toks() {
                        if !categories.iter().any(|c| c == t.text()) {
                            categories.push(t.text().to_string());
                        }
                    }
                }
                def.elements.push(Element {
                    id: None,
                    title: name,
                    description: long_of(b),
                    categories,
                    kind,
                });
            }
            Err(why) => skipped.push(format!("{name}: {why}")),
        }
    }
    Ok(Parsed {
        definition: def,
        skipped,
    })
}

// ---- reading the text: words, strings, blocks

#[derive(Clone, Debug, PartialEq)]
enum Tok {
    Word(String),
    Str(String),
}

impl Tok {
    fn text(&self) -> &str {
        match self {
            Tok::Word(s) | Tok::Str(s) => s,
        }
    }

    fn word(&self) -> Option<&str> {
        match self {
            Tok::Word(s) => Some(s),
            Tok::Str(_) => None,
        }
    }
}

fn bad(line: usize, msg: &str) -> DefError {
    DefError::Bad(format!("line {line}: {msg}"))
}

/// Words and strings with their line numbers; comments (`/* */`, `//`) dropped.
fn tokens(text: &str) -> Result<Vec<(Tok, usize)>, DefError> {
    let c: Vec<char> = text.chars().collect();
    let mut out = Vec::new();
    let (mut i, mut line) = (0, 1);
    while i < c.len() {
        let ch = c[i];
        if ch == '\n' {
            line += 1;
            i += 1;
        } else if ch.is_whitespace() {
            i += 1;
        } else if ch == '/' && c.get(i + 1) == Some(&'*') {
            let start = line;
            i += 2;
            loop {
                match c.get(i) {
                    None => return Err(bad(start, "a comment (/*) is never closed")),
                    Some('*') if c.get(i + 1) == Some(&'/') => {
                        i += 2;
                        break;
                    }
                    Some(x) => {
                        line += usize::from(*x == '\n');
                        i += 1;
                    }
                }
            }
        } else if ch == '/' && c.get(i + 1) == Some(&'/') {
            while i < c.len() && c[i] != '\n' {
                i += 1;
            }
        } else if ch == '"' {
            let start = line;
            let mut s = String::new();
            i += 1;
            loop {
                match c.get(i) {
                    None => return Err(bad(start, "a string is never closed")),
                    Some('\\') => {
                        if let Some(n) = c.get(i + 1) {
                            s.push(match n {
                                'n' => '\n',
                                't' => '\t',
                                o => *o,
                            });
                            line += usize::from(*n == '\n');
                        }
                        i += 2;
                    }
                    // a quote written twice inside a string is one quote
                    Some('"') if c.get(i + 1) == Some(&'"') => {
                        s.push('"');
                        i += 2;
                    }
                    Some('"') => {
                        i += 1;
                        break;
                    }
                    Some(x) => {
                        line += usize::from(*x == '\n');
                        s.push(*x);
                        i += 1;
                    }
                }
            }
            out.push((Tok::Str(s), start));
        } else {
            let start = i;
            while i < c.len() && !c[i].is_whitespace() && c[i] != '"' {
                i += 1;
            }
            out.push((Tok::Word(c[start..i].iter().collect()), line));
        }
    }
    Ok(out)
}

#[derive(Debug)]
struct Block {
    kind: String,
    items: Vec<Item>,
}

#[derive(Debug)]
enum Item {
    Tok(Tok),
    Block(Block),
}

/// The items up to the `/end` of `inside` (kind and line of its `/begin`), or to the end of the text.
fn parse_items(
    toks: &[(Tok, usize)],
    i: &mut usize,
    inside: Option<(&str, usize)>,
    notes: &mut Vec<String>,
) -> Result<Vec<Item>, DefError> {
    let mut items = Vec::new();
    while *i < toks.len() {
        let (t, line) = (&toks[*i].0, toks[*i].1);
        let next = toks.get(*i + 1).and_then(|n| n.0.word());
        match t.word() {
            Some("/begin") => {
                let kind = next
                    .ok_or_else(|| bad(line, "/begin without a block name"))?
                    .to_string();
                *i += 2;
                if kind == "A2ML" {
                    // the grammar of IF_DATA, not data: skipped whole
                    while *i < toks.len()
                        && !(toks[*i].0.word() == Some("/end")
                            && toks.get(*i + 1).and_then(|n| n.0.word()) == Some("A2ML"))
                    {
                        *i += 1;
                    }
                    if *i >= toks.len() {
                        return Err(bad(line, "/begin A2ML is never closed"));
                    }
                    *i += 2;
                    continue;
                }
                let inner = parse_items(toks, i, Some((kind.as_str(), line)), notes)?;
                items.push(Item::Block(Block { kind, items: inner }));
            }
            Some("/end") => {
                let kind = next.unwrap_or("");
                return match inside {
                    Some((k, _)) if k == kind => {
                        *i += 2;
                        Ok(items)
                    }
                    Some((k, l)) => Err(bad(
                        line,
                        &format!("/end {kind} where the /begin {k} of line {l} should end"),
                    )),
                    None => Err(bad(line, &format!("/end {kind} without its /begin"))),
                };
            }
            Some("/include") => {
                let what = toks.get(*i + 1).map(|n| n.0.text()).unwrap_or("");
                notes.push(format!(
                    "/include {what}: the definition goes on in another file, which is not read"
                ));
                *i += 2;
            }
            _ => {
                items.push(Item::Tok(t.clone()));
                *i += 1;
            }
        }
    }
    match inside {
        Some((k, l)) => Err(bad(l, &format!("/begin {k} is never closed"))),
        None => Ok(items),
    }
}

/// For `options`: a keyword followed by every number after it (MATRIX_DIM has one to three).
const NUMBERS: usize = usize::MAX;

const CHARACTERISTIC_KEYS: &[(&str, usize)] = &[
    ("BIT_MASK", 1),
    ("BYTE_ORDER", 1),
    ("CALIBRATION_ACCESS", 1),
    ("COMPARISON_QUANTITY", 1),
    ("DISCRETE", 0),
    ("DISPLAY_IDENTIFIER", 1),
    ("ECU_ADDRESS_EXTENSION", 1),
    ("ENCODING", 1),
    ("EXTENDED_LIMITS", 2),
    ("FORMAT", 1),
    ("GUARD_RAILS", 0),
    ("MATRIX_DIM", NUMBERS),
    ("MAX_REFRESH", 2),
    ("MODEL_LINK", 1),
    ("NUMBER", 1),
    ("PHYS_UNIT", 1),
    ("READ_ONLY", 0),
    ("REF_MEMORY_SEGMENT", 1),
    ("STEP_SIZE", 1),
    ("SYMBOL_LINK", 2),
];

const AXIS_PTS_KEYS: &[(&str, usize)] = &[
    ("BYTE_ORDER", 1),
    ("CALIBRATION_ACCESS", 1),
    ("DEPOSIT", 1),
    ("DISPLAY_IDENTIFIER", 1),
    ("ECU_ADDRESS_EXTENSION", 1),
    ("EXTENDED_LIMITS", 2),
    ("FORMAT", 1),
    ("GUARD_RAILS", 0),
    ("MAX_REFRESH", 2),
    ("MODEL_LINK", 1),
    ("MONOTONY", 1),
    ("PHYS_UNIT", 1),
    ("READ_ONLY", 0),
    ("REF_MEMORY_SEGMENT", 1),
    ("STEP_SIZE", 1),
    ("SYMBOL_LINK", 2),
];

const AXIS_DESCR_KEYS: &[(&str, usize)] = &[
    ("AXIS_PTS_REF", 1),
    ("BYTE_ORDER", 1),
    ("CURVE_AXIS_REF", 1),
    ("DEPOSIT", 1),
    ("EXTENDED_LIMITS", 2),
    ("FIX_AXIS_PAR", 3),
    ("FIX_AXIS_PAR_DIST", 3),
    ("FORMAT", 1),
    ("MAX_GRAD", 1),
    ("MONOTONY", 1),
    ("PHYS_UNIT", 1),
    ("READ_ONLY", 0),
    ("STEP_SIZE", 1),
];

const COMPU_KEYS: &[(&str, usize)] = &[
    ("COEFFS", 6),
    ("COEFFS_LINEAR", 2),
    ("COMPU_TAB_REF", 1),
    ("REF_UNIT", 1),
    ("STATUS_STRING_REF", 1),
];

type Options = HashMap<String, Vec<Tok>>;

impl Block {
    fn blocks<'a>(&'a self, kind: &str) -> impl Iterator<Item = &'a Block> + 'a {
        let kind = kind.to_string();
        self.items.iter().filter_map(move |it| match it {
            Item::Block(b) if b.kind == kind => Some(b),
            _ => None,
        })
    }

    fn block(&self, kind: &str) -> Option<&Block> {
        self.blocks(kind).next()
    }

    fn toks(&self) -> impl Iterator<Item = &Tok> {
        self.items.iter().filter_map(|it| match it {
            Item::Tok(t) => Some(t),
            Item::Block(_) => None,
        })
    }

    /// The block's first `n` values (its fixed parameters).
    fn args(&self, n: usize) -> Result<Vec<&Tok>, String> {
        let v: Vec<&Tok> = self.toks().take(n).collect();
        if v.len() < n {
            return Err(format!(
                "{} has {} of its {n} parameters",
                self.kind,
                v.len()
            ));
        }
        Ok(v)
    }

    /// The optional keywords after the `fixed` parameters, each with its values. A keyword not in `known` is an error: where
    /// its values end cannot be told.
    fn options(&self, fixed: usize, known: &[(&str, usize)]) -> Result<Options, String> {
        let toks: Vec<&Tok> = self.toks().skip(fixed).collect();
        let mut out = Options::new();
        let mut i = 0;
        while i < toks.len() {
            let k = toks[i]
                .word()
                .ok_or_else(|| format!("\"{}\" where a keyword should be", toks[i].text()))?;
            let n = known
                .iter()
                .find(|x| x.0 == k)
                .map(|x| x.1)
                .ok_or_else(|| format!("{k} is a keyword Icarus does not read"))?;
            i += 1;
            let take = if n == NUMBERS {
                toks[i..]
                    .iter()
                    .take_while(|t| t.word().and_then(num).is_some())
                    .count()
            } else {
                n
            };
            if i + take > toks.len() {
                return Err(format!("{k} needs {take} value(s)"));
            }
            out.insert(
                k.to_string(),
                toks[i..i + take].iter().map(|t| (*t).clone()).collect(),
            );
            i += take;
        }
        Ok(out)
    }
}

/// "0x8010", "12", "-1.5e3" as a number.
fn num(s: &str) -> Option<f64> {
    let s = s.trim();
    if let Some(h) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        return u64::from_str_radix(h, 16).ok().map(|v| v as f64);
    }
    s.parse::<f64>().ok().filter(|v| v.is_finite())
}

fn addr(t: &Tok) -> Result<u64, String> {
    let s = t.text().trim();
    let v = match s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        Some(h) => u64::from_str_radix(h, 16).ok(),
        None => s.parse::<u64>().ok(),
    };
    v.ok_or_else(|| format!("\"{s}\" is not an address"))
}

fn count(t: &Tok) -> Result<usize, String> {
    num(t.text())
        .filter(|v| *v >= 1.0 && v.fract() == 0.0 && *v <= 65536.0)
        .map(|v| v as usize)
        .ok_or_else(|| format!("\"{}\" is not a number of points", t.text()))
}

fn opt_text<'a>(o: &'a Options, k: &str) -> Option<&'a str> {
    o.get(k).and_then(|v| v.first()).map(|t| t.text())
}

fn opt_nums(o: &Options, k: &str) -> Option<Result<Vec<f64>, String>> {
    o.get(k).map(|v| {
        v.iter()
            .map(|t| num(t.text()).ok_or_else(|| format!("{k}: \"{}\" is not a number", t.text())))
            .collect()
    })
}

/// The decimals of a C-style format: "%6.2" has 2.
fn decimals(format: &str) -> u8 {
    format
        .split_once('.')
        .and_then(|(_, d)| {
            d.chars()
                .take_while(char::is_ascii_digit)
                .collect::<String>()
                .parse::<u8>()
                .ok()
        })
        .unwrap_or(0)
        .min(9)
}

fn byte_order(s: &str) -> Result<bool, String> {
    match s {
        "MSB_FIRST" | "BIG_ENDIAN" => Ok(true),
        "MSB_LAST" | "LITTLE_ENDIAN" => Ok(false),
        o => Err(format!("byte order {o} is not read")),
    }
}

/// A FORM conversion's formula in Icarus's own terms: the input X1 (or X) as X, numbers, + - * / and parentheses. Functions,
/// bit and logic operators, and a second input are refused with the reason, never guessed.
fn form_expr(f: &str) -> Result<Expr, String> {
    if f.chars()
        .any(|c| matches!(c, '^' | '&' | '|' | '<' | '>' | '!' | '~' | '%' | '?' | ':'))
    {
        return Err("uses bit or logic operators, which Icarus does not work out".into());
    }
    let cs: Vec<char> = f.chars().collect();
    let mut out = String::new();
    let mut i = 0;
    while i < cs.len() {
        let c = cs[i];
        if c == '0' && matches!(cs.get(i + 1), Some('x' | 'X')) {
            out.push_str("0x");
            i += 2;
            while i < cs.len() && cs[i].is_ascii_hexdigit() {
                out.push(cs[i]);
                i += 1;
            }
            continue;
        }
        if c.is_ascii_digit() || c == '.' {
            while i < cs.len() && (cs[i].is_ascii_digit() || cs[i] == '.') {
                out.push(cs[i]);
                i += 1;
            }
            if matches!(cs.get(i), Some('e' | 'E')) {
                out.push(cs[i]);
                i += 1;
                if matches!(cs.get(i), Some('+' | '-')) {
                    out.push(cs[i]);
                    i += 1;
                }
            }
            continue;
        }
        if c == 'X' {
            let mut j = i + 1;
            let mut digits = String::new();
            while j < cs.len() && cs[j].is_ascii_digit() {
                digits.push(cs[j]);
                j += 1;
            }
            if !(digits.is_empty() || digits == "1") {
                return Err(format!(
                    "uses a second input (X{digits}), which Icarus does not work out"
                ));
            }
            out.push('X');
            i = j;
            continue;
        }
        if c.is_ascii_alphabetic() || c == '_' {
            return Err(
                "uses functions (sin, exp, sqrt, ...), which Icarus does not work out".into(),
            );
        }
        out.push(c);
        i += 1;
    }
    let e = Expr::parse(&out).map_err(|e| format!("is not a formula Icarus reads ({e})"))?;
    if !e.vars().is_empty() || !out.contains('X') {
        return Err("does not depend on the stored number".into());
    }
    Ok(e)
}

fn type_bytes(datatype: &str) -> Result<u64, String> {
    match datatype {
        "UBYTE" | "SBYTE" => Ok(1),
        "UWORD" | "SWORD" => Ok(2),
        "ULONG" | "SLONG" | "FLOAT32_IEEE" => Ok(4),
        "A_UINT64" | "A_INT64" | "FLOAT64_IEEE" => Ok(8),
        o => Err(format!("data type {o} is not read")),
    }
}

/// How one number of `datatype` is stored; `big_endian` matters only for more than one byte.
fn cell(datatype: &str, big_endian: Option<bool>) -> Result<Cell, String> {
    let (bits, signed, float) = match datatype {
        "UBYTE" => (8, false, false),
        "SBYTE" => (8, true, false),
        "UWORD" => (16, false, false),
        "SWORD" => (16, true, false),
        "ULONG" => (32, false, false),
        "SLONG" => (32, true, false),
        "FLOAT32_IEEE" => (32, false, true),
        "FLOAT64_IEEE" => (64, false, true),
        "A_UINT64" | "A_INT64" => {
            return Err(format!(
                "data type {datatype}: 64-bit integers are not read (past 2^53 they cannot be shown exactly)"
            ))
        }
        o => return Err(format!("data type {o} is not read")),
    };
    let little_endian = match (bits, big_endian) {
        (8, _) => false,
        (_, Some(big)) => !big,
        (_, None) => return Err("no BYTE_ORDER says in which order its bytes are stored".into()),
    };
    Ok(Cell {
        bits,
        signed,
        little_endian,
        float,
    })
}

/// Where each data type starts: on a multiple of its alignment.
#[derive(Clone, Copy)]
struct Align {
    byte: u64,
    word: u64,
    long: u64,
    int64: u64,
    float32: u64,
    float64: u64,
}

impl Default for Align {
    fn default() -> Align {
        Align {
            byte: 1,
            word: 2,
            long: 4,
            int64: 8,
            float32: 4,
            float64: 8,
        }
    }
}

impl Align {
    fn set(&mut self, key: &str, v: u64) {
        let v = v.max(1);
        match key {
            "ALIGNMENT_BYTE" => self.byte = v,
            "ALIGNMENT_WORD" => self.word = v,
            "ALIGNMENT_LONG" => self.long = v,
            "ALIGNMENT_INT64" => self.int64 = v,
            "ALIGNMENT_FLOAT32_IEEE" => self.float32 = v,
            "ALIGNMENT_FLOAT64_IEEE" => self.float64 = v,
            _ => {}
        }
    }

    fn of(&self, datatype: &str) -> u64 {
        match datatype {
            "UWORD" | "SWORD" => self.word,
            "ULONG" | "SLONG" => self.long,
            "A_UINT64" | "A_INT64" => self.int64,
            "FLOAT32_IEEE" => self.float32,
            "FLOAT64_IEEE" => self.float64,
            _ => self.byte,
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Part {
    Values {
        column_dir: bool,
    },
    /// Axis points stored with the values: 0 = X, 1 = Y.
    Axis(usize),
    Reserved(u64),
}

struct Comp {
    pos: i64,
    part: Part,
    datatype: String,
}

struct Layout {
    comps: Vec<Comp>,
    align: Align,
}

impl Layout {
    fn datatype(&self, part: impl Fn(&Part) -> bool) -> Option<&str> {
        self.comps
            .iter()
            .find(|c| part(&c.part))
            .map(|c| c.datatype.as_str())
    }

    /// Where the values and each axis's points start for a record at `address`: the parts in the order of their positions,
    /// each on its alignment.
    fn place(&self, address: u64, values: u64, axes: [u64; 2]) -> Result<(u64, [u64; 2]), String> {
        let mut order: Vec<&Comp> = self.comps.iter().collect();
        order.sort_by_key(|c| c.pos);
        let (mut at, mut v, mut a) = (address, address, [address; 2]);
        for c in order {
            if let Part::Reserved(n) = c.part {
                at += n;
                continue;
            }
            let size = type_bytes(&c.datatype)?;
            let al = self.align.of(&c.datatype).max(1);
            at = at.div_ceil(al) * al;
            match c.part {
                Part::Values { .. } => {
                    v = at;
                    at += size * values;
                }
                Part::Axis(k) => {
                    a[k] = at;
                    at += size * axes[k];
                }
                Part::Reserved(_) => {}
            }
        }
        Ok((v, a))
    }
}

/// A conversion: its formula from stored number to real value, decimals and unit.
struct Compu {
    math: Expr,
    decimals: u8,
    unit: String,
}

enum AxisSpec {
    Fixed(Vec<String>),
    Common(Data),
    Std {
        compu: Compu,
        n: usize,
        big: Option<bool>,
        decimals: u8,
        units: String,
        min: Option<f64>,
        max: Option<f64>,
    },
}

impl AxisSpec {
    fn count(&self) -> usize {
        match self {
            AxisSpec::Fixed(l) => l.len(),
            AxisSpec::Common(d) => d.cols,
            AxisSpec::Std { n, .. } => *n,
        }
    }
}

struct Ctx<'a> {
    /// MOD_COMMON's BYTE_ORDER, as written.
    order: Option<String>,
    align: Align,
    /// MOD_COMMON's DEPOSIT DIFFERENCE: axis points stored as differences unless an axis says otherwise.
    difference: bool,
    layouts: HashMap<String, &'a Block>,
    compus: HashMap<String, &'a Block>,
    axis_pts: HashMap<String, &'a Block>,
}

fn bin(op: Op, a: Expr, b: Expr) -> Expr {
    Expr::Bin(op, Box::new(a), Box::new(b))
}

impl Ctx<'_> {
    fn big_endian(&self, opts: &Options) -> Result<Option<bool>, String> {
        opt_text(opts, "BYTE_ORDER")
            .or(self.order.as_deref())
            .map(byte_order)
            .transpose()
    }

    fn layout(&self, name: &str) -> Result<Layout, String> {
        let b = self
            .layouts
            .get(name)
            .ok_or_else(|| format!("its record layout {name} is not in the file"))?;
        let toks: Vec<&Tok> = b.toks().skip(1).collect();
        let w = |j: usize| toks.get(j).map(|t| t.text()).unwrap_or("");
        let pos = |j: usize| {
            num(w(j))
                .map(|p| p as i64)
                .ok_or_else(|| format!("record layout {name}: \"{}\" is not a position", w(j)))
        };
        let mut comps = Vec::new();
        let mut align = self.align;
        let mut i = 0;
        while i < toks.len() {
            let k = w(i);
            match k {
                "FNC_VALUES" => {
                    let column_dir = match w(i + 3) {
                        "ROW_DIR" => false,
                        "COLUMN_DIR" => true,
                        o => return Err(format!("record layout {name}: values stored {o} are not read")),
                    };
                    if w(i + 4) != "DIRECT" {
                        return Err(format!(
                            "record layout {name}: values reached through a pointer ({}) are not read",
                            w(i + 4)
                        ));
                    }
                    comps.push(Comp {
                        pos: pos(i + 1)?,
                        part: Part::Values { column_dir },
                        datatype: w(i + 2).to_string(),
                    });
                    i += 5;
                }
                "AXIS_PTS_X" | "AXIS_PTS_Y" => {
                    if w(i + 3) != "INDEX_INCR" {
                        return Err(format!(
                            "record layout {name}: axis points stored {} are not read (only rising, INDEX_INCR)",
                            w(i + 3)
                        ));
                    }
                    if w(i + 4) != "DIRECT" {
                        return Err(format!(
                            "record layout {name}: axis points reached through a pointer ({}) are not read",
                            w(i + 4)
                        ));
                    }
                    comps.push(Comp {
                        pos: pos(i + 1)?,
                        part: Part::Axis(usize::from(k == "AXIS_PTS_Y")),
                        datatype: w(i + 2).to_string(),
                    });
                    i += 5;
                }
                "RESERVED" => {
                    let n = match w(i + 2) {
                        "BYTE" => 1,
                        "WORD" => 2,
                        "LONG" => 4,
                        o => return Err(format!("record layout {name}: RESERVED {o} is not read")),
                    };
                    comps.push(Comp {
                        pos: pos(i + 1)?,
                        part: Part::Reserved(n),
                        datatype: String::new(),
                    });
                    i += 3;
                }
                "STATIC_RECORD_LAYOUT" | "STATIC_ADDRESS_OFFSETS" => i += 1,
                a if a.starts_with("ALIGNMENT_") => {
                    let v = num(w(i + 1))
                        .ok_or_else(|| format!("record layout {name}: {a} needs a number"))?;
                    align.set(a, v as u64);
                    i += 2;
                }
                f if f.starts_with("FIX_NO_AXIS_PTS_") => i += 2,
                n if n.starts_with("NO_AXIS_PTS_") => {
                    return Err(format!(
                        "record layout {name}: the number of axis points is stored in the image ({n}), which is not read"
                    ))
                }
                o => return Err(format!("record layout {name}: {o} is not read")),
            }
        }
        if comps
            .iter()
            .filter(|c| matches!(c.part, Part::Values { .. }))
            .count()
            > 1
        {
            return Err(format!("record layout {name} has FNC_VALUES twice"));
        }
        Ok(Layout { comps, align })
    }

    fn compu(&self, name: &str) -> Result<Compu, String> {
        if name == "NO_COMPU_METHOD" {
            return Ok(Compu {
                math: Expr::identity(),
                decimals: 2,
                unit: String::new(),
            });
        }
        let b = self
            .compus
            .get(name)
            .ok_or_else(|| format!("its conversion {name} is not in the file"))?;
        let a = b.args(5)?;
        let opts = b
            .options(5, COMPU_KEYS)
            .map_err(|e| format!("conversion {name}: {e}"))?;
        let coeffs = |k: &str, n: usize| -> Result<Vec<f64>, String> {
            let v = opt_nums(&opts, k).ok_or_else(|| format!("conversion {name} has no {k}"))??;
            if v.len() != n {
                return Err(format!("conversion {name}: {k} needs {n} numbers"));
            }
            Ok(v)
        };
        let math = match a[2].text() {
            "IDENTICAL" => Expr::identity(),
            "LINEAR" => {
                let c = coeffs("COEFFS_LINEAR", 2)?;
                if c[0] == 0.0 {
                    return Err(format!(
                        "conversion {name}: LINEAR with a factor of 0 gives the same value for every stored number"
                    ));
                }
                bin(
                    Op::Add,
                    bin(Op::Mul, Expr::Num(c[0]), Expr::X),
                    Expr::Num(c[1]),
                )
            }
            "RAT_FUNC" => {
                let c = coeffs("COEFFS", 6)?;
                let (sa, sb, sc, sd, se, sf) = (c[0], c[1], c[2], c[3], c[4], c[5]);
                if sa != 0.0 || sd != 0.0 {
                    return Err(format!(
                        "conversion {name}: a RAT_FUNC with squared terms (a or d not 0) gives two real values for some stored numbers; not read"
                    ));
                }
                if (sb == 0.0 && se == 0.0) || (se == 0.0 && sf == 0.0) {
                    return Err(format!(
                        "conversion {name}: this RAT_FUNC gives no real value for a stored number"
                    ));
                }
                // stored = (b*v + c) / (e*v + f), so v = (f*stored - c) / (b - e*stored)
                bin(
                    Op::Div,
                    bin(Op::Sub, bin(Op::Mul, Expr::Num(sf), Expr::X), Expr::Num(sc)),
                    bin(Op::Sub, Expr::Num(sb), bin(Op::Mul, Expr::Num(se), Expr::X)),
                )
            }
            // a formula from the stored number X1 to the real value (ICA-56): read when it is one Icarus's own formulas can
            // say (numbers, + - * /, parentheses); Icarus inverts it itself, so FORMULA_INV is not needed
            "FORM" => {
                let f = b
                    .block("FORMULA")
                    .and_then(|f| f.toks().next())
                    .map(|t| t.text().to_string())
                    .ok_or_else(|| format!("conversion {name}: FORM without a FORMULA"))?;
                form_expr(&f)
                    .map_err(|why| format!("conversion {name}: its FORMULA {f:?} {why}"))?
            }
            o => {
                return Err(format!(
                    "its conversion {name} is {o}, which Icarus does not read"
                ))
            }
        };
        Ok(Compu {
            math,
            decimals: decimals(a[3].text()),
            unit: a[4].text().to_string(),
        })
    }

    fn check_common(&self, opts: &Options) -> Result<(), String> {
        if opt_text(opts, "ECU_ADDRESS_EXTENSION").is_some_and(|e| num(e) != Some(0.0)) {
            return Err("an address extension (ECU_ADDRESS_EXTENSION) is not read".into());
        }
        Ok(())
    }

    fn difference(&self, opts: &Options) -> bool {
        match opt_text(opts, "DEPOSIT") {
            Some(d) => d == "DIFFERENCE",
            None => self.difference,
        }
    }

    /// An AXIS_PTS block's points: one row of `MaxAxisPoints`.
    fn axis_pts_data(&self, b: &Block) -> Result<Data, String> {
        let a = b.args(10)?;
        let opts = b.options(10, AXIS_PTS_KEYS)?;
        self.check_common(&opts)?;
        if self.difference(&opts) {
            return Err(
                "axis points stored as differences (DEPOSIT DIFFERENCE) are not read".into(),
            );
        }
        let layout = self.layout(a[4].text())?;
        if layout
            .datatype(|p| matches!(p, Part::Values { .. }))
            .is_some()
        {
            return Err(format!(
                "its record layout {} holds values, not axis points",
                a[4].text()
            ));
        }
        let datatype = layout
            .datatype(|p| *p == Part::Axis(0))
            .ok_or_else(|| format!("its record layout {} has no AXIS_PTS_X", a[4].text()))?
            .to_string();
        let n = count(a[7])?;
        let compu = self.compu(a[6].text())?;
        let (_, at) = layout.place(addr(a[2])?, 0, [n as u64, 0])?;
        Ok(Data {
            address: at[0],
            cell: cell(&datatype, self.big_endian(&opts)?)?,
            rows: 1,
            cols: n,
            column_major: false,
            minor_stride_bits: 0,
            major_stride_bits: 0,
            decimals: opt_text(&opts, "FORMAT").map_or(compu.decimals, decimals),
            units: opt_text(&opts, "PHYS_UNIT").map_or(compu.unit, str::to_string),
            math: compu.math,
            vars: vec![],
            min: num(a[8].text()),
            max: num(a[9].text()),
        })
    }

    fn axis_descr(&self, b: &Block, big: Option<bool>) -> Result<AxisSpec, String> {
        let a = b.args(6)?;
        let opts = b
            .options(6, AXIS_DESCR_KEYS)
            .map_err(|e| format!("its axis: {e}"))?;
        if self.difference(&opts) {
            return Err(
                "axis points stored as differences (DEPOSIT DIFFERENCE) are not read".into(),
            );
        }
        let compu = self.compu(a[2].text())?;
        let decimals = opt_text(&opts, "FORMAT").map_or(compu.decimals, decimals);
        let units = opt_text(&opts, "PHYS_UNIT").map_or(compu.unit.clone(), str::to_string);
        let n = count(a[3])?;
        match a[0].text() {
            "FIX_AXIS" => {
                let series =
                    |k: &str, step: &dyn Fn(f64) -> f64| -> Option<Result<Vec<f64>, String>> {
                        opt_nums(&opts, k).map(|p| {
                            let p = p?;
                            let n = count(&Tok::Word(p[2].to_string()))?;
                            Ok((0..n).map(|i| p[0] + i as f64 * step(p[1])).collect())
                        })
                    };
                let points = match series("FIX_AXIS_PAR", &|shift| 2f64.powf(shift))
                    .or_else(|| series("FIX_AXIS_PAR_DIST", &|dist| dist))
                {
                    Some(p) => p?,
                    None => b
                        .block("FIX_AXIS_PAR_LIST")
                        .ok_or("a FIX_AXIS without FIX_AXIS_PAR, FIX_AXIS_PAR_DIST or FIX_AXIS_PAR_LIST")?
                        .toks()
                        .map(|t| num(t.text()))
                        .collect::<Option<Vec<f64>>>()
                        .filter(|v| !v.is_empty())
                        .ok_or("FIX_AXIS_PAR_LIST holds something that is not a number")?,
                };
                Ok(AxisSpec::Fixed(
                    points
                        .into_iter()
                        .map(|p| fmt_num(compu.math.eval(p), decimals))
                        .collect(),
                ))
            }
            "COM_AXIS" => {
                let name =
                    opt_text(&opts, "AXIS_PTS_REF").ok_or("a COM_AXIS without AXIS_PTS_REF")?;
                let pts = self
                    .axis_pts
                    .get(name)
                    .ok_or_else(|| format!("its axis points {name} are not in the file"))?;
                let d = self
                    .axis_pts_data(pts)
                    .map_err(|e| format!("its axis points {name}: {e}"))?;
                if d.cols != n {
                    return Err(format!(
                        "its axis points {name} are {} points, its axis says {n}",
                        d.cols
                    ));
                }
                Ok(AxisSpec::Common(d))
            }
            "STD_AXIS" => Ok(AxisSpec::Std {
                n,
                big: opt_text(&opts, "BYTE_ORDER")
                    .map(byte_order)
                    .transpose()?
                    .or(big),
                decimals,
                units,
                min: num(a[4].text()),
                max: num(a[5].text()),
                compu,
            }),
            o => Err(format!("{o} axes are not read")),
        }
    }

    fn characteristic(&self, b: &Block) -> Result<Kind, String> {
        let a = b.args(9)?;
        let opts = b.options(9, CHARACTERISTIC_KEYS)?;
        let kind = a[2].text();
        // a block of values (ICA-56): NUMBER n, or MATRIX_DIM x [y [z]]; shown as a table with numbered rows and columns
        let mut block: Option<(usize, usize)> = None;
        let n_axes = match kind {
            "VALUE" => 0,
            "CURVE" => 1,
            "MAP" => 2,
            "VAL_BLK" => {
                let dims: Vec<f64> =
                    match (opt_nums(&opts, "MATRIX_DIM"), opt_nums(&opts, "NUMBER")) {
                        (Some(m), _) => m?,
                        (None, Some(n)) => n?,
                        (None, None) => return Err("a VAL_BLK without NUMBER or MATRIX_DIM".into()),
                    };
                let d = |k: usize| dims.get(k).copied().unwrap_or(1.0);
                if dims.is_empty() || dims.iter().any(|v| *v < 1.0 || v.fract() != 0.0) {
                    return Err("a VAL_BLK whose size is not whole numbers of 1 or more".into());
                }
                if d(2) > 1.0 || dims.len() > 3 {
                    return Err("a VAL_BLK of three dimensions is not read".into());
                }
                block = Some((d(0) as usize, d(1) as usize));
                0
            }
            o => return Err(format!("{o} characteristics are not read")),
        };
        if opts.contains_key("BIT_MASK") {
            return Err("a value in some bits of a number (BIT_MASK) is not read".into());
        }
        if b.block("VIRTUAL_CHARACTERISTIC").is_some()
            || b.block("DEPENDENT_CHARACTERISTIC").is_some()
        {
            return Err("worked out from other values (VIRTUAL or DEPENDENT_CHARACTERISTIC), not stored: not read".into());
        }
        self.check_common(&opts)?;
        let layout = self.layout(a[4].text())?;
        let compu = self.compu(a[6].text())?;
        let big = self.big_endian(&opts)?;
        let descrs: Vec<&Block> = b.blocks("AXIS_DESCR").collect();
        if descrs.len() != n_axes {
            return Err(format!("a {kind} with {} AXIS_DESCR", descrs.len()));
        }
        let axes = descrs
            .iter()
            .map(|d| self.axis_descr(d, big))
            .collect::<Result<Vec<_>, _>>()?;
        let counts: Vec<usize> = axes.iter().map(AxisSpec::count).collect();
        let n_values: usize = match block {
            Some((cols, rows)) => cols.saturating_mul(rows),
            None => counts.iter().product(),
        };
        if n_values > 1 << 20 {
            return Err(format!(
                "{n_values} values is more than Icarus reads in one element"
            ));
        }
        let Some((column_dir, datatype)) = layout.comps.iter().find_map(|c| match c.part {
            Part::Values { column_dir } => Some((column_dir, c.datatype.as_str())),
            _ => None,
        }) else {
            return Err(format!(
                "its record layout {} has no FNC_VALUES",
                a[4].text()
            ));
        };
        for (k, spec) in axes.iter().enumerate() {
            let stored = layout.datatype(|p| *p == Part::Axis(k)).is_some();
            match (spec, stored) {
                (AxisSpec::Std { .. }, false) => {
                    return Err(format!(
                        "a STD_AXIS whose points its record layout does not place (no AXIS_PTS_{})",
                        ["X", "Y"][k]
                    ))
                }
                (AxisSpec::Fixed(_) | AxisSpec::Common(_), true) => {
                    return Err(format!(
                        "its record layout places points for axis {}, which is not a STD_AXIS",
                        ["X", "Y"][k]
                    ))
                }
                _ => {}
            }
        }
        if layout.comps.iter().any(|c| c.part == Part::Axis(1)) && n_axes < 2 {
            return Err("its record layout places points for a Y axis it does not have".into());
        }
        let axis_count = |k: usize| counts.get(k).copied().unwrap_or(0) as u64;
        let (at, axis_at) =
            layout.place(addr(a[3])?, n_values as u64, [axis_count(0), axis_count(1)])?;
        let (rows, cols) = match (block, n_axes) {
            (Some((c, r)), _) => (r, c),
            (None, 0) => (1, 1),
            (None, 1) => (counts[0], 1),
            _ => (counts[1], counts[0]),
        };
        let z = Data {
            address: at,
            cell: cell(datatype, big)?,
            rows,
            cols,
            column_major: column_dir,
            minor_stride_bits: 0,
            major_stride_bits: 0,
            decimals: opt_text(&opts, "FORMAT").map_or(compu.decimals, decimals),
            units: opt_text(&opts, "PHYS_UNIT").map_or(compu.unit, str::to_string),
            math: compu.math,
            vars: vec![],
            min: num(a[7].text()),
            max: num(a[8].text()),
        };
        let mut built = Vec::new();
        for (k, spec) in axes.into_iter().enumerate() {
            built.push(match spec {
                AxisSpec::Fixed(labels) => Axis::Labels(labels),
                AxisSpec::Common(d) => Axis::Stored(d),
                AxisSpec::Std {
                    compu,
                    n,
                    big,
                    decimals,
                    units,
                    min,
                    max,
                } => {
                    let datatype = layout.datatype(|p| *p == Part::Axis(k)).unwrap_or("");
                    Axis::Stored(Data {
                        address: axis_at[k],
                        cell: cell(datatype, big)?,
                        rows: 1,
                        cols: n,
                        column_major: false,
                        minor_stride_bits: 0,
                        major_stride_bits: 0,
                        math: compu.math,
                        vars: vec![],
                        units,
                        decimals,
                        min,
                        max,
                    })
                }
            });
        }
        let mut built = built.into_iter();
        if let Some((cols, rows)) = block {
            return Ok(Kind::Table(Table {
                x: Axis::Index(cols),
                y: Axis::Index(rows),
                z,
            }));
        }
        Ok(match n_axes {
            0 => Kind::Constant(z),
            1 => Kind::Table(Table {
                x: Axis::Index(1),
                y: built.next().unwrap(),
                z,
            }),
            _ => {
                let x = built.next().unwrap();
                Kind::Table(Table {
                    x,
                    y: built.next().unwrap(),
                    z,
                })
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rom::Rom;

    /// An A2L written for this test, describing the same maps as `TWIN_XDF` below (and some things Icarus leaves out).
    const TWIN_A2L: &str = r#"/* Written by hand for Icarus's tests: the twin of TWIN_XDF */
ASAP2_VERSION 1 61
/begin PROJECT Test "Test ECU"
  /begin HEADER "made up" VERSION "1" /end HEADER
  /begin MODULE Engine "Test ECU, engine module"
    /begin A2ML
      block "IF_DATA" taggedunion if_data { "XCP" struct { uint; }; };
    /end A2ML
    /begin MOD_COMMON ""
      BYTE_ORDER MSB_LAST
      ALIGNMENT_BYTE 1
      ALIGNMENT_WORD 2
    /end MOD_COMMON
    /begin MOD_PAR ""
      /begin MEMORY_SEGMENT Cal "calibration" DATA FLASH INTERN 0x8000 0x100 -1 -1 -1 -1 -1 /end MEMORY_SEGMENT
      /begin MEMORY_SEGMENT Work "working memory" VARIABLES RAM INTERN 0x4000 0x100 -1 -1 -1 -1 -1 /end MEMORY_SEGMENT
    /end MOD_PAR
    /begin COMPU_METHOD CM_rpm "engine speed" RAT_FUNC "%6.0" "rpm"
      COEFFS 0 4 0 0 0 1
    /end COMPU_METHOD
    /begin COMPU_METHOD CM_deg "ignition angle" LINEAR "%5.1" "deg"
      COEFFS_LINEAR 0.5 -10
    /end COMPU_METHOD
    /begin COMPU_METHOD CM_load "load" IDENTICAL "%4.0" "%" /end COMPU_METHOD
    /begin COMPU_METHOD CM_tmot "coolant" LINEAR "%4.0" "degC"
      COEFFS_LINEAR 1 -40
    /end COMPU_METHOD
    /begin COMPU_METHOD CM_idle "idle speed" RAT_FUNC "%5.0" "rpm"
      COEFFS 0 10 400 0 0 1   // stored = 10 * rpm + 400
    /end COMPU_METHOD
    /begin COMPU_METHOD CM_square "squared" RAT_FUNC "%5.1" ""
      COEFFS 1 0 0 0 0 1
    /end COMPU_METHOD
    /begin COMPU_METHOD CM_states "fan states" TAB_VERB "%3.0" ""
      COMPU_TAB_REF VT_fan
    /end COMPU_METHOD
    /begin RECORD_LAYOUT RL_u8 FNC_VALUES 1 UBYTE ROW_DIR DIRECT /end RECORD_LAYOUT
    /begin RECORD_LAYOUT RL_u16 FNC_VALUES 1 UWORD ROW_DIR DIRECT /end RECORD_LAYOUT
    /begin RECORD_LAYOUT RL_s8_columns FNC_VALUES 1 SBYTE COLUMN_DIR DIRECT /end RECORD_LAYOUT
    /begin RECORD_LAYOUT RL_axis_u16 AXIS_PTS_X 1 UWORD INDEX_INCR DIRECT /end RECORD_LAYOUT
    /begin RECORD_LAYOUT RL_curve
      AXIS_PTS_X 1 UBYTE INDEX_INCR DIRECT
      FNC_VALUES 2 UWORD ROW_DIR DIRECT
    /end RECORD_LAYOUT
    /begin RECORD_LAYOUT RL_counted
      NO_AXIS_PTS_X 1 UBYTE
      AXIS_PTS_X 2 UBYTE INDEX_INCR DIRECT
      FNC_VALUES 3 UBYTE ROW_DIR DIRECT
    /end RECORD_LAYOUT
    /begin AXIS_PTS RpmPoints "engine speed breakpoints" 0x8000 nmot RL_axis_u16 0 CM_rpm 4 0 8000
    /end AXIS_PTS
    /begin CHARACTERISTIC IgnBase "base ignition timing" MAP 0x8010 RL_s8_columns 0 CM_deg -20 50
      /begin AXIS_DESCR COM_AXIS nmot CM_rpm 4 0 8000
        AXIS_PTS_REF RpmPoints
      /end AXIS_DESCR
      /begin AXIS_DESCR FIX_AXIS rl CM_load 3 0 100
        FIX_AXIS_PAR_DIST 20 20 3
      /end AXIS_DESCR
      /begin IF_DATA XCP LINK_MAP "IgnBase" 0x8010 0 0 0 1 0 0 /end IF_DATA
    /end CHARACTERISTIC
    /begin CHARACTERISTIC WarmUp "warm-up enrichment" CURVE 0x8030 RL_curve 0 NO_COMPU_METHOD 0 65535
      FORMAT "%5.0"
      /begin AXIS_DESCR STD_AXIS tmot CM_tmot 3 -40 215 /end AXIS_DESCR
    /end CHARACTERISTIC
    /begin CHARACTERISTIC IdleTarget "idle speed target" VALUE 0x8040 RL_u16 0 CM_idle 500 2000
      BYTE_ORDER MSB_FIRST
    /end CHARACTERISTIC
    /begin CHARACTERISTIC FanMode "fan mode" VALUE 0x8050 RL_u8 0 CM_states 0 3 /end CHARACTERISTIC
    /begin CHARACTERISTIC Squared "squared" VALUE 0x8051 RL_u8 0 CM_square 0 10 /end CHARACTERISTIC
    /begin CHARACTERISTIC Counted "counted in the image" CURVE 0x8060 RL_counted 0 NO_COMPU_METHOD 0 255
      /begin AXIS_DESCR STD_AXIS x NO_COMPU_METHOD 4 0 255 /end AXIS_DESCR
    /end CHARACTERISTIC
    /begin CHARACTERISTIC Block "a block of values" VAL_BLK 0x8070 RL_u8 0 NO_COMPU_METHOD 0 255
      NUMBER 4
    /end CHARACTERISTIC
    /begin CHARACTERISTIC Rescaled "rescale axis" CURVE 0x8080 RL_u8 0 NO_COMPU_METHOD 0 255
      /begin AXIS_DESCR RES_AXIS x NO_COMPU_METHOD 4 0 255 AXIS_PTS_REF RpmPoints /end AXIS_DESCR
    /end CHARACTERISTIC
    /begin CHARACTERISTIC Mystery "a keyword from a later version" VALUE 0x8090 RL_u8 0 NO_COMPU_METHOD 0 255
      SOMETHING_NEW 1 2
    /end CHARACTERISTIC
    /begin MEASUREMENT nmot "engine speed" UWORD CM_rpm 1 100 0 8000
      ECU_ADDRESS 0x4000
    /end MEASUREMENT
    /begin FUNCTION Ignition "ignition"
      /begin DEF_CHARACTERISTIC IgnBase /end DEF_CHARACTERISTIC
    /end FUNCTION
  /end MODULE
/end PROJECT
"#;

    /// The same maps as a TunerPro definition (written for this test).
    const TWIN_XDF: &str = r#"<!-- Written by hand for Icarus's tests: the twin of TWIN_A2L -->
<XDFFORMAT version="1.70">
  <XDFHEADER>
    <deftitle>Test ECU</deftitle>
    <BASEOFFSET offset="0x8000" subtract="1" />
    <DEFAULTS datasizeinbits="8" sigdigits="2" outputtype="1" signed="0" lsbfirst="1" float="0" />
    <REGION type="0xFFFFFFFF" startaddress="0x0" size="0x100" name="Binary File" />
  </XDFHEADER>
  <XDFTABLE uniqueid="0x1">
    <title>RpmPoints</title>
    <XDFAXIS id="x" uniqueid="0x0"><indexcount>1</indexcount></XDFAXIS>
    <XDFAXIS id="y" uniqueid="0x0"><indexcount>4</indexcount></XDFAXIS>
    <XDFAXIS id="z">
      <EMBEDDEDDATA mmedtypeflags="0x02" mmedaddress="0x8000" mmedelementsizebits="16" mmedrowcount="4" mmedcolcount="1" />
      <units>rpm</units>
      <decimalpl>0</decimalpl>
      <min>0</min>
      <max>8000</max>
      <MATH equation="X/4"><VAR id="X" /></MATH>
    </XDFAXIS>
  </XDFTABLE>
  <XDFTABLE uniqueid="0x2">
    <title>IgnBase</title>
    <XDFAXIS id="x" uniqueid="0x0">
      <indexcount>4</indexcount>
      <embedinfo type="3" linkobjid="0x1" />
      <MATH equation="X"><VAR id="X" /></MATH>
    </XDFAXIS>
    <XDFAXIS id="y" uniqueid="0x0">
      <indexcount>3</indexcount>
      <LABEL index="0" value="20" />
      <LABEL index="1" value="40" />
      <LABEL index="2" value="60" />
    </XDFAXIS>
    <XDFAXIS id="z">
      <EMBEDDEDDATA mmedtypeflags="0x05" mmedaddress="0x8010" mmedelementsizebits="8" mmedrowcount="3" mmedcolcount="4" />
      <units>deg</units>
      <decimalpl>1</decimalpl>
      <min>-20</min>
      <max>50</max>
      <MATH equation="0.5*X-10"><VAR id="X" /></MATH>
    </XDFAXIS>
  </XDFTABLE>
  <XDFTABLE uniqueid="0x3">
    <title>WarmUp</title>
    <XDFAXIS id="x" uniqueid="0x0"><indexcount>1</indexcount></XDFAXIS>
    <XDFAXIS id="y" uniqueid="0x0">
      <indexcount>3</indexcount>
      <EMBEDDEDDATA mmedaddress="0x8030" mmedelementsizebits="8" mmedcolcount="3" />
      <decimalpl>0</decimalpl>
      <MATH equation="X-40"><VAR id="X" /></MATH>
    </XDFAXIS>
    <XDFAXIS id="z">
      <EMBEDDEDDATA mmedtypeflags="0x02" mmedaddress="0x8034" mmedelementsizebits="16" mmedrowcount="3" mmedcolcount="1" />
      <decimalpl>0</decimalpl>
      <min>0</min>
      <max>65535</max>
    </XDFAXIS>
  </XDFTABLE>
  <XDFCONSTANT uniqueid="0x4">
    <title>IdleTarget</title>
    <EMBEDDEDDATA mmedtypeflags="0x00" mmedaddress="0x8040" mmedelementsizebits="16" />
    <units>rpm</units>
    <decimalpl>0</decimalpl>
    <min>500</min>
    <max>2000</max>
    <MATH equation="(X-400)/10"><VAR id="X" /></MATH>
  </XDFCONSTANT>
</XDFFORMAT>"#;

    fn image() -> Rom {
        let mut img = vec![0u8; 0x100];
        for (i, raw) in [3200u16, 8000, 12000, 16000].iter().enumerate() {
            img[2 * i..2 * i + 2].copy_from_slice(&raw.to_le_bytes()); // 800, 2000, 3000, 4000 rpm
        }
        // ignition, column by column (each engine speed's three loads), 0.5 deg a step from -10
        img[0x10..0x1C].copy_from_slice(&[20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 0xF0]);
        img[0x30..0x33].copy_from_slice(&[60, 100, 160]); // 20, 60, 120 degC
        for (i, v) in [300u16, 200, 100].iter().enumerate() {
            img[0x34 + 2 * i..0x36 + 2 * i].copy_from_slice(&v.to_le_bytes()); // after the axis, on a word boundary
        }
        img[0x40..0x42].copy_from_slice(&12400u16.to_be_bytes()); // (12400 - 400) / 10 = 1200 rpm, most significant first
        Rom::new(img)
    }

    #[test]
    fn an_a2l_loads_the_same_maps_as_its_xdf_twin() {
        let a = crate::def::parse_any(TWIN_A2L.as_bytes()).unwrap();
        let x = crate::def::parse_any(TWIN_XDF.as_bytes()).unwrap();
        assert!(x.skipped.is_empty(), "{:?}", x.skipped);
        let (ad, xd) = (&a.definition, &x.definition);
        assert_eq!(ad.title, "Test ECU");
        assert_eq!(
            (ad.base_offset, ad.base_subtract, ad.image_size),
            (0x8000, true, Some(0x100)),
            "the image begins at the calibration segment, not the RAM one"
        );
        let titles: Vec<&str> = ad.elements.iter().map(|e| e.title.as_str()).collect();
        assert_eq!(
            titles,
            vec!["RpmPoints", "IgnBase", "WarmUp", "IdleTarget", "Block"]
        );
        assert_eq!(ad.element("IgnBase").unwrap().categories, vec!["Ignition"]);
        assert_eq!(
            ad.element("RpmPoints").unwrap().categories,
            vec!["Axis points"]
        );
        assert_eq!(
            ad.element("IgnBase").unwrap().description,
            "base ignition timing"
        );

        let rom = image();
        for name in ["RpmPoints", "IgnBase", "WarmUp"] {
            let (Kind::Table(at), Kind::Table(xt)) = (
                &ad.element(name).unwrap().kind,
                &xd.element(name).unwrap().kind,
            ) else {
                panic!("{name} is a table in both")
            };
            let (av, xv) = (ad.table(&rom, at).unwrap(), xd.table(&rom, xt).unwrap());
            assert_eq!(av, xv, "{name}");
            assert_eq!(
                (
                    at.z.rows,
                    at.z.cols,
                    &at.z.units,
                    at.z.decimals,
                    at.z.min,
                    at.z.max
                ),
                (
                    xt.z.rows,
                    xt.z.cols,
                    &xt.z.units,
                    xt.z.decimals,
                    xt.z.min,
                    xt.z.max
                ),
                "{name}"
            );
            assert_eq!(ad.span(&at.z), xd.span(&xt.z), "{name}: the same bytes");
        }
        let Kind::Table(ign) = &ad.element("IgnBase").unwrap().kind else {
            panic!()
        };
        let v = ad.table(&rom, ign).unwrap();
        assert_eq!(v.x, vec!["800", "2000", "3000", "4000"]);
        assert_eq!(v.y, vec!["20", "40", "60"]);
        assert_eq!(
            v.z,
            vec![
                vec![0.0, 15.0, 30.0, 45.0],
                vec![5.0, 20.0, 35.0, 50.0],
                vec![10.0, 25.0, 40.0, -18.0]
            ],
            "COLUMN_DIR: each engine speed's loads one after another"
        );
        let Kind::Table(warm) = &ad.element("WarmUp").unwrap().kind else {
            panic!()
        };
        let w = ad.table(&rom, warm).unwrap();
        assert_eq!(
            (w.y.clone(), w.z.concat()),
            (
                vec!["20".to_string(), "60".into(), "120".into()],
                vec![300.0, 200.0, 100.0]
            )
        );
        let (Kind::Constant(ai), Kind::Constant(xi)) = (
            &ad.element("IdleTarget").unwrap().kind,
            &xd.element("IdleTarget").unwrap().kind,
        ) else {
            panic!()
        };
        assert_eq!(ad.values(&rom, ai).unwrap(), vec![vec![1200.0]]);
        assert_eq!(ad.values(&rom, ai).unwrap(), xd.values(&rom, xi).unwrap());

        // an edit through either definition lands in the same byte
        let (mut r1, mut r2) = (image(), image());
        let Kind::Table(xign) = &xd.element("IgnBase").unwrap().kind else {
            panic!()
        };
        assert_eq!(ad.set(&mut r1, &ign.z, 2, 3, -6.0).unwrap(), -6.0);
        assert_eq!(xd.set(&mut r2, &xign.z, 2, 3, -6.0).unwrap(), -6.0);
        assert_eq!(r1.changes(), vec![0x1B..0x1C]);
        assert_eq!(r1.bytes(), r2.bytes());
        assert_eq!(ad.set(&mut r1, ai, 0, 0, 900.0).unwrap(), 900.0);
        assert_eq!(&r1.bytes()[0x40..0x42], &9400u16.to_be_bytes());
        assert!(
            ad.set(&mut r1, ai, 0, 0, 2500.0).is_err(),
            "above the upper limit"
        );
        assert_eq!(
            ad.covering(0x02..0x03)
                .iter()
                .map(|e| e.title.as_str())
                .collect::<Vec<_>>(),
            vec!["RpmPoints", "IgnBase"],
            "a changed breakpoint shows in the map that uses it"
        );
    }

    #[test]
    fn what_is_not_read_is_listed_with_the_reason() {
        let a = parse(TWIN_A2L).unwrap();
        let why = |name: &str| {
            a.skipped
                .iter()
                .find(|s| s.starts_with(&format!("{name}: ")))
                .cloned()
                .unwrap_or_else(|| panic!("{name} not listed: {:?}", a.skipped))
        };
        assert!(why("FanMode").contains("TAB_VERB"));
        assert!(why("Squared").contains("squared terms"));
        assert!(why("Counted").contains("NO_AXIS_PTS_X"));
        assert!(why("Rescaled").contains("RES_AXIS axes are not read"));
        assert!(why("Mystery").contains("SOMETHING_NEW is a keyword Icarus does not read"));
        assert_eq!(a.skipped.len(), 5, "{:?}", a.skipped);
    }

    /// ICA-56: an A2L written for this test with the constructs the twin leaves out: value blocks (NUMBER and MATRIX_DIM)
    /// and FORM conversions that Icarus's own formulas can say load; the rest is named with the reason (a three-dimensional
    /// block, FORM with a function, a second input or bit operators, TAB_INTP, BIT_MASK, 64-bit integers, CURVE_AXIS, and
    /// /include).
    #[test]
    fn more_constructs_load_or_are_named() {
        const MORE: &str = r#"/* Written by hand for Icarus's tests */
ASAP2_VERSION 1 61
/begin PROJECT More "more constructs"
  /begin MODULE M ""
    /begin MOD_COMMON "" BYTE_ORDER MSB_LAST /end MOD_COMMON
    /begin MOD_PAR ""
      /begin MEMORY_SEGMENT Cal "calibration" DATA FLASH INTERN 0x8000 0x100 -1 -1 -1 -1 -1 /end MEMORY_SEGMENT
    /end MOD_PAR
    /begin COMPU_METHOD CM_form "a formula" FORM "%5.1" "kPa"
      /begin FORMULA "X1*0.5+3" /end FORMULA
    /end COMPU_METHOD
    /begin COMPU_METHOD CM_hex "a formula in hex" FORM "%5.0" ""
      /begin FORMULA "(X1 - 0x10) / 2" FORMULA_INV "X1*2+0x10" /end FORMULA
    /end COMPU_METHOD
    /begin COMPU_METHOD CM_sin "a function" FORM "%5.1" "" /begin FORMULA "sin(X1)" /end FORMULA /end COMPU_METHOD
    /begin COMPU_METHOD CM_two "two inputs" FORM "%5.1" "" /begin FORMULA "X1+X2" /end FORMULA /end COMPU_METHOD
    /begin COMPU_METHOD CM_bits "bits" FORM "%5.1" "" /begin FORMULA "X1&0x0F" /end FORMULA /end COMPU_METHOD
    /begin COMPU_METHOD CM_interp "a table" TAB_INTP "%5.1" "" COMPU_TAB_REF T1 /end COMPU_METHOD
    /begin RECORD_LAYOUT RL_u8 FNC_VALUES 1 UBYTE ROW_DIR DIRECT /end RECORD_LAYOUT
    /begin RECORD_LAYOUT RL_i64 FNC_VALUES 1 A_INT64 ROW_DIR DIRECT /end RECORD_LAYOUT
    /begin CHARACTERISTIC Pressure "" VALUE 0x8000 RL_u8 0 CM_form 0 200 /end CHARACTERISTIC
    /begin CHARACTERISTIC Hexed "" VALUE 0x8001 RL_u8 0 CM_hex 0 200 /end CHARACTERISTIC
    /begin CHARACTERISTIC Row "four values" VAL_BLK 0x8010 RL_u8 0 NO_COMPU_METHOD 0 255 NUMBER 4 /end CHARACTERISTIC
    /begin CHARACTERISTIC Grid "three by two" VAL_BLK 0x8020 RL_u8 0 NO_COMPU_METHOD 0 255 MATRIX_DIM 3 2 1 /end CHARACTERISTIC
    /begin CHARACTERISTIC Cube "" VAL_BLK 0x8030 RL_u8 0 NO_COMPU_METHOD 0 255 MATRIX_DIM 2 2 2 /end CHARACTERISTIC
    /begin CHARACTERISTIC Sine "" VALUE 0x8040 RL_u8 0 CM_sin 0 1 /end CHARACTERISTIC
    /begin CHARACTERISTIC Two "" VALUE 0x8041 RL_u8 0 CM_two 0 1 /end CHARACTERISTIC
    /begin CHARACTERISTIC Bits "" VALUE 0x8042 RL_u8 0 CM_bits 0 1 /end CHARACTERISTIC
    /begin CHARACTERISTIC Interp "" VALUE 0x8043 RL_u8 0 CM_interp 0 1 /end CHARACTERISTIC
    /begin CHARACTERISTIC Masked "" VALUE 0x8044 RL_u8 0 NO_COMPU_METHOD 0 1 BIT_MASK 0x04 /end CHARACTERISTIC
    /begin CHARACTERISTIC Wide "" VALUE 0x8048 RL_i64 0 NO_COMPU_METHOD 0 1 /end CHARACTERISTIC
    /begin CHARACTERISTIC Shaped "" CURVE 0x8050 RL_u8 0 NO_COMPU_METHOD 0 255
      /begin AXIS_DESCR CURVE_AXIS x NO_COMPU_METHOD 4 0 255 CURVE_AXIS_REF Row /end AXIS_DESCR
    /end CHARACTERISTIC
    /include "more.a2l"
  /end MODULE
/end PROJECT
"#;
        let a = parse(MORE).unwrap();
        let d = &a.definition;
        let mut img = vec![0u8; 0x100];
        img[0x00] = 100;
        img[0x01] = 0x30;
        img[0x10..0x14].copy_from_slice(&[1, 2, 3, 4]);
        img[0x20..0x26].copy_from_slice(&[10, 11, 12, 20, 21, 22]);
        let rom = Rom::new(img);
        let value = |name: &str| {
            let z = match &d.element(name).unwrap().kind {
                Kind::Constant(z) => z,
                Kind::Table(t) => &t.z,
                Kind::Flag(_) => panic!("{name} is a flag"),
            };
            d.values(&rom, z).unwrap()
        };
        assert_eq!(value("Pressure"), vec![vec![53.0]], "100 * 0.5 + 3");
        assert_eq!(value("Hexed"), vec![vec![16.0]], "(0x30 - 0x10) / 2");
        assert_eq!(
            value("Row"),
            vec![vec![1.0, 2.0, 3.0, 4.0]],
            "one row of four"
        );
        assert_eq!(
            value("Grid"),
            vec![vec![10.0, 11.0, 12.0], vec![20.0, 21.0, 22.0]],
            "two rows of three, row by row"
        );
        let why = |name: &str| {
            a.skipped
                .iter()
                .find(|s| s.starts_with(&format!("{name}: ")))
                .cloned()
                .unwrap_or_else(|| panic!("{name} not listed: {:?}", a.skipped))
        };
        assert!(why("Cube").contains("three dimensions"));
        assert!(why("Sine").contains("functions"));
        assert!(why("Two").contains("second input (X2)"));
        assert!(why("Bits").contains("bit or logic"));
        assert!(why("Interp").contains("TAB_INTP"));
        assert!(why("Masked").contains("BIT_MASK"));
        assert!(why("Wide").contains("64-bit integers"));
        assert!(why("Shaped").contains("CURVE_AXIS axes are not read"));
        assert!(
            a.skipped.iter().any(|s| s.starts_with("/include more.a2l")),
            "{:?}",
            a.skipped
        );
        assert_eq!(a.skipped.len(), 9, "{:?}", a.skipped);
    }

    #[test]
    fn broken_files_are_errors_with_the_line() {
        let e = parse("/begin PROJECT P \"\"\n  /begin MODULE M \"\"\n/end PROJECT\n")
            .unwrap_err()
            .to_string();
        assert!(
            e.contains("line 3") && e.contains("/begin MODULE of line 2"),
            "{e}"
        );
        assert!(parse("/begin PROJECT P \"never closed\n")
            .unwrap_err()
            .to_string()
            .contains("never closed"));
        assert!(parse("ASAP2_VERSION 1 61\n")
            .unwrap_err()
            .to_string()
            .contains("no /begin PROJECT"));
        assert!(crate::def::parse_any(b"just text")
            .unwrap_err()
            .to_string()
            .contains("neither"));
        // a Latin-1 degree sign is read, not refused
        let latin = b"/begin PROJECT P \"\xB0C\" /begin MODULE M \"\" /end MODULE /end PROJECT";
        assert_eq!(
            crate::def::parse_any(latin).unwrap().definition.title,
            "\u{b0}C"
        );
        // a quote inside a string
        assert_eq!(
            tokens(r#""say ""hi"" \"now\"""#).unwrap()[0].0,
            Tok::Str("say \"hi\" \"now\"".into())
        );
    }
}
