//! Module data: each module's own live values (a dealer tool's data list), for any make, kept as data. A definition says
//! which read-only request to send a module (`22` read data by identifier, `21` by local identifier, `1A` identification)
//! and what the bytes of its answer mean. This crate only reads and writes the files and decodes answers; sending the
//! requests, and every safety gate on them, is `icarus-obd`'s `moddata`.
//!
//! A **set** is one folder from one source (an OBDb import, a hand-written list):
//!
//! ```text
//! <set>/
//!   set.toml      what the set is for (make, models, years), where it came from, its license and attribution
//!   700.toml      one file per module, named by its request ID ("750.2A.toml" with an extended address)
//! ```
//!
//! A module file has a `[module]` table and `[[request]]` entries, each with `[[request.value]]` entries. Numbers may be
//! written in hex (`tx = 0x701`). Every field is listed on the structs below; an unknown field is an error, as in the
//! vehicle profiles, so a typo never passes as "not given".
//!
//! Files carry no raw request bytes: Icarus builds every request itself from `service` and `ident` (`22 hi lo`, `21 id`,
//! `1A id`), and refuses any other service when the file is read.

pub mod capture;
pub mod ddt;
pub mod decode;
pub mod obdb;
mod sha256;

pub use decode::{decode, Decoded, Shown};
pub use sha256::sha256_hex;

use serde::Deserialize;
use std::collections::{BTreeMap, BTreeSet};

/// The format number this Icarus reads and writes.
pub const FORMAT: u32 = 1;

/// The only services a definition may name: they read and change nothing.
pub const SERVICES: [u8; 3] = [0x21, 0x22, 0x1A];

/// The most bytes a definition may reach into an answer: an ISO-TP message carries at most 4095 bytes (classic CAN), so
/// a value past that can never be read, and a bigger number in a file is a mistake, not a far-away value.
pub const MAX_BYTES: usize = 4095;

/// Year ranges are inclusive `[from, to]` pairs; an empty list means every year.
pub type Years = Vec<[u16; 2]>;

/// Whether `year` is in `ranges` (an empty list holds every year).
pub fn year_in(ranges: &[[u16; 2]], year: u16) -> bool {
    ranges.is_empty() || ranges.iter().any(|[a, b]| (*a..=*b).contains(&year))
}

/// Whether two year lists share a year (an empty list is every year).
pub fn years_overlap(a: &[[u16; 2]], b: &[[u16; 2]]) -> bool {
    a.is_empty()
        || b.is_empty()
        || a.iter()
            .any(|[a0, a1]| b.iter().any(|[b0, b1]| a0 <= b1 && b0 <= a1))
}

/// `set.toml`.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct SetInfo {
    pub format: u32,
    /// The set's name, which is also its folder's name.
    pub name: String,
    /// A vehicle profile's name (`toyota`), matched against the profile the VIN picks.
    #[serde(default)]
    pub make: String,
    /// Words matched against the vehicle's model; empty: every model of the make.
    #[serde(default)]
    pub models: Vec<String>,
    #[serde(default)]
    pub years: Years,
    pub source: Source,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct Source {
    /// "OBDb", "DDT2000", "opendbc", "Icarus" (built in) or "hand".
    pub from: String,
    #[serde(default)]
    pub file: String,
    #[serde(default)]
    pub sha256: String,
    #[serde(default)]
    pub imported: String,
    #[serde(default)]
    pub license: String,
    /// Shown wherever these values appear.
    #[serde(default)]
    pub attribution: String,
}

/// One module file.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ModuleDef {
    pub format: u32,
    pub module: ModuleHead,
    #[serde(default, rename = "request")]
    pub requests: Vec<RequestDef>,
}

/// How a module is reached.
#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Ids {
    /// 11-bit CAN IDs, the only kind Icarus reaches today.
    #[default]
    Can11,
    /// 29-bit CAN IDs.
    Can29,
    /// K-line (ISO 9141-2, ISO 14230).
    Kline,
}

impl Ids {
    pub fn word(self) -> &'static str {
        match self {
            Ids::Can11 => "can11",
            Ids::Can29 => "can29",
            Ids::Kline => "kline",
        }
    }
}

/// Which services a module's identity and codes use.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Dialect {
    Uds,
    Kwp,
}

#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ModuleHead {
    pub name: String,
    /// The ID requests go to.
    pub tx: u32,
    /// The ID answers come from: tx + 8 when not given.
    #[serde(default)]
    pub rx: Option<u32>,
    /// An extended address byte (Toyota behind 750, BMW): not reachable yet.
    #[serde(default)]
    pub ext: Option<u8>,
    /// The tester address byte that goes with `ext`.
    #[serde(default)]
    pub tester: Option<u8>,
    #[serde(default)]
    pub ids: Ids,
    /// As a vehicle profile's module `bus`: "hs-can" (the default) or another bus.
    #[serde(default)]
    pub bus: Option<String>,
    #[serde(default)]
    pub dialect: Option<Dialect>,
    /// The source's word for what the module is ("engine", "transmission", "airbag"...); "airbag" means identity only.
    #[serde(default)]
    pub kind: String,
    /// Identity fingerprints (what the module's identification values read), to tell variants apart.
    #[serde(default)]
    pub identity: Vec<Identity>,
}

impl ModuleHead {
    /// The answer ID: `rx`, else tx + 8 (saturating: an ID that large is refused by `validate` anyway).
    pub fn rx(&self) -> u32 {
        self.rx.unwrap_or(self.tx.saturating_add(8))
    }

    /// "701", "750.2A", "18DA10F1".
    pub fn label(&self) -> String {
        let id = match self.ids {
            Ids::Can29 => format!("{:08X}", self.tx),
            _ => format!("{:03X}", self.tx),
        };
        match self.ext {
            Some(e) => format!("{id}.{e:02X}"),
            None => id,
        }
    }
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct Identity {
    #[serde(default)]
    pub diag: Option<String>,
    #[serde(default)]
    pub supplier: Option<String>,
    #[serde(default)]
    pub soft: Option<String>,
    #[serde(default)]
    pub version: Option<String>,
}

#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct RequestDef {
    /// 0x21, 0x22 or 0x1A.
    pub service: u8,
    /// Two bytes for 22, one for 21 and 1A.
    pub ident: u16,
    /// How often it is worth asking, in seconds (never faster than the pacer allows).
    #[serde(default)]
    pub every: Option<f64>,
    /// Data bytes needed after the service byte and the identifier's echo; filled in when not given.
    #[serde(default)]
    pub min_len: Option<usize>,
    #[serde(default)]
    pub timeout_ms: Option<u32>,
    /// Model years the request is for (empty: every year).
    #[serde(default)]
    pub years: Years,
    /// Years the source marks as not yet verified.
    #[serde(default)]
    pub unverified: Years,
    /// The source says the request needs a diagnostic session; Icarus opens none, so it is never sent.
    #[serde(default)]
    pub needs_session: Option<String>,
    /// A sample positive answer in hex (`62 1F 01 ...`), decoded offline by `icarus data --sample`.
    #[serde(default)]
    pub sample: Option<String>,
    /// More sample answers (a capture keeps the different answers it saw), decoded by `--sample` too.
    #[serde(default)]
    pub samples: Vec<String>,
    #[serde(default, rename = "value")]
    pub values: Vec<ValueDef>,
}

impl RequestDef {
    /// The bytes Icarus sends: `22 hi lo`, `21 id` or `1A id`.
    pub fn bytes(&self) -> Vec<u8> {
        if self.service == 0x22 {
            vec![0x22, (self.ident >> 8) as u8, self.ident as u8]
        } else {
            vec![self.service, self.ident as u8]
        }
    }

    /// Bytes of the identifier's echo in the answer.
    pub fn echo_len(&self) -> usize {
        if self.service == 0x22 {
            2
        } else {
            1
        }
    }

    /// "22 1F01", "21 04".
    pub fn label(&self) -> String {
        if self.service == 0x22 {
            format!("22 {:04X}", self.ident)
        } else {
            format!("{:02X} {:02X}", self.service, self.ident)
        }
    }

    /// The data bytes after the echo in a sample answer, when one is given and it is this request's positive answer.
    pub fn sample_data(&self) -> Result<Option<Vec<u8>>, String> {
        let Some(s) = &self.sample else {
            return Ok(None);
        };
        self.answer_data(s).map(Some)
    }

    /// The data bytes after the echo of every sample answer (`sample`, then `samples`).
    pub fn samples_data(&self) -> Result<Vec<Vec<u8>>, String> {
        self.sample
            .iter()
            .chain(&self.samples)
            .map(|s| self.answer_data(s))
            .collect()
    }

    /// The data bytes after the echo of an answer in hex, when it is this request's positive answer.
    fn answer_data(&self, s: &str) -> Result<Vec<u8>, String> {
        let b = parse_hex(s)?;
        let head = self.bytes();
        if b.first() != Some(&(self.service + 0x40)) || b.get(1..head.len()) != Some(&head[1..]) {
            return Err(format!(
                "sample {s:?} is not a positive answer to {}",
                self.label()
            ));
        }
        Ok(b[head.len()..].to_vec())
    }
}

/// Bit order of a value (see `decode`).
#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "kebab-case")]
pub enum Order {
    /// Most significant bit first, from bit `bit` of `byte` (bit 0 = 0x80), running on into the next bytes.
    #[default]
    Msb,
    /// The bytes holding the value reversed first, then read as `msb` (OBDb's `blsb`); only above 8 bits.
    LsbBytes,
    /// DDT2000's little endian: the bytes covering `bit .. bit + bits` as one number, shifted right by `bit`, masked to
    /// `bits`, then its bytes reversed. Unverified until the DDT2000 importer (build 3) is tested with a real file: how
    /// DDT's BitOffset is meant is read from the format's description, not yet checked against one.
    DdtLittle,
}

impl Order {
    pub fn word(self) -> &'static str {
        match self {
            Order::Msb => "msb",
            Order::LsbBytes => "lsb-bytes",
            Order::DdtLittle => "ddt-little",
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum ValueKind {
    #[default]
    Number,
    /// A raw value named from `states`.
    State,
    /// Text (ASCII), with 00, FF and spaces trimmed.
    Text,
    /// Bytes shown in hex.
    Hex,
}

impl ValueKind {
    pub fn word(self) -> &'static str {
        match self {
            ValueKind::Number => "number",
            ValueKind::State => "state",
            ValueKind::Text => "text",
            ValueKind::Hex => "hex",
        }
    }
}

fn one() -> f64 {
    1.0
}

#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ValueDef {
    /// Unique within the module: the key for state, live data and recordings.
    pub id: String,
    pub name: String,
    /// Grouping ("Transmission.Temperatures").
    #[serde(default)]
    pub path: String,
    #[serde(default)]
    pub about: String,
    /// 0-based, counted from the first byte after the service byte and the identifier's echo.
    pub byte: usize,
    /// 0-7. For `msb` and `lsb-bytes`: where the value starts, counted from the most significant bit of `byte` (0 = 0x80).
    /// For `ddt-little`: how far the bytes from `byte` on, read as one number, are shifted right (0 = from the least
    /// significant bit), as DDT2000's BitOffset is read; that reading of DDT's files is not yet checked against a real
    /// file, so `ddt-little` stays unverified until the DDT2000 importer (build 3) is tested with one.
    #[serde(default)]
    pub bit: u8,
    /// 1-64 for numbers and states; text and hex: a multiple of 8.
    pub bits: u16,
    #[serde(default)]
    pub order: Order,
    #[serde(default)]
    pub signed: bool,
    /// value = (raw * mul + bias) / div + add
    #[serde(default = "one")]
    pub mul: f64,
    #[serde(default)]
    pub bias: f64,
    #[serde(default = "one")]
    pub div: f64,
    #[serde(default)]
    pub add: f64,
    #[serde(default)]
    pub decimals: Option<u8>,
    /// "°C", "km/h", "kPa", ... (converted for display where US units are the custom).
    #[serde(default)]
    pub unit: String,
    /// With `max` (and max > min): outside it the value is clamped and marked "at limit".
    #[serde(default)]
    pub min: Option<f64>,
    #[serde(default)]
    pub max: Option<f64>,
    /// A value at or below this means "no reading".
    #[serde(default)]
    pub null_below: Option<f64>,
    /// A value at or above this means "no reading".
    #[serde(default)]
    pub null_above: Option<f64>,
    #[serde(default)]
    pub kind: ValueKind,
    /// `kind = "state"`: raw value (a decimal string) -> label.
    #[serde(default)]
    pub states: BTreeMap<String, String>,
    /// True only for built-in values seen on a real vehicle; imports are false.
    #[serde(default)]
    pub checked: bool,
    /// Kept, but not offered by default.
    #[serde(default)]
    pub hidden: bool,
    /// What the importer assumed about this value, if anything.
    #[serde(default)]
    pub note: String,
}

impl ValueDef {
    /// A plain 8-bit unsigned number at `byte`, scaled 1:1 (for building values in code).
    pub fn new(id: &str, name: &str, byte: usize, bits: u16) -> ValueDef {
        ValueDef {
            id: id.into(),
            name: name.into(),
            path: String::new(),
            about: String::new(),
            byte,
            bit: 0,
            bits,
            order: Order::Msb,
            signed: false,
            mul: 1.0,
            bias: 0.0,
            div: 1.0,
            add: 0.0,
            decimals: None,
            unit: String::new(),
            min: None,
            max: None,
            null_below: None,
            null_above: None,
            kind: ValueKind::Number,
            states: BTreeMap::new(),
            checked: false,
            hidden: false,
            note: String::new(),
        }
    }

    /// Bytes of the answer (after the echo) the value needs. Saturating: a `byte` too large to count in bits needs more
    /// than any answer holds (`validate` refuses one past `MAX_BYTES`).
    pub fn end_byte(&self) -> usize {
        self.byte
            .saturating_mul(8)
            .saturating_add(self.bit as usize + self.bits as usize)
            .div_ceil(8)
    }
}

/// What an importer made of a file: the set, its modules, and everything not imported with the reason.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Imported {
    pub set: Option<SetInfo>,
    pub modules: Vec<ModuleDef>,
    /// (what, why) for each thing left out.
    pub left_out: Vec<(String, String)>,
    /// What the importer assumed (an answer ID, the order of on/off...).
    pub notes: Vec<String>,
}

/// Hex text ("62 1F 01", "621F01") as bytes.
pub fn parse_hex(s: &str) -> Result<Vec<u8>, String> {
    let h: String = s.chars().filter(|c| !c.is_whitespace()).collect();
    if !h.len().is_multiple_of(2) || !h.chars().all(|c| c.is_ascii_hexdigit()) {
        return Err(format!("{s:?} is not hex bytes"));
    }
    Ok((0..h.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&h[i..i + 2], 16).unwrap())
        .collect())
}

/// Refuses a file whose `format` is missing or not this Icarus's.
fn check_format(text: &str, what: &str) -> Result<(), String> {
    let t: toml::Table = toml::from_str(text).map_err(|e| format!("{what}: {e}"))?;
    match t.get("format").and_then(|f| f.as_integer()) {
        Some(1) => Ok(()),
        Some(n) if n > 1 => Err(format!(
            "{what}: format {n} was made by a newer Icarus (this one reads format {FORMAT})"
        )),
        Some(n) => Err(format!("{what}: format {n} is not one Icarus knows")),
        None => Err(format!("{what}: no format number (format = {FORMAT})")),
    }
}

fn check_years(y: &[[u16; 2]], what: &str) -> Result<(), String> {
    for [a, b] in y {
        if a > b {
            return Err(format!("{what}: the year range [{a}, {b}] runs backwards"));
        }
    }
    Ok(())
}

/// `set.toml`, checked.
pub fn parse_set(text: &str) -> Result<SetInfo, String> {
    check_format(text, "set.toml")?;
    let s: SetInfo = toml::from_str(text).map_err(|e| format!("set.toml: {e}"))?;
    check_set(&s)?;
    Ok(s)
}

/// A set's name: lowercase letters, digits and dashes (it is a folder name, so nothing that could leave the folder).
pub fn check_set_name(name: &str) -> Result<(), String> {
    if name.is_empty()
        || !name
            .chars()
            .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-')
    {
        return Err(format!(
            "set name {name:?}: lowercase letters, digits and dashes only"
        ));
    }
    Ok(())
}

/// A set's information checked: its name (`check_set_name`) and its years.
pub fn check_set(s: &SetInfo) -> Result<(), String> {
    check_set_name(&s.name)?;
    check_years(&s.years, "set.toml")
}

/// One module file, checked (`validate`), with `min_len` filled in where it was left out.
pub fn parse_module(text: &str, what: &str) -> Result<ModuleDef, String> {
    check_format(text, what)?;
    let mut m: ModuleDef = toml::from_str(text).map_err(|e| format!("{what}: {e}"))?;
    validate(&mut m).map_err(|e| format!("{what}: {e}"))?;
    Ok(m)
}

/// The rules every module definition follows, whoever wrote it. Fills in `min_len` where it is left out.
pub fn validate(m: &mut ModuleDef) -> Result<(), String> {
    if m.format != FORMAT {
        return Err(format!("format {} (this Icarus reads {FORMAT})", m.format));
    }
    let h = &m.module;
    let top = id_top(h.ids);
    if h.tx > top {
        return Err(format!(
            "{}: {:X} is not a {} ID",
            h.name,
            h.tx,
            h.ids.word()
        ));
    }
    if h.rx() > top {
        return Err(format!(
            "{}: {:X}/{:X} are not {} IDs",
            h.name,
            h.tx,
            h.rx(),
            h.ids.word()
        ));
    }
    let mut ids = BTreeSet::new();
    for r in m.requests.iter_mut() {
        let what = format!("request {}", r.label());
        if !SERVICES.contains(&r.service) {
            return Err(format!(
                "{what}: service {:02X} is not one Icarus reads with (21, 22, 1A)",
                r.service
            ));
        }
        if r.service != 0x22 && r.ident > 0xFF {
            return Err(format!(
                "{what}: service {:02X} takes a one-byte identifier",
                r.service
            ));
        }
        if r.every.is_some_and(|e| !(e.is_finite() && e > 0.0)) {
            return Err(format!(
                "{what}: every must be a positive number of seconds"
            ));
        }
        check_years(&r.years, &what)?;
        check_years(&r.unverified, &what)?;
        r.samples_data().map_err(|e| format!("{what}: {e}"))?;
        let mut need = 0;
        for v in &r.values {
            check_value(v).map_err(|e| format!("{what}, value {}: {e}", v.id))?;
            if !ids.insert(v.id.clone()) {
                return Err(format!("{what}: value id {} is used twice", v.id));
            }
            need = need.max(v.end_byte());
        }
        match r.min_len {
            Some(n) if n > MAX_BYTES => {
                return Err(format!(
                    "{what}: min_len {n} is more than an answer can hold ({MAX_BYTES} bytes)"
                ))
            }
            Some(n) if n < need => {
                return Err(format!(
                    "{what}: min_len {n} is shorter than its values need ({need} bytes)"
                ))
            }
            Some(_) => {}
            None => r.min_len = Some(need),
        }
    }
    for (i, a) in m.requests.iter().enumerate() {
        for b in &m.requests[i + 1..] {
            if (a.service, a.ident) == (b.service, b.ident) && years_overlap(&a.years, &b.years) {
                return Err(format!(
                    "request {} is listed twice for the same years",
                    a.label()
                ));
            }
        }
    }
    Ok(())
}

/// The largest ID of a kind: 7FF for 11-bit CAN (and K-line's header bytes as Icarus keeps them), 1FFFFFFF for 29-bit.
pub fn id_top(ids: Ids) -> u32 {
    if ids == Ids::Can29 {
        0x1FFF_FFFF
    } else {
        0x7FF
    }
}

/// The rules for one value (`validate` applies them to every value; an importer applies them to each value as it adds it,
/// so one bad value leaves out only itself).
pub fn check_value(v: &ValueDef) -> Result<(), String> {
    if v.id.is_empty() {
        return Err("no id".into());
    }
    if v.bit > 7 {
        return Err(format!("bit {} (0-7 within the byte)", v.bit));
    }
    if v.byte >= MAX_BYTES || v.end_byte() > MAX_BYTES {
        return Err(format!(
            "byte {} with {} bit(s) reaches past the {MAX_BYTES} bytes an answer can hold",
            v.byte, v.bits
        ));
    }
    match v.kind {
        ValueKind::Text | ValueKind::Hex => {
            if v.bits == 0 || !v.bits.is_multiple_of(8) || v.bit != 0 {
                return Err(format!(
                    "{} values start on a byte and are whole bytes",
                    v.kind.word()
                ));
            }
        }
        ValueKind::Number | ValueKind::State => {
            if !(1..=64).contains(&v.bits) {
                return Err(format!("bits {} (1-64)", v.bits));
            }
        }
    }
    for (n, x) in [
        ("mul", v.mul),
        ("bias", v.bias),
        ("div", v.div),
        ("add", v.add),
    ] {
        if !x.is_finite() {
            return Err(format!("{n} is not a number"));
        }
    }
    if v.div == 0.0 {
        return Err("div is 0".into());
    }
    if v.kind == ValueKind::State {
        if v.states.is_empty() {
            return Err("a state value needs states".into());
        }
        if let Some(k) = v.states.keys().find(|k| k.trim().parse::<i128>().is_err()) {
            return Err(format!("state key {k:?} is not a whole number"));
        }
        let (lo, hi) = state_range(v);
        if let Some(k) = v.states.keys().find(|k| {
            k.trim()
                .parse::<i128>()
                .is_ok_and(|n| !(lo..=hi).contains(&n))
        }) {
            return Err(format!(
                "state key {k:?} can never be read: {} {} bit(s) hold {lo} to {hi}",
                if v.signed { "signed" } else { "unsigned" },
                v.bits
            ));
        }
    }
    Ok(())
}

/// The raw values a state value's bits can hold (signed or not), for checking its keys.
pub fn state_range(v: &ValueDef) -> (i128, i128) {
    let bits = u32::from(v.bits.clamp(1, 64));
    if v.signed {
        (-(1i128 << (bits - 1)), (1i128 << (bits - 1)) - 1)
    } else {
        (0, (1i128 << bits) - 1)
    }
}

/// How a string is written in TOML.
fn q(s: &str) -> String {
    toml::Value::String(s.to_string()).to_string()
}

fn f(x: f64) -> String {
    format!("{x:?}")
}

fn years(y: &[[u16; 2]]) -> String {
    format!(
        "[{}]",
        y.iter()
            .map(|[a, b]| format!("[{a}, {b}]"))
            .collect::<Vec<_>>()
            .join(", ")
    )
}

/// `set.toml` text.
pub fn set_to_toml(s: &SetInfo) -> String {
    let mut o = format!("format = {FORMAT}\nname = {}\n", q(&s.name));
    if !s.make.is_empty() {
        o += &format!("make = {}\n", q(&s.make));
    }
    if !s.models.is_empty() {
        o += &format!(
            "models = [{}]\n",
            s.models.iter().map(|m| q(m)).collect::<Vec<_>>().join(", ")
        );
    }
    if !s.years.is_empty() {
        o += &format!("years = {}\n", years(&s.years));
    }
    let src = &s.source;
    o += &format!("\n[source]\nfrom = {}\n", q(&src.from));
    for (k, v) in [
        ("file", &src.file),
        ("sha256", &src.sha256),
        ("imported", &src.imported),
        ("license", &src.license),
        ("attribution", &src.attribution),
    ] {
        if !v.is_empty() {
            o += &format!("{k} = {}\n", q(v));
        }
    }
    o
}

/// A module file's text, with IDs in hex.
pub fn module_to_toml(m: &ModuleDef) -> String {
    let h = &m.module;
    let w = if h.ids == Ids::Can29 { 8 } else { 3 };
    let mut o = format!(
        "format = {FORMAT}\n\n[module]\nname = {}\ntx = 0x{:0w$X}\n",
        q(&h.name),
        h.tx
    );
    if let Some(rx) = h.rx {
        o += &format!("rx = 0x{rx:0w$X}\n");
    }
    if let Some(e) = h.ext {
        o += &format!("ext = 0x{e:02X}\n");
    }
    if let Some(t) = h.tester {
        o += &format!("tester = 0x{t:02X}\n");
    }
    if h.ids != Ids::Can11 {
        o += &format!("ids = {}\n", q(h.ids.word()));
    }
    if let Some(b) = &h.bus {
        o += &format!("bus = {}\n", q(b));
    }
    if let Some(d) = h.dialect {
        o += &format!(
            "dialect = {}\n",
            q(if d == Dialect::Uds { "uds" } else { "kwp" })
        );
    }
    if !h.kind.is_empty() {
        o += &format!("kind = {}\n", q(&h.kind));
    }
    if !h.identity.is_empty() {
        o += "identity = [\n";
        for i in &h.identity {
            let parts: Vec<String> = [
                ("diag", &i.diag),
                ("supplier", &i.supplier),
                ("soft", &i.soft),
                ("version", &i.version),
            ]
            .iter()
            .filter_map(|(k, v)| v.as_ref().map(|v| format!("{k} = {}", q(v))))
            .collect();
            o += &format!("  {{ {} }},\n", parts.join(", "));
        }
        o += "]\n";
    }
    for r in &m.requests {
        o += &format!(
            "\n[[request]]\nservice = 0x{:02X}\nident = 0x{:0iw$X}\n",
            r.service,
            r.ident,
            iw = if r.service == 0x22 { 4 } else { 2 }
        );
        if let Some(e) = r.every {
            o += &format!("every = {}\n", f(e));
        }
        if let Some(n) = r.min_len {
            o += &format!("min_len = {n}\n");
        }
        if let Some(t) = r.timeout_ms {
            o += &format!("timeout_ms = {t}\n");
        }
        if !r.years.is_empty() {
            o += &format!("years = {}\n", years(&r.years));
        }
        if !r.unverified.is_empty() {
            o += &format!("unverified = {}\n", years(&r.unverified));
        }
        if let Some(s) = &r.needs_session {
            o += &format!("needs_session = {}\n", q(s));
        }
        if let Some(s) = &r.sample {
            o += &format!("sample = {}\n", q(s));
        }
        if !r.samples.is_empty() {
            o += &format!(
                "samples = [{}]\n",
                r.samples
                    .iter()
                    .map(|s| q(s))
                    .collect::<Vec<_>>()
                    .join(", ")
            );
        }
        for v in &r.values {
            o += &value_to_toml(v);
        }
    }
    o
}

fn value_to_toml(v: &ValueDef) -> String {
    let mut o = format!(
        "\n  [[request.value]]\n  id = {}\n  name = {}\n",
        q(&v.id),
        q(&v.name)
    );
    let mut line = |k: &str, val: String| o += &format!("  {k} = {val}\n");
    if !v.path.is_empty() {
        line("path", q(&v.path));
    }
    if !v.about.is_empty() {
        line("about", q(&v.about));
    }
    line("byte", v.byte.to_string());
    if v.bit != 0 {
        line("bit", v.bit.to_string());
    }
    line("bits", v.bits.to_string());
    if v.order != Order::Msb {
        line("order", q(v.order.word()));
    }
    if v.signed {
        line("signed", "true".into());
    }
    if v.mul != 1.0 {
        line("mul", f(v.mul));
    }
    if v.bias != 0.0 {
        line("bias", f(v.bias));
    }
    if v.div != 1.0 {
        line("div", f(v.div));
    }
    if v.add != 0.0 {
        line("add", f(v.add));
    }
    if let Some(d) = v.decimals {
        line("decimals", d.to_string());
    }
    if !v.unit.is_empty() {
        line("unit", q(&v.unit));
    }
    for (k, x) in [
        ("min", v.min),
        ("max", v.max),
        ("null_below", v.null_below),
        ("null_above", v.null_above),
    ] {
        if let Some(x) = x {
            line(k, f(x));
        }
    }
    if v.kind != ValueKind::Number {
        line("kind", q(v.kind.word()));
    }
    if !v.states.is_empty() {
        let mut keys: Vec<&String> = v.states.keys().collect();
        keys.sort_by_key(|k| k.trim().parse::<i128>().unwrap_or(0));
        line(
            "states",
            format!(
                "{{ {} }}",
                keys.iter()
                    .map(|k| format!("{} = {}", q(k), q(&v.states[*k])))
                    .collect::<Vec<_>>()
                    .join(", ")
            ),
        );
    }
    if v.checked {
        line("checked", "true".into());
    }
    if v.hidden {
        line("hidden", "true".into());
    }
    if !v.note.is_empty() {
        line("note", q(&v.note));
    }
    o
}

/// A module's file name in its set's folder: "701.toml", "750.2A.toml".
pub fn file_name(h: &ModuleHead) -> String {
    format!("{}.toml", h.label())
}

/// Every file of a set, as (file name, text): `set.toml` first, then one per module, each name its own ("701.toml",
/// then "701_709.toml" for a second module on that request ID, "701_709_2.toml" for a third answering from the same ID).
pub fn set_files(set: &SetInfo, modules: &[ModuleDef]) -> Vec<(String, String)> {
    let mut out = vec![("set.toml".to_string(), set_to_toml(set))];
    let mut seen = BTreeSet::new();
    for m in modules {
        let mut name = file_name(&m.module);
        // the same request ID answering from another ID (or the same one, with another tester address): its own file
        let base = format!("{}_{:X}", m.module.label(), m.module.rx());
        let mut n = 1;
        while seen.contains(&name) {
            name = if n == 1 {
                format!("{base}.toml")
            } else {
                format!("{base}_{n}.toml")
            };
            n += 1;
        }
        seen.insert(name.clone());
        out.push((name, module_to_toml(m)));
    }
    out
}

/// Units as shown: where US customary units are the custom, metric units are converted (km/h to mph, °C to °F...).
pub mod units {
    /// A value in `unit` as shown: converted when `us` (mph, mi, °F, psi, gal/h, lb-ft), else as given.
    pub fn shown(unit: &str, v: f64, us: bool) -> (f64, &str) {
        if us {
            match unit {
                "km/h" => return (v * 0.621371, "mph"),
                "km" => return (v * 0.621371, "mi"),
                "°C" => return (v * 1.8 + 32.0, "°F"),
                "kPa" => return (v * 0.145038, "psi"),
                "bar" => return (v * 14.5038, "psi"),
                "L/h" => return (v * 0.264172, "gal/h"),
                "Nm" => return (v * 0.737562, "lb-ft"),
                _ => {}
            }
        }
        (v, unit)
    }

    /// Whether miles per hour, °F and psi are the custom here: `ICARUS_UNITS` (`us` or `metric`) when set, else the
    /// locale's measurement (en_US and the few like it).
    pub fn us_customary() -> bool {
        static US: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
        *US.get_or_init(|| {
            match std::env::var("ICARUS_UNITS").ok().as_deref() {
                Some("us") => return true,
                Some("metric") => return false,
                _ => {}
            }
            let loc = ["LC_ALL", "LC_MEASUREMENT", "LANG"]
                .iter()
                .filter_map(|k| std::env::var(k).ok())
                .find(|v| !v.is_empty())
                .unwrap_or_default();
            ["en_US", "es_US", "en_LR", "my_MM"]
                .iter()
                .any(|p| loc.starts_with(p))
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A module file written for this test (made-up identifiers, as everywhere in Icarus's tests).
    pub(crate) const MODULE: &str = r#"
format = 1

[module]
name = "transmission (UDS)"
tx = 0x701
kind = "transmission"
identity = [{ diag = "0C", supplier = "ICR" }]

[[request]]
service = 0x22
ident = 0x1F11
every = 1.0
years = [[2016, 2030]]
sample = "62 1F 11 6E 00 C8 01"

  [[request.value]]
  id = "ATF_TEMP"
  name = "transmission fluid temperature"
  path = "Transmission.Temperatures"
  byte = 0
  bits = 8
  add = -40.0
  unit = "°C"

  [[request.value]]
  id = "OUT_RPM"
  name = "output shaft speed"
  byte = 1
  bits = 16
  unit = "rpm"

  [[request.value]]
  id = "LOCKUP"
  name = "lock-up"
  byte = 3
  bit = 7
  bits = 1
  kind = "state"
  states = { "0" = "off", "1" = "on" }

[[request]]
service = 0x21
ident = 0x04

  [[request.value]]
  id = "GEAR"
  name = "gear"
  byte = 0
  bits = 8
"#;

    #[test]
    fn a_module_file_reads_and_writes_back_the_same() {
        let m = parse_module(MODULE, "701.toml").unwrap();
        assert_eq!((m.module.tx, m.module.rx()), (0x701, 0x709));
        assert_eq!(m.module.label(), "701");
        assert_eq!(m.requests.len(), 2);
        let r = &m.requests[0];
        assert_eq!(r.bytes(), vec![0x22, 0x1F, 0x11]);
        assert_eq!(r.min_len, Some(4), "filled in from the values");
        assert_eq!(m.requests[1].bytes(), vec![0x21, 0x04]);
        assert_eq!(r.sample_data().unwrap(), Some(vec![0x6E, 0x00, 0xC8, 0x01]));
        let mut more = m.clone();
        more.requests[0].samples = vec!["62 1F 11 6F 00 C9 00".into()];
        assert_eq!(
            more.requests[0].samples_data().unwrap(),
            vec![vec![0x6E, 0x00, 0xC8, 0x01], vec![0x6F, 0x00, 0xC9, 0x00]]
        );
        assert_eq!(parse_module(&module_to_toml(&more), "again").unwrap(), more);
        let text = module_to_toml(&m);
        assert!(
            text.contains("tx = 0x701") && text.contains("ident = 0x1F11"),
            "{text}"
        );
        assert_eq!(parse_module(&text, "again").unwrap(), m);
    }

    #[test]
    fn the_rules_refuse_bad_files() {
        let bad = |from: &str, to: &str| {
            let t = MODULE.replacen(from, to, 1);
            assert_ne!(t, MODULE, "{from}");
            parse_module(&t, "x").unwrap_err()
        };
        assert!(bad("format = 1", "format = 2").contains("newer Icarus"));
        assert!(bad("service = 0x21", "service = 0x2E").contains("not one Icarus reads with"));
        assert!(bad("service = 0x21", "service = 0x31").contains("21, 22, 1A"));
        assert!(bad("ident = 0x04", "ident = 0x104").contains("one-byte"));
        assert!(bad("bits = 16", "bits = 65").contains("1-64"));
        assert!(bad("unit = \"rpm\"", "unit = \"rpm\"\n  div = 0.0").contains("div is 0"));
        assert!(bad("kind = \"state\"", "kind = \"text\"").contains("whole bytes"));
        assert!(bad(
            "states = { \"0\" = \"off\", \"1\" = \"on\" }",
            "states = { \"x\" = \"off\" }"
        )
        .contains("whole number"));
        assert!(bad("id = \"OUT_RPM\"", "id = \"ATF_TEMP\"").contains("used twice"));
        assert!(bad("years = [[2016, 2030]]", "years = [[2030, 2016]]").contains("backwards"));
        assert!(bad("every = 1.0", "every = 1.0\nmin_len = 2").contains("shorter"));
        assert!(bad("every = 1.0", "every = 1.0\nsent = \"2E 1F 11 00\"").contains("unknown field"));
        assert!(bad("sample = \"62 1F 11", "sample = \"62 1F 12").contains("not a positive answer"));
        assert!(bad(
            "sample = \"62 1F 11 6E 00 C8 01\"",
            "sample = \"62 1F 11 6E 00 C8 01\"\nsamples = [\"62 1F 11 6F\", \"7F 22 31\"]"
        )
        .contains("not a positive answer"));
        assert!(bad("tx = 0x701", "tx = 0x901").contains("not a can11 ID"));
        assert!(bad("tx = 0x701", "tx = 0x701\nrx = 0x801").contains("not can11 IDs"));
        // the same request twice is fine only for different years
        let twice = MODULE.replace("ident = 0x04", "ident = 0x04\nyears = [[2016, 2017]]")
            + "\n[[request]]\nservice = 0x21\nident = 0x04\nyears = [[2018, 2020]]\n";
        assert!(parse_module(&twice, "x").is_ok());
        let clash = twice.replace("[[2018, 2020]]", "[[2017, 2020]]");
        assert!(parse_module(&clash, "x")
            .unwrap_err()
            .contains("twice for the same years"));
    }

    #[test]
    fn malformed_numbers_are_refused_not_a_panic_or_a_wrong_value() {
        let bad = |from: &str, to: &str| {
            let t = MODULE.replacen(from, to, 1);
            assert_ne!(t, MODULE, "{from}");
            parse_module(&t, "x").unwrap_err()
        };
        // the largest request ID there is, with no answer ID given: tx + 8 would overflow
        assert!(bad("tx = 0x701", "tx = 0xFFFFFFFF").contains("not a can11 ID"));
        assert!(bad("tx = 0x701", "tx = 0xFFFFFFFF\nids = \"can29\"").contains("not a can29 ID"));
        let h = ModuleHead {
            tx: u32::MAX,
            ..parse_module(MODULE, "x").unwrap().module
        };
        assert_eq!(h.rx(), u32::MAX, "saturates");
        // a byte far past any answer: refused (in a release build it used to wrap and read as 0)
        for byte in [
            "2305843009213693951",
            "9223372036854775807",
            "4095",
            "100000",
        ] {
            let e = bad("byte = 1", &format!("byte = {byte}"));
            assert!(e.contains("past the 4095 bytes"), "{byte}: {e}");
        }
        assert!(
            bad("byte = 1", "byte = 4094").contains("past the 4095"),
            "16 bits from 4094 end at 4096"
        );
        assert!(
            parse_module(&MODULE.replacen("byte = 1", "byte = 4093", 1), "x").is_ok(),
            "16 bits from 4093 end at 4095"
        );
        let far = ValueDef::new("X", "x", usize::MAX, 64);
        assert_eq!(far.end_byte(), usize::MAX / 8 + 1, "saturates, no overflow");
        assert!(check_value(&far).is_err());
        assert!(bad("every = 1.0", "every = 1.0\nmin_len = 5000").contains("more than an answer"));
        // state keys the bits can never hold
        assert!(bad("\"1\" = \"on\"", "\"2\" = \"on\"").contains("can never be read"));
        assert!(
            bad("\"0\" = \"off\"", "\"-1\" = \"off\"").contains("unsigned 1 bit(s) hold 0 to 1")
        );
        let mut v = ValueDef::new("S", "s", 0, 8);
        v.kind = ValueKind::State;
        v.signed = true;
        v.states = BTreeMap::from([("-128".into(), "low".into()), ("127".into(), "high".into())]);
        assert!(check_value(&v).is_ok());
        v.states.insert("128".into(), "never".into());
        assert!(check_value(&v)
            .unwrap_err()
            .contains("signed 8 bit(s) hold -128 to 127"));
        v.bits = 64;
        v.signed = false;
        v.states = BTreeMap::from([("18446744073709551615".into(), "all set".into())]);
        assert!(check_value(&v).is_ok(), "u64::MAX fits 64 bits");
    }

    #[test]
    fn a_set_reads_and_writes_back() {
        let s = SetInfo {
            format: 1,
            name: "toy-truck".into(),
            make: "toyota".into(),
            models: vec!["Truck".into()],
            years: vec![[2016, 2023]],
            source: Source {
                from: "hand".into(),
                attribution: "made for a test".into(),
                ..Default::default()
            },
        };
        assert_eq!(parse_set(&set_to_toml(&s)).unwrap(), s);
        let bad = set_to_toml(&s).replace("toy-truck", "Toy Truck");
        assert!(parse_set(&bad).unwrap_err().contains("lowercase"));
        for name in ["../x", "a/b", "", ".", "A"] {
            assert!(check_set_name(name).is_err(), "{name:?}");
        }
        let m = parse_module(MODULE, "x").unwrap();
        let files = set_files(&s, &[m.clone(), m.clone()]);
        let names: Vec<&str> = files.iter().map(|f| f.0.as_str()).collect();
        assert_eq!(names, vec!["set.toml", "701.toml", "701_709.toml"]);
        // a third module with the same label and answer ID (split by its tester address) does not overwrite the second
        let files = set_files(&s, &[m.clone(), m.clone(), m.clone(), m]);
        let names: Vec<&str> = files.iter().map(|f| f.0.as_str()).collect();
        assert_eq!(
            names,
            vec![
                "set.toml",
                "701.toml",
                "701_709.toml",
                "701_709_2.toml",
                "701_709_3.toml"
            ]
        );
    }

    #[test]
    fn years_match_and_overlap() {
        assert!(year_in(&[], 1990));
        assert!(year_in(&[[2016, 2023]], 2017) && !year_in(&[[2016, 2023]], 2024));
        assert!(years_overlap(&[[2016, 2018]], &[[2018, 2020]]));
        assert!(!years_overlap(&[[2016, 2017]], &[[2018, 2020]]));
        assert!(years_overlap(&[], &[[2018, 2020]]));
    }

    #[test]
    fn us_units_convert() {
        assert_eq!(units::shown("°C", 100.0, true), (212.0, "°F"));
        assert_eq!(units::shown("°C", 100.0, false), (100.0, "°C"));
        let (mph, u) = units::shown("km/h", 100.0, true);
        assert!((mph - 62.1371).abs() < 1e-9 && u == "mph");
        assert_eq!(units::shown("rpm", 700.0, true), (700.0, "rpm"));
    }
}
