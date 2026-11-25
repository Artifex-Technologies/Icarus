//! CAN databases (`.dbc`, ICA-6, module data build 4): what the frames on a bus mean. A DBC names each message ID and the
//! signals packed in its bytes (start bit, length, byte order, sign, scale and offset, unit, and named values such as
//! "open"/"closed"), so the frames Icarus watches go by name and value instead of hex. Written from the format's public
//! descriptions, not from any program's source; the files themselves are the user's (a vehicle's DBC from opendbc or a
//! maker), never part of this repository.
//!
//! Read: `BO_` messages (11-bit IDs, 29-bit ones with bit 31 set as DBC writes them), their `SG_` signals in Intel (`@1`,
//! least significant byte first) or Motorola (`@0`, most significant first; the start bit is the most significant bit in
//! DBC's numbering) order, signed or unsigned, a multiplexor (`M`) and multiplexed signals (`m3`: only when the multiplexor
//! reads 3), and `VAL_` tables. Left out with the reason: floating-point signals (`SIG_VALTYPE_`) and lines that do not
//! read. Comments, attributes and nodes are not needed to decode and are passed over.

use std::collections::BTreeMap;

#[derive(Clone, Debug, PartialEq)]
pub struct Signal {
    pub name: String,
    /// DBC's start bit: the least significant bit (Intel), or the most significant (Motorola).
    pub start: u32,
    pub len: u32,
    pub intel: bool,
    pub signed: bool,
    pub factor: f64,
    pub offset: f64,
    pub min: f64,
    pub max: f64,
    pub unit: String,
    /// `M`: this signal says which multiplexed signals are in the frame.
    pub multiplexor: bool,
    /// `m<n>`: present only when the multiplexor reads n.
    pub multiplexed: Option<u64>,
    /// Named raw values (VAL_).
    pub values: BTreeMap<i64, String>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Message {
    pub id: u32,
    pub ext: bool,
    pub name: String,
    pub dlc: u32,
    pub sender: String,
    pub signals: Vec<Signal>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Dbc {
    pub messages: Vec<Message>,
    /// What was left out, each with the reason ("line 12: ...", "Engine.Torque: ...").
    pub skipped: Vec<String>,
}

/// One signal's value in one frame.
#[derive(Clone, Debug, PartialEq)]
pub struct Decoded {
    pub signal: String,
    pub raw: i64,
    pub value: f64,
    pub unit: String,
    /// The VAL_ table's name for the raw value, when there is one.
    pub label: Option<String>,
}

/// DBC writes a 29-bit ID with bit 31 set.
const EXT_FLAG: u32 = 0x8000_0000;

fn num(s: &str) -> Option<f64> {
    s.trim().parse().ok()
}

/// One `SG_` line (without the `SG_`), or why it does not read.
fn signal(line: &str) -> Result<Signal, String> {
    let (head, rest) = line.split_once(':').ok_or("no ':'")?;
    let mut h = head.split_whitespace();
    let name = h.next().ok_or("no name")?.to_string();
    let (mut multiplexor, mut multiplexed) = (false, None);
    if let Some(m) = h.next() {
        if m == "M" {
            multiplexor = true;
        } else if let Some(n) = m.strip_prefix('m') {
            // m3, or m3M (a multiplexed multiplexor, extended multiplexing): read as multiplexed by 3
            let digits: String = n.chars().take_while(|c| c.is_ascii_digit()).collect();
            multiplexed = Some(
                digits
                    .parse()
                    .map_err(|_| format!("multiplexing {m} does not read"))?,
            );
        } else {
            return Err(format!("{m} before ':' does not read"));
        }
    }
    let rest = rest.trim();
    // start|len@order sign (factor,offset) [min|max] "unit" receivers
    let (layout, rest) = rest.split_once(char::is_whitespace).ok_or("no layout")?;
    let (bits, order) = layout.split_once('@').ok_or("no '@' in the layout")?;
    let (start, len) = bits.split_once('|').ok_or("no '|' in the layout")?;
    let start: u32 = start.parse().map_err(|_| "the start bit is not a number")?;
    let len: u32 = len.parse().map_err(|_| "the length is not a number")?;
    if len == 0 || len > 64 {
        return Err(format!("a length of {len} bits"));
    }
    let intel = match order.chars().next() {
        Some('1') => true,
        Some('0') => false,
        _ => return Err("the byte order is not 0 or 1".into()),
    };
    let signed = match order.chars().nth(1) {
        Some('-') => true,
        Some('+') => false,
        _ => return Err("no + or - after the byte order".into()),
    };
    let rest = rest.trim();
    let (fo, rest) = rest
        .strip_prefix('(')
        .and_then(|r| r.split_once(')'))
        .ok_or("no (factor,offset)")?;
    let (f, o) = fo.split_once(',').ok_or("no ',' in (factor,offset)")?;
    let (factor, offset) = (
        num(f).ok_or("the factor is not a number")?,
        num(o).ok_or("the offset is not a number")?,
    );
    let rest = rest.trim();
    let (mm, rest) = rest
        .strip_prefix('[')
        .and_then(|r| r.split_once(']'))
        .ok_or("no [min|max]")?;
    let (min, max) = mm.split_once('|').ok_or("no '|' in [min|max]")?;
    let unit = rest
        .trim()
        .strip_prefix('"')
        .and_then(|r| r.split_once('"'))
        .map(|(u, _)| u.to_string())
        .unwrap_or_default();
    Ok(Signal {
        name,
        start,
        len,
        intel,
        signed,
        factor,
        offset,
        min: num(min).unwrap_or(f64::NAN),
        max: num(max).unwrap_or(f64::NAN),
        unit,
        multiplexor,
        multiplexed,
        values: BTreeMap::new(),
    })
}

/// A `VAL_ id signal n "text" ... ;` line's pairs.
fn value_table(rest: &str) -> Option<(u32, String, BTreeMap<i64, String>)> {
    let mut it = rest.trim().splitn(3, char::is_whitespace);
    let id: u32 = it.next()?.parse().ok()?;
    let sig = it.next()?.to_string();
    let mut body = it.next()?.trim().trim_end_matches(';').trim();
    let mut out = BTreeMap::new();
    while !body.is_empty() {
        let (n, r) = body.split_once(char::is_whitespace)?;
        let r = r.trim_start().strip_prefix('"')?;
        let (text, r) = r.split_once('"')?;
        out.insert(n.parse().ok()?, text.to_string());
        body = r.trim();
    }
    Some((id, sig, out))
}

/// Reads a DBC; what does not read is listed in `skipped`, never guessed.
pub fn parse(text: &str) -> Dbc {
    let mut d = Dbc::default();
    let mut tables: Vec<(u32, String, BTreeMap<i64, String>)> = Vec::new();
    let mut floats: Vec<(u32, String)> = Vec::new();
    for (i, raw) in text.lines().enumerate() {
        let line = raw.trim();
        if let Some(rest) = line.strip_prefix("BO_ ") {
            let mut it = rest.split_whitespace();
            let (Some(id), Some(name), Some(dlc)) = (it.next(), it.next(), it.next()) else {
                d.skipped
                    .push(format!("line {}: a BO_ that does not read", i + 1));
                continue;
            };
            let Ok(id) = id.parse::<u32>() else {
                d.skipped.push(format!(
                    "line {}: the message ID {id} is not a number",
                    i + 1
                ));
                continue;
            };
            let ext = id & EXT_FLAG != 0;
            d.messages.push(Message {
                id: if ext { id & 0x1FFF_FFFF } else { id },
                ext,
                name: name.trim_end_matches(':').to_string(),
                dlc: dlc.parse().unwrap_or(8),
                sender: it.next().unwrap_or("").to_string(),
                signals: Vec::new(),
            });
        } else if let Some(rest) = line.strip_prefix("SG_ ") {
            let Some(m) = d.messages.last_mut() else {
                d.skipped
                    .push(format!("line {}: a signal outside any message", i + 1));
                continue;
            };
            match signal(rest) {
                Ok(s) => m.signals.push(s),
                Err(why) => d.skipped.push(format!("line {}: {}: {why}", i + 1, m.name)),
            }
        } else if let Some(rest) = line.strip_prefix("VAL_ ") {
            match value_table(rest) {
                Some(t) => tables.push(t),
                None => d
                    .skipped
                    .push(format!("line {}: a VAL_ table that does not read", i + 1)),
            }
        } else if let Some(rest) = line.strip_prefix("SIG_VALTYPE_ ") {
            let mut it = rest.split_whitespace();
            if let (Some(id), Some(sig)) =
                (it.next().and_then(|v| v.parse::<u32>().ok()), it.next())
            {
                floats.push((id, sig.to_string()));
            }
        }
    }
    let norm = |id: u32| {
        if id & EXT_FLAG != 0 {
            id & 0x1FFF_FFFF
        } else {
            id
        }
    };
    for (id, sig, values) in tables {
        if let Some(s) = d
            .messages
            .iter_mut()
            .filter(|m| m.id == norm(id))
            .flat_map(|m| m.signals.iter_mut())
            .find(|s| s.name == sig)
        {
            s.values = values;
        }
    }
    for (id, sig) in floats {
        for m in d.messages.iter_mut().filter(|m| m.id == norm(id)) {
            if m.signals.iter().any(|s| s.name == sig) {
                m.signals.retain(|s| s.name != sig);
                d.skipped.push(format!(
                    "{}.{sig}: a floating-point signal (SIG_VALTYPE_), not read",
                    m.name
                ));
            }
        }
    }
    d
}

/// The raw bits of a signal from a frame's data, or None when they run past it.
fn raw_bits(s: &Signal, data: &[u8]) -> Option<u64> {
    let bit = |pos: u32| -> Option<u64> {
        let byte = *data.get((pos / 8) as usize)?;
        Some(u64::from((byte >> (pos % 8)) & 1))
    };
    let mut v = 0u64;
    if s.intel {
        for k in 0..s.len {
            v |= bit(s.start + k)? << k;
        }
    } else {
        // Motorola: from the most significant bit at `start`, down through each byte, then to the next byte's top bit
        let mut pos = s.start;
        for _ in 0..s.len {
            v = (v << 1) | bit(pos)?;
            if pos.is_multiple_of(8) {
                pos += 15;
            } else {
                pos -= 1;
            }
        }
    }
    Some(v)
}

impl Signal {
    /// This signal's value in `data`: raw, scaled, and its name in the value table.
    pub fn decode(&self, data: &[u8]) -> Option<Decoded> {
        let bits = raw_bits(self, data)?;
        let raw = if self.signed && self.len < 64 && bits >> (self.len - 1) & 1 == 1 {
            (bits | !0u64 << self.len) as i64
        } else {
            bits as i64
        };
        Some(Decoded {
            signal: self.name.clone(),
            raw,
            value: raw as f64 * self.factor + self.offset,
            unit: self.unit.clone(),
            label: self.values.get(&raw).cloned(),
        })
    }
}

impl Decoded {
    /// "EngineSpeed 800 rpm", or with a named value "DriverDoor open": whole numbers without decimals, others to three
    /// places at most.
    pub fn text(&self) -> String {
        if let Some(l) = &self.label {
            return format!("{} {l}", self.signal);
        }
        let v = self.value;
        let n = if v.fract() == 0.0 && v.abs() < 1e15 {
            format!("{v:.0}")
        } else {
            let s = format!("{v:.3}");
            s.trim_end_matches('0').trim_end_matches('.').to_string()
        };
        if self.unit.is_empty() {
            format!("{} {n}", self.signal)
        } else {
            format!("{} {n} {}", self.signal, self.unit)
        }
    }
}

impl Dbc {
    /// "Engine: EngineSpeed 800 rpm, Coolant 90 degC" for a frame, None for an ID the DBC does not name.
    pub fn line(&self, id: u32, ext: bool, data: &[u8]) -> Option<String> {
        let (m, values) = self.decode(id, ext, data)?;
        Some(format!(
            "{}: {}",
            m.name,
            values
                .iter()
                .map(Decoded::text)
                .collect::<Vec<_>>()
                .join(", ")
        ))
    }

    /// The message with this ID.
    pub fn message(&self, id: u32, ext: bool) -> Option<&Message> {
        self.messages.iter().find(|m| m.id == id && m.ext == ext)
    }

    /// Every signal of a frame that the DBC names, in the message's order; multiplexed signals only when the multiplexor
    /// says so. None for an ID the DBC does not name.
    pub fn decode(&self, id: u32, ext: bool, data: &[u8]) -> Option<(&Message, Vec<Decoded>)> {
        let m = self.message(id, ext)?;
        let mux = m
            .signals
            .iter()
            .find(|s| s.multiplexor)
            .and_then(|s| s.decode(data))
            .map(|d| d.raw);
        let out = m
            .signals
            .iter()
            .filter(|s| match s.multiplexed {
                Some(n) => mux == Some(n as i64),
                None => true,
            })
            .filter_map(|s| s.decode(data))
            .collect();
        Some((m, out))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Written for this test: both byte orders, a signed signal, a value table, a multiplexed pair, a 29-bit ID, a float
    /// and a broken line.
    const DBC: &str = r#"VERSION ""
BU_: ECM BCM ABS
BO_ 708 Engine: 8 ECM
 SG_ EngineSpeed : 7|16@0+ (1,0) [0|8000] "rpm" Vector__XXX
 SG_ Coolant : 31|8@0- (1,0) [-40|215] "degC" Vector__XXX
BO_ 180 Wheels: 8 ABS
 SG_ Counter : 56|8@1+ (1,0) [0|255] "" Vector__XXX
 SG_ FrontLeft : 0|16@1+ (0.01,0) [0|300] "km/h" Vector__XXX
BO_ 1552 Body: 4 BCM
 SG_ DriverDoor : 19|1@1+ (1,0) [0|1] "" Vector__XXX
 SG_ Page M : 0|8@1+ (1,0) [0|255] "" Vector__XXX
 SG_ Fuel m1 : 8|8@1+ (0.5,0) [0|100] "%" Vector__XXX
 SG_ Odometer m2 : 8|16@1+ (1,0) [0|65535] "km" Vector__XXX
BO_ 2566869221 Charger: 8 BMS
 SG_ Volts : 0|16@1+ (0.1,0) [0|1000] "V" Vector__XXX
 SG_ Ratio : 16|32@1+ (1,0) [0|1] "" Vector__XXX
 SG_ Broken 8|8@1+ (1,0) [0|1] "" Vector__XXX
VAL_ 1552 DriverDoor 0 "closed" 1 "open" ;
SIG_VALTYPE_ 2566869221 Ratio : 1;
"#;

    #[test]
    fn signals_decode_in_both_byte_orders() {
        let d = parse(DBC);
        assert_eq!(d.messages.len(), 4);
        let (m, v) = d
            .decode(0x2C4, false, &[0x03, 0x1C, 0x00, 0xD8, 0, 0, 0, 0])
            .unwrap();
        assert_eq!(m.name, "Engine");
        assert_eq!(
            (v[0].signal.as_str(), v[0].value, v[0].unit.as_str()),
            ("EngineSpeed", 796.0, "rpm")
        );
        assert_eq!(
            (v[1].signal.as_str(), v[1].value),
            ("Coolant", -40.0),
            "0xD8 signed is -40"
        );
        let (_, w) = d
            .decode(0x0B4, false, &[0x10, 0x27, 0, 0, 0, 0, 0, 0x2A])
            .unwrap();
        assert_eq!(
            (w[0].value, w[1].value),
            (42.0, 100.0),
            "the counter in byte 7; 0x2710 * 0.01"
        );
        let door = |b2: u8| d.decode(0x610, false, &[0x00, 0x00, b2, 0x00]).unwrap().1[0].clone();
        assert_eq!(
            (door(0x08).raw, door(0x08).label.as_deref()),
            (1, Some("open"))
        );
        assert_eq!(door(0x00).label.as_deref(), Some("closed"));
        // multiplexed: page 1 carries fuel, page 2 the odometer
        let names = |data: &[u8]| {
            d.decode(0x610, false, data)
                .unwrap()
                .1
                .iter()
                .map(|x| x.signal.clone())
                .collect::<Vec<_>>()
        };
        assert_eq!(names(&[1, 80, 0, 0]), ["DriverDoor", "Page", "Fuel"]);
        assert_eq!(
            names(&[2, 0x10, 0x27, 0]),
            ["DriverDoor", "Page", "Odometer"]
        );
        // a 29-bit ID (bit 31 set in the file), and what was left out
        let (c, cv) = d
            .decode(0x18FF_50E5, true, &[0x10, 0x0F, 0, 0, 0, 0, 0, 0])
            .unwrap();
        assert_eq!(
            (c.name.as_str(), cv.len(), cv[0].value),
            ("Charger", 1, 385.6)
        );
        assert!(
            d.decode(0x18FF_50E5, false, &[0; 8]).is_none(),
            "11-bit and 29-bit IDs are not the same"
        );
        assert!(
            d.skipped
                .iter()
                .any(|s| s.contains("Charger.Ratio") && s.contains("floating-point")),
            "{:?}",
            d.skipped
        );
        assert!(
            d.skipped.iter().any(|s| s.contains("Charger: no ':'")),
            "{:?}",
            d.skipped
        );
        assert_eq!(d.skipped.len(), 2, "{:?}", d.skipped);
        // bits past the frame give no value
        assert!(d.decode(0x0B4, false, &[0x10]).unwrap().1.is_empty());
    }
}
