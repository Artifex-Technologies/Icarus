//! DDT2000 ECU files (`ecus/<name>.xml`, Renault, Dacia, Nissan; module data build 3, ICA-5). DDT2000's databases are
//! Renault's material: Icarus only reads a file the user gives, never ships one or uses one in its tests (tests use tiny
//! XML written for them).
//!
//! What is read, from the file format (ideas and format facts only, as with every reference project):
//! - `Target@Name` names the module; `CAN/SendId/CANId@Value` and `ReceiveId` (decimal) give tx and rx. A file with no CAN
//!   node is a K-line module, which Icarus does not reach yet: nothing is imported from it, and it says so.
//! - A request is kept only when its `SentBytes` are exactly `22 hi lo`, `21 id` or `1A id`, with no `Sent` data items,
//!   no `ManuelSend` and no `DenyAccess` (its sessions are not read, so a request with access rules is left out rather
//!   than guessed); anything else is left out with what it would do.
//! - `Received` data items: `FirstByte` is 1-based over the whole reply, so `byte = FirstByte - 1 - (1 + echo length)` (an
//!   item inside the echo is left out); `BitOffset` is `bit`; `Bytes@count` (x 8) or `Bits@count` is `bits`;
//!   `Bytes@ascii` is text; `Endian` Big is `msb`, Little is `ddt-little` (the request's `Endian` is the default).
//! - `Scaled@Step/Offset/DivideBy` become `mul/bias/div` (the offset is added before the divide) and `Unit` the unit;
//!   `List/Item` become states for unscaled values; an unscaled value with no list shows as hex (or as a plain number when
//!   it is not whole bytes). Signed only at exactly 8 or 16 bits, as DDT tools read them.
//! - `ReplyBytes` becomes the request's `sample`. Screens are not read, so values carry no `path`.
//!
//! Every value is marked unverified for every year: a DDT2000 file is somebody's description, not a check on my vehicle.

use super::{
    check_set, sha256_hex, validate, Ids, Imported, ModuleDef, ModuleHead, Order, RequestDef,
    SetInfo, Source, ValueDef, ValueKind, FORMAT,
};
use roxmltree::{Document, Node};
use std::collections::BTreeMap;

pub const ATTRIBUTION: &str =
    "from a DDT2000 ECU file I gave Icarus (Renault's material, not shipped with Icarus): values not checked";
pub const LICENSE: &str = "Renault's; read from the user's own copy, never shipped";

/// Every year: the requests' `unverified` (a DDT2000 file describes; it does not check).
const EVERY_YEAR: [u16; 2] = [0, 9999];

pub struct Options {
    /// The path the file was read from (its name becomes the set's when no `--name` is given).
    pub path: String,
    /// `--name`: the set's name.
    pub name: Option<String>,
    /// `--make`: the vehicle profile it is for.
    pub make: Option<String>,
    /// Today, for `source.imported` (YYYY-MM-DD).
    pub today: String,
}

/// A `Data` element: how many bits, and how its raw number becomes a value.
struct DataDef {
    bits: u16,
    text: bool,
    signed: bool,
    /// (step, offset, divide by, unit)
    scaled: Option<(f64, f64, f64, String)>,
    states: BTreeMap<String, String>,
}

fn child<'a, 'i>(n: Node<'a, 'i>, name: &str) -> Option<Node<'a, 'i>> {
    n.children()
        .find(|c| c.is_element() && c.tag_name().name() == name)
}

fn named<'a, 'i>(n: Node<'a, 'i>, name: &'static str) -> impl Iterator<Item = Node<'a, 'i>> {
    n.descendants()
        .filter(move |c| c.is_element() && c.tag_name().name() == name)
}

fn num(s: Option<&str>) -> Option<f64> {
    s.and_then(|v| v.trim().replace(',', ".").parse().ok())
}

fn data_def(d: Node) -> Result<DataDef, String> {
    if let Some(b) = child(d, "Bytes") {
        let n: u16 = b
            .attribute("count")
            .and_then(|c| c.trim().parse().ok())
            .ok_or("Bytes without a count")?;
        let text = matches!(
            b.attribute("ascii").map(str::trim),
            Some("1" | "true" | "True")
        );
        return Ok(DataDef {
            bits: n.checked_mul(8).ok_or("too many bytes")?,
            text,
            signed: false,
            scaled: None,
            states: BTreeMap::new(),
        });
    }
    let b = child(d, "Bits").ok_or("neither Bytes nor Bits")?;
    let bits: u16 = b
        .attribute("count")
        .and_then(|c| c.trim().parse().ok())
        .ok_or("Bits without a count")?;
    let signed = matches!(
        b.attribute("signed").map(str::trim),
        Some("1" | "true" | "True")
    ) && matches!(bits, 8 | 16);
    let scaled = child(b, "Scaled").map(|s| {
        (
            num(s.attribute("Step")).unwrap_or(1.0),
            num(s.attribute("Offset")).unwrap_or(0.0),
            num(s.attribute("DivideBy"))
                .filter(|v| *v != 0.0)
                .unwrap_or(1.0),
            s.attribute("Unit").unwrap_or("").trim().to_string(),
        )
    });
    let mut states = BTreeMap::new();
    if let Some(l) = child(b, "List") {
        for it in l
            .children()
            .filter(|c| c.is_element() && c.tag_name().name() == "Item")
        {
            if let (Some(v), Some(t)) = (it.attribute("Value"), it.attribute("Text")) {
                if let Ok(raw) = v.trim().parse::<i64>() {
                    states.insert(raw.to_string(), t.trim().to_string());
                }
            }
        }
    }
    Ok(DataDef {
        bits,
        text: false,
        signed,
        scaled,
        states,
    })
}

/// A value id from a DDT name: letters and digits, upper case, words joined by underscores.
fn value_id(name: &str) -> String {
    let mut out = String::new();
    for c in name.chars() {
        if c.is_ascii_alphanumeric() {
            out.push(c.to_ascii_uppercase());
        } else if !out.ends_with('_') && !out.is_empty() {
            out.push('_');
        }
    }
    let out = out.trim_end_matches('_').to_string();
    if out.is_empty() {
        "VALUE".into()
    } else {
        out
    }
}

/// A set name from a file or target name: lowercase letters, digits and dashes.
fn set_name(s: &str) -> String {
    let mut out = String::new();
    for c in s.chars() {
        if c.is_ascii_alphanumeric() {
            out.push(c.to_ascii_lowercase());
        } else if !out.ends_with('-') && !out.is_empty() {
            out.push('-');
        }
    }
    out.trim_end_matches('-').to_string()
}

/// Reads a DDT2000 ECU file into one module of read requests; what is left out is listed with why.
pub fn parse(text: &str, opts: &Options) -> Result<Imported, String> {
    let doc = Document::parse(text).map_err(|e| format!("not XML: {e}"))?;
    let root = doc.root_element();
    let target = named(root, "Target")
        .next()
        .ok_or("no Target: is this a DDT2000 ECU file (ecus/NAME.xml)?")?;
    let tname = target.attribute("Name").unwrap_or("").trim().to_string();
    let Some(can) = named(root, "CAN").next() else {
        return Err(format!(
            "{}: a K-line module (the file has no CAN node); Icarus does not reach K-line modules yet, so nothing was imported",
            if tname.is_empty() { "this ECU" } else { &tname }
        ));
    };
    let id = |side: &str| {
        child(can, side)
            .and_then(|s| child(s, "CANId"))
            .and_then(|c| c.attribute("Value"))
            .and_then(|v| v.trim().parse::<u32>().ok())
    };
    let tx = id("SendId").ok_or("the CAN node has no SendId/CANId")?;
    let rx = id("ReceiveId").ok_or("the CAN node has no ReceiveId/CANId")?;
    let ids = if tx > 0x7FF || rx > 0x7FF {
        Ids::Can29
    } else {
        Ids::Can11
    };
    let mut out = Imported::default();
    let mut datas: BTreeMap<String, Result<DataDef, String>> = BTreeMap::new();
    for d in named(root, "Data") {
        if let Some(n) = d.attribute("Name") {
            datas.insert(n.trim().to_string(), data_def(d));
        }
    }
    let mut module = ModuleDef {
        format: FORMAT,
        module: ModuleHead {
            name: if tname.is_empty() {
                format!("module ({tx:X})")
            } else {
                tname.clone()
            },
            tx,
            rx: Some(rx),
            ext: None,
            tester: None,
            ids,
            bus: None,
            dialect: None,
            kind: String::new(),
            identity: vec![],
            protocol: None,
            header_byte: None,
        },
        requests: vec![],
    };
    for r in named(root, "Request") {
        let rname = r.attribute("Name").unwrap_or("").trim();
        let what = format!("request {rname:?}");
        let sent_node = child(r, "Sent");
        let sent_hex = sent_node
            .and_then(|s| child(s, "SentBytes"))
            .and_then(|b| b.text())
            .unwrap_or("");
        let Ok(sent) = super::parse_hex(sent_hex) else {
            out.left_out
                .push((what, format!("its SentBytes {sent_hex:?} are not hex")));
            continue;
        };
        if child(r, "ManuelSend").is_some() {
            out.left_out.push((
                what,
                "sent only by hand in DDT2000 (ManuelSend): not imported".into(),
            ));
            continue;
        }
        if child(r, "DenyAccess").is_some() {
            out.left_out.push((
                what,
                "it has access rules (DenyAccess), which Icarus does not read: not imported".into(),
            ));
            continue;
        }
        if sent_node.is_some_and(|s| named(s, "DataItem").next().is_some()) {
            out.left_out.push((
                what,
                "it sends values of its own (Sent data items): not imported (only reads are)"
                    .into(),
            ));
            continue;
        }
        let (service, ident) = match sent.as_slice() {
            [0x22, hi, lo] => (0x22, u16::from_be_bytes([*hi, *lo])),
            [0x21 | 0x1A, i] => (sent[0], u16::from(*i)),
            [s, ..] => {
                out.left_out.push((
                    what,
                    format!(
                        "service {s:02X} ({}): not imported (only reads are: 22 with two bytes, 21 and 1A with one)",
                        crate::diag::service_name(*s)
                    ),
                ));
                continue;
            }
            [] => {
                out.left_out.push((what, "it sends nothing".into()));
                continue;
            }
        };
        let echo = sent.len() - 1;
        let default_little = matches!(r.attribute("Endian").map(str::trim), Some("Little"));
        let mut values = Vec::new();
        let received = child(r, "Received");
        for it in received.iter().flat_map(|rc| {
            rc.children()
                .filter(|c| c.is_element() && c.tag_name().name() == "DataItem")
        }) {
            let Some(name) = it.attribute("Name").map(str::trim) else {
                continue;
            };
            let item = format!("{what}: {name:?}");
            let Some(first) = it
                .attribute("FirstByte")
                .and_then(|f| f.trim().parse::<usize>().ok())
            else {
                out.left_out.push((item, "no FirstByte".into()));
                continue;
            };
            if first < 2 + echo {
                out.left_out
                    .push((item, "inside the answer's echo, not data".into()));
                continue;
            }
            let def = match datas.get(name) {
                Some(Ok(d)) => d,
                Some(Err(why)) => {
                    out.left_out
                        .push((item, format!("its Data does not read: {why}")));
                    continue;
                }
                None => {
                    out.left_out
                        .push((item, "no Data of that name in the file".into()));
                    continue;
                }
            };
            let bit: u8 = it
                .attribute("BitOffset")
                .and_then(|b| b.trim().parse().ok())
                .unwrap_or(0);
            let little = match it.attribute("Endian").map(str::trim) {
                Some("Little") => true,
                Some("Big") => false,
                _ => default_little,
            };
            let mut v = ValueDef::new(&value_id(name), name, first - 2 - echo, def.bits);
            v.bit = bit;
            v.order = if little && def.bits > 8 {
                Order::DdtLittle
            } else {
                Order::Msb
            };
            v.signed = def.signed;
            if def.text {
                v.kind = ValueKind::Text;
            } else if let Some((step, offset, div, unit)) = &def.scaled {
                v.mul = *step;
                v.bias = *offset;
                v.div = *div;
                v.unit = unit.clone();
            } else if !def.states.is_empty() {
                v.kind = ValueKind::State;
                v.states = def.states.clone();
            } else if def.bits.is_multiple_of(8) {
                v.kind = ValueKind::Hex;
            }
            v.note = "from a DDT2000 file, not checked".into();
            // value ids unique in the module
            let taken = |id: &str| {
                module
                    .requests
                    .iter()
                    .flat_map(|q| &q.values)
                    .chain(values.iter())
                    .any(|q: &ValueDef| q.id == id)
            };
            if taken(&v.id) {
                let base = v.id.clone();
                let mut n = 2;
                while taken(&format!("{base}_{n}")) {
                    n += 1;
                }
                v.id = format!("{base}_{n}");
            }
            values.push(v);
        }
        if values.is_empty() {
            out.left_out.push((
                what,
                "it names no value Icarus can read in its answer".into(),
            ));
            continue;
        }
        let sample = named(r, "ReplyBytes")
            .next()
            .and_then(|b| b.text())
            .and_then(|t| super::parse_hex(t).ok())
            .map(|b| {
                b.iter()
                    .map(|x| format!("{x:02X}"))
                    .collect::<Vec<_>>()
                    .join(" ")
            });
        let req = RequestDef {
            service,
            ident,
            every: None,
            min_len: None,
            timeout_ms: None,
            years: vec![],
            unverified: vec![EVERY_YEAR],
            needs_session: None,
            sample,
            samples: vec![],
            values,
        };
        if module
            .requests
            .iter()
            .any(|q| (q.service, q.ident) == (req.service, req.ident))
        {
            out.left_out.push((what, "the same request again".into()));
            continue;
        }
        let mut trial = module.clone();
        trial.requests.push(req);
        match validate(&mut trial) {
            Ok(()) => module = trial,
            Err(e) => out.left_out.push((what, e)),
        }
    }
    let file = opts
        .path
        .rsplit(['/', '\\'])
        .next()
        .unwrap_or("")
        .to_string();
    let name = match &opts.name {
        Some(n) => n.clone(),
        None => format!(
            "ddt-{}",
            set_name(file.trim_end_matches(".xml").trim_end_matches(".XML"))
        ),
    };
    let set = SetInfo {
        format: FORMAT,
        name,
        make: opts.make.clone().unwrap_or_default(),
        models: vec![],
        years: vec![],
        source: Source {
            from: "DDT2000".into(),
            file,
            sha256: sha256_hex(text.as_bytes()),
            imported: opts.today.clone(),
            license: LICENSE.into(),
            attribution: ATTRIBUTION.into(),
        },
    };
    check_set(&set)?;
    out.set = Some(set);
    if !module.requests.is_empty() {
        out.modules = vec![module];
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::moddata::{decode, Shown};

    /// A tiny ECU file written for this test (made-up names and identifiers, never Renault's): reads by local identifier
    /// and by data identifier, a value list, text, a little-endian value, and requests that are left out.
    const ECU: &str = r#"<?xml version="1.0" encoding="utf-8"?>
<Ecu xmlns="http://www-diag.renault.com/2002/ECU">
  <Target Name="Test engine module">
    <CAN>
      <SendId><CANId Value="2016"/></SendId>
      <ReceiveId><CANId Value="2024"/></ReceiveId>
    </CAN>
  </Target>
  <Datas>
    <Data Name="Coolant temperature"><Bits count="8"><Scaled Step="1" Offset="-40" Unit="degC"/></Bits></Data>
    <Data Name="Battery voltage"><Bits count="8"><Scaled Step="1" Offset="0" DivideBy="16" Unit="V"/></Bits></Data>
    <Data Name="Engine speed"><Bits count="16"><Scaled Step="0.125" Unit="rpm"/></Bits></Data>
    <Data Name="Fan state"><Bits count="2"><List><Item Value="0" Text="off"/><Item Value="1" Text="slow"/><Item Value="2" Text="fast"/></List></Bits></Data>
    <Data Name="Torque"><Bits count="16" signed="1"><Scaled Step="0.5" Unit="Nm"/></Bits></Data>
    <Data Name="Odometer"><Bits count="16"><Scaled Step="1" Unit="km"/></Bits></Data>
    <Data Name="Part number"><Bytes count="6" ascii="true"/></Data>
    <Data Name="Calibration"><Bytes count="2"/></Data>
    <Data Name="Setting"><Bits count="8"/></Data>
  </Datas>
  <Requests>
    <Request Name="Read engine values">
      <Sent><SentBytes>2101</SentBytes></Sent>
      <Received MinBytes="8">
        <ReplyBytes>61 01 82 E0 19 00 80 F6 FF</ReplyBytes>
        <DataItem Name="Coolant temperature" FirstByte="3"/>
        <DataItem Name="Battery voltage" FirstByte="4"/>
        <DataItem Name="Engine speed" FirstByte="5"/>
        <DataItem Name="Fan state" FirstByte="7" BitOffset="0"/>
        <DataItem Name="Torque" FirstByte="8"/>
        <DataItem Name="Unknown" FirstByte="9"/>
        <DataItem Name="Coolant temperature" FirstByte="2"/>
      </Received>
    </Request>
    <Request Name="Read identity" Endian="Little">
      <Sent><SentBytes>22F187</SentBytes></Sent>
      <Received>
        <DataItem Name="Part number" FirstByte="4"/>
        <DataItem Name="Calibration" FirstByte="10" Endian="Big"/>
        <DataItem Name="Odometer" FirstByte="12"/>
      </Received>
    </Request>
    <Request Name="Write a setting"><Sent><SentBytes>2E0101</SentBytes><DataItem Name="Setting" FirstByte="4"/></Sent></Request>
    <Request Name="Clear codes"><Sent><SentBytes>14FF00</SentBytes></Sent></Request>
    <Request Name="Protected read"><DenyAccess>Session</DenyAccess><Sent><SentBytes>2102</SentBytes></Sent>
      <Received><DataItem Name="Setting" FirstByte="3"/></Received></Request>
    <Request Name="By hand"><ManuelSend/><Sent><SentBytes>2103</SentBytes></Sent>
      <Received><DataItem Name="Setting" FirstByte="3"/></Received></Request>
  </Requests>
</Ecu>
"#;

    fn opts() -> Options {
        Options {
            path: "/home/me/ecus/Test_ENGINE_v1.xml".into(),
            name: None,
            make: Some("renault".into()),
            today: "2026-11-25".into(),
        }
    }

    /// ICA-5: the tiny file reads into one module of two read requests whose values decode as DDT2000 describes them; the
    /// rest is left out with the reason.
    #[test]
    fn a_tiny_ddt2000_file_reads_into_a_definition() {
        let imp = parse(ECU, &opts()).unwrap();
        let set = imp.set.as_ref().unwrap();
        assert_eq!(
            (
                set.name.as_str(),
                set.make.as_str(),
                set.source.from.as_str()
            ),
            ("ddt-test-engine-v1", "renault", "DDT2000")
        );
        assert_eq!(imp.modules.len(), 1);
        let m = &imp.modules[0];
        assert_eq!(
            (m.module.name.as_str(), m.module.tx, m.module.rx),
            ("Test engine module", 0x7E0, Some(0x7E8))
        );
        assert_eq!(m.requests.len(), 2);
        let engine = &m.requests[0];
        assert_eq!(
            (engine.service, engine.ident, engine.sample.as_deref()),
            (0x21, 0x01, Some("61 01 82 E0 19 00 80 F6 FF"))
        );
        assert!(engine.unverified.contains(&[0, 9999]));
        // after "61 01": 0x82 = 130 - 40 = 90 degC, 0xE0 / 16 = 14 V, 0x1900 * 0.125 = 800 rpm, fan "fast" (0x80: top two bits),
        // torque 0xF6FF signed = -2305 * 0.5
        let data = [0x82, 0xE0, 0x19, 0x00, 0x80, 0xF6, 0xFF];
        let got: Vec<(String, Shown)> = engine
            .values
            .iter()
            .map(|v| (v.id.clone(), decode(v, &data).unwrap().shown))
            .collect();
        assert_eq!(
            got,
            vec![
                ("COOLANT_TEMPERATURE".to_string(), Shown::Number(90.0)),
                ("BATTERY_VOLTAGE".to_string(), Shown::Number(14.0)),
                ("ENGINE_SPEED".to_string(), Shown::Number(800.0)),
                ("FAN_STATE".to_string(), Shown::State("fast".into())),
                ("TORQUE".to_string(), Shown::Number(-1152.5)),
            ]
        );
        let id = &m.requests[1];
        assert_eq!((id.service, id.ident), (0x22, 0xF187));
        let data = b"ABC123\x12\x34\x10\x27";
        let got: Vec<Shown> = id
            .values
            .iter()
            .map(|v| decode(v, data).unwrap().shown)
            .collect();
        assert_eq!(got[0], Shown::Text("ABC123".into()));
        assert_eq!(got[1], Shown::Hex("12 34".into()));
        assert_eq!(
            id.values[2].order,
            Order::DdtLittle,
            "the request's Endian is the default"
        );
        let why = |w: &str| {
            imp.left_out
                .iter()
                .find(|(x, _)| x.contains(w))
                .map(|x| x.1.clone())
                .unwrap_or_else(|| panic!("{w}: {:?}", imp.left_out))
        };
        assert!(why("\"Unknown\"").contains("no Data of that name"));
        assert!(imp
            .left_out
            .iter()
            .any(|(w, y)| w.contains("Coolant temperature") && y.contains("echo")));
        assert!(why("Write a setting").contains("Sent data items"));
        assert!(why("Clear codes").contains("service 14"));
        assert!(why("Protected read").contains("DenyAccess"));
        assert!(why("By hand").contains("ManuelSend"));
    }

    #[test]
    fn a_k_line_file_and_one_that_is_not_ddt2000() {
        let k = ECU.replace("<CAN>", "<K>").replace("</CAN>", "</K>");
        assert!(parse(&k, &opts()).unwrap_err().contains("K-line"));
        assert!(parse("<Something/>", &opts())
            .unwrap_err()
            .contains("no Target"));
        assert!(parse("not xml", &opts()).unwrap_err().contains("not XML"));
    }
}
