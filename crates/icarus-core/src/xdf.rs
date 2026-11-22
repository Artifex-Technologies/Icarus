//! TunerPro definition files (`.xdf`, XML): the most common free format for factory ECU maps. Written from the format as it
//! appears in definition files, not from any program's source.
//!
//! What was checked against a real definition (a Siemens MS43 XDF, 2026-11-07): the header (`BASEOFFSET`, `DEFAULTS`, `REGION`,
//! categories, where `CATEGORYMEM category="N"` names the category with index N-1), `EMBEDDEDDATA` with `mmedtypeflags` 0x01
//! (signed) and 0x02 (least significant byte first), row and column counts (rows are the y axis), axes linked to another
//! element (`embedinfo type="3" linkobjid=...`) and `MATH equation="..."` in X. Read as believed but not yet seen in a real
//! file: 0x04 (column-major), 0x10000 (floating point), stored axis breakpoints, `LABEL` axes, strides and `XDFFLAG` masks.
//!
//! A formula can use other values besides X, each declared as a `VAR` of the `MATH`: `type="link" linkid="0x..."` is
//! another element's value in real units (the element with that `uniqueid`, a single value), `type="address"` the raw
//! number at `address` (`sizeinbits`, `flags` as in `EMBEDDEDDATA`). They are worked out first, in the order they use each
//! other; formulas that use each other in a loop, or a link to an element that is missing or not a single value, leave
//! the element out with the reason.

use crate::def::{Axis, Data, DefError, Definition, Element, Flag, Kind, Source, Table, Var};
use crate::expr::Expr;
use crate::rom::Cell;
use roxmltree::{Document, Node};
use std::collections::HashMap;

pub const FLAG_SIGNED: u32 = 0x01;
pub const FLAG_LSB_FIRST: u32 = 0x02;
pub const FLAG_COLUMN_MAJOR: u32 = 0x04;
pub const FLAG_FLOAT: u32 = 0x10000;

/// A parsed definition and the elements that were left out, each with the reason.
#[derive(Debug)]
pub struct Parsed {
    pub definition: Definition,
    pub skipped: Vec<String>,
}

#[derive(Clone, Copy)]
struct Defaults {
    bits: u8,
    signed: bool,
    lsb_first: bool,
    float: bool,
}

pub fn parse(text: &str) -> Result<Parsed, DefError> {
    let doc = Document::parse(text.trim_start_matches('\u{feff}'))
        .map_err(|e| DefError::Bad(format!("not readable XML: {e}")))?;
    let root = doc.root_element();
    if root.tag_name().name() != "XDFFORMAT" {
        return Err(DefError::Bad(format!(
            "not a TunerPro definition (the root is <{}>, not <XDFFORMAT>)",
            root.tag_name().name()
        )));
    }
    let mut def = Definition::default();
    let mut categories: HashMap<u32, String> = HashMap::new();
    let mut defaults = Defaults {
        bits: 8,
        signed: false,
        lsb_first: false,
        float: false,
    };
    if let Some(h) = child(root, "XDFHEADER") {
        def.title = text_of(h, "deftitle");
        def.description = text_of(h, "description");
        if let Some(b) = child(h, "BASEOFFSET") {
            def.base_offset = attr_num(b, "offset").unwrap_or(0) as u64;
            def.base_subtract = attr_num(b, "subtract").unwrap_or(0) != 0;
        }
        if let Some(d) = child(h, "DEFAULTS") {
            defaults = Defaults {
                bits: attr_num(d, "datasizeinbits").unwrap_or(8) as u8,
                signed: attr_num(d, "signed").unwrap_or(0) != 0,
                lsb_first: attr_num(d, "lsbfirst").unwrap_or(0) != 0,
                float: attr_num(d, "float").unwrap_or(0) != 0,
            };
        }
        if let Some(r) = child(h, "REGION") {
            def.image_size = attr_num(r, "size").map(|s| s as usize);
        }
        for c in h.children().filter(|n| n.has_tag_name("CATEGORY")) {
            if let (Some(i), Some(name)) = (attr_num(c, "index"), c.attribute("name")) {
                categories.insert(i as u32, name.to_string());
            }
        }
    }
    let mut skipped = Vec::new();
    for n in root.children().filter(|n| n.is_element()) {
        let kind = match n.tag_name().name() {
            "XDFTABLE" => table(n, defaults),
            "XDFCONSTANT" => embedded(n, defaults, 1, 1).map(Kind::Constant),
            "XDFFLAG" => flag(n, defaults),
            _ => continue,
        };
        let title = text_of(n, "title");
        match kind {
            Ok(kind) => def.elements.push(Element {
                id: attr_num(n, "uniqueid").map(|v| v as u32),
                title,
                description: text_of(n, "description"),
                categories: n
                    .children()
                    .filter(|c| c.has_tag_name("CATEGORYMEM"))
                    .filter_map(|c| attr_num(c, "category"))
                    .filter_map(|k| categories.get(&((k as u32).wrapping_sub(1))).cloned())
                    .collect(),
                kind,
            }),
            Err(e) => skipped.push(format!("{}: {e}", shown_title(title))),
        }
    }
    // formulas that use other elements: each one must lead to single values, never round in a loop (leaving one out can
    // leave another without its value, so until none is left)
    loop {
        let problems = def.formula_problems();
        if problems.is_empty() {
            break;
        }
        let mut out: Vec<String> = problems
            .iter()
            .rev()
            .map(|(i, why)| format!("{}: {why}", shown_title(def.elements.remove(*i).title)))
            .collect();
        out.reverse();
        skipped.extend(out);
    }
    Ok(Parsed {
        definition: def,
        skipped,
    })
}

fn shown_title(title: String) -> String {
    if title.is_empty() {
        "(untitled)".to_string()
    } else {
        title
    }
}

fn child<'a, 'i>(n: Node<'a, 'i>, tag: &str) -> Option<Node<'a, 'i>> {
    n.children().find(|c| c.has_tag_name(tag))
}

fn text_of(n: Node, tag: &str) -> String {
    child(n, tag)
        .and_then(|c| c.text())
        .unwrap_or("")
        .trim()
        .to_string()
}

/// "0x5B5C", "23" or "12.000000" as a number.
fn num(s: &str) -> Option<f64> {
    let s = s.trim();
    if let Some(h) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        return u64::from_str_radix(h, 16).ok().map(|v| v as f64);
    }
    s.parse::<f64>().ok()
}

fn attr_num(n: Node, a: &str) -> Option<i64> {
    n.attribute(a).and_then(num).map(|v| v as i64)
}

/// A node's formula and its variables besides X (see the top of this file).
fn math(n: Node, d: Defaults) -> Result<(Expr, Vec<Var>), DefError> {
    let Some(m) = child(n, "MATH") else {
        return Ok((Expr::identity(), vec![]));
    };
    let mut vars = Vec::new();
    for v in m.children().filter(|c| c.has_tag_name("VAR")) {
        let id = v.attribute("id").unwrap_or("").trim();
        if id.eq_ignore_ascii_case("X") {
            continue;
        }
        if id.is_empty() {
            return Err(DefError::Bad("a formula variable has no name".into()));
        }
        let kind = v
            .attribute("type")
            .unwrap_or("")
            .trim()
            .to_ascii_lowercase();
        let source = match kind.as_str() {
            "link" => Source::Element(
                attr_num(v, "linkid")
                    .filter(|l| (0..=u32::MAX as i64).contains(l))
                    .ok_or_else(|| {
                        DefError::Bad(format!(
                            "formula variable {id} links to no element (no linkid)"
                        ))
                    })? as u32,
            ),
            "address" => {
                let address = attr_num(v, "address").filter(|a| *a >= 0).ok_or_else(|| {
                    DefError::Bad(format!("formula variable {id} has no address"))
                })?;
                let flags = attr_num(v, "flags").map(|f| f as u32);
                let cell = Cell {
                    bits: attr_num(v, "sizeinbits").map_or(d.bits, |b| b as u8),
                    signed: flags.map_or(d.signed, |f| f & FLAG_SIGNED != 0),
                    little_endian: flags.map_or(d.lsb_first, |f| f & FLAG_LSB_FIRST != 0),
                    float: flags.map_or(d.float, |f| f & FLAG_FLOAT != 0),
                };
                cell.check()?;
                Source::Address {
                    address: address as u64,
                    cell,
                }
            }
            "" => {
                return Err(DefError::Bad(format!(
                    "formula variable {id} does not say where its value comes from (no type)"
                )))
            }
            other => {
                return Err(DefError::Bad(format!(
                    "formula variable {id} is of type \"{other}\", which Icarus does not read"
                )))
            }
        };
        vars.push(Var {
            name: id.to_string(),
            source,
        });
    }
    let names: Vec<&str> = vars.iter().map(|v| v.name.as_str()).collect();
    let expr = Expr::parse_with(m.attribute("equation").unwrap_or("X"), &names)?;
    Ok((expr, vars))
}

/// The numbers of an element or axis node: its EMBEDDEDDATA, MATH, units and limits.
fn embedded(
    n: Node,
    d: Defaults,
    rows_default: usize,
    cols_default: usize,
) -> Result<Data, DefError> {
    let e = child(n, "EMBEDDEDDATA").ok_or_else(|| DefError::Missing("no EMBEDDEDDATA".into()))?;
    let address =
        attr_num(e, "mmedaddress").ok_or_else(|| DefError::Missing("no address".into()))?;
    if address < 0 {
        return Err(DefError::Bad("negative address".into()));
    }
    let flags = attr_num(e, "mmedtypeflags").map(|f| f as u32);
    let float = flags.map_or(d.float, |f| f & FLAG_FLOAT != 0);
    let cell = Cell {
        bits: attr_num(e, "mmedelementsizebits").map_or(d.bits, |b| b as u8),
        signed: flags.map_or(d.signed, |f| f & FLAG_SIGNED != 0),
        little_endian: flags.map_or(d.lsb_first, |f| f & FLAG_LSB_FIRST != 0),
        float,
    };
    cell.check()?;
    let count = |a: &str, default: usize| {
        attr_num(e, a)
            .filter(|v| *v > 0)
            .map_or(default, |v| v as usize)
    };
    let float_of = |tag: &str| child(n, tag).and_then(|c| c.text()).and_then(num);
    let (math, vars) = math(n, d)?;
    Ok(Data {
        address: address as u64,
        cell,
        rows: count("mmedrowcount", rows_default),
        cols: count("mmedcolcount", cols_default),
        column_major: flags.is_some_and(|f| f & FLAG_COLUMN_MAJOR != 0),
        minor_stride_bits: attr_num(e, "mmedminorstridebits").unwrap_or(0).max(0),
        major_stride_bits: attr_num(e, "mmedmajorstridebits").unwrap_or(0).max(0),
        math,
        vars,
        units: text_of(n, "units"),
        decimals: float_of("decimalpl").map_or(2, |v| v.clamp(0.0, 9.0) as u8),
        min: float_of("min"),
        max: float_of("max"),
    })
}

fn axis(n: Node, d: Defaults) -> Result<Axis, DefError> {
    let count = text_of(n, "indexcount")
        .parse::<usize>()
        .unwrap_or(1)
        .max(1);
    if let Some(link) = child(n, "embedinfo").and_then(|e| attr_num(e, "linkobjid")) {
        let (math, vars) = math(n, d)?;
        return Ok(Axis::Link {
            id: link as u32,
            math,
            vars,
            units: text_of(n, "units"),
        });
    }
    if child(n, "EMBEDDEDDATA").is_some_and(|e| e.attribute("mmedaddress").is_some()) {
        return Ok(Axis::Stored(embedded(n, d, 1, count)?));
    }
    let mut labels: Vec<(i64, String)> = n
        .children()
        .filter(|c| c.has_tag_name("LABEL"))
        .filter_map(|c| Some((attr_num(c, "index")?, c.attribute("value")?.to_string())))
        .collect();
    if !labels.is_empty() {
        labels.sort_by_key(|l| l.0);
        return Ok(Axis::Labels(labels.into_iter().map(|l| l.1).collect()));
    }
    Ok(Axis::Index(count))
}

fn table(n: Node, d: Defaults) -> Result<Kind, DefError> {
    let axis_node = |id: &str| {
        n.children()
            .find(|c| c.has_tag_name("XDFAXIS") && c.attribute("id") == Some(id))
    };
    let z = embedded(
        axis_node("z").ok_or_else(|| DefError::Missing("no z axis".into()))?,
        d,
        1,
        1,
    )?;
    let x = axis_node("x")
        .map(|a| axis(a, d))
        .transpose()?
        .unwrap_or(Axis::Index(z.cols));
    let y = axis_node("y")
        .map(|a| axis(a, d))
        .transpose()?
        .unwrap_or(Axis::Index(z.rows));
    Ok(Kind::Table(Table { x, y, z }))
}

fn flag(n: Node, d: Defaults) -> Result<Kind, DefError> {
    let data = embedded(n, d, 1, 1)?;
    let mask = child(n, "mask")
        .and_then(|m| m.text())
        .and_then(num)
        .map_or(1, |m| m as u64);
    if mask == 0 {
        return Err(DefError::Bad("flag mask is 0".into()));
    }
    Ok(Kind::Flag(Flag {
        address: data.address,
        cell: data.cell,
        mask,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rom::Rom;

    /// A small definition written for these tests in the layout real XDF files use.
    const SAMPLE: &str = r#"<!-- Written by hand for Icarus's tests -->
<XDFFORMAT version="1.70">
  <XDFHEADER>
    <flags>0x1</flags>
    <deftitle>Test ECU</deftitle>
    <description>Made up</description>
    <BASEOFFSET offset="0x100" subtract="0" />
    <DEFAULTS datasizeinbits="8" sigdigits="2" outputtype="1" signed="0" lsbfirst="0" float="0" />
    <REGION type="0xFFFFFFFF" startaddress="0x0" size="0x200" name="Binary File" />
    <CATEGORY index="0x0" name="Axis" />
    <CATEGORY index="0x1" name="Ignition" />
  </XDFHEADER>
  <XDFTABLE uniqueid="0x10" flags="0x30">
    <title>rpm breakpoints</title>
    <CATEGORYMEM index="0" category="1" />
    <XDFAXIS id="x" uniqueid="0x0"><indexcount>1</indexcount></XDFAXIS>
    <XDFAXIS id="y" uniqueid="0x0"><indexcount>3</indexcount></XDFAXIS>
    <XDFAXIS id="z">
      <EMBEDDEDDATA mmedtypeflags="0x02" mmedaddress="0x00" mmedelementsizebits="16" mmedrowcount="3" mmedcolcount="1" />
      <decimalpl>0</decimalpl>
      <MATH equation="X"><VAR id="X" /></MATH>
    </XDFAXIS>
  </XDFTABLE>
  <XDFTABLE uniqueid="0x20" flags="0x30">
    <title>Spark advance</title>
    <description>Base timing &#xB0;BTDC</description>
    <CATEGORYMEM index="0" category="2" />
    <XDFAXIS id="x" uniqueid="0x0">
      <indexcount>2</indexcount>
      <LABEL index="1" value="high load" />
      <LABEL index="0" value="low load" />
    </XDFAXIS>
    <XDFAXIS id="y" uniqueid="0x0">
      <indexcount>3</indexcount>
      <embedinfo type="3" linkobjid="0x10" />
      <MATH equation="X"><VAR id="X" /></MATH>
    </XDFAXIS>
    <XDFAXIS id="z">
      <EMBEDDEDDATA mmedtypeflags="0x01" mmedaddress="0x10" mmedelementsizebits="8" mmedrowcount="3" mmedcolcount="2" />
      <units>&#xB0;</units>
      <decimalpl>1</decimalpl>
      <min>-20</min>
      <max>50</max>
      <MATH equation="0.5*X"><VAR id="X" /></MATH>
    </XDFAXIS>
  </XDFTABLE>
  <XDFCONSTANT uniqueid="0x30">
    <title>Rev limit</title>
    <EMBEDDEDDATA mmedtypeflags="0x00" mmedaddress="0x20" mmedelementsizebits="16" />
    <units>rpm</units>
    <MATH equation="X*4"><VAR id="X" /></MATH>
  </XDFCONSTANT>
  <XDFFLAG uniqueid="0x40">
    <title>Fan switch</title>
    <EMBEDDEDDATA mmedaddress="0x22" mmedelementsizebits="8" />
    <mask>0x04</mask>
  </XDFFLAG>
  <XDFCONSTANT uniqueid="0x50">
    <title>Needs another value</title>
    <EMBEDDEDDATA mmedaddress="0x24" mmedelementsizebits="8" />
    <MATH equation="X*Y"><VAR id="X" /><VAR id="Y" /></MATH>
  </XDFCONSTANT>
</XDFFORMAT>"#;

    fn image() -> Rom {
        let mut img = vec![0u8; 0x200];
        // rpm breakpoints, LSB first: 800, 2400, 6000
        img[0x100..0x106].copy_from_slice(&[0x20, 0x03, 0x60, 0x09, 0x70, 0x17]);
        // spark, signed, 0.5 deg/bit: rows 800 / 2400 / 6000, cols low / high load
        img[0x110..0x116].copy_from_slice(&[20, 10, 50, 30, 60, 0xF0]);
        // rev limit 1750*4 = 7000, big-endian
        img[0x120..0x122].copy_from_slice(&1750u16.to_be_bytes());
        img[0x122] = 0b0000_0100;
        Rom::new(img)
    }

    #[test]
    fn reads_tables_constants_and_flags() {
        let p = parse(SAMPLE).unwrap();
        let def = &p.definition;
        assert_eq!(
            (def.title.as_str(), def.base_offset, def.image_size),
            ("Test ECU", 0x100, Some(0x200))
        );
        assert_eq!(def.elements.len(), 4);
        assert_eq!(p.skipped.len(), 1);
        assert!(
            p.skipped[0].starts_with("Needs another value: "),
            "{:?}",
            p.skipped
        );
        let rom = image();
        let spark = def.element("Spark advance").unwrap();
        assert_eq!(spark.categories, vec!["Ignition"]);
        assert_eq!(spark.description, "Base timing °BTDC");
        let Kind::Table(t) = &spark.kind else {
            panic!()
        };
        let v = def.table(&rom, t).unwrap();
        assert_eq!(v.x, vec!["low load", "high load"], "labels in index order");
        assert_eq!(v.y, vec!["800", "2400", "6000"]);
        assert_eq!(
            v.z,
            vec![vec![10.0, 5.0], vec![25.0, 15.0], vec![30.0, -8.0]]
        );
        let Kind::Constant(rev) = &def.element("Rev limit").unwrap().kind else {
            panic!()
        };
        assert_eq!(def.values(&rom, rev).unwrap(), vec![vec![7000.0]]);
        let Kind::Flag(f) = &def.element("Fan switch").unwrap().kind else {
            panic!()
        };
        assert!(def.flag(&rom, f).unwrap());
    }

    #[test]
    fn an_edit_lands_in_the_right_byte() {
        let p = parse(SAMPLE).unwrap();
        let def = &p.definition;
        let mut rom = image();
        let Kind::Table(t) = &def.element("Spark").unwrap().kind else {
            panic!()
        };
        assert_eq!(def.set(&mut rom, &t.z, 2, 1, -6.0).unwrap(), -6.0);
        assert_eq!(rom.changes(), vec![0x115..0x116]);
        assert_eq!(rom.bytes()[0x115] as i8, -12);
        assert!(
            def.set(&mut rom, &t.z, 0, 0, 60.0).is_err(),
            "above the definition's max of 50"
        );
        let names: Vec<_> = def
            .covering(0x115..0x116)
            .iter()
            .map(|e| e.title.clone())
            .collect();
        assert_eq!(names, vec!["Spark advance"]);
    }

    /// Formulas that use other values, written for this test: a map that uses a value that uses another value, a raw
    /// number from an address, and the cases that cannot be worked out.
    const LINKED: &str = r#"<!-- Written by hand for Icarus's tests -->
<XDFFORMAT version="1.70">
  <XDFHEADER>
    <deftitle>Linked formulas</deftitle>
    <BASEOFFSET offset="0" subtract="0" />
    <DEFAULTS datasizeinbits="8" sigdigits="2" outputtype="1" signed="0" lsbfirst="0" float="0" />
    <REGION type="0xFFFFFFFF" startaddress="0x0" size="0x20" name="Binary File" />
  </XDFHEADER>
  <XDFCONSTANT uniqueid="0x100">
    <title>Injector size</title>
    <EMBEDDEDDATA mmedtypeflags="0x02" mmedaddress="0x00" mmedelementsizebits="16" />
    <units>cc/min</units>
    <decimalpl>0</decimalpl>
    <MATH equation="X/2"><VAR id="X" /></MATH>
  </XDFCONSTANT>
  <XDFTABLE uniqueid="0x102">
    <title>Fuel map</title>
    <XDFAXIS id="x" uniqueid="0x0"><indexcount>2</indexcount></XDFAXIS>
    <XDFAXIS id="y" uniqueid="0x0"><indexcount>2</indexcount></XDFAXIS>
    <XDFAXIS id="z">
      <EMBEDDEDDATA mmedaddress="0x10" mmedelementsizebits="8" mmedrowcount="2" mmedcolcount="2" />
      <units>mg</units>
      <decimalpl>1</decimalpl>
      <MATH equation="X*Y/Scale">
        <VAR id="X" />
        <VAR id="Y" type="link" linkid="0x101" />
        <VAR id="Scale" type="address" address="0x04" sizeinbits="8" flags="0x0" />
      </MATH>
    </XDFAXIS>
  </XDFTABLE>
  <XDFCONSTANT uniqueid="0x101">
    <title>Fuel per pulse</title>
    <EMBEDDEDDATA mmedaddress="0x02" mmedelementsizebits="8" />
    <decimalpl>1</decimalpl>
    <MATH equation="X*Y/100"><VAR id="X" /><VAR id="Y" type="link" linkid="0x100" /></MATH>
  </XDFCONSTANT>
  <XDFCONSTANT uniqueid="0x103">
    <title>Scaled by zero</title>
    <EMBEDDEDDATA mmedaddress="0x07" mmedelementsizebits="8" />
    <MATH equation="X*X*Y"><VAR id="X" /><VAR id="Y" type="address" address="0x06" sizeinbits="8" /></MATH>
  </XDFCONSTANT>
  <XDFCONSTANT uniqueid="0x200">
    <title>Loop A</title>
    <EMBEDDEDDATA mmedaddress="0x08" mmedelementsizebits="8" />
    <MATH equation="X+Y"><VAR id="X" /><VAR id="Y" type="link" linkid="0x201" /></MATH>
  </XDFCONSTANT>
  <XDFCONSTANT uniqueid="0x201">
    <title>Loop B</title>
    <EMBEDDEDDATA mmedaddress="0x09" mmedelementsizebits="8" />
    <MATH equation="X+Y"><VAR id="X" /><VAR id="Y" type="link" linkid="0x200" /></MATH>
  </XDFCONSTANT>
  <XDFCONSTANT uniqueid="0x202">
    <title>Missing link</title>
    <EMBEDDEDDATA mmedaddress="0x0A" mmedelementsizebits="8" />
    <MATH equation="X+Y"><VAR id="X" /><VAR id="Y" type="link" linkid="0x999" /></MATH>
  </XDFCONSTANT>
  <XDFCONSTANT uniqueid="0x203">
    <title>Uses a map</title>
    <EMBEDDEDDATA mmedaddress="0x0B" mmedelementsizebits="8" />
    <MATH equation="X+Y"><VAR id="X" /><VAR id="Y" type="link" linkid="0x102" /></MATH>
  </XDFCONSTANT>
  <XDFCONSTANT uniqueid="0x204">
    <title>Unknown variable kind</title>
    <EMBEDDEDDATA mmedaddress="0x0C" mmedelementsizebits="8" />
    <MATH equation="X+Y"><VAR id="X" /><VAR id="Y" type="inputvar" /></MATH>
  </XDFCONSTANT>
</XDFFORMAT>"#;

    #[test]
    fn formulas_that_use_other_values_load_in_real_units() {
        let p = parse(LINKED).unwrap();
        let def = &p.definition;
        let titles: Vec<&str> = def.elements.iter().map(|e| e.title.as_str()).collect();
        assert_eq!(
            titles,
            vec![
                "Injector size",
                "Fuel map",
                "Fuel per pulse",
                "Scaled by zero"
            ]
        );
        let why = |t: &str| {
            p.skipped
                .iter()
                .find(|s| s.starts_with(&format!("{t}: ")))
                .cloned()
                .unwrap_or_else(|| panic!("{t} not left out: {:?}", p.skipped))
        };
        assert!(why("Loop A").contains("\"Loop A\" uses \"Loop B\" uses \"Loop A\""));
        assert!(why("Loop B").contains("loop"));
        assert!(why("Missing link").contains("0x999, which the definition lacks"));
        assert!(why("Uses a map").contains("not a single value"));
        assert!(why("Unknown variable kind").contains("\"inputvar\""));
        assert_eq!(p.skipped.len(), 5);

        let mut img = vec![0u8; 0x20];
        img[0..2].copy_from_slice(&880u16.to_le_bytes()); // 440 cc/min
        img[2] = 50; // 50 * 440 / 100 = 220
        img[4] = 100; // the map's divisor, raw
        img[7] = 5;
        img[0x10..0x14].copy_from_slice(&[10, 20, 30, 40]); // X * 220 / 100
        let mut rom = Rom::new(img);
        let Kind::Constant(fpp) = &def.element("Fuel per pulse").unwrap().kind else {
            panic!()
        };
        assert_eq!(def.values(&rom, fpp).unwrap(), vec![vec![220.0]]);
        let Kind::Table(map) = &def.element("Fuel map").unwrap().kind else {
            panic!()
        };
        assert_eq!(
            def.table(&rom, map).unwrap().z,
            vec![vec![22.0, 44.0], vec![66.0, 88.0]],
            "worked out after the values it uses, in their order"
        );

        // stored from a shown value with the linked values as they are: 33 mg / 2.2 = 15
        assert_eq!(def.set(&mut rom, &map.z, 0, 0, 33.0).unwrap(), 33.0);
        assert_eq!(rom.bytes()[0x10], 15);
        // a change to the value at the start of the chain changes what the others show, not what they store
        let Kind::Constant(inj) = &def.element("Injector size").unwrap().kind else {
            panic!()
        };
        assert_eq!(def.set(&mut rom, inj, 0, 0, 220.0).unwrap(), 220.0);
        assert_eq!(def.values(&rom, fpp).unwrap(), vec![vec![110.0]]);
        assert_eq!(def.table(&rom, map).unwrap().z[0][0], 16.5);
        assert_eq!(rom.bytes()[0x10], 15);
        let changed: Vec<&str> = def
            .covering(0..2)
            .iter()
            .map(|e| e.title.as_str())
            .collect();
        assert_eq!(
            changed,
            vec!["Injector size", "Fuel map", "Fuel per pulse"],
            "what uses the changed value shows as changed too"
        );
        // with the value it uses at 0, the formula gives 0 whatever is stored: refused, never a guess
        let Kind::Constant(zero) = &def.element("Scaled by zero").unwrap().kind else {
            panic!()
        };
        assert_eq!(def.values(&rom, zero).unwrap(), vec![vec![0.0]]);
        let err = def.set(&mut rom, zero, 0, 0, 3.0).unwrap_err().to_string();
        assert!(err.contains("same value whatever is stored"), "{err}");
        assert_eq!(rom.bytes()[7], 5, "nothing stored");
    }

    #[test]
    fn other_files_are_refused_clearly() {
        assert!(parse("<html><body/></html>")
            .unwrap_err()
            .to_string()
            .contains("not a TunerPro definition"));
        assert!(parse("not xml at all").is_err());
    }
}
