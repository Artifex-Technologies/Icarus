//! A definition: what the numbers in a ROM mean. Each element names a place in the image (a table, a single value, or a flag
//! bit), how its numbers are stored, and the formula from stored number to real unit. The same model is filled from a TunerPro
//! `.xdf` (see `xdf`) and, later, from A2L files and Icarus's own definitions.

use crate::expr::{Expr, ExprError};
use crate::rom::{Cell, Rom, RomError};
use std::fmt;

#[derive(Clone, Debug, Default)]
pub struct Definition {
    pub title: String,
    pub description: String,
    /// Added to (or, with `base_subtract`, taken from) every address to give the offset in the file.
    pub base_offset: u64,
    pub base_subtract: bool,
    /// The image size the definition was written for, when it says.
    pub image_size: Option<usize>,
    pub elements: Vec<Element>,
}

#[derive(Clone, Debug)]
pub struct Element {
    /// The definition's own id (links between elements use it).
    pub id: Option<u32>,
    pub title: String,
    pub description: String,
    pub categories: Vec<String>,
    pub kind: Kind,
}

// Elements are built once per definition and matched by reference (`Constant(d) | Table(Table { z: d, .. })`); boxing Table saves little.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Debug)]
pub enum Kind {
    /// One value.
    Constant(Data),
    /// A curve (one axis) or a map (two).
    Table(Table),
    /// One bit (or bits) of a byte or word: a switch.
    Flag(Flag),
}

/// Numbers stored in the image: `rows` x `cols` cells from `address`.
#[derive(Clone, Debug)]
pub struct Data {
    pub address: u64,
    pub cell: Cell,
    pub rows: usize,
    pub cols: usize,
    /// Stored column by column instead of row by row.
    pub column_major: bool,
    /// Bits from one cell to the next along a row (0: the cell's own size, packed).
    pub minor_stride_bits: i64,
    /// Bits from one row to the next (0: packed).
    pub major_stride_bits: i64,
    pub math: Expr,
    pub units: String,
    pub decimals: u8,
    pub min: Option<f64>,
    pub max: Option<f64>,
}

#[derive(Clone, Debug)]
pub struct Table {
    /// Across the top (columns).
    pub x: Axis,
    /// Down the side (rows).
    pub y: Axis,
    pub z: Data,
}

#[derive(Clone, Debug)]
pub enum Axis {
    /// Only positions 0, 1, 2...
    Index(usize),
    /// Fixed labels written in the definition.
    Labels(Vec<String>),
    /// Breakpoints stored in the image.
    Stored(Data),
    /// Breakpoints taken from another element (its id), converted by this axis's formula.
    Link { id: u32, math: Expr, units: String },
}

#[derive(Clone, Debug)]
pub struct Flag {
    pub address: u64,
    pub cell: Cell,
    pub mask: u64,
}

#[derive(Debug)]
pub enum DefError {
    Rom(RomError),
    Expr(ExprError),
    Missing(String),
    Bad(String),
}

impl fmt::Display for DefError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            DefError::Rom(e) => e.fmt(f),
            DefError::Expr(e) => write!(f, "formula: {e}"),
            DefError::Missing(m) | DefError::Bad(m) => f.write_str(m),
        }
    }
}

impl std::error::Error for DefError {}

impl From<RomError> for DefError {
    fn from(e: RomError) -> Self {
        DefError::Rom(e)
    }
}

impl From<ExprError> for DefError {
    fn from(e: ExprError) -> Self {
        DefError::Expr(e)
    }
}

/// A table as read from an image: the breakpoints of both axes and the values, in real units.
#[derive(Clone, Debug, PartialEq)]
pub struct TableValues {
    pub x: Vec<String>,
    pub y: Vec<String>,
    /// rows x cols.
    pub z: Vec<Vec<f64>>,
}

impl Definition {
    /// The file offset of a definition address.
    pub fn offset(&self, address: u64) -> Result<usize, DefError> {
        let a = if self.base_subtract {
            address.checked_sub(self.base_offset)
        } else {
            address.checked_add(self.base_offset)
        };
        a.and_then(|a| usize::try_from(a).ok())
            .ok_or_else(|| DefError::Bad(format!("address 0x{address:X} is below the base offset")))
    }

    pub fn by_id(&self, id: u32) -> Option<&Element> {
        self.elements.iter().find(|e| e.id == Some(id))
    }

    /// Elements whose title or description contains `text` (any case).
    pub fn find(&self, text: &str) -> Vec<&Element> {
        let t = text.to_lowercase();
        self.elements
            .iter()
            .filter(|e| {
                e.title.to_lowercase().contains(&t) || e.description.to_lowercase().contains(&t)
            })
            .collect()
    }

    /// The element titled exactly `title`, else the only one whose title contains it.
    pub fn element(&self, title: &str) -> Result<&Element, DefError> {
        if let Some(e) = self.elements.iter().find(|e| e.title == title) {
            return Ok(e);
        }
        let t = title.to_lowercase();
        let hits: Vec<&Element> = self
            .elements
            .iter()
            .filter(|e| e.title.to_lowercase().contains(&t))
            .collect();
        match hits.len() {
            1 => Ok(hits[0]),
            0 => Err(DefError::Missing(format!("no element titled \"{title}\""))),
            n => Err(DefError::Missing(format!(
                "{n} elements match \"{title}\"; give the full title"
            ))),
        }
    }

    /// The file offset of cell (row, col) of `d`.
    pub fn cell_offset(&self, d: &Data, row: usize, col: usize) -> Result<usize, DefError> {
        if row >= d.rows || col >= d.cols {
            return Err(DefError::Bad(format!(
                "cell ({row}, {col}) is outside a {}x{} table",
                d.rows, d.cols
            )));
        }
        let size_bits = 8 * d.cell.bytes() as i64;
        let minor = if d.minor_stride_bits > 0 {
            d.minor_stride_bits
        } else {
            size_bits
        };
        let (major_count, minor_idx, major_idx) = if d.column_major {
            (d.rows, row, col)
        } else {
            (d.cols, col, row)
        };
        let major = if d.major_stride_bits > 0 {
            d.major_stride_bits
        } else {
            minor * major_count as i64
        };
        let bits = major_idx as i64 * major + minor_idx as i64 * minor;
        if bits % 8 != 0 {
            return Err(DefError::Bad(
                "cells that do not start on a byte are not supported".into(),
            ));
        }
        Ok(self.offset(d.address)? + (bits / 8) as usize)
    }

    /// All values of `d`, rows x cols, in real units.
    pub fn values(&self, rom: &Rom, d: &Data) -> Result<Vec<Vec<f64>>, DefError> {
        let mut out = Vec::with_capacity(d.rows);
        for r in 0..d.rows {
            let mut row = Vec::with_capacity(d.cols);
            for c in 0..d.cols {
                row.push(d.math.eval(rom.read(self.cell_offset(d, r, c)?, d.cell)?));
            }
            out.push(row);
        }
        Ok(out)
    }

    /// Stores `value` (real units) in cell (row, col) of `d`; returns the value as stored (after rounding to the cell).
    pub fn set(
        &self,
        rom: &mut Rom,
        d: &Data,
        row: usize,
        col: usize,
        value: f64,
    ) -> Result<f64, DefError> {
        if d.min.is_some_and(|m| value < m) || d.max.is_some_and(|m| value > m) {
            return Err(DefError::Bad(format!(
                "{value} is outside the definition's limits ({} to {})",
                d.min.map_or("-".into(), |m| m.to_string()),
                d.max.map_or("-".into(), |m| m.to_string())
            )));
        }
        let at = self.cell_offset(d, row, col)?;
        let raw = if d.cell.float {
            d.math.invert_float(value)?
        } else {
            let (lo, hi) = d.cell.range();
            d.math.invert_int(value, lo, hi)? as f64
        };
        rom.write(at, d.cell, raw)?;
        Ok(d.math.eval(rom.read(at, d.cell)?))
    }

    fn axis_labels(&self, rom: &Rom, axis: &Axis, count: usize) -> Result<Vec<String>, DefError> {
        let nums = |v: Vec<f64>, decimals: u8| {
            v.into_iter()
                .map(|n| fmt_num(n, decimals))
                .collect::<Vec<_>>()
        };
        let labels = match axis {
            Axis::Index(_) => (0..count).map(|i| i.to_string()).collect(),
            Axis::Labels(l) => l.clone(),
            Axis::Stored(d) => nums(self.values(rom, d)?.concat(), d.decimals),
            Axis::Link { id, math, .. } => {
                let e = self.by_id(*id).ok_or_else(|| {
                    DefError::Missing(format!(
                        "axis links to element 0x{id:X}, which the definition lacks"
                    ))
                })?;
                let d = match &e.kind {
                    Kind::Table(t) => &t.z,
                    Kind::Constant(d) => d,
                    Kind::Flag(_) => {
                        return Err(DefError::Bad("an axis cannot link to a flag".into()))
                    }
                };
                let v = self
                    .values(rom, d)?
                    .concat()
                    .into_iter()
                    .map(|n| math.eval(n))
                    .collect();
                nums(v, d.decimals)
            }
        };
        Ok(labels)
    }

    /// A table's axes and values.
    pub fn table(&self, rom: &Rom, t: &Table) -> Result<TableValues, DefError> {
        Ok(TableValues {
            x: self.axis_labels(rom, &t.x, t.z.cols)?,
            y: self.axis_labels(rom, &t.y, t.z.rows)?,
            z: self.values(rom, &t.z)?,
        })
    }

    /// Whether a flag is set.
    pub fn flag(&self, rom: &Rom, f: &Flag) -> Result<bool, DefError> {
        let raw = rom.read(self.offset(f.address)?, f.cell)? as i64 as u64;
        Ok(raw & f.mask != 0)
    }

    /// Switches a flag on or off (only its mask's bits change).
    pub fn set_flag(&self, rom: &mut Rom, f: &Flag, on: bool) -> Result<(), DefError> {
        let at = self.offset(f.address)?;
        let raw = rom.read(at, f.cell)? as i64 as u64;
        let new = if on { raw | f.mask } else { raw & !f.mask };
        rom.write(at, f.cell, new as f64)?;
        Ok(())
    }

    /// Which elements cover the bytes in `range` (file offsets): what a tune changed, by name.
    pub fn covering(&self, range: std::ops::Range<usize>) -> Vec<&Element> {
        self.elements
            .iter()
            .filter(|e| {
                let spans: Vec<std::ops::Range<usize>> = match &e.kind {
                    Kind::Constant(d) => self.span(d).into_iter().collect(),
                    Kind::Table(t) => {
                        let mut v: Vec<_> = self.span(&t.z).into_iter().collect();
                        for a in [&t.x, &t.y] {
                            if let Axis::Stored(d) = a {
                                v.extend(self.span(d));
                            }
                        }
                        v
                    }
                    Kind::Flag(f) => self
                        .offset(f.address)
                        .ok()
                        .map(|o| o..o + f.cell.bytes())
                        .into_iter()
                        .collect(),
                };
                spans
                    .iter()
                    .any(|s| s.start < range.end && range.start < s.end)
            })
            .collect()
    }

    /// The bytes `d` occupies (first to last cell).
    pub fn span(&self, d: &Data) -> Option<std::ops::Range<usize>> {
        let first = self.cell_offset(d, 0, 0).ok()?;
        let last = self
            .cell_offset(d, d.rows.checked_sub(1)?, d.cols.checked_sub(1)?)
            .ok()?;
        Some(first.min(last)..first.max(last) + d.cell.bytes())
    }
}

pub fn fmt_num(n: f64, decimals: u8) -> String {
    format!("{:.*}", decimals as usize, n)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn data(address: u64, rows: usize, cols: usize, math: &str) -> Data {
        Data {
            address,
            cell: Cell::U8,
            rows,
            cols,
            column_major: false,
            minor_stride_bits: 0,
            major_stride_bits: 0,
            math: Expr::parse(math).unwrap(),
            units: String::new(),
            decimals: 1,
            min: None,
            max: None,
        }
    }

    fn el(id: u32, title: &str, kind: Kind) -> Element {
        Element {
            id: Some(id),
            title: title.into(),
            description: String::new(),
            categories: vec![],
            kind,
        }
    }

    #[test]
    fn a_map_with_a_linked_axis_reads_in_real_units() {
        // image: rpm breakpoints at 0x10 (x/0.04 -> rpm = 40*X), a 2x3 map at 0x20 (X/2)
        let mut img = vec![0u8; 0x40];
        img[0x10..0x12].copy_from_slice(&[25, 50]);
        img[0x20..0x26].copy_from_slice(&[10, 20, 30, 40, 50, 60]);
        let def = Definition {
            base_offset: 0x10,
            elements: vec![
                el(
                    1,
                    "rpm axis",
                    Kind::Table(Table {
                        x: Axis::Index(1),
                        y: Axis::Index(2),
                        z: data(0x00, 2, 1, "40*X"),
                    }),
                ),
                el(
                    2,
                    "spark map",
                    Kind::Table(Table {
                        x: Axis::Labels(vec!["a".into(), "b".into(), "c".into()]),
                        y: Axis::Link {
                            id: 1,
                            math: Expr::identity(),
                            units: "rpm".into(),
                        },
                        z: data(0x10, 2, 3, "X/2"),
                    }),
                ),
            ],
            ..Default::default()
        };
        let rom = Rom::new(img);
        let Kind::Table(t) = &def.element("spark").unwrap().kind else {
            panic!()
        };
        let v = def.table(&rom, t).unwrap();
        assert_eq!(v.y, vec!["1000.0", "2000.0"]);
        assert_eq!(v.x, vec!["a", "b", "c"]);
        assert_eq!(v.z, vec![vec![5.0, 10.0, 15.0], vec![20.0, 25.0, 30.0]]);
        assert_eq!(
            def.covering(0x24..0x25)
                .iter()
                .map(|e| e.title.as_str())
                .collect::<Vec<_>>(),
            vec!["spark map"]
        );
    }

    #[test]
    fn column_major_and_strides_place_cells() {
        let def = Definition::default();
        let mut d = data(0x100, 2, 3, "X");
        assert_eq!(def.cell_offset(&d, 1, 2).unwrap(), 0x105);
        d.column_major = true;
        assert_eq!(
            def.cell_offset(&d, 1, 2).unwrap(),
            0x105,
            "col 2 * 2 rows + row 1"
        );
        assert_eq!(def.cell_offset(&d, 1, 0).unwrap(), 0x101);
        d.column_major = false;
        d.major_stride_bits = 64; // each row starts 8 bytes after the last
        assert_eq!(def.cell_offset(&d, 1, 1).unwrap(), 0x109);
        assert!(def.cell_offset(&d, 2, 0).is_err());
    }

    #[test]
    fn edits_go_through_the_formula_and_respect_limits() {
        let def = Definition::default();
        let mut d = data(0, 1, 2, "0.375*X+12");
        d.max = Some(100.0);
        let mut rom = Rom::new(vec![0; 2]);
        assert_eq!(
            def.set(&mut rom, &d, 0, 1, 15.1).unwrap(),
            15.0,
            "the nearest value the cell can hold"
        );
        assert_eq!(rom.bytes(), &[0, 8]);
        assert!(
            def.set(&mut rom, &d, 0, 0, 120.0).is_err(),
            "above the definition's maximum"
        );
        assert_eq!(rom.changes(), vec![1..2]);
    }

    #[test]
    fn addresses_below_a_subtracted_base_are_errors() {
        let def = Definition {
            base_offset: 0x8000,
            base_subtract: true,
            ..Default::default()
        };
        assert_eq!(def.offset(0x8010).unwrap(), 0x10);
        assert!(def.offset(0x10).is_err());
    }
}
