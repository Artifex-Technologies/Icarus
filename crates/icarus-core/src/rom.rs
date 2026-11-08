//! A ROM image: the bytes read from an ECU (or a file), kept twice: as read, and as edited. Nothing here talks to an ECU; it is
//! the file a tune is made in. Every edit can be compared with the original, so a tune always says exactly what it changed.

use std::fmt;

/// How one stored number is laid out: its size, sign, byte order, and whether it is an IEEE float.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Cell {
    /// 8, 16, 32 (or 64 for a double).
    pub bits: u8,
    pub signed: bool,
    /// Least significant byte first (Intel, Infineon C16x, most Bosch/Siemens); false is Motorola order (SH, PowerPC, 68k).
    pub little_endian: bool,
    pub float: bool,
}

impl Cell {
    pub const U8: Cell = Cell {
        bits: 8,
        signed: false,
        little_endian: false,
        float: false,
    };

    pub fn bytes(&self) -> usize {
        (self.bits as usize).div_ceil(8)
    }

    /// The smallest and largest raw integer the cell holds.
    pub fn range(&self) -> (i64, i64) {
        let b = self.bits.min(63) as u32;
        if self.signed {
            (-(1i64 << (b - 1)), (1i64 << (b - 1)) - 1)
        } else {
            (0, if b >= 63 { i64::MAX } else { (1i64 << b) - 1 })
        }
    }

    pub fn check(&self) -> Result<(), RomError> {
        let ok = if self.float {
            matches!(self.bits, 32 | 64)
        } else {
            matches!(self.bits, 8 | 16 | 32)
        };
        if ok {
            Ok(())
        } else {
            Err(RomError::Layout(format!(
                "{}-bit {} cells are not supported",
                self.bits,
                if self.float { "float" } else { "integer" }
            )))
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum RomError {
    /// The address (and length) does not fit in the image.
    OutOfRange {
        address: usize,
        len: usize,
        size: usize,
    },
    Layout(String),
}

impl fmt::Display for RomError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            RomError::OutOfRange { address, len, size } => {
                write!(f, "0x{address:X} (+{len} bytes) is outside the image (0x{size:X} bytes): wrong definition for this file?")
            }
            RomError::Layout(m) => f.write_str(m),
        }
    }
}

impl std::error::Error for RomError {}

#[derive(Clone, Debug)]
pub struct Rom {
    original: Vec<u8>,
    data: Vec<u8>,
}

impl Rom {
    pub fn new(bytes: Vec<u8>) -> Rom {
        Rom {
            original: bytes.clone(),
            data: bytes,
        }
    }

    pub fn len(&self) -> usize {
        self.data.len()
    }

    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// The edited image.
    pub fn bytes(&self) -> &[u8] {
        &self.data
    }

    /// The image as it was read.
    pub fn original(&self) -> &[u8] {
        &self.original
    }

    fn span(&self, address: usize, len: usize) -> Result<std::ops::Range<usize>, RomError> {
        match address.checked_add(len) {
            Some(end) if end <= self.data.len() => Ok(address..end),
            _ => Err(RomError::OutOfRange {
                address,
                len,
                size: self.data.len(),
            }),
        }
    }

    /// The raw number stored at `address` (in the edited image).
    pub fn read(&self, address: usize, cell: Cell) -> Result<f64, RomError> {
        read_from(&self.data, self.span(address, cell.bytes())?, cell)
    }

    /// The raw number at `address` as it was before any edit.
    pub fn read_original(&self, address: usize, cell: Cell) -> Result<f64, RomError> {
        read_from(&self.original, self.span(address, cell.bytes())?, cell)
    }

    /// Stores a raw number (rounded and kept inside the cell's range for an integer cell).
    pub fn write(&mut self, address: usize, cell: Cell, raw: f64) -> Result<(), RomError> {
        cell.check()?;
        let r = self.span(address, cell.bytes())?;
        let n = cell.bytes();
        let bits: u64 = if cell.float {
            if n == 4 {
                (raw as f32).to_bits() as u64
            } else {
                raw.to_bits()
            }
        } else {
            let (lo, hi) = cell.range();
            (raw.round().clamp(lo as f64, hi as f64) as i64) as u64
        };
        for k in 0..n {
            let byte = (bits >> (8 * k)) as u8;
            let at = if cell.little_endian { k } else { n - 1 - k };
            self.data[r.start + at] = byte;
        }
        Ok(())
    }

    /// The byte ranges that differ from the original, merged when they touch.
    pub fn changes(&self) -> Vec<std::ops::Range<usize>> {
        diff(&self.original, &self.data)
    }

    /// Puts the bytes in `r` back as they were read (an element's span: undoing one map).
    pub fn revert_range(&mut self, r: std::ops::Range<usize>) {
        let end = r.end.min(self.data.len());
        if r.start < end {
            self.data[r.start..end].copy_from_slice(&self.original[r.start..end]);
        }
    }

    /// Puts every byte back as it was read.
    pub fn revert(&mut self) {
        self.data.copy_from_slice(&self.original);
    }
}

fn read_from(buf: &[u8], r: std::ops::Range<usize>, cell: Cell) -> Result<f64, RomError> {
    cell.check()?;
    let b = &buf[r];
    let mut v: u64 = 0;
    for k in 0..b.len() {
        let byte = if cell.little_endian {
            b[b.len() - 1 - k]
        } else {
            b[k]
        };
        v = (v << 8) | byte as u64;
    }
    if cell.float {
        return Ok(if b.len() == 4 {
            f32::from_bits(v as u32) as f64
        } else {
            f64::from_bits(v)
        });
    }
    if cell.signed {
        let shift = 64 - 8 * b.len() as u32;
        return Ok((((v << shift) as i64) >> shift) as f64);
    }
    Ok(v as f64)
}

/// The ranges where two images differ (compared over the shorter length; a length difference is one more range at the end).
pub fn diff(a: &[u8], b: &[u8]) -> Vec<std::ops::Range<usize>> {
    let mut out: Vec<std::ops::Range<usize>> = Vec::new();
    let n = a.len().min(b.len());
    let mut i = 0;
    while i < n {
        if a[i] != b[i] {
            let start = i;
            while i < n && a[i] != b[i] {
                i += 1;
            }
            match out.last_mut() {
                Some(last) if last.end == start => last.end = i,
                _ => out.push(start..i),
            }
        } else {
            i += 1;
        }
    }
    if a.len() != b.len() {
        out.push(n..a.len().max(b.len()));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const BE16: Cell = Cell {
        bits: 16,
        signed: false,
        little_endian: false,
        float: false,
    };
    const LE16S: Cell = Cell {
        bits: 16,
        signed: true,
        little_endian: true,
        float: false,
    };

    #[test]
    fn byte_order_and_sign() {
        let rom = Rom::new(vec![0x12, 0x34, 0xFF, 0xFE, 0x80]);
        assert_eq!(rom.read(0, BE16).unwrap(), 0x1234 as f64);
        assert_eq!(
            rom.read(
                0,
                Cell {
                    little_endian: true,
                    ..BE16
                }
            )
            .unwrap(),
            0x3412 as f64
        );
        assert_eq!(rom.read(2, LE16S).unwrap(), -257.0, "0xFEFF signed");
        assert_eq!(
            rom.read(
                4,
                Cell {
                    signed: true,
                    ..Cell::U8
                }
            )
            .unwrap(),
            -128.0
        );
        assert!(
            matches!(rom.read(4, BE16), Err(RomError::OutOfRange { .. })),
            "past the end is an error, never a guess"
        );
        assert!(rom.read(0, Cell { bits: 12, ..BE16 }).is_err());
    }

    #[test]
    fn writes_round_trip_and_stay_in_range() {
        let mut rom = Rom::new(vec![0; 12]);
        rom.write(0, LE16S, -300.0).unwrap();
        assert_eq!(rom.read(0, LE16S).unwrap(), -300.0);
        rom.write(2, Cell::U8, 999.0).unwrap();
        assert_eq!(rom.read(2, Cell::U8).unwrap(), 255.0, "clamped to the cell");
        let f = Cell {
            bits: 32,
            signed: false,
            little_endian: false,
            float: true,
        };
        rom.write(4, f, 14.7).unwrap();
        assert!((rom.read(4, f).unwrap() - 14.7).abs() < 1e-5);
        assert_eq!(&rom.bytes()[4..8], &14.7f32.to_be_bytes());
        assert_eq!(
            rom.read_original(0, LE16S).unwrap(),
            0.0,
            "the original is kept"
        );
        assert_eq!(rom.changes(), vec![0..3, 4..8]);
        rom.revert();
        assert!(rom.changes().is_empty());
    }

    #[test]
    fn diff_merges_runs_and_notes_length() {
        assert_eq!(diff(&[1, 2, 3, 4], &[1, 9, 9, 4]), vec![1..3]);
        assert_eq!(diff(&[1, 2], &[1, 2, 3]), vec![2..3]);
        assert!(diff(&[5; 8], &[5; 8]).is_empty());
    }
}
