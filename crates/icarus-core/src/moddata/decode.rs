//! What the bytes of an answer mean: a value's bits taken out in its order, signed where it is, then one formula that holds
//! every source's scaling exactly, `value = (raw * mul + bias) / div + add` (DDT2000: Step, Offset, DivideBy; OBDb: mul,
//! div, add; DBC: factor, offset), then the range (clamped and marked "at limit") and the "no reading" bounds.

use super::{Order, ValueDef, ValueKind};

/// A value as read, in its definition's own unit (conversion for display is `units::shown`).
#[derive(Clone, Debug, PartialEq)]
pub enum Shown {
    Number(f64),
    State(String),
    Text(String),
    Hex(String),
    /// The value fell on a "no reading" bound.
    NoReading,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Decoded {
    /// The bits as read (before sign and scaling); 0 for text and hex.
    pub raw: u64,
    pub shown: Shown,
    /// The range clamp acted: the reading is pinned at a limit, not a real value.
    pub at_limit: bool,
}

fn bit_at(data: &[u8], pos: usize) -> u64 {
    ((data[pos / 8] >> (7 - pos % 8)) & 1) as u64
}

fn msb(data: &[u8], start: usize, bits: usize) -> u64 {
    (start..start + bits).fold(0u64, |acc, p| (acc << 1) | bit_at(data, p))
}

/// The value's bits, in its order. `data` starts after the service byte and the identifier's echo.
pub fn raw_bits(v: &ValueDef, data: &[u8]) -> Result<u64, String> {
    let need = v.end_byte();
    if data.len() < need {
        return Err(format!(
            "{} needs {need} bytes after the echo; the answer has {}",
            v.id,
            data.len()
        ));
    }
    let bits = v.bits as usize;
    let start = v.byte * 8 + v.bit as usize;
    Ok(match v.order {
        Order::Msb => msb(data, start, bits),
        Order::LsbBytes if bits <= 8 => msb(data, start, bits),
        Order::LsbBytes => {
            let mut d = data.to_vec();
            let end = (v.byte + bits.div_ceil(8)).min(d.len());
            d[v.byte..end].reverse();
            msb(&d, start, bits)
        }
        Order::DdtLittle => {
            let n = (v.bit as usize + bits).div_ceil(8);
            let whole = data[v.byte..v.byte + n]
                .iter()
                .fold(0u128, |a, b| (a << 8) | *b as u128);
            let value = (whole >> v.bit) & ((1u128 << bits) - 1);
            let len = bits.div_ceil(8);
            let be = &value.to_be_bytes()[16 - len..];
            be.iter().rev().fold(0u64, |a, b| (a << 8) | *b as u64)
        }
    })
}

/// Two's complement at `bits`.
fn signed(raw: u64, bits: u16) -> i64 {
    if bits >= 64 {
        raw as i64
    } else if raw & (1 << (bits - 1)) != 0 {
        raw as i64 - (1i64 << bits)
    } else {
        raw as i64
    }
}

/// Text with 00, FF and spaces trimmed; None when it holds characters that are not printable.
fn text(b: &[u8]) -> Option<String> {
    let t: String = b
        .iter()
        .filter(|c| **c != 0 && **c != 0xFF)
        .map(|&c| c as char)
        .collect();
    t.chars()
        .all(|c| c.is_ascii_graphic() || c == ' ')
        .then(|| t.trim().to_string())
}

fn hex(b: &[u8]) -> String {
    b.iter()
        .map(|x| format!("{x:02X}"))
        .collect::<Vec<_>>()
        .join(" ")
}

/// One value from an answer's data (the bytes after the service byte and the identifier's echo).
pub fn decode(v: &ValueDef, data: &[u8]) -> Result<Decoded, String> {
    if matches!(v.kind, ValueKind::Text | ValueKind::Hex) {
        let need = v.end_byte();
        if data.len() < need {
            return Err(format!(
                "{} needs {need} bytes after the echo; the answer has {}",
                v.id,
                data.len()
            ));
        }
        let b = &data[v.byte..need];
        let shown = match (v.kind, text(b)) {
            (ValueKind::Text, Some(t)) => Shown::Text(t),
            _ => Shown::Hex(hex(b)),
        };
        return Ok(Decoded {
            raw: 0,
            shown,
            at_limit: false,
        });
    }
    let raw = raw_bits(v, data)?;
    let n: f64 = if v.signed {
        signed(raw, v.bits) as f64
    } else {
        raw as f64
    };
    if v.kind == ValueKind::State {
        let key = if v.signed {
            signed(raw, v.bits).to_string()
        } else {
            raw.to_string()
        };
        let label = v
            .states
            .iter()
            .find(|(k, _)| k.trim().parse::<i128>().ok() == key.parse::<i128>().ok())
            .map(|(_, l)| l.clone())
            .unwrap_or_else(|| format!("{key} (not in the list)"));
        return Ok(Decoded {
            raw,
            shown: Shown::State(label),
            at_limit: false,
        });
    }
    let mut x = (n * v.mul + v.bias) / v.div + v.add;
    let mut at_limit = false;
    if let (Some(lo), Some(hi)) = (v.min, v.max) {
        if hi > lo {
            if x < lo {
                (x, at_limit) = (lo, true);
            } else if x > hi {
                (x, at_limit) = (hi, true);
            }
        }
    }
    let eps = f64::EPSILON;
    if v.null_below.is_some_and(|b| x <= b + eps) || v.null_above.is_some_and(|b| x >= b - eps) {
        return Ok(Decoded {
            raw,
            shown: Shown::NoReading,
            at_limit: false,
        });
    }
    Ok(Decoded {
        raw,
        shown: Shown::Number(x),
        at_limit,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(byte: usize, bit: u8, bits: u16, order: Order) -> ValueDef {
        ValueDef {
            bit,
            order,
            ..ValueDef::new("X", "x", byte, bits)
        }
    }

    fn raw(v: &ValueDef, d: &[u8]) -> u64 {
        raw_bits(v, d).unwrap()
    }

    #[test]
    fn msb_aligned_non_aligned_and_across_bytes() {
        let d = [0xAB, 0xCD, 0xEF];
        assert_eq!(raw(&v(0, 0, 8, Order::Msb), &d), 0xAB);
        assert_eq!(raw(&v(1, 0, 16, Order::Msb), &d), 0xCDEF);
        // bits 4.. of byte 0 (0xB = 1011), then 1100 of byte 1: 1011 1100
        assert_eq!(raw(&v(0, 4, 8, Order::Msb), &d), 0xBC);
        assert_eq!(raw(&v(0, 0, 1, Order::Msb), &d), 1);
        assert_eq!(raw(&v(0, 1, 1, Order::Msb), &d), 0);
        // 12 bits from bit 2 of byte 1: CD EF = 1100 1101 1110 1111 -> 00 1101 1110 11 = 0x37B
        assert_eq!(raw(&v(1, 2, 12, Order::Msb), &d), 0x37B);
        assert!(raw_bits(&v(2, 0, 16, Order::Msb), &d).is_err(), "too short");
    }

    #[test]
    fn lsb_bytes_at_16_and_24_bits() {
        let d = [0x34, 0x12, 0x56, 0x99];
        assert_eq!(raw(&v(0, 0, 16, Order::LsbBytes), &d), 0x1234);
        assert_eq!(raw(&v(0, 0, 24, Order::LsbBytes), &d), 0x561234);
        assert_eq!(raw(&v(1, 0, 16, Order::LsbBytes), &d), 0x5612);
        assert_eq!(
            raw(&v(0, 0, 8, Order::LsbBytes), &d),
            0x34,
            "8 bits or fewer: msb"
        );
    }

    #[test]
    fn ddt_little_aligned_and_not() {
        let d = [0x34, 0x12, 0xF0];
        // aligned: ordinary little endian
        assert_eq!(raw(&v(0, 0, 16, Order::DdtLittle), &d), 0x1234);
        assert_eq!(raw(&v(0, 0, 8, Order::DdtLittle), &d), 0x34);
        // not aligned, worked by hand: bytes 34 12 F0 cover bits 4..20; 0x3412F0 >> 4 = 0x3412F, & 0xFFFF = 0x412F, bytes
        // 41 2F reversed: 0x2F41
        assert_eq!(raw(&v(0, 4, 16, Order::DdtLittle), &d), 0x2F41);
        // 4 bits from bit 2 of 0x34 (0011 0100): 0x34 >> 2 = 0x0D, & 0xF = 0xD
        assert_eq!(raw(&v(0, 2, 4, Order::DdtLittle), &d), 0xD);
    }

    #[test]
    fn signed_at_12_bits_and_the_formula() {
        let mut x = v(0, 0, 12, Order::Msb);
        x.signed = true;
        // 0xFFF = -1, 0x800 = -2048
        assert_eq!(
            decode(&x, &[0xFF, 0xF0]).unwrap().shown,
            Shown::Number(-1.0)
        );
        assert_eq!(
            decode(&x, &[0x80, 0x00]).unwrap().shown,
            Shown::Number(-2048.0)
        );
        // bias before the divide and add after it give different results on the same raw value
        let mut a = v(0, 0, 8, Order::Msb);
        a.bias = 10.0;
        a.div = 2.0;
        let mut b = a.clone();
        (b.bias, b.add) = (0.0, 10.0);
        assert_eq!(decode(&a, &[100]).unwrap().shown, Shown::Number(55.0));
        assert_eq!(decode(&b, &[100]).unwrap().shown, Shown::Number(60.0));
        let mut t = v(0, 0, 8, Order::Msb);
        (t.mul, t.div, t.add) = (5.0, 4.0, -40.0);
        assert_eq!(decode(&t, &[200]).unwrap().shown, Shown::Number(210.0));
    }

    #[test]
    fn clamp_marks_at_limit_and_null_bounds_mean_no_reading() {
        let mut x = v(0, 0, 8, Order::Msb);
        (x.min, x.max) = (Some(0.0), Some(100.0));
        let d = decode(&x, &[150]).unwrap();
        assert_eq!((d.shown, d.at_limit), (Shown::Number(100.0), true));
        assert!(!decode(&x, &[50]).unwrap().at_limit);
        // only a max: no clamp
        x.min = None;
        assert_eq!(decode(&x, &[150]).unwrap().shown, Shown::Number(150.0));
        let mut n = v(0, 0, 8, Order::Msb);
        n.add = -40.0;
        (n.null_below, n.null_above) = (Some(-40.0), Some(215.0));
        assert_eq!(decode(&n, &[0]).unwrap().shown, Shown::NoReading);
        assert_eq!(decode(&n, &[255]).unwrap().shown, Shown::NoReading);
        assert_eq!(decode(&n, &[130]).unwrap().shown, Shown::Number(90.0));
    }

    #[test]
    fn states_hit_and_miss() {
        let mut s = v(0, 7, 1, Order::Msb);
        s.kind = ValueKind::State;
        s.states = [("0".into(), "off".into()), ("1".into(), "on".into())].into();
        assert_eq!(
            decode(&s, &[0x01]).unwrap().shown,
            Shown::State("on".into())
        );
        assert_eq!(
            decode(&s, &[0x00]).unwrap().shown,
            Shown::State("off".into())
        );
        let mut g = v(0, 0, 8, Order::Msb);
        g.kind = ValueKind::State;
        g.states = [("1".into(), "first".into())].into();
        assert_eq!(
            decode(&g, &[7]).unwrap().shown,
            Shown::State("7 (not in the list)".into())
        );
    }

    #[test]
    fn text_is_trimmed_and_hex_shows_bytes() {
        let mut t = v(0, 0, 48, Order::Msb);
        t.kind = ValueKind::Text;
        assert_eq!(
            decode(&t, b" AB12\0\xFF").unwrap().shown,
            Shown::Text("AB12".into())
        );
        assert_eq!(
            decode(&t, &[0x01, 0x41, 0, 0, 0, 0]).unwrap().shown,
            Shown::Hex("01 41 00 00 00 00".into()),
            "unprintable text shows as hex"
        );
        let mut h = v(1, 0, 16, Order::Msb);
        h.kind = ValueKind::Hex;
        assert_eq!(
            decode(&h, &[0, 0xBE, 0xEF]).unwrap().shown,
            Shown::Hex("BE EF".into())
        );
        assert!(decode(&h, &[0, 0xBE]).is_err());
    }
}
