//! On-board test results (OBD-II service 06): for each emissions monitor the engine module keeps the value its last test
//! measured and the limits it must stay within. It shows how close a part is to failing before any code sets (a catalyst
//! at 90 % of its limit), and the misfire counts per cylinder, the best clue to a rough idle.
//!
//! The CAN form (ISO 15765-4, vehicles from about 2008) is read: each answer holds blocks of nine bytes, monitor ID (MID),
//! test ID (TID), unit and scaling ID (UASID), value, minimum, maximum. Pass or fail compares the raw numbers, which needs no
//! scaling; a value in real units is shown only for the scalings Icarus knows for certain.

use crate::elm::ObdError;
use crate::j1979::bitmap;
use crate::link::Link;

#[derive(Clone, Debug, PartialEq)]
pub struct TestResult {
    pub mid: u8,
    pub tid: u8,
    pub uasid: u8,
    pub value: i64,
    pub min: i64,
    pub max: i64,
}

impl TestResult {
    pub fn passed(&self) -> bool {
        self.min <= self.value && self.value <= self.max
    }

    /// Value, minimum and maximum in real units, when the scaling is one Icarus knows.
    pub fn scaled(&self) -> Option<(f64, f64, f64, &'static str)> {
        let (k, unit) = scaling(self.uasid & 0x7F)?;
        Some((
            self.value as f64 * k,
            self.min as f64 * k,
            self.max as f64 * k,
            unit,
        ))
    }
}

/// Unit and scaling IDs (SAE J1979): only those Icarus is sure of; others are shown raw.
fn scaling(id: u8) -> Option<(f64, &'static str)> {
    Some(match id {
        0x01 => (1.0, ""),
        0x0A => (0.000122, "V"),
        0x0B => (0.001, "V"),
        0x0C => (0.01, "V"),
        0x10 => (1.0, "ms"),
        0x11 => (100.0, "ms"),
        0x12 => (1.0, "s"),
        0x24 => (1.0, "counts"),
        _ => return None,
    })
}

const SENSOR: [&str; 16] = [
    "bank 1 sensor 1",
    "bank 1 sensor 2",
    "bank 1 sensor 3",
    "bank 1 sensor 4",
    "bank 2 sensor 1",
    "bank 2 sensor 2",
    "bank 2 sensor 3",
    "bank 2 sensor 4",
    "bank 3 sensor 1",
    "bank 3 sensor 2",
    "bank 3 sensor 3",
    "bank 3 sensor 4",
    "bank 4 sensor 1",
    "bank 4 sensor 2",
    "bank 4 sensor 3",
    "bank 4 sensor 4",
];

/// What a monitor ID tests.
pub fn monitor_name(mid: u8) -> String {
    match mid {
        0x01..=0x10 => format!("Oxygen sensor, {}", SENSOR[(mid - 0x01) as usize]),
        0x21..=0x24 => format!("Catalyst, bank {}", mid - 0x20),
        0x31..=0x34 => format!("EGR system, bank {}", mid - 0x30),
        0x35..=0x38 => format!("Variable valve timing, bank {}", mid - 0x34),
        0x39 => "Evaporative system: large leak (fuel cap off)".into(),
        0x3A => "Evaporative system: 0.090 inch leak".into(),
        0x3B => "Evaporative system: 0.040 inch leak".into(),
        0x3C => "Evaporative system: 0.020 inch leak".into(),
        0x3D => "Evaporative purge flow".into(),
        0x41..=0x50 => format!("Oxygen sensor heater, {}", SENSOR[(mid - 0x41) as usize]),
        0x61..=0x64 => format!("Heated catalyst, bank {}", mid - 0x60),
        0x71..=0x74 => format!("Secondary air system {}", mid - 0x70),
        0x81..=0x84 => format!("Fuel system, bank {}", mid - 0x80),
        0x85..=0x88 => format!("Boost pressure control, bank {}", mid - 0x84),
        0x90 | 0x91 => format!("NOx adsorber, bank {}", mid - 0x8F),
        0x98 | 0x99 => format!("NOx catalyst, bank {}", mid - 0x97),
        0xA1 => "Misfire, all cylinders".into(),
        0xA2..=0xAD => format!("Misfire, cylinder {}", mid - 0xA1),
        0xB0 | 0xB1 => format!("Particulate filter, bank {}", mid - 0xAF),
        _ => format!("Monitor {mid:02X} (the maker's own)"),
    }
}

/// What a test ID measures, for the standard ones Icarus knows (the rest are the maker's own).
pub fn test_name(mid: u8, tid: u8) -> String {
    let misfire = (0xA1..=0xAD).contains(&mid);
    match (misfire, tid) {
        (true, 0x0B) => "misfires, average of the last 10 drive cycles".into(),
        (true, 0x0C) => "misfires, this or the last drive cycle".into(),
        (false, 0x01) if mid <= 0x10 => "rich-to-lean switch threshold".into(),
        (false, 0x02) if mid <= 0x10 => "lean-to-rich switch threshold".into(),
        (false, 0x05) if mid <= 0x10 => "rich-to-lean switch time".into(),
        (false, 0x06) if mid <= 0x10 => "lean-to-rich switch time".into(),
        (false, 0x07) if mid <= 0x10 => "lowest voltage during the test".into(),
        (false, 0x08) if mid <= 0x10 => "highest voltage during the test".into(),
        (_, t) if t >= 0x80 => format!("test {t:02X} (the maker's own)"),
        (_, t) => format!("test {t:02X}"),
    }
}

/// The blocks of a CAN service 06 answer.
pub fn parse(data: &[u8]) -> Vec<TestResult> {
    if data.first() != Some(&0x46) {
        return vec![];
    }
    data[1..]
        .chunks_exact(9)
        .filter(|b| b[1] != 0) // TID 0 would be a support map, not a result
        .map(|b| {
            let signed = b[2] & 0x80 != 0;
            let n = |hi: u8, lo: u8| {
                let v = u16::from_be_bytes([hi, lo]);
                if signed {
                    v as i16 as i64
                } else {
                    v as i64
                }
            };
            TestResult {
                mid: b[0],
                tid: b[1],
                uasid: b[2],
                value: n(b[3], b[4]),
                min: n(b[5], b[6]),
                max: n(b[7], b[8]),
            }
        })
        .collect()
}

/// Every result the engine module (7E8, else the first module) gives.
pub fn read(elm: &mut dyn Link) -> Result<Vec<TestResult>, ObdError> {
    if !elm.protocol().is_can() {
        return Err(ObdError::Adapter(
            "on-board test results are read on CAN vehicles only so far".into(),
        ));
    }
    let mut mids = Vec::new();
    let mut base = 0u8;
    loop {
        let a = match elm.request(&[0x06, base]) {
            Ok(a) => a,
            Err(ObdError::NoData) => break,
            Err(e) => return Err(e),
        };
        let Some(x) = a
            .iter()
            .find(|x| x.from == "7E8")
            .or(a.first())
            .filter(|x| x.data.len() >= 6 && x.data[..2] == [0x46, base])
        else {
            break;
        };
        let found = bitmap(base, &x.data[2..6]);
        let more = found.contains(&base.wrapping_add(0x20));
        mids.extend(found.into_iter().filter(|m| m % 0x20 != 0));
        if !more || base >= 0xE0 {
            break;
        }
        base += 0x20;
    }
    let mut out = Vec::new();
    for mid in mids {
        match elm.request(&[0x06, mid]) {
            Ok(a) => {
                if let Some(x) = a.iter().find(|x| x.from == "7E8").or(a.first()) {
                    out.extend(parse(&x.data).into_iter().filter(|r| r.mid == mid));
                }
            }
            Err(ObdError::NoData) => {}
            Err(e) => return Err(e),
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elm::Elm;
    use crate::sim::SimCar;

    #[test]
    fn blocks_names_and_pass_fail() {
        let r = parse(&[
            0x46, 0x21, 0x80, 0x01, 0x01, 0x20, 0x00, 0x00, 0x02, 0xA0, 0x21, 0x81, 0x01, 0x03,
            0x00, 0x00, 0x00, 0x02, 0x00,
        ]);
        assert_eq!(r.len(), 2);
        assert!(
            r[0].passed() && !r[1].passed(),
            "0x120 within 0..0x2A0; 0x300 above 0x200"
        );
        assert_eq!(r[0].scaled().unwrap().0, 288.0);
        let s = parse(&[0x46, 0x01, 0x01, 0x8A, 0xFF, 0x38, 0xFF, 0x00, 0x00, 0x10]);
        assert_eq!(
            (s[0].value, s[0].min),
            (-200, -256),
            "signed scaling (0x8A)"
        );
        assert_eq!(monitor_name(0x05), "Oxygen sensor, bank 2 sensor 1");
        assert_eq!(monitor_name(0xA4), "Misfire, cylinder 3");
        assert_eq!(monitor_name(0x42), "Oxygen sensor heater, bank 1 sensor 2");
        assert_eq!(
            test_name(0xA4, 0x0C),
            "misfires, this or the last drive cycle"
        );
        assert!(parse(&[0x7F, 0x06, 0x12]).is_empty());
    }

    #[test]
    fn the_simulated_engine_reports_its_tests() {
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        let r = read(&mut elm).unwrap();
        let cat = r.iter().find(|x| x.mid == 0x21).unwrap();
        assert!(cat.passed());
        let cyl3 = r.iter().find(|x| x.mid == 0xA4 && x.tid == 0x0C).unwrap();
        assert_eq!(cyl3.scaled().unwrap().3, "counts");
        assert!(cyl3.value > 0, "the simulated cylinder 3 misfires a little");
        assert_eq!(
            r.iter()
                .filter(|x| (0xA2..=0xA7).contains(&x.mid) && x.tid == 0x0B)
                .count(),
            6,
            "six cylinders"
        );
    }
}
