//! Reading a VIN (ISO 3779, and the US rules in 49 CFR 565): where the vehicle was built, its maker's prefix, its model year
//! and whether the check digit adds up. The VIN is what Icarus uses to pick a vehicle profile.
//!
//! The 17 characters: 1-3 world manufacturer identifier (WMI; the first says the country or region), 4-8 the vehicle (model,
//! body, engine), 9 the check digit, 10 the model year, 11 the plant, 12-17 the serial number.

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VinInfo {
    pub vin: String,
    pub wmi: String,
    pub region: &'static str,
    /// The model year, when position 10 holds a year code.
    pub year: Option<u16>,
    /// None when the VIN holds letters no real VIN uses (I, O, Q), so the sum means nothing.
    pub check_digit_ok: Option<bool>,
    pub problems: Vec<String>,
}

const YEAR_LETTERS: &str = "ABCDEFGHJKLMNPRSTVWXY";

fn value(c: char) -> Option<u32> {
    match c {
        '0'..='9' => c.to_digit(10),
        'A' | 'J' => Some(1),
        'B' | 'K' | 'S' => Some(2),
        'C' | 'L' | 'T' => Some(3),
        'D' | 'M' | 'U' => Some(4),
        'E' | 'N' | 'V' => Some(5),
        'F' | 'W' => Some(6),
        'G' | 'P' | 'X' => Some(7),
        'H' | 'Y' => Some(8),
        'R' | 'Z' => Some(9),
        _ => None,
    }
}

/// The check digit position 9 should hold ('0'-'9' or 'X'), from the other 16 characters.
pub fn check_digit(vin: &str) -> Option<char> {
    const WEIGHTS: [u32; 17] = [8, 7, 6, 5, 4, 3, 2, 10, 0, 9, 8, 7, 6, 5, 4, 3, 2];
    let mut sum = 0;
    for (i, c) in vin.chars().enumerate() {
        sum += value(c)? * WEIGHTS.get(i)?;
    }
    let r = sum % 11;
    Some(if r == 10 {
        'X'
    } else {
        char::from_digit(r, 10).unwrap()
    })
}

fn region(c: char) -> &'static str {
    match c {
        '1' | '4' | '5' | '7' => "United States",
        '2' => "Canada",
        '3' => "Mexico",
        '6' => "Australia",
        '8' => "South America",
        '9' => "Brazil",
        'J' => "Japan",
        'K' => "South Korea",
        'L' => "China",
        'M' => "India, Indonesia or Thailand",
        'N' => "Turkey or Iran",
        'S' => "United Kingdom",
        'T' => "Switzerland, Czechia or Hungary",
        'U' => "Romania or Denmark",
        'V' => "France or Spain",
        'W' => "Germany",
        'X' => "Russia or the Netherlands",
        'Y' => "Sweden, Finland or Belgium",
        'Z' => "Italy",
        'A'..='H' => "Africa",
        _ => "unknown",
    }
}

pub fn decode(vin: &str) -> Result<VinInfo, String> {
    let v: String = vin.trim().to_uppercase();
    if v.len() != 17 || !v.chars().all(|c| c.is_ascii_alphanumeric()) {
        return Err(format!(
            "a VIN has 17 letters and digits; {vin:?} has {}",
            v.chars().count()
        ));
    }
    let chars: Vec<char> = v.chars().collect();
    let mut problems = Vec::new();
    let bad: Vec<char> = chars
        .iter()
        .copied()
        .filter(|c| matches!(c, 'I' | 'O' | 'Q'))
        .collect();
    if !bad.is_empty() {
        problems.push(format!(
            "holds {}: real VINs never use I, O or Q (they look like 1 and 0)",
            bad.iter().collect::<String>()
        ));
    }
    let check_digit_ok = if bad.is_empty() {
        check_digit(&v).map(|c| c == chars[8])
    } else {
        None
    };
    if check_digit_ok == Some(false) {
        problems.push("the check digit (position 9) does not add up: a typo, or a VIN from outside North America that does not use one".into());
    }
    let y = chars[9];
    let year = if let Some(i) = YEAR_LETTERS.find(y) {
        // the codes repeat every 30 years; since 2010 a letter in position 7 marks the later cycle (vehicles under 10,000 lb)
        let base = if chars[6].is_ascii_alphabetic() {
            2010
        } else {
            1980
        };
        Some(base + i as u16)
    } else if let Some(d) = y.to_digit(10).filter(|d| *d > 0) {
        Some(2000 + d as u16)
    } else {
        problems.push(format!("position 10 ({y}) is not a model-year code"));
        None
    };
    Ok(VinInfo {
        wmi: v[..3].to_string(),
        region: region(chars[0]),
        year,
        check_digit_ok,
        problems,
        vin: v,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_textbook_vin() {
        // the worked example from the check digit rules (a 1989 Mack truck)
        let i = decode("1M8GDM9AXKP042788").unwrap();
        assert_eq!(check_digit("1M8GDM9AXKP042788"), Some('X'));
        assert_eq!(
            (i.check_digit_ok, i.region, i.year, i.wmi.as_str()),
            (Some(true), "United States", Some(1989), "1M8")
        );
        assert!(i.problems.is_empty());
    }

    #[test]
    fn model_years_and_mistakes() {
        assert_eq!(
            decode("3TMCZ5AN0HM000000").unwrap().year,
            Some(2017),
            "H with a letter in position 7"
        );
        assert_eq!(decode("JF1VA2E60K9800000").unwrap().year, Some(2019));
        assert_eq!(
            decode("1G1ZZ5ST0Y0000000").unwrap().year,
            Some(2030),
            "Y with a letter in position 7 is the next cycle"
        );
        assert_eq!(decode("1G1JC1240Y7000000").unwrap().year, Some(2000));
        assert_eq!(decode("1G1JC124577000000").unwrap().year, Some(2007));
        let sim = decode("5TFICARUSSIM00001").unwrap();
        assert_eq!(sim.check_digit_ok, None);
        assert!(sim.problems[0].contains("never use I, O or Q"));
        let typo = decode("1M8GDM9A1KP042788").unwrap();
        assert_eq!(typo.check_digit_ok, Some(false));
        assert!(decode("1M8GDM9AXKP04278").is_err());
        assert!(decode("1M8GDM9AXKP04278-").is_err());
    }
}
