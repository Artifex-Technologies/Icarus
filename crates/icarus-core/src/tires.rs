//! Tires and gears: what a different tire size does to the speedometer and to the gearing, and which axle ratio brings the
//! original feel back. Pure arithmetic, no vehicle needed. (Changing the vehicle's own tire size or axle ratio setting is a
//! per-make configuration or calibration change, done through a profile once Icarus writes configuration.)
//!
//! Sizes: metric "265/70R16" (width mm / aspect % / rim inches, "P" or "LT" in front allowed) or flotation "33x12.50R15"
//! (diameter inches x width).

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tire {
    /// Overall diameter in inches.
    pub diameter: f64,
    /// Section width in inches.
    pub width: f64,
    pub rim: f64,
}

impl Tire {
    pub fn parse(s: &str) -> Result<Tire, String> {
        let t = s.trim().to_uppercase().replace(' ', "");
        let t = t.trim_start_matches("LT").trim_start_matches('P');
        let bad = || format!("{s:?} is not a tire size like 265/70R16 or 33x12.50R15");
        if let Some((w, rest)) = t.split_once('/') {
            let (aspect, rim) = rest
                .split_once('R')
                .or_else(|| rest.split_once('-'))
                .ok_or_else(bad)?;
            let rim = rim.trim_end_matches(|c: char| c.is_ascii_alphabetic());
            let (w, aspect, rim): (f64, f64, f64) = (
                w.parse().map_err(|_| bad())?,
                aspect.parse().map_err(|_| bad())?,
                rim.parse().map_err(|_| bad())?,
            );
            if !(100.0..=500.0).contains(&w)
                || !(20.0..=100.0).contains(&aspect)
                || !(10.0..=30.0).contains(&rim)
            {
                return Err(bad());
            }
            let side = w * aspect / 100.0 / 25.4;
            return Ok(Tire {
                diameter: rim + 2.0 * side,
                width: w / 25.4,
                rim,
            });
        }
        if let Some((d, rest)) = t.split_once('X') {
            let (w, rim) = rest.split_once('R').ok_or_else(bad)?;
            let (d, w, rim): (f64, f64, f64) = (
                d.parse().map_err(|_| bad())?,
                w.parse().map_err(|_| bad())?,
                rim.parse().map_err(|_| bad())?,
            );
            if !(20.0..=50.0).contains(&d)
                || !(5.0..=20.0).contains(&w)
                || !(10.0..=30.0).contains(&rim)
                || d <= rim
            {
                return Err(bad());
            }
            return Ok(Tire {
                diameter: d,
                width: w,
                rim,
            });
        }
        Err(bad())
    }

    /// Revolutions per mile (from the circumference; real tires squat a little, so the maker's figure is a few % higher).
    pub fn revs_per_mile(&self) -> f64 {
        63360.0 / (std::f64::consts::PI * self.diameter)
    }
}

/// Axle ratios commonly offered for trucks and off-road builds.
pub const COMMON_RATIOS: [f64; 13] = [
    3.31, 3.42, 3.55, 3.73, 3.91, 4.10, 4.30, 4.56, 4.63, 4.88, 5.13, 5.29, 5.38,
];

#[derive(Clone, Debug, PartialEq)]
pub struct Change {
    pub stock: Tire,
    pub new: Tire,
    /// New diameter / stock diameter.
    pub ratio: f64,
    /// The speedometer shows `shown` when the vehicle really does `actual` (both at `speed`): it reads low with bigger tires.
    pub shown_at_actual: f64,
    pub actual_at_shown: f64,
    /// The axle ratio that feels like the stock one with the new tires, and the nearest common ratios.
    pub effective_gear: Option<f64>,
    pub equivalent_gear: Option<f64>,
    pub suggested: Vec<f64>,
    /// With a regear (`installed` axle ratio): what it acts like on the new tires, engine speed against stock at the same
    /// road speed (1.166 = 16.6 % more rpm), and the speedometer if the vehicle measures speed at the transmission output
    /// (the reading is multiplied by this; at the wheels only the tires count).
    pub installed_acts_like: Option<f64>,
    pub rpm_vs_stock: Option<f64>,
    pub shaft_reading_factor: Option<f64>,
}

/// What changing from `stock` to `new` does at `speed` (any unit), with the stock axle ratio `gear` and the `installed` one
/// after a regear, if given.
pub fn change(
    stock: Tire,
    new: Tire,
    speed: f64,
    gear: Option<f64>,
    installed: Option<f64>,
) -> Change {
    let ratio = new.diameter / stock.diameter;
    let equivalent = gear.map(|g| g * ratio);
    let mut suggested: Vec<f64> = match equivalent {
        Some(e) => {
            let mut v = COMMON_RATIOS.to_vec();
            v.sort_by(|a, b| (a - e).abs().partial_cmp(&(b - e).abs()).unwrap());
            v.truncate(2);
            v.sort_by(|a, b| a.partial_cmp(b).unwrap());
            v
        }
        None => vec![],
    };
    suggested.dedup();
    Change {
        stock,
        new,
        ratio,
        shown_at_actual: speed / ratio,
        actual_at_shown: speed * ratio,
        effective_gear: gear.map(|g| g / ratio),
        equivalent_gear: equivalent,
        suggested,
        installed_acts_like: installed.map(|i| i / ratio),
        rpm_vs_stock: gear.zip(installed).map(|(g, i)| i / g / ratio),
        shaft_reading_factor: gear.zip(installed).map(|(g, i)| i / g / ratio),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sizes() {
        let a = Tire::parse("265/70R16").unwrap();
        assert!((a.diameter - 30.61).abs() < 0.01, "{}", a.diameter);
        let b = Tire::parse("LT285/75R16").unwrap();
        assert!((b.diameter - 32.83).abs() < 0.01, "{}", b.diameter);
        let c = Tire::parse("33x12.50R15").unwrap();
        assert_eq!((c.diameter, c.rim), (33.0, 15.0));
        assert!((Tire::parse("p265/65r17").unwrap().diameter - 30.56).abs() < 0.01);
        assert!((a.revs_per_mile() - 658.9).abs() < 0.5);
        for bad in [
            "",
            "265/70",
            "33x12.50",
            "26/70R16",
            "265/70R60",
            "15x12.5R15",
            "abc",
        ] {
            assert!(Tire::parse(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn bigger_tires_on_a_truck() {
        let stock = Tire::parse("265/70R16").unwrap();
        let new = Tire::parse("285/75R16").unwrap();
        let c = change(stock, new, 65.0, Some(3.909), None);
        assert!((c.ratio - 1.0727).abs() < 0.001);
        assert!(
            (c.shown_at_actual - 60.6).abs() < 0.1,
            "at a real 65 the speedometer shows about 60.6"
        );
        assert!(
            (c.actual_at_shown - 69.7).abs() < 0.1,
            "showing 65 is really about 69.7"
        );
        assert!(
            (c.effective_gear.unwrap() - 3.644).abs() < 0.01,
            "the 3.91 now feels like a 3.64"
        );
        assert!((c.equivalent_gear.unwrap() - 4.193).abs() < 0.01);
        assert_eq!(c.suggested, vec![4.10, 4.30]);
        assert!(change(stock, new, 65.0, None, None).suggested.is_empty());
        // a regear past the stock feel: 3.91 to 4.88 with 265/65R17 to 285/70R17
        let c = change(
            Tire::parse("P265/65R17").unwrap(),
            Tire::parse("LT285/70R17").unwrap(),
            65.0,
            Some(3.91),
            Some(4.88),
        );
        assert!((c.ratio - 1.0703).abs() < 0.001, "{}", c.ratio);
        assert!((c.installed_acts_like.unwrap() - 4.560).abs() < 0.005);
        assert!((c.rpm_vs_stock.unwrap() - 1.1661).abs() < 0.001);
        assert!((c.equivalent_gear.unwrap() - 4.185).abs() < 0.005);
    }
}
