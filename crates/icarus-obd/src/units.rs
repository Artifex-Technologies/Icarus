//! US customary units beside the metric ones the standards give (SAE J1979: km/h, °C, kPa...), for what Icarus shows an
//! owner in the US: one table, used by the window's units setting and by the reports.

/// `v` in `unit`, in its US customary unit, when it has one.
pub fn customary(unit: &str, v: f64) -> Option<(f64, &'static str)> {
    Some(match unit {
        "km/h" => (v * 0.621371, "mph"),
        "km" => (v * 0.621371, "mi"),
        "°C" => (v * 1.8 + 32.0, "°F"),
        "kPa" => (v * 0.145038, "psi"),
        "L/h" => (v * 0.264172, "gal/h"),
        "Nm" => (v * 0.737562, "lb-ft"),
        _ => return None,
    })
}

/// "44.7 mph (72.0 km/h)": US first and metric after, to `dec` decimals; a unit with no US form alone ("13.8 V").
pub fn both(unit: &str, v: f64, dec: usize) -> String {
    match customary(unit, v) {
        Some((c, cu)) => format!("{c:.dec$} {cu} ({v:.dec$} {unit})"),
        None => format!("{v:.dec$} {unit}").trim_end().to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn us_first_metric_after() {
        assert_eq!(both("km/h", 72.0, 1), "44.7 mph (72.0 km/h)");
        assert_eq!(both("°C", 90.0, 1), "194.0 °F (90.0 °C)");
        assert_eq!(both("V", 13.8, 1), "13.8 V");
        assert_eq!(both("", 3.0, 0), "3");
        assert!(customary("rpm", 800.0).is_none());
    }
}
