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

/// The standard values (SAE J1979 PIDs) that are absolute pressures, above a vacuum rather than above the outside air:
/// intake manifold (0B), barometric (33), evaporative system vapor (53) and fuel rail (59) pressure. Fuel pressure (0A)
/// and the gauge rail pressure (23) are above the outside air.
pub const ABSOLUTE_PIDS: &[u8] = &[0x0B, 0x33, 0x53, 0x59];

/// `both` for standard value `pid`: an absolute pressure says so beside psi ("7.0 psi absolute (48.0 kPa)"), since psi in
/// the US is read as above the outside air (a tire gauge, boost) and 7 psi of manifold pressure would read as boost.
pub fn both_pid(pid: u8, unit: &str, v: f64, dec: usize) -> String {
    let s = both(unit, v, dec);
    if unit == "kPa" && ABSOLUTE_PIDS.contains(&pid) {
        s.replacen(" psi (", " psi absolute (", 1)
    } else {
        s
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

    #[test]
    fn absolute_pressures_say_so() {
        assert_eq!(
            both_pid(0x0B, "kPa", 48.0, 1),
            "7.0 psi absolute (48.0 kPa)"
        );
        assert_eq!(
            both_pid(0x33, "kPa", 101.0, 1),
            "14.6 psi absolute (101.0 kPa)"
        );
        // fuel pressure is gauge: plain psi
        assert_eq!(both_pid(0x0A, "kPa", 300.0, 1), "43.5 psi (300.0 kPa)");
        assert_eq!(both_pid(0x0D, "km/h", 72.0, 1), "44.7 mph (72.0 km/h)");
        // every absolute PID is a kPa value in the table, and says absolute in its name or explanation
        for p in ABSOLUTE_PIDS {
            let d = crate::j1979::pid(*p).unwrap();
            assert_eq!(d.unit, "kPa", "{p:02X}");
            assert!(
                (d.name.to_lowercase() + d.about).contains("bsolute")
                    || d.name.contains("MAP")
                    || d.name.contains("Barometric"),
                "{p:02X} {}",
                d.name
            );
        }
    }
}
