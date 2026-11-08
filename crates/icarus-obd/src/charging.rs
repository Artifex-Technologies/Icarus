//! Battery and charging: the voltage the engine module sees (PID 42, else the adapter's reading of the port), with engine
//! speed, over some seconds, judged for the battery fitted. Engine running: is the alternator charging enough for this
//! battery (an AGM wants a higher absorption voltage than a flooded one) and not too much. Engine off (after sitting an
//! hour, so the surface charge has gone): about how charged it is.
//!
//! The ranges are the usual guidance for 12 V lead-acid batteries; the battery maker's figures come first. The module's
//! reading is a little below the battery's terminals (wiring), and many newer vehicles lower the alternator voltage on
//! purpose once the battery is full ("smart charging"), so one low reading is not a fault: watch a whole drive.

use crate::elm::ObdError;
use crate::j1979;
use crate::link::Link;
use std::time::{Duration, Instant};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Battery {
    Agm,
    Flooded,
}

impl Battery {
    pub fn parse(s: &str) -> Option<Battery> {
        match s.to_lowercase().as_str() {
            "agm" => Some(Battery::Agm),
            "flooded" | "lead-acid" | "wet" => Some(Battery::Flooded),
            _ => None,
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            Battery::Agm => "AGM",
            Battery::Flooded => "flooded (wet) lead-acid",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Check {
    pub running: bool,
    pub samples: usize,
    pub avg: f64,
    pub min: f64,
    pub max: f64,
    pub verdict: String,
    pub ok: bool,
}

/// Judges voltages (engine running or not) for a battery.
pub fn judge(volts: &[f64], running: bool, battery: Battery) -> Option<Check> {
    if volts.is_empty() {
        return None;
    }
    let avg = volts.iter().sum::<f64>() / volts.len() as f64;
    let (min, max) = volts
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), v| {
            (a.min(*v), b.max(*v))
        });
    let agm = battery == Battery::Agm;
    let (verdict, ok) = if running {
        let (good_from, good_to) = if agm { (14.1, 14.8) } else { (13.8, 14.5) };
        if avg < 13.0 {
            ("Not charging (or resting on purpose: some vehicles lower the alternator voltage when the battery is full). Watch a whole drive; if it stays under 13 V, check the alternator, its belt and the battery cables.".to_string(), false)
        } else if avg < good_from {
            (
                format!(
                    "Charging lightly ({avg:.2} V). {}",
                    if agm {
                        "An AGM is kept up at this voltage but not fully charged (it wants about 14.4-14.8 V to finish): a smart charger with an AGM mode now and then keeps it at its best."
                    } else {
                        "Enough to keep a flooded battery up; a full charge takes longer."
                    }
                ),
                true,
            )
        } else if avg <= good_to {
            (format!("Good for {} ({avg:.2} V).", battery.name()), true)
        } else {
            (format!("High ({avg:.2} V): over about {good_to} V a {} battery is overcharged and dries out. Check the voltage regulator.", battery.name()), false)
        }
    } else {
        // resting voltage to state of charge (after sitting)
        let steps: [(f64, &str); 4] = if agm {
            [
                (12.8, "full"),
                (12.6, "about 75 %"),
                (12.4, "about 50 %"),
                (12.2, "about 25 %"),
            ]
        } else {
            [
                (12.6, "full"),
                (12.4, "about 75 %"),
                (12.2, "about 50 %"),
                (12.0, "about 25 %"),
            ]
        };
        let state = steps
            .iter()
            .find(|(v, _)| avg >= *v)
            .map(|(_, s)| *s)
            .unwrap_or("low");
        let ok = avg >= steps[1].0;
        (format!("Resting at {avg:.2} V: {state} for {} (read it after the vehicle has sat an hour; just after driving the reading is high). {}", battery.name(), if ok { "" } else { "Charge it soon: a lead-acid battery left low loses capacity for good." }), ok)
    };
    Some(Check {
        running,
        samples: volts.len(),
        avg,
        min,
        max,
        verdict: verdict.trim_end().to_string(),
        ok,
    })
}

/// Samples for `seconds` (every half second) and judges.
pub fn check(elm: &mut dyn Link, seconds: u64, battery: Battery) -> Result<Check, String> {
    let e = |x: ObdError| x.to_string();
    let first = |r: Vec<j1979::Reading>| {
        r.iter()
            .find(|x| x.from == "7E8")
            .or(r.first())
            .map(|x| x.value)
    };
    let mut volts = Vec::new();
    let mut rpm_max: f64 = 0.0;
    let end = Instant::now() + Duration::from_secs(seconds.max(2));
    while Instant::now() < end {
        let t0 = Instant::now();
        let v = match j1979::read(elm, 0x42) {
            Ok(r) => first(r),
            Err(ObdError::NoData) => None,
            Err(x) => return Err(e(x)),
        };
        let v = match v {
            Some(v) => Some(v),
            None => elm.voltage(),
        };
        if let Some(v) = v {
            volts.push(v);
        }
        if let Ok(r) = j1979::read(elm, 0x0C) {
            rpm_max = rpm_max.max(first(r).unwrap_or(0.0));
        }
        std::thread::sleep(Duration::from_millis(500).saturating_sub(t0.elapsed()));
    }
    judge(&volts, rpm_max >= 400.0, battery).ok_or_else(|| "no voltage could be read".into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elm::Elm;

    #[test]
    fn verdicts() {
        let agm = |v: f64, run: bool| judge(&[v, v], run, Battery::Agm).unwrap();
        assert!(agm(14.4, true).ok && agm(14.4, true).verdict.starts_with("Good"));
        assert!(
            agm(13.6, true).verdict.starts_with("Charging lightly")
                && agm(13.6, true).verdict.contains("AGM mode")
        );
        assert!(!agm(15.2, true).ok);
        assert!(!agm(12.4, true).ok, "under 13 V running");
        assert!(agm(12.85, false).verdict.contains("full"));
        assert!(!agm(12.3, false).ok && agm(12.3, false).verdict.contains("Charge it soon"));
        let flooded = judge(&[12.65], false, Battery::Flooded).unwrap();
        assert!(
            flooded.verdict.contains("full"),
            "12.65 V is full for a flooded battery"
        );
        assert!(judge(&[], true, Battery::Agm).is_none());
        assert_eq!(Battery::parse("AGM"), Some(Battery::Agm));
    }

    #[test]
    fn the_simulated_vehicle() {
        let mut elm = Elm::connect(Box::new(crate::sim::SimCar::new())).unwrap();
        let c = check(&mut elm, 2, Battery::Agm).unwrap();
        assert!(c.running && c.ok && (c.avg - 14.1).abs() < 0.01, "{c:?}");
        let mut elm = Elm::connect(Box::new(crate::sim::SimCar::engine_off())).unwrap();
        let c = check(&mut elm, 2, Battery::Agm).unwrap();
        assert!(!c.running && (c.avg - 12.65).abs() < 0.01, "{c:?}");
    }
}
