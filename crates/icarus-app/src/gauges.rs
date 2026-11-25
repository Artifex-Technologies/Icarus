//! Trail gauges (ICA-27): a few standard values as big readouts for driving off-road, each colored when it wants watching
//! (amber) or stopping (red). The values come in the vehicle's units (°C, km/h, V); the bands are written in them too and
//! the page shows them in the units chosen (`logs::shown`).

/// How a reading stands.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Level {
    Fine,
    /// Worth watching: getting hot, the battery not charging.
    Watch,
    /// Stop and look: overheating, the voltage too low to keep the modules up.
    Act,
}

/// A gauge: the standard value (OBD-II PID), and its bands (None: no band that way).
pub struct Gauge {
    pub pid: u8,
    pub low_act: Option<f64>,
    pub low_watch: Option<f64>,
    pub high_watch: Option<f64>,
    pub high_act: Option<f64>,
    /// What the colors mean, for the tile's hover text.
    pub about: &'static str,
}

const fn gauge(
    pid: u8,
    low: (Option<f64>, Option<f64>),
    high: (Option<f64>, Option<f64>),
    about: &'static str,
) -> Gauge {
    Gauge {
        pid,
        low_act: low.0,
        low_watch: low.1,
        high_watch: high.0,
        high_act: high.1,
        about,
    }
}

/// The gauges in their order on the page. The bands are general ones for a petrol engine, not this vehicle's own limits.
pub const GAUGES: &[Gauge] = &[
    gauge(0x05, (None, None), (Some(110.0), Some(118.0)), "amber from 110 °C (230 °F), red from 118 °C (245 °F): ease off and let it cool, then check the coolant"),
    gauge(0x5C, (None, None), (Some(125.0), Some(135.0)), "amber from 125 °C (257 °F), red from 135 °C (275 °F): the oil is thinning; slow down"),
    gauge(0x0F, (None, None), (Some(60.0), Some(75.0)), "amber from 60 °C (140 °F), red from 75 °C (167 °F): hot air in, less power, more knock"),
    gauge(0x42, (Some(11.5), Some(12.0)), (Some(15.0), Some(15.5)), "amber under 12.0 V or over 15.0 V, red under 11.5 V or over 15.5 V: not charging, or charging too hard"),
    gauge(0x0C, (None, None), (None, None), "engine speed"),
    gauge(0x0D, (None, None), (None, None), "road speed as the vehicle reads it (corrected for the tires below it, when they are set on the Garage page)"),
    gauge(0x04, (None, None), (None, None), "how hard the engine works, against what it could do"),
    gauge(0x2F, (Some(10.0), Some(20.0)), (None, None), "amber under 20 %, red under 10 %"),
    gauge(0x46, (None, None), (None, None), "outside air"),
    gauge(0x33, (None, None), (None, None), "air pressure outside: lower the higher you are (about 1 kPa, 0.15 psi, less every 100 m or 330 ft)"),
];

/// The gauges this vehicle can feed (`offered`: the PIDs it gives), in the page's order.
pub fn gauge_pids(offered: &[u8]) -> Vec<u8> {
    GAUGES
        .iter()
        .map(|g| g.pid)
        .filter(|p| offered.contains(p))
        .collect()
}

/// How `v` (in the vehicle's units) stands on `pid`'s gauge.
pub fn level(pid: u8, v: f64) -> Level {
    let Some(g) = GAUGES.iter().find(|g| g.pid == pid) else {
        return Level::Fine;
    };
    let below = |b: Option<f64>| b.is_some_and(|b| v < b);
    let above = |b: Option<f64>| b.is_some_and(|b| v >= b);
    if below(g.low_act) || above(g.high_act) {
        Level::Act
    } else if below(g.low_watch) || above(g.high_watch) {
        Level::Watch
    } else {
        Level::Fine
    }
}

/// The lowest and highest of the readings kept (None when there are none).
pub fn range(h: &std::collections::VecDeque<(f64, f64)>) -> Option<(f64, f64)> {
    h.iter().map(|x| x.1).fold(None, |acc, v| match acc {
        None => Some((v, v)),
        Some((lo, hi)) => Some((lo.min(v), hi.max(v))),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_gauges_a_vehicle_feeds_in_the_pages_order() {
        assert_eq!(
            gauge_pids(&[0x0D, 0x05, 0x01, 0x42, 0x0C]),
            [0x05, 0x42, 0x0C, 0x0D]
        );
        assert!(gauge_pids(&[0x01, 0x03]).is_empty());
        // every gauge is a value Icarus decodes
        assert!(GAUGES
            .iter()
            .all(|g| icarus_obd::j1979::pid(g.pid).is_some()));
    }

    #[test]
    fn the_bands() {
        assert_eq!(
            [level(0x05, 90.0), level(0x05, 110.0), level(0x05, 119.0)],
            [Level::Fine, Level::Watch, Level::Act]
        );
        assert_eq!(
            [
                level(0x42, 14.1),
                level(0x42, 11.9),
                level(0x42, 11.0),
                level(0x42, 15.2),
                level(0x42, 16.0)
            ],
            [
                Level::Fine,
                Level::Watch,
                Level::Act,
                Level::Watch,
                Level::Act
            ]
        );
        assert_eq!(
            [level(0x2F, 50.0), level(0x2F, 15.0), level(0x2F, 5.0)],
            [Level::Fine, Level::Watch, Level::Act]
        );
        assert_eq!(level(0x0C, 6500.0), Level::Fine);
        assert_eq!(level(0x99, 1.0), Level::Fine);
    }

    #[test]
    fn lowest_and_highest() {
        let h: std::collections::VecDeque<(f64, f64)> =
            [(0.0, 13.9), (1.0, 14.4), (2.0, 12.1)].into();
        assert_eq!(range(&h), Some((12.1, 14.4)));
        assert_eq!(range(&Default::default()), None);
    }
}
