//! What a recording stops asking for (ICA-86): after its first minute, a value the vehicle never answered, and a value that
//! copied an earlier one at every reading while both changed (a vehicle may list PID 46 ambient and never fill it, or answer
//! PID 3D catalyst bank 2 with a copy of 3C). Each request costs a slot in the recording's rate, so a dead value slows every
//! live one.
//!
//! Two values that only sat at the same constant (fuel trims at 0 in open loop, two temperatures at the morning's cold) are
//! not copies: a copy has to change and be followed each time. Nothing is dropped when nothing answered at all: then the
//! adapter or the vehicle is the problem, not the values.

/// How long a recording watches before it drops anything, and how many readings it needs in that time.
pub const WATCH_S: f64 = 60.0;
pub const MIN_READINGS: u32 = 10;

/// Why a value left the recording.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Why {
    /// The vehicle never answered it.
    NoAnswer,
    /// The same as this earlier value at every reading.
    SameAs(u8),
}

/// Watches a recording's first minute: [`Pruner::see`] each reading, then [`Pruner::decide`] once.
#[derive(Clone, Debug)]
pub struct Pruner {
    pids: Vec<u8>,
    answered: Vec<bool>,
    /// The first value each PID gave, and whether it ever gave another.
    first: Vec<Option<String>>,
    changed: Vec<bool>,
    /// For each PID, the earlier PIDs (by index) it has equalled at every reading so far.
    twins: Vec<Vec<usize>>,
    readings: u32,
    started: Option<f64>,
    decided: bool,
}

/// A value as the recording writes it (two decimals), so "the same" is what the log would show.
fn cell(v: Option<f64>) -> Option<String> {
    v.map(|v| format!("{v:.2}"))
}

impl Pruner {
    pub fn new(pids: &[u8]) -> Pruner {
        Pruner {
            pids: pids.to_vec(),
            answered: vec![false; pids.len()],
            first: vec![None; pids.len()],
            changed: vec![false; pids.len()],
            twins: (0..pids.len()).map(|i| (0..i).collect()).collect(),
            readings: 0,
            started: None,
            decided: false,
        }
    }

    /// One reading at `t` seconds: the value of each PID (in the order given to `new`), None for no answer.
    pub fn see(&mut self, t: f64, values: &[Option<f64>]) {
        if self.decided {
            return;
        }
        self.started.get_or_insert(t);
        self.readings += 1;
        let cells: Vec<Option<String>> = values.iter().map(|v| cell(*v)).collect();
        for (i, c) in cells.iter().enumerate().take(self.pids.len()) {
            let Some(c) = c else { continue };
            self.answered[i] = true;
            match &self.first[i] {
                None => self.first[i] = Some(c.clone()),
                Some(f) if f != c => self.changed[i] = true,
                Some(_) => {}
            }
        }
        for i in 0..self.pids.len() {
            let mine = cells.get(i).cloned().flatten();
            self.twins[i].retain(|&j| cells.get(j).cloned().flatten() == mine);
        }
    }

    /// Once the first minute is over (with enough readings): what to stop asking for, in the order given; None before then
    /// and after the first answer.
    pub fn decide(&mut self, t: f64) -> Option<Vec<(u8, Why)>> {
        let start = self.started?;
        if self.decided || t - start < WATCH_S || self.readings < MIN_READINGS {
            return None;
        }
        self.decided = true;
        if !self.answered.iter().any(|a| *a) {
            return Some(Vec::new());
        }
        let mut out = Vec::new();
        let mut dropped = vec![false; self.pids.len()];
        for i in 0..self.pids.len() {
            if !self.answered[i] {
                dropped[i] = true;
                out.push((self.pids[i], Why::NoAnswer));
                continue;
            }
            // a copy of an earlier value that stays: both changed, and this one followed it each time
            if !self.changed[i] {
                continue;
            }
            if let Some(&j) = self.twins[i]
                .iter()
                .find(|&&j| !dropped[j] && self.changed[j])
            {
                dropped[i] = true;
                out.push((self.pids[i], Why::SameAs(self.pids[j])));
            }
        }
        Some(out)
    }
}

/// The words for what was dropped, one line each ("PID 46 (Ambient air temperature): no answer in the first minute").
pub fn words(dropped: &[(u8, Why)]) -> Vec<String> {
    let name = |p: u8| {
        crate::j1979::pid(p)
            .map(|d| format!("PID {p:02X} ({})", d.name))
            .unwrap_or_else(|| format!("PID {p:02X}"))
    };
    dropped
        .iter()
        .map(|(p, why)| match why {
            Why::NoAnswer => format!(
                "{}: no answer in the first minute, no longer asked",
                name(*p)
            ),
            Why::SameAs(q) => format!(
                "{}: the same as {} at every reading, no longer asked",
                name(*p),
                name(*q)
            ),
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A made-up minute: ambient never answers, catalyst bank 2 follows bank 1 as both warm, the fuel trims sit at 0
    /// together (open loop) and so stay, and engine speed changes on its own. Nothing is decided before the minute is over.
    #[test]
    fn a_dead_value_and_a_copy_leave_after_the_first_minute() {
        let pids = [0x0C, 0x46, 0x3C, 0x3D, 0x06, 0x08];
        let mut p = Pruner::new(&pids);
        for k in 0..120 {
            let t = k as f64 * 0.5;
            let cat = 100.0 + k as f64 * 2.0;
            p.see(
                t,
                &[
                    Some(700.0 + (k % 7) as f64),
                    None,
                    Some(cat),
                    Some(cat),
                    Some(0.0),
                    Some(0.0),
                ],
            );
            if t < WATCH_S {
                assert_eq!(p.decide(t), None, "before the minute");
            }
        }
        let got = p.decide(60.0).unwrap();
        assert_eq!(got, vec![(0x46, Why::NoAnswer), (0x3D, Why::SameAs(0x3C))]);
        assert_eq!(p.decide(61.0), None, "once");
        let w = words(&got);
        assert!(
            w[0].starts_with("PID 46 (") && w[0].contains("no answer"),
            "{w:?}"
        );
        assert!(w[1].contains("PID 3C"), "{w:?}");
    }

    /// A value that answered once is kept; one that differed from its twin once is kept; a slow adapter with too few readings
    /// decides nothing yet; a vehicle that answered nothing at all keeps everything.
    #[test]
    fn what_stays() {
        let mut p = Pruner::new(&[0x05, 0x5C, 0x46]);
        for k in 0..30 {
            let t = k as f64 * 2.5;
            let c = 20.0 + k as f64;
            let oil = if k == 17 { c + 1.0 } else { c };
            let amb = if k == 3 { Some(-6.0) } else { None };
            p.see(t, &[Some(c), Some(oil), amb]);
        }
        assert_eq!(p.decide(75.0), Some(Vec::new()));

        let mut slow = Pruner::new(&[0x46]);
        for k in 0..5 {
            slow.see(k as f64 * 20.0, &[None]);
        }
        assert_eq!(slow.decide(80.0), None, "five readings are not enough");

        let mut silent = Pruner::new(&[0x0C, 0x46]);
        for k in 0..20 {
            silent.see(k as f64 * 4.0, &[None, None]);
        }
        assert_eq!(silent.decide(80.0), Some(Vec::new()));
    }

    /// With the simulated vehicle: PID 3D is not one it answers, so a minute of readings drops it and keeps 3C and 46.
    #[test]
    fn the_simulated_vehicle_loses_what_it_does_not_answer() {
        let mut elm = crate::elm::Elm::connect(Box::new(crate::sim::SimCar::new())).unwrap();
        let pids = [0x0C, 0x3C, 0x3D, 0x46];
        let mut p = Pruner::new(&pids);
        for k in 0..MIN_READINGS + 2 {
            let all = crate::j1979::read_many(&mut elm, &pids).unwrap();
            let values: Vec<Option<f64>> = pids
                .iter()
                .map(|q| {
                    all.iter()
                        .find(|(x, _)| x == q)
                        .and_then(|(_, r)| r.first())
                        .map(|r| r.value)
                })
                .collect();
            p.see(k as f64 * 6.0, &values);
        }
        assert_eq!(
            p.decide(WATCH_S + 6.0 * 12.0),
            Some(vec![(0x3D, Why::NoAnswer)])
        );
    }
}
