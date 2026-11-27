//! SAE J1939, the CAN language of heavy-duty vehicles (diesel trucks and buses, motorhomes on a truck chassis, farm and
//! construction equipment): modules broadcast parameter groups (PGNs) on 29-bit IDs, usually at 250 kbit/s, and report
//! their active faults in DM1 as pairs of SPN (what: a parameter) and FMI (how it failed). Icarus only listens here (`icarus
//! bus watch --j1939`, or an adapter on protocol A): nothing is sent onto the bus.
//!
//! What is decoded follows public descriptions of SAE J1939-21 (the 29-bit ID, the transport protocol), J1939-71 (a few
//! common engine values) and J1939-73 (DM1); the SAE documents themselves are not used. The SPN and source-address names
//! are a handful of common ones from public references, not checked against the SAE lists: any other shows as its number.

use std::collections::{BTreeMap, HashMap};

/// What a 29-bit J1939 ID says: priority, parameter group, the sender's address and, for a message to one address
/// (PDU1: PF below 240), that address.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Id {
    pub priority: u8,
    pub pgn: u32,
    pub source: u8,
    pub dest: Option<u8>,
}

pub fn id(raw: u32) -> Id {
    let pf = (raw >> 16) & 0xFF;
    let ps = (raw >> 8) & 0xFF;
    let dp = (raw >> 24) & 1;
    let edp = (raw >> 25) & 1;
    let pdu1 = pf < 240;
    Id {
        priority: ((raw >> 26) & 7) as u8,
        pgn: (edp << 17) | (dp << 16) | (pf << 8) | if pdu1 { 0 } else { ps },
        source: (raw & 0xFF) as u8,
        dest: pdu1.then_some(ps as u8),
    }
}

/// The transport protocol's connection management (TP.CM) and data transfer (TP.DT) groups.
pub const TP_CM: u32 = 0xEC00;
pub const TP_DT: u32 = 0xEB00;
/// DM1: the active faults (J1939-73).
pub const DM1: u32 = 0xFECA;

/// A message bigger than 8 bytes being put together from TP.DT packets: its group, size and the packets so far.
struct Transfer {
    pgn: u32,
    size: usize,
    packets: Vec<Option<[u8; 7]>>,
}

/// One active fault: SPN (the parameter), FMI (how it failed), how often it occurred, and whether the sender uses one of
/// the old SPN numberings (the "conversion method" bit: the number may then not be the standard one).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Fault {
    pub spn: u32,
    pub fmi: u8,
    pub occurrences: u8,
    pub old_numbering: bool,
}

impl Fault {
    pub fn describe(&self) -> String {
        let name = spn_name(self.spn)
            .map(|n| format!(" {n}"))
            .unwrap_or_default();
        format!(
            "SPN {}{name}, FMI {} ({}), {} time(s){}",
            self.spn,
            self.fmi,
            fmi_name(self.fmi),
            self.occurrences,
            if self.old_numbering {
                " (an older SPN numbering: the number may differ)"
            } else {
                ""
            }
        )
    }
}

/// A DM1 message: the warning lamps (each `Some(on)` or `None` when not available) and the active faults.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Dm1 {
    pub mil: Option<bool>,
    pub red_stop: Option<bool>,
    pub amber_warning: Option<bool>,
    pub protect: Option<bool>,
    pub faults: Vec<Fault>,
}

/// Reads a DM1 message's bytes: two bytes of lamps (each two bits: off, on, error, not available), then four bytes per fault.
/// "No fault" is sent as SPN 0 FMI 0 (or all ones), which is left out.
pub fn dm1(data: &[u8]) -> Option<Dm1> {
    if data.len() < 2 {
        return None;
    }
    let lamp = |shift: u8| match (data[0] >> shift) & 3 {
        0 => Some(false),
        1 => Some(true),
        _ => None,
    };
    let mut d = Dm1 {
        mil: lamp(6),
        red_stop: lamp(4),
        amber_warning: lamp(2),
        protect: lamp(0),
        faults: vec![],
    };
    for c in data[2..].as_chunks::<4>().0 {
        let spn = u32::from(c[0]) | (u32::from(c[1]) << 8) | (u32::from(c[2] & 0xE0) << 11);
        let fmi = c[2] & 0x1F;
        if (spn == 0 && fmi == 0) || *c == [0xFF; 4] {
            continue;
        }
        d.faults.push(Fault {
            spn,
            fmi,
            occurrences: c[3] & 0x7F,
            old_numbering: c[3] & 0x80 != 0,
        });
    }
    Some(d)
}

/// FMI: how a parameter failed (J1939-73's list).
pub fn fmi_name(fmi: u8) -> &'static str {
    match fmi {
        0 => "above normal, most severe",
        1 => "below normal, most severe",
        2 => "erratic, intermittent or incorrect",
        3 => "voltage above normal or shorted high",
        4 => "voltage below normal or shorted low",
        5 => "current below normal or open circuit",
        6 => "current above normal or grounded circuit",
        7 => "mechanical system not responding or out of adjustment",
        8 => "abnormal frequency, pulse width or period",
        9 => "abnormal update rate",
        10 => "abnormal rate of change",
        11 => "root cause not known",
        12 => "bad intelligent device or component",
        13 => "out of calibration",
        14 => "special instructions",
        15 => "above normal, least severe",
        16 => "above normal, moderately severe",
        17 => "below normal, least severe",
        18 => "below normal, moderately severe",
        19 => "received network data in error",
        20 => "data drifted high",
        21 => "data drifted low",
        31 => "condition exists",
        _ => "reserved",
    }
}

/// A few common SPNs by name (public references; any other is shown by its number).
pub fn spn_name(spn: u32) -> Option<&'static str> {
    Some(match spn {
        84 => "wheel-based vehicle speed",
        91 => "accelerator pedal position",
        100 => "engine oil pressure",
        102 => "intake manifold (boost) pressure",
        105 => "intake manifold temperature",
        108 => "barometric pressure",
        110 => "engine coolant temperature",
        111 => "engine coolant level",
        157 => "injector metering rail pressure",
        158 => "keyswitch battery voltage",
        168 => "battery voltage",
        171 => "ambient air temperature",
        174 => "fuel temperature",
        175 => "engine oil temperature",
        183 => "engine fuel rate",
        190 => "engine speed",
        639 => "J1939 network (CAN) #1",
        1761 => "diesel exhaust fluid tank level",
        3251 => "diesel particulate filter differential pressure",
        _ => return None,
    })
}

/// A few common source addresses by name (public references).
pub fn source_name(sa: u8) -> String {
    match sa {
        0x00 => "engine #1".into(),
        0x01 => "engine #2".into(),
        0x03 => "transmission #1".into(),
        0x0B => "brakes".into(),
        0x17 => "instrument cluster".into(),
        0x21 => "body controller".into(),
        0x3D => "exhaust emission controller".into(),
        0xF9 => "diagnostic tool".into(),
        _ => format!("address {sa:02X}"),
    }
}

/// A value read from one of the common groups: what it is (with its SPN) and the number, in metric units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Value {
    pub name: &'static str,
    pub spn: u32,
    pub value: f64,
    pub unit: &'static str,
}

/// The common engine values a group carries (J1939-71, as public descriptions give them): engine speed (EEC1), coolant
/// temperature (ET1), wheel-based speed (CCVS1), fuel rate (LFE1), oil pressure (EFL/P1). "Not available" and "error"
/// (the top of each range) are left out.
pub fn values(pgn: u32, d: &[u8]) -> Vec<Value> {
    let b = |i: usize| d.get(i).copied().filter(|v| *v <= 0xFA).map(f64::from);
    let w = |i: usize| {
        let v = u16::from(*d.get(i)?) | (u16::from(*d.get(i + 1)?) << 8);
        (v <= 0xFAFF).then_some(f64::from(v))
    };
    let mut out = Vec::new();
    let mut put = |name, spn, v: Option<f64>, unit| {
        if let Some(value) = v {
            out.push(Value {
                name,
                spn,
                value,
                unit,
            });
        }
    };
    match pgn {
        0xF004 => put("engine speed", 190, w(3).map(|v| v * 0.125), "rpm"),
        0xFEEE => put("coolant temperature", 110, b(0).map(|v| v - 40.0), "°C"),
        0xFEF1 => put("vehicle speed", 84, w(1).map(|v| v / 256.0), "km/h"),
        0xFEF2 => put("fuel rate", 183, w(0).map(|v| v * 0.05), "L/h"),
        0xFEEF => put("oil pressure", 100, b(3).map(|v| v * 4.0), "kPa"),
        _ => {}
    }
    out
}

/// A value in US units where it has one (the owner's: mph, °F, psi, gal/h), with the metric one beside it.
pub fn shown(v: &Value) -> String {
    match v.unit {
        "°C" => format!("{:.0} °F ({:.0} °C)", v.value * 9.0 / 5.0 + 32.0, v.value),
        "km/h" => format!("{:.1} mph ({:.1} km/h)", v.value / 1.609_344, v.value),
        "kPa" => format!("{:.0} psi ({:.0} kPa)", v.value * 0.145_038, v.value),
        "L/h" => format!("{:.2} gal/h ({:.2} L/h)", v.value / 3.785_41, v.value),
        _ => format!("{:.0} {}", v.value, v.unit),
    }
}

/// J1939 messages put back together (single frames and the transport protocol's BAM and RTS/CTS transfers, watched only),
/// the latest common values per sender, and each sender's active faults.
#[derive(Default)]
pub struct Decoder {
    transfers: HashMap<(u8, u8), Transfer>,
    /// (sender, SPN) -> the latest value.
    pub values: BTreeMap<(u8, u32), Value>,
    /// Each sender's last DM1.
    pub dm1: BTreeMap<u8, Dm1>,
    /// Messages seen, per group, for the summary.
    pub groups: BTreeMap<u32, u64>,
}

impl Decoder {
    /// One 29-bit frame. A line to show when a sender's active faults changed (a new DM1 that differs from its last).
    pub fn add(&mut self, raw: u32, data: &[u8], at: f64) -> Option<String> {
        let i = id(raw);
        let (pgn, msg) = match i.pgn & 0x3FF00 {
            TP_CM => {
                self.connection(i, data);
                return None;
            }
            TP_DT => self.packet(i, data)?,
            _ => (i.pgn, data.to_vec()),
        };
        self.message(i.source, pgn, &msg, at)
    }

    fn connection(&mut self, i: Id, d: &[u8]) {
        let key = (i.source, i.dest.unwrap_or(0xFF));
        match d.first() {
            // BAM (to everyone) or RTS (to one address): size, packets and the group carried
            Some(0x20) | Some(0x10) if d.len() >= 8 => {
                let size = usize::from(d[1]) | (usize::from(d[2]) << 8);
                let n = usize::from(d[3]);
                if !(9..=1785).contains(&size) || n == 0 || n * 7 < size {
                    self.transfers.remove(&key);
                    return;
                }
                let pgn = u32::from(d[5]) | (u32::from(d[6]) << 8) | (u32::from(d[7]) << 16);
                self.transfers.insert(
                    key,
                    Transfer {
                        pgn,
                        size,
                        packets: vec![None; n],
                    },
                );
            }
            // the transfer was given up
            Some(0xFF) => {
                self.transfers.remove(&key);
            }
            _ => {}
        }
    }

    fn packet(&mut self, i: Id, d: &[u8]) -> Option<(u32, Vec<u8>)> {
        let key = (i.source, i.dest.unwrap_or(0xFF));
        let t = self.transfers.get_mut(&key)?;
        let seq = usize::from(*d.first()?);
        if seq == 0 || seq > t.packets.len() || d.len() < 8 {
            return None;
        }
        let mut p = [0u8; 7];
        p.copy_from_slice(&d[1..8]);
        t.packets[seq - 1] = Some(p);
        if t.packets.iter().any(Option::is_none) {
            return None;
        }
        let t = self.transfers.remove(&key)?;
        let mut m: Vec<u8> = t.packets.iter().flatten().flatten().copied().collect();
        m.truncate(t.size);
        Some((t.pgn, m))
    }

    fn message(&mut self, source: u8, pgn: u32, d: &[u8], at: f64) -> Option<String> {
        *self.groups.entry(pgn).or_default() += 1;
        for v in values(pgn, d) {
            self.values.insert((source, v.spn), v);
        }
        if pgn != DM1 {
            return None;
        }
        let new = dm1(d)?;
        if self.dm1.get(&source) == Some(&new) {
            return None;
        }
        let line = if new.faults.is_empty() {
            format!("{at:8.3}  J1939 {}: no active faults", source_name(source))
        } else {
            let mut l = format!(
                "{at:8.3}  J1939 {}: {} active fault(s)",
                source_name(source),
                new.faults.len()
            );
            for f in &new.faults {
                l.push_str(&format!("\n{:>8}    {}", "", f.describe()));
            }
            l
        };
        self.dm1.insert(source, new);
        Some(line)
    }

    /// What was read, for the end of a watch: each sender's values and active faults, with its warning lamps.
    pub fn summary(&self) -> Vec<String> {
        let mut out = Vec::new();
        let mut senders: Vec<u8> = self
            .values
            .keys()
            .map(|k| k.0)
            .chain(self.dm1.keys().copied())
            .collect();
        senders.sort_unstable();
        senders.dedup();
        for sa in senders {
            out.push(format!("{}:", source_name(sa)));
            for ((_, _), v) in self.values.range((sa, 0)..=(sa, u32::MAX)) {
                out.push(format!("  {} (SPN {}): {}", v.name, v.spn, shown(v)));
            }
            if let Some(d) = self.dm1.get(&sa) {
                let lamps: Vec<&str> = [
                    (d.mil, "check engine"),
                    (d.red_stop, "red stop"),
                    (d.amber_warning, "amber warning"),
                    (d.protect, "protect"),
                ]
                .iter()
                .filter(|(on, _)| *on == Some(true))
                .map(|(_, n)| *n)
                .collect();
                out.push(format!(
                    "  active faults: {}{}",
                    d.faults.len(),
                    if lamps.is_empty() {
                        String::new()
                    } else {
                        format!("; lamps on: {}", lamps.join(", "))
                    }
                ));
                for f in &d.faults {
                    out.push(format!("    {}", f.describe()));
                }
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_name_the_group_and_the_sender() {
        // EEC1 from the engine: priority 3, PGN F004 (PDU2: the PS byte is part of the group)
        assert_eq!(
            id(0x0CF0_0400),
            Id {
                priority: 3,
                pgn: 0xF004,
                source: 0,
                dest: None
            }
        );
        // a request to address 00 (PDU1: the PS byte is the destination)
        assert_eq!(
            id(0x18EA_00F9),
            Id {
                priority: 6,
                pgn: 0xEA00,
                source: 0xF9,
                dest: Some(0)
            }
        );
        assert_eq!(id(0x18FE_CA03).pgn, DM1);
        assert_eq!(id(0x18FE_CA03).source, 3);
    }

    #[test]
    fn dm1_lamps_and_faults() {
        // amber lamp on; SPN 110 FMI 0, 3 times; SPN 3251 (above 16 bits' low part: 0x0CB3) FMI 16 once
        let d = dm1(&[0x04, 0xFF, 0x6E, 0x00, 0x00, 0x03, 0xB3, 0x0C, 0x10, 0x01]).unwrap();
        assert_eq!(
            (d.mil, d.amber_warning, d.red_stop),
            (Some(false), Some(true), Some(false))
        );
        assert_eq!(d.faults.len(), 2);
        assert_eq!(
            (d.faults[0].spn, d.faults[0].fmi, d.faults[0].occurrences),
            (110, 0, 3)
        );
        assert_eq!((d.faults[1].spn, d.faults[1].fmi), (3251, 16));
        assert!(
            d.faults[0]
                .describe()
                .contains("engine coolant temperature")
                && d.faults[0].describe().contains("most severe")
        );
        // the three high SPN bits sit at the top of the third byte: SPN 520192 (0x7F000) is 00 F0 E0
        assert_eq!(
            dm1(&[0, 0, 0x00, 0xF0, 0xE0 | 2, 1]).unwrap().faults[0].spn,
            0x7F000
        );
        // no fault: SPN 0 FMI 0, and the padding
        assert!(dm1(&[0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF])
            .unwrap()
            .faults
            .is_empty());
        // an unknown SPN is shown by its number
        assert!(Fault {
            spn: 524287,
            fmi: 31,
            occurrences: 1,
            old_numbering: true
        }
        .describe()
        .contains("older SPN numbering"));
    }

    #[test]
    fn common_values_and_their_limits() {
        // EEC1: 1500 rpm = 12000 = 0x2EE0 in bytes 4-5
        let v = values(0xF004, &[0xF0, 0x7D, 0x7D, 0xE0, 0x2E, 0x00, 0xFF, 0xFF]);
        assert_eq!((v[0].spn, v[0].value), (190, 1500.0));
        // ET1: 125 = 85 °C, shown in °F first
        let t = values(0xFEEE, &[125, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]);
        assert_eq!(shown(&t[0]), "185 °F (85 °C)");
        // CCVS1: 100 km/h = 25600 in bytes 2-3
        let s = values(0xFEF1, &[0xFF, 0x00, 0x64, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]);
        assert_eq!(s[0].value, 100.0);
        // not available (FF FF) and error (FE xx) are left out
        assert!(values(0xF004, &[0, 0, 0, 0xFF, 0xFF, 0, 0, 0]).is_empty());
        assert!(values(0xFEEE, &[0xFE, 0, 0, 0, 0, 0, 0, 0]).is_empty());
    }

    #[test]
    fn a_dm1_in_a_bam_transfer_is_put_back_together() {
        let mut d = Decoder::default();
        // BAM from the engine: 14 bytes, 2 packets, PGN FECA
        assert_eq!(
            d.add(0x1CEC_FF00, &[0x20, 14, 0, 2, 0xFF, 0xCA, 0xFE, 0x00], 0.0),
            None
        );
        assert_eq!(
            d.add(
                0x1CEB_FF00,
                &[1, 0x04, 0xFF, 0x6E, 0x00, 0x00, 0x03, 0xB3],
                0.05
            ),
            None
        );
        let line = d
            .add(
                0x1CEB_FF00,
                &[2, 0x0C, 0x10, 0x01, 0x9D, 0x00, 0x04, 0x02],
                0.10,
            )
            .unwrap();
        assert!(line.contains("engine #1: 3 active fault(s)"), "{line}");
        assert!(
            line.contains("SPN 110 engine coolant temperature") && line.contains("SPN 157"),
            "{line}"
        );
        // the same DM1 again changes nothing; one with no faults says so
        assert_eq!(
            d.add(0x1CEC_FF00, &[0x20, 14, 0, 2, 0xFF, 0xCA, 0xFE, 0x00], 1.0),
            None
        );
        d.add(
            0x1CEB_FF00,
            &[1, 0x04, 0xFF, 0x6E, 0x00, 0x00, 0x03, 0xB3],
            1.05,
        );
        assert_eq!(
            d.add(
                0x1CEB_FF00,
                &[2, 0x0C, 0x10, 0x01, 0x9D, 0x00, 0x04, 0x02],
                1.1
            ),
            None
        );
        let line = d
            .add(0x18FE_CA00, &[0x00, 0xFF, 0, 0, 0, 0, 0xFF, 0xFF], 2.0)
            .unwrap();
        assert!(line.contains("no active faults"), "{line}");
        // packets out of order still complete; a stray packet with no transfer is ignored
        assert_eq!(d.add(0x1CEB_FF03, &[1, 0, 0, 0, 0, 0, 0, 0], 3.0), None);
        d.add(0x1CEC_FF03, &[0x20, 10, 0, 2, 0xFF, 0xCA, 0xFE, 0x00], 3.0);
        d.add(
            0x1CEB_FF03,
            &[2, 0x02, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF],
            3.1,
        );
        let line = d
            .add(
                0x1CEB_FF03,
                &[1, 0x00, 0xFF, 0xA8, 0x00, 0x04, 0x01, 0x7F],
                3.2,
            )
            .unwrap();
        assert!(
            line.contains("transmission #1: 2 active fault(s)")
                && line.contains("SPN 168 battery voltage"),
            "{line}"
        );
    }

    #[test]
    fn the_summary_names_values_faults_and_lamps() {
        let mut d = Decoder::default();
        d.add(
            0x0CF0_0400,
            &[0xF0, 0x7D, 0x7D, 0xE0, 0x2E, 0x00, 0xFF, 0xFF],
            0.0,
        );
        d.add(0x18FE_CA00, &[0x44, 0xFF, 0x6E, 0x00, 0x00, 0x03], 0.1);
        let s = d.summary().join("\n");
        assert!(
            s.contains("engine #1:") && s.contains("engine speed (SPN 190): 1500 rpm"),
            "{s}"
        );
        assert!(
            s.contains("active faults: 1; lamps on: check engine, amber warning"),
            "{s}"
        );
    }

    /// The simulated truck (`sim:j1939`) watched through an adapter on protocol A: the bus analyzer reads the engine's speed
    /// and its three active faults out of the BAM transfer, and the transmission's "no active faults".
    #[test]
    fn the_simulated_truck_watched_on_protocol_a() {
        let mut elm = crate::elm::Elm::listen(Box::new(crate::sim::SimCar::truck()), 0xA).unwrap();
        assert_eq!(elm.protocol, crate::elm::Protocol::J1939);
        let mut an = crate::bus::Analyzer::default();
        an.j1939 = Some(Decoder::default());
        let mut at = 0.0;
        elm.monitor_ids(None, |line| {
            if let Some(f) = crate::bus::parse_line(line, crate::elm::Protocol::J1939, at) {
                an.add(&f);
            }
            at += 0.005;
            let d = an.j1939.as_ref().unwrap();
            !(d.dm1.len() == 2 && d.values.contains_key(&(0, 110)))
        })
        .unwrap();
        let d = an.j1939.as_ref().unwrap();
        let spns: Vec<u32> = d.dm1[&0].faults.iter().map(|f| f.spn).collect();
        assert_eq!(spns, [110, 3251, 157]);
        assert_eq!(d.dm1[&0].amber_warning, Some(true));
        assert!(d.dm1[&3].faults.is_empty());
        let rpm = d.values[&(0, 190)].value;
        assert!((600.0..=700.0).contains(&rpm), "{rpm}");
        assert_eq!(shown(&d.values[&(0, 110)]), "185 °F (85 °C)");
        assert!(
            an.log
                .iter()
                .any(|l| l.contains("engine #1: 3 active fault(s)")),
            "{:?}",
            an.log
        );
        assert!(
            an.log
                .iter()
                .any(|l| l.contains("transmission #1: no active faults")),
            "{:?}",
            an.log
        );
    }
}
