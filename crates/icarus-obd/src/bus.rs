//! Watching the CAN bus: every frame the modules send each other, from the adapter's monitor mode (ATMA/STMA) or a Linux
//! CAN interface (`socketcan`). Icarus only listens here; nothing is sent onto the bus.
//!
//! The `Analyzer` keeps a table of IDs (how often each comes, its last bytes, which bytes just changed: press a pedal or a
//! switch and watch which byte follows), puts diagnostic messages (IDs 700-7FF, ISO-TP) back together and says what they
//! ask, and learns actuator tests from another tool: an output-control request (UDS `2F ... 03`, KWP2000 `30 .. 07`) becomes
//! a `[[test]]` entry, unchecked, for the vehicle's profile. A write of one data identifier that the module accepts (UDS
//! `2E`, KWP2000 `3B`) becomes a `[[setting]]` entry the same way, with the value read just before it (`22`/`21`) as "was".

use crate::actuator::{Condition, Control, Protocol, TestDef};
use crate::elm::Protocol as Bus;
use crate::settings::SettingDef;
use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};

#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub id: u32,
    /// A 29-bit ID.
    pub ext: bool,
    pub data: Vec<u8>,
    /// Seconds since watching began.
    pub at: f64,
}

/// One line of the adapter's monitor output (headers on): "2C4 03 1A F8" (11-bit) or "18 DA F1 10 03 41 0D 00" (29-bit).
pub fn parse_line(line: &str, bus: Bus, at: f64) -> Option<Frame> {
    let toks: Vec<&str> = line.split_whitespace().collect();
    if toks.is_empty()
        || !toks
            .iter()
            .all(|t| t.chars().all(|c| c.is_ascii_hexdigit()))
    {
        return None;
    }
    let ext = matches!(bus, Bus::Can29At500 | Bus::Can29At250 | Bus::J1939);
    let (id, rest) = if ext {
        if toks.len() < 4 || toks[..4].iter().any(|t| t.len() != 2) {
            return None;
        }
        (
            u32::from_str_radix(&toks[..4].concat(), 16).ok()?,
            &toks[4..],
        )
    } else {
        if toks[0].len() != 3 {
            return None;
        }
        (u32::from_str_radix(toks[0], 16).ok()?, &toks[1..])
    };
    if rest.len() > 8 || rest.iter().any(|t| t.len() != 2) {
        return None;
    }
    let data = rest
        .iter()
        .map(|t| u8::from_str_radix(t, 16).unwrap())
        .collect();
    Some(Frame { id, ext, data, at })
}

#[derive(Clone, Debug, Default)]
pub struct IdStats {
    pub count: u64,
    pub first: f64,
    pub last: f64,
    pub data: Vec<u8>,
    /// When each byte last changed (seconds), for highlighting.
    pub changed_at: Vec<f64>,
}

impl IdStats {
    pub fn hz(&self) -> f64 {
        let span = self.last - self.first;
        if self.count > 1 && span > 0.0 {
            (self.count - 1) as f64 / span
        } else {
            0.0
        }
    }
}

/// Service names for what a diagnostic message asks.
fn service(sid: u8) -> &'static str {
    match sid {
        0x01 => "current data",
        0x02 => "freeze frame",
        0x03 => "stored codes",
        0x04 => "clear codes (OBD-II)",
        0x06 => "on-board test results",
        0x07 => "pending codes",
        0x09 => "vehicle information",
        0x0A => "permanent codes",
        0x10 => "diagnostic session",
        0x11 => "module reset",
        0x14 => "clear codes",
        0x18 => "read codes (KWP2000)",
        0x19 => "read codes",
        0x1A => "read identification (KWP2000)",
        0x21 => "read data (KWP2000 local ID)",
        0x22 => "read data by identifier",
        0x23 => "read memory",
        0x27 => "security access",
        0x28 => "communication control",
        0x2E => "write data by identifier",
        0x2F => "output control",
        0x30 => "output control (KWP2000)",
        0x31 => "routine",
        0x34 => "request download (programming)",
        0x35 => "request upload",
        0x36 => "transfer data",
        0x37 => "end of transfer",
        0x3B => "write data (KWP2000)",
        0x3D => "write memory",
        0x3E => "tester present",
        0x85 => "trouble code setting",
        _ => "another service",
    }
}

fn hex(b: &[u8]) -> String {
    b.iter()
        .map(|x| format!("{x:02X}"))
        .collect::<Vec<_>>()
        .join(" ")
}

/// What a whole diagnostic message says, in words.
pub fn describe(msg: &[u8]) -> String {
    let Some(&sid) = msg.first() else {
        return String::new();
    };
    if sid == 0x7F && msg.len() >= 3 {
        return format!("refused {}: {}", service(msg[1]), crate::uds::nrc(msg[2]));
    }
    if (0x40..0x80).contains(&sid) || sid >= 0xC0 {
        let req = sid.wrapping_sub(0x40);
        return format!("answer to {}: {}", service(req), hex(&msg[1..]));
    }
    let detail = match (sid, &msg[1..]) {
        (0x01, [pid, ..]) => crate::j1979::pid(*pid)
            .map(|p| format!("PID {pid:02X} ({})", p.name))
            .unwrap_or(format!("PID {pid:02X}")),
        (0x10, [s, ..]) => match s {
            0x01 => "default".into(),
            0x02 => "programming".into(),
            0x03 => "extended".into(),
            s => format!("{s:02X}"),
        },
        (0x22, [hi, lo, ..]) => format!(
            "{:02X}{:02X}{}",
            hi,
            lo,
            crate::uds::IDENTITY
                .iter()
                .find(|(d, _)| *d == u16::from_be_bytes([*hi, *lo]))
                .map(|(_, n)| format!(" ({n})"))
                .unwrap_or_default()
        ),
        (0x2F, [hi, lo, p, rest @ ..]) => format!(
            "{:02X}{:02X}, {}{}",
            hi,
            lo,
            match p {
                0x00 => "return control to the module",
                0x01 => "reset to default",
                0x02 => "freeze current state",
                0x03 => "short-term adjustment",
                _ => "?",
            },
            if rest.is_empty() {
                String::new()
            } else {
                format!(" to {}", hex(rest))
            }
        ),
        (0x2E, [hi, lo, rest @ ..]) => format!("{hi:02X}{lo:02X} to {}", hex(rest)),
        (0x3B, [lid, rest @ ..]) => format!("{lid:02X} to {}", hex(rest)),
        (0x21, [lid, ..]) => format!("{lid:02X}"),
        (0x30, [lid, p, rest @ ..]) => format!(
            "{lid:02X}, {}{}",
            if *p == 0 { "return control" } else { "adjust" },
            if rest.is_empty() {
                String::new()
            } else {
                format!(" to {}", hex(rest))
            }
        ),
        (_, rest) => hex(rest),
    };
    format!(
        "{}{}",
        service(sid),
        if detail.is_empty() {
            String::new()
        } else {
            format!(": {detail}")
        }
    )
}

#[derive(Default)]
pub struct Analyzer {
    pub ids: BTreeMap<u32, IdStats>,
    /// ISO-TP messages being put together, per ID: (bytes, expected length).
    partial: HashMap<u32, (Vec<u8>, usize)>,
    /// What the diagnostic messages said, newest last (at most 500).
    pub log: VecDeque<String>,
    /// Actuator tests seen run by another tool.
    pub learned: Vec<TestDef>,
    /// Module settings seen changed by another tool (the module accepted the write).
    pub settings: Vec<SettingDef>,
    /// The last value each (module, identifier, KWP2000?) was read as, and writes waiting for the module's answer.
    read_values: HashMap<(u16, u16, bool), Vec<u8>>,
    pending_writes: HashMap<(u16, u16, bool), Vec<u8>>,
    /// Modules another tool has in the extended diagnostic session.
    extended: HashSet<u16>,
    pub frames: u64,
    /// Diagnostic messages described so far (the log keeps only the last 500).
    pub logged: u64,
}

impl Analyzer {
    pub fn add(&mut self, f: &Frame) {
        self.frames += 1;
        let s = self.ids.entry(f.id).or_insert_with(|| IdStats {
            first: f.at,
            ..Default::default()
        });
        if s.changed_at.len() < f.data.len() {
            s.changed_at.resize(f.data.len(), f.at);
        }
        for (i, b) in f.data.iter().enumerate() {
            if s.data.get(i) != Some(b) && s.count > 0 {
                s.changed_at[i] = f.at;
            }
        }
        s.data = f.data.clone();
        s.count += 1;
        s.last = f.at;
        if !f.ext && (0x700..=0x7FF).contains(&f.id) {
            if let Some(msg) = self.isotp(f) {
                self.diagnostic(f.id, &msg, f.at);
            }
        }
    }

    fn isotp(&mut self, f: &Frame) -> Option<Vec<u8>> {
        let &pci = f.data.first()?;
        match pci >> 4 {
            0 => {
                let n = (pci & 0x0F) as usize;
                (n > 0 && f.data.len() > n).then(|| f.data[1..1 + n].to_vec())
            }
            1 if f.data.len() >= 2 => {
                let n = (((pci & 0x0F) as usize) << 8) | f.data[1] as usize;
                self.partial.insert(f.id, (f.data[2..].to_vec(), n));
                None
            }
            2 => {
                let (buf, n) = self.partial.get_mut(&f.id)?;
                let take = (*n - buf.len()).min(f.data.len() - 1);
                buf.extend_from_slice(&f.data[1..1 + take]);
                if buf.len() >= *n {
                    let (b, _) = self.partial.remove(&f.id)?;
                    return Some(b);
                }
                None
            }
            _ => None, // flow control
        }
    }

    fn diagnostic(&mut self, id: u32, msg: &[u8], at: f64) {
        let Some(&sid) = msg.first() else { return };
        let answer = sid >= 0x40;
        let arrow = if id == 0x7DF {
            "7DF to every module".to_string()
        } else if answer {
            format!("{id:03X} answer")
        } else {
            format!("{id:03X} request")
        };
        self.log
            .push_back(format!("{at:8.3}  {arrow:<22} {}", describe(msg)));
        self.logged += 1;
        while self.log.len() > 500 {
            self.log.pop_front();
        }
        self.learn(id as u16, msg);
    }

    /// A data identifier another tool read and then changed (the module said yes) becomes a setting: the value before is
    /// "was", each value written "set", "set-2", ... (unchecked, parked conditions). Answers come from the module's ID + 8.
    fn learn_setting(&mut self, id: u16, msg: &[u8]) {
        let rx_tx = id.wrapping_sub(8);
        match msg {
            [0x10, 0x03] => {
                self.extended.insert(id);
            }
            [0x10, 0x01] => {
                self.extended.remove(&id);
            }
            [0x62, hi, lo, v @ ..] if !v.is_empty() => {
                self.read_values
                    .insert((rx_tx, u16::from_be_bytes([*hi, *lo]), false), v.to_vec());
            }
            [0x61, lid, v @ ..] if !v.is_empty() => {
                self.read_values
                    .insert((rx_tx, *lid as u16, true), v.to_vec());
            }
            [0x2E, hi, lo, v @ ..] if !v.is_empty() && v.len() <= 16 => {
                self.pending_writes
                    .insert((id, u16::from_be_bytes([*hi, *lo]), false), v.to_vec());
            }
            [0x3B, lid, v @ ..] if !v.is_empty() && v.len() <= 16 => {
                self.pending_writes
                    .insert((id, *lid as u16, true), v.to_vec());
            }
            [0x6E, hi, lo, ..] => {
                self.setting_written(rx_tx, u16::from_be_bytes([*hi, *lo]), Protocol::Uds)
            }
            [0x7B, lid, ..] => self.setting_written(rx_tx, *lid as u16, Protocol::Kwp),
            _ => {}
        }
    }

    fn setting_written(&mut self, tx: u16, did: u16, protocol: Protocol) {
        let key = (tx, did, protocol == Protocol::Kwp);
        let Some(value) = self.pending_writes.remove(&key) else {
            return;
        };
        let before = self
            .read_values
            .get(&key)
            .filter(|b| b.len() == value.len())
            .cloned();
        // what was written is what the module now holds
        self.read_values.insert(key, value.clone());
        let extended = protocol == Protocol::Uds && self.extended.contains(&tx);
        if let Some(s) = self
            .settings
            .iter_mut()
            .find(|s| s.module == tx && s.id == did && s.protocol == protocol)
        {
            s.extended |= extended;
            if value.len() == s.values.values().next().map_or(0, Vec::len)
                && s.name_of(&value).is_none()
            {
                let n = (2..)
                    .find(|n| !s.values.contains_key(&format!("set-{n}")))
                    .unwrap_or(2);
                s.values.insert(format!("set-{n}"), value);
            }
            return;
        }
        let mut values = BTreeMap::new();
        if let Some(b) = before.filter(|b| *b != value) {
            values.insert("was".to_string(), b);
        }
        values.insert("set".to_string(), value);
        let s = SettingDef {
            name: format!("Learned: setting {did:0w$X} on {tx:03X}", w = if protocol == Protocol::Uds { 4 } else { 2 }),
            module: tx,
            rx: None,
            protocol,
            id: did,
            values,
            extended,
            conditions: vec![Condition::EngineOff, Condition::Stationary],
            note: format!("Learned by watching another tool ({}): find out what it changes before relying on it.", crate::report::now()),
            checked: false,
        };
        if s.refused().is_none() {
            self.settings.push(s);
        }
    }

    /// An output-control request from another tool becomes a test (the most cautious conditions, unchecked).
    fn learn(&mut self, tx: u16, msg: &[u8]) {
        self.learn_setting(tx, msg);
        let (protocol, out, state) = match msg {
            [0x2F, hi, lo, 0x03, state @ ..] if !state.is_empty() && state.len() <= 4 => {
                (Protocol::Uds, u16::from_be_bytes([*hi, *lo]), state)
            }
            [0x30, lid, 0x07, state @ ..] if !state.is_empty() && state.len() <= 4 => {
                (Protocol::Kwp, *lid as u16, state)
            }
            _ => return,
        };
        if let Some(t) = self
            .learned
            .iter_mut()
            .find(|t| t.module == tx && t.id == out && t.protocol == protocol)
        {
            // several different one-byte settings: a setting with a range rather than on/off
            if state.len() == 1 && t.on.len() == 1 && t.on[0] != state[0] {
                t.control = Control::Percent;
                t.warning = format!(
                    "{} Seen with several settings; check its scaling (now: percent x {}).",
                    t.warning.split(" Seen with").next().unwrap_or(""),
                    t.scale
                );
            }
            return;
        }
        let t = TestDef {
            name: format!("Learned: output {out:0w$X} on {tx:03X}", w = if protocol == Protocol::Uds { 4 } else { 2 }),
            module: tx,
            rx: None,
            protocol,
            id: out,
            control: Control::Onoff,
            on: state.to_vec(),
            scale: 2.55,
            conditions: vec![Condition::EngineOff, Condition::Stationary],
            max_seconds: 10,
            warning: format!("Learned by watching another tool ({}): find out what it drives before relying on it.", crate::report::now()),
            watch: None,
            checked: false,
        };
        if t.refused().is_none() {
            self.learned.push(t);
        }
    }
}

/// A `[[test]]` entry for a vehicle profile.
pub fn to_toml(t: &TestDef) -> String {
    let mut s = String::from("[[test]]\n");
    s += &format!("name = {:?}\nmodule = 0x{:03X}\n", t.name, t.module);
    if let Some(rx) = t.rx {
        s += &format!("rx = 0x{rx:03X}\n");
    }
    s += &format!(
        "protocol = \"{}\"\nid = 0x{:X}\n",
        if t.protocol == Protocol::Uds {
            "uds"
        } else {
            "kwp"
        },
        t.id
    );
    s += &format!(
        "control = \"{}\"\n",
        if t.control == Control::Percent {
            "percent"
        } else {
            "onoff"
        }
    );
    if t.control == Control::Percent {
        s += &format!("scale = {}\n", t.scale);
    } else {
        s += &format!(
            "on = [{}]\n",
            t.on.iter()
                .map(|b| format!("0x{b:02X}"))
                .collect::<Vec<_>>()
                .join(", ")
        );
    }
    let conds: Vec<&str> = t
        .conditions
        .iter()
        .map(|c| match c {
            Condition::EngineOff => "\"engine-off\"",
            Condition::EngineRunning => "\"engine-running\"",
            Condition::Stationary => "\"stationary\"",
        })
        .collect();
    s += &format!(
        "conditions = [{}]\nmax_seconds = {}\nwarning = {:?}\n",
        conds.join(", "),
        t.max_seconds,
        t.warning
    );
    if let Some(w) = t.watch {
        s += &format!("watch = 0x{w:02X}\n");
    }
    s += &format!("checked = {}\n", t.checked);
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    fn f(id: u32, data: &[u8], at: f64) -> Frame {
        Frame {
            id,
            ext: false,
            data: data.to_vec(),
            at,
        }
    }

    #[test]
    fn lines_from_the_adapter() {
        assert_eq!(
            parse_line("2C4 03 1A F8", Bus::Can11At500, 1.0).unwrap(),
            f(0x2C4, &[0x03, 0x1A, 0xF8], 1.0)
        );
        let e = parse_line("18 DA F1 10 03 41 0D 00", Bus::Can29At500, 0.0).unwrap();
        assert_eq!((e.id, e.ext, e.data.len()), (0x18DAF110, true, 4));
        for bad in [
            "BUFFER FULL",
            "<DATA ERROR",
            "2C4 03 1A F8 00 00 00 00 00 00",
            "12 34",
            "",
        ] {
            assert!(parse_line(bad, Bus::Can11At500, 0.0).is_none(), "{bad}");
        }
    }

    #[test]
    fn the_table_marks_changing_bytes() {
        let mut a = Analyzer::default();
        a.add(&f(0x2C4, &[0x03, 0x10, 0x00], 0.0));
        a.add(&f(0x2C4, &[0x03, 0x14, 0x00], 0.5));
        a.add(&f(0x2C4, &[0x03, 0x14, 0x00], 1.0));
        let s = &a.ids[&0x2C4];
        assert_eq!((s.count, s.hz()), (3, 2.0));
        assert_eq!(s.changed_at, vec![0.0, 0.5, 0.0], "byte 1 changed at 0.5 s");
    }

    #[test]
    fn a_setting_the_module_accepted_is_learned() {
        let mut a = Analyzer::default();
        let convo: &[(u32, &[u8])] = &[
            (0x7C0, &[0x02, 0x10, 0x03]),
            (0x7C8, &[0x02, 0x50, 0x03]),
            (0x7C0, &[0x03, 0x22, 0xA0, 0x01]),
            (0x7C8, &[0x04, 0x62, 0xA0, 0x01, 0x01]),
            (0x7C0, &[0x04, 0x2E, 0xA0, 0x01, 0x00]),
            (0x7C8, &[0x03, 0x6E, 0xA0, 0x01]),
            (0x7C0, &[0x04, 0x2E, 0xA0, 0x01, 0x02]),
            (0x7C8, &[0x03, 0x6E, 0xA0, 0x01]),
            // refused: not learned
            (0x7B0, &[0x03, 0x3B, 0x30, 0x01]),
            (0x7B8, &[0x03, 0x7F, 0x3B, 0x22]),
            // the airbag module: never learned
            (0x780, &[0x04, 0x2E, 0x12, 0x34, 0x01]),
            (0x788, &[0x03, 0x6E, 0x12, 0x34]),
        ];
        for (i, (id, d)) in convo.iter().enumerate() {
            a.add(&f(*id, d, i as f64));
        }
        assert!(
            a.log[4].contains("write data by identifier: A001 to 00"),
            "{:?}",
            a.log
        );
        assert_eq!(a.settings.len(), 1, "{:?}", a.settings);
        let s = &a.settings[0];
        assert_eq!(
            (s.module, s.id, s.protocol, s.extended, s.checked),
            (0x7C0, 0xA001, Protocol::Uds, true, false)
        );
        let v: Vec<(&str, &[u8])> = s
            .values
            .iter()
            .map(|(k, v)| (k.as_str(), v.as_slice()))
            .collect();
        assert_eq!(
            v,
            vec![
                ("set", &[0x00][..]),
                ("set-2", &[0x02][..]),
                ("was", &[0x01][..])
            ]
        );
        let entry = crate::settings::to_toml(s);
        let p = crate::vehicles::parse("x", "test", &format!("name = \"x\"\n{entry}")).unwrap();
        assert_eq!(&p.settings[0], s, "{entry}");
    }

    #[test]
    fn a_shop_tool_running_a_throttle_test_is_learned() {
        let mut a = Analyzer::default();
        let convo: &[(u32, &[u8])] = &[
            (0x7E0, &[0x02, 0x10, 0x03]),
            (0x7E8, &[0x02, 0x50, 0x03]),
            (0x7E0, &[0x05, 0x2F, 0xF0, 0x02, 0x03, 0x4C]),
            (0x7E8, &[0x05, 0x6F, 0xF0, 0x02, 0x03, 0x4C]),
            (0x7E0, &[0x05, 0x2F, 0xF0, 0x02, 0x03, 0x80]),
            (0x7E0, &[0x04, 0x2F, 0xF0, 0x02, 0x00]),
            (0x7B0, &[0x04, 0x30, 0x10, 0x07, 0x01]),
            (0x780, &[0x05, 0x2F, 0x12, 0x34, 0x03, 0x01]),
        ];
        for (i, (id, d)) in convo.iter().enumerate() {
            a.add(&f(*id, d, i as f64));
        }
        assert!(
            a.log[0].contains("diagnostic session: extended"),
            "{:?}",
            a.log
        );
        assert!(a.log[2].contains("output control: F002, short-term adjustment to 4C"));
        assert!(a.log[5].contains("return control to the module"));
        assert_eq!(
            a.learned.len(),
            2,
            "the airbag module's output is never learned: {:?}",
            a.learned
        );
        let t = &a.learned[0];
        assert_eq!(
            (t.module, t.id, t.protocol, t.control),
            (0x7E0, 0xF002, Protocol::Uds, Control::Percent),
            "two settings: a percentage"
        );
        assert_eq!(
            (a.learned[1].protocol, a.learned[1].on.clone()),
            (Protocol::Kwp, vec![0x01])
        );
        // the entry reads back as a profile test, unchecked
        let toml = format!("name = \"x\"\n{}", to_toml(&a.learned[1]));
        let p = crate::vehicles::parse("x", "test", &toml).unwrap();
        assert_eq!((p.tests[0].id, p.tests[0].checked), (0x10, false));
    }

    #[test]
    fn long_messages_are_put_together() {
        let mut a = Analyzer::default();
        a.add(&f(
            0x7E8,
            &[0x10, 0x14, 0x49, 0x02, 0x01, 0x31, 0x41, 0x42],
            0.0,
        ));
        a.add(&f(0x7E0, &[0x30, 0x00, 0x00], 0.0));
        a.add(&f(
            0x7E8,
            &[0x21, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49],
            0.0,
        ));
        a.add(&f(
            0x7E8,
            &[0x22, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50],
            0.0,
        ));
        assert_eq!(a.log.len(), 1);
        assert!(a.log[0].contains("answer to vehicle information"));
    }
}
