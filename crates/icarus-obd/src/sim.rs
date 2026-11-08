//! A simulated vehicle behind a simulated ELM327 adapter (`--adapter sim`): for trying Icarus without a vehicle, and for the
//! tests. It speaks like a real adapter (echo, headers, spaces, the `>` prompt, ISO-TP frames on CAN 11-bit at 500 kbit/s)
//! and has two modules: an engine (7E8) and a transmission (7E9). Its VIN, 1ICARUSSIM0000001, cannot be a real one (real VINs
//! never use I, O or Q).

use crate::port::Port;
use std::collections::VecDeque;
use std::io;
use std::time::Duration;

pub const VIN: &str = "1ICARUSSIM0000001";

pub struct SimCar {
    input: Vec<u8>,
    output: VecDeque<u8>,
    echo: bool,
    headers: bool,
    spaces: bool,
    tick: u32,
    /// Where requests go (ATSH): 7DF is OBD's broadcast to all modules, anything else one module.
    header: u16,
    /// Only answers from this ID are shown (ATCRA).
    filter: Option<u16>,
    /// The instrument cluster answers its first trouble-code request with "response pending" (7F 19 78).
    meter_busy: bool,
}

impl Default for SimCar {
    fn default() -> Self {
        Self::new()
    }
}

const ENGINE_PIDS: &[u8] = &[
    0x01, 0x03, 0x04, 0x05, 0x06, 0x07, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x14, 0x15, 0x1F,
    0x20, 0x2F, 0x33, 0x3C, 0x40, 0x42, 0x44, 0x46, 0x49,
];
const TCM_PIDS: &[u8] = &[0x0D];

impl SimCar {
    pub fn new() -> SimCar {
        SimCar {
            input: Vec::new(),
            output: VecDeque::new(),
            echo: true,
            headers: false,
            spaces: true,
            tick: 0,
            header: 0x7DF,
            filter: None,
            meter_busy: true,
        }
    }

    fn say(&mut self, lines: &[String]) {
        for l in lines {
            self.output.extend(l.as_bytes());
            self.output.push_back(b'\r');
        }
        self.output.extend(b"\r>");
    }

    fn hex(&self, b: &[u8]) -> String {
        b.iter()
            .map(|x| format!("{x:02X}"))
            .collect::<Vec<_>>()
            .join(if self.spaces { " " } else { "" })
    }

    /// One module's message as the adapter would print it.
    fn frames(&self, header: &str, payload: &[u8]) -> Vec<String> {
        let sep = if self.spaces { " " } else { "" };
        if payload.len() <= 7 {
            let mut f = vec![payload.len() as u8];
            f.extend_from_slice(payload);
            return vec![if self.headers {
                format!("{header}{sep}{}", self.hex(&f))
            } else {
                self.hex(payload)
            }];
        }
        let mut out = Vec::new();
        if !self.headers {
            out.push(format!("{:03X}", payload.len()));
        }
        let mut first = vec![0x10 | (payload.len() >> 8) as u8, payload.len() as u8];
        first.extend_from_slice(&payload[..6]);
        out.push(if self.headers {
            format!("{header}{sep}{}", self.hex(&first))
        } else {
            format!("0:{sep}{}", self.hex(&payload[..6]))
        });
        for (i, chunk) in payload[6..].chunks(7).enumerate() {
            let seq = (i + 1) as u8 & 0x0F;
            let mut cf = vec![0x20 | seq];
            cf.extend_from_slice(chunk);
            out.push(if self.headers {
                format!("{header}{sep}{}", self.hex(&cf))
            } else {
                format!("{:X}:{sep}{}", seq, self.hex(chunk))
            });
        }
        out
    }

    fn bitmap(pids: &[u8], base: u8) -> [u8; 4] {
        let mut m = [0u8; 4];
        for &p in pids {
            if p > base && (p as u16) <= base as u16 + 32 {
                let i = (p - base - 1) as usize;
                m[i / 8] |= 0x80 >> (i % 8);
            }
        }
        m
    }

    fn engine_pid(&self, pid: u8) -> Option<Vec<u8>> {
        let t = self.tick as f64;
        let rpm = 790.0 + 40.0 * (t * 0.7).sin();
        let r = (rpm * 4.0) as u16;
        Some(match pid {
            0x00 | 0x20 | 0x40 => Self::bitmap(ENGINE_PIDS, pid).to_vec(),
            0x01 => vec![0x81, 0x07, 0x65, 0x04], // MIL on, 1 code; evaporative test not yet run
            0x03 => vec![0x02, 0x00],
            0x04 => vec![52],
            0x05 => vec![(88 + 40) as u8],
            0x06 => vec![(128.0 + 6.0 + 4.0 * (t * 1.3).sin()) as u8], // short trim adding fuel: it reads lean
            0x07 => vec![128 + 15], // long trim +11.7 %: the P0171 story
            0x0B => vec![31],
            0x0C => r.to_be_bytes().to_vec(),
            0x0D => vec![0],
            0x0E => vec![(2.0 * (12.0 + 64.0)) as u8],
            0x0F => vec![(31 + 40) as u8],
            0x10 => (380u16).to_be_bytes().to_vec(),
            0x11 => vec![37],
            0x14 => vec![
                (if self.tick.is_multiple_of(2) {
                    0.15
                } else {
                    0.78
                } * 200.0) as u8,
                0x86,
            ],
            0x15 => vec![(0.62 * 200.0) as u8, 0xFF],
            0x1F => ((self.tick * 2) as u16).to_be_bytes().to_vec(),
            0x2F => vec![160],
            0x33 => vec![86],
            0x3C => ((520 + 40) as u16 * 10).to_be_bytes().to_vec(),
            0x42 => (14_100u16).to_be_bytes().to_vec(),
            0x44 => (32_768u16).to_be_bytes().to_vec(),
            0x46 => vec![(18 + 40) as u8],
            0x49 => vec![0],
            _ => return None,
        })
    }

    fn answer(&mut self, req: &[u8]) -> Vec<String> {
        self.tick += 1;
        let mut msgs: Vec<(String, Vec<u8>)> = if self.header == 0x7DF {
            self.broadcast(req)
                .into_iter()
                .map(|(h, p)| (h.to_string(), p))
                .collect()
        } else {
            self.module(self.header, req)
                .map(|p| (format!("{:03X}", self.header + 8), p))
                .into_iter()
                .collect()
        };
        if let Some(f) = self.filter {
            msgs.retain(|(h, _)| *h == format!("{f:03X}"));
        }
        if msgs.is_empty() {
            return vec!["NO DATA".into()];
        }
        msgs.iter().flat_map(|(h, p)| self.frames(h, p)).collect()
    }

    /// One module's answer to a request sent to it alone: the engine and transmission speak UDS, the ABS only KWP2000 (as
    /// many Toyota modules do), the airbag and instrument cluster UDS. Anything else is "not supported" (7F) or silence.
    fn module(&mut self, tx: u16, req: &[u8]) -> Option<Vec<u8>> {
        let refuse = |code: u8| Some(vec![0x7F, req[0], code]);
        let (part, uds, codes): (&[u8], bool, &[u8]) = match tx {
            0x7E0 => (b"89661-ICARUS", true, &[0x01, 0x71, 0x00, 0x09]),
            0x7E1 => (b"35010-ICARUS", true, &[]),
            0x7B0 => (b"89540-ICARUS", false, &[0x52, 0x41, 0x60]),
            0x780 => (b"89170-ICARUS", true, &[]),
            0x7C0 => (b"83800-ICARUS", true, &[]),
            _ => return None,
        };
        match (req, uds) {
            ([0x3E, 0x00], true) => Some(vec![0x7E, 0x00]),
            ([0x3E, 0x00], false) => refuse(0x12),
            ([0x3E, 0x01], false) => Some(vec![0x7E]),
            ([0x01, pid], _) if tx == 0x7E0 => self
                .engine_pid(*pid)
                .map(|d| [&[0x41, *pid][..], &d].concat()),
            ([0x19, 0x02, mask], true) => {
                if tx == 0x7C0 && self.meter_busy {
                    self.meter_busy = false;
                    return refuse(0x78);
                }
                Some([&[0x59, 0x02, *mask][..], codes].concat())
            }
            ([0x18, ..], false) => Some([&[0x58, (codes.len() / 3) as u8][..], codes].concat()),
            ([0x22, 0xF1, 0x87], true) => Some([&[0x62, 0xF1, 0x87][..], part].concat()),
            ([0x22, 0xF1, 0x88], true) => {
                Some([&[0x62, 0xF1, 0x88][..], b"ICARUS-SW-0001"].concat())
            }
            ([0x22, 0xF1, 0x90], true) if tx == 0x7E0 => {
                Some([&[0x62, 0xF1, 0x90][..], VIN.as_bytes()].concat())
            }
            ([0x22, ..], true) => refuse(0x31),
            _ => refuse(0x11),
        }
    }

    /// Answers to OBD's broadcast (7DF), from every module that has one.
    fn broadcast(&mut self, req: &[u8]) -> Vec<(&'static str, Vec<u8>)> {
        let mut msgs: Vec<(&'static str, Vec<u8>)> = Vec::new();
        match req {
            [0x01, pid] => {
                if let Some(d) = self.engine_pid(*pid) {
                    msgs.push(("7E8", [&[0x41, *pid][..], &d].concat()));
                }
                if *pid == 0x00 || TCM_PIDS.contains(pid) {
                    let d = if *pid == 0x00 {
                        Self::bitmap(TCM_PIDS, 0).to_vec()
                    } else {
                        vec![0]
                    };
                    msgs.push(("7E9", [&[0x41, *pid][..], &d].concat()));
                }
            }
            [0x03] => {
                msgs.push(("7E8", vec![0x43, 0x01, 0x01, 0x71]));
                msgs.push(("7E9", vec![0x43, 0x00]));
            }
            [0x07] => msgs.push(("7E8", vec![0x47, 0x00])),
            [0x0A] => msgs.push(("7E8", vec![0x4A, 0x00])),
            [0x09, 0x02] => msgs.push(("7E8", [&[0x49, 0x02, 0x01][..], VIN.as_bytes()].concat())),
            [0x09, 0x04] => {
                msgs.push((
                    "7E8",
                    [&[0x49, 0x04, 0x01][..], b"ICARUS-SIM-CAL01"].concat(),
                ));
                msgs.push((
                    "7E9",
                    [&[0x49, 0x04, 0x01][..], b"ICARUS-SIM-TCM01"].concat(),
                ));
            }
            [0x09, 0x06] => msgs.push(("7E8", vec![0x49, 0x06, 0x01, 0x1C, 0xA5, 0x0F, 0x3E])),
            [0x09, 0x0A] => {
                msgs.push((
                    "7E8",
                    [&[0x49, 0x0A, 0x01][..], b"ECM\0-EngineControl\0"].concat(),
                ));
                msgs.push((
                    "7E9",
                    [&[0x49, 0x0A, 0x01][..], b"TCM\0-TransmisCtrl\0\0"].concat(),
                ));
            }
            _ => {}
        }
        msgs
    }

    fn handle(&mut self, cmd: &str) {
        let raw = cmd.trim();
        if self.echo {
            self.output.extend(raw.as_bytes());
            self.output.push_back(b'\r');
        }
        let c: String = raw
            .chars()
            .filter(|c| !c.is_whitespace())
            .collect::<String>()
            .to_uppercase();
        let ok = vec!["OK".to_string()];
        let reply: Vec<String> = match c.as_str() {
            "ATZ" => {
                (self.echo, self.headers, self.spaces) = (true, false, true);
                (self.header, self.filter) = (0x7DF, None);
                vec!["".into(), "ELM327 v1.5".into()]
            }
            "ATI" => vec!["ELM327 v1.5".into()],
            "ATE0" | "ATE1" => {
                self.echo = c.ends_with('1');
                ok
            }
            "ATH0" | "ATH1" => {
                self.headers = c.ends_with('1');
                ok
            }
            "ATS0" | "ATS1" => {
                self.spaces = c.ends_with('1');
                ok
            }
            "ATDPN" => vec!["A6".into()],
            "ATCRA" => {
                self.filter = None;
                ok
            }
            _ if c.starts_with("ATSH") || c.starts_with("ATCRA") => {
                match u16::from_str_radix(&c[if c.starts_with("ATSH") { 4 } else { 5 }..], 16) {
                    Ok(id) if id <= 0x7FF => {
                        if c.starts_with("ATSH") {
                            self.header = id;
                        } else {
                            self.filter = Some(id);
                        }
                        ok
                    }
                    _ => vec!["?".into()],
                }
            }
            _ if c.starts_with("ATFCSH") || c.starts_with("ATFCSD") || c.starts_with("ATFCSM") => {
                ok
            }
            "ATRV" => vec!["12.6V".into()],
            _ if c.starts_with("ATL")
                || c.starts_with("ATSP")
                || c.starts_with("ATST")
                || c.starts_with("ATAT") =>
            {
                ok
            }
            _ if c.starts_with("AT") || c.starts_with("ST") => vec!["?".into()],
            _ if !c.is_empty()
                && c.len().is_multiple_of(2)
                && c.chars().all(|ch| ch.is_ascii_hexdigit()) =>
            {
                let req: Vec<u8> = (0..c.len())
                    .step_by(2)
                    .map(|i| u8::from_str_radix(&c[i..i + 2], 16).unwrap())
                    .collect();
                self.answer(&req)
            }
            _ => vec!["?".into()],
        };
        self.say(&reply);
    }
}

impl Port for SimCar {
    fn send(&mut self, data: &[u8]) -> io::Result<()> {
        for &b in data {
            if b == b'\r' {
                let cmd = String::from_utf8_lossy(&std::mem::take(&mut self.input)).to_string();
                self.handle(&cmd);
            } else if b != b'\n' {
                self.input.push(b);
            }
        }
        Ok(())
    }

    fn recv(&mut self, buf: &mut [u8], _timeout: Duration) -> io::Result<usize> {
        let n = buf.len().min(self.output.len());
        for (i, b) in self.output.drain(..n).enumerate() {
            buf[i] = b;
        }
        Ok(n)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elm::Elm;
    use crate::j1979;

    #[test]
    fn talks_like_an_adapter() {
        let mut s = SimCar::new();
        s.send(b"ATI\r").unwrap();
        let mut b = [0u8; 64];
        let n = s.recv(&mut b, Duration::ZERO).unwrap();
        assert_eq!(&b[..n], b"ATI\rELM327 v1.5\r\r>", "echo on until ATE0");
        // without headers a long message comes in the adapter's numbered-line form
        s.send(b"ATE0\r0902\r").unwrap();
        let n = s.recv(&mut [0u8; 512], Duration::ZERO).unwrap();
        assert!(n > 0);
    }

    #[test]
    fn a_whole_session_reads_the_simulated_car() {
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        assert_eq!(j1979::vin(&mut elm).unwrap().as_deref(), Some(VIN));
        let sup = j1979::supported(&mut elm).unwrap();
        assert_eq!(sup.len(), 2, "engine and transmission");
        assert!(
            sup[0].2.contains(&0x0C) && sup[0].2.contains(&0x49),
            "{:?}",
            sup[0].2
        );
        assert_eq!(sup[1].2, vec![0x0D]);
        let codes = j1979::dtcs(&mut elm, j1979::DtcKind::Stored).unwrap();
        assert_eq!(codes[0].2, vec!["P0171".to_string()]);
        assert!(codes[1].2.is_empty());
        let cal = j1979::calibration_ids(&mut elm).unwrap();
        assert_eq!(cal[0].2, vec!["ICARUS-SIM-CAL01".to_string()]);
        assert_eq!(
            j1979::ecu_names(&mut elm).unwrap()[0].2,
            vec!["ECM-EngineControl".to_string()]
        );
        assert_eq!(
            j1979::cvns(&mut elm).unwrap()[0].1,
            vec!["1CA50F3E".to_string()]
        );
        let rpm = j1979::read(&mut elm, 0x0C).unwrap();
        assert!((700.0..900.0).contains(&rpm[0].value));
        let ltft = j1979::read(&mut elm, 0x07).unwrap();
        assert!((ltft[0].value - 11.71875).abs() < 1e-9);
    }
}
