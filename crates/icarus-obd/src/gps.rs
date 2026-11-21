//! GPS: the NMEA 0183 sentences every GPS receiver speaks (a USB receiver such as a u-blox stick, `/dev/ttyACM0`; a
//! Bluetooth one, `bt:...`; a phone app that shares its GPS over the network, `tcp:HOST:PORT`; or `sim`), read for three
//! things: the speedometer check (GPS speed against the vehicle's own, at a steady speed), diagnosing a receiver or
//! antenna (satellites seen and used, each one's signal strength, the fix and its precision), and later positions in
//! recordings. Positions stay on this computer: nothing here sends them anywhere or writes them to a file.
//!
//! Written for Icarus from the published sentence layouts; Knio/pynmea2 (MIT) was the reference for which fields each
//! sentence has. Every sentence's checksum is checked; one that does not add up is dropped, never half-read. Talker IDs
//! (GP GPS, GL GLONASS, GA Galileo, GB/BD BeiDou, GQ QZSS, GN combined) are all read.

use crate::port::{Port, Target};
use std::collections::BTreeMap;
use std::time::{Duration, Instant};

/// One satellite as GSV reports it.
#[derive(Clone, Debug, PartialEq)]
pub struct Sat {
    /// "GPS", "GLONASS", "Galileo", "BeiDou", "QZSS".
    pub system: &'static str,
    pub prn: u16,
    pub elevation: Option<u8>,
    pub azimuth: Option<u16>,
    /// Signal to noise, dB-Hz: under ~25 is weak, 30-45 is normal under open sky; None: seen in the almanac, not received.
    pub snr: Option<u8>,
}

/// What the receiver says now, built up from its sentences.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Fix {
    /// RMC's status: A (valid) or V (no fix).
    pub valid: bool,
    /// GGA's quality: 0 none, 1 GPS, 2 differential, 4/5 RTK, 6 dead reckoning.
    pub quality: Option<u8>,
    /// GSA's mode: 1 none, 2 2D, 3 3D.
    pub mode: Option<u8>,
    pub lat: Option<f64>,
    pub lon: Option<f64>,
    pub altitude_m: Option<f64>,
    /// Over the ground, km/h (RMC's knots converted, or VTG's km/h).
    pub speed_kmh: Option<f64>,
    pub course: Option<f64>,
    /// "hhmmss" UTC and "ddmmyy", as the receiver gives them.
    pub time_utc: Option<String>,
    pub date: Option<String>,
    pub used: Option<u8>,
    pub hdop: Option<f64>,
    pub pdop: Option<f64>,
    pub vdop: Option<f64>,
    /// Satellites in view, by (system, PRN).
    pub sats: BTreeMap<(&'static str, u16), Sat>,
    /// Sentences read, and dropped (bad checksum or unreadable).
    pub sentences: u64,
    pub dropped: u64,
}

fn system_of(talker: &str) -> &'static str {
    match talker {
        "GL" => "GLONASS",
        "GA" => "Galileo",
        "GB" | "BD" => "BeiDou",
        "GQ" => "QZSS",
        "GI" => "NavIC",
        _ => "GPS",
    }
}

/// The fields of one sentence (`$GPRMC,...*hh`), after its checksum is checked: (talker, type, fields).
pub fn split(line: &str) -> Result<(String, String, Vec<String>), String> {
    let line = line.trim();
    let body = line
        .strip_prefix('$')
        .or_else(|| line.strip_prefix('!'))
        .ok_or("not an NMEA sentence")?;
    let (data, sum) = body.rsplit_once('*').ok_or("no checksum")?;
    let want = u8::from_str_radix(sum.trim(), 16).map_err(|_| "bad checksum digits")?;
    let got = data.bytes().fold(0u8, |a, b| a ^ b);
    if got != want {
        return Err(format!("checksum {got:02X}, sentence says {want:02X}"));
    }
    let mut f = data.split(',');
    let head = f.next().unwrap_or("");
    if head.len() < 5 || !head.is_ascii() {
        return Err("no sentence type".into());
    }
    let (talker, kind) = if let Some(own) = head.strip_prefix('P') {
        ("P".to_string(), own.to_string()) // a maker's own sentence
    } else {
        (head[..2].to_string(), head[2..].to_string())
    };
    Ok((talker, kind, f.map(str::to_string).collect()))
}

/// ddmm.mmmm (or dddmm.mmmm) and a hemisphere as degrees.
fn coord(v: &str, hemi: &str) -> Option<f64> {
    let x: f64 = v.parse().ok()?;
    let deg = (x / 100.0).trunc();
    let d = deg + (x - deg * 100.0) / 60.0;
    match hemi {
        "N" | "E" => Some(d),
        "S" | "W" => Some(-d),
        _ => None,
    }
}

fn num<T: std::str::FromStr>(f: &[String], i: usize) -> Option<T> {
    f.get(i).and_then(|s| s.trim().parse().ok())
}

impl Fix {
    /// Reads one sentence into the fix; Err when it was dropped.
    pub fn feed(&mut self, line: &str) -> Result<(), String> {
        let (talker, kind, f) = match split(line) {
            Ok(x) => x,
            Err(e) => {
                self.dropped += 1;
                return Err(e);
            }
        };
        self.sentences += 1;
        match kind.as_str() {
            "RMC" => {
                self.time_utc = f
                    .first()
                    .filter(|t| !t.is_empty())
                    .map(|t| t[..t.len().min(6)].to_string());
                self.valid = f.get(1).map(String::as_str) == Some("A");
                if self.valid {
                    self.lat = coord(f.get(2).map_or("", |s| s), f.get(3).map_or("", |s| s));
                    self.lon = coord(f.get(4).map_or("", |s| s), f.get(5).map_or("", |s| s));
                    self.speed_kmh = num::<f64>(&f, 6).map(|k| k * 1.852);
                    self.course = num(&f, 7);
                }
                self.date = f.get(8).filter(|d| d.len() == 6).cloned();
            }
            "VTG" => {
                // the km/h field is the one before "K"
                if let Some(i) = f.iter().position(|x| x == "K") {
                    if let Some(v) = i.checked_sub(1).and_then(|j| num::<f64>(&f, j)) {
                        if self.valid {
                            self.speed_kmh = Some(v);
                        }
                    }
                }
            }
            "GGA" => {
                self.quality = num(&f, 5);
                self.used = num(&f, 6);
                self.hdop = num(&f, 7);
                if self.quality.is_some_and(|q| q > 0) {
                    self.lat =
                        coord(f.get(1).map_or("", |s| s), f.get(2).map_or("", |s| s)).or(self.lat);
                    self.lon =
                        coord(f.get(3).map_or("", |s| s), f.get(4).map_or("", |s| s)).or(self.lon);
                    self.altitude_m = num(&f, 8);
                }
            }
            "GSA" => {
                self.mode = num(&f, 1);
                self.pdop = num(&f, 14);
                self.hdop = num(&f, 15).or(self.hdop);
                self.vdop = f
                    .get(16)
                    .and_then(|v| v.split('*').next())
                    .and_then(|v| v.trim().parse().ok());
            }
            "GSV" => {
                let system = system_of(&talker);
                if num::<u8>(&f, 1) == Some(1) {
                    // the first of a group: that system's list starts over
                    self.sats.retain(|(s, _), _| *s != system);
                }
                // groups of four; NMEA 4.10 adds a signal ID at the end, which is no satellite
                for c in f.get(3..).unwrap_or(&[]).chunks(4).filter(|c| c.len() == 4) {
                    let Some(prn) = c.first().and_then(|p| p.trim().parse::<u16>().ok()) else {
                        continue;
                    };
                    let field = |i: usize| c.get(i).map(|s| s.trim()).filter(|s| !s.is_empty());
                    self.sats.insert(
                        (system, prn),
                        Sat {
                            system,
                            prn,
                            elevation: field(1).and_then(|s| s.parse().ok()),
                            azimuth: field(2).and_then(|s| s.parse().ok()),
                            snr: field(3).and_then(|s| s.parse().ok()),
                        },
                    );
                }
            }
            _ => {}
        }
        Ok(())
    }

    /// Satellites with a signal, and the strongest one's dB-Hz.
    pub fn heard(&self) -> (usize, Option<u8>) {
        let snrs: Vec<u8> = self
            .sats
            .values()
            .filter_map(|s| s.snr)
            .filter(|s| *s > 0)
            .collect();
        (snrs.len(), snrs.iter().max().copied())
    }

    /// "3D fix, 9 satellites used, HDOP 0.9" or "no fix".
    pub fn summary(&self) -> String {
        if !self.valid && !self.quality.is_some_and(|q| q > 0) {
            return "no fix".into();
        }
        let kind = match self.mode {
            Some(3) => "3D fix",
            Some(2) => "2D fix (height unknown)",
            _ => "fix",
        };
        let mut s = kind.to_string();
        if let Some(u) = self.used {
            s += &format!(", {u} satellites used");
        }
        if let Some(h) = self.hdop {
            s += &format!(", HDOP {h:.1}");
        }
        s
    }

    /// What the readings say about the receiver and its antenna, in words, after `seconds` of listening.
    pub fn diagnose(&self, seconds: f64) -> Vec<String> {
        let mut out = Vec::new();
        if self.sentences == 0 {
            out.push(if self.dropped > 0 {
                "Only garbled sentences: the baud rate is probably wrong (most receivers use 9600; some 4800 or 115200: \
                 add @9600 to the device)."
                    .into()
            } else {
                "No NMEA sentences at all: wrong device, the receiver is not powered, or it speaks a binary protocol \
                 only (switch it to NMEA)."
                    .into()
            });
            return out;
        }
        if self.dropped * 10 > self.sentences {
            out.push(format!(
                "{} of {} sentences had bad checksums: a loose connection, interference on the cable, or a wrong baud rate.",
                self.dropped,
                self.sentences + self.dropped
            ));
        }
        let (heard, best) = self.heard();
        let fixed = self.valid || self.quality.is_some_and(|q| q > 0);
        match (fixed, heard, best) {
            (false, 0, _) if seconds >= 60.0 => out.push(
                "No satellite heard in a minute: the antenna is disconnected or broken, its cable is damaged, or it has \
                 no view of the sky (a metal roof, a metallized windshield, a garage)."
                    .into(),
            ),
            (false, 0, _) => out.push(
                "No satellite heard yet: a receiver starting cold can take up to a minute; keep listening (--seconds 90)."
                    .into(),
            ),
            (false, _, Some(b)) if b < 25 => out.push(format!(
                "{heard} satellites heard but all weak (best {b} dB-Hz): an antenna problem (cable, connector, a passive \
                 antenna where an active one belongs) or a poor view of the sky."
            )),
            (false, _, _) => out.push(format!(
                "{heard} satellites heard with usable signals, no fix yet: still acquiring (give it a minute), or too few \
                 above the horizon here."
            )),
            (true, _, _) => {
                if self.mode == Some(2) {
                    out.push("A 2D fix only: too few satellites for height; open sky helps.".into());
                }
                match self.hdop {
                    Some(h) if h > 5.0 => out.push(format!(
                        "HDOP {h:.1}: poor geometry or reception, positions may be off by tens of meters."
                    )),
                    Some(h) if h > 2.0 => out.push(format!("HDOP {h:.1}: fair; speeds are fine, positions a little loose.")),
                    Some(_) => out.push("Good fix: positions and speed can be trusted.".into()),
                    None => {}
                }
                if let Some(b) = best.filter(|b| *b < 30) {
                    out.push(format!(
                        "The strongest signal is only {b} dB-Hz (30-45 is normal under open sky): check the antenna \
                         and its cable."
                    ));
                }
            }
        }
        out
    }
}

/// A source of NMEA lines: a receiver, a network stream, or the simulated one.
pub enum Source {
    Port(Box<dyn Port>, Vec<u8>),
    Sim(SimGps),
}

impl Source {
    /// `sim`, `/dev/ttyACM0[@baud]` (9600 unless given), `bt:AA:BB:CC:DD:EE:FF[#channel]` or `tcp:host:port`.
    pub fn open(spec: &str) -> Result<Source, String> {
        if spec == "sim" {
            return Ok(Source::Sim(SimGps::new()));
        }
        let spec = if spec.starts_with('/') && !spec.contains('@') {
            format!("{spec}@9600")
        } else {
            spec.to_string()
        };
        let t = Target::parse(&spec).map_err(|e| e.replace("adapter", "GPS source"))?;
        let p = t.open().map_err(|e| format!("{spec}: {e}"))?;
        Ok(Source::Port(p, Vec::new()))
    }

    /// The next complete line, waiting up to `timeout`.
    pub fn line(&mut self, timeout: Duration) -> Result<Option<String>, String> {
        match self {
            Source::Sim(s) => {
                // one set of sentences a second, as a receiver sends them
                let gap = Duration::from_millis(1000 / s.cycle.len() as u64);
                if timeout < gap {
                    std::thread::sleep(timeout);
                    return Ok(None);
                }
                std::thread::sleep(gap);
                Ok(Some(s.next_line()))
            }
            Source::Port(p, buf) => {
                let until = Instant::now() + timeout;
                loop {
                    if let Some(i) = buf.iter().position(|b| *b == b'\n') {
                        let l: Vec<u8> = buf.drain(..=i).collect();
                        return Ok(Some(String::from_utf8_lossy(&l).trim().to_string()));
                    }
                    if buf.len() > 4096 {
                        buf.clear(); // no line ending in 4 KB: not NMEA
                    }
                    let left = until.saturating_duration_since(Instant::now());
                    if left.is_zero() {
                        return Ok(None);
                    }
                    let mut b = [0u8; 512];
                    let n = p.recv(&mut b, left).map_err(|e| e.to_string())?;
                    buf.extend_from_slice(&b[..n]);
                }
            }
        }
    }
}

/// Listens for `seconds` and returns what the receiver said.
pub fn listen(src: &mut Source, seconds: f64) -> Result<Fix, String> {
    let mut fix = Fix::default();
    let until = Instant::now() + Duration::from_secs_f64(seconds);
    while Instant::now() < until {
        match src.line(until.saturating_duration_since(Instant::now()))? {
            Some(l) if !l.is_empty() => {
                let _ = fix.feed(&l);
            }
            _ => {}
        }
    }
    Ok(fix)
}

/// Adds the checksum to a sentence body (`GPRMC,...` -> `$GPRMC,...*hh`).
pub fn sentence(body: &str) -> String {
    format!("${body}*{:02X}", body.bytes().fold(0u8, |a, b| a ^ b))
}

/// A receiver with a good fix, at the made-up point 0° 0° (no real place), at a steady 75.6 km/h. (The simulated vehicle
/// stands still, so `icarus gps speed` on the two shows the missing-speed-signal case.)
pub struct SimGps {
    cycle: Vec<String>,
    i: usize,
}

impl Default for SimGps {
    fn default() -> Self {
        Self::new()
    }
}

impl SimGps {
    pub fn new() -> SimGps {
        let knots = 75.6 / 1.852;
        SimGps {
            cycle: vec![
                sentence(&format!(
                    "GPRMC,120000.00,A,0000.0000,N,00000.0000,E,{knots:.3},90.0,010126,,,A"
                )),
                sentence("GPVTG,90.0,T,,M,40.821,N,75.6,K,A"),
                sentence("GPGGA,120000.00,0000.0000,N,00000.0000,E,1,09,0.9,10.0,M,0.0,M,,"),
                sentence("GNGSA,A,3,02,05,07,12,15,18,24,25,29,,,,1.6,0.9,1.3"),
                sentence("GPGSV,3,1,10,02,45,060,42,05,30,120,38,07,62,200,44,12,15,300,31"),
                sentence("GPGSV,3,2,10,15,55,010,40,18,20,240,35,24,70,150,45,25,35,330,39"),
                sentence("GPGSV,3,3,10,29,10,090,28,31,05,180,"),
            ],
            i: 0,
        }
    }

    fn next_line(&mut self) -> String {
        let l = self.cycle[self.i % self.cycle.len()].clone();
        self.i += 1;
        l
    }
}

/// Speed from GPS against the vehicle's own (PID 0D), both in km/h: how far the vehicle's reading is off, in percent
/// (negative: it reads low, as bigger tires make it). None below 5 km/h, and when the vehicle says it stands still while
/// GPS has it moving (`speed_sensor_dead`): that is a fault, not a percentage.
pub fn speed_error(vehicle_kmh: f64, gps_kmh: f64) -> Option<f64> {
    (gps_kmh > 5.0 && !speed_sensor_dead(vehicle_kmh, gps_kmh))
        .then(|| (vehicle_kmh - gps_kmh) / gps_kmh * 100.0)
}

/// The vehicle reports standing still while GPS has it moving at a walking pace or more: its speed signal is missing.
pub fn speed_sensor_dead(vehicle_kmh: f64, gps_kmh: f64) -> bool {
    vehicle_kmh < 1.0 && gps_kmh > 10.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checksums_and_fields() {
        let s = sentence("GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W");
        let (t, k, f) = split(&s).unwrap();
        assert_eq!((t.as_str(), k.as_str(), f.len()), ("GP", "RMC", 11));
        let bad = s.replace("022.4", "023.4");
        assert!(split(&bad).unwrap_err().contains("checksum"));
        assert!(split("GPRMC,1,2").is_err(), "no $");
        assert!(split("$GPRMC,1,2").is_err(), "no checksum");
    }

    #[test]
    fn a_fix_from_its_sentences() {
        let mut fx = Fix::default();
        for l in SimGps::new().cycle {
            fx.feed(&l).unwrap();
        }
        assert!(fx.valid);
        assert_eq!((fx.quality, fx.mode, fx.used), (Some(1), Some(3), Some(9)));
        assert!((fx.speed_kmh.unwrap() - 75.6).abs() < 0.01);
        assert_eq!((fx.lat, fx.lon), (Some(0.0), Some(0.0)));
        assert_eq!(fx.hdop, Some(0.9));
        assert_eq!(fx.vdop, Some(1.3));
        assert_eq!(fx.sats.len(), 10);
        assert_eq!(
            fx.heard(),
            (9, Some(45)),
            "one satellite in view with no signal"
        );
        assert_eq!(fx.summary(), "3D fix, 9 satellites used, HDOP 0.9");
        assert_eq!(
            fx.diagnose(10.0),
            ["Good fix: positions and speed can be trusted."]
        );
    }

    #[test]
    fn coordinates() {
        assert!((coord("4807.038", "N").unwrap() - 48.1173).abs() < 1e-4);
        assert!((coord("01131.000", "W").unwrap() + 11.516_666).abs() < 1e-5);
        assert_eq!(coord("", "N"), None);
        assert_eq!(coord("4807.038", ""), None);
    }

    #[test]
    fn diagnosing_a_receiver() {
        let mut none = Fix::default();
        assert!(none.diagnose(5.0)[0].contains("No NMEA sentences"));
        none.dropped = 20;
        assert!(none.diagnose(5.0)[0].contains("baud rate"));
        // sentences, no fix, nothing heard: early, then after a minute the antenna
        let mut cold = Fix::default();
        cold.feed(&sentence("GPRMC,,V,,,,,,,,,,N")).unwrap();
        assert!(cold.diagnose(10.0)[0].contains("up to a minute"));
        assert!(cold.diagnose(90.0)[0].contains("antenna is disconnected"));
        // weak signals only
        cold.feed(&sentence(
            "GPGSV,1,1,03,02,45,060,18,05,30,120,21,07,62,200,15",
        ))
        .unwrap();
        assert!(cold.diagnose(90.0)[0].contains("all weak (best 21"));
        // a GLONASS group replaces only GLONASS satellites
        cold.feed(&sentence("GLGSV,1,1,01,70,40,100,33")).unwrap();
        cold.feed(&sentence("GLGSV,1,1,01,71,40,100,34")).unwrap();
        assert_eq!(cold.sats.len(), 4);
        assert!(
            cold.sats.contains_key(&("GLONASS", 71)) && !cold.sats.contains_key(&("GLONASS", 70))
        );
    }

    #[test]
    fn the_speedometer_against_gps() {
        let e = speed_error(72.0, 75.6).unwrap();
        assert!((e + 4.76).abs() < 0.01, "{e}");
        assert_eq!(speed_error(0.0, 2.0), None, "standing still says nothing");
        assert_eq!(speed_error(0.0, 75.6), None);
        assert!(speed_sensor_dead(0.0, 75.6) && !speed_sensor_dead(70.0, 75.6));
    }

    #[test]
    fn the_simulated_receiver_through_a_source() {
        let mut src = Source::open("sim").unwrap();
        let fx = listen(&mut src, 1.2).unwrap();
        assert!(
            fx.valid && (7..=9).contains(&fx.sentences),
            "one set a second: {fx:?}"
        );
        assert!(Source::open("nonsense").is_err());
    }
}
