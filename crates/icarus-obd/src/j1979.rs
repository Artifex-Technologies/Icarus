//! The standard OBD-II services (SAE J1979 / ISO 15031-5), which every vehicle sold in the US since 1996 answers: live data
//! (service 01), stored, pending and permanent trouble codes (03, 07, 0A) and the vehicle's identity (09). Everything here only
//! reads; nothing changes the vehicle.
//!
//! Each value carries a short "about" so the numbers teach something while you watch them.

use crate::elm::{Answer, Elm, ObdError};

/// One entry per answering module: its header, what it most likely is, and what it said.
pub type PerModule<T> = Vec<(String, &'static str, T)>;

pub struct Pid {
    pub pid: u8,
    pub name: &'static str,
    pub unit: &'static str,
    pub about: &'static str,
    decode: fn(&[u8]) -> Option<f64>,
}

impl Pid {
    /// The value from the data bytes that follow the PID in the answer.
    pub fn decode(&self, b: &[u8]) -> Option<f64> {
        (self.decode)(b)
    }
}

fn a(b: &[u8]) -> Option<f64> {
    b.first().map(|&v| v as f64)
}
fn ab(b: &[u8]) -> Option<f64> {
    (b.len() >= 2).then(|| (256 * b[0] as u32 + b[1] as u32) as f64)
}
fn pct(b: &[u8]) -> Option<f64> {
    a(b).map(|v| v * 100.0 / 255.0)
}
fn temp(b: &[u8]) -> Option<f64> {
    a(b).map(|v| v - 40.0)
}
fn trim(b: &[u8]) -> Option<f64> {
    a(b).map(|v| (v - 128.0) * 100.0 / 128.0)
}
fn lambda(b: &[u8]) -> Option<f64> {
    ab(b).map(|v| v * 2.0 / 65536.0)
}
fn cat_temp(b: &[u8]) -> Option<f64> {
    ab(b).map(|v| v / 10.0 - 40.0)
}
fn o2_volts(b: &[u8]) -> Option<f64> {
    a(b).map(|v| v / 200.0)
}

macro_rules! pid {
    ($pid:expr, $name:expr, $unit:expr, $f:expr, $about:expr) => {
        Pid {
            pid: $pid,
            name: $name,
            unit: $unit,
            decode: $f,
            about: $about,
        }
    };
}

/// The live values Icarus knows (service 01).
pub static PIDS: &[Pid] = &[
    pid!(0x04, "Engine load", "%", pct, "How hard the engine works compared with the most air it can take in at this speed."),
    pid!(0x05, "Coolant temperature", "°C", temp, "Engine temperature. Fuel and timing tables are corrected by it; most engines run 85-105 °C."),
    pid!(0x06, "Short-term fuel trim, bank 1", "%", trim, "The ECU's correction right now from the oxygen sensor: + adds fuel (it saw lean), - takes fuel away (rich)."),
    pid!(0x07, "Long-term fuel trim, bank 1", "%", trim, "The learned correction. Beyond about ±10 % the fuel tables (or injectors, MAF, a leak) do not match the engine."),
    pid!(0x08, "Short-term fuel trim, bank 2", "%", trim, "As bank 1, for the other cylinder bank of a V or flat engine."),
    pid!(0x09, "Long-term fuel trim, bank 2", "%", trim, "As bank 1, for the other cylinder bank."),
    pid!(0x0A, "Fuel pressure (gauge)", "kPa", |b| a(b).map(|v| 3.0 * v), "Fuel pressure above the outside air."),
    pid!(0x0B, "Intake manifold pressure (MAP)", "kPa", a, "Absolute pressure after the throttle: ~30 kPa at idle, ~100 at full throttle, above 100 with boost."),
    pid!(0x0C, "Engine speed", "rpm", |b| ab(b).map(|v| v / 4.0), "Crankshaft revolutions per minute."),
    pid!(0x0D, "Vehicle speed", "km/h", a, "Road speed as the ECU sees it (from wheel or output-shaft sensors)."),
    pid!(0x0E, "Timing advance", "° before TDC", |b| a(b).map(|v| v / 2.0 - 64.0), "When cylinder 1's spark fires, in crank degrees before top dead centre. Knock pulls it back."),
    pid!(0x0F, "Intake air temperature", "°C", temp, "Air temperature at the intake: hot air is less dense, so the ECU adds less fuel and may pull timing."),
    pid!(0x10, "Mass air flow", "g/s", |b| ab(b).map(|v| v / 100.0), "Air entering the engine per second, measured by the MAF sensor. Fuel is metered from it."),
    pid!(0x11, "Throttle position", "%", pct, "How far the throttle plate is open (sensor reading, not the pedal)."),
    pid!(0x14, "O2 sensor bank 1 sensor 1", "V", o2_volts, "Narrow-band oxygen sensor before the catalyst: ~0.1 V lean, ~0.9 V rich, switching across 0.45 V in closed loop."),
    pid!(0x15, "O2 sensor bank 1 sensor 2", "V", o2_volts, "Oxygen sensor after the catalyst: a steady signal means the catalyst stores oxygen (is working)."),
    pid!(0x18, "O2 sensor bank 2 sensor 1", "V", o2_volts, "Before the catalyst on bank 2."),
    pid!(0x19, "O2 sensor bank 2 sensor 2", "V", o2_volts, "After the catalyst on bank 2."),
    pid!(0x1F, "Run time since start", "s", ab, "Seconds since the engine started."),
    pid!(0x21, "Distance with the check-engine light on", "km", ab, "How far the vehicle went while the MIL was lit."),
    pid!(0x23, "Fuel rail pressure", "kPa", |b| ab(b).map(|v| v * 10.0), "Direct-injection or diesel rail pressure."),
    pid!(0x24, "Wide-band lambda, bank 1 sensor 1", "λ", lambda, "Air/fuel ratio as lambda: 1.00 is chemically correct (14.7:1 on petrol), below 1 rich, above 1 lean."),
    pid!(0x2C, "Commanded EGR", "%", pct, "How far the ECU opens the exhaust gas recirculation valve (cooler combustion, less NOx)."),
    pid!(0x2E, "Commanded evaporative purge", "%", pct, "How much fuel vapour from the charcoal canister the ECU lets into the intake."),
    pid!(0x2F, "Fuel level", "%", pct, "Fuel tank level."),
    pid!(0x30, "Warm-ups since codes cleared", "", a, "Engine warm-up cycles since trouble codes were last cleared (monitors need several to finish)."),
    pid!(0x31, "Distance since codes cleared", "km", ab, "Distance since trouble codes were last cleared."),
    pid!(0x33, "Barometric pressure", "kPa", a, "Outside air pressure: ~101 kPa at sea level, less at altitude (less air, less power)."),
    pid!(0x34, "Wide-band lambda (current), bank 1 sensor 1", "λ", lambda, "Air/fuel ratio from a current-type wide-band sensor."),
    pid!(0x3C, "Catalyst temperature, bank 1 sensor 1", "°C", cat_temp, "Catalytic converter temperature. Too rich or misfiring makes it climb; above ~900 °C it is being damaged."),
    pid!(0x3D, "Catalyst temperature, bank 2 sensor 1", "°C", cat_temp, "As bank 1."),
    pid!(0x3E, "Catalyst temperature, bank 1 sensor 2", "°C", cat_temp, "Further down the catalyst."),
    pid!(0x42, "Control module voltage", "V", |b| ab(b).map(|v| v / 1000.0), "The supply voltage the ECU sees: ~12.6 V engine off, 13.5-14.5 V running."),
    pid!(0x43, "Absolute load", "%", |b| ab(b).map(|v| v * 100.0 / 255.0), "Air per intake stroke compared with a full cylinder at standard conditions; above 100 % with boost."),
    pid!(0x44, "Commanded lambda", "λ", lambda, "The air/fuel ratio the ECU is aiming for: 1.00 cruising, below 1 (richer) at full load to cool the engine."),
    pid!(0x45, "Relative throttle position", "%", pct, "Throttle opening relative to its learned closed position."),
    pid!(0x46, "Ambient air temperature", "°C", temp, "Outside air temperature."),
    pid!(0x49, "Accelerator pedal position D", "%", pct, "The driver's pedal (drive-by-wire): the request, which the ECU turns into a throttle opening."),
    pid!(0x4C, "Commanded throttle", "%", pct, "The throttle opening the ECU asks for."),
    pid!(0x4D, "Time with the check-engine light on", "min", ab, "Minutes the engine ran with the MIL lit."),
    pid!(0x4E, "Time since codes cleared", "min", ab, "Engine minutes since trouble codes were last cleared."),
    pid!(0x52, "Ethanol content", "%", pct, "Flex-fuel sensor reading: more ethanol needs more fuel and allows more timing."),
    pid!(0x5C, "Engine oil temperature", "°C", temp, "Oil temperature: it lags coolant on warm-up and runs hotter under hard use."),
    pid!(0x5D, "Fuel injection timing", "°", |b| ab(b).map(|v| v / 128.0 - 210.0), "When fuel is injected, in crank degrees."),
    pid!(0x5E, "Engine fuel rate", "L/h", |b| ab(b).map(|v| v / 20.0), "Fuel used per hour, from the injector commands."),
    pid!(0x61, "Driver's demand torque", "%", |b| a(b).map(|v| v - 125.0), "Torque requested by the pedal, as a share of the engine's reference torque."),
    pid!(0x62, "Actual engine torque", "%", |b| a(b).map(|v| v - 125.0), "Torque the ECU calculates the engine makes."),
    pid!(0x63, "Engine reference torque", "Nm", ab, "The torque that the two percentages above are shares of."),
];

pub fn pid(n: u8) -> Option<&'static Pid> {
    PIDS.iter().find(|p| p.pid == n)
}

/// One value from one module.
#[derive(Clone, Debug, PartialEq)]
pub struct Reading {
    pub from: String,
    pub module: &'static str,
    pub value: f64,
}

/// Every module's answer to a live value.
pub fn read(elm: &mut Elm, n: u8) -> Result<Vec<Reading>, ObdError> {
    let p =
        pid(n).ok_or_else(|| ObdError::Garbled(format!("Icarus does not know PID {n:02X} yet")))?;
    let answers = elm.request(&[0x01, n])?;
    Ok(answers
        .iter()
        .filter(|a| a.data.len() > 2 && a.data[0] == 0x41 && a.data[1] == n)
        .filter_map(|a| {
            p.decode(&a.data[2..]).map(|value| Reading {
                from: a.from.clone(),
                module: a.module(),
                value,
            })
        })
        .collect())
}

/// The service 01 PIDs each module offers (from the PID 00/20/40... bit maps).
pub fn supported(elm: &mut Elm) -> Result<PerModule<Vec<u8>>, ObdError> {
    let mut out: PerModule<Vec<u8>> = Vec::new();
    let mut base = 0u8;
    loop {
        let answers = match elm.request(&[0x01, base]) {
            Ok(a) => a,
            Err(ObdError::NoData) if base > 0 => break,
            Err(e) => return Err(e),
        };
        let mut more = false;
        for a in answers
            .iter()
            .filter(|a| a.data.len() >= 6 && a.data[0] == 0x41 && a.data[1] == base)
        {
            let pids = bitmap(base, &a.data[2..6]);
            more |= pids.contains(&base.wrapping_add(0x20));
            match out.iter_mut().find(|o| o.0 == a.from) {
                Some(o) => o.2.extend(pids),
                None => out.push((a.from.clone(), a.module(), pids)),
            }
        }
        if !more || base >= 0xE0 {
            break;
        }
        base += 0x20;
    }
    Ok(out)
}

/// The PIDs a 4-byte bit map after PID `base` says are offered (the top bit of the first byte is base+1).
pub fn bitmap(base: u8, b: &[u8]) -> Vec<u8> {
    let mut v = Vec::new();
    for (i, byte) in b.iter().take(4).enumerate() {
        for bit in 0..8 {
            if byte & (0x80 >> bit) != 0 {
                v.push(base.wrapping_add((i * 8 + bit + 1) as u8));
            }
        }
    }
    v
}

/// A trouble code from its two bytes: P (powertrain), C (chassis), B (body), U (network), then four hex digits. P0xxx codes are
/// the same on every vehicle; P1xxx are the maker's own.
pub fn dtc(a: u8, b: u8) -> String {
    let letter = ['P', 'C', 'B', 'U'][(a >> 6) as usize];
    format!("{letter}{}{:X}{:02X}", (a >> 4) & 3, a & 0x0F, b)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DtcKind {
    /// Service 03: confirmed codes (the ones that light the MIL).
    Stored,
    /// Service 07: seen once this drive cycle, not yet confirmed.
    Pending,
    /// Service 0A: kept until the monitor itself passes again; clearing codes does not remove them.
    Permanent,
}

impl DtcKind {
    fn service(self) -> u8 {
        match self {
            DtcKind::Stored => 0x03,
            DtcKind::Pending => 0x07,
            DtcKind::Permanent => 0x0A,
        }
    }
}

/// Trouble codes per module.
pub fn dtcs(elm: &mut Elm, kind: DtcKind) -> Result<PerModule<Vec<String>>, ObdError> {
    let sid = kind.service();
    let answers = match elm.request(&[sid]) {
        Ok(a) => a,
        Err(ObdError::NoData) => return Ok(vec![]),
        Err(e) => return Err(e),
    };
    Ok(parse_dtcs(&answers, sid, elm.protocol.is_can()))
}

pub fn parse_dtcs(answers: &[Answer], sid: u8, can: bool) -> PerModule<Vec<String>> {
    let mut out: PerModule<Vec<String>> = Vec::new();
    for a in answers
        .iter()
        .filter(|a| a.data.first() == Some(&(sid + 0x40)))
    {
        // on CAN a count byte comes first; on older buses every answer holds three codes, padded with 00 00
        let body = if can {
            a.data.get(2..).unwrap_or(&[])
        } else {
            &a.data[1..]
        };
        let codes: Vec<String> = body
            .chunks_exact(2)
            .filter(|c| c != &[0, 0])
            .map(|c| dtc(c[0], c[1]))
            .collect();
        match out.iter_mut().find(|o| o.0 == a.from) {
            Some(o) => o.2.extend(codes),
            None => out.push((a.from.clone(), a.module(), codes)),
        }
    }
    out
}

/// A freeze frame (service 02): the values the engine module saved at the moment a code set the check-engine light, and
/// the code that caused it. The best clue to an intermittent fault: speed, load, temperature and fuel trims at that moment.
#[derive(Clone, Debug, PartialEq)]
pub struct FreezeFrame {
    pub from: String,
    pub module: &'static str,
    pub code: Option<String>,
    /// (PID, value) for the values Icarus can read.
    pub values: Vec<(u8, f64)>,
}

/// The freeze frame (frame 0) of each module that has one.
pub fn freeze_frame(elm: &mut Elm) -> Result<Vec<FreezeFrame>, ObdError> {
    let mut out: Vec<FreezeFrame> = Vec::new();
    // which values each module froze (the PID 00/20/40... bit maps, asked of frame 0)
    let mut base = 0u8;
    loop {
        let answers = match elm.request(&[0x02, base, 0x00]) {
            Ok(a) => a,
            Err(ObdError::NoData) => break,
            Err(e) => return Err(e),
        };
        let mut more = false;
        for a in answers
            .iter()
            .filter(|a| a.data.len() >= 7 && a.data[0] == 0x42 && a.data[1] == base)
        {
            let pids = bitmap(base, &a.data[3..7]);
            more |= pids.contains(&base.wrapping_add(0x20));
            let f = match out.iter_mut().position(|f| f.from == a.from) {
                Some(i) => &mut out[i],
                None => {
                    out.push(FreezeFrame {
                        from: a.from.clone(),
                        module: a.module(),
                        code: None,
                        values: Vec::new(),
                    });
                    out.last_mut().unwrap()
                }
            };
            f.values.extend(
                pids.into_iter()
                    .filter(|p| pid(*p).is_some() || *p == 0x02)
                    .map(|p| (p, f64::NAN)),
            );
        }
        if !more || base >= 0xE0 {
            break;
        }
        base += 0x20;
    }
    for f in out.iter_mut() {
        let wanted: Vec<u8> = f.values.iter().map(|v| v.0).collect();
        f.values.clear();
        for p in wanted {
            let answers = match elm.request(&[0x02, p, 0x00]) {
                Ok(a) => a,
                Err(ObdError::NoData) => continue,
                Err(e) => return Err(e),
            };
            let Some(a) = answers.iter().find(|a| {
                a.from == f.from && a.data.len() > 3 && a.data[0] == 0x42 && a.data[1] == p
            }) else {
                continue;
            };
            if p == 0x02 {
                if a.data.len() >= 5 && a.data[3..5] != [0, 0] {
                    f.code = Some(dtc(a.data[3], a.data[4]));
                }
            } else if let Some(v) = pid(p).and_then(|d| d.decode(&a.data[3..])) {
                f.values.push((p, v));
            }
        }
    }
    out.retain(|f| f.code.is_some() || !f.values.is_empty());
    Ok(out)
}

/// What service 01 PID 01 says: the check-engine light, the code count, and which emissions self-tests (monitors) have run.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Monitors {
    pub mil: bool,
    pub codes: u8,
    pub diesel: bool,
    /// (name, complete) for each monitor this vehicle has.
    pub tests: Vec<(&'static str, bool)>,
}

pub fn monitors(b: &[u8]) -> Option<Monitors> {
    if b.len() < 4 {
        return None;
    }
    let diesel = b[1] & 0x08 != 0;
    let mut tests = Vec::new();
    for (bit, name) in [
        (0, "misfire"),
        (1, "fuel system"),
        (2, "comprehensive components"),
    ] {
        if b[1] & (1 << bit) != 0 {
            tests.push((name, b[1] & (0x10 << bit) == 0));
        }
    }
    let names: [&str; 8] = if diesel {
        [
            "NMHC catalyst",
            "NOx / SCR",
            "",
            "boost pressure",
            "",
            "exhaust gas sensor",
            "particulate filter",
            "EGR / VVT",
        ]
    } else {
        [
            "catalyst",
            "heated catalyst",
            "evaporative system",
            "secondary air",
            "A/C refrigerant",
            "oxygen sensor",
            "oxygen sensor heater",
            "EGR / VVT",
        ]
    };
    for (bit, name) in names.iter().enumerate() {
        if !name.is_empty() && b[2] & (1 << bit) != 0 {
            tests.push((name, b[3] & (1 << bit) == 0));
        }
    }
    Some(Monitors {
        mil: b[0] & 0x80 != 0,
        codes: b[0] & 0x7F,
        diesel,
        tests,
    })
}

/// PID 03, fuel system status: whether the ECU is using the oxygen sensor (closed loop) or its tables alone (open loop).
pub fn fuel_system(a: u8) -> &'static str {
    match a {
        0 => "not running",
        1 => "open loop: engine still cold",
        2 => "closed loop: correcting fuel from the oxygen sensor",
        4 => "open loop: high load or deceleration fuel cut (the tables alone set the fuel)",
        8 => "open loop: a fault (sensor or system) stops closed loop",
        16 => "closed loop, but a fault in at least one oxygen sensor",
        _ => "unknown",
    }
}

/// Text from service 09 answers (VIN, calibration IDs, ECU name): the printable characters, padding removed; older buses send
/// it four bytes per numbered message, which join up the same way.
fn text_items(answers: &[Answer], pid: u8, item_len: usize) -> PerModule<Vec<String>> {
    let mut out: PerModule<Vec<u8>> = Vec::new();
    for a in answers
        .iter()
        .filter(|a| a.data.len() > 2 && a.data[0] == 0x49 && a.data[1] == pid)
    {
        // byte 2 is the item count on CAN and the message number on older buses: either way the text starts after it
        let body = a.data.get(3..).unwrap_or(&[]);
        match out.iter_mut().find(|o| o.0 == a.from) {
            Some(o) => o.2.extend_from_slice(body),
            None => out.push((a.from.clone(), a.module(), body.to_vec())),
        }
    }
    out.into_iter()
        .map(|(from, module, bytes)| {
            let items = bytes
                .chunks(item_len)
                .map(|c| {
                    c.iter()
                        .filter(|b| b.is_ascii_graphic() || **b == b' ')
                        .map(|&b| b as char)
                        .collect::<String>()
                        .trim()
                        .to_string()
                })
                .filter(|s| !s.is_empty())
                .collect();
            (from, module, items)
        })
        .collect()
}

/// The vehicle identification number (service 09 PID 02).
pub fn vin(elm: &mut Elm) -> Result<Option<String>, ObdError> {
    let answers = match elm.request(&[0x09, 0x02]) {
        Ok(a) => a,
        Err(ObdError::NoData) => return Ok(None),
        Err(e) => return Err(e),
    };
    Ok(text_items(&answers, 0x02, 64)
        .into_iter()
        .flat_map(|t| t.2)
        .map(|s| s.replace(' ', ""))
        .find(|s| s.len() >= 17)
        .map(|s| s[s.len() - 17..].to_string()))
}

/// Each module's calibration IDs (service 09 PID 04): the name of the software calibration it runs, which is how a tune or
/// a definition file is matched to an ECU.
pub fn calibration_ids(elm: &mut Elm) -> Result<PerModule<Vec<String>>, ObdError> {
    match elm.request(&[0x09, 0x04]) {
        Ok(a) => Ok(text_items(&a, 0x04, 16)),
        Err(ObdError::NoData) => Ok(vec![]),
        Err(e) => Err(e),
    }
}

/// Each module's name (service 09 PID 0A), e.g. "ECM-EngineControl".
pub fn ecu_names(elm: &mut Elm) -> Result<PerModule<Vec<String>>, ObdError> {
    match elm.request(&[0x09, 0x0A]) {
        Ok(a) => Ok(text_items(&a, 0x0A, 20)),
        Err(ObdError::NoData) => Ok(vec![]),
        Err(e) => Err(e),
    }
}

/// Calibration verification numbers (service 09 PID 06): a checksum of each calibration, which changes with any edit.
pub fn cvns(elm: &mut Elm) -> Result<Vec<(String, Vec<String>)>, ObdError> {
    let answers = match elm.request(&[0x09, 0x06]) {
        Ok(a) => a,
        Err(ObdError::NoData) => return Ok(vec![]),
        Err(e) => return Err(e),
    };
    Ok(answers
        .iter()
        .filter(|a| a.data.len() > 3 && a.data[0] == 0x49 && a.data[1] == 0x06)
        .map(|a| {
            (
                a.from.clone(),
                a.data[3..]
                    .chunks_exact(4)
                    .map(|c| c.iter().map(|b| format!("{b:02X}")).collect())
                    .collect(),
            )
        })
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn standard_formulas() {
        assert_eq!(pid(0x0C).unwrap().decode(&[0x1A, 0xF8]), Some(1726.0));
        assert_eq!(pid(0x05).unwrap().decode(&[0x7B]), Some(83.0));
        assert_eq!(pid(0x06).unwrap().decode(&[0x80]), Some(0.0));
        assert_eq!(pid(0x07).unwrap().decode(&[0x8A]), Some(7.8125));
        assert_eq!(pid(0x0E).unwrap().decode(&[0x94]), Some(10.0));
        assert_eq!(pid(0x44).unwrap().decode(&[0x80, 0x00]), Some(1.0));
        assert_eq!(pid(0x3C).unwrap().decode(&[0x1F, 0x40]), Some(760.0));
        assert_eq!(
            pid(0x0C).unwrap().decode(&[0x1A]),
            None,
            "too short is no value, not a wrong one"
        );
        let mut seen = std::collections::HashSet::new();
        assert!(PIDS.iter().all(|p| seen.insert(p.pid)), "each PID once");
    }

    #[test]
    fn bit_maps_and_codes() {
        assert_eq!(bitmap(0x00, &[0x80, 0x00, 0x00, 0x01]), vec![0x01, 0x20]);
        assert_eq!(bitmap(0x20, &[0x40, 0, 0, 0]), vec![0x22]);
        assert_eq!(dtc(0x01, 0x71), "P0171");
        assert_eq!(dtc(0x03, 0x01), "P0301");
        assert_eq!(dtc(0x41, 0x23), "C0123");
        assert_eq!(dtc(0xC1, 0x00), "U0100");
        assert_eq!(dtc(0x12, 0x34), "P1234");
    }

    #[test]
    fn trouble_codes_on_can_and_k_line() {
        let can = [Answer {
            from: "7E8".into(),
            data: vec![0x43, 0x02, 0x01, 0x71, 0x03, 0x01],
        }];
        assert_eq!(
            parse_dtcs(&can, 0x03, true),
            vec![(
                "7E8".to_string(),
                "engine (ECM/PCM)",
                vec!["P0171".to_string(), "P0301".to_string()]
            )]
        );
        let k = [Answer {
            from: "486B10".into(),
            data: vec![0x43, 0x01, 0x71, 0x00, 0x00, 0x00, 0x00],
        }];
        assert_eq!(parse_dtcs(&k, 0x03, false)[0].2, vec!["P0171".to_string()]);
    }

    #[test]
    fn readiness_monitors() {
        // MIL on, 2 codes; spark ignition: misfire+fuel+components available, components incomplete; catalyst, evap, O2,
        // O2 heater available; evap incomplete
        let m = monitors(&[0x82, 0x47, 0x65, 0x04]).unwrap();
        assert!(m.mil && !m.diesel);
        assert_eq!(m.codes, 2);
        assert!(m.tests.contains(&("comprehensive components", false)));
        assert!(m.tests.contains(&("misfire", true)));
        assert!(m.tests.contains(&("evaporative system", false)));
        assert!(m.tests.contains(&("catalyst", true)));
        assert!(!m.tests.iter().any(|t| t.0 == "secondary air"));
        assert_eq!(
            fuel_system(2),
            "closed loop: correcting fuel from the oxygen sensor"
        );
    }

    #[test]
    fn identity_text() {
        let can = [Answer {
            from: "7E8".into(),
            data: [&[0x49, 0x04, 0x01][..], b"CAL-ONE\0\0\0\0\0\0\0\0\0"].concat(),
        }];
        assert_eq!(text_items(&can, 0x04, 16)[0].2, vec!["CAL-ONE".to_string()]);
    }
}
