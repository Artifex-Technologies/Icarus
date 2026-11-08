//! What a trouble code means. A code is a letter and four characters: the letter is the system (P powertrain, C chassis,
//! B body, U network), the first digit says whether the meaning is standard (the same on every vehicle) or the maker's own,
//! and for standard powertrain codes the next digit names the subsystem.
//!
//! The standard meanings here were written for Icarus in plain words (the numbering follows SAE J2012 / ISO 15031-6); the
//! numbered families (per cylinder, per oxygen sensor) are generated from their pattern. The maker's own codes come from the
//! vehicle profile's `[codes]` table, which anyone can extend in `~/.config/icarus/vehicles/`.

use crate::vehicles::Profile;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CodeInfo {
    pub code: String,
    /// "powertrain (engine, transmission)" ...
    pub system: &'static str,
    /// True when the meaning is the same on every vehicle.
    pub standard: bool,
    /// For standard P0 codes: the subsystem the third character names.
    pub area: Option<&'static str>,
    pub text: Option<String>,
    /// Where to start looking, for the common codes.
    pub hint: Option<&'static str>,
    /// Where the text came from ("standard" or the profile's name).
    pub source: String,
}

pub fn explain(code: &str, profile: Option<&Profile>) -> Result<CodeInfo, String> {
    let c = code.trim().to_uppercase();
    let b = c.as_bytes();
    if b.len() != 5
        || !matches!(b[0], b'P' | b'C' | b'B' | b'U')
        || !c[1..].chars().all(|x| x.is_ascii_hexdigit())
        || b[1] > b'3'
    {
        return Err(format!("{code:?} is not a trouble code (a letter P, C, B or U and four characters, e.g. P0171)"));
    }
    let system = match b[0] {
        b'P' => "powertrain (engine, transmission)",
        b'C' => "chassis (brakes, steering, suspension)",
        b'B' => "body (airbags, climate, lights, doors)",
        _ => "network (communication between modules)",
    };
    // which ranges are standard (SAE J2012): P0, P2, P34-P39, B0/C0/U0, B3/C3/U3; the maker's: P1, P30-P33, B1-B2, C1-C2, U1-U2
    let standard = match (b[0], b[1]) {
        (b'P', b'0' | b'2') => true,
        (b'P', b'3') => b[2] >= b'4',
        (b'P', _) => false,
        (_, b'0' | b'3') => true,
        _ => false,
    };
    let area = (b[0] == b'P' && b[1] == b'0').then(|| match b[2] {
        b'0' => "fuel and air metering, auxiliary emission controls",
        b'1' | b'2' => "fuel and air metering",
        b'3' => "ignition system or misfire",
        b'4' => "auxiliary emission controls (EGR, secondary air, catalyst, evaporative system)",
        b'5' => "vehicle speed, idle control and auxiliary inputs",
        b'6' => "computer and output circuits",
        b'7' | b'8' | b'9' => "transmission",
        _ => "hybrid propulsion",
    });
    let from_profile = profile.and_then(|p| p.codes.get(&c).map(|t| (t.clone(), p.name.clone())));
    let (text, source) = match from_profile {
        Some((t, src)) => (Some(t), src),
        None if standard => (standard_text(&c), "standard".to_string()),
        None => (None, String::new()),
    };
    Ok(CodeInfo {
        hint: if standard { hint(&c) } else { None },
        code: c,
        system,
        standard,
        area,
        text,
        source,
    })
}

/// The anatomy of a code in one line, for teaching.
pub fn anatomy(i: &CodeInfo) -> String {
    format!(
        "{}: {}; {}{}",
        &i.code[..1],
        i.system,
        if i.standard {
            "a standard code (same meaning on every vehicle)"
        } else {
            "the maker's own code (its meaning depends on the make)"
        },
        i.area
            .map(|a| format!("; subsystem: {a}"))
            .unwrap_or_default()
    )
}

const SENSORS: [(u16, &str); 6] = [
    (130, "bank 1 sensor 1"),
    (136, "bank 1 sensor 2"),
    (142, "bank 1 sensor 3"),
    (150, "bank 2 sensor 1"),
    (156, "bank 2 sensor 2"),
    (162, "bank 2 sensor 3"),
];
const O2_FAULTS: [&str; 6] = [
    "circuit",
    "circuit low voltage",
    "circuit high voltage",
    "slow response",
    "no activity detected",
    "heater circuit",
];

fn standard_text(c: &str) -> Option<String> {
    if let Some((_, t)) = GENERIC.iter().find(|(k, _)| *k == c) {
        return Some(t.to_string());
    }
    // numbered families: only all-decimal P0 codes
    if !c.starts_with("P0") || !c[2..].chars().all(|x| x.is_ascii_digit()) {
        return None;
    }
    let n: u16 = c[2..].parse().ok()?;
    for (base, where_) in SENSORS {
        if (base..base + 6).contains(&n) {
            return Some(format!(
                "Oxygen sensor {} ({where_})",
                O2_FAULTS[(n - base) as usize]
            ));
        }
    }
    match n {
        201..=212 => Some(format!("Injector circuit, cylinder {}", n - 200)),
        261..=296 => {
            let cyl = (n - 261) / 3 + 1;
            let kind =
                ["circuit low", "circuit high", "contribution/balance"][((n - 261) % 3) as usize];
            Some(format!("Injector {kind}, cylinder {cyl}"))
        }
        301..=312 => Some(format!("Misfire detected, cylinder {}", n - 300)),
        351..=362 => Some(format!(
            "Ignition coil {} primary/secondary circuit (usually cylinder {})",
            (b'A' + (n - 351) as u8) as char,
            n - 350
        )),
        _ => None,
    }
}

fn hint(c: &str) -> Option<&'static str> {
    Some(match c {
        "P0171" | "P0174" => "Too much air or too little fuel. Common: a vacuum leak after the airflow sensor, a dirty or failing MAF sensor, low fuel pressure (pump, filter). In live data: long-term trim high at idle but near 0 at 2500 rpm points to a vacuum leak; high everywhere points to fuel delivery or the MAF.",
        "P0172" | "P0175" => "Too much fuel. Common: a leaking injector, high fuel pressure, a coolant sensor reading cold, a purge valve stuck open, a MAF reading high.",
        "P0300" | "P0301" | "P0302" | "P0303" | "P0304" | "P0305" | "P0306" | "P0307" | "P0308" => "Swap the named cylinder's coil (or plug) with a neighbour's and clear the code: if the misfire follows the part, that part is bad. Otherwise the injector, a vacuum leak at that cylinder, or compression. Fix misfires soon: unburnt fuel overheats the catalyst.",
        "P0420" | "P0430" => "The sensor after the catalyst switches almost like the one before it: the catalyst no longer stores oxygen. Rule out exhaust leaks and a lazy rear sensor first, and fix misfires or rich running (they kill catalysts).",
        "P0128" => "The engine warms up too slowly: usually a thermostat stuck open, sometimes the coolant temperature sensor.",
        "P0440" | "P0442" | "P0455" | "P0456" | "P0457" => "Start with the fuel cap (its seal, and that it clicks). Then the evaporative hoses and the purge and vent valves; a smoke test finds small leaks.",
        "P0700" => "This only says the transmission module has a code of its own: read it on the Modules page (or icarus obd scan).",
        "P0562" | "P0563" => "Check the battery, the alternator's output (13.5-14.5 V running) and the battery terminals and grounds.",
        "P0101" | "P0102" | "P0103" => "Check for air leaks between the MAF sensor and the throttle, clean the sensor with MAF cleaner, check its connector.",
        "P0016" | "P0017" | "P0018" | "P0019" => "Cam and crank do not line up as expected: low or old oil (variable valve timing runs on it), a stretched timing chain, or a sensor.",
        "U0100" | "U0101" | "U0121" | "U0140" | "U0151" | "U0155" => "A module stopped answering on the network: check that module's fuses, power and ground, then the wiring and connectors of the CAN bus. Low battery voltage can cause it too.",
        _ => return None,
    })
}

/// Parts most often involved, most likely first: what to look up when shopping (a starting point, not a diagnosis; test
/// before buying).
pub fn parts(code: &str) -> &'static [&'static str] {
    let c = code.trim().to_uppercase();
    if c.len() != 5 || !c.starts_with('P') {
        return &[];
    }
    let n: Option<u16> = if c[1..].chars().all(|x| x.is_ascii_digit()) {
        c[1..].parse().ok()
    } else {
        None
    };
    let Some(n) = n else { return &[] };
    match n {
        10..=15 | 20..=22 => &[
            "variable valve timing (VVT) solenoid",
            "engine oil and filter",
        ],
        16..=19 => &[
            "timing chain kit",
            "camshaft position sensor",
            "variable valve timing (VVT) solenoid",
        ],
        30..=58 | 135 | 141 | 147 | 155 | 161 | 167 => &["oxygen sensor"],
        87 | 190..=193 => &["fuel rail pressure sensor", "fuel pump", "fuel filter"],
        100..=104 => &["mass air flow (MAF) sensor", "air intake hose"],
        105..=108 => &["MAP sensor"],
        110..=113 => &["intake air temperature sensor"],
        115..=119 => &["coolant temperature sensor"],
        120..=123 | 220..=223 | 2135 | 2138 => &[
            "throttle body",
            "throttle position sensor",
            "accelerator pedal sensor",
        ],
        125 | 128 => &["thermostat", "coolant temperature sensor"],
        130..=167 | 2195..=2198 | 2270 | 2271 => &["oxygen sensor"],
        171 | 174 => &[
            "intake manifold gasket",
            "PCV valve and hose",
            "mass air flow (MAF) sensor",
            "fuel filter",
            "fuel pump",
        ],
        172 | 175 => &[
            "fuel injector",
            "fuel pressure regulator",
            "coolant temperature sensor",
            "EVAP purge valve",
        ],
        200..=212 | 261..=296 => &["fuel injector", "injector connector"],
        217 => &["thermostat", "water pump", "radiator", "radiator cap"],
        230 => &["fuel pump relay", "fuel pump"],
        234 | 299 => &[
            "turbocharger",
            "boost control (wastegate) solenoid",
            "intercooler hose",
        ],
        300 => &["spark plugs", "ignition coil"],
        301..=312 => &["spark plug", "ignition coil", "fuel injector"],
        325..=333 => &["knock sensor"],
        335..=339 => &["crankshaft position sensor"],
        340..=349 => &["camshaft position sensor"],
        350..=362 => &["ignition coil", "spark plug"],
        400..=404 => &["EGR valve"],
        410 | 411 => &["secondary air injection pump", "secondary air check valve"],
        420 | 421 | 430 | 431 => &["catalytic converter", "oxygen sensor (after the catalyst)"],
        440 | 442 | 455..=457 => &[
            "gas cap",
            "EVAP vent valve",
            "EVAP purge valve",
            "EVAP canister",
        ],
        441 | 443 => &["EVAP purge valve"],
        446 | 449 => &["EVAP vent valve"],
        451..=453 => &["fuel tank pressure sensor"],
        460..=463 => &["fuel level sending unit"],
        480 | 481 => &["cooling fan relay", "cooling fan"],
        500..=503 => &["vehicle speed sensor"],
        505..=507 => &["idle air control valve", "throttle body"],
        520..=523 => &["oil pressure sensor"],
        562 | 563 => &["alternator", "battery"],
        571 => &["brake light switch"],
        705 => &["transmission range sensor"],
        710..=713 => &["transmission fluid temperature sensor"],
        715..=717 => &["transmission input speed sensor"],
        720..=722 => &["transmission output speed sensor"],
        740..=744 => &["torque converter clutch solenoid", "transmission fluid"],
        750..=770 => &["transmission shift solenoid", "transmission fluid"],
        _ => &[],
    }
}

/// Standard meanings not covered by a numbered family.
const GENERIC: &[(&str, &str)] = &[
    ("P0010", "Camshaft position actuator A circuit (bank 1)"),
    (
        "P0011",
        "Camshaft position A: timing over-advanced or system performance (bank 1)",
    ),
    (
        "P0012",
        "Camshaft position A: timing over-retarded (bank 1)",
    ),
    ("P0013", "Camshaft position actuator B circuit (bank 1)"),
    (
        "P0014",
        "Camshaft position B: timing over-advanced or system performance (bank 1)",
    ),
    (
        "P0015",
        "Camshaft position B: timing over-retarded (bank 1)",
    ),
    (
        "P0016",
        "Crankshaft/camshaft position correlation (bank 1, sensor A)",
    ),
    (
        "P0017",
        "Crankshaft/camshaft position correlation (bank 1, sensor B)",
    ),
    (
        "P0018",
        "Crankshaft/camshaft position correlation (bank 2, sensor A)",
    ),
    (
        "P0019",
        "Crankshaft/camshaft position correlation (bank 2, sensor B)",
    ),
    ("P0020", "Camshaft position actuator A circuit (bank 2)"),
    (
        "P0021",
        "Camshaft position A: timing over-advanced or system performance (bank 2)",
    ),
    (
        "P0022",
        "Camshaft position A: timing over-retarded (bank 2)",
    ),
    (
        "P0030",
        "Oxygen sensor heater control circuit (bank 1 sensor 1)",
    ),
    (
        "P0031",
        "Oxygen sensor heater control circuit low (bank 1 sensor 1)",
    ),
    (
        "P0032",
        "Oxygen sensor heater control circuit high (bank 1 sensor 1)",
    ),
    (
        "P0036",
        "Oxygen sensor heater control circuit (bank 1 sensor 2)",
    ),
    (
        "P0037",
        "Oxygen sensor heater control circuit low (bank 1 sensor 2)",
    ),
    (
        "P0038",
        "Oxygen sensor heater control circuit high (bank 1 sensor 2)",
    ),
    (
        "P0050",
        "Oxygen sensor heater control circuit (bank 2 sensor 1)",
    ),
    (
        "P0051",
        "Oxygen sensor heater control circuit low (bank 2 sensor 1)",
    ),
    (
        "P0052",
        "Oxygen sensor heater control circuit high (bank 2 sensor 1)",
    ),
    (
        "P0056",
        "Oxygen sensor heater control circuit (bank 2 sensor 2)",
    ),
    (
        "P0057",
        "Oxygen sensor heater control circuit low (bank 2 sensor 2)",
    ),
    (
        "P0058",
        "Oxygen sensor heater control circuit high (bank 2 sensor 2)",
    ),
    (
        "P0068",
        "Manifold pressure / airflow and throttle position do not agree",
    ),
    ("P0087", "Fuel rail/system pressure too low"),
    ("P0088", "Fuel rail/system pressure too high"),
    ("P0100", "Mass air flow (MAF) sensor circuit"),
    ("P0101", "Mass air flow (MAF) sensor range/performance"),
    ("P0102", "Mass air flow (MAF) sensor circuit low"),
    ("P0103", "Mass air flow (MAF) sensor circuit high"),
    ("P0104", "Mass air flow (MAF) sensor circuit intermittent"),
    (
        "P0105",
        "Manifold absolute pressure (MAP) / barometric pressure circuit",
    ),
    (
        "P0106",
        "Manifold absolute pressure (MAP) / barometric pressure range/performance",
    ),
    (
        "P0107",
        "Manifold absolute pressure (MAP) / barometric pressure circuit low",
    ),
    (
        "P0108",
        "Manifold absolute pressure (MAP) / barometric pressure circuit high",
    ),
    ("P0110", "Intake air temperature sensor circuit"),
    ("P0111", "Intake air temperature sensor range/performance"),
    ("P0112", "Intake air temperature sensor circuit low"),
    ("P0113", "Intake air temperature sensor circuit high"),
    ("P0115", "Engine coolant temperature sensor circuit"),
    (
        "P0116",
        "Engine coolant temperature sensor range/performance",
    ),
    ("P0117", "Engine coolant temperature sensor circuit low"),
    ("P0118", "Engine coolant temperature sensor circuit high"),
    (
        "P0119",
        "Engine coolant temperature sensor circuit intermittent",
    ),
    ("P0120", "Throttle/pedal position sensor A circuit"),
    (
        "P0121",
        "Throttle/pedal position sensor A range/performance",
    ),
    ("P0122", "Throttle/pedal position sensor A circuit low"),
    ("P0123", "Throttle/pedal position sensor A circuit high"),
    (
        "P0125",
        "Coolant too cold for closed-loop fuel control (the engine takes too long to warm up)",
    ),
    (
        "P0128",
        "Coolant temperature below the thermostat's regulating temperature",
    ),
    ("P0170", "Fuel trim (bank 1)"),
    ("P0171", "System too lean (bank 1)"),
    ("P0172", "System too rich (bank 1)"),
    ("P0173", "Fuel trim (bank 2)"),
    ("P0174", "System too lean (bank 2)"),
    ("P0175", "System too rich (bank 2)"),
    ("P0190", "Fuel rail pressure sensor circuit"),
    ("P0191", "Fuel rail pressure sensor range/performance"),
    ("P0192", "Fuel rail pressure sensor circuit low"),
    ("P0193", "Fuel rail pressure sensor circuit high"),
    ("P0200", "Injector circuit"),
    ("P0217", "Engine coolant over-temperature"),
    ("P0218", "Transmission fluid over-temperature"),
    ("P0219", "Engine over-speed"),
    ("P0220", "Throttle/pedal position sensor B circuit"),
    (
        "P0221",
        "Throttle/pedal position sensor B range/performance",
    ),
    ("P0222", "Throttle/pedal position sensor B circuit low"),
    ("P0223", "Throttle/pedal position sensor B circuit high"),
    ("P0230", "Fuel pump primary circuit"),
    ("P0234", "Turbocharger/supercharger overboost"),
    ("P0299", "Turbocharger/supercharger underboost"),
    ("P0300", "Random or multiple cylinder misfire detected"),
    ("P0313", "Misfire detected with low fuel"),
    ("P0314", "Single cylinder misfire (cylinder not named)"),
    (
        "P0316",
        "Misfire detected on start-up (first 1000 revolutions)",
    ),
    (
        "P0325",
        "Knock sensor 1 circuit (bank 1 or the only sensor)",
    ),
    ("P0326", "Knock sensor 1 range/performance"),
    ("P0327", "Knock sensor 1 circuit low"),
    ("P0328", "Knock sensor 1 circuit high"),
    ("P0330", "Knock sensor 2 circuit (bank 2)"),
    ("P0332", "Knock sensor 2 circuit low"),
    ("P0333", "Knock sensor 2 circuit high"),
    ("P0335", "Crankshaft position sensor A circuit"),
    ("P0336", "Crankshaft position sensor A range/performance"),
    ("P0337", "Crankshaft position sensor A circuit low"),
    ("P0338", "Crankshaft position sensor A circuit high"),
    ("P0339", "Crankshaft position sensor A circuit intermittent"),
    (
        "P0340",
        "Camshaft position sensor A circuit (bank 1 or the only sensor)",
    ),
    (
        "P0341",
        "Camshaft position sensor A range/performance (bank 1)",
    ),
    ("P0342", "Camshaft position sensor A circuit low (bank 1)"),
    ("P0343", "Camshaft position sensor A circuit high (bank 1)"),
    ("P0345", "Camshaft position sensor A circuit (bank 2)"),
    ("P0350", "Ignition coil primary/secondary circuit"),
    ("P0400", "Exhaust gas recirculation (EGR) flow"),
    ("P0401", "Exhaust gas recirculation (EGR) flow insufficient"),
    ("P0402", "Exhaust gas recirculation (EGR) flow excessive"),
    ("P0403", "Exhaust gas recirculation (EGR) control circuit"),
    (
        "P0404",
        "Exhaust gas recirculation (EGR) control circuit range/performance",
    ),
    ("P0410", "Secondary air injection system"),
    ("P0411", "Secondary air injection: incorrect flow detected"),
    ("P0420", "Catalyst efficiency below threshold (bank 1)"),
    (
        "P0421",
        "Warm-up catalyst efficiency below threshold (bank 1)",
    ),
    ("P0430", "Catalyst efficiency below threshold (bank 2)"),
    (
        "P0431",
        "Warm-up catalyst efficiency below threshold (bank 2)",
    ),
    ("P0440", "Evaporative emission system"),
    ("P0441", "Evaporative emission system: incorrect purge flow"),
    ("P0442", "Evaporative emission system: small leak detected"),
    (
        "P0443",
        "Evaporative emission system: purge control valve circuit",
    ),
    ("P0446", "Evaporative emission system: vent control circuit"),
    (
        "P0451",
        "Evaporative emission system: pressure sensor range/performance",
    ),
    ("P0452", "Evaporative emission system: pressure sensor low"),
    ("P0453", "Evaporative emission system: pressure sensor high"),
    (
        "P0455",
        "Evaporative emission system: large leak detected (or no flow)",
    ),
    (
        "P0456",
        "Evaporative emission system: very small leak detected",
    ),
    (
        "P0457",
        "Evaporative emission system: leak detected (fuel cap loose or off)",
    ),
    ("P0460", "Fuel level sensor A circuit"),
    ("P0461", "Fuel level sensor A range/performance"),
    ("P0462", "Fuel level sensor A circuit low"),
    ("P0463", "Fuel level sensor A circuit high"),
    ("P0480", "Cooling fan 1 control circuit"),
    ("P0481", "Cooling fan 2 control circuit"),
    ("P0500", "Vehicle speed sensor A"),
    ("P0501", "Vehicle speed sensor A range/performance"),
    ("P0502", "Vehicle speed sensor A circuit low"),
    (
        "P0503",
        "Vehicle speed sensor A intermittent, erratic or high",
    ),
    ("P0505", "Idle air control system"),
    ("P0506", "Idle control system: rpm lower than expected"),
    ("P0507", "Idle control system: rpm higher than expected"),
    ("P0520", "Engine oil pressure sensor/switch circuit"),
    (
        "P0521",
        "Engine oil pressure sensor/switch range/performance",
    ),
    ("P0522", "Engine oil pressure sensor/switch circuit low"),
    ("P0523", "Engine oil pressure sensor/switch circuit high"),
    ("P0560", "System voltage"),
    ("P0562", "System voltage low"),
    ("P0563", "System voltage high"),
    ("P0571", "Brake switch A circuit"),
    ("P0600", "Serial communication link"),
    ("P0601", "Control module internal memory checksum error"),
    ("P0602", "Control module programming error"),
    ("P0603", "Control module keep-alive memory (KAM) error"),
    ("P0604", "Control module RAM error"),
    ("P0605", "Control module ROM error"),
    ("P0606", "Control module processor"),
    (
        "P0700",
        "Transmission control system: the transmission module has a code (MIL request)",
    ),
    ("P0705", "Transmission range sensor A circuit (PRNDL input)"),
    ("P0710", "Transmission fluid temperature sensor A circuit"),
    (
        "P0711",
        "Transmission fluid temperature sensor A range/performance",
    ),
    (
        "P0712",
        "Transmission fluid temperature sensor A circuit low",
    ),
    (
        "P0713",
        "Transmission fluid temperature sensor A circuit high",
    ),
    ("P0715", "Input/turbine speed sensor A circuit"),
    ("P0716", "Input/turbine speed sensor A range/performance"),
    ("P0717", "Input/turbine speed sensor A: no signal"),
    ("P0720", "Output speed sensor circuit"),
    ("P0721", "Output speed sensor range/performance"),
    ("P0722", "Output speed sensor: no signal"),
    ("P0730", "Incorrect gear ratio"),
    ("P0731", "Gear 1 incorrect ratio"),
    ("P0732", "Gear 2 incorrect ratio"),
    ("P0733", "Gear 3 incorrect ratio"),
    ("P0734", "Gear 4 incorrect ratio"),
    ("P0735", "Gear 5 incorrect ratio"),
    ("P0736", "Reverse incorrect ratio"),
    ("P0740", "Torque converter clutch solenoid circuit"),
    ("P0741", "Torque converter clutch: performance or stuck off"),
    ("P0742", "Torque converter clutch: stuck on"),
    ("P0750", "Shift solenoid A"),
    ("P0751", "Shift solenoid A: performance or stuck off"),
    ("P0752", "Shift solenoid A: stuck on"),
    ("P0755", "Shift solenoid B"),
    ("P0756", "Shift solenoid B: performance or stuck off"),
    ("P0757", "Shift solenoid B: stuck on"),
    ("P0760", "Shift solenoid C"),
    ("P0765", "Shift solenoid D"),
    ("P0770", "Shift solenoid E"),
    ("P2096", "Post-catalyst fuel trim too lean (bank 1)"),
    ("P2097", "Post-catalyst fuel trim too rich (bank 1)"),
    ("P2098", "Post-catalyst fuel trim too lean (bank 2)"),
    ("P2099", "Post-catalyst fuel trim too rich (bank 2)"),
    (
        "P2135",
        "Throttle/pedal position sensors A and B do not agree",
    ),
    (
        "P2138",
        "Throttle/pedal position sensors D and E do not agree",
    ),
    ("P2195", "Oxygen sensor signal stuck lean (bank 1 sensor 1)"),
    ("P2196", "Oxygen sensor signal stuck rich (bank 1 sensor 1)"),
    ("P2197", "Oxygen sensor signal stuck lean (bank 2 sensor 1)"),
    ("P2198", "Oxygen sensor signal stuck rich (bank 2 sensor 1)"),
    ("P2270", "Oxygen sensor signal stuck lean (bank 1 sensor 2)"),
    ("P2271", "Oxygen sensor signal stuck rich (bank 1 sensor 2)"),
    ("U0001", "High-speed CAN communication bus"),
    ("U0073", "Control module communication bus off"),
    (
        "U0100",
        "Lost communication with the engine module (ECM/PCM)",
    ),
    (
        "U0101",
        "Lost communication with the transmission module (TCM)",
    ),
    ("U0121", "Lost communication with the ABS module"),
    ("U0140", "Lost communication with the body control module"),
    (
        "U0151",
        "Lost communication with the airbag (restraints) module",
    ),
    ("U0155", "Lost communication with the instrument cluster"),
    (
        "U0401",
        "Invalid data received from the engine module (ECM/PCM)",
    ),
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn standard_codes_and_families() {
        let t = |c: &str| explain(c, None).unwrap().text;
        assert_eq!(t("P0171").as_deref(), Some("System too lean (bank 1)"));
        assert_eq!(t("p0304").as_deref(), Some("Misfire detected, cylinder 4"));
        assert_eq!(
            t("P0138").as_deref(),
            Some("Oxygen sensor circuit high voltage (bank 1 sensor 2)")
        );
        assert_eq!(
            t("P0161").as_deref(),
            Some("Oxygen sensor heater circuit (bank 2 sensor 2)")
        );
        assert_eq!(t("P0206").as_deref(), Some("Injector circuit, cylinder 6"));
        assert_eq!(
            t("P0266").as_deref(),
            Some("Injector contribution/balance, cylinder 2")
        );
        assert_eq!(
            t("P0294").as_deref(),
            Some("Injector circuit low, cylinder 12")
        );
        assert_eq!(
            t("P0353").as_deref(),
            Some("Ignition coil C primary/secondary circuit (usually cylinder 3)")
        );
        assert_eq!(
            t("P0999"),
            None,
            "a standard code Icarus has no words for yet"
        );
        let mut seen = std::collections::HashSet::new();
        for (c, _) in GENERIC {
            assert!(seen.insert(*c), "{c} twice");
            assert!(
                explain(c, None).unwrap().standard,
                "{c} is in a standard range"
            );
            assert!(
                standard_text_family_free(c),
                "{c} is also made by a pattern"
            );
        }
    }

    fn standard_text_family_free(c: &str) -> bool {
        // GENERIC must not shadow a numbered family
        let n: Option<u16> = c[2..].parse().ok();
        c.starts_with('U')
            || c.starts_with("P2")
            || !n.is_some_and(|n| {
                SENSORS.iter().any(|(b, _)| (*b..b + 6).contains(&n))
                    || (201..=212).contains(&n)
                    || (261..=296).contains(&n)
                    || (301..=312).contains(&n)
                    || (351..=362).contains(&n)
            })
    }

    #[test]
    fn anatomy_and_maker_codes() {
        let i = explain("P0420", None).unwrap();
        assert!(i.standard && i.hint.is_some());
        assert_eq!(
            i.area,
            Some("auxiliary emission controls (EGR, secondary air, catalyst, evaporative system)")
        );
        assert!(anatomy(&i).starts_with("P: powertrain"));
        let m = explain("P1349", None).unwrap();
        assert!(!m.standard && m.text.is_none());
        assert!(!explain("C1241", None).unwrap().standard);
        assert!(explain("U0100", None).unwrap().standard);
        assert!(
            !explain("P3000", None).unwrap().standard && explain("P3400", None).unwrap().standard
        );
        for bad in ["P017", "X0171", "P01711", "P4000", "P01G1"] {
            assert!(explain(bad, None).is_err(), "{bad}");
        }
        let all = crate::vehicles::builtin();
        let toyota = crate::vehicles::find(&all, "toyota").unwrap();
        assert_eq!(parts("P0420")[0], "catalytic converter");
        assert_eq!(parts("P0138"), &["oxygen sensor"]);
        assert_eq!(parts("p0304")[1], "ignition coil");
        assert!(
            parts("C1241").is_empty() && parts("P1349").is_empty() && parts("P2A00").is_empty()
        );
        let c = explain("C1241", Some(toyota)).unwrap();
        assert!(c.text.unwrap().contains("battery"));
        assert_eq!(c.source, toyota.name);
    }
}
