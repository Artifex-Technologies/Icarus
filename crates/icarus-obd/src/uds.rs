//! Talking to each module the way dealer tools (Techstream, FORScan) do: not only the engine's standard OBD-II answers, but
//! every module on the bus (ABS, airbags, instrument cluster, body...) with the diagnostic services of ISO 14229 (UDS) and the
//! older ISO 14230 (KWP2000) that many Toyota modules still use on CAN.
//!
//! **Read-only.** Every request passes `check_read_only`, which allows only services that read (identification, data,
//! trouble codes, tester present). Clearing codes, writing data, routines, security access and programming are refused here
//! until Icarus has the safety steps of docs/PLAN.md section 4.

use crate::link::Link;
use crate::elm::{Answer, ObdError};
use crate::j1979::dtc;
use std::thread::sleep;
use std::time::Duration;

/// Services Icarus may send: they only read.
const READ_ONLY: &[(u8, &str)] = &[
    (0x01, "OBD current data"),
    (0x03, "OBD stored codes"),
    (0x07, "OBD pending codes"),
    (0x09, "OBD vehicle information"),
    (0x0A, "OBD permanent codes"),
    (0x18, "KWP2000 read trouble codes by status"),
    (0x19, "UDS read trouble code information"),
    (0x1A, "KWP2000 read ECU identification"),
    (0x21, "KWP2000 read data by local identifier"),
    (0x22, "UDS read data by identifier"),
    (0x3E, "tester present"),
];

/// Refuses any service that is not on the read-only list (the reason names what it would do).
pub fn check_read_only(data: &[u8]) -> Result<(), ObdError> {
    let Some(&sid) = data.first() else {
        return Err(ObdError::Adapter("an empty request".into()));
    };
    if READ_ONLY.iter().any(|(s, _)| *s == sid) {
        return Ok(());
    }
    let what = match sid {
        0x04 | 0x14 => "clear trouble codes",
        0x10 => "change the diagnostic session",
        0x11 => "reset the module",
        0x27 => "unlock security access",
        0x28 => "switch communication off",
        0x2E | 0x3B => "write data",
        0x2F | 0x30 => "drive outputs (actuator tests)",
        0x31 => "run a routine",
        0x34 | 0x35 | 0x36 | 0x37 => "transfer software (programming)",
        0x3D => "write memory",
        0x85 => "control trouble code setting",
        _ => "something Icarus does not know is safe",
    };
    Err(ObdError::Adapter(format!(
        "Icarus is read-only for now: service 0x{sid:02X} would {what}"
    )))
}

/// What a negative response code (`7F <service> <code>`) means.
pub fn nrc(code: u8) -> &'static str {
    match code {
        0x10 => "general reject",
        0x11 => "service not supported",
        0x12 => "sub-function not supported",
        0x13 => "wrong message length or format",
        0x14 => "response too long",
        0x21 => "busy, repeat the request",
        0x22 => "conditions not correct (engine running? speed? session?)",
        0x24 => "request sequence error",
        0x31 => "request out of range (this identifier does not exist here)",
        0x33 => "security access denied",
        0x35 => "invalid key",
        0x78 => "response pending",
        0x7E => "sub-function not supported in this session",
        0x7F => "service not supported in this session",
        _ => "another refusal",
    }
}

/// One module's answer to a request, pending responses (7F xx 78) waited out.
fn ask(elm: &mut dyn Link, tx: u16, rx: u16, data: &[u8]) -> Result<Option<Answer>, ObdError> {
    check_read_only(data)?;
    for attempt in 0..4 {
        let answers = match elm.request_to(tx, rx, data) {
            Ok(a) => a,
            Err(ObdError::NoData) => return Ok(None),
            Err(e) => return Err(e),
        };
        let Some(a) = answers.into_iter().last() else {
            return Ok(None);
        };
        match a.refusal() {
            Some((_, 0x78)) | Some((_, 0x21)) if attempt < 3 => sleep(Duration::from_millis(300)),
            _ => return Ok(Some(a)),
        }
    }
    Ok(None)
}

/// A module trouble code with its status, as UDS and KWP2000 report them.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ModuleDtc {
    /// "P0171", "C1241"...
    pub code: String,
    /// UDS's failure type byte (what kind of fault: "-1C" circuit voltage out of range...), when given.
    pub failure_type: Option<u8>,
    pub status: u8,
    /// Status bits in words.
    pub states: Vec<&'static str>,
}

fn uds_states(s: u8) -> Vec<&'static str> {
    let names = [
        "failing now",
        "failed this cycle",
        "pending",
        "confirmed",
        "not tested since codes were cleared",
        "failed since codes were cleared",
        "not tested this cycle",
        "warning light requested",
    ];
    (0..8)
        .filter(|b| s & (1 << b) != 0)
        .map(|b| names[b])
        .collect()
}

/// UDS `19 02` answer: `59 02 <availability> (<3 bytes DTC> <status>)*`.
pub fn parse_uds_dtcs(d: &[u8]) -> Option<Vec<ModuleDtc>> {
    if d.len() < 3 || d[0] != 0x59 || d[1] != 0x02 {
        return None;
    }
    Some(
        d[3..]
            .chunks_exact(4)
            .filter(|c| c[..3] != [0, 0, 0])
            .map(|c| ModuleDtc {
                code: dtc(c[0], c[1]),
                failure_type: Some(c[2]),
                status: c[3],
                states: uds_states(c[3]),
            })
            .collect(),
    )
}

/// KWP2000 `18` answer: `58 <count> (<2 bytes DTC> <status>)*`.
pub fn parse_kwp_dtcs(d: &[u8]) -> Option<Vec<ModuleDtc>> {
    if d.len() < 2 || d[0] != 0x58 {
        return None;
    }
    Some(
        d[2..]
            .chunks_exact(3)
            .filter(|c| c[..2] != [0, 0])
            .map(|c| {
                let mut states = Vec::new();
                if c[2] & 0x20 != 0 {
                    states.push("stored");
                }
                if c[2] & 0x40 != 0 {
                    states.push("present now");
                }
                if c[2] & 0x80 != 0 {
                    states.push("warning light requested");
                }
                ModuleDtc {
                    code: dtc(c[0], c[1]),
                    failure_type: None,
                    status: c[2],
                    states,
                }
            })
            .collect(),
    )
}

/// Which diagnostic language a module answered in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dialect {
    Uds,
    Kwp,
}

/// A module's trouble codes: UDS first, KWP2000 when the module does not know UDS's service.
pub fn module_dtcs(
    elm: &mut dyn Link,
    tx: u16,
    rx: u16,
) -> Result<Option<(Dialect, Vec<ModuleDtc>)>, ObdError> {
    if let Some(a) = ask(elm, tx, rx, &[0x19, 0x02, 0xFF])? {
        if let Some(v) = parse_uds_dtcs(&a.data) {
            return Ok(Some((Dialect::Uds, v)));
        }
    }
    if let Some(a) = ask(elm, tx, rx, &[0x18, 0x02, 0xFF, 0x00])? {
        if let Some(v) = parse_kwp_dtcs(&a.data) {
            return Ok(Some((Dialect::Kwp, v)));
        }
    }
    Ok(None)
}

/// The standard UDS identification values (ISO 14229 data identifiers F187-F197).
pub const IDENTITY: &[(u16, &str)] = &[
    (0xF187, "part number"),
    (0xF188, "software number"),
    (0xF189, "software version"),
    (0xF18A, "supplier"),
    (0xF191, "hardware number"),
    (0xF197, "system name"),
    (0xF190, "VIN"),
];

/// Text when the value is printable, hex otherwise.
fn shown(b: &[u8]) -> String {
    let t: String = b
        .iter()
        .filter(|c| **c != 0 && **c != 0xFF)
        .map(|&c| c as char)
        .collect();
    if !t.is_empty() && t.chars().all(|c| c.is_ascii_graphic() || c == ' ') {
        t.trim().to_string()
    } else {
        b.iter()
            .map(|x| format!("{x:02X}"))
            .collect::<Vec<_>>()
            .join(" ")
    }
}

/// The identification values a module gives (each asked separately: many modules refuse several at once).
pub fn module_identity(
    elm: &mut dyn Link,
    tx: u16,
    rx: u16,
) -> Result<Vec<(&'static str, String)>, ObdError> {
    let mut out = Vec::new();
    for (did, name) in IDENTITY {
        if let Some(a) = ask(elm, tx, rx, &[0x22, (did >> 8) as u8, *did as u8])? {
            if a.data.len() > 3
                && a.data[0] == 0x62
                && a.data[1..3] == [(did >> 8) as u8, *did as u8]
            {
                out.push((*name, shown(&a.data[3..])));
            }
        }
    }
    Ok(out)
}

/// Whether a module answers at request ID `tx`: any answer counts, a refusal too.
pub fn present(elm: &mut dyn Link, tx: u16, rx: u16) -> Result<bool, ObdError> {
    for req in [&[0x3E, 0x00][..], &[0x3E, 0x01][..]] {
        if ask(elm, tx, rx, req)?.is_some() {
            return Ok(true);
        }
    }
    Ok(false)
}

/// Every request ID from 700 to 7F7 except the OBD broadcast (7DF): a slower search that finds modules anywhere.
pub fn sweep() -> Vec<crate::vehicles::ModuleAddr> {
    (0x700..=0x7F7)
        .filter(|id| *id != 0x7DF)
        .map(|tx| crate::vehicles::ModuleAddr {
            tx,
            rx: None,
            name: "found by sweep".into(),
            ext: None,
            bus: None,
            checked: false,
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elm::Elm;
    use crate::sim::SimCar;

    #[test]
    fn only_reading_services_go_out() {
        for ok in [
            &[0x22, 0xF1, 0x90][..],
            &[0x19, 0x02, 0xFF],
            &[0x3E, 0x00],
            &[0x01, 0x0C],
        ] {
            assert!(check_read_only(ok).is_ok(), "{ok:02X?}");
        }
        for (bad, why) in [
            (&[0x14, 0xFF, 0xFF, 0xFF][..], "clear"),
            (&[0x2E, 0xF1, 0x90], "write"),
            (&[0x27, 0x01], "security"),
            (&[0x34], "programming"),
            (&[0x31, 0x01], "routine"),
            (&[0x10, 0x03], "session"),
            (&[0x04], "clear"),
        ] {
            let e = check_read_only(bad).unwrap_err().to_string();
            assert!(e.contains("read-only") && e.contains(why), "{e}");
        }
        assert!(check_read_only(&[]).is_err());
    }

    #[test]
    fn trouble_code_answers() {
        let uds = [0x59, 0x02, 0xFF, 0x01, 0x71, 0x00, 0x09, 0, 0, 0, 0];
        let v = parse_uds_dtcs(&uds).unwrap();
        assert_eq!(v.len(), 1);
        assert_eq!(
            (v[0].code.as_str(), v[0].failure_type, v[0].states.clone()),
            ("P0171", Some(0), vec!["failing now", "confirmed"])
        );
        let kwp = [0x58, 0x01, 0x52, 0x41, 0x60];
        let k = parse_kwp_dtcs(&kwp).unwrap();
        assert_eq!(
            (k[0].code.as_str(), k[0].states.clone()),
            ("C1241", vec!["stored", "present now"])
        );
        assert!(parse_uds_dtcs(&[0x7F, 0x19, 0x11]).is_none());
        assert_eq!(
            nrc(0x31),
            "request out of range (this identifier does not exist here)"
        );
    }

    #[test]
    fn scanning_the_simulated_toyota() {
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        let all = crate::vehicles::builtin();
        let vin = crate::j1979::vin(&mut elm).unwrap().unwrap();
        let toyota = crate::vehicles::for_vin(&all, &vin).unwrap();
        assert_eq!(toyota.id, "toyota", "the VIN picks the profile");
        let found: Vec<u16> = crate::vehicles::candidates(toyota)
            .into_iter()
            .filter(|m| m.unreachable().is_none())
            .filter(|m| present(&mut elm, m.tx, m.rx()).unwrap())
            .map(|m| m.tx)
            .collect();
        assert_eq!(found, vec![0x7E0, 0x7E1, 0x7B0, 0x780, 0x7C0]);
        let id = module_identity(&mut elm, 0x7E0, 0x7E0 + 8).unwrap();
        assert!(
            id.contains(&("part number", "89661-ICARUS".to_string())),
            "{id:?}"
        );
        let (d, codes) = module_dtcs(&mut elm, 0x7E0, 0x7E0 + 8).unwrap().unwrap();
        assert_eq!((d, codes[0].code.as_str()), (Dialect::Uds, "P0171"));
        let (d, codes) = module_dtcs(&mut elm, 0x7B0, 0x7B0 + 8).unwrap().unwrap();
        assert_eq!(
            (d, codes[0].code.as_str()),
            (Dialect::Kwp, "C1241"),
            "the ABS speaks KWP2000 only"
        );
        let (_, codes) = module_dtcs(&mut elm, 0x780, 0x780 + 8).unwrap().unwrap();
        assert!(codes.is_empty());
        let (d, codes) = module_dtcs(&mut elm, 0x7C0, 0x7C0 + 8).unwrap().unwrap();
        assert_eq!(
            (d, codes.len()),
            (Dialect::Uds, 0),
            "a 'response pending' is waited out, not taken as the answer"
        );
        // after module requests, an OBD broadcast reaches every module again
        assert_eq!(crate::j1979::read(&mut elm, 0x0C).unwrap().len(), 1);
        assert!(elm.request(&[0x01, 0x00]).unwrap().len() >= 2);
    }
}
