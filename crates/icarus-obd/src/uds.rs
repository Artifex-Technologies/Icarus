//! Talking to each module the way dealer tools (Techstream, FORScan) do: not only the engine's standard OBD-II answers, but
//! every module on the bus (ABS, airbags, instrument cluster, body...) with the diagnostic services of ISO 14229 (UDS) and the
//! older ISO 14230 (KWP2000) that many Toyota modules still use on CAN.
//!
//! **Read-only.** Every request passes `check_read_only`, which allows only services that read (identification, data,
//! trouble codes, tester present). Clearing codes, writing data, routines, security access and programming are refused here
//! until Icarus has the safety steps of docs/DESIGN.md ("Writing a module (flash): the conditions").

use crate::elm::{Answer, ObdError};
use crate::j1979::dtc;
use crate::link::Link;
use std::thread::sleep;
use std::time::Duration;

/// Services Icarus may send: they only read.
const READ_ONLY: &[(u8, &str)] = &[
    (0x01, "OBD current data"),
    (0x03, "OBD stored codes"),
    (0x07, "OBD pending codes"),
    (0x09, "OBD vehicle information"),
    (0x0A, "OBD permanent codes"),
    (0x13, "KWP2000 read trouble codes"),
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
        0x34..=0x37 => "transfer software (programming)",
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

/// The wait before a request is sent again after "response pending" (7F xx 78) or "busy" (7F xx 21).
pub const PENDING_WAIT: Duration = Duration::from_millis(300);
/// Sends of one request window at most: the first and three more after a pending or busy answer.
pub const WINDOW_SENDS: usize = 4;
/// The wait before `read_data` asks a second window, after a first that held only a late answer to another identifier: at
/// least module data's least gap between requests (`moddata::MIN_GAP`, 50 ms), so the resend is paced like any request.
pub const RESEND_WAIT: Duration = Duration::from_millis(50);

/// Every answer in one request's window, pending responses (7F xx 78) and "busy" (7F xx 21) waited out: when the window
/// ends with one of them, the request is sent again after 300 ms (a read is harmless to repeat), up to three times.
fn ask_window(elm: &mut dyn Link, tx: u32, rx: u32, data: &[u8]) -> Result<Vec<Answer>, ObdError> {
    ask_window_with(elm, tx, rx, data, &mut sleep)
}

/// `ask_window`, waiting with `nap` (module data passes its pacer's clock, so the waits are the pacer's and tests check
/// them without sleeping).
fn ask_window_with(
    elm: &mut dyn Link,
    tx: u32,
    rx: u32,
    data: &[u8],
    nap: &mut dyn FnMut(Duration),
) -> Result<Vec<Answer>, ObdError> {
    check_read_only(data)?;
    for attempt in 0..WINDOW_SENDS {
        let answers = match elm.request_to(tx, rx, data) {
            Ok(a) => a,
            Err(ObdError::NoData) => return Ok(vec![]),
            Err(e) => return Err(e),
        };
        match answers.last().and_then(|a| a.refusal()) {
            Some((_, 0x78)) | Some((_, 0x21)) if attempt + 1 < WINDOW_SENDS => nap(PENDING_WAIT),
            _ => return Ok(answers),
        }
    }
    Ok(vec![])
}

/// One module's answer to a request, pending responses (7F xx 78) waited out.
fn ask(elm: &mut dyn Link, tx: u32, rx: u32, data: &[u8]) -> Result<Option<Answer>, ObdError> {
    Ok(ask_window(elm, tx, rx, data)?.into_iter().last())
}

/// A module's answer to a data request (`read_data`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DataAnswer {
    /// The bytes after the service byte and the identifier's echo.
    Data(Vec<u8>),
    /// `7F <service> <code>`.
    Refused(u8),
    /// No answer, or none to this identifier.
    Silent,
}

/// Reads one data identifier: `22 hi lo` (UDS), `21 id` (KWP2000 local identifier) or `1A id` (KWP2000 identification),
/// nothing else. The answer must be the positive one with the identifier echoed (`62 hi lo ...`); a positive answer to
/// another identifier is a late answer to an earlier request: it is dropped and the request sent once more, and if that
/// does not match either, the module counts as silent.
///
/// The most it sends for one call is two windows of `WINDOW_SENDS`, 8 sends, and no two are closer than 50 ms: 300 ms
/// (`PENDING_WAIT`) within a window, `RESEND_WAIT` before the second.
pub fn read_data(
    elm: &mut dyn Link,
    tx: u32,
    rx: u32,
    service: u8,
    ident: u16,
) -> Result<DataAnswer, ObdError> {
    read_data_with(elm, tx, rx, service, ident, &mut sleep)
}

/// `read_data`, waiting with `nap` (the pacer's clock).
pub fn read_data_with(
    elm: &mut dyn Link,
    tx: u32,
    rx: u32,
    service: u8,
    ident: u16,
    nap: &mut dyn FnMut(Duration),
) -> Result<DataAnswer, ObdError> {
    let req = match service {
        0x22 => vec![0x22, (ident >> 8) as u8, ident as u8],
        0x21 | 0x1A if ident <= 0xFF => vec![service, ident as u8],
        _ => {
            return Err(ObdError::Adapter(format!(
                "a data read is 21, 22 or 1A with its identifier, not {service:02X} {ident:X}"
            )))
        }
    };
    for second in [false, true] {
        if second {
            nap(RESEND_WAIT);
        }
        let window = ask_window_with(elm, tx, rx, &req, nap)?;
        if let Some(a) = window.iter().find(|a| {
            a.data.first() == Some(&(service + 0x40)) && a.data.get(1..req.len()) == Some(&req[1..])
        }) {
            return Ok(DataAnswer::Data(a.data[req.len()..].to_vec()));
        }
        if let Some((_, code)) = window
            .iter()
            .rev()
            .filter_map(|a| a.refusal())
            .find(|(s, _)| *s == service)
        {
            return Ok(DataAnswer::Refused(code));
        }
        if !window
            .iter()
            .any(|a| a.data.first() == Some(&(service + 0x40)))
        {
            return Ok(DataAnswer::Silent);
        }
    }
    Ok(DataAnswer::Silent)
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
            .as_chunks::<4>()
            .0
            .iter()
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

/// ISO 14230 `13` answer: `53 <count>` and the codes, two bytes each, or three with a status byte (both are in use).
pub fn parse_kwp13_dtcs(d: &[u8]) -> Option<Vec<ModuleDtc>> {
    if d.len() < 2 || d[0] != 0x53 {
        return None;
    }
    let (n, rest) = (d[1] as usize, &d[2..]);
    let size = match n {
        0 => return Some(Vec::new()),
        _ if rest.len() == n * 2 => 2,
        _ if rest.len() == n * 3 => 3,
        _ => return None,
    };
    Some(
        rest.chunks(size)
            .filter(|c| c[..2] != [0, 0])
            .map(|c| {
                let status = c.get(2).copied();
                ModuleDtc {
                    code: dtc(c[0], c[1]),
                    failure_type: None,
                    status: status.unwrap_or(0),
                    states: status.map(kwp_states).unwrap_or_else(|| vec!["stored"]),
                }
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
            .as_chunks::<3>()
            .0
            .iter()
            .filter(|c| c[..2] != [0, 0])
            .map(|c| ModuleDtc {
                code: dtc(c[0], c[1]),
                failure_type: None,
                status: c[2],
                states: kwp_states(c[2]),
            })
            .collect(),
    )
}

/// A KWP2000 status byte in words.
fn kwp_states(s: u8) -> Vec<&'static str> {
    let mut states = Vec::new();
    if s & 0x20 != 0 {
        states.push("stored");
    }
    if s & 0x40 != 0 {
        states.push("present now");
    }
    if s & 0x80 != 0 {
        states.push("warning light requested");
    }
    states
}

/// Which diagnostic language a module answered in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dialect {
    Uds,
    Kwp,
}

/// A module's codes as read: the dialect and the codes, or, when no request gave them, what each request got
/// ("19 02 FF: refused, service not supported; 13: no answer").
pub type CodesRead = Result<(Dialect, Vec<ModuleDtc>), String>;

/// The read-only ways of asking a module for its codes, in order: UDS `19 02` with the status mask FF, then 2F, 09 and 08
/// (a module may refuse a mask it does not support with "request out of range"); KWP2000 `18` (status 02, then 00, every
/// group), then ISO 14230's `13` with and without the group (several makers' CAN modules, Toyota's among them, answer
/// only one of these). Each one is tried only when the one before was refused or not understood.
const CODE_REQUESTS: &[(Dialect, &[u8])] = &[
    (Dialect::Uds, &[0x19, 0x02, 0xFF]),
    (Dialect::Uds, &[0x19, 0x02, 0x2F]),
    (Dialect::Uds, &[0x19, 0x02, 0x09]),
    (Dialect::Uds, &[0x19, 0x02, 0x08]),
    (Dialect::Kwp, &[0x18, 0x02, 0xFF, 0x00]),
    (Dialect::Kwp, &[0x18, 0x00, 0xFF, 0x00]),
    (Dialect::Kwp, &[0x13, 0xFF, 0x00]),
    (Dialect::Kwp, &[0x13]),
];

fn hex_req(r: &[u8]) -> String {
    r.iter()
        .map(|b| format!("{b:02X}"))
        .collect::<Vec<_>>()
        .join(" ")
}

/// A module's trouble codes, by the first request in `CODE_REQUESTS` that gives them. Once the module refuses UDS's
/// service outright ("service not supported", 11), the other UDS masks are skipped.
pub fn module_dtcs(elm: &mut dyn Link, tx: u32, rx: u32) -> Result<CodesRead, ObdError> {
    let mut tried: Vec<String> = Vec::new();
    let mut skip_uds = false;
    for (dialect, req) in CODE_REQUESTS {
        if skip_uds && *dialect == Dialect::Uds {
            continue;
        }
        let got = ask(elm, tx, rx, req)?;
        let parsed = got.as_ref().and_then(|a| match req[0] {
            0x19 => parse_uds_dtcs(&a.data),
            0x18 => parse_kwp_dtcs(&a.data),
            _ => parse_kwp13_dtcs(&a.data),
        });
        if let Some(v) = parsed {
            return Ok(Ok((*dialect, v)));
        }
        let what = match got.as_ref().and_then(|a| a.refusal()) {
            Some((_, code)) => {
                if req[0] == 0x19 && code == 0x11 {
                    skip_uds = true;
                }
                format!("refused, {}", nrc(code))
            }
            None if got.is_some() => "an answer Icarus could not read".to_string(),
            None => "no answer".to_string(),
        };
        tried.push(format!("{}: {what}", hex_req(req)));
    }
    Ok(Err(tried.join("; ")))
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
    tx: u32,
    rx: u32,
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

/// Runs `f` with the requests to one module: behind a gateway (`ext`, e.g. Toyota's body modules at 750), each carries the
/// module's address byte; afterwards normal addressing again, whatever `f` returned. Every module loop goes through this.
pub fn at<T>(
    elm: &mut dyn Link,
    ext: Option<u8>,
    f: impl FnOnce(&mut dyn Link) -> Result<T, ObdError>,
) -> Result<T, ObdError> {
    elm.set_ext(ext)?;
    let r = f(elm);
    let back = elm.set_ext(None);
    let v = r?;
    back?;
    Ok(v)
}

/// Whether a module answers at request ID `tx`: any answer counts, a refusal too.
pub fn present(elm: &mut dyn Link, tx: u32, rx: u32) -> Result<bool, ObdError> {
    for req in [&[0x3E, 0x00][..], &[0x3E, 0x01][..]] {
        if ask(elm, tx, rx, req)?.is_some() {
            return Ok(true);
        }
    }
    Ok(false)
}

/// IDs the sweep never sends on, from the loaded profiles: OBD's broadcast (7DF, and 18DB33F1 on 29-bit CAN), the OBD-II
/// answer IDs (7E8-7EF, and 18DAF1xx: every 29-bit answer to the tester), and the answer ID of every module any profile
/// names (its `rx`, or its request ID + 8), its ranges' too.
///
/// A request sent on one of these would look, to every module on the bus, like that module's own answer: a gateway or the
/// tester side of another module can act on it, and the module whose ID it is sees a frame it did not send (on some
/// modules, a bus fault). 7DF would reach every module at once. So the sweep sends only on IDs that could be requests.
pub fn sweep_skips(profiles: &[crate::vehicles::Profile]) -> std::collections::BTreeSet<u32> {
    let mut skip: std::collections::BTreeSet<u32> = (0x7E8..=0x7EF).collect();
    skip.insert(0x7DF);
    skip.insert(0x18DB_33F1);
    skip.extend((0..=0xFF).map(|a| 0x18DA_F100 | a));
    skip.extend(
        profiles
            .iter()
            .flat_map(|p| crate::vehicles::named(p).into_iter().map(|m| m.rx())),
    );
    skip
}

/// One scan's memory of the modules it has found, so that it never sends on their answer IDs.
///
/// `sweep_skips` knows only the answer IDs that profiles name, but the sweep is there to find modules no profile names. When
/// one answers at, say, 740 (answering from 748), a later request on 748 would look like that module's own answer, the
/// hazard `sweep_skips` describes. Candidates go up in order and an answer ID is usually its request ID + 8, so without this
/// the sweep would reach it. Both scans (the app's and the command line's) ask through [`ScanGuard::present`].
#[derive(Debug, Default)]
pub struct ScanGuard {
    /// Answer ID -> the request ID of the module found answering from it.
    answers: std::collections::BTreeMap<u32, u32>,
}

impl ScanGuard {
    /// The module found earlier in this scan whose answer ID is `tx`, when there is one: the scan must not send on `tx`.
    pub fn answer_of(&self, tx: u32) -> Option<u32> {
        self.answers.get(&tx).copied()
    }

    /// [`present`] for one step of a scan: false, with nothing sent, when `m`'s request ID is the answer ID of a module found
    /// earlier in this scan; otherwise it asks, and remembers `m`'s answer ID when `m` answers.
    pub fn present(
        &mut self,
        elm: &mut dyn Link,
        m: &crate::vehicles::ModuleAddr,
    ) -> Result<bool, ObdError> {
        if self.answer_of(m.tx).is_some() {
            return Ok(false);
        }
        let there = at(elm, m.ext, |l| present(l, m.tx, m.rx()))?;
        if there {
            self.answers.insert(m.rx(), m.tx);
        }
        Ok(there)
    }
}

/// Every candidate request ID from 700 to 7F7 (a slower search that finds modules anywhere), leaving out the IDs of
/// `sweep_skips`: the broadcast and every known answer ID, which the sweep must not transmit on. The answer IDs of modules
/// the sweep itself finds are left out as it goes, by [`ScanGuard`].
pub fn sweep(profiles: &[crate::vehicles::Profile]) -> Vec<crate::vehicles::ModuleAddr> {
    swept(profiles, 0x700..=0x7F7)
}

/// `sweep` for a vehicle whose OBD-II uses `protocol`: on 29-bit CAN, every 29-bit physical request ID 18DAxxF1 instead
/// (xx 00-FF but F1, the tester's own address; their answers are 18DAF1xx, which no request uses).
pub fn sweep_on(
    profiles: &[crate::vehicles::Profile],
    protocol: crate::elm::Protocol,
) -> Vec<crate::vehicles::ModuleAddr> {
    if !crate::link::Reach::of(protocol).can29 {
        return sweep(profiles);
    }
    swept(
        profiles,
        (0..=0xFFu32)
            .map(|a| crate::vehicles::FIXED29 | a << 8 | 0xF1)
            .filter(|&id| crate::vehicles::is_fixed29_request(id)),
    )
}

fn swept(
    profiles: &[crate::vehicles::Profile],
    ids: impl IntoIterator<Item = u32>,
) -> Vec<crate::vehicles::ModuleAddr> {
    let skip = sweep_skips(profiles);
    ids.into_iter()
        .filter(|id| !skip.contains(id))
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
    fn the_sweep_never_sends_on_the_broadcast_or_an_answer_id() {
        use crate::sim::recording::Recorder;
        use std::collections::BTreeSet;
        let mut all = crate::vehicles::builtin();
        all.push(
            crate::vehicles::parse(
                "mine",
                "a user file",
                "name = \"mine\"\n[[module]]\ntx = 0x7A0\nrx = 0x7A3\nname = \"odd answer ID\"\n",
            )
            .unwrap(),
        );
        let skip = sweep_skips(&all);
        for id in [0x7DF, 0x7E8, 0x7E9, 0x7EF, 0x7B8, 0x758, 0x77D, 0x7A3] {
            assert!(skip.contains(&id), "{id:03X}");
        }
        let list = sweep(&all);
        assert!(list.iter().all(|m| !skip.contains(&m.tx)));
        assert!(list.iter().any(|m| m.tx == 0x7E0) && list.iter().any(|m| m.tx == 0x7B0));
        // on the wire: every request goes out under the header (ATSH) set last, which must never be a skipped ID
        let (port, sent) = Recorder::new(SimCar::new());
        let mut elm = Elm::connect(Box::new(port)).unwrap();
        sent.lock().unwrap().clear();
        let mut found = Vec::new();
        for m in &list {
            if present(&mut elm, m.tx, m.rx()).unwrap() {
                found.push(m.tx);
            }
        }
        assert_eq!(
            found,
            vec![0x700, 0x701, 0x780, 0x7B0, 0x7C0, 0x7C4, 0x7E0, 0x7E1]
        );
        let sent_on = sent_on(&sent.lock().unwrap());
        assert!(
            sent_on.is_disjoint(&skip),
            "{:03X?}",
            sent_on.intersection(&skip).collect::<Vec<_>>()
        );
        assert_eq!(sent_on, list.iter().map(|m| m.tx).collect::<BTreeSet<_>>());
    }

    /// The IDs requests went out on, from a log of what was sent to the adapter: every request goes out under the header
    /// (ATSH) set last, 7DF until one is set; a six-digit ATSH is a 29-bit header's lower bytes under the priority byte
    /// set last (ATCP, 18 until set).
    pub(crate) fn sent_on(log: &[String]) -> std::collections::BTreeSet<u32> {
        let (mut header, mut on) = (0x7DF, std::collections::BTreeSet::new());
        let mut priority = 0x18;
        for l in log {
            let c = l.trim_end_matches('\r');
            if let Some(p) = c.strip_prefix("ATCP") {
                priority = u32::from_str_radix(p, 16).unwrap();
            } else if let Some(h) = c.strip_prefix("ATSH") {
                header = u32::from_str_radix(h, 16).unwrap();
                if h.len() == 6 {
                    header |= priority << 24;
                }
            } else if !c.is_empty() && c.chars().all(|ch| ch.is_ascii_hexdigit()) {
                on.insert(header);
            }
        }
        on
    }

    #[test]
    fn the_sweep_never_sends_on_the_answer_id_of_a_module_it_found() {
        use crate::sim::recording::Recorder;
        // with no profile loaded, only 7DF and 7E8-7EF are known answer IDs: 708, 709, 788, 7B8 and 7C8 (where the simulated
        // 700, 701, 780, 7B0 and 7C0 answer from) are in the list, and only what the sweep finds can keep it off them
        let list = sweep(&[]);
        for id in [0x708, 0x709, 0x788, 0x7B8, 0x7C8] {
            assert!(list.iter().any(|m| m.tx == id), "{id:03X}");
        }
        let (port, sent) = Recorder::new(SimCar::new());
        let mut elm = Elm::connect(Box::new(port)).unwrap();
        sent.lock().unwrap().clear();
        let mut guard = ScanGuard::default();
        let mut found = Vec::new();
        for m in &list {
            if guard.present(&mut elm, m).unwrap() {
                found.push(m.tx);
            }
        }
        assert_eq!(
            found,
            vec![0x700, 0x701, 0x780, 0x7B0, 0x7C0, 0x7C4, 0x7E0, 0x7E1]
        );
        for (rx, tx) in [
            (0x708, 0x700),
            (0x709, 0x701),
            (0x788, 0x780),
            (0x7B8, 0x7B0),
            (0x7C8, 0x7C0),
            (0x7E9, 0x7E1),
        ] {
            assert_eq!(guard.answer_of(rx), Some(tx), "{rx:03X}");
        }
        let on = sent_on(&sent.lock().unwrap());
        for id in [0x708, 0x709, 0x788, 0x7B8, 0x7C8] {
            assert!(!on.contains(&id), "a request went out on {id:03X}");
        }
        // the IDs around them were still asked
        for id in [0x787, 0x789, 0x7B7, 0x7B9, 0x7C7, 0x7C9] {
            assert!(on.contains(&id), "{id:03X} was not asked");
        }
    }

    #[test]
    fn modules_behind_the_gateway() {
        let toyota = crate::vehicles::find(&crate::vehicles::builtin(), "toyota")
            .unwrap()
            .clone();
        let body = crate::vehicles::candidates(&toyota)
            .into_iter()
            .find(|m| m.ext == Some(0x40))
            .unwrap();
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        let mut guard = ScanGuard::default();
        assert!(
            guard.present(&mut elm, &body).unwrap(),
            "750 with address 40 answers"
        );
        let (id, codes) = at(&mut elm, body.ext, |l| {
            Ok((
                module_identity(l, body.tx, body.rx())?,
                module_dtcs(l, body.tx, body.rx())?,
            ))
        })
        .unwrap();
        assert!(
            id.contains(&("part number", "89221-ICARUS".to_string())),
            "{id:?}"
        );
        let (d, codes) = codes.unwrap();
        assert_eq!((d, codes[0].code.as_str()), (Dialect::Uds, "B1234"));
        // another module at the same gateway, its own address
        let tpms = crate::vehicles::ModuleAddr {
            ext: Some(0x2A),
            name: "tire pressure".into(),
            ..body.clone()
        };
        assert!(guard.present(&mut elm, &tpms).unwrap());
        // afterwards normal addressing: 750 without an address byte reaches nothing, the engine answers as before
        assert!(!present(&mut elm, 0x750, 0x758).unwrap());
        assert_eq!(
            module_dtcs(&mut elm, 0x7E0, 0x7E8).unwrap().unwrap().1[0].code,
            "P0171"
        );
        // reset even when what ran for the module failed
        let e = at(&mut elm, Some(0x40), |_| -> Result<(), ObdError> {
            Err(ObdError::NoData)
        });
        assert!(e.is_err() && !present(&mut elm, 0x750, 0x758).unwrap());
    }

    #[test]
    fn iso14230_code_answers() {
        // two bytes per code, or three with a status byte; a count that fits neither is not read
        let two = parse_kwp13_dtcs(&[0x53, 0x02, 0x94, 0x21, 0x01, 0x71]).unwrap();
        assert_eq!(
            two.iter().map(|c| c.code.as_str()).collect::<Vec<_>>(),
            ["B1421", "P0171"]
        );
        let three = parse_kwp13_dtcs(&[0x53, 0x01, 0x52, 0x41, 0x60]).unwrap();
        assert_eq!(
            (three[0].code.as_str(), three[0].states.clone()),
            ("C1241", vec!["stored", "present now"])
        );
        assert_eq!(parse_kwp13_dtcs(&[0x53, 0x00]).unwrap().len(), 0);
        assert!(parse_kwp13_dtcs(&[0x53, 0x02, 0x94, 0x21, 0x01]).is_none());
        assert!(parse_kwp13_dtcs(&[0x7F, 0x13, 0x11]).is_none());
        assert!(
            check_read_only(&[0x13]).is_ok() && !crate::lock::changes_vehicle(&[0x13, 0xFF, 0x00])
        );
    }

    /// Every module a scan finds, as the command line's and the window's scans go: the reachable candidates, each asked
    /// through the scan's guard.
    fn scan(elm: &mut Elm, list: &[crate::vehicles::ModuleAddr]) -> Vec<u32> {
        let mut guard = ScanGuard::default();
        let reach = elm.reach();
        list.iter()
            .filter(|m| m.unreachable(reach).is_none())
            .filter(|m| guard.present(elm, m).unwrap())
            .map(|m| m.tx)
            .collect()
    }

    #[test]
    fn scanning_the_simulated_honda_on_29_bit_ids() {
        use crate::sim::recording::Recorder;
        use crate::vehicles::{candidates_on, for_vin};
        let (port, sent) = Recorder::new(SimCar::honda());
        let mut elm = Elm::connect(Box::new(port)).unwrap();
        assert_eq!(elm.protocol, crate::elm::Protocol::Can29At500);
        let all = crate::vehicles::builtin();
        let vin = crate::j1979::vin(&mut elm).unwrap().unwrap();
        assert_eq!(vin, crate::sim::HONDA_VIN);
        let honda = for_vin(&all, &vin).unwrap();
        assert_eq!(honda.id, "honda");
        // the profile's scan: OBD-II's engine and transmission at their 29-bit addresses, not 7E0-7E7
        let list = candidates_on(honda, elm.protocol);
        assert!(list.iter().all(|m| m.is_29bit()), "{list:?}");
        assert_eq!(scan(&mut elm, &list), vec![0x18DA_10F1, 0x18DA_18F1]);
        // 11-bit IDs cannot reach these modules, and say why
        let toyota_like = crate::vehicles::candidates(honda);
        let why = toyota_like[0].unreachable(elm.reach()).unwrap();
        assert!(why.contains("11-bit") && why.contains("29-bit"), "{why}");
        // the sweep asks every 29-bit address and finds all five modules
        sent.lock().unwrap().clear();
        let sweep = sweep_on(&all, elm.protocol);
        assert_eq!(sweep.len(), 255, "18DA00F1-18DAFFF1 but 18DAF1F1");
        assert_eq!(scan(&mut elm, &sweep), crate::sim::HONDA_MODULES);
        let on = sent_on(&sent.lock().unwrap());
        let skip = sweep_skips(&all);
        assert!(
            on.is_disjoint(&skip),
            "{:08X?}",
            on.intersection(&skip).collect::<Vec<_>>()
        );
        assert!(!on.contains(&0x18DB_33F1) && !on.iter().any(|id| id & 0xFFFF_FF00 == 0x18DA_F100));
        // the stability assist's identity and codes, over 29-bit headers
        let vsa = crate::vehicles::ModuleAddr {
            tx: 0x18DA_28F1,
            rx: None,
            name: "VSA".into(),
            ext: None,
            bus: None,
            checked: false,
        };
        assert_eq!(vsa.rx(), 0x18DA_F128);
        let id = module_identity(&mut elm, vsa.tx, vsa.rx()).unwrap();
        assert!(
            id.contains(&("part number", "57100-ICARUS".to_string())),
            "{id:?}"
        );
        let (d, codes) = module_dtcs(&mut elm, vsa.tx, vsa.rx()).unwrap().unwrap();
        assert_eq!((d, codes[0].code.as_str()), (Dialect::Uds, "C0210"));
        let log = sent.lock().unwrap().clone();
        for c in ["ATCP18", "ATSHDA28F1", "ATCRA18DAF128", "ATFCSH18DA28F1"] {
            assert!(log.contains(&format!("{c}\r")), "{c}: {log:?}");
        }
        // an 11-bit request on this vehicle is refused before anything is sent
        sent.lock().unwrap().clear();
        let e = elm
            .request_to(0x7E0, 0x7E8, &[0x3E, 0x00])
            .unwrap_err()
            .to_string();
        assert!(e.contains("11-bit") && e.contains("29-bit"), "{e}");
        assert!(sent.lock().unwrap().is_empty());
        // and the broadcast reaches every module again afterwards (18DB33F1 under priority 18)
        assert_eq!(
            crate::j1979::read(&mut elm, 0x0C).unwrap()[0].from,
            "18DAF110"
        );
        let log = sent.lock().unwrap().clone();
        assert_eq!(&log[..2], &["ATCP18\r", "ATSHDB33F1\r"], "{log:?}");
    }

    #[test]
    fn scanning_the_simulated_gm_finds_its_body_range() {
        use crate::sim::recording::Recorder;
        let (port, sent) = Recorder::new(SimCar::gm());
        let mut elm = Elm::connect(Box::new(port)).unwrap();
        let all = crate::vehicles::builtin();
        let vin = crate::j1979::vin(&mut elm).unwrap().unwrap();
        let gm = crate::vehicles::for_vin(&all, &vin).unwrap();
        assert_eq!(gm.id, "gm");
        let list = crate::vehicles::candidates_on(gm, elm.protocol);
        let body = list.iter().find(|m| m.tx == 0x241).unwrap();
        assert_eq!((body.rx(), body.checked), (0x641, false));
        sent.lock().unwrap().clear();
        let found = scan(&mut elm, &list);
        assert_eq!(
            found,
            crate::sim::GM_MODULES
                .iter()
                .map(|m| m.0)
                .collect::<Vec<_>>()
        );
        // the body range was asked on its request IDs only, never on an answer ID (64x) or the broadcast
        let on = sent_on(&sent.lock().unwrap());
        assert!((0x240..=0x24F).all(|id| on.contains(&id)), "{on:03X?}");
        assert!(on.is_disjoint(&sweep_skips(&all)), "{on:03X?}");
        assert!((0x640..=0x64F).all(|id| sweep_skips(&all).contains(&id)));
        let (identity, codes) = (
            module_identity(&mut elm, 0x241, 0x641).unwrap(),
            module_dtcs(&mut elm, 0x241, 0x641).unwrap().unwrap(),
        );
        assert!(identity.contains(&("part number", "ICARUS-GM-BCM".to_string())));
        assert_eq!(codes.1[0].code, "B1234");
        assert!(sent.lock().unwrap().contains(&"ATCRA641\r".to_string()));
        // the transmission at 7E2 answers from 7EA, in the broadcast too
        assert!(crate::j1979::supported(&mut elm)
            .unwrap()
            .iter()
            .any(|s| s.0 == "7EA"));
    }

    #[test]
    fn scanning_the_simulated_toyota() {
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        let all = crate::vehicles::builtin();
        let vin = crate::j1979::vin(&mut elm).unwrap().unwrap();
        let toyota = crate::vehicles::for_vin(&all, &vin).unwrap();
        assert_eq!(toyota.id, "toyota", "the VIN picks the profile");
        let reach = elm.reach();
        let found: Vec<u32> = crate::vehicles::candidates(toyota)
            .into_iter()
            .filter(|m| m.unreachable(reach).is_none())
            .filter(|m| present(&mut elm, m.tx, m.rx()).unwrap())
            .map(|m| m.tx)
            .collect();
        assert_eq!(
            found,
            vec![0x7E0, 0x7E1, 0x700, 0x701, 0x7B0, 0x780, 0x7C0, 0x7C4]
        );
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
        // the transmission refuses the status mask FF ("request out of range") and gives its codes to mask 2F
        let (d, codes) = module_dtcs(&mut elm, 0x7E1, 0x7E9).unwrap().unwrap();
        assert_eq!((d, codes.len()), (Dialect::Uds, 0));
        // the air conditioning refuses UDS and KWP2000's 18 and answers ISO 14230's 13
        let (d, codes) = module_dtcs(&mut elm, 0x7C4, 0x7CC).unwrap().unwrap();
        assert_eq!(
            (d, codes[0].code.as_str(), codes[0].states.clone()),
            (Dialect::Kwp, "B1421", vec!["stored"])
        );
        // a module that answers none: every request named with what it got
        let why = module_dtcs(&mut elm, 0x7A1, 0x7A9).unwrap().unwrap_err();
        assert!(
            why.starts_with("19 02 FF: no answer") && why.ends_with("13: no answer"),
            "{why}"
        );
        // after module requests, an OBD broadcast reaches every module again
        assert_eq!(crate::j1979::read(&mut elm, 0x0C).unwrap().len(), 1);
        assert!(elm.request(&[0x01, 0x00]).unwrap().len() >= 2);
    }
}
