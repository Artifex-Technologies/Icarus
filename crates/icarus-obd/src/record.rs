//! What each vehicle answered (ICA-70, PLAN_LEARN_VEHICLES): every request Icarus sent to one module and what came back,
//! data, a refusal with its reason, or silence, kept per vehicle so a later connection (and jev, later) can learn from it.
//! What a vehicle refuses, and why, is data too.
//!
//! Kept in the vehicle's garage folder (`record.json`; the simulated vehicle's in Icarus's state folder), never in the
//! repository: VINs and what real vehicles answered stay out of it. One entry per module and request, with how often it was
//! asked, when first and last, and the adapter and protocol of the last time.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// What a module did with a request.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Got {
    /// An answer with this many bytes.
    Data(usize),
    /// A refusal (7F) with this reason code.
    Refused(u8),
    /// No answer.
    Silent,
}

/// One request to one module and what it brought back (`Elm::take_outcomes`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Outcome {
    pub tx: u32,
    /// The address byte in front of each frame, for a module behind a gateway (Toyota's 750).
    pub ext: Option<u8>,
    pub request: Vec<u8>,
    pub got: Got,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Entry {
    /// "7E0", or "750/2A" behind a gateway.
    pub module: String,
    /// The request's bytes in hex ("22F190").
    pub request: String,
    /// "data", "refused" or "silent".
    pub got: String,
    /// A refusal's code and its reason in words.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub code: Option<u8>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
    /// An answer's length.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bytes: Option<usize>,
    pub count: u32,
    /// Seconds since 1970, the first and the last time.
    pub first: u64,
    pub last: u64,
    pub adapter: String,
    pub protocol: String,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Record {
    /// By "MODULE REQUEST".
    pub requests: BTreeMap<String, Entry>,
}

fn id_text(id: u32) -> String {
    if id > 0x7FF {
        format!("{id:08X}")
    } else {
        format!("{id:03X}")
    }
}

impl Record {
    /// Takes in what one connection learned (`adapter`, `protocol` as said, `now` in seconds since 1970).
    pub fn add(&mut self, outcomes: &[Outcome], adapter: &str, protocol: &str, now: u64) {
        for o in outcomes {
            let module = match o.ext {
                Some(a) => format!("{}/{a:02X}", id_text(o.tx)),
                None => id_text(o.tx),
            };
            let request: String = o.request.iter().map(|b| format!("{b:02X}")).collect();
            let e = self
                .requests
                .entry(format!("{module} {request}"))
                .or_insert_with(|| Entry {
                    module,
                    request,
                    first: now,
                    ..Default::default()
                });
            let (got, code, reason, bytes) = match o.got {
                Got::Data(n) => ("data", None, None, Some(n)),
                Got::Refused(c) => (
                    "refused",
                    Some(c),
                    Some(crate::uds::nrc(c).to_string()),
                    None,
                ),
                Got::Silent => ("silent", None, None, None),
            };
            e.got = got.into();
            e.code = code;
            e.reason = reason;
            e.bytes = bytes;
            e.count += 1;
            e.last = now;
            e.adapter = adapter.into();
            e.protocol = protocol.into();
        }
    }

    /// A module's entries, by request.
    pub fn module(&self, module: &str) -> Vec<&Entry> {
        self.requests
            .values()
            .filter(|e| e.module == module)
            .collect()
    }
}

/// Where `vin`'s record is kept: its garage folder (made when missing), or the state folder for the simulated vehicle.
pub fn path(vin: &str) -> Result<PathBuf, String> {
    let dir = if vin == crate::sim::VIN {
        crate::audit::path()
            .and_then(|p| p.parent().map(|d| d.join("sim")))
            .ok_or("no state folder")?
    } else {
        crate::garage::ensure(vin)
            .map_err(|e| format!("the vehicle's folder could not be made: {e}"))?
    };
    std::fs::create_dir_all(&dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    Ok(dir.join("record.json"))
}

/// The record in `file` (empty when there is none or it does not read).
pub fn load_in(file: &Path) -> Record {
    std::fs::read(file)
        .ok()
        .and_then(|b| serde_json::from_slice(&b).ok())
        .unwrap_or_default()
}

/// Adds `outcomes` to the record in `file` (through a file beside it, renamed over, so a stop halfway leaves the old one).
pub fn merge_in(
    file: &Path,
    outcomes: &[Outcome],
    adapter: &str,
    protocol: &str,
    now: u64,
) -> Result<(), String> {
    if outcomes.is_empty() {
        return Ok(());
    }
    let mut r = load_in(file);
    r.add(outcomes, adapter, protocol, now);
    let text = serde_json::to_string_pretty(&r).map_err(|e| e.to_string())?;
    let tmp = file.with_extension("json.tmp");
    std::fs::write(&tmp, text).map_err(|e| format!("{}: {e}", tmp.display()))?;
    std::fs::rename(&tmp, file).map_err(|e| format!("{}: {e}", file.display()))
}

/// Adds `outcomes` to `vin`'s record.
pub fn merge(vin: &str, outcomes: &[Outcome], adapter: &str, protocol: &str) -> Result<(), String> {
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    merge_in(&path(vin)?, outcomes, adapter, protocol, now)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elm::Elm;
    use crate::link::Link;

    /// ICA-70: a scan of the simulated vehicle's engine module leaves each request with its outcome in the record: the
    /// values it gave, the ones it refused with the reason, and a module behind the gateway by its address byte; a second
    /// connection adds to the counts.
    #[test]
    fn each_request_is_kept_with_what_came_back() {
        let mut elm = Elm::connect(Box::new(crate::sim::SimCar::new())).unwrap();
        crate::uds::module_identity_notes(&mut elm, 0x7E0, 0x7E8).unwrap();
        crate::uds::at(&mut elm, Some(0x2A), |l| {
            l.request_to(0x750, 0x758, &[0x21, 0x04]).map(|_| ())
        })
        .unwrap();
        let out = elm.take_outcomes();
        assert!(!out.is_empty());
        assert!(elm.take_outcomes().is_empty(), "taken once");
        let file = std::env::temp_dir().join(format!("icarus-record-{}.json", std::process::id()));
        let _ = std::fs::remove_file(&file);
        let adapter = Link::adapter(&elm);
        merge_in(&file, &out, &adapter, "ISO 15765-4 CAN", 1_800_000_000).unwrap();
        let r = load_in(&file);
        let engine = r.module("7E0");
        let vin = engine
            .iter()
            .find(|e| e.request == "22F190")
            .expect("the VIN read");
        assert_eq!((vin.got.as_str(), vin.count), ("data", 1));
        let refused: Vec<&&Entry> = engine.iter().filter(|e| e.got == "refused").collect();
        assert!(!refused.is_empty(), "{engine:?}");
        assert!(
            refused
                .iter()
                .all(|e| e.code.is_some() && e.reason.as_deref().is_some_and(|r| !r.is_empty())),
            "each refusal with its reason"
        );
        assert!(
            r.requests.contains_key("750/2A 2104"),
            "behind the gateway: {:?}",
            r.requests.keys().collect::<Vec<_>>()
        );
        // a second connection: the same keys, counted twice, the first time kept
        merge_in(&file, &out, &adapter, "ISO 15765-4 CAN", 1_800_000_100).unwrap();
        let again = load_in(&file);
        let e = &again.requests["7E0 22F190"];
        assert_eq!(
            (e.count, e.first, e.last),
            (2, 1_800_000_000, 1_800_000_100)
        );
        assert_eq!(again.requests.len(), r.requests.len());
        let _ = std::fs::remove_file(&file);
    }
}
