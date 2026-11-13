//! Connecting by itself: the adapter to use when none was named (the one that last worked, else a paired Bluetooth OBD adapter),
//! and how long to wait between tries while it sleeps (an OBDLink MX+ powers down with the engine off and answers again once
//! the engine starts or its button is pressed).

use std::path::PathBuf;
use std::time::Duration;

/// Between two tries while no adapter answers.
pub const RETRY: Duration = Duration::from_secs(10);

/// Names of Bluetooth OBD adapters, as they announce themselves (lowercase, matched anywhere in the name).
const OBD_NAMES: [&str; 10] = [
    "obdlink", "obd", "elm327", "vlinker", "veepeak", "vgate", "bafx", "carista", "konnwei", "icar",
];

fn remembered_path() -> Option<PathBuf> {
    let base = std::env::var_os("XDG_CONFIG_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
    Some(base.join("icarus/adapter"))
}

/// The adapter that last connected (one line, as `--adapter` takes it), when there is one.
pub fn remembered() -> Option<String> {
    let t = std::fs::read_to_string(remembered_path()?).ok()?;
    let a = t.lines().next()?.trim().to_string();
    (!a.is_empty() && a != "sim").then_some(a)
}

/// Keeps `spec` as the adapter to use next time (never the simulated vehicle).
pub fn remember(spec: &str) {
    let spec = spec.trim();
    if spec.is_empty() || spec == "sim" || spec.contains('\n') {
        return;
    }
    if let Some(p) = remembered_path() {
        if let Some(d) = p.parent() {
            let _ = std::fs::create_dir_all(d);
        }
        let _ = std::fs::write(p, format!("{spec}\n"));
    }
}

/// The paired Bluetooth OBD adapters in `bluetoothctl devices Paired` ("Device AA:BB:CC:DD:EE:FF OBDLink MX+ 46448"), as
/// `bt:` adapters, in the order listed.
pub fn paired_obd(listing: &str) -> Vec<String> {
    listing
        .lines()
        .filter_map(|l| {
            let mut w = l.split_whitespace();
            (w.next()? == "Device").then_some(())?;
            let mac = w.next()?;
            let name = w.collect::<Vec<_>>().join(" ").to_lowercase();
            let ok_mac = mac.len() == 17
                && mac
                    .split(':')
                    .all(|p| p.len() == 2 && p.chars().all(|c| c.is_ascii_hexdigit()));
            (ok_mac && OBD_NAMES.iter().any(|n| name.contains(n))).then(|| format!("bt:{mac}"))
        })
        .collect()
}

fn paired_obd_here() -> Vec<String> {
    std::process::Command::new("bluetoothctl")
        .args(["devices", "Paired"])
        .stderr(std::process::Stdio::null())
        .output()
        .map(|o| paired_obd(&String::from_utf8_lossy(&o.stdout)))
        .unwrap_or_default()
}

/// The adapter to start with, and whether it is a real one to connect to by itself: the one named (`--adapter`, then
/// ICARUS_ADAPTER), else the one that last worked, else a paired Bluetooth OBD adapter, else the simulated vehicle (not
/// connected by itself).
pub fn choose(
    named: Option<String>,
    remembered: Option<String>,
    paired: impl FnOnce() -> Vec<String>,
) -> (String, bool) {
    if let Some(n) = named.filter(|n| !n.trim().is_empty()) {
        let real = n.trim() != "sim";
        return (n, real);
    }
    if let Some(r) = remembered {
        return (r, true);
    }
    match paired().into_iter().next() {
        Some(p) => (p, true),
        None => ("sim".into(), false),
    }
}

pub fn choose_here(named: Option<String>) -> (String, bool) {
    choose(named, remembered(), paired_obd_here)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn paired_obd_adapters_are_found_by_name() {
        let list = "Device AA:BB:CC:DD:EE:01 OBDLink MX+ 46448\nDevice AA:BB:CC:DD:EE:02 Pixel Buds\nDevice AA:BB:CC:DD:EE:03 vLinker FS\n\
                    Device not-a-mac OBDII\nDevice AA:BB:CC:DD:EE:04 V-LINK OBDII\n";
        assert_eq!(
            paired_obd(list),
            [
                "bt:AA:BB:CC:DD:EE:01",
                "bt:AA:BB:CC:DD:EE:03",
                "bt:AA:BB:CC:DD:EE:04"
            ]
        );
        assert!(paired_obd("").is_empty());
    }

    #[test]
    fn the_adapter_named_then_remembered_then_paired() {
        let none = || Vec::<String>::new();
        let one = || vec!["bt:AA:BB:CC:DD:EE:01".to_string()];
        assert_eq!(
            choose(Some("/dev/ttyUSB0".into()), Some("bt:X".into()), one),
            ("/dev/ttyUSB0".into(), true)
        );
        assert_eq!(
            choose(Some("sim".into()), None, one),
            ("sim".into(), false),
            "the simulator is never auto-connected"
        );
        assert_eq!(
            choose(None, Some("bt:X".into()), one),
            ("bt:X".into(), true)
        );
        assert_eq!(
            choose(None, None, one),
            ("bt:AA:BB:CC:DD:EE:01".into(), true)
        );
        assert_eq!(choose(None, None, none), ("sim".into(), false));
    }
}
