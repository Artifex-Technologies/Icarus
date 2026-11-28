//! ICA-202: `icarus obd codes` on the simulated vehicle prints the freeze frame in the units setting's units (ICARUS_UNITS):
//! mph and °F first with metric after for `us`, km/h and °C alone for `metric`. Each run gets a home of its own (HOME and
//! the XDG folders in a temp folder, removed after), so nothing of the real home is read or written.

use std::path::PathBuf;
use std::process::Command;

fn codes(units: &str) -> String {
    let home: PathBuf = std::env::temp_dir().join(format!(
        "icarus-codes-units-{units}-{}-{:?}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos()
    ));
    std::fs::create_dir_all(&home).unwrap();
    let o = Command::new(env!("CARGO_BIN_EXE_icarus"))
        .args(["obd", "codes", "--adapter", "sim"])
        .env("HOME", &home)
        .env("XDG_CONFIG_HOME", home.join("config"))
        .env("XDG_STATE_HOME", home.join("state"))
        .env("XDG_DATA_HOME", home.join("data"))
        .env("ICARUS_UNITS", units)
        .env_remove("ICARUS_ADAPTER")
        .env_remove("ICARUS_READ_ONLY")
        .stdin(std::process::Stdio::null())
        .output()
        .unwrap();
    let _ = std::fs::remove_dir_all(&home);
    let out = String::from_utf8_lossy(&o.stdout).into_owned();
    assert!(
        o.status.success(),
        "{out}{}",
        String::from_utf8_lossy(&o.stderr)
    );
    // the freeze frame's part of what it printed
    out.split("Freeze frame")
        .nth(1)
        .unwrap_or_else(|| panic!("no freeze frame in\n{out}"))
        .to_string()
}

#[test]
fn the_freeze_frame_follows_the_units_setting() {
    let us = codes("us");
    for want in [
        "44.7 mph (72.0 km/h)",
        "194.0 °F (90.0 °C)",
        "7.0 psi absolute (48.0 kPa)",
    ] {
        assert!(us.contains(want), "{want} in\n{us}");
    }
    let metric = codes("metric");
    for want in ["72.0 km/h", "90.0 °C", "48.0 kPa"] {
        assert!(metric.contains(want), "{want} in\n{metric}");
    }
    assert!(
        !metric.contains("mph") && !metric.contains("°F") && !metric.contains("psi"),
        "{metric}"
    );
}
