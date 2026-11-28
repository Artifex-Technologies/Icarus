//! ICA-210: `icarus logs compare` through the command itself, on two made-up recordings in a home of the test's own (HOME
//! in a temp folder, removed after), so nothing of the real home is read.

use std::path::PathBuf;
use std::process::Command;

/// A recording: `n` warm, closed-loop rows at 2200 rpm, 70 % load, 20 g/s, with this timing.
fn drive(n: usize, timing: f64) -> String {
    let mut s = String::from(
        "# adapter: made-up adapter\ntime_s,\"Engine speed [rpm]\",\"Engine load [%]\",\"Timing advance [° before TDC]\",\
         \"Mass air flow [g/s]\",\"Short-term fuel trim, bank 1 [%]\",\"Long-term fuel trim, bank 1 [%]\",\
         \"Coolant temperature [°C]\",\"Commanded lambda [λ]\"\n",
    );
    for i in 0..n {
        s.push_str(&format!(
            "{:.1},2200,70,{timing:.1},20,1.0,-4.0,90,1.00\n",
            i as f64 * 0.5
        ));
    }
    s
}

#[test]
fn two_recordings_by_name_and_a_minimum_row_count() {
    let home: PathBuf = std::env::temp_dir().join(format!(
        "icarus-logs-compare-{}-{:?}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos()
    ));
    let logs = home.join("Documents/Icarus/logs");
    std::fs::create_dir_all(&logs).unwrap();
    std::fs::write(logs.join("a.csv"), drive(12, 25.0)).unwrap();
    std::fs::write(logs.join("b.csv"), drive(12, 20.0)).unwrap();
    let run = |args: &[&str]| {
        let o = Command::new(env!("CARGO_BIN_EXE_icarus"))
            .args(args)
            .env("HOME", &home)
            // the binary is built without icarus-obd's test-home guard (ICA-225): a config or state folder from the
            // shell that runs the tests must not reach the real one
            .env("XDG_CONFIG_HOME", home.join("config"))
            .env("XDG_STATE_HOME", home.join("state"))
            .env("XDG_DATA_HOME", home.join("data"))
            .env_remove("ICARUS_ADAPTER")
            .env_remove("ICARUS_READ_ONLY")
            .env("ICARUS_UNITS", "us")
            .stdin(std::process::Stdio::null())
            .output()
            .unwrap();
        (
            o.status.success(),
            String::from_utf8_lossy(&o.stdout).into_owned(),
            String::from_utf8_lossy(&o.stderr).into_owned(),
        )
    };
    let (ok, out, err) = run(&["logs", "compare", "a.csv", "b.csv"]);
    assert!(ok, "{err}");
    assert!(
        out.contains("25.0° (12 rows)") && out.contains("-5.0°"),
        "{out}"
    );
    assert!(out.contains("coolant 176 °F (80 °C) or more"), "{out}");
    assert!(out.contains("made-up adapter"), "{out}");
    let (ok, out, _) = run(&["logs", "compare", "a.csv", "b.csv", "--min-rows", "20"]);
    assert!(ok);
    assert!(
        out.contains("thin (12 rows)") && !out.contains("-5.0°"),
        "{out}"
    );
    let (ok, _, err) = run(&["logs", "compare", "a.csv"]);
    assert!(!ok && err.contains("usage"), "{err}");
    let (ok, _, err) = run(&["logs", "compare", "a.csv", "missing.csv"]);
    assert!(!ok && err.contains("missing.csv"), "{err}");
    let _ = std::fs::remove_dir_all(&home);
}
