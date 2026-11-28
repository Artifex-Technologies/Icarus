//! ICA-185 and ICA-215: `icarus garage service` through the command itself, in a home of the test's own (HOME and the XDG
//! folders in a temp folder, removed after), so nothing of the real home is read or written. The VIN is the textbook
//! example from the check-digit rules, not a vehicle of mine.

use std::path::{Path, PathBuf};
use std::process::Command;

const VIN: &str = "1M8GDM9AXKP042788";

struct Home(PathBuf);

impl Home {
    fn new() -> Home {
        let dir = std::env::temp_dir().join(format!(
            "icarus-maintenance-cli-{}-{:?}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        Home(dir)
    }

    fn path(&self) -> &Path {
        &self.0
    }

    fn run(&self, args: &[&str]) -> (bool, String, String) {
        let o = Command::new(env!("CARGO_BIN_EXE_icarus"))
            .args(args)
            .env("HOME", self.path())
            .env("XDG_CONFIG_HOME", self.path().join("config"))
            .env("XDG_STATE_HOME", self.path().join("state"))
            .env("XDG_DATA_HOME", self.path().join("data"))
            .env("ICARUS_UNITS", "us")
            .env_remove("ICARUS_ADAPTER")
            .stdin(std::process::Stdio::null())
            .output()
            .unwrap();
        (
            o.status.success(),
            String::from_utf8_lossy(&o.stdout).into_owned(),
            String::from_utf8_lossy(&o.stderr).into_owned(),
        )
    }
}

impl Drop for Home {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// A made-up recording: `n` warm, closed-loop rows at 2200 rpm, 70 % load, 20 g/s, with this timing, and the odometer
/// (standard value A6, in km) when given.
fn drive(n: usize, timing: f64, km: Option<f64>) -> String {
    let mut s = String::from(
        "# adapter: made-up adapter\ntime_s,\"Engine speed [rpm]\",\"Engine load [%]\",\"Timing advance [° before TDC]\",\
         \"Mass air flow [g/s]\",\"Short-term fuel trim, bank 1 [%]\",\"Long-term fuel trim, bank 1 [%]\",\
         \"Coolant temperature [°C]\",\"Commanded lambda [λ]\",\"Odometer [km]\"\n",
    );
    for i in 0..n {
        let odo = km.map(|k| format!("{k:.1}")).unwrap_or_default();
        s.push_str(&format!(
            "{:.1},2200,70,{timing:.1},20,1.0,-4.0,90,1.00,{odo}\n",
            i as f64 * 0.5
        ));
    }
    s
}

#[test]
fn a_service_logged_listed_due_and_compared() {
    let h = Home::new();
    let (ok, out, err) = h.run(&[
        "garage",
        "service",
        VIN,
        "add",
        "oil",
        "--miles",
        "80,000",
        "--date",
        "2026-05-01",
        "--note",
        "0W-20",
    ]);
    assert!(ok, "{err}");
    assert!(
        out.contains("Logged: 2026-05-01 at 80,000 mi: oil and filter (0W-20)"),
        "{out}"
    );
    let file = h
        .path()
        .join("Documents/Icarus/Vehicles")
        .join(VIN)
        .join("maintenance.toml");
    assert!(file.is_file(), "the log is in the vehicle's folder");
    // the garage holds one vehicle: the VIN may be left out
    let (ok, _, err) = h.run(&[
        "garage",
        "service",
        "add",
        "spark plugs",
        "--miles",
        "84213",
        "--date",
        "2026-11-29",
    ]);
    assert!(ok, "{err}");
    let (ok, out, err) = h.run(&["garage", "service", "interval", "oil", "--miles", "5000"]);
    assert!(
        ok && out.contains("oil and filter: every 5,000 mi."),
        "{out}{err}"
    );
    let (ok, out, _) = h.run(&["garage", "service", VIN, "list"]);
    assert!(ok);
    assert!(
        out.contains("2026-05-01 at 80,000 mi: oil and filter (0W-20)")
            && out.contains("2026-11-29 at 84,213 mi: spark plugs")
            && out.contains("oil and filter: every 5,000 mi"),
        "{out}"
    );
    // due by miles: at 85,000; 400 left at 84,600 is soon, 300 past at 85,300 overdue
    let (ok, out, _) = h.run(&["garage", "service", VIN, "due", "--miles", "84600"]);
    assert!(ok);
    assert!(
        out.contains("oil and filter [due soon]: in 400 mi (at 85,000 mi)"),
        "{out}"
    );
    let (_, out, _) = h.run(&["garage", "service", VIN, "due", "--miles", "85300"]);
    assert!(out.contains("[OVERDUE]: overdue by 300 mi"), "{out}");
    // with no miles given, the highest logged is used and says so
    let (_, out, _) = h.run(&["garage", "service", VIN, "due"]);
    assert!(
        out.contains("Odometer: 84,213 mi (as logged on 2026-11-29)")
            && out.contains("[ok]: in 787 mi"),
        "{out}"
    );

    // recordings on each side of the plugs: compared under its name
    let logs = h.path().join("Documents/Icarus/logs");
    std::fs::create_dir_all(&logs).unwrap();
    std::fs::write(
        logs.join("drive-2026-11-27_183005.csv"),
        drive(12, 20.0, None),
    )
    .unwrap();
    std::fs::write(
        logs.join("drive-2026-11-30_071500.csv"),
        drive(12, 25.0, Some(135_603.3)),
    )
    .unwrap();
    let (ok, out, err) = h.run(&["garage", "service", VIN, "compare", "spark-plugs"]);
    assert!(ok, "{err}");
    assert!(
        out.starts_with("Before and after spark plugs: 2026-11-29 at 84,213 mi: spark plugs"),
        "{out}"
    );
    assert!(
        out.contains("1. Before spark plugs") && out.contains("2. After spark plugs"),
        "{out}"
    );
    assert!(
        out.contains("20.0° (12 rows)") && out.contains("+5.0°"),
        "{out}"
    );
    assert!(out.contains("carry no misfire counts"), "{out}");
    assert!(out.contains("coolant 176 °F (80 °C) or more"), "{out}");
    // a minimum the sides do not reach: said plainly
    let (ok, out, _) = h.run(&[
        "garage",
        "service",
        VIN,
        "compare",
        "2026-11-29",
        "--min-rows",
        "20",
    ]);
    assert!(ok);
    assert!(
        out.contains("Before spark plugs has only 12 rows at matching conditions, fewer than the minimum of 20"),
        "{out}"
    );

    // the odometer a recording read is offered, never saved by itself (no terminal here: said, not asked)
    let (ok, out, _) = h.run(&["garage", "service", VIN, "odometer"]);
    assert!(
        ok && out.contains("84,260 mi from the newest recording"),
        "{out}"
    );
    let (ok, out, err) = h.run(&[
        "garage",
        "service",
        VIN,
        "add",
        "air-filter",
        "--date",
        "2026-12-01",
    ]);
    assert!(ok, "{err}");
    assert!(
        out.contains("Logged: 2026-12-01: air filter")
            && out.contains("A reading to confirm: 84,260 mi"),
        "{out}"
    );
    assert!(!std::fs::read_to_string(&file).unwrap().contains("84260"));

    // mistakes are refused
    let (ok, _, err) = h.run(&["garage", "service", VIN, "add", "other"]);
    assert!(!ok && err.contains("note"), "{err}");
    let (ok, _, err) = h.run(&["garage", "service", VIN, "add", "wipers"]);
    assert!(!ok && err.contains("not a kind of service"), "{err}");
    let (ok, _, err) = h.run(&[
        "garage",
        "service",
        VIN,
        "add",
        "oil",
        "--date",
        "11/29/2026",
    ]);
    assert!(!ok && err.contains("not a date"), "{err}");
    let (ok, _, err) = h.run(&["garage", "service", VIN, "compare", "brakes"]);
    assert!(!ok && err.contains("no brakes logged"), "{err}");
    let (ok, _, err) = h.run(&["garage", "service", VIN]);
    assert!(!ok && err.contains("usage"), "{err}");
}
