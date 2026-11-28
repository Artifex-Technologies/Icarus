//! ICA-211 and ICA-212 through the command itself, in a home of the test's own (HOME and the XDG folders in a temp folder,
//! removed after), so nothing of the real home is read or written. The VINs are made up (the first is the textbook example
//! from the check-digit rules), not vehicles of mine.

use std::path::{Path, PathBuf};
use std::process::Command;

const VIN: &str = "1M8GDM9AXKP042788";
const OTHER: &str = "1ABCDEFGH12345678";

struct Home(PathBuf);

impl Home {
    fn new() -> Home {
        let dir = std::env::temp_dir().join(format!(
            "icarus-fuel-cli-{}-{:?}",
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

    fn ok(&self, args: &[&str]) -> String {
        let (ok, out, err) = self.run(args);
        assert!(ok, "{args:?}: {out}{err}");
        out
    }
}

impl Drop for Home {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// A made-up recording with the recorder's first lines (no `# fuel:` line: it is worked out when read): `n` warm,
/// closed-loop rows at 2200 rpm, 70 % load, 20 g/s, with this timing.
fn drive(vehicle: &str, began: &str, n: usize, timing: f64) -> String {
    let mut s = format!(
        "# adapter: made-up adapter\n# vehicle: {vehicle}\n# began: {began}\ntime_s,\"Engine speed [rpm]\",\
         \"Engine load [%]\",\"Timing advance [° before TDC]\",\"Mass air flow [g/s]\",\
         \"Short-term fuel trim, bank 1 [%]\",\"Long-term fuel trim, bank 1 [%]\",\"Coolant temperature [°C]\",\
         \"Commanded lambda [λ]\"\n"
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
fn fill_ups_tag_drives_when_read_and_an_experiment_says_what_is_missing() {
    let h = Home::new();
    h.ok(&["garage", VIN, "2017 Toyota Tacoma 3.5L V6"]);
    // the first fill-up says what was in the tank; the Tacoma's 21.1 gallons from its description
    let out = h.ok(&[
        "fuel",
        "add",
        "--vin",
        VIN,
        "--grade",
        "91",
        "--ethanol",
        "0",
        "--gallons",
        "12",
        "--date",
        "2026-11-01 09:00",
        "--before",
        "91 E0",
    ]);
    assert!(
        out.contains("blend 91 E0") && out.contains("taken as 91 E0, as stated"),
        "{out}"
    );
    h.ok(&[
        "fuel",
        "add",
        "--grade",
        "87",
        "--ethanol",
        "10",
        "--gallons",
        "21.1",
        "--date",
        "2026-11-10 12:00",
    ]);
    let list = h.ok(&["fuel", "list"]);
    assert!(
        list.contains("2026-11-10 12:00  87 E10  21.1 gal") && list.contains("blend 87 E10"),
        "{list}"
    );
    // a second vehicle with a folder of its own
    h.ok(&["garage", OTHER, "2010 Honda Civic"]);
    h.ok(&[
        "fuel",
        "add",
        "--vin",
        OTHER,
        "--tank",
        "13",
        "--grade",
        "87",
        "--ethanol",
        "10",
        "--gallons",
        "13",
        "--date",
        "2026-10-01",
    ]);

    let logs = h.path().join("Documents/Icarus/logs");
    std::fs::create_dir_all(&logs).unwrap();
    let truck = "2017 Toyota Tacoma 3.5L V6";
    std::fs::write(
        logs.join("drive-a.csv"),
        drive(truck, "2026-11-05 08:00:00", 140, 25.0),
    )
    .unwrap();
    std::fs::write(
        logs.join("drive-b.csv"),
        drive(truck, "2026-11-11 08:00:00", 80, 21.0),
    )
    .unwrap();
    // another vehicle's drive, and one with no rows
    std::fs::write(
        logs.join("drive-c.csv"),
        drive("2010 Honda Civic", "2026-11-11 09:00:00", 80, 30.0),
    )
    .unwrap();
    std::fs::write(
        logs.join("drive-d.csv"),
        drive(truck, "2026-11-12 08:00:00", 0, 0.0),
    )
    .unwrap();

    // two drives: each shown with its tank, worked out now (they carry no fuel line)
    let out = h.ok(&["logs", "compare", "drive-a.csv", "drive-b.csv"]);
    assert!(
        out.contains("91 E0 tank, blend 91 E0") && out.contains("87 E10 tank, blend 87 E10"),
        "{out}"
    );
    // pooled by fuel: two vehicles in the garage, so --vin; the other vehicle's drive and the empty one left out
    let (ok, _, err) = h.run(&["logs", "compare", "--by-fuel"]);
    assert!(!ok && err.contains("which vehicle?"), "{err}");
    let out = h.ok(&["logs", "compare", "--by-fuel", "--vin", VIN]);
    assert!(out.starts_with("Comparing 2 groups of drives"), "{out}");
    assert!(
        out.contains("1. 91 E0: 140 of 140 rows") && out.contains("(drive-a.csv)"),
        "{out}"
    );
    assert!(
        out.contains("2. 87 E10: 80 of 80 rows") && out.contains("(drive-b.csv)"),
        "{out}"
    );
    assert!(
        !out.contains("drive-c.csv"),
        "another vehicle's drive: {out}"
    );
    assert!(
        out.contains("Skipped, not readable or with no rows: drive-d.csv"),
        "{out}"
    );
    assert!(out.contains("-4.0°"), "{out}");
    assert!(
        out.contains("1 recording of other vehicles left out"),
        "{out}"
    );

    // an experiment: 87 has 40 s of the 60 a group needs
    let out = h.ok(&[
        "logs",
        "experiment",
        "91 vs 87",
        "91 E0",
        "87 E10",
        "--vin",
        VIN,
    ]);
    assert!(
        out.contains("20 s more at 60-80 % load, 1,800-2,600 rpm, on 87 E10 (has 40 s)"),
        "{out}"
    );
    let kept = std::fs::read_to_string(h.path().join(format!(
        "Documents/Icarus/Vehicles/{truck}/experiments.toml"
    )))
    .unwrap();
    assert!(
        kept.contains("[\"91 vs 87\"]") && kept.contains("b = \"87 E10\""),
        "{kept}"
    );
    // a fill-up logged afterwards, dated before drive-b: drive-b is now on it (worked out when read)
    h.ok(&[
        "fuel",
        "add",
        "--vin",
        VIN,
        "--grade",
        "91",
        "--ethanol",
        "0",
        "--gallons",
        "21.1",
        "--date",
        "2026-11-11 07:00",
    ]);
    let out = h.ok(&["logs", "experiment", "91 vs 87", "--vin", VIN]);
    assert!(
        out.contains("91 E0 (2 drives) against 87 E10 (0 drives)"),
        "{out}"
    );
    assert!(out.contains("No recordings on 87 E10 yet"), "{out}");
}
