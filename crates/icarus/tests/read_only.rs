//! ICA-192: the session's Read-Only floor through the command itself, `--read-only` and ICARUS_READ_ONLY=1, on the simulated
//! vehicle. Each run gets a home of its own (HOME and the XDG folders in a temp folder, removed after), so no vehicle is
//! switched (no read-only.toml) and nothing of the real home is read or written.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

/// A home of the test's own, removed when dropped.
struct Home(PathBuf);

impl Home {
    fn new(tag: &str) -> Home {
        let dir = std::env::temp_dir().join(format!(
            "icarus-read-only-{tag}-{}-{:?}",
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

    /// `icarus ARGS` in this home, with ICARUS_READ_ONLY set to `env` (unset for None) and no default adapter.
    fn icarus(&self, args: &[&str], env: Option<&str>) -> Output {
        let mut c = Command::new(env!("CARGO_BIN_EXE_icarus"));
        c.args(args)
            .env("HOME", self.path())
            .env("XDG_CONFIG_HOME", self.path().join("config"))
            .env("XDG_STATE_HOME", self.path().join("state"))
            .env("XDG_DATA_HOME", self.path().join("data"))
            .env_remove("ICARUS_ADAPTER")
            .env_remove("ICARUS_READ_ONLY")
            .stdin(std::process::Stdio::null());
        if let Some(v) = env {
            c.env("ICARUS_READ_ONLY", v);
        }
        c.output().unwrap()
    }
}

impl Drop for Home {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn text(b: &[u8]) -> String {
    String::from_utf8_lossy(b).into_owned()
}

/// What changes a vehicle, each a command that would clear, test, change a setting or run a routine on the simulated
/// vehicle with its engine off.
const WRITES: [&[&str]; 6] = [
    &["obd", "clear", "--adapter", "sim:off", "--yes"],
    &[
        "obd",
        "clear",
        "--module",
        "7E0",
        "--adapter",
        "sim:off",
        "--yes",
    ],
    &[
        "obd",
        "test",
        "Cooling fan",
        "--adapter",
        "sim:off",
        "--yes",
    ],
    &[
        "obd",
        "setting",
        "Cluster: seat belt chime",
        "off",
        "--adapter",
        "sim:off",
        "--yes",
    ],
    &[
        "obd",
        "setting-undo",
        "Cluster: seat belt chime",
        "--adapter",
        "sim:off",
        "--yes",
    ],
    &[
        "obd",
        "service",
        "Oil life reset (simulated)",
        "--adapter",
        "sim:off",
        "--yes",
    ],
];

/// With the floor on, by the flag (before the command or after it) or by the variable, each write is refused with
/// 'session read-only' before the adapter is opened (so before any frame), and the refusal is in the action log.
#[test]
fn every_write_is_refused_before_the_adapter_opens() {
    for how in ["flag first", "flag last", "variable"] {
        let home = Home::new("writes");
        for w in WRITES {
            let (args, env): (Vec<&str>, _) = match how {
                "flag first" => (
                    std::iter::once("--read-only")
                        .chain(w.iter().copied())
                        .collect(),
                    None,
                ),
                "flag last" => (w.iter().copied().chain(["--read-only"]).collect(), None),
                _ => (w.to_vec(), Some("1")),
            };
            let out = home.icarus(&args, env);
            let err = text(&out.stderr);
            assert!(!out.status.success(), "{how}: {args:?} went: {err}");
            assert!(err.contains("session read-only"), "{how}: {args:?}: {err}");
            assert!(
                !err.contains("connecting to"),
                "{how}: {args:?} opened the adapter: {err}"
            );
        }
        let log = std::fs::read_to_string(home.path().join("state/icarus/actions.log")).unwrap();
        assert_eq!(
            log.matches("write refused: the session is read-only")
                .count(),
            WRITES.len(),
            "{how}: {log}"
        );
    }
}

/// With the floor on, every reading command works on the simulated vehicle as it does without it, and `obd info` says
/// the session is read-only.
#[test]
fn every_read_still_works_with_the_floor_on() {
    let home = Home::new("reads");
    let report = home.path().join("report.md").display().to_string();
    let reads: [&[&str]; 14] = [
        &["obd", "info"],
        &["obd", "codes"],
        &["obd", "pids"],
        &["obd", "live", "0C", "0D", "--count", "2", "--every", "100"],
        &["obd", "scan"],
        &["obd", "module", "7E0"],
        &["obd", "asbuilt", "7C0"],
        &["obd", "selftests"],
        &["obd", "charging", "--seconds", "1"],
        &["obd", "tests"],
        &["obd", "services"],
        &["obd", "settings"],
        &["obd", "data"],
        &["obd", "report", "--out", &report],
    ];
    for r in reads {
        let args: Vec<&str> = r
            .iter()
            .copied()
            .chain(["--adapter", "sim", "--read-only"])
            .collect();
        let out = home.icarus(&args, None);
        assert!(
            out.status.success(),
            "{args:?}: {}{}",
            text(&out.stdout),
            text(&out.stderr)
        );
    }
    assert!(Path::new(&report).exists());
    let info = home.icarus(&["obd", "info", "--adapter", "sim"], Some("yes"));
    assert!(text(&info.stdout).contains("this session is read-only"));
    let plain = home.icarus(&["obd", "info", "--adapter", "sim"], None);
    assert!(plain.status.success() && !text(&plain.stdout).contains("read-only"));
}

/// Without the floor (no flag, the variable unset or 0), a write command gets past it as before: a routine asked for
/// without --yes connects, finds the routine and sends nothing, where the floor refuses it.
#[test]
fn without_the_floor_commands_behave_as_before() {
    let home = Home::new("off");
    let ask = [
        "obd",
        "service",
        "Oil life reset (simulated)",
        "--adapter",
        "sim:off",
    ];
    for env in [None, Some("0"), Some("off")] {
        let out = home.icarus(&ask, env);
        assert!(out.status.success(), "{env:?}: {}", text(&out.stderr));
        assert!(text(&out.stdout).contains("Nothing sent"), "{env:?}");
    }
    let refused = home.icarus(&ask, Some("1"));
    assert!(!refused.status.success() && text(&refused.stderr).contains("session read-only"));
}
