//! The record of everything Icarus did that changed something on a vehicle (actuator tests, cleared codes):
//! `~/.local/state/icarus/actions.log`, one line each, with the time. Tests never write to it: this crate's own tests log
//! nothing, and the window's and the command's (the `test-home` feature, ICA-225) log only into a test's own home in the
//! temporary folder (`crate::env`), never the real one.

/// `$XDG_STATE_HOME/icarus/actions.log`, else `~/.local/state/icarus/actions.log` (through `crate::env`, so a test's own
/// home in tests).
pub fn path() -> Option<std::path::PathBuf> {
    crate::env("XDG_STATE_HOME")
        .map(std::path::PathBuf::from)
        .filter(|p| p.is_absolute())
        .or_else(|| crate::env("HOME").map(|h| std::path::PathBuf::from(h).join(".local/state")))
        .map(|d| d.join("icarus/actions.log"))
}

pub fn log(line: &str) {
    if cfg!(test) {
        return;
    }
    let Some(p) = path() else { return };
    // the window's and the command's tests: only ever into a test's own home (`crate::env` gives no other)
    #[cfg(feature = "test-home")]
    if !crate::in_test_area(&p) {
        return;
    }
    if let Some(d) = p.parent() {
        let _ = std::fs::create_dir_all(d);
    }
    use std::io::Write;
    if let Ok(mut f) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&p)
    {
        let _ = writeln!(f, "{} {line}", crate::report::now());
    }
}
