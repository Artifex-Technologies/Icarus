//! The record of everything Icarus did that changed something on a vehicle (actuator tests, cleared codes):
//! `~/.local/state/icarus/actions.log`, one line each, with the time. Tests never write to it.

pub fn path() -> Option<std::path::PathBuf> {
    std::env::var_os("XDG_STATE_HOME")
        .map(std::path::PathBuf::from)
        .filter(|p| p.is_absolute())
        .or_else(|| {
            std::env::var_os("HOME").map(|h| std::path::PathBuf::from(h).join(".local/state"))
        })
        .map(|d| d.join("icarus/actions.log"))
}

pub fn log(line: &str) {
    if cfg!(test) {
        return;
    }
    let Some(p) = path() else { return };
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
