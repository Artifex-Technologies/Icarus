//! The devices the window works with and each one's role (ICA-80, my decision 2026-11-22): **talk** (Icarus asks the vehicle
//! through it) or **listen only** (silent watching: it is never sent a request, so it can sit on a Y-splitter while another
//! device or another program talks). Kept per computer in `~/.config/icarus/devices.toml`, one `listen = "SPEC"` line per
//! listening device; every other device talks.

use std::path::PathBuf;

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Roles {
    /// The adapters set to listen only (as the adapter field names them: `/dev/ttyUSB0`, `bt:…`, `sim:listen`).
    pub listen: Vec<String>,
}

impl Roles {
    /// `listen = "SPEC"` lines (quoted or not); anything else is left out.
    pub fn parse(text: &str) -> Roles {
        let mut r = Roles::default();
        for line in text.lines() {
            let line = line.split('#').next().unwrap_or("").trim();
            if let Some((k, v)) = line.split_once('=') {
                let v = v.trim().trim_matches('"').trim();
                if k.trim() == "listen" && !v.is_empty() && !r.listen.iter().any(|x| x == v) {
                    r.listen.push(v.to_string());
                }
            }
        }
        r
    }

    pub fn text(&self) -> String {
        let mut t = String::from("# Icarus: devices set to listen only (never sent a request); every other device talks\n");
        for l in &self.listen {
            t.push_str(&format!("listen = \"{}\"\n", l.replace('"', "")));
        }
        t
    }

    /// `$XDG_CONFIG_HOME/icarus/devices.toml`, else `~/.config/icarus/devices.toml`.
    pub fn path() -> Option<PathBuf> {
        let base = std::env::var_os("XDG_CONFIG_HOME")
            .filter(|v| !v.is_empty())
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
        Some(base.join("icarus/devices.toml"))
    }

    pub fn load() -> Roles {
        Self::path()
            .and_then(|p| std::fs::read_to_string(p).ok())
            .map(|t| Roles::parse(&t))
            .unwrap_or_default()
    }

    pub fn save(&self) -> std::io::Result<()> {
        let Some(p) = Self::path() else {
            return Err(std::io::Error::other("no home folder"));
        };
        if let Some(d) = p.parent() {
            std::fs::create_dir_all(d)?;
        }
        std::fs::write(p, self.text())
    }

    pub fn is_listen(&self, spec: &str) -> bool {
        let spec = spec.trim();
        self.listen.iter().any(|l| l == spec)
    }

    pub fn set_listen(&mut self, spec: &str, listen: bool) {
        let spec = spec.trim();
        self.listen.retain(|l| l != spec);
        if listen && !spec.is_empty() {
            self.listen.push(spec.to_string());
        }
    }

    /// Whether Icarus may talk through `spec`: never through a device set to listen only.
    pub fn may_talk(&self, spec: &str) -> Result<(), String> {
        if self.is_listen(spec) {
            return Err(format!(
                "{} is set to listen only (Settings, the gear): Icarus sends it no request. Talk through another device, or change its role.",
                spec.trim()
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn listening_devices_are_kept_and_never_talked_through() {
        let r = Roles::parse("# a note\nlisten = \"/dev/ttyUSB0\"\nlisten = sim:listen\nlisten = \"/dev/ttyUSB0\"\nother = x\nlisten = \"\"\n");
        assert_eq!(
            r.listen,
            vec!["/dev/ttyUSB0".to_string(), "sim:listen".to_string()],
            "once each, empty left out"
        );
        assert_eq!(
            Roles::parse(&r.text()),
            r,
            "what is written is read back the same"
        );
        assert!(
            r.may_talk(" /dev/ttyUSB0 ").is_err() && r.may_talk("bt:AA:BB:CC:DD:EE:FF").is_ok()
        );
        let mut m = r.clone();
        m.set_listen("/dev/ttyUSB0", false);
        assert!(m.may_talk("/dev/ttyUSB0").is_ok() && m.is_listen("sim:listen"));
        m.set_listen("bt:AA:BB:CC:DD:EE:FF", true);
        assert!(m.may_talk("bt:AA:BB:CC:DD:EE:FF").is_err());
        assert_eq!(
            Roles::parse(""),
            Roles::default(),
            "a fresh computer: every device talks"
        );
    }
}
