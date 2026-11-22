//! The window's levels (ICA-74): **Basic** for everyone (diagnostics, troubleshooting, analysis); **Advanced** shows more and
//! unlocks control of each module, as a dealer tool does (actuator tests, module settings, learning from another tool, the bus
//! view); **Tuning**, only while Advanced is on, unlocks tuning (Maps). Kept per computer in `~/.config/icarus/window.toml`; a
//! fresh one is Basic. Hiding a page is not the only guard: every change to a vehicle keeps its own checks, and a vehicle set to
//! Read-Only in the Garage stays read-only at every level.

use std::path::PathBuf;

/// What a page needs to be shown.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Need {
    Basic,
    Advanced,
    Tuning,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Levels {
    pub advanced: bool,
    /// Only ever on with `advanced`.
    pub tuning: bool,
}

impl Levels {
    /// Whether a page that needs `need` is shown.
    pub fn allows(self, need: Need) -> bool {
        match need {
            Need::Basic => true,
            Need::Advanced => self.advanced,
            Need::Tuning => self.advanced && self.tuning,
        }
    }

    /// Advanced off turns Tuning off with it.
    pub fn set_advanced(&mut self, on: bool) {
        self.advanced = on;
        if !on {
            self.tuning = false;
        }
    }

    /// `advanced = true` and `tuning = true` lines; anything else (a missing file, a hand-edited one) is Basic. Tuning without
    /// Advanced is off.
    pub fn parse(text: &str) -> Levels {
        let mut l = Levels::default();
        for line in text.lines() {
            let line = line.split('#').next().unwrap_or("").trim();
            if let Some((k, v)) = line.split_once('=') {
                let on = v.trim() == "true";
                match k.trim() {
                    "advanced" => l.advanced = on,
                    "tuning" => l.tuning = on,
                    _ => {}
                }
            }
        }
        if !l.advanced {
            l.tuning = false;
        }
        l
    }

    pub fn text(self) -> String {
        format!(
            "# Icarus's window: its levels (Basic unless these say otherwise)\nadvanced = {}\ntuning = {}\n",
            self.advanced,
            self.advanced && self.tuning
        )
    }

    /// `$XDG_CONFIG_HOME/icarus/window.toml`, else `~/.config/icarus/window.toml`.
    pub fn path() -> Option<PathBuf> {
        let base = std::env::var_os("XDG_CONFIG_HOME")
            .filter(|v| !v.is_empty())
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
        Some(base.join("icarus/window.toml"))
    }

    pub fn load() -> Levels {
        Self::path()
            .and_then(|p| std::fs::read_to_string(p).ok())
            .map(|t| Levels::parse(&t))
            .unwrap_or_default()
    }

    pub fn save(self) -> std::io::Result<()> {
        let Some(p) = Self::path() else {
            return Err(std::io::Error::other("no home folder"));
        };
        if let Some(d) = p.parent() {
            std::fs::create_dir_all(d)?;
        }
        std::fs::write(p, self.text())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_fresh_window_is_basic_and_tuning_needs_advanced() {
        let fresh = Levels::parse("");
        assert_eq!(fresh, Levels::default());
        assert!(
            fresh.allows(Need::Basic)
                && !fresh.allows(Need::Advanced)
                && !fresh.allows(Need::Tuning)
        );
        // tuning written without advanced (a hand-edited file) stays off
        assert_eq!(Levels::parse("tuning = true\n"), Levels::default());
        let adv = Levels::parse("advanced = true # more detail\n");
        assert!(adv.allows(Need::Advanced) && !adv.allows(Need::Tuning));
        let all = Levels::parse("advanced = true\ntuning = true\n");
        assert!(all.allows(Need::Tuning));
        // what is written is read back the same
        assert_eq!(Levels::parse(&all.text()), all);
        assert_eq!(Levels::parse(&adv.text()), adv);
        // Advanced off takes Tuning with it
        let mut l = all;
        l.set_advanced(false);
        assert_eq!(l, Levels::default());
        assert!(
            !Levels::parse("advanced = yes\n").advanced,
            "only true turns it on"
        );
    }
}
