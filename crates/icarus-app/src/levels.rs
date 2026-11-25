//! The window's levels (ICA-74): **Basic** for everyone (diagnostics, troubleshooting, analysis); **Advanced** shows more and
//! unlocks control of each module, as a dealer tool does (actuator tests, module settings, learning from another tool, the bus
//! view); **Tuning**, only while Advanced is on, unlocks tuning (Maps). Kept per computer in `~/.config/icarus/window.toml`; a
//! fresh one is Basic. Hiding a page is not the only guard: every change to a vehicle keeps its own checks, and a vehicle set to
//! Read-Only in the Garage stays read-only at every level.
//!
//! The same file keeps the window's other settings (ICA-79, the gear's page): the units (Imperial, Metric, both side by side,
//! or the system's custom) and the theme (the desktop's, dark or light). Every setting is written each time, so changing
//! one never drops another.

use std::path::PathBuf;

/// What a page needs to be shown.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Need {
    Basic,
    Advanced,
    Tuning,
}

/// Units for speed, temperature, pressure and the rest.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Units {
    /// The system's custom (its locale: miles and °F in the US).
    #[default]
    Auto,
    /// mph, °F, psi.
    Imperial,
    /// km/h, °C, kPa: as vehicles report them.
    Metric,
    /// Both side by side: the system's custom first, the other beside it.
    Both,
}

/// The window's colours.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Theme {
    /// The desktop's light or dark.
    #[default]
    System,
    Dark,
    Light,
}

impl Units {
    pub const ALL: [Units; 4] = [Units::Imperial, Units::Metric, Units::Both, Units::Auto];
    pub fn word(self) -> &'static str {
        match self {
            Units::Auto => "auto",
            Units::Imperial => "imperial",
            Units::Metric => "metric",
            Units::Both => "both",
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Units::Auto => "As the system has it",
            Units::Imperial => "Imperial (mph, °F, psi)",
            Units::Metric => "Metric (km/h, °C, kPa)",
            Units::Both => "Both side by side",
        }
    }
}

impl Theme {
    pub const ALL: [Theme; 3] = [Theme::System, Theme::Dark, Theme::Light];
    pub fn word(self) -> &'static str {
        match self {
            Theme::System => "system",
            Theme::Dark => "dark",
            Theme::Light => "light",
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Theme::System => "As the desktop has it",
            Theme::Dark => "Dark",
            Theme::Light => "Light",
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Levels {
    pub advanced: bool,
    /// Only ever on with `advanced`.
    pub tuning: bool,
    pub units: Units,
    pub theme: Theme,
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
                let v = v.trim().trim_matches('"');
                let on = v == "true";
                match k.trim() {
                    "advanced" => l.advanced = on,
                    "tuning" => l.tuning = on,
                    "units" => {
                        l.units = Units::ALL
                            .into_iter()
                            .find(|u| u.word() == v)
                            .unwrap_or_default()
                    }
                    "theme" => {
                        l.theme = Theme::ALL
                            .into_iter()
                            .find(|t| t.word() == v)
                            .unwrap_or_default()
                    }
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
            "# Icarus's window: its levels (Basic unless these say otherwise), units and theme\nadvanced = {}\ntuning = {}\nunits = \"{}\"\ntheme = \"{}\"\n",
            self.advanced,
            self.advanced && self.tuning,
            self.units.word(),
            self.theme.word()
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

    #[test]
    fn units_and_theme_are_kept_beside_the_levels() {
        let l = Levels::parse("advanced = true\nunits = \"both\"\ntheme = \"light\"\n");
        assert_eq!(
            (l.advanced, l.units, l.theme),
            (true, Units::Both, Theme::Light)
        );
        // what is written is read back the same, every setting at once (changing one never drops another)
        assert_eq!(Levels::parse(&l.text()), l);
        let mut m = l;
        m.set_advanced(false);
        assert_eq!(
            (
                Levels::parse(&m.text()).units,
                Levels::parse(&m.text()).theme
            ),
            (Units::Both, Theme::Light)
        );
        // a file from before these settings, or a word that is not one: the system's custom and the desktop's theme
        assert_eq!(
            Levels::parse("advanced = true\ntuning = true\n").units,
            Units::Auto
        );
        assert_eq!(
            Levels::parse("units = furlongs\ntheme = neon\n"),
            Levels::default()
        );
        assert_eq!(
            Levels::parse("units = metric\n").units,
            Units::Metric,
            "unquoted too"
        );
    }
}
