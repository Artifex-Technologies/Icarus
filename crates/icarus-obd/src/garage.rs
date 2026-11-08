//! The vehicles Icarus has met, by VIN, with the name the person gave them ("2017 Toyota Tacoma 3.5L V6"): a VIN gives the
//! model year and the maker, not the model, and searches and parts links need the model. Kept in
//! `~/.config/icarus/garage.toml` on this computer only.

use std::collections::BTreeMap;
use std::path::PathBuf;

/// A vehicle's name split into year, make, model and the rest (engine, trim).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Described {
    pub year: Option<u16>,
    pub make: String,
    pub model: String,
    pub rest: String,
}

impl Described {
    /// "2017 Toyota Tacoma 3.5L V6" (a year first is optional).
    pub fn parse(s: &str) -> Described {
        let mut w = s.split_whitespace().peekable();
        let year = w
            .peek()
            .and_then(|y| y.parse::<u16>().ok())
            .filter(|y| (1950..=2100).contains(y));
        if year.is_some() {
            w.next();
        }
        let make = w.next().unwrap_or("").to_string();
        let model = w.next().unwrap_or("").to_string();
        Described {
            year,
            make,
            model,
            rest: w.collect::<Vec<_>>().join(" "),
        }
    }

    pub fn name(&self) -> String {
        [
            self.year.map(|y| y.to_string()).unwrap_or_default(),
            self.make.clone(),
            self.model.clone(),
            self.rest.clone(),
        ]
        .into_iter()
        .filter(|p| !p.is_empty())
        .collect::<Vec<_>>()
        .join(" ")
    }
}

pub fn path() -> Option<PathBuf> {
    crate::vehicles::user_dir().and_then(|d| d.parent().map(|p| p.join("garage.toml")))
}

/// VIN -> name.
pub fn load() -> BTreeMap<String, String> {
    path()
        .and_then(|p| std::fs::read_to_string(p).ok())
        .and_then(|t| toml::from_str(&t).ok())
        .unwrap_or_default()
}

pub fn get(vin: &str) -> Option<Described> {
    load()
        .get(&vin.trim().to_uppercase())
        .map(|n| Described::parse(n))
}

pub fn set(vin: &str, name: &str) -> Result<(), String> {
    let p = path().ok_or("no config folder")?;
    let mut all = load();
    let name = name.split_whitespace().collect::<Vec<_>>().join(" ");
    if name.is_empty() {
        all.remove(&vin.trim().to_uppercase());
    } else {
        all.insert(vin.trim().to_uppercase(), name);
    }
    let body = toml::to_string(&all).map_err(|e| e.to_string())?;
    std::fs::create_dir_all(p.parent().unwrap()).map_err(|e| e.to_string())?;
    let tmp = p.with_extension("tmp");
    std::fs::write(&tmp, format!("# Icarus's names for the vehicles it has met (VIN = \"year make model engine\"); on this computer only.\n{body}"))
        .and_then(|_| std::fs::rename(&tmp, &p))
        .map_err(|e| format!("{}: {e}", p.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names() {
        let d = Described::parse(" 2017  Toyota Tacoma 3.5L V6 ");
        assert_eq!(
            (d.year, d.make.as_str(), d.model.as_str(), d.rest.as_str()),
            (Some(2017), "Toyota", "Tacoma", "3.5L V6")
        );
        assert_eq!(d.name(), "2017 Toyota Tacoma 3.5L V6");
        let n = Described::parse("Subaru WRX STI");
        assert_eq!(
            (n.year, n.model.as_str(), n.rest.as_str()),
            (None, "WRX", "STI")
        );
        assert_eq!(Described::parse("").name(), "");
    }
}
