//! The vehicles Icarus has met, by VIN, with the name the person gave them ("2017 Toyota Tacoma 3.5L V6"): a VIN gives the
//! model year and the maker, not the model, and searches and parts links need the model. Kept in
//! `~/.config/icarus/garage.toml` on this computer only (a flat VIN = "year make model engine" map, as older installs have it).
//!
//! What the person adds beyond that (a name of their own, the sub-model, drivetrain, transmission, the miles) is a vehicle's
//! `Details`, kept beside it in `~/.config/icarus/details.toml`: the Garage page lists a vehicle as
//! "2017 Toyota Tacoma V6 - Old Blue" (`label`).

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

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

    /// The engine's size from the rest ("3.5L" of "3.5L V6"), if it gives one.
    pub fn engine_size(&self) -> String {
        self.rest
            .split_whitespace()
            .find(|w| is_size(w))
            .unwrap_or("")
            .to_string()
    }

    /// The engine without its size ("V6" of "3.5L V6").
    pub fn engine(&self) -> String {
        self.rest
            .split_whitespace()
            .filter(|w| !is_size(w))
            .collect::<Vec<_>>()
            .join(" ")
    }

    /// From its parts as the Garage page edits them: the size before the engine, as a description reads ("3.5L V6").
    pub fn from_parts(
        year: Option<u16>,
        make: &str,
        model: &str,
        engine_size: &str,
        engine: &str,
    ) -> Described {
        let rest = [engine_size.trim(), engine.trim()]
            .into_iter()
            .filter(|p| !p.is_empty())
            .collect::<Vec<_>>()
            .join(" ");
        Described {
            year,
            make: make.trim().to_string(),
            model: model.trim().to_string(),
            rest,
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

/// An engine size as written: "3.5L", "2.7l", "5.0L".
fn is_size(w: &str) -> bool {
    let Some(n) = w.strip_suffix('L').or_else(|| w.strip_suffix('l')) else {
        return false;
    };
    !n.is_empty() && n.parse::<f64>().is_ok_and(|v| v > 0.0 && v < 20.0)
}

/// What the person tells about a vehicle beyond its description: a name of their own and what a VIN does not give.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Details {
    /// a name of their own ("Old Blue")
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub nickname: String,
    /// the trim ("TRD Off-Road")
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub sub_model: String,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub drivetrain: String,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub transmission: String,
    /// the odometer as last written down
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub miles: Option<u32>,
}

/// How the Garage page lists a vehicle: "2017 Toyota Tacoma V6 - Old Blue" (year, make, model and engine, then its own name);
/// the parts it has, else its own name, else the VIN.
pub fn label(vin: &str, described: Option<&Described>, details: &Details) -> String {
    let car = described
        .map(|d| {
            let engine = if d.engine().is_empty() {
                d.engine_size()
            } else {
                d.engine()
            };
            [
                d.year.map(|y| y.to_string()).unwrap_or_default(),
                d.make.clone(),
                d.model.clone(),
                engine,
            ]
            .into_iter()
            .filter(|p| !p.is_empty())
            .collect::<Vec<_>>()
            .join(" ")
        })
        .unwrap_or_default();
    let own = details.nickname.trim();
    match (car.is_empty(), own.is_empty()) {
        (false, false) => format!("{car} - {own}"),
        (false, true) => car,
        (true, false) => own.to_string(),
        (true, true) => vin.to_string(),
    }
}

pub fn details_path() -> Option<PathBuf> {
    crate::vehicles::user_dir().and_then(|d| d.parent().map(|p| p.join("details.toml")))
}

fn details_all(file: &Path) -> BTreeMap<String, Details> {
    std::fs::read_to_string(file)
        .ok()
        .and_then(|t| toml::from_str(&t).ok())
        .unwrap_or_default()
}

/// A vehicle's details in `file` (none saved: empty).
pub fn details_in(file: &Path, vin: &str) -> Details {
    details_all(file)
        .remove(&vin.trim().to_uppercase())
        .unwrap_or_default()
}

/// A vehicle's details (none saved: empty).
pub fn details(vin: &str) -> Details {
    details_path()
        .map(|p| details_in(&p, vin))
        .unwrap_or_default()
}

/// Saves a vehicle's details in `file` (all empty: forgotten), the text fields trimmed.
pub fn set_details_in(file: &Path, vin: &str, d: &Details) -> Result<(), String> {
    let tidy = |s: &str| s.split_whitespace().collect::<Vec<_>>().join(" ");
    let d = Details {
        nickname: tidy(&d.nickname),
        sub_model: tidy(&d.sub_model),
        drivetrain: tidy(&d.drivetrain),
        transmission: tidy(&d.transmission),
        miles: d.miles,
    };
    let mut all = details_all(file);
    if d == Details::default() {
        all.remove(&vin.trim().to_uppercase());
    } else {
        all.insert(vin.trim().to_uppercase(), d);
    }
    let body = toml::to_string(&all).map_err(|e| e.to_string())?;
    if let Some(dir) = file.parent() {
        std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    }
    let tmp = file.with_extension("tmp");
    std::fs::write(
        &tmp,
        format!("# Icarus: what you told it about each vehicle (its own name, sub-model, drivetrain, transmission, miles), by VIN. On this computer only.\n{body}"),
    )
    .and_then(|_| std::fs::rename(&tmp, file))
    .map_err(|e| format!("{}: {e}", file.display()))
}

pub fn set_details(vin: &str, d: &Details) -> Result<(), String> {
    set_details_in(&details_path().ok_or("no config folder")?, vin, d)
}

/// The vehicle library: `~/Documents/Icarus/Vehicles`, a folder per vehicle.
pub fn library() -> PathBuf {
    std::env::var_os("HOME")
        .map(PathBuf::from)
        .unwrap_or_default()
        .join("Documents/Icarus/Vehicles")
}

/// The folders inside a vehicle's folder.
pub const FOLDERS: [&str; 5] = [
    "OEM tunes",
    "Custom tunes",
    "Definitions",
    "Reports",
    "Logs",
];
const ABOUT: &str = "vehicle.toml";

/// A name as a folder name: no slashes or control characters, at most 80 characters.
fn folder_name(s: &str) -> String {
    let t: String = s
        .chars()
        .map(|c| {
            if c == '/' || c == '\\' || c.is_control() {
                '-'
            } else {
                c
            }
        })
        .collect();
    let t = t.trim().trim_matches('.').to_string();
    t.chars().take(80).collect()
}

fn vin_in(dir: &Path) -> Option<String> {
    let t = std::fs::read_to_string(dir.join(ABOUT)).ok()?;
    let v: toml::Table = toml::from_str(&t).ok()?;
    v.get("vin").and_then(|x| x.as_str()).map(str::to_string)
}

/// Where a vehicle's folder is (or would be) in `lib`: its name, else its VIN; a name another vehicle's folder already
/// has gets the end of the VIN added.
pub fn folder_in(lib: &Path, vin: &str, name: Option<&str>) -> PathBuf {
    let base = name
        .map(folder_name)
        .filter(|n| !n.is_empty())
        .unwrap_or_else(|| vin.to_string());
    let p = lib.join(&base);
    match vin_in(&p) {
        Some(v) if v != vin => {
            lib.join(format!("{base} ({})", &vin[vin.len().saturating_sub(6)..]))
        }
        _ => p,
    }
}

/// Makes a vehicle's folder with its subfolders and its `vehicle.toml` (VIN and name).
pub fn ensure_in(lib: &Path, vin: &str, name: Option<&str>) -> std::io::Result<PathBuf> {
    let dir = folder_in(lib, vin, name);
    for f in FOLDERS {
        std::fs::create_dir_all(dir.join(f))?;
    }
    let mut about = toml::Table::new();
    about.insert("vin".into(), vin.into());
    if let Some(n) = name {
        about.insert("name".into(), n.into());
    }
    std::fs::write(
        dir.join(ABOUT),
        format!(
            "# Icarus: this vehicle's folder.\n{}",
            toml::to_string(&about).unwrap_or_default()
        ),
    )?;
    Ok(dir)
}

/// Where a vehicle's own records go (setting changes, learning sessions): its folder's `Logs`, made if needed. The
/// simulated vehicle, or one whose VIN could not be read, uses `~/.local/state/icarus/sim` instead.
pub fn logs_dir(vin: Option<&str>) -> Result<PathBuf, String> {
    let dir = match vin {
        Some(v) if v != crate::sim::VIN => ensure(v)
            .map_err(|x| format!("the vehicle's folder could not be made: {x}"))?
            .join("Logs"),
        _ => crate::audit::path()
            .and_then(|p| p.parent().map(|d| d.join("sim")))
            .ok_or("no state folder")?,
    };
    std::fs::create_dir_all(&dir).map_err(|x| format!("{}: {x}", dir.display()))?;
    Ok(dir)
}

/// The name of this vehicle's folder in `lib` when the vehicle has a name of its own (`name`), for a recording's
/// `# vehicle:` line: "2017 Toyota Tacoma 3.5L V6". None for an unnamed vehicle, whose folder is its VIN (a recording does
/// not carry the VIN), and for the simulated vehicle.
pub fn folder_label_in(lib: &Path, vin: &str, name: Option<&str>) -> Option<String> {
    if vin == crate::sim::VIN {
        return None;
    }
    name.map(folder_name).filter(|n| !n.is_empty())?;
    folder_in(lib, vin, name)
        .file_name()
        .map(|f| f.to_string_lossy().into_owned())
}

/// `folder_label_in` for the garage on this computer.
pub fn folder_label(vin: &str) -> Option<String> {
    if vin == crate::sim::VIN {
        return None;
    }
    folder_label_in(&library(), vin, load().get(vin).map(String::as_str))
}

/// This vehicle's folder, made if needed (the simulated vehicle has none).
pub fn ensure(vin: &str) -> std::io::Result<PathBuf> {
    if vin == crate::sim::VIN {
        return Err(std::io::Error::other("the simulated vehicle has no folder"));
    }
    let names = load();
    ensure_in(&library(), vin, names.get(vin).map(String::as_str))
}

/// The vehicles in `lib`: (VIN, name, folder).
pub fn vehicles_in(lib: &Path) -> Vec<(String, Option<String>, PathBuf)> {
    let mut out: Vec<(String, Option<String>, PathBuf)> = std::fs::read_dir(lib)
        .into_iter()
        .flatten()
        .flatten()
        .map(|e| e.path())
        .filter_map(|d| {
            let t = std::fs::read_to_string(d.join(ABOUT)).ok()?;
            let v: toml::Table = toml::from_str(&t).ok()?;
            Some((
                v.get("vin")?.as_str()?.to_string(),
                v.get("name").and_then(|n| n.as_str()).map(str::to_string),
                d,
            ))
        })
        .collect();
    out.sort_by(|a, b| {
        a.1.clone()
            .unwrap_or(a.0.clone())
            .cmp(&b.1.clone().unwrap_or(b.0.clone()))
    });
    out
}

pub fn vehicles() -> Vec<(String, Option<String>, PathBuf)> {
    vehicles_in(&library())
}

/// The files in one of a vehicle's folders: (name, bytes), sorted by name.
pub fn files(dir: &Path, sub: &str) -> Vec<(String, u64)> {
    let mut v: Vec<(String, u64)> = std::fs::read_dir(dir.join(sub))
        .into_iter()
        .flatten()
        .flatten()
        .filter(|e| e.file_type().is_ok_and(|t| t.is_file()))
        .map(|e| {
            (
                e.file_name().to_string_lossy().to_string(),
                e.metadata().map(|m| m.len()).unwrap_or(0),
            )
        })
        .collect();
    v.sort();
    v
}

/// Copies a tune into the vehicle's folder: into "OEM tunes" read-only (a stock image must never be changed or replaced by
/// accident: it is the way back), or into "Custom tunes". Never over an existing file: a second copy gets " (2)".
pub fn import(dir: &Path, src: &Path, oem: bool) -> std::io::Result<PathBuf> {
    use std::os::unix::fs::PermissionsExt;
    let sub = dir.join(if oem { "OEM tunes" } else { "Custom tunes" });
    std::fs::create_dir_all(&sub)?;
    let name = src
        .file_name()
        .ok_or_else(|| std::io::Error::new(std::io::ErrorKind::InvalidInput, "no file name"))?
        .to_string_lossy()
        .to_string();
    let (stem, ext) = match name.rsplit_once('.') {
        Some((a, b)) => (a.to_string(), format!(".{b}")),
        None => (name.clone(), String::new()),
    };
    let target = std::iter::once(sub.join(&name))
        .chain((2..).map(|n| sub.join(format!("{stem} ({n}){ext}"))))
        .find(|p| !p.exists())
        .unwrap();
    std::fs::copy(src, &target)?;
    let mode = if oem { 0o444 } else { 0o644 };
    std::fs::set_permissions(&target, std::fs::Permissions::from_mode(mode))?;
    Ok(target)
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
    let vin_key = vin.trim().to_uppercase();
    let old_name = load().get(&vin_key).cloned();
    let body = toml::to_string(&all).map_err(|e| e.to_string())?;
    std::fs::create_dir_all(p.parent().unwrap()).map_err(|e| e.to_string())?;
    let tmp = p.with_extension("tmp");
    std::fs::write(&tmp, format!("# Icarus's names for the vehicles it has met (VIN = \"year make model engine\"); on this computer only.\n{body}"))
        .and_then(|_| std::fs::rename(&tmp, &p))
        .map_err(|e| format!("{}: {e}", p.display()))?;
    // the vehicle's folder follows its name
    let lib = library();
    let from = folder_in(&lib, &vin_key, old_name.as_deref());
    let new_name = all.get(&vin_key).cloned();
    let to = folder_in(&lib, &vin_key, new_name.as_deref());
    if from != to && from.exists() && !to.exists() {
        std::fs::rename(&from, &to).map_err(|e| format!("{}: {e}", from.display()))?;
    }
    if from.exists() || to.exists() {
        ensure_in(&lib, &vin_key, new_name.as_deref()).map_err(|e| e.to_string())?;
    }
    Ok(())
}

/// A vehicle's tires against stock, for correcting its speed reading (`icarus garage tires`, the Tools page), kept by VIN
/// in `~/.config/icarus/tires.toml` on this computer only.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TireSetting {
    pub stock: String,
    pub installed: String,
    /// The vehicle measures speed at the transmission output (a regear counts), not at the wheels.
    #[serde(default)]
    pub at_transmission: bool,
    #[serde(default)]
    pub stock_ratio: Option<f64>,
    #[serde(default)]
    pub installed_ratio: Option<f64>,
}

impl TireSetting {
    /// What the vehicle's speed reading is multiplied by for the real speed.
    pub fn factor(&self) -> Result<f64, String> {
        let (a, b) = (
            icarus_core::tires::Tire::parse(&self.stock)?,
            icarus_core::tires::Tire::parse(&self.installed)?,
        );
        Ok(icarus_core::tires::speed_factor(
            a,
            b,
            self.at_transmission,
            self.stock_ratio,
            self.installed_ratio,
        ))
    }
}

pub fn tires_path() -> Option<PathBuf> {
    crate::vehicles::user_dir().and_then(|d| d.parent().map(|p| p.join("tires.toml")))
}

fn tires_in(file: &Path) -> BTreeMap<String, TireSetting> {
    std::fs::read_to_string(file)
        .ok()
        .and_then(|t| toml::from_str(&t).ok())
        .unwrap_or_default()
}

/// The tire setting of the vehicle with this VIN, if one was saved.
pub fn tires(vin: &str) -> Option<TireSetting> {
    tires_path().and_then(|p| tires_in(&p).remove(&vin.trim().to_uppercase()))
}

/// Saves (Some) or forgets (None) a vehicle's tire setting in `file`; the sizes must parse.
pub fn set_tires_in(file: &Path, vin: &str, t: Option<TireSetting>) -> Result<(), String> {
    let mut all = tires_in(file);
    match t {
        Some(t) => {
            t.factor()?;
            all.insert(vin.trim().to_uppercase(), t);
        }
        None => {
            all.remove(&vin.trim().to_uppercase());
        }
    }
    let body = toml::to_string(&all).map_err(|e| e.to_string())?;
    if let Some(d) = file.parent() {
        std::fs::create_dir_all(d).map_err(|e| format!("{}: {e}", d.display()))?;
    }
    let tmp = file.with_extension("tmp");
    std::fs::write(
        &tmp,
        format!("# Icarus: each vehicle's tires against stock, by VIN, for its corrected speed. On this computer only.\n{body}"),
    )
    .and_then(|_| std::fs::rename(&tmp, file))
    .map_err(|e| format!("{}: {e}", file.display()))
}

pub fn set_tires(vin: &str, t: Option<TireSetting>) -> Result<(), String> {
    set_tires_in(&tires_path().ok_or("no config folder")?, vin, t)
}

#[cfg(test)]
mod tests {
    #[test]
    fn a_vehicle_is_listed_with_its_own_name_and_its_details_are_kept() {
        use super::*;
        let d = Described::parse("2017 Toyota Tacoma 3.5L V6");
        assert_eq!(
            (d.engine_size().as_str(), d.engine().as_str()),
            ("3.5L", "V6")
        );
        assert_eq!(
            Described::from_parts(d.year, &d.make, &d.model, "3.5L", "V6").name(),
            "2017 Toyota Tacoma 3.5L V6"
        );
        assert_eq!(
            Described::parse("2015 Ford F-150 2.7l EcoBoost").engine_size(),
            "2.7l"
        );
        assert_eq!(Described::parse("2010 Honda Civic").engine(), "");
        let mine = Details {
            nickname: "Old Blue".into(),
            ..Details::default()
        };
        assert_eq!(
            label("VIN1", Some(&d), &mine),
            "2017 Toyota Tacoma V6 - Old Blue"
        );
        assert_eq!(
            label("VIN1", Some(&d), &Details::default()),
            "2017 Toyota Tacoma V6"
        );
        assert_eq!(
            label(
                "VIN1",
                Some(&Described::parse("2017 Toyota Tacoma 3.5L")),
                &mine
            ),
            "2017 Toyota Tacoma 3.5L - Old Blue"
        );
        assert_eq!(label("VIN1", None, &mine), "Old Blue");
        assert_eq!(label("VIN1", None, &Details::default()), "VIN1");

        let dir = std::env::temp_dir().join(format!("icarus-details-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let f = dir.join("details.toml");
        assert_eq!(
            details_in(&f, "vin1"),
            Details::default(),
            "nothing saved yet"
        );
        let all = Details {
            nickname: "  Old Blue ".into(),
            sub_model: "TRD  Off-Road".into(),
            drivetrain: "4WD".into(),
            transmission: "6-speed automatic".into(),
            miles: Some(84_200),
        };
        set_details_in(&f, "vin1", &all).unwrap();
        let back = details_in(&f, "VIN1");
        assert_eq!(
            (back.nickname.as_str(), back.sub_model.as_str(), back.miles),
            ("Old Blue", "TRD Off-Road", Some(84_200))
        );
        // another vehicle beside it, then this one emptied: forgotten, the other kept
        set_details_in(
            &f,
            "VIN2",
            &Details {
                nickname: "Grandma's".into(),
                ..Details::default()
            },
        )
        .unwrap();
        set_details_in(&f, "VIN1", &Details::default()).unwrap();
        assert_eq!(details_in(&f, "VIN1"), Details::default());
        assert_eq!(details_in(&f, "VIN2").nickname, "Grandma's");
        assert!(!std::fs::read_to_string(&f).unwrap().contains("VIN1"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn tire_settings_by_vin() {
        let dir = std::env::temp_dir().join(format!("icarus-tires-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let f = dir.join("tires.toml");
        let t = TireSetting {
            stock: "P265/65R17".into(),
            installed: "LT285/70R17".into(),
            at_transmission: false,
            stock_ratio: Some(3.91),
            installed_ratio: Some(4.88),
        };
        set_tires_in(&f, "1abcdefgh12345678", Some(t.clone())).unwrap();
        assert_eq!(tires_in(&f)["1ABCDEFGH12345678"], t);
        assert!(
            (t.factor().unwrap() - 1.0703).abs() < 0.001,
            "{}",
            t.factor().unwrap()
        );
        let bad = TireSetting {
            installed: "huge".into(),
            ..t
        };
        assert!(
            set_tires_in(&f, "1ABCDEFGH12345678", Some(bad)).is_err(),
            "a size that does not parse is refused"
        );
        set_tires_in(&f, "1ABCDEFGH12345678", None).unwrap();
        assert!(tires_in(&f).is_empty());
        let _ = std::fs::remove_dir_all(&dir);
    }

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

    #[test]
    fn vehicle_folders() {
        use std::os::unix::fs::PermissionsExt;
        let lib = std::env::temp_dir().join(format!("icarus-garage-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&lib);
        let a = ensure_in(
            &lib,
            "3TMCZ5AN0HM000000",
            Some("2017 Toyota Tacoma 3.5L V6"),
        )
        .unwrap();
        assert!(
            a.ends_with("2017 Toyota Tacoma 3.5L V6")
                && a.join("OEM tunes").is_dir()
                && a.join("Reports").is_dir()
        );
        // a second vehicle with the same name gets the end of its VIN
        let b = ensure_in(
            &lib,
            "3TMCZ5AN0HM999999",
            Some("2017 Toyota Tacoma 3.5L V6"),
        )
        .unwrap();
        assert!(
            b.ends_with("2017 Toyota Tacoma 3.5L V6 (999999)"),
            "{}",
            b.display()
        );
        assert_eq!(
            ensure_in(
                &lib,
                "3TMCZ5AN0HM000000",
                Some("2017 Toyota Tacoma 3.5L V6")
            )
            .unwrap(),
            a,
            "the same folder again"
        );
        let unnamed = ensure_in(&lib, "JF1VA2E60K9800000", None).unwrap();
        assert!(unnamed.ends_with("JF1VA2E60K9800000"));
        // a recording's vehicle line: the folder's name, never a bare VIN
        let label = |vin: &str, name: Option<&str>| folder_label_in(&lib, vin, name);
        assert_eq!(
            label("3TMCZ5AN0HM999999", Some("2017 Toyota Tacoma 3.5L V6")).as_deref(),
            Some("2017 Toyota Tacoma 3.5L V6 (999999)")
        );
        assert_eq!(label("JF1VA2E60K9800000", None), None);
        assert_eq!(label(crate::sim::VIN, Some("Sim")), None);
        assert_eq!(folder_name("a/b\\c."), "a-b-c");
        let v = vehicles_in(&lib);
        assert_eq!(v.len(), 3);
        // a stock image goes in read-only and is never replaced
        let src = lib.join("stock.bin");
        std::fs::write(&src, [1u8, 2, 3]).unwrap();
        let o1 = import(&a, &src, true).unwrap();
        let o2 = import(&a, &src, true).unwrap();
        assert!(o1.ends_with("OEM tunes/stock.bin") && o2.ends_with("OEM tunes/stock (2).bin"));
        assert_eq!(
            std::fs::metadata(&o1).unwrap().permissions().mode() & 0o777,
            0o444
        );
        let c = import(&a, &src, false).unwrap();
        assert!(c.ends_with("Custom tunes/stock.bin"));
        assert_eq!(files(&a, "OEM tunes").len(), 2);
        for f in [&o1, &o2] {
            let mut p = std::fs::metadata(f).unwrap().permissions();
            p.set_mode(0o644);
            std::fs::set_permissions(f, p).unwrap();
        }
        let _ = std::fs::remove_dir_all(&lib);
    }
}
