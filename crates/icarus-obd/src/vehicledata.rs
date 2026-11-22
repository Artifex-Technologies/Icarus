//! A vehicle's module data read once, for the window and `icarus mcp`: the set chosen as `icarus obd data` chooses it
//! (`choose`: the one remembered for this vehicle, else the only one that fits its make, model year and model; never a
//! guess), each module's values read through moddata's gates (services 21 and 22 only, paced, airbag modules identity
//! only), and what each request did kept in the vehicle's state, as the command line keeps it. Read-only.

use crate::link::Link;
use crate::moddata::{self, Loaded, Outcome, Pacer, Scope, State};
use crate::vehicles::{self, Profile};
use crate::{garage, j1979, report, vin};

/// The set for this vehicle: `wanted` (`--set`), else the one its state names, else the only one that fits. Nothing is
/// guessed: with the make, model year or model unknown, a set that names one does not fit (it is named with --set), and
/// the simulated vehicle's set is refused on any other vehicle however it is named.
pub fn choose<'a>(
    sets: &'a [Loaded],
    wanted: Option<&str>,
    state: &State,
    make: Option<&str>,
    year: Option<u16>,
    model: Option<&str>,
    vin: Option<&str>,
) -> Result<&'a Loaded, String> {
    if let Some(n) = wanted.or(state.set.as_deref()) {
        if let Some(why) = moddata::set_refused(n, vin) {
            return Err(why);
        }
        return sets
            .iter()
            .find(|s| s.set.name == n)
            .ok_or_else(|| format!("no module-data set named {n:?} (icarus data lists them)"));
    }
    let fit = moddata::sets_for(sets, make, year, model, vin);
    match fit.as_slice() {
        [one] => Ok(one),
        [] => {
            let mut unknown = Vec::new();
            if make.is_none() {
                unknown.push("make (--make NAME)");
            }
            if year.is_none() {
                unknown.push("model year (--year YYYY)");
            }
            if model.is_none() {
                unknown.push("model (name the vehicle: icarus garage VIN \"2017 Toyota Tacoma\")");
            }
            Err(format!(
                "no module-data set fits this vehicle ({} {}{}){}: import one with icarus import obdb FILE.json (the \
                 vehicle's signal set from github.com/OBDb, downloaded), or name one with --set",
                year.map(|y| y.to_string()).unwrap_or_default(),
                make.unwrap_or("unknown make"),
                model.map(|m| format!(" {m}")).unwrap_or_default(),
                if unknown.is_empty() {
                    String::new()
                } else {
                    format!(
                        "; its {} is not known, and a set is never guessed",
                        unknown.join(", ")
                    )
                }
            ))
        }
        many => Err(format!(
            "several sets fit ({}): choose one with --set NAME (it is remembered for this vehicle)",
            many.iter()
                .map(|s| s.set.name.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        )),
    }
}

/// The chosen set as used on this vehicle (`Loaded::on_vehicle`: the gates decided by the vehicle's make too), with a note
/// when the set is for another make or the vehicle's make is not known.
pub fn on_vehicle(set: &Loaded, all: &[Profile], make: Option<&str>) -> (Loaded, Option<String>) {
    let note = match make {
        Some(m) if !m.eq_ignore_ascii_case(&set.set.make) => Some(format!(
            "the set {:?} is for {:?}, this vehicle is {m:?}: the stricter of the two makes' airbag gates holds",
            set.set.name, set.set.make
        )),
        None if !set.set.make.is_empty() => Some(format!(
            "this vehicle's make is not known: a module of unknown kind in the set {:?} gets no data request, since it \
             could be the airbag module",
            set.set.name
        )),
        _ => None,
    };
    (set.on_vehicle(all, make), note)
}

/// One module's values as read.
#[derive(Clone, Debug, PartialEq)]
pub struct ModuleValues {
    /// "701", "750.2A".
    pub label: String,
    pub name: String,
    /// (value name, its reading with its unit, checked on this vehicle).
    pub values: Vec<(String, String, bool)>,
    /// What else happened: a request refused or unanswered, the module not reachable yet.
    pub notes: Vec<String>,
}

/// What one reading of the vehicle's module data gave.
#[derive(Clone, Debug, PartialEq)]
pub struct VehicleValues {
    pub set: String,
    /// The set's source and its attribution, shown wherever its values are.
    pub attribution: String,
    pub notes: Vec<String>,
    pub modules: Vec<ModuleValues>,
}

impl VehicleValues {
    /// As text (icarus mcp, plain reports).
    pub fn text(&self) -> String {
        let mut o = format!("Module data from the set {}.\n", self.set);
        for n in &self.notes {
            o += &format!("Note: {n}\n");
        }
        for m in &self.modules {
            o += &format!("\n{}  {}\n", m.label, m.name);
            for (name, v, checked) in &m.values {
                o += &format!(
                    "  {name}: {v}{}\n",
                    if *checked {
                        " (checked on this vehicle)"
                    } else {
                        ""
                    }
                );
            }
            for n in &m.notes {
                o += &format!("  {n}\n");
            }
        }
        if !self.attribution.is_empty() {
            o += &format!("\n{}\n", self.attribution);
        }
        o
    }
}

/// Reads the vehicle's module data once: every module of the chosen set with data requests, or only the one whose label
/// or request ID is `module` ("701", "750.2A"). Values in US units when `us`.
pub fn read(
    link: &mut dyn Link,
    profiles: &[Profile],
    module: Option<&str>,
    us: bool,
) -> Result<VehicleValues, String> {
    let vin_s = j1979::vin(link).map_err(|e| e.to_string())?;
    let vin = vin_s.as_deref();
    let year = vin.and_then(|v| vin::decode(v).ok()).and_then(|d| d.year);
    let make = vin
        .and_then(|v| vehicles::for_vin(profiles, v))
        .map(|p| p.id.clone());
    let model = vin
        .and_then(garage::get)
        .map(|d| d.model)
        .filter(|m| !m.is_empty());
    let state_path = moddata::state_path(vin)?;
    let mut state = match &state_path {
        Some(p) => State::load(p)?,
        None => State::default(),
    };
    let mut notes = Vec::new();
    if state_path.is_none() {
        notes.push(
            "the VIN could not be read, so what each request did is kept for this reading only"
                .to_string(),
        );
    }
    let (sets, problems) = moddata::load_all(moddata::user_dir().as_deref(), profiles);
    notes.extend(
        problems
            .into_iter()
            .map(|p| format!("a set was left out: {p}")),
    );
    let chosen = choose(
        &sets,
        None,
        &state,
        make.as_deref(),
        year,
        model.as_deref(),
        vin,
    )?;
    let (set, note) = on_vehicle(chosen, profiles, make.as_deref());
    notes.extend(note);
    let new_set = state.set.as_deref() != Some(set.set.name.as_str());
    state.set = Some(set.set.name.clone());
    let scope = Scope {
        year,
        all_years: false,
        refused_before: state.refused_keys(),
        retry: false,
    };
    let today = report::now()[..10].to_string();
    let wanted = |m: &icarus_core::moddata::ModuleDef| match module {
        None => !m.requests.is_empty(),
        Some(want) => {
            m.module.label().eq_ignore_ascii_case(want)
                || u32::from_str_radix(want.trim_start_matches("0x"), 16).ok() == Some(m.module.tx)
        }
    };
    let mut pacer = Pacer::real();
    let mut changed = new_set;
    let mut modules = Vec::new();
    for m in set.modules.iter().filter(|m| wanted(m)) {
        let mut mv = ModuleValues {
            label: m.module.label(),
            name: m.module.name.clone(),
            values: Vec::new(),
            notes: Vec::new(),
        };
        if let Err(why) = moddata::reachable(m) {
            mv.notes.push(why.to_string());
            modules.push(mv);
            continue;
        }
        if m.requests.is_empty() && set.gates.is_airbag(&m.module) {
            mv.notes.push("airbag module: identity only".into());
        }
        for (r, o) in moddata::read_module(link, &mut pacer, &set.gates, m, &scope, None) {
            changed |= state.record(moddata::key(m, r), &o, &today);
            match o {
                Outcome::Values(vs) => {
                    for v in vs {
                        let checked = state.checked(m.module.tx, &v.id).is_some();
                        mv.values.push((v.name.clone(), v.text(us), checked));
                    }
                }
                Outcome::NotDue => {}
                other => mv
                    .notes
                    .push(format!("{}: {}", r.label(), other.describe())),
            }
        }
        modules.push(mv);
    }
    if module.is_some() && modules.is_empty() {
        return Err(format!(
            "the set {} has no module {}",
            set.set.name,
            module.unwrap_or("")
        ));
    }
    if changed {
        if let Some(p) = &state_path {
            state.save(p)?;
        }
    }
    Ok(VehicleValues {
        set: set.set.name.clone(),
        attribution: set.set.source.attribution.clone(),
        notes,
        modules,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elm::Elm;
    use crate::sim::SimCar;

    #[test]
    fn the_simulated_vehicle_reads_its_own_set() {
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        let profiles = vehicles::builtin();
        let all = read(&mut elm, &profiles, None, false).unwrap();
        assert_eq!(all.set, "sim");
        let labels: Vec<&str> = all.modules.iter().map(|m| m.label.as_str()).collect();
        assert!(
            labels.contains(&"700") && labels.contains(&"701"),
            "{labels:?}"
        );
        assert!(all.modules.iter().any(|m| !m.values.is_empty()), "{all:?}");
        let one = read(&mut elm, &profiles, Some("701"), false).unwrap();
        assert_eq!(one.modules.len(), 1);
        assert!(one.text().contains("701"));
        assert!(read(&mut elm, &profiles, Some("7FF"), false).is_err());
    }
}
