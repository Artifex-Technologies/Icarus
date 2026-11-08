//! A diagnostic report: everything a code read found, in one Markdown file to keep, print or show a mechanic: the vehicle,
//! the check-engine light and monitors, every code with its meaning, where to start and the parts usually involved (with
//! search and parts links), the freeze frame, the modules and a snapshot of live values. Reports hold the VIN: they are
//! saved on this computer (`~/Documents/Icarus/reports` by default) and never sent anywhere.

use crate::dtc;
use crate::elm::{Elm, ObdError};
use crate::garage::{self, Described};
use crate::j1979::{self, DtcKind, FreezeFrame, Monitors};
use crate::lookup;
use crate::uds::{self, Dialect, ModuleDtc};
use crate::vehicles::{self, ModuleAddr, Profile};
use crate::vin::{self, VinInfo};
use std::fmt::Write;
use std::path::PathBuf;

/// A module that answered: where, its identity values, and its codes.
pub type ModuleFound = (
    ModuleAddr,
    Vec<(&'static str, String)>,
    Option<(Dialect, Vec<ModuleDtc>)>,
);

#[derive(Clone, Default)]
pub struct Diagnosis {
    pub when: String,
    pub vin: Option<String>,
    pub vin_info: Option<VinInfo>,
    pub vehicle: Option<Described>,
    pub profile: Option<Profile>,
    pub adapter: String,
    pub protocol: String,
    pub voltage: Option<f64>,
    pub monitors: Vec<(String, Monitors)>,
    pub fuel_system: Option<&'static str>,
    /// (kind, module, codes) from the standard OBD-II services.
    pub codes: Vec<(&'static str, String, Vec<String>)>,
    pub freeze: Vec<FreezeFrame>,
    pub modules: Vec<ModuleFound>,
    pub snapshot: Vec<(u8, f64)>,
    /// On-board test results (service 06; CAN vehicles).
    pub selftests: Vec<crate::mode06::TestResult>,
}

/// The local date and time, "2026-11-07 19:40".
pub fn now() -> String {
    unsafe {
        let t = libc::time(std::ptr::null_mut());
        let mut tm: libc::tm = std::mem::zeroed();
        libc::localtime_r(&t, &mut tm);
        format!(
            "{:04}-{:02}-{:02} {:02}:{:02}",
            tm.tm_year + 1900,
            tm.tm_mon + 1,
            tm.tm_mday,
            tm.tm_hour,
            tm.tm_min
        )
    }
}

/// Everything the standard services give (and, with `scan`, every module of the profile), read-only.
pub fn collect(elm: &mut Elm, profiles: &[Profile], scan: bool) -> Result<Diagnosis, ObdError> {
    let mut d = Diagnosis {
        when: now(),
        adapter: format!(
            "{}{}",
            elm.id,
            elm.stn
                .as_ref()
                .map(|s| format!(" ({s})"))
                .unwrap_or_default()
        ),
        protocol: elm.protocol.describe().to_string(),
        voltage: elm.voltage,
        ..Default::default()
    };
    d.vin = j1979::vin(elm)?;
    if let Some(v) = &d.vin {
        d.vin_info = vin::decode(v).ok();
        d.profile = vehicles::for_vin(profiles, v).cloned();
        d.vehicle = garage::get(v);
    }
    if let Ok(a) = elm.request(&[0x01, 0x01]) {
        for x in a
            .iter()
            .filter(|x| x.data.len() >= 6 && x.data[..2] == [0x41, 0x01])
        {
            if let Some(m) = j1979::monitors(&x.data[2..]) {
                d.monitors.push((format!("{} ({})", x.module(), x.from), m));
            }
        }
    }
    d.fuel_system = elm
        .request(&[0x01, 0x03])
        .ok()
        .and_then(|a| {
            a.into_iter()
                .find(|x| x.data.len() >= 3 && x.data[..2] == [0x41, 0x03])
        })
        .map(|x| j1979::fuel_system(x.data[2]));
    for (kind, label) in [
        (DtcKind::Stored, "stored"),
        (DtcKind::Pending, "pending"),
        (DtcKind::Permanent, "permanent"),
    ] {
        for (from, module, list) in j1979::dtcs(elm, kind)? {
            if !list.is_empty() {
                d.codes.push((label, format!("{module} ({from})"), list));
            }
        }
    }
    d.freeze = j1979::freeze_frame(elm)?;
    d.selftests = crate::mode06::read(elm).unwrap_or_default();
    for p in [0x05, 0x0C, 0x04, 0x06, 0x07, 0x08, 0x09, 0x0F, 0x42, 0x44] {
        if let Ok(r) = j1979::read(elm, p) {
            if let Some(x) = r.iter().find(|x| x.from == "7E8").or(r.first()) {
                d.snapshot.push((p, x.value));
            }
        }
    }
    if scan {
        let profile = d
            .profile
            .clone()
            .or_else(|| vehicles::find(profiles, "generic").ok().cloned());
        for m in profile
            .as_ref()
            .map(vehicles::candidates)
            .unwrap_or_default()
        {
            if m.unreachable().is_some() || !uds::present(elm, m.tx, m.rx())? {
                continue;
            }
            let id = uds::module_identity(elm, m.tx, m.rx())?;
            let codes = uds::module_dtcs(elm, m.tx, m.rx())?;
            d.modules.push((m, id, codes));
        }
    }
    Ok(d)
}

fn code_block(out: &mut String, code: &str, states: &str, d: &Diagnosis) {
    let info = dtc::explain(code, d.profile.as_ref());
    let text = info.as_ref().ok().and_then(|i| i.text.clone());
    let _ = writeln!(
        out,
        "- **{code}**{} {}{}",
        if states.is_empty() {
            String::new()
        } else {
            format!(" ({states})")
        },
        text.as_deref().unwrap_or(match &info {
            Ok(i) if !i.standard => "the maker's own code: look it up for this make",
            _ => "(Icarus has no words for this code yet)",
        }),
        info.as_ref()
            .ok()
            .filter(|i| !i.source.is_empty() && i.source != "standard")
            .map(|i| format!(" _(from {})_", i.source))
            .unwrap_or_default()
    );
    if let Some(h) = info.as_ref().ok().and_then(|i| i.hint) {
        let _ = writeln!(out, "  - Where to start: {h}");
    }
    let v = d.vehicle.as_ref();
    let mut links = vec![format!(
        "[look it up]({})",
        lookup::code_search(code, text.as_deref(), v)
    )];
    for p in dtc::parts(code) {
        links.push(format!("[{p}]({})", lookup::part_search(p, v)));
    }
    let _ = writeln!(out, "  - Links: {}", links.join(" · "));
}

pub fn markdown(d: &Diagnosis) -> String {
    let mut o = String::new();
    let name = d
        .vehicle
        .as_ref()
        .map(|v| v.name())
        .filter(|n| !n.is_empty());
    let _ = writeln!(o, "# Icarus diagnostic report\n");
    let _ = writeln!(
        o,
        "{}{}\n",
        d.when,
        name.as_ref().map(|n| format!(", {n}")).unwrap_or_default()
    );
    let _ = writeln!(o, "## Vehicle\n");
    let _ = writeln!(o, "| | |\n| --- | --- |");
    let _ = writeln!(o, "| VIN | {} |", d.vin.as_deref().unwrap_or("(not given)"));
    if let Some(v) = &d.vin_info {
        let _ = writeln!(o, "| Built in | {} |", v.region);
        if let Some(y) = v.year {
            let _ = writeln!(o, "| Model year | {y} |");
        }
    }
    if let Some(p) = &d.profile {
        let _ = writeln!(o, "| Make profile | {} |", p.name);
    }
    let _ = writeln!(o, "| Adapter | {} |", d.adapter);
    let _ = writeln!(o, "| Protocol | {} |", d.protocol);
    if let Some(v) = d.voltage {
        let _ = writeln!(o, "| Battery | {v:.1} V |");
    }
    if let Some(f) = d.fuel_system {
        let _ = writeln!(o, "| Fuel system | {f} |");
    }
    if let Some(v) = &d.vehicle {
        if let Some(url) = lookup::parts_catalog(v) {
            let _ = writeln!(o, "\nParts catalog for this vehicle: [RockAuto]({url})");
        }
    }
    let _ = writeln!(o, "\n## Check-engine light and readiness monitors\n");
    for (module, m) in &d.monitors {
        let _ = writeln!(
            o,
            "{module}: check-engine light **{}**, {} confirmed code(s).\n",
            if m.mil { "ON" } else { "off" },
            m.codes
        );
        for (t, done) in &m.tests {
            let _ = writeln!(
                o,
                "- {t}: {}",
                if *done {
                    "complete"
                } else {
                    "not yet run since codes were cleared"
                }
            );
        }
        let _ = writeln!(o);
    }
    let _ = writeln!(o, "## Trouble codes (OBD-II)\n");
    if d.codes.is_empty() {
        let _ = writeln!(o, "None stored, pending or permanent.\n");
    }
    for (kind, module, codes) in &d.codes {
        let _ = writeln!(
            o,
            "{} codes, {module}:\n",
            kind[..1].to_uppercase() + &kind[1..]
        );
        for c in codes {
            code_block(&mut o, c, "", d);
        }
        let _ = writeln!(o);
    }
    if !d.freeze.is_empty() {
        let _ = writeln!(
            o,
            "## Freeze frame\n\nWhat the engine was doing when the code set:\n"
        );
        for f in &d.freeze {
            let _ = writeln!(
                o,
                "{} ({}){}:\n",
                f.module,
                f.from,
                f.code
                    .as_ref()
                    .map(|c| format!(", stored by {c}"))
                    .unwrap_or_default()
            );
            for (p, v) in &f.values {
                if let Some(x) = j1979::pid(*p) {
                    let _ = writeln!(o, "- {}: {v:.1} {}", x.name, x.unit);
                }
            }
            let _ = writeln!(o);
        }
    }
    if !d.selftests.is_empty() {
        let _ = writeln!(o, "## On-board test results\n\nThe engine module's own measurements and their limits (service 06):\n");
        let mut last = None;
        for t in &d.selftests {
            if last != Some(t.mid) {
                let _ = writeln!(
                    o,
                    "{}{}:\n",
                    if last.is_some() { "\n" } else { "" },
                    crate::mode06::monitor_name(t.mid)
                );
                last = Some(t.mid);
            }
            let _ = writeln!(
                o,
                "- {}: **{}**, {}",
                crate::mode06::test_name(t.mid, t.tid),
                if t.passed() { "pass" } else { "FAIL" },
                selftest_values(t)
            );
        }
        let _ = writeln!(o);
    }
    if !d.modules.is_empty() {
        let _ = writeln!(o, "## Modules\n");
        for (m, id, codes) in &d.modules {
            let _ = writeln!(o, "### {:03X} {}\n", m.tx, m.name);
            for (what, v) in id {
                let _ = writeln!(o, "- {what}: {v}");
            }
            match codes {
                None => {
                    let _ = writeln!(o, "- trouble codes: the module would not say");
                }
                Some((_, list)) if list.is_empty() => {
                    let _ = writeln!(o, "- no trouble codes");
                }
                Some((_, list)) => {
                    let _ = writeln!(o);
                    for c in list {
                        code_block(&mut o, &c.code, &c.states.join(", "), d);
                    }
                }
            }
            let _ = writeln!(o);
        }
    }
    if !d.snapshot.is_empty() {
        let _ = writeln!(o, "## Live values when the report was made\n");
        for (p, v) in &d.snapshot {
            if let Some(x) = j1979::pid(*p) {
                let _ = writeln!(o, "- {}: {v:.2} {}", x.name, x.unit);
            }
        }
        let _ = writeln!(o);
    }
    let _ = writeln!(o, "---\nMade by Icarus, read-only: nothing on the vehicle was changed. The parts listed are where faults usually lie, a starting point for testing, not a diagnosis.");
    o
}

/// "288 (limits 0 to 672)" or with units.
pub fn selftest_values(t: &crate::mode06::TestResult) -> String {
    match t.scaled() {
        Some((v, lo, hi, unit)) => {
            let f = |x: f64| {
                if x.fract() == 0.0 {
                    format!("{x:.0}")
                } else {
                    format!("{x:.3}")
                }
            };
            format!(
                "{}{}{unit} (limits {} to {})",
                f(v),
                if unit.is_empty() { "" } else { " " },
                f(lo),
                f(hi)
            )
        }
        None => format!(
            "{} (limits {} to {}, unit code {:02X})",
            t.value, t.min, t.max, t.uasid
        ),
    }
}

/// In the vehicle's own folder (`~/Documents/Icarus/Vehicles/<vehicle>/Reports/report-2026-11-07-19-40.md`), or
/// `~/Documents/Icarus/reports` when the vehicle gave no VIN (or is the simulation).
pub fn default_path(d: &Diagnosis) -> PathBuf {
    let home = std::env::var_os("HOME")
        .map(PathBuf::from)
        .unwrap_or_default();
    let stamp: String = d
        .when
        .chars()
        .map(|c| if c == ' ' || c == ':' { '-' } else { c })
        .collect();
    if let Some(v) = d.vin.as_deref().filter(|v| *v != crate::sim::VIN) {
        let names = garage::load();
        return garage::folder_in(&garage::library(), v, names.get(v).map(String::as_str))
            .join("Reports")
            .join(format!("report-{stamp}.md"));
    }
    home.join("Documents/Icarus/reports")
        .join(format!("icarus-report-{stamp}.md"))
}

/// The default path, or the same with -2, -3... when a report of the same minute exists: a report never replaces another.
pub fn free_path(d: &Diagnosis) -> PathBuf {
    let first = default_path(d);
    if !first.exists() {
        return first;
    }
    let stem = first
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("icarus-report")
        .to_string();
    (2..)
        .map(|n| first.with_file_name(format!("{stem}-{n}.md")))
        .find(|p| !p.exists())
        .unwrap()
}

/// Writes the report and returns where (never over another report unless `path` names it).
pub fn save(d: &Diagnosis, path: Option<PathBuf>) -> std::io::Result<PathBuf> {
    if let Some(v) = d.vin.as_deref() {
        let _ = garage::ensure(v); // the vehicle's folder, with its Reports folder (not for the simulation)
    }
    let p = path.unwrap_or_else(|| free_path(d));
    if let Some(dir) = p.parent() {
        std::fs::create_dir_all(dir)?;
    }
    std::fs::write(&p, markdown(d))?;
    Ok(p)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::SimCar;

    #[test]
    fn a_report_of_the_simulated_vehicle() {
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        let d = collect(&mut elm, &vehicles::builtin(), true).unwrap();
        let md = markdown(&d);
        for want in [
            "## Vehicle",
            "5TFICARUSSIM00001",
            "Toyota / Lexus / Scion",
            "check-engine light **ON**",
            "**P0171** System too lean (bank 1)",
            "Where to start: Too much air",
            "[look it up](https://duckduckgo.com/?q=P0171+System+too+lean+%28bank+1%29)",
            "[mass air flow (MAF) sensor](https://",
            "## Freeze frame",
            "stored by P0171",
            "Vehicle speed: 72.0 km/h",
            "### 7B0 ABS / VSC / TRC",
            "**C1241** (stored, present now) Low battery voltage",
            "_(from Toyota / Lexus / Scion)_",
            "## Live values",
        ] {
            assert!(md.contains(want), "missing {want:?} in\n{md}");
        }
        let p = default_path(&Diagnosis {
            when: "2026-11-07 19:40".into(),
            ..Default::default()
        });
        assert!(
            p.ends_with("Documents/Icarus/reports/icarus-report-2026-11-07-19-40.md"),
            "{}",
            p.display()
        );
    }
}
