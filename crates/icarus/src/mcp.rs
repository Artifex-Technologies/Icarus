//! `icarus mcp`: Icarus as a Model Context Protocol server (stdio, JSON-RPC 2.0, one message per line), so an assistant
//! (Sarina, through her operator file ~/.sarina/mcp.json) can read a vehicle and explain it.
//!
//! **Read-only, always.** The tools here only read: identity, codes with their meanings, the freeze frame, live values,
//! every module's codes, the tires/gears arithmetic, and the learning sessions Icarus saved while watching another tool. Clearing codes, actuator tests and anything else that changes a
//! vehicle are never offered to a model (a test checks the tool list), as Sarina's own rules keep launching agents
//! CLI-only. Nothing from an immobilizer, smart-key or certification module is given out (`icarus_obd::fence`): the
//! module list does not ask them, module data sends them no data request, and a learning session never holds their
//! messages. The adapter is the one named by ICARUS_ADAPTER in the server's environment, else the one the window last used.
//! What a tool says may go on to a model in the cloud, so no tool hands on the VIN whole or a module's serial number:
//! `vehicle_check` hides the VIN's own serial number (its last 6 characters) unless asked for it (`show_vin`); a learning
//! session is saved with both masked (`bus::Analyzer`'s log); and module data never reads either (`moddata`'s gates).

use icarus_obd::elm::Elm;
use icarus_obd::port::Target;
use icarus_obd::{dtc, garage, j1979, learning, report, uds, vehicles};
use serde_json::{json, Value};
use std::io::{BufRead, Write};

/// Icarus's own version: the VERSION file at the top of the repository, read when this is built (Labyrinth_OS's docs/VERSIONING.md).
const VERSION: &str = include_str!("../../../VERSION").trim_ascii();

fn tool_list() -> Value {
    json!([
        {
            "name": "explain_code",
            "description": "What a vehicle trouble code means, where to start and the parts usually involved. Offline.",
            "inputSchema": {"type": "object", "properties": {
                "code": {"type": "string", "description": "e.g. P0171"},
                "vehicle": {"type": "string", "description": "optional, e.g. 2017 Toyota Tacoma"}
            }, "required": ["code"]}
        },
        {
            "name": "vehicle_check",
            "description": "Read the connected vehicle (OBD adapter): identity, check-engine light, readiness, trouble codes explained, freeze frame, current values. Read-only. The VIN's last 6 characters (its serial number) are hidden unless show_vin is true.",
            "inputSchema": {"type": "object", "properties": {
                "show_vin": {"type": "boolean", "description": "optional: the whole VIN; only when the person asks for it"}
            }}
        },
        {
            "name": "vehicle_modules",
            "description": "Every module's own trouble codes (ABS, airbags, transmission, body), explained. Read-only; can take a minute.",
            "inputSchema": {"type": "object", "properties": {}}
        },
        {
            "name": "module_data",
            "description": "Each module's own values (a dealer tool's data list: transmission temperature, knock correction, wheel speeds, tire pressures...), read once from the vehicle with the module-data set that fits it (imported from OBDb or ABRP, or the simulator's). Read-only: data reads only (services 21 and 22), paced, airbag modules identity only. Optional: one module by its ID (701, 750.2A).",
            "inputSchema": {"type": "object", "properties": {
                "module": {"type": "string", "description": "optional: one module, e.g. 701 or 750.2A"}
            }}
        },
        {
            "name": "learning_sessions",
            "description": "What Icarus saw while another diagnostic tool worked on one of my vehicles: the marks I made, the settings and output tests it learned, what it saw but could not learn, every message in words. Without a file: the list and the newest session; with one: that session. Read-only.",
            "inputSchema": {"type": "object", "properties": {
                "file": {"type": "string", "description": "optional: a session's file name from the list, e.g. learning-2026-11-08-1530.md"}
            }}
        },
        {
            "name": "drive_logs",
            "description": "Recorded drives (icarus obd live --csv): without a file, the list and a summary of the newest; with one, that drive's summary: fuel trims for each bank by driving condition (idle, light, moderate, hard) and the gap between the banks, the rear oxygen sensors, coolant and voltage, and sudden timing drops (at the rev limiter or a shift, or other). Offline: reads files only.",
            "inputSchema": {"type": "object", "properties": {
                "file": {"type": "string", "description": "optional: a recording's file name from the list, e.g. drive-2026-11-13_132432.csv"}
            }}
        },
        {
            "name": "tires_gears",
            "description": "Speedometer error and gearing for a tire size change, and the regear that restores stock.",
            "inputSchema": {"type": "object", "properties": {
                "stock": {"type": "string", "description": "e.g. P265/65R17"},
                "new": {"type": "string", "description": "e.g. LT285/70R17"},
                "stock_ratio": {"type": "number"},
                "installed_ratio": {"type": "number"},
                "speed": {"type": "number"}
            }, "required": ["stock", "new"]}
        }
    ])
}

/// The adapter the window last connected to (icarus-app writes ~/.config/icarus/adapter).
fn remembered_adapter() -> Option<String> {
    let base = std::env::var_os("XDG_CONFIG_HOME")
        .map(std::path::PathBuf::from)
        .or_else(|| {
            std::env::var_os("HOME").map(|h| std::path::PathBuf::from(h).join(".config"))
        })?;
    let t = std::fs::read_to_string(base.join("icarus/adapter")).ok()?;
    let a = t.lines().next()?.trim().to_string();
    (!a.is_empty()).then_some(a)
}

/// The recorded drives: the list (newest first, at most 20) and the newest one's summary, or the named one's.
fn drive_logs(file: Option<&str>) -> Result<String, String> {
    drive_logs_in(&icarus_obd::drivelog::folders(), file)
}

/// `drive_logs` on the recordings in these folders (a test gives its own).
fn drive_logs_in(dirs: &[std::path::PathBuf], file: Option<&str>) -> Result<String, String> {
    use icarus_obd::drivelog;
    let dirs = dirs.to_vec();
    if let Some(f) = file.filter(|f| !f.is_empty()) {
        let p = drivelog::by_name(&dirs, f).ok_or_else(|| format!("no recording named {f}"))?;
        return drivelog::summarize_file(&p);
    }
    let all = drivelog::list(&dirs);
    let Some(newest) = all.first() else {
        return Ok(
            "No recorded drives yet (icarus obd live ... --csv FILE, or Record in the window)."
                .into(),
        );
    };
    let mut out = String::from("Recordings, newest first:\n");
    for p in all.iter().take(20) {
        out.push_str(&format!(
            "- {}\n",
            p.file_name()
                .map(|f| f.to_string_lossy())
                .unwrap_or_default()
        ));
    }
    out.push('\n');
    out.push_str(
        &drivelog::summarize_file(newest)
            .unwrap_or_else(|e| format!("The newest could not be read: {e}")),
    );
    Ok(out)
}

/// The saved learning sessions, newest first: (file, which vehicle). Reads only (no folder is made).
fn all_sessions() -> Vec<(std::path::PathBuf, String)> {
    let mut v: Vec<(std::path::PathBuf, String)> = Vec::new();
    for (_, name, dir) in garage::vehicles() {
        let label = name.unwrap_or_else(|| "a vehicle".into());
        v.extend(
            learning::sessions(&dir.join("Logs"))
                .into_iter()
                .map(|p| (p, label.clone())),
        );
    }
    if let Some(sim) = icarus_obd::audit::path().and_then(|p| p.parent().map(|d| d.join("sim"))) {
        v.extend(
            learning::sessions(&sim)
                .into_iter()
                .map(|p| (p, "the simulated vehicle".to_string())),
        );
    }
    v.sort_by_key(|(p, _)| std::cmp::Reverse(std::fs::metadata(p).and_then(|m| m.modified()).ok()));
    v
}

fn learning_sessions(file: Option<&str>) -> Result<String, String> {
    let all = all_sessions();
    let name = |p: &std::path::Path| {
        p.file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("")
            .to_string()
    };
    if let Some(f) = file {
        // only a session from the list: a name, never a path
        let (p, _) = all
            .iter()
            .find(|(p, _)| name(p) == f)
            .ok_or_else(|| format!("no learning session named {f:?}"))?;
        return std::fs::read_to_string(p).map_err(|e| e.to_string());
    }
    let Some((newest, _)) = all.first() else {
        return Ok(
            "Icarus has no learning sessions yet (the Learning page, or icarus learn, makes one)."
                .into(),
        );
    };
    let mut t = String::from("Learning sessions, newest first:\n");
    for (p, v) in all.iter().take(20) {
        t += &format!("- {} ({v})\n", name(p));
    }
    t += "\nThe newest:\n\n";
    t += &std::fs::read_to_string(newest).map_err(|e| e.to_string())?;
    Ok(t)
}

fn connect() -> Result<Elm, String> {
    // a drive being recorded has the adapter (one Bluetooth connection): taking it would end the recording
    if let Some(p) = icarus_obd::drivelog::recording_now() {
        return Err(format!(
            "the adapter is busy recording a drive ({}); drive_logs reads recordings, the one being written included",
            p.file_name().map(|f| f.to_string_lossy().into_owned()).unwrap_or_default()
        ));
    }
    // the adapter named in this server's definition, else the one the window last connected to (~/.config/icarus/adapter),
    // so a definition need not carry a Bluetooth address
    let spec = std::env::var("ICARUS_ADAPTER").ok().filter(|a| !a.trim().is_empty()).or_else(remembered_adapter).ok_or_else(|| {
        "no adapter: set ICARUS_ADAPTER in this server's definition (e.g. bt:AA:BB:CC:DD:EE:FF), or connect once in the window".to_string()
    })?;
    icarus_obd::elm::connect_target(&Target::parse(&spec)?)
        .map(|(elm, _)| elm)
        .map_err(|e| format!("{spec}: {e}"))
}

fn code_line(code: &str, extra: &str, profile: Option<&vehicles::Profile>) -> String {
    match dtc::explain(code, profile) {
        Ok(i) => {
            let text = i.text.clone().unwrap_or_else(|| {
                if i.standard {
                    "(no description yet)".into()
                } else {
                    "the maker's own code".into()
                }
            });
            let mut s = format!("- {code}{extra}: {text}");
            if !s.ends_with(['.', '!', '?']) {
                s.push('.');
            }
            if let Some(h) = i.hint {
                s += &format!(" Where to start: {h}");
            }
            for (label, t) in dtc::depth(&i) {
                s += &format!(" {label}: {t}.");
            }
            let p = dtc::parts(code);
            if !p.is_empty() {
                s += &format!(" Parts usually involved: {}.", p.join(", "));
            }
            s
        }
        Err(e) => format!("- {code}: {e}"),
    }
}

fn values(list: &[(u8, f64)]) -> String {
    list.iter()
        .filter_map(|(p, v)| {
            j1979::pid(*p).map(|d| match d.unit {
                "km/h" => format!(
                    "{} {v:.0} km/h ({:.0} mph)",
                    d.name.to_lowercase(),
                    v / 1.609344
                ),
                unit => format!("{} {v:.1} {unit}", d.name.to_lowercase()),
            })
        })
        .collect::<Vec<_>>()
        .join(", ")
}

/// `vehicle_check`: the connected vehicle read and told (`check_text`), its VIN whole only with `show_vin`.
fn vehicle_check(show_vin: bool) -> Result<String, String> {
    let mut elm = connect()?;
    let all = vehicles::load(vehicles::user_dir().as_deref()).0;
    let d = report::collect(&mut elm, &all, false).map_err(|e| e.to_string())?;
    let notes = dropped_notes(&mut elm);
    Ok(check_text(&d, &notes, show_vin))
}

/// What `vehicle_check` says of a reading `d`, after `notes`. The VIN's serial number (its last 6 characters) is hidden
/// unless `show_vin` (ICA-106): what a tool says may go on to a model in the cloud, and the rest of the VIN (maker, model,
/// year, plant) is what an explanation needs.
fn check_text(d: &report::Diagnosis, notes: &str, show_vin: bool) -> String {
    let mut o = notes.to_string();
    let vin = match d.vin.as_deref() {
        None => "not given".to_string(),
        Some(v) if show_vin => v.to_string(),
        Some(v) => format!(
            "{} (its last 6 characters hidden; show_vin gives them)",
            icarus_obd::vin::masked(v)
        ),
    };
    o += &format!("Vehicle: VIN {vin}");
    if let Some(v) = &d.vin_info {
        o += &format!(
            ", built in {}{}",
            v.region,
            v.year
                .map(|y| format!(", model year {y}"))
                .unwrap_or_default()
        );
    }
    if let Some(n) = d
        .vehicle
        .as_ref()
        .map(|v| v.name())
        .filter(|n| !n.is_empty())
    {
        o += &format!(", named \"{n}\"");
    }
    o += &format!(
        ". Make profile: {}. Protocol: {}.",
        d.profile
            .as_ref()
            .map_or("generic OBD-II", |p| p.name.as_str()),
        d.protocol
    );
    if let Some(v) = d.voltage {
        o += &format!(" Battery {v:.1} V.");
    }
    o += "\n";
    for (module, m) in &d.monitors {
        let open: Vec<&str> = m.tests.iter().filter(|t| !t.1).map(|t| t.0).collect();
        o += &format!("{module}: check-engine light {}, {} confirmed code(s); monitors not yet complete: {}.\n", if m.mil { "ON" } else { "off" }, m.codes, if open.is_empty() { "none".into() } else { open.join(", ") });
    }
    if let Some(f) = d.fuel_system {
        o += &format!("Fuel system: {f}.\n");
    }
    if d.codes.is_empty() {
        o += "No OBD-II trouble codes stored, pending or permanent.\n";
    } else {
        o += "Trouble codes:\n";
        for (kind, module, codes) in &d.codes {
            for c in codes {
                o += &code_line(c, &format!(" ({kind}, {module})"), d.profile.as_ref());
                o += "\n";
            }
        }
    }
    for f in &d.freeze {
        o += &format!(
            "Freeze frame{}: {}.\n",
            f.code
                .as_ref()
                .map(|c| format!(" (when {c} set)"))
                .unwrap_or_default(),
            values(&f.values)
        );
    }
    if !d.snapshot.is_empty() {
        o += &format!("Now: {}.\n", values(&d.snapshot));
    }
    let failing: Vec<String> = d
        .selftests
        .iter()
        .filter(|t| !t.passed())
        .map(|t| {
            format!(
                "{} ({})",
                icarus_obd::mode06::monitor_name(t.mid),
                icarus_obd::mode06::test_name(t.mid, t.tid)
            )
        })
        .collect();
    if !d.selftests.is_empty() {
        o += &format!(
            "On-board test results: {} read, {} failing{}.\n",
            d.selftests.len(),
            failing.len(),
            if failing.is_empty() {
                String::new()
            } else {
                format!(": {}", failing.join("; "))
            }
        );
    }
    o
}

/// `module_data` on a connected vehicle: the values as text, with the set's attribution.
fn module_data_on(elm: &mut Elm, module: Option<&str>) -> Result<String, String> {
    let all = vehicles::load(vehicles::user_dir().as_deref()).0;
    let us = icarus_core::moddata::units::us_customary();
    Ok(icarus_obd::vehicledata::read(elm, &all, module, us)?.text())
}

fn vehicle_modules() -> Result<String, String> {
    let mut elm = connect()?;
    let all = vehicles::load(vehicles::user_dir().as_deref()).0;
    vehicle_modules_on(&mut elm, &all, &icarus_obd::fence::current())
}

/// Every module's codes on a connected vehicle. The immobilizer, smart-key and certification modules (`fence`) are not
/// asked, so nothing from them reaches an assistant.
fn vehicle_modules_on(
    elm: &mut Elm,
    all: &[vehicles::Profile],
    fence: &icarus_obd::fence::Fence,
) -> Result<String, String> {
    let e = |x: icarus_obd::elm::ObdError| x.to_string();
    let vin = j1979::vin(elm).map_err(e)?;
    let profile = vin
        .as_deref()
        .and_then(|v| vehicles::for_vin(all, v))
        .cloned()
        .or_else(|| vehicles::find(all, "generic").ok().cloned());
    let mut o = String::new();
    let protocol = elm.protocol;
    for m in profile
        .as_ref()
        .map(|p| vehicles::candidates_on(p, protocol))
        .unwrap_or_default()
    {
        if m.unreachable(elm.reach()).is_some() {
            continue;
        }
        if let Some(k) = fence.at(m.tx, m.ext, m.rx()) {
            o += &format!(
                "{:03X} {}: not asked ({}: nothing from a key module is given to an assistant).\n",
                m.tx,
                m.name,
                k.describe()
            );
            continue;
        }
        let (tx, rx) = (m.tx, m.rx());
        let codes = match uds::at_module(elm, &m, |l| {
            if !uds::present(l, tx, rx)? {
                return Ok(None);
            }
            uds::module_dtcs(l, tx, rx).map(Some)
        }) {
            // the adapter refused to switch to that module's bus
            Err(icarus_obd::elm::ObdError::Adapter(_)) if m.on_ms_can() => continue,
            r => r.map_err(e)?,
        };
        let Some(codes) = codes else { continue };
        match codes {
            Err(why) => {
                o += &format!(
                    "{:03X} {}: answered, but none of Icarus's requests gave its codes ({why}).\n",
                    m.tx, m.name
                )
            }
            // a module listing every code it can set, none failed (UDS 19 02 FF), is clean (ICA-207)
            Ok(read) => match read.none_failed() {
                Some(clean) => o += &format!("{:03X} {}: {clean}.\n", m.tx, m.name),
                None => {
                    o += &format!("{:03X} {}:\n", m.tx, m.name);
                    for c in &read.codes {
                        o += &code_line(
                            &c.code,
                            &format!(" ({})", c.states.join(", ")),
                            profile.as_ref(),
                        );
                        o += "\n";
                    }
                    if let Some(more) = read.others() {
                        o += &format!("{more}.\n");
                    }
                }
            },
        }
    }
    if o.is_empty() {
        o = "No module answered.".into();
    }
    o += &dropped_notes(elm);
    Ok(o)
}

/// The notes on modules left out of a broadcast's answers (`Elm::take_dropped`), one line each, so none is lost silently.
fn dropped_notes(elm: &mut Elm) -> String {
    elm.take_dropped()
        .iter()
        .map(|n| format!("Note: {n}.\n"))
        .collect()
}

/// Whether `vehicle_check` was asked for the whole VIN: only by `show_vin: true`. Anything else (left out, false, the text
/// "true", a number) hides its serial number.
fn show_vin(args: &Value) -> bool {
    args.get("show_vin").and_then(Value::as_bool) == Some(true)
}

fn call(name: &str, args: &Value) -> Result<String, String> {
    let s = |k: &str| args.get(k).and_then(|v| v.as_str()).map(str::to_string);
    let n = |k: &str| args.get(k).and_then(|v| v.as_f64());
    match name {
        "explain_code" => {
            let code = s("code").ok_or("give a code, e.g. P0171")?;
            let all = vehicles::load(vehicles::user_dir().as_deref()).0;
            let vehicle = s("vehicle").map(|v| garage::Described::parse(&v));
            let profile = vehicle
                .as_ref()
                .and_then(|v| vehicles::find(&all, &v.make).ok());
            let i = dtc::explain(&code, profile).map_err(|e| e.to_string())?;
            Ok(format!(
                "{}\n{}",
                dtc::anatomy(&i),
                code_line(&i.code, "", profile)
            ))
        }
        "vehicle_check" => vehicle_check(show_vin(args)),
        "drive_logs" => drive_logs(args["file"].as_str()),
        "vehicle_modules" => vehicle_modules(),
        "module_data" => module_data_on(&mut connect()?, s("module").as_deref()),
        "learning_sessions" => learning_sessions(s("file").as_deref()),
        "tires_gears" => {
            let stock =
                icarus_core::tires::Tire::parse(&s("stock").ok_or("give the stock tire size")?)?;
            let new = icarus_core::tires::Tire::parse(&s("new").ok_or("give the new tire size")?)?;
            let (gear, installed, speed) = (
                n("stock_ratio"),
                n("installed_ratio"),
                n("speed").unwrap_or(65.0),
            );
            let c = icarus_core::tires::change(
                stock,
                new,
                speed,
                gear,
                installed.filter(|_| gear.is_some()),
            );
            Ok(icarus_core::tires::explain(&c, speed, gear, installed).join("\n"))
        }
        other => Err(format!("no tool {other}")),
    }
}

/// Answers one JSON-RPC message (None for a notification).
pub fn handle(msg: &Value) -> Option<Value> {
    let id = msg.get("id").cloned()?;
    let method = msg.get("method").and_then(|m| m.as_str()).unwrap_or("");
    let result = match method {
        "initialize" => {
            let version = msg
                .pointer("/params/protocolVersion")
                .and_then(|v| v.as_str())
                .unwrap_or("2024-11-05");
            json!({"protocolVersion": version, "capabilities": {"tools": {}}, "serverInfo": {"name": "icarus", "version": VERSION}})
        }
        "ping" => json!({}),
        "tools/list" => json!({"tools": tool_list()}),
        "tools/call" => {
            let name = msg
                .pointer("/params/name")
                .and_then(|v| v.as_str())
                .unwrap_or("");
            let args = msg
                .pointer("/params/arguments")
                .cloned()
                .unwrap_or(json!({}));
            match call(name, &args) {
                Ok(text) => json!({"content": [{"type": "text", "text": text}], "isError": false}),
                Err(e) => json!({"content": [{"type": "text", "text": e}], "isError": true}),
            }
        }
        _ => {
            return Some(
                json!({"jsonrpc": "2.0", "id": id, "error": {"code": -32601, "message": format!("no method {method}")}}),
            )
        }
    };
    Some(json!({"jsonrpc": "2.0", "id": id, "result": result}))
}

/// Serves on stdin/stdout until stdin closes.
pub fn serve() -> Result<(), String> {
    let stdin = std::io::stdin();
    let mut out = std::io::stdout().lock();
    for line in stdin.lock().lines() {
        let line = line.map_err(|e| e.to_string())?;
        if line.trim().is_empty() {
            continue;
        }
        let reply = match serde_json::from_str::<Value>(&line) {
            Ok(msg) => handle(&msg),
            Err(_) => Some(
                json!({"jsonrpc": "2.0", "id": null, "error": {"code": -32700, "message": "not JSON"}}),
            ),
        };
        if let Some(r) = reply {
            writeln!(out, "{r}").map_err(|e| e.to_string())?;
            out.flush().map_err(|e| e.to_string())?;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every tool offered; a new one is added here only after checking it cannot change a vehicle.
    const TOOLS: &[&str] = &[
        "explain_code",
        "vehicle_check",
        "vehicle_modules",
        "module_data",
        "learning_sessions",
        "drive_logs",
        "tires_gears",
    ];

    #[test]
    fn only_reading_tools_are_offered() {
        let r = handle(&json!({"jsonrpc": "2.0", "id": 1, "method": "tools/list"})).unwrap();
        let names: Vec<String> = r["result"]["tools"]
            .as_array()
            .unwrap()
            .iter()
            .map(|t| t["name"].as_str().unwrap().to_string())
            .collect();
        assert_eq!(names, TOOLS);
        for n in &names {
            for banned in [
                "clear", "test", "flash", "write", "actuator", "control", "reset",
            ] {
                assert!(!n.contains(banned), "{n} would change a vehicle");
            }
        }
    }

    /// ICA-106: vehicle_check hands a model the VIN without its serial number unless show_vin asks for it whole.
    #[test]
    fn vehicle_check_hides_the_vins_serial_number_unless_asked() {
        let vin = icarus_obd::sim::VIN;
        let d = report::Diagnosis {
            vin: Some(vin.to_string()),
            vin_info: icarus_obd::vin::decode(vin).ok(),
            protocol: "ISO 15765-4 CAN (11 bit ID, 500 kbaud)".into(),
            ..Default::default()
        };
        let hidden = check_text(&d, "", false);
        assert!(
            !hidden.contains(vin) && !hidden.contains(&vin[11..]),
            "{hidden}"
        );
        assert!(
            hidden.contains(&format!("VIN {}******", &vin[..11])),
            "{hidden}"
        );
        assert!(
            hidden.contains("model year"),
            "the year still reads: {hidden}"
        );
        let whole = check_text(&d, "", true);
        assert!(whole.contains(&format!("VIN {vin}")), "{whole}");
        let none = check_text(&report::Diagnosis::default(), "", false);
        assert!(none.contains("VIN not given"), "{none}");
        // the tool says how to ask for it
        let r = handle(&json!({"jsonrpc": "2.0", "id": 1, "method": "tools/list"})).unwrap();
        let tool = r["result"]["tools"]
            .as_array()
            .unwrap()
            .iter()
            .find(|t| t["name"] == "vehicle_check")
            .unwrap()
            .clone();
        assert_eq!(
            tool["inputSchema"]["properties"]["show_vin"]["type"],
            "boolean"
        );
        // what a tools/call's arguments ask for: the whole VIN only with show_vin true itself
        for (args, whole) in [
            (json!({}), false),
            (json!(null), false),
            (json!({"show_vin": false}), false),
            (json!({"show_vin": "true"}), false),
            (json!({"show_vin": 1}), false),
            (json!({"show_vin": null}), false),
            (json!({"show_vin": true}), true),
        ] {
            assert_eq!(show_vin(&args), whole, "{args}");
        }
    }

    /// ICA-26: the drive_logs tool reads a recording as `obd live --csv` writes it (the adapter line first): listed, the newest
    /// summarized without a name, that one by its name, and a name outside the folders refused.
    #[test]
    fn drive_logs_reads_a_recording() {
        let dir = std::env::temp_dir().join(format!("icarus-mcp-logs-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let head = "time_s,\"Engine speed [rpm]\",\"Vehicle speed [km/h]\",\"Coolant temperature [°C]\",\"Short-term fuel trim, bank 1 [%]\",\"Long-term fuel trim, bank 1 [%]\",\"Short-term fuel trim, bank 2 [%]\",\"Long-term fuel trim, bank 2 [%]\",\"Engine load [%]\",\"Control module voltage [V]\"";
        let mut csv = format!(
            "{}\n{head}\n",
            icarus_obd::drivelog::adapter_line("ELM327 v1.4b")
        );
        for k in 0..20 {
            csv.push_str(&format!(
                "{}.0,{},{},90,1.0,-6.0,0.5,-3.0,{},13.9\n",
                k,
                if k < 10 { 650 } else { 1900 },
                if k < 10 { 0 } else { 70 },
                if k < 10 { 16 } else { 35 }
            ));
        }
        std::fs::write(dir.join("drive-2026-11-24_070000.csv"), &csv).unwrap();
        let dirs = [dir.clone()];
        let all = drive_logs_in(&dirs, None).unwrap();
        assert!(
            all.contains("- drive-2026-11-24_070000.csv") && all.contains("idle"),
            "{all}"
        );
        let one = drive_logs_in(&dirs, Some("drive-2026-11-24_070000.csv")).unwrap();
        assert!(one.contains("drive-2026-11-24_070000.csv"), "{one}");
        assert!(drive_logs_in(&dirs, Some("../x.csv")).is_err());
        assert!(drive_logs_in(&[dir.join("none")], None)
            .unwrap()
            .starts_with("No recorded drives yet"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_session() {
        let init = handle(&json!({"jsonrpc": "2.0", "id": 0, "method": "initialize", "params": {"protocolVersion": "2025-03-26"}})).unwrap();
        assert_eq!(init["result"]["protocolVersion"], "2025-03-26");
        assert_eq!(
            init["result"]["serverInfo"]["version"],
            include_str!("../../../VERSION").trim(),
            "Icarus's VERSION"
        );
        assert!(
            handle(&json!({"jsonrpc": "2.0", "method": "notifications/initialized"})).is_none(),
            "no answer to a notification"
        );
        let r = handle(&json!({"jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": {"name": "explain_code", "arguments": {"code": "P0304"}}})).unwrap();
        let text = r["result"]["content"][0]["text"].as_str().unwrap();
        assert!(
            text.contains("Misfire detected, cylinder 4") && text.contains("ignition coil"),
            "{text}"
        );
        let r = handle(&json!({"jsonrpc": "2.0", "id": 3, "method": "tools/call", "params": {"name": "tires_gears", "arguments": {"stock": "P265/65R17", "new": "LT285/70R17", "stock_ratio": 3.91, "installed_ratio": 4.88}}})).unwrap();
        assert!(r["result"]["content"][0]["text"]
            .as_str()
            .unwrap()
            .contains("acts like a 4.56"));
        let r = handle(&json!({"jsonrpc": "2.0", "id": 4, "method": "tools/call", "params": {"name": "clear_codes", "arguments": {}}})).unwrap();
        assert_eq!(r["result"]["isError"], true);
        // module data, read-only, from the simulated vehicle
        let mut elm = Elm::connect(Box::new(icarus_obd::sim::SimCar::new())).unwrap();
        let text = module_data_on(&mut elm, Some("701")).unwrap();
        assert!(
            text.contains("Module data from the set sim") && text.contains("701"),
            "{text}"
        );
        let r = handle(&json!({"jsonrpc": "2.0", "id": 5, "method": "resources/list"})).unwrap();
        assert_eq!(r["error"]["code"], -32601);
    }

    /// ICA-101: a key module (the simulated cluster, given the smart-key kind for this test) is not asked by
    /// `vehicle_modules`, so nothing from it reaches an assistant; the other modules are read as before.
    #[test]
    fn vehicle_modules_never_asks_a_key_module() {
        let mut all = vehicles::builtin();
        let mut elm = Elm::connect(Box::new(icarus_obd::sim::SimCar::new())).unwrap();
        let open = icarus_obd::fence::Fence::default();
        let text = vehicle_modules_on(&mut elm, &all, &open).unwrap();
        assert!(text.contains("7C0 instrument cluster"), "{text}");
        assert!(!text.contains("not asked"), "{text}");
        // ICA-207: the engine and transmission at 700/701 list every code they can set, none failed: clean, none listed
        for clean in [
            "700 engine (UDS): no trouble codes (6 it can set, none failed).",
            "701 transmission (UDS): no trouble codes (4 it can set, none failed).",
        ] {
            assert!(text.contains(clean), "{clean:?} in {text}");
        }
        assert!(
            !text.contains("P0100") && !text.contains("not tested"),
            "{text}"
        );
        assert!(
            text.contains("P0171"),
            "a failed code is still listed: {text}"
        );
        let toyota = all.iter_mut().find(|p| p.id == "toyota").unwrap();
        toyota
            .modules
            .iter_mut()
            .find(|m| m.tx == 0x7C0)
            .unwrap()
            .kind = "smart-key".into();
        let fence = icarus_obd::fence::Fence::of(&all);
        elm.take_sent();
        let text = vehicle_modules_on(&mut elm, &all, &fence).unwrap();
        assert!(
            text.contains(
                "7C0 instrument cluster (combination meter): not asked (7C0 (the smart-key module)"
            ),
            "{text}"
        );
        assert!(
            text.contains("7E0 engine (ECM)"),
            "the others as before: {text}"
        );
        assert!(
            !elm.take_sent().iter().any(|(_, tx, _)| *tx == 0x7C0),
            "nothing was sent to the cluster"
        );
    }
}
