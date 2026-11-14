//! `icarus mcp`: Icarus as a Model Context Protocol server (stdio, JSON-RPC 2.0, one message per line), so an assistant
//! (Sarina, through her operator file ~/.sarina/mcp.json) can read a vehicle and explain it.
//!
//! **Read-only, always.** The tools here only read: identity, codes with their meanings, the freeze frame, live values,
//! every module's codes, the tires/gears arithmetic, and the learning sessions Icarus saved while watching another tool. Clearing codes, actuator tests and anything else that changes a
//! vehicle are never offered to a model (a test checks the tool list), as Sarina's own rules keep launching agents
//! CLI-only. The adapter is the one named by ICARUS_ADAPTER in the server's environment, else the one the window last used.

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
            "description": "Read the connected vehicle (OBD adapter): identity, check-engine light, readiness, trouble codes explained, freeze frame, current values. Read-only.",
            "inputSchema": {"type": "object", "properties": {}}
        },
        {
            "name": "vehicle_modules",
            "description": "Every module's own trouble codes (ABS, airbags, transmission, body), explained. Read-only; can take a minute.",
            "inputSchema": {"type": "object", "properties": {}}
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
    use icarus_obd::drivelog;
    let dirs = drivelog::folders();
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
    let port = Target::parse(&spec)?
        .open()
        .map_err(|e| format!("{spec}: {e}"))?;
    Elm::connect(port).map_err(|e| e.to_string())
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

fn vehicle_check() -> Result<String, String> {
    let mut elm = connect()?;
    let all = vehicles::load(vehicles::user_dir().as_deref()).0;
    let d = report::collect(&mut elm, &all, false).map_err(|e| e.to_string())?;
    let mut o = String::new();
    o += &format!("Vehicle: VIN {}", d.vin.as_deref().unwrap_or("not given"));
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
    Ok(o)
}

fn vehicle_modules() -> Result<String, String> {
    let mut elm = connect()?;
    let e = |x: icarus_obd::elm::ObdError| x.to_string();
    let all = vehicles::load(vehicles::user_dir().as_deref()).0;
    let vin = j1979::vin(&mut elm).map_err(e)?;
    let profile = vin
        .as_deref()
        .and_then(|v| vehicles::for_vin(&all, v))
        .cloned()
        .or_else(|| vehicles::find(&all, "generic").ok().cloned());
    let mut o = String::new();
    for m in profile
        .as_ref()
        .map(vehicles::candidates)
        .unwrap_or_default()
    {
        if m.unreachable().is_some() || !uds::present(&mut elm, m.tx, m.rx()).map_err(e)? {
            continue;
        }
        match uds::module_dtcs(&mut elm, m.tx, m.rx()).map_err(e)? {
            None => {
                o += &format!(
                    "{:03X} {}: answered, but would not give its codes.\n",
                    m.tx, m.name
                )
            }
            Some((_, codes)) if codes.is_empty() => {
                o += &format!("{:03X} {}: no codes.\n", m.tx, m.name)
            }
            Some((_, codes)) => {
                o += &format!("{:03X} {}:\n", m.tx, m.name);
                for c in codes {
                    o += &code_line(
                        &c.code,
                        &format!(" ({})", c.states.join(", ")),
                        profile.as_ref(),
                    );
                    o += "\n";
                }
            }
        }
    }
    if o.is_empty() {
        o = "No module answered.".into();
    }
    Ok(o)
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
        "vehicle_check" => vehicle_check(),
        "drive_logs" => drive_logs(args["file"].as_str()),
        "vehicle_modules" => vehicle_modules(),
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
        let r = handle(&json!({"jsonrpc": "2.0", "id": 5, "method": "resources/list"})).unwrap();
        assert_eq!(r["error"]["code"], -32601);
    }
}
