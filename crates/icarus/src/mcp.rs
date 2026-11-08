//! `icarus mcp`: Icarus as a Model Context Protocol server (stdio, JSON-RPC 2.0, one message per line), so an assistant
//! (Sarina, through her operator file ~/.sarina/mcp.json) can read a vehicle and explain it.
//!
//! **Read-only, always.** The tools here only read: identity, codes with their meanings, the freeze frame, live values,
//! every module's codes, and the tires/gears arithmetic. Clearing codes, actuator tests and anything else that changes a
//! vehicle are never offered to a model (a test checks the tool list), as Sarina's own rules keep launching agents
//! CLI-only. The adapter is the one named by ICARUS_ADAPTER in the server's environment.

use icarus_obd::elm::Elm;
use icarus_obd::port::Target;
use icarus_obd::{dtc, garage, j1979, report, uds, vehicles};
use serde_json::{json, Value};
use std::io::{BufRead, Write};

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

fn connect() -> Result<Elm, String> {
    let spec = std::env::var("ICARUS_ADAPTER").map_err(|_| {
        "no adapter: set ICARUS_ADAPTER in this server's definition (e.g. bt:AA:BB:CC:DD:EE:FF)"
            .to_string()
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
        "vehicle_modules" => vehicle_modules(),
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
            json!({"protocolVersion": version, "capabilities": {"tools": {}}, "serverInfo": {"name": "icarus", "version": env!("CARGO_PKG_VERSION")}})
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
