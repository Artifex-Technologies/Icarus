//! `icarus`: the command line of the Icarus tuning tool. Files first (definitions, images, what a tune changed), then the
//! vehicle through an OBD adapter, read-only.

use icarus_core::def::{fmt_num, Definition, Kind};
use icarus_core::rom::{self, Rom};
use icarus_core::xdf;
use icarus_obd::elm::Elm;
use icarus_obd::j1979::{self, DtcKind};
use icarus_obd::port::Target;
use std::io::Write;
use std::process::ExitCode;
use std::time::{Duration, Instant};

const HELP: &str = "icarus: vehicle tuning, learning and diagnostics

Files (nothing is sent to a vehicle):
  icarus def FILE.xdf [--find TEXT]          what a definition describes (with --find: the matching maps)
  icarus show IMAGE.bin FILE.xdf NAME        one map or value from an image, in real units
  icarus diff ORIGINAL.bin TUNED.bin [--def FILE.xdf]
                                             what a tune changed: byte ranges, and with a definition each map, cell by cell

Vehicle, through an OBD adapter (read-only: nothing is cleared or written):
  icarus obd info  --adapter A               adapter, protocol, VIN, modules, calibration IDs, check-engine light, monitors
  icarus obd codes --adapter A               stored, pending and permanent trouble codes, per module
  icarus obd pids  --adapter A               the live values each module offers, and what they mean
  icarus obd live  --adapter A [PID...] [--every MS] [--count N] [--csv FILE]
                                             watch live values (PIDs in hex, e.g. 0C 05 06 07); Ctrl+C stops

  The adapter A: sim (a simulated vehicle), /dev/ttyUSB0[@baud], bt:AA:BB:CC:DD:EE:FF[#channel] (paired Bluetooth, such as
  an OBDLink MX+), tcp:host:port (Wi-Fi). ICARUS_ADAPTER sets a default.";

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let r = match args.first().map(String::as_str) {
        Some("def") => cmd_def(&args[1..]),
        Some("show") => cmd_show(&args[1..]),
        Some("diff") => cmd_diff(&args[1..]),
        Some("obd") => cmd_obd(&args[1..]),
        Some("-h" | "--help" | "help") | None => {
            println!("{HELP}");
            Ok(())
        }
        Some(other) => Err(format!("unknown command {other:?}\n\n{HELP}")),
    };
    match r {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("icarus: {e}");
            ExitCode::FAILURE
        }
    }
}

/// Positional arguments and `--name value` options.
struct Args {
    pos: Vec<String>,
    opts: Vec<(String, String)>,
}

fn parse_args(a: &[String], with_value: &[&str]) -> Result<Args, String> {
    let mut out = Args {
        pos: vec![],
        opts: vec![],
    };
    let mut i = 0;
    while i < a.len() {
        if let Some(name) = a[i].strip_prefix("--") {
            if !with_value.contains(&name) {
                return Err(format!("unknown option --{name}"));
            }
            let v = a.get(i + 1).ok_or(format!("--{name} needs a value"))?;
            out.opts.push((name.to_string(), v.clone()));
            i += 2;
        } else {
            out.pos.push(a[i].clone());
            i += 1;
        }
    }
    Ok(out)
}

impl Args {
    fn opt(&self, name: &str) -> Option<&str> {
        self.opts
            .iter()
            .rev()
            .find(|o| o.0 == name)
            .map(|o| o.1.as_str())
    }
}

fn load_def(path: &str) -> Result<Definition, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
    let p = xdf::parse(&text).map_err(|e| format!("{path}: {e}"))?;
    if !p.skipped.is_empty() {
        eprintln!(
            "note: {} element(s) of {path} were left out (first: {})",
            p.skipped.len(),
            p.skipped[0]
        );
    }
    Ok(p.definition)
}

fn load_image(path: &str, def: Option<&Definition>) -> Result<Vec<u8>, String> {
    let b = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    if let Some(size) = def.and_then(|d| d.image_size) {
        if size != b.len() {
            eprintln!("note: the definition is for a 0x{size:X}-byte image; {path} is 0x{:X} bytes. Check they belong together.", b.len());
        }
    }
    Ok(b)
}

fn kind_name(k: &Kind) -> &'static str {
    match k {
        Kind::Table(t) if t.z.rows > 1 && t.z.cols > 1 => "map",
        Kind::Table(_) => "curve",
        Kind::Constant(_) => "value",
        Kind::Flag(_) => "switch",
    }
}

fn cmd_def(a: &[String]) -> Result<(), String> {
    let args = parse_args(a, &["find"])?;
    let [path] = args.pos.as_slice() else {
        return Err("usage: icarus def FILE.xdf [--find TEXT]".into());
    };
    let def = load_def(path)?;
    if let Some(text) = args.opt("find") {
        for e in def.find(text) {
            println!(
                "{:<7} {}{}",
                kind_name(&e.kind),
                e.title,
                if e.description.is_empty() {
                    String::new()
                } else {
                    format!("  ({})", e.description)
                }
            );
        }
        return Ok(());
    }
    println!(
        "{}",
        if def.title.is_empty() {
            "(untitled definition)"
        } else {
            &def.title
        }
    );
    if !def.description.is_empty() {
        println!("{}", def.description.lines().next().unwrap_or(""));
    }
    let count = |k: &str| {
        def.elements
            .iter()
            .filter(|e| kind_name(&e.kind) == k)
            .count()
    };
    println!(
        "{} maps, {} curves, {} values, {} switches",
        count("map"),
        count("curve"),
        count("value"),
        count("switch")
    );
    let mut cats: Vec<(String, usize)> = Vec::new();
    for c in def.elements.iter().flat_map(|e| e.categories.iter()) {
        match cats.iter_mut().find(|x| &x.0 == c) {
            Some(x) => x.1 += 1,
            None => cats.push((c.clone(), 1)),
        }
    }
    cats.sort();
    if !cats.is_empty() {
        println!("\nCategories:");
        for (c, n) in cats {
            println!("  {c:<32} {n}");
        }
    }
    Ok(())
}

fn print_element(def: &Definition, rom: &Rom, e: &icarus_core::def::Element) -> Result<(), String> {
    println!("{} ({})", e.title, kind_name(&e.kind));
    if !e.description.is_empty() {
        println!("{}", e.description);
    }
    match &e.kind {
        Kind::Constant(d) => {
            let v = def.values(rom, d).map_err(|x| x.to_string())?;
            println!("{} {}", fmt_num(v[0][0], d.decimals), d.units);
        }
        Kind::Flag(f) => println!(
            "{}",
            if def.flag(rom, f).map_err(|x| x.to_string())? {
                "on"
            } else {
                "off"
            }
        ),
        Kind::Table(t) => {
            let v = def.table(rom, t).map_err(|x| x.to_string())?;
            if !t.z.units.is_empty() {
                println!("values in {}", t.z.units);
            }
            let w =
                v.x.iter()
                    .chain(v.y.iter())
                    .map(|s| s.len())
                    .chain(v.z.concat().iter().map(|n| fmt_num(*n, t.z.decimals).len()))
                    .max()
                    .unwrap_or(4)
                    .max(4)
                    + 1;
            print!("{:>w$}", "", w = w);
            for x in &v.x {
                print!("{x:>w$}");
            }
            println!();
            for (r, row) in v.z.iter().enumerate() {
                print!("{:>w$}", v.y.get(r).map(String::as_str).unwrap_or(""));
                for n in row {
                    print!("{:>w$}", fmt_num(*n, t.z.decimals));
                }
                println!();
            }
        }
    }
    Ok(())
}

fn cmd_show(a: &[String]) -> Result<(), String> {
    let args = parse_args(a, &[])?;
    let [image, defp, name] = args.pos.as_slice() else {
        return Err("usage: icarus show IMAGE.bin FILE.xdf NAME".into());
    };
    let def = load_def(defp)?;
    let rom = Rom::new(load_image(image, Some(&def))?);
    let e = def.element(name).map_err(|e| e.to_string())?;
    print_element(&def, &rom, e)
}

fn cmd_diff(a: &[String]) -> Result<(), String> {
    let args = parse_args(a, &["def"])?;
    let [orig, tuned] = args.pos.as_slice() else {
        return Err("usage: icarus diff ORIGINAL.bin TUNED.bin [--def FILE.xdf]".into());
    };
    let def = args.opt("def").map(load_def).transpose()?;
    let (ob, tb) = (
        load_image(orig, def.as_ref())?,
        load_image(tuned, def.as_ref())?,
    );
    let ranges = rom::diff(&ob, &tb);
    if ranges.is_empty() {
        println!("The two images are the same.");
        return Ok(());
    }
    let bytes: usize = ranges.iter().map(|r| r.len()).sum();
    println!("{bytes} byte(s) differ in {} place(s).", ranges.len());
    let Some(def) = def else {
        for r in ranges.iter().take(50) {
            println!("  0x{:06X}..0x{:06X} ({} bytes)", r.start, r.end, r.len());
        }
        return Ok(());
    };
    if ob.len() != tb.len() {
        return Err("the images differ in size, so a cell-by-cell comparison would mislead".into());
    }
    let (or, tr) = (Rom::new(ob), Rom::new(tb));
    let mut seen: Vec<*const icarus_core::def::Element> = Vec::new();
    let mut unnamed = 0usize;
    for r in &ranges {
        let hits = def.covering(r.clone());
        if hits.is_empty() {
            unnamed += r.len();
        }
        for e in hits {
            if seen.contains(&(e as *const _)) {
                continue;
            }
            seen.push(e as *const _);
            println!("\n{} ({})", e.title, kind_name(&e.kind));
            match &e.kind {
                Kind::Flag(f) => {
                    let (a, b) = (
                        def.flag(&or, f).map_err(|x| x.to_string())?,
                        def.flag(&tr, f).map_err(|x| x.to_string())?,
                    );
                    println!(
                        "  {} -> {}",
                        if a { "on" } else { "off" },
                        if b { "on" } else { "off" }
                    );
                }
                Kind::Constant(d) => {
                    let (a, b) = (
                        def.values(&or, d).map_err(|x| x.to_string())?,
                        def.values(&tr, d).map_err(|x| x.to_string())?,
                    );
                    println!(
                        "  {} -> {} {}",
                        fmt_num(a[0][0], d.decimals),
                        fmt_num(b[0][0], d.decimals),
                        d.units
                    );
                }
                Kind::Table(t) => {
                    let (a, b) = (
                        def.table(&or, t).map_err(|x| x.to_string())?,
                        def.table(&tr, t).map_err(|x| x.to_string())?,
                    );
                    if a.x != b.x || a.y != b.y {
                        println!("  the axis breakpoints changed too");
                    }
                    let mut n = 0;
                    for (ri, (ra, rb)) in a.z.iter().zip(&b.z).enumerate() {
                        for (ci, (va, vb)) in ra.iter().zip(rb).enumerate() {
                            if va != vb {
                                n += 1;
                                if n <= 12 {
                                    println!(
                                        "  at {} / {}: {} -> {} {}",
                                        b.y.get(ri).map(String::as_str).unwrap_or("-"),
                                        b.x.get(ci).map(String::as_str).unwrap_or("-"),
                                        fmt_num(*va, t.z.decimals),
                                        fmt_num(*vb, t.z.decimals),
                                        t.z.units
                                    );
                                }
                            }
                        }
                    }
                    if n > 12 {
                        println!("  ... {} cells changed in all", n);
                    }
                }
            }
        }
    }
    if unnamed > 0 {
        println!("\n{unnamed} changed byte(s) are in no element of the definition (code, checksums, or maps it does not describe).");
    }
    Ok(())
}

fn connect(args: &Args) -> Result<Elm, String> {
    let spec = args.opt("adapter").map(str::to_string).or_else(|| std::env::var("ICARUS_ADAPTER").ok()).ok_or(
        "which adapter? --adapter sim | /dev/ttyUSB0 | bt:AA:BB:CC:DD:EE:FF | tcp:host:port (or set ICARUS_ADAPTER)",
    )?;
    let target = Target::parse(&spec)?;
    let port = target.open().map_err(|e| format!("{spec}: {e}"))?;
    eprintln!("connecting to {spec}...");
    Elm::connect(port).map_err(|e| e.to_string())
}

fn cmd_obd(a: &[String]) -> Result<(), String> {
    let args = parse_args(
        a.get(1..).unwrap_or(&[]),
        &["adapter", "every", "count", "csv"],
    )?;
    match a.first().map(String::as_str) {
        Some("info") => obd_info(&mut connect(&args)?),
        Some("codes") => obd_codes(&mut connect(&args)?),
        Some("pids") => obd_pids(&mut connect(&args)?),
        Some("live") => obd_live(&mut connect(&args)?, &args),
        _ => Err("usage: icarus obd info|codes|pids|live --adapter A (see icarus --help)".into()),
    }
}

fn e(x: icarus_obd::elm::ObdError) -> String {
    x.to_string()
}

fn obd_info(elm: &mut Elm) -> Result<(), String> {
    println!(
        "Adapter:   {}{}",
        elm.id,
        elm.stn
            .as_ref()
            .map(|s| format!(" ({s})"))
            .unwrap_or_default()
    );
    if let Some(v) = elm.voltage {
        println!("Battery:   {v:.1} V at the OBD port");
    }
    println!("Protocol:  {}", elm.protocol.describe());
    println!(
        "VIN:       {}",
        j1979::vin(elm)
            .map_err(e)?
            .unwrap_or_else(|| "(not given)".into())
    );
    let names = j1979::ecu_names(elm).map_err(e)?;
    let cals = j1979::calibration_ids(elm).map_err(e)?;
    let cvns = j1979::cvns(elm).map_err(e)?;
    let sup = j1979::supported(elm).map_err(e)?;
    println!("\nModules:");
    for (from, module, _) in &sup {
        let name = names
            .iter()
            .find(|n| &n.0 == from)
            .and_then(|n| n.2.first().cloned())
            .unwrap_or_default();
        println!(
            "  {from}  {module}{}",
            if name.is_empty() {
                String::new()
            } else {
                format!(": {name}")
            }
        );
        if let Some(c) = cals.iter().find(|c| &c.0 == from) {
            println!("        calibration: {}", c.2.join(", "));
        }
        if let Some(c) = cvns.iter().find(|c| &c.0 == from) {
            println!("        verification number (CVN): {}", c.1.join(", "));
        }
    }
    let answers = elm.request(&[0x01, 0x01]).map_err(e)?;
    for a in answers
        .iter()
        .filter(|a| a.data.len() >= 6 && a.data[..2] == [0x41, 0x01])
    {
        let Some(m) = j1979::monitors(&a.data[2..]) else {
            continue;
        };
        println!("\n{} ({}):", a.module(), a.from);
        println!(
            "  check-engine light {}, {} confirmed code(s){}",
            if m.mil { "ON" } else { "off" },
            m.codes,
            if m.diesel { "; diesel monitors" } else { "" }
        );
        for (name, done) in &m.tests {
            println!(
                "  {:<28} {}",
                name,
                if *done {
                    "complete"
                } else {
                    "not yet run since codes were cleared"
                }
            );
        }
    }
    if let Ok(a) = elm.request(&[0x01, 0x03]) {
        if let Some(x) = a
            .iter()
            .find(|a| a.data.len() >= 3 && a.data[..2] == [0x41, 0x03])
        {
            println!("\nFuel system: {}", j1979::fuel_system(x.data[2]));
        }
    }
    Ok(())
}

fn obd_codes(elm: &mut Elm) -> Result<(), String> {
    for (kind, label) in [
        (DtcKind::Stored, "Stored (confirmed)"),
        (DtcKind::Pending, "Pending (seen once)"),
        (DtcKind::Permanent, "Permanent"),
    ] {
        let list = j1979::dtcs(elm, kind).map_err(e)?;
        let any = list.iter().any(|l| !l.2.is_empty());
        println!("{label}: {}", if any { "" } else { "none" });
        for (from, module, codes) in list.iter().filter(|l| !l.2.is_empty()) {
            println!("  {module} ({from}): {}", codes.join(", "));
        }
    }
    println!("\nP0xxx codes mean the same on every vehicle; P1xxx and other x1xx codes are the maker's own (look up its list).");
    Ok(())
}

fn obd_pids(elm: &mut Elm) -> Result<(), String> {
    for (from, module, pids) in j1979::supported(elm).map_err(e)? {
        println!("{module} ({from}):");
        for p in pids {
            match j1979::pid(p) {
                Some(d) => println!("  {p:02X}  {} [{}]\n      {}", d.name, d.unit, d.about),
                None if p == 0x01 => println!("  01  Monitor status\n      The check-engine light, the code count and which emissions self-tests have run (icarus obd info)."),
                None if p == 0x03 => println!("  03  Fuel system status\n      Open loop (tables alone) or closed loop (corrected by the oxygen sensor) (icarus obd info)."),
                None if p % 0x20 != 0 => println!("  {p:02X}  (a value Icarus cannot read yet)"),
                None => {}
            }
        }
    }
    Ok(())
}

fn obd_live(elm: &mut Elm, args: &Args) -> Result<(), String> {
    let mut pids: Vec<u8> = args
        .pos
        .iter()
        .map(|p| {
            u8::from_str_radix(p.trim_start_matches("0x"), 16)
                .map_err(|_| format!("{p} is not a PID in hex"))
        })
        .collect::<Result<_, _>>()?;
    if pids.is_empty() {
        pids = vec![0x0C, 0x05, 0x04, 0x06, 0x07, 0x0E, 0x44];
    }
    for p in &pids {
        if j1979::pid(*p).is_none() {
            return Err(format!(
                "Icarus does not know PID {p:02X} yet (icarus obd pids lists what it knows)"
            ));
        }
    }
    let every = Duration::from_millis(
        args.opt("every")
            .map(|v| v.parse::<u64>().map_err(|_| "--every needs milliseconds"))
            .transpose()?
            .unwrap_or(500),
    );
    let count = args
        .opt("count")
        .map(|v| v.parse::<u64>().map_err(|_| "--count needs a number"))
        .transpose()?;
    let mut csv = args
        .opt("csv")
        .map(|p| std::fs::File::create(p).map_err(|e| format!("{p}: {e}")))
        .transpose()?;
    let names: Vec<String> = pids
        .iter()
        .map(|p| {
            let d = j1979::pid(*p).unwrap();
            format!("{} [{}]", d.name, d.unit)
        })
        .collect();
    println!("time_s, {}", names.join(", "));
    if let Some(f) = csv.as_mut() {
        writeln!(
            f,
            "time_s,{}",
            names
                .iter()
                .map(|n| format!("\"{n}\""))
                .collect::<Vec<_>>()
                .join(",")
        )
        .map_err(|e| e.to_string())?;
    }
    let start = Instant::now();
    let mut n = 0u64;
    loop {
        let t0 = Instant::now();
        let mut row = Vec::new();
        for p in &pids {
            // the engine's answer when it gives one, else the first module's
            let v = match j1979::read(elm, *p) {
                Ok(r) => r
                    .iter()
                    .find(|x| x.from == "7E8")
                    .or(r.first())
                    .map(|x| format!("{:.2}", x.value))
                    .unwrap_or_default(),
                Err(icarus_obd::elm::ObdError::NoData) => String::new(),
                Err(x) => return Err(x.to_string()),
            };
            row.push(v);
        }
        let t = format!("{:.2}", start.elapsed().as_secs_f64());
        println!("{t}, {}", row.join(", "));
        if let Some(f) = csv.as_mut() {
            writeln!(f, "{t},{}", row.join(",")).map_err(|e| e.to_string())?;
        }
        n += 1;
        if count.is_some_and(|c| n >= c) {
            return Ok(());
        }
        if let Some(rest) = every.checked_sub(t0.elapsed()) {
            std::thread::sleep(rest);
        }
    }
}
