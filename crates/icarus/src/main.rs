//! `icarus`: the command line of the Icarus tuning tool. Files first (definitions, images, what a tune changed), then the
//! vehicle through an OBD adapter, read-only.

use icarus_core::def::{fmt_num, Definition, Kind};
use icarus_core::rom::{self, Rom};
use icarus_core::xdf;
use icarus_obd::elm::Elm;
use icarus_obd::j1979::{self, DtcKind};
use icarus_obd::port::Target;
use icarus_obd::{
    actuator, bus, charging, clear, dtc, garage, lookup, mode06, nhtsa, report, settings, sim,
    socketcan, uds, vehicles, vin,
};
mod mcp;

use std::io::Write;
use std::process::ExitCode;
use std::time::{Duration, Instant};

const HELP: &str = "icarus: vehicle tuning, learning and diagnostics

Files (nothing is sent to a vehicle):
  icarus def FILE.xdf [--find TEXT]          what a definition describes (with --find: the matching maps)
  icarus show IMAGE.bin FILE.xdf NAME        one map or value from an image, in real units
  icarus set IMAGE.bin FILE.xdf NAME VALUE [--row R --col C] [--out NEW.bin] [--checksums FILE]
                                             change one value, table cell (from 0) or switch (on/off) and save a NEW file
                                             (never over one), its checksums corrected where they are known
                                             (<definition>.checksums.toml beside the definition, or --checksums)
  icarus checksums IMAGE.bin CHECKSUMS.toml  whether an image's checksums are right
  icarus diff ORIGINAL.bin TUNED.bin [--def FILE.xdf]
                                             what a tune changed: byte ranges, and with a definition each map, cell by cell

Vehicle, through an OBD adapter (read-only: nothing is cleared or written):
  icarus obd info  --adapter A               adapter, protocol, VIN, modules, calibration IDs, check-engine light, monitors
  icarus obd codes --adapter A               stored, pending and permanent trouble codes, per module
  icarus obd pids  --adapter A               the live values each module offers, and what they mean
  icarus obd live  --adapter A [PID...] [--every MS] [--count N] [--csv FILE]
                                             watch live values (PIDs in hex, e.g. 0C 05 06 07); Ctrl+C stops
  icarus obd scan  --adapter A [--make NAME] [--sweep]
                                             find every module (ABS, airbags, cluster...): identity and trouble codes,
                                             as dealer tools do. The make comes from the VIN (or --make, see icarus
                                             vehicles); --sweep tries every ID from 700 to 7F7 (slower)
  icarus obd module ID[/ANSWER] --adapter A  one module (request ID in hex, e.g. 7B0, or 713/77D): identity and codes

Bus (watching only: nothing is sent onto the bus):
  icarus bus watch (--adapter A | --can can0) [--seconds N] [--raw]
                                             every ID with its rate and changing bytes, diagnostic messages in words, and
                                             actuator tests another tool runs, as entries for the vehicle's file
  icarus learn (--adapter A | --can can0) [--seconds N] [--ask-sarina]
                                             learn from another tool working on the vehicle (a Y-cable on the OBD port):
                                             type what you are about to do in it and press Enter (a mark), then do it;
                                             \"stop\" or Ctrl+C ends. The settings and tests it changed come out named by
                                             your marks, saved with everything seen in the vehicle's Logs folder;
                                             --ask-sarina has Sarina read the session and say what she makes of it

Interfaces:
  icarus devices                             the tuners and interfaces plugged in (SCT, COBB, HP Tuners, CP210x/FTDI
                                             cables...), what each is and whether Icarus can open it; nothing is sent

Tools:
  icarus tires STOCK NEW [--gear 3.909] [--speed 65]
                                             what new tires do to the speedometer and gearing, and which regear restores it

For an assistant:
  icarus mcp                                 a read-only MCP server on stdin/stdout (codes explained, a vehicle check,
                                             every module's codes, tires and gears) for Sarina or another assistant; the
                                             adapter comes from ICARUS_ADAPTER. Nothing that clears or tests is offered

Vehicles:
  icarus vin VIN [--nhtsa]                   what a VIN says: where it was built, maker, model year, check digit;
                                             --nhtsa adds model, engine and drive from NHTSA (sends the VIN to NHTSA)
  icarus recalls \"2017 Toyota Tacoma\"        recalls on record at NHTSA (sends only year, make and model)
  icarus vehicles                            the makes Icarus knows (one editable file each)
  icarus garage [VIN \"2017 Toyota Tacoma 3.5L V6\"]
                                             name a vehicle (searches and parts links use it), or list the named ones

Trouble codes:
  icarus dtc CODE... [--vehicle \"2017 Toyota Tacoma 3.5L V6\"] [--make toyota]
                                             what a code means, where to start, the parts usually involved, links to look
                                             it up and to find parts (no vehicle needed)
  icarus obd codes --adapter A               the vehicle's codes, each explained, and the freeze frame
  icarus obd tests --adapter A               actuator tests known for this vehicle (fan, throttle, pumps...)
  icarus obd test NAME --adapter A [--value PERCENT] [--seconds N] [--yes]
                                             run one: asks first, checks its conditions, stops after N seconds or Ctrl+C
                                             and hands control back to the module (sim:off = the simulated engine off)
  icarus obd settings --adapter A            module settings known for this vehicle and their values now (reads only)
  icarus obd setting NAME VALUE --adapter A [--yes]
                                             change one to one of its named values: asks first, needs a wired adapter,
                                             engine off and parked; the value before is kept in the vehicle's Logs folder
                                             and the new one read back
  icarus obd setting-undo NAME --adapter A [--yes]
                                             put it back to what it was before its last change
  icarus obd selftests --adapter A           on-board test results (Mode 06): each monitor's last measurement and limits,
                                             misfire counts per cylinder
  icarus obd charging --adapter A [--battery agm|flooded] [--seconds 20]
                                             battery and charging voltage judged for the battery fitted (engine running:
                                             charging; engine off after sitting: state of charge)
  icarus obd clear --adapter A [--module ID] [--yes]
                                             clear codes (every OBD-II module, or one): saves a report first, engine off
  icarus obd report --adapter A [--scan] [--out FILE]
                                             everything in one Markdown report (~/Documents/Icarus/reports); --scan adds
                                             every module

  The adapter A: sim (a simulated vehicle), /dev/ttyUSB0[@baud], bt:AA:BB:CC:DD:EE:FF[#channel] (paired Bluetooth, such as
  an OBDLink MX+), tcp:host:port (Wi-Fi). ICARUS_ADAPTER sets a default.";

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let r = match args.first().map(String::as_str) {
        Some("def") => cmd_def(&args[1..]),
        Some("show") => cmd_show(&args[1..]),
        Some("diff") => cmd_diff(&args[1..]),
        Some("set") => cmd_set(&args[1..]),
        Some("checksums") => cmd_checksums(&args[1..]),
        Some("obd") => cmd_obd(&args[1..]),
        Some("vin") => cmd_vin(&args[1..]),
        Some("vehicles") => cmd_vehicles(),
        Some("dtc") => cmd_dtc(&args[1..]),
        Some("garage") => cmd_garage(&args[1..]),
        Some("bus") => cmd_bus(&args[1..]),
        Some("learn") => cmd_learn(&args[1..]),
        Some("tires") => cmd_tires(&args[1..]),
        Some("recalls") => cmd_recalls(&args[1..]),
        Some("mcp") => mcp::serve(),
        Some("devices") => cmd_devices(),
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
        if a[i] == "--sweep" || a[i] == "--scan" || a[i] == "--yes" {
            out.opts.push((a[i][2..].to_string(), "yes".into()));
            i += 1;
            continue;
        }
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

/// The checksum descriptions for a definition: --checksums FILE, else <definition>.checksums.toml beside it, else none.
fn load_checksums(
    defp: &str,
    given: Option<&str>,
    size: usize,
) -> Result<Vec<icarus_core::checksum::Spec>, String> {
    let path = match given {
        Some(p) => std::path::PathBuf::from(p),
        None => {
            let p = icarus_core::checksum::file_for(std::path::Path::new(defp));
            if !p.exists() {
                return Ok(vec![]);
            }
            p
        }
    };
    let text = std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    icarus_core::checksum::parse(&text, size).map_err(|e| format!("{}: {e}", path.display()))
}

fn cmd_set(a: &[String]) -> Result<(), String> {
    let args = parse_args(a, &["row", "col", "out", "checksums"])?;
    let [image, defp, name, value] = args.pos.as_slice() else {
        return Err("usage: icarus set IMAGE.bin FILE.xdf NAME VALUE [--row R --col C] [--out NEW.bin] [--checksums FILE]".into());
    };
    let def = load_def(defp)?;
    let mut rom = Rom::new(load_image(image, Some(&def))?);
    let specs = load_checksums(defp, args.opt("checksums"), rom.len())?;
    let e = def.element(name).map_err(|e| e.to_string())?;
    let num = |k: &str| {
        args.opt(k)
            .map(|v| {
                v.parse::<usize>()
                    .map_err(|_| format!("--{k} is a number (from 0)"))
            })
            .transpose()
    };
    match &e.kind {
        Kind::Flag(f) => {
            let on = match value.to_lowercase().as_str() {
                "on" | "1" | "true" | "yes" => true,
                "off" | "0" | "false" | "no" => false,
                _ => return Err("a switch is on or off".into()),
            };
            def.set_flag(&mut rom, f, on).map_err(|e| e.to_string())?;
            println!("{}: {}", e.title, if on { "on" } else { "off" });
        }
        Kind::Constant(d) | Kind::Table(icarus_core::def::Table { z: d, .. }) => {
            let v: f64 = value
                .parse()
                .map_err(|_| format!("{value:?} is not a number"))?;
            let (row, col) = (num("row")?.unwrap_or(0), num("col")?.unwrap_or(0));
            if matches!(e.kind, Kind::Table(_))
                && (args.opt("row").is_none() || args.opt("col").is_none())
            {
                return Err(format!("{} is a table: say which cell with --row and --col (from 0; {} rows, {} columns)", e.title, d.rows, d.cols));
            }
            let before = def.values(&rom, d).map_err(|e| e.to_string())?
                [row.min(d.rows.saturating_sub(1))][col.min(d.cols.saturating_sub(1))];
            let stored = def
                .set(&mut rom, d, row, col, v)
                .map_err(|e| e.to_string())?;
            println!(
                "{} [{row}, {col}]: {} -> {} {}",
                e.title,
                fmt_num(before, d.decimals),
                fmt_num(stored, d.decimals),
                d.units
            );
            if (stored - v).abs() > 1e-9 {
                println!(
                    "  (stored as {}: the nearest value the cell can hold)",
                    fmt_num(stored, d.decimals)
                );
            }
        }
    }
    let out = args
        .opt("out")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| icarus_core::edit::default_path(std::path::Path::new(image)));
    let saved = icarus_core::edit::save(&rom, &specs, &out)?;
    println!("{}", saved.summary());
    Ok(())
}

fn cmd_checksums(a: &[String]) -> Result<(), String> {
    let args = parse_args(a, &[])?;
    let [image, file] = args.pos.as_slice() else {
        return Err("usage: icarus checksums IMAGE.bin CHECKSUMS.toml   (the descriptions: crates/icarus-core/src/checksum.rs)".into());
    };
    let bytes = std::fs::read(image).map_err(|e| format!("{image}: {e}"))?;
    let text = std::fs::read_to_string(file).map_err(|e| format!("{file}: {e}"))?;
    let specs = icarus_core::checksum::parse(&text, bytes.len())?;
    if specs.is_empty() {
        println!("No checksums described in {file}.");
    }
    for (s, st) in specs
        .iter()
        .zip(icarus_core::checksum::check(&specs, &bytes, &bytes))
    {
        let w = s.algorithm.width() * 2;
        println!(
            "{:<28} stored 0x{:0w$X}, computed 0x{:0w$X}: {}{}",
            s.name,
            st.stored,
            st.computed,
            if st.ok() { "right" } else { "WRONG" },
            if s.checked {
                ""
            } else {
                " (description not yet checked on a real image)"
            }
        );
    }
    Ok(())
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
        &[
            "adapter", "every", "count", "csv", "make", "out", "value", "seconds", "module",
            "battery",
        ],
    )?;
    match a.first().map(String::as_str) {
        Some("info") => obd_info(&mut connect(&args)?),
        Some("codes") => obd_codes(&mut connect(&args)?),
        Some("pids") => obd_pids(&mut connect(&args)?),
        Some("live") => obd_live(&mut connect(&args)?, &args),
        Some("scan") => obd_scan(&mut connect(&args)?, &args),
        Some("module") => obd_module(&mut connect(&args)?, &args),
        Some("report") => obd_report(&mut connect(&args)?, &args),
        Some("tests") => obd_tests(&mut connect(&args)?),
        Some("clear") => obd_clear(&mut connect(&args)?, &args),
        Some("selftests") => obd_selftests(&mut connect(&args)?),
        Some("charging") => obd_charging(&mut connect(&args)?, &args),
        Some("test") => obd_test(&mut connect(&args)?, &args),
        Some("settings") => obd_settings(&mut connect(&args)?),
        Some("setting") => obd_setting(&args, false),
        Some("setting-undo") => obd_setting(&args, true),
        _ => Err(
            "usage: icarus obd info|codes|pids|live|scan|module --adapter A (see icarus --help)"
                .into(),
        ),
    }
}

fn print_module(elm: &mut Elm, m: &vehicles::ModuleAddr) -> Result<(), String> {
    let (tx, rx) = (m.tx, m.rx());
    let at = if rx == tx + 8 {
        format!("{tx:03X}")
    } else {
        format!("{tx:03X}/{rx:03X}")
    };
    println!(
        "\n{at}  {}{}",
        m.name,
        if m.checked {
            ""
        } else {
            "  (address not yet checked on a vehicle)"
        }
    );
    for (what, value) in uds::module_identity(elm, tx, rx).map_err(e)? {
        println!("     {what:<17} {value}");
    }
    match uds::module_dtcs(elm, tx, rx).map_err(e)? {
        None => println!("     trouble codes     (the module would not say)"),
        Some((dialect, codes)) => {
            let lang = if dialect == uds::Dialect::Uds {
                "UDS"
            } else {
                "KWP2000"
            };
            if codes.is_empty() {
                println!("     trouble codes     none ({lang})");
            }
            for c in codes {
                let ftb = c
                    .failure_type
                    .map(|f| format!("-{f:02X}"))
                    .unwrap_or_default();
                println!(
                    "     trouble code      {}{ftb}  {} ({lang})",
                    c.code,
                    c.states.join(", ")
                );
            }
        }
    }
    Ok(())
}

fn profiles() -> Vec<vehicles::Profile> {
    let (all, problems) = vehicles::load(vehicles::user_dir().as_deref());
    for p in problems {
        eprintln!("note: a vehicle file was left out: {p}");
    }
    all
}

fn obd_scan(elm: &mut Elm, args: &Args) -> Result<(), String> {
    let all = profiles();
    let profile = match args.opt("make") {
        Some(m) => vehicles::find(&all, m)?.clone(),
        None => {
            let vin = j1979::vin(elm).map_err(e)?;
            match vin.as_deref().and_then(|v| vehicles::for_vin(&all, v)) {
                Some(p) => p.clone(),
                None => {
                    eprintln!("note: no vehicle profile for this VIN ({}); scanning the OBD-II range only (--make or --sweep look further)", vin.as_deref().unwrap_or("none given"));
                    vehicles::find(&all, "generic")?.clone()
                }
            }
        }
    };
    let list = if args.opt("sweep").is_some() {
        uds::sweep()
    } else {
        vehicles::candidates(&profile)
    };
    println!("Vehicle profile: {} ({})", profile.name, profile.source);
    if !profile.notes.is_empty() {
        println!(
            "{}",
            profile
                .notes
                .split_whitespace()
                .collect::<Vec<_>>()
                .join(" ")
        );
    }
    eprintln!("looking at {} addresses...", list.len());
    let (mut found, mut later) = (0, Vec::new());
    for m in &list {
        if let Some(why) = m.unreachable() {
            later.push(format!("{:03X}  {}: {why}", m.tx, m.name));
            continue;
        }
        if uds::present(elm, m.tx, m.rx()).map_err(e)? {
            found += 1;
            print_module(elm, m)?;
        }
    }
    println!("\n{found} module(s) answered. Codes: P powertrain, C chassis (ABS, steering), B body (airbags, lights), U network.");
    if !later.is_empty() {
        println!("Not reachable yet:\n  {}", later.join("\n  "));
    }
    Ok(())
}

fn obd_module(elm: &mut Elm, args: &Args) -> Result<(), String> {
    let [id] = args.pos.as_slice() else {
        return Err(
            "usage: icarus obd module ID[/ANSWER_ID] --adapter A (hex, e.g. 7B0 or 713/77D)".into(),
        );
    };
    let (t, r) = id
        .split_once('/')
        .map_or((id.as_str(), None), |(a, b)| (a, Some(b)));
    let hex = |s: &str| {
        u16::from_str_radix(s.trim_start_matches("0x"), 16)
            .map_err(|_| format!("{s} is not a module ID in hex"))
    };
    let tx = hex(t)?;
    let rx = r.map(hex).transpose()?;
    if !(0x700..=0x7F7).contains(&tx) || tx == 0x7DF {
        return Err("module request IDs are 700-7F7 (7DF is the broadcast)".into());
    }
    let m = vehicles::ModuleAddr {
        tx,
        rx,
        name: "module".into(),
        ext: None,
        bus: None,
        checked: false,
    };
    if !uds::present(elm, tx, m.rx()).map_err(e)? {
        return Err(format!("nothing answered at {tx:03X}"));
    }
    print_module(elm, &m)
}

fn cmd_vin(a: &[String]) -> Result<(), String> {
    let online = a.iter().any(|x| x == "--nhtsa");
    let rest: Vec<&String> = a.iter().filter(|x| *x != "--nhtsa").collect();
    let [v] = rest.as_slice() else {
        return Err("usage: icarus vin VIN [--nhtsa]".into());
    };
    let i = vin::decode(v)?;
    let all = profiles();
    println!("VIN          {}", i.vin);
    println!("made in      {} (position 1)", i.region);
    println!(
        "maker code   {} (positions 1-3){}",
        i.wmi,
        vehicles::for_vin(&all, &i.vin)
            .map(|p| format!(": {}", p.name))
            .unwrap_or_default()
    );
    println!(
        "vehicle      {} (positions 4-8: model, body, engine; the maker's own code)",
        &i.vin[3..8]
    );
    println!(
        "check digit  {} (position 9){}",
        &i.vin[8..9],
        match i.check_digit_ok {
            Some(true) => ": adds up",
            Some(false) => ": does NOT add up",
            None => "",
        }
    );
    println!(
        "model year   {} (position 10)",
        i.year.map_or("-".to_string(), |y| y.to_string())
    );
    println!("plant        {} (position 11)", &i.vin[10..11]);
    println!("serial       {} (positions 12-17)", &i.vin[11..]);
    for p in i.problems {
        println!("note: {p}");
    }
    if online {
        eprintln!("asking NHTSA (this sends the VIN to NHTSA)...");
        let d = nhtsa::decode_vin(&i.vin)?;
        println!("\nNHTSA: {}", d.name());
        for (k, val) in [
            ("trim", &d.trim),
            ("drive", &d.drive),
            ("body", &d.body),
            ("fuel", &d.fuel),
            ("built in", &d.plant_country),
        ] {
            if !val.is_empty() {
                println!("  {k:<9} {val}");
            }
        }
        if let Some(n) = &d.note {
            println!("  note: {n}");
        }
        println!("Name it: icarus garage {} \"{}\"", i.vin, d.name());
    }
    Ok(())
}

fn cmd_vehicles() -> Result<(), String> {
    for p in profiles() {
        let n = vehicles::candidates(&p).len();
        println!("{:<12} {}  [{}]", p.id, p.name, p.source);
        if !p.wmi.is_empty() {
            println!(
                "             VIN starts {}; {n} module addresses",
                p.wmi.join(" ")
            );
        }
    }
    println!(
        "\nAdd or correct a make: copy one of crates/icarus-obd/vehicles/*.toml into {}.",
        vehicles::user_dir().map_or("~/.config/icarus/vehicles".into(), |d| d
            .display()
            .to_string())
    );
    Ok(())
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

/// One code: its meaning, where to start, the parts usually involved, and links to look it up.
fn print_code(
    code: &str,
    states: &str,
    profile: Option<&vehicles::Profile>,
    vehicle: Option<&garage::Described>,
    indent: &str,
) {
    match dtc::explain(code, profile) {
        Err(e) => println!("{indent}{code}: {e}"),
        Ok(i) => {
            let text = i.text.clone().unwrap_or_else(|| {
                if i.standard {
                    "(Icarus has no words for this code yet)".into()
                } else {
                    "the maker's own code: look it up for this make".into()
                }
            });
            println!(
                "{indent}{code}{}  {text}",
                if states.is_empty() {
                    String::new()
                } else {
                    format!(" ({states})")
                }
            );
            if let Some(h) = i.hint {
                println!("{indent}    where to start: {h}");
            }
            let parts = dtc::parts(code);
            if !parts.is_empty() {
                println!("{indent}    parts usually involved: {}", parts.join(", "));
            }
            println!(
                "{indent}    look it up: {}",
                lookup::code_search(code, i.text.as_deref(), vehicle)
            );
            if let Some(p) = parts.first() {
                println!(
                    "{indent}    find parts: {}",
                    lookup::part_search(p, vehicle)
                );
            }
        }
    }
}

fn obd_codes(elm: &mut Elm) -> Result<(), String> {
    let all = profiles();
    let vin = j1979::vin(elm).map_err(e)?;
    let profile = vin.as_deref().and_then(|v| vehicles::for_vin(&all, v));
    let vehicle = vin.as_deref().and_then(garage::get);
    if vehicle.is_none() {
        if let Some(v) = &vin {
            println!("(Name this vehicle once for better searches and parts links: icarus garage {v} \"2017 Toyota Tacoma 3.5L V6\")\n");
        }
    }
    for (kind, label) in [
        (DtcKind::Stored, "Stored (confirmed)"),
        (DtcKind::Pending, "Pending (seen once)"),
        (DtcKind::Permanent, "Permanent"),
    ] {
        let list = j1979::dtcs(elm, kind).map_err(e)?;
        let any = list.iter().any(|l| !l.2.is_empty());
        println!("{label}: {}", if any { "" } else { "none" });
        for (from, module, codes) in list.iter().filter(|l| !l.2.is_empty()) {
            println!("  {module} ({from}):");
            for c in codes {
                print_code(c, "", profile, vehicle.as_ref(), "    ");
            }
        }
    }
    for f in j1979::freeze_frame(elm).map_err(e)? {
        println!(
            "\nFreeze frame, {} ({}){}: what the engine was doing when the code set",
            f.module,
            f.from,
            f.code
                .as_ref()
                .map(|c| format!(", stored by {c}"))
                .unwrap_or_default()
        );
        for (p, v) in &f.values {
            if let Some(x) = j1979::pid(*p) {
                println!("  {:<36} {v:.1} {}", x.name, x.unit);
            }
        }
    }
    println!("\nicarus obd scan reads every module's own codes (ABS, airbags, body...); icarus obd report saves all of it.");
    Ok(())
}

fn vehicle_tests(elm: &mut Elm) -> Result<Vec<actuator::TestDef>, String> {
    let all = profiles();
    let vin = j1979::vin(elm).map_err(e)?;
    let profile = vin.as_deref().and_then(|v| vehicles::for_vin(&all, v));
    Ok(vehicles::tests_for(profile, vin.as_deref()))
}

fn obd_tests(elm: &mut Elm) -> Result<(), String> {
    let tests = vehicle_tests(elm)?;
    if tests.is_empty() {
        println!("No actuator tests are known for this vehicle yet: their IDs are the maker's own. They go in the vehicle's profile as [[test]] entries (docs/PLAN.md).");
    }
    for t in &tests {
        let conds: Vec<&str> = t.conditions.iter().map(|c| c.describe()).collect();
        println!(
            "{}  ({:03X}, {:?}, up to {} s{})",
            t.name,
            t.module,
            t.protocol,
            t.max_seconds,
            if t.checked {
                ""
            } else {
                ", IDs not yet checked on a vehicle"
            }
        );
        if !conds.is_empty() {
            println!("    needs: {}", conds.join(", "));
        }
        if !t.warning.is_empty() {
            println!("    {}", t.warning);
        }
    }
    Ok(())
}

fn obd_test(elm: &mut Elm, args: &Args) -> Result<(), String> {
    let [name] = args.pos.as_slice() else {
        return Err(
            "usage: icarus obd test \"NAME\" --adapter A [--value PERCENT] [--seconds N] [--yes]"
                .into(),
        );
    };
    let tests = vehicle_tests(elm)?;
    let n = name.to_lowercase();
    let hits: Vec<&actuator::TestDef> = tests
        .iter()
        .filter(|t| t.name.to_lowercase().contains(&n))
        .collect();
    let t = match hits.as_slice() {
        [t] => *t,
        [] => {
            return Err(format!(
                "no test matches {name:?} (icarus obd tests lists them)"
            ))
        }
        _ => {
            return Err(format!(
                "{name:?} matches {} tests; be more exact",
                hits.len()
            ))
        }
    };
    let value: f64 = match (t.control, args.opt("value")) {
        (actuator::Control::Percent, Some(v)) => {
            v.parse().map_err(|_| "--value needs a percentage")?
        }
        (actuator::Control::Percent, None) => {
            return Err("this test needs --value (0-100 %)".into())
        }
        _ => 0.0,
    };
    let secs: u32 = args
        .opt("seconds")
        .map(|v| v.parse().map_err(|_| "--seconds needs a number"))
        .transpose()?
        .unwrap_or(3)
        .clamp(1, t.max_seconds);
    println!(
        "{} on module {:03X} for {secs} s{}",
        t.name,
        t.module,
        if t.control == actuator::Control::Percent {
            format!(", set to {value} %")
        } else {
            String::new()
        }
    );
    if !t.warning.is_empty() {
        println!("WARNING: {}", t.warning);
    }
    println!("Vehicle parked, in Park or neutral with the parking brake on. Ctrl+C stops the test at once.");
    if args.opt("yes").is_none() {
        print!("Type yes to start: ");
        std::io::stdout().flush().ok();
        let mut line = String::new();
        std::io::stdin()
            .read_line(&mut line)
            .map_err(|x| x.to_string())?;
        if line.trim() != "yes" {
            return Err("not started".into());
        }
    }
    actuator::catch_interrupt();
    let mut run = actuator::start(elm, t, value)?;
    println!("running...");
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(secs as u64);
    let why = loop {
        if actuator::interrupted() {
            break "stopped with Ctrl+C".to_string();
        }
        if std::time::Instant::now() >= deadline {
            break "done".to_string();
        }
        if let Err(x) = actuator::keep(elm, &mut run) {
            break x;
        }
        std::thread::sleep(std::time::Duration::from_millis(100));
    };
    actuator::stop(elm, &mut run, &why)?;
    println!("Stopped ({why}); control is back with the module.");
    Ok(())
}

fn vehicle_settings(elm: &mut Elm) -> Result<(Option<String>, Vec<settings::SettingDef>), String> {
    let all = profiles();
    let vin = j1979::vin(elm).map_err(e)?;
    let profile = vin.as_deref().and_then(|v| vehicles::for_vin(&all, v));
    let list = vehicles::settings_for(profile, vin.as_deref());
    Ok((vin, list))
}

/// Where a vehicle's setting changes are kept: its own folder's Logs (the simulation's in Icarus's state folder).
fn obd_settings(elm: &mut Elm) -> Result<(), String> {
    let (_, list) = vehicle_settings(elm)?;
    if list.is_empty() {
        println!("No module settings are known for this vehicle yet: their IDs are the maker's own. Watch a dealer tool change one (icarus bus), then add what it learned to the vehicle's profile as a [[setting]] entry (docs/PLAN.md).");
    }
    for s in &list {
        let now = match settings::read(elm, s) {
            Ok((v, _)) => s.show(&v),
            Err(x) => format!("could not read it: {x}"),
        };
        println!(
            "{}  ({:03X}, {}{})",
            s.name,
            s.module,
            s.protocol.label(),
            if s.checked {
                ""
            } else {
                ", not yet checked on a vehicle"
            }
        );
        println!(
            "    now: {now}   values: {}",
            s.values
                .keys()
                .map(String::as_str)
                .collect::<Vec<_>>()
                .join(", ")
        );
        if !s.note.is_empty() {
            println!("    {}", s.note);
        }
    }
    Ok(())
}

fn obd_setting(args: &Args, undo: bool) -> Result<(), String> {
    let (name, value) = match (undo, args.pos.as_slice()) {
        (false, [n, v]) => (n, Some(v)),
        (true, [n]) => (n, None),
        _ => {
            return Err(if undo {
                "usage: icarus obd setting-undo \"NAME\" --adapter A [--yes]"
            } else {
                "usage: icarus obd setting \"NAME\" VALUE --adapter A [--yes]  (VALUE: one of the names icarus obd settings lists)"
            }
            .into())
        }
    };
    let spec = args
        .opt("adapter")
        .map(str::to_string)
        .or_else(|| std::env::var("ICARUS_ADAPTER").ok())
        .unwrap_or_default();
    if let Ok(t) = Target::parse(&spec) {
        if !t.wired() {
            return Err("changing a setting writes the module's memory: use a wired adapter (USB), not Bluetooth or Wi-Fi".into());
        }
    }
    let mut elm = connect(args)?;
    let (vin, list) = vehicle_settings(&mut elm)?;
    let n = name.to_lowercase();
    let hits: Vec<&settings::SettingDef> = match list.iter().find(|s| s.name.to_lowercase() == n) {
        Some(s) => vec![s],
        None => list
            .iter()
            .filter(|s| s.name.to_lowercase().contains(&n))
            .collect(),
    };
    let s = match hits.as_slice() {
        [s] => *s,
        [] => {
            return Err(format!(
                "no setting matches {name:?} (icarus obd settings lists them)"
            ))
        }
        _ => {
            return Err(format!(
                "{name:?} matches {} settings; be more exact",
                hits.len()
            ))
        }
    };
    let log = settings::log_for(vin.as_deref())?;
    let target = match value {
        Some(v) => s.values.get(v.as_str()).cloned().ok_or_else(|| {
            format!(
                "{} takes {} (a value not seen on the vehicle goes into its profile first)",
                s.name,
                s.values
                    .keys()
                    .map(String::as_str)
                    .collect::<Vec<_>>()
                    .join(", ")
            )
        })?,
        None => {
            settings::last_change(&log, s)
                .ok_or_else(|| format!("no change of {} in {}", s.name, log.display()))?
                .before
        }
    };
    let (now, _) = settings::read(&mut elm, s)?;
    println!(
        "{} on module {:03X}: {} -> {}",
        s.name,
        s.module,
        s.show(&now),
        s.show(&target)
    );
    if now == target {
        println!("Already so: nothing to write.");
        return Ok(());
    }
    settings::ready(&mut elm, s)?;
    if !s.checked {
        println!("WARNING: not yet checked on a vehicle. {}", s.note);
    }
    println!("Ignition on, engine off, parked; a charger on the battery if you have one. The value there now is kept in {} first.", log.display());
    if args.opt("yes").is_none() {
        print!("Type yes to change it: ");
        std::io::stdout().flush().ok();
        let mut line = String::new();
        std::io::stdin()
            .read_line(&mut line)
            .map_err(|x| x.to_string())?;
        if line.trim() != "yes" {
            return Err("not changed".into());
        }
    }
    match settings::change(&mut elm, s, &target, &log)? {
        None => println!("Already so: nothing written."),
        Some(c) => println!(
            "Changed: {} -> {} (read back from the module). The way back: icarus obd setting-undo {:?}",
            s.show(&c.before),
            s.show(&c.after),
            s.name
        ),
    }
    Ok(())
}

fn obd_charging(elm: &mut Elm, args: &Args) -> Result<(), String> {
    let battery = match args.opt("battery") {
        Some(b) => charging::Battery::parse(b).ok_or("--battery is agm or flooded")?,
        None => charging::Battery::Agm,
    };
    let secs: u64 = args
        .opt("seconds")
        .map(|v| v.parse().map_err(|_| "--seconds needs a number"))
        .transpose()?
        .unwrap_or(20);
    eprintln!("measuring for {secs} s...");
    let c = charging::check(elm, secs, battery)?;
    println!(
        "{} ({} samples: {:.2} V average, {:.2} to {:.2} V; engine {})",
        c.verdict,
        c.samples,
        c.avg,
        c.min,
        c.max,
        if c.running { "running" } else { "off" }
    );
    Ok(())
}

fn obd_selftests(elm: &mut Elm) -> Result<(), String> {
    let r = mode06::read(elm).map_err(e)?;
    if r.is_empty() {
        println!("The engine module gave no on-board test results.");
    }
    let mut last = None;
    for t in &r {
        if last != Some(t.mid) {
            println!("\n{}", mode06::monitor_name(t.mid));
            last = Some(t.mid);
        }
        println!(
            "  {:<46} {:<4} {}",
            mode06::test_name(t.mid, t.tid),
            if t.passed() { "pass" } else { "FAIL" },
            report::selftest_values(t)
        );
    }
    println!("\nEach result is the module's own last measurement against its limits: a value near a limit shows a part on its way out before a code sets.");
    Ok(())
}

fn obd_clear(elm: &mut Elm, args: &Args) -> Result<(), String> {
    // one module (its dialect is learned by reading its codes first) or every OBD-II module
    let target = match args.opt("module") {
        None => None,
        Some(id) => {
            let (t, r) = id.split_once('/').map_or((id, None), |(a, b)| (a, Some(b)));
            let hex = |s: &str| {
                u16::from_str_radix(s.trim_start_matches("0x"), 16)
                    .map_err(|_| format!("{s} is not a module ID in hex"))
            };
            let tx = hex(t)?;
            let rx = r.map(hex).transpose()?.unwrap_or(tx + 8);
            let Some((d, codes)) = uds::module_dtcs(elm, tx, rx).map_err(e)? else {
                return Err(format!("{tx:03X} did not give its codes"));
            };
            println!(
                "{tx:03X} has {} code(s): {}",
                codes.len(),
                codes
                    .iter()
                    .map(|c| c.code.as_str())
                    .collect::<Vec<_>>()
                    .join(", ")
            );
            Some((tx, rx, d))
        }
    };
    println!("{}", clear::WARNING);
    if args.opt("yes").is_none() {
        print!("Type yes to save a report and clear: ");
        std::io::stdout().flush().ok();
        let mut line = String::new();
        std::io::stdin()
            .read_line(&mut line)
            .map_err(|x| x.to_string())?;
        if line.trim() != "yes" {
            return Err("not cleared".into());
        }
    }
    let d = report::collect(elm, &profiles(), target.is_some()).map_err(e)?;
    let saved = report::save(&d, None)
        .map_err(|x| format!("not cleared: the report could not be saved ({x})"))?;
    println!("Report saved first: {}", saved.display());
    match target {
        None => println!("Cleared: {}", clear::clear_obd(elm)?.join(", ")),
        Some((tx, rx, d)) => {
            clear::clear_module(elm, tx, rx, d)?;
            println!("Cleared module {tx:03X}.");
        }
    }
    Ok(())
}

fn obd_report(elm: &mut Elm, args: &Args) -> Result<(), String> {
    let d = report::collect(elm, &profiles(), args.opt("scan").is_some()).map_err(e)?;
    let p = report::save(&d, args.opt("out").map(std::path::PathBuf::from))
        .map_err(|x| x.to_string())?;
    println!("Report saved: {}", p.display());
    Ok(())
}

fn cmd_dtc(a: &[String]) -> Result<(), String> {
    let args = parse_args(a, &["vehicle", "make"])?;
    if args.pos.is_empty() {
        return Err(
            "usage: icarus dtc CODE... [--vehicle \"2017 Toyota Tacoma 3.5L V6\"] [--make toyota]"
                .into(),
        );
    }
    let all = profiles();
    let vehicle = args.opt("vehicle").map(garage::Described::parse);
    let profile = match args.opt("make") {
        Some(m) => Some(vehicles::find(&all, m)?),
        None => vehicle
            .as_ref()
            .and_then(|v| vehicles::find(&all, &v.make).ok()),
    };
    for code in &args.pos {
        let i = dtc::explain(code, profile).map_err(|x| x.to_string())?;
        println!("{}", dtc::anatomy(&i));
        print_code(code, "", profile, vehicle.as_ref(), "");
        if let Some(url) = vehicle.as_ref().and_then(lookup::parts_catalog) {
            println!("    parts catalog: {url}");
        }
        println!();
    }
    Ok(())
}

fn cmd_bus(a: &[String]) -> Result<(), String> {
    if a.first().map(String::as_str) != Some("watch") {
        return Err(
            "usage: icarus bus watch (--adapter A | --can can0) [--seconds N] [--raw]".into(),
        );
    }
    let args = parse_args(&a[1..], &["adapter", "can", "seconds"])?;
    let raw = a.iter().any(|x| x == "--raw");
    let secs: f64 = args
        .opt("seconds")
        .map(|v| v.parse().map_err(|_| "--seconds needs a number"))
        .transpose()?
        .unwrap_or(10.0);
    actuator::catch_interrupt();
    let mut an = bus::Analyzer::default();
    let start = std::time::Instant::now();
    let mut printed = 0u64;
    let mut step = |an: &mut bus::Analyzer, f: Option<bus::Frame>| {
        if let Some(f) = f {
            if raw {
                println!(
                    "{:8.3}  {:>8X}  {}",
                    f.at,
                    f.id,
                    f.data
                        .iter()
                        .map(|b| format!("{b:02X}"))
                        .collect::<Vec<_>>()
                        .join(" ")
                );
            }
            an.add(&f);
        }
        let new = (an.logged - printed).min(an.log.len() as u64) as usize;
        for l in an.log.iter().skip(an.log.len() - new) {
            println!("{l}");
        }
        printed = an.logged;
        start.elapsed().as_secs_f64() < secs && !actuator::interrupted()
    };
    eprintln!("watching for {secs} s (Ctrl+C stops); nothing is sent onto the bus...");
    let ended = if let Some(ifn) = args.opt("can") {
        let sock = socketcan::open(ifn).map_err(|x| x.to_string())?;
        loop {
            let f = sock
                .read(
                    std::time::Duration::from_millis(100),
                    start.elapsed().as_secs_f64(),
                )
                .map_err(|x| x.to_string())?;
            if !step(&mut an, f) {
                break "stopped".to_string();
            }
        }
    } else {
        let mut elm = connect(&args)?;
        let proto = elm.protocol;
        elm.monitor(|line| {
            let f = bus::parse_line(line, proto, start.elapsed().as_secs_f64());
            step(&mut an, f)
        })
        .map_err(e)?
    };
    let now = start.elapsed().as_secs_f64();
    println!(
        "\n{} frames, {} IDs ({ended}). Bytes that changed in the last second are marked *:",
        an.frames,
        an.ids.len()
    );
    for (id, s) in &an.ids {
        let bytes: Vec<String> = s
            .data
            .iter()
            .enumerate()
            .map(|(i, b)| {
                format!(
                    "{b:02X}{}",
                    if now - s.changed_at.get(i).copied().unwrap_or(0.0) < 1.0 && s.count > 1 {
                        "*"
                    } else {
                        " "
                    }
                )
            })
            .collect();
        let shown = if *id > 0x7FF {
            format!("{id:08X}")
        } else {
            format!("{id:03X}")
        };
        println!(
            "  {shown:>8}  {:7.1} Hz  {:>7}  {}",
            s.hz(),
            s.count,
            bytes.join(" ")
        );
    }
    if !an.settings.is_empty() {
        println!("\nSettings another tool changed (add them to the vehicle's profile, then find out what each one is):\n");
        for s in &an.settings {
            println!("{}", settings::to_toml(s));
        }
    }
    if !an.learned.is_empty() {
        println!("\nTests seen run by another tool (add them to the vehicle's file, ~/.config/icarus/vehicles/MAKE.toml, after checking what each drives):\n");
        for t in &an.learned {
            println!("{}", bus::to_toml(t));
        }
    }
    Ok(())
}

fn cmd_learn(a: &[String]) -> Result<(), String> {
    let args = parse_args(
        &a.iter()
            .filter(|x| *x != "--ask-sarina")
            .cloned()
            .collect::<Vec<_>>(),
        &["adapter", "can", "seconds"],
    )?;
    let ask = a.iter().any(|x| x == "--ask-sarina");
    if ask && icarus_obd::learning::sarina().is_none() {
        return Err(
            "--ask-sarina: Sarina is not installed (labyrinth-ask-sarina is not on PATH)".into(),
        );
    }
    let secs: Option<f64> = args
        .opt("seconds")
        .map(|v| v.parse().map_err(|_| "--seconds needs a number"))
        .transpose()?;
    // each line typed while watching is a mark; "stop" ends the session (as do Ctrl+C and --seconds)
    let (ltx, lrx) = std::sync::mpsc::channel::<String>();
    std::thread::spawn(move || {
        use std::io::BufRead;
        for line in std::io::stdin().lock().lines().map_while(Result::ok) {
            if ltx.send(line).is_err() {
                return;
            }
        }
    });
    actuator::catch_interrupt();
    let mut an = bus::Analyzer::default();
    let start = std::time::Instant::now();
    let mut printed = 0u64;
    let mut step = |an: &mut bus::Analyzer, f: Option<bus::Frame>| {
        if let Some(f) = f {
            an.add(&f);
        }
        let mut go =
            !actuator::interrupted() && secs.is_none_or(|s| start.elapsed().as_secs_f64() < s);
        while let Ok(l) = lrx.try_recv() {
            if l.trim().eq_ignore_ascii_case("stop") {
                go = false;
            } else {
                an.mark(&l, start.elapsed().as_secs_f64());
            }
        }
        let new = (an.logged - printed).min(an.log.len() as u64) as usize;
        for l in an.log.iter().skip(an.log.len() - new) {
            println!("{l}");
        }
        printed = an.logged;
        go
    };
    let all = profiles();
    let (vin, source, vehicle) = if let Some(ifn) = args.opt("can") {
        (
            None,
            format!("the CAN interface {ifn}"),
            "a vehicle Icarus did not ask (CAN interface)".to_string(),
        )
    } else {
        let mut elm = connect(&args)?;
        let vin = j1979::vin(&mut elm).map_err(e)?;
        let vehicle = match vin.as_deref() {
            Some(v) if v == sim::VIN => "the simulated vehicle".to_string(),
            Some(v) => garage::get(v)
                .map(|d| d.name())
                .or_else(|| vehicles::for_vin(&all, v).map(|p| p.name.clone()))
                .unwrap_or_else(|| "a vehicle".into()),
            None => "a vehicle that gave no VIN".into(),
        };
        eprintln!("Learning on {vehicle}. Icarus only listens: nothing is sent onto the bus.");
        eprintln!("Before each change in the other tool, type what you are about to do and press Enter (\"seat belt chime off\"); \"stop\" or Ctrl+C ends.");
        let proto = elm.protocol;
        let ended = elm
            .monitor(|line| {
                let f = bus::parse_line(line, proto, start.elapsed().as_secs_f64());
                step(&mut an, f)
            })
            .map_err(e)?;
        eprintln!("({ended})");
        (
            vin,
            format!(
                "the adapter {}",
                args.opt("adapter")
                    .map(str::to_string)
                    .or_else(|| std::env::var("ICARUS_ADAPTER").ok())
                    .unwrap_or_default()
            ),
            vehicle,
        )
    };
    if let Some(ifn) = args.opt("can") {
        eprintln!("Learning on the CAN interface {ifn}. Icarus only listens. Type what you are about to do and press Enter before each change; \"stop\" or Ctrl+C ends.");
        let sock = socketcan::open(ifn).map_err(|x| x.to_string())?;
        loop {
            let f = sock
                .read(
                    std::time::Duration::from_millis(100),
                    start.elapsed().as_secs_f64(),
                )
                .map_err(|x| x.to_string())?;
            if !step(&mut an, f) {
                break;
            }
        }
    }
    let modules = vin
        .as_deref()
        .and_then(|v| vehicles::for_vin(&all, v))
        .map(vehicles::candidates)
        .unwrap_or_default();
    let text = icarus_obd::learning::session_text(&an, &source, &vehicle, &modules);
    let dir = garage::logs_dir(vin.as_deref())?;
    let saved = icarus_obd::learning::save(&dir, &text)?;
    println!(
        "\n{} frames. Learned: {} setting(s), {} test(s); {} thing(s) seen but not learned. Saved: {}",
        an.frames,
        an.settings.len(),
        an.learned.len(),
        an.notes.len(),
        saved.display()
    );
    for s in &an.settings {
        println!("\n{}", settings::to_toml(s));
    }
    for t in &an.learned {
        println!("\n{}", bus::to_toml(t));
    }
    for n in &an.notes {
        println!("not learned: {n}");
    }
    if !an.settings.is_empty() || !an.learned.is_empty() {
        println!("\nRename them (and the values \"was\" and \"set\") as you like, then add them to ~/.config/icarus/vehicles/MAKE.toml, or use the Learning page, which does both.");
    }
    if ask {
        eprintln!("\nAsking Sarina about the session (she only reads)...");
        match icarus_obd::learning::ask_sarina(&saved) {
            Ok(answer) => println!("\nSarina:\n{answer}"),
            Err(x) => eprintln!("{x}"),
        }
    }
    Ok(())
}

fn cmd_recalls(a: &[String]) -> Result<(), String> {
    let v = garage::Described::parse(&a.join(" "));
    let (Some(year), false, false) = (v.year, v.make.is_empty(), v.model.is_empty()) else {
        return Err("usage: icarus recalls \"2017 Toyota Tacoma\"   (year make model; sends only those to NHTSA)".into());
    };
    let list = nhtsa::recalls(&v.make, &v.model, year)?;
    println!(
        "{} recall(s) on record for the {year} {} {}:",
        list.len(),
        v.make,
        v.model
    );
    for r in &list {
        println!("\n{} ({}) {}", r.campaign, r.date, r.component);
        println!("  {}", r.summary);
        if !r.remedy.is_empty() {
            println!("  Remedy: {}", r.remedy);
        }
    }
    println!(
        "\nWhether one vehicle is affected and still open: its VIN at {}",
        nhtsa::VIN_RECALL_CHECK
    );
    Ok(())
}

fn cmd_tires(a: &[String]) -> Result<(), String> {
    let args = parse_args(a, &["gear", "speed", "installed"])?;
    let [stock, new] = args.pos.as_slice() else {
        return Err("usage: icarus tires STOCK NEW [--gear STOCK_RATIO] [--installed NEW_RATIO] [--speed 65]   (sizes like 265/70R16 or 33x12.50R15)".into());
    };
    let (stock, new) = (
        icarus_core::tires::Tire::parse(stock)?,
        icarus_core::tires::Tire::parse(new)?,
    );
    let gear = args
        .opt("gear")
        .map(|g| {
            g.parse::<f64>()
                .map_err(|_| "--gear needs a ratio like 3.909")
        })
        .transpose()?;
    let installed = args
        .opt("installed")
        .map(|g| {
            g.parse::<f64>()
                .map_err(|_| "--installed needs a ratio like 4.88")
        })
        .transpose()?;
    if installed.is_some() && gear.is_none() {
        return Err("--installed needs the stock ratio too (--gear)".into());
    }
    let speed: f64 = args
        .opt("speed")
        .map(|v| v.parse().map_err(|_| "--speed needs a number"))
        .transpose()?
        .unwrap_or(65.0);
    let c = icarus_core::tires::change(stock, new, speed, gear, installed);
    for line in icarus_core::tires::explain(&c, speed, gear, installed) {
        println!("{line}");
    }
    Ok(())
}

fn cmd_garage(a: &[String]) -> Result<(), String> {
    match a {
        [] => {
            let all = garage::load();
            if all.is_empty() {
                println!("No vehicles named yet: icarus garage VIN \"2017 Toyota Tacoma 3.5L V6\"");
            }
            for (vin, name) in all {
                println!("{vin}  {name}");
            }
            Ok(())
        }
        [v, name @ ..] => {
            let i = vin::decode(v)?;
            garage::set(&i.vin, &name.join(" "))?;
            println!(
                "{}",
                if name.is_empty() {
                    format!("{} forgotten.", i.vin)
                } else {
                    format!("{} is now \"{}\".", i.vin, name.join(" "))
                }
            );
            Ok(())
        }
    }
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

/// `icarus devices`: the recognised tuners and interfaces plugged in now (read from /sys; nothing is sent to them).
fn cmd_devices() -> Result<(), String> {
    use icarus_obd::devices;
    let found = devices::scan(std::path::Path::new("/sys/bus/usb/devices"));
    if found.is_empty() {
        println!("No tuner or vehicle interface Icarus knows is plugged in.");
        println!(
            "Known: {}.",
            devices::KNOWN
                .iter()
                .filter(|k| k.product.is_some())
                .map(|k| format!("{} {}", k.maker, k.name))
                .collect::<Vec<_>>()
                .join(", ")
        );
        return Ok(());
    }
    for f in &found {
        println!(
            "{} {}  ({:04x}:{:04x}{})",
            f.known.maker,
            f.known.name,
            f.vendor,
            f.product,
            if f.label.is_empty() {
                String::new()
            } else {
                format!(", says \"{}\"", f.label)
            }
        );
        println!("    {}", f.status(devices::can_open));
        println!("    {}", f.known.note);
    }
    Ok(())
}
