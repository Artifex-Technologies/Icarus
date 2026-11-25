//! `icarus`: the command line of the Icarus tuning tool. Files first (definitions, images, what a tune changed), then the
//! vehicle through an OBD adapter, read-only.

use icarus_core::def::{fmt_num, Definition, Kind};
use icarus_core::rom::{self, Rom};
use icarus_obd::elm::Elm;
use icarus_obd::j1979::{self, DtcKind};
use icarus_obd::link::Link;
use icarus_obd::port::Target;
use icarus_obd::{
    actuator, bus, charging, clear, dtc, garage, lock, lookup, mode06, nhtsa, report, settings,
    sim, slcan, socketcan, uds, vehicles, vin,
};
mod data;
mod gps;
mod mcp;

use std::io::Write;
use std::process::ExitCode;
use std::time::{Duration, Instant};

const HELP: &str = "icarus: vehicle tuning, learning and diagnostics

Files (nothing is sent to a vehicle; DEF is a TunerPro .xdf or an ASAM .a2l definition):
  icarus def DEF [--find TEXT]               what a definition describes (with --find: the matching maps)
  icarus show IMAGE.bin DEF NAME             one map or value from an image, in real units
  icarus set IMAGE.bin DEF NAME VALUE [--row R --col C] [--out NEW.bin | --version NAME [--note TEXT]] [--checksums FILE]
                                             change one value, table cell (from 0) or switch (on/off) and save a NEW file
                                             (never over one), its checksums corrected where they are known
                                             (<definition>.checksums.toml beside the definition, or --checksums); with
                                             --version the change is kept as a named version of the image instead
  icarus checksums IMAGE.bin CHECKSUMS.toml  whether an image's checksums are right
  icarus diff ORIGINAL.bin TUNED.bin|VERSION.version.toml [--def DEF]
                                             what a tune (or a version) changed: byte ranges, and with a definition each
                                             map, cell by cell
  icarus version new ORIGINAL.bin TUNED.bin NAME [--def DEF] [--note TEXT]
                                             keep what a tune changes as a named version of the original: a small file of
                                             changes beside the vehicle's files (Custom tunes), never over another file
  icarus version list ORIGINAL.bin           \"stock\" (the original) and the versions kept for it
  icarus version image ORIGINAL.bin VERSION.version.toml [--out NEW.bin] [--def DEF | --checksums FILE]
                                             a version's full image as a NEW file, its checksums corrected where known
  icarus version rename ORIGINAL.bin VERSION.version.toml NEW_NAME
                                             give a version another name (never over another version)
  icarus version delete VERSION.version.toml remove a version's file (stock, the original, is never one)

Vehicle, through an OBD adapter (read-only: nothing is cleared or written):
  icarus obd info  --adapter A               adapter, protocol, VIN, modules, calibration IDs, check-engine light, monitors
  icarus obd codes --adapter A               stored, pending and permanent trouble codes, per module
  icarus obd pids  --adapter A               the live values each module offers, and what they mean
  icarus obd live  --adapter A [PID...] [--every MS] [--count N] [--csv FILE] [--gps SOURCE] [--all]
                                             watch live values (PIDs in hex, e.g. 0C 05 06 07); Ctrl+C stops
  icarus obd scan  --adapter A [--make NAME] [--sweep]
                                             find every module (ABS, airbags, cluster...): identity and trouble codes,
                                             as dealer tools do. The make comes from the VIN (or --make, see icarus
                                             vehicles); --sweep tries every request ID from 700 to 7F7 (slower;
                                             never 7DF or a known answer ID such as 7E8-7EF)
  icarus obd asbuilt ID[/ANSWER] --adapter A one module's as-built configuration block (22 DE00 on, read only), saved
                                             in the vehicle's folder: the backup before any configuration change
  icarus obd module ID[/ANSWER] --adapter A  one module (request ID in hex: 7B0, 713/77D, 18DA30F1): identity and codes
  icarus obd data [MODULE...] --adapter A [--set SET] [--year YYYY] [--find TEXT] [--every MS --count N] [--csv FILE]
                  [--all-years] [--all] [--refused]
                                             each module's own values (a dealer tool's data list: transmission fluid
                                             temperature, knock correction...) from a module-data set (icarus data), in
                                             US units where those are the custom (ICARUS_UNITS=us|metric). Read-only:
                                             services 21 and 22 only, paced, never 7DF or an answer ID, airbag modules
                                             identity only; requests that need a session are listed, never sent. --all
                                             asks again what the vehicle refused before. The set is chosen by the
                                             VIN's make, the model year and the model (icarus garage); with one of them
                                             unknown, none is guessed: --set. Ctrl+C ends --every, keeping what was
                                             learned
  icarus obd data --check MODULE VALUE_ID [--note TEXT] --adapter A
                                             mark a value as checked on this vehicle (it read right against a gauge)

Module data (definition files; nothing is sent to a vehicle):
  icarus import obdb FILE.json [--name SET] [--make NAME] [--dry-run] [--replace]
                                             import a vehicle's OBDb signal set you downloaded (github.com/OBDb, e.g.
                                             Toyota-Tacoma/signalsets/v3/default.json) into ~/.config/icarus/data/SET.
                                             OBDb's data is CC BY-SA 4.0: Icarus ships none of it, and shows its
                                             attribution wherever its values appear
  icarus import abrp FILE.json [--name SET] [--make NAME] [--model WORDS] [--dry-run] [--replace]
                                             an electric or hybrid vehicle's PID list from A Better Routeplanner
                                             (github.com/Iternio-Planning-AB/ev-obd-pids, e.g. hkmc/Ioniq5.json):
                                             battery charge, health, current, voltage, temperatures (Apache-2.0)
  icarus import btlog FILE [--name SET] [--make NAME] [--transcript OUT.csv] [--keep-vin] [--dry-run] [--replace]
                                             learn from another app at work: a phone's Bluetooth log (idevicebtlogger,
                                             pcap or PacketLogger .pklg) of an app talking to its ELM327/STN adapter.
                                             Each identifier a module answered on a data read (21, 22) becomes a
                                             request with one unnamed placeholder value, not verified, its answers kept
                                             as samples (icarus data SET --sample); everything else the app sent
                                             (identity reads 1A, writes, tests, routines, clearing, security access) is
                                             listed, never imported. --transcript writes the timed requests and answers
                                             as CSV (- for stdout, the summary then on stderr), to match against a
                                             screen recording. Kept: only the OBD conversation; Bluetooth addresses,
                                             device names, link keys and other channels are never written; VINs are
                                             masked unless --keep-vin
  icarus import buslog FILE [the same options]
                                             the same from a bus log: icarus bus watch --raw (through a Y-splitter, while
                                             another tool works) or candump; either name reads either kind of file
  icarus import ddt ECU.xml                  DDT2000 files: not built yet (build 3); it says so and imports nothing
  icarus data [SET] [--module ID] [--find TEXT] [--sample] [--refused]
                                             the sets, or one set's modules, requests and values (--sample decodes the
                                             sample answers a file carries)
  icarus data check FILE.toml [--make NAME]  whether a hand-written module file is right, and what Icarus would send
                                             (with the make's profile, as on the vehicle)
  icarus data name SET MODULE VALUE_ID --name TEXT [--id NEW_ID] [--keep] [--byte N] [--bit N] [--bits N] [--signed]
                   [--mul X] [--div X] [--add X] [--unit U] [--kind number|text|hex]
                                             name a value of a set learned from a capture (its placeholder RAW_...):
                                             where it sits in the answer and how it scales; --keep keeps the placeholder
                                             and adds the named value beside it (one answer, several values). The file
                                             is checked and written back; the samples show what it decodes to

Bus (watching only: nothing is sent onto the bus):
  icarus bus watch (--adapter A | --can can0) [--seconds N] [--raw] [--diagnostic] [--dbc FILE]
                                             every ID with its rate and changing bytes, diagnostic messages in words, and
                                             actuator tests another tool runs, as entries for the vehicle's file. A is also
                                             slcan:/dev/ttyACM0[@baud][#bitrate] (an SLCAN adapter, opened listen-only);
                                             --diagnostic keeps only IDs 700-7FF (another tool's requests and the answers),
                                             for a bus busier than an ELM/STN adapter's serial link carries; --dbc names the
                                             signals of a CAN database (.dbc) in each ID's last frame
  icarus learn (--adapter A | --can can0) [--seconds N] [--ask-sarina]
                                             learn from another tool working on the vehicle (a Y-cable on the OBD port):
                                             type what you are about to do in it and press Enter (a mark), then do it;
                                             \"stop\" or Ctrl+C ends. The settings and tests it changed come out named by
                                             your marks, saved with everything seen in the vehicle's Logs folder;
                                             --ask-sarina (only when Sarina is installed) has Sarina read the session
                                             and say what Sarina makes of it

Interfaces:
  icarus devices                             the tuners and interfaces plugged in (SCT, COBB, HP Tuners, CP210x/FTDI
                                             cables...), what each is and whether Icarus can open it; nothing is sent

Tools:
  icarus gps [--gps SOURCE] [--seconds 15] [--position]
                                             what a GPS receiver hears: the fix, each satellite's signal, and what that
                                             says about the receiver and its antenna. SOURCE: /dev/ttyACM0[@baud] (USB,
                                             9600 unless given), bt:MAC, tcp:HOST:PORT (a phone sharing its GPS) or sim;
                                             ICARUS_GPS sets a default. Positions only on screen, and only with --position
  icarus gps speed --adapter A [--gps SOURCE] [--seconds 10]
                                             GPS speed against the vehicle's own (PID 0D), once a second, then how far
                                             the vehicle's reads off (the speedometer check without typing)
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
  icarus garage tires VIN STOCK INSTALLED [--at wheels|transmission] [--gear 3.91 --installed 4.88]
                                             save a vehicle's tires (and regear) for its corrected speed in obd live
                                             and Live data; VIN off forgets it
  icarus garage history VIN                  when each trouble code came and went on this vehicle (every reading of its
                                             codes is kept in its folder, Logs/codes-history.tsv)
  icarus garage read-only VIN on|off         a vehicle's Read-Only switch: on, Icarus clears, tests and changes nothing
                                             on it (the same switch as on the Garage page)

Trouble codes:
  icarus dtc CODE... [--vehicle \"2017 Toyota Tacoma 3.5L V6\"] [--make toyota]
                                             what a code means, where to start, the parts usually involved, links to look
                                             it up and to find parts (no vehicle needed)
  icarus obd codes --adapter A               the vehicle's codes, each explained, and the freeze frame
  icarus obd tests --adapter A               actuator tests known for this vehicle (fan, throttle, pumps...)
  icarus obd services --adapter A            service routines known for this vehicle (relearns, resets, TPMS, coding)
  icarus obd service \"NAME\" --adapter A --yes
                                             run one (its conditions, the battery and the Read-Only switch checked
                                             first; coding only over a cable; logged)
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
  an OBDLink MX+), tcp:host:port (Wi-Fi), can:can0 or can0 (a Linux CAN interface such as a CANable: Icarus speaks
  ISO-TP over it itself). ICARUS_ADAPTER sets a default.";

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let r = match args.first().map(String::as_str) {
        Some("def") => cmd_def(&args[1..]),
        Some("show") => cmd_show(&args[1..]),
        Some("diff") => cmd_diff(&args[1..]),
        Some("set") => cmd_set(&args[1..]),
        Some("checksums") => cmd_checksums(&args[1..]),
        Some("version") => cmd_version(&args[1..]),
        Some("obd") => cmd_obd(&args[1..]),
        Some("vin") => cmd_vin(&args[1..]),
        Some("vehicles") => cmd_vehicles(),
        Some("dtc") => cmd_dtc(&args[1..]),
        Some("garage") => cmd_garage(&args[1..]),
        Some("bus") => cmd_bus(&args[1..]),
        Some("learn") => cmd_learn(&args[1..]),
        Some("tires") => cmd_tires(&args[1..]),
        Some("gps") => gps::cmd_gps(&args[1..]),
        Some("recalls") => cmd_recalls(&args[1..]),
        Some("import") => data::cmd_import(&args[1..]),
        Some("data") => data::cmd_data(&args[1..]),
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
        if [
            "--sweep",
            "--scan",
            "--yes",
            "--dry-run",
            "--replace",
            "--sample",
            "--refused",
            "--all-years",
            "--all",
            "--check",
            "--keep-vin",
            "--position",
            "--keep",
            "--signed",
        ]
        .contains(&a[i].as_str())
        {
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

/// A TunerPro (.xdf) or ASAM (.a2l) definition.
fn load_def(path: &str) -> Result<Definition, String> {
    let p =
        icarus_core::def::load(std::path::Path::new(path)).map_err(|e| format!("{path}: {e}"))?;
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
        return Err("usage: icarus def DEF.xdf|.a2l [--find TEXT]".into());
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
    if def.base_offset != 0 {
        println!(
            "Addresses: file offset = address {} 0x{:X}",
            if def.base_subtract { "-" } else { "+" },
            def.base_offset
        );
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
        return Err("usage: icarus show IMAGE.bin DEF.xdf|.a2l NAME".into());
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
    let args = parse_args(a, &["row", "col", "out", "checksums", "version", "note"])?;
    let [image, defp, name, value] = args.pos.as_slice() else {
        return Err("usage: icarus set IMAGE.bin DEF.xdf|.a2l NAME VALUE [--row R --col C] [--out NEW.bin | --version NAME [--note TEXT]] [--checksums FILE]".into());
    };
    if args.opt("version").is_some() && args.opt("out").is_some() {
        return Err("--version keeps the change as a version (stock stays as it is); --out writes an image: one or the other".into());
    }
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
    // ICA-58: the change kept as a version of the image (a small file of what changed against stock), not as an image
    if let Some(vname) = args.opt("version") {
        let (p, v) = icarus_core::version::save_edits(
            &rom,
            Some(&def),
            std::path::Path::new(image),
            vname,
            args.opt("note").unwrap_or(""),
        )?;
        println!(
            "Kept version \"{}\": {} byte(s) changed against stock, in {}",
            v.name,
            v.changed_bytes(),
            p.display()
        );
        return Ok(());
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
        return Err(
            "usage: icarus diff ORIGINAL.bin TUNED.bin|VERSION.version.toml [--def DEF.xdf|.a2l]"
                .into(),
        );
    };
    let def = args.opt("def").map(load_def).transpose()?;
    let ob = load_image(orig, def.as_ref())?;
    // a version is its changes against the original: the image it makes is compared
    let tb = if icarus_core::version::is_version_file(std::path::Path::new(tuned)) {
        let v = icarus_core::version::load(std::path::Path::new(tuned))?;
        println!(
            "Version \"{}\" of {}{}",
            v.name,
            orig,
            if v.note.is_empty() {
                String::new()
            } else {
                format!(": {}", v.note)
            }
        );
        v.apply(&ob)?
    } else {
        load_image(tuned, def.as_ref())?
    };
    for line in diff_report(def.as_ref(), &ob, &tb)? {
        println!("{line}");
    }
    Ok(())
}

/// What `tb` changes against `ob`: byte ranges, and with a definition each element, cell by cell.
fn diff_report(def: Option<&Definition>, ob: &[u8], tb: &[u8]) -> Result<Vec<String>, String> {
    let mut out = Vec::new();
    let ranges = rom::diff(ob, tb);
    if ranges.is_empty() {
        out.push("The two images are the same.".to_string());
        return Ok(out);
    }
    let bytes: usize = ranges.iter().map(|r| r.len()).sum();
    out.push(format!(
        "{bytes} byte(s) differ in {} place(s).",
        ranges.len()
    ));
    let Some(def) = def else {
        for r in ranges.iter().take(50) {
            out.push(format!(
                "  0x{:06X}..0x{:06X} ({} bytes)",
                r.start,
                r.end,
                r.len()
            ));
        }
        return Ok(out);
    };
    if ob.len() != tb.len() {
        return Err("the images differ in size, so a cell-by-cell comparison would mislead".into());
    }
    let (or, tr) = (Rom::new(ob.to_vec()), Rom::new(tb.to_vec()));
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
            out.push(String::new());
            out.push(format!("{} ({})", e.title, kind_name(&e.kind)));
            match &e.kind {
                Kind::Flag(f) => {
                    let (a, b) = (
                        def.flag(&or, f).map_err(|x| x.to_string())?,
                        def.flag(&tr, f).map_err(|x| x.to_string())?,
                    );
                    out.push(format!(
                        "  {} -> {}",
                        if a { "on" } else { "off" },
                        if b { "on" } else { "off" }
                    ));
                }
                Kind::Constant(d) => {
                    let (a, b) = (
                        def.values(&or, d).map_err(|x| x.to_string())?,
                        def.values(&tr, d).map_err(|x| x.to_string())?,
                    );
                    out.push(format!(
                        "  {} -> {} {}",
                        fmt_num(a[0][0], d.decimals),
                        fmt_num(b[0][0], d.decimals),
                        d.units
                    ));
                }
                Kind::Table(t) => {
                    let (a, b) = (
                        def.table(&or, t).map_err(|x| x.to_string())?,
                        def.table(&tr, t).map_err(|x| x.to_string())?,
                    );
                    if a.x != b.x || a.y != b.y {
                        out.push("  the axis breakpoints changed too".to_string());
                    }
                    let mut n = 0;
                    for (ri, (ra, rb)) in a.z.iter().zip(&b.z).enumerate() {
                        for (ci, (va, vb)) in ra.iter().zip(rb).enumerate() {
                            if va != vb {
                                n += 1;
                                if n <= 12 {
                                    out.push(format!(
                                        "  at {} / {}: {} -> {} {}",
                                        b.y.get(ri).map(String::as_str).unwrap_or("-"),
                                        b.x.get(ci).map(String::as_str).unwrap_or("-"),
                                        fmt_num(*va, t.z.decimals),
                                        fmt_num(*vb, t.z.decimals),
                                        t.z.units
                                    ));
                                }
                            }
                        }
                    }
                    if n > 12 {
                        out.push(format!("  ... {} cells changed in all", n));
                    }
                }
            }
        }
    }
    if unnamed > 0 {
        out.push(String::new());
        out.push(format!("{unnamed} changed byte(s) are in no element of the definition (code, checksums, or maps it does not describe)."));
    }
    Ok(out)
}

/// `icarus version`: versions of a tune ("stock", the original, plus named sets of changes against it; icarus-core
/// version.rs), kept beside the vehicle's files.
fn cmd_version(a: &[String]) -> Result<(), String> {
    use icarus_core::version::{self, Version};
    use std::path::Path;
    let args = parse_args(
        a.get(1..).unwrap_or(&[]),
        &["def", "note", "out", "checksums"],
    )?;
    match (a.first().map(String::as_str), args.pos.as_slice()) {
        (Some("new"), [orig, tuned, name]) => {
            let def = args.opt("def").map(load_def).transpose()?;
            let ob = load_image(orig, def.as_ref())?;
            let tb = load_image(tuned, def.as_ref())?;
            let file_name = Path::new(orig)
                .file_name()
                .map(|n| n.to_string_lossy().to_string())
                .unwrap_or_default();
            let v = Version::between(name, args.opt("note").unwrap_or(""), &file_name, &ob, &tb, def.as_ref())?;
            let p = version::save(&v, Path::new(orig))?;
            println!(
                "Saved version \"{}\": {} byte(s) in {} place(s) changed against {orig}, in {}",
                v.name,
                v.changed_bytes(),
                v.changes.len(),
                p.display()
            );
            Ok(())
        }
        (Some("list"), [orig]) => {
            let ob = load_image(orig, None)?;
            println!("stock: {orig} (the original, never changed)");
            for (p, v) in version::list(Path::new(orig), &ob) {
                println!(
                    "{}: {} byte(s) in {} place(s){}  ({})",
                    v.name,
                    v.changed_bytes(),
                    v.changes.len(),
                    if v.note.is_empty() { String::new() } else { format!(", {}", v.note) },
                    p.display()
                );
            }
            Ok(())
        }
        (Some("image"), [orig, vpath]) => {
            let ob = load_image(orig, None)?;
            let v = version::load(Path::new(vpath))?;
            let mut rom = Rom::new(ob.clone());
            rom.load_edits(&v.apply(&ob)?).map_err(|e| e.to_string())?;
            let specs = match (args.opt("checksums"), args.opt("def")) {
                (Some(c), _) => load_checksums("", Some(c), rom.len())?,
                (None, Some(d)) => load_checksums(d, None, rom.len())?,
                (None, None) => vec![],
            };
            let out = args.opt("out").map(std::path::PathBuf::from).unwrap_or_else(|| {
                icarus_core::edit::save_dir(Path::new(orig)).join(format!(
                    "{} - {}.bin",
                    Path::new(orig).file_stem().unwrap_or_default().to_string_lossy(),
                    v.name
                ))
            });
            let saved = icarus_core::edit::save(&rom, &specs, &out)?;
            println!("{}", saved.summary());
            Ok(())
        }
        (Some("rename"), [orig, vpath, name]) => {
            let p = version::rename(Path::new(vpath), Path::new(orig), name)?;
            println!("Renamed to \"{}\": {}", name.trim(), p.display());
            Ok(())
        }
        (Some("delete"), [vpath]) => {
            let v = version::delete(Path::new(vpath))?;
            println!("Deleted version \"{}\" ({vpath}); stock and the other versions stay", v.name);
            Ok(())
        }
        _ => Err("usage: icarus version new ORIGINAL.bin TUNED.bin NAME [--def DEF] [--note TEXT]\n       icarus version list ORIGINAL.bin\n       icarus version image ORIGINAL.bin VERSION.version.toml [--out NEW.bin] [--def DEF | --checksums FILE]\n       icarus version rename ORIGINAL.bin VERSION.version.toml NEW_NAME\n       icarus version delete VERSION.version.toml".into()),
    }
}

/// The adapter `--adapter` (or ICARUS_ADAPTER) names, parsed.
fn adapter_target(args: &Args) -> Result<(String, Target), String> {
    let spec = args.opt("adapter").map(str::to_string).or_else(|| std::env::var("ICARUS_ADAPTER").ok()).ok_or(
        "which adapter? --adapter sim | /dev/ttyUSB0 | bt:AA:BB:CC:DD:EE:FF | tcp:host:port | can:can0 (or set ICARUS_ADAPTER)",
    )?;
    let target = Target::parse(&spec)?;
    Ok((spec, target))
}

/// The link to the vehicle: an ELM327/STN adapter, or a Linux CAN interface (`can0`, spoken to over SocketCAN).
fn connect(args: &Args) -> Result<Box<dyn Link>, String> {
    let (spec, target) = adapter_target(args)?;
    eprintln!("connecting to {spec}...");
    if let Target::Can(ifn) = &target {
        let sock = socketcan::open(ifn).map_err(|x| x.to_string())?;
        // the interface's bit rate from the kernel (ICA-68): shown, and 250 kbit/s named as such
        return icarus_obd::canlink::CanLink::connect(sock, ifn)
            .map(|l| Box::new(l.with_bitrate(socketcan::bitrate(ifn))) as Box<dyn Link>)
            .map_err(|x| x.to_string());
    }
    let (elm, note) =
        icarus_obd::elm::connect_target(&target).map_err(|e| format!("{spec}: {e}"))?;
    if let Some(n) = note {
        eprintln!("note: {n}");
    }
    Ok(Box::new(elm) as Box<dyn Link>)
}

/// The Linux CAN interface to watch: `--can can0`, or `--adapter can0` / `can:can0`.
fn can_interface(args: &Args) -> Option<String> {
    args.opt("can")
        .map(str::to_string)
        .or_else(|| match adapter_target(args) {
            Ok((_, Target::Can(ifn))) => Some(ifn),
            _ => None,
        })
}

/// An ELM327/STN adapter itself, for what only an adapter does (its monitor mode).
/// The adapter set up to listen only (ICA-80): `Elm::listen` on the protocol `--protocol` names (an ELM327 protocol number,
/// 6 by default: CAN 11-bit at 500 kbit/s). No request goes onto the bus, not even `connect`'s protocol search.
fn listen_elm(args: &Args) -> Result<Elm, String> {
    let (spec, target) = adapter_target(args)?;
    let protocol = match args.opt("protocol") {
        None => 6,
        Some(p) => u8::from_str_radix(p.trim_start_matches('A'), 16)
            .ok()
            .filter(|n| (1..=0xC).contains(n))
            .ok_or("--protocol is an ELM327 protocol number, 1 to C (6: CAN 11-bit, 500 kbit/s)")?,
    };
    eprintln!("connecting to {spec} to listen (protocol {protocol:X})...");
    let (elm, note) =
        icarus_obd::elm::listen_target(&target, protocol).map_err(|e| format!("{spec}: {e}"))?;
    if let Some(n) = note {
        eprintln!("note: {n}");
    }
    Ok(elm)
}

fn connect_elm(args: &Args) -> Result<Elm, String> {
    let (spec, target) = adapter_target(args)?;
    eprintln!("connecting to {spec}...");
    let (elm, note) =
        icarus_obd::elm::connect_target(&target).map_err(|e| format!("{spec}: {e}"))?;
    if let Some(n) = note {
        eprintln!("note: {n}");
    }
    Ok(elm)
}

/// Keeps what the modules did with each request in the vehicle's record (ICA-70), when the command asked single modules: the
/// VIN says which vehicle (one more read).
fn keep_record(link: &mut dyn Link) {
    let outcomes = link.take_outcomes();
    if outcomes.is_empty() {
        return;
    }
    let Ok(Some(vin)) = icarus_obd::j1979::vin(link) else {
        eprintln!("note: the vehicle gave no VIN, so what its modules answered was not kept");
        return;
    };
    let protocol = link.protocol().describe();
    if let Err(e) = icarus_obd::record::merge(&vin, &outcomes, &link.adapter(), protocol) {
        eprintln!("note: the vehicle's record was not kept: {e}");
    }
}

/// Prints the notes on modules left out of broadcasts' answers (`Link::take_dropped`): every command that shows a
/// broadcast's answers calls it, so a module is never lost silently.
pub(crate) fn note_dropped(elm: &mut dyn Link) {
    for n in elm.take_dropped() {
        eprintln!("note: {n}");
    }
}

fn cmd_obd(a: &[String]) -> Result<(), String> {
    // --fresh: ask everything, also what the vehicle's record says it refused for good (ICA-71)
    let fresh = a.iter().any(|x| x == "--fresh");
    let a: Vec<String> = a.iter().filter(|x| *x != "--fresh").cloned().collect();
    let a = a.as_slice();
    let args = parse_args(
        a.get(1..).unwrap_or(&[]),
        &[
            "adapter", "every", "count", "csv", "make", "out", "value", "seconds", "module",
            "battery", "set", "year", "find", "note", "gps",
        ],
    )?;
    // connected, run, and the senders any broadcast left out noted however it ended
    let module_reads = matches!(
        a.first().map(String::as_str),
        Some("scan" | "module" | "asbuilt")
    );
    let with = |f: &dyn Fn(&mut dyn Link) -> Result<(), String>| {
        let mut link = connect(&args)?;
        // a command that asks single modules learns from the vehicle's record first (the VIN says which)
        if module_reads && !fresh {
            if let Ok(Some(vin)) = icarus_obd::j1979::vin(&mut *link) {
                if let Ok(p) = icarus_obd::record::path(&vin) {
                    link.remember(&icarus_obd::record::load_in(&p));
                }
            }
        }
        let r = f(&mut *link);
        note_dropped(&mut *link);
        keep_record(&mut *link);
        r
    };
    match a.first().map(String::as_str) {
        Some("info") => with(&obd_info),
        Some("codes") => with(&obd_codes),
        Some("pids") => with(&obd_pids),
        Some("live") => with(&|e| obd_live(e, &args)),
        Some("scan") => with(&|e| obd_scan(e, &args)),
        Some("module") => with(&|e| obd_module(e, &args)),
        Some("asbuilt") => with(&|e| obd_asbuilt(e, &args)),
        Some("data") => data::obd_data(&args),
        Some("report") => with(&|e| obd_report(e, &args)),
        Some("tests") => with(&obd_tests),
        Some("services") => with(&obd_services),
        Some("service") => obd_service(&args),
        Some("clear") => with(&|e| obd_clear(e, &args)),
        Some("selftests") => with(&obd_selftests),
        Some("charging") => with(&|e| obd_charging(e, &args)),
        Some("test") => with(&|e| obd_test(e, &args)),
        Some("settings") => with(&obd_settings),
        Some("setting") => obd_setting(&args, false),
        Some("setting-undo") => obd_setting(&args, true),
        _ => Err(
            "usage: icarus obd info|codes|pids|live|scan|module|data --adapter A (see icarus --help)"
                .into(),
        ),
    }
}

fn print_module(elm: &mut dyn Link, m: &vehicles::ModuleAddr) -> Result<(), String> {
    let (tx, rx) = (m.tx, m.rx());
    let at = if m.rx.is_none_or(|r| r == tx + 8) {
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
    if let Some(a) = m.ext {
        println!("     behind the gateway at {tx:03X}, address {a:02X} (read only)");
    }
    let ((identity, id_notes), codes) = uds::at_module(elm, m, |l| {
        Ok((
            uds::module_identity_notes(l, tx, rx)?,
            uds::module_dtcs(l, tx, rx)?,
        ))
    })
    .map_err(e)?;
    for (what, value) in identity {
        println!("     {what:<17} {value}");
    }
    // what it refused or did not answer, and why: as useful as what it gave when finding out what a vehicle speaks
    for (i, note) in id_notes.iter().enumerate() {
        println!(
            "     {:<17} {note}",
            if i == 0 { "identification" } else { "" }
        );
    }
    match codes {
        Err(why) => println!("     trouble codes     none of Icarus's requests gave them\n                       ({why})"),
        Ok((dialect, codes)) => {
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

fn obd_scan(elm: &mut dyn Link, args: &Args) -> Result<(), String> {
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
        uds::sweep_on(&all, elm.protocol())
    } else {
        vehicles::candidates_on(&profile, elm.protocol())
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
    let mut guard = uds::ScanGuard::default();
    for m in &list {
        if let Some(why) = m.unreachable(elm.reach()) {
            later.push(format!("{:03X}  {}: {why}", m.tx, m.name));
            continue;
        }
        let there = match guard.present(elm, m) {
            // the adapter refused to switch to that module's bus: it, and the bus from now on, is listed as not reachable
            Err(icarus_obd::elm::ObdError::Adapter(why)) if m.on_ms_can() => {
                later.push(format!("{:03X}  {}: {why}", m.tx, m.name));
                continue;
            }
            r => r.map_err(e)?,
        };
        if there {
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

/// `icarus obd asbuilt MODULE`: the module's as-built block read whole (22 DE00 on) and saved in the vehicle's folder.
fn obd_asbuilt(elm: &mut dyn Link, args: &Args) -> Result<(), String> {
    let [id] = args.pos.as_slice() else {
        return Err("usage: icarus obd asbuilt MODULE --adapter A   (the request ID in hex, e.g. 726, or 726/72E)".into());
    };
    let (tx, rx) = match id.split_once('/') {
        Some((a, b)) => (
            u32::from_str_radix(a, 16).map_err(|_| format!("{a} is not an ID in hex"))?,
            u32::from_str_radix(b, 16).map_err(|_| format!("{b} is not an ID in hex"))?,
        ),
        None => {
            let t = u32::from_str_radix(id, 16).map_err(|_| format!("{id} is not an ID in hex"))?;
            (t, t + 8)
        }
    };
    let vin = j1979::vin(elm).map_err(e)?;
    let b = icarus_obd::asbuilt::read(elm, tx, rx)?;
    for (did, bytes) in &b {
        let hex: Vec<String> = bytes.iter().map(|x| format!("{x:02X}")).collect();
        println!("{did:04X}  {}", hex.join(" "));
    }
    let p = icarus_obd::asbuilt::save(vin.as_deref(), tx, &b)?;
    println!(
        "{} identifiers saved to {} (the way back before any change).",
        b.len(),
        p.display()
    );
    Ok(())
}

fn obd_module(elm: &mut dyn Link, args: &Args) -> Result<(), String> {
    let [id] = args.pos.as_slice() else {
        return Err(
            "usage: icarus obd module ID[/ANSWER_ID] --adapter A (hex, e.g. 7B0, 713/77D or 18DA30F1)".into(),
        );
    };
    let (t, r) = id
        .split_once('/')
        .map_or((id.as_str(), None), |(a, b)| (a, Some(b)));
    let hex = |s: &str| {
        u32::from_str_radix(s.trim_start_matches("0x"), 16)
            .map_err(|_| format!("{s} is not a module ID in hex"))
    };
    let tx = hex(t)?;
    let rx = r.map(hex).transpose()?;
    if let Some(why) = vehicles::request_id_refused(tx, &profiles()) {
        return Err(format!(
            "{why} (module request IDs are 700-7F7, or 18DAxxF1 on 29-bit CAN)"
        ));
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

fn obd_info(elm: &mut dyn Link) -> Result<(), String> {
    println!("Adapter:   {}", elm.adapter());
    if let Some(v) = elm.voltage() {
        println!("Battery:   {v:.1} V at the OBD port");
    }
    println!("Protocol:  {}", elm.protocol().describe());
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
            for (label, t) in dtc::depth(&i) {
                println!("{indent}    {}: {t}", label.to_lowercase());
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

fn obd_codes(elm: &mut dyn Link) -> Result<(), String> {
    let all = profiles();
    let vin = j1979::vin(elm).map_err(e)?;
    let profile = vin.as_deref().and_then(|v| vehicles::for_vin(&all, v));
    let vehicle = vin.as_deref().and_then(garage::get);
    if vehicle.is_none() {
        if let Some(v) = &vin {
            println!("(Name this vehicle once for better searches and parts links: icarus garage {v} \"2017 Toyota Tacoma 3.5L V6\")\n");
        }
    }
    let mut readings: Vec<icarus_obd::history::Reading> = Vec::new();
    for (kind, label, key) in [
        (DtcKind::Stored, "Stored (confirmed)", "stored"),
        (DtcKind::Pending, "Pending (seen once)", "pending"),
        (DtcKind::Permanent, "Permanent", "permanent"),
    ] {
        let list = j1979::dtcs(elm, kind).map_err(e)?;
        readings.push((
            key.to_string(),
            list.iter()
                .flat_map(|(from, module, codes)| {
                    codes
                        .iter()
                        .map(move |c| (format!("{module} ({from})"), c.clone()))
                })
                .collect(),
        ));
        let any = list.iter().any(|l| !l.2.is_empty());
        println!("{label}: {}", if any { "" } else { "none" });
        for (from, module, codes) in list.iter().filter(|l| !l.2.is_empty()) {
            println!("  {module} ({from}):");
            for c in codes {
                print_code(c, "", profile, vehicle.as_ref(), "    ");
            }
        }
    }
    if let Err(why) = icarus_obd::history::record(vin.as_deref(), &readings) {
        eprintln!("note: the code history could not be saved: {why}");
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

/// The service routines known for the connected vehicle.
fn vehicle_services(elm: &mut dyn Link) -> Result<Vec<icarus_obd::service::ServiceDef>, String> {
    let all = profiles();
    let vin = j1979::vin(elm).map_err(e)?;
    let profile = vin.as_deref().and_then(|v| vehicles::for_vin(&all, v));
    Ok(vehicles::services_for(profile, vin.as_deref()))
}

fn obd_services(elm: &mut dyn Link) -> Result<(), String> {
    let list = vehicle_services(elm)?;
    if list.is_empty() {
        println!(
            "No service routines are known for this vehicle yet: their IDs are the maker's own. They go in the vehicle's \
             profile as [[service]] entries, learned by watching another tool (icarus learn) or from the maker's service \
             information."
        );
    }
    for s in &list {
        let conds: Vec<&str> = s.conditions.iter().map(|c| c.describe()).collect();
        println!(
            "{}  ({:03X}, routine {:04X}{}{})",
            s.name,
            s.module,
            s.routine,
            if s.wired { ", wired only" } else { "" },
            if s.checked {
                ""
            } else {
                ", not yet checked on a vehicle"
            }
        );
        if !conds.is_empty() {
            println!("    needs: {}", conds.join(", "));
        }
        if !s.warning.is_empty() {
            println!("    {}", s.warning);
        }
    }
    Ok(())
}

/// `icarus obd service "NAME" --adapter A --yes`: runs one of the vehicle's service routines (without --yes, says what it
/// would do and stops).
fn obd_service(args: &Args) -> Result<(), String> {
    let [name] = args.pos.as_slice() else {
        return Err(
            "usage: icarus obd service \"NAME\" --adapter A [--yes]   (names: icarus obd services)"
                .into(),
        );
    };
    let spec = args
        .opt("adapter")
        .map(str::to_string)
        .or_else(|| std::env::var("ICARUS_ADAPTER").ok())
        .unwrap_or_default();
    let wired = Target::parse(&spec).map(|t| t.wired()).unwrap_or(false);
    let mut elm = connect(args)?;
    let list = vehicle_services(&mut elm)?;
    let s = list
        .iter()
        .find(|s| s.name.eq_ignore_ascii_case(name))
        .ok_or(format!(
            "no service routine {name:?} for this vehicle (icarus obd services)"
        ))?;
    println!(
        "{}: routine {:04X} in {:03X}{}.",
        s.name,
        s.routine,
        s.module,
        if s.checked {
            ""
        } else {
            " (not yet checked on a vehicle)"
        }
    );
    if !s.warning.is_empty() {
        println!("{}", s.warning);
    }
    if args.opt("yes").is_none() {
        println!("Nothing sent. Run it with --yes once the vehicle is ready (icarus obd services says what it needs).");
        return Ok(());
    }
    actuator::catch_interrupt();
    let status = icarus_obd::service::run(&mut elm, s, wired)?;
    println!(
        "Done{}. Every run is in ~/.local/state/icarus/actions.log.",
        if status.is_empty() {
            String::new()
        } else {
            format!(": the module answered {status}")
        }
    );
    Ok(())
}

fn vehicle_tests(elm: &mut dyn Link) -> Result<Vec<actuator::TestDef>, String> {
    let all = profiles();
    let vin = j1979::vin(elm).map_err(e)?;
    let profile = vin.as_deref().and_then(|v| vehicles::for_vin(&all, v));
    Ok(vehicles::tests_for(profile, vin.as_deref()))
}

fn obd_tests(elm: &mut dyn Link) -> Result<(), String> {
    let tests = vehicle_tests(elm)?;
    if tests.is_empty() {
        println!("No actuator tests are known for this vehicle yet: their IDs are the maker's own. They go in the vehicle's profile as [[test]] entries (docs/DESIGN.md).");
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

fn obd_test(elm: &mut dyn Link, args: &Args) -> Result<(), String> {
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

fn vehicle_settings(
    elm: &mut dyn Link,
) -> Result<(Option<String>, Vec<settings::SettingDef>), String> {
    let all = profiles();
    let vin = j1979::vin(elm).map_err(e)?;
    let profile = vin.as_deref().and_then(|v| vehicles::for_vin(&all, v));
    let list = vehicles::settings_for(profile, vin.as_deref());
    Ok((vin, list))
}

/// Where a vehicle's setting changes are kept: its own folder's Logs (the simulation's in Icarus's state folder).
fn obd_settings(elm: &mut dyn Link) -> Result<(), String> {
    let (_, list) = vehicle_settings(elm)?;
    if list.is_empty() {
        println!("No module settings are known for this vehicle yet: their IDs are the maker's own. Watch a dealer tool change one (icarus bus), then add what it learned to the vehicle's profile as a [[setting]] entry (docs/DESIGN.md).");
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
    note_dropped(&mut elm);
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

fn obd_charging(elm: &mut dyn Link, args: &Args) -> Result<(), String> {
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

fn obd_selftests(elm: &mut dyn Link) -> Result<(), String> {
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

fn obd_clear(elm: &mut dyn Link, args: &Args) -> Result<(), String> {
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
            if let Some(why) = actuator::not_one_module(tx) {
                return Err(why);
            }
            let rx = r.map(hex).transpose()?.unwrap_or(tx + 8);
            // never an airbag module: by ID, and by the name this vehicle's profile gives it
            let ps = profiles();
            let name = j1979::vin(elm)
                .ok()
                .flatten()
                .and_then(|v| icarus_obd::vehicles::for_vin(&ps, &v).cloned())
                .and_then(|p| {
                    p.modules
                        .into_iter()
                        .find(|m| m.tx == u32::from(tx))
                        .map(|m| m.name)
                })
                .unwrap_or_default();
            if let Some(why) = clear::refused(tx, rx, &name) {
                return Err(why);
            }
            let (d, codes) = uds::module_dtcs(elm, tx.into(), rx.into())
                .map_err(e)?
                .map_err(|why| format!("{tx:03X} did not give its codes ({why})"))?;
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

fn obd_report(elm: &mut dyn Link, args: &Args) -> Result<(), String> {
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
    bus_watch(a, &serial_port).map(|_| ())
}

/// A decoded value as `bus watch --dbc` shows it: whole numbers without decimals, others to three places at most.
fn dbc_number(v: f64) -> String {
    if v.fract() == 0.0 && v.abs() < 1e15 {
        format!("{v:.0}")
    } else {
        let s = format!("{v:.3}");
        s.trim_end_matches('0').trim_end_matches('.').to_string()
    }
}

/// A serial port, raw (an SLCAN adapter's).
fn serial_port(path: &str, baud: u32) -> std::io::Result<Box<dyn icarus_obd::port::Port>> {
    Ok(Box::new(icarus_obd::port::open_serial(path, baud)?))
}

/// What opens a serial port (`serial_port`, or a recorded session in the tests).
type OpenSerial<'a> = dyn Fn(&str, u32) -> std::io::Result<Box<dyn icarus_obd::port::Port>> + 'a;

/// The SLCAN adapter `--adapter slcan:...` names: (serial port, baud, CAN bit rate).
fn slcan_target(args: &Args) -> Option<(String, u32, u32)> {
    match adapter_target(args) {
        Ok((
            _,
            Target::Slcan {
                path,
                baud,
                bitrate,
            },
        )) => Some((path, baud, bitrate)),
        _ => None,
    }
}

/// Frames from an SLCAN adapter opened listen-only on `port`, handed to `step` until it says stop; the channel is closed
/// afterwards.
fn watch_slcan(
    port: Box<dyn icarus_obd::port::Port>,
    bitrate: u32,
    start: Instant,
    mut step: impl FnMut(Option<bus::Frame>) -> bool,
) -> Result<String, String> {
    let mut s = slcan::Slcan::listen(port, bitrate).map_err(e)?;
    let watched = loop {
        match s.read(Duration::from_millis(100), start.elapsed().as_secs_f64()) {
            Err(x) => break Err(x.to_string()),
            Ok(f) => {
                if !step(f) {
                    break Ok("stopped".to_string());
                }
            }
        }
    };
    let closed = s.close().map_err(e);
    let ended = watched?;
    closed?;
    Ok(ended)
}

/// `icarus bus watch`: an SLCAN adapter's serial port is opened with `serial` (the tests give a recorded session).
fn bus_watch(a: &[String], serial: &OpenSerial<'_>) -> Result<bus::Analyzer, String> {
    if a.first().map(String::as_str) != Some("watch") {
        return Err(
            "usage: icarus bus watch (--adapter A [--protocol N] | --can can0 | --adapter slcan:/dev/ttyACM0) [--seconds N] [--raw] [--diagnostic] [--dbc FILE]"
                .into(),
        );
    }
    // --raw is a switch of its own: taken out before the options are read (parse_args refused it as unknown, so it never worked)
    let raw = a.iter().any(|x| x == "--raw");
    // --diagnostic: only IDs 700-7FF (another tool's requests and the modules' answers), when the whole bus is more than an
    // ELM/STN adapter's serial link carries
    let diagnostic = a.iter().any(|x| x == "--diagnostic");
    let rest: Vec<String> = a[1..]
        .iter()
        .filter(|x| *x != "--raw" && *x != "--diagnostic")
        .cloned()
        .collect();
    let args = parse_args(&rest, &["adapter", "can", "seconds", "protocol", "dbc"])?;
    // a CAN database (ICA-6): what each ID's bytes mean, shown after the table
    let dbc = args
        .opt("dbc")
        .map(|p| {
            std::fs::read_to_string(p)
                .map(|t| icarus_core::dbc::parse(&t))
                .map_err(|x| format!("{p}: {x}"))
        })
        .transpose()?;
    if let Some(d) = &dbc {
        for s in &d.skipped {
            eprintln!("note: the DBC: {s}");
        }
    }
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
    let ended = if let Some(ifn) = can_interface(&args) {
        let sock = socketcan::open(&ifn).map_err(|x| x.to_string())?;
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
    } else if let Some((path, baud, bitrate)) = slcan_target(&args) {
        // listen-only (L), never O: the adapter acknowledges nothing and sends nothing
        let port = serial(&path, baud).map_err(|x| format!("{path}: {x}"))?;
        if raw {
            println!(
                "{}",
                icarus_obd::drivelog::adapter_line(&format!("SLCAN {path} at {bitrate} bit/s"))
            );
        }
        watch_slcan(port, bitrate, start, |f| step(&mut an, f))?
    } else {
        // listen only: the adapter is set up without the protocol search's request (it sent one 0100 before, 2026-11-22)
        let mut elm = listen_elm(&args)?;
        let proto = elm.protocol;
        if raw {
            // the adapter first, so a firmware change shows in the log; bus::parse_line skips this line
            println!(
                "{}",
                icarus_obd::drivelog::adapter_line(&icarus_obd::link::Link::adapter(&elm))
            );
        }
        elm.monitor_ids(diagnostic.then_some((0x700, 0x700)), |line| {
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
    if let Some(d) = &dbc {
        println!("\nWhat the DBC names in the last frame of each ID:");
        let mut named = 0;
        for (id, s) in &an.ids {
            let Some((m, values)) = d.decode(*id, *id > 0x7FF, &s.data) else {
                continue;
            };
            named += 1;
            let shown: Vec<String> = values
                .iter()
                .map(|v| match &v.label {
                    Some(l) => format!("{} {l}", v.signal),
                    None => format!(
                        "{} {}{}",
                        v.signal,
                        dbc_number(v.value),
                        if v.unit.is_empty() {
                            String::new()
                        } else {
                            format!(" {}", v.unit)
                        }
                    ),
                })
                .collect();
            let idt = if *id > 0x7FF {
                format!("{id:08X}")
            } else {
                format!("{id:03X}")
            };
            println!("  {idt:>8}  {}: {}", m.name, shown.join(", "));
        }
        if named == 0 {
            println!("  (none of the IDs seen is in it)");
        }
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
    if !an.services.is_empty() {
        println!("\nService routines seen started by another tool (add them to the vehicle's file after checking what each does):\n");
        for s in &an.services {
            println!("{}", icarus_obd::service::to_toml(s));
        }
    }
    Ok(an)
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
        return Err("--ask-sarina: Sarina is not installed".into());
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
    let can_if = can_interface(&args);
    let slcan_t = slcan_target(&args);
    let (vin, source, vehicle) = if let Some(ifn) = &can_if {
        (
            None,
            format!("the CAN interface {ifn}"),
            "a vehicle Icarus did not ask (CAN interface)".to_string(),
        )
    } else if let Some((path, ..)) = &slcan_t {
        (
            None,
            format!("the SLCAN adapter {path}"),
            "a vehicle Icarus did not ask (SLCAN adapter)".to_string(),
        )
    } else {
        let mut elm = connect_elm(&args)?;
        let vin = j1979::vin(&mut elm).map_err(e)?;
        note_dropped(&mut elm);
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
    if let Some(ifn) = &can_if {
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
    if let Some((path, baud, bitrate)) = &slcan_t {
        eprintln!("Learning on the SLCAN adapter {path}, opened listen-only. Type what you are about to do and press Enter before each change; \"stop\" or Ctrl+C ends.");
        let port = serial_port(path, *baud).map_err(|x| format!("{path}: {x}"))?;
        watch_slcan(port, *bitrate, start, |f| step(&mut an, f))?;
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
            let locked = lock::read_only();
            if all.is_empty() && locked.is_empty() {
                println!("No vehicles named yet: icarus garage VIN \"2017 Toyota Tacoma 3.5L V6\"");
            }
            for (vin, name) in &all {
                let tag = if locked.contains(vin) {
                    "  (Read-Only)"
                } else {
                    ""
                };
                println!("{vin}  {name}{tag}");
            }
            for vin in locked.iter().filter(|v| !all.contains_key(*v)) {
                println!("{vin}  (Read-Only, not named)");
            }
            Ok(())
        }
        [what, v, rest @ ..] if what == "tires" => {
            let i = vin::decode(v)?;
            if rest == ["off"] {
                garage::set_tires(&i.vin, None)?;
                println!("{}: tire setting forgotten.", i.vin);
                return Ok(());
            }
            let args = parse_args(rest, &["at", "gear", "installed"])?;
            let [stock, installed] = args.pos.as_slice() else {
                return Err("usage: icarus garage tires VIN STOCK INSTALLED [--at wheels|transmission] [--gear STOCK_RATIO --installed NEW_RATIO], or VIN off".into());
            };
            let ratio = |k: &str| {
                args.opt(k)
                    .map(|v| {
                        v.parse::<f64>()
                            .map_err(|_| format!("--{k} needs a ratio like 3.91"))
                    })
                    .transpose()
            };
            let t = garage::TireSetting {
                stock: stock.clone(),
                installed: installed.clone(),
                at_transmission: match args.opt("at") {
                    None | Some("wheels") => false,
                    Some("transmission") => true,
                    Some(o) => return Err(format!("--at wheels or transmission, not {o}")),
                },
                stock_ratio: ratio("gear")?,
                installed_ratio: ratio("installed")?,
            };
            let f = t.factor()?;
            garage::set_tires(&i.vin, Some(t))?;
            println!(
                "{}: saved. Its speed reading times {f:.3} is the real speed (icarus obd live shows it corrected).",
                i.vin
            );
            Ok(())
        }
        [what, v] if what == "history" => {
            let i = vin::decode(v)?;
            let h = icarus_obd::history::load(&i.vin);
            if h.is_empty() {
                println!(
                    "No codes in {}'s history yet (each reading of its codes is kept).",
                    i.vin
                );
            }
            for s in &h {
                println!("{}", icarus_obd::history::describe(s));
            }
            Ok(())
        }
        [what, v, on] if what == "read-only" && (on == "on" || on == "off") => {
            let i = vin::decode(v)?;
            lock::set(&i.vin, on == "on")?;
            println!(
                "{}",
                if on == "on" {
                    format!("{}: Read-Only on. Icarus reads it and clears, tests and changes nothing on it.", i.vin)
                } else {
                    format!(
                        "{}: Read-Only off (each write still has its own checks).",
                        i.vin
                    )
                }
            );
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

fn obd_pids(elm: &mut dyn Link) -> Result<(), String> {
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

fn obd_live(elm: &mut dyn Link, args: &Args) -> Result<(), String> {
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
    // the vehicle's saved tires (icarus garage tires): its speed, corrected, in a column of its own
    let factor = if pids.contains(&0x0D) {
        j1979::vin(elm)
            .ok()
            .flatten()
            .and_then(|v| garage::tires(&v))
            .and_then(|t| t.factor().ok())
    } else {
        None
    };
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
    let mut names = names;
    if let Some(f) = factor {
        names.push(format!(
            "Vehicle speed, corrected for the tires (x{f:.3}) [km/h]"
        ));
    }
    // --gps SOURCE: position and speed from a receiver in columns of their own (only in this file, never sent)
    let gps = args
        .opt("gps")
        .map(icarus_obd::gps::Follower::start)
        .transpose()?;
    if gps.is_some() {
        names.extend(icarus_obd::gps::COLUMNS.iter().map(|c| c.to_string()));
    }
    if let Some(f) = csv.as_mut() {
        writeln!(
            f,
            "{}",
            icarus_obd::drivelog::adapter_line(&icarus_obd::link::Link::adapter(&*elm))
        )
        .map_err(|e| e.to_string())?;
    }
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
    // ICA-86: after the first minute, a value never answered or copying another is no longer asked (its column stays,
    // empty), so the rest are read more often; --all asks every value the whole time
    let mut pruner = (args.opt("all").is_none()).then(|| icarus_obd::prune::Pruner::new(&pids));
    let mut asked = pids.clone();
    loop {
        let t0 = Instant::now();
        // up to six values a request on CAN; the engine's answer when it gives one, else the first module's
        let all = j1979::read_many(elm, &asked).map_err(|x| x.to_string())?;
        let value = |p: u8| {
            all.iter()
                .find(|(q, _)| *q == p)
                .and_then(|(_, r)| r.iter().find(|x| x.from == "7E8").or(r.first()))
                .map(|x| x.value)
        };
        let mut row: Vec<String> = pids
            .iter()
            .map(|p| value(*p).map(|v| format!("{v:.2}")).unwrap_or_default())
            .collect();
        if let Some(f) = factor {
            row.push(
                value(0x0D)
                    .map(|v| format!("{:.2}", v * f))
                    .unwrap_or_default(),
            );
        }
        if let Some(g) = &gps {
            row.extend(
                g.cells()
                    .iter()
                    .map(|c| c.map(|v| format!("{v:.6}")).unwrap_or_default()),
            );
        }
        let secs = start.elapsed().as_secs_f64();
        if let Some(p) = pruner.as_mut() {
            p.see(secs, &pids.iter().map(|q| value(*q)).collect::<Vec<_>>());
            if let Some(dropped) = p.decide(secs) {
                for w in icarus_obd::prune::words(&dropped) {
                    eprintln!("note: {w} (--all keeps asking)");
                }
                asked.retain(|q| !dropped.iter().any(|(d, _)| d == q));
                pruner = None;
            }
        }
        let t = format!("{secs:.2}");
        println!("{t}, {}", row.join(", "));
        // a module left out of a value's answer is said as it happens (once), not only when the watch ends
        note_dropped(elm);
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

#[cfg(test)]
mod tests {
    use super::*;

    /// A recording made with --gps has the GPS's three columns, one made without has none (both on the simulators, into
    /// a file of the test's own).
    #[test]
    fn a_recording_with_and_without_gps() {
        let dir = std::env::temp_dir().join(format!("icarus-rec-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        for (gps, file) in [(true, "with.csv"), (false, "without.csv")] {
            let path = dir.join(file).display().to_string();
            let mut a: Vec<String> = ["0C", "--count", "2", "--every", "100", "--csv", &path]
                .iter()
                .map(|x| x.to_string())
                .collect();
            if gps {
                a.extend(["--gps".to_string(), "sim".to_string()]);
            }
            let args = parse_args(&a, &["count", "every", "csv", "gps"]).unwrap();
            let mut elm = Elm::connect(Box::new(icarus_obd::sim::SimCar::new())).unwrap();
            obd_live(&mut elm, &args).unwrap();
            let text = std::fs::read_to_string(&path).unwrap();
            let header = text.lines().find(|l| l.starts_with("time_s")).unwrap();
            assert_eq!(header.contains("GPS latitude"), gps, "{header}");
            assert_eq!(header.contains("GPS speed [km/h]"), gps, "{header}");
        }
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A 2x2 map, written for this test.
    const DEF: &str = r#"<XDFFORMAT version="1.70">
  <XDFHEADER><deftitle>Test</deftitle></XDFHEADER>
  <XDFTABLE uniqueid="0x1">
    <title>Spark</title>
    <XDFAXIS id="x" uniqueid="0x0"><indexcount>2</indexcount></XDFAXIS>
    <XDFAXIS id="y" uniqueid="0x0"><indexcount>2</indexcount></XDFAXIS>
    <XDFAXIS id="z">
      <EMBEDDEDDATA mmedaddress="0x10" mmedelementsizebits="8" mmedrowcount="2" mmedcolcount="2" />
      <units>deg</units>
      <decimalpl>1</decimalpl>
      <MATH equation="X/2"><VAR id="X" /></MATH>
    </XDFAXIS>
  </XDFTABLE>
</XDFFORMAT>"#;

    /// ICA-58: `icarus set ... --version NAME` keeps the change as a version (no image written), refused with --out; `icarus
    /// version rename` and `delete` then act on that file only.
    #[test]
    fn set_keeps_a_version_and_version_renames_and_deletes_it() {
        let dir =
            std::env::temp_dir().join(format!("icarus-cli-setversion-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let image = dir.join("stock.bin");
        let mut stock = vec![0u8; 0x20];
        stock[0x10..0x14].copy_from_slice(&[20, 30, 40, 50]);
        std::fs::write(&image, &stock).unwrap();
        let defp = dir.join("test.xdf");
        std::fs::write(&defp, DEF).unwrap();
        let s = |v: &[&str]| v.iter().map(|x| x.to_string()).collect::<Vec<_>>();
        let (img, d) = (image.to_str().unwrap(), defp.to_str().unwrap());
        assert!(cmd_set(&s(&[
            img,
            d,
            "Spark",
            "21",
            "--row",
            "0",
            "--col",
            "1",
            "--version",
            "Tow",
            "--out",
            "x.bin"
        ]))
        .unwrap_err()
        .contains("one or the other"));
        cmd_set(&s(&[
            img,
            d,
            "Spark",
            "21",
            "--row",
            "0",
            "--col",
            "1",
            "--version",
            "Tow",
            "--note",
            "towing",
        ]))
        .unwrap();
        let vs = icarus_core::version::list(&image, &stock);
        assert_eq!(vs.len(), 1);
        let (p, v) = &vs[0];
        assert_eq!(
            (v.name.as_str(), v.note.as_str(), v.changes.len()),
            ("Tow", "towing", 1)
        );
        assert_eq!(
            (v.changes[0].offset, v.changes[0].now.clone()),
            (0x11, vec![42])
        );
        assert_eq!(std::fs::read(&image).unwrap(), stock, "stock untouched");
        let files: Vec<String> = std::fs::read_dir(&dir)
            .unwrap()
            .flatten()
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .filter(|n| n.ends_with(".bin"))
            .collect();
        assert_eq!(files, ["stock.bin"], "no image written");
        cmd_version(&s(&["rename", img, p.to_str().unwrap(), "Towing"])).unwrap();
        let vs = icarus_core::version::list(&image, &stock);
        assert_eq!(vs[0].1.name, "Towing");
        assert!(!p.exists());
        cmd_version(&s(&["delete", vs[0].0.to_str().unwrap()])).unwrap();
        assert!(icarus_core::version::list(&image, &stock).is_empty() && image.exists());
        assert!(cmd_version(&s(&["delete", img]))
            .unwrap_err()
            .contains("not a version file"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn diff_shows_what_a_saved_version_changes() {
        let dir = std::env::temp_dir().join(format!("icarus-cli-version-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let oem = dir.join("Truck").join("OEM tunes");
        std::fs::create_dir_all(&oem).unwrap();
        let image = oem.join("stock.bin");
        let mut stock = vec![0u8; 0x20];
        stock[0x10..0x14].copy_from_slice(&[20, 30, 40, 50]);
        std::fs::write(&image, &stock).unwrap();
        let def = icarus_core::def::parse_any(DEF.as_bytes())
            .unwrap()
            .definition;
        // as on the Maps page: a cell changed, then "Save as version"
        let mut rom = Rom::new(stock.clone());
        let Kind::Table(t) = &def.element("Spark").unwrap().kind else {
            panic!()
        };
        def.set(&mut rom, &t.z, 1, 0, 22.0).unwrap();
        let (path, _) =
            icarus_core::version::save_edits(&rom, Some(&def), &image, "More spark", "").unwrap();
        assert_eq!(
            path,
            dir.join("Truck/Custom tunes/stock - More spark.version.toml")
        );
        let v = icarus_core::version::load(&path).unwrap();
        assert_eq!(v.changes[0].elements, vec!["Spark"]);
        let lines = diff_report(Some(&def), &stock, &v.apply(&stock).unwrap()).unwrap();
        assert_eq!(lines[0], "1 byte(s) differ in 1 place(s).");
        assert!(lines.iter().any(|l| l == "Spark (map)"), "{lines:?}");
        assert!(
            lines
                .iter()
                .any(|l| l.trim_end() == "  at 1 / 0: 20.0 -> 22.0 deg"),
            "{lines:?}"
        );
        assert_eq!(
            std::fs::read(&image).unwrap(),
            stock,
            "stock stays as it was"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    use icarus_obd::canlink::{serve, CanLink, SimBus};

    /// `icarus obd info --adapter can0`: the command's own steps over the CAN interface's link, the simulated vehicle on an
    /// in-memory bus in place of the interface (no interface, no root).
    #[test]
    fn obd_info_over_a_can_interface() {
        assert_eq!(Target::parse("can0").unwrap(), Target::Can("can0".into()));
        let mut link = CanLink::connect(SimBus::new(sim::SimCar::new()), "can0").unwrap();
        obd_info(&mut link).unwrap();
        // it asked what obd info shows: the VIN, the modules' names and calibrations, the monitors
        let asked: Vec<Vec<u8>> = link
            .frames()
            .sent
            .iter()
            .filter(|f| f.id == 0x7DF)
            .map(|f| f.data[1..1 + f.data[0] as usize].to_vec())
            .collect();
        for want in [
            &[0x09, 0x02][..],
            &[0x09, 0x0A],
            &[0x09, 0x04],
            &[0x01, 0x01],
        ] {
            assert!(asked.iter().any(|a| a == want), "{want:02X?}: {asked:02X?}");
        }
        note_dropped(&mut link);
    }

    /// `icarus bus watch --adapter slcan:/dev/ttyACM0`, the serial port being a recorded SLCAN session: its frames are
    /// read (11-bit and 29-bit; remote frames passed over), and the channel was opened listen-only and closed, never `O`.
    #[test]
    fn bus_watch_reads_a_recorded_slcan_session_listen_only() {
        use icarus_obd::slcan::recorded::Session;
        let (session, sent) = Session::new(&[
            "t2C48031AF8001C000000",
            "t3B750100500000",
            "T18DAF1103410D00",
            "r7DF8",
            "t2C48031AF9001C000000",
            "t610420000000",
        ]);
        let session = std::cell::RefCell::new(Some(session));
        let a: Vec<String> = [
            "watch",
            "--adapter",
            "slcan:/dev/ttyACM0",
            "--seconds",
            "0.3",
        ]
        .map(String::from)
        .to_vec();
        let an = bus_watch(&a, &|path, baud| {
            assert_eq!((path, baud), ("/dev/ttyACM0", 115_200));
            Ok(Box::new(session.borrow_mut().take().unwrap()))
        })
        .unwrap();
        assert_eq!(an.frames, 5, "every data frame, the remote one passed over");
        for id in [0x2C4, 0x3B7, 0x610, 0x18DA_F110] {
            assert!(an.ids.contains_key(&id), "{id:X}");
        }
        assert_eq!(an.ids[&0x2C4].count, 2);
        let log = sent.lock().unwrap().clone();
        assert_eq!(log, ["C\r", "S6\r", "L\r", "C\r"]);
        assert!(!log.iter().any(|c| c.starts_with('O')), "{log:?}");
        // diagnostics are never sent through one
        let args = parse_args(&a[1..3], &["adapter"]).unwrap();
        let why = connect(&args).err().unwrap();
        assert!(why.contains("listen-only"), "{why}");
    }

    /// The same through `--adapter vcan0` on a real virtual CAN interface, when one exists (made once with root:
    /// `sudo ip link add vcan0 type vcan && sudo ip link set vcan0 up`): a thread plays the simulated vehicle on a second
    /// socket.
    #[test]
    #[ignore = "needs a vcan0 interface"]
    fn obd_info_over_a_real_vcan0() {
        let Ok(mut car_side) = socketcan::open("vcan0") else {
            eprintln!("no vcan0 here: skipped");
            return;
        };
        let stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let done = stop.clone();
        let car = std::thread::spawn(move || {
            let mut sim = SimBus::new(sim::SimCar::new());
            serve(&mut car_side, &mut sim, || {
                done.load(std::sync::atomic::Ordering::Relaxed)
            })
        });
        let args = parse_args(&["--adapter".into(), "vcan0".into()], &["adapter"]).unwrap();
        let link = connect(&args);
        let r = link.and_then(|mut l| {
            assert_eq!(l.adapter(), "SocketCAN vcan0");
            obd_info(&mut *l)
        });
        stop.store(true, std::sync::atomic::Ordering::Relaxed);
        car.join().unwrap().unwrap();
        r.unwrap();
    }
}
