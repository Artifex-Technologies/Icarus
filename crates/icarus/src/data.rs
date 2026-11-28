//! Module data on the command line: `icarus import obdb`, `icarus import btlog|buslog` (a capture of another tool at
//! work), `icarus data` (offline) and `icarus obd data` (on the vehicle,
//! read-only: services 21 and 22 only, paced, never on 7DF or an answer ID, airbag modules identity only).

use super::{connect, e, parse_args, profiles, Args};
use icarus_core::moddata::{self as md, capture, obdb, units, ModuleDef, Shown, ValueDef};
use icarus_obd::link::Link;
use icarus_obd::moddata::{self, Loaded, Outcome, Pacer, Scope, State};
use icarus_obd::{actuator, garage, j1979, naming, report, uds, vehicles, vin};
use std::collections::BTreeMap;
use std::io::Write;
use std::time::{Duration, Instant};

fn today() -> String {
    report::now()[..10].to_string()
}

fn hex_id(s: &str) -> Result<u32, String> {
    u32::from_str_radix(s.trim_start_matches("0x"), 16)
        .map_err(|_| format!("{s} is not a module ID in hex"))
}

const CAPTURE_USAGE: &str =
    "usage: icarus import btlog|buslog FILE [--name SET] [--make NAME] [--transcript OUT.csv] \
                             [--keep-vin] [--dry-run] [--replace]";

/// `icarus import obdb FILE.json [--name SET] [--make NAME] [--dry-run] [--replace]`, `icarus import abrp FILE.json ...`,
/// `icarus import btlog|buslog FILE ...`
pub fn cmd_import(a: &[String]) -> Result<(), String> {
    let args = parse_args(
        a.get(1..).unwrap_or(&[]),
        &["name", "make", "model", "transcript"],
    )?;
    match a.first().map(String::as_str) {
        Some("obdb") => import_obdb(&args),
        Some("abrp") => import_abrp(&args),
        Some("btlog" | "buslog") => import_capture(&args),
        Some("ddt") => import_ddt(&args),
        _ => Err(format!(
            "usage: icarus import obdb FILE.json [--name SET] [--make NAME] [--dry-run] [--replace]\n{ABRP_USAGE}\n{CAPTURE_USAGE}"
        )),
    }
}

/// A module the source names only by its address takes the make's profile's name for it.
fn name_from_profile(imp: &mut moddata::Imported, all: &[vehicles::Profile]) {
    let Some(p) = imp
        .set
        .as_ref()
        .and_then(|s| all.iter().find(|p| p.id == s.make))
    else {
        return;
    };
    for m in imp
        .modules
        .iter_mut()
        .filter(|m| m.module.kind.is_empty() && m.module.ext.is_none())
    {
        if let Some(a) = p
            .modules
            .iter()
            .find(|a| a.tx == m.module.tx && a.ext.is_none())
        {
            m.module.name = a.name.clone();
        }
    }
}

fn import_obdb(args: &Args) -> Result<(), String> {
    let [path] = args.pos.as_slice() else {
        return Err("usage: icarus import obdb FILE.json [--name SET] [--make NAME] [--dry-run] [--replace]".into());
    };
    let text = std::fs::read_to_string(path).map_err(|x| format!("{path}: {x}"))?;
    let imp = obdb::parse(
        &text,
        &obdb::Options {
            path: path.clone(),
            name: args.opt("name").map(str::to_string),
            make: args.opt("make").map(str::to_string),
            today: today(),
        },
    )?;
    finish_import("OBDb signal set", path, imp, args)
}

/// `icarus import ddt ECU.xml [--name SET] [--make NAME] [--dry-run] [--replace]`: a DDT2000 ECU file (ICA-5), the user's
/// own copy (Renault's material, never shipped with Icarus).
fn import_ddt(args: &Args) -> Result<(), String> {
    let [path] = args.pos.as_slice() else {
        return Err(
            "usage: icarus import ddt ECU.xml [--name SET] [--make NAME] [--dry-run] [--replace]"
                .into(),
        );
    };
    let text = std::fs::read_to_string(path).map_err(|x| format!("{path}: {x}"))?;
    let imp = md::ddt::parse(
        &text,
        &md::ddt::Options {
            path: path.clone(),
            name: args.opt("name").map(str::to_string),
            make: args.opt("make").map(str::to_string),
            today: today(),
        },
    )?;
    finish_import("DDT2000 ECU file", path, imp, args)
}

/// `icarus import abrp FILE.json [--name SET] [--make NAME] [--model WORDS] [--dry-run] [--replace]`: an EV PID list from
/// A Better Routeplanner's ev-obd-pids.
fn import_abrp(args: &Args) -> Result<(), String> {
    let [path] = args.pos.as_slice() else {
        return Err(ABRP_USAGE.into());
    };
    let text = std::fs::read_to_string(path).map_err(|x| format!("{path}: {x}"))?;
    let imp = md::abrp::parse(
        &text,
        &md::abrp::Options {
            path: path.clone(),
            name: args.opt("name").map(str::to_string),
            make: args.opt("make").map(str::to_string),
            model: args.opt("model").map(str::to_string),
            today: today(),
        },
    )?;
    finish_import("ABRP PID list", path, imp, args)
}

const ABRP_USAGE: &str =
    "usage: icarus import abrp FILE.json [--name SET] [--make NAME] [--model WORDS] [--dry-run] [--replace]";

/// What an importer made, checked against the vehicle profiles and the gates, listed, and written unless --dry-run.
fn finish_import(
    kind: &str,
    path: &str,
    mut imp: moddata::Imported,
    args: &Args,
) -> Result<(), String> {
    let all = profiles();
    if let Some(set) = imp.set.as_mut() {
        match vehicles::find(&all, &set.make) {
            Ok(p) => set.make = p.id.clone(),
            Err(_) => eprintln!(
                "note: no vehicle profile named {:?}; the set will only be used with --set (or import with --make NAME, see icarus vehicles)",
                set.make
            ),
        }
    }
    name_from_profile(&mut imp, &all);
    moddata::gate_imported(&mut imp, &all);
    let set = imp.set.clone().ok_or("no set")?;
    println!(
        "{kind} {path}\n  set {:?} for {} {}",
        set.name,
        set.make,
        set.models.join(" ")
    );
    let (mut requests, mut values) = (0, 0);
    for m in &imp.modules {
        let n: usize = m.requests.iter().map(|r| r.values.len()).sum();
        let unv = m
            .requests
            .iter()
            .filter(|r| !r.unverified.is_empty())
            .count();
        requests += m.requests.len();
        values += n;
        let how = match moddata::reachable(m) {
            Ok(()) => String::new(),
            Err(why) => format!("  ({why})"),
        };
        println!(
            "  {:<8} {:<30} {:>3} request(s), {:>4} value(s), {unv} not verified for every year{how}",
            m.module.label(),
            m.module.name,
            m.requests.len(),
            n
        );
    }
    println!(
        "  {requests} request(s), {values} value(s) in {} module(s)",
        imp.modules.len()
    );
    if !imp.left_out.is_empty() {
        let mut why: BTreeMap<&str, usize> = BTreeMap::new();
        for (_, w) in &imp.left_out {
            *why.entry(w.as_str()).or_default() += 1;
        }
        println!("Left out ({}):", imp.left_out.len());
        for (w, n) in why {
            println!("  {n:>4}  {w}");
        }
    }
    if !imp.notes.is_empty() {
        println!("Assumed ({}; the first few):", imp.notes.len());
        for n in imp.notes.iter().take(8) {
            println!("  {n}");
        }
    }
    println!("{}", set.source.attribution);
    if args.opt("dry-run").is_some() {
        println!("(--dry-run: nothing written)");
        return Ok(());
    }
    let root = moddata::user_dir().ok_or("no config folder")?;
    let dir = moddata::write_set(&root, &imp, args.opt("replace").is_some())?;
    println!("Written to {}", dir.display());
    Ok(())
}

/// The largest capture read: a phone's Bluetooth log of an hour is far smaller, and a bus log is held whole in memory
/// as frames (several times its size).
const MAX_CAPTURE: u64 = 256 << 20;

/// What a capture gave: what was read (VINs masked unless kept, module serial numbers always), the set (gates applied), and
/// where its make came from.
struct Learned {
    read: capture::Read,
    imp: moddata::Imported,
    masked: usize,
    /// Exchanges that held a module's serial number (masked).
    serials: usize,
    make_from: &'static str,
}

/// A capture's bytes as a transcript and a module-data set for `all`'s vehicles (no file written here).
fn learn(
    bytes: &[u8],
    file_name: &str,
    name: Option<&str>,
    make: Option<&str>,
    keep_vin: bool,
    all: &[vehicles::Profile],
) -> Result<Learned, String> {
    let mut read = capture::read(bytes)?;
    // the VIN (never written or shown) says the make and the model year when --make does not
    let vin = capture::find_vin(&read.exchanges);
    let masked = if keep_vin {
        0
    } else {
        capture::mask_vins(&mut read.exchanges)
    };
    // a module's serial number, whatever --keep-vin says: no set's value needs it, and it tells one module (and so one
    // vehicle) from every other (ICA-106)
    let serials = capture::mask_serials(&mut read.exchanges);
    let years: Vec<[u16; 2]> = vin
        .as_deref()
        .and_then(|v| vin::decode(v).ok())
        .and_then(|i| i.year)
        .map(|y| vec![[y, y]])
        .unwrap_or_default();
    let (make, make_from) = match make {
        Some(m) => match vehicles::find(all, m) {
            Ok(p) => (p.id.clone(), "--make"),
            Err(_) => (m.to_lowercase(), "--make (no vehicle profile of that name)"),
        },
        None => match vin.as_deref().and_then(|v| vehicles::for_vin(all, v)) {
            Some(p) => (p.id.clone(), "the VIN in the capture"),
            None => (String::new(), ""),
        },
    };
    let mut imp = capture::build(
        &read.exchanges,
        &capture::Options {
            file_name: file_name.to_string(),
            name: name.map(str::to_string),
            make,
            years,
            today: today(),
            from: read.from.to_string(),
            sha256: capture::digest(bytes),
        },
    )?;
    name_from_profile(&mut imp, all);
    moddata::gate_imported(&mut imp, all);
    // a module left with nothing (every request refused: listed in left_out) gets no file
    imp.modules
        .retain(|m| !m.requests.is_empty() || !m.module.identity.is_empty());
    Ok(Learned {
        read,
        imp,
        masked,
        serials,
        make_from,
    })
}

/// `icarus import btlog|buslog FILE [--name SET] [--make NAME] [--transcript OUT.csv] [--keep-vin] [--dry-run]
/// [--replace]`: either name reads either kind of capture (told apart by its first bytes).
fn import_capture(args: &Args) -> Result<(), String> {
    let [path] = args.pos.as_slice() else {
        return Err(CAPTURE_USAGE.into());
    };
    let size = std::fs::metadata(path)
        .map_err(|x| format!("{path}: {x}"))?
        .len();
    if size > MAX_CAPTURE {
        return Err(format!(
            "{path}: {size} bytes is more than Icarus reads from one capture ({MAX_CAPTURE}); capture a shorter session"
        ));
    }
    let bytes = std::fs::read(path).map_err(|x| format!("{path}: {x}"))?;
    let file_name = std::path::Path::new(path)
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or("capture")
        .to_string();
    let all = profiles();
    let keep_vin = args.opt("keep-vin").is_some();
    let l = learn(
        &bytes,
        &file_name,
        args.opt("name"),
        args.opt("make"),
        keep_vin,
        &all,
    )?;
    let set = l.imp.set.clone().ok_or("no set")?;
    // with --transcript -, the CSV alone goes to stdout and everything else to stderr
    let csv_out = args.opt("transcript") == Some("-");
    macro_rules! say {
        ($($t:tt)*) => {
            if csv_out {
                eprintln!($($t)*)
            } else {
                println!($($t)*)
            }
        };
    }
    say!("{} {path} ({})", capitalised(l.read.from), l.read.format);
    for n in &l.read.notes {
        say!("  {n}");
    }
    say!(
        "  Kept: only the diagnostic conversation (each request and what came back). {}",
        if l.read.from.starts_with("Bluetooth") {
            "Bluetooth addresses, device names, link keys and every other Bluetooth channel are never read into any output; adapter commands (AT, ST) are counted, not kept."
        } else {
            "Every other frame on the bus is ignored."
        }
    );
    if keep_vin {
        say!("  VINs kept as they are (--keep-vin): the transcript and the set hold the vehicle's VIN");
    } else if l.masked > 0 {
        say!(
            "  {} exchange(s) held a VIN (or text like one): masked in the transcript and the set (--keep-vin keeps it)",
            l.masked
        );
    }
    if l.serials > 0 {
        say!(
            "  {} exchange(s) held a module's serial number: masked in the transcript and the set",
            l.serials
        );
    }
    // what was asked, by service
    let mut by: BTreeMap<u8, (usize, usize)> = BTreeMap::new();
    for e in &l.read.exchanges {
        if let Some(&s) = e.request.first() {
            let c = by.entry(s).or_default();
            c.0 += 1;
            c.1 += usize::from(
                e.replies
                    .iter()
                    .any(|r| r.data.first() == Some(&s.wrapping_add(0x40))),
            );
        }
    }
    say!("  {} request(s) to the vehicle:", l.read.exchanges.len());
    for (s, (n, ok)) in &by {
        say!(
            "    {s:02X} {:<34} {n:>5} sent, {ok:>5} answered",
            icarus_core::diag::service_name(*s)
        );
    }
    say!(
        "\n  set {:?} for {}{}",
        set.name,
        if set.make.is_empty() {
            "no make (it is used only with --set; and on a make not known, a module of unknown kind is refused: it could be the airbag module; --make NAME)".to_string()
        } else {
            format!("{} (from {})", set.make, l.make_from)
        },
        if set.years.is_empty() {
            String::new()
        } else {
            format!(", model year {}", years(&set.years))
        }
    );
    let mut values = 0;
    for m in &l.imp.modules {
        values += m.requests.len();
        let how = match moddata::reachable(m) {
            Ok(()) => String::new(),
            Err(why) => format!("  ({why})"),
        };
        say!(
            "  {:<12} {:<34} {:>3} identifier(s) kept{how}",
            m.module.label(),
            m.module.name,
            m.requests.len()
        );
    }
    say!(
        "  {values} identifier(s) in {} module(s), each one unnamed placeholder value, not verified",
        l.imp.modules.len()
    );
    if !l.imp.left_out.is_empty() {
        let mut why: BTreeMap<&str, Vec<&str>> = BTreeMap::new();
        for (w, y) in &l.imp.left_out {
            why.entry(y.as_str()).or_default().push(w.as_str());
        }
        say!("Seen, not imported ({}):", l.imp.left_out.len());
        for (y, ws) in why {
            let more = if ws.len() > 6 {
                format!(" and {} more", ws.len() - 6)
            } else {
                String::new()
            };
            say!(
                "  {:>4}  {y}  [{}{more}]",
                ws.len(),
                ws.iter().take(6).copied().collect::<Vec<_>>().join(", ")
            );
        }
    }
    for n in &l.imp.notes {
        say!("Assumed: {n}");
    }
    say!("{}", set.source.attribution);
    if let Some(out) = args.opt("transcript") {
        let csv = capture::transcript_csv(&l.read.exchanges, &|c| uds::nrc(c).to_string());
        if out == "-" {
            print!("{csv}");
        } else {
            std::fs::write(out, csv).map_err(|x| format!("{out}: {x}"))?;
            say!(
                "Transcript: {} exchange(s) written to {out} (time_s from the start of the capture{})",
                l.read.exchanges.len(),
                if l.read.from.starts_with("Bluetooth") {
                    ", clock_utc the phone's clock in UTC"
                } else if l.read.exchanges.iter().any(|e| e.clock.is_some()) {
                    ", clock_utc the logging computer's clock in UTC"
                } else {
                    ""
                }
            );
        }
    }
    if args.opt("dry-run").is_some() {
        say!("(--dry-run: no set written)");
        return Ok(());
    }
    let root = moddata::user_dir().ok_or("no config folder")?;
    let dir = moddata::write_set(&root, &l.imp, args.opt("replace").is_some())?;
    say!(
        "Written to {}. To name a value once you know what it is: icarus data name {} MODULE RAW_... --name TEXT (with \
         --byte, --bits, --mul/--div/--add, --unit), and see it decode the samples with icarus data {} --sample",
        dir.display(),
        set.name,
        set.name
    );
    Ok(())
}

fn capitalised(s: &str) -> String {
    let mut c = s.chars();
    c.next()
        .map(|f| f.to_uppercase().collect::<String>() + c.as_str())
        .unwrap_or_default()
}

fn print_value(v: &ValueDef) {
    let mut how = format!("byte {}", v.byte);
    if v.bit != 0 || !v.bits.is_multiple_of(8) {
        how += &format!(" bit {}", v.bit);
    }
    how += &format!(", {} bit(s)", v.bits);
    if v.order != md::Order::Msb {
        how += &format!(" {}", v.order.word());
    }
    if v.signed {
        how += ", signed";
    }
    let unit = if v.unit.is_empty() {
        String::new()
    } else {
        format!(" [{}]", v.unit)
    };
    println!(
        "      {:<28} {}{unit}  ({how}){}",
        v.id,
        v.name,
        if v.hidden { " hidden" } else { "" }
    );
    if !v.note.is_empty() {
        println!("        note: {}", v.note);
    }
}

fn years(y: &[[u16; 2]]) -> String {
    y.iter()
        .map(|[a, b]| match (a, b) {
            (0, 9999) => "every year".to_string(),
            (0, b) => format!("to {b}"),
            (a, 9999) => format!("{a} on"),
            (a, b) if a == b => a.to_string(),
            (a, b) => format!("{a}-{b}"),
        })
        .collect::<Vec<_>>()
        .join(", ")
}

/// The options `icarus data` takes with a value.
const DATA_OPTIONS: [&str; 13] = [
    "module", "find", "make", "name", "id", "byte", "bit", "bits", "mul", "div", "add", "unit",
    "kind",
];

/// `icarus data [SET] [--module ID] [--find TEXT] [--sample] [--refused]`, `icarus data check FILE.toml [--make NAME]`,
/// `icarus data name SET MODULE VALUE_ID --name TEXT ...`
pub fn cmd_data(a: &[String]) -> Result<(), String> {
    let args = parse_args(a, &DATA_OPTIONS)?;
    let all = profiles();
    let user = moddata::user_dir();
    match args.pos.first().map(String::as_str) {
        Some("check") => return data_check(&args, &all),
        Some("name") => return data_name(&args),
        _ => {}
    }
    let Some(name) = args.pos.first() else {
        let (sets, problems) = moddata::load_all(user.as_deref(), &all);
        for p in problems {
            eprintln!("note: {p}");
        }
        if sets.is_empty() {
            println!("No module-data sets yet: icarus import obdb FILE.json (a vehicle's signal set from github.com/OBDb).");
        }
        for s in &sets {
            let values: usize = s
                .modules
                .iter()
                .flat_map(|m| &m.requests)
                .map(|r| r.values.len())
                .sum();
            println!(
                "{:<20} {} {}{}: {} module(s), {values} value(s){}  [{}, {}]",
                s.set.name,
                s.set.make,
                s.set.models.join(" "),
                if s.set.years.is_empty() {
                    String::new()
                } else {
                    format!(" ({})", years(&s.set.years))
                },
                s.modules.len(),
                if s.refused.is_empty() {
                    String::new()
                } else {
                    format!(", {} refused", s.refused.len())
                },
                s.set.source.from,
                s.source
            );
            if !s.set.source.attribution.is_empty() {
                println!("{:<20} {}", "", s.set.source.attribution);
            }
        }
        return Ok(());
    };
    let s = moddata::load_set(name, user.as_deref(), &all)?;
    let module = args.opt("module").map(hex_id).transpose()?;
    let find = args.opt("find").map(str::to_lowercase);
    println!("{} ({}; {})", s.set.name, s.set.source.from, s.source);
    for m in s
        .modules
        .iter()
        .filter(|m| module.is_none_or(|t| m.module.tx == t))
    {
        let reach = moddata::reachable(m)
            .err()
            .map(|w| format!("  ({w})"))
            .unwrap_or_default();
        println!(
            "\n{}  {}  (answers from {:X}){}",
            m.module.label(),
            m.module.name,
            m.module.rx(),
            reach
        );
        for r in &m.requests {
            let vals: Vec<&ValueDef> = r
                .values
                .iter()
                .filter(|v| {
                    find.as_ref().is_none_or(|f| {
                        [&v.id, &v.name, &v.path]
                            .iter()
                            .any(|t| t.to_lowercase().contains(f))
                    })
                })
                .collect();
            if vals.is_empty() {
                continue;
            }
            let mut about = Vec::new();
            if let Some(e) = r.every {
                about.push(format!("every {e} s"));
            }
            if !r.years.is_empty() {
                about.push(format!("years {}", years(&r.years)));
            }
            if !r.unverified.is_empty() {
                about.push(format!("unverified {}", years(&r.unverified)));
            }
            if let Some(n) = &r.needs_session {
                about.push(format!("needs session {n}: never sent"));
            }
            println!("  {}  {}", r.label(), about.join("; "));
            for v in &vals {
                print_value(v);
            }
            if args.opt("sample").is_some() {
                let ids: Vec<&str> = vals.iter().map(|v| v.id.as_str()).collect();
                for l in sample_lines(r, &ids, units::us_customary())? {
                    println!("{l}");
                }
            }
        }
    }
    if args.opt("refused").is_some() || !s.refused.is_empty() {
        println!("\nRefused ({}):", s.refused.len());
        for (w, why) in &s.refused {
            println!("  {w}: {why}");
        }
    }
    if !s.set.source.attribution.is_empty() {
        println!("\n{}", s.set.source.attribution);
    }
    Ok(())
}

/// What each sample answer of `r` decodes to, for the values `ids`, as `icarus data SET --sample` shows it.
fn sample_lines(r: &md::RequestDef, ids: &[&str], us: bool) -> Result<Vec<String>, String> {
    let mut out = Vec::new();
    for (i, d) in r.samples_data()?.iter().enumerate() {
        if let Outcome::Values(rs) = moddata::decode_all(r, d, None) {
            for x in rs.iter().filter(|x| ids.contains(&x.id.as_str())) {
                out.push(format!(
                    "      sample{}: {:<20} {}",
                    if i == 0 {
                        String::new()
                    } else {
                        format!(" {}", i + 1)
                    },
                    x.id,
                    x.text(us)
                ));
            }
        }
    }
    Ok(out)
}

const NAME_USAGE: &str =
    "usage: icarus data name SET MODULE VALUE_ID --name TEXT [--id NEW_ID] [--keep] [--byte N] [--bit N] [--bits N] \
     [--signed] [--mul X] [--div X] [--add X] [--unit U] [--kind number|text|hex]";

/// A value named by `name_value`: the module file written, the module as read back, and where the value is in it.
struct Named {
    set: String,
    path: std::path::PathBuf,
    module: ModuleDef,
    request: usize,
    id: String,
}

/// Names (and lays out) a value of a set learned from a capture in `root` (the user's data folder) and writes its module
/// file back, checked as Icarus checks every module file when it reads it. Without `--keep` the value itself is renamed
/// and laid out anew; with it, the value stays and a new one (`--id`) is added beside it, for an answer that holds
/// several values. A layout (`--bit`, `--bits`, `--signed`, `--mul`, `--div`, `--add`, `--unit`) makes it a number
/// unless `--kind` says otherwise. The value stays not checked: that is for the vehicle (`icarus obd data --check`).
fn name_value(root: &std::path::Path, args: &Args, today: &str) -> Result<Named, String> {
    let [_, set, module, id] = args.pos.as_slice() else {
        return Err(NAME_USAGE.into());
    };
    let name = args.opt("name").ok_or(NAME_USAGE)?;
    md::check_set_name(set)?;
    let dir = root.join(set);
    if !dir.join("set.toml").is_file() {
        return Err(format!(
            "no set {set:?} in {} (icarus data lists the sets; a built-in one is never changed)",
            root.display()
        ));
    }
    let (info, _, _) = moddata::read_dir(&dir)?;
    if !matches!(
        info.source.from.as_str(),
        "Bluetooth capture" | "bus capture"
    ) {
        return Err(format!(
            "{set} came from {}, not from a capture: icarus data name names a capture's values (edit this set's files by \
             hand, then icarus data check FILE.toml)",
            info.source.from
        ));
    }
    // the module's file: the one whose module is MODULE ("7E0", "750.40", or its request ID) and holds VALUE_ID
    let wanted_tx = hex_id(module).ok();
    let mut found = Vec::new();
    let mut paths: Vec<std::path::PathBuf> = std::fs::read_dir(&dir)
        .map_err(|x| format!("{}: {x}", dir.display()))?
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "toml") && !p.ends_with("set.toml"))
        .collect();
    paths.sort();
    for p in paths {
        let Ok(text) = std::fs::read_to_string(&p) else {
            continue;
        };
        let Ok(m) = md::parse_module(&text, &p.display().to_string()) else {
            continue; // a file Icarus cannot read is listed by icarus data, not named in
        };
        let h = &m.module;
        let this =
            h.label().eq_ignore_ascii_case(module) || (h.ext.is_none() && wanted_tx == Some(h.tx));
        let at = m.requests.iter().enumerate().find_map(|(ri, r)| {
            r.values
                .iter()
                .position(|v| v.id.eq_ignore_ascii_case(id))
                .map(|vi| (ri, vi))
        });
        if let (true, Some(at)) = (this, at) {
            found.push((p, m, at));
        }
    }
    let (path, mut m, (ri, vi)) = match found.len() {
        0 => {
            return Err(format!(
                "{set} has no value {id} in module {module} (icarus data {set} --module {module} lists them)"
            ))
        }
        1 => found.remove(0),
        _ => {
            let files: Vec<String> = found.iter().map(|f| f.0.display().to_string()).collect();
            return Err(format!(
                "{module} {id} is in several files ({}): edit the one you mean by hand",
                files.join(", ")
            ));
        }
    };
    let old = m.requests[ri].values[vi].clone();
    let keep = args.opt("keep").is_some();
    let mut v = old.clone();
    match args.opt("id") {
        Some(new) => v.id = new.to_string(),
        None if keep => {
            return Err("--keep keeps the value and adds a new one: give it an --id".into())
        }
        None => {}
    }
    v.name = name.to_string();
    let int = |k: &str| {
        args.opt(k)
            .map(|s| {
                s.parse::<usize>()
                    .map_err(|_| format!("--{k} needs a whole number, not {s:?}"))
            })
            .transpose()
    };
    let num = |k: &str| {
        args.opt(k)
            .map(|s| {
                s.parse::<f64>()
                    .map_err(|_| format!("--{k} needs a number, not {s:?}"))
            })
            .transpose()
    };
    if let Some(b) = int("byte")? {
        v.byte = b;
    }
    let mut layout = false;
    if let Some(b) = int("bit")? {
        v.bit = u8::try_from(b).map_err(|_| "--bit is 0-7".to_string())?;
        layout = true;
    }
    let bits = int("bits")?;
    if let Some(b) = bits {
        v.bits = u16::try_from(b).map_err(|_| format!("--bits {b} is too many"))?;
        layout = true;
    }
    if args.opt("signed").is_some() {
        v.signed = true;
        layout = true;
    }
    for (k, field) in [
        ("mul", &mut v.mul),
        ("div", &mut v.div),
        ("add", &mut v.add),
    ] {
        if let Some(x) = num(k)? {
            *field = x;
            layout = true;
        }
    }
    if let Some(u) = args.opt("unit") {
        v.unit = u.to_string();
        layout = true;
    }
    let kind = match args.opt("kind") {
        Some("number") => Some(md::ValueKind::Number),
        Some("text") => Some(md::ValueKind::Text),
        Some("hex") => Some(md::ValueKind::Hex),
        Some(_) => return Err(
            "--kind is number, text or hex (a state value needs its states: edit the file by hand)"
                .into(),
        ),
        None => layout.then_some(md::ValueKind::Number),
    };
    if let Some(k) = kind {
        if k == md::ValueKind::Number && old.kind != k && bits.is_none() {
            return Err(format!(
                "{} covers the whole answer ({} bits): give the number's width with --bits (and --byte where it starts)",
                old.id, old.bits
            ));
        }
        v.kind = k;
    }
    v.checked = false;
    v.note = if keep {
        format!(
            "named by hand on {today} from the bytes of {}; not checked",
            old.id
        )
    } else if old.note.contains("not named, not checked") {
        old.note.replace(
            "not named, not checked",
            &format!("named by hand on {today} (was {}), not checked", old.id),
        )
    } else if old.note.contains("named by hand") {
        old.note.clone()
    } else {
        format!("named by hand on {today}; not checked")
    };
    let new_id = v.id.clone();
    if keep {
        m.requests[ri].values.push(v);
    } else {
        m.requests[ri].values[vi] = v;
    }
    let what = path.display().to_string();
    md::validate(&mut m).map_err(|x| format!("{what}: {x}; nothing written"))?;
    let text = md::module_to_toml(&m);
    // as Icarus will read it back
    let back = md::parse_module(&text, &what).map_err(|x| format!("{x}; nothing written"))?;
    let tmp = path.with_extension("toml.new");
    std::fs::write(&tmp, &text).map_err(|x| format!("{}: {x}", tmp.display()))?;
    std::fs::rename(&tmp, &path).map_err(|x| format!("{what}: {x}"))?;
    named_attribution(&dir)?;
    Ok(Named {
        set: set.clone(),
        path,
        module: back,
        request: ri,
        id: new_id,
    })
}

/// Brings a capture set's attribution up to date with its naming (ICA-61): how many of its values are named by hand. A
/// hand-written attribution is left alone.
fn named_attribution(dir: &std::path::Path) -> Result<(), String> {
    let (mut info, modules, _) = moddata::read_dir(dir)?;
    if !capture::is_capture_attribution(&info.source.attribution) {
        return Ok(());
    }
    let (named, total) = capture::naming_counts(&modules);
    let now = capture::attribution_named(named, total);
    if now == info.source.attribution {
        return Ok(());
    }
    info.source.attribution = now;
    let path = dir.join("set.toml");
    let tmp = dir.join("set.toml.new");
    std::fs::write(&tmp, md::set_to_toml(&info)).map_err(|x| format!("{}: {x}", tmp.display()))?;
    std::fs::rename(&tmp, &path).map_err(|x| format!("{}: {x}", path.display()))
}

/// `icarus data name SET MODULE VALUE_ID --name TEXT ...` (see `name_value`).
fn data_name(args: &Args) -> Result<(), String> {
    let root = moddata::user_dir().ok_or("no config folder")?;
    let n = name_value(&root, args, &today())?;
    let r = &n.module.requests[n.request];
    let label = n.module.module.label();
    println!("{label} {}: written to {}", r.label(), n.path.display());
    if let Some(v) = r.values.iter().find(|v| v.id == n.id) {
        print_value(v);
    }
    for l in sample_lines(r, &[n.id.as_str()], units::us_customary())? {
        println!("{l}");
    }
    println!(
        "Not checked yet: icarus data {} --sample shows every value; once it matches a gauge or the app on the vehicle, \
         icarus obd data --check {:X} {} --adapter A",
        n.set, n.module.module.tx, n.id
    );
    Ok(())
}

fn data_check(args: &Args, all: &[vehicles::Profile]) -> Result<(), String> {
    let [_, path] = args.pos.as_slice() else {
        return Err("usage: icarus data check FILE.toml [--make NAME]".into());
    };
    // the make decides whether a module of unknown kind may be asked (where its profile names no airbag module, not)
    let make = match args.opt("make") {
        Some(m) => vehicles::find(all, m)?.id.clone(),
        None => String::new(),
    };
    let text = std::fs::read_to_string(path).map_err(|x| format!("{path}: {x}"))?;
    let mut m = md::parse_module(&text, path)?;
    println!(
        "{path}: {} at {} (answers from {:X}), the file is well formed",
        m.module.name,
        m.module.label(),
        m.module.rx()
    );
    if let Err(why) = moddata::reachable(&m) {
        println!("  {why}");
    }
    let gates = moddata::Gates::new(all, std::slice::from_ref(&m), &make);
    let refused = moddata::gate(std::slice::from_mut(&mut m), &gates);
    for r in &m.requests {
        let bytes: Vec<String> = r.bytes().iter().map(|b| format!("{b:02X}")).collect();
        println!(
            "  would send {} to {:03X}{}: {} value(s), {} data byte(s) expected",
            bytes.join(" "),
            m.module.tx,
            if r.needs_session.is_some() {
                " (no: it needs a session)"
            } else {
                ""
            },
            r.values.len(),
            r.min_len.unwrap_or(0)
        );
    }
    for (w, why) in &refused {
        println!("  refused {w}: {why}");
    }
    Ok(())
}

use icarus_obd::vehicledata::choose;

/// The chosen set as used on this vehicle, its note (another make, or an unknown one) said on stderr.
fn on_vehicle(set: &Loaded, all: &[vehicles::Profile], make: Option<&str>) -> Loaded {
    let (set, note) = icarus_obd::vehicledata::on_vehicle(set, all, make);
    if let Some(n) = note {
        eprintln!("note: {n}");
    }
    set
}

/// A CSV column: (module request ID, value id) and its heading.
type Column = ((u32, String), String);

/// The last reading of each (module, value id).
type Last = BTreeMap<(u32, String), moddata::Reading>;

/// The table's columns, fixed before the first pass so every row lines up with the header: every value of the chosen
/// modules that may be asked for (not those of a module not reachable yet, a request that needs a session or one for
/// other model years), so a value that first answers on a later pass already has its column.
fn columns(
    chosen: &[&ModuleDef],
    scope: &Scope,
    only: &dyn Fn(&ValueDef) -> bool,
    us: bool,
) -> Vec<Column> {
    let mut out: Vec<Column> = Vec::new();
    for m in chosen.iter().filter(|m| moddata::reachable(m).is_ok()) {
        for r in &m.requests {
            let for_year = r.years.is_empty()
                || match scope.year {
                    Some(y) => md::year_in(&r.years, y),
                    None => scope.all_years,
                };
            if r.needs_session.is_some() || !for_year {
                continue;
            }
            for v in r.values.iter().filter(|v| only(v)) {
                let k = (m.module.tx, v.id.clone());
                if out.iter().any(|c| c.0 == k) {
                    continue;
                }
                let u = units::shown(&v.unit, 0.0, us).1;
                let unit = if u.is_empty() {
                    String::new()
                } else {
                    format!(" [{u}]")
                };
                out.push((k, format!("{} {}{unit}", m.module.label(), v.name)));
            }
        }
    }
    out
}

/// What one request's outcome does to the last readings: its values replace theirs; anything but "not due yet" clears
/// them, so a module that stopped answering, refused or failed shows an empty cell, never its last reading as if live.
fn note_outcome(last: &mut Last, m: &ModuleDef, r: &md::RequestDef, o: &Outcome) {
    match o {
        Outcome::Values(vs) => {
            for v in vs {
                last.insert((m.module.tx, v.id.clone()), v.clone());
            }
        }
        Outcome::NotDue => {}
        _ => {
            for v in &r.values {
                last.remove(&(m.module.tx, v.id.clone()));
            }
        }
    }
}

/// The CSV header line (without the line end).
fn csv_header(columns: &[Column]) -> String {
    std::iter::once("time_s".to_string())
        .chain(
            columns
                .iter()
                .map(|c| format!("\"{}\"", c.1.replace('"', "'"))),
        )
        .collect::<Vec<_>>()
        .join(",")
}

/// One row's cells, one per column: the number in the units shown, a state or text in quotes, empty for no reading.
fn row(columns: &[Column], last: &Last, us: bool) -> Vec<String> {
    columns
        .iter()
        .map(|(k, _)| match last.get(k) {
            Some(r) => match (&r.shown, r.number(us)) {
                (_, Some(x)) => format!("{x:.2}"),
                (Shown::NoReading, _) => String::new(),
                _ => format!("\"{}\"", r.text(us).replace('"', "'")),
            },
            None => String::new(),
        })
        .collect()
}

/// One `icarus obd data` reading run: what it reads and how it shows it.
struct Watch<'a> {
    set: &'a Loaded,
    chosen: Vec<&'a ModuleDef>,
    scope: Scope,
    only: &'a dyn Fn(&ValueDef) -> bool,
    us: bool,
    /// --every: passes this far apart until --count, Ctrl+C or a recording.
    every: Option<Duration>,
    count: Option<u64>,
    show_refused: bool,
    today: String,
    /// The set was chosen for this vehicle just now: the state is saved even if nothing else changes.
    new_set: bool,
}

/// How a run ended.
#[derive(Debug, PartialEq)]
struct Ran {
    passes: u64,
    /// Why it stopped before it was done (Ctrl+C, a recording that owns the adapter), if it did.
    stopped: Option<String>,
}

impl Watch<'_> {
    /// The passes: the first printed in full, then (with --every or --csv) one row a pass. `save` keeps the vehicle's
    /// state: after the first pass, and at the end however it ends, Ctrl+C included. `wait` sleeps between passes.
    fn run(
        &self,
        link: &mut dyn Link,
        pacer: &mut Pacer,
        state: &mut State,
        save: &mut dyn FnMut(&State) -> Result<(), String>,
        mut csv: Option<&mut dyn Write>,
        wait: &dyn Fn(Duration),
    ) -> Result<Ran, String> {
        let us = self.us;
        let table = self.every.is_some() || csv.is_some();
        let columns = if table {
            columns(&self.chosen, &self.scope, self.only, us)
        } else {
            vec![]
        };
        let mut last = Last::new();
        let mut later = Vec::new();
        let mut changed = self.new_set;
        let start = Instant::now();
        let mut pass = 0u64;
        let mut stopped = None;
        'passes: loop {
            let t0 = Instant::now();
            let first = pass == 0;
            for m in &self.chosen {
                if let Err(why) = moddata::reachable(m) {
                    if first {
                        let why = why.trim_start_matches("not reachable yet: ");
                        later.push(format!("{}  {}: {why}", m.module.label(), m.module.name));
                    }
                    continue;
                }
                if let Some(why) = pacer.stopped() {
                    stopped = Some(why);
                    break 'passes;
                }
                let got = moddata::read_module(
                    link,
                    pacer,
                    &self.set.gates,
                    m,
                    &self.scope,
                    Some(self.only),
                );
                if first {
                    println!("\n{}  {}", m.module.label(), m.module.name);
                    if m.requests.is_empty() {
                        println!(
                            "   no data requests{}",
                            self.set
                                .gates
                                .identity_only(&m.module)
                                .map(|why| format!(" ({why})"))
                                .unwrap_or_default()
                        );
                    }
                }
                for (r, o) in &got {
                    changed |= state.record(moddata::key(m, r), o, &self.today);
                    note_outcome(&mut last, m, r, o);
                    match o {
                        Outcome::Values(vs) if first => {
                            for v in vs {
                                let checked = if state.checked(m.module.tx, &v.id).is_some() {
                                    "  (checked)"
                                } else {
                                    ""
                                };
                                println!("   {:<44} {}{checked}", v.name, v.text(us));
                            }
                        }
                        Outcome::Values(_) | Outcome::NotDue => {}
                        Outcome::Skipped(_) if first && !self.show_refused => {}
                        other if first => println!("   {}: {}", r.label(), other.describe()),
                        _ => {}
                    }
                }
            }
            if first {
                if !later.is_empty() {
                    println!("\nNot reachable yet:\n  {}", later.join("\n  "));
                }
                if self.show_refused && !self.set.refused.is_empty() {
                    println!("\nRefused by Icarus's gates when the set was loaded:");
                    for (w, why) in &self.set.refused {
                        println!("  {w}: {why}");
                    }
                }
                if !self.set.set.source.attribution.is_empty() {
                    println!("\n{}", self.set.set.source.attribution);
                }
                if changed {
                    save(state)?;
                    changed = false;
                }
                if table {
                    let names: Vec<&str> = columns.iter().map(|c| c.1.as_str()).collect();
                    println!("\ntime_s, {}", names.join(", "));
                    if let Some(f) = csv.as_mut() {
                        writeln!(f, "{}", csv_header(&columns)).map_err(|x| x.to_string())?;
                    }
                }
            }
            if table {
                let cells = row(&columns, &last, us);
                let t = format!("{:.2}", start.elapsed().as_secs_f64());
                println!("{t}, {}", cells.join(", "));
                if let Some(f) = csv.as_mut() {
                    writeln!(f, "{t},{}", cells.join(",")).map_err(|x| x.to_string())?;
                }
            }
            pass += 1;
            let Some(every) = self.every else { break };
            if self.count.is_some_and(|c| pass >= c) {
                break;
            }
            if let Some(why) = pacer.stopped() {
                stopped = Some(why);
                break;
            }
            if let Some(rest) = every.checked_sub(t0.elapsed()) {
                wait(rest);
            }
        }
        if pass == 0 && !self.set.set.source.attribution.is_empty() {
            println!("\n{}", self.set.set.source.attribution);
        }
        // what the vehicle did is kept however the run ended
        if changed {
            save(state)?;
        }
        match stopped {
            // stopped before anything was asked: that is an error (a recording owns the adapter)
            Some(why) if pacer.sent == 0 => Err(why),
            stopped => Ok(Ran {
                passes: pass,
                stopped,
            }),
        }
    }
}

/// `icarus obd data [MODULE...] --adapter A ...` and `icarus obd data --check MODULE VALUE_ID [--note TEXT]`.
/// What `obd data` and `obd suggest` start from: the link, the vehicle's model year, where its state is kept and the
/// state, and the set chosen for it (as used on this vehicle: its gates rebuilt for the make).
struct Opened {
    elm: Box<dyn Link>,
    year: Option<u16>,
    state_path: Option<std::path::PathBuf>,
    state: State,
    set: Loaded,
    new_set: bool,
}

fn open_set(args: &Args) -> Result<Opened, String> {
    let all = profiles();
    let mut elm = connect(args)?;
    let vin_s = j1979::vin(&mut elm).map_err(e)?;
    super::note_dropped(&mut elm);
    let vin = vin_s.as_deref();
    let year = match args.opt("year") {
        Some(y) => Some(y.parse::<u16>().map_err(|_| "--year needs a model year")?),
        None => vin.and_then(|v| vin::decode(v).ok()).and_then(|d| d.year),
    };
    let make = match args.opt("make") {
        Some(m) => Some(vehicles::find(&all, m)?.id.clone()),
        None => vin
            .and_then(|v| vehicles::for_vin(&all, v))
            .map(|p| p.id.clone()),
    };
    let model = vin
        .and_then(garage::get)
        .map(|d| d.model)
        .filter(|m| !m.is_empty());
    let state_path = moddata::state_path(vin)?;
    let mut state = match &state_path {
        Some(p) => State::load(p)?,
        None => {
            eprintln!(
                "note: the VIN could not be read, so what each request does is kept for this run only (never shared with \
                 another vehicle's)"
            );
            State::default()
        }
    };
    let user = moddata::user_dir();
    let (sets, problems) = moddata::load_all(user.as_deref(), &all);
    for p in problems {
        eprintln!("note: a module-data set was left out: {p}");
    }
    let set = choose(
        &sets,
        args.opt("set"),
        &state,
        make.as_deref(),
        year,
        model.as_deref(),
        vin,
    )?;
    // the airbag caution follows the vehicle's make, not only the set's
    let set = on_vehicle(set, &all, make.as_deref());
    let new_set = state.set.as_deref() != Some(set.set.name.as_str());
    state.set = Some(set.set.name.clone());
    Ok(Opened {
        elm,
        year,
        state_path,
        state,
        set,
        new_set,
    })
}

/// The modules named on the command line ("701", or "750.2A" behind an extended address), or every one with requests.
fn chosen_modules<'s>(set: &'s Loaded, pos: &[String]) -> Result<Vec<&'s ModuleDef>, String> {
    if pos.is_empty() {
        return Ok(set
            .modules
            .iter()
            .filter(|m| !m.requests.is_empty())
            .collect());
    }
    pos.iter()
        .map(|p| {
            if let Some(m) = set
                .modules
                .iter()
                .find(|m| m.module.label().eq_ignore_ascii_case(p))
            {
                return Ok(m);
            }
            let tx = hex_id(p)?;
            set.module(tx).ok_or(format!(
                "{} has no module {tx:03X} (icarus data {} lists them)",
                set.set.name, set.set.name
            ))
        })
        .collect()
}

/// `icarus obd suggest [MODULE...] [--seconds N] [--set SET]` (ICA-7, `naming`): the standard values and the set's requests
/// read in turn, then what the bytes no definition names follow. Guesses to check; nothing is written.
pub fn obd_suggest(args: &Args) -> Result<(), String> {
    let mut o = open_set(args)?;
    let secs = args
        .opt("seconds")
        .map(|v| v.parse::<f64>().map_err(|_| "--seconds needs a number"))
        .transpose()?
        .unwrap_or(60.0);
    let modules: Vec<ModuleDef> = chosen_modules(&o.set, &args.pos)?
        .into_iter()
        .cloned()
        .collect();
    let scope = Scope {
        year: o.year,
        all_years: args.opt("all-years").is_some(),
        refused_before: o.state.refused_keys(),
        retry: args.opt("all").is_some(),
    };
    actuator::catch_interrupt();
    println!(
        "Reading the standard values and {} module(s) of set {} in turn for {secs} s (Ctrl+C stops early). Keep the engine \
         running; values that rise and fall (rev it, or drive) are what can be matched. To name what changes with \
         something done, type what you are doing as you do it (\"driver door open\") and press Enter: each line is a mark \
         (Enter alone: \"mark 1\", \"mark 2\"...); do each thing twice or more.",
        modules.len(),
        o.set.set.name
    );
    // each line typed is a mark, timed when Enter was pressed
    let (tx, rx) = std::sync::mpsc::channel::<(Instant, String)>();
    std::thread::spawn(move || {
        let mut n = 0;
        for line in std::io::stdin().lines() {
            let Ok(line) = line else { break };
            n += 1;
            let said: String = line
                .trim()
                .chars()
                .filter(|c| !c.is_control())
                .take(60)
                .collect();
            let name = if said.is_empty() {
                format!("mark {n}")
            } else {
                said
            };
            if tx.send((Instant::now(), name)).is_err() {
                break;
            }
        }
    });
    let got = naming::sample(
        &mut o.elm,
        &mut Pacer::real(),
        &o.set.gates,
        &modules,
        &scope,
        naming::PIDS,
        &Instant::now,
        &mut |t| t >= secs || actuator::interrupted(),
        &mut || rx.try_iter().collect(),
    );
    let found = naming::suggest(&got.standard, &got.unknown);
    let reads: usize = got.unknown.iter().map(|u| u.reads.len()).sum();
    println!(
        "{} pass(es): {} standard value(s), {} request(s) with unnamed bytes ({reads} answers).",
        got.passes,
        got.standard.iter().filter(|s| !s.reads.is_empty()).count(),
        got.unknown.len()
    );
    if found.is_empty() {
        println!(
            "Nothing follows a standard value closely enough (r {} or more over {} reads). A longer run, with the values \
             moving, may find more.",
            naming::MIN_R,
            naming::MIN_READS
        );
    } else {
        println!("Suggestions (guesses to check, not written anywhere):");
        for f in &found {
            println!("  {}", f.text());
        }
    }
    if !got.marks.is_empty() {
        let by_marks = naming::suggest_marks(&got.unknown, &got.marks, &found);
        println!("{} mark(s).", got.marks.len());
        if by_marks.is_empty() {
            println!(
                "Nothing changed near every mark of a name and at no other time (a change counts from {} s before a mark \
                 to {} s after).",
                naming::MARK_BEFORE,
                naming::MARK_AFTER
            );
        } else {
            println!("Changed with the marks (guesses to check, not written anywhere):");
            for f in &by_marks {
                println!("  {}", f.text());
            }
        }
    }
    for (what, why) in &got.left_out {
        println!("  not read: {what}: {why}");
    }
    Ok(())
}

pub fn obd_data(args: &Args) -> Result<(), String> {
    let Opened {
        mut elm,
        year,
        state_path,
        mut state,
        set,
        new_set,
    } = open_set(args)?;
    let set = &set;
    let mut save = |st: &State| match &state_path {
        Some(p) => st.save(p),
        None => Ok(()),
    };
    if args.opt("check").is_some() {
        let [module, id] = args.pos.as_slice() else {
            return Err(
                "usage: icarus obd data --check MODULE VALUE_ID [--note TEXT] --adapter A".into(),
            );
        };
        let Some(path) = &state_path else {
            return Err(
                "the VIN could not be read, so there is no vehicle folder to keep the check in"
                    .into(),
            );
        };
        let tx = hex_id(module)?;
        let m = set
            .module(tx)
            .ok_or(format!("{} has no module {tx:03X}", set.set.name))?;
        let v = m
            .requests
            .iter()
            .flat_map(|r| &r.values)
            .find(|v| v.id.eq_ignore_ascii_case(id))
            .ok_or(format!(
                "{tx:03X} has no value {id} (icarus data {} --module {tx:X})",
                set.set.name
            ))?;
        state.check(tx, &v.id, args.opt("note"), &today());
        state.save(path)?;
        println!(
            "{tx:03X} {} ({}) marked as checked on this vehicle ({})",
            v.id,
            v.name,
            path.display()
        );
        return Ok(());
    }
    let chosen = chosen_modules(set, &args.pos)?;
    let us = units::us_customary();
    println!(
        "Module data: set {} ({}){}",
        set.set.name,
        set.set.source.from,
        year.map(|y| format!(", model year {y}"))
            .unwrap_or_default()
    );
    let find = args.opt("find").map(str::to_lowercase);
    let only = |v: &ValueDef| match &find {
        Some(f) => [&v.id, &v.name, &v.path]
            .iter()
            .any(|t| t.to_lowercase().contains(f)),
        None => !v.hidden,
    };
    let every = args
        .opt("every")
        .map(|v| v.parse::<u64>().map_err(|_| "--every needs milliseconds"))
        .transpose()?
        .map(Duration::from_millis);
    let count = args
        .opt("count")
        .map(|v| v.parse::<u64>().map_err(|_| "--count needs a number"))
        .transpose()?;
    if every.is_some() {
        actuator::catch_interrupt();
    }
    let mut csv = args
        .opt("csv")
        .map(|p| std::fs::File::create(p).map_err(|x| format!("{p}: {x}")))
        .transpose()?;
    let watch = Watch {
        set,
        chosen,
        scope: Scope {
            year,
            all_years: args.opt("all-years").is_some(),
            refused_before: state.refused_keys(),
            retry: args.opt("all").is_some(),
        },
        only: &only,
        us,
        every,
        count,
        show_refused: args.opt("refused").is_some(),
        today: today(),
        new_set,
    };
    let ran = watch.run(
        &mut elm,
        &mut Pacer::real(),
        &mut state,
        &mut save,
        csv.as_mut().map(|f| f as &mut dyn Write),
        &std::thread::sleep,
    )?;
    if let Some(why) = ran.stopped {
        eprintln!(
            "{why}: stopped after {} pass(es); what the vehicle did is saved",
            ran.passes
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use icarus_obd::elm::Elm;
    use icarus_obd::sim::{SimCar, VIN};
    use std::cell::Cell;

    fn sim() -> Loaded {
        moddata::load_set("sim", None, &vehicles::builtin()).unwrap()
    }

    /// The sim set and a made-up user set for a Toyota "Truck".
    fn sets() -> Vec<Loaded> {
        let mut truck = sim();
        truck.set.name = "toy-truck".into();
        truck.set.models = vec!["Truck".into()];
        vec![sim(), truck]
    }

    fn sim_scope() -> Scope {
        Scope {
            year: Some(2025),
            ..Default::default()
        }
    }

    #[test]
    fn the_simulated_set_is_never_chosen_for_another_vehicle() {
        let sets = sets();
        let none = State::default();
        let real = Some("JTEXAMPLE00000000");
        for vin in [None, real] {
            let e = choose(
                &sets,
                Some("sim"),
                &none,
                Some("toyota"),
                Some(2025),
                None,
                vin,
            )
            .unwrap_err();
            assert!(e.contains("never sent to another vehicle"), "{vin:?}: {e}");
            // nor because a state remembers it
            let remembered = State {
                set: Some("sim".into()),
                ..Default::default()
            };
            assert!(choose(
                &sets,
                None,
                &remembered,
                Some("toyota"),
                Some(2025),
                None,
                vin
            )
            .is_err());
        }
        let ok = |wanted, vin| {
            choose(
                &sets,
                wanted,
                &none,
                Some("toyota"),
                Some(2017),
                Some("Truck"),
                vin,
            )
            .unwrap()
            .set
            .name
            .clone()
        };
        assert_eq!(ok(Some("sim"), Some(VIN)), "sim");
        assert_eq!(ok(None, Some(VIN)), "sim");
        assert_eq!(ok(None, real), "toy-truck");
        assert_eq!(
            ok(Some("toy-truck"), None),
            "toy-truck",
            "--set works with the VIN unread"
        );
        // the model unknown: never guessed, and the message says how to tell it
        let e = choose(&sets, None, &none, Some("toyota"), Some(2017), None, real).unwrap_err();
        assert!(
            e.contains("model (name the vehicle") && e.contains("--set"),
            "{e}"
        );
        let e = choose(&sets, None, &none, None, None, None, None).unwrap_err();
        assert!(e.contains("make") && e.contains("model year"), "{e}");
    }

    #[test]
    fn the_columns_are_fixed_before_the_first_answer() {
        let set = sim();
        let chosen: Vec<&ModuleDef> = set.modules.iter().collect();
        let all = |_: &ValueDef| true;
        let cols = columns(&chosen, &sim_scope(), &all, true);
        let has = |id: &str| cols.iter().any(|c| c.0 .1 == id);
        assert!(has("SIM_ECT") && has("SIM_ATF") && has("SIM_ISC"));
        let ect = cols.iter().find(|c| c.0 .1 == "SIM_ECT").unwrap();
        assert_eq!(ect.1, "700 coolant temperature [°F]");
        // nothing for a request that is never sent: other years (1F04), a session (1F05)
        let never: Vec<&str> = set.modules[0].requests[3..5]
            .iter()
            .flat_map(|r| &r.values)
            .map(|v| v.id.as_str())
            .collect();
        assert!(
            !never.is_empty() && never.iter().all(|id| !has(id)),
            "{never:?}"
        );
        // --all-years with the year unknown takes 1F04's values too
        let any_year = Scope {
            all_years: true,
            ..Default::default()
        };
        assert!(columns(&chosen, &any_year, &all, true).len() > cols.len());
        assert!(csv_header(&cols).starts_with("time_s,\"700 coolant temperature [°F]\""));
    }

    #[test]
    fn a_reading_that_is_no_longer_live_is_cleared() {
        let set = sim();
        let m = &set.modules[0];
        let r = &m.requests[0];
        let all = |_: &ValueDef| true;
        let cols = columns(&[m], &sim_scope(), &all, false);
        let mut last = Last::new();
        let Outcome::Values(vs) =
            moddata::decode_all(r, &r.sample_data().unwrap().unwrap(), Some(2025))
        else {
            panic!()
        };
        note_outcome(&mut last, m, r, &Outcome::Values(vs));
        let ect = cols.iter().position(|c| c.0 .1 == "SIM_ECT").unwrap();
        assert_eq!(row(&cols, &last, false)[ect], "90.00");
        // not due: the reading stands
        note_outcome(&mut last, m, r, &Outcome::NotDue);
        assert_eq!(row(&cols, &last, false)[ect], "90.00");
        // anything else: an empty cell, never the old reading
        for o in [
            Outcome::Silent,
            Outcome::Refused(0x22),
            Outcome::Short(2),
            Outcome::Failed("garbled".into()),
            Outcome::Skipped("stopped answering".into()),
        ] {
            note_outcome(
                &mut last,
                m,
                r,
                &moddata::decode_all(r, &r.sample_data().unwrap().unwrap(), Some(2025)),
            );
            note_outcome(&mut last, m, r, &o);
            assert_eq!(row(&cols, &last, false)[ect], "", "{o:?}");
        }
        assert_eq!(row(&cols, &last, false).len(), cols.len());
    }

    /// A link that answers each request to one module with the next window of a script (an empty window: silence), and
    /// counts the requests.
    struct Script(Vec<Vec<u8>>, usize);
    impl Link for Script {
        fn request(
            &mut self,
            _: &[u8],
        ) -> Result<Vec<icarus_obd::elm::Answer>, icarus_obd::elm::ObdError> {
            unreachable!("module data never broadcasts")
        }
        fn request_to(
            &mut self,
            _: u32,
            rx: u32,
            _: &[u8],
        ) -> Result<Vec<icarus_obd::elm::Answer>, icarus_obd::elm::ObdError> {
            self.1 += 1;
            Ok(self
                .0
                .get(self.1 - 1)
                .filter(|d| !d.is_empty())
                .map(|d| {
                    vec![icarus_obd::elm::Answer {
                        from: format!("{rx:03X}"),
                        data: d.clone(),
                    }]
                })
                .unwrap_or_default())
        }
        fn protocol(&self) -> icarus_obd::elm::Protocol {
            icarus_obd::elm::Protocol::Can11At500
        }
        fn adapter(&self) -> String {
            "script".into()
        }
        fn voltage(&mut self) -> Option<f64> {
            None
        }
    }

    fn go() -> Option<String> {
        None
    }

    /// An OBDb-style import (with `--make toyota`): one module at 7A0 whose 22 4A01 has a layout for each range of model
    /// years, each with its own value id.
    fn layouts() -> Loaded {
        let cmd = |y: &str| {
            format!(
                r#"{{"hdr": "7A0", "cmd": {{"22": "4A01"}}, "filter": {y}, "signals": [{{"id": "A", "name": "A", "fmt": {{"len": 8}}}}]}}"#
            )
        };
        let json = format!(
            r#"{{"commands": [{}, {}]}}"#,
            cmd(r#"{"from": 2010, "to": 2012}"#),
            cmd(r#"{"from": 2013, "to": 2015}"#)
        );
        let imp = obdb::parse(
            &json,
            &obdb::Options {
                path: "Toy-Truck/signalsets/v3/default.json".into(),
                make: Some("toyota".into()),
                ..Default::default()
            },
        )
        .unwrap();
        let profiles = vehicles::builtin();
        let mut modules = imp.modules;
        let set = imp.set.unwrap();
        let gates = moddata::Gates::new(&profiles, &modules, &set.make);
        let refused = moddata::gate(&mut modules, &gates);
        assert!(refused.is_empty(), "{refused:?}");
        Loaded {
            set,
            modules,
            refused,
            source: "test".into(),
            gates,
        }
    }

    #[test]
    fn one_request_with_several_year_layouts_clears_the_layout_that_was_read() {
        let set = layouts();
        let ids: Vec<&str> = set.modules[0]
            .requests
            .iter()
            .flat_map(|r| &r.values)
            .map(|v| v.id.as_str())
            .collect();
        assert_eq!(ids, vec!["A", "A_22_4A01"], "two layouts of one 22 4A01");
        let all = |_: &ValueDef| true;
        let mut w = watch(&set, &all, Some(Duration::ZERO), Some(3));
        w.scope.year = Some(2014);
        // a 2014 vehicle: the second layout answers, then the module goes quiet, then answers again
        let mut link = Script(
            vec![vec![0x62, 0x4A, 0x01, 7], vec![], vec![0x62, 0x4A, 0x01, 9]],
            0,
        );
        let mut csv: Vec<u8> = Vec::new();
        let mut state = State::default();
        w.run(
            &mut link,
            &mut Pacer::new(Instant::now, no_sleep, go),
            &mut state,
            &mut |_| Ok(()),
            Some(&mut csv),
            &|_| {},
        )
        .unwrap();
        assert_eq!(link.1, 3, "only the 2013-2015 layout is sent, once a pass");
        let text = String::from_utf8(csv).unwrap();
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(
            lines[0], "time_s,\"7A0 A\"",
            "one column: the layout for 2014"
        );
        let cell = |l: &str| l.split_once(',').unwrap().1.to_string();
        assert_eq!(cell(lines[1]), "7.00");
        assert_eq!(
            cell(lines[2]),
            "",
            "silent: an empty cell, never the last reading as if live"
        );
        assert_eq!(cell(lines[3]), "9.00");
        // the vehicle's state knows the request by its key, whichever layout answered
        assert!(state
            .requests
            .iter()
            .any(|r| r.ident == 0x4A01 && r.answered.is_some()));
    }

    #[test]
    fn a_set_for_another_make_does_not_ask_a_module_that_could_be_the_airbag_module() {
        // a Toyota set (Toyota's profile names its airbag module) with a module known only by its address
        let mut set = layouts();
        set.set.name = "toy-truck".into();
        assert!(set.modules[0].module.kind.is_empty());
        let sets = vec![set];
        let chosen = choose(
            &sets,
            Some("toy-truck"),
            &State::default(),
            Some("gm"),
            Some(2014),
            None,
            Some("1GEXAMPLE00000000"),
        )
        .unwrap();
        for make in [Some("gm"), None] {
            let on = on_vehicle(chosen, &vehicles::builtin(), make);
            assert!(
                on.modules[0].requests.is_empty(),
                "{make:?}: no data request kept"
            );
            assert!(
                on.refused
                    .iter()
                    .any(|(_, why)| why.contains("could be the airbag module")),
                "{make:?}: {:?}",
                on.refused
            );
            // and the run sends nothing to it, even handed the module
            let all = |_: &ValueDef| true;
            let mut w = watch(&on, &all, None, None);
            w.chosen = on.modules.iter().collect();
            w.scope.year = Some(2014);
            let mut link = Script(vec![vec![0x62, 0x4A, 0x01, 7]], 0);
            w.run(
                &mut link,
                &mut Pacer::new(Instant::now, no_sleep, go),
                &mut State::default(),
                &mut |_| Ok(()),
                None,
                &|_| {},
            )
            .unwrap();
            assert_eq!(link.1, 0, "{make:?}: nothing was sent");
        }
        // on a Toyota, as loaded
        let on = on_vehicle(chosen, &vehicles::builtin(), Some("toyota"));
        assert_eq!(on.modules[0].requests.len(), 2);
    }

    thread_local! {
        static STOP_AFTER: Cell<u32> = const { Cell::new(u32::MAX) };
        static ASKED: Cell<u32> = const { Cell::new(0) };
    }
    fn stop_after() -> Option<String> {
        let n = ASKED.with(|a| {
            a.set(a.get() + 1);
            a.get()
        });
        (n > STOP_AFTER.with(|s| s.get())).then(|| "stopped (Ctrl+C)".to_string())
    }
    fn no_sleep(_: Duration) {}

    fn watch<'a>(
        set: &'a Loaded,
        only: &'a dyn Fn(&ValueDef) -> bool,
        every: Option<Duration>,
        count: Option<u64>,
    ) -> Watch<'a> {
        Watch {
            set,
            chosen: set
                .modules
                .iter()
                .filter(|m| !m.requests.is_empty())
                .collect(),
            scope: sim_scope(),
            only,
            us: false,
            every,
            count,
            show_refused: false,
            today: "2026-11-14".into(),
            new_set: true,
        }
    }

    /// Fields of a CSV line, quotes respected.
    fn fields(l: &str) -> usize {
        let mut quoted = false;
        1 + l
            .chars()
            .filter(|c| {
                if *c == '"' {
                    quoted = !quoted;
                }
                *c == ',' && !quoted
            })
            .count()
    }

    #[test]
    fn every_csv_row_lines_up_with_the_header() {
        STOP_AFTER.with(|s| s.set(u32::MAX));
        let set = sim();
        let all = |_: &ValueDef| true;
        let w = watch(&set, &all, Some(Duration::ZERO), Some(3));
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        let mut csv: Vec<u8> = Vec::new();
        let mut saved = 0;
        let ran = w
            .run(
                &mut elm,
                &mut Pacer::new(Instant::now, no_sleep, stop_after),
                &mut State::default(),
                &mut |_| {
                    saved += 1;
                    Ok(())
                },
                Some(&mut csv),
                &|_| {},
            )
            .unwrap();
        assert_eq!(
            ran,
            Ran {
                passes: 3,
                stopped: None
            }
        );
        let text = String::from_utf8(csv).unwrap();
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines.len(), 4, "{text}");
        let n = fields(lines[0]);
        assert!(n > 5);
        for l in &lines[1..] {
            assert_eq!(fields(l), n, "{l}");
        }
        assert!(saved >= 1);
    }

    #[test]
    fn ctrl_c_ends_the_watch_and_keeps_what_was_learned() {
        let set = sim();
        let all = |_: &ValueDef| true;
        let w = watch(&set, &all, Some(Duration::ZERO), None);
        // Ctrl+C in the middle of the first pass, after 1F03 was refused as not there
        STOP_AFTER.with(|s| s.set(4));
        ASKED.with(|a| a.set(0));
        let mut elm = Elm::connect(Box::new(SimCar::new())).unwrap();
        let mut state = State::default();
        let mut saved: Vec<State> = Vec::new();
        let ran = w
            .run(
                &mut elm,
                &mut Pacer::new(Instant::now, no_sleep, stop_after),
                &mut state,
                &mut |s| {
                    saved.push(s.clone());
                    Ok(())
                },
                None,
                &|_| {},
            )
            .unwrap();
        assert_eq!(ran.stopped.as_deref(), Some("stopped (Ctrl+C)"));
        assert_eq!(ran.passes, 0);
        let kept = saved.last().expect("the state was saved");
        assert!(
            kept.refused_keys().contains(&(0x700, 0x22, 0x1F03)),
            "{kept:?}"
        );
        assert!(kept.requests.iter().any(|r| r.answered.is_some()));
        // and later passes: stopped between passes, still Ok
        STOP_AFTER.with(|s| s.set(40));
        ASKED.with(|a| a.set(0));
        let ran = w
            .run(
                &mut Elm::connect(Box::new(SimCar::new())).unwrap(),
                &mut Pacer::new(Instant::now, no_sleep, stop_after),
                &mut State::default(),
                &mut |_| Ok(()),
                None,
                &|_| {},
            )
            .unwrap();
        assert!(ran.passes >= 1 && ran.stopped.is_some(), "{ran:?}");
        // stopped before anything was asked (a recording owns the adapter): an error, nothing sent
        STOP_AFTER.with(|s| s.set(0));
        ASKED.with(|a| a.set(0));
        let e = w
            .run(
                &mut Elm::connect(Box::new(SimCar::new())).unwrap(),
                &mut Pacer::new(Instant::now, no_sleep, stop_after),
                &mut State::default(),
                &mut |_| Ok(()),
                None,
                &|_| {},
            )
            .unwrap_err();
        assert!(e.contains("Ctrl+C"));
    }

    /// A bus log written for this test (made-up identifiers and a made-up VIN with Toyota's maker code).
    const CAPTURE: &str = "   0.100       7E0  03 22 12 34 00 00 00 00
   0.110       7E8  05 62 12 34 01 02 00 00
   0.200       7E0  03 22 F1 90 00 00 00 00
   0.210       7E8  10 14 62 F1 90 4A 54 58
   0.211       7E0  30 00 00 00 00 00 00 00
   0.220       7E8  21 58 58 58 41 30 30 48
   0.230       7E8  22 30 30 30 30 30 30 31
   0.250       7E0  03 22 23 45 00 00 00 00
   0.260       7E8  06 62 23 45 0A 0B 0C 00
   0.300       780  03 22 01 01 00 00 00 00
   0.310       788  04 62 01 01 05 00 00 00
   0.400       7E0  04 2E 12 34 01 00 00 00
   0.410       7E8  03 6E 12 34 00 00 00 00
   0.500       7DF  02 01 0C 00 00 00 00 00
   0.510       7E8  04 41 0C 0B B8 00 00 00
   0.600       7B0  02 21 05 00 00 00 00 00
   0.610       7B8  03 7F 21 12 00 00 00 00
";

    /// ICA-106: a capture with a module's serial number in it (22 F18C, and KWP2000's 1A 89) gives a transcript and a set
    /// without its bytes, --keep-vin or not: masked in the transcript, and its read left out of the set (identity), so
    /// nothing reads it on the vehicle later.
    #[test]
    fn a_capture_with_a_serial_number_gives_neither_transcript_nor_set_its_bytes() {
        // made-up serial numbers "SN01" and "SN02"
        let log = "   0.100       7E0  03 22 F1 8C 00 00 00 00
   0.110       7E8  07 62 F1 8C 53 4E 30 31
   0.200       7E0  02 1A 89 00 00 00 00 00
   0.210       7E8  06 5A 89 53 4E 30 32 00
   0.300       7E0  03 22 12 34 00 00 00 00
   0.310       7E8  05 62 12 34 01 02 00 00
";
        let all = vehicles::builtin();
        for keep_vin in [false, true] {
            let l = learn(
                log.as_bytes(),
                "bench.log",
                None,
                Some("toyota"),
                keep_vin,
                &all,
            )
            .unwrap();
            assert_eq!(l.serials, 2);
            let csv = capture::transcript_csv(&l.read.exchanges, &|_| String::new());
            assert!(
                !csv.contains("53 4E 30") && csv.matches("[serial number masked]").count() == 2,
                "{csv}"
            );
            assert!(csv.contains("62 12 34 01 02"), "the rest stays: {csv}");
            let files = md::set_files(l.imp.set.as_ref().unwrap(), &l.imp.modules);
            for (name, text) in &files {
                assert!(!text.contains("53 4E 30"), "{name}: {text}");
                assert!(!text.contains("0xF18C"), "{name}: {text}");
            }
            assert!(
                l.imp.left_out.iter().any(|(w, y)| w == "7E0 22 F1 8C"
                    && y == "the module's serial number (identity): seen, not imported"),
                "{:?}",
                l.imp.left_out
            );
        }
    }

    /// ICA-61: a capture set's attribution follows its naming: unnamed at first, "N of M named" after one, "named by hand,
    /// not verified" once every placeholder is named; a hand-written attribution is never changed.
    #[test]
    fn the_attribution_says_when_every_value_is_named() {
        let root = std::env::temp_dir().join(format!("icarus-attr-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).unwrap();
        let all = vehicles::builtin();
        let l = learn(CAPTURE.as_bytes(), "truck.log", None, None, false, &all).unwrap();
        moddata::write_set(&root, &l.imp, false).unwrap();
        let attribution = || {
            moddata::read_dir(&root.join("truck"))
                .unwrap()
                .0
                .source
                .attribution
        };
        assert_eq!(attribution(), capture::ATTRIBUTION);
        let placeholders: Vec<(String, String)> = moddata::read_dir(&root.join("truck"))
            .unwrap()
            .1
            .iter()
            .flat_map(|m| {
                let label = m.module.label();
                m.requests
                    .iter()
                    .flat_map(|r| &r.values)
                    .filter(|v| v.note.contains("not named, not checked"))
                    .map(move |v| (label.clone(), v.id.clone()))
                    .collect::<Vec<_>>()
            })
            .collect();
        assert!(placeholders.len() >= 2, "{placeholders:?}");
        for (k, (module, id)) in placeholders.iter().enumerate() {
            let a: Vec<String> = ["name", "truck", module, id, "--name", &format!("Value {k}")]
                .iter()
                .map(|s| s.to_string())
                .collect();
            name_value(&root, &parse_args(&a, &DATA_OPTIONS).unwrap(), "2026-11-24").unwrap();
            if k == 0 {
                assert_eq!(
                    attribution(),
                    format!(
                        "learned from a capture of another tool: 1 of {} values named by hand, the rest unnamed; not verified",
                        placeholders.len()
                    )
                );
            }
        }
        assert_eq!(
            attribution(),
            "learned from a capture of another tool: values named by hand, not verified"
        );
        // the set still reads, and a hand-written attribution stays as written
        moddata::load_set("truck", Some(&root), &all).unwrap();
        let set = root.join("truck/set.toml");
        let text = std::fs::read_to_string(&set).unwrap().replace(
            "learned from a capture of another tool: values named by hand, not verified",
            "my own notes",
        );
        std::fs::write(&set, text).unwrap();
        named_attribution(&root.join("truck")).unwrap();
        assert_eq!(attribution(), "my own notes");
        let _ = std::fs::remove_dir_all(&root);
    }

    #[test]
    fn a_placeholder_of_a_capture_set_is_named_and_decodes_its_samples() {
        let root = std::env::temp_dir().join(format!("icarus-name-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).unwrap();
        let all = vehicles::builtin();
        let l = learn(CAPTURE.as_bytes(), "truck.log", None, None, false, &all).unwrap();
        moddata::write_set(&root, &l.imp, false).unwrap();
        let name = |words: &[&str]| {
            let a: Vec<String> = words.iter().map(|s| s.to_string()).collect();
            name_value(&root, &parse_args(&a, &DATA_OPTIONS).unwrap(), "2026-11-21")
        };
        // 22 1234 answered 01 02: the second byte is a temperature 40 below its raw value (a new value beside the
        // placeholder), the first a mode (the placeholder itself, narrowed to its byte)
        let n = name(&[
            "name",
            "truck",
            "7E0",
            "RAW_22_1234",
            "--keep",
            "--id",
            "ECT",
            "--name",
            "Coolant temperature",
            "--byte",
            "1",
            "--bits",
            "8",
            "--add",
            "-40",
            "--unit",
            "°C",
        ])
        .unwrap();
        assert_eq!((n.id.as_str(), n.module.module.tx), ("ECT", 0x7E0));
        name(&[
            "name",
            "truck",
            "7e0",
            "raw_22_1234",
            "--id",
            "MODE",
            "--name",
            "Mode",
            "--bits",
            "8",
            "--kind",
            "hex",
        ])
        .unwrap();
        // as icarus data truck --sample shows it, from the files as Icarus reads them
        let s = moddata::load_set("truck", Some(&root), &all).unwrap();
        let m = s.modules.iter().find(|m| m.module.tx == 0x7E0).unwrap();
        let r = m.requests.iter().find(|r| r.ident == 0x1234).unwrap();
        let ids: Vec<&str> = r.values.iter().map(|v| v.id.as_str()).collect();
        assert_eq!(ids, vec!["MODE", "ECT"]);
        let ect = &r.values[1];
        assert_eq!(
            (ect.name.as_str(), ect.byte, ect.bits),
            ("Coolant temperature", 1, 8)
        );
        assert!(
            !ect.checked && ect.note.contains("named by hand"),
            "{}",
            ect.note
        );
        assert!(
            r.values[0].note.contains("was RAW_22_1234"),
            "{}",
            r.values[0].note
        );
        let lines = sample_lines(r, &ids, false).unwrap();
        assert!(
            lines
                .iter()
                .any(|l| l.contains("ECT") && l.contains("-38 °C")),
            "{lines:?}"
        );
        assert!(lines.iter().any(|l| l.contains("MODE")), "{lines:?}");
        // what is refused, with nothing written
        let e = |words: &[&str]| name(words).err().unwrap_or_default();
        assert!(e(&["name", "truck", "7E0", "NOPE", "--name", "x"]).contains("no value NOPE"));
        assert!(
            e(&["name", "truck", "7E0", "ECT", "--name", "x", "--byte", "9"])
                .contains("nothing written")
        );
        assert!(e(&[
            "name",
            "truck",
            "7E0",
            "RAW_22_2345",
            "--name",
            "x",
            "--unit",
            "km"
        ])
        .contains("--bits"));
        assert!(e(&["name", "truck", "7E0", "ECT", "--keep", "--name", "x"]).contains("--id"));
        assert!(e(&["name", "sim", "700", "SIM_ECT", "--name", "x"]).contains("built-in"));
        std::fs::create_dir_all(root.join("hand")).unwrap();
        std::fs::write(
            root.join("hand/set.toml"),
            "format = 1\nname = \"hand\"\n\n[source]\nfrom = \"hand\"\n",
        )
        .unwrap();
        assert!(e(&["name", "hand", "7E0", "X", "--name", "y"]).contains("not from a capture"));
        let again = moddata::load_set("truck", Some(&root), &all).unwrap();
        assert_eq!(again.modules, s.modules, "a refused naming changes nothing");
        let _ = std::fs::remove_dir_all(&root);
    }

    #[test]
    fn a_capture_becomes_a_gated_set_and_a_transcript_without_the_vin() {
        let all = vehicles::builtin();
        let l = learn(CAPTURE.as_bytes(), "truck.log", None, None, false, &all).unwrap();
        let set = l.imp.set.as_ref().unwrap();
        assert_eq!((set.name.as_str(), set.make.as_str()), ("truck", "toyota"));
        assert_eq!(l.make_from, "the VIN in the capture");
        assert_eq!(set.years.len(), 1, "the VIN's model year");
        assert_eq!(set.source.from, "bus capture");
        let ecm = l.imp.modules.iter().find(|m| m.module.tx == 0x7E0).unwrap();
        assert_eq!(ecm.module.name, "engine (ECM)", "named from the profile");
        let labels: Vec<String> = ecm.requests.iter().map(|r| r.label()).collect();
        assert_eq!(
            labels,
            vec!["22 1234", "22 2345"],
            "the VIN's read left out"
        );
        assert_eq!(l.masked, 1);
        assert!(
            l.imp.modules.iter().all(|m| m.module.tx != 0x780),
            "airbag module: identity only, so no file with nothing in it"
        );
        let why = |w: &str| {
            l.imp
                .left_out
                .iter()
                .find(|(x, _)| x.contains(w))
                .map(|x| x.1.clone())
                .unwrap_or_else(|| panic!("{w}: {:?}", l.imp.left_out))
        };
        assert!(why("780 22 0101").contains("airbag"));
        assert_eq!(
            why("7E0 22 F1 90"),
            "the VIN (identity): seen, not imported"
        );
        assert!(why("2E 12 34").contains("seen, not imported"));
        assert!(why("01 0C").contains("seen, not imported"));
        assert!(why("7B0 21 05").contains("refused"));
        let csv = capture::transcript_csv(&l.read.exchanges, &|c| uds::nrc(c).to_string());
        assert!(
            !csv.contains("4A 54 58") && csv.contains("[VIN masked]"),
            "{csv}"
        );
        assert!(
            csv.contains("7B0/7B8,read data (KWP2000 local ID),21 05,7F 21 12 (refused: "),
            "{csv}"
        );
        // the files read back and load through the gates
        let files = md::set_files(set, &l.imp.modules);
        let (s2, mods, problems) = moddata::read_files(&files, "test").unwrap();
        assert!(problems.is_empty(), "{problems:?}");
        assert_eq!((s2, mods.len()), (set.clone(), l.imp.modules.len()));
        // --keep-vin keeps it; --make names the make
        let kept = learn(
            CAPTURE.as_bytes(),
            "truck.log",
            Some("mine"),
            Some("toyota"),
            true,
            &all,
        )
        .unwrap();
        assert_eq!(kept.masked, 0);
        assert_eq!(kept.make_from, "--make");
        assert!(
            capture::transcript_csv(&kept.read.exchanges, &|_| String::new()).contains("4A 54 58")
        );
        // with no make known, a module of unknown kind is refused (it could be the airbag module)
        let unknown = learn(
            b"0.1 7A0 03 22 12 34\n0.2 7A8 04 62 12 34 01\n",
            "x.log",
            None,
            None,
            false,
            &all,
        )
        .unwrap();
        assert!(
            unknown.imp.modules.is_empty(),
            "every request refused: no module file"
        );
        assert!(unknown
            .imp
            .left_out
            .iter()
            .any(|(_, y)| y.contains("could be the airbag module")));
    }
}
