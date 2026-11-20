//! Module data on the command line: `icarus import obdb`, `icarus import btlog|buslog` (a capture of another tool at
//! work), `icarus data` (offline) and `icarus obd data` (on the vehicle,
//! read-only: services 21 and 22 only, paced, never on 7DF or an answer ID, airbag modules identity only).

use super::{connect, e, parse_args, profiles, Args};
use icarus_core::moddata::{self as md, capture, obdb, units, ModuleDef, Shown, ValueDef};
use icarus_obd::link::Link;
use icarus_obd::moddata::{self, Loaded, Outcome, Pacer, Scope, State};
use icarus_obd::{actuator, garage, j1979, report, uds, vehicles, vin};
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

/// `icarus import obdb FILE.json [--name SET] [--make NAME] [--dry-run] [--replace]`, `icarus import btlog|buslog FILE ...`
pub fn cmd_import(a: &[String]) -> Result<(), String> {
    let args = parse_args(a.get(1..).unwrap_or(&[]), &["name", "make", "transcript"])?;
    match a.first().map(String::as_str) {
        Some("obdb") => import_obdb(&args),
        Some("btlog" | "buslog") => import_capture(&args),
        Some("ddt") => {
            let [path] = args.pos.as_slice() else {
                return Err("usage: icarus import ddt ECU.xml".into());
            };
            let text = std::fs::read_to_string(path).map_err(|x| format!("{path}: {x}"))?;
            md::ddt::parse(&text).map(|_| ())
        }
        _ => Err(format!(
            "usage: icarus import obdb FILE.json [--name SET] [--make NAME] [--dry-run] [--replace]\n{CAPTURE_USAGE}"
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
            .find(|a| u32::from(a.tx) == m.module.tx && a.ext.is_none())
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
    let all = profiles();
    let mut imp = obdb::parse(
        &text,
        &obdb::Options {
            path: path.clone(),
            name: args.opt("name").map(str::to_string),
            make: args.opt("make").map(str::to_string),
            today: today(),
        },
    )?;
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
        "OBDb signal set {path}\n  set {:?} for {} {}",
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

/// What a capture gave: what was read (VINs masked unless kept), the set (gates applied), and where its make came from.
struct Learned {
    read: capture::Read,
    imp: moddata::Imported,
    masked: usize,
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
        "Written to {}. To name a value once you know what it is: edit its module file there (name, unit, byte, bits, \
         mul/div/add), check it with icarus data check FILE.toml, and see it decode the samples with icarus data {} --sample",
        dir.display(),
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

/// `icarus data [SET] [--module ID] [--find TEXT] [--sample] [--refused]`, `icarus data check FILE.toml [--make NAME]`
pub fn cmd_data(a: &[String]) -> Result<(), String> {
    let args = parse_args(a, &["module", "find", "make"])?;
    let all = profiles();
    let user = moddata::user_dir();
    if args.pos.first().map(String::as_str) == Some("check") {
        return data_check(&args, &all);
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
                for (i, d) in r.samples_data()?.iter().enumerate() {
                    if let Outcome::Values(rs) = moddata::decode_all(r, d, None) {
                        for x in rs.iter().filter(|x| vals.iter().any(|v| v.id == x.id)) {
                            println!(
                                "      sample{}: {:<20} {}",
                                if i == 0 {
                                    String::new()
                                } else {
                                    format!(" {}", i + 1)
                                },
                                x.id,
                                x.text(units::us_customary())
                            );
                        }
                    }
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

/// The set for this vehicle: `wanted` (`--set`), else the one its state names, else the only one that fits. Nothing is
/// guessed: with the make, model year or model unknown, a set that names one does not fit (it is named with --set), and
/// the simulated vehicle's set is refused on any other vehicle however it is named.
fn choose<'a>(
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
        many => {
            Err(format!(
            "several sets fit ({}): choose one with --set NAME (it is remembered for this vehicle)",
            many.iter().map(|s| s.set.name.as_str()).collect::<Vec<_>>().join(", ")
        ))
        }
    }
}

/// The chosen set as used on this vehicle (`Loaded::on_vehicle`: the gates decided by the vehicle's make too), with a note
/// when the set is for another make or the vehicle's make is not known.
fn on_vehicle(set: &Loaded, all: &[vehicles::Profile], make: Option<&str>) -> Loaded {
    match make {
        Some(m) if !m.eq_ignore_ascii_case(&set.set.make) => eprintln!(
            "note: the set {:?} is for {:?}, this vehicle is {m:?}: the stricter of the two makes' airbag gates holds",
            set.set.name, set.set.make
        ),
        None if !set.set.make.is_empty() => eprintln!(
            "note: this vehicle's make is not known (--make NAME): a module of unknown kind in the set {:?} gets no data \
             request, since it could be the airbag module",
            set.set.name
        ),
        _ => {}
    }
    set.on_vehicle(all, make)
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
                            if self.set.gates.is_airbag(&m.module) {
                                " (airbag module: identity only)"
                            } else {
                                ""
                            }
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
pub fn obd_data(args: &Args) -> Result<(), String> {
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
    let set = &on_vehicle(set, &all, make.as_deref());
    let new_set = state.set.as_deref() != Some(set.set.name.as_str());
    state.set = Some(set.set.name.clone());
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
    let chosen: Vec<&ModuleDef> = if args.pos.is_empty() {
        set.modules
            .iter()
            .filter(|m| !m.requests.is_empty())
            .collect()
    } else {
        args.pos
            .iter()
            .map(|p| {
                // "701", or "750.2A" for a module behind an extended address
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
            .collect::<Result<_, String>>()?
    };
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
        let never: Vec<&str> = set.modules[0].requests[3..]
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
            _: u16,
            rx: u16,
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
   0.300       780  03 22 01 01 00 00 00 00
   0.310       788  04 62 01 01 05 00 00 00
   0.400       7E0  04 2E 12 34 01 00 00 00
   0.410       7E8  03 6E 12 34 00 00 00 00
   0.500       7DF  02 01 0C 00 00 00 00 00
   0.510       7E8  04 41 0C 0B B8 00 00 00
   0.600       7B0  02 21 05 00 00 00 00 00
   0.610       7B8  03 7F 21 12 00 00 00 00
";

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
        assert_eq!(labels, vec!["22 1234", "22 F190"]);
        let vin_sample = ecm.requests[1].sample.as_deref().unwrap();
        assert!(
            !vin_sample.contains("4A 54 58") && vin_sample.contains("2A 2A"),
            "{vin_sample}"
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
