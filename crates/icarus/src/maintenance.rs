//! `icarus garage service [VIN] add|list|interval|due|compare|odometer`: each vehicle's maintenance log (ICA-185) in its
//! folder's `maintenance.toml` (icarus-obd's `maintenance`), and the drives before a service against the drives after it
//! (ICA-215). The VIN may be left out when the garage holds one vehicle. The odometer is typed in; a reading the vehicle
//! gives (a recording's, or `odometer --adapter A`) is only offered, for the owner to confirm. Nothing is written to the
//! vehicle.

use super::{connect, parse_args, profiles};
use icarus_obd::maintenance::{self as mt, Kind, Offer, Service, When};
use icarus_obd::{drivelog, garage, j1979, vehicledata, vin};
use std::io::IsTerminal;
use std::path::PathBuf;

pub const USAGE: &str = "usage: icarus garage service [VIN] add KIND [--miles N] [--date YYYY-MM-DD[ HH:MM]] [--note TEXT]
       icarus garage service [VIN] list
       icarus garage service [VIN] interval KIND [--miles N] [--months M]   (KIND off: no interval)
       icarus garage service [VIN] due [--miles N]
       icarus garage service [VIN] compare KIND|DATE [--drives N] [--min-rows N]
       icarus garage service [VIN] odometer [--adapter A]
  KIND: oil, spark-plugs, air-filter, fluids, brakes, tire-rotation, sensor, fuel, other (with --note)";

const WORDS: [&str; 6] = ["add", "list", "interval", "due", "compare", "odometer"];

/// The vehicle the command is for: the VIN given, else the garage's only vehicle; and its folder when it has one.
fn vehicle(a: &[String]) -> Result<(String, Option<PathBuf>, &[String]), String> {
    let (vin, rest) = match a.first() {
        Some(w) if !WORDS.contains(&w.as_str()) => (vin::decode(w)?.vin, &a[1..]),
        _ => {
            let folders = garage::vehicles();
            let mut vins: Vec<(String, Option<String>)> = folders
                .iter()
                .map(|(v, n, _)| (v.clone(), n.clone()))
                .collect();
            if vins.is_empty() {
                vins = garage::load()
                    .into_iter()
                    .map(|(v, n)| (v, Some(n)))
                    .collect();
            }
            match vins.as_slice() {
                [(v, _)] => (v.clone(), a),
                [] => return Err(
                    "no vehicle in the garage yet: give its VIN (icarus garage service VIN ...)"
                        .into(),
                ),
                many => {
                    return Err(format!(
                        "the garage holds {} vehicles: give the VIN of one ({})",
                        many.len(),
                        many.iter()
                            .map(|(v, n)| match n {
                                Some(n) => format!("{v} {n}"),
                                None => v.clone(),
                            })
                            .collect::<Vec<_>>()
                            .join("; ")
                    ))
                }
            }
        }
    };
    let dir = garage::vehicles()
        .into_iter()
        .find(|(v, _, _)| *v == vin)
        .map(|(_, _, d)| d);
    Ok((vin, dir, rest))
}

/// The vehicle's folder, made if needed (a log is written into it).
fn folder(vin: &str, dir: Option<PathBuf>) -> Result<PathBuf, String> {
    match dir {
        Some(d) => Ok(d),
        None => {
            garage::ensure(vin).map_err(|e| format!("{vin}: its folder could not be made: {e}"))
        }
    }
}

fn number(v: Option<&str>, what: &str) -> Result<Option<u32>, String> {
    v.map(|s| {
        s.replace(',', "")
            .trim()
            .parse::<u32>()
            .map_err(|_| format!("--{what} needs a whole number, not {s:?}"))
    })
    .transpose()
}

/// The recordings that may be this vehicle's, and a note on them.
fn recordings(dir: &std::path::Path) -> mt::Recordings {
    let shared = drivelog::folders().into_iter().next().unwrap_or_default();
    mt::recordings_for(&garage::library(), dir, &shared)
}

pub fn cmd_service(a: &[String]) -> Result<(), String> {
    let (vin, dir, rest) = vehicle(a)?;
    let args = parse_args(
        rest,
        &[
            "miles", "date", "note", "months", "drives", "min-rows", "adapter",
        ],
    )?;
    let Some((what, pos)) = args.pos.split_first() else {
        return Err(USAGE.into());
    };
    match (what.as_str(), pos) {
        ("add", [kind]) => {
            let kind = Kind::parse(kind)?;
            let dir = folder(&vin, dir)?;
            let date = match args.opt("date") {
                Some(d) => When::parse(d)?.text(),
                None => When::now().text(),
            };
            let recs = recordings(&dir);
            let mut miles = number(args.opt("miles"), "miles")?;
            if miles.is_none() {
                miles = confirm_offer(mt::offer_from(&recs))?;
            }
            let (s, warnings) = mt::add(
                &dir,
                Service::new(kind, &date, miles, args.opt("note").unwrap_or("")),
            )?;
            println!("Logged: {}", s.line());
            for w in warnings {
                println!("Note: {w}");
            }
            // which of that day's drives a before and after will count on which side
            if let Some(n) = s.when().and_then(|w| mt::day_note(w, &recs.paths)) {
                println!("Note: {n}.");
            }
            if let Some(i) = mt::load(&dir)?.interval(kind) {
                println!("Next {}: {}.", kind.name(), i.text());
            }
            println!("({})", mt::path(&dir).display());
            Ok(())
        }
        ("list", []) => {
            let Some(dir) = dir else {
                println!(
                    "{vin}: nothing logged yet (icarus garage service {vin} add KIND --miles N)."
                );
                return Ok(());
            };
            let log = mt::load(&dir)?;
            if log.services.is_empty() {
                println!("{vin}: nothing logged yet.");
            }
            for s in &log.services {
                println!("{}", s.line());
            }
            let intervals: Vec<String> = Kind::ALL
                .iter()
                .filter_map(|k| {
                    log.interval(*k)
                        .map(|i| format!("  {}: {}", k.name(), i.text()))
                })
                .collect();
            if !intervals.is_empty() {
                println!("\nIntervals:\n{}", intervals.join("\n"));
            }
            println!("\n({})", mt::path(&dir).display());
            Ok(())
        }
        ("interval", [kind, rest @ ..]) => {
            let kind = Kind::parse(kind)?;
            let dir = folder(&vin, dir)?;
            let (miles, months) = (
                number(args.opt("miles"), "miles")?,
                number(args.opt("months"), "months")?,
            );
            match rest {
                [off] if off == "off" => {
                    mt::set_interval(&dir, kind, None, None)?;
                    println!("{}: no interval.", kind.name());
                }
                [] if miles.is_none() && months.is_none() => {
                    let i = mt::load(&dir)?.interval(kind);
                    println!(
                        "{}: {}",
                        kind.name(),
                        i.map(|i| i.text()).unwrap_or("no interval".into())
                    );
                }
                [] => {
                    mt::set_interval(&dir, kind, miles, months)?;
                    println!(
                        "{}: {}.",
                        kind.name(),
                        mt::Interval::new(miles, months).text()
                    );
                }
                _ => return Err(USAGE.into()),
            }
            Ok(())
        }
        ("due", []) => {
            let log = match &dir {
                Some(d) => mt::load(d)?,
                None => mt::Log::default(),
            };
            let typed = number(args.opt("miles"), "miles")?;
            let (odometer, from) = match typed {
                Some(m) => (Some(m), "as given".to_string()),
                None => {
                    // the last odometer written down: the log's highest, or the Garage page's miles
                    let logged = log
                        .highest_miles()
                        .map(|(m, s)| (m, format!("as logged on {}", s.date)));
                    let details = garage::details(&vin)
                        .miles
                        .map(|m| (m, "as written on the Garage page".to_string()));
                    match [logged, details].into_iter().flatten().max_by_key(|x| x.0) {
                        Some((m, f)) => (Some(m), f),
                        None => (None, String::new()),
                    }
                }
            };
            let lines = mt::due(&log, When::now().day, odometer);
            if lines.is_empty() {
                println!(
                    "No intervals set: icarus garage service {vin} interval oil --miles 5000 --months 6"
                );
                return Ok(());
            }
            match odometer {
                Some(m) => println!("Odometer: {} ({from}).", mt::miles_text(m.into())),
                None => println!("Odometer not known: due by months only (give --miles N)."),
            }
            if typed.is_none() {
                if let Some(o) = dir.as_deref().and_then(|d| mt::offer_from(&recordings(d))) {
                    if Some(o.miles) > odometer {
                        println!(
                            "A reading to confirm: {}. If it is right: --miles {}",
                            o.text(),
                            o.miles
                        );
                    }
                }
            }
            for d in lines {
                println!("{}", d.text());
            }
            Ok(())
        }
        ("compare", [what]) => {
            let dir = dir.ok_or(format!(
                "{vin}: nothing logged yet, so nothing to compare around"
            ))?;
            let log = mt::load(&dir)?;
            let ev = mt::event(&log, what)?;
            let drives = number(args.opt("drives"), "drives")?
                .map(|n| n as usize)
                .unwrap_or(mt::DRIVES);
            let min_rows = number(args.opt("min-rows"), "min-rows")?
                .map(|n| n as usize)
                .filter(|n| *n > 0)
                .unwrap_or(drivelog::compare::MIN_ROWS);
            let recs = recordings(&dir);
            let mut ba = mt::before_after(&ev, &recs.paths, drives, min_rows)?;
            ba.note = recs.note;
            let warm = if icarus_core::moddata::units::us_customary() {
                icarus_obd::units::both("°C", drivelog::compare::WARM_C, 0)
            } else {
                format!("{:.0} °C", drivelog::compare::WARM_C)
            };
            print!("{}", ba.words(&warm));
            Ok(())
        }
        ("odometer", []) => {
            let mut offers: Vec<Offer> = vec![];
            if let Some(d) = &dir {
                offers.extend(mt::offer_from(&recordings(d)));
            }
            if args.opt("adapter").is_some() {
                let (o, notes) = read_odometer(&args, &vin)?;
                offers.extend(o);
                for n in notes {
                    println!("Note: {n}");
                }
            }
            if offers.is_empty() {
                println!(
                    "No odometer reading found for {vin} (a recording with an odometer column, or --adapter A for the \
                     vehicle's own). Type it in: --miles N."
                );
            }
            for o in &offers {
                println!("{}", o.text());
            }
            if !offers.is_empty() {
                println!(
                    "Readings are offered, never saved by themselves: confirm one with --miles N."
                );
            }
            Ok(())
        }
        _ => Err(USAGE.into()),
    }
}

/// A reading offered for a service logged without --miles: asked about at a terminal (Enter keeps none, y takes it, or
/// type the miles), only said otherwise. Never taken silently.
fn confirm_offer(offer: Option<Offer>) -> Result<Option<u32>, String> {
    let Some(o) = offer else {
        return Ok(None);
    };
    if !std::io::stdin().is_terminal() {
        println!(
            "Logged without the odometer. A reading to confirm: {}; if it is right, add miles = {} to the service in \
             its file, or log it with --miles next time.",
            o.text(),
            o.miles
        );
        return Ok(None);
    }
    print!(
        "Odometer: {}. Use it? y, or type the miles, or Enter for none: ",
        o.text()
    );
    use std::io::Write;
    let _ = std::io::stdout().flush();
    let mut line = String::new();
    std::io::stdin()
        .read_line(&mut line)
        .map_err(|e| e.to_string())?;
    let t = line.trim();
    Ok(match t {
        "" | "n" | "no" => None,
        "y" | "yes" => Some(o.miles),
        _ => Some(
            t.replace(',', "")
                .parse::<u32>()
                .map_err(|_| format!("{t:?} is not a number of miles"))?,
        ),
    })
}

/// The vehicle's own odometer, read (never changed): standard value A6, and the module-data set's odometer value when the
/// set has one. Only from the vehicle whose VIN this is.
fn read_odometer(args: &super::Args, want: &str) -> Result<(Vec<Offer>, Vec<String>), String> {
    let mut link = connect(args)?;
    let on = j1979::vin(link.as_mut()).ok().flatten();
    if on.as_deref() != Some(want) {
        return Err(format!(
            "the vehicle on the adapter gives {}, not {want}: its odometer is not this vehicle's",
            on.as_deref().unwrap_or("no VIN")
        ));
    }
    let (mut offers, mut notes) = (vec![], vec![]);
    match j1979::read(link.as_mut(), 0xA6) {
        Ok(r) => match r
            .first()
            .and_then(|x| mt::miles_of(x.value, "km").map(|m| (m, x)))
        {
            Some((m, x)) => offers.push(Offer {
                miles: m,
                from: format!("the vehicle's odometer (standard value A6, {})", x.module),
                doubt: None,
            }),
            None => notes.push("the vehicle does not give standard value A6 (odometer)".into()),
        },
        Err(e) => notes.push(format!("standard value A6 (odometer): {e}")),
    }
    let all = profiles();
    match vehicledata::choose_for(link.as_mut(), &all) {
        Ok(c) => {
            let module = c.set.modules.iter().find(|m| {
                m.requests
                    .iter()
                    .flat_map(|r| &r.values)
                    .any(|v| v.name.to_lowercase().contains("odometer"))
            });
            match module {
                Some(m) => {
                    let label = m.module.label();
                    match vehicledata::read(link.as_mut(), &all, Some(&label), false) {
                        Ok(vv) => {
                            let (o, n) = mt::offers_in(&vv, "");
                            offers.extend(o);
                            notes.extend(n);
                        }
                        Err(e) => notes.push(format!("module data: {e}")),
                    }
                }
                None => notes.push(format!("the set {} has no odometer value", c.set.set.name)),
            }
        }
        Err(e) => notes.push(format!("module data: {e}")),
    }
    Ok((offers, notes))
}
