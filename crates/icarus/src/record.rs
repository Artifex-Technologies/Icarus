//! `icarus record`: a drive recorded with no window (ICA-208), by hand (`icarus record --adapter A`, until Ctrl+C or
//! SIGTERM) or by itself (`--auto`, what the `icarus-record` user service runs: it waits for the adapter and the engine,
//! and ends each drive once the engine has been off for a while). The recorder is the window's (`icarus_obd::drive`); the
//! sounds and notifications are `icarus_obd::notify`'s (ICA-209). `icarus record redecode` decodes a recording's module
//! values again from the answers it kept (ICA-44).

use super::{open_link, parse_args, Args};
use icarus_obd::drive::{self, Config, Driver, Modules, Plan};
use icarus_obd::link::Link;
use icarus_obd::moddata::{self, Pacer};
use icarus_obd::notify::{self, Notifier};
use icarus_obd::port::Target;
use icarus_obd::{actuator, drivelog, j1979, vehicledata, vehicles};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

pub const USAGE: &str = "usage: icarus record [--adapter A] [--auto [--off-after SECS]] [--quiet] [--gps SOURCE] [--no-modules]\n       \
icarus record redecode FILE.csv [--set SET] [--year YYYY] [--out NEW.csv]";

pub fn cmd_record(a: &[String]) -> Result<(), String> {
    if a.first().map(String::as_str) == Some("redecode") {
        return redecode(&parse_args(&a[1..], &["set", "year", "out"])?);
    }
    let args = parse_args(a, &["adapter", "off-after", "gps"])?;
    if !args.pos.is_empty() {
        return Err(USAGE.into());
    }
    let auto = args.opt("auto").is_some();
    // the adapter named, else ICARUS_ADAPTER, else the one the window used last
    let spec = args
        .opt("adapter")
        .map(str::to_string)
        .or_else(|| std::env::var("ICARUS_ADAPTER").ok().filter(|s| !s.trim().is_empty()))
        .or_else(drive::remembered_adapter)
        .ok_or(
            "which adapter? --adapter /dev/ttyUSB0@115200 | bt:AA:BB:CC:DD:EE:FF | sim:drive ... (or set ICARUS_ADAPTER; \
             --auto also takes the one the window connected to last)",
        )?;
    let target = Target::parse(&spec)?;
    let mut cfg = Config {
        auto,
        ..Config::default()
    };
    if let Some(s) = args.opt("off-after") {
        let secs: u64 = s
            .parse()
            .ok()
            .filter(|n| *n >= 5)
            .ok_or("--off-after takes seconds, 5 or more")?;
        cfg.off_after = Duration::from_secs(secs);
    }
    let quiet = args.opt("quiet").is_some() || notify::quiet_from_env();
    let dir = drivelog::folders()
        .into_iter()
        .next()
        .ok_or("no home folder")?;
    drive::catch_stop_signals();
    let mut cli = Cli {
        spec: spec.clone(),
        target,
        gps: args.opt("gps").map(str::to_string),
        modules: args.opt("no-modules").is_none(),
        profiles: vehicles::load(vehicles::user_dir().as_deref()).0,
        dir,
        pending: Vec::new(),
        how: None,
    };
    let mut n = Notifier::system(quiet);
    cli.say(&format!(
        "{} on {spec}{}; Ctrl+C or SIGTERM stops, closing the file.",
        if auto {
            "Recording each drive by itself"
        } else {
            "Recording a drive"
        },
        if quiet { " (silent)" } else { "" }
    ));
    let r = drive::run(&cfg, &mut cli, &mut n);
    cli.keep_outcomes();
    let files = r?;
    for f in &files {
        if let Ok(s) = drivelog::summarize_file(f) {
            // the summary's first lines: readings, minutes, top speed in mph and km/h
            if let Some(l) = s.lines().nth(2) {
                eprintln!("{}: {l}", f.display());
            }
        }
    }
    Ok(())
}

/// The command line's side of the loop: the real adapter, the vehicle's set, the terminal (or the journal).
struct Cli {
    spec: String,
    target: Target,
    gps: Option<String>,
    modules: bool,
    profiles: Vec<vehicles::Profile>,
    dir: PathBuf,
    /// What the modules did with each request, to keep in the vehicle's record (ICA-70): (VIN, outcomes).
    pending: Vec<icarus_obd::record::Outcome>,
    /// The VIN, adapter and protocol they were asked through.
    how: Option<(String, String, String)>,
}

impl Cli {
    fn keep_outcomes(&mut self) {
        if let Some((vin, adapter, protocol)) = &self.how {
            if !self.pending.is_empty() {
                if let Err(e) = icarus_obd::record::merge(vin, &self.pending, adapter, protocol) {
                    self.say(&format!("Note: the vehicle's record was not kept: {e}"));
                }
            }
        }
        self.pending.clear();
    }
}

/// Module data's pacer, stopping on Ctrl+C or SIGTERM only (a recording owns the adapter: it is this one).
fn pacer() -> Pacer {
    fn stop() -> Option<String> {
        actuator::interrupted().then(|| "stopped".to_string())
    }
    let gap = std::env::var("ICARUS_DATA_GAP_MS")
        .ok()
        .and_then(|v| v.parse::<u64>().ok())
        .map(Duration::from_millis)
        .unwrap_or(moddata::MIN_GAP);
    Pacer::new(Instant::now, std::thread::sleep, stop).with_gap(gap)
}

impl Driver for Cli {
    fn open(&mut self) -> Result<Box<dyn Link>, String> {
        let (link, note) = open_link(&self.spec, &self.target)?;
        if let Some(n) = note {
            self.say(&format!("Note: {n}"));
        }
        Ok(link)
    }

    fn plan(&mut self, link: &mut dyn Link) -> Result<Plan, String> {
        let mut offered: Vec<u8> = j1979::supported(link)
            .map_err(|e| e.to_string())?
            .into_iter()
            .flat_map(|s| s.2)
            .filter(|p| j1979::pid(*p).is_some())
            .collect();
        offered.sort();
        offered.dedup();
        let vin = j1979::vin(link).ok().flatten();
        let modules = if self.modules {
            match vehicledata::choose_for(link, &self.profiles) {
                Ok(c) => {
                    for n in &c.notes {
                        self.say(&format!("Module data: {n}"));
                    }
                    self.say(&format!(
                        "Module data from the set {} (read in turn, its answers kept).",
                        c.set.set.name
                    ));
                    if !c.set.set.source.attribution.is_empty() {
                        self.say(&c.set.set.source.attribution);
                    }
                    Some(Modules::new(c.set, c.scope, pacer()))
                }
                Err(e) => {
                    self.say(&format!("Module data: {e}. Standard values only."));
                    None
                }
            }
        } else {
            None
        };
        self.how = vin
            .clone()
            .map(|v| (v, link.adapter(), link.protocol().describe().to_string()));
        Ok(Plan {
            offered,
            adapter: link.adapter(),
            modules,
            gps: self.gps.clone(),
            dir: self.dir.clone(),
            keep_awake: true,
            vin,
        })
    }

    fn stop(&self) -> Option<String> {
        actuator::interrupted().then(|| "stopped (Ctrl+C or SIGTERM)".to_string())
    }

    fn say(&mut self, line: &str) {
        eprintln!("icarus record: {line}");
    }

    fn outcomes(
        &mut self,
        _vin: Option<&str>,
        _link: &dyn Link,
        o: Vec<icarus_obd::record::Outcome>,
    ) {
        self.pending.extend(o);
        if self.pending.len() >= 500 {
            self.keep_outcomes();
        }
    }
}

/// `icarus record redecode FILE`: the file's module values decoded again with the set as it is now, into a new file beside
/// it (never over one).
fn redecode(args: &Args) -> Result<(), String> {
    let [file] = args.pos.as_slice() else {
        return Err(USAGE.into());
    };
    let path = Path::new(file);
    let text = std::fs::read_to_string(path).map_err(|e| format!("{file}: {e}"))?;
    let named = drive::module_data_of(&text);
    let set = args
        .opt("set")
        .map(str::to_string)
        .or_else(|| named.as_ref().map(|n| n.0.clone()))
        .ok_or("the recording names no module-data set: --set SET")?;
    let year = match args.opt("year") {
        Some(y) => Some(y.parse::<u16>().map_err(|_| "--year takes a model year")?),
        None => named.and_then(|n| n.1),
    };
    let profiles = vehicles::load(vehicles::user_dir().as_deref()).0;
    let loaded = moddata::load_set(&set, moddata::user_dir().as_deref(), &profiles)?;
    let out_text = drive::redecode(&text, &loaded.modules, year)?;
    let out = match args.opt("out") {
        Some(o) => PathBuf::from(o),
        None => path.with_file_name(format!(
            "{}-redecoded.csv",
            path.file_stem()
                .map(|s| s.to_string_lossy().into_owned())
                .unwrap_or_else(|| "drive".into())
        )),
    };
    use std::io::Write;
    let mut f = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&out)
        .map_err(|e| format!("{}: {e} (a new file only, never over one)", out.display()))?;
    f.write_all(out_text.as_bytes())
        .map_err(|e| format!("{}: {e}", out.display()))?;
    println!(
        "{}: decoded again with the set {set}{} into {}",
        path.display(),
        year.map(|y| format!(" for {y}")).unwrap_or_default(),
        out.display()
    );
    if !loaded.set.source.attribution.is_empty() {
        println!("{}", loaded.set.source.attribution);
    }
    Ok(())
}
