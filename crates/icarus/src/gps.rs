//! GPS on the command line: `icarus gps` (what the receiver hears, and what that says about it and its antenna) and
//! `icarus gps speed` (GPS speed against the vehicle's own, for the speedometer check). Positions are shown only when asked
//! (`--position`) and never written anywhere.

use super::{connect, parse_args};
use icarus_core::moddata::units;
use icarus_obd::gps::{self, Fix, Source};
use icarus_obd::j1979;

pub const USAGE: &str = "usage: icarus gps [--gps SOURCE] [--seconds N] [--position]\n       \
icarus gps speed --adapter A [--gps SOURCE] [--seconds N]";

fn source(spec: Option<&str>) -> Result<(String, Source), String> {
    let spec = spec
        .map(str::to_string)
        .or_else(|| std::env::var("ICARUS_GPS").ok().filter(|s| !s.is_empty()))
        .ok_or("which GPS? --gps /dev/ttyACM0 (a USB receiver), bt:AA:BB:CC:DD:EE:FF, tcp:HOST:PORT (a phone sharing \
                its GPS) or sim; or set ICARUS_GPS")?;
    eprintln!("listening to {spec}...");
    Ok((spec.clone(), Source::open(&spec)?))
}

fn speed_text(kmh: f64) -> String {
    if units::us_customary() {
        format!("{:.1} mph ({kmh:.1} km/h)", kmh / 1.609_344)
    } else {
        format!("{kmh:.1} km/h")
    }
}

fn print_fix(fx: &Fix, seconds: f64, position: bool) {
    println!("Fix         {}", fx.summary());
    if let Some(v) = fx.speed_kmh.filter(|_| fx.valid) {
        println!("Speed       {}", speed_text(v));
    }
    if let (Some(t), Some(d)) = (&fx.time_utc, &fx.date) {
        println!(
            "Time (UTC)  20{}-{}-{} {}:{}:{}",
            &d[4..6],
            &d[2..4],
            &d[0..2],
            &t[0..2],
            t.get(2..4).unwrap_or(""),
            t.get(4..6).unwrap_or("")
        );
    }
    if position {
        if let (Some(la), Some(lo)) = (fx.lat, fx.lon) {
            println!("Position    {la:.6}, {lo:.6}");
        }
    } else if fx.lat.is_some() {
        println!("Position    (known; shown with --position)");
    }
    if let Some(p) = fx.pdop {
        println!(
            "Precision   PDOP {p:.1}, HDOP {}, VDOP {} (under 2 is good, over 5 poor)",
            fx.hdop.map_or("-".into(), |h| format!("{h:.1}")),
            fx.vdop.map_or("-".into(), |v| format!("{v:.1}"))
        );
    }
    let (heard, _) = fx.heard();
    println!(
        "\nSatellites in view: {} ({heard} with a signal)",
        fx.sats.len()
    );
    let mut sats: Vec<_> = fx.sats.values().collect();
    sats.sort_by(|a, b| b.snr.cmp(&a.snr).then(a.prn.cmp(&b.prn)));
    for s in sats {
        let bar = s
            .snr
            .map_or(String::new(), |n| "#".repeat((n as usize).div_ceil(3)));
        println!(
            "  {:<8} {:>3}  elevation {:>3}  signal {:>8}  {bar}",
            s.system,
            s.prn,
            s.elevation.map_or("-".into(), |e| format!("{e}°")),
            s.snr.map_or("none".into(), |n| format!("{n} dB-Hz"))
        );
    }
    println!(
        "\n{} sentences read{}",
        fx.sentences,
        if fx.dropped > 0 {
            format!(", {} dropped (bad checksum)", fx.dropped)
        } else {
            String::new()
        }
    );
    for d in fx.diagnose(seconds) {
        println!("- {d}");
    }
}

pub fn cmd_gps(a: &[String]) -> Result<(), String> {
    let (sub, rest) = match a.first().map(String::as_str) {
        Some("speed") => (Some("speed"), &a[1..]),
        _ => (None, a),
    };
    let args = parse_args(rest, &["gps", "seconds", "adapter"])?;
    if !args.pos.is_empty() {
        return Err(USAGE.into());
    }
    let seconds = match args.opt("seconds") {
        Some(s) => s
            .parse::<f64>()
            .ok()
            .filter(|s| (1.0..=600.0).contains(s))
            .ok_or("--seconds: 1 to 600")?,
        None if sub.is_some() => 10.0,
        None => 15.0,
    };
    let (_, mut src) = source(args.opt("gps"))?;
    match sub {
        None => {
            let fx = gps::listen(&mut src, seconds)?;
            print_fix(&fx, seconds, args.opt("position").is_some());
            Ok(())
        }
        Some(_) => speed(&args, &mut src, seconds),
    }
}

/// GPS speed and the vehicle's (PID 0D) side by side, once a second, then the average error.
fn speed(args: &super::Args, src: &mut Source, seconds: f64) -> Result<(), String> {
    let mut elm = connect(args)?;
    println!("Hold a steady speed on a straight, level road. Comparing for {seconds:.0} s:\n");
    let mut errors = Vec::new();
    let mut dead = 0;
    let rounds = seconds.round() as usize;
    for _ in 0..rounds {
        let fx = gps::listen(src, 1.0)?;
        let vehicle = j1979::read(&mut elm, 0x0D)
            .map_err(|e| e.to_string())?
            .first()
            .map(|r| r.value);
        let gps_kmh = fx.speed_kmh.filter(|_| fx.valid);
        match (vehicle, gps_kmh) {
            (Some(v), Some(g)) if gps::speed_sensor_dead(v, g) => {
                dead += 1;
                println!(
                    "  vehicle {:<26} GPS {:<26} the vehicle reports no speed while moving",
                    speed_text(v),
                    speed_text(g)
                );
            }
            (Some(v), Some(g)) => {
                let err = gps::speed_error(v, g);
                println!(
                    "  vehicle {:<26} GPS {:<26} {}",
                    speed_text(v),
                    speed_text(g),
                    err.map_or("(too slow to compare)".into(), |e| format!("{e:+.1} %"))
                );
                errors.extend(err);
            }
            (None, _) => println!("  the vehicle did not report its speed"),
            (_, None) => println!("  no GPS fix ({})", fx.summary()),
        }
    }
    if dead >= 3 && dead * 2 >= rounds {
        println!(
            "\nThe vehicle reported no speed while GPS had it moving: its speed signal is missing (a vehicle speed or \
             wheel speed sensor, its wiring, or the module that sends it). Its trouble codes should say which; the \
             speedometer, cruise control and shifting can all suffer."
        );
        return Ok(());
    }
    if errors.len() < 3 {
        return Err("too few readings with both speeds above 5 km/h to judge".into());
    }
    let avg = errors.iter().sum::<f64>() / errors.len() as f64;
    let spread = errors.iter().fold(0f64, |m, e| m.max((e - avg).abs()));
    println!(
        "\nThe vehicle's speed reads {:.1} % {} (from {} readings, within ±{spread:.1} %).",
        avg.abs(),
        if avg < 0.0 { "low" } else { "high" },
        errors.len()
    );
    if spread > 2.0 {
        println!(
            "The readings varied: hold the speed steadier, or compare for longer (--seconds 30)."
        );
    }
    if avg.abs() > 2.0 {
        println!(
            "A steady error like this is what a different tire size (or axle ratio, where speed is measured at the \
             transmission) gives: icarus tires works out which. The engine computer uses this speed too, for shift points \
             and the speed limiter."
        );
    }
    Ok(())
}
