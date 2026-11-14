//! Reading a recorded drive: the CSV `icarus obd live --csv` writes (a time column, then one column per value, named as
//! `"Short-term fuel trim, bank 1 [%]"`). Summarized the way a person reads fuel trims: by driving condition (idle, light,
//! moderate, hard), each bank's total trim (short + long term), the gap between the banks, the rear oxygen sensors, and the
//! sudden timing drops under load sorted into the rev limiter or a shift (RPM falls right after) and the rest. Offline: only
//! the file is read.

use std::path::{Path, PathBuf};

/// The columns a summary uses, found by their names.
#[derive(Debug, Default, Clone, Copy)]
struct Cols {
    rpm: Option<usize>,
    speed: Option<usize>,
    coolant: Option<usize>,
    stft1: Option<usize>,
    ltft1: Option<usize>,
    stft2: Option<usize>,
    ltft2: Option<usize>,
    timing: Option<usize>,
    maf: Option<usize>,
    throttle: Option<usize>,
    o2_1: Option<usize>,
    o2_2: Option<usize>,
    volts: Option<usize>,
}

fn find(header: &[String], want: &[&str]) -> Option<usize> {
    header.iter().position(|h| {
        let h = h.to_lowercase();
        want.iter().all(|w| h.contains(w))
    })
}

fn cols(header: &[String]) -> Cols {
    Cols {
        rpm: find(header, &["engine speed"]),
        speed: find(header, &["vehicle speed"]),
        coolant: find(header, &["coolant"]),
        stft1: find(header, &["short-term fuel trim", "bank 1"]),
        ltft1: find(header, &["long-term fuel trim", "bank 1"]),
        stft2: find(header, &["short-term fuel trim", "bank 2"]),
        ltft2: find(header, &["long-term fuel trim", "bank 2"]),
        timing: find(header, &["timing advance"]),
        maf: find(header, &["mass air flow"]),
        throttle: find(header, &["throttle position"]),
        o2_1: find(header, &["bank 1 sensor 2"]),
        o2_2: find(header, &["bank 2 sensor 2"]),
        volts: find(header, &["module voltage"]),
    }
}

/// A CSV line split on commas outside double quotes.
fn split(line: &str) -> Vec<String> {
    let mut out = vec![];
    let (mut cur, mut quoted) = (String::new(), false);
    for c in line.chars() {
        match c {
            '"' => quoted = !quoted,
            ',' if !quoted => out.push(std::mem::take(&mut cur)),
            _ => cur.push(c),
        }
    }
    out.push(cur);
    out
}

#[derive(Debug, Default, Clone, PartialEq)]
pub struct Band {
    pub name: &'static str,
    pub readings: usize,
    /// Mean total trim (short + long term), percent.
    pub bank1: Option<f64>,
    pub bank2: Option<f64>,
    /// Mean rear oxygen sensor voltage.
    pub o2_1: Option<f64>,
    pub o2_2: Option<f64>,
}

#[derive(Debug, Default, Clone, PartialEq)]
pub struct Summary {
    pub readings: usize,
    pub seconds: f64,
    pub top_kmh: Option<f64>,
    pub coolant_c: Option<(f64, f64)>,
    pub volts: Option<(f64, f64)>,
    pub bands: Vec<Band>,
    /// Sudden timing drops (4 degrees or more between readings) at 40 % throttle or more: at the rev limiter or a shift (RPM
    /// falls by 1000 or more right after), and the rest.
    pub limiter_or_shift: usize,
    pub other_drops: usize,
    /// Long-term trims at the start and at the end (bank 1, bank 2).
    pub ltft_start: (Option<f64>, Option<f64>),
    pub ltft_end: (Option<f64>, Option<f64>),
}

fn mean(v: &[f64]) -> Option<f64> {
    (!v.is_empty()).then(|| v.iter().sum::<f64>() / v.len() as f64)
}

pub fn summarize(text: &str) -> Result<Summary, String> {
    let mut lines = text.lines();
    let header = split(lines.next().ok_or("the file is empty")?);
    let c = cols(&header);
    if c.stft1.is_none() && c.ltft1.is_none() {
        return Err("no fuel trim columns in this log".into());
    }
    let rows: Vec<Vec<Option<f64>>> = lines
        .map(|l| {
            split(l)
                .iter()
                .map(|x| x.trim().parse::<f64>().ok())
                .collect()
        })
        .filter(|r: &Vec<_>| !r.is_empty())
        .collect();
    let get = |r: &Vec<Option<f64>>, i: Option<usize>| i.and_then(|i| r.get(i).copied().flatten());
    let mut s = Summary {
        readings: rows.len(),
        ..Default::default()
    };
    if let (Some(a), Some(b)) = (
        rows.first().and_then(|r| r[0]),
        rows.last().and_then(|r| r[0]),
    ) {
        s.seconds = b - a;
    }
    let range = |i: Option<usize>| {
        let v: Vec<f64> = rows
            .iter()
            .filter_map(|r| get(r, i))
            .filter(|x| *x > 0.0)
            .collect();
        (!v.is_empty()).then(|| {
            (
                v.iter().cloned().fold(f64::MAX, f64::min),
                v.iter().cloned().fold(f64::MIN, f64::max),
            )
        })
    };
    s.coolant_c = range(c.coolant);
    s.volts = range(c.volts);
    s.top_kmh = range(c.speed).map(|r| r.1);
    let total =
        |r: &Vec<Option<f64>>, st: Option<usize>, lt: Option<usize>| match (get(r, st), get(r, lt))
        {
            (None, None) => None,
            (a, b) => Some(a.unwrap_or(0.0) + b.unwrap_or(0.0)),
        };
    type Cond = fn(f64, f64, f64, f64) -> bool;
    let conds: [(&'static str, Cond); 4] = [
        ("idle", |rpm, kmh, _maf, _thr| kmh < 1.6 && rpm > 400.0),
        ("cruise / light", |_r, kmh, maf, thr| {
            kmh >= 1.6 && maf < 15.0 && thr < 50.0
        }),
        ("moderate", |_r, _k, maf, thr| maf >= 15.0 && thr < 50.0),
        ("hard", |_r, _k, _m, thr| thr >= 50.0),
    ];
    for (name, cond) in conds {
        let pick: Vec<&Vec<Option<f64>>> = rows
            .iter()
            .filter(|r| {
                let v = |i| get(r, i).unwrap_or(0.0);
                cond(v(c.rpm), v(c.speed), v(c.maf), v(c.throttle))
            })
            .collect();
        if pick.is_empty() {
            continue;
        }
        let b1: Vec<f64> = pick
            .iter()
            .filter_map(|r| total(r, c.stft1, c.ltft1))
            .collect();
        let b2: Vec<f64> = pick
            .iter()
            .filter_map(|r| total(r, c.stft2, c.ltft2))
            .collect();
        let o1: Vec<f64> = pick.iter().filter_map(|r| get(r, c.o2_1)).collect();
        let o2: Vec<f64> = pick.iter().filter_map(|r| get(r, c.o2_2)).collect();
        s.bands.push(Band {
            name,
            readings: pick.len(),
            bank1: mean(&b1),
            bank2: mean(&b2),
            o2_1: mean(&o1),
            o2_2: mean(&o2),
        });
    }
    for i in 1..rows.len() {
        let (a, b) = (&rows[i - 1], &rows[i]);
        let (Some(t0), Some(t1)) = (get(a, c.timing), get(b, c.timing)) else {
            continue;
        };
        if get(b, c.throttle).unwrap_or(0.0) < 40.0 || t0 - t1 < 4.0 {
            continue;
        }
        let fell = rows
            .get(i + 1)
            .and_then(|n| Some(get(b, c.rpm)? - get(n, c.rpm)?))
            .unwrap_or(0.0)
            >= 1000.0;
        if fell {
            s.limiter_or_shift += 1;
        } else {
            s.other_drops += 1;
        }
    }
    let first = rows.iter().find(|r| get(r, c.ltft1).is_some());
    let last = rows.iter().rev().find(|r| get(r, c.ltft1).is_some());
    if let (Some(f), Some(l)) = (first, last) {
        s.ltft_start = (get(f, c.ltft1), get(f, c.ltft2));
        s.ltft_end = (get(l, c.ltft1), get(l, c.ltft2));
    }
    Ok(s)
}

fn pct(v: Option<f64>) -> String {
    v.map(|x| format!("{x:+.1} %"))
        .unwrap_or_else(|| "-".into())
}

fn volts(v: Option<f64>) -> String {
    v.map(|x| format!("{x:.2} V")).unwrap_or_else(|| "-".into())
}

/// The summary in words and a table (Markdown), speeds in km/h and mph, temperatures in °C and °F.
pub fn words(name: &str, s: &Summary) -> String {
    let mut o = format!(
        "# Drive log {name}\n\n{} readings over {:.0} minutes",
        s.readings,
        s.seconds / 60.0
    );
    if let Some(k) = s.top_kmh {
        o.push_str(&format!(
            ", top speed {k:.0} km/h ({:.0} mph)",
            k * 0.621371
        ));
    }
    o.push_str(".\n");
    if let Some((lo, hi)) = s.coolant_c {
        o.push_str(&format!(
            "Coolant {lo:.0} to {hi:.0} °C ({:.0} to {:.0} °F). ",
            lo * 1.8 + 32.0,
            hi * 1.8 + 32.0
        ));
    }
    if let Some((lo, hi)) = s.volts {
        o.push_str(&format!("Voltage {lo:.2} to {hi:.2} V."));
    }
    o.push_str("\n\n## Fuel trims by driving condition (short + long term)\n\n| Condition | Readings | Bank 1 | Bank 2 | Bank 1 - bank 2 | Rear O2 bank 1 | Rear O2 bank 2 |\n|---|---:|---:|---:|---:|---:|---:|\n");
    for b in &s.bands {
        let gap = b.bank1.zip(b.bank2).map(|(a, c)| a - c);
        o.push_str(&format!(
            "| {} | {} | {} | {} | {} | {} | {} |\n",
            b.name,
            b.readings,
            pct(b.bank1),
            pct(b.bank2),
            pct(gap),
            volts(b.o2_1),
            volts(b.o2_2)
        ));
    }
    o.push_str(&format!(
        "\nLong-term trims from start to end: bank 1 {} to {}, bank 2 {} to {}.\n\nTiming drops of 4 degrees or more under load: {} at the rev limiter or a shift (RPM fell right after), {} others (mostly timing easing as the load rises, the engine's normal map; knock shows as a sudden cut at steady load, and a maker's own knock reading tells for sure).\n",
        pct(s.ltft_start.0),
        pct(s.ltft_end.0),
        pct(s.ltft_start.1),
        pct(s.ltft_end.1),
        s.limiter_or_shift,
        s.other_drops
    ));
    o
}

/// Where recordings are kept: ~/Documents/Icarus/logs and each vehicle's Logs folder.
pub fn folders() -> Vec<PathBuf> {
    let home = std::env::var_os("HOME")
        .map(PathBuf::from)
        .unwrap_or_default();
    let docs = home.join("Documents/Icarus");
    let mut v = vec![docs.join("logs")];
    if let Ok(rd) = std::fs::read_dir(docs.join("Vehicles")) {
        v.extend(rd.flatten().map(|e| e.path().join("Logs")));
    }
    v
}

/// Every CSV recording in `dirs`, newest first.
pub fn list(dirs: &[PathBuf]) -> Vec<PathBuf> {
    let mut v: Vec<(std::time::SystemTime, PathBuf)> = dirs
        .iter()
        .filter_map(|d| std::fs::read_dir(d).ok())
        .flat_map(|rd| rd.flatten())
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "csv"))
        .filter_map(|p| Some((std::fs::metadata(&p).ok()?.modified().ok()?, p)))
        .collect();
    v.sort_by_key(|b| std::cmp::Reverse(b.0));
    v.into_iter().map(|(_, p)| p).collect()
}

/// A recording by its file name, only among `dirs` (no path from outside them).
pub fn by_name(dirs: &[PathBuf], name: &str) -> Option<PathBuf> {
    if name.contains('/') || name.contains("..") {
        return None;
    }
    list(dirs)
        .into_iter()
        .find(|p| p.file_name().is_some_and(|f| f == name))
}

pub fn summarize_file(path: &Path) -> Result<String, String> {
    let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    let name = path
        .file_name()
        .map(|f| f.to_string_lossy().into_owned())
        .unwrap_or_default();
    Ok(words(&name, &summarize(&text)?))
}

#[cfg(test)]
mod tests {
    use super::*;

    const HEAD: &str = "time_s,\"Engine speed [rpm]\",\"Vehicle speed [km/h]\",\"Coolant temperature [°C]\",\"Short-term fuel trim, bank 1 [%]\",\"Long-term fuel trim, bank 1 [%]\",\"Short-term fuel trim, bank 2 [%]\",\"Long-term fuel trim, bank 2 [%]\",\"Timing advance [° before TDC]\",\"Mass air flow [g/s]\",\"Throttle position [%]\",\"Engine load [%]\",\"O2 sensor bank 1 sensor 2 [V]\",\"O2 sensor bank 2 sensor 2 [V]\",\"Control module voltage [V]\"";

    #[test]
    fn trims_by_condition_and_the_limiter() {
        let log = format!(
            "{HEAD}\n\
             0.0,650,0,90,1.0,-8.0,0.0,-4.0,15,3.0,15,16,0.78,0.74,13.6\n\
             1.0,650,0,91,-1.0,-8.0,2.0,-4.0,15,3.0,15,16,0.80,0.76,13.6\n\
             2.0,1800,60,91,0.0,-9.0,0.0,-6.0,30,10.0,20,30,0.30,0.40,13.5\n\
             3.0,5900,80,92,0.0,-2.0,0.0,-1.0,28,170,83,82,0.88,0.88,13.4\n\
             4.0,6050,85,92,0.0,-2.0,0.0,-1.0,6,175,83,82,0.88,0.88,13.4\n\
             5.0,4200,90,92,0.0,-2.0,0.0,-1.0,23,130,83,82,0.88,0.88,13.4\n"
        );
        let s = summarize(&log).unwrap();
        assert_eq!(s.readings, 6);
        let idle = s.bands.iter().find(|b| b.name == "idle").unwrap();
        assert_eq!(
            (idle.readings, idle.bank1, idle.bank2),
            (2, Some(-8.0), Some(-3.0))
        );
        assert_eq!(idle.o2_1, Some(0.79));
        assert_eq!(
            s.bands.iter().find(|b| b.name == "hard").unwrap().readings,
            3
        );
        assert_eq!(
            (s.limiter_or_shift, s.other_drops),
            (1, 0),
            "6050 -> 4200 rpm after the drop: the limiter"
        );
        assert_eq!(s.ltft_start, (Some(-8.0), Some(-4.0)));
        assert_eq!(s.top_kmh, Some(90.0));
        let w = words("drive.csv", &s);
        assert!(
            w.contains("| idle | 2 | -8.0 % | -3.0 % | -5.0 % | 0.79 V | 0.75 V |"),
            "{w}"
        );
        assert!(w.contains("56 mph"));
        assert!(
            summarize("time_s,\"Engine speed [rpm]\"\n0,700\n").is_err(),
            "no trims, no summary"
        );
    }

    #[test]
    fn a_log_is_found_by_name_only_in_its_folders() {
        assert!(by_name(&[PathBuf::from("/nonexistent")], "../../etc/passwd").is_none());
        assert!(by_name(&[PathBuf::from("/nonexistent")], "a/b.csv").is_none());
    }
}
