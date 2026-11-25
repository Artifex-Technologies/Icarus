//! Reading a recorded drive: the CSV `icarus obd live --csv` writes (a time column, then one column per value, named as
//! `"Short-term fuel trim, bank 1 [%]"`). Summarized the way a person reads fuel trims: by driving condition (idle, light,
//! moderate, hard), each bank's total trim (short + long term), the gap between the banks, the rear oxygen sensors, and the
//! sudden timing drops under load sorted into the rev limiter or a shift (RPM falls right after) and the rest. Readings in
//! fuel cut (coasting: moving, short-term trims at 0, the rear oxygen sensors near 0 V, the pedal at rest) are counted apart
//! and left out of the trims: there the trims say nothing and the rear sensors read lean, and on some engines commanded
//! lambda reads well below 1 (0.48 on a Tacoma, ICA-86), which is not running rich. Offline: only the file is read.

use std::collections::BTreeMap;
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
    pedal: Option<usize>,
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
        pedal: find(header, &["accelerator pedal"]),
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
    /// Readings in fuel cut while coasting, left out of the bands.
    pub fuel_cut: usize,
    /// Readings while the engine warms up (coolant under `WARM_C`), left out of the bands.
    pub warming: usize,
}

/// Below this coolant temperature (°C; 158 °F) the engine is still warming up: it runs richer on purpose and its trims have
/// not settled, so those readings say little about the fuelling.
pub const WARM_C: f64 = 70.0;

fn mean(v: &[f64]) -> Option<f64> {
    (!v.is_empty()).then(|| v.iter().sum::<f64>() / v.len() as f64)
}

/// A recording as columns over time, for charts: the times (seconds), and each column's name and values (None where the
/// vehicle gave none).
#[derive(Debug, Default, Clone, PartialEq)]
pub struct Series {
    pub t: Vec<f64>,
    pub names: Vec<String>,
    pub cols: Vec<Vec<Option<f64>>>,
}

impl Series {
    /// The column whose name holds every one of `words` (lowercase), e.g. ["short-term", "bank 1"].
    pub fn find(&self, words: &[&str]) -> Option<usize> {
        find(&self.names, words)
    }
}

/// A value averaged over engine speed and load, the way a tuner's table is laid out: columns of RPM (`RPM_STEP` wide),
/// rows of load (`LOAD_STEP` %), each cell the mean of `value` over the readings that fell in it, and how many.
#[derive(Clone, Debug, PartialEq)]
pub struct Grid {
    /// The lower edges of the columns (rpm) and rows (load %).
    pub rpm: Vec<f64>,
    pub load: Vec<f64>,
    /// cells[row][col]: (mean, readings).
    pub cells: Vec<Vec<Option<(f64, usize)>>>,
    pub min: f64,
    pub max: f64,
}

pub const RPM_STEP: f64 = 500.0;
pub const LOAD_STEP: f64 = 10.0;

/// `value` (a column) over engine speed and engine load; None when the recording lacks either or has no reading of it.
pub fn grid(s: &Series, value: usize) -> Option<Grid> {
    let (r, l) = (s.find(&["engine speed"])?, s.find(&["engine load"])?);
    let mut acc: BTreeMap<(usize, usize), (f64, usize)> = BTreeMap::new();
    for i in 0..s.t.len() {
        let (Some(rpm), Some(load), Some(v)) = (s.cols[r][i], s.cols[l][i], s.cols.get(value)?[i])
        else {
            continue;
        };
        if !(rpm.is_finite() && load.is_finite() && v.is_finite()) || rpm < 0.0 || load < 0.0 {
            continue;
        }
        let k = ((load / LOAD_STEP) as usize, (rpm / RPM_STEP) as usize);
        let e = acc.entry(k).or_insert((0.0, 0));
        e.0 += v;
        e.1 += 1;
    }
    let (rows, cols) = (
        acc.keys().map(|k| k.0).max()? + 1,
        acc.keys().map(|k| k.1).max()? + 1,
    );
    let mut cells = vec![vec![None; cols]; rows];
    let (mut min, mut max) = (f64::MAX, f64::MIN);
    for ((row, col), (sum, n)) in acc {
        let m = sum / n as f64;
        min = min.min(m);
        max = max.max(m);
        cells[row][col] = Some((m, n));
    }
    Some(Grid {
        rpm: (0..cols).map(|c| c as f64 * RPM_STEP).collect(),
        load: (0..rows).map(|r| r as f64 * LOAD_STEP).collect(),
        cells,
        min,
        max,
    })
}

/// A drive's GPS track (recorded with GPS), as points in metres east and north of the track's south-west corner (an
/// equirectangular projection: fine over a drive's distances), each with the GPS speed there.
pub fn track(s: &Series) -> Option<Vec<(f64, f64, Option<f64>)>> {
    let (la, lo) = (s.find(&["gps latitude"])?, s.find(&["gps longitude"])?);
    let sp = s.find(&["gps speed"]);
    let pts: Vec<(f64, f64, Option<f64>)> = (0..s.t.len())
        .filter_map(|i| {
            let (a, o) = (s.cols[la][i]?, s.cols[lo][i]?);
            (a.abs() <= 90.0 && o.abs() <= 180.0).then(|| (a, o, sp.and_then(|c| s.cols[c][i])))
        })
        .collect();
    if pts.len() < 2 {
        return None;
    }
    let lat0 = pts.iter().map(|p| p.0).fold(f64::MAX, f64::min);
    let lon0 = pts.iter().map(|p| p.1).fold(f64::MAX, f64::min);
    let mid = pts.iter().map(|p| p.0).sum::<f64>() / pts.len() as f64;
    let m_per_deg = 111_320.0;
    Some(
        pts.into_iter()
            .map(|(a, o, v)| {
                (
                    (o - lon0) * m_per_deg * mid.to_radians().cos(),
                    (a - lat0) * m_per_deg,
                    v,
                )
            })
            .collect(),
    )
}

/// A recording being written right now (the newest drive file changed in the last few seconds): its writer has the adapter
/// (a Bluetooth adapter takes one connection), so nothing else should try for it meanwhile.
pub fn recording_now() -> Option<PathBuf> {
    let newest = list(&folders()).into_iter().next()?;
    let age = std::fs::metadata(&newest)
        .and_then(|m| m.modified())
        .ok()?
        .elapsed()
        .ok()?;
    (age < std::time::Duration::from_secs(5)).then_some(newest)
}

/// A recording's first line: the adapter it was made through, as the adapter names itself (`ATI`, and an STN chip's
/// `STI`, which carries its firmware version), so a firmware update shows in the logs. Readers skip `#` lines.
pub fn adapter_line(adapter: &str) -> String {
    format!("# adapter: {}", adapter.replace(['\r', '\n'], " "))
}

/// The adapter a recording names in its first lines, when it does.
pub fn adapter_of(text: &str) -> Option<String> {
    text.lines()
        .take_while(|l| l.starts_with('#'))
        .find_map(|l| l.strip_prefix("# adapter: ").map(|a| a.trim().to_string()))
}

/// The lines of a recording after its `#` lines.
fn data_lines(text: &str) -> impl Iterator<Item = &str> {
    text.lines().skip_while(|l| l.starts_with('#'))
}

pub fn series(text: &str) -> Result<Series, String> {
    let mut lines = data_lines(text);
    let header = split(lines.next().ok_or("the file is empty")?);
    let names: Vec<String> = header
        .iter()
        .skip(1)
        .map(|h| h.trim().to_string())
        .collect();
    let mut s = Series {
        names: names.clone(),
        cols: vec![vec![]; names.len()],
        ..Default::default()
    };
    for l in lines {
        let f = split(l);
        let Some(t) = f.first().and_then(|x| x.trim().parse::<f64>().ok()) else {
            continue;
        };
        s.t.push(t);
        for (i, c) in s.cols.iter_mut().enumerate() {
            c.push(f.get(i + 1).and_then(|x| x.trim().parse::<f64>().ok()));
        }
    }
    if s.t.is_empty() {
        return Err("no readings in this log".into());
    }
    Ok(s)
}

/// A CSV header for these column names, as `icarus obd live --csv` writes it.
pub fn csv_header(names: &[String]) -> String {
    format!(
        "time_s,{}",
        names
            .iter()
            .map(|n| format!("\"{n}\""))
            .collect::<Vec<_>>()
            .join(",")
    )
}

/// A CSV row: the time, then each value (empty where there is none).
pub fn csv_row(t: f64, values: &[Option<f64>]) -> String {
    let v: Vec<String> = values
        .iter()
        .map(|x| x.map(|x| format!("{x:.2}")).unwrap_or_default())
        .collect();
    format!("{t:.2},{}", v.join(","))
}

pub fn summarize(text: &str) -> Result<Summary, String> {
    let mut lines = data_lines(text);
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
    // fuel cut while coasting: moving, the short-term trims at 0 (no correction while no fuel goes in), the rear oxygen
    // sensors near 0 V (only air passes), and the pedal at rest (its lowest reading in this log, which is not 0 on every
    // vehicle); a log without a rear sensor or a short-term trim marks none
    let pedal_rest = rows
        .iter()
        .filter_map(|r| get(r, c.pedal))
        .fold(f64::MAX, f64::min);
    let fuel_cut = |r: &Vec<Option<f64>>| {
        let near0 = |i: Option<usize>| get(r, i).map(|x| x.abs() < 0.05);
        let lean = |i: Option<usize>| get(r, i).map(|x| x < 0.15);
        get(r, c.speed).is_some_and(|k| k >= 1.6)
            && near0(c.stft1) == Some(true)
            && near0(c.stft2) != Some(false)
            && lean(c.o2_1) == Some(true)
            && lean(c.o2_2) != Some(false)
            && get(r, c.pedal).is_none_or(|p| p <= pedal_rest + 3.0)
    };
    s.fuel_cut = rows.iter().filter(|r| fuel_cut(r)).count();
    let warming = |r: &Vec<Option<f64>>| get(r, c.coolant).is_some_and(|t| t < WARM_C);
    s.warming = rows
        .iter()
        .filter(|r| warming(r) && get(r, c.rpm).is_some_and(|x| x > 400.0))
        .count();
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
                !fuel_cut(r) && !warming(r) && cond(v(c.rpm), v(c.speed), v(c.maf), v(c.throttle))
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
    if s.warming > 0 {
        o.push_str(&format!(
            "\nWarming up (coolant under {WARM_C:.0} °C, {:.0} °F): {} readings, left out of the trims above (the engine runs richer on purpose until it is warm).\n",
            WARM_C * 1.8 + 32.0,
            s.warming
        ));
    }
    if s.fuel_cut > 0 {
        o.push_str(&format!(
            "\nFuel cut while coasting: {} readings (short-term trims at 0, rear oxygen sensors near 0 V, the pedal at rest), left out of the trims above. Commanded lambda can read well below 1 there; that is not running rich.\n",
            s.fuel_cut
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
    let mut out = words(&name, &summarize(&text)?);
    if let Some(a) = adapter_of(&text) {
        out = format!("{out}\nRecorded through: {a}.");
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    const HEAD: &str = "time_s,\"Engine speed [rpm]\",\"Vehicle speed [km/h]\",\"Coolant temperature [°C]\",\"Short-term fuel trim, bank 1 [%]\",\"Long-term fuel trim, bank 1 [%]\",\"Short-term fuel trim, bank 2 [%]\",\"Long-term fuel trim, bank 2 [%]\",\"Timing advance [° before TDC]\",\"Mass air flow [g/s]\",\"Throttle position [%]\",\"Engine load [%]\",\"O2 sensor bank 1 sensor 2 [V]\",\"O2 sensor bank 2 sensor 2 [V]\",\"Control module voltage [V]\"";

    #[test]
    fn a_value_over_rpm_and_load_and_a_track() {
        let log = "time_s,\"Engine speed [rpm]\",\"Engine load [%]\",\"Timing advance [° before TDC]\",\"GPS latitude [°]\",\"GPS longitude [°]\",\"GPS speed [km/h]\"\n\
                   0,700,20,10,45.0000,-110.0000,0\n\
                   1,720,22,12,45.0000,-110.0000,0\n\
                   2,2600,55,30,45.0009,-110.0000,40\n\
                   3,2700,58,32,45.0009,-109.9987,50\n";
        let s = series(log).unwrap();
        let g = grid(&s, s.find(&["timing advance"]).unwrap()).unwrap();
        assert_eq!((g.rpm.len(), g.load.len()), (6, 6));
        assert_eq!(
            g.cells[2][1],
            Some((11.0, 2)),
            "idle: 700-720 rpm, 20-22 % load, timing 10 and 12"
        );
        assert_eq!(g.cells[5][5], Some((31.0, 2)));
        assert_eq!((g.min, g.max), (11.0, 31.0));
        assert!(
            grid(&series("time_s,\"x\"\n0,1\n").unwrap(), 0).is_none(),
            "no rpm or load: no grid"
        );
        let t = track(&s).unwrap();
        assert_eq!(t.len(), 4);
        assert_eq!((t[0].0, t[0].1), (0.0, 0.0), "the south-west corner");
        assert!(
            (t[2].1 - 100.2).abs() < 0.5,
            "0.0009° north is about 100 m: {}",
            t[2].1
        );
        assert!(
            (t[3].0 - 102.3).abs() < 1.0,
            "0.0013° east at 45° is about 102 m: {}",
            t[3].0
        );
        assert_eq!(t[3].2, Some(50.0));
        assert!(track(&series("time_s,\"x\"\n0,1\n").unwrap()).is_none());
    }

    #[test]
    fn the_adapter_line_is_kept_and_skipped() {
        let log = format!(
            "{}\n{HEAD}\n0.0,650,0,90,1.0,-8.0,0.0,-4.0,15,3.0,15,16,0.78,0.74,13.6\n",
            adapter_line("ELM327 v1.4b (STN2256 v5.10.3)")
        );
        assert_eq!(
            adapter_of(&log).as_deref(),
            Some("ELM327 v1.4b (STN2256 v5.10.3)")
        );
        assert_eq!(
            summarize(&log).unwrap().readings,
            1,
            "the summary reads past it"
        );
        assert_eq!(series(&log).unwrap().names.len(), 14);
        assert_eq!(
            adapter_of(&format!("{HEAD}\n")),
            None,
            "older recordings have none"
        );
        assert_eq!(
            adapter_line("a\r\nb"),
            "# adapter: a  b",
            "one line, whatever the adapter says"
        );
    }

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
    fn fuel_cut_is_counted_apart_and_never_read_as_rich() {
        // closed loop at cruise, then coasting in fuel cut (lambda read 0.48, trims 0, rear sensors at 0 V, pedal at rest),
        // then a stop at idle
        let head = format!("{HEAD},\"Accelerator pedal position D [%]\",\"Commanded lambda [λ]\"");
        let log = format!(
            "{head}\n\
             0.0,1500,60,90,-1.0,-4.0,-0.5,-2.0,30,10.0,20,25,0.72,0.74,13.6,25.0,1.00\n\
             0.5,1500,60,90,1.0,-4.0,0.5,-2.0,30,10.0,20,25,0.70,0.72,13.6,25.0,1.00\n\
             1.0,1400,55,90,0.0,-4.0,0.0,-2.0,25,3.0,14,10,0.01,0.01,13.6,15.7,0.48\n\
             1.5,1300,50,90,0.0,-4.0,0.0,-2.0,25,3.0,14,10,0.00,0.01,13.6,15.7,0.48\n\
             2.0,700,0,90,1.0,-4.0,0.0,-2.0,15,3.0,14,15,0.75,0.75,13.6,15.7,1.00\n\
             2.5,900,0,20,0.0,-4.0,0.0,-2.0,10,4.0,15,20,0.20,0.20,13.6,15.7,0.85\n"
        );
        let s = summarize(&log).unwrap();
        assert_eq!(s.fuel_cut, 2);
        // the cold start's reading is not idle's: idle keeps its warm reading alone
        assert_eq!(s.warming, 1);
        let idle = s.bands.iter().find(|b| b.name == "idle").unwrap();
        assert_eq!((idle.readings, idle.o2_1), (1, Some(0.75)));
        let cruise = s.bands.iter().find(|b| b.name == "cruise / light").unwrap();
        // the two closed-loop readings only: the trims and the rear sensors are not dragged toward 0
        assert_eq!(
            (cruise.readings, cruise.bank1, cruise.o2_1),
            (2, Some(-4.0), Some(0.71))
        );
        let w = words("drive.csv", &s);
        assert!(
            w.contains("Fuel cut while coasting: 2 readings") && w.contains("not running rich"),
            "{w}"
        );
        assert!(
            w.contains("Warming up (coolant under 70 °C, 158 °F): 1 readings"),
            "{w}"
        );
        // the same readings with the pedal pressed are not fuel cut (a lean moment under way is not coasting)
        let pressed = log.replace(",15.7,0.48", ",40.0,0.48");
        assert_eq!(summarize(&pressed).unwrap().fuel_cut, 0);
    }

    #[test]
    fn a_recording_as_columns_and_back() {
        let names = vec![
            "Engine speed [rpm]".to_string(),
            "Short-term fuel trim, bank 1 [%]".to_string(),
        ];
        let text = format!(
            "{}\n{}\n{}\n",
            csv_header(&names),
            csv_row(0.5, &[Some(650.0), Some(-1.56)]),
            csv_row(1.0, &[Some(660.0), None])
        );
        let s = series(&text).unwrap();
        assert_eq!(s.t, [0.5, 1.0]);
        assert_eq!(s.names, names);
        assert_eq!(s.cols[1], [Some(-1.56), None]);
        assert_eq!(s.find(&["short-term", "bank 1"]), Some(1));
        assert!(series("time_s,\"x\"\n").is_err());
    }

    #[test]
    fn a_log_is_found_by_name_only_in_its_folders() {
        assert!(by_name(&[PathBuf::from("/nonexistent")], "../../etc/passwd").is_none());
        assert!(by_name(&[PathBuf::from("/nonexistent")], "a/b.csv").is_none());
    }
}
