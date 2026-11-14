//! The Logs page and recording: drives recorded as CSV (by the window's Record button on the Live data page, or by `icarus obd
//! live --csv`), listed, summarized (icarus-obd's drivelog, the same summary assistants read through `icarus mcp`'s drive_logs)
//! and drawn: stacked charts on one time line (fuel trims of both banks, engine speed and road speed, throttle, the rear
//! oxygen sensors, timing, air flow, coolant), with a line under the pointer that reads every chart at that moment.

use eframe::egui::{self, vec2, Color32, CornerRadius, RichText, Sense, Stroke};
use icarus_obd::drivelog::{self, Series, Summary};
use icarus_obd::j1979;
use std::collections::HashMap;
use std::io::Write;
use std::path::PathBuf;
use yggdrasil_ui::theme;

/// The values read every sweep while recording a drive, when the vehicle offers them: what fuel trims, timing, air flow,
/// the throttle and the mixture are read from. Engine speed comes first: a row is written each time it comes round.
pub const DRIVE_FAST: &[u8] = &[
    0x0C, 0x0D, 0x04, 0x06, 0x07, 0x08, 0x09, 0x0E, 0x10, 0x11, 0x49, 0x15, 0x19, 0x44, 0x24, 0x28, 0x34, 0x38,
];

/// A drive's values from what the vehicle offers: (read every sweep, the rest read in turn).
pub fn drive_set(offered: &[u8]) -> (Vec<u8>, Vec<u8>) {
    let fast: Vec<u8> = DRIVE_FAST.iter().copied().filter(|p| offered.contains(p)).collect();
    let rest = offered.iter().copied().filter(|p| !fast.contains(p)).collect();
    (fast, rest)
}

/// Whether miles per hour, °F and psi are the custom here (the locale's measurement: en_US and the few like it).
pub fn us_units() -> bool {
    static US: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *US.get_or_init(|| {
        let loc = ["LC_ALL", "LC_MEASUREMENT", "LANG"]
            .iter()
            .filter_map(|k| std::env::var(k).ok())
            .find(|v| !v.is_empty())
            .unwrap_or_default();
        ["en_US", "es_US", "en_LR", "my_MM"].iter().any(|p| loc.starts_with(p))
    })
}

/// A value as shown: in the customary units where those are used (us_units), else as the vehicle gives it.
pub fn shown(unit: &str, v: f64) -> (f64, &str) {
    if us_units() {
        match unit {
            "km/h" => return (v * 0.621371, "mph"),
            "km" => return (v * 0.621371, "mi"),
            "°C" => return (v * 1.8 + 32.0, "°F"),
            "kPa" => return (v * 0.145038, "psi"),
            "L/h" => return (v * 0.264172, "gal/h"),
            "Nm" => return (v * 0.737562, "lb-ft"),
            _ => {}
        }
    }
    (v, unit)
}

/// The unit in a recording's column name (`"Vehicle speed [km/h]"`).
fn unit_of(name: &str) -> &str {
    name.rsplit_once(" [").map(|(_, u)| u.trim_end_matches(']')).unwrap_or("")
}

/// Recording the live values to a CSV, one row per sweep.
pub struct Recorder {
    pub path: PathBuf,
    file: std::fs::File,
    /// Read every sweep, and the rest read in turn (two a sweep).
    pub fast: Vec<u8>,
    pub rest: Vec<u8>,
    pids: Vec<u8>,
    row: HashMap<u8, f64>,
    t0: Option<f64>,
    pub rows: usize,
    /// `systemd-inhibit ... cat` reading from a pipe of ours: the laptop doesn't sleep (closing the lid included) while a
    /// drive is recorded, until the recording stops or the window ends however it ends (the pipe closes, cat and the
    /// inhibitor end with it).
    awake: Option<std::process::Child>,
}

impl Drop for Recorder {
    fn drop(&mut self) {
        if let Some(mut c) = self.awake.take() {
            drop(c.stdin.take());
            let _ = c.kill();
            let _ = c.wait();
        }
    }
}

/// `drive-YYYY-MM-DD_HHMMSS.csv` from the local time.
fn file_name() -> String {
    let stamp = std::process::Command::new("date")
        .arg("+%Y-%m-%d_%H%M%S")
        .output()
        .ok()
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| "now".into());
    format!("drive-{stamp}.csv")
}

impl Recorder {
    /// Starts a recording in ~/Documents/Icarus/logs: `fast` read every sweep, `rest` in turn.
    pub fn start(fast: &[u8], rest: &[u8]) -> Result<Recorder, String> {
        let pids: Vec<u8> = fast.iter().chain(rest).copied().filter(|p| j1979::pid(*p).is_some()).collect();
        let dir = drivelog::folders().into_iter().next().ok_or("no home folder")?;
        std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
        let path = dir.join(file_name());
        let mut file = std::fs::File::create(&path).map_err(|e| format!("{}: {e}", path.display()))?;
        let names: Vec<String> =
            pids.iter().filter_map(|p| j1979::pid(*p)).map(|d| format!("{} [{}]", d.name, d.unit)).collect();
        writeln!(file, "{}", drivelog::csv_header(&names)).map_err(|e| e.to_string())?;
        let awake = std::process::Command::new("systemd-inhibit")
            .args(["--what=sleep:idle:handle-lid-switch", "--who=Icarus", "--why=Recording a drive", "cat"])
            .stdin(std::process::Stdio::piped())
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null())
            .spawn()
            .ok();
        Ok(Recorder {
            path,
            file,
            fast: fast.to_vec(),
            rest: rest.to_vec(),
            pids,
            row: HashMap::new(),
            t0: None,
            rows: 0,
            awake,
        })
    }

    /// One reading: when the first value of a sweep comes round again, the sweep before it becomes a row.
    pub fn add(&mut self, pid: u8, value: f64, at: f64) {
        if !self.pids.contains(&pid) {
            return;
        }
        let t0 = *self.t0.get_or_insert(at);
        if pid == self.pids[0] && !self.row.is_empty() {
            let values: Vec<Option<f64>> = self.pids.iter().map(|p| self.row.get(p).copied()).collect();
            let _ = writeln!(self.file, "{}", drivelog::csv_row(at - t0, &values));
            self.rows += 1;
            self.row.clear();
        }
        self.row.insert(pid, value);
    }
}

/// What the page holds: the recordings, the one chosen, and it read.
#[derive(Default)]
pub struct Logs {
    pub list: Vec<PathBuf>,
    pub chosen: Option<PathBuf>,
    loaded: Option<(Result<Summary, String>, Result<Series, String>)>,
    /// The time under the pointer, across the charts.
    hover: Option<f64>,
    /// When the chosen file was last changed (as read), and when that was last looked at: a recording still being written
    /// (the background `icarus obd live --csv`, or Record here) is read again every couple of seconds.
    stamp: Option<std::time::SystemTime>,
    checked: Option<std::time::Instant>,
}

fn changed(p: &std::path::Path) -> Option<std::time::SystemTime> {
    std::fs::metadata(p).and_then(|m| m.modified()).ok()
}

/// A recording being written right now (`icarus obd live --csv` in the background, say): it has the adapter, so the window
/// doesn't try for it meanwhile.
pub use icarus_obd::drivelog::recording_now;

impl Logs {
    pub fn refresh(&mut self) {
        self.list = drivelog::list(&drivelog::folders());
        if self.chosen.is_none() {
            self.chosen = self.list.first().cloned();
            self.loaded = None;
        }
    }

    fn load(&mut self) {
        if self.loaded.is_some() {
            return;
        }
        if let Some(p) = &self.chosen {
            self.stamp = changed(p);
            let text = std::fs::read_to_string(p).map_err(|e| e.to_string());
            self.loaded = Some(match text {
                Ok(t) => (drivelog::summarize(&t), drivelog::series(&t)),
                Err(e) => (Err(e.clone()), Err(e)),
            });
        }
    }

    pub fn page(&mut self, ui: &mut egui::Ui, pal: &theme::Palette) {
        if self.list.is_empty() {
            self.refresh();
        }
        // a recording still being written: read again every couple of seconds
        let tick = std::time::Duration::from_secs(2);
        if self.checked.is_none_or(|c| c.elapsed() >= tick) {
            self.checked = Some(std::time::Instant::now());
            let newest_before = self.list.first().cloned();
            self.list = drivelog::list(&drivelog::folders());
            // following the newest recording when it was the one shown and a newer one started
            if self.chosen.is_none() || (self.chosen == newest_before && self.list.first() != newest_before.as_ref()) {
                self.chosen = self.list.first().cloned();
                self.loaded = None;
            }
            if let Some(p) = &self.chosen {
                if changed(p) != self.stamp {
                    self.loaded = None;
                }
            }
        }
        let live = self.chosen.as_ref().and_then(|p| changed(p)).and_then(|t| t.elapsed().ok()).is_some_and(|a| a.as_secs() < 5);
        if live {
            ui.ctx().request_repaint_after(tick);
        }
        ui.horizontal(|ui| {
            ui.label("Recording");
            let name = |p: &PathBuf| p.file_name().map(|f| f.to_string_lossy().into_owned()).unwrap_or_default();
            let mut pick = None;
            egui::ComboBox::from_id_salt("log-pick")
                .width(320.0)
                .selected_text(self.chosen.as_ref().map(name).unwrap_or_else(|| "none yet".into()))
                .show_ui(ui, |ui| {
                    for p in &self.list {
                        if ui.selectable_label(Some(p) == self.chosen.as_ref(), name(p)).clicked() {
                            pick = Some(p.clone());
                        }
                    }
                });
            if let Some(p) = pick {
                self.chosen = Some(p);
                self.loaded = None;
                self.hover = None;
            }
            if ui.button("Refresh").clicked() {
                self.refresh();
                self.loaded = None;
            }
            if live {
                ui.label(RichText::new("● recording now, updating every 2 s").color(pal.red));
            }
        });
        if self.list.is_empty() {
            ui.label(RichText::new("No recordings yet: Live data > Record, or icarus obd live ... --csv FILE.").color(pal.dim));
            return;
        }
        self.load();
        let Some((summary, series)) = &self.loaded else { return };
        egui::ScrollArea::vertical().id_salt("logs").show(ui, |ui| {
            egui::Frame::new().inner_margin(egui::Margin { left: 0, right: 18, top: 4, bottom: 8 }).show(ui, |ui| {
                match summary {
                    Ok(s) => summary_ui(ui, pal, s),
                    Err(e) => {
                        ui.label(RichText::new(format!("No summary: {e}")).color(pal.dim));
                    }
                }
                ui.add_space(10.0);
                match series {
                    Ok(s) => charts(ui, pal, s, &mut self.hover),
                    Err(e) => {
                        ui.label(RichText::new(format!("Nothing to draw: {e}")).color(pal.red));
                    }
                }
            });
        });
    }
}

fn summary_ui(ui: &mut egui::Ui, pal: &theme::Palette, s: &Summary) {
    let mut line = format!("{} readings over {:.0} minutes", s.readings, s.seconds / 60.0);
    if let Some(k) = s.top_kmh {
        line.push_str(&format!(", top {k:.0} km/h ({:.0} mph)", k * 0.621371));
    }
    if let Some((lo, hi)) = s.coolant_c {
        line.push_str(&format!(", coolant {lo:.0}-{hi:.0} °C ({:.0}-{:.0} °F)", lo * 1.8 + 32.0, hi * 1.8 + 32.0));
    }
    if let Some((lo, hi)) = s.volts {
        line.push_str(&format!(", {lo:.2}-{hi:.2} V"));
    }
    ui.label(RichText::new(line).color(pal.dim));
    ui.add_space(6.0);
    let pct = |v: Option<f64>| v.map(|x| format!("{x:+.1} %")).unwrap_or_else(|| "-".into());
    let v = |v: Option<f64>| v.map(|x| format!("{x:.2} V")).unwrap_or_else(|| "-".into());
    egui::Grid::new("log-bands").striped(true).spacing([18.0, 6.0]).show(ui, |ui| {
        for h in ["Fuel trims", "Readings", "Bank 1", "Bank 2", "Bank 1 - bank 2", "Rear O2 bank 1", "Rear O2 bank 2"] {
            ui.label(RichText::new(h).strong());
        }
        ui.end_row();
        for b in &s.bands {
            ui.label(b.name);
            ui.label(b.readings.to_string());
            ui.label(pct(b.bank1));
            ui.label(pct(b.bank2));
            ui.label(pct(b.bank1.zip(b.bank2).map(|(a, c)| a - c)));
            ui.label(v(b.o2_1));
            ui.label(v(b.o2_2));
            ui.end_row();
        }
    });
    ui.label(
        RichText::new(format!(
            "Long-term trims from start to end: bank 1 {} to {}, bank 2 {} to {}. Timing drops under load: {} at the rev limiter or a shift, {} others.",
            pct(s.ltft_start.0),
            pct(s.ltft_end.0),
            pct(s.ltft_start.1),
            pct(s.ltft_end.1),
            s.limiter_or_shift,
            s.other_drops
        ))
        .small()
        .color(pal.dim),
    );
}

/// The colours of the lines in one chart, in order.
fn colors(pal: &theme::Palette) -> [Color32; 4] {
    [pal.accent, Color32::from_rgb(236, 152, 64), pal.green, Color32::from_rgb(90, 160, 236)]
}

/// The charts, top to bottom: (title, the columns by the words in their names, a zero line). A chart shows only the
/// columns a recording has; one with none of them is left out.
const CHARTS: &[(&str, &[&[&str]], bool)] = &[
    (
        "Fuel trims",
        &[&["short-term", "bank 1"], &["long-term", "bank 1"], &["short-term", "bank 2"], &["long-term", "bank 2"]],
        true,
    ),
    (
        "Air/fuel: aimed for, and measured by each bank's front sensor",
        &[&["commanded lambda"], &["wide-band", "bank 1"], &["wide-band", "bank 2"]],
        false,
    ),
    ("Engine speed", &[&["engine speed"]], false),
    ("Speed", &[&["vehicle speed"]], false),
    ("Pedal and throttle", &[&["accelerator pedal"], &["throttle position"], &["commanded throttle"]], false),
    ("Load", &[&["engine load"], &["absolute load"]], false),
    ("Timing", &[&["timing advance"]], false),
    ("Air flow", &[&["mass air flow"]], false),
    ("Rear oxygen sensors", &[&["o2 sensor", "bank 1 sensor 2"], &["o2 sensor", "bank 2 sensor 2"]], false),
    ("Temperatures", &[&["coolant"], &["intake air"], &["ambient"], &["oil temperature"]], false),
    ("Catalysts", &[&["catalyst", "bank 1 sensor 1"], &["catalyst", "bank 2 sensor 1"]], false),
    ("Fuel rail pressure", &[&["fuel rail"]], false),
    ("Engine torque", &[&["demand torque"], &["actual engine torque"]], false),
    ("Fuel used", &[&["fuel rate"]], false),
    ("Battery and charging", &[&["module voltage"]], false),
    ("Fuel level", &[&["fuel level"]], false),
];

fn charts(ui: &mut egui::Ui, pal: &theme::Palette, s: &Series, hover: &mut Option<f64>) {
    let (t0, t1) = (s.t[0], *s.t.last().unwrap_or(&s.t[0]));
    let mut new_hover = None;
    for (title, wanted, zero) in CHARTS {
        let idx: Vec<usize> = wanted.iter().filter_map(|w| s.find(w)).collect();
        if idx.is_empty() {
            continue;
        }
        // the title, and each line's name and its value under the pointer
        let unit = unit_of(&s.names[idx[0]]);
        let conv = |v: f64| shown(unit, v).0;
        ui.horizontal_wrapped(|ui| {
            ui.label(RichText::new(format!("{title} ({})", shown(unit, 0.0).1)).strong());
            for (k, &i) in idx.iter().enumerate() {
                let short = s.names[i].split(" [").next().unwrap_or("").replace("fuel trim, ", "");
                let at = hover.and_then(|h| value_at(s, i, h)).map(|v| format!(" {:.2}", conv(v))).unwrap_or_default();
                ui.label(RichText::new(format!("● {short}{at}")).color(colors(pal)[k % 4]).small());
            }
        });
        let (rect, resp) = ui.allocate_exact_size(vec2(ui.available_width(), 120.0), Sense::hover());
        let p = ui.painter_at(rect);
        p.rect_filled(rect, CornerRadius::same(4), pal.field);
        let vals = idx.iter().flat_map(|&i| s.cols[i].iter().flatten().map(|v| conv(*v)));
        let (mut lo, mut hi) = vals.fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), v| (a.min(v), b.max(v)));
        if *zero {
            lo = lo.min(0.0);
            hi = hi.max(0.0);
        }
        if !lo.is_finite() {
            continue;
        }
        let span = if hi > lo { hi - lo } else { 1.0 };
        let x = |t: f64| rect.left() + ((t - t0) / (t1 - t0).max(1e-9)) as f32 * rect.width();
        let y = |v: f64| rect.bottom() - 6.0 - ((v - lo) / span) as f32 * (rect.height() - 12.0);
        if *zero && lo < 0.0 && hi > 0.0 {
            p.line_segment([egui::pos2(rect.left(), y(0.0)), egui::pos2(rect.right(), y(0.0))], Stroke::new(1.0, pal.border));
        }
        for (k, &i) in idx.iter().enumerate() {
            // one line through the values read (a value read in turn is there every few seconds), broken where none came for
            // a while (the adapter dropped, the engine was off)
            let stroke = Stroke::new(1.4, colors(pal)[k % 4]);
            let mut pts: Vec<egui::Pos2> = vec![];
            let mut last = f64::NEG_INFINITY;
            for (t, v) in s.t.iter().zip(&s.cols[i]) {
                let Some(v) = v else { continue };
                if t - last > 30.0 {
                    if pts.len() > 1 {
                        p.add(egui::Shape::line(std::mem::take(&mut pts), stroke));
                    }
                    pts.clear();
                }
                pts.push(egui::pos2(x(*t), y(conv(*v))));
                last = *t;
            }
            match pts.len() {
                0 => {}
                1 => {
                    p.circle_filled(pts[0], 2.0, stroke.color);
                }
                _ => {
                    p.add(egui::Shape::line(pts, stroke));
                }
            }
        }
        p.text(rect.left_top() + vec2(4.0, 2.0), egui::Align2::LEFT_TOP, format!("{hi:.1}"), egui::FontId::proportional(10.0), pal.dim);
        p.text(rect.left_bottom() + vec2(4.0, -2.0), egui::Align2::LEFT_BOTTOM, format!("{lo:.1}"), egui::FontId::proportional(10.0), pal.dim);
        if let Some(pos) = resp.hover_pos() {
            new_hover = Some(t0 + ((pos.x - rect.left()) / rect.width()) as f64 * (t1 - t0));
        }
        if let Some(h) = *hover {
            let hx = x(h);
            p.line_segment([egui::pos2(hx, rect.top()), egui::pos2(hx, rect.bottom())], Stroke::new(1.0, pal.dim));
        }
        ui.add_space(8.0);
    }
    let minutes = (t1 - t0) / 60.0;
    ui.label(
        RichText::new(match hover {
            Some(h) => format!("{:.1} of {minutes:.1} minutes", (*h - t0) / 60.0),
            None => format!("{minutes:.1} minutes: point at a chart to read every chart at that moment"),
        })
        .small()
        .color(pal.dim),
    );
    *hover = new_hover.or(*hover).filter(|_| new_hover.is_some() || ui.ui_contains_pointer());
}

/// The value of column `i` at the reading nearest to time `t`.
fn value_at(s: &Series, i: usize, t: f64) -> Option<f64> {
    let k = s.t.partition_point(|x| *x < t).min(s.t.len().saturating_sub(1));
    s.cols[i].get(k).copied().flatten()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_recording_writes_one_row_per_sweep() {
        let dir = std::env::temp_dir().join(format!("icarus-rec-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("r.csv");
        let file = std::fs::File::create(&path).unwrap();
        let mut r = Recorder {
            path: path.clone(),
            file,
            fast: vec![0x0C, 0x06],
            rest: vec![],
            pids: vec![0x0C, 0x06],
            row: HashMap::new(),
            t0: None,
            rows: 0,
            awake: None,
        };
        r.add(0x0C, 650.0, 10.0);
        r.add(0x06, -1.5, 10.2);
        r.add(0x0C, 660.0, 10.5);
        r.add(0x06, 0.8, 10.7);
        r.add(0x0C, 670.0, 11.0);
        drop(r);
        let text = std::fs::read_to_string(&path).unwrap();
        assert_eq!(text, "0.50,650.00,-1.50\n1.00,660.00,0.80\n");
        let s = Series { t: vec![0.0, 1.0, 2.0], names: vec!["a".into()], cols: vec![vec![Some(1.0), None, Some(3.0)]] };
        assert_eq!(value_at(&s, 0, 1.9), Some(3.0));
        assert_eq!(value_at(&s, 0, 0.4), None);
        let (fast, rest) = drive_set(&[0x05, 0x06, 0x0C, 0x2F, 0x34]);
        assert_eq!((fast, rest), (vec![0x0C, 0x06, 0x34], vec![0x05, 0x2F]), "engine speed first, the rest in turn");
        assert_eq!(unit_of("Vehicle speed [km/h]"), "km/h");
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
