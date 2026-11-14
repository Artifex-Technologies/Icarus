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

/// Recording the live values to a CSV, one row per sweep of the chosen values.
pub struct Recorder {
    pub path: PathBuf,
    file: std::fs::File,
    pids: Vec<u8>,
    row: HashMap<u8, f64>,
    t0: Option<f64>,
    pub rows: usize,
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
    /// Starts a recording of `pids` in ~/Documents/Icarus/logs.
    pub fn start(pids: &[u8]) -> Result<Recorder, String> {
        let dir = drivelog::folders().into_iter().next().ok_or("no home folder")?;
        std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
        let path = dir.join(file_name());
        let mut file = std::fs::File::create(&path).map_err(|e| format!("{}: {e}", path.display()))?;
        let names: Vec<String> =
            pids.iter().filter_map(|p| j1979::pid(*p)).map(|d| format!("{} [{}]", d.name, d.unit)).collect();
        writeln!(file, "{}", drivelog::csv_header(&names)).map_err(|e| e.to_string())?;
        Ok(Recorder { path, file, pids: pids.iter().copied().filter(|p| j1979::pid(*p).is_some()).collect(), row: HashMap::new(), t0: None, rows: 0 })
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
}

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

/// The charts, top to bottom: (title, the columns by the words in their names, a zero line).
const CHARTS: &[(&str, &[&[&str]], bool)] = &[
    (
        "Fuel trims (%)",
        &[&["short-term", "bank 1"], &["long-term", "bank 1"], &["short-term", "bank 2"], &["long-term", "bank 2"]],
        true,
    ),
    ("Engine speed (rpm)", &[&["engine speed"]], false),
    ("Speed (km/h)", &[&["vehicle speed"]], false),
    ("Throttle (%)", &[&["throttle position"]], false),
    ("Rear oxygen sensors (V)", &[&["bank 1 sensor 2"], &["bank 2 sensor 2"]], false),
    ("Timing (° before TDC)", &[&["timing advance"]], false),
    ("Air flow (g/s)", &[&["mass air flow"]], false),
    ("Coolant (°C)", &[&["coolant"]], false),
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
        ui.horizontal_wrapped(|ui| {
            ui.label(RichText::new(*title).strong());
            for (k, &i) in idx.iter().enumerate() {
                let short = s.names[i].split(" [").next().unwrap_or("").replace("fuel trim, ", "");
                let at = hover.and_then(|h| value_at(s, i, h)).map(|v| format!(" {v:.2}")).unwrap_or_default();
                ui.label(RichText::new(format!("● {short}{at}")).color(colors(pal)[k % 4]).small());
            }
        });
        let (rect, resp) = ui.allocate_exact_size(vec2(ui.available_width(), 120.0), Sense::hover());
        let p = ui.painter_at(rect);
        p.rect_filled(rect, CornerRadius::same(4), pal.field);
        let vals = idx.iter().flat_map(|&i| s.cols[i].iter().flatten().copied());
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
            // one line per stretch of values (a gap where the vehicle gave none)
            let mut pts: Vec<egui::Pos2> = vec![];
            for (t, v) in s.t.iter().zip(&s.cols[i]) {
                match v {
                    Some(v) => pts.push(egui::pos2(x(*t), y(*v))),
                    None => {
                        if pts.len() > 1 {
                            p.add(egui::Shape::line(std::mem::take(&mut pts), Stroke::new(1.4, colors(pal)[k % 4])));
                        }
                        pts.clear();
                    }
                }
            }
            if pts.len() > 1 {
                p.add(egui::Shape::line(pts, Stroke::new(1.4, colors(pal)[k % 4])));
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
        let mut r = Recorder { path: path.clone(), file, pids: vec![0x0C, 0x06], row: HashMap::new(), t0: None, rows: 0 };
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
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
