//! The Garage page's Maintenance card (ICA-185, ICA-215): the vehicle's maintenance log (icarus-obd's `maintenance`, the
//! same `maintenance.toml` as `icarus garage service`), a service added with its date and odometer, the intervals set per
//! kind, what is due by miles or months, and for a service the drives recorded before it against the drives after it
//! ("Before and after", the Logs page's Compare table under the service's name). The odometer is typed in; a reading the
//! vehicle gave (standard value A6 or the module-data set's odometer, live, or the newest recording's) is offered with a
//! button, never filled in by itself. What reads recordings (the newest one's odometer, a before and after, which of the
//! day's drives fall after a service) runs on a thread of its own, its answer coming back to the card, so the window
//! never waits on the files. Self-contained: the page hands it the vehicle shown and any live reading.

use eframe::egui::{self, vec2, RichText};
use icarus_obd::drivelog::{self, compare};
use icarus_obd::maintenance::{self as mt, Kind, Offer, Service, When};
use std::path::{Path, PathBuf};
use std::sync::mpsc::{channel, Receiver, Sender};
use yggdrasil_ui::theme;

/// What a background job of the card answers, with the vehicle folder it was asked for.
enum Answer {
    Recorded(PathBuf, Option<Offer>),
    Compared(PathBuf, String, Box<Result<mt::BeforeAfter, String>>),
    DayNote(PathBuf, Option<String>),
}

/// The card's state: what is typed, and what was read for the vehicle shown.
pub struct Card {
    /// The vehicle folder the rest was read for: another vehicle shown reads again.
    dir: Option<PathBuf>,
    log: Option<Result<mt::Log, String>>,
    /// The newest recording's odometer: None until looked for (once per vehicle), Some(None) when none has one.
    recorded: Option<Option<Offer>>,
    looking: bool,
    kind: Option<Kind>,
    miles: String,
    date: String,
    note: String,
    /// The interval being typed, and the kind it was filled in for (filled again only when the kind or vehicle changes,
    /// so emptying both fields and Set clears the interval).
    every_miles: String,
    every_months: String,
    every_for: Option<Kind>,
    /// The odometer now, for what is due (blank: the highest one logged).
    now_miles: String,
    drives: Option<usize>,
    /// Before and after: the service it was asked for (its date and kind), whether it is being worked out, and its answer.
    asked: Option<String>,
    compared: Option<(String, Result<mt::BeforeAfter, String>)>,
    said: Option<Result<String, String>>,
    day_note: Option<String>,
    tx: Sender<Answer>,
    rx: Receiver<Answer>,
}

impl Default for Card {
    fn default() -> Card {
        let (tx, rx) = channel();
        Card {
            dir: None,
            log: None,
            recorded: None,
            looking: false,
            kind: None,
            miles: String::new(),
            date: String::new(),
            note: String::new(),
            every_miles: String::new(),
            every_months: String::new(),
            every_for: None,
            now_miles: String::new(),
            drives: None,
            asked: None,
            compared: None,
            said: None,
            day_note: None,
            tx,
            rx,
        }
    }
}

/// An odometer reading the window holds for the connected vehicle: standard value A6 (km) as last read on Live data, else
/// a module-data value named odometer as last read, from its number and unit (km or mi), not its text.
pub fn live_offer(
    a6_km: Option<f64>,
    module: Option<&icarus_obd::vehicledata::VehicleValues>,
) -> Option<Offer> {
    if let Some(m) = a6_km.and_then(|v| mt::miles_of(v, "km")) {
        return Some(Offer {
            miles: m,
            from: "the vehicle's odometer (standard value A6), read live".into(),
            doubt: None,
        });
    }
    mt::offers_in(module?, ", read on the Modules page")
        .0
        .into_iter()
        .next()
}

fn number(s: &str) -> Result<Option<u32>, String> {
    let t = s.trim().replace(',', "");
    if t.is_empty() {
        return Ok(None);
    }
    t.parse::<u32>()
        .map(Some)
        .map_err(|_| format!("{s:?} is not a whole number"))
}

fn recordings(dir: &Path) -> mt::Recordings {
    let shared = drivelog::folders().into_iter().next().unwrap_or_default();
    mt::recordings_for(&icarus_obd::garage::library(), dir, &shared)
}

impl Card {
    fn read(&mut self, dir: &Path) {
        if self.dir.as_deref() != Some(dir) {
            let (kind, drives) = (self.kind, self.drives);
            *self = Card::default();
            self.dir = Some(dir.to_path_buf());
            self.kind = kind;
            self.drives = drives;
        }
        if self.log.is_none() {
            self.log = Some(mt::load(dir));
        }
    }

    /// Runs `job` on a thread of its own; its answer comes back through the card's channel, the window drawn again.
    fn spawn(&self, ctx: &egui::Context, job: impl FnOnce() -> Answer + Send + 'static) {
        let (tx, ctx) = (self.tx.clone(), ctx.clone());
        std::thread::spawn(move || {
            let _ = tx.send(job());
            ctx.request_repaint();
        });
    }

    /// The answers come back, those for another vehicle than the one shown dropped.
    fn take_answers(&mut self) {
        while let Ok(a) = self.rx.try_recv() {
            let here = |d: &PathBuf| self.dir.as_ref() == Some(d);
            match a {
                Answer::Recorded(d, o) if here(&d) => {
                    self.recorded = Some(o);
                    self.looking = false;
                }
                Answer::Compared(d, key, r) if here(&d) && self.asked.as_ref() == Some(&key) => {
                    self.asked = None;
                    self.compared = Some((key, *r));
                }
                Answer::DayNote(d, n) if here(&d) => self.day_note = n,
                _ => {}
            }
        }
    }

    /// The card for the vehicle with this folder; `live` is a reading the window holds for it, when connected to it.
    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        pal: &theme::Palette,
        dir: &Path,
        live: Option<Offer>,
    ) {
        self.read(dir);
        self.take_answers();
        let log = match self.log.as_ref().expect("just read") {
            Ok(l) => l.clone(),
            Err(e) => {
                super::card(ui, pal, "Maintenance", |ui| {
                    ui.label(RichText::new(format!("The log does not read: {e}")).color(pal.red));
                    ui.label(
                        RichText::new("Correct it by hand (Open the folder), then Refresh.")
                            .small()
                            .color(pal.dim),
                    );
                });
                return;
            }
        };
        let mut reread = false;
        super::card(
            ui,
            pal,
            &format!("Maintenance ({})", log.services.len()),
            |ui| {
                ui.label(
                    RichText::new(format!(
                        "Kept in {} in this vehicle's folder, a plain file you can edit by hand.",
                        mt::FILE
                    ))
                    .small()
                    .color(pal.dim),
                );
                self.due_part(ui, pal, &log);
                ui.add_space(6.0);
                reread |= self.add_part(ui, pal, dir, &log, live);
                ui.add_space(6.0);
                self.list_part(ui, pal, dir, &log);
                if let Some(s) = &self.said {
                    match s {
                        Ok(t) => ui.label(RichText::new(t).color(pal.green)),
                        Err(e) => ui.label(RichText::new(e).color(pal.red)),
                    };
                }
                if let Some(n) = &self.day_note {
                    ui.label(RichText::new(n).small().color(pal.dim));
                }
            },
        );
        if reread {
            self.log = None;
        }
    }

    /// What is due, by the odometer typed (else the highest logged) and today's date.
    fn due_part(&mut self, ui: &mut egui::Ui, pal: &theme::Palette, log: &mt::Log) {
        ui.horizontal(|ui| {
            ui.label(RichText::new("Odometer now").weak());
            ui.add(
                egui::TextEdit::singleline(&mut self.now_miles)
                    .hint_text("miles (blank: the last logged)")
                    .desired_width(190.0),
            );
        });
        let now = number(&self.now_miles)
            .ok()
            .flatten()
            .or(log.highest_miles().map(|x| x.0));
        let due = mt::due(log, When::now().day, now);
        if due.is_empty() {
            ui.label(
                RichText::new(
                    "No intervals set yet: pick a kind below and set its miles or months.",
                )
                .color(pal.dim),
            );
        }
        for d in &due {
            let color = match d.state {
                mt::State::Overdue => pal.red,
                mt::State::Soon => pal.amber,
                mt::State::Later => pal.green,
                _ => pal.dim,
            };
            ui.label(RichText::new(d.text()).color(color));
        }
    }

    /// Adding a service, and the picked kind's interval. True when the log changed.
    fn add_part(
        &mut self,
        ui: &mut egui::Ui,
        pal: &theme::Palette,
        dir: &Path,
        log: &mt::Log,
        live: Option<Offer>,
    ) -> bool {
        let mut changed = false;
        let kind = *self.kind.get_or_insert(Kind::Oil);
        // now, with the time: a test drive right after the service then counts as after it
        if self.date.is_empty() {
            self.date = When::now().text();
        }
        egui::Grid::new(("maintenance-add", dir))
            .num_columns(2)
            .spacing(vec2(12.0, 6.0))
            .show(ui, |ui| {
                ui.label(RichText::new("Service").weak());
                egui::ComboBox::from_id_salt(("maintenance-kind", dir))
                    .selected_text(kind.name())
                    .show_ui(ui, |ui| {
                        for k in Kind::ALL {
                            if ui.selectable_label(k == kind, k.name()).clicked() {
                                self.kind = Some(k);
                            }
                        }
                    });
                ui.end_row();
                ui.label(RichText::new("Date").weak());
                ui.add(
                    egui::TextEdit::singleline(&mut self.date)
                        .hint_text("2026-11-29 14:30 (the time places that day's drives)")
                        .desired_width(220.0),
                );
                ui.end_row();
                ui.label(RichText::new("Odometer").weak());
                ui.add(
                    egui::TextEdit::singleline(&mut self.miles)
                        .hint_text("miles, as the cluster shows")
                        .desired_width(220.0),
                );
                ui.end_row();
                ui.label(RichText::new("Note").weak());
                ui.add(
                    egui::TextEdit::singleline(&mut self.note)
                        .hint_text(if kind == Kind::Other {
                            "what was done (needed)"
                        } else {
                            "optional"
                        })
                        .desired_width(220.0),
                );
                ui.end_row();
            });
        // a reading offered, for the owner to confirm: never filled in by itself
        if self.recorded.is_none() && !self.looking {
            self.looking = true;
            let d = dir.to_path_buf();
            self.spawn(ui.ctx(), move || {
                let o = mt::offer_from(&recordings(&d));
                Answer::Recorded(d, o)
            });
        }
        if let Some(o) = live.or_else(|| self.recorded.clone().flatten()) {
            ui.horizontal_wrapped(|ui| {
                ui.label(RichText::new(format!("{}.", o.text())).small().color(
                    if o.doubt.is_some() {
                        pal.amber
                    } else {
                        pal.dim
                    },
                ));
                if ui
                    .small_button("Use this reading")
                    .on_hover_text("Fills it in for you to check before Log it")
                    .clicked()
                {
                    self.miles = o.miles.to_string();
                }
            });
        }
        ui.horizontal(|ui| {
            if ui.button("Log it").clicked() {
                let r = number(&self.miles).and_then(|miles| {
                    mt::add(dir, Service::new(kind, &self.date, miles, &self.note))
                });
                self.day_note = None;
                self.said = Some(r.map(|(s, w)| {
                    changed = true;
                    self.miles.clear();
                    self.note.clear();
                    self.date.clear();
                    if let Some(when) = s.when() {
                        let d = dir.to_path_buf();
                        self.spawn(ui.ctx(), move || {
                            let n = mt::day_note(when, &recordings(&d).paths);
                            Answer::DayNote(d, n.map(|n| format!("{n}.")))
                        });
                    }
                    let mut t = format!("Logged: {}", s.line());
                    for x in w {
                        t += &format!(". Note: {x}");
                    }
                    t
                }));
            }
        });
        // the interval of the kind picked, filled in when the kind (or the vehicle) changes, then left as typed
        if self.every_for != Some(kind) {
            let have = log.interval(kind).unwrap_or_default();
            self.every_miles = have.miles.map(|m| m.to_string()).unwrap_or_default();
            self.every_months = have.months.map(|m| m.to_string()).unwrap_or_default();
            self.every_for = Some(kind);
        }
        ui.horizontal_wrapped(|ui| {
            ui.label(RichText::new(format!("{} every", kind.name())).weak());
            ui.add(
                egui::TextEdit::singleline(&mut self.every_miles)
                    .hint_text("miles")
                    .desired_width(80.0),
            );
            ui.label(RichText::new("mi or").weak());
            ui.add(
                egui::TextEdit::singleline(&mut self.every_months)
                    .hint_text("months")
                    .desired_width(60.0),
            );
            ui.label(RichText::new("months").weak());
            if ui
                .small_button("Set")
                .on_hover_text("Both blank: no interval")
                .clicked()
            {
                let r = number(&self.every_miles).and_then(|mi| {
                    let mo = number(&self.every_months)?;
                    mt::set_interval(dir, kind, mi, mo).map(|_| mt::Interval::new(mi, mo))
                });
                self.said = Some(r.map(|i| {
                    changed = true;
                    format!("{}: {}.", kind.name(), i.text())
                }));
            }
        });
        changed
    }

    /// The services logged, newest first, each with Before and after.
    fn list_part(&mut self, ui: &mut egui::Ui, pal: &theme::Palette, dir: &Path, log: &mt::Log) {
        if log.services.is_empty() {
            ui.label(RichText::new("Nothing logged yet.").color(pal.dim));
            return;
        }
        let mut drives = self.drives.unwrap_or(mt::DRIVES);
        ui.horizontal(|ui| {
            ui.label(RichText::new("Before and after: drives on each side").weak());
            ui.add(egui::DragValue::new(&mut drives).range(1..=20));
        });
        if self.drives != Some(drives) {
            self.drives = Some(drives);
            self.compared = None;
            self.asked = None;
        }
        let mut ask: Option<Service> = None;
        egui::ScrollArea::vertical().id_salt(("maintenance-list", dir)).max_height(180.0).show(ui, |ui| {
            for s in log.services.iter().rev() {
                ui.horizontal_wrapped(|ui| {
                    ui.label(s.line());
                    if ui
                        .small_button("Before and after")
                        .on_hover_text("The drives recorded before it against the drives after it, at matching conditions")
                        .clicked()
                    {
                        ask = Some(s.clone());
                    }
                });
            }
        });
        if let Some(s) = ask {
            let key = format!("{} {}", s.date, s.kind.key());
            self.asked = Some(key.clone());
            self.compared = None;
            let d = dir.to_path_buf();
            self.spawn(ui.ctx(), move || {
                let r = s
                    .when()
                    .ok_or_else(|| "its date does not read".to_string())
                    .and_then(|w| {
                        let ev = mt::Event {
                            name: s.kind.name().to_string(),
                            when: w,
                            services: vec![s.clone()],
                        };
                        let recs = recordings(&d);
                        mt::before_after(&ev, &recs.paths, drives, compare::MIN_ROWS).map(
                            |mut b| {
                                b.note = recs.note;
                                b
                            },
                        )
                    });
                Answer::Compared(d, key, Box::new(r))
            });
        }
        if self.asked.is_some() {
            ui.horizontal(|ui| {
                ui.spinner();
                ui.label(RichText::new("Reading the drives around it...").color(pal.dim));
            });
            return;
        }
        let Some((_, r)) = &self.compared else {
            return;
        };
        ui.add_space(6.0);
        match r {
            Err(e) => {
                ui.label(RichText::new(format!("Before and after: {e}")).color(pal.amber));
            }
            Ok(b) => before_after(ui, pal, b),
        }
    }
}

/// The Before and after view: the service, the drives on each side, a side too thin said plainly, the comparison's tables
/// (as the Logs page's Compare view draws them) and the misfire counts.
fn before_after(ui: &mut egui::Ui, pal: &theme::Palette, b: &mt::BeforeAfter) {
    ui.label(RichText::new(format!("Before and after {}", b.event.name)).strong());
    ui.label(RichText::new(b.event.title()).small());
    for l in b.sides() {
        ui.label(RichText::new(l).small().color(pal.dim));
    }
    for l in b.thin_sides() {
        ui.label(RichText::new(l).color(pal.amber));
    }
    let c = &b.comparison;
    ui.label(
        RichText::new(c.conditions(&crate::logs::value_text("°C", compare::WARM_C, 0)))
            .small()
            .color(pal.dim),
    );
    egui::ScrollArea::horizontal()
        .id_salt("maintenance-compare")
        .show(ui, |ui| {
            for t in [compare::Table::Trims, compare::Table::Timing] {
                ui.add_space(6.0);
                ui.label(RichText::new(t.title()).strong());
                if c.groups(t).is_empty() {
                    ui.label(RichText::new("No rows at those conditions.").color(pal.dim));
                    continue;
                }
                crate::logs::compare_table(ui, pal, c, t);
            }
        });
    ui.add_space(6.0);
    ui.label(RichText::new("Misfire counts").strong());
    for l in b.misfire_lines() {
        ui.label(RichText::new(l).small());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_live_reading_is_offered_in_miles_only_with_a_known_unit() {
        let o = live_offer(Some(135_603.3), None).unwrap();
        assert_eq!(o.miles, 84_260);
        assert!(o.from.contains("A6"));
        // from the reading's number and unit, whatever marks its text carries
        let vv = icarus_obd::vehicledata::VehicleValues {
            set: "made-up".into(),
            attribution: String::new(),
            notes: vec![],
            modules: vec![icarus_obd::vehicledata::ModuleValues {
                label: "7E0".into(),
                name: "Engine".into(),
                values: vec![
                    ("Coolant".into(), "190 °F".into(), false),
                    ("Odometer".into(), "84213 mi (unverified)".into(), false),
                ],
                numbers: vec![
                    ("Coolant".into(), 88.0, "°C".into()),
                    ("Odometer".into(), 84_213.0, "mi".into()),
                ],
                notes: vec![],
            }],
        };
        let o = live_offer(None, Some(&vv)).unwrap();
        assert_eq!((o.miles, o.from.contains("7E0")), (84_213, true));
        let mut unchecked = vv.clone();
        unchecked.modules[0].numbers[1].2 = String::new();
        assert_eq!(
            live_offer(None, Some(&unchecked)),
            None,
            "no unit: never offered"
        );
        let (_, notes) = mt::offers_in(&unchecked, "");
        assert!(notes[0].contains("not offered"), "{notes:?}");
        assert_eq!(number(" 84,213 "), Ok(Some(84_213)));
        assert_eq!(number(""), Ok(None));
        assert!(number("lots").is_err());
    }
}
