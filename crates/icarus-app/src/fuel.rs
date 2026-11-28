//! The Garage page's Fuel log card (ICA-211): the shown vehicle's fill-ups, each with the tank's estimated blend after it,
//! and a form to add one; the same file as `icarus fuel` (icarus-obd's `fuel`: `fuel.toml` in the vehicle's folder). Self-
//! contained: its own state, read from and written to that file only. A fill-up is added to the log as it is on disk at that
//! moment, the folder locked (`FuelLog::update`), never to the copy shown, so one added by `icarus fuel add` or by hand
//! meanwhile is kept.

use eframe::egui::{self, vec2, RichText};
use icarus_obd::fuel::{self, FillUp, FuelLog};
use icarus_obd::garage::Described;
use std::path::Path;
use yggdrasil_ui::theme;

/// The card's state: the vehicle it shows (by VIN), its log as read, the fields being typed, and what the last action said.
#[derive(Default)]
pub struct FuelCard {
    vin: String,
    log: Option<Result<FuelLog, String>>,
    grade: String,
    ethanol: String,
    gallons: String,
    odometer: String,
    date: String,
    note: String,
    tank: String,
    /// What was in the tank before (first fill-up only): "91 E0".
    before: String,
    said: Option<Result<String, String>>,
}

/// A number typed in a field: None when empty, an error naming the field when it does not read.
fn typed(s: &str, what: &str) -> Result<Option<f64>, String> {
    let t = s.trim().trim_start_matches(['E', 'e']).replace(',', "");
    if t.is_empty() {
        return Ok(None);
    }
    t.parse::<f64>()
        .map(Some)
        .map_err(|_| format!("{}: {what}", s.trim()))
}

impl FuelCard {
    /// The card for the vehicle shown on the Garage page (its VIN, folder and description).
    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        pal: &theme::Palette,
        vin: &str,
        dir: &Path,
        described: Option<&Described>,
    ) {
        if self.vin != vin {
            *self = FuelCard {
                vin: vin.to_string(),
                ..Default::default()
            };
        }
        if self.log.is_none() {
            self.log = Some(FuelLog::load(dir));
        }
        crate::card(ui, pal, "Fuel log", |ui| {
            ui.label(
                RichText::new(
                    "Each fill-up is taken to fill the tank, so the blend in it is estimated from what was left and the \
                     gallons added. A drive recorded after one names that tank and the blend at its top, and Compare \
                     shows it beside the drive.",
                )
                .small()
                .weak(),
            );
            let log = match self.log.as_ref().expect("just read") {
                Ok(l) => l.clone(),
                Err(e) => {
                    ui.label(
                        RichText::new(format!("The fuel log does not read: {e}")).color(pal.red),
                    );
                    if ui.button("Read it again").clicked() {
                        self.log = None;
                    }
                    return;
                }
            };
            let size = fuel::tank_size(&log, described);
            ui.add_space(4.0);
            egui::Grid::new(("fuel-add", vin))
                .num_columns(2)
                .spacing(vec2(12.0, 6.0))
                .show(ui, |ui| {
                    let field = |ui: &mut egui::Ui, label: &str, value: &mut String, hint: &str| {
                        ui.label(RichText::new(label).weak());
                        ui.add(
                            egui::TextEdit::singleline(value)
                                .hint_text(hint)
                                .desired_width(220.0),
                        );
                        ui.end_row();
                    };
                    field(ui, "Grade (octane, AKI)", &mut self.grade, "87");
                    field(ui, "Ethanol %", &mut self.ethanol, "10");
                    field(ui, "Gallons added", &mut self.gallons, "14.2");
                    field(ui, "Odometer (mi)", &mut self.odometer, "optional");
                    field(
                        ui,
                        "Date and time",
                        &mut self.date,
                        "now (or 2026-11-20 14:05)",
                    );
                    field(ui, "Note", &mut self.note, "optional");
                    let hint = match size {
                        Some((t, from)) => format!("{t} gal ({from})"),
                        None => "needed once: gallons it holds".into(),
                    };
                    field(ui, "Tank size (gal)", &mut self.tank, &hint);
                    // only for a first fill-up: with an earlier one logged, what was left is worked out from it
                    let date = fuel::normal_date(&self.date).unwrap_or_else(|_| fuel::now(false));
                    if !log.has_fill_before(&date) {
                        field(
                            ui,
                            "In the tank before",
                            &mut self.before,
                            "optional, if you know: 91 E0",
                        );
                    }
                });
            ui.horizontal(|ui| {
                if ui.button("Add fill-up").clicked() {
                    self.said = Some(self.add(dir, described));
                }
                if ui
                    .button("Read again")
                    .on_hover_text("After editing fuel.toml by hand")
                    .clicked()
                {
                    self.log = None;
                    self.said = None;
                }
            });
            match &self.said {
                Some(Ok(s)) => {
                    ui.label(RichText::new(s).color(pal.green));
                }
                Some(Err(e)) => {
                    ui.label(RichText::new(e).color(pal.red));
                }
                None => {}
            }
            ui.add_space(6.0);
            if log.fills.is_empty() {
                ui.label(RichText::new("No fill-ups logged yet.").weak());
                return;
            }
            let tanks = fuel::tanks(&log, size.map(|s| s.0));
            for t in tanks.iter().rev() {
                let f = &t.fill;
                ui.horizontal_wrapped(|ui| {
                    ui.label(RichText::new(&f.date).weak());
                    ui.label(RichText::new(f.label()).strong());
                    ui.label(format!("{} gal", f.gallons));
                    if let Some(m) = f.odometer {
                        ui.label(RichText::new(format!("{m} mi")).weak());
                    }
                    if !f.note.is_empty() {
                        ui.label(RichText::new(&f.note).weak());
                    }
                });
                ui.label(
                    RichText::new(format!("The tank: {}", t.estimate_words()))
                        .small()
                        .weak(),
                );
            }
        });
    }

    /// Adds the fill-up typed (and the tank's size, when typed) to the log in `dir` as it is on disk now; what to say.
    fn add(&mut self, dir: &Path, described: Option<&Described>) -> Result<String, String> {
        let tank = typed(&self.tank, "the tank's size in gallons")?;
        if tank.is_some_and(|t| !(t > 0.0 && t < 500.0)) {
            return Err(format!("{}: the tank's size in gallons", self.tank.trim()));
        }
        let grade = typed(&self.grade, "the octane the pump shows (AKI), like 87")?
            .ok_or("The grade: the octane the pump shows (AKI), like 87 or 91.")?;
        let ethanol = typed(&self.ethanol, "the ethanol percent, like 10")?
            .ok_or("The ethanol percent the pump shows: 10 for E10, 0 for ethanol-free.")?;
        let gallons = typed(&self.gallons, "the gallons added")?.ok_or("The gallons added.")?;
        let odometer = match self.odometer.trim().replace([',', ' '], "") {
            m if m.is_empty() => None,
            m => Some(
                m.parse::<u32>()
                    .map_err(|_| format!("{}: the miles, a whole number", self.odometer.trim()))?,
            ),
        };
        let date = match self.date.trim() {
            "" => fuel::now(false),
            d => d.to_string(),
        };
        let fill = FillUp {
            date,
            grade,
            ethanol,
            gallons,
            odometer,
            note: self.note.clone(),
            before: Some(self.before.clone()).filter(|b| !b.trim().is_empty()),
        };
        // the folder is there (the Garage page lists only vehicles with one); the log read again, locked, and refused if
        // it does not read, so nothing added meanwhile is lost
        let r = FuelLog::update(dir, |log| {
            if tank.is_some() {
                log.tank_gallons = tank;
            }
            if fuel::tank_size(log, described).is_none() {
                return Err(
                    "How much does the tank hold? Type its size in gallons once: it is kept."
                        .into(),
                );
            }
            log.add(fill)
        });
        let (_, log, _) = match r {
            Ok(r) => r,
            Err(e) => {
                // shown as it is on disk now
                self.log = Some(FuelLog::load(dir));
                return Err(e);
            }
        };
        let added = log.fills.last().map(FillUp::label).unwrap_or_default();
        self.log = Some(Ok(log));
        for f in [
            &mut self.gallons,
            &mut self.odometer,
            &mut self.date,
            &mut self.note,
            &mut self.tank,
            &mut self.before,
        ] {
            f.clear();
        }
        Ok(format!("Added: {added}."))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_fill_up_typed_is_added_and_the_tank_asked_for_once() {
        let dir = std::env::temp_dir().join(format!("icarus-app-fuel-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let mut c = FuelCard {
            grade: "87".into(),
            ethanol: "E10".into(),
            gallons: "15".into(),
            date: "2026-11-10 17:30".into(),
            ..Default::default()
        };
        let ford = Described::parse("2017 Ford F-150");
        assert!(c
            .add(&dir, Some(&ford))
            .unwrap_err()
            .contains("How much does the tank hold"));
        c.tank = "26".into();
        assert_eq!(c.add(&dir, Some(&ford)).unwrap(), "Added: 87 E10.");
        let log = FuelLog::load(&dir).unwrap();
        assert_eq!((log.tank_gallons, log.fills.len()), (Some(26.0), 1));
        assert_eq!(log.fills[0].date, "2026-11-10 17:30");
        assert!(
            c.gallons.is_empty() && c.grade == "87",
            "the grade stays for the next"
        );
        c.gallons = "lots".into();
        assert!(c.add(&dir, Some(&ford)).unwrap_err().contains("lots"));

        // a fill-up added meanwhile by icarus fuel add (or by hand) is kept: the card adds to the log on disk
        let mut other = FuelLog::load(&dir).unwrap();
        other
            .add(FillUp {
                date: "2026-11-15 08:00".into(),
                grade: 91.0,
                gallons: 12.0,
                ..Default::default()
            })
            .unwrap();
        other.save(&dir).unwrap();
        c.gallons = "14".into();
        c.date = "2026-11-20 09:00".into();
        c.add(&dir, Some(&ford)).unwrap();
        let dates: Vec<String> = FuelLog::load(&dir)
            .unwrap()
            .sorted()
            .iter()
            .map(|f| f.date.clone())
            .collect();
        assert_eq!(
            dates,
            ["2026-11-10 17:30", "2026-11-15 08:00", "2026-11-20 09:00"]
        );
        // one that does not read is refused, not replaced
        std::fs::write(dir.join(fuel::FILE), "[[fill]]\nethonal = 10\n").unwrap();
        c.gallons = "14".into();
        assert!(c.add(&dir, Some(&ford)).is_err());
        assert!(std::fs::read_to_string(dir.join(fuel::FILE))
            .unwrap()
            .contains("ethonal"));
        assert!(matches!(c.log, Some(Err(_))), "shown as it is now");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
