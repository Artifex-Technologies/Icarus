//! Icarus's window: the vehicle (adapter, identity, check-engine light, monitors, codes), its modules (identity and codes,
//! as dealer tools show them), live values with graphs, and maps from an image and a definition (with what a tune
//! changed). Read-only, like the rest of Icarus for now. Yggdrasil's look (yggdrasil-ui).

mod worker;

use eframe::egui::{self, vec2, Color32, CornerRadius, Margin, RichText, Sense, Stroke};
use icarus_core::def::{fmt_num, Definition, Kind};
use icarus_core::rom::{self, Rom};
use icarus_core::xdf;
use icarus_obd::j1979;
use icarus_obd::uds::Dialect;
use std::collections::{BTreeSet, HashMap, VecDeque};
use std::sync::mpsc::{channel, Receiver, Sender};
use worker::{Cmd, Event, Info, ModuleView};
use yggdrasil_ui::{chrome, theme};

#[derive(Clone, Copy, PartialEq, Eq)]
enum Page {
    Vehicle,
    Modules,
    Live,
    Maps,
}

const PAGES: [(Page, &str); 4] = [
    (Page::Vehicle, "Vehicle"),
    (Page::Modules, "Modules"),
    (Page::Live, "Live data"),
    (Page::Maps, "Maps"),
];

#[derive(PartialEq, Eq)]
enum Link {
    Off,
    Busy,
    On,
}

struct Maps {
    def_path: String,
    bin_path: String,
    cmp_path: String,
    def: Option<Definition>,
    rom: Option<Rom>,
    cmp: Option<Rom>,
    /// Per element: changed between the two images.
    changed: Vec<bool>,
    note: String,
    search: String,
    only_changed: bool,
    selected: Option<usize>,
}

/// What to do at start (command line): `--connect` (the adapter from ICARUS_ADAPTER, or `--adapter A`), `--page NAME`,
/// `--scan` and `--live` once connected, `--maps DEF.xdf IMAGE.bin [SECOND.bin]`.
#[derive(Default)]
struct Start {
    connect: bool,
    adapter: Option<String>,
    page: Option<Page>,
    scan: bool,
    live: bool,
    maps: Option<(String, String, String)>,
}

impl Start {
    fn parse(a: &[String]) -> Result<Start, String> {
        let mut s = Start::default();
        let mut i = 0;
        while i < a.len() {
            match a[i].as_str() {
                "--connect" => s.connect = true,
                "--scan" => s.scan = true,
                "--live" => s.live = true,
                "--adapter" => {
                    i += 1;
                    s.adapter = Some(a.get(i).ok_or("--adapter needs a value")?.clone());
                }
                "--page" => {
                    i += 1;
                    let want = a.get(i).ok_or("--page needs a name")?.to_lowercase();
                    s.page = Some(
                        PAGES
                            .iter()
                            .find(|p| p.1.to_lowercase().starts_with(&want))
                            .ok_or(format!("no page {want}"))?
                            .0,
                    );
                }
                "--maps" => {
                    let d = a
                        .get(i + 1)
                        .ok_or("--maps needs a definition and an image")?
                        .clone();
                    let b = a.get(i + 2).ok_or("--maps needs an image too")?.clone();
                    let c = a.get(i + 3).filter(|x| !x.starts_with("--")).cloned();
                    i += if c.is_some() { 3 } else { 2 };
                    s.maps = Some((d, b, c.unwrap_or_default()));
                }
                other => return Err(format!("unknown option {other}")),
            }
            i += 1;
        }
        Ok(s)
    }
}

struct App {
    start: Start,
    page: Page,
    tx: Sender<Cmd>,
    rx: Receiver<Event>,
    adapter: String,
    link: Link,
    status: String,
    error: Option<String>,
    info: Option<Info>,
    profile_choice: String,
    sweep: bool,
    scanning: bool,
    modules: Vec<ModuleView>,
    scan_note: String,
    unreachable: Vec<String>,
    live_on: bool,
    chosen: BTreeSet<u8>,
    history: HashMap<u8, VecDeque<(f64, f64)>>,
    maps: Maps,
}

impl App {
    fn new(ctx: &egui::Context, start: Start) -> App {
        let (tx, wrx) = channel();
        let (wtx, rx) = channel();
        let c = ctx.clone();
        std::thread::spawn(move || worker::run(wrx, wtx, move || c.request_repaint()));
        let mut app = App {
            page: start.page.unwrap_or(Page::Vehicle),
            tx,
            rx,
            adapter: start
                .adapter
                .clone()
                .or_else(|| std::env::var("ICARUS_ADAPTER").ok())
                .unwrap_or_else(|| "sim".into()),
            link: Link::Off,
            status: "Not connected.".into(),
            error: None,
            info: None,
            profile_choice: String::new(),
            sweep: false,
            scanning: false,
            modules: Vec::new(),
            scan_note: String::new(),
            unreachable: Vec::new(),
            live_on: false,
            chosen: [0x0C, 0x05, 0x06, 0x07, 0x44].into_iter().collect(),
            history: HashMap::new(),
            maps: Maps {
                def_path: String::new(),
                bin_path: String::new(),
                cmp_path: String::new(),
                def: None,
                rom: None,
                cmp: None,
                changed: Vec::new(),
                note: String::new(),
                search: String::new(),
                only_changed: false,
                selected: None,
            },
            start,
        };
        if let Some((d, b, c)) = app.start.maps.take() {
            (app.maps.def_path, app.maps.bin_path, app.maps.cmp_path) = (d, b, c);
            app.load_maps();
        }
        if app.start.connect {
            app.link = Link::Busy;
            let _ = app.tx.send(Cmd::Connect(app.adapter.trim().to_string()));
        }
        app
    }

    fn events(&mut self) {
        while let Ok(ev) = self.rx.try_recv() {
            match ev {
                Event::Status(s) => self.status = s,
                Event::Connected(info) => {
                    self.link = Link::On;
                    self.error = None;
                    self.status = "Connected.".into();
                    self.chosen.retain(|p| info.live.contains(p));
                    self.info = Some(*info);
                    self.modules.clear();
                    self.history.clear();
                    if std::mem::take(&mut self.start.scan) {
                        self.scanning = true;
                        let _ = self.tx.send(Cmd::Scan {
                            profile: None,
                            sweep: false,
                        });
                    }
                    if std::mem::take(&mut self.start.live) && !self.chosen.is_empty() {
                        self.live_on = true;
                        let _ = self
                            .tx
                            .send(Cmd::Live(self.chosen.iter().copied().collect()));
                    }
                }
                Event::Failed(m) => {
                    if self.link == Link::Busy {
                        self.link = Link::Off;
                    }
                    self.scanning = false;
                    self.live_on = false;
                    self.status = "Something went wrong.".into();
                    self.error = Some(m);
                }
                Event::Disconnected => {
                    self.link = Link::Off;
                    self.info = None;
                    self.live_on = false;
                    self.status = "Not connected.".into();
                }
                Event::ScanStarted(profile, n) => {
                    self.modules.clear();
                    self.unreachable.clear();
                    self.scan_note = format!("{profile}: {n} addresses");
                }
                Event::Module(m) => self.modules.push(m),
                Event::ScanDone { found, unreachable } => {
                    self.scanning = false;
                    self.unreachable = unreachable;
                    self.status = format!("Scan done: {found} module(s) answered.");
                }
                Event::Reading { pid, value, at } => {
                    let h = self.history.entry(pid).or_default();
                    h.push_back((at, value));
                    while h.len() > 300 {
                        h.pop_front();
                    }
                }
            }
        }
    }

    fn connect_row(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Adapter");
            ui.add_enabled(
                self.link == Link::Off,
                egui::TextEdit::singleline(&mut self.adapter).desired_width(280.0),
            );
            match self.link {
                Link::Off => {
                    if ui.button("Connect").clicked() {
                        self.link = Link::Busy;
                        self.error = None;
                        let _ = self.tx.send(Cmd::Connect(self.adapter.trim().to_string()));
                    }
                }
                Link::Busy => {
                    ui.spinner();
                }
                Link::On => {
                    if ui.button("Disconnect").clicked() {
                        let _ = self.tx.send(Cmd::Disconnect);
                    }
                }
            }
        });
        ui.label(
            RichText::new("sim (a simulated vehicle) · bt:AA:BB:CC:DD:EE:FF (paired Bluetooth, e.g. OBDLink MX+) · /dev/ttyUSB0 · tcp:192.168.0.10:35000")
                .small()
                .weak(),
        );
    }

    fn page_vehicle(&mut self, ui: &mut egui::Ui, pal: &theme::Palette) {
        self.connect_row(ui);
        ui.add_space(12.0);
        let Some(info) = &self.info else {
            ui.label("Plug the adapter into the vehicle's OBD port (under the dashboard, driver's side), switch the ignition on (the engine can stay off), then connect.");
            return;
        };
        card(ui, pal, "Vehicle", |ui| {
            egui::Grid::new("veh")
                .num_columns(2)
                .spacing(vec2(16.0, 6.0))
                .show(ui, |ui| {
                    row(ui, "VIN", info.vin.as_deref().unwrap_or("(not given)"));
                    if let Some(v) = &info.vin_info {
                        row(ui, "Built in", v.region);
                        if let Some(y) = v.year {
                            row(ui, "Model year", &y.to_string());
                        }
                    }
                    row(
                        ui,
                        "Profile",
                        info.profile
                            .as_ref()
                            .map_or("generic OBD-II (no profile for this VIN)", |p| {
                                p.name.as_str()
                            }),
                    );
                    row(ui, "Adapter", &info.adapter);
                    if let Some(v) = info.voltage {
                        row(ui, "Battery", &format!("{v:.1} V at the OBD port"));
                    }
                    row(ui, "Protocol", &info.protocol);
                    if let Some(f) = info.fuel_system {
                        row(ui, "Fuel system", f);
                    }
                });
        });
        for (module, m) in &info.monitors {
            card(
                ui,
                pal,
                &format!("Check-engine light and monitors: {module}"),
                |ui| {
                    let (txt, col) = if m.mil {
                        ("ON", pal.red)
                    } else {
                        ("off", pal.green)
                    };
                    ui.label(
                        RichText::new(format!(
                            "Check-engine light {txt}, {} confirmed code(s)",
                            m.codes
                        ))
                        .color(col)
                        .strong(),
                    );
                    ui.add_space(4.0);
                    for (name, done) in &m.tests {
                        ui.horizontal(|ui| {
                            dot(ui, if *done { pal.green } else { pal.amber }, *done);
                            ui.label(*name);
                            ui.label(
                                RichText::new(if *done {
                                    "complete"
                                } else {
                                    "not yet run since codes were cleared"
                                })
                                .weak(),
                            );
                        });
                    }
                },
            );
        }
        card(ui, pal, "Trouble codes (OBD-II)", |ui| {
            if info.codes.is_empty() {
                ui.label("None stored, pending or permanent.");
            }
            for (kind, module, codes) in &info.codes {
                ui.label(format!("{kind}: {module}: {}", codes.join(", ")));
            }
            ui.label(
                RichText::new(
                    "The Modules page reads every module's own codes (ABS, airbags, body...).",
                )
                .weak(),
            );
        });
        if let Some(p) = &info.profile {
            if !p.notes.is_empty() {
                card(ui, pal, "About this make", |ui| {
                    ui.label(p.notes.split_whitespace().collect::<Vec<_>>().join(" "));
                });
            }
        }
    }

    fn page_modules(&mut self, ui: &mut egui::Ui, pal: &theme::Palette) {
        if self.link != Link::On {
            ui.label("Connect on the Vehicle page first.");
            return;
        }
        ui.horizontal(|ui| {
            ui.label("Make");
            let auto = self
                .info
                .as_ref()
                .and_then(|i| i.profile.as_ref())
                .map_or("generic".to_string(), |p| p.name.clone());
            egui::ComboBox::from_id_salt("make")
                .selected_text(if self.profile_choice.is_empty() {
                    format!("from the VIN ({auto})")
                } else {
                    self.profile_choice.clone()
                })
                .show_ui(ui, |ui| {
                    if ui
                        .selectable_label(self.profile_choice.is_empty(), "from the VIN")
                        .clicked()
                    {
                        self.profile_choice.clear();
                    }
                    for p in
                        icarus_obd::vehicles::load(icarus_obd::vehicles::user_dir().as_deref()).0
                    {
                        if ui
                            .selectable_label(self.profile_choice == p.id, &p.name)
                            .clicked()
                        {
                            self.profile_choice = p.id.clone();
                        }
                    }
                });
            ui.checkbox(&mut self.sweep, "every ID 700-7F7 (slower)");
            if self.scanning {
                ui.spinner();
            } else if ui.button("Scan").clicked() {
                self.scanning = true;
                self.live_on = false;
                let profile =
                    (!self.profile_choice.is_empty()).then(|| self.profile_choice.clone());
                let _ = self.tx.send(Cmd::Scan {
                    profile,
                    sweep: self.sweep,
                });
            }
        });
        if !self.scan_note.is_empty() {
            ui.label(RichText::new(&self.scan_note).weak());
        }
        ui.add_space(8.0);
        for m in &self.modules {
            let title = format!("{:03X}  {}", m.addr.tx, m.addr.name);
            card(ui, pal, &title, |ui| {
                if !m.addr.checked {
                    ui.label(
                        RichText::new("address from public sources, not yet checked on a vehicle")
                            .small()
                            .weak(),
                    );
                }
                egui::Grid::new(("id", m.addr.tx))
                    .num_columns(2)
                    .spacing(vec2(16.0, 4.0))
                    .show(ui, |ui| {
                        for (what, v) in &m.identity {
                            row(ui, what, v);
                        }
                    });
                match &m.dtcs {
                    None => {
                        ui.label(RichText::new("It would not give its trouble codes.").weak());
                    }
                    Some((d, codes)) => {
                        let lang = if *d == Dialect::Uds { "UDS" } else { "KWP2000" };
                        if codes.is_empty() {
                            ui.label(
                                RichText::new(format!("No trouble codes ({lang})"))
                                    .color(pal.green),
                            );
                        }
                        for c in codes {
                            let confirmed = c.states.iter().any(|s| {
                                matches!(*s, "confirmed" | "stored" | "warning light requested")
                            });
                            let ftb = c
                                .failure_type
                                .map(|f| format!("-{f:02X}"))
                                .unwrap_or_default();
                            ui.horizontal(|ui| {
                                ui.label(
                                    RichText::new(format!("{}{ftb}", c.code))
                                        .strong()
                                        .color(if confirmed { pal.red } else { pal.amber }),
                                );
                                ui.label(c.states.join(", "));
                                ui.label(RichText::new(lang).weak());
                            });
                        }
                    }
                }
            });
        }
        if !self.unreachable.is_empty() {
            card(ui, pal, "Not reachable yet", |ui| {
                for u in &self.unreachable {
                    ui.label(u);
                }
            });
        }
    }

    fn page_live(&mut self, ui: &mut egui::Ui, pal: &theme::Palette) {
        let Some(info) = &self.info else {
            ui.label("Connect on the Vehicle page first.");
            return;
        };
        let offered = info.live.clone();
        ui.horizontal(|ui| {
            if self.live_on {
                if ui.button("Stop").clicked() {
                    self.live_on = false;
                    let _ = self.tx.send(Cmd::Live(vec![]));
                }
                ui.spinner();
            } else if ui
                .add_enabled(
                    !self.chosen.is_empty() && !self.scanning,
                    egui::Button::new("Start"),
                )
                .clicked()
            {
                self.live_on = true;
                let _ = self
                    .tx
                    .send(Cmd::Live(self.chosen.iter().copied().collect()));
            }
            ui.label(
                RichText::new(format!(
                    "{} of {} values chosen; fewer values update faster",
                    self.chosen.len(),
                    offered.len()
                ))
                .weak(),
            );
        });
        ui.add_space(6.0);
        ui.columns(2, |cols| {
            egui::ScrollArea::vertical()
                .id_salt("pick")
                .show(&mut cols[0], |ui| {
                    for p in &offered {
                        let d = j1979::pid(*p).unwrap();
                        let mut on = self.chosen.contains(p);
                        if ui
                            .checkbox(&mut on, d.name)
                            .on_hover_text(d.about)
                            .changed()
                        {
                            if on {
                                self.chosen.insert(*p);
                            } else {
                                self.chosen.remove(p);
                            }
                            if self.live_on {
                                let _ = self
                                    .tx
                                    .send(Cmd::Live(self.chosen.iter().copied().collect()));
                            }
                        }
                    }
                });
            egui::ScrollArea::vertical()
                .id_salt("values")
                .show(&mut cols[1], |ui| {
                    for p in &self.chosen {
                        let d = j1979::pid(*p).unwrap();
                        let h = self.history.get(p);
                        let last = h.and_then(|h| h.back()).map(|x| x.1);
                        card(ui, pal, d.name, |ui| {
                            ui.horizontal(|ui| {
                                ui.label(
                                    RichText::new(
                                        last.map_or("-".to_string(), |v| format!("{v:.2}")),
                                    )
                                    .size(24.0)
                                    .strong(),
                                );
                                ui.label(d.unit);
                            });
                            if let Some(h) = h {
                                graph(ui, pal, h);
                            }
                            ui.label(RichText::new(d.about).small().weak());
                        });
                    }
                });
        });
    }

    fn load_maps(&mut self) {
        let m = &mut self.maps;
        m.note.clear();
        m.selected = None;
        let result = (|| -> Result<(), String> {
            let text = std::fs::read_to_string(m.def_path.trim())
                .map_err(|e| format!("definition: {e}"))?;
            let p = xdf::parse(&text).map_err(|e| format!("definition: {e}"))?;
            let bin = std::fs::read(m.bin_path.trim()).map_err(|e| format!("image: {e}"))?;
            let cmp = if m.cmp_path.trim().is_empty() {
                None
            } else {
                Some(std::fs::read(m.cmp_path.trim()).map_err(|e| format!("second image: {e}"))?)
            };
            let mut changed = vec![false; p.definition.elements.len()];
            if let Some(c) = &cmp {
                if c.len() != bin.len() {
                    return Err("the two images differ in size".into());
                }
                for r in rom::diff(&bin, c) {
                    for e in p.definition.covering(r) {
                        if let Some(i) = p
                            .definition
                            .elements
                            .iter()
                            .position(|x| std::ptr::eq(x, e))
                        {
                            changed[i] = true;
                        }
                    }
                }
            }
            let n = changed.iter().filter(|c| **c).count();
            m.note = format!(
                "{} elements{}{}",
                p.definition.elements.len(),
                if p.skipped.is_empty() {
                    String::new()
                } else {
                    format!(
                        ", {} left out (formulas Icarus cannot read yet)",
                        p.skipped.len()
                    )
                },
                if cmp.is_some() {
                    format!("; {n} changed in the second image")
                } else {
                    String::new()
                }
            );
            if let Some(size) = p.definition.image_size {
                if size != bin.len() {
                    m.note.push_str(&format!(
                        ". Note: the definition is for 0x{size:X} bytes, the image has 0x{:X}",
                        bin.len()
                    ));
                }
            }
            m.def = Some(p.definition);
            m.rom = Some(Rom::new(bin));
            m.cmp = cmp.map(Rom::new);
            m.only_changed = m.cmp.is_some() && n > 0;
            // comparing a tune: open on the first thing it changed
            m.selected = changed.iter().position(|c| *c);
            m.changed = changed;
            Ok(())
        })();
        if let Err(e) = result {
            m.def = None;
            m.note = e;
        }
    }

    fn page_maps(&mut self, ui: &mut egui::Ui, pal: &theme::Palette) {
        egui::Grid::new("paths")
            .num_columns(2)
            .spacing(vec2(10.0, 6.0))
            .show(ui, |ui| {
                ui.label("Definition (.xdf)");
                ui.add(egui::TextEdit::singleline(&mut self.maps.def_path).desired_width(520.0));
                ui.end_row();
                ui.label("Image (.bin)");
                ui.add(egui::TextEdit::singleline(&mut self.maps.bin_path).desired_width(520.0));
                ui.end_row();
                ui.label("Compare with");
                ui.add(
                    egui::TextEdit::singleline(&mut self.maps.cmp_path)
                        .hint_text("optional: a modified image, to see what it changed")
                        .desired_width(520.0),
                );
                ui.end_row();
            });
        ui.horizontal(|ui| {
            if ui.button("Open").clicked() {
                self.load_maps();
            }
            ui.label(RichText::new(&self.maps.note).weak());
        });
        ui.add_space(6.0);
        let Maps {
            def,
            rom,
            cmp,
            changed,
            search,
            only_changed,
            selected,
            ..
        } = &mut self.maps;
        let (Some(def), Some(rom)) = (def.as_ref(), rom.as_ref()) else {
            return;
        };
        let q = search.to_lowercase();
        let shown: Vec<usize> = def
            .elements
            .iter()
            .enumerate()
            .filter(|(i, e)| {
                (!*only_changed || changed[*i])
                    && (q.is_empty()
                        || e.title.to_lowercase().contains(&q)
                        || e.description.to_lowercase().contains(&q))
            })
            .map(|(i, _)| i)
            .collect();
        let avail = ui.available_height();
        ui.horizontal_top(|ui| {
            ui.vertical(|ui| {
                ui.set_width(300.0);
                ui.add(
                    egui::TextEdit::singleline(search)
                        .hint_text("Search maps")
                        .desired_width(300.0),
                );
                if cmp.is_some() {
                    ui.checkbox(only_changed, "only what changed");
                }
                ui.label(
                    RichText::new(format!("{} shown", shown.len()))
                        .small()
                        .weak(),
                );
                let rh = 22.0;
                egui::ScrollArea::vertical()
                    .id_salt("elements")
                    .max_height(avail - 70.0)
                    .auto_shrink([false, false])
                    .show_rows(ui, rh, shown.len(), |ui, range| {
                        for &i in &shown[range] {
                            let e = &def.elements[i];
                            let sel = *selected == Some(i);
                            let (rect, resp) = ui.allocate_exact_size(
                                vec2(ui.available_width(), rh),
                                Sense::click(),
                            );
                            if sel {
                                ui.painter()
                                    .rect_filled(rect, CornerRadius::same(4), pal.select);
                            } else if resp.hovered() {
                                ui.painter()
                                    .rect_filled(rect, CornerRadius::same(4), pal.hover);
                            }
                            let col = if changed.get(i).copied().unwrap_or(false) {
                                pal.amber
                            } else {
                                pal.text
                            };
                            ui.painter().text(
                                rect.left_center() + vec2(6.0, 0.0),
                                egui::Align2::LEFT_CENTER,
                                &e.title,
                                egui::FontId::proportional(13.0),
                                col,
                            );
                            if resp.clicked() {
                                *selected = Some(i);
                            }
                        }
                    });
            });
            ui.separator();
            ui.vertical(|ui| {
                let Some(i) = *selected else {
                    ui.label("Choose a map on the left.");
                    return;
                };
                element_view(ui, pal, def, rom, cmp.as_ref(), &def.elements[i]);
            });
        });
    }
}

fn element_view(
    ui: &mut egui::Ui,
    pal: &theme::Palette,
    def: &Definition,
    rom: &Rom,
    cmp: Option<&Rom>,
    e: &icarus_core::def::Element,
) {
    ui.label(RichText::new(&e.title).size(18.0).strong());
    if !e.description.is_empty() {
        ui.label(&e.description);
    }
    if !e.categories.is_empty() {
        ui.label(RichText::new(e.categories.join(", ")).weak());
    }
    ui.add_space(6.0);
    match &e.kind {
        Kind::Flag(f) => {
            let a = def.flag(rom, f);
            let b = cmp.map(|c| def.flag(c, f));
            match (a, b) {
                (Ok(a), None) => ui.label(if a { "on" } else { "off" }),
                (Ok(a), Some(Ok(b))) => ui.label(format!(
                    "{} -> {}",
                    if a { "on" } else { "off" },
                    if b { "on" } else { "off" }
                )),
                (Err(x), _) | (_, Some(Err(x))) => ui.colored_label(pal.red, x.to_string()),
            };
        }
        Kind::Constant(d) => match def.values(rom, d) {
            Ok(v) => {
                let after = cmp
                    .and_then(|c| def.values(c, d).ok())
                    .map(|w| w[0][0])
                    .filter(|w| *w != v[0][0]);
                ui.label(
                    RichText::new(format!("{} {}", fmt_num(v[0][0], d.decimals), d.units))
                        .size(22.0),
                );
                if let Some(w) = after {
                    ui.label(
                        RichText::new(format!(
                            "second image: {} {}",
                            fmt_num(w, d.decimals),
                            d.units
                        ))
                        .color(pal.amber),
                    );
                }
            }
            Err(x) => {
                ui.colored_label(pal.red, x.to_string());
            }
        },
        Kind::Table(t) => {
            let v = match def.table(rom, t) {
                Ok(v) => v,
                Err(x) => {
                    ui.colored_label(pal.red, x.to_string());
                    return;
                }
            };
            let w = cmp.and_then(|c| def.table(c, t).ok());
            if !t.z.units.is_empty() {
                ui.label(RichText::new(format!("values in {}", t.z.units)).weak());
            }
            let all: Vec<f64> = v.z.concat();
            let (lo, hi) = all
                .iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), x| {
                    (a.min(*x), b.max(*x))
                });
            egui::ScrollArea::both()
                .id_salt("table")
                .auto_shrink([false, false])
                .show(ui, |ui| {
                    egui::Grid::new("cells")
                        .spacing(vec2(2.0, 2.0))
                        .show(ui, |ui| {
                            ui.label("");
                            for x in &v.x {
                                ui.label(RichText::new(x).small().strong());
                            }
                            ui.end_row();
                            for (r, rowv) in v.z.iter().enumerate() {
                                ui.label(
                                    RichText::new(v.y.get(r).map(String::as_str).unwrap_or(""))
                                        .small()
                                        .strong(),
                                );
                                for (c, n) in rowv.iter().enumerate() {
                                    let f = if hi > lo {
                                        ((n - lo) / (hi - lo)) as f32
                                    } else {
                                        0.5
                                    };
                                    let other = w
                                        .as_ref()
                                        .and_then(|w| w.z.get(r).and_then(|rw| rw.get(c)))
                                        .copied()
                                        .filter(|o| o != n);
                                    let shown = other.unwrap_or(*n);
                                    let frame = egui::Frame::new()
                                        .fill(heat(f))
                                        .inner_margin(Margin::symmetric(5, 2))
                                        .stroke(if other.is_some() {
                                            Stroke::new(2.0, pal.amber)
                                        } else {
                                            Stroke::NONE
                                        });
                                    let resp = frame
                                        .show(ui, |ui| {
                                            ui.label(
                                                RichText::new(fmt_num(shown, t.z.decimals))
                                                    .color(Color32::WHITE)
                                                    .small(),
                                            )
                                        })
                                        .response;
                                    if let Some(o) = other {
                                        let _ = resp.on_hover_text(format!(
                                            "first image {} -> second {}",
                                            fmt_num(*n, t.z.decimals),
                                            fmt_num(o, t.z.decimals)
                                        ));
                                    }
                                }
                                ui.end_row();
                            }
                        });
                });
        }
    }
}

/// Blue (low) to red (high), dark enough for white text.
fn heat(f: f32) -> Color32 {
    let f = f.clamp(0.0, 1.0);
    let lerp = |a: u8, b: u8| (a as f32 + (b as f32 - a as f32) * f) as u8;
    Color32::from_rgb(lerp(40, 190), lerp(86, 60), lerp(150, 50))
}

fn graph(ui: &mut egui::Ui, pal: &theme::Palette, h: &VecDeque<(f64, f64)>) {
    let (rect, _) = ui.allocate_exact_size(vec2(ui.available_width(), 60.0), Sense::hover());
    ui.painter()
        .rect_filled(rect, CornerRadius::same(4), pal.field);
    if h.len() < 2 {
        return;
    }
    let (t0, t1) = (h.front().unwrap().0, h.back().unwrap().0);
    let (lo, hi) = h
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), x| {
            (a.min(x.1), b.max(x.1))
        });
    let span = if hi > lo { hi - lo } else { 1.0 };
    let pts: Vec<egui::Pos2> = h
        .iter()
        .map(|(t, v)| {
            let x = rect.left() + ((t - t0) / (t1 - t0).max(1e-9)) as f32 * rect.width();
            let y = rect.bottom() - 4.0 - ((v - lo) / span) as f32 * (rect.height() - 8.0);
            egui::pos2(x, y)
        })
        .collect();
    ui.painter()
        .add(egui::Shape::line(pts, Stroke::new(1.5, pal.accent)));
}

/// A small status dot (the fonts have no filled circle).
fn dot(ui: &mut egui::Ui, col: Color32, filled: bool) {
    let (rect, _) = ui.allocate_exact_size(vec2(12.0, 12.0), Sense::hover());
    if filled {
        ui.painter().circle_filled(rect.center(), 4.5, col);
    } else {
        ui.painter()
            .circle_stroke(rect.center(), 4.0, Stroke::new(1.5, col));
    }
}

fn row(ui: &mut egui::Ui, k: &str, v: &str) {
    ui.label(RichText::new(k).weak());
    ui.label(v);
    ui.end_row();
}

fn card(ui: &mut egui::Ui, pal: &theme::Palette, title: &str, body: impl FnOnce(&mut egui::Ui)) {
    egui::Frame::new()
        .fill(pal.field)
        .corner_radius(CornerRadius::same(10))
        .inner_margin(Margin::symmetric(14, 10))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.label(RichText::new(title).strong());
            ui.add_space(4.0);
            body(ui);
        });
    ui.add_space(8.0);
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        theme::apply_accessibility(ui.ctx());
        let ctx = ui.ctx().clone();
        chrome::resize_edges(&ctx);
        let accent = theme::system_accent();
        let dark = theme::system_dark();
        theme::apply(&ctx, dark, accent);
        let pal = theme::palette_for(dark, accent);
        self.events();

        egui::Panel::left("pages")
            .default_size(190.0)
            .resizable(false)
            .frame(
                egui::Frame::new()
                    .fill(pal.sidebar)
                    .inner_margin(Margin::symmetric(10, 12)),
            )
            .show(ui, |ui| {
                ui.label(RichText::new("Icarus").size(20.0).strong());
                ui.label(RichText::new("read-only").small().weak());
                ui.add_space(14.0);
                for (p, name) in PAGES {
                    let sel = self.page == p;
                    let (rect, resp) =
                        ui.allocate_exact_size(vec2(ui.available_width(), 34.0), Sense::click());
                    if sel {
                        ui.painter()
                            .rect_filled(rect, CornerRadius::same(8), pal.select);
                    } else if resp.hovered() {
                        ui.painter()
                            .rect_filled(rect, CornerRadius::same(8), pal.hover);
                    }
                    ui.painter().text(
                        rect.left_center() + vec2(12.0, 0.0),
                        egui::Align2::LEFT_CENTER,
                        name,
                        egui::FontId::proportional(14.0),
                        pal.text,
                    );
                    if resp.clicked() {
                        self.page = p;
                    }
                }
                ui.with_layout(egui::Layout::bottom_up(egui::Align::Min), |ui| {
                    let (text, col) = match self.link {
                        Link::On => ("connected", pal.green),
                        Link::Busy => ("connecting", pal.amber),
                        Link::Off => ("not connected", pal.dim),
                    };
                    ui.horizontal(|ui| {
                        dot(ui, col, true);
                        ui.label(RichText::new(text).color(col).small());
                    });
                });
            });

        egui::CentralPanel::default()
            .frame(egui::Frame::new().fill(pal.bg).inner_margin(Margin {
                left: 24,
                right: 12,
                top: 6,
                bottom: 12,
            }))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    let title = PAGES.iter().find(|p| p.0 == self.page).map_or("", |p| p.1);
                    ui.label(RichText::new(title).size(18.0).strong());
                    chrome::drag_area(ui, ui.available_width() - chrome::controls_width());
                    chrome::window_controls(ui, &pal);
                });
                ui.label(RichText::new(&self.status).weak());
                if let Some(e) = &self.error {
                    ui.colored_label(pal.red, e);
                }
                ui.add_space(6.0);
                match self.page {
                    Page::Maps => self.page_maps(ui, &pal),
                    page => {
                        egui::ScrollArea::vertical()
                            .auto_shrink([false, false])
                            .show(ui, |ui| match page {
                                Page::Vehicle => self.page_vehicle(ui, &pal),
                                Page::Modules => self.page_modules(ui, &pal),
                                _ => self.page_live(ui, &pal),
                            });
                    }
                }
            });
        chrome::outline(&ctx, &pal);
    }
}

fn main() -> eframe::Result {
    let start = match Start::parse(&std::env::args().skip(1).collect::<Vec<_>>()) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("icarus-app: {e}\nusage: icarus-app [--connect] [--adapter A] [--page vehicle|modules|live|maps] [--scan] [--live] [--maps DEF.xdf IMAGE.bin [SECOND.bin]]");
            std::process::exit(2);
        }
    };
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1180.0, 760.0])
            .with_min_inner_size([760.0, 480.0])
            .with_title("Icarus")
            .with_app_id("icarus")
            .with_decorations(false),
        ..Default::default()
    };
    eframe::run_native(
        "Icarus",
        options,
        Box::new(|cc| Ok(Box::new(App::new(&cc.egui_ctx, start)))),
    )
}
