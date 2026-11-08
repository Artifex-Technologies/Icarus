//! Icarus's window: the vehicle (adapter, identity, check-engine light, monitors, codes), its modules (identity and codes,
//! as dealer tools show them), live values with graphs, actuator tests, module settings, learning from another tool, the
//! bus, and maps from an image and a definition (with what a tune changed). Yggdrasil's look (yggdrasil-ui).

mod worker;

use eframe::egui::{self, vec2, Color32, CornerRadius, Margin, RichText, Sense, Stroke};
use icarus_core::def::{fmt_num, Definition, Kind};
use icarus_core::rom::{self, Rom};
use icarus_core::xdf;
use icarus_obd::garage::{self, Described};
use icarus_obd::j1979;
use icarus_obd::uds::Dialect;
use icarus_obd::vehicles::Profile;
use icarus_obd::{dtc, lookup};
use std::collections::{BTreeSet, HashMap, VecDeque};
use std::sync::mpsc::{channel, Receiver, Sender};
use worker::{Cmd, Event, Info, ModuleView};
use yggdrasil_ui::{chrome, theme};

#[derive(Clone, Copy, PartialEq, Eq)]
enum Page {
    Vehicle,
    Modules,
    Live,
    Tests,
    Settings,
    Learn,
    Bus,
    Maps,
    Garage,
    Tools,
}

const PAGES: [(Page, &str); 10] = [
    (Page::Vehicle, "Vehicle"),
    (Page::Modules, "Modules"),
    (Page::Live, "Live data"),
    (Page::Tests, "Tests"),
    (Page::Settings, "Settings"),
    (Page::Learn, "Learning"),
    (Page::Bus, "Bus"),
    (Page::Maps, "Maps"),
    (Page::Garage, "Garage"),
    (Page::Tools, "Tools"),
];

/// Answers from the NHTSA thread.
enum Bg {
    Vin(Result<icarus_obd::nhtsa::VinDecode, String>),
    Recalls(Result<Vec<icarus_obd::nhtsa::Recall>, String>),
    /// Sarina's answer about a learning session.
    Sarina(Result<String, String>),
}

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
    /// The table cell (row, column) being edited.
    cell: Option<(usize, usize)>,
    value_text: String,
    /// Checksum descriptions for this definition (<definition>.checksums.toml), and what loading them said.
    specs: Vec<icarus_core::checksum::Spec>,
    specs_note: String,
    save_path: String,
    save_note: String,
}

/// An edit asked for on the Maps page, applied after drawing.
enum MapEdit {
    Set(f64),
    Step(i64),
    Flag(bool),
    UndoElement,
    UndoAll,
}

/// What to do at start (command line): `--connect` (the adapter from ICARUS_ADAPTER, or `--adapter A`), `--page NAME`,
/// `--scan` and `--live` once connected, `--maps DEF.xdf IMAGE.bin [SECOND.bin]`, `--learn` (a learning watch once
/// connected; `--mark TEXT` marks at its start and `--learn-seconds N` stops it, for demonstrations and screenshots).
#[derive(Default)]
struct Start {
    connect: bool,
    adapter: Option<String>,
    page: Option<Page>,
    scan: bool,
    live: bool,
    bus: bool,
    learn: bool,
    mark: Option<String>,
    learn_secs: Option<f64>,
    maps: Option<(String, String, String)>,
    /// `--map NAME`: the map to show first (with `--cell R,C`, that cell chosen for editing).
    map: Option<String>,
    cell: Option<(usize, usize)>,
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
                "--bus" => s.bus = true,
                "--learn" => s.learn = true,
                "--mark" => {
                    i += 1;
                    s.mark = Some(a.get(i).ok_or("--mark needs a text")?.clone());
                }
                "--learn-seconds" => {
                    i += 1;
                    s.learn_secs = Some(
                        a.get(i)
                            .and_then(|v| v.parse().ok())
                            .ok_or("--learn-seconds needs a number")?,
                    );
                }
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
                "--map" => {
                    i += 1;
                    s.map = Some(a.get(i).ok_or("--map needs a name")?.clone());
                }
                "--cell" => {
                    i += 1;
                    let v = a.get(i).ok_or("--cell needs ROW,COLUMN")?;
                    let (r, c) = v.split_once(',').ok_or("--cell is ROW,COLUMN (from 0)")?;
                    s.cell = Some((
                        r.trim().parse().map_err(|_| "--cell is ROW,COLUMN")?,
                        c.trim().parse().map_err(|_| "--cell is ROW,COLUMN")?,
                    ));
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
    /// The name being typed for this vehicle (garage), and whether to save it.
    name_edit: String,
    /// A report with every module's codes too.
    report_all: bool,
    last_report: Option<String>,
    /// Actuator tests: the percent chosen per test, the one held now, the last thing said, the watched value.
    test_values: HashMap<String, f64>,
    holding: Option<String>,
    /// After a test stopped by itself while the button was still held: the button must be let go before any test starts.
    release_needed: bool,
    test_note: String,
    test_reading: Option<f64>,
    /// A clear waiting for the person's yes: None = every OBD-II module, Some = one module (tx, rx, dialect).
    confirm_clear: Option<(Option<(u16, u16, Dialect)>, String)>,
    selftests: Option<Vec<icarus_obd::mode06::TestResult>>,
    battery_agm: bool,
    /// NHTSA lookups run on their own thread; answers come back here.
    bg_tx: Sender<Bg>,
    bg_rx: Receiver<Bg>,
    nhtsa: Option<icarus_obd::nhtsa::VinDecode>,
    recalls: Option<Vec<icarus_obd::nhtsa::Recall>>,
    asking: bool,
    charging: Option<icarus_obd::charging::Check>,
    /// The bus monitor: the source (adapter or a Linux CAN interface), its last view.
    bus_can: bool,
    bus_ifname: String,
    bus: Option<worker::BusView>,
    /// Settings: each one's value now (from the worker), the value picked per setting, a change waiting for the person's
    /// yes (the setting and the value's name; None = undo), and whether the values were asked for since connecting.
    setting_views: Vec<worker::SettingView>,
    setting_pick: HashMap<String, String>,
    confirm_setting: Option<(icarus_obd::settings::SettingDef, Option<String>)>,
    settings_asked: bool,
    /// Learning: the mark being typed, what the last watch learned (being named here; true once added to the file),
    /// where its session was saved, earlier sessions, and Sarina's answer.
    learn_mark: String,
    learn_tests: Vec<(icarus_obd::actuator::TestDef, bool)>,
    learn_settings: Vec<LearnSetting>,
    learn_notes: Vec<String>,
    learn_saved: Option<Result<String, String>>,
    learn_sessions: Vec<std::path::PathBuf>,
    sarina_busy: bool,
    sarina_answer: Option<(String, Result<String, String>)>,
    /// `--learn-seconds`: when the learning watch started at launch is stopped.
    learn_stop_at: Option<std::time::Instant>,
    /// Vehicle folders: (VIN, name, folder), the one shown, a path to import.
    garage: Vec<(String, Option<String>, std::path::PathBuf)>,
    garage_sel: Option<usize>,
    import_path: String,
    /// Tools: tire sizes and gears, and a speedometer check.
    t_stock: String,
    t_new: String,
    t_gear: String,
    t_installed: String,
    t_speed: String,
    s_gauge: String,
    s_gps: String,
    s_obd: String,
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
        let (bg_tx, bg_rx) = channel::<Bg>();
        let (tx, wrx) = channel();
        let (wtx, rx) = channel();
        let c = ctx.clone();
        std::thread::spawn(move || worker::run(wrx, wtx, move || c.request_repaint()));
        let mut app = App {
            name_edit: String::new(),
            report_all: true,
            last_report: None,
            test_values: HashMap::new(),
            holding: None,
            release_needed: false,
            test_note: String::new(),
            test_reading: None,
            confirm_clear: None,
            selftests: None,
            battery_agm: true,
            bg_tx: bg_tx.clone(),
            bg_rx,
            nhtsa: None,
            recalls: None,
            asking: false,
            charging: None,
            bus_can: false,
            bus_ifname: "can0".into(),
            bus: None,
            setting_views: Vec::new(),
            setting_pick: HashMap::new(),
            confirm_setting: None,
            settings_asked: false,
            learn_mark: String::new(),
            learn_tests: Vec::new(),
            learn_settings: Vec::new(),
            learn_notes: Vec::new(),
            learn_saved: None,
            learn_sessions: Vec::new(),
            sarina_busy: false,
            sarina_answer: None,
            learn_stop_at: None,
            garage: icarus_obd::garage::vehicles(),
            garage_sel: None,
            import_path: String::new(),
            t_stock: "P265/65R17".into(),
            t_new: "LT285/70R17".into(),
            t_gear: "3.91".into(),
            t_installed: "4.88".into(),
            t_speed: "65".into(),
            s_gauge: String::new(),
            s_gps: String::new(),
            s_obd: String::new(),
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
                cell: None,
                value_text: String::new(),
                specs: Vec::new(),
                specs_note: String::new(),
                save_path: String::new(),
                save_note: String::new(),
            },
            start,
        };
        if let Some((d, b, c)) = app.start.maps.take() {
            (app.maps.def_path, app.maps.bin_path, app.maps.cmp_path) = (d, b, c);
            app.load_maps();
            if let (Some(name), Some(def)) = (app.start.map.take(), app.maps.def.as_ref()) {
                let want = name.to_lowercase();
                app.maps.selected = def
                    .elements
                    .iter()
                    .position(|e| e.title.to_lowercase().contains(&want));
                if let (Some(i), Some((r, c)), Some(rom)) =
                    (app.maps.selected, app.start.cell, app.maps.rom.as_ref())
                {
                    if let Kind::Table(t) = &def.elements[i].kind {
                        if let Ok(v) = def.values(rom, &t.z) {
                            if let Some(n) = v.get(r).and_then(|row| row.get(c)) {
                                app.maps.cell = Some((r, c));
                                app.maps.value_text = fmt_num(*n, t.z.decimals);
                            }
                        }
                    }
                }
            }
        }
        if app.start.connect {
            app.link = Link::Busy;
            let _ = app.tx.send(Cmd::Connect(app.adapter.trim().to_string()));
        }
        app
    }

    fn background(&mut self) {
        while let Ok(b) = self.bg_rx.try_recv() {
            self.asking = false;
            match b {
                Bg::Vin(Ok(d)) => {
                    self.name_edit = d.name();
                    self.status = "NHTSA's answer is in the name field: Save keeps it.".into();
                    self.nhtsa = Some(d);
                }
                Bg::Recalls(Ok(r)) => {
                    self.status = format!("{} recall(s) on record.", r.len());
                    self.recalls = Some(r);
                }
                Bg::Vin(Err(e)) | Bg::Recalls(Err(e)) => self.error = Some(e),
                Bg::Sarina(r) => {
                    self.sarina_busy = false;
                    self.status = if r.is_ok() {
                        "Sarina answered.".into()
                    } else {
                        "Sarina did not answer.".into()
                    };
                    if let Some(a) = self.sarina_answer.as_mut() {
                        a.1 = r;
                    }
                }
            }
        }
    }

    fn ask_nhtsa(&mut self, ctx: &egui::Context, job: impl FnOnce() -> Bg + Send + 'static) {
        self.asking = true;
        let (tx, c) = (self.bg_tx.clone(), ctx.clone());
        std::thread::spawn(move || {
            let _ = tx.send(job());
            c.request_repaint();
        });
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
                    // a name to start from: the saved one, else the model year and the make from the VIN
                    self.name_edit = info.vehicle.as_ref().map(|v| v.name()).unwrap_or_else(|| {
                        let year = info
                            .vin_info
                            .as_ref()
                            .and_then(|v| v.year)
                            .map(|y| y.to_string())
                            .unwrap_or_default();
                        let make = info
                            .profile
                            .as_ref()
                            .map(|p| p.name.split([' ', '/']).next().unwrap_or("").to_string())
                            .unwrap_or_default();
                        format!("{year} {make} ").trim_start().to_string()
                    });
                    if let Some(v) = info.vin.as_deref() {
                        if icarus_obd::garage::ensure(v).is_ok() {
                            self.garage = icarus_obd::garage::vehicles();
                        }
                    }
                    self.info = Some(*info);
                    self.modules.clear();
                    self.history.clear();
                    self.selftests = None;
                    self.setting_views.clear();
                    self.settings_asked = false;
                    self.refresh_sessions();
                    if std::mem::take(&mut self.start.scan) {
                        self.scanning = true;
                        let _ = self.tx.send(Cmd::Scan {
                            profile: None,
                            sweep: false,
                        });
                    }
                    if std::mem::take(&mut self.start.bus) {
                        let _ = self.tx.send(Cmd::BusStart(None, false));
                    }
                    if std::mem::take(&mut self.start.learn) {
                        let _ = self.tx.send(Cmd::BusStart(None, true));
                        if let Some(m) = self.start.mark.take() {
                            let _ = self.tx.send(Cmd::BusMark(m));
                        }
                        self.learn_stop_at = self.start.learn_secs.map(|s| {
                            std::time::Instant::now() + std::time::Duration::from_secs_f64(s)
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
                Event::TestStarted(name) => {
                    self.test_note = format!("Running: {name}. Let go to stop.");
                    self.test_reading = None;
                }
                Event::TestStopped { name, why } => {
                    self.test_note = format!("{name}: stopped ({why}).");
                    self.holding = None;
                    self.release_needed = true;
                }
                Event::TestReading(v) => self.test_reading = Some(v),
                Event::Bus(v) => {
                    if let Some(e) = &v.ended {
                        self.status = format!("Bus: {e}.");
                        // what was learned is named on the Learning page
                        self.learn_tests = v.learned.iter().map(|t| (t.clone(), false)).collect();
                        self.learn_settings = v.settings.iter().map(LearnSetting::new).collect();
                        self.learn_notes = v.notes.clone();
                        if v.learning {
                            self.learn_saved = v.saved.clone();
                            self.refresh_sessions();
                        }
                    }
                    self.bus = Some(*v);
                }
                Event::Settings(v) => self.setting_views = v,
                Event::SettingDone(r) => match r {
                    Ok(m) => {
                        self.status = m;
                        self.error = None;
                    }
                    Err(m) => {
                        self.status = "The setting was not changed.".into();
                        self.error = Some(m);
                    }
                },
                Event::Charging(c) => {
                    self.status = "Battery and charging measured.".into();
                    self.charging = Some(c);
                }
                Event::SelfTests(r) => {
                    self.status = format!("{} on-board test results read.", r.len());
                    self.selftests = Some(r);
                }
                Event::Cleared(msg, info) => {
                    self.status = msg;
                    self.error = None;
                    self.info = Some(*info);
                    self.modules.clear();
                    self.scan_note =
                        "Codes were cleared: scan again to see the modules now.".into();
                }
                Event::ReportSaved(p) => {
                    self.status = format!("Report saved: {p}");
                    self.last_report = Some(p);
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
        let mut links: Vec<String> = Vec::new();
        let mut ask_clear = false;
        let mut ask_vin = false;
        let mut ask_recalls = false;
        let mut read_selftests = false;
        let mut measure = false;
        let mut save_name = false;
        let mut report = false;
        card(ui, pal, "This vehicle's name", |ui| {
            ui.label(RichText::new("A VIN gives the year and make, not the model: name it once, and code searches and parts links use it.").weak());
            ui.horizontal(|ui| {
                ui.add(
                    egui::TextEdit::singleline(&mut self.name_edit)
                        .hint_text("2017 Toyota Tacoma 3.5L V6")
                        .desired_width(320.0),
                );
                if ui
                    .add_enabled(info.vin.is_some(), egui::Button::new("Save"))
                    .clicked()
                {
                    save_name = true;
                }
                if ui.add_enabled(info.vin.is_some() && !self.asking, egui::Button::new("Fill in from NHTSA")).on_hover_text("Asks NHTSA (US Department of Transportation) for the model, engine and drive: this sends the VIN to NHTSA").clicked() {
                    ask_vin = true;
                }
                if let Some(url) = info.vehicle.as_ref().and_then(lookup::parts_catalog) {
                    if ui.button("Parts catalog").on_hover_text(&url).clicked() {
                        links.push(url);
                    }
                }
            });
        });
        card(ui, pal, "Trouble codes (OBD-II)", |ui| {
            if info.codes.is_empty() {
                ui.label(RichText::new("None stored, pending or permanent.").color(pal.green));
            }
            for (kind, module, codes) in &info.codes {
                ui.label(RichText::new(format!("{kind}: {module}")).weak());
                for c in codes {
                    let col = if *kind == "pending" {
                        pal.amber
                    } else {
                        pal.red
                    };
                    code_row(
                        ui,
                        pal,
                        c,
                        kind,
                        col,
                        info.profile.as_ref(),
                        info.vehicle.as_ref(),
                        &mut links,
                    );
                }
            }
            ui.label(
                RichText::new(
                    "The Modules page reads every module's own codes (ABS, airbags, body...).",
                )
                .weak(),
            );
            if !info.codes.is_empty() && ui.button("Clear codes...").clicked() {
                ask_clear = true;
            }
        });
        for f in &info.freeze {
            let title = format!("Freeze frame: {} ({})", f.module, f.from);
            card(ui, pal, &title, |ui| {
                ui.label(
                    RichText::new(format!(
                        "What the engine was doing when {} set the check-engine light:",
                        f.code.as_deref().unwrap_or("the code")
                    ))
                    .weak(),
                );
                egui::Grid::new(("ff", f.from.as_str()))
                    .num_columns(2)
                    .spacing(vec2(16.0, 4.0))
                    .show(ui, |ui| {
                        for (p, v) in &f.values {
                            if let Some(x) = j1979::pid(*p) {
                                row(ui, x.name, &format!("{v:.1} {}", x.unit));
                            }
                        }
                    });
            });
        }
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
        if let Some(d) = &self.nhtsa {
            card(ui, pal, "NHTSA's record for this VIN", |ui| {
                egui::Grid::new("vpic")
                    .num_columns(2)
                    .spacing(vec2(16.0, 4.0))
                    .show(ui, |ui| {
                        for (k, v) in [
                            ("Model", d.name()),
                            ("Trim", d.trim.clone()),
                            ("Drive", d.drive.clone()),
                            ("Body", d.body.clone()),
                            ("Fuel", d.fuel.clone()),
                            ("Built in", d.plant_country.clone()),
                        ] {
                            if !v.is_empty() {
                                row(ui, k, &v);
                            }
                        }
                    });
                if let Some(n) = &d.note {
                    ui.label(RichText::new(n).small().weak());
                }
            });
        }
        card(ui, pal, "Recalls", |ui| {
            let named = info
                .vehicle
                .as_ref()
                .filter(|v| v.year.is_some() && !v.make.is_empty() && !v.model.is_empty());
            ui.label(RichText::new("Recalls on record at NHTSA for this year, make and model (only those three are sent).").weak());
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(
                        named.is_some() && !self.asking,
                        egui::Button::new("Look up recalls"),
                    )
                    .clicked()
                {
                    ask_recalls = true;
                }
                if ui
                    .button("Check this VIN at NHTSA")
                    .on_hover_text(
                        "Whether this vehicle is affected and still open (opens nhtsa.gov)",
                    )
                    .clicked()
                {
                    links.push(icarus_obd::nhtsa::VIN_RECALL_CHECK.to_string());
                }
                if named.is_none() {
                    ui.label(
                        RichText::new("Name the vehicle first (year make model).")
                            .small()
                            .weak(),
                    );
                }
            });
            if let Some(list) = &self.recalls {
                if list.is_empty() {
                    ui.label(RichText::new("None on record.").color(pal.green));
                }
                for r in list {
                    ui.add_space(4.0);
                    ui.label(
                        RichText::new(format!("{} ({}) {}", r.campaign, r.date, r.component))
                            .strong(),
                    );
                    ui.label(&r.summary);
                    if !r.remedy.is_empty() {
                        ui.label(RichText::new(format!("Remedy: {}", r.remedy)).weak());
                    }
                }
            }
        });
        card(ui, pal, "Battery and charging", |ui| {
            ui.label(RichText::new("About 20 seconds of the voltage the engine computer sees. Engine running: is it charging this battery well. Engine off, after the vehicle has sat an hour: how charged it is.").weak());
            ui.horizontal(|ui| {
                ui.radio_value(&mut self.battery_agm, true, "AGM");
                ui.radio_value(&mut self.battery_agm, false, "flooded");
                if ui.button("Measure").clicked() {
                    measure = true;
                }
            });
            if let Some(c) = &self.charging {
                ui.label(RichText::new(&c.verdict).color(if c.ok { pal.text } else { pal.amber }));
                ui.label(
                    RichText::new(format!(
                        "{} samples, {:.2} V average ({:.2} to {:.2}), engine {}",
                        c.samples,
                        c.avg,
                        c.min,
                        c.max,
                        if c.running { "running" } else { "off" }
                    ))
                    .small()
                    .weak(),
                );
            }
        });
        card(ui, pal, "On-board test results (Mode 06)", |ui| {
            ui.label(RichText::new("Each emissions monitor's last measurement against its limits: a value near a limit shows a part on its way out before a code sets; misfire counts per cylinder.").weak());
            if ui
                .button(if self.selftests.is_some() {
                    "Read again"
                } else {
                    "Read"
                })
                .clicked()
            {
                read_selftests = true;
            }
            if let Some(r) = &self.selftests {
                if r.is_empty() {
                    ui.label("The engine module gave none.");
                }
                let mut last = None;
                for t in r {
                    if last != Some(t.mid) {
                        ui.add_space(4.0);
                        ui.label(RichText::new(icarus_obd::mode06::monitor_name(t.mid)).strong());
                        last = Some(t.mid);
                    }
                    ui.horizontal_wrapped(|ui| {
                        ui.label(
                            RichText::new(if t.passed() { "pass" } else { "FAIL" })
                                .strong()
                                .color(if t.passed() { pal.green } else { pal.red }),
                        );
                        ui.label(icarus_obd::mode06::test_name(t.mid, t.tid));
                        ui.label(RichText::new(icarus_obd::report::selftest_values(t)).weak());
                    });
                }
            }
        });
        card(ui, pal, "Report", |ui| {
            ui.label(RichText::new("Everything above (and, if chosen, every module's codes) in one file to keep or show a mechanic, with search and parts links. Saved in ~/Documents/Icarus/reports.").weak());
            ui.horizontal(|ui| {
                ui.checkbox(&mut self.report_all, "with every module");
                if ui.button("Save report").clicked() {
                    report = true;
                }
                if let Some(p) = &self.last_report {
                    if ui.button("Open the last one").on_hover_text(p).clicked() {
                        let _ = std::process::Command::new("xdg-open").arg(p).spawn();
                    }
                }
            });
        });
        if let Some(p) = &info.profile {
            if !p.notes.is_empty() {
                card(ui, pal, "About this make", |ui| {
                    ui.label(p.notes.split_whitespace().collect::<Vec<_>>().join(" "));
                });
            }
        }
        let vin = info.vin.clone();
        self.open_links(links);
        if save_name {
            if let Some(v) = vin.clone() {
                match garage::set(&v, &self.name_edit) {
                    Ok(()) => {
                        if let Some(i) = self.info.as_mut() {
                            i.vehicle = Some(Described::parse(&self.name_edit))
                                .filter(|d| !d.name().is_empty());
                        }
                        self.status = "Name saved.".into();
                    }
                    Err(e) => self.error = Some(e),
                }
            }
        }
        if report {
            self.live_on = false;
            let _ = self.tx.send(Cmd::Report(self.report_all));
        }
        if ask_vin {
            if let Some(v) = vin.clone() {
                self.ask_nhtsa(ui.ctx(), move || Bg::Vin(icarus_obd::nhtsa::decode_vin(&v)));
            }
        }
        if ask_recalls {
            if let Some(v) = self.info.as_ref().and_then(|i| i.vehicle.clone()) {
                if let Some(y) = v.year {
                    self.ask_nhtsa(ui.ctx(), move || {
                        Bg::Recalls(icarus_obd::nhtsa::recalls(&v.make, &v.model, y))
                    });
                }
            }
        }
        if measure {
            self.live_on = false;
            let b = if self.battery_agm {
                icarus_obd::charging::Battery::Agm
            } else {
                icarus_obd::charging::Battery::Flooded
            };
            let _ = self.tx.send(Cmd::Charging(20, b));
        }
        if read_selftests {
            self.live_on = false;
            let _ = self.tx.send(Cmd::SelfTests);
        }
        if ask_clear {
            self.confirm_clear =
                Some((None, "every engine and transmission module (OBD-II)".into()));
        }
    }

    fn open_links(&mut self, links: Vec<String>) {
        for url in links {
            if let Err(e) = lookup::open(&url) {
                self.error = Some(format!("could not open the browser: {e}"));
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
        let mut links: Vec<String> = Vec::new();
        let all = icarus_obd::vehicles::load(icarus_obd::vehicles::user_dir().as_deref()).0;
        let chosen = icarus_obd::vehicles::find(&all, &self.profile_choice)
            .ok()
            .filter(|_| !self.profile_choice.is_empty());
        let profile = chosen.or(self.info.as_ref().and_then(|i| i.profile.as_ref()));
        let vehicle = self.info.as_ref().and_then(|i| i.vehicle.as_ref());
        let mut ask_clear: Option<(u16, u16, Dialect, String)> = None;
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
                                .map(|f| format!(" -{f:02X}"))
                                .unwrap_or_default();
                            let extra = format!("{}{ftb}, {lang}", c.states.join(", "));
                            code_row(
                                ui,
                                pal,
                                &c.code,
                                &extra,
                                if confirmed { pal.red } else { pal.amber },
                                profile,
                                vehicle,
                                &mut links,
                            );
                        }
                        if !codes.is_empty() && ui.button("Clear this module's codes...").clicked()
                        {
                            ask_clear = Some((
                                m.addr.tx,
                                m.addr.rx(),
                                *d,
                                format!("{:03X} {}", m.addr.tx, m.addr.name),
                            ));
                        }
                    }
                }
            });
        }
        if let Some((tx, rx, d, label)) = ask_clear {
            self.confirm_clear = Some((Some((tx, rx, d)), label));
        }
        if !self.unreachable.is_empty() {
            card(ui, pal, "Not reachable yet", |ui| {
                for u in &self.unreachable {
                    ui.label(u);
                }
            });
        }
        self.open_links(links);
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

    fn page_tests(&mut self, ui: &mut egui::Ui, pal: &theme::Palette) {
        let Some(info) = &self.info else {
            ui.label("Connect on the Vehicle page first. (The simulated vehicle with its engine off is sim:off.)");
            return;
        };
        let tests = info.tests.clone();
        card(ui, pal, "Before you test", |ui| {
            ui.label("Actuator tests make the vehicle move parts: fans, throttle, pumps, valves. Park it: in Park (or neutral with the parking brake on), wheels chocked for brake and ABS tests, people and hands clear.");
            ui.label("Hold the button to run a test. Letting go stops it at once and hands control back to the module; so does a time limit, a broken condition or a lost connection. Every test is logged in ~/.local/state/icarus/actions.log.");
        });
        if !self.test_note.is_empty() {
            ui.label(
                RichText::new(&self.test_note)
                    .strong()
                    .color(if self.holding.is_some() {
                        pal.amber
                    } else {
                        pal.text
                    }),
            );
            ui.add_space(4.0);
        }
        if tests.is_empty() {
            card(ui, pal, "No tests for this vehicle yet", |ui| {
                ui.label("The IDs behind each test (which output, which module) are each maker's own and often per model. A vehicle's tests go in its profile as [[test]] entries once they are known: from the maker's service information, or by watching what another scan tool sends while it runs the test.");
            });
            return;
        }
        let mut press: Option<(icarus_obd::actuator::TestDef, f64)> = None;
        let mut held_now: Option<String> = None;
        for t in &tests {
            card(ui, pal, &t.name, |ui| {
                ui.label(
                    RichText::new(format!(
                        "module {:03X}, {}, up to {} s{}",
                        t.module,
                        t.protocol.label(),
                        t.max_seconds,
                        if t.checked {
                            ""
                        } else {
                            " · IDs not yet checked on a vehicle"
                        }
                    ))
                    .small()
                    .weak(),
                );
                if !t.conditions.is_empty() {
                    ui.label(format!(
                        "Needs: {}",
                        t.conditions
                            .iter()
                            .map(|c| c.describe())
                            .collect::<Vec<_>>()
                            .join(", ")
                    ));
                }
                if !t.warning.is_empty() {
                    ui.label(RichText::new(&t.warning).color(pal.amber));
                }
                let value = self.test_values.entry(t.name.clone()).or_insert(25.0);
                ui.horizontal(|ui| {
                    ui.visuals_mut().widgets.inactive.bg_fill = pal.bg;
                    ui.visuals_mut().widgets.inactive.weak_bg_fill = pal.bg;
                    if t.control == icarus_obd::actuator::Control::Percent {
                        ui.add_enabled(
                            self.holding.is_none(),
                            egui::Slider::new(value, 0.0..=100.0).suffix(" %").integer(),
                        );
                    }
                    let mine = self.holding.as_deref() == Some(t.name.as_str());
                    let label = if mine {
                        "Running: let go to stop"
                    } else {
                        "Hold to run"
                    };
                    let b = ui.add_enabled(
                        self.holding.is_none() || mine,
                        egui::Button::new(RichText::new(label).strong().color(pal.on_accent()))
                            .fill(if mine { pal.amber } else { pal.accent })
                            .min_size(vec2(190.0, 34.0))
                            .sense(Sense::click_and_drag()),
                    );
                    if b.is_pointer_button_down_on() {
                        held_now = Some(t.name.clone());
                        if self.holding.is_none() {
                            press = Some((t.clone(), *value));
                        }
                    }
                    if mine {
                        if let (Some(w), Some(v)) =
                            (t.watch.and_then(j1979::pid), self.test_reading)
                        {
                            ui.label(
                                RichText::new(format!("{}: {v:.1} {}", w.name, w.unit)).strong(),
                            );
                        }
                    }
                });
            });
        }
        // start on press, keep alive while held, stop on release; never restart without a release in between
        if held_now.is_none() {
            self.release_needed = false;
        }
        match (&self.holding, held_now) {
            (None, Some(_)) if self.release_needed => {}
            (None, Some(_)) => {
                if let Some((def, value)) = press {
                    self.holding = Some(def.name.clone());
                    self.live_on = false;
                    self.test_note = format!("Starting: {}...", def.name);
                    let _ = self.tx.send(Cmd::TestStart { def, value });
                }
            }
            (Some(_), Some(_)) => {
                let _ = self.tx.send(Cmd::TestHold);
                ui.ctx()
                    .request_repaint_after(std::time::Duration::from_millis(60));
            }
            (Some(_), None) => {
                let _ = self.tx.send(Cmd::TestStop);
                self.holding = None;
            }
            (None, None) => {}
        }
    }

    fn page_bus(&mut self, ui: &mut egui::Ui, pal: &theme::Palette) {
        let running = self.bus.as_ref().is_some_and(|b| b.running);
        ui.horizontal(|ui| {
            ui.add_enabled_ui(!running, |ui| {
                ui.radio_value(&mut self.bus_can, false, "the connected adapter");
                ui.radio_value(&mut self.bus_can, true, "a Linux CAN interface");
                if self.bus_can {
                    ui.add(egui::TextEdit::singleline(&mut self.bus_ifname).desired_width(80.0));
                }
            });
            if running {
                if ui.button("Stop").clicked() {
                    let _ = self.tx.send(Cmd::BusStop);
                }
                ui.spinner();
            } else if ui.button("Watch").clicked() {
                self.live_on = false;
                self.bus = None;
                let _ = self.tx.send(Cmd::BusStart(
                    self.bus_can.then(|| self.bus_ifname.trim().to_string()),
                    false,
                ));
            }
        });
        ui.label(RichText::new("Watching only: nothing is sent onto the bus. Press a pedal or a switch and see which byte follows (changed bytes light up). Diagnostic messages are put in words; actuator tests another tool runs, and settings it changes, are learned.").weak());
        let Some(b) = &self.bus else { return };
        ui.label(
            RichText::new(format!(
                "{} frames, {} IDs{}",
                b.frames,
                b.rows.len(),
                b.ended
                    .as_ref()
                    .map(|e| format!(" ({e})"))
                    .unwrap_or_default()
            ))
            .weak(),
        );
        ui.columns(2, |cols| {
            egui::ScrollArea::vertical()
                .id_salt("bus-ids")
                .max_height(420.0)
                .show(&mut cols[0], |ui| {
                    egui::Grid::new("bus-table")
                        .striped(true)
                        .spacing(vec2(12.0, 2.0))
                        .show(ui, |ui| {
                            for h in ["ID", "Hz", "count", "bytes"] {
                                ui.label(RichText::new(h).small().weak());
                            }
                            ui.end_row();
                            for (id, count, hz, data, ages) in &b.rows {
                                ui.label(
                                    RichText::new(if *id > 0x7FF {
                                        format!("{id:08X}")
                                    } else {
                                        format!("{id:03X}")
                                    })
                                    .monospace(),
                                );
                                ui.label(RichText::new(format!("{hz:.1}")).monospace());
                                ui.label(RichText::new(count.to_string()).monospace());
                                ui.horizontal(|ui| {
                                    ui.spacing_mut().item_spacing.x = 4.0;
                                    for (i, byte) in data.iter().enumerate() {
                                        let age = ages.get(i).copied().unwrap_or(f64::INFINITY);
                                        let col = if age < 1.0 { pal.amber } else { pal.text };
                                        ui.label(
                                            RichText::new(format!("{byte:02X}"))
                                                .monospace()
                                                .color(col),
                                        );
                                    }
                                });
                                ui.end_row();
                            }
                        });
                });
            egui::ScrollArea::vertical()
                .id_salt("bus-log")
                .max_height(420.0)
                .stick_to_bottom(true)
                .show(&mut cols[1], |ui| {
                    if b.log.is_empty() {
                        ui.label(RichText::new("No diagnostic messages yet.").weak());
                    }
                    for l in &b.log {
                        ui.label(RichText::new(l).monospace().small());
                    }
                });
        });
        if !b.learned.is_empty() || !b.settings.is_empty() {
            ui.add_space(6.0);
            ui.label(format!(
                "Learned while watching: {} test(s), {} setting(s). The Learning page names them and adds them to the vehicle's file.",
                b.learned.len(),
                b.settings.len()
            ));
        }
        if running {
            ui.ctx()
                .request_repaint_after(std::time::Duration::from_millis(250));
        }
    }

    fn refresh_sessions(&mut self) {
        let vin = self.info.as_ref().and_then(|i| i.vin.clone());
        self.learn_sessions = icarus_obd::garage::logs_dir(vin.as_deref())
            .map(|d| icarus_obd::learning::sessions(&d))
            .unwrap_or_default();
    }

    fn ask_sarina(&mut self, ctx: &egui::Context, file: std::path::PathBuf) {
        self.sarina_busy = true;
        self.status = "Sarina is reading the session (she only reads)...".into();
        let name = file
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("")
            .to_string();
        self.sarina_answer = Some((name, Err("asking...".into())));
        let (tx, c) = (self.bg_tx.clone(), ctx.clone());
        std::thread::spawn(move || {
            let _ = tx.send(Bg::Sarina(icarus_obd::learning::ask_sarina(&file)));
            c.request_repaint();
        });
    }

    fn page_settings(&mut self, ui: &mut egui::Ui, pal: &theme::Palette) {
        let Some(info) = &self.info else {
            ui.label("Connect on the Vehicle page first. (The simulated vehicle with its engine off is sim:off.)");
            return;
        };
        let defs = info.settings.clone();
        let wired = icarus_obd::port::Target::parse(self.adapter.trim()).is_ok_and(|t| t.wired());
        if !defs.is_empty() && !self.settings_asked {
            self.settings_asked = true;
            let _ = self.tx.send(Cmd::SettingsRead);
        }
        card(ui, pal, "Module settings", |ui| {
            ui.label("What dealer tools call customizations: a feature a module keeps switched on or off, or a value it keeps (a chime, a lock behaviour, a light).");
            ui.label("A change writes the module's memory, so Icarus asks first, needs the conditions (by default engine off, parked) and a wired adapter, keeps the value there before in the vehicle's Logs folder, reads the new one back, and puts the old one back if the module did not take it. Never airbags, immobiliser, keys or odometer.");
        });
        if !wired {
            ui.colored_label(pal.amber, "This adapter is wireless: settings can be read, but changing one needs a wired adapter (USB).");
        }
        if defs.is_empty() {
            card(ui, pal, "No settings for this vehicle yet", |ui| {
                ui.label("Each maker's settings are its own, and they differ by model and year, so Icarus learns them from your vehicle: the Learning page watches another tool change one and names it.");
            });
            return;
        }
        ui.horizontal(|ui| {
            if ui.button("Read the values again").clicked() {
                let _ = self.tx.send(Cmd::SettingsRead);
            }
        });
        ui.add_space(4.0);
        let mut ask: Option<(icarus_obd::settings::SettingDef, Option<String>)> = None;
        for d in &defs {
            let view = self.setting_views.iter().find(|v| v.name == d.name);
            let now = view
                .map(|v| v.now.clone())
                .unwrap_or_else(|| "not read yet".into());
            // the value offered first is one it does not have now (chosen once its value is known)
            let mut unread = d.values.keys().next().cloned().unwrap_or_default();
            let pick = if view.is_some() {
                self.setting_pick.entry(d.name.clone()).or_insert_with(|| {
                    d.values
                        .keys()
                        .find(|k| **k != now)
                        .cloned()
                        .unwrap_or_default()
                })
            } else {
                &mut unread
            };
            card(ui, pal, &d.name, |ui| {
                ui.label(
                    RichText::new(format!(
                        "module {:03X}, {}{}",
                        d.module,
                        d.protocol.label(),
                        if d.checked {
                            ""
                        } else {
                            ", not yet checked on a vehicle"
                        }
                    ))
                    .small()
                    .weak(),
                );
                if !d.note.is_empty() {
                    ui.label(RichText::new(&d.note).small().weak());
                }
                ui.horizontal(|ui| {
                    ui.label("Now:");
                    ui.label(RichText::new(&now).strong());
                    ui.add_space(12.0);
                    egui::ComboBox::from_id_salt(("setting-pick", &d.name))
                        .selected_text(pick.as_str())
                        .show_ui(ui, |ui| {
                            for k in d.values.keys() {
                                ui.selectable_value(pick, k.clone(), k);
                            }
                        });
                    if ui
                        .add_enabled(
                            wired && *pick != now && !pick.is_empty(),
                            egui::Button::new("Change..."),
                        )
                        .clicked()
                    {
                        ask = Some((d.clone(), Some(pick.clone())));
                    }
                    if let Some(back) = view.and_then(|v| v.undo_to.as_ref()) {
                        if ui
                            .add_enabled(
                                wired && *back != now,
                                egui::Button::new(format!("Undo the last change (back to {back})")),
                            )
                            .clicked()
                        {
                            ask = Some((d.clone(), None));
                        }
                    }
                });
            });
        }
        if ask.is_some() {
            self.confirm_setting = ask;
        }
    }

    fn page_learn(&mut self, ui: &mut egui::Ui, pal: &theme::Palette) {
        let running = self.bus.as_ref().is_some_and(|b| b.running && b.learning);
        let other_watch = self.bus.as_ref().is_some_and(|b| b.running && !b.learning);
        card(ui, pal, "Learning from another tool", |ui| {
            ui.label("Icarus learns a vehicle's settings and output tests by watching another tool use them on it: a dealer tool, a shop scan tool, a phone app that does customizations. It only listens; the other tool does the work.");
            ui.label("1. A Y-splitter on the OBD port: the other tool on one side, Icarus's adapter on the other (or a CAN interface tapped into the bus). Ignition on, engine off.");
            ui.label("2. Start learning. Before each change in the other tool, type what you are about to do (\"seat belt chime off\") and press Mark: what the module accepts next is named by it. One change per mark works best.");
            ui.label("3. Stop. Name the values, add each to the vehicle's file; the Settings and Tests pages then offer them. The session is saved in the vehicle's Logs folder, and Sarina can read it and say what she makes of anything Icarus could not work out.");
        });
        ui.horizontal(|ui| {
            ui.add_enabled_ui(!running && !other_watch, |ui| {
                ui.radio_value(&mut self.bus_can, false, "the connected adapter");
                ui.radio_value(&mut self.bus_can, true, "a Linux CAN interface");
                if self.bus_can {
                    ui.add(egui::TextEdit::singleline(&mut self.bus_ifname).desired_width(80.0));
                }
            });
            if running {
                if ui.button("Stop and save").clicked() {
                    let _ = self.tx.send(Cmd::BusStop);
                }
                ui.spinner();
            } else if ui
                .add_enabled(
                    !other_watch,
                    egui::Button::new(RichText::new("Start learning").color(pal.on_accent()))
                        .fill(pal.accent),
                )
                .clicked()
            {
                self.live_on = false;
                self.bus = None;
                self.learn_saved = None;
                self.sarina_answer = None;
                let _ = self.tx.send(Cmd::BusStart(
                    self.bus_can.then(|| self.bus_ifname.trim().to_string()),
                    true,
                ));
            }
        });
        if other_watch {
            ui.label(RichText::new("The Bus page is watching: stop it there first.").weak());
        }
        if running {
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label("About to do:");
                let r = ui.add(
                    egui::TextEdit::singleline(&mut self.learn_mark)
                        .hint_text("seat belt chime off")
                        .desired_width(320.0),
                );
                let enter = r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                if (ui.button("Mark").clicked() || enter) && !self.learn_mark.trim().is_empty() {
                    let _ = self
                        .tx
                        .send(Cmd::BusMark(std::mem::take(&mut self.learn_mark)));
                    r.request_focus();
                }
            });
        }
        if let Some(b) = &self.bus {
            if b.learning {
                ui.label(
                    RichText::new(format!(
                        "{} frames; learned so far: {} setting(s), {} test(s); {} thing(s) seen but not learned",
                        b.frames,
                        b.settings.len(),
                        b.learned.len(),
                        b.notes.len()
                    ))
                    .weak(),
                );
                egui::ScrollArea::vertical()
                    .id_salt("learn-log")
                    .max_height(220.0)
                    .stick_to_bottom(true)
                    .show(ui, |ui| {
                        for l in b.log.iter().filter(|l| !l.contains(" 7DF ")) {
                            let mark = l.contains("-- mark:");
                            ui.label(RichText::new(l).monospace().small().color(if mark {
                                pal.accent
                            } else {
                                pal.text
                            }));
                        }
                    });
            }
        }
        if running {
            ui.ctx()
                .request_repaint_after(std::time::Duration::from_millis(250));
            return;
        }
        match &self.learn_saved {
            Some(Ok(p)) => {
                ui.add_space(6.0);
                ui.label(format!("Session saved: {p}"));
            }
            Some(Err(e)) => {
                ui.colored_label(pal.red, format!("The session could not be saved: {e}"));
            }
            None => {}
        }
        let profile = self
            .info
            .as_ref()
            .and_then(|i| i.profile.as_ref())
            .map(|p| (p.id.clone(), p.name.clone()));
        let add_label = profile.as_ref().map_or(
            "Connect to a vehicle with a make profile to add it".to_string(),
            |p| format!("Add to the {} file", p.1),
        );
        let mut add_setting: Option<usize> = None;
        let mut add_test: Option<usize> = None;
        for (i, ls) in self.learn_settings.iter_mut().enumerate() {
            card(
                ui,
                pal,
                &format!("Setting {:X} on module {:03X}", ls.def.id, ls.def.module),
                |ui| {
                    ui.label(RichText::new("The other tool changed this and the module took it. \"was\" is what it read just before. Name the setting and its values; it is added unchecked, to be changed only parked with the engine off.").weak());
                    ui.add_enabled_ui(!ls.added, |ui| {
                        ui.horizontal(|ui| {
                            ui.label("Name");
                            ui.add(
                                egui::TextEdit::singleline(&mut ls.def.name).desired_width(360.0),
                            );
                        });
                        for (old, new) in ls.names.iter_mut() {
                            ui.horizontal(|ui| {
                                let bytes = ls.def.values[old.as_str()]
                                    .iter()
                                    .map(|b| format!("{b:02X}"))
                                    .collect::<Vec<_>>()
                                    .join(" ");
                                ui.label(RichText::new(format!("{bytes:>8}")).monospace());
                                ui.add(egui::TextEdit::singleline(new).desired_width(160.0));
                                if old != new {
                                    ui.label(
                                        RichText::new(format!("(was {old:?})")).small().weak(),
                                    );
                                }
                            });
                        }
                        ui.checkbox(
                            &mut ls.def.extended,
                            "needs the extended session (seen: the tool opened it first)",
                        );
                    });
                    if ls.added {
                        ui.colored_label(
                            pal.green,
                            "In the vehicle's file: the Settings page offers it.",
                        );
                    } else if ui
                        .add_enabled(profile.is_some(), egui::Button::new(add_label.as_str()))
                        .clicked()
                    {
                        add_setting = Some(i);
                    }
                },
            );
        }
        for (i, (t, added)) in self.learn_tests.iter_mut().enumerate() {
            card(
                ui,
                pal,
                &format!("Output test {:X} on module {:03X}", t.id, t.module),
                |ui| {
                    ui.label(RichText::new("The other tool ran this output test. Name it after what it drives; it is added unchecked, with the most cautious conditions.").weak());
                    ui.add_enabled_ui(!*added, |ui| {
                        ui.horizontal(|ui| {
                            ui.label("Name");
                            ui.add(egui::TextEdit::singleline(&mut t.name).desired_width(360.0));
                        });
                    });
                    if *added {
                        ui.colored_label(
                            pal.green,
                            "In the vehicle's file: the Tests page offers it.",
                        );
                    } else if ui
                        .add_enabled(profile.is_some(), egui::Button::new(add_label.as_str()))
                        .clicked()
                    {
                        add_test = Some(i);
                    }
                },
            );
        }
        if !self.learn_notes.is_empty() {
            card(ui, pal, "Seen but not learned", |ui| {
                ui.label(RichText::new("What the other tool did that Icarus does not make into a setting or a test. Sarina can say what she makes of it.").weak());
                for n in &self.learn_notes {
                    ui.label(format!("• {n}"));
                }
            });
        }
        if let Some(i) = add_setting {
            let id = profile.as_ref().map(|p| p.0.clone()).unwrap_or_default();
            match self.learn_settings[i].named() {
                Err(e) => self.error = Some(e),
                Ok(d) => match icarus_obd::vehicles::add_setting_to_user_file(
                    &id,
                    &icarus_obd::settings::to_toml(&d),
                ) {
                    Ok(p) => {
                        self.status = format!("Added to {}.", p.display());
                        self.error = None;
                        self.learn_settings[i].added = true;
                        if let Some(info) = self.info.as_mut() {
                            info.settings.push(d);
                        }
                        self.settings_asked = false;
                    }
                    Err(e) => self.error = Some(e),
                },
            }
        }
        if let Some(i) = add_test {
            let id = profile.as_ref().map(|p| p.0.clone()).unwrap_or_default();
            let mut t = self.learn_tests[i].0.clone();
            t.name = t.name.trim().to_string();
            match icarus_obd::vehicles::add_test_to_user_file(&id, &icarus_obd::bus::to_toml(&t)) {
                Ok(p) => {
                    self.status = format!("Added to {}: it is on the Tests page now.", p.display());
                    self.error = None;
                    self.learn_tests[i].1 = true;
                    if let Some(info) = self.info.as_mut() {
                        info.tests.push(t);
                    }
                }
                Err(e) => self.error = Some(e),
            }
        }
        // Sarina, for what Icarus could not work out
        let sarina = icarus_obd::learning::sarina().is_some();
        if !self.learn_sessions.is_empty() {
            let mut ask: Option<std::path::PathBuf> = None;
            card(ui, pal, "Sessions", |ui| {
                if !sarina {
                    ui.label(RichText::new("With Sarina installed (optional), she can read a session and say what each setting most likely is and what the tool did that Icarus could not learn.").weak());
                }
                for p in self.learn_sessions.iter().take(8) {
                    let name = p
                        .file_name()
                        .and_then(|n| n.to_str())
                        .unwrap_or("")
                        .to_string();
                    ui.horizontal(|ui| {
                        ui.label(RichText::new(&name).monospace());
                        if ui.small_button("Open").clicked() {
                            let _ = std::process::Command::new("xdg-open").arg(p).spawn();
                        }
                        if sarina
                            && ui
                                .add_enabled(
                                    !self.sarina_busy,
                                    egui::Button::new("Ask Sarina").small(),
                                )
                                .clicked()
                        {
                            ask = Some(p.clone());
                        }
                    });
                }
            });
            if let Some(p) = ask {
                self.ask_sarina(ui.ctx(), p);
            }
        }
        if let Some((name, answer)) = &self.sarina_answer {
            card(ui, pal, &format!("Sarina on {name}"), |ui| {
                if self.sarina_busy {
                    ui.horizontal(|ui| {
                        ui.spinner();
                        ui.label("Reading the session...");
                    });
                } else {
                    match answer {
                        Ok(t) => {
                            ui.add(egui::Label::new(t.as_str()).selectable(true));
                        }
                        Err(e) => {
                            ui.colored_label(pal.red, e);
                        }
                    }
                }
            });
        }
    }

    fn page_garage(&mut self, ui: &mut egui::Ui, pal: &theme::Palette) {
        use icarus_obd::garage;
        ui.horizontal(|ui| {
            ui.label(
                RichText::new(format!(
                    "Vehicle folders in {}",
                    garage::library().display()
                ))
                .weak(),
            );
            if ui.button("Refresh").clicked() {
                self.garage = garage::vehicles();
            }
        });
        if self.garage.is_empty() {
            ui.label("No vehicles yet: each vehicle connected gets a folder here (OEM tunes, Custom tunes, Definitions, Reports, Logs).");
            return;
        }
        if self.garage_sel.is_none_or(|i| i >= self.garage.len()) {
            self.garage_sel = Some(0);
        }
        let mut action: Option<(&'static str, std::path::PathBuf, std::path::PathBuf)> = None;
        ui.columns(2, |cols| {
            for (i, (vin, name, _)) in self.garage.iter().enumerate() {
                if cols[0].selectable_label(self.garage_sel == Some(i), name.clone().unwrap_or_else(|| vin.clone())).clicked() {
                    self.garage_sel = Some(i);
                }
            }
            let ui = &mut cols[1];
            let Some((vin, name, dir)) = self.garage_sel.and_then(|i| self.garage.get(i)).cloned() else {
                ui.label("Choose a vehicle.");
                return;
            };
            ui.label(RichText::new(name.unwrap_or_else(|| vin.clone())).size(17.0).strong());
            ui.label(RichText::new(format!("VIN {vin}")).weak());
            if ui.button("Open the folder").clicked() {
                action = Some(("open", dir.clone(), dir.clone()));
            }
            let oem = garage::files(&dir, "OEM tunes").first().map(|f| dir.join("OEM tunes").join(&f.0));
            for sub in garage::FOLDERS {
                let files = garage::files(&dir, sub);
                card(ui, pal, &format!("{sub} ({})", files.len()), |ui| {
                    if sub == "OEM tunes" {
                        ui.label(RichText::new("Stock images, read-only: the way back. Never replaced, never edited.").small().weak());
                    }
                    for (f, size) in &files {
                        let path = dir.join(sub).join(f);
                        ui.horizontal_wrapped(|ui| {
                            ui.label(f);
                            ui.label(RichText::new(format!("{} KB", size / 1024)).small().weak());
                            match sub {
                                "OEM tunes" | "Custom tunes" => {
                                    if ui.small_button("Open in Maps").clicked() {
                                        action = Some(("maps", path.clone(), dir.clone()));
                                    }
                                    if sub == "Custom tunes" {
                                        if let Some(o) = &oem {
                                            if ui.small_button("Compare with OEM").clicked() {
                                                action = Some(("compare", path.clone(), o.clone()));
                                            }
                                        }
                                    }
                                }
                                "Definitions" => {
                                    if ui.small_button("Use in Maps").clicked() {
                                        action = Some(("definition", path.clone(), dir.clone()));
                                    }
                                }
                                _ => {
                                    if ui.small_button("Open").clicked() {
                                        action = Some(("open", path.clone(), dir.clone()));
                                    }
                                }
                            }
                        });
                    }
                });
            }
            card(ui, pal, "Add a file", |ui| {
                ui.add(egui::TextEdit::singleline(&mut self.import_path).hint_text("/path/to/file.bin or .xdf").desired_width(f32::INFINITY));
                ui.horizontal(|ui| {
                    if ui.button("As OEM tune (read-only)").clicked() {
                        action = Some(("oem", std::path::PathBuf::from(self.import_path.trim()), dir.clone()));
                    }
                    if ui.button("As custom tune").clicked() {
                        action = Some(("custom", std::path::PathBuf::from(self.import_path.trim()), dir.clone()));
                    }
                    if ui.button("As definition").clicked() {
                        action = Some(("def", std::path::PathBuf::from(self.import_path.trim()), dir.clone()));
                    }
                });
            });
        });
        let first_def = |dir: &std::path::Path| {
            garage::files(dir, "Definitions")
                .into_iter()
                .find(|f| f.0.to_lowercase().ends_with(".xdf"))
                .map(|f| dir.join("Definitions").join(f.0))
        };
        match action {
            Some(("open", p, _)) => {
                let _ = std::process::Command::new("xdg-open").arg(&p).spawn();
            }
            Some(("maps", p, dir)) => {
                if let Some(d) = first_def(&dir) {
                    self.maps.def_path = d.display().to_string();
                }
                self.maps.bin_path = p.display().to_string();
                self.maps.cmp_path.clear();
                self.page = Page::Maps;
                if !self.maps.def_path.is_empty() {
                    self.load_maps();
                }
            }
            Some(("compare", custom, oem)) => {
                if let Some(d) = oem.parent().and_then(|p| p.parent()).and_then(first_def) {
                    self.maps.def_path = d.display().to_string();
                }
                self.maps.bin_path = oem.display().to_string();
                self.maps.cmp_path = custom.display().to_string();
                self.page = Page::Maps;
                if !self.maps.def_path.is_empty() {
                    self.load_maps();
                }
            }
            Some(("definition", p, _)) => {
                self.maps.def_path = p.display().to_string();
                self.page = Page::Maps;
            }
            Some((kind @ ("oem" | "custom" | "def"), src, dir)) => {
                let r = if kind == "def" {
                    std::fs::create_dir_all(dir.join("Definitions")).and_then(|_| {
                        let to = dir
                            .join("Definitions")
                            .join(src.file_name().unwrap_or_default());
                        std::fs::copy(&src, &to).map(|_| to)
                    })
                } else {
                    garage::import(&dir, &src, kind == "oem")
                };
                match r {
                    Ok(p) => {
                        self.status = format!("Added {}", p.display());
                        self.import_path.clear();
                    }
                    Err(e) => self.error = Some(format!("{}: {e}", src.display())),
                }
            }
            _ => {}
        }
    }

    fn page_tools(&mut self, ui: &mut egui::Ui, pal: &theme::Palette) {
        card(ui, pal, "Tires and gears", |ui| {
            egui::Grid::new("tires")
                .num_columns(2)
                .spacing(vec2(12.0, 6.0))
                .show(ui, |ui| {
                    for (label, v, hint) in [
                        ("Stock tire", &mut self.t_stock, "265/70R16"),
                        ("New tire", &mut self.t_new, "285/75R16"),
                        ("Stock axle ratio", &mut self.t_gear, "3.91 (optional)"),
                        (
                            "Installed axle ratio",
                            &mut self.t_installed,
                            "after a regear (optional)",
                        ),
                        ("Speed", &mut self.t_speed, "65"),
                    ] {
                        ui.label(label);
                        ui.add(
                            egui::TextEdit::singleline(v)
                                .hint_text(hint)
                                .desired_width(160.0),
                        );
                        ui.end_row();
                    }
                });
            let num = |s: &str| s.trim().parse::<f64>().ok();
            match (
                icarus_core::tires::Tire::parse(&self.t_stock),
                icarus_core::tires::Tire::parse(&self.t_new),
            ) {
                (Ok(a), Ok(b)) => {
                    let (gear, installed, speed) = (
                        num(&self.t_gear),
                        num(&self.t_installed),
                        num(&self.t_speed).unwrap_or(65.0),
                    );
                    let installed = installed.filter(|_| gear.is_some());
                    let c = icarus_core::tires::change(a, b, speed, gear, installed);
                    ui.add_space(6.0);
                    for line in icarus_core::tires::explain(&c, speed, gear, installed) {
                        ui.label(line);
                    }
                }
                (Err(e), _) | (_, Err(e)) => {
                    ui.colored_label(pal.amber, e);
                }
            }
        });
        card(ui, pal, "Speedometer check", |ui| {
            ui.label(RichText::new("At a steady speed, note what the gauge shows, what the engine computer reports (Live data: vehicle speed) and a phone's GPS. Two different errors mean the gauge and the engine computer take speed from different places.").weak());
            egui::Grid::new("speedcheck")
                .num_columns(2)
                .spacing(vec2(12.0, 6.0))
                .show(ui, |ui| {
                    for (label, v) in [
                        ("GPS", &mut self.s_gps),
                        ("Gauge", &mut self.s_gauge),
                        ("Engine computer (OBD)", &mut self.s_obd),
                    ] {
                        ui.label(label);
                        ui.add(egui::TextEdit::singleline(v).desired_width(100.0));
                        ui.end_row();
                    }
                });
            if let Some(gps) = self.s_gps.trim().parse::<f64>().ok().filter(|g| *g > 0.0) {
                for (what, v) in [("Gauge", &self.s_gauge), ("Engine computer", &self.s_obd)] {
                    if let Ok(x) = v.trim().parse::<f64>() {
                        let e = (x - gps) / gps * 100.0;
                        ui.label(format!(
                            "{what}: reads {:.1} % {} (shows {x:.0} at a real {gps:.0})",
                            e.abs(),
                            if e < 0.0 { "low" } else { "high" }
                        ));
                    }
                }
            }
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
            let cpath = icarus_core::checksum::file_for(std::path::Path::new(m.def_path.trim()));
            (m.specs, m.specs_note) = match std::fs::read_to_string(&cpath) {
                Ok(t) => match icarus_core::checksum::parse(&t, bin.len()) {
                    Ok(v) => {
                        let st = icarus_core::checksum::check(&v, &bin, &bin);
                        let bad: Vec<&str> = st.iter().filter(|s| !s.trusted).map(|s| s.name.as_str()).collect();
                        let note = if bad.is_empty() {
                            format!("{} checksum(s) known, right in this image: corrected when you save", v.len())
                        } else {
                            format!("checksums not matching this image (never corrected): {}", bad.join(", "))
                        };
                        (v, note)
                    }
                    Err(e) => (Vec::new(), format!("{}: {e}", cpath.display())),
                },
                Err(_) => (Vec::new(), "checksums unknown for this definition: a saved file is for study, not for a module".into()),
            };
            m.save_path = icarus_core::edit::default_path(std::path::Path::new(m.bin_path.trim()))
                .display()
                .to_string();
            m.save_note.clear();
            m.cell = None;
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
        let mut edit: Option<MapEdit> = None;
        let Maps {
            def,
            rom,
            cmp,
            changed,
            search,
            only_changed,
            selected,
            cell,
            value_text,
            specs,
            specs_note,
            save_path,
            save_note,
            ..
        } = &mut self.maps;
        let (Some(def), Some(rom_mut)) = (def.as_ref(), rom.as_mut()) else {
            return;
        };
        let rom: &Rom = rom_mut;
        // what has been edited, and saving it
        let edits = rom.changes();
        ui.horizontal(|ui| {
            let bytes: usize = edits.iter().map(|r| r.len()).sum();
            if edits.is_empty() {
                ui.label(RichText::new("No edits yet: click a cell to change it.").weak());
            } else {
                ui.label(
                    RichText::new(format!(
                        "Edited: {bytes} byte(s) in {} place(s)",
                        edits.len()
                    ))
                    .color(pal.amber),
                );
                if ui.button("Undo all").clicked() {
                    edit = Some(MapEdit::UndoAll);
                }
            }
        });
        ui.label(RichText::new(specs_note.as_str()).small().weak());
        ui.horizontal(|ui| {
            ui.label("Save as");
            ui.add(egui::TextEdit::singleline(save_path).desired_width(440.0));
            if ui
                .add_enabled(!edits.is_empty(), egui::Button::new("Save"))
                .clicked()
            {
                *save_note = match icarus_core::edit::save(
                    rom,
                    specs,
                    std::path::Path::new(save_path.trim()),
                ) {
                    Ok(saved) => saved.summary(),
                    Err(e) => e,
                };
            }
        });
        if !save_note.is_empty() {
            ui.label(RichText::new(save_note.as_str()).small());
        }
        ui.add_space(4.0);
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
                if *selected != Some(i) {
                    *cell = None;
                }
                let e = &def.elements[i];
                edit_bar(ui, pal, def, rom, e, cell, value_text, &mut edit);
                element_view(ui, pal, def, rom, cmp.as_ref(), e, cell, value_text);
            });
        });
        // apply what was asked for
        let Some(ed) = edit else { return };
        let (Some(i), Some(rom)) = (*selected, self.maps.rom.as_mut()) else {
            return;
        };
        let Some(def) = self.maps.def.as_ref() else {
            return;
        };
        let e = &def.elements[i];
        let (row, col) = self.maps.cell.unwrap_or((0, 0));
        let result: Result<(), String> = match (ed, &e.kind) {
            (MapEdit::UndoAll, _) => {
                rom.revert();
                Ok(())
            }
            (MapEdit::UndoElement, Kind::Table(t)) => {
                def.span(&t.z).map(|r| rom.revert_range(r));
                Ok(())
            }
            (MapEdit::UndoElement, Kind::Constant(d)) => {
                def.span(d).map(|r| rom.revert_range(r));
                Ok(())
            }
            (MapEdit::Flag(on), Kind::Flag(f)) => {
                def.set_flag(rom, f, on).map_err(|x| x.to_string())
            }
            (
                MapEdit::Set(v),
                Kind::Table(icarus_core::def::Table { z: d, .. }) | Kind::Constant(d),
            ) => def
                .set(rom, d, row, col, v)
                .map(|_| ())
                .map_err(|x| x.to_string()),
            (
                MapEdit::Step(n),
                Kind::Table(icarus_core::def::Table { z: d, .. }) | Kind::Constant(d),
            ) => (|| {
                let at = def.cell_offset(d, row, col).map_err(|x| x.to_string())?;
                let raw = rom.read(at, d.cell).map_err(|x| x.to_string())?;
                // one step of the stored number, in the direction the real value goes
                let up = d.math.eval(raw + 1.0) >= d.math.eval(raw);
                let new = raw + if up { n as f64 } else { -n as f64 };
                let real = d.math.eval(new);
                if d.min.is_some_and(|m| real < m) || d.max.is_some_and(|m| real > m) {
                    return Err(format!(
                        "{} is outside the definition's limits",
                        fmt_num(real, d.decimals)
                    ));
                }
                rom.write(at, d.cell, new).map_err(|x| x.to_string())
            })(),
            _ => Ok(()),
        };
        match result {
            Ok(()) => self.maps.save_note.clear(),
            Err(e) => self.maps.save_note = e,
        }
        // the value shown in the box follows the cell
        if let (Some(rom), Some((r, c))) = (self.maps.rom.as_ref(), self.maps.cell) {
            if let Kind::Table(icarus_core::def::Table { z: d, .. }) | Kind::Constant(d) = &e.kind {
                if let Ok(v) = def.values(rom, d) {
                    self.maps.value_text = fmt_num(v[r][c], d.decimals);
                }
            }
        }
    }
}

/// The row above a map: the chosen cell's value to type over or step, a switch, and undoing this map.
#[allow(clippy::too_many_arguments)]
fn edit_bar(
    ui: &mut egui::Ui,
    pal: &theme::Palette,
    def: &Definition,
    rom: &Rom,
    e: &icarus_core::def::Element,
    cell: &mut Option<(usize, usize)>,
    value_text: &mut String,
    edit: &mut Option<MapEdit>,
) {
    let span = match &e.kind {
        Kind::Table(t) => def.span(&t.z),
        Kind::Constant(d) => def.span(d),
        Kind::Flag(_) => None,
    };
    let edited = span
        .as_ref()
        .is_some_and(|r| rom.bytes()[r.clone()] != rom.original()[r.clone()]);
    ui.horizontal(|ui| {
        match &e.kind {
            Kind::Flag(f) => {
                if let Ok(mut on) = def.flag(rom, f) {
                    if ui.checkbox(&mut on, "on").changed() {
                        *edit = Some(MapEdit::Flag(on));
                    }
                }
            }
            Kind::Constant(d) | Kind::Table(icarus_core::def::Table { z: d, .. }) => {
                if matches!(e.kind, Kind::Constant(_)) && cell.is_none() {
                    *cell = Some((0, 0));
                    if let Ok(v) = def.values(rom, d) {
                        *value_text = fmt_num(v[0][0], d.decimals);
                    }
                }
                match *cell {
                    None => {
                        ui.label(RichText::new("Click a cell to change it.").weak());
                    }
                    Some((r, c)) => {
                        if let Kind::Table(t) = &e.kind {
                            // the cell by its axes' values (load / speed, say), as the table's edges show them
                            let at = def
                                .table(rom, t)
                                .ok()
                                .and_then(|v| {
                                    Some(format!("Cell at {} / {}:", v.y.get(r)?, v.x.get(c)?))
                                })
                                .unwrap_or_else(|| format!("Row {}, column {}:", r + 1, c + 1));
                            ui.label(at);
                        }
                        let resp =
                            ui.add(egui::TextEdit::singleline(value_text).desired_width(90.0));
                        ui.label(RichText::new(&d.units).weak());
                        let enter =
                            resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                        if ui.button("Set").clicked() || enter {
                            match value_text.trim().parse::<f64>() {
                                Ok(v) => *edit = Some(MapEdit::Set(v)),
                                Err(_) => *value_text = String::new(),
                            }
                        }
                        if ui.button("-").on_hover_text("one step down").clicked() {
                            *edit = Some(MapEdit::Step(-1));
                        }
                        if ui.button("+").on_hover_text("one step up").clicked() {
                            *edit = Some(MapEdit::Step(1));
                        }
                        if let Ok(at) = def.cell_offset(d, r, c) {
                            if let (Ok(now), Ok(was)) =
                                (rom.read(at, d.cell), rom.read_original(at, d.cell))
                            {
                                if now != was {
                                    ui.label(
                                        RichText::new(format!(
                                            "was {}",
                                            fmt_num(d.math.eval(was), d.decimals)
                                        ))
                                        .color(pal.amber),
                                    );
                                }
                            }
                        }
                    }
                }
            }
        }
        if edited && ui.button("Undo this map").clicked() {
            *edit = Some(MapEdit::UndoElement);
        }
    });
}

#[allow(clippy::too_many_arguments)]
fn element_view(
    ui: &mut egui::Ui,
    pal: &theme::Palette,
    def: &Definition,
    rom: &Rom,
    cmp: Option<&Rom>,
    e: &icarus_core::def::Element,
    cell: &mut Option<(usize, usize)>,
    value_text: &mut String,
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
                                    let edited =
                                        def.cell_offset(&t.z, r, c).ok().is_some_and(|at| {
                                            rom.read(at, t.z.cell).ok()
                                                != rom.read_original(at, t.z.cell).ok()
                                        });
                                    let chosen = *cell == Some((r, c));
                                    let frame = egui::Frame::new()
                                        .fill(heat(f))
                                        .inner_margin(Margin::symmetric(5, 2))
                                        .stroke(if chosen {
                                            Stroke::new(2.0, Color32::WHITE)
                                        } else if edited {
                                            Stroke::new(2.0, pal.accent)
                                        } else if other.is_some() {
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
                                        .response
                                        .interact(Sense::click());
                                    if resp.clicked() {
                                        *cell = Some((r, c));
                                        *value_text = fmt_num(*n, t.z.decimals);
                                    }
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

/// One trouble code: the code, what it means, where to start, and buttons to look it up and to find the parts usually
/// involved (the links clicked are collected in `links` and opened by the page).
#[allow(clippy::too_many_arguments)]
fn code_row(
    ui: &mut egui::Ui,
    pal: &theme::Palette,
    code: &str,
    extra: &str,
    col: Color32,
    profile: Option<&Profile>,
    vehicle: Option<&Described>,
    links: &mut Vec<String>,
) {
    let info = dtc::explain(code, profile).ok();
    let text = info.as_ref().and_then(|i| i.text.clone());
    ui.horizontal_wrapped(|ui| {
        ui.label(RichText::new(code).strong().color(col));
        ui.label(text.clone().unwrap_or_else(|| match &info {
            Some(i) if !i.standard => "the maker's own code".into(),
            _ => "(no description yet)".into(),
        }));
        if !extra.is_empty() {
            ui.label(RichText::new(format!("({extra})")).weak());
        }
        if ui
            .add(egui::Button::new(RichText::new("Look up").small().color(pal.accent)).small())
            .on_hover_text("Search the web for this code on this vehicle")
            .clicked()
        {
            links.push(lookup::code_search(code, text.as_deref(), vehicle));
        }
    });
    if let Some(i) = &info {
        ui.label(RichText::new(dtc::anatomy(i)).small().color(pal.dim));
    }
    if let Some(h) = info.as_ref().and_then(|i| i.hint) {
        ui.label(RichText::new(format!("Where to start: {h}")).small());
    }
    let parts = dtc::parts(code);
    if !parts.is_empty() {
        ui.horizontal_wrapped(|ui| {
            ui.label(RichText::new("Parts:").small().weak());
            for p in parts {
                if ui
                    .add(egui::Button::new(RichText::new(*p).small().color(pal.accent)).small())
                    .on_hover_text("Search for this part for this vehicle")
                    .clicked()
                {
                    links.push(lookup::part_search(p, vehicle));
                }
            }
        });
    }
    ui.add_space(6.0);
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
            // fields and buttons on a card get the page colour, so they stand out from the card
            ui.visuals_mut().widgets.inactive.bg_fill = pal.bg;
            ui.visuals_mut().widgets.inactive.weak_bg_fill = pal.bg;
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
        if let Some(at) = self.learn_stop_at {
            if std::time::Instant::now() >= at {
                self.learn_stop_at = None;
                let _ = self.tx.send(Cmd::BusStop);
            } else {
                ctx.request_repaint_after(std::time::Duration::from_millis(200));
            }
        }
        self.background();

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
                ui.label(RichText::new("vehicle electronics").small().weak());
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
                                Page::Tests => self.page_tests(ui, &pal),
                                Page::Settings => self.page_settings(ui, &pal),
                                Page::Learn => self.page_learn(ui, &pal),
                                Page::Bus => self.page_bus(ui, &pal),
                                Page::Garage => self.page_garage(ui, &pal),
                                Page::Tools => self.page_tools(ui, &pal),
                                _ => self.page_live(ui, &pal),
                            });
                    }
                }
            });
        if let Some((target, label)) = self.confirm_clear.clone() {
            let mut go = false;
            let mut cancel = false;
            let resp = egui::Modal::new(egui::Id::new("confirm-clear")).show(&ctx, |ui| {
                ui.set_max_width(460.0);
                ui.label(
                    RichText::new(format!("Clear the codes of {label}?"))
                        .size(17.0)
                        .strong(),
                );
                ui.add_space(6.0);
                ui.label(icarus_obd::clear::WARNING);
                ui.label(RichText::new("The engine must be off, ignition on.").weak());
                ui.add_space(10.0);
                ui.horizontal(|ui| {
                    if ui
                        .add(
                            egui::Button::new(
                                RichText::new("Save a report and clear").color(pal.on_accent()),
                            )
                            .fill(pal.red),
                        )
                        .clicked()
                    {
                        go = true;
                    }
                    if ui.button("Cancel").clicked() {
                        cancel = true;
                    }
                });
            });
            if go {
                self.live_on = false;
                let _ = self.tx.send(Cmd::Clear(target));
                self.status = "Saving a report, then clearing...".into();
                self.confirm_clear = None;
            } else if cancel || resp.should_close() {
                self.confirm_clear = None;
            }
        }
        if let Some((def, value)) = self.confirm_setting.clone() {
            let mut go = false;
            let mut cancel = false;
            let now = self
                .setting_views
                .iter()
                .find(|v| v.name == def.name)
                .map(|v| v.now.clone())
                .unwrap_or_else(|| "not read".into());
            let to = match &value {
                Some(v) => v.clone(),
                None => self
                    .setting_views
                    .iter()
                    .find(|v| v.name == def.name)
                    .and_then(|v| v.undo_to.clone())
                    .unwrap_or_default(),
            };
            let resp = egui::Modal::new(egui::Id::new("confirm-setting")).show(&ctx, |ui| {
                ui.set_max_width(480.0);
                ui.label(
                    RichText::new(format!("{}: {now} -> {to}?", def.name))
                        .size(17.0)
                        .strong(),
                );
                ui.add_space(6.0);
                ui.label(format!(
                    "This writes module {:03X}'s memory. The value there now is read and kept in the vehicle's Logs folder first; the new one is read back afterwards, and if the module did not take it the old one is written back.",
                    def.module
                ));
                let conds: Vec<&str> = def.conditions.iter().map(|c| c.describe()).collect();
                if !conds.is_empty() {
                    ui.label(format!("Needs: {}. Battery at 12 V or more (a charger on it helps).", conds.join(", ")));
                }
                if !def.checked {
                    ui.colored_label(
                        pal.amber,
                        format!("Not yet checked on a vehicle. {}", def.note),
                    );
                }
                ui.add_space(10.0);
                ui.horizontal(|ui| {
                    if ui
                        .add(
                            egui::Button::new(RichText::new("Change it").color(pal.on_accent()))
                                .fill(pal.accent),
                        )
                        .clicked()
                    {
                        go = true;
                    }
                    if ui.button("Cancel").clicked() {
                        cancel = true;
                    }
                });
            });
            if go {
                self.live_on = false;
                let bytes = value.as_ref().and_then(|v| def.values.get(v).cloned());
                if value.is_some() && bytes.is_none() {
                    self.error = Some(format!("{} has no value named {to}", def.name));
                } else {
                    let _ = self.tx.send(Cmd::SettingChange { def, value: bytes });
                    self.status = "Changing the setting...".into();
                }
                self.confirm_setting = None;
            } else if cancel || resp.should_close() {
                self.confirm_setting = None;
            }
        }
        chrome::outline(&ctx, &pal);
    }
}

/// A learned setting being named on the Learning page: its values' names as learned and as typed, and whether it is in
/// the vehicle's file now.
struct LearnSetting {
    def: icarus_obd::settings::SettingDef,
    names: Vec<(String, String)>,
    added: bool,
}

impl LearnSetting {
    fn new(d: &icarus_obd::settings::SettingDef) -> LearnSetting {
        LearnSetting {
            def: d.clone(),
            names: d.values.keys().map(|k| (k.clone(), k.clone())).collect(),
            added: false,
        }
    }

    /// The setting with the names as typed, checked: every value named, no two the same.
    fn named(&self) -> Result<icarus_obd::settings::SettingDef, String> {
        let mut d = self.def.clone();
        d.name = d.name.trim().to_string();
        if d.name.is_empty() {
            return Err("give the setting a name".into());
        }
        let mut values = std::collections::BTreeMap::new();
        for (old, new) in &self.names {
            let n = new.trim();
            if n.is_empty() {
                return Err(format!("give the value now called {old:?} a name"));
            }
            if values
                .insert(n.to_string(), self.def.values[old].clone())
                .is_some()
            {
                return Err(format!("two values are named {n:?}"));
            }
        }
        d.values = values;
        Ok(d)
    }
}

fn main() -> eframe::Result {
    let start = match Start::parse(&std::env::args().skip(1).collect::<Vec<_>>()) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("icarus-app: {e}\nusage: icarus-app [--connect] [--adapter A] [--page vehicle|modules|live|tests|settings|learning|bus|maps|garage|tools] [--scan] [--live] [--learn [--mark TEXT] [--learn-seconds N]] [--maps DEF.xdf IMAGE.bin [SECOND.bin]] [--map NAME [--cell R,C]]");
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
