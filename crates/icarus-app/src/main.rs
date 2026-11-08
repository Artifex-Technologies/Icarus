//! Icarus's window: the vehicle (adapter, identity, check-engine light, monitors, codes), its modules (identity and codes,
//! as dealer tools show them), live values with graphs, and maps from an image and a definition (with what a tune
//! changed). Read-only, like the rest of Icarus for now. Yggdrasil's look (yggdrasil-ui).

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
    Bus,
    Maps,
    Garage,
    Tools,
}

const PAGES: [(Page, &str); 8] = [
    (Page::Vehicle, "Vehicle"),
    (Page::Modules, "Modules"),
    (Page::Live, "Live data"),
    (Page::Tests, "Tests"),
    (Page::Bus, "Bus"),
    (Page::Maps, "Maps"),
    (Page::Garage, "Garage"),
    (Page::Tools, "Tools"),
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
    bus: bool,
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
                "--bus" => s.bus = true,
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
    /// The bus monitor: the source (adapter or a Linux CAN interface), its last view.
    bus_can: bool,
    bus_ifname: String,
    bus: Option<worker::BusView>,
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
            bus_can: false,
            bus_ifname: "can0".into(),
            bus: None,
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
                    if std::mem::take(&mut self.start.scan) {
                        self.scanning = true;
                        let _ = self.tx.send(Cmd::Scan {
                            profile: None,
                            sweep: false,
                        });
                    }
                    if std::mem::take(&mut self.start.bus) {
                        let _ = self.tx.send(Cmd::BusStart(None));
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
                    }
                    self.bus = Some(*v);
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
        let mut read_selftests = false;
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
            if let Some(v) = vin {
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
                ));
            }
        });
        ui.label(RichText::new("Watching only: nothing is sent onto the bus. Press a pedal or a switch and see which byte follows (changed bytes light up). Diagnostic messages are put in words; actuator tests another tool runs are learned.").weak());
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
        let mut add: Option<icarus_obd::actuator::TestDef> = None;
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
        if !b.learned.is_empty() {
            ui.add_space(8.0);
            let profile = self
                .info
                .as_ref()
                .and_then(|i| i.profile.as_ref())
                .map(|p| (p.id.clone(), p.name.clone()));
            for t in &b.learned {
                card(ui, pal, &t.name, |ui| {
                    ui.label(RichText::new("Another tool ran this output test. Check what it drives before relying on it; it is added unchecked, with the most cautious conditions.").weak());
                    ui.label(
                        RichText::new(icarus_obd::bus::to_toml(t))
                            .monospace()
                            .small(),
                    );
                    let label = profile.as_ref().map_or(
                        "Connect to a vehicle with a make profile to add it".to_string(),
                        |p| format!("Add to the {} file", p.1),
                    );
                    if ui
                        .add_enabled(profile.is_some(), egui::Button::new(label))
                        .clicked()
                    {
                        add = Some(t.clone());
                    }
                });
            }
        }
        if let Some(t) = add {
            let id = self
                .info
                .as_ref()
                .and_then(|i| i.profile.as_ref())
                .map(|p| p.id.clone())
                .unwrap_or_default();
            match icarus_obd::vehicles::add_test_to_user_file(&id, &icarus_obd::bus::to_toml(&t)) {
                Ok(p) => {
                    self.status = format!("Added to {}: it is on the Tests page now.", p.display());
                    if let Some(i) = self.info.as_mut() {
                        i.tests.push(t);
                    }
                }
                Err(e) => self.error = Some(e),
            }
        }
        if running {
            ui.ctx()
                .request_repaint_after(std::time::Duration::from_millis(250));
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
