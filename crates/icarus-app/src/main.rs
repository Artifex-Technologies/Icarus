//! Icarus's window: the vehicle (adapter, identity, check-engine light, monitors, codes), its modules (identity and codes,
//! as dealer tools show them), live values with graphs, actuator tests, module settings, learning from another tool, the
//! bus, and maps from an image and a definition (with what a tune changed). Yggdrasil's look (yggdrasil-ui).

mod autoconn;
mod confirm;
mod devices;
mod gauges;
mod levels;
mod listen;
mod logs;
mod single;
mod surface;
mod worker;

use eframe::egui::{self, vec2, Color32, CornerRadius, Margin, RichText, Sense, Stroke};
use icarus_core::def::{fmt_num, Definition, Kind};
use icarus_core::rom::{self, Rom};
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
    /// Big readouts for the trail (ICA-27).
    Gauges,
    Tests,
    Settings,
    Learn,
    Bus,
    Maps,
    Garage,
    Tools,
    Logs,
    /// The window's own settings (its levels), behind the gear.
    App,
}

/// The level a page needs (ICA-74): Basic for diagnosis and analysis, Advanced for controlling modules, Tuning for maps.
fn need(p: Page) -> levels::Need {
    use levels::Need;
    match p {
        Page::Vehicle
        | Page::Modules
        | Page::Live
        | Page::Gauges
        | Page::Logs
        | Page::Garage
        | Page::Tools
        | Page::App => Need::Basic,
        Page::Tests | Page::Settings | Page::Learn | Page::Bus => Need::Advanced,
        Page::Maps => Need::Tuning,
    }
}

const PAGES: [(Page, &str); 12] = [
    (Page::Vehicle, "Vehicle"),
    (Page::Modules, "Modules"),
    (Page::Live, "Live data"),
    (Page::Gauges, "Trail gauges"),
    (Page::Logs, "Logs"),
    (Page::Tests, "Tests"),
    (Page::Settings, "Module settings"),
    (Page::Learn, "Learning"),
    (Page::Bus, "Bus"),
    (Page::Maps, "Maps"),
    (Page::Garage, "Garage"),
    (Page::Tools, "Tools"),
];

/// Answers from the NHTSA thread.
enum Bg {
    /// Boxed: a decode is far larger than the other answers.
    Vin(Result<Box<icarus_obd::nhtsa::VinDecode>, String>),
    Recalls(Result<Vec<icarus_obd::nhtsa::Recall>, String>),
    /// Sarina's answer about a learning session.
    Sarina(Result<String, String>),
    /// What a GPS receiver said over some seconds.
    Gps(Result<icarus_obd::gps::Fix, String>, f64),
}

/// What a clear removes the codes from: None = every OBD-II module, Some = one module (tx, rx, dialect).
type ClearTarget = Option<(u16, u16, Dialect)>;

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
    look: Look,
    /// The image's versions kept beside it (icarus-core version.rs), and the one shown against stock.
    versions: Vec<(std::path::PathBuf, icarus_core::version::Version)>,
    version_shown: Option<usize>,
    version_name: String,
    version_note: String,
    version_msg: String,
    /// The version whose deletion is asked and not yet confirmed (ICA-58).
    version_delete: Option<usize>,
}

impl Maps {
    /// Nothing open yet.
    fn empty() -> Maps {
        Maps {
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
            look: Look {
                view: View::Cells,
                yaw: 0.6,
                pitch: 0.9,
            },
            versions: Vec::new(),
            version_shown: None,
            version_name: String::new(),
            version_note: String::new(),
            version_msg: String::new(),
            version_delete: None,
        }
    }
}

/// How a table is shown: its cells, a 2D graph or a 3D surface (surface.rs), and the 3D view's turn and tilt (radians).
struct Look {
    view: View,
    yaw: f32,
    pitch: f32,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum View {
    Cells,
    Graph,
    Surface,
}

/// An edit asked for on the Maps page, applied after drawing.
enum MapEdit {
    Set(f64),
    Step(i64),
    Flag(bool),
    UndoElement,
    UndoAll,
}

/// The Bus page's signals cell for one ID (ICA-88): the message's name and what the DBC names in its last bytes; "" for
/// an ID the DBC does not name.
fn dbc_cell(d: &icarus_core::dbc::Dbc, id: u32, data: &[u8]) -> String {
    d.line(id, id > 0x7FF, data).unwrap_or_default()
}

/// Something asked for in the Maps page's versions row, done after drawing.
enum VersionAct {
    /// Compare with the original only ("stock").
    Stock,
    /// Show what version i changes against stock.
    Show(usize),
    /// Take version i's changes as the edits, to go on from them.
    Edit(usize),
    /// Keep the edits as a new version.
    Save,
    /// Give version i the name typed in the name box (ICA-58).
    Rename(usize),
    /// Remove version i's file, once confirmed (ICA-58).
    Delete(usize),
}

/// Per element: whether its values differ between images `a` and `b` (the same size).
fn changed_elements(def: &Definition, a: &[u8], b: &[u8]) -> Vec<bool> {
    let mut changed = vec![false; def.elements.len()];
    for r in rom::diff(a, b) {
        for e in def.covering(r) {
            if let Some(i) = def.elements.iter().position(|x| std::ptr::eq(x, e)) {
                changed[i] = true;
            }
        }
    }
    changed
}

/// What to do at start (command line): `--connect` (the adapter from ICARUS_ADAPTER, or `--adapter A`), `--page NAME`,
/// `--scan` and `--live` once connected, `--maps DEF.xdf|.a2l IMAGE.bin [SECOND.bin]`, `--learn` (a learning watch once
/// connected; `--mark TEXT` marks at its start and `--learn-seconds N` stops it, for demonstrations and screenshots),
/// `--gps SOURCE` (the Tools page's GPS source, listened to at start).
#[derive(Default)]
struct Start {
    connect: bool,
    adapter: Option<String>,
    page: Option<Page>,
    scan: bool,
    /// `--module-data`: read each module's own values once connected.
    module_data: bool,
    live: bool,
    /// `--gauges`: read the trail gauges once connected.
    gauges: bool,
    /// `--record`: record a drive (every value the vehicle offers) once connected.
    record: bool,
    bus: bool,
    learn: bool,
    mark: Option<String>,
    learn_secs: Option<f64>,
    maps: Option<(String, String, String)>,
    /// `--map NAME`: the map to show first (with `--cell R,C`, that cell chosen for editing).
    map: Option<String>,
    cell: Option<(usize, usize)>,
    /// `--view cells|2d|3d`: how the Maps page shows a table at start.
    view: Option<View>,
    gps: Option<String>,
}

impl Start {
    fn parse(a: &[String]) -> Result<Start, String> {
        let mut s = Start::default();
        let mut i = 0;
        while i < a.len() {
            match a[i].as_str() {
                "--connect" => s.connect = true,
                // the session floor (ICA-192) is taken from the whole line before it is read here (`take_floor`)
                icarus_obd::lock::SESSION_FLAG => {}
                "--scan" => s.scan = true,
                "--module-data" => s.module_data = true,
                "--live" => s.live = true,
                "--gauges" => s.gauges = true,
                "--record" => s.record = true,
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
                "--gps" => {
                    i += 1;
                    s.gps = Some(a.get(i).ok_or("--gps needs a source")?.clone());
                }
                "--adapter" => {
                    i += 1;
                    s.adapter = Some(a.get(i).ok_or("--adapter needs a value")?.clone());
                }
                "--page" => {
                    i += 1;
                    let want = a.get(i).ok_or("--page needs a name")?.to_lowercase();
                    // "settings" is the window's own (the gear); a module's settings are "module settings"
                    s.page = Some(if want == "settings" {
                        Page::App
                    } else {
                        PAGES
                            .iter()
                            .find(|p| p.1.to_lowercase().starts_with(&want))
                            .ok_or(format!("no page {want}"))?
                            .0
                    });
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
                "--view" => {
                    i += 1;
                    s.view = Some(match a.get(i).map(|v| v.to_lowercase()).as_deref() {
                        Some("cells") => View::Cells,
                        Some("2d") => View::Graph,
                        Some("3d") => View::Surface,
                        _ => return Err("--view is cells, 2d or 3d".into()),
                    });
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

/// Whether a start's line asks for the session floor (ICA-192), held at once through `hold` when it does: `--read-only`
/// anywhere on it (where an option took it as its value too, and beside an option this window does not know), or
/// ICARUS_READ_ONLY in this start's environment (`env_on`). It does not wait for the line to be read, so a line that
/// does not parse cannot lose it.
fn take_floor(args: &[String], env_on: bool, hold: &dyn Fn()) -> bool {
    let asked = env_on || args.iter().any(|a| a == icarus_obd::lock::SESSION_FLAG);
    if asked {
        hold();
    }
    asked
}

/// A start's line, read: what the window does at start, the line a window already open is handed, and whether the session
/// floor was asked for.
struct Begun {
    start: Start,
    hand: Vec<String>,
    read_only: bool,
}

/// Reads a start's line: the session floor first (`take_floor`, through `hold`), then the options (a line that does not
/// parse is said by this start, never handed to an open window to be dropped there). ICARUS_READ_ONLY is said as
/// `--read-only` on the line handed over, so the window already open takes it with the rest.
fn begin(mut args: Vec<String>, env_on: bool, hold: &dyn Fn()) -> Result<Begun, String> {
    let read_only = take_floor(&args, env_on, hold);
    let start = Start::parse(&args)?;
    if read_only && !args.iter().any(|a| a == icarus_obd::lock::SESSION_FLAG) {
        args.push(icarus_obd::lock::SESSION_FLAG.into());
    }
    Ok(Begun {
        start,
        hand: args,
        read_only,
    })
}

/// What the window talks through: the worker's two ends, what later starts hand over (single.rs), and the adapter chosen
/// with whether to connect to it by itself (autoconn.rs).
struct Wires {
    tx: Sender<Cmd>,
    rx: Receiver<Event>,
    handed: Receiver<Vec<String>>,
    chosen: (String, bool),
}

struct App {
    start: Start,
    /// The name being typed for this vehicle (garage), and whether to save it.
    name_edit: String,
    /// A report with every module's codes too.
    report_all: bool,
    last_report: Option<String>,
    /// Each module's own values as last read (Modules page), and whether a reading is under way.
    module_data: Option<Result<icarus_obd::vehicledata::VehicleValues, String>>,
    reading_data: bool,
    /// Actuator tests: the percent chosen per test, the one held now, the last thing said, the watched value.
    test_values: HashMap<String, f64>,
    holding: Option<String>,
    /// After a test stopped by itself while the button was still held: the button must be let go before any test starts.
    release_needed: bool,
    test_note: String,
    test_reading: Option<f64>,
    /// The clear, service routine and setting change waiting for the person's yes, each tied to the connection it was asked
    /// on and closed when that one ends (ICA-105).
    confirms: confirm::Confirms,
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
    /// A CAN database for the Bus page (ICA-88): its path as typed, what was read, and what opening it said.
    bus_dbc_path: String,
    bus_dbc: Option<icarus_core::dbc::Dbc>,
    bus_dbc_note: String,
    /// What the vehicle profile calls the bus's IDs (ICA-17), and for which profile ("" for none): read again when it changes.
    bus_names: (String, std::collections::BTreeMap<u32, String>),
    bus: Option<worker::BusView>,
    /// Settings: each one's value now (from the worker), the value picked per setting, and whether the values were asked for
    /// since connecting (a change waiting for the person's yes is in `confirms`).
    setting_views: Vec<worker::SettingView>,
    setting_pick: HashMap<String, String>,
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
    /// Each vehicle's details (its own name, sub-model ...), by VIN, as saved; and the shown one's fields being edited.
    garage_details: std::collections::BTreeMap<String, garage::Details>,
    garage_edit: Option<GarageEdit>,
    /// The VINs whose Read-only switch is on (`icarus_obd::lock`).
    read_only: std::collections::BTreeSet<String>,
    import_path: String,
    /// Tools: tire sizes and gears, and a speedometer check.
    t_stock: String,
    t_new: String,
    t_gear: String,
    t_installed: String,
    t_speed: String,
    /// The vehicle measures speed at the transmission output (a regear counts), for the tire setting saved per vehicle.
    t_at_transmission: bool,
    /// The connected vehicle's saved tire setting and what its speed reading is multiplied by for the real speed.
    tires: Option<(icarus_obd::garage::TireSetting, f64)>,
    s_gauge: String,
    s_gps: String,
    /// The GPS source (`/dev/ttyACM0`, `tcp:HOST:PORT`, `sim`...), what it said last (and over how many seconds), and
    /// whether it is being listened to.
    gps_src: String,
    gps_fix: Option<(icarus_obd::gps::Fix, f64)>,
    gps_busy: bool,
    /// Record drives with the GPS's columns too (opted into on the Live data page).
    record_gps: bool,
    /// The module values a recording reads every sweep, one a line (Settings; `drive::watch_path`, ICA-218).
    record_watch: String,
    /// `--gps`: listen once the Tools page is drawn.
    gps_now: bool,
    s_obd: String,
    page: Page,
    /// The window's levels (levels.rs).
    levels: levels::Levels,
    /// Which devices only listen (devices.rs, ICA-80), and a device listening now while another talks: what it heard
    /// (the requests it saw) beside what the talking device sent.
    roles: devices::Roles,
    listener: Option<listen::Listener>,
    listen_status: String,
    listen_file: Option<std::path::PathBuf>,
    listen_pick: usize,
    listen_protocol: u8,
    heard: VecDeque<icarus_obd::bus::Frame>,
    heard_frames: u64,
    sent_log: VecDeque<(f64, u32, Vec<u8>)>,
    new_device: String,
    /// Asks the window to draw again (for a listener's thread).
    repaint: std::sync::Arc<dyn Fn() + Send + Sync>,
    /// Turning Tuning on waits for this question's yes.
    tuning_ask: bool,
    /// The first-start welcome is open (ICA-78).
    welcome: bool,
    tx: Sender<Cmd>,
    rx: Receiver<Event>,
    /// Connect by itself, and again whenever the adapter answers once more (a real adapter is known; autoconn.rs).
    auto: bool,
    /// Disconnect was pressed: no more tries until Connect is.
    user_off: bool,
    last_try: std::time::Instant,
    /// What a later `icarus-app ...` handed over (single.rs).
    handed: Receiver<Vec<String>>,
    adapter: String,
    link: Link,
    status: String,
    /// Modules left out of a broadcast's answers on this connection (`Event::Dropped`), shown under the status line.
    dropped: Vec<String>,
    error: Option<String>,
    info: Option<Info>,
    profile_choice: String,
    sweep: bool,
    scanning: bool,
    /// Basic's guided scan (ICA-75): where it is.
    guided: Guided,
    modules: Vec<ModuleView>,
    scan_note: String,
    unreachable: Vec<String>,
    live_on: bool,
    /// The live values read are the trail gauges' (their page started them), not the ones chosen on Live data.
    gauges_on: bool,
    chosen: BTreeSet<u8>,
    history: HashMap<u8, VecDeque<(f64, f64)>>,
    /// A drive being recorded by the worker (icarus_obd::drive): its file, values a row and rows so far.
    recording: Option<Recording>,
    logs: logs::Logs,
    maps: Maps,
}

impl App {
    fn new(ctx: &egui::Context, start: Start) -> App {
        let (tx, wrx) = channel();
        let (wtx, rx) = channel();
        let c = ctx.clone();
        let handle = std::thread::spawn(move || {
            worker::run(wrx, wtx, icarus_obd::fence::current, move || {
                c.request_repaint()
            })
        });
        // kept for the end: a recording still going when the window closes is stopped and its file closed (`wind_down`)
        if let Ok(mut w) = WORKER.lock() {
            *w = Some((tx.clone(), handle));
        }
        // the adapter: the one named, the one that last worked, or a paired Bluetooth OBD adapter (autoconn.rs)
        let chosen = autoconn::choose_here(
            start
                .adapter
                .clone()
                .or_else(|| std::env::var("ICARUS_ADAPTER").ok()),
        );
        // later starts hand their arguments over here (single.rs)
        let (handed_tx, handed_rx) = channel::<Vec<String>>();
        let c2 = ctx.clone();
        single::listen(move |args| {
            // the session floor first, on this thread and whatever the rest of the line says (ICA-192), so the start that
            // asked for it hears that this window holds it
            take_floor(&args, false, &icarus_obd::lock::hold_session_read_only);
            let held = icarus_obd::lock::session_read_only();
            let _ = handed_tx.send(args);
            c2.request_repaint();
            held
        });
        App::wired(
            ctx,
            start,
            Wires {
                tx,
                rx,
                handed: handed_rx,
                chosen,
            },
        )
    }

    /// The window on `wires` (`new` gives the worker's thread, the adapter chosen and the hand-over's socket; a test gives
    /// its own).
    fn wired(ctx: &egui::Context, start: Start, wires: Wires) -> App {
        let Wires {
            tx,
            rx,
            handed: handed_rx,
            chosen,
        } = wires;
        let levels = levels::Levels::load();
        let (bg_tx, bg_rx) = channel::<Bg>();
        let c2 = ctx.clone();
        let repaint: std::sync::Arc<dyn Fn() + Send + Sync> =
            std::sync::Arc::new(move || c2.request_repaint());
        let mut app = App {
            name_edit: String::new(),
            report_all: true,
            last_report: None,
            module_data: None,
            reading_data: false,
            test_values: HashMap::new(),
            holding: None,
            release_needed: false,
            test_note: String::new(),
            test_reading: None,
            confirms: Default::default(),
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
            bus_dbc_path: String::new(),
            bus_dbc: None,
            bus_dbc_note: String::new(),
            bus_names: (String::from("?"), Default::default()),
            bus: None,
            setting_views: Vec::new(),
            setting_pick: HashMap::new(),
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
            garage_details: Default::default(),
            garage_edit: None,
            read_only: icarus_obd::lock::read_only(),
            import_path: String::new(),
            t_stock: "P265/65R17".into(),
            t_new: "LT285/70R17".into(),
            t_gear: "3.91".into(),
            t_installed: "4.88".into(),
            t_speed: "65".into(),
            t_at_transmission: false,
            tires: None,
            s_gauge: String::new(),
            s_gps: String::new(),
            gps_src: start
                .gps
                .clone()
                .or_else(|| std::env::var("ICARUS_GPS").ok())
                .unwrap_or_default(),
            gps_fix: None,
            gps_busy: false,
            record_gps: false,
            record_watch: watch_field(&icarus_obd::drive::load_watch(
                icarus_obd::drive::watch_path().as_deref(),
            )),
            gps_now: start.gps.is_some(),
            s_obd: String::new(),
            // a start page the levels do not show (--page tests on Basic) is the vehicle's
            page: start
                .page
                .filter(|p| levels.allows(need(*p)))
                .unwrap_or(Page::Vehicle),
            levels,
            roles: devices::Roles::load(),
            listener: None,
            listen_status: String::new(),
            listen_file: None,
            listen_pick: 0,
            listen_protocol: 6,
            heard: VecDeque::new(),
            heard_frames: 0,
            sent_log: VecDeque::new(),
            new_device: String::new(),
            repaint,
            tuning_ask: false,
            // a plain first start (not one opened at a page) gets the welcome
            welcome: !levels.welcomed && start.page.is_none(),
            tx,
            rx,
            adapter: chosen.0.clone(),
            auto: chosen.1,
            user_off: false,
            last_try: std::time::Instant::now(),
            handed: handed_rx,
            link: Link::Off,
            status: "Not connected.".into(),
            dropped: Vec::new(),
            error: None,
            info: None,
            profile_choice: String::new(),
            sweep: false,
            scanning: false,
            guided: Guided::Off,
            modules: Vec::new(),
            scan_note: String::new(),
            unreachable: Vec::new(),
            live_on: false,
            gauges_on: false,
            chosen: [0x0C, 0x05, 0x06, 0x07, 0x44].into_iter().collect(),
            history: HashMap::new(),
            recording: None,
            logs: logs::Logs::default(),
            maps: Maps::empty(),
            start,
        };
        if let Some(v) = app.start.view {
            app.maps.look.view = v;
        }
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
        // a real adapter known (named, the last one that worked, or a paired Bluetooth OBD adapter): connect at once
        if app.start.connect || app.auto {
            app.link = Link::Busy;
            app.last_try = std::time::Instant::now();
            let _ = app.tx.send(Cmd::Connect(app.adapter.trim().to_string()));
        }
        app
    }

    /// While a known adapter does not answer (asleep with the engine off), try again every few seconds.
    /// What is polled: the chosen values (or the trail gauges', when their page started them); while recording, the
    /// recording's sweep reads them besides its own.
    fn send_live(&self) {
        let wanted: Vec<u8> = if self.gauges_on {
            gauges::gauge_pids(self.info.as_ref().map_or(&[][..], |i| &i.live))
        } else {
            self.chosen.iter().copied().collect()
        };
        let _ = self.tx.send(Cmd::Live(wanted));
    }

    /// Records a drive: every value the vehicle offers and its module data (icarus_obd::drive, in the worker), live data
    /// started for it.
    fn start_recording(&mut self) {
        if self.info.is_none() {
            self.status = "Connect first: a recording starts once the vehicle answers.".into();
            return;
        }
        let gps = (self.record_gps && !self.gps_src.trim().is_empty())
            .then(|| self.gps_src.trim().to_string());
        let _ = self.tx.send(Cmd::Record(Some(worker::RecordStart {
            gps,
            quiet: self.levels.record_quiet,
            watch: icarus_obd::drive::parse_watch(&self.record_watch),
        })));
        self.status = "Starting the recording...".into();
        self.live_on = true;
        self.send_live();
    }

    fn stop_recording(&mut self) {
        if self.recording.is_some() {
            let _ = self.tx.send(Cmd::Record(None));
        }
    }

    /// What the worker's recording did.
    fn on_recording(&mut self, r: worker::Rec) {
        let name = |p: &std::path::Path| {
            p.file_name()
                .map(|f| f.to_string_lossy().into_owned())
                .unwrap_or_default()
        };
        match r {
            worker::Rec::Started { path, values } => {
                // a new part keeps the rows counted so far
                let (rows, said) = self
                    .recording
                    .take()
                    .map_or((0, Vec::new()), |r| (r.rows, r.said));
                self.status = format!(
                    "Recording {values} values to {} (the Logs page shows it as it comes in).",
                    name(&path)
                );
                self.recording = Some(Recording {
                    path,
                    values,
                    rows,
                    said,
                });
            }
            worker::Rec::Rows(n) => {
                if let Some(r) = self.recording.as_mut() {
                    r.rows = n;
                }
            }
            worker::Rec::Said(line) => {
                if let Some(r) = self.recording.as_mut() {
                    r.said.push(line);
                }
            }
            worker::Rec::Stopped { rows, paths } => {
                self.recording = None;
                self.status = format!(
                    "Saved {rows} rows to {} (see the Logs page).",
                    paths
                        .iter()
                        .map(|p| p.display().to_string())
                        .collect::<Vec<_>>()
                        .join(", ")
                );
                if self.live_on {
                    self.send_live();
                }
            }
            worker::Rec::Failed(why) => {
                if self.recording.take().is_some() {
                    self.status = format!("The recording ended: {why}");
                } else {
                    self.status = format!("Could not record: {why}");
                }
            }
        }
    }

    fn keep_trying(&mut self, ctx: &egui::Context) {
        if !self.auto || self.user_off || self.link != Link::Off {
            return;
        }
        if self.last_try.elapsed() >= autoconn::RETRY {
            self.last_try = std::time::Instant::now();
            // a recording in the background has the adapter: show it (Logs follows it) rather than try for the adapter
            if let Some(p) = logs::recording_now().filter(|_| self.recording.is_none()) {
                let name = p
                    .file_name()
                    .map(|f| f.to_string_lossy().into_owned())
                    .unwrap_or_default();
                self.status = format!("The adapter is recording in the background ({name}): the Logs page shows it as it comes in.");
                return;
            }
            self.link = Link::Busy;
            let _ = self.tx.send(Cmd::Connect(self.adapter.trim().to_string()));
        }
        ctx.request_repaint_after(std::time::Duration::from_secs(1));
    }

    /// A later `icarus-app ARGS`: its page, and its adapter (connecting to it when asked), then to the front.
    fn take_handed(&mut self, ctx: &egui::Context) {
        while let Ok(args) = self.handed.try_recv() {
            // a later start asking for read-only made this window read-only as it was handed over, before its line was read
            // (single::listen in `new`); nothing makes it writable again
            if let Ok(st) = Start::parse(&args) {
                if st.record {
                    if self.link == Link::On && self.recording.is_none() {
                        self.start_recording();
                    } else {
                        self.start.record = true;
                    }
                }
                if let Some(p) = st.page {
                    self.page = p;
                }
                if let Some(a) = st
                    .adapter
                    .filter(|a| *a != self.adapter && self.link == Link::Off)
                {
                    self.adapter = a;
                }
                if st.connect && self.link == Link::Off {
                    self.user_off = false;
                    self.auto = self.adapter.trim() != "sim";
                    self.link = Link::Busy;
                    self.last_try = std::time::Instant::now();
                    let _ = self.tx.send(Cmd::Connect(self.adapter.trim().to_string()));
                }
            }
            single::raise(ctx);
        }
    }

    fn background(&mut self) {
        while let Ok(b) = self.bg_rx.try_recv() {
            self.asking = false;
            match b {
                Bg::Vin(Ok(d)) => {
                    self.name_edit = d.name();
                    self.status = "NHTSA's answer is in the name field: Save keeps it.".into();
                    self.nhtsa = Some(*d);
                }
                Bg::Recalls(Ok(r)) => {
                    self.status = format!("{} recall(s) on record.", r.len());
                    self.recalls = Some(r);
                }
                Bg::Vin(Err(e)) | Bg::Recalls(Err(e)) => self.error = Some(e),
                Bg::Gps(r, secs) => {
                    self.gps_busy = false;
                    match r {
                        Ok(fx) => {
                            self.status = format!("GPS: {}.", fx.summary());
                            self.gps_fix = Some((fx, secs));
                        }
                        Err(e) => self.error = Some(format!("GPS: {e}")),
                    }
                }
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
        // the listening device's frames: the requests it saw are kept for the comparison
        let mut ended = None;
        if let Some(l) = &self.listener {
            while let Ok(ev) = l.rx.try_recv() {
                match ev {
                    listen::Event::Status(s) => self.listen_status = s,
                    listen::Event::Frames(fs) => {
                        self.heard_frames += fs.len() as u64;
                        for f in fs.into_iter().filter(listen::is_request) {
                            self.heard.push_back(f);
                            if self.heard.len() > 200 {
                                self.heard.pop_front();
                            }
                        }
                    }
                    listen::Event::Ended(why) => ended = Some(why),
                }
            }
        }
        if let Some(why) = ended {
            self.listen_status = format!("Listening ended: {why}.");
            self.listener = None;
        }
        while let Ok(ev) = self.rx.try_recv() {
            // a confirmation waiting for a yes closes when its connection ends, another starts or the link fails (ICA-105)
            self.confirms.on_event(&ev);
            match ev {
                Event::Sent(v) => {
                    for x in v {
                        self.sent_log.push_back(x);
                        if self.sent_log.len() > 200 {
                            self.sent_log.pop_front();
                        }
                    }
                }
                Event::Status(s) => self.status = s,
                Event::Dropped(n) => self.dropped.push(n),
                Event::Connected(info) => {
                    self.link = Link::On;
                    self.dropped.clear();
                    // the adapter to start with next time, and to come back to when it answers again
                    autoconn::remember(self.adapter.trim());
                    self.auto = self.adapter.trim() != "sim";
                    self.error = None;
                    self.status = "Connected.".into();
                    if self.guided == Guided::Connecting {
                        // the guided scan goes on: every module's codes
                        self.guided = Guided::Scanning;
                        self.scanning = true;
                        let _ = self.tx.send(Cmd::Scan {
                            profile: None,
                            sweep: false,
                        });
                    }
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
                    self.refresh_tires();
                    self.modules.clear();
                    self.history.clear();
                    self.selftests = None;
                    self.setting_views.clear();
                    self.settings_asked = false;
                    self.refresh_sessions();
                    self.module_data = None;
                    if std::mem::take(&mut self.start.module_data) {
                        self.reading_data = true;
                        let _ = self.tx.send(Cmd::ModuleData);
                    }
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
                        self.send_live();
                    }
                    if std::mem::take(&mut self.start.gauges) {
                        self.gauges_on = true;
                        self.live_on = true;
                        self.send_live();
                    }
                    // a recording goes on where it was (the adapter dropped and came back), or starts when asked
                    if self.recording.is_some() {
                        self.live_on = true;
                        self.send_live();
                        self.status = "Connected again: recording goes on.".into();
                    } else if std::mem::take(&mut self.start.record) {
                        self.start_recording();
                    }
                }
                Event::Failed(m) => {
                    if self.link == Link::Busy {
                        self.link = Link::Off;
                    }
                    if matches!(self.guided, Guided::Connecting | Guided::Scanning) {
                        self.guided = Guided::Off;
                    }
                    self.scanning = false;
                    self.live_on = false;
                    if m.contains("speaks to an adapter") {
                        // one program per adapter: another (icarus record, its service) holds it; said as it is
                        self.status = format!(
                            "The adapter is in use: {m}. A recording by icarus record (or its service) holds it: \
                             systemctl --user stop icarus-record to use it here."
                        );
                        self.error = None;
                    } else if self.auto && !self.user_off {
                        // asleep or out of reach: say so, and try again shortly (keep_trying)
                        self.status = format!(
                            "Waiting for the adapter ({}): start the engine, or switch the ignition on and press its button.",
                            self.adapter.trim()
                        );
                        self.error = None;
                    } else {
                        self.status = "Something went wrong.".into();
                        self.error = Some(m);
                    }
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
                    if self.guided == Guided::Scanning {
                        self.guided = Guided::Done;
                    }
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
                Event::ServiceDone(r) => match r {
                    Ok(m) => {
                        self.status = m;
                        self.error = None;
                    }
                    Err(m) => {
                        self.status = "The routine did not run to the end.".into();
                        self.error = Some(m);
                    }
                },
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
                Event::ClearFailed(m) => {
                    // said as it is: a refused clear is not the adapter going to sleep (`Failed` under `auto`)
                    self.status = "The clear did not run to the end.".into();
                    self.error = Some(m);
                }
                Event::Cleared(msg, info) => {
                    self.status = msg;
                    self.error = None;
                    self.info = Some(*info);
                    self.refresh_tires();
                    self.modules.clear();
                    self.scan_note =
                        "Codes were cleared: scan again to see the modules now.".into();
                }
                Event::ReportSaved(p) => {
                    self.status = format!("Report saved: {p}");
                    self.last_report = Some(p);
                }
                Event::ModuleData(r) => {
                    self.reading_data = false;
                    self.status = match &r {
                        Ok(v) => format!("Module data read with the set {}.", v.set),
                        Err(_) => "Module data could not be read.".into(),
                    };
                    self.module_data = Some(r.map(|b| *b));
                }
                Event::Recording(r) => self.on_recording(r),
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

    /// Basic's guided scan (ICA-75): one button from connecting to every module's codes, each in plain words with how soon
    /// it needs attention and what to check first, most urgent first.
    fn guided_scan(&mut self, ui: &mut egui::Ui, pal: &theme::Palette) {
        let busy = matches!(self.guided, Guided::Connecting | Guided::Scanning) || self.scanning;
        ui.horizontal(|ui| {
            let label = if self.guided == Guided::Done {
                "Scan again"
            } else {
                "Scan my vehicle"
            };
            if ui
                .add_enabled(
                    !busy,
                    egui::Button::new(RichText::new(label).size(16.0).strong()),
                )
                .clicked()
            {
                self.error = None;
                if self.link == Link::On {
                    self.guided = Guided::Scanning;
                    self.scanning = true;
                    self.live_on = false;
                    let _ = self.tx.send(Cmd::Scan {
                        profile: None,
                        sweep: false,
                    });
                } else {
                    self.guided = Guided::Connecting;
                    self.link = Link::Busy;
                    self.user_off = false;
                    self.auto = self.adapter.trim() != "sim";
                    self.last_try = std::time::Instant::now();
                    let _ = self.tx.send(Cmd::Connect(self.adapter.trim().to_string()));
                }
            }
            if busy {
                ui.spinner();
                ui.label(match self.guided {
                    Guided::Connecting => "Connecting to the adapter...".to_string(),
                    _ => format!(
                        "Asking every module for its trouble codes... ({} answered so far)",
                        self.modules.len()
                    ),
                });
            }
        });
        if self.guided == Guided::Off {
            ui.add(egui::Label::new(RichText::new("Plug the adapter into the OBD port (under the dashboard, driver's side), switch the ignition on (the engine can stay off), then press Scan my vehicle. Nothing is changed in the vehicle.").weak()).wrap());
            return;
        }
        if self.guided != Guided::Done {
            return;
        }
        // every code: the engine's OBD-II codes from connecting, and each module's own
        let profile = self.info.as_ref().and_then(|i| i.profile.clone());
        let mut found: Vec<(String, String)> = Vec::new();
        for (_, module, codes) in self
            .info
            .as_ref()
            .map(|i| i.codes.clone())
            .unwrap_or_default()
        {
            for c in codes {
                found.push((c, module.clone()));
            }
        }
        for m in &self.modules {
            if let Ok(read) = &m.dtcs {
                for d in &read.codes {
                    found.push((d.code.clone(), m.addr.name.clone()));
                }
            }
        }
        let rows = guided_rows(&found, profile.as_ref());
        ui.add_space(6.0);
        if rows.is_empty() {
            ui.label(
                RichText::new(format!(
                    "No trouble codes: none of the {} modules that answered has one stored.",
                    self.modules.len()
                ))
                .color(pal.green)
                .strong(),
            );
            return;
        }
        ui.label(
            RichText::new(format!(
                "{} trouble code{} in {} module{}, most urgent first:",
                rows.len(),
                if rows.len() == 1 { "" } else { "s" },
                self.modules.len(),
                if self.modules.len() == 1 { "" } else { "s" }
            ))
            .strong(),
        );
        let mut last: Option<dtc::Urgency> = None;
        for (u, code, text, why, first, modules) in rows {
            if last != Some(u) {
                ui.add_space(8.0);
                let color = match u {
                    dtc::Urgency::Serious => pal.red,
                    dtc::Urgency::Soon => pal.amber,
                    dtc::Urgency::CanWait => pal.dim,
                };
                ui.label(RichText::new(u.label()).color(color).strong());
                last = Some(u);
            }
            ui.label(RichText::new(format!("{code}  {text}")).strong());
            ui.add(egui::Label::new(RichText::new(format!("{why} ({modules})")).weak()).wrap());
            if let Some(f) = first {
                ui.add(egui::Label::new(RichText::new(format!("First check: {f}"))).wrap());
            }
            ui.add_space(4.0);
        }
        ui.add_space(6.0);
        ui.add(egui::Label::new(RichText::new("This is where to start, not a diagnosis. Each code has more on the Modules page (causes, parts, a web search); clearing codes is there too.").small().weak()).wrap());
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
                        self.user_off = false;
                        self.auto = self.adapter.trim() != "sim";
                        self.last_try = std::time::Instant::now();
                        let _ = self.tx.send(Cmd::Connect(self.adapter.trim().to_string()));
                    }
                }
                Link::Busy => {
                    ui.spinner();
                }
                Link::On => {
                    if ui.button("Disconnect").clicked() {
                        self.user_off = true;
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
        card(ui, pal, "Trouble codes", |ui| self.guided_scan(ui, pal));
        // not connected: the card above says how to start
        let Some(info) = &self.info else {
            return;
        };
        card(ui, pal, "Vehicle", |ui| {
            egui::Grid::new("veh")
                .num_columns(2)
                .spacing(vec2(16.0, 6.0))
                .show(ui, |ui| {
                    row(ui, "VIN", info.vin.as_deref().unwrap_or("(not given)"));
                    row(
                        ui,
                        "Writes",
                        if icarus_obd::lock::session_read_only() {
                            "none: this session is read-only (--read-only or ICARUS_READ_ONLY=1)"
                        } else if info.vin.as_deref().is_some_and(|v| self.is_read_only(v)) {
                            "none: Read-Only is on (Garage)"
                        } else {
                            "allowed, each with its own checks (Read-Only is off)"
                        },
                    );
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
                        // in the chosen units, as live values are
                        for (p, v) in &f.values {
                            if let Some(x) = j1979::pid(*p) {
                                row(ui, x.name, &logs::value_text(x.unit, *v, 1));
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
        card(ui, pal, "Code history", |ui| match &info.history {
            Err(why) => {
                ui.colored_label(pal.amber, format!("The history could not be kept: {why}"));
            }
            Ok(h) if h.is_empty() => {
                ui.label(
                    RichText::new(if info.vin.is_some() {
                        "No code seen on this vehicle yet. Every reading is kept in its folder (Logs/codes-history.tsv)."
                    } else {
                        "The VIN could not be read, so no history is kept (the codes could belong to any vehicle)."
                    })
                    .weak(),
                );
            }
            Ok(h) => {
                ui.label(
                    RichText::new(
                        "When each code came and went, from every reading of this vehicle's codes.",
                    )
                    .weak(),
                );
                for sp in h {
                    ui.horizontal_wrapped(|ui| {
                        dot(
                            ui,
                            if sp.gone.is_none() {
                                pal.amber
                            } else {
                                pal.green
                            },
                            sp.gone.is_none(),
                        );
                        ui.label(icarus_obd::history::describe(sp));
                    });
                }
            }
        });
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
                self.ask_nhtsa(ui.ctx(), move || {
                    Bg::Vin(icarus_obd::nhtsa::decode_vin(&v).map(Box::new))
                });
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
            self.confirms.clear =
                self.ask((None, "every engine and transmission module (OBD-II)".into()));
        }
    }

    /// The connected vehicle's saved tire setting, read again (after connecting, or saving one).
    fn refresh_tires(&mut self) {
        self.tires = self
            .info
            .as_ref()
            .and_then(|i| i.vin.as_deref())
            .and_then(icarus_obd::garage::tires)
            .and_then(|t| t.factor().ok().map(|f| (t, f)));
    }

    /// "Save a report and clear" in the clear's confirmation: the clear goes to the worker with the connection it was asked
    /// on, and the confirmation closes.
    fn clear_yes(&mut self, target: ClearTarget, on: worker::Connection) {
        self.live_on = false;
        let _ = self.tx.send(Cmd::Clear { target, on });
        self.status = "Saving a report, then clearing...".into();
        self.confirms.clear = None;
    }

    /// A confirmation of `what` on the connection open now (None when there is none: nothing is asked).
    fn ask<T>(&self, what: T) -> Option<confirm::Asked<T>> {
        self.info.as_ref().map(|i| confirm::Asked {
            what,
            on: i.connection.clone(),
        })
    }

    /// Whether this vehicle's Read-Only switch is on.
    fn is_read_only(&self, vin: &str) -> bool {
        self.read_only.contains(&vin.trim().to_uppercase())
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
            ui.checkbox(&mut self.sweep, "every request ID 700-7F7 (slower)")
                .on_hover_text("Never 7DF or an answer ID (7E8-7EF, or one a vehicle profile names): a request there would look like another module's answer.");
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
        card(ui, pal, "Module data", |ui| {
            ui.label(RichText::new("Each module's own values (a dealer tool's data list), from the module-data set that fits this vehicle (icarus import obdb or abrp). Read only: data reads, paced; airbag modules identity only.").weak());
            ui.horizontal(|ui| {
                if self.reading_data {
                    ui.spinner();
                } else if ui.button("Read module data").clicked() {
                    self.reading_data = true;
                    self.live_on = false;
                    let _ = self.tx.send(Cmd::ModuleData);
                }
            });
            match &self.module_data {
                None => {}
                Some(Err(why)) => {
                    ui.colored_label(pal.amber, why);
                }
                Some(Ok(v)) => {
                    for n in &v.notes {
                        ui.label(RichText::new(n).small().weak());
                    }
                    for m in &v.modules {
                        egui::CollapsingHeader::new(format!("{}  {}", m.label, m.name))
                            .id_salt(("module-data", &m.label))
                            .default_open(true)
                            .show(ui, |ui| {
                                egui::Grid::new(("md", &m.label))
                                    .num_columns(2)
                                    .spacing(vec2(16.0, 2.0))
                                    .show(ui, |ui| {
                                        for (name, val, checked) in &m.values {
                                            ui.label(name);
                                            ui.label(if *checked {
                                                format!("{val}  (checked)")
                                            } else {
                                                val.clone()
                                            });
                                            ui.end_row();
                                        }
                                    });
                                for n in &m.notes {
                                    ui.label(RichText::new(n).small().weak());
                                }
                            });
                    }
                    if !v.attribution.is_empty() {
                        ui.label(RichText::new(&v.attribution).small().color(pal.dim));
                    }
                }
            }
        });
        let mut links: Vec<String> = Vec::new();
        let all = icarus_obd::vehicles::load(icarus_obd::vehicles::user_dir().as_deref()).0;
        let chosen = icarus_obd::vehicles::find(&all, &self.profile_choice)
            .ok()
            .filter(|_| !self.profile_choice.is_empty());
        let profile = chosen.or(self.info.as_ref().and_then(|i| i.profile.as_ref()));
        let vehicle = self.info.as_ref().and_then(|i| i.vehicle.as_ref());
        // the make the worker clears on: the VIN's profile, not one chosen for its explanations
        let clear_make = self
            .info
            .as_ref()
            .and_then(|i| i.profile.as_ref())
            .map(|p| p.id.clone());
        let mut ask_clear: Option<(u16, u16, Dialect, String)> = None;
        for m in &self.modules {
            let adv = self.levels.advanced;
            let title = module_title(&m.addr, adv);
            card(ui, pal, &title, |ui| {
                if adv && !m.addr.checked {
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
                // Advanced: what it did not give, each with the request and why (a refusal's words)
                if adv && !m.notes.is_empty() {
                    ui.label(RichText::new("Not given:").small().strong());
                    for n in &m.notes {
                        ui.label(RichText::new(n).small().weak());
                    }
                }
                match &m.dtcs {
                    Err(why) if adv => {
                        ui.label(
                            RichText::new("None of Icarus's requests gave its trouble codes:")
                                .weak(),
                        );
                        ui.add(egui::Label::new(RichText::new(why).small().weak()).wrap());
                    }
                    Err(_) => {
                        ui.label(
                            RichText::new("This module would not tell us its trouble codes.")
                                .weak(),
                        );
                    }
                    Ok(read) => {
                        let (d, codes) = (&read.dialect, &read.codes);
                        let lang = if *d == Dialect::Uds { "UDS" } else { "KWP2000" };
                        // a module listing every code it can set, none failed (UDS 19 02 FF), is clean (ICA-207)
                        if let Some(clean) = clean_codes_label(read, lang) {
                            ui.label(RichText::new(clean).color(pal.green));
                        }
                        for c in codes {
                            let confirmed = c.states.iter().any(|s| {
                                matches!(*s, "confirmed" | "stored" | "warning light requested")
                            });
                            let extra = code_extra(&c.states, c.failure_type, lang, adv);
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
                        if let Some(more) = read.others() {
                            ui.label(RichText::new(more).small().color(pal.dim));
                        }
                        // airbag modules: never cleared by Icarus (clear::refused); say so where the button would be. The
                        // clear path takes 11-bit IDs on the main bus only: a 29-bit module, one answering outside
                        // 700-7FF (GM's body range) or one on Ford's MS-CAN gets no button
                        let small = |id: u32| u16::try_from(id).ok().filter(|i| *i <= 0x7FF);
                        let airbag = icarus_obd::actuator::is_airbag(m.addr.tx, &m.addr.name)
                            || icarus_obd::actuator::is_airbag(m.addr.rx(), "")
                            || m.addr.kind.eq_ignore_ascii_case("airbag");
                        // as the worker judges it (ICA-200): on the make the VIN picks, where its profile names no
                        // airbag module a module of unknown kind could be it, so it gets no button and says why
                        let unknown = icarus_obd::clear::refused_on(
                            &all,
                            clear_make.as_deref(),
                            m.addr.tx,
                            m.addr.rx(),
                            &m.addr.name,
                        )
                        .filter(|why| {
                            small(m.addr.tx).is_some() && why.contains("could be the airbag module")
                        });
                        let clear_ids = small(m.addr.tx)
                            .zip(small(m.addr.rx()))
                            .filter(|(t, r)| {
                                icarus_obd::clear::refused_on(
                                    &all,
                                    clear_make.as_deref(),
                                    (*t).into(),
                                    (*r).into(),
                                    &m.addr.name,
                                )
                                .is_none()
                            })
                            .filter(|_| m.addr.ext.is_none() && !m.addr.on_ms_can());
                        if !codes.is_empty() && !airbag {
                            if let Some(why) = &unknown {
                                ui.label(
                                    RichText::new(format!(
                                        "Not cleared by Icarus: {}.",
                                        why.trim_start_matches("refused: ")
                                    ))
                                    .small()
                                    .color(pal.dim),
                                );
                            }
                        }
                        if !codes.is_empty() && airbag {
                            ui.label(
                                RichText::new(
                                    "Airbag codes are never cleared by Icarus: a dealer or an airbag specialist \
                                     clears them after the repair.",
                                )
                                .small()
                                .color(pal.dim),
                            );
                        }
                        // a key module (immobilizer, smart key, certification) by its profile kind or name: no button
                        // either (clear_module refuses it by address too, `fence`)
                        let key = icarus_obd::fence::Kind::of(&m.addr.kind, &m.addr.name).is_some();
                        if !codes.is_empty() && key {
                            ui.label(
                                RichText::new(
                                    "A key module's codes are never cleared by Icarus: its identity and codes are \
                                     read, nothing else.",
                                )
                                .small()
                                .color(pal.dim),
                            );
                        }
                        // a module behind a gateway is read only in this build (no clear path carries its address)
                        if let Some((tx, rx)) =
                            clear_ids.filter(|_| !codes.is_empty() && !airbag && !key)
                        {
                            if ui.button("Clear this module's codes...").clicked() {
                                ask_clear = Some((
                                    tx,
                                    rx,
                                    *d,
                                    format!("{:03X} {}", m.addr.tx, m.addr.name),
                                ));
                            }
                        }
                    }
                }
            });
        }
        if let Some((tx, rx, d, label)) = ask_clear {
            self.confirms.clear = self.ask((Some((tx, rx, d)), label));
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
        if self.live_on && self.gauges_on {
            ui.label(RichText::new("The trail gauges are being read (their page); Start here reads these values instead.").weak());
        }
        ui.horizontal(|ui| {
            if self.live_on && !self.gauges_on {
                if ui.button("Stop").clicked() {
                    self.live_on = false;
                    self.stop_recording();
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
                self.gauges_on = false;
                self.send_live();
            }
            if let Some((rows, n, file)) = self.recording.as_ref().map(|r| (r.rows, r.values, r.path.file_name().map(|f| f.to_string_lossy().into_owned()).unwrap_or_default())) {
                if ui.button("Stop recording").clicked() {
                    self.stop_recording();
                }
                ui.label(RichText::new(format!("● recording {n} values, {rows} rows")).color(pal.red)).on_hover_text(file);
            } else if ui
                .add_enabled(!self.scanning, egui::Button::new("Record drive"))
                .on_hover_text(
                    "Record every value the vehicle offers (fuel trims, timing, air flow, throttle and mixture every sweep, \
                     the rest every few seconds) and each module's own values (transmission temperature, knock...: the \
                     vehicle's module-data set, answers kept; knock, gear and lock-up every sweep, Settings changes \
                     which) until stopped, keeping the laptop awake and going on through adapter drops; a tone and a \
                     notice when it starts, stalls, resumes and stops (Settings); see it on the Logs page",
                )
                .clicked()
            {
                self.start_recording();
            }
            if self.recording.is_none() && !self.gps_src.trim().is_empty() {
                ui.checkbox(&mut self.record_gps, "with GPS positions")
                    .on_hover_text("Latitude, longitude and GPS speed from the Tools page's GPS source, in the recording's file only");
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
        // ICA-218: the module values read every sweep, and the rows a second once known
        if let Some(r) = &self.recording {
            for line in &r.said {
                ui.add(egui::Label::new(RichText::new(line).weak()).wrap());
            }
        }
        // Advanced: the request going out now (ICA-76)
        let adv = self.levels.advanced;
        if adv {
            if let Some((_, id, b)) = self.sent_log.back() {
                let hex = b
                    .iter()
                    .map(|x| format!("{x:02X}"))
                    .collect::<Vec<_>>()
                    .join(" ");
                ui.label(
                    RichText::new(format!("Last request: {}  {hex}", id_text(*id)))
                        .small()
                        .monospace()
                        .weak(),
                );
            }
        }
        ui.add_space(6.0);
        ui.columns(2, |cols| {
            // every value at once, or none: the same as ticking or clearing each box (a recording keeps its own values)
            cols[0].horizontal(|ui| {
                let mut changed = false;
                if ui
                    .add_enabled(self.chosen.len() < offered.len(), egui::Button::new("Select all"))
                    .on_hover_text("Every value the vehicle offers; with many values each one updates more slowly")
                    .clicked()
                {
                    self.chosen.extend(offered.iter().copied());
                    changed = true;
                }
                if ui
                    .add_enabled(!self.chosen.is_empty(), egui::Button::new("Clear selections"))
                    .clicked()
                {
                    self.chosen.clear();
                    changed = true;
                }
                if changed && self.live_on {
                    self.send_live();
                }
            });
            cols[0].add_space(4.0);
            egui::ScrollArea::vertical()
                .id_salt("pick")
                .show(&mut cols[0], |ui| {
                    for p in &offered {
                        let d = j1979::pid(*p).unwrap();
                        let mut on = self.chosen.contains(p);
                        if ui
                            .checkbox(&mut on, pid_label(*p, d.name, adv))
                            .on_hover_text(d.about)
                            .changed()
                        {
                            if on {
                                self.chosen.insert(*p);
                            } else {
                                self.chosen.remove(p);
                            }
                            if self.live_on {
                                self.send_live();
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
                        card(ui, pal, &pid_label(*p, d.name, adv), |ui| {
                            ui.horizontal(|ui| {
                                ui.label(
                                    RichText::new(last.map_or("-".to_string(), |v| {
                                        format!("{:.2}", logs::shown(d.unit, v).0)
                                    }))
                                    .size(24.0)
                                    .strong(),
                                );
                                ui.label(logs::shown(d.unit, 0.0).1);
                                // both units chosen: the other beside it
                                if let Some((x, u)) = last.and_then(|v| logs::also(d.unit, v)) {
                                    ui.add_space(8.0);
                                    ui.label(RichText::new(format!("{x:.1} {u}")).size(16.0).weak());
                                }
                            });
                            if let (0x0D, Some((_, f)), Some(v)) = (*p, &self.tires, last) {
                                let (x, unit) = logs::shown(d.unit, v * f);
                                ui.label(
                                    RichText::new(format!(
                                        "{x:.1} {unit} corrected for the tires (x{f:.3})"
                                    ))
                                    .color(pal.accent),
                                );
                            }
                            if let Some(h) = h {
                                graph(ui, pal, h);
                            }
                            ui.label(RichText::new(d.about).small().weak());
                        });
                    }
                });
        });
    }

    /// Trail gauges (ICA-27): the trail values the vehicle gives as big readouts, colored when one wants watching (amber)
    /// or stopping (red), with the lowest and highest since they started. Read through the same live polling as Live data,
    /// in its place.
    fn page_gauges(&mut self, ui: &mut egui::Ui, pal: &theme::Palette) {
        let Some(info) = &self.info else {
            ui.label("Connect on the Vehicle page first.");
            return;
        };
        let pids = gauges::gauge_pids(&info.live);
        if pids.is_empty() {
            ui.label("This vehicle offers none of the trail values (coolant, oil and intake temperatures, voltage, speed...).");
            return;
        }
        let on = self.live_on && self.gauges_on;
        ui.horizontal(|ui| {
            if on {
                if ui.button(RichText::new("Stop").size(16.0)).clicked() {
                    self.gauges_on = false;
                    self.live_on = false;
                    // a recording goes on reading its own values
                    if self.recording.is_some() {
                        self.send_live();
                    } else {
                        let _ = self.tx.send(Cmd::Live(vec![]));
                    }
                }
                ui.spinner();
            } else if ui
                .add_enabled(!self.scanning, egui::Button::new(RichText::new("Start").size(16.0)))
                .clicked()
            {
                self.gauges_on = true;
                self.live_on = true;
                self.send_live();
            }
            ui.label(
                RichText::new(format!(
                    "{} values for the trail. Amber: watch it; red: stop and look. Lowest and highest since they started.",
                    pids.len()
                ))
                .weak(),
            );
        });
        ui.add_space(8.0);
        let per_row = ((ui.available_width() / 250.0).floor() as usize).clamp(1, 4);
        for row in pids.chunks(per_row) {
            ui.columns(per_row, |cols| {
                for (c, p) in cols.iter_mut().zip(row) {
                    self.gauge_tile(c, pal, *p);
                }
            });
            ui.add_space(10.0);
        }
        ui.label(
            RichText::new("The bands are general ones for a petrol engine, not this vehicle's own limits: point at a gauge for them.")
                .small()
                .weak(),
        );
    }

    fn gauge_tile(&self, ui: &mut egui::Ui, pal: &theme::Palette, pid: u8) {
        let (Some(d), Some(g)) = (
            j1979::pid(pid),
            gauges::GAUGES.iter().find(|g| g.pid == pid),
        ) else {
            return;
        };
        let h = self.history.get(&pid);
        let last = h.and_then(|h| h.back()).map(|x| x.1);
        let level = last.map_or(gauges::Level::Fine, |v| gauges::level(pid, v));
        let (color, edge, width) = match level {
            gauges::Level::Fine => (pal.text, pal.border, 1.0),
            gauges::Level::Watch => (pal.amber, pal.amber, 2.5),
            gauges::Level::Act => (pal.red, pal.red, 3.0),
        };
        // speed corrected for the tires when they are set (Garage page)
        let f = match (pid, &self.tires) {
            (0x0D, Some((_, f))) => *f,
            _ => 1.0,
        };
        // volts and pressure move in small steps: one decimal
        let dec = if matches!(pid, 0x42 | 0x33) { 1 } else { 0 };
        let unit = logs::shown(d.unit, 0.0).1;
        egui::Frame::new()
            .fill(pal.field)
            .stroke(Stroke::new(width, edge))
            .corner_radius(CornerRadius::same(12))
            .inner_margin(Margin::symmetric(14, 12))
            .show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.label(RichText::new(d.name).strong());
                ui.horizontal(|ui| {
                    let text = last.map_or("-".to_string(), |v| {
                        format!("{:.dec$}", logs::shown(d.unit, v * f).0)
                    });
                    ui.label(RichText::new(text).size(44.0).strong().color(color));
                    ui.label(RichText::new(unit).size(16.0).weak());
                });
                let span =
                    h.and_then(gauges::range)
                        .map_or("low - · high -".to_string(), |(lo, hi)| {
                            format!(
                                "low {:.dec$} · high {:.dec$}",
                                logs::shown(d.unit, lo * f).0,
                                logs::shown(d.unit, hi * f).0
                            )
                        });
                ui.label(RichText::new(span).weak());
                if f != 1.0 {
                    ui.label(
                        RichText::new(format!("corrected for the tires (x{f:.3})"))
                            .small()
                            .color(pal.accent),
                    );
                }
            })
            .response
            .on_hover_text(g.about);
    }

    fn page_tests(&mut self, ui: &mut egui::Ui, pal: &theme::Palette) {
        let Some(info) = &self.info else {
            ui.label("Connect on the Vehicle page first. (The simulated vehicle with its engine off is sim:off.)");
            return;
        };
        let tests = info.tests.clone();
        let services = info.services.clone();
        let on = info.connection.clone();
        if !services.is_empty() {
            card(ui, pal, "Service routines", |ui| {
                ui.label(RichText::new("Relearns, resets, TPMS, coding: each the maker's own routine, run once after a confirmation. Its conditions, the battery and the Read-Only switch are checked first; every run is logged.").weak());
                for s in &services {
                    ui.horizontal_wrapped(|ui| {
                        if ui.button("Run...").clicked() {
                            self.confirms.service = Some(confirm::Asked {
                                what: s.clone(),
                                on: on.clone(),
                            });
                        }
                        ui.label(RichText::new(&s.name).strong());
                        ui.label(
                            RichText::new(format!(
                                "{:03X}, routine {:04X}{}{}",
                                s.module,
                                s.routine,
                                if s.wired { ", cable only" } else { "" },
                                if s.checked {
                                    ""
                                } else {
                                    ", not yet checked on a vehicle"
                                }
                            ))
                            .small()
                            .weak(),
                        );
                    });
                }
            });
        }
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

    /// The profile the Bus page names IDs from: the one chosen on the Modules page, else the connected vehicle's (VIN).
    fn refresh_bus_names(&mut self) {
        let from_vin = self.info.as_ref().and_then(|i| i.profile.as_ref());
        let key = if self.profile_choice.is_empty() {
            from_vin.map(|p| p.id.clone()).unwrap_or_default()
        } else {
            self.profile_choice.clone()
        };
        if self.bus_names.0 == key {
            return;
        }
        let names = if self.profile_choice.is_empty() {
            icarus_obd::vehicles::bus_names(from_vin)
        } else {
            let all = icarus_obd::vehicles::load(icarus_obd::vehicles::user_dir().as_deref()).0;
            icarus_obd::vehicles::bus_names(icarus_obd::vehicles::find(&all, &key).ok())
        };
        self.bus_names = (key, names);
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
        // a CAN database names the bytes (ICA-88)
        ui.horizontal(|ui| {
            ui.label("CAN database (.dbc)");
            ui.add(
                egui::TextEdit::singleline(&mut self.bus_dbc_path)
                    .hint_text("optional: a vehicle's .dbc, to name the signals")
                    .desired_width(320.0),
            );
            if ui.button("Open").clicked() {
                let p = self.bus_dbc_path.trim().to_string();
                match std::fs::read_to_string(&p) {
                    Ok(t) => {
                        let d = icarus_core::dbc::parse(&t);
                        self.bus_dbc_note = format!(
                            "{} message(s){}",
                            d.messages.len(),
                            if d.skipped.is_empty() {
                                String::new()
                            } else {
                                format!("; left out: {}", d.skipped.join("; "))
                            }
                        );
                        self.bus_dbc = Some(d);
                    }
                    Err(e) => {
                        self.bus_dbc_note = format!("{p}: {e}");
                        self.bus_dbc = None;
                    }
                }
            }
            if self.bus_dbc.is_some() && ui.button("Close it").clicked() {
                self.bus_dbc = None;
                self.bus_dbc_note.clear();
            }
        });
        if !self.bus_dbc_note.is_empty() {
            ui.label(RichText::new(&self.bus_dbc_note).small().weak());
        }
        ui.add_space(8.0);
        card(ui, pal, "Listen with another device", |ui| {
            self.listen_body(ui, pal)
        });
        self.refresh_bus_names();
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
        ui.horizontal_top(|ui| {
            egui::ScrollArea::vertical()
                .id_salt("bus-ids")
                .max_height(420.0)
                .show(ui, |ui| {
                    egui::Grid::new("bus-table")
                        .striped(true)
                        .spacing(vec2(12.0, 2.0))
                        .show(ui, |ui| {
                            for h in ["ID", "what", "Hz", "count", "bytes"] {
                                ui.label(RichText::new(h).small().weak());
                            }
                            if self.bus_dbc.is_some() {
                                ui.label(RichText::new("signals").small().weak());
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
                                // what the profile calls it, cut to fit (a cut label shows the whole name on hover)
                                let what =
                                    self.bus_names.1.get(id).map(String::as_str).unwrap_or("");
                                ui.allocate_ui_with_layout(
                                    vec2(200.0, 14.0),
                                    egui::Layout::left_to_right(egui::Align::Center),
                                    |ui| {
                                        ui.add(
                                            egui::Label::new(RichText::new(what).small())
                                                .truncate(),
                                        )
                                    },
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
                                if let Some(d) = &self.bus_dbc {
                                    ui.label(RichText::new(dbc_cell(d, *id, data)).small());
                                }
                                ui.end_row();
                            }
                        });
                });
            egui::ScrollArea::vertical()
                .id_salt("bus-log")
                .max_height(420.0)
                .stick_to_bottom(true)
                .show(ui, |ui| {
                    ui.vertical(|ui| {
                        if b.log.is_empty() {
                            ui.label(RichText::new("No diagnostic messages yet.").weak());
                        }
                        for l in &b.log {
                            ui.label(RichText::new(l).monospace().small());
                        }
                    });
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

    /// The window's own settings (the gear): the levels (Advanced shows more and unlocks module control; Tuning shows only while
    /// Advanced is on and asks first), the units, the theme, and the adapters, protocols and devices Icarus works with.
    fn page_app(&mut self, ui: &mut egui::Ui, pal: &theme::Palette) {
        ui.heading("Settings");
        ui.label(RichText::new("Icarus on this computer.").weak());
        ui.add_space(10.0);
        // each switch on its row, its words below it (wrapped: a grid's cells do not wrap)
        let row = |ui: &mut egui::Ui, name: &str, on: &mut bool, words: &str| -> bool {
            let changed = ui
                .horizontal(|ui| {
                    let c = switch(ui, pal, on).changed();
                    ui.label(RichText::new(name).strong());
                    c
                })
                .inner;
            ui.add(egui::Label::new(RichText::new(words).weak()).wrap());
            ui.add_space(10.0);
            changed
        };
        let mut adv = self.levels.advanced;
        if row(ui, "Advanced", &mut adv, "More detail on every page, and control of each module: actuator tests (the throttle and the rest), module settings, service routines, learning from another tool and the bus view.") {
            self.levels.set_advanced(adv);
            self.save_levels();
        }
        if self.levels.advanced {
            let mut tun = self.levels.tuning;
            if row(
                ui,
                "Tuning",
                &mut tun,
                "Maps and tuning files for the engine and the other computers.",
            ) {
                if tun {
                    self.tuning_ask = true;
                } else {
                    self.levels.tuning = false;
                    self.save_levels();
                }
            }
        }
        ui.add_space(12.0);
        ui.label(RichText::new("Every change to a vehicle keeps its own checks, and a vehicle set to Read-Only in the Garage stays read-only at every level.").weak());
        ui.add_space(18.0);

        // units and looks (ICA-79): kept with the levels, on this computer
        ui.label(RichText::new("Units").strong());
        let mut u = self.levels.units;
        for x in levels::Units::ALL {
            ui.radio_value(&mut u, x, x.label());
        }
        ui.add(egui::Label::new(RichText::new("For speed, temperature, pressure, distance, fuel flow and torque on every page. Vehicles report metric; drive summaries give both.").weak()).wrap());
        if u != self.levels.units {
            self.levels.units = u;
            self.save_levels();
        }
        ui.add_space(14.0);
        ui.label(RichText::new("Theme").strong());
        let mut t = self.levels.theme;
        for x in levels::Theme::ALL {
            ui.radio_value(&mut t, x, x.label());
        }
        if t != self.levels.theme {
            self.levels.theme = t;
            self.save_levels();
        }
        ui.add_space(14.0);
        // ICA-209: what a recording tells the driver without a look at the screen
        ui.label(RichText::new("Recording a drive").strong());
        let mut sound = !self.levels.record_quiet;
        ui.checkbox(
            &mut sound,
            "A tone and a notice when a recording starts, stalls, resumes and stops",
        );
        ui.add(egui::Label::new(RichText::new("Through the desktop's sound and notifications. icarus record has its own: --quiet, or ICARUS_QUIET=1.").weak()).wrap());
        if sound == self.levels.record_quiet {
            self.levels.record_quiet = !sound;
            self.save_levels();
            let _ = self.tx.send(Cmd::RecordQuiet(self.levels.record_quiet));
        }
        // ICA-218: the module values read every sweep, not only in their turn
        ui.add_space(8.0);
        ui.label("Module values read every sweep");
        ui.add(egui::Label::new(RichText::new(format!(
            "Whole words of a value's name, one a line (empty: the defaults; none alone: every value in turn; # starts a comment line). The requests carrying them are read every sweep, up to {} a sweep (more take turns); each costs the rows a second a little, the rest stay in turn. icarus record takes this too, unless given --watch.",
            icarus_obd::drive::PRIORITY_PER_SWEEP
        )).weak()).wrap());
        let edit = ui.add(
            egui::TextEdit::multiline(&mut self.record_watch)
                .desired_rows(4)
                .desired_width(320.0),
        );
        let defaults = ui.button("Defaults").on_hover_text(
            "Knock feedback, gear, lock-up, the shift lever's switches and sport mode",
        );
        if defaults.clicked() {
            self.record_watch = watch_field(
                &icarus_obd::drive::WATCH_DEFAULT
                    .iter()
                    .map(|w| w.to_string())
                    .collect::<Vec<_>>(),
            );
        }
        // kept as it is typed, so `icarus record --auto` (the service) reads the latest whenever it starts a drive
        if edit.changed() || defaults.clicked() {
            let names = icarus_obd::drive::parse_watch(&self.record_watch);
            if let Some(p) = icarus_obd::drive::watch_path() {
                if let Err(e) = icarus_obd::drive::save_watch(&p, &names) {
                    self.status = format!("The watched module values were not kept: {e}");
                }
            }
        }
        ui.add_space(18.0);

        // the devices and their roles (ICA-80): talk, or listen only (never sent a request)
        ui.label(RichText::new("Devices").strong());
        ui.add(egui::Label::new(RichText::new("Talk: Icarus asks the vehicle through it. Listen only: it watches silently on the Bus page and is never sent a request (the USB cable on a Y-splitter while the Bluetooth adapter or another program talks).").weak()).wrap());
        let mut known: Vec<String> = Vec::new();
        for d in [
            Some(self.adapter.trim().to_string()),
            autoconn::remembered(),
        ]
        .into_iter()
        .flatten()
        .chain(self.roles.listen.clone())
        {
            if !d.is_empty() && !known.contains(&d) {
                known.push(d);
            }
        }
        let connected_to = (self.link == Link::On).then(|| self.adapter.trim().to_string());
        let mut change: Option<(String, bool)> = None;
        egui::Grid::new("device-roles")
            .num_columns(2)
            .show(ui, |ui| {
                for d in &known {
                    ui.label(RichText::new(d).monospace());
                    let mut listen = self.roles.is_listen(d);
                    ui.horizontal(|ui| {
                        let busy = connected_to.as_deref() == Some(d.as_str());
                        ui.add_enabled_ui(!busy, |ui| {
                            ui.radio_value(&mut listen, false, "Talk");
                            ui.radio_value(&mut listen, true, "Listen only");
                        });
                        if busy {
                            ui.label(
                                RichText::new("(connected: disconnect to change)")
                                    .small()
                                    .weak(),
                            );
                        }
                    });
                    if listen != self.roles.is_listen(d) {
                        change = Some((d.clone(), listen));
                    }
                    ui.end_row();
                }
            });
        ui.horizontal(|ui| {
            ui.add(
                egui::TextEdit::singleline(&mut self.new_device)
                    .hint_text("another device: /dev/ttyUSB0, bt:…")
                    .desired_width(220.0),
            );
            if ui
                .add_enabled(
                    !self.new_device.trim().is_empty(),
                    egui::Button::new("Add as listen only"),
                )
                .clicked()
            {
                change = Some((self.new_device.trim().to_string(), true));
                self.new_device.clear();
            }
        });
        if let Some((d, listen)) = change {
            self.roles.set_listen(&d, listen);
            if let Err(e) = self.roles.save() {
                self.error = Some(format!("The devices' roles were not saved: {e}"));
            }
        }
        ui.add_space(18.0);

        // what Icarus works with: the vehicle adapters and protocols, and the tuners and interfaces it recognises by their USB
        // IDs (icarus_obd::devices::KNOWN, the table in docs/INTERFACES.md)
        ui.label(RichText::new("Adapters and protocols").strong());
        ui.add(egui::Label::new(RichText::new("ELM327 and OBDLink (STN) adapters over USB, Bluetooth or Wi-Fi; a CAN interface through Linux's SocketCAN; a CAN-to-USB (slcan) adapter for listening only. OBD-II on every protocol those adapters speak (CAN, ISO 9141-2, KWP2000 on the K-line, J1850), and each module's own diagnostics on CAN (UDS and KWP2000).").weak()).wrap());
        ui.add_space(10.0);
        ui.label(RichText::new("Tuners and interfaces Icarus recognises").strong());
        egui::Grid::new("known-devices")
            .striped(true)
            .num_columns(3)
            .show(ui, |ui| {
                for h in ["Device", "USB ID", "What Icarus does with it"] {
                    ui.label(RichText::new(h).strong());
                }
                ui.end_row();
                for k in icarus_obd::devices::KNOWN {
                    ui.label(format!("{} {}", k.maker, k.name));
                    ui.label(match k.product {
                        Some(p) => format!("{:04x}:{p:04x}", k.vendor),
                        None => format!("{:04x} (any)", k.vendor),
                    });
                    ui.add(egui::Label::new(RichText::new(k.note).weak()).wrap());
                    ui.end_row();
                }
            });
    }

    /// A second device listening while the connected one talks (ICA-80): which one, start and stop, and what it saw beside
    /// what Icarus sent.
    fn listen_body(&mut self, ui: &mut egui::Ui, pal: &theme::Palette) {
        let devices = self.roles.listen.clone();
        if devices.is_empty() {
            ui.add(egui::Label::new(RichText::new("Set a device to listen only in Settings (the gear): the USB cable on a Y-splitter, for example, while the Bluetooth adapter talks. It is never sent a request: it watches what another device or program asks and what the modules answer.").weak()).wrap());
            return;
        }
        self.listen_pick = self.listen_pick.min(devices.len() - 1);
        let running = self.listener.is_some();
        ui.horizontal(|ui| {
            ui.add_enabled_ui(!running, |ui| {
                egui::ComboBox::from_id_salt("listen-device")
                    .selected_text(&devices[self.listen_pick])
                    .show_ui(ui, |ui| {
                        for (i, d) in devices.iter().enumerate() {
                            ui.selectable_value(&mut self.listen_pick, i, d);
                        }
                    });
                let protos = [
                    (6u8, "CAN 11-bit, 500 kbit/s"),
                    (7, "CAN 29-bit, 500 kbit/s"),
                    (8, "CAN 11-bit, 250 kbit/s"),
                    (9, "CAN 29-bit, 250 kbit/s"),
                ];
                let shown = protos
                    .iter()
                    .find(|p| p.0 == self.listen_protocol)
                    .map_or("CAN", |p| p.1);
                egui::ComboBox::from_id_salt("listen-protocol")
                    .selected_text(shown)
                    .show_ui(ui, |ui| {
                        for (n, name) in protos {
                            ui.selectable_value(&mut self.listen_protocol, n, name);
                        }
                    });
            });
            if running {
                if ui.button("Stop listening").clicked() {
                    if let Some(l) = &self.listener {
                        l.stop();
                    }
                }
                ui.spinner();
            } else if ui.button("Listen").clicked() {
                let spec = devices[self.listen_pick].clone();
                let dir = icarus_obd::drivelog::folders().into_iter().next();
                let r = self.repaint.clone();
                let l = listen::start(&spec, self.listen_protocol, dir, move || r());
                self.listen_file = l.file.clone();
                self.listen_status = format!("Starting {spec}...");
                self.heard.clear();
                self.heard_frames = 0;
                self.listener = Some(l);
            }
        });
        if !self.listen_status.is_empty() {
            ui.label(RichText::new(&self.listen_status).weak());
        }
        if let Some(f) = &self.listen_file {
            ui.label(
                RichText::new(format!(
                    "Kept in {} (icarus import buslog learns from it).",
                    f.display()
                ))
                .small()
                .weak(),
            );
        }
        let hex = |b: &[u8]| {
            b.iter()
                .map(|x| format!("{x:02X}"))
                .collect::<Vec<_>>()
                .join(" ")
        };
        let sent: Vec<(u32, Vec<u8>)> = self
            .sent_log
            .iter()
            .map(|(_, id, b)| (*id, b.clone()))
            .collect();
        let seen: Vec<(u32, Vec<u8>)> = self
            .heard
            .iter()
            .map(|f| (f.id, listen::request_bytes(f)))
            .collect();
        let matched = sent.iter().filter(|s| seen.contains(s)).count();
        ui.label(format!(
            "The listener saw {} frames; {} of the {} requests Icarus sent here were seen by it.",
            self.heard_frames,
            matched,
            sent.len()
        ));
        ui.columns(2, |cols| {
            cols[0].label(RichText::new("Sent by Icarus (the talking device)").strong());
            for (id, b) in sent.iter().rev().take(14) {
                let mark = if seen.contains(&(*id, b.clone())) {
                    "  seen by it"
                } else {
                    ""
                };
                cols[0].label(RichText::new(format!("{id:03X}  {}{mark}", hex(b))).monospace());
            }
            cols[1].label(RichText::new("Requests the listener saw").strong());
            for (id, b) in seen.iter().rev().take(14) {
                let mark = if sent.contains(&(*id, b.clone())) {
                    "  (Icarus)"
                } else {
                    ""
                };
                cols[1].label(
                    RichText::new(format!("{id:03X}  {}{mark}", hex(b)))
                        .monospace()
                        .color(if mark.is_empty() { pal.accent } else { pal.dim }),
                );
            }
        });
        ui.add(egui::Label::new(RichText::new("A request the listener saw that Icarus did not send is another device or program asking (the OBDLink app, VF Tuner): what Icarus learns from.").small().weak()).wrap());
    }

    fn save_levels(&mut self) {
        if let Err(e) = self.levels.save() {
            self.error = Some(format!("Icarus's settings were not saved: {e}"));
        }
        // a page the levels no longer show is left for the vehicle's
        if !self.levels.allows(need(self.page)) {
            self.page = Page::Vehicle;
        }
    }

    fn page_settings(&mut self, ui: &mut egui::Ui, pal: &theme::Palette) {
        let Some(info) = &self.info else {
            ui.label("Connect on the Vehicle page first. (The simulated vehicle with its engine off is sim:off.)");
            return;
        };
        let defs = info.settings.clone();
        // the connected port, as it was opened (not the adapter field, which may have been edited since)
        let wired = info.wired;
        if !defs.is_empty() && !self.settings_asked {
            self.settings_asked = true;
            let _ = self.tx.send(Cmd::SettingsRead);
        }
        card(ui, pal, "Module settings", |ui| {
            ui.label("What dealer tools call customizations: a feature a module keeps switched on or off, or a value it keeps (a chime, a lock behaviour, a light).");
            ui.label("A change writes the module's memory, so Icarus asks first, needs the conditions (by default engine off, parked) and a wired adapter, keeps the value there before in the vehicle's Logs folder, reads the new one back, and puts the old one back if the module did not take it. Never airbags, immobiliser, keys or odometer.");
        });
        if !wired {
            ui.colored_label(pal.amber, "This adapter is not a USB cable (Bluetooth, Wi-Fi, or a port Linux does not put on USB): settings can be read, but changing one needs a wired adapter (USB).");
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
        if let Some(a) = ask {
            self.confirms.setting = self.ask(a);
        }
    }

    fn page_learn(&mut self, ui: &mut egui::Ui, pal: &theme::Palette) {
        // Sarina is optional: nothing about her shows unless she is installed
        let sarina = icarus_obd::learning::sarina().is_some();
        let running = self.bus.as_ref().is_some_and(|b| b.running && b.learning);
        let other_watch = self.bus.as_ref().is_some_and(|b| b.running && !b.learning);
        card(ui, pal, "Learning from another tool", |ui| {
            ui.label("Icarus learns a vehicle's settings and output tests by watching another tool use them on it: a dealer tool, a shop scan tool, a phone app that does customizations. It only listens; the other tool does the work.");
            ui.label("1. A Y-splitter on the OBD port: the other tool on one side, Icarus's adapter on the other (or a CAN interface tapped into the bus). Ignition on, engine off.");
            ui.label("2. Start learning. Before each change in the other tool, type what you are about to do (\"seat belt chime off\") and press Mark: what the module accepts next is named by it. One change per mark works best.");
            ui.label(if sarina {
                "3. Stop. Name the values, add each to the vehicle's file; the Settings and Tests pages then offer them. The session is saved in the vehicle's Logs folder, and Sarina can read it and say what she makes of anything Icarus could not work out."
            } else {
                "3. Stop. Name the values, add each to the vehicle's file; the Settings and Tests pages then offer them. The session is saved in the vehicle's Logs folder."
            });
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
                ui.label(RichText::new(if sarina {
                    "What the other tool did that Icarus does not make into a setting or a test. Sarina can say what she makes of it."
                } else {
                    "What the other tool did that Icarus does not make into a setting or a test."
                }).weak());
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
        // the saved sessions (Ask Sarina only when Sarina is installed)
        if !self.learn_sessions.is_empty() {
            let mut ask: Option<std::path::PathBuf> = None;
            card(ui, pal, "Sessions", |ui| {
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
        if let Some((name, answer)) = self.sarina_answer.as_ref().filter(|_| sarina) {
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
                self.read_only = icarus_obd::lock::read_only();
                self.garage_details.clear();
                self.garage_edit = None;
            }
        });
        for (vin, _, _) in &self.garage {
            if !self.garage_details.contains_key(vin) {
                self.garage_details
                    .insert(vin.clone(), garage::details(vin));
            }
        }
        if self.garage.is_empty() {
            ui.label("No vehicles yet: each vehicle connected gets a folder here (OEM tunes, Custom tunes, Definitions, Reports, Logs).");
            return;
        }
        if self.garage_sel.is_none_or(|i| i >= self.garage.len()) {
            self.garage_sel = Some(0);
        }
        let mut action: Option<(&'static str, std::path::PathBuf, std::path::PathBuf)> = None;
        let mut save: Option<GarageEdit> = None;
        ui.columns(2, |cols| {
            for (i, (vin, name, _)) in self.garage.iter().enumerate() {
                let details = self.garage_details.get(vin).cloned().unwrap_or_default();
                let shown = garage::label(vin, name.as_deref().map(Described::parse).as_ref(), &details);
                if cols[0].selectable_label(self.garage_sel == Some(i), shown).clicked() {
                    self.garage_sel = Some(i);
                }
            }
            let ui = &mut cols[1];
            let Some((vin, name, dir)) = self.garage_sel.and_then(|i| self.garage.get(i)).cloned() else {
                ui.label("Choose a vehicle.");
                return;
            };
            let described = name.as_deref().map(Described::parse);
            let saved = GarageEdit::of(&vin, described.as_ref(), &self.garage_details.get(&vin).cloned().unwrap_or_default());
            if self.garage_edit.as_ref().is_none_or(|e| e.vin != vin) {
                self.garage_edit = Some(saved.clone());
            }
            let edit = self.garage_edit.as_mut().expect("just set");
            // its own name at the top, edited where it is shown
            let title = ui.add(
                egui::TextEdit::singleline(&mut edit.nickname)
                    .font(egui::FontId::proportional(19.0))
                    .hint_text("Give it a name")
                    .desired_width(f32::INFINITY),
            );
            let enter = title.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            ui.add_space(4.0);
            egui::Grid::new(("garage-details", &vin)).num_columns(2).spacing(vec2(12.0, 6.0)).show(ui, |ui| {
                let field = |ui: &mut egui::Ui, label: &str, value: &mut String, hint: &str| {
                    ui.label(RichText::new(label).weak());
                    ui.add(egui::TextEdit::singleline(value).hint_text(hint).desired_width(220.0));
                    ui.end_row();
                };
                field(ui, "Year", &mut edit.year, "not set");
                field(ui, "Make", &mut edit.make, "not set");
                field(ui, "Model", &mut edit.model, "not set");
                field(ui, "Sub-model", &mut edit.sub_model, "not set");
                field(ui, "Engine size", &mut edit.engine_size, "not set (3.5L)");
                field(ui, "Engine", &mut edit.engine, "not set (V6)");
                field(ui, "Drivetrain", &mut edit.drivetrain, "not set");
                field(ui, "Transmission", &mut edit.transmission, "not set");
                field(ui, "Miles", &mut edit.miles, "not set");
                ui.label(RichText::new("VIN").weak());
                ui.label(&vin);
                ui.end_row();
                ui.label(RichText::new("Read-Only").weak());
                let mut locked = self.read_only.contains(&vin);
                if switch(ui, pal, &mut locked)
                    .on_hover_text("On: Icarus clears, tests and changes nothing on this vehicle. Codes, live data, module values, reports and recordings still work.")
                    .changed()
                {
                    action = Some((if locked { "lock" } else { "unlock" }, dir.clone(), std::path::PathBuf::from(&vin)));
                }
                ui.end_row();
            });
            let changed = *edit != saved;
            ui.horizontal(|ui| {
                if ui.add_enabled(changed, egui::Button::new("Save")).clicked() || (enter && changed) {
                    save = Some(edit.clone());
                }
                if changed && ui.button("Undo changes").clicked() {
                    *edit = saved.clone();
                }
            });
            ui.add_space(6.0);
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
                                    // a version is changes, not an image: it is seen against the OEM image it was made from
                                    if !icarus_core::version::is_version_file(&path) && ui.small_button("Open in Maps").clicked() {
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
                ui.add(egui::TextEdit::singleline(&mut self.import_path).hint_text("/path/to/file.bin, .xdf or .a2l").desired_width(f32::INFINITY));
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
                .find(|f| {
                    let n = f.0.to_lowercase();
                    n.ends_with(".xdf") || n.ends_with(".a2l")
                })
                .map(|f| dir.join("Definitions").join(f.0))
        };
        if let Some(e) = save {
            match e.parts() {
                Ok((name, details)) => {
                    let was = self
                        .garage
                        .iter()
                        .find(|(v, _, _)| *v == e.vin)
                        .and_then(|(_, n, _)| n.clone())
                        .unwrap_or_default();
                    // the description only when it changed (the vehicle's folder follows it)
                    let r = if Described::parse(&was).name() != name {
                        garage::set(&e.vin, &name)
                    } else {
                        Ok(())
                    };
                    match r.and_then(|_| garage::set_details(&e.vin, &details)) {
                        Ok(()) => {
                            self.garage = garage::vehicles();
                            self.garage_details.remove(&e.vin);
                            self.garage_edit = None;
                            self.status = "Saved.".into();
                        }
                        Err(err) => self.error = Some(err),
                    }
                }
                Err(err) => self.error = Some(err),
            }
        }
        match action {
            Some(("open", p, _)) => {
                let _ = std::process::Command::new("xdg-open").arg(&p).spawn();
            }
            Some((kind @ ("lock" | "unlock"), _, vin)) => {
                let vin = vin.display().to_string();
                match icarus_obd::lock::set(&vin, kind == "lock") {
                    Ok(()) => {
                        self.read_only = icarus_obd::lock::read_only();
                        self.status = if kind == "lock" {
                            "Read-Only on: nothing on this vehicle is cleared, tested or changed."
                                .into()
                        } else {
                            "Read-Only off.".into()
                        };
                    }
                    Err(e) => self.error = Some(e),
                }
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
                    if let Some(vin) = self.info.as_ref().and_then(|i| i.vin.clone()) {
                        ui.add_space(6.0);
                        ui.checkbox(
                            &mut self.t_at_transmission,
                            "This vehicle measures speed at the transmission output (a regear counts too)",
                        );
                        ui.horizontal(|ui| {
                            if ui
                                .button("Save for this vehicle")
                                .on_hover_text(
                                    "Live data then shows its speed corrected for these tires",
                                )
                                .clicked()
                            {
                                let t = icarus_obd::garage::TireSetting {
                                    stock: self.t_stock.trim().to_string(),
                                    installed: self.t_new.trim().to_string(),
                                    at_transmission: self.t_at_transmission,
                                    stock_ratio: gear,
                                    installed_ratio: installed,
                                };
                                match icarus_obd::garage::set_tires(&vin, Some(t)) {
                                    Ok(()) => {
                                        self.status =
                                            "Tires saved: Live data shows the corrected speed."
                                                .into()
                                    }
                                    Err(e) => self.error = Some(e),
                                }
                                self.refresh_tires();
                            }
                            if self.tires.is_some() && ui.button("Forget").clicked() {
                                let _ = icarus_obd::garage::set_tires(&vin, None);
                                self.refresh_tires();
                            }
                            if let Some((t, f)) = &self.tires {
                                ui.label(
                                    RichText::new(format!(
                                        "saved: {} on stock {}, speed x{f:.3}",
                                        t.installed, t.stock
                                    ))
                                    .weak(),
                                );
                            }
                        });
                    }
                }
                (Err(e), _) | (_, Err(e)) => {
                    ui.colored_label(pal.amber, e);
                }
            }
        });
        self.gps_card(ui, pal);
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

    /// A GPS receiver: listen for a while, show the fix, each satellite's signal and what that says about the receiver
    /// and its antenna, and hand its speed to the speedometer check. Positions are not shown.
    fn gps_card(&mut self, ui: &mut egui::Ui, pal: &theme::Palette) {
        let mut listen = std::mem::take(&mut self.gps_now);
        card(ui, pal, "GPS", |ui| {
            ui.label(RichText::new("A GPS receiver (USB /dev/ttyACM0, Bluetooth bt:MAC, a phone sharing its GPS at tcp:HOST:PORT, or sim): its fix, every satellite's signal, and what that says about the receiver and its antenna. Its speed goes into the speedometer check below. Positions stay on this computer and are not shown here.").weak());
            ui.horizontal(|ui| {
                ui.add(
                    egui::TextEdit::singleline(&mut self.gps_src)
                        .hint_text("/dev/ttyACM0")
                        .desired_width(220.0),
                );
                if ui
                    .add_enabled(
                        !self.gps_busy && !self.gps_src.trim().is_empty(),
                        egui::Button::new("Listen 10 s"),
                    )
                    .clicked()
                {
                    listen = true;
                }
                if self.gps_busy {
                    ui.spinner();
                }
            });
            let Some((fx, secs)) = &self.gps_fix else {
                return;
            };
            ui.add_space(4.0);
            ui.label(RichText::new(fx.summary()).strong());
            let us = icarus_core::moddata::units::us_customary();
            let speed = fx
                .speed_kmh
                .filter(|_| fx.valid)
                .map(|k| if us { k / 1.609_344 } else { k });
            if let Some(v) = speed {
                ui.horizontal(|ui| {
                    ui.label(format!("Speed {v:.1} {}", if us { "mph" } else { "km/h" }));
                    if ui.small_button("Use in the speedometer check").clicked() {
                        self.s_gps = format!("{v:.1}");
                    }
                });
            }
            let (heard, _) = fx.heard();
            ui.label(format!(
                "Satellites in view: {} ({heard} with a signal)",
                fx.sats.len()
            ));
            let mut sats: Vec<_> = fx.sats.values().collect();
            sats.sort_by(|a, b| b.snr.cmp(&a.snr).then(a.prn.cmp(&b.prn)));
            egui::Grid::new("gps-sats")
                .num_columns(3)
                .spacing(vec2(10.0, 2.0))
                .show(ui, |ui| {
                    for s in sats {
                        ui.label(RichText::new(format!("{} {}", s.system, s.prn)).small());
                        let snr = s.snr.unwrap_or(0);
                        let col = match snr {
                            0 => pal.dim,
                            1..=24 => pal.amber,
                            _ => pal.accent,
                        };
                        ui.add(
                            egui::ProgressBar::new(snr as f32 / 50.0)
                                .desired_width(160.0)
                                .fill(col),
                        );
                        ui.label(
                            RichText::new(s.snr.map_or("none".into(), |n| format!("{n} dB-Hz")))
                                .small(),
                        );
                        ui.end_row();
                    }
                });
            for d in fx.diagnose(*secs) {
                ui.label(d);
            }
        });
        if listen {
            self.gps_busy = true;
            let src = self.gps_src.trim().to_string();
            let (tx, c) = (self.bg_tx.clone(), ui.ctx().clone());
            std::thread::spawn(move || {
                let secs = 10.0;
                let r = icarus_obd::gps::Source::open(&src)
                    .and_then(|mut s| icarus_obd::gps::listen(&mut s, secs));
                let _ = tx.send(Bg::Gps(r, secs));
                c.request_repaint();
            });
        }
    }

    fn load_maps(&mut self) {
        let m = &mut self.maps;
        m.note.clear();
        m.selected = None;
        let result = (|| -> Result<(), String> {
            let p = icarus_core::def::load(std::path::Path::new(m.def_path.trim()))
                .map_err(|e| format!("definition: {e}"))?;
            let bin = std::fs::read(m.bin_path.trim()).map_err(|e| format!("image: {e}"))?;
            let cmp_path = std::path::Path::new(m.cmp_path.trim());
            let cmp = if m.cmp_path.trim().is_empty() {
                None
            } else if icarus_core::version::is_version_file(cmp_path) {
                // a version: the image it makes from this one
                Some(icarus_core::version::load(cmp_path)?.apply(&bin)?)
            } else {
                Some(std::fs::read(cmp_path).map_err(|e| format!("second image: {e}"))?)
            };
            let changed = match &cmp {
                Some(c) if c.len() != bin.len() => {
                    return Err("the two images differ in size".into())
                }
                Some(c) => changed_elements(&p.definition, &bin, c),
                None => vec![false; p.definition.elements.len()],
            };
            let n = changed.iter().filter(|c| **c).count();
            m.note = format!(
                "{} elements{}{}",
                p.definition.elements.len(),
                if p.skipped.is_empty() {
                    String::new()
                } else {
                    format!(", {} left out (first: {})", p.skipped.len(), p.skipped[0])
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
            m.versions = icarus_core::version::list(std::path::Path::new(m.bin_path.trim()), &bin);
            m.version_shown = m.versions.iter().position(|(p, _)| p.as_path() == cmp_path);
            m.version_msg.clear();
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
                ui.label("Definition (.xdf, .a2l)");
                ui.add(egui::TextEdit::singleline(&mut self.maps.def_path).desired_width(520.0));
                ui.end_row();
                ui.label("Image (.bin)");
                ui.add(egui::TextEdit::singleline(&mut self.maps.bin_path).desired_width(520.0));
                ui.end_row();
                ui.label("Compare with");
                ui.add(
                    egui::TextEdit::singleline(&mut self.maps.cmp_path)
                        .hint_text(
                            "optional: a modified image or a .version.toml, to see what it changed",
                        )
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
        let mut vact: Option<VersionAct> = None;
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
            look,
            versions,
            version_shown,
            version_name,
            version_note,
            version_msg,
            version_delete,
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
        // versions: stock (the image as read, never changed) and named sets of changes against it, kept beside it
        ui.horizontal_wrapped(|ui| {
            ui.label("Versions:");
            if ui
                .selectable_label(cmp.is_none(), "Stock")
                .on_hover_text("The original image, as it was read: never changed")
                .clicked()
            {
                vact = Some(VersionAct::Stock);
            }
            for (k, (_, v)) in versions.iter().enumerate() {
                let tip = format!(
                    "{} byte(s) changed in {} place(s){}",
                    v.changed_bytes(),
                    v.changes.len(),
                    if v.note.is_empty() {
                        String::new()
                    } else {
                        format!(": {}", v.note)
                    }
                );
                if ui
                    .selectable_label(*version_shown == Some(k), v.name.as_str())
                    .on_hover_text(tip)
                    .clicked()
                {
                    vact = Some(VersionAct::Show(k));
                }
            }
            if let Some(k) = *version_shown {
                if ui
                    .button("Edit from it")
                    .on_hover_text(
                        "Its changes become the edits, to go on from; stock stays as it is",
                    )
                    .clicked()
                {
                    vact = Some(VersionAct::Edit(k));
                }
                let new_name = version_name.trim();
                let label = if new_name.is_empty() {
                    "Rename".to_string()
                } else {
                    format!("Rename to \"{new_name}\"")
                };
                if ui
                    .add_enabled(!new_name.is_empty(), egui::Button::new(label))
                    .on_hover_text("Type the new name in the name box below, then rename; never over another version")
                    .clicked()
                {
                    vact = Some(VersionAct::Rename(k));
                }
                if ui
                    .button("Delete")
                    .on_hover_text("Remove this version's file (asks first); stock and the other versions stay")
                    .clicked()
                {
                    *version_delete = Some(k);
                }
            }
        });
        // deleting a version, once confirmed
        if let Some(k) = *version_delete {
            match versions.get(k).filter(|_| *version_shown == Some(k)) {
                Some((_, v)) => {
                    let name = v.name.clone();
                    ui.horizontal(|ui| {
                        ui.label(format!("Delete version \"{name}\"? Its file goes; stock and the other versions stay."));
                        if ui.button("Delete it").clicked() {
                            vact = Some(VersionAct::Delete(k));
                        }
                        if ui.button("Keep it").clicked() {
                            *version_delete = None;
                        }
                    });
                }
                None => *version_delete = None,
            }
        }
        ui.horizontal(|ui| {
            ui.label("Keep the edits as a version");
            ui.add(egui::TextEdit::singleline(version_name).hint_text("name").desired_width(160.0));
            ui.add(egui::TextEdit::singleline(version_note).hint_text("note (optional)").desired_width(240.0));
            if ui
                .add_enabled(
                    !edits.is_empty() && !version_name.trim().is_empty(),
                    egui::Button::new("Save version"),
                )
                .on_hover_text("A small file of what changed against stock, beside the vehicle's files; never over another file")
                .clicked()
            {
                vact = Some(VersionAct::Save);
            }
        });
        if !version_msg.is_empty() {
            ui.label(RichText::new(version_msg.as_str()).small());
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
                element_view(ui, pal, def, rom, cmp.as_ref(), e, cell, value_text, look);
            });
        });
        if let Some(a) = vact {
            self.maps.version_msg = version_act(&mut self.maps, a).unwrap_or_else(|e| e);
        }
        // apply what was asked for
        let Some(ed) = edit else { return };
        let (Some(i), Some(rom)) = (self.maps.selected, self.maps.rom.as_mut()) else {
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
                if let Some(r) = def.span(&t.z) {
                    rom.revert_range(r);
                }
                Ok(())
            }
            (MapEdit::UndoElement, Kind::Constant(d)) => {
                if let Some(r) = def.span(d) {
                    rom.revert_range(r);
                }
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
                // the formula with the values it uses as they are in this image
                let math = def.math(rom, d).map_err(|x| x.to_string())?;
                // one step of the stored number, in the direction the real value goes
                let up = math.eval(raw + 1.0) >= math.eval(raw);
                let new = raw + if up { n as f64 } else { -n as f64 };
                let real = math.eval(new);
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

/// Does what the Maps page's versions row asked for; returns what to say about it.
fn version_act(m: &mut Maps, a: VersionAct) -> Result<String, String> {
    let Maps {
        def,
        rom,
        cmp,
        cmp_path,
        bin_path,
        changed,
        only_changed,
        selected,
        cell,
        versions,
        version_shown,
        version_name,
        version_note,
        version_delete,
        ..
    } = m;
    let (Some(def), Some(rom)) = (def.as_ref(), rom.as_mut()) else {
        return Ok(String::new());
    };
    let image = std::path::PathBuf::from(bin_path.trim());
    let mut to_stock = |cmp: &mut Option<Rom>, cmp_path: &mut String| {
        *cmp = None;
        cmp_path.clear();
        *version_shown = None;
        *changed = vec![false; def.elements.len()];
        *only_changed = false;
    };
    match a {
        VersionAct::Stock => {
            to_stock(cmp, cmp_path);
            Ok(String::new())
        }
        VersionAct::Show(k) => {
            let (p, v) = versions.get(k).ok_or("that version is no longer there")?;
            let bytes = v.apply(rom.original())?;
            *changed = changed_elements(def, rom.original(), &bytes);
            let n = changed.iter().filter(|c| **c).count();
            *cmp = Some(Rom::new(bytes));
            *cmp_path = p.display().to_string();
            *version_shown = Some(k);
            *only_changed = n > 0;
            if let Some(first) = changed.iter().position(|c| *c) {
                *selected = Some(first);
                *cell = None;
            }
            Ok(format!(
                "Version \"{}\" against stock: {} byte(s) in {} place(s); {n} element(s) changed, outlined in their tables",
                v.name,
                v.changed_bytes(),
                v.changes.len()
            ))
        }
        VersionAct::Edit(k) => {
            if !rom.changes().is_empty() {
                return Err("There are edits not kept yet: keep them (as a version or an image) or Undo all first.".into());
            }
            let (_, v) = versions.get(k).ok_or("that version is no longer there")?;
            rom.load_edits(&v.apply(rom.original())?)
                .map_err(|e| e.to_string())?;
            let name = v.name.clone();
            to_stock(cmp, cmp_path);
            Ok(format!(
                "Editing from version \"{name}\": its changes are the edits now (stock stays as it is); keep them as a new version or save an image"
            ))
        }
        VersionAct::Rename(k) => {
            let (p, v) = versions.get(k).ok_or("that version is no longer there")?;
            let old = v.name.clone();
            let new = icarus_core::version::rename(p, &image, version_name)?;
            *versions = icarus_core::version::list(&image, rom.original());
            // still shown: the same changes under the new name
            *version_shown = versions.iter().position(|(q, _)| *q == new);
            *cmp_path = new.display().to_string();
            version_name.clear();
            Ok(format!(
                "Renamed \"{old}\" to \"{}\"",
                version_shown
                    .and_then(|i| versions.get(i))
                    .map(|(_, v)| v.name.as_str())
                    .unwrap_or("")
            ))
        }
        VersionAct::Delete(k) => {
            *version_delete = None;
            let (p, _) = versions.get(k).ok_or("that version is no longer there")?;
            let v = icarus_core::version::delete(p)?;
            *versions = icarus_core::version::list(&image, rom.original());
            to_stock(cmp, cmp_path);
            Ok(format!(
                "Deleted version \"{}\"; stock and the other versions stay",
                v.name
            ))
        }
        VersionAct::Save => {
            let (p, v) = icarus_core::version::save_edits(
                rom,
                Some(def),
                &image,
                version_name,
                version_note,
            )?;
            *versions = icarus_core::version::list(&image, rom.original());
            *version_shown = versions
                .iter()
                .position(|(p, _)| p.display().to_string() == *cmp_path);
            version_name.clear();
            version_note.clear();
            Ok(format!(
                "Kept version \"{}\": {} byte(s) changed against stock, in {}",
                v.name,
                v.changed_bytes(),
                p.display()
            ))
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
                        if let (Ok(at), Ok(math)) = (def.cell_offset(d, r, c), def.math(rom, d)) {
                            if let (Ok(now), Ok(was)) =
                                (rom.read(at, d.cell), rom.read_original(at, d.cell))
                            {
                                if now != was {
                                    ui.label(
                                        RichText::new(format!(
                                            "was {}",
                                            fmt_num(math.eval(was), d.decimals)
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
    look: &mut Look,
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
            // the cells, or a picture of them: a line per row, or a surface for a map
            let can_3d = t.z.rows > 1 && t.z.cols > 1;
            ui.horizontal(|ui| {
                ui.selectable_value(&mut look.view, View::Cells, "Cells");
                ui.selectable_value(&mut look.view, View::Graph, "2D");
                if can_3d {
                    ui.selectable_value(&mut look.view, View::Surface, "3D");
                }
            });
            let view = match look.view {
                View::Surface if !can_3d => View::Graph,
                other => other,
            };
            if view != View::Cells {
                // the second image's values when comparing, as the cells show them
                let z = w.as_ref().map_or(&v.z, |w| &w.z);
                let hit = if view == View::Surface {
                    surface::draw_3d(
                        ui,
                        z,
                        &mut look.yaw,
                        &mut look.pitch,
                        pal.field,
                        Color32::from_black_alpha(110),
                        heat,
                    )
                } else {
                    surface::draw_2d(ui, z, pal.field, pal.hover, heat)
                };
                match hit {
                    Some(((r, c), clicked)) => {
                        ui.label(format!(
                            "At {} / {}: {} {}",
                            v.y.get(r).map_or("-", String::as_str),
                            v.x.get(c).map_or("-", String::as_str),
                            fmt_num(z[r][c], t.z.decimals),
                            t.z.units
                        ));
                        if clicked {
                            *cell = Some((r, c));
                            *value_text = fmt_num(v.z[r][c], t.z.decimals);
                        }
                    }
                    None => {
                        ui.label(
                            RichText::new(if view == View::Surface {
                                "Drag to turn it. Point at a value to read it; click it to change it above."
                            } else {
                                "The first row's line is blue, the last row's red. Point at a value to read it; click it to change it above."
                            })
                            .weak(),
                        );
                    }
                }
                return;
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
    if let Some(i) = info.as_ref().filter(|i| i.detail.is_some()) {
        egui::CollapsingHeader::new(RichText::new("In depth").small())
            .id_salt(("code-depth", code, extra))
            .show(ui, |ui| {
                for (label, t) in dtc::depth(i) {
                    ui.label(RichText::new(format!("{label}: {t}")).small());
                }
            });
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

/// The Garage page's fields for the vehicle shown, as typed (saved with Save, or Enter in its name).
#[derive(Clone, Debug, Default, PartialEq)]
struct GarageEdit {
    vin: String,
    nickname: String,
    year: String,
    make: String,
    model: String,
    sub_model: String,
    engine_size: String,
    engine: String,
    drivetrain: String,
    transmission: String,
    miles: String,
}

impl GarageEdit {
    fn of(vin: &str, described: Option<&Described>, d: &garage::Details) -> GarageEdit {
        let dd = described.cloned().unwrap_or_default();
        GarageEdit {
            vin: vin.to_string(),
            nickname: d.nickname.clone(),
            year: dd.year.map(|y| y.to_string()).unwrap_or_default(),
            make: dd.make.clone(),
            model: dd.model.clone(),
            sub_model: d.sub_model.clone(),
            engine_size: dd.engine_size(),
            engine: dd.engine(),
            drivetrain: d.drivetrain.clone(),
            transmission: d.transmission.clone(),
            miles: d.miles.map(|m| m.to_string()).unwrap_or_default(),
        }
    }

    /// The description (garage.toml) and the details (details.toml) these fields make, or what is wrong with them.
    fn parts(&self) -> Result<(String, garage::Details), String> {
        let year = match self.year.trim() {
            "" => None,
            y => Some(
                y.parse::<u16>()
                    .ok()
                    .filter(|y| (1950..=2100).contains(y))
                    .ok_or(format!("{y}: a year, like 2017"))?,
            ),
        };
        let miles = match self.miles.trim().replace([',', ' '], "") {
            m if m.is_empty() => None,
            m => Some(
                m.parse::<u32>()
                    .map_err(|_| format!("{}: the miles, a whole number", self.miles.trim()))?,
            ),
        };
        let name = Described::from_parts(
            year,
            &self.make,
            &self.model,
            &self.engine_size,
            &self.engine,
        )
        .name();
        let details = garage::Details {
            nickname: self.nickname.clone(),
            sub_model: self.sub_model.clone(),
            drivetrain: self.drivetrain.clone(),
            transmission: self.transmission.clone(),
            miles,
        };
        Ok((name, details))
    }
}

/// An on/off switch, drawn as Settings draws its switches (the accent colour when on).
fn switch(ui: &mut egui::Ui, pal: &theme::Palette, on: &mut bool) -> egui::Response {
    let (rect, mut resp) = ui.allocate_exact_size(vec2(44.0, 24.0), Sense::click());
    if resp.clicked() {
        *on = !*on;
        resp.mark_changed();
    }
    let t = ui.ctx().animate_bool(resp.id, *on);
    let track = if *on { pal.accent } else { pal.hover };
    ui.painter()
        .rect_filled(rect, CornerRadius::same(12), track);
    let x = egui::lerp(rect.left() + 12.0..=rect.right() - 12.0, t);
    ui.painter().circle_filled(
        egui::pos2(x, rect.center().y),
        9.0,
        if *on { pal.on_accent() } else { Color32::WHITE },
    );
    resp
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

/// A CAN ID as written: three digits for 11-bit, eight for 29-bit.
fn id_text(id: u32) -> String {
    if id > 0x7FF {
        format!("{id:08X}")
    } else {
        format!("{id:03X}")
    }
}

/// A module's title on the Modules page (ICA-76): its name, and in Advanced its request and answer IDs.
fn module_title(a: &icarus_obd::vehicles::ModuleAddr, advanced: bool) -> String {
    if advanced {
        format!("{}/{}  {}", id_text(a.tx), id_text(a.rx()), a.name)
    } else {
        a.name.clone()
    }
}

/// The line under a trouble code (ICA-76): its state in words, and in Advanced the failure-type byte and the protocol.
fn code_extra(states: &[&str], failure_type: Option<u8>, lang: &str, advanced: bool) -> String {
    let words = states.join(", ");
    if !advanced {
        return words;
    }
    let ftb = failure_type
        .map(|f| format!(" -{f:02X}"))
        .unwrap_or_default();
    format!("{words}{ftb}, {lang}")
}

/// The Modules page's line for a module with no code to show (ICA-207): "No trouble codes (93 it can set, none failed)
/// (UDS)" for one that lists every code it can set with none failed, "No trouble codes (UDS)" for one that lists none.
fn clean_codes_label(read: &icarus_obd::uds::ModuleCodes, lang: &str) -> Option<String> {
    let clean = read.none_failed()?;
    let mut c = clean.chars();
    let first = c.next().map(|f| f.to_uppercase().collect::<String>());
    Some(format!(
        "{}{} ({lang})",
        first.unwrap_or_default(),
        c.as_str()
    ))
}

/// A live value's name (ICA-76): in Advanced with its PID number first.
fn pid_label(pid: u8, name: &str, advanced: bool) -> String {
    if advanced {
        format!("{pid:02X}  {name}")
    } else {
        name.to_string()
    }
}

/// One line of the guided scan: how urgent, the code, what it means, why that urgency, what to check first, the modules.
type GuidedRow = (
    dtc::Urgency,
    String,
    String,
    &'static str,
    Option<String>,
    String,
);

/// The guided scan's lines from (code, module) pairs: each code once with every module that has it, in plain words, most
/// urgent first (then by code).
fn guided_rows(found: &[(String, String)], profile: Option<&Profile>) -> Vec<GuidedRow> {
    let mut codes: Vec<(String, Vec<String>)> = Vec::new();
    for (c, m) in found {
        match codes.iter_mut().find(|(x, _)| x == c) {
            Some((_, ms)) if !ms.contains(m) => ms.push(m.clone()),
            Some(_) => {}
            None => codes.push((c.clone(), vec![m.clone()])),
        }
    }
    let mut rows: Vec<GuidedRow> = codes
        .iter()
        .map(|(c, ms)| {
            let info = dtc::explain(c, profile);
            let (u, why) = info
                .as_ref()
                .map(dtc::urgency)
                .unwrap_or((dtc::Urgency::Soon, "Check it soon."));
            let text = info
                .as_ref()
                .ok()
                .and_then(|i| i.text.clone())
                .unwrap_or_else(|| {
                    "a maker's own code (its meaning is in the maker's service information)".into()
                });
            let first = info.as_ref().ok().and_then(dtc::first_check);
            (u, c.clone(), text, why, first, ms.join(", "))
        })
        .collect();
    rows.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.cmp(&b.1)));
    rows
}

/// Basic's guided scan (ICA-75).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Guided {
    Off,
    Connecting,
    Scanning,
    Done,
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        theme::apply_accessibility(ui.ctx());
        let ctx = ui.ctx().clone();
        chrome::resize_edges(&ctx);
        let accent = theme::system_accent();
        let dark = match self.levels.theme {
            levels::Theme::System => theme::system_dark(),
            levels::Theme::Dark => true,
            levels::Theme::Light => false,
        };
        logs::set_units(self.levels.units);
        theme::apply(&ctx, dark, accent);
        let pal = theme::palette_for(dark, accent);
        self.events();
        self.take_handed(&ctx);
        self.keep_trying(&ctx);
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
                    if !self.levels.allows(need(p)) {
                        continue;
                    }
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
                    // the window's own settings: the levels (and later the units, the look and the devices)
                    let gear = ui
                        .selectable_label(
                            self.page == Page::App,
                            RichText::new("\u{2699} Settings").size(14.0),
                        )
                        .on_hover_text("Icarus's own settings: Basic, Advanced and Tuning");
                    if gear.clicked() {
                        self.page = Page::App;
                    }
                    ui.add_space(6.0);
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
                for n in &self.dropped {
                    ui.colored_label(pal.amber, n);
                }
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
                                Page::Logs => self.logs.page(ui, &pal),
                                Page::App => self.page_app(ui, &pal),
                                Page::Gauges => self.page_gauges(ui, &pal),
                                _ => self.page_live(ui, &pal),
                            });
                    }
                }
            });
        if self.welcome {
            let mut done = false;
            egui::Modal::new(egui::Id::new("welcome")).show(&ctx, |ui| {
                ui.set_max_width(480.0);
                ui.label(RichText::new("Welcome to Icarus").size(19.0).strong());
                ui.add_space(6.0);
                ui.label("Icarus reads what your vehicle's computers know: trouble codes, live values, and how each module is doing. Three steps:");
                ui.add_space(6.0);
                for (n, step) in [
                    "Plug the adapter into the OBD port, under the dashboard on the driver's side.",
                    "Switch the ignition on (the engine can stay off).",
                    "Press Scan my vehicle on the Vehicle page: every code comes back in plain words, the most urgent first, with what to check.",
                ]
                .iter()
                .enumerate()
                {
                    ui.horizontal_wrapped(|ui| {
                        ui.label(RichText::new(format!("{}.", n + 1)).strong());
                        ui.label(*step);
                    });
                }
                ui.add_space(6.0);
                ui.label(RichText::new("Nothing is changed in the vehicle by reading. The gear (Settings) has more: units, the theme, and Advanced for more detail and control.").weak());
                ui.add_space(10.0);
                if ui.button(RichText::new("Start").strong()).clicked() {
                    done = true;
                }
            });
            if done {
                self.welcome = false;
                self.page = Page::Vehicle;
                self.levels.welcomed = true;
                self.save_levels();
            }
        }
        if self.tuning_ask {
            let mut answer = None;
            egui::Modal::new(egui::Id::new("confirm-tuning")).show(&ctx, |ui| {
                ui.set_max_width(460.0);
                ui.label(RichText::new("Turn on tuning?").size(17.0).strong());
                ui.add_space(6.0);
                ui.label("Tuning changes the software in a vehicle's computers. A wrong change can stop a vehicle from starting. Icarus keeps the original first and checks the conditions before it writes anything, but the decision to write is yours.");
                ui.add_space(10.0);
                ui.horizontal(|ui| {
                    if ui.button("Turn on tuning").clicked() {
                        answer = Some(true);
                    }
                    if ui.button("Cancel").clicked() {
                        answer = Some(false);
                    }
                });
            });
            if let Some(yes) = answer {
                self.tuning_ask = false;
                if yes {
                    self.levels.tuning = true;
                    self.save_levels();
                }
            }
        }
        if let Some(confirm::Asked { what: def, on }) = self.confirms.service.clone() {
            let mut go = false;
            let mut cancel = false;
            let resp = egui::Modal::new(egui::Id::new("confirm-service")).show(&ctx, |ui| {
                ui.set_max_width(460.0);
                ui.label(
                    RichText::new(format!("Run {}?", def.name))
                        .size(17.0)
                        .strong(),
                );
                ui.add_space(6.0);
                ui.label(format!(
                    "Routine {:04X} in module {:03X}{}.",
                    def.routine,
                    def.module,
                    if def.checked {
                        ""
                    } else {
                        ", not yet checked on a vehicle"
                    }
                ));
                if !def.warning.is_empty() {
                    ui.label(&def.warning);
                }
                if !def.conditions.is_empty() {
                    ui.label(
                        RichText::new(format!(
                            "Needs: {}.",
                            def.conditions
                                .iter()
                                .map(|c| c.describe())
                                .collect::<Vec<_>>()
                                .join(", ")
                        ))
                        .weak(),
                    );
                }
                ui.add_space(10.0);
                ui.horizontal(|ui| {
                    if ui
                        .add(
                            egui::Button::new(RichText::new("Run").color(pal.on_accent()))
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
                let _ = self.tx.send(Cmd::Service { def, on });
                self.confirms.service = None;
            } else if cancel || resp.should_close() {
                self.confirms.service = None;
            }
        }
        if let Some(confirm::Asked {
            what: (target, label),
            on,
        }) = self.confirms.clear.clone()
        {
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
                self.clear_yes(target, on);
            } else if cancel || resp.should_close() {
                self.confirms.clear = None;
            }
        }
        if let Some(confirm::Asked {
            what: (def, value),
            on,
        }) = self.confirms.setting.clone()
        {
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
                    let _ = self.tx.send(Cmd::SettingChange {
                        def,
                        value: bytes,
                        on,
                    });
                    self.status = "Changing the setting...".into();
                }
                self.confirms.setting = None;
            } else if cancel || resp.should_close() {
                self.confirms.setting = None;
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
    let args: Vec<String> = std::env::args().skip(1).collect();
    // the session floor (ICA-192) held from the whole line, then the line read, before anything is handed over
    let begun = match begin(
        args,
        icarus_obd::lock::env_asks_read_only(),
        &icarus_obd::lock::hold_session_read_only,
    ) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("icarus-app: {e}\nusage: icarus-app [--read-only] [--connect] [--adapter A] [--page vehicle|modules|live|logs|tests|settings|learning|bus|maps|garage|tools] [--scan] [--live] [--record] [--learn [--mark TEXT] [--learn-seconds N]] [--maps DEF.xdf|.a2l IMAGE.bin [SECOND.bin|VERSION.version.toml]] [--map NAME [--cell R,C]] [--view cells|2d|3d]");
            std::process::exit(2);
        }
    };
    // one Icarus at a time: an open one takes these arguments and comes to the front
    match single::hand_over(&begun.hand, begun.read_only) {
        single::Handed::NoWindow => {}
        single::Handed::Taken => return Ok(()),
        single::Handed::NotReadOnly => {
            eprintln!(
                "icarus-app: the window already open took this start but did not say it is read-only now (an Icarus from \
                 before --read-only, maybe), so it may still change a vehicle: close it and start again with --read-only"
            );
            std::process::exit(1);
        }
    }
    let start = begun.start;
    let mut viewport = egui::ViewportBuilder::default()
        .with_inner_size([1180.0, 760.0])
        .with_min_inner_size([760.0, 480.0])
        .with_title("Icarus")
        .with_app_id("icarus")
        .with_decorations(false);
    // the window's own icon (where the desktop takes it from the window rather than from icarus.desktop)
    if let Ok(icon) = eframe::icon_data::from_png_bytes(include_bytes!(
        "../../../packaging/icons/256x256/icarus.png"
    )) {
        viewport = viewport.with_icon(icon);
    }
    let options = eframe::NativeOptions {
        viewport,
        ..Default::default()
    };
    let r = eframe::run_native(
        "Icarus",
        options,
        Box::new(|cc| Ok(Box::new(App::new(&cc.egui_ctx, start)))),
    );
    wind_down(std::time::Duration::from_secs(5));
    r
}

/// The worker of the window that is open, kept so the window's end waits for it (`wind_down`).
type WorkerSlot = Option<(std::sync::mpsc::Sender<Cmd>, std::thread::JoinHandle<()>)>;
static WORKER: std::sync::Mutex<WorkerSlot> = std::sync::Mutex::new(None);

/// The window has closed: the worker is told to end (`Cmd::Quit`: a running test handed back, a recording stopped, its
/// file flushed and synced, the stopped notice given, the computer let sleep again) and waited for, up to `most` (a sweep
/// in progress ends first).
fn wind_down(most: std::time::Duration) {
    let Some((tx, handle)) = WORKER.lock().ok().and_then(|mut w| w.take()) else {
        return;
    };
    let _ = tx.send(Cmd::Quit);
    drop(tx);
    let end = std::time::Instant::now() + most;
    while !handle.is_finished() && std::time::Instant::now() < end {
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
    if handle.is_finished() {
        let _ = handle.join();
    }
}

/// What the Live data page shows of a recording the worker writes.
pub(crate) struct Recording {
    pub(crate) path: std::path::PathBuf,
    pub(crate) values: usize,
    pub(crate) rows: usize,
    /// What it said of its module values: the watched ones, then its rows a second (ICA-218).
    pub(crate) said: Vec<String>,
}

/// The Settings field for the watched module values: a name a line, `none` for nothing watched.
fn watch_field(names: &[String]) -> String {
    if names.is_empty() {
        "none".into()
    } else {
        names.join("\n")
    }
}

/// A home of a test's own for the tests that run the worker (it reads the config folder and writes the vehicle's record,
/// code history and folder): HOME and the XDG folders point into a temp folder, one such test at a time, and the folder
/// is removed after. The variables are not put back: a worker still running after a failed test can then only write into
/// the removed folder, never the real home.
#[cfg(test)]
pub(crate) struct TestHome {
    pub(crate) dir: std::path::PathBuf,
    _one_at_a_time: std::sync::MutexGuard<'static, ()>,
}

#[cfg(test)]
impl TestHome {
    pub(crate) fn new(tag: &str) -> TestHome {
        static ONE: std::sync::Mutex<()> = std::sync::Mutex::new(());
        let one = ONE.lock().unwrap_or_else(|e| e.into_inner());
        let dir = std::env::temp_dir().join(format!(
            "icarus-app-{tag}-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        std::env::set_var("HOME", &dir);
        std::env::set_var("XDG_CONFIG_HOME", dir.join("config"));
        std::env::set_var("XDG_STATE_HOME", dir.join("state"));
        std::env::set_var("XDG_DATA_HOME", dir.join("data"));
        // a session floor asked for in the shell that runs the tests is not this test's
        std::env::remove_var(icarus_obd::lock::SESSION_ENV);
        TestHome {
            dir,
            _one_at_a_time: one,
        }
    }
}

#[cfg(test)]
impl Drop for TestHome {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

#[cfg(test)]
mod advanced_detail {
    use super::*;

    /// ICA-76: Basic speaks in words; Advanced adds the IDs, the fault byte and the protocol, and the PID numbers.
    #[test]
    fn advanced_adds_the_raw_detail_and_basic_keeps_to_words() {
        let a = icarus_obd::vehicles::ModuleAddr {
            tx: 0x7E0,
            rx: None,
            name: "engine (ECM)".into(),
            ext: None,
            bus: None,
            kind: String::new(),
            checked: true,
        };
        assert_eq!(module_title(&a, false), "engine (ECM)");
        assert_eq!(module_title(&a, true), "7E0/7E8  engine (ECM)");
        let w = ["confirmed", "warning light requested"];
        assert_eq!(
            code_extra(&w, Some(0x1C), "UDS", false),
            "confirmed, warning light requested"
        );
        assert_eq!(
            code_extra(&w, Some(0x1C), "UDS", true),
            "confirmed, warning light requested -1C, UDS"
        );
        assert_eq!(pid_label(0x0C, "Engine speed", false), "Engine speed");
        assert_eq!(pid_label(0x0C, "Engine speed", true), "0C  Engine speed");
        assert_eq!(id_text(0x18DA10F1), "18DA10F1");
    }
}

#[cfg(test)]
mod guided {
    #[test]
    fn the_guided_scan_lists_each_code_once_most_urgent_first() {
        let found: Vec<(String, String)> = [
            ("P0456", "engine"),
            ("U0100", "body"),
            ("P0301", "engine"),
            ("U0100", "meter"),
            ("U0100", "body"),
        ]
        .iter()
        .map(|(c, m)| (c.to_string(), m.to_string()))
        .collect();
        let rows = super::guided_rows(&found, None);
        let order: Vec<(&str, &str)> = rows.iter().map(|r| (r.1.as_str(), r.5.as_str())).collect();
        assert_eq!(
            order,
            vec![
                ("P0301", "engine"),
                ("U0100", "body, meter"),
                ("P0456", "engine")
            ],
            "serious, soon, can wait; once each"
        );
        assert!(
            rows[0].4.as_deref().unwrap_or("").contains("coil"),
            "what to check first"
        );
        assert!(super::guided_rows(&[], None).is_empty());
    }
}

#[cfg(test)]
mod level_pages {
    use super::*;

    fn shown(l: levels::Levels) -> Vec<&'static str> {
        PAGES
            .iter()
            .filter(|(p, _)| l.allows(need(*p)))
            .map(|(_, n)| *n)
            .collect()
    }

    #[test]
    fn each_level_shows_its_pages() {
        let basic = levels::Levels::default();
        assert_eq!(
            shown(basic),
            [
                "Vehicle",
                "Modules",
                "Live data",
                "Trail gauges",
                "Logs",
                "Garage",
                "Tools"
            ]
        );
        let adv = levels::Levels {
            advanced: true,
            tuning: false,
            ..Default::default()
        };
        assert_eq!(
            shown(adv),
            [
                "Vehicle",
                "Modules",
                "Live data",
                "Trail gauges",
                "Logs",
                "Tests",
                "Module settings",
                "Learning",
                "Bus",
                "Garage",
                "Tools"
            ]
        );
        let all = levels::Levels {
            advanced: true,
            tuning: true,
            ..Default::default()
        };
        assert_eq!(
            shown(all).len(),
            PAGES.len(),
            "Tuning adds Maps: every page"
        );
        assert!(shown(all).contains(&"Maps") && !shown(adv).contains(&"Maps"));
        // the gear's page is there at every level
        assert!(basic.allows(need(Page::App)));
    }
}

#[cfg(test)]
mod versions_row {
    use super::*;

    const DEF: &str = r#"<XDFFORMAT version="1.70">
  <XDFHEADER><deftitle>Test</deftitle></XDFHEADER>
  <XDFTABLE uniqueid="0x1">
    <title>Spark</title>
    <XDFAXIS id="x" uniqueid="0x0"><indexcount>2</indexcount></XDFAXIS>
    <XDFAXIS id="y" uniqueid="0x0"><indexcount>2</indexcount></XDFAXIS>
    <XDFAXIS id="z">
      <EMBEDDEDDATA mmedaddress="0x10" mmedelementsizebits="8" mmedrowcount="2" mmedcolcount="2" />
      <units>deg</units>
      <decimalpl>1</decimalpl>
      <MATH equation="X/2"><VAR id="X" /></MATH>
    </XDFAXIS>
  </XDFTABLE>
</XDFFORMAT>"#;

    /// ICA-58: on the Maps page a version is kept, shown, renamed (still shown, under the new name) and deleted (back to
    /// stock); a rename with no name typed is refused and changes nothing; stock is never touched.
    #[test]
    fn a_version_is_renamed_and_deleted_from_the_maps_page() {
        let dir = std::env::temp_dir().join(format!("icarus-app-versions-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let image = dir.join("stock.bin");
        let mut stock = vec![0u8; 0x20];
        stock[0x10..0x14].copy_from_slice(&[20, 30, 40, 50]);
        std::fs::write(&image, &stock).unwrap();
        let def = icarus_core::def::parse_any(DEF.as_bytes())
            .unwrap()
            .definition;
        let mut rom = Rom::new(stock.clone());
        let icarus_core::def::Kind::Table(t) = &def.element("Spark").unwrap().kind else {
            panic!("a table")
        };
        def.set(&mut rom, &t.z, 0, 1, 21.0).unwrap();
        let mut m = Maps::empty();
        m.bin_path = image.display().to_string();
        m.changed = vec![false; def.elements.len()];
        m.def = Some(def);
        m.rom = Some(rom);
        m.version_name = "Tow".into();
        assert!(version_act(&mut m, VersionAct::Save)
            .unwrap()
            .contains("Kept version \"Tow\""));
        version_act(&mut m, VersionAct::Show(0)).unwrap();
        assert_eq!(m.version_shown, Some(0));
        m.version_name = "Towing".into();
        let said = version_act(&mut m, VersionAct::Rename(0)).unwrap();
        assert_eq!(said, "Renamed \"Tow\" to \"Towing\"");
        assert_eq!(
            (m.version_shown, m.versions[0].1.name.as_str()),
            (Some(0), "Towing")
        );
        assert_eq!(
            m.cmp_path,
            m.versions[0].0.display().to_string(),
            "the renamed file is the one shown"
        );
        assert!(m.version_name.is_empty());
        assert!(
            version_act(&mut m, VersionAct::Rename(0)).is_err(),
            "no name typed"
        );
        assert_eq!(m.versions[0].1.name, "Towing");
        m.version_delete = Some(0);
        let said = version_act(&mut m, VersionAct::Delete(0)).unwrap();
        assert!(said.starts_with("Deleted version \"Towing\""), "{said}");
        assert!(
            m.versions.is_empty()
                && m.version_shown.is_none()
                && m.version_delete.is_none()
                && m.cmp.is_none()
        );
        assert_eq!(std::fs::read(&image).unwrap(), stock, "stock untouched");
        let _ = std::fs::remove_dir_all(&dir);
    }
}

#[cfg(test)]
mod bus_dbc {
    use super::*;
    use std::time::{Duration, Instant};

    /// ICA-88: the Bus page's signals column on the simulated bus: the worker watches it as the page does, and the DBC
    /// names engine speed in 2C4's last bytes; an ID the DBC does not name has an empty cell.
    #[test]
    fn the_bus_page_names_the_signals() {
        // the simulated vehicle's record and the config folder: a home of the test's own
        let home = TestHome::new("busdbc");
        let dbc = icarus_core::dbc::parse(
            "BO_ 708 Engine: 8 ECM\n SG_ EngineSpeed : 7|16@0+ (1,0) [0|8000] \"rpm\" Vector__XXX\n",
        );
        let (ctx, crx) = std::sync::mpsc::channel();
        let (etx, erx) = std::sync::mpsc::channel();
        // an empty fence: the test reads no profile or module-data set from the home folder for its bus watch
        let worker = std::thread::spawn(move || {
            worker::run(crx, etx, icarus_obd::fence::Fence::default, || {})
        });
        ctx.send(Cmd::Connect("sim".into())).unwrap();
        let end = Instant::now() + Duration::from_secs(30);
        let mut connected = false;
        let mut cell = None;
        let mut other = None;
        while Instant::now() < end && cell.is_none() {
            let Ok(e) = erx.recv_timeout(Duration::from_millis(200)) else {
                continue;
            };
            match e {
                Event::Connected(_) if !connected => {
                    connected = true;
                    ctx.send(Cmd::BusStart(None, false)).unwrap();
                }
                Event::Bus(v) => {
                    for (id, _, _, data, _) in &v.rows {
                        if *id == 0x2C4 {
                            cell = Some(dbc_cell(&dbc, *id, data));
                        } else {
                            other = Some(dbc_cell(&dbc, *id, data));
                        }
                    }
                }
                _ => {}
            }
        }
        let _ = ctx.send(Cmd::BusStop);
        // the worker ends once the watch has stopped and the window's side is gone
        drop(ctx);
        worker.join().unwrap();
        drop(home);
        let cell = cell.expect("2C4 seen on the bus page");
        let rpm: f64 = cell
            .strip_prefix("Engine: EngineSpeed ")
            .and_then(|r| r.strip_suffix(" rpm"))
            .and_then(|n| n.parse().ok())
            .unwrap_or_else(|| panic!("{cell}"));
        assert!((750.0..=830.0).contains(&rpm), "{cell}");
        assert_eq!(other.as_deref(), Some(""), "an ID the DBC does not name");
    }
}

#[cfg(test)]
mod read_only_start {
    use super::*;
    use std::cell::Cell;

    fn line(a: &[&str]) -> Vec<String> {
        a.iter().map(|s| s.to_string()).collect()
    }

    /// ICA-192: the floor is asked for by `--read-only` anywhere on the line (beside an option the window does not know,
    /// and where `--mark` took it as its text) or by ICARUS_READ_ONLY, and held at once; a line without either holds
    /// nothing.
    #[test]
    fn the_floor_is_taken_from_the_whole_line() {
        let held = Cell::new(0);
        let hold = || held.set(held.get() + 1);
        for (a, env) in [
            (&["--read-only"][..], false),
            (&["--page", "live"], true),
            (&["--read-only", "--page", "modules", "--sacn"], false),
            (&["--learn", "--mark"], true),
            (&["--learn", "--mark", "--read-only"], false),
        ] {
            held.set(0);
            assert!(take_floor(&line(a), env, &hold), "{a:?}");
            assert_eq!(held.get(), 1, "{a:?}: held");
        }
        held.set(0);
        assert!(!take_floor(
            &line(&["--page", "live", "--mark", "read-only"]),
            false,
            &hold
        ));
        assert_eq!(held.get(), 0);
    }

    /// ICA-192 at start: `icarus-app --read-only` and ICARUS_READ_ONLY=1 each hold the floor before the window opens, and
    /// the line a window already open is handed carries `--read-only` once; a line that does not parse is this start's
    /// error (it exits 2), not handed over; without either, nothing is held and the line goes as given.
    #[test]
    fn a_start_holds_the_floor_and_hands_it_on() {
        let held = Cell::new(0);
        let hold = || held.set(held.get() + 1);
        let b = begin(line(&["--read-only", "--connect"]), false, &hold).unwrap();
        assert!(b.read_only && b.start.connect);
        assert_eq!(held.get(), 1);
        assert_eq!(b.hand, line(&["--read-only", "--connect"]));
        let b = begin(line(&["--page", "live"]), true, &hold).unwrap();
        assert!(b.read_only);
        assert_eq!(held.get(), 2, "ICARUS_READ_ONLY held it");
        assert_eq!(b.hand, line(&["--page", "live", "--read-only"]));
        let b = begin(line(&["--read-only"]), true, &hold).unwrap();
        assert_eq!(b.hand, line(&["--read-only"]), "said once");
        let e = begin(
            line(&["--read-only", "--page", "modules", "--sacn"]),
            false,
            &hold,
        )
        .err()
        .expect("a line that does not parse");
        assert_eq!(e, "unknown option --sacn");
        held.set(0);
        let b = begin(line(&["--page", "live"]), false, &hold).unwrap();
        assert!(!b.read_only);
        assert_eq!(held.get(), 0);
        assert_eq!(b.hand, line(&["--page", "live"]));
    }
}

/// The window itself (`App::events`) on a worker of the test's own, as the window runs it: what reaches its dialogs and
/// what it says.
#[cfg(test)]
mod window_writes {
    use super::*;
    use std::time::{Duration, Instant};

    /// The window on a worker of the test's own: the simulated vehicles (`worker::tests::two_vehicles`), `floor` as the
    /// session floor, and the adapter `other` (the simulated vehicle giving another VIN) chosen as a real adapter is, so
    /// the window connects by itself and `auto` is on.
    fn window(floor: fn() -> Result<(), String>) -> (App, std::thread::JoinHandle<()>) {
        let (tx, wrx) = channel();
        let (wtx, rx) = channel();
        let worker = std::thread::spawn(move || {
            worker::run_with(
                wrx,
                wtx,
                icarus_obd::fence::Fence::default,
                || {},
                &worker::tests::two_vehicles,
                &floor,
            )
        });
        let (_later, handed) = channel();
        let app = App::wired(
            &egui::Context::default(),
            Start::default(),
            Wires {
                tx,
                rx,
                handed,
                chosen: ("other".into(), true),
            },
        );
        (app, worker)
    }

    /// The worker's events taken as the window takes them each frame, until `ok`; panics after 30 s.
    fn until(app: &mut App, what: &str, ok: impl Fn(&App) -> bool) {
        let end = Instant::now() + Duration::from_secs(30);
        loop {
            app.events();
            if ok(app) {
                return;
            }
            assert!(Instant::now() < end, "no {what}");
            std::thread::sleep(Duration::from_millis(20));
        }
    }

    fn number(app: &App) -> u64 {
        app.info.as_ref().map_or(0, |i| i.connection.number)
    }

    /// The clear, routine and setting dialogs, opened on the connection open now as their buttons open them.
    fn open_all(app: &mut App) {
        app.confirms.clear =
            app.ask((None, "every engine and transmission module (OBD-II)".into()));
        app.confirms.service = app.ask(icarus_obd::sim::services().remove(0));
        app.confirms.setting = app.ask((icarus_obd::sim::settings().remove(0), Some("off".into())));
        assert!(!none_open(app), "the dialogs opened");
    }

    fn none_open(app: &App) -> bool {
        app.confirms.clear.is_none()
            && app.confirms.service.is_none()
            && app.confirms.setting.is_none()
    }

    /// ICA-105 in the window: its clear, routine and setting dialogs, open on one connection, are gone once the worker
    /// says the adapter disconnected, once it connected again (the window's events, as each frame takes them), and once a
    /// connection did not come up.
    #[test]
    fn the_windows_dialogs_close_when_their_connection_ends() {
        let home = TestHome::new("app-ica105");
        let (mut app, worker) = window(icarus_obd::lock::session_allows_changes);
        until(&mut app, "the connection", |a| a.link == Link::On);
        assert_eq!(number(&app), 1);
        open_all(&mut app);
        let _ = app.tx.send(Cmd::Disconnect);
        until(&mut app, "the disconnect", |a| a.link == Link::Off);
        assert!(none_open(&app), "gone after Disconnected");
        let _ = app.tx.send(Cmd::Connect("other".into()));
        until(&mut app, "the second connection", |a| number(a) == 2);
        open_all(&mut app);
        // the adapter connects again, to the simulated vehicle's own VIN
        let _ = app.tx.send(Cmd::Connect("sim".into()));
        until(&mut app, "the third connection", |a| number(a) == 3);
        assert!(none_open(&app), "gone after Connected");
        assert_eq!(
            app.info.as_ref().and_then(|i| i.vin.as_deref()),
            Some(icarus_obd::sim::VIN)
        );
        open_all(&mut app);
        let _ = app.tx.send(Cmd::Connect("nowhere".into()));
        until(&mut app, "the failure", |a| {
            a.status.starts_with("Waiting for the adapter")
        });
        assert!(none_open(&app), "gone after Failed");
        drop(app);
        worker.join().unwrap();
        drop(home);
    }

    /// A session floor of the test's own that refuses, as `lock::session_allows_changes` does in a read-only run (the floor
    /// for the whole run would be every test's in this process).
    fn read_only_floor() -> Result<(), String> {
        Err("session read-only: this test's own floor".into())
    }

    /// ICA-192 in the window: on a read-only session with a real adapter (`auto` on), "Save a report and clear" is refused
    /// with nothing sent, and the window says why ("session read-only"), not that the adapter is asleep; the link stays up.
    #[test]
    fn a_refused_clear_is_said_as_it_is() {
        let home = TestHome::new("app-ica192");
        let (mut app, worker) = window(read_only_floor);
        until(&mut app, "the connection", |a| a.link == Link::On);
        until(&mut app, "the connection's requests", |a| {
            !a.sent_log.is_empty()
        });
        assert!(app.auto, "connected as to a real adapter");
        app.sent_log.clear();
        app.confirms.clear =
            app.ask((None, "every engine and transmission module (OBD-II)".into()));
        let Some(confirm::Asked {
            what: (target, _),
            on,
        }) = app.confirms.clear.clone()
        else {
            panic!("the clear's dialog did not open")
        };
        app.clear_yes(target, on);
        assert!(app.confirms.clear.is_none(), "the yes closed it");
        until(&mut app, "the refusal", |a| a.error.is_some());
        let e = app.error.clone().unwrap_or_default();
        assert!(
            e.starts_with("not cleared") && e.contains("session read-only"),
            "{e}"
        );
        assert_eq!(app.status, "The clear did not run to the end.");
        assert!(app.link == Link::On, "the link is still up");
        let _ = app.tx.send(Cmd::Disconnect);
        until(&mut app, "the disconnect", |a| a.link == Link::Off);
        assert!(app.sent_log.is_empty(), "nothing went onto the bus");
        drop(app);
        worker.join().unwrap();
        drop(home);
    }
}
