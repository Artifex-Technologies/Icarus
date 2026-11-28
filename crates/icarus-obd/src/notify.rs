//! Telling the driver what a recording is doing without a look at the screen (ICA-209): a short tone of its own and a desktop
//! notification when a recording starts, stalls (no row for `STALL_AFTER` seconds while the engine was last seen running),
//! goes on again, and stops (with its row count). The tone goes through the desktop's sound, whichever player is there
//! (`pw-play`, `paplay` with the freedesktop sound theme's files, else `canberra-gtk-play` by the sound's name), and the
//! notification through `notify-send`; a missing program or a failing one is passed over quietly. Silenced, nothing is
//! played or shown: the window's setting, `icarus record --quiet`, or `ICARUS_QUIET=1`.
//!
//! The programs are reached through a `Desktop`, so the tests stand in for them and never play a sound or show anything.

use std::path::{Path, PathBuf};

/// Seconds without a row before a recording counts as stalled.
pub const STALL_AFTER: f64 = 10.0;

/// The environment variable that silences `icarus record` (any value but empty, 0, no, false or off).
pub const QUIET_ENV: &str = "ICARUS_QUIET";

/// What a recording did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Happened {
    Started,
    Stalled,
    Resumed,
    Stopped,
}

impl Happened {
    /// The freedesktop sound theme's name for its tone: each its own, so it is told by ear.
    pub fn sound(self) -> &'static str {
        match self {
            Happened::Started => "service-login",
            Happened::Stalled => "dialog-warning",
            Happened::Resumed => "network-connectivity-established",
            Happened::Stopped => "complete",
        }
    }
}

/// The desktop's programs, as the notifier reaches them.
pub trait Desktop: Send {
    /// Whether `program` can be run (it is on the PATH).
    fn has(&self, program: &str) -> bool;
    /// Whether a file is there.
    fn exists(&self, path: &Path) -> bool;
    /// Runs `program` with `args` in the background; nothing is waited for and a failure is not said.
    fn run(&mut self, program: &str, args: &[String]);
}

/// This computer's desktop.
pub struct System;

impl Desktop for System {
    fn has(&self, program: &str) -> bool {
        std::env::var_os("PATH").is_some_and(|p| {
            std::env::split_paths(&p).any(|d| {
                let f = d.join(program);
                std::fs::metadata(&f).is_ok_and(|m| {
                    use std::os::unix::fs::PermissionsExt;
                    m.is_file() && m.permissions().mode() & 0o111 != 0
                })
            })
        })
    }

    fn exists(&self, path: &Path) -> bool {
        path.is_file()
    }

    fn run(&mut self, program: &str, args: &[String]) {
        let child = std::process::Command::new(program)
            .args(args)
            .stdin(std::process::Stdio::null())
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null())
            .spawn();
        // waited for on a thread of its own, so it leaves nothing behind and never holds the recording up
        if let Ok(mut c) = child {
            std::thread::spawn(move || {
                let _ = c.wait();
            });
        }
    }
}

/// Where the freedesktop sound theme keeps its tones.
const SOUNDS: &str = "/usr/share/sounds/freedesktop/stereo";

/// Whether `ICARUS_QUIET` asks for silence.
pub fn quiet_from_env() -> bool {
    quiet_value(std::env::var(QUIET_ENV).ok().as_deref())
}

fn quiet_value(v: Option<&str>) -> bool {
    v.map(|v| v.trim().to_ascii_lowercase())
        .is_some_and(|v| !matches!(v.as_str(), "" | "0" | "no" | "false" | "off"))
}

/// Watches a recording's rows and says what it does (`Happened`), each once when it happens.
pub struct Notifier {
    pub quiet: bool,
    desktop: Box<dyn Desktop>,
    recording: bool,
    /// When the last row (or the start) was, and whether the engine was off in it (no row is then expected).
    last: f64,
    engine_off: bool,
    stalled: bool,
    pub stall_after: f64,
}

impl Notifier {
    pub fn new(quiet: bool, desktop: Box<dyn Desktop>) -> Notifier {
        Notifier {
            quiet,
            desktop,
            recording: false,
            last: 0.0,
            engine_off: false,
            stalled: false,
            stall_after: STALL_AFTER,
        }
    }

    /// The desktop's own programs.
    pub fn system(quiet: bool) -> Notifier {
        Notifier::new(quiet, Box::new(System))
    }

    /// A recording started at `now` (seconds on any clock the caller keeps), into `file`.
    pub fn started(&mut self, now: f64, file: &str) {
        self.recording = true;
        self.last = now;
        self.engine_off = false;
        self.stalled = false;
        self.say(
            Happened::Started,
            "Recording started",
            &format!("Icarus is recording the drive ({file})."),
        );
    }

    /// A row was written at `now`; `engine_off` when it showed the engine stopped (rows ending after it are then expected:
    /// the ignition went off).
    pub fn row(&mut self, now: f64, engine_off: bool) {
        if !self.recording {
            return;
        }
        self.last = now;
        self.engine_off = engine_off;
        if self.stalled {
            self.stalled = false;
            self.say(
                Happened::Resumed,
                "Recording resumed",
                "Rows are coming in again.",
            );
        }
    }

    /// The time is `now`: a recording with no row for `stall_after` seconds (the engine last seen running) has stalled.
    pub fn tick(&mut self, now: f64) {
        if self.recording
            && !self.stalled
            && !self.engine_off
            && now - self.last >= self.stall_after
        {
            self.stalled = true;
            self.say(
                Happened::Stalled,
                "Recording stalled",
                &format!(
                    "No rows for {:.0} s: the adapter or the vehicle stopped answering. Icarus keeps trying.",
                    self.stall_after
                ),
            );
        }
    }

    /// The recording stopped with `rows` rows (all its parts).
    pub fn stopped(&mut self, rows: usize, file: &str) {
        if !self.recording {
            return;
        }
        self.recording = false;
        self.stalled = false;
        self.say(
            Happened::Stopped,
            "Recording stopped",
            &format!("{rows} rows saved ({file})."),
        );
    }

    /// A notice of something else about the recording (it goes on without the GPS): shown, no tone; nothing when quiet.
    pub fn notice(&mut self, title: &str, body: &str) {
        if self.quiet || !self.desktop.has("notify-send") {
            return;
        }
        self.desktop.run(
            "notify-send",
            &[
                "--app-name=Icarus".into(),
                "--icon=icarus".into(),
                "--urgency=normal".into(),
                title.into(),
                body.into(),
            ],
        );
    }

    fn say(&mut self, what: Happened, title: &str, body: &str) {
        if self.quiet {
            return;
        }
        self.tone(what);
        if self.desktop.has("notify-send") {
            let urgency = if what == Happened::Stalled {
                "critical"
            } else {
                "normal"
            };
            self.desktop.run(
                "notify-send",
                &[
                    "--app-name=Icarus".into(),
                    "--icon=icarus".into(),
                    format!("--urgency={urgency}"),
                    title.into(),
                    body.into(),
                ],
            );
        }
    }

    /// The tone through the first player there is: a file player with the theme's file, else canberra by the name.
    fn tone(&mut self, what: Happened) {
        let file = PathBuf::from(SOUNDS).join(format!("{}.oga", what.sound()));
        let have_file = self.desktop.exists(&file);
        for player in ["pw-play", "paplay"] {
            if have_file && self.desktop.has(player) {
                self.desktop
                    .run(player, &[file.to_string_lossy().into_owned()]);
                return;
            }
        }
        if self.desktop.has("canberra-gtk-play") {
            self.desktop.run(
                "canberra-gtk-play",
                &[
                    "-i".into(),
                    what.sound().into(),
                    "-d".into(),
                    "Icarus".into(),
                ],
            );
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    /// The commands a stand-in was asked to run: (program, arguments).
    pub(crate) type Ran = Arc<Mutex<Vec<(String, Vec<String>)>>>;

    /// A desktop that has the programs and files it is given and keeps every command it was asked to run.
    #[derive(Clone, Default)]
    pub(crate) struct StandIn {
        pub programs: Vec<&'static str>,
        pub files: bool,
        pub ran: Ran,
    }

    impl Desktop for StandIn {
        fn has(&self, program: &str) -> bool {
            self.programs.contains(&program)
        }
        fn exists(&self, _path: &Path) -> bool {
            self.files
        }
        fn run(&mut self, program: &str, args: &[String]) {
            self.ran
                .lock()
                .unwrap()
                .push((program.to_string(), args.to_vec()));
        }
    }

    impl StandIn {
        pub(crate) fn every() -> StandIn {
            StandIn {
                programs: vec!["pw-play", "paplay", "canberra-gtk-play", "notify-send"],
                files: true,
                ran: Default::default(),
            }
        }
        /// The notifications shown so far: their titles and texts.
        pub(crate) fn shown(&self) -> Vec<(String, String)> {
            self.ran
                .lock()
                .unwrap()
                .iter()
                .filter(|(p, _)| p == "notify-send")
                .map(|(_, a)| (a[a.len() - 2].clone(), a[a.len() - 1].clone()))
                .collect()
        }
        pub(crate) fn tones(&self) -> Vec<String> {
            self.ran
                .lock()
                .unwrap()
                .iter()
                .filter(|(p, _)| p != "notify-send")
                .map(|(_, a)| a.join(" "))
                .collect()
        }
    }

    #[test]
    fn started_stalled_resumed_and_stopped_each_once() {
        let d = StandIn::every();
        let mut n = Notifier::new(false, Box::new(d.clone()));
        n.tick(0.0);
        assert!(d.shown().is_empty(), "nothing before a recording");
        n.started(100.0, "drive-x.csv");
        for t in [100.5, 101.0, 101.5] {
            n.row(t, false);
            n.tick(t);
        }
        n.tick(110.0);
        assert_eq!(d.shown().len(), 1, "8.5 s without a row is not a stall yet");
        n.tick(111.5);
        n.tick(115.0);
        n.tick(130.0);
        n.row(131.0, false);
        n.row(131.5, false);
        n.tick(132.0);
        n.stopped(1234, "drive-x.csv");
        n.stopped(1234, "drive-x.csv");
        let shown = d.shown();
        let titles: Vec<&str> = shown.iter().map(|s| s.0.as_str()).collect();
        assert_eq!(
            titles,
            [
                "Recording started",
                "Recording stalled",
                "Recording resumed",
                "Recording stopped"
            ]
        );
        assert!(shown[0].1.contains("drive-x.csv"), "{shown:?}");
        assert!(shown[1].1.contains("No rows for 10 s"), "{shown:?}");
        assert_eq!(shown[3].1, "1234 rows saved (drive-x.csv).");
        // a tone of its own for each, through the first player there is
        let tones = d.tones();
        assert_eq!(tones.len(), 4);
        assert!(
            tones[0].ends_with("service-login.oga") && tones[1].ends_with("dialog-warning.oga")
        );
        assert!(
            d.ran
                .lock()
                .unwrap()
                .iter()
                .all(|(p, _)| p == "pw-play" || p == "notify-send"),
            "pw-play first"
        );
    }

    #[test]
    fn an_engine_switched_off_is_no_stall_and_quiet_is_silent() {
        let d = StandIn::every();
        let mut n = Notifier::new(false, Box::new(d.clone()));
        n.started(0.0, "a.csv");
        n.row(1.0, true);
        n.tick(30.0);
        assert_eq!(
            d.shown().len(),
            1,
            "the ignition went off: rows end, no stall"
        );
        // silenced: nothing at all
        let q = StandIn::every();
        let mut n = Notifier::new(true, Box::new(q.clone()));
        n.started(0.0, "a.csv");
        n.tick(20.0);
        n.row(21.0, false);
        n.stopped(3, "a.csv");
        assert!(q.ran.lock().unwrap().is_empty());
    }

    #[test]
    fn the_player_is_picked_and_a_missing_one_passed_over() {
        // no file player and no theme files: canberra by the sound's name
        let d = StandIn {
            programs: vec!["canberra-gtk-play", "notify-send"],
            files: false,
            ran: Default::default(),
        };
        let mut n = Notifier::new(false, Box::new(d.clone()));
        n.started(0.0, "a.csv");
        assert_eq!(d.tones(), ["-i service-login -d Icarus"]);
        // paplay when pw-play is missing
        let d = StandIn {
            programs: vec!["paplay"],
            files: true,
            ran: Default::default(),
        };
        let mut n = Notifier::new(false, Box::new(d.clone()));
        n.started(0.0, "a.csv");
        assert_eq!(d.ran.lock().unwrap()[0].0, "paplay");
        assert!(
            d.shown().is_empty(),
            "no notify-send: nothing shown, nothing said"
        );
        // nothing there at all: nothing run
        let d = StandIn::default();
        let mut n = Notifier::new(false, Box::new(d.clone()));
        n.started(0.0, "a.csv");
        n.stopped(1, "a.csv");
        assert!(d.ran.lock().unwrap().is_empty());
    }

    #[test]
    fn the_quiet_variable() {
        assert!(!quiet_value(None) && !quiet_value(Some("")) && !quiet_value(Some("0")));
        assert!(!quiet_value(Some("off")) && !quiet_value(Some("False")));
        assert!(quiet_value(Some("1")) && quiet_value(Some("yes")));
    }
}
