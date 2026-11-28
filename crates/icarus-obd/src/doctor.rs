//! Checking my setup in one go (ICA-216, `icarus doctor` and the window's Check my setup): each problem that stops Icarus
//! reaching an adapter, or telling me what a recording does, named with a plain verdict and what to do about it.
//!
//! - **Adapters**: the USB tuners and cables plugged in (`devices::scan_in`) and the serial ports there, each yours to
//!   open or not, with `devices`' own advice (the udev rule, the uucp group).
//! - **The serial ports' group**: whether I am in the group that owns them (uucp on Arch, dialout elsewhere) in this
//!   session, joined but not yet logged in again, or not at all.
//! - **The udev rule**: `70-icarus-tuners.rules` in `/etc/udev/rules.d` or the package's `/usr/lib/udev/rules.d`, and
//!   whether it is this build's.
//! - **Bluetooth**: a controller, not blocked (rfkill), bluetoothd running, bluetoothctl there and the controller on;
//!   **a paired OBD adapter** (an OBDLink MX+ and the like, by name); **rfcomm**: what is bound (none is needed).
//! - **Who holds an adapter**: the serial ports open in another program (/proc, as `port::open_serial` names it since
//!   ICA-104), which Icarus program that is, and the Icarus programs running.
//! - **Notices and tones**: notify-send and a player for the sound theme (`notify`'s own list).
//! - **This build and the installed one**, **the units**, **the Read-Only list** (the vehicles' names, never their VINs)
//!   and **where the files are**.
//! - With `--adapter A` only, **the adapter itself** (`adapter_check`): its ID, chip, device line, serial speed and the
//!   voltage at the port, refused while a recording or another program has it.
//!
//! Nothing here changes the computer: every read goes through a `SystemTree` (no write in it; commands only from its
//! read-only list), what to do is advice text only (`sudo usermod -aG uucp $USER, then log in again`), and no program is
//! ever stopped. Every line is passed through `scrub`, so no MAC address or VIN is shown in full; a device's serial number
//! is never read.

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

use crate::systree::SystemTree;

/// How a check came out.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Verdict {
    /// Works.
    Ok,
    /// Worth knowing, nothing to do.
    Note,
    /// Works in part, or may get in the way: what to do is said.
    Warn,
    /// Stops Icarus doing something: what to do is said.
    Fail,
}

impl Verdict {
    /// The word shown before a check.
    pub fn word(self) -> &'static str {
        match self {
            Verdict::Ok => "ok",
            Verdict::Note => "note",
            Verdict::Warn => "check",
            Verdict::Fail => "fix",
        }
    }
}

/// One check's outcome.
#[derive(Clone, Debug, PartialEq)]
pub struct Check {
    /// Its short name (`adapters`, `group`, `udev`...), what the tests and the window go by.
    pub id: &'static str,
    pub title: &'static str,
    pub verdict: Verdict,
    /// What was found, in a sentence.
    pub found: String,
    /// What to do, when something is to be done.
    pub fix: Option<String>,
    /// The details, a line each.
    pub lines: Vec<String>,
}

impl Check {
    fn new(id: &'static str, title: &'static str, verdict: Verdict, found: String) -> Check {
        Check {
            id,
            title,
            verdict,
            found,
            fix: None,
            lines: Vec::new(),
        }
    }

    fn fix(mut self, fix: impl Into<String>) -> Check {
        self.fix = Some(fix.into());
        self
    }

    fn lines(mut self, lines: Vec<String>) -> Check {
        self.lines = lines;
        self
    }

    /// The same with every MAC address and VIN in it masked (`scrub`).
    fn scrubbed(self) -> Check {
        Check {
            found: scrub(&self.found),
            fix: self.fix.as_deref().map(scrub),
            lines: self.lines.iter().map(|l| scrub(l)).collect(),
            ..self
        }
    }
}

/// Every check's outcome, in order.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Report {
    pub checks: Vec<Check>,
}

impl Report {
    /// The worst verdict (Ok for none).
    pub fn worst(&self) -> Verdict {
        self.checks
            .iter()
            .map(|c| c.verdict)
            .max()
            .unwrap_or(Verdict::Ok)
    }

    pub fn get(&self, id: &str) -> Option<&Check> {
        self.checks.iter().find(|c| c.id == id)
    }

    /// How many checks came out `v`.
    pub fn count(&self, v: Verdict) -> usize {
        self.checks.iter().filter(|c| c.verdict == v).count()
    }

    /// Adds a check (the adapter's, `adapter_check`), masked as the rest are.
    pub fn push(&mut self, c: Check) {
        self.checks.push(c.scrubbed());
    }

    /// As `icarus doctor` prints it.
    pub fn text(&self) -> String {
        let mut o = String::new();
        for c in &self.checks {
            o.push_str(&format!(
                "[{}] {}: {}\n",
                c.verdict.word(),
                c.title,
                c.found
            ));
            for l in &c.lines {
                o.push_str(&format!("      {l}\n"));
            }
            if let Some(f) = &c.fix {
                o.push_str(&format!("      what to do: {f}\n"));
            }
        }
        let (fix, check) = (self.count(Verdict::Fail), self.count(Verdict::Warn));
        o.push_str(&match (fix, check) {
            (0, 0) => "Everything Icarus needs here is in place.\n".to_string(),
            _ => format!(
                "{fix} to fix, {check} to check, {} fine. Nothing was changed: what to do is for me to do.\n",
                self.checks.len() - fix - check
            ),
        });
        o
    }
}

/// `text` with every MAC address (`AA:BB:CC:DD:EE:FF`, and BlueZ's `AA_BB_CC_DD_EE_FF`) shown by its maker's part only
/// (`AA:BB:CC:**:**:**`) and every VIN masked as Icarus masks it elsewhere (`vin::masked`: its serial part hidden).
pub fn scrub(text: &str) -> String {
    let chars: Vec<char> = text.chars().collect();
    let mut out = String::with_capacity(text.len());
    let mut i = 0;
    let boundary = |j: usize| j >= chars.len() || !chars[j].is_ascii_alphanumeric();
    while i < chars.len() {
        let at_start = i == 0 || !chars[i - 1].is_ascii_alphanumeric();
        if at_start && i + 17 <= chars.len() && boundary(i + 17) {
            let w: String = chars[i..i + 17].iter().collect();
            if let Some(sep) = mac_separator(&w) {
                let s = sep.to_string();
                out.push_str(&w[..9]);
                out.push_str(&["**"; 3].join(&s));
                i += 17;
                continue;
            }
            if vin_like(&w) {
                out.push_str(&crate::vin::masked(&w));
                i += 17;
                continue;
            }
        }
        out.push(chars[i]);
        i += 1;
    }
    out
}

/// The separator of a MAC address written `w` (17 characters), when it is one.
fn mac_separator(w: &str) -> Option<char> {
    let sep = w.chars().nth(2)?;
    if sep != ':' && sep != '_' && sep != '-' {
        return None;
    }
    let parts: Vec<&str> = w.split(sep).collect();
    (parts.len() == 6
        && parts
            .iter()
            .all(|p| p.len() == 2 && p.bytes().all(|b| b.is_ascii_hexdigit())))
    .then_some(sep)
}

/// Whether 17 characters read as a VIN: letters and digits, never I, O or Q, both letters and digits in it.
fn vin_like(w: &str) -> bool {
    w.len() == 17
        && w.chars().all(|c| {
            c.is_ascii_digit() || (c.is_ascii_uppercase() && !matches!(c, 'I' | 'O' | 'Q'))
        })
        && w.chars().any(|c| c.is_ascii_digit())
        && w.chars().filter(|c| c.is_ascii_alphabetic()).count() >= 2
}

/// Who I am, as /proc and /etc say.
#[derive(Clone, Debug, Default)]
struct Me {
    user: String,
    pid: Option<u32>,
    /// This session's groups (what the kernel gives this program).
    now: BTreeSet<String>,
    /// The groups /etc/group lists me in (and my primary group): what the next login gives.
    listed: BTreeSet<String>,
    /// Every group: number to name.
    groups: BTreeMap<u32, String>,
    known: bool,
}

fn me(tree: &dyn SystemTree) -> Me {
    let mut m = Me::default();
    let status = tree
        .read(Path::new("/proc/self/status"))
        .unwrap_or_default();
    let field = |name: &str| -> Vec<u32> {
        status
            .lines()
            .find_map(|l| l.strip_prefix(name))
            .map(|v| {
                v.split_whitespace()
                    .filter_map(|n| n.parse().ok())
                    .collect()
            })
            .unwrap_or_default()
    };
    let uid = field("Uid:").first().copied();
    let mut gids = field("Groups:");
    gids.extend(field("Gid:").first());
    for l in tree
        .read(Path::new("/etc/group"))
        .unwrap_or_default()
        .lines()
    {
        let f: Vec<&str> = l.split(':').collect();
        if let [name, _, gid, ..] = f.as_slice() {
            if let Ok(g) = gid.parse() {
                m.groups.insert(g, name.to_string());
            }
        }
    }
    let mut primary = None;
    for l in tree
        .read(Path::new("/etc/passwd"))
        .unwrap_or_default()
        .lines()
    {
        let f: Vec<&str> = l.split(':').collect();
        if let [name, _, u, g, ..] = f.as_slice() {
            if uid.is_some() && u.parse::<u32>().ok() == uid {
                m.user = name.to_string();
                primary = g.parse::<u32>().ok();
            }
        }
    }
    if m.user.is_empty() {
        m.user = tree.var("USER").unwrap_or_default();
    }
    m.now = gids
        .iter()
        .filter_map(|g| m.groups.get(g).cloned())
        .collect();
    for l in tree
        .read(Path::new("/etc/group"))
        .unwrap_or_default()
        .lines()
    {
        let f: Vec<&str> = l.split(':').collect();
        if let [name, _, gid, members] = f.as_slice() {
            if (!m.user.is_empty() && members.split(',').any(|x| x.trim() == m.user))
                || (primary.is_some() && gid.parse::<u32>().ok() == primary)
            {
                m.listed.insert(name.to_string());
            }
        }
    }
    m.pid = tree
        .read_link(Path::new("/proc/self"))
        .and_then(|p| p.to_string_lossy().parse().ok());
    m.known = uid.is_some() && !m.groups.is_empty();
    m
}

/// A serial port in /dev an adapter could be on.
#[derive(Clone, Debug)]
struct SerialPort {
    path: PathBuf,
    name: String,
    group: Option<String>,
    mine: bool,
    /// The other programs that have it open (this one left out).
    holders: Vec<(u32, String)>,
}

fn serial_ports(tree: &dyn SystemTree, me: &Me) -> Vec<SerialPort> {
    tree.list(Path::new("/dev"))
        .unwrap_or_default()
        .into_iter()
        .filter(|n| {
            ["ttyUSB", "ttyACM", "rfcomm"]
                .iter()
                .any(|p| n.starts_with(p))
        })
        .map(|name| {
            let path = Path::new("/dev").join(&name);
            SerialPort {
                group: tree
                    .owner(&path)
                    .and_then(|o| me.groups.get(&o.gid).cloned()),
                mine: tree.may_open(&path),
                holders: crate::port::holding_in(tree, Path::new("/proc"), &path)
                    .into_iter()
                    .filter(|(pid, _)| Some(*pid) != me.pid)
                    .collect(),
                path,
                name,
            }
        })
        .collect()
}

/// An Icarus program running (not this one): its process, its program name and its command line.
#[derive(Clone, Debug)]
struct Running {
    pid: u32,
    comm: String,
    args: Vec<String>,
}

impl Running {
    fn describe(&self) -> String {
        let what = if self.comm == "icarus-app" {
            "an Icarus window".to_string()
        } else if self.recording() {
            "an Icarus recording".to_string()
        } else {
            "the icarus command".to_string()
        };
        let line = if self.args.is_empty() {
            self.comm.clone()
        } else {
            let mut a = self.args.clone();
            if let Some(first) = a.first_mut() {
                *first = self.comm.clone();
            }
            a.join(" ")
        };
        format!("{line} (process {}, {what})", self.pid)
    }

    /// `icarus record`, which holds the adapter for the drive (not `icarus record redecode`, which reads files).
    fn recording(&self) -> bool {
        self.comm == "icarus"
            && self.args.get(1).map(String::as_str) == Some("record")
            && self.args.get(2).map(String::as_str) != Some("redecode")
    }

    /// The adapter its command line names (`--adapter A`), when it names one.
    fn adapter(&self) -> Option<&str> {
        self.args
            .iter()
            .position(|a| a == "--adapter")
            .and_then(|i| self.args.get(i + 1))
            .map(String::as_str)
            .or_else(|| self.args.iter().find_map(|a| a.strip_prefix("--adapter=")))
    }
}

fn running_icarus(tree: &dyn SystemTree, me: &Me) -> Vec<Running> {
    let mut out = Vec::new();
    for p in tree.list(Path::new("/proc")).unwrap_or_default() {
        let Ok(pid) = p.parse::<u32>() else {
            continue;
        };
        if Some(pid) == me.pid {
            continue;
        }
        let dir = Path::new("/proc").join(&p);
        let comm = tree
            .read(&dir.join("comm"))
            .map(|c| c.trim().to_string())
            .unwrap_or_default();
        if comm != "icarus" && comm != "icarus-app" {
            continue;
        }
        let args = tree
            .read(&dir.join("cmdline"))
            .unwrap_or_default()
            .split('\0')
            .filter(|a| !a.is_empty())
            .map(str::to_string)
            .collect();
        out.push(Running { pid, comm, args });
    }
    out
}

fn process_running(tree: &dyn SystemTree, name: &str) -> bool {
    tree.list(Path::new("/proc"))
        .unwrap_or_default()
        .iter()
        .filter(|p| p.parse::<u32>().is_ok())
        .any(|p| {
            tree.read(&Path::new("/proc").join(p).join("comm"))
                .is_some_and(|c| c.trim() == name)
        })
}

/// Where USB devices are listed.
const USB_DEVICES: &str = "/sys/bus/usb/devices";
/// Where the udev rule goes: from a checkout (`scripts/install-local.sh --udev`), and from the icarus-git package.
pub const RULE_ETC: &str = "/etc/udev/rules.d/70-icarus-tuners.rules";
pub const RULE_LIB: &str = "/usr/lib/udev/rules.d/70-icarus-tuners.rules";
/// This build's udev rule.
const RULE: &str = include_str!("../../../packaging/70-icarus-tuners.rules");
/// This build's version.
pub const BUILD_VERSION: &str = include_str!("../../../VERSION").trim_ascii();

/// The words in an OBD adapter's Bluetooth name (an OBDLink MX+, "OBDII", a vLinker...), any case.
const OBD_NAMES: &[&str] = &[
    "obdlink", "obd", "elm327", "vlink", "v-link", "vgate", "vlinker", "veepeak", "carista",
    "konnwei", "bafx", "kiwi", "scan",
];

/// Whether a Bluetooth name is an OBD adapter's.
pub fn obd_name(name: &str) -> bool {
    let n = name.to_ascii_lowercase();
    OBD_NAMES.iter().any(|w| n.contains(w))
}

/// Every check but the adapter's own, read through `tree`.
pub fn run(tree: &dyn SystemTree) -> Report {
    let me = me(tree);
    let ports = serial_ports(tree, &me);
    let found = crate::devices::scan_in(tree, Path::new(USB_DEVICES));
    let bt = bluetooth(tree);
    let checks = vec![
        adapters(tree, &found, &ports),
        group(&me, &ports),
        udev(tree, &found),
        bt.0,
        paired(tree, bt.1),
        rfcomm(tree, &ports),
        holders(tree, &me, &ports),
        notices(tree),
        version(tree),
        units(tree),
        read_only(tree),
        files(tree),
    ];
    Report {
        checks: checks.into_iter().map(Check::scrubbed).collect(),
    }
}

fn adapters(tree: &dyn SystemTree, found: &[crate::devices::Found], ports: &[SerialPort]) -> Check {
    let mut lines = Vec::new();
    let mut blocked: Vec<String> = Vec::new();
    let mut theirs: BTreeSet<String> = BTreeSet::new();
    for f in found {
        let status = f.status(|p| tree.may_open(p));
        lines.push(format!(
            "{} {} ({:04x}:{:04x}{}): {status}",
            f.known.maker,
            f.known.name,
            f.vendor,
            f.product,
            if f.label.is_empty() {
                String::new()
            } else {
                format!(", says \"{}\"", f.label)
            }
        ));
        if !status.starts_with("ready") {
            blocked.push(status);
        }
        theirs.extend(f.ports.iter().cloned());
    }
    for p in ports.iter().filter(|p| !theirs.contains(&p.name)) {
        let group = p
            .group
            .as_deref()
            .map(|g| format!(" (group {g})"))
            .unwrap_or_default();
        if p.mine {
            lines.push(format!(
                "{}: a serial port, yours to open",
                p.path.display()
            ));
        } else {
            let s = format!(
                "{} is there but not yours to open{group}: {}",
                p.path.display(),
                crate::devices::UUCP
            );
            lines.push(format!(
                "{}: a serial port, not yours to open{group}",
                p.path.display()
            ));
            blocked.push(s);
        }
    }
    if found.is_empty() && ports.is_empty() {
        return Check::new(
            "adapters",
            "Adapters",
            Verdict::Note,
            "no USB adapter or tuner Icarus knows is plugged in, and no serial port is there (Bluetooth is checked below)"
                .into(),
        );
    }
    let n = found.len() + ports.iter().filter(|p| !theirs.contains(&p.name)).count();
    match blocked.first() {
        None => Check::new(
            "adapters",
            "Adapters",
            Verdict::Ok,
            format!("{n} plugged in, each yours to open"),
        )
        .lines(lines),
        Some(first) => Check::new(
            "adapters",
            "Adapters",
            Verdict::Fail,
            format!("{} of {n} not yours to open", blocked.len()),
        )
        .fix(first.clone())
        .lines(lines),
    }
}

fn group(me: &Me, ports: &[SerialPort]) -> Check {
    const T: &str = "Serial ports' group";
    if !me.known {
        return Check::new(
            "group",
            T,
            Verdict::Note,
            "could not read who I am (/proc/self/status, /etc/group)".into(),
        );
    }
    let names: BTreeSet<&str> = me.groups.values().map(String::as_str).collect();
    let g = ports
        .iter()
        .find_map(|p| p.group.clone().filter(|g| g != "root"))
        .unwrap_or_else(|| {
            if !names.contains("uucp") && names.contains("dialout") {
                "dialout".into()
            } else {
                "uucp".into()
            }
        });
    let join = format!("sudo usermod -aG {g} $USER, then log in again");
    if me.now.contains(&g) {
        return Check::new(
            "group",
            T,
            Verdict::Ok,
            format!(
                "{} is in the {g} group, which owns the serial ports",
                me.user
            ),
        );
    }
    if me.listed.contains(&g) {
        return Check::new(
            "group",
            T,
            Verdict::Warn,
            format!(
                "{} was added to the {g} group, but this session began before that, so it does not have it yet",
                me.user
            ),
        )
        .fix("log out and in again (or restart)");
    }
    let blocked = ports
        .iter()
        .any(|p| !p.mine && p.group.as_deref() == Some(g.as_str()));
    Check::new(
        "group",
        T,
        if blocked { Verdict::Fail } else { Verdict::Warn },
        format!(
            "{} is not in the {g} group: a serial cable the udev rule does not give the person at the seat (an FT232R \
             ELM327 cable, a 0403:6015 that is not an OBDLink) opens only to that group",
            if me.user.is_empty() { "this user" } else { &me.user }
        ),
    )
    .fix(join)
}

/// A rule file's lines that do something (no comments, no blank lines), each trimmed.
fn rule_lines(text: &str) -> Vec<String> {
    text.lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .map(str::to_string)
        .collect()
}

fn udev(tree: &dyn SystemTree, found: &[crate::devices::Found]) -> Check {
    const T: &str = "udev rule";
    let etc = tree.read(Path::new(RULE_ETC));
    let lib = tree.read(Path::new(RULE_LIB));
    let install = "install the icarus-git package (it puts the rule in /usr/lib/udev/rules.d), or from a checkout \
                   bash scripts/install-local.sh --udev; then plug the device in again";
    let (path, text) = match (&etc, &lib) {
        (Some(t), _) => (RULE_ETC, t),
        (None, Some(t)) => (RULE_LIB, t),
        (None, None) => {
            let needed = found.iter().any(|f| {
                crate::devices::udev_rule_covers(f.vendor, f.product, &f.product_name)
                    && !f.status(|p| tree.may_open(p)).starts_with("ready")
            });
            return Check::new(
                "udev",
                T,
                if needed { Verdict::Fail } else { Verdict::Warn },
                "Icarus's udev rule (70-icarus-tuners.rules) is not installed: the tuners with no Linux driver, the \
                 CP210x cables and the OBDLink EX/SX are not given to the person at the seat"
                    .into(),
            )
            .fix(install);
        }
    };
    let mut lines = vec![format!("in use: {path}")];
    if etc.is_some() && lib.is_some() {
        lines.push(format!(
            "{RULE_ETC} overrides the package's copy in {RULE_LIB}"
        ));
    }
    if rule_lines(text) == rule_lines(RULE) {
        Check::new(
            "udev",
            T,
            Verdict::Ok,
            "installed, the same as this build's".into(),
        )
        .lines(lines)
    } else {
        Check::new(
            "udev",
            T,
            Verdict::Warn,
            "installed, but not this build's rule (older, or edited)".into(),
        )
        .fix(if path == RULE_ETC && lib.as_deref().map(rule_lines) == Some(rule_lines(RULE)) {
            "the package's copy is this build's: remove the older /etc/udev/rules.d/70-icarus-tuners.rules (sudo rm), \
             or put this build's there (bash scripts/install-local.sh --udev from the checkout); then plug the device \
             in again"
        } else if path == RULE_ETC {
            "put this build's in place: bash scripts/install-local.sh --udev from the checkout (and update the \
             icarus-git package); then plug the device in again"
        } else {
            "update the icarus-git package (Agora, or sudo pacman -Syu); then plug the device in again"
        })
        .lines(lines)
    }
}

/// The Bluetooth check, and whether paired devices can be listed (bluetoothd running, bluetoothctl there).
fn bluetooth(tree: &dyn SystemTree) -> (Check, Result<(), String>) {
    const T: &str = "Bluetooth";
    let controllers: Vec<String> = tree
        .list(Path::new("/sys/class/bluetooth"))
        .unwrap_or_default()
        .into_iter()
        .filter(|n| n.starts_with("hci") && !n.contains(':'))
        .collect();
    let daemon = process_running(tree, "bluetoothd");
    let tool = tree.which("bluetoothctl").is_some();
    let can_list = match (daemon, tool) {
        (true, true) => Ok(()),
        (false, _) => Err("bluetoothd is not running".to_string()),
        (true, false) => Err("bluetoothctl is not installed".to_string()),
    };
    if controllers.is_empty() {
        return (
            Check::new(
                "bluetooth",
                T,
                Verdict::Note,
                "no Bluetooth controller on this computer (or it is switched off in the firmware)"
                    .into(),
            ),
            can_list,
        );
    }
    let mut lines = vec![format!("controller: {}", controllers.join(", "))];
    // rfkill: a controller switched off by software (the desktop, rfkill) or by a switch or key
    let mut soft = false;
    let mut hard = false;
    let rf = Path::new("/sys/class/rfkill");
    for r in tree.list(rf).unwrap_or_default() {
        let d = rf.join(&r);
        if tree
            .read(&d.join("type"))
            .is_some_and(|t| t.trim() == "bluetooth")
        {
            soft |= tree.read(&d.join("soft")).is_some_and(|t| t.trim() == "1");
            hard |= tree.read(&d.join("hard")).is_some_and(|t| t.trim() == "1");
        }
    }
    let check = if hard {
        Check::new(
            "bluetooth",
            T,
            Verdict::Fail,
            "the controller is switched off by a switch or key on the computer".into(),
        )
        .fix("switch it on with that switch or key (often Fn with the aeroplane or Bluetooth key)")
    } else if soft {
        Check::new(
            "bluetooth",
            T,
            Verdict::Fail,
            "the controller is blocked (rfkill)".into(),
        )
        .fix("switch Bluetooth on in the desktop's settings, or: rfkill unblock bluetooth")
    } else if !daemon {
        Check::new(
            "bluetooth",
            T,
            Verdict::Fail,
            "bluetoothd, the Bluetooth service, is not running".into(),
        )
        .fix("sudo systemctl enable --now bluetooth")
    } else if !tool {
        Check::new(
            "bluetooth",
            T,
            Verdict::Warn,
            "bluetoothd runs, but bluetoothctl is not installed, so the paired adapters cannot be listed".into(),
        )
        .fix("sudo pacman -S bluez-utils")
    } else {
        let show = tree.output("bluetoothctl", &["show"]).unwrap_or_default();
        let powered = show
            .lines()
            .map(clean)
            .find_map(|l| l.trim().strip_prefix("Powered:").map(|v| v.trim() == "yes"));
        if powered == Some(false) {
            Check::new(
                "bluetooth",
                T,
                Verdict::Warn,
                "bluetoothd runs, but the controller is off".into(),
            )
            .fix("switch Bluetooth on in the desktop's settings (or: bluetoothctl power on)")
        } else {
            Check::new(
                "bluetooth",
                T,
                Verdict::Ok,
                "a controller, and bluetoothd running".into(),
            )
        }
    };
    if daemon {
        lines.push("bluetoothd: running".into());
    }
    (check.lines(lines), can_list)
}

/// A line of a command's output with terminal colors and prompts taken out.
fn clean(line: &str) -> String {
    let mut out = String::new();
    let mut chars = line.chars().peekable();
    while let Some(c) = chars.next() {
        if c == '\u{1b}' {
            // ESC [ ... letter
            if chars.peek() == Some(&'[') {
                chars.next();
                for d in chars.by_ref() {
                    if d.is_ascii_alphabetic() {
                        break;
                    }
                }
            }
            continue;
        }
        if c != '\u{1}' && c != '\u{2}' && c != '\r' {
            out.push(c);
        }
    }
    out
}

/// A Bluetooth name with a number of 4 digits or more at its end hidden: an OBDLink MX+ calls itself "OBDLink MX+" and
/// digits of its serial number.
fn masked_name(name: &str) -> String {
    let name = name.trim();
    match name.rsplit_once(' ') {
        Some((head, tail)) if tail.len() >= 4 && tail.bytes().all(|b| b.is_ascii_digit()) => {
            format!("{head} {}", "*".repeat(tail.len()))
        }
        _ => name.to_string(),
    }
}

/// The devices `bluetoothctl devices Paired` lists: (address, name, its serial digits hidden).
fn paired_devices(text: &str) -> Vec<(String, String)> {
    text.lines()
        .map(clean)
        .filter_map(|l| {
            let rest = l.split_once("Device ")?.1.trim().to_string();
            let (addr, name) = rest.split_once(' ').unwrap_or((rest.as_str(), ""));
            mac_separator(addr)
                .filter(|s| *s == ':')
                .map(|_| (addr.to_uppercase(), masked_name(name)))
        })
        .collect()
}

fn paired(tree: &dyn SystemTree, can_list: Result<(), String>) -> Check {
    const T: &str = "Bluetooth adapter";
    if let Err(why) = can_list {
        return Check::new(
            "paired",
            T,
            Verdict::Note,
            format!("not checked: {why} (see Bluetooth)"),
        );
    }
    let mut list = tree
        .output("bluetoothctl", &["devices", "Paired"])
        .map(|t| paired_devices(&t))
        .unwrap_or_default();
    if list.is_empty() {
        // BlueZ before 5.65 has `paired-devices` instead
        list = tree
            .output("bluetoothctl", &["paired-devices"])
            .map(|t| paired_devices(&t))
            .unwrap_or_default();
    }
    let (obd, other): (Vec<_>, Vec<_>) = list.into_iter().partition(|(_, n)| obd_name(n));
    let mut lines = Vec::new();
    for (addr, name) in &obd {
        let info = tree
            .output("bluetoothctl", &["info", addr])
            .unwrap_or_default();
        let yes = |k: &str| {
            info.lines()
                .map(clean)
                .any(|l| l.trim().strip_prefix(k).is_some_and(|v| v.trim() == "yes"))
        };
        let mut said = vec![if yes("Paired:") || info.is_empty() {
            "paired"
        } else {
            "not paired now"
        }];
        if yes("Trusted:") {
            said.push("trusted");
        }
        if yes("Connected:") {
            said.push("connected now");
        }
        lines.push(format!(
            "{} ({addr}): {}; Icarus reaches it as --adapter bt:{addr}",
            if name.is_empty() { "an adapter" } else { name },
            said.join(", ")
        ));
    }
    if !other.is_empty() {
        lines.push(format!(
            "{} other paired device(s), none with an OBD adapter's name",
            other.len()
        ));
    }
    if obd.is_empty() {
        return Check::new(
            "paired",
            T,
            Verdict::Warn,
            "no OBD adapter (an OBDLink MX+ or the like) is paired with this computer".into(),
        )
        .fix(
            "pair it once, plugged into the vehicle (an MX+: press its button first): the desktop's Bluetooth \
             settings, or bluetoothctl (scan on, then pair and trust its address); then icarus obd info --adapter \
             bt:ADDRESS",
        )
        .lines(lines);
    }
    Check::new(
        "paired",
        T,
        Verdict::Ok,
        format!(
            "{} paired: {}",
            obd.len(),
            obd.iter()
                .map(|(_, n)| if n.is_empty() {
                    "an adapter"
                } else {
                    n.as_str()
                })
                .collect::<Vec<_>>()
                .join(", ")
        ),
    )
    .lines(lines)
}

fn rfcomm(tree: &dyn SystemTree, ports: &[SerialPort]) -> Check {
    const T: &str = "rfcomm";
    let bound: Vec<&SerialPort> = ports
        .iter()
        .filter(|p| p.name.starts_with("rfcomm"))
        .collect();
    let mut lines: Vec<String> = if tree.which("rfcomm").is_some() {
        tree.output("rfcomm", &[])
            .unwrap_or_default()
            .lines()
            .map(clean)
            .map(|l| l.trim().to_string())
            .filter(|l| !l.is_empty())
            .collect()
    } else {
        vec![]
    };
    if bound.is_empty() {
        return Check::new(
            "rfcomm",
            T,
            Verdict::Ok,
            "no rfcomm port bound, and none is needed: Icarus connects to bt:ADDRESS itself".into(),
        )
        .lines(lines);
    }
    let blocked: Vec<&&SerialPort> = bound.iter().filter(|p| !p.mine).collect();
    for p in &bound {
        lines.push(format!(
            "{}: {}",
            p.path.display(),
            if p.mine {
                "yours to open"
            } else {
                "not yours to open"
            }
        ));
    }
    if blocked.is_empty() {
        Check::new(
            "rfcomm",
            T,
            Verdict::Ok,
            format!(
                "{} bound, yours to open (Icarus needs none: bt:ADDRESS reaches the adapter directly)",
                bound.len()
            ),
        )
        .lines(lines)
    } else {
        Check::new(
            "rfcomm",
            T,
            Verdict::Warn,
            format!(
                "{} bound but not yours to open (bt:ADDRESS needs no binding and reaches the adapter all the same)",
                blocked.len()
            ),
        )
        .fix(format!(
            "use --adapter bt:ADDRESS instead; or {}",
            crate::devices::UUCP
        ))
        .lines(lines)
    }
}

fn holders(tree: &dyn SystemTree, me: &Me, ports: &[SerialPort]) -> Check {
    const T: &str = "Who has an adapter";
    let running = running_icarus(tree, me);
    let by_pid: BTreeMap<u32, &Running> = running.iter().map(|r| (r.pid, r)).collect();
    let mut lines = Vec::new();
    let mut held = 0;
    for p in ports.iter().filter(|p| !p.holders.is_empty()) {
        held += 1;
        let who: Vec<String> = p
            .holders
            .iter()
            .map(|(pid, name)| match by_pid.get(pid) {
                Some(r) => r.describe(),
                None => format!("{name} (process {pid})"),
            })
            .collect();
        lines.push(format!("{}: open in {}", p.path.display(), who.join("; ")));
    }
    for r in &running {
        if !ports
            .iter()
            .any(|p| p.holders.iter().any(|(pid, _)| *pid == r.pid))
        {
            lines.push(format!("running: {}", r.describe()));
        }
    }
    lines.push(
        "programs of other users (ModemManager, gpsd) are not seen, and a port's exclusive mark (TIOCEXCL) shows only \
         to a program that opens it, which this check does not"
            .into(),
    );
    if held == 0 {
        return Check::new(
            "holders",
            T,
            Verdict::Ok,
            "no adapter port is open in another program".into(),
        )
        .lines(lines);
    }
    Check::new(
        "holders",
        T,
        Verdict::Warn,
        format!("{held} adapter port(s) open in another program: one program at a time speaks to an adapter"),
    )
    .fix(
        "close that program (in the window: Disconnect; a recording: stop it) before using the adapter elsewhere; \
         Icarus never closes another program for you",
    )
    .lines(lines)
}

/// The window's settings file's value for `key` (`window.toml`, ICA-79), as written.
fn window_setting(tree: &dyn SystemTree, key: &str) -> Option<String> {
    let text = tree.read(&config_dir(tree)?.join("window.toml"))?;
    text.lines().find_map(|l| {
        let l = l.split('#').next().unwrap_or("").trim();
        let (k, v) = l.split_once('=')?;
        (k.trim() == key).then(|| v.trim().trim_matches('"').to_string())
    })
}

fn notices(tree: &dyn SystemTree) -> Check {
    use crate::notify;
    const T: &str = "Notices and tones";
    let shows = tree.which(notify::NOTIFIER).is_some();
    let files = notify::Happened::ALL
        .iter()
        .all(|h| tree.exists(&Path::new(notify::SOUNDS).join(format!("{}.oga", h.sound()))));
    let player = notify::FILE_PLAYERS
        .iter()
        .find(|p| files && tree.which(p).is_some())
        .map(|p| format!("{p} with the freedesktop sound theme"))
        .or_else(|| {
            tree.which(notify::NAME_PLAYER)
                .map(|_| format!("{} by the sound's name", notify::NAME_PLAYER))
        });
    let mut lines = Vec::new();
    lines.push(format!(
        "notices: {}",
        if shows {
            "notify-send"
        } else {
            "none (notify-send is not installed)"
        }
    ));
    lines.push(format!(
        "tones: {}",
        player.as_deref().unwrap_or("none (no player for them)")
    ));
    if tree
        .var(notify::QUIET_ENV)
        .is_some_and(|v| crate::lock::asks_read_only(&v))
    {
        lines.push(format!(
            "{} is set: icarus record plays and shows nothing",
            notify::QUIET_ENV
        ));
    }
    if window_setting(tree, "record_quiet").as_deref() == Some("true") {
        lines.push("the window's recording tones and notices are off (Settings)".into());
    }
    let mut fixes = Vec::new();
    if !shows {
        fixes.push("sudo pacman -S libnotify (for notify-send)");
    }
    if player.is_none() {
        fixes.push(
            "sudo pacman -S sound-theme-freedesktop pipewire-audio (pw-play), or libcanberra",
        );
    }
    if fixes.is_empty() {
        return Check::new(
            "notices",
            T,
            Verdict::Ok,
            "a recording's notices and tones can be shown and played".into(),
        )
        .lines(lines);
    }
    Check::new(
        "notices",
        T,
        Verdict::Warn,
        match (shows, player.is_some()) {
            (false, false) => "a recording can neither show its notices nor play its tones",
            (false, true) => "a recording can play its tones, but not show its notices",
            _ => "a recording can show its notices, but not play its tones",
        }
        .into(),
    )
    .fix(fixes.join("; "))
    .lines(lines)
}

fn version(tree: &dyn SystemTree) -> Check {
    const T: &str = "Version";
    let mut lines = Vec::new();
    if let Some(exe) = tree.read_link(Path::new("/proc/self/exe")) {
        lines.push(format!("this program: {}", tidy_home(tree, &exe)));
        if exe.components().any(|c| c.as_os_str() == "target") {
            lines.push("this run is a build from a checkout, not the installed program".into());
        }
    }
    if let Some(p) = tree.which("icarus") {
        let at = tree.resolve(&p).unwrap_or(p);
        lines.push(format!("icarus on the PATH: {}", tidy_home(tree, &at)));
    }
    let pkg = tree
        .which("pacman")
        .and_then(|_| tree.output("pacman", &["-Q", "icarus-git"]))
        .and_then(|t| {
            let mut w = t.split_whitespace();
            (w.next() == Some("icarus-git"))
                .then(|| w.next().map(str::to_string))
                .flatten()
        });
    let Some(v) = pkg else {
        return Check::new(
            "version",
            T,
            Verdict::Note,
            format!("this build is {BUILD_VERSION}; the icarus-git package is not installed"),
        )
        .lines(lines);
    };
    // epoch:pkgver-pkgrel; icarus-git's pkgver begins with VERSION (1.0.0.r321.gabcdef0)
    let plain = v.split_once(':').map_or(v.as_str(), |(_, r)| r);
    let release = plain
        .split(['.', '-'])
        .take(3)
        .collect::<Vec<_>>()
        .join(".");
    if release == BUILD_VERSION {
        Check::new(
            "version",
            T,
            Verdict::Ok,
            format!(
                "icarus-git {v} is installed, the same release as this build ({BUILD_VERSION})"
            ),
        )
        .lines(lines)
    } else {
        Check::new(
            "version",
            T,
            Verdict::Warn,
            format!("icarus-git {v} is installed, but this build is {BUILD_VERSION}"),
        )
        .fix("update it through Agora (or sudo pacman -Syu), or run the build you mean")
        .lines(lines)
    }
}

/// `path` with the home folder written `~`.
fn tidy_home(tree: &dyn SystemTree, path: &Path) -> String {
    match tree.var("HOME").map(PathBuf::from) {
        Some(h) if path.starts_with(&h) && h.is_absolute() && h != Path::new("/") => {
            let rest = path.strip_prefix(&h).unwrap_or(path);
            if rest.as_os_str().is_empty() {
                "~".into()
            } else {
                format!("~/{}", rest.display())
            }
        }
        _ => path.display().to_string(),
    }
}

fn units(tree: &dyn SystemTree) -> Check {
    const T: &str = "Units";
    let locale = ["LC_ALL", "LC_MEASUREMENT", "LANG"]
        .iter()
        .find_map(|k| tree.var(k))
        .unwrap_or_default();
    let us = ["en_US", "es_US", "en_LR", "my_MM"]
        .iter()
        .any(|p| locale.starts_with(p));
    let system = if us {
        "mph, °F and psi"
    } else {
        "km/h, °C and kPa"
    };
    let word = window_setting(tree, "units");
    let said = match word.as_deref() {
        Some("imperial") => "Imperial (mph, °F, psi)".to_string(),
        Some("metric") => "Metric (km/h, °C, kPa)".to_string(),
        Some("both") => format!("both side by side, {system} first"),
        _ => format!(
            "as the system has it ({}: {system})",
            if locale.is_empty() {
                "no locale set"
            } else {
                &locale
            }
        ),
    };
    let mut lines = vec![];
    if word.is_none() {
        lines.push("the window's Settings have not been saved yet".into());
    }
    if let Some(u) = tree.var("ICARUS_UNITS") {
        lines.push(format!("ICARUS_UNITS={u} (the command's module values)"));
    }
    lines.push("recordings keep metric in their files; what is shown follows this".into());
    Check::new("units", T, Verdict::Ok, format!("the window shows {said}")).lines(lines)
}

/// `$XDG_CONFIG_HOME/icarus`, else `~/.config/icarus`.
fn config_dir(tree: &dyn SystemTree) -> Option<PathBuf> {
    tree.var("XDG_CONFIG_HOME")
        .map(PathBuf::from)
        .filter(|p| p.is_absolute())
        .or_else(|| tree.var("HOME").map(|h| PathBuf::from(h).join(".config")))
        .map(|c| c.join("icarus"))
}

/// `$XDG_STATE_HOME/icarus`, else `~/.local/state/icarus`.
fn state_dir(tree: &dyn SystemTree) -> Option<PathBuf> {
    tree.var("XDG_STATE_HOME")
        .map(PathBuf::from)
        .filter(|p| p.is_absolute())
        .or_else(|| {
            tree.var("HOME")
                .map(|h| PathBuf::from(h).join(".local/state"))
        })
        .map(|c| c.join("icarus"))
}

fn read_only(tree: &dyn SystemTree) -> Check {
    const T: &str = "Read-Only list";
    let dir = config_dir(tree);
    let read = |f: &str| dir.as_ref().and_then(|d| tree.read(&d.join(f)));
    let vins = read("read-only.toml")
        .map(|t| crate::lock::parse_list(&t))
        .unwrap_or_default();
    let names: BTreeMap<String, String> = read("garage.toml")
        .and_then(|t| toml::from_str(&t).ok())
        .unwrap_or_default();
    let details: BTreeMap<String, crate::garage::Details> = read("details.toml")
        .and_then(|t| toml::from_str(&t).ok())
        .unwrap_or_default();
    let mut lines: Vec<String> = vins
        .iter()
        .map(|v| {
            let d = details.get(v).cloned().unwrap_or_default();
            let described = names.get(v).map(|n| crate::garage::Described::parse(n));
            let label = crate::garage::label(v, described.as_ref(), &d);
            if label == *v || label.trim().is_empty() {
                "a vehicle with no name yet (name it on the Garage page)".to_string()
            } else {
                label
            }
        })
        .collect();
    let session = tree
        .var(crate::lock::SESSION_ENV)
        .is_some_and(|v| crate::lock::asks_read_only(&v));
    let found = match vins.len() {
        0 => "no vehicle is switched to Read-Only".to_string(),
        1 => "1 vehicle is switched to Read-Only".to_string(),
        n => format!("{n} vehicles are switched to Read-Only"),
    };
    if session {
        lines.push(format!(
            "{} is set: every run started from here is read-only, whatever the list says",
            crate::lock::SESSION_ENV
        ));
    }
    Check::new("readonly", T, Verdict::Ok, found).lines(lines)
}

fn files(tree: &dyn SystemTree) -> Check {
    const T: &str = "Files";
    let Some(home) = tree.var("HOME").map(PathBuf::from) else {
        return Check::new(
            "files",
            T,
            Verdict::Warn,
            "HOME is not set, so Icarus cannot find its files".into(),
        )
        .fix("start Icarus from a normal login (HOME set)");
    };
    let docs = home.join("Documents/Icarus");
    let config = config_dir(tree).unwrap_or_else(|| home.join(".config/icarus"));
    let state = state_dir(tree).unwrap_or_else(|| home.join(".local/state/icarus"));
    let garage = docs.join("Vehicles");
    let vehicles = tree
        .list(&garage)
        .unwrap_or_default()
        .into_iter()
        .filter(|n| tree.list(&garage.join(n)).is_some())
        .count();
    let at = |what: &str, p: &Path, extra: String| {
        format!(
            "{what}: {}{}{extra}",
            tidy_home(tree, p),
            if tree.exists(p) {
                ""
            } else {
                " (not made yet)"
            }
        )
    };
    let lines = vec![
        at("settings", &config, String::new()),
        at(
            "vehicle profiles of my own",
            &config.join("vehicles"),
            String::new(),
        ),
        at("action log", &state.join("actions.log"), String::new()),
        at(
            "garage",
            &garage,
            if vehicles > 0 {
                format!(", {vehicles} vehicle folder(s), each with its Logs and Reports")
            } else {
                String::new()
            },
        ),
        at(
            "recordings without a vehicle",
            &docs.join("logs"),
            String::new(),
        ),
        at(
            "reports without a vehicle",
            &docs.join("reports"),
            String::new(),
        ),
        at(
            "the simulated vehicle's records",
            &state.join("sim"),
            String::new(),
        ),
    ];
    Check::new(
        "files",
        T,
        Verdict::Ok,
        format!("in {}", tidy_home(tree, &home)),
    )
    .lines(lines)
}

/// The adapter's own check (`icarus doctor --adapter A`): refused while a recording or another program has it, else its
/// identity as `identify` reads it (`elm::identify_target`: reads only, nothing onto the vehicle's bus). `spec` is the
/// adapter as written.
pub fn adapter_check(
    tree: &dyn SystemTree,
    spec: &str,
    target: &crate::port::Target,
    identify: &dyn Fn(&crate::port::Target) -> Result<crate::elm::AdapterInfo, String>,
) -> Check {
    use crate::port::Target;
    const T: &str = "The adapter";
    let me = me(tree);
    let path = match target {
        Target::Serial { path, .. } | Target::Slcan { path, .. } => Some(path.as_str()),
        Target::J2534(crate::port::J2534Device::Mongoose(p)) => Some(p.as_str()),
        Target::J2534(crate::port::J2534Device::OpenPort(p)) => p.as_deref(),
        _ => None,
    };
    let running = running_icarus(tree, &me);
    let same = |a: &str| {
        let a_path = a.split('@').next().unwrap_or("");
        a == spec
            || path.is_some_and(|p| {
                a_path == p
                    || matches!(
                        (tree.resolve(Path::new(a_path)), tree.resolve(Path::new(p))),
                        (Some(x), Some(y)) if x == y
                    )
            })
    };
    let recording: Vec<String> = running
        .iter()
        .filter(|r| r.recording() && r.adapter().is_none_or(same))
        .map(Running::describe)
        .collect();
    if !recording.is_empty() {
        return Check::new(
            "adapter",
            T,
            Verdict::Warn,
            format!(
                "not asked: a recording may have it ({})",
                recording.join("; ")
            ),
        )
        .fix("ask again once the recording has stopped (Icarus never stops it for you)");
    }
    if let Some(p) = path {
        let held: Vec<String> = crate::port::holding_in(tree, Path::new("/proc"), Path::new(p))
            .into_iter()
            .filter(|(pid, _)| Some(*pid) != me.pid)
            .map(|(pid, name)| {
                running
                    .iter()
                    .find(|r| r.pid == pid)
                    .map(Running::describe)
                    .unwrap_or(format!("{name} (process {pid})"))
            })
            .collect();
        if !held.is_empty() {
            return Check::new(
                "adapter",
                T,
                Verdict::Warn,
                format!("not asked: {p} is open in {}", held.join("; ")),
            )
            .fix("close that program first (Icarus never closes it for you), then ask again");
        }
    }
    match identify(target) {
        Ok(a) => {
            let mut lines = vec![format!("ID: {}", a.id)];
            if let Some(s) = &a.stn {
                lines.push(format!("chip: {s}"));
            }
            if let Some(d) = &a.device {
                lines.push(format!("device: {d}"));
            }
            match a.speed {
                Some(b) => lines.push(format!("serial speed: answered at {b} baud")),
                None => lines.push("serial speed: none (not a serial line)".into()),
            }
            if let Some(v) = a.voltage {
                lines.push(format!("voltage at the OBD port: {v:.1} V"));
            }
            if let Some(n) = &a.note {
                lines.push(n.clone());
            }
            Check::new(
                "adapter",
                T,
                Verdict::Ok,
                format!(
                    "{} answered: {}",
                    spec,
                    a.device.clone().or(a.stn.clone()).unwrap_or(a.id.clone())
                ),
            )
            .lines(lines)
            .scrubbed()
        }
        Err(e) => Check::new(
            "adapter",
            T,
            Verdict::Fail,
            format!("{spec} did not answer as an adapter: {e}"),
        )
        .fix(
            "check it is plugged into the vehicle (an OBDLink EX or MX+ takes its power from the OBD port) and, for \
             Bluetooth, paired and in reach",
        )
        .scrubbed(),
    }
}

#[cfg(test)]
mod tests;
