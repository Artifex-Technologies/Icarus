//! The system tree (ICA-216): every read `icarus doctor` makes of this computer, in one small trait, so its tests hand it a
//! made-up computer (`MadeUp`) and never read the real one. File reads, folder listings, symlinks, who may open a device
//! (access(2), which opens nothing), programs on the PATH, a few read-only commands' output and environment variables.
//!
//! There is no write in it: nothing here changes the computer. The only way out to the system that is not a read is a
//! command, and `ThisSystem` runs only those on `read_only_command`'s list (listing paired Bluetooth devices, asking a
//! package's version); any other is refused before it starts. The made-up computer records every command asked and every
//! one that is not on that list as a write, so a test can hold the doctor to none.
//!
//! `devices::scan_in` and `port::holding_in` read through it too, so the doctor sees the same USB devices and the same
//! port holders the rest of Icarus does.

use std::cell::RefCell;
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Component, Path, PathBuf};
use std::time::{Duration, Instant};

/// A file's owner, group and permission bits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Owner {
    pub uid: u32,
    pub gid: u32,
    pub mode: u32,
}

/// What Icarus reads of a computer. Reads only.
pub trait SystemTree {
    /// A file's text (None when it is not there or cannot be read).
    fn read(&self, path: &Path) -> Option<String>;
    /// The names in a folder, sorted (None when it is not a folder this user may read).
    fn list(&self, dir: &Path) -> Option<Vec<String>>;
    /// Where a symlink points (None when `path` is not one).
    fn read_link(&self, path: &Path) -> Option<PathBuf>;
    /// Where `path` leads with every symlink followed (None when it leads nowhere).
    fn resolve(&self, path: &Path) -> Option<PathBuf>;
    /// Whether anything is at `path`.
    fn exists(&self, path: &Path) -> bool;
    /// Its owner, group and mode.
    fn owner(&self, path: &Path) -> Option<Owner>;
    /// Whether this user may read and write `path` (access(2): permissions and ACLs such as udev's uaccess). Opens nothing.
    fn may_open(&self, path: &Path) -> bool;
    /// Where `program` is on the PATH, when it is there and can be run.
    fn which(&self, program: &str) -> Option<PathBuf>;
    /// What a read-only command printed (`read_only_command`; anything else is refused, None), or None when it could not
    /// run or did not finish in a few seconds.
    fn output(&self, program: &str, args: &[&str]) -> Option<String>;
    /// An environment variable.
    fn var(&self, name: &str) -> Option<String>;
}

/// Whether `program args` only reads: the commands the doctor may ask, all of them. Listing Bluetooth devices and asking
/// about one (never `pair`, `trust`, `connect`, `power`, `remove`), the rfcomm bindings (no `bind`, `release`), and a
/// package's installed version.
pub fn read_only_command(program: &str, args: &[&str]) -> bool {
    let mac = |s: &str| {
        s.len() == 17
            && s.split(':').count() == 6
            && s.split(':')
                .all(|b| b.len() == 2 && b.bytes().all(|c| c.is_ascii_hexdigit()))
    };
    match (program, args) {
        ("bluetoothctl", ["devices", "Paired"] | ["paired-devices"] | ["show"] | ["list"]) => true,
        ("bluetoothctl", ["info", a]) => mac(a),
        ("rfcomm", []) => true,
        ("pacman", ["-Q", p]) => !p.starts_with('-'),
        _ => false,
    }
}

/// This computer: its files, its PATH and its programs. HOME and the XDG folders come through `crate::env`, so the
/// window's and the command's tests (the `test-home` feature) only ever get a test's own home.
pub struct ThisSystem;

/// How long a command may take before the doctor gives up on it (and ends that command, its own child, nothing else).
const COMMAND_LIMIT: Duration = Duration::from_secs(4);

impl SystemTree for ThisSystem {
    fn read(&self, path: &Path) -> Option<String> {
        std::fs::read(path)
            .ok()
            .map(|b| String::from_utf8_lossy(&b).into_owned())
    }

    fn list(&self, dir: &Path) -> Option<Vec<String>> {
        let mut v: Vec<String> = std::fs::read_dir(dir)
            .ok()?
            .flatten()
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .collect();
        v.sort();
        Some(v)
    }

    fn read_link(&self, path: &Path) -> Option<PathBuf> {
        std::fs::read_link(path).ok()
    }

    fn resolve(&self, path: &Path) -> Option<PathBuf> {
        std::fs::canonicalize(path).ok()
    }

    fn exists(&self, path: &Path) -> bool {
        std::fs::symlink_metadata(path).is_ok()
    }

    fn owner(&self, path: &Path) -> Option<Owner> {
        use std::os::unix::fs::MetadataExt;
        std::fs::metadata(path).ok().map(|m| Owner {
            uid: m.uid(),
            gid: m.gid(),
            mode: m.mode() & 0o7777,
        })
    }

    fn may_open(&self, path: &Path) -> bool {
        crate::devices::can_open(path)
    }

    fn which(&self, program: &str) -> Option<PathBuf> {
        if program.is_empty() || program.contains('/') {
            return None;
        }
        let path = std::env::var_os("PATH")?;
        std::env::split_paths(&path)
            .map(|d| d.join(program))
            .find(|f| {
                std::fs::metadata(f).is_ok_and(|m| {
                    use std::os::unix::fs::PermissionsExt;
                    m.is_file() && m.permissions().mode() & 0o111 != 0
                })
            })
    }

    fn output(&self, program: &str, args: &[&str]) -> Option<String> {
        if !read_only_command(program, args) {
            return None;
        }
        let exe = self.which(program)?;
        let mut child = std::process::Command::new(exe)
            .args(args)
            .env("LC_ALL", "C")
            .env("NO_COLOR", "1")
            .stdin(std::process::Stdio::null())
            .stdout(std::process::Stdio::piped())
            .stderr(std::process::Stdio::null())
            .spawn()
            .ok()?;
        let mut out = child.stdout.take()?;
        // read on a thread of its own, so a long answer never fills the pipe and holds the command up
        let reader = std::thread::spawn(move || {
            let mut s = Vec::new();
            let _ = std::io::Read::read_to_end(&mut out, &mut s);
            s
        });
        let by = Instant::now() + COMMAND_LIMIT;
        loop {
            match child.try_wait() {
                Ok(Some(_)) => break,
                Ok(None) if Instant::now() < by => std::thread::sleep(Duration::from_millis(20)),
                _ => {
                    // the doctor's own command that did not answer in time (bluetoothctl waiting for a daemon)
                    let _ = child.kill();
                    let _ = child.wait();
                    return None;
                }
            }
        }
        reader
            .join()
            .ok()
            .map(|b| String::from_utf8_lossy(&b).into_owned())
    }

    fn var(&self, name: &str) -> Option<String> {
        let v = if name == "HOME" || name.starts_with("XDG_") {
            crate::env(name)
        } else {
            std::env::var_os(name)
        };
        v.map(|v| v.to_string_lossy().into_owned())
            .filter(|v| !v.is_empty())
    }
}

/// A made-up computer for tests: files, folders, symlinks and devices held in memory, the PATH's programs, commands'
/// answers and environment variables, all given by the test. Nothing of the real computer is read. Every command asked is
/// recorded (`asked`), and each one that is not read-only (`read_only_command`) as a write (`writes`).
#[derive(Default)]
pub struct MadeUp {
    files: BTreeMap<PathBuf, String>,
    dirs: BTreeSet<PathBuf>,
    links: BTreeMap<PathBuf, PathBuf>,
    owners: BTreeMap<PathBuf, Owner>,
    openable: BTreeSet<PathBuf>,
    programs: BTreeMap<String, PathBuf>,
    answers: BTreeMap<String, String>,
    vars: BTreeMap<String, String>,
    asked: RefCell<Vec<String>>,
    writes: RefCell<Vec<String>>,
}

/// `path` with `.` and `..` worked out, without reading anything.
fn tidy(path: &Path) -> PathBuf {
    let mut out = PathBuf::new();
    for c in path.components() {
        match c {
            Component::ParentDir => {
                out.pop();
            }
            Component::CurDir => {}
            c => out.push(c),
        }
    }
    out
}

impl MadeUp {
    pub fn new() -> MadeUp {
        MadeUp::default()
    }

    fn parents(&mut self, p: &Path) {
        let mut d = p.parent();
        while let Some(x) = d {
            if x.as_os_str().is_empty() {
                break;
            }
            self.dirs.insert(x.to_path_buf());
            d = x.parent();
        }
    }

    /// A file with `text`.
    pub fn file(&mut self, path: &str, text: &str) -> &mut MadeUp {
        let p = PathBuf::from(path);
        self.parents(&p);
        self.files.insert(p, text.to_string());
        self
    }

    /// A folder (empty unless something is put in it).
    pub fn dir(&mut self, path: &str) -> &mut MadeUp {
        let p = PathBuf::from(path);
        self.parents(&p);
        self.dirs.insert(p);
        self
    }

    /// A symlink at `path` to `to` (relative to the link's folder unless absolute).
    pub fn link(&mut self, path: &str, to: &str) -> &mut MadeUp {
        let p = PathBuf::from(path);
        self.parents(&p);
        self.links.insert(p, PathBuf::from(to));
        self
    }

    /// A device node owned by `uid`:`gid` with `mode`, which this user may open or not.
    pub fn device(&mut self, path: &str, uid: u32, gid: u32, mode: u32, mine: bool) -> &mut MadeUp {
        let p = PathBuf::from(path);
        self.file(path, "");
        self.owners.insert(p.clone(), Owner { uid, gid, mode });
        if mine {
            self.openable.insert(p);
        } else {
            self.openable.remove(&p);
        }
        self
    }

    /// A program on the PATH.
    pub fn program(&mut self, name: &str) -> &mut MadeUp {
        let p = PathBuf::from("/usr/bin").join(name);
        self.programs.insert(name.into(), p.clone());
        self.file(p.to_str().unwrap_or_default(), "");
        self
    }

    /// What `program args` prints.
    pub fn answer(&mut self, program: &str, args: &[&str], text: &str) -> &mut MadeUp {
        self.answers
            .insert(format!("{program} {}", args.join(" ")), text.into());
        self
    }

    pub fn set_var(&mut self, name: &str, value: &str) -> &mut MadeUp {
        self.vars.insert(name.into(), value.into());
        self
    }

    pub fn unset_var(&mut self, name: &str) -> &mut MadeUp {
        self.vars.remove(name);
        self
    }

    /// Takes a file, folder or link away.
    pub fn remove(&mut self, path: &str) -> &mut MadeUp {
        let p = PathBuf::from(path);
        self.files.retain(|k, _| !k.starts_with(&p));
        self.dirs.retain(|k| !k.starts_with(&p));
        self.links.retain(|k, _| !k.starts_with(&p));
        self.programs.retain(|_, v| !v.starts_with(&p));
        self
    }

    /// Takes a program off the PATH.
    pub fn no_program(&mut self, name: &str) -> &mut MadeUp {
        self.programs.remove(name);
        self
    }

    /// Every command asked, as "program args".
    pub fn asked(&self) -> Vec<String> {
        self.asked.borrow().clone()
    }

    /// Every command asked that would not only read.
    pub fn writes(&self) -> Vec<String> {
        self.writes.borrow().clone()
    }

    /// `path` with its folders' symlinks followed (the last part too when `last`), at most 40 links.
    fn follow(&self, path: &Path, last: bool) -> Option<PathBuf> {
        let mut todo: Vec<PathBuf> = vec![tidy(path)];
        let mut hops = 0;
        'again: while let Some(p) = todo.pop() {
            let parts: Vec<Component> = p.components().collect();
            let mut at = PathBuf::new();
            for (i, c) in parts.iter().enumerate() {
                at.push(c);
                let is_last = i + 1 == parts.len();
                if is_last && !last {
                    break;
                }
                if let Some(to) = self.links.get(&at) {
                    hops += 1;
                    if hops > 40 {
                        return None;
                    }
                    let base = at.parent().unwrap_or(Path::new("/")).to_path_buf();
                    let mut next = tidy(&base.join(to));
                    for rest in &parts[i + 1..] {
                        next.push(rest);
                    }
                    todo.push(next);
                    continue 'again;
                }
            }
            return Some(at);
        }
        None
    }

    fn there(&self, p: &Path) -> bool {
        self.files.contains_key(p) || self.dirs.contains(p) || self.links.contains_key(p)
    }
}

impl SystemTree for MadeUp {
    fn read(&self, path: &Path) -> Option<String> {
        let p = self.follow(path, true)?;
        self.files.get(&p).cloned()
    }

    fn list(&self, dir: &Path) -> Option<Vec<String>> {
        let d = self.follow(dir, true)?;
        if !self.dirs.contains(&d) {
            return None;
        }
        let mut names: BTreeSet<String> = BTreeSet::new();
        let keys = self
            .files
            .keys()
            .chain(self.dirs.iter())
            .chain(self.links.keys());
        for k in keys {
            if k.parent() == Some(d.as_path()) {
                if let Some(n) = k.file_name() {
                    names.insert(n.to_string_lossy().into_owned());
                }
            }
        }
        Some(names.into_iter().collect())
    }

    fn read_link(&self, path: &Path) -> Option<PathBuf> {
        let p = self.follow(path, false)?;
        self.links.get(&p).cloned()
    }

    fn resolve(&self, path: &Path) -> Option<PathBuf> {
        let p = self.follow(path, true)?;
        self.there(&p).then_some(p)
    }

    fn exists(&self, path: &Path) -> bool {
        self.follow(path, false).is_some_and(|p| self.there(&p))
    }

    fn owner(&self, path: &Path) -> Option<Owner> {
        let p = self.follow(path, true)?;
        self.owners.get(&p).copied().or_else(|| {
            self.there(&p).then_some(Owner {
                uid: 0,
                gid: 0,
                mode: 0o644,
            })
        })
    }

    fn may_open(&self, path: &Path) -> bool {
        self.follow(path, true)
            .is_some_and(|p| self.openable.contains(&p))
    }

    fn which(&self, program: &str) -> Option<PathBuf> {
        self.programs.get(program).cloned()
    }

    fn output(&self, program: &str, args: &[&str]) -> Option<String> {
        let key = format!("{program} {}", args.join(" "));
        self.asked.borrow_mut().push(key.clone());
        if !read_only_command(program, args) {
            self.writes.borrow_mut().push(key);
            return None;
        }
        self.which(program)?;
        self.answers.get(&key).cloned()
    }

    fn var(&self, name: &str) -> Option<String> {
        self.vars.get(name).cloned()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_reading_commands_are_on_the_list() {
        assert!(read_only_command("bluetoothctl", &["devices", "Paired"]));
        assert!(read_only_command(
            "bluetoothctl",
            &["info", "00:11:22:33:44:55"]
        ));
        assert!(!read_only_command(
            "bluetoothctl",
            &["pair", "00:11:22:33:44:55"]
        ));
        assert!(!read_only_command("bluetoothctl", &["info", "; reboot"]));
        assert!(!read_only_command("bluetoothctl", &["power", "on"]));
        assert!(read_only_command("rfcomm", &[]));
        assert!(!read_only_command(
            "rfcomm",
            &["bind", "0", "00:11:22:33:44:55"]
        ));
        assert!(read_only_command("pacman", &["-Q", "icarus-git"]));
        assert!(!read_only_command("pacman", &["-S", "icarus-git"]));
        assert!(!read_only_command("pacman", &["-Q", "--sync"]));
        assert!(!read_only_command(
            "sudo",
            &["usermod", "-aG", "uucp", "me"]
        ));
        assert!(!read_only_command("usermod", &[]));
    }

    #[test]
    fn a_made_up_computer_reads_follows_links_and_lists() {
        let mut t = MadeUp::new();
        t.file("/sys/bus/usb/devices/1-2/idVendor", "0403\n")
            .link("/sys/class/tty/ttyUSB0", "../../devices/usb1/1-2/ttyUSB0")
            .dir("/sys/devices/usb1/1-2/ttyUSB0")
            .link("/proc/self", "4242")
            .file("/proc/4242/comm", "icarus\n")
            .device("/dev/ttyUSB0", 0, 14, 0o660, false);
        assert_eq!(
            t.read(Path::new("/sys/bus/usb/devices/1-2/idVendor"))
                .as_deref(),
            Some("0403\n")
        );
        assert_eq!(
            t.list(Path::new("/sys/bus/usb/devices")),
            Some(vec!["1-2".to_string()])
        );
        assert_eq!(
            t.resolve(Path::new("/sys/class/tty/ttyUSB0")),
            Some(PathBuf::from("/sys/devices/usb1/1-2/ttyUSB0"))
        );
        assert_eq!(
            t.read(Path::new("/proc/self/comm")).as_deref(),
            Some("icarus\n")
        );
        assert_eq!(t.owner(Path::new("/dev/ttyUSB0")).unwrap().gid, 14);
        assert!(!t.may_open(Path::new("/dev/ttyUSB0")));
        assert!(t.read(Path::new("/etc/passwd")).is_none(), "nothing real");
        assert!(t.output("sudo", &["true"]).is_none());
        assert_eq!(t.writes(), vec!["sudo true".to_string()]);
    }
}
