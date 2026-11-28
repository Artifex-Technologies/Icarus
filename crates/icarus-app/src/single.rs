//! One Icarus window at a time. Two windows each wanted the one adapter (a Bluetooth OBD adapter takes one connection), and
//! two of the same made it unclear which to use; a second `icarus-app [ARGS]` now hands its arguments to the open one (over a
//! socket in the runtime folder) and ends, and the open one turns to the page asked for and comes to the front.
//!
//! The open one answers with a line: it took them, and whether it is read-only now. A start that asks for a read-only
//! session (ICA-192) waits for that answer, and when the open window does not say it is read-only (an Icarus from before
//! the answer says nothing) it says so and ends with an error, never quietly leaving a window that can still change a
//! vehicle.

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::PathBuf;
use std::time::Duration;

fn socket_path() -> Option<PathBuf> {
    Some(PathBuf::from(std::env::var_os("XDG_RUNTIME_DIR")?).join("icarus-app.sock"))
}

/// The arguments as one line (unit separators between them; control characters left out).
pub fn encode(args: &[String]) -> String {
    let clean: Vec<String> = args
        .iter()
        .map(|a| a.chars().filter(|c| !c.is_control()).take(300).collect())
        .collect();
    format!("{}\n", clean.join("\u{1f}"))
}

pub fn decode(line: &str) -> Vec<String> {
    line.trim_end_matches('\n')
        .split('\u{1f}')
        .filter(|a| !a.is_empty())
        .map(str::to_string)
        .collect()
}

/// The open Icarus's answers: it took the arguments, and it is read-only now too.
const TAKEN: &str = "taken";
const TAKEN_READ_ONLY: &str = "taken read-only";

/// How handing a start's arguments over went.
#[derive(Debug, PartialEq, Eq)]
pub enum Handed {
    /// No Icarus is open (or it could not be reached): this one opens the window.
    NoWindow,
    /// The open one took them (and said it is read-only now, when that was asked).
    Taken,
    /// A read-only session was asked for, and the open one took the arguments without saying it is read-only: it may still
    /// change a vehicle.
    NotReadOnly,
}

/// Hands `args` to the Icarus already open; with `read_only` (the start asks for the session floor), the open one must say
/// it holds it.
pub fn hand_over(args: &[String], read_only: bool) -> Handed {
    let Some(path) = socket_path() else {
        return Handed::NoWindow;
    };
    let Ok(s) = UnixStream::connect(&path) else {
        return Handed::NoWindow;
    };
    hand_over_on(s, args, read_only)
}

/// `hand_over` on a connection to the open Icarus.
fn hand_over_on(mut s: UnixStream, args: &[String], read_only: bool) -> Handed {
    let _ = s.set_write_timeout(Some(Duration::from_secs(2)));
    if s.write_all(encode(args).as_bytes()).is_err() {
        return Handed::NoWindow;
    }
    if !read_only {
        return Handed::Taken;
    }
    let _ = s.set_read_timeout(Some(Duration::from_secs(2)));
    let mut answer = String::new();
    let _ = BufReader::new(s).read_line(&mut answer);
    if answer.trim_end() == TAKEN_READ_ONLY {
        Handed::Taken
    } else {
        Handed::NotReadOnly
    }
}

/// This is the open Icarus: each later start's arguments go to `on_args` (on a thread of its own), which says whether this
/// window is read-only now, and the start is answered so. A socket left by an Icarus that crashed is taken over (nothing
/// answered on it).
pub fn listen(on_args: impl Fn(Vec<String>) -> bool + Send + 'static) {
    let Some(path) = socket_path() else { return };
    let _ = std::fs::remove_file(&path);
    let Ok(listener) = UnixListener::bind(&path) else {
        return;
    };
    std::thread::spawn(move || {
        for conn in listener.incoming().flatten() {
            serve(&conn, &on_args);
        }
    });
}

/// One later start: its line to `on_args`, and the answer back.
fn serve(conn: &UnixStream, on_args: &dyn Fn(Vec<String>) -> bool) {
    let _ = conn.set_read_timeout(Some(Duration::from_secs(2)));
    let _ = conn.set_write_timeout(Some(Duration::from_secs(2)));
    let mut line = String::new();
    if BufReader::new(conn).read_line(&mut line).is_ok() {
        let answer = if on_args(decode(&line)) {
            TAKEN_READ_ONLY
        } else {
            TAKEN
        };
        // a start that does not wait for the answer has gone already: nothing to tell
        let mut w = conn;
        let _ = w.write_all(format!("{answer}\n").as_bytes());
    }
}

/// Brings the window to the front: the window asks itself, and on Hyprland (which may not let a window raise itself) the
/// compositor is asked too.
pub fn raise(ctx: &eframe::egui::Context) {
    ctx.send_viewport_cmd(eframe::egui::ViewportCommand::Focus);
    if std::env::var_os("HYPRLAND_INSTANCE_SIGNATURE").is_some() {
        std::thread::spawn(|| {
            // Hyprland's Lua dispatchers (0.57 and later), then the older word form
            for d in [
                "hl.dsp.focus({ window = \"class:icarus\" })",
                "focuswindow class:icarus",
            ] {
                let ok = std::process::Command::new("hyprctl")
                    .args(["dispatch", d])
                    .output()
                    .is_ok_and(|o| String::from_utf8_lossy(&o.stdout).trim() == "ok");
                if ok {
                    break;
                }
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(a: &[&str]) -> Vec<String> {
        a.iter().map(|s| s.to_string()).collect()
    }

    /// The open window, on one end of a socket pair: `serve` with its answer (whether it is read-only now); what it was
    /// handed comes back from the thread.
    fn window(read_only: bool) -> (UnixStream, std::thread::JoinHandle<Vec<String>>) {
        let (start, window) = UnixStream::pair().unwrap();
        let t = std::thread::spawn(move || {
            let got = std::sync::Mutex::new(Vec::new());
            serve(&window, &|a| {
                *got.lock().unwrap() = a;
                read_only
            });
            got.into_inner().unwrap()
        });
        (start, t)
    }

    /// ICA-192: a start that asks for read-only hears from the open window that it holds the floor; a window that took
    /// the line without saying so (one not read-only, or an Icarus from before the answer, which reads the line and says
    /// nothing) is said to be one that may still change a vehicle. A start that does not ask waits for nothing.
    #[test]
    fn a_window_open_must_say_it_is_read_only() {
        let asked = line(&["--read-only", "--page", "live"]);
        let (s, w) = window(true);
        assert_eq!(hand_over_on(s, &asked, true), Handed::Taken);
        assert_eq!(w.join().unwrap(), asked, "the window had the line");
        let (s, w) = window(false);
        assert_eq!(hand_over_on(s, &asked, true), Handed::NotReadOnly);
        w.join().unwrap();
        // an older Icarus: the line read, no answer
        let (s, old) = UnixStream::pair().unwrap();
        let t = std::thread::spawn(move || {
            let mut l = String::new();
            let _ = BufReader::new(&old).read_line(&mut l);
            l
        });
        assert_eq!(hand_over_on(s, &asked, true), Handed::NotReadOnly);
        assert!(t.join().unwrap().starts_with("--read-only"));
        // read-only not asked: taken once written
        let (s, w) = window(false);
        assert_eq!(
            hand_over_on(s, &line(&["--page", "live"]), false),
            Handed::Taken
        );
        w.join().unwrap();
    }

    #[test]
    fn arguments_go_over_as_one_line() {
        let a: Vec<String> = ["--page", "live", "--mark", "pulled\nover"]
            .iter()
            .map(|s| s.to_string())
            .collect();
        let line = super::encode(&a);
        assert_eq!(line.matches('\n').count(), 1);
        assert_eq!(
            super::decode(&line),
            ["--page", "live", "--mark", "pulledover"]
        );
        assert!(super::decode(&super::encode(&[])).is_empty());
    }
}
