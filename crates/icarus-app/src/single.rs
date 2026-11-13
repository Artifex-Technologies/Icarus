//! One Icarus window at a time. Two windows each wanted the one adapter (a Bluetooth OBD adapter takes one connection), and
//! two of the same made it unclear which to use; a second `icarus-app [ARGS]` now hands its arguments to the open one (over a
//! socket in the runtime folder) and ends, and the open one turns to the page asked for and comes to the front.

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

/// Hands `args` to the Icarus already open; true when one took them (this one should then end).
pub fn hand_over(args: &[String]) -> bool {
    let Some(path) = socket_path() else {
        return false;
    };
    let Ok(mut s) = UnixStream::connect(&path) else {
        return false;
    };
    let _ = s.set_write_timeout(Some(Duration::from_secs(2)));
    s.write_all(encode(args).as_bytes()).is_ok()
}

/// This is the open Icarus: each later start's arguments go to `on_args` (on a thread of its own). A socket left by an Icarus
/// that crashed is taken over (nothing answered on it).
pub fn listen(on_args: impl Fn(Vec<String>) + Send + 'static) {
    let Some(path) = socket_path() else { return };
    let _ = std::fs::remove_file(&path);
    let Ok(listener) = UnixListener::bind(&path) else {
        return;
    };
    std::thread::spawn(move || {
        for conn in listener.incoming().flatten() {
            let _ = conn.set_read_timeout(Some(Duration::from_secs(2)));
            let mut line = String::new();
            if BufReader::new(conn).read_line(&mut line).is_ok() {
                on_args(decode(&line));
            }
        }
    });
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
