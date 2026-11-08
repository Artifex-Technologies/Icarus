//! Learning from another tool: while it works on my own vehicle, Icarus watches the bus (`bus::Analyzer`), the person marks
//! what they are about to do in it ("seat belt chime off"), and what the modules accepted becomes `[[test]]` and
//! `[[setting]]` entries named by those marks. A session is saved as a Markdown file in the vehicle's Logs folder: the
//! marks, what was learned, what was seen but not learned, and every diagnostic message in words.
//!
//! What Icarus cannot work out by itself (what an identifier is, what a routine did, why a write was refused) can go to
//! Sarina, when she is installed: she reads the session file and answers, read-only (`sarina_question`). The file has no
//! VIN in it.

use crate::bus::{self, Analyzer};
use crate::settings;
use std::path::{Path, PathBuf};

/// The session as Markdown. `source` says what was watched ("the adapter sim:off", "CAN interface can0"), `vehicle` what
/// it was (a make or a name; never the VIN), `modules` the module addresses Icarus knows for it (their names help whoever
/// reads the session, Sarina included).
pub fn session_text(
    a: &Analyzer,
    source: &str,
    vehicle: &str,
    modules: &[crate::vehicles::ModuleAddr],
) -> String {
    let mut t = format!("# Icarus learning session, {}\n\n", crate::report::now());
    t += &format!(
        "Watched {source} on {vehicle}: {} frames, {} IDs. Icarus only listened; another tool did the work.\n\n",
        a.frames,
        a.ids.len()
    );
    let seen: Vec<&crate::vehicles::ModuleAddr> = modules
        .iter()
        .filter(|m| a.ids.contains_key(&(m.tx as u32)) || a.ids.contains_key(&(m.rx() as u32)))
        .collect();
    if !seen.is_empty() {
        t += "## Modules on the bus (as Icarus's profile names them)\n\n";
        for m in seen {
            t += &format!(
                "- {:03X} (answers {:03X}): {}{}\n",
                m.tx,
                m.rx(),
                m.name,
                if m.checked {
                    ""
                } else {
                    " (address from public sources, not yet checked on this vehicle)"
                }
            );
        }
        t += "\n";
    }
    t += "## Marks (what I said I was about to do)\n\n";
    if a.marks.iter().all(|(_, l)| l.is_empty()) {
        t += "None.\n";
    }
    for (at, label) in a.marks.iter().filter(|(_, l)| !l.is_empty()) {
        t += &format!("- {at:.1} s: {label}\n");
    }
    t += "\n## Settings learned\n\n";
    if a.settings.is_empty() {
        t += "None.\n";
    }
    for s in &a.settings {
        t += &format!("```toml\n{}```\n\n", settings::to_toml(s));
    }
    t += "\n## Output tests learned\n\n";
    if a.learned.is_empty() {
        t += "None.\n";
    }
    for x in &a.learned {
        t += &format!("```toml\n{}```\n\n", bus::to_toml(x));
    }
    t += "\n## Seen but not learned\n\n";
    if a.notes.is_empty() {
        t += "Nothing.\n";
    }
    for n in &a.notes {
        t += &format!("- {n}\n");
    }
    t += &format!(
        "\n## What the tool and the modules said ({} messages{})\n\n```\n",
        a.logged,
        if a.logged as usize > a.log.len() {
            format!(", the last {} here", a.log.len())
        } else {
            String::new()
        }
    );
    for l in &a.log {
        t += l;
        t += "\n";
    }
    t += "```\n";
    t
}

/// Saves a session in `dir` as `learning-YYYY-MM-DD-HHMM.md` (a number added if that name is taken).
pub fn save(dir: &Path, text: &str) -> Result<PathBuf, String> {
    std::fs::create_dir_all(dir).map_err(|x| format!("{}: {x}", dir.display()))?;
    let stamp: String = crate::report::now()
        .chars()
        .filter(|c| c.is_ascii_digit() || *c == '-' || *c == ' ')
        .collect();
    let stamp = stamp.replace(' ', "-");
    let mut p = dir.join(format!("learning-{stamp}.md"));
    let mut n = 2;
    while p.exists() {
        p = dir.join(format!("learning-{stamp}-{n}.md"));
        n += 1;
    }
    std::fs::write(&p, text).map_err(|x| format!("{}: {x}", p.display()))?;
    Ok(p)
}

/// The learning sessions in `dir`, newest first.
pub fn sessions(dir: &Path) -> Vec<PathBuf> {
    let mut v: Vec<PathBuf> = std::fs::read_dir(dir)
        .into_iter()
        .flatten()
        .flatten()
        .map(|e| e.path())
        .filter(|p| {
            p.file_name()
                .and_then(|n| n.to_str())
                .is_some_and(|n| n.starts_with("learning-") && n.ends_with(".md"))
        })
        .collect();
    v.sort_by_key(|p| std::cmp::Reverse(std::fs::metadata(p).and_then(|m| m.modified()).ok()));
    v
}

/// The program Sarina is asked through (Labyrinth's), when it is installed.
pub fn sarina() -> Option<PathBuf> {
    std::env::var_os("PATH").and_then(|path| {
        std::env::split_paths(&path)
            .map(|d| d.join("labyrinth-ask-sarina"))
            .find(|p| p.is_file())
    })
}

/// What Sarina is asked about a saved session (she runs in its folder, read-only).
pub fn sarina_question(file: &Path) -> String {
    let name = file
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or("the session file");
    format!(
        "Icarus, my vehicle tool, watched another diagnostic tool work on my own vehicle and saved what it saw in {name} \
         (in this folder). Read it. For each setting and output test it learned, say what it most likely is (which module, \
         what each value means) and suggest a short name and names for the values. For everything under \"Seen but not \
         learned\", explain what the tool was doing and whether a later Icarus could learn it safely. If something is \
         unclear, say what I should try in the other tool while Icarus watches again. Do not change any files. Answer \
         briefly, as a list."
    )
}

/// Asks Sarina about a saved session and returns her answer (this waits for her; run it off the window's thread).
pub fn ask_sarina(file: &Path) -> Result<String, String> {
    let prog = sarina().ok_or("Sarina is not installed")?;
    let dir = file.parent().ok_or("the session has no folder")?;
    let out = std::process::Command::new(prog)
        .arg("--cwd")
        .arg(dir)
        .arg(sarina_question(file))
        .stdin(std::process::Stdio::null())
        .output()
        .map_err(|x| format!("Sarina could not be started: {x}"))?;
    let text = String::from_utf8_lossy(&out.stdout).trim().to_string();
    if out.status.success() && !text.is_empty() {
        Ok(text)
    } else {
        let err = String::from_utf8_lossy(&out.stderr).trim().to_string();
        Err(format!(
            "Sarina did not answer{}",
            if err.is_empty() {
                String::new()
            } else {
                format!(": {err}")
            }
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::Frame;

    #[test]
    fn a_session_is_written_down_without_the_vin() {
        let mut a = Analyzer::default();
        a.mark("Seat belt chime off", 0.5);
        for (i, (id, d)) in [
            (0x7C0u32, vec![0x03, 0x22, 0xA0, 0x01]),
            (0x7C8, vec![0x04, 0x62, 0xA0, 0x01, 0x01]),
            (0x7C0, vec![0x04, 0x2E, 0xA0, 0x01, 0x00]),
            (0x7C8, vec![0x03, 0x6E, 0xA0, 0x01]),
            (0x7E0, vec![0x02, 0x27, 0x01]),
        ]
        .into_iter()
        .enumerate()
        {
            a.add(&Frame {
                id,
                ext: false,
                data: d,
                at: 1.0 + i as f64,
            });
        }
        let cluster = crate::vehicles::ModuleAddr {
            tx: 0x7C0,
            rx: None,
            name: "combination meter (cluster)".into(),
            ext: None,
            bus: None,
            checked: false,
        };
        let t = session_text(&a, "the adapter sim:off", "Toyota", &[cluster]);
        assert!(
            t.contains(
                "- 7C0 (answers 7C8): combination meter (cluster) (address from public sources"
            ),
            "{t}"
        );
        assert!(t.contains("- 0.5 s: Seat belt chime off"), "{t}");
        assert!(
            t.contains("name = \"Seat belt chime off\"")
                && t.contains("values = { set = [0x00], was = [0x01] }"),
            "{t}"
        );
        assert!(t.contains("security access (level 01)"), "{t}");
        assert!(t.contains("write data by identifier: A001 to 00"), "{t}");
        assert!(!t.contains(crate::sim::VIN));
        let dir = std::env::temp_dir().join(format!("icarus-learning-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let p1 = save(&dir, &t).unwrap();
        let p2 = save(&dir, &t).unwrap();
        assert_ne!(
            p1, p2,
            "a second session in the same minute gets its own file"
        );
        assert_eq!(sessions(&dir).len(), 2);
        assert!(sarina_question(&p1).contains(p1.file_name().unwrap().to_str().unwrap()));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
