//! Learning from another tool: while it works on my own vehicle, Icarus watches the bus (`bus::Analyzer`), the person marks
//! what they are about to do in it ("seat belt chime off"), and what the modules accepted becomes `[[test]]` and
//! `[[setting]]` entries named by those marks. A session is saved as a Markdown file in the vehicle's Logs folder: the
//! marks, what was learned, what was seen but not learned, and every diagnostic message in words.
//!
//! What Icarus cannot work out by itself (what an identifier is, what a routine did, why a write was refused) can go to
//! Sarina, only when Sarina is installed (`sarina`): Sarina reads the session file and answers, read-only
//! (`sarina_question`). The file has neither the VIN nor a module's serial number in it, since it is also handed to
//! `icarus mcp`'s learning_sessions: the vehicle is named by its make or name, the analyzer's log masks both wherever a
//! message carries them (`capture::mask_message`), and a write of either is never learned as a setting.

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
        .filter(|m| a.ids.contains_key(&m.tx) || a.ids.contains_key(&m.rx()))
        .collect();
    if !seen.is_empty() {
        t += "## Modules on the bus (as Icarus's profile names them)\n\n";
        for m in seen {
            let key =
                a.fence.on_id(m.tx).is_some() || crate::fence::Kind::of(&m.kind, &m.name).is_some();
            t += &format!(
                "- {:03X} (answers {:03X}): {}{}{}\n",
                m.tx,
                m.rx(),
                m.name,
                if m.checked {
                    ""
                } else {
                    " (address from public sources, not yet checked on this vehicle)"
                },
                if key {
                    "; a key module: fenced, its messages are not written down"
                } else {
                    ""
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
    t += "\n## Service routines learned\n\n";
    if a.services.is_empty() {
        t += "None.\n";
    }
    for x in &a.services {
        t += &format!("```toml\n{}```\n\n", crate::service::to_toml(x));
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

/// The program Sarina is asked through, only when Sarina is installed. Sarina is optional: Labyrinth OS ships its
/// `labyrinth-ask-sarina` on every system, so that alone does not count; Sarina's own program (`sarina`) must be there too.
/// Nothing about Sarina is offered without it.
pub fn sarina() -> Option<PathBuf> {
    sarina_in(&std::env::var_os("PATH")?)
}

fn sarina_in(path: &std::ffi::OsStr) -> Option<PathBuf> {
    let find = |name: &str| {
        std::env::split_paths(path)
            .map(|d| d.join(name))
            .find(|p| p.is_file())
    };
    find("sarina")?;
    find("labyrinth-ask-sarina")
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
            kind: String::new(),
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

    /// A message as the bus carries it: one frame, or a first frame and its consecutive frames (ISO-TP).
    fn frames(id: u32, msg: &[u8], at: f64) -> Vec<Frame> {
        let frame = |data: Vec<u8>| Frame {
            id,
            ext: false,
            data,
            at,
        };
        if msg.len() <= 7 {
            return vec![frame([&[msg.len() as u8][..], msg].concat())];
        }
        let mut out = vec![frame(
            [
                &[0x10 | (msg.len() >> 8) as u8, msg.len() as u8][..],
                &msg[..6],
            ]
            .concat(),
        )];
        for (i, part) in msg[6..].chunks(7).enumerate() {
            out.push(frame([&[0x20 | ((i + 1) & 0x0F) as u8][..], part].concat()));
        }
        out
    }

    /// ICA-106: another tool reads the VIN (09 02, as most do on connecting) and a module's serial number (22 F18C), and
    /// writes the serial number: the session, which `icarus mcp` hands on, holds none of their bytes, and learns no setting
    /// from the write.
    #[test]
    fn a_session_holds_neither_the_vin_nor_a_serial_number() {
        let vin = crate::sim::VIN.as_bytes();
        let serial = b"SN0123456789"; // made up
        let answer_vin = [&[0x49, 0x02, 0x01][..], vin].concat();
        let answer_serial = [&[0x62, 0xF1, 0x8C][..], serial].concat();
        let write_serial = [&[0x2E, 0xF1, 0x8C][..], serial].concat();
        let convo: Vec<(u32, &[u8])> = vec![
            (0x7DF, &[0x09, 0x02]),
            (0x7E8, &answer_vin),
            (0x7E0, &[0x22, 0xF1, 0x8C]),
            (0x7E8, &answer_serial),
            (0x7E0, &[0x22, 0xF1, 0x8C]),
            (0x7E8, &answer_serial),
            (0x7E0, &write_serial),
            (0x7E8, &[0x6E, 0xF1, 0x8C]),
        ];
        let mut a = Analyzer::default();
        for (i, (id, msg)) in convo.into_iter().enumerate() {
            for f in frames(id, msg, i as f64) {
                a.add(&f);
            }
        }
        assert_eq!(a.logged, 9, "eight messages and a note: {:?}", a.log);
        assert!(a.settings.is_empty(), "{:?}", a.settings);
        let t = session_text(&a, "the adapter sim:off", "Toyota", &[]);
        let hex = |b: &[u8]| -> Vec<String> { b.iter().map(|x| format!("{x:02X}")).collect() };
        for secret in [vin, &serial[..]] {
            for w in hex(secret).windows(3) {
                assert!(!t.contains(&w.join(" ")), "{} in {t}", w.join(" "));
            }
            assert!(!t.contains(std::str::from_utf8(secret).unwrap()), "{t}");
        }
        assert_eq!(t.matches("[VIN masked]").count(), 1, "{t}");
        assert_eq!(t.matches("[serial number masked]").count(), 3, "{t}");
        assert!(
            t.contains(
                "a write of the VIN or of the module's serial number: identity, never learned"
            ),
            "{t}"
        );
        // what is not identity is told as before
        assert!(t.contains("read data by identifier: F18C"), "{t}");
    }

    #[test]
    fn sarina_counts_only_when_she_is_installed() {
        let root = std::env::temp_dir().join(format!("icarus-sarina-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        let (labyrinth, sarina) = (root.join("labyrinth"), root.join("sarina"));
        std::fs::create_dir_all(&labyrinth).unwrap();
        std::fs::create_dir_all(&sarina).unwrap();
        std::fs::write(labyrinth.join("labyrinth-ask-sarina"), "").unwrap();
        let only_labyrinth = std::env::join_paths([&labyrinth]).unwrap();
        assert_eq!(
            sarina_in(&only_labyrinth),
            None,
            "Labyrinth's ask script alone is not Sarina"
        );
        std::fs::write(sarina.join("sarina"), "").unwrap();
        let both = std::env::join_paths([&labyrinth, &sarina]).unwrap();
        assert_eq!(
            sarina_in(&both),
            Some(labyrinth.join("labyrinth-ask-sarina"))
        );
        let _ = std::fs::remove_dir_all(&root);
    }
}
