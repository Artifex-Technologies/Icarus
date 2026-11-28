//! ICA-216: the doctor on made-up computers only (`systree::MadeUp`): a healthy one passes every check, each broken one is
//! named with what to do, and no check asks anything but a read. Every address, VIN and name here is made up.

use super::*;
use crate::systree::{read_only_command, MadeUp};

const MAC: &str = "00:04:3E:12:34:56";
const VIN: &str = "1FTFW1E50JFA00001";

/// A computer where everything Icarus needs is in place: an OBDLink EX on ttyUSB0 (the uucp group's, and mine), the
/// package's udev rule, Bluetooth with a paired MX+, notices and tones, the package of this build, a named vehicle on the
/// Read-Only list.
fn healthy() -> MadeUp {
    let mut t = MadeUp::new();
    t.link("/proc/self", "4000")
        .file(
            "/proc/4000/status",
            "Name:\ticarus\nUid:\t1000\t1000\t1000\t1000\nGid:\t1000\t1000\t1000\t1000\nGroups:\t14 998 1000 \n",
        )
        .file("/proc/4000/comm", "icarus\n")
        .link("/proc/4000/exe", "/usr/bin/icarus")
        .file(
            "/etc/group",
            "root:x:0:\nuucp:x:14:sam\nwheel:x:998:sam\nsam:x:1000:\n",
        )
        .file(
            "/etc/passwd",
            "root:x:0:0::/root:/bin/bash\nsam:x:1000:1000::/home/sam:/bin/bash\n",
        );
    // the OBDLink EX on USB, and its port
    for (f, v) in [
        ("idVendor", "0403"),
        ("idProduct", "6015"),
        ("manufacturer", "OBDLink"),
        ("product", "OBDLink EX"),
        ("busnum", "1"),
        ("devnum", "5"),
    ] {
        t.file(&format!("/sys/bus/usb/devices/1-2/{f}"), &format!("{v}\n"));
    }
    t.dir("/sys/bus/usb/devices/1-2:1.0/ttyUSB0")
        .device("/dev/ttyUSB0", 0, 14, 0o660, true)
        .device("/dev/bus/usb/001/005", 0, 0, 0o664, true)
        .dir("/dev/serial")
        .file(RULE_LIB, RULE);
    // Bluetooth
    t.dir("/sys/class/bluetooth/hci0")
        .file("/sys/class/rfkill/rfkill0/type", "bluetooth\n")
        .file("/sys/class/rfkill/rfkill0/soft", "0\n")
        .file("/sys/class/rfkill/rfkill0/hard", "0\n")
        .file("/proc/612/comm", "bluetoothd\n")
        .program("bluetoothctl")
        .answer(
            "bluetoothctl",
            &["show"],
            "Controller 00:1A:7D:00:00:01 (public)\n\tName: laptop\n\tPowered: yes\n",
        )
        .answer(
            "bluetoothctl",
            &["devices", "Paired"],
            &format!("Device {MAC} OBDLink MX+ 29999\nDevice 11:22:33:44:55:66 Headphones\n"),
        )
        .answer(
            "bluetoothctl",
            &["info", MAC],
            &format!(
                "Device {MAC} (public)\n\tName: OBDLink MX+ 29999\n\tPaired: yes\n\tTrusted: yes\n\tConnected: no\n"
            ),
        );
    // notices and tones
    t.program("notify-send").program("pw-play");
    for h in crate::notify::Happened::ALL {
        t.file(&format!("{}/{}.oga", crate::notify::SOUNDS, h.sound()), "");
    }
    // the package, this build
    t.program("pacman").program("icarus").answer(
        "pacman",
        &["-Q", "icarus-git"],
        &format!("icarus-git {BUILD_VERSION}.r321.gacbab44-1\n"),
    );
    // my files
    t.set_var("HOME", "/home/sam")
        .set_var("LANG", "en_US.UTF-8")
        .file(
            "/home/sam/.config/icarus/window.toml",
            "units = \"imperial\"\n",
        )
        .file(
            "/home/sam/.config/icarus/read-only.toml",
            &format!("vins = [\"{VIN}\"]\n"),
        )
        .file(
            "/home/sam/.config/icarus/garage.toml",
            &format!("\"{VIN}\" = \"2018 Ford F-150\"\n"),
        )
        .file(
            "/home/sam/.config/icarus/details.toml",
            &format!("[\"{VIN}\"]\nnickname = \"Old Blue\"\n"),
        )
        .dir("/home/sam/Documents/Icarus/Vehicles/Old Blue/Logs")
        .dir("/home/sam/.local/state/icarus");
    t
}

#[test]
fn a_healthy_computer_passes_every_check() {
    let t = healthy();
    let r = run(&t);
    let ids: Vec<&str> = r.checks.iter().map(|c| c.id).collect();
    assert_eq!(
        ids,
        [
            "adapters",
            "group",
            "udev",
            "bluetooth",
            "paired",
            "rfcomm",
            "holders",
            "notices",
            "version",
            "units",
            "readonly",
            "files"
        ]
    );
    for c in &r.checks {
        assert_eq!(c.verdict, Verdict::Ok, "{c:?}");
        assert!(c.fix.is_none(), "{c:?}");
    }
    let text = r.text();
    assert!(
        text.ends_with("Everything Icarus needs here is in place.\n"),
        "{text}"
    );
    assert!(
        text.contains("(0403:6015, says \"OBDLink OBDLink EX\"): ready: /dev/ttyUSB0"),
        "{text}"
    );
    assert!(text.contains("sam is in the uucp group"), "{text}");
    assert!(
        text.contains("OBDLink MX+ ***** (00:04:3E:**:**:**): paired, trusted"),
        "{text}"
    );
    assert!(!text.contains("29999"), "the MX+'s serial digits: {text}");
    assert!(
        text.contains("the window shows Imperial (mph, °F, psi)"),
        "{text}"
    );
    assert!(text.contains("2018 Ford F-150 - Old Blue"), "{text}");
    assert!(
        text.contains("garage: ~/Documents/Icarus/Vehicles, 1 vehicle folder(s)"),
        "{text}"
    );
    // never an address, a VIN or a serial number in full
    assert!(!text.contains(MAC) && !text.contains("12:34:56"), "{text}");
    assert!(
        !text.contains("11:22:33:44:55:66") && !text.contains("00:00:01"),
        "{text}"
    );
    assert!(!text.contains(VIN) && !text.contains("JFA00001"), "{text}");
    assert!(t.writes().is_empty(), "{:?}", t.writes());
}

/// One way the computer can be broken: what to change in the healthy one, the check that must name it, its verdict, and
/// words its "what to do" must have.
struct Broken {
    what: &'static str,
    change: fn(&mut MadeUp),
    id: &'static str,
    verdict: Verdict,
    fix: &'static str,
}

fn broken() -> Vec<Broken> {
    vec![
        Broken {
            what: "the cable's port is not mine to open",
            change: |t| {
                t.device("/dev/ttyUSB0", 0, 14, 0o660, false);
            },
            id: "adapters",
            verdict: Verdict::Fail,
            fix: "install the icarus-git package (its udev rule), then plug it in again; with the package installed \
                  already, join the uucp group",
        },
        Broken {
            what: "a serial port no device of Icarus's list is on, not mine",
            change: |t| {
                t.device("/dev/ttyACM0", 0, 14, 0o660, false);
            },
            id: "adapters",
            verdict: Verdict::Fail,
            fix: "sudo usermod -aG uucp $USER",
        },
        Broken {
            what: "not in the uucp group, the port closed to me",
            change: |t| {
                t.file(
                    "/proc/4000/status",
                    "Uid:\t1000\t1000\t1000\t1000\nGid:\t1000\t1000\t1000\t1000\nGroups:\t998 1000\n",
                )
                .file("/etc/group", "root:x:0:\nuucp:x:14:\nwheel:x:998:sam\nsam:x:1000:\n")
                .device("/dev/ttyUSB0", 0, 14, 0o660, false);
            },
            id: "group",
            verdict: Verdict::Fail,
            fix: "sudo usermod -aG uucp $USER, then log in again",
        },
        Broken {
            what: "added to uucp, not yet logged in again",
            change: |t| {
                t.file(
                    "/proc/4000/status",
                    "Uid:\t1000\t1000\t1000\t1000\nGid:\t1000\t1000\t1000\t1000\nGroups:\t998 1000\n",
                );
            },
            id: "group",
            verdict: Verdict::Warn,
            fix: "log out and in again",
        },
        Broken {
            what: "a Debian-style computer: dialout owns the ports",
            change: |t| {
                t.file(
                    "/proc/4000/status",
                    "Uid:\t1000\t1000\t1000\t1000\nGid:\t1000\t1000\t1000\t1000\nGroups:\t1000\n",
                )
                .file("/etc/group", "root:x:0:\ndialout:x:20:\nsam:x:1000:\n")
                .device("/dev/ttyUSB0", 0, 20, 0o660, true);
            },
            id: "group",
            verdict: Verdict::Warn,
            fix: "sudo usermod -aG dialout $USER, then log in again",
        },
        Broken {
            what: "no udev rule",
            change: |t| {
                t.remove(RULE_LIB);
            },
            id: "udev",
            verdict: Verdict::Warn,
            fix: "install the icarus-git package",
        },
        Broken {
            what: "an old rule in /etc",
            change: |t| {
                t.file(
                    RULE_ETC,
                    "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"5059\", TAG+=\"uaccess\"\n",
                );
            },
            id: "udev",
            verdict: Verdict::Warn,
            fix: "bash scripts/install-local.sh --udev",
        },
        Broken {
            what: "bluetoothd not running",
            change: |t| {
                t.remove("/proc/612");
            },
            id: "bluetooth",
            verdict: Verdict::Fail,
            fix: "sudo systemctl enable --now bluetooth",
        },
        Broken {
            what: "Bluetooth blocked",
            change: |t| {
                t.file("/sys/class/rfkill/rfkill0/soft", "1\n");
            },
            id: "bluetooth",
            verdict: Verdict::Fail,
            fix: "rfkill unblock bluetooth",
        },
        Broken {
            what: "the controller off",
            change: |t| {
                t.answer(
                    "bluetoothctl",
                    &["show"],
                    "Controller 00:1A:7D:00:00:01 (public)\n\tPowered: no\n",
                );
            },
            id: "bluetooth",
            verdict: Verdict::Warn,
            fix: "switch Bluetooth on",
        },
        Broken {
            what: "no bluetoothctl",
            change: |t| {
                t.no_program("bluetoothctl");
            },
            id: "bluetooth",
            verdict: Verdict::Warn,
            fix: "sudo pacman -S bluez-utils",
        },
        Broken {
            what: "no OBD adapter paired",
            change: |t| {
                t.answer(
                    "bluetoothctl",
                    &["devices", "Paired"],
                    "Device 11:22:33:44:55:66 Headphones\n",
                );
            },
            id: "paired",
            verdict: Verdict::Warn,
            fix: "pair it once",
        },
        Broken {
            what: "an rfcomm port bound but not mine",
            change: |t| {
                t.device("/dev/rfcomm0", 0, 14, 0o660, false);
            },
            id: "rfcomm",
            verdict: Verdict::Warn,
            fix: "use --adapter bt:ADDRESS instead",
        },
        Broken {
            what: "the window has the cable",
            change: |t| {
                t.file("/proc/4242/comm", "icarus-app\n")
                    .file("/proc/4242/cmdline", "/usr/bin/icarus-app\0")
                    .link("/proc/4242/fd/7", "/dev/ttyUSB0");
            },
            id: "holders",
            verdict: Verdict::Warn,
            fix: "Icarus never closes another program for you",
        },
        Broken {
            what: "no notify-send and no player",
            change: |t| {
                t.no_program("notify-send").no_program("pw-play");
            },
            id: "notices",
            verdict: Verdict::Warn,
            fix: "sudo pacman -S libnotify",
        },
        Broken {
            what: "an older package installed",
            change: |t| {
                t.answer(
                    "pacman",
                    &["-Q", "icarus-git"],
                    "icarus-git 0.9.0.r12.g0000000-1\n",
                );
            },
            id: "version",
            verdict: Verdict::Warn,
            fix: "update it through Agora",
        },
        Broken {
            what: "no HOME",
            change: |t| {
                t.unset_var("HOME");
            },
            id: "files",
            verdict: Verdict::Warn,
            fix: "HOME set",
        },
    ]
}

#[test]
fn each_broken_setup_is_named_with_what_to_do() {
    for b in broken() {
        let mut t = healthy();
        (b.change)(&mut t);
        let r = run(&t);
        let c = r
            .get(b.id)
            .unwrap_or_else(|| panic!("{}: no {} check", b.what, b.id));
        assert_eq!(c.verdict, b.verdict, "{}: {c:?}", b.what);
        let fix = c.fix.as_deref().unwrap_or_default();
        assert!(fix.contains(b.fix), "{}: {fix:?} lacks {:?}", b.what, b.fix);
        let text = r.text();
        assert!(
            text.contains(&format!("[{}] {}: ", b.verdict.word(), c.title)),
            "{}: {text}",
            b.what
        );
        assert!(!text.ends_with("in place.\n"), "{}: {text}", b.what);
        assert!(t.writes().is_empty(), "{}: {:?}", b.what, t.writes());
    }
}

/// The holder is named as the Icarus program it is.
#[test]
fn a_held_port_names_the_icarus_program() {
    let mut t = healthy();
    t.file("/proc/5150/comm", "icarus\n")
        .file(
            "/proc/5150/cmdline",
            "/usr/bin/icarus\0record\0--auto\0--adapter\0/dev/ttyUSB0\0",
        )
        .link("/proc/5150/fd/3", "/dev/ttyUSB0");
    let r = run(&t);
    let c = r.get("holders").unwrap();
    assert!(
        c.lines
            .iter()
            .any(|l| l == "/dev/ttyUSB0: open in icarus record --auto --adapter /dev/ttyUSB0 (process 5150, an Icarus recording)"),
        "{c:?}"
    );
}

/// ICA-216's "nothing it does changes the system": on every made-up computer, healthy and broken, with the adapter's
/// check too, every command the doctor asked is on the read-only list and the made-up computer recorded no write.
#[test]
fn no_check_writes_anything() {
    let mut trees: Vec<MadeUp> = vec![healthy()];
    for b in broken() {
        let mut t = healthy();
        (b.change)(&mut t);
        trees.push(t);
    }
    for t in &trees {
        let _ = run(t);
        let sim = crate::port::Target::parse("sim").unwrap();
        let _ = adapter_check(t, "sim", &sim, &crate::elm::identify_target);
        assert!(t.writes().is_empty(), "{:?}", t.writes());
        for a in t.asked() {
            let mut w = a.split_whitespace();
            let program = w.next().unwrap_or_default();
            let args: Vec<&str> = w.collect();
            assert!(read_only_command(program, &args), "{a}");
        }
    }
    // the healthy one asked bluetoothctl and pacman, and nothing else
    let asked = trees[0].asked();
    assert!(
        asked
            .iter()
            .all(|a| a.starts_with("bluetoothctl ") || a.starts_with("pacman ")),
        "{asked:?}"
    );
    assert!(
        asked.contains(&"bluetoothctl devices Paired".to_string()),
        "{asked:?}"
    );
}

/// The checks read the computer only through the system tree: the doctor's own code (before its tests) names no file,
/// process, environment or libc call of its own.
#[test]
fn the_checks_read_only_through_the_system_tree() {
    let src = include_str!("../doctor.rs");
    let code = src.split("#[cfg(test)]").next().unwrap();
    for banned in [
        "std::fs",
        "std::process",
        "std::env",
        "libc::",
        "File::",
        "Command::",
    ] {
        assert!(!code.contains(banned), "doctor.rs uses {banned}");
    }
}

#[test]
fn the_adapter_check_reads_who_the_adapter_is() {
    let t = healthy();
    let sim = crate::port::Target::parse("sim").unwrap();
    let c = adapter_check(&t, "sim", &sim, &crate::elm::identify_target);
    assert_eq!(c.verdict, Verdict::Ok, "{c:?}");
    assert!(c.lines.contains(&"ID: ELM327 v1.5".to_string()), "{c:?}");
    assert!(
        c.lines
            .iter()
            .any(|l| l.starts_with("voltage at the OBD port: ")),
        "{c:?}"
    );
    // an STN adapter gives its chip and device line
    let a = crate::elm::identify_port(Box::new(crate::sim::SimCar::new().with_stn())).unwrap();
    assert_eq!(a.stn.as_deref(), Some("STN2255 v5.10.3"));
    assert_eq!(a.device.as_deref(), Some(crate::sim::STDI));
    // one that does not answer is named with what to do
    let c = adapter_check(&t, "sim", &sim, &|_| Err("no answer to ATZ in time".into()));
    assert_eq!(c.verdict, Verdict::Fail);
    assert!(c.fix.unwrap().contains("plugged into the vehicle"));
}

/// The adapter is not asked while a recording or another program may have it: the adapter is never touched.
#[test]
fn the_adapter_is_not_asked_while_another_program_has_it() {
    let asked = std::cell::Cell::new(0);
    let id = |_: &crate::port::Target| {
        asked.set(asked.get() + 1);
        Ok(crate::elm::AdapterInfo::default())
    };
    let usb = crate::port::Target::parse("/dev/ttyUSB0").unwrap();
    // a recording with no --adapter (the default adapter, whichever it is)
    let mut t = healthy();
    t.file("/proc/5150/comm", "icarus\n")
        .file("/proc/5150/cmdline", "icarus\0record\0--auto\0");
    let c = adapter_check(&t, "/dev/ttyUSB0", &usb, &id);
    assert_eq!(c.verdict, Verdict::Warn, "{c:?}");
    assert!(c.found.contains("a recording may have it"), "{c:?}");
    // a recording on another adapter is no reason, but the window holding this port is
    let mut t = healthy();
    t.file("/proc/5150/comm", "icarus\n")
        .file(
            "/proc/5150/cmdline",
            &format!("icarus\0record\0--adapter\0bt:{MAC}\0"),
        )
        .file("/proc/4242/comm", "icarus-app\n")
        .file("/proc/4242/cmdline", "icarus-app\0")
        .link("/proc/4242/fd/9", "/dev/ttyUSB0");
    let c = adapter_check(&t, "/dev/ttyUSB0", &usb, &id);
    assert_eq!(c.verdict, Verdict::Warn, "{c:?}");
    assert!(c.found.contains("an Icarus window"), "{c:?}");
    // `icarus record redecode` reads files, not the adapter
    let mut t = healthy();
    t.file("/proc/5150/comm", "icarus\n")
        .file("/proc/5150/cmdline", "icarus\0record\0redecode\0a.csv\0");
    assert_eq!(asked.get(), 0);
    let c = adapter_check(&t, "/dev/ttyUSB0", &usb, &id);
    assert_eq!((c.verdict, asked.get()), (Verdict::Ok, 1), "{c:?}");
}

#[test]
fn addresses_and_vins_are_masked() {
    assert_eq!(masked_name("OBDLink MX+ 29999"), "OBDLink MX+ *****");
    assert_eq!(masked_name("Headphones 2"), "Headphones 2");
    assert_eq!(masked_name("OBDII"), "OBDII");
    assert_eq!(
        scrub(&format!("bt:{MAC} and dev_00_04_3E_12_34_56")),
        "bt:00:04:3E:**:**:** and dev_00_04_3E_**_**_**"
    );
    assert_eq!(scrub(&format!("VIN {VIN}.")), "VIN 1FTFW1E50JF******.");
    // words, versions and paths are left alone
    for s in [
        "STN2255 v5.10.3",
        "/usr/lib/udev/rules.d/70-icarus-tuners.rules",
        "icarus-git 1.0.0.r321.gacbab44-1",
        "OBDLINKMXPLUSABCD",
    ] {
        assert_eq!(scrub(s), s);
    }
}

#[test]
fn obd_adapters_are_told_by_their_names() {
    for n in [
        "OBDLink MX+ 29999",
        "OBDII",
        "vLinker MC-IOS",
        "Veepeak",
        "V-LINK",
    ] {
        assert!(obd_name(n), "{n}");
    }
    for n in ["Headphones", "Keyboard K380", "Pixel 9"] {
        assert!(!obd_name(n), "{n}");
    }
}

#[test]
fn units_follow_the_window_and_the_locale() {
    let mut t = healthy();
    t.remove("/home/sam/.config/icarus/window.toml");
    let c = units(&t);
    assert!(
        c.found
            .contains("as the system has it (en_US.UTF-8: mph, °F and psi)"),
        "{c:?}"
    );
    t.set_var("LANG", "de_DE.UTF-8")
        .file("/home/sam/.config/icarus/window.toml", "units = \"both\"\n");
    assert!(units(&t)
        .found
        .contains("both side by side, km/h, °C and kPa first"));
}

/// An unnamed vehicle on the Read-Only list is said without its VIN.
#[test]
fn the_read_only_list_names_vehicles_never_vins() {
    let mut t = healthy();
    t.remove("/home/sam/.config/icarus/garage.toml")
        .remove("/home/sam/.config/icarus/details.toml")
        .set_var("ICARUS_READ_ONLY", "1");
    let c = read_only(&t).scrubbed();
    assert_eq!(c.found, "1 vehicle is switched to Read-Only");
    assert!(
        c.lines[0].starts_with("a vehicle with no name yet"),
        "{c:?}"
    );
    assert!(c.lines[1].starts_with("ICARUS_READ_ONLY is set"), "{c:?}");
    assert!(!format!("{c:?}").contains("1FTFW1E50JF"), "{c:?}");
}
