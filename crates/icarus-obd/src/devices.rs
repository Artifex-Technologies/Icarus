//! Tuners and vehicle interfaces Icarus knows by their USB IDs, and which of them are plugged in now (`icarus devices`).
//!
//! The IDs come from the makers' own Windows and macOS driver files, read as data (docs/INTERFACES.md): what a device is on
//! the wire and how Linux reaches it, never how the maker's software works. Recognising a device and opening it is the first
//! step; speaking each one's own protocol comes device by device, learned by watching its maker's program in Limbo.
//!
//! Nothing here sends anything to a device: the scan reads `/sys`, and "can be opened" is checked with access(2), not by
//! opening it.

use std::path::{Path, PathBuf};

/// How Linux reaches a device.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Access {
    /// A USB-serial chip Linux drives itself (cp210x, ftdi_sio, cdc_acm): the device is a `/dev/ttyUSB*` or `/dev/ttyACM*` port.
    Serial,
    /// No Linux driver binds: Icarus opens it through usbfs (`/dev/bus/usb/BBB/DDD`), as libusb does, which is how the
    /// makers' own macOS programs reach these devices too. FTDI chips under a maker's own IDs are reached this way (as FTDI's
    /// D2XX library does on Windows), so Linux's serial driver is not forced onto them.
    RawUsb,
}

/// A device (or, with no product ID, any device of that maker) Icarus recognises.
#[derive(Clone, Copy, Debug)]
pub struct Known {
    pub vendor: u16,
    pub product: Option<u16>,
    pub maker: &'static str,
    pub name: &'static str,
    pub access: Access,
    /// What it is for and what Icarus does with it so far.
    pub note: &'static str,
}

const fn dev(
    vendor: u16,
    product: u16,
    maker: &'static str,
    name: &'static str,
    access: Access,
    note: &'static str,
) -> Known {
    Known {
        vendor,
        product: Some(product),
        maker,
        name,
        access,
        note,
    }
}

/// Most specific first: a vendor-only entry comes after that vendor's products.
pub const KNOWN: &[Known] = &[
    dev(
        0x5059,
        0x4040,
        "SCT",
        "X4 (7015) / MB103",
        Access::RawUsb,
        "Ford tuner; its protocol is locked (a challenge-response with SCT's secret, encrypted data): use SCT's own program in a Limbo Windows guest",
    ),
    dev(
        0x5059,
        0x4080,
        "SCT / Derive",
        "device on Derive's WinUSB driver",
        Access::RawUsb,
        "recognised, its protocol not yet learned",
    ),
    Known {
        vendor: 0x5059,
        product: None,
        maker: "SCT / Derive",
        name: "device",
        access: Access::RawUsb,
        note: "a model Icarus has not seen",
    },
    dev(
        0x1a84,
        0x0120,
        "COBB",
        "AccessPORT 2.0",
        Access::RawUsb,
        "recognised, its protocol not yet learned",
    ),
    dev(
        0x1a84,
        0x0121,
        "COBB",
        "AccessPORT 2.0B",
        Access::RawUsb,
        "recognised, its protocol not yet learned",
    ),
    dev(
        0x1a84,
        0x0210,
        "COBB",
        "USB to OBD cable (FTDI)",
        Access::RawUsb,
        "the AccessPORT's cable; recognised",
    ),
    dev(
        0x1a84,
        0x0211,
        "COBB",
        "USB to OBD cable (Atmel)",
        Access::RawUsb,
        "the AccessPORT's cable; recognised",
    ),
    dev(
        0x1a84,
        0x0212,
        "COBB",
        "USB to OBD cable (SS, FTDI)",
        Access::RawUsb,
        "the AccessPORT's cable; recognised",
    ),
    dev(
        0x1a84,
        0x0213,
        "COBB",
        "USB to OBD cable (Atmel)",
        Access::RawUsb,
        "the AccessPORT's cable; recognised",
    ),
    Known {
        vendor: 0x1a84,
        product: None,
        maker: "COBB",
        name: "device",
        access: Access::RawUsb,
        note: "a model Icarus has not seen",
    },
    dev(
        0x0403,
        0xd018,
        "HP Tuners",
        "MPVI (two channels, A and B)",
        Access::RawUsb,
        "an FTDI FT2232 under HP Tuners' ID; recognised",
    ),
    dev(
        0x0403,
        0xd019,
        "HP Tuners",
        "CDV",
        Access::RawUsb,
        "an FTDI chip under HP Tuners' ID; recognised",
    ),
    dev(
        0x0403,
        0xcc4d,
        "Tactrix",
        "OpenPort 2.0",
        Access::Serial,
        "a J2534 cable (Linux's cdc_acm makes it /dev/ttyACM*): icarus obd --adapter openport",
    ),
    dev(
        0x10c4,
        0xea60,
        "Silicon Labs",
        "CP210x USB-serial (VF Tuner's WiFlash, many OBD cables)",
        Access::Serial,
        "a serial port; the WiFlash also answers over Wi-Fi at 192.168.4.1",
    ),
    dev(
        0x0403,
        0x6001,
        "FTDI",
        "FT232R USB-serial (many ELM327 cables)",
        Access::Serial,
        "a serial port: icarus obd --adapter /dev/ttyUSB0",
    ),
    dev(
        0x0403,
        0x6015,
        "FTDI",
        "FT231X USB-serial (OBDLink SX/EX and others)",
        Access::Serial,
        "a serial port: icarus obd --adapter /dev/ttyUSB0",
    ),
];

/// What one plugged-in device is, by its IDs.
pub fn identify(vendor: u16, product: u16) -> Option<&'static Known> {
    KNOWN
        .iter()
        .find(|k| k.vendor == vendor && k.product == Some(product))
        .or_else(|| {
            KNOWN
                .iter()
                .find(|k| k.vendor == vendor && k.product.is_none())
        })
}

/// A recognised device that is plugged in.
#[derive(Clone, Debug)]
pub struct Found {
    pub known: &'static Known,
    pub vendor: u16,
    pub product: u16,
    /// What the device says it is (its USB strings), when it says.
    pub label: String,
    /// Its USB product string alone (what the udev rule's `ATTRS{product}` matches), empty when it gives none.
    pub product_name: String,
    /// Its usbfs node.
    pub node: PathBuf,
    /// Its serial ports, for a serial device.
    pub ports: Vec<String>,
}

/// Whether Icarus's udev rule (`packaging/70-icarus-tuners.rules`, which the icarus-git package installs) gives the
/// person at the seat this device, by its IDs and its USB product string. FTDI's own 0403:6001 is on too many other
/// devices to give out, and 0403:6015 only on OBDLink's cables (their product string says "OBDLink"). A test holds this
/// to the rule file, line by line.
pub fn udev_rule_covers(vendor: u16, product: u16, product_name: &str) -> bool {
    match (vendor, product) {
        (0x5059, _) | (0x1a84, _) => true,
        (0x10c4, 0xea60 | 0xea63 | 0xea70) => true,
        (0x0403, 0xd018 | 0xd019 | 0xcc4d) => true,
        (0x0403, 0x6015) => product_name.starts_with("OBDLink"),
        _ => false,
    }
}

/// What a serial port's owner group gives: on Arch and Labyrinth OS, the uucp group owns /dev/ttyUSB* and /dev/ttyACM*.
const UUCP: &str = "join the uucp group (sudo usermod -aG uucp $USER), then log in again";

impl Found {
    /// Whether Icarus can use it now, and if not, what to do: the udev rule where it covers the device (and the uucp group
    /// for a serial port when the rule is installed already), the uucp group alone for a serial port it does not cover.
    pub fn status(&self, can_open: impl Fn(&Path) -> bool) -> String {
        let rule = udev_rule_covers(self.vendor, self.product, &self.product_name);
        match self.known.access {
            Access::Serial if self.ports.is_empty() => {
                "no serial port yet (Linux's driver did not take it)".into()
            }
            Access::Serial => {
                let p = Path::new("/dev").join(&self.ports[0]);
                if can_open(&p) {
                    format!("ready: {}", p.display())
                } else if rule {
                    format!(
                        "{} is there but not yours to open: install the icarus-git package (its udev rule), then plug it \
                         in again; with the package installed already, {UUCP}",
                        p.display()
                    )
                } else {
                    format!("{} is there but not yours to open: {UUCP}", p.display())
                }
            }
            Access::RawUsb if can_open(&self.node) => "ready: Icarus can open it (raw USB)".into(),
            Access::RawUsb if rule => {
                "plugged in, but not yours to open: install the icarus-git package (its udev rule), then plug it in again"
                    .into()
            }
            Access::RawUsb => {
                "plugged in, but not yours to open: Icarus's udev rule does not cover it yet (it would need a line for \
                 its IDs)"
                    .into()
            }
        }
    }
}

fn hex(root: &Path, dev: &str, file: &str) -> Option<u16> {
    u16::from_str_radix(
        std::fs::read_to_string(root.join(dev).join(file))
            .ok()?
            .trim(),
        16,
    )
    .ok()
}

fn text(root: &Path, dev: &str, file: &str) -> String {
    std::fs::read_to_string(root.join(dev).join(file))
        .map(|s| s.trim().to_string())
        .unwrap_or_default()
}

/// The recognised devices plugged in now, read from `root` (`/sys/bus/usb/devices`).
pub fn scan(root: &Path) -> Vec<Found> {
    let Ok(entries) = std::fs::read_dir(root) else {
        return vec![];
    };
    let mut names: Vec<String> = entries
        .flatten()
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    let mut out = Vec::new();
    for dev in names.iter().filter(|n| !n.contains(':')) {
        let (Some(vendor), Some(product)) =
            (hex(root, dev, "idVendor"), hex(root, dev, "idProduct"))
        else {
            continue;
        };
        let Some(known) = identify(vendor, product) else {
            continue;
        };
        let product_name = text(root, dev, "product");
        let label = [text(root, dev, "manufacturer"), product_name.clone()]
            .into_iter()
            .filter(|s| !s.is_empty())
            .collect::<Vec<_>>()
            .join(" ");
        let bus: u32 = text(root, dev, "busnum").parse().unwrap_or(0);
        let num: u32 = text(root, dev, "devnum").parse().unwrap_or(0);
        let mut ports = Vec::new();
        for iface in names.iter().filter(|n| n.starts_with(&format!("{dev}:"))) {
            let d = root.join(iface);
            for e in std::fs::read_dir(&d).into_iter().flatten().flatten() {
                let n = e.file_name().to_string_lossy().into_owned();
                if n.starts_with("ttyUSB") {
                    ports.push(n);
                } else if n == "tty" {
                    ports.extend(
                        std::fs::read_dir(e.path())
                            .into_iter()
                            .flatten()
                            .flatten()
                            .map(|t| t.file_name().to_string_lossy().into_owned()),
                    );
                }
            }
        }
        ports.sort();
        out.push(Found {
            known,
            vendor,
            product,
            label,
            product_name,
            node: PathBuf::from(format!("/dev/bus/usb/{bus:03}/{num:03}")),
            ports,
        });
    }
    out
}

/// Whether this user may read and write `p` (access(2): permissions and ACLs, such as udev's uaccess). Opens nothing.
pub fn can_open(p: &Path) -> bool {
    use std::os::unix::ffi::OsStrExt;
    let Ok(c) = std::ffi::CString::new(p.as_os_str().as_bytes()) else {
        return false;
    };
    unsafe { libc::access(c.as_ptr(), libc::R_OK | libc::W_OK) == 0 }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn usb(
        root: &Path,
        dev: &str,
        vendor: &str,
        product: &str,
        label: (&str, &str),
        bus: u32,
        num: u32,
    ) {
        let d = root.join(dev);
        std::fs::create_dir_all(&d).unwrap();
        for (f, v) in [
            ("idVendor", vendor),
            ("idProduct", product),
            ("manufacturer", label.0),
            ("product", label.1),
        ] {
            std::fs::write(d.join(f), format!("{v}\n")).unwrap();
        }
        std::fs::write(d.join("busnum"), format!("{bus}\n")).unwrap();
        std::fs::write(d.join("devnum"), format!("{num}\n")).unwrap();
    }

    /// The table here and the one in docs/INTERFACES.md ("Recognised now") list the same devices: the window's settings show
    /// this one (ICA-79), the doc explains it.
    #[test]
    fn every_known_device_is_in_the_interfaces_doc() {
        let doc = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../docs/INTERFACES.md"
        ))
        .unwrap();
        let table: String = doc
            .split("## Recognised now")
            .nth(1)
            .and_then(|t| t.split("\n## ").next())
            .unwrap()
            .to_string();
        for k in KNOWN {
            let vendor = format!("{:04x}", k.vendor);
            let found = match k.product {
                // written in full ("5059:4040"), or after the vendor's first ID in the same cell ("1a84:0120` / `0121")
                Some(p) => {
                    table.contains(&format!("{vendor}:{p:04x}"))
                        || table.lines().any(|l| {
                            l.contains(&format!("`{vendor}:")) && l.contains(&format!("`{p:04x}`"))
                        })
                }
                None => table.contains(&format!("{vendor}:")),
            };
            assert!(
                found,
                "{} {} ({vendor}:{:04x?}) is not in docs/INTERFACES.md's table",
                k.maker, k.name, k.product
            );
        }
    }

    #[test]
    fn known_devices_are_named_by_their_ids() {
        assert_eq!(identify(0x5059, 0x4040).unwrap().name, "X4 (7015) / MB103");
        assert_eq!(identify(0x1a84, 0x0121).unwrap().name, "AccessPORT 2.0B");
        assert_eq!(
            identify(0x1a84, 0x0999).unwrap().note,
            "a model Icarus has not seen",
            "any COBB device is still COBB's"
        );
        assert_eq!(identify(0x0403, 0xd018).unwrap().maker, "HP Tuners");
        assert!(identify(0x046d, 0xc52b).is_none(), "a mouse is not a tuner");
    }

    #[test]
    fn a_scan_finds_tuners_and_their_ports() {
        let root = std::env::temp_dir().join(format!("icarus-usb-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        usb(&root, "1-9", "5059", "4040", ("SCT LLC", "SCT MB103"), 1, 8);
        usb(&root, "1-4", "046d", "c52b", ("Logitech", "Receiver"), 1, 3);
        usb(
            &root,
            "3-2",
            "10c4",
            "ea60",
            ("Silicon Labs", "CP2102 USB to UART Bridge Controller"),
            3,
            5,
        );
        std::fs::create_dir_all(root.join("3-2:1.0/ttyUSB0")).unwrap();
        let found = scan(&root);
        assert_eq!(found.len(), 2, "{found:?}");
        let sct = &found[0];
        assert_eq!(
            (sct.known.maker, sct.label.as_str()),
            ("SCT", "SCT LLC SCT MB103")
        );
        assert_eq!(sct.node, PathBuf::from("/dev/bus/usb/001/008"));
        assert_eq!(sct.status(|_| true), "ready: Icarus can open it (raw USB)");
        assert!(sct.status(|_| false).contains("udev rule"));
        let cp = &found[1];
        assert_eq!(cp.ports, vec!["ttyUSB0"]);
        assert_eq!(cp.status(|_| true), "ready: /dev/ttyUSB0");
        let _ = std::fs::remove_dir_all(&root);
    }

    /// ICA-203: a serial cable that cannot be opened gets the udev rule's advice only when the rule covers it (an OBDLink
    /// EX, a CP210x), and the uucp group's otherwise (an FT232R cable, a 0403:6015 that is not OBDLink's).
    #[test]
    fn a_cable_the_rule_does_not_cover_gets_the_uucp_group() {
        let root = std::env::temp_dir().join(format!("icarus-usb-uucp-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        for (dev, vendor, product, label, num) in [
            ("1-1", "0403", "6015", ("OBDLink", "OBDLink EX"), 2),
            ("1-2", "0403", "6015", ("FTDI", "FT231X USB UART"), 3),
            ("1-3", "0403", "6001", ("FTDI", "FT232R USB UART"), 4),
            (
                "1-4",
                "10c4",
                "ea60",
                ("Silicon Labs", "CP2102 USB to UART Bridge Controller"),
                5,
            ),
        ] {
            usb(&root, dev, vendor, product, label, 1, num);
            std::fs::create_dir_all(root.join(format!("{dev}:1.0/ttyUSB{}", num - 2))).unwrap();
        }
        let found = scan(&root);
        let said: Vec<(String, String)> = found
            .iter()
            .map(|f| (f.product_name.clone(), f.status(|_| false)))
            .collect();
        assert_eq!(said.len(), 4, "{said:?}");
        let udev = "install the icarus-git package (its udev rule), then plug it in again";
        let uucp = "join the uucp group (sudo usermod -aG uucp $USER), then log in again";
        for (name, status) in &said {
            match name.as_str() {
                "OBDLink EX" | "CP2102 USB to UART Bridge Controller" => {
                    assert!(
                        status.contains(udev) && status.ends_with(uucp),
                        "{name}: {status}"
                    )
                }
                _ => assert!(
                    !status.contains("udev")
                        && status.ends_with(&format!("not yours to open: {uucp}")),
                    "{name}: {status}"
                ),
            }
        }
        let _ = std::fs::remove_dir_all(&root);
    }

    /// The rule file's lines, each as (vendor, products: empty for any, product string prefix: empty for any).
    fn rule_lines() -> Vec<(String, Vec<String>, String)> {
        let rules = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../packaging/70-icarus-tuners.rules"
        ))
        .unwrap();
        let value = |line: &str, key: &str| -> Option<String> {
            line.split(',')
                .map(str::trim)
                .find_map(|m| m.strip_prefix(key)?.strip_prefix("==\""))
                .map(|v| v.trim_end_matches('"').to_string())
        };
        rules
            .lines()
            .map(str::trim)
            .filter(|l| !l.is_empty() && !l.starts_with('#'))
            .map(|l| {
                let attr = |k: &str| {
                    value(l, &format!("ATTRS{{{k}}}")).or_else(|| value(l, &format!("ATTR{{{k}}}")))
                };
                (
                    attr("idVendor").unwrap_or_default(),
                    attr("idProduct")
                        .map(|p| p.split('|').map(str::to_string).collect())
                        .unwrap_or_default(),
                    attr("product")
                        .unwrap_or_default()
                        .trim_end_matches('*')
                        .to_string(),
                )
            })
            .collect()
    }

    /// ICA-203: every known device the udev advice is given for has a line in the rule that matches it, and the ones it is
    /// not given for have none: what `status` says and what the package installs agree.
    #[test]
    fn the_udev_advice_follows_the_rule_file() {
        let lines = rule_lines();
        assert!(lines.len() >= 6, "{lines:?}");
        let matches = |vendor: u16, product: u16, name: &str| {
            lines.iter().any(|(v, ps, prefix)| {
                *v == format!("{vendor:04x}")
                    && (ps.is_empty() || ps.contains(&format!("{product:04x}")))
                    && name.starts_with(prefix.as_str())
            })
        };
        for k in KNOWN {
            // a vendor-only entry: any product of that maker
            let product = k.product.unwrap_or(0x0001);
            for name in ["OBDLink EX", "FT231X USB UART", ""] {
                assert_eq!(
                    udev_rule_covers(k.vendor, product, name),
                    matches(k.vendor, product, name),
                    "{} {} ({:04x}:{product:04x}, {name:?}): status and the rule file disagree",
                    k.maker,
                    k.name,
                    k.vendor
                );
            }
        }
        assert!(!matches(0x0403, 0x6001, "FT232R USB UART"));
        assert!(
            matches(0x0403, 0x6015, "OBDLink EX") && !matches(0x0403, 0x6015, "FT231X USB UART")
        );
    }
}
