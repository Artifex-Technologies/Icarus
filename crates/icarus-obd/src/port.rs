//! The link to an OBD adapter: a USB serial port, a Bluetooth RFCOMM channel (OBDLink MX+, most Bluetooth ELM327s) or a TCP
//! socket (Wi-Fi adapters). Each is just bytes in and out with a timeout; the adapter's language is in `elm`.

use std::io::{self, Read, Write};
use std::net::TcpStream;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::path::{Path, PathBuf};
use std::time::Duration;

pub trait Port {
    fn send(&mut self, data: &[u8]) -> io::Result<()>;
    /// Bytes that arrived, waiting up to `timeout`; 0 when nothing came in time.
    fn recv(&mut self, buf: &mut [u8], timeout: Duration) -> io::Result<usize>;
    /// Whether this port, as it was opened, is a cable all the way (what `Target::wired` asks of a path, asked of the open
    /// port itself, so where a path leads later does not change it). Not unless the port can tell: a socket, a pipe or a
    /// stand-in is not.
    fn wired(&self) -> bool {
        false
    }
    /// The speed this computer's side of a serial line runs at (baud), when this port is a serial line whose speed can be
    /// changed (`open_serial`); None for anything else (a Bluetooth socket, TCP, a stand-in). The adapter is asked to
    /// change its own (`Elm::monitor_ids`) only when the port has a speed and is `wired` too: `/dev/rfcomm0` or a pty
    /// has a speed, which carries nothing.
    fn speed(&self) -> Option<u32> {
        None
    }
    /// Changes this computer's side of the serial line to `baud`, once what was sent has gone out. Only a port with a
    /// `speed` can.
    fn set_speed(&mut self, _baud: u32) -> io::Result<()> {
        Err(io::Error::new(
            io::ErrorKind::Unsupported,
            "this link to the adapter has no serial speed to change",
        ))
    }
}

/// Where to reach the adapter, as written on the command line:
///   `sim` or `sim:off`             Icarus's simulated car, engine running or ignition on with the engine off
///   `sim:listen`                   a second simulated adapter on the same simulated bus, for listening (ICA-80)
///   `sim:j1939`                    a simulated heavy-duty vehicle on J1939, for watching (ICA-94)
///   `sim:drive`                    the simulated car on a short drive: engine off, running, off (ICA-208)
///   `/dev/ttyUSB0` or `/dev/ttyUSB0@115200`   a USB adapter (38400 baud unless given)
///   `bt:AA:BB:CC:DD:EE:FF` or `...#2`         a paired Bluetooth adapter, RFCOMM channel 1 unless given
///   `tcp:192.168.0.10:35000`       a Wi-Fi adapter
///   `can:can0`, or just `can0`/`vcan0`        a Linux CAN interface (SocketCAN: a CANable), spoken to directly
///   `slcan:/dev/ttyACM0[@baud][#bitrate]`     an SLCAN adapter on a serial port, for watching the bus listen-only
///                                             (115200 baud and 500 kbit/s unless given)
///   `openport` or `openport:/dev/ttyACM0`     a Tactrix OpenPort 2.0 J2534 cable (the first one plugged in, or that port)
///   `mongoose:/dev/ttyACM0`        a Drewtech Mongoose Pro J2534 cable on that port (raw CAN at 500 kbit/s, Icarus's ISO-TP)
///   `sim:j2534`                    the simulated car behind a simulated J2534 device
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Target {
    /// The simulated vehicle: engine running (`sim`) or ignition on, engine off (`sim:off`); `sim:listen` is a second adapter
    /// on the same simulated bus, whose watch also shows what the other simulated adapters asked and were answered.
    Sim {
        engine_on: bool,
        listen: bool,
        /// `sim:j1939`: a heavy-duty vehicle whose modules talk J1939 (watched on protocol A).
        j1939: bool,
        /// `sim:drive`: the engine off, then running, then off again (`SimCar::drive`), for `icarus record --auto`.
        drive: bool,
    },
    Serial {
        path: String,
        baud: u32,
    },
    Bluetooth {
        addr: [u8; 6],
        channel: u8,
    },
    Tcp(String),
    /// A Linux CAN interface by name (`can0`): diagnostics over SocketCAN (`canlink`), or watching the bus.
    Can(String),
    /// An SLCAN adapter (Lawicel CANUSB, a CANable with slcan firmware) on a serial port: watching only (`slcan`).
    Slcan {
        path: String,
        baud: u32,
        /// The CAN bit rate, one of `slcan::BITRATES`.
        bitrate: u32,
    },
    /// A J2534 device, reached through Icarus's own driver for it (`passthru::open_link`), not as an ELM327.
    J2534(J2534Device),
}

/// The J2534 devices Icarus has a driver for.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum J2534Device {
    /// `sim:j2534`: the simulated car behind a simulated J2534 device.
    Sim,
    /// `openport` (the first one plugged in) or `openport:/dev/ttyACM0`: a Tactrix OpenPort 2.0 (`openport`).
    OpenPort(Option<String>),
    /// `mongoose:/dev/ttyACM0`: a Drewtech Mongoose Pro on that serial port (`mongoose`).
    Mongoose(String),
}

/// Whether `s` is a CAN interface's usual name: `can` or `vcan` and a number.
fn can_name(s: &str) -> bool {
    let digits = s.strip_prefix("vcan").or_else(|| s.strip_prefix("can"));
    digits.is_some_and(|d| !d.is_empty() && d.len() <= 3 && d.chars().all(|c| c.is_ascii_digit()))
}

/// What the wired check (`Target::wired_on`) reads of the computer: this one (`ThisComputer`), or one a test makes up.
pub trait Computer {
    /// Its sysfs tree: `/sys`.
    fn sys(&self) -> PathBuf;
    /// Where a device path leads with every symlink followed; None when it leads nowhere.
    fn resolve(&self, path: &Path) -> Option<PathBuf>;
}

/// This computer: `/sys` and its own symlinks.
pub struct ThisComputer;

impl Computer for ThisComputer {
    fn sys(&self) -> PathBuf {
        PathBuf::from("/sys")
    }
    fn resolve(&self, path: &Path) -> Option<PathBuf> {
        std::fs::canonicalize(path).ok()
    }
}

/// Whether the sysfs link `link` (under `sys`), followed, leads to a device on a USB cable: through a bus's root hub and a
/// device on that bus (`devices/pci0000:00/0000:00:14.0/usb1/1-2/1-2:1.0`). Never a device under a Bluetooth controller
/// (`.../bluetooth/hci0/hci0:256`): the kernel moves an `rfcomm` port there while it is connected, and the controller is
/// often on USB itself. Never one on USB/IP's virtual host controller (`devices/platform/vhci_hcd.0/usb3/3-1/...`): a
/// cable plugged into another computer, its bytes carried over the network (Wi-Fi). A virtual device (an slcan or vcan
/// interface, an `rfcomm` port not connected) has no device link at all, and anything that does not resolve is not on
/// USB.
fn on_usb(sys: &Path, link: &Path) -> bool {
    let (Ok(root), Ok(dev)) = (
        std::fs::canonicalize(sys),
        std::fs::canonicalize(sys.join(link)),
    ) else {
        return false;
    };
    let Ok(rest) = dev.strip_prefix(&root) else {
        return false;
    };
    let parts: Vec<String> = rest
        .components()
        .map(|c| c.as_os_str().to_string_lossy().into_owned())
        .collect();
    parts.first().is_some_and(|p| p == "devices")
        && !parts
            .iter()
            .any(|p| p == "bluetooth" || p.starts_with("vhci_hcd"))
        && parts.windows(2).any(|w| {
            w[0].strip_prefix("usb").is_some_and(|n| {
                !n.is_empty()
                    && n.bytes().all(|b| b.is_ascii_digit())
                    && w[1].starts_with(&format!("{n}-"))
            })
        })
}

/// Whether the tty `name` (`ttyUSB0`) is on a USB cable: its device in sysfs (`class/tty/NAME/device`) is on a USB bus
/// (`on_usb`). A Bluetooth `rfcomm` port is refused by its name as well as by where it sits.
fn tty_on_usb(sys: &Path, name: &str) -> bool {
    !name.starts_with("rfcomm") && on_usb(sys, &Path::new("class/tty").join(name).join("device"))
}

/// Whether `path` is a serial port on a USB cable: its symlinks followed, a tty in /dev (`/dev/ttyUSB0`, `/dev/ttyACM0`)
/// that `tty_on_usb` puts on USB. A pty (in /dev/pts) is not one.
fn usb_tty(pc: &dyn Computer, path: &str) -> bool {
    let Some(real) = pc.resolve(Path::new(path)) else {
        return false;
    };
    let Some(name) = real.file_name().and_then(|n| n.to_str()) else {
        return false;
    };
    real.parent() == Some(Path::new("/dev")) && tty_on_usb(&pc.sys(), name)
}

/// Whether the character device numbered `major:minor` is a tty on a USB cable: sysfs's `dev/char/MAJ:MIN` leads to its
/// folder, which must be a tty's own (`class/tty/NAME`), and `tty_on_usb` must put that tty on USB. A raw USB device
/// node, or anything else that is not a tty, is not one.
fn usb_char(sys: &Path, major: u32, minor: u32) -> bool {
    let Ok(dir) = std::fs::canonicalize(sys.join(format!("dev/char/{major}:{minor}"))) else {
        return false;
    };
    let Some(name) = dir.file_name().and_then(|n| n.to_str()) else {
        return false;
    };
    std::fs::canonicalize(sys.join("class/tty").join(name)).is_ok_and(|t| t == dir)
        && tty_on_usb(sys, name)
}

/// The link type of every CAN interface (the kernel's ARPHRD_CAN, in `class/net/NAME/type`).
const ARPHRD_CAN: &str = "280";

/// Whether the network interface `ifn` is a CAN adapter on a USB cable (a CANable on gs_usb: its device in sysfs,
/// `class/net/NAME/device`, is on a USB bus). An slcan or vcan interface has no device in sysfs, whatever it is named: a
/// vcan reaches no vehicle by itself, but cannelloni, socketcand or the kernel's CAN gateway (`cangw`) can carry its
/// frames to one over anything, so it is not counted as a cable.
fn usb_can(pc: &dyn Computer, ifn: &str) -> bool {
    if ifn.is_empty() || ifn.contains('/') || ifn == "." || ifn == ".." {
        return false;
    }
    let dir = Path::new("class/net").join(ifn);
    std::fs::read_to_string(pc.sys().join(&dir).join("type")).is_ok_and(|t| t.trim() == ARPHRD_CAN)
        && on_usb(&pc.sys(), &dir.join("device"))
}

impl Target {
    /// A cable all the way: what anything that writes a module's memory needs (a dropped wireless link mid-write can leave
    /// the module half-written). Judged by where Linux puts the device in sysfs, not by what it is called, and whatever
    /// cannot be told counts as not wired:
    /// - a serial port (an ELM327 cable, SLCAN, the OpenPort, the Mongoose), its symlinks followed, must be a tty in /dev
    ///   whose device sysfs puts on a USB bus (`on_usb`): not under a Bluetooth controller, where a connected `rfcomm`
    ///   port sits (an `rfcomm` name is refused as well), and not on USB/IP's virtual host controller; a pty (socat's
    ///   stand-in for a Wi-Fi adapter), a built-in serial port and a symlink to any of them are not;
    /// - a CAN interface must be a CAN adapter sysfs puts on a USB bus (gs_usb): an slcan interface over Bluetooth is not,
    ///   even named `can0`, and neither is a vcan interface, whose frames a bridge can carry anywhere;
    /// - the OpenPort found by its USB IDs (`openport`) is judged by the serial port it was found with;
    /// - Bluetooth and TCP never; the simulation always.
    ///
    /// A path is judged by where it leads now. A connection kept open is judged by its own port instead (`Port::wired`,
    /// `Elm::wired`), so a symlink moved after connecting does not change the answer.
    pub fn wired(&self) -> bool {
        self.wired_on(&ThisComputer)
    }

    /// `wired`, reading `pc` (a computer a test makes up) instead of this one.
    pub fn wired_on(&self, pc: &dyn Computer) -> bool {
        match self {
            Target::Sim { .. } | Target::J2534(J2534Device::Sim) => true,
            Target::Serial { path, .. }
            | Target::Slcan { path, .. }
            | Target::J2534(J2534Device::OpenPort(Some(path)) | J2534Device::Mongoose(path)) => {
                usb_tty(pc, path)
            }
            // the cable `passthru::open_link` would find
            Target::J2534(J2534Device::OpenPort(None)) => {
                crate::openport::find(&pc.sys().join("bus/usb/devices"))
                    .is_ok_and(|p| usb_tty(pc, &p))
            }
            Target::Can(ifn) => usb_can(pc, ifn),
            Target::Bluetooth { .. } | Target::Tcp(_) => false,
        }
    }

    pub fn parse(s: &str) -> Result<Target, String> {
        if s == "sim:j2534" {
            return Ok(Target::J2534(J2534Device::Sim));
        }
        if s == "openport" {
            return Ok(Target::J2534(J2534Device::OpenPort(None)));
        }
        if let Some(path) = s.strip_prefix("mongoose:") {
            if !path.starts_with('/') {
                return Err(
                    "mongoose: needs the cable's serial port, e.g. mongoose:/dev/ttyACM0".into(),
                );
            }
            return Ok(Target::J2534(J2534Device::Mongoose(path.to_string())));
        }
        if let Some(path) = s.strip_prefix("openport:") {
            if !path.starts_with('/') {
                return Err(
                    "openport: needs the cable's serial port, e.g. openport:/dev/ttyACM0 (or just openport)".into(),
                );
            }
            return Ok(Target::J2534(J2534Device::OpenPort(Some(path.to_string()))));
        }
        if matches!(
            s,
            "sim" | "sim:off" | "sim:listen" | "sim:j1939" | "sim:drive"
        ) {
            return Ok(Target::Sim {
                engine_on: s != "sim:off",
                listen: s == "sim:listen",
                j1939: s == "sim:j1939",
                drive: s == "sim:drive",
            });
        }
        if let Some(rest) = s.strip_prefix("slcan:") {
            let (rest, bitrate) = match rest.split_once('#') {
                None => (rest, 500_000),
                Some((r, b)) => {
                    let n = match b.strip_suffix('k') {
                        Some(k) => k.parse::<u32>().ok().and_then(|k| k.checked_mul(1000)),
                        None => b.parse::<u32>().ok(),
                    };
                    match n.filter(|n| crate::slcan::BITRATES.contains(n)) {
                        Some(n) => (r, n),
                        None => {
                            return Err(format!(
                                "slcan: bit rate {b:?} is not one SLCAN sets (10k 20k 50k 100k 125k 250k 500k 800k \
                                 1000k)"
                            ))
                        }
                    }
                }
            };
            let (path, baud) = rest.split_once('@').unwrap_or((rest, "115200"));
            if !path.starts_with('/') {
                return Err("slcan: needs the serial port, e.g. slcan:/dev/ttyACM0".into());
            }
            let baud = baud
                .parse::<u32>()
                .map_err(|_| format!("bad baud rate {baud}"))?;
            return Ok(Target::Slcan {
                path: path.to_string(),
                baud,
                bitrate,
            });
        }
        if let Some(ifn) = s.strip_prefix("can:") {
            // a network interface name: at most 15 characters, no slashes or spaces
            if ifn.is_empty()
                || ifn.len() > 15
                || !ifn
                    .chars()
                    .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_' || c == '.')
            {
                return Err(format!(
                    "can: needs a CAN interface's name, e.g. can:can0, not {ifn:?}"
                ));
            }
            return Ok(Target::Can(ifn.to_string()));
        }
        if can_name(s) {
            return Ok(Target::Can(s.to_string()));
        }
        if let Some(rest) = s.strip_prefix("tcp:") {
            if !rest.contains(':') {
                return Err("tcp: needs host:port, e.g. tcp:192.168.0.10:35000".into());
            }
            return Ok(Target::Tcp(rest.to_string()));
        }
        if let Some(rest) = s.strip_prefix("bt:") {
            let (mac, ch) = rest.split_once('#').unwrap_or((rest, "1"));
            let parts: Vec<&str> = mac.split(':').collect();
            if parts.len() != 6 {
                return Err(format!(
                    "bt: needs a Bluetooth address like AA:BB:CC:DD:EE:FF, not {mac}"
                ));
            }
            let mut addr = [0u8; 6];
            for (i, p) in parts.iter().enumerate() {
                addr[i] = u8::from_str_radix(p, 16)
                    .map_err(|_| format!("bad Bluetooth address {mac}"))?;
            }
            let channel = ch
                .parse::<u8>()
                .ok()
                .filter(|c| (1..=30).contains(c))
                .ok_or("RFCOMM channel must be 1-30")?;
            return Ok(Target::Bluetooth { addr, channel });
        }
        if s.starts_with('/') {
            let (path, baud) = s.split_once('@').unwrap_or((s, "38400"));
            let baud = baud
                .parse::<u32>()
                .map_err(|_| format!("bad baud rate {baud}"))?;
            return Ok(Target::Serial {
                path: path.to_string(),
                baud,
            });
        }
        Err(format!("unknown adapter {s:?}: use sim, /dev/ttyUSB0[@baud], bt:AA:BB:CC:DD:EE:FF[#channel], tcp:host:port, can:can0, slcan:/dev/ttyACM0, openport or mongoose:/dev/ttyACM0"))
    }

    /// The byte stream to an ELM327/STN adapter. A CAN interface is not one: it is reached frame by frame
    /// (`socketcan::open`, `canlink::CanLink`).
    pub fn open(&self) -> io::Result<Box<dyn Port>> {
        Ok(match self {
            Target::Can(ifn) => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!("{ifn} is a CAN interface, not an ELM327 adapter: this needs an OBD adapter (sim, /dev/ttyUSB0, bt:, tcp:)"),
                ))
            }
            // never spoken to as an ELM327: an SLCAN adapter is opened listen-only, by `slcan` alone
            Target::Slcan { path, .. } => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!("{path} is an SLCAN adapter, opened listen-only for watching the bus (icarus bus watch, icarus learn); this needs an OBD adapter or a CAN interface"),
                ))
            }
            // spoken to by its own driver, never as an ELM327
            Target::J2534(_) => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "a J2534 cable is reached through Icarus's own driver for it: the commands that ask the vehicle (icarus obd, scan, module, live...) take it; the window and bus watching need an ELM327 adapter or a CAN interface for now",
                ))
            }
            Target::Sim { j1939: true, .. } => Box::new(crate::sim::SimCar::truck()),
            Target::Sim { drive: true, .. } => Box::new(crate::sim::SimCar::drive()),
            Target::Sim { listen: true, .. } => Box::new(crate::sim::SimCar::listening()),
            Target::Sim { engine_on: true, .. } => Box::new(crate::sim::SimCar::new()),
            Target::Sim { engine_on: false, .. } => Box::new(crate::sim::SimCar::engine_off()),
            Target::Serial { path, baud } => Box::new(open_serial(path, *baud)?),
            Target::Bluetooth { addr, channel } => Box::new(open_rfcomm(*addr, *channel)?),
            Target::Tcp(hostport) => {
                let s = connect_tcp(hostport, TCP_LIMIT)?;
                s.set_nodelay(true)?;
                Box::new(TcpPort(s))
            }
        })
    }
}

/// How long a Wi-Fi adapter has to take the connection (ICA-110): a plain connect to an address nothing answers on held the
/// worker for about two minutes (the kernel's SYN retries).
pub const TCP_LIMIT: Duration = Duration::from_secs(5);

/// Connects to `hostport` (`192.168.0.10:35000`), trying each address it names in turn for up to `limit` each. A failure
/// says what usually causes it: this computer on another network than the adapter's, or another program (a phone app)
/// holding the adapter's one connection.
pub fn connect_tcp(hostport: &str, limit: Duration) -> io::Result<TcpStream> {
    use std::net::ToSocketAddrs;
    let addrs: Vec<std::net::SocketAddr> = hostport
        .to_socket_addrs()
        .map_err(|e| io::Error::new(e.kind(), format!("{hostport}: {e}")))?
        .collect();
    let mut last = None;
    for a in &addrs {
        match TcpStream::connect_timeout(a, limit) {
            Ok(s) => return Ok(s),
            Err(e) => last = Some(e),
        }
    }
    let (kind, why) = match last {
        Some(e) if e.kind() == io::ErrorKind::TimedOut => {
            (e.kind(), format!("no answer in {} s", limit.as_secs_f64()))
        }
        Some(e) => (e.kind(), e.to_string()),
        None => (io::ErrorKind::NotFound, "it names no address".to_string()),
    };
    Err(io::Error::new(
        kind,
        format!(
            "could not reach the adapter at {hostport} ({why}): is this computer on the adapter's own Wi-Fi network, not \
             another one? A Wi-Fi adapter takes one connection at a time, so close any phone app connected to it"
        ),
    ))
}

/// A file descriptor (serial line or Bluetooth socket), read with poll so a silent adapter never hangs Icarus.
pub struct FdPort {
    fd: OwnedFd,
    /// The serial line's speed (`open_serial`), which `set_speed` changes; None for a Bluetooth socket or anything else.
    speed: Option<u32>,
    /// A serial port this program marked exclusive itself (`open_serial`: TIOCEXCL, `mark_exclusive`), its mark taken
    /// off when it closes. A mark that was there before (another program's) is not this program's to take off.
    exclusive: bool,
}

impl Drop for FdPort {
    fn drop(&mut self) {
        // a tty can outlive the last close (a pty while its other end is open: socat's stand-in for a Wi-Fi adapter), and
        // its exclusive mark with it; cleared, so the next program to open it is not refused for nothing
        if self.exclusive {
            unsafe { libc::ioctl(self.fd.as_raw_fd(), libc::TIOCNXCL) };
        }
    }
}

/// Marks the tty open at `fd` exclusive (TIOCEXCL: the kernel refuses any other open of it but root's), and says
/// whether this call made the mark. It did not when the tty was marked already: another program's mark, which only
/// root's open gets past (gpsd marks its receiver so, without a lock). Nor when the kernel cannot say (TIOCGEXCL is
/// Linux 3.8 and later): only a mark this program is sure it made is taken off when it lets go.
fn mark_exclusive(fd: &OwnedFd, path: &str) -> io::Result<bool> {
    let mut was: libc::c_int = 0;
    let known = unsafe { libc::ioctl(fd.as_raw_fd(), libc::TIOCGEXCL, &mut was) } == 0;
    if unsafe { libc::ioctl(fd.as_raw_fd(), libc::TIOCEXCL) } != 0 {
        let e = io::Error::last_os_error();
        return Err(io::Error::new(
            e.kind(),
            format!("could not take {path} for this program alone ({e})"),
        ));
    }
    Ok(known && was == 0)
}

impl FdPort {
    /// Whether this open port is a serial port on a USB cable, judged by the open file itself: its device number
    /// (`st_rdev`) leads to its folder in sysfs (`usb_char`), so a symlink moved since it was opened cannot change the
    /// answer. A Bluetooth socket, a pipe or anything else that is not a character device is not.
    pub fn wired_on(&self, pc: &dyn Computer) -> bool {
        let mut st: libc::stat = unsafe { std::mem::zeroed() };
        if unsafe { libc::fstat(self.fd.as_raw_fd(), &mut st) } != 0 {
            return false;
        }
        st.st_mode & libc::S_IFMT == libc::S_IFCHR
            && usb_char(&pc.sys(), libc::major(st.st_rdev), libc::minor(st.st_rdev))
    }
}

impl Port for FdPort {
    fn send(&mut self, mut data: &[u8]) -> io::Result<()> {
        while !data.is_empty() {
            let n = unsafe { libc::write(self.fd.as_raw_fd(), data.as_ptr().cast(), data.len()) };
            if n < 0 {
                let e = io::Error::last_os_error();
                if e.kind() == io::ErrorKind::Interrupted {
                    continue;
                }
                return Err(e);
            }
            data = &data[n as usize..];
        }
        Ok(())
    }

    fn recv(&mut self, buf: &mut [u8], timeout: Duration) -> io::Result<usize> {
        let mut p = libc::pollfd {
            fd: self.fd.as_raw_fd(),
            events: libc::POLLIN,
            revents: 0,
        };
        let ms = timeout.as_millis().min(i32::MAX as u128) as i32;
        let r = unsafe { libc::poll(&mut p, 1, ms) };
        if r < 0 {
            // a signal (Ctrl+C) woke the wait: nothing has come yet, and the caller looks at its interrupt flag (ICA-83: a bus
            // watch ended with "Interrupted system call" and no summary)
            let e = io::Error::last_os_error();
            return if e.kind() == io::ErrorKind::Interrupted {
                Ok(0)
            } else {
                Err(e)
            };
        }
        if r == 0 {
            return Ok(0);
        }
        if p.revents & (libc::POLLERR | libc::POLLHUP) != 0 && p.revents & libc::POLLIN == 0 {
            return Err(io::Error::new(
                io::ErrorKind::BrokenPipe,
                "the adapter went away",
            ));
        }
        let n = unsafe { libc::read(self.fd.as_raw_fd(), buf.as_mut_ptr().cast(), buf.len()) };
        if n < 0 {
            let e = io::Error::last_os_error();
            return if e.kind() == io::ErrorKind::Interrupted {
                Ok(0)
            } else {
                Err(e)
            };
        }
        Ok(n as usize)
    }

    fn wired(&self) -> bool {
        self.wired_on(&ThisComputer)
    }

    fn speed(&self) -> Option<u32> {
        self.speed
    }

    fn set_speed(&mut self, baud: u32) -> io::Result<()> {
        if self.speed.is_none() {
            return Err(io::Error::new(
                io::ErrorKind::Unsupported,
                "not a serial line: no speed to change",
            ));
        }
        let speed = baud_const(baud)?;
        let fd = self.fd.as_raw_fd();
        unsafe {
            let mut t: libc::termios = std::mem::zeroed();
            if libc::tcgetattr(fd, &mut t) != 0 {
                return Err(io::Error::last_os_error());
            }
            libc::cfsetispeed(&mut t, speed);
            libc::cfsetospeed(&mut t, speed);
            // TCSADRAIN: what was sent goes out at the old speed first
            if libc::tcsetattr(fd, libc::TCSADRAIN, &t) != 0 {
                return Err(io::Error::last_os_error());
            }
        }
        self.speed = Some(baud);
        Ok(())
    }
}

fn baud_const(baud: u32) -> io::Result<libc::speed_t> {
    Ok(match baud {
        9600 => libc::B9600,
        19200 => libc::B19200,
        38400 => libc::B38400,
        57600 => libc::B57600,
        115200 => libc::B115200,
        230400 => libc::B230400,
        460800 => libc::B460800,
        500000 => libc::B500000,
        921600 => libc::B921600,
        1000000 => libc::B1000000,
        2000000 => libc::B2000000,
        _ => {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("unsupported baud rate {baud}"),
            ))
        }
    })
}

/// Where the computer's programs and their open files are read from: `/proc` (`Procs::this`), or a made-up one in tests.
pub struct Procs(pub PathBuf);

impl Procs {
    pub fn this() -> Procs {
        Procs(PathBuf::from("/proc"))
    }

    /// The programs that have the file at `path` open, as (process ID, name), lowest first: each process whose open files
    /// (`PID/fd/N`, links to where each leads) include it, named by `PID/comm`. Only what this user may read: the programs
    /// of other users (ModemManager, gpsd) are not seen.
    pub fn holding(&self, path: &Path) -> Vec<(u32, String)> {
        let want = std::fs::canonicalize(path).unwrap_or_else(|_| path.to_path_buf());
        let Ok(all) = std::fs::read_dir(&self.0) else {
            return Vec::new();
        };
        let mut out = Vec::new();
        for p in all.flatten() {
            let Some(pid) = p.file_name().to_str().and_then(|n| n.parse::<u32>().ok()) else {
                continue;
            };
            let Ok(fds) = std::fs::read_dir(p.path().join("fd")) else {
                continue;
            };
            if fds
                .flatten()
                .any(|f| std::fs::read_link(f.path()).is_ok_and(|to| to == want))
            {
                let name = std::fs::read_to_string(p.path().join("comm"))
                    .map(|c| c.trim().to_string())
                    .unwrap_or_default();
                out.push((
                    pid,
                    if name.is_empty() {
                        "a program".into()
                    } else {
                        name
                    },
                ));
            }
        }
        out.sort();
        out
    }
}

/// The refusal of a port that is open already: named by `holders` (from `Procs::holding`) when they could be seen, this
/// program (`me`) told apart from the others.
fn in_use(path: &str, holders: &[(u32, String)], me: u32) -> io::Error {
    let others: Vec<String> = holders
        .iter()
        .filter(|(pid, _)| *pid != me)
        .map(|(pid, name)| format!("{name}, process {pid}"))
        .collect();
    let why = if !others.is_empty() {
        format!(
            "another program has {path} open ({}): one program at a time speaks to an adapter, so close that one first",
            others.join("; ")
        )
    } else if holders.iter().any(|(pid, _)| *pid == me) {
        format!(
            "{path} is open already in this program (another connection to it): one connection at a time speaks to an adapter"
        )
    } else {
        format!(
            "another program has {path} open (one this user cannot see, or that has just let it go): one program at a time \
             speaks to an adapter, so close that one first"
        )
    };
    io::Error::new(io::ErrorKind::ResourceBusy, why)
}

/// A serial line in raw mode (8N1, no flow control), held by this program alone (ICA-104): two programs on one adapter
/// garble each other's requests, and could cut a write or a routine in two. The port is locked (flock, which binds root
/// too) and marked exclusive (TIOCEXCL: the kernel refuses any other open of it but root's), so a second program is
/// refused; and a port another program had open before, locking nothing (a terminal program, `cat`), is refused when
/// /proc shows that program. Either refusal names the program when this user may see it.
pub fn open_serial(path: &str, baud: u32) -> io::Result<FdPort> {
    open_serial_with(path, baud, &Procs::this(), std::process::id())
}

/// `open_serial`, seeing the other programs through `procs`, this one being process `me`.
fn open_serial_with(path: &str, baud: u32, procs: &Procs, me: u32) -> io::Result<FdPort> {
    let speed = baud_const(baud)?;
    let c = std::ffi::CString::new(path)
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "bad path"))?;
    let fd = unsafe { libc::open(c.as_ptr(), libc::O_RDWR | libc::O_NOCTTY | libc::O_CLOEXEC) };
    if fd < 0 {
        let e = io::Error::last_os_error();
        // EBUSY: another program marked it exclusive
        return Err(if e.raw_os_error() == Some(libc::EBUSY) {
            in_use(path, &procs.holding(Path::new(path)), me)
        } else {
            e
        });
    }
    let fd = unsafe { OwnedFd::from_raw_fd(fd) };
    while unsafe { libc::flock(fd.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) } != 0 {
        let e = io::Error::last_os_error();
        match e.raw_os_error() {
            Some(libc::EINTR) => continue,
            Some(libc::EWOULDBLOCK) => {
                drop(fd);
                return Err(in_use(path, &procs.holding(Path::new(path)), me));
            }
            _ => {
                return Err(io::Error::new(
                    e.kind(),
                    format!("could not lock {path} for this program alone ({e})"),
                ))
            }
        }
    }
    let mine = mark_exclusive(&fd, path)?;
    let port = FdPort {
        fd,
        speed: Some(baud),
        exclusive: mine,
    };
    let before: Vec<(u32, String)> = procs
        .holding(Path::new(path))
        .into_iter()
        .filter(|(pid, _)| *pid != me)
        .collect();
    if !before.is_empty() {
        return Err(in_use(path, &before, me));
    }
    let fd = &port.fd;
    unsafe {
        let mut t: libc::termios = std::mem::zeroed();
        if libc::tcgetattr(fd.as_raw_fd(), &mut t) != 0 {
            return Err(io::Error::last_os_error());
        }
        libc::cfmakeraw(&mut t);
        t.c_cflag |= libc::CLOCAL | libc::CREAD;
        t.c_cflag &= !(libc::CRTSCTS | libc::CSTOPB | libc::PARENB);
        t.c_cc[libc::VMIN] = 0;
        t.c_cc[libc::VTIME] = 0;
        libc::cfsetispeed(&mut t, speed);
        libc::cfsetospeed(&mut t, speed);
        if libc::tcsetattr(fd.as_raw_fd(), libc::TCSANOW, &t) != 0 {
            return Err(io::Error::last_os_error());
        }
        libc::tcflush(fd.as_raw_fd(), libc::TCIOFLUSH);
    }
    Ok(port)
}

const AF_BLUETOOTH: libc::c_int = 31;
const BTPROTO_RFCOMM: libc::c_int = 3;

#[repr(C)]
struct SockaddrRc {
    family: libc::sa_family_t,
    /// Least significant byte first: AA:BB:CC:DD:EE:FF is stored FF EE DD CC BB AA.
    bdaddr: [u8; 6],
    channel: u8,
}

/// An RFCOMM (Bluetooth serial) connection to a paired adapter, without the old `rfcomm bind` tool.
pub fn open_rfcomm(addr: [u8; 6], channel: u8) -> io::Result<FdPort> {
    let fd = unsafe {
        libc::socket(
            AF_BLUETOOTH,
            libc::SOCK_STREAM | libc::SOCK_CLOEXEC,
            BTPROTO_RFCOMM,
        )
    };
    if fd < 0 {
        return Err(io::Error::last_os_error());
    }
    let fd = unsafe { OwnedFd::from_raw_fd(fd) };
    let mut bd = addr;
    bd.reverse();
    let sa = SockaddrRc {
        family: AF_BLUETOOTH as libc::sa_family_t,
        bdaddr: bd,
        channel,
    };
    let r = unsafe {
        libc::connect(
            fd.as_raw_fd(),
            (&sa as *const SockaddrRc).cast(),
            std::mem::size_of::<SockaddrRc>() as libc::socklen_t,
        )
    };
    if r != 0 {
        let e = io::Error::last_os_error();
        return Err(io::Error::new(e.kind(), format!("Bluetooth connect failed ({e}); is the adapter paired, on, and plugged into the car?")));
    }
    Ok(FdPort {
        fd,
        speed: None,
        exclusive: false,
    })
}

pub struct TcpPort(TcpStream);

impl Port for TcpPort {
    fn send(&mut self, data: &[u8]) -> io::Result<()> {
        self.0.write_all(data)
    }

    fn recv(&mut self, buf: &mut [u8], timeout: Duration) -> io::Result<usize> {
        self.0
            .set_read_timeout(Some(timeout.max(Duration::from_millis(1))))?;
        match self.0.read(buf) {
            Ok(0) => Err(io::Error::new(
                io::ErrorKind::BrokenPipe,
                "the adapter closed the connection",
            )),
            Ok(n) => Ok(n),
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock
                        | io::ErrorKind::TimedOut
                        | io::ErrorKind::Interrupted
                ) =>
            {
                Ok(0)
            }
            Err(e) => Err(e),
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    #[test]
    fn a_signal_while_waiting_is_nothing_yet_not_an_error() {
        // a pipe stands in for the serial port; SIGUSR1 (handled, not restarted) interrupts the wait as Ctrl+C does
        extern "C" fn on_signal(_: libc::c_int) {}
        let mut fds = [0; 2];
        assert_eq!(unsafe { libc::pipe(fds.as_mut_ptr()) }, 0);
        let mut port = super::FdPort {
            fd: unsafe { <std::os::fd::OwnedFd as std::os::fd::FromRawFd>::from_raw_fd(fds[0]) },
            speed: None,
            exclusive: false,
        };
        unsafe {
            let mut sa: libc::sigaction = std::mem::zeroed();
            sa.sa_sigaction = on_signal as extern "C" fn(libc::c_int) as usize;
            sa.sa_flags = 0; // no SA_RESTART: the wait comes back with EINTR
            libc::sigaction(libc::SIGUSR1, &sa, std::ptr::null_mut());
        }
        let me = unsafe { libc::pthread_self() };
        let t = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(50));
            unsafe { libc::pthread_kill(me, libc::SIGUSR1) };
        });
        let mut buf = [0u8; 16];
        let r = super::Port::recv(&mut port, &mut buf, Duration::from_secs(2));
        t.join().unwrap();
        assert_eq!(r.unwrap(), 0, "an interrupted wait is nothing yet");
        unsafe { libc::close(fds[1]) };
    }

    use super::*;

    /// A made-up computer in a scratch folder: its `/dev` and `/sys` under `root`, laid out as Linux lays them out.
    pub(crate) struct Made {
        root: PathBuf,
    }

    impl Computer for Made {
        fn sys(&self) -> PathBuf {
            self.root.join("sys")
        }
        fn resolve(&self, path: &Path) -> Option<PathBuf> {
            let root = std::fs::canonicalize(&self.root).ok()?;
            let real = std::fs::canonicalize(self.root.join(path.strip_prefix("/").ok()?)).ok()?;
            Some(Path::new("/").join(real.strip_prefix(root).ok()?))
        }
    }

    impl Made {
        /// A new one, in a folder of its own: tests run side by side, and each makes its own computer.
        fn new(name: &str) -> Made {
            static MADE: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
            let n = MADE.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            let root = std::env::temp_dir()
                .join(format!("icarus-wired-{name}-{}-{n}", std::process::id()));
            let _ = std::fs::remove_dir_all(&root);
            std::fs::create_dir_all(&root).unwrap();
            Made { root }
        }
        /// A file (its folders made first).
        fn file(&self, at: &str, text: &str) {
            let p = self.root.join(at);
            std::fs::create_dir_all(p.parent().unwrap()).unwrap();
            std::fs::write(p, text).unwrap();
        }
        fn dir(&self, at: &str) {
            std::fs::create_dir_all(self.root.join(at)).unwrap();
        }
        /// A symlink at `at` to `to`, written relative as udev and sysfs write theirs.
        fn link(&self, at: &str, to: &str) {
            let p = self.root.join(at);
            std::fs::create_dir_all(p.parent().unwrap()).unwrap();
            std::os::unix::fs::symlink(to, p).unwrap();
        }
        /// The `device` link in sysfs folder `at` to `device`, a folder above it, written as the kernel writes it
        /// (`../../../1-2:1.0`).
        fn device_link(&self, at: &str, device: &str) {
            let up = at.matches('/').count() - device.matches('/').count() + 1;
            let last = &device[device.rfind('/').unwrap() + 1..];
            self.link(
                &format!("sys/{at}/device"),
                &format!("{}{last}", "../".repeat(up)),
            );
        }
        /// A tty in /dev, its sysfs folder at `class_at` (under sys), whose device is `device` (a folder above it), or a
        /// virtual one (no device link).
        fn tty(&self, name: &str, device: Option<&str>, class_at: &str) {
            self.file(&format!("dev/{name}"), "");
            self.dir(&format!("sys/{class_at}"));
            self.link(
                &format!("sys/class/tty/{name}"),
                &format!("../../{class_at}"),
            );
            if let Some(d) = device {
                self.device_link(class_at, d);
            }
        }
        /// The character device `major:minor` in sysfs (`dev/char/MAJ:MIN`), leading to its folder at `class_at`.
        fn char_dev(&self, major: u32, minor: u32, class_at: &str) {
            self.link(
                &format!("sys/dev/char/{major}:{minor}"),
                &format!("../../{class_at}"),
            );
        }
        /// A network interface of the kernel's CAN type, its sysfs folder at `class_at`, whose device is `device` (a
        /// folder above it), or a virtual one.
        fn can(&self, name: &str, device: Option<&str>, class_at: &str) {
            self.file(&format!("sys/{class_at}/type"), "280\n");
            self.link(
                &format!("sys/class/net/{name}"),
                &format!("../../{class_at}"),
            );
            if let Some(d) = device {
                self.device_link(class_at, d);
            }
        }
    }

    impl Drop for Made {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.root);
        }
    }

    const HUB: &str = "devices/pci0000:00/0000:00:14.0/usb1";

    /// A laptop like mine, made up: an OBDLink EX on USB (ftdi_sio, ttyUSB0), an OpenPort on USB (cdc_acm, ttyACM0, found
    /// by its IDs too), an OBDLink MX+ over Bluetooth (rfcomm0, connected, so the kernel has moved it under the Bluetooth
    /// controller, which is on USB; with a udev symlink to it), an FTDI cable plugged into another computer and attached
    /// here over USB/IP (ttyUSB1), a socat pty standing in for a Wi-Fi adapter, the built-in serial port, a CANable (gs_usb
    /// can0) and a vcan. Each tty also by its device number (`dev/char`), as an open port is judged.
    pub(crate) fn laptop() -> Made {
        let m = Made::new("laptop");
        let ftdi = format!("{HUB}/1-1/1-1:1.0/ttyUSB0");
        m.tty("ttyUSB0", Some(&ftdi), &format!("{ftdi}/tty/ttyUSB0"));
        m.char_dev(188, 0, &format!("{ftdi}/tty/ttyUSB0"));
        m.link(
            "dev/serial/by-id/usb-FTDI_FT231X_USB_UART-if00-port0",
            "../../ttyUSB0",
        );
        let acm = format!("{HUB}/1-2/1-2:1.0");
        m.tty("ttyACM0", Some(&acm), &format!("{acm}/tty/ttyACM0"));
        m.char_dev(166, 0, &format!("{acm}/tty/ttyACM0"));
        m.file(&format!("sys/{HUB}/1-2/idVendor"), "0403\n");
        m.file(&format!("sys/{HUB}/1-2/idProduct"), "cc4d\n");
        m.link("sys/bus/usb/devices/1-2", &format!("../../../{HUB}/1-2"));
        m.link("sys/bus/usb/devices/1-2:1.0", &format!("../../../{acm}"));
        // rfcomm_reparent_device: a connected rfcomm port's device is the Bluetooth connection (hci0:256)
        let hci = format!("{HUB}/1-4/1-4:1.0/bluetooth/hci0/hci0:256");
        m.tty("rfcomm0", Some(&hci), &format!("{hci}/rfcomm0"));
        m.char_dev(216, 0, &format!("{hci}/rfcomm0"));
        m.link("dev/obdlink", "rfcomm0");
        let usbip = "devices/platform/vhci_hcd.0/usb3/3-1/3-1:1.0/ttyUSB1";
        m.tty("ttyUSB1", Some(usbip), &format!("{usbip}/tty/ttyUSB1"));
        m.char_dev(188, 1, &format!("{usbip}/tty/ttyUSB1"));
        m.file("dev/pts/3", "");
        m.link("tmp/ttyV0", "../dev/pts/3");
        m.tty(
            "ttyS0",
            Some("devices/platform/serial8250"),
            "devices/platform/serial8250/tty/ttyS0",
        );
        m.char_dev(4, 64, "devices/platform/serial8250/tty/ttyS0");
        // the raw USB device node of the EX (/dev/bus/usb/001/002): on USB, but not a tty
        m.char_dev(189, 1, &format!("{HUB}/1-1"));
        let canable = format!("{HUB}/1-3/1-3:1.0");
        m.can("can0", Some(&canable), &format!("{canable}/net/can0"));
        m.can("vcan0", None, "devices/virtual/net/vcan0");
        m
    }

    /// What is wired is told by sysfs, not by a name, and anything that cannot be told is not wired. Read from a made-up
    /// computer, so the test passes on any machine.
    #[test]
    fn only_a_cable_is_wired() {
        let pc = laptop();
        let w = |s: &str| Target::parse(s).unwrap().wired_on(&pc);
        // cables on USB, by name or by udev's symlink
        assert!(w("/dev/ttyUSB0") && w("/dev/ttyUSB0@115200") && w("/dev/ttyACM0@115200"));
        assert!(w("/dev/serial/by-id/usb-FTDI_FT231X_USB_UART-if00-port0"));
        assert!(w("slcan:/dev/ttyACM0") && w("can0"));
        assert!(w("openport") && w("openport:/dev/ttyACM0") && w("mongoose:/dev/ttyACM0"));
        // the simulation reaches no vehicle
        assert!(w("sim") && w("sim:off") && w("sim:j2534"));
        // Bluetooth and Wi-Fi, under any name
        assert!(
            !w("/dev/rfcomm0") && !w("/dev/obdlink"),
            "a symlink to rfcomm0"
        );
        assert!(!w("/tmp/ttyV0") && !w("/dev/pts/3"), "a pty");
        assert!(!w("bt:AA:BB:CC:DD:EE:FF") && !w("tcp:192.168.0.10:35000"));
        assert!(!w("mongoose:/dev/obdlink") && !w("openport:/dev/rfcomm0"));
        assert!(
            !w("/dev/ttyUSB1"),
            "a cable on another computer, attached over USB/IP"
        );
        // a vcan: a bridge (cannelloni, cangw) can carry its frames anywhere
        assert!(!w("vcan0") && !w("can:vcan0"));
        // what cannot be told: not there, not a CAN interface, not on USB
        assert!(!w("/dev/ttyUSB9") && !w("/dev/nonsense") && !w("can7") && !w("can:.."));
        assert!(
            !w("/dev/ttyS0"),
            "a built-in serial port is not known to be a cable"
        );
    }

    /// A connected rfcomm port sits under the Bluetooth controller, which is on USB: where it sits refuses it, not only
    /// its name.
    #[test]
    fn a_connected_bluetooth_port_is_not_on_usb_whatever_its_name() {
        let pc = laptop();
        let sys = pc.sys();
        assert!(on_usb(&sys, Path::new("class/tty/ttyUSB0/device")));
        assert!(!on_usb(&sys, Path::new("class/tty/rfcomm0/device")));
        assert!(
            !on_usb(&sys, Path::new("class/tty/ttyUSB1/device")),
            "USB/IP"
        );
        // the same ports by their device numbers, as an open port is judged
        assert!(usb_char(&sys, 188, 0) && usb_char(&sys, 166, 0));
        assert!(!usb_char(&sys, 216, 0), "rfcomm0");
        assert!(!usb_char(&sys, 188, 1), "USB/IP");
        assert!(!usb_char(&sys, 4, 64), "the built-in serial port");
        assert!(
            !usb_char(&sys, 189, 1),
            "a raw USB device node is not a tty"
        );
        assert!(!usb_char(&sys, 188, 9), "not there");
    }

    /// An open port is judged by the open file: whatever a path leads to later, a pipe (here standing in for anything that
    /// is not a character device: a Bluetooth socket, a pty's other end) is never wired.
    #[test]
    fn an_open_port_that_is_not_a_character_device_is_not_wired() {
        let pc = laptop();
        let mut fds = [0; 2];
        assert_eq!(unsafe { libc::pipe(fds.as_mut_ptr()) }, 0);
        let port = FdPort {
            fd: unsafe { OwnedFd::from_raw_fd(fds[0]) },
            speed: None,
            exclusive: false,
        };
        assert!(!port.wired_on(&pc));
        unsafe { libc::close(fds[1]) };
        assert!(!Silent.wired(), "a port that cannot tell is not wired");
    }

    /// Who has a port open, read from a made-up /proc: the processes whose open files lead to it, by name, also when it is
    /// asked for by a symlink; other processes, and entries that are not processes, left out.
    #[test]
    fn the_programs_holding_a_port_are_named_from_proc() {
        let m = Made::new("proc");
        m.file("dev/ttyUSB0", "");
        m.file("dev/ttyUSB1", "");
        m.link("dev/serial/by-id/usb-cable", "../../ttyUSB0");
        let port = std::fs::canonicalize(m.root.join("dev/ttyUSB0")).unwrap();
        let other = std::fs::canonicalize(m.root.join("dev/ttyUSB1")).unwrap();
        m.file("proc/4242/comm", "minicom\n");
        m.link("proc/4242/fd/0", "/dev/null");
        m.link("proc/4242/fd/3", port.to_str().unwrap());
        m.link("proc/4242/fd/4", "socket:[1234]");
        m.file("proc/77/comm", "gpsd\n");
        m.link("proc/77/fd/5", other.to_str().unwrap());
        m.file("proc/90/comm", "\n");
        m.link("proc/90/fd/1", port.to_str().unwrap());
        m.link("proc/self", "4242");
        let procs = Procs(m.root.join("proc"));
        let both = vec![(90, "a program".to_string()), (4242, "minicom".to_string())];
        assert_eq!(procs.holding(&m.root.join("dev/ttyUSB0")), both);
        assert_eq!(
            procs.holding(&m.root.join("dev/serial/by-id/usb-cable")),
            both
        );
        assert_eq!(
            procs.holding(&m.root.join("dev/ttyUSB1")),
            vec![(77, "gpsd".to_string())]
        );
        assert!(procs.holding(&m.root.join("dev/ttyUSB9")).is_empty());
        assert!(Procs(m.root.join("none")).holding(&port).is_empty());
    }

    #[test]
    fn a_port_in_use_is_refused_in_words() {
        let e = in_use("/dev/ttyUSB0", &[(4242, "icarus-app".into())], 1);
        assert_eq!(e.kind(), io::ErrorKind::ResourceBusy);
        assert!(
            e.to_string()
                .starts_with("another program has /dev/ttyUSB0 open (icarus-app, process 4242)"),
            "{e}"
        );
        let e = in_use("/dev/ttyUSB0", &[], 1).to_string();
        assert!(
            e.starts_with("another program has /dev/ttyUSB0 open (one this user cannot see"),
            "{e}"
        );
        let e = in_use("/dev/ttyUSB0", &[(1, "icarus".into())], 1).to_string();
        assert!(e.contains("open already in this program"), "{e}");
    }

    /// A pseudo-terminal made for a test, standing in for a cable's serial port, its other end kept open as socat keeps
    /// one: (that other end, the port's path).
    pub(crate) fn pty() -> (OwnedFd, String) {
        let m = unsafe { libc::posix_openpt(libc::O_RDWR | libc::O_NOCTTY | libc::O_CLOEXEC) };
        assert!(m >= 0, "{}", io::Error::last_os_error());
        let m = unsafe { OwnedFd::from_raw_fd(m) };
        assert_eq!(unsafe { libc::grantpt(m.as_raw_fd()) }, 0);
        assert_eq!(unsafe { libc::unlockpt(m.as_raw_fd()) }, 0);
        let mut name = [0 as libc::c_char; 128];
        assert_eq!(
            unsafe { libc::ptsname_r(m.as_raw_fd(), name.as_mut_ptr(), name.len()) },
            0
        );
        let path = unsafe { std::ffi::CStr::from_ptr(name.as_ptr()) }
            .to_str()
            .unwrap()
            .to_string();
        (m, path)
    }

    /// The pty at `path` opened as any program would, with no lock and no mark: Ok, or the error number open gave.
    fn raw_open(path: &str) -> Result<OwnedFd, i32> {
        let c = std::ffi::CString::new(path).unwrap();
        let fd = unsafe { libc::open(c.as_ptr(), libc::O_RDWR | libc::O_NOCTTY | libc::O_CLOEXEC) };
        if fd < 0 {
            return Err(io::Error::last_os_error().raw_os_error().unwrap_or(0));
        }
        Ok(unsafe { OwnedFd::from_raw_fd(fd) })
    }

    /// Whether the tty open at `fd` is marked exclusive (TIOCGEXCL).
    fn marked(fd: &OwnedFd) -> bool {
        let mut on: libc::c_int = 0;
        assert_eq!(
            unsafe { libc::ioctl(fd.as_raw_fd(), libc::TIOCGEXCL, &mut on) },
            0,
            "{}",
            io::Error::last_os_error()
        );
        on != 0
    }

    /// ICA-104: one program at a time on an adapter. A pty stands in for the cable. The made-up /proc is empty at first,
    /// so only the kernel's locks (flock and TIOCEXCL) can refuse the second program; then it names the window.
    #[test]
    fn a_port_another_program_has_open_is_refused_with_its_name() {
        let (_other_end, path) = pty();
        let m = Made::new("held");
        let procs = Procs(m.root.join("proc"));
        // opened before Icarus takes it, and unseen by the made-up /proc: it shows the lock Icarus holds
        let early = raw_open(&path).expect("open before");
        let first = open_serial_with(&path, 38_400, &procs, 1).expect("the first program gets it");
        // flock: held, so another open file of the port cannot lock it
        assert_eq!(
            unsafe { libc::flock(early.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) },
            -1,
            "the lock is held"
        );
        assert_eq!(
            io::Error::last_os_error().raw_os_error(),
            Some(libc::EWOULDBLOCK)
        );
        // TIOCEXCL: the kernel refuses a fresh open of it (root's open passes it, the lock still holds root)
        assert!(marked(&early));
        if unsafe { libc::geteuid() } != 0 {
            assert_eq!(raw_open(&path).err(), Some(libc::EBUSY));
        }
        drop(early);
        let e = open_serial_with(&path, 38_400, &procs, 2)
            .err()
            .expect("a second program is refused by the locks alone");
        assert_eq!(e.kind(), io::ErrorKind::ResourceBusy);
        assert!(
            e.to_string().starts_with(&format!(
                "another program has {path} open (one this user cannot see"
            )),
            "{e}"
        );
        // the window, say, as /proc shows it: named
        m.file("proc/4242/comm", "icarus-app\n");
        m.link("proc/4242/fd/7", &path);
        let e = open_serial_with(&path, 38_400, &procs, 2)
            .err()
            .expect("a second program is refused");
        assert_eq!(e.kind(), io::ErrorKind::ResourceBusy);
        assert!(
            e.to_string().starts_with(&format!(
                "another program has {path} open (icarus-app, process 4242)"
            )),
            "{e}"
        );
        // let go, it is anyone's again: the exclusive mark is cleared, though the pty's other end is still open
        drop(first);
        std::fs::remove_dir_all(m.root.join("proc/4242")).unwrap();
        drop(open_serial_with(&path, 38_400, &procs, 2).expect("free again"));
        // a program that opened it before, locking nothing (a terminal program), is seen in /proc and refused too
        let raw = raw_open(&path).expect("free, so anyone may open it");
        m.file("proc/5150/comm", "screen\n");
        m.link("proc/5150/fd/3", &path);
        let e = open_serial_with(&path, 38_400, &procs, 2)
            .err()
            .expect("refused while screen has it")
            .to_string();
        assert!(
            e.starts_with(&format!(
                "another program has {path} open (screen, process 5150)"
            )),
            "{e}"
        );
        drop(raw);
        std::fs::remove_dir_all(m.root.join("proc/5150")).unwrap();
        assert!(open_serial_with(&path, 38_400, &procs, 2).is_ok());
    }

    /// ICA-104: a program that locked the port (flock) without marking it is refused by the lock alone, and the refused
    /// open leaves no mark behind.
    #[test]
    fn a_port_another_program_locked_is_refused() {
        let (_other_end, path) = pty();
        let theirs = raw_open(&path).expect("open");
        assert_eq!(
            unsafe { libc::flock(theirs.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) },
            0
        );
        let m = Made::new("locked");
        let procs = Procs(m.root.join("proc"));
        let e = open_serial_with(&path, 38_400, &procs, 2)
            .err()
            .expect("refused by the lock");
        assert_eq!(e.kind(), io::ErrorKind::ResourceBusy);
        assert!(
            e.to_string().starts_with(&format!(
                "another program has {path} open (one this user cannot see"
            )),
            "{e}"
        );
        assert!(!marked(&theirs), "a refused open leaves no mark");
        drop(theirs);
        assert!(open_serial_with(&path, 38_400, &procs, 2).is_ok());
    }

    /// ICA-104: a mark another program made (gpsd marks its receiver so, without a lock) is never taken off by Icarus.
    /// Root's open gets past it, and taking it off when letting go would leave that program's port open to everyone.
    #[test]
    fn another_programs_exclusive_mark_is_left_on() {
        let (_other_end, path) = pty();
        let theirs = raw_open(&path).expect("open");
        assert_eq!(
            unsafe { libc::ioctl(theirs.as_raw_fd(), libc::TIOCEXCL) },
            0
        );
        // what open_serial does past the open, on a second open file of the port (as root's open would get one)
        let second = theirs.try_clone().unwrap();
        assert!(
            !mark_exclusive(&second, &path).unwrap(),
            "not this program's mark"
        );
        drop(FdPort {
            fd: second,
            speed: None,
            exclusive: false,
        });
        assert!(marked(&theirs), "their mark stays");
        // the whole open, as whoever runs the tests: refused (EBUSY), or, as root, let in; either way the mark stays
        let m = Made::new("theirs");
        drop(open_serial_with(
            &path,
            38_400,
            &Procs(m.root.join("proc")),
            2,
        ));
        assert!(marked(&theirs), "their mark stays");
        // on a port nobody marked, the mark is this program's own
        let (_other_end, free) = pty();
        let fd = raw_open(&free).expect("open");
        assert!(mark_exclusive(&fd, &free).unwrap());
        assert!(marked(&fd));
        let mine = FdPort {
            fd: fd.try_clone().unwrap(),
            speed: None,
            exclusive: true,
        };
        drop(mine);
        assert!(!marked(&fd), "taken off when this program lets go");
    }

    /// A port that does not say whether it is a cable (`Port::wired`'s default).
    struct Silent;

    impl Port for Silent {
        fn send(&mut self, _: &[u8]) -> io::Result<()> {
            Ok(())
        }
        fn recv(&mut self, _: &mut [u8], _: Duration) -> io::Result<usize> {
            Ok(0)
        }
    }

    /// An slcan interface (here over Bluetooth, through rfcomm0) has no device in sysfs, so it is not wired even
    /// named can0, or vcan0; and with no OpenPort plugged in, `openport` is not wired.
    #[test]
    fn a_can_interface_over_bluetooth_is_not_wired_whatever_its_name() {
        let pc = Made::new("slcan");
        let hci = format!("{HUB}/1-4/1-4:1.0/bluetooth/hci0/hci0:256");
        pc.tty("rfcomm0", Some(&hci), &format!("{hci}/rfcomm0"));
        pc.can("can0", None, "devices/virtual/net/can0");
        pc.can("vcan0", None, "devices/virtual/net/vcan0");
        let w = |s: &str| Target::parse(s).unwrap().wired_on(&pc);
        assert!(!w("can0") && !w("can:can0") && !w("vcan0"));
        assert!(!w("slcan:/dev/rfcomm0"));
        assert!(!w("openport"));
        assert!(w("sim"));
    }

    #[test]
    fn j2534_cables_from_the_command_line() {
        assert_eq!(
            Target::parse("openport").unwrap(),
            Target::J2534(J2534Device::OpenPort(None))
        );
        assert_eq!(
            Target::parse("openport:/dev/ttyACM1").unwrap(),
            Target::J2534(J2534Device::OpenPort(Some("/dev/ttyACM1".into())))
        );
        assert!(Target::parse("openport:ttyACM0").is_err());
        assert_eq!(
            Target::parse("mongoose:/dev/ttyACM0").unwrap(),
            Target::J2534(J2534Device::Mongoose("/dev/ttyACM0".into()))
        );
        assert!(Target::parse("mongoose").is_err() && Target::parse("mongoose:ttyACM0").is_err());
        assert_eq!(
            Target::parse("sim:j2534").unwrap(),
            Target::J2534(J2534Device::Sim)
        );
        let e = Target::parse("openport").unwrap().open().err().unwrap();
        assert!(e.to_string().contains("own driver"), "{e}");
    }

    #[test]
    fn targets_from_the_command_line() {
        assert_eq!(
            Target::parse("sim").unwrap(),
            Target::Sim {
                engine_on: true,
                listen: false,
                j1939: false,
                drive: false,
            }
        );
        assert_eq!(
            Target::parse("sim:off").unwrap(),
            Target::Sim {
                engine_on: false,
                listen: false,
                j1939: false,
                drive: false,
            }
        );
        assert_eq!(
            Target::parse("/dev/ttyUSB0").unwrap(),
            Target::Serial {
                path: "/dev/ttyUSB0".into(),
                baud: 38400
            }
        );
        assert_eq!(
            Target::parse("/dev/rfcomm0@115200").unwrap(),
            Target::Serial {
                path: "/dev/rfcomm0".into(),
                baud: 115200
            }
        );
        assert_eq!(
            Target::parse("bt:00:11:22:33:44:5A#2").unwrap(),
            Target::Bluetooth {
                addr: [0x00, 0x11, 0x22, 0x33, 0x44, 0x5A],
                channel: 2
            }
        );
        assert_eq!(
            Target::parse("tcp:192.168.0.10:35000").unwrap(),
            Target::Tcp("192.168.0.10:35000".into())
        );
        // a CAN interface: by name alone when it looks like one, or after can:
        for (s, ifn) in [
            ("can0", "can0"),
            ("vcan0", "vcan0"),
            ("can:slcan0", "slcan0"),
        ] {
            assert_eq!(Target::parse(s).unwrap(), Target::Can(ifn.into()), "{s}");
        }
        assert!(
            Target::parse("can0").unwrap().open().is_err(),
            "not an ELM327"
        );
        // an SLCAN adapter: 115200 baud and 500 kbit/s unless given; never opened as an ELM327
        let slcan = |path: &str, baud, bitrate| Target::Slcan {
            path: path.into(),
            baud,
            bitrate,
        };
        assert_eq!(
            Target::parse("slcan:/dev/ttyACM0").unwrap(),
            slcan("/dev/ttyACM0", 115_200, 500_000)
        );
        assert_eq!(
            Target::parse("slcan:/dev/ttyUSB1@921600#125k").unwrap(),
            slcan("/dev/ttyUSB1", 921_600, 125_000)
        );
        assert_eq!(
            Target::parse("slcan:/dev/ttyACM0#250000").unwrap(),
            slcan("/dev/ttyACM0", 115_200, 250_000)
        );
        assert!(Target::parse("slcan:/dev/ttyACM0").unwrap().open().is_err());
        for bad in [
            "slcan:",
            "slcan:ttyACM0",
            "slcan:/dev/ttyACM0#33k",
            "slcan:/dev/x@fast",
        ] {
            assert!(Target::parse(bad).is_err(), "{bad}");
        }
        for bad in [
            "bt:00:11",
            "bt:00:11:22:33:44:ZZ",
            "bt:00:11:22:33:44:55#0",
            "tcp:host",
            "COM3",
            "/dev/x@fast",
            "can:",
            "can:../x",
            "canx",
            "can",
        ] {
            assert!(Target::parse(bad).is_err(), "{bad}");
        }
        assert_eq!(
            std::mem::size_of::<SockaddrRc>(),
            10,
            "the kernel's struct sockaddr_rc"
        );
    }

    /// ICA-110: a Wi-Fi adapter (`tcp:`) reached through `Target::Tcp`, here a listener on loopback speaking ELM327 as the
    /// simulated adapter, and a value read through it. No network: the loopback interface only.
    #[test]
    fn a_wifi_adapter_is_reached_over_tcp_and_read() {
        let l = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = l.local_addr().unwrap();
        let adapter = std::thread::spawn(move || {
            let (mut s, _) = l.accept().unwrap();
            s.set_read_timeout(Some(Duration::from_millis(5))).unwrap();
            let mut car = crate::sim::SimCar::new();
            let mut buf = [0u8; 1024];
            loop {
                match s.read(&mut buf) {
                    Ok(0) => break,
                    Ok(n) => car.send(&buf[..n]).unwrap(),
                    Err(e)
                        if matches!(
                            e.kind(),
                            io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                        ) => {}
                    Err(_) => break,
                }
                let n = car.recv(&mut buf, Duration::ZERO).unwrap();
                if n > 0 && s.write_all(&buf[..n]).is_err() {
                    break;
                }
            }
        });
        let target = Target::parse(&format!("tcp:{addr}")).unwrap();
        assert_eq!(target, Target::Tcp(addr.to_string()));
        let (mut elm, note) = crate::elm::connect_target(&target).unwrap();
        assert!(note.is_none());
        let rpm = elm.request(&[0x01, 0x0C]).unwrap();
        assert!(
            rpm.iter().any(|a| a.data.starts_with(&[0x41, 0x0C])),
            "{rpm:?}"
        );
        assert!(!crate::link::Link::wired(&elm), "Wi-Fi is never a cable");
        drop(elm);
        adapter.join().unwrap();
    }

    /// A listening socket on loopback that never takes a connection, its queue of connections not yet taken full
    /// (`listen` with a backlog of 0, one connection in it): Linux drops a new connection's first packet, so connecting
    /// to it waits as connecting to an address nothing answers on does. The socket and the connection filling its queue.
    fn unanswering() -> (OwnedFd, std::net::SocketAddr, Vec<TcpStream>) {
        let fd = unsafe { libc::socket(libc::AF_INET, libc::SOCK_STREAM | libc::SOCK_CLOEXEC, 0) };
        assert!(fd >= 0, "{}", io::Error::last_os_error());
        let fd = unsafe { OwnedFd::from_raw_fd(fd) };
        let mut sa: libc::sockaddr_in = unsafe { std::mem::zeroed() };
        sa.sin_family = libc::AF_INET as libc::sa_family_t;
        sa.sin_addr.s_addr = u32::from(std::net::Ipv4Addr::LOCALHOST).to_be();
        let len = std::mem::size_of::<libc::sockaddr_in>() as libc::socklen_t;
        assert_eq!(
            unsafe {
                libc::bind(
                    fd.as_raw_fd(),
                    (&sa as *const libc::sockaddr_in).cast(),
                    len,
                )
            },
            0
        );
        assert_eq!(unsafe { libc::listen(fd.as_raw_fd(), 0) }, 0);
        let mut got = len;
        assert_eq!(
            unsafe {
                libc::getsockname(
                    fd.as_raw_fd(),
                    (&mut sa as *mut libc::sockaddr_in).cast(),
                    &mut got,
                )
            },
            0
        );
        let addr =
            std::net::SocketAddr::from((std::net::Ipv4Addr::LOCALHOST, u16::from_be(sa.sin_port)));
        // fill the queue: connections go in until one is not answered
        let mut held = Vec::new();
        for _ in 0..8 {
            match TcpStream::connect_timeout(&addr, Duration::from_millis(300)) {
                Ok(s) => held.push(s),
                Err(_) => break,
            }
        }
        (fd, addr, held)
    }

    /// ICA-110: a connection nobody answers ends at the limit, not after the kernel's two minutes, and says what usually
    /// causes it.
    #[test]
    fn a_tcp_adapter_that_does_not_answer_is_given_up_in_time() {
        let (_listening, addr, _held) = unanswering();
        let t = std::time::Instant::now();
        let e = connect_tcp(&addr.to_string(), Duration::from_millis(300)).unwrap_err();
        assert!(t.elapsed() < Duration::from_secs(3), "{:?}", t.elapsed());
        assert_eq!(e.kind(), io::ErrorKind::TimedOut, "{e}");
        let e = e.to_string();
        assert!(
            e.contains("no answer in 0.3 s")
                && e.contains("another one")
                && e.contains("one connection at a time"),
            "{e}"
        );
        // the adapter as Icarus opens it (`Target::Tcp`): given up at TCP_LIMIT
        let t = std::time::Instant::now();
        let e = Target::Tcp(addr.to_string()).open().err().unwrap();
        let took = t.elapsed();
        assert!(
            took >= TCP_LIMIT - Duration::from_millis(100)
                && took < TCP_LIMIT + Duration::from_secs(3),
            "{took:?}"
        );
        assert_eq!(e.kind(), io::ErrorKind::TimedOut, "{e}");
        assert!(e.to_string().contains("one connection at a time"), "{e}");
        // a port nothing listens on is refused at once, in the same words
        let free = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let gone = free.local_addr().unwrap();
        drop(free);
        let e = connect_tcp(&gone.to_string(), Duration::from_millis(300)).unwrap_err();
        assert_eq!(e.kind(), io::ErrorKind::ConnectionRefused, "{e}");
        assert!(e.to_string().contains("phone app"), "{e}");
    }

    /// ICA-206: a connection is judged by the port it opened. A pseudo-terminal stands in for each adapter's port, its
    /// device number leading in a made-up sysfs to the EX's cable (ttyUSB0) or to the MX+ over Bluetooth (rfcomm0); a udev
    /// symlink (`/dev/obd`) moved to the other one after connecting changes what the path is, not what the link is.
    #[test]
    fn a_link_is_judged_by_the_port_it_opened_not_where_its_path_leads_now() {
        use crate::link::Link;
        /// The simulated adapter's talk, over the open port standing in for it, judged on the made-up computer.
        struct Opened {
            car: crate::sim::SimCar,
            port: FdPort,
            pc: std::sync::Arc<Made>,
        }
        impl Port for Opened {
            fn send(&mut self, d: &[u8]) -> io::Result<()> {
                self.car.send(d)
            }
            fn recv(&mut self, b: &mut [u8], t: Duration) -> io::Result<usize> {
                self.car.recv(b, t)
            }
            fn wired(&self) -> bool {
                self.port.wired_on(&*self.pc)
            }
        }
        let pc = std::sync::Arc::new(laptop());
        let number = |fd: &OwnedFd| {
            let mut st: libc::stat = unsafe { std::mem::zeroed() };
            assert_eq!(unsafe { libc::fstat(fd.as_raw_fd(), &mut st) }, 0);
            (libc::major(st.st_rdev), libc::minor(st.st_rdev))
        };
        let procs = Procs(pc.root.join("proc"));
        let ftdi = format!("{HUB}/1-1/1-1:1.0/ttyUSB0/tty/ttyUSB0");
        let hci = format!("{HUB}/1-4/1-4:1.0/bluetooth/hci0/hci0:256/rfcomm0");
        let obd = Target::parse("/dev/obd").unwrap();
        for (opened, then) in [("ttyUSB0", "rfcomm0"), ("rfcomm0", "ttyUSB0")] {
            let (_other_end, path) = pty();
            let port = open_serial_with(&path, 115_200, &procs, 2).unwrap();
            let (major, minor) = number(&port.fd);
            pc.char_dev(major, minor, if opened == "ttyUSB0" { &ftdi } else { &hci });
            let _ = std::fs::remove_file(pc.root.join("dev/obd"));
            pc.link("dev/obd", opened);
            assert_eq!(obd.wired_on(&*pc), opened == "ttyUSB0");
            let link: Box<dyn Link> = Box::new(
                crate::elm::Elm::connect(Box::new(Opened {
                    car: crate::sim::SimCar::new(),
                    port,
                    pc: pc.clone(),
                }))
                .unwrap(),
            );
            // udev gives /dev/obd to the other adapter
            std::fs::remove_file(pc.root.join("dev/obd")).unwrap();
            pc.link("dev/obd", then);
            assert_eq!(obd.wired_on(&*pc), then == "ttyUSB0", "the path, now");
            assert_eq!(
                link.wired(),
                opened == "ttyUSB0",
                "the link opened on {opened}, whatever /dev/obd leads to now"
            );
            drop(link);
            std::fs::remove_file(pc.root.join(format!("sys/dev/char/{major}:{minor}"))).unwrap();
        }
    }
}
