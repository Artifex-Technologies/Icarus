//! The link to an OBD adapter: a USB serial port, a Bluetooth RFCOMM channel (OBDLink MX+, most Bluetooth ELM327s) or a TCP
//! socket (Wi-Fi adapters). Each is just bytes in and out with a timeout; the adapter's language is in `elm`.

use std::io::{self, Read, Write};
use std::net::TcpStream;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::time::Duration;

pub trait Port {
    fn send(&mut self, data: &[u8]) -> io::Result<()>;
    /// Bytes that arrived, waiting up to `timeout`; 0 when nothing came in time.
    fn recv(&mut self, buf: &mut [u8], timeout: Duration) -> io::Result<usize>;
}

/// Where to reach the adapter, as written on the command line:
///   `sim` or `sim:off`             Icarus's simulated car, engine running or ignition on with the engine off
///   `sim:listen`                   a second simulated adapter on the same simulated bus, for listening (ICA-80)
///   `sim:j1939`                    a simulated heavy-duty vehicle on J1939, for watching (ICA-94)
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

impl Target {
    /// A cable all the way: what anything that writes a module's memory needs (a dropped wireless link mid-write can leave
    /// the module half-written). A serial port named `rfcomm` is Bluetooth; the simulation and a CAN interface (a USB CAN
    /// adapter) count as wired.
    pub fn wired(&self) -> bool {
        match self {
            Target::Sim { .. } | Target::Can(_) => true,
            // a USB cable, or the simulation
            Target::J2534(_) => true,
            Target::Serial { path, .. } | Target::Slcan { path, .. } => !path.contains("rfcomm"),
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
        if matches!(s, "sim" | "sim:off" | "sim:listen" | "sim:j1939") {
            return Ok(Target::Sim {
                engine_on: s != "sim:off",
                listen: s == "sim:listen",
                j1939: s == "sim:j1939",
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
            Target::Sim { listen: true, .. } => Box::new(crate::sim::SimCar::listening()),
            Target::Sim { engine_on: true, .. } => Box::new(crate::sim::SimCar::new()),
            Target::Sim { engine_on: false, .. } => Box::new(crate::sim::SimCar::engine_off()),
            Target::Serial { path, baud } => Box::new(open_serial(path, *baud)?),
            Target::Bluetooth { addr, channel } => Box::new(open_rfcomm(*addr, *channel)?),
            Target::Tcp(hostport) => {
                let s = TcpStream::connect(hostport)?;
                s.set_nodelay(true)?;
                Box::new(TcpPort(s))
            }
        })
    }
}

/// A file descriptor (serial line or Bluetooth socket), read with poll so a silent adapter never hangs Icarus.
pub struct FdPort(OwnedFd);

impl Port for FdPort {
    fn send(&mut self, mut data: &[u8]) -> io::Result<()> {
        while !data.is_empty() {
            let n = unsafe { libc::write(self.0.as_raw_fd(), data.as_ptr().cast(), data.len()) };
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
            fd: self.0.as_raw_fd(),
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
        let n = unsafe { libc::read(self.0.as_raw_fd(), buf.as_mut_ptr().cast(), buf.len()) };
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

/// A serial line in raw mode (8N1, no flow control).
pub fn open_serial(path: &str, baud: u32) -> io::Result<FdPort> {
    let speed = baud_const(baud)?;
    let c = std::ffi::CString::new(path)
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "bad path"))?;
    let fd = unsafe { libc::open(c.as_ptr(), libc::O_RDWR | libc::O_NOCTTY | libc::O_CLOEXEC) };
    if fd < 0 {
        return Err(io::Error::last_os_error());
    }
    let fd = unsafe { OwnedFd::from_raw_fd(fd) };
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
    Ok(FdPort(fd))
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
    Ok(FdPort(fd))
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
mod tests {
    #[test]
    fn a_signal_while_waiting_is_nothing_yet_not_an_error() {
        // a pipe stands in for the serial port; SIGUSR1 (handled, not restarted) interrupts the wait as Ctrl+C does
        extern "C" fn on_signal(_: libc::c_int) {}
        let mut fds = [0; 2];
        assert_eq!(unsafe { libc::pipe(fds.as_mut_ptr()) }, 0);
        let mut port = super::FdPort(unsafe {
            <std::os::fd::OwnedFd as std::os::fd::FromRawFd>::from_raw_fd(fds[0])
        });
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

    #[test]
    fn only_a_cable_is_wired() {
        let w = |s: &str| Target::parse(s).unwrap().wired();
        assert!(w("sim") && w("/dev/ttyUSB0") && w("/dev/ttyACM0@115200"));
        assert!(!w("/dev/rfcomm0") && !w("bt:AA:BB:CC:DD:EE:FF") && !w("tcp:192.168.0.10:35000"));
        assert!(w("openport") && w("openport:/dev/ttyACM0") && w("sim:j2534"));
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
            }
        );
        assert_eq!(
            Target::parse("sim:off").unwrap(),
            Target::Sim {
                engine_on: false,
                listen: false,
                j1939: false,
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
        assert!(Target::parse("can0").unwrap().wired());
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
}
