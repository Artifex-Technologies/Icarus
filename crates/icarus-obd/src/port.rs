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
///   `/dev/ttyUSB0` or `/dev/ttyUSB0@115200`   a USB adapter (38400 baud unless given)
///   `bt:AA:BB:CC:DD:EE:FF` or `...#2`         a paired Bluetooth adapter, RFCOMM channel 1 unless given
///   `tcp:192.168.0.10:35000`       a Wi-Fi adapter
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Target {
    /// The simulated vehicle: engine running (`sim`) or ignition on, engine off (`sim:off`).
    Sim {
        engine_on: bool,
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
}

impl Target {
    /// A cable all the way: what anything that writes a module's memory needs (a dropped wireless link mid-write can leave
    /// the module half-written). A serial port named `rfcomm` is Bluetooth; the simulation counts as wired.
    pub fn wired(&self) -> bool {
        match self {
            Target::Sim { .. } => true,
            Target::Serial { path, .. } => !path.contains("rfcomm"),
            Target::Bluetooth { .. } | Target::Tcp(_) => false,
        }
    }

    pub fn parse(s: &str) -> Result<Target, String> {
        if s == "sim" || s == "sim:off" {
            return Ok(Target::Sim {
                engine_on: s == "sim",
            });
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
        Err(format!("unknown adapter {s:?}: use sim, /dev/ttyUSB0[@baud], bt:AA:BB:CC:DD:EE:FF[#channel] or tcp:host:port"))
    }

    pub fn open(&self) -> io::Result<Box<dyn Port>> {
        Ok(match self {
            Target::Sim { engine_on: true } => Box::new(crate::sim::SimCar::new()),
            Target::Sim { engine_on: false } => Box::new(crate::sim::SimCar::engine_off()),
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
            return Err(io::Error::last_os_error());
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
            return Err(io::Error::last_os_error());
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
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
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
    use super::*;

    #[test]
    fn only_a_cable_is_wired() {
        let w = |s: &str| Target::parse(s).unwrap().wired();
        assert!(w("sim") && w("/dev/ttyUSB0") && w("/dev/ttyACM0@115200"));
        assert!(!w("/dev/rfcomm0") && !w("bt:AA:BB:CC:DD:EE:FF") && !w("tcp:192.168.0.10:35000"));
    }

    #[test]
    fn targets_from_the_command_line() {
        assert_eq!(
            Target::parse("sim").unwrap(),
            Target::Sim { engine_on: true }
        );
        assert_eq!(
            Target::parse("sim:off").unwrap(),
            Target::Sim { engine_on: false }
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
        for bad in [
            "bt:00:11",
            "bt:00:11:22:33:44:ZZ",
            "bt:00:11:22:33:44:55#0",
            "tcp:host",
            "COM3",
            "/dev/x@fast",
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
