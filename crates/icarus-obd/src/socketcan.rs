//! Linux's own CAN interfaces (SocketCAN): USB CAN adapters such as a CANable show up as `can0`, set up once with
//! `sudo ip link set can0 up type can bitrate 500000`. Icarus reads frames from them for the bus monitor; it never writes to
//! the socket (watching only, like the adapter's monitor mode).

use crate::bus::Frame;
use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::time::Duration;

const AF_CAN: libc::c_int = 29;
const CAN_RAW: libc::c_int = 1;
const CAN_EFF_FLAG: u32 = 0x8000_0000;
const CAN_RTR_FLAG: u32 = 0x4000_0000;
const CAN_ERR_FLAG: u32 = 0x2000_0000;

#[repr(C)]
struct SockaddrCan {
    family: libc::sa_family_t,
    ifindex: libc::c_int,
    /// The kernel's address union (transport protocol IDs or J1939), unused for raw sockets; 8-byte aligned like the kernel's.
    addr: [u64; 2],
}

#[derive(Debug)]
pub struct CanSocket(OwnedFd);

/// Opens a raw CAN socket on an interface (`can0`, `vcan0`...).
pub fn open(ifname: &str) -> io::Result<CanSocket> {
    let name = std::ffi::CString::new(ifname)
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "bad interface name"))?;
    let ifindex = unsafe { libc::if_nametoindex(name.as_ptr()) };
    if ifindex == 0 {
        return Err(io::Error::new(
            io::ErrorKind::NotFound,
            format!("no network interface {ifname} (CAN interfaces: ip -brief link show type can)"),
        ));
    }
    let fd = unsafe { libc::socket(AF_CAN, libc::SOCK_RAW | libc::SOCK_CLOEXEC, CAN_RAW) };
    if fd < 0 {
        return Err(io::Error::last_os_error());
    }
    let fd = unsafe { OwnedFd::from_raw_fd(fd) };
    let sa = SockaddrCan {
        family: AF_CAN as libc::sa_family_t,
        ifindex: ifindex as libc::c_int,
        addr: [0; 2],
    };
    let r = unsafe {
        libc::bind(
            fd.as_raw_fd(),
            (&sa as *const SockaddrCan).cast(),
            std::mem::size_of::<SockaddrCan>() as libc::socklen_t,
        )
    };
    if r != 0 {
        let e = io::Error::last_os_error();
        return Err(io::Error::new(
            e.kind(),
            format!("{ifname}: {e} (is it a CAN interface, and up?)"),
        ));
    }
    Ok(CanSocket(fd))
}

/// A classic CAN frame from the kernel's 16-byte `struct can_frame` (ID with flags, length, 3 reserved bytes, 8 data bytes).
/// Remote and error frames are skipped.
pub fn decode(raw: &[u8; 16], at: f64) -> Option<Frame> {
    let id = u32::from_ne_bytes([raw[0], raw[1], raw[2], raw[3]]);
    if id & (CAN_RTR_FLAG | CAN_ERR_FLAG) != 0 {
        return None;
    }
    let ext = id & CAN_EFF_FLAG != 0;
    let len = (raw[4] as usize).min(8);
    Some(Frame {
        id: if ext { id & 0x1FFF_FFFF } else { id & 0x7FF },
        ext,
        data: raw[8..8 + len].to_vec(),
        at,
    })
}

impl CanSocket {
    /// The next frame, waiting up to `timeout` (None: none came, or it was not a data frame).
    pub fn read(&self, timeout: Duration, at: f64) -> io::Result<Option<Frame>> {
        let mut p = libc::pollfd {
            fd: self.0.as_raw_fd(),
            events: libc::POLLIN,
            revents: 0,
        };
        let r = unsafe { libc::poll(&mut p, 1, timeout.as_millis().min(i32::MAX as u128) as i32) };
        if r < 0 {
            return Err(io::Error::last_os_error());
        }
        if r == 0 {
            return Ok(None);
        }
        let mut raw = [0u8; 16];
        let n = unsafe { libc::read(self.0.as_raw_fd(), raw.as_mut_ptr().cast(), raw.len()) };
        if n < 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(if n as usize == raw.len() {
            decode(&raw, at)
        } else {
            None
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frames_from_the_kernel() {
        let mut raw = [0u8; 16];
        raw[..4].copy_from_slice(&0x2C4u32.to_ne_bytes());
        raw[4] = 3;
        raw[8..11].copy_from_slice(&[0x03, 0x1A, 0xF8]);
        assert_eq!(
            decode(&raw, 1.5).unwrap(),
            Frame {
                id: 0x2C4,
                ext: false,
                data: vec![0x03, 0x1A, 0xF8],
                at: 1.5
            }
        );
        raw[..4].copy_from_slice(&(0x18DA_F110u32 | CAN_EFF_FLAG).to_ne_bytes());
        let e = decode(&raw, 0.0).unwrap();
        assert_eq!((e.id, e.ext), (0x18DA_F110, true));
        raw[..4].copy_from_slice(&(0x123u32 | CAN_ERR_FLAG).to_ne_bytes());
        assert!(decode(&raw, 0.0).is_none(), "error frames are not data");
        assert_eq!(
            std::mem::size_of::<SockaddrCan>(),
            24,
            "the kernel's struct sockaddr_can"
        );
        assert!(open("no-such-can9")
            .unwrap_err()
            .to_string()
            .contains("no network interface"));
    }
}
