//! Linux's own CAN interfaces (SocketCAN): USB CAN adapters such as a CANable show up as `can0`, set up once with
//! `sudo ip link set can0 up type can bitrate 500000`. The bus monitor only reads frames from them (it never writes to the
//! socket, like the adapter's monitor mode); diagnostics (`canlink`, `--adapter can:can0`) send requests through them.

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

/// The kernel's `struct can_frame` for a classic data frame (at most 8 bytes; a 29-bit ID when `f.ext`).
pub fn encode(f: &Frame) -> io::Result<[u8; 16]> {
    let max = if f.ext { 0x1FFF_FFFF } else { 0x7FF };
    if f.id > max || f.data.len() > 8 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            format!("not a CAN frame: ID {:X}, {} bytes", f.id, f.data.len()),
        ));
    }
    let mut raw = [0u8; 16];
    let id = if f.ext { f.id | CAN_EFF_FLAG } else { f.id };
    raw[..4].copy_from_slice(&id.to_ne_bytes());
    raw[4] = f.data.len() as u8;
    raw[8..8 + f.data.len()].copy_from_slice(&f.data);
    Ok(raw)
}

impl CanSocket {
    /// The next frame, waiting up to `timeout` (None: none came, or it was not a data frame).
    pub fn read(&self, timeout: Duration, at: f64) -> io::Result<Option<Frame>> {
        Ok(self.read_raw(timeout)?.and_then(|raw| decode(&raw, at)))
    }

    /// The next frame as the kernel gives it, waiting up to `timeout`.
    fn read_raw(&self, timeout: Duration) -> io::Result<Option<[u8; 16]>> {
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
        Ok((n as usize == raw.len()).then_some(raw))
    }
}

/// A CAN interface carries the diagnostic link's frames (`canlink`): every frame written is one `struct can_frame`.
impl crate::canlink::Frames for CanSocket {
    fn send(&mut self, f: &Frame) -> io::Result<()> {
        let raw = encode(f)?;
        let n = unsafe { libc::write(self.0.as_raw_fd(), raw.as_ptr().cast(), raw.len()) };
        if n < 0 {
            return Err(io::Error::last_os_error());
        }
        if n as usize != raw.len() {
            return Err(io::Error::new(
                io::ErrorKind::WriteZero,
                "the CAN interface took part of a frame",
            ));
        }
        Ok(())
    }

    fn recv(&mut self, timeout: Duration) -> io::Result<Option<Frame>> {
        // remote and error frames are skipped without ending the wait
        let deadline = std::time::Instant::now() + timeout;
        loop {
            let left = deadline.saturating_duration_since(std::time::Instant::now());
            let Some(raw) = self.read_raw(left)? else {
                return Ok(None);
            };
            if let Some(f) = decode(&raw, 0.0) {
                return Ok(Some(f));
            }
            if left.is_zero() {
                return Ok(None);
            }
        }
    }
}

// rtnetlink (linux/rtnetlink.h, if_link.h, can/netlink.h): the link's CAN bit timing
const RTM_NEWLINK: u16 = 16;
const RTM_GETLINK: u16 = 18;
const NLM_F_REQUEST: u16 = 1;
const NLMSG_HDR: usize = 16;
const IFINFOMSG: usize = 16;
const IFLA_LINKINFO: u16 = 18;
const IFLA_INFO_DATA: u16 = 2;
const IFLA_CAN_BITTIMING: u16 = 1;

/// The bit rate an interface was set up with (`ip link set can0 type can bitrate N`), as the kernel tells it over rtnetlink
/// (ICA-68). None for an interface without CAN bit timing (vcan) or when the kernel does not answer.
pub fn bitrate(ifname: &str) -> Option<u32> {
    bitrate_in(&getlink(ifname)?)
}

/// The kernel's answer to RTM_GETLINK for one interface (read only: nothing is sent on the interface).
fn getlink(ifname: &str) -> Option<Vec<u8>> {
    let name = std::ffi::CString::new(ifname).ok()?;
    let index = unsafe { libc::if_nametoindex(name.as_ptr()) };
    if index == 0 {
        return None;
    }
    let fd = unsafe {
        libc::socket(
            libc::AF_NETLINK,
            libc::SOCK_RAW | libc::SOCK_CLOEXEC,
            libc::NETLINK_ROUTE,
        )
    };
    if fd < 0 {
        return None;
    }
    let fd = unsafe { OwnedFd::from_raw_fd(fd) };
    let mut req = Vec::with_capacity(NLMSG_HDR + IFINFOMSG);
    req.extend_from_slice(&((NLMSG_HDR + IFINFOMSG) as u32).to_ne_bytes());
    req.extend_from_slice(&RTM_GETLINK.to_ne_bytes());
    req.extend_from_slice(&NLM_F_REQUEST.to_ne_bytes());
    req.extend_from_slice(&1u32.to_ne_bytes()); // sequence
    req.extend_from_slice(&0u32.to_ne_bytes()); // port: the kernel's
    req.extend_from_slice(&[libc::AF_UNSPEC as u8, 0, 0, 0]); // family, pad, type
    req.extend_from_slice(&(index as i32).to_ne_bytes());
    req.extend_from_slice(&[0; 8]); // flags, change
    let sent = unsafe { libc::send(fd.as_raw_fd(), req.as_ptr().cast(), req.len(), 0) };
    if sent != req.len() as isize {
        return None;
    }
    let mut pfd = libc::pollfd {
        fd: fd.as_raw_fd(),
        events: libc::POLLIN,
        revents: 0,
    };
    if unsafe { libc::poll(&mut pfd, 1, 1000) } <= 0 {
        return None;
    }
    let mut buf = vec![0u8; 32 << 10];
    let n = unsafe { libc::recv(fd.as_raw_fd(), buf.as_mut_ptr().cast(), buf.len(), 0) };
    if n <= 0 {
        return None;
    }
    buf.truncate(n as usize);
    Some(buf)
}

/// The attributes in `b`: (type without the nested and byte-order flags, payload).
fn attributes(mut b: &[u8]) -> Vec<(u16, &[u8])> {
    let mut out = Vec::new();
    while b.len() >= 4 {
        let len = u16::from_ne_bytes([b[0], b[1]]) as usize;
        let kind = u16::from_ne_bytes([b[2], b[3]]) & 0x3fff;
        if len < 4 || len > b.len() {
            break;
        }
        out.push((kind, &b[4..len]));
        b = &b[((len + 3) & !3).min(b.len())..];
    }
    out
}

/// The bit rate in the kernel's answer to RTM_GETLINK: the link's IFLA_LINKINFO, its IFLA_INFO_DATA, its
/// IFLA_CAN_BITTIMING, whose first field is the bit rate. None for anything else (an error, another kind of link, vcan).
pub fn bitrate_in(answer: &[u8]) -> Option<u32> {
    if answer.len() < NLMSG_HDR + IFINFOMSG {
        return None;
    }
    let len = (u32::from_ne_bytes(answer[0..4].try_into().ok()?) as usize).min(answer.len());
    if u16::from_ne_bytes([answer[4], answer[5]]) != RTM_NEWLINK || len < NLMSG_HDR + IFINFOMSG {
        return None;
    }
    let link = attributes(&answer[NLMSG_HDR + IFINFOMSG..len]);
    let info = link.iter().find(|(k, _)| *k == IFLA_LINKINFO)?.1;
    let data = attributes(info)
        .into_iter()
        .find(|(k, _)| *k == IFLA_INFO_DATA)?
        .1;
    let timing = attributes(data)
        .into_iter()
        .find(|(k, _)| *k == IFLA_CAN_BITTIMING)?
        .1;
    let rate = u32::from_ne_bytes(timing.get(0..4)?.try_into().ok()?);
    (rate > 0).then_some(rate)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One attribute, padded to four bytes as the kernel writes it.
    fn attr(kind: u16, payload: &[u8]) -> Vec<u8> {
        let mut a = ((4 + payload.len()) as u16).to_ne_bytes().to_vec();
        a.extend_from_slice(&kind.to_ne_bytes());
        a.extend_from_slice(payload);
        while !a.len().is_multiple_of(4) {
            a.push(0);
        }
        a
    }

    /// The kernel's answer to RTM_GETLINK for a link with these attributes.
    fn answer(kind: u16, attrs: &[u8]) -> Vec<u8> {
        let mut m = ((NLMSG_HDR + IFINFOMSG + attrs.len()) as u32)
            .to_ne_bytes()
            .to_vec();
        m.extend_from_slice(&kind.to_ne_bytes());
        m.extend_from_slice(&[0; 10]);
        m.extend_from_slice(&[0; IFINFOMSG]);
        m.extend_from_slice(attrs);
        m
    }

    /// ICA-68: the bit rate is read out of a parsed netlink answer as the kernel lays it out: IFLA_IFNAME and IFLA_MTU
    /// first, then IFLA_LINKINFO (nested) with IFLA_INFO_KIND "can" and IFLA_INFO_DATA (nested) holding the state and the
    /// bit timing. A vcan link (no data), an error message and a cut answer give none.
    #[test]
    fn the_bit_rate_from_the_kernels_answer() {
        let mut timing = 250_000u32.to_ne_bytes().to_vec();
        timing.extend_from_slice(&[0; 28]); // sample point, tq, segments, sjw, brp
        let data = [
            attr(2, &0u32.to_ne_bytes()),
            attr(IFLA_CAN_BITTIMING, &timing),
        ]
        .concat();
        let info = [attr(1, b"can\0"), attr(IFLA_INFO_DATA | 0x8000, &data)].concat();
        let link = [
            attr(3, b"can0\0"),
            attr(4, &16u32.to_ne_bytes()),
            attr(IFLA_LINKINFO | 0x8000, &info),
        ]
        .concat();
        let a = answer(RTM_NEWLINK, &link);
        assert_eq!(bitrate_in(&a), Some(250_000));
        let vcan = answer(
            RTM_NEWLINK,
            &[
                attr(3, b"vcan0\0"),
                attr(IFLA_LINKINFO, &attr(1, b"vcan\0")),
            ]
            .concat(),
        );
        assert_eq!(bitrate_in(&vcan), None);
        assert_eq!(bitrate_in(&answer(2, &link)), None, "an error message");
        assert_eq!(bitrate_in(&a[..a.len() - 20]), None, "cut short");
        assert_eq!(bitrate_in(&[]), None);
        // a made-up length longer than the answer does not read past it
        let mut long = a.clone();
        long[..4].copy_from_slice(&9999u32.to_ne_bytes());
        assert_eq!(bitrate_in(&long), Some(250_000));
    }

    /// The request is one the kernel answers: the loopback interface's link (read only, nothing sent on any network) comes
    /// back as RTM_NEWLINK with its name, and has no CAN bit timing; an interface that does not exist gives none.
    #[test]
    fn the_kernel_answers_the_request() {
        let a = getlink("lo").expect("an answer for lo");
        assert_eq!(u16::from_ne_bytes([a[4], a[5]]), RTM_NEWLINK);
        let len = (u32::from_ne_bytes(a[0..4].try_into().unwrap()) as usize).min(a.len());
        let name = attributes(&a[NLMSG_HDR + IFINFOMSG..len])
            .into_iter()
            .find(|(k, _)| *k == 3)
            .map(|(_, v)| v.to_vec());
        assert_eq!(name.as_deref(), Some(&b"lo\0"[..]));
        assert_eq!(bitrate("lo"), None);
        assert_eq!(bitrate("no-such-can9"), None);
    }

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
        // what Icarus writes reads back the same; IDs and lengths a classic frame cannot carry are refused
        for f in [
            Frame {
                id: 0x7DF,
                ext: false,
                data: vec![0x02, 0x01, 0x00, 0, 0, 0, 0, 0],
                at: 0.0,
            },
            Frame {
                id: 0x18DA_10F1,
                ext: true,
                data: vec![0x03, 0x22, 0xF1, 0x87],
                at: 0.0,
            },
        ] {
            assert_eq!(decode(&encode(&f).unwrap(), 0.0).unwrap(), f);
        }
        let bad = |id, ext, n| {
            encode(&Frame {
                id,
                ext,
                data: vec![0; n],
                at: 0.0,
            })
            .is_err()
        };
        assert!(bad(0x800, false, 1) && bad(0x2000_0000, true, 1) && bad(0x7E0, false, 9));
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
