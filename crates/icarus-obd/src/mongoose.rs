//! The Drewtech (Drew Technologies) Mongoose Pro, a J2534 cable, driven by Icarus itself: no Windows library. The cable is a
//! virtual serial port to Linux (115200 baud) speaking a framed binary protocol, and on that protocol it is a raw CAN device:
//! frames go out one at a time, and what it receives comes as unsolicited pushes once a pass filter is set. So `Mongoose` is
//! a `canlink::Frames`, and Icarus's own ISO-TP (`canlink::CanLink`: flow control, "response pending", the broadcast)
//! runs over it (`--adapter mongoose:/dev/ttyACM0`).
//!
//! The protocol was learned from roffe/gocan's native Mongoose Pro GM II driver (github.com/roffe/gocan, with thanks; credited
//! in docs/REFERENCES.md), which decoded it from the cable's Windows library. Nothing was copied: the code here is written for
//! Icarus. Only the Mongoose Pro GM II is known to speak it; the other Mongoose Pro models (the Toyota one Techstream uses,
//! Ford, Honda) are expected to, unchecked. Not yet tried on a cable in hand: the tests use a stand-in that answers as gocan
//! describes the cable, in front of the simulated car on `canlink::SimBus`.
//!
//! A packet on the wire, little-endian unless said:
//!
//! | Bytes | |
//! | --- | --- |
//! | 0-1 | length: what follows byte 3 |
//! | 2-3 | check: length XOR 0x51E6 |
//! | 4-7 | from Icarus: 01, the channel, 00 00; from the cable: 00, 00, its board, the channel |
//! | 8 | the command |
//! | 9 | 01 or 00 (a request, or its last part); a reply sets 0x80 |
//! | 10-11 | the sequence number (1-255), which the reply carries back |
//! | 16-19 | in a reply, its status (0: done), and after it any error text |
//!
//! Commands Icarus sends: 00 (wake), 09 (board info, with the firmware version at 36-39), 0C (a setting, read), 11 (status),
//! 03 (to the firmware; then with 00, open: status 0x20A is "no battery voltage on pin 16"), 06 (connect at a bit rate),
//! 12 (CAN), 0D (a pass filter: with none the cable passes nothing on), 08 (send a frame: the ID big-endian at 28-31, the
//! data from 32; the reply's status 0x100 is queued, 0x101 the cable's queue is full), 07 (disconnect), 05 (close).
//! Received frames are 09 pushes with 00 in byte 9: the ID big-endian at 28-31, the data from 32, its length plus four at
//! 26-27. 0A pushes say how a sent frame went.

use crate::bus::Frame;
use crate::canlink::Frames;
use crate::elm::ObdError;
use crate::port::Port;
use std::collections::VecDeque;
use std::io;
use std::time::{Duration, Instant};

pub const NAME: &str = "Drewtech Mongoose";

const CHECK: u16 = 0x51E6;
/// How long the cable has to answer a command.
const ANSWER: Duration = Duration::from_secs(2);
/// The wake-up the Windows library starts with goes unanswered once the firmware runs: it waits this long.
const WAKE: Duration = Duration::from_millis(250);
/// The CAN channel's number on the wire.
const CAN: u8 = 5;

const WAKE_UP: u8 = 0x00;
const FIRMWARE: u8 = 0x03;
const CLOSE: u8 = 0x05;
const CONNECT: u8 = 0x06;
const DISCONNECT: u8 = 0x07;
const SEND: u8 = 0x08;
const RECEIVED: u8 = 0x09;
const SENT: u8 = 0x0A;
const SETTING: u8 = 0x0C;
const FILTER: u8 = 0x0D;
const STATUS: u8 = 0x11;
const PROTOCOL: u8 = 0x12;

/// A send's reply status: queued, or the cable's queue is full (tried again for up to a second); a bus fault is waited out.
const QUEUED: u32 = 0x100;
const QUEUE_FULL: u32 = 0x101;
const BUS_FAULT: u32 = 0x105;
/// The open's status when pin 16 has no battery voltage.
const NO_BATTERY: u32 = 0x20A;

/// One packet from the wire.
#[derive(Debug, PartialEq, Eq)]
struct Packet {
    /// Sent by Icarus (01 in byte 4), not by the cable.
    out: bool,
    channel: u8,
    /// From byte 8 on.
    body: Vec<u8>,
}

impl Packet {
    fn command(&self) -> u8 {
        self.body[0]
    }
    fn is_reply(&self) -> bool {
        self.body[1] & 0x80 != 0
    }
    fn seq(&self) -> u8 {
        self.body[2]
    }
    fn word(&self, at: usize) -> u32 {
        self.body
            .get(at..at + 4)
            .map_or(0, |b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }
    /// A reply's status (body bytes 8-11).
    fn status(&self) -> u32 {
        self.word(8)
    }
    /// The text after a reply's status, up to its NUL.
    fn text(&self, from: usize) -> String {
        let t = self.body.get(from..).unwrap_or_default();
        let end = t.iter().position(|&b| b == 0).unwrap_or(t.len());
        String::from_utf8_lossy(&t[..end]).trim().to_string()
    }
    /// A received frame: (ID, data).
    fn frame(&self) -> Option<(u32, Vec<u8>)> {
        let b = &self.body;
        if b.len() < 24 {
            return None;
        }
        let n = (u16::from_le_bytes([b[18], b[19]]) as usize).saturating_sub(4);
        let id = u32::from_be_bytes([b[20], b[21], b[22], b[23]]) & 0x1FFF_FFFF;
        let data = b[24..].iter().take(n.min(8)).copied().collect();
        Some((id, data))
    }
}

/// The wire bytes of a packet from Icarus on `channel`.
fn wire(channel: u8, body: &[u8]) -> Vec<u8> {
    let len = (body.len() + 4) as u16;
    let mut p = Vec::with_capacity(body.len() + 8);
    p.extend_from_slice(&len.to_le_bytes());
    p.extend_from_slice(&(len ^ CHECK).to_le_bytes());
    p.extend_from_slice(&[0x01, channel, 0, 0]);
    p.extend_from_slice(body);
    p
}

/// The next whole packet at the front of `buf`, taken out of it. A byte that cannot start one (its check does not match, or
/// the length is too short for any packet) is dropped, so a stream joined mid-packet finds its way back.
fn next_packet(buf: &mut Vec<u8>) -> Option<Packet> {
    loop {
        if buf.len() < 4 {
            return None;
        }
        let len = u16::from_le_bytes([buf[0], buf[1]]);
        let check = u16::from_le_bytes([buf[2], buf[3]]);
        if check != len ^ CHECK || len < 12 {
            buf.drain(..1);
            continue;
        }
        let total = len as usize + 4;
        if buf.len() < total {
            return None;
        }
        let p: Vec<u8> = buf.drain(..total).collect();
        let out = p[4] == 0x01;
        return Some(Packet {
            out,
            channel: if out { p[5] } else { p[7] },
            body: p[8..].to_vec(),
        });
    }
}

/// A Mongoose Pro behind a byte stream: its serial port (`open_tty`), or a stand-in in tests.
pub struct Mongoose<P: Port> {
    port: P,
    buf: Vec<u8>,
    seq: u8,
    /// Frames received and not read yet.
    rx: VecDeque<Frame>,
    /// Send replies by sequence number, not waited for yet.
    sent: VecDeque<(u8, u32)>,
    firmware: String,
    started: Instant,
    closed: bool,
}

/// Frames kept for `recv` at most.
const RX: usize = 8192;

/// Opens the cable on its serial port and connects CAN at `bitrate`.
pub fn open_tty(path: &str, bitrate: u32) -> Result<Mongoose<crate::port::FdPort>, ObdError> {
    Mongoose::open(
        crate::port::open_serial(path, 115_200).map_err(ObdError::Io)?,
        bitrate,
    )
}

impl<P: Port> Mongoose<P> {
    /// Opens the cable as its Windows library does (wake-up, board info, a setting, status on newer firmware, the jump to the
    /// firmware, open), connects CAN at `bitrate` and sets one pass filter for every ID (Icarus picks out the answers).
    pub fn open(port: P, bitrate: u32) -> Result<Self, ObdError> {
        let mut m = Mongoose {
            port,
            buf: Vec::new(),
            seq: 0,
            rx: VecDeque::new(),
            sent: VecDeque::new(),
            firmware: String::new(),
            started: Instant::now(),
            closed: false,
        };
        let _ = m.ask(0, WAKE_UP, 0x01, &[], WAKE);
        // anything that came before is from an earlier program
        m.buf.clear();
        m.rx.clear();
        let info = m.ask(0, RECEIVED, 0x01, &[], ANSWER)?;
        // four bytes, the last part of the version first
        let fw = info.body.get(28..32).map(|v| [v[3], v[2], v[1], v[0]]);
        m.firmware = fw
            .map(|v| format!("{}.{}.{}.{}", v[0], v[1], v[2], v[3]))
            .unwrap_or_default();
        let _ = m.ask(0, SETTING, 0x00, &0x2Fu32.to_le_bytes(), ANSWER);
        if fw.is_some_and(|v| v >= [1, 1, 4, 0]) {
            let _ = m.ask(0, STATUS, 0x01, &[], ANSWER);
        }
        m.ask(0, FIRMWARE, 0x01, &[], ANSWER)?;
        let open = m.ask(0, FIRMWARE, 0x00, &[], ANSWER)?;
        if open.status() == NO_BATTERY {
            return Err(ObdError::NoConnection(format!(
                "the {NAME} sees no battery voltage at the OBD port (pin 16): is it plugged into the vehicle, ignition on?"
            )));
        }
        m.done(&open, "open")?;
        let _ = m.ask(0, RECEIVED, 0x01, &[], ANSWER);
        let mut c = [0u8; 8];
        c[4..].copy_from_slice(&bitrate.to_le_bytes());
        let r = m.ask(CAN, CONNECT, 0x00, &c, ANSWER)?;
        m.done(&r, &format!("CAN at {bitrate} bit/s"))?;
        let mut p = [0u8; 12];
        p[..4].copy_from_slice(&1u32.to_le_bytes());
        p[4..8].copy_from_slice(&6u32.to_le_bytes());
        p[8..].copy_from_slice(&0x0Eu32.to_le_bytes());
        let r = m.ask(CAN, PROTOCOL, 0x00, &p, ANSWER)?;
        m.done(&r, "the CAN protocol")?;
        // a pass filter with mask 0: every ID
        let mut f = [0u8; 16];
        f[6] = 0x01;
        f[7] = 0x04;
        let r = m.ask(CAN, FILTER, 0x00, &f, ANSWER)?;
        m.done(&r, "a pass filter")?;
        Ok(m)
    }

    /// "Drewtech Mongoose" and its firmware version.
    pub fn label(&self) -> String {
        if self.firmware.is_empty() {
            NAME.into()
        } else {
            format!("{NAME} {}", self.firmware)
        }
    }

    fn next_seq(&mut self) -> u8 {
        self.seq = self.seq % 255 + 1;
        self.seq
    }

    /// Sends a command (`rest` follows the body's first eight bytes: command, flag, sequence, zeros) and waits up to
    /// `timeout` for its reply. Frames that come meanwhile are kept for `recv`.
    fn ask(
        &mut self,
        channel: u8,
        command: u8,
        flag: u8,
        rest: &[u8],
        timeout: Duration,
    ) -> Result<Packet, ObdError> {
        let seq = self.next_seq();
        let mut body = vec![command, flag, seq, 0, 0, 0, 0, 0];
        body.extend_from_slice(rest);
        self.port
            .send(&wire(channel, &body))
            .map_err(ObdError::Io)?;
        let deadline = Instant::now() + timeout;
        loop {
            while let Some(p) = next_packet(&mut self.buf) {
                if let Some(r) = self.take(p) {
                    if r.command() == command && r.seq() == seq {
                        return Ok(r);
                    }
                }
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Err(ObdError::Io(io::Error::new(
                    io::ErrorKind::TimedOut,
                    format!("the {NAME} did not answer command {command:02X} in time (is it plugged in, and is this its port?)"),
                )));
            }
            self.fill(left).map_err(ObdError::Io)?;
        }
    }

    /// A reply's status as a result.
    fn done(&self, r: &Packet, what: &str) -> Result<(), ObdError> {
        match r.status() {
            0 => Ok(()),
            s => Err(ObdError::Adapter(format!(
                "the {NAME} refused {what}: {} (0x{s:X})",
                r.text(16)
            ))),
        }
    }

    fn fill(&mut self, timeout: Duration) -> io::Result<()> {
        let mut b = [0u8; 1024];
        let n = self.port.recv(&mut b, timeout)?;
        self.buf.extend_from_slice(&b[..n]);
        Ok(())
    }

    /// A packet from the cable: a received frame is kept, a send's reply noted; a command's reply is handed back.
    fn take(&mut self, p: Packet) -> Option<Packet> {
        if p.out || p.body.len() < 4 {
            return None;
        }
        match (p.command(), p.is_reply()) {
            (RECEIVED, false) => {
                if let Some((id, data)) = p.frame() {
                    if self.rx.len() >= RX {
                        self.rx.pop_front();
                    }
                    self.rx.push_back(Frame {
                        id,
                        ext: id > 0x7FF,
                        data,
                        at: self.started.elapsed().as_secs_f64(),
                    });
                }
                None
            }
            (SEND, _) => {
                self.sent.push_back((p.seq(), p.status()));
                if self.sent.len() > 64 {
                    self.sent.pop_front();
                }
                None
            }
            (SENT, _) => None,
            _ => Some(p),
        }
    }

    /// Waits for the reply to the send with sequence number `seq`.
    fn send_reply(&mut self, seq: u8) -> io::Result<u32> {
        let deadline = Instant::now() + ANSWER;
        loop {
            while let Some(p) = next_packet(&mut self.buf) {
                self.take(p);
            }
            if let Some(i) = self.sent.iter().position(|(s, _)| *s == seq) {
                let (_, code) = self.sent.remove(i).unwrap_or_default();
                if code != BUS_FAULT {
                    return Ok(code);
                }
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    format!("the {NAME} did not take a frame in time"),
                ));
            }
            self.fill(left)?;
        }
    }
}

impl<P: Port> Frames for Mongoose<P> {
    fn send(&mut self, f: &Frame) -> io::Result<()> {
        if f.ext {
            return Err(io::Error::new(
                io::ErrorKind::Unsupported,
                format!("the {NAME} driver sends 11-bit CAN frames only (how it marks a 29-bit one is not learned yet)"),
            ));
        }
        if f.data.len() > 8 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "a CAN frame holds 8 bytes at most",
            ));
        }
        let give_up = Instant::now() + Duration::from_secs(1);
        loop {
            let seq = self.next_seq();
            // the Windows library's fixed 32-byte send
            let mut body = [0u8; 32];
            body[0] = SEND;
            body[2] = seq;
            body[4] = 0x01;
            body[16..18].copy_from_slice(&((4 + f.data.len()) as u16).to_le_bytes());
            body[20..24].copy_from_slice(&f.id.to_be_bytes());
            body[24..24 + f.data.len()].copy_from_slice(&f.data);
            self.port.send(&wire(CAN, &body))?;
            match self.send_reply(seq)? {
                QUEUED => return Ok(()),
                QUEUE_FULL if Instant::now() < give_up => {
                    std::thread::sleep(Duration::from_millis(1))
                }
                QUEUE_FULL => {
                    return Err(io::Error::other(format!(
                        "the {NAME}'s send queue stayed full"
                    )))
                }
                code => {
                    return Err(io::Error::other(format!(
                        "the {NAME} refused a frame (0x{code:X})"
                    )))
                }
            }
        }
    }

    fn recv(&mut self, timeout: Duration) -> io::Result<Option<Frame>> {
        let deadline = Instant::now() + timeout;
        loop {
            while let Some(p) = next_packet(&mut self.buf) {
                self.take(p);
            }
            if let Some(f) = self.rx.pop_front() {
                return Ok(Some(f));
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Ok(None);
            }
            self.fill(left)?;
        }
    }

    /// The serial port the cable was opened on, judged by its own device (`Port::wired`).
    fn wired(&self) -> bool {
        self.port.wired()
    }
}

impl<P: Port> Drop for Mongoose<P> {
    /// Disconnects CAN and closes the cable, without waiting for its answers.
    fn drop(&mut self) {
        if !self.closed {
            self.closed = true;
            for (ch, cmd) in [(CAN, DISCONNECT), (0, CLOSE)] {
                let seq = self.next_seq();
                let _ = self.port.send(&wire(ch, &[cmd, 0x00, seq, 0, 0, 0, 0, 0]));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::canlink::{CanLink, SimBus};
    use crate::link::Link;
    use crate::sim::SimCar;

    /// A packet as the cable sends it (board 1).
    fn from_cable(channel: u8, body: &[u8]) -> Vec<u8> {
        let len = (body.len() + 4) as u16;
        let mut p = len.to_le_bytes().to_vec();
        p.extend_from_slice(&(len ^ CHECK).to_le_bytes());
        p.extend_from_slice(&[0x00, 0x00, 0x01, channel]);
        p.extend_from_slice(body);
        p
    }

    /// A stand-in Mongoose Pro in front of the simulated car on a CAN bus (`SimBus`), answering as gocan describes the
    /// cable: no answer to the wake-up, the firmware version in the board info, a send's 08 reply and 0A push, every frame
    /// from the car as a 09 push once the pass filter is set. What it sends back is handed over a few bytes at a time.
    struct Cable {
        bus: SimBus,
        out: VecDeque<u8>,
        commands: Vec<u8>,
        battery: bool,
        filter: bool,
        bitrate: u32,
        /// The port is on a USB cable (`Port::wired`).
        wired: bool,
    }

    impl Cable {
        fn new() -> Cable {
            Cable {
                bus: SimBus::new(SimCar::new()),
                // half a packet left over from an earlier program
                out: [0x24, 0x00, 0xC2, 0x51, 0x00, 0x00].into_iter().collect(),
                commands: Vec::new(),
                battery: true,
                filter: false,
                bitrate: 0,
                wired: false,
            }
        }

        fn reply(&mut self, p: &Packet, status: u32, extra: &[u8]) {
            let mut b = vec![0u8; 24];
            b[0] = p.command();
            b[1] = p.body[1] | 0x80;
            b[2] = p.seq();
            b[8..12].copy_from_slice(&status.to_le_bytes());
            b[12..12 + extra.len().min(12)].copy_from_slice(&extra[..extra.len().min(12)]);
            if extra.len() > 12 {
                b.extend_from_slice(&extra[12..]);
            }
            self.out.extend(from_cable(p.channel, &b));
        }
    }

    impl Port for Cable {
        fn send(&mut self, data: &[u8]) -> io::Result<()> {
            let mut buf = data.to_vec();
            while let Some(p) = next_packet(&mut buf) {
                assert!(p.out, "Icarus's packets say they are Icarus's");
                self.commands.push(p.command());
                match p.command() {
                    WAKE_UP => {}
                    RECEIVED => {
                        // board info: the firmware 1.2.4.0 (last part first) at body bytes 28-31
                        let mut x = vec![0u8; 24];
                        x[16..20].copy_from_slice(&[0, 4, 2, 1]);
                        self.reply(&p, 0, &x);
                    }
                    FIRMWARE if p.body[1] == 0x01 => self.reply(&p, 0, b"1.2.4.0\0"),
                    FIRMWARE => {
                        let s = if self.battery { 0 } else { NO_BATTERY };
                        self.reply(&p, s, &[]);
                    }
                    CONNECT => {
                        self.bitrate = p.word(12);
                        self.reply(&p, 0, &[]);
                    }
                    FILTER => {
                        self.filter = p.word(16) == 0 && p.body[14] == 0x01;
                        self.reply(&p, 0, &[0, 0, 0, 0, 0x34, 0x12, 0, 0]);
                    }
                    SEND => {
                        // a send's length is at body bytes 16-17 (a received frame's at 18-19)
                        let n = u16::from_le_bytes([p.body[16], p.body[17]]) as usize - 4;
                        let id =
                            u32::from_be_bytes([p.body[20], p.body[21], p.body[22], p.body[23]]);
                        let data = p.body[24..24 + n].to_vec();
                        let mut r = vec![0u8; 12];
                        r[0] = SEND;
                        r[1] = 0x80;
                        r[2] = p.seq();
                        r[8..12].copy_from_slice(&QUEUED.to_le_bytes());
                        self.out.extend(from_cable(CAN, &r));
                        r[0] = SENT;
                        self.out.extend(from_cable(CAN, &r));
                        self.bus
                            .send(&Frame {
                                id,
                                ext: false,
                                data,
                                at: 0.0,
                            })
                            .unwrap();
                    }
                    _ => self.reply(&p, 0, &[]),
                }
            }
            Ok(())
        }

        fn recv(&mut self, buf: &mut [u8], timeout: Duration) -> io::Result<usize> {
            if self.out.is_empty() && self.filter {
                while let Some(f) = self.bus.recv(Duration::ZERO).unwrap() {
                    let mut b = vec![0u8; 24];
                    b[0] = RECEIVED;
                    b[12..16].copy_from_slice(&0x000D_37FCu32.to_le_bytes());
                    b[18..20].copy_from_slice(&((4 + f.data.len()) as u16).to_le_bytes());
                    b[20..24].copy_from_slice(&f.id.to_be_bytes());
                    b.extend_from_slice(&f.data);
                    self.out.extend(from_cable(CAN, &b));
                }
            }
            if self.out.is_empty() {
                std::thread::sleep(timeout.min(Duration::from_millis(2)));
                return Ok(0);
            }
            let n = self.out.len().min(7).min(buf.len());
            for (i, b) in self.out.drain(..n).enumerate() {
                buf[i] = b;
            }
            Ok(n)
        }

        fn wired(&self) -> bool {
            self.wired
        }
    }

    /// ICA-206: a Mongoose link is as wired as the serial port the cable was opened on.
    #[test]
    fn a_mongoose_link_is_judged_by_its_port() {
        for on_usb in [true, false] {
            let mut cable = Cable::new();
            cable.wired = on_usb;
            let l =
                CanLink::connect(Mongoose::open(cable, 500_000).unwrap(), "/dev/ttyACM0").unwrap();
            assert_eq!(l.wired(), on_usb);
        }
    }

    /// A received frame laid out as the module doc says (written for this test): the engine's speed answer, 7E8 03 41 0D 2A.
    #[test]
    fn a_received_frame_is_read() {
        let mut body = vec![RECEIVED, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        body.extend_from_slice(&0x0001_86A0u32.to_le_bytes()); // timestamp
        body.extend_from_slice(&[0, 0]);
        body.extend_from_slice(&(4u16 + 5).to_le_bytes()); // the ID's four bytes and five of data
        body.extend_from_slice(&0x7E8u32.to_be_bytes());
        body.extend_from_slice(&[0x03, 0x41, 0x0D, 0x2A, 0x00]);
        let packet = from_cable(CAN, &body);
        // the length (what follows byte 3) and its check, length XOR 0x51E6
        assert_eq!(&packet[..4], &[0x21, 0x00, 0xC7, 0x51]);
        let mut buf = vec![0x51, 0xE6, 0x07]; // noise first
        buf.extend_from_slice(&packet);
        let p = next_packet(&mut buf).unwrap();
        assert!(!p.out && p.channel == 5 && p.command() == RECEIVED && !p.is_reply());
        assert_eq!(p.frame(), Some((0x7E8, vec![0x03, 0x41, 0x0D, 0x2A, 0x00])));
        assert!(buf.is_empty());
        // what Icarus sends reads back the same
        let mut w = wire(CAN, &[CONNECT, 0, 7, 0, 0, 0, 0, 0]);
        let q = next_packet(&mut w).unwrap();
        assert_eq!(
            (q.out, q.channel, q.command(), q.seq()),
            (true, 5, CONNECT, 7)
        );
    }

    #[test]
    fn obd_and_one_module_over_a_mongoose() {
        let m = Mongoose::open(Cable::new(), 500_000).unwrap();
        assert_eq!(m.label(), "Drewtech Mongoose 1.2.4.0");
        assert_eq!(m.port.bitrate, 500_000);
        let mut l = CanLink::connect(m, "/dev/ttyACM0").unwrap().named(NAME);
        assert_eq!(l.adapter(), "Drewtech Mongoose");
        // a long answer, with Icarus's flow control going back through the cable
        assert_eq!(
            crate::j1979::vin(&mut l).unwrap().as_deref(),
            Some(crate::sim::VIN)
        );
        let m = l.request_to(0x7C0, 0x7C8, &[0x19, 0x02, 0xFF]).unwrap();
        assert_eq!(
            m[0].data[0], 0x59,
            "the pending answer is waited out: {m:?}"
        );
    }

    #[test]
    fn no_battery_voltage_is_said() {
        let mut c = Cable::new();
        c.battery = false;
        let e = Mongoose::open(c, 500_000).err().unwrap();
        assert!(e.to_string().contains("no battery voltage"), "{e}");
    }

    #[test]
    fn the_open_is_the_windows_librarys_and_29_bit_is_refused() {
        let mut m = Mongoose::open(Cable::new(), 500_000).unwrap();
        assert_eq!(
            m.port.commands,
            [
                WAKE_UP, RECEIVED, SETTING, STATUS, FIRMWARE, FIRMWARE, RECEIVED, CONNECT,
                PROTOCOL, FILTER
            ]
        );
        let e = m
            .send(&Frame {
                id: 0x18DB_33F1,
                ext: true,
                data: vec![0x02, 0x01, 0x00],
                at: 0.0,
            })
            .unwrap_err();
        assert_eq!(e.kind(), io::ErrorKind::Unsupported);
        assert!(m.recv(Duration::ZERO).unwrap().is_none());
    }
}
