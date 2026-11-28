//! Small captures written for the tests: HCI packets as a phone would log them, wrapped as pcap or PacketLogger files.
//! Every address, name and byte here is made up.

use super::btlog::fcs;

/// What a logged packet is.
#[derive(Clone, Debug)]
pub enum Kind {
    /// ACL data (an HCI ACL packet: handle and flags, length, data).
    Acl,
    /// An HCI event (never read by Icarus).
    Event,
}

#[derive(Clone, Debug)]
pub struct Pkt {
    pub clock: f64,
    /// Sent by the phone (else received).
    pub sent: bool,
    pub kind: Kind,
    pub bytes: Vec<u8>,
}

/// An RFCOMM UIH frame on `dlci`; `credit` sets the poll/final bit and puts a credit byte in.
pub fn uih(dlci: u8, cr: bool, credit: bool, data: &[u8]) -> Vec<u8> {
    let addr = (dlci << 2) | (u8::from(cr) << 1) | 1;
    let mut f = vec![addr, if credit { 0xFF } else { 0xEF }];
    if data.len() < 128 {
        f.push(((data.len() as u8) << 1) | 1);
    } else {
        f.push(((data.len() & 0x7F) as u8) << 1);
        f.push((data.len() >> 7) as u8);
    }
    if credit {
        f.push(0x05);
    }
    f.extend_from_slice(data);
    f.push(fcs(&f[..2]));
    f
}

/// An iAP2 link packet (`payload` empty: header only).
pub fn iap2_packet(seq: u8, session: u8, payload: &[u8]) -> Vec<u8> {
    let len = 9 + if payload.is_empty() {
        0
    } else {
        payload.len() + 1
    };
    let mut p = vec![
        0xFF,
        0x5A,
        (len >> 8) as u8,
        len as u8,
        0x40,
        seq,
        0,
        session,
    ];
    let sum = |b: &[u8]| b.iter().fold(0u8, |a, x| a.wrapping_add(*x));
    p.push(0u8.wrapping_sub(sum(&p)));
    if !payload.is_empty() {
        p.extend_from_slice(payload);
        p.push(0u8.wrapping_sub(sum(payload)));
    }
    p
}

/// A conversation being written: packets in time order.
pub struct Session {
    pub packets: Vec<Pkt>,
    t: f64,
}

impl Session {
    fn new() -> Session {
        Session {
            packets: vec![],
            t: 1_763_560_000.0,
        }
    }

    /// An L2CAP packet on `cid` as ACL fragments of at most `frag` bytes (not yet logged: no time).
    fn frags(sent: bool, handle: u16, cid: u16, data: &[u8], frag: usize) -> Vec<Pkt> {
        let mut pdu = (data.len() as u16).to_le_bytes().to_vec();
        pdu.extend_from_slice(&cid.to_le_bytes());
        pdu.extend_from_slice(data);
        pdu.chunks(frag.max(1))
            .enumerate()
            .map(|(i, part)| {
                let flags = if i == 0 { 0x2000 } else { 0x1000 };
                let mut acl = ((handle & 0x0FFF) | flags).to_le_bytes().to_vec();
                acl.extend_from_slice(&(part.len() as u16).to_le_bytes());
                acl.extend_from_slice(part);
                Pkt {
                    clock: 0.0,
                    sent,
                    kind: Kind::Acl,
                    bytes: acl,
                }
            })
            .collect()
    }

    /// A packet logged next.
    fn log(&mut self, mut p: Pkt) {
        self.t += 0.01;
        p.clock = self.t;
        self.packets.push(p);
    }

    /// An L2CAP packet on `cid`, cut into ACL fragments of at most `frag` bytes.
    fn l2(&mut self, sent: bool, handle: u16, cid: u16, data: &[u8], frag: usize) {
        for p in Session::frags(sent, handle, cid, data, frag) {
            self.log(p);
        }
    }

    fn event(&mut self, bytes: &[u8]) {
        self.log(Pkt {
            clock: 0.0,
            sent: false,
            kind: Kind::Event,
            bytes: bytes.to_vec(),
        });
    }

    /// An L2CAP channel set up by the phone: PSM `psm`, the phone's channel `mine`, the other side's `theirs`.
    fn connect(&mut self, handle: u16, id: u8, psm: u16, mine: u16, theirs: u16) {
        let mut req = vec![0x02, id, 4, 0];
        req.extend_from_slice(&psm.to_le_bytes());
        req.extend_from_slice(&mine.to_le_bytes());
        self.l2(true, handle, 1, &req, 64);
        let mut rsp = vec![0x03, id, 8, 0];
        for v in [theirs, mine, 0, 0] {
            rsp.extend_from_slice(&v.to_le_bytes());
        }
        self.l2(false, handle, 1, &rsp, 64);
    }

    /// The phone closes its channel `mine` (the other side's `theirs`), and the other side confirms.
    fn disconnect(&mut self, handle: u16, id: u8, mine: u16, theirs: u16) {
        let mut req = vec![0x06, id, 4, 0];
        req.extend_from_slice(&theirs.to_le_bytes());
        req.extend_from_slice(&mine.to_le_bytes());
        self.l2(true, handle, 1, &req, 64);
        let mut rsp = vec![0x07, id, 4, 0];
        rsp.extend_from_slice(&theirs.to_le_bytes());
        rsp.extend_from_slice(&mine.to_le_bytes());
        self.l2(false, handle, 1, &rsp, 64);
    }

    /// A phone's SDP query (PSM 1) on channels `sdp` (the phone's, the other side's), closed when `close`, then its
    /// conversation with an adapter on RFCOMM channels `rf` (the phone's, the other side's), all on handle 0x0B: three
    /// requests, each answered.
    pub fn sdp_then_rfcomm(sdp: (u16, u16), close: bool, rf: (u16, u16)) -> Session {
        let mut s = Session::new();
        s.connect(0x0B, 1, 1, sdp.0, sdp.1);
        // an SDP answer whose bytes are not RFCOMM's
        s.l2(
            false,
            0x0B,
            sdp.0,
            &[0x03, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00],
            64,
        );
        if close {
            s.disconnect(0x0B, 2, sdp.0, sdp.1);
        }
        s.connect(0x0B, 3, 3, rf.0, rf.1);
        for (c, a) in [
            ("ATZ\r", "ELM327 v1.4b\r\r>"),
            ("ATSH7E0\r", "OK\r\r>"),
            ("221234\r", "62 12 34 01\r\r>"),
            ("225678\r", "62 56 78 02\r\r>"),
            ("2105\r", "61 05 03\r\r>"),
        ] {
            s.l2(true, 0x0B, rf.1, &uih(2, true, false, c.as_bytes()), 64);
            s.l2(false, 0x0B, rf.0, &uih(2, false, false, a.as_bytes()), 64);
        }
        s
    }

    /// An adapter conversation from (command, answer) pairs, as `elm`'s tests write them (the carriage return and the
    /// prompt added), on RFCOMM channel 0x41/0x40 (DLCI 2), handle 0x0B; the answers cut into ACL fragments of 10 bytes.
    pub fn talk(pairs: &[(&str, &str)]) -> Session {
        let mut s = Session::new();
        s.connect(0x0B, 1, 3, 0x40, 0x41);
        for (c, a) in pairs {
            let cmd = uih(2, true, false, format!("{c}\r").as_bytes());
            s.l2(true, 0x0B, 0x41, &cmd, 64);
            let ans = uih(2, false, false, format!("{a}\r\r>").as_bytes());
            s.l2(false, 0x0B, 0x40, &ans, 10);
        }
        s
    }

    /// The phone's conversation with an adapter on RFCOMM channel 0x41/0x40 (DLCI 2), handle 0x0B, plus a hands-free
    /// channel on DLCI 4 and an audio-like channel (PSM 19) on handle 0x0C that must never be read.
    pub fn standard() -> Session {
        let mut s = Session::new();
        s.event(b"\x03\x0B\x00CONTACT-NAME-EVENT");
        s.connect(0x0B, 1, 3, 0x40, 0x41);
        s.l2(true, 0x0B, 0x41, &[0x03, 0x3F, 0x01, 0x1C], 64);
        s.l2(false, 0x0B, 0x40, &[0x03, 0x73, 0x01, 0xD7], 64);
        s.connect(0x0C, 2, 0x19, 0x50, 0x51);
        // an audio-like channel whose bytes would read as an adapter's if it were RFCOMM: never read
        let fake = uih(2, true, false, b"221234\rCONTACT\r");
        s.l2(true, 0x0C, 0x51, &fake, 64);
        let fake = uih(2, false, false, b"62 12 34 01\r>");
        s.l2(false, 0x0C, 0x50, &fake, 64);
        // hands-free on DLCI 4: AT commands, but not an adapter's
        let hf = uih(4, true, false, b"AT+BRSF=959\r");
        s.l2(true, 0x0B, 0x41, &hf, 64);
        let ag = uih(4, false, false, b"\r\n+BRSF: 871 CONTACT >\r\n\r\nOK\r\n");
        s.l2(false, 0x0B, 0x40, &ag, 64);
        let talk: [(&str, &str); 12] = [
            ("ATZ\r", "\r\rELM327 v1.4b\r\r>"),
            ("ATE0\r", "ATE0\rOK\r\r>"),
            ("ATH1\r", "OK\r\r>"),
            ("ATSH7E0\r", "OK\r\r>"),
            (
                "221234\r",
                "7E8 10 0C 62 12 34 01 02 03\r7E8 21 04 05 06 07 08 09 0A\r\r>",
            ),
            ("22 56 78\r", "7E8 03 7F 22 31\r\r>"),
            ("229ABC\r", "NO DATA\r\r>"),
            ("2E 12 34 01\r", "7E8 03 6E 12 34\r\r>"),
            ("ATH0\r", "OK\r\r>"),
            ("2105\r", "61 05 11 22\r\r>"),
            (
                "22F190\r",
                "014\r0: 62 F1 90 4A 54 45\r1: 53 54 30 30 30 30 30\r2: 54 30 30 30 30 30 31\r\r>",
            ),
            ("STDI\r", "OBDLink MX+ r0.0\r\r>"),
        ];
        for (i, (c, a)) in talk.iter().enumerate() {
            let cmd = uih(2, true, i % 3 == 0, c.as_bytes());
            s.l2(true, 0x0B, 0x41, &cmd, 64);
            // the long answers cut into small ACL fragments
            let ans = uih(2, false, i % 2 == 0, a.as_bytes());
            s.l2(false, 0x0B, 0x40, &ans, 10);
        }
        s
    }

    /// An adapter conversation (handle 0x0B, RFCOMM channel 0x41/0x40) whose packets are cut into ACL fragments of each
    /// side's own size (the phone's 6 bytes, the adapter's 10) and interleaved: each answer's fragments alternate with
    /// those of the phone's next command.
    pub fn interleaved() -> Session {
        Session::interleaved_in(6, 10)
    }

    /// The same conversation with the phone's fragments `phone` bytes long and the adapter's `adapter` (ICA-63: both the
    /// same size, so only the order the fragments come in tells the two sides apart).
    pub fn interleaved_in(phone: usize, adapter: usize) -> Session {
        let mut s = Session::new();
        s.connect(0x0B, 1, 3, 0x40, 0x41);
        let talk = [
            ("ATZ\r", "ELM327 v1.4b\r\r>"),
            ("ATH1\r", "OK\r\r>"),
            ("ATSH7E0\r", "OK\r\r>"),
            (
                "221234\r",
                "7E8 10 0B 62 12 34 01 02 03\r7E8 21 04 05 06 07 08 AA AA\r\r>",
            ),
            ("2105\r", "7E8 03 61 05 09\r\r>"),
            ("22 5678\r", "7E8 04 62 56 78 0C\r\r>"),
        ];
        let cmd =
            |c: &str| Session::frags(true, 0x0B, 0x41, &uih(2, true, false, c.as_bytes()), phone);
        let ans = |a: &str| {
            Session::frags(
                false,
                0x0B,
                0x40,
                &uih(2, false, false, a.as_bytes()),
                adapter,
            )
        };
        for p in cmd(talk[0].0) {
            s.log(p);
        }
        for (k, (_, a)) in talk.iter().enumerate() {
            let mut answer = ans(a).into_iter();
            let mut next = talk
                .get(k + 1)
                .map(|t| cmd(t.0))
                .unwrap_or_default()
                .into_iter();
            loop {
                let (x, y) = (answer.next(), next.next());
                if x.is_none() && y.is_none() {
                    break;
                }
                for p in [x, y].into_iter().flatten() {
                    s.log(p);
                }
            }
        }
        s
    }

    /// The same kind of conversation inside iAP2 link packets, one of them sent twice.
    pub fn iap2() -> Session {
        let mut s = Session::new();
        s.connect(0x0B, 1, 3, 0x40, 0x41);
        let syn = iap2_packet(0, 0, &[]);
        s.l2(true, 0x0B, 0x41, &uih(2, true, false, &syn), 64);
        let control = iap2_packet(1, 1, &[0x40, 0x40, 0x00, 0x06, 0x68, 0x00]);
        s.l2(true, 0x0B, 0x41, &uih(2, true, false, &control), 64);
        let mut ea = vec![0x00, 0x01];
        ea.extend_from_slice(b"221234\r");
        let req = iap2_packet(2, 2, &ea);
        // one link packet over two RFCOMM frames, then sent again (a retransmission)
        let (a, b) = req.split_at(6);
        s.l2(true, 0x0B, 0x41, &uih(2, true, false, a), 64);
        s.l2(true, 0x0B, 0x41, &uih(2, true, false, b), 64);
        s.l2(true, 0x0B, 0x41, &uih(2, true, false, &req), 64);
        let mut ea = vec![0x00, 0x01];
        ea.extend_from_slice(b"62 12 34 07\r\r>");
        let ans = iap2_packet(1, 2, &ea);
        s.l2(false, 0x0B, 0x40, &uih(2, false, false, &ans), 64);
        s
    }

    /// A Bluetooth LE adapter: ATT writes and notifications on LE connection 0x40.
    pub fn ble() -> Session {
        let mut s = Session::new();
        let mut w = vec![0x52, 0x12, 0x00];
        w.extend_from_slice(b"2101\r");
        s.l2(true, 0x40, 4, &w, 64);
        let mut n = vec![0x1B, 0x15, 0x00];
        n.extend_from_slice(b"61 01 05\r\r>");
        s.l2(false, 0x40, 4, &n, 64);
        s
    }
}

/// A pcap file: link type 201 (H4 with a direction word) when `phdr`, else 187 (H4).
pub fn pcap_file(packets: &[Pkt], phdr: bool) -> Vec<u8> {
    let mut f = 0xA1B2_C3D4u32.to_le_bytes().to_vec();
    f.extend_from_slice(&2u16.to_le_bytes());
    f.extend_from_slice(&4u16.to_le_bytes());
    f.extend_from_slice(&[0; 8]);
    f.extend_from_slice(&65535u32.to_le_bytes());
    f.extend_from_slice(&(if phdr { 201u32 } else { 187 }).to_le_bytes());
    for p in packets {
        let mut data = Vec::new();
        if phdr {
            data.extend_from_slice(&u32::from(!p.sent).to_be_bytes());
        }
        data.push(match p.kind {
            Kind::Acl => 0x02,
            Kind::Event => 0x04,
        });
        data.extend_from_slice(&p.bytes);
        let secs = p.clock.floor();
        f.extend_from_slice(&(secs as u32).to_le_bytes());
        f.extend_from_slice(&(((p.clock - secs) * 1e6).round() as u32).to_le_bytes());
        f.extend_from_slice(&(data.len() as u32).to_le_bytes());
        f.extend_from_slice(&(data.len() as u32).to_le_bytes());
        f.extend_from_slice(&data);
    }
    f
}

/// An Apple PacketLogger file, big- or little-endian.
pub fn pklg_file(packets: &[Pkt], big: bool) -> Vec<u8> {
    let w = |v: u32| {
        if big {
            v.to_be_bytes()
        } else {
            v.to_le_bytes()
        }
    };
    let mut f = Vec::new();
    for p in packets {
        let kind = match (&p.kind, p.sent) {
            (Kind::Event, _) => 0x01,
            (Kind::Acl, true) => 0x02,
            (Kind::Acl, false) => 0x03,
        };
        let secs = p.clock.floor();
        f.extend_from_slice(&w(9 + p.bytes.len() as u32));
        f.extend_from_slice(&w(secs as u32));
        f.extend_from_slice(&w(((p.clock - secs) * 1e6).round() as u32));
        f.push(kind);
        f.extend_from_slice(&p.bytes);
    }
    f
}
