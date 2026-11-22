//! A phone's Bluetooth log, read down to an OBD adapter's conversation (`elm`). Only HCI ACL data (what the phone and the
//! devices it is connected to sent each other) is read; HCI commands and events, where Bluetooth addresses, device names
//! and link keys travel, are skipped whole, unread.
//!
//! **Files.** Two formats, as `idevicebtlogger` (libimobiledevice) writes them:
//!
//! - **pcap** (`idevicebtlogger -f pcap FILE`): the classic pcap file (a 24-byte header with magic A1B2C3D4 in either byte
//!   order, microsecond or nanosecond times, then records of seconds, fraction, captured length, original length and the
//!   bytes). Link type 201, Bluetooth HCI H4 with a 4-byte direction word in front (its lowest bit set: received by the
//!   phone), then the H4 packet type (02: ACL data); or link type 187, plain H4, which does not say who sent a packet
//!   (then the RFCOMM command/response bit tells the two sides apart, and the adapter is the side that sends `>`). pcapng
//!   and Android's btsnoop are recognised and refused with a word on converting them.
//! - **Apple PacketLogger** (`.pklg`, idevicebtlogger's default): records of a 4-byte length (counting what follows it),
//!   4-byte seconds, 4-byte microseconds, one type byte and the packet. The type says what the packet is: 00 an HCI
//!   command, 01 an HCI event, 02 ACL data the phone sent, 03 ACL data it received (others: SCO audio, notes, power and
//!   configuration). Files are big-endian (as PacketLogger writes them); little-endian files are read too, the order told
//!   by which reading of the first record makes sense.
//!
//! **Layers.** HCI ACL packets (a 12-bit connection handle, a packet-boundary flag: a start or a continuation, and the
//! length) are put back together into L2CAP packets per connection and direction (L2CAP: a length, a channel ID, the
//! data); without directions (link type 187) a packet stays open for each side, and a continuation goes to the one it
//! fits (`pick`: each side's own fragment size, what is left, and the RFCOMM checksum). The signalling channel's connection requests and answers say which channels are RFCOMM (PSM 3), and its
//! disconnections which were closed; each side picks its own channel IDs (a phone's SDP query and its RFCOMM channel often
//! both get 0x40), so a channel is known by the ID its packets carry in each direction, and an ID used again names the
//! newer channel. A channel set up before the capture began is tried as RFCOMM, and kept only when its frames' checksums
//! (FCS) hold. RFCOMM's data frames
//! (UIH) carry the serial bytes per data link (DLCI); with credit-based flow control a frame with the poll/final bit set
//! carries a credit byte before the data, which is left out. An MFi accessory such as the OBDLink MX+ talks to an iPhone
//! app inside Apple's iAP2 link packets on its RFCOMM channel: where a stream holds them (FF 5A, a length, control,
//! sequence and acknowledgement numbers, a session, a header checksum, then the payload and its checksum, both checksums
//! adding up to zero), the external-accessory session's bytes (after its two-byte session number) are taken out, and a
//! packet sent again with the same sequence number is read once. Bluetooth LE adapters are read from ATT writes (the app)
//! and notifications (the adapter) on each LE connection.
//!
//! A stream is the adapter's conversation when one side sends `>` prompts and the other side's lines look like AT, ST and
//! hex commands; every other channel is dropped as it is found. A malformed record ends the reading with an error (never a
//! panic); a capture cut short in its last record is read up to there, and noted.

use super::elm::{self, Chunk};
use super::Read;
use std::collections::{HashMap, HashSet};

/// The largest record read: far above any HCI packet; a bigger one is a broken file, not a packet.
const MAX_RECORD: usize = 1 << 18;

/// Who sent a packet, from the phone's side.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Dir {
    /// The phone sent it.
    Sent,
    /// The phone received it.
    Received,
}

/// One ACL packet: when, who sent it (when the file says), its bytes (handle, flags, length, data).
struct Acl<'a> {
    clock: f64,
    dir: Option<Dir>,
    data: &'a [u8],
}

struct Capture<'a> {
    format: String,
    start: Option<f64>,
    acl: Vec<Acl<'a>>,
    records: usize,
    notes: Vec<String>,
}

fn u16le(b: &[u8]) -> u16 {
    u16::from_le_bytes([b[0], b[1]])
}

fn u32le(b: &[u8]) -> u32 {
    u32::from_le_bytes([b[0], b[1], b[2], b[3]])
}

fn u32be(b: &[u8]) -> u32 {
    u32::from_be_bytes([b[0], b[1], b[2], b[3]])
}

/// The first record's length if the file is PacketLogger in this byte order (None: it is not).
fn pklg_first(b: &[u8], big: bool) -> Option<usize> {
    let h = b.get(..13)?;
    let len = if big { u32be(h) } else { u32le(h) } as usize;
    let kind = h[12];
    let known = matches!(kind, 0x00..=0x0B | 0x10..=0x12 | 0xF7..=0xFF);
    ((9..=MAX_RECORD).contains(&len) && known).then_some(len)
}

/// Whether a file is a binary capture (pcap, pcapng, btsnoop or PacketLogger) rather than a text log.
pub fn looks_binary(b: &[u8]) -> bool {
    if b.len() >= 4 {
        let m = u32le(b);
        if matches!(
            m,
            0xA1B2_C3D4 | 0xD4C3_B2A1 | 0xA1B2_3C4D | 0x4D3C_B2A1 | 0x0A0D_0D0A
        ) {
            return true;
        }
    }
    b.starts_with(b"btsnoop\0")
        || pklg_first(b, true).is_some()
        || pklg_first(b, false).is_some()
        || b.iter()
            .take(4096)
            .any(|c| *c < 0x09 || ((0x0E..0x20).contains(c) && *c != 0x1B))
}

fn pcap(b: &[u8]) -> Result<Capture<'_>, String> {
    if b.len() < 24 {
        return Err("a pcap file shorter than its own header".into());
    }
    let (le, nanos) = match u32le(b) {
        0xA1B2_C3D4 => (true, false),
        0xD4C3_B2A1 => (false, false),
        0xA1B2_3C4D => (true, true),
        0x4D3C_B2A1 => (false, true),
        _ => return Err("not a pcap file".into()),
    };
    let rd = |p: &[u8]| if le { u32le(p) } else { u32be(p) };
    let link = rd(&b[20..24]) & 0x0FFF_FFFF;
    let phdr = match link {
        201 => true,
        187 => false,
        other => {
            return Err(format!(
                "pcap link type {other}: not a Bluetooth HCI capture (Icarus reads link types 201 and 187, as \
                 idevicebtlogger -f pcap writes them)"
            ))
        }
    };
    let mut c = Capture {
        format: format!(
            "pcap, Bluetooth HCI H4{}",
            if phdr {
                " with direction (link type 201)"
            } else {
                " without direction (link type 187)"
            }
        ),
        start: None,
        acl: Vec::new(),
        records: 0,
        notes: Vec::new(),
    };
    let mut pos = 24;
    while pos < b.len() {
        let Some(h) = b.get(pos..pos + 16) else {
            c.notes.push(
                "the capture ends inside a record's header (cut short): read up to there".into(),
            );
            break;
        };
        let (secs, frac, incl) = (rd(&h[0..4]), rd(&h[4..8]), rd(&h[8..12]) as usize);
        if incl > MAX_RECORD {
            return Err(format!(
                "record {} at byte {pos} says it holds {incl} bytes: not a capture Icarus can read",
                c.records + 1
            ));
        }
        let Some(data) = b.get(pos + 16..pos + 16 + incl) else {
            c.notes.push(
                "the capture ends inside its last record (cut short): read up to there".into(),
            );
            break;
        };
        pos += 16 + incl;
        c.records += 1;
        let clock = f64::from(secs) + f64::from(frac) / if nanos { 1e9 } else { 1e6 };
        c.start.get_or_insert(clock);
        let (dir, h4) = if phdr {
            if data.len() < 5 {
                continue;
            }
            let d = if u32be(data) & 1 == 1 {
                Dir::Received
            } else {
                Dir::Sent
            };
            (Some(d), &data[4..])
        } else {
            (None, data)
        };
        if h4.first() == Some(&0x02) {
            c.acl.push(Acl {
                clock,
                dir,
                data: &h4[1..],
            });
        }
    }
    Ok(c)
}

fn pklg(b: &[u8]) -> Result<Capture<'_>, String> {
    let big = if pklg_first(b, true).is_some() {
        true
    } else if pklg_first(b, false).is_some() {
        false
    } else {
        return Err(
            "not a capture Icarus reads: not pcap (idevicebtlogger -f pcap) and not Apple PacketLogger (.pklg)".into(),
        );
    };
    let rd = |p: &[u8]| if big { u32be(p) } else { u32le(p) };
    let mut c = Capture {
        format: format!(
            "Apple PacketLogger ({})",
            if big { "big-endian" } else { "little-endian" }
        ),
        start: None,
        acl: Vec::new(),
        records: 0,
        notes: Vec::new(),
    };
    let mut pos = 0;
    while pos < b.len() {
        let Some(h) = b.get(pos..pos + 13) else {
            c.notes.push(
                "the capture ends inside a record's header (cut short): read up to there".into(),
            );
            break;
        };
        let len = rd(&h[0..4]) as usize;
        if !(9..=MAX_RECORD).contains(&len) {
            return Err(format!(
                "record {} at byte {pos} gives its length as {len}: not a PacketLogger record",
                c.records + 1
            ));
        }
        let Some(rec) = b.get(pos + 4..pos + 4 + len) else {
            c.notes.push(
                "the capture ends inside its last record (cut short): read up to there".into(),
            );
            break;
        };
        pos += 4 + len;
        c.records += 1;
        let clock = f64::from(rd(&rec[0..4])) + f64::from(rd(&rec[4..8])) / 1e6;
        c.start.get_or_insert(clock);
        let dir = match rec[8] {
            0x02 => Dir::Sent,
            0x03 => Dir::Received,
            _ => continue, // commands, events, audio, notes: never read
        };
        c.acl.push(Acl {
            clock,
            dir: Some(dir),
            data: &rec[9..],
        });
    }
    Ok(c)
}

/// RFCOMM's frame check (GSM 07.10): CRC-8 over the address and control bytes (and the length for frames other than
/// UIH), reflected polynomial E0, starting at FF, sent inverted.
pub fn fcs(bytes: &[u8]) -> u8 {
    let mut crc = 0xFFu8;
    for b in bytes {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xE0
            } else {
                crc >> 1
            };
        }
    }
    0xFF - crc
}

/// One RFCOMM frame.
struct Rf<'a> {
    dlci: u8,
    /// The command/response bit: set in data frames from the side that opened the connection.
    cr: bool,
    uih: bool,
    data: &'a [u8],
}

/// An L2CAP packet's data as one RFCOMM frame, if it is one (its checksum right).
fn rfcomm(b: &[u8]) -> Option<Rf<'_>> {
    if b.len() < 4 || b[0] & 1 == 0 {
        return None;
    }
    let ctrl = b[1];
    let kind = ctrl & !0x10;
    if !matches!(kind, 0xEF | 0x2F | 0x63 | 0x0F | 0x43) {
        return None;
    }
    let (len, head) = if b[2] & 1 == 1 {
        ((b[2] >> 1) as usize, 3)
    } else {
        (((b[2] >> 1) as usize) | ((*b.get(3)? as usize) << 7), 4)
    };
    let dlci = b[0] >> 2;
    let uih = kind == 0xEF;
    let credit = usize::from(uih && ctrl & 0x10 != 0 && dlci != 0);
    if b.len() != head + credit + len + 1 {
        return None;
    }
    let over = if uih { &b[..2] } else { &b[..head] };
    if fcs(over) != b[b.len() - 1] {
        return None;
    }
    Some(Rf {
        dlci,
        cr: b[0] & 2 != 0,
        uih,
        data: &b[head + credit..head + credit + len],
    })
}

/// A byte stream between the phone and one device.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum Key {
    /// An RFCOMM data link on a connection.
    Rfcomm(u16, u8),
    /// ATT on an LE connection.
    Att(u16),
}

/// Bytes one side of a stream sent.
#[derive(Clone, Debug)]
struct Piece {
    /// Order in the capture.
    seq: usize,
    clock: f64,
    /// The phone's side when the capture says who sent what; else the side that opened the connection (RFCOMM) or wrote
    /// (ATT).
    side_a: bool,
    bytes: Vec<u8>,
}

/// What a dynamic L2CAP channel was set up for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Chan {
    Rfcomm,
    /// Anything else (SDP, audio, contacts, messages...): never read.
    Other,
}

/// The other direction (None, when the file does not say who sent what, stays None).
fn flip(d: Option<Dir>) -> Option<Dir> {
    d.map(|d| match d {
        Dir::Sent => Dir::Received,
        Dir::Received => Dir::Sent,
    })
}

/// A channel as packets name it: (handle, direction, channel ID). Each side picks its own IDs, so the same number can mean
/// two channels: a packet carries its receiver's ID, so packets the phone sends carry the other side's ID and packets it
/// receives carry its own; the direction tells the two apart (without directions, both share one key).
type ChanKey = (u16, Option<Dir>, u16);

/// An L2CAP packet being put back together: its bytes so far, and how long its first fragment was.
struct Part {
    buf: Vec<u8>,
    first: usize,
}

impl Part {
    /// The bytes still to come, once the L2CAP header (its length) is in.
    fn room(&self) -> Option<usize> {
        (self.buf.len() >= 4)
            .then(|| (4 + u16le(&self.buf) as usize).saturating_sub(self.buf.len()))
    }
}

/// A whole L2CAP packet (at least its header and length, more cut off): (handle, channel, data).
fn whole(handle: u16, mut pdu: Vec<u8>) -> (u16, u16, Vec<u8>) {
    pdu.truncate(4 + u16le(&pdu) as usize);
    let cid = u16le(&pdu[2..]);
    pdu.drain(..4);
    (handle, cid, pdu)
}

/// Which open packet a continuation fragment belongs to, without directions (each side may have one open). Each side
/// cuts its packets into fragments of its own size, so a fragment of a packet is as long as the packet's first and
/// leaves more to come, or is its last, as long as what is left (and no longer than the first); failing that, one it
/// does not run past. Of two, a last fragment that makes a whole RFCOMM frame (its checksum right) goes first, then
/// one that leaves more to come, then the older.
fn pick(open: &[Part], frag: &[u8]) -> Option<usize> {
    let n = frag.len();
    let likely = |p: &Part| {
        p.room()
            .is_some_and(|r| (n == p.first && n < r) || (n == r && n <= p.first))
    };
    let fits = |p: &Part| p.room().is_none_or(|r| n <= r);
    // 2: it ends the packet as a whole RFCOMM frame; 1: more is to come; 0: it ends the packet as something else
    let rank = |p: &Part| match p.room() {
        Some(r) if r == n => {
            let mut b = p.buf[4..].to_vec();
            b.extend_from_slice(frag);
            2 * usize::from(rfcomm(&b).is_some())
        }
        _ => 1,
    };
    let best = |of: &dyn Fn(&Part) -> bool| {
        (0..open.len())
            .filter(|i| of(&open[*i]))
            .rev()
            .max_by_key(|i| rank(&open[*i]))
    };
    best(&likely).or_else(|| best(&fits))
}

/// L2CAP packets put back together from ACL fragments, and which channels are RFCOMM.
#[derive(Default)]
struct L2cap {
    /// Packets open per connection and direction: one a direction; without directions, one for each side (two).
    partial: HashMap<(u16, Option<Dir>), Vec<Part>>,
    /// (handle, the request's direction, signalling identifier) -> the PSM asked for.
    asked: HashMap<(u16, Option<Dir>, u8), u16>,
    /// Channels open now: set up by a connection, closed by a disconnection; a newer connection replaces an older one.
    chans: HashMap<ChanKey, Chan>,
    /// Channels other than RFCOMM set up over the capture (counted, never read).
    others: usize,
    malformed: usize,
}

impl L2cap {
    /// One ACL packet; a whole L2CAP packet when this one completes it: (handle, channel, data).
    fn acl(&mut self, a: &Acl) -> Option<(u16, u16, Vec<u8>)> {
        if a.data.len() < 4 {
            self.malformed += 1;
            return None;
        }
        let hf = u16le(a.data);
        let handle = hf & 0x0FFF;
        let start = (hf >> 12) & 3 != 1;
        let len = u16le(&a.data[2..]) as usize;
        let Some(data) = a.data.get(4..4 + len) else {
            self.malformed += 1;
            return None;
        };
        let open = self.partial.entry((handle, a.dir)).or_default();
        let i = if start {
            let part = Part {
                buf: data.to_vec(),
                first: data.len(),
            };
            // with directions, a side's new packet ends its last one (given up)
            if a.dir.is_some() {
                open.clear();
            }
            if part.room() == Some(0) {
                return Some(whole(handle, part.buf));
            }
            // without, a start while one is open is the other side's: one kept for each side
            if open.len() >= 2 {
                open.remove(0);
            }
            open.push(part);
            open.len() - 1
        } else {
            let i = if open.len() > 1 {
                pick(open, data)
            } else {
                (!open.is_empty()).then_some(0)
            };
            let Some(i) = i else {
                self.malformed += 1;
                return None;
            };
            open[i].buf.extend_from_slice(data);
            i
        };
        // whole once its header's length is in
        if open[i].room().is_none_or(|r| r > 0) {
            return None;
        }
        Some(whole(handle, open.remove(i).buf))
    }

    /// The signalling channel, `dir` the way this packet went: which dynamic channels are RFCOMM (PSM 3), which are
    /// something else, and which were closed.
    fn signalling(&mut self, handle: u16, dir: Option<Dir>, mut b: &[u8]) {
        while b.len() >= 4 {
            let (code, id, len) = (b[0], b[1], u16le(&b[2..]) as usize);
            let Some(body) = b.get(4..4 + len) else {
                return;
            };
            match code {
                // connection request: PSM, the requester's channel
                0x02 if len >= 4 => {
                    self.asked.insert((handle, dir, id), u16le(body));
                }
                // connection response (from the responder): its channel, the requester's channel, the result
                0x03 if len >= 8 => {
                    let (dcid, scid, result) = (u16le(body), u16le(&body[2..]), u16le(&body[4..]));
                    if result == 0 {
                        if let Some(psm) = self.asked.remove(&(handle, flip(dir), id)) {
                            let kind = if psm == 3 { Chan::Rfcomm } else { Chan::Other };
                            self.others += usize::from(kind == Chan::Other);
                            // packets to the responder carry its channel; packets to the requester carry the requester's
                            self.chans.insert((handle, flip(dir), dcid), kind);
                            self.chans.insert((handle, dir, scid), kind);
                        }
                    }
                }
                // disconnection request: the receiver's channel, the sender's channel
                0x06 if len >= 4 => {
                    let (dcid, scid) = (u16le(body), u16le(&body[2..]));
                    self.chans.remove(&(handle, dir, dcid));
                    self.chans.remove(&(handle, flip(dir), scid));
                }
                // disconnection response (from the request's receiver): the same two channels
                0x07 if len >= 4 => {
                    let (dcid, scid) = (u16le(body), u16le(&body[2..]));
                    self.chans.remove(&(handle, flip(dir), dcid));
                    self.chans.remove(&(handle, dir, scid));
                }
                _ => {}
            }
            b = &b[4 + len..];
        }
    }

    /// What the channel a packet went on was set up for, when the capture showed it.
    fn chan(&self, handle: u16, dir: Option<Dir>, cid: u16) -> Option<Chan> {
        self.chans.get(&(handle, dir, cid)).copied()
    }
}

/// The external-accessory bytes inside Apple's iAP2 link packets, when a side's stream holds them (None: it does not).
fn iap2(pieces: &[Piece]) -> Option<Vec<Piece>> {
    let mut all = Vec::new();
    let mut owner = Vec::new();
    for (i, p) in pieces.iter().enumerate() {
        all.extend_from_slice(&p.bytes);
        owner.resize(all.len(), i);
    }
    let sum = |b: &[u8]| b.iter().fold(0u8, |a, x| a.wrapping_add(*x));
    let (mut out, mut found, mut last) = (Vec::new(), false, None);
    let mut i = 0;
    while i + 9 <= all.len() {
        if all[i] == 0xFF && all[i + 1] == 0x5A {
            let len = u16::from_be_bytes([all[i + 2], all[i + 3]]) as usize;
            if len >= 9 && i + len <= all.len() && sum(&all[i..i + 9]) == 0 {
                found = true;
                if len > 10 && sum(&all[i + 9..i + len]) == 0 {
                    let payload = &all[i + 9..i + len - 1];
                    let seq = all[i + 5];
                    if last != Some(seq) {
                        last = Some(seq);
                        // the control session's messages start 40 40; an accessory session's data has its number first
                        if payload.len() > 2 && payload[..2] != [0x40, 0x40] {
                            let p = &pieces[owner[i + len - 1]];
                            out.push(Piece {
                                seq: p.seq,
                                clock: p.clock,
                                side_a: p.side_a,
                                bytes: payload[2..].to_vec(),
                            });
                        }
                    }
                }
                i += len;
                continue;
            }
        }
        i += 1;
    }
    found.then_some(out)
}

/// Whether the app side's lines look like an ELM327/STN conversation (AT, ST and hex commands).
fn elm_like(app: &[Piece]) -> bool {
    let text: Vec<u8> = app.iter().flat_map(|p| p.bytes.iter().copied()).collect();
    let (mut total, mut good) = (0usize, 0usize);
    for line in text.split(|b| *b == b'\r') {
        let l: Vec<u8> = line
            .iter()
            .copied()
            .filter(|b| !matches!(b, b'\n' | b' '))
            .collect();
        if l.is_empty() {
            continue;
        }
        total += 1;
        let upper = l.to_ascii_uppercase();
        let adapter = upper.starts_with(b"AT") || upper.starts_with(b"ST");
        let chars_ok = l
            .iter()
            .all(|b| b.is_ascii_alphanumeric() || matches!(b, b',' | b':' | b'.' | b'@'));
        if chars_ok && (adapter || l.iter().all(u8::is_ascii_hexdigit)) {
            good += 1;
        }
    }
    good >= 1 && good * 10 >= total * 8
}

/// Reads a Bluetooth capture down to its adapter conversations' exchanges.
pub fn read(b: &[u8]) -> Result<Read, String> {
    if b.starts_with(b"btsnoop\0") {
        return Err("an Android btsnoop log: Icarus reads pcap and PacketLogger (Wireshark can save it as pcap)".into());
    }
    if b.len() >= 4 && u32le(b) == 0x0A0D_0D0A {
        return Err(
            "a pcapng file: Icarus reads classic pcap (convert it: editcap -F pcap IN.pcapng OUT.pcap)".into(),
        );
    }
    let cap = if b.len() >= 4
        && matches!(
            u32le(b),
            0xA1B2_C3D4 | 0xD4C3_B2A1 | 0xA1B2_3C4D | 0x4D3C_B2A1
        ) {
        pcap(b)?
    } else {
        pklg(b)?
    };
    let start = cap.start.unwrap_or(0.0);
    let mut l2 = L2cap::default();
    let mut streams: Vec<(Key, Vec<Piece>)> = Vec::new();
    let mut index: HashMap<Key, usize> = HashMap::new();
    let mut not_rfcomm: HashSet<(u16, u16)> = HashSet::new();
    let mut add = |key: Key, piece: Piece| {
        let i = *index.entry(key).or_insert_with(|| {
            streams.push((key, Vec::new()));
            streams.len() - 1
        });
        streams[i].1.push(piece);
    };
    for (seq, a) in cap.acl.iter().enumerate() {
        let Some((handle, cid, pdu)) = l2.acl(a) else {
            continue;
        };
        match cid {
            0x0001 => l2.signalling(handle, a.dir, &pdu),
            0x0004 => {
                // ATT (LE): the app writes, the adapter notifies
                let side_a = match pdu.first() {
                    Some(0x12 | 0x52) => true,
                    Some(0x1B | 0x1D) => false,
                    _ => continue,
                };
                if pdu.len() > 3 {
                    add(
                        Key::Att(handle),
                        Piece {
                            seq,
                            clock: a.clock,
                            side_a,
                            bytes: pdu[3..].to_vec(),
                        },
                    );
                }
            }
            c if c >= 0x0040 => {
                let chan = l2.chan(handle, a.dir, c);
                if chan == Some(Chan::Other) {
                    continue; // another kind of channel (audio, contacts, messages...): never read
                }
                match rfcomm(&pdu) {
                    Some(f) if f.uih && f.dlci != 0 && !f.data.is_empty() => {
                        let side_a = match a.dir {
                            Some(d) => d == Dir::Sent,
                            None => f.cr,
                        };
                        add(
                            Key::Rfcomm(handle, f.dlci),
                            Piece {
                                seq,
                                clock: a.clock,
                                side_a,
                                bytes: f.data.to_vec(),
                            },
                        );
                    }
                    Some(_) => {}
                    None => {
                        if chan.is_none() {
                            not_rfcomm.insert((handle, c));
                        }
                    }
                }
            }
            _ => {}
        }
    }
    let mut exchanges = Vec::new();
    let (mut kept, mut dropped, mut adapter_commands, mut wrapped) = (0, 0, 0, 0);
    for (_, pieces) in streams {
        let (mut a, mut b): (Vec<Piece>, Vec<Piece>) = pieces.into_iter().partition(|p| p.side_a);
        let mut was_wrapped = false;
        if let Some(x) = iap2(&a) {
            a = x;
            was_wrapped = true;
        }
        if let Some(x) = iap2(&b) {
            b = x;
            was_wrapped = true;
        }
        let prompts = |s: &[Piece]| {
            s.iter()
                .flat_map(|p| &p.bytes)
                .filter(|c| **c == b'>')
                .count()
        };
        let (pa, pb) = (prompts(&a), prompts(&b));
        let (app, adapter) = if pb >= pa { (a, b) } else { (b, a) };
        if pa.max(pb) == 0 || !elm_like(&app) {
            dropped += 1;
            continue;
        }
        kept += 1;
        wrapped += usize::from(was_wrapped);
        let mut all: Vec<(usize, bool, Piece)> = app
            .into_iter()
            .map(|p| (p.seq, false, p))
            .chain(adapter.into_iter().map(|p| (p.seq, true, p)))
            .collect();
        all.sort_by_key(|(seq, _, _)| *seq);
        let chunks: Vec<Chunk> = all
            .into_iter()
            .map(|(_, adapter, p)| Chunk {
                at: p.clock - start,
                clock: Some(p.clock),
                adapter,
                bytes: p.bytes,
            })
            .collect();
        let c = elm::conversation(&chunks);
        adapter_commands += c.adapter_commands;
        exchanges.extend(c.exchanges);
    }
    exchanges.sort_by(|x, y| x.at.total_cmp(&y.at));
    let mut notes = cap.notes;
    notes.insert(
        0,
        format!(
            "{} record(s), {} ACL packet(s); {kept} adapter conversation(s) kept{}; {} other Bluetooth channel(s) dropped \
             unread",
            cap.records,
            cap.acl.len(),
            if wrapped > 0 {
                format!(" ({wrapped} inside Apple's iAP2 packets)")
            } else {
                String::new()
            },
            dropped + l2.others + not_rfcomm.len()
        ),
    );
    if adapter_commands > 0 {
        notes.push(format!(
            "{adapter_commands} adapter command(s) (AT, ST) followed, not kept"
        ));
    }
    if l2.malformed > 0 {
        notes.push(format!("{} malformed ACL packet(s) skipped", l2.malformed));
    }
    Ok(Read {
        from: "Bluetooth capture",
        format: cap.format,
        exchanges,
        notes,
    })
}

#[cfg(test)]
mod tests {
    use super::super::testkit::*;
    use super::*;

    #[test]
    fn the_frame_check_matches_known_frames() {
        // SABM and UA on the control channel, as every RFCOMM session starts
        assert_eq!(fcs(&[0x03, 0x3F, 0x01]), 0x1C);
        assert_eq!(fcs(&[0x03, 0x73, 0x01]), 0xD7);
        let f = uih(2, true, false, b"ATZ\r");
        let r = rfcomm(&f).unwrap();
        assert_eq!(
            (r.dlci, r.cr, r.uih, r.data),
            (2, true, true, &b"ATZ\r"[..])
        );
        let mut bad = f.clone();
        *bad.last_mut().unwrap() ^= 1;
        assert!(rfcomm(&bad).is_none());
        let credit = uih(2, false, true, b"OK\r\r>");
        assert_eq!(
            rfcomm(&credit).unwrap().data,
            b"OK\r\r>",
            "the credit byte is left out"
        );
        let long = uih(2, true, false, &[b'A'; 200]);
        assert_eq!(rfcomm(&long).unwrap().data.len(), 200, "a two-byte length");
    }

    fn read_ok(b: &[u8]) -> Read {
        read(b).unwrap_or_else(|e| panic!("{e}"))
    }

    #[test]
    fn a_pcap_with_directions_gives_the_conversation_and_nothing_else() {
        let cap = Session::standard();
        let r = read_ok(&pcap_file(&cap.packets, true));
        assert!(r.format.contains("201"), "{}", r.format);
        check_standard(&r);
    }

    #[test]
    fn packetlogger_in_both_byte_orders() {
        let cap = Session::standard();
        for big in [true, false] {
            let r = read_ok(&pklg_file(&cap.packets, big));
            assert!(r.format.contains("PacketLogger"));
            check_standard(&r);
        }
    }

    #[test]
    fn plain_h4_without_directions_uses_the_rfcomm_roles() {
        let cap = Session::standard();
        let r = read_ok(&pcap_file(&cap.packets, false));
        assert!(r.format.contains("187"));
        check_standard(&r);
    }

    /// What `Session::standard` holds, as read back.
    fn check_standard(r: &Read) {
        let e = &r.exchanges;
        let reqs: Vec<String> = e.iter().map(|x| super::super::hex(&x.request)).collect();
        assert_eq!(
            reqs,
            vec![
                "22 12 34",
                "22 56 78",
                "22 9A BC",
                "2E 12 34 01",
                "21 05",
                "22 F1 90"
            ],
            "{:?}",
            r.notes
        );
        assert_eq!(e[0].tx, Some(0x7E0));
        assert_eq!(e[0].replies[0].from, Some(0x7E8));
        assert_eq!(
            e[0].replies[0].data.len(),
            12,
            "a long answer split over fragments"
        );
        assert_eq!(e[1].replies[0].data, vec![0x7F, 0x22, 0x31]);
        assert_eq!(e[2].problem.as_deref(), Some("NO DATA"));
        assert_eq!(e[4].replies[0].data, vec![0x61, 0x05, 0x11, 0x22]);
        assert!(e[0].at > 0.0 && e[0].clock.unwrap() > 1.0e9);
        assert!(
            r.notes[0].contains("1 adapter conversation(s) kept"),
            "{}",
            r.notes[0]
        );
        assert!(
            r.notes[0].contains("2 other Bluetooth channel(s) dropped"),
            "{}",
            r.notes[0]
        );
        // nothing from the other channels anywhere
        let all = format!("{:?}", r);
        for secret in ["BRSF", "CONTACT", "42 52 53 46"] {
            assert!(!all.contains(secret), "{secret} leaked: {all}");
        }
    }

    fn check_reused(r: &Read, what: &str) {
        let reqs: Vec<String> = r
            .exchanges
            .iter()
            .map(|x| super::super::hex(&x.request))
            .collect();
        assert_eq!(
            reqs,
            vec!["22 12 34", "22 56 78", "21 05"],
            "{what}: {:?}",
            r.notes
        );
        assert_eq!(r.exchanges[0].replies[0].data, vec![0x62, 0x12, 0x34, 0x01]);
        assert!(
            r.notes[0].contains("1 adapter conversation(s) kept"),
            "{what}: {}",
            r.notes[0]
        );
    }

    #[test]
    fn sdp_then_rfcomm_on_the_same_channel_ids_keeps_the_adapter() {
        // SDP on 0x40 at both ends, then RFCOMM on the phone's 0x41 and the other side's 0x40 again
        let s = Session::sdp_then_rfcomm((0x40, 0x40), false, (0x41, 0x40));
        check_reused(&read_ok(&pklg_file(&s.packets, true)), "pklg");
        check_reused(&read_ok(&pcap_file(&s.packets, true)), "pcap 201");
        check_reused(&read_ok(&pcap_file(&s.packets, false)), "pcap 187");
        // the same, SDP closed first
        let s = Session::sdp_then_rfcomm((0x40, 0x40), true, (0x41, 0x40));
        check_reused(&read_ok(&pklg_file(&s.packets, true)), "pklg, closed");
    }

    #[test]
    fn the_phone_using_a_closed_channel_id_again_keeps_the_adapter() {
        let s = Session::sdp_then_rfcomm((0x40, 0x50), true, (0x40, 0x51));
        check_reused(&read_ok(&pklg_file(&s.packets, false)), "pklg");
        check_reused(&read_ok(&pcap_file(&s.packets, true)), "pcap 201");
        check_reused(&read_ok(&pcap_file(&s.packets, false)), "pcap 187");
    }

    #[test]
    fn a_disconnection_forgets_both_ends_of_a_channel() {
        let sig = |code: u8, id: u8, words: &[u16]| {
            let mut b = vec![code, id, (words.len() * 2) as u8, 0];
            for w in words {
                b.extend_from_slice(&w.to_le_bytes());
            }
            b
        };
        let (sent, got) = (Some(Dir::Sent), Some(Dir::Received));
        let mut l2 = L2cap::default();
        // the phone asks for PSM 1 from its 0x40; the other side answers with its 0x50
        l2.signalling(0x0B, sent, &sig(0x02, 7, &[1, 0x40]));
        l2.signalling(0x0B, got, &sig(0x03, 7, &[0x50, 0x40, 0, 0]));
        assert_eq!(l2.chan(0x0B, sent, 0x50), Some(Chan::Other));
        assert_eq!(l2.chan(0x0B, got, 0x40), Some(Chan::Other));
        assert_eq!(
            l2.chan(0x0B, sent, 0x40),
            None,
            "the phone's own ID is not on its sent packets"
        );
        // the other side closes it: its request names its own channel second
        l2.signalling(0x0B, got, &sig(0x06, 8, &[0x40, 0x50]));
        assert_eq!(l2.chan(0x0B, sent, 0x50), None);
        assert_eq!(l2.chan(0x0B, got, 0x40), None);
        // opened again the other way round (the other side asks, for RFCOMM), closed by the phone's response
        l2.signalling(0x0B, got, &sig(0x02, 9, &[3, 0x60]));
        l2.signalling(0x0B, sent, &sig(0x03, 9, &[0x41, 0x60, 0, 0]));
        assert_eq!(l2.chan(0x0B, sent, 0x60), Some(Chan::Rfcomm));
        assert_eq!(l2.chan(0x0B, got, 0x41), Some(Chan::Rfcomm));
        l2.signalling(0x0B, sent, &sig(0x06, 10, &[0x60, 0x41]));
        l2.signalling(0x0B, got, &sig(0x07, 10, &[0x60, 0x41]));
        assert!(l2.chans.is_empty(), "{:?}", l2.chans);
        assert_eq!(l2.others, 1);
    }

    #[test]
    fn both_sides_fragments_interleaved_without_directions_are_kept_apart() {
        let s = Session::interleaved();
        for (what, file) in [
            ("pcap 187", pcap_file(&s.packets, false)),
            ("pcap 201", pcap_file(&s.packets, true)),
            ("pklg", pklg_file(&s.packets, true)),
        ] {
            let r = read_ok(&file);
            let e = &r.exchanges;
            let reqs: Vec<String> = e.iter().map(|x| super::super::hex(&x.request)).collect();
            assert_eq!(
                reqs,
                vec!["22 12 34", "21 05", "22 56 78"],
                "{what}: {:?}",
                r.notes
            );
            assert_eq!(
                e[0].replies[0].data,
                vec![0x62, 0x12, 0x34, 1, 2, 3, 4, 5, 6, 7, 8],
                "{what}"
            );
            assert_eq!(e[1].replies[0].data, vec![0x61, 0x05, 0x09], "{what}");
            assert_eq!(e[2].replies[0].data, vec![0x62, 0x56, 0x78, 0x0C], "{what}");
            assert!(
                r.notes.iter().all(|n| !n.contains("malformed")),
                "{what}: {:?}",
                r.notes
            );
        }
    }

    #[test]
    fn an_iap2_wrapped_conversation_is_unwrapped() {
        let r = read_ok(&pklg_file(&Session::iap2().packets, true));
        assert!(r.notes[0].contains("iAP2"), "{}", r.notes[0]);
        let e = &r.exchanges;
        assert_eq!(e.len(), 1, "a resent packet is read once");
        assert_eq!(e[0].request, vec![0x22, 0x12, 0x34]);
        assert_eq!(e[0].replies[0].data, vec![0x62, 0x12, 0x34, 0x07]);
    }

    #[test]
    fn a_bluetooth_le_adapter_is_read_from_att() {
        let r = read_ok(&pcap_file(&Session::ble().packets, true));
        assert_eq!(r.exchanges.len(), 1);
        assert_eq!(r.exchanges[0].replies[0].data, vec![0x61, 0x01, 0x05]);
    }

    #[test]
    fn malformed_files_are_errors_and_never_a_panic() {
        assert!(read(b"").is_err());
        assert!(read(b"\x00\x00\x00\x01\x00").is_err());
        assert!(read(b"btsnoop\0\x00\x00\x00\x01")
            .unwrap_err()
            .contains("btsnoop"));
        assert!(read(&0x0A0D_0D0Au32.to_le_bytes())
            .unwrap_err()
            .contains("pcapng"));
        let good = pcap_file(&Session::standard().packets, true);
        // another link type
        let mut other = good.clone();
        other[20] = 1;
        assert!(read(&other).unwrap_err().contains("link type 1"));
        // a record claiming a huge length
        let mut huge = good.clone();
        huge[24 + 8..24 + 12].copy_from_slice(&0x0100_0000u32.to_le_bytes());
        assert!(read(&huge).unwrap_err().contains("holds"));
        let pk = pklg_file(&Session::standard().packets, true);
        let mut zero = pk.clone();
        let first = 4 + u32::from_be_bytes([pk[0], pk[1], pk[2], pk[3]]) as usize;
        zero[first..first + 4].copy_from_slice(&[0, 0, 0, 3]);
        assert!(read(&zero)
            .unwrap_err()
            .contains("not a PacketLogger record"));
        // a capture cut inside its last record reads up to there, noted
        let r = read(&good[..good.len() - 3]).unwrap();
        assert!(
            r.notes.iter().any(|n| n.contains("cut short")),
            "{:?}",
            r.notes
        );
        assert_eq!(r.exchanges.len(), 6);
        // every cut of a good file reads or fails, never a panic
        for cut in (0..good.len()).step_by(7) {
            let _ = read(&good[..cut]);
        }
        for cut in (0..pk.len()).step_by(5) {
            let _ = read(&pk[..cut]);
        }
        // bytes flipped all over: never a panic
        let mut x = 0x1234_5678u32;
        for _ in 0..300 {
            let mut f = good.clone();
            for _ in 0..8 {
                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                let i = 24 + (x as usize) % (f.len() - 24);
                f[i] = (x >> 8) as u8;
            }
            let _ = read(&f);
        }
    }
}
