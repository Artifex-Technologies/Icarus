//! Diagnostic messages on CAN, shared by everything that watches or reads them back (the bus watch in `icarus-obd`'s
//! `bus`, and the capture importers in `moddata::capture`): ISO 15765-2 (ISO-TP) frames put back together, and what each
//! diagnostic service is called.

use std::collections::HashMap;
use std::hash::Hash;

/// The longest message classic ISO-TP carries (a 12-bit length).
pub const MAX_LEN: usize = 4095;

/// One CAN frame's data as ISO-TP sees it (the protocol control information in the first byte, or the second with
/// extended addressing).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Frame<'a> {
    /// A whole message of up to 7 bytes.
    Single(&'a [u8]),
    /// The start of a longer message: its whole length and the first bytes.
    First { len: usize, data: &'a [u8] },
    /// The next part of a longer message, numbered 1..15, 0, 1...
    Consecutive { seq: u8, data: &'a [u8] },
    /// The receiver saying how to send the rest: no message data.
    FlowControl,
}

/// What a frame's data is, if it is an ISO-TP frame at all.
pub fn frame(b: &[u8]) -> Option<Frame<'_>> {
    let &pci = b.first()?;
    match pci >> 4 {
        0 => {
            let n = (pci & 0x0F) as usize;
            (n > 0 && n <= 7 && b.len() > n).then(|| Frame::Single(&b[1..=n]))
        }
        1 => {
            let len = (((pci & 0x0F) as usize) << 8) | *b.get(1)? as usize;
            (len > 7 && b.len() > 2).then(|| Frame::First { len, data: &b[2..] })
        }
        2 => (b.len() > 1).then(|| Frame::Consecutive {
            seq: pci & 0x0F,
            data: &b[1..],
        }),
        3 => Some(Frame::FlowControl),
        _ => None,
    }
}

/// What feeding one frame gave.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Fed {
    /// A whole message.
    Message(Vec<u8>),
    /// Part of a message; more frames to come.
    Partial,
    /// Flow control: nothing to keep.
    FlowControl,
    /// The frame does not fit: not ISO-TP, a part that never started, or a part lost or repeated (the message it belonged
    /// to is dropped, never kept short or wrong).
    Broken(&'static str),
}

struct Open {
    buf: Vec<u8>,
    len: usize,
    next: u8,
}

/// Messages being put together, one per sender (`K`: a CAN ID, or an ID and an extended address byte).
pub struct Reassembler<K> {
    open: HashMap<K, Open>,
}

impl<K> Default for Reassembler<K> {
    fn default() -> Self {
        Reassembler {
            open: HashMap::new(),
        }
    }
}

impl<K: Eq + Hash> Reassembler<K> {
    /// One frame from `key`. A single or first frame while a message from the same sender is still open drops the open one
    /// (the sender gave up on it).
    pub fn feed(&mut self, key: K, b: &[u8]) -> Fed {
        match frame(b) {
            None => Fed::Broken("not an ISO-TP frame"),
            Some(Frame::FlowControl) => Fed::FlowControl,
            Some(Frame::Single(d)) => {
                self.open.remove(&key);
                Fed::Message(d.to_vec())
            }
            Some(Frame::First { len, data }) => {
                let mut buf = data.to_vec();
                buf.truncate(len);
                self.open.insert(key, Open { buf, len, next: 1 });
                Fed::Partial
            }
            Some(Frame::Consecutive { seq, data }) => {
                let Some(o) = self.open.get_mut(&key) else {
                    return Fed::Broken("a part of a message that never started");
                };
                if seq != o.next {
                    self.open.remove(&key);
                    return Fed::Broken("a part was lost or repeated");
                }
                o.next = (o.next + 1) & 0x0F;
                let take = (o.len - o.buf.len()).min(data.len());
                o.buf.extend_from_slice(&data[..take]);
                if o.buf.len() >= o.len {
                    let o = self.open.remove(&key).expect("open");
                    return Fed::Message(o.buf);
                }
                Fed::Partial
            }
        }
    }
}

/// Whether a service byte asks something (a request) rather than answers (a positive answer is the request's + 0x40;
/// 7F is a refusal).
pub fn is_request(sid: u8) -> bool {
    matches!(sid, 0x00..=0x3F | 0x80..=0xBF)
}

/// Whether a service byte is an answer: positive (0x40-0x7E, 0xC0-0xFE) or a refusal (7F).
pub fn is_answer(sid: u8) -> bool {
    !is_request(sid) && sid != 0xFF
}

/// The request service an answer answers (a refusal names it in its second byte).
pub fn answers(msg: &[u8]) -> Option<u8> {
    match *msg.first()? {
        0x7F => msg.get(1).copied(),
        s if is_answer(s) => Some(s - 0x40),
        _ => None,
    }
}

/// What a request service is called.
pub fn service_name(sid: u8) -> &'static str {
    match sid {
        0x01 => "current data",
        0x02 => "freeze frame",
        0x03 => "stored codes",
        0x04 => "clear codes (OBD-II)",
        0x05 => "oxygen sensor tests",
        0x06 => "on-board test results",
        0x07 => "pending codes",
        0x08 => "control of an on-board system",
        0x09 => "vehicle information",
        0x0A => "permanent codes",
        0x10 => "diagnostic session",
        0x11 => "module reset",
        0x14 => "clear codes",
        0x17 => "read codes (KWP2000 status)",
        0x18 => "read codes (KWP2000)",
        0x19 => "read codes",
        0x1A => "read identification (KWP2000)",
        0x21 => "read data (KWP2000 local ID)",
        0x22 => "read data by identifier",
        0x23 => "read memory",
        0x27 => "security access",
        0x28 => "communication control",
        0x2C => "define data identifier",
        0x2E => "write data by identifier",
        0x2F => "output control",
        0x30 => "output control (KWP2000)",
        0x31 => "routine",
        0x34 => "request download (programming)",
        0x35 => "request upload",
        0x36 => "transfer data",
        0x37 => "end of transfer",
        0x3B => "write data (KWP2000)",
        0x3D => "write memory",
        0x3E => "tester present",
        0x81 => "start communication (KWP2000)",
        0x82 => "stop communication (KWP2000)",
        0x85 => "trouble code setting",
        _ => "another service",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frames_are_read() {
        assert_eq!(
            frame(&[0x03, 0x22, 0x12, 0x34]),
            Some(Frame::Single(&[0x22, 0x12, 0x34]))
        );
        assert_eq!(frame(&[0x03, 0x22]), None, "shorter than it says");
        assert_eq!(frame(&[0x00, 0x22]), None);
        assert_eq!(
            frame(&[0x10, 0x05, 1, 2]),
            None,
            "a first frame of a single frame's length"
        );
        assert_eq!(frame(&[0x30, 0, 0]), Some(Frame::FlowControl));
        assert_eq!(frame(&[0x45, 0, 0]), None);
        assert_eq!(frame(&[]), None);
    }

    #[test]
    fn a_long_message_is_put_together_and_a_lost_part_drops_it() {
        let mut r = Reassembler::default();
        assert_eq!(r.feed(1u32, &[0x10, 0x0A, 1, 2, 3, 4, 5, 6]), Fed::Partial);
        assert_eq!(r.feed(1, &[0x30, 0, 0]), Fed::FlowControl);
        assert_eq!(
            r.feed(1, &[0x21, 7, 8, 9, 10, 0xAA, 0xAA, 0xAA]),
            Fed::Message(vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10]),
            "padding after the length is not kept"
        );
        r.feed(2, &[0x10, 0x0A, 1, 2, 3, 4, 5, 6]);
        assert!(matches!(r.feed(2, &[0x22, 7, 8, 9, 10]), Fed::Broken(_)));
        assert!(
            matches!(r.feed(2, &[0x21, 7, 8, 9, 10]), Fed::Broken(_)),
            "dropped"
        );
        assert!(matches!(r.feed(3, &[0x21, 7]), Fed::Broken(_)));
    }

    #[test]
    fn services() {
        assert!(is_request(0x22) && is_request(0x85) && !is_request(0x62));
        assert_eq!(answers(&[0x62, 0x12]), Some(0x22));
        assert_eq!(answers(&[0x7F, 0x22, 0x31]), Some(0x22));
        assert_eq!(answers(&[0x22]), None);
        assert_eq!(service_name(0x2E), "write data by identifier");
    }
}
