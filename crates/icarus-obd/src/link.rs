//! The link to the vehicle's modules, whatever carries it: an ELM327/STN adapter (`elm`), a J2534 pass-through device
//! (`passthru`: Tactrix OpenPort, VF Tuner's WiFlash, ...), or the simulated car. Everything above this (OBD-II services,
//! UDS/KWP, reports, tests) asks through a `Link` and does not care which.
//!
//! Every link to a real vehicle refuses, before sending, a request `lock::changes_vehicle` names when the vehicle's
//! Read-Only switch is on (`lock::writes_allowed`), as `Elm` does: the write paths check too, but this is the floor.
//!
//! Module IDs are CAN IDs: up to 7FF an 11-bit ID, above it a 29-bit one (Honda's 18DAxxF1).

use crate::elm::{Answer, Elm, ObdError, Protocol};

/// Which modules a link can reach: the ID widths it can send on the vehicle's diagnostic bus (OBD pins 6 and 14), and the
/// other buses it can switch to.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Reach {
    /// 11-bit module IDs (7E0, 7B0, 241...).
    pub can11: bool,
    /// 29-bit module IDs (18DA30F1...).
    pub can29: bool,
    /// Ford's medium-speed CAN on OBD pins 3 and 11 (`bus = "ms-can"` in a profile).
    pub ms_can: bool,
}

impl Reach {
    /// What any link on a vehicle using `p` reaches: the ID width of the vehicle's own ISO 15765 CAN protocol, no other
    /// bus (J1939, heavy trucks' own language, is not one Icarus asks modules in).
    pub fn of(p: Protocol) -> Reach {
        Reach {
            can11: matches!(p, Protocol::Can11At500 | Protocol::Can11At250),
            can29: matches!(p, Protocol::Can29At500 | Protocol::Can29At250),
            ms_can: false,
        }
    }
}

/// The CAN bus a link's requests go out on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Bus {
    /// The vehicle's diagnostic CAN on OBD pins 6 and 14 (high-speed CAN): every OBD-II request.
    Main,
    /// Ford's medium-speed CAN on pins 3 and 11 (125 kbit/s, 11-bit IDs).
    MsCan,
}

pub trait Link {
    /// An OBD-II request to every module (CAN: the functional ID 7DF, or 18DB33F1 on 29-bit CAN); each module's answer.
    fn request(&mut self, data: &[u8]) -> Result<Vec<Answer>, ObdError>;
    /// A request to one module on CAN: sent to `tx`, answered from `rx` (11-bit: usually tx + 8; 29-bit 18DAxxF1: from
    /// 18DAF1xx).
    fn request_to(&mut self, tx: u32, rx: u32, data: &[u8]) -> Result<Vec<Answer>, ObdError>;
    /// The vehicle's signalling.
    fn protocol(&self) -> Protocol;
    /// What carries the link: the adapter's or device's name and firmware.
    fn adapter(&self) -> String;
    /// The voltage at the OBD port (the vehicle's battery) measured now, when the adapter can.
    fn voltage(&mut self) -> Option<f64>;
    /// Sets the extended address byte for the `request_to` calls that follow (a module behind a gateway), or None for
    /// normal addressing. A link that cannot address modules that way refuses Some.
    fn set_ext(&mut self, ext: Option<u8>) -> Result<(), ObdError> {
        match ext {
            None => Ok(()),
            Some(_) => Err(ObdError::Adapter(
                "this link cannot reach modules behind a gateway (extended addressing)".into(),
            )),
        }
    }
    /// Which modules this link can reach (`vehicles::ModuleAddr::unreachable` says why one is not).
    fn reach(&self) -> Reach {
        Reach::of(self.protocol())
    }
    /// Moves the `request_to` calls that follow to another bus (`uds::at_module` does, and moves them back). Broadcasts
    /// always go on the main bus. A link that cannot switch refuses any bus but the main one.
    fn set_bus(&mut self, bus: Bus) -> Result<(), ObdError> {
        match bus {
            Bus::Main => Ok(()),
            Bus::MsCan => Err(ObdError::Adapter(
                "this link cannot switch to Ford's medium-speed CAN".into(),
            )),
        }
    }
    /// The notes on modules left out of broadcasts' answers since the last call (a message that did not add up), each once
    /// a connection: whoever shows a broadcast's answers shows these too, so a module is never lost silently.
    fn take_dropped(&mut self) -> Vec<String> {
        Vec::new()
    }
}

impl Link for Elm {
    fn request(&mut self, data: &[u8]) -> Result<Vec<Answer>, ObdError> {
        Elm::request(self, data)
    }
    fn request_to(&mut self, tx: u32, rx: u32, data: &[u8]) -> Result<Vec<Answer>, ObdError> {
        Elm::request_to(self, tx, rx, data)
    }
    fn protocol(&self) -> Protocol {
        self.protocol
    }
    fn adapter(&self) -> String {
        format!(
            "{}{}",
            self.id,
            self.stn
                .as_ref()
                .map(|s| format!(" ({s})"))
                .unwrap_or_default()
        )
    }
    fn set_ext(&mut self, ext: Option<u8>) -> Result<(), ObdError> {
        Elm::set_ext(self, ext);
        Ok(())
    }
    fn voltage(&mut self) -> Option<f64> {
        self.command("ATRV", std::time::Duration::from_secs(2))
            .ok()
            .and_then(|r| {
                r.first()
                    .and_then(|v| v.trim().trim_end_matches('V').parse::<f64>().ok())
            })
            .or(self.voltage)
    }
    fn reach(&self) -> Reach {
        Elm::reach(self)
    }
    fn set_bus(&mut self, bus: Bus) -> Result<(), ObdError> {
        Elm::set_bus(self, bus)
    }
    fn take_dropped(&mut self) -> Vec<String> {
        Elm::take_dropped(self)
    }
}
