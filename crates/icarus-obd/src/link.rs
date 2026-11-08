//! The link to the vehicle's modules, whatever carries it: an ELM327/STN adapter (`elm`), a J2534 pass-through device
//! (`passthru`: Tactrix OpenPort, VF Tuner's WiFlash, ...), or the simulated car. Everything above this (OBD-II services,
//! UDS/KWP, reports, tests) asks through a `Link` and does not care which.

use crate::elm::{Answer, Elm, ObdError, Protocol};

pub trait Link {
    /// An OBD-II request to every module (CAN: the functional ID 7DF); each module's answer.
    fn request(&mut self, data: &[u8]) -> Result<Vec<Answer>, ObdError>;
    /// A request to one module on 11-bit CAN: sent to `tx`, answered from `rx` (usually tx + 8).
    fn request_to(&mut self, tx: u16, rx: u16, data: &[u8]) -> Result<Vec<Answer>, ObdError>;
    /// The vehicle's signalling.
    fn protocol(&self) -> Protocol;
    /// What carries the link: the adapter's or device's name and firmware.
    fn adapter(&self) -> String;
    /// The voltage at the OBD port (the vehicle's battery) measured now, when the adapter can.
    fn voltage(&mut self) -> Option<f64>;
}

impl Link for Elm {
    fn request(&mut self, data: &[u8]) -> Result<Vec<Answer>, ObdError> {
        Elm::request(self, data)
    }
    fn request_to(&mut self, tx: u16, rx: u16, data: &[u8]) -> Result<Vec<Answer>, ObdError> {
        Elm::request_to(self, tx, rx, data)
    }
    fn protocol(&self) -> Protocol {
        self.protocol
    }
    fn adapter(&self) -> String {
        format!("{}{}", self.id, self.stn.as_ref().map(|s| format!(" ({s})")).unwrap_or_default())
    }
    fn voltage(&mut self) -> Option<f64> {
        self.command("ATRV", std::time::Duration::from_secs(2))
            .ok()
            .and_then(|r| r.first().and_then(|v| v.trim().trim_end_matches('V').parse::<f64>().ok()))
            .or(self.voltage)
    }
}
