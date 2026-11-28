//! J2534 pass-through devices: the standard interface most tuning and dealer software uses on Windows (SAE J2534-1, the
//! `PassThru...` functions). Icarus does not load anyone's Windows library; each device gets a driver of its own that speaks
//! the device's wire protocol (USB, serial or Wi-Fi) and offers the same handful of operations, `PassThru`. On top of any
//! of them, `PassThruLink` is a `Link`: OBD-II and one-module requests over ISO 15765 (ISO-TP on CAN), with the device
//! doing the segmentation and flow control, as J2534 devices do.
//!
//! Devices: `SimPassThru` (the simulated car, for tests and `--adapter sim:j2534`); the Tactrix OpenPort 2.0 (`openport`,
//! `--adapter openport`); the Drewtech Mongoose Pro (`mongoose`, a raw-CAN cable, so `open_link` puts `canlink::CanLink`
//! over it rather than `PassThruLink`); the WiFlash (VF Tuner's ESP32 device) once its protocol has been learned by watching it
//! (docs/INTERFACES.md). `open_link` opens the one an `--adapter` names.

use crate::elm::{Answer, ObdError, Protocol};
use crate::link::Link;
use crate::port::J2534Device;
use crate::sim::SimCar;
use std::time::{Duration, Instant};

/// The link to the vehicle through the J2534 device `dev` names (`--adapter openport`, `sim:j2534`): ISO 15765 at
/// 500 kbit/s, as `PassThruLink::connect` opens it.
pub fn open_link(dev: &J2534Device) -> Result<Box<dyn Link>, String> {
    let e = |x: ObdError| x.to_string();
    Ok(match dev {
        J2534Device::Sim => {
            Box::new(PassThruLink::connect(SimPassThru::new(SimCar::new())).map_err(e)?)
        }
        J2534Device::OpenPort(Some(path)) => {
            Box::new(PassThruLink::connect(crate::openport::open_tty(path).map_err(e)?).map_err(e)?)
        }
        // a raw-CAN cable: Icarus's own ISO-TP over it, at OBD-II's 500 kbit/s
        J2534Device::Mongoose(path) => {
            let cable = crate::mongoose::open_tty(path, 500_000).map_err(e)?;
            let label = cable.label();
            Box::new(
                crate::canlink::CanLink::connect(cable, path)
                    .map_err(e)?
                    .named(label)
                    .with_bitrate(Some(500_000)),
            )
        }
        // the port found is named, as `--adapter` did not say it
        J2534Device::OpenPort(None) => {
            let path = crate::openport::find(std::path::Path::new("/sys/bus/usb/devices"))?;
            let at = |x: ObdError| format!("{path}: {x}");
            Box::new(
                PassThruLink::connect(crate::openport::open_tty(&path).map_err(at)?).map_err(at)?,
            )
        }
    })
}

/// The J2534 protocols Icarus uses. J1850 and Chrysler SCI are left out: none of the vehicles in view use them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum J2534Protocol {
    /// Raw CAN frames.
    Can,
    /// ISO-TP over CAN: the device segments and reassembles, and sends flow control.
    Iso15765,
    /// KWP2000 on the K-line.
    Iso14230,
    /// ISO 9141 on the K-line.
    Iso9141,
}

/// One message on a channel: on CAN its 11- or 29-bit ID and the payload (for ISO 15765 the whole reassembled message).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PassThruMsg {
    pub id: u32,
    pub data: Vec<u8>,
}

/// What a device says about itself (J2534 PassThruReadVersion).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Versions {
    pub device: String,
    pub firmware: String,
    pub api: String,
}

/// The operations of a J2534 device Icarus needs, by the standard's meaning.
pub trait PassThru {
    fn version(&mut self) -> Result<Versions, ObdError>;
    /// Opens a channel (PassThruConnect); returns its ID.
    fn connect(&mut self, protocol: J2534Protocol, baud: u32) -> Result<u32, ObdError>;
    fn disconnect(&mut self, channel: u32) -> Result<(), ObdError>;
    /// An ISO 15765 flow-control filter (PassThruStartMsgFilter, FLOW_CONTROL_FILTER): answers from `rx` are taken in, and
    /// flow control for them is sent to `tx`. Returns the filter's ID.
    fn flow_control(&mut self, channel: u32, tx: u32, rx: u32) -> Result<u32, ObdError>;
    /// Sends one message (PassThruWriteMsgs).
    fn write(&mut self, channel: u32, msg: &PassThruMsg, timeout: Duration)
        -> Result<(), ObdError>;
    /// The next message received on the channel, or None when none came in `timeout` (PassThruReadMsgs).
    fn read(&mut self, channel: u32, timeout: Duration) -> Result<Option<PassThruMsg>, ObdError>;
    /// The voltage at the OBD port (PassThruIoctl READ_VBATT), when the device measures it.
    fn battery_voltage(&mut self) -> Result<Option<f64>, ObdError>;
}

/// How long a module has to start answering (ISO 15765-3's P2) and, after it said "response pending" (7F xx 78), how much
/// longer it may take each time (P2*).
const P2: Duration = Duration::from_millis(1000);
const P2_STAR: Duration = Duration::from_millis(5000);
const OBD_FUNCTIONAL: u32 = 0x7DF;

/// A J2534 device as Icarus's link to the vehicle (11-bit CAN at 500 kbit/s, ISO 15765).
pub struct PassThruLink<D: PassThru> {
    dev: D,
    channel: u32,
    name: String,
    /// (tx, rx) pairs that have a flow-control filter.
    pairs: Vec<(u32, u32)>,
}

impl<D: PassThru> PassThruLink<D> {
    /// Opens an ISO 15765 channel and the filters for OBD-II's eight engine-side modules (7E0-7E7 answered from 7E8-7EF).
    pub fn connect(mut dev: D) -> Result<Self, ObdError> {
        let v = dev.version()?;
        let channel = dev.connect(J2534Protocol::Iso15765, 500_000)?;
        let mut pairs = Vec::new();
        for i in 0..8 {
            dev.flow_control(channel, 0x7E0 + i, 0x7E8 + i)?;
            pairs.push((0x7E0 + i, 0x7E8 + i));
        }
        // the battery asked once here, so a device that cannot answer fails to connect; a write asks again (`voltage`)
        dev.battery_voltage()?;
        let name = [v.device, v.firmware]
            .into_iter()
            .filter(|s| !s.is_empty())
            .collect::<Vec<_>>()
            .join(" ");
        Ok(PassThruLink {
            dev,
            channel,
            name,
            pairs,
        })
    }

    /// The device itself, to look at (what a simulated one was sent). Read only: a message written to it past `request`
    /// and `request_to` would skip the Read-Only switch's check (`lock`), so nothing outside this module gets the device
    /// to write with. Dropping the link closes the device.
    ///
    /// ```
    /// use icarus_obd::link::Link;
    /// use icarus_obd::passthru::{PassThru, PassThruLink, SimPassThru};
    /// use icarus_obd::sim::SimCar;
    /// let mut l = PassThruLink::connect(SimPassThru::new(SimCar::new())).unwrap();
    /// l.request(&[0x01, 0x00]).unwrap();
    /// let m = l.device().sent[0].clone();
    /// assert_eq!((m.id, m.data), (0x7DF, vec![0x01, 0x00]));
    /// ```
    ///
    /// Writing to the device through it does not build:
    ///
    /// ```compile_fail,E0596
    /// use icarus_obd::link::Link;
    /// use icarus_obd::passthru::{PassThru, PassThruLink, SimPassThru};
    /// use icarus_obd::sim::SimCar;
    /// let mut l = PassThruLink::connect(SimPassThru::new(SimCar::new())).unwrap();
    /// l.request(&[0x01, 0x00]).unwrap();
    /// let m = l.device().sent[0].clone();
    /// l.device().write(0, &m, std::time::Duration::from_secs(1)).unwrap();
    /// ```
    pub fn device(&self) -> &D {
        &self.dev
    }

    /// Sends `data` to `to` and gathers the answers from IDs `from` accepts: a module that says "response pending"
    /// (7F xx 78) is waited for (P2* each time); with `one`, the first real answer ends it.
    fn exchange(
        &mut self,
        to: u32,
        from: impl Fn(u32) -> bool,
        data: &[u8],
        one: bool,
    ) -> Result<Vec<Answer>, ObdError> {
        self.dev.write(
            self.channel,
            &PassThruMsg {
                id: to,
                data: data.to_vec(),
            },
            P2,
        )?;
        let mut answers = Vec::new();
        let mut deadline = Instant::now() + P2;
        while let Some(left) = deadline.checked_duration_since(Instant::now()) {
            let Some(m) = self.dev.read(self.channel, left)? else {
                break;
            };
            if !from(m.id) {
                continue;
            }
            if m.data.len() >= 3 && m.data[0] == 0x7F && m.data[2] == 0x78 {
                deadline = Instant::now() + P2_STAR;
                continue;
            }
            answers.push(Answer {
                from: format!("{:03X}", m.id),
                data: m.data,
            });
            if one {
                break;
            }
        }
        if answers.is_empty() {
            return Err(ObdError::NoData);
        }
        Ok(answers)
    }
}

impl<D: PassThru> PassThruLink<D> {
    /// The backstop under every write path, as on the ELM327 and CAN links: a request that would change the vehicle goes
    /// only when the session is not read-only and the vehicle's Read-Only switch is off.
    fn read_only_check(&mut self, data: &[u8]) -> Result<(), ObdError> {
        if crate::lock::changes_vehicle(data) {
            crate::lock::writes_allowed(self).map_err(ObdError::ReadOnly)?;
        }
        Ok(())
    }
}

impl<D: PassThru> Link for PassThruLink<D> {
    fn request(&mut self, data: &[u8]) -> Result<Vec<Answer>, ObdError> {
        self.read_only_check(data)?;
        self.exchange(
            OBD_FUNCTIONAL,
            |id| (0x7E8..=0x7EF).contains(&id),
            data,
            false,
        )
    }

    fn request_to(&mut self, tx: u32, rx: u32, data: &[u8]) -> Result<Vec<Answer>, ObdError> {
        self.read_only_check(data)?;
        if tx > 0x7FF || rx > 0x7FF {
            return Err(ObdError::Adapter(format!(
                "{tx:X}/{rx:X} are not 11-bit CAN IDs"
            )));
        }
        if !self.pairs.contains(&(tx, rx)) {
            self.dev.flow_control(self.channel, tx, rx)?;
            self.pairs.push((tx, rx));
        }
        self.exchange(tx, |id| id == rx, data, true)
    }

    fn protocol(&self) -> Protocol {
        Protocol::Can11At500
    }

    fn adapter(&self) -> String {
        self.name.clone()
    }

    /// The battery measured now (`atr 16` on the OpenPort); None when the device cannot answer this time. Never a reading
    /// from when it connected: the write paths judge the battery by this.
    fn voltage(&mut self) -> Option<f64> {
        self.dev.battery_voltage().ok().flatten()
    }
}

/// The simulated car behind a simulated J2534 device: the same modules and answers as `SimCar` behind its simulated ELM327,
/// at the message level a J2534 device works at. Only answers that pass a flow-control filter are delivered, as on a real
/// device.
pub struct SimPassThru {
    car: SimCar,
    channel: Option<u32>,
    /// rx IDs with a filter.
    filters: Vec<u32>,
    inbox: std::collections::VecDeque<PassThruMsg>,
    /// Every message written to the device, for the tests to look at.
    pub sent: Vec<PassThruMsg>,
}

impl SimPassThru {
    pub fn new(car: SimCar) -> SimPassThru {
        SimPassThru {
            car,
            channel: None,
            filters: Vec::new(),
            inbox: Default::default(),
            sent: Vec::new(),
        }
    }

    fn open(&self, channel: u32) -> Result<(), ObdError> {
        if self.channel == Some(channel) {
            Ok(())
        } else {
            Err(ObdError::Adapter(format!("channel {channel} is not open")))
        }
    }
}

impl PassThru for SimPassThru {
    fn version(&mut self) -> Result<Versions, ObdError> {
        Ok(Versions {
            device: "Icarus simulated J2534".into(),
            firmware: "1.0".into(),
            api: "04.04".into(),
        })
    }

    fn connect(&mut self, protocol: J2534Protocol, baud: u32) -> Result<u32, ObdError> {
        if protocol != J2534Protocol::Iso15765 || baud != 500_000 {
            return Err(ObdError::Adapter(
                "the simulated car speaks ISO 15765 at 500 kbit/s".into(),
            ));
        }
        self.channel = Some(1);
        Ok(1)
    }

    fn disconnect(&mut self, channel: u32) -> Result<(), ObdError> {
        self.open(channel)?;
        self.channel = None;
        self.filters.clear();
        Ok(())
    }

    fn flow_control(&mut self, channel: u32, _tx: u32, rx: u32) -> Result<u32, ObdError> {
        self.open(channel)?;
        self.filters.push(rx);
        Ok(self.filters.len() as u32)
    }

    fn write(
        &mut self,
        channel: u32,
        msg: &PassThruMsg,
        _timeout: Duration,
    ) -> Result<(), ObdError> {
        self.open(channel)?;
        self.sent.push(msg.clone());
        let answers: Vec<(u32, Vec<u8>)> = if msg.id == OBD_FUNCTIONAL {
            self.car
                .broadcast_answers(&msg.data)
                .into_iter()
                .filter_map(|(h, d)| u32::from_str_radix(h, 16).ok().map(|id| (id, d)))
                .collect()
        } else {
            self.car
                .module_answers(msg.id, &msg.data)
                .into_iter()
                .map(|d| (msg.id + 8, d))
                .collect()
        };
        for (id, data) in answers {
            if self.filters.contains(&id) {
                self.inbox.push_back(PassThruMsg { id, data });
            }
        }
        Ok(())
    }

    fn read(&mut self, channel: u32, _timeout: Duration) -> Result<Option<PassThruMsg>, ObdError> {
        self.open(channel)?;
        Ok(self.inbox.pop_front())
    }

    fn battery_voltage(&mut self) -> Result<Option<f64>, ObdError> {
        Ok(Some(12.6))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::j1979;

    fn link() -> PassThruLink<SimPassThru> {
        PassThruLink::connect(SimPassThru::new(SimCar::new())).unwrap()
    }

    #[test]
    fn a_read_only_vehicle_is_never_changed_over_j2534() {
        crate::lock::switch_for_test(&[crate::sim::VIN]);
        let mut l = link();
        assert!(matches!(l.request(&[0x04]), Err(ObdError::ReadOnly(_))));
        assert!(matches!(
            l.request_to(0x7C0, 0x7C8, &[0x2E, 0xA0, 0x01, 0x00]),
            Err(ObdError::ReadOnly(_))
        ));
        assert!(l.request(&[0x01, 0x0C]).is_ok(), "reads go");
        crate::lock::switch_for_test(&[]);
    }

    #[test]
    fn obd_services_read_the_same_over_j2534() {
        let mut l = link();
        assert_eq!(l.adapter(), "Icarus simulated J2534 1.0");
        assert_eq!(l.voltage(), Some(12.6));
        assert_eq!(
            j1979::vin(&mut l).unwrap().as_deref(),
            Some(crate::sim::VIN)
        );
        let rpm = l.request(&[0x01, 0x0C]).unwrap();
        assert_eq!(rpm[0].from, "7E8");
        assert_eq!(&rpm[0].data[..2], &[0x41, 0x0C]);
    }

    #[test]
    fn one_module_and_response_pending() {
        let mut l = link();
        // the engine's identity (UDS 22 F1 87), and the instrument cluster, which first answers "response pending"
        let a = l.request_to(0x7E0, 0x7E8, &[0x22, 0xF1, 0x87]).unwrap();
        assert_eq!(a.len(), 1);
        assert_eq!(a[0].data[0], 0x62);
        let m = l.request_to(0x7C0, 0x7C8, &[0x19, 0x02, 0xFF]).unwrap();
        assert_eq!(
            m[0].data[0], 0x59,
            "the pending answer is waited out, not returned: {m:?}"
        );
        assert!(matches!(
            l.request_to(0x702, 0x70A, &[0x3E, 0x00]),
            Err(ObdError::NoData)
        ));
        assert!(
            l.request_to(0x800, 0x808, &[0x3E, 0x00]).is_err(),
            "29-bit IDs are refused"
        );
    }
}
