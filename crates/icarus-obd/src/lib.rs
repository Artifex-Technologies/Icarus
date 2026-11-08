//! Icarus's link to a vehicle. `port` reaches the adapter (USB, Bluetooth, Wi-Fi or the simulated car), `elm` speaks an
//! ELM327/STN adapter's language and `passthru` a J2534 device's; either is a `link::Link`, and the services above it (`j1979`
//! the standard OBD-II services, `uds` the modules') ask through that. Most of this crate only reads (values, codes, the
//! vehicle's identity); writes live in their own modules, each with its own allow-list and safeguards: `actuator` (output
//! tests), `clear` (clearing codes) and `settings` (module settings).

pub mod actuator;
pub mod audit;
pub mod bus;
pub mod charging;
pub mod clear;
pub mod devices;
pub mod dtc;
pub mod elm;
pub mod garage;
pub mod j1979;
pub mod link;
pub mod lookup;
pub mod mode06;
pub mod nhtsa;
pub mod passthru;
pub mod port;
pub mod power_guard;
pub mod report;
pub mod settings;
pub mod sim;
pub mod socketcan;
pub mod uds;
pub mod vehicles;
pub mod vin;
