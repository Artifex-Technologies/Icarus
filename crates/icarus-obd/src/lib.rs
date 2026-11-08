//! Icarus's link to a vehicle. `port` reaches the adapter (USB, Bluetooth, Wi-Fi or the simulated car), `elm` speaks an
//! ELM327/STN adapter's language and `passthru` a J2534 device's; either is a `link::Link`, and the services above it (`j1979`
//! the standard OBD-II services, `uds` the modules') ask through that. This crate only reads: it asks for values, codes and the
//! vehicle's identity, and never clears codes or writes to a module.

pub mod actuator;
pub mod audit;
pub mod bus;
pub mod charging;
pub mod clear;
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
pub mod sim;
pub mod socketcan;
pub mod uds;
pub mod vehicles;
pub mod vin;
