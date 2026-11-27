//! Icarus's link to a vehicle. `port` reaches the adapter (USB, Bluetooth, Wi-Fi or the simulated car), `elm` speaks an
//! ELM327/STN adapter's language, `passthru` a J2534 device's and `canlink` ISO-TP over a Linux CAN interface; each is a
//! `link::Link`, and the services above it (`j1979` the standard OBD-II services, `uds` the modules') ask through that. Most of this crate only reads (values, codes, the
//! vehicle's identity); writes live in their own modules, each with its own allow-list and safeguards: `actuator` (output
//! tests), `clear` (clearing codes) and `settings` (module settings). `moddata` reads each module's own live values from
//! definition files, with read-only requests only.

pub mod actuator;
pub mod asbuilt;
pub mod audit;
pub mod bus;
pub mod canlink;
pub mod charging;
pub mod clear;
pub mod devices;
pub mod drivelog;
pub mod dtc;
pub mod elm;
pub mod garage;
pub mod gps;
pub mod history;
pub mod j1939;
pub mod j1979;
pub mod learning;
pub mod link;
pub mod lock;
pub mod lookup;
pub mod moddata;
pub mod mode06;
pub mod mongoose;
pub mod naming;
pub mod nhtsa;
pub mod obdex;
pub mod openport;
pub mod passthru;
pub mod port;
pub mod power_guard;
pub mod prune;
pub mod record;
pub mod report;
pub mod service;
pub mod settings;
pub mod sim;
pub mod slcan;
pub mod socketcan;
pub mod uds;
pub mod units;
pub mod vehicledata;
pub mod vehicles;
pub mod vin;
