//! Icarus's link to a vehicle. `port` reaches the adapter (USB, Bluetooth, Wi-Fi or the simulated car), `elm` speaks the
//! adapter's language, and `j1979` the standard OBD-II services. This crate only reads: it asks for values, codes and the
//! vehicle's identity, and never clears codes or writes to a module.

pub mod elm;
pub mod j1979;
pub mod port;
pub mod sim;
pub mod uds;
