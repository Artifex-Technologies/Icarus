//! Icarus's link to a vehicle. `port` reaches the adapter (USB, Bluetooth, Wi-Fi or the simulated car), `elm` speaks an
//! ELM327/STN adapter's language, `passthru` a J2534 device's and `canlink` ISO-TP over a Linux CAN interface; each is a
//! `link::Link`, and the services above it (`j1979` the standard OBD-II services, `uds` the modules') ask through that. Most of this crate only reads (values, codes, the
//! vehicle's identity); writes live in their own modules, each with its own allow-list and safeguards: `actuator` (output
//! tests), `clear` (clearing codes), `settings` (module settings) and `service` (routines), and `fence` keeps all of them, and
//! the bus learner, away from the immobilizer, smart-key and certification modules. `moddata` reads each module's own live
//! values from definition files, with read-only requests only.

pub mod actuator;
pub mod asbuilt;
pub mod audit;
pub mod benchmake;
pub mod bus;
pub mod canlink;
pub mod charging;
pub mod clear;
pub mod devices;
pub mod drive;
pub mod drivelog;
pub mod dtc;
pub mod elm;
pub mod fence;
pub mod fuel;
pub mod garage;
pub mod gps;
pub mod health;
pub mod history;
pub mod j1939;
pub mod j1979;
pub mod learning;
pub mod link;
pub mod lock;
pub mod lookup;
pub mod maintenance;
pub mod moddata;
pub mod mode06;
pub mod mongoose;
pub mod naming;
pub mod nhtsa;
pub mod notify;
pub mod obdex;
pub mod openport;
pub mod passthru;
pub mod port;
pub mod power_guard;
pub mod prune;
pub mod record;
pub mod report;
pub mod roles;
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

/// The environment variables Icarus's files are found by (HOME, XDG_CONFIG_HOME, XDG_STATE_HOME...): read from the
/// environment, except in this crate's tests, where HOME is a folder of the test run's own and every XDG variable is
/// unset, so no test reads or writes the real home or config folder.
pub(crate) fn env(name: &str) -> Option<std::ffi::OsString> {
    #[cfg(test)]
    return (name == "HOME").then(|| test_home().into_os_string());
    #[cfg(not(test))]
    std::env::var_os(name)
}

/// This test run's own home folder (made when first asked), in the system's temporary folder.
#[cfg(test)]
pub(crate) fn test_home() -> std::path::PathBuf {
    let d = std::env::temp_dir().join(format!("icarus-obd-test-home-{}", std::process::id()));
    let _ = std::fs::create_dir_all(&d);
    d
}
