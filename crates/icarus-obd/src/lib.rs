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
pub mod doctor;
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
pub mod systree;
pub mod uds;
pub mod units;
pub mod vehicledata;
pub mod vehicles;
pub mod vin;

/// The environment variables Icarus's files are found by (HOME, XDG_CONFIG_HOME, XDG_STATE_HOME...): read from the
/// environment, except in tests, so no test reads or writes the real home or config folder (ICA-225):
///
/// - in this crate's own tests, HOME is a folder of the test run's own and every XDG variable is unset;
/// - with the `test-home` feature (which icarus-app and the icarus command turn on for their tests only, through their
///   dev-dependencies: a release build never has it), a variable is taken only when it is a folder in the system's
///   temporary folder (a test's own home, as icarus-app's `TestHome` sets); otherwise HOME is this test run's own folder
///   and an XDG variable is unset.
///
/// The window and the command find their own files (window.toml, the remembered adapter) through this too, so their tests
/// get the same guard.
pub fn env(name: &str) -> Option<std::ffi::OsString> {
    #[cfg(test)]
    return (name == "HOME").then(|| test_home().into_os_string());
    #[cfg(all(feature = "test-home", not(test)))]
    return std::env::var_os(name)
        .filter(|v| in_test_area(std::path::Path::new(v)))
        .or_else(|| (name == "HOME").then(|| test_home().into_os_string()));
    #[cfg(not(any(test, feature = "test-home")))]
    std::env::var_os(name)
}

/// Whether this build keeps its tests out of the real home (this crate's own tests, or the `test-home` feature). The
/// window's and the command's tests assert it, so a test build that lost the feature fails instead of writing the home.
pub fn test_home_active() -> bool {
    cfg!(any(test, feature = "test-home"))
}

/// Whether `p` is a folder a test may use as a home: an absolute path inside the system's temporary folder, with no `..`
/// that could lead out of it.
#[cfg(any(test, feature = "test-home"))]
pub(crate) fn in_test_area(p: &std::path::Path) -> bool {
    p.is_absolute()
        && !p
            .components()
            .any(|c| matches!(c, std::path::Component::ParentDir))
        && p.starts_with(std::env::temp_dir())
}

/// This test run's own home folder (made when first asked), in the system's temporary folder.
#[cfg(any(test, feature = "test-home"))]
pub(crate) fn test_home() -> std::path::PathBuf {
    let d = std::env::temp_dir().join(format!("icarus-obd-test-home-{}", std::process::id()));
    let _ = std::fs::create_dir_all(&d);
    d
}

#[cfg(test)]
mod tests {
    /// ICA-225: what a test may take for a home: a folder in the temporary folder, never one outside it or one that leads
    /// out of it with `..`.
    #[test]
    fn a_test_home_is_in_the_temporary_folder() {
        let tmp = std::env::temp_dir();
        assert!(super::in_test_area(&tmp.join("icarus-app-x/state")));
        assert!(super::in_test_area(&super::test_home()));
        assert!(!super::in_test_area(std::path::Path::new(
            "/home/someone/.local/state"
        )));
        assert!(!super::in_test_area(&tmp.join("../home/someone")));
        assert!(!super::in_test_area(std::path::Path::new("relative/state")));
        assert!(super::test_home_active());
    }
}
