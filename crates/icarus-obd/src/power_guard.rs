//! Keeping the computer awake while Icarus changes something on a vehicle. A laptop that suspends in the middle (its lid
//! closed, a sleep timer, the power key) leaves a module half-controlled, or, while flashing, half-written: one of the
//! common ways a module is bricked. `Inhibit::take` asks systemd-logind to block sleep, hibernation, idle actions, the lid
//! switch and the power/suspend keys until it is dropped.

use std::process::{Child, Command, Stdio};

pub struct Inhibit(Option<Child>);

/// The systemd-inhibit command line.
pub fn args(why: &str) -> Vec<String> {
    vec![
        "--what=sleep:idle:shutdown:handle-lid-switch:handle-power-key:handle-suspend-key:handle-hibernate-key".into(),
        "--who=Icarus".into(),
        format!("--why={why}"),
        "--mode=block".into(),
        "sleep".into(),
        "infinity".into(),
    ]
}

impl Inhibit {
    /// Holds the lock until dropped. Fails when logind refuses or systemd-inhibit is missing (a caller that writes a module's
    /// memory must not go on then; a short test may).
    pub fn take(why: &str) -> Result<Inhibit, String> {
        if cfg!(test) {
            return Ok(Inhibit(None));
        }
        let mut child = Command::new("systemd-inhibit")
            .args(args(why))
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .map_err(|e| format!("could not keep the computer awake (systemd-inhibit: {e})"))?;
        std::thread::sleep(std::time::Duration::from_millis(150));
        if let Ok(Some(st)) = child.try_wait() {
            return Err(format!(
                "could not keep the computer awake (systemd-inhibit ended: {st})"
            ));
        }
        Ok(Inhibit(Some(child)))
    }
}

impl Drop for Inhibit {
    fn drop(&mut self) {
        if let Some(c) = self.0.as_mut() {
            let _ = c.kill();
            let _ = c.wait();
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn what_is_blocked() {
        let a = super::args("Flashing the engine module");
        assert!(
            a[0].contains("sleep")
                && a[0].contains("handle-lid-switch")
                && a[0].contains("shutdown")
        );
        assert!(
            a.contains(&"--mode=block".to_string())
                && a.contains(&"--why=Flashing the engine module".to_string())
        );
    }
}
