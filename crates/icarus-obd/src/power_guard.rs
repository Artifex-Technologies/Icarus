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
        "cat".into(),
    ]
}

impl Inhibit {
    /// Holds the lock until dropped. Fails when logind refuses or systemd-inhibit is missing (a caller that writes a module's
    /// memory must not go on then; a short test may).
    pub fn take(why: &str) -> Result<Inhibit, String> {
        if cfg!(any(test, feature = "test-home")) {
            return Ok(Inhibit(None));
        }
        // held by cat on our pipe (`tied`): it ends with this program, however it ends
        let mut child = tied("systemd-inhibit", &args(why))
            .map_err(|e| format!("could not keep the computer awake (systemd-inhibit: {e})"))?;
        std::thread::sleep(std::time::Duration::from_millis(150));
        if let Ok(Some(st)) = child.try_wait() {
            return Err(format!(
                "could not keep the computer awake (systemd-inhibit ended: {st})"
            ));
        }
        Ok(Inhibit(Some(child)))
    }

    /// While a drive is recorded (ICA-208): sleep, idle actions and the lid switch blocked, as the window's recording always
    /// did, but not a shutdown asked for (the recording's file is closed then). Fails when logind refuses (a service outside
    /// the seat's session may not be let block sleep): a recording goes on without it, and says so.
    ///
    /// The lock lives exactly as long as this program: what holds it is `cat` reading a pipe only this program writes
    /// (`tied`), so however the program ends (the window closed, a crash, SIGKILL) the pipe closes, cat ends and the
    /// inhibitor with it. Never `sleep infinity`, which would outlive a program that died and keep the laptop awake.
    pub fn recording(why: &str) -> Result<Inhibit, String> {
        if cfg!(any(test, feature = "test-home")) {
            return Ok(Inhibit(None));
        }
        let mut child = tied("systemd-inhibit", &recording_args(why))
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

/// The systemd-inhibit command line while recording a drive: what it runs is `cat` on our pipe (`tied`).
pub fn recording_args(why: &str) -> Vec<String> {
    vec![
        "--what=sleep:idle:handle-lid-switch".into(),
        "--who=Icarus".into(),
        format!("--why={why}"),
        "--mode=block".into(),
        "cat".into(),
    ]
}

/// `program` started with its input a pipe from this program (kept in the child's `stdin`, never handed to another
/// child: Rust's pipes close on exec): when this program ends, however it ends, the pipe closes and a `cat` at the end of
/// it sees the end of its input and stops.
pub fn tied(program: &str, args: &[String]) -> std::io::Result<Child> {
    Command::new(program)
        .args(args)
        .stdin(Stdio::piped())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
}

impl Drop for Inhibit {
    fn drop(&mut self) {
        if let Some(c) = self.0.as_mut() {
            drop(c.stdin.take());
            let _ = c.kill();
            let _ = c.wait();
        }
    }
}

#[cfg(test)]
mod tests {
    /// ICA-208 review: the recording's lock ends with the program. Its holder is `cat` on our pipe: once the pipe's writing
    /// end closes (as the kernel closes it when a program dies, SIGKILL included) cat ends by itself, nothing killing it.
    #[test]
    fn the_recording_lock_ends_when_its_pipe_closes() {
        let a = super::recording_args("Recording a drive");
        assert_eq!(a.last().map(String::as_str), Some("cat"));
        assert!(
            !a.iter().any(|x| x == "infinity"),
            "nothing that outlives the program"
        );
        assert!(a[0].contains("handle-lid-switch") && !a[0].contains("shutdown"));
        let mut c = super::tied("cat", &[]).expect("cat");
        assert!(
            c.try_wait().unwrap().is_none(),
            "held while the pipe is open"
        );
        drop(c.stdin.take());
        let end = std::time::Instant::now() + std::time::Duration::from_secs(5);
        while c.try_wait().unwrap().is_none() {
            assert!(std::time::Instant::now() < end, "cat outlived its pipe");
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
    }

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
