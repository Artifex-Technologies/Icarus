//! The write confirmations waiting for the person's yes: a clear, a service routine, a setting change. Each is tied to the
//! connection it was asked on (ICA-105). It closes when that connection ends or another one starts (`Disconnected`,
//! `Connected`) and when anything on the link fails (`Failed`: a live reading that broke off, a connection that did not
//! come up), so a yes is never given to a vehicle that came on after the question; and the command its yes sends carries the
//! connection, which the worker checks before anything goes out.

use crate::worker::{Connection, Event};
use crate::ClearTarget;
use icarus_obd::service::ServiceDef;
use icarus_obd::settings::SettingDef;

/// A confirmation, and the connection it was asked on.
#[derive(Clone)]
pub struct Asked<T> {
    pub what: T,
    pub on: Connection,
}

#[derive(Default)]
pub struct Confirms {
    /// A clear waiting for "Save a report and clear": what is cleared, and its name in words.
    pub clear: Option<Asked<(ClearTarget, String)>>,
    /// A service routine waiting for "Run".
    pub service: Option<Asked<ServiceDef>>,
    /// A setting change waiting for "Change it": the setting and the value's name (None: undo its last change).
    pub setting: Option<Asked<(SettingDef, Option<String>)>>,
}

impl Confirms {
    /// Closes every confirmation when the link ends, starts again or fails; anything else leaves them open.
    pub fn on_event(&mut self, e: &Event) {
        if matches!(
            e,
            Event::Disconnected | Event::Connected(_) | Event::Failed(_)
        ) {
            *self = Confirms::default();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use icarus_obd::sim;

    fn none_open(c: &Confirms) -> bool {
        c.clear.is_none() && c.service.is_none() && c.setting.is_none()
    }

    pub(crate) fn all_open() -> Confirms {
        let on = Connection {
            number: 1,
            vin: Some(sim::VIN.into()),
        };
        Confirms {
            clear: Some(Asked {
                what: (None, "every engine and transmission module (OBD-II)".into()),
                on: on.clone(),
            }),
            service: Some(Asked {
                what: sim::services().remove(0),
                on: on.clone(),
            }),
            setting: Some(Asked {
                what: (sim::settings().remove(0), Some("off".into())),
                on,
            }),
        }
    }

    /// ICA-105: the clear, routine and setting dialogs open, then the link drops: each one is gone, so no yes can reach the
    /// vehicle that comes on next. A failure on the link closes them too; a status line leaves them. (A new connection
    /// closing them is shown with the worker's own Connected event, in worker.rs.)
    #[test]
    fn the_dialogs_are_gone_after_disconnected() {
        let mut c = all_open();
        c.on_event(&Event::Status("Reading the settings...".into()));
        c.on_event(&Event::Dropped("7E9: a message that did not add up".into()));
        assert!(
            c.clear.is_some() && c.service.is_some() && c.setting.is_some(),
            "still open"
        );
        c.on_event(&Event::Disconnected);
        assert!(none_open(&c));
        let mut c = all_open();
        c.on_event(&Event::Failed("the adapter stopped answering".into()));
        assert!(none_open(&c), "a failed live link closes them");
    }
}
