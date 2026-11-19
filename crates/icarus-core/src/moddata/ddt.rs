//! DDT2000 ECU files (`ecus/<name>.xml`, Renault, Dacia, Nissan): **not built yet** (module data from DDT2000 files; see
//! `docs/DESIGN.md`). DDT2000's databases are Renault's material: Icarus will only read a file the user gives, never ship one
//! or use one in its tests (tests use tiny XML written for them).
//!
//! What the importer will do, from the file format (ideas and format facts only, as with every reference project):
//! - `Target@Name` names the module; `CAN/SendId/CANId@Value` and `ReceiveId` (decimal) give tx and rx; no CAN node means a
//!   K-line module, kept but not reachable yet.
//! - A request is kept only when its `SentBytes` are exactly `22 hi lo`, `21 id` or `1A id`, with no `Sent` data items, no
//!   `ManuelSend` and a `DenyAccess` that allows the default session; anything else is left out with what it would do.
//! - `Received` data items: `FirstByte` is 1-based over the whole reply, so `byte = FirstByte - 1 - (1 + echo length)`
//!   (an item inside the echo is left out); `BitOffset` is `bit`; `Bytes@count` (x 8) or `Bits@count` is `bits`;
//!   `Bytes@ascii` is text; `Endian` Big is `msb`, Little is `ddt-little` (the request's `Endian` is the default).
//! - `Scaled@Step/Offset/DivideBy` become `mul/bias/div` (the offset is added before the divide); `List/Item` become states
//!   for unscaled values; an unscaled value with no list shows as hex. Signed only at exactly 8 or 16 bits, as DDT tools
//!   read them.
//! - `ReplyBytes` becomes `sample`; the first screen that shows a value gives its `path`.

use super::Imported;

/// Reads a DDT2000 ECU file. Not built yet: always refuses, saying so.
pub fn parse(_text: &str) -> Result<Imported, String> {
    Err("DDT2000 files are not read yet (see docs/DESIGN.md, Known limitations)".into())
}

#[cfg(test)]
mod tests {
    #[test]
    fn says_it_is_not_built_yet() {
        assert!(super::parse("<Target/>")
            .unwrap_err()
            .contains("not read yet"));
    }
}
