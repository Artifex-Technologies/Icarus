# Contracts

What other projects may rely on from Icarus, and what it uses from them. One section per contract.

## Provided by Icarus

### `icarus mcp` (a Model Context Protocol server)

stdin/stdout, seven read-only tools: `explain_code`, `vehicle_check`, `vehicle_modules`, `module_data`, `learning_sessions`,
`drive_logs`, `tires_gears`.
`vehicle_check` gives the VIN with its last 6 characters (the serial number) as `******` unless it is called with
`show_vin: true`. No tool gives a module's serial number: `learning_sessions` gives sessions with the VIN and serial
numbers masked (bytes as `2A`, the line marked `[VIN masked]` or `[serial number masked]`), and `module_data` never reads
`22 F190` or `22 F18C`.
Nothing that clears codes, runs a test or writes is offered to a model; a test fails if a tool with such a name appears.
Nothing from an immobilizer, smart-key or certification module reaches a model: `vehicle_modules` does not ask them,
`module_data` sends them no data request, and a learning session never holds their messages. The
adapter comes from `ICARUS_ADAPTER` in the server's environment (`sim` for the simulated vehicle); `ICARUS_READ_ONLY=1` there
holds the session's Read-Only floor as for any command. The definition file for an
assistant and the tools in full: [README.md](../README.md), "Sarina (or another assistant)".
Used by: Sarina (through a definition file named by `SARINA_MCP_FILE`, opt-in).

### The build for packages

`cargo build --release -p icarus -p icarus-app` gives `icarus` (the command) and `icarus-app` (the window); `cargo test
--workspace` must pass with no vehicle, adapter or network, and with HOME pointed at a scratch folder. `packaging/` holds the
desktop entry, the icons, the udev rule `70-icarus-tuners.rules` (the person at the seat may open the listed tuners and
cables) and the user service `icarus-record.service` (`icarus record --auto`; installed to `/usr/lib/systemd/user/`, never
enabled by the package: I turn it on with `systemctl --user enable --now icarus-record`). `VERSION` is Icarus's own version. Used by: Labyrinth OS's `icarus-git` recipe (and through it Agora's catalog).

### Vehicle profiles (`crates/icarus-obd/vehicles/*.toml`, `~/.config/icarus/vehicles/`)

One TOML file per make (module addresses, quirks, `[codes]`, `[[test]]`, `[[setting]]`, `[[service]]`), picked by the VIN; a user file of the
same name replaces a built-in one. Addresses from public sources carry `checked = false` until seen on a vehicle. A
`[[module]]`'s `tx`/`rx` are 11-bit CAN IDs (up to `0x7FF`) or 29-bit ones written with all eight digits (`0x18DA30F1`,
answered from `0x18DAF130` unless `rx` is given); `[[range]]` (`from`, `to`, `answer_offset`, `name`) names up to 64
11-bit request IDs a scan asks, each answered `answer_offset` higher. A `[[module]]`'s optional `kind` says what it is;
`"immobilizer"`, `"smart-key"` and `"certification"` fence it by address (identity reads only; no test, setting, routine
or clear; nothing learned from it), and an entry of any profile aimed at it is left out when the profiles load. A
`[[setting]]`'s optional `mask` (as long as its values) names its bits of an identifier other settings share; a change,
its read-back and its undo touch only those. `[bus]` names IDs the modules send unasked, by
their hex ID (three digits, or eight for 29-bit): `"0B4" = "wheel speeds"`; the Bus page shows them, with each module's
request and answer IDs, in its "what" column (a user file adds to the built-in table).
Used by: Icarus itself, and anyone writing down a module found on a vehicle.

## Used by Icarus (provided elsewhere)

| Contract | Provider | Used here by |
| --- | --- | --- |
| The `yggdrasil-ui` crate (git dependency pinned to a commit) | Yggdrasil | `crates/icarus-app` (the window's look) |
| `labyrinth-ask-sarina` (a read-only question, answer on stdout) and `sarina` present | Labyrinth OS / Sarina | the Learning page's suggestions (`crates/icarus-obd/src/learning.rs`), optional |
| Limbo's guests (Windows-only tuner software under Wine) | Limbo | [INTERFACES.md](INTERFACES.md) |
| NHTSA's public vPIC and recalls services (asked first) | NHTSA | the VIN look-up and recalls |
