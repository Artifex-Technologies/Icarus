# Contracts

What other projects may rely on from Icarus, and what it uses from them. One section per contract.

## Provided by Icarus

### `icarus mcp` (a Model Context Protocol server)

stdin/stdout, five read-only tools: `explain_code`, `vehicle_check`, `vehicle_modules`, `learning_sessions`, `tires_gears`.
Nothing that clears codes, runs a test or writes is offered to a model; a test fails if a tool with such a name appears. The
adapter comes from `ICARUS_ADAPTER` in the server's environment (`sim` for the simulated vehicle). The definition file for an
assistant and the tools in full: [README.md](../README.md), "Sarina (or another assistant)".
Used by: Sarina (through a definition file named by `SARINA_MCP_FILE`, opt-in).

### The build for packages

`cargo build --release -p icarus -p icarus-app` gives `icarus` (the command) and `icarus-app` (the window); `cargo test
--workspace` must pass with no vehicle, adapter or network, and with HOME pointed at a scratch folder. `packaging/` holds the
desktop entry, the icons and the udev rule `70-icarus-tuners.rules` (the person at the seat may open the listed tuners and
cables). `VERSION` is Icarus's own version. Used by: Labyrinth OS's `icarus-git` recipe (and through it Agora's catalog).

### Vehicle profiles (`crates/icarus-obd/vehicles/*.toml`, `~/.config/icarus/vehicles/`)

One TOML file per make (module addresses, quirks, `[codes]`, `[[test]]`, `[[setting]]`), picked by the VIN; a user file of the
same name replaces a built-in one. Addresses from public sources carry `checked = false` until seen on a vehicle.
Used by: Icarus itself, and anyone writing down a module found on a vehicle.

## Used by Icarus (provided elsewhere)

| Contract | Provider | Used here by |
| --- | --- | --- |
| The `yggdrasil-ui` crate (git dependency pinned to a commit) | Yggdrasil | `crates/icarus-app` (the window's look) |
| `labyrinth-ask-sarina` (a read-only question, answer on stdout) and `sarina` present | Labyrinth OS / Sarina | the Learning page's suggestions (`crates/icarus-obd/src/learning.rs`), optional |
| Limbo's guests (Windows-only tuner software under Wine) | Limbo | [INTERFACES.md](INTERFACES.md) |
| NHTSA's public vPIC and recalls services (asked first) | NHTSA | the VIN look-up and recalls |
