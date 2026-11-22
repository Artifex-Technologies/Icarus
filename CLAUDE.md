# Icarus: notes for Claude Code

## What this is

Icarus is an all-in-one vehicle tool in Rust for Linux: trouble codes and live data from every module (OBD-II, UDS,
KWP2000), actuator tests and module settings with safeguards, a CAN bus view, and tuning files (TunerPro XDF maps, diffs,
edits with checksums). A Cargo workspace: `crates/icarus-core` (images, definitions, formulas, XDF, module data),
`crates/icarus-obd` (adapters, the ELM327/STN language, OBD-II, UDS, the simulated vehicle), `crates/icarus` (the command),
`crates/icarus-app` (the window). What it is built to do and the rules it follows: [docs/DESIGN.md](docs/DESIGN.md).

## Build and test

```sh
systemd-run --user --scope -q -p MemoryMax=8G -p MemorySwapMax=0 env CARGO_BUILD_JOBS=4 nice -n 19 cargo test --workspace
systemd-run --user --scope -q -p MemoryMax=8G -p MemorySwapMax=0 env CARGO_BUILD_JOBS=4 nice -n 19 cargo build --release
cargo fmt --all --check     # the whole workspace is in rustfmt's default style (since 2026-11-21): keep it so
```

`cargo test --workspace`, `cargo clippy --workspace --all-targets -- -D warnings` and `cargo fmt --all --check` must pass; no test needs a vehicle, an adapter or a network (the simulated vehicle in
`crates/icarus-obd/src/sim.rs` stands in). Trying the CLI on `sim` writes reports and the action log into the real home: run
it with HOME/XDG_STATE_HOME pointed at a scratch folder. There is no CI workflow in this repository; Labyrinth OS's
`icarus-git` recipe runs the tests when it builds the package.

## Rules

- **Privacy and vehicles.** Before pushing, check the diff names no host name, network address, SSID, MAC or Bluetooth
  address, and **no VIN, ECU image, calibration or log from a real vehicle** (they stay outside the repository; `.gitignore`
  keeps `*.bin` and `/private` out). The repository is public, which is why nothing from a real vehicle may ever be committed.
  Write docs in the first person as the owner, and "Sarina", not "her".
- Push every commit as you make it; stage files by explicit path; `git pull --rebase` first.
- **Limits:** no one-click emissions "delete" features, no immobiliser bypass, no odometer changes. Thresholds and
  calibration values are ordinary maps; emissions readings are ordinary live data.
- **Clean-room:** the reference projects (LibreTune, FoxTune, ZedSuite, Denso ECU Toolkit, ecutools, HyperTuner: GPL; Atlas:
  proprietary) are for ideas and file formats only, never code. Third-party definition files and ECU images are never
  committed; tests use small files written for them.
- **Writes.** Icarus is meant to do more than read (my decision, 2026-11-08): actuator tests, clearing codes, changing a
  module's settings, coding and flashing all belong in it. Each kind of write has its own module and allow-list (actuator
  tests in `crates/icarus-obd/src/actuator.rs`, clearing in `clear.rs`, settings in `settings.rs`, service routines in
  `service.rs`) and the same safeguards (each calls `lock::writes_allowed`, and the adapter link refuses changes to a
  Read-Only vehicle underneath):
  the value or image there before is read and kept first, with a way back; only what was learned on my own vehicle or
  verified for it (no guessed IDs in a real vehicle's profile: an unverified ID could drive something else); conditions
  (parked, engine off where it matters, voltage held up); wired for anything that writes memory; the airbag modules never.
  A new kind of write is built that way, not left out. Flashing and coding wait for the conditions in docs/DESIGN.md
  ("Never brick a module", "Writing a module (flash): the conditions"), built and tested on a bench ECU first.
- Disassembling ECU firmware happens in Limbo, as for any binary.
- Nothing that clears, tests or writes is ever offered through `icarus mcp`.

## Contracts

- What Icarus provides (`icarus mcp`, the build Labyrinth OS packages, vehicle profiles) and what it uses (yggdrasil-ui,
  Sarina's question command, Limbo, NHTSA): [docs/CONTRACTS.md](docs/CONTRACTS.md).

## Open work

docs/plans/BACKLOG.md and docs/plans/LOG.md (private; ask the owner).

## Know-how

- Icarus is packaged as `icarus-git` (the recipe is in Labyrinth_OS `packages/pkgbuilds/icarus-git`, with the store's copy in
  `profile/airootfs/usr/share/labyrinth/pkgbuilds`), published in Labyrinth's `[labyrinth]` package repository and listed in
  Agora's catalog, so it updates through Agora and pacman like the other apps.
- Adapters: `sim`, `bt:MAC[#channel]`, `/dev/ttyUSB0[@baud]`, `tcp:HOST:PORT`, `can:can0` (SocketCAN, `canlink`'s own
  ISO-TP; tests use `canlink::SimBus`), `slcan:/dev/ttyACM0[@baud][#bitrate]` (bus watching only, listen-only `L`, never
  `O`; tests use `slcan::recorded::Session`); `ICARUS_ADAPTER` sets a default.
- `crates/icarus-obd/src/uds.rs` lets only reading services out; each write lives in its own module behind its allow-list.
- Interfaces and tuners with Windows-only software, and how Icarus reaches them: [docs/INTERFACES.md](docs/INTERFACES.md).
