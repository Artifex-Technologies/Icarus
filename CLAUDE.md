# Icarus: notes for Claude Code

Push every commit as you make it; don't ask me first. Before pushing, check the diff names no host name, network address,
SSID, MAC or Bluetooth address, and **no VIN, ECU image, calibration or log from a real vehicle** (they stay outside the
repository; `.gitignore` keeps `*.bin` and `/private` out).

- This repository is **private** and stays out of the public package repository and Agora's catalog: Icarus is installed on
  my own machines only.
- `cargo test --workspace` must pass; no test needs a vehicle, an adapter or a network (the simulated vehicle in
  `crates/icarus-obd/src/sim.rs` stands in). Trying the CLI on `sim` writes reports and the action log into the real home:
  run it with HOME/XDG_STATE_HOME pointed at a scratch folder.
- Clean-room: the reference projects (LibreTune, FoxTune, ZedSuite, Denso ECU Toolkit, ecutools, HyperTuner: GPL; Atlas:
  proprietary) are for ideas and file formats only, never code. Third-party definition files and ECU images are never
  committed; tests use small files written for them.
- Limits: no one-click emissions "delete" features, no immobilizer bypass, no odometer changes. Thresholds and calibration
  values are ordinary maps; emissions readings are ordinary live data.
- The only writes to a vehicle are actuator tests (only through `crates/icarus-obd/src/actuator.rs`) and clearing codes
  (only through `clear.rs`, a report saved first), with their allow-lists (the airbag ban, conditions, time limit, return of control on every path). Do not widen them without the owner's say; no
  guessed test IDs in a real vehicle's profile (unverified IDs could drive something else). Flashing and coding wait for
  docs/PLAN.md section 4's conditions, built and tested on a bench ECU.
  Disassembling ECU firmware happens in Limbo, as for any binary.
- The plan and its order: [docs/PLAN.md](docs/PLAN.md).
