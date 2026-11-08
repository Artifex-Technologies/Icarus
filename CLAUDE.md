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
- Icarus is meant to do more than read (my decision, 2026-11-08: "We built it with the intention of doing more than that"):
  actuator tests, clearing codes, changing a module's settings, coding and flashing all belong in it. Each kind of write has
  its own module and allow-list (today: actuator tests in `crates/icarus-obd/src/actuator.rs`, clearing in `clear.rs`, a
  report saved first) and the same safeguards: the value or image there before is read and kept first, with a way back;
  only what was learned on my own vehicle or verified for it (no guessed IDs in a real vehicle's profile: an unverified ID
  could drive something else); conditions (parked, engine off where it matters, voltage held up); wired for anything that
  writes memory; the airbag modules never. A new kind of write is built that way, not left out. Flashing and coding still
  wait for docs/PLAN.md section 4's conditions, built and tested on a bench ECU first.
  Disassembling ECU firmware happens in Limbo, as for any binary.
- The plan and its order: [docs/PLAN.md](docs/PLAN.md).
