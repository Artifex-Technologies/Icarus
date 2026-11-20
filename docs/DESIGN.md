# Icarus: what it is built to do, and the rules it follows

Factory ECUs first, open ECUs second (for open ECUs, LibreTune is the tool, not something rebuilt here). Every step reads
before it writes, and nothing writes to a module until reading, checking and restoring are proven on a bench.

## Scope: an all-in-one vehicle tool

One tool for everything a vehicle's electronics need, in place of a code reader, a dealer tool (Techstream, FORScan), a
bidirectional shop scanner, a tuning suite and a bus analyser: on Linux, for many makes, without per-vehicle licences.

| Area | What Icarus has |
| --- | --- |
| Codes | every module (UDS, KWP2000), meanings, where to start, parts and look-up links, freeze frame, readiness, reports, clearing (report first), Mode 06 test results (CAN) |
| Live data | 51 standard values with graphs; recording a drive from the window (every value the vehicle offers: the fast ones every sweep, the rest in turn; laptop kept awake; goes on after the adapter drops); the Logs page (fuel trims by driving condition, charts on one time line, following a recording as it is written); `drive_logs` for assistants |
| Bidirectional | actuator tests, fenced (hold-to-run, conditions, airbag ban) |
| Configuration | tires/gears calculator, speedometer check, battery and charging check |
| Tuning | map viewer, tune diff (XDF) |
| Vehicle info | VIN decoder, make profiles, named vehicles, vehicle folders, NHTSA model/engine (asks first) and recalls |
| Bus | monitor (OBDLink monitor mode, Linux SocketCAN): ID table with changing bytes, diagnostics in words, tests learned from another tool |
| Hardware | ELM327/STN over USB, Bluetooth, Wi-Fi |
| Vehicles | 12 makes + generic, addresses from public sources |

## The foundation (2026-11-07)

- `icarus-core`: ROM images (original and edited kept side by side, byte diff), definitions (maps, curves, values, switches),
  formulas both ways (stored number to real unit and back, nearest storable value, definition limits), the TunerPro `.xdf`
  reader. Checked on a real definition: 3,683 of 3,703 elements of a Siemens MS43 XDF load in 0.3 s, and `diff` of a stock
  and a modified image named the one changed curve (fan stages, 80-105 °C to 65-100 °C).
- `icarus-obd`: USB serial, Bluetooth RFCOMM (OBDLink MX+), TCP and a simulated vehicle; the ELM327 language with headers and
  ISO-TP reassembly; SAE J1979 services 01 (48 live values, each explained), 03/07/0A (codes), 09 (VIN, calibration IDs,
  CVNs, module names), readiness monitors and fuel system status.
- `icarus`: `def`, `show`, `diff`, `obd info|codes|pids|live` (CSV logging).

## Dealer-tool features (FORScan, Techstream)

| Feature | Icarus |
| --- | --- |
| Find every module, read its identity and trouble codes (UDS and KWP2000) | done (`obd scan`, `obd module`), read-only, tested on the simulated vehicle |
| Each module's own live data (Toyota: KWP2000 `21` local identifiers and UDS `22`; e.g. transmission fluid temperature) | build 1 done 2026-11-14, tested on the simulated vehicle: the definition format (`icarus-core` `moddata`: TOML sets, one formula for every source, three bit orders), the OBDb importer (`icarus import obdb`, the user's downloaded file, attribution shown), reading (`icarus obd data`: 21/22 only, gates at load and at send, paced, per-vehicle state), ISO-TP checks (lost, repeated or missing frames are errors). |
| Many makes, not one vehicle: a TOML profile per make (module addresses, quirks), picked by the VIN; user files add or correct | done: 12 makes + generic; addresses from public sources, each `checked = false` until seen on a vehicle |
| Code descriptions, where to start, parts usually involved, look-up and parts links, freeze frame, saved reports | done 2026-11-07 (standard codes in Icarus's words; maker codes per profile `[codes]`) |
| Clear codes | done 2026-11-07: asks first, engine off, a report saved before; OBD-II 04, UDS/KWP 14 per module; logged |
| Actuator tests (run the fan, open the throttle, cycle the ABS pump, click a relay) | done 2026-11-07: the engine (actuator.rs: allow-list, airbag ban, conditions, 30 s cap, hold-to-run, control returned on every path, log), `[[test]]` profile entries, five on the simulated vehicle. |
| Learn module data from another tool: a phone's Bluetooth log of an app and its adapter (pcap, PacketLogger), or a bus log (`icarus bus watch --raw`, candump) | done 2026-11-19, tested on small files written for the tests (`icarus import btlog`/`buslog`, docs/CAPTURES.md): a timed transcript and a set of every identifier a module answered on a data read (21, 22; identity reads 1A listed, not imported), unnamed and unverified; only the OBD conversation kept, VINs masked (in requests and answers) |
| Learn a test from another tool: watch the bus (adapter monitor mode, a Y-cable) while a shop tool runs it, decode its 2F/30 requests into a `[[test]]` entry | done 2026-11-07 (Bus page, icarus bus watch); one button adds it to the vehicle's file |
| Learning from another tool, as a step of its own | done 2026-11-08: the Learning page and `icarus learn`: marks name what is learned next, settings and tests are named and added there, what is not learned (security access, routines, refused writes) is noted, each session saved in the vehicle's Logs; Sarina (optional) reads a session and suggests what each thing is (`labyrinth-ask-sarina`, read-only; `learning_sessions` in `icarus mcp`) |
| Settings in the window | done 2026-11-08: the Settings page (value now, change with a yes, undo) |
| Customizations: a setting a module keeps (Toyota's customizations, FORScan's feature switches) | done 2026-11-08, tested on the simulated vehicle: settings.rs (`22`/`2E`, `21`/`3B` only; value before kept in the vehicle's Logs first, read back, put back if not taken, undo; wired, 12 V, parked, engine off; never airbag, immobiliser, key or odometer), `[[setting]]` profile entries, learned from the bus like tests (`icarus obd settings / setting / setting-undo`). |

The smaller writes (clearing codes, actuator tests, settings) each have their own module and allow-list, keep what was
there first and have a way back; flashing and coding open only when the conditions below are built.

## Editing (files only)

Editing in a table (Maps page: click a cell, type or step; undo a map or all) and on the command line
(`icarus set`), saved as new files only (`icarus-core` edit.rs), checksums corrected from a per-definition description
that must match the stock image first (`checksum.rs`: sums, XOR, CRC-16/CCITT, CRC-32; `icarus checksums`).

## Never brick a module (the rules every write follows)

A module is bricked when a write stops halfway or puts the wrong thing in. Icarus's writes follow these, in code:

1. **A verified backup first.** The whole module is read twice and the two reads must match before anything is written;
   the image goes into the vehicle's OEM tunes folder, read-only, never replaced (done: the folder rule).
2. **Only an image made for this module.** Its identity (part number, hardware and software numbers, calibration ID) must
   match what the module reports; a file for another module is refused.
3. **Checksums right.** The image's checksums are corrected and checked for its ECU family before writing.
4. **Steady power.** A battery charger or maintainer on (the battery above 12.5 V and holding steady, sampled during the
   write too); the computer kept awake (done: power_guard blocks sleep, lid switch and power keys). The computer's own
   battery working and charged (above 50 %), not mains only: a laptop with a dead battery switches off the moment its
   cable is pulled or the supply dips, halfway through the write.
5. **A wired cable.** Never over Bluetooth or Wi-Fi: a dropped wireless link mid-write is a classic brick.
6. **Calibration only, when the family allows it.** Write the calibration area and leave the boot loader alone, so the
   module can always be reached and written again.
7. **Read back and compare** after writing; on any doubt, write the backup back.
8. **Every first on the bench.** A family's first read and write happen on a spare ECU on the bench, never in the vehicle.
9. **Logged.** Every write in the action log.

## Writing a module (flash): the conditions

1. A spare ECU of the same part number on the bench (bench harness, bench power supply) for every first.
2. Before any write: a full backup read back and verified, checksums valid, battery voltage held up by a charger, the image
   matched to the ECU's identity.
3. After: read back and compare; a tested way back to the stock image.
4. Wired interface only: Bluetooth (the MX+) is for reading and logging, never flashing (a dropped link mid-write can brick
   the module).

## Known limitations

- Everything is tested against the simulated vehicle and adapter (`sim`); nothing has been tried on a real vehicle yet, and
  each make's module addresses are marked "not yet checked".
- No maker's actuator test or setting IDs are known yet: they are learned on a vehicle by watching another tool.
- Icarus does not read or write an ECU's memory, and knows no ECU family's checksum description yet (edited images are
  saved as files only).
- Not supported yet: Toyota body modules behind the gateway and BMW's (extended addressing), 29-bit module IDs (Honda) and
  GM's body range, Ford's medium-speed CAN, Mode 06 on older protocols, module data in the window and through `icarus mcp`,
  DDT2000 and DBC files, diagnostics through SocketCAN adapters (they watch the bus only), A2L definitions, service functions (relearns, resets, TPMS, injector coding).

## Interfaces

What each tuner and interface is on the wire, and how Icarus reaches it from Linux: [INTERFACES.md](INTERFACES.md).
