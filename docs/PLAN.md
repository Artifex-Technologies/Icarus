# Icarus: the plan

Started 2026-11-07. Factory ECUs first, open ECUs second (for those, LibreTune goes into Agora instead of being rebuilt here:
on 2026-11-07 it had only nightly builds, no stable release, AUR or Flathub package, so it waits for an open ECU to tune or a
stable release, whichever comes first).
Every step reads before it writes, and nothing writes to a module until reading, checking and restoring are proven on a bench.

## Scope: an all-in-one vehicle tool

The aim (2026-11-07): one tool for everything a vehicle's electronics need, in place of a code reader, a dealer tool
(Techstream, FORScan), a bidirectional shop scanner (Mucar, Snap-on), a tuning suite and a bus analyser: on Linux, for many
makes, without per-vehicle licences.

| Area | Have | Next | Later |
| --- | --- | --- | --- |
| Codes | every module (UDS, KWP2000), meanings, where to start, parts and look-up links, freeze frame, readiness, reports, clearing (report first), Mode 06 test results (CAN) | Mode 06 on older protocols | code history per vehicle |
| Live data | 48 standard values with graphs, CSV logs | each make's own values (Toyota transmission temperature...); a log viewer | trail gauges for the off-road builds |
| Bidirectional | actuator tests, fenced (hold-to-run, conditions, airbag ban) | learn tests by watching a shop tool on the bus | each make's test lists |
| Service functions | | | relearns (throttle, idle, steering angle), resets (oil life, battery), TPMS IDs, injector coding: per make |
| Configuration | tires/gears calculator, speedometer check, battery and charging check | | FORScan/Techstream-style customizations (tire size, axle ratio, features) and battery registration (BMW, VW/Audi, Ford, Ram...), with a backup and a way back; on a Tacoma, tire size and axle ratio live in the ECM calibration (a tuning edit) |
| Tuning | map viewer, tune diff (XDF) | map editing with checksums | read and flash ECUs (STI first, bench ECU), A2L; open ECUs through LibreTune |
| Vehicle info | VIN decoder, make profiles, named vehicles, vehicle folders, NHTSA model/engine (asks first) and recalls | an offline copy of NHTSA's vPIC data | |
| Bus | monitor (OBDLink monitor mode, Linux SocketCAN): ID table with changing bytes, diagnostics in words, tests learned from another tool | naming IDs per make | DBC files |
| Hardware | ELM327/STN over USB, Bluetooth, Wi-Fi | SocketCAN (CANable and friends) | J2534 cables (Tactrix OpenPort 2.0) for flashing |
| Vehicles | 12 makes + generic, addresses from public sources | extended addressing (Toyota body, BMW), 29-bit IDs (Honda), Ford's MS-CAN | each make's addresses checked on real vehicles |

## 0. Foundation (done 2026-11-07)

- `icarus-core`: ROM images (original and edited kept side by side, byte diff), definitions (maps, curves, values, switches),
  formulas both ways (stored number to real unit and back, nearest storable value, definition limits), the TunerPro `.xdf`
  reader. Checked on a real definition: 3,683 of 3,703 elements of a Siemens MS43 XDF load in 0.3 s, and `diff` of a stock
  and a modified image named the one changed curve (fan stages, 80-105 °C to 65-100 °C).
- `icarus-obd`: USB serial, Bluetooth RFCOMM (OBDLink MX+), TCP and a simulated vehicle; the ELM327 language with headers and
  ISO-TP reassembly; SAE J1979 services 01 (48 live values, each explained), 03/07/0A (codes), 09 (VIN, calibration IDs,
  CVNs, module names), readiness monitors and fuel system status.
- `icarus`: `def`, `show`, `diff`, `obd info|codes|pids|live` (CSV logging).

Not yet tried on a real vehicle or adapter: everything in `icarus-obd` was tested against the simulated adapter only.

## 1. Know the vehicles (needs: a Linux laptop, the OBDLink MX+)

The vehicles: a 2017 Toyota Tacoma TRD Sport 4x4 (3.5 V6, Denso engine computer) to test on now, and a 2019 Subaru WRX STI
(EJ257, Denso) planned for rally. The STI's ECU family is the best documented for open tools (read and flashed over OBD with a
Tactrix OpenPort 2.0), so it is the likely first ECU Icarus reads in full; Toyota protects the Tacoma's flash more, so there
the first goals are everything readable: live data, codes, Toyota's own values (transmission fluid temperature, which the
Tacoma has no gauge for) and the CAN bus.

1. First real connection: `icarus obd info` on each vehicle; note protocol, modules, calibration IDs (they identify the
   ECU software, which decides what definitions and tools apply).
2. STN extras on the OBDLink: long frames (`STPX`), the vehicle's other CAN buses where the adapter reaches them (Ford's
   MS-CAN, for one).
3. UDS (ISO 14229) reads: part and software numbers (`22 F1xx`), trouble codes with their status (`19`).
4. A CAN bus view: watch raw frames (`STMA`), name the ones known for the vehicle, see what changes when a pedal or switch
   moves. The best way to learn how the modules talk.
5. Log viewer: graphs of the CSV logs, fuel trims against load and speed.
6. Axis units in `show` and `diff`, formulas that use other values (the 20 MS43 elements left out).

## 1b. Dealer-tool features (FORScan, Techstream)

What those tools do, and where Icarus is:

| Feature | Icarus |
| --- | --- |
| Find every module, read its identity and trouble codes (UDS and KWP2000) | done (`obd scan`, `obd module`), read-only, tested on the simulated vehicle |
| Each module's own live data (Toyota: KWP2000 `21` local identifiers and UDS `22`; e.g. transmission fluid temperature) | next: needs the identifiers per module, from public sources, checked on the Tacoma |
| Many makes, not one vehicle: a TOML profile per make (module addresses, quirks), picked by the VIN; user files add or correct | done: 12 makes + generic; addresses from public sources, each `checked = false` until seen on a vehicle |
| Toyota body modules behind the gateway (ID 750 with an extended address byte), BMW's gateway | next: extended addressing (`ATCEA`) |
| Honda's and others' 29-bit module IDs (18DAxxF1); GM's body range (24x answering at 64x) | next |
| Ford's medium-speed CAN (pins 3/11) | next: OBDLink's bus switching |
| Code descriptions, where to start, parts usually involved, look-up and parts links, freeze frame, saved reports | done 2026-11-07 (standard codes in Icarus's words; maker codes per profile `[codes]`) |
| Mode 06 (on-board test results), a VIN decoder from NHTSA's public vPIC data (model, engine) | next |
| Clear codes | done 2026-11-07: asks first, engine off, a report saved before; OBD-II 04, UDS/KWP 14 per module; logged |
| Actuator tests (run the fan, open the throttle, cycle the ABS pump, click a relay) | done 2026-11-07: the engine (actuator.rs: allow-list, airbag ban, conditions, 30 s cap, hold-to-run, control returned on every path, log), `[[test]]` profile entries, five on the simulated vehicle. No real IDs yet: each maker's own |
| Learn a test from another tool: watch the bus (adapter monitor mode, a Y-cable) while a shop tool runs it, decode its 2F/30 requests into a `[[test]]` entry | done 2026-11-07 (Bus page, icarus bus watch); one button adds it to the vehicle's file |
| Customizations / as-built configuration (FORScan's paddle shifters and tire size; Toyota's customizations) | after: writes; a backup of the whole configuration before any change, and the way back |
| Service functions (relearns, resets, bleeding, injector coding, TPMS IDs) | after: per vehicle |

Writes of any kind open only when docs/PLAN.md section 4's conditions are built: a backup first, a clear statement of what
changes, and the way back.

## 2. Read the ECU (needs: the ECU's family, a wired interface)

Reading the whole calibration depends on the ECU: its bootloader or diagnostic service for memory reads (`23`
ReadMemoryByAddress, or the maker's own), and its security access (`27`, seed and key). Each family is its own module in
Icarus, with its checksums. The output is a full image, saved twice, with a report of its identity and checksums.

**The SCT X4:** if a vehicle carries an SCT tune now, the X4 holds its stock file and is locked to that vehicle until it
returns it to stock. Return to stock with the X4 before any other tool reads or writes that ECU, and keep a copy of the
stock file it saved.

## 3. Edit (files only)

Done (2026-11-07): editing in a table (Maps page: click a cell, type or step; undo a map or all) and on the command line
(`icarus set`), saved as new files only (`icarus-core` edit.rs), checksums corrected from a per-definition description
that must match the stock image first (`checksum.rs`: sums, XOR, CRC-16/CCITT, CRC-32; `icarus checksums`).
Next: each ECU family's checksum description (from a source, checked on a real image of that family; Toyota/Denso first),
2D and 3D views, versions ("stock" plus named versions, each a set of changes against the original), A2L definitions
(ASAM MCD-2 MC) beside XDF.

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

## 4. Write (flash): only after all of this

1. A spare ECU of the same part number on the bench (bench harness, bench power supply) for every first.
2. Before any write: a full backup read back and verified, checksums valid, battery voltage held up by a charger, the image
   matched to the ECU's identity.
3. After: read back and compare; a tested way back to the stock image.
4. Wired interface only: Bluetooth (the MX+) is for reading and logging, never flashing (a dropped link mid-write can brick
   the module).

## 5. Transmission and other modules

TCM shift tables, line pressure and converter lock-up through the same steps (identify, read, edit, write on the bench first).

- **Tacoma S mode that holds the gear.** On the 2016+ Tacoma's 6-speed automatic, S mode limits the highest gear (S4: no
  higher than 4th) and the transmission still shifts below it on its own; holding the chosen gear is the TCM's shift logic,
  a calibration edit, not a setting (I want this; 2026-11-08). The 2016 Focus had S and L the same way.
- **Learned settings.** What a dealer tool changes on my own vehicle (a customization, a feature switched on), recorded by
  watching the bus while the tool does it (`icarus bus watch`, as actuator tests are learned), becomes a setting Icarus can
  change itself: the value there before read and kept, one-step undo, parked, wired. Waiting for the laptop at the truck.

## 6. Learning and the rest of the ecosystem

- Sarina explains a log or a code in conversation ("why is long-term trim +12 % at idle?"): `icarus mcp` (done 2026-11-07,
  read-only tools: codes explained, a vehicle check, every module's codes, tires and gears). Next: a log file as a tool
  (a CSV from `obd live --csv`), still read-only.
- Off-road extras: gauges for the trail (temperatures, voltages), logging to a file on the vehicle's computer.

## Hardware worth having

What each tuner and interface is on the wire, and how Icarus reaches it from Linux: [INTERFACES.md](INTERFACES.md).

| What | Why |
| --- | --- |
| OBDLink MX+ (have) | reading, logging, CAN monitoring |
| SCT X4 (have) | holds the stock file of any vehicle it tuned |
| A wired interface (Tactrix OpenPort 2.0 or OBDLink EX) | reading and writing ECUs; which one depends on the vehicles |
| A battery charger / bench supply | stable voltage while reading or writing |
| A spare ECU + bench harness per ECU family | every first read and write happens here, not in the vehicle |
