# Icarus

A vehicle tuning, learning and diagnostics tool for Linux: engine, transmission and other control modules (ECU, ECM, PCM,
TCM). I'm building it to learn vehicle electronics hands-on while I build two off-road vehicles, and because no open tool on
Linux tunes factory ECUs.

Part of the Artifex ecosystem (Labyrinth OS, Yggdrasil, Heimdall, Sarina, Daedalus...). This repository is private.

## Install (on my machines)

```sh
bash scripts/install-local.sh        # builds, puts icarus and icarus-app in ~/.local/bin, Icarus in the app menu
```

## Trouble codes

Every code comes explained: what it means (standard codes in Icarus's own words, the maker's own codes from the vehicle
profile's `[codes]` table), what the letter and digits say, where to start for the common ones, the parts usually involved,
and links: a web search for the code on this vehicle, and searches for each part plus RockAuto's catalog for the vehicle.
Name a vehicle once (`icarus garage VIN "2017 Toyota Tacoma 3.5L V6"`, or the field on the Vehicle page) so the links name
the model. Searches go to DuckDuckGo unless `ICARUS_SEARCH` is a URL with `{q}`.

```sh
icarus dtc P0304 C1241 --vehicle "2017 Toyota Tacoma 3.5L V6"   # no vehicle needed
icarus obd codes --adapter A                                     # the vehicle's codes, explained, and the freeze frame
icarus obd report --adapter A --scan                             # everything in ~/Documents/Icarus/reports/*.md
```

The freeze frame (service 02) is what the engine was doing when the code set: speed, load, temperatures, fuel trims.

## The window

`icarus-app` (Icarus in the app menu): the **Vehicle** page connects to the adapter and shows the VIN, the make's profile,
the codes explained with Look up and parts buttons, the freeze frame, the check-engine light and readiness monitors, the
vehicle's name, and Save report; **Modules** scans every module for its identity and codes;
**Live data** picks values and draws them as they change, each with what it means; **Maps** opens a definition and an image
(and a second image to compare: it starts on the first map the second one changed, changed cells outlined). Start options:
`--connect [--adapter A] --page modules --scan`, `--live`, `--maps DEF.xdf IMAGE.bin [SECOND.bin]`.

## What it does today

Everything below only reads: nothing is sent to a module that changes it.

**Files** (no vehicle needed):

```sh
icarus def FILE.xdf [--find TEXT]                 # what a TunerPro definition describes; --find searches map names
icarus show IMAGE.bin FILE.xdf NAME               # one map, curve or value from an image, in real units
icarus diff ORIGINAL.bin TUNED.bin --def FILE.xdf # what a tune changed, map by map and cell by cell
```

`diff` is the quickest way to learn from an existing tune: give it the stock image and a modified one, and it names every
map that changed and shows each cell's old and new value.

**Vehicle**, through an OBD-II adapter:

```sh
icarus obd info  --adapter A    # adapter, protocol, VIN, modules, calibration IDs, check-engine light, readiness monitors
icarus obd codes --adapter A    # stored, pending and permanent trouble codes per module
icarus obd pids  --adapter A    # the live values each module offers, and what each one means
icarus obd live  --adapter A 0C 06 07 14 44 --csv run.csv   # watch values (PIDs in hex), optionally into a CSV log
icarus obd scan  --adapter A --make toyota   # every module (ABS, airbags, cluster...): identity and trouble codes
icarus obd module 7B0 --adapter A            # one module, by its request ID
```

`scan` and `module` work like a dealer tool (Techstream, FORScan): they talk to each module directly in UDS (ISO 14229) or
the older KWP2000 that many Toyota modules still use, not only to the engine's standard OBD-II side. A guard in the code lets
only reading services out: clearing codes, writing, actuator tests and programming are refused until the safety steps exist.

**Vehicles.** Icarus is for any make: what it knows about each one (where its modules answer, its quirks) is a small TOML
file in `crates/icarus-obd/vehicles/`, picked by the VIN. Built in: Toyota/Lexus, Subaru, Ford/Lincoln, GM, Stellantis,
Honda/Acura, Nissan, Hyundai/Kia, Mazda, VW/Audi, BMW/Mini, Mitsubishi, and a generic OBD-II profile. A file of the same name
in `~/.config/icarus/vehicles/` replaces a built-in one, and a new name adds a make, so a module found on a vehicle can be
written down without rebuilding. Addresses from public sources are marked "not yet checked" until seen on a real vehicle.

```sh
icarus vin VIN       # where it was built, maker, model year, check digit
icarus vehicles      # the makes Icarus knows
```

The adapter `A` is one of:

| | |
| --- | --- |
| `sim` | a simulated vehicle (engine + transmission, a P0171 lean code to diagnose): try everything without a car |
| `bt:AA:BB:CC:DD:EE:FF` | a paired Bluetooth adapter such as the OBDLink MX+ (RFCOMM channel 1; `#2` picks another) |
| `/dev/ttyUSB0[@115200]` | a USB adapter (38400 baud unless given) |
| `tcp:192.168.0.10:35000` | a Wi-Fi adapter |

`ICARUS_ADAPTER` sets a default. Pair a Bluetooth adapter once with `bluetoothctl` (`scan on`, `pair`, `trust`); Icarus
connects to it directly, no `rfcomm bind` needed.

## Build

```sh
cargo test --workspace
cargo build --release      # target/release/icarus
```

Rust (stable), Linux. Dependencies: `roxmltree` (XML) and `libc`.

## Layout

| Crate | What |
| --- | --- |
| `crates/icarus-core` | ROM images (original + edited, diff), definitions (tables, values, switches), formulas, the TunerPro `.xdf` reader |
| `crates/icarus-obd` | adapters (USB, Bluetooth, Wi-Fi, simulated), the ELM327/STN language, the standard OBD-II services |
| `crates/icarus` | the `icarus` command |
| `crates/icarus-app` | the window (egui, Yggdrasil's look from yggdrasil-ui); a worker thread owns the adapter |

## Where it is going

See [docs/PLAN.md](docs/PLAN.md): reading a factory ECU's memory, editing maps with checks, flashing with a verified backup,
transmission modules, a CAN bus view, and a desktop app.

## What Icarus will not do

- Ship one-click emissions "delete" features. Thresholds and calibration values (what sets what off) are ordinary maps, and
  emissions readings (oxygen sensors, lambda, fuel trims, catalyst temperature, monitors) are part of the live data.
- Bypass immobilizers or change odometer readings.
- Write to a module without a verified full backup first.

## Licence

MPL-2.0, like the rest of the ecosystem. The reference projects I looked at are GPL or proprietary: Icarus takes ideas and
file formats from them, never code.
