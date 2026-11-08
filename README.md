# Icarus

A vehicle tuning, learning and diagnostics tool for Linux: engine, transmission and other control modules (ECU, ECM, PCM,
TCM). I'm building it to learn vehicle electronics hands-on while I build two off-road vehicles, and because no open tool on
Linux tunes factory ECUs.

Part of the Artifex ecosystem (Labyrinth OS, Yggdrasil, Heimdall, Sarina, Daedalus...). This repository is private.

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
