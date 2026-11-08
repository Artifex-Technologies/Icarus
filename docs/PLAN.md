# Icarus: the plan

Started 2026-11-07. Factory ECUs first, open ECUs second (for those, LibreTune goes into Agora instead of being rebuilt here:
on 2026-11-07 it had only nightly builds, no stable release, AUR or Flathub package, so it waits for an open ECU to tune or a
stable release, whichever comes first).
Every step reads before it writes, and nothing writes to a module until reading, checking and restoring are proven on a bench.

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
| Freeze frames, readiness per module, code descriptions | next |
| Clear codes | after: a write (small, but it erases the evidence of a fault; asks first, shows what is cleared) |
| Actuator tests (run the fan, cycle the ABS pump, click a relay) | after: writes; each test with its conditions (engine off, vehicle still) |
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

The desktop app (Yggdrasil's look, egui) with map editing in a table, 2D and 3D, versions ("stock" plus named versions,
each a set of changes against the original), checksum correction per family, A2L definitions (ASAM MCD-2 MC) beside XDF.

## 4. Write (flash): only after all of this

1. A spare ECU of the same part number on the bench (bench harness, bench power supply) for every first.
2. Before any write: a full backup read back and verified, checksums valid, battery voltage held up by a charger, the image
   matched to the ECU's identity.
3. After: read back and compare; a tested way back to the stock image.
4. Wired interface only: Bluetooth (the MX+) is for reading and logging, never flashing (a dropped link mid-write can brick
   the module).

## 5. Transmission and other modules

TCM shift tables, line pressure and converter lock-up through the same steps (identify, read, edit, write on the bench first).

## 6. Learning and the rest of the ecosystem

- Sarina explains a log or a code in conversation ("why is long-term trim +12 % at idle?").
- Off-road extras: gauges for the trail (temperatures, voltages), logging to a file on the vehicle's computer.

## Hardware worth having

| What | Why |
| --- | --- |
| OBDLink MX+ (have) | reading, logging, CAN monitoring |
| SCT X4 (have) | holds the stock file of any vehicle it tuned |
| A wired interface (Tactrix OpenPort 2.0 or OBDLink EX) | reading and writing ECUs; which one depends on the vehicles |
| A battery charger / bench supply | stable voltage while reading or writing |
| A spare ECU + bench harness per ECU family | every first read and write happens here, not in the vehicle |
