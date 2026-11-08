# Icarus: the plan

Started 2026-11-07. Factory ECUs first, open ECUs second (for those, LibreTune goes into Agora instead of being rebuilt here).
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

## 1. Know the vehicles (needs: the vehicles named, the OBDLink MX+)

1. First real connection: `icarus obd info` on each vehicle; note protocol, modules, calibration IDs (they identify the
   ECU software, which decides what definitions and tools apply).
2. STN extras on the OBDLink: long frames (`STPX`), the vehicle's other CAN buses where the adapter reaches them (Ford's
   MS-CAN, for one).
3. UDS (ISO 14229) reads: part and software numbers (`22 F1xx`), trouble codes with their status (`19`).
4. A CAN bus view: watch raw frames (`STMA`), name the ones known for the vehicle, see what changes when a pedal or switch
   moves. The best way to learn how the modules talk.
5. Log viewer: graphs of the CSV logs, fuel trims against load and speed.
6. Axis units in `show` and `diff`, formulas that use other values (the 20 MS43 elements left out).

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
