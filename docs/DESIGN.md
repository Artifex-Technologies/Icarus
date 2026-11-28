# Icarus: what it is built to do, and the rules it follows

Factory ECUs first, open ECUs second (for open ECUs, LibreTune is the tool, not something rebuilt here). Every step reads
before it writes, and nothing writes to a module until reading, checking and restoring are proven on a bench.

## Scope: an all-in-one vehicle tool

One tool for everything a vehicle's electronics need, in place of a code reader, a dealer tool (Techstream, FORScan), a
bidirectional shop scanner, a tuning suite and a bus analyser: on Linux, for many makes, without per-vehicle licences.

| Area | What Icarus has |
| --- | --- |
| Codes | every module (UDS, KWP2000), meanings, where to start, parts and look-up links, freeze frame, readiness, reports, clearing (report first), Mode 06 test results (CAN) |
| Live data | 92 standard values with graphs; recording a drive from the window or with no window (`icarus record`, and `--auto` with its user service `icarus-record`: starts when the engine runs, ends once it has been off a while; ICA-208), one recorder for both (`icarus_obd::drive`): every value the vehicle offers (the fast ones every sweep, the rest in turn) and each module's own values from the vehicle's module-data set through module data's gates and pacer, their raw answers kept so `icarus record redecode` reads old drives with a better set (ICA-44), the watched ones (knock feedback, gear, lock-up, shift lever and sport mode by default, matched as whole words, one request a module and name; `--watch NAME` or the window's Settings) every sweep, two requests a sweep within a 150 ms budget, never waiting on a busy module (one that answers busy is left alone a while), the rest in turn (one every other row while watching), the rows a second said after 10 s (ICA-218); a part file per ignition cycle; laptop kept awake; the adapter held alone; goes on through adapter glitches, opening the adapter again (ICA-112); a tone and a notice when it starts, stalls, resumes and stops (ICA-209); the Logs page (fuel trims by driving condition, charts on one time line, following a recording as it is written; Compare, two drives or more side by side at matching conditions, as `icarus logs compare`); `drive_logs` for assistants |
| Bidirectional | actuator tests, fenced (hold-to-run, conditions, airbag ban, key modules fenced by address) |
| Configuration | tires/gears calculator, speedometer check, battery and charging check |
| Tuning | map viewer, tune diff (XDF) |
| Vehicle info | VIN decoder, make profiles, named vehicles, vehicle folders, NHTSA model/engine (asks first) and recalls |
| Bus | monitor (OBDLink monitor mode, Linux SocketCAN): ID table with changing bytes, diagnostics in words, tests learned from another tool; a USB adapter's serial link sped up for the watch and put back after (STBR/ATBRD, ICA-213; tested on the simulated adapter only) |
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
| Find every module, read its identity and trouble codes (UDS and KWP2000) | done (`obd scan`, `obd module`), read-only, tested on the simulated vehicle; since 2026-11-27 the sweep finds a module wherever it answers from (not only at its request ID + 8; through an ELM327 or OBDLink at least what asking at the guess alone found), never sends on an ID it heard sending an answer of any kind, and `obd asbuilt` sweeps no chassis or driver-assist module (by its kind, its name or abbreviation, or the vehicle profile's name for its address) beyond what its module-data file lists; since 2026-11-28 (ICA-207) a UDS code whose status has none of the failure bits (only "not tested") is counted, not listed: a module answering `19 02 FF` with every code it can set shows "no trouble codes (N it can set, none failed)" (one that supports no failure bit at all has every code shown) |
| Each module's own live data (Toyota: KWP2000 `21` local identifiers and UDS `22`; e.g. transmission fluid temperature) | build 1 done 2026-11-14, tested on the simulated vehicle: the definition format (`icarus-core` `moddata`: TOML sets, one formula for every source, three bit orders), the OBDb importer (`icarus import obdb`, the user's downloaded file, attribution shown), reading (`icarus obd data`: 21/22 only, gates at load and at send, paced, per-vehicle state), ISO-TP checks (lost, repeated or missing frames are errors). Build 2 done 2026-11-21: the window's Module data card and `icarus mcp`'s `module_data` (`icarus_obd::vehicledata`: the set chosen as `obd data` chooses it, each module read once, the state kept). |
| Many makes, not one vehicle: a TOML profile per make (module addresses, quirks), picked by the VIN; user files add or correct | done: 12 makes + generic; addresses from public sources, each `checked = false` until seen on a vehicle |
| Code descriptions, where to start, parts usually involved, look-up and parts links, freeze frame, saved reports | done 2026-11-07 (standard codes in Icarus's words; maker codes per profile `[codes]`) |
| Clear codes | done 2026-11-07: asks first, engine off, a report saved before; OBD-II 04, UDS/KWP 14 per module; logged. Since 2026-11-28 (ICA-200): where the vehicle's profile names no airbag module (or its make is not known), a module of unknown kind (one the sweep found, or the profile does not name) is not cleared, from the window or `obd clear --module`: it could be the airbag module |
| Actuator tests (run the fan, open the throttle, cycle the ABS pump, click a relay) | done 2026-11-07: the engine (actuator.rs: allow-list, airbag ban, conditions, 30 s cap, hold-to-run, control returned on every path, log), `[[test]]` profile entries, five on the simulated vehicle. |
| Learn module data from another tool: a phone's Bluetooth log of an app and its adapter (pcap, PacketLogger), or a bus log (`icarus bus watch --raw`, candump) | done 2026-11-19, tested on small files written for the tests (`icarus import btlog`/`buslog`, docs/CAPTURES.md): a timed transcript and a set of every identifier a module answered on a data read (21, 22; identity reads 1A, 22 F190 and 22 F18C listed, not imported), unnamed and unverified; only the OBD conversation kept, VINs masked (in requests and answers), module serial numbers always |
| Learn a test from another tool: watch the bus (adapter monitor mode, a Y-cable) while a shop tool runs it, decode its 2F/30 requests into a `[[test]]` entry | done 2026-11-07 (Bus page, icarus bus watch); one button adds it to the vehicle's file |
| Learning from another tool, as a step of its own | done 2026-11-08: the Learning page and `icarus learn`: marks name what is learned next; a write the module took becomes a `[[setting]]`, an output test the module took (`2F .. 03`, `30 .. 07`, ICA-221) a `[[test]]` and a routine the tool starts and the module takes (`31 01` answered `71 01`) an unchecked `[[service]]` entry (engine off, parked); settings and tests are named and added to the vehicle's file there, a learned routine is added by hand from the session file; only the rest is noted, not learned (security access, a routine's stop and results, writes of the VIN or a serial number, writes too long for a setting, routine starts too long to learn, memory writes and programming, other kinds of output control, writes, output tests and routine starts the module refused or did not answer, ICA-219), each session saved in the vehicle's Logs; Sarina (optional) reads a session and suggests what each thing is (`labyrinth-ask-sarina`, read-only; `learning_sessions` in `icarus mcp`). Since 2026-11-27 (ICA-101): nothing is learned from an immobilizer, smart-key or certification module (fenced by the frame's ID; its messages are not even written into the session), and on any module a write, routine or output test sent after security access (`27`) is noted, never learned, until the module is back in its default session |
| Settings in the window | done 2026-11-08: the Settings page (value now, change with a yes, undo) |
| Customizations: a setting a module keeps (Toyota's customizations, FORScan's feature switches) | done 2026-11-08, tested on the simulated vehicle: settings.rs (`22`/`2E`, `21`/`3B` only; value before kept in the vehicle's Logs first, read back, put back if not taken, undo; wired, 12 V, parked, engine off; never airbag, immobilizer, key or odometer), `[[setting]]` profile entries, learned from the bus like tests (`icarus obd settings / setting / setting-undo`). Since 2026-11-27 (ICA-102): settings that share one identifier each have a `mask`; a change writes the value read just before with only its bits replaced, and the read-back, the put-back and undo touch only them, so one setting never undoes another; the learner gives a learned setting the bits the other tool changed (a first write that changed nothing takes the bits of the next one that does; one with no read before it stays a whole-value setting, since its bits are not known). Since 2026-11-28 (ICA-201): undo takes the setting's own last change, the log line with its name as well as its module, identifier and mask, so after a change of another setting on the same identifier (other bits, the same bits under another name, or both whole-value) it puts back only the one it names, and a setting never changed has none. |

The smaller writes (clearing codes, actuator tests, settings, service routines) each have their own module and allow-list,
keep what was there first and have a way back; flashing and coding open only when the conditions below are built.

**The airbag list** (ICA-108, `actuator::is_airbag_at`): airbag modules by request and answer ID, 11- and 29-bit CAN (Nissan's 752/772 and Honda's 18DA53F1/18DAF153 added, each with its public source in the code; Honda's also in its
profile, Nissan's not: one Leaf-only source must not lift the unknown-kind caution for every Nissan and Infiniti, my
decision of 2026-11-28, so Nissan keeps it), VW's 715 answering from 77F as its profile has it, and K-line and DoIP addresses (none confirmed yet), checked by actuator tests, clearing, settings, service routines, module data and the as-built sweep. A 29-bit or K-line module whose kind is not known gets identity only on any make, since the list knows few such addresses. GM's and Stellantis' airbag addresses stay out until a public source confirms them; until then their modules of unknown kind get identity only. The same caution holds for every write (ICA-200): where
the vehicle's profile names no airbag module on 11-bit CAN (or the make is not known), an actuator test, a setting, a
service routine or a clear aimed at a module the profile does not name is refused: a profile's entry is left out when the
profiles load, a learned one is not added to the user's file, and at the moment of the write (the make picked by the VIN
read then) nothing is sent to the module.

**The key modules' fence** (ICA-101, `crates/icarus-obd/src/fence.rs`). Immobilizer, smart-key and certification
modules are fenced by address, whatever a definition calls itself: a profile's `[[module]]` or a module-data file's
`[module]` gives the kind (`kind = "immobilizer"`, `"smart-key"`, `"certification"`; IDs are u32, so 29-bit modules fit).
Actuator tests, settings (reading them too), service routines and clearing codes refuse such a module before anything is
sent; an entry aimed at one is left out when the profiles load; module data and the as-built sweep give it identity only,
whichever file names it (a key module one set's file names gets no data request from another set's file on its address
either, and the sweep asks the whole fence, profiles and sets); the bus learner learns nothing from it and does not write
its messages into a session; `icarus mcp` does not ask it. On any module, a write the tool sent before a `27` unlock and
whose answer had not come is dropped at the unlock, so no answer after it teaches anything. No maker's addresses are built
in: my truck's go in from its own identity reads once I have confirmed them.

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
   cable is pulled or the supply dips, halfway through the write. A battery that cannot be measured counts as too low
   (done for actuator tests, clearing codes, settings and service routines: a SocketCAN interface or the Mongoose
   measures nothing, so they are refused through either). The reading is taken at the moment of the write: an adapter
   that read 12.6 V when it connected and gives no number now is refused, not judged by the old reading.
5. **A wired cable.** Never over Bluetooth or Wi-Fi: a dropped wireless link mid-write is a classic brick. Icarus tells a
   cable by where Linux puts the device, not by what it is called (done: `port::Target::wired`): a serial port or CAN
   interface counts only when sysfs puts it on a USB bus, and anything it cannot tell is not wired. So a symlink to an
   `rfcomm` port, a pty standing in for a Wi-Fi adapter, a built-in serial port, an slcan interface over Bluetooth and a
   vcan interface (cannelloni or `cangw` can carry its frames anywhere) do not count. A connected `rfcomm` port is moved
   by the kernel under the Bluetooth controller, which on most computers is on USB itself, so nothing under a Bluetooth
   controller counts (and an `rfcomm` name is refused as well); nor does a cable attached over USB/IP (`vhci_hcd`), whose
   bytes go over the network. A connection, in the window or on the command line (`obd service`, `obd setting`), is
   judged by the port it opened (`Link::wired`: a serial port by its device number, a CAN interface by its index), so a
   udev symlink moved to another adapter after connecting does not change the answer (ICA-206).
6. **Calibration only, when the family allows it.** Write the calibration area and leave the boot loader alone, so the
   module can always be reached and written again.
7. **Read back and compare** after writing; on any doubt, write the backup back.
8. **Every first on the bench.** A family's first read and write happen on a spare ECU on the bench, never in the vehicle.
9. **Logged.** Every write in the action log.
10. **The Read-Only switch.** Every write path asks `lock::writes_allowed` first: when the vehicle on the port (by the VIN
    read at that moment) has its Read-Only switch on (Garage page, `icarus garage read-only VIN on`), the write is refused;
    while any vehicle is switched, one whose VIN cannot be read is refused too (done: clearing, actuator tests, settings).
    Under them, `Elm::request` and `request_to` refuse any request `lock::changes_vehicle` names (everything but reads,
    tester present and hand-backs) to a switched vehicle, so a write path that forgets the check still cannot reach it.
    Any other link to a vehicle (a J2534 device, SocketCAN diagnostics) must do the same before it sends (done), and no
    link hands out what is under it (its socket, device or adapter commands) to send on past that check.
    Under the switch, a session floor that does not depend on the VIN (done): any command or the window started with
    `--read-only`, or with `ICARUS_READ_ONLY=1` in its environment (any value but empty, 0, no, false or off), refuses
    every write for the whole run, whatever VIN the vehicle gives and whatever `read-only.toml` says. A spare ECM on the
    bench gives its donor's VIN, so the switch alone does not cover it. `lock::writes_allowed` asks the floor first,
    before the VIN is read, so the write paths and the links (the ELM327, SocketCAN, the Mongoose, J2534 and so the
    OpenPort) refuse with "session read-only" and nothing sent; the command line refuses `obd clear`, `test`, `service`,
    `setting` and `setting-undo` before it opens the adapter. Nothing turns the floor off during a run. The window takes
    it from `--read-only` anywhere on its line before the line is read, so a line that does not parse cannot lose it; a
    window already open is handed it by a later start and answers that it holds it, and a start whose open window does
    not answer so (an older Icarus) ends with an error rather than leave a writable window behind. The window's worker
    asks the floor before each write too, so a refused clear sends not even its report's reads.
11. **One program per adapter.** Two programs on one adapter garble each other's requests and could cut a write or a
    routine in two. A serial port (an ELM327 or STN cable, the OpenPort, the Mongoose, SLCAN, a GPS) is held by the
    program that opened it alone (done: `port::open_serial` locks it with flock and marks it exclusive with TIOCEXCL), so
    a second program (the window and `icarus obd` on one cable at once) is refused with "another program has
    /dev/ttyUSB0 open" and that program's name, when this user may see it in /proc; so is a port another program had
    open before, locking nothing (a terminal program). Icarus takes off only an exclusive mark it made itself: a port
    another program marked (gpsd does, without a lock), which root's open gets past, keeps that program's mark.
    ModemManager, which runs as root and would probe a ttyACM cable with AT commands past both, is told by the udev rule
    to leave the OpenPort alone. Tested on a pseudo-terminal standing in for the cable; not yet tried on a cable in hand.
12. **A yes belongs to its connection.** The window's confirmations (a clear, a service routine, a setting change) are
    asked on one connection and close when it ends, another starts or the link fails; the command a yes sends carries
    that connection's number and VIN, and the worker refuses one given on an earlier connection, or once the connection
    has ended, before anything is sent (done).

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
- Not supported yet: BMW's gateway (its answers carry the tester's address, not the module's), clearing codes of modules
  behind a gateway (Toyota's give their identity, codes and module data, read only), 29-bit module IDs (Honda) and
  GM's body range, Ford's medium-speed CAN, Mode 06 on older protocols,
  DDT2000 modules on K-line, diagnostics through SocketCAN adapters (they watch the bus only), service functions (relearns, resets, TPMS, injector coding).

## Interfaces

What each tuner and interface is on the wire, and how Icarus reaches it from Linux: [INTERFACES.md](INTERFACES.md).
