# Tuners and interfaces that ship Windows-only software

What each device is on the wire, what its maker's software does with it, and how Icarus can reach it from Linux. Learned by
reading the makers' installers as data (never running them on the host) and by watching their programs inside a Limbo guest
(Limbo's USB passthrough, 2026-11-08). Clean-room: what is written here is behaviour seen from outside (USB IDs, file types,
exported function names, traffic), never the makers' code; their encrypted files stay encrypted.

## The way in

1. **Read the installer as data** in a quarantine folder: what kind of installer, who signed it, which drivers (their `.inf`
   files name the USB IDs and say which Windows USB stack is used), which libraries.
2. **Run the maker's program only inside Limbo**: a disposable Linux guest with Wine, the device passed through by its USB ID
   (listed in `/etc/limbo/host-policy.json` `usb_allow`, approved per session with `sudo limbo passthrough approve`).
3. **Record the conversation on the host** (`usbmon`, Wireshark): the guest's USB traffic passes through the host's USB stack.
4. **Write Icarus's own driver** from what was seen, read-only commands first, each command on an allow-list.

The udev rule `packaging/70-icarus-tuners.rules` lets the person at the computer open these devices (no root).

## Recognised now (`icarus devices`)

| Device | USB ID | On the wire | Linux | Source |
| --- | --- | --- | --- | --- |
| SCT X4 (7015) "MB103" | `5059:4040` | vendor bulk | raw USB | seen plugged in; SCT's installer |
| SCT / Derive newer devices | `5059:4080` | WinUSB | raw USB | Derive's driver `.inf` |
| COBB AccessPORT 2.0 / 2.0B | `1a84:0120` / `0121` | libusb-win32 or WinUSB | raw USB (COBB's macOS app uses libusb) | AccessPort Manager's driver `.inf` |
| COBB USB to OBD cable (FTDI) | `1a84:0210` | WinUSB | raw USB | same |
| COBB USB to OBD cable (Atmel) | `1a84:0211`, `0213` | WinUSB | raw USB | same |
| COBB USB to OBD cable (SS) | `1a84:0212` | FTDI chip, COBB's ID | raw USB | COBB's FTDI `.inf` |
| HP Tuners MPVI (channels A, B) | `0403:d018` | FTDI FT2232, HP Tuners' ID | raw USB (FTDI's D2XX on Windows; `ftdi_sio` does not know the ID) | HP Tuners' FTDI driver pack 2.08.28 |
| HP Tuners CDV | `0403:d019` | FTDI, HP Tuners' ID | raw USB | same |
| HP Tuners MPVI2, MPVI Pro, MPVI3, MPVI4 | not in the installers | named in VCM Suite / VCM Live | to read when one is plugged in | VCM Suite 2024, VCM Live beta |
| VF Tuner WiFlash | `10c4:ea60` (generic CP210x) | ESP32 behind CP210x serial; Wi-Fi 192.168.4.1 | serial (`cp210x`) | VF Tuner 4.5 |
| Tactrix OpenPort 2.0 | `0403:cc4c` | vendor bulk (J2534) | raw USB | as others publish it; not yet seen here |
| FTDI and CP210x cables (ELM327, OBDLink SX/EX) | `0403:6001`, `0403:6015`, `10c4:ea60` | serial | serial | standard IDs |

`icarus devices` lists what of this is plugged in and whether this user may open it (the udev rule in
`packaging/70-icarus-tuners.rules` gives the person at the seat each of these; `bash scripts/install-local.sh --udev`). It
sends nothing to any device. Speaking each device's own protocol is the next step, device by device.

## SCT X4 (7015) — Ford

| | |
| --- | --- |
| USB | `5059:4040` "SCT LLC SCT MB103": one vendor-specific interface, three bulk endpoints (0x81 in, 0x02 out, 0x83 in), no mass storage |
| Linux | no driver binds; raw USB (libusb/usbfs) only |
| Maker's software | SCT Device Updater: a WiX bundle; the app is a packed .NET program (needs .NET 4.8); drivers: Derive's WinUSB driver for `5059:4080` (newer devices) and an older InstallShield-packed driver for the X4 |
| State | passed through to a Limbo guest and seen there (2026-11-08); the updater under Wine not yet tried with it |

## VF Tuner (OVT) — Toyota and Lexus

| | |
| --- | --- |
| Software | VF Tuner 4.5 (VF Tuner LLC): unsigned Inno Setup 6.4 installer; a C++Builder program |
| Interface | `esp32.dll`, a **SAE J2534 PassThru** library (`PassThruOpen`, `PassThruConnect`, `PassThruReadMsgs`, `PassThruWriteMsgs`, `PassThruIoctl`, `PassThruStartMsgFilter`, `PassThruSetProgrammingVoltage`, ...): the "WiFlash" device is an ESP32 behind a Silicon Labs CP210x USB-serial chip (the installer ships the CP210x driver) |
| Linux | the CP210x needs no driver (`cp210x`, `/dev/ttyUSB*`); what `esp32.dll` says over that serial line is to be learned by watching it with the device in hand |
| Data | definitions, trouble codes and checksums are encrypted (`BinDefs.enc`, `DTCDefs.enc`, `Nsum.enc`): not opened. Bundled profiles: six Tundra calibrations (`89663-0CS..`). `Logger.ini` lists SAE J1979 PIDs by its own names |

Because VF Tuner talks J2534, a J2534 layer in Icarus (the API on top of each device's own transport) would also reach the
common J2534 cables (Tactrix OpenPort 2.0, Mongoose and the like).

## Silicon Labs CP210x driver (`cp210.exe`)

A third-party WinRAR self-extractor (a Russian SFX template) around Silicon Labs' signed CP210x VCP installers (old and
Windows 10 versions), started by an unsigned `start.exe` that only picks one by the Windows version (no network use seen).
Its `.inf` names `10C4:EA60`, `EA63`, `EA70`. Not needed on Linux at all.

## COBB AccessPORT

AccessPort Manager (Windows: an NSIS installer with `APDriverInstall.exe`; macOS: an app with a bundled libusb). The
Windows drivers are libusb-win32 (`AP_20.inf`) and WinUSB (`accessport.inf`, class "COBB Devices") for vendor `1a84`, and an
FTDI driver for the "SS" cable (`1a84:0212`). That the macOS app reaches the devices through libusb means raw USB is all they
need: the same route Linux has.

## HP Tuners

VCM Suite and VCM Live (WiX `.msi` installers; the drivers come separately): the MPVI and CDV are FTDI chips under HP Tuners'
product IDs (`0403:d018` with channels A and B, `0403:d019`), reached through FTDI's D2XX library. The programs also name the
MPVI2 ("MPVI2 PL A"), MPVI Pro, MPVI3 and MPVI4, whose IDs are not in these installers: `icarus devices` will show their IDs
when one is plugged in (as an unknown device of a known maker only if the maker's vendor ID is known).

## FORScan (Ford, Mazda, Lincoln)

FORScan 2.3.71 (an Inno Setup 5.5.7 installer; the program is packed): no hardware or drivers of its own. It works through
ELM327 adapters, the OBDLink family (it carries their firmware updates: SX/EX on STN2230/2231, MX+ on STN2255/2256) and the
vLinker FS, and through J2534 cables. Icarus already speaks the ELM327/STN language (the OBDLink MX+ included) and now has a
J2534 layer, so the adapters FORScan uses are Icarus's adapters too; a vLinker FS will show its USB ID in `icarus devices`.

## Techstream (Toyota)

Toyota supplies Techstream through a TIS subscription, for any SAE J2534 cable. A "TechStream 12.20.024-v2" `.ova` (a 2018
Windows XP virtual machine packaged by a third party, "Getco") is a repack of it, not Toyota's: it was not opened or run (an
unlicensed, unpatched Windows with modified software of unknown origin). Nothing in it is needed: Techstream reaches the
vehicle through a J2534 cable, which Icarus's J2534 layer is for, and Toyota's modules speak UDS and KWP2000, which Icarus has.

