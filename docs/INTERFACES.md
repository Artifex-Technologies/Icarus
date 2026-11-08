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
