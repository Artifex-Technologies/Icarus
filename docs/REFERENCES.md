# References

Projects I shared or found are references for Icarus: they show what a vehicle tool can do and how a device or format works,
so the gaps in Icarus can be filled. Nothing is copied from them, not code and not data: Icarus's code is written for Icarus,
from the standards and from what each reference teaches. This page credits what was learned from where.

| Icarus | Learned from | What |
| --- | --- | --- |
| The Tactrix OpenPort 2.0 driver (`crates/icarus-obd/src/openport.rs`) | NikolaKozina's and Dale Schultz's j2534 library for Linux and macOS, https://github.com/NikolaKozina/j2534 (BSD 3-Clause) | the cable's command words, their answers and the layout of the packets it sends |
| The Drewtech Mongoose Pro driver (`crates/icarus-obd/src/mongoose.rs`) | roffe/gocan's native Mongoose Pro GM II driver, https://github.com/roffe/gocan (MIT) | the wire protocol: the packet layout and its check word, the commands of the open, connect, filter and send, the status codes |
| The bench wiring drawings for the 2016-2017 Tacoma 3.5 L automatic ECM (`Schematics/Toyota Tacoma 2016-2023`) | TacomaWorld members' excerpts of Toyota's factory wiring diagrams and owners' posts there (a 2017 Tacoma's CAN pins, two bench revivals), https://www.tacomaworld.com (each post is linked in that folder's README); PCMflash's manual, v1.08, https://pcmflash.ru/downloads/pcmflash_46.pdf?108; Toyota's technical service bulletins on NHTSA (T-SB-0077-16, T-SB-0062-18, T-SB-0043-20, T-SB-0092-20, T-SB-0134-16), https://static.nhtsa.gov/odi/tsbs/; the J1962 socket from Wikipedia's On-board diagnostics article and obdtester.com | which ECM pins carry +B, +B2, IGSW and the grounds, and their wire colors; the connector names, pin counts and housings; the minimal bench layout (PCMflash's drawing, whose power and ground pins match the factory diagram); the likely CAN pair (PCMflash and a 2017 Tacoma owner's TacomaWorld post) and the likely BOOT pin (PCMflash only), which no factory excerpt shows, so both stay to be checked in TIS; which part numbers make up the 2016-2017 automatic family, and the voltage window for flashing |
