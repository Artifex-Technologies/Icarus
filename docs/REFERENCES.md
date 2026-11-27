# References

Projects I shared or found are references for Icarus: they show what a vehicle tool can do and how a device or format works,
so the gaps in Icarus can be filled. Nothing is copied from them, not code and not data: Icarus's code is written for Icarus,
from the standards and from what each reference teaches. This page credits what was learned from where.

| Icarus | Learned from | What |
| --- | --- | --- |
| The Tactrix OpenPort 2.0 driver (`crates/icarus-obd/src/openport.rs`) | NikolaKozina's and Dale Schultz's j2534 library for Linux and macOS, https://github.com/NikolaKozina/j2534 (BSD 3-Clause) | the cable's command words, their answers and the layout of the packets it sends |
| The Drewtech Mongoose Pro driver (`crates/icarus-obd/src/mongoose.rs`) | roffe/gocan's native Mongoose Pro GM II driver, https://github.com/roffe/gocan (MIT) | the wire protocol: the packet layout and its check word, the commands of the open, connect, filter and send, the status codes |
