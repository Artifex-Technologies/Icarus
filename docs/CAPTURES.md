# Learning module data from a capture

Another tool that already knows a vehicle (a phone app such as OBDLink's, a shop scan tool, a dealer tool) shows which
identifiers each module answers. Icarus reads a recording of that tool at work and turns it into two things:

1. a **transcript**: every request the tool sent a module and what came back, with the time, as CSV;
2. a **module-data set** (Icarus's own format, `~/.config/icarus/data/SET/`): every identifier a module answered on a data
   read (UDS `22`, KWP2000 `21`) becomes a request with one placeholder value covering its answer's bytes, unnamed
   (`22 1234 (8 bytes)`) and not verified, with the answers seen kept as samples (a `22` read of several identifiers at
   once counts as a read of each, K-line and J1850 modules too). Identity reads (KWP2000 `1A`) are
   listed as seen, not imported: a module's identity is read in a later build.

```sh
icarus import btlog obdlink.pklg --make toyota --transcript obdlink.csv   # a phone's Bluetooth log
icarus import buslog drive.log --make toyota --transcript drive.csv      # a bus log (icarus bus watch --raw)
icarus data obdlink --sample                                             # the identifiers and their sample answers
```

`btlog` and `buslog` read either kind of file (the format is told by its first bytes). `--name SET` names the set (else
it is named after the file), `--dry-run` writes no set (the transcript is still written when asked for), `--replace`
replaces a set of that name, `--transcript -` prints the CSV on stdout (and everything else on stderr, so the CSV can be
redirected on its own). Captures up to 256 MiB are read. Without `--make`, the make comes from a VIN in the capture
when there is one (and the set takes its model year); with neither, the set names no make, and the gates refuse every
module of unknown kind (it could be the airbag module).

## Capturing an iPhone's Bluetooth on Linux

1. On the iPhone, install Apple's Bluetooth logging profile (from Apple's developer "Profiles and Logs" page; it is
   installed under Settings, General, VPN & Device Management, and expires after a few days). Restart the iPhone if the
   profile says to.
2. Connect the iPhone to the Linux computer with its cable and trust the computer (`usbmuxd` running, libimobiledevice
   installed: `idevice_id -l` lists the phone).
3. Start the log, then use the app on the vehicle (connect, open the data lists you want, let them run), then stop it
   with Ctrl+C:

   ```sh
   idevicebtlogger -f pcap obdlink.pcap     # or: idevicebtlogger obdlink.pklg (Apple PacketLogger, its default)
   ```

4. `icarus import btlog obdlink.pcap --make toyota --transcript obdlink.csv`.

To match values to names later, record the phone's screen at the same time (the iPhone's own screen recording). The
transcript's `clock_utc` column is the phone's clock (UTC) for each request, `time_s` the seconds since the log started.

## Capturing on the bus (any tool, any make)

A Y-splitter on the OBD port carries both: the other tool on one branch, an adapter on the laptop on the other, listening
only. `icarus bus watch --adapter /dev/ttyUSB0 --raw --seconds 600 > drive.log` (an OBDLink EX or another STN adapter
in monitor mode), or `candump -l can0` on a CAN interface. Then `icarus import buslog drive.log`. A `candump -l` log
holds the computer's clock: the transcript's `clock_utc` is that clock (UTC) for each request, and `time_s` counts from
the log's first frame. `icarus bus watch --raw` holds no time of day: `time_s` counts from when watching began and
`clock_utc` stays empty.

## What is kept (privacy)

A phone's Bluetooth log holds everything the phone sent and received over Bluetooth while it ran: headphones, the car's
hands-free, a watch. Icarus keeps only the OBD conversation:

- Only HCI ACL data is read. HCI commands and events, where Bluetooth addresses, device names and link keys travel, are
  skipped unread; no Bluetooth address, device name or key is written to any output.
- A channel is kept only when it is an adapter's conversation (one side sends the adapter's `>` prompt, the other side's
  lines are AT, ST and hex commands). Every other channel (audio, hands-free, contacts, messages) is dropped as it is found,
  never written or quoted; the output says only how many were dropped.
- Adapter commands (AT, ST) are followed for what they change (headers, request IDs) and counted, never written: their
  answers hold the adapter's serial number and the battery voltage.
- A VIN (or text like one: 17 or more characters a VIN may hold) is masked with `*` in the transcript and the set, unless
  `--keep-vin` is given: in answers (`22 F190`, `1A 90`, `09 02`) and in requests (a VIN written with `2E F190` or
  `3B 90`), also where it is split over several messages (`09 02` on K-line and J1850 gives it four bytes at a time).
  The whole field of a VIN's own identifier is masked whatever it holds; more is masked than need be rather than less. The VIN is used, unwritten, to pick the make and
  model year when `--make` is not given.
- The capture file itself is never copied: the set records its file name and sha256 only.

A bus log holds only CAN frames; Icarus keeps the diagnostic ones (IDs 700-7FF, 29-bit 18DA/18DB, and where the log
shows them used for diagnostics BMW's 6F1 with its answers on 6xx and GM's 24x with their answers on 64x) and ignores
the rest.

The captures, transcripts and sets of a real vehicle stay outside this repository.

## What is imported, and what is only listed

| Seen | What happens |
| --- | --- |
| A data read (`22`, `21`) one module answered | a request in the set, one hex placeholder value over the shortest answer's bytes, up to 8 different answers as samples, marked unverified for every year |
| A `22` read of several identifiers at once (`22 1234 5678`) | one request per identifier, its answer (`62 1234 .. 5678 ..`) split at each identifier's echo: the split that fits every answer to that request and the lengths single reads of those identifiers gave (the one with the most identifiers in it); an answer that still splits more than one way is listed, not imported |
| A K-line or J1850 module (a header such as `81 10 F1`) | imported like a CAN one, the module known by its address (`10`, the header's target byte; `ids = "kline"`); `icarus obd data` does not reach K-line yet |
| The same, refused or unanswered every time | listed with the module's refusal code or the adapter's words (NO DATA) |
| An identity read (`1A`) | listed as "seen, not imported" (a module's identity is read in a later build) |
| An answer longer than one value can cover (over 8191 bytes) | listed, the rest of the capture imported |
| A write, test, routine, clearing, security access, session or reset (`2E`, `2F`, `31`, `3B`, `14`, `27`, `10`, `11`...) | listed as "seen, not imported" |
| Standard OBD-II (`01`, `09`...) and anything sent to every module (`7DF`, K-line's functional addresses `33` and `6A`) | listed as "seen, not imported" |
| An airbag or occupant module | identity only, as for every set (the vehicle gates of `icarus obd data` apply when the set is written and when it is used) |

Everything the gates of `icarus obd data` refuse (an answer ID used as a request ID, a second module on one request ID, a
module of unknown kind on a make without a named airbag module) is refused here too, with the reason. A set learned from a
capture is read-only data like any other: `icarus obd data` sends only its reads, paced. A module left with no request
after the gates gets no file in the set (it is listed with the reasons).

## Naming a value

1. Find the moment in the screen recording where the app shows the value, and the rows of the transcript at that time
   (`clock_utc`): the identifier whose answer bytes follow the value on the screen is the one.
2. Name it with `icarus data name`, one value per number on the screen: where it sits (`--byte`, from 0 after the
   identifier's echo; `--bit`, `--bits`, `--signed`) and how it scales (`--mul`, `--div`, `--add`, `--unit`). Without
   `--keep` the placeholder itself becomes the named value; with `--keep --id NEW_ID` it stays and the named value is
   added beside it (one answer, several values). A layout makes it a number unless `--kind text` or `--kind hex` says
   otherwise. The module file is checked as Icarus checks every module file and written back (nothing is written when
   it does not pass); the samples stay, and what each decodes to is shown:

   ```sh
   icarus data name obdlink 7E0 RAW_22_1234 --keep --id ECT --name "Coolant temperature" --byte 1 --bits 8 --add -40 --unit °C
   icarus data name obdlink 7E0 RAW_22_1234 --id MODE --name "Mode" --bits 8 --kind hex
   ```

   A value can also be written by hand in its module file in `~/.config/icarus/data/SET/` (for example `7E0.toml`:
   `name`, `byte`, `bits`, `mul`/`div`/`add`, `unit`), then checked with
   `icarus data check ~/.config/icarus/data/SET/7E0.toml --make toyota`.
3. `icarus data SET --sample` shows what each sample answer decodes to.
4. On the vehicle, `icarus obd data 7E0 --set SET --adapter A` reads it live; `icarus obd data --check 7E0 VALUE_ID` marks it
   as checked once it matched a gauge or the app.

## The formats Icarus reads

- **pcap**, link type 201 (Bluetooth HCI H4 with a 4-byte direction word: its lowest bit set when the phone received the
  packet) or 187 (plain H4; then the RFCOMM command/response bit tells the two sides apart). pcapng and Android's btsnoop
  are recognised and refused (`editcap -F pcap` converts pcapng).
- **Apple PacketLogger** (`.pklg`): records of a 4-byte length (of what follows it), 4-byte seconds, 4-byte microseconds,
  one type byte (02 ACL data the phone sent, 03 ACL data it received; commands, events, audio and notes are skipped) and
  the packet; big-endian, or little-endian told by the first record.
- Inside: HCI ACL fragments put back together into L2CAP packets per connection and direction (without directions, one
  packet open for each side, each fragment given to the one it fits by the side's own fragment size, what is left and
  the RFCOMM checksum, so the two sides' fragments may interleave); RFCOMM channels found from
  the L2CAP connection requests (PSM 3) and forgotten at their disconnection, each channel known by the ID its packets
  carry in each direction (the two sides choose their IDs apart, and reuse them: an SDP query and the RFCOMM channel after
  it often both get 0x40), or, for a channel opened before the log began, by RFCOMM frames whose checksums
  hold; RFCOMM data frames (UIH) with or without a credit byte; Apple's iAP2 link packets on an MFi accessory's channel
  (such as the OBDLink MX+ with an iPhone): both checksums checked, the external-accessory session's bytes taken out
  (sessions told by each packet's session byte; the control session, whose messages start `40 40`, left out whole, also
  a message of it split over several packets), a packet sent twice read once; Bluetooth LE adapters from ATT writes and
  notifications.
- **The adapter's language** (ELM327 and STN): commands end at a carriage return, answers at the `>` prompt; the settings
  that change requests and answers are followed once the adapter answered OK (`ATZ`/`ATD`/`ATWS`, `ATSP`/`ATTP` and
  `ATDPN`'s answer, `ATSH` with 3, 6 or 8 digits (6 under automatic protocol: 29-bit CAN when they start DA or DB,
  else K-line or J1850 header bytes such as `81 10 F1`) and `ATCP`, `ATCRA`/`ATAR`/`ATCF`+`ATCM` (3 or 8 digits; `ATCFC0/1`
  is flow control, not the filter), `ATCEA`, `ATH`, `ATCAF`,
  `ATS`, `ATD0/1`) and STN's one-shot `STPX H:..., D:..., R:...`; answers with and without headers, long answers as CAN
  frames or as the adapter's `0:`/`1:` lines, CAN formatting off with the app's own flow control, response pending
  (`7F xx 78`) skipped; an empty command repeats the last one.
- **Bus logs**: `icarus bus watch --raw` lines (`time  ID  bytes`, the watch's own words skipped) and `candump` (both its
  log format `(time) can0 ID#DATA` and its screen format). ISO-TP per sender, extended addressing told per ID by which
  way its frames make more whole messages (an ID that reads as well both ways, as sub-addresses 10-3F can, follows the ID
  it talks to), each request paired with its answer by service and echo (from the usual answer ID first, and behind a
  gateway with the request's address byte first: two modules behind 750 are told apart), within five seconds and among
  the last 64 requests still waiting (a log without times waits by count).

## Known limitations

- Not yet tried on a real phone's log or a real bus log: the formats are read as their public descriptions give them, and
  tested on small files written for the tests.
- Bus logs: BMW's and GM's modules (requests on 6F1 and 24x) are read into the transcript, but the gates of
  `icarus obd data` take request IDs 700-7F7 only, so they are listed as refused, not kept in the set.
- Modules on K-line, J1850 and 29-bit CAN are imported, but `icarus obd data` does not reach them yet.
