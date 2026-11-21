# OBDex's trouble codes

`codes.jsonl.zz` holds the 9,533 generic SAE J2012 codes of [OBDex](https://github.com/foerbsnavi/OBDex) (data under CC0
1.0, a public-domain dedication), English only, at most three sources per code, one code per line in JSON, sorted, zlib
compressed. `crates/icarus-obd/src/obdex.rs` reads it.

Made from OBDex's published `generic.min.json` at commit `9ba1ffa` (2026-11-07):

```sh
curl -LO https://foerbsnavi.github.io/OBDex/generic.min.json
cargo run -p icarus-obd --example obdex_import -- generic.min.json
```

To update: the same two commands, then the commit and date above.
