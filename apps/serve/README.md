# mfsk-serve

A resident process that decodes and encodes through `mfsk-core`: JSON lines on stdin, JSON lines on stdout. One decoder per mode is kept between
requests, so the callsign hash table, the latest periods and a period's checkpoint state live as long as the process does.

```sh
cargo install --git https://github.com/jl1nie/mfsk-core --rev <commit> --profile dist mfsk-serve
echo '{"op":"decode","id":1,"mode":"FT8","wav":"slot.wav","depth":"normal"}' | mfsk-serve   # or "profile":"p1" | "p2": a hardware class's depth, AP and budget (extrapolated, not measured there)
mfsk-serve bench slot.wav        # time the decoder per depth, cold and warm, and at the checkpoints
```

The protocol, the checkpoints, the encode request, and how to read `bench` on a board are in the manual:
[`docs/reference/MANUAL_SERVE.md`](../../docs/reference/MANUAL_SERVE.md) (日本語: [`MANUAL_SERVE.ja.md`](../../docs/reference/MANUAL_SERVE.ja.md)).

A sample app: `publish = false`, no tag, no network, no support promise. Tests: `cargo test -p mfsk-serve --release`.
