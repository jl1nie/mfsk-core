# ft8-compare

Not part of the workspace (root `Cargo.toml` `exclude`). Runs `mfsk-core` and the crates.io crate `rust-ft8`
(pinned `=0.2.1`) on the same 12 kHz / 16-bit WAV files, and compares both with WSJT-X's own `jt9`, stage by stage.
The measurement this was made for, and what it found: [`docs/notes/FT8_REAL_VS_JT9.md`](../../docs/notes/FT8_REAL_VS_JT9.md).

```sh
cargo build --release                                      # here; FT8_COMPARE_ONLY=mfsk skips rust-ft8
target/release/ft8-compare out.csv a.wav b.wav ...         # one row per decode: file,decoder,message,freq,dt,snr,ms
python3 compare.py out.csv <dir with jt9_<name>.out> <dir with reference <name>.txt>     # recall / extras against jt9 and a reference text
python3 score_truth.py truth.csv out.csv <jt9 dir> ft8_dense_                           # against a truth.csv (scripts/gen_ft8_*_wavs.py)
```

`jt9` output: `jt9 -8 -d 3 -L 200 -H 3000 file.wav` from `scripts/build_jt9_upstream.sh` (v3.2.0-rc1). The *trace* build
(`JT9_TRACE=1 scripts/build_jt9_upstream.sh ../WSJT-X v3.2.0-rc1`) writes `ft8_trace.dat` (stages, candidates, LLRs, `cs`, `sync2d`,
the residual after each search pass) and decodes identically; never time with it. Our own side of those traces was a local patch that
is not in the tree (write-only, env-gated); the scripts that read both sides (`stage_trace.py`, `extras_trace.py`, `llr_compare.py`,
`llr_stats.py`, `llr_bitpos.py`, `cs_compare.py`, `cand_compare.py`, `resid_compare.py`, `phantom_stats.py`) expect it.
`src/bin/osd_probe.rs` feeds jt9's LLRs to this crate's `bp_llr_zsum` + `osd174_91`. Scripts that need numpy are marked in their header.
