# Vendored golden recordings (tier B)

Real off-air / reference recordings from the WSJT-X source tree
(`WSJT-X/samples/`), committed here so that **CI can actually run the
golden tests**.

Before this directory existed, every WSJT-X-sample test resolved its
audio from a sibling `../../WSJT-X/` checkout — present only on a
developer machine. CI has no such checkout, so `ft4_wsjtx_samples`,
`fst4_wsjtx_samples`, `q65_wsjtx_samples`, `wspr_wsjtx_samples` and
`msk144_wsjtx_samples` all hit their `eprintln!("skipping…"); return;`
path and **reported success without asserting anything**. Measured
2026-08-11: the only protocols CI genuinely covered were FT8 and JT9,
the two whose recordings happened to already be committed under
`embedded-poc/assets/`.

`mfsk-core/tests/common/corpus.rs` resolves these, and panics on a
missing asset when `MFSK_REQUIRE_CORPUS=1` — which `ci.yml` sets — so
the failure mode cannot come back silently.

| path | upstream | protocol |
|---|---|---|
| `ft4/000000_000002.wav` | `samples/FT4/` | FT4 |
| `fst4/210115_0058.wav` | `samples/FST4+FST4W/` | FST4-60 |
| `wspr/150426_0918.wav` | `samples/WSPR/` | WSPR |
| `msk144/181211_1205{00,08}.wav` | `samples/MSK144/` | MSK144 |
| `jt65/000000_0004.wav` | `samples/JT65/JT65B/` | JT65 |
| `q65/60A_EME_6m/` | `samples/Q65/` | Q65-60A |
| `q65/60D_EME_10GHz/` | `samples/Q65/` | Q65-60D |

FT8 and JT9 are not here — their recordings (`qso*.wav`,
`130418_1742.wav`, `191111_*.wav`) already sit in the parent directory.

Q65 is only partly vendored: the full sample tree is ~25 MB, so the
two EME sub-modes with hard assertions are committed and the rest
still resolve from an upstream checkout when one is present.

## Licence

WSJT-X is GPL-3.0-or-later, as is this crate, so redistributing these
recordings here is compatible. Upstream: https://sourceforge.net/projects/wsjt/
