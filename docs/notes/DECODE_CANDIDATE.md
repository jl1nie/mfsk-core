# Decoding one known candidate: what each option is pinned by

`decode_candidate` (WSPR, JT65, JT9, Q65) replaced the public `SniperRequest` (deprecated since 0.17.0; see
`docs/reference/LIBRARY.md` §2.5). A clean synthetic signal decodes under almost every option, so a test that only decodes one
cannot fail when an option is dropped. Each option is therefore pinned in a condition where it *decides*, found by a sweep
(noise from a fixed generator, so the conditions are reproducible). The golden tests that use the options (`wspr_wsjtx_samples`
and others) are `#[ignore]`d diagnostics: a wrong start sample passed all of the non-ignored ones, so they guard nothing in the
gate.

| mode | option | pinned by (`src/<mode>/candidate.rs`) |
|---|---|---|
| all | start, frequency, sample rate | clean slot: decodes at the alignment, not at 0, not 2-3 Hz off, not at another rate |
| WSPR | `drift_hz`, `refine_drift` | drift 3 Hz: found with the refine cascade on, missed when it is off and the drift is held; 5 Hz is out of reach either way |
| WSPR | `nblocks` | an empty ladder tries nothing |
| WSPR | `confirmed` | sigma 45, seed 0: OSD finds the message Fano loses; accepted only through a table that holds its callsign (not through one without it) |
| JT65 | `Jt65Strategy` | sigma 16, seed 0: lost by hard decisions and by the ladder `[0]`, found by `[0, 8, 16, 24, 32]` and by Chase, lost by Chase with `max_trials = 0`; seed 1 only Chase finds |
| Q65 | `ap_hint`, `ap_list` | sigma 6.5, seed 0: plain BP and fading alone lose it, the hint (alone, and with fading) and the list find it, the list with `iterations == 0` |
| Q65 | `fading` model and width | clean signal: Lorentzian at `b90_ts` 500 decodes, the Gaussian of the same width does not |
| Q65 | precedence | a list that lacks the message, with or without fading, is decoded by the list alone: no decode, no BP fallback |
| Q65 | `pileup`, `hash_table` | `tests/q65_pileup_flag.rs`, `q65::rx::tests::sniper_hash_table_resolves_hashed_callsign`: through `DecodeRequest::sniper` and `SniperRequest::decode`, so they must move with `SniperRequest` |

The deprecated builders are pinned as shims too (`the_deprecated_builder_is_the_function` in WSPR and JT65): a shim that dropped
`refine_drift`, `confirmed` or the strategy fails them.

**The streamed Q65 decode** (`q65::decode_candidate_streamed`) is pinned against the slice decode, which is the reference: on a
synthetic slot (clean and sigma 6.5; plain, hint, list, fading, a wrong list, 3 Hz off) and on the vendored 10 GHz EME recording
(the point decodes at the alignments the scan finds, and 5 Hz off: 8 compared, message, time, flag and SNR bits equal); a counting
source pins the reads (miss: 63 symbols, no restart; hit: plus 85 symbols from the start, one restart); a short source gives
nothing. Mutations caught: the source scale, the read start, the wide bins, the hint in the fading branch, the model and width,
the AP flags, the SNR, the narrow rows' start. The scale cannot change any decode (so it is pinned by `PcmWindows`' own test). Also pinned: an empty list reads nothing, a start whose frame end overflows `usize` is no decode on either side
(`checked_add`), and the SNR fallback is lazy and gets the windows (`snr::tests`); the fallback's *content* (it re-reads the wide energies)
is not pinned: no input that decodes reaches it.

**The streamed coarse search** (`q65::search::coarse_search_streamed`) is pinned against the whole-spectrogram search to the bit (start,
frequency, score, drift): a synthetic 15A slot at 1, 5 and more bands than columns, with and without a drift search; two real recordings (the
10 GHz EME file, 3 candidates, and the 6 m EME files, 44) at 1 and 8 bands, drift 0 and 4. A counting source pins the reads (a forward sweep a band,
a restart for each after the first, nothing skipped), and 10 000 bands is the columns' worth of sweeps. Mutations caught: a band's end, the drift
margins, the unscaled column mean, the frame start, the row count, the column sum, the band clamp, the scale; no guard on short audio aborts the
process. A narrow search window changes which candidates the percentile test admits (one signal of two on the synthetic slot), so the
several-candidate case is the real recordings'.

**Other platforms.** The noisy-slot conditions passed on CI's `ubuntu-latest` (x86_64, tier A+B, commit 72f6fea4) as well as on Apple arm64;
fixed-point is not run. Their original wording follows.

**Not verified: other platforms.** The noisy-slot conditions (WSPR sigma 45 seed 0, JT65 sigma 16, Q65 sigma 6.5 (here and in the
streamed test, on a 16-bit slot scaled by 1/30), the Gaussian
against the Lorentzian at 500) were chosen at the point where the options part, so they sit near a threshold. They pass on Apple
arm64 in debug, release and with one rayon thread; they have not run on x86 (rustfft's AVX rounding differs) or in
fixed-point. Run them on the Linux reference machine, or in CI, before relying on them; a flip there means the condition
needs a margin, not that the option broke.

**Before removing `SniperRequest`:** the in-crate tests that build one through the internal `DecodeRequest::sniper`
(`tests/q65_pileup_flag.rs`, `tests/q65_a15_roundtrip.rs`, `tests/q65_eme_submodes.rs`, `src/q65/rx.rs`, `src/wspr/decode.rs`,
`src/wspr/subtract.rs`, `src/jt65/mod.rs`) must move to `decode_candidate`, or they are the only guard of Q65's `pileup` and
`hash_table`.
