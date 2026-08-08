//! Protocol trait hierarchy.
//!
//! A `Protocol` is a zero-sized type that ties together the four axes of
//! variation across WSJT-family digital modes:
//!
//! | Axis               | Trait              | Examples                          |
//! |--------------------|--------------------|-----------------------------------|
//! | Tones / baseband   | `ModulationParams` | 8-FSK @ 6.25 Hz (FT8) vs 4-FSK (FT4) |
//! | Frame layout       | `FrameLayout`      | Costas pattern, sync positions    |
//! | FEC                | `FecCodec`         | LDPC(174,91) / Reed–Solomon / Fano |
//! | Message payload    | `MessageCodec`     | WSJT 77-bit / JT 72-bit / WSPR 50 |
//!
//! Splitting the traits lets implementations share code: FT4 reuses FT8's
//! `Ldpc174_91` and `Wsjt77Message` and differs only in `ModulationParams` +
//! `FrameLayout`, so SIMD optimisations to the shared LDPC decoder
//! automatically benefit every LDPC-based protocol.

use alloc::string::String;
use alloc::vec::Vec;

/// Runtime protocol tag — used at FFI boundaries where generics cannot cross
/// the C ABI. Order is stable; append new variants at the end.
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ProtocolId {
    /// FT8 — 15 s slot, 8-FSK, LDPC(174,91), 77-bit message.
    Ft8 = 0,
    /// FT4 — 7.5 s slot, 4-FSK, LDPC(174,91), 77-bit message.
    Ft4 = 1,
    /// FT2 (experimental / contest variant).
    Ft2 = 2,
    /// FST4 — 60 s slot, 4-FSK, LDPC(240,101) + CRC-24, 77-bit message.
    Fst4 = 3,
    /// JT65 — 60 s slot, 65-tone FSK, Reed-Solomon(63,12), 72-bit message.
    Jt65 = 4,
    /// JT9 — 60 s slot, 9-FSK, convolutional r=½ K=32 + Fano, 72-bit message.
    Jt9 = 5,
    /// WSPR — 120 s slot, 4-FSK, convolutional r=½ K=32 + Fano, 50-bit message.
    Wspr = 6,
    /// Q65 — 65-tone FSK, QRA(15,65) over GF(64), 77-bit Wsjt77 message.
    /// Multiple T/R-period × tone-spacing variants share this tag at the
    /// FFI level; the protocol-layer ZST disambiguates.
    Q65 = 7,
    /// uvpacket — 4-GFSK packet protocol for narrow-FM voice channels
    /// at U/VHF (Rayleigh-fading-tolerant). 4 sub-modes share this
    /// family ID; the protocol-layer ZST disambiguates.
    UvPacket = 8,
}

/// Baseband modulation parameters (tones, symbol rate, Gray mapping, Gaussian
/// shaping and the tunable DSP ratios the pipeline reads per protocol).
///
/// All constants are evaluated at compile time; the trait carries no data so
/// implementors are typically zero-sized types.
pub trait ModulationParams: Copy + Default + 'static {
    /// Number of FSK tones (M in M-ary FSK).
    const NTONES: u32;

    /// Information bits carried per modulated symbol (= log2(NTONES)).
    const BITS_PER_SYMBOL: u32;

    /// Samples per symbol at the 12 kHz pipeline sample rate.
    const NSPS: u32;

    /// Symbol duration in seconds (= NSPS / 12000).
    const SYMBOL_DT: f32;

    /// Spacing between adjacent tones, in Hz.
    const TONE_SPACING_HZ: f32;

    /// Gray-code map: `GRAY_MAP[tone_index]` returns the NATURAL-bit pattern
    /// for that tone. The map covers at least the data alphabet
    /// (`2^BITS_PER_SYMBOL` entries) and at most the full tone set
    /// (`NTONES` entries). Protocols whose sync tones are part of
    /// the data alphabet (FT8 / FT4 / FST4 / WSPR) have
    /// `len() == NTONES == 2^BITS_PER_SYMBOL`; protocols that
    /// reserve additional sync-only tones (JT9, JT65, Q65) either
    /// trim the map to the data alphabet (JT9: 8 entries for 9
    /// tones) or extend it with identity over the sync slots
    /// (JT65 / Q65). Pinned by `tests/protocol_invariants.rs`.
    const GRAY_MAP: &'static [u8];

    // ── GFSK shaping ────────────────────────────────────────────────────
    /// Gaussian bandwidth-time product. FT8 = 2.0, FT4 = 1.0, FST4 ≈ 1.0.
    const GFSK_BT: f32;
    /// Modulation index h — the phase increment per symbol is `2π · h`.
    /// FT8 and FT4 both use 1.0 (orthogonal tones at `1/T` spacing).
    const GFSK_HMOD: f32;

    // ── Per-protocol DSP ratios ─────────────────────────────────────────
    /// Per-symbol FFT size = `NSPS * NFFT_PER_SYMBOL_FACTOR`.
    /// FT8 = 2 (window is 2·NSPS), FT4 = 4 (window is 4·NSPS) — trade-off
    /// between frequency resolution and time localisation.
    const NFFT_PER_SYMBOL_FACTOR: u32;
    /// Coarse-sync time-step = `NSPS / NSTEP_PER_SYMBOL`.
    /// FT8 = 4 (quarter-symbol resolution), FT4 = 1 (symbol-granular).
    const NSTEP_PER_SYMBOL: u32;
    /// Downsample decimation factor: baseband rate = `12 000 / NDOWN` Hz.
    /// FT8 = 60 (→200 Hz), FT4 = 18 (→667 Hz). Proportional to tone spacing.
    const NDOWN: u32;

    /// LLR scale factor applied after standard-deviation normalisation.
    /// FT8 uses 2.83 (empirical, from WSJT-X ft8b.f90). Different
    /// bits-per-symbol counts may shift the optimum — FT4's 2-bit LLR
    /// dynamics are not identical to FT8's 3-bit case.
    const LLR_SCALE: f32 = 2.83;

    /// Maximum coherent-integration depth for the 3rd LLR variant.
    /// `compute_llr` builds three variants `llra/llrb/llrc` from
    /// `nsym` ∈ `{1, 2, LLR_NSYM_MAX}` symbol blocks. WSJT-X uses
    /// `nsym=1, 2, 4` for FT4 (`get_ft4_bitmetrics.f90:69-71`); we
    /// default to `nsym=3` (FT8 path is calibrated to it). FT4
    /// overrides to `4` for an extra ~3 dB SNR boost on stable
    /// signals — closes the recall gap on real-WAV recordings. FST4
    /// overrides to `8`, matching WSJT-X's own 1/2/4/8-symbol
    /// correlation ladder in `get_fst4_bitmetrics.f90` (issue #146 —
    /// FST4 had silently been using the uncalibrated FT8 default of
    /// 3, never wired to its own bit-metric depth).
    /// Any value ≥ 1 works — `nt = NTONES^nsym` combination
    /// hypotheses are computed generically, no per-nsym lookup table
    /// — but cost grows exponentially with `nsym`, so keep it at the
    /// WSJT-X-matched depth for the protocol rather than raising it
    /// further.
    const LLR_NSYM_MAX: u32 = 3;

    /// Optional extra coherent-integration depth strictly between the
    /// `nsym=2` and `nsym=LLR_NSYM_MAX` variants, populating [`LlrSet`]'s
    /// `llre` slot. `None` for every protocol whose ladder has no gap
    /// (FT8: {1,2,3}; FT4: {1,2,4}) — `llre` stays empty and costs
    /// nothing. FST4 overrides to `Some(4)`: WSJT-X's
    /// `get_fst4_bitmetrics.f90` tries all four of nsym ∈ {1,2,4,8}
    /// (`fst4_decode.f90:429-433`), but `LLR_NSYM_MAX=8` alone only
    /// gives `compute_llr_generic` two depths + the deepest ({1,2,8}) —
    /// nsym=4 would otherwise never run. Diagnostic measurement (issue
    /// #146, `fst4_diag_nsym4_ladder` in `tests/fst4_sweep.rs`) found a
    /// real but modest effect: a standalone nsym=4 pass recovered 4/43
    /// (~9%) of near-threshold FST4-30 AWGN failures and 2/38 (~5%) of
    /// FST4-300's, over and above the existing {1,2,8,d} ladder.
    ///
    /// [`LlrSet`]: crate::engine::llr::LlrSet
    const LLR_NSYM_MID: Option<u32> = None;

    /// Optional 77-bit pre-LDPC scrambler. WSJT-X applies an
    /// FT4-specific scrambler in `genft4.f90:64`
    /// (`msgbits=mod(msgbits+rvec,2)`) before computing CRC-14 and
    /// running LDPC encode; the receiver removes it after LDPC
    /// decode + CRC verify (`ft4_decode.f90:430`). Without this our
    /// decoder converges on a valid codeword whose unscrambled
    /// payload is the WSJT-X-transmitted message — but emerges as
    /// nonsense because we never undo the XOR.
    ///
    /// Default `None` (FT8 / others don't scramble); FT4 and FST4
    /// both override to the same 77-element `rvec` (WSJT-X
    /// `genfst4.f90:29-31` uses the identical array to
    /// `genft4.f90`'s). Length must be 77 when set.
    const INFO_SCRAMBLE_RVEC: Option<&'static [u8]> = None;

    /// Window function applied per `NSPS`-sample chunk in
    /// [`crate::engine::sync::compute_spectra`] before the NFFT1 FFT.
    /// Default = [`SpectrumWindow::Rectangular`] (preserves FT8's
    /// existing synth-roundtrip behaviour); FT4 overrides to
    /// [`SpectrumWindow::Nuttall4`] to match WSJT-X
    /// `getcandidates4.f90:22` and suppress sidelobe leakage that
    /// otherwise inflates the per-bin baseline near strong signals.
    const SPECTRUM_WINDOW: SpectrumWindow = SpectrumWindow::Rectangular;
}

/// Window function applied to each NSPS-sample chunk before the
/// coarse-sync FFT. See [`ModulationParams::SPECTRUM_WINDOW`].
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SpectrumWindow {
    /// No window (multiplied by 1.0). Default. Suitable for synth
    /// roundtrip and for protocols whose sync metric tolerates
    /// rectangular-window sidelobes.
    Rectangular,
    /// Nuttall-4 window (a0=0.3635819, a1=0.4891775, a2=0.1365995,
    /// a3=0.0106411). Matches WSJT-X `getcandidates4.f90`/
    /// `getcandidates.f90` (FT4 / FT8 respectively in WSJT-X, though
    /// in our port only FT4 currently opts in — FT8's existing path
    /// is calibrated to rectangular).
    Nuttall4,
}

/// One Costas / pilot block: a contiguous run of tones starting at a specific
/// symbol index within the frame.
///
/// FT8 has three identical blocks (positions 0/36/72, same Costas-7 pattern);
/// FT4 has four *different* blocks (positions 0/33/66/99, each a permutation
/// of `[0,1,2,3]`). The trait is shaped to accommodate both.
#[derive(Copy, Clone, Debug)]
pub struct SyncBlock {
    /// Symbol index (0-based) where this block starts.
    pub start_symbol: u32,
    /// Tone sequence for this block. `pattern.len()` is the block length.
    pub pattern: &'static [u8],
}

/// How sync information is carried in the channel symbol stream.
///
/// * `Block` — dedicated contiguous sync blocks (Costas arrays) occupy
///   specific symbol positions, with data symbols filling the rest. Used by
///   FT8, FT4, FST4.
/// * `Interleaved` — every channel symbol carries one sync bit (fixed
///   position within the tone index) AND payload bits. The sync bits
///   concatenated across the frame form a known pseudorandom vector.
///   Used by WSPR: `tone = 2·data_bit + sync_bit`, so LSB of each
///   4-FSK symbol reproduces the 162-bit `npr3` sync vector.
#[derive(Copy, Clone, Debug)]
pub enum SyncMode {
    Block(&'static [SyncBlock]),
    Interleaved {
        /// Position of the sync bit within the tone index, LSB-first.
        /// WSPR = 0 (LSB).
        sync_bit_pos: u8,
        /// Sync vector, one bit per frame symbol. Length == `N_SYMBOLS`.
        vector: &'static [u8],
    },
}

impl SyncMode {
    /// Block list for `Block` mode; empty slice for `Interleaved`.
    /// Sync/LLR/TX helpers that only handle block-structured sync can iterate
    /// this unconditionally — they will no-op on WSPR-style protocols, which
    /// then need their own interleaved-sync pipeline entry point.
    pub const fn blocks(&self) -> &'static [SyncBlock] {
        match self {
            SyncMode::Block(b) => b,
            SyncMode::Interleaved { .. } => &[],
        }
    }
}

/// Frame structure: data / sync symbol counts, the ordered list of sync
/// blocks, and the TX-side nominal start offset.
pub trait FrameLayout: Copy + Default + 'static {
    /// Data symbols carrying FEC-coded payload.
    const N_DATA: u32;

    /// Sync symbols (sum of `pattern.len()` across `SYNC_BLOCKS`).
    const N_SYNC: u32;

    /// Total channel symbols per frame (= N_DATA + N_SYNC). Excludes any
    /// GFSK ramp-up / ramp-down symbols that are a shaping artifact.
    const N_SYMBOLS: u32;

    /// Extra symbol slots on each side of the frame reserved for amplitude
    /// ramp (FT4 has 1 each side = 2; FT8 has 0 — ramp absorbed into the
    /// first/last data symbol envelope). Applied at the transmitter.
    const N_RAMP: u32;

    /// Sync-symbol layout. Most WSJT protocols use `SyncMode::Block` with
    /// dedicated Costas blocks (FT8/FT4/FST4); WSPR uses `SyncMode::Interleaved`
    /// with a per-symbol sync bit. Callers that only support block sync should
    /// read `SYNC_MODE.blocks()` and treat an empty slice as "unsupported".
    const SYNC_MODE: SyncMode;

    /// Nominal TX/RX slot length in seconds (informational — used by
    /// schedulers and UI, not by the DSP pipeline). FT8 = 15 s, FT4 = 7.5 s.
    const T_SLOT_S: f32;

    /// Time (seconds) from the start of the slot-audio buffer to the start
    /// of the first frame symbol — the "dt = 0" reference point used by
    /// sync, signal subtraction, and DT reporting. FT8 = 0.5, FT4 = 0.5.
    const TX_START_OFFSET_S: f32;

    /// Optional bit interleaver: permutation table such that
    /// `cw[CODEWORD_INTERLEAVE[j]]` is the codeword bit transmitted at
    /// **channel-bit position** `j`. Length must equal
    /// `<Self as Protocol>::Fec::N` when `Some`.
    ///
    /// `None` (default) means the codeword bits flow into the channel in
    /// natural order — what FT8 / FT4 / FST4 / WSPR / JT9 / JT65 / Q65
    /// all do, since their existing FECs and operating channels make
    /// burst-error tolerance a non-issue (or it's handled inside the FEC,
    /// as Q65's QRA does symbol-level dispersion).
    ///
    /// `Some(table)` is for codecs targeting **time-selective fading**
    /// channels where a deep fade null can wipe out consecutive channel
    /// bits. The interleaver spreads consecutive codeword bits across the
    /// frame so the same fade null hits scattered codeword bits, which
    /// soft-decision LDPC handles well. The table is a permutation of
    /// `0..codeword_bits`; a polynomial form `INTERLEAVE[j] = (s * j)
    /// mod n` with `gcd(s, n) = 1` gives uniform stride spacing.
    ///
    /// Both [`crate::engine::tx::codeword_to_itone`] and the pipeline's
    /// LLR-deinterleave step honour this constant; protocols that
    /// override get TX/RX symmetry for free.
    const CODEWORD_INTERLEAVE: Option<&'static [u16]> = None;
}

// ──────────────────────────────────────────────────────────────────────────
// FEC
// ──────────────────────────────────────────────────────────────────────────

/// LDPC belief-propagation check-node update kernel.
///
/// `SumProduct` is the WSJT-X-equivalent log-domain sum-product update
/// (the `2·atanh(∏ tanh(L/2))` formula). `NormalizedMinSum` and
/// `OffsetMinSum` are min-sum approximations that skip the
/// transcendental functions entirely — significantly faster on
/// FPU-poor embedded targets at a small (typically <0.2 dB on Q65 /
/// FT8 / FT4 thresholds with α=0.75 or β=0.5) SNR cost.
///
/// Both min-sum variants use the standard min1/min2 trick (track the
/// two smallest |L| at each check node) plus XOR-accumulated signs,
/// so the per-iteration cost is roughly O(check_degree) instead of
/// the sum-product's O(check_degree²) for the per-edge `tanh`-cache
/// lookups.
///
/// Use `SumProduct` on host targets (default), `NormalizedMinSum` or
/// `OffsetMinSum` on `no_std` / FPU-limited builds.
#[derive(Copy, Clone, Debug, Default)]
pub enum BpKind {
    /// WSJT-X-equivalent log-domain sum-product. Default — best
    /// accuracy, reference output.
    #[default]
    SumProduct,
    /// Normalised min-sum: `L_c→v ≈ α · sign(∏) · min|L|`. Typical
    /// `alpha ≈ 0.75`. Trades ~0.05–0.15 dB threshold for ~3-5×
    /// faster check-node update on f32, more on fixed-point.
    NormalizedMinSum {
        /// Magnitude scale factor, typically `0.7..=0.9`.
        alpha: f32,
    },
    /// Offset min-sum: `L_c→v ≈ sign(∏) · max(min|L| − β, 0)`.
    /// Typical `beta ≈ 0.5`. Performs slightly differently from NMS
    /// near low SNR; included for sweep comparison and parity with
    /// the LDPC literature.
    OffsetMinSum {
        /// Magnitude offset, typically `0.0..=1.0`.
        beta: f32,
    },
}

/// Options controlling FEC decoding depth / fall-backs.
///
/// This is deliberately a plain data struct rather than a trait — it describes
/// *how* to decode, not *what* code to use. Codecs ignore fields that don't
/// apply (e.g. convolutional decoders ignore `osd_depth`).
#[derive(Copy, Clone, Debug)]
pub struct FecOpts<'a> {
    /// Maximum belief-propagation iterations (LDPC).
    pub bp_max_iter: u32,
    /// Ordered-statistics-decoding search depth (0 disables OSD fallback).
    pub osd_depth: u32,
    /// Optional a-priori hint: bits whose LLR should be clamped to a strong
    /// known value before decoding. `Some((mask, values))` where `mask[i] == 1`
    /// means `values[i]` is locked to `values[i]`.
    ///
    /// Lifetime is per-call: the caller allocates the AP vectors for the
    /// duration of this decode — typical usage builds a `Vec<u8>` from an
    /// `ApHint` and borrows into `FecOpts` for a single `decode_soft` call.
    pub ap_mask: Option<(&'a [u8], &'a [u8])>,
    /// Optional integrity verifier called when the FEC reaches a
    /// parity-converged candidate. Returning `false` rejects the
    /// candidate and BP keeps iterating; returning `true` accepts.
    /// `None` accepts unconditionally — appropriate for FEC users
    /// whose message codec carries no inline integrity field.
    ///
    /// Typical use: pipeline code threads `<P::Msg as
    /// MessageCodec>::verify_info` here so that, e.g., FT8/FT4/FST4
    /// reject parity-only candidates whose CRC-14 doesn't pass.
    pub verify_info: Option<fn(&[u8]) -> bool>,
    /// LDPC BP check-node update kernel. Defaults to `SumProduct`
    /// (WSJT-X-equivalent). Embedded callers select
    /// `NormalizedMinSum { alpha: 0.75 }` to trade ~0.1 dB threshold
    /// for substantially faster decode on f32 / fixed-point math.
    pub bp_kind: BpKind,
    /// Override for the Fano sequential decoder's per-information-bit
    /// cycle budget. `None` = codec-supplied default. WSJT-X's own
    /// `jt9_decode.f90:84-101` ladder is `limit=5000` (base/`-d1`) →
    /// `10000` (`-d2`) → `30000` (`-d3`) → `100000` ("Decode Again");
    /// JT9's [`crate::jt9::decode::Jt9Depth`] exposes the same four
    /// tiers.
    pub max_cycles_per_bit: Option<u64>,
}

impl<'a> Default for FecOpts<'a> {
    fn default() -> Self {
        Self {
            bp_max_iter: 30,
            osd_depth: 0,
            ap_mask: None,
            verify_info: None,
            bp_kind: BpKind::SumProduct,
            max_cycles_per_bit: None,
        }
    }
}

/// Seals [`FecCodec`] against downstream implementors (issue #198).
/// `pub(crate)` — visible anywhere in this crate (every implementor
/// lives in a different module) but unnameable outside it.
pub(crate) mod sealed {
    pub trait Sealed {}
}

/// Result of a successful FEC decode.
#[derive(Clone, Debug)]
pub struct FecResult {
    /// Hard-decision information bits (length = `FecCodec::K`).
    pub info: Vec<u8>,
    /// Number of hard-decision errors corrected (for quality metric).
    pub hard_errors: u32,
    /// Iterations consumed (0 if N/A).
    pub iterations: u32,
}

/// Forward-error-correction codec: maps `K` information bits ↔ `N` codeword
/// bits.
///
/// Implementors MUST be `Default`-constructible so generic pipeline code can
/// obtain an instance via `P::Fec::default()` without plumbing state.
/// Stateless codecs (matrices in `const` / `static`) are the common case.
///
/// # Symbol granularity
///
/// The trait surface speaks in **bits**: `&[u8]` info / codeword, `&[f32]`
/// bit-LLRs, `K` and `N` counted in bits. Non-binary codes (Q65's QRA over
/// GF(2⁶), JT65's RS over GF(2⁶)) implement this surface by packing /
/// unpacking bits ↔ symbols inside their own `encode`, and by using a
/// private symbol-level decode path that lives outside `decode_soft`. In
/// particular [`crate::q65::Q65Fec::decode_soft`] returns `None` by design —
/// the real Q65 decode runs over GF(64) probability vectors via
/// [`crate::fec::qra::Q65Codec`] and is invoked from
/// [`crate::q65::DecodeRequest`]/[`crate::q65::SniperRequest`], not through
/// this trait.
///
/// Counting `K` / `N` in bits keeps the cross-protocol invariant
/// `FecCodec::N ≤ N_DATA × BITS_PER_SYMBOL` (pinned in
/// `tests/protocol_invariants.rs::assert_codec_consistency`) meaningful for
/// both binary (LDPC, conv) and non-binary (RS, QRA) codes.
///
/// Sealed (issue #198, pre-0.8.0 public-API review): `decode_soft` is
/// f32-hardcoded today, which blocks fixed-point BP from ever running
/// through the generic pipeline for FT4/FST4/MSK144 (FT8 only gets it
/// via its own separate bespoke engine, which bypasses this trait
/// entirely). Generic-izing the signature (`decode_soft<T: LlrScalar>`)
/// is future work with its own numerical-verification cost; sealing
/// now means that redesign can land as a signature change on the
/// existing implementors (`Ldpc174_91`, `Ldpc240_101`, `Ldpc128_90`,
/// `ConvFano`, `ConvFano232`, `Rs63_12`, `Q65Fec`) without being a
/// breaking change for any
/// downstream implementor, since none can exist — `sealed::Sealed` is
/// private to this crate.
pub trait FecCodec: sealed::Sealed + Default + 'static {
    /// Codeword length, in **bits** (regardless of the underlying symbol
    /// alphabet — see "Symbol granularity" above).
    const N: usize;

    /// Information-bit length.
    const K: usize;

    /// Systematic encode: `info.len() == K`, `codeword.len() == N`. The first
    /// `K` bits of `codeword` must equal `info` (systematic form).
    /// Non-binary codes pack bits into their native symbols internally.
    fn encode(&self, info: &[u8], codeword: &mut [u8]);

    /// Soft-decision decode from log-likelihood ratios.
    ///
    /// `llr.len() == N`. On success returns the `K` information bits plus
    /// decoder statistics. On failure returns `None`.
    ///
    /// Non-binary codes whose natural decode operates on symbol-level
    /// probability vectors (Q65) MAY return `None` unconditionally and
    /// expose their real decode through a protocol-specific entry point.
    fn decode_soft(&self, llr: &[f32], opts: &FecOpts) -> Option<FecResult>;
}

/// `Fec` codecs with a pooled-scratch `decode_soft` sibling (perf
/// review: `engine::pipeline::process_candidate_basic_impl`'s LLR/OSD
/// staircase calls `decode_soft` up to 15× per FST4 candidate, 12× for
/// FT4, each allocating ~9 fresh `Vec`s internally with no pooling —
/// the same shape of problem `fec::ldpc::bp::BpScratch`/issue #199/#201
/// already fixed for FT8's own `NormalizedMinSum` kernel, just never
/// ported to the generic pipeline or to the `SumProduct` kernel
/// [`FecOpts::default`] actually selects). Not on the sealed
/// [`FecCodec`] itself — that would force a `Scratch` type onto
/// `Q65Fec`/`ConvFano`/`Ldpc128_90` too, none of which want LDPC BP
/// scratch. Implemented for `Ldpc174_91`/`Ldpc240_101`
/// (`crate::fec::ldpc`/`crate::fec::ldpc240_101`) only — the two `Fec`
/// types `Ft4` and every FST4 sub-mode actually use. Lives next to
/// [`FecCodec`] (not in `engine::pipeline`, where it's consumed) so
/// its implementors — `fec::ldpc`/`fec::ldpc240_101` — don't need a
/// new dependency on `engine::pipeline`, only their existing one on
/// this module.
///
/// `pub` only under the `internal-testing` feature (issue #203) —
/// mirrors `engine::pipeline::GenericPipelineProtocol`'s own gating,
/// since that trait's `where Self::Fec: BpPooledFec` bound needs
/// `BpPooledFec` to be at least as visible as whatever visibility
/// `GenericPipelineProtocol`-bounded functions have on a given feature
/// set (Rust's `private_bounds` lint) — not because any external
/// caller ever names `BpPooledFec` itself; the bound is satisfied
/// automatically by `P: GenericPipelineProtocol` alone.
///
/// `#[allow(dead_code)]`: unlike `GenericPipelineProtocol` (an empty
/// marker trait, nothing for the lint to flag), this trait has a real
/// method (`decode_soft_pooled`). Under a feature set with no protocol
/// that reaches `GenericPipelineProtocol`'s generic pipeline (e.g.
/// `--no-default-features`, `--features wspr`) nothing ever calls it,
/// even though `impl BpPooledFec for Ldpc174_91`/`Ldpc240_101` are
/// unconditionally compiled — caught by CI's `-D warnings` build
/// matrix, not by the `full`/`full,internal-testing` combo this was
/// developed against.
#[cfg(feature = "internal-testing")]
pub trait BpPooledFec: FecCodec {
    /// Reusable working memory for [`Self::decode_soft_pooled`] —
    /// concretely `fec::ldpc::bp::BpScratch` for both current
    /// implementors.
    type Scratch: Default;

    /// [`FecCodec::decode_soft`] with caller-provided scratch, reused
    /// across every call for one candidate instead of reallocated per
    /// call.
    fn decode_soft_pooled(
        &self,
        llr: &[f32],
        opts: &FecOpts,
        scratch: &mut Self::Scratch,
    ) -> Option<FecResult>;
}
#[cfg(not(feature = "internal-testing"))]
#[allow(dead_code)] // see the `internal-testing` branch's doc comment above
pub(crate) trait BpPooledFec: FecCodec {
    type Scratch: Default;

    fn decode_soft_pooled(
        &self,
        llr: &[f32],
        opts: &FecOpts,
        scratch: &mut Self::Scratch,
    ) -> Option<FecResult>;
}

// ──────────────────────────────────────────────────────────────────────────
// Message codec
// ──────────────────────────────────────────────────────────────────────────

/// Human-facing message payload codec (callsigns, grids, reports, free text).
///
/// Operates on the FEC-decoded information bits (`PAYLOAD_BITS` wide, NOT
/// including any CRC protecting them — callers handle the CRC layer).
///
/// Unlike `FecCodec`, this trait is an acceptable place for `dyn` when the
/// caller juggles heterogeneous protocols at runtime (FFI, CLI dump tools):
/// message unpacking is a cold path relative to DSP/FEC inner loops.
pub trait MessageCodec: Default + 'static {
    /// Decoded high-level representation returned by `unpack`.
    type Unpacked;

    /// Number of information bits consumed by `pack` / produced by `unpack`.
    const PAYLOAD_BITS: u32;

    /// CRC width guarding the payload during transmission (0 if the FEC itself
    /// provides all error detection, as with JT65 Reed–Solomon).
    const CRC_BITS: u32;

    /// Encode high-level fields to a bit vector of length `PAYLOAD_BITS`.
    /// Returns `None` on encoding failure (invalid callsign format, overflow…).
    fn pack(&self, fields: &MessageFields) -> Option<Vec<u8>>;

    /// Decode a `PAYLOAD_BITS`-long bit vector to the protocol-specific
    /// unpacked representation. `ctx` carries side information such as the
    /// callsign-hash table.
    fn unpack(&self, payload: &[u8], ctx: &DecodeContext) -> Option<Self::Unpacked>;

    /// Verify the integrity of post-FEC info bits. The FEC layer
    /// invokes this when a candidate codeword satisfies parity:
    /// returning `true` accepts the codeword; returning `false`
    /// causes the FEC to keep iterating.
    ///
    /// Default: accept unconditionally — appropriate for codecs whose
    /// message format carries no inline integrity field (the FEC layer
    /// has already enforced parity convergence by the time this is
    /// called).
    ///
    /// CRC-bearing codecs override this. For example,
    /// [`crate::msg::Wsjt77Message`] verifies the CRC-14 stored in
    /// info bits 77..91. The associated-function (no `&self`) shape
    /// keeps the verifier compatible with the function-pointer field
    /// on [`FecOpts::verify_info`].
    fn verify_info(info: &[u8]) -> bool {
        let _ = info;
        true
    }
}

/// Generic input to `MessageCodec::pack` — protocol-specific codecs accept
/// the subset of fields they understand and return `None` for unsupported
/// combinations.
#[derive(Clone, Debug, Default)]
pub struct MessageFields {
    pub call1: Option<String>,
    pub call2: Option<String>,
    pub grid: Option<String>,
    pub report: Option<i32>,
    pub free_text: Option<String>,
}

/// Side information passed to `MessageCodec::unpack`.
///
/// `callsign_hash_table` is an opaque pointer the protocol crate
/// downcasts to its own table type — generic code does not need to know the
/// shape. This keeps `mfsk-msg` optional at the `mfsk-core` level.
#[derive(Clone, Debug, Default)]
pub struct DecodeContext {
    /// Optional hashed-callsign lookup owned by the caller. Concrete layout is
    /// protocol-defined; interpret via `Any::downcast_ref` inside the codec.
    pub callsign_hash_table: Option<alloc::sync::Arc<dyn core::any::Any + Send + Sync>>,
}

// ──────────────────────────────────────────────────────────────────────────
// Protocol facade
// ──────────────────────────────────────────────────────────────────────────

/// The full protocol description: ties `ModulationParams`, `FrameLayout`, a
/// FEC codec and a message codec together under one trait for ergonomic
/// `<P: Protocol>` bounds.
pub trait Protocol: ModulationParams + FrameLayout + 'static {
    /// FEC codec carrying `N_DATA * BITS_PER_SYMBOL` coded bits.
    type Fec: FecCodec;

    /// Message codec consuming the FEC-decoded information bits.
    type Msg: MessageCodec;

    /// Runtime tag used at FFI / WASM boundaries.
    const ID: ProtocolId;
}
