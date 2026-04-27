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

/// Runtime protocol tag — used at FFI boundaries where generics cannot cross
/// the C ABI. Order is stable; append new variants at the end.
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
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
    /// UvPacket — 4-FSK GFSK packet protocol for AFSK-on-SSB at U/VHF
    /// (multipath fading channels). Variable-byte payload, no UTC slot.
    /// Multiple baud-rate variants share this tag.
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
    /// soft-decision LDPC handles well. uvpacket uses a stride-25
    /// polynomial interleaver: `INTERLEAVE[j] = (7 * j) mod 174` (the
    /// modular inverse of stride 25, since 25 × 7 ≡ 1 mod 174).
    ///
    /// Both [`crate::core::tx::codeword_to_itone`] and the pipeline's
    /// LLR-deinterleave step honour this constant; protocols that
    /// override get TX/RX symmetry for free.
    const CODEWORD_INTERLEAVE: Option<&'static [u16]> = None;
}

// ──────────────────────────────────────────────────────────────────────────
// FEC
// ──────────────────────────────────────────────────────────────────────────

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
}

impl<'a> Default for FecOpts<'a> {
    fn default() -> Self {
        Self {
            bp_max_iter: 30,
            osd_depth: 0,
            ap_mask: None,
            verify_info: None,
        }
    }
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
pub trait FecCodec: Default + 'static {
    /// Codeword length.
    const N: usize;

    /// Information-bit length.
    const K: usize;

    /// Systematic encode: `info.len() == K`, `codeword.len() == N`. The first
    /// `K` bits of `codeword` must equal `info` (systematic form).
    fn encode(&self, info: &[u8], codeword: &mut [u8]);

    /// Soft-decision decode from log-likelihood ratios.
    ///
    /// `llr.len() == N`. On success returns the `K` information bits plus
    /// decoder statistics. On failure returns `None`.
    fn decode_soft(&self, llr: &[f32], opts: &FecOpts) -> Option<FecResult>;
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
    /// message format carries no inline integrity field (e.g.
    /// [`crate::msg::PacketBytesMessage`] in `uvpacket`).
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
    pub callsign_hash_table: Option<std::sync::Arc<dyn std::any::Any + Send + Sync>>,
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
