//! Protocol-generic transmit-side helpers: message bits → tone sequence.
//!
//! The tone sequence assembly (slot Costas arrays into their positions, map
//! LDPC codeword bits into per-symbol Gray-coded tone indices) is protocol-
//! agnostic given [`Protocol`]: iterate `SYNC_BLOCKS` and fill each data
//! chunk between consecutive blocks with `chunk_len × BITS_PER_SYMBOL` bits.
//!
//! GFSK waveform synthesis lives in [`super::dsp::gfsk`] — the `tones_to_*`
//! helpers there consume the output of this module.

use alloc::vec;
use alloc::vec::Vec;

use super::dsp::{cpfsk, gfsk::GfskCfg};
use super::{FecCodec, FrameLayout, MessageCodec, ModulationParams, Protocol};

/// Ordered list of `(first_data_symbol, chunk_len_in_symbols)` covering
/// every data slot in the frame — leading slots before the first sync
/// block, slots between consecutive sync blocks, and trailing slots
/// after the last sync block. Chunks of zero length are omitted.
///
/// FT8: `[(7, 29), (43, 29)]` — sync at 0/36/72, last block ends at the
/// end of the frame so there is no trailing chunk.
/// FT4: `[(4, 29), (37, 29), (70, 29)]` — same shape.
/// A hypothetical protocol with sync at frame head plus mid-frame would
/// produce a non-empty trailing chunk after the mid-frame sync block.
pub fn data_chunks<P: Protocol>() -> Vec<(usize, usize)> {
    let blocks = P::SYNC_MODE.blocks();
    let n_sym = P::N_SYMBOLS as usize;
    let mut chunks: Vec<(usize, usize)> = Vec::with_capacity(blocks.len() + 1);

    // Leading data (before the first sync block). All existing
    // WSJT-family protocols put their first sync block at symbol 0
    // so this chunk is empty in practice; the branch is here for
    // protocols whose first sync sits later in the frame.
    if let Some(first) = blocks.first() {
        let pre = first.start_symbol as usize;
        if pre > 0 {
            chunks.push((0, pre));
        }
    } else if n_sym > 0 {
        // Pathological case: no sync blocks at all (shouldn't happen
        // for `SyncMode::Block`-using protocols). Treat the entire
        // frame as one data chunk so the helper degrades gracefully.
        chunks.push((0, n_sym));
        return chunks;
    }

    // Slots between consecutive sync blocks.
    for i in 0..blocks.len().saturating_sub(1) {
        let after = blocks[i].start_symbol as usize + blocks[i].pattern.len();
        let before_next = blocks[i + 1].start_symbol as usize;
        if before_next > after {
            chunks.push((after, before_next - after));
        }
    }

    // Trailing data (after the last sync block). FT8/FT4 put their
    // final sync at the tail so this is empty; protocols with mid-
    // frame-only sync layouts produce a non-empty trailing chunk.
    if let Some(last) = blocks.last() {
        let after_last = last.start_symbol as usize + last.pattern.len();
        if n_sym > after_last {
            chunks.push((after_last, n_sym - after_last));
        }
    }

    chunks
}

/// Convert an LDPC codeword (MSB-first per symbol group) into the `N_SYMBOLS`
/// tone-index sequence. Sync blocks are slotted into their positions from
/// `Protocol::SYNC_BLOCKS`; data symbols consume `BITS_PER_SYMBOL` codeword
/// bits each, passed through the Gray map.
///
/// When [`P::CODEWORD_INTERLEAVE`](crate::engine::FrameLayout::CODEWORD_INTERLEAVE)
/// is `Some`, the codeword bits are read in interleaved order: channel bit
/// position `j` gets `cw[INTERLEAVE[j]]`. This is the TX half of the
/// burst-error-tolerance scheme available for fading-channel protocols;
/// protocols with the default `None` constant get the historical
/// natural-order behaviour.
///
/// Panics if `cw.len() < total_data_symbols × BITS_PER_SYMBOL`.
pub fn codeword_to_itone<P: Protocol>(cw: &[u8]) -> Vec<u8> {
    let n_sym = P::N_SYMBOLS as usize;
    let bps = P::BITS_PER_SYMBOL as usize;
    let gray = P::GRAY_MAP;
    let interleave = P::CODEWORD_INTERLEAVE;

    let mut itone = vec![0u8; n_sym];

    for block in P::SYNC_MODE.blocks() {
        let start = block.start_symbol as usize;
        for (i, &c) in block.pattern.iter().enumerate() {
            itone[start + i] = c;
        }
    }

    let chunks = data_chunks::<P>();
    let mut cw_offset = 0usize;
    for (start_sym, chunk_len) in chunks {
        for k in 0..chunk_len {
            let b = cw_offset + k * bps;
            let mut v = 0u8;
            for j in 0..bps {
                let cw_idx = match interleave {
                    Some(table) => table[b + j] as usize,
                    None => b + j,
                };
                v = (v << 1) | (cw[cw_idx] & 1);
            }
            itone[start_sym + k] = gray[v as usize];
        }
        cw_offset += chunk_len * bps;
    }

    itone
}

/// Encode a 77-bit message into `P`'s tone sequence — the whole transmit
/// chain up to the waveform, for any protocol whose FEC takes the message
/// plus a CRC (FT8, FT4 and every FST4 sub-mode):
///
/// ```text
/// message77 ─(INFO_SCRAMBLE_RVEC)→ scrambled ─(Msg::append_crc)→ info[K]
///           ─(Fec::encode)→ codeword[N] ─(codeword_to_itone)→ tones
/// ```
///
/// One function where there used to be three, `ft8::wave_gen`,
/// `ft4::encode` and `fst4::encode` each carrying its own copy (#391).
/// They differed only in data the trait already carries: FT4 and FST4
/// XOR the message with an RVEC first (WSJT-X `genft4.f90:67`,
/// `genfst4.f90:63`) and FT8 does not; FST4's LDPC(240, 101) wants a
/// CRC-24 where the others want a CRC-14, which is the message codec's
/// length dispatch.
///
/// The CRC is computed over the **scrambled** bits, as WSJT-X does, so
/// the receiver's CRC check runs before it unscrambles.
///
/// Panics if `P::Msg` is not a 77-bit codec with a CRC matching
/// `P::Fec::K` — a programming error, not a runtime condition.
pub fn message_to_tones<P: Protocol>(message77: &[u8; 77]) -> Vec<u8> {
    assert_eq!(
        <P::Msg as MessageCodec>::PAYLOAD_BITS,
        77,
        "message_to_tones: not a 77-bit message codec"
    );
    let mut info = vec![0u8; P::Fec::K];
    info[..77].copy_from_slice(message77);
    if let Some(rvec) = <P as ModulationParams>::INFO_SCRAMBLE_RVEC {
        for (b, &r) in info[..77].iter_mut().zip(rvec) {
            *b = (*b ^ r) & 1;
        }
    }
    assert!(
        <P::Msg as MessageCodec>::append_crc(&mut info),
        "message_to_tones: the message codec has no CRC for K = {}",
        P::Fec::K
    );
    info_to_tones::<P>(&info)
}

/// FEC-encode `P::Fec::K` info bits (message plus CRC, already
/// scrambled) and map the codeword to tones. The tail of
/// [`message_to_tones`], and what the receiver uses to rebuild a decoded
/// signal's tones for SNR estimation and subtraction: a decode's info
/// bits passed the CRC, so they are already in this form.
pub fn info_to_tones<P: Protocol>(info: &[u8]) -> Vec<u8> {
    let mut cw = vec![0u8; P::Fec::N];
    P::Fec::default().encode(info, &mut cw);
    codeword_to_itone::<P>(&cw)
}

/// How a protocol's tone sequence becomes audio — the one thing that
/// separates the two transmit families WSJT-X has.
///
/// WSJT-X picks the family by the sign of the `toneSpacing` it hands
/// `Modulator::start` (see [`super::dsp::envelope`]'s module doc):
/// FT8, FT4 and FST4 transmit a pre-computed GFSK waveform with a
/// raised-cosine ramp (`gen_ft8wave.f90`, `gen_ft4wave.f90`,
/// `gen_fst4wave.f90`); WSPR, JT9, JT65 and Q65 are plain continuous-phase
/// FSK generated in the modulator.
#[derive(Clone, Copy, Debug)]
pub enum Waveform {
    /// Gaussian-shaped FSK with this configuration, defined at its own
    /// `sample_rate` (12 kHz for every WSJT mode).
    Gfsk(GfskCfg),
    /// Plain continuous-phase FSK at `ModulationParams::TONE_SPACING_HZ`
    /// and `SYMBOL_DT`, with [`super::dsp::envelope`]'s ramp.
    Cpfsk,
}

/// A protocol [`synthesize`] can turn into audio. Implemented by the
/// WSJT-family modes; a protocol with a transmit chain that is not FSK
/// at all simply does not implement it.
pub trait FskWaveform: ModulationParams + FrameLayout {
    const WAVEFORM: Waveform;
}

/// The GFSK configuration at `sample_rate`: samples per symbol follow
/// `SYMBOL_DT` exactly as CPFSK's do, and the ramp keeps its fraction of
/// a symbol. At the configuration's own rate it is returned unchanged,
/// so 12 kHz output is exactly what the per-mode functions produced.
fn gfsk_at<P: FskWaveform>(cfg: &GfskCfg, sample_rate: u32) -> GfskCfg {
    if sample_rate as f32 == cfg.sample_rate {
        return *cfg;
    }
    let sps = cpfsk::nsps(sample_rate, P::SYMBOL_DT);
    GfskCfg {
        sample_rate: sample_rate as f32,
        samples_per_symbol: sps,
        ramp_samples: cfg.ramp_samples * sps / cfg.samples_per_symbol,
        ..*cfg
    }
}

/// Samples per symbol `P` transmits at `sample_rate`.
fn samples_per_symbol<P: FskWaveform>(sample_rate: u32) -> usize {
    match P::WAVEFORM {
        Waveform::Gfsk(cfg) => gfsk_at::<P>(&cfg, sample_rate).samples_per_symbol,
        Waveform::Cpfsk => cpfsk::nsps(sample_rate, P::SYMBOL_DT),
    }
}

/// Output length of [`synthesize`] / [`synthesize_into`] for one full
/// frame of `P` at `sample_rate` — what to size a buffer to before the
/// tones exist, which the `_into` forms and a C caller both need.
pub fn synth_len<P: FskWaveform>(sample_rate: u32) -> usize {
    P::N_SYMBOLS as usize * samples_per_symbol::<P>(sample_rate)
}

/// Synthesise one frame of `P`'s tones at `f0_hz` into `out`. No
/// allocation on the CPFSK path; the GFSK path allocates its phase
/// increments internally, as it always has.
///
/// One entry point where each mode used to have its own family
/// (`tones_to_f32` / `_into` / `_with_gfsk`, `synthesize_audio` /
/// `_into` / `_for`) — #391. The waveform is [`FskWaveform::WAVEFORM`].
///
/// # Panics
///
/// If `tones.len() != P::N_SYMBOLS`, a tone is `>= P::NTONES`, or
/// `out.len() != synth_len::<P>(sample_rate)`.
pub fn synthesize_into<P: FskWaveform>(
    out: &mut [f32],
    tones: &[u8],
    sample_rate: u32,
    f0_hz: f32,
    amplitude: f32,
) {
    check_tones::<P>(tones);
    match P::WAVEFORM {
        Waveform::Gfsk(cfg) => super::dsp::gfsk::synth_f32_into(
            out,
            tones,
            f0_hz,
            amplitude,
            &gfsk_at::<P>(&cfg, sample_rate),
        ),
        Waveform::Cpfsk => cpfsk::synth_f32_into(
            out,
            tones,
            cpfsk::nsps(sample_rate, P::SYMBOL_DT),
            f0_hz,
            P::TONE_SPACING_HZ,
            sample_rate,
            amplitude,
        ),
    }
}

/// Allocating form of [`synthesize_into`].
pub fn synthesize<P: FskWaveform>(
    tones: &[u8],
    sample_rate: u32,
    f0_hz: f32,
    amplitude: f32,
) -> Vec<f32> {
    let mut out = vec![0.0f32; synth_len::<P>(sample_rate)];
    synthesize_into::<P>(&mut out, tones, sample_rate, f0_hz, amplitude);
    out
}

/// 16-bit form of [`synthesize_into`]: the waveform at unit amplitude,
/// scaled so its peak is `amplitude_i16` and truncated toward zero —
/// the conversion the GFSK `i16` path has always used, now available
/// for the CPFSK modes too.
pub fn synthesize_i16_into<P: FskWaveform>(
    out: &mut [i16],
    tones: &[u8],
    sample_rate: u32,
    f0_hz: f32,
    amplitude_i16: i16,
) {
    let n = synth_len::<P>(sample_rate);
    assert_eq!(
        out.len(),
        n,
        "synthesize_i16_into: out.len() must equal synth_len"
    );
    let mut tmp = vec![0.0f32; n];
    synthesize_into::<P>(&mut tmp, tones, sample_rate, f0_hz, 1.0);
    let scale = amplitude_i16 as f32;
    for (dst, &src) in out.iter_mut().zip(tmp.iter()) {
        *dst = (src * scale) as i16;
    }
}

/// Allocating form of [`synthesize_i16_into`].
pub fn synthesize_i16<P: FskWaveform>(
    tones: &[u8],
    sample_rate: u32,
    f0_hz: f32,
    amplitude_i16: i16,
) -> Vec<i16> {
    let mut out = vec![0i16; synth_len::<P>(sample_rate)];
    synthesize_i16_into::<P>(&mut out, tones, sample_rate, f0_hz, amplitude_i16);
    out
}

fn check_tones<P: FskWaveform>(tones: &[u8]) {
    assert_eq!(
        tones.len(),
        P::N_SYMBOLS as usize,
        "synthesize: expected one tone per frame symbol"
    );
    assert!(
        tones.iter().all(|&t| u32::from(t) < P::NTONES),
        "synthesize: tone out of range 0..{}",
        P::NTONES
    );
}

#[cfg(test)]
mod waveform_tests {
    use super::*;

    /// A GFSK configuration must describe its own protocol: symbol length,
    /// shaping and modulation index are also trait constants, and the two
    /// must not drift apart. This is what catches an FST4 sub-mode wired
    /// to another sub-mode's `FST4_*_GFSK`.
    fn gfsk_matches_trait<P: FskWaveform>() {
        let Waveform::Gfsk(cfg) = P::WAVEFORM else {
            panic!("expected a GFSK protocol");
        };
        assert_eq!(cfg.sample_rate, 12_000.0);
        assert_eq!(cfg.samples_per_symbol, P::NSPS as usize);
        assert_eq!(cfg.bt, P::GFSK_BT);
        assert_eq!(cfg.hmod, P::GFSK_HMOD);
    }

    #[test]
    #[cfg(all(feature = "ft8", feature = "ft4", feature = "fst4"))]
    fn gfsk_configs_match_their_protocols() {
        gfsk_matches_trait::<crate::ft8::Ft8>();
        gfsk_matches_trait::<crate::ft4::Ft4>();
        gfsk_matches_trait::<crate::fst4::Fst4s15>();
        gfsk_matches_trait::<crate::fst4::Fst4s30>();
        gfsk_matches_trait::<crate::fst4::Fst4s60>();
        gfsk_matches_trait::<crate::fst4::Fst4s120>();
        gfsk_matches_trait::<crate::fst4::Fst4s300>();
    }

    /// GFSK away from 12 kHz: the symbol grid scales like CPFSK's, and
    /// every 4th sample at 48 kHz lands on the same instant as a 12 kHz
    /// sample, where the two waveforms must agree closely — same tone
    /// frequencies and phase track, the pulse only sampled finer.
    #[test]
    #[cfg(feature = "ft8")]
    fn gfsk_at_48k_tracks_the_12k_waveform() {
        use crate::ft8::Ft8;
        let m = crate::msg::wsjt77::pack77("CQ", "K1ABC", "FN42").unwrap();
        let tones = message_to_tones::<Ft8>(&m);
        let a = synthesize::<Ft8>(&tones, 12_000, 1500.0, 1.0);
        let b = synthesize::<Ft8>(&tones, 48_000, 1500.0, 1.0);
        assert_eq!(b.len(), 4 * a.len());
        let worst = a
            .iter()
            .zip(b.iter().step_by(4))
            .map(|(x, y)| (x - y).abs())
            .fold(0.0f32, f32::max);
        // Measured 0.0157 of full scale (FT8, this message); the bound
        // leaves 3x for float noise, far below a wrong tone or a phase
        // slip, either of which reaches ~2.0.
        assert!(worst < 0.05, "48 kHz diverges from 12 kHz by {worst}");
    }
}
