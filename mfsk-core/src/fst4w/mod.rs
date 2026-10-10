//! # `fst4w` — FST4W, the WSPR-style beacon on FST4's modulation
//!
//! WSJT-X has no separate FST4W decoder: `lib/fst4_decode.f90` runs with
//! `iwspr=1`. The modulation is FST4's (4-GFSK, BT 2.0, 160 symbols with the same
//! five sync blocks, the same Gray map), and the front end — candidate search,
//! downsampler, bit metrics — is shared. What differs, from
//! `genfst4.f90` / `fst4_decode.f90` at `v3.3.0-beta1`:
//!
//! | | FST4 | FST4W |
//! |---|---|---|
//! | periods | 15‥300 s here | **120, 300, 900, 1800 s** |
//! | FEC | LDPC(240,101) + CRC-24 | **LDPC(240,74)** + CRC-24 over 74 bits |
//! | scrambler | `rvec` over the 77 bits | none |
//! | message | any pack77 type | **pack77 type 0.6**: the first 50 of 77 bits |
//!
//! The four ZSTs are [`Fst4w120`], [`Fst4w300`], [`Fst4w900`] and [`Fst4w1800`];
//! their message codec is [`Fst4wMessage`], their FEC
//! [`crate::fec::Ldpc240_74`]. Transmit is in [`encode`]; the receive side
//! (a decoder ladder of its own — Keff 66 then Keff 50 per LLR variant, with a
//! known-call list) follows it.
//!
//! Embedded is out of scope: a 1800 s slot needs a 21.6 M-point FFT.

use alloc::string::String;
use alloc::vec::Vec;

use crate::engine::{
    DecodeContext, FrameLayout, MessageCodec, MessageFields, ModulationParams, Protocol,
    ProtocolId, SyncMode,
};
use crate::fec::Ldpc240_74;
use crate::fec::ldpc240_74::{PAYLOAD_BITS, check_crc24_74, crc24};
use crate::fst4::FST4_SYNC_BLOCKS;
use crate::fst4::encode as fst4_encode;
use crate::msg::hash_table::CallsignHashTable;
use crate::msg::wsjt77::{self, Wsjt77Fields};

pub mod encode;

/// The 27 bits `fst4_decode.f90:744-746` appends to a 50-bit payload before
/// `unpack77`: 21 zero bits, `n3 = 6` (`110`), `i3 = 0` (`000`).
pub const PAYLOAD_SUFFIX: [u8; 27] = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0,
];

/// FST4W's message codec: a 50-bit WSPR-type payload (`i3 = 0, n3 = 6`) with the
/// 24-bit CRC over the 74-bit word.
#[derive(Copy, Clone, Debug, Default)]
pub struct Fst4wMessage;

impl Fst4wMessage {
    /// The 50 payload bits for `text`, or `None` where `genfst4` says
    /// `*** bad message ***` ([`wsjt77::pack77_wspr`] with the 50-bit preference).
    ///
    /// Leading blanks are stripped properly. `genfst4.f90:40-43` does not: its
    /// `message=message(i+1:)` drops `i+1` characters at pass `i`, so with two or
    /// more leading blanks it eats the start of the message — a divergence from
    /// WSJT-X on purpose, pinned by `tests/fst4w_encode.rs`.
    pub fn pack_text(text: &str) -> Option<[u8; PAYLOAD_BITS]> {
        let bits77 = wsjt77::pack77_wspr(text, true)?;
        let mut p = [0u8; PAYLOAD_BITS];
        p.copy_from_slice(&bits77[..PAYLOAD_BITS]);
        Some(p)
    }

    /// The 77 bits `unpack77` is given for a 50-bit payload.
    pub fn payload_to_77(payload: &[u8]) -> Option<[u8; 77]> {
        if payload.len() != PAYLOAD_BITS {
            return None;
        }
        let mut b = [0u8; 77];
        b[..PAYLOAD_BITS].copy_from_slice(payload);
        b[PAYLOAD_BITS..].copy_from_slice(&PAYLOAD_SUFFIX);
        Some(b)
    }
}

impl MessageCodec for Fst4wMessage {
    type Unpacked = Wsjt77Fields;
    const PAYLOAD_BITS: u32 = PAYLOAD_BITS as u32;
    const CRC_BITS: u32 = 24;

    /// `fields.free_text` is the whole message (`CALL GRID4 DBM`, `PFX/CALL DBM`,
    /// `<CALL> GRID6`); without it, `call1`, `grid` and `report` (dBm) build the
    /// first form.
    fn pack(&self, fields: &MessageFields) -> Option<Vec<u8>> {
        let text: String = match &fields.free_text {
            Some(t) => t.clone(),
            None => alloc::format!(
                "{} {} {}",
                fields.call1.as_deref()?,
                fields.grid.as_deref()?,
                fields.report?
            ),
        };
        Self::pack_text(&text).map(|p| p.to_vec())
    }

    fn unpack(&self, payload: &[u8], ctx: &DecodeContext) -> Option<Self::Unpacked> {
        let buf = Self::payload_to_77(payload)?;
        if let Some(any) = ctx.callsign_hash_table.as_ref()
            && let Some(ht) = any.downcast_ref::<CallsignHashTable>()
        {
            return wsjt77::unpack77_fields(&buf, ht);
        }
        wsjt77::unpack77_fields(&buf, &CallsignHashTable::new())
    }

    /// `get_crc24(m74, 74) == 0` (`decode240_74.f90:88`).
    fn verify_info(info: &[u8]) -> bool {
        check_crc24_74(info)
    }

    /// The CRC-24 over the 74-bit word with its CRC slot zero, MSB first
    /// (`genfst4.f90:69-71`).
    fn append_crc(info: &mut [u8]) -> bool {
        if info.len() != 74 {
            return false;
        }
        for b in info[PAYLOAD_BITS..].iter_mut() {
            *b = 0;
        }
        let crc = crc24(info);
        for i in 0..24 {
            info[PAYLOAD_BITS + i] = ((crc >> (23 - i)) & 1) as u8;
        }
        true
    }
}

/// Define an FST4W period ZST. `fst4_decode.f90:395-430` has no `iwspr` branch
/// in its per-period constants, so these are FST4's, shared rather than
/// copied: `nsps`, `ndown`, the GFSK configuration and the front-end FFT size
/// come from the FST4 sub-mode of the same period where there is one.
macro_rules! fst4w_period {
    (
        $(#[$attr:meta])*
        $name:ident,
        nsps = $nsps:literal,
        ndown = $ndown:literal,
        tr_period_s = $period:literal,
        snr_calfac = $snr_calfac:literal,
        decode_fft1_size = $fft1:literal,
        gfsk = $gfsk:path,
    ) => {
        $(#[$attr])*
        #[derive(Copy, Clone, Debug, Default)]
        pub struct $name;

        #[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
        impl $name {
            /// `fst4_decode.f90`'s `snr_calfac` for this period. Read by the
            /// receiver (`fst4w::decode`), which is the next change.
            #[allow(dead_code)]
            pub(crate) const SNR_CALFAC: f32 = $snr_calfac;
        }

        impl ModulationParams for $name {
            const NTONES: u32 = 4;
            const BITS_PER_SYMBOL: u32 = 2;
            const NSPS: u32 = $nsps;
            const TONE_SPACING_HZ: f32 =
                crate::engine::tone_spacing_hz(Self::GFSK_HMOD, Self::NSPS);
            const GRAY_MAP: &'static [u8] = &[0, 1, 3, 2];
            const GFSK_BT: f32 = 2.0;
            const GFSK_HMOD: f32 = 1.0;
            /// `genfst4.f90:66-71`: the 50 bits go to the CRC and the encoder
            /// as they are.
            const INFO_SCRAMBLE_RVEC: Option<&'static [u8]> = None;
            const LLR_NSYM_MAX: u32 = 8;
            const LLR_NSYM_MID: Option<u32> = Some(4);
        }

        impl crate::engine::SyncFrontEnd for $name {
            const NFFT_PER_SYMBOL_FACTOR: u32 = 2;
            const NSTEP_PER_SYMBOL: u32 = 2;
            const NDOWN: u32 = $ndown;
        }

        impl crate::engine::tx::FskWaveform for $name {
            const WAVEFORM: crate::engine::tx::Waveform = crate::engine::tx::Waveform::Gfsk($gfsk);
        }

        impl FrameLayout for $name {
            const N_DATA: u32 = 120;
            const N_SYNC: u32 = 40;
            const N_RAMP: u32 = 0;
            const SYNC_MODE: SyncMode = SyncMode::Block(&FST4_SYNC_BLOCKS);
            const T_SLOT_S: f32 = $period as f32;
            /// `fst4sim.f90` sends FST4W 1 s after the period start, as FST4.
            const TX_START_OFFSET_S: f32 = 1.0;
        }

        impl Protocol for $name {
            type Fec = Ldpc240_74;
            type Msg = Fst4wMessage;
            type SyncPhasors = ();
            const ID: ProtocolId = ProtocolId::Fst4w;
            const DECODE_FFT1_SIZE: u32 = $fft1;
        }
    };
}

fst4w_period! {
    /// FST4W-120: 120 s period, 1.4634 Hz tone spacing. `nsps=8200`, `ndown=205`
    /// (`fst4_decode.f90:407-410`), FST4-120's front end.
    Fst4w120,
    nsps = 8_200,
    ndown = 205,
    tr_period_s = 120,
    snr_calfac = 390.0,
    decode_fft1_size = 1_443_200,
    gfsk = fst4_encode::FST4_120_GFSK,
}

fst4w_period! {
    /// FST4W-300: 300 s period, 0.5580 Hz tone spacing. `nsps=21504`,
    /// `ndown=512` (`fst4_decode.f90:411-414`), FST4-300's front end.
    Fst4w300,
    nsps = 21_504,
    ndown = 512,
    tr_period_s = 300,
    snr_calfac = 340.0,
    decode_fft1_size = 4_194_304,
    gfsk = fst4_encode::FST4_300_GFSK,
}

fst4w_period! {
    /// FST4W-900: 900 s period, 0.1803 Hz tone spacing. `nsps=66560`,
    /// `ndown=1664`, `nfft1=6480·1664` (`fst4_decode.f90:415-418`).
    Fst4w900,
    nsps = 66_560,
    ndown = 1_664,
    tr_period_s = 900,
    snr_calfac = 320.0,
    decode_fft1_size = 10_782_720,
    gfsk = fst4_encode::FST4_900_GFSK,
}

fst4w_period! {
    /// FST4W-1800: 1800 s period, 0.0893 Hz tone spacing. `nsps=134400`,
    /// `ndown=3360`, `nfft1=6426·3360` (`fst4_decode.f90:419-422`).
    Fst4w1800,
    nsps = 134_400,
    ndown = 3_360,
    tr_period_s = 1800,
    snr_calfac = 320.0,
    decode_fft1_size = 21_591_360,
    gfsk = fst4_encode::FST4_1800_GFSK,
}
