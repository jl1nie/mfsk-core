//! FST4W beacon protocol: FST4 waveform geometry with the first 50 bits
//! of WSJT77 subtype 0.6 protected by CRC-24 and LDPC(240,74).
//!
//! The 50-bit message is intentionally *not* the classic WSPR payload.
//! This module mirrors pinned WSJT-X
//! `lib/77bit/packjt77.f90::pack77_06` and the `iwspr=1` path in
//! `lib/fst4/genfst4.f90`.

use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;

use crate::core::{
    DecodeContext, FecCodec, FrameLayout, MessageCodec, MessageFields, ModulationParams, Protocol,
    ProtocolId, SyncBlock, SyncMode,
};
use crate::fec::{Ldpc240_74, ldpc240_74};
use crate::msg::hash_table::{CallsignHashTable, ihashcall};
use crate::msg::wsjt77;

const SYNC_A: [u8; 8] = [0, 1, 3, 2, 1, 0, 2, 3];
const SYNC_B: [u8; 8] = [2, 3, 1, 0, 3, 2, 0, 1];
const SYNC_BLOCKS: [SyncBlock; 5] = [
    SyncBlock {
        start_symbol: 0,
        pattern: &SYNC_A,
    },
    SyncBlock {
        start_symbol: 38,
        pattern: &SYNC_B,
    },
    SyncBlock {
        start_symbol: 76,
        pattern: &SYNC_A,
    },
    SyncBlock {
        start_symbol: 114,
        pattern: &SYNC_B,
    },
    SyncBlock {
        start_symbol: 152,
        pattern: &SYNC_A,
    },
];

#[derive(Copy, Clone, Debug, Default)]
pub struct Fst4w50Message;

impl MessageCodec for Fst4w50Message {
    type Unpacked = String;
    const PAYLOAD_BITS: u32 = 50;
    const CRC_BITS: u32 = 24;

    fn pack(&self, fields: &MessageFields) -> Option<Vec<u8>> {
        if let Some(text) = fields.free_text.as_deref() {
            return pack_message(text).map(|bits| bits.to_vec());
        }

        let call = fields.call1.as_deref()?;
        let message = if let Some(grid) = fields.grid.as_deref() {
            if call.starts_with('<') {
                format!("{call} {grid}")
            } else {
                format!("{call} {grid} {}", fields.report?)
            }
        } else {
            format!("{call} {}", fields.report?)
        };
        pack_message(&message).map(|bits| bits.to_vec())
    }

    fn unpack(&self, payload: &[u8], context: &DecodeContext) -> Option<Self::Unpacked> {
        if payload.len() != 50 {
            return None;
        }
        let mut message77 = [0u8; 77];
        message77[..50].copy_from_slice(payload);
        write_bits77(&mut message77, 71, 3, 6);
        if let Some(any) = context.callsign_hash_table.as_ref()
            && let Some(table) = any.downcast_ref::<CallsignHashTable>()
        {
            return wsjt77::unpack77_with_hash(&message77, table);
        }
        wsjt77::unpack77(&message77)
    }

    fn verify_info(info: &[u8]) -> bool {
        ldpc240_74::check_crc24(info)
    }
}

macro_rules! fst4w_submode {
    (
        $(#[$attr:meta])*
        $name:ident,
        nsps = $nsps:literal,
        ndown = $ndown:literal,
        tr_period_s = $period:literal,
    ) => {
        $(#[$attr])*
        #[derive(Copy, Clone, Debug, Default)]
        pub struct $name;

        impl ModulationParams for $name {
            const NTONES: u32 = 4;
            const BITS_PER_SYMBOL: u32 = 2;
            const NSPS: u32 = $nsps;
            const SYMBOL_DT: f32 = $nsps as f32 / 12_000.0;
            const TONE_SPACING_HZ: f32 = 12_000.0 / $nsps as f32;
            const GRAY_MAP: &'static [u8] = &[0, 1, 3, 2];
            const GFSK_BT: f32 = 2.0;
            const GFSK_HMOD: f32 = 1.0;
            const NFFT_PER_SYMBOL_FACTOR: u32 = 2;
            const NSTEP_PER_SYMBOL: u32 = 2;
            const NDOWN: u32 = $ndown;
            const LLR_NSYM_MAX: u32 = 8;
            const LLR_NSYM_MID: Option<u32> = Some(4);
        }

        impl FrameLayout for $name {
            const N_DATA: u32 = 120;
            const N_SYNC: u32 = 40;
            const N_SYMBOLS: u32 = 160;
            const N_RAMP: u32 = 0;
            const SYNC_MODE: SyncMode = SyncMode::Block(&SYNC_BLOCKS);
            const T_SLOT_S: f32 = $period as f32;
            const TX_START_OFFSET_S: f32 = 1.0;
        }

        impl Protocol for $name {
            type Fec = Ldpc240_74;
            type Msg = Fst4w50Message;
            const ID: ProtocolId = ProtocolId::Fst4w;
        }
    };
}

fst4w_submode! {
    /// FST4W-120 (2 minute period).
    Fst4w120,
    nsps = 8_200,
    ndown = 205,
    tr_period_s = 120,
}
fst4w_submode! {
    /// FST4W-300 (5 minute period).
    Fst4w300,
    nsps = 21_504,
    ndown = 512,
    tr_period_s = 300,
}
fst4w_submode! {
    /// FST4W-900 (15 minute period).
    Fst4w900,
    nsps = 66_560,
    ndown = 1_664,
    tr_period_s = 900,
}
fst4w_submode! {
    /// FST4W-1800 (30 minute period).
    Fst4w1800,
    nsps = 134_400,
    ndown = 3_360,
    tr_period_s = 1800,
}

const NTOKENS: u32 = 2_063_592;
const BASE36: &[u8; 36] = b"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

fn write_bits50(bits: &mut [u8; 50], start: usize, len: usize, value: u32) {
    for index in 0..len {
        bits[start + index] = ((value >> (len - 1 - index)) & 1) as u8;
    }
}

fn write_bits77(bits: &mut [u8; 77], start: usize, len: usize, value: u32) {
    for index in 0..len {
        bits[start + index] = ((value >> (len - 1 - index)) & 1) as u8;
    }
}

fn parse_power(field: &str) -> Option<u32> {
    if field.is_empty() || field.len() > 2 || !field.bytes().all(|byte| byte.is_ascii_digit()) {
        return None;
    }
    Some(field.parse::<u32>().ok()?.min(60))
}

fn encoded_power(power_dbm: u32) -> u32 {
    // Exact for non-negative integer dBm: WSJT-X uses `nint(0.3*idbm)`.
    (power_dbm * 3 + 5) / 10
}

fn base36_digit(byte: u8) -> Option<u32> {
    BASE36
        .iter()
        .position(|candidate| *candidate == byte)
        .map(|index| index as u32)
}

/// Match `chkcall.f90` for the base call selected from a Type-2 compound
/// callsign. This is intentionally stricter than the general WSJT77 parser.
fn is_wspr_base_call(call: &str) -> bool {
    let bytes = call.as_bytes();
    if bytes.is_empty() || bytes.len() > 6 {
        return false;
    }
    if !bytes.first().is_some_and(|byte| byte.is_ascii_uppercase())
        && !bytes.get(1).is_some_and(|byte| byte.is_ascii_uppercase())
    {
        return false;
    }
    if bytes[0] == b'Q' && call != "QU1RK" {
        return false;
    }

    let area = if bytes.get(2).is_some_and(|byte| byte.is_ascii_digit()) {
        2
    } else if bytes.get(1).is_some_and(|byte| byte.is_ascii_digit()) {
        1
    } else {
        return false;
    };
    let suffix = &bytes[area + 1..];
    (1..=3).contains(&suffix.len()) && suffix.iter().all(|byte| byte.is_ascii_uppercase())
}

fn pack_type1(fields: &[&str]) -> Option<[u8; 50]> {
    let [call, grid, power] = fields else {
        return None;
    };
    if !(3..=6).contains(&call.len()) {
        return None;
    }
    let power = parse_power(power)?;
    let n28 = wsjt77::pack28(call).unwrap_or_else(|| NTOKENS + ihashcall(call, 22));
    let grid = wsjt77::pack_grid4(grid)?;

    let mut bits = [0u8; 50];
    write_bits50(&mut bits, 0, 28, n28);
    write_bits50(&mut bits, 28, 15, grid);
    write_bits50(&mut bits, 43, 5, encoded_power(power));
    // Bits 49 and 50 (Fortran numbering) stay zero: Type 1 is x00.
    Some(bits)
}

fn pack_type2(fields: &[&str]) -> Option<[u8; 50]> {
    let [compound, power] = fields else {
        return None;
    };
    if !(5..=10).contains(&compound.len()) {
        return None;
    }
    let power = parse_power(power)?;
    let slash = compound.find('/')?;
    if slash == 0 || slash + 1 == compound.len() || compound[slash + 1..].contains('/') {
        return None;
    }
    let prefix_or_call = &compound[..slash];
    let suffix_or_call = &compound[slash + 1..];
    let base = if prefix_or_call.len() > suffix_or_call.len() {
        prefix_or_call
    } else {
        suffix_or_call
    };
    if !is_wspr_base_call(base) {
        return None;
    }
    let n28 = wsjt77::pack28(base)?;

    // Fortran `i1=index(call,'/')` is one-based. i1 <= 4 means a
    // one-to-three-character prefix; otherwise the right side is a suffix.
    let npfx = if slash < 4 {
        if prefix_or_call.len() > 3 {
            return None;
        }
        let mut value = 0u32;
        for byte in prefix_or_call.bytes() {
            value = value.checked_mul(36)?.checked_add(base36_digit(byte)?)?;
        }
        value
    } else {
        let suffix = suffix_or_call.as_bytes();
        let value = match suffix {
            [a] => base36_digit(*a)?,
            [a, b] => 36 * base36_digit(*a)? + base36_digit(*b)?,
            [a, b, digit] if digit.is_ascii_digit() => {
                360 * base36_digit(*a)? + 10 * base36_digit(*b)? + base36_digit(*digit)?
            }
            _ => return None,
        };
        46_656 + value
    };

    let mut bits = [0u8; 50];
    write_bits50(&mut bits, 0, 28, n28);
    write_bits50(&mut bits, 28, 16, npfx);
    write_bits50(&mut bits, 44, 5, encoded_power(power));
    bits[49] = 1;
    Some(bits)
}

fn pack_type3(fields: &[&str]) -> Option<[u8; 50]> {
    let [bracketed_call, grid] = fields else {
        return None;
    };
    if !(5..=12).contains(&bracketed_call.len()) {
        return None;
    }
    let call = bracketed_call.strip_prefix('<')?.strip_suffix('>')?;
    if call.is_empty()
        || call.len() > 10
        || !call
            .bytes()
            .all(|byte| byte.is_ascii_uppercase() || byte.is_ascii_digit() || byte == b'/')
    {
        return None;
    }

    let grid = grid.as_bytes();
    if grid.len() != 4 && grid.len() != 6 {
        return None;
    }
    let j1 = grid[0].checked_sub(b'A')? as u32;
    let j2 = grid[1].checked_sub(b'A')? as u32;
    let j3 = grid[2].checked_sub(b'0')? as u32;
    let j4 = grid[3].checked_sub(b'0')? as u32;
    if j1 > 17 || j2 > 17 || j3 > 9 || j4 > 9 {
        return None;
    }
    let (j5, j6) = if grid.len() == 4 {
        (24, 24)
    } else {
        let j5 = grid[4].checked_sub(b'A')? as u32;
        let j6 = grid[5].checked_sub(b'A')? as u32;
        if j5 > 23 || j6 > 23 {
            return None;
        }
        (j5, j6)
    };
    let grid_index = ((((j1 * 18 + j2) * 10 + j3) * 10 + j4) * 25 + j5) * 25 + j6;

    let mut bits = [0u8; 50];
    write_bits50(&mut bits, 0, 22, ihashcall(call, 22));
    write_bits50(&mut bits, 22, 25, grid_index);
    write_bits50(&mut bits, 47, 3, 2); // Type 3 marker `010`.
    Some(bits)
}

/// Pack one pinned-WSJT-X FST4W message into its 50 on-air message bits.
///
/// Accepted forms are:
///
/// - Type 1: `CALL GRID4 DBM`
/// - Type 2: `PREFIX/CALL DBM` or `CALL/SUFFIX DBM`
/// - Type 3: `<CALL> GRID4` or `<CALL> GRID6`
pub fn pack_message(message: &str) -> Option<[u8; 50]> {
    let normalized = message
        .split_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
        .to_ascii_uppercase();
    let fields: Vec<&str> = normalized.split_whitespace().collect();
    match fields.len() {
        3 => pack_type1(&fields),
        2 if fields[0].starts_with('<') => pack_type3(&fields),
        2 => pack_type2(&fields),
        _ => None,
    }
}

fn append_crc24(message50: &[u8; 50]) -> [u8; 74] {
    let mut info = [0u8; 74];
    info[..50].copy_from_slice(message50);
    let crc = ldpc240_74::crc24(&info);
    for index in 0..24 {
        info[50 + index] = ((crc >> (23 - index)) & 1) as u8;
    }
    info
}

pub fn message_to_tones(message50: &[u8; 50]) -> Vec<u8> {
    let info = append_crc24(message50);
    let codec = Ldpc240_74;
    let mut codeword = [0u8; 240];
    codec.encode(&info, &mut codeword);
    crate::core::tx::codeword_to_itone::<Fst4w120>(&codeword)
}

pub fn message_to_tones_from_text(message: &str) -> Option<Vec<u8>> {
    Some(message_to_tones(&pack_message(message)?))
}

pub fn type1_to_tones(callsign: &str, grid: &str, power_dbm: i32) -> Option<Vec<u8>> {
    message_to_tones_from_text(&format!("{callsign} {grid} {power_dbm}"))
}

pub use crate::fst4::decode::{
    FST4_120_DOWNSAMPLE as FST4W_120_DOWNSAMPLE, FST4_300_DOWNSAMPLE as FST4W_300_DOWNSAMPLE,
    FST4_900_DOWNSAMPLE as FST4W_900_DOWNSAMPLE, FST4_1800_DOWNSAMPLE as FST4W_1800_DOWNSAMPLE,
    decode_frame_for, decode_frame_with_cache_and_options_for, decode_frame_with_cache_for,
    decode_frame_with_options_for,
};
pub use crate::fst4::encode::{
    FST4_120_GFSK as FST4W_120_GFSK, FST4_300_GFSK as FST4W_300_GFSK,
    FST4_900_GFSK as FST4W_900_GFSK, FST4_1800_GFSK as FST4W_1800_GFSK, tones_to_f32_with_gfsk,
    tones_to_i16_with_gfsk,
};

#[cfg(test)]
mod tests {
    use super::*;

    fn bit_string(bits: &[u8]) -> String {
        bits.iter()
            .map(|bit| if *bit == 0 { '0' } else { '1' })
            .collect()
    }

    fn tone_string(tones: &[u8]) -> String {
        tones.iter().map(|tone| (b'0' + *tone) as char).collect()
    }

    #[test]
    fn type1_encode_decode_roundtrip() {
        let bits = pack_message("K1ABC FN42 37").expect("pack FST4W payload");
        let info = append_crc24(&bits);
        assert!(ldpc240_74::check_crc24(&info));

        let codec = Ldpc240_74;
        let mut codeword = [0u8; 240];
        codec.encode(&info, &mut codeword);
        let llr: Vec<f32> = codeword
            .iter()
            .map(|bit| if *bit == 1 { 8.0 } else { -8.0 })
            .collect();
        let decoded = codec
            .decode_soft(
                &llr,
                &crate::core::FecOpts {
                    verify_info: Some(Fst4w50Message::verify_info),
                    ..crate::core::FecOpts::default()
                },
            )
            .expect("decode");
        assert_eq!(&decoded.info[..50], &bits);
        assert_eq!(
            Fst4w50Message.unpack(&decoded.info[..50], &DecodeContext::default()),
            Some("K1ABC FN42 37".into())
        );
    }

    #[test]
    fn wsjtx_v3_0_2_subtype_0_6_vectors_match_exactly() {
        // Generated by tools/wsjtx_fst4w50_oracle.f90 against pinned
        // WSJT-X commit ccdfaf3c1c109010d15399674ce278167cfde848.
        let vectors = [
            (
                "K1ABC FN42 37",
                "00001001101111011110001101010101000011001100101100",
                "K1ABC FN42 37",
            ),
            (
                "PJ4/K1ABC 37",
                "00001001101111011110001101011000000101000000010111",
                "PJ4/K1ABC 37",
            ),
            (
                "K1ABC/7 37",
                "00001001101111011110001101011011011001000111010111",
                "K1ABC/7 37",
            ),
            (
                "K1ABC/AB 37",
                "00001001101111011110001101011011011110110011010111",
                "K1ABC/AB 37",
            ),
            (
                "K1ABC/AB7 37",
                "00001001101111011110001101011100010011000101010111",
                "K1ABC/AB7 37",
            ),
            (
                "<K1ABC> FN42LX",
                "10110010001111010010110011000101010001000110000010",
                "<...> FN42LX",
            ),
            (
                "<K1ABC> FN42",
                "10110010001111010010110011000101010001101110110010",
                "<...> FN42",
            ),
        ];

        for (message, expected_bits, expected_without_hash) in vectors {
            let bits = pack_message(message).unwrap_or_else(|| panic!("pack {message}"));
            assert_eq!(bit_string(&bits), expected_bits, "{message}");
            assert_eq!(
                Fst4w50Message.unpack(&bits, &DecodeContext::default()),
                Some(expected_without_hash.into()),
                "{message}"
            );
        }
    }

    #[test]
    fn wsjtx_v3_0_2_complete_tone_frames_match_exactly() {
        // Generated by tools/wsjtx_fst4w_tone_oracle.f90 against the
        // pinned source. These cover all three subtype-0.6 message layouts,
        // CRC-24, LDPC(240,74), Gray mapping, and sync insertion.
        let vectors = [
            (
                "K1ABC FN42 37",
                "0132102300313221230211110020203201302123103201112313232323030132223312132210013210230003121120221323203311231211022310320110330000022101001223313221230101321023",
            ),
            (
                "PJ4/K1ABC 37",
                "0132102300313221230211300110001123032023103201133221030033210020321213011133013210231111212123321013130200102021232310320130130012001111232113131000012301321023",
            ),
            (
                "<K1ABC> FN42LX",
                "0132102332030221032020111010130033200223103201112331131121012110003213302300013210230101200220013032212103020102002310320120220213322010323330013212321201321023",
            ),
        ];

        for (message, expected_tones) in vectors {
            let tones =
                message_to_tones_from_text(message).unwrap_or_else(|| panic!("pack {message}"));
            assert_eq!(tone_string(&tones), expected_tones, "{message}");
        }
    }

    #[test]
    fn type3_hash_resolves_through_decode_context() {
        let bits = pack_message("<K1ABC> FN42LX").expect("pack Type 3");
        let mut table = CallsignHashTable::new();
        table.insert("K1ABC");
        let context = DecodeContext {
            callsign_hash_table: Some(alloc::sync::Arc::new(table)),
        };
        assert_eq!(
            Fst4w50Message.unpack(&bits, &context),
            Some("<K1ABC> FN42LX".into())
        );
    }

    #[test]
    fn all_periods_share_the_upstream_geometry() {
        fn check<P: Protocol>(period: f32, nsps: u32, ndown: u32) {
            assert_eq!(P::T_SLOT_S, period);
            assert_eq!(P::NSPS, nsps);
            assert_eq!(P::NDOWN, ndown);
            assert_eq!(P::Fec::N, 240);
            assert_eq!(P::Fec::K, 74);
        }
        check::<Fst4w120>(120.0, 8_200, 205);
        check::<Fst4w300>(300.0, 21_504, 512);
        check::<Fst4w900>(900.0, 66_560, 1_664);
        check::<Fst4w1800>(1800.0, 134_400, 3_360);
    }

    #[test]
    fn tone_frame_matches_fst4_layout() {
        let tones = type1_to_tones("K1ABC", "FN42", 37).expect("encode");
        assert_eq!(tones.len(), 160);
        assert_eq!(&tones[..8], &SYNC_A);
        assert_eq!(&tones[38..46], &SYNC_B);
    }

    #[test]
    fn fst4w_120_audio_roundtrip() {
        if std::env::var("RUN_FST4_ROUNDTRIP").is_err() {
            eprintln!("skipping FST4W-120 audio roundtrip; set RUN_FST4_ROUNDTRIP=1");
            return;
        }
        let tones = type1_to_tones("K1ABC", "FN42", 37).expect("encode");
        let audio = tones_to_i16_with_gfsk(&tones, 1_500.0, 10_000, &FST4W_120_GFSK);
        let mut slot = vec![0i16; 120 * 12_000];
        slot[12_000..12_000 + audio.len()].copy_from_slice(&audio);
        let decodes =
            decode_frame_for::<Fst4w120>(&slot, &FST4W_120_DOWNSAMPLE, 1_000.0, 2_000.0, 0.8, 20);
        assert!(
            decodes.iter().any(|decode| {
                Fst4w50Message.unpack(&decode.info[..50], &DecodeContext::default())
                    == Some("K1ABC FN42 37".into())
            }),
            "clean FST4W-120 waveform did not decode"
        );
    }
}
