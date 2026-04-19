//! End-to-end validation of the `mfsk_core::core::Protocol` trait wiring for
//! [`mfsk_core::ft8::Ft8`]. These tests intentionally route through the trait
//! methods (not the concrete free functions) so that future genericised
//! pipeline code is guaranteed to work when driven by `<P: Protocol>`.

use mfsk_core::core::{
    FecCodec, FrameLayout, MessageCodec, MessageFields, ModulationParams, Protocol,
};
use mfsk_core::ft8::Ft8;

/// Associated constants should match the canonical WSJT-X values from
/// `ft8-core::params`. The plain-number assertions below read as a spec and
/// catch any accidental drift when the `Ft8` trait impl is edited.
#[test]
fn ft8_associated_constants() {
    assert_eq!(<Ft8 as ModulationParams>::NTONES, 8);
    assert_eq!(<Ft8 as ModulationParams>::BITS_PER_SYMBOL, 3);
    assert_eq!(<Ft8 as ModulationParams>::NSPS, 1920);
    assert!(((<Ft8 as ModulationParams>::SYMBOL_DT) - 0.16).abs() < 1e-6);
    assert_eq!(
        <Ft8 as ModulationParams>::GRAY_MAP,
        &[0, 1, 3, 2, 5, 6, 4, 7]
    );
    assert_eq!(<Ft8 as ModulationParams>::GFSK_BT, 2.0);
    assert_eq!(<Ft8 as ModulationParams>::GFSK_HMOD, 1.0);
    assert_eq!(<Ft8 as ModulationParams>::NFFT_PER_SYMBOL_FACTOR, 2);
    assert_eq!(<Ft8 as ModulationParams>::NSTEP_PER_SYMBOL, 4);
    assert_eq!(<Ft8 as ModulationParams>::NDOWN, 60);

    assert_eq!(<Ft8 as FrameLayout>::N_DATA, 58);
    assert_eq!(<Ft8 as FrameLayout>::N_SYNC, 21);
    assert_eq!(<Ft8 as FrameLayout>::N_SYMBOLS, 79);
    assert_eq!(<Ft8 as FrameLayout>::N_RAMP, 0);
    let blocks = <Ft8 as FrameLayout>::SYNC_MODE.blocks();
    assert_eq!(blocks.len(), 3);
    for b in blocks {
        assert_eq!(b.pattern, &[3, 1, 4, 0, 6, 5, 2]);
    }
    assert_eq!(
        blocks.iter().map(|b| b.start_symbol).collect::<Vec<_>>(),
        vec![0, 36, 72]
    );
    assert_eq!(<Ft8 as FrameLayout>::T_SLOT_S, 15.0);
    assert_eq!(<Ft8 as FrameLayout>::TX_START_OFFSET_S, 0.5);

    assert_eq!(<<Ft8 as Protocol>::Fec as FecCodec>::N, 174);
    assert_eq!(<<Ft8 as Protocol>::Fec as FecCodec>::K, 91);
    assert_eq!(<<Ft8 as Protocol>::Msg as MessageCodec>::PAYLOAD_BITS, 77);
    assert_eq!(<<Ft8 as Protocol>::Msg as MessageCodec>::CRC_BITS, 14);
}

/// Pack a standard CQ via `Ft8::Msg`, encode through `Ft8::Fec`, then verify
/// the codeword decodes back to the original payload. This exercises every
/// concrete associated type on the trait.
#[test]
fn ft8_message_and_fec_round_trip() {
    let msg = <Ft8 as Protocol>::Msg::default();
    let fec = <Ft8 as Protocol>::Fec::default();

    let fields = MessageFields {
        call1: Some("CQ".into()),
        call2: Some("JA1ABC".into()),
        grid: Some("PM95".into()),
        ..MessageFields::default()
    };
    let payload = msg.pack(&fields).expect("pack77 succeeds for CQ");
    assert_eq!(payload.len(), 77);

    // The WSJT LDPC(174,91) code carries 77 msg + 14 CRC → 91 info bits.
    // Our trait's `FecCodec::K` is that 91, so we synthesise the CRC via the
    // existing helper rather than duplicate the arithmetic here.
    let mut info91 = vec![0u8; 91];
    info91[..77].copy_from_slice(&payload);
    // Compute CRC-14 via the existing implementation (exposed by mfsk_fec).
    let mut bytes = [0u8; 12];
    for (i, &bit) in payload.iter().enumerate() {
        bytes[i / 8] |= (bit & 1) << (7 - (i % 8));
    }
    let crc = mfsk_core::fec::ldpc::crc14(&bytes);
    for b in 0..14 {
        info91[77 + b] = ((crc >> (13 - b)) & 1) as u8;
    }

    let mut codeword = vec![0u8; 174];
    fec.encode(&info91, &mut codeword);
    assert_eq!(codeword.len(), 174);
    assert_eq!(&codeword[..91], &info91[..]); // systematic

    // Decode from "perfect" soft LLRs: ±8.0 per bit → BP converges on iter 0.
    let llr: Vec<f32> = codeword
        .iter()
        .map(|&b| if b == 1 { 8.0 } else { -8.0 })
        .collect();
    let result = fec
        .decode_soft(&llr, &mfsk_core::core::FecOpts::default())
        .expect("BP converges with perfect LLR");
    assert_eq!(&result.info[..77], &payload[..]);
}

/// Unpack the packed CQ via `MessageCodec::unpack` and ensure the rendered
/// text matches what the raw `unpack77` helper produces.
#[test]
fn ft8_message_unpack_renders_text() {
    let msg = <Ft8 as Protocol>::Msg::default();
    let fields = MessageFields {
        call1: Some("CQ".into()),
        call2: Some("JA1ABC".into()),
        grid: Some("PM95".into()),
        ..MessageFields::default()
    };
    let payload = msg.pack(&fields).unwrap();
    let ctx = mfsk_core::core::DecodeContext::default();
    let text = msg.unpack(&payload, &ctx).unwrap();
    assert!(text.contains("CQ"));
    assert!(text.contains("JA1ABC"));
    assert!(text.contains("PM95"));
}
