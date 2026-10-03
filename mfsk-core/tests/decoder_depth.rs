//! What `Depth` does per mode, against what the v3.2.0-rc1 decoders do for
//! `ndepth` 1 / 2 / 3. Each test cites the upstream lines it checks.
#![cfg(feature = "full")]

#[allow(dead_code)]
mod common;

use mfsk_core::decoder::{DecodeParams, Decoder, Depth, Fst4Extras, SlotInput};
use mfsk_core::fst4::Fst4s60;

const FST4_WAV: &str = asset_path!("golden/fst4/210115_0058.wav");

/// `fst4_decode.f90:234-248`: `jittermax` is 2 at `ndepth` 2 and 3 and 0 at
/// 1, OSD on at all three. Fast must therefore decode what the golden
/// recording's single position holds (OSD still runs), and may only lose
/// signals that need the `i0 ± 1` retry; Normal and Deep are identical.
#[test]
fn fst4_fast_is_osd_without_the_timing_retry() {
    let Some(a) = common::load_wav_i16_opt(FST4_WAV) else {
        common::skip_or_fail(FST4_WAV);
        return;
    };
    let run = |depth| {
        let mut d = Decoder::<Fst4s60>::new(DecodeParams::for_band((100.0, 3000.0)).depth(depth))
            .with_extras(Fst4Extras::default());
        d.decode(&SlotInput::i16(&a))
            .rows
            .into_iter()
            .map(|r| r.decoded.text)
            .collect::<Vec<_>>()
    };
    let (fast, normal, deep) = (run(Depth::Fast), run(Depth::Normal), run(Depth::Deep));
    assert_eq!(normal, deep, "FST4 Normal and Deep are the same decoder");
    assert!(!normal.is_empty());
    for m in &fast {
        assert!(normal.contains(m), "Fast found {m:?} that Normal did not");
    }
}

/// `q65_decode.f90:183-188`, `q65_loops.f90:27-40`: the grid search widens
/// with depth (1x1 cells and `maxiters` 40; 3x3 and 60; 5x5 and 100, with the
/// b90 sweep two wider each way). A wider search only adds cells, so what
/// Fast decodes Normal and Deep decode too.
#[test]
fn q65_depth_only_adds_cells() {
    use mfsk_core::decoder::{Q65Extras, SearchTuning};
    use mfsk_core::q65::Q65d60;
    let Some(path) = common::corpus::golden_path("q65/60D_EME_10GHz/201212_1838.wav") else {
        common::skip_or_fail("Q65 60D golden");
        return;
    };
    let a = common::load_wav_f32_opt(&path).unwrap();
    let run = |depth| {
        let mut d = Decoder::<Q65d60>::new(DecodeParams::for_band((200.0, 3000.0)).depth(depth))
            .with_extras(Q65Extras {
                search: SearchTuning {
                    time_tolerance_early_sec: Some(7.0),
                    time_tolerance_late_sec: Some(5.0),
                    score_threshold: Some(0.05),
                    max_candidates: Some(8),
                },
                fading: Some((mfsk_core::fec::qra::FadingModel::Gaussian, 10.0)),
                ..Default::default()
            });
        d.decode(&SlotInput::f32(&a))
            .rows
            .into_iter()
            .map(|r| r.decoded.text)
            .collect::<Vec<_>>()
    };
    let (fast, normal, deep) = (run(Depth::Fast), run(Depth::Normal), run(Depth::Deep));
    for m in &fast {
        assert!(normal.contains(m) && deep.contains(m), "{m:?}");
    }
    assert!(!fast.is_empty());
}

/// `jt65_decode.f90:110-145`: 2 passes and `nvec = 100` at `ndepth` 1, 2 and
/// 1000 at 2, 4 passes and 1000 at 3, the signals of each pass subtracted
/// before the next (`subtract65.f90`). On the `jt65sim` golden (the one
/// message at 700/1100/1500/1900/2300 Hz, -18 dB) every depth must find
/// all five carriers and nothing else; the second pass is what finds the
/// 1500 Hz copy the first misses (a single pass with the same candidates
/// finds four).
#[test]
fn jt65_depths_find_every_carrier_and_no_phantom() {
    use mfsk_core::Jt65;
    let Some(path) = common::corpus::golden_path("jt65/jt65a_5sig_m18.wav") else {
        common::skip_or_fail("JT65 golden");
        return;
    };
    let a = common::load_wav_f32_opt(&path).unwrap();
    for depth in [Depth::Fast, Depth::Normal, Depth::Deep] {
        let mut d = Decoder::<Jt65>::new(DecodeParams::for_band((300.0, 2700.0)).depth(depth));
        let rows = d.decode(&SlotInput::f32(&a)).rows;
        assert!(
            rows.iter().all(|r| r.decoded.text == "K1ABC W9XYZ EN37"),
            "{depth:?}: phantom in {:?}",
            rows.iter().map(|r| &r.decoded.text).collect::<Vec<_>>()
        );
        let mut carriers: Vec<i32> = rows
            .iter()
            .map(|r| (r.decoded.freq_hz / 100.0).round() as i32 * 100)
            .collect();
        carriers.sort();
        carriers.dedup();
        assert_eq!(carriers, [700, 1100, 1500, 1900, 2300], "{depth:?}");
    }
}

/// `jt9_decode.f90:81-135`: the pass around the Rx frequency (`nqd = 1`)
/// only adds to the wide scan: what the wide scan decodes is still
/// decoded, whichever frequency is the Rx frequency.
#[test]
fn jt9_rx_frequency_pass_only_adds() {
    use mfsk_core::Jt9;
    let Some(a) = common::load_wav_f32_opt(asset_path!("130418_1742.wav")) else {
        common::skip_or_fail("JT9 recording");
        return;
    };
    let run = |rx: Option<f32>| {
        let mut p = DecodeParams::for_band((1050.0, 1550.0)).depth(Depth::Fast);
        if let Some(f) = rx {
            p = p.rx_freq(f);
        }
        let mut d = Decoder::<Jt9>::new(p);
        d.decode(&SlotInput::f32(&a))
            .rows
            .into_iter()
            .map(|r| r.decoded.text)
            .collect::<Vec<_>>()
    };
    let wide = run(None);
    assert!(!wide.is_empty());
    for rx in [1100.0, 1300.0, 1500.0] {
        let narrow = run(Some(rx));
        for m in &wide {
            assert!(narrow.contains(m), "rx {rx}: lost {m:?}");
        }
        eprintln!("rx {rx}: wide {} narrow {}", wide.len(), narrow.len());
    }
}

/// `wsprd`'s arguments per GUI depth (`mainwindow.cpp:2824-2826`): Fast
/// `-qB` (two passes, no DT jitter), Normal `-C 500 -o 4`, Deep adds `-d`.
/// On the WSJT-X golden Normal and Deep must keep what Fast finds and
/// Fast, with no final pass, finds the strong stations.
#[test]
fn wspr_depths_on_the_golden() {
    use mfsk_core::Wspr;
    use mfsk_core::decoder::{SearchTuning, WsprExtras};
    let Some(path) = common::corpus::golden_path("wspr/150426_0918.wav") else {
        common::skip_or_fail("WSPR golden");
        return;
    };
    let a = common::load_wav_f32_opt(&path).unwrap();
    let run = |depth| {
        let t = std::time::Instant::now();
        let mut d = Decoder::<Wspr>::new(DecodeParams::for_band((1400.0, 1620.0)).depth(depth))
            .with_extras(WsprExtras {
                search: SearchTuning {
                    max_candidates: Some(100),
                    ..Default::default()
                },
                ..Default::default()
            });
        let r = d.decode(&SlotInput::f32(&a)).rows;
        eprintln!("{depth:?}: {} in {:?}", r.len(), t.elapsed());
        r.into_iter().map(|r| r.decoded.text).collect::<Vec<_>>()
    };
    let (fast, normal, deep) = (run(Depth::Fast), run(Depth::Normal), run(Depth::Deep));
    assert!(fast.len() >= 5, "Fast: {fast:?}");
    assert!(
        normal.len() >= fast.len(),
        "Normal {normal:?} vs Fast {fast:?}"
    );
    eprintln!(
        "deep extra: {:?}",
        deep.iter()
            .filter(|m| !normal.contains(m))
            .collect::<Vec<_>>()
    );
    assert!(
        deep.len() >= normal.len(),
        "Deep {deep:?} vs Normal {normal:?}"
    );
}

/// Upstream's AP is derived from the QSO context (`ft8b.f90:55-70`). A reply
/// to the operator (`K1JT HA0DU RR73`) at the edge of what a blind decode
/// hears: with the QSO context the AP passes (MyCall DxCall RR73) find it
/// more often than without, and never anything else.
#[test]
fn ft8_qso_context_ap_finds_the_weak_reply() {
    use mfsk_core::Ft8;
    use mfsk_core::decoder::{ApMode, Ft8Extras, QsoProgress, Tuning};
    use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
    use mfsk_core::msg::wsjt77::pack77;

    const MSG: &str = "K1JT HA0DU RR73";
    let bits = pack77("K1JT", "HA0DU", "RR73").unwrap();
    let tones = message_to_tones::<Ft8>(&bits);
    let amp = 6_000.0f32;
    let clean = synthesize_i16::<Ft8>(&tones, 12_000, 1_500.0, amp as i16);

    let decodes = |params: DecodeParams, snr_db: f32, seed: u64| {
        let mut audio = vec![0f32; 180_000];
        for (i, &v) in clean.iter().enumerate() {
            if 6_000 + i < audio.len() {
                audio[6_000 + i] = f32::from(v);
            }
        }
        // SNR in 2500 Hz: (A^2 / 2) / (sigma^2 * 2500 / 6000).
        let sigma = ((amp * amp / 2.0) / (10f32.powf(snr_db / 10.0) * 2500.0 / 6000.0)).sqrt();
        let mut ch = common::channel::AwgnChannel::new(sigma, seed);
        ch.apply(&mut audio);
        let pcm: Vec<i16> = audio
            .iter()
            .map(|v| v.round().clamp(-32768.0, 32767.0) as i16)
            .collect();
        let mut d = Decoder::<Ft8>::new(params).with_extras(Ft8Extras {
            tuning: Tuning {
                sync_min: Some(1.3),
                max_cand: Some(50),
                ..Default::default()
            },
            ..Default::default()
        });
        d.decode(&SlotInput::i16(&pcm))
            .rows
            .into_iter()
            .map(|r| r.decoded.text)
            .collect::<Vec<_>>()
    };
    let base = DecodeParams::for_band((200.0, 3000.0)).depth(Depth::Deep);
    let (mut off_hits, mut on_hits) = (0, 0);
    for snr in [-22.0f32, -23.0, -24.0] {
        for seed in 1..=10u64 {
            let off = decodes(base.clone().ap(ApMode::Off), snr, seed);
            let on = decodes(
                base.clone()
                    .ap(ApMode::Full)
                    .station("K1JT", "FN20")
                    .qso("HA0DU", "KN07", QsoProgress::Rogers)
                    .rx_freq(1_500.0),
                snr,
                seed,
            );
            assert!(
                off.iter().chain(on.iter()).all(|m| m == MSG),
                "phantom: {off:?} {on:?}"
            );
            off_hits += usize::from(!off.is_empty());
            on_hits += usize::from(!on.is_empty());
        }
    }
    eprintln!("hits of 30: AP off {off_hits}, AP on {on_hits}");
    assert!(on_hits > off_hits, "AP on {on_hits} vs off {off_hits}");
}
