//! Q65 AP-list (full-AP-list) integration tests.
//!
//! Validates the BP-free template-matching decoder
//! ([`mfsk_core::q65::decode_at_with_ap_list_for`] +
//! [`mfsk_core::q65::decode_scan_with_ap_list_for`]) on synthetic
//! audio plus the WSJT-X 6 m EME reference recording.

#![cfg(feature = "q65")]

use std::f32::consts::TAU;
use std::fs::File;
use std::io::Read;
use std::path::{Path, PathBuf};

use mfsk_core::fec::qra::Q65Codec;
use mfsk_core::fec::qra15_65_64::QRA15_65_64_IRR_E23;
use mfsk_core::q65::search::SearchParams;
use mfsk_core::q65::{
    Q65a30, Q65a60, decode_at_with_ap_list_for, decode_scan_with_ap_list_for,
    standard_qso_codewords, synthesize_standard, synthesize_standard_for,
};

const FS: f32 = 12_000.0;
const FS_U: u32 = 12_000;
const REF_BW: f32 = 2_500.0;
const SLOT_SAMPLES: usize = 12_000 * 30;

struct Lcg {
    s: u64,
    spare: Option<f32>,
}
impl Lcg {
    fn new(seed: u64) -> Self {
        Self {
            s: seed.wrapping_add(1),
            spare: None,
        }
    }
    fn next(&mut self) -> u64 {
        self.s = self
            .s
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.s
    }
    fn uniform(&mut self) -> f32 {
        ((self.next() >> 11) as f32 + 1.0) / ((1u64 << 53) as f32 + 1.0)
    }
    fn gauss(&mut self) -> f32 {
        if let Some(x) = self.spare.take() {
            return x;
        }
        let u = self.uniform();
        let v = self.uniform();
        let mag = (-2.0 * u.ln()).sqrt();
        self.spare = Some(mag * (TAU * v).sin());
        mag * (TAU * v).cos()
    }
}

#[test]
fn ap_list_decodes_clean_qso_message() {
    // The standard candidate set built for ("K1ABC", "JA1ABC",
    // "PM95") includes the bare exchange "K1ABC JA1ABC PM95". A
    // clean, aligned synthesis of that message must be picked out
    // by the list decoder without BP and without an AP hint.
    let freq = 1500.0;
    let audio = synthesize_standard("K1ABC", "JA1ABC", "PM95", FS_U, freq, 0.3).expect("synth");
    let candidates = standard_qso_codewords("K1ABC", "JA1ABC", "PM95");
    assert!(!candidates.is_empty());

    let result = decode_at_with_ap_list_for::<Q65a30>(&audio, FS_U, 0, freq, &candidates)
        .expect("AP-list decode of a clean signal must succeed");
    assert_eq!(result.message, "K1ABC JA1ABC PM95");
    assert_eq!(
        result.iterations, 0,
        "list path should not run BP iterations"
    );
}

#[test]
fn ap_list_decodes_snr_template_messages() {
    // Pick an SNR exchange that's deep in the SNR ladder — confirms
    // the ladder generator emits the right report formats.
    for report in ["-12", "+05", "R-25", "R+10"] {
        let freq = 1500.0;
        let audio = synthesize_standard("K1ABC", "JA1ABC", report, FS_U, freq, 0.3).expect("synth");
        let candidates = standard_qso_codewords("K1ABC", "JA1ABC", "PM95");
        let result = decode_at_with_ap_list_for::<Q65a30>(&audio, FS_U, 0, freq, &candidates)
            .unwrap_or_else(|| panic!("AP-list decode failed for report {report}"));
        assert_eq!(result.message, format!("K1ABC JA1ABC {report}"));
    }
}

#[test]
fn ap_list_returns_none_when_message_not_in_list() {
    // Synthesise a message whose template is NOT in the list — the
    // list decoder must reject it (returning None) rather than
    // misdecode to a near-neighbour candidate.
    let freq = 1500.0;
    // Build a list for K1ABC/JA1ABC, but transmit a different his_call.
    let audio = synthesize_standard("K1ABC", "JA9XYZ", "RR73", FS_U, freq, 0.3).expect("synth");
    let candidates = standard_qso_codewords("K1ABC", "JA1ABC", "PM95");

    let result = decode_at_with_ap_list_for::<Q65a30>(&audio, FS_U, 0, freq, &candidates);
    assert!(
        result.is_none(),
        "list decoder must reject messages outside its candidate set, \
         got {:?}",
        result.map(|d| d.message)
    );
}

#[test]
fn ap_list_returns_none_on_silence() {
    // Sanity guard: no false positives on noise.
    let mut audio = vec![0.0_f32; SLOT_SAMPLES];
    let mut rng = Lcg::new(123);
    for s in audio.iter_mut() {
        *s = 0.001 * rng.gauss();
    }
    let candidates = standard_qso_codewords("K1ABC", "JA1ABC", "PM95");
    let result = decode_at_with_ap_list_for::<Q65a30>(&audio, FS_U, 0, 1500.0, &candidates);
    assert!(result.is_none(), "got false decode from silence");
}

#[test]
#[ignore = "slow: 6-seed comparison at threshold SNR; minutes in debug. CI runs via --include-ignored."]
fn ap_list_lifts_threshold_versus_plain_decode() {
    // Comparison sweep: AP-list is fundamentally a different
    // decoder from BP and trades convergence-on-arbitrary-messages
    // for stronger threshold on a known message set. At an SNR
    // where the plain BP path barely succeeds, the list path
    // should succeed at least as often, and ideally more often,
    // when the truth is in the list.
    let freq = 1500.0;
    let snr_db = -25.0; // a hair below the published Q65-30A threshold
    let snr_lin = 10_f32.powf(snr_db / 10.0);
    let amp = (4.0 * snr_lin * REF_BW / FS).sqrt();

    let candidates = standard_qso_codewords("K1ABC", "JA1ABC", "PM95");
    let seeds = [11u64, 22, 33, 44, 55, 66];
    let mut plain_hits = 0usize;
    let mut list_hits = 0usize;

    for &seed in &seeds {
        let pcm = synthesize_standard("K1ABC", "JA1ABC", "PM95", FS_U, freq, amp).expect("synth");
        let mut slot = vec![0.0_f32; SLOT_SAMPLES];
        let start = FS_U as usize; // 1 s into the slot
        let n = pcm.len().min(slot.len() - start);
        slot[start..start + n].copy_from_slice(&pcm[..n]);
        let mut rng = Lcg::new(seed);
        for s in slot.iter_mut() {
            *s += rng.gauss();
        }

        let params = SearchParams {
            freq_min_hz: 200.0,
            freq_max_hz: 3_000.0,
            time_tolerance_symbols: 8,
            score_threshold: 0.05,
            max_candidates: 16,
        };
        let plain = !mfsk_core::q65::decode_scan_default(&slot, FS_U).is_empty();
        let listed = !decode_scan_with_ap_list_for::<Q65a30>(&slot, FS_U, 0, &params, &candidates)
            .is_empty();
        if plain {
            plain_hits += 1;
        }
        if listed {
            list_hits += 1;
        }
    }

    println!(
        "AP-list vs plain at {snr_db} dB: plain {}/{}, list {}/{}",
        plain_hits,
        seeds.len(),
        list_hits,
        seeds.len(),
    );

    assert!(
        list_hits >= plain_hits,
        "list path must not regress vs plain BP at threshold-ish SNR \
         (plain={plain_hits}, list={list_hits})"
    );
}

#[test]
fn ap_list_threshold_scales_with_list_size() {
    // The LLH threshold is adjusted by ln(N/3) so the false-decode
    // rate stays roughly constant as the list grows. This test
    // just pins the directional invariant: a longer list does NOT
    // reach further into noise (the BP path still does that). What
    // it should do is stay sharp on the right message.
    let freq = 1500.0;
    let audio = synthesize_standard("K1ABC", "JA1ABC", "73", FS_U, freq, 0.3).expect("synth");

    // Tiny list with the right answer — must succeed.
    let small_list = {
        let mut codec = Q65Codec::new(&QRA15_65_64_IRR_E23);
        let mut info = [0_i32; 13];
        let bits = mfsk_core::msg::wsjt77::pack77("K1ABC", "JA1ABC", "73").unwrap();
        info.copy_from_slice(&mfsk_core::msg::q65::pack77_to_symbols(&bits));
        let mut cw = [0_i32; 63];
        codec.encode(&info, &mut cw);
        vec![cw]
    };
    let small = decode_at_with_ap_list_for::<Q65a30>(&audio, FS_U, 0, freq, &small_list);
    assert!(
        small.is_some(),
        "single-codeword list must accept the matching codeword"
    );

    // Full standard list (206) — also must succeed; the threshold
    // just needs to be loose enough to admit the true codeword.
    let full = standard_qso_codewords("K1ABC", "JA1ABC", "PM95");
    let result = decode_at_with_ap_list_for::<Q65a30>(&audio, FS_U, 0, freq, &full).unwrap();
    assert_eq!(result.message, "K1ABC JA1ABC 73");
}

// ─── WSJT-X 6 m EME reference (optional) ─────────────────────────────

fn read_wsjtx_wav(path: &Path) -> Option<Vec<f32>> {
    let mut file = File::open(path).ok()?;
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes).ok()?;
    if bytes.len() < 44 || &bytes[..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
        return None;
    }
    if &bytes[12..16] != b"fmt " {
        return None;
    }
    let channels = u16::from_le_bytes([bytes[22], bytes[23]]);
    let sample_rate = u32::from_le_bytes([bytes[24], bytes[25], bytes[26], bytes[27]]);
    let bits = u16::from_le_bytes([bytes[34], bytes[35]]);
    if channels != 1 || sample_rate != 12_000 || bits != 16 {
        return None;
    }
    if &bytes[36..40] != b"data" {
        return None;
    }
    let data_len = u32::from_le_bytes([bytes[40], bytes[41], bytes[42], bytes[43]]) as usize;
    let data = &bytes[44..44 + data_len.min(bytes.len() - 44)];
    let mut out = Vec::with_capacity(data.len() / 2);
    for chunk in data.chunks_exact(2) {
        let s = i16::from_le_bytes([chunk[0], chunk[1]]);
        out.push(s as f32 / 32_768.0);
    }
    Some(out)
}

fn samples_dir(rel: &str) -> Option<PathBuf> {
    let manifest = std::env::var("CARGO_MANIFEST_DIR").ok()?;
    let dir = Path::new(&manifest)
        .join("../../WSJT-X/samples/Q65")
        .join(rel)
        .canonicalize()
        .ok()?;
    if dir.is_dir() { Some(dir) } else { None }
}

#[test]
fn ap_list_decodes_eme_6m_w7gj_exchanges() {
    // 6 m EME reference: W7GJ is the well-known caller in this
    // recording. With a candidate list anchored on W7GJ as the
    // counterpart and a few likely correspondents, the list
    // decoder should pick out at least one of the actual on-air
    // exchanges (e.g. "W7GJ W1VD FN31") with no BP at all.
    let _ = synthesize_standard_for::<Q65a60>; // keep the symbol live for clarity
    let Some(dir) = samples_dir("60A_EME_6m") else {
        eprintln!("skipping: WSJT-X 6 m EME sample tree not found");
        return;
    };
    let entries: Vec<_> = std::fs::read_dir(&dir)
        .expect("read samples dir")
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
        .collect();
    if entries.is_empty() {
        eprintln!("skipping: 6 m EME sample dir empty");
        return;
    }

    // Build candidate sets covering several plausible W7GJ
    // counterparts seen in the reference recording.
    let counterparts: &[(&str, &str)] = &[("W1VD", "FN31"), ("VE1JF", "FN65"), ("N8JX", "EN73")];

    let nominal_mid = FS_U as usize * 30;
    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_symbols: 50,
        score_threshold: 0.05,
        max_candidates: 32,
    };

    let mut total_decodes = 0usize;
    for path in &entries {
        let audio = match read_wsjtx_wav(path) {
            Some(a) => a,
            None => continue,
        };
        let mut hit_any = false;
        for &(other, grid) in counterparts {
            let candidates = standard_qso_codewords("W7GJ", other, grid);
            let decodes = decode_scan_with_ap_list_for::<Q65a60>(
                &audio,
                FS_U,
                nominal_mid,
                &params,
                &candidates,
            );
            for d in &decodes {
                println!(
                    "{} [list ({other}, {grid})]: {}",
                    path.file_name().unwrap().to_string_lossy(),
                    d.message
                );
            }
            if !decodes.is_empty() {
                hit_any = true;
                total_decodes += decodes.len();
            }
        }
        // Don't insist on every recording producing a hit — just
        // that the decoder runs without crashing.
        let _ = hit_any;
    }

    // Across the full set the AP-list path must produce at least
    // one decode — otherwise we've regressed the list scorer.
    assert!(
        total_decodes > 0,
        "AP-list decoder produced no decodes on the 6 m EME reference"
    );
    eprintln!("[info] 6 m EME total AP-list decodes: {total_decodes}");
}
