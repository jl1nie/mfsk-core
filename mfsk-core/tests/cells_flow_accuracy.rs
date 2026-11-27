//! The cells flow's reported **SNR, DT and frequency** against what WSJT-X's `jt9` printed for the same decodes: tier B, on ft8_lib's 22 real recordings
//! (`tests/fixtures/ft8_lib_jt9_beta6.tsv`, `jt9 -8 -d 3` of v3.3.0-beta6, 402 decodes; the wavs are vendored in `embedded-poc/assets/`).
//!
//! The cells flow's `snr_db` is `SnrBaseline`'s: the power of the 79 transmitted tones over the baseline of the cells' own average spectrum (`baseline.f90`'s fit), `K` and the dB scale fitted to `jt9`'s
//! (2026-11-27: 264 matched decodes, SD 2.14 dB, mean absolute 1.58, largest 7.8, 91 % within 4 dB; the staircase's own adjacent-tone figure was -11.5 dB off with SD 9.0). `jt9` prints DT to 0.1 s and
//! frequency to 1 Hz, so the SD of the DT (35 ms) and of the frequency (0.9 Hz) on the plain flow is the rounding of the reference, not the 80 ms / 3.125 Hz grid: no refinement was added there.
//! Two recordings sit 4-5 dB off together (`websdr_test5`, `_test7`): the ones whose baseline the faithful path also got wrong until #747 (a passband that rolls off inside the band).
//!
//! **Fails if** the SNR leaves the measured band (mean, SD, share within 4 dB, largest), the DT or the frequency drifts, or the flows decode fewer of `jt9`'s than the floor (recall of the matched set).
#![cfg(all(feature = "fixed-point", feature = "internal-testing"))]
use mfsk_core::engine::dsp::resample::resample_to_8k;
use mfsk_core::ft8::decode::{DecodeDepth, DecodeResult};
use mfsk_core::ft8::decode_block::{
    Ft8Rate, GatedSearch, decode_block_cells, decode_block_cells_sic,
};
use mfsk_core::msg::wsjt77::unpack77;

const REFERENCE: &str = include_str!("fixtures/ft8_lib_jt9_beta6.tsv");

fn audio8k(name: &str) -> Vec<i16> {
    let p = format!(
        "{}/../embedded-poc/assets/{name}.wav",
        env!("CARGO_MANIFEST_DIR")
    );
    let w = std::fs::read(&p).unwrap_or_else(|e| panic!("vendored wav {p}: {e}"));
    let s: Vec<i16> = w[44..]
        .chunks_exact(2)
        .take(180_000)
        .map(|b| i16::from_le_bytes([b[0], b[1]]))
        .collect();
    resample_to_8k(&s, 12_000)
}

struct Errors {
    matched: usize,
    snr: Vec<f32>,
    dt: Vec<f32>,
    freq: Vec<f32>,
}

fn stat(v: &[f32]) -> (f32, f32, f32) {
    let n = v.len() as f32;
    let m = v.iter().sum::<f32>() / n;
    let sd = (v.iter().map(|x| (x - m) * (x - m)).sum::<f32>() / n).sqrt();
    (m, sd, v.iter().fold(0.0f32, |a, x| a.max(x.abs())))
}

fn norm(s: &str) -> String {
    s.split_whitespace().collect::<Vec<_>>().join(" ")
}

fn errors(decode: impl Fn(&[i16]) -> Vec<DecodeResult>) -> Errors {
    let mut files: Vec<&str> = REFERENCE
        .lines()
        .filter(|l| !l.starts_with('#') && !l.is_empty())
        .map(|l| l.split('\t').next().unwrap())
        .collect();
    files.sort();
    files.dedup();
    let mut e = Errors {
        matched: 0,
        snr: Vec::new(),
        dt: Vec::new(),
        freq: Vec::new(),
    };
    for f in files {
        let out = decode(&audio8k(f));
        for l in REFERENCE
            .lines()
            .filter(|l| !l.starts_with('#') && !l.is_empty())
        {
            let c: Vec<&str> = l.split('\t').collect();
            if c[0] != f {
                continue;
            }
            let msg = norm(c[4]);
            if let Some(r) = out
                .iter()
                .find(|r| unpack77(r.message77()).map(|m| norm(&m)) == Some(msg.clone()))
            {
                e.matched += 1;
                let (js, jd, jf): (f32, f32, f32) = (
                    c[3].parse().unwrap(),
                    c[2].parse().unwrap(),
                    c[1].parse().unwrap(),
                );
                e.snr.push(r.snr_db - js);
                e.dt.push(r.dt_sec - jd);
                e.freq.push(r.freq_hz - jf);
            }
        }
    }
    e
}

fn report(name: &str, e: &Errors) {
    let (sm, ss, sx) = stat(&e.snr);
    let (dm, ds, dx) = stat(&e.dt);
    let (fm, fs, fx) = stat(&e.freq);
    let within = |v: &[f32], t: f32| v.iter().filter(|x| x.abs() <= t).count();
    eprintln!(
        "{name}: matched {} | snr mean {sm:+.2} sd {ss:.2} max {sx:.1}, within 2 / 4 dB {} / {} | dt mean {dm:+.3} sd {ds:.3} max {dx:.3} | freq mean {fm:+.2} sd {fs:.2} max {fx:.2}",
        e.matched,
        within(&e.snr, 2.0),
        within(&e.snr, 4.0),
    );
}

#[test]
fn the_plain_flow_reports_jt9s_snr_dt_and_frequency() {
    let e = errors(|a| {
        decode_block_cells(
            a,
            Ft8Rate::Hz8000,
            &GatedSearch::new(200.0, 3000.0, 1.3, 30),
            30,
            DecodeDepth::EMBEDDED,
        )
        .unwrap()
    });
    report("plain flow, 30 candidates, OSD off", &e);
    assert!(e.matched >= 260, "recall of jt9's: {}", e.matched);
    let (m, sd, mx) = stat(&e.snr);
    assert!(m.abs() <= 0.5, "SNR mean error {m}");
    assert!(sd <= 2.6, "SNR error SD {sd}");
    assert!(mx <= 9.0, "largest SNR error {mx}");
    assert!(
        e.snr.iter().filter(|x| x.abs() <= 4.0).count() * 100 >= e.matched * 88,
        "share within 4 dB"
    );
    let (_, dsd, dmx) = stat(&e.dt);
    assert!(dsd <= 0.045 && dmx <= 0.10, "DT sd {dsd} max {dmx}");
    let (_, fsd, fmx) = stat(&e.freq);
    assert!(fsd <= 1.2 && fmx <= 3.2, "freq sd {fsd} max {fmx}");
}

#[test]
fn the_sic_flow_reports_them_too() {
    let e = errors(|a| {
        decode_block_cells_sic(
            a,
            Ft8Rate::Hz8000,
            &GatedSearch::new(200.0, 3000.0, 1.3, 30),
            30,
            DecodeDepth::EMBEDDED,
            2,
        )
        .unwrap()
    });
    report("SIC 2 passes, 30 candidates, OSD off", &e);
    assert!(e.matched >= 300, "recall of jt9's: {}", e.matched);
    let (m, sd, mx) = stat(&e.snr);
    assert!(m.abs() <= 0.5, "SNR mean error {m}");
    assert!(sd <= 2.6, "SNR error SD {sd}");
    assert!(mx <= 9.0, "largest SNR error {mx}");
    // the subtraction passes place each decode on the cells (`View::refine`) and report that place: finer than the grid
    let (_, dsd, dmx) = stat(&e.dt);
    assert!(dsd <= 0.035 && dmx <= 0.07, "DT sd {dsd} max {dmx}");
    let (_, fsd, fmx) = stat(&e.freq);
    assert!(fsd <= 0.6 && fmx <= 2.0, "freq sd {fsd} max {fmx}");
}
