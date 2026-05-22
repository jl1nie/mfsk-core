//! WAV-fed streaming RX bench (M5StickS3 / ESP32-S3 / LX7).
//!
//! Thin shim — all logic lives in `embedded_shared::apps::rx_wavsim`.
//!
//! Build: `cargo build --release --bin rx-wavsim`.

const QSO_WAVS: &[&[u8]] = &[
    include_bytes!("../../../assets/qso1.wav"),
    include_bytes!("../../../assets/qso2.wav"),
    include_bytes!("../../../assets/qso3_busy.wav"),
];

fn main() -> ! {
    // Ship config (post Hann→Rect window fix in stage1_inc):
    //   pass1=30, max_cand=15, q_thresh=DEFAULT_Q_THRESH (=6),
    //   bp_max_iter=DEFAULT_BP_MAX_ITER (=30).
    // qso3_busy: 6/18 JTDX hits. Compute-optimal Pareto across the
    // 9-cfg sweep (logs/s3_rect_sweep_*). Wider cfgs (45/20, 60/30)
    // recover no extra decodes — N1PJT HB9CQK at -10 dB is
    // structurally lost without fine_refine_pass1 (192k cd0 FFT,
    // infeasible on Xtensa).
    // Phase-C (run_speculative_slot) timing: see flash logs.
    embedded_shared::apps::rx_wavsim::run(
        QSO_WAVS,
        30,
        15,
        mfsk_core::ft8::decode_block::DEFAULT_Q_THRESH,
        mfsk_core::ft8::params::DEFAULT_BP_MAX_ITER,
    )
}
