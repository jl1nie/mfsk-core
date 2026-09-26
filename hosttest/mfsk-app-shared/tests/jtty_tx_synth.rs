//! The JTTY transmit sequencer driving mfsk-core's streaming synthesiser: the samples
//! the sequencer says are the message, taken from `Synth` at the offsets it says, are
//! the message — whatever the size of the audio chunks (#499).

use mfsk_app_shared_hosttest::jtty_tx::{JttyTx, SegKind, TxConfig};
use mfsk_core::jtty::pack::{self, ExchangeProfile};
use mfsk_core::jtty::tx::{self, Synth};

/// What the board does: poll per audio chunk, fill message segments from the synth.
fn transmit(text: &str, chunk: usize) -> (Vec<f32>, Vec<f32>, usize) {
    let tones = pack::tones(text, ExchangeProfile::Unknown)
        .unwrap()
        .unwrap();
    let whole = tx::synth_f32(&tones, 1500.0, 1.0);
    let mut synth = Synth::<f32>::new(&tones, 1500.0, 1.0);
    let mut seq = JttyTx::new(TxConfig {
        hold: 0,
        ..TxConfig::new()
    });
    seq.submit(synth.total_samples()).unwrap();

    let (mut sent, mut blanked) = (Vec::new(), 0usize);
    let total = TxConfig::new().lead + synth.total_samples() + TxConfig::new().tail + 1000;
    let mut t = 0;
    while t < total {
        let c = chunk.min(total - t);
        for seg in seq.poll(c, false).segments {
            blanked += if seg.blanks_receiver() { seg.len } else { 0 };
            if let SegKind::Message { offset } = seg.kind {
                // the synthesiser is asked in order; the offset says where it must be
                assert_eq!(offset, sent.len(), "offset {offset}");
                let mut buf = vec![0f32; seg.len];
                assert_eq!(synth.fill(&mut buf), seg.len);
                sent.extend_from_slice(&buf);
            }
        }
        t += c;
    }
    (sent, whole, blanked)
}

#[test]
fn the_message_comes_out_whole_whatever_the_chunk_size() {
    for chunk in [1, 480, 960, 4096, 22_656] {
        let (sent, whole, blanked) = transmit("CQ K1ABC CQ", chunk);
        assert_eq!(sent.len(), whole.len(), "chunk {chunk}");
        let worst = sent
            .iter()
            .zip(&whole)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f32::max);
        assert!(worst < 6e-3, "chunk {chunk}: {worst}");
        let cfg = TxConfig::new();
        assert_eq!(blanked, cfg.lead + whole.len() + cfg.tail, "chunk {chunk}");
    }
}

#[test]
fn a_sixteen_frame_message_is_the_longest_the_sequencer_takes() {
    let (sent, whole, _) = transmit(&"X".repeat(80), 960);
    assert_eq!(whole.len(), mfsk_app_shared_hosttest::jtty_tx::MAX_SAMPLES);
    assert_eq!(sent.len(), whole.len());
}
