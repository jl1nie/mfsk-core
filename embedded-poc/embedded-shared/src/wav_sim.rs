//! WAV-fed audio source — sends audio chunks to the stage1_inc worker
//! via a FreeRTOS Queue at real-time pace.
//!
//! Stand-in for the I2S PDM mic. `spawn` consumes a slice of WAV
//! byte-slices (each 12 kHz / mono / 16-bit, 44-byte RIFF header) and
//! sends `ChunkMsg::Samples` (CHUNK_LEN samples per 100 ms) followed
//! by `ChunkMsg::SlotEnd` at WAV completion.
//!
//! No shared state. No notify-out-pointer split. The chunk queue is
//! the only way data flows out of this task.

use core::ffi::c_void;
use core::ptr;

use alloc::boxed::Box;
use alloc::vec::Vec;

use esp_idf_svc::sys::{configTICK_RATE_HZ, vTaskDelay, xTaskCreatePinnedToCore, QueueHandle_t};

use crate::pipeline::{send_box, ChunkMsg, CHUNK_LEN};

const PD_PASS: i32 = 1;

const fn ms_to_ticks(ms: u32) -> u32 {
    ms / (1000 / configTICK_RATE_HZ)
}

/// PCM WAV header size (RIFF 12 + fmt 24 + data 8).
const WAV_HEADER_BYTES: usize = 44;
/// One FT8 slot at 12 kHz. Any baked WAV with > SLOT_SAMPLES is
/// trimmed; shorter WAVs are padded by stage1_inc receiving the
/// reported `total_samples` regardless.
const SLOT_SAMPLES: usize = 180_000;

/// Task-local config passed via task arg.
struct Cfg {
    wavs: &'static [&'static [u8]],
    chunk_q: QueueHandle_t,
}

/// Spawn the WAV-feed task. Cycles through `wavs` indefinitely, sending
/// CHUNK_LEN-sized `ChunkMsg::Samples` every 100 ms and `ChunkMsg::SlotEnd`
/// at WAV completion.
pub fn spawn(wavs: &'static [&'static [u8]], chunk_q: QueueHandle_t) {
    assert!(!wavs.is_empty(), "wav_sim::spawn: no WAVs provided");
    let cfg = Box::new(Cfg { wavs, chunk_q });
    let arg = Box::into_raw(cfg) as *mut c_void;
    let r = unsafe {
        xTaskCreatePinnedToCore(
            Some(sim_task_main),
            c"wav_sim".as_ptr(),
            4096,
            arg,
            // Below decode main (6) so notifies preempt; above
            // stage1_inc (3) so chunks arrive promptly.
            4,
            ptr::null_mut(),
            0, // PRO_CPU
        )
    };
    assert_eq!(r, PD_PASS, "xTaskCreatePinnedToCore(wav_sim) failed: {r}");
    log::info!("wav_sim: spawned, {} WAV(s) in playlist", wavs.len());
}

extern "C" fn sim_task_main(arg: *mut c_void) {
    let cfg: Box<Cfg> = unsafe { Box::from_raw(arg as *mut Cfg) };
    let chunk_q = cfg.chunk_q;
    let wavs = cfg.wavs;
    // Keep cfg alive forever; leak intentionally since the task never returns.
    core::mem::forget(cfg);

    let delay_ticks: u32 = ms_to_ticks(100);

    loop {
        for (idx, wav) in wavs.iter().enumerate() {
            log::info!("wav_sim: starting WAV[{idx}] ({} bytes)", wav.len());
            let payload = &wav[WAV_HEADER_BYTES..];
            let n_samples = (payload.len() / 2).min(SLOT_SAMPLES);

            let mut i = 0;
            while i < n_samples {
                let end = (i + CHUNK_LEN).min(n_samples);
                let n_this = end - i;
                let mut chunk: Vec<i16> = Vec::with_capacity(n_this);
                for k in 0..n_this {
                    let off = (i + k) * 2;
                    chunk.push(i16::from_le_bytes([payload[off], payload[off + 1]]));
                }
                send_box(chunk_q, Box::new(ChunkMsg::Samples(chunk)));
                i = end;
                unsafe { vTaskDelay(delay_ticks) };
            }

            send_box(
                chunk_q,
                Box::new(ChunkMsg::SlotEnd {
                    wav_idx: idx,
                    total_samples: n_samples,
                }),
            );
        }
    }
}
