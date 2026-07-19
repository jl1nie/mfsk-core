/* streaming_recipe.c — integration recipe for `mfsk_ft8_stream_*`.
 *
 * **What this is.** A documentation artefact, not a runnable program.
 * It shows where the streaming wrapper API fits into a typical
 * real-time receiver: a capture-side callback that pushes DMA chunks,
 * a decode-side timer that snapshots a 15-second slot, decodes, and
 * drains the consumed prefix.
 *
 * **What this is NOT.** It does not bring up any specific audio
 * source. The `audio_source_*` symbols below are placeholders — wire
 * them up to whatever your platform actually provides:
 *
 *   - ESP32 I2S PDM mic            → esp-idf `i2s_channel_read`
 *   - ESP32-S3 I2S line-in codec   → esp-idf `i2s_channel_read`
 *   - ESP32-S3 USB Audio Class     → tinyusb host UAC ISO callback
 *                                    or `esp_usb_audio` component
 *   - RP2350 I2S via PIO           → pico-extras `audio_i2s` rx
 *   - STM32 USB Audio host         → STM32CubeUSB UAC class driver
 *   - Desktop                      → ALSA / PortAudio / CoreAudio
 *
 * The API surface this recipe exercises is identical across all of
 * those — the only thing that changes is what fires `audio_chunk_cb`
 * and at what rate. mfsk-ffi-ft8 itself ships no audio drivers and
 * deliberately stays platform-agnostic (see docs/EMBEDDED.md
 * "What we don't ship").
 *
 * **Building.** Compiles standalone against `mfsk_ft8.h`. There is no
 * audio source linked, so it does nothing useful at runtime — the
 * value is in the structure, the comments, and the type-checking that
 * confirms the API surface is consistent with what you read in
 * docs/EMBEDDED.md.
 *
 *   gcc -I mfsk-ffi-ft8/include \
 *       -c mfsk-ffi-ft8/examples/streaming_recipe.c \
 *       -o /tmp/streaming_recipe.o
 *
 * Slot-boundary alignment: see docs/EMBEDDED.md §"Streaming capture".
 * decode_block tolerates ±2 s drift via coarse-sync, so NTP / GPS
 * PPS / freerunning timers all work for triggering on_slot_boundary.
 */

#include "mfsk_ft8.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/* ── Tunables ──────────────────────────────────────────────────────── */

/* Source sample rate in Hz. Common values:
 *   16000 — Core2 SPM1423 PDM mic, most ESP32 I2S codecs default
 *   24000 — some Cortex-M I2S codecs
 *   48000 — USB Audio Class default, line-in codecs (WM8960, etc.)
 *   12000 — bypass the resampler entirely (rare; only purpose-built
 *           SDR audio paths run at 12 kHz natively)
 */
#define SRC_RATE_HZ 16000U

/* 12 kHz ring capacity. 180_000 = 15 s = one full FT8 slot. Smaller
 * saves ~360 KB of buffer at the cost of less history; larger lets
 * the decode task lag without overwriting (e.g. 360_000 = 30 s for
 * "decode every other slot" workflows). Must be ≥ 168_000 for
 * decode_block's minimum input.
 */
#define RING_CAPACITY 180000U

/* Decode parameters — see docs/EMBEDDED.md §"API at a glance". */
#define FREQ_MIN_HZ 200.0f
#define FREQ_MAX_HZ 3000.0f
#define SYNC_MIN    1.0f
#define MAX_CAND    30

/* ── State ─────────────────────────────────────────────────────────── */

static MfskFt8Stream *g_stream;

/* `g_slot` lives in PSRAM if the linker supplies it (default on
 * Core2 with `CONFIG_SPIRAM_USE_MALLOC=y`); on internal-DRAM-only
 * targets just declare it as a regular `.bss` array — 360 KB fits
 * comfortably on RP2350 / Cortex-M with external PSRAM.
 */
static int16_t g_slot[180000];

/* ── Capture side ─────────────────────────────────────────────────── *
 *
 * Wire `audio_chunk_cb` into your platform's audio-arrived event:
 *
 *   ESP-IDF I2S PDM:
 *     while (running) {
 *         size_t n = 0;
 *         i2s_channel_read(rx_handle, buf, sizeof(buf), &n,
 *                          portMAX_DELAY);
 *         audio_chunk_cb(buf, n / sizeof(int16_t));
 *     }
 *
 *   tinyusb UAC2 host (ISO IN endpoint):
 *     void uac_iso_in_cb(uint8_t dev, uint8_t ep,
 *                        const uint8_t *buf, uint16_t len, …) {
 *         audio_chunk_cb((const int16_t *)buf, len / 2);
 *     }
 *
 *   ALSA capture (host desktop):
 *     while (running) {
 *         snd_pcm_readi(handle, buf, FRAMES);
 *         audio_chunk_cb(buf, FRAMES);
 *     }
 *
 * Single-threaded reminder: `MfskFt8Stream` is not internally
 * synchronised. If your capture and decode tasks can run on
 * different cores, gate `audio_chunk_cb` and `on_slot_boundary` with
 * a mutex around the stream pointer (or hop the chunks across a
 * lock-free SPSC queue and call `mfsk_ft8_stream_push_i16` only from
 * the decode side).
 */
static void audio_chunk_cb(const int16_t *samples, size_t n) {
    /* Push into the resampler + ring. Stays at SRC_RATE_HZ until
     * the wrapper resamples to 12 kHz internally. Returns Ok unless
     * the stream pointer was somehow stale (shouldn't happen after
     * init).
     */
    (void)mfsk_ft8_stream_push_i16(g_stream, samples, n);
}

/* ── Decode side ──────────────────────────────────────────────────── *
 *
 * Fire `on_slot_boundary()` once per 15-second FT8 slot. Sources:
 *   - NTP-synced wall clock: trigger when `gettimeofday()` lands on
 *     a 15 s UTC multiple (simplest on Wi-Fi targets).
 *   - GPS PPS: 1 PPS divided by 15 (mobile / offline operation).
 *   - Free-running esp_timer / FreeRTOS xTimer at 15 s period
 *     (stand-alone benches; decode_block's coarse-sync absorbs ±2 s
 *     of drift, so any timer stable to ~50 ppm/hr works).
 */
static void on_slot_boundary(void) {
    /* Need at least 14 s of audio (decode_block's minimum). With a
     * 15 s ring and 1+ s of fill since the last drain, this is
     * satisfied unless the capture task has stalled.
     */
    if (mfsk_ft8_stream_buffered_samples(g_stream) < 168000) {
        fprintf(stderr, "slot underrun — capture task stalled?\n");
        return;
    }

    /* Snapshot the latest 15 s. Does not modify the ring (so a
     * race with capture-side push is benign — the snapshot may or
     * may not include the very last in-flight sample; either way
     * the rest of the slot is consistent).
     */
    size_t n = mfsk_ft8_stream_peek_latest(g_stream, g_slot, 180000);
    if (n < 168000) {
        return; /* unreachable given the buffered_samples check above */
    }

    MfskFt8ResultList results = {0};
    MfskFt8Status st = mfsk_ft8_decode_i16(
        g_slot, n,
        FREQ_MIN_HZ, FREQ_MAX_HZ, SYNC_MIN, MAX_CAND,
        MFSK_FT8_DEPTH_BP_ALL,
        &results);

    if (st == MFSK_FT8_STATUS_OK) {
        for (size_t i = 0; i < results.len; ++i) {
            const MfskFt8Result *r = &results.items[i];
            printf("  %+4.1f dB  %5.1f Hz  dt=%+.2f s  '%s'\n",
                   (double)r->snr_db, (double)r->freq_hz,
                   (double)r->dt_sec, r->text);
        }
    }
    mfsk_ft8_result_list_free(&results);

    /* Drain the consumed slot to free room for fresh audio. Use
     * `n` (what we actually decoded) rather than 180000 to be
     * robust if the ring was below capacity. For overlapped decode
     * (e.g. 5 s overlap between successive slots), drain only
     * (n - 60000) instead.
     */
    mfsk_ft8_stream_drain(g_stream, n);
}

/* ── Lifecycle ────────────────────────────────────────────────────── */

bool rx_init(void) {
    g_stream = mfsk_ft8_stream_new(SRC_RATE_HZ, RING_CAPACITY);
    if (!g_stream) {
        fprintf(stderr, "mfsk_ft8_stream_new failed\n");
        return false;
    }
    return true;
}

void rx_shutdown(void) {
    mfsk_ft8_stream_free(g_stream);
    g_stream = NULL;
}

/* No `main()` — this file is meant to be `#include`'d into your app
 * or copied piece-wise. If you want a runnable smoke test that
 * decodes a baked WAV, see `tests/c_smoke/smoke.c`.
 */
