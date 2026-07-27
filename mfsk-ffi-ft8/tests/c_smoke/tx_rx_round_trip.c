/* TX → RX round-trip smoke test.
 *
 * Encode a known message, synthesise PCM, immediately decode it,
 * print what came back. Demonstrates the full C ABI surface end-
 * to-end without any audio file I/O.
 *
 * Build:
 *   cargo build -p mfsk-ffi-ft8 --release
 *   gcc -O2 -I mfsk-ffi-ft8/include \
 *       mfsk-ffi-ft8/tests/c_smoke/tx_rx_round_trip.c \
 *       -L target/release -lmfsk_ft8 -lm -lpthread -ldl \
 *       -Wl,-rpath,$PWD/target/release \
 *       -o /tmp/mfsk_round_trip
 *   /tmp/mfsk_round_trip
 */

#include "mfsk_ft8.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SLOT_SAMPLES 168000  /* 14 s × 12 kHz */

int main(void) {
    /* 1. Pack "CQ JA1ABC PM86" into 77 bits. */
    uint8_t msg77[77];
    MfskStatus st = mfsk_ft8_pack77("CQ", "JA1ABC", "PM86", msg77);
    if (st != MFSK_STATUS_OK) {
        fprintf(stderr, "pack77 failed: %d\n", (int)st); return 2;
    }
    printf("packed 'CQ JA1ABC PM86' OK (status=%d)\n", (int)st);

    /* 2. 77 bits → 79-tone Gray-mapped sequence. */
    uint8_t itone[79];
    st = mfsk_ft8_message_to_tones(msg77, itone);
    if (st != MFSK_STATUS_OK) {
        fprintf(stderr, "message_to_tones failed: %d\n", (int)st); return 2;
    }
    printf("encoded 79 tones (first 7 Costas: %d %d %d %d %d %d %d)\n",
           itone[0], itone[1], itone[2], itone[3], itone[4], itone[5], itone[6]);

    /* 3. Synthesise into a slot-aligned i16 buffer.
     *
     * `tones_to_i16` writes 151 680 samples (12.64 s) starting at
     * offset 0. To match the receive-side `TX_START_OFFSET_S = 0.5 s`,
     * we offset into a 14-second slot buffer. */
    size_t synth_len = mfsk_ft8_synth_output_len();   /* 151 680 */
    int16_t *slot = calloc(SLOT_SAMPLES, sizeof(int16_t));
    if (!slot) { fprintf(stderr, "oom\n"); return 2; }
    size_t prepend_silence = (size_t)(0.5f * 12000.0f);  /* 6 000 */
    st = mfsk_ft8_tones_to_i16(
        itone,
        /* f0_hz */ 1500.0f,
        /* amplitude_i16 */ 16384,
        slot + prepend_silence,
        synth_len);
    if (st != MFSK_STATUS_OK) {
        fprintf(stderr, "tones_to_i16 failed: %d\n", (int)st); return 2;
    }
    printf("synthesised %zu samples at 1500 Hz, peak %u\n",
           synth_len, 16384);

    /* 4. Decode the synthesised slot. */
    MfskDecodeOptions *options = mfsk_ft8_options_new(
        200.0f, 3000.0f, 1.0f, 30, MFSK_DECODE_DEPTH_BP_ALL_OSD);
    MfskResultList results = {0};
    st = mfsk_ft8_decode_i16(slot, SLOT_SAMPLES, options, &results);
    mfsk_ft8_options_free(options);
    free(slot);
    if (st != MFSK_STATUS_OK) {
        fprintf(stderr, "decode failed: %d\n", (int)st); return 2;
    }

    printf("decoded %zu message(s):\n", results.len);
    int round_trip_ok = 0;
    for (size_t i = 0; i < results.len; i++) {
        const MfskResult *r = &results.items[i];
        printf("  [%zu] %.0f Hz  SNR=%+.0f dB  '%s'\n",
               i, (double)r->freq_hz, (double)r->snr_db, r->text);
        if (strcmp(r->text, "CQ JA1ABC PM86") == 0) round_trip_ok = 1;
    }

    mfsk_ft8_result_list_free(&results);

    if (!round_trip_ok) {
        fprintf(stderr, "ROUND-TRIP FAIL: expected 'CQ JA1ABC PM86' not in results\n");
        return 1;
    }
    printf("ROUND-TRIP OK\n");
    return 0;
}
