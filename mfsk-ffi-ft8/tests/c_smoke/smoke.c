/* C-side smoke test for mfsk-ffi-ft8.
 *
 * Loads a 12 kHz / mono / i16 PCM WAV (no need for a real WAV
 * parser — we just `fseek` past the 44-byte RIFF/fmt header) and
 * calls `mfsk_ft8_decode_i16`. Prints the decoded messages.
 *
 * Build (host, default features):
 *   cargo build -p mfsk-ffi-ft8 --release
 *   gcc -O2 -I mfsk-ffi-ft8/include \
 *       mfsk-ffi-ft8/tests/c_smoke/smoke.c \
 *       -L target/release -lmfsk_ft8 -lm -lpthread -ldl \
 *       -Wl,-rpath,$PWD/target/release \
 *       -o /tmp/mfsk_smoke
 *   /tmp/mfsk_smoke embedded-poc/m5stack-core2/assets/qso1.wav
 */

#include "mfsk_ft8.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: %s <wav-12khz-mono-i16>\n", argv[0]);
        return 2;
    }
    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror(argv[1]); return 2; }
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    if (fsize < 44 + 168000 * 2) {
        fprintf(stderr, "%s: file too short (%ld bytes)\n", argv[1], fsize);
        return 2;
    }
    fseek(f, 44, SEEK_SET);
    size_t n_samples = (fsize - 44) / 2;
    int16_t *audio = malloc(n_samples * 2);
    if (!audio) { fprintf(stderr, "oom\n"); return 2; }
    if (fread(audio, 2, n_samples, f) != n_samples) {
        fprintf(stderr, "short read\n"); return 2;
    }
    fclose(f);

    MfskDecodeOptions *options = mfsk_ft8_options_new(
        /* freq_min */ 200.0f,
        /* freq_max */ 3000.0f,
        /* sync_min */ 1.0f,
        /* max_cand */ 100,
        MFSK_DECODE_DEPTH_BP_ALL_OSD);

    MfskResultList results = {0};
    MfskStatus st = mfsk_ft8_decode_i16(audio, n_samples, options, &results);

    printf("== %s ==\n", argv[1]);
    printf("status=%d, found %zu message(s):\n", (int)st, results.len);
    for (size_t i = 0; i < results.len; i++) {
        const MfskResult *r = &results.items[i];
        printf("  [%zu] %5.0f Hz  dt=%+.2f s  SNR=%+.0f dB  e=%u  '%s'\n",
               i, (double)r->freq_hz, (double)r->dt_sec,
               (double)r->snr_db, r->hard_errors, r->text);
    }

    mfsk_ft8_result_list_free(&results);
    mfsk_ft8_options_free(options);
    free(audio);
    return (int)st;
}
