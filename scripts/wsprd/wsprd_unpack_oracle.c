/* Oracle for mfsk-core's msg::wspr::unpack and the WSPR decoder's message text: WSJT-X's own
 * `unpk_` (lib/wsprd/wsprd_utils.c, v3.2.0-rc1), called the way lib/wsprd/wsprd.c:1496 calls it.
 *
 *   wsprd_unpack_oracle < cases        one case per line:  <50 bits '0'/'1'> TAB <seed call | ->
 *
 * The seed call is entered into wsprd's hash table the way a Type 1 / Type 2 decode does
 * (wsprd_utils.c:353-354, 367-368: hashtab[nhash(call, strlen(call), 146)] = call), so a Type 3
 * with that hash resolves; `-` leaves the table empty (a hash never heard).
 *
 * Output, one line per case:   rc TAB text
 *   rc   = unpk_'s return value, wsprd's `noprint` (wsprd.c:1496): 1 means the decode is not
 *          printed, not written to ALL_WSPR.TXT, not subtracted; the text is then `-`.
 *   text = `call_loc_pow`, the message field wsprd prints (wsprd.c:1535, 1553: `%-s`), verbatim.
 * The buffers are sized and cleared as wsprd.c:808-810, 1373-1375 does.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "wsprd_utils.h"

int main(void) {
    char *hashtab = calloc(WSPRD_HASH_COUNT * WSPRD_CALLSIGN_SIZE, 1);
    char *loctab = calloc(WSPRD_HASH_COUNT * WSPRD_GRID4_SIZE, 1);
    char line[512];
    while (fgets(line, sizeof line, stdin)) {
        line[strcspn(line, "\r\n")] = 0;
        char *tab = strchr(line, '\t');
        if (!tab || tab - line != 50) { fprintf(stderr, "bad line: %s\n", line); return 2; }
        *tab = 0;
        const char *seed = tab + 1;
        memset(hashtab, 0, WSPRD_HASH_COUNT * WSPRD_CALLSIGN_SIZE);
        memset(loctab, 0, WSPRD_HASH_COUNT * WSPRD_GRID4_SIZE);
        /* nhash of an empty string is unmasked (nhash.c:9); wsprd's store_hash_call refuses it. */
        if (strcmp(seed, "-") != 0 && *seed) {
            int ih = (int)nhash(seed, strlen(seed), (uint32_t)146);
            snprintf(hashtab + ih * WSPRD_CALLSIGN_SIZE, WSPRD_CALLSIGN_SIZE, "%s", seed);
        }
        unsigned char data[11] = {0};
        for (int i = 0; i < 50; i++)
            if (line[i] == '1') data[i / 8] |= 1 << (7 - i % 8);
        signed char message[11];
        for (int i = 0; i < 11; i++) message[i] = data[i] > 127 ? data[i] - 256 : data[i];
        char callsign[13], call_loc_pow[23];
        memset(callsign, 0, sizeof callsign);
        memset(call_loc_pow, 0, sizeof call_loc_pow);
        int noprint = unpk_(message, hashtab, loctab, call_loc_pow, callsign);
        printf("%d\t%s\n", noprint, noprint ? "-" : call_loc_pow);
    }
    return 0;
}
