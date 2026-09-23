/* Deterministic replacement for WSJT-X's lib/sgran.c.
 *
 * WHY THIS EXISTS
 *
 * `gran_()` (lib/gran.c) draws its Gaussian noise from the C library's
 * `rand()`. Upstream's `sgran_()` seeds that generator from /dev/urandom
 * or the wall clock (lib/init_random_seed.c), so every run of `ft8sim`,
 * `ft4sim`, `jt9sim` or `wsprsim` lays down a different noise realisation
 * under the same file name -- and a corpus generated on one machine is
 * therefore not the corpus generated on another.
 *
 * That cost a full investigation on 2026-09-23: `sweep-baseline.json`
 * crossings measured on one box disagreed with the same code re-measured
 * on another by up to 0.68 dB, with the decoder proven byte-identical.
 * The two generators that *did* transfer exactly were `fst4sim` and
 * `q65sim` -- and the only thing separating them is this call.
 * `fst4sim.f90:109` is literally `!   call sgran()`, commented out
 * upstream, and `q65sim.f90` never had it. So a deterministic simulator
 * is upstream's own precedent, not a divergence invented here.
 *
 * WHAT IT DOES
 *
 * Seeds `rand()` from `MFSK_SIM_SEED` (default 1, which is also what the
 * C standard says an unseeded `rand()` uses). A corpus is then a pure
 * function of (simulator binary, arguments, seed), so it regenerates
 * identically anywhere and a baseline travels with it.
 *
 * Determinism does not shrink the sampling error of a 50%-crossing
 * estimate -- with 20 trials per cell that error is sd 0.08-0.27 dB per
 * channel (measured on FT4, five independent corpora). It makes that
 * error *the same everywhere*, which is the property a shared baseline
 * needs. To average it down instead, generate several corpora with
 * different `MFSK_SIM_SEED` values into separate out-dirs.
 *
 * DIVERGENCE FROM WSJT-X: deliberate, and confined to the test-fixture
 * generators. No decoder path links this file.
 */

#include <stdlib.h>

void sgran_(void)
{
  const char *s = getenv("MFSK_SIM_SEED");
  unsigned long seed = 1UL;

  if (s != NULL && *s != '\0') {
    seed = strtoul(s, NULL, 10);
  }
  srand((unsigned) seed);
}
