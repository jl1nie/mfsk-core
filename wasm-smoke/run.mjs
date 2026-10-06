// Runs every mode's silent decode, in the wasm build `wasm-pack build --target nodejs` left in ./pkg.
// A panic surfaces as `RuntimeError: unreachable`; each mode is caught on its own so the
// failure names the mode instead of stopping at the first.
import { createRequire } from 'node:module';
const m = createRequire(import.meta.url)('./pkg/mfsk_wasm_smoke.js');

const jobs = [...m.modes().map((n) => [n, () => m.decode_silence(n)]), ['MSK144', () => m.msk144()]];
let failed = 0;
for (const [name, run] of jobs) {
  const t = Date.now();
  try {
    const rows = run();
    console.log(`ok    ${name.padEnd(10)} rows=${rows} ${Date.now() - t} ms`);
  } catch (e) {
    failed++;
    console.log(`FAIL  ${name.padEnd(10)} ${String(e).split('\n')[0]}`);
  }
}
if (failed) {
  console.error(`${failed} of ${jobs.length} modes panicked on wasm32-unknown-unknown`);
  process.exit(1);
}
console.log(`all ${jobs.length} modes ran`);
