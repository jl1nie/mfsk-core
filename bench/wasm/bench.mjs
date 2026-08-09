#!/usr/bin/env node
// Node driver for the mfsk-wasm-bench harness (issue #208).
//
// Usage:
//   node bench.mjs [runs]
//   MFSK_BENCH_WAV=/path/to/file.wav node bench.mjs [runs]
//
// Loads pkg/mfsk_wasm_bench.js (built via ./build.sh, wasm-pack --target
// nodejs) and times `decode_wav()` over a fixture WAV, printing the
// median wall-clock time of `runs` steady-state calls (default 7,
// discarding one warmup call first) plus the decode output itself so a
// human can eyeball recall alongside timing.
//
// Defaults to embedded-poc/assets/qso3_busy.wav (committed, CI-safe,
// known-correct golden decode). Point MFSK_BENCH_WAV at a local copy of
// sim_busy_band.wav / sim_extreme_hard.wav (the WAVs used for issue
// #208's own numbers — not committed to this repo, see README.md) to
// reproduce those exact figures.

import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import path from "node:path";

const __dirname = path.dirname(fileURLToPath(import.meta.url));

function parseWavMonoI16(buf) {
  if (buf.toString("ascii", 0, 4) !== "RIFF" || buf.toString("ascii", 8, 12) !== "WAVE") {
    throw new Error("not a RIFF/WAVE file");
  }
  let offset = 12;
  let fmt = null;
  let data = null;
  while (offset + 8 <= buf.length) {
    const id = buf.toString("ascii", offset, offset + 4);
    const size = buf.readUInt32LE(offset + 4);
    const body = offset + 8;
    if (id === "fmt ") {
      fmt = {
        audioFormat: buf.readUInt16LE(body),
        numChannels: buf.readUInt16LE(body + 2),
        sampleRate: buf.readUInt32LE(body + 4),
        bitsPerSample: buf.readUInt16LE(body + 14),
      };
    } else if (id === "data") {
      data = buf.subarray(body, body + size);
    }
    offset = body + size + (size % 2);
  }
  if (!fmt || !data) throw new Error("missing fmt/data chunk");
  if (fmt.audioFormat !== 1 || fmt.bitsPerSample !== 16) {
    throw new Error(`expected 16-bit PCM, got format=${fmt.audioFormat} bits=${fmt.bitsPerSample}`);
  }
  const samplesAllChannels = new Int16Array(
    data.buffer,
    data.byteOffset,
    data.byteLength / 2,
  );
  if (fmt.numChannels === 1) return samplesAllChannels;
  // Downmix to mono by taking channel 0 — golden fixtures here are mono,
  // this is just a safety net if pointed at a stereo file.
  const mono = new Int16Array(samplesAllChannels.length / fmt.numChannels);
  for (let i = 0; i < mono.length; i++) mono[i] = samplesAllChannels[i * fmt.numChannels];
  return mono;
}

function median(xs) {
  const sorted = [...xs].sort((a, b) => a - b);
  const mid = Math.floor(sorted.length / 2);
  return sorted.length % 2 ? sorted[mid] : (sorted[mid - 1] + sorted[mid]) / 2;
}

async function main() {
  const runs = Number(process.argv[2] ?? 7);
  const wavPath =
    process.env.MFSK_BENCH_WAV ??
    path.join(__dirname, "../../embedded-poc/assets/qso3_busy.wav");

  const { decode_wav, decode_wav_subtract } = await import(
    path.join(__dirname, "pkg/mfsk_wasm_bench.js")
  );

  const audio = parseWavMonoI16(readFileSync(wavPath));
  console.log(`WAV: ${wavPath} (${audio.length} samples)`);

  const variants = [
    ["decode_wav (default, no SIC)", decode_wav],
    ["decode_wav_subtract (.sic_early())", decode_wav_subtract],
  ];

  const medians = {};
  for (const [label, fn] of variants) {
    // Warmup — discarded, not steady-state.
    const warmupOut = fn(audio);
    const nStations = warmupOut ? warmupOut.split("\n").length : 0;

    const times = [];
    for (let i = 0; i < runs; i++) {
      const t0 = performance.now();
      fn(audio);
      times.push(performance.now() - t0);
    }

    const m = median(times);
    medians[label] = m;
    console.log(`\n=== ${label} ===`);
    console.log(`runs: ${times.map((t) => t.toFixed(1)).join(", ")} ms`);
    console.log(`median: ${m.toFixed(1)} ms -> ${nStations} station(s)`);
    console.log(warmupOut || "(none)");
  }

  const labels = variants.map(([label]) => label);
  console.log(
    `\nratio (${labels[1]} / ${labels[0]}): ${(medians[labels[1]] / medians[labels[0]]).toFixed(2)}x`,
  );
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
