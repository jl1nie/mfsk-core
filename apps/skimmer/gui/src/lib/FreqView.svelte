<script lang="ts">
  import * as api from './api';
  import { isDark } from './analysis';

  let { dir, band, since, until }: { dir: string; band: string | null; since: number; until: number } = $props();
  let mode = $state<'FT8' | 'FT4'>('FT8');
  let bins = $state<[number, number][]>([]);
  let error = $state('');
  let cv: HTMLCanvasElement | undefined = $state();

  const BIN = 25;
  const TOP = 3000;
  /** A transmission of FT8 is 50 Hz wide, FT4 90 Hz; a gap needs a guard bin each side. */
  const need = $derived(mode === 'FT8' ? 50 : 90);

  $effect(() => {
    void since;
    void until;
    void band;
    void mode;
    void load();
  });

  async function load() {
    if (!band) {
      bins = [];
      return;
    }
    try {
      error = '';
      bins = await api.dbOccupancy(dir, band, mode, BIN, since, until);
    } catch (e) {
      error = String(e);
    }
  }

  const counts = $derived.by(() => {
    const a = new Array(TOP / BIN).fill(0);
    for (const [hz, n] of bins) {
      const i = Math.floor(hz / BIN);
      if (i >= 0 && i < a.length) a[i] = n;
    }
    return a;
  });

  /** Runs of empty bins at least `need` Hz long (plus one bin of guard each side). */
  const gaps = $derived.by(() => {
    const out: { lo: number; hi: number }[] = [];
    let s = -1;
    for (let i = 0; i <= counts.length; i++) {
      const empty = i < counts.length && counts[i] === 0;
      if (empty && s < 0) s = i;
      if (!empty && s >= 0) {
        const lo = s * BIN + BIN; // guard
        const hi = i * BIN - BIN;
        if (hi - lo >= need && s * BIN >= 200) out.push({ lo, hi });
        s = -1;
      }
    }
    return out.sort((a, b) => b.hi - b.lo - (a.hi - a.lo)).slice(0, 5);
  });

  $effect(() => {
    void counts;
    void gaps;
    draw();
  });

  function draw() {
    if (!cv) return;
    const dpr = window.devicePixelRatio || 1;
    const W = cv.parentElement!.clientWidth;
    const H = 170;
    cv.width = W * dpr;
    cv.height = H * dpr;
    cv.style.height = `${H}px`;
    const ctx = cv.getContext('2d')!;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, W, H);
    const dark = isDark();
    const L = 6;
    const B = 18;
    const cw = (W - L - 6) / counts.length;
    const peak = Math.max(...counts, 1);
    ctx.fillStyle = 'rgba(70,180,110,0.25)';
    for (const g of gaps) ctx.fillRect(L + (g.lo / BIN) * cw, 0, ((g.hi - g.lo) / BIN) * cw, H - B);
    ctx.fillStyle = dark ? '#6aa0f0' : '#2563eb';
    counts.forEach((n, i) => {
      if (!n) return;
      const h = Math.max(2, Math.sqrt(n / peak) * (H - B - 6));
      ctx.fillRect(L + i * cw, H - B - h, Math.max(1, cw - 1), h);
    });
    ctx.fillStyle = dark ? '#aab4bf' : '#55606b';
    ctx.font = '10px sans-serif';
    ctx.textAlign = 'center';
    for (let hz = 0; hz <= TOP; hz += 500) ctx.fillText(String(hz), L + (hz / BIN) * cw, H - 4);
  }
</script>

<div class="freq">
  <div class="bar">
    <label><input type="radio" bind:group={mode} value="FT8" /> FT8</label>
    <label><input type="radio" bind:group={mode} value="FT4" /> FT4</label>
    <span class="hint">{band ?? 'Choose a band above'} · audio Hz, decodes per {BIN} Hz (square-root scale). Green: room for a {need} Hz transmission.</span>
  </div>
  {#if error}<p class="err">{error}</p>{/if}
  <canvas bind:this={cv}></canvas>
  {#if band && gaps.length}
    <p class="sum">
      Free: {#each gaps as g, i (g.lo)}{i ? ' · ' : ''}<b>{g.lo}–{g.hi} Hz</b> (try {Math.round((g.lo + g.hi - need) / 2 / 10) * 10}){/each}
    </p>
  {:else if band}
    <p class="hint">No free stretch of {need} Hz seen in this period. A shorter period shows the band as it is now.</p>
  {/if}
</div>

<style>
  .bar {
    display: flex;
    flex-wrap: wrap;
    gap: 4px 14px;
    align-items: center;
    padding: 4px 0 8px;
    font-size: 12px;
  }
  .hint {
    color: var(--muted);
    font-size: 12px;
  }
  .err {
    color: #d9822b;
    font-size: 12px;
  }
  .sum {
    font-size: 12.5px;
  }
  canvas {
    width: 100%;
    display: block;
  }
</style>
