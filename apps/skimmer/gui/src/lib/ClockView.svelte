<script lang="ts">
  import * as api from './api';
  import type { DtPoint } from './types';
  import { hhmm, isDark, ymd } from './analysis';

  let { dir, since, until }: { dir: string; since: number; until: number } = $props();
  let pts = $state<DtPoint[]>([]);
  let error = $state('');
  let cv: HTMLCanvasElement | undefined = $state();

  $effect(() => {
    void since;
    void until;
    void load();
  });

  let seq = 0;
  async function load() {
    const mine = ++seq;
    const span = until - Math.max(since, until - 90 * 86400);
    const bucket = Math.max(60, Math.round(span / 240 / 60) * 60);
    try {
      error = '';
      const r = await api.dbDt(dir, bucket, Math.max(since, until - 90 * 86400), until);
      if (mine === seq) pts = r;
    } catch (e) {
      error = String(e);
    }
  }

  const all = $derived(pts.length ? [...pts.map((p) => p.medianS)].sort((a, b) => a - b)[Math.floor(pts.length / 2)] : null);
  const last = $derived(pts.length ? pts[pts.length - 1] : null);

  $effect(() => {
    void pts;
    draw();
  });

  function draw() {
    if (!cv) return;
    const dpr = window.devicePixelRatio || 1;
    const W = cv.parentElement!.clientWidth;
    const H = 190;
    cv.width = W * dpr;
    cv.height = H * dpr;
    cv.style.height = `${H}px`;
    const ctx = cv.getContext('2d')!;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, W, H);
    const dark = isDark();
    const L = 40;
    const R = 8;
    const T = 8;
    const B = 18;
    const lim = 1; // seconds either side of zero
    const y = (s: number) => T + ((lim - Math.max(-lim, Math.min(lim, s))) / (2 * lim)) * (H - T - B);
    ctx.font = '10px sans-serif';
    ctx.fillStyle = dark ? '#aab4bf' : '#55606b';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (const s of [-1, -0.5, 0, 0.5, 1]) {
      ctx.strokeStyle = s === 0 ? (dark ? '#8b95a1' : '#687280') : dark ? '#2d333a' : '#dde1e6';
      ctx.beginPath();
      ctx.moveTo(L, y(s));
      ctx.lineTo(W - R, y(s));
      ctx.stroke();
      ctx.fillText(`${s > 0 ? '+' : ''}${s.toFixed(1)}`, L - 5, y(s));
    }
    if (pts.length < 2) return;
    const t0 = pts[0].t;
    const t1 = pts[pts.length - 1].t;
    const x = (t: number) => L + ((t - t0) / Math.max(1, t1 - t0)) * (W - L - R);
    ctx.strokeStyle = dark ? '#6aa0f0' : '#2563eb';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    pts.forEach((p, i) => (i ? ctx.lineTo(x(p.t), y(p.medianS)) : ctx.moveTo(x(p.t), y(p.medianS))));
    ctx.stroke();
    ctx.textAlign = 'center';
    ctx.textBaseline = 'alphabetic';
    ctx.fillStyle = dark ? '#aab4bf' : '#55606b';
    ctx.fillText(`${ymd(t0).slice(5)} ${hhmm(t0)}`, L + 28, H - 4);
    ctx.fillText(`${ymd(t1).slice(5)} ${hhmm(t1)} UTC`, W - R - 44, H - 4);
  }
</script>

<div class="clock">
  <p class="hint">
    Median DT of FT8/FT4 stations at −12 dB or stronger. Every station's DT carries its own clock error, so the median of many
    is this skimmer's: it moves with the PC clock, the NTP offset and the network delay.
  </p>
  {#if error}<p class="err">{error}</p>{/if}
  {#if last}
    <p class="sum">
      Latest <b>{last.medianS >= 0 ? '+' : ''}{last.medianS.toFixed(2)} s</b> ({last.n} stations) · median over the period
      <b>{all! >= 0 ? '+' : ''}{all!.toFixed(2)} s</b>
      {#if Math.abs(all!) > 0.3}
        <br />Clearly away from 0: the cause is likely here, not in the stations. A positive median means a common delay
        of that size: try Network delay {Math.round(all! * 1000)} ms in Settings and watch it return to 0.
      {/if}
    </p>
  {:else}
    <p class="hint">Not enough FT8 or FT4 decodes in this period.</p>
  {/if}
  <canvas bind:this={cv}></canvas>
</div>

<style>
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
    line-height: 1.6;
  }
  canvas {
    width: 100%;
    display: block;
  }
</style>
