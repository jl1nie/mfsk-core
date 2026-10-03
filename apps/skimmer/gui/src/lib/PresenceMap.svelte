<script lang="ts">
  import * as api from './api';
  import type { Activity, Query } from './types';
  import { heat, isDark, stamp, ymd } from './analysis';

  /** One station's hours on the air within a query: UTC hour of day across, days down. */
  let { dir, q, call }: { dir: string; q: Query; call: string } = $props();
  let rows = $state<Activity[]>([]);
  let error = $state('');
  let cv: HTMLCanvasElement | undefined = $state();
  let tip = $state<{ x: number; y: number; text: string } | null>(null);

  const escapeRe = (s: string) => s.replace(/[.*+?^${}()|[\]\\/]/g, '\\$&');

  $effect(() => {
    void q;
    void call;
    void load();
  });

  async function load() {
    try {
      error = '';
      rows = await api.dbActivity(dir, { ...q, call: `^${escapeRe(call)}$` });
    } catch (e) {
      error = String(e);
    }
  }

  const firstDay = $derived(rows.length ? Math.floor(rows[0].hour / 24) : 0);
  const lastDay = $derived(Math.max(firstDay, rows.length ? Math.floor(rows[rows.length - 1].hour / 24) : 0));
  const nDays = $derived(lastDay - firstDay + 1);
  const byCell = $derived.by(() => {
    const m = new Map<number, { count: number; bands: Set<string>; snr: number }>();
    for (const r of rows) {
      const e = m.get(r.hour) ?? { count: 0, bands: new Set<string>(), snr: -99 };
      e.count += r.decodes;
      e.bands.add(r.band);
      e.snr = Math.max(e.snr, r.bestSnr);
      m.set(r.hour, e);
    }
    return m;
  });
  /** UTC hours of the day in which it was heard on most days. */
  const usual = $derived.by(() => {
    const days = Array.from({ length: 24 }, () => new Set<number>());
    for (const h of byCell.keys()) days[((h % 24) + 24) % 24].add(Math.floor(h / 24));
    const peak = Math.max(...days.map((d) => d.size));
    if (peak < 2) return '';
    const hot = days.map((d) => d.size >= Math.max(2, peak * 0.6));
    const spans: string[] = [];
    for (let h = 0; h < 24; h++) {
      if (!hot[h]) continue;
      let e = h;
      while (e + 1 < 24 && hot[e + 1]) e++;
      spans.push(`${String(h).padStart(2, '0')}–${String(e + 1).padStart(2, '0')}`);
      h = e;
    }
    return `${spans.join(', ')} UTC (${peak} of ${nDays} days at the busiest hour)`;
  });

  const L = 70;
  const ROW = 14;
  $effect(() => {
    void byCell;
    draw();
  });

  function draw() {
    if (!cv) return;
    const dpr = window.devicePixelRatio || 1;
    const W = cv.parentElement!.clientWidth;
    const H = 18 + nDays * ROW + 4;
    cv.width = W * dpr;
    cv.height = H * dpr;
    cv.style.height = `${H}px`;
    const ctx = cv.getContext('2d')!;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, W, H);
    if (!rows.length) return;
    const dark = isDark();
    const peak = Math.max(...[...byCell.values()].map((c) => c.count), 1);
    const cw = (W - L) / 24;
    ctx.font = '10px sans-serif';
    ctx.fillStyle = dark ? '#aab4bf' : '#55606b';
    ctx.textBaseline = 'middle';
    ctx.textAlign = 'center';
    for (let h = 0; h < 24; h += 3) ctx.fillText(String(h).padStart(2, '0'), L + (h + 0.5) * cw, 8);
    ctx.textAlign = 'right';
    for (let d = 0; d < nDays; d++) {
      const day = lastDay - d; // newest on top
      const y = 18 + d * ROW;
      ctx.fillStyle = dark ? '#aab4bf' : '#55606b';
      if (nDays <= 40 || d % 5 === 0) ctx.fillText(ymd(day * 86400).slice(5), L - 6, y + ROW / 2);
      for (let h = 0; h < 24; h++) {
        const c = byCell.get(day * 24 + h);
        ctx.fillStyle = heat(c ? Math.log10(1 + c.count) / Math.log10(1 + peak) : 0, dark);
        ctx.fillRect(L + h * cw, y, Math.ceil(cw) - 1, ROW - 1);
      }
    }
  }

  function move(e: MouseEvent) {
    if (!cv) return;
    const r = cv.getBoundingClientRect();
    const x = e.clientX - r.left;
    const y = e.clientY - r.top;
    const cw = (r.width - L) / 24;
    const h = Math.floor((x - L) / cw);
    const d = Math.floor((y - 18) / ROW);
    if (h < 0 || h > 23 || d < 0 || d >= nDays) {
      tip = null;
      return;
    }
    const day = lastDay - d;
    const c = byCell.get(day * 24 + h);
    tip = c
      ? {
          x: x + 12,
          y: y + 12,
          text: `${stamp((day * 24 + h) * 3600)} UTC · ${c.count}× · ${[...c.bands].join(', ')} · best ${c.snr} dB`,
        }
      : null;
  }
</script>

<div class="presence">
  {#if error}<p class="err">{error}</p>{/if}
  {#if usual}<p class="sum">Usually heard: {usual}</p>{/if}
  <div class="stage">
    <canvas bind:this={cv} onmousemove={move} onmouseleave={() => (tip = null)}></canvas>
    {#if tip}<div class="tip" style="left:{tip.x}px;top:{tip.y}px">{tip.text}</div>{/if}
  </div>
</div>

<style>
  .sum {
    font-size: 12.5px;
    margin: 4px 0 8px;
  }
  .err {
    color: #d9822b;
    font-size: 12px;
  }
  .stage {
    position: relative;
  }
  canvas {
    width: 100%;
    display: block;
  }
  .tip {
    position: absolute;
    pointer-events: none;
    background: var(--panel);
    border: 1px solid var(--line);
    padding: 3px 7px;
    font-size: 12px;
    white-space: nowrap;
    border-radius: 4px;
  }
</style>
