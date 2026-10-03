<script lang="ts">
  import type { Activity } from './types';
  import { heat, isDark, sortBands } from './analysis';

  let { activity, nowMs }: { activity: Activity[]; nowMs: number } = $props();
  let mode = $state<'timeline' | 'typical'>('typical');
  let cv: HTMLCanvasElement | undefined = $state();
  let tip = $state<{ x: number; y: number; text: string } | null>(null);

  const bands = $derived(sortBands(activity.map((a) => a.band)));
  const minHour = $derived(activity.length ? Math.min(...activity.map((a) => a.hour)) : 0);
  const maxHour = $derived(Math.max(Math.floor(nowMs / 3_600_000), activity.length ? Math.max(...activity.map((a) => a.hour)) : 0));
  const days = $derived(Math.max(1, Math.ceil((maxHour - minHour + 1) / 24)));

  /** cells[band][column] = distinct stations (mean over days for "typical"). */
  const grid = $derived.by(() => {
    const cols = mode === 'typical' ? 24 : maxHour - minHour + 1;
    const cells = new Map<string, number[]>();
    for (const b of bands) cells.set(b, new Array(cols).fill(0));
    for (const a of activity) {
      const row = cells.get(a.band)!;
      const c = mode === 'typical' ? ((a.hour % 24) + 24) % 24 : a.hour - minHour;
      row[c] += mode === 'typical' ? a.stations / days : a.stations;
    }
    return { cols, cells };
  });

  const L = 44; // left margin for the band labels
  const T = 18;
  const ROW = 22;

  $effect(() => {
    void grid;
    draw();
  });

  function draw() {
    if (!cv) return;
    const dpr = window.devicePixelRatio || 1;
    const W = cv.parentElement!.clientWidth;
    const H = T + bands.length * ROW + 4;
    cv.width = W * dpr;
    cv.height = H * dpr;
    cv.style.height = `${H}px`;
    const ctx = cv.getContext('2d')!;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, W, H);
    const dark = isDark();
    const text = dark ? '#aab4bf' : '#55606b';
    ctx.font = '11px sans-serif';
    ctx.textBaseline = 'middle';
    const cw = (W - L) / grid.cols;
    bands.forEach((b, i) => {
      const row = grid.cells.get(b)!;
      const peak = Math.max(...row, 1e-9);
      ctx.fillStyle = text;
      ctx.textAlign = 'right';
      ctx.fillText(b, L - 6, T + i * ROW + ROW / 2);
      row.forEach((v, c) => {
        ctx.fillStyle = heat(v / peak, dark);
        ctx.fillRect(L + c * cw, T + i * ROW, Math.ceil(cw), ROW - 2);
      });
    });
    ctx.fillStyle = text;
    ctx.textAlign = 'center';
    if (mode === 'typical') {
      for (let h = 0; h < 24; h += 3) ctx.fillText(String(h).padStart(2, '0'), L + (h + 0.5) * cw, 8);
    } else {
      // A label at every UTC midnight, or every few days on a long range.
      const step = Math.max(1, Math.ceil(days / 8));
      for (let c = 0; c < grid.cols; c++) {
        const hour = minHour + c;
        if (hour % 24 === 0 && (hour / 24) % step === 0) {
          const d = new Date(hour * 3_600_000).toISOString().slice(5, 10);
          ctx.fillText(d, L + c * cw, 8);
          ctx.fillStyle = dark ? 'rgba(255,255,255,.25)' : 'rgba(0,0,0,.25)';
          ctx.fillRect(L + c * cw, T - 4, 1, bands.length * ROW + 4);
          ctx.fillStyle = text;
        }
      }
    }
    // Now.
    if (mode === 'typical') {
      const c = Math.floor((nowMs / 3_600_000) % 24);
      ctx.strokeStyle = '#d9822b';
      ctx.lineWidth = 1.5;
      ctx.strokeRect(L + c * cw, T - 2, cw, bands.length * ROW + 2);
    }
  }

  function move(e: MouseEvent) {
    if (!cv) return;
    const r = cv.getBoundingClientRect();
    const x = e.clientX - r.left;
    const y = e.clientY - r.top;
    const cw = (r.width - L) / grid.cols;
    const c = Math.floor((x - L) / cw);
    const i = Math.floor((y - T) / ROW);
    if (c < 0 || c >= grid.cols || i < 0 || i >= bands.length) {
      tip = null;
      return;
    }
    const v = grid.cells.get(bands[i])![c];
    const when =
      mode === 'typical'
        ? `${String(c).padStart(2, '0')}:00 UTC`
        : new Date((minHour + c) * 3_600_000).toISOString().slice(0, 13).replace('T', ' ') + ':00 UTC';
    tip = {
      x: x + 12,
      y: y + 12,
      text: `${bands[i]} · ${when} · ${mode === 'typical' ? v.toFixed(1) + ' stations/h on average' : v + ' stations'}`,
    };
  }
</script>

<div class="bands">
  <div class="bar">
    <label><input type="radio" bind:group={mode} value="typical" /> By UTC hour (average over {days} day{days > 1 ? 's' : ''})</label>
    <label><input type="radio" bind:group={mode} value="timeline" /> Timeline</label>
    <span class="hint">Colour is relative to each band's own busiest hour: it shows when a band opens, not how big it is.</span>
  </div>
  {#if activity.length === 0}
    <p class="hint">Nothing recorded in this period.</p>
  {/if}
  <div class="stage">
    <canvas bind:this={cv} onmousemove={move} onmouseleave={() => (tip = null)}></canvas>
    {#if tip}<div class="tip" style="left:{tip.x}px;top:{tip.y}px">{tip.text}</div>{/if}
  </div>
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
