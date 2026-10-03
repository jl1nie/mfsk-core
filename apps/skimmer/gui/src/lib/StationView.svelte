<script lang="ts">
  import * as api from './api';
  import type { Presence } from './types';
  import { heat, hhmm, isDark, ymd } from './analysis';

  let { dir, since, until }: { dir: string; since: number; until: number } = $props();
  let call = $state('');
  let suggestions = $state<[string, number][]>([]);
  let rows = $state<Presence[]>([]);
  let error = $state('');
  let loaded = $state('');
  let cv: HTMLCanvasElement | undefined = $state();
  let tip = $state<{ x: number; y: number; text: string } | null>(null);

  async function suggest() {
    const p = call.trim();
    suggestions = p.length >= 2 ? await api.dbCalls(dir, p).catch(() => []) : [];
  }

  async function load(c = call) {
    const q = c.trim().toUpperCase();
    if (!q) return;
    try {
      error = '';
      rows = await api.dbPresence(dir, q, since, until);
      loaded = q;
      suggestions = [];
    } catch (e) {
      error = String(e);
    }
  }

  // Re-read when the period changes.
  $effect(() => {
    void since;
    void until;
    if (loaded) void load(loaded);
  });

  const firstDay = $derived(rows.length ? Math.floor(rows[0].hour / 24) : 0);
  const lastDay = $derived(Math.max(firstDay, rows.length ? Math.floor(rows[rows.length - 1].hour / 24) : 0));
  const nDays = $derived(lastDay - firstDay + 1);
  const byCell = $derived.by(() => {
    const m = new Map<number, { count: number; bands: Set<string>; snr: number }>();
    for (const r of rows) {
      const e = m.get(r.hour) ?? { count: 0, bands: new Set(), snr: -99 };
      e.count += r.count;
      e.bands.add(r.band);
      e.snr = Math.max(e.snr, r.bestSnr);
      m.set(r.hour, e);
    }
    return m;
  });
  /** UTC hours of the day in which the station was heard on the most days. */
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
  const bandCounts = $derived.by(() => {
    const m = new Map<string, number>();
    for (const r of rows) m.set(r.band, (m.get(r.band) ?? 0) + r.count);
    return [...m].sort((a, b) => b[1] - a[1]);
  });
  const best = $derived(rows.length ? Math.max(...rows.map((r) => r.bestSnr)) : 0);
  const total = $derived(rows.reduce((s, r) => s + r.count, 0));

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
          text: `${ymd(day * 86400)} ${String(h).padStart(2, '0')}h · ${c.count}× · ${[...c.bands].join(', ')} · best ${c.snr} dB`,
        }
      : null;
  }
</script>

<div class="station">
  <form
    onsubmit={(e) => {
      e.preventDefault();
      void load();
    }}
  >
    <input
      bind:value={call}
      oninput={suggest}
      placeholder="Callsign, e.g. VK3NV"
      spellcheck="false"
      autocomplete="off"
      list="calls"
    />
    <datalist id="calls">
      {#each suggestions as [c, n] (c)}<option value={c}>{n}×</option>{/each}
    </datalist>
    <button type="submit">Show</button>
  </form>
  {#if error}<p class="err">{error}</p>{/if}
  {#if loaded && rows.length === 0}
    <p class="hint">{loaded} was not heard in this period.</p>
  {:else if rows.length}
    <p class="sum">
      <b>{loaded}</b> · {total} decodes · best {best} dB · first {ymd(rows[0].hour * 3600)}
      {hhmm(rows[0].hour * 3600)}, last {ymd(rows[rows.length - 1].hour * 3600)} {hhmm(rows[rows.length - 1].hour * 3600)} UTC
      <br />
      Bands: {bandCounts.map(([b, n]) => `${b} (${n})`).join(', ')}
      {#if usual}<br />Usually heard: {usual}{/if}
    </p>
  {:else}
    <p class="hint">Enter a call to see which UTC hours of which days it was heard in.</p>
  {/if}
  <div class="stage">
    <canvas bind:this={cv} onmousemove={move} onmouseleave={() => (tip = null)}></canvas>
    {#if tip}<div class="tip" style="left:{tip.x}px;top:{tip.y}px">{tip.text}</div>{/if}
  </div>
</div>

<style>
  form {
    display: flex;
    gap: 8px;
    padding: 4px 0 8px;
  }
  input {
    width: 200px;
  }
  .sum {
    font-size: 12.5px;
    line-height: 1.6;
    margin: 0 0 8px;
  }
  .hint,
  .err {
    font-size: 12px;
    color: var(--muted);
  }
  .err {
    color: #d9822b;
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
