<script lang="ts">
  import { geoAzimuthalEquidistant, geoMercator, geoPath } from 'd3-geo';
  import { feature } from 'topojson-client';
  import land110 from 'world-atlas/land-110m.json';
  import type { MapPoint } from './types';
  import { bandColour, gridLonLat, isDark, snrColour, sortBands, stamp } from './analysis';

  let {
    points,
    me,
    since,
    until,
    slice = $bindable(300),
    picked = null,
    onclear,
  }: {
    points: MapPoint[];
    me: string;
    since: number;
    until: number;
    /** Seconds per window of the animation; the parent re-reads the points at it. */
    slice: number;
    /** The station opened in Results: ringed and named on the map. */
    picked?: { call: string; grid: string } | null;
    onclear?: () => void;
  } = $props();

  const WINDOWS = [
    [60, '1 min'],
    [300, '5 min'],
    [900, '15 min'],
    [1800, '30 min'],
    [3600, '1 h'],
  ] as const;

  let anim = $state(false);
  let playing = $state(false);
  let fps = $state(4);
  let repeat = $state(true);
  /** Play on past windows with no station in them. */
  let skipEmpty = $state(true);
  let cur = $state(0);

  type Proj = 'azimuthal' | 'mercator';
  let proj = $state<Proj>('azimuthal');
  let paths = $state(false);
  let box: HTMLDivElement | undefined = $state();
  let cv: HTMLCanvasElement | undefined = $state();
  let size = $state({ w: 600, h: 520 });
  let tip = $state<{ x: number; y: number; text: string } | null>(null);
  type Dot = { call: string; grid: string; band: string; snr: number; count: number };
  let pts: { x: number; y: number; h: Dot }[] = [];

  const first = $derived(points.length ? points[0].t : 0);
  const t0 = $derived(Math.floor((since > 0 ? since : first) / slice) * slice);
  const frames = $derived(Math.max(1, Math.ceil((Math.max(until, t0) - t0) / slice)));
  const byT = $derived.by(() => {
    const m = new Map<number, MapPoint[]>();
    for (const p of points) (m.get(p.t) ?? m.set(p.t, []).get(p.t)!).push(p);
    return m;
  });
  /** The windows with a station in them, in order. */
  const times = $derived([...byT.keys()].sort((a, b) => a - b));
  /** Whole range: each station on each band once, with how many slices it was heard in. */
  const overall = $derived.by(() => {
    const m = new Map<string, Dot>();
    for (const p of points) {
      const k = `${p.call}|${p.band}`;
      const e = m.get(k);
      if (e) {
        e.count++;
        e.snr = Math.max(e.snr, p.snr);
      } else m.set(k, { call: p.call, grid: p.grid, band: p.band, snr: p.snr, count: 1 });
    }
    return [...m.values()];
  });
  /** The stations heard in the window at `cur`. */
  const frame = $derived<(Dot & { alpha: number })[]>(
    (byT.get(cur) ?? []).map((p) => ({ call: p.call, grid: p.grid, band: p.band, snr: p.snr, count: 1, alpha: 1 })),
  );
  const shown = $derived(anim ? frame : overall.map((d) => ({ ...d, alpha: 1 })));
  /** The bands of the whole period (not of the window shown, which would change as it plays): the
   * paths are coloured by band and the key lists them; the dots are coloured by SNR. */
  const bandsShown = $derived(sortBands(points.map((p) => p.band)));

  $effect(() => {
    // A new range or window starts the animation again.
    void t0;
    void slice;
    cur = t0;
    playing = false;
  });
  $effect(() => {
    if (!playing) return;
    const id = setInterval(() => {
      // The next window: the next with a station in it, or simply the next.
      const next = skipEmpty ? times.find((x) => x > cur) : cur + slice < t0 + frames * slice ? cur + slice : undefined;
      if (next === undefined) {
        // The end: start again, or stop.
        if (repeat) cur = skipEmpty ? (times[0] ?? t0) : t0;
        else playing = false;
        return;
      }
      cur = next;
    }, 1000 / fps);
    return () => clearInterval(id);
  });

  const land = feature(land110, land110.objects.land);
  const mine = $derived(gridLonLat(me));

  $effect(() => {
    if (!box) return;
    const ro = new ResizeObserver(() => {
      const w = box!.clientWidth;
      size = { w, h: proj === 'azimuthal' ? Math.min(w, 640) : Math.round(w * 0.55) };
    });
    ro.observe(box);
    return () => ro.disconnect();
  });

  $effect(() => {
    void shown;
    void picked;
    void proj;
    void paths;
    void bandsShown;
    void size;
    void mine;
    draw();
  });

  function draw() {
    if (!cv) return;
    const dpr = window.devicePixelRatio || 1;
    const { w, h } = size;
    cv.width = w * dpr;
    cv.height = h * dpr;
    cv.style.height = `${h}px`;
    const ctx = cv.getContext('2d');
    if (!ctx) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const dark = isDark();
    const sea = dark ? '#16202b' : '#dfeaf5';
    const ground = dark ? '#2b3641' : '#f4f1e8';
    const ink = dark ? 'rgba(255,255,255,0.35)' : 'rgba(0,0,0,0.3)';
    ctx.clearRect(0, 0, w, h);

    let p;
    if (proj === 'azimuthal' && mine) {
      const s = (Math.min(w, h) / 2 - 8) / Math.PI;
      p = geoAzimuthalEquidistant().rotate([-mine[0], -mine[1]]).scale(s).translate([w / 2, h / 2]).clipAngle(180);
    } else {
      p = geoMercator().scale(w / (2 * Math.PI)).translate([w / 2, h * 0.62]);
    }
    const path = geoPath(p, ctx);

    // The sea, then the land.
    ctx.fillStyle = sea;
    if (proj === 'azimuthal' && mine) {
      ctx.beginPath();
      ctx.arc(w / 2, h / 2, (Math.min(w, h) / 2 - 8), 0, 2 * Math.PI);
      ctx.fill();
    } else {
      ctx.fillRect(0, 0, w, h);
    }
    ctx.beginPath();
    path(land as any);
    ctx.fillStyle = ground;
    ctx.fill();
    ctx.strokeStyle = ink;
    ctx.lineWidth = 0.5;
    ctx.stroke();

    // Distance rings from home: straight great circles read directly.
    if (proj === 'azimuthal' && mine) {
      const s = (Math.min(w, h) / 2 - 8) / Math.PI;
      ctx.strokeStyle = ink;
      ctx.fillStyle = dark ? '#aab4bf' : '#55606b';
      ctx.font = '10px sans-serif';
      for (const km of [2500, 5000, 10000, 15000, 20000]) {
        const r = s * (km / 6371);
        ctx.beginPath();
        ctx.arc(w / 2, h / 2, r, 0, 2 * Math.PI);
        ctx.setLineDash([3, 4]);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillText(`${km / 1000}k`, w / 2 + 3, h / 2 - r - 2);
      }
      // North up; N, E, S, W.
      ctx.fillText('N', w / 2 - 3, 11);
    }

    // The stations.
    pts = [];
    const me2 = mine ? p(mine) : null;
    if (paths && mine) {
      ctx.lineWidth = 1;
      for (const d of shown) {
        const g = gridLonLat(d.grid);
        if (!g) continue;
        ctx.beginPath();
        path({ type: 'LineString', coordinates: [mine, g] } as any);
        // A path is always its band's colour (one meaning); the dots carry the SNR.
        ctx.strokeStyle = bandColour(d.band, 0.55 * d.alpha);
        ctx.stroke();
      }
    }
    for (const d of [...shown].sort((a, b) => a.snr - b.snr)) {
      const g = gridLonLat(d.grid);
      const xy = g ? p(g) : null;
      if (!xy) continue;
      const r = anim ? 4 : 2.5 + Math.min(4, Math.log10(1 + d.count) * 2);
      ctx.beginPath();
      ctx.arc(xy[0], xy[1], r, 0, 2 * Math.PI);
      ctx.fillStyle = snrColour(d.snr, d.alpha);
      ctx.fill();
      ctx.strokeStyle = dark ? `rgba(0,0,0,${d.alpha})` : `rgba(255,255,255,${d.alpha})`;
      ctx.lineWidth = 0.8;
      ctx.stroke();
      pts.push({ x: xy[0], y: xy[1], h: d });
    }
    if (anim) {
      ctx.font = '12px sans-serif';
      ctx.fillStyle = dark ? '#e4e7eb' : '#1c2026';
      ctx.textAlign = 'left';
      ctx.fillText(`${stamp(cur)}–${new Date((cur + slice) * 1000).toISOString().slice(11, 16)} UTC · ${
        frame.length
      } stations`, 8, h - 8);
    }
    // The station opened in Results.
    if (picked) {
      const g = gridLonLat(picked.grid);
      const xy = g ? p(g) : null;
      if (xy) {
        if (mine) {
          ctx.beginPath();
          path({ type: 'LineString', coordinates: [mine, g] } as any);
          ctx.strokeStyle = '#d9822b';
          ctx.lineWidth = 1.6;
          ctx.stroke();
        }
        ctx.beginPath();
        ctx.arc(xy[0], xy[1], 9, 0, 2 * Math.PI);
        ctx.strokeStyle = '#d9822b';
        ctx.lineWidth = 2.5;
        ctx.stroke();
        ctx.font = 'bold 12px sans-serif';
        ctx.textAlign = 'left';
        ctx.textBaseline = 'middle';
        ctx.fillStyle = dark ? '#fff' : '#000';
        ctx.fillText(picked.call, xy[0] + 13, xy[1]);
      }
    }
    if (me2) {
      ctx.beginPath();
      ctx.moveTo(me2[0], me2[1] - 7);
      ctx.lineTo(me2[0] + 6, me2[1] + 5);
      ctx.lineTo(me2[0] - 6, me2[1] + 5);
      ctx.closePath();
      ctx.fillStyle = dark ? '#fff' : '#000';
      ctx.fill();
    }
  }

  function move(e: MouseEvent) {
    if (!cv) return;
    const r = cv.getBoundingClientRect();
    const x = e.clientX - r.left;
    const y = e.clientY - r.top;
    let best: (typeof pts)[number] | null = null;
    let bd = 100;
    for (const q of pts) {
      const d = (q.x - x) ** 2 + (q.y - y) ** 2;
      if (d < bd) {
        bd = d;
        best = q;
      }
    }
    tip = best
      ? {
          x: x + 12,
          y: y + 12,
          text: `${best.h.call} ${best.h.grid} · ${best.h.band} · ${best.h.snr} dB${anim ? '' : ` · ${best.h.count} slice${best.h.count > 1 ? 's' : ''}`}`,
        }
      : null;
  }
</script>

<div class="map" bind:this={box}>
  <div class="bar">
    <label><input type="radio" bind:group={proj} value="azimuthal" /> Great-circle (centred on home)</label>
    <label><input type="radio" bind:group={proj} value="mercator" /> Mercator</label>
    <label><input type="checkbox" bind:checked={paths} /> paths</label>
    {#if picked}
      <button type="button" class="picked" title="Opened in Results; click to clear" onclick={() => onclear?.()}>Selected: {picked.call} ✕</button>
    {/if}
    <label class="anim"><input type="checkbox" bind:checked={anim} /> Animate</label>
  </div>
  {#if proj === 'azimuthal' && !mine}
    <p class="hint">Enter your grid in Settings > Station for the great-circle map. Showing Mercator.</p>
  {/if}
  {#if anim}
    <div class="bar">
      <button type="button" onclick={() => (playing = !playing)}>{playing ? '⏸ Pause' : '▶ Play'}</button>
      <label>
        Window
        <select bind:value={slice}>
          {#each WINDOWS as [v, l] (v)}<option value={v}>{l}</option>{/each}
        </select>
      </label>
      <label>
        <span title="Windows shown per second">Speed</span>
        <select bind:value={fps} title="Windows shown per second">
          {#each [1, 2, 4, 8, 16] as f (f)}<option value={f}>{f}</option>{/each}
        </select>
      </label>
      <label><input type="checkbox" bind:checked={repeat} /> repeat</label>
      <label title="Play on past the windows in which no station was heard"><input type="checkbox" bind:checked={skipEmpty} /> skip empty windows</label>
      <input
        class="seek"
        type="range"
        min={t0}
        max={t0 + (frames - 1) * slice}
        step={slice}
        bind:value={cur}
        oninput={() => (playing = false)}
      />
    </div>
  {/if}
  <!-- The key sits on its own row above the map and always takes its height: nothing below moves. -->
  <div class="key">
    <span class="snrkey"><i style="background:{snrColour(-24)}"></i>−24 dB <i style="background:{snrColour(-7)}"></i>−7 <i style="background:{snrColour(10)}"></i>+10 dB · size = decodes</span>
    <span class="bandkey" class:off={!paths || bandsShown.length === 0} title="Path colour by band">
      {#if paths}
        {#each bandsShown as b (b)}<span class="bl"><i style="background:{bandColour(b)}"></i>{b}</span>{/each}
      {/if}
    </span>
  </div>
  <div class="stage">
    <canvas bind:this={cv} onmousemove={move} onmouseleave={() => (tip = null)}></canvas>
    {#if tip}<div class="tip" style="left:{tip.x}px;top:{tip.y}px">{tip.text}</div>{/if}
  </div>
</div>

<style>
  .map {
    width: 100%;
  }
  .bar {
    display: flex;
    flex-wrap: wrap;
    gap: 4px 14px;
    align-items: center;
    padding: 4px 0 8px;
    font-size: 12px;
  }
  .picked {
    padding: 1px 9px;
    border-radius: 12px;
    font-size: 12px;
    background: #d9822b;
    color: #fff;
  }
  .bl i {
    display: inline-block;
    width: 14px;
    height: 3px;
    margin-right: 3px;
    vertical-align: middle;
  }
  .seek {
    flex: 1;
    min-width: 200px;
  }
  .key {
    display: flex;
    flex-wrap: nowrap;
    align-items: center;
    justify-content: space-between;
    gap: 14px;
    min-height: 20px;
    margin-bottom: 4px;
    font-size: 11.5px;
    color: var(--muted);
    overflow: hidden;
    white-space: nowrap;
  }
  .bandkey {
    display: inline-flex;
    gap: 2px 10px;
    min-width: 0;
  }
  .bandkey.off {
    visibility: hidden;
  }
  .snrkey i {
    display: inline-block;
    width: 10px;
    height: 10px;
    border-radius: 50%;
    margin: 0 3px 0 8px;
    vertical-align: -1px;
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
  .hint {
    color: var(--muted);
    margin: 0 0 6px;
    font-size: 12px;
  }
</style>
