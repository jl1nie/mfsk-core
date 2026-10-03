<script lang="ts">
  import { geoAzimuthalEquidistant, geoMercator, geoPath } from 'd3-geo';
  import { feature } from 'topojson-client';
  import land110 from 'world-atlas/land-110m.json';
  import type { Heard } from './types';
  import { gridLonLat, isDark, snrColour } from './analysis';

  let { heard, me }: { heard: Heard[]; me: string } = $props();

  type Proj = 'azimuthal' | 'mercator';
  let proj = $state<Proj>('azimuthal');
  let paths = $state(false);
  let box: HTMLDivElement | undefined = $state();
  let cv: HTMLCanvasElement | undefined = $state();
  let size = $state({ w: 600, h: 520 });
  let tip = $state<{ x: number; y: number; text: string } | null>(null);
  let pts: { x: number; y: number; h: Heard }[] = [];

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
    void heard;
    void proj;
    void paths;
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
      ctx.lineWidth = 0.6;
      for (const s of heard) {
        const g = gridLonLat(s.grid);
        if (!g) continue;
        ctx.beginPath();
        path({ type: 'LineString', coordinates: [mine, g] } as any);
        ctx.strokeStyle = snrColour(s.bestSnr).replace(')', ' / 0.35)');
        ctx.stroke();
      }
    }
    for (const s of [...heard].sort((a, b) => a.bestSnr - b.bestSnr)) {
      const g = gridLonLat(s.grid);
      const xy = g ? p(g) : null;
      if (!xy) continue;
      const r = 2.5 + Math.min(4, Math.log10(1 + s.count) * 2);
      ctx.beginPath();
      ctx.arc(xy[0], xy[1], r, 0, 2 * Math.PI);
      ctx.fillStyle = snrColour(s.bestSnr);
      ctx.fill();
      ctx.strokeStyle = dark ? '#000' : '#fff';
      ctx.lineWidth = 0.8;
      ctx.stroke();
      pts.push({ x: xy[0], y: xy[1], h: s });
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
          text: `${best.h.call} ${best.h.grid} · ${best.h.band} · ${best.h.count}× · best ${best.h.bestSnr} dB${
            best.h.km != null ? ` · ${Math.round(best.h.km)} km @ ${Math.round(best.h.bearing ?? 0)}°` : ''
          }`,
        }
      : null;
  }
</script>

<div class="map" bind:this={box}>
  <div class="bar">
    <label><input type="radio" bind:group={proj} value="azimuthal" /> Great-circle (centred on home)</label>
    <label><input type="radio" bind:group={proj} value="mercator" /> Mercator</label>
    <label><input type="checkbox" bind:checked={paths} /> paths</label>
    <span class="legend"><i style="background:{snrColour(-24)}"></i>−24 dB <i style="background:{snrColour(-7)}"></i>−7 <i style="background:{snrColour(10)}"></i>+10 dB · size = decodes</span>
  </div>
  {#if proj === 'azimuthal' && !mine}
    <p class="hint">Enter your grid in Settings > Station for the great-circle map. Showing Mercator.</p>
  {/if}
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
  .legend {
    color: var(--muted);
    margin-left: auto;
  }
  .legend i {
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
