<script lang="ts">
  import { geoAzimuthalEquidistant, geoMercator, geoPath } from 'd3-geo';
  import { feature } from 'topojson-client';
  import land110 from 'world-atlas/land-110m.json';
  import { gridLonLat, isDark } from './analysis';

  /** Home, one station and the great circle between them: where the signal came from. */
  let {
    me,
    grid,
    call,
    km,
    bearing,
  }: { me: string; grid: string; call: string; km: number | null; bearing: number | null } = $props();

  const land = feature(land110, land110.objects.land);
  const W = 190;
  const H = 150;
  let cv: HTMLCanvasElement | undefined = $state();

  $effect(() => {
    void me;
    void grid;
    draw();
  });

  function draw() {
    if (!cv) return;
    const dpr = window.devicePixelRatio || 1;
    cv.width = W * dpr;
    cv.height = H * dpr;
    const ctx = cv.getContext('2d');
    if (!ctx) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const dark = isDark();
    const mine = gridLonLat(me);
    const theirs = gridLonLat(grid);
    const sea = dark ? '#16202b' : '#dfeaf5';
    const ground = dark ? '#2b3641' : '#f4f1e8';
    const ink = dark ? 'rgba(255,255,255,0.35)' : 'rgba(0,0,0,0.3)';
    ctx.clearRect(0, 0, W, H);
    // Centred on home when it is known: straight lines from the centre are great circles.
    const round = !!mine;
    const r = Math.min(W, H) / 2 - 4;
    const p = round
      ? geoAzimuthalEquidistant().rotate([-mine![0], -mine![1]]).scale(r / Math.PI).translate([W / 2, H / 2]).clipAngle(180)
      : geoMercator().scale(W / (2 * Math.PI)).translate([W / 2, H * 0.62]);
    const path = geoPath(p, ctx);
    ctx.fillStyle = sea;
    if (round) {
      ctx.beginPath();
      ctx.arc(W / 2, H / 2, r, 0, 2 * Math.PI);
      ctx.fill();
    } else ctx.fillRect(0, 0, W, H);
    ctx.beginPath();
    path(land as any);
    ctx.fillStyle = ground;
    ctx.fill();
    ctx.strokeStyle = ink;
    ctx.lineWidth = 0.5;
    ctx.stroke();
    if (mine && theirs) {
      ctx.beginPath();
      path({ type: 'LineString', coordinates: [mine, theirs] } as any);
      ctx.strokeStyle = '#d9822b';
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }
    const dot = (g: [number, number] | null, fill: string, r2: number) => {
      const xy = g ? p(g) : null;
      if (!xy) return;
      ctx.beginPath();
      ctx.arc(xy[0], xy[1], r2, 0, 2 * Math.PI);
      ctx.fillStyle = fill;
      ctx.fill();
      ctx.strokeStyle = dark ? '#000' : '#fff';
      ctx.lineWidth = 1;
      ctx.stroke();
    };
    dot(mine, dark ? '#fff' : '#000', 3);
    dot(theirs, '#d9822b', 4.5);
  }
</script>

<div class="mini">
  <canvas bind:this={cv} style="width:{W}px;height:{H}px"></canvas>
  <div class="cap">
    <b>{call}</b>
    {grid}{#if km != null} · {Math.round(km).toLocaleString()} km @ {Math.round(bearing ?? 0)}°{/if}
  </div>
</div>

<style>
  .mini {
    display: flex;
    flex-direction: column;
    gap: 3px;
    flex: none;
  }
  canvas {
    border-radius: 6px;
    display: block;
  }
  .cap {
    font-size: 11.5px;
    color: var(--muted);
    max-width: 190px;
  }
  .cap b {
    color: var(--text);
  }
</style>
