<script lang="ts">
  import type { ChannelSetting, DecodeRow, ModeInfo } from './types';
  import { BIG_PX, THUMB_PX, paint, spanMs, type WaterfallStore } from './waterfall';

  let {
    store,
    tick,
    channels,
    server,
    servers,
    onserver,
    focus,
    onfocus,
    rows,
    slotS,
    geom,
    open = $bindable(true),
  }: {
    store: WaterfallStore;
    /** Bumped whenever the store has new rows. */
    tick: number;
    channels: ChannelSetting[];
    /** The server whose channels are shown as thumbnails. */
    server: number;
    /** Every server, for the tabs (shown when there are several). */
    servers: { name: string; state: string; step: string; held: boolean }[];
    onserver: (i: number) => void;
    focus: number;
    onfocus: (i: number) => void;
    rows: DecodeRow[];
    slotS: Record<string, number>;
    geom: Record<string, ModeInfo>;
    open: boolean;
  } = $props();

  let thumbs: HTMLCanvasElement[] = $state([]);
  let big: HTMLCanvasElement | undefined = $state();
  let over: HTMLCanvasElement | undefined = $state();
  let hover = $state('');
  let hoverAt = $state({ x: 0, y: 0 });
  let scheduled = false;
  /** The large waterfall holds rows of the focused channel (re-read whenever rows arrive). */
  const ready = $derived(tick >= 0 && store.big.channel === focus && store.big.rows.length > 0);

  /** A decode's box on the large waterfall, in canvas pixels. */
  type Box = { x0: number; x1: number; y0: number; y1: number; text: string };
  let boxes: Box[] = [];

  const hhmmss = (ms: number) => {
    const d = new Date(ms);
    const z = (n: number) => String(n).padStart(2, '0');
    return `${z(d.getUTCHours())}${z(d.getUTCMinutes())}${z(d.getUTCSeconds())}`;
  };

  function draw() {
    scheduled = false;
    if (!open) return;
    // Thumbnails.
    channels.forEach((_, i) => {
      const cv = thumbs[i];
      const s = store.thumbs.get(i);
      if (!cv || !s || s.rows.length === 0) return;
      const w = s.rows[0].length;
      if (cv.width !== w) cv.width = w;
      if (cv.height !== THUMB_PX) cv.height = THUMB_PX;
      const ctx = cv.getContext('2d');
      if (!ctx) return;
      const img = ctx.createImageData(w, THUMB_PX);
      paint(img, s, spanMs(slotS[channels[i].mode] ?? 15));
      ctx.putImageData(img, 0, 0);
    });
    // The large one.
    const b = store.big;
    if (big && b.channel === focus && b.rows.length > 0) {
      const w = b.rows[0].length;
      if (big.width !== w || big.height !== BIG_PX) {
        big.width = w;
        big.height = BIG_PX;
      }
      const ctx = big.getContext('2d');
      if (ctx) {
        const img = ctx.createImageData(w, BIG_PX);
        paint(img, b, spanMs(slotS[channels[focus]?.mode ?? ''] ?? 15));
        ctx.putImageData(img, 0, 0);
      }
    }
    drawOverlay();
  }

  function drawOverlay() {
    const cv = over;
    const b = store.big;
    if (!cv || !big) return;
    const W = big.clientWidth;
    const H = big.clientHeight;
    if (cv.width !== W) cv.width = W;
    if (cv.height !== H) cv.height = H;
    const ctx = cv.getContext('2d');
    if (!ctx) return;
    ctx.clearRect(0, 0, W, H);
    boxes = [];
    if (b.channel !== focus || b.rows.length < 1) return;
    const hzSpan = b.binHz * b.rows[0].length;
    const xOf = (hz: number) => ((hz - b.fLo) / hzSpan) * W;
    const n = b.rows.length;
    const newest = b.utc[0];
    const span = spanMs(slotS[channels[focus]?.mode ?? ''] ?? 15);
    const yOf = (utc: number) => ((newest - utc) / span) * H;
    ctx.font = '10px sans-serif';
    ctx.textBaseline = 'top';
    // Frequency ticks every 500 Hz.
    ctx.strokeStyle = 'rgba(255,255,255,0.35)';
    ctx.fillStyle = 'rgba(255,255,255,0.8)';
    for (let f = Math.ceil(b.fLo / 500) * 500; f < b.fLo + hzSpan; f += 500) {
      const x = xOf(f);
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, 6);
      ctx.stroke();
      ctx.fillText(String(f), x + 2, 1);
    }
    // Slot boundaries.
    const c = channels[focus];
    const T = (slotS[c?.mode ?? ''] ?? 15) * 1000;
    ctx.strokeStyle = 'rgba(255,255,255,0.45)';
    for (let t = Math.floor(newest / T) * T; t > newest - span; t -= T) {
      const y = yOf(t);
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(W, y);
      ctx.stroke();
      ctx.fillText(hhmmss(t), 2, y + 1);
    }
    // The decodes of this channel, over the slot they came from.
    ctx.strokeStyle = 'rgba(255,255,255,0.9)';
    for (const r of rows) {
      if (r.channel !== focus || r.slotUtcMs === null || !c) continue;
      const f = r.freqHz - c.dialHz;
      const g = geom[c.mode];
      if (!g) continue;
      // The reported frequency is the lowest tone; the frame starts offsetS
      // (+ dt) after the slot start and lasts frameS.
      const t0 = r.slotUtcMs + (g.offsetS + r.dtS) * 1000;
      const yLow = yOf(t0);
      const yHigh = yOf(t0 + g.frameS * 1000);
      // Until it has left the screen entirely: below the bottom edge, not before.
      if (yLow < 0 || yHigh > H) continue;
      const x0 = xOf(f);
      const x1 = xOf(f + g.widthHz);
      ctx.strokeRect(x0, yHigh, x1 - x0, yLow - yHigh);
      boxes.push({ x0, x1, y0: yHigh, y1: yLow, text: `${r.text}  ${r.snrDb.toFixed(0)} dB` });
    }
  }

  function schedule() {
    if (scheduled) return;
    scheduled = true;
    requestAnimationFrame(draw);
  }

  $effect(() => {
    void tick;
    void focus;
    void rows.length;
    void open;
    schedule();
  });

  function onmove(e: MouseEvent) {
    const r = over?.getBoundingClientRect();
    if (!r) return;
    const x = e.clientX - r.left;
    const y = e.clientY - r.top;
    const hit = boxes.find((bx) => x >= bx.x0 && x <= bx.x1 && y >= bx.y0 && y <= bx.y1);
    hover = hit?.text ?? '';
    hoverAt = { x, y };
  }
</script>

<section class="wf">
  <header>
    <button class="link" onclick={() => (open = !open)}>{open ? '▾' : '▸'} Waterfall</button>
  </header>
  {#if open}
    {#if servers.length > 1}
      <div class="tabs" role="tablist" aria-label="Servers">
        {#each servers as sv, i (i)}
          <button role="tab" class:on={server === i} aria-selected={server === i} onclick={() => onserver(i)} title={sv.step}>
            <i class="dot {sv.state}"></i>{sv.name}
          </button>
        {/each}
      </div>
    {/if}
    <div class="thumbs">
      {#each channels as c, i (`${c.server ?? 0}:${c.mode}${c.dialHz}`)}
        {#if (c.server ?? 0) === server}
          <button class="thumb" class:on={i === focus} onclick={() => onfocus(i)} title="Show this channel large">
            <canvas bind:this={thumbs[i]} width="256" height="60"></canvas>
            <span>{c.mode} {(c.dialHz / 1000).toFixed(1)}</span>
          </button>
        {/if}
      {/each}
    </div>
    <div class="bigbox">
      <canvas class="big" bind:this={big} width="1024" height={BIG_PX}></canvas>
      <canvas class="over" bind:this={over} onmousemove={onmove} onmouseleave={() => (hover = '')}></canvas>
      {#if hover}
        <div class="tip" style="left: {hoverAt.x + 10}px; top: {hoverAt.y + 10}px">{hover}</div>
      {/if}
      {#if !ready}
        <div class="wait">waiting for {channels[focus] ? channels[focus].mode + ' ' + (channels[focus].dialHz / 1000).toFixed(1) : 'a channel'}…</div>
      {/if}
    </div>
  {/if}
</section>

<style>
  .tabs {
    display: flex;
    flex-wrap: wrap;
    gap: 2px;
    margin-bottom: 6px;
    border-bottom: 1px solid var(--line, #444);
  }
  .tabs button {
    display: inline-flex;
    align-items: center;
    gap: 6px;
    border: none;
    border-radius: 6px 6px 0 0;
    background: transparent;
    color: var(--muted);
    padding: 4px 12px;
    font-size: 12.5px;
  }
  .tabs button.on {
    background: var(--panel);
    color: var(--text);
    box-shadow: inset 0 -2px 0 var(--accent);
  }
  .wf {
    border-bottom: 1px solid var(--line, #444);
    padding: 0.25rem 0.5rem 0.5rem;
  }
  header {
    display: flex;
    align-items: center;
  }
  .thumbs {
    display: flex;
    gap: 6px;
    margin: 0.25rem 0;
  }
  .thumb {
    flex: 1 1 0;
    min-width: 0;
    padding: 2px;
    display: flex;
    flex-direction: column;
    gap: 2px;
    align-items: stretch;
  }
  .thumb.on {
    outline: 2px solid var(--accent, #4da3ff);
  }
  .thumb canvas {
    width: 100%;
    height: 54px;
    image-rendering: auto;
    background: #000;
  }
  .thumb span {
    font-size: 0.75em;
    text-align: center;
  }
  .bigbox {
    position: relative;
  }
  .big,
  .over {
    width: 100%;
    height: 220px;
    display: block;
    background: #000;
  }
  .over {
    position: absolute;
    left: 0;
    top: 0;
    background: transparent;
  }
  .tip {
    position: absolute;
    pointer-events: none;
    background: rgba(0, 0, 0, 0.8);
    color: #fff;
    padding: 2px 6px;
    border-radius: 3px;
    font-size: 0.8em;
    white-space: nowrap;
  }
  .wait {
    position: absolute;
    inset: 0;
    display: grid;
    place-items: center;
    color: #aaa;
    pointer-events: none;
  }
</style>
