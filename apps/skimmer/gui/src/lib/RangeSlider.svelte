<script lang="ts">
  /**
   * Two handles on one track. A handle at an end means "no limit" (null), so
   * a range open on one side needs no number typed. `log` spreads small
   * values out (distance).
   */
  let {
    lo,
    hi,
    min,
    max,
    log = false,
    unit = '',
    onchange,
    bars = [],
  }: {
    lo: number | null;
    hi: number | null;
    min: number;
    max: number;
    log?: boolean;
    unit?: string;
    onchange: (lo: number | null, hi: number | null) => void;
    /** Counts per value across min..max, drawn behind the track. */
    bars?: [number, number][];
  } = $props();

  const N = 1000;
  const toPos = (v: number) =>
    Math.round(
      (log ? Math.log10(1 + (999 * (v - min)) / (max - min)) / 3 : (v - min) / (max - min)) * N,
    );
  const nice = (v: number) => (v < 500 ? Math.round(v / 10) * 10 : v < 2000 ? Math.round(v / 50) * 50 : Math.round(v / 100) * 100);
  const fromPos = (p: number) => {
    const x = p / N;
    return log ? nice(min + ((max - min) * (10 ** (3 * x) - 1)) / 999) : Math.round(min + (max - min) * x);
  };

  let a = $state(0);
  let b = $state(N);
  let dragging = false;
  $effect(() => {
    if (dragging) return;
    a = lo === null ? 0 : Math.max(0, Math.min(N, toPos(lo)));
    b = hi === null ? N : Math.max(0, Math.min(N, toPos(hi)));
  });

  const loV = $derived(a <= 0 ? null : fromPos(a));
  const hiV = $derived(b >= N ? null : fromPos(b));
  const label = $derived(
    loV === null && hiV === null
      ? 'any'
      : `${loV === null ? '…' : loV}${unit} to ${hiV === null ? '…' : hiV}${unit}`,
  );
  const peak = $derived(Math.max(1, ...bars.map((x) => x[1])));

  function move(which: 'a' | 'b', e: Event) {
    dragging = true;
    const v = Number((e.currentTarget as HTMLInputElement).value);
    if (which === 'a') a = Math.min(v, b);
    else b = Math.max(v, a);
  }
  function done() {
    dragging = false;
    onchange(loV, hiV);
  }
</script>

<div class="rs">
  {#if bars.length}
    <svg class="bars" viewBox="0 0 {N} 30" preserveAspectRatio="none" aria-hidden="true">
      {#each bars as [v, n] (v)}
        <rect x={toPos(Math.max(min, Math.min(max, v))) - 6} width="12" y={30 - (n / peak) * 30} height={(n / peak) * 30} />
      {/each}
    </svg>
  {/if}
  <div class="track">
    <div class="fill" style="left:{a / 10}%;right:{100 - b / 10}%"></div>
    <input type="range" min="0" max={N} step="1" value={a} oninput={(e) => move('a', e)} onchange={done} aria-label="lower limit" />
    <input type="range" min="0" max={N} step="1" value={b} oninput={(e) => move('b', e)} onchange={done} aria-label="upper limit" />
  </div>
  <div class="label">{label}</div>
</div>

<style>
  .rs {
    width: 200px;
  }
  .bars {
    width: 100%;
    height: 26px;
    display: block;
    fill: var(--line);
  }
  .track {
    position: relative;
    height: 20px;
  }
  .track::before {
    content: '';
    position: absolute;
    left: 0;
    right: 0;
    top: 9px;
    height: 3px;
    background: var(--line);
    border-radius: 2px;
  }
  .fill {
    position: absolute;
    top: 9px;
    height: 3px;
    background: var(--accent);
  }
  input[type='range'] {
    position: absolute;
    left: 0;
    width: 100%;
    top: 0;
    margin: 0;
    height: 20px;
    background: none;
    pointer-events: none;
    appearance: none;
    -webkit-appearance: none;
  }
  input[type='range']::-webkit-slider-runnable-track {
    background: none;
    height: 20px;
  }
  input[type='range']::-webkit-slider-thumb {
    -webkit-appearance: none;
    pointer-events: auto;
    width: 14px;
    height: 14px;
    margin-top: 3px;
    border-radius: 50%;
    background: var(--accent);
    border: 2px solid var(--panel);
    cursor: pointer;
  }
  .label {
    font-size: 11.5px;
    color: var(--text);
    text-align: center;
  }
</style>
