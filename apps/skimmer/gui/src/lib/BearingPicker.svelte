<script lang="ts">
  /**
   * Eight 45-degree wedges around home: click to pick a direction, click a
   * neighbour to widen the sector, click an end wedge to narrow it. The
   * sector is sent as a bearing range that wraps through north when needed.
   */
  let {
    from,
    to,
    onchange,
  }: {
    from: number | null;
    to: number | null;
    onchange: (from: number | null, to: number | null) => void;
  } = $props();

  const NAMES = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];
  const C = 60;
  const RO = 56;
  const RI = 20;

  const norm = (d: number) => ((d % 360) + 360) % 360;
  const inRange = (b: number) => {
    if (from === null || to === null) return false;
    const f = norm(from);
    const t = norm(to);
    const x = norm(b);
    return f <= t ? x >= f && x <= t : x >= f || x <= t;
  };
  /** Wedges whose centre lies in the sector. */
  const picked = $derived(NAMES.map((_, i) => inRange(i * 45)));

  function wedge(i: number) {
    const a0 = ((i * 45 - 22.5) * Math.PI) / 180;
    const a1 = ((i * 45 + 22.5) * Math.PI) / 180;
    const pt = (r: number, a: number) => `${(C + r * Math.sin(a)).toFixed(1)} ${(C - r * Math.cos(a)).toFixed(1)}`;
    return `M ${pt(RI, a0)} L ${pt(RO, a0)} A ${RO} ${RO} 0 0 1 ${pt(RO, a1)} L ${pt(RI, a1)} A ${RI} ${RI} 0 0 0 ${pt(RI, a0)} Z`;
  }
  const label = (i: number) => {
    const a = (i * 45 * Math.PI) / 180;
    const r = (RO + RI) / 2;
    return { x: C + r * Math.sin(a), y: C - r * Math.cos(a) + 3.5 };
  };

  function set(idx: number[]) {
    if (idx.length === 0 || idx.length === 8) return onchange(null, null);
    // idx is one cyclic run, in order.
    onchange(norm(idx[0] * 45 - 22.5), norm(idx[idx.length - 1] * 45 + 22.5));
  }

  function click(i: number) {
    const sel = picked.map((p, k) => (p ? k : -1)).filter((k) => k >= 0);
    if (sel.length === 0) return set([i]);
    // The run in cyclic order: it starts at the member whose predecessor is not one.
    const start = sel.find((k) => !picked[(k + 7) % 8]) ?? sel[0];
    const run: number[] = [];
    for (let k = start, n = 0; picked[k % 8] && n < 8; k++, n++) run.push(k % 8);
    if (run.length !== sel.length) return set([i]); // not one run
    const first = run[0];
    const last = run[run.length - 1];
    if (run.includes(i)) {
      if (run.length === 1) return set([]);
      if (i === first) return set(run.slice(1));
      if (i === last) return set(run.slice(0, -1));
      return set([i]);
    }
    if (i === (first + 7) % 8) return set([i, ...run]);
    if (i === (last + 1) % 8) return set([...run, i]);
    return set([i]);
  }
</script>

<svg viewBox="0 0 120 120" width="120" height="120" role="group" aria-label="bearing sector">
  {#each NAMES as n, i (n)}
    <path d={wedge(i)} class:on={picked[i]} onclick={() => click(i)} role="button" tabindex="0" aria-label={n}
      onkeydown={(e) => (e.key === 'Enter' || e.key === ' ') && click(i)} />
    <text x={label(i).x} y={label(i).y} class:on={picked[i]}>{n}</text>
  {/each}
  <circle cx={C} cy={C} r="13" class="home" onclick={() => onchange(null, null)} role="button" tabindex="0" aria-label="any bearing"
    onkeydown={(e) => (e.key === 'Enter' || e.key === ' ') && onchange(null, null)} />
  <text x={C} y={C + 3.5} class="any">any</text>
</svg>

<style>
  svg {
    display: block;
  }
  path {
    fill: var(--alt);
    stroke: var(--panel);
    stroke-width: 1.5;
    cursor: pointer;
  }
  path:hover {
    fill: var(--line);
  }
  path.on {
    fill: var(--accent);
  }
  text {
    font-size: 9px;
    text-anchor: middle;
    fill: var(--text);
    pointer-events: none;
  }
  text.on {
    fill: var(--accent-text);
  }
  .home {
    fill: var(--panel);
    stroke: var(--line);
    cursor: pointer;
  }
  .any {
    font-size: 7px;
    fill: var(--muted);
  }
</style>
