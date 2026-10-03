<script lang="ts">
  /**
   * The hours of the UTC day a band takes part in: a strip of 24 hours to drag
   * across (drag leftwards to run through midnight), and presets for the usual.
   * Empty from/to is the whole day.
   */
  import { gridLonLat } from './analysis';

  let {
    from,
    to,
    grid = '',
    onchange,
  }: {
    from: string;
    to: string;
    /** The server's locator: its local time sets what "day" and "night" mean. */
    grid?: string;
    onchange: (from: string, to: string) => void;
  } = $props();

  const hour = (s: string) => {
    const m = /^(\d{1,2}):(\d{2})$/.exec(s.trim());
    return m ? Number(m[1]) + Number(m[2]) / 60 : null;
  };
  const clock = (h: number) => `${String(h % 24).padStart(2, '0')}:00`;

  const f = $derived(hour(from));
  const t = $derived(hour(to));
  const allDay = $derived(f === null || t === null || f === t);
  const isIn = (h: number) => allDay || (f! < t! ? h >= f! && h < t! : h >= f! || h < t!);

  // A drag is a start cell and the cell the pointer is on now.
  let drag = $state<{ a: number; b: number } | null>(null);
  const shown = (h: number) => {
    if (!drag) return isIn(h);
    const { a, b } = drag;
    return a <= b ? h >= a && h <= b : h >= a || h <= b;
  };

  function down(h: number, e: PointerEvent) {
    drag = { a: h, b: h };
    (e.currentTarget as HTMLElement).releasePointerCapture?.(e.pointerId);
  }
  function over(h: number) {
    if (drag) drag.b = h;
  }
  function up() {
    if (!drag) return;
    const { a, b } = drag;
    drag = null;
    const end = (b + 1) % 24;
    if (a === end) onchange('', '');
    else onchange(clock(a), clock(end));
  }

  /**
   * Hours from UTC to the local time of the server: its longitude / 15 (mean
   * solar time) when the grid is given, else this PC's time zone.
   */
  const place = $derived.by(() => {
    const g = gridLonLat(grid);
    if (g) return { off: Math.round(g[0] / 15), name: `at ${grid.toUpperCase()}` };
    return { off: Math.round(-new Date().getTimezoneOffset() / 60), name: 'on this PC' };
  });
  const utcOf = (localHour: number) => clock((((localHour - place.off) % 24) + 24) % 24);
  const presets = $derived<[string, string, string, string][]>([
    ['all day', '', '', 'The whole UTC day'],
    ['day', utcOf(6), utcOf(18), `06:00–18:00 local time ${place.name} (UTC${place.off >= 0 ? '+' : ''}${place.off})`],
    ['night', utcOf(18), utcOf(6), `18:00–06:00 local time ${place.name} (UTC${place.off >= 0 ? '+' : ''}${place.off})`],
  ]);
  // The same hours on that clock.
  const local = $derived.by(() => {
    if (allDay) return '';
    const z = (x: number) => clock((((Math.round(x + place.off) % 24) + 24) % 24));
    return `${z(f!)}–${z(t!)} local ${place.name}`;
  });
</script>

<svelte:window onpointerup={up} />

<div class="hp">
  <div class="strip" role="group" aria-label="UTC hours">
    {#each Array.from({ length: 24 }, (_, i) => i) as h (h)}
      <button
        type="button"
        class="cell"
        class:on={shown(h)}
        class:tick={h % 6 === 0}
        title={`${clock(h)}–${clock(h + 1)} UTC`}
        onpointerdown={(e) => down(h, e)}
        onpointerenter={() => over(h)}
      ></button>
    {/each}
  </div>
  <div class="scale"><span>0</span><span>6</span><span>12</span><span>18</span><span>24 UTC</span></div>
  <div class="pre">
    {#each presets as [name, a, b, tip] (name)}
      <button type="button" class="chip" class:on={from === a && to === b} title={tip} onclick={() => onchange(a, b)}>{name}</button>
    {/each}
    <span class="hint">{allDay ? 'all day' : `${from}–${to} UTC`}{local ? ` · ${local}` : ''}</span>
  </div>
</div>

<style>
  .hp {
    display: flex;
    flex-direction: column;
    gap: 2px;
    user-select: none;
  }
  .strip {
    display: grid;
    grid-template-columns: repeat(24, 1fr);
    gap: 1px;
    width: 264px;
    touch-action: none;
  }
  .cell {
    height: 16px;
    padding: 0;
    border: none;
    border-radius: 2px;
    background: var(--line);
    cursor: pointer;
  }
  .cell.on {
    background: var(--accent);
  }
  .cell.tick {
    box-shadow: -1px 0 0 var(--text);
  }
  .scale {
    display: flex;
    justify-content: space-between;
    width: 264px;
    font-size: 10px;
    color: var(--muted);
  }
  .pre {
    display: flex;
    flex-wrap: wrap;
    gap: 4px;
    align-items: center;
  }
  .chip {
    padding: 1px 8px;
    font-size: 11.5px;
    border-radius: 12px;
  }
  .chip.on {
    background: var(--accent);
    color: var(--accent-text);
  }
  .hint {
    font-size: 11.5px;
    color: var(--muted);
  }
</style>
