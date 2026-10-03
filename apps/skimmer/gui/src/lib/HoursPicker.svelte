<script lang="ts">
  /**
   * The hours of the UTC day a band takes part in: a strip of 24 hours, one click each.
   * Empty `hours` is the whole day.
   */
  import { gridLonLat } from './analysis';

  let {
    hours,
    grid = '',
    onchange,
  }: {
    hours: boolean[];
    /** The server's locator: its local time sets what "day" and "night" mean. */
    grid?: string;
    onchange: (hours: boolean[]) => void;
  } = $props();

  const all = $derived(hours.length !== 24 || hours.every(Boolean));
  const on = (h: number) => all || hours[h];
  const none = $derived(!all && hours.every((x) => !x));

  function toggle(h: number) {
    const next = hours.length === 24 ? [...hours] : Array<boolean>(24).fill(true);
    next[h] = !next[h];
    onchange(next.every(Boolean) ? [] : next);
  }

  /** Hours from UTC to the server's local time: its longitude / 15 (mean solar time), else this PC's zone. */
  const place = $derived.by(() => {
    const g = gridLonLat(grid);
    if (g) return { off: Math.round(g[0] / 15), name: `at ${grid.toUpperCase()}` };
    return { off: Math.round(-new Date().getTimezoneOffset() / 60), name: 'on this PC' };
  });
  const inLocal = (from: number, to: number) =>
    Array.from({ length: 24 }, (_, utc) => {
      const l = (((utc + place.off) % 24) + 24) % 24;
      return from < to ? l >= from && l < to : l >= from || l < to;
    });
  const presets = $derived<[string, boolean[], string][]>([
    ['all day', [], 'The whole UTC day'],
    ['day', inLocal(6, 18), `06:00-18:00 local time ${place.name} (UTC${place.off >= 0 ? '+' : ''}${place.off})`],
    ['night', inLocal(18, 6), `18:00-06:00 local time ${place.name} (UTC${place.off >= 0 ? '+' : ''}${place.off})`],
    ['invert', all ? Array<boolean>(24).fill(false) : hours.map((x) => !x), 'The hours that are out come in, and the other way round'],
  ]);
  const same = (a: boolean[], b: boolean[]) =>
    (a.length === 0 ? Array<boolean>(24).fill(true) : a).every((x, i) => x === (b.length === 0 ? true : b[i]));

  /** `21-08`: the runs of hours that are in, as UTC hours (the end is the first hour out). */
  const label = $derived.by(() => {
    if (all) return 'all day';
    if (none) return 'no hour: never heard';
    const runs: string[] = [];
    for (let h = 0; h < 24; h++) {
      if (!hours[h] || hours[(h + 23) % 24]) continue;
      let e = h;
      while (hours[(e + 1) % 24] && (e + 1) % 24 !== h) e++;
      runs.push(`${String(h).padStart(2, '0')}-${String((e + 1) % 24).padStart(2, '0')}`);
    }
    return `${runs.join(', ')} UTC`;
  });
  const local = $derived(
    all || none
      ? ''
      : ` · local ${place.name}: ${Array.from({ length: 24 }, (_, l) => l)
          .filter((l) => hours[(((l - place.off) % 24) + 24) % 24] && !hours[(((l - 1 - place.off) % 24) + 24) % 24])
          .map((l) => {
            let e = l;
            while (hours[(((e + 1 - place.off) % 24) + 24) % 24] && e - l < 23) e++;
            return `${String(l).padStart(2, '0')}-${String((e + 1) % 24).padStart(2, '0')}`;
          })
          .join(', ')}`,
  );
</script>

<div class="hp">
  <div class="strip" role="group" aria-label="UTC hours">
    {#each Array.from({ length: 24 }, (_, i) => i) as h (h)}
      <button
        type="button"
        class="cell"
        class:on={on(h)}
        class:tick={h % 6 === 0}
        aria-pressed={on(h)}
        aria-label={`${h}:00 UTC`}
        title={`${String(h).padStart(2, '0')}:00-${String((h + 1) % 24).padStart(2, '0')}:00 UTC: ${on(h) ? 'in the rotation' : 'out'}`}
        onclick={() => toggle(h)}
      ></button>
    {/each}
  </div>
  <div class="scale"><span>0</span><span>6</span><span>12</span><span>18</span><span>24 UTC</span></div>
  <div class="pre">
    {#each presets as [name, v, tip] (name)}
      <button type="button" class="chip" class:on={name !== 'invert' && same(hours, v)} title={tip} onclick={() => onchange(v)}>{name}</button>
    {/each}
    <span class="hint" class:warn={none}>{label}{local}</span>
  </div>
</div>

<style>
  .hp {
    display: flex;
    flex-direction: column;
    gap: 4px;
  }
  .strip {
    display: grid;
    grid-template-columns: repeat(24, 1fr);
    gap: 1px;
    width: 264px;
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
  .hint.warn {
    color: #d9822b;
  }
</style>
