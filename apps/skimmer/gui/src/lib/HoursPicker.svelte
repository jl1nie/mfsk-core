<script lang="ts">
  /**
   * The hours of the UTC day a band takes part in: a strip of 24 hours, one click each.
   * Empty `hours` is the whole day.
   */
  import { clockOf, gridLonLat, sunTimes } from './analysis';

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

  /** Sunrise and sunset today at the server (minutes of the UTC day), when its locator is known. */
  const sun = $derived.by(() => {
    const g = gridLonLat(grid);
    return g ? sunTimes(g, Date.now()) : null;
  });
  /**
   * "Day" and "night" from today's sunrise and sunset at the server, each with an hour of grey line either side:
   * day runs from an hour before sunrise to an hour after sunset, night from an hour before sunset to an hour after
   * sunrise, so the hours of dawn and dusk (when the DX opens) are in both. An hour is in if any part of it is.
   */
  const MARGIN = 60;
  const sunHours = $derived.by(() => {
    if (!sun || sun.rise === null || sun.set === null) return null;
    const wrap = (m: number) => ((m % 1440) + 1440) % 1440;
    const inArc = (m: number, from: number, to: number) => {
      const f = wrap(from);
      const t = wrap(to);
      const x = wrap(m);
      return f <= t ? x >= f && x <= t : x >= f || x <= t;
    };
    const hours = (from: number, to: number) =>
      Array.from({ length: 24 }, (_, h) => [0, 10, 20, 30, 40, 50, 59].some((k) => inArc(h * 60 + k, from, to)));
    return {
      day: hours(sun.rise - MARGIN, sun.set + MARGIN),
      night: hours(sun.set - MARGIN, sun.rise + MARGIN),
    };
  });

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
    sunHours
      ? ['day', sunHours.day, `Sunrise ${clockOf(sun!.rise!)} to sunset ${clockOf(sun!.set!)} UTC today at ${grid.toUpperCase()}, with an hour of grey line either side`]
      : ['day', inLocal(6, 18), `06:00-18:00 local time ${place.name} (UTC${place.off >= 0 ? '+' : ''}${place.off}); give the grid for today's sunrise and sunset`],
    sunHours
      ? ['night', sunHours.night, `Sunset ${clockOf(sun!.set!)} to sunrise ${clockOf(sun!.rise!)} UTC today at ${grid.toUpperCase()}, with an hour of grey line either side`]
      : ['night', inLocal(18, 6), `18:00-06:00 local time ${place.name} (UTC${place.off >= 0 ? '+' : ''}${place.off}); give the grid for today's sunrise and sunset`],
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
  {#if sun && sun.rise !== null}
    <div class="sunline" aria-hidden="true">
      <i class="rise" style="left:{(sun.rise / 1440) * 100}%" title="Sunrise {clockOf(sun.rise)} UTC">↑</i>
      <i class="set" style="left:{(sun.set! / 1440) * 100}%" title="Sunset {clockOf(sun.set!)} UTC">↓</i>
    </div>
  {/if}
  <div class="scale"><span>0</span><span>6</span><span>12</span><span>18</span><span>24 UTC</span></div>
  <div class="pre">
    {#each presets as [name, v, tip] (name)}
      <button type="button" class="chip" class:on={same(hours, v)} title={tip} onclick={() => onchange(v)}>{name}</button>
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
  .sunline {
    position: relative;
    height: 11px;
    width: 264px;
    font-size: 10px;
    line-height: 11px;
  }
  .sunline i {
    position: absolute;
    transform: translateX(-50%);
    font-style: normal;
    font-weight: 700;
  }
  .sunline .rise {
    color: #d9822b;
  }
  .sunline .set {
    color: #7a5cc4;
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
