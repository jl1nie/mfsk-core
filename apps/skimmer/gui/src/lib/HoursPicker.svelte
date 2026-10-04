<script lang="ts">
  /**
   * The hours of the UTC day a band takes part in: a strip of 24 hours, one click each. Empty `hours` is the
   * whole day. "Day" and "night" can follow the calendar instead: the strip then shows today's hours, which the
   * rotation recomputes from each day's sunrise and sunset at the server.
   */
  import { clockOf, gridLonLat, sunTimes } from './analysis';

  type Follow = '' | 'day' | 'night';
  let {
    hours,
    follow = '',
    margin = 1,
    grid = '',
    onchange,
  }: {
    hours: boolean[];
    /** "day" or "night": follows the Sun at the server day by day; empty: the fixed hours. */
    follow?: Follow;
    /** Hours of grey line either side of sunrise and sunset. */
    margin?: number;
    /** The server's locator: today's sunrise and sunset, and its local time, come from it. */
    grid?: string;
    onchange: (v: { hours: boolean[]; follow: Follow; margin: number }) => void;
  } = $props();

  /** Sunrise and sunset today at the server (minutes of the UTC day), when its locator is known. */
  const sun = $derived.by(() => {
    const g = gridLonLat(grid);
    return g ? sunTimes(g, Date.now()) : null;
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

  /**
   * Today's "day" and "night": day from `margin` hours before sunrise to `margin` after sunset, night from `margin`
   * before sunset to `margin` after sunrise, so the grey line (dawn and dusk, when the DX opens) is in both. An
   * hour is in if any part of it is. Without a locator, or in a polar day or night: 06-18 and 18-06 local.
   */
  const sunHours = $derived.by(() => {
    if (!sun || sun.rise === null || sun.set === null) return null;
    const m = Math.max(0, margin) * 60;
    const wrap = (x: number) => ((x % 1440) + 1440) % 1440;
    const inArc = (x: number, from: number, to: number) => {
      const f = wrap(from);
      const t = wrap(to);
      const v = wrap(x);
      return f <= t ? v >= f && v <= t : v >= f || v <= t;
    };
    const hrs = (from: number, to: number) =>
      Array.from({ length: 24 }, (_, h) => [0, 10, 20, 30, 40, 50, 59].some((k) => inArc(h * 60 + k, from, to)));
    return { day: hrs(sun.rise - m, sun.set + m), night: hrs(sun.set - m, sun.rise + m) };
  });
  const dayHours = $derived(sunHours?.day ?? inLocal(6, 18));
  const nightHours = $derived(sunHours?.night ?? inLocal(18, 6));

  /** What the strip shows: today's hours when following, else the fixed ones. */
  const shown = $derived(follow === 'day' ? dayHours : follow === 'night' ? nightHours : hours);
  const all = $derived(shown.length !== 24 || shown.every(Boolean));
  const on = (h: number) => all || shown[h];
  const none = $derived(!all && shown.every((x) => !x));
  const same = (a: boolean[], b: boolean[]) =>
    (a.length === 0 ? Array<boolean>(24).fill(true) : a).every((x, i) => x === (b.length === 0 ? true : b[i]));
  /** The fixed hours are today's day or night, so they could follow the calendar. */
  const kind = $derived<Follow>(follow || (same(hours, dayHours) ? 'day' : same(hours, nightHours) ? 'night' : ''));

  const emit = (h: boolean[], f: Follow, m = margin) => onchange({ hours: h, follow: f, margin: m });

  function toggle(h: number) {
    if (follow) return; // following the calendar: the hours are the Sun's
    const next = hours.length === 24 ? [...hours] : Array<boolean>(24).fill(true);
    next[h] = !next[h];
    emit(next.every(Boolean) ? [] : next, '');
  }

  const presets = $derived<[string, boolean[], Follow, string][]>([
    ['all day', [], '', 'The whole UTC day'],
    [
      'day',
      dayHours,
      'day',
      sunHours
        ? `Sunrise ${clockOf(sun!.rise!)} to sunset ${clockOf(sun!.set!)} UTC today at ${grid.toUpperCase()}, with ${margin} h of grey line either side`
        : `06:00-18:00 local time ${place.name} (UTC${place.off >= 0 ? '+' : ''}${place.off}); give the grid for the sunrise and sunset`,
    ],
    [
      'night',
      nightHours,
      'night',
      sunHours
        ? `Sunset ${clockOf(sun!.set!)} to sunrise ${clockOf(sun!.rise!)} UTC today at ${grid.toUpperCase()}, with ${margin} h of grey line either side`
        : `18:00-06:00 local time ${place.name} (UTC${place.off >= 0 ? '+' : ''}${place.off}); give the grid for the sunrise and sunset`,
    ],
  ]);

  /** `21-08`: the runs of hours that are in, as UTC hours (the end is the first hour out). */
  const label = $derived.by(() => {
    if (all) return 'all day';
    if (none) return 'no hour: never heard';
    const runs: string[] = [];
    for (let h = 0; h < 24; h++) {
      if (!shown[h] || shown[(h + 23) % 24]) continue;
      let e = h;
      while (shown[(e + 1) % 24] && (e + 1) % 24 !== h) e++;
      runs.push(`${String(h).padStart(2, '0')}-${String((e + 1) % 24).padStart(2, '0')}`);
    }
    return `${runs.join(', ')} UTC${follow ? ' today' : ''}`;
  });
  const local = $derived(
    all || none
      ? ''
      : ` · local ${place.name}: ${Array.from({ length: 24 }, (_, l) => l)
          .filter((l) => shown[(((l - place.off) % 24) + 24) % 24] && !shown[(((l - 1 - place.off) % 24) + 24) % 24])
          .map((l) => {
            let e = l;
            while (shown[(((e + 1 - place.off) % 24) + 24) % 24] && e - l < 23) e++;
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
        class:locked={!!follow}
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
    {#each presets as [name, v, tag, tip] (name)}
      <button
        type="button"
        class="chip"
        class:on={tag === '' ? !follow && same(hours, []) : follow === tag || (!follow && same(hours, v))}
        title={tip}
        onclick={() => emit(v, tag)}>{name}</button
      >
    {/each}
    <span class="hint" class:warn={none}>{label}{local}</span>
  </div>
  {#if kind}
    <div class="pre">
      <label class="follow" title="Recomputed every day from each day's sunrise and sunset at the server: there is no moment of renewal, the edges move a minute or two a day">
        <input
          type="checkbox"
          checked={!!follow}
          onchange={(e) => emit(e.currentTarget.checked ? shown : shown, e.currentTarget.checked ? kind : '')}
        />
        follow the calendar
      </label>
      {#if follow}
        <label class="follow" title="Grey line: hours either side of sunrise and sunset that count as day (and as night)">
          ±
          <input
            class="mg"
            type="number"
            min="0"
            max="6"
            step="0.5"
            value={margin}
            onchange={(e) => emit(shown, follow, Math.min(6, Math.max(0, Number(e.currentTarget.value) || 0)))}
          />
          h
        </label>
      {/if}
    </div>
  {/if}
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
  .cell.locked {
    cursor: default;
  }
  .follow {
    display: inline-flex;
    align-items: center;
    gap: 4px;
    font-size: 11.5px;
    color: var(--muted);
  }
  .follow .mg {
    width: 48px;
  }
  .hint {
    font-size: 11.5px;
    color: var(--muted);
  }
  .hint.warn {
    color: #d9822b;
  }
</style>
