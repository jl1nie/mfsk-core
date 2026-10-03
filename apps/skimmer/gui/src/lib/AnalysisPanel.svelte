<script lang="ts">
  import * as api from './api';
  import type { Activity, Heard } from './types';
  import { PERIODS, range, sortBands, type PeriodId } from './analysis';
  import MapView from './MapView.svelte';
  import BandsView from './BandsView.svelte';
  import StationView from './StationView.svelte';
  import FreqView from './FreqView.svelte';
  import ClockView from './ClockView.svelte';
  import CqList from './CqList.svelte';

  let { dir, me }: { dir: string; me: string } = $props();

  type Tab = 'map' | 'cq' | 'bands' | 'station' | 'freq' | 'clock';
  const TABS: [Tab, string][] = [
    ['map', 'Map'],
    ['cq', 'CQ'],
    ['bands', 'Band openings'],
    ['station', 'Station'],
    ['freq', 'Free frequencies'],
    ['clock', 'Clock'],
  ];
  let tab = $state<Tab>('map');
  let period = $state<PeriodId>('24h');
  let band = $state<string | null>(null);
  let nowMs = $state(Date.now());
  let activity = $state<Activity[]>([]);
  let heard = $state<Heard[]>([]);
  let span = $state<[number | null, number | null, number]>([null, null, 0]);
  let error = $state('');

  const r = $derived(range(period, nowMs));
  const bands = $derived(sortBands(activity.map((a) => a.band)));
  // The band defaults to the one with the most stations.
  $effect(() => {
    if (band && bands.includes(band)) return;
    const tot = new Map<string, number>();
    for (const a of activity) tot.set(a.band, (tot.get(a.band) ?? 0) + a.stations);
    band = [...tot].sort((a, b) => b[1] - a[1])[0]?.[0] ?? null;
  });

  async function load() {
    nowMs = Date.now();
    const [since, until] = range(period, nowMs);
    try {
      error = '';
      span = await api.dbSpan(dir);
      activity = await api.dbActivity(dir, since, until);
      heard = tab === 'map' ? await api.dbHeard(dir, me, band, since, until) : heard;
    } catch (e) {
      error = String(e).includes('unable to open') ? 'No database yet: decodes are recorded once the skimmer runs with the database on.' : String(e);
      activity = [];
      heard = [];
    }
  }

  $effect(() => {
    void period;
    void tab;
    void band;
    void dir;
    void me;
    void load();
    const t = setInterval(load, 60_000);
    return () => clearInterval(t);
  });
</script>

<section class="analysis">
  <div class="top">
    <nav>
      {#each TABS as [id, label] (id)}
        <button class:on={tab === id} onclick={() => (tab = id)}>{label}</button>
      {/each}
    </nav>
    <div class="ctl">
      <label>
        Period
        <select bind:value={period}>
          {#each PERIODS as p (p.id)}<option value={p.id}>{p.label}</option>{/each}
        </select>
      </label>
      {#if tab === 'map' || tab === 'cq' || tab === 'freq'}
        <label>
          Band
          <select bind:value={band}>
            {#if tab !== 'freq'}<option value={null}>all</option>{/if}
            {#each bands as b (b)}<option value={b}>{b}</option>{/each}
          </select>
        </label>
      {/if}
      <button onclick={load} title="Read the database again (it also refreshes every minute)">↻</button>
    </div>
  </div>
  {#if error}
    <p class="err">{error}</p>
  {:else if span[2] === 0}
    <p class="hint">The database is empty.</p>
  {:else}
    <p class="hint">
      {span[2].toLocaleString()} decodes stored, {span[0] != null ? new Date(span[0] * 1000).toISOString().slice(0, 10) : ''} to
      {span[1] != null ? new Date(span[1] * 1000).toISOString().slice(0, 10) : ''}{me ? ` · home ${me}` : ' · no home grid (Settings > Station)'}
    </p>
  {/if}
  <div class="body">
    {#if tab === 'map'}
      <MapView {heard} {me} />
    {:else if tab === 'cq'}
      <CqList {dir} {me} {band} since={r[0]} until={r[1]} />
    {:else if tab === 'bands'}
      <BandsView {activity} {nowMs} />
    {:else if tab === 'station'}
      <StationView {dir} since={r[0]} until={r[1]} />
    {:else if tab === 'freq'}
      <FreqView {dir} {band} since={r[0]} until={r[1]} />
    {:else}
      <ClockView {dir} since={r[0]} until={r[1]} />
    {/if}
  </div>
</section>

<style>
  .analysis {
    background: var(--panel);
    border: 1px solid var(--line);
    border-radius: 6px;
    padding: 10px 14px 14px;
    margin-bottom: 10px;
    min-height: 360px;
  }
  .top {
    display: flex;
    flex-wrap: wrap;
    gap: 8px 20px;
    align-items: center;
    justify-content: space-between;
  }
  nav {
    display: flex;
    gap: 4px;
    flex-wrap: wrap;
  }
  nav button.on {
    background: var(--accent);
    color: var(--accent-text);
  }
  .ctl {
    display: flex;
    gap: 12px;
    align-items: center;
    font-size: 12px;
  }
  .hint {
    color: var(--muted);
    font-size: 12px;
    margin: 8px 0;
  }
  .err {
    color: #d9822b;
    font-size: 12.5px;
    margin: 8px 0;
  }
</style>
