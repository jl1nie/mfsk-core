<script lang="ts">
  import * as api from './api';
  import type { Activity, Query, Spot, StationRow } from './types';
  import BandsView from './BandsView.svelte';
  import PresenceMap from './PresenceMap.svelte';
  import MiniMap from './MiniMap.svelte';
  import { stamp } from './analysis';

  let {
    dir,
    q,
    activity,
    nowMs,
    me,
    picked = $bindable(null),
  }: {
    dir: string;
    q: Query;
    activity: Activity[];
    nowMs: number;
    /** The locator the small map is drawn from: home, or the server the map is centred on. */
    me: string;
    /** The station opened, which the Map tab marks. */
    picked: { call: string; grid: string } | null;
  } = $props();
  let view = $state<'stations' | 'decodes'>('stations');
  let stations = $state<StationRow[]>([]);
  let decodes = $state<Spot[]>([]);
  let open = $state<string | null>(null);
  let error = $state('');
  const LIMIT = { stations: 1000, decodes: 2000 };

  $effect(() => {
    void q;
    void view;
    void load();
  });

  let seq = 0;
  async function load() {
    const mine = ++seq;
    try {
      error = '';
      if (view === 'stations') {
        const r = await api.dbStations(dir, q, LIMIT.stations);
        if (mine === seq) stations = r;
      } else {
        const r = await api.dbDecodes(dir, q, LIMIT.decodes);
        if (mine === seq) decodes = r;
      }
    } catch (e) {
      error = String(e);
    }
  }

  /** More than one server heard anything: say which. */
  const multi = $derived(activity.length > 0 && (stations.some((x) => x.servers.includes(',')) || new Set(decodes.map((d) => d.server)).size > 1 || new Set(stations.map((x) => x.servers)).size > 1));
  /** Open a station (or close it): the Map tab marks the open one. */
  function pick(s: StationRow) {
    if (open === s.call) {
      open = null;
      picked = null;
    } else {
      open = s.call;
      picked = s.grid ? { call: s.call, grid: s.grid } : null;
    }
  }

  const km = (r: { km: number | null; bearing: number | null }) =>
    r.km != null ? `${Math.round(r.km).toLocaleString()} km @ ${Math.round(r.bearing ?? 0)}°` : '';
</script>

<div class="results">
  <!-- When the bands opened, for what the query found. -->
  <BandsView {activity} {nowMs} />
  <div class="bar">
    <label><input type="radio" bind:group={view} value="stations" /> Stations</label>
    <label><input type="radio" bind:group={view} value="decodes" /> Decodes</label>
    <span class="hint">
      {#if view === 'stations'}
        {stations.length}{stations.length === LIMIT.stations ? '+' : ''} stations, most heard first. Click one for its hours on the air.
      {:else}
        {decodes.length}{decodes.length === LIMIT.decodes ? '+' : ''} decodes, newest first.
      {/if}
    </span>
  </div>
  {#if error}<p class="err">{error}</p>{/if}
  <div class="scroll">
    {#if view === 'stations'}
      <table>
        <thead>
          <tr><th>Call</th><th>Grid</th><th>Bands</th>{#if multi}<th>Servers</th>{/if}<th>Heard</th><th>Best</th><th>Distance</th><th>First</th><th>Last</th></tr>
        </thead>
        <tbody>
          {#each stations as s (s.call)}
            <tr class="row" class:open={open === s.call} onclick={() => pick(s)}>
              <td>{s.call}</td>
              <td>{s.grid ?? ''}</td>
              <td>{s.bands}</td>
              {#if multi}<td>{s.servers}</td>{/if}
              <td>{s.count}</td>
              <td>{s.bestSnr} dB</td>
              <td>{km(s)}</td>
              <td>{stamp(s.first)}</td>
              <td>{stamp(s.last)}</td>
            </tr>
            {#if open === s.call}
              <tr class="detail">
                <td colspan={multi ? 9 : 8}>
                  <div class="drill">
                    {#if s.grid}<MiniMap {me} grid={s.grid} call={s.call} km={s.km} bearing={s.bearing} />{/if}
                    <div class="hours"><PresenceMap {dir} {q} call={s.call} /></div>
                  </div>
                </td>
              </tr>
            {/if}
          {/each}
        </tbody>
      </table>
    {:else}
      <table>
        <thead>
          <tr><th>UTC</th>{#if multi}<th>Server</th>{/if}<th>Call</th><th>Grid</th><th>Band</th><th>Mode</th><th>SNR</th><th>DT</th><th>Hz</th><th>Message</th><th>Distance</th></tr>
        </thead>
        <tbody>
          {#each decodes as d, i (i)}
            <tr>
              <td>{stamp(d.t)}</td>
              {#if multi}<td>{d.server}</td>{/if}
              <td>{d.call ?? ''}</td>
              <td>{d.grid ?? ''}</td>
              <td>{d.band}</td>
              <td>{d.mode}</td>
              <td>{d.snr}</td>
              <td>{d.dt.toFixed(1)}</td>
              <td>{d.audioHz}</td>
              <td class="msg">{d.text}</td>
              <td>{km(d)}</td>
            </tr>
          {/each}
        </tbody>
      </table>
    {/if}
  </div>
</div>

<style>
  .bar {
    display: flex;
    gap: 14px;
    align-items: center;
    padding: 4px 0 8px;
    font-size: 12px;
  }
  .hint {
    color: var(--muted);
  }
  .err {
    color: #d9822b;
    font-size: 12px;
  }
  .results {
    flex: 1;
    min-height: 0;
    display: flex;
    flex-direction: column;
  }
  .results > :global(.bands) {
    flex: none;
  }
  .scroll {
    flex: 1;
    min-height: 0;
    overflow: auto;
  }
  table {
    border-collapse: collapse;
    width: 100%;
    font-size: 12.5px;
  }
  th,
  td {
    text-align: left;
    padding: 2px 12px 2px 0;
    white-space: nowrap;
  }
  th {
    position: sticky;
    top: 0;
    background: var(--panel);
    color: var(--muted);
    font-weight: 500;
  }
  .row {
    cursor: pointer;
  }
  .row:hover,
  .row.open {
    background: var(--alt);
  }
  .msg {
    font-family: ui-monospace, monospace;
  }
  .drill {
    display: flex;
    gap: 14px;
    align-items: flex-start;
  }
  .hours {
    flex: 1;
    min-width: 0;
  }
  .detail td {
    padding: 6px 0 12px;
    white-space: normal;
  }
</style>
