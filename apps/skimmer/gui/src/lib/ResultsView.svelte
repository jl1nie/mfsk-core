<script lang="ts">
  import * as api from './api';
  import type { Activity, Query, Spot, StationRow } from './types';
  import BandsView from './BandsView.svelte';
  import PresenceMap from './PresenceMap.svelte';
  import { stamp } from './analysis';

  let { dir, q, activity, nowMs }: { dir: string; q: Query; activity: Activity[]; nowMs: number } = $props();
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

  async function load() {
    try {
      error = '';
      if (view === 'stations') stations = await api.dbStations(dir, q, LIMIT.stations);
      else decodes = await api.dbDecodes(dir, q, LIMIT.decodes);
    } catch (e) {
      error = String(e);
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
          <tr><th>Call</th><th>Grid</th><th>Bands</th><th>Heard</th><th>Best</th><th>Distance</th><th>First</th><th>Last</th></tr>
        </thead>
        <tbody>
          {#each stations as s (s.call)}
            <tr class="row" class:open={open === s.call} onclick={() => (open = open === s.call ? null : s.call)}>
              <td>{s.call}</td>
              <td>{s.grid ?? ''}</td>
              <td>{s.bands}</td>
              <td>{s.count}</td>
              <td>{s.bestSnr} dB</td>
              <td>{km(s)}</td>
              <td>{stamp(s.first)}</td>
              <td>{stamp(s.last)}</td>
            </tr>
            {#if open === s.call}
              <tr class="detail"><td colspan="8"><PresenceMap {dir} {q} call={s.call} /></td></tr>
            {/if}
          {/each}
        </tbody>
      </table>
    {:else}
      <table>
        <thead>
          <tr><th>UTC</th><th>Call</th><th>Grid</th><th>Band</th><th>Mode</th><th>SNR</th><th>DT</th><th>Hz</th><th>Message</th><th>Distance</th></tr>
        </thead>
        <tbody>
          {#each decodes as d, i (i)}
            <tr>
              <td>{stamp(d.t)}</td>
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
  .detail td {
    padding: 6px 0 12px;
    white-space: normal;
  }
</style>
