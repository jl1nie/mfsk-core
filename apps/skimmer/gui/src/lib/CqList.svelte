<script lang="ts">
  import * as api from './api';
  import type { Cq } from './types';
  import { hhmm, ymd } from './analysis';

  let { dir, me, band, since, until }: { dir: string; me: string; band: string | null; since: number; until: number } = $props();
  let kind = $state<string>('*');
  let rows = $state<Cq[]>([]);
  let error = $state('');

  $effect(() => {
    void band;
    void since;
    void until;
    void kind;
    void load();
  });

  async function load() {
    try {
      error = '';
      rows = await api.dbCqs(dir, me, band, kind === '*' ? null : kind, since, until, 300);
    } catch (e) {
      error = String(e);
    }
  }
</script>

<div class="cqs">
  <div class="bar">
    <label>
      CQ
      <select bind:value={kind}>
        <option value="*">all</option>
        <option value="">plain CQ</option>
        <option value="DX">CQ DX</option>
        <option value="POTA">CQ POTA</option>
        <option value="SOTA">CQ SOTA</option>
        <option value="TEST">CQ TEST</option>
      </select>
    </label>
    <span class="hint">{rows.length}{rows.length === 300 ? '+' : ''} shown, newest first{band ? ` · ${band}` : ''}</span>
  </div>
  {#if error}<p class="err">{error}</p>{/if}
  <div class="scroll">
    <table>
      <thead>
        <tr><th>UTC</th><th>Call</th><th>Grid</th><th>CQ</th><th>Band</th><th>Mode</th><th>SNR</th><th>Distance</th></tr>
      </thead>
      <tbody>
        {#each rows as r (r.t + (r.call ?? '') + r.band + r.mode)}
          <tr>
            <td>{ymd(r.t).slice(5)} {hhmm(r.t)}</td>
            <td>{r.call ?? ''}</td>
            <td>{r.grid ?? ''}</td>
            <td>{r.kind || '—'}</td>
            <td>{r.band}</td>
            <td>{r.mode}</td>
            <td>{r.snr}</td>
            <td>{r.km != null ? `${Math.round(r.km)} km @ ${Math.round(r.bearing ?? 0)}°` : ''}</td>
          </tr>
        {/each}
      </tbody>
    </table>
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
  .scroll {
    max-height: 340px;
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
    padding: 2px 10px 2px 0;
    white-space: nowrap;
  }
  th {
    position: sticky;
    top: 0;
    background: var(--panel);
    color: var(--muted);
    font-weight: 500;
  }
</style>
