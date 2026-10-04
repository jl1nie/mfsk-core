<script lang="ts">
  import { untrack } from 'svelte';
  import * as api from './api';
  import type { Activity, MapPoint, Query, Summary } from './types';
  import { blankForm, formErrors, toQuery, type QueryForm } from './analysis';
  import QueryBar from './QueryBar.svelte';
  import MapView from './MapView.svelte';
  import ResultsView from './ResultsView.svelte';
  import DatabaseView from './DatabaseView.svelte';

  let { dir: recorded, me }: { dir: string; me: string } = $props();
  /** The database file read: the one recorded into, unless another is opened in the Database tab. */
  let dir = $state(untrack(() => recorded));
  $effect(() => {
    dir = recorded;
  });

  type Tab = 'map' | 'results' | 'database';
  const TABS: [Tab, string][] = [
    ['map', 'Map'],
    ['results', 'Results'],
    ['database', 'Database'],
  ];
  const KEY = 'skimmer.analysis';

  function restore(): { form: QueryForm; tab: Tab } {
    try {
      const s = JSON.parse(localStorage.getItem(KEY) ?? 'null');
      if (s?.form) return { form: { ...blankForm(), ...s.form }, tab: TABS.some(([id]) => id === s.tab) ? s.tab : 'map' };
    } catch {
      /* private window, or unreadable: start fresh */
    }
    return { form: blankForm(), tab: 'map' };
  }
  const saved = restore();
  let form = $state<QueryForm>(saved.form);
  let tab = $state<Tab>(saved.tab);
  let nowMs = $state(Date.now());
  /** Bumped by Search and by the minute timer: reads everything again. */
  let rev = $state(0);
  let slice = $state(300);

  let bands = $state<string[]>([]);
  let activity = $state<Activity[]>([]);
  let points = $state<MapPoint[]>([]);
  let summary = $state<Summary | null>(null);
  let snrHist = $state<[number, number][]>([]);
  /** (name, locator) of each server there are decodes from. */
  let servers = $state<[string, string][]>([]);
  /** The map is centred on home or on a server. */
  let centre = $state('');
  /** The station opened in Results, which the map marks. */
  let picked = $state<{ call: string; grid: string } | null>(null);
  let span = $state<[number | null, number | null, number]>([null, null, 0]);
  let error = $state('');

  // The query the data on screen answers; presets end at the time of the last read.
  let q = $state<Query>(untrack(() => toQuery(form, me, nowMs)));

  function apply() {
    // A box that cannot be read would be ignored without a word: not searched at all.
    if (formErrors(form).length) return;
    rev++;
  }

  /** A read that is not the latest is dropped: a slow one must not overwrite a newer answer. */
  let seq = 0;
  async function load() {
    const mine = ++seq;
    nowMs = Date.now();
    const next = toQuery(form, me, nowMs);
    try {
      error = '';
      localStorage.setItem(KEY, JSON.stringify({ form, tab }));
    } catch {
      /* not essential */
    }
    try {
      const [sp, bs, sm, sh, sv] = await Promise.all([
        api.dbSpan(dir),
        api.dbBands(dir),
        api.dbSummary(dir, next),
        api.dbSnrHist(dir, next),
        api.dbServers(dir),
      ]);
      if (mine !== seq) return;
      span = sp;
      bands = bs;
      summary = sm;
      snrHist = sh;
      servers = sv;
      const act = tab === 'results' ? await api.dbActivity(dir, next) : null;
      const pts = tab === 'map' ? await api.dbPoints(dir, next, slice) : null;
      if (mine !== seq) return;
      q = next;
      if (act) activity = act;
      if (pts) points = pts;
    } catch (e) {
      const m = String(e);
      error = m.includes('unable to open')
        ? 'No database yet: decodes are recorded once the skimmer runs with the database on.'
        : m;
    }
  }

  $effect(() => {
    void rev;
    void tab;
    void slice;
    void dir;
    void me;
    // Typing in the form must not read the database: only Search does.
    untrack(() => void load());
  });
  $effect(() => {
    const t = setInterval(() => rev++, 60_000);
    return () => clearInterval(t);
  });

  const mapMe = $derived(servers.find((x) => x[0] === centre)?.[1] || me);
  const line = $derived(
    summary
      ? `${summary.decodes.toLocaleString()} decodes · ${summary.stations.toLocaleString()} stations (of ${span[2].toLocaleString()} stored)${me ? '' : ' · no home grid: Settings > Station'}`
      : '',
  );
</script>

<section class="analysis">
  <QueryBar bind:form {bands} onapply={apply} summary={line} {error} {snrHist} servers={servers.map((x) => x[0])} />
  <nav>
    {#each TABS as [id, label] (id)}
      <button class:on={tab === id} onclick={() => (tab = id)}>{label}</button>
    {/each}
  </nav>
  <div class="body">
    {#if tab === 'map'}
      {#if servers.length > 1}
        <label class="centre">
          Centre the map on
          <select bind:value={centre}>
            <option value="">home ({me || 'no grid'})</option>
            {#each servers as [name, grid] (name)}
              <option value={name}>{name || 'before names'}{grid ? ` (${grid})` : ''}</option>
            {/each}
          </select>
        </label>
      {/if}
      <MapView {points} me={mapMe} since={q.since} until={q.until} bind:slice {picked} onclear={() => (picked = null)} bandsChosen={q.bands} />
    {:else if tab === 'results'}
      <ResultsView {dir} {q} {activity} {nowMs} me={mapMe} bind:picked />
    {:else}
      <DatabaseView {dir} {recorded} {q} onchanged={apply} onopen={(f) => (dir = f ?? recorded)} />
    {/if}
  </div>
</section>

<style>
  .analysis {
    background: var(--panel);
    border: 1px solid var(--line);
    border-radius: 6px;
    padding: 10px 14px 14px;
    flex: 1;
    min-height: 0;
    display: flex;
    flex-direction: column;
    overflow: auto;
  }
  .body {
    flex: 1;
    min-height: 0;
    display: flex;
    flex-direction: column;
  }
  .centre {
    font-size: 12px;
    color: var(--muted);
    margin-bottom: 4px;
    display: block;
  }
  nav {
    display: flex;
    gap: 4px;
    flex-wrap: wrap;
    margin-bottom: 8px;
  }
  nav button.on {
    background: var(--accent);
    color: var(--accent-text);
  }
</style>
