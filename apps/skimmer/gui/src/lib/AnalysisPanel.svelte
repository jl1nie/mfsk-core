<script lang="ts">
  import { untrack } from 'svelte';
  import * as api from './api';
  import type { Activity, MapPoint, Query, Summary } from './types';
  import { blankForm, toQuery, type QueryForm } from './analysis';
  import QueryBar from './QueryBar.svelte';
  import MapView from './MapView.svelte';
  import BandsView from './BandsView.svelte';
  import ResultsView from './ResultsView.svelte';
  import ClockView from './ClockView.svelte';

  let { dir, me }: { dir: string; me: string } = $props();

  type Tab = 'map' | 'bands' | 'results' | 'clock';
  const TABS: [Tab, string][] = [
    ['map', 'Map'],
    ['bands', 'Band openings'],
    ['results', 'Results'],
    ['clock', 'Clock'],
  ];
  const KEY = 'skimmer.analysis';

  function restore(): { form: QueryForm; tab: Tab } {
    try {
      const s = JSON.parse(localStorage.getItem(KEY) ?? 'null');
      if (s?.form) return { form: { ...blankForm(), ...s.form }, tab: s.tab ?? 'map' };
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
  let span = $state<[number | null, number | null, number]>([null, null, 0]);
  let error = $state('');

  // The query the data on screen answers; presets end at the time of the last read.
  let q = $state<Query>(untrack(() => toQuery(form, me, nowMs)));

  function apply() {
    rev++;
  }

  async function load() {
    nowMs = Date.now();
    const next = toQuery(form, me, nowMs);
    try {
      error = '';
      localStorage.setItem(KEY, JSON.stringify({ form, tab }));
    } catch {
      /* not essential */
    }
    try {
      const [sp, bs, sm] = await Promise.all([api.dbSpan(dir), api.dbBands(dir), api.dbSummary(dir, next)]);
      span = sp;
      bands = bs;
      summary = sm;
      q = next;
      if (tab === 'bands') activity = await api.dbActivity(dir, next);
      if (tab === 'map') points = await api.dbPoints(dir, next, slice);
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

  const line = $derived(
    summary
      ? `${summary.decodes.toLocaleString()} decodes · ${summary.stations.toLocaleString()} stations (of ${span[2].toLocaleString()} stored)${me ? '' : ' · no home grid: Settings > Station'}`
      : '',
  );
</script>

<section class="analysis">
  <QueryBar bind:form {bands} onapply={apply} summary={line} {error} />
  <nav>
    {#each TABS as [id, label] (id)}
      <button class:on={tab === id} onclick={() => (tab = id)}>{label}</button>
    {/each}
  </nav>
  <div class="body">
    {#if tab === 'map'}
      <MapView {points} {me} since={q.since} until={q.until} bind:slice />
    {:else if tab === 'bands'}
      <BandsView {activity} {nowMs} />
    {:else if tab === 'results'}
      <ResultsView {dir} {q} />
    {:else}
      <ClockView {dir} since={q.since} until={q.until} />
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
