<script lang="ts">
  import * as api from './api';
  import type { DbInfo, Query } from './types';
  import { stamp, ymd } from './analysis';

  let {
    dir,
    recorded,
    q,
    onchanged,
    onopen,
  }: {
    dir: string;
    /** The file being recorded into. */
    recorded: string;
    q: Query;
    onchanged: () => void;
    /** Read another database file (null: back to the recording one). */
    onopen: (file: string | null) => void;
  } = $props();

  let info = $state<DbInfo | null>(null);
  let message = $state('');
  let busy = $state('');
  let days = $state(90);
  let from = $state<string>('*');
  let willGo = $state<number | null>(null);

  const mb = (b: number) => (b >= 1e9 ? `${(b / 1e9).toFixed(2)} GB` : `${(b / 1e6).toFixed(1)} MB`);
  const label = (s: string) => s || 'before names';
  const cutoff = () => Math.floor(Date.now() / 1000) - Math.max(0, days) * 86400;

  async function load() {
    try {
      info = await api.dbInfo(dir);
    } catch (e) {
      message = String(e).includes('unable to open') ? 'No database yet.' : String(e);
      info = null;
    }
  }
  $effect(() => {
    void dir;
    void load();
  });
  // How many a deletion would take, as the numbers are typed.
  $effect(() => {
    void days;
    void from;
    void info;
    api
      .dbCountBefore(dir, cutoff(), from === '*' ? null : from)
      .then((n) => (willGo = n))
      .catch(() => (willGo = null));
  });

  async function run(what: string, f: () => Promise<string>) {
    busy = what;
    message = '';
    try {
      message = await f();
    } catch (e) {
      message = `${what} failed: ${e}`;
    } finally {
      busy = '';
      await load();
      onchanged();
    }
  }

  const deleteOld = () =>
    run('Delete', async () => {
      const n = willGo ?? 0;
      const where = from === '*' ? 'all servers' : label(from);
      if (n === 0) return 'Nothing is that old.';
      if (!(await api.ask(`Delete ${n.toLocaleString()} decodes older than ${days} days (${where})? This cannot be undone.`)))
        return 'Cancelled.';
      const done = await api.dbDeleteBefore(dir, cutoff(), from === '*' ? null : from);
      return `Deleted ${done.toLocaleString()} decodes. The file keeps its size until it is compacted.`;
    });

  const compact = () =>
    run('Compact', async () => {
      await api.dbVacuum(dir);
      return 'Compacted: the free space is back with the system.';
    });

  type Span = 'query' | '24h' | '7d' | '30d' | 'all' | 'custom';
  let span = $state<Span>('query');
  let cFrom = $state('');
  let cTo = $state('');
  const toLocal = (s: number) => new Date(s * 1000).toISOString().slice(0, 16);
  $effect(() => {
    if (span === 'custom' && !cFrom) {
      cFrom = toLocal(q.since > 0 ? q.since : Date.now() / 1000 - 86400);
      cTo = toLocal(q.until);
    }
  });
  /** The period of the CSV; the query's filters (band, call, ...) apply to it. */
  const csvQuery = (): Query => {
    const now = Math.floor(Date.now() / 1000);
    switch (span) {
      case '24h': return { ...q, since: now - 86400, until: now };
      case '7d': return { ...q, since: now - 7 * 86400, until: now };
      case '30d': return { ...q, since: now - 30 * 86400, until: now };
      case 'all': return { ...q, since: 0, until: now };
      case 'custom': {
        const f = Date.parse(`${cFrom}:00Z`) / 1000;
        const t = Date.parse(`${cTo}:00Z`) / 1000;
        return { ...q, since: Number.isFinite(f) ? f : 0, until: Number.isFinite(t) ? t : now };
      }
      default: return q;
    }
  };

  const exportCsv = () =>
    run('Export', async () => {
      const dest = await api.saveAs(`skimmer-decodes-${ymd(Date.now() / 1000).replaceAll('-', '')}.csv`, 'csv');
      if (!dest) return 'Cancelled.';
      const n = await api.dbExportCsv(dir, csvQuery(), dest);
      return `${n.toLocaleString()} decodes written to ${dest}`;
    });

  const backup = () =>
    run('Copy', async () => {
      const dest = await api.saveAs(`skimmer-backup-${ymd(Date.now() / 1000).replaceAll('-', '')}.db`, 'db');
      if (!dest) return 'Cancelled.';
      await api.dbBackup(dir, dest);
      return `A compact copy was written to ${dest}`;
    });
</script>

<div class="dbv">
  <section>
    <h3>File</h3>
    <p class="row">
      <span class="path" title={dir}>{dir}</span>
      <button type="button" disabled={!!busy} onclick={async () => { const f = await api.pickDb(dir); if (f) onopen(f); }}>Open another…</button>
      {#if dir !== recorded}
        <button type="button" onclick={() => onopen(null)}>Back to the recording file</button>
      {/if}
    </p>
    {#if dir !== recorded}<p class="hint">Reading another file; the skimmer records into {recorded}.</p>{/if}
  </section>
  {#if info}
    <section>
      <h3>The database</h3>
      <p>
        <b>{mb(info.bytes + info.walBytes)}</b> on disk ({mb(info.bytes)} file{info.walBytes ? `, ${mb(info.walBytes)} log` : ''}{info.reclaimable
          ? `, ${mb(info.reclaimable)} could be returned`
          : ''})
        · {info.decodes.toLocaleString()} decodes · {info.stations.toLocaleString()} stations known
        {#if info.first !== null && info.last !== null}· {stamp(info.first)} to {stamp(info.last)} UTC{/if}
      </p>
      {#if info.servers.length > 1 || (info.servers[0] && info.servers[0][0] !== '')}
        <table>
          <tbody>
            {#each info.servers as [name, n] (name)}
              <tr>
                <td>{label(name)}</td>
                <td class="num">{n.toLocaleString()} decodes</td>
              </tr>
            {/each}
          </tbody>
        </table>
      {/if}
    </section>

    <section>
      <h3>Clear out old decodes</h3>
      <p class="row">
        Older than
        <input type="number" min="0" step="30" bind:value={days} /> days
        in
        <select bind:value={from}>
          <option value="*">all servers</option>
          {#each info.servers as [name] (name)}<option value={name}>{label(name)}</option>{/each}
        </select>
        <span class="hint">{willGo === null ? '' : `${willGo.toLocaleString()} decodes`}</span>
        <button type="button" disabled={!!busy || !willGo} onclick={deleteOld}>Delete…</button>
      </p>
      <p class="row">
        <button type="button" disabled={!!busy} onclick={compact}>Compact the file</button>
        <span class="hint">Deleting does not shrink the file; compacting does. It needs the recorder to be quiet for a moment (it waits up to 20 s).</span>
      </p>
    </section>

    <section>
      <h3>Export</h3>
      <p class="row">
        Period
        <select bind:value={span}>
          <option value="query">same as the query above</option>
          <option value="24h">last 24 h</option>
          <option value="7d">last 7 days</option>
          <option value="30d">last 30 days</option>
          <option value="all">everything</option>
          <option value="custom">from – to (UTC)</option>
        </select>
        {#if span === 'custom'}
          <input type="datetime-local" bind:value={cFrom} />
          –
          <input type="datetime-local" bind:value={cTo} />
        {/if}
      </p>
      <p class="row">
        <button type="button" disabled={!!busy} onclick={exportCsv}>Decodes as CSV…</button>
        <span class="hint">The decodes of that period with the query's filters (band, call, SNR…), with bearing and distance from the server that heard them.</span>
      </p>
      <p class="row">
        <button type="button" disabled={!!busy} onclick={backup}>Copy of the database…</button>
        <span class="hint">A compact, consistent copy of everything, made while recording.</span>
      </p>
    </section>
  {/if}
  {#if busy}<p class="hint">{busy}…</p>{/if}
  {#if message}<p class="msg">{message}</p>{/if}
</div>

<style>
  .dbv {
    display: flex;
    flex-direction: column;
    gap: 14px;
    font-size: 13px;
  }
  h3 {
    margin: 0 0 4px;
    font-size: 13px;
  }
  p {
    margin: 4px 0;
  }
  .path {
    flex: 1;
    min-width: 0;
    color: var(--muted);
    font-size: 11.5px;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .row {
    display: flex;
    flex-wrap: wrap;
    gap: 8px;
    align-items: center;
  }
  .row input {
    width: 70px;
  }
  .hint {
    color: var(--muted);
    font-size: 12px;
  }
  .msg {
    padding: 6px 10px;
    background: var(--alt);
    border-radius: 6px;
  }
  table {
    border-collapse: collapse;
    margin-top: 4px;
  }
  td {
    padding: 2px 16px 2px 0;
  }
  .num {
    text-align: right;
    font-variant-numeric: tabular-nums;
  }
</style>
