<script lang="ts">
  import { tick } from 'svelte';
  import type { ChannelSetting, DecodeRow } from './types';

  let {
    rows,
    channels,
    serverNames,
    slotS,
    onclear,
    channel = $bindable(-1),
    onpick,
  }: {
    rows: DecodeRow[];
    channels: ChannelSetting[];
    /** Names of the servers, when there is more than one (the table then says which heard a row). */
    serverNames: string[];
    slotS: Record<string, number>;
    onclear: () => void;
    channel?: number;
    /** The user chose a channel (not All) in the selector. */
    onpick?: (i: number) => void;
  } = $props();

  // One row shape for every mode: UTC, mode, frequency, SNR, DT, message.
  let cqOnly = $state(false);
  let search = $state('');
  let follow = $state(true);
  /** Extra columns: what the decoder knew of each row. */
  let detail = $state(false);
  let box: HTMLDivElement | undefined = $state();

  const needle = $derived(search.trim().toUpperCase());
  const isCq = (t: string) => t.startsWith('CQ ');
  /** The hover text of a row: sync score, errors the FEC corrected, and the key that identifies the message. */
  const tip = (r: DecodeRow) =>
    (r.syncScore === null ? 'no sync score' : `sync ${r.syncScore.toFixed(1)}`) +
    (r.hardErrors === null ? '' : ` · ${r.hardErrors} error${r.hardErrors === 1 ? '' : 's'} corrected`) +
    (r.hashResolved ? ' · a <...> resolved from the callsign table' : '') +
    (r.copiedLastTx ? ' · copied last Tx' : '') +
    (r.early ? ' · decoded early, at ~11.8 s' : '') +
    (r.key ? ` · key ${r.key}` : '');

  const shown = $derived(
    rows.filter(
      (r) =>
        (channel < 0 || r.channel === channel) &&
        (!cqOnly || isCq(r.text)) &&
        (needle === '' || r.text.toUpperCase().includes(needle)),
    ),
  );

  /**
   * The background band of a row: which 15 s period its slot *ends* in, so
   * rows decoded together share a colour whatever their mode. FT4's two
   * half-slots fall in the same period as the FT8 slot they overlap, and a
   * WSPR or JT65 slot in the period it is decoded with.
   */
  const BAND_MS = 15_000;
  const bandOf = (r: DecodeRow) => {
    if (r.slotUtcMs === null) return 0;
    const end = r.slotUtcMs + (slotS[r.mode] ?? 15) * 1000;
    return Math.floor((end - 1) / BAND_MS) % 2;
  };
  /** A heading row wherever the channel or the slot changes. */
  const heading = (i: number) =>
    i === 0 || shown[i - 1].slotUtcMs !== shown[i].slotUtcMs || shown[i - 1].channel !== shown[i].channel;

  const hhmmss = (ms: number | null) => {
    if (ms === null) return '------';
    const d = new Date(ms);
    const z = (n: number) => String(n).padStart(2, '0');
    return `${z(d.getUTCHours())}${z(d.getUTCMinutes())}${z(d.getUTCSeconds())}`;
  };

  // Keep the newest row in view unless the user has scrolled up.
  // Keyed on the newest row, not the count: at the row cap the count stops
  // changing and the table would stop following.
  $effect(() => {
    void shown[shown.length - 1]?.id;
    if (!follow || !box) return;
    tick().then(() => box && (box.scrollTop = box.scrollHeight));
  });

  function onscroll() {
    if (!box) return;
    follow = box.scrollTop + box.clientHeight >= box.scrollHeight - 4;
  }
</script>

<div class="decodes">
  <div class="toolbar">
    <select bind:value={channel} onchange={() => channel >= 0 && onpick?.(channel)}>
      <option value={-1}>All channels</option>
      {#each channels as c, i (`${c.server ?? 0}:${c.mode}${c.dialHz}`)}
        <option value={i}>{serverNames.length > 1 ? `${serverNames[c.server ?? 0]} · ` : ''}{c.mode} {(c.dialHz / 1000).toFixed(1)} kHz</option>
      {/each}
    </select>
    <label class="check"><input type="checkbox" bind:checked={cqOnly} /> CQ only</label>
    <label class="check" title="Show the sync score and the errors the FEC corrected for each row, and ⏱ on a row decoded early (~11.8 s into the slot)"><input type="checkbox" bind:checked={detail} /> Detail</label>
    <input class="search" bind:value={search} placeholder="Search call or text" spellcheck="false" />
    <span class="count">{shown.length} rows</span>
    <button class="link" onclick={onclear} title="Clear the table (ALL.TXT keeps every decode)">Clear</button>
    {#if !follow}
      <button class="link" onclick={() => (follow = true)}>Follow ↓</button>
    {/if}
  </div>

  <div class="table" bind:this={box} {onscroll}>
    <table>
      <thead>
        <tr><th>UTC</th>{#if serverNames.length > 1}<th>Server</th>{/if}<th>Mode</th><th class="num">MHz</th><th class="num">dB</th><th class="num">DT</th>{#if detail}<th class="num">Sync</th><th class="num">Err</th>{/if}<th>Message</th></tr>
      </thead>
      <tbody>
        {#each shown as r, i (r.id)}
          {#if heading(i)}
            <tr class="slot" class:alt={bandOf(r) === 1}>
              <td colspan={(serverNames.length > 1 ? 7 : 6) + (detail ? 2 : 0)}>{serverNames.length > 1 ? `${serverNames[channels[r.channel]?.server ?? 0]} · ` : ''}{r.mode} {(r.dialHz / 1000).toFixed(1)} kHz · slot {hhmmss(r.slotUtcMs)} UTC</td>
            </tr>
          {/if}
          <tr class:alt={bandOf(r) === 1} class:cq={isCq(r.text)} title={tip(r)}>
            <td class="mono">{hhmmss(r.slotUtcMs)}</td>
            {#if serverNames.length > 1}<td>{serverNames[channels[r.channel]?.server ?? 0] ?? ''}</td>{/if}
            <td>{r.mode}</td>
            <td class="num mono">{(r.freqHz / 1e6).toFixed(4)}</td>
            <td class="num mono">{r.snrDb.toFixed(0)}</td>
            <td class="num mono">{r.dtS.toFixed(1)}</td>
            {#if detail}<td class="num mono">{r.syncScore === null ? '–' : r.syncScore.toFixed(1)}</td><td class="num mono">{r.hardErrors ?? '–'}</td>{/if}
            <td class="mono msg">{r.text}{#if r.hashResolved}<span class="resolved" title="A &lt;...&gt; in this row was resolved from the callsign table"> ✓</span>{/if}{#if detail && r.early}<span class="early" title="Decoded early, at ~11.8 s into the slot, before the slot ended"> ⏱</span>{/if}</td>
          </tr>
        {/each}
      </tbody>
    </table>
  </div>
</div>
