<script lang="ts">
  import { tick } from 'svelte';
  import type { ChannelSetting, JttyRow } from './types';

  let {
    rows,
    channels,
    onclear,
  }: {
    rows: JttyRow[];
    channels: ChannelSetting[];
    onclear: () => void;
  } = $props();

  let search = $state('');
  let follow = $state(true);
  let box: HTMLDivElement | undefined = $state();

  const needle = $derived(search.trim().toUpperCase());
  const shown = $derived(
    rows.filter((r) => !needle || r.text.toUpperCase().includes(needle) || r.calls.some((c) => c.includes(needle))),
  );

  const hhmmss = (ms: number | null) => {
    if (ms === null) return '------';
    const d = new Date(ms);
    const z = (n: number) => String(n).padStart(2, '0');
    return `${z(d.getUTCHours())}${z(d.getUTCMinutes())}${z(d.getUTCSeconds())}`;
  };

  /** How a message stands: still being received, or how it ended. */
  const standing = (r: JttyRow) =>
    r.kind === 'growing'
      ? { mark: '…', tip: 'Still being received' }
      : r.kind === 'complete'
        ? { mark: '', tip: 'Complete: its last frame arrived' }
        : r.kind === 'expired'
          ? { mark: '✂', tip: 'No continuation came within three frames: given up on' }
          : { mark: '✂', tip: 'Cut off: the reception ended (a retune, a gap or a stop)' };

  // Keep the newest message in view unless the user has scrolled up; a growing
  // message changes its text, not the row count, so follow the content.
  $effect(() => {
    void shown.map((r) => r.text.length).join();
    if (follow && box) tick().then(() => box && (box.scrollTop = box.scrollHeight));
  });
  const onscroll = () => {
    if (box) follow = box.scrollHeight - box.scrollTop - box.clientHeight < 24;
  };
</script>

<div class="decodes">
  <div class="bar">
    <input class="search" bind:value={search} placeholder="Search call or text" spellcheck="false" />
    <span class="count">{shown.length} messages</span>
    <button class="link" onclick={onclear} title="Clear the list (ALL.TXT keeps every finished message)">Clear</button>
    {#if !follow}
      <button class="link" onclick={() => (follow = true)}>Follow ↓</button>
    {/if}
  </div>

  <div class="table" bind:this={box} {onscroll}>
    <table>
      <thead>
        <tr><th>UTC</th><th class="num">MHz</th><th class="num">dB</th><th>Message</th></tr>
      </thead>
      <tbody>
        {#each shown as r (r.channel + ':' + r.key)}
          <tr class:cq={r.text.startsWith('CQ ')} class:growing={r.kind === 'growing'} title={channels[r.channel] ? `${channels[r.channel].mode} ${(channels[r.channel].dialHz / 1000).toFixed(1)} kHz` : ''}>
            <td class="mono">{hhmmss(r.startUtcMs)}</td>
            <td class="num mono">{(r.freqHz / 1e6).toFixed(4)}</td>
            <td class="num mono">{r.snrDb.toFixed(0)}</td>
            <td class="mono msg">{r.text}{#if standing(r).mark}<span class="jstate" title={standing(r).tip}> {standing(r).mark}</span>{/if}</td>
          </tr>
        {/each}
      </tbody>
    </table>
  </div>
</div>

<style>
  .jstate {
    color: var(--muted);
    cursor: help;
  }
  tr.growing .msg {
    color: var(--muted);
  }
</style>
