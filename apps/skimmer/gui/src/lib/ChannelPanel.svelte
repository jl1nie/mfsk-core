<script lang="ts">
  import type { ChannelSetting, DepthSetting, ModeInfo } from './types';
  import { BANDS, PRESETS } from './presets';

  let {
    channels = $bindable(),
    modes,
    slotS,
    now,
    active,
    slotCounts,
    onchange,
    onoptions,
  }: {
    channels: ChannelSetting[];
    modes: ModeInfo[];
    /** Slot length in seconds by mode name. */
    slotS: Record<string, number>;
    /** Host clock, ms. */
    now: number;
    active: boolean[];
    slotCounts: number[];
    onchange: () => void;
    /** A channel's decode options changed; applied to a running skimmer. */
    onoptions: (i: number) => void;
  } = $props();

  let band = $state('40m');
  let mode = $state('FT8');
  let dialKhz = $state('');

  const same = (a: ChannelSetting, b: ChannelSetting) => a.mode === b.mode && a.dialHz === b.dialHz;

  function add(c: ChannelSetting) {
    if (channels.some((x) => same(x, c))) return;
    channels.push(c);
    onchange();
  }

  function addTyped() {
    const khz = Number(dialKhz);
    if (!Number.isFinite(khz) || khz <= 0) return;
    add({ mode, dialHz: Math.round(khz * 1000) });
    dialKhz = '';
  }

  function remove(i: number) {
    channels.splice(i, 1);
    onchange();
  }

  const num = (v: string): number | null => {
    const n = Number(v);
    return v.trim() !== '' && Number.isFinite(n) ? n : null;
  };

  function optionSummary(c: ChannelSetting): string {
    const parts: string[] = [];
    if (c.bandLo != null && c.bandHi != null) parts.push(`${c.bandLo}–${c.bandHi} Hz`);
    if (c.dxCall) parts.push(`DX ${c.dxCall}`);
    if (c.depth) parts.push(c.depth);
    return parts.join(' · ');
  }

  function channelState(i: number): string {
    if (active.length === 0) return '';
    return active[i] ? `${slotCounts[i] ?? 0}` : 'paused';
  }

  /** Where the current slot of a mode is: fraction elapsed, seconds left, start (UTC hhmmss). */
  function slot(modeName: string) {
    const period = (slotS[modeName] ?? 0) * 1000;
    if (period <= 0) return null;
    const into = now % period;
    const start = new Date(now - into);
    const z = (n: number) => String(n).padStart(2, '0');
    return {
      frac: into / period,
      left: Math.ceil((period - into) / 1000),
      start: `${z(start.getUTCHours())}${z(start.getUTCMinutes())}${z(start.getUTCSeconds())}`,
      periodS: period / 1000,
    };
  }
</script>

<section>
  <h2>Channels</h2>
  <ul class="channels">
    {#each channels as c, i (c.mode + c.dialHz)}
      {@const s = slot(c.mode)}
      <li class:paused={active.length > 0 && !active[i]}>
        <div class="line">
          <span class="mode">{c.mode}</span>
          <span class="freq">{(c.dialHz / 1000).toFixed(1)} kHz</span>
          {#if s}
            <span
              class="slotpie"
              style="--p: {(s.frac * 100).toFixed(1)}%"
              title="Slot {s.start} UTC ({s.periodS} s slots): {s.left} s to the next boundary, then this slot is decoded"
            ></span>
          {/if}
          <span class="count" title="Decodes in the latest slot, or paused: outside the radio's band">{channelState(i)}</span>
          <button class="link" onclick={() => remove(i)} aria-label="Remove">✕</button>
        </div>
        <details class="opts">
          <summary>Decode options{optionSummary(c) ? ` — ${optionSummary(c)}` : ''}</summary>
          <label title="Audio band searched. Empty: the mode's default">
            Band
            <input
              value={c.bandLo ?? ''}
              placeholder="lo"
              inputmode="decimal"
              onchange={(e) => { c.bandLo = num(e.currentTarget.value); onoptions(i); }}
            />–<input
              value={c.bandHi ?? ''}
              placeholder="hi"
              inputmode="decimal"
              onchange={(e) => { c.bandHi = num(e.currentTarget.value); onoptions(i); }}
            /> Hz
          </label>
          <label title="Hunt one station: its call is given to the decoder as an a-priori hint (FT8, FT4, FST4, Q65)">
            DX call
            <input
              value={c.dxCall ?? ''}
              placeholder="JA1ABC"
              onchange={(e) => { c.dxCall = e.currentTarget.value.trim().toUpperCase() || null; onoptions(i); }}
            />
          </label>
          <label title="WSJT-X decoding depth">
            Depth
            <select
              value={c.depth ?? ''}
              onchange={(e) => { c.depth = e.currentTarget.value as DepthSetting; onoptions(i); }}
            >
              <option value="">default (deep)</option>
              <option value="fast">fast</option>
              <option value="normal">normal</option>
              <option value="deep">deep</option>
            </select>
          </label>
        </details>
      </li>
    {:else}
      <li class="empty">No channels yet</li>
    {/each}
  </ul>

  <div class="row">
    <select bind:value={mode}>
      {#each modes as m (m.name)}<option value={m.name}>{m.name}</option>{/each}
    </select>
    <input
      bind:value={dialKhz}
      placeholder="dial kHz"
      inputmode="decimal"
      onkeydown={(e) => e.key === 'Enter' && addTyped()}
    />
    <button onclick={addTyped}>Add</button>
  </div>

  <h3>
    Presets
    <select bind:value={band}>
      {#each BANDS as b (b)}<option value={b}>{b}</option>{/each}
    </select>
  </h3>
  <ul class="presets">
    {#each PRESETS.filter((p) => p.band === band) as p (p.label)}
      <li>
        <button class="link" disabled={channels.some((x) => same(x, p.channel))} onclick={() => add({ ...p.channel })}>
          + {p.label}
        </button>
      </li>
    {/each}
  </ul>
</section>

<style>
  .opts {
    margin: 0.25rem 0 0.5rem 0.5rem;
    font-size: 0.85em;
  }
  .opts label {
    display: block;
    margin: 0.2rem 0;
  }
  .opts input {
    width: 5em;
  }
  .opts input[placeholder='JA1ABC'] {
    width: 8em;
  }
</style>
