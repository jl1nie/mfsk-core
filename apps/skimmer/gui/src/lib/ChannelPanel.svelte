<script lang="ts">
  import type { ChannelSetting, ModeInfo } from './types';
  import { BANDS, PRESETS } from './presets';

  let {
    channels = $bindable(),
    modes,
    slotS,
    now,
    active,
    slotCounts,
    onchange,
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
