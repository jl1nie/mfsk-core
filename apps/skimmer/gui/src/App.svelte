<script lang="ts">
  import { onMount } from 'svelte';
  import * as api from './lib/api';
  import type { DecodeRow, ModeInfo, Settings, Status, UiEvent } from './lib/types';
  import ChannelPanel from './lib/ChannelPanel.svelte';
  import DecodeTable from './lib/DecodeTable.svelte';

  /** Rows kept in the table; older ones stay in the log file. */
  const MAX_ROWS = 5000;

  let settings = $state<Settings | null>(null);
  let modes = $state<ModeInfo[]>([]);
  let running = $state(false);
  let phase = $state('Stopped');
  /** From the connected device: the highest gain index, and whether this client can set it. */
  let maxGain = $state<number | null>(null);
  let canControl = $state(false);
  /** The gain the radio had when we connected (SDR# had set it), and the one moved to since. */
  let deviceGain = $state<number | null>(null);
  let gainSet = $state<number | null>(null);
  const gainShown = $derived(gainSet ?? deviceGain);
  let lastGainMove = 0;
  let detail = $state('');
  let notice = $state('');
  let health = $state<Status | null>(null);
  /** Per configured channel: in the current stream, or paused. */
  let active = $state<boolean[]>([]);
  // Raw, not a deep proxy: 5000 row objects are replaced wholesale, never mutated.
  let rows = $state.raw<DecodeRow[]>([]);
  let nextId = 0;
  let gainTimer: ReturnType<typeof setTimeout> | undefined;
  /** The slider moves continuously; the radio gets the value once it rests for a moment. */
  function gainChanged() {
    clearTimeout(gainTimer);
    gainTimer = setTimeout(() => {
      if (running && gainSet !== null) api.setGain(gainSet);
    }, 150);
  }

  /** Decodes the window has received; compare with the table to see whether it dropped any. */
  let received = $state(0);
  let pending: DecodeRow[] = [];
  let flushing = false;
  /** Per configured channel: decodes in the latest slot it reported. */
  let slotCounts = $state<number[]>([]);
  let slotOf: (number | null)[] = [];
  let serverOpen = $state(false);
  let autoPfb = $state(4);
  /** Host clock, ms; drives the slot bars. The skimmer's slot grid is anchored on the same clock. */
  let now = $state(Date.now());

  const mhz = (hz: number) => (hz / 1e6).toFixed(3);
  const slotS = $derived(Object.fromEntries(modes.map((m) => [m.name, m.slotS])));

  onMount(() => {
    let unlisten: (() => void) | undefined;
    const clock = setInterval(() => (now = Date.now()), 200);
    // The window asks the backend for the radio's state, so a missed or reordered
    // event (the autostart connects before the first paint) cannot leave it unread.
    const radioPoll = setInterval(async () => {
      if (running) takeRadio(await api.radioState().catch(() => null));
    }, 1000);
    (async () => {
      settings = await api.loadSettings();
      modes = await api.modes();
      autoPfb = await api.autoPfbChannels();
      unlisten = await api.onEvent(handle);
      if (await api.autostartRequested()) await startStop();
    })();
    return () => {
      clearInterval(clock);
      clearInterval(radioPoll);
      unlisten?.();
    };
  });

  // Every change to the settings is saved, a moment after the last one.
  let saveTimer: ReturnType<typeof setTimeout> | undefined;
  $effect(() => {
    if (!settings) return;
    const snapshot = $state.snapshot(settings);
    clearTimeout(saveTimer);
    saveTimer = setTimeout(() => api.saveSettings(snapshot).catch((e) => (notice = String(e))), 400);
  });

  /** The radio's state as the backend last heard it (event or poll). A slider the user is
   * dragging is left alone for a moment so a reading does not pull it back. */
  function takeRadio(r: { gain: number; maxGain: number; canControl: boolean } | null) {
    if (r === null) {
      deviceGain = null;
      gainSet = null;
      canControl = false;
      return;
    }
    maxGain = r.maxGain;
    canControl = r.canControl;
    deviceGain = r.gain;
    if (Date.now() - lastGainMove > 1500) gainSet = null;
  }

  function handle(e: UiEvent) {
    switch (e.type) {
      case 'connecting':
        phase = `Connecting to ${e.server}…`;
        break;
      case 'connected':
        maxGain = e.maxGain;
        deviceGain = e.gain;
        gainSet = null;
        canControl = e.control;
        phase = e.control ? 'Connected with control of the radio' : 'Connected as a guest';
        detail = `device centre ${mhz(e.deviceHz)} MHz, band ${(e.bandwidthHz / 1e3).toFixed(0)} kHz`;
        break;
      case 'radio':
        takeRadio(e);
        break;
      case 'yielded':
        phase = 'Had control of the radio: leaving it for SDR# and retrying';
        notice = 'Control was given back (Settings ⚙: "Give control back" is on). Turn it off to run alone.';
        break;
      case 'noChannelFits':
        phase = `No channel fits the band around ${mhz(e.deviceHz)} MHz; waiting for the radio to move`;
        active = active.map(() => false);
        break;
      case 'streaming':
        phase = 'Decoding';
        detail = `IQ ${(e.rate / 1e3).toFixed(0)} kS/s at ${mhz(e.centerHz)} MHz · device ${mhz(e.deviceHz)} MHz · ${e.channelizer}`;
        active = e.active;
        break;
      case 'moved':
        phase = `Radio moved to ${mhz(e.deviceHz)} MHz; planning again`;
        break;
      case 'decode': {
        const { type: _, ...row } = e;
        received += 1;
        addRow({ ...row, id: nextId++ });
        break;
      }
      case 'gap':
        // Counted in the health line and written to the health log; a
        // notice per gap was more noise than an operator wanted.
        break;
      case 'reanchor':
        notice = `Time re-anchored by ${e.byS >= 0 ? '+' : ''}${e.byS.toFixed(3)} s`;
        break;
      case 'status': {
        const { type: _, ...s } = e;
        health = s;
        break;
      }
      case 'disconnected':
        phase = `Disconnected (${e.error}); retrying`;
        active = active.map(() => false);
        break;
    }
  }

  /**
   * One table update per tick, however many decodes arrived in it. The
   * per-channel counts are updated here, from the rows the table gets, so the
   * two cannot disagree. A timer, not requestAnimationFrame: a window that is
   * covered or minimised gets no animation frames, and the table would stand
   * still while the decoder ran.
   */
  function flushRows() {
    flushing = false;
    if (pending.length === 0) return;
    const batch = pending;
    pending = [];
    for (const r of batch) {
      while (slotCounts.length <= r.channel) slotCounts.push(0);
      if (slotOf[r.channel] !== r.slotUtcMs) {
        slotOf[r.channel] = r.slotUtcMs;
        slotCounts[r.channel] = 0;
      }
      slotCounts[r.channel] += 1;
    }
    const all = rows.concat(batch);
    rows = all.length > MAX_ROWS ? all.slice(-MAX_ROWS) : all;
  }

  function clearRows() {
    pending = [];
    rows = [];
  }

  function addRow(r: DecodeRow) {
    pending.push(r);
    if (!flushing) {
      flushing = true;
      setTimeout(flushRows, 100);
    }
  }

  async function startStop() {
    if (!settings) return;
    notice = '';
    serverOpen = false;
    if (running) {
      // The skimmer's thread has ended when this returns; events it sent
      // before that are already queued ahead of the state below.
      await api.stop();
      running = false;
      phase = 'Stopped';
      detail = '';
      active = [];
      takeRadio(null);
      return;
    }
    if (settings.channels.length === 0) {
      notice = 'Add a channel first.';
      return;
    }
    try {
      slotCounts = settings.channels.map(() => 0);
      slotOf = [];
      health = null;
      await api.start($state.snapshot(settings));
      running = true;
    } catch (e) {
      notice = String(e);
    }
  }

  /** A channel was added or removed: a running skimmer plans again from scratch. */
  async function channelsChanged() {
    slotCounts = settings!.channels.map(() => 0);
    slotOf = [];
    active = [];
    if (!running) return;
    await api.stop();
    await api.start($state.snapshot(settings!));
    running = true;
  }

  /** Decode options only: no restart; the running skimmer's decoder takes them at its next slot. */
  async function channelOptionsChanged(i: number) {
    if (!running) return;
    await api.setChannelOptions(i, $state.snapshot(settings!.channels[i]));
  }

  async function chooseLogDir() {
    const dir = await api.pickFolder(settings!.logDir);
    if (dir) settings!.logDir = dir;
  }
</script>

{#if settings}
  <header>
    <div class="server">
      <label>
        SpyServer
        <input bind:value={settings.server} disabled={running} placeholder="host:5555" spellcheck="false" />
      </label>
      <button
        class="icon"
        class:on={serverOpen}
        title="Settings"
        aria-label="Settings"
        onclick={() => (serverOpen = !serverOpen)}>⚙</button
      >
      {#if serverOpen}
        <div class="popover">
          <h2>Settings</h2>
          <h3>Radio</h3>
          <div
            class="field slider"
            title="The radio's own gain index (SpyServer; the HF+ steps its attenuator and LNA), as the radio reports it, also when SDR# moves it. Moving the slider writes it when this client has control and shows the value written; as a guest it cannot write, and the slider keeps showing the value read. Nothing is saved."
          >
            <span>Gain</span>
            {#if gainShown !== null}
              <input
                type="range"
                min="0"
                max={maxGain ?? 20}
                step="1"
                value={gainShown}
                oninput={(e) => {
                  if (canControl) {
                    gainSet = Number(e.currentTarget.value);
                    lastGainMove = Date.now();
                    gainChanged();
                  } else {
                    // A guest cannot write it: show what was read.
                    e.currentTarget.value = String(gainShown);
                  }
                }}
                aria-label="Gain index"
              />
              <output>{gainShown}{maxGain !== null ? ` / ${maxGain}` : ''}</output>
            {:else}
              <span class="hint">{running ? 'reading…' : 'read when connected'}</span>
            {/if}
          </div>
          {#if running && !canControl}
            <p class="hint">This client is a guest (SDR# has control): the gain is SDR#'s to set.</p>
          {/if}
          <h3>Connection</h3>
          <div class="field">
            <span>IQ format</span>
            <select bind:value={settings.format} disabled={running}>
              <option value="float">float32</option>
              <option value="int16">int16</option>
            </select>
          </div>
          <div class="field">
            <span>Channelizer</span>
            <select bind:value={settings.channelizer} disabled={running}>
              <option value="auto">Auto</option>
              <option value="direct">Direct</option>
              <option value="pfb">Filter bank</option>
            </select>
          </div>
          <p class="hint">
            Auto uses the filter bank from {autoPfb} channels in the stream, direct below that: the measured break-even
            (direct costs about 0.9 % of a core per channel at 768 kS/s, the bank a fixed 2.4 % plus 0.25 % per channel).
          </p>
          <label class="check">
            <input type="checkbox" bind:checked={settings.yieldControl} disabled={running} />
            <span>
              Give control back when nobody else is connected, for an SDR# started later. Off (default): with no other
              client the skimmer takes control and tunes the radio; beside a running SDR# it is a guest and never tunes.
            </span>
          </label>
          <label class="check">
            <input type="checkbox" bind:checked={settings.logEnabled} disabled={running} />
            <span>Write ALL.TXT</span>
          </label>
          <div class="folder" class:off={!settings.logEnabled}>
            <span class="path" title={settings.logDir}>{settings.logDir}</span>
            <button onclick={chooseLogDir} disabled={running || !settings.logEnabled}>Choose…</button>
          </div>
          {#if running}<p class="hint">Disconnect to change these.</p>{/if}
        </div>
      {/if}
    </div>
    <button class:primary={!running} onclick={startStop}>{running ? 'Disconnect' : 'Connect'}</button>
    <div class="state">
      <div class="phase">{phase}</div>
      <div class="detail">{detail}</div>
    </div>
    {#if health}
      <div class="health" title="Arrival delay · anchor drift · longest push · longest decode · read queue · slots queued/dropped · gaps · re-anchors">
        delay {health.delayMs.toFixed(0)} ms · drift {health.driftMs >= 0 ? '+' : ''}{health.driftMs.toFixed(0)} ms ·
        push {health.longestPushMs.toFixed(0)} ms · decode {health.longestDecodeMs.toFixed(0)} ms · queue {(health.queuedBytes / 1e3).toFixed(0)} kB ·
        slots {health.queuedSlots}/{health.droppedSlots} ·
        {health.gaps} gap · {health.reanchors} re-anchor · window got {received}
      </div>
    {/if}
  </header>
  {#if notice}
    <div class="notice">
      {notice}
      <button class="link" onclick={() => (notice = '')}>dismiss</button>
    </div>
  {/if}

  <main>
    <aside>
      <ChannelPanel
        bind:channels={settings.channels}
        {modes}
        {slotS}
        {now}
        {active}
        {slotCounts}
        onchange={channelsChanged}
        onoptions={channelOptionsChanged}
        bind:myCall={settings.myCall}
        bind:myGrid={settings.myGrid}
        onstation={() => running && api.setStation(settings!.myCall, settings!.myGrid)}
      />

    </aside>

    <DecodeTable {rows} channels={settings.channels} {slotS} onclear={clearRows} />
  </main>
{:else}
  <p class="loading">Loading…</p>
{/if}
