<script lang="ts">
  import { onMount } from 'svelte';
  import * as api from './lib/api';
  import type { DecodeRow, ModeInfo, Settings, Status, UiEvent } from './lib/types';
  import { bandOfHz, clockOf, gridLonLat, sortBands, sunTimes } from './lib/analysis';
  import HoursPicker from './lib/HoursPicker.svelte';
  import ChannelPanel from './lib/ChannelPanel.svelte';
  import DecodeTable from './lib/DecodeTable.svelte';
  import AnalysisPanel from './lib/AnalysisPanel.svelte';
  import WaterfallPanel from './lib/WaterfallPanel.svelte';
  import { WaterfallStore } from './lib/waterfall';

  /** Servers at once: four, which a normal PC's threads carry (measured: sixteen busy FT8 decoders at once take a few seconds on 8 threads). */
  const MAX_SERVERS = 4;

  /** Rows kept in the table; older ones stay in the log file. */
  const MAX_ROWS = 5000;

  let settings = $state<Settings | null>(null);
  let modes = $state<ModeInfo[]>([]);
  let running = $state(false);
  /** What the window knows about one server: how it is doing, its radio, its health. */
  type Srv = {
    phase: string;
    detail: string;
    /** The rotation step in force, "2/3 until 12:30 UTC", or empty. */
    step: string;
    /** From the connected device: the highest gain index, and whether this client can set it. */
    maxGain: number | null;
    canControl: boolean;
    /** The gain the radio had when we connected (SDR# had set it), and the one moved to since. */
    deviceGain: number | null;
    gainSet: number | null;
    lastGainMove: number;
    health: Status | null;
    /** The rotation is held on the band being heard. */
    held: boolean;
    /** The step in force (an index into the steps actually run) and when it ends (ms); null: none is in. */
    stepIdx: number | null;
    stepEnds: number;
    state: 'off' | 'connecting' | 'on' | 'waiting' | 'error';
  };
  const blankSrv = (): Srv => ({
    phase: 'Stopped', detail: '', step: '', maxGain: null, canControl: false, deviceGain: null,
    gainSet: null, lastGainMove: 0, health: null, held: false, stepIdx: null, stepEnds: 0, state: 'off',
  });
  /** By index in `settings.servers`. */
  let srv = $state<Srv[]>([]);
  /** The server the gain slider, the health line and the thumbnails are about. */
  let sel = $state(0);
  const cur = $derived(srv[sel] ?? blankSrv());
  const gainShown = $derived(cur.gainSet ?? cur.deviceGain);
  function syncSrv() {
    const n = settings?.servers.length ?? 0;
    while (srv.length < n) srv.push(blankSrv());
    if (srv.length > n) srv.length = n;
    if (sel >= n) sel = Math.max(0, n - 1);
  }
  let notice = $state('');
  const health = $derived(cur.health);
  /** Per configured channel: in the current stream, or paused. */
  let active = $state<boolean[]>([]);
  // Raw, not a deep proxy: 5000 row objects are replaced wholesale, never mutated.
  let rows = $state.raw<DecodeRow[]>([]);
  const wfStore = new WaterfallStore();
  let wfTick = $state(0);
  let wfFocus = $state(0);
  /** The part of Settings shown. */
  const SETTINGS_TABS = [
    ['server', 'Server'],
    ['rotation', 'Rotation'],
    ['station', 'Station'],
    ['receiver', 'Receiver'],
    ['recording', 'Recording'],
  ] as const;
  let settingsTab = $state<(typeof SETTINGS_TABS)[number][0]>('server');
  /** The live view (waterfall + decodes) or the Analysis of the database. */
  let view = $state<'live' | 'analysis'>('live');
  const gridOk = $derived(
    !settings?.myGrid.trim() || /^[A-R]{2}\d{2}([A-X]{2})?$/i.test(settings.myGrid.trim()),
  );
  function stationChanged() {
    if (!settings) return;
    settings.myCall = settings.myCall.trim().toUpperCase();
    const g = settings.myGrid.trim();
    settings.myGrid = g.slice(0, 4).toUpperCase() + g.slice(4).toLowerCase();
    if (running) api.setStation(settings.myCall, settings.myGrid);
  }
  /** The clock line from the core: NTP offset, or why NTP is not in use. */
  let clockText = $state('');
  /** The table's channel selector (-1 = All); it and the large waterfall follow each other, except on All. */
  let tableCh = $state(-1);
  let wfOpen = $state(true);
  let nextId = 0;
  let gainTimer: ReturnType<typeof setTimeout> | undefined;
  /** The slider moves continuously; the radio gets the value once it rests for a moment. */
  function gainChanged() {
    clearTimeout(gainTimer);
    const server = sel;
    gainTimer = setTimeout(() => {
      const g = srv[server]?.gainSet;
      if (running && g !== null && g !== undefined) api.setGain(server, g);
    }, 150);
  }

  /** Decodes the window has received; compare with the table to see whether it dropped any. */
  let received = $state(0);
  /** `NTP +83 ms` from `NTP +83 ms (round trip 24 ms)`; a failure keeps its whole text. */
  const clockShort = $derived(clockText ? clockText.replace(/ \(.*\)$/, '') : 'PC clock');
  const clockBad = $derived(clockText.includes('failed') || clockText.includes('keeping'));
  /** Shown only when something is wrong; the counts since connecting. */
  const problems = $derived.by(() => {
    const h = health;
    if (!h) return [];
    const p: string[] = [];
    if (h.droppedSlots > 0) p.push(`${h.droppedSlots} slot${h.droppedSlots > 1 ? 's' : ''} dropped`);
    if (h.gaps > 0) p.push(`${h.gaps} gap${h.gaps > 1 ? 's' : ''}`);
    if (h.reanchors > 0) p.push(`${h.reanchors} re-anchor${h.reanchors > 1 ? 's' : ''}`);
    if (h.queuedBytes > 500_000 || h.queuedSlots > 2)
      p.push(`backlog ${(h.queuedBytes / 1e3).toFixed(0)} kB, ${h.queuedSlots} slots`);
    return p;
  });
  const healthDetail = $derived(
    health
      ? [
          clockText || 'PC clock',
          `arrival delay past the anchor estimate ${health.delayMs.toFixed(0)} ms, anchor drift ${health.driftMs.toFixed(0)} ms`,
          `longest push ${health.longestPushMs.toFixed(0)} ms, longest decode ${health.longestDecodeMs.toFixed(0)} ms`,
          `read queue ${(health.queuedBytes / 1e3).toFixed(0)} kB, slots queued ${health.queuedSlots} / dropped ${health.droppedSlots}`,
          `gaps ${health.gaps}, re-anchors ${health.reanchors}`,
          `decodes the window received ${received}`,
        ].join('\n')
      : '',
  );
  let pending: DecodeRow[] = [];
  let flushing = false;
  /** Per configured channel: decodes in the latest slot it reported. */
  let slotCounts = $state<number[]>([]);
  let slotOf: (number | null)[] = [];
  let serverOpen = $state(false);
  let autoPfb = $state(4);
  /** Host clock, ms; drives the slot bars. The skimmer's slot grid is anchored on the same clock. */
  let now = $state(Date.now());

  /** The bands a server's rotation runs through, in order: those its channels are in. */
  function stepBands(server: number): string[] {
    const sv = settings?.servers[server];
    if (!sv) return [];
    const have = new Set(
      (settings?.channels ?? []).filter((c) => (c.server ?? 0) === server).map((c) => bandOfHz(c.dialHz)),
    );
    return sv.rotation.filter((r) => have.has(r.band)).map((r) => r.band);
  }

  /** Sunrise and sunset today at a server, for its Settings: UTC, and the local mean solar time. */
  function sunLine(grid: string): string {
    const g = gridLonLat(grid);
    if (!g) return 'give the grid to see it';
    const t = sunTimes(g, Date.now());
    if (t.rise === null) return t.polar === 'day' ? 'the Sun does not set today' : 'the Sun does not rise today';
    const off = Math.round(g[0] / 15);
    const loc = (m: number) => clockOf((((m + off * 60) % 1440) + 1440) % 1440);
    return `↑ ${clockOf(t.rise)} ↓ ${clockOf(t.set!)} UTC · local (UTC${off >= 0 ? '+' : ''}${off}) ${loc(t.rise)} / ${loc(t.set!)}`;
  }

  const mhz = (hz: number) => (hz / 1e6).toFixed(3);
  const slotS = $derived(Object.fromEntries(modes.map((m) => [m.name, m.slotS])));
  const geom = $derived(Object.fromEntries(modes.map((m) => [m.name, m])));

  onMount(() => {
    let unlisten: (() => void) | undefined;
    const clock = setInterval(() => (now = Date.now()), 200);
    // The window asks the backend for the radio's state, so a missed or reordered
    // event (the autostart connects before the first paint) cannot leave it unread.
    const radioPoll = setInterval(async () => {
      if (!running || !settings) return;
      for (let i = 0; i < settings.servers.length; i++) {
        takeRadio(i, await api.radioState(i).catch(() => null));
      }
    }, 1000);
    (async () => {
      settings = await api.loadSettings();
      syncSrv();
      settings.servers.forEach((_, i) => syncRotation(i));
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
  function takeRadio(i: number, r: { gain: number; maxGain: number; canControl: boolean } | null) {
    const s = srv[i];
    if (!s) return;
    if (r === null) {
      s.deviceGain = null;
      s.gainSet = null;
      s.canControl = false;
      return;
    }
    s.maxGain = r.maxGain;
    s.canControl = r.canControl;
    s.deviceGain = r.gain;
    if (Date.now() - s.lastGainMove > 1500) s.gainSet = null;
  }

  /** The window's numbers of the channels one server listens to. */
  const channelsOf = (server: number) =>
    (settings?.channels ?? []).flatMap((c, i) => ((c.server ?? 0) === server ? [i] : []));

  function handle(e: UiEvent) {
    syncSrv();
    const s = srv[e.server];
    if (!s) return;
    switch (e.type) {
      case 'connecting':
        s.phase = `Connecting to ${e.address}…`;
        s.state = 'connecting';
        break;
      case 'connected':
        s.maxGain = e.maxGain;
        s.deviceGain = e.gain;
        s.gainSet = null;
        s.canControl = e.control;
        s.phase = e.control ? 'Connected with control of the radio' : 'Connected as a guest';
        s.detail = `device centre ${mhz(e.deviceHz)} MHz, band ${(e.bandwidthHz / 1e3).toFixed(0)} kHz`;
        break;
      case 'radio':
        takeRadio(e.server, e);
        break;
      case 'waterfall':
        wfStore.push(e);
        wfTick++;
        break;
      case 'yielded':
        s.phase = 'Had control of the radio: leaving it for SDR# and retrying';
        s.state = 'waiting';
        notice = 'Control was given back (Settings ⚙: "Give control back" is on). Turn it off to run alone.';
        break;
      case 'noChannelFits':
        s.phase = `No channel fits the band around ${mhz(e.deviceHz)} MHz; waiting for the radio to move`;
        s.state = 'waiting';
        for (const c of channelsOf(e.server)) active[c] = false;
        break;
      case 'streaming':
        if (settings?.waterfall) api.setWaterfall(wfFocus, settings.waterfallFine);
        s.phase = 'Decoding';
        s.state = 'on';
        // The device is not shown: the server's first word after a retune still has the old centre.
        s.detail = `IQ ${(e.rate / 1e3).toFixed(0)} kS/s at ${mhz(e.centerHz)} MHz · ${e.channelizer}`;
        e.channels.forEach((c, k) => (active[c] = e.active[k]));
        break;
      case 'step':
        {
          const until = new Date(e.endsUtcS * 1000).toISOString().slice(11, 16);
          s.held = e.held;
          s.stepIdx = e.index;
          s.stepEnds = e.endsUtcS * 1000;
          s.step = e.held
            ? `held on step ${(e.index ?? 0) + 1}/${e.of}`
            : e.index === null
              ? `no band in until ${until} UTC`
              : `step ${e.index + 1}/${e.of} until ${until} UTC`;
        }
        break;
      case 'moved':
        s.phase = `Radio moved to ${mhz(e.deviceHz)} MHz; planning again`;
        break;
      case 'decode': {
        const { type: _, server: __, ...row } = e;
        received += 1;
        addRow({ ...row, id: nextId++ });
        break;
      }
      case 'gap':
        // Counted in the health line and written to the health log; a
        // notice per gap was more noise than an operator wanted.
        break;
      case 'reanchor':
        notice = `${settings?.servers[e.server]?.name ?? 'Server'}: time re-anchored by ${e.byS >= 0 ? '+' : ''}${e.byS.toFixed(3)} s`;
        break;
      case 'clock':
        clockText = e.text;
        break;
      case 'status': {
        const { type: _, server: __, ...st } = e;
        s.health = st;
        clockText = st.clock;
        break;
      }
      case 'off':
        s.phase = 'Off';
        s.state = 'off';
        s.detail = '';
        s.step = '';
        s.held = false;
        for (const c of channelsOf(e.server)) active[c] = false;
        break;
      case 'disconnected':
        s.phase = `Disconnected (${e.error}); retrying`;
        s.state = 'error';
        for (const c of channelsOf(e.server)) active[c] = false;
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

  /** Show a channel's waterfall large; the backend sends whole rows only for it. */
  function focusChannel(i: number) {
    wfFocus = i;
    sel = settings?.channels[i]?.server ?? sel;
    wfStore.big.channel = -1;
    wfStore.big.rows = [];
    wfStore.big.utc = [];
    if (running) api.setWaterfall(i, settings?.waterfallFine ?? false);
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
      for (let i = 0; i < srv.length; i++) {
        srv[i] = { ...blankSrv() };
        takeRadio(i, null);
      }
      active = [];
      return;
    }
    if (settings.channels.length === 0) {
      notice = 'Add a channel first.';
      return;
    }
    try {
      slotCounts = settings.channels.map(() => 0);
      slotOf = [];
      wfStore.clear();
      for (let i = 0; i < srv.length; i++) srv[i] = { ...blankSrv(), phase: 'Starting…', state: 'connecting' };
      await api.start($state.snapshot(settings));
      running = true;
      if (settings.waterfall) api.setWaterfall(wfFocus, settings.waterfallFine);
    } catch (e) {
      notice = String(e);
    }
  }

  /** A channel was added or removed: a running skimmer plans again from scratch. */
  async function channelsChanged() {
    settings!.servers.forEach((_, i) => syncRotation(i));
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

  /** Choose a server: its channels, thumbnails, gain and settings are shown. */
  function selectServer(i: number) {
    sel = i;
    // The large waterfall follows to a channel of the server shown.
    const mine = channelsOf(i);
    if (mine.length && !mine.includes(wfFocus)) focusChannel(mine[0]);
  }

  /** The dot of a server: switch it off (its connection is closed) or on. */
  function toggleServer(i: number) {
    const sv = settings?.servers[i];
    if (!sv) return;
    sv.enabled = !sv.enabled;
    if (running) void api.setServerEnabled(i, sv.enabled);
    else if (!sv.enabled) srv[i] = { ...blankSrv() };
  }

  /** The rotation of a server, beside its channels: press to hold it on the band it is on, or let it go on. */
  function toggleHold(i: number) {
    if (!running || !srv[i]) return;
    const hold = !srv[i].held;
    srv[i].held = hold;
    void api.setHold(i, hold);
  }

  /** What the channel list shows of a server's rotation: the band, the time left, whether it is held. */
  function rotationInfo(server: number): { band: string; left: string; held: boolean } | null {
    const s = srv[server];
    const sv = settings?.servers[server];
    if (!s || !sv?.rotate || !running || s.state === 'off' || !sv.enabled || s.stepIdx === null) return null;
    const band = stepBands(server)[s.stepIdx] ?? `step ${s.stepIdx + 1}`;
    const left = Math.max(0, Math.round((s.stepEnds - now) / 1000));
    return { band, left: `${Math.floor(left / 60)}:${String(left % 60).padStart(2, '0')}`, held: s.held };
  }

  function addServer() {
    if (!settings || settings.servers.length >= MAX_SERVERS) return;
    const n = settings.servers.length + 1;
    settings.servers.push({
      name: `Server ${n}`, address: '', grid: '', networkDelayMs: 0, enabled: true, tune: false, yieldControl: false, rotate: false, rotation: [],
    });
    syncSrv();
    sel = settings.servers.length - 1;
    serverOpen = true;
  }

  async function confirmRemove(i: number) {
    const name = settings?.servers[i]?.name ?? 'this server';
    const n = (settings?.channels ?? []).filter((c) => (c.server ?? 0) === i).length;
    if (await api.ask(`Remove ${name}${n ? ` and its ${n} channel${n > 1 ? 's' : ''}` : ''}? Its decodes stay in the database.`)) {
      await removeServer(i);
    }
  }

  async function removeServer(i: number) {
    if (!settings || settings.servers.length < 2) return;
    settings.channels = settings.channels
      .filter((c) => (c.server ?? 0) !== i)
      .map((c) => ((c.server ?? 0) > i ? { ...c, server: (c.server ?? 0) - 1 } : c));
    settings.servers.splice(i, 1);
    syncSrv();
    await channelsChanged();
  }

  /** The rotation lists the bands the server's channels are in, in the order the user left them. */
  function syncRotation(i: number) {
    const sv = settings?.servers[i];
    if (!sv) return;
    const bands = new Set(
      (settings?.channels ?? []).filter((c) => (c.server ?? 0) === i).map((c) => bandOfHz(c.dialHz)),
    );
    const next = sv.rotation.filter((r) => bands.has(r.band));
    for (const b of sortBands(bands)) if (!next.some((r) => r.band === b)) next.push({ band: b, minutes: 6, hours: [], follow: '', marginHours: 1 });
    if (JSON.stringify(next) !== JSON.stringify(sv.rotation)) sv.rotation = next;
  }

  function toggleRotation(i: number, on: boolean) {
    settings!.servers[i].rotate = on;
    syncRotation(i);
    rotationChanged();
  }

  function moveStep(i: number, k: number, by: number) {
    const r = settings!.servers[i].rotation;
    [r[k], r[k + by]] = [r[k + by], r[k]];
    rotationChanged();
  }

  /** A rotation changed: a running skimmer plans again from scratch. */
  function rotationChanged() {
    void channelsChanged();
  }

  /** Record into a database file that exists (to add to it). */
  async function chooseDb() {
    const f = await api.pickDb(settings!.dbPath);
    if (f) settings!.dbPath = f;
  }

  /** Record into a new database file (or one to be created). */
  async function newDb() {
    const f = await api.saveAs(settings!.dbPath || 'skimmer.db', 'db');
    if (f) settings!.dbPath = f;
  }

  async function chooseLogDir() {
    const dir = await api.pickFolder(settings!.logDir);
    if (dir) settings!.logDir = dir;
  }
</script>

{#if settings}
  <header>
    <div class="server">
      <div class="srvs" role="tablist" aria-label="Servers">
        {#each settings.servers as sv, i (i)}
          <span class="srvchip" class:on={sel === i} class:off={!sv.enabled}>
            <button
              type="button"
              class="power {sv.enabled ? (srv[i]?.state ?? 'off') : 'disabled'}"
              aria-label={sv.enabled ? 'Switch this server off' : 'Switch this server on'}
              aria-pressed={sv.enabled}
              title={sv.enabled ? `${srv[i]?.phase ?? ''}\nClick to switch this server off` : 'Off. Click to switch this server on'}
              onclick={() => toggleServer(i)}
            ></button>
            <button type="button" role="tab" class="name" aria-selected={sel === i} title={sv.address} onclick={() => selectServer(i)}>{sv.name}</button>
          </span>
        {/each}
        {#if settings.servers.length < MAX_SERVERS}
          <button class="srvchip add" title="Add a server" aria-label="Add a server" onclick={addServer}>+</button>
        {/if}
      </div>
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
          <div class="stabs" role="tablist" aria-label="Settings">
            {#each SETTINGS_TABS as [id, label] (id)}
              <button type="button" role="tab" class:on={settingsTab === id} aria-selected={settingsTab === id} onclick={() => (settingsTab = id)}>{label}</button>
            {/each}
          </div>
          {#if settingsTab === 'server'}
          <h3 class="serverhead">
            Server · {settings.servers[sel]?.name}
            <button
              class="remove"
              disabled={settings.servers.length < 2}
              title={settings.servers.length < 2 ? 'At least one server stays' : 'Remove this server and its channels'}
              onclick={() => confirmRemove(sel)}>Remove</button
            >
          </h3>
          {#if settings.servers[sel]}
            {@const sv = settings.servers[sel]}
            <div class="field"><span>Name</span><input bind:value={sv.name} disabled={running} spellcheck="false" /></div>
            <div class="field"><span>Address</span><input bind:value={sv.address} disabled={running} placeholder="host:5555" spellcheck="false" /></div>
            <div class="field" title="Where this server's antenna is. Bearings and distances of what it hears are measured from here.">
              <span>Grid</span>
              <input class="grid" class:bad={!!sv.grid.trim() && !/^[A-R]{2}\d{2}([A-X]{2})?$/i.test(sv.grid.trim())} bind:value={sv.grid} placeholder={settings.myGrid || 'PM95'} spellcheck="false" disabled={running} />
            </div>
            <div class="field" title="Sunrise and sunset today at the server's locator (the Sun's upper limb on the horizon, with refraction). Mean solar time for the local one.">
              <span>Sun today</span>
              <span class="sun">{sunLine(sv.grid || settings.myGrid)}</span>
            </div>
            <div class="field" title="Fixed delay from the SDR to this PC (server buffer, path), taken off every arrival time. Zero on a LAN. If every station shows the same DT offset, enter it here.">
              <span>Network delay (ms)</span>
              <input type="number" step="10" min="0" bind:value={sv.networkDelayMs}
                onchange={() => running && api.setNetworkDelay(sel, Number(sv.networkDelayMs) || 0)} />
            </div>
            <label class="check">
              <input type="checkbox" bind:checked={sv.yieldControl} disabled={running} />
              <span>
                Give control back when nobody else is connected, for an SDR# started later. Off (default): with no other
                client the skimmer takes control and tunes the radio; beside a running SDR# it is a guest and never tunes.
              </span>
            </label>
            {/if}
          <h3>Radio · {settings.servers[sel]?.name}</h3>
          <div
            class="field slider"
            title="The radio's own gain index (SpyServer; the HF+ steps its attenuator and LNA), as the radio reports it, also when SDR# moves it. Moving the slider writes it when this client has control and shows the value written; as a guest it cannot write, and the slider keeps showing the value read. Nothing is saved."
          >
            <span>Gain</span>
            {#if gainShown !== null}
              <input
                type="range"
                min="0"
                max={cur.maxGain ?? 20}
                step="1"
                value={gainShown}
                oninput={(e) => {
                  if (cur.canControl) {
                    srv[sel].gainSet = Number(e.currentTarget.value);
                    srv[sel].lastGainMove = Date.now();
                    gainChanged();
                  } else {
                    // A guest cannot write it: show what was read.
                    e.currentTarget.value = String(gainShown);
                  }
                }}
                aria-label="Gain index"
              />
              <output>{gainShown}{cur.maxGain !== null ? ` / ${cur.maxGain}` : ''}</output>
            {:else}
              <span class="hint">{running ? 'reading…' : 'read when connected'}</span>
            {/if}
          </div>
          {#if running && !cur.canControl}
            <p class="hint">This client is a guest (SDR# has control): the gain is SDR#'s to set.</p>
          {/if}
          {:else if settingsTab === 'rotation'}
          <h3>Rotation · {settings.servers[sel]?.name}</h3>
          {#if settings.servers[sel]}
            {@const sv = settings.servers[sel]}
          <label class="check" title="One SDR holds one band at a time. Rotating gives each band of this server's channels its turn (the modes of a band are heard together). Connect begins with the first band (Settings > Rotation follows the UTC clock changes that).">
              <input type="checkbox" checked={sv.rotate} onchange={(e) => toggleRotation(sel, e.currentTarget.checked)} />
              <span>Rotate through the bands of its channels</span>
            </label>
            {#if sv.rotate}
              {#if sv.rotation.length < 1}
                <p class="hint">Needs channels.</p>
              {/if}
              {#each sv.rotation as r, k (r.band)}
                <div class="field rot">
                  <span>{r.band}</span>
                  <input type="number" min="4" step="2" value={r.minutes} title="Minutes per turn, rounded up to a whole number of the slots of its modes (WSPR: 2 min)"
                    onchange={(e) => { r.minutes = Math.max(4, Number(e.currentTarget.value) || 6); rotationChanged(); }} />
                  <span class="hint">min</span>
                  <button class="link" aria-label="Earlier" disabled={k === 0} onclick={() => moveStep(sel, k, -1)}>▲</button>
                  <button class="link" aria-label="Later" disabled={k === sv.rotation.length - 1} onclick={() => moveStep(sel, k, 1)}>▼</button>
                </div>
                <div class="rothours">
                  <HoursPicker
                    hours={r.hours}
                    follow={r.follow ?? ''}
                    margin={r.marginHours ?? 1}
                    grid={sv.grid || settings.myGrid}
                    onchange={(v) => { r.hours = v.hours; r.follow = v.follow; r.marginHours = v.margin; rotationChanged(); }}
                  />
                </div>
              {/each}
              <p class="hint">At least 4 minutes each, rounded up to a whole number of the slots of the band's modes (WSPR's are 2 minutes, so 5 becomes 6); a retune costs a slot or two. Click the hours (UTC) a band takes part in, one by one, or pick a preset. The cycle goes on among the bands that are in; when none is, nothing is heard. Add or remove channels to change the bands.</p>
            {/if}
          {/if}
          <label class="check" title="Off (default): Connect starts a rotation at its first band. On: the cycle counts from UTC midnight, so a restart, or another server with the same steps, is at the same step at the same moment.">
            <input type="checkbox" bind:checked={settings.rotationUtc} disabled={running} />
            <span>Rotation follows the UTC clock (otherwise it begins with the first band)</span>
          </label>
          {:else if settingsTab === 'station'}
          <h3>Station</h3>
          <div class="field" title="Your callsign and locator. Used for the QSO-context a-priori decoding, and as the centre of the maps and the origin of every bearing. A channel can override them in its options.">
            <span>My call</span>
            <input
              class="call"
              bind:value={settings.myCall}
              placeholder="JL1NIE"
              spellcheck="false"
              onchange={stationChanged}
            />
          </div>
          <div class="field" title="4 or 6 characters, e.g. PM95 or PM95tl">
            <span>My grid</span>
            <input
              class="grid"
              class:bad={!gridOk}
              bind:value={settings.myGrid}
              placeholder="PM95"
              spellcheck="false"
              onchange={stationChanged}
            />
          </div>
          <h3>Clock</h3>
          <div class="field" title="The skimmer stamps IQ with UTC. The PC clock is not changed: the offset to the NTP server is added.">
            <span>Clock</span>
            <select bind:value={settings.clockSource} disabled={running}>
              <option value="system">PC clock</option>
              <option value="ntp">NTP</option>
            </select>
            <input bind:value={settings.ntpServer} disabled={running || settings.clockSource !== 'ntp'} spellcheck="false" />
          </div>
          {:else if settingsTab === 'receiver'}
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
          <h3>Waterfall</h3>
          <label class="check" title="A fine spectrum of each channel's audio (2.9 Hz per bin) under the channel list, with the decodes marked on it">
            <input type="checkbox" bind:checked={settings.waterfall} disabled={running} />
            <span>Waterfall</span>
          </label>
          <label class="check" title="8192-point FFT for the channel shown large: 1.5 Hz per bin, 0.34 s per row">
            <input
              type="checkbox"
              bind:checked={settings.waterfallFine}
              disabled={!settings.waterfall}
              onchange={() => running && api.setWaterfall(wfFocus, settings!.waterfallFine)}
            />
            <span>Fine (1.5 Hz per bin)</span>
          </label>
          {:else}
          <h3>Recording</h3>
          <label class="check" title="Every decode in a database file, indexed, for the Analysis view.">
            <input type="checkbox" bind:checked={settings.dbEnabled} disabled={running} />
            <span>Keep decodes in a database</span>
          </label>
          <div class="folder" class:off={!settings.dbEnabled} title="The database file decodes are recorded into">
            <span class="path" title={settings.dbPath}>{settings.dbPath}</span>
            <button onclick={chooseDb} disabled={running || !settings.dbEnabled}>Open…</button>
            <button onclick={newDb} disabled={running || !settings.dbEnabled}>New…</button>
          </div>
          <div class="folder" title="Where STATUS.log is written (a line per health event, for a long run)">
            <span class="path" title={settings.logDir}>{settings.logDir}</span>
            <button onclick={chooseLogDir} disabled={running}>Choose…</button>
          </div>
          {/if}
          {#if running}<p class="hint">Disconnect to change these.</p>{/if}
        </div>
      {/if}
    </div>
    <button class:primary={!running} onclick={startStop}>{running ? 'Disconnect' : 'Connect'}</button>
    <div class="state">
      <div class="phase">
        <span class="ph">{cur.phase}</span>
        {#if health}
          <span class="health" title={healthDetail}>
            <span class:warn={clockBad}>{clockShort}</span>
            <span>delay {health.delayMs.toFixed(0)} ms · drift {health.driftMs >= 0 ? '+' : ''}{health.driftMs.toFixed(0)} ms</span>
            <span>decode {health.longestDecodeMs.toFixed(0)} ms</span>
            {#each problems as p (p)}<span class="warn">{p}</span>{/each}
          </span>
        {/if}
      </div>
      <div class="detail">{cur.detail}</div>
    </div>
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
        servers={settings.servers}
        {sel}
        {rotationInfo}
        onhold={toggleHold}
        onmove={(server, by) => api.rotateBand(server, by)}
        {modes}
        {slotS}
        {now}
        {active}
        {slotCounts}
        onchange={channelsChanged}
        onoptions={channelOptionsChanged}
        stationCall={settings.myCall}
        stationGrid={settings.myGrid}
      />

    </aside>

    <div class="rightcol">
      <div class="views" role="tablist">
        <button role="tab" class:on={view === 'live'} aria-selected={view === 'live'} onclick={() => (view = 'live')}>
          {settings.waterfall ? 'Waterfall' : 'Decodes'}
        </button>
        <button
          role="tab"
          class:on={view === 'analysis'}
          aria-selected={view === 'analysis'}
          title="Maps and statistics of everything recorded in the database"
          onclick={() => (view = 'analysis')}>Analysis</button
        >
      </div>
      {#if view === 'analysis'}
        <AnalysisPanel dir={settings.dbPath} me={settings.myGrid.trim()} />
      {/if}
      <!-- Kept mounted behind the Analysis view, so its filters and scroll survive. -->
      <div class="live" style:display={view === 'analysis' ? 'none' : 'contents'}>
      {#if settings.waterfall}
        <WaterfallPanel
          store={wfStore}
          tick={wfTick}
          channels={settings.channels}
          server={sel}
          servers={settings.servers.map((sv, i) => ({ name: sv.name, state: sv.enabled ? (srv[i]?.state ?? 'off') : 'disabled', step: '', held: false }))}
          onserver={selectServer}
          focus={Math.min(wfFocus, Math.max(0, settings.channels.length - 1))}
          onfocus={(i) => {
            focusChannel(i);
            tableCh = i;
          }}
          {rows}
          {slotS}
          {geom}
          bind:open={wfOpen}
        />
      {/if}
      <DecodeTable
        {rows}
        channels={settings.channels}
        serverNames={settings.servers.map((x) => x.name)}
        {slotS}
        onclear={clearRows}
        bind:channel={tableCh}
        onpick={focusChannel}
      />
      </div>
    </div>
  </main>
{:else}
  <p class="loading">Loading…</p>
{/if}
