<script lang="ts">
  import type { ApSetting, ChannelSetting, DepthSetting, ModeInfo, ServerSetting } from './types';
  import { BANDS, PRESETS } from './presets';

  let {
    channels = $bindable(),
    servers,
    sel,
    modes,
    slotS,
    now,
    active,
    slotCounts,
    onchange,
    onoptions,
    stationCall,
    stationGrid,
  }: {
    channels: ChannelSetting[];
    servers: ServerSetting[];
    /** The server new channels go to. */
    sel: number;
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
    /** The operator's call and locator from Settings: what a channel without its own uses. */
    stationCall: string;
    stationGrid: string;
  } = $props();

  let band = $state('40m');
  let mode = $state('FT8');
  let dialKhz = $state('');

  const same = (a: ChannelSetting, b: ChannelSetting) =>
    (a.server ?? 0) === (b.server ?? 0) && a.mode === b.mode && a.dialHz === b.dialHz;

  function add(c: ChannelSetting) {
    c = { ...c, server: sel };
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
    if (c.rxFreqHz != null) parts.push(`Rx ${c.rxFreqHz} Hz`);
    if (c.depth) parts.push(c.depth);
    if (c.ap) parts.push(`AP ${c.ap}`);
    if (c.dxCall) parts.push(`DX ${c.dxCall}`);
    if (c.hisCall) parts.push(`QSO ${c.hisCall}`);
    if (c.averaging) parts.push('avg');
    return parts.join(' · ');
  }

  // The channel being edited, as strings until Apply.
  let dialog: HTMLDialogElement | undefined = $state();
  let editing = $state<number | null>(null);
  type Form = {
    lo: string; hi: string; rx: string; tol: string; tx: string;
    depth: DepthSetting; ap: ApSetting; dx: string;
    hisCall: string; hisGrid: string; progress: string; contest: string;
    myCall: string; myGrid: string;
    averaging: boolean; deepSearch: boolean; emeDelay: boolean;
  };
  const blank = (): Form => ({
    lo: '', hi: '', rx: '', tol: '', tx: '', depth: '', ap: '', dx: '',
    hisCall: '', hisGrid: '', progress: '', contest: '', myCall: '', myGrid: '', averaging: false, deepSearch: false, emeDelay: false,
  });
  let f = $state<Form>(blank());
  let eError = $state('');

  const str = (v: number | null | undefined) => (v != null ? String(v) : '');

  function openOptions(i: number) {
    const c = channels[i];
    editing = i;
    f = {
      lo: str(c.bandLo), hi: str(c.bandHi), rx: str(c.rxFreqHz), tol: str(c.tolHz), tx: str(c.txFreqHz),
      depth: c.depth ?? '', ap: c.ap ?? '', dx: c.dxCall ?? '',
      hisCall: c.hisCall ?? '', hisGrid: c.hisGrid ?? '', progress: c.progress ?? '', contest: c.contest ?? '',
      myCall: c.myCall ?? '', myGrid: c.myGrid ?? '',
      averaging: !!c.averaging, deepSearch: !!c.deepSearch, emeDelay: !!c.emeDelay,
    };
    eError = '';
    dialog?.showModal();
  }

  /** A number field: empty is unset, anything else must be a number >= 0. */
  function field(v: string, what: string): number | null | undefined {
    if (v.trim() === '') return null;
    const n = Number(v);
    if (!Number.isFinite(n) || n < 0) {
      eError = `${what} must be a number of Hz, 0 or more.`;
      return undefined;
    }
    return n;
  }

  function applyOptions() {
    if (editing === null) return;
    eError = '';
    const lo = field(f.lo, 'Band low');
    const hi = field(f.hi, 'Band high');
    const rx = field(f.rx, 'Rx frequency');
    const tol = field(f.tol, 'Tolerance');
    const tx = field(f.tx, 'Tx frequency');
    if ([lo, hi, rx, tol, tx].includes(undefined)) return;
    if ((lo === null) !== (hi === null) || (lo != null && hi != null && !(lo < hi))) {
      eError = 'Band needs both ends, low below high, or neither.';
      return;
    }
    const c = channels[editing];
    c.bandLo = lo; c.bandHi = hi; c.rxFreqHz = rx; c.tolHz = tol; c.txFreqHz = tx;
    c.depth = f.depth; c.ap = f.ap;
    c.dxCall = f.dx.trim().toUpperCase() || null;
    c.hisCall = f.hisCall.trim().toUpperCase() || null;
    c.hisGrid = f.hisGrid.trim().toUpperCase() || null;
    c.progress = f.progress || null;
    c.contest = f.contest || null;
    c.averaging = f.averaging; c.deepSearch = f.deepSearch; c.emeDelay = f.emeDelay;
    c.myCall = f.myCall.trim().toUpperCase() || null;
    c.myGrid = f.myGrid.trim().toUpperCase() || null;
    onoptions(editing);
    dialog?.close();
  }

  function resetOptions() {
    f = blank();
    eError = '';
  }

  function channelState(i: number): string {
    if (active.length === 0) return '';
    if (active[i]) return `${slotCounts[i] ?? 0}`;
    // In a rotation a channel out of its step is waiting for it, not out of band.
    const sv = servers[channels[i].server ?? 0];
    return sv?.rotate && sv.rotation.length ? 'later' : 'paused';
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

<dialog bind:this={dialog} class="optdialog" onclose={() => (editing = null)}>
  {#if editing !== null}
    {@const c = channels[editing]}
    <form method="dialog" onsubmit={(e) => { e.preventDefault(); applyOptions(); }}>
      <h3>{c.mode} · {(c.dialHz / 1000).toFixed(1)} kHz</h3>

      <fieldset>
        <legend>Search</legend>
        <label title="Audio band searched (nfa, nfb). Empty: the mode's default">
          Band (Hz)
          <input bind:value={f.lo} placeholder="low" inputmode="decimal" />–<input bind:value={f.hi} placeholder="high" inputmode="decimal" />
        </label>
        <label title="nfqso: where the single-signal modes search and where QSO-partner AP is tried">
          Rx frequency (Hz) <input bind:value={f.rx} placeholder="none" inputmode="decimal" />
        </label>
        <label title="ntol: tolerance around the Rx frequency (JT9, Q65). Empty: 50 Hz">
          Tolerance (Hz) <input bind:value={f.tol} placeholder="50" inputmode="decimal" />
        </label>
        <label title="nftx: the Tx frequency">
          Tx frequency (Hz) <input bind:value={f.tx} placeholder="none" inputmode="decimal" />
        </label>
        <label title="WSJT-X decoding depth (ndepth)">
          Depth
          <select bind:value={f.depth}>
            <option value="">default (deep)</option>
            <option value="fast">fast</option>
            <option value="normal">normal</option>
            <option value="deep">deep</option>
          </select>
        </label>
      </fieldset>

      <fieldset>
        <legend>A-priori</legend>
        <label title="Enable AP (lft8apon) / CQ only (lapcqonly). Default: FT8 and JT65 off, the others full">
          AP
          <select bind:value={f.ap}>
            <option value="">mode default</option>
            <option value="off">off</option>
            <option value="cq">CQ only</option>
            <option value="full">full</option>
          </select>
        </label>
        <label title="Hunt one station: its call is given to the decoder as an a-priori hint (FT8, FT4, FST4, Q65)">
          DX call <input class="call" bind:value={f.dx} placeholder="JA1ABC" />
        </label>
        <label title="This channel's own mycall / mygrid, if it differs from Settings' (a second callsign, a portable locator). With a QSO below, FT8, FT4 and FST4 derive upstream's QSO-context AP from them. Empty uses Settings'.">
          My call <input class="call" bind:value={f.myCall} placeholder={stationCall || 'from Settings'} />
          grid <input class="grid" bind:value={f.myGrid} placeholder={stationGrid || 'from Settings'} />
        </label>
      </fieldset>

      <fieldset>
        <legend>QSO in progress</legend>
        <label title="hiscall / hisgrid. With 'My call' above, FT8, FT4 and FST4 derive upstream's QSO-context AP from these">
          His call <input class="call" bind:value={f.hisCall} placeholder="JA1ABC" />
          grid <input class="grid" bind:value={f.hisGrid} placeholder="PM95" />
        </label>
        <label title="nQSOProgress">
          Progress
          <select bind:value={f.progress}>
            <option value="">calling (CQ)</option>
            <option value="replying">replying</option>
            <option value="report">report</option>
            <option value="rogerreport">roger + report</option>
            <option value="rogers">RR73 / 73</option>
            <option value="signoff">sign-off</option>
          </select>
        </label>
        <label title="ncontest">
          Activity
          <select bind:value={f.contest}>
            <option value="">none</option>
            <option value="gridexchange">NA VHF / WW Digi / ARRL Digi / Q65 pileup</option>
            <option value="euvhf">EU VHF</option>
            <option value="fieldday">ARRL Field Day</option>
            <option value="rttyroundup">ARRL RTTY Roundup</option>
            <option value="fox">FT8 DXpedition (Fox)</option>
            <option value="hound">FT8 DXpedition (Hound)</option>
          </select>
        </label>
      </fieldset>

      <fieldset>
        <legend>Other</legend>
        <label class="check" title="ndepth & 16 (JT65: sums the same station over periods; Q65: averages)">
          <input type="checkbox" bind:checked={f.averaging} /> Average over periods
        </label>
        <label class="check" title="ndepth & 32 (JT65). Carried; the decoder does not read it yet">
          <input type="checkbox" bind:checked={f.deepSearch} /> Deep search (JT65)
        </label>
        <label class="check" title="emedelay: decode at 52 s">
          <input type="checkbox" bind:checked={f.emeDelay} /> EME delay
        </label>
      </fieldset>

      {#if eError}<p class="err">{eError}</p>{/if}
      <div class="buttons">
        <button type="button" class="link" onclick={resetOptions}>Reset</button>
        <button type="button" onclick={() => dialog?.close()}>Cancel</button>
        <button type="submit">Apply</button>
      </div>
      <p class="hint">Applies to the running skimmer at this channel's next slot. Each mode reads what WSJT-X's decoder for it reads.</p>
    </form>
  {/if}
</dialog>

<section>
  <h2>Channels</h2>
  {#each servers as sv, si (si)}
    {#if servers.length > 1}
      <h3 class="srvhead" class:sel={si === sel}>{sv.name}{sv.rotate && sv.rotation.length ? ` · rotating ${sv.rotation.map((r) => `${r.band} ${r.minutes}${r.hours.length === 24 && !r.hours.every(Boolean) ? ' (some hours)' : ''}`).join(' / ')}` : ''}</h3>
    {/if}
    <ul class="channels">
      {#each channels as c, i (`${c.server ?? 0}:${c.mode}${c.dialHz}`)}
        {#if (c.server ?? 0) === si}
      {@const s = slot(c.mode)}
      {@const idle = active.length > 0 && !active[i]}
      <li class:paused={idle}>
        <div class="line">
          <span class="mode">{c.mode}</span>
          <span class="freq">{(c.dialHz / 1000).toFixed(1)} kHz</span>
          {#if s}
            <span
              class="slotpie"
              class:idle
              style="--p: {idle ? 0 : (s.frac * 100).toFixed(1)}%"
              title={idle
                ? 'Not decoding now (outside the radio\'s band, or waiting for its turn in the rotation)'
                : `Slot ${s.start} UTC (${s.periodS} s slots): ${s.left} s to the next boundary, then this slot is decoded`}
            ></span>
          {/if}
          <span class="count" title="Decodes in the latest slot; paused: outside the radio's band; later: heard when its band's turn in the rotation comes">{channelState(i)}</span>
          <button
            class="link gear"
            class:set={optionSummary(c) !== ''}
            onclick={() => openOptions(i)}
            aria-label="Decode options"
            title={optionSummary(c) || 'Decode options: band, DX call, depth'}>⚙</button>
          <button class="link" onclick={() => remove(i)} aria-label="Remove">✕</button>
        </div>
      </li>
        {/if}
      {/each}
      {#if !channels.some((c) => (c.server ?? 0) === si)}
        <li class="empty">No channels yet</li>
      {/if}
    </ul>
  {/each}

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
  .gear.set {
    color: var(--accent, #4da3ff);
  }
  .optdialog {
    border: 1px solid var(--line, #444);
    border-radius: 8px;
    background: var(--panel, Canvas);
    color: inherit;
    padding: 1rem 1.25rem;
    min-width: 20rem;
  }
  .optdialog::backdrop {
    background: rgba(0, 0, 0, 0.45);
  }
  .optdialog h3 {
    margin: 0 0 0.75rem;
  }
  .optdialog label {
    display: flex;
    align-items: center;
    gap: 0.5rem;
    margin: 0.5rem 0;
  }
  .optdialog input {
    width: 6em;
  }
  .optdialog input.call {
    width: 8em;
  }
  .optdialog input.grid {
    width: 5em;
  }
  .optdialog fieldset {
    border: 1px solid var(--line, #444);
    border-radius: 6px;
    margin: 0.5rem 0;
    padding: 0.25rem 0.75rem 0.5rem;
  }
  .optdialog legend {
    font-size: 0.8em;
    color: var(--muted, #888);
  }
  .optdialog .check {
    gap: 0.4rem;
  }
  .optdialog .check input {
    width: auto;
  }
  .optdialog {
    max-height: 90vh;
    overflow: auto;
  }
  .optdialog .buttons {
    display: flex;
    justify-content: flex-end;
    gap: 0.5rem;
    margin-top: 1rem;
  }
  .optdialog .err {
    color: #e66;
    margin: 0.25rem 0;
  }
  .optdialog .hint {
    color: var(--muted, #888);
    font-size: 0.8em;
    margin: 0.75rem 0 0;
  }
</style>
