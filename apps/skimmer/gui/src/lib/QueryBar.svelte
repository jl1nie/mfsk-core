<script lang="ts">
  import RangeSlider from './RangeSlider.svelte';
  import BearingPicker from './BearingPicker.svelte';
  import { MODE_CHIPS, PRESETS, badNumber, num, blankForm, formErrors, sortBands, toLocalInput, type PresetId, type QueryForm } from './analysis';

  let {
    form = $bindable(),
    bands,
    onapply,
    summary,
    error,
    snrHist,
  }: {
    form: QueryForm;
    /** Bands there are decodes for. */
    bands: string[];
    onapply: () => void;
    /** "1 234 decodes · 210 stations" of the applied query. */
    summary: string;
    error: string;
    /** Decodes per dB for the query without its SNR limits. */
    snrHist: [number, number][];
  } = $props();

  let open = $state(true);
  const problems = $derived(formErrors(form));

  type Preset = [string, number | null, number | null];
  const SNR_PRESETS: Preset[] = [['weak ≤ −15', null, -15], ['mid', -15, 0], ['strong ≥ 0', 0, null]];
  const KM_PRESETS: Preset[] = [['local < 500', null, 500], ['regional 500–3000', 500, 3000], ['DX > 5000', 5000, null], ['long path > 15000', 15000, null]];

  function setRange(kLo: 'snrMin' | 'kmMin' | 'bearingFrom', kHi: 'snrMax' | 'kmMax' | 'bearingTo', lo: number | null, hi: number | null) {
    form[kLo] = lo === null ? '' : String(lo);
    form[kHi] = hi === null ? '' : String(hi);
    onapply();
  }
  const same = (a: string, b: string, lo: number | null, hi: number | null) => num(a) === lo && num(b) === hi;

  function toggle(list: string[], v: string): string[] {
    return list.includes(v) ? list.filter((x) => x !== v) : [...list, v];
  }

  function preset() {
    if (form.preset === 'custom' && !form.from) {
      const now = Date.now() / 1000;
      form.from = toLocalInput(now - 86400);
      form.to = toLocalInput(now);
    }
    onapply();
  }

  function reset() {
    form = blankForm();
    onapply();
  }

  /** One line for the collapsed bar. */
  const brief = $derived(
    [
      PRESETS.find((p) => p.id === form.preset)?.label,
      form.bands.length ? form.bands.join(' ') : '',
      form.modes.length ? form.modes.join(' ') : '',
      form.call ? `call ${form.call}` : '',
      form.grid ? `grid ${form.grid}` : '',
      form.cq !== 'any' ? `CQ ${form.cq === '*' ? '' : form.cq === '' ? '(plain)' : form.cq}` : '',
    ]
      .filter(Boolean)
      .join(' · '),
  );
</script>

<form
  class="q"
  onsubmit={(e) => {
    e.preventDefault();
    onapply();
  }}
>
  <div class="head">
    <button type="button" class="link" onclick={() => (open = !open)}>{open ? '▾' : '▸'} Query</button>
    {#if !open}<span class="brief">{brief}</span>{/if}
    <span class="sum" class:err={!!error || problems.length > 0}>{problems[0] ?? (error || summary)}</span>
  </div>
  {#if open}
    <div class="grid">
      <label>
        Period (UTC)
        <!-- Set the value in the handler: with bind:value, onchange can run first and search with the old one. -->
        <select value={form.preset} onchange={(e) => { form.preset = e.currentTarget.value as PresetId; preset(); }}>
          {#each PRESETS as p (p.id)}<option value={p.id}>{p.label}</option>{/each}
        </select>
      </label>
      {#if form.preset === 'custom'}
        <label>From <input type="datetime-local" value={form.from} onchange={(e) => { form.from = e.currentTarget.value; onapply(); }} /></label>
        <label>To <input type="datetime-local" value={form.to} onchange={(e) => { form.to = e.currentTarget.value; onapply(); }} /></label>
      {/if}
      <label title="Regular expression, case-insensitive; ^JA matches calls starting JA, ^(JA|JH)\d, W[0-9]X">
        Call (regex) <input value={form.call} onchange={(e) => { form.call = e.currentTarget.value; onapply(); }} placeholder="^JA" spellcheck="false" />
      </label>
      <label title="Regular expression on the locator: ^PM95, ^(PM|QM), ^[A-R][A-R]0">
        Grid (regex) <input value={form.grid} onchange={(e) => { form.grid = e.currentTarget.value; onapply(); }} placeholder="^PM" spellcheck="false" />
      </label>
      <label title="Regular expression on the message text">
        Text (regex) <input value={form.text} onchange={(e) => { form.text = e.currentTarget.value; onapply(); }} placeholder="POTA" spellcheck="false" />
      </label>
      <label>
        CQ
        <select value={form.cq} onchange={(e) => { form.cq = e.currentTarget.value; onapply(); }}>
          <option value="any">any message</option>
          <option value="*">every CQ</option>
          <option value="">plain CQ</option>
          <option value="DX">CQ DX</option>
          <option value="POTA">CQ POTA</option>
          <option value="SOTA">CQ SOTA</option>
          <option value="TEST">CQ TEST</option>
        </select>
      </label>
    </div>
    <div class="filters">
      <div class="card">
        <div class="t">SNR</div>
        <RangeSlider lo={num(form.snrMin)} hi={num(form.snrMax)} min={-30} max={20} unit=" dB" bars={snrHist}
          onchange={(l, h) => setRange('snrMin', 'snrMax', l, h)} />
        <div class="pre">
          {#each SNR_PRESETS as [name, l, h] (name)}
            <button type="button" class="chip" class:on={same(form.snrMin, form.snrMax, l, h)} onclick={() => setRange('snrMin', 'snrMax', l, h)}>{name}</button>
          {/each}
        </div>
        <div class="num">
          <input class:bad={badNumber(form.snrMin)} value={form.snrMin} placeholder="min" onchange={(e) => { form.snrMin = e.currentTarget.value; onapply(); }} />
          –
          <input class:bad={badNumber(form.snrMax)} value={form.snrMax} placeholder="max" onchange={(e) => { form.snrMax = e.currentTarget.value; onapply(); }} />
        </div>
      </div>
      <div class="card" title="Great-circle distance from your grid (Settings > Station)">
        <div class="t">Distance</div>
        <RangeSlider lo={num(form.kmMin)} hi={num(form.kmMax)} min={0} max={20000} log unit=" km"
          onchange={(l, h) => setRange('kmMin', 'kmMax', l, h)} />
        <div class="pre">
          {#each KM_PRESETS as [name, l, h] (name)}
            <button type="button" class="chip" class:on={same(form.kmMin, form.kmMax, l, h)} onclick={() => setRange('kmMin', 'kmMax', l, h)}>{name}</button>
          {/each}
        </div>
        <div class="num">
          <input class:bad={badNumber(form.kmMin)} value={form.kmMin} placeholder="min" onchange={(e) => { form.kmMin = e.currentTarget.value; onapply(); }} />
          –
          <input class:bad={badNumber(form.kmMax)} value={form.kmMax} placeholder="max" onchange={(e) => { form.kmMax = e.currentTarget.value; onapply(); }} />
        </div>
      </div>
      <div class="card" title="Bearing from your grid, clockwise from north. Click a direction; click a neighbour to widen, an end wedge to narrow. From greater than to wraps through north.">
        <div class="t">Bearing</div>
        <BearingPicker from={num(form.bearingFrom)} to={num(form.bearingTo)}
          onchange={(f, t) => setRange('bearingFrom', 'bearingTo', f, t)} />
        <div class="num">
          <input class:bad={badNumber(form.bearingFrom)} value={form.bearingFrom} placeholder="from" onchange={(e) => { form.bearingFrom = e.currentTarget.value; onapply(); }} />
          –
          <input class:bad={badNumber(form.bearingTo)} value={form.bearingTo} placeholder="to" onchange={(e) => { form.bearingTo = e.currentTarget.value; onapply(); }} />
        </div>
      </div>
    </div>
    <div class="chips">
      <span class="lab">Band</span>
      {#each sortBands(bands) as b (b)}
        <button type="button" class="chip" class:on={form.bands.includes(b)} onclick={() => { form.bands = toggle(form.bands, b); onapply(); }}>{b}</button>
      {/each}
      <span class="lab">Mode</span>
      {#each MODE_CHIPS as m (m)}
        <button type="button" class="chip" class:on={form.modes.includes(m)} onclick={() => { form.modes = toggle(form.modes, m); onapply(); }}>{m.replace('*', '')}</button>
      {/each}
      <span class="actions">
        <button type="submit" class="primary">Search</button>
        <button type="button" onclick={reset}>Reset</button>
      </span>
    </div>
  {/if}
</form>

<style>
  .q {
    border-bottom: 1px solid var(--line);
    padding-bottom: 8px;
    margin-bottom: 8px;
  }
  .head {
    display: flex;
    gap: 12px;
    align-items: baseline;
    font-size: 12px;
  }
  .brief,
  .sum {
    color: var(--muted);
  }
  .sum {
    margin-left: auto;
  }
  .sum.err {
    color: #d9822b;
  }
  .grid {
    display: flex;
    flex-wrap: wrap;
    gap: 8px 14px;
    padding: 8px 0 6px;
  }
  label {
    display: flex;
    flex-direction: column;
    gap: 2px;
    font-size: 11.5px;
    color: var(--muted);
  }
  label input:not(.n),
  label select {
    width: 150px;
  }
  .filters {
    display: flex;
    flex-wrap: wrap;
    gap: 10px 18px;
    padding: 4px 0 8px;
  }
  .card {
    display: flex;
    flex-direction: column;
    gap: 4px;
    min-width: 200px;
  }
  .card .t {
    font-size: 11.5px;
    color: var(--muted);
  }
  .pre {
    display: flex;
    flex-wrap: wrap;
    gap: 4px;
    max-width: 230px;
  }
  .num {
    display: flex;
    gap: 4px;
    align-items: center;
  }
  .num input {
    width: 70px;
  }
  .chips {
    display: flex;
    flex-wrap: wrap;
    gap: 5px;
    align-items: center;
  }
  .lab {
    font-size: 11.5px;
    color: var(--muted);
    margin: 0 4px 0 8px;
  }
  .chip {
    padding: 2px 8px;
    font-size: 12px;
    border-radius: 12px;
  }
  .chip.on {
    background: var(--accent);
    color: var(--accent-text);
  }
  .actions {
    margin-left: auto;
    display: flex;
    gap: 6px;
  }
</style>
