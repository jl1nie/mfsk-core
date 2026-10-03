<script lang="ts">
  import { MODE_CHIPS, PRESETS, blankForm, sortBands, toLocalInput, type PresetId, type QueryForm } from './analysis';

  let {
    form = $bindable(),
    bands,
    onapply,
    summary,
    error,
  }: {
    form: QueryForm;
    /** Bands there are decodes for. */
    bands: string[];
    onapply: () => void;
    /** "1 234 decodes · 210 stations" of the applied query. */
    summary: string;
    error: string;
  } = $props();

  let open = $state(true);

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
    <span class="sum" class:err={!!error}>{error || summary}</span>
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
      <label>SNR dB <span class="range"><input class="n" value={form.snrMin} onchange={(e) => { form.snrMin = e.currentTarget.value; onapply(); }} placeholder="min" /> – <input class="n" value={form.snrMax} onchange={(e) => { form.snrMax = e.currentTarget.value; onapply(); }} placeholder="max" /></span></label>
      <label title="Great-circle distance from your grid (Settings > Station)">
        Distance km <span class="range"><input class="n" value={form.kmMin} onchange={(e) => { form.kmMin = e.currentTarget.value; onapply(); }} placeholder="min" /> – <input class="n" value={form.kmMax} onchange={(e) => { form.kmMax = e.currentTarget.value; onapply(); }} placeholder="max" /></span>
      </label>
      <label title="Bearing from your grid, degrees clockwise from north. From greater than to wraps through north: 315 to 45 is the northern quarter.">
        Bearing ° <span class="range"><input class="n" value={form.bearingFrom} onchange={(e) => { form.bearingFrom = e.currentTarget.value; onapply(); }} placeholder="from" /> – <input class="n" value={form.bearingTo} onchange={(e) => { form.bearingTo = e.currentTarget.value; onapply(); }} placeholder="to" /></span>
      </label>
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
  .range {
    display: flex;
    gap: 4px;
    align-items: center;
  }
  .n {
    width: 62px;
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
