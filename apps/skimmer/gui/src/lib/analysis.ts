/** Shared by the Analysis views: the query form, bands, colours. */
import type { Query } from './types';

export const PRESETS = [
  { id: '6h', label: 'Last 6 h', s: 6 * 3600 },
  { id: '24h', label: 'Last 24 h', s: 24 * 3600 },
  { id: '7d', label: 'Last 7 days', s: 7 * 86400 },
  { id: '30d', label: 'Last 30 days', s: 30 * 86400 },
  { id: 'all', label: 'Everything', s: 0 },
  { id: 'custom', label: 'From – to', s: 0 },
] as const;
export type PresetId = (typeof PRESETS)[number]['id'];

/** The query form as typed: text boxes stay text until applied. */
export interface QueryForm {
  preset: PresetId;
  /** `YYYY-MM-DDTHH:MM`, UTC (for 'custom'). */
  from: string;
  to: string;
  call: string;
  grid: string;
  text: string;
  bands: string[];
  modes: string[];
  snrMin: string;
  snrMax: string;
  kmMin: string;
  kmMax: string;
  bearingFrom: string;
  bearingTo: string;
  cq: string; // 'any' | '*' | '' | 'DX' ...
  servers: string[];
}

export const MODE_CHIPS = ['FT8', 'FT4', 'FST4*', 'Q65*', 'WSPR', 'JT9', 'JT65'];

export function blankForm(): QueryForm {
  return {
    preset: '24h', from: '', to: '', call: '', grid: '', text: '', bands: [], modes: [],
    snrMin: '', snrMax: '', kmMin: '', kmMax: '', bearingFrom: '', bearingTo: '', cq: 'any', servers: [],
  };
}

const utc = (s: string) => {
  const t = Date.parse(`${s}:00Z`);
  return Number.isFinite(t) ? Math.floor(t / 1000) : null;
};
/** A number typed in a box; full-width digits and minus signs from a Japanese IME are read too. */
export const num = (s: string | number | null) => {
  if (s === null || s === undefined) return null;
  const t = String(s)
    .replace(/[０-９．]/g, (c) => String.fromCharCode(c.charCodeAt(0) - 0xfee0))
    .replace(/[－ー−‐]/g, '-')
    .trim();
  const v = t === '' ? NaN : Number(t);
  return Number.isFinite(v) ? v : null;
};
/** A number box is fine if empty or a number; anything else would be silently ignored. */
export const badNumber = (s: string) => s.trim() !== '' && num(s) === null;

const NUMBERS: [keyof QueryForm, string][] = [
  ['snrMin', 'SNR min'], ['snrMax', 'SNR max'], ['kmMin', 'Distance min'], ['kmMax', 'Distance max'],
  ['bearingFrom', 'Bearing from'], ['bearingTo', 'Bearing to'],
];

/** What is wrong with the form, in words; empty if it can be searched. */
export function formErrors(f: QueryForm): string[] {
  const out: string[] = [];
  for (const [k, label] of NUMBERS) {
    const v = f[k] as string;
    if (badNumber(v)) out.push(`${label}: "${v}" is not a number`);
  }
  if ((f.bearingFrom.trim() === '') !== (f.bearingTo.trim() === '')) out.push('Bearing needs both from and to');
  for (const [k, label] of [['call', 'Call'], ['grid', 'Grid'], ['text', 'Text']] as const) {
    try {
      new RegExp(f[k]);
    } catch {
      out.push(`${label}: not a valid regular expression`);
    }
  }
  return out;
}

export const toLocalInput = (utcS: number) => new Date(utcS * 1000).toISOString().slice(0, 16);

/** The form as a query ending now (presets move with the clock). */
export function toQuery(f: QueryForm, me: string, nowMs: number): Query {
  const now = Math.floor(nowMs / 1000);
  let since = 0;
  let until = now;
  const p = PRESETS.find((x) => x.id === f.preset)!;
  if (f.preset === 'custom') {
    since = utc(f.from) ?? 0;
    until = utc(f.to) ?? now;
  } else if (p.s > 0) {
    since = now - p.s;
  }
  const bf = num(f.bearingFrom);
  const bt = num(f.bearingTo);
  return {
    since, until, me,
    call: f.call.trim(), grid: f.grid.trim(), text: f.text.trim(),
    bands: f.bands, modes: f.modes,
    snrMin: num(f.snrMin), snrMax: num(f.snrMax), kmMin: num(f.kmMin), kmMax: num(f.kmMax),
    bearingFrom: bf !== null && bt !== null ? bf : null,
    bearingTo: bf !== null && bt !== null ? bt : null,
    cq: f.cq === 'any' ? null : f.cq,
    servers: f.servers,
  };
}

/** The amateur band a dial frequency falls in; as skimmer_core::store::band_of. */
export function bandOfHz(hz: number): string {
  const B: [string, number, number][] = [
    ['2200m', 135.7e3, 137.8e3], ['630m', 472e3, 479e3], ['160m', 1.8e6, 2.0e6], ['80m', 3.5e6, 4.0e6],
    ['60m', 5.25e6, 5.45e6], ['40m', 7.0e6, 7.3e6], ['30m', 10.1e6, 10.15e6], ['20m', 14.0e6, 14.35e6],
    ['17m', 18.068e6, 18.168e6], ['15m', 21.0e6, 21.45e6], ['12m', 24.89e6, 24.99e6], ['10m', 28.0e6, 29.7e6],
    ['6m', 50.0e6, 54.0e6], ['2m', 144.0e6, 148.0e6],
  ];
  return B.find(([, lo, hi]) => hz >= lo && hz <= hi)?.[0] ?? 'other';
}

export const BAND_ORDER = ['2200m', '630m', '160m', '80m', '60m', '40m', '30m', '20m', '17m', '15m', '12m', '10m', '6m', '2m'];

export function sortBands(bands: Iterable<string>): string[] {
  return [...new Set(bands)].sort((a, b) => BAND_ORDER.indexOf(a) - BAND_ORDER.indexOf(b));
}

/** A colour for a band, apart from its neighbours' (the paths of several bands on one map). */
export function bandColour(band: string, alpha = 1): string {
  const i = BAND_ORDER.indexOf(band);
  if (i < 0) return `hsl(0 0% 55% / ${alpha})`;
  // Golden-angle steps in hue keep adjacent bands (20 m, 17 m, 15 m) clearly different.
  return `hsl(${(i * 137.5 + 20) % 360} 72% 48% / ${alpha})`;
}

/** Blue (weak) to red (strong) over `-24..+10` dB. */
export function snrColour(snr: number, alpha = 1): string {
  const t = Math.max(0, Math.min(1, (snr + 24) / 34));
  return `hsl(${240 - 240 * t} 80% 50% / ${alpha})`;
}

/** 0..1 to a light-to-dark ramp for counts. */
export function heat(t: number, dark: boolean): string {
  const x = Math.max(0, Math.min(1, t));
  if (x === 0) return dark ? '#20262d' : '#eef0f3';
  return `hsl(${215 - 190 * x} ${55 + 35 * x}% ${dark ? 25 + 35 * x : 88 - 45 * x}%)`;
}

export const isDark = () => window.matchMedia('(prefers-color-scheme: dark)').matches;

export const hhmm = (utcS: number) => new Date(utcS * 1000).toISOString().slice(11, 16);
export const ymd = (utcS: number) => new Date(utcS * 1000).toISOString().slice(0, 10);
export const stamp = (utcS: number) => `${ymd(utcS)} ${hhmm(utcS)}`;

/** Centre of a 4- or 6-character locator: [lon, lat], or null. */
export function gridLonLat(g: string): [number, number] | null {
  const m = /^([A-R]{2})(\d{2})([A-X]{2})?$/i.exec(g.trim());
  if (!m) return null;
  const a = m[1].toUpperCase();
  let lon = -180 + 20 * (a.charCodeAt(0) - 65) + 2 * Number(m[2][0]);
  let lat = -90 + 10 * (a.charCodeAt(1) - 65) + Number(m[2][1]);
  if (m[3]) {
    const s = m[3].toUpperCase();
    lon += ((s.charCodeAt(0) - 65) * 5) / 60 + 2.5 / 60;
    lat += ((s.charCodeAt(1) - 65) * 2.5) / 60 + 1.25 / 60;
  } else {
    lon += 1;
    lat += 0.5;
  }
  return [lon, lat];
}
