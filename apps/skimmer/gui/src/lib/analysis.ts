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
}

export const MODE_CHIPS = ['FT8', 'FT4', 'FST4*', 'Q65*', 'WSPR', 'JT9', 'JT65'];

export function blankForm(): QueryForm {
  return {
    preset: '24h', from: '', to: '', call: '', grid: '', text: '', bands: [], modes: [],
    snrMin: '', snrMax: '', kmMin: '', kmMax: '', bearingFrom: '', bearingTo: '', cq: 'any',
  };
}

const utc = (s: string) => {
  const t = Date.parse(`${s}:00Z`);
  return Number.isFinite(t) ? Math.floor(t / 1000) : null;
};
/** A number typed in a box; full-width digits and minus signs from a Japanese IME are read too. */
const num = (s: string | number | null) => {
  if (s === null || s === undefined) return null;
  const t = String(s)
    .replace(/[０-９．]/g, (c) => String.fromCharCode(c.charCodeAt(0) - 0xfee0))
    .replace(/[－ー−‐]/g, '-')
    .trim();
  const v = t === '' ? NaN : Number(t);
  return Number.isFinite(v) ? v : null;
};
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
  };
}

export const BAND_ORDER = ['2200m', '630m', '160m', '80m', '60m', '40m', '30m', '20m', '17m', '15m', '12m', '10m', '6m', '2m'];

export function sortBands(bands: Iterable<string>): string[] {
  return [...new Set(bands)].sort((a, b) => BAND_ORDER.indexOf(a) - BAND_ORDER.indexOf(b));
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
