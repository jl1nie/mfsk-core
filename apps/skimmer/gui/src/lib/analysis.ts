/** Shared by the Analysis views: periods, bands, colours. */

export const PERIODS = [
  { id: '6h', label: '6 h', s: 6 * 3600 },
  { id: '24h', label: '24 h', s: 24 * 3600 },
  { id: '7d', label: '7 days', s: 7 * 86400 },
  { id: '30d', label: '30 days', s: 30 * 86400 },
  { id: 'all', label: 'All', s: 0 },
] as const;
export type PeriodId = (typeof PERIODS)[number]['id'];

/** `[since, until]` in UTC seconds for a period ending now. */
export function range(id: PeriodId, nowMs: number): [number, number] {
  const until = Math.floor(nowMs / 1000);
  const p = PERIODS.find((x) => x.id === id)!;
  return [p.s === 0 ? 0 : until - p.s, until];
}

export const BAND_ORDER = ['2200m', '630m', '160m', '80m', '60m', '40m', '30m', '20m', '17m', '15m', '12m', '10m', '6m', '2m'];

export function sortBands(bands: Iterable<string>): string[] {
  return [...new Set(bands)].sort((a, b) => BAND_ORDER.indexOf(a) - BAND_ORDER.indexOf(b));
}

/** Blue (weak) to red (strong) over `-24..+10` dB. */
export function snrColour(snr: number): string {
  const t = Math.max(0, Math.min(1, (snr + 24) / 34));
  const hue = 240 - 240 * t;
  return `hsl(${hue} 80% 50%)`;
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
