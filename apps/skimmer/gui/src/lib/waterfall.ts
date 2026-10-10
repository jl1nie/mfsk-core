// The waterfall rows the backend sends, kept for drawing.
//
// Each row is 0-255 dB above that row's own noise floor (see skimmer-core's
// `waterfall`). The channel in focus sends whole rows (2.9 or 1.5 Hz per bin);
// the others send thumbnails (four bins to one, every other row).

export interface Strip {
  /** Audio frequency of bin 0, and Hz per bin. */
  fLo: number;
  binHz: number;
  /** Newest first. */
  rows: Uint8Array[];
  utc: number[];
}

/** Rows older than this are dropped: the longest span any mode shows. */
const KEEP_MS = 135_000;

// The height of the large waterfall and of a thumbnail is a setting
// (`Settings::wf_height`, `wf_thumb_height`), chosen in the panel's header.

/**
 * The time one screen of a channel's waterfall covers: whole slots, at least
 * 20 s, at most 130 s. FT8 shows two slots (30 s), FT4 three (22.5 s), a 60 s
 * mode two (120 s), WSPR one (120 s): the scroll speed follows the protocol.
 */
export function spanMs(slotSeconds: number): number {
  const t = slotSeconds > 0 ? slotSeconds : 15;
  const slots = Math.max(1, Math.ceil(20 / t));
  return Math.min(130, t * Math.max(slots, t >= 60 ? 1 : 2)) * 1000;
}

/** A thumbnail bin: four of the coarse waterfall's 2.93 Hz bins. */
const THUMB_BIN_HZ = 11.72;

const empty = (): Strip => ({ fLo: 0, binHz: 1, rows: [], utc: [] });

export class WaterfallStore {
  thumbs = new Map<number, Strip>();
  big: Strip & { channel: number } = { ...empty(), channel: -1 };
  /** Rows of the focused channel seen, to thin its thumbnail as the backend thins the others'. */
  private focusRows = 0;

  push(e: { channel: number; focus: boolean; utcMs: number; fLoHz: number; binHz: number; levels: number[] }) {
    const row = Uint8Array.from(e.levels);
    if (e.focus) {
      const b = this.big;
      if (b.channel !== e.channel || b.binHz !== e.binHz || b.rows[0]?.length !== row.length) {
        b.channel = e.channel;
        b.rows = [];
        b.utc = [];
      }
      b.fLo = e.fLoHz;
      b.binHz = e.binHz;
      add(b, row, e.utcMs);
      // The backend sends the focused channel whole, to the large view only: its
      // thumbnail would stand still. Make its thumbnail row here, as the backend does
      // for the others: the strongest of four coarse bins, every second row.
      if (++this.focusRows % 2 === 0) {
        const by = Math.max(1, Math.round(THUMB_BIN_HZ / e.binHz));
        const pooled = new Uint8Array(Math.ceil(row.length / by));
        for (let i = 0; i < row.length; i++) pooled[(i / by) | 0] = Math.max(pooled[(i / by) | 0], row[i]);
        let t = this.thumbs.get(e.channel);
        if (!t) {
          t = empty();
          this.thumbs.set(e.channel, t);
        }
        // Rows come oldest first after a choice of channel (the backend replays its past):
        // those the thumbnail has already, or older, are not put in front of newer ones.
        if (t.utc.length === 0 || e.utcMs > t.utc[0]) {
          t.fLo = e.fLoHz;
          t.binHz = e.binHz * by;
          add(t, pooled, e.utcMs);
        }
      }
    } else {
      let s = this.thumbs.get(e.channel);
      if (!s) {
        s = empty();
        this.thumbs.set(e.channel, s);
      }
      s.fLo = e.fLoHz;
      s.binHz = e.binHz;
      add(s, row, e.utcMs);
    }
  }

  clear() {
    this.thumbs.clear();
    this.big = { ...empty(), channel: -1 };
  }
}

function add(s: Strip, row: Uint8Array, utc: number) {
  s.rows.unshift(row);
  s.utc.unshift(utc);
  let n = s.utc.length;
  while (n > 1 && s.utc[n - 1] < utc - KEEP_MS) n--;
  s.rows.length = n;
  s.utc.length = n;
}

/** Black, blue, cyan, yellow, red: a level of 0-255 as RGBA (0 is black). */
export const PALETTE: Uint8ClampedArray = (() => {
  const stops: [number, [number, number, number]][] = [
    [0, [0, 0, 0]],
    [0.25, [0, 30, 170]],
    [0.5, [0, 200, 220]],
    [0.75, [240, 230, 60]],
    [1, [255, 60, 20]],
  ];
  const p = new Uint8ClampedArray(256 * 4);
  for (let v = 0; v < 256; v++) {
    const t = v / 255;
    let k = 1;
    while (k < stops.length - 1 && t > stops[k][0]) k++;
    const [t0, c0] = stops[k - 1];
    const [t1, c1] = stops[k];
    const u = (t - t0) / (t1 - t0);
    for (let c = 0; c < 3; c++) p[v * 4 + c] = c0[c] + (c1[c] - c0[c]) * u;
    p[v * 4 + 3] = 255;
  }
  return p;
})();

/**
 * Paint a strip into `img` over the last `spanMs`, newest on top: pixel row `y`
 * is the strongest of the rows in its slice of time (a weak signal must not
 * vanish when several rows share a pixel), or the row above it when the slice
 * holds none.
 */
export function paint(img: ImageData, s: Strip, spanMs: number) {
  const w = img.width;
  const h = img.height;
  const d = img.data;
  // Opaque black, also where there are no rows yet.
  for (let o = 3; o < d.length; o += 4) d[o] = 255;
  if (s.rows.length === 0) return;
  const newest = s.utc[0];
  const per = spanMs / h;
  const oldest = s.utc[s.utc.length - 1];
  let r = 0;
  let acc: Uint8Array | null = null;
  for (let y = 0; y < h; y++) {
    const t0 = newest - (y + 1) * per;
    // Older than the oldest row: nothing to show, and nothing to stretch.
    if (t0 + per < oldest) break;
    let got: Uint8Array | null = null;
    while (r < s.rows.length && s.utc[r] > t0) {
      const row = s.rows[r];
      if (got === null) got = row.slice();
      else for (let x = 0; x < got.length && x < row.length; x++) if (row[x] > got[x]) got[x] = row[x];
      r++;
    }
    if (got !== null) acc = got;
    if (acc === null) continue;
    const n = Math.min(acc.length, w);
    let o = y * w * 4;
    for (let x = 0; x < n; x++, o += 4) {
      const v = acc[x] * 4;
      d[o] = PALETTE[v];
      d[o + 1] = PALETTE[v + 1];
      d[o + 2] = PALETTE[v + 2];
      d[o + 3] = 255;
    }
  }
}
