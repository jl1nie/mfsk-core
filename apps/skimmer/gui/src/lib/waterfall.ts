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

const THUMB_ROWS = 60;
export const BIG_ROWS = 110;

const empty = (): Strip => ({ fLo: 0, binHz: 1, rows: [], utc: [] });

export class WaterfallStore {
  thumbs = new Map<number, Strip>();
  big: Strip & { channel: number } = { ...empty(), channel: -1 };

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
      add(b, row, e.utcMs, BIG_ROWS);
    } else {
      let s = this.thumbs.get(e.channel);
      if (!s) {
        s = empty();
        this.thumbs.set(e.channel, s);
      }
      s.fLo = e.fLoHz;
      s.binHz = e.binHz;
      add(s, row, e.utcMs, THUMB_ROWS);
    }
  }

  clear() {
    this.thumbs.clear();
    this.big = { ...empty(), channel: -1 };
  }
}

function add(s: Strip, row: Uint8Array, utc: number, max: number) {
  s.rows.unshift(row);
  s.utc.unshift(utc);
  if (s.rows.length > max) {
    s.rows.length = max;
    s.utc.length = max;
  }
}

/** Black, blue, cyan, yellow, red: a level of 0-255 as RGBA. */
export const PALETTE: Uint8ClampedArray = (() => {
  const stops: [number, [number, number, number]][] = [
    [0, [0, 0, 24]],
    [0.25, [0, 40, 200]],
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

/** Paint `rows` (newest on top) into `img`, each row `rowPx` tall. */
export function paint(img: ImageData, rows: Uint8Array[], rowPx: number) {
  const w = img.width;
  const d = img.data;
  d.fill(0);
  for (let r = 0; r < rows.length; r++) {
    const row = rows[r];
    const n = Math.min(row.length, w);
    for (let py = 0; py < rowPx; py++) {
      const y = r * rowPx + py;
      if (y >= img.height) return;
      let o = y * w * 4;
      for (let x = 0; x < n; x++, o += 4) {
        const v = row[x] * 4;
        d[o] = PALETTE[v];
        d[o + 1] = PALETTE[v + 1];
        d[o + 2] = PALETTE[v + 2];
        d[o + 3] = 255;
      }
    }
  }
}
