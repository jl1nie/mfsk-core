// Mirrors of the Rust side (src-tauri/src/main.rs). Field names are camelCase
// there by `serde(rename_all…)`, so these are the JSON shapes as received.

export type WireFormat = 'float' | 'int16';

export type DepthSetting = '' | 'fast' | 'normal' | 'deep';
export type ApSetting = '' | 'off' | 'cq' | 'full';

/** A channel and its share of WSJT-X's decode parameter block; unset keeps the mode's default. */
export interface ChannelSetting {
  mode: string;
  dialHz: number;
  /** Audio band searched, Hz; both or neither. */
  bandLo?: number | null;
  bandHi?: number | null;
  /** Rx frequency, tolerance and Tx frequency, Hz. */
  rxFreqHz?: number | null;
  tolHz?: number | null;
  txFreqHz?: number | null;
  /** Empty is the library default (deep). */
  depth?: DepthSetting | null;
  /** Empty is the mode's GUI default (FT8 and JT65 off, the others full). */
  ap?: ApSetting | null;
  /** A station to hunt (a-priori hint). */
  dxCall?: string | null;
  /** This channel's own call and locator; empty uses its server's. */
  myCall?: string | null;
  myGrid?: string | null;
  /** The QSO in progress; nQSOProgress as a name or 0-5. */
  hisCall?: string | null;
  hisGrid?: string | null;
  progress?: string | null;
  /** An ncontest activity by name; empty is none. */
  contest?: string | null;
  averaging?: boolean;
  deepSearch?: boolean;
  emeDelay?: boolean;
  /** Which server (an index into `Settings.servers`) listens to it. */
  server?: number;
}

/** One SpyServer. */
export interface ServerSetting {
  /** Shown in the window and kept in the database; unique. */
  name: string;
  address: string;
  /** Where its antenna is (a locator): the origin of the bearings of what it hears. */
  grid: string;
  /** Your callsign at this server (QSO-context AP); a channel can override it. */
  call: string;
  /** IQ sample format on the wire; int16 halves the bytes. */
  format: WireFormat;
  /** How the band is cut into channels. */
  channelizer: 'auto' | 'direct' | 'pfb';
  /** Fixed delay between the SDR and this PC, taken off arrival times, ms. */
  networkDelayMs: number;
  /** Connected when Connect is pressed; off keeps its settings and channels but listens to nothing. */
  enabled: boolean;
  /** Hold control and tune the radio even beside an operator's client. */
  tune: boolean;
  /** Leave control to an SDR# started later, instead of holding it. */
  yieldControl: boolean;
  /** Rotate through the bands of this server's channels: one step per band, in this order. */
  rotate: boolean;
  rotation: RotationStep[];
  /** Send what this server hears to PSK Reporter as spots, under `call` at `grid`. Off by default. */
  pskReporter?: boolean;
  /** Free text for the receiver record. */
  pskAntenna?: string;
}

/** One band of a rotation: the channels of the band (its modes together) are heard for `minutes` per turn. */
export interface RotationStep {
  band: string;
  minutes: number;
  /** The UTC hours this band takes part in: 24 flags, hour 0 first; empty is all day. */
  hours: boolean[];
  /** "day" or "night": follows the Sun at the server day by day; empty: the fixed `hours`. */
  follow?: '' | 'day' | 'night';
  /** Hours of grey line either side of sunrise and sunset when following. */
  marginHours?: number;
}

export interface Settings {
  servers: ServerSetting[];
  /** The channels of every server, in one list. */
  channels: ChannelSetting[];
  /** Draw the channels' waterfalls; fine is 1.5 Hz per bin instead of 2.9. */
  waterfall: boolean;
  waterfallFine: boolean;
  /** Height of the large waterfall and of a thumbnail, in pixels. */
  wfHeight: number;
  wfThumbHeight: number;
  /** The PC clock as it is, or corrected against an NTP server. */
  clockSource: 'system' | 'ntp';
  ntpServer: string;
  /** The rotation counts from UTC midnight instead of starting at the first band on Connect. */
  rotationUtc: boolean;
  /** Percent of a slot's period its decode may run before it stops; 0 is no limit. */
  slotBudgetPct: number;
  /** Decoder threads per channel (1-8): the next slot is decoded on another thread while the last is still being decoded. */
  decodeLanes: number;
  /** FT8 rows from ~11.8 s into the slot, the rest at its end, as WSJT-X shows them. */
  earlyDecode: boolean;
  /** Every decode in a SQLite file, for the Analysis view. */
  dbEnabled: boolean;
  /** The database file recorded into (and read by Analysis unless another is opened). */
  dbPath: string;
  /** Folder the health log STATUS.log is written in. */
  logDir: string;
}

export interface ModeInfo {
  name: string;
  /** Slot (T/R period); slots start on multiples of it from 00:00 UTC. */
  slotS: number;
  /** Seconds from the slot start to the first symbol at dt = 0. */
  offsetS: number;
  /** Length of a frame, s. */
  frameS: number;
  /** Width of a frame on the band, Hz. */
  widthHz: number;
}

/** One JTTY message: it grows while it is received, so the row of its `key` on its channel is replaced in place. */
export interface JttyRow {
  channel: number;
  /** Stable for the life of the message; a string, since it is 64 bits. */
  key: string;
  /** UTC of the start of its first frame, ms since the epoch. */
  startUtcMs: number | null;
  /** Where its latest frame ends: how far it has got. */
  endUtcMs: number | null;
  dialHz: number;
  /** RF frequency of the lowest tone of its latest frame. */
  freqHz: number;
  /** SNR in 2 500 Hz of its first frame. */
  snrDb: number;
  text: string;
  /** The callsigns of its call atoms, in order, each once. */
  calls: string[];
  kind: 'growing' | 'complete' | 'expired' | 'ended';
  final: boolean;
}

export interface DecodeRow {
  /** Unique and increasing, assigned on arrival: the table's key. */
  id: number;
  channel: number;
  mode: string;
  /** UTC of the slot start, ms since the epoch. */
  slotUtcMs: number | null;
  dialHz: number;
  freqHz: number;
  snrDb: number;
  dtS: number;
  text: string;
  /** The message's bits, hex: one message heard on two channels or servers has one key. */
  key: string;
  /** On the scale of the mode's own search; null where the mode reports none (WSPR, JT9, JT65, Q65, FT8's a7/a8). */
  syncScore: number | null;
  /** Hard-decision errors the FEC corrected; null where the mode reports no count. */
  hardErrors: number | null;
  /** The text needed the callsign table: a `<...>` was resolved. */
  hashResolved: boolean;
  copiedLastTx: boolean;
  /** Found at FT8's early checkpoint (~11.8 s), before the slot ended. */
  early: boolean;
  /** Replaces the row of this channel and slot with the same key and frequency (its `<...>` now reads resolved). */
  update: boolean;
}

export interface Status {
  streamedS: number;
  delayMs: number;
  driftMs: number;
  longestPushMs: number;
  queuedBytes: number;
  queuedSlots: number;
  droppedSlots: number;
  /** Slots the slot budget stopped, since connecting. */
  budgetCutSlots: number;
  longestDecodeMs: number;
  gaps: number;
  reanchors: number;
  /** The NTP line; empty on the PC clock. */
  clock: string;
}

/** What a server's PSK Reporter sender has done. */
export interface PskStatus {
  offered: number;
  duplicates: number;
  overflowed: number;
  /** Not spots: they began before the radio was retuned. */
  stale: number;
  spotsSent: number;
  datagramsSent: number;
  pending: number;
  /** ms since the epoch of the last datagram, or null. */
  lastSendMs: number | null;
  error: string | null;
  /** Where the datagrams go, `host:port`. */
  endpoint: string;
}

export type UiEventBody =
  | { type: 'connecting'; address: string }
  | ({ type: 'psk' } & PskStatus)
  | {
      type: 'connected';
      deviceKind: number;
      maxRate: number;
      bandwidthHz: number;
      maxGain: number;
      gain: number;
      control: boolean;
      deviceHz: number;
    }
  | { type: 'radio'; gain: number; maxGain: number; canControl: boolean }
  | {
      type: 'waterfall';
      channel: number;
      focus: boolean;
      utcMs: number;
      fLoHz: number;
      binHz: number;
      levels: number[];
    }
  | { type: 'yielded' }
  | { type: 'noChannelFits'; deviceHz: number }
  | {
      type: 'streaming';
      /** The window's number of each channel of this server, in its own order. */
      channels: number[];
      rate: number;
      decimation: number;
      centerHz: number;
      deviceHz: number;
      active: boolean[];
      channelizer: string;
    }
  | { type: 'moved'; deviceHz: number; iqHz: number }
  | ({ type: 'decode' } & DecodeRow)
  | ({ type: 'jtty' } & JttyRow)
  | { type: 'gap'; messages: number; atS: number }
  | { type: 'reanchor'; byS: number }
  | { type: 'clock'; text: string }
  | { type: 'step'; index: number | null; of: number; endsUtcS: number; held: boolean }
  | ({ type: 'status' } & Status)
  | { type: 'off' }
  | { type: 'disconnected'; error: string };

/** An event and the server (an index into `Settings.servers`) it came from. */
export type UiEvent = UiEventBody & { server: number };

export interface RadioState {
  gain: number;
  maxGain: number;
  canControl: boolean;
}

/** What to look for in the database; mirrors skimmer_core::store::Query. */
export interface Query {
  /** UTC seconds. */
  since: number;
  until: number;
  me: string;
  call: string;
  grid: string;
  text: string;
  bands: string[];
  modes: string[];
  snrMin: number | null;
  snrMax: number | null;
  kmMin: number | null;
  kmMax: number | null;
  bearingFrom: number | null;
  bearingTo: number | null;
  /** null: any message; '*': any CQ; '': plain CQ; 'DX', 'POTA'... */
  cq: string | null;
  /** Only what these servers heard (by name); empty is all. */
  servers: string[];
}

export interface Activity {
  /** UTC hour since the epoch. */
  hour: number;
  band: string;
  stations: number;
  decodes: number;
  bestSnr: number;
}

export interface StationRow {
  call: string;
  grid: string | null;
  bands: string;
  /** Comma-separated servers that heard it. */
  servers: string;
  count: number;
  bestSnr: number;
  first: number;
  last: number;
  bearing: number | null;
  km: number | null;
}

export interface Spot {
  t: number;
  server: string;
  call: string | null;
  grid: string | null;
  band: string;
  mode: string;
  audioHz: number;
  snr: number;
  dt: number;
  cq: string | null;
  text: string;
  bearing: number | null;
  km: number | null;
  /** What the decoder knew of the row; null for a file from before it was kept. */
  sync: number | null;
  hardErrors: number | null;
  resolved: boolean | null;
}

export interface MapPoint {
  /** Start of the slice, UTC seconds. */
  t: number;
  call: string;
  grid: string;
  band: string;
  snr: number;
}

export interface Summary {
  decodes: number;
  stations: number;
}

/** What the database holds and how big it is. */
export interface DbInfo {
  path: string;
  bytes: number;
  walBytes: number;
  reclaimable: number;
  decodes: number;
  stations: number;
  first: number | null;
  last: number | null;
  /** [server, decodes]; the legacy single server is "". */
  servers: [string, number][];
}
