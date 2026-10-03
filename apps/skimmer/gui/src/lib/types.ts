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
  /** This channel's own call and locator; empty uses Settings'. */
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
  /** Its step in that server's rotation (an index into `stepMinutes`). */
  step?: number;
}

/** One SpyServer. */
export interface ServerSetting {
  /** Shown in the window and kept in the database; unique. */
  name: string;
  address: string;
  /** Where its antenna is (a locator): the origin of the bearings of what it hears. */
  grid: string;
  /** Fixed delay between the SDR and this PC, taken off arrival times, ms. */
  networkDelayMs: number;
  /** Hold control and tune the radio even beside an operator's client. */
  tune: boolean;
  /** Leave control to an SDR# started later, instead of holding it. */
  yieldControl: boolean;
  /** A rotation: minutes of each step; a channel's `step` says which it is heard in. One or none: no rotation. */
  stepMinutes: number[];
}

export interface Settings {
  servers: ServerSetting[];
  /** The channels of every server, in one list. */
  channels: ChannelSetting[];
  format: WireFormat;
  /** Draw the channels' waterfalls; fine is 1.5 Hz per bin instead of 2.9. */
  waterfall: boolean;
  waterfallFine: boolean;
  /** The PC clock as it is, or corrected against an NTP server. */
  clockSource: 'system' | 'ntp';
  ntpServer: string;
  channelizer: 'auto' | 'direct' | 'pfb';
  /** Every decode in a SQLite file, for the Analysis view. */
  dbEnabled: boolean;
  logEnabled: boolean;
  /** Folder the ALL.TXT is written in. */
  logDir: string;
  /** The operator, for the QSO-context AP. */
  myCall: string;
  myGrid: string;
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
}

export interface Status {
  streamedS: number;
  delayMs: number;
  driftMs: number;
  longestPushMs: number;
  queuedBytes: number;
  queuedSlots: number;
  droppedSlots: number;
  longestDecodeMs: number;
  gaps: number;
  reanchors: number;
  /** The NTP line; empty on the PC clock. */
  clock: string;
}

export type UiEventBody =
  | { type: 'connecting'; address: string }
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
  | { type: 'gap'; messages: number; atS: number }
  | { type: 'reanchor'; byS: number }
  | { type: 'clock'; text: string }
  | { type: 'step'; index: number; of: number; endsUtcS: number }
  | ({ type: 'status' } & Status)
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

export interface DtPoint {
  t: number;
  medianS: number;
  n: number;
}
