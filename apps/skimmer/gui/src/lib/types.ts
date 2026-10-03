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
  /** The QSO in progress; nQSOProgress as a name or 0-5. */
  hisCall?: string | null;
  hisGrid?: string | null;
  progress?: string | null;
  /** An ncontest activity by name; empty is none. */
  contest?: string | null;
  averaging?: boolean;
  deepSearch?: boolean;
  emeDelay?: boolean;
}

export interface Settings {
  server: string;
  channels: ChannelSetting[];
  format: WireFormat;
  tune: boolean;
  channelizer: 'auto' | 'direct' | 'pfb';
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
}

export type UiEvent =
  | { type: 'connecting'; server: string }
  | {
      type: 'connected';
      deviceKind: number;
      maxRate: number;
      bandwidthHz: number;
      control: boolean;
      deviceHz: number;
    }
  | { type: 'yielded' }
  | { type: 'noChannelFits'; deviceHz: number }
  | {
      type: 'streaming';
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
  | ({ type: 'status' } & Status)
  | { type: 'disconnected'; error: string };
