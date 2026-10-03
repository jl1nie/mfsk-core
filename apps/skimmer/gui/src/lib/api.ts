import { invoke } from '@tauri-apps/api/core';
import { listen, type UnlistenFn } from '@tauri-apps/api/event';
import { open } from '@tauri-apps/plugin-dialog';
import type { ChannelSetting, ModeInfo, RadioState, Settings, UiEvent } from './types';

export const loadSettings = () => invoke<Settings>('load_settings');
export const saveSettings = (settings: Settings) => invoke<void>('save_settings', { settings });
export const modes = () => invoke<ModeInfo[]>('modes');
export const autoPfbChannels = () => invoke<number>('auto_pfb_channels');
export const start = (settings: Settings) => invoke<void>('start', { settings });
export const stop = () => invoke<void>('stop');
export const onEvent = (cb: (e: UiEvent) => void): Promise<UnlistenFn> =>
  listen<UiEvent>('skimmer', (e) => cb(e.payload));

/** A folder chosen in the system dialog, or null if cancelled. */
export async function pickFolder(defaultPath: string): Promise<string | null> {
  const picked = await open({ directory: true, multiple: false, defaultPath: defaultPath || undefined });
  return typeof picked === 'string' ? picked : null;
}
export const setChannelOptions = (index: number, channel: ChannelSetting) =>
  invoke<void>('set_channel_options', { index, channel });
export const autostartRequested = () => invoke<boolean>('autostart_requested');
export const setNetworkDelay = (ms: number) => invoke<void>('set_network_delay', { ms });
export const setStation = (myCall: string, myGrid: string) => invoke<void>('set_station', { myCall, myGrid });
export const setGain = (gain: number) => invoke<void>('set_gain', { gain });
export const radioState = () => invoke<RadioState | null>('radio_state');
export const setWaterfall = (focus: number | null, fine: boolean) =>
  invoke<void>('set_waterfall', { focus, fine });

import type { Activity, DtPoint, MapPoint, Query, Spot, StationRow, Summary } from './types';
export const dbActivity = (dir: string, q: Query) => invoke<Activity[]>('db_activity', { dir, q });
export const dbStations = (dir: string, q: Query, limit: number) =>
  invoke<StationRow[]>('db_stations', { dir, q, limit });
export const dbDecodes = (dir: string, q: Query, limit: number) => invoke<Spot[]>('db_decodes', { dir, q, limit });
export const dbPoints = (dir: string, q: Query, sliceS: number) =>
  invoke<MapPoint[]>('db_points', { dir, q, sliceS });
export const dbSummary = (dir: string, q: Query) => invoke<Summary>('db_summary', { dir, q });
export const dbBands = (dir: string) => invoke<string[]>('db_bands', { dir });
export const dbDt = (dir: string, bucketS: number, since: number, until: number) =>
  invoke<DtPoint[]>('db_dt', { dir, bucketS, since, until });
export const dbSpan = (dir: string) => invoke<[number | null, number | null, number]>('db_span', { dir });
