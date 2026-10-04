import { invoke } from '@tauri-apps/api/core';
import { listen, type UnlistenFn } from '@tauri-apps/api/event';
import { confirm, open, save } from '@tauri-apps/plugin-dialog';
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
export const setServerEnabled = (server: number, on: boolean) => invoke<void>('set_server_enabled', { server, on });
export const rotateBand = (server: number, by: number) => invoke<void>('rotate_band', { server, by });
export const setHold = (server: number, hold: boolean) => invoke<void>('set_hold', { server, hold });
export const setNetworkDelay = (server: number, ms: number) => invoke<void>('set_network_delay', { server, ms });
export const setStation = (server: number, myCall: string, myGrid: string) =>
  invoke<void>('set_station', { server, myCall, myGrid });
export const setGain = (server: number, gain: number) => invoke<void>('set_gain', { server, gain });
export const radioState = (server: number) => invoke<RadioState | null>('radio_state', { server });
export const setWaterfall = (focus: number | null, fine: boolean) =>
  invoke<void>('set_waterfall', { focus, fine });

import type { Activity, DbInfo, MapPoint, Query, Spot, StationRow, Summary } from './types';
export const dbActivity = (dir: string, q: Query) => invoke<Activity[]>('db_activity', { db: dir, q });
export const dbStations = (dir: string, q: Query, limit: number) =>
  invoke<StationRow[]>('db_stations', { db: dir, q, limit });
export const dbDecodes = (dir: string, q: Query, limit: number) => invoke<Spot[]>('db_decodes', { db: dir, q, limit });
export const dbPoints = (dir: string, q: Query, sliceS: number) =>
  invoke<MapPoint[]>('db_points', { db: dir, q, sliceS });
export const dbSummary = (dir: string, q: Query) => invoke<Summary>('db_summary', { db: dir, q });
export const dbBands = (dir: string) => invoke<string[]>('db_bands', { db: dir });
export const dbSpan = (dir: string) => invoke<[number | null, number | null, number]>('db_span', { db: dir });
export const dbSnrHist = (dir: string, q: Query) => invoke<[number, number][]>('db_snr_hist', { db: dir, q });
export const dbServers = (dir: string) => invoke<[string, string][]>('db_servers', { db: dir });

/** A yes/no question in the system dialog. */
export const ask = (message: string) => confirm(message, { title: 'Skimmer', kind: 'warning' });

export const dbInfo = (dir: string) => invoke<DbInfo>('db_info', { db: dir });
export const dbCountBefore = (dir: string, before: number, server: string | null) =>
  invoke<number>('db_count_before', { db: dir, before, server });
export const dbDeleteBefore = (dir: string, before: number, server: string | null) =>
  invoke<number>('db_delete_before', { db: dir, before, server });
export const dbVacuum = (dir: string) => invoke<void>('db_vacuum', { db: dir });
export const dbBackup = (dir: string, dest: string) => invoke<void>('db_backup', { db: dir, dest });
export const dbExportCsv = (dir: string, q: Query, dest: string) => invoke<number>('db_export_csv', { db: dir, q, dest });
/** A file name chosen in the system's Save dialog, or null if cancelled. */
/** An existing database file chosen in the system's Open dialog, or null. */
export async function pickDb(defaultPath: string): Promise<string | null> {
  const p = await open({ multiple: false, directory: false, defaultPath: defaultPath || undefined, filters: [{ name: 'Database', extensions: ['db', 'sqlite'] }] });
  return typeof p === 'string' ? p : null;
}
export async function saveAs(defaultPath: string, extension: string): Promise<string | null> {
  return save({ defaultPath, filters: [{ name: extension.toUpperCase(), extensions: [extension] }] });
}
