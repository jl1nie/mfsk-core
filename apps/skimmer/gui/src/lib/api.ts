import { invoke } from '@tauri-apps/api/core';
import { listen, type UnlistenFn } from '@tauri-apps/api/event';
import { open } from '@tauri-apps/plugin-dialog';
import type { ChannelSetting, ModeInfo, Settings, UiEvent } from './types';

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
export const setStation = (myCall: string, myGrid: string) => invoke<void>('set_station', { myCall, myGrid });
