import type { ChannelSetting } from './types';

// WSJT-X's default frequency list (models/FrequencyList.cpp,
// `default_frequency_list`, checkout 2b9d65408, wsjtx-2.7.0-rc4a), the
// entries for all regions and for IARU Region 3, 160 m to 6 m. One addition
// that list does not carry: FT8 at 7041 kHz, where JA stations run FT8 on
// 40 m (decoded there 2026-10-03, 19-26 stations a slot). And JTTY (#650), from
// v3.3.0-beta1's list (checkout 57ba9337f): 1839, 3572, 7078, 10140, 14090, 18104,
// 21090, 24920, 28090 and 50160 kHz.
export interface Preset {
  band: string;
  label: string;
  channel: ChannelSetting;
}

const p = (band: string, mode: string, dialHz: number, note = ''): Preset => ({
  band,
  label: `${mode} ${(dialHz / 1000).toFixed(1)} kHz${note}`,
  channel: { mode, dialHz },
});

export const PRESETS: Preset[] = [
  p('160m', 'WSPR', 1_836_600),
  p('160m', 'JT65', 1_838_000),
  p('160m', 'JT9', 1_839_000),
  p('160m', 'FT8', 1_840_000),
  p('160m', 'JTTY', 1_839_000),
  p('80m', 'WSPR', 3_568_600),
  p('80m', 'FT4', 3_568_000, ' (R3)'),
  p('80m', 'JT65', 3_570_000),
  p('80m', 'JT9', 3_572_000),
  p('80m', 'FT8', 3_573_000),
  p('80m', 'FT4', 3_575_000),
  p('80m', 'JTTY', 3_572_000),
  p('40m', 'WSPR', 7_038_600),
  p('40m', 'FT8', 7_041_000, ' (JA)'),
  p('40m', 'FT4', 7_047_500),
  p('40m', 'FT8', 7_074_000),
  p('40m', 'JT65', 7_076_000),
  p('40m', 'JT9', 7_078_000),
  p('40m', 'JTTY', 7_078_000),
  p('30m', 'FT8', 10_136_000),
  p('30m', 'JT65', 10_138_000),
  p('30m', 'WSPR', 10_138_700),
  p('30m', 'JT9', 10_140_000),
  p('30m', 'FT4', 10_140_000),
  p('30m', 'JTTY', 10_140_000),
  p('20m', 'FT8', 14_074_000),
  p('20m', 'JT65', 14_076_000),
  p('20m', 'JT9', 14_078_000),
  p('20m', 'FT4', 14_080_000),
  p('20m', 'WSPR', 14_095_600),
  p('20m', 'JTTY', 14_090_000),
  p('17m', 'FT8', 18_100_000),
  p('17m', 'JT65', 18_102_000),
  p('17m', 'JT9', 18_104_000),
  p('17m', 'FT4', 18_104_000),
  p('17m', 'WSPR', 18_104_600),
  p('17m', 'JTTY', 18_104_000),
  p('15m', 'FT8', 21_074_000),
  p('15m', 'JT65', 21_076_000),
  p('15m', 'JT9', 21_078_000),
  p('15m', 'WSPR', 21_094_600),
  p('15m', 'FT4', 21_140_000),
  p('15m', 'JTTY', 21_090_000),
  p('12m', 'FT8', 24_915_000),
  p('12m', 'JT65', 24_917_000),
  p('12m', 'JT9', 24_919_000),
  p('12m', 'FT4', 24_919_000),
  p('12m', 'WSPR', 24_924_600),
  p('12m', 'JTTY', 24_920_000),
  p('10m', 'FT8', 28_074_000),
  p('10m', 'JT65', 28_076_000),
  p('10m', 'JT9', 28_078_000),
  p('10m', 'WSPR', 28_124_600),
  p('10m', 'FT4', 28_180_000),
  p('10m', 'JTTY', 28_090_000),
  p('6m', 'JT65', 50_276_000, ' (R3)'),
  p('6m', 'WSPR', 50_293_000, ' (R3)'),
  p('6m', 'JT65', 50_310_000),
  p('6m', 'JT9', 50_312_000),
  p('6m', 'FT8', 50_313_000),
  p('6m', 'FT4', 50_318_000),
  p('6m', 'FT8', 50_323_000),
  p('6m', 'JTTY', 50_160_000),
];

export const BANDS = [...new Set(PRESETS.map((x) => x.band))];
