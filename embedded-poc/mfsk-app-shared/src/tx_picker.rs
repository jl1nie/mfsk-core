//! TX DF / period selection (Phase 3/4 連動).
//!
//! 帯域内の混雑を避けて TX audio offset (200..2700 Hz) と slot parity
//! (Even / Odd) を選ぶ。手動オーバーライドあり。
//!
//! 設計:
//! - 直近 N slot (~10) の rolling occupancy: (parity, df_bin) → energy + decode_count
//! - df_bin は 10 Hz 解像度で 250 bin (200..2700 Hz)
//! - score 関数で quiet 度を評価し top-K (5) を抽出、guard_band ±50 Hz で多様性確保
//! - 結果は WF 上にマーカーで表示、BtnB で巡回、BtnA で確定
//!
//! `OccupancyMap.energy` は `snr_norm::NoiseFloorTracker.noise_floor_lin()`
//! で割って正規化 → AGC / SFFT サイドロブバイアス無視で quiet 判定可能。

#![allow(dead_code)]

use heapless::Vec;

pub const NUM_BINS: usize = 250; // 200..2700 Hz, 10 Hz 解像度
pub const BIN_HZ: u16 = 10;
pub const BIN_BASE_HZ: u16 = 200;
pub const GUARD_HZ: u16 = 50;
pub const HISTORY_SLOTS: usize = 10;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Parity {
    Even,
    Odd,
}

impl Parity {
    pub fn from_slot_index(slot: u32) -> Self {
        if slot % 2 == 0 {
            Parity::Even
        } else {
            Parity::Odd
        }
    }
}

#[derive(Debug, Clone)]
pub struct TxCandidate {
    pub df_hz: u16,
    pub parity: Parity,
    pub score: f32,
    pub reason: TopReason,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TopReason {
    Quietest,           // energy が最も低い
    DiverseFromBest,    // best から GUARD_HZ 以上離れた次善
    Standard,           // 1500 Hz 中心バイアス採用
    OppositeParity,     // 自局直近 TX と反対 parity 推奨
}

/// Rolling 占有マップ。slot 終端で `commit_slot` を呼んで進める。
pub struct OccupancyMap {
    /// energy[parity][bin] — `noise_floor_lin` で割って正規化済み (dB 単位ではなく linear ratio)。
    energy_norm: [[f32; NUM_BINS]; 2],
    /// 直近 HISTORY_SLOTS slot 内のデコード数。
    decode_count: [[u8; NUM_BINS]; 2],
    /// 自局直近 TX (df, parity, slot_index)。
    last_self_tx: Option<(u16, Parity, u32)>,
    /// EMA 係数。
    alpha: f32,
    /// 累積 slot index。
    slot_index: u32,
}

impl OccupancyMap {
    pub const fn new() -> Self {
        Self {
            energy_norm: [[0.0; NUM_BINS]; 2],
            decode_count: [[0; NUM_BINS]; 2],
            last_self_tx: None,
            alpha: 0.3,
            slot_index: 0,
        }
    }

    /// このスロットの STFT (200..2700 Hz の bin 当たり magnitude) と
    /// noise floor から energy を更新。`mags_per_bin.len() == NUM_BINS`
    /// に decimate 済みを期待 (caller 側で 10 Hz binning)。
    pub fn ingest_slot_energy(&mut self, parity: Parity, mags_per_bin: &[f32], noise_floor_lin: f32) {
        let p = parity as usize;
        let nf = noise_floor_lin.max(f32::EPSILON);
        for (i, &m) in mags_per_bin.iter().enumerate().take(NUM_BINS) {
            let new_e = (m / nf).max(0.0);
            self.energy_norm[p][i] = self.alpha * new_e + (1.0 - self.alpha) * self.energy_norm[p][i];
        }
    }

    /// このスロットでデコードされた message の DF を 1 件記録。
    pub fn ingest_decode(&mut self, parity: Parity, df_hz: u16) {
        if df_hz < BIN_BASE_HZ || df_hz >= BIN_BASE_HZ + (NUM_BINS as u16) * BIN_HZ {
            return;
        }
        let bin = ((df_hz - BIN_BASE_HZ) / BIN_HZ) as usize;
        let p = parity as usize;
        self.decode_count[p][bin] = self.decode_count[p][bin].saturating_add(1);
    }

    /// スロット境界で呼んで履歴を 1 段進める (decode_count を軽く減衰)。
    pub fn commit_slot(&mut self) {
        self.slot_index = self.slot_index.wrapping_add(1);
        // decode_count は HISTORY_SLOTS で割った速度で減衰。
        for p in 0..2 {
            for v in self.decode_count[p].iter_mut() {
                if *v > 0 && self.slot_index as usize % HISTORY_SLOTS == 0 {
                    *v = v.saturating_sub(1);
                }
            }
        }
    }

    pub fn record_self_tx(&mut self, df_hz: u16, parity: Parity) {
        self.last_self_tx = Some((df_hz, parity, self.slot_index));
    }

    /// Top-K 候補抽出。多様性のため guard_band 内の重複は skip。
    pub fn propose_candidates(&self, k: usize) -> Vec<TxCandidate, 8> {
        let mut all: Vec<(f32, u16, Parity), { NUM_BINS * 2 }> = Vec::new();
        for p_idx in 0..2 {
            let parity = if p_idx == 0 { Parity::Even } else { Parity::Odd };
            for bin in 0..NUM_BINS {
                let df = BIN_BASE_HZ + (bin as u16) * BIN_HZ;
                let score = self.score(parity, df);
                let _ = all.push((score, df, parity));
            }
        }
        all.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(core::cmp::Ordering::Equal));

        let mut out: Vec<TxCandidate, 8> = Vec::new();
        for &(score, df, parity) in all.iter() {
            if out.len() >= k.min(out.capacity()) {
                break;
            }
            // GUARD_HZ 内の既選定 candidate を排除。
            let too_close = out.iter().any(|c| {
                c.parity == parity && (c.df_hz as i32 - df as i32).abs() < GUARD_HZ as i32
            });
            if too_close {
                continue;
            }
            let reason = if out.is_empty() {
                TopReason::Quietest
            } else {
                TopReason::DiverseFromBest
            };
            let _ = out.push(TxCandidate {
                df_hz: df,
                parity,
                score,
                reason,
            });
        }
        out
    }

    fn score(&self, parity: Parity, df_hz: u16) -> f32 {
        let p = parity as usize;
        let bin = ((df_hz - BIN_BASE_HZ) / BIN_HZ) as usize;
        let energy_pen = self.energy_norm[p][bin];
        let decode_pen = self.decode_count[p][bin] as f32;
        // 1500 Hz 中心への弱バイアス (passband 中央の方が AGC / フィルタ的に安定)。
        let center = 1500.0_f32;
        let center_bonus = -(((df_hz as f32) - center).abs() / 1500.0) * 0.2;
        // 直近自局 TX と同 parity ペナルティ。
        let same_parity_pen = if let Some((_, last_par, _)) = self.last_self_tx {
            if last_par == parity {
                0.5
            } else {
                0.0
            }
        } else {
            0.0
        };

        // 高いほど良い。
        -2.0 * energy_pen - 1.0 * decode_pen - same_parity_pen + center_bonus
    }
}
