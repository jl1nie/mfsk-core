//! SNR 正規化 (Phase 3 連動).
//!
//! mfsk-core の `compute_snr_db_generic` には 2 つ既知バイアスがある:
//!
//! 1. **i16 fixed-point per-block auto-gain** — `fill_symbol_spectra` が
//!    block ごとに gain を変えるため、cs[k] magnitude は **同一 block 内でのみ
//!    比較可能**。per-decode の xsig/xnoi 比は保たれて WSJT-X 互換だが、
//!    cross-decode 絶対値比較は信頼できない。
//!
//! 2. **NFFT_SPEC=8192 (1.46 Hz/bin) のサイドロブ漏れ** — FT8 tone 間
//!    6.25 Hz の隣接強信号が antipodal 推定 (`cs[(itone+4)%8]`) に混入し、
//!    強信号近傍の弱信号 SNR が低めにバイアス。qso3_busy 系で顕著。
//!
//! 本モジュールは両者を吸収するためのワークアラウンド:
//! - **noise floor は decode_block 経路に依存せず、slot 全体の STFT
//!   magnitude P25** から取る (cross-block 一貫性あり、サイドロブ平均化)
//! - **calibration offset** (WSJT-X reference との median 差) を 1 度
//!   算出して固定値で適用
//!
//! Phase 3 の WAV 駆動 UI 構築時に校正値を実測。

#![allow(dead_code)]

/// Residual offset for the SNR shown in the UI. mfsk-core 0.5.8's
/// `xsnr2_db_simple` already lands within ±3 dB of WSJT-X / JTDX
/// across the qso3_busy reference (median-window noise floor
/// rejects per-block auto-gain contamination), so this offset is
/// kept at zero for the wav_sim path. Tweak it only when a board
/// has a measurable RF analog gain offset that survives the
/// xsnr2 normalisation.
pub const DEFAULT_CALIBRATION_OFFSET_DB: f32 = 0.0;

/// SNR の 5 段階品質バケット (UI 表示推奨)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SnrBucket {
    VeryStrong, // ≥ -5 dB  (強い、1分以内に応答可能)
    Strong,     // -10..-5
    Medium,     // -15..-10
    Weak,       // -20..-15 (取れたらラッキー)
    VeryWeak,   // < -20    (ノイズすれすれ、再送願い)
}

impl SnrBucket {
    pub fn from_db(db: f32) -> Self {
        if db >= -5.0 {
            Self::VeryStrong
        } else if db >= -10.0 {
            Self::Strong
        } else if db >= -15.0 {
            Self::Medium
        } else if db >= -20.0 {
            Self::Weak
        } else {
            Self::VeryWeak
        }
    }

    pub fn glyph(&self) -> &'static str {
        match self {
            Self::VeryStrong => "++++",
            Self::Strong => "+++",
            Self::Medium => "++",
            Self::Weak => "+",
            Self::VeryWeak => "·",
        }
    }
}

/// `200..2700 Hz` の STFT magnitude から noise floor を P25 で推定し、
/// EMA で smoothing する tracker。tx_picker と表示 SNR の両方で共有。
pub struct NoiseFloorTracker {
    /// EMA された noise floor magnitude (linear 単位)。
    ema_floor: f32,
    alpha: f32,
    calibration_offset_db: f32,
}

impl NoiseFloorTracker {
    pub fn new() -> Self {
        Self {
            ema_floor: f32::NAN,
            alpha: 0.3,
            calibration_offset_db: DEFAULT_CALIBRATION_OFFSET_DB,
        }
    }

    /// 各 slot 終端で 1 度呼ぶ。`mags` は 200..2700 Hz に対応する
    /// 線形 magnitude (sqrt(power) でも power そのものでも OK、内部で
    /// 統一して保持)。
    pub fn update(&mut self, mags: &[f32]) {
        if mags.is_empty() {
            return;
        }
        // 簡易 P25: heap 不使用で配列を sort してインデックス参照。slot
        // ごと数百要素なので O(N log N) でも問題なし。
        let mut buf: heapless::Vec<f32, 1024> = heapless::Vec::new();
        for &m in mags.iter().take(buf.capacity()) {
            let _ = buf.push(m);
        }
        buf.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
        let p25 = buf[buf.len() / 4];

        if self.ema_floor.is_nan() {
            self.ema_floor = p25;
        } else {
            self.ema_floor = self.alpha * p25 + (1.0 - self.alpha) * self.ema_floor;
        }
    }

    /// 現在の noise floor magnitude (linear)。tx_picker の occupancy 正規化に。
    pub fn noise_floor_lin(&self) -> f32 {
        self.ema_floor
    }

    /// `compute_snr_db` の生値を WSJT-X-comparable に直す。
    /// 校正 offset を加えるだけ (per-decode の SNR 計算自体は decode_block
    /// 内で済んでいるため、ここでは bias 補正のみ)。
    pub fn normalize(&self, raw_snr_db: f32) -> f32 {
        raw_snr_db + self.calibration_offset_db
    }

    /// Phase 3 校正で確定した offset を runtime で差し替える API
    /// (実測値が出たらここに食わせるか、`DEFAULT_CALIBRATION_OFFSET_DB`
    /// を更新して再ビルドする)。
    pub fn set_calibration_offset(&mut self, db: f32) {
        self.calibration_offset_db = db;
    }
}
