# FST4 embedded DDC — 解析グリッドの複素パラメータ化

設計ドラフト、2026-08-20。実装前の記録であり、測定値はまだ入っていない
（見積りは「〜」付きで区別してある）。議論の出発点は #307 の VK3NV
コメント（2026-08-19）と、それに対する分類パス（#307, 同日）。

関連 issue: [#307](https://github.com/jl1nie/mfsk-core/issues/307)（非2冪 FFT）、
[#309](https://github.com/jl1nie/mfsk-core/issues/309)（ストリーミング DDC）、
[#306](https://github.com/jl1nie/mfsk-core/issues/306)（FST4 埋め込み実現性の傘）、
[#323](https://github.com/jl1nie/mfsk-core/issues/323)（12 kHz リテラルの分類）。

## 1. なぜ必要か

埋め込み FST4 の `coarse_sync` は**今そのままでは動かない**。
`compute_spectra` が張る `nfft1 = NSPS × NFFT_PER_SYMBOL_FACTOR` は FST4-60 で
7776 で、埋め込み FFT バックエンド
(`embedded-poc/embedded-shared/src/esp_dsp_fft.rs`) は
`len.is_power_of_two() && len >= 4` を assert するため panic する。
非 2 冪長は `DirectDft`（`DIRECT_DFT_MAX_LEN = 64`）しか経路がない。

そして panic するのは `coarse_sync` だけではない。候補精査側の
`downsample_cached` は **fwd 746 496 点（2¹⁰·3⁶）と inv 6 912 点（2⁸·27）**を
要求し、どちらも非 2 冪である。埋め込み FST4 を成立させるには両方が要る。

### なぜ DDC で解くのか

**NSPS = 3888 = 2⁴·3⁵ なので、整数間引きでは 2 冪の `nfft1` に到達できない。**
どんな整数 D で割っても 3ᵏ が残る。`nfos` を変えても（`nfft1 = nfos·nsps`、
`nfos` は整数）3⁻⁵ を供給できないので同じ。**有理リサンプリングが不可避**で、
それを担うのが DDC。

VK3NV の `K = 2^m` 案 — 1 シンボルが 2 冪個のサンプルになるよう RX 解析レートを
選べば `nfft1 = 2K` が**構成上つねに radix-2** になる — がこの設計の出発点。

### 主目的はメモリ削減ではない

VK3NV 自身が #307 で書いている通り:

> For a wideband search the transform-size reduction may only be around 2×;
> for a narrow search it can be much larger. **The more important point is
> guaranteed access to ESP-DSP's optimized radix-2 path.**

実数広帯域だと Nyquist 制約から間引きは 2 倍弱にしかならない。**得られるのは
Bluestein 実装も Kconfig 変更も不要になること**であって、メモリではない。

## 2. 不変量

`K` をどう選んでも物理格子は変わらない（VK3NV の主張、コードで確認済み）:

| 量 | 式 | FST4-60 |
|---|---|---|
| `df` | `Fs/nfft1 = 1/(2·SYMBOL_DT)` | 1.5432 Hz（レート不変） |
| `tstep` | `nstep/Fs = SYMBOL_DT/2` | 0.162 s（レート不変） |
| `nhsym` | `2·T_SLOT/SYMBOL_DT − 3` | 367 行（レート不変） |
| `nfos` | `nfft1/nsps` | 2（`i + nfos·k` のトーン索引が保たれる） |

**スペクトログラムの形は K に依らない。**変わるのは FFT 長と音声バッファだけ。

## 3. スコープ

統一・複素パラメータ化。`(center_freq, bandwidth)` で広帯域とスナイパーの
両方を同一機構で表現する。最初の実装対象は **FST4-60 のみ**。
FST4-120/300 は今回対象外（K の切り上げで `nfft1` が 8192 上限を超えるため、
別途 Kconfig 変更が要る — §8 参照）。

## 4. 設計

### 4.1 `RxGrid` — 解析グリッド記述子（新規、`engine::sync`）

VK3NV の `RxAnalysisDescriptor` に相当。ビン↔Hz 写像を 1 箇所に集約する。

```rust
pub struct RxGrid {
    pub sample_rate_hz: f32,
    pub center_hz: f32,     // 0.0 = 実数・DC 基準（現行 12 kHz 経路）
    pub complex_input: bool,
}
```

- 実数経路: `bin = hz/df`、`usable = nh1` → **現行と完全に同一挙動**
- 複素経路: **fftshift-on-store**、`bin = (hz−center)/df + nfft1/2`、`usable = nfft1`

**fftshift が効く理由**: 周波数軸が単調になるので `coarse_sync` の相関ループ
（`abs_bin = i + d.nfos * k`、`engine/sync.rs`）が**無変更のまま**動く。
ラップ処理がホットループに入らない。変更が要るのは `ia`/`ib` の算出と
`nh1` クランプだけで、どちらも `RxGrid` メソッド呼び出しに置き換わる。

### 4.2 `engine::dsp::polyphase` — 有理リサンプラ（新規・汎用）

既存の `engine::dsp::fir_decimate`（`FirStage` / `design_lowpass`）は
**整数間引き専用**。有理 L/M を担う兄弟モジュールを追加する。
`design_lowpass` はそのまま再利用。`FirStage` と同じく I/Q を
**planar 分離**して保持する（esp-dsp が実数ストリーム前提なのでこの形が活きる）。

### 4.3 `engine::dsp::ddc` — 汎用複素ダウンコンバータ（新規）

```
12 kHz 実数 i16
  └ mixer (exp(-j2π·center·n/Fs))            → 複素 12 kHz
  └ FirStage 整数段カスケード（緩いフィルタ）  → 複素 12000/D
  └ PolyphaseResampler（鋭いフィルタ・最終段） → 複素 Fs_c = K/SYMBOL_DT
  └ compute_spectra（複素入力）                → Spectrogram (nfft1 = 2K)
```

鋭い段を**最後（最低レート）**に置くのが要点。単段だと総 MAC 数は間引き率に
依らず遷移帯域比だけで決まる（＝狭帯域にしても安くならない）ため、WSPR が
`StreamingDdcCascade` を必要としたのと同じ理由でカスケードが必須。

FST4-60 の粗同期段パラメータ（見積り）:

| 用途 | 必要 BW | K | Fs_c | nfft1 | 全体比 | カスケード | 〜MAC/slot (I+Q) |
|---|---:|---:|---:|---:|---:|---|---:|
| スナイパー ±250 Hz | 600 Hz | 256 | 790.12 Hz | **512** | 16/243 | /3 → 16/81 (〜84 taps/phase) | 〜15M |
| 広帯域 100–3000 Hz | 2900 Hz | 1024 | 3160.49 Hz | **2048** | 64/243 | /1 → 64/243 (〜184 taps/phase) | 〜79M |

いずれも radix-2 かつ 8192 上限内。単段実装比でそれぞれ 〜3.1× / 〜1.8× 安い。

### 4.4 refine 段 — 有理リサンプラは不要

**K と K′ が両方 2 冪なら比 K/K′ は必ず 2 冪。**この帰結が refine 段を単純にする。

現行 refine は `ds_spb = NSPS/NDOWN = 36`、`ds_rate = 111.11 Hz`。
DDC ベースバンド（`Fs_c = K/SYMBOL_DT`）からここへ降ろす比は:

| 用途 | K | Fs_c | ds_spb | ds_rate | 比 | 段の形 |
|---|---:|---:|---:|---:|---:|---|
| スナイパー | 256 | 790.12 Hz | 36 | 111.11 Hz | 9/64 | 有理（L=9、小さい） |
| 広帯域 | 1024 | 3160.49 Hz | 36 | 111.11 Hz | 9/256 | 有理（L=9、小さい） |

refine 帯域占有は 4 tones + 1.5+1.5 pad = 7 tones = 21.6 Hz で、111 Hz の
ベースバンドに対して余裕が大きいためフィルタは緩く済む。

**`ds_spb = 32` にすれば全段 radix-2 になる**（比が 1/8 と 1/32 の純 2 冪整数
間引きになり、`symbol_spectra` も 36 点 `DirectDft` → 32 点 radix-2 になる）。
魅力的だが **`NDOWN` 幾何を WSJT-X から動かす**変更であり、`symbol_spectra` /
`fine_sync_power` / Costas 参照表 / LLR 経路すべてに波及して**フロントエンド
ではなくデコード挙動を変える**。#23（FST4-60A ゴールデン固定 — `NSPS`/`NDOWN`/
`GFSK_BT` の取り違え）の前例もあるため、**既定は `ds_spb = 36` 据え置き**とし、
`ds_spb = 32` は感度スイープを通した上での後追い候補として記録に留める。

### 4.5 esp-dsp / LX7 PIE 経路

**接続点は「タップ 1 本」ではなく FIR 段まるごと。**esp-dsp の `fir_f32_t` が
自前の遅延線と位置状態を持つため、`dot()` だけ差し替える形にすると状態機械が
二重になる。`fft-extern`（`engine/fft.rs` の
`mfsk_core_make_default_fft_planner`）と同じ extern フック方式にする:

```rust
// engine/dsp/fir_kernel.rs
pub trait FirDecimator {
    fn push_block(&mut self, xi: &[f32], xq: &[f32],
                  yi: &mut Vec<f32>, yq: &mut Vec<f32>);
}
pub fn default_fir_decimator(..) -> Box<dyn FirDecimator>;
```

`embedded-shared` 側が `dsps_fird_f32_aes3`（LX7 PIE）を I/Q 各 1 本の
`fir_f32_t` として束ねる。esp-dsp バインディングは現状 FFT 系のみ
（`esp_dsp_fft.rs`）なので FIR 系は新規追加。Phase D1（commit `053bd67`）で
FFT を `_ae32_` → `_aes3_` に移した前例に倣う（`docs/notes/PHASE_D_PIE_SIMD.md`）。

**block API が前提**: esp-dsp の FIR は `(fir, input, output, len)` のブロック形。
現行 `FirStage::push_one` はサンプル単位なので `push_block` を**追加**する
（既存 `push_one` と `wspr::ddc` は無変更のまま）。FFI 呼び出しコストが
ブロック長で償却されるのも block 形が要る理由。

**実装時に header で要確認**（この環境に esp-dsp が無く未検証）:

- `dsps_fird_f32_aes3` のタップ数制約（4 の倍数か 8 の倍数か）
- coeffs / delay の 16 バイト境界要求
- `len` が `decim` の倍数である必要の有無

**タップ数の衝突と解法**: `FirStage::new` は `ntaps` 奇数を assert する
（線形位相・整数群遅延のため）が、PIE は 4 の倍数を要求する見込み。
対称・奇数長のコアを保ったまま**末尾にゼロタップを詰めて**揃える。
ゼロ詰め版と非ゼロ詰め版が 1 サンプルオフセットを除いて bit-identical で
あることをテストで固定する。16 バイト境界は既存の `Align16Quad` /
`AlignedComplexBuf`（`engine/fft.rs`）と commit `1b14f56`
(`perf(wspr): 16-byte-align subtract's FFT buffers`) の先例に倣う。

## 5. 変更ファイル

**新規**

- `mfsk-core/src/engine/dsp/polyphase.rs` — 有理リサンプラ
- `mfsk-core/src/engine/dsp/ddc.rs` — ミキサ + カスケード組み立て
- `mfsk-core/src/engine/dsp/fir_kernel.rs` — `FirDecimator` + extern フック
- `mfsk-core/src/fst4/ddc.rs` — FST4 固有の K 選定 / カスケード分解
- `embedded-poc/embedded-shared/src/esp_dsp_fir.rs` — esp-dsp `dsps_fird_f32_aes3`

**変更**

- `mfsk-core/src/engine/sync.rs` — `RxGrid`、`compute_spectra` 複素経路、
  `coarse_sync` の `ia`/`ib`/`nh1`（相関ループ本体は無変更）
- `mfsk-core/src/engine/pipeline.rs` — refine を DDC ベースバンド起点に
- `mfsk-core/src/engine/dsp/fir_decimate.rs` — `push_block` 追加（`push_one` 保持）
- `mfsk-core/src/engine/dsp/mod.rs`, `mfsk-core/Cargo.toml` — 登録

## 6. 検証

WSPR の DDC が通った基準（`wspr-ddc` / `wspr-ddc-cascade`）を踏襲する。

1. **実数経路の非回帰（最優先）** — `RxGrid { center: 0.0, complex_input: false }`
   で既存マージゲートが緑のまま。`sync_dims_of_matches_nsps_at_12khz` と
   同じ形で bit-exact を固定。
2. **リサンプラ単体** — 通過帯域内単一トーンの振幅/位相、帯域外除去
   （エイリアス折返しが指定ストップバンド以下）、ゼロ詰めタップ版の bit-identical。
3. **エンドツーエンド等価（本命）** — FST4-60 ゴールデン WAV
   (`210115_0058.wav`, N5TM @1101 Hz / K9KFR @1331 Hz) で、12 kHz 実数
   `coarse_sync` と DDC 経由の候補集合が freq/dt 許容内で一致すること。
4. **感度** — `scripts/run-sensitivity-sweeps.sh fst4` で 50% クロッシングが
   `docs/notes/sweep-baseline.json` から 0.5 dB 以上動かないこと。
5. **実機** — `cargo +esp build --release --bin fst4-bench` が通り、
   `coarse_sync` が S3 上で **panic せず完走**すること（今日時点では
   `is_power_of_two` assert で落ちる）。`embedded-poc/scripts/flash-monitor.sh`
   でログ取得。

```sh
# 1
MFSK_REQUIRE_CORPUS=1 cargo test -p mfsk-core --features full,internal-testing --release
# 3
MFSK_REQUIRE_CORPUS=1 cargo test -p mfsk-core --features full,internal-testing --release \
    --test fst4_wsjtx_samples
# 4
scripts/run-sensitivity-sweeps.sh fst4
```

## 7. 段階

1. `polyphase.rs` + `fir_decimate::push_block`（純ホスト、検証 2）
2. `RxGrid` + `compute_spectra` 複素経路（検証 1 で非回帰を固定）
3. `ddc.rs` + `fst4/ddc.rs` の粗同期段（検証 3）
4. refine 段を DDC ベースバンド起点に（検証 3 + 4）
5. esp-dsp `dsps_fird_f32_aes3` バックエンド + 実機（検証 5）

段 1-4 はホストのみで完結し、段 5 の esp-dsp 差し替えは `FirDecimator` の
入れ替えなので前段の検証結果を無効化しない。

## 8. 未解決 / 対象外

- **FST4-120/300**: K の 2 冪切り上げで `nfft1` がそれぞれ 16384 / 32768 と
  なり 8192 上限を超える。`CONFIG_DSP_MAX_FFT_SIZE_32768` への変更で解けるが
  今回対象外。FST4-120 は必要 K が 4100 で 4096 をわずかに超えるため次の 2 冪
  まで切り上がり、**実効間引きがほぼ 1.0 倍になる**（rate が 12 kHz に戻る）
  という縮退が起きる点は記録しておく。
- **デコーダ側のギャップは別問題**: #306 の結論どおり、フロントエンドを
  完璧にしてもデコーダの超過（`full` 40.1 s / `no8_osd` 13.6 s 対 予算 7.2 s）は
  埋まらない。本設計は #307 の FFT 制約を解くものであって、#306 の
  時間予算問題を解くものではない。
- **`ds_spb = 32` 全段 radix-2 案**: §4.4 参照。感度スイープを通した上での
  後追い候補。
