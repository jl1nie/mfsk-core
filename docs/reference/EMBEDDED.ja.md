# 組込ターゲット

`mfsk-core` は `no_std + alloc` 対応で、FT8 デコードパス
(`mfsk_core::ft8::decode_block`) はキャラクタ側 FFT バックエンドが
あれば **~150 KB の RAM** のチップでも動く。本文書は組込統合者向け
のリファレンス — ライブラリが呼び出し側に何を要求するか、どんな
scratch バッファが必要か、C ABI の形、現在テストしている対象上で
の性能。

ホスト専用利用 (no embedded) は [`docs/reference/LIBRARY.ja.md`](LIBRARY.ja.md)、
本ライブラリで作った既存 FT8 コントローラの操作は
[`docs/reference/MANUAL_M5STICKS3.ja.md`](MANUAL_M5STICKS3.ja.md) 参照。

## アーキテクチャ: f32 と固定小数点が 1 つのコードベースを共有する仕組み

DSP / FEC パイプライン全体は **scalar trait** でパラメータ化されて
おり、同じソースが host 側 f32 パスと組込側整数パスのいずれにも
**コード重複なし** で compile される:

- [`engine::scalar::SpecScalar`] — spectrogram / DFT 出力 scalar
  (host は `f32`、embedded cs 格納は `Q14i16`)。
- [`engine::scalar::LlrScalar`] — wide-accumulator 付き LLR scalar
  (host は `f32`、組込 BP は **`Q11i16` + i32 wide accumulator**、
  0.6.2 以降。0.5.x までは `Q3i8` だった。拡張の動機は host
  fixed-point + rustfft sweep (pre-0.6.3 計測 — 0.6.3 の OSD
  tightening で f32 host recall は 16/18 → 13/18 に CRC-luck
  phantom 3 件分下がる前) の結果: `qso3_busy.wav` に対し f32 は
  16/18 取れたが `Q3i8` の ~0.875 LLR 量子化ステップが Xtensa 上の
  recall 天井を決め 9/18 まで落ちた — DSP 側ではなく LLR 解像度が
  ボトルネックだった。`Q11i16` (~1/2048 LSB、BP scratch ~6 KB →
  ~12 KB、S3 / Core2 内蔵 DRAM 予算内) で解像度律速を解消、host
  fixed-point recall は f32 同等まで到達 (完全に gap close)。
  ただし実機 embedded の上乗せは 1 件のみ (6/18 → 7 total) —
  残る host gap は LLR scalar ではなく組込パイプラインの他要素
  (NSTEP-half、coarse-sync 簡略化、`fine_refine_pass1` 無し) が
  律速。`Q3i8` 型は比較経路用に `engine::scalar` に残置)。
- [`engine::scalar::Cmplx<S>`] — `SpecScalar` 上のジェネリック複素数。
  0.6.3 (cleanup β.5) 以降は `num_complex::Complex<S>` の type
  alias、組込整数パスと host f32 パスで同じ複素演算実装を共有。
- `compute_llr_generic<P, S, T>`、`compute_snr_db_generic<P, S>`、
  `bp_decode_generic_nms<P, T>` — すべて scalar 型をジェネリック
  パラメータとして取り、`(P, S, T)` の組ごとに 1 monomorphisation。

`fixed-point` Cargo feature は **プロトコル glue がどの scalar 型を
拾うかを切り替えるだけ** — generic 本体は不変。組込ポートと host
ビルドは 99 % のコードを共有しており、バグ修正と最適化は一度の作業
で両方に適用される。

### fixed-point スイッチが現在配線されている範囲

| Component | Generic over | Fixed-point switch 配線済み? |
|---|---|---|
| LDPC BP NMS (`fec::ldpc::bp`) | `LlrScalar` | ✅ `fixed-point` 経由 |
| LLR 計算 (`engine::llr`) | `SpecScalar` × `LlrScalar` | ✅ `fixed-point` 経由 |
| BP scratch pool (`BpScratch<P, T>`) | `LdpcParams` × `LlrScalar` | ✅ — FT8 LDPC(174,91) と FST4/uvpacket LDPC(240,101) で機能 |
| FT8 spectrogram + DFT (`ft8::decode_block`) | `SpecScalar` × `AudioSample` | ✅ `fixed-point` 経由 |
| **FT4 / WSPR / Q65 / JT9 / JT65** | (host f32 のみ) | ❌ — これらは現状 `decode_block` を通っていない |

つまり: **trait 基盤は protocol 非依存だが、組込ビルドで実際に整数
パスに切り替わるプロトコルは FT8 のみ。** FT4 (Costas/Gray/LDPC
piece が共通なので次の候補) を追加するのは FT4 専用シンボルレイアウト
への `decode_block` 型移植であって、trait レイヤに新規要素はない。

## テスト対象

| Target | MCU | Backend | Status |
|---|---|---|---|
| **M5StickS3** | **ESP32-S3 (Xtensa LX7 dual-core, 240 MHz, 8 MB Octal PSRAM, ES8311 codec, ST7789P3 135×240 LCD, KEY1/KEY2)** | esp-dsp `_ae32_` asm (LX6/LX7 共通、scalar single-issue) — LX7 PIE `_aes3_` への移行は Phase D D1 で予定、[`PHASE_D_PIE_SIMD.md`](../notes/PHASE_D_PIE_SIMD.md) 参照 | **デモ / 音響 fallback コントローラ** (2026-05-17 pivot) — `embedded-poc/m5stack-s3-app/` (LCD UI + QSO FSM + BLE CI-V + 音響 mic + WiFi UDP log)。VBUS 源回路が無く USB-OTG host が成立しないため、本命の UAC コントローラ役は CoreS3 に移譲され、StickS3 は音響経路の実機検証 / デモ機としての位置付けに再定義された。 |
| **M5Stack Core2** | **ESP32-D0WD-V3** (Xtensa LX6, dual-core 240 MHz, single-issue f32 FPU, 16 MB flash, ~4 MB PSRAM) — `espflash board-info` 確認: `Chip type: esp32 (revision v3.1)` / `Features: WiFi, BT, Dual Core, 240MHz`。ESP32-S2 (LX7、single-core、BT 無し) や S3 では **ない**。 | esp-dsp ASM (`dsps_dotprod_s16_ae32`、`dsps_fft2r_*`) | **本番アプリ (`wav_sim` 専用)** — `embedded-poc/m5stack-core2-app/` が baked `wav_sim` 音源ループに対し同じ `decode_block` を LX6 上で走らせて `mfsk-app-shared` API を交差検証する役割。古典 ESP32 には USB peripheral が無いので mic / speaker / USB-Host 経路はこのボードでは扱わない — Core2 は共有 QSO FSM の LX6 second-board verifier。(独立した Core2 コンピュート bench `embedded-poc/m5stack-core2/` は #61 Phase 3 (0.6.3) で retired、wav_sim 経路はこの app crate に統合済み。) |
| ESP32-S3 compute bench | Xtensa LX7 | esp-dsp ASM | **タイミング回帰 bench** — `embedded-poc/m5stack-s3/`、缶詰 WAV 入力に対し `decode_block` を走らせ per-stage timing sweep。エンドユーザ向けではない。 |
| **M5Stack CoreS3** | ESP32-S3 LX7 + AXP2101 PMIC + AW9523B I/O expander (P1 の BUS_OUT_EN が VBUS boost 駆動) | esp-dsp `_ae32_` asm (Phase D D1 で `_aes3_` 化、S3-app と共通) | **本命の UAC コントローラ ターゲット** (Phase B-Core、2026-05-17 pivot) — `embedded-poc/m5stack-cores3-app/`。Phase 0-Core (bringup) + Phase 1-Core (AW9523B BUS_OUT_EN + UAC host) は commit `1a93c92` で出荷済み。M5StickS3 に無い VBUS 源回路を持つので、IC-705 への USB-Host audio class はここで実装する。`docs/notes/ROADMAP.md` Phase B-Core 参照。 |

### その他のターゲット — 検証済 vs 願望

`fft-extern` 契約はターゲット移植可能になるよう **設計** されており、
`mfsk-ffi-ft8` は複数の非 Xtensa MCU に clean に cross-build できる:

| Target | `cargo build` clean | FFT shim 提供 | HW テスト済 |
|---|---|---|---|
| `xtensa-esp32-espidf` | ✅ | ✅ esp-dsp (Core2) | ✅ qso1/2/3 sweep |
| `xtensa-esp32s3-espidf` | ✅ | ✅ esp-dsp (S3 bench + S3-app + CoreS3-app bring-up 中、Phase B-Core) | ✅ qso1/2/3 sweep |
| `thumbv8m.main-none-eabihf` (RP2350 Cortex-M33) | ✅ | ❌ 候補: pico-sdk-rs 経由 CMSIS-DSP | ❌ |
| `riscv32imac-unknown-none-elf` (RP2350 Hazard3) | ✅ | ❌ DSP ライブラリ無し、FFT は `microfft` | ❌ |
| `thumbv7em-none-eabihf` (Cortex-M4F / M7) | 未試行 | ❌ 候補: CMSIS-DSP `arm_*_q15` | ❌ |
| `thumbv6m-none-eabi` (Cortex-M0+ / RP2040) | 未試行 | ❌ scalar Rust のみ (DSP unit 無し) | ❌ |

**ESP32 / ESP32-S3** (Xtensa LX6 / LX7) のみが実音源で end-to-end
回しているターゲット。それ以外についてはライブラリは **build はできる**
(`cargo build -p mfsk-ffi-ft8 --release --no-default-features
--features embedded-fixed-point,embedded-runtime --target <T>` で
試せる) が、FFT extern Rust シンボルは自前で供給する必要がある。
具体的な RP2040 / RP2350 / Cortex-M shim は将来作業として追跡。

`embedded-poc/embedded-shared/src/esp_dsp_fft.rs` がコピー元の
worked example。

## 組込利用向け Cargo feature

デフォルトは `std`、`parallel`、`fft-rustfft` を含む — これらを切って
組込ベースラインを選ぶ:

```toml
[dependencies]
mfsk-core = { version = "0.8", default-features = false, features = [
    "alloc",            # Vec / Box / String — decode 必須
    "ft8",              # FT8 protocol glue
    "fft-extern",       # 呼び出し側が FFT バックエンドを供給
    "fixed-point",      # u16 spec + i16 DFT + Q11i16 LLR + 整数 NMS BP
    # オプション:
    # "profile-coarse", # stage-2 sub-stage timing 常時出力
] }
```

**`fixed-point` は `nstep-half` を含意する** (0.6.4 以降):
組込ビルドは NSTEP = NSPS/2 = 960 サンプル/spectrogram カラム、
host デフォルトは WSJT-X 忠実な NSPS/4 = 480。実運用では 2 つの
feature は常に同時に有効化されてきており、結合することで host
fixed-point ビルドが組込時間グリッドを忠実にシミュレートする。

Stage-3 感度は Cargo feature ではなく
`process_candidates_into` の runtime パラメータ (`q_thresh: u32`)。
[`mfsk_core::ft8::decode_block::DEFAULT_Q_THRESH`] は 12 — 現在
出荷しているすべてのターゲットで full recall。下げ (q=6) ると
探索を広げるが wall-clock cost が増える、上げ (q=14) るとターゲット
あたり弱い decode を 1 件落とすかわりに `qso3_busy` で 0–78 ms
節約。本番では `q_thresh` はデフォルトのまま。

Feature リファレンス:

| Feature | 変える内容 | 用途 |
|---|---|---|
| `std` | `std::env`、`std::time::Instant` を取り込む。rustfft からは分離。 | std がある esp-idf-svc 系ターゲット。bare-metal では任意。 |
| `alloc` | `extern crate alloc` + Vec / Box。 | 全 decode パス。 |
| `fft-extern` | `mfsk_core_make_default_fft_planner` extern fn (i16 用 `_planner16` も) 経由の FFT バックエンド。 | 任意の組込ターゲット。 |
| `fft-rustfft` | rustfft を FFT バックエンドに。 | Host 専用。 |
| `fixed-point` | 組込整数パイプライン: u16 spectrogram + i16 内部 DFT + Q11i16 LLR + 整数 NMS BP。`nstep-half` を含意。(0.5.x は `Q3i8` だったが、host fixed-point + rustfft で `qso3_busy.wav` の recall が f32 16/18 → Q3i8 9/18 と落ちる LLR 解像度律速が判明、0.6.2 で `Q11i16` に拡張。`Q3i8` 型は比較経路用に `engine::scalar` に残置。) | 任意の組込ターゲット — host f32 に近い recall (1/2048 LSB)、PSRAM 帯域半減、~12 KB BP scratch (Q11i16、0.6.2 以降)。 |
| `nstep-half` | spectrogram カラムレートを NSTEP = NSPS/2 (WSJT-X 忠実な NSPS/4 でなく)。 | `fixed-point` で自動有効。host ビルドで組込パスを明示的に simulate する以外では独立に enable しない。 |
| `parallel` | Rayon 並列 candidate 処理。 | Host 専用。組込では常に off (`std::thread` 無し)。 |
| `profile-coarse` | coarse_sync sub-stage timing を常時 stderr 出力。 | 診断専用。 |

## FFT extern Rust 契約

`mfsk_core::engine::fft::FftPlanner` (および i16 パス用
`FftPlanner16`) が decode パスの FFT trait。`fft-extern` 配下では
リンクされたバイナリが 2 つの `extern "Rust"` factory 関数を提供
することを要求する:

<!-- 非コンパイル: `MyEspDspPlanner`/`MyEspDspPlanner16` はダウンストリーム
     バイナリが用意する具体型のプレースホルダーで、この例はライブラリ
     クレート自身では実行できないバイナリ側の弱リンク契約を示す
     ものである — `engine::fft::default_planner` 自身の doc comment
     (この例の短縮版) も同じ理由で `ignore` にしている。 -->

```rust,ignore
#[unsafe(no_mangle)]
pub extern "Rust" fn mfsk_core_make_default_fft_planner()
    -> Box<dyn mfsk_core::engine::fft::FftPlanner>
{
    Box::new(MyEspDspPlanner::new())
}

#[unsafe(no_mangle)]
pub extern "Rust" fn mfsk_core_make_default_fft_planner16()
    -> Box<dyn mfsk_core::engine::fft::FftPlanner16>
{
    Box::new(MyEspDspPlanner16::new())
}
```

`embedded-poc/embedded-shared/src/esp_dsp_fft.rs` は esp-dsp の
Xtensa ASM カーネル (`dsps_fft2r_fc32_ae32` + i16 用
`dsps_fft2r_sc16_ae32`) にブリッジする実装例。RP2040 / Cortex-M
実装は CMSIS-DSP に同様にブリッジする。

### 除去済み: i16 × Q15 dot product extern (0.8.0, issue #162)

per-symbol DFT (BASIS パス) には別途 `mfsk_core_dot_q15_i32`
extern シンボルが必須だった。0.6.4 (Phase 1.7.7-Stick) 以降この
extern は既に decoder から使われておらず — per-symbol DFT は
in-tree の Goertzel recursion を通っていた (次節参照) — 0.8.0 で
このシンボル自体、`mfsk_core::core::dotprod` モジュール、BASIS
fill path 全体を削除した。新規統合ではここに何も実装する必要が
ない。

## per-symbol DFT: Goertzel

FT8 per-symbol DFT は `Σ x[n] · exp(-jωn)` を 8 トーン周波数それぞれ
について NSPS = 1920 サンプル全域で評価する。各 candidate の `cs`
matrix で全 (symbol, tone) ペア (79 sym × 8 tone = 632 DFT/candidate、
~15 candidate/slot ⇒ ~9.5k DFT/slot) について。実装は
`mfsk-core/src/ft8/decode_block/fill_symbol_spectra.rs` にある:

### Goertzel — `fill_symbol_spectra_goertzel`

一般化 Goertzel recursion: (sym, tone) ごとに 2-tap IIR、3 個の
f32 状態をスタック上に持ち、return 時に破棄。**呼び出し側
scratch ゼロ**、内部 DRAM 静的バッファゼロ、extern シンボル要求
ゼロ。0.6.4 以降、組込呼び出し側の唯一のパス。置き換えられた
legacy BASIS (Q15 sin/cos dot-product) fill path は 0.8.0 で
完全に削除された (issue #162)。

性能トリック: ループを **sample-outer / tone-inner** で並べることで
8 個の per-tone recursion (FT8 トーン数分) が FPU パイプラインを通る
独立した 8 本の dependent chain として走る。LLVM が定数境界の
`NTONES = 8` 内側ループを unroll し、Xtensa FPU が per-chain
latency をほぼ全部並列吸収する。結果: stage-3 cost は旧 BASIS asm
dot product と一致 (S3 `qso3_busy.wav` 上 ~1.4 s)、**scratch ゼロ +
旧 BASIS 比 SNR +0.16..+0.63 dB 改善** (f32 Goertzel は Q15 BASIS
より精度が高かった)。

何故 BASIS を引退させたか: 事前計算済 Q15 sin/cos テーブル
(`BASIS_RE` / `BASIS_IM`、各 `NTONES × NSPS = 15 360` i16 entry ≈
30 KB) を ASM dot product が定格 throughput を出すには fast 内部
SRAM (DRAM) に置く必要があった — 内部 DRAM 30 KB / 軸 × 2 軸 ×
2 core = **120 KB の内部 DRAM** がまさに M5StickS3 Qso モードの
双方向 I2S DMA descriptor が割当てたい量。ボードの空き連続内部
チャンクが両者を満たせず、Qso モード起動が `i2s_alloc_dma_desc:
allocate DMA buffer failed` で失敗した。Goertzel は perf を落とさず
120 KB を解放し、0.8.0 で BASIS コードと `basis_re`/`basis_im`
scratch 引数そのものを削除して仕上げた — 新規統合では scratch の
配置を一切考える必要がない。

## Q-format クイックリファレンス

| Stage | Format | Range | File |
|---|---|---|---|
| Spectrogram cell | u16 (mag²) | `>> FP_SPEC_SHIFT (12)`、0.6.4 以降飽和 | `ft8::decode_block::spectrogram::Spectrogram` |
| Symbol cs | `Cmplx<f32>` (デフォルト) または `Cmplx<Q14i16>` (`fixed-point`) | f32 無制限、Q14 ±2 | `engine::scalar::Cmplx` (`num_complex::Complex` の type alias) |
| LLR | f32 (host) または **Q11i16** (`fixed-point`、0.6.2 以降 — 0.5.x は `Q3i8`。解像度律速の recall 天井を解消するため拡張) | f32 無制限、Q11i16 ±16 (~1/2048 LSB) (Q3i8 ±16 (~1/8 LSB) は `engine::scalar` に比較経路用として残置) | `engine::scalar::LlrScalar` |
| BP messages | T (LLR と同じ) | — | `fec::ldpc::bp::bp_decode_generic_nms_with_scratch` |

## C / C++ / 非 Rust ESP-IDF プロジェクトからの利用 (`mfsk-ffi-ft8`)

[`mfsk-ffi-ft8`](https://github.com/jl1nie/mfsk-core/tree/main/mfsk-ffi-ft8)
は FT8 ブロックデコーダの小さな C ABI を export する。非 Rust
ESP-IDF (or RP2040 / Cortex-M) プロジェクトから組込 FT8 デコーダ
を呼ぶ推奨方法。

`embedded-fixed-point` feature では `no_std + alloc` なので、
生成される `libmfsk_ft8.a` は Rust の `std` ランタイムを持ち込まず、
2 種類の libc レイヤを混ぜる toolchain weirdness 無く C から
ドロップイン link 可能。

**ESP32 Core2 上で end-to-end 検証済** (本来は開発専用の独立
コンピュート bench `embedded-poc/m5stack-core2/` で実施 — この
bench は #61 Phase 3 (0.6.3) で retired、wav_sim 経路は production-
app 形態の `embedded-poc/m5stack-core2-app/` に統合された): 別経路の `ffi_smoke_one` が
`mfsk_ft8_decode_i16` (C ABI) を direct-Rust `decode_one` パスと
同じ baked WAV に対し呼んで同一 recall — qso1 (3 / 3)、qso2
(5 / 5)、**qso3 busy band (7 / 7)**。caller-managed BASIS scratch
を内部 RAM に置くと FFI パスが内部ヒープ alloc 比 ~2.6 倍速
(qso3 3.74 s vs 9.57 s)。Goertzel 化 (0.6.4+) 後は **scratch 引数
すら不要** で同 recall。

### API 概観

cbindgen 生成ヘッダ — `mfsk-ffi-ft8/include/mfsk_ft8.h`、ビルド毎
に再生成。フル surface:

```c
typedef struct MfskResult {
    char     text[40];   // NUL 終端の unpack 済メッセージ
    float    freq_hz;    // carrier
    float    dt_sec;     // slot 開始基準の時間 offset
    float    snr_db;     // JTDX 絶対値に対し xsnr2_db_simple で
                         // 校正済 (実機で ±3 dB 以内)
    uint32_t hard_errors;
    uint8_t  pass;       // staircase stage (0=fast Bp、1=full Bp…)
} MfskResult;

typedef struct MfskResultList {
    MfskResult *items;
    size_t      len;
    size_t      _capacity;  // private
} MfskResultList;

// Opaque デコードチューニングハンドル — mfsk_ft8_options_new で構築、
// mfsk_ft8_options_free で解放。NULL は常に有効な options 引数
// (このクレートの既定値: 200-3000 Hz、sync_min 1.0、max_cand 30、
// MFSK_DECODE_DEPTH_BP_ALL_OSD を使う)。
typedef struct MfskDecodeOptions MfskDecodeOptions;

MfskDecodeOptions *mfsk_ft8_options_new(
    float freq_min_hz, float freq_max_hz,     // typical 200, 3000
    float sync_min, int max_cand,             // typical 1.0, 30
    MfskDecodeDepth depth);                   // 1=BpAll、2=BpAllOsd
void mfsk_ft8_options_free(MfskDecodeOptions *opts);

// 組込のメインエントリ。呼び出し側 scratch は不要 — decoder は
// in-tree Goertzel パス (内部 DRAM scratch ゼロ) で per-symbol DFT
// を埋める。0.8.0 (issue #162) 以前はここで `basis_re`/`basis_im`
// scratch ポインタも受け取っていたが、削除済み BASIS fill path
// 用だったため除去した。0.8.0 (issue #205) 以前は 5 個のチューニング
// 引数を options 経由でなく位置引数で受け取っており、host ビルドは
// 別名の `mfsk_ft8_decode_i16_alloc` を export していた —
// pre-0.8.0 ヘッダでビルドしていた C 呼び出し側は両方の更新が必要。
MfskStatus mfsk_ft8_decode_i16(
    const int16_t *audio, size_t n_samples,   // 12 kHz, mono, ≥168 000
    const MfskDecodeOptions *options,         // NULL = 既定値
    MfskResultList *out);                     // callee が populate

void mfsk_ft8_result_list_free(MfskResultList *list);
```

### `mfsk_ft8_decode_i16` の呼び出し方

管理すべき scratch バッファは無い — そのまま呼ぶだけ:

```c
#include "mfsk_ft8.h"

MfskDecodeOptions *options = mfsk_ft8_options_new(
    200.0f, 3000.0f, 1.0f, 30, MFSK_DECODE_DEPTH_BP_ALL);

MfskResultList results = {0};
MfskStatus st = mfsk_ft8_decode_i16(audio, n_samples, options, &results);
// ... results を使う ...
mfsk_ft8_result_list_free(&results);
mfsk_ft8_options_free(options);
```

### Streaming capture: I2S / USB Audio → 12 kHz ring

`mfsk_ft8_decode_i16` は 15 秒の 12 kHz スロットを一度に取る。
実際の受信機はそうではなく、codec の動作レート (典型的に I2S や
USB Audio Class 1/2 から 16 / 24 / 48 kHz) で小さな DMA チャンク
を取る。`mfsk_ft8_stream_*` ファミリが両者の橋渡しを各 consumer に
再実装させない:

```c
typedef struct MfskFt8Stream MfskFt8Stream;

// 構築: 任意 src rate + ring 容量 (12 kHz サンプル数)。
// 標準 15 s スロットなら 180000 を渡す。
MfskFt8Stream *mfsk_ft8_stream_new(uint32_t src_rate_hz, size_t cap);
void           mfsk_ft8_stream_free(MfskFt8Stream *);

// DMA chunk を push。内部で 12 kHz に再サンプリング、ring に追加
// (満杯時は古いサンプルから上書き — rolling-window モデル)。
MfskStatus mfsk_ft8_stream_push_i16(MfskFt8Stream *,
                                    const int16_t *samples, size_t n);

// Snapshot: 最新 `cap` 個の 12 kHz サンプルを `out` にコピー。
// ring は変更しない — decode 成功後 _drain() を呼んで新音源用
// 領域を空ける。
size_t mfsk_ft8_stream_buffered_samples(const MfskFt8Stream *);
size_t mfsk_ft8_stream_peek_latest(const MfskFt8Stream *,
                                   int16_t *out, size_t cap);
void   mfsk_ft8_stream_drain(MfskFt8Stream *, size_t n);
void   mfsk_ft8_stream_clear(MfskFt8Stream *);
```

内部: Q32 fixed-point linear resampler (carry-over 状態あり、
チャンク境界 glitch なし) + 固定容量 i16 ring。純粋スカラ演算 —
FFT なし、DSP backend なし。`host` ビルドと `embedded-fixed-point`
ビルドの両方で利用可能。

**典型的な RTOS 配線** (capture と decode を別タスクで):

```c
// 一回だけのセットアップ
static MfskFt8Stream *g_stream;
static MfskDecodeOptions *g_options;
static int16_t g_slot[180000];          // 360 KB; PSRAM 可

void rx_init(void) {
    g_stream = mfsk_ft8_stream_new(/*src*/16000, /*cap*/180000);
    g_options = mfsk_ft8_options_new(200.0f, 3000.0f, 1.0f, 30,
                                      MFSK_DECODE_DEPTH_BP_ALL);
}

// Capture タスク: I2S DMA コールバック
void on_i2s_chunk(const int16_t *samples, size_t n) {
    mfsk_ft8_stream_push_i16(g_stream, samples, n);
}

// Decode タスク: UTC スロット境界毎 15 秒間隔で発火
void on_slot_boundary(void) {
    if (mfsk_ft8_stream_buffered_samples(g_stream) < 168000) return;
    size_t n = mfsk_ft8_stream_peek_latest(g_stream, g_slot, 180000);

    MfskResultList results = {0};
    mfsk_ft8_decode_i16(g_slot, n,        // 180000 ではなく n。ring が
                                          // 満杯でなければ peek は短く
                                          // 返してくる。
                        g_options, &results);
    // ... results を使った後 ...
    mfsk_ft8_result_list_free(&results);
    mfsk_ft8_stream_drain(g_stream, 180000);  // 次スロット用に空ける
}
```

**スロット境界アライメント。** UTC アライメントは ±2 s 以内で十分
— `decode_block` の coarse-sync ステージが Costas-array サーチ
で内部に吸収する。Wi-Fi ボードでは NTP が最も簡単、オフライン /
モバイル用途では GPS PPS、スタンドアロン bench なら任意の参照時刻
から正確に 15 秒間隔で free-run しても 1 時間で 50 ppm 以内の
タイマ安定度があれば decode できる。

**Resampler 品質。** 線形補間 — 演算の単純さ (i64 mul / shift、
FPU 無し MCU でも余裕、LX6/LX7 で ASM throughput に追随) のため
選択。16 → 12 kHz や 48 → 12 kHz の典型比率と実音源パスバンド
(200–3000 Hz) では混入歪 ~–55 dBc、FT8 LDPC の動作 SNR より遥か
に下。FT8 以外の用途で透明な fidelity が必要なら、ring の前に
polyphase FIR を入れる。

### Build フラグ

#### Host (`libmfsk_ft8.so` / `libmfsk_ft8.a`、デスクトップテスト用)

```sh
cargo build -p mfsk-ffi-ft8 --release
# → target/release/libmfsk_ft8.{so,a}
# → mfsk-ffi-ft8/include/mfsk_ft8.h (cbindgen 生成)
```

デフォルト feature は `mfsk-core/std + ft8 + fft-rustfft` を引き
込む。生成された `.so` を link する C smoke test:
`mfsk-ffi-ft8/tests/c_smoke/smoke.c`

```sh
gcc -O2 -I mfsk-ffi-ft8/include \
    mfsk-ffi-ft8/tests/c_smoke/smoke.c \
    -L target/release -lmfsk_ft8 -lm -lpthread -ldl \
    -Wl,-rpath,$PWD/target/release \
    -o /tmp/mfsk_smoke
/tmp/mfsk_smoke embedded-poc/assets/qso3_busy.wav
```

#### 組込 (Xtensa ESP32、`libmfsk_ft8.a` を ESP-IDF link 用)

```sh
source ~/export-esp.sh                     # Xtensa toolchain
RUSTFLAGS="-C panic=abort" \
cargo build -p mfsk-ffi-ft8 --release \
    --no-default-features \
    --features embedded-fixed-point,embedded-runtime \
    --target xtensa-esp32-espidf            # or -esp32s3-espidf
# → target/xtensa-esp32-espidf/release/libmfsk_ft8.a
```

`-C panic=abort` 必須 — Rust unwinding panic は `std` を要求する
が、組込は `panic = "abort"` 一択。ESP-IDF プロジェクトは典型的に
これを `.cargo/config.toml` で設定する:

```toml
[target.xtensa-esp32-espidf]
rustflags = ["-C", "link-arg=-nostartfiles", "-C", "panic=abort"]
```

#### Feature リファレンス

| Feature | デフォルト | 目的 |
|---|---|---|
| `host` | ✓ | Host ビルド — `mfsk-core/std + ft8 + fft-rustfft` を引く。host ネイティブ f32 パスを backend とする `mfsk_ft8_decode_i16` を export。0.8.0 (issue #205) 以前はこの feature が別名の `mfsk_ft8_decode_i16_alloc` を export していたが、組込ビルドと同名のシンボルに統合 (backend は host ネイティブのまま)。 |
| `embedded-fixed-point` | — | `no_std + alloc`。`mfsk-core/fft-extern + fixed-point` (`nstep-half` を含意) を引く。同じ `mfsk_ft8_decode_i16` シンボルを export、backend は fixed-point パス。リンカが `mfsk_core_make_default_fft_planner` + `_planner16` を解決する必要あり (esp-dsp にブリッジする小さな Rust shim 経由が典型)。 |
| `embedded-runtime` | — | デフォルト `#[panic_handler]` (libc `abort` 呼ぶ) + `#[global_allocator]` (libc `malloc`/`free`) を提供。自己完結型 `staticlib` 用; 同一 image 内で別 Rust runtime を積む場合は off。 |

### ESP-IDF (CMake) プロジェクトへのリンク方法

```text
your-app/                          # esp-idf プロジェクトルート
├── main/main.c                    # mfsk_ft8_decode_i16(...) を呼ぶ
├── components/mfsk_ft8/
│   ├── CMakeLists.txt             # IMPORTED static-lib component
│   ├── include/mfsk_ft8.h         # mfsk-ffi-ft8 ビルドから
│   └── lib/libmfsk_ft8.a          # mfsk-ffi-ft8 ビルドから
└── shim/                          # 小さな Rust crate (esp-dsp ブリッジ)
    ├── Cargo.toml                 # mfsk-ffi-ft8 に依存
    ├── .cargo/config.toml         # target = xtensa-esp32-espidf, panic=abort
    └── src/lib.rs                 # mfsk_core_make_default_fft_planner[16] を提供
```

`shim/` Rust crate が必要なのは mfsk-core の FFT-extern 契約が
`extern "Rust"` シンボル (`extern "C"` とは ABI が違う) を使う
ため。純粋 C コンパイル単位ではこれを満たせない。shim は ~50 行
の Rust + `embedded-poc/embedded-shared/src/esp_dsp_fft.rs` の
vendored コピー。

`components/mfsk_ft8/CMakeLists.txt` 最小例:

```cmake
idf_component_register(INCLUDE_DIRS "include"
                       REQUIRES espressif__esp-dsp)
add_library(mfsk_ft8_rust STATIC IMPORTED)
set_target_properties(mfsk_ft8_rust PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/lib/libmfsk_ft8.a)
target_link_libraries(${COMPONENT_LIB} INTERFACE mfsk_ft8_rust)
```

ワークする骨組は
[`embedded-poc/idf-component/`](https://github.com/jl1nie/mfsk-core/tree/main/embedded-poc/idf-component)。

## 出荷していないもの

mfsk-core はデコード / エンコードパイプラインで止まる。以下は
ハードウェアバラつきで汎用インタフェースが役に立たないので
**意図的にスコープ外**:

- 音声キャプチャ (I2S、マイクゲイン、サンプリングレートクロック復元)
- ディスプレイ / UI (TFT、OLED)
- ネットワーキング (Wi-Fi、BLE、MQTT)
- RTOS タスク配線
- 時刻 / クロック同期 (NTP、GPS)
- 永続ストレージ / 設定

`embedded-poc/` の crate 群が 2 つの特定ボード向けに (esp-idf-svc
で) この全部を配線する一例を示す:

- `embedded-poc/m5stack-s3-app/` — M5StickS3 FT8 controller
  (ES8311 音響 mic、IC-705 への BLE CI-V、LCD UI、QSO FSM、
  オプション WiFi UDP log)。本番、日常使用ターゲット。
- `embedded-poc/m5stack-core2-app/` — Core2 (LX6) 兄弟、デコーダ
  を LCD 配線した baked `wav_sim` 音源ループに対し走らせる。
  外部 I/O は保留。`mfsk-app-shared` API を LX6 上で交差検証
  するために存在。

いずれも **example** であり、改変なしに fork することを期待した
maintained アプリではない。テンプレートとして参照し、使えるもの
だけコピー。

## 性能ベンチマーク

3 本のオンエア録音を WAV asset として組込 (12 kHz / mono / i16
PCM、各 ≈ 360 KB)、`rx-wavsim` ストリーミング bench がリアルタイム
ペースでキューパイプラインに流し WAV 完了 notify ごとに 1 スロット
ずつデコード。**post-SlotEnd** = SlotEnd notify から「デコード
完了」までの wall-clock — つまりユーザ知覚 RX latency
(stage 2 はオーディオキャプチャの尾部で走り、この予算からは隠れる。
下の「Streaming RX pipeline architecture」参照)。

`q_thresh = 12` (本番デフォルト、full recall)。

`qso3_busy.wav` は **WSJT-X 公式配布 FT8 リファレンス録音**
(`samples/FT8/210703_133430.wav`、混雑 7 局スロット;
`cmp` で 2026-05-04 bit 一致確認)。`qso1` / `qso2` はオンエア
オリジナル録音 — 幅としては有用だが正式リファレンスではない。

下の S3 LX7 数値は 0.6.3 Q11i16 ship sweep の計測値
(`embedded-poc/m5stack-s3/logs/` に 2026-05-09 開発実行として archive
されていたが raw log file は repo に保存されておらず、0.6.2 → 0.6.3
の Q3i8 → Q11i16 移行 phase log
`logs/s3_phaseA..C_q3i8_2026-05-04.log` 等のみ残置)。0.6.4 Goertzel
は同じ wall-clock を保ちつつ同じ decode で +0.16..+0.63 dB SNR を
追加。0.6.5 firmware での再測定が、0.6.3 OSD tightening が embedded
側数値を動かしていないかの確認には適切。

| WAV | S3 LX7 post-SlotEnd | decoded |
|---|---:|---:|
| qso1 (中域)                            | **1.10 s** | 3 |
| qso2 (中域)                            | **1.68 s** | 4 |
| **qso3 busy band (WSJT-X reference)**  | **1.19 s** | **7 / 18 JTDX** |

### WSJT-X リファレンスでの host wide-band との比較

同じ `qso3_busy.wav` で `decode_frame` (host wide-band: rustfft、
`DecodeDepth::FULL`、max_cand=200、OSD-3 fallback) と `decode_block`
(組込相当: 整数パイプライン、max_cand=15、q=12) を side-by-side
で走らせた結果:

| run | callsigns / 18 JTDX truth | wall-clock | hardware |
|---|---:|---:|---|
| host wide-band (`decode_frame DecodeDepth::FULL 200`) | **16 / 18** | ~140 ms | Ryzen デスクトップ |
| host fixed-point (= embedded, `decode_block` 15) | 7 / 18 | ~6 ms | Ryzen デスクトップ |
| **M5StickS3 LX7** (`decode_block`, 実機)  | 7 / 18 | **1.19 s** | post-SlotEnd, 240 MHz dual-core |
| **M5Stack Core2 LX6** (`decode_block`, 実機) | 7 / 18 | ~2.8 s | post-SlotEnd, 240 MHz dual-core |

組込パスが busy band で外す 11 callsign は host wide-band が走る
広い PASS1=200 サーチ + 反復減算 + OSD-3 fallback を要し、組込
予算ではスキップしている。host fixed-point (6 ms) と組込実機
(1.19 s / 2.8 s) の wall-clock 差は素の CPU 比 (Ryzen ~5 GHz × 16
core vs Xtensa 240 MHz × 2 core) — 両者が同一整数パイプライン
を走らせているのでアルゴリズム / パイプラインオーバーヘッドは無い。

#### なぜ組込パスで PASS1 を広げず / OSD を有効化しないか

実機 S3 LX7 上で WSJT-X リファレンス busy band に対しテスト
(`logs/s3_pass100_max30_2026-05-04.log`):

| config | qso3 post-SlotEnd | qso3 recall | total recall |
|---|---:|---:|---:|
| Bp/30/15 (ship)  | **~1.2 s** | 7/18 | 14/22 (phantom 込で 15) |
| Bp/100/30        | **~1.6 s** | 7/18 (不変) | +1 (qso1 の OH3NIV のみ) |
| DecodeDepth::FULL/200/100 (host 推定) | ~7 s | 7/18 (+1 で qso3 N1JFU) | 16/22 |

`PASS1=30 / max_cand=15` 維持を決めた非自明な 2 知見:

1. **qso3 busy band の recall は BP / OSD 努力でなく coarse_sync
   ランクで上限が決まる。** PASS1 30 → 100 + max_cand 15 → 30
   への拡大で qso3 callsign は何も増えない — 取り逃した signal
   は coarse_sync ランク 100 以下に全くない。WSJT-X wide-band パス
   の代名詞である反復減算が必要で、`decode_block` はそれを実装して
   いない。
2. **FT8 QSO turnaround 予算は post-SlotEnd ~2 s**、フル 15 s
   スロットではない。decode 後に UI が waterfall を描き、callsign
   list を更新、RPRT を render、次スロット TX を準備、そして —
   NTP 同期や GPS disciplined RTC が無いチップでは — decode され
   た signal の `dt_sec` の **中央値** から slot timing を推定し
   直す (素の平均は外れ値に弱い: 1 件の bogus-sync だが CRC-valid
   decode が slot phase をだいぶずらす; ESP32 の内部 RTC drift が
   大きいので frame アライメントはこの decoder 由来推定に追従する
   必要がある)。Bp/100/30 で qso3 では全部やる前に ~0.4 s しか
   残らず、次 TX 開始までキツすぎる。qso1 限定 +1 recall は
   ヘッドルーム喪失に見合わない。

つまり組込 `decode_block` は 2 s 予算に綺麗に収まる recall floor
で出荷している。これ以上を狙うには (a) 反復減算を組込パスに移植
(コスト未知 — `docs/notes/ROADMAP.md` の「Embedded fine_refine attempt
postmortem」参照) または (b) QSO turnaround に間に合わない遅着
「スポッターモード」decode を受け入れるかのどちらか。

`qso3_busy.wav` の per-stage 分解:

| stage | Core2 LX6 | S3 LX7 | 備考 |
|---|---:|---:|---|
| stage 1 (キャプチャ中インクリメンタル) | 15 s 中の演算 ≈ 1.0 s | 同 | キャプチャ CPU の ~6 % |
| stage 2 `coarse_sync_split_with_allsum` (キャプチャ中) | 0.65 s | 0.16 s | SlotEnd notify latency 下に隠れる |
| pass 2 `pass2_split` (post-SlotEnd) | 0.19 s | 0.12 s | dual-core、head/tail 分割 |
| stage 3 `stage3_split` (post-SlotEnd) | ≈ 2.5 s | 1.06 s | dual-core、**work-stealing** per-cand |

両チップをこのレンジに入れた wall-clock 改善 2 件:

1. **stage 2 がキャプチャの下に隠れる。** `stage1_inc` がペア 92
   を確定した瞬間 (SlotEnd の ≈ 200 ms 前) に `SpecBundle`
   (spec + per-half allsum) を `spec_q` キューに送るので、main は
   `coarse_sync_split_with_allsum` をオーディオキャプチャの尾部
   と並列に走らせる — post-SlotEnd 予算の中ではなく。
2. **stage 3 の work-stealing。** `dual_core::stage3_split` は
   candidate を head / tail に事前分割しない。PRO_CPU と APP_CPU
   が共有 `Vec<Option<RefinedCandidate>>` から
   `AtomicUsize::fetch_add(1)` で次 candidate を取るので、忙しい
   core が反対側に落ちた遅い / 失敗 candidate で stall しない。
   qso3 (15 cand 中 ~半数が失敗し 4 種類の LLR variant 全部走る)
   で per-cand BP wall-clock variance を吸収する。

## Streaming RX pipeline アーキテクチャ

Phase E 以降のパイプライン (`embedded-poc/embedded-shared/src/`
配線済) は **キューベース、per-slot 単一所有** — 共有可変状態
なし、notify-and-out-pointer 分離なし:

```text
wav_sim / I2S キャプチャ (PRO_CPU, prio 4)
  │
  │  ChunkMsg = Samples(Vec<i16>) | SlotEnd { wav_idx, total_samples }
  ▼
chunk_q (depth 4)
  │
  ▼
stage1_inc worker (APP_CPU, prio 3)
  │  内部: per-slot WorkerCtx { audio, spec, allsum_head/tail,
  │                              next_pair, … }
  │  ペア 92 着地と同時 (SlotEnd の ≈ 200 ms 前) に SpecBundle を
  │  送出し、main がキャプチャ尾部で stage 2 を開始できる
  │
  ├──▶ spec_q (depth 2): SpecBundle { spec, allsum_head, allsum_tail }
  └──▶ slot_q (depth 2): Slot { audio, wav_idx, inc_total_us }
       (SlotEnd ChunkMsg 後)
       │
       ▼
main / decode タスク (PRO_CPU, prio 6)
       │  spec_q recv → stage 2 (coarse_sync_split_with_allsum, dual-core)
       │  slot_q recv → pass 2 (refine_candidates, dual-core)
       │              → stage 3 (work-stealing per-cand, dual-core)
       ▼
DecodeResult[]
```

`dual_core` が stage 2 / pass 2 / stage 3 dispatch 用に別の
FreeRTOS Queue 群 (job キュー 1 本 + per-variant result キュー)
を export。所有権移譲はすべてキュー上の `Box::into_raw`
raw-pointer item 経由 — host の `mpsc::sync_channel` 相当の
semantics。

パイプライン不変条件:
- キャプチャタスクは 1 slot 分の Samples / SlotEnd を FIFO 順で
  送る。
- `stage1_inc` は 1 slot あたり SpecBundle を最大 1 回送出する
  (`next_pair == N_PAIRS` の初回、またはペア 92 が来なければ
  `finalize_slot` の fallback)。
- main は受信 FIFO 順で SpecBundle ↔ Slot をペアリング。
- main は `STAGE3_RESULT_Q` recv で return 前にブロックするので、
  worker 側 raw pointer (audio、cs scratch、work-stealing slot
  配列) は呼び出し中 worker のアクセスより長く生きる。

`embedded-poc/embedded-shared/src/pipeline.rs` (キュー helper +
`ChunkMsg` / `SpecBundle` / `Slot` 型) と
`embedded-poc/embedded-shared/src/dual_core.rs` (work-stealing
stage 3 dispatch + Job enum) 参照。

## バイナリフットプリント (Core2 リファレンス、`xtensa-esp32-elf-size -A`)

| Region | 0.5.x BASIS | 0.6.4 Goertzel | 中身 |
|---|---|---|---|
| **IRAM** (`.iram0.text` + `.iram0.vectors`) | **69 KB** | **69 KB** | esp-idf 割込ハンドラ、Wi-Fi/BT IRAM 常駐ルーチン |
| **DRAM** (`.dram0.data` + `.dram0.bss`) | **76 KB** | **~16 KB** | 内部 RAM 静的データ: spectrogram cache + esp-idf statics。BASIS scratch (60 KB) は 0.6.4 で除去。 |
| **Flash text** (`.flash.text`) | **448 KB** | **~448 KB** | App + esp-idf コード |
| **Flash rodata** (`.flash.rodata`) | **1.21 MB** | **1.21 MB** | 読み取り専用データ — **オフライン実音源 bench 用に 3 本の baked WAV (~1.08 MB) を含む** |
| **総 app バイナリ** | **~2.0 MB** | **~1.94 MB** | `espflash flash` 書込量 |

baked WAV asset (1.08 MB) と同梱 esp-idf ランタイムを引くと、
`mfsk-core` 本体 + M5Stack Core2 example glue は flash text に
おおよそ **150–200 KB** 貢献。IRAM/DRAM 合計値には esp-idf が
含まれており、ライブラリ本体は IRAM 要求なし、Phase 1.7.7 以降は
**内部 DRAM scratch 要求も無し**。スロットあたり working set
合計: ~120 KB cs Box × 1 + ~360 KB spectrogram (PSRAM) + ~12 KB
BP scratch (Q11i16、0.6.2 以降。0.5.x の Q3i8 期は ~6 KB だった)。素の ESP32 (PSRAM なし) では 320 KB SRAM で
spectrogram を回せない — 本番向け WAV 入力に対し組込パスは PSRAM
必須。

**BASIS 除去で空いた 120 KB の内部 DRAM** がまさに M5StickS3
Qso モードの双方向 I2S DMA に必要な量。この alloc が今は初回で
成功する。

## 次に読むべきもの

読者の意図別:

- **既存 FT8 コントローラを操作したい** →
  [`docs/reference/MANUAL_M5STICKS3.ja.md`](MANUAL_M5STICKS3.ja.md) (ビルド /
  flash / `cfg.toml` / `BootMode` サイクル / UI / QSO workflow /
  トラブルシュート)。
- **新しい MCU に `mfsk-core` を統合したい** →
  まず [FFT extern 契約](#fft-extern-rust-契約)、C から呼ぶなら
  続けて [`mfsk-ffi-ft8` C ABI](#c--c--非-rust-esp-idf-プロジェクトからの利用-mfsk-ffi-ft8)。
  [`embedded-poc/embedded-shared/src/esp_dsp_fft.rs`](https://github.com/jl1nie/mfsk-core/blob/main/embedded-poc/embedded-shared/src/esp_dsp_fft.rs)
  shim がコピー元の worked example。
- **組込 app のどれかにコントリビュートしたい** →
  [`embedded-poc/CLAUDE.md`](https://github.com/jl1nie/mfsk-core/blob/main/embedded-poc/CLAUDE.md)
  でクロスボードツールチェイン notes + LX6/LX7 比較表、それから
  ボード固有 gotcha のために per-crate `CLAUDE.md`。
- **組込ロードマップを追いたい** →
  [`docs/notes/ROADMAP.md`](../notes/ROADMAP.md) Phase B-Stick (M5StickS3 demo /
  音響 fallback) と Phase B-Core (M5Stack CoreS3 main UAC
  controller) セクション。
