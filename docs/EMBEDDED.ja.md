# 組み込みターゲット

`mfsk-core` は `no_std + alloc` 対応です。FT8 デコードパス
(`mfsk_core::ft8::decode_block`) は、呼び出し側が FFT と内積の
バックエンドを供給する条件で、~150 KB の RAM 環境でも動作します。
本文書は組み込みコール元に求められる契約と、ライブラリ側で**提供しない**
ものを説明します。

## アーキテクチャ: f32 と固定小数点が同一コードベースで共存

DSP / FEC パイプライン全体が **scalar trait** 化されており、
同じソースがホスト用 f32 パスと組み込み用整数パスの **どちらにも
コード重複なしでコンパイル**されます:

- [`core::scalar::SpecScalar`] — スペクトログラム / DFT 出力 scalar
  (ホスト: `f32`; 組み込み cs 格納: `Q14i16`)
- [`core::scalar::LlrScalar`] — wide-accumulator 型を含む LLR scalar
  (ホスト: `f32`; 組み込み BP: `Q11i16` + i32 wide)。0.6.2 で組込
  LLR scalar は `Q11i16` (i16, 1/2048 解像度) に統一。0.5.x で評価
  していた `Q3i8` (i8, 1/8) は実機 S3 の `qso3_busy.wav` で Q11i16
  が recall +1 件 (XE2X HA2NP RR73) かつ post-SlotEnd 1.341 s →
  1.191 s で高速だったためロールバック
- [`core::scalar::Cmplx<S>`] — `SpecScalar` 上の generic 複素数。
  `repr(C)` で `num_complex::Complex` とレイアウト互換。
- `compute_llr_generic<P, S, T>`, `compute_snr_db_generic<P, S>`,
  `bp_decode_generic_nms<P, T>` — すべて scalar 型を generic 引数に
  取り、`(P, S, T)` の組ごとに 1 monomorphisation。

`fixed-point` の Cargo feature
は **protocol glue がどの scalar 型を選ぶかを切り替える**だけで、
generic 本体は変わりません。組み込みポートはコードの 99 % を
ホストビルドと共有 — バグ修正・最適化は一回当てればどちらにも効きます。

### 現状で fixed-point 切替が wired up されている範囲

| コンポーネント | generic 対象 | fixed-point 切替 |
|---|---|---|
| LDPC BP NMS (`fec::ldpc::bp`) | `LlrScalar` | ✅ `fixed-point` |
| LLR 計算 (`core::llr`) | `SpecScalar` × `LlrScalar` | ✅ `fixed-point` |
| BP scratch pool (`BpScratch<P, T>`) | `LdpcParams` × `LlrScalar` | ✅ — FT8 LDPC(174,91) と FST4/uvpacket LDPC(240,101) 両方 |
| FT8 spectrogram + DFT (`ft8::decode_block`) | `SpecScalar` × `AudioSample` | ✅ `fixed-point` |
| **FT4 / WSPR / Q65 / JT9 / JT65** | (ホスト f32 のみ) | ❌ — これらは `decode_block` を経由していない |

つまり **trait インフラは protocol-agnostic だが、組み込みビルドで
整数パスに切り替わる protocol は現状 FT8 のみ**。
FT4 (次の有力候補。Costas/Gray/LDPC の部品を共有) を加えるのは
`decode_block` の形を FT4 のシンボル配置に合わせて移植する作業で、
trait 層に新規追加は不要です。

## 検証済みターゲット

| ターゲット | MCU | バックエンド | 状況 |
|---|---|---|---|
| **M5Stack Core2** | **ESP32-D0WD-V3** (Xtensa LX6, dual-core 240 MHz, 単発 f32 FPU, 16 MB flash, 約 4 MB PSRAM) — `espflash board-info` で確認: `Chip type: esp32 (revision v3.1)` / `Features: WiFi, BT, Dual Core, 240MHz`。**ESP32-S2 ではない** (S2 は LX7 single-core で BT 無し) し S3 でもない | esp-dsp ASM (`dsps_dotprod_s16_ae32`, `dsps_fft2r_*`) | リファレンス実音声ベンチ。下のベンチ + フットプリント節参照 |
| ESP32-S3 (DevKitC) | Xtensa LX7 + PIE SIMD | esp-dsp ASM | 旧リファレンス。同じ `fft-extern` 契約 |

### 他ターゲット — 検証状況 vs 構想

`fft-extern` + `dotprod-extern` 契約は他ターゲットへの portable な
**設計**で、`mfsk-ffi-ft8` は非 Xtensa MCU 向けにも clean に cross-build します:

| ターゲット | `cargo build` 通る | FFT/dotprod shim 提供 | 実機検証 |
|---|---|---|---|
| `xtensa-esp32-espidf` | ✅ | ✅ esp-dsp (Core2) | ✅ qso1/2/3 sweep |
| `xtensa-esp32s3-espidf` | ✅ | ✅ esp-dsp (S3 PoC) | ✅ 旧リファレンス |
| `thumbv8m.main-none-eabihf` (RP2350 Cortex-M33) | ✅ | ❌ 候補: pico-sdk-rs 経由で CMSIS-DSP | ❌ |
| `riscv32imac-unknown-none-elf` (RP2350 Hazard3) | ✅ | ❌ DSP ライブラリ無し。FFT は `microfft`、dot は scalar Rust | ❌ |
| `thumbv7em-none-eabihf` (Cortex-M4F / M7) | 未試行 | ❌ 候補: CMSIS-DSP の `arm_*_q15` | ❌ |
| `thumbv6m-none-eabi` (Cortex-M0+ / RP2040) | 未試行 | ❌ scalar Rust のみ (DSP ユニット無し) | ❌ |

**実機 end-to-end 動作確認は ESP32 / ESP32-S3 (Xtensa LX6 / LX7) のみ**。
他ターゲットは cross-build できる (試: `cargo build -p mfsk-ffi-ft8
--release --no-default-features --features embedded-fixed-point,
embedded-runtime --target <T>`) が、2 つの extern Rust シンボルを
ユーザ側で用意する必要あり。RP2040 / RP2350 / Cortex-M 用の具体的
shim は将来の作業です。

`embedded-poc/m5stack-core2/` が コピー元の見本です。

## 組み込み用 Cargo feature

デフォルトの `std`, `parallel`, `fft-rustfft` を切り、組み込み
ベースラインを選びます:

```toml
[dependencies]
mfsk-core = { version = "0.6", default-features = false, features = [
    "alloc",            # Vec / Box / String — decode で必須
    "ft8",              # FT8 protocol glue
    "fft-extern",       # FFT backend は呼び出し側提供
    "fixed-point",      # u16 spec + i16 DFT + Q11i16 LLR + i16 NMS BP
    # 任意:
    # "profile-coarse",            # stage-2 サブステージ常時計測
] }
```

Stage-3 の感度は Cargo feature ではなく `process_candidates_into` の
ランタイム引数 `q_thresh: u32` で渡します。
[`mfsk_core::ft8::decode_block::DEFAULT_Q_THRESH`] は 12 で、現状
出荷している全 target で full recall。Phase E + work-stealing 後の
LX6 (Core2) / LX7 (M5StickS3) 実機で 12 vs 14 を A/B 測定し
(`logs/core2_q_sweep_2026-05-04.log`、`logs/s3_q_sweep_2026-05-04.log`)、
relaxed q=14 は **qso3 でのみ 0–78 ms 削減**するが、各 chip で **qso3 の弱
信号 1 局を失う** (S3: W1DIG -15.5 dB, Core2: N1PJT -18 dB) — 12
で post-SlotEnd 1.5 s 切り (Core2) / 0.8 s 切り (S3) を達成済みなので、
recall を犠牲にする価値は無く q=12 デフォルト推奨。

Feature 一覧:

| Feature | 効果 | 用途 |
|---|---|---|
| `std` | `std::env`, `std::time::Instant` を入れる。rustfft とは分離。 | std を持つ esp-idf-svc 系ターゲット。bare-metal では任意 |
| `alloc` | `extern crate alloc` + Vec / Box | 全 decode パス |
| `fft-extern` | `mfsk_core_make_default_fft_planner` extern fn 経由で FFT backend | 組み込み全般 |
| `fft-rustfft` | rustfft を FFT backend | ホスト専用 |
| `fixed-point` | u16 spec + i16 内部 DFT + Q11i16 LLR + i16 NMS BP の組み込み整数パイプライン | どの組み込みでも。host f32 にほぼ recall を揃えつつ spec/DFT 側で PSRAM 帯域半減。0.5.x で評価していた `Q3i8` LLR は 0.6.2 で実機 S3 検証時に Q11i16 が +1 件 recall でしかも高速だったためロールバック (上記「アーキテクチャ」を参照) |
| `profile-coarse` | coarse_sync サブステージ計測を常時出力 | 診断専用 |

## 2 つの extern Rust 契約

### FFT backend

`mfsk_core::core::fft::FftPlanner` は decode パスの FFT trait です。
`fft-extern` 下では、バイナリ側に `extern "Rust"` factory を
要求します:

```rust
#[unsafe(no_mangle)]
pub extern "Rust" fn mfsk_core_make_default_fft_planner()
    -> Box<dyn mfsk_core::core::fft::FftPlanner>
{
    Box::new(MyEspDspPlanner::new())
}
```

`embedded-poc/m5stack-core2/src/esp_dsp_fft.rs` が esp-dsp Xtensa
ASM kernel (`dsps_fft2r_fc32_ae32` + i16 パスの
`dsps_fft2r_sc16_ae32`) へ橋渡しする実装例です。RP2040 / Cortex-M
は同じ要領で CMSIS-DSP に橋渡しできます。

### i16 × Q15 内積

`fft-extern + fixed-point` 下では、`ft8::decode_block` のシンボル
DFT が `mfsk_core::core::dotprod::dot_q15_i32` を呼びます。
標準は Rust スカラ実装ですが、別の extern symbol で上書き可能:

```rust
#[unsafe(no_mangle)]
pub unsafe extern "Rust" fn mfsk_core_dot_q15_i32(
    a: *const i16,
    b: *const i16,
    n: usize,
) -> i32 {
    // dsps_dotprod_s16_ae32 (LX6/LX7) や
    // arm_dot_prod_q15 (Cortex-M with CMSIS-DSP) などへ橋渡し
}
```

これが stage 3 の最内ループです。ターゲットネイティブの MAC ユニットへ
橋渡しすれば、cached-RAM ターゲットで大きく速くなります。LX6 +
esp-dsp で MAC 1 サイクルあたり約 2 mac/cycle。

## BASIS scratch

`fill_symbol_spectra_into` (および `decode_block_into` /
`process_candidates_into` / `refine_candidates_into` ラッパー群) は
基底ベクター用の i16 scratch を呼び出し側から受け取ります:

```rust
const SCRATCH_LEN: usize = mfsk_core::ft8::decode_block::BASIS_SCRATCH_LEN;
static mut BASIS_RE: [i16; SCRATCH_LEN] = [0; SCRATCH_LEN];
static mut BASIS_IM: [i16; SCRATCH_LEN] = [0; SCRATCH_LEN];
```

`BASIS_SCRATCH_LEN = NTONES × NSPS = 15 360` (約 30 KB / 軸、
2 軸合計 60 KB)。これは **dot-product 内ループの hot data** で、
esp-dsp の `dsps_dotprod_s16_ae32` ASM kernel が 2 MAC/cycle で
回るのは basis が**高速な内部 SRAM (DRAM)** に乗っているときのみ。
PSRAM 有効な Core2 では、PSRAM-resident な basis はキャッシュ越しで
1 項あたり 5–10 cycle のストールを発生させ、kernel が定格の ~30 % に
落ちます — **stage 3 wall-clock が文字通り 2〜3 倍**になります。
`static` 配列なら `.bss` に自動配置 (内部 DRAM)、heap で確保したい
場合は ESP-IDF の capability-flag アロケータを:

```c
int16_t *basis = heap_caps_malloc(BASIS_SCRATCH_LEN * sizeof(int16_t),
                                  MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
```

通常の `malloc` で 60 KB 要求すると ESP-IDF のデフォルトルーティングで
PSRAM に流れます。複数 decode 呼び出しを跨いで保持してください — slot
ごとに reset 不要。

`_into` 系 `pub fn` 群は、組み込みコール元が basis を decode 毎に
アロケートせず通せるよう用意されています。非 `_into` 版
(`decode_block` など) は標準ヒープに確保するため、PSRAM 有効な
組み込み環境では上記の slow path に落ちます — ホストでは問題ないが
組み込みで足を撃ちやすい。

## Q-format 早見表

| ステージ | 形式 | レンジ | ファイル |
|---|---|---|---|
| spectrogram cell | u16 (mag²) | `>> FP_SPEC_SHIFT (12)` | `ft8::decode_block::Spectrogram` |
| DFT 基底 | Q15 i16 (cos, sin) | ±2¹⁵ ≈ ±1.0 | `fill_symbol_spectra_into` |
| シンボル cs | `Cmplx<f32>` (デフォルト) または `Cmplx<Q14i16>` (manual via `core::scalar`) | f32 制限なし、Q14 ±2 | `core::scalar::Cmplx` |
| LLR | f32 (ホスト) または Q11i16 (`fixed-point`) | f32 制限なし、Q11 ±16, 1/2048 LSB | `core::scalar::LlrScalar` |
| BP メッセージ | T (LLR と同じ) | — | `fec::ldpc::bp::bp_decode_generic_nms_with_scratch` |

## C / C++ / 非 Rust ESP-IDF プロジェクトから使う (`mfsk-ffi-ft8`)

[`mfsk-ffi-ft8`](https://github.com/jl1nie/mfsk-core/tree/main/mfsk-ffi-ft8)
は FT8 ブロックデコーダ部分の小さな C ABI です。組み込み (ESP-IDF /
RP2040 / Cortex-M) C/C++ プロジェクトから FT8 デコードを使う
推奨方法。

`embedded-fixed-point` feature 下で `no_std + alloc` 構成のため、
出力される `libmfsk_ft8.a` は Rust の `std` ランタイムを抱え込まず、
C 側から libc 二重リンク等の問題なく drop-in リンク可能。

**ESP32 Core2 で end-to-end 動作検証済み** (m5stack-core2 例):
別途 `ffi_smoke_one` パスが `mfsk_ft8_decode_i16` (C ABI) を同じ
ベイク済み WAV に対して呼んだ結果、直接 Rust の `decode_one` パスと
完全一致 — qso1 (3/3)、qso2 (5/5)、**qso3 busy band (7/7)**。
caller-managed BASIS scratch を内部 RAM に置けば、同じ FFI 呼び出しを
内部 heap-alloc で行ったケースの **約 2.6 倍速い** (qso3 で
3.74 s vs 9.57 s)、直接 Rust process_candidates_into パスとも 5 % 以内。
ログ:
- `embedded-poc/m5stack-core2/logs/ffi_into_2026-05-02.log`
  (推奨 caller-scratch)
- `embedded-poc/m5stack-core2/logs/ffi_smoke_2026-05-02.log`
  (heap-alloc 比較用)

### API 概要

cbindgen 生成 header — `mfsk-ffi-ft8/include/mfsk_ft8.h`、ビルド毎に
再生成。全 surface:

```c
typedef struct MfskFt8Result {
    char     text[40];   // NUL-終端 unpack メッセージ
    float    freq_hz;    // キャリア周波数
    float    dt_sec;     // スロット先頭からの時間オフセット
    float    snr_db;     // xsnr2_db_simple で校正済み (実機で
                         // JTDX 絶対値と ±3 dB 以内)
    uint32_t hard_errors;
    uint8_t  pass;       // staircase ステージ (0=fast Bp, 1=full Bp,…)
} MfskFt8Result;

typedef struct MfskFt8ResultList {
    MfskFt8Result *items;
    size_t         len;
    size_t         _capacity;  // 内部使用
} MfskFt8ResultList;

// プライマリ decode 関数の所要 scratch 長 (i16 個数)。
// 同じ長さの 2 配列を呼び出し側で確保。
size_t mfsk_ft8_basis_scratch_len(void);

// PRIMARY 組み込みエントリ。caller-managed scratch — dot-product ASM
// kernel をピーク速度で回すには、必ず内部 RAM に置く (PSRAM 不可)。
MfskFt8Status mfsk_ft8_decode_i16(
    const int16_t *audio, size_t n_samples,   // 12 kHz mono、≥168 000
    float freq_min_hz, float freq_max_hz,     // 典型 200, 3000
    float sync_min, int max_cand,             // 典型 1.0, 30
    MfskFt8Depth depth,                       // 0=Bp, 1=BpAll, 2=BpAllOsd
    int16_t *basis_re, int16_t *basis_im,     // scratch
    MfskFt8ResultList *out);                  // 結果

// HOST 専用簡便版 — basis を内部 heap-alloc。組み込みビルドでは
// 意図的に提供しない (下記「caller-supply scratch の理由」参照)。
#ifdef MFSK_FT8_HOST  // default `host` feature 有効時
MfskFt8Status mfsk_ft8_decode_i16_alloc(
    const int16_t *audio, size_t n_samples,
    float freq_min_hz, float freq_max_hz,
    float sync_min, int max_cand,
    MfskFt8Depth depth,
    MfskFt8ResultList *out);
#endif

void mfsk_ft8_result_list_free(MfskFt8ResultList *list);
```

### Caller-managed BASIS scratch

C ABI は Rust 側 `_into` 系をミラーした形 — `mfsk_ft8_decode_i16`
は basis を引数で受け取る方式で、内部 `malloc` を隠しません。理由は
[BASIS scratch](#basis-scratch) 節で説明した通り (PSRAM 配置で
ASM kernel が崩れる)。FFI 側でも同じ理由で「内部 `malloc` する
簡便ラッパー」は意図的に提供しません — C 呼び出し側からは戻り先
heap が読めないため。呼び出し側がポリシーを決めます:

```c
// 一番シンプルで正しいパターン: static .bss 配列 — 内部 DRAM に
// 自動配置され、プロセス寿命中保持される。
#include "mfsk_ft8.h"
static int16_t basis_re[15360];   // = mfsk_ft8_basis_scratch_len()
static int16_t basis_im[15360];

MfskFt8ResultList results = {0};
MfskFt8Status st = mfsk_ft8_decode_i16(
    audio, n_samples,
    200.0f, 3000.0f, 1.0f, 30,
    MFSK_FT8_DEPTH_BP_ALL,
    basis_re, basis_im,
    &results);
```

### Build flag

#### Host (`libmfsk_ft8.so` / `libmfsk_ft8.a` ホストテスト用)

```sh
cargo build -p mfsk-ffi-ft8 --release
# → target/release/libmfsk_ft8.{so,a}
# → mfsk-ffi-ft8/include/mfsk_ft8.h (cbindgen 生成)
```

デフォルト feature で `mfsk-core/std + ft8 + fft-rustfft` が入る。
`.so` を C テストから link する例が
`mfsk-ffi-ft8/tests/c_smoke/smoke.c`:

```sh
gcc -O2 -I mfsk-ffi-ft8/include \
    mfsk-ffi-ft8/tests/c_smoke/smoke.c \
    -L target/release -lmfsk_ft8 -lm -lpthread -ldl \
    -Wl,-rpath,$PWD/target/release \
    -o /tmp/mfsk_smoke
/tmp/mfsk_smoke embedded-poc/m5stack-core2/assets/qso3_busy.wav
```

#### 組み込み (Xtensa ESP32, `libmfsk_ft8.a` を ESP-IDF link)

```sh
source ~/export-esp.sh                     # Xtensa toolchain
RUSTFLAGS="-C panic=abort" \
cargo build -p mfsk-ffi-ft8 --release \
    --no-default-features \
    --features embedded-fixed-point,embedded-runtime \
    --target xtensa-esp32-espidf
# → target/xtensa-esp32-espidf/release/libmfsk_ft8.a
```

`-C panic=abort` 必須 — Rust unwinding panic は `std` を要求し、
組み込みでは `panic = "abort"` 一択。ESP-IDF プロジェクトでは通常
`.cargo/config.toml` に書く:

```toml
[target.xtensa-esp32-espidf]
rustflags = ["-C", "link-arg=-nostartfiles", "-C", "panic=abort"]
```

#### Feature 一覧

| Feature | Default | 用途 |
|---|---|---|
| `host` | ✓ | ホストビルド — `mfsk-core/std + ft8 + fft-rustfft`。`mfsk_ft8_decode_i16` (caller scratch) と `mfsk_ft8_decode_i16_alloc` (heap 簡便) の**両方**を export |
| `embedded-fixed-point` | — | `no_std + alloc`。`mfsk-core/fft-extern + fixed-point`。**`mfsk_ft8_decode_i16` のみ** export — heap-alloc 簡便版は意図的に除外 (上記参照)。`mfsk_core_make_default_fft_planner_*` と `mfsk_core_dot_q15_i32` を linker が解決する必要 (典型的には小さな Rust shim が esp-dsp に橋渡し) |
| `embedded-runtime` | — | crate 内に default `#[panic_handler]` (libc `abort` 呼び) + `#[global_allocator]` (libc `malloc`/`free`) を提供。自己完結 `staticlib` 用。同じイメージに別の Rust ランタイムを重ねるときは OFF |

### ESP-IDF (CMake) プロジェクトへの組み込み

```
your-app/                          # esp-idf project ルート
├── main/main.c                    # mfsk_ft8_decode_i16(...) を呼ぶ
├── components/mfsk_ft8/
│   ├── CMakeLists.txt             # IMPORTED static-lib コンポーネント
│   ├── include/mfsk_ft8.h         # mfsk-ffi-ft8 ビルド成果物
│   └── lib/libmfsk_ft8.a          # mfsk-ffi-ft8 ビルド成果物
└── shim/                          # 小さな Rust crate (esp-dsp bridge)
    ├── Cargo.toml                 # mfsk-ffi-ft8 に依存
    ├── .cargo/config.toml         # target = xtensa-esp32-espidf, panic=abort
    └── src/lib.rs                 # mfsk_core_make_default_fft_planner と
                                   # mfsk_core_dot_q15_i32 を esp-dsp 経由で提供
```

shim/ の Rust crate が必要なのは、mfsk-core の FFT-extern 契約が
`extern "Rust"` (C と ABI が違う) を使うため、純 C コンパイルユニットからは
提供不可。shim は ~50 行 Rust + `embedded-poc/m5stack-core2/src/esp_dsp_fft.rs`
の vendor copy で済みます。

`components/mfsk_ft8/CMakeLists.txt` 最小例:

```cmake
idf_component_register(INCLUDE_DIRS "include"
                       REQUIRES espressif__esp-dsp)
add_library(mfsk_ft8_rust STATIC IMPORTED)
set_target_properties(mfsk_ft8_rust PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/lib/libmfsk_ft8.a)
target_link_libraries(${COMPONENT_LIB} INTERFACE mfsk_ft8_rust)
```

完成 skeleton は
[`embedded-poc/idf-component/`](https://github.com/jl1nie/mfsk-core/tree/main/embedded-poc/idf-component)。

## ライブラリで提供しないもの

mfsk-core はデコード/エンコードパイプラインまでです。以下は
ハードウェア依存が大きく汎用 API が役に立たないため、**意図的に
スコープ外**です:

- 音声入出力 (I2S, マイクゲイン, sample-rate clock recovery)
- ディスプレイ / UI (TFT, OLED)
- ネットワーク (Wi-Fi, BLE, MQTT)
- RTOS タスク連携
- 時刻同期 (NTP, GPS)
- 永続化 / 設定ストレージ

`embedded-poc/m5stack-core2/src/main.rs` が 1 ターゲット (esp-idf-svc)
での全部入り例ですが、これは **整合した動作するサンプルバイナリ**で
あって、メンテされるアプリケーションではありません。他ターゲットは
独自の glue を書いてください。中身を template として参照してください。

## パフォーマンスベンチマーク

3 つの実 QSO 録音 (12 kHz / mono / i16 PCM、各 ≈ 360 KB) をバイナリに
ベイクし、`rx-wavsim` streaming bench が queue pipeline に real-time
pace で push、WAV 完了 notify ごとに 1 slot decode する構成で計測。
**post-SlotEnd** = SlotEnd notify から「decode 完了」までの wall-clock
= ユーザ体感 RX latency (stage 2 は capture 末尾と並列実行されるため
この budget には入らない。「Streaming RX pipeline アーキテクチャ」節
参照)。

`q_thresh = 12` (production デフォルト、full recall)。

`qso3_busy.wav` は **WSJT-X 公式配布 FT8 reference 録音**
(`samples/FT8/210703_133430.wav`、busy 7-station slot。
2026-05-04 に `cmp` で bit-identical 確認済)。`qso1` / `qso2` は
informational な実 QSO キャプチャで、breadth として有用だが
公式 reference ではない。

| WAV | 結果 (0.6.2) | S3 LX7 post-SlotEnd |
|---|---|---:|
| qso1 (mid-band)                      | 3 局 (informational) | — |
| qso2 (mid-band)                      | 3 局 (informational) | — |
| **qso3 busy band (WSJT-X リファレンス、JTDX golden 18 entry)** | **6/18 + 1 bonus = 7 件** | **≈ 1.19 s** |

0.6.2 で組込 LLR scalar を Q11i16 に戻したことで qso3 recall が 6 →
**7 件** (XE2X HA2NP RR73 復活)、post-SlotEnd 1.341 s → 1.191 s に
高速化。残り 12 件の JTDX-confirmed 信号は `fine_refine_pass1` 相当
の 192k-FFT cd0 chain を Xtensa で動かせていないことに起因する
構造的限界 — 下記「embedded fine_refine の天井」参照。

### embedded fine_refine の天井 (0.6.2 status)

0.6.2 開発中に embedded recall を JTDX-extra 帯まで持ち上げる経路を
2 通り試したが、いずれも同じ事実に行き当たった:

1. **per-symbol DFT iteration** (`fill_symbol_spectra` を候補毎に
   呼ぶ): 見積 ~100 ms に対し実測で 5 s 超 — slot あたり
   200k+ × 1920-sample DFT が走り、FreeRTOS Task Watchdog が trip。
2. **`cd0` 複素ベースバンドを `esp-dsp` カスケード FIR で生成**
   (3:1 → 4:1 → 5:1 = 60:1): 計算量側は解決したが、PSRAM 帯域
   ~80 MB/s (S3 OCT mode) の天井に当たる — 15 候補分の cd0 を
   PSRAM 経由で構築するとメモリ転送だけで ~1 s。`dsps_fird_init_f32`
   の 16 byte alignment 要件、scratch 断片化、decode loop 内
   lazy alloc による TLSF heap 破壊などは付随的。

このため embedded ship recall は当面 7 件で固定。i16-throughout
streaming FIR cd0 path もしくは chunk-processing リファクタが
入るまで動かない。0.6.x patch 範囲外、将来の
`decode_block_with_fine_refine` issue で追跡予定。

### WSJT-X リファレンスでの host wide-band 比較

WSJT-X 公式 reference (qso3 = 210703_133430.wav) について、
`decode_frame` (host wide-band: rustfft、`BpAllOsd`、max_cand=200、
OSD-3 fallback あり) vs `decode_block` (embedded equivalent: 整数
pipeline、max_cand=15、q=12) を直接比較
(test: `mfsk-core/tests/ft8_reference_suite_recall.rs`):

| 実行 | 認識数 / 13 truth | wall-clock | ハードウェア |
|---|---:|---:|---|
| host wide-band (`decode_frame BpAllOsd 200`) | **13 / 13** | 140 ms | Ryzen デスクトップ |
| host fixed-point (= embedded、`decode_block` 15) | 7 / 13 | 6 ms | Ryzen デスクトップ |
| **M5StickS3 LX7** (`decode_block`、実機) | 7 / 13 | **707 ms** | post-SlotEnd、240 MHz dual-core |
| **M5Stack Core2 LX6** (`decode_block`、実機) | 7 / 13 | **1434 ms** | post-SlotEnd、240 MHz dual-core |

embedded が落とす 6 局は host wide-band の **PASS1=200 + OSD-3
fallback** が拾うもので、embedded budget では届かない。host
fixed-point (6 ms) と実機 (707 ms / 1434 ms) の wall-clock 差は
純粋な CPU 性能比 (Ryzen ~5 GHz × 16 core vs Xtensa 240 MHz × 2 core)
であって、アルゴリズム / pipeline 上のオーバーヘッドではない —
両方とも同一の整数 pipeline を走っている。

#### embedded で PASS1 / OSD を広げない理由

S3 LX7 実機検証 (`logs/s3_pass100_max30_2026-05-04.log`):

| 設定 | qso3 post-SlotEnd | qso3 recall | total recall |
|---|---:|---:|---:|
| Bp/30/15 (ship) | **0.71 s** | 7/13 | 15/22 |
| Bp/100/30 | **1.59 s** | 7/13 (変化なし) | +1 (qso1 OH3NIV のみ) |
| BpAllOsd/200/100 (host 推定) | ~7 s | 7/13 (+1 qso3 N1JFU) | 16/22 |

非自明な 2 つの所見で `PASS1=30 / max_cand=15` 維持を決定:

1. **qso3 busy band の recall は coarse_sync ランクで頭打ち、BP / OSD
   作業量ではない。** PASS1 を 30→100、max_cand を 15→30 にしても
   qso3 で 1 局も追加で復号できない — 落としている 6 局は
   coarse_sync ランク 100 圏外。WSJT-X wide-band の特徴である
   **iterative subtraction** が必要だが `decode_block` 未実装。
2. **FT8 の QSO turnaround budget は post-SlotEnd ~2 秒**、15 秒スロット
   全体ではない。decode 後に UI は waterfall 描画、callsign list 更新、
   RPRT 表示、次スロット TX 準備、そして **NTP / GPS 同期 RTC が無い
   chip では復号メッセージの `dt_sec` の中央値から slot timing 再推定**
   (素朴な平均は外れ値に弱く、CRC は通るが sync がずれた 1 局で
   slot 位相が暴れる。ESP32 内蔵 RTC ドリフトでは frame alignment を
   この decoder 出力由来の推定値に従属させる必要あり) を全部こなす
   必要がある。Bp/100/30 だと qso3 後に 0.4 秒しか残らず、これ全部を
   回すには厳しい。+1 (qso1 のみ) の recall 増では割に合わない。

ステージ別内訳 (qso3 busy band):

| stage | Core2 LX6 | S3 LX7 | 備考 |
|---|---:|---:|---|
| stage 1 (incremental, capture 中) | ≈ 1.0 s 分の compute / 15 s | 同 | capture CPU 利用率 ~6 % |
| stage 2 `coarse_sync_split_with_allsum` (capture 中) | 0.65 s | 0.18 s | SlotEnd notify latency と並列、隠蔽済 |
| pass 2 `pass2_split` (post-SlotEnd) | 0.19 s | 0.12 s | dual-core, head/tail split |
| stage 3 `stage3_split` (post-SlotEnd) | 1.24 s | 0.58 s | dual-core, **work-stealing** per-cand |

両 chip がこのレンジに乗っている wall-clock 改善は 2 つ:

1. **Stage 2 を capture 中に隠蔽。** `stage1_inc` が pair 92 完成時
   (SlotEnd の ~200 ms 前) に `SpecBundle` (spec + per-half allsum)
   を `spec_q` で送出。main 側は audio capture 末尾と並列で
   `coarse_sync_split_with_allsum` を実行 → post-SlotEnd budget に
   は入らない。
2. **Stage 3 work-stealing。** `dual_core::stage3_split` は candidate
   を事前分割しない。PRO_CPU / APP_CPU が共有
   `Vec<Option<RefinedCandidate>>` から `AtomicUsize::fetch_add(1)`
   で動的に取り合うため、失敗 cand が偏って片方が手待ちになる事態が
   発生しない。qso3 (15 cand 中 ~半数が失敗して 4 LLR variant 全
   走り) の per-cand BP wall-clock 不均衡を吸収する。

生ログ:
`embedded-poc/m5stack-core2/logs/core2_q_sweep_2026-05-04.log`、
`embedded-poc/m5stack-s3/logs/s3_workstealing_2026-05-04.log`。

## Streaming RX pipeline アーキテクチャ

Phase E 後の pipeline (`embedded-poc/embedded-shared` 配下) は
**queue ベース、slot 単位の単一所有権** — 共有 mutable state なし、
notify と out-pointer の分離なし:

```
wav_sim (PRO_CPU, prio 4)
  │
  │  ChunkMsg = Samples(Vec<i16>) | SlotEnd { wav_idx, total_samples }
  ▼
chunk_q (depth 4)
  │
  ▼
stage1_inc worker (APP_CPU, prio 3)
  │  内部状態: per-slot WorkerCtx { audio, spec, allsum_head/tail,
  │                                 next_pair, … }
  │  pair 92 完成と同時 (SlotEnd の ~200 ms 前) に SpecBundle を発火 →
  │  main は capture 末尾と並列で stage 2 を回せる
  │
  ├──▶ spec_q (depth 2): SpecBundle { spec, allsum_head, allsum_tail }
  └──▶ slot_q (depth 2): Slot { audio, wav_idx, inc_total_us }
       (SlotEnd ChunkMsg を受けてから)
       │
       ▼
main / decode task (PRO_CPU, prio 6)
       │  spec_q recv → stage 2 (coarse_sync_split_with_allsum, dual-core)
       │  slot_q recv → pass 2 (refine_candidates, dual-core)
       │              → stage 3 (per-cand work-stealing, dual-core)
       ▼
DecodeResult[]
```

`dual_core` が stage 2 / pass 2 / stage 3 用の dispatch queue を別途
保持 (1 つの job queue + variant ごとの result queue)。所有権の移動は
すべて `Box::into_raw` の生ポインタを queue item に乗せる方式 —
host の `mpsc::sync_channel` と同等のセマンティクス。

Pipeline 不変条件:
- wav_sim は 1 slot 内で Samples / SlotEnd を FIFO 順に送る。
- stage1_inc は SpecBundle を 1 slot あたり最大 1 回だけ送る
  (`next_pair == N_PAIRS` 到達直後、または `finalize_slot` 時の
  fallback)。
- main は SpecBundle と Slot を受信順 (FIFO) でペアリング。
- main は `STAGE3_RESULT_Q` recv で block するので、worker 側の
  生ポインタ (audio, cs scratch, work-stealing slot array) は
  関数呼び出し時間内ずっと有効。

参照: `embedded-poc/embedded-shared/src/pipeline.rs` (queue helper +
`ChunkMsg`/`SpecBundle`/`Slot` 型) と
`embedded-poc/embedded-shared/src/dual_core.rs` (work-stealing stage 3
dispatch + Job enum)。

## バイナリフットプリント (Core2 リファレンス, ELF)

`xtensa-esp32-elf-size -A` 計測:

| 領域 | サイズ | 内容 |
|---|---|---|
| IRAM (`.iram0.text` + vectors) | **69 KB** | 内部 RAM コード (esp-idf 割り込みハンドラ等) |
| DRAM (`.dram0.data` + `.bss`) | **76 KB** | 内部 RAM 静的領域 (BASIS scratch 60 KB + spectrogram キャッシュ等) |
| Flash (`.flash.text`) | **448 KB** | アプリコード |
| Flash (`.flash.rodata`) | **1.21 MB** | 読み出し専用データ (うちベイクした実 QSO WAV 約 1.08 MB) |
| **合計バイナリ** | **1.997 MB** | |

実 QSO WAV (1.08 MB) を除けば mfsk-core + esp-idf + アプリ全部で
約 920 KB。ライブラリ側だけのコードサイズはおおよそ **150-200 KB**
の見積もり (esp-idf を分離して測ると正確)。

