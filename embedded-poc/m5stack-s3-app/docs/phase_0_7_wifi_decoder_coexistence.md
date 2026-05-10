# Phase 0.7: WiFi / decoder coexistence via runtime BASIS allocation

## Context

M5StickS3 FT8 controller (`embedded-poc/m5stack-s3-app/`) は ESP32-S3 の
内部 DRAM ~161 KB を WiFi + decoder + LCD で食い合っている。Phase 0.6 で
判明した「WiFi 上げると decoder spawn が `xTaskCreatePinnedToCore=-1`
で失敗」を `WIFI_SSID` 設定時に decoder 自体を skip する build-time
スイッチで暫定回避中 (`src/main.rs:113-119`)。

**根本原因**: BASIS_RE/IM (Q15 reference basis for the esp-dsp `dot_q15_i32`
asm kernel) が `.bss` で 2 ペア × 60 KB = **120 KB を boot 時から内部
DRAM に予約**している。WiFi モードでも decoder が動かないだけで 120 KB
は無駄に占有されたまま。BASIS は PSRAM 不可 (asm dotprod が 1 cycle/sample
なのは fast RAM 配置時のみ、PSRAM 配置で 5-10× slowdown、`mfsk-core/src/
ft8/decode_block.rs:1442-1449` に明記)。

**ユーザ要求**:
1. WiFi と decoder は同時動作不要、boot mode 切替で OK
2. WiFi は本番 firmware でも有効にしたい (ADIF upload / OTA / remote
   config の余地を残す)。デバッグ専用にしない
3. BASIS は main + worker 両方 heap 化して合計 120 KB を mode 依存で
   開放する

**達成像**:
- decode mode: 120 KB BASIS を heap で確保、decoder + LCD + (optional UAC)
- wifi mode: BASIS alloc skip、WiFi STA + UDP log + LCD、+ (将来 ADIF/OTA)
- mode 切替は (a) cfg.toml の WIFI_SSID 在/不在 + (b) NVS-stored boot
  mode flag + (c) boot 時ボタン長押し override の 3-tier
- production firmware は両方 build に含み、device の NVS で挙動が決まる

## パイプラインと BASIS lifecycle (確認済み)

```
Stage1_inc (BASIS なし、heap Vec)
   ↓ Spectrogram + allsum
coarse_sync_split (BASIS なし、純粋 spectral search)
   ↓ Candidate list
Pass2_refine (BASIS 使う、main & worker 各々 [i16; 15360] × 2)
   ↓ Refined candidates
Stage3_BP (Pass2 と同 BASIS buffer を逐次再利用)
```

各 core 内で Pass2 → Stage3 は sequential、同一 BASIS pair を共有。
よって **per-core 1 pair (RE+IM) = 60 KB、2 core で 120 KB 合計**。

## 変更ファイル

### 1. `embedded-poc/embedded-shared/src/dual_core.rs`

現在 (lines 57-58):
```rust
static mut BASIS_RE_C1: [i16; BASIS_SCRATCH_LEN] = [0; BASIS_SCRATCH_LEN];
static mut BASIS_IM_C1: [i16; BASIS_SCRATCH_LEN] = [0; BASIS_SCRATCH_LEN];
```

変更後:
- 上記 2 static を削除
- `init` を `init(re_c1: *mut i16, im_c1: *mut i16)` 拡張 (旧 `init()` は
  `init_with_static_basis()` に rename、`compute_bench` / `rx_wavsim` 用)
- 内部に `AtomicPtr<i16>` 2 個で受けた pointer を保存
- `worker_main` (lines 177-214) の `&mut BASIS_RE_C1` / `&mut BASIS_IM_C1`
  を `core::slice::from_raw_parts_mut(ptr, BASIS_SCRATCH_LEN)` で再構築

`compute_bench` / `rx_wavsim` bin は内部で static BASIS を持ち
`init_with_static_basis()` を呼ぶ薄い wrapper 経由で互換維持。

### 2. `embedded-poc/m5stack-s3-app/src/decode_pipeline.rs`

現在 (lines 33-35):
```rust
const BASIS_SCRATCH_LEN: usize = NTONES * NSPS;
static mut BASIS_RE: [i16; BASIS_SCRATCH_LEN] = [0; BASIS_SCRATCH_LEN];
static mut BASIS_IM: [i16; BASIS_SCRATCH_LEN] = [0; BASIS_SCRATCH_LEN];
```

変更後:
- 2 つの static を削除
- `pub fn run(re_main, im_main, re_c1, im_c1: &'static mut [i16]) -> !`
  に変更。caller (main.rs) が boot で確保した slice を受け取る
- `pass2_split` / `stage3_split` 呼び出し (lines 130-148) の `&mut BASIS_RE`
  / `&mut BASIS_IM` を引数 slice に差し替え
- 冒頭で `dual_core::init(re_c1.as_mut_ptr(), im_c1.as_mut_ptr())` 呼ぶ

### 3. `embedded-poc/m5stack-s3-app/src/main.rs`

新規 `boot_mode` モジュール (新ファイル `src/boot_mode.rs`):

```rust
pub enum BootMode { Decode, Wifi }

pub fn determine(nvs: &EspDefaultNvsPartition,
                 buttons: &mut Buttons) -> BootMode {
    // 1. Boot 時に KEY1 押下中なら反転 (override)、500ms 以内に検出
    // 2. NVS "boot_mode" = "wifi" なら Wifi
    // 3. それ以外 (default) は Decode
}

pub fn flip_and_restart(nvs: &EspDefaultNvsPartition,
                        current: BootMode) -> ! {
    // current の逆を NVS に書き、esp_restart()
}
```

`main.rs` の boot 順:
1. `link_patches` → `LOGGER.install` → log banner + build-stamp
2. `Peripherals::take`
3. NVS open + `boot_mode::determine`
4. **decode mode**:
   - `log_free_internal("pre-basis-alloc")`
   - `alloc_basis_dram` を 4 回 (re_main / im_main / re_c1 / im_c1)
     - `heap_caps_aligned_alloc(16, 30720, MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT)`
     - null check + alignment assertion (`ptr & 0xF == 0`)
   - `log_free_internal("post-basis-alloc")` (差分 ~123 KB 期待)
   - LCD 起動 + button 監視 thread
   - `std::thread::spawn` で `decode_pipeline::run(re_main, im_main, re_c1, im_c1)`
5. **wifi mode**:
   - BASIS alloc skip
   - WiFi STA up (既存) → UDP sink (既存)
   - `flash_log.rs` の littlefs 読み出し → 前回 decode mode で貯まった
     ログを UDP で吐く (Phase 0.7-stretch、§9 参照)
   - LCD + button 監視

両モード共通の button hook (新規 `buttons.rs` 拡張):
- KEY2 (GPIO 12) 長押し 2 秒 → `boot_mode::flip_and_restart`
- LCD 上部 status bar に現 mode を表示 (`Mode: DECODE` / `Mode: WIFI`)

### 4. `embedded-poc/m5stack-s3-app/src/buttons.rs`

既存 button-edge 検出に **long-press 検出** (2 秒継続) を追加。長押し
イベントを `Buttons::take_event()` に enum (`Press(Key)` /
`LongPress(Key)`) で返す。decode_pipeline / display 両方の loop が
poll する。

### 5. (オプション) `embedded-poc/m5stack-s3-app/src/flash_log.rs`

現状 placeholder (45 行、open() = None)。**§9 stretch goal**:
- partition `littlefs` (0x340000..0xC0000、1 MB) を mount
- `LogFanout::flash` に sink 登録 → 全 `log::info!` を `/littlefs/run.log`
  に append (rotate at 800 KB)
- wifi mode boot 時に dump 経路: `read_to_string` → UDP send_line ×N
  → unlink

これは BASIS heap 化とは独立に動かせるので、別 commit に分離。本プラン
は heap 化を確実に通すことが先。

## Boot-time sequence (decode mode)

```
0.000 s  link_patches, LOGGER install
0.010 s  Peripherals::take, NVS open
0.020 s  boot_mode::determine → Decode
0.025 s  free_internal probe: ~250 KB (after IDF startup)
0.030 s  alloc_basis_dram × 4 (60 KB main + 60 KB worker = 120 KB)
0.040 s  free_internal probe: ~127 KB (差分 ~123 KB)
0.050 s  LCD init, button thread spawn
0.100 s  decode_pipeline thread spawn (32 KB stack)
0.200 s  dsp_worker spawn (in dual_core::init) on core 1
0.500 s  Stage1_inc task spawn (16 KB stack)
1.000 s+ Streaming decode loop
```

WiFi mode は手順 3 で Wifi を選んだあと alloc step を完全 skip し、WiFi
STA 起動に移る (現在の WIFI_SSID-set 経路をほぼ流用)。

## Lifecycle & ownership

- BASIS は decode mode boot 時に一度確保、program lifetime 中は free
  しない (mode 切替は `esp_restart` で heap がリセットされるため、
  fragmentation 問題が発生する余地がない)
- `&'static mut [i16]` は decode_pipeline thread closure に move、ほかから
  alias されない (single-thread-per-core ownership は既存と同じ)
- worker pair は `dual_core::init` 経由で `AtomicPtr` に saved、
  `worker_main` のみが reads。`init` ↔ `xTaskCreatePinnedToCore` の
  happens-before 関係で memory ordering 保証

## Verification

### DRAM probe
新規 helper `log_free_internal(label: &str)`:
```rust
let free  = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
let largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
log::info!("[mem] {label} free_internal={free} largest={largest}");
```
boot 中に 5 点で呼び出し: pre-alloc / post-alloc / post-LCD / post-thread-spawn /
post-WiFi-init。

期待値:
- decode mode: post-alloc free が pre-alloc - ~123 KB
- wifi mode: post-WiFi free ~100-130 KB (現在の wifi mode は 60-80 KB に
  落ちているはず、120 KB 改善が観測されれば成功)

### Decode parity
- `qso3_busy.wav` ループで `decode_block` の recall + 速度を pre/post で
  pair-match
- 期待: recall 完全保持 (7 total / 1.19s post-SlotEnd、Issue #40 fix 後の
  baseline と一致)、wall-clock ±5% 以内
- `dual_core::pass2_split` / `stage3_split` の timing log で per-stage
  ms を pre/post 比較

### Alignment sanity
`alloc_basis_dram` 内で `assert_eq!(ptr as usize & 0xF, 0)`。違反すると
asm 退化 path に落ちて wall-clock が ~2× 悪化するので boot で検出。

### Mode switch
1. cfg.toml に SSID 入れた状態 + NVS clear で boot → "Mode: DECODE" (NVS
   default 優先)
2. KEY2 長押し → "Mode: WIFI" に LCD 切替後 esp_restart → boot で
   WiFi STA up
3. KEY2 長押し再度 → decode mode に戻る
4. boot 時 KEY1 押下 → 現 NVS 設定とは逆 mode で boot (override)

### Live UDP capture (wifi mode)
`embedded-poc/scripts/udp-log-listen.sh` で boot から 30 秒キャプチャ、
`[mem]` 行が ASCII で見えること、`alive tick=N` が継続することを確認。

## Risks & 緩和

| Risk | 緩和 |
|------|------|
| `heap_caps_aligned_alloc` がない / 違う signature | esp-idf-sys バインディングを `~/.cargo/registry/.../esp-idf-sys-*/build/bindings.rs` で確認、未 export なら `extern "C"` で自前宣言 |
| Largest free block が 30 KB 未満で alloc 失敗 | BASIS alloc を boot 最初 (LCD/WiFi 前) に移動、heap fresh 状態で確保 |
| Worker `AtomicPtr` の memory ordering | `init` は `xTaskCreatePinnedToCore(worker_main)` より前に store、task spawn が release barrier |
| `'static mut [i16]` の Rust UB | 各 slice は single-owner thread に move、aliased read なし。worker pair も `dual_core::init` 後に worker_main が排他読みで安全 |
| Button long-press detection が QSO 操作と競合 | KEY2 は現在 unused (KEY1 = next freq、KEY2 reserved per `board.rs`)。長押し 2 秒は通常 UI 操作と区別可能 |
| NVS write 失敗 (flash 摩耗 / IO error) | エラー log + WiFi mode で続行 (= デバッグ可能)。critical path ではない |
| compute_bench / rx_wavsim の互換崩れ | `init_with_static_basis()` wrapper で旧 init API を保持、bin 側は単一行変更 |
| WSL2 環境での flash 不安定 | `/dev/ttyACM0` 認識のため `sudo modprobe cdc_acm`、`usbipd attach --wsl` の手順は memory に記録済 |

## Out of scope (defer)

- **Concurrent WiFi + decoder**: 物理的に 161 KB に収まらないため不可能
- **MALLOC_CAP_INTERNAL の fragmentation hardening**: ordering で boot 時
  対応のみ、runtime fragmentation 対策はやらない
- **OTA / ADIF upload / remote config**: WiFi mode の枠組みだけ用意、
  本機能は別 phase
- **flash_log/littlefs wiring**: §9 stretch、本プランの core path には
  含めない。BASIS heap 化が green になってから別 commit
- **WiFi sdkconfig PSRAM tuning**: heap 化で十分 (`CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP`
  等は触らない。動作変える前に baseline 確立)
- **stage1_inc heap 移動**: 現状 heap Vec ですでに `.bss` ではない、対象外

## Critical Files
- `/home/minoru/src/mfsk-core/embedded-poc/m5stack-s3-app/src/main.rs`
- `/home/minoru/src/mfsk-core/embedded-poc/m5stack-s3-app/src/decode_pipeline.rs`
- `/home/minoru/src/mfsk-core/embedded-poc/m5stack-s3-app/src/boot_mode.rs` (new)
- `/home/minoru/src/mfsk-core/embedded-poc/m5stack-s3-app/src/buttons.rs`
- `/home/minoru/src/mfsk-core/embedded-poc/embedded-shared/src/dual_core.rs`
- 参考 (read-only): `/home/minoru/src/mfsk-core/mfsk-core/src/ft8/decode_block.rs:1376-1468`
  (BASIS_SCRATCH_LEN + DRAM-placement 制約のドキュメント)
- 参考 (read-only): `/home/minoru/src/mfsk-core/embedded-poc/embedded-shared/src/stage1_inc.rs`
  (BASIS なしを再確認)

## 実装順序

1. `dual_core.rs` API 変更 (`init` 拡張 + `init_with_static_basis` wrapper)
   → compute_bench / rx_wavsim を新 API 対応 → cargo check pass on
   host + xtensa
2. `decode_pipeline.rs` の static BASIS 削除 + `run` signature 変更
3. `main.rs` に `alloc_basis_dram` + `log_free_internal` + decode mode 分岐
   修正、WIFI_SSID-empty 経路で動作確認 (NVS / button まだ未統合)
4. 実機 flash + `udp-log-listen` で `[mem]` 行 + alive_tick 観測、recall
   parity 確認
5. `boot_mode.rs` 新規 + NVS read/write + KEY2 long-press
6. mode 切替 boot loop の実機検証 (decode↔wifi 行き来 5 回)
7. (stretch) `flash_log` littlefs wiring を別 commit で
