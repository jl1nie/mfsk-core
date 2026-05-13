# mfsk-core M5Stack S3 test bench

FT8 decode-on-hardware PoC for **M5Stack S3 系 (ESP32-S3, Xtensa LX7
dual-core @ 240 MHz, 8 MB Octal PSRAM 想定)**。`m5stack-core2` (LX6)
クレートからの複製で、issue #15 Phase 2 の baseline 取得が主目的
(2026-05-04 〜)。

## なぜ S3 を別クレートにしているか

`mfsk-core` 本体はジェネリックで LX6 / LX7 両対応を維持するが、
embedded-poc レイヤは **構造分岐を許容する** 方針 (issue #15 前提結論
4)。LX6 と LX7 でメモリ階層 (内蔵 SRAM 容量、PSRAM 帯域、PIE SIMD
有無) が別物なので、`m5stack-core2/` の partition は LX6 制約に最適化
された解として固定し、LX7 側は別途調整できるようにする。

`m5stack-core2/` は Phase 2 では一切変更しない。共通切り出し
(`embedded-poc/common/`) は Phase 3-5 で重複が顕在化したら検討。

## ビルド & flash

`CLAUDE.md` 参照。要点だけ:

```sh
source ~/export-esp.sh
cargo build --release            # xtensa-esp32s3-espidf
espflash flash --monitor --port /dev/ttyACM0 \
    target/xtensa-esp32s3-espidf/release/mfsk-core-m5stack-s3
```

`rx-wavsim` は m5stack-core2 と同じく WAV-fed streaming bench:

```sh
cargo build --release --bin rx-wavsim
espflash flash --monitor --port /dev/ttyACM0 \
    target/xtensa-esp32s3-espidf/release/rx-wavsim \
    | tee logs/s3_baseline_$(date +%Y-%m-%d).log
```

## LX6 baseline (比較対象, 0.5.3 / mfsk-core)

`m5stack-core2/` の `rx-wavsim` 実機 (`logs/rx_wavsim_phaseE_2026-05-03.log`):

| WAV | total | stage1 (take) | stage2 | pass2 | stage3 | results |
|-|-|-|-|-|-|-|
| qso1 | 1.83 s | 0.05 | 0.68 | 0.18 | 0.92 | 3/3 ✓ |
| qso2 | 1.45 s | 0.05 | 0.68 | 0.18 | 0.54 | 5/5 ✓ |
| qso3 (busy) | **1.98 s** | 0.05 | 0.65 | 0.18 | 1.10 | 7/7 ✓ |

これに対する LX7 の素のリビルド差分が Phase 2 baseline。
qso3 < 1.0 s に届けば Phase 4 (coarse_sync MAC 化) と Phase 5 (LX7
メモリ partition) はスキップ寄りに倒す判断ゲート。

## LX6 → S3 移植時の差分

| 項目 | LX6 (m5stack-core2) | S3 (m5stack-s3) |
|-|-|-|
| target | `xtensa-esp32-espidf` | `xtensa-esp32s3-espidf` |
| MCU env | `esp32` | `esp32s3` |
| PSRAM mode | QUAD (~40 MB/s) | OCT (~80 MB/s, M5Stack S3 想定) |
| 内蔵 SRAM | ~280 KB usable | ~512 KB DRAM |
| PIE SIMD | 無し | 有り (esp-dsp が自動選択) |
| esp-dsp `dsps_fft2r_fc32_ae32_` | 動作確認済 | 動作確認済 (両 LX で共通) |
| dual_core dispatch + Phase E2 race | 未解決、sequential per-half 運用 | 同方針継承、配線せず |

## 既知の deferred / unverified

- **dual_core dispatch + Phase E2 race**: LX6 で再現する worker tail
  経路の失敗が S3 でも出る可能性が高い。`rx_wavsim` は sequential
  per-half on main を継承して 7/7 信頼運用する。
- **live RX path**: 本クレートは compute bench 専用。実 RX は
  `embedded-poc/m5stack-s3-app` に集約済み（旧 `rx-skeleton` stub は
  2026-05-13 撤去、Core2 統合は #61 で吸収予定）。
- **README の Files セクション** は本クレートでは省略（兄弟クレート
  m5stack-core2 README にある table と同じ構成になる想定）。共通化は
  [`docs/CLEANUP_2026_05.md`](../../docs/CLEANUP_2026_05.md) の δ で扱う。
