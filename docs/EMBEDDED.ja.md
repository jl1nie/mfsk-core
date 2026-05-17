# 組み込みクレート — 概要

`mfsk-core` (crates.io 公開の host-and-embedded 共通 FT8 デコーダ
ライブラリ) には `embedded-poc/` 配下に Xtensa ESP32 実機統合用の
組み込みクレートが同梱されている。本文書は **組み込み全体像の入口** —
対応ボード、各クレートの役割、運用マニュアルの所在をまとめる。

ホスト側ライブラリ利用 (組み込みなし) は
[`docs/LIBRARY.ja.md`](LIBRARY.ja.md) 参照。

---

## ボードと役割

| ボード | ステータス | 役割 |
|---|---|---|
| **M5StickS3** (ESP32-S3 LX7) | ✅ Production | 手持ち FT8 コントローラ — 内蔵マイク、IC-705 への BLE CI-V、LCD UI、QSO FSM、optional WiFi UDP log。**日常運用ターゲット** |
| **M5Stack Core2** (ESP32 classic LX6) | 🔬 診断 / 2 ボード検証用 | デコーダを `wav_sim` baked-WAV ループで動作、LCD に結果表示。外付け I/O (mic / speaker / BLE) **未確定 — HW spec TBD**。LX6 上で `mfsk-app-shared` の ボード非依存 API を相互検証する目的 |
| **M5Stack CoreS3** (ESP32-S3 LX7) | 📋 Planned | **メイン UAC コントローラターゲット**として S3 の後継 — CoreS3 は VBUS source + AW9523B BUS_OUT_EN 回路を備えており、M5StickS3 にはない真の USB-Host audio class to IC-705 が実装可能。HW 未調達 |
| **M5StickS3 compute bench** (ESP32-S3) | 🛠️ Tool | デコーダ専用ベンチクレート (`m5stack-s3/`)、`decode_block` を canned WAV で timing sweep。エンドユーザー向けではない |

---

## `embedded-poc/` 配下のクレート構成

```
embedded-poc/
├── CLAUDE.md                   ← cross-board agent / ツールチェインメモ
├── scripts/
│   ├── flash-monitor.sh        ← flash + capture wrapper (正規)
│   └── udp-log-listen.sh       ← PC 側 UDP log receiver
├── assets/                     ← WSJT-X リファレンス WAV
├── m5stack-s3-app/             ← M5StickS3 FT8 コントローラ (PRODUCTION)
│   ├── MANUAL.md / MANUAL.ja.md   ← ユーザーマニュアル (ここから読む)
│   ├── cfg-sample.toml         ← WiFi / station 設定テンプレート
│   └── src/
├── m5stack-core2-app/          ← Core2 wav_sim デコーダ (DIAGNOSTIC)
│   └── cfg-sample.toml
├── m5stack-cores3-app/         ← (計画中) CoreS3 メイン UAC ターゲット
├── m5stack-s3/                 ← S3 compute bench (DECODER PROFILING)
├── mfsk-app-shared/            ← ボード非依存アプリロジック (QSO / UI / WiFi / log)
├── embedded-shared/            ← no_std デコーダ統合 (FFT planner / dual_core / stage1_inc)
└── idf-component/              ← C only ESP-IDF プロジェクト用 mfsk-ffi-ft8 ラッパー
```

### 2 つの共有レイヤ

app クレート間の共有クレートは 2 つあり、抽象度が異なる：

- **`embedded-shared/`** — デコーダレベル。`no_std`、streaming pipeline
  (`stage1_inc` incremental FFT spectrogram builder)、dual-core
  dispatcher、esp-dsp FFT planner ブリッジ、I2S resampler、ボード別
  sample feed (`wav_sim`、`compute_bench`)。デコーダカーネルに触れる
  もの全般。
- **`mfsk-app-shared/`** — コントローラレベル。QSO ステートマシン、UI
  primitive + `embedded-graphics` 描画、WiFi STA + UDP log datagram
  sink、ADIF + flash log placeholder、NVS-backed `BootMode`、
  `LogFanout` log multiplexer。デコーダの上位層。

境界は **データフロー** (channel + 共有 state mutex)、callback では
ない — trait は跨がない。両クレートとも全 app クレート (s3-app、
core2-app、future cores3-app) で consume される。

---

## エンドユーザー向け: ファームウェアの運用

ボード別マニュアルから始めること。現状存在するのは 1 つ：

- **M5StickS3**: [`embedded-poc/m5stack-s3-app/MANUAL.ja.md`](../embedded-poc/m5stack-s3-app/MANUAL.ja.md)
  ([EN](../embedded-poc/m5stack-s3-app/MANUAL.md))

S3 マニュアル収録内容：

1. 7 つの boot mode と日常運用 2 mode (Acoustic、Qso)
2. 必須 / optional ハードウェア
3. 初回 setup: toolchain、build、flash
4. `cfg.toml` リファレンス (WiFi 認証情報、operator callsign / grid)
5. `BootMode` 切替 (KEY1 boot-hold / KEY2 long-press)
6. Phase 1.7.7 以降のデコードパイプライン構成 (Goertzel per-symbol
   DFT、内部 DRAM scratch ゼロ)
7. UI / ボタン / メニュー
8. QSO ワークフロー
9. Troubleshooting (USB-CDC フリーズ、OOM、BLE ペアリング、…)

Core2 と CoreS3 ユーザーマニュアルは外付け I/O ハードウェアが確定して
から追加。

---

## ライブラリ consumer 向け: `mfsk-core` の組み込み側要件

`mfsk-core` は `no_std + alloc` 対応。FT8 デコードパス
(`mfsk_core::ft8::decode_block`) は ~150 KB 程度の使用可能 RAM があれば
動作する、ただし以下が前提：

- 複素単精度 FFT バックエンド (`FftPlanner` / `FftPlanner16` trait
  ペア)。組み込みターゲットは `fft-extern` feature を有効化し、
  linker が拾う factory 関数を提供する。in-tree の
  `embedded-poc/embedded-shared/esp_dsp_fft.rs` が Xtensa LX6/LX7 →
  `esp-dsp` の `dsps_fft2r_*` への参照実装。
- (optional) BASIS パス用 Q15 dot product (`mfsk_core_dot_q15_i32`
  extern)。**Phase 1.7.7-Stick 以降は新規 code では使用しない** —
  per-symbol DFT は `fill_symbol_spectra_goertzel` の Goertzel 再帰へ
  移行、外部 symbol 不要。BASIS `dot_q15_i32` パスは 0.7.0 cleanup
  まで API 後方互換のため残置。

### Cargo feature の組み合わせ

組み込みに関連する feature は 2 つ：

- **`fft-extern`** — extern-factory FFT planner 契約を有効化。組み込み
  クレートは自前の `mfsk_core_make_default_fft_planner*` 実装と組で使う。
- **`fixed-point`** — デコーダの spec / cs パスを f32 から i16 / Q15
  scalar 版へ切替。組み込みターゲットは PSRAM 帯域半減と BP scratch
  縮小 (12 KB → 6 KB) のため有効化。

**注意 (0.6.4 で追加)**: `fixed-point` は今や `nstep-half` を implies
する (NSTEP = NSPS/2 = 960 samples/spectrogram column、host default
NSPS/4 = 480 ではなく)。両 feature は歴史的理由で独立だったが、組み込み
ターゲットでは常に共在化していた。decouple すると host fixed-point の
time-grid が embedded と異なる結果に → host validation tests が
embedded 挙動と乖離する原因に。coupling すれば host fixed-point ビルド
が embedded パスを忠実に simulate できる。

### スカラーバリアント

DSP / FEC パイプライン全体は **scalar trait** でパラメータ化されており、
同一ソースが host-friendly f32 パスと embedded-friendly integer パスの
両方にコード重複なくコンパイルされる：

- `core::scalar::SpecScalar` — spectrogram / DFT 出力 scalar (host f32、
  embedded `fixed-point` 時 `i16` Q14)
- `core::scalar::LlrT` — LLR scalar (host f32、embedded Q3i8)
- `core::scalar::CoarseAcc` — coarse-sync accumulator (f32、integer
  variant は 0.5 以降で retire)

trait 選択は `#[cfg(feature = …)]` type alias で完結、**runtime dispatch
は無い**。

---

## 開発者向け: どこから読み始めるか

### レイヤ別アーキテクチャ

| レイヤ | クレート | 中核ファイル |
|---|---|---|
| デコーダカーネル (host + embedded) | `mfsk-core` | `src/ft8/decode_block/{spectrogram, coarse_sync, fill_symbol_spectra, process_candidates}.rs` |
| Streaming pipeline + dual-core | `embedded-poc/embedded-shared` | `src/stage1_inc.rs`、`src/dual_core.rs`、`src/esp_dsp_fft.rs` |
| QSO FSM + UI + WiFi | `embedded-poc/mfsk-app-shared` | `src/qso.rs`、`src/ui/`、`src/wifi.rs`、`src/log_sink.rs`、`src/boot_mode.rs` |
| ボード接続 (I2S / BLE / LCD / PMIC) | per-app クレート | `src/{audio, civ, display, pmic, board}.rs` |

### ツールチェイン bring-up

[`embedded-poc/CLAUDE.md`](../embedded-poc/CLAUDE.md) 参照: 共通
`espup install` / `~/export-esp.sh` setup、flash-and-capture
テンプレート、LX6 vs LX7 (vs LX7+PIE) 比較表。

### なぜ Goertzel (Phase 1.7.7-Stick 設計根拠)

FT8 の per-symbol DFT は、各 tone 周波数で `Σ x[n] exp(-jωn)` を 1920
sample にわたって計算、これを candidate の cs matrix 全 (symbol、
tone) 組み合わせで実行する。当初の embedded パスは事前計算済 Q15
sin/cos table (`BASIS`) + asm Q15 dot product で実装 — 高速 (PIE 経由
LX7 で 1 cycle/sample) だが、table が内部 DRAM 60 KB × 2 (re/im) × 2
(dual_core main + worker) = 120 KB を要求した。

その 120 KB の内部 DRAM は M5StickS3 Qso mode で I2S bidir DMA
descriptor が必要とする領域そのもの。基板の連続空き内部チャンクが両者
同時確保には小さすぎ、Qso mode boot が失敗していた。

Phase 1.7.7 で BASIS dot-product を一般化 Goertzel 再帰に置換 — (sym,
tone) ごとに 2-tap IIR + f32 state 3 値、return 時に破棄。内部 DRAM
scratch ゼロ。

性能のトリック: ループ順を **sample-outer / tone-inner** にすることで
8 個の per-tone 再帰 (FT8 tone 1 個ずつ) が独立な dependent chain
として FPU pipeline を並列通過。LLVM が定数 bound `NTONES = 8` の
内側ループを unroll、Xtensa FPU が per-chain latency を並列に吸収する。
結果: stage3 cost が BASIS asm dot product と同等 (S3 `qso3_busy.wav`
で ~1.4 s)、**scratch ゼロ + +0.16..+0.63 dB SNR 改善** (f32 Goertzel が
旧 Q15 BASIS dot product より精度高い)。

移行の完全ログは project memory `phase177-goertzel` と PR #103 参照。

---

## リリースステータス

[`docs/ROADMAP.md`](ROADMAP.md) に embedded クレート全体の phase 別計画。
2026-05-18 時点：

- **M5StickS3 (`m5stack-s3-app`)**: Phase 1.5 → 1.7.7-Stick 出荷済
  (acoustic capture、BLE CI-V、QSO FSM、Goertzel 移行)。Phase 2
  (TX scheduler 堅牢化) と Phase 3 (UI polish) 進行中。
- **M5Stack Core2 (`m5stack-core2-app`)**: Phase 2 scaffold (boot、
  wav_sim デコード、LCD コーナークロップ描画)。外付け I/O は HW spec
  確定待ちで defer。
- **M5Stack CoreS3 (`m5stack-cores3-app`)**: 未作成。Phase 0-Core
  (crate skeleton + S3 sibling からの UAC port) は HW 到着待ち。

`mfsk-core` 自体は crates.io で `0.6.3`; Phase 1.7.7 機能 (Goertzel
移行、`fixed-point` → `nstep-half` coupling、mag² saturation fix) は
`0.6.4` で出荷予定 (PR #103 main merge 後にリリース PR)。
