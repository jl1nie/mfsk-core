# 組込ターゲット

`mfsk-core` は `no_std + alloc` 対応である。FT8 のデコードパス
(`mfsk_core::ft8::decode_block`) は、呼び出し側が FFT バックエンドを
供給すれば実用 RAM ~150 KB のチップでも動作する。FST4・FT4・WSPR も
それぞれ別の経路で実機に到達している。本書は組込インテグレータ向けの
リファレンスで、ライブラリが呼び出し側に要求するもの、必要なスクラッチ、
そして我々が実際に動かしているターゲットで期待できる性能を扱う。

## 目的別の入口

- **新しい MCU に `mfsk-core` を載せたい** →
  [FFT extern Rust 契約](#fft-extern-rust-契約)、次に
  [組込利用向け Cargo feature](#組込利用向け-cargo-feature)。
  [`embedded-poc/embedded-shared/src/esp_dsp_fft.rs`](https://github.com/jl1nie/mfsk-core/blob/main/embedded-poc/embedded-shared/src/esp_dsp_fft.rs)
  が写して使える実例。
- **C から、あるいは非 Rust の ESP-IDF プロジェクトから呼びたい** →
  [C / 非 Rust プロジェクトからの呼び出し](#c--非-rust-プロジェクトからの呼び出し)、
  ABI そのものは [`BINDINGS.md`](BINDINGS.ja.md)。
- **既存のコントローラを運用したい** →
  [`MANUAL_M5STACK_CORES3.md`](MANUAL_M5STACK_CORES3.ja.md)（CoreS3、
  USB 経由で無線機から音声を取る）または
  [`MANUAL_M5STICKS3.md`](MANUAL_M5STICKS3.ja.md)（StickS3 デモ機）。
- **組込アプリに手を入れたい** →
  [`embedded-poc/CLAUDE.md`](https://github.com/jl1nie/mfsk-core/blob/main/embedded-poc/CLAUDE.md)
  のボード横断ツールチェーン注記と LX6/LX7 比較表、次に各クレートの
  `CLAUDE.md`。
- **FT8 ではなく WSPR** → [WSPR on embedded](#wspr-on-embedded)。
- **ロードマップを追いたい** → [`ROADMAP.md`](../notes/ROADMAP.md) の
  Phase B-Stick（StickS3 デモ / 音響フォールバック）、Phase B-Core
  （CoreS3 メイン UAC コントローラ）、Phase E（WSPR）。
- **ホスト専用の利用** → [`LIBRARY.md`](LIBRARY.ja.md)。

## 目次

- [アーキテクチャ: f32 と固定小数点が 1 つのコードベースを共有する仕組み](#アーキテクチャ-f32-と固定小数点が-1-つのコードベースを共有する仕組み)
- [テスト対象](#テスト対象)
- [組込利用向け Cargo feature](#組込利用向け-cargo-feature)
- [FFT extern Rust 契約](#fft-extern-rust-契約)
- [per-symbol DFT: Goertzel](#per-symbol-dft-goertzel)
- [Q-format クイックリファレンス](#q-format-クイックリファレンス)
- [C / 非 Rust プロジェクトからの呼び出し](#c--非-rust-プロジェクトからの呼び出し)
- [出荷していないもの](#出荷していないもの)
- [性能ベンチマーク](#性能ベンチマーク)
- [Streaming RX pipeline アーキテクチャ](#streaming-rx-pipeline-アーキテクチャ)
- [バイナリフットプリント](#バイナリフットプリント-core2-リファレンスxtensa-esp32-elf-size--a)
- [プロトコル別の組込ステータス](#プロトコル別の組込ステータス)
- [WSPR on embedded](#wspr-on-embedded)

## アーキテクチャ: f32 と固定小数点が 1 つのコードベースを共有する仕組み

DSP / FEC パイプライン全体は **scalar trait** でパラメータ化されて
おり、同じソースが host 側 f32 パスと組込側整数パスのいずれにも
**コード重複なし** で compile される:

- [`engine::scalar::SpecScalar`] — spectrogram / DFT 出力 scalar
  (host は `f32`、embedded cs 格納は `Q14i16`)。
- [`engine::scalar::LlrScalar`] — wide-accumulator 付き LLR scalar。
  host は `f32`、`fixed-point-llr` 下では **`Q11i16` + i32 wide
  accumulator**。[^llrwidth]
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

[^llrwidth]: なぜ Q11i16 で、それより狭くないのか。LLR 型は 0.5.x まで
    `Q3i8` で、0.6.2 が拡げた。`qso3_busy.wav` に対し、host fixed-point
    + rustfft は f32 なら 16/18 取れたが `Q3i8` では **9/18** まで落ち、
    その ~0.875 の量子化ステップが recall 天井を決めていた — DSP 側では
    ない。`Q11i16`（~1/2048 LSB）は host ではその差を完全に埋める。実機
    では上乗せは1件（6/18 → 7）で、残る host との差は NSTEP-half・
    coarse-sync の簡略化・`fine_refine_pass1` 不在による。BP scratch は
    ~6 KB から ~12 KB に倍増するが、S3 / Core2 の内蔵 DRAM 予算内に
    収まる。`Q3i8` 型は比較経路用に `engine::scalar` に残置。（上記の
    sweep は 0.6.3 の OSD tightening 以前の計測で、その後 f32 host
    recall は CRC-luck phantom 3 件分下がって 16/18 → 13/18 になった。）

### fixed-point スイッチが現在配線されている範囲

| Component | Generic over | Fixed-point switch 配線済み? |
|---|---|---|
| LDPC BP NMS (`fec::ldpc::bp`) | `LlrScalar` | ✅ **`fixed-point-llr`** 経由 — #349 以降は独立した opt-in feature。LX7 では i16 ループは f32 の 0.85 倍（＝遅い） |
| LLR 計算 (`engine::llr`) | `SpecScalar` × `LlrScalar` | ✅ `fixed-point`（スペクトル）× `fixed-point-llr`（LLR 型） |
| BP scratch pool (`BpScratch<P, T>`) | `LdpcParams` × `LlrScalar` | ✅ — FT8 LDPC(174,91) と FST4/uvpacket LDPC(240,101) で機能 |
| FT8 spectrogram + DFT (`ft8::decode_block`) | `SpecScalar` × `AudioSample` | ✅ `fixed-point` 経由 |
| WSPR (`wspr::decode`, `wspr::ddc`) | — | ❌ — 組込でも host と同じ plain f32 を `fft-extern` 経由で実行。整数パスを一度も必要としていない。下記 [WSPR on embedded](#wspr-on-embedded) 参照 |
| **FT4** | (host f32 のみ) | ❌ — かつ必要が無い。FT4 は FST4 と同じく generic な `engine::pipeline` を通るので `fixed-point` はこの経路では no-op であり、そもそも LX7 では f32 より遅いと実測されている (issue #198)。**実機でビルドしデコードするところまで到達済み** — [プロトコル別の組込ステータス](#プロトコル別の組込ステータス) 参照 |
| **Q65 / JT9 / JT65** | (host f32 のみ) | ❌ — host 専用 (`fft-rustfft`、したがって `std`) で、組込パス自体がまだ無い |

つまり: **trait 基盤は protocol 非依存だが、組込ビルドで実際に整数
パスに切り替わるプロトコルは FT8 のみ。**

**整数パスに切り替わることは「チップ上で動く」の定義ではない。**
generic な `engine::pipeline` そのものが組込経路であり、FST4 は
`decode_block` を移植せずに実機へ到達した（issue #306）。FT4 も同じ道を
通った。`decode_block` が存在するのは FT8 自身の downsample chain が
192 000 点 FFT を要求するからで、特定の FFT を回避する手段であって
組込可能性の定義ではない。[プロトコル別の組込ステータス](#プロトコル別の組込ステータス)
を参照。

WSPR は全く別経路で組込に到達した（詳細は後述）。上の表だけ見ると
「FT8 しかチップ上で動かない」と読めてしまうのでここで一言添えておく
——`decode_block` のスペクトログラム/DFT 機構も整数パイプラインも
不要で、理由は S3 の dual-core の余裕が WSPR のずっと遅いケイデンス
（120 秒スロット vs FT8 の post-SlotEnd ~1.2 秒目標）を plain f32
のままで十分吸収するから。

## テスト対象

| Target | MCU | Backend | Status |
|---|---|---|---|
| **M5StickS3** | **ESP32-S3 (Xtensa LX7 dual-core, 240 MHz, 8 MB Octal PSRAM, ES8311 codec, ST7789P3 135×240 LCD, KEY1/KEY2)** | esp-dsp `_ae32_` asm (LX6/LX7 共通、scalar single-issue) — LX7 PIE `_aes3_` への移行は Phase D D1 で予定、[`PHASE_D_PIE_SIMD.md`](../notes/PHASE_D_PIE_SIMD.md) 参照 | **デモ / 音響 fallback コントローラ** (2026-05-17 pivot) — `embedded-poc/m5stack-s3-app/` (LCD UI + QSO FSM + BLE CI-V + 音響 mic + WiFi UDP log)。VBUS 源回路が無く（S3 のシリコン自体は host 可、基板が電源経路を配線していない — 外部 5 V を与えれば host 動作する旨の報告が issue #360 にある）バスパワーの USB デバイスを繋いでも給電できないため、本命の UAC コントローラ役は CoreS3 に移譲され、StickS3 は音響経路の実機検証 / デモ機としての位置付けに再定義された。 |
| **M5Stack Core2** | **ESP32-D0WD-V3** (Xtensa LX6, dual-core 240 MHz, single-issue f32 FPU, 16 MB flash, ~4 MB PSRAM) — `espflash board-info` 確認: `Chip type: esp32 (revision v3.1)` / `Features: WiFi, BT, Dual Core, 240MHz`。ESP32-S2 (LX7、single-core、BT 無し) や S3 では **ない**。 | esp-dsp ASM (`dsps_dotprod_s16_ae32`、`dsps_fft2r_*`) | **本番アプリ (`wav_sim` 専用)** — `embedded-poc/m5stack-core2-app/` が baked `wav_sim` 音源ループに対し同じ `decode_block` を LX6 上で走らせて `mfsk-app-shared` API を交差検証する役割。古典 ESP32 には USB peripheral が無いので mic / speaker / USB-Host 経路はこのボードでは扱わない — Core2 は共有 QSO FSM の LX6 second-board verifier。(独立した Core2 コンピュート bench `embedded-poc/m5stack-core2/` は #61 Phase 3 (0.6.3) で retired、wav_sim 経路はこの app crate に統合済み。) |
| ESP32-S3 compute bench | Xtensa LX7 | esp-dsp ASM | **タイミング回帰 bench** — `embedded-poc/m5stack-s3/`、缶詰 WAV 入力に対し `decode_block` を走らせ per-stage timing sweep。エンドユーザ向けではない。 |
| **M5Stack CoreS3** | ESP32-S3 LX7 + AXP2101 PMIC + AW9523B I/O expander (P1 の BUS_OUT_EN が VBUS boost 駆動) | esp-dsp `_ae32_` asm (Phase D D1 で `_aes3_` 化、S3-app と共通) | **本命の UAC コントローラ ターゲット** (Phase B-Core、2026-05-17 pivot) — `embedded-poc/m5stack-cores3-app/`。Phase 0-Core (bringup) + Phase 1-Core (AW9523B BUS_OUT_EN + UAC host) は commit `1a93c92` で出荷済み。M5StickS3 に無い VBUS 源回路を持つので（StickS3 側の制約はシリコンではなく基板 — issue #360）、IC-705 への USB-Host audio class はここで実装する。`docs/notes/ROADMAP.md` Phase B-Core 参照。 |

### その他のターゲット — 検証済 vs 願望

`fft-extern` 契約はターゲット移植可能になるよう **設計** されており、
`no_std` の feature セットは複数の非 Xtensa MCU に cross-build できる:

| Target | ビルド | FFT shim 提供 | HW テスト済 |
|---|---|---|---|
| `xtensa-esp32-espidf` | ✅ | ✅ esp-dsp (Core2) | ✅ qso1/2/3 sweep |
| `xtensa-esp32s3-espidf` | ✅ | ✅ esp-dsp (S3 bench + S3-app + CoreS3-app) | ✅ 実機オンエア |
| `thumbv8m.main-none-eabihf` (RP2350 Cortex-M33) | ✅ [^xbuild] | ❌ 候補: pico-sdk-rs 経由 CMSIS-DSP | ❌ |
| `riscv32imac-unknown-none-elf` (RP2350 Hazard3) | ✅ [^xbuild] | ❌ DSP ライブラリ無し、FFT は `microfft` | ❌ |
| `thumbv7em-none-eabihf` (Cortex-M4F / M7) | 未試行 | ❌ 候補: CMSIS-DSP `arm_*_q15` | ❌ |
| `thumbv6m-none-eabi` (Cortex-M0+ / RP2040) | 未試行 | ❌ scalar Rust のみ (DSP unit 無し) | ❌ |

[^xbuild]: RP2350 の2行は退役した `mfsk-ffi-ft8` に対して検証したもので、
    0.11.0 でそれが削除されて以降**再確認していない**。現在ビルドされる
    のは `mfsk-core` 自身であり、この feature セットは
    `scripts/pre-push-check.sh` のマトリクス（`alloc ft8 fft-extern` と
    `alloc ft8 fft-extern fixed-point`）が push のたびに検査している —
    ただしホストターゲットでのみ。この2つの ✅ は現在の実測ではなく
    「動くはず」と読むこと。

**ESP32 / ESP32-S3** (Xtensa LX6 / LX7) のみが実音源で end-to-end
回しているターゲット。それ以外については:

```sh
cargo build -p mfsk-core --release --no-default-features \
    --features alloc,ft8,fft-extern,fixed-point --target <T>
```

とし、FFT extern Rust シンボルは自前で供給する必要がある。具体的な
RP2040 / RP2350 / Cortex-M shim は将来作業として追跡。

`embedded-poc/embedded-shared/src/esp_dsp_fft.rs` がコピー元の
worked example。

## 組込利用向け Cargo feature

デフォルトは `std`、`parallel`、`fft-rustfft` を含む — これらを切って
組込ベースラインを選ぶ:

```toml
[dependencies]
mfsk-core = { version = "0.11", default-features = false, features = [
    "alloc",            # Vec / Box / String — decode 必須
    "ft8",              # FT8 protocol glue
    "fft-extern",       # 呼び出し側が FFT バックエンドを供給
    "fixed-point",      # u16 spec + i16 DFT（LLR/BP は #349 以降 f32。
                        # i16 の LLR/BP が要るなら "fixed-point-llr" を追加）
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

## C / 非 Rust プロジェクトからの呼び出し

**どちらにせよ Rust のシムが要る。** `mfsk-core` の組込 FFT 契約は
`extern "Rust"` シンボル（`mfsk_core_make_default_fft_planner`）であり、
これは `extern "C"` とは別の ABI で、C の翻訳単位からは定義できない。
したがって非 Rust の統合でも最低限 Rust の staticlib を1つリンクする
ことになる。

どのみち Rust を書くのなら、C ABI を経由するより `mfsk-core` を直接
呼ぶ方が単純である。**FT8 専用の組込 C ABI だった `mfsk-ffi-ft8` が
0.11.0 で退役した**のはそのためで、`embedded-poc/` の3ボードはいずれも
`mfsk-core` を直接呼んでいる。推奨する形はこれ:

```text
your-app/                      # esp-idf プロジェクトルート
├── main/main.c                # アプリケーション
├── components/mfsk/
│   ├── CMakeLists.txt         # IMPORTED static-lib コンポーネント
│   └── lib/libyourshim.a      # 下の Rust ビルド成果物
└── shim/                      # Rust staticlib
    ├── Cargo.toml             # mfsk-core に依存
    ├── .cargo/config.toml     # target = xtensa-esp32s3-espidf, panic=abort
    └── src/lib.rs             # 自分で定義する #[no_mangle] extern "C"
                               # エントリポイントと extern "Rust" FFT planner
```

Xtensa ツールチェーンでシムをビルドする:

```sh
source ~/export-esp.sh
RUSTFLAGS="-C panic=abort" cargo build --release \
    --target xtensa-esp32s3-espidf          # または xtensa-esp32-espidf
```

`-C panic=abort` は必須である。Rust の unwinding panic は `std` を
要するため。ESP-IDF プロジェクトでは通常 `.cargo/config.toml` に置く:

```toml
[target.xtensa-esp32s3-espidf]
rustflags = ["-C", "link-arg=-nostartfiles", "-C", "panic=abort"]
```

アーカイブはコンポーネントとして取り込む:

```cmake
idf_component_register(INCLUDE_DIRS "include"
                       REQUIRES espressif__esp-dsp)
add_library(mfsk_rust STATIC IMPORTED)
set_target_properties(mfsk_rust PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/lib/libyourshim.a)
target_link_libraries(${COMPONENT_LIB} INTERFACE mfsk_rust)
```

**手書きのエントリポイントではなく完全な C ABI が欲しい場合**、
`mfsk-ffi` はこれらのターゲットでも staticlib としてビルドでき、
全プロトコルを覆う — [`BINDINGS.md`](BINDINGS.ja.md) 参照。ただし
FT8 専用シムより大きく、session/stream の機構も引き込むので、単一
プロトコルの MCU ビルドなら手書きシムの方が普通は小さい。

> `embedded-poc/idf-component/README.md` は今も退役した
> `mfsk-ffi-ft8` を前提に書かれている。Rust シムが*なぜ*必要かの説明と
> CMake コンポーネントの形は今も正しいが、そこに出てくるクレート名・
> feature 名・シンボル名は正しくない。

### Streaming capture: I2S / USB Audio → 12 kHz

デコードは 12 kHz の1スロット丸ごとを取る。実際の受信機はそれを持って
おらず、コーデックのレートのまま小さな DMA チャンクを寄越す（I2S や
USB Audio Class 1/2 で典型的には 16 / 24 / 48 kHz）。選択肢は2つ:

- **Rust から**は `engine::dsp::resample` で 12 kHz に変換し、リングは
  アプリ側が持つ。`embedded-poc/embedded-shared/src/pipeline.rs` が
  実例で、その構成は[下](#streaming-rx-pipeline-アーキテクチャ)にある。
- **C から**は `mfsk-ffi` の `mfsk_stream_*` 群がまさにこのリングで、
  モード自身のスロット長からサイズが決まり、ライブラリ側は時計を一切
  読まない — [`BINDINGS.md` §2.5](BINDINGS.ja.md#25-ストリーミング取り込み) 参照。
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

組込パスが取りこぼす 11 局はチューニングの怠慢ではない。PASS1 を広げ
OSD を有効化する案は実測のうえ却下されている — 取りこぼす信号は
coarse_sync のランク 100 より下に居て、BP の努力ではなく反復減算を要する
から。そして FT8 のターンアラウンド予算はスロット全体ではなく
post-SlotEnd ~2 秒だから。数値は
[`DESIGN_RATIONALE.md` §5](../notes/DESIGN_RATIONALE.md#5-why-the-embedded-path-doesnt-widen-pass1-or-enable-osd)
にある。

### `qso3_busy.wav` の per-stage 分解

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

## プロトコル別の組込ステータス

| プロトコル | 実機への経路 | 状況 |
|---|---|---|
| **FT8** | `ft8::decode_block`、`fixed-point` 整数パイプライン | **オンエアでデコード中。** IC-705 の 40 m で1スロットあたり6〜8局（CoreS3、2026-08-23/24）。基準ターゲットであり、[性能ベンチマーク](#性能ベンチマーク)の数値はすべて FT8 |
| **FST4** | 汎用 `engine::pipeline` + `fft-extern` — **`decode_block` の移植なし** | **オンエアでデコード中**（CoreS3）。FST4-60 の実機時間は `no8_osd` で 13.6 s、締切重視の既定値で ~7 s 予算の約 1.95 倍 |
| **FT4** | 汎用 `engine::pipeline`、ホスト f32（LX7 では `fixed-point` の方が*遅かった*、#198） | **オンエアでデコード中**（CoreS3） |
| **WSPR** | `fft-extern` 経由のホスト `wspr::decode` f32 と `wspr::ddc` | **オンエアでデコード中。** `slot 1 src=uac decoded 1 station(s)`。110 s の締切に対し 82.8〜90.1 s で decode 完了 |
| **Q65 / JT9 / JT65** | — | **ホスト専用。** `rustfft` を直接呼ぶため `fft-rustfft` を、したがって `std` を引く。組込パスはまだ無い |

**FST4 は `decode_block` を移植せずに実機へ到達した**（issue #306）。
FT4 も同じ道を通った。`decode_block` があるのは FT8 自身の
ダウンサンプル鎖が 192 000 点 FFT を要するからで、特定の FFT を回避する
手段であって「チップで動く」の定義ではない。長く逆に思われていたので
明記しておくと、`fst4` は**ホスト専用 feature ではない** —
`alloc,fst4,fft-extern` で型検査が通る。

### FST4 と FT4 のチューニング履歴

FST4 と FT4 を組込予算に収めるには、それぞれ 18 回と 9 回の実測が
必要だった — 時間が実際どこへ行くのか、どの OSD レバーが本物か、
`no8_osd` が何と何を交換しているのか、どの「回帰」が測定の側の産物
だったのか。これらはリファレンスではなく測定日誌なので、他のスイープと
同じ場所に置いてある:

- [`FST4_BENCHMARK.md` §17](../notes/FST4_BENCHMARK.md) — 実機での18回の
  試行と、`decode_rung_major` の `offsets` がデコーダ側ではなく呼び出し側の
  判断になった経緯。
- [`FT4_BENCHMARK.md` §50](../notes/FT4_BENCHMARK.md) — そもそもビルドを
  通すまでに要したこと、ボトルネックが統計的でなく構造的である理由、
  候補数の過剰の修正。

### 実機 UAC ブリングアップ

issue [#163](https://github.com/jl1nie/mfsk-core/issues/163) — USB Audio
Class キャプチャパスの IC-705 実機確認 — は **2026-08-23 に解決**した。
WiFi を張ったまま10分間途切れなく、125 MB、エラーゼロ
（`embedded-poc/m5stack-cores3-app/logs/uac_stream_2026-08-23.log`）。

ブリングアップのチェックリストは、次にこのパスが壊れたときのために
残してある: [`UAC_BRINGUP_CORES3.md`](../notes/UAC_BRINGUP_CORES3.md)。
ボードに触る前に `embedded-poc/CLAUDE.md` の「USB host VBUS on CoreS3」と
「Stacks, heaps, and the space between them」を読むこと。

## WSPR on embedded

上記とは構造的に別の、2つ目の組込ストーリー — WSPR は `decode_block`
も `fixed-point` も一切通らない。host と
同じ `wspr::decode` の f32 パスを `fft-extern` 経由でそのまま
device 上で走らせ、新規追加は 1 つだけ: `wspr::ddc`、streaming
down-converter。参照デコーダのスロット全体 FFT チャネライザ
(`wspr::baseband::decimate_to_baseband`) は S3 上で全く動かせない
— 11.25 MiB の `Complex<f32>` バッファを 1,474,560 点 FFT で必要と
し、2 の冪でもなく `esp-dsp` の 8,192 上限も超える。`wspr::ddc` は
1500 Hz でミックス (ちょうど Fs/8 なので 8 要素テーブル、毎サンプル
の三角関数なし)、単段 FIR ローパス、32 サンプルごとの間引き —
状態は約 25 KB、スロット長に非依存。参照チャネライザとの等価性は
仮定ではなく実測: golden 9/9、AWGN sweep は全 SNR で 500 trial/セル
に対し 1 trial 差以内、phantom は双方 0。

Cargo feature ([組込利用向け Cargo feature](#組込利用向け-cargo-feature)
の一般形も参照):

| Feature | 変わるもの | 既定 |
|---|---|---|
| `wspr` | WSPR プロトコルの配線。単体では TX-only 組込ビーコンビルド — FFT backend 不要。 | off |
| `wspr-ddc` | 参照のスロット全体チャネライザではなく streaming down-converter を選択。host は正確な参照実装のまま。組込は選択の余地なし — 参照実装はそこでは動かない。 | off (組込の `wspr-bench` が on にする) |
| `wspr-fano-cap-fast` | Fano デコーダの cycle budget を 5,000 cycles/bit に制限 (`wsprd` 自身の既定は 10,000、host はこちらを使用) — 120 秒スロットの締切が必要とする wall-clock と引き換えに床の感度を払う。 | off |
| `wspr-pass2-topn` | pass-2 候補を refined sync でランク付けし上位 2 件のみ deep-process (dual-core の分割と一致)、全 survivor に対するフルラダーの代わりに。 | off |

Device (M5Stack CoreS3、WiFi associate 継続、dual-core): 4 スロット
連続の定常状態でデコードは 110 秒締切 (120 秒スロット − spot
アップロード予約 10 秒) に対し 82.8〜90.1 秒、全スロットで golden
9/9 維持。ダウンコンバートは実際のデューティサイクルで前スロットの
デコードと並走し (両者は重なる — 1 スロットの ~114 秒キャプチャ
窓は 1 回のデコードより長い)、18.5〜24.1 秒かかる。うまくいかな
かった試みも含めた計測の全記録 (再挑戦を防ぐため削除せず保持) は
[`docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`](../notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md)、
これらの数字が出てくる実行可能なベンチは
`embedded-poc/m5stack-cores3-app/src/bin/wspr_bench.rs`。

**このバイナリでは未検証**: 実音声キャプチャ。上記は全て WAV 給餌 /
合成ベースバンドに対する計測。両ラインが依存していた UAC 実機検証
[#163](https://github.com/jl1nie/mfsk-core/issues/163) は
**2026-08-23 に完了** — FT8 controller 側で、WiFi 接続を保ったまま
192,512 B/s・エラー 0 で 10 分連続キャプチャを確認した。`wspr_app` は
同じ `uac.rs` を共有するので経路自体は実証済みで、未実施なのは
*このバイナリ*を実機の無線機に繋ぐことのみ。キャプチャ窓は
USB ストリーム開始位置ではなく UTC の偶数分グリッド上で開くように
なり、各スポットはデコード後の時計読みではなく「受信した窓の開始
時刻」を持つ (#313 item 1、2026-09-07) — ソフトウェアのみの変更で、
本段落の他の項目と同様に実機未検証。
`mfsk_app_shared::wsprnet` (wsprnet.org へのスポット送信、WSJT-X
自身の `Network/wsprnet.cpp` から移植) は実装済みで既定 off。
`SpotSink::Http` パスは実装済みだが実エンドポイントに対しては未検証。

