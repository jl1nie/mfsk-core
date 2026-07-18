# FT8 感度ベンチマーク — 環境セットアップ

クリーンチェックアウトから FT8 の AWGN/フェージング SNR スイープ
(`tests/ft8_sweep.rs`) を再現する手順。[`FT4_BENCHMARK.ja.md`](FT4_BENCHMARK.ja.md)
・[`FST4_BENCHMARK.ja.md`](FST4_BENCHMARK.ja.md) と同じ **WSJT-X Fortran
ソース → simN シミュレータ → WAV コーパス → `cargo test --ignored`**
というパイプラインを `ft8sim` に適用したもの。

これは既存 CI の「ft8 characterization」スイート
（`ft8_decode_block_snr_sweep` 等、`.github/workflows/ci.yml` 参照）とは
別物 — あちらは自前の LCG ノイズ生成器で信号を合成しており、WSJT-X 本家
のground truthとは無関係。`ft8sim` は WSJT-X 自身の Fortran シミュレータ
（`fst4sim`/`ft4sim` と同じ系統）なので、そこから生成したコーパスは
自己無矛盾チェックではなく真の Watterson フェージングリファレンスになる。

## 1. 前提パッケージ / 2. `ft8sim` をビルド

パッケージ・手順とも [`FT4_BENCHMARK.ja.md`](FT4_BENCHMARK.ja.md) の
1-2 節と同一 — `ft8sim` は FT4 と全く同じ共有 lib サブツリー
（LDPC(174,91)、CRC-14、`sgran` 乱数シードを共有）を再利用する:

```sh
sudo apt-get install gfortran build-essential libboost-dev libfftw3-dev
scripts/build_ft8sim.sh [/path/to/WSJT-X] [out-dir]
# デフォルト: WSJT-X-dir = ../WSJT-X, out-dir = target/ft8sim/
```

動作確認:

```sh
target/ft8sim/ft8sim "CQ JL1NIE PM95" 1500 0.0 0.0 0.0 1 -15
ls 000000_*.wav
```

（`ft4sim` と同じ 7 引数 CLI: `message f0 DT fdop del nfiles snr` —
T/R 周期引数はなし、FT8 にもサブモードはない。）

## 3. WAV コーパスの生成

```sh
scripts/gen_ft8_sweep_wavs.sh [ft8sim-path] [out-dir]
# デフォルト: ft8sim-path = target/ft8sim/ft8sim
#           out-dir     = embedded-poc/assets/ft8_sweep/
```

同じく 4 チャネル × グリッド × `TRIALS` 構成。デフォルトの `SNRS`
グリッド（-5〜-26 dB）は FT8 の公称 WSJT-X AWGN 閾値である約
**-20〜-21 dB**（2500 Hz 基準帯域幅）を挟む。

## 4. スイープの実行

```sh
MFSK_FT8_SWEEP_DIR=embedded-poc/assets/ft8_sweep \
  cargo test --test ft8_sweep --release \
  --features ft8,fft-rustfft,parallel,uvpacket \
  -- --ignored --nocapture
```

### 実測値 2026-07-18（このコーパス/シード）

線形補間による 50% クロス点:

| チャネル | 50% クロス点(概算) |
|---|---:|
| AWGN | ≈ -20.4 dB |
| CCIR good | ≈ -20.0 dB |
| CCIR moderate | ≈ -18.3 dB |
| CCIR poor | ≈ -18.2 dB |

AWGN と CCIR good は公称値 -20/-21 dB の約 1 dB 以内に収まっており、
FT4 が公称値 -17.5 dB に対して見せた約 2 dB のギャップ
（`FT4_BENCHMARK.ja.md` 参照）よりかなり良い一致。これは FT8 の
本番 `decode_frame` が（FT4/FST4 が使う汎用 `core::pipeline` 経路
ではなく）#48 統合後の WSJT-X 忠実パイプライン `decode_block` を
通っていることと整合する。CCIR moderate/poor はより大きい約
2〜2.5 dB のギャップを見せており、未調査 — フェージング下の感度が
優先課題になった場合、`FST4_BENCHMARK.ja.md` 6 節の「直す前に診断する」
アプローチの候補になる。

## 5. `DecodeStrictness` probe はここにはない

FT4/FST4 のスイープと異なり、今回は strictness 較正 probe を含めて
いない。FT8 の本番経路は `process_candidate`（`pub` ではない）を、
FT4/FST4 が共有する未較正コピー（issue #72）とは別の、既に較正済みの
`ft8::decode::DecodeStrictness`（`src/ft8/decode.rs` の
"Calibrated from real WAV bench 2026-04-07" というコメント参照）と共に
呼んでいる。外部テストからこれを振る public なフックがない。今回の
スイープが FT8 に初めてもたらすのは、新しい較正対象ではなく、既存の
較正値を検証するための体系的な Watterson フェージングコーパスである。

## 6. 旧 CI 「ft8 characterization」スイート — 2026-07-18 削除

`.github/workflows/ci.yml` には push-only の matrix entry
`ft8 characterization` があり、`ft8_coarse_sync_concurrent`、
`ft8_decode_block_coarse_diag`、`ft8_decode_block_depth_sweep`、
`ft8_decode_block_pass1_sweep`、`ft8_decode_block_snr_sweep`、
`ft8_qso3_coarse_sync_probe`（`ft8/**` push 毎に約10分）を含んでいた。
6件全部を監査した上で、matrix entry ごと削除:

- 6件全部が `println!` のみの診断で **assertion が一切ない** —
  数値が何であれ絶対に fail しないので、かかっていた時間に見合う
  regression 検知能力がゼロだった。
- `ft8_coarse_sync_concurrent` は `fixed-point` feature 必須だが、CI の
  `full` feature set には含まれていない — この cleanup 以前から
  静かに 0 テスト実行だった。
- `ft8_decode_block_snr_sweep` が実際に自前 LCG ノイズ生成器で AWGN を
  合成していたもの（フェージングモデルなし）で、外部の ground truth
  なしに自分たちの decoder 2 つを比較するだけ — 「golden でない
  simulator で検証している」という指摘に最も直接該当する。
- 残り 4 件は実録音（`REAL_QSO_WAVS`、`embedded-poc/assets/` に
  チェックイン済み）を使っていたので合成データではなかったが、
  それでも assertion はなかった。

FT8 の recall regression 検知は既に別の場所でカバーされており、
代替は不要だった: `ft8_qso3_apoff_recall` / `ft8_qso3_apon_recall` は
hard-assertion で常時実行（`default` suite、`#[ignore]` されていない）
— これが実際に recall regression を検知するテスト。

別の `ft8 recall` matrix tier（`ft8_decode_block_real_qso`、
`ft8_reference_suite_recall`）も同じ問題を抱えていることが判明し、
同日中に対応済み: `ft8_reference_suite_recall`（PASS1_LIMIT/max_cand
の組込チューニング用 config sweep、情報提供のみ）は削除。
`ft8_decode_block_real_qso` は hard-assertion floor テストに変換
（embedded ship config vs host `decode_frame` の truth を
`qso1`/`qso2`/`qso3_busy` で比較 — `qso1`/`qso2` は WSJT-X golden が
なく他に一切テストされていなかった）、`#[ignore]` も外したので
matrix tier 自体を削除 — 今は `default` で実行される。
