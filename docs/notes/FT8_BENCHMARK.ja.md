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
本番 `decode_frame` が（FT4/FST4 が使う汎用 `engine::pipeline` 経路
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

## 実験ログ

日付つきの節（§6 以降: 試したこと、測ったこと、棄却したこと）は [`docs/historical/FT8_BENCHMARK_LOG.ja.md`](../historical/FT8_BENCHMARK_LOG.ja.md) にあります（節番号は同じ）。現在の結果: [`BENCHMARKS.md`](BENCHMARKS.md)、[`UPSTREAM_EVALUATION.md`](UPSTREAM_EVALUATION.md)、`sweep-baseline.json`。

- 6. 旧 CI 「ft8 characterization」スイート — 2026-07-18 削除
- 7. CCIR moderate/poor のフェージング差 — 診断して解消 (issue #72 follow-up, 2026-07-18)
- 8. SIC LPF 窓のバグ — JTDX 17/18 → 18/18 で解消 (issue #180, 2026-07-25)
- 9. AWGN/CCIR スイープ再計測、`DecodeDepth` 再設計はスイープに無影響と確認 (issue #182 follow-up, 2026-07-26)
- 10. CCIR moderate/poor の「WSJT-X 公称値なし」を本物の `jt9 -8 -d3` ground truth で置き換え (2026-07-26)
- 11. issue #190 の真因: 数値的な差ではなく `jt9` CLI の暗黙の CQ-AP (2026-07-26)
- 12. 0.11.0 タグ前の tier-C 再計測 — §9 の数値は最大 1.1 dB 古い (2026-09-20)
- 13. WSJT-X 3.2.0-rc1 の本物の `jt9` は FT8 で最大 0.84 dB 高感度、FT4 は動かず (2026-09-24)
- 14. WSJT-X 3.x の移植後: 何が動き、何が閉じなかったか (#439, PR #451, 2026-09-25)
- 15. a-priori パスを `ft8b.f90` と比べる: どの LLR 変種か、何がハードエラーを縛るか (#456, 2026-09-26)
- 16. a7 と a8 のリストデコーダを `jt9` 3.2.0-rc1 と比べる (#464, 2026-09-26)
