# FT4 感度ベンチマーク — 環境セットアップ

クリーンチェックアウトから FT4 の AWGN/フェージング SNR スイープ
(`tests/ft4_sweep.rs`) を再現する手順。[#146](https://github.com/jl1nie/mfsk-core/issues/146)
向けに構築した FST4 の手法 ([`FST4_BENCHMARK.ja.md`](FST4_BENCHMARK.ja.md))
をそのまま踏襲したもので、
[#72](https://github.com/jl1nie/mfsk-core/issues/72) の
`DecodeStrictness` 再較正に使う。FT4 は現状 `DecodeStrictness::Normal`
決め打ちで、閾値の数値自体は FT8 の較正値をコピペしただけのまま
一度も FT4 用に調整されていない。

パイプライン全体は: **WSJT-X Fortran ソース → `ft4sim` バイナリ →
ディスク上の WAV コーパス → `cargo test --ignored` が WAV を読んで
recall テーブルを出力**。ビルド時に一度だけ WSJT-X ソースに触れる以外、
テスト実行時にネットワークは不要。

## 1. 前提パッケージ

| 要件 | Ubuntu/Debian パッケージ | 用途 |
|---|---|---|
| `gfortran` | `gfortran` | `ft4sim.f90` + 依存ライブラリのコンパイル |
| `gcc` / `g++` | `build-essential` | `gran.c`, `sgran.c`, `init_random_seed.c`, `crc14.cpp` のコンパイル |
| `libboost-dev` | `libboost-dev` | `crc14.cpp` が `boost::augmented_crc` を使用 |
| FFTW (単精度) | `libfftw3-dev` | `-lfftw3f` でリンク |
| WSJT-X ソースツリー | — (下記参照) | `lib/ft4/*.f90` を提供 |

```sh
sudo apt-get install gfortran build-essential libboost-dev libfftw3-dev
```

**WSJT-X ソース。** `scripts/build_ft4sim.sh` は `lib/` サブツリーのみ
必要（Qt ビルドや `cmake` は不要）— `ft4/ft4sim.f90` シミュレータが
入っている WSJT-X チェックアウトならどれでも動く:

```sh
git clone https://github.com/jl1nie/WSJT-X.git ../WSJT-X
```

リポジトリルートの `CLAUDE.md` のテスト用パス規則に従い、このチェック
アウトへの絶対パスを決め打ちしないこと — `build_ft4sim.sh` は明示引数
として受け取り、デフォルトは `../WSJT-X`（本リポジトリの兄弟ディレクトリ）
という規約だけを使う。

## 2. `ft4sim` をビルド

```sh
scripts/build_ft4sim.sh [/path/to/WSJT-X] [out-dir]
# デフォルト: WSJT-X-dir = ../WSJT-X（本リポジトリの兄弟）, out-dir = target/ft4sim/
```

`target/ft4sim/ft4sim` を生成。FT4 は FT8 の `encode174_91`
(LDPC(174,91)) をそのまま再利用する（`genft4.f90` が直接呼ぶ）ため、
`build_fst4sim.sh` が既にコンパイルしている共有 lib サブツリー
（`wavhdr`、`packjt77`、`watterson`、`fftw3mod`/`four2a`）に FT4 固有の
コンパイル単位 2 つ（`genft4.f90`、`gen_ft4wave.f90`）を足すだけで済む。
加えて `crc14.cpp`（Boost ベースの CRC-14、C++）と `sgran.c` /
`init_random_seed.c`（乱数シード）が必要 — `fst4sim` はこれらを使わない
（FST4 は CRC-24 を使い `sgran` も呼ばない）。

ビルドの動作確認:

```sh
target/ft4sim/ft4sim "CQ JL1NIE PM95" 1500 0.0 0.0 0.0 1 -15
ls 000000_*.wav   # -15 dB の 7.5 秒 AWGN トライアル 1 件
```

（`ft4sim` の CLI には T/R 周期引数がない — `fst4sim` と違い FT4 に
サブモードはないため: `message f0 DT fdop del nfiles snr` の位置引数 7 個。）

## 3. WAV コーパスの生成

```sh
scripts/gen_ft4_sweep_wavs.sh [ft4sim-path] [out-dir]
# デフォルト: ft4sim-path = target/ft4sim/ft4sim
#           out-dir      = embedded-poc/assets/ft4_sweep/
```

4 種のチャネル条件（`awgn`、`ccir_good`、`ccir_moderate`、`ccir_poor`）
× `SNRS` グリッド × `TRIALS` 回（デフォルト 20）。
`gen_fst4_sweep_wavs.sh` と同じくインクリメンタル — グリッドを広げて
再実行しても不足分だけ生成する。

デフォルトの `SNRS` グリッド（-5〜-23 dB）は、FT4 の公称 WSJT-X AWGN
閾値である約 **-17.5 dB**（2500 Hz 基準帯域幅、FT8 の -21 dB と比較 —
FT4 は 7.5 秒という短いスロットと少ない FEC インターリーブと引き換えに
感度を犠牲にしている）を挟むように設定。実測の 50% クロス点が想定より
弱ければ、FST4 #146 のグリッド打ち切り (censoring) の教訓に従いさらに
深いグリッドへ拡張すること。

## 4. スイープの実行

```sh
MFSK_FT4_SWEEP_DIR=embedded-poc/assets/ft4_sweep \
  cargo test --test ft4_sweep --release \
  --features ft4,fft-rustfft,parallel,uvpacket \
  -- --ignored --nocapture
```

`uvpacket` が必要なのは `mod common` 経由で取り込まれる
`tests/common/channel.rs` が無条件に `mfsk_core::uvpacket` を import
するため — FT4 自体とは無関係。

出力はチャネル × SNR セルごとの recall テーブルのみ（pass/fail の
assertion はなし、あくまで測定ツール）。

### 実測値 2026-07-18（このコーパス/シード、`DecodeStrictness::Normal`）

隣接グリッド点間の線形補間による 50% クロス点:

| チャネル | 50% クロス点(概算) |
|---|---:|
| AWGN | ≈ -15.5 dB |
| CCIR good | ≈ -14.7 dB |
| CCIR moderate | ≈ -14.3 dB |
| CCIR poor | ≈ -13.7 dB |

いずれも公称値 -17.5 dB より約 2 dB 弱い。まだ根本原因は特定していない
— `ft4sim` の SNR 規約の問題、実装側の本物のギャップ（#146 と同種の
調査候補）、あるいは公称値自体が別の指標を指している可能性のいずれか。
今回はこれ以上追わずここに記録するに留める。今後着手する際は
`FST4_BENCHMARK.ja.md` 6 節の「直す前に診断する」順序に従うこと。

## 5. `DecodeStrictness` の probe (issue #72)

同ファイル内の `ft4_strictness_probe` は `engine::pipeline::decode_frame`
を `Strict` / `Normal` / `Deep` それぞれで直接叩き、上記スイープで
判明した部分 recall のセルについて「golden recall」（実際に送信された
メッセージと一致）と「any-msg recall」（golden かどうか問わず CRC を
通った decode 全て — このテストでは各トライアルの真の内容が既知なので
false-accept 膨張の代理指標になる）の両方を出す。

```sh
MFSK_FT4_SWEEP_DIR=embedded-poc/assets/ft4_sweep \
  cargo test --test ft4_sweep --release --features ft4,fft-rustfft,parallel,uvpacket \
  ft4_strictness_probe -- --ignored --nocapture
```

**2026-07-18 実測の知見:** 「未較正のコピペだからおそらく効かない」
という想定に反し、このノブは FT4 に対して no-op ではない。AWGN では
効果は小さいが、フェージング下では無視できない — 例えば
`ccir_poor` -13 dB: Strict 9/20 → Normal 14/20 → Deep 16/20 (golden
recall)。ただし `Deep` はいくつかのセルで "any-msg" が "golden" より
速く増える（`ccir_moderate` -16 dB: golden 1→2、any-msg 1→4）—
Deep の追加 decode の一部は false-accept であり、本物の感度向上ではない。
これは FT8 のオリジナル較正コメントが述べていた
sensitivity/false-positive のトレードオフそのもので、FT4 では今回まで
未検証だった。

まとめ: 現状デフォルトの `Normal` は既に妥当な落とし所にある —
実利用上重要なフェージングセル（`ccir_poor` -13 dB では Deep の
7-decode 分の gain のうち 5 を確保）で `Strict → Deep` の実質的な
gain の大半を捉えつつ、`Deep` の最悪の false-accept 増加は避けている。
今回で未解決なのは、FT8 からコピペした数値ではなく FT4 固有に較正した
閾値なら、`Deep` の FP コストなしに `Normal` を上回れるか、という点 —
これには既存の 3 段階の named level だけでなく `osd_max_errors` /
`osd_score_min` の数値そのものを振る sweep が必要。

## 6. 数値の再較正 (issue #72, 2026-07-18)

`engine::pipeline::process_candidate_basic` 内の
`strictness.osd_score_min()` / `strictness.osd_max_errors()` 呼び出し
箇所を追跡したところ、両方とも `!is_fst4` の条件下でのみ実行される
ことが判明 — つまり **FST4 はこれらの値を一切使わない**（#146 の
fix）。実際には `engine::pipeline::DecodeStrictness` の数値は FT4 専用
になっている。これは `Normal` の再較正が FST4 の（別途調整済みの）
recall に対して回帰リスクゼロであることを意味し、単に特性を測定する
だけでなく実際に数値を動かす道を開いた。

手動で sweep: 候補値を編集 → `ft4_strictness_probe` を部分 recall の
8 セルに対して再実行 → any-msg が golden より速く増えない（= 新たな
false-accept が出ない）範囲で golden recall が上がっていれば採用、
そうでなければ破棄。

- `osd_score_min`: `2.2 → 1.8`。`2.0` で 3 セル改善（新規 FP ほぼ
  なし）、`1.8` でさらに 2 セル改善。`1.6` まで下げると golden の
  伸びはゼロで false-accept だけ増えた — `1.8` が上限。
- `osd_max_errors`: `(depth3, depth4, other) = (26, 30, 29) →
  (28, 30, 31)`。`(29, 31, 32)` は同一結果（それ以上動かす利点なし）、
  `(30, 33, 33)` は golden 増加ゼロで false-accept が 2 件増加 —
  `(28, 30, 31)` が上限。

フルスイープへの効果（Normal、変更前→変更後）:

| チャネル | -17 dB | -16 dB | -15 dB | -14 dB | -13 dB |
|---|---:|---:|---:|---:|---:|
| AWGN | 0%→10% | 20%→20% | 85%→85% | 95%→95% | 95%→95% |
| CCIR good | 5%→10% | 20%→20% | 40%→45% | 90%→100% | 90%→90% |
| CCIR moderate | 5%→10% | 5%→5% | 30%→40% | 70%→70% | 95%→95% |
| CCIR poor | 0%→0% | 10%→15% | 20%→25% | 45%→50% | 70%→75% |

小幅かつ単調な改善で、どこにも regression なし（ゲートを緩めるだけ
なので recall は維持か増加のみ）— dB スケールの跳躍ではなく、実質的
だが漸進的な gain。`engine::pipeline::DecodeStrictness::{osd_max_errors,
osd_score_min}` に反映済み。`Strict`/`Deep` の数値は未変更（現状どの
呼び出し元も使っていない、FT8 からの未検証コピーのまま）。

## 7. Coherent full-slot Δt search (`ft4_sync_search`, 2026-07-18)

上記の `DecodeStrictness` チューニングとは別に、`sync4d.f90`/
`ft4_decode.f90` に対する WSJT-X 忠実性監査で判明: `engine::sync::coarse_sync`
の広域(±2.5s) Δt 探索は **non-coherent**（パワースペクトラムのbin）
なのに対し、WSJT-X 本家の広域 Δt 探索（`isync=1`、3セグメントで
各350〜450サンプル）は **coherent**（複素Costas相関）。
`tests/ft4_coherent_wide_search_diag.rs` で実測確認: CCIR フェージング
下では `coarse_sync` の non-coherent な選択が真のピークから0.5〜2.4秒
もずれることがあり、旧来のローカル `sync2d_refine`（±20 downsampled
samples ≈ ±30ms）の範囲外——それでも真のピークの coherent スコアの
方が一貫して高い。AWGN では影響なし（non-coherent のずれは16ms以内
に収まり、旧refineの範囲内）。

修正: `engine::sync2d::ft4_sync_search`、WSJT-Xの`isync=1`/`isync=2`
ループを模した coherent full-slot Δt 探索（±12Hz/3Hz の coarse ×
絶対座標`[-344, 1012]` downsampled-sample窓・step4、その後±4Hz/1Hz ×
±5サンプルの fine）で `sync2d_refine`/`Sync2dConfig::for_ft4` を置換
（FST4は#146で既に同等の`fst4_sync_search`を獲得済み）。

**最初の実装はネットでregressionでした**（クリーンなAWGNですらrecall
低下）— 実際に起きた罠として記録しておく価値がある: Δt探索を広げると、
**真の信号自身の周波数近傍にある`coarse_sync`候補が、より多く独立に
正当で自己無矛盾なCostasロックに到達してしまう**（ノイズではない —
`sync_quality`も、独自に実装したWSJT-X `nsync_qual`相当のbit-metric
ゲートも、どちらも高スコアを出すことを確認済み）。試して破棄した対策:
(a) dedup時にhard_errors最小の重複を残す — 逆効果、hard_errorsは位置
精度と相関しない。(b) `nsync_qual`相当の追加ゲート — WSJT-X自身の
ゲート設計はここには適用できないと判明。WSJT-Xの「1周波数ピークに
つき1候補」というシンプルな構造は、そもそもこの種の近接重複候補を
生まないので、そのゲートは元々こういう候補を裁定するために作られた
ものではなかった。

**本当の根本原因**: `DecodeResult.freq_hz`が精密化前の`cand.freq_hz`
から設定されており、`refined.freq_hz`ではなかった——潜在バグ。以前
無害だったのは、旧来の狭いローカルrefineが元々真値に近い候補しか
補正しなかったから。広域探索で遠く離れた候補も成功するようになり、
それぞれが古い`cand.freq_hz`をそのまま報告し続けた——recallの問題
ではなくreport層のバグだった。`process_candidate_basic`内の3箇所の
`DecodeResult`構築を全て`refined.freq_hz`に修正。dedupの
sync_score-tie-breakは残した（正しく報告されるようになった今では
ほぼ見た目上の意味しか持たないが、精密化が完全に収束しない稀なケース
のために有効）。

### 実測値 2026-07-18（6節の`DecodeStrictness`のみのbaseline比）

| チャネル | -17 dB | -16 dB | -15 dB | -14 dB | -13 dB |
|---|---:|---:|---:|---:|---:|
| AWGN | 10%→20% | 20%→30% | 85%→100% | 95%→95% | 95%→100% |
| CCIR good | 10%→20% | 20%→40% | 45%→65% | 100%→100% | 90%→95% |
| CCIR moderate | 10%→10% | 5%→5% | 40%→50% | 70%→80% | 95%→95% |
| CCIR poor | 0%→0% | 15%→20% | 25%→35% | 50%→55% | 75%→80% |

50%クロス点の改善（線形補間）: AWGN ≈+0.2dB、CCIR good ≈+0.7dB、
CCIR moderate ≈+0.3dB、CCIR poor ≈+0.25dB。CCIR poor -5dB
（100%→95%、1トライアル分）以外に regression なし——20トライアル/
セルのサンプリングノイズの範囲内。フル非ignoredテストスイート
（`cargo test --release --features full`）と
`ft4_wsjtx_sample_recall_vs_golden`で全green確認済み。
