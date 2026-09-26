# FT4 感度ベンチマーク — 環境セットアップ

クリーンチェックアウトから、任意のマシンで FT4 の AWGN/フェージング SNR
スイープ (`tests/ft4_sweep.rs`) を再現する手順。これは
[`FST4_BENCHMARK.md`](FST4_BENCHMARK.ja.md) の FST4 の手法
([#146](https://github.com/jl1nie/mfsk-core/issues/146) 向けに構築)
を踏襲したもので、
[#72](https://github.com/jl1nie/mfsk-core/issues/72) で追跡している
`DecodeStrictness` の再較正に使う — FT4 は現状 `DecodeStrictness::Normal`
を決め打ちしており、閾値の数値は FT8 の較正値をコピペしたままで、FT4
自身の SNR/OSD の挙動に合わせて調整し直されたことは一度もない。

パイプライン全体は: **WSJT-X Fortran ソース → `ft4sim` バイナリ → ディスク上の
WAV コーパス → `cargo test --ignored` が WAV を読んで recall テーブルを
出力**。ここでテスト実行時にネットワークアクセスは不要 — 一度きりのビルド
ステップだけが WSJT-X ソースに触れる。

## 1. 前提条件

| 要件 | Ubuntu/Debian パッケージ | 用途 |
|---|---|---|
| `gfortran` | `gfortran` | `ft4sim.f90` と補助 lib のコンパイル |
| `gcc` / `g++` | `build-essential` | `gran.c`, `sgran.c`, `init_random_seed.c`, `crc14.cpp` のコンパイル |
| `libboost-dev` | `libboost-dev` | `crc14.cpp` が `boost::augmented_crc` を使用 |
| FFTW 単精度 | `libfftw3-dev` | `-lfftw3f` としてリンク |
| WSJT-X ソースツリー | — (下記参照) | `lib/ft4/*.f90` を提供 |

```sh
sudo apt-get install gfortran build-essential libboost-dev libfftw3-dev
```

**WSJT-X ソース。** `scripts/build_ft4sim.sh` が必要とするのは `lib/`
サブツリーだけ (Qt ビルドも `cmake` ステップも不要) — `ft4/ft4sim.f90`
シミュレータを含む WSJT-X のチェックアウトならどれでも動く。upstream の
ツリーも含む:

```sh
git clone https://github.com/jl1nie/WSJT-X.git ../WSJT-X
```

リポジトリルートの `CLAUDE.md` にあるテスト fixture パスの規則に従い、
このチェックアウトへの絶対パスは決して決め打ちしない — `build_ft4sim.sh`
はこれを明示的な引数として受け取る (デフォルトとしてのみ、リポジトリ
ルートの兄弟ディレクトリ `../WSJT-X` という規約にフォールバックする)。

## 2. `ft4sim` のビルド

```sh
scripts/build_ft4sim.sh [/path/to/WSJT-X] [out-dir]
# defaults: WSJT-X-dir = ../WSJT-X (sibling of this repo), out-dir = target/ft4sim/
```

`target/ft4sim/ft4sim` を生成する。FT4 は FT8 の `encode174_91`
LDPC(174,91) コーデックを再利用する (`genft4.f90` が直接呼ぶ) ため、
このビルドが `build_fst4sim.sh` が既にコンパイルしているのと同じ共有 lib
サブツリー (`wavhdr`、`packjt77`、`watterson`、`fftw3mod`/`four2a`) の上に
足すのは、FT4 固有のコンパイル単位 2 つ (`genft4.f90`、`gen_ft4wave.f90`)
だけである。加えて `crc14.cpp` (Boost ベースの CRC-14、C++) と `sgran.c` /
`init_random_seed.c` (乱数シード) が必要になる — `fst4sim` はこれらを必要と
しない。FST4 は CRC-24 を使い、`sgran` を呼ばないからである。

ビルドの動作確認:

```sh
target/ft4sim/ft4sim "CQ JL1NIE PM95" 1500 0.0 0.0 0.0 1 -15
ls 000000_*.wav   # one 7.5 s AWGN trial at -15 dB
```

(`ft4sim` の CLI には T/R 周期の引数がない — `fst4sim` と違い FT4 に
サブモードはないため: `message f0 DT fdop del nfiles snr` の位置引数 7 個。)

## 3. WAV コーパスの生成

```sh
scripts/gen_ft4_sweep_wavs.sh [ft4sim-path] [out-dir]
# defaults: ft4sim-path = target/ft4sim/ft4sim
#           out-dir     = embedded-poc/assets/ft4_sweep/
```

4 種のチャネル条件 (`awgn`、`ccir_good`、`ccir_moderate`、`ccir_poor`)
× `SNRS` グリッド × `TRIALS` 回の繰り返し (デフォルト 20) をカバーする。
`gen_fst4_sweep_wavs.sh` と同じ冪等/インクリメンタルな挙動 — グリッドを
広げて再実行しても、不足しているセルだけを補充する。

デフォルトの `SNRS` グリッド (`-5` から `-23` dB まで) は、FT4 の公表されて
いる WSJT-X AWGN 閾値である約 **-17.5 dB** (2500 Hz 基準帯域幅。FT8 の
-21 dB と比較 — FT4 は、より短い 7.5 秒のスロットと少ない FEC
インターリーブと引き換えに感度を犠牲にしている) を挟む。測定した 50%
クロス点が想定より弱いと判明した場合は、FST4 #146 のグリッド打ち切り
(censoring) の教訓に従い、グリッドをさらに深く拡張すること。

## 4. スイープの実行

```sh
MFSK_FT4_SWEEP_DIR=embedded-poc/assets/ft4_sweep \
  cargo test --test ft4_sweep --release \
  --features ft4,fft-rustfft,parallel,uvpacket \
  -- --ignored --nocapture
```

`uvpacket` が必要なのは、(`mod common` 経由で取り込まれる)
`tests/common/channel.rs` が無条件に `mfsk_core::uvpacket` を import する
ためだけである — FT4 自体とは無関係。

出力は素の recall テーブル (チャネル × SNR セルごとの decode 数 / トライアル
数) — pass/fail の assertion はなく、これは測定ツールである。

### 実測値 2026-07-18 (このコーパス/シード、`DecodeStrictness::Normal`)

隣接するグリッド点間の線形補間による 50% クロス点:

| チャネル | ~50% クロス点 |
|---|---:|
| AWGN | ≈ -15.5 dB |
| CCIR good | ≈ -14.7 dB |
| CCIR moderate | ≈ -14.3 dB |
| CCIR poor | ≈ -13.7 dB |

いずれも、公表されている -17.5 dB より約 2 dB 悪い。まだ根本原因は
特定していない — `ft4sim` の SNR 規約の問題、実装側の本物のパイプライン
ギャップ (#146 と同じ流儀の調査の候補)、あるいは公表値自体が別の指標を
指している可能性のいずれか。今回のパスではこれ以上追わず、ここに記録する
に留める。今後これに着手する際に使うべき「直す前に診断する」順序については、
`FST4_BENCHMARK.md` の 6 節を参照。

## 5. `DecodeStrictness` probe (issue #72)

`ft4_strictness_probe` (同じファイル内) は `engine::pipeline::decode_frame`
を `Strict` / `Normal` / `Deep` それぞれで直接駆動し、上記のスイープで特定
した部分 recall のセルをいくつか横断して、「golden」recall (既知の送信
メッセージと一致) と「any-msg」recall (golden かどうかを問わず CRC を通った
あらゆる decode — ここでは各トライアルの真の内容が既知なので、false-accept
の膨張の代理指標になる) の両方を報告する。

```sh
MFSK_FT4_SWEEP_DIR=embedded-poc/assets/ft4_sweep \
  cargo test --test ft4_sweep --release --features ft4,fft-rustfft,parallel,uvpacket \
  ft4_strictness_probe -- --ignored --nocapture
```

**2026-07-18 実測の知見:** このノブは FT4 では no-op *ではない*。「較正して
いないコピペなのでおそらく関係ない」という想定に反する。効果の大きさは
AWGN では小さいが、フェージング下では無視できない — 例えば `ccir_poor` の
-13 dB: Strict 9/20 → Normal 14/20 → Deep 16/20 (golden recall)。ただし
`Deep` は、いくつかのセルで "any-msg" を "golden" より速く増やす
(`ccir_moderate` -16 dB: golden 1→2、any-msg 1→4)。つまり Deep の追加
decode の一部は false-accept であり、本物の感度向上ではない — これは
FT8 のオリジナルの較正コメントが述べているのと同じ感度/false-positive の
トレードオフで、FT4 については今回まで未検証だった。

まとめ: `Normal` (現在ハードコードされているデフォルト) は既に妥当な落とし所に
ある — 重要なフェージングのセルで `Strict → Deep` の実質的な gain の大半を
捉えつつ (`ccir_poor` -13 dB では Deep の 7 decode 分の gain のうち 5 を
確保)、`Deep` の最悪の false-accept 増加は避けている。これがまだ答えていない
未解決の問いは、(FT8 からコピーした数値ではなく) FT4 *固有* の閾値の数値なら、
`Deep` の FP コストなしに `Normal` を上回れるかどうかである — それには、既存の
3 つの named level だけでなく、`osd_max_errors` / `osd_score_min` の値を振る
数値スイープが必要になる。

## 6. 数値の再較正 (issue #72, 2026-07-18)

`engine::pipeline::process_candidate_basic` 内の `strictness.osd_score_min()` /
`strictness.osd_max_errors()` の呼び出し箇所を辿ったところ、両方とも
`!is_fst4` の背後でゲートされており、**FST4 はこれらの値を完全に迂回する**
(#146 の fix) — 実際には `engine::pipeline::DecodeStrictness` の数値は
FT4 専用である。したがって `Normal` を再較正しても、FST4 の (別途調整された)
recall に対する regression のリスクはゼロであり、数値を単に特性評価するだけ
でなく実際に動かす道が開けた。

手作業でスイープした: 候補値を編集し、8 つの部分 recall セルに対して
`ft4_strictness_probe` を再実行し、any-msg が golden より速く増えることなく
(つまり新たな false-accept なしに) golden recall が上がったなら変更を残し、
そうでなければ破棄する。

- `osd_score_min`: `2.2 → 1.8`。`2.0` で 3 セル改善し、新たな FP はほぼ 0 件。
  `1.8` でさらに 2 セルがきれいに改善。`1.6` はそれ以上何も改善せず、
  false-accept を増やしただけ — `1.8` が上限である。
- `osd_max_errors`: `(depth3, depth4, other) = (26, 30, 29) → (28, 30, 31)`。
  `(29, 31, 32)` は同一の結果 (これ以上進めても利点なし)。`(30, 33, 33)`
  は golden recall の追加増加がゼロのまま false-accept が 2 件増えた —
  `(28, 30, 31)` が上限である。

フルスイープへの正味の効果 (Normal、変更前 → 変更後):

| チャネル | -17 dB | -16 dB | -15 dB | -14 dB | -13 dB |
|---|---:|---:|---:|---:|---:|
| AWGN | 0%→10% | 20%→20% | 85%→85% | 95%→95% | 95%→95% |
| CCIR good | 5%→10% | 20%→20% | 40%→45% | 90%→100% | 90%→90% |
| CCIR moderate | 5%→10% | 5%→5% | 30%→40% | 70%→70% | 95%→95% |
| CCIR poor | 0%→0% | 10%→15% | 20%→25% | 45%→50% | 70%→75% |

小幅で単調であり、どこにも regression はない (ゲートを緩めることは recall を
維持するか増やすことしかできない) — 実質的だが漸進的な gain であり、dB
スケールの跳躍ではない。`engine::pipeline::DecodeStrictness::{osd_max_errors,
osd_score_min}` に反映済み。`Strict`/`Deep` の数値は未変更 (依然として元の
未検証の FT8 コピーのまま — 現時点でこれらを使う呼び出し元はない)。

## 7. coherent な full-slot Δt 探索 (`ft4_sync_search`, 2026-07-18)

上記の `DecodeStrictness` チューニングとは別に、`sync4d.f90`/`ft4_decode.f90`
と本ライブラリの FT4 同期パイプラインとを突き合わせた WSJT-X 忠実性の監査で、
次のことが判明した。`engine::sync::coarse_sync` の広域 (±2.5 s) Δt 探索は
*non-coherent* (magnitude-squared のスペクトログラム bin) であるのに対し、
WSJT-X 自身の広域 Δt 探索 (`isync=1`、1 セグメントあたり約 350-450 サンプル
× 3 セグメント) は *coherent* (複素 Costas 相関) である。
`tests/ft4_coherent_wide_search_diag.rs` がその帰結を経験的に確認した:
CCIR フェージング下では、`coarse_sync` の non-coherent な選択が真のピークから
0.5-2.4 s ずれた位置に落ちることがあり、これは旧来のローカルな
`sync2d_refine` (±20 downsampled samples ≈ ±30 ms) の範囲を大きく外れている
— それでも真のピークの *coherent* スコアの方が一貫して高かった。AWGN は
影響を受けなかった (non-coherent のずれは約 16 ms 未満に収まり、旧 refine
の届く範囲内だった)。

修正: `engine::sync2d::ft4_sync_search`。WSJT-X の `isync=1`/`isync=2` ループを
模した coherent full-slot Δt 探索で (±12 Hz/3 Hz の coarse × 絶対座標
`[-344, 1012]` downsampled-sample 窓・step 4、続いて ±4 Hz/1 Hz × ±5 sample
の fine)、FT4 では `sync2d_refine`/`Sync2dConfig::for_ft4` を置き換える
(FST4 は #146 で、同等の `fst4_sync_search` を既に獲得していた)。

**最初の試みは正味で regression だった** (クリーンな AWGN ですら recall が
低下した) — 実際の罠なので記録しておく価値がある: Δt 探索を広げると、
実信号自身の周波数近傍にある `coarse_sync` 候補が、*より多く*、独立に、本物の
自己無矛盾な Costas ロックに到達できるようになる (ノイズではない —
`sync_quality` と、ゼロから実装した WSJT-X `nsync_qual` 相当の bit-metric
ゲートの両方が、これらに高いスコアを付けることを確認した)。試して破棄した
もの: (a) dedup 時に `hard_errors` が最小の重複を優先する — 悪化した、
hard_errors は位置精度と相関しない。(b) 追加の `nsync_qual` 式ゲート —
WSJT-X 自身のゲート設計はここには当てはまらないことが判明した。WSJT-X の
より単純な「周波数ピークごとに候補 1 つ」というアーキテクチャは、そもそも
こうした近接重複候補を生成しないので、そのゲートはこれらの間を裁定する
ためのものでは元々なかった。

**実際の根本原因**: `DecodeResult.freq_hz` が `refined.freq_hz` ではなく、
*精密化前の* `cand.freq_hz` から設定されていた — 潜在バグであり、以前は無害
だった。旧来の狭いローカル refine は、そもそも真値に近いところから始まった
候補しか補正しなかったからである。広域探索は遠く離れた候補も成功させる
ようになり、それぞれが自分の古い `cand.freq_hz` を報告し続けた — recall の
バグではなく報告層のバグである。`process_candidate_basic` 内の `DecodeResult`
構築 3 箇所すべてを `refined.freq_hz` を使うよう修正した。dedup の
`sync_score` による tie-break は残した (重複が、正しく報告されるようになれば
ほぼ同じ精密化後の位置に収束するので、今ではほぼ見た目上のものである)。

### 実測値 2026-07-18 (6 節の `DecodeStrictness` のみのベースライン比)

| チャネル | -17 dB | -16 dB | -15 dB | -14 dB | -13 dB |
|---|---:|---:|---:|---:|---:|
| AWGN | 10%→20% | 20%→30% | 85%→100% | 95%→95% | 95%→100% |
| CCIR good | 10%→20% | 20%→40% | 45%→65% | 100%→100% | 90%→95% |
| CCIR moderate | 10%→10% | 5%→5% | 40%→50% | 70%→80% | 95%→95% |
| CCIR poor | 0%→0% | 15%→20% | 25%→35% | 50%→55% | 75%→80% |

50% クロス点の改善 (線形補間): AWGN ≈+0.2 dB、CCIR good ≈+0.7 dB、CCIR
moderate ≈+0.3 dB、CCIR poor ≈+0.25 dB。CCIR poor の -5 dB (100%→95%、
1 トライアル分) を除き regression なし — 20 トライアル/セルでは
サンプリングノイズの範囲内。非 ignored のテストスイート全体
(`cargo test --release --features full`) と
`ft4_wsjtx_sample_recall_vs_golden` に対して検証済み — すべて green。

## 8. `nsync>=18` の OSD-depth ゲート — 本物のバグ、ただし AWGN のギャップの原因ではないと確認 (issue #72, 2026-07-18)

6 節自身の「直す前に診断する」原則 (およびこのファイルの姉妹ファイル
`FST4_BENCHMARK.md` §6 の同一の規則) に従い、これ以上数値に手を付ける前に、
WSJT-X と本実装の diff を推測して全スイープを再実行し確かめる代わりに、
`ft4_diag_weak_trials` (`tests/ft4_sweep.rs`、`fst4_diag_weak_trials` を
踏襲) を作り、7 節以後の ≈-15.7 dB のクロス点を挟む個々の AWGN
トライアルで `coarse_sync` + `process_candidate_basic` を直接トレースした。

**クロス点では coarse_sync がボトルネックではない。** -17..-14 dB の AWGN
セル (80 トライアル) 全体で、真の周波数の近傍に候補がそもそも 1 つもなかった
トライアルは 8/20 (-17 dB) と 6/20 (-16 dB) だけだった — 支配的な失敗モード
(-17 dB と -16 dB の両方で 10/20) は、まさに真の freq/dt で見つかり、
`coarse_sync` のスコアも成功したトライアルと同じ範囲にある候補が、それでも
`process_candidate_basic` の内部で失敗するというものだった。

**その経路を計装している最中に発見**: `sync_quality` の FT4 出力 (`nsync`) は
`N_SYNC = 16` (Costas ブロック 4 つ × 各 4 シンボル、`ft4/mod.rs:98`) で
頭打ちになる — 実測した約 140 候補の値は、decode の成否にかかわらず 16 が
上限で 10-16 に集中していたことで確認した。ところが
`process_candidate_basic` の OSD depth エスカレーションのゲート
(`core/pipeline.rs`) は、depth-3 のラングと depth-4 の Top-K rescue の両方で
`nsync >= 18` をチェックしていた — これは FT8 の `N_SYNC = 21` (12/21、18/21)
に対して較正された閾値で、プロトコルごとにスケールされずリテラルとして
コピペされていた。**上限が 16 のとき 18 は数学的に到達不可能である**:
FT4 の候補は OSD depth-2 より先へ決して進めず、depth-3/depth-4 の
エスカレーション階層全体が黙って落とされていた — issue #72 が
`DecodeStrictness` について既に名指ししていたのと同じ「FT8 からのコピペ」
パターンが、別のゲートで現れたものである。

`core/pipeline.rs` で修正した: `osd_attempt_min`/`osd_depth3_min` は、FT8
の元の 12/21 と 18/21 が示唆する比を使って `P::N_SYNC` でスケールされる
ようにした (FT4 の `N_SYNC=16` では → 9/14)。`P::ID == Ft4` でゲートして
いるので、FT8 (`N_SYNC=21` — 式は 12/18 を正確に再現するのでバイト同一)
と FST4 (既に別途調整済み、issue #146) は影響を受けない。

**狭い範囲で再検証 (AWGN のみ、-17..-14 dB、診断の単純化した候補ループ
ではなく、`ft4_snr_sweep` 経由の実際の `decode_frame`)**: 4/20、6/20、
20/20、19/20 — 7 節のチューニング後のベースライン (20%、30%、
100%、95%) と**バイト同一**。`ft4_diag_weak_trials` は、今度は正しく OSD
depth-3 にエスカレーションする `nsync>=14` の個々のトライアルをトレース
し (一時的なトレースで確認、その後除去)、それでも失敗した — これらは
depth 制限ではなく、純粋に SNR 制限である。非 ignored のスイート全体 +
`ft4_wsjtx_sample_recall_vs_golden` (6/6) は green。

**結論**: 本物であり残す価値がある (プロトコルのスケールに関する正真正銘の
バグであり、未検証の隅 — CCIR フェージング、busy-band の複数信号 — は
排除されていない) が、**約 1.8 dB の AWGN ギャップの説明では
ないと確認された**。まさに 6 節が警告した種類の結果である: もっともらしく
聞こえ、ソースで検証された diff が、フルの再スイープでは針を動かさな
かったと示された。当初の監査に残っている 2 つの候補 — (1) bitmetrics を
計算する前の、WSJT-X 流の早期の `smax` に基づく候補の reject が全くない、
(2) `ft4_sync_search` が、単調性のゲート付きの WSJT-X の 3 セグメント Δt
探索を再現しているかが不明 — は、まだ調査されていない。

## 9. `ft4_sync_search` の scorer が各 Costas ブロック内で non-coherent だった (issue #72, 2026-07-18) — AWGN で約 1 dB の gain

`ft4_sync_search` に入力される `score` と `osd_score_min` ゲートの意味が、
WSJT-X のそれと実際に同じかどうかという直接の問いがきっかけだった —
8 節の「scorer は忠実」という注記を信頼する前に問うべき正しい問いで、
その注記は*外側*の構造しか確認していなかったことが判明した。

`ft4_sync_search` の doc コメントは、scorer が `sync4d.f90` の
`sync = p(z1)+p(z2)+p(z3)+p(z4)` と「厳密に」一致すると主張していた。
最も外側のレベル (Costas ブロック 4 つ、magnitude の総和、ブロック間は
non-coherent) では正しい — しかし `sync4d.f90` を最終的な式だけでなく
1 行ずつ読み直すと、各 `z_k` は**ブロック k の 4 シンボル全体にまたがる
1 つの coherent な内積**である
(`z1=sum(cd0(i1:i1+4*NSS-1:2)*conjg(csync2))`、4 シンボル分の波形を
連結した参照から作られる単一の複素アキュムレータ)。`ft4_sync_search` は
代わりに `score_costas_block` を呼んでいて、これは**各シンボルを個別に**
相関させ、それぞれの `.norm_sqr()` (パワー) を取り、4 シンボルをパワー加算
する。N=4 サンプルにわたる non-coherent な結合は、1 回の coherent な N=4
相関に比べて弁別力を約 sqrt(4) ≈ 3 dB 失う — FST4 (`project_fst4_coherent_sync`
memory、issue #146) で既に診断され修正されたのと同一のメカニズムである:
`fst4_sync_search` は既に正しい仕組み (`make_costas_ref_continuous` +
`score_flat_coherent`、ブロックごとに 1 つの平坦な参照を 1 回相関させ、
magnitude を取る) を備えており、FT4 のブロックごとのループに配線されて
いなかっただけだ。

`score_costas_block` 自体が誤っていないことを確認するため `sync8d.f90`
(FT8) と突き合わせた: FT8 自身の結合は本当にシンボルごとで、non-coherent
で、パワー加算である (7 つの Costas シンボル位置それぞれのループの中で
`sync = sync + p(z1)+p(z2)+p(z3)`、`p(z)` に sqrt はなく — 純粋なパワー)
— `score_costas_block` は *FT8 に対しては*忠実であり、FT4 の本質的に異なる
(ブロックごとに coherent な) 結合方式に流用したときにだけ誤りとなる。
修正は `ft4_sync_search` のみに限定した (`core/sync2d.rs`):
`make_costas_ref`/`score_costas_block` を
`make_costas_ref_continuous`/`score_flat_coherent` に置き換え、
`fst4_sync_search` が既に使っているのと全く同じヘルパーを再利用した。
`score_costas_block` 自体、(他のプロトコルと共有される) `sync2d_refine`、
および FT8 自身の fine-sync は変更していない。

**測定** (`ft4_snr_sweep`、`--ignored --nocapture`、AWGN -20..-13 dB
+ 3 つの CCIR チャネル全部 -18..-12 dB、診断ハーネスではなく実際の
`decode_frame`):

| チャネル | -18 dB | -17 dB | -16 dB | -15 dB | -14 dB | -13 dB |
|---|---:|---:|---:|---:|---:|---:|
| AWGN | 0%→0% | 20%→40% | 30%→75% | 100%→100% | 95%→100% | —→100% |
| CCIR good | —→10% | 20%→45% | 40%→60% | 65%→95% | 100%→100% | 95%→95% |
| CCIR moderate | —→5% | 10%→10% | 5%→35% | 40%→65% | 70%→95% | 95%→95% |
| CCIR poor | —→0% | 0%→0% | 15%→25% | 25%→50% | 50%→75% | 75%→90% |

(AWGN の -20/-19/-18 dB は変更前も変更後も 0% のままだった — クロス点は
グリッドの内側に安全に収まっており、床に張り付いてはいない。)

50% クロス点 (線形補間): **AWGN -15.7→-16.7 dB (+1.0 dB)**、CCIR good
-15.6→-16.7 dB (+1.1 dB)、CCIR moderate -15.0→-15.5 dB (+0.5 dB)、
CCIR poor -14.25→-15.0 dB (+0.75 dB)。スイープした範囲のどこにも regression
はない。これは issue #72 の調査全体で AWGN における最大の単独の gain
であり — WSJT-X の公表値 -17.5 dB との約 1.8 dB のギャップの半分以上が
1 回の修正で埋まった (残りのギャップは現在 ≈0.8 dB)。フェージング
チャネルの gain は AWGN より小さく、これは 4 シンボルのブロック幅にわたる
チャネルの非相関によって coherent な結合が部分的に損なわれることと整合
する — もっともらしいが、まだ個別には検証していない説明である。

`ft4_wsjtx_sample_recall_vs_golden`: 依然として 6/6 で、busy な WSJT-X の
サンプル WAV は CRC を通った decode を合計 13 件返すようになった (以前は
11 件) — 新しい 2 件は callsign/grid/report のフィールドが整っており、新たな
false-accept ではなく、本物の弱い信号を回収した結果と整合する。非 ignored
のスイート全体は green (350+ テスト、失敗 0)。

8 節の教訓を反対側から補強する: 特に数値/スケールの忠実性が問いで
あるとき、「WSJT-X と一致する」という主張の検証を最も外側の式で止めない
こと — バグは、コードコメントが実際に確認していたより 1 段深いところに
あった。

## 10. BP/OSD の decode 強度 — 大半は棄却、忠実性の fix が 1 件、AWGN では効果なし (issue #72, 2026-07-18)

同期側の fix (8-9 節) が入ったので、残りの ≈0.8 dB の AWGN ギャップが
そうではなく decode 強度の問題 (LDPC の BP/OSD が WSJT-X の `decode174_91`
より弱い) かどうかを調べた — これは `project_wsjtx_compliance_audit.md` が
「BP algorithm (tanh-product vs min-sum NMS) — 最大の一般的なギャップ」と
指摘していた一般的な懸念である。

**BP algorithm: host では既に忠実であり、この懸念はここには当てはまらない。**
`fec::ldpc::bp::FecOpts::default()` は `bp_kind: BpKind::SumProduct`
(`core/protocol.rs:384`) を設定する — 真の対数領域の tanh/atanh belief
propagation (`tov = 2·atanh(−∏tanh(−toc/2))`) であり、WSJT-X の
`bpdecode174_91.f90:101,107-110` と代数的に一致する。
`BpKind::NormalizedMinSum`/`OffsetMinSum` は同じモジュールに存在するが、
embedded の fixed-point パス (`bp_decode_generic_nms`、`bp.rs` に embedded
専用と doc されている) でしか使われず — この AWGN スイープ全体を含め、
どの host テストからも実行されない。compliance audit の懸念は embedded
パスについては現実のものだが、ここで測定されたものは何も説明しない。

**OSD はほとんど使われない。** `ft4_diag_weak_trials` を拡張して
`DecodeResult.pass` (どの LLR variant か / BP か OSD rescue か) と
`hard_errors` を出力させた。クロス点付近で成功した 45 件の decode (AWGN
-18..-15 dB) のうち: 43 件は素の BP で成功 (pass 0/1/2、OSD 不要)、OSD
depth-3 の rescue (pass 5) を要したのは 2 件だけで、OSD depth-2/4 (pass 4、
または Top-K パス) は一度も発火しなかった。同じトレースで失敗した 17 件の
候補のほぼすべてで `nsync` と `score` はどちらも試行されており (`osd@9=true`)、
その値は成功例の値と大きく重なる — 分離は見られず、ゲートのアーティファクト
ではなく純粋に SNR 制限の失敗であることと整合する。

**本物の BP パラメータの不一致を 1 つ見つけて修正したが、AWGN では効果なし。**
プロトコルごとに WSJT-X 自身の `max_iterations` (BP の反復予算) を確認した:
`ft8b.f90:96` と `fst4/decode240_101.f90:27` はどちらも 30 を使い
(`core/pipeline.rs` にハードコードされた共有の `bp_max_iter: 30` と一致)、
`ft4_decode.f90:194` は **40** を使う — 外れ値は FT8/FST4 ではなく FT4 で
ある。修正を `P::ID == Ft4` に限定し (8-9 節と同じパターン)、FT8/FST4 は
バイト同一のままとした。狭い範囲で再検証 (AWGN -19..-14 dB): 9 節の fix 後の
ベースライン (40%/75%/100%/100%) と**バイト同一** — この範囲では BP の収束は
反復予算に制限されていない。候補は 30 反復よりずっと手前で収束するか、
予算にかかわらず収束しないかのどちらかである。それでも残した (コストゼロで、
今では WSJT-X にバイト忠実であり、この狭い AWGN の確認が扱わなかった
フェージング/busy-band のケースでは効くかもしれない)。

**結論**: BP/OSD の強度は、残りの AWGN ギャップの原因ではない。algorithm は
host で既に WSJT-X に忠実であり、ここでの OSD の限界的な寄与 (成功の約 4%)
は大きな効果を隠すには小さすぎ、見つかった本物のパラメータのギャップは
AWGN の recall を動かさない。`ft4_wsjtx_sample_recall_vs_golden` は依然として
6/6、非 ignored のスイート全体と `-D clippy::perf` は green。8 節の締めの注記
にある 2 つの候補 — bitmetrics より前の WSJT-X の早期の `smax` に基づく
候補 reject と、3 セグメント Δt 探索の構造 — が、残りの ≈0.8 dB を探す
場所として引き続き最有力である。

## 11. 3 セグメント Δt 探索の retry — 実装、測定、revert (issue #72, 2026-07-18)

WSJT-X の `ft4_decode.f90` の `iseg=1,2,3` ループは、単一の collapse した
パスより位置を徹底的に*見つける*だけではない — 候補ごとに最大 3 つの異なる
Δt 位置で **decode** を試みることができる (セグメント 1 `[108,560]` は
`smax≥1.2` であれば常に最初に試され、セグメント 2/3 も試されるが、それは
それぞれの `smax` がセグメント 1 のものを上回る場合だけである)。
`ft4_sync_search` の単一の collapse した全窓パス (9 節) は、大域最良の位置を
1 つ選んで 1 回だけ decode する — 原理的には、より広い union 窓のどこかに
ある、スコアがより高い偽の位置が、低 SNR で真の信号自身の (スコアはより
低いが本物の) 位置に勝ってしまう脆弱性がある。

診断が WSJT-X のセグメントごとの探索を文字通り再現できるように
`ft4_sync_search_window` (`core/sync2d.rs`、`ft4_sync_search` は今では
完全な `[-344,1012]` の union を渡すその薄いラッパー) を作り、現在失敗して
いるすべての候補について、(collapse した大域最良ではなく) セグメント 1 の
位置だけで decode すれば成功したかどうかを確認する `ft4_diag_segment_retry`
(`tests/ft4_sweep.rs`) を作った。

**最初のパスは劇的に過大報告した**: 10/17 の「rescue」。fix を
`process_candidate_basic` に実装し (単一の collapse した呼び出しを置き換える
FT4 専用の 3 セグメントループ)、実際の `ft4_snr_sweep` で再検証した —
修正前のベースラインと**バイト同一**で、動きは 0 だった。間違っていたのは
理論ではなく診断である: `try_decode_at` は、本物の OSD 結果が通る
`hard_errors ≥ osd_max_errors` の reject ゲートなしに OSD を無条件に試して
おり、そのゲートが reject するために存在するまさにその種の、過剰訂正された
低信頼度のコードワードを受け入れてしまっていた。診断を、同じゲートを適用し、
decode したメッセージを (単に「CRC を通った」だけでなく) `GOLDEN_MSG` と
照合するよう修正した — 修正後の結果: **rescue は 0/17**、実際のスイープと
完全に一致した。`process_candidate_basic` のセグメントループを単一の collapse
したパスに戻した (測定された利益ゼロで、FT4 候補ごとの探索/decode コストが
3 倍になるのを避ける)。`ft4_sync_search_window` は、無害で再利用でき、また
診断 (`ft4_diag_segment_retry`) は、偽の競合ピークがクリーンな AWGN よりも
ありうる CCIR フェージングや busy-band 下での将来の再確認に残す価値が
あるため、残した。

教訓をはっきり述べる: 仮説に最良のチャンスを「与える」ために本番の実際の
ゲートを飛ばす診断は、答えのように見えるほど大きな偽陽性を作り出しうる。
(単純化した代用品だけでなく) *実際の*本番コードパスに対して再検証すること
— 6 節以来このファイルで繰り返されるテーマ — が、欠陥のある数字を根拠に
コードが出荷される前にそれを捕まえた。

## 12. 誤った (古い) スコアに対する `osd_score_min` ゲート — 本物の fix、AWGN でさらに約 0.5 dB の gain (issue #72, 2026-07-18)

11 節の診断を構築している間に、decode の結果と並べて `cand.score` を出力した
ところ、別個の本物の問題が浮上した: `cand.score` は `coarse_sync` の
non-coherent なスコアであり、`ft4_sync_search` が計算する coherent なスコア
(9 節が WSJT-X に忠実になるよう修正したもの) とは異なる、無関係な量である。
`strictness.osd_score_min()` (`1.8`) は 6 節で `cand.score` に対して調整
された — coherent なスコアが別個の量として存在する以前の話だ — そして
`process_candidate_basic` の OSD 試行ゲート (`core/pipeline.rs`) は、今でも
coherent なスコアではなく `cand.score` をチェックしている。

直接測定した (`ft4_diag_weak_trials`、現在失敗しているクロス点付近の AWGN
候補 17 件): **13/17 (76%) が `cand.score < 1.8`** であり、そのため OSD を
そもそも試みない — 素の BP が既に失敗しており、それが彼らの得る唯一の
ラングである。17 件すべてが WSJT-X 自身の `syncmin=1.2` を (正しく計算された
coherent なスコアで) 楽々とクリアしている — これらは、ゲートが守ろうと
しているノイズではなく、紛れもなく本物の信号である。

WSJT-X 自身の FT4 デコーダには、OSD 試行のスコアゲートが全くない —
`decode174_91` は BP と OSD を 1 回の呼び出しの中で一緒に実行し、
`ndepth`/`maxosd` で制御され、スコアのチェックでは決して制御されない。
8 節の FST4 迂回 (`is_fst4`) は、#146 で見つかった同一の症状のために既に
存在する: 「すべての本物の候補の coarse-sync スコアが `osd_score_min` を
下回っていた (OSD を完全に塞いでいた)」。同じ迂回を FT4 に拡張した
(`bypass_osd_score_min = is_fst4 || P::ID == Ft4`)。両方を迂回する FST4 と
は異なり、(古くなった量のゲートではなく、本物の hard-error の上限である)
`osd_max_errors` を false-accept の安全網として残した。

**測定** (`ft4_snr_sweep`、実際の `decode_frame`):

| チャネル | -19 | -18 | -17 | -16 | -15 | -14 | -13 |
|---|---:|---:|---:|---:|---:|---:|---:|
| AWGN | 0%→0% | 0%→15% | 40%→60% | 75%→80% | 100%→100% | 100%→100% | 100%→100% |
| CCIR good | —→0% | 10%→15% | 45%→65% | 60%→75% | 95%→95% | 100%→100% | 95%→95% |
| CCIR moderate | —→5% | 5%→15% | 10%→15% | 35%→40% | 65%→80% | 95%→100% | 95%→95% |
| CCIR poor | —→0% | 0%→0% | 0%→0% | 25%→55% | 50%→60% | 75%→85% | 90%→90% |

どこにも regression はない。50% クロス点 (線形補間): **AWGN
-16.7→-17.2 dB (+0.5 dB)** — WSJT-X の公表値 -17.5 dB とのギャップは今や
≈0.3 dB だけである。CCIR good -16.7→-17.3 dB (+0.6 dB)、CCIR moderate
-15.5→-15.75 dB (+0.25 dB)、CCIR poor -15.0→-16.1 dB (+1.1 dB、単一
チャネルとして最大の gain — poor フェージングのトライアルは、塞がれた
`cand.score<1.8` の帯域に座っている候補が明らかに最も多かった)。

`ft4_wsjtx_sample_recall_vs_golden`: 依然として 6/6、decode の合計も依然と
して 13 件 (実サンプルに新たな false-accept はない — 残してある
`osd_max_errors` ゲートが役目を果たしている)。非 ignored のスイート全体と
`-D clippy::perf` は green。

**issue #72 の AWGN の累積結果**: -15.5 dB (チューニング前) → -17.2 dB
(現在)、WSJT-X の公表値 -17.5 dB とのギャップを本質的に閉じた。WSJT-X
ソースで検証された 3 つの本物の fix が寄与した (6、9、12 節)。もっともらしく
見える 2 つの候補は、構築され、測定され、信念に基づいて出荷される代わりに
正しく破棄された (8、11 節)。

## 13. Coarse-candidate 段は誤ったアルゴリズムだった — `getcandidates4.f90`
    の忠実移植 (`ft4_coarse_sync`)、decode 速度が約 25 倍向上 (2026-07-20)

続編の調査 (#72 ではない — `dapper-soaring-nest` 計画。coarse-sync の
精度と BP/OSD の速度の両方を改善せよという依頼が発端) は、セクション
1〜12 とは別の観察から始まった。FT4 の golden WAV の decode は
プロトコルスイート全体で最も遅く (6 信号・7.5 秒の録音で 1.20 秒 —
`BENCHMARKS.md` の「Decode speed」表を参照)、しかも golden テストは、
200 で足りるはずだというコード内コメントに反して `max_cand=2000` を
必要としていた。

**根本原因。`lib/ft4/getcandidates4.f90` と `lib/ft4/ft4_baseline.f90`
を 1 行ずつ読んで確認した**: `engine::sync::coarse_sync` (FT8/FST4 など
と共有、このセクションまで FT4 にも使われていた) は 2 次元 (周波数 ×
lag) の Costas 配列相関探索であり、構造的に WSJT-X が FT4 に対して
やっていることとは*異なる*。`getcandidates4` は lag/Δt 次元をそもそも
探索しない。純粋な周波数領域のピリオドグラムである (4 シンボル幅の
重なり合うセグメントに対する Nuttall 窓 FFT、時間平均、15 bin の
boxcar 平滑化、`ft4_baseline` の 5 項多項式ノイズフロアによる正規化、
局所最大の周波数ピークごとに候補 1 つ)。FT4 の実際の Δt 決定は
すべてずっと後段の `ft4_sync_search` (セクション 9) で行われ、そこは
各候補自身の `dt_sec` を無視して*絶対的*な全窓探索をすでに行っている。
結果として、汎用探索が周波数ごとに出す最大 8 個の lag が異なる候補は、
下流ではどれも機能的に冗長である — 各々が独立に `ft4_sync_search` + LLR
+ BP + OSD の全コストを払い、同じ結果に収束するはずのものを重複して
処理している。

**変更前の計測** (`ft4_diag_candidate_cost_split`、
`tests/ft4_sweep.rs`、golden WAV): 2000 候補がわずか 440 の相異なる
周波数に分布 (冗長度 4.5 倍)、そして候補あたりコストの合計約 6.65 秒の
うち `ft4_sync_search` だけで 5.09 秒を占めた (LLR+BP+OSD は 1.46 秒に
すぎない)。これは BP/OSD 自体がボトルネックだったことは一度もない
ことを裏付ける。修正すべきは候補生成であり、`fec::ldpc::bp` がすでに
パリティ/CRC 収束で打ち切ること、OSD depth-4 が実質的にまず発火しない
こと (セクション 10) とも整合する。

**修正**: `engine::ft4_coarse::ft4_coarse_sync` — `getcandidates4.f90`
+ `ft4_baseline.f90` の忠実な移植で、既存の忠実なプリミティブを再利用
した (`engine::baseline::fit_baseline` は単体テスト済みだが、これまで
は汎用の Costas 相関スコアに対する不整合な正規化器として試され —
そして revert された — だけだった。`engine::sync::parabolic_peak`)。
`ft4_sync_search` と並べて `engine::` に置いた (`ft4::` ではなく)。
`engine` はどのプロトコル feature が有効かに関わらずコンパイルされ、
`engine::pipeline` が無条件に呼び出す必要があるため。`engine::pipeline::decode_frame`
/ `decode_frame_subtract` 内の `P::ID == Ft4` 分岐として配線した。
既存の sync2d-refine 分岐と同じパターンである。WSJT-X からの意図的な
逸脱が 1 つある: 候補は `max_cand` に切り詰める前にスコア順に
ソートされる (WSJT-X 自身の周波数走査順の埋め方は、Fortran の固定長
配列の都合であって、意図した順位付けではない)。

**変更後の計測** (同じ診断、golden WAV): 31 個の相異なる周波数に 31
候補 (冗長性は解消 — ちょうど周波数あたり 1.0)、`ft4_sync_search` の
コスト合計 75.7 ms、LLR+BP+OSD は 25.8 ms。実際の本番エントリポイント
(`decode_frame_subtract`、rayon 並列、subtract 3 パス): **1.20 秒 →
48.8 ms (約 25 倍)**。`ft4_wsjtx_sample_recall_vs_golden` は **6/6**
のまま (decode 総数は 12、従来は 13 — 近似重複/境界すれすれの accept
が 1 件減ったのであって、golden の取りこぼしではない)。golden テストの
`max_cand` は 2000→100 に下がり、recall はバイト単位で同一だった —
100 は `ft4_decode.f90` における WSJT-X 自身の `MAXCAND=100` にも一致
しており、計画外だが心強い対称性である。

**更新 (2026-07-26): この 48.8 ms という数値は陳腐化し、その後修正
された (issue #182)。** Issue #178〜#180 (`FT8_BENCHMARK.md` セクション
8 を参照) は、recall の品質のために FT4 の `decode_frame_subtract` を、
このセクションが計測した安価な定振幅 subtract から、FT8 が使うのと
同じ WSJT-X 忠実なチャネル対応 LPF subtract へ移行した。しかしこの行
はその後再計測されず、約 526〜576 ms へ退行していた (複数回実行の中央
値。`decode_frame_subtract(&audio, 100.0, 2700.0, 0.05, 100)` を直接
呼んだ場合も、このファイル自身の `ft4_diag_candidate_cost_split`
計装経由の場合も同じ)。

根本原因 (SIC subtract ループが受理した候補ごとに行う 2 つの呼び出し
の周りに一時的な `Instant` タイマーを追加して判明): `subtract_tones_lpf`
そのもの (issue #180 ですでに FFT キャッシュ済み、1 呼び出しあたり
1 ms 未満) ではなく、`engine::dsp::subtract::refine_freq` が
約 35 ms/呼び出し × 実 decode 14 件 ≈ 490 ms で、退行のほぼすべて
だった。`refine_freq` は、subtract の前に `ft4_coarse_sync` の約
5.2 Hz bin というキャリア推定を補償するため、±5 Hz を 0.1 Hz 分解能で
グリッド探索する (約 101 回の評価)。各評価は `generate_iq` →
`synth_complex_f32_into` を呼び、GFSK 整形された変調の**全体**を毎回
一から再構築していた (erf ベースの Gaussian パルステーブル、
`O(nsym·pulse_len)` のシンボルごとの畳み込み、`O(nwave)` の位相積分
+ `sin`/`cos` ループ全体)。1 回の呼び出し内の 101 回の評価の間で違う
のはキャリア周波数だけなのに、である — このファイルの
`fine_refine_3stage` 修正と同じ、「探索ループ内で不変なものを再計算
する」バグパターンだ。

**修正**: `generate_iq` のキャリア項は位相積分の前に全サンプルへ一様に
加算されるので、`phi(k; f0) = phi_mod(k) + k·(2π·f0·dt)` となる —
任意のキャリアは、キャリアを含まない (トーン変調のみの) 位相の上に
乗る純粋な線形位相ランプである。`refine_freq` は 1 呼び出しにつき
キャリアなしのフェーザを一度だけ構築し、各グリッド点は完全な再合成
ではなく、サンプルごとの安価な NCO 回転 + 加法定理で導出するように
なった (`ls_amp_mag_tweaked`、`core/dsp/subtract.rs`)。約 59k サンプル
のバッファにわたる f32 ドリフトを抑えるため、ローターは定期的に再
正規化される。凍結した完全再合成パスに対する差分テスト
(`ls_amp_mag_tweaked_matches_full_resynthesis`、絶対/相対の複合許容誤差
— LS 振幅には数十分の 1 Hz 間隔で深い相関ヌルの櫛があり、そこでは
相対誤差だけでは意味のある指標にならない) と、argmax 保存テスト
(`refine_freq_finds_true_offgrid_carrier`) で検証した。

計測結果: `refine_freq` は約 35 ms/呼び出し → 約 15.7 ms/呼び出し
(`RAYON_NUM_THREADS=1`、スレッド数から呼び出し単体のコストを切り分け
るため)。`decode_frame_subtract` の本番の実時間: マルチスレッド
(デフォルトの rayon、元の 48.8 ms を計測したのと同じ条件) で
約 575.8 ms → **約 280 ms**、シングルスレッドで 約 893.6 ms →
**約 602 ms**。48.8 ms への完全な復帰ではない — LPF subtract +
周波数 refine のステップは意図的かつ恒久的な recall 品質の追加分 (上記
の Rayleigh フェージング干渉源シナリオで 10/10 対 0/10) であり、#178
以前の定振幅パスより常にコストがかかる。この修正が除くのはその追加
コストの*冗長な*部分であって、追加コストそのものではない。候補数は
不変 (31/31、冗長性なし)、recall にも影響なし (依然 6/6)。現在の
ヘッドライン数値は `BENCHMARKS.md` の「Decode speed」表を参照。

**テストスイート内で見つかり修正された実在の退行 (本番ではない)**:
`ft4_roundtrip.rs` のクリーン信号ラウンドトリップテスト 2 件が失敗し
始めた。根本原因 (一時的なクレート内デバッグテストで 18 種類の異なる
メッセージを調査して確認、真のキャリア付近に着地したのは 0/18): これら
2 件のテストは、送信された 1 信号の周囲に絞った約 400〜600 Hz の帯域を
探索する — 範囲内のどこにも他の内容がないため、`fit_baseline` の
セグメントごとの低パーセンタイルによるノイズフロア当てはめには信号
しか当てはめる対象がなく、推定値が系統的に間違って出る。その帯域を
100 Hz〜2700 Hz (freq_min, freq_max) に広げる — このリポジトリの他の
すべての FT4 呼び出し元がすでに使っている規約 — と、両テストは完全に
直った (調査で 18/18)。当初は代わりに合成ディザノイズを加えて直そう
としたが、先に試して破棄した: 約 16 dB SNR (sigma=3000) でも効かず、
続く 18 メッセージの調査で、狭い帯域ではノイズレベルによらず失敗が
0/18 だったことが確認された — 決定的であってノイズ感度ではなく、
ノイズでは最初から直りようがなかったのだ。これは WSJT-X 忠実性の
ギャップではない: `getcandidates4` のベースライン当てはめは、現実的に
広い運用帯域と本物の信号外の内容があることを前提としており、それは
まさに WSJT-X 自身の実際の使い方 (約 200〜4910 Hz) およびここの他の
すべての呼び出し元と同じである — 2 件のラウンドトリップテストの狭い
帯域のほうが例外で、古い Costas 相関探索の (ベースラインを持たない)
異なるスコアリングのもとでたまたま成り立っていた仮定にすぎなかった。

**AWGN/CCIR スイープ、全面再計測** (`ft4_snr_sweep`、
`--ignored --nocapture`): 50%-crossing は、閾値の再調整ではなく純粋な
候補生成の入れ替えによって変化した。したがって (セクション 6〜12
とは異なり)「gate を緩めれば recall は保たれるか増えるしかない」という
不変条件は厳密には当てはまらない — 構造的に異なるアルゴリズムは、
SNR 応答曲線を一様に改善したり保ったりするのではなく、正当に
その形を作り変えうる。

| Channel | 50% crossing (before) | 50% crossing (after) | Δ |
|---|---:|---:|---:|
| AWGN | -17.2 dB | -16.9 dB | -0.3 dB (regressed) |
| CCIR good | -17.3 dB | -17.5 dB | +0.2 dB |
| CCIR moderate | -15.75 dB | -15.7 dB | ~0 (noise) |
| CCIR poor | -16.1 dB | -16.0 dB | ~0 (noise) |

4 チャネル中 3 つは同等以上。AWGN だけが約 0.3 dB 悪化した (WSJT-X の
公表値との比較ギャップが約 0.3 dB から約 0.6 dB に広がる) — ただし
一様ではない: AWGN の -19/-18 dB はむしろ*改善*した (0%→5%、
15%→25%) 一方、-17 dB だけが低下した (60%→45%、20 トライアル中 3 件の
振れ)。曲線は一様にシフトしたのではなく形が変わったということだ。
約 25 倍の decode 速度向上と、4 チャネル中 3 つが退行していないことを
踏まえ、このトレードはこのパスの中でこれ以上追わずに受け入れた。AWGN
のギャップを再び詰める必要が出た場合に再検討する (候補: `ft4_coarse_sync`
の 15 bin の平滑化幅が、AWGN の crossing 付近で `sync_min` の位置と
違う形で干渉しているか — 未検証)。

非 ignored の全スイート (922 件 pass、0 件 fail) と
`-D clippy::perf -D warnings` は green。

## 14. デッドコードの後始末: `sync2d_refine`/`Sync2dConfig` を削除 (2026-07-20)

上のセクション 7 (および FST4 の #146) は、両プロトコルをすでに共有の
2 パス*局所* refine (`engine::sync2d::sync2d_refine` / `Sync2dConfig`、
coarse-sync 候補の周囲の ±10〜±20 ダウンサンプル済みサンプルの窓) から、
それぞれ独自の全スロット coherent 探索
(`ft4_sync_search`/`fst4_sync_search`) へ移していた — しかし古い関数と
その config 構造体は削除されず、参照されないまま残っていた。リポジトリ
全体 (`mfsk-core/src`、`mfsk-core/tests`、`embedded-poc`、
`mfsk-ffi*`) を、doc コメント内の言及ではなく実際の呼び出し箇所について
grep した: ゼロ。`sync2d_refine`、`Sync2dConfig`、そしてそこからしか
到達できない 2 つのヘルパー (`twiddle_ref`、`score_at<P>`) を、
不要になった `make_costas_ref`/`score_costas_block` の import とともに
削除した — 162 行。`Sync2dResult` (現役の両探索関数がいまも返す共有の
出力型) は残る。検証: 非 ignored の全スイートが green、
`-D clippy::perf -D warnings` がクリーン、さらに FST4 のみ / FT8 のみ
(FT4/FST4 なし) の feature ビルドがなおコンパイルできることを別途
確認した — このモジュールには自前の `#[cfg(feature = "ft4")]` gate は
ない (`ft4_coarse` と同じ理屈で、`engine` は無条件にコンパイルされる)
ので、どちらかのプロトコルを除外した構成に迷い込んだ参照があれば
そこで発覚したはずである。

## 15. Phase 4 `smax` early-reject — 実装・計測・revert (2026-07-20)

セクション 13 の Phase 4 は明示的に条件付きだった: coarse-sync の修正が
入ったあとでも、プロファイルで LLR/BP/OSD が実時間の有意な割合を占める
場合にのみ、候補ごとの早期打ち切り (WSJT-X の `ft4_decode.f90:279
if(smax.lt.1.2) cycle`) を追求する、というものだ。それでも依頼を受けて
再検討した。セクション 6 が確立した「数値を選ぶ前に測る」という規律
は同じである。

**較正** (`ft4_diag_smax_calibration`、`tests/ft4_sweep.rs`、新規):
現行の本番候補生成器 (`ft4_coarse_sync`) + `ft4_sync_search` を、AWGN/
CCIR の crossing 近傍の全領域 (チャネル/SNR セル 17 個 × 20 トライアル)
で実行し、すべての候補について coherent `score` を、それが
`GOLDEN_MSG` に decode されたかどうかとともに記録した。golden に
成功した候補は 142 件、観測された最小スコアは **266.1**、golden でない
候補は 16858 件。最初の読み「16418/16858 (97.4%) が golden の下限を下回る
スコア」は、強力なフィルタリングの余地があるように見えた。

**実装**: `process_candidate_basic` (`core/pipeline.rs`) 内の gate —
`if P::ID == Ft4 && score < 200.0 { return None }` — を
`ft4_sync_search` の直後、`symbol_spectra`/LLR/BP/OSD の前に置いた。
`200.0` は、観測された下限 266.1 より約 25% の安全マージンを持たせて
選んだもので、WSJT-X 自身の `1.2` を当て推量で換算したものではない
(絶対スケールが違う: こちらは単位 RMS に正規化した `cd0` 上の 4 つの
Costas ブロックにわたる振幅和である)。

**計測**: golden WAV は 6/6 のまま (12/12 decode、バイト単位で同一)、
全 AWGN/CCIR スイープは gate 導入前のベースラインとバイト単位で同一、
全スイート (922 テスト) と clippy はクリーン — 設計どおり recall に
安全である。しかし、golden でないスコアのうち*200.0 という cutoff
自体の下*に実際に落ちたものの割合 (最初の読みが比較した 266.1 という
golden の下限ではなく) を再確認すると、まったく違う数字が出た:
**91/16858 (0.5%)**。「97.4% が golden の下限を下回る」という統計は
事実だったが誤解を招くものだった — ジャンク候補のスコアは 266.1 の
すぐ*下*の帯 (大半は [200, 266)) に密集しており、その遥か下に散らばっ
てはいない。「安全」と「有効」の間にギャップはない: より多くの
ジャンクを捕まえるために cutoff を 266 に向けて広げると、それを小さく
保つ動機となったまさにその候補に対する安全マージンを削ってしまう。

**Revert した。** この gate は、実コードとマジックナンバーの cutoff を
追加して、得られたのは候補処理の作業量にして実測 0.5% の削減にすぎな
かった — セクション 13 以降すでに実時間コストが 1 桁ミリ秒であること
を考えれば、保守の負担に見合わない。`ft4_diag_smax_calibration` は
残した (いまは (チャネル, SNR, トライアル) のグリッド全体にわたって
rayon で並列化されている — このパスで、`process_candidate_basic` 自身の
内部処理を逐次のまま放置するのではなく、このファイルの診断もファイル/
トライアルの水準で並列化するという一般的な戒めとして指摘された。上の
既存の `ft4_snr_sweep` の規約に従う) 。あとで別のコーパスや別のスコア
量でこれを再検討する人のための、再利用可能なリファレンスとして。

非 ignored の全スイート (922 件 pass) と `-D clippy::perf -D warnings`
は、revert の前後とも green。

## 16. `refine_freq` の探索半径は 5 倍広すぎた (issue #182 続報, 2026-07-26)

セクション 13 の更新の続き: issue #182 (`FT8_BENCHMARK.md`、同日の
`fine_refine_3stage` 移植) は、`refine_freq` の支配的コスト — グリッド点
ごとの GFSK 完全再合成 — を、一度だけ構築したキャリアなし参照に対する
サンプルごとの NCO 調整 (`ls_amp_mag_tweaked`) に切り替えることで修正
した。これで `refine_freq` は約 35 ms/呼び出しから約 15.7〜16.2
ms/呼び出しに、`decode_frame_subtract` の golden WAV での実時間は
約 530〜580 ms から約 280 ms に下がった — 実際の修正ではあったが、#178 以前のベースライン 48.8 ms の 5.7 倍のままであり、このセクション
が扱うのはユーザが報告した「まだ遅い」という症状である。

`decode_frame_subtract` の残りの部分から `refine_freq` /
`subtract_tones_lpf` を切り離した単独のマイクロベンチマーク (それぞれ
14 回の呼び出し、golden WAV の実際の accept 済み decode 数に合わせた)
で再計測した: `refine_freq` だけで合計 280 ms のうち 227 ms (81%、
16.2 ms/呼び出し)、`subtract_tones_lpf` は 13 ms (5%、issue #180 で
すでに FFT キャッシュ済み) にすぎなかった。NCO 修正のあとでも、
ボトルネックは `subtract_tones_lpf` ではなく `refine_freq` の
±5 Hz/0.1 Hz のグリッド (1 呼び出しあたり評価 101 回) であることが確認
できた。NCO 修正が削ったのは*評価 1 回あたり*のコストであって、
*評価回数*ではない。

**評価回数が不必要に多かった根本原因**: `refine_freq` の呼び出し側
コメント (「+/-5 Hz refine radius」— WSJT-X に一致すると主張されて
いた) を `lib/ft4/subtractft4.f90` と直接突き合わせた。WSJT-X の
`subtractft4` には周波数 refine のステップが**まったくない** — decode
された `f0` で参照を合成して直接 subtract するだけで、グリッド探索も
`ctwk` 風の調整もない (その仕組みは FT8 専用の `sync8d.f90` にあり、
*subtract* ではなく*同期*の間に動作する)。呼び出し側コメントの
「WSJT-X に一致」という主張は単に誤りだった。`refine_freq` は
mfsk-core 固有のステップで、このコードベース自身の coarse-sync の周波数
分解能を補償するものであり、WSJT-X が subtract 経路で行う何かの移植
ではない。

±5 Hz という数字は、`refine_freq` 自身の doc コメントに由来する。それは
*汎用*の `engine::sync::coarse_sync` の約 2.93 Hz の FFT bin 分解能
(「両側 1 bin をカバーするため 2.5 Hz を推奨」) に対して書かれたもの
だった。しかし FT4 は、セクション 13 (`ft4_coarse_sync`、独自の異なる
bin 構造を持つ `getcandidates4.f90` の移植) で、あの汎用 coarse-sync
経路の使用をやめている — さらに直接的には、`refine_freq` に入る
`r.freq_hz` は、生の coarse-sync bin ではなく、`process_candidate_basic`
の `ft4_sync_search` 後の refine 済みの値 (セクション 7 自身の修正)
である。`engine::sync2d::ft4_sync_search` の df 探索 (`core/sync2d.rs`)
を 1 行ずつ読むと、その粗いパス (`idf` が `-12..=12` で step 3) も細か
いパス (`si` が `-4..=4` で step 1) も、生成するのは**整数 Hz** の `df`
値だけである — 報告される `freq_hz` は常に
`candidate.freq_hz + <整数>` だ。これにより、真の連続的な最適値は
構成上、報告値の ±0.5 Hz 以内に収まる — 汎用 coarse-sync のコメントが
仮定した ±2.5 Hz よりはるかに強い保証であり、FT4 がこのより厳しい
上界を持つ経路に移ったとき、±5 Hz の半径は再導出されなかった。

**修正**: `ft4/decode.rs` の `decode_frame_subtract_with_options` の
呼び出し側 — `refine_freq_radius_hz` を `5.0 → 1.0` (既存の 0.1 Hz
ステップは維持した。`ls_amp_mag_tweaked` の周波数応答の主ローブは、
約 7.5 秒のトーン列に対して 1 Hz を十分下回るほど狭く、*ステップ*を
広げるとそれを丸ごと飛び越すおそれがあるため。大きすぎたのは*半径*
だけだった)。グリッドは 1 呼び出しあたり評価 101 回から 21 回に減り、
上記の ±0.5 Hz の量子化上界の両側にそれぞれ 0.5 Hz のマージンが残る。

**計測**: golden WAV の `decode_frame_subtract` 実時間 **280.3 ms →
110.2 ms** (約 2.5 倍)、recall はバイト単位で同一 (golden 6/6、decode
総数 14/14、メッセージ/freq/dt も従来と同じ)。セクション 11 で参照した
繁忙帯域の Rayleigh フェージング退行ガード
(`ft4_busy_band_fading_probe.rs::busy_band_fading_baseline`、issue #177〜#179 の LPF subtract 移行がまさに修正するために作られたシナリオ)
は、目標の回収が **10/10** のままだった。非 ignored の全スイートと
`-D clippy::perf -D warnings` は green。#178 以前のベースラインからの
累積: 48.8 ms → 110.2 ms (残りのギャップは 2.3 倍) — 残差は LPF
subtract + freq-refine ステップの、いまや削減不能なコスト (実 decode
ごとに `ls_amp_mag_tweaked` の評価 21 回 + `subtract_tones_lpf` の FFT
ペア 1 組) であり、セクション 13 の元の更新が述べた意図的な recall
品質の追加分であって、冗長な作業ではない。

## 17. 最初のハードウェア計測 — M5Stack CoreS3 (2026-08-29)

FT4 はこれまで基板向けにコンパイルされたことすらなく、ましてや実機で
動かされたこともなかった。このセクションは最初の実機の数値と、それが
まだ収まらない理由である。そこに至るまでに何を作る必要があったかを
含む完全な記録は `docs/reference/EMBEDDED.md` の「FT4 on embedded」で
あり、こちらはベンチマーク側の記録である。

**セットアップ。** `ft4-bench`
(`embedded-poc/m5stack-cores3-app/src/bin/`)、M5Stack CoreS3 @ 240 MHz、
`opt-level = 3`、シングルコア、WiFi なし。ツリー内の WSJT-X golden
`000000_000002.wav` に対して、100〜2700 Hz、`sync_min = 0.05`、
`max_cand = 100` で `ft4_coarse_sync` が見つける 31 個の coarse 候補 —
`ft4_wsjtx_sample_recall_and_precision` が走らせるのと同じ探索から、
その `.sic_rounds(3)` を除いたもの (組込み FT8 は単一パスで出荷される
ので、複数パスの数値では基板が実際に走らせるものを表さない)。アセット
は `ft4_bake_golden_precomputed` で焼き込んだ。ログ:
`embedded-poc/m5stack-cores3-app/logs/ft4-bench_clean_2026-08-29.log`。

**予算**: 7.5 s のスロット − (0.5 s の TX オフセット + 105 × 48 ms) =
**1.96 s**。

| stage | total | per candidate | share |
|---|---:|---:|---:|
| `downsample_cached` (5120-pt inverse FFT) | 2 252 ms | 72.7 ms | 13 % |
| **`ft4_sync_search`** | **13 225 ms** | **424 ms** | **76 %** |
| LLR + BP (`DecodeDepth::EMBEDDED`) | ~1 861 ms | ~60 ms | 11 % |
| **total (production `process_candidate_basic`)** | **17 339 ms** | 559 ms | — |
| same at `DecodeDepth::FULL` | 19 684 ms | 635 ms | — |

**実機で 11 件の相異なる decode、同じアセットでのホストと両 depth で
同一。** つまりここでは `DecodeDepth::EMBEDDED` は recall を損なわず、
このファイルでは OSD は何も稼いでいない — 最適化として OSD を削ること
を誰かが提案する前に知っておく価値がある。(golden テストが assert
する 14/14 には `sic_rounds(3)` が必要であり、単一パスはホストでも 11
が頭打ちである。)

**17 339 ms 対 1 960 ms は 8.8 倍超過**であり、超過分は関数 1 つによる。

**コストは構造的なものだ。** 31 候補すべてにわたる候補あたりの
`ft4_sync_search`: **最小 423 835 µs、p50 423 897 µs、最大 424 684 µs**
— ばらつきは 0.2 %。これは固定グリッドであって、候補に依存する何かで
はなく、セクション 7 自身の設計から直接導かれる: 探索は、各候補の
`dt_sec` に関わらず、すべての候補について絶対的な `[-344, 1012]` の
ダウンサンプル済みサンプルの窓を走査する。約 19 900 個の (Δf, Δt)
セル × 4 Costas ブロック × 4 シンボル × 32 サンプル ≈ 10.2 M 複素 MAC。
そのうち 424 ms は複素 MAC あたり約 10 サイクル — その場でのフェーザ
回転を伴うスカラー f32 ループである。

2 つのレバーがあり、どちらも未試行で、どちらもこのベンチに対して
計測可能である: 内積に対する `dsps_dotprod_f32_aes3` と、探索窓の
狭小化 (WSJT-X は時計を仮定できないのでスロット全体を探索する。UTC に
アンカーされた基板はそれを仮定でき、`wspr-fano-cap-fast` /
`wspr-pass2-topn` が、それが失う recall とともに文書化された組込み専用
トレードの先例である)。

同じ 31 候補に対するホストの参照値、逐次
(`ft4_diag_candidate_cost_split`、`tests/ft4_sweep.rs`): 約 89 ms、
したがって実機/ホスト比は約 195 倍。目安として、FST4-60 の最初の
組込みの数値 (issue #306) は、最適化パスの前で約 1728 倍だった。

**ここでは計測していないもの**: `ft4_coarse_sync` 自体 (`NFFT1 = 2304`
= 256 × 9 で、ESP-DSP カーネルはまだない — 候補リストは代わりに焼き
込んである)。ホストのスロットでは 0.3 ms なので、結論は変わらない。
(**上書き — §25**: 実機ではこの段は 1 288 ms であり、結論は変わる。)

## 18. Δt 窓の狭小化 — recall にいくらかかるか (2026-08-29)

§17 は、`ft4_sync_search` を、8.8 倍超過の予算の 76 % として、候補
あたり 0.2 % のばらつきで位置づけた — コストが Δt 窓だけで決まる固定
グリッドである。それを狭めるのは明らかなレバーであり、WSJT-X はそれを
取れない (時計を仮定できないので広く探索する) 一方、UTC にアンカーされ
た受信機は取れる。このセクションはその代価である。

**窓の算術。** `i0` は `ds_rate = 12 000/NDOWN = 666.67 Hz` の
ダウンサンプル済みサンプルを数え、`dt = i0/ds_rate −
TX_START_OFFSET_S` なので、`dt = 0` は `i0 = 333` にあり、本番の
`[-344, 1012]` はスロット全体ではなく **±1.0 s** である。粗い段のコスト
は `9 × ceil(n_i0/4)` セルに固定の 99 セルの細かいパスを足したもの。

### WSJT-X golden について

(§23.5 は、このファイルが実電波の録音ではなく `ft4sim_mult` の出力で
あることを確定させた。それでも DT の広がりは持っている — シミュレータ
は各信号の DT を ±0.5 s で一様に引く — し、このセクションが必要と
するのはそれである。実電波の DT 分布についての証拠ではない。)

`ft4_diag_sync_window_recall` (`tests/ft4_wsjtx_samples.rs`)、
`000000_000002.wav`、31 個の coarse 候補、`DecodeDepth::EMBEDDED`。
対照実験では、本番の窓でのハーネスが `process_candidate_basic` とまった
く同じものを decode することを assert しており、したがって表が計測して
いるのはハーネスではなく窓である。「search ms」は
`ft4_sync_search_window` 単体の、31 候補すべてにわたって合計した実測
の実時間である。

| half-width | i0 window | cells | predicted | measured | decodes | first loss |
|---:|---|---:|---:|---:|---:|---|
| ±1.000 s | [-344, 1012] | 3159 | 1.00× | 76.6 ms | 11 | — |
| ±0.750 s | [-167, 833] | 2358 | 1.34× | 1.35× | 11 | — |
| **±0.500 s** | **[0, 667]** | **1602** | **1.97×** | **1.91×** | **11** | **—** |
| ±0.375 s | [83, 583] | 1233 | 2.56× | 2.40× | 10 | `N1TRK KB7RUQ RR73` (dt −0.44) |
| ±0.300 s | [133, 533] | 1008 | 3.13× | 2.92× | 9 | + `W7BOB KJ7G RR73` (dt −0.36) |
| ±0.250 s | [167, 500] | 855 | 3.69× | 3.43× | 8 | + `VE3LON K7RL R 549 WA` (dt +0.29) |
| ±0.200 s | [200, 467] | 702 | 4.50× | 4.18× | 4 | |
| ±0.150 s | [233, 433] | 558 | 5.66× | 5.24× | 3 | |
| ±0.100 s | [267, 400] | 405 | 7.80× | 7.16× | 2 | |

計測した高速化はセル数に追従し、固定の細かいパスの分だけ届かない。
11 個の実信号は dt −0.44 … +0.30 s に及び、**±0.5 s は無料**である。

### 機構の切り分け

`ft4_diag_dt_window_reach` (`tests/ft4_sweep.rs`) は、ほかに何も変えず
に、*真の* DT を窓に対して掃引する。**これは意図的に tier-C コーパスを
使わない**: `gen_ft4_sweep_wavs.sh` は `DT=0.0` に固定しているので、
その中のすべての信号はすべての窓の真ん中にあり、どんな幅でも損失なし
と報告してしまう — decoder ではなく、フィクスチャの性質である。別の
`ft4sim` コーパスを、DT ∈ [−0.5, +0.5] を 0.05 s ステップで、セルあたり
20 トライアル、AWGN で生成した。(グリッドが ±0.5 s で止まるのは、
`ft4sim` が 6.048 s のファイルを書き、フレームが `[0.5+DT, 5.54+DT]`
を占めるためで、それを越えると信号は窓に取りこぼされるのではなく
ファイルに切り詰められ、両者は切り分けられない。上の golden は、実際
の録音でより広い裾をカバーしている。)

-14 dB では、結果は**名目上の窓の縁での急峻な崖**になる: 内側は 100 %、
外側は 0 %、なだらかな肩はない。±0.500 s は ±0.5 s のグリッド全体で
100 % を保つ。±0.375 s は |DT| ≈ 0.35 まで、±0.250 s は |DT| ≈ 0.25
まで保つ。

-17 dB (-16.9 dB の AWGN crossing 付近) では、recall は予想どおり
10〜55 % であり — そして**DT が窓の内側にある限り列ごとに同一**である。
狭めて失うのは*到達範囲*であって*感度*ではない: 狭い窓がなお見える
信号は、広い窓で見えたときとまったく同じ頻度で decode される。

### これで何が残るか

両方の計測手段で、損失なしの最大幅は **±0.5 s で、支配的な段で 1.91 倍
の価値**がある。これを §17 の実機の数値に当てはめると、13 225 ms →
約 6.9 s、スロット合計は 17 339 ms → 約 11.1 s で予算 1 960 ms に対して
**8.8 倍ではなく 5.7 倍超過**となる。必要ではあるが十分ではない —
残りの係数は、グリッド内部の算術 (`dsps_dotprod_f32_aes3`、未試験) か、
`cd0` の置き場所 (候補あたり 40 KB、現在は PSRAM の `Vec`。
`internal_pool` は FT8 のスクラッチについてまさにこの移動で約 5〜10 倍
とすでに文書化している) から得る必要がある。どちらもホストではなく
実機側の計測である。

## 19. §17 の実機数値に対する 2 つの最適化 (2026-08-29)

§17 は FT4 を 1.96 s のスロット予算の 8.8 倍超過と計測し、
`ft4_sync_search` がその 76 % を占めていた。§18 は、その探索の Δt 窓を
狭めるとどれだけ失うかを計測した (±0.5 s までは何も失わない)。この
セクションは残る 2 つのレバーであり、どちらも適用して計測した。

### 19.1 内積 — `FlatRef` + `dot_f32` (ホスト + 実機)

`ft4_sync_search_window` は、周波数シフトを `cd0` に、最内のサンプル
ループの**内側**で、`(df, i0)` セルごとにやり直す回転フェーザ
(`twid *= step`) として適用していた。しかし `twid[n] = step^n` は `i0`
ではなく Costas ブロック*内*のオフセットで添字づけられる — したがって
各 `df` が掃引する約 340 個の `i0` 位置にわたって同一であり、セルごと
に作り直すのはそれだけの回数だけ冗長だった。

`fst4_sync_search` はすでにこの問題の正しい側にいた: その `FlatRef` は
シフトを `(block, df)` ごとに一度だけ参照に折り込み、素の複素内積を
残す。それを `dot_f32` — したがって LX7 上の `dotprod-extern` の
`dsps_dotprod_f32_aes3` — が処理できる。FT4 はいま同じ仕組みを使って
いる。この関数自身のコメントがすでに恒等式を記録していた
(「`cd0` の各サンプルをその場でツイドルするのと同じ内積」)。新しく導出
したものは何もない。

**ビット単位では同一でない**。これは意図的である: 積が再結合されるし、
`FlatRef::fill` は漸化式を累積するのではなくサンプルごとにフェーザを
評価するので、丸め誤差は*より少ない*。着地前に検証した:

| check | result |
|---|---|
| `ft4_wsjtx_samples` golden | 14/14 total, 6/6 golden, 0 phantoms — unchanged |
| `run-sensitivity-sweeps.sh ft4` | awgn/ccir_good/moderate/poor all **+0.00 dB** vs baseline, 160 trials each |
| golden stage counters | `nsync_fail`/`nsync_pass`/`osd_attempt`/`n_new` identical on all 3 SIC passes |
| merge gate | green |

**ホスト**、`ft4_diag_sync_window_recall`、31 候補:
`ft4_sync_search` **76.6 ms → 27.9 ms (2.75 倍)**。golden の 3 パス
`decode_loop`、3 回の中央値: **36.5 ms → 26.6 ms (1.37 倍)** — その経路
は rayon 並列で、探索はその一部にすぎないので希釈されている。

### 19.2 `cd0` の置き場所 — 計測した、そして仮説は誤りだった

§18 は、第 3 のレバーとして `cd0` の PSRAM 常駐を示唆して締めくくられ
ていた: 候補あたり 40 KB、グリッド全体で候補あたり約 13 MB の読み出し、
そして `internal_pool` 自身の doc コメントは、まさにこの種のホット
バッファを内部 DRAM に移すことで FT8 の `cs` スクラッチについて約
5〜10 倍と記録している。

**計測結果: 1.12 倍。** バイト数についての算術は正しかった。それが
ボトルネックだったという推論は正しくなかった。§19.1 のあとでは、
アクセスパターンは 2 KB スライスに対する逐次の `dot_f32` であり、S3 の
PSRAM データキャッシュはそれをうまく捌く — それ以前のコードは、サンプル
ごとの複素乗算で演算律速であって帯域律速ではなく、バッファを移動して
も壊れていないものは直せなかったのだ。残す価値はある (約 40 KB で、
起動時に確保してしまえばコストはない) が、見かけほどのレバーではない。

本番経路では、このバッファは必要に応じて確保するのではなく、起動時に
`worker_arena` 経由で取る必要があることに注意: WiFi が上がっていると、
この基板で空いている最大の内部ブロックは 31 744 B である。

### 19.3 実機での複合効果

`ft4-bench`、CoreS3 @ 240 MHz、`opt-level = 3`、シングルコア、同じ 31
候補。ログ: `logs/ft4-bench_opt_2026-08-29.log`。

| configuration | search | cumulative |
|---|---:|---:|
| §17 baseline | 13 225 ms | 1.00× |
| + `FlatRef`/`dot_f32` (§19.1) | 4 447 ms | **2.97×** |
| + `cd0` in internal DRAM (§19.2) | 3 937 ms | 3.36× |
| + ±0.5 s window (§18) | **2 492 ms** | **5.31×** |

候補ごとのばらつきは全体を通して平坦なまま (パス 2: 最小 142.5 ms、
p50 142.9 ms、最大 145.5 ms) — 依然として固定グリッドであり、ただより
安くなっただけだ。

**スロット合計、本番経路** (`process_candidate_basic`。これは完全な窓で
自前の PSRAM `cd0` を構築するので、§19.1 のみを含む):
**17 339 ms → 8 642 ms、2.01 倍**、`DecodeDepth::EMBEDDED` でも `FULL`
でも、ホストと完全に一致する **11 件の相異なる decode** のまま。

3 つすべてを段ごとの内訳に投影すると、2 251 (downsample) + 2 492
(search) + 1 943 (LLR/BP) = **約 6 686 ms 対 1 960 ms — 8.8 倍から
下がって 3.4 倍超過。**

### 19.4 残っているもの

探索はもはや支配的ではない。投影した内訳は downsample 34 % / search
37 % / LLR+BP 29 % — 攻めるべき単一の項はなく、`downsample_cached`
(5120 点の逆 FFT に、候補ごとの 92 160 bin の抽出とテーパーを加えたも
の) が同格になった。それは、DDC フロントエンドが高速化するのではなく
丸ごと取り除く段であり、FST4 がすでにテンプレートを持っている
`mfsk_core::ft4::ddc` の作業 (`docs/notes/FST4_DDC_DESIGN.md`) である —
そしてこのベンチがなお依存しているホスト焼き込みの広帯域 FFT も廃止
できる。

## 20. DDC フロントエンド — `downsample_cached` を置き換え、コストはゼロ (2026-08-30)

§19.4 の引き継ぎを実行した。`mfsk_core::ft4::ddc` は候補ごとの `cd0` を
ミキシングとフィルタリングで組み立てるので、`ft4-bench` のアセットが
ホスト上で今も焼き込んでいる 92 160 点の順方向 FFT には、もう供給先が
残っていない。

**ホスト上の作業のみ。** ここまでの内容はまだ実機で動かしていない。
動かすために何が足りないかは §20.4 に書いてある。

### 20.1 FT4 が易しいケースである理由

`fst4::ddc` には有理数リサンプラが必要である
(`FST4_DDC_DESIGN.md` §4.2)。`NSPS = 3888 = 2⁴·3⁵` だと `3⁵` の分母が
残り、整数のデシメーションではそこに届かないからだ。FT4 の `NDOWN = 18`
は 12 kHz をちょうど割り切る: `12 000/18 = 666.667 Hz` はすでに
`SyncDims::ds_rate` であり、`ds_spb = NSPS/NDOWN = 32` は 2 のべき乗である。
したがってモジュール全体が `FirStage` 2 つとミキサ 2 つで済む —
`PolyphaseResampler` も `RxGrid` も不要で、`cd0` より下流には何の変更もない:

```text
12 kHz real i16
  → Mixer(f0 + 31.25 Hz)                   complex @ 12 kHz
  → FirStage A: 199 taps, fc 320 Hz, ÷18   complex @ 666.667 Hz
  → FirStage B: 263 taps, fc 56 Hz,  ÷1    complex @ 666.667 Hz
  → Mixer(−31.25 Hz)                       cd0, f0 at DC
```

サンプル位置合わせにトリミングは不要である: `FirStage` はカウンタを
`group_delay + 1` から始めるので、各段の出力 0 はその段自身の入力 0 を
中心とし、`cd0[0]` は `audio[0]` と揃ったままになる — これにより
`ft4_sync_search` の絶対値 `[-344, 1012]` の窓が、どちらのフロントエンド
でも同じ意味を持つ。

### 20.2 通過帯域はデコードパラメータである

`downsample_cached` は `[f0 − 1.5·Δf, f0 + 4.5·Δf]` =
`[−31.25, +93.75] Hz` を残し、**それ以外をゼロにする**。この帯域は
`f0` を中心に非対称であり (トーンは `f0` から上へ並ぶ)、候補を中心とする
ローパスでは再現できない — そこでミキサ対を置き、*帯域*を中心に据えて
から `f0` を DC に回して戻すことで、すべてのタップを実数のまま保つ。

これを誤るのは見た目だけの誤りではない。
`process_candidate_basic_impl` は `cd0` を全長にわたって RMS 正規化し
(WSJT-X `ft4_decode.f90:231-232`)、`compute_llr` の `LLR_SCALE` はその
単位 RMS の入力に対して較正されている。したがって参照帯域の外側の雑音を
通してしまうと、BP に入るすべての LLR のスケールが変わる。±333 Hz の
ベースバンドをそのまま渡していれば、RMS は約 2.3× 高くなっていた。

等価雑音帯域幅として測定した — 正規化が見る唯一の数値であり、各経路の
任意のゲインは自身のトーン応答で割って除いてある
(`ft4::ddc::tests::noise_bandwidth_matches_the_
reference_band`):

| path | noise/tone power ratio |
|---|---:|
| `downsample_cached` | 2.150e-3 |
| `ft4::ddc` | 2.161e-3 |
| difference | **+0.021 dB** |

参照側の 101 ビンのレイズドコサインテーパは、1 ビンあたり
`12 000/92 160` Hz で 13.0 Hz にあたる — ±49.5 Hz まで平坦で、±62.5 で
ゼロになる。段 B は ~49 Hz まで平坦で ~63 までに null になる。一致は
チューニングではなく構成による。

### 20.3 golden と crossing 全域での等価性

**WSJT-X golden** (`ft4_ddc_equivalence::ft4_ddc_baseband_decodes_the_
golden_like_the_fft_path`)。`ft4_coarse_sync` から得た同一の 31 候補で、
下流はすべて同一に保った:

| depth | FFT path | DDC path | max Δfreq | max Δi0 |
|---|---:|---:|---:|---:|
| `EMBEDDED` | 11 distinct | 11 distinct, same set | 1.00 Hz (1 of 11) | 0 |
| `FULL` | 11 distinct | 11 distinct, same set | 1.00 Hz (1 of 11) | 0 |

1.00 Hz は `ft4_sync_search_window` 自身のグリッド刻み
(`df = idf as f32`) であり、つまり表現しうる最小の不一致である —
1 つの候補が、同程度に良い 2 つのセルの間に位置している。11 件中 10 件は
完全に一致し、sync 位置はまったく動かない。

**Tier-C ペアスイープ** (`ft4_ddc_recall_matches_the_fft_path_across_
the_crossing`、`#[ignore]`)。4 チャネル × 7 SNR タグ × 20 trial = 560
ファイルで、各チャネルの 50% crossing をまたぐ。ペアで行った: 両アームが
同じファイルを同じ候補リストからデコードするので、雑音の実現値が相殺され、
cell あたり 20 trial で実際の差を分解するのに足りる。

| | decodes of 560 |
|---|---:|
| FFT front end | 237 |
| DDC front end | **238** |
| disagreements | 5 (3 for the DDC, 2 against) |

不一致はすべて、すでに自身の crossing 上にある cell に出ている。
**フロントエンドの入れ替えのコストは 0.0 dB である。**

### 20.4 これがまだ*やっていない*こと

- **実機では未測定。** 予測される候補あたりのコストは
  `5000 × 199` + `5120 × 263` ≈ 2.3 M 複素 MAC で、これに対し
  `downsample_cached` が CoreS3 で測定したのは 2 251 ms
  / 31 候補 ≈ 72 ms である (§19.3)。これは算術であり、コストについての
  算術が測定の前にどれほどの価値しか持たないかを、§19.2 がこのファイル
  自身への注意書きとして残している: この形をした直前の仮説は 5–10× を
  予測して 1.12× を出した。
- **esp-dsp の FIR バックエンドがない。** `FirStage::push_block` は
  存在する (`FST4_DDC_DESIGN.md` §4.5) が、`dsps_fird_f32_aes3` を束ねる
  ものはまだなく、今日実機で動かせばスカラー経路を使うことになる。
- **粗探索段にカーネルはできたが、それを使った実機実行はない。**
  `ft4_coarse_sync` の `NFFT1 = 2304` (= 256·9) は最後に残った 2 のべき乗
  でない長さだった。`engine::dsp::fft_mixed_2304` は、
  `fft_mixed_5120` が候補ごとの逆変換を担うのと同じ方式でこれを担い
  (256 × 9 の Cooley-Tukey、9 点因子はさらに `fft_15::
  fft_3` 上の 3 × 3)、`EspDspPlanner` が両方向を配線する。ベンチは
  今も焼き込み済みの候補リストを読むため、実機ではまだ何も動かしていない。
  この段のホスト側コストは 0.3 ms であり、S3 ではスロットあたり約 152 回の
  変換が測るべき数値である。
  **2026-08-30 に測定 (§25): 1 288 ms、すなわち変換 1 回あたり 8.5 ms で、
  それだけでデコード予算全体の 66 %。** この箇条書きと §20 にあった
  「ホストで 0.3 ms だから問題になりえない」という推論は誤りだった —
  そのホストの数値が何を計時していたにせよ、この段のデバイス/ホスト比は、
  候補ごとの各段が示す ~30x とは似ても似つかない。
- **`decode_frame` には配線していない。** `fst4::ddc` が選んだのと同じ
  方針である: ライブラリの部品として呼び出し側が手に取るものであり、
  ホストのフロントエンドを黙って入れ替える feature flag ではない。

## 21. 候補予算 — この探索パラメータは WSJT-X のものではなかった (2026-08-30)

`ft4_coarse_sync` より後の段はすべて候補ごとの処理である: Δt 探索、
LLR ラダー、BP、OSD。したがって候補数は複数あるつまみの一つではなく、
それらすべてに掛かる乗数だが — これまで誰も測っていなかった。
`tests/ft4_candidate_budget.rs` がそれを測る。

### 21.1 1.0 未満の `sync_min` は閾値ですらない

`getcandidates4.f90` は平滑化したスペクトルを、フィットしたベースライン
(`ft4_baseline.f90`、`engine::baseline::fit_baseline` に移植) で割る。
これにより **雑音は構成上 ~1.0 になる**。それを下回る `sync_min` は
帯域内のあらゆるピークを通してしまう。WSJT-X は `syncmin = 1.2` を渡す
(`ft4_decode.f90:195`) が、このクレートのベンチは **0.05**、スイープ
ハーネスは **0.8** を渡していた。

測定結果 — `sync_min` はフィルタするだけで並べ替えはしないので、
ファイルあたり 1 パス:

| `sync_min` | mean candidates, 560 sweep files | golden, 14 signals | decodes (of 237) |
|---:|---:|---:|---:|
| 0.05 | 67.1 | 31 | 237 |
| 0.80 | 67.1 | 31 | 237 |
| 1.00 | 54.8 | 28 | 237 |
| 1.10 | 12.1 | 12 | 237 |
| **1.20** (upstream) | **1.6** | **12** | **237** |
| 1.30 | 1.0 | 12 | 237 |
| 1.40 | 0.8 | 12 | 235 |
| 1.50 | 0.6 | 12 | 226 |
| 1.70 | 0.4 | 11 | 174 |
| 2.00 | 0.1 | 11 | 77 |

スイープ列は、4 チャネル × 7 SNR タグ × 20 trial で各チャネルの 50%
crossing をまたぐもの — すなわち閾値が効くとすればまさにそこである。
にもかかわらず 1.4 まで効かない。

したがって、忠実な値は**混雑した実録音で 2.6×、疎な録音で 42× 安く、
どちらでも測定上の recall コストはゼロ**である。これはさらに押し込むための
チューニングつまみではない: 膝は 1.4 にある。

### 21.2 `max_cand` は golden で上限が決まり、スイープには見えない

WSJT-X golden では、11 件の single-pass デコードは 31 件中ランク 0-11 から
出ているので、`max_cand = 12` でそのすべてを再現できる:

```text
rank  0   1147.7 Hz  score 1067.27   KB0VHA KA1YQC R 539 MA
rank  1   2066.6 Hz  score  104.29   VE3LON K7RL R 549 WA
...
rank 11   2412.8 Hz  score    1.55   W7BOB KJ7G RR73
```

スイープコーパスでは**すべての**デコードがランク 0 にある — これは
フィクスチャの性質 (1 ファイルにつき信号 1 つなので常に最強ピークになる)
であって、ランキングについての知見ではない。§18 の窓の問題に対して
`gen_ft4_sweep_wavs.sh` の固定 `DT=0.0` セットが招いたのと同じ罠である。
したがって `max_cand` の上限は golden の 12 でしか決まらず、*混雑した*帯域
にある弱い信号のための正当な計測器はまだ存在しない — 実際の golden 録音に
SNR を振った信号を注入すれば作れるだろう。

### 21.3 何が変わったか

`ft4_wsjtx_samples::bench_assets::SYNC_MIN` は今 1.2 であり、
`embedded-poc/assets/ft4_golden_candidates.bin` は再焼き込みされた:
**31 候補 → 12**、`DecodeDepth::EMBEDDED` でも `FULL` でも同じ 11 デコード。
§17-19 のデバイスの数値はすべて 31 候補で測定されたものであり、候補あたりの
数値はそのまま引き継がれるが、スロット合計は引き継がれない。

これを §20 の DDC (2 251 ms のダウンサンプル段を丸ごと除去する) と
組み合わせたスロットの予測:

```text
(2 492 search + 1 943 LLR/BP) × 12/31 ≈ 1 717 ms   vs 1 960 ms budget
```

**紙の上では、初めて予算内に収まる。** 但し書きが 2 つあり、うち 1 つは
同じ日に届いた:

- 紙の上、という点が肝心である: DDC も小さくなった候補リストも実機では
  まだ動かしておらず、予測が測定の前にどれほどの価値しか持たないかは、
  このファイル自身の §19.2 が常に思い出させてくれる。
- **12 候補は静かな帯域の数字である。** §23 は平均 15-18、混雑した帯域では
  最大 25 と測定しており、同じ予測は予算を **1.2-1.7× 超える** ことになる。
  上の行は軽トラフィックの場合であり、FT4 が想定する contest の場合では
  ない。

### 21.4 余談: OSD は元を取っている、ただし golden 上ではない

同じパスで両方の depth を測った。golden では `EMBEDDED` と `FULL` は
同一にデコードする — ベンチの「OSD は何も買わない」という一行の出所は
そこである。弱い 560 スイープファイルでは:

| depth | decodes of 560 |
|---|---:|
| `DecodeDepth::FULL` | 237 |
| `DecodeDepth::EMBEDDED` | 179 |

つまり出荷構成は crossing で **58 デコード、recall の 4 分の 1** を手放して
いる。golden の信号は、単に OSD を必要としないほど強いだけである。
組込み FT4 レシーバが OSD にコストを払うべきかは、確定した問題ではなく
実際に問うべき問題になり、上の予算と同じ枠に属する: 候補数の削減で得られる
分は、おおよそ OSD のコストの大きさに等しい。

## 22. ラダーのどの rung がコストに見合うか (2026-08-30)

§21 は候補数を半減させた。このセクションはスロット予測のもう半分、
LLR + BP + OSD の 1 943 ms を扱う。FT4 は 4 つの BP rung —
`llra` (nsym=1)、`llrb` (nsym=2)、`llrc` (nsym=`LLR_NSYM_MAX`=4)、
`llrd` (nsym=1 bit-normalised) — を登り、その後 `depth.osd` なら、
4 つすべてを depth 2 または 3 の OSD で再試行する。

FST4 自身のアブレーションでは、その `nsym=8` rung が BP コストの ~99 % を
占め、`llrd` は recall に一度も寄与しなかった。どちらの主張も移植できない:
FT4 の `nsym=4` rung はグループあたり 4⁴ = 256 のトーン仮説を列挙するが、
FST4 は 4⁸ = 65 536 である。

### 22.1 弱いデータでの測定

`tests/ft4_llr_ladder_ablation.rs`、560 スイープファイル (4 チャネル ×
7 SNR タグ × 20 trial) で各チャネルの 50 % crossing をまたぎ、ファイルごとに
フロントエンドを 1 つ共有し、候補は golden の周波数へオラクルでフィルタして
あるので、候補選択が rung の問いを汚染しない:

| config | decodes | vs FULL | CPU |
|---|---:|---:|---:|
| `abcd`+OSD (= `FULL`) | 235 | +0 | 1.00× |
| **`abc`+OSD** | **235** | **+0** | **0.78×** |
| `abd`+OSD | 189 | −46 | 0.87× |
| `ab`+OSD | 189 | −46 | 0.59× |
| `a`+OSD | 132 | −103 | 0.48× |
| `abcd` (= `EMBEDDED`) | 179 | −56 | 0.10× |
| **`abc`** | **179** | **+0 vs `abcd`** | **0.08×** |
| `abd` | 136 | −99 | 0.07× |
| `ab` | 136 | −99 | 0.05× |
| `a` | 79 | −156 | 0.03× |

ハーネスの自己検査: そのハーネスの `abcd`+OSD はパイプライン自身の 237 に
対して 235 に達し、`abcd` は 179 にちょうど一致する。欠けた 2 デコードは、
ハーネスが再実装していない depth-4 Top-K OSD 段によるもので — 図らずも
その段が 237 中 +2 であることを測定している。

3 つの知見を、大きい順に:

- **`llrd` は何も寄与しない。** OSD ありで 235 → 235、なしで 179 → 179、
  実際の golden では 11 → 11 distinct である。それでいてラダーのコストの、
  OSD ありで 22 %、なしで 20 % を占める。その LLR 自体はタダである
  (`compute_llr_fast` が `llra` と一緒に返す); コストになるのは BP 呼び出しと
  OSD バリアントである。
- **`llrc` が recall の rung である** — 外すと OSD ありで 46 デコード、
  なしで 43 デコードを失う。FST4 の `nsym=8` とは逆であり、FST4 の結果を
  ここにそのまま当てはめることができなかった理由である。
- **OSD は BP 側のコストの ~10× で +56 デコードである。** 実機ではその比は
  すでに測定済みである: `ft4-bench` は 31 候補で `FULL` が 10 987 ms、
  `EMBEDDED` が 8 642 ms と報告しており、つまり OSD は ~2 345 ms で、
  12 候補では 1 960 ms の予算のうち ~900 ms にあたる。

  56 デコードは、このファイルの他の部分が引用に使っている単位ではないので、
  cell ごとの recall で換算する。§22.4 の表から各チャネルの 50 % crossing を
  補間すると:

  | channel | `FULL` | `EMBEDDED` | OSD is worth |
  |---|---:|---:|---:|
  | awgn | −16.89 dB | −16.62 dB | 0.27 dB |
  | ccir_good | −17.42 dB | −17.00 dB | 0.42 dB |
  | ccir_moderate | −15.67 dB | −14.75 dB | 0.92 dB |
  | ccir_poor | −16.00 dB | −14.33 dB | **1.67 dB** |

  `FULL` 列は `sweep-baseline.json` に保存された crossing
  (−16.89 / −17.46 / −15.71 / −16.00) を、まったく別のハーネスから
  0.04 dB 以内で再現しており、上の数値が言っている通りの意味を持つことの
  確認になっている。**OSD はフェージングチャネルの段である**: AWGN では
  4 分の 1 dB、CCIR-poor では 2 dB 近い。予算に収めるために OSD を落とす
  組込み FT4 レシーバは、一様な 25 % ではなく、経路に応じておよそ
  0.3-1.7 dB を差し出していることになる。

### 22.2 FT4 で `llrd` を落とすのはトレードではなく忠実性の修正である

WSJT-X の FT4 デコーダには 4 つ目の*ブラインド*バリアントがない。
`ft4_decode.f90:341-342` は `llrd` を `ipass > 3` の場合にだけ、
`llrd = llrc` の先頭 29 ビットを a-priori パターンで上書きしたものとして
作る — それは **AP** バリアントである。4 つ目のブラインドな
`llrd = scalefac*bmetd` は FT8 の形 (`ft8c.f90:192`) であり、汎用ラダーが
それを継承して FT4 にも適用していた。このクレート自身の AP 経路
(`msg::pipeline_ap`) は `llr_set.llrd` を WSJT-X とまさに同じ目的に使って
おり、手を付けていない。

そこで `process_candidate_basic_impl` は、`P::ID == Ft4` のとき、BP の
階段と OSD のバリアントリストの両方でブラインドな `llrd` rung を
スキップするようになった。検証済み: golden は single-pass で 11/14、
SIC ありで 14/14 のまま変わらず、phantom はゼロ。上のアブレーション。
そして `run-sensitivity-sweeps.sh ft4` は**4 チャネルすべてで +0.00 dB**
(各 160 trial)。

### 22.4 OSD の数値の裏にある cell ごとの recall

cell あたり 20 trial の recall、同じ 560 ファイル:

```text
FULL (abcd+OSD)          m14  m15  m16  m17  m18  m19  m20
awgn                      20   20   18    9    5    1    0
ccir_good                 20   20   19   15    3    0    0
ccir_moderate             19   14    8    3    3    0    0
ccir_poor                 17   11   10    0    0    0    0

EMBEDDED (abcd)          m14  m15  m16  m17  m18  m19  m20
awgn                      20   20   18    5    2    0    0
ccir_good                 20   20   15   10    0    0    0
ccir_moderate             16    8    2    2    0    0    0
ccir_poor                 12    6    3    0    0    0    0
```

### 22.3 古くなっていた記述の訂正

`DecodeDepth::osd` の doc コメントは、OSD が「host-only」であり、
「`fft-rustfft` 以外のビルドからは完全にコンパイルアウトされ」、
「恒久的なアーキテクチャ上の境界」だと述べていた。コードにはそのどれも
ない — `fec::ldpc::osd` にもパイプラインの OSD ブロックにも FFT バックエンドの
`cfg` は付いていない — し、FST4 (#306) と FT4 の両ベンチが ESP32-S3 上で
`DecodeDepth::FULL` を動かしてそのコストを報告している。組込みでの OSD は
予算上の判断であり、§21 の後ではそれは現実に効く判断になった:
crossing での 56/560 デコードに対して ~900 ms である。

## 23. バンド占有率 — 設計点、そして予算が実際にどこにあるか (2026-08-30)

§21.2 は、足りないものを名指しして閉じていた: 「*混雑した*帯域にある弱い
信号のための正当な計測器はまだ存在しない」。それは今や存在し、しかも
すでに WSJT-X ツリーの中にあった — `lib/ft4/ft4sim_mult.f90` は N 個の
信号を 1 つのスロットに敷き、それぞれが自身の SNR と周波数を持ち、それぞれ
**±0.5 s のランダムな DT** を持ち、グラウンドトゥルースを出力する。
`scripts/build_ft4sim.sh` は今やこれをリンクし、
`scripts/gen_ft4_mult_wavs.sh` がこれからコーパスを構築する: 5 種の占有率 ×
50 スロット × (5 / 10 / 14 / 20 / 30) 信号、300-2600 Hz にわたり、計 3 950
のグラウンドトゥルース信号が 45 MB に収まり、生成に ~5 秒、すべての信号の
SNR、DT、周波数、メッセージを収めた `manifest.tsv` が付く。メッセージは
upstream 自身の `lib/ft4/messages.txt` — 実際の 40 m のデコードログ —
から取っている。

AWGN のみ — このシミュレータにはフェージングがないので、それについては
`ft4_sweep` の CCIR チャネルが引き続き計測器となる。

### 23.1 どの占有率を想定して設計するか

**contest ではない**。そして正直な答えは、**FT4 の占有率が実際にどうなのか、
ここでは誰も知らない**ということである。ESP32 のレシーバは飽和した contest
バンドを運用しようとはしないし、そこでの recall は要件ではなくトレード
である。重要なのは 40 m の普通の賑やかな夜である。

利用できる唯一の拠り所は、見た目より弱い。§23.5 は、WSJT-X の FT4
サンプルが 7.080 MHz の実際のデコードログを描画したものであり、
**300-2700 Hz の探索帯域に 14 信号**あることを示す — だがそのログの日付は
`190106` で、その行自身が `Rx FT8` と言っており、FT4 は WSJT-X 2.1 の
2019 年 7 月 15 日まで公開すらされていなかった (`Release_Notes.txt`)。
つまり 14 は **FT8 の 40 m 占有率を FT4 として描画したもの**であり、FT4 に
とっては設計点ではなく上限である: FT4 のユーザ層は FT8 のごく一部であり、
contest 以外ではそのサブバンドは通常まばらである。

このリポジトリにも WSJT-X ツリーにも、実際の FT4 のバンド占有率を測った
ものはない。したがってコーパスは、ある数値を狙うのではなくそれを*挟み込む*
ものであり、組込みレシーバのために読むべき行は疎なものである:

- **5-10 信号** — FT4 で妥当な普通のケース
- **14** — 40 m での FT8 の密度、つまり FT4 としては悲観的なケース
- **20-30** — FT4 がそこに達する証拠のない上限

これを決着させるには FT4 のサブバンドに合わせた無線機からの録音が必要で、
これは §23.5 が最後に行き当たる、欠けている測定と同じものである。

| signals/slot | mean candidates | max | deepest decoding rank | rank p90 |
|---:|---:|---:|---:|---:|
| 5 | 5.3 | 8 | 6 | 4 |
| 10 | 9.2 | 13 | 10 | 7 |
| **14 — the 40 m snapshot** | **12.3** | 18 | 15 | 10 |
| 20 | 14.8 | 21 | 19 | 12 |
| 30 (contest stress) | 17.3 | 23 | 22 | 14 |

**結局、golden の 12 候補は正しい数だった** — 14 信号の帯域が生み出すのが
それであり、golden とはそれだからである。したがって §21.3 の予測は妥当な
範囲全体で成り立ち、FT4 が見た証拠のない密度でしか予算線に達しない:

| band | candidates | projected slot | vs 1 960 ms |
|---|---:|---:|---|
| **5 signals — plausible FT4** | **~5** | **~700 ms** | **inside, 2.8× margin** |
| **10 signals — plausible FT4** | **~9** | **~1 180 ms** | **inside, 1.7× margin** |
| 14 (FT8 density, pessimistic) | ~12 | ~1 570 ms | inside |
| 20 | ~15 | ~1 960 ms | at the line |
| 30 (no evidence it occurs) | ~17 | ~2 220 ms | 1.1× over |

候補数の増え方は信号数よりはるかに遅い — `sync_min = 1.2` は上限では
なくベースライン正規化したスペクトルに対する閾値なので、劣線形になる。
帯域の占有率を 14 から 30 へ倍増させても、候補は 110 % ではなく 40 %
増えるだけである。

**`max_cand` は生き残らないパラメータである。** デコードは設計点でランク 15、
ストレス下で 22 に達するので、§21.2 が golden から導いた上限 (12) を設定値
として適用してはならない — 占有率 20 では実在の信号を切り捨ててしまう。
これは 100 のままにする。

### 23.2 占有率が引き起こすのは感度ではなく干渉である

グラウンドトゥルースに対する recall。single pass (`dual_core` が走らせる
形 — subtract 経路なし) と、プロダクションの `sic_rounds(2)` を比較する:

| signals/slot | single pass | `sic_rounds(2)` | SIC is worth |
|---:|---:|---:|---:|
| 5 | 217/250 (87 %) | 241/250 (96 %) | +9 pts |
| 10 | 371/500 (74 %) | 438/500 (88 %) | +14 pts |
| **14** | **477/700 (68 %)** | **568/700 (81 %)** | **+13 pts** |
| 20 | 565/1000 (56 %) | 721/1000 (72 %) | +16 pts |
| 30 | 626/1500 (42 %) | 888/1500 (59 %) | +17 pts |

運用者の言葉で言えば、FT4 が妥当に見るであろう占有率では、subtract 経路の
ないボードは 5 局中約 **4.4 局**、または 10 局中 **7.4 局**を報告するのに
対し、フルデコーダは 4.8 局と 8.8 局を報告する。それは 1 スロットあたり
**半局から 1 局半の差**であり — 悲観的な FT8 密度の行では、14 局中 9.5 局
に対して 11.3 局である。

これは組込みの subtract の問いを組み替える。飽和したバンドでは不可欠
だろうが、FT4 が妥当に走る密度では、追加のデコードパスを丸ごと払って
1 スロットあたりおよそ 1 局が得られるだけである。

その仕組みは、弱い側ではなく、SNR ごとの表の強い側に見える:

```text
                 -17  -16  -15  -14  -13  -12  -11  -10   -8   -6   -3    0   +5  +10
single pass, 14   47%  61%  63%  70%  71%  72%  71%  66%  70%  62%  61%  70%  76% 100%
sic_rounds(2), 14 56%  66%  72%  80%  86%  82%  88%  82%  88%  76%  78%  84%  98% 100%
```

+5 dB の信号は FT4 自身の閾値より 22 dB 上にあるのに、SIC なしでは 4 回に
1 回見逃される。これは感度ではない — 1 つの信号が別の信号の 83 Hz の占有
帯域幅の中に居座っているということであり、golden の 11/14 → 14/14 が
数百ではなく 3 信号で記録しているのと同じ効果である。

こうして組込みのトレードは仮定ではなく定量化され、§23 の最初の草稿が
残した向きとは逆を指す: **妥当な FT4 占有率では、subtract 経路がないことの
コストは recall の 9-14 ポイント — 1 スロットあたり約 1 局 — で、それと引き
換えにデコードパス丸ごと 1 回分を節約できる。** スロット予算が 1 960 ms の
レシーバにとって、これは妨げではなく擁護しうるトレードである。妨げに
なるのは、FT4 が達することを示されていないバンド密度においてのみである。

### 23.3 単一ファイルを超えて初めて測った precision

両アーム・5 つの占有率すべてにわたり、**5 118 デコード中 phantom 6 件
(0.12 %)**、グラウンドトゥルース信号は 3 950 — 設計点で約 50 スロットに
1 回、contest ストレスで 12 スロットに 1 回の誤デコードである。FT4 の
`max_extra: 0` という予算は、単一の golden ファイルでしか確認されたことが
なかった; 真の率は小さいがゼロではなく、運用者が目にする前に知っておく
価値がある。

### 23.4 このコーパスがまだ見えないもの

- **フェージングなし。** `ft4sim_mult` はガウス雑音しか加えない。
- **SNR 下限 −17 dB** — シミュレータは `isnr` をそこでクランプし、それは
  FT4 自身の閾値なので、深く弱い裾には手が届かない。
- **DT は ±0.5 s で一様**であり、これはちょうど §18 の狭めた探索窓の内側に
  収まるので、このコーパスではその窓を試すこともできない。
- **golden と同じジェネレータである** (§23.5) ので、独立した計測器ではない
  — upstream がすでに選んだ場面の周りで、場面のパラメータを 1 つずつ変える
  ための手段である。

### 23.5 「実際のオフエアの golden」はシミュレートされた場面であり — upstream に他のものはなかった

平易に述べておく価値がある。このツリーのいくつかのコメントは別のことを
言っていたからだ: `WSJT-X/samples/FT4/000000_000002.wav` は
**`ft4sim_mult` の出力**であり、`lib/ft4/messages.txt` の `File 2` ブロック
から生成されている。

ツリー内の 4 つの証拠があり、どれも外部の情報源を必要としない:

1. ファイル名がシミュレータ自身の `000000_%06d.wav` パターンである。
   `WSJT-X/samples/` にある*他のすべての*プロトコルのサンプルは、実際の
   UTC の録音タイムスタンプを持つ — `FT8/210703_133430.wav`、
   `JT9/130418_1742.wav`、`WSPR/150426_0918.wav`、
   `FST4+FST4W/210115_0058.wav`、`MSK144/181211_120500.wav`。持たないのは
   FT4 だけである。
2. `File 2` の 19 行がファイルを正確に再現する (下記)。
3. それらの行の日付は `190106` で、その文面自身が `Rx FT8` と言っている。
4. FT4 は **WSJT-X 2.1、2019 年 7 月 15 日** に導入された
   (`Release_Notes.txt`) — そのログが録られてから 6 か月*後*である。

つまりこの場面は 40 m の実際の **FT8** バンドのスナップショットであり、
当時は録音すべき FT4 のトラフィックがなかったため FT4 として描画された
ものである。これは upstream への批判ではない; 新しいモードのために彼らが
できた唯一のことである。だが同時に、広く引用される「FT4 のサンプル録音」
は FT4 のバンド占有率についての証拠ではなく、このリポジトリにも WSJT-X
ツリーにもそれを含むものはない、ということである。

```text
   297 Hz   -9 dB  N1TRK N4FKH 569 VA        2300 Hz  -13 dB  AC6BW KR9A R 559 WI
   422 Hz   -9 dB  N1TRK KB7RUQ RR73         2310 Hz   -1 dB  WD9IGY KX1X 73
   520 Hz   -9 dB  W9JA PY2APK RRR           2413 Hz  -17 dB  W7BOB KJ7G RR73
   560 Hz   -8 dB  CQ RU AB5XS EM12          2560 Hz  -12 dB  CQ RU W0FRC DM79
   727 Hz  -12 dB  NZ7P WA7JAY 589 CA        2567 Hz   -7 dB  NI6G W7DRW 569 AZ
  1148 Hz  +16 dB  KB0VHA KA1YQC R 539 MA    ─── above the 300-2700 search band ───
  1640 Hz   -3 dB  CQ RU N9OY EN43           2725 Hz   +3 dB  K4SQC VE3RX RR73
  1910 Hz  -10 dB  K1JT WB4HXE 559 GA        2813 Hz   -5 dB  CQ RU W1QA FN32
  2067 Hz   +6 dB  VE3LON K7RL R 549 WA      2995 Hz   +1 dB  CQ RU WS4WW FM17
                                             3158 Hz  +14 dB  HB9BUN KG4W R 549 VA
                                             3337 Hz   -7 dB  W9TO KN3ILZ 529 PA
```

3 つの帰結:

- **ファイルには 14 ではなく 19 の信号がある。** `FT4_FULL_REFERENCE` の
  14 は探索帯域の内側にあるものであり、残る 5 つは 2700 Hz より上にあるので、
  `jt9 -H 2700` もこのクレートもそれらを報告することはない。「jt9 との
  14/14 パリティ」とは、共有された帯域にわたる参照デコーダとのパリティで
  あって、ファイルに含まれるものの 100 % ではない。
- **そのグラウンドトゥルースは正確である** — 信号ごとの SNR、DT、周波数が、
  同じ `messages.txt` から得られる。報告された SNR は 14 件にわたって真値と
  数 dB 以内で一致しており、これは誰も利用可能だと気づいていなかった
  `pipeline::ft4_snr_db` の独立した検査になる。
- **このリポジトリにも upstream にも、実際のオフエアの FT4 録音はない。**
  「実信号」についての FT4 の主張はすべて、実際の *FT8* デコードログを描画
  したものの上に立っており、それは妥当な場面 (占有率、SNR の広がり、
  7.080 MHz でのメッセージの混ざり具合) を運ぶが、アーティファクト —
  ドリフト、スプラッタ、LO オフセット、平坦でない雑音 — は何も持たず、
  FT4 よりはるかにユーザの多いモードを描いている。これを解消するには FT4
  のサブバンドに合わせた無線機からの録音が必要である。このプロジェクトには
  それがある (すでに USB オーディオで CoreS3 に給電している IC-705) ので、
  FT4 ライン全体で残っている測定のうち最も安価なものになる。

## 24. ホストのシミュレーションはボードの代役になるか? FT4 では、なる (2026-08-30)

実際のオフエアの FT4 は録音しにくい — FT4 の活動は、upstream にも録音が
なかった (§23.5) ほど薄く、それが彼ら自身のサンプルがシミュレートである
理由である。したがってシミュレーションが計測器であり、答える価値のある
問いは「実データはいつ手に入るか」ではなく、**ホストがデバイスのどれだけを
再現するか**である。

FT4 についての答えは、算術のすべて、ということになる。

### 24.1 `fixed-point` は FT4 では no-op である

CoreS3 のビルドは `mfsk-core/fixed-point` を有効にしている
(`m5stack-cores3-app` での `cargo tree -f '{f}'` がそれを列挙する) ので、
「ボードは `SpecCell = u16` の量子化でデコードするので、ホストの f32 の
数値は移転できない」と読める。だが FT4 については、そうではない。

```sh
grep -rl 'feature = "fixed-point"' mfsk-core/src --include=*.rs
```

は 6 ファイルを返す: `ft8/decode.rs`、`ft8/decode_block.rs`、
`ft8/decode_block/{spectrogram,coarse_sync,process_candidates}.rs`、
そして `engine/fft.rs` — ここでの唯一の箇所は `default_planner_16()`
であり、`decode_block` だけが使う i16 プランナである。**汎用の
`engine::pipeline` 経路にはこれでゲートされているものが何もなく**、FT4
(FST4 と同様) は完全にその経路上にある: `ft4_coarse_sync` 自身の f32
ピリオドグラム、`downsample_cached`、`ft4_sync_search`、`symbol_spectra`、
LLR ラダー、BP、そして OSD。

読んだだけでなく測定もした: `MFSK_SWEEP_FEATURES=full,internal-testing,fixed-point`
の下での `run-sensitivity-sweeps.sh ft4` は、f32 の crossing を桁まで
再現する — −16.89 / −17.46 / −15.71 / −16.00 dB、各 160 trial、
4 チャネルすべてで `+0.00 dB`。golden テストも変更なしで通る。これは
feature についての事実であり、偶然ではない。

したがってボードは FT4 を**ホストと同じ f32 の算術**で走らせており、
§21-23 のすべての recall の数値はボードの数値でもある。

### 24.2 ホストとボードの間に残るもの

- **FFT カーネル。** ホストは rustfft を使い、ボードは esp-dsp の radix-2
  アセンブリとこのクレートの `fft_mixed_5120` / `fft_mixed_2304` ラッパー
  を使う。どちらも f32 で、違いは丸めと演算順序だけである。それが問題に
  ならないという証拠はすでに記録にある: `ft4-bench` は
  `DecodeDepth::EMBEDDED` でも `FULL` でも、**ホストと同一の 11 distinct
  メッセージ**をデコードした (§17)。
- **`dotprod-extern`。** `ft4_sync_search` の内積は、LX7 では
  `dsps_dotprod_f32_aes3` を通り、ホストではスカラーループを通る。§19.1 の
  `FlatRef` 変更と同じ再結合の議論であり、それはスイープ全体に対して
  +0.00 dB で確認済みである。
- **算術より上のすべて** — スロットのタイミング、オーディオキャプチャ、
  メモリ圧迫 — であり、それこそがデバイス実行が測るもので、ホストのテスト
  では測れない。

### 24.3 FT4 の証拠基盤の現状

| question | instrument | status |
|---|---|---|
| sensitivity curve, 4 channels | `ft4sim` sweep, 160 trials/channel | measured, baseline-tracked |
| Δt reach | `ft4sim` DT sweep (§18) | measured |
| band occupancy / interference / precision | `ft4sim_mult` (§23) | measured, 3 950 signals |
| numeric path host vs board | feature audit + fixed-point sweep (§24.1) | **identical for FT4** |
| FFT kernel host vs board | `ft4-bench` on the golden (§17) | identical decodes |
| slot timing on hardware | `ft4-bench` | measured end-to-end, 12 candidates + DDC (§25): **2.33x over budget** |
| real receiver artefacts | — | **no instrument, here or upstream** |
| true FT4 band occupancy | — | **no instrument, here or upstream** |

最後の 2 行は、より良いシミュレータが片付けてくれる作業項目ではない。
それらはこのラインが主張できることの誠実な境界であり、コーパスは、それらが
答えられたふりをするのではなく、挟み込むように作られている。
## 25. ハードウェア上のスロット全体 — 三つの見積りはすべて外れた (2026-08-30)

`ft4-bench` が、CoreS3 上で FT4 のスロットをエンドツーエンドで初めて
走らせた。`ft4_coarse_sync` は `fft_mixed_2304` 経由でデバイス上で計算し、
候補ごとのベースバンドは `ft4::ddc` で作る。経路上に baked asset は無く、
あるのは 90 000 サンプルの音声だけである。ログ:
`m5stack-cores3-app/logs/ft4-bench_wholeslot_2026-08-30.log`。
240 MHz、`opt-level = 3`、シングルコア、候補 12 件。

### 25.1 うまくいった点

**デコード: 5 つの arm すべてで 11 件**。どの深さでも、どちらの窓でも
host と同一。FFT arm との差は `K1JT WB4HXE` の 1910.6 Hz 対 1909.6 Hz
だけで、これは `ft4_ddc_equivalence` が host 上で 11 件中 1 件として
既に固定している、1 Hz の探索グリッド 1 ステップ分である。

**粗同期カーネルは正確である。** `compare_candidates` はデバイス側の
候補 12 件すべてを host で baked した一覧と突き合わせ、**最大 Δfreq
0.00 Hz、最大 Δscore 0.00 %** だった。2304 点変換の 2 通りの因数分解
— host の rustfft のプランナと、ここでの esp-dsp の
`dsps_fft2r_fc32_aes3_` 上の 256 × 9 — でも、ピーク選択はまったく
動かない。ビット一致は最初から期待できなかった。「同じピークが同じ
場所で `SYNC_MIN` を超える」と予測した節が期待してよい水準より、
これは良い結果である。

**広帯域キャッシュは確かに無くなった。** DDC の arm は空の `fft_cache`
を渡し、同じ 11 件をデコードする。host 上で
`ft4_ddc_arm_never_reads_the_wideband_cache` が assert している内容が、
ボード上でも成り立つ。

### 25.2 うまくいかなかった点 — 予算

| arm | coarse | candidates | slot | vs 1 960 ms |
|---|---:|---:|---:|---:|
| FFT EMBEDDED | 1 288 | 2 839 | 4 127 | 2.11× |
| FFT FULL | 1 288 | 2 831 | 4 119 | 2.10× |
| DDC EMBEDDED (±1.0 s) | 1 288 | 3 856 | 5 144 | 2.62× |
| DDC FULL (±1.0 s) | 1 288 | 3 853 | 5 141 | 2.62× |
| **DDC EMBEDDED (±0.5 s, ship)** | **1 288** | **3 287** | **4 576** | **2.33×** |

§21.3 は `(2492 + 1943) × 12/31 ≈ 1717 ms` と見積り、これを「予算内に
収まった、紙の上では初めて」と呼んだ。測定した ship 構成は
**4 576 ms — その見積りの 2.7 倍** である。互いに独立した 3 つの誤りが
あり、どれも算術の誤りではない:

**1. 粗同期ステージは見積りに一度も入っていなかった。** baked されて
いたうえ、本ファイル (§20) もベンチ自身の doc も「host では 0.3 ms
なので、その不在がデバイス合計を過小評価するのは、それにデバイス/host
比を掛けた分程度 — 小さい」として片付けていた。実際は **1 288 ms** で
ある: スロットの 28 %、予算全体の 66 % をこれ一つで占める。スロットあたり
152 回の変換で、1 回およそ 8.5 ms。host の数値が何を測っていたにせよ、
このステージのデバイス/host 比は、候補ごとのステージが示す約 30 倍とは
似ても似つかず、これを無視してよいとした外挿には根拠が無かった。
これは [`feedback_bottleneck_hypothesis_measure_first`] の三度目である:
測定ではなく議論によって除外されたステージ。

**2. DDC はここでは FFT フロントエンドより 2.3 倍 *遅く*、速くはない。**
`candidate_baseband` は 1 候補あたり 154 006 µs、`downsample_cached` は
66 974 で、**スロット全体で +1 044 ms** になる。§20 は DDC の *忠実度*
(0.0 dB、同じデコード) と host コストを測っており、組込みでの採用理由は
ボードが実行できない変換を取り除くことだった。その理由は述べたとおり
そのまま成り立つ — 受信機を成立させるのはそれである — が、これは
速度ではなく **実現可能性** の勝利であり、§21.3 の見積りは
`downsample_cached` の 2 251 ms の行を、静かに回収済みとして扱っていた。
実際には回収されていない: DDC はそれをより高価なものに置き換える。
理由は、候補ごとに 90 000 サンプルへかかる 199 + 263 個のスカラ f32
タップであり、`dsps_fird_f32_aes3` が未活用の明白なレバーである。

**3. 帯域の絞り込みは §19 が示唆したほど効かない。** 探索単体での 1.58 倍
(1 547 → 960 ms) は §18-19 を再現するが、探索はいまやスロットの大半では
なく 21 % なので、ship arm の得は 569 ms、合計では 1.17 倍にとどまる。

あわせて確定したこと: **internal-DRAM への `cd0` 配置は死んだ**。ここでは
1.01 倍で、§19.2 の 1.12 倍に対し、量産モードが行わねばならない 40 KB の
確保に見合わない。また **クリーンな候補リストでは OSD はただ同然である**:
`FULL` と `EMBEDDED` の差は 2 つのフロントエンドでそれぞれ 8 ms と 3 ms。
§22 は 31 候補で OSD をおよそ 2 345 ms としたが、それは 19 個のノイズ
候補がそれぞれラダー全体を回して失敗し、OSD に落ちたものだった。
`sync_min = 1.2` では、ほぼすべての候補が BP でデコードできる実信号で、
OSD は走らない。**OSD のコストは候補数ではなく失敗数に比例する** —
つまり §21 の「組込みは OSD を残すか」という問いは、ラダーの ablation が
見せたよりずっと安く済む。

### 25.3 4 576 ms は実際どこにあるか

| stage | ms | share |
|---|---:|---:|
| `ft4_coarse_sync` | 1 288 | 28 % |
| `candidate_baseband` (DDC) | ~1 848 | 40 % |
| `ft4_sync_search` @ ±0.5 s | ~960 | 21 % |
| LLR + BP | ~479 | 10 % |

探索と LLR/BP を合わせて 1 439 ms で、1 960 ms の予算に余裕を持って
収まる。超過はすべて、ハードウェアで一度も測られていなかった 2 つの
ステージにある。どちらにも未試行のレバーがある — DDC には esp-dsp の
FIR、粗同期ステージには、256 点の内側パスが既に esp-dsp のものである
のに 152 回の変換が 1 回あたり 8.5 ms かかるのはなぜか、という問い —
なので、2.33 倍は判決ではなく出発点である。§23 の結論、すなわち
*候補数* が現実的な占有度では素直に振る舞うという点は、ここでも
変わらない。変わるのは、残りの作業がどのステージにあるかである。

### 25.4 PIE のステージングコピーではない (2026-08-30)

粗同期ステージが 1 回の変換あたり 8.5 ms かかる原因として挙がった 3 つの
容疑のうち最も安く確かめられるのは、その内側の行がどれも esp-dsp の
インプレース PIE パスに乗っていない、というものだった:
`MixedRadix2304Fft::process` は、呼び出し側のバッファが 16 バイト
アラインでないときは常に `AlignedStaging` 経由でコピーする。そして
`symbol_spectra_avg` が渡すのは素の `vec![Complex::new(0.0, 0.0);
NFFT1]` で、その *保証される* アラインメントは
`align_of::<Complex32>() = 4` である。esp-idf のアロケータがそれでも
16 を返すかどうかは、host のコードには答えられない問いだ。

`pie_alignment_report()` は issue #260 のために既にあったが、**`record_pie_path`
を呼んでいたのは素の radix-2 パスだけ** で、3 つの mixed-radix カーネル
— つまりこのボードで FT4 と FT8 が実際に走らせるすべての変換 — からは
見えなかった。それらを配線し、各パスの前後でカウンタを読んだ
(ログ: `ft4-bench_piealign_2026-08-30.log`):

| pass | inner rows | in-place | staged |
|---|---:|---:|---:|
| pass 0 — coarse, 2304 = 256 × 9 | 1 368 | **1 368** | **0** |
| pass 1f — `downsample_cached`, 5120 = 1024 × 5 | 60 | 0 | **60** |
| pass 1d — DDC (FIR only) | 0 | 0 | 0 |

1 368 = 152 × 9、60 = 12 × 5 なので、カウンタは見るべき行をちょうど見て
いる。pass 1d のゼロは、各パスがカーネルをちょうど一つだけ走らせている
ことのセルフチェックである。

**仮説は粗同期ステージについては死んだ**: 内側の 1 368 行はすべて、既に
PIE アセンブリ上でインプレースに走っている。したがって 1 288 ms は
カーネルではなく *ラッパー* の中にある — 2 304 点にわたる Nuttall 窓、
256 点の行 9 本のストライド付き gather/scatter、2 304 回のツイドル乗算、
1 152 ビンにわたる振幅の累積、いずれもスカラの Rust である。256 点の
esp-dsp 変換 9 本は、8.5 のうちおよそ 0.2 ms のオーダーにすぎない。
**次のプローブは combine ステージを直接計時する**; すでにコストの約 2 %
であるカーネルを最適化しても意味は無い。

**そして別の欠陥が見つかった**: `fft_mixed_5120` は行の **100 %** を
ステージングしている。5 120 要素のバッファは 16 を法として 8 の位置に
置かれるので、1 024 点の行はすべて入りと出で 1 回ずつコピーを払う。
これは実在し、修正法（バッファのアラインメント）も分かっているが、
それが何ではないかに注意: `downsample_cached` はここでは対照 arm で
ある。出荷する受信機は DDC を使い、これを呼ばないので、修正しても
予算は動かない — 経路上にあるからではなく、事実だから記録しておく。
コストになるという証拠が出るまで修正は見送る: 8 KB の行 60 本は約 1 MB
のコピーで、803 ms のステージに対するものだから、ここにも「カーネル
ではなくラッパーだ」という同じ結論がおそらく当てはまる。

### 25.5 レイヤ分割 — combine ステージである (2026-08-30)

§25.4 が「ラッパー」に至ったのは消去法であって測定ではなく、しかも
「ラッパー」とは 3 つの別のものを指す。mixed-radix カーネルはいまやそれら
を別々に計時する — `Fft::process` 全体、その中の esp-dsp のアセンブリ、
ステージングのコピー — ので、combine = process − kernel − staging と
なり、呼び出し側が変換の周りで費やす分は、自身のウォールクロックから
process を引いたものである。内側の 1 行あたり `esp_timer_get_time` を
2 回読むが、1 290 に対して 1 ms 未満。ログ:
`ft4-bench_layersplit_2026-08-30.log`。

| stage | total | kernel | staging | **combine** | outside transform |
|---|---:|---:|---:|---:|---:|
| pass 0 coarse (2304) | 1 290 ms | 201 (15 %) | **0** | **853 (66 %)** | 236 (18 %) |
| pass 1f `downsample_cached` (5120) | 803 ms | 29 (3 %) | 34 (4 %) | **508 (63 %)** | 231 (28 %) |
| pass 1d DDC (FIR) | 1 848 ms | 0 | 0 | 0 | 1 848 (100 %) |

**答えは combine ステージである: 粗同期ステージの 66 % と
`downsample_cached` の 63 %。** これは `fft_2304_with` / `fft_5120_with` の
Cooley-Tukey 再結合 — ツイドル乗算と内側の行の間のストライド付き
gather/scatter — で、`mfsk_core::engine::dsp::fft_mixed_*` にあるスカラの
Rust である。853 ms と 508 ms、2 つのステージ合わせて 1 361 ms で、予算は
1 960 ms。

そのスループットが手掛かりである: 2304 の combine は 1 変換あたり約
5.6 ms、およそ 30 k flop に対して、つまり **240 MHz の FPU で約
5 Mflop/s**。算術は何も高価ではない。高価なのはアクセスパターンだ。
これは [`reference_fft_loop_interleave_trick`] が同じシリコン上で一度
扱った形である（FPU に独立した依存連鎖を与えるようループをインター
リーブして、直列の DSP 再帰で −40 %）。

これが §25.4 に強いる 3 つの訂正。§25.4 は測定せずに推測していた:

- **カーネルは「~2 %」ではなく 15 %**。1 368 行で 201 ms は 256 点の行
  あたり 147 µs — C であってアセンブリではない `dsps_bit_rev_fc32_ansi`
  を含めて数えれば妥当だが、私の見積りの 4 倍である。測るための計器を
  いま作ったばかりの数値を推測したのは、弁護できない。
- **5120 のステージング欠陥は実在し、無視できる程度**: 803 のうち
  34 ms、4 %。確認する価値はあったが、直す価値は無い — しかも出荷する
  受信機が呼ばない対照 arm 上にあるので、直すことの正味の価値はゼロ
  である。
- **`outside transform` は 18-28 %** で、容疑者リストにまったく載って
  いなかった: 粗同期では Nuttall 窓と振幅累積 (236 ms)、
  `downsample_cached` では 92 160 ビンの抽出 (231 ms)。

pass 1d の 100 % outside の行がセルフチェックである: DDC は変換を走らせ
ないので、その 1 848 ms は 1 マイクロ秒残らず FIR であり、
`dsps_fird_f32_aes3` が手つかずのレバーとして残っている。

**FT4 の組込みの現在地。** 2 つのレバーがあり、どちらも見積りではなく
測定によるものである: mixed-radix の combine (1 361 ms、しかも
`fft_mixed_3840` と共有なので、このボード上の FT8 も同じ税を払っている)
と、DDC のスカラ FIR (1 848 ms)。どちらもまだ試みられていない。

## 26. combine ステージ: 2 つの誤った理論と 41 % の改善 (2026-08-30)

§25.5 は `ft4_coarse_sync` の 1 290 ms の 66 % を Cooley-Tukey の
combine — `fft_mixed_2304` の転置、ツイドル、9 点の列、すなわち esp-dsp
の 256 点の行の周りのスカラ Rust — に帰した。本節は、それを攻めたときに
何が起きたかである。3 回の試みのうち 2 回は失敗しており、結果より順序
のほうが重要だ。

`fft_*_with` を呼ぶのは `embedded-shared` のプランナとモジュール自身の
ユニットテストだけで、host は 2304 と 5120 には rustfft を直接使う。
つまりこれは組込み専用のコードであり、以下の変更はすべて、flash する前に
host で追加的に **ビット一致** を固定した — デコードは動きようが無く、
実際動かなかった: どの実行のどの arm でも 11 件、`compare_candidates` は
終始 12 件中 12 件を最大 Δfreq 0.00 Hz で対応付けた。

| step | coarse | combine | ship slot | |
|---|---:|---:|---:|---|
| §25 baseline | 1 290 | 853 | 4 576 (2.33×) | |
| blocked transposes | 1 496 | 1 033 | 4 790 (2.44×) | **worse — reverted** |
| scratch hoisted out of the call | 1 165 | 774 | 4 460 (2.27×) | |
| …and 16-byte aligned | 1 138 | 777 | 4 434 (2.26×) | |
| **…in internal DRAM** | **758** | **404** | **4 055 (2.06×)** | **−41 % coarse** |

### 26.1 ブロッキングの試み — 誤りだった

理論: `m[n2·256 + n1]` と列の gather は、1 変換あたり 2 048 バイト間隔の
約 6 900 アドレスに触れ、18 KB のスクラッチは
`CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL` を超えるので Quad PSRAM にある。
1 ライン約 32 バイトとして、18 KB を動かすのに >200 KB のバス
トラフィックになる — 40 MB/s で 5.1 ms、測定値は 5.6 ms。算術は正しい
数字に着地した。

両方の転置を、スタック上にステージした 32 列のストリップにブロック化
すると、**21 % 悪化** した (853 → 1 033 ms)。算術は量については正しく、
メカニズムについて誤っていた: ストライドアクセスは既にうまく捌かれて
おり、ブロック化は何ももたらさず、往復に増えたステージングコピーは
純粋なコストだった。revert した; 残った成果物は、そのために必要になった
`fft_2304_with_scratch` のシームと、ビット一致を固定する `#[test]` で、
revert が安く済んだのはそのおかげである。

「間違った理由で正しい大きさを予測した算術」を、この系列が出したのは
これで三度目 — `feedback_bottleneck_hypothesis_measure_first` も参照。
教訓は、見積りが悪いということではなく、見積りが合うことは証拠に
ならないということらしい。

### 26.2 アロケーション — 十分の一にすぎなかった

呼び出しごとの `vec![Complex32::new(0.0, 0.0); 2304]` は、スロットあたり
152 回の malloc と 18 KB のゼロ埋め 152 回で、しかもステップ 1 が完全に
上書きするバッファに対するものである。これをプランナに引き上げると:
1 290 → 1 165 ms。

これは §25.4 のステージング税を *再導入* もした — 引き上げた `Vec` が
16 を法として 8 の位置に置かれ、呼び出しごとのものにはなかった 29 ms の
コピーが現れた。代わりに `AlignedStaging` 経由で確保するとゼロに戻り、
1 138 ms になった。危険として記録する価値がある: バッファのアライン
メントはそれをどう確保したかの性質であり、確保を移すと黙って変わる。

### 26.3 原因はワーキングセットだった

`symbol_spectra_avg` は 1 変換あたり 3 つの 18 KB バッファを生かしておく
— 入力、ツイドルテーブル、スクラッチ — 合計約 54 KB で、S3 のデータ
キャッシュは約 32 KB。だから 1 つのバッファの *内部* の局所性を改善して
も役に立たなかった: 集合が収まらないので、どの順序で歩いても各パスは
PSRAM からストリームすることになる。

3 つのうち 1 つを外す: スクラッチに `heap_caps_aligned_alloc(16, 18 432,
MALLOC_CAP_INTERNAL)`、失敗時は PSRAM のパスにフォールバック。
**combine 777 → 404 ms、coarse 1 138 → 758 ms。** §25 との累計では、
coarse −41 %、combine −53 %、ship slot は予算の 2.33× → 2.06×。

これは `internal_pool` が FT8 の `cs` スクラッチについて文書化して
いるのと同じ効果で、§19.2 が `cd0` について 1.12× しか測らなかったもの
でもある — 違いはバッファが *何のためのものか* にある。`cd0` はグリッド
セルごとに 1 回ストリームされ、これはキャッシュが喜んで捌く。こちらは
1 変換あたりストライド付きで 5 回走査され、キャッシュには捌けない。

**コストは 18 KB の internal DRAM** で、受信機に必要なのはちょうど 1
つである: 出荷経路は coarse + DDC であり、`fft_mixed_5120` — 上記の
どの実行でも 508 ms のまま手つかずで、だからこそ変更が切り分けられて
いたことを示す対照になっている — は対照 arm 上の `downsample_cached`
に属する。WiFi が上がっているとこのボードの最大の空き internal ブロック
は 31 744 B なので、量産の FT4 モードはこれをオンデマンドで確保する
のではなく、起動時に `worker_arena` 経由で予約しなければならない。

### 26.4 予算はここからどうなるか

ship slot 4 055 ms に対し 1 960。残りを大きい順に:

| stage | ms |
|---|---:|
| DDC (`candidate_baseband`, scalar FIR) | ~1 848 |
| `ft4_sync_search` @ ±0.5 s | ~960 |
| coarse (combine 404 + kernel 144 + window/magnitude 208) | 758 |
| LLR + BP | ~479 |

DDC はいまや最大の単一項目であり、`dsps_fird_f32_aes3` はまだ未試行で
ある。`fft_mixed_3840` はここで直した構造を共有しているので、このボード
上の FT8 の `NFFT_SPEC` パスもおそらく同じ PSRAM 税を払っている —
未測定であり、今回は仮説だと明記しておく。

## 27. DDC: 前提が誤りで、改善は 10 % だった (2026-08-30)

§25.5 は DDC のレバーを「esp-dsp の FIR (`dsps_fird_f32_aes3`) を
バインドする、未試行」と挙げた。何かを書く前に測ってみると、その前提は
既に偽だと分かった: `FirStage::dot` は issue #307 以来
`dotprod::dot_f32` を経由しており、`dotprod-extern` の下ではそれが
`dsps_dotprod_f32_aes3` **そのもの** である。FIR はずっと esp-dsp の上に
いたのだ。

乗っていなかったのは、そのカーネルの PIE パスである。PIE パスは、両方の
オペランドが 16 バイトアラインで、かつ `n % 4 == 0` であることを要求
する。`ft4-bench` の新しい `dotprod_probe` は、アラインメントだけが変数
になるよう internal-DRAM のバッファで測る:

| n | aligned | offset by one f32 |
|---:|---:|---:|
| 196 | **7 936 ps/tap** | 18 104 |
| 199 | 18 082 | 18 081 |
| 200 | **7 900** | 18 073 |
| 204 | **7 869** | 18 046 |
| 260 | **7 520** | 17 751 |
| 263 | 17 740 | 17 739 |
| 264 | **7 501** | 17 736 |

2.3 倍。そして `ft4::ddc` のステージは **199 タップと 263 タップ** —
つまりアラインメントがどうであれ、長さの条件を満たした呼び出しは
これまで一つも無かった。dot は、DDC の 1 候補あたり 154 ms のうち約
81 ms である。

**試み 1、却下: 各窓をアラインされたパディング済みバッファへステージ
する。** どの呼び出しでも両方の前提条件を満たす。測定は **1 848 → 1 937
ms** — コピーのコストが約 2.7 µs で、節約できる dot は約 2.0 µs。
revert した。

**試み 2、採用: タップをパディングし、履歴をアラインし、何もコピー
しない。** タップは 4 の倍数までゼロパディングし（追加される項はちょうど
`0.0·x`）、履歴は `Vec<f32>` — 4 バイトアラインしかないので、4 の倍数の
添字でも 16 バイトアラインにはならなかった — から `repr(align(16))` の
ストアへ移した。窓のアラインメントは依然として `win_start` とともに
巡回するので、これが効くのは、たまたま合った割合だけである: ステージ A
の半分 (`decim = 18`)、ステージ B の 4 分の 1 (`decim = 1`)。

**1 848 → 1 659 ms、−10.2 %。** ship slot は 4 055 → 3 876 ms、2.06× →
1.97×。事前の予測は約 200 ms で、測定は 189 — §26.1 の見積りが外れた
あとの、当たった見積りとして記録する価値がある。

host は算術を厳密なまま保つ: パディングした dot は `dotprod-extern` の
後ろにあり、アラインメントの変更が動かすのは配置であって値ではない。

途中でバグが 1 つ。形が繰り返し現れるので書いておく: 履歴のサイズを
*パディング後の* タップ数に合わせたところ、`compact` がバッファ長を
トリガにしていたため、最後の有効な窓で `win_start + pad` が 1 つ末尾を
越えた。ブートループ、`range end index 777 out of range for slice of
length 776`。トリガは明示的な上限になり、確保にはその上限を越える
余裕としてパディングを含めるようになった。

### 27.1 試み 3: 4 つの事前シフト済みタップテーブル (2026-08-30)

それらの dot のもう半分は、たまたま合った割合ではなく *すべての* 呼び出し
で窓がアラインされている必要があった。`esp_dsp_dotprod` 自身のモジュール
doc は、そのための方法 — 窓の位相ごとに 1 つのタップテーブル、位相 `p`
は `p` 個の先頭ゼロを持ち、dot は 4 アラインである `win_start - p` から
始まる — を既に検討して却下していた: FST4 の広帯域 coarse カスケード
(L = 64) で **168 KB**、空き internal DRAM は約 190 KB。

その算術は FT4 には持ち越せない。これらのステージは 199 タップと 263
タップなので、4 位相でも **両方合わせて約 7.4 KB** である。却下は、
それが書かれた対象のカスケードについては妥当で、ここには単に当てはまら
ない — 注意しておく価値があるのは、その記述が技法そのものに対する一般
的な判決のように読めたからだ。

  DDC 1 659 -> **1 306 ms** (-21 %、§25 が測った 1 848 ms に対しては
  -29 %)。FIR ステージ 122 -> 94 ms、ステージ B 単体は 43 -> 23 ms —
  単独では最大の動きで、`decim = 1` のため窓のうちアラインされて着地
  していたのは 4 分の 1 だけだったからである。ship slot 3 876 ->
  **3 516 ms、1.97x -> 1.79x**。

事前の予測は約 1 300 ms、測定は 1 306。

追加される項はすべてちょうど `0.0 · x` なので、動くのは和の丸めだけで
ある; デコードはどの arm でも 11 件のままだった。host は影響を受けない
— 位相テーブルは `dotprod-extern` の後ろにあり、バックエンドの無い
ビルドは従来どおり単一の逆順タップの dot を保つ。Tier A+B は green
(82 バイナリ)、`wspr-ddc-cascade` の golden も green。

`dsps_fird_f32_aes3` 本体 — ブロック全体を引き受け、スライディング窓を
自分で持つ — は未試行のままで、いまでは見かけより小さい賞品である:
それが吸収するはずの dot は、既に高速パスに乗っている。

### 27.2 一日を終えた FT4 の現在地

| stage | §25 | now |
|---|---:|---:|
| `ft4_coarse_sync` | 1 288 | 758 |
| DDC front end | 1 848 | 1 306 |
| `ft4_sync_search` @ ±0.5 s | ~960 | ~960 |
| LLR + BP | ~479 | ~479 |
| **ship slot** | **4 576 (2.33x)** | **3 516 (1.79x)** |

誰も測っていなかった 2 つのステージが、いま動いた 2 つである。Δt 探索
は手つかずで、大きさの順では次にくる。

## 28. FT8 も同じ税を払っていた (2026-08-30)

§26 は `fft_mixed_2304` を直し、`fft_mixed_3840` — FT8 の `NFFT_SPEC`
カーネル — が同一の構造を持ち「おそらく同じ PSRAM 税を払っている」と
述べたが、未測定の仮説だと明示していた。いま測定した。それも、2 回の
flash をまたぐのではなく **1 回の実行の内部** での A/B である。2 つの
エントリポイントの違いは、ちょうど 1 点だけだからだ:

- `fft_3840_with` は呼び出しごとに `vec![…; 3840]` を確保する — 30 KB、
  PSRAM で、このボード上のすべての FT8 デコードがこれまで使ってきたもの;
- プランナの `Fft::process` は `fft_3840_with_scratch` に、引き上げた
  internal-DRAM のバッファを渡す。

内側の 256 点カーネル、ツイドル、算術はすべて同じ。

```
fft_mixed_3840 x200 — per-call vec (PSRAM) 15 808 µs/xform
                    | hoisted internal      6 478 µs/xform
                    | 2.44x | max |A-B| 0e0
```

**2.44×**、そして出力はビット単位で等しく、これが、配置の変更以外の
何物でもないことを確かめるチェックである。

2 つの注意点があり、どちらも重要:

- **これは変換であって、FT8 のスロットではない。** スロットに対して
  どれだけの価値があるかは測っていない — `ft4-bench` は FT4 のハーネス
  で、このボードには FT8 のベンチが無い。2.44× を FT8 のデコード高速化
  として引用してはならない。
- **WiFi が上がると、最大の空きブロック 31 744 B に対して internal
  DRAM を 30 KB。** ここで成功したのは、このベンチが WiFi も USB host
  も上げないからである。コントローラでは、ほぼ確実に失敗して PSRAM に
  フォールバックする — ログの 1 行を除いては、黙って。実アプリで FT8 が
  この改善を得るには、`internal_pool` が既に `cs` に対して行っている
  ように、オンデマンドで確保するのではなく、起動時に `worker_arena`
  経由でバッファを予約する必要がある。

## 29. coherent スコアラの 78 % は高速パスに乗っていなかった (2026-08-30)

§27 は、マイクロベンチマークを根拠に `ft4::ddc` の FIR を直した。
*実際の* `dot_f32` 呼び出し箇所を数えると — `dotprod-extern` バックエンド
に新設したカウンタを、どの前提条件が破れたかで分けて — その修正が届いた
所と届かなかった所が分かった:

| call site | calls / slot | PIE | slow: alignment | slow: length |
|---|---:|---:|---:|---:|
| front ends (FIR) | 248 904 | **100 %** | 0 | 0 |
| `ft4_sync_search` (±1.0 s) | 284 688 | **22 %** | 219 456 | 0 |

FIR の作業は完全に届いていた。`engine::sync2d` の coherent スコアラ —
`ft4_sync_search` の内側ループ全体であり、`fst4_sync_search` のもの
でもある — は、**純粋にアラインメントが原因で** 78 % がスカラパスに
いた。長さは一度も問題ではなかった: Costas ブロックは
`nsym · ds_spb · 2` 個の f32、FT4 では 256 である。

2 つのオペランドが、どちらも破れていた:

- `FlatRef::plain` / `swapped` は素の `Vec<f32>` で、4 バイトアライン
  しか保証されない;
- `c` は `cd0[s0..]` を `f32` として見たものなので、そのバイトオフセット
  は `s0 · 8` — 16 バイトアラインになるのは `s0` が偶数のときだけで、
  `s0` はグリッドを掃引する。

**そしてここでは、§27 で負けたコピーが勝つ。** FIR のステージング
コピーは dot ごとだった; こちらは候補ごとで、約 25 000 回の dot に
償却される。結果として、それすら不要だった: `FlatRef` のバッファを
アラインし、**先頭がゼロの奇数位相のペア** — `s0 - 1`（これはアラインされ
ている）から読み、最初の複素サンプルがゼロの参照と対にする — を加えると、
何もコピーせずに両方のオペランドがアラインされる。FIR の 4 位相ではなく
2 位相なのは、ここでの単位が複素サンプルだからである。

```
pass2 ft4_sync_search dot_f32: 284 688 calls | 284 688 PIE (100 %)
search ±1.0 s  1 593 -> 1 048 ms
search ±0.5 s    960 ->   700 ms
ship slot      3 516 -> 3 288 ms, 1.79x -> 1.67x
```

`i0_sum` はどの実行でも変わらないので、探索は同じ位置を選んでいる;
どの arm でも 11 件をデコード。

残しておく価値のある失敗が 1 つ: 最初の flash では、スカラパスにまだ
5 184 回の呼び出しがあった。奇数位相の参照が `n·2 + 2` = 258 — 4 の
倍数でない — に切り戻されていたからである。`AlignedF32` は既にそれを
ゼロで 260 にパディングしていた; パディング後の長さを使うのが、正しい
うえに前提条件が求めるものでもある。境界はいまでは `n + 1` ではなく
そのパディング後の長さから表現している。どれだけパディングされるかは
`n` の偶奇に依存し、それは FT4 と FST4 で異なるからである。

host は影響を受けない — 奇数位相は `dotprod-extern` の後ろにあり、
バックエンドの無いビルドは従来どおり単一の参照を保つ。Tier A+B は
green (82 バイナリ)。

**FST4 はこれを無償で得る。** `score_flat_coherent` は共有されており、
`fst4_sync_search` は FST4 広帯域モニタの律速ステージである
(`docs/notes/FST4_BENCHMARK.md`)。ここでは測っていない。

## 30. デュアルコアの価値は 2 倍ではなく 1.33 倍 (2026-08-30)

`dual_core.rs` と `wspr_dual_core.rs` はそれぞれ約 650 行と約 715 行で
ある。「候補ループは恥ずかしいほど並列だ」という理由でそれを FT4 向けに
書くのは、§26.1 と §27 がそれぞれ既に一度代償を払った誤りなので、まず
実現可能性のプローブとした: 12 候補を、バイト単位で同一のコードを走らせ
る 2 つのピン留めタスクに分け、同じ作業に対するシングルコアの参照と
比較する。

```
dual-core probe — 12 candidates | 1 core 2906 ms | 2 cores 2182 ms
                | 1.33x | decodes 11 (serial 11)
```

**1.33×。** 候補ループは構造上は並列だが、実際にはそうではない。理由は
事前に見えていた 2 つと、見えていなかった 1 つである:

- `esp_dsp_fft` の `Fc32Guard` は、すべての変換をまたいで保持される
  プロセスグローバルのスピンロックである。esp-dsp の fc32 ツイドル
  テーブルは単一のグローバルで、長さごとに *リサイズ* されねば
  ならないからだ。DDC と Δt 探索は FFT を使わないが、
  `symbol_spectra` のシンボルごとの 32 点変換は使う — SHIP の候補時間の
  約 19 % — そしてそれらは直列化される。スピンロックはまた、待つ
  コアがサイクルを譲らずに燃やすことを意味する。
- 両コアが `cd0` (1 候補あたり 40 KB) とスロットの音声を PSRAM から
  ストリームするので、§26.3 が既にこのボードの真の制約だと示した
  バスを奪い合う。
- 候補のコストは等しくないので、6/6 の分割では最後にどちらかのコアが
  遊ぶ。

ship slot に投影すると: 候補 2 530 → 約 1 902 ms、slot → 約 2 660 ms、
**1.67× → 1.36×**。実在するが、約 700 行の量産用の仕組みと、internal
DRAM 上のワーカースタックに対するものである — それは 2304 のスクラッチ
(18 KB、§26.3) や FT8 の 3840 のスクラッチ (30 KB、§28) が既に取り合って
いるのと同じ 31 744 B のブロックだ。わずかなコードでアラインメントの
作業が返してきたものと比べると、割の悪い取引である。

### 30.1 代わりに何をするか、同じ証拠に基づいて

SHIP slot の候補作業 2 530 ms は、いま DDC 1 339 (53 %)、Δt 探索 700
(28 %)、LLR+BP 491 (19 %) に分かれる。その中のすべての `dot_f32` は PIE
パスに乗っている (§29) ので、dot は終わった。

DDC に残っているのは算術 **ではない**。ステージ A は 1 候補あたり 70 ms
で、そのうち dot は約 16。残りの約 54 ms は `FirStage::push_one` が
90 000 回呼ばれる分 — 入力サンプルごとにストア 2 つ、カウンタ、
compaction の判定が、出力 4 995 個を生むステージのために。それはスロット
あたり約 648 ms、予算の 20 % を、帳簿付けに費やしている。

（§27 の作り直しによるリグレッションではない: 同じ分割は、それらの変更
の前に 52 ms、後に 54 ms と測れており、その間に dot は 36 → 16 になった。）

`FirStage::push_block` はブロックモードのエントリポイントと自称して存在
するが、本体は依然として `push_one` のループで、しかも `CandidateDdc` は
そもそもそこに届かない — `push_i16` は 1 サンプルずつ混合して push
するので、ステージ A はブロックを一度も見ない。チャンク単位で混合し、
`push_block` に本物の本体（ブロックを履歴に追記し、それから出力をその
ストライドで計算する）を与えるのは、純粋なデータ移動で、構成上ビット
一致し、2 ファイルに収まる。

**これが次の一手として良い**: デュアルコアと同等かそれ以上で、コードは
わずか、しかも FT8 と将来のデュアルコアのワーカーが両方とも必要とする
internal DRAM を消費しない。

## 31. ブロックモード FIR: 予測の 648 ではなく 172 ms (2026-08-30)

§30.1 は、`FirStage::push_one` が 1 候補あたり 90 000 回呼ばれて出力
4 995 個を生むことを指摘し、ステージ A の 70 ms のうち約 54 ms をそこに
置いた — スロットあたり約 648 ms。`push_block` は `wspr::ddc` が必要と
して以来ブロックのエントリポイントと自称していたが、本体は依然として
`push_one` のループで、`CandidateDdc::push_i16` はこれを一度も呼んで
いなかった。

両方を直した: `push_block` は各ランを `copy_from_slice` で一括追記し、
窓がそのラン内で閉じる出力を出すようになり、`push_i16` は 1 024
サンプルのチャンクで混合するので、ステージが実際にブロックを見る。

```
DDC        1 339 -> 1 167 ms  (-13 %)
stage A       70 ->    61 ms
ship slot  3 288 -> 3 119 ms  1.67x -> 1.59x
```

**予測の約 648 に対して 172 ms。** 見積りは差の帰属を誤っていた: ステージ
A の dot 以外の 54 ms は、すべてが帳簿付けだったわけではない。
`ddc_stage_probe` はステージに 90 000 サンプルの `Vec<f32>` を 2 本 —
PSRAM の 720 KB — 与えており、その 54 ms のかなりの部分はそれを読む
ことだった。それはどれだけバッチ化しても消えない。この日 4 回目の
外れで、根本原因は §26.1 と同じ: 測定された残差を、たまたま目に入った
メカニズムに帰属させたこと。

それでも残した — 構成上ビット一致で、DDC 自身の 199/18 と 263/1 を含む
7 通りの `(ntaps, decim, margin, chunk)` の形にわたって固定されており、
172 ms はスロットの 5 % で、internal DRAM を消費しない局所的な変更に
対するものだからである。

### 31.1 デュアルコアの価値は、いまや増えるのではなく減った

同じ実行で再測定: **1.33x -> 1.17x**。Amdahl の法則そのもの — 並列な部分
(FIR) が安くなり、直列化される部分 (`Fc32Guard` の後ろの
`symbol_spectra` の FFT) は安くならなかった。並列側に着地する最適化は
どれも、§30 の約 700 行の根拠を悪くする。デュアルコアは終わったものと
して扱う。

### 31.2 予算は実際どこにあるか

ship slot 3 119 ms に対し 1 960: coarse 758、DDC 1 167、Δt 探索 700、
LLR+BP 約 494。

しかしこれは **14 信号** のシーンで、§23 は現実的な FT4 の占有度を 5-10
とした。1 候補あたり 197 ms として:

| occupancy | candidates | slot | vs budget |
|---|---:|---:|---:|
| 5 signals | 5.3 | ~1 801 ms | **0.92x** |
| 10 signals | 9.2 | ~2 570 ms | 1.31x |
| 14 (measured) | 12.3 | 3 119 ms | 1.59x |

**FT4 はいまや自身の設計範囲の下端に収まる**。今朝の時点では、どの時点
でもそうではなかった。残る差は、その範囲の上端と、FT8 密度という最悪
条件である。

捕捉中に粗同期ステージをストリーミングする (§25 のリスト、項目 2) と、
スロット後の予算から 758 ms が取れ、**現実的な範囲全体が予算内になる**:
10 信号は約 1 812 ms、0.92x になる。`symbol_spectra_avg` は、スロット
にわたる 152 個の独立した窓掛け変換であり、それぞれ、サンプルが届き次第
計算できる。`wspr_app` は既にこの形を出荷している
(`ddc_loop` -> `DDC_READY_IDX` -> `scan_loop`)。これはまた、グローバル
ガードの後ろの純粋な FFT なので、並列化 *できない* ステージでもある
— したがって予算から外すことが、そのステージにとって唯一のレバーである。

## 32. coarse ステージのストリーミング化と、運だった 100 % (2026-08-30)

`symbol_spectra_avg` はスロット全体にわたる約 152 回の窓掛け変換であり、
各変換は既に到着済みのサンプルだけに依存する。[`Ft4SavgBuilder`] は
音声ブロックから `savg` を累積するので、受信機は収録中に各行を完成させ、
その後に支払うのは `ft4_coarse_sync_from_savg` だけになる。ブロックサイズ
によらずビット一致であり、`NSTEP` を割り切らないものを含む 9 通りの
チャンクサイズでホスト上に固定してある。

```
pass0s streamed coarse — build 798 ms (overlaps capture, 7 500 ms of it)
                       + pick    6 ms (post-slot)
                       | 12 candidates, identical to whole-slot pass0
post-slot coarse cost 761 ms -> 6 ms  (754 ms leaves the budget)
```

**総長を事前に知っている必要がある**。これは利便性の問題ではない。
`getcandidates4.f90` が平均するのはちょうど `(nz − NFFT1)/NSTEP` 行で、
これは「収まる行すべて」では*ない*。90 000 サンプルのスロットではそれが
152 であり、152 行目もなお完全に音声の内側に収まる。貪欲に行を出力する
builder だと 153 行を平均してしまい、別の `savg` を生む。

### 32.1 2 つの誤り、どちらも bench が捕まえた

**書き直しの結果、非ストリーミング経路が 2.5× 悪化した。** 最初の版は
push のたびにバッファし、行ごとに `drain` で詰め直していた。呼び出し側が
スロット全体を一度に渡す場合 — まさに `symbol_spectra_avg` がそうする —
これは二次的なコストになる。152 行それぞれが、90 000 サンプルの残りを
memmove する。`ft4_coarse_sync` は 758 → 1 906 ms になった。現在は、
何も保持していない限り呼び出し側のブロックから直接行を読み、2 つの
ブロックにまたがる行だけが履歴に触れる。

**しかも §29 の結果は運だったことが判明した。** 同じ実行で、スコアラの
dots は PIE 経路の 100 % から **0 %** に落ち、search は 1 049 →
1 789 ms になった — `sync2d` には一切変更がないのにである。odd-phase
参照が扱うのは `s0` の*パリティ*だが、それは `cd0` の先頭が 16 バイト
アラインされている場合に限る。先頭が 8-mod-16 だと、どの `s0` でも
うまくいかない。`cd0` は `Vec<Complex<f32>>` でアラインメントは 4 で
あり、それを保証するものは何もなかった。§29 はたまたまアラインされた
40 KB のブロックを得ただけで、バイナリ内の別の場所でもう 1 つ 18 KB の
内部バッファをリークさせるとヒープがずれて、それを失った。

`AlignedCd0` は現在これを明示的にアラインし、必要なときは呼び出しごとに
1 回コピーする。そのコストは候補あたり ~7 ms (search 1 049 → 1 129 ms、
±1.0 s)で、2.3× の改善とコイントスとを分ける差である。**制御されて
いないアロケータに依存する、実測された 100 % は保証ではない** — しかも
これは既に結果として書き上げられていた。

### 32.2 coarse ステージを外した後の予算

| | whole-slot coarse | streamed coarse |
|---|---:|---:|
| post-slot coarse | 761 ms | 6 ms |
| candidates (12) | 2 431 ms | 2 431 ms |
| **slot** | **3 192 (1.63×)** | **2 437 (1.24×)** |

候補あたり 203 ms として、§23 の占有度の数値と突き合わせると:

| occupancy | candidates | slot | vs budget |
|---|---:|---:|---:|
| 5 signals | 5.3 | ~1 082 ms | **0.55×** |
| 10 signals | 9.2 | ~1 874 ms | **0.96×** |
| 14 (FT8 density) | 12.3 | ~2 503 ms | 1.28× |

**FT4 の現実的な占有度の範囲がすべて収まるようになった**。FT8 密度の
pessimum が 1.28× 超過である。今朝の同じ測定は設計点で 2.33× であり、
どこでも超過していた。

残っているのはデコード速度の問題ではない。`Ft4SavgBuilder` はライブラリ
部品で、まだどの組込みバイナリからも呼ばれていない。これを UAC の収録
経路に配線すること — `wspr_app` が既に `ddc_loop` → `DDC_READY_IDX` →
`scan_loop` と走らせているのと同じ方式 — が、754 ms を実際の受信機の
余裕に変える。

## 33. ストリーミング coarse ステージは収録に追いつくか? (2026-08-30)

§32 は 10 % の duty cycle を根拠に、754 ms を post-slot の予算から外した。
だが平均値は、それが機能するかどうかにはほぼ無関係である。
`uac::reader_thread` は、リサンプル済み音声 1 回の read 分 —
`dst_scratch` は 48 k → 12 k で ~256 サンプル分の大きさ — を
`AudioSink::push_samples` に渡す。**reader スレッドから、sink の mutex を
保持したまま**であるため、遅いブロックは次の `uac_host_device_read` を
遅らせ、その後ろにキューされることはない。しかも仕事は均等には
分散しない。行は `NSTEP = 576` サンプルごとに到着するので、数ブロックに
1 つが 2 304 点の変換を丸ごと支払い、その隣のブロックは数十マイクロ秒しか
払わない。

| block | audio/block | blocks | with a transform | min | p50 | **max** | max / budget | duty |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 128 | 10.7 ms | 704 | 152 | 4 µs | 18 µs | 5 485 µs | **51 %** | 10 % |
| **256** | **21.3 ms** | 352 | 152 | 19 µs | 38 µs | **5 508 µs** | **25 %** | 10 % |
| 512 | 42.7 ms | 176 | 152 | 39 µs | 5 393 µs | 5 659 µs | **13 %** | 10 % |

**UAC の実際のブロックサイズでは、最悪のブロックが自身のリアルタイム
予算の 25 %** であり、128 サンプルでも 51 % である。builder は reader
スレッドから直接駆動できる。キューの後ろに専用のタスクを置く必要はない。

どのブロックサイズでも 152 行、そして `savg[100]` は 3 つすべてで最後の
桁まで一致した — ホストテストのブロック独立性はデバイス上でも成り立つ。

これが確立**しない**ことが 2 つあり、どちらも実機の無線機が手元に
ないと閉じられない:

- 測ったのは、それ以外に何も載っていないコア上の builder 単体である。
  アプリではそのスレッドがリサンプルも行い、そのコアは WiFi と USB
  ホストも抱えている。
- reader ループでの 5.5 ms の追加レイテンシが USB ホストドライバに
  パケットを 1 つ落とさせるかどうかは、そのバッファリングの性質であって、
  この測定の性質ではない。マージンは「落とさない」と言っているが、
  それは知っていることと同じではない。

(この測定の初回実行では、n = 512 で 152 行ではなく 129 行と報告された —
「このブロックは変換を行ったか」の判定がブロック自身の予算の割合に
なっており、512 では行ブロックがその両側に落ちる。固定閾値なら 5 400 µs
と 40 µs を曖昧さなく分離できる。修正済みであり、黙って数え間違える
診断は無いよりも悪いので記しておく。)

## 34. 候補ループが実際に守るスロット予算 (2026-08-30)

§33 の受信機は、coarse ステージが出したすべての候補を実行して 2 412 ms
かかった。モニタとしては問題ないが、トランシーバとしては問題である。
1 960 ms の予算とは、フレームの終了からスロットの終了までの隙間であり、
キーダウンしなければならない無線機に 2 412 ms はない。

`decode_slot` は `budget_ms` を受け取り、それを過ぎたら候補の開始を
やめるようになった。`fst4_monitor::run_candidate_loop` が使うのと同じ
形で、次の候補が収まるかを予測するのではなく、各候補の前に確認する。
**締切は entry からではなく `CapturedSlot::closed_us` から測る**:
前のスロットの仕上げにまだかかっていたために開始が遅れたデコーダは、
その分だけ持ち時間が減る。これが受信機を、徐々に遅れていくのではなく
最新の状態に保つ会計である。

**カットは最も弱いものを取る。** `ft4_coarse_sync` は候補を coarse
スコアの降順で返すので、切り詰めれば末尾が落ちる。

実測、`TX_TURNAROUND_BUDGET_MS` での `ft4-demo`:

```
slot 1 — 11 of 12 candidates tried, 10 decodes in 2146 ms of 1960 ms — cut 1 weakest at score 1.55
slot 2 — 10 of 12 candidates tried,  9 decodes in 1963 ms of 1960 ms — cut 2 weakest at score 2.63
```

このスロットで諦めるのは、11 個のうち最も弱い −17 dB の
`W7BOB KJ7G RR73` であり、より厳しいスロットでは −13 dB の
`NZ7P WA7JAY` も諦める。この入れ替わりは締切が役目を果たしている
ということである。デモのペース配分ではスロットのクローズが 7 490 または
7 510 ms 間隔で着地し、デコーダには残りがそのまま与えられる。

### 34.1 オーバーシュートは候補 1 つ分で、構造的なものである

1 960 ms の締切に対して 2 146 ms は 186 ms の超過であり、締切を過ぎた
時点で既に実行中だった候補である。開始前に確認する方式では、これ以上は
良くできない。しかも FT4 の候補 1 つは予算全体の ~10 % で、52 s の締切に
対する FST4 モニタの候補の ~1 % とは桁が違う。そこでうまく機能する形が、
ここではゆるい。

それでも許容範囲ではある。次のスロットの送信は
`TX_START_OFFSET_S = 0.5` で始まるので、186 ms の超過なら 314 ms が
残る。候補ごとの実行平均から予測してこれを閉じるかどうかは本物の問題
だが、意図的にまだ答えを出していない — 予測が外れると候補を早く止めて
しまい、デコードを丸ごと失うのに対し、オーバーシュートが失うのは
マージンだけである。選ぶ前に測ること。

### 34.2 これは設計点ではなく pessimum である

golden は 14 信号の FT8 密度シーン (§23.5) であり、だからこそ切るものが
ある。FT4 自身の 5-10 信号の占有度では、coarse ステージが出す候補は
5-9 個で、ループは何も落とさずに予算内で終わる — §31.2 の 0.55× と
0.96× である。`RX_ONLY_BUDGET_MS` (7 500 ms) は送信しない受信機のための
もので、超過が失うのは次のスロットだけであり、そもそも何も切る必要が
ない。

## 35. FT8 のステージ分割と、ウォーターフォールが払っていたもの (2026-08-30)

1 つの疑問から出た 2 つの測定: i16 音声を f32 に変換するのは esp-dsp の
f32 FFT を使うためか、そしてそれは表示のための対価として妥当か?

### 35.1 FT8 は変換しないし、ステージ 1 は表示のコストではない

`fixed-point` の `compute_spectrogram` は `audio[k].to_i16() as i32` を
読み、sc16 カーネルまで整数のままである — i16 経路はまさに変換を起こさ
ないために存在する。そして `stage1_inc` は coarse 探索と並べて `wf_q` を
供給するので、スペクトログラムは両方に役立つ。ウォーターフォールは
その上の相乗りであり、存在理由ではない。

FT8 の初のステージ別内訳、出荷先のボード上 (CoreS3、`qso3_busy.wav`、
出荷設定、7 デコード):

| stage | ms | share |
|---|---:|---:|
| **1 — spectrogram** | **1 403** | **51 %** |
| 2 — coarse sync | 236 | 9 % |
| pass 2 — re-rank | 166 | 6 % |
| 3 — refine + LLR/BP | 915 | 34 % |
| **total** | **2 726** | |

このツリーにあったこれまでの FT8 のステージ別の数値は、すべて StickS3 か
アプリ自身のログ由来だった。

### 35.2 FT4 のウォーターフォールは、それが乗るステージの 10 % を食っていた

FT4 の `wf_row` は別の話であり、疑問はまさにそこに落ちた。*行*は無料
である — `push_with_rows` は coarse ステージが既に計算しているスペクトルを
渡すだけだ — が、その上のマッピングは一度も測定されておらず、「行は無料」
は同じ主張ではない。

| version | per row | per slot | of the coarse stage |
|---|---:|---:|---:|
| `10 * log10` per column | 623 µs | 94 ms | **10 %** |
| integer log2 (`leading_zeros`) | 384 µs | 58 ms | 6 % |
| …and the bin mapping folded into two constants | **174 µs** | **26 ms** | **3 %** |

`f32::log10` はここで ~620 サイクルであり、1 行に 240 回あった。これを
指数部 — MSB のビット位置に、半オクターブ分解能のための小数 1 ビットを
足したもの、FT8 の `decimate_pair_to_wf` が昔からやっていること — に
置き換えて 3 分の 1 が取れた。

残りはもっと悪く、言い訳がきかない。**どのビンを覆うかを求めるための、
列ごとの浮動小数点除算が 4 回**、1 行で 960 回である。定数だけに依存して
決して変わらないマッピングのために。開始値とステップに畳み込んだので、
ループは加算 1 回と切り捨て 2 回になる。

パレットは 30 dB に対して 16 段階の粗いステップなので、レベルを半オク
ターブ (~1.5 dB) に量子化しても表現できる範囲より下であり、絵は
変わらない。

**教訓は、マイクロ秒ではなく枠組みにある。**「変換は無料」は事実で
あったが、「ウォーターフォールは無料」の代わりに通用してしまい、後者は
測られたことがなく、事実でもなかった。スロットあたり 94 ms は致命的では
ないが、それが乗っているステージの 10 分の 1 であり、絵のために使われて
いる。

## 36. `fixed-point` は実際に FT8 に何をもたらすのか (2026-08-30)

「FT8 は FT4 の単純な経路を取れたのでは?」という問いから出た 3 つの
測定。このボード上で FT4 は端から端まで f32 で動く。FT8 の組込みビルドは
スペクトログラムを u16 に、LLR/BP ラダーを Q11i16 に量子化しており、
記録されている理由は PSRAM 帯域幅の半減である。

### 36.1 Recall: コストはゼロ

`fixed-point` は `nstep-half` を含意するので、これまで両者を分離した
測定はなかった。`nstep-half` は単独でも使えるので、分離できる:

| build | decodes |
|---|---:|
| `full` (f32, NSTEP = NSPS/4) | 14 |
| `full,nstep-half` (f32, NSTEP = NSPS/2) | 12 |
| `full,fixed-point` | 12 |

後ろの 2 つは**同じ 12 メッセージ**をデコードする。14 → 12 の全体が
coarse の時間グリッドによるものであり、量子化はここではコストを生まない。
`ft8_qso3_decode_set` と、`ft8_qso3_apoff_recall` の修正済みフロアを参照。

### 36.2 スペクトログラム: メモリ半分で 1.11×

両方の builder を 1 つのバイナリにインスタンス化 (`compute_spectrogram_f32_timed`)、
同じ音声、同じアロケータ状態:

```
stage 1 (spec, u16)   1 403 ms   351 KB
stage 1 f32 (A/B)     1 563 ms   702 KB   → fixed-point is 1.11x
```

メモリは設計どおり半分になる。**時間はそうならない**: 11 % であり、
帯域幅律速のステージにデータを半分渡せば時間は半分に近づくはずである。
つまりステージ 1 は帯域幅律速ではなく、184 回の変換と、その周りの
窓掛け/絶対値/格納が支配している。

セル型だけの純粋な対比ではない: f32 経路は矩形窓で、fixed-point 経路は
Hann であり、後者はシフトを選ぶためにスロットもスキャンする。それが
各経路のコストである。

### 36.3 BP: fixed-point は**遅い**

`LlrScalar` はトレイトで、`BpScratch`/`bp_decode_nms_with_scratch` は
それに対してジェネリックなので、スペクトログラムと違って両方を単純に
インスタンス化できる。同じ LLR、同じ反復回数、同じバイナリ:

```
BP Q11i16   22 813 us/run
BP f32      19 455 us/run   → Q11i16 is 0.85x f32
```

**18 % 遅い。** LX7 には f32 FPU がある。チェックノードの演算を i16 で
行っても何も節約できず、飽和演算ヘルパのコストを払うことになる。これは
`#198` が FST4 と FT4 について記録したのと同じ結果 — LX7 では
`fixed-point` が f32 より*遅い*と測定された — であり、今回は FT8 自身の
BP について示された。これが、FT8 でそれが再検討されてこなかった理由である。

### 36.4 だから答えは、おおむねイエス

| claim | measured |
|---|---|
| costs recall | **no** — identical decode set |
| avoids an i16 → f32 conversion | no conversion exists either way |
| halves the spectrogram | **yes**, 702 → 351 KB |
| speeds up the spectrogram | 1.11× |
| speeds up BP | **0.85× — it is slower** |

`fixed-point` が守っているのは速度ではなく**メモリ**である: 8 MB を持つ
ボード上で、PSRAM の 702 KB に対して 351 KB。明確に正しい唯一の場所は
ストリーミングの `stage1_inc` で、その設計全体が増分 u16 スペクトログラム
だが、その経路は §36.2 が測ったものでは**ない**ので、ここではそこが
間違っているとは言えない。

ここには、それを引き剥がすという提案はない。単純な経路が、想定より
FT8 の多くの部分で使えたこと、とりわけ LLR/BP の半分が 18 % のコストを
払い、測定可能な利益を何も買っていない量子化を抱えていること、を述べて
いる。

## 37. 予算は余裕ではなくデコードである (2026-08-31)

§31.2 と §32.2 は、FT4 は現実的な占有度で予算に収まると結論づけ、それが
「ならば予算をさらに空けても得るものは少ない」の代わりに通用してしまった。
これらは別の主張であり、後者は誤りである。

§34 のカットオフは**今まさに発火している**:

```
slot — 11 of 12 candidates tried, 10 decodes — cut 1 at score 1.55
slot — 10 of 12 candidates tried,  9 decodes — cut 2 at score 2.63
```

締切なしなら 11 個デコードでき、あるなら 9 または 10 個である。そして
§23 の占有度の表は、それがこの録音の人工物ではないことを示している —
**デコードに成功する最深のランクは候補数に追随する**:

| signals/slot | mean candidates | deepest decoding rank |
|---:|---:|---:|
| 5 | 5.3 | 6 |
| 10 | 9.2 | **10** |
| 14 | 12.3 | **15** |
| 20 | 14.8 | **19** |

候補リストの末尾は死荷重ではない。測定されたどの占有度でも、それを
切り詰めるとデコードを失う。

つまりミリ秒は局に換算される。候補あたり ~197 ms なら、1 960 ms の予算は
9.9 個の候補に届く。197 ms 節約するごとに 1 個増え、その節約が効くのは
**混雑したバンドで最も大きく**、それはまさにデコードが欲しいときである。
これは §36 の「やる価値のあること」の順位を逆転させる。

| lever | saving | candidates |
|---|---:|---:|
| wire the streamed coarse into the receiver (built, unwired) | **754 ms** | **+3.9** |
| shared decimation (§37.1) | 278 ms | +1.4 |
| LLR/BP — 479 ms, never broken down | ? | ? |

### 37.1 共有 decimation の測定

`ft4::ddc` は候補間で何も共有しない: 候補ごとに 90 000 サンプルの
スロット全体をミキシングしてフィルタする。一方 FT8 のスペクトログラムと
`downsample_cached` の広帯域変換は、それぞれ一度だけ計算されてすべての
候補が読む。ステージ A だけで、候補の 96 のうち 61 ms である。

実数音声に対する共有 decimate-by-2 を 1 つ置き、その後は候補ごとの
ミキシングと、半分のサンプル・半分のタップでの decimate-by-9:

```
A now (12k, per candidate)   60 986 us  (199 taps, /18, 90 000 in -> 4 995 out)
A after /2 (6k, per cand)    30 828 us  (101 taps,  /9, 45 000 in -> 4 995 out)
shared /2 (once, complex)   167 130 us  (111 taps,  /2, 90 000 in -> 44 973 out)

today    731 ms (12 x 60)
proposed 453 ms (shared 83 + 12 x 30)
```

候補ごとの半分は予測どおりに出たが、これは §26.1、§27、§31 の後では、
当然のこととせずに記しておく価値がある。共有側は期待より高くつく — 83 ms は
候補 1.4 個分 — ので損益分岐は約 **2.7 候補**であり、§23 の 5-10 信号の
占有度では常に得、閑散としたバンドではおおむね中立である。

まだ未実装であり、差し替えるだけでは済まない。共有ステージには実数入力
の `FirStage` が要る(現行のものは I と Q の両方をフィルタするので、仕事の
半分が無駄になり、上の 83 ms はその半分が取り除かれることを前提にして
いる)。通過帯域は 2 ステージではなく 3 ステージになり、§20 の「通過帯域は
フィルタ設計の自由な選択ではなく、デコードのパラメータである」という
点が当てはまる — +0.021 dB の等価雑音帯域幅の一致を再確立しなければ
ならず、`ft4_ddc_equivalence` の 560 ファイル 0.0 dB の結果を再実行
しなければならない。3 ステージ目は丸めを変えるので、何もビット一致には
ならない。

## 38. LLR/BP の末尾の内側 — それは BP である (2026-08-31)

出荷スロットの 3 つ目の構成要素は、これまで引き算でしかなかった:
`pass3 − pass1d − pass2` ≈ 479 ms。`process_candidate_precomputed` が
呼ぶ公開部品で分解した。最も強い候補について:

| stage | µs / candidate | × 12 |
|---|---:|---:|
| `symbol_spectra` (per-symbol DFT) | 1 696 | 20 ms |
| `compute_llr` | 1 370 | 16 ms |
| **the rest — BP, its ladder, SNR** | **21 060** | **252 ms** |
| whole tail | 24 126 | 290 ms |

**87 % が BP である。** DFT と LLR 計算を合わせても 13 %。

290 ms は、それが置き換える引き算より 189 ms *低い*が、その差自体が
情報になる: これは rank 0、最も強い候補であり、20 回の繰り返しで測った。
弱い候補ほど BP の反復が増えるので、252 ms は下限であってスロットの
実際の値はそれより上であり — それは、コストを生んでいるのが反復回数だ
ということの別の言い方である。

### 38.1 ここには取れるものがない

このステージのどのレバーも、使い切られているか、閉じていると測定
されている:

- **量子化** — §36.3 は FT8 の BP が Q11i16 で 0.85× であること、すなわち
  このコアでは f32 より*遅い*ことを測定した。FT4 は既に f32 である。
  得るものは何もなく、FT4 が追随しなかったのが正しかった理由である。
- **`llrd`** — §22 は、どちらの regime でも recall への寄与がゼロで、
  ラダーの 20-22 % を消費していることを見出した。既に削除済み
  (`b27bea0`)。
- **`llrc` (nsym = 4)** — §22 は、これがなければ −46 デコードと測定した。
  使えない。
- **反復回数の上限** — 直接に recall のノブである。

したがって 252 ms は残る。§37.1 の共有 decimation は同程度の金額で
278 ms であり、しかも**recall 中立**である — 変えるのはフィルタ連鎖で
あって探索でも判定でもない — ので、こちらが先になる。同じミリ秒で、
片方は再検証すべき副作用がない。

### 37.2 実数入力経路の測定 (2026-08-31)

§37.1 の 83 ms は `shared / 2` — 複素入力の測定値を、使わない Q チャネル
を落とせばコストがちょうど半分になるという仮定で半分にしたものだった。
仮定する代わりに、`FirStage` に実数入力のエントリポイント
(`push_block_real`、出力あたり dot が 2 つでなく 1 つ、
`push_block_real_matches_push_block` によりこのステージ自身の 165 タップ /
decim 2 を含む 5 つの形状で `push_block` の I チャネルとビット一致に固定)
を追加し、プローブをそれに直接向けた:

```
A now (12k, per candidate)          61 011 us  (199 taps, /18, 90 000 in -> 4 995 out)
A after /2 (6k, per cand)           30 843 us  (101 taps,  /9, 45 000 in -> 4 995 out)
shared /2 (once, REAL)             108 699 us  (165 taps,  /2, 90 000 in -> 44 959 out)

today    732 ms (12 x 61)
proposed 478 ms (shared 108 + 12 x 30)
```

83 ms ではなく 108.7 ms — 複素のコストを半分にしたのは ~30 % 過小評価
だった(実数の dot は 2 つの半分ではなく 1 つの積和ループであり、
`push_block_real` の履歴管理における、サンプルあたりの固定オーバーヘッド
は、片方のチャネルが半分になったからといって半分にならない)。節約は
§37.1 の見積りより小さいが、それでも本物である: §37 の ~197 ms/候補の
レートで **254 ms、~1.3 候補**、対する §37.1 は 278 ms/1.4。§23 が測定した
どの占有度でも依然として得であり、ストリーミング coarse を受信機に配線
することに次ぐ 2 番手であることも変わらない。

これまでの新規コードは `push_block_real` とその等価性テストだけであり —
`ft4::ddc` の `CandidateDdc` 自体は変更なし — これは測定であって、まだ
出荷された最適化ではない。

## 40. FT8 の LLR/BP を f32 で — 見込みは 6 %、スロットは 0.7 % しか動かなかった (2026-09-01)

§36 は、このボード上で FT8 の BP が **Q11i16 で 0.85×** であることを測定し、
issue #349 の最初の提案ステップは「FT8 スロットの 34 % を占めるステージの
18 %」と読めた。それは見込みであり、これがその価値である。

`fixed-point` は i16 の LLR/BP ホットループを含意しなくなった。スカラは
今や独自の feature `fixed-point-llr` で、デフォルトは off である — した
がってビルドは、整数 BP なしで u16 スペクトログラム(702 → 351 KB の出所は
ここ)を得られ、より狭いスクラッチを望むターゲットはそれを要求する。
動くのは 2 つの型エイリアスで、パイプラインの他は何も変わらない。

CoreS3 上の A/B、`ft8-bench`、`qso3_busy.wav`、3 通りの re-rank 幅:

| pass-2 width | stage 3, Q11i16 | stage 3, f32 | Δ | slot total |
|---|---:|---:|---:|---:|
| top 15 | 909 904 µs | 874 858 µs | **−3.9 %** | 4.333 → 4.298 s |
| top 20 | 932 529 µs | 897 881 µs | **−3.7 %** | 4.355 → 4.325 s |
| top 30 | 1 135 751 µs | 1 099 570 µs | **−3.2 %** | 4.560 → 4.530 s |

両ビルドのどの arm でも 7 デコードで、ホストでは golden 集合が同一 —
12 メッセージで、`ft8_qso3_decode_set` を 2 つの feature 間で diff した。

**ステージ 3 が動くのは 18 % ではなく 3.5 % で、スロットは 0.7 % である。**
18 % は本物だが、ステージ全体ではない。A/B ハーネスの `bp_decode_nms` は
収束しない LLR に対して `max_iter = 30` まで走る(bench は `0 converged`
と出力する)が、パイプラインでは大半の BP 呼び出しがそれよりずっと前に
CRC ヒットで抜け、ステージ 3 は refine パス、DFT、LLR 計算、OSD も抱えて
いる。カーネルごとの比率を、そのカーネルを部分的にしか実行しないステージ
に適用すると、5× 過大評価になる。

したがってこの変更は取る価値がある — スロットあたり 35 ms、recall
コストなし、しかも一度も測定されなかった結合を取り除く — が、「34 % の
18 %」がこれで買えたものではなく、#349 はこの訂正を添えて読むべきで
ある。

i16 スクラッチが節約する 12 KB はヒープである(`BpScratch` のバッファは
`Vec`)ので、スタック上では何も動かない。`m5stack-core2-app` は
`fixed-point-llr` を on に固定したままにする: そのボードは LX6 であり、
上の A/B は LX7 であり、その履歴は整数ループで測られている。

### 40.1 #349 がなお求めているもの、および求めていて誤っている 1 点

ここで手つかずのもの: ステップ 2 (`stage1_inc`、ストリーミング u16
スペクトログラムを同じ方法で測定する — §36 が計時したのは*バッチ*
builder であり、そう明記した)。

ステップ 3 の「`fixed-point` は `nstep-half` を含意するのをやめるべき」は
実施されておらず、すべきでもない。`nstep-half` は意図的に結合されている
(Phase 1.7.7b で、`mfsk-core/Cargo.toml` が理由を載せている): ホストの
`fixed-point` ビルドは組込みパイプラインをシミュレートするために存在し、
NSTEP が違えば別のものをシミュレートする — 独立させたときは `qso3_busy`
の単一パスで 4 対 7 デコードだった。結合されているのは*シミュレーション
とそれがシミュレートする対象*の間であって、数値フォーマットと探索
パラメータの間ではない。誤りだったのは BP スカラをスペクトログラムの
それに結びつけていたことで、それがこの変更の変えるものである。

## 41. ストリーミングスペクトログラムの測定 — u16 は f32 の 0.63× (2026-09-01)

§36 は**バッチ**の `compute_spectrogram` を計時し、それが `stage1_inc`
について何も決着させないと、はっきり述べた: その builder の設計全体は
収録中に供給される増分 u16 スペクトログラムであり、帯域幅の議論はそこで
はまだ成り立つかもしれない。Issue #349 のステップ 2 はそれを測ることで
ある。測定してみると成り立たず、バッチ builder より悪く失敗する。

`stage1_inc::compute_pair_into` の本体は今や `pair_kernel_i16` であり、
`pair_kernel_f32` は同じ計算を別のスカラで行うものである: 2 行の音声を
1 つの複素変換に詰める方法も、2 つのスペクトログラム行への demux も同じ
で、`sc16` の代わりに esp-dsp の `fc32` 変換を使う。`scalar_ab` は 1 スロット
の 92 ペアに対して両方を、それぞれ 3 パスの最良値で走らせる。

```
stage1_inc A/B:  1 342 166 us u16   vs   852 059 us f32   (345 KB vs 690 KB spec)
```

**u16 は f32 の 0.63×** — メモリ半分と引き換えにスロットあたり 490 ms
遅い。§36 のバッチの数値に対してスペクトログラム行あたりで正規化すると:

| builder | u16 | f32 | |
|---|---:|---:|---|
| batch `compute_spectrogram` | 7.86 ms/row | 8.49 ms/row | u16 1.08× |
| **`stage1_inc`** | **7.29 ms/row** | **4.63 ms/row** | **u16 0.63×** |

逆転はペアトリックによる。`stage1_inc` は 2 つの実数行を 1 つの複素変換に
詰めるので、バッチ builder が 184 回払うところを 92 回の変換で済ませる —
そして f32 の arm はその節約の全部を得る (8.49 → 4.63) 一方、u16 の arm は
ほとんど得られない (7.86 → 7.29)。`sc16` の 3840 点混合基数変換はこのコア
で 14.6 ms かかり、`fc32` は 9.3 であって、その差が半減を食いつぶす。

したがって `fixed-point` はストリーミング builder でもメモリを守って
おり、そこではそれに対してスロットあたり ~490 ms を課す。§40 (BP) と §36
(バッチスペクトログラム) を合わせると、その主張された 3 つの速度上の利点
はすべて測定済みで、どれも利点ではない。

### 41.1 これが行わないこと

**ここで `stage1_inc` を f32 に切り替えることはしない。** スペクトログラム
のセル型はこの builder に局所的なものではない: `coarse_sync`、`pass2`、
ウォーターフォール行 builder のすべてが `&[u16]` を取り、スロットバッファ
は 345 → 690 KB になる。それは独自の recall の問題と独自のメモリ予算を
伴う変更であり、タイミング測定の付随作業ではない。このセクションが確立
するのは、u16 を保つ理由が 345 KB であるということで、それは 2 つの音声
バッファと PSRAM キャッシュも抱えるボード上の実在する制約であり — 速度
ではない。速度というのが主張されていたことである。

### 41.2 指標は正しくなるまでに 2 度間違えた

タイミング A/B は、片方の arm が同じものを計算していなければ無価値なので、
`scalar_ab` は 2 つのスペクトルを相互チェックする。最初の 2 回の試みは、
変換ではなくチェッカを測っていた:

- **行ごとのピークビン**: 171/184 が一致。次に i16 arm に、ワーカーが実際に
  ロックするシフトを与えると **144/184** に落ちた — より忠実な arm から
  *悪い*スコア。20 局のいるバンドでは argmax が 2 つの近いピークの間で
  入れ替わり、雑音の行ではコイントスである。
- **ピークが平均の 8× 上にある行のピークビン**: 136/174。同じ問題で、
  サンプルがより小さい。
- **バンド上の行ごとの Pearson r**: 平均 **0.9882**、最悪 **0.9137**、
  180 行(u16 arm が平坦に量子化する 4 行は、0 とスコアせずに数えて
  スキップした)。これが、チェックが本来問うものである: 量子化を除いて
  同じスペクトルか。

§33 の行数カウント診断と同じ教訓 — 黙って自分自身を測るチェックは、
無いよりも悪い。

## 42. 共有 decimation、実装してボード上で (2026-09-01)

§37.1 と §37.2 は、ある提案の 3 つの脚 — スロットの実数音声に対する共有
decimate-by-2 を 1 つ置き、その後は半分のタップで 6 kHz の候補ごとの連鎖
— を測定し、パッチではなくプローブとしてそこに置いたままにした。この
セクションはそのパッチと、ボードがそれをどう扱ったかである。

### 実装したもの

`ft4::ddc` は共有側の半分を得た:

- **`SlotDecimator`** — 165 タップ、÷2、`fc` 2 800 Hz、
  `FirStage::push_block_real` 経由で駆動するので、ステージは、ゼロの履歴
  に対して出力ごとに dot が 2 つでなく 1 つのコストで済む。
  `decimate_slot()` はワンショット形であり、構造体が存在するのは、仕事が
  ブロック単位で、受信機は音声を到着に従って得るからである。
- **`CandidateDdc::new_half_rate`** — Hz で同じ連鎖 (320 Hz のステージ A、
  56 Hz のステージ B、同じバンド中心と derotation) を、ハーフレートの
  ストリームに対して供給する: ÷18 と 199 ではなく、101 タップと ÷9。
- **`candidate_baseband_half`**、および `push_i16` の隣に `push_f32`。

`ft4_rx::decode_slot` は候補ループの前に一度 `decimate_slot` を呼び、
ループの中で `candidate_baseband_half` を呼ぶ。締切の内側で、意図的に:
それはデコードの仕事であり、`elapsed_us` はどちらにせよスロットクローズ
から走る。

### ボード上で

`ft4-demo`、CoreS3、golden スロットのリプレイ、
`TX_TURNAROUND_BUDGET_MS` (1 960 ms)。ログは
`logs/ft4_demo_baseline_2026-09-01.log` と
`logs/ft4_demo_shared_decim_wired_2026-09-01.log`、それぞれ 9 スロット。

| | candidates | decodes | slot | per candidate |
|---|---:|---:|---:|---:|
| baseline | 11 of 12 (cut 1 at 1.55) | **10** | 2 067-2 118 ms | ~188 ms |
| shared | **12 of 12**, nothing cut | **11** | 2 019-2 101 ms | **~168 ms** |

**締切が発火しなくなった。それが要点のすべてである。** §37 は、この受信機
ではミリ秒が余裕ではなくデコードであると論じた。予算が、ループが候補
リストのどこまで届くかを縛るからである。測定はその議論が閉じることで
ある: 候補あたり 20 ms が 12 番目の候補を買い、その 12 番目の候補は −17 dB
の `W7BOB KJ7G RR73` — 11 個のうち最も弱く、§34 がカットにより諦めると
記録したものである。

**今回は予測が正しかった。** §37.2 は、108.7 ms を一度だけ払って候補あたり
30.5 ms の節約、すなわち 12 個で候補あたり ~21 ms と見込み、測定は 20 で
あった。§26.1、§27、§31 で同じ形の算術が 3 度外れた後なので、はっきり
述べておく価値がある: これはコストについての算術ではなく 3 つの脚すべて
の*測定*であり、それが違いである。

スロットは依然として 60-140 ms 超過する。これは §34.1 の構造的な
オーバーシュート — 締切を過ぎたときに既に実行中の候補 — であり、以前
より小さいのは、候補が安くなったからにすぎない。

### 感度に何を払ったか: 測定可能なものは何も

分割により連鎖は 3 ステージになるので、何もビット一致ではなく、§20 の
「通過帯域はデコードのパラメータである」が、それが書かれたときには存在
しなかったフィルタカスケードに当てはまる。仮定せずに再確立した:

- **参照バンドに対する等価雑音帯域幅**: 2 ステージ **+0.021 dB**、共有 +
  2 ステージ **+0.021 dB**
  (`ft4::ddc::tests::noise_bandwidth_matches_the_reference_band`)。
  追加のステージが動かすのは 1 ミリデシベル未満であり、これはバンド端の
  45× 上にある 2 800 Hz のコーナーが買うものである。
- **golden 録音**、両方の深さ、`ft4_ddc_equivalence` の新しい 3 つ目の
  arm で: 11 個の相異なるデコード、FFT フロントエンドと同じ 11 メッセージ、
  refine 後の `i0` は同一で、11 候補のうち 1 つが 1 Hz グリッドステップ
  1 つ分ずれる — 2 ステージ arm が持つのと同じシグネチャである。
- **エイリアシング**、共有ステージが持ち込む、フルレート経路には起こり
  得ない唯一の失敗モード: `6 000 − f` の音は `f` に折り返されるので、
  探索バンドの上端の候補にとって危険な入力は 3 206 Hz から始まる。
  `shared_rejects_content_that_folds_into_the_band` は 3 250-5 000 Hz を
  50 dB より大きく下に固定する。これが、コーナーが 2 800 Hz で、ステージ
  が最初に想定した 111 ではなく 165 タップである理由である (§37.1)。

**まだ実行していない: 560 ファイルのペア sweep。** `ft4_ddc_equivalence` の
tier-C テストは今や FFT と DDC の隣に共有 arm を持ち、同じ 2 % のフロアを
使うが、`ft4_sweep` コーパスはこれをビルドしたマシンにない。それが
§37.1 の再検証のうち、まだ残っている唯一の部分である — golden と ENBW は
1 つの録音でのバンドとデコードを覆う。crossing が動かなかったことを
言えるのは sweep だけである。

### 42.1 ストリーミング化、§32 が coarse ステージに行ったのと同じ動き

共有ステージは最初の実装では、§32 以前の coarse ステージと同じ形をして
いた: スロットが閉じた*後*に行われるブロック単位の仕事で、音声がまだ
到着している間にできるものである。そこで `SlotAccum` は今や
`Ft4SavgBuilder` と並べて `SlotDecimator` を駆動し、`CapturedSlot` は音声
とピリオドグラムの隣に 6 kHz のスロットを運び、`decode_slot` がそれを読む。

これが安全なのは、出力が音声をどうブロックに切ったかに依存しないからに
すぎない — UAC の read は ~256 サンプルで、何も割り切らない。
`slot_decimator_is_block_independent` がそれを強い方法で固定する: 251、
1、1 024、37、4 096 サンプルのブロックでストリームした結果は、バッファ
全体に対する `decimate_slot` と**ビット一致**であり、これは `FirStage` が
自身の履歴を持ち運ぶことで無料で得られる。

測定、同じデモ、同じ golden スロット、8 スロット:

| front end | candidates | decodes | slot |
|---|---:|---:|---:|
| baseline (÷18 per candidate) | 11 of 12 | 10 | 2 067-2 118 ms |
| shared ÷2, inside the decode | 12 of 12 | 11 | 2 019-2 101 ms |
| **shared ÷2, during capture** | **12 of 12** | **11** | **1 998-2 000 ms** |

3 つすべてで同じ 11 メッセージであり、ホストがデコードするのと同じ 11
である。

**注記すべきことが 2 つあり、その 1 つは説明がついていない。** ばらつきが
潰れる — 2 019-2 101 ms が 1 998-2 000 になり、8 スロットにわたって 2 ms
の幅である — のは、固定の 100 ms の塊を post-slot 経路から外したときに
そう見えるはずのものである。だが*節約*は ~60 ms であり、プローブがこの
ステージを単独で測った ~109 (§37.2) ではない。ステージは移動したから
といって安くなり得ないので、差は、それが今何と重なっているか、あるいは
どう測られているかにあるが、このセクションはどちらかを知らない。それは
均して隠さずに記録しておく: §26.1、§27、§31 はいずれも、プローブとパイプ
ラインの間の差が興味深い部分だった例である。

1 960 ms の締切に対するオーバーシュートは、60-140 から ~40 ms になった。

### 次にすること、この証拠に基づいて

デコード経路には、明らかに残っているものはない: §38.1 は LLR/BP の末尾の
すべてのレバーが閉じていると測定し、§31.1 は dual-core を閉じ、2 つの
フロントエンドステージは今や共有かストリーム化されている。未解決の項目は、
この系列がまだ全く測定していないもの — `FirStage::push_block` 用の
esp-dsp バインディング (`dsps_fird_f32_aes3`)、共有フロントエンドの
560 ファイルペア sweep、そして FT4 ブートモードをリプレイされたスロット
ではなく無線機に対して走らせること — である。

## 43. 予算がスロットの逆の端に固定されていた (2026-09-01)

§34 は候補ループに、`CapturedSlot::closed_us` から測って
`TX_TURNAROUND_BUDGET_MS =
1 960 ms` のデッドラインを与え、§42 は受信機がその内側に余裕で収まって
いると報告した。どちらも数値としては正しいが、どこを起点にするかが
間違っている。

FT4 は高速な QSO のためにあるので、デッドラインはスロットの終わりでは
なく、この局が送信を開始しなければならない瞬間である。0 で始まる
スロットの中では次のようになる:

```text
  0.50 s  the other station's transmission starts
  5.54 s  its frame ends (105 symbols x 48 ms)
  6.04 s  ...plus the +0.5 s of DT the search window allows: every
          sample the decoder can read has now arrived
  7.50 s  slot boundary          <- the old code closed here
  8.00 s  THIS station must be transmitting
```

デコードのウィンドウは **6.04 → 8.00 s** である。旧来の固定方法では、
デコーダに 7.50 s から 1 960 ms を与えていた。つまり応答は 9.46 s に
なり、送信に出ていなければならない時刻より 1.46 s 遅い。**QSO に対応
できるビルドが持っていたのは 500 ms であって、1 960 ms ではなかった。**
幅そのものは正しかった。どちらの導出も同じ 0.5 s を引いているからで
ある。ずれていたのは、ウィンドウが半秒遅れて置かれていた点だった。

旧コードが待機に費やしていた 1.25 s は、どの候補も読まない音声だった。
`ft4_sync_search_window` のウィンドウは `i0 = 667` が上限で、フレームは
105 x 32 = 3 360 ダウンサンプル後サンプルなので、スロットの 5 000 のうち
4 027 までしか読まない。

### 43.1 早く閉じても測定可能なコストはない

`tests/ft4_early_close.rs` は、WSJT-X の golden に対して受信機自身の
パイプラインを、ウィンドウを閉じる位置を変えながら実行する。アームは
2 つあり、より短い区間で平均したピリオドグラム (実際の早期クローズが
行うこと) と、末尾をゼロ埋めしたものである:

| close at | candidates | decodes |
|---|---:|---:|
| 6.041 s (what the search reaches) | 12 | 11 |
| 6.100 s | 12 | 11 |
| **6.250 s (shipped)** | **12** | **11** |
| 6.500 s | 12 | 11 |
| 7.500 s (whole slot) | 12 | 11 |

どちらのアームでも集合は同一で、欠落したデコードも余分なデコードも
ない。これが確立*しない*ことが 2 つある。560 ファイルのスイープは
実行していない (このマシンにコーパスがない) こと、そして golden の DT は
−0.44…+0.30 s に収まっているため、±0.5 s ウィンドウの上端をここでは
何も試していないことである。

**出荷しているクローズは 6.04 ではなく 6.25 s である。** 余分な 0.21 s は
DDC チェーンの群遅延による。ベースバンドのサンプル `n` を、フラッシュ
のゼロではなく実際の履歴から計算するには、チェーンは `n·18 + gd`
までの入力を必要とし、`gd` は 3 段合計でおよそ 2 540 入力サンプルに
なる。これがなければ、DT ウィンドウの上端にある信号の最後のシンボルが
ゼロに対してフィルタされることになる。コストは予算 210 ms で、候補
およそ 1 つ分にあたる。

`ft4_rx::CAPTURE_CLOSE_SAMPLES = 75 000` と
`TX_TURNAROUND_BUDGET_MS = 1 750` (`8.0 − 6.25`) である。`SlotAccum` は
残りの 1.25 s を破棄し、スロットグリッドを 7.5 s グリッドのまま保つ。
そうしないと、各ウィンドウが直前のものより 1.25 s ずつ早く開き、送信
からずれていってしまう。

### 43.2 ボード上で

`ft4-demo`、同じ golden、再固定の直後で 2 コア化の前:

```
slot — 11 of 12 candidates tried, 10 decodes in 1777 ms of 1750 ms — cut 1 at 1.55
slot close spacing 7499-7501 ms, no drift over 17 slots
key-up margin: -27 ms
```

したがってこの時点での実態は §42 より**悪かった**。§42 は、送信する局
が持っていないデッドラインに対して測っていたからである。12 候補で
~2 000 ms かかり、実際の予算は 1 750 である。

## 44. デュアルコアの再測定と、負けたパイプライン (2026-09-01)

§30 は候補の分割を 1.33x と測り、§31.1 は FIR の作業後にそれを 1.17x
と再測定して「デュアルコアは打ち切りとして扱う」と結論した。その結論
は当時の証拠に照らして正しく、いまでは正しくない。§42 の共有デシメー
ションによって、候補ごとの PSRAM ストリーミングの大半がループから
取り除かれたからで、それは分割が伸び悩んだ 3 つの理由のうちの 1 つ
だった。

同じ 12 候補で再実行し、並行してもう 1 つの設計も測った:

| arrangement | 12 candidates | vs 1 core |
|---|---:|---:|
| single core | 1 923 ms | 1.00x |
| **candidate split, two cores** | **1 367 ms** | **1.40x** |
| stage pipeline (DDC ‖ search+BP) | 1 456 ms | 1.32x |

**パイプラインの前提が間違っていた。** これを作ったのは、`Fc32Guard`
がすべての変換にまたがって保持されるプロセスグローバルなスピンロック
なので、変換を含まない前半 (DDC) を専用のコアに置けば、競合する対象が
何も残らないはずだったからである。各ステージの測定値は DDC 792 ms に
対して tail 1 130 ms で、上限は 1.70x だったが、到達したのは 1.32x で、
打ち負かすはずだった候補分割を下回った。ガードは律速要因ではなかった。

### 44.1 本番パスの動作

`ft4_rx::decode_slot` は、コア 1 に固定したワーカーを 1 つ生成し、両方
のコアが、リストを半分に分割するのではなく、共有カーソルから候補を
取る。候補のコストは均等ではなく (§30 の 3 つ目の理由)、またカーソル
方式にすると、コア間の調整なしにデッドラインがコアごとに機能する。
結果はインデックスごとのスロットにロックなしで書き込まれ (カーソルは
各インデックスを 1 度しか渡さない)、dedup の前に候補順に並べ直される。
どちらのコアが先に終わったかによって、画面に表示される内容が変わる
ことはない。ワーカーの生成に失敗しても致命的ではない。ループは
シングルコアで動作し、その旨を表示する。

## 45. メモリの問題はスタックであり、WiFi がそれを露わにした

2 つ目のデコードタスクを追加したことで、内部 DRAM の問題が切実に
なった。FreeRTOS のスタックは内部 DRAM から取られ、§30 は WiFi が
上がった状態での最大の空き内部ブロックを 31 744 B と記録していた。
要求は 32 KB である。

**最初の試みは原因を取り違えていた。** WiFi が接続されると、
`ft4-demo` のスロットは 1 387-1 585 ms から 1 789-1 969 ms になり、明白
な容疑者は FreeRTOS 優先度 23 の WiFi ドライバタスクだった。これは
`fst4_app` が自身のデコーダについて測った内容でもある。そこでネット
ワーク経路に、NTP が時計を合わせたあと無線を止めるモードを追加した。
測定結果は、**無線を止めて 1 786-1 919 ms**、つまり変化なしである。
`esp_wifi_stop` は受信機を静かにするだけで何も解放せず、そもそも劣化
は接続の前、ドライバの*初期化*の時点で始まっていた。

原因はメモリだった。最大の空き内部ブロックが一部始終を追跡していた。
アイドル時 69 632 B、ドライバ初期化後 47 104 B、接続が確立すると
31 744 B で、そしてこの時点でデコーダ自身の確保は PSRAM に落ちる。
これは §26.3 が、2 304 点ワークスペースで 41 % と見積もったものである。

**したがって修正は、スタックに内部 DRAM を無駄遣いするのをやめること
だった。** `board::log_task_stacks` はオーバーフローの危険があるタスク
だけを報告していた。すべてのタスクの余裕を報告させると、次のように
なった:

| task | asked for | actually used |
|---|---:|---:|
| `ft4feed` (the demo's whole decode) | 32 KB | **2 584 B** |
| `ft4_slot` (the app's decode) | 32 KB | same code |
| `ft4_cand` (the core-1 worker) | 32 KB | **4 536 B** |
| `net` (PSRAM-backed) | 24 KB | 1 480 B |

これらの数値はどれも、`decode_pipeline` の FT8 パスから引き継いだもの
で、このワークロードに対して確認されたことはなかった。重要なバッファ
はヒープにあり (`cd0` だけで候補ごとに 40 KB)、スタックに載るのは小さ
なローカル変数である。

各 8 KB で、WiFi を接続したまま無線を止めない状態:

```
slot 1 — 12 of 12 candidates tried, 11 decodes in 1298 ms of 1750 ms
slot 2 — 12 of 12 ...                11 decodes in 1290 ms
slot 3 — 12 of 12 ...                11 decodes in 1315 ms
slot 4 — 12 of 12 ...                11 decodes in 1401 ms
slot 5 — 12 of 12 ...                11 decodes in 1375 ms
core-1 worker stack 3 644-3 656 B free of 8 192
```

**ネットワークを上げた状態のほうが、なかった時のどの測定よりも速く**、
停止/再同期の仕組みは残さず削除した。それは測定が反証した診断の上に
作られていたからである。

(高水位マークのログにも、途中で単位のバグがあった。ESP-IDF の
`uxTaskGetStackHighWaterMark` は、素の FreeRTOS がワード単位で返す
ところをバイト単位で返し、最初の版は
4 倍して「14624 B free of 8192」と表示した。この分母に対してこれほど
大きな数字は、それ自体がエラーメッセージである。)

### 45.1 FT4 の現在地

| | candidates | decodes | slot | budget |
|---|---:|---:|---:|---|
| start of 2026-09-01 | 11 of 12 | 10 | 2 067-2 118 ms | 1 960 ms, anchored to the slot end |
| **now** | **12 of 12** | **11** | **1 290-1 401 ms** | **1 750 ms, anchored to key-up** |

WiFi は終始接続したままである。以前の数値ではそうではなかった。

## 46. Δt ウィンドウを WSJT-X のものに戻す (2026-09-02)

§18-19 はこの受信機の Δt 探索を、WSJT-X の `[-344, 1012]` (±1.0 s)
から `(0, 667)` (±0.5 s) へ狭め、golden と `ft4sim` の DT スイープで
無損失であることを測り、探索ステージで 1.5-1.9x のコストと見積もった。
測定は正しかった。そこから引き出した結論が正しくなかった。

**狭いウィンドウが実際に手放すのは、時計がずれている局である。** そして
実際のバンドでは、そうした局のほうが、追加の予算で拾える低 SNR の局
より多い。2 つの時計誤差は加算されもする。この受信機自身の壁時計に
よるスロット整列はまだ未解決 (#313) なので、UTC の ±0.5 s 内に十分
収まっている局が、*こちら*の ±0.5 s の外にいることがありうる。

**フィクスチャでは、これを示せるはずがなかった。** golden の DT は
−0.44…+0.30 s に収まっており、構成上、狭いウィンドウの内側にある。
「golden で無損失」とは、そのファイルについての記述だった。これを示せ
る計器は §18 自身の DT スイープで、そこではウィンドウの内側で recall
は 100 %、外側で 0 % である。到達範囲は崖であり、端を越えたものは劣化
するのではなく完全に失われる。

### コストの実測

`ft4-demo`、同じ golden、ウィンドウ以外は同じビルド:

| window | candidates | decodes | slot | budget |
|---|---:|---:|---:|---:|
| ±0.5 s | 12 of 12 | 11 | 1 259-1 360 ms | 1 750 ms |
| **±1.0 s (WSJT-X)** | **10-11 of 12** | **9-10** | 1 245-1 435 ms | **1 225 ms** |

**コストは §19 が置いた場所にはない。** 探索ステージが 2 倍になっても
スロットはほとんど動かない。§42 の共有デシメーションと §44 の 2 つ目の
コアの前と比べて、候補に占める割合が小さくなっているからである。
コストになるのは*予算*のほうだ。`i0 = 1012` を覆うと
`CAPTURE_CLOSE_SAMPLES` が 6.25 s から 6.775 s に押され、525 ms は、
1 候補あたり ~115 ms として 1 から 2 候補分にあたる。

そのためデッドラインは最も弱い候補を 1 つか 2 つ切り捨てる。これは
意図的に取ったトレードである。低 SNR の局は失い、DT のずれた局は取り
戻す。

### 525 ms はどこから戻ってくるか

§45 の勘定では、1 スロットあたり 419 ms がドット積ではなく、issue
#352 がその内訳を持っている。候補ごとに ~21 ms は、`FirStage` が
履歴からドット積を取る前に、すべての入力サンプルを履歴にコピーして
いることで、呼び出し側のバッファに対するゼロコピー経路なら、これを
取り除ける。525 ms のうち 254 ms にあたり、ウィンドウが要したコストの
およそ半分を、探索にまったく触れない変更で賄えることになる。

(#352 の下限の残り半分、`CandidateDdc` のサンプルごとの `Vec::push` は
すでに取り込み済みである。現地測定で候補あたり 63.1 → 57.4 ms、
エンドツーエンドで 1 290-1 401 → 1 259-1 360 ms、ビット単位で同一。)

## 47. Δt 探索の内部と、すべてを遅くしたキャッシュ (2026-09-02)

§46 は WSJT-X の ±1.0 s ウィンドウを戻し、そのために予算 525 ms を支払
った。この節は、その一部がどこから来たかを述べる。3 つのステップの
うち 2 つが誤りだったので、この道筋には紙面を割く価値がある。

### 内訳の実測

`ft4-bench`、CoreS3、golden で最も強い候補。探索のコストは
`fixed + cells × per_cell` で、縮退したウィンドウ
(`ib_min == ib_max`) は、新しい API なしで `fixed` を測る:

| window | time | cells |
|---|---:|---:|
| ±1.0 s (WSJT-X) | 86.7 ms | 3 159 |
| ±0.5 s | 57.7 ms | 1 602 |
| degenerate | 28.2 ms | 108 |

→ **1 セルあたり 18.6 µs、固定分 26.2 ms。**

1 セルは 128 複素サンプルの Costas ブロック 4 つ、つまり実数 MAC
2 048 回なので、18.6 µs は **2.18 サイクル/MAC** であり、これはこの
チップのアライン済み `dsps_dotprod_f32_aes3` がやることである。グリッド
スキャンにはもう削れるものが残っていない。

固定分の 26.2 ms は `FlatRef::fill` であり、それだけである:

    18 FlatRef fills   25 718 us
    AlignedCd0 copy         3 us

`fill` は `cos` と `sin` を**サンプルごとに**評価する。4 ブロック × 128
サンプル × 18 回の呼び出し (粗 `df` 9 つ、精 9 つ) である。もう一方の
容疑者だった `cd0` のアライメントコピーは 3 マイクロ秒で、`cd0` が
アラインされていないときだけコピーし、たいていはそうではない。

### 明白な修正は、2.6 倍の退行だった

9 つの粗参照は `df`、`ds_spb`、`ds_rate` に依存し、`cd0` にも候補にも
依存しない。12 候補が同じ 9 つの参照を 12 回作り直していた。そこで、
スロットごとに 1 度だけ構築する。`FlatRef` で ~144 KB で、両方のコアに
`&Ft4CoarseRefs` を渡す。

ボードでの測定: **10-11 ではなく 4-6 候補**、スロット 1 275-1 855 ms。
キャッシュした参照をまずローカルスクラッチにステージングする案は、
書き込んだばかりのバッファはキャッシュが温まっていて、事前計算した
ものはそうではない、という理論に基づいていたが、さらに悪かった。
3-4 候補、1 569-2 295 ms である。

どちらの理論も、診断に耐えなかった:

```text
coarse refs at 0x3fcea190 (INTERNAL) .. 0x3c273760 (PSRAM)
uncached 223 024 us (dots fast 23724 align 0 len 0)
cached   216 941 us (dots fast 23724 align 0 len 0)
```

**同じバイナリ内のキャッシュなしパスが 87 ms から 223 ms に遅くなって
いた。** キャッシュ自体のコードに罪はなかった。144 KB を確保したこと
が原因だった。呼び出しごとの `FlatRef` は各 ~1 KB で、
`CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL` が内部 DRAM に置くほど小さい。
144 KB のキャッシュがそれらを PSRAM に追い出し、探索は、ワーキング
セットが内部にあるときだけ測定された速度で動く。§29 は同じ機構を逆の
側から記録していた (「内部バッファをもう 1 つ 18 KB リークさせるとヒープ
がずれ、次の実行では 0 % [高速ドット] と測定された」)。これは、幸運な
逃げ道としてではなく、2.6 倍の退行として現れた同じ教訓である。

カウンタが除外するものに注意されたい。ドット積は終始 PIE パスに留まって
いた (高速 23 724、低速ゼロ) ので、アライメントの失敗ではなかった。

### 効いたもの: 参照ではなく位相子をキャッシュする

`fill` の高いコストは超越関数であり、したがって保持する価値があるのは、
それが構築する参照ではなく位相子 `e^{j·2π·df·n/ds_rate}` である。4 つの
Costas ブロックはすべて同じ長さなので、`df` ごとに 1 つのテーブルで
すべてを賄える。**128 複素 × 9 df ≈ 9 KB** で、144 KB に対する値である。

```text
coarse refs at 0x3fcedaac (INTERNAL) .. 0x3fceb184 (INTERNAL)
uncached 86 485 us | cached 79 293 us | same result true
```

キャッシュなしパスは 86.5 ms に戻り (追い出しがなくなった)、キャッシュ
ありパスは**候補ごとに 7.2 ms** を節約する。fill は依然として実行され、
依然として呼び出し側のスクラッチに書き込む。`cos`/`sin` が、テーブル
の参照と複素乗算に置き換わるだけで、テーブルは同じ式から構築するので、
結果はビット単位で同一である
(`cached_coarse_refs_are_bit_identical` が `i0`、`freq_hz`、`score` を
ビットパターンで固定する)。

ボード上では:

| | candidates | decodes | slot |
|---|---:|---:|---:|
| ±1.0 s window, no cache | 10-11 of 12 | 9-10 | 1 355-1 435 ms |
| **+ phasor table** | **11-12 of 12** | **10-11** | 1 286-1 458 ms |

§46 が WSJT-X の全ウィンドウを探索するために手放した 1 から 2 候補が
戻り、デコーダが計算する内容は何も変わっていない。

### 持ち帰れる部分

「高コストなものを事前計算する」という考え方が、144 KB の退行と 9 KB
の勝利の両方を生んだ。両者を分けたのは着想ではなく**サイズ**だった。
このボードでは、内部 DRAM のワーキングセットを追い出すほど大きな
テーブルは、無関係なコードを遅くし、遅くなったコードは同じ関数内、
呼び出し箇所 2 つ先にあった。ここで何かをキャッシュする前に、それが
何を追い出すかを問うこと。

## 48. 残っていた AWGN ギャップは、欠けていた AP パスだった (2026-09-13)

§9 は WSJT-X の公表値 -17.5 dB に対する ~1.8 dB のギャップの大半を
埋め、≈0.8 dB を残した。これは後に 0.61 dB (ベースライン -16.89 dB) と
測定された。§10 は BP/OSD のデコード強度を除外した。残りはそのどちら
でもなかった。**WSJT-X はすべての FT4 デコードで a-priori パスを実行
するが、mfsk-core は呼び出し側がヒントを与えない限り何も実行して
いなかった。**

`ft4_decode.f90:328` は `npasses = 3 + nappasses(nQSOProgress)` で、
3 回のプレーン LLR パスのあとに AP パスが続く。その `iaptype = 1`
(`:361-368`) は最初の 29 ビットを CQ パターンにロックし
(`apmask(1:29)=1`、`llrd(1:29)=apmag*mcq(1:29)`)、**局についての知識
をまったく必要としない**。`fst4_decode.f90:419,445` も同じ構造である。

FT8 には issue #190 以来同等のもの、Pass 12 のブラインド CQ があり、
それを追加したことが、FT8 自身の公表値に対するギャップを埋めた。FT4 と
FST4 にはそれがなかった。`BLIND_CQ_MIN_NSYNC` の doc コメントは
`msg::pipeline_ap::ap_passes` の pass 7 が FT4/FST4 における相当物だと
主張していたが、そうではない。pass 7 は相手局のコールサインを必要とし、
これは上流の iaptype 2/3 である。iaptype 1 に対応するものはここには
なく、したがってブラインドの FT4 デコードは AP をまったく試みていな
かった。

2 つの修正が一緒に入り、最初のものが 2 つ目の前提だった。FT4/FST4 の
AP は、ビットのおよそ半分を真値の**反対**にロックしていた。`ApHint` は
メッセージを記述するが、これらのプロトコルは FEC の前にメッセージを
RVEC と XOR するので、コードワードはスクランブルされたメッセージを
運ぶからである。これが訂正されるまでは、常時オンの AP パスは事態を
改善せず悪化させていたはずである。

### 結果 — `ft4sim` スイープ、40 trials/point、1 dB グリッド

| channel | before | after | delta |
|---|---:|---:|---:|
| AWGN | -16.89 dB | **-18.00 dB** | **-1.11 dB** |
| CCIR good | -17.46 dB | -17.62 dB | -0.15 dB |
| CCIR moderate | -15.71 dB | -16.33 dB | -0.62 dB |
| CCIR poor | -16.00 dB | -16.25 dB | -0.25 dB |

AWGN は公表値 -17.5 dB を 0.5 dB **上回る**ようになった。それまでは
0.6 dB 下回っていた。

### この数値を、あるがままに読むこと

**スイープコーパスが送信するのは `CQ JL1NIE PM95`** (`gen_ft4_sweep_wavs.sh`)
なので、ブラインド CQ の事前分布は、送信されているメッセージそのものを
ヒントとして与えている。これはこの変更にとって最良のケースであり、
上流が同じパスを実行する以上、WSJT-X 自身の公表値が享受しているのと
同じ最良のケースでもある。一般的な +1.1 dB ではない。

混在したトラフィックでは効果はなく、重要なことに、無害である。
WSJT-X の golden (`000000_000002.wav`、参照メッセージ 14 件、大半は
交換フレーム) は、シングルパスで 11/14、SIC ありで 14/14 とデコード
され、どちらも **extra 0** である。CQ の事前分布が届かない 3 件は
交換フレームで、何も捏造されなかった。`tests/ft4_ap_scramble.rs` は
precision の線を直接固定する。送信していない局に対するヒントは、3 つの
SNR でも純粋なノイズに対しても、その局を生み出さない。

### 2 つのフォローアップ、完了 (2026-09-13)

`apmag` はここでは `max(|llr|) * 1.01` で、上流の `* 1.1`
(`ft4_decode.f90:327`、`fst4_decode.f90:418`) とは異なり、AP ビットは
WSJT-X が与えるより弱い票しか得ていなかった。いまは
`Protocol::AP_MAG_SCALE` で、FT8 の 1.01 (`ft8b.f90:303`) と FT4/FST4 の
1.1 である。FT8 と FT4 は `Ldpc174_91` を共有し、コーデックはどちらの
プロトコルに仕えているかを判別できないからである。感度への影響は
中立と測定された。これは忠実性の修正であり、利得ではない。

FST4 のスイープも、5 つすべてのサブモードで実行された。**20 セル中
12 セルで +0.00 dB、残りの 6 セルで −0.03 から −0.10 dB、悪化したもの
はなし。** このパスは配線されて FST4 でも到達可能で、FST4-60 の AWGN
80 ファイル中 1 回だけ勝つ。ほとんど勝たないのは、FST4 の nsym=4
ラダーと zsum-OSD が、このパスに到達するまでにすでに収束しているから
である。FT4 のラダーにはそこに実際のギャップがあったが、FST4 には
ない。最悪ケースのコストは実時間で 1.5% と測定された。詳細は
`FST4_BENCHMARK.md` §16 にある。

## 49. FT4 を動かしうる 2 つの変更は、動かさなかった (2026-09-14)

ブラインド CQ パス (§48) は、変更が作業ツリーにある状態で測定された
ので、それが設定したベースラインにはそれが含まれている。その測定の
*後*に 2 つの変更が入り、どちらも FT4 のデコードパスに触れるので、
どちらも、タグ付けの前に、推論で済ませずに再スイープされた。

**`apmag` がプロトコルごとになった。** `apmag = max(|llr|) * scale` は、
AP でロックされたビットが、最も強いチャネル観測をどれだけ上回って
クランプされるかを決める。2 つの LDPC コーデックは両方とも `1.01` を
ハードコードしていた。FT8 (`ft8b.f90:303`) では正しく、`1.1` を使う
FT4 (`ft4_decode.f90:327`) では誤りである。値は今は
`Protocol::AP_MAG_SCALE` である。

**並列の AP エンジンが削除された。** `msg::pipeline_ap` の
`decode_band_ap` / `process_candidate_ap` はなくなり、AP は
`process_candidate_basic` 自身のラダーの末尾の 1 段になり、これで
`ft4/decode.rs` から 89 行が取り除かれた。これは、同じデコードに
別のコードパスで到達するもので、まさに、うっかり曲線を動かしてしまう
種類の変更である。

| channel | before | after |
|---|---:|---:|
| AWGN | −18.00 dB | −18.00 dB |
| CCIR good | −17.62 dB | −17.62 dB |
| CCIR moderate | −16.33 dB | **−16.40 dB** |
| CCIR poor | −16.25 dB | −16.25 dB |

3 セルはビット単位で同一、1 セルは 0.07 dB 良かったが、これは 1 セル
あたり 180 trials における補間の粒度であって、利得ではない。2 つの
仮説はどちらも検証され、否定された。

**`apmag` の結果は、正確に読む価値がある。** §48 では「中立と測定
された」と記録されたが、それはスイープではなくスポットチェックだった。
これがそのスイープであり、一致している。FT4 のクランプを 1.01 から 1.1
に上げても、このコーパスではどの閾値も変わらない。これは依然として
正しい変更である。上流がやっていることであり、WSJT-X からの乖離には
理由が要る。しかし、測定できるものは何も買えておらず、「上流に合わせた」
が利得を含意してしまうより、そう述べるほうが有用である。

FT8 も、予算スケジューラの候補の並べ替え (安い順、最初に勝つ dedup の
前に粗い順へ再ソート。予算が設定されていなくても、dedup の結果を変え
うる) のために、同じ実行で再スイープされた。4 つのチャネルすべてで
ビット単位で同一だった: −21.60 / −21.11 / −20.00
/ −19.67 dB。

## 50. EMBEDDED.md から移した実機ブリングアップのログ (2026-09-16)

以下の記録はもともと `EMBEDDED.md` の `## FT4 on embedded` 節だった。
285 行あり、冒頭は「superseded」と印を付けた status 行で、現行の
status 行はそこから 275 行も下に置かれていた。これは計測の日誌であり、
それはまさにこのファイルの役割である。リファレンスマニュアル側には
結果としての status と、ここへのポインタを残した。

元の文面のまま移した。このファイル自身の §32、§34、§37-38、§42-46
への相互参照も含む — それらは最初からここを指していた。


**Status (2026-08-30, superseded — 本節末尾の [現在地
(2026-09-01)](#現在地-2026-09-01--予算内に収まった受信機)
を参照): ビルドでき、実機で正しくデコードでき、スロット予算を
3.4× 超過している**（2026-08-29 の最適化後。最適化前は 8.8×）。
残る超過は 3 つのステージに分散しており、そのうち 1 つ —
`downsample_cached` — は、ホストで検証済みの DDC フロントエンドが
まるごと取り除く。

### そもそもビルドするのに何が必要だったか

`mfsk-core/Cargo.toml` の `ft4 = []` は、FT4 がバックエンドに依存しない
と常に主張してきたし、実際そのとおりである — `src/ft4/` は
`engine::pipeline` の上に載った trait impl と設定の 745 行で、
`rustfft` も FT8 固有の参照もどこにもない。しかし、この feature を有効に
した組込み crate はこれまで 1 つもなく、`scripts/pre-push-check.sh` にも
`ci.yml` にも `alloc ft4 fft-extern` の段がなかったため、この主張は一度も
検証されていなかった。最初の `cargo check` はちょうど 1 行で失敗した —
`ft4/subtract.rs` の `Vec` に `use alloc::vec::Vec;` がなかったのである。
issue #306 が FST4 で 2 度見つけた抜けとバイト単位で同一だ。現在は
両方のマトリクスにその段が入っている。

FT4 とボードの間には、どちらも 2 のべき乗ではない 2 つの FFT 長が立ち
はだかった。

| 長さ | 場所 | 解決方法 |
|---|---|---|
| `fft1_size = 92_160` | `build_fft_cache`、スロットごとに 1 回 | **ホストで bake** し、`decode_frame` の `precomputed_fft` の継ぎ目に渡す — FST4 が使うのと同じ逃げ道 |
| `fft2_size = 5_120` | `downsample_cached`、候補ごとに 1 回 | `engine::dsp::fft_mixed_5120` — Cooley-Tukey 1024 × 5。既存の `fft_15::fft_5` カーネルを再利用し、`fft_mixed_3840` の 256 × 15 と同じ形 |

`engine::llr::symbol_spectra` のシンボルごとの DFT に新しいカーネルは
要らない。FT4 の `ds_spb = NSPS/NDOWN = 32` は 2 のべき乗だからだ。
(`ft4_coarse_sync` 自身の `NFFT1 = 2304` = 256 × 9 にはもう 1 つ同じ形の
ラッパーが必要だった。`engine::dsp::fft_mixed_2304` がそれで、2026-08-30
に追加された — bench は今も候補リストを bake しており、そのステージは
ホストのスロットで 0.3 ms である。)

### 予算

FT4 のスロットは 7.5 s。送信は 0.5 s に始まり、105 シンボル
× 48 ms = 5.04 s 続くので、フレームは 5.54 s に終わり、デコードに使える
のは **1.96 s** である。WSPR や FST4 のモニターループ — 意図的に余裕を
持たせて作ってあり、超過は fault である — とは異なり、これは FT8 の
15 s スロットと同じ形の予算だ: 本当にきつく、超過は運用上の限界になる。

### 計測

`ft4-bench`、M5Stack CoreS3 @ 240 MHz、`opt-level = 3`、シングルコア、
WiFi なし。WSJT-X golden の `000000_000002.wav` から得た 31 個の粗候補、
シングルパス。ログ: `embedded-poc/m5stack-cores3-app/logs/
ft4-bench_clean_2026-08-29.log`。

| ステージ | 合計 | 候補あたり | 割合 |
|---|---:|---:|---:|
| `downsample_cached` (5120 点逆 FFT) | 2 252 ms | 72.7 ms | 13 % |
| **`ft4_sync_search`** | **13 225 ms** | **424 ms** | **76 %** |
| LLR + BP (`DecodeDepth::EMBEDDED`) | ~1 861 ms | ~60 ms | 11 % |
| **合計、production の呼び出し** | **17 339 ms** | 559 ms | — |
| 同、`DecodeDepth::FULL` | 19 684 ms | 635 ms | — |

**同じアセットでホストと同一の 11 個の distinct decode** が両方の depth
で得られた — つまり ship 設定はここで recall を一切失っておらず、この
ファイルでは OSD は何も買っていない。メモリは一度も問題にならなかった:
PSRAM 7.47 MB と内部 DRAM 240 KB が終始空いており、bench タスクは
96 KB のスタックのうち 16.7 KB を使った。

**17 339 ms 対 1 960 ms は 8.8× の超過である。**

### ボトルネックは統計的ではなく構造的である

31 候補すべてにおける `ft4_sync_search` の候補あたりコスト: **min
423 835 µs、p50 423 897 µs、max 424 684 µs** — ばらつきは 0.2 %。これは
候補依存の何かではなく、固定グリッドの特徴である。
`ft4_sync_search_window` は、候補自身の `dt_sec` に関係なく、すべての
候補で同じ絶対範囲 `[-344, 1012]` のダウンサンプル済みサンプルの窓を
走査し（忠実な移植 — WSJT-X の FT4 デコーダはここでだけ Δt を決める）、
それぞれ 4 Costas ブロック × 4 シンボル × 32 サンプルからなる
約 19 900 個の (Δf, Δt) セルをスコアリングする。これは候補あたり
約 10.2 M 回の複素 MAC であり、その 424 ms は複素 MAC 1 回あたり
およそ 10 サイクルに相当する — その場で位相子を回転させるスカラ f32 の
内側ループである。

したがって、効かせられる箇所はグリッドとその内部の演算であり、どちらも
試みる前に計測できる:

- 内積への **`dsps_dotprod_f32_aes3`** (LX7 PIE)。この crate の
  `dotprod-bench` が、このチップで PIE にどれだけの価値があるかを既に
  計測している。カーネルの実力から見て、約 10 サイクル/MAC はかなり
  遠い。
- **窓を狭める** — **ホストで 2026-08-29 に計測**。
  `docs/notes/FT4_BENCHMARK.md` §18 を参照。production の窓は
  スロット全体ではなく ±1.0 s である（`i0` はダウンサンプル済みサンプル
  で、`dt = 0` は `i0 = 333` にある）。2 つの計測器が **±0.5 s は
  無料** で一致した: 実際の off-air golden では 11 個のデコードがすべて
  生き残り（真の DT は −0.44 … +0.30 s に分布）、探索は計測値
  **1.91×**。また `ft4sim` の DT スイープは窓の端でぴったり硬い崖を示す
  — 内側は 100 %、外側は 0 %、そして閾値付近でも DT が窓内にある限り
  recall の列は列ごとに *同一* だった。窓を狭めて失うのは **到達範囲で
  あって感度ではない**。

### 2 つのレバーを適用、3 つ目は見た目より小さく計測された

3 つとも適用済みで、同じ 31 候補で計測した
(`logs/ft4-bench_opt_2026-08-29.log`、詳細は
`docs/notes/FT4_BENCHMARK.md` §19):

| 構成 | 探索 | 累積 |
|---|---:|---:|
| baseline | 13 225 ms | 1.00× |
| + `FlatRef` / `dot_f32` | 4 447 ms | **2.97×** |
| + `cd0` を内部 DRAM に | 3 937 ms | 3.36× |
| + ±0.5 s の窓 | **2 492 ms** | **5.31×** |

**演算が勝因だった。** `ft4_sync_search_window` は周波数シフトを最も内側の
サンプルループの中で適用し、`(df, i0)` セルごとに回転位相子を最初から
やり直していた — だがその位相子は Costas ブロック *内* のオフセットで
添字付けされるため、`df` ごとの約 340 個の `i0` 位置すべてで同一だった。
`fst4_sync_search` は既にそれを参照側に畳み込んで (`FlatRef`) おり、
それによって `dot_f32` — したがって `dsps_dotprod_f32_aes3` — が使える
素の内積も残る。FT4 も今では同じ
ことを行う。感度は不変（スイープの 4 チャネルすべてで +0.00 dB）で、
golden のステージカウンタも同一だった。

**PSRAM 仮説は誤りだった: 予測した 5-10× ではなく 1.12× だった。**
バイト数は正しかったが、推論が誤っていた — 変更後のアクセスは 2 KB の
スライスに対する逐次の `dot_f32` であり、S3 の PSRAM キャッシュはこれを
うまく捌く。旧ループは帯域律速ではなく演算律速だったのである。
残してある（40 KB、起動時に確保すれば無償）が、レバーではない。
production の FT4 モードでは、いずれにせよ起動時に `worker_arena` 経由で
これが必要になる: WiFi が上がっていると、ここで最大の空き内部ブロックは
31 744 B である。

**スロット合計、production パス: 17 339 ms → 8 642 ms (2.01×)**、両方の
depth でホストと一致する 11 個のデコードは変わらず。3 つすべてを適用した
場合の予測は約 6 686 ms 対 1 960 ms — **8.8× から 3.4× の超過へ**。

残りはもはや 1 つのものではない: downsample 34 % / 探索 37 % /
LLR+BP 29 %。`downsample_cached` は探索と同等になっており、これは
高速化するのではなく DDC フロントエンドが取り除くステージである —
それが次の節だ。

### DDC フロントエンド (2026-08-30 にホストで検証、実機ではまだ)

`mfsk_core::ft4::ddc` は、混合とフィルタリングによって候補ごとの `cd0`
を作るので、この bench がホストで bake する 92 160 点の変換には供給する
ものが何も残らない。詳細は `docs/notes/FT4_BENCHMARK.md` §20。

**FT4 は簡単なケースである。** `fst4::ddc` に有理数リサンプラが必要なのは
`NSPS = 3888 = 2⁴·3⁵` のせいで `3⁵` の分母が残るからだが、FT4 の
`NDOWN = 18` は 12 kHz を割り切り、`666.667 Hz` は既に
`SyncDims::ds_rate` であり、`ds_spb = 32` は 2 のべき乗である。このモジュール
は 2 つの `FirStage` と 2 つのミキサーだけで構成される — `PolyphaseResampler`
も `RxGrid` も不要で、`cd0` より下流は何も変わらない:

```text
12 kHz real i16
  → Mixer(f0 + 31.25 Hz)                   complex @ 12 kHz
  → FirStage A: 199 taps, fc 320 Hz, ÷18   complex @ 666.667 Hz
  → FirStage B: 263 taps, fc 56 Hz,  ÷1    complex @ 666.667 Hz
  → Mixer(−31.25 Hz)                       cd0, f0 at DC
```

**通過帯域はデコードのパラメータであり、フィルタ設計上の自由な選択では
ない。** `downsample_cached` は `[f0 − 31.25, f0 + 93.75] Hz` を残し、
残りをゼロにする — トーンは `f0` から上へ伸びるので `f0` について非対称
である。その後 `process_candidate_basic_impl` が `cd0` を全長にわたって
RMS 正規化し、`LLR_SCALE` はそれを基準に較正されているため、参照帯域の
外側に入り込んだノイズは BP に入るすべての LLR のスケールを変えてしまう
（±333 Hz のベースバンド全体だと約 2.3× 高かっただろう）。そこで
ミキサー対を使う: *帯域* を中心に置き、実数タップで対称にフィルタし、
`f0` を DC に回し戻す。参照に対する等価雑音帯域幅の計測値:
**+0.021 dB**。

**等価性。** WSJT-X golden で、同じ 31 候補から: 両フロントエンドで
11 個の distinct decode、同一の集合、`DecodeDepth::EMBEDDED` と `FULL`
の両方で。精緻化後の同期位置は一度も動かず、11 個のうち 1 候補が
`ft4_sync_search` のグリッド 1 ステップ (1 Hz) 離れた位置に着地する。
tier-C スイープでは、4 チャネルの 50 % crossing にまたがる同じ 560 個の
ノイズ実現で対にして: **FFT 237 decode、DDC 238**、不一致 5 件は 3 対 2
に分かれた。この入れ替えのコストは 0.0 dB である。

### 候補の予算は 2.6x 大きすぎた (2026-08-30)

`ft4_coarse_sync` より後のすべてのステージは候補ごとであり、bench の
探索は `sync_min = 0.05` を渡していた。それは *ノイズフロアより下* である:
`getcandidates4.f90` は平滑化したスペクトルを当てはめたベースラインで
割るので、ノイズは約 1.0 にあり、それより低い閾値はバンド内のあらゆる
ピークを通してしまう。WSJT-X 自身の値は 1.2 (`ft4_decode.f90:195`) である。

4 チャネルの 50 % crossing をまたぐ 560 個のスイープファイルと golden
録音で計測: 0.05 → 1.2 で候補数は 67.1 から 1.6 (スイープ)、31 から
12 (golden) になり、**どちらも recall は同一**。膝は 1.4 にあり、
そこから最初のデコードが落ち始める。
`bench_assets::SYNC_MIN` は今では 1.2 で、bake 済みの候補リストは
再生成した — 31 → 12、両 depth で同じ 11 個のデコード。
`FT4_BENCHMARK.md` の 17-19 節はすべて 31 候補で計測された。

DDC と組み合わせた予測は
`(2 492 + 1 943) × 12/31 ≈ 1 717 ms` 対 予算 1 960 ms — 初めて予算の
内側に入る、**紙の上では**。どちらの変更もボード上では計測されていない。

同じ計測は、逆方向の主張も訂正した: bench の「`EMBEDDED` と `FULL` は
同一にデコードする」は golden では真だが、弱いデータでは偽であり
（crossing で 560 件中 237 対 179）、したがって ship depth は OSD を
省くために recall のおよそ 4 分の 1 を手放している。
`docs/notes/FT4_BENCHMARK.md` §21 を参照。

`fst4::ddc` と同様、これは呼び出し側が手を伸ばす部品であり、ホストの
フロントエンドを入れ替える feature flag ではない。

### 現在地 (2026-09-01) — 予算内に収まった受信機

**本節冒頭の 3.4× は superseded** であり、かつてこの節を締めくくっていた
「ハードウェア計測なし」も同様である。上で予測したものはすべて、その後
CoreS3 上で構築して実行した。記録は `docs/notes/FT4_BENCHMARK.md`
§32-§34、§37-§38、§42 にあり、要約は次のとおり:

| 変更 | 効果 |
|---|---|
| `Ft4SavgBuilder` — 粗ステージが捕捉 *中* に走る (§32) | スロット終了後 761 ms → 6 ms |
| スロット締切に縛られた候補ループ (§34) | 超過が事実ではなく運用上の選択になった |
| 共有デシメーション (§42) | 候補あたり ~188 → ~168 ms |
| そのデシメーションを捕捉パスからストリーミング (§42.1) | スロット 2 067-2 118 ms → **1 998-2 000** |
| 予算をスロット終了ではなく key-up から導出し直す (§43) | QSO 可能な予算は 1 960 ではなく **500 ms** だった |
| 2 コア、共有カーソルから候補を取る (§44) | 候補ループで 1.40× |
| タスクスタックを計測からサイズ決定 (§45) | WiFi 接続中で 1 290-1 401 ms |
| WSJT-X 自身の ±1.0 s Δt 窓を復元 (§46) | 予算 1 750 → **1 225 ms**; 11 decode → 9-10 |

**予算はスロット境界ではなく key-up である。** FT4 は高速 QSO モード
なので、締切は局が送信しなければならない瞬間 — 次のスロットの 0.5 s
後 — であり、捕捉窓は探索が届く範囲の音声が到着した時点で閉じる
（WSJT-X の ±1.0 s の Δt 窓全体で 7.5 s 中 6.775 s、DDC チェーンの
群遅延を含む）。代わりにスロット終了を基準にすると、送信する build が
デコードに使える時間は 500 ms だった。タイムラインは §43、窓がなぜ
WSJT-X のものであり、それが何を犠牲にするかは §46 を参照。

**スタックは内部 DRAM であり、WiFi が取るのも内部 DRAM である。**
デコードタスクは各 32 KB を要求して 2.6-4.5 KB しか使わなかった。
WiFi 接続中はその無駄のせいで最大の空き内部ブロックが 31 744 B まで
下がり、デコーダ自身の確保が PSRAM に押しやられて 1 スロットあたり
400-580 ms を費やした。無線を止めても直らない
(`esp_wifi_stop` は何も解放しない)。直るのはスタックのサイズ決定である。
§45。

再生した golden スロット上の `ft4-demo` — 14 信号、FT8 密度の最悪ケース —
は、今では **12 候補中 12 を実行して 11 をデコード** する。共有
フロントエンド以前は 12 中 11 を実行して 10 で、いずれも 1 960 ms の
トランシーバ予算での話だ。FT4 が実際に目にする 5-10 信号の占有度
(§23) では、ループは何も切り捨てずに終わる。

**共有デシメーションはフロントエンドの後半である。**
`ft4::ddc` の候補ごとのチェーンは、以前は候補ごとにスロットの全
90 000 サンプルを 12 kHz でフィルタしていた。`NDOWN = 18` は
`2 · 9` に因数分解されるので、`SlotDecimator` (165 タップ、÷2、
`FirStage::push_block_real` を通した実数入力) がスロットごとに 1 回
走り、`CandidateDdc::new_half_rate` は同じチェーンを 6 kHz で
101 タップの Hz 単位で走らせる。コーナーは 3 000 ではなく 2 800 Hz
である。新しいナイキストより上の成分が候補の帯域 *の中へ* 折り返す
ためで、タップ数が 111 ではなく 165 なのは、2 700 → 3 300 の遷移では
探索帯域の上端を守れないからだ。参照に対する等価雑音帯域幅は、追加
ステージありで +0.021 dB、なしで +0.021 dB。

**スロットグリッドの整列 (#354)。** FT4 boot mode は今ではグリッドを
UTC に固定する。境界を所有する `SlotAccum` — FT8 パスでは
`Ft8ChunkSink` が所有するのとは異なる — は `anchor_or_reanchor` を得て、
`apps/ft4.rs` から駆動される（時計を所有するのはボード側の半分で、
`time_sync::samples_to_next_slot_12k_ms(7_500)`。共有側の半分は、指示
されたときにだけグリッドを動かす）。最初のライブ音声ブロックが次の窓を
次の 7.5 s 境界に固定し、100 ms を超える位相誤差は再固定する — これが
NTP が RTC で初期化された時計を数秒ステップさせる場合を捕まえる。その後
各スロットのデコードの DT の中央値が残差を trim する — STAGING の
レイテンシと、RTC のみの固定が残したものである — そして整列すると
ゼロに落ち着く。**これが扱わないもの**: 時計が *なく*、かつデコードも
ない cold start。FT4 の粗ステージは `dt = 0` を返すので、FT8 と違って
任意の位相からグリッドを引き上げる `bootstrap_dt_median` がない —
それが #356（電波から位相をロックする）である。

**まだ欠けているもの**: `FirStage::push_block` 用の esp-dsp バインディング
がない (`dsps_fird_f32_aes3`)。共有フロントエンド用の 560 ファイルの
対スイープは書かれているがまだ走らせていない。そして FT4 boot mode は
無線機に対して走らせていない（デフォルトでは bake 済みの golden スロットを
再生し、`MFSK_FT4_REPLAY=0` でそれをオフにする）ので、上のグリッド整列は
ホスト上での推論であり、ハードウェアではまだ確認されていない。


## 51. 実物の `jt9 -5` は WSJT-X 2b9d654 と 3.2.0-rc1 の間で変化しなかった (2026-09-24)

`ft4_decode.f90` は上流で変更された (`MAXCAND` 100 → 200、`syncmin`
1.2 → 1.18、AP の `napwid` 80 → 50; #440)。そのため「対実物の
`jt9`」比較の FT4 側を、`WSJTX/wsjtx` の `967c85a` からビルドした
`jt9` に対して再確認した。`ft4_sweep/` コーパス全体（AWGN と 3 つの
CCIR チャネル、1040 ファイル、`-d1/-d2/-d3`: 3120 出力）と、実物の
`000000_000002.wav`（16 / 19 / 19 decode）について、出力は `2b9d654`
ビルドのものとバイト単位で同一である。したがって crossing も変わらない:
AWGN は `-d1/-d2/-d3` で −17.11 / −17.57 / −18.17 dB であり、
`BENCHMARKS.md` の FT4 比較はそのまま成り立つ。

これはこれらのファイルについてだけの記述である。各ファイルは 1 信号しか
含まず、CLI はコールサインの文脈を設定しないので、`MAXCAND` も `napwid`
も働いていない。方法、スコアラ、踏んだ落とし穴は `FT8_BENCHMARK.md`
§13 と `scripts/score-jt9-sweep.py` にある。
