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

## 実験ログ

日付つきの節（§5 以降: 試したこと、測ったこと、棄却したこと）は [`docs/historical/FT4_BENCHMARK_LOG.ja.md`](../historical/FT4_BENCHMARK_LOG.ja.md) にあります（節番号は同じ）。現在の結果: [`BENCHMARKS.md`](BENCHMARKS.md)、[`UPSTREAM_EVALUATION.md`](UPSTREAM_EVALUATION.md)、`sweep-baseline.json`。

- 5. `DecodeStrictness` probe (issue #72)
- 6. 数値の再較正 (issue #72, 2026-07-18)
- 7. coherent な full-slot Δt 探索 (`ft4_sync_search`, 2026-07-18)
- 8. `nsync>=18` の OSD-depth ゲート — 本物のバグ、ただし AWGN のギャップの原因ではないと確認 (issue #72, 2026-07-18)
- 9. `ft4_sync_search` の scorer が各 Costas ブロック内で non-coherent だった (issue #72, 2026-07-18) — AWGN で約 1 dB の gain
- 10. BP/OSD の decode 強度 — 大半は棄却、忠実性の fix が 1 件、AWGN では効果なし (issue #72, 2026-07-18)
- 11. 3 セグメント Δt 探索の retry — 実装、測定、revert (issue #72, 2026-07-18)
- 12. 誤った (古い) スコアに対する `osd_score_min` ゲート — 本物の fix、AWGN でさらに約 0.5 dB の gain (issue #72, 2026-07-18)
- 13. Coarse-candidate 段は誤ったアルゴリズムだった — `getcandidates4.f90`
- 14. デッドコードの後始末: `sync2d_refine`/`Sync2dConfig` を削除 (2026-07-20)
- 15. Phase 4 `smax` early-reject — 実装・計測・revert (2026-07-20)
- 16. `refine_freq` の探索半径は 5 倍広すぎた (issue #182 続報, 2026-07-26)
- 17. 最初のハードウェア計測 — M5Stack CoreS3 (2026-08-29)
- 18. Δt 窓の狭小化 — recall にいくらかかるか (2026-08-29)
- 19. §17 の実機数値に対する 2 つの最適化 (2026-08-29)
- 20. DDC フロントエンド — `downsample_cached` を置き換え、コストはゼロ (2026-08-30)
- 21. 候補予算 — この探索パラメータは WSJT-X のものではなかった (2026-08-30)
- 22. ラダーのどの rung がコストに見合うか (2026-08-30)
- 23. バンド占有率 — 設計点、そして予算が実際にどこにあるか (2026-08-30)
- 24. ホストのシミュレーションはボードの代役になるか? FT4 では、なる (2026-08-30)
- 25. ハードウェア上のスロット全体 — 三つの見積りはすべて外れた (2026-08-30)
- 26. combine ステージ: 2 つの誤った理論と 41 % の改善 (2026-08-30)
- 27. DDC: 前提が誤りで、改善は 10 % だった (2026-08-30)
- 28. FT8 も同じ税を払っていた (2026-08-30)
- 29. coherent スコアラの 78 % は高速パスに乗っていなかった (2026-08-30)
- 30. デュアルコアの価値は 2 倍ではなく 1.33 倍 (2026-08-30)
- 31. ブロックモード FIR: 予測の 648 ではなく 172 ms (2026-08-30)
- 32. coarse ステージのストリーミング化と、運だった 100 % (2026-08-30)
- 33. ストリーミング coarse ステージは収録に追いつくか? (2026-08-30)
- 34. 候補ループが実際に守るスロット予算 (2026-08-30)
- 35. FT8 のステージ分割と、ウォーターフォールが払っていたもの (2026-08-30)
- 36. `fixed-point` は実際に FT8 に何をもたらすのか (2026-08-30)
- 37. 予算は余裕ではなくデコードである (2026-08-31)
- 38. LLR/BP の末尾の内側 — それは BP である (2026-08-31)
- 40. FT8 の LLR/BP を f32 で — 見込みは 6 %、スロットは 0.7 % しか動かなかった (2026-09-01)
- 41. ストリーミングスペクトログラムの測定 — u16 は f32 の 0.63× (2026-09-01)
- 42. 共有 decimation、実装してボード上で (2026-09-01)
- 43. 予算がスロットの逆の端に固定されていた (2026-09-01)
- 44. デュアルコアの再測定と、負けたパイプライン (2026-09-01)
- 45. メモリの問題はスタックであり、WiFi がそれを露わにした
- 46. Δt ウィンドウを WSJT-X のものに戻す (2026-09-02)
- 47. Δt 探索の内部と、すべてを遅くしたキャッシュ (2026-09-02)
- 48. 残っていた AWGN ギャップは、欠けていた AP パスだった (2026-09-13)
- 49. FT4 を動かしうる 2 つの変更は、動かさなかった (2026-09-14)
- 50. EMBEDDED.md から移した実機ブリングアップのログ (2026-09-16)
- 51. 実物の `jt9 -5` は WSJT-X 2b9d654 と 3.2.0-rc1 の間で変化しなかった (2026-09-24)
