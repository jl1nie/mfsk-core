# MSK144 感度ベンチマーク — 環境セットアップ

MSK144 の SNR sensitivity sweep (`tests/msk144_snr_sweep.rs`) を
実際の WSJT-X と突き合わせて検証した方法と、その検証をクリーンな
チェックアウトから再現する手順。この調査の経緯は
[#156](https://github.com/jl1nie/mfsk-core/issues/156)
（[#157](https://github.com/jl1nie/mfsk-core/pull/157) の SNR バイアス
修正の follow-up）を参照。

FT4/FST4 のベンチマークハーネス
([`FT4_BENCHMARK.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/notes/FT4_BENCHMARK.md)、
[`FST4_BENCHMARK.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/notes/FST4_BENCHMARK.md))
は `lib/` だけからビルドできる小さな単体 Fortran `*sim` バイナリを
必要とするのに対し、**MSK144 の日常的な回帰テストは WSJT-X の
チェックアウトを一切必要としない** — `tests/msk144_snr_sweep.rs` は
完全に自己完結している（独自の合成信号ジェネレータ、独自のノイズ
モデル、外部プロセス無し）。ここで説明する WSJT-X 比較ステップ
（フル版の `jt9` CLI のビルド）は**一回きりの検証**であり、CI や
日常の開発で繰り返す必要はない。新しい WSJT-X リリースと再照合したい
場合や SNR グリッドを拡張したい場合にのみ再現すればよい。

## 1. 回帰テスト（WSJT-X 不要）

```sh
cargo test --release -p mfsk-core --features "msk144,fft-rustfft,uvpacket,parallel" \
  --test msk144_snr_sweep -- --ignored --nocapture
```

- `--release` と `--features parallel` の両方が重要: このスイープは
  2 ping-length 設定 × 7 SNR ポイント × 20 seed で、各 seed が
  15〜30 秒スロット全体に対するフルの `Depth::Deep`
  `decode_slot()` 呼び出し。`parallel` 無しだと release でも
  約 266 秒（各 seed のトライアルは独立だが逐次実行）、
  `parallel` 有りだと約 22 秒（rayon の `into_par_iter()` で
  SNR ポイントごとの 20 seed を並列化）。
- 出力は単なる recall 表（SNR ポイントごとの hits/20）— pass/fail
  のアサーションは無い。FT4/FST4/FT8 のスイープテストと同様、
  ハードコードされた閾値 dB ゲートはここでは安定した回帰シグナルに
  ならない（FFT バックエンドや浮動小数点の細部に敏感で、実際の
  recall 回帰を反映しないため）。以下のベースラインと目視で照合する。
- CI は main への merge 後、"catchall characterization" スイート
  (`.github/workflows/ci.yml`) 経由で自動実行される — PR で強制的に
  走らせたい場合は `run-full-sweep` ラベルを付与する。

テスト内部の合成器は WSJT-X 本家の `msk144sim.f90` の信号モデルを
Rust で直接再現している（本クレート自身の OQPSK TX パスではない
ので、recall の数字が自クレートの modulator に対して自己成就的に
ならない）:

- スロット中 1 秒ごとに 1 回のメテオピング
  (`t = 1, 2, ..., ntr_period - 1`)、`2.718 * t * exp(-t)` の減衰
  エンベロープ (`makepings.f90`)。
- 連続位相バイナリ FSK キャリア。本クレート自身の LDPC(128,90) +
  CRC-13 エンコーダ出力から、`msk144::decode` 自身の独立オラクル
  ユニットテストが既に使っている差分関係を通じて復元 — つまり
  *メッセージエンコーディング*は本クレート自身のものだが、
  *波形合成*は意図的にそうではない。
- WSJT-X の **2500 Hz 基準帯域幅** SNR convention に較正した
  AWGN（下記参照）— つまりスイープが出力する `snr_db` は本クレート
  自身の生の full-Nyquist-band convention ではなく、WSJT-X 本家が
  報告する SNR と直接比較可能。

## 2. 一回きりの WSJT-X 突き合わせを再現する

### 2.1 前提パッケージ

MSK144 の `jt9` ターゲットは CMake の configure 時点で WSJT-X の
依存関係を**まるごと**引き込む（`find_package(... REQUIRED)` は
どのターゲットをビルドするかに関わらず無条件に実行される）—
FT4/FST4 の単体 `*sim` バイナリよりもはるかに重い前提条件になる:

| 要件 | Ubuntu/Debian パッケージ | 用途 |
|---|---|---|
| Qt5 (Widgets, SerialPort, Multimedia, PrintSupport, Sql, LinguistTools) | `qtbase5-dev qtmultimedia5-dev libqt5serialport5-dev qttools5-dev qttools5-dev-tools` | `fort_qt` からリンクされる |
| Boost (log, log_setup) | `libboost-log-dev libboost-dev` | `find_package(Boost ... log_setup log)` |
| libusb-1.0 | `libusb-1.0-0-dev` | Hamlib のオプション USB リグバックエンド |
| readline | `libreadline-dev` | Hamlib CLI の history |
| autoconf/automake/libtool/pkg-config | 同名 | Hamlib の `./bootstrap` |
| `cmake`, `gfortran`, `libfftw3-dev` | 同名 | 通常は既にインストール済み |

```sh
sudo apt-get install -y \
  qtbase5-dev qtmultimedia5-dev libqt5serialport5-dev \
  qttools5-dev qttools5-dev-tools \
  libboost-log-dev libboost-dev \
  libusb-1.0-0-dev libreadline-dev \
  autoconf automake libtool pkg-config git
```

### 2.2 Hamlib をソースからビルド

`WSJT-X/INSTALL` 本家のレシピは Hamlib の `integration` ブランチを
指定しているが、**このブランチはもう存在しない**（しばらく前に
`master` へマージ済み）。代わりに `master` を使う、それ以外の手順は
そのまま適用できる:

```sh
mkdir -p ~/hamlib-prefix && cd ~/hamlib-prefix
git clone https://github.com/Hamlib/Hamlib src
cd src && ./bootstrap
mkdir ../build && cd ../build
../src/configure --prefix=$HOME/hamlib-prefix \
   --disable-shared --enable-static \
   --without-cxx-binding --disable-winradio \
   CFLAGS="-g -O2 -fdata-sections -ffunction-sections" \
   LDFLAGS="-Wl,--gc-sections"
make -j"$(nproc)" && make install-strip
```

### 2.3 `jt9` と `msk144sim` をビルド

```sh
mkdir -p ~/wsjtx-build && cd ~/wsjtx-build
cmake -D CMAKE_PREFIX_PATH=$HOME/hamlib-prefix \
  -DWSJT_SKIP_MANPAGES=ON -DWSJT_GENERATE_DOCS=OFF \
  /path/to/WSJT-X
cmake --build . --target jt9 -j"$(nproc)"
cmake --build . --target msk144sim -j"$(nproc)"
```

`--target jt9`/`--target msk144sim` はフルの Qt GUI アプリケーション
(`wsjtx` 本体) のコンパイルをスキップする — CMake の configure 自体は
（上記の全依存を要するため）フルで走るが、実際のビルドはこの 2
バイナリだけに絞られる。

**サニティチェック**として、他の用途に使う前にまず vendored golden
WAV で確認する:

```sh
cd /path/to/WSJT-X/samples/MSK144
~/wsjtx-build/jt9 -k -d 3 -f 1477 -F 60 181211_120500.wav 181211_120800.wav
```

`mfsk-core/tests/msk144_wsjtx_samples.rs` の `Golden` テーブルにある
3 件の golden decode を全て再現するはず — message / freq / timing /
SNR 全て一致。デフォルトの `-F 20` (周波数探索許容幅) は 2 番目の
ファイルの 1458 Hz 信号には狭すぎるため、上記のように広げる必要が
ある。

**落とし穴**: `jt9` は実行時のカレントディレクトリに
`decoded.txt` / `jt9_wisdom.dat` / `timer.out` を書き出す —
repo root ではなくスクラッチディレクトリから実行すること。

### 2.4 `msk144sim` の SNR convention

`msk144sim.f90` は注入するノイズを加算する前に
`fac = sqrt(6000.0/2500.0)` でスケーリングしている — シミュレーション
自体は 12kHz サンプルレート／6kHz Nyquist で動くにも関わらず、
`snrdb` 引数は WSJT-X の**2500 Hz 基準帯域幅**に較正されている。
これは本クレートの生の full-Nyquist-band SNR convention
（他の箇所、例えば `msk144::decode::SlotDecode::snr_db` 自身の
`pmax`/`pnoise` 計算で使われているもの）と WSJT-X の報告値との間の
「+10·log10(6000/2500) ≈ +3.8 dB」という換算係数の一次資料的な
裏付けになる — シミュレータのソースから直接確認済みで、推測では
ない。

**落とし穴**: `msk144sim` の RNG はプロセス起動をまたいで
再シード**されない** — 同じ引数で 2 回実行するとバイト単位で
同一の WAV が生成される。独立したノイズ実現値が欲しい場合は
`nfiles > 1` を指定した**単一の**実行呼び出しでのみ得られる
（内部の `do ifile=1,nfiles` ループの中で RNG ストリームが進む）。

```
Usage:   msk144sim       message      TRp freq width snr nfiles
Example: msk144sim "K1ABC W9XYZ EN37"  15 1500  0.12   2    1   # short ping
         msk144sim "K1ABC W9XYZ EN37"  30 1500  2.5   15    1   # long ping
```

`TRp`/`width` がメテオピングのプロファイルを決める:
`TRp=15, width=0.12` は 15 秒スロット中 1 秒ごとに約 0.4 秒のピング、
`TRp=30, width=2.5` は 30 秒スロット中 1 秒ごとに約 2.5 秒のピング。
これらは WSJT-X 本家自身のサンプル例であり、
`tests/msk144_snr_sweep.rs` の "short"/"long" 設定が再現しているのも
これ。

### 2.5 突き合わせの実行

これ用のチェックイン済みスクリプトは無い（一回きりの検証であり、
繰り返す CI ステップではないため）— おおよその形:

```sh
for snr in -9 -8 -7 -6 -5 -4 -3; do
  msk144sim "K1ABC W9XYZ EN37" 15 1500 0.12 "$snr" 20   # 1回の呼び出しで20ファイル
  for f in 000000_*.wav; do
    jt9 -k -d 3 "$f" | grep -qF "K1ABC W9XYZ EN37" && echo hit
  done
done
```

...同じ WAV を `jt9 -k -d 3` と、`msk144::decode::decode_slot()` を
直接呼ぶ使い捨ての Rust バイナリの両方でデコードする（ローカルの
`mfsk-core` チェックアウトへの `path` 依存を持つ小さな Cargo
プロジェクトで十分 — クレート本体に恒久的な bin ターゲットを
追加する必要は無い）。

## 3. 結果 (2026-07-19、WSJT-X `jt9` との比較)

SNR ポイントごとに 20 seed、両設定:

| config | SNR (dB) | jt9 | mfsk-core |
|---|---|---|---|
| short (~0.4 秒 ping) | -7 | 0/20 | 0/20 |
| | -6 | 3/20 | 3/20 |
| | -5 | 17/20 | 16/20 |
| | -4 | 20/20 | 20/20 |
| long (~2.5 秒 ping) | -7 | 1/20 | 0/20 |
| | -6 | 1/20 | 1/20 |
| | -5 | 14/20 | 13/20 |
| | -4 | 20/20 | 20/20 |

**28 セル中 25 セルが完全一致、残り 3 件も 20 ファイル中ちょうど
1 ファイルの差** — 閾値カーブが最も急峻な部分での境界的なノイズ
実現値のブレであり、系統的なギャップではない。50% recall crossing
は両 ping プロファイルとも WSJT-X convention（2500 Hz 基準帯域幅）で
概ね -5.5〜-6 dB。

これは今後 `tests/msk144_snr_sweep.rs` を実行した結果と比較する際の
ベースラインとなる — テスト内蔵の合成器は `msk144sim` とバイト単位
で同一ではない（同じモデルの独立した Rust 再実装で、RNG も独自）
ため、この表と完全一致ではなく*近い*値になるはず。このベースライン
からの数 dB のシフトこそが調査に値するシグナルであり、セルごとの
小さなブレではない。
