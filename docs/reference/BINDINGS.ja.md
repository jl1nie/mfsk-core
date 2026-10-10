# mfsk-core — C / Kotlin / Swift バインディング

> **English:** [BINDINGS.md](BINDINGS.md)

Rust 以外から mfsk-core を利用するための文書。Rust ホスト API は
[`LIBRARY.md`](LIBRARY.ja.md)、`no_std` / 組込ターゲットは
[`EMBEDDED.md`](EMBEDDED.ja.md) を参照。

| バインディング | 場所 | ビルド・テスト |
|---|---|---|
| **C / C++** | `mfsk-ffi/`、ヘッダ `mfsk-ffi/include/mfsk.h` | CI `ffi` ジョブ — 両 feature セットでの Rust テストに加え、実在の C++ ドライバ `examples/cpp_smoke/`（マルチスレッド負荷試験を含む） |
| **Kotlin / Android** | `bindings/kotlin/`（C シム + `Mfsk.kt`） | CI `kotlin` ジョブ、デスクトップ JVM 上 |
| **Swift / Apple** | `bindings/swift/`（SwiftPM パッケージ `MfskCore`） | CI `swift` ジョブ、`macos-latest` 上 — XCTest 98件と `aarch64-apple-ios` クロスビルド。**単一デコーダ化の書き換え以降ビルドされていない** |

3つとも同一の C ABI（`mfsk_abi_version()` は 3）の上に載っている。`mfsk.h` は
cbindgen 生成でリポジトリにコミットされており、そのドキュメントコメントが
シンボル単位の正本である。本書はその地図であって、置き換えではない。

## 目次

- [1. 生成物とリンク](#1-生成物とリンク)
- [2. C ABI](#2-c-abi)
  - [2.1 形: デコーダと、呼び出し側が所有するメモリ](#21-形-デコーダと呼び出し側が所有するメモリ)
  - [2.2 1周期をデコードする](#22-1周期をデコードする)
  - [2.3 `MfskParams` — パラメータブロック](#23-mfskparams--パラメータブロック)
    - [2.3.1 `MfskExtras` — ライブラリ独自のオプション](#231-mfskextras--ライブラリ独自のオプション)
  - [2.4 `MfskDecode` — 結果1行](#24-mfskdecode--結果1行)
  - [2.5 ストリーミング取り込み](#25-ストリーミング取り込み)
  - [2.6 送信](#26-送信)
  - [2.7 イントロスペクション](#27-イントロスペクション)
  - [2.8 Q65: リストとサブモード番号](#28-q65-リストとサブモード番号)
    - [2.8.1 JTTY — スロット呼び出しではなく受信器ハンドル](#281-jtty--スロット呼び出しではなく受信器ハンドル)
    - [2.8.2 広帯域 IQ — SDR ストリーム用の受信器ハンドル](#282-広帯域-iq--sdr-ストリーム用の受信器ハンドル)
  - [2.9 メッセージ](#29-メッセージ)
  - [2.10 スレッドとランタイム](#210-スレッドとランタイム)
  - [2.11 エラーとメモリ規則](#211-エラーとメモリ規則)
  - [2.12 シンボル索引](#212-シンボル索引)
- [3. 移行](#3-移行)
  - [3.1 0.13 ABI から](#31-013-abi-から)
  - [3.2 0.12 ABI から](#32-012-abi-から)
- [4. Kotlin / Android](#4-kotlin--android)
- [5. Swift / Apple](#5-swift--apple)

---

## 1. 生成物とリンク

`cargo build -p mfsk-ffi --release` が出力するもの:

* `target/release/libmfsk.so`（Linux / Android 共有オブジェクト）
* `target/release/libmfsk.a`（静的、同梱用）
* `mfsk-ffi/include/mfsk.h`（cbindgen 生成、コミット済み）

タグ付きリリースには `linux-x86_64` のビルド済み tarball が GitHub Release に
添付される（`mfsk-ffi/README.md` 参照）。他プラットフォームはローカルビルドが
必要だが、CI がソース変更のたびに Windows-GNU と Android arm64 を
クロスコンパイルしているため、バイナリが公開されていないだけでビルド検証は
済んでいる。

**`MFSK_API`** は全宣言に付与される。`libmfsk.a` をリンクするときは
`MFSK_STATIC`、DLL 自体をビルドするときは `MFSK_BUILDING` を定義し、
DLL を利用するときは何も定義しない。これが無いと Windows DLL は
リンク可能なシンボルを一切エクスポートせず、Unix 共有オブジェクトは
Rust 内部を含む非 static シンボルを全部エクスポートしてしまう。

呼び出し規約は `extern "C"` 自身のもの、すなわち Windows では全シグネチャが
`__cdecl`。これは生成ではなく文書化で担保している — cbindgen は戻り値型の前に
プレフィクスを置けるが、戻り値型と関数名の間という `__cdecl` の位置には置けない。

| プラットフォーム | リンク行 |
|---|---|
| Linux | `-lmfsk -lpthread -ldl -lm` |
| Android | `-lmfsk -llog -lm`（`-ldl`/`-lpthread` 不要 — どちらも Bionic libc 内） |
| macOS | `-lmfsk -lpthread -lm` |
| Windows (MSVC) | `mfsk.dll.lib` と `ws2_32.lib userenv.lib ntdll.lib bcrypt.lib` |
| Windows (GNU) | `-lmfsk -lws2_32 -luserenv -lntdll -lbcrypt` |

**Android は 16 KB ページアラインメントを要する。** Android 15 には
カーネルページサイズ 16 KB の端末があり、4 KB 向けにリンクされた `.so` は
そこでロードできない — つまり最新ハードウェアでだけ
`UnsatisfiedLinkError` が出る。`.cargo/config.toml` が Android 3 トリプルに
`-C link-arg=-Wl,-z,max-page-size=16384` を設定し、**CI が生成された `.so` に
それが載っていることを assert している**。環境変数の `RUSTFLAGS` は
`target.*.rustflags` とマージされず**上書きする**ため、ワークフローを
1箇所いじるだけで静かに外れうるからである。

---

## 2. C ABI

### 2.1 形: デコーダと、呼び出し側が所有するメモリ

ほぼ全体を2つの規則が覆う。

1. **デコーダがデコードハンドルであり、それは WSJT-X のものである。**
   `jt9 -s` はモードごとに常駐デコーダを1つ走らせ、周期ごとにパラメータ
   ブロックを渡して駆動する。`mfsk_decoder_open` → 1周期ずつデコード →
   `mfsk_decoder_close` はそのモデルそのものである。ハンドルが所有するのは
   上流が周期をまたいで保持するものだけで、他には何も持たない:
   コールサインハッシュテーブル（上流と同じく他のデコーダとは共有しない）、
   FT8 の a7 リスト、Q65 と JT65 の平均、WSPR のコールテーブル。
   `mfsk_decoder_clear` は WSJT-X の "Clear Avg" である。1つのハンドルが
   スロット系の20モード — FT8、FT4、FST4 の5周期、WSPR、JT9、JT65、Q65 の
   10サブモード — をすべて受け持ち、0.12 ABI のファミリごとのデコード関数は
   なくなった。MSK144、JTTY、uvpacket にはデコーダがない（open が
   `MFSK_STATUS_UNKNOWN_PROTOCOL`。JTTY は専用の受信器、§2.8.1）。
2. **確保済みメモリは境界を越えない。** 結果行・合成音声・展開テキストは
   すべて呼び出し側がサイズを決めて所有するバッファに書かれる。解放すべき
   ポインタが存在しないので、呼び出しと解放の間で例外が巻き戻ったときに
   ラッパがリークする、というカテゴリ自体が消える。

ハンドルは `MfskDecoder*`、`MfskStream*`、`MfskJttyReceiver*`、
`MfskIqReceiver*`、`MfskQ65History*`、`MfskQ65Callers*` で、それぞれに
`_open`/`_new` と `_close`/`_free` がある。互いに別の不完全型なので、取り違えは
未定義動作ではなく C の型エラーになる。

### 2.2 1周期をデコードする

最小の流れ。これは `mfsk-ffi/examples/cpp_smoke/main.cpp` が CI で実際に
走らせている形である:

```c
#include "mfsk.h"

MfskParams p;
memset(&p, 0, sizeof p);
p.size = sizeof p;
mfsk_params_init(MFSK_MODE_FT8, &p);        /* モードの既定値。ゼロ埋めでは同じにならない */
p.band_hi_hz = 2600.0f;

MfskStatus st = MFSK_STATUS_INTERNAL;
MfskDecoder *d = mfsk_decoder_open(MFSK_MODE_FT8, &p, NULL, &st);   /* extras が NULL: depth 自身の値 */
if (d == NULL) { /* 理由は mfsk_last_error() */ }

MfskDecode rows[16];
memset(rows, 0, sizeof rows);
for (int i = 0; i < 16; ++i) rows[i].size = sizeof rows[i];
size_t n = 0;
if (mfsk_decoder_decode_i16(d, pcm, n_pcm, 12000, MFSK_PERIOD_NONE,
                            rows, 16, &n) == MFSK_STATUS_OK) {
    for (size_t i = 0; i < n; ++i) {
        printf("%.1f Hz  %.0f dB  %s\n",
               rows[i].freq_hz, rows[i].snr_db, rows[i].text);
    }
} else {
    fprintf(stderr, "%s\n", mfsk_decoder_last_error(d));
}
mfsk_decoder_close(d);
```

open の `params` / `extras` が `NULL` ならモード自身の既定値になる。
`out_cap` は受け取ってよい行数の上限、`*out_len` には**常に**見つかった行数が
入るので、配列が短いと、検出できない切り詰め結果ではなく、必要な件数つきの
`MFSK_STATUS_INVALID_ARG` が返る。再試行は余裕のあるバッファで同じ呼び出しを
もう一度行うこと（`mfsk_decoder_decode_stream` なら同じストリームで）で、すでに
見つかった行から答えるので、デコードはやり直されない。`sample_rate` が 12 000
以外ならリサンプルされる。`decode_f32` はどんなレベルの音声も受け取る: `float` で動くエンジン
（WSPR、JT9、JT65、Q65）はそのまま受け取り、WSJT-X と同じく 16 ビット音声を
取る FT8、FT4、FST4 には固定レベルへ換算して渡す。

```c
MfskDecoder *mfsk_decoder_open(uint32_t mode, const MfskParams *params,
                               const MfskExtras *extras, MfskStatus *out_status);
void         mfsk_decoder_close(MfskDecoder *dec);

MfskStatus mfsk_decoder_decode_i16(MfskDecoder *dec, const int16_t *samples,
                                   size_t n_samples, uint32_t sample_rate, int64_t period,
                                   MfskDecode *out, size_t out_cap, size_t *out_len);
MfskStatus mfsk_decoder_decode_f32(MfskDecoder *dec, const float *samples, ...);
```

**`period`** は周期の UTC グリッド上の番号（`utc_seconds / T`）、単発の録音なら
`MFSK_PERIOD_NONE`（`INT64_MIN`）。連続した周期を必要とする状態 — FT8 の a7、
JT65 と Q65 の平均 — は番号が与えられたときだけ使われ、Q65 の平均は番号に
欠番があると最初からやり直す。

ハンドルが他にできること:

| 呼び出し | 効果 |
|---|---|
| `mfsk_decoder_set_params(d, &p)` | GUI が周期ごとに書き換えるのと同じく、周期の間にパラメータブロックを変える。状態は保たれる |
| `mfsk_decoder_set_extras(d, &e)` | ライブラリ独自のオプションを置き換える。ブロックで未設定のものは depth の値に戻る。モードに無いオプションは `MFSK_STATUS_UNSUPPORTED` で、何も変わらない |
| `mfsk_decoder_set_q65_callers(d, callers)` | Q65 専用: コンテストの呼び出し局リスト（§2.8）をコピーして渡す。`MFSK_CONTEST_GRID_EXCHANGE` なら full-AP リストに加わる。NULL で外す。`set_extras` をまたいで残る |
| `mfsk_decoder_set_on_decode(d, cb, user)` | 呼び出しが返す配列に加えて、各行を**見つかった時点で**渡す。NULL で停止 |
| `mfsk_decoder_set_budget(d, check, user)` | 呼び出し側の述語を候補ごとに1回呼び、`false` が返ったら停止する。デコーダを持つ全モードが `MFSK_CAP_BUDGET` を公開して受け付ける。ビットの無いモードなら `MFSK_STATUS_UNSUPPORTED` |
| `mfsk_decoder_last_budget(d, &report)` | 打ち切りで残ったもの — 飛ばした候補数、実行した段数、飛ばした最良候補がどれほど良かったか、`rows_subtracted`（FT8 `SIC_EARLY` のチェックポイント B・C での引き算。サイズ版管理の構造体の末尾に追加）。予算未設定ならゼロ。WSPR・JT9・JT65・Q65 が立てるのは `exhausted` だけで、件数は 0 のまま |
| `mfsk_decoder_decode_prefix_i16` / `_f32(d, pcm, n, rate, period, out, cap, &len)` | 早期デコード（#572）: 音声が届くたびに、**その周期でここまでの全サンプル**と `period` で呼ぶ。FT8 は 141 696 サンプル（約 11.8 s）でチェックポイント A の行を返し（`stage == MFSK_STAGE_EARLY`）、162 432 では何も返さず、周期全体（180 000）で完全な集合（同じ音声に `decode_i16` が返す行）を返す。それ以外の呼び出しと、早期デコードの無いモードの周期全体までの呼び出しは何も返さない。コールバックは周期を通じて各行を 1 回見て、`delivery` はその周期の呼び出し全体で数える。出力バッファ不足で断られた後の再試行には、保持した行が返る。`MFSK_PERIOD_NONE` なら通常のデコード |
| `mfsk_decoder_delivery_is_exact(d)` | 今のモード・depth・extras で、コールバックが呼び出しの返す行をちょうどそのまま、1 回ずつ同じ順で見るか（`STREAMING.md` §3a）。FT8 の single pass と sniper、`MFSK_DEPTH_FAST` の FT4、FST4、WSPR は `false` で、そこでは `MfskDecode::delivery` で対にする。`set_params` / `set_extras` の後は問い直す |
| `mfsk_decoder_add_callsign(d, "JL1NIE")` | ハッシュテーブルに種を入れ、後の `<...>` を解決できるようにする。ハッシュ化コールを持たないメッセージのモードは `MFSK_STATUS_UNSUPPORTED` |
| `mfsk_decoder_copy_info(d, i, out, cap, &len)` | 直近のデコードの行 `i` の背後にある FEC 情報ビット（`MfskDecode::info_bits` 個） |
| `mfsk_decoder_unpack77(d, msg, out, cap, &len)` | `<...>` をこのデコーダのテーブルで解決する `mfsk_unpack77` |
| `mfsk_decoder_clear(d)` | 周期をまたいで持っているものをすべて忘れる（WSJT-X の "Clear Avg"、`ndepth & 128`） |
| `mfsk_decoder_last_error(d)` | このハンドルのエラー枠。グローバルと違い、スレッドを乗り換えても残る |

予算の述語は候補ごとに呼ばれ、**ライブラリは自前の時計を一切読まない** —
締切は述語が何と比較するかで決まる。これが wasm や、スロット途中でバックグラ
ウンドに回った携帯からでも使える理由である。コールバックと述語はどちらも
`desktop` ビルドでは rayon のワーカーから、複数同時に、完了順に呼ばれうる。
`mobile` ビルドでは呼び出しスレッド上で候補順に呼ばれる。どちらでも、返される
配列が正本である。

### 2.3 `MfskParams` — パラメータブロック

`MfskParams` は WSJT-X の `params` common block（`lib/jt9com.f90`）、つまり
GUI が周期ごとに埋め、デコーダが読むものである。モードは上流のデコーダが読む
ものを読み、残りは `jt9` と同じく無視する。何かを上書きする前に、ライブラリに
モードの既定値を書かせること:

```c
MfskParams p;
memset(&p, 0, sizeof p);
p.size = sizeof p;
mfsk_params_init(MFSK_MODE_FT8, &p);
p.depth = MFSK_DEPTH_NORMAL;
p.rx_freq_hz = 1500.0f;
```

| フィールド | 意味 |
|---|---|
| `size` | 呼び出し側が理解している `sizeof(MfskParams)` |
| `depth` | `ndepth & 7`: `MFSK_DEPTH_FAST` 1、`_NORMAL` 2、`_DEEP` 3。0 は Deep（GUI の既定）。`ndepth` と同じく、探索設定のすべてを決める |
| `flags` | `MFSK_PARAM_AVERAGING`（bit 0、`ndepth & 16`: JT65、Q65）、`MFSK_PARAM_DEEP_SEARCH`（bit 1、`ndepth & 32`: JT65）、`MFSK_PARAM_EME_DELAY`（bit 2、`emedelay`） |
| `ap_mode` | `MFSK_AP_OFF` 0（`lft8apon` オフ）、`_CQ_ONLY` 1（`lapcqonly`）、`_FULL` 2（QSO 文脈が許すすべての仮説）。`_init` はモード自身の既定値を書く: FT8 と JT65 はオフ（GUI の "Enable AP" ボックス）、FT4 は full |
| `contest` | `ncontest`: `MFSK_CONTEST_NONE` 0、`_GRID_EXCHANGE` 1（NA VHF、WW Digi、ARRL Digi、Q65 pileup）、`_EU_VHF` 2、`_FIELD_DAY` 3、`_RTTY_ROUNDUP` 4、`_FOX` 6、`_HOUND` 7 |
| `qso_progress` | `nQSOProgress`: `MFSK_QSO_CALLING` 0、`_REPLYING` 1、`_REPORT` 2、`_ROGER_REPORT` 3、`_ROGERS` 4、`_SIGNOFF` 5 |
| `band_lo_hz`, `band_hi_hz` | 探索する音声帯域（`nfa`、`nfb`）。`_init` は FT8 と FT4 に 200–4000 Hz（`jt9` のコマンドライン）、FST4 に 600–1400 Hz（GUI の F Low / F High）を書き、他のモードはレジストリの帯域を保つ |
| `rx_freq_hz`, `tol_hz` | Rx 周波数とその許容幅（`nfqso`、`ntol`）。NaN は未設定 — 0 Hz は周波数である |
| `tx_freq_hz` | Tx 周波数（`nftx`）。NaN は未設定。FT8 のアプリオリ探索を左右する（`MFSK_CAP_TX_FREQ`） |
| `mycall`, `mygrid`, `hiscall`, `hisgrid` | 自局と QSO 相手。NUL 終端のインラインテキスト（15文字と7文字）、不明なら空。AP 仮説の材料になる |

どのフィールドも素の整数か浮動小数で、`enum` も `bool` も無い。設定ファイルや
新しいヘッダから来た値は、不正な Rust 値になるのではなく、この ABI が拒否できる
誤答になる。**不正なブロックは丸めず拒否する**: 列挙した値の外にある depth、
AP モード、contest、QSO 進行度、および NaN の帯域や `band_hi_hz <= band_lo_hz`
の帯域（`_init` を飛ばした呼び出し側）は `MFSK_STATUS_INVALID_ARG`。

**QSO 文脈は送信側の文脈ではなく、メッセージの文脈である。** AP 仮説は探索を
誘導するのではなくメッセージのビットを固定するので、`hiscall` を置く場所を
誤ればヒントが誤りになり、その AP パスはデコードできない。どのモードも AP なしの
候補を先に試す（Q65 は #555 以降）ので、文脈を誤ったときの代償は、そのための
弱い信号での AP 利得ということになる。

### 2.3.1 `MfskExtras` — ライブラリ独自のオプション

上流に無い、ライブラリが足したもの（モード別）。`mfsk_extras_init` で初期化する。
すべてを「未設定」にする — float は NaN、個数は 0、既定値のある選択肢は -1 —
**ゼロ埋めでは同じにならない**（`strictness` の 0 は Strict、`osd` の 0 は
オフ）。`mfsk_decoder_open` と `mfsk_decoder_set_extras` が受け取り、open に
NULL を渡せば `_init` の値になる。

**モードが持たないオプションは、捨てずに拒否する:** `mfsk_decoder_open` または
`set_extras` が `MFSK_STATUS_UNSUPPORTED` を返し、オプション名は
`mfsk_last_error()`（`set_extras` なら `mfsk_decoder_last_error`）に入る。
呼び出し側は最初のスロットより前に知ることができる。範囲外の値は
`MFSK_STATUS_INVALID_ARG` で、これは存在しないオプションではなく呼び出し側の誤り
である。

| フィールド | 意味 | モード |
|---|---|---|
| `sync_min` | depth の値に対する sync しきい値。NaN は depth の値。**モード間で比較できない** | FT8、FT4、FST4 |
| `max_cand` | depth の値に対する候補数の上限。0 は depth の値 | 全モード |
| `osd` | -1 は depth の値、0 オフ、1 オン | FT8、FT4、FST4 |
| `strictness` | 受理/棄却プロファイル: -1 既定、0 strict、1 normal、2 deep | FT8、FT4、FST4 |
| `strategy`, `sic_rounds` | `MFSK_STRATEGY_DEFAULT` 0、`_SINGLE_PASS` 1（1パス、減算なし）、`_SIC_ROUNDS` 2（`sic_rounds` ラウンド）、`_SIC_EARLY` 3（チェックポイント付きパス）。FT8 と FT4 は WSJT-X と同じく既定で減算する | FT8: 0–3、FT4: 0–2、FST4: 0–1（上流の `fst4_decode` に減算は無い） |
| `eq_mode` | 0 オフ、1 局所の信号ごとの等化。探索ではなく*入力音声*の性質 — アナログフィルタが傾けた通過帯域を平坦にする | FT8、FT4、FST4 |
| `message_filter` | 0 プロトコル自身のメッセージフィルタ、1 コーデックの判定のみ | FT8、FT4、FST4 |
| `a7` | 0 以外で FT8 の a7 リストデコーダ（`ft8_a7.f90`）を有効にする。デコーダ自身の2周期前のデコード結果を使う。`period` が要る | FT8 |
| `sniper_hz` | `rx_freq_hz` を中心とするルーフィングフィルタ探索の半幅、Hz。0 は広帯域探索。**設計上 FT8 のみ** — トランシーバのアナログフィルタを絞った運用に対応する | FT8 |
| `has_ap_hint`, `ap_call1`, `ap_call2`, `ap_grid`, `ap_report` | QSO 文脈の AP とは別の自由形式のアプリオリヒント（指定されればそちらが勝つ）: メッセージのフィールドを順に。CQ なら `ap_call1` は `"CQ"`、`ap_report` は `"RRR"`、`"RR73"`、`"73"` か レポート | FT8、FT4、FST4、Q65 |
| `nb_percent` | インパルスノイズブランカ（WSJT-X の **NB**）: スロット変換の前に、最も大きい `n` パーセントのサンプルを消す。0..=25、0 は何も消さない | FST4（`MFSK_CAP_NOISE_BLANKER`） |
| `nb_sweep_step`, `nb_ftol_hz` | 0 以外（5、2、1）でブランキング水準 `0, step, 2*step, .. 20` パーセントごとに1回ずつデコードする。`nb_ftol_hz`（正の値、併用必須）はブランクしたパスの `rx_freq_hz` を中心とする半幅で、**Rx 周波数が無ければ 0 % のパスだけ**が走る。最大21回のデコード | FST4 |
| `t_early_s`, `t_late_s`, `score_threshold` | 名目開始より何秒前/後からフレームが始まってよいか、および coarse-sync の受理値 0..1。NaN はモード自身の値 | WSPR、JT9、JT65、Q65 |
| `max_cycles_per_bit` | ビットあたりの Fano サイクル数（`wsprd -C`）。0 は depth の値 | WSPR |
| `chase_trials` | Chase の試行回数（`nvec`）。0 は depth の値 | JT65 |
| `pileup` | **Q65 Pileup**: 両コールサインだけを指す AP ヒントは余りの78ビット目を空けておくので、"copied last Tx" フラグつきの返信もマッチする。AP ヒントが要る | Q65 |
| `max_drift` | **Max Drift**、スペクトルビン 0..=50: フレーム全体にわたる線形のトーンドリフトを探して除去する。素の探索の `2*bins+1` 倍のコスト。0 はオフ | Q65 |
| `fading_b90_ts`, `fading_model` | 高速フェージングのメトリック: 拡がり帯域幅×シンボル周期（NaN は素の AWGN）。モデル 0 Gaussian、1 Lorentzian（`fading_b90_ts` と併せてのみ読む） | Q65 |

Pileup の返信は `MfskDecode::flags` に `MFSK_DECODE_FLAG_COPIED_LAST_TX` が
立って返る。WSJT-X ではこれが `#` と表示される。送るには、
`mfsk_encode_q65_flagged` が `copied_last_tx` つきの `mfsk_encode_q65` である。

`mfsk_decoder_set_extras` はブロック全体を置き換える: 未設定のままのものは
depth の値に戻る。2つの構造体はこれまでも伸びてきたし今後も伸びるので、どちらも
size 版管理である: 古いヘッダでビルドした呼び出し側は短い `size` を渡し、
ライブラリはその先頭部分だけを読み、残りは既定値のままにする。

### 2.4 `MfskDecode` — 結果1行

平坦で固定サイズ、あなたの配列に書かれる。`text` はインラインの
`char[MFSK_DECODE_TEXT_LEN]` で、NUL 終端。

**デコードの前に `out[0].size = sizeof(MfskDecode)` を設定する。** ライブラリは自分の
`sizeof` ではなく、この値を間隔として配列を進む。古いヘッダ（短い構造体）でビルドした
プログラムには、各行が配列上のその位置に、知っているフィールドだけ書かれ、`out_cap` 行の
外には何も書かれない。新しいヘッダでビルドしたものは、各行の後ろの部分がそのまま残る。
各行の `size` には呼び出し側の間隔が返るので、配列をそのまま
`mfsk_q65_history_record` や次のデコードに渡せる（#635 より前は書いたバイト数が返り、
新しいヘッダでは `size` がライブラリ自身の `sizeof` に縮んで、履歴側が 1 行目の
後ろの部分から 2 行目を読んでいた）。`0` は「このヘッダの構造体」を意味し、ヘッダと
ライブラリが同じ版のときだけ正しい。4 未満または 4 の倍数でない `size` は
`MFSK_STATUS_INVALID_ARG` で、何も書かない。#607 より前は間隔がライブラリ自身の大きさ
だったため、古い呼び出し側では 2 行目以降が誤った位置に書かれていた。
`mfsk_q65_history_record` も同じ規則で配列を読む。

| フィールド | 意味 |
|---|---|
| `size` | 呼び出し側が理解している `sizeof(MfskDecode)` |
| `mode` | ファミリではなく**具体的なサブモード** — FST4 の5周期も Q65 の10サブモードもそれぞれ別に報告される |
| `text` | デコードされたメッセージ。`<...>` は可能ならデコーダのテーブルで解決済み |
| `freq_hz`, `dt_sec`, `snr_db` | 搬送波、スロットの `dt = 0` 基準からの時間オフセット、2500 Hz 基準帯域での SNR |
| `sync_score` | このデコードの sync スコア。そのモード自身の探索の尺度で、モード間では比較できない。WSPR・JT9・JT65・Q65 と FT8 の a7/a8 リストデコードでは `0.0` で、`MFSK_DECODE_FLAG_HAS_SYNC_SCORE` が立たない |
| `sync_cv` | ブロックごとの sync パワーの変動係数 — 安定したチャネルでは 0 近傍、QSB 下では高くなる。行が持つ唯一のフェージング指標。`sync_score` が無い行では `0.0` で、`MFSK_DECODE_FLAG_HAS_SYNC_CV` が立たない |
| `hard_errors` | FEC が訂正した硬判定誤り数。数を報告しない WSPR・JT9・JT65・Q65 では `0` で、`MFSK_DECODE_FLAG_HAS_HARD_ERRORS` が立たない（クリーンなデコードは、フラグが立った `0`） |
| `info_bits` | `mfsk_decoder_copy_info` が返す情報ブロックの幅: FT8 と FT4 は 91、FST4 は 101、WSPR は 50、JT9 と JT65 は 72、Q65 は 77 |
| `pass` | どのデコードパスが行を作ったか。**プロトコル固有** — 診断用であってロジック用ではない |
| `flags` | bit 0 = `MFSK_DECODE_FLAG_HASH_RESOLVED`（テキストが `<...>` 参照の解決にハッシュテーブルを要した）、bit 1 = `MFSK_DECODE_FLAG_COPIED_LAST_TX`（Q65 Pileup の返信。WSJT-X の `#`）、bit 2–4 = `MFSK_DECODE_FLAG_HAS_SYNC_SCORE` / `_HAS_SYNC_CV` / `_HAS_HARD_ERRORS`（上の3つの数値が、報告しないモードの `0` ではなくそのモード自身の値） |
| `key_bits`, `key` | メッセージの識別キー。`key_bits` ビット（FT8・FT4・FST4・Q65 は 77、JT9・JT65 は 72、WSPR は 50、キー無しは 0）を、最上位ビットから `key` の 10 バイトに詰め、残りは 0。同じメッセージは、テキストが違っても（片方だけ `<...>` が解決した場合）2 つのデコーダで同じキーになる。1 つのメッセージが 2 つの周波数にあってもキーは 1 つ。77 ビット系のモードでは `mfsk_decoder_copy_info` のブロックの先頭 77 ビット |
| `delivery` | この行がその周期の何番目の配信か、または何番目の配信だったか。コールバックに渡す行は自分の位置（0, 1, 2…）を持ち、返却される行は自分だった配信の位置を持つので、両者を厳密に対応づけられる。コールバックが見なかった返却行と、コールバックなしの呼び出しの全行は `-1` |
| `stage` | `mfsk_decoder_decode_prefix_*` の呼び出し列がいつその行を見つけたか: `MFSK_STAGE_EARLY`（周期の終わる前の呼び出し — FT8 のチェックポイント A、約 11.8 s、次の周期で応答するのに間に合う）、`MFSK_STAGE_FINAL`（音声が周期全体だった呼び出し）、通常のデコードでは `MFSK_STAGE_NONE`。末尾に追加 |

### 2.5 ストリーミング取り込み

音声を押し込む1スロット分の入れ物。モード自身の `slot_samples_12k` から
サイズが決まるので、FST4-300 の 3.6 M サンプルのスロットも FT4 の
90 000 サンプルのスロットと同じ扱いになる。IQ 受信器（§2.8.2）と同じく、
サンプル数からモードの UTC グリッドにスロットを切り出す: 時計が無ければ
グリッドは最初のサンプルから自走し — 録音を再生するならそれが正しい —、
あれば、スロットは自分の境界で始まる。完成したスロットは最大1つが待ち、
新しいものがそれを置き換える（`mfsk_stream_dropped` が数える）。

```c
MfskStream *mfsk_stream_open(uint32_t mode, uint32_t sample_rate, MfskStatus *out);
MfskStatus  mfsk_stream_push_i16(MfskStream *s, const int16_t *samples, size_t n);
MfskStatus  mfsk_stream_push_f32(MfskStream *s, const float *samples, size_t n);
uint64_t    mfsk_stream_position(const MfskStream *s);     /* 取り込んだ 12 kHz サンプル数: 自身の時計 */
MfskStatus  mfsk_stream_set_time(MfskStream *s, int64_t utc_ns, uint64_t at_sample,
                                 int32_t *out_change);     /* MFSK_CLOCK_* */
bool        mfsk_stream_slot_ready(const MfskStream *s);
bool        mfsk_stream_slot_is_whole(const MfskStream *s);
MfskStatus  mfsk_stream_set_prefix_points(MfskStream *s, const size_t *points, size_t n);
uint64_t    mfsk_stream_dropped(const MfskStream *s);
size_t      mfsk_stream_take_slot_i16(MfskStream *s, int16_t *out, size_t cap,
                                      int64_t *out_period, int64_t *out_utc_ns);
void        mfsk_stream_clear(MfskStream *s);
void        mfsk_stream_close(MfskStream *s);

/* 融合版: リングから直接デコードする */
MfskStatus  mfsk_decoder_decode_stream(MfskDecoder *dec, MfskStream *stream,
                                       MfskDecode *out, size_t out_cap, size_t *out_len,
                                       int64_t *out_period, int64_t *out_slot_start_utc_ns);
MfskStatus  mfsk_decoder_prefix_points(const MfskDecoder *dec, size_t *out, size_t cap,
                                       size_t *out_len);
```

**`Instant` も `SystemTime` も、いかなる時計も使わない。** ホストは、サンプル
`at_sample`（`mfsk_stream_position` が数えるもの）が UTC の `utc_ns` だったと、
読み取りを得るたびに伝える。ストリームは最大 400 ppm でその読み取りに追従する
ので、ノイズのある読み取りや時計のドリフトはスロット境界をミリ秒単位で動かす
だけで、何も失わない。`*out_change` には `MFSK_CLOCK_FIRST` 0（アンカー設定）、
`MFSK_CLOCK_SLEWED` 1（スルー上限以内で移動。開いているスロットは影響なし）、
`MFSK_CLOCK_STEPPED` 2（1秒超のずれ: 時計を再アンカーし、跳びをまたいだスロットは
捨てる）のいずれかが入る。

take してから decode するより `mfsk_decoder_decode_stream` を使うこと。
FST4-300 のスロットを取り出して渡し直すのは 7 MB を無駄に動かすだけである。
これはスロット自身の番号を period として使い（呼び出し側が数えなくても a7 や
平均が連続した周期を見る）、スロットがまだ無いときは `*out_len = 0` で
`MFSK_STATUS_UNSUPPORTED` を返すので、呼び出し側は `mfsk_stream_slot_ready` の
代わりにこれを poll できる。ストリームとデコーダは同じモードでなければならない
（さもなくば `MFSK_STATUS_INVALID_ARG`）。スロットに切られないモードはストリームを
開けない（`UNSUPPORTED`）。`mfsk_stream_take_slot_i16` はサンプルが欲しい呼び出し
側のためにスロットをコピーして取り出す: 書いたサンプル数を返し（スロットが無い、
または `cap` が小さいと 0）、スロットの period と、時計があれば UTC 開始時刻を
返す（時計が無ければ `*out_utc_ns` は 0）。

**ストリームの早期デコードはオプトイン（#601）**: `mfsk_decoder_prefix_points` は、デコーダの
`decode_prefix` の一連の呼び出しが周期全体より前に処理を行う位置を返す。FT8 の Normal または
Deep（`SicEarly` 戦略）なら `141696, 162432`、それ以外のモードと設定では無し。params や extras を
変えたら取り直す。それを `mfsk_stream_set_prefix_points` に渡すと、次に開くスロットから、
ストリームは各ポイントで**それまでのスロット**を用意し、その後スロット全体を用意する。
`mfsk_decoder_decode_stream` は用意された先頭部分を `decode_prefix` と同じようにデコードするので、
チェックポイント A の行は約 11.8 s で `stage == MFSK_STAGE_EARLY` 付きで返り、`on_decode`
コールバックにも届く。スロット全体の呼び出しは周期の完全な集合を返し、その中の早期の行は早期の
印を保つ。先頭部分とスロット全体の区別は `mfsk_stream_slot_is_whole` で、
`mfsk_stream_take_slot_i16` は先頭部分なら短い長さを返す。取られていない先頭部分は同じ周期の
新しい配信に置き換えられ、`mfsk_stream_dropped` には数えない。時計が後ろへ跳んだ後、一部を渡し済みの
周期は再び渡さないので、2 つの録音を継ぎ合わせたデコードは起きない。オプトインなのは、既存の
`take_slot_i16` の呼び出し側に突然短いスロットが渡らないようにするため。`n == 0` で無効になる。

### 2.6 送信

3段階。各段が呼び出し側のサイズ済みバッファに書き込む:

```text
mfsk_pack77*  →  mfsk_message_to_tones  →  mfsk_tones_to_i16 / _f32
```

バッファは `mfsk_symbol_count(mode)` と `mfsk_synth_output_len(mode)` で
サイズを決める。**定数を焼き込まずライブラリに訊くこと** — FST4 の5サブモードは
シンボルあたりサンプル数が 30 倍違う（720 → 21 504）ので、60A から取った定数は
残り4つで静かに誤りになる。

7つの `mfsk_encode_*` ヘルパは、よくある
`call1 / call2 / report` メッセージ用のワンコール近道である:

```c
MfskStatus mfsk_encode_ft8(const char *call1, const char *call2, const char *report,
                           float freq_hz, float *out, size_t cap, size_t *out_len);
```

同様に `mfsk_encode_ft4`、`_fst4s60`、`_wspr`（call, grid, power_dbm）、
`_jt9`、`_jt65`、`_q65`（先頭に `submode`）。type-1 / type-4 / フリーテキストの
メッセージは `mfsk_pack77_*` とトーンパイプラインを通すこと。

### 2.7 イントロスペクション

```c
uint32_t    mfsk_mode_count(void);                    /* このビルドにあるモード数 */
MfskStatus  mfsk_mode_at(uint32_t index, MfskMode *out);
const char *mfsk_mode_name(uint32_t mode);            /* static、解放不要 */
MfskStatus  mfsk_mode_from_name(const char *name, MfskMode *out);
MfskStatus  mfsk_mode_info(uint32_t mode, MfskModeInfo *out);
uint64_t    mfsk_mode_caps(uint32_t mode);            /* MFSK_CAP_* ビット */
MfskStatus  mfsk_params_init(uint32_t mode, MfskParams *out);
MfskStatus  mfsk_extras_init(MfskExtras *out);
uint32_t    mfsk_abi_version(void);
uint32_t    mfsk_version(void);
```

**`MfskMode` は全モードを指し、その discriminant は ABI である。**
レジストリ項目ごとに1つ＋ MSK144 と JTTY で、一度割り当てたら並べ替えない —
意図的にレジストリのインデックス*ではない*。レジストリへの収録は feature で
決まるので、`q65` 無しのビルドでは以降のインデックスがすべてずれてしまう。
このビルドにどれがあるかは `mfsk_mode_count` / `mfsk_mode_at` が答える。

**ケイパビリティは推測せず公開される。** 語は `mfsk_mode_caps(mode)` か
`MfskModeInfo::caps`。デコーダハンドルが20のスロット系モードすべてを受け持つので、
`MFSK_CAP_DECODE_HANDLE` はもうデコーダが開くかどうかを意味しない。そのモードが
**77ビットメッセージのスロットファミリ**（FT8、FT4、FST4）に属することを意味し、
`MfskParams` の QSO 文脈 AP、a7、スナイパー窓が当てはまるのはそれである。WSPR、
JT9、JT65、Q65 も同じハンドルでデコードされ、固有の `MfskExtras` フィールドを持ち、
持たないオプションは `MFSK_STATUS_UNSUPPORTED` になる。他のビットは、モードが
どのオプションを尊重するかを示すので、呼び出し側は知っているべきことを読める:

| ビット | 定数 | 意味 |
|---|---|---|
| 0 | `MFSK_CAP_DECODE_HANDLE` | 77ビットのスロットファミリ: QSO 文脈 AP、a7、スナイパーが当てはまる |
| 1 | `MFSK_CAP_SNIPER` | 狭帯域の単一目標探索（`sniper_hz`）。**設計上 FT8 のみ** |
| 2 | `MFSK_CAP_AP_NARROW` | 目標指定探索での AP ヒント |
| 3 | `MFSK_CAP_AP_WIDEBAND` | 広帯域探索での AP ヒント |
| 4 | `MFSK_CAP_SIC_ROUNDS` | 平坦な逐次干渉除去（`MFSK_STRATEGY_SIC_ROUNDS`） |
| 5 | `MFSK_CAP_SIC_EARLY` | チェックポイント模倣の早期デコード（`MFSK_STRATEGY_SIC_EARLY`）。FT8 のみ |
| 6 | `MFSK_CAP_OSD` | OSD の*スイッチ*が尊重される。無いときは「切れない」であって「持たない」ではない |
| 7 | `MFSK_CAP_EQ_MODE` | 等化がデコーダに届く |
| 8 | `MFSK_CAP_STRICTNESS` | strictness プロファイルが受け付けて捨てられるのではなく尊重される |
| 9 | `MFSK_CAP_BUDGET` | `mfsk_decoder_set_budget` が受け付けられる: デコーダを持つ全モード |
| 10–15 | `MFSK_CAP_KNOWN_FILTER` … `MFSK_CAP_STREAM_RECEIVER` | `mfsk.h` を参照。`_KNOWN_FILTER`、`_KNOWN_SUBTRACT`、`_FFT_CACHE` は Rust API の記述で、0.12 の `keep_known` / `keep_fft_cache` が無くなって以来 C ABI にそれらの呼び出しは無い |
| 16 | `MFSK_CAP_NOISE_BLANKER` | WSJT-X のインパルスノイズブランカ（`nb_percent`、`nb_sweep_step`）。**FST4 の全サブモードのみ** |
| 17 | `MFSK_CAP_TX_FREQ` | 送信周波数（`tx_freq_hz`）がアプリオリ探索を左右する。**FT8 のみ** |

ビットは `mfsk_core::registry::caps` を写したもので、
`mfsk-core/tests/registry_caps.rs` が両方向でトレイト実装と結びつけている —
トレイトを持たないプロトコルを挙げれば*コンパイル*エラー、トレイトを実装して
ビットを立て忘れれば実行時失敗になる。`mfsk-ffi/tests/mode_introspection.rs` が
最後の輪を閉じ、各 `MFSK_CAP_*` を写し元のレジストリ定数と比較する。この連鎖が
あるのは、手書きのケイパビリティ表が2リリースで嘘になるからである。

**`mfsk_mode_defaults` はもう無い。** 既定値はデータで、`mfsk_params_init` が
書く。`MfskExtras` は探索設定を「depth 自身の値」のままにするので、モード間で
読み違える per-mode の `sync_min` は存在しない（FT4 のベースライン正規化スコア、
FT8 と FST4 の絶対 Costas スコア、WSPR・JT9・JT65・Q65 の 0‥1 の sync 比率は比較
できない）。`mfsk_params_init` はデコーダを持たないモード（MSK144、JTTY、
uvpacket）に対して `MFSK_STATUS_UNKNOWN_PROTOCOL` を返す。

**`MfskModeInfo::decode_fft1_size` は予算を立てる前に読むべきフィールドである。**
デコーダがスロット全体にかける順方向 FFT の長さで、FT4 は 92 160 点、FST4-300 は
**4 194 304** 点 — 45 倍の差があるのにほかのどのフィールドも示唆しておらず、
「全モード同じ呼び出し形」がスマートフォンのメモリ事情として誤りである理由
でもある。

**サイズ版管理。** `MfskModeInfo`、`MfskParams`、`MfskExtras`、`MfskDecode` ほか
の行はすべて先頭に `size` を持つ。`sizeof` を設定する（または構造体をゼロ埋め
するとライブラリが埋める）。ヘッダより新しいライブラリは宣言された先頭部分
だけを書き、`size` を実際に書いた量に書き換え（`MfskDecode` の配列の行では
呼び出し側の間隔にして、配列がそのまま読めるようにする）、入力の構造体も宣言された先頭
部分だけを読む。

`mfsk_abi_version()` を `mfsk_version()` と別にしているのは意図的で、クレート
バージョンは境界と無関係な理由でも動くからである。

### 2.8 Q65: リストとサブモード番号

Q65 は通常のハンドル（§2.2）でデコードされる: Pileup、Max Drift、高速フェージング
のメトリックは `MfskExtras`（§2.3.1）、EME 遅延と平均は `MfskParams::flags`、AP
リストは上流と同じく `mycall`、`hiscall`、`hisgrid`、`qso_progress`、`ap_mode` から
作られる。モードは `MfskMode`（`MFSK_MODE_Q65A30`）で開き、`dt_sec` は WSJT-X の
DT 列と同じくモードの名目開始から測る。ハンドルの外に残るのは、WSJT-X が持つ
2つのリスト — デコーダはそれについて状態を持たず、アプリケーションの時計しか
時計が無いので、あなたが所有するオブジェクトである — と、`mfsk_encode_q65*` が
今も取るサブモード番号である。

`MfskQ65History`（`q65_hist`、直近100件のデコード）: `mfsk_q65_history_new` /
`_free` / `_len` / `_push(freq, text)`、行配列をまるごと渡す
`mfsk_q65_history_record(rows, n)`、DX を入力していない "Decode Again" が読む DX の
コールとグリッドを返す `mfsk_q65_history_lookup(rx_freq, &dx)`。`MfskQ65Callers`
（`q65_hist2`、グリッドつきで呼んだ局を最大50件）: `mfsk_q65_callers_new` /
`_free` / `_len` / `_get`、`mfsk_q65_callers_record(freq, text, now)`、
`mfsk_q65_callers_expire(now)`、`mfsk_q65_callers_remove(call)`。時刻は呼び出し側が
渡す Unix 秒で、ライブラリは時計を読まない。どちらもスレッドセーフではない。
コンテストのリストは `mfsk_decoder_set_q65_callers` でデコーダに渡り、これは
コピーするので、後でリストを変えたら渡し直す必要がある。

`MfskQ65SubMode` は**独自の番号**を持ち、`a15` が 6 である。これは
`mfsk_encode_q65` と `mfsk_encode_q65_flagged` の `submode` 引数であり、両者が
一致すると思い込まず `MfskMode` に橋渡しすること。

### 2.8.1 JTTY — スロット呼び出しではなく受信器ハンドル

JTTY（WSJT-X 3.2 のキーボードモード）にはスロットが無い。フレームは送信側が
好きなときに始まり、メッセージは複数フレームからなるので、受信器が状態を持ち、
出力はメッセージの*更新*になる。モードは `MFSK_MODE_JTTY`、
`MFSK_CAP_STREAM_RECEIVER` と `MFSK_CAP_ENCODE` を公開し（`MFSK_CAP_DECODE_HANDLE` は
持たない）、
`mfsk_mode_info` は 1 フレームを記述する — `t_slot_s` はフレーム周期
（1.888 秒）、`slot_samples_12k` は 22 656。

```c
MfskJttyParams p;  mfsk_jtty_params_init(&p);        /* rjtty の既定値。NULL でも同じ */
MfskStatus st;
MfskJttyReceiver *rx = mfsk_jtty_open(48000, &p, &st); /* 任意のレート。12000 以外はリサンプル */

for (各オーディオコールバック)  {                     /* チャンクサイズは任意 */
    mfsk_jtty_push_i16(rx, pcm, n);                   /* 完成した窓をデコードしてから戻る */
    MfskJttyUpdate u = {0};                           /* u.size = sizeof u（0 でも可） */
    while (mfsk_jtty_poll(rx, &u) == 1)               /* 1 = 1 行書いた, 0 = 無し, <0 = MfskStatus */
        show(u.id, u.text, u.complete, u.f1_hz);      /* 同じ id の行を置き換える */
}
mfsk_jtty_finish(rx);                                 /* 録音が終わった: 未完の最終行 */
mfsk_jtty_close(rx);
```

デコードは `push` の中で呼び出しスレッド上（と `mfsk_runtime_configure` が
設定したプール）で走る。完成した 0.47 秒分のオーディオあたり数十ミリ秒なので、
UI スレッドではなくワーカーから呼ぶこと。更新はハンドル内のキューで待ち、
キューは**メッセージ単位で合体する** — ポーリングの間に 2 回伸びたメッセージは
最新のテキストで 1 回だけ返る。これは upstream の規則であり、キューの上限
（異なるメッセージ最大 1024。ポーリングしない呼び出し側は古いものを失う）の
根拠でもある。`id` はメッセージの存続中変わらない。ハンドルはスレッドセーフでは
なく、`MfskStream` と同様に一度に 1 スレッドのみ。`jtty` フィーチャ無しのビルドでも
関数は残り、`MFSK_STATUS_UNKNOWN_PROTOCOL` を返す。

**SNR。** `MfskJttyUpdate::snr_db`（`text` の後ろに追加。小さい `size` を渡した呼び出し側には
渡らない）は、そのメッセージの最初のフレームの SNR（2 500 Hz 基準、下限 -17 dB）で、
WSJT-X v3.3.0-beta1 が報告する値と同じ。表示するときは四捨五入する。上流が落ち着いた単純な
推定なので、強い信号は低く出る（真の +30 dB が約 +11、0 dB 以下では 1 dB 以内）。
Kotlin は `MfskJttyUpdate.snrDb`、Swift は `JttyUpdate.snrDb`。

**行が出た理由。** `MfskJttyUpdate::kind`（`snr_db` の後ろに追加）は `MFSK_JTTY_UPDATE_GROWING`（0）、
`_COMPLETE`（1）、`_EXPIRED`（2: 3 フレーム周期以内に続きが来なかった）、`_RECEPTION_ENDED`（3:
`mfsk_jtty_finish` が途中で打ち切った）のいずれかで、WSJT-X v3.3.0-beta1 の `UPDATE_*` と同じ。
後ろの 2 つは `complete` だけでは区別できなかった。行はメッセージごとにまとめられるので、最後の更新の
理由になる。Kotlin は `MfskJttyUpdate.kind`（`MfskJttyUpdateKind`）、Swift は `JttyUpdate.kind`
（`JttyUpdate.Kind`）。

**ライブ音声を流すとき。** 揃えるべきスロットは無く、ライブラリは時計を持たない。
時刻は `open` / `reset` 以降に push したサンプル数で、`MfskJttyUpdate::start_s` はそれを
サンプルレートで割ったもの。UTC が要るなら、最初のサンプルの UTC を控えて呼び出し側で
換算する。ライブ入力での帰結は、**サンプル数が実時間に追従していなければならない**こと。
フレームが前のフレームの続きと判定されるのは、直前から 1〜3 フレーム周期（1.888 秒、
±0.1 秒）後に始まるときだけなので:

- オーディオコールバックがサンプルを*落とす*（アンダーラン、USB の乱れ、ストリームの
  一時停止）と、以降のフレームがすべて早まってメッセージが切れる。落とした量と同じ数の
  ゼロを push するか、長い欠落のあとは `mfsk_jtty_reset` する;
- ソースのクロックが少し速い／遅い（操作者の時計と別のサウンドカード）場合は、その ppm
  誤差だけ時間軸が伸縮する。数百 ppm なら、続きになり得る 3 フレーム周期の間でも ±0.1 秒
  よりはるかに小さい;
- チャンクサイズは無関係で、何かに揃える利点も無い。

`push` は数十ミリ秒かかることがあるので、オーディオコールバックや UI スレッドからではなく
ワーカーから呼び、サンプルはリングバッファで渡す。結果はフレームの終わりから約 0.5 秒後に
出る（窓は最後のサンプルが届いた時点でデコードされる）。MSK144 には同じ問題がより厳しい形で
あり、逐次受信器はまだ無い（#497）。

**送信**は他のモードと同じ 3 段で、77 ビットメッセージの代わりにテキストが入る:

```c
size_t n = 0;
mfsk_jtty_encode_tones("CQ K1ABC CQ", /*profile*/ 0, NULL, 0, &n);   /* サイズ問い合わせ: n = 59 */
uint8_t tones[16 * 59];
mfsk_jtty_encode_tones("CQ K1ABC CQ", 0, tones, sizeof tones, &n);   /* upstream の pack_jtty + genjtty */
int16_t pcm[16 * 59 * 384 + 4096];  size_t m;                        /* 必要量は mfsk_jtty_synth_len(n) */
mfsk_jtty_tones_to_i16(tones, n, 1500.0f, 8000.0f, pcm, sizeof pcm / 2, &m);
```

`mfsk_jtty_encode_tones_ex(text, profile, is_final, ...)` は WSJT-X v3.3.0-beta1 の `is_final`
付きの同じ関数: 0 を渡すと最後のフレームにメッセージ終端フラグを立てないので、入力しながら
分けて送り、後のメッセージで閉じることができる（`mfsk_jtty_encode_tones` は `is_final` = 1）。

`profile` は 0 = 不明、1 = Field Day、2 = RTTY Roundup。整形が変わるのは RTTY Roundup
だけ（シリアル番号と州の候補、`599 5` → `599 005`）。パッカーはフレーム数が最小に
なるように選ぶ: コールサイン・グリッド・レポート・制御フレーズは 1 フレーム、
その他のテキストは 5 文字で 1 フレーム。80 文字超、16 フレーム超、RTTY のシリアルが
収まらない場合は `MFSK_STATUS_INVALID_ARG`（理由は `mfsk_last_error`）。空メッセージは
`OK` で `*out_len = 0`。WSJT-X が `pack_jtty` の外側に持つ F キーテンプレートと N1MM
タグはホスト側の方針であり、ライブラリには無い（線引きは #463）。

### 2.8.2 広帯域 IQ — SDR ストリーム用の受信器ハンドル

`mfsk_iq_*` は `mfsk_core::iq::IqReceiver`（LIBRARY.ja.md §2.7）の C 側の顔です。広帯域の複素 IQ
ストリームを 1 本入れると、ダイヤル周波数ごとにモードを載せた N チャンネルが出てきて、スロットは
サンプル数を基準に UTC で切り出されます。入力が音声ではなく IQ で、受信器が状態（チャンネルごとの
フィルタ、開いているスロット、サンプル時計）を持つので、JTTY と同じく専用のハンドルです。何も探しません。
どのダイヤルがどのモードかは呼び出し側が指定します。**各チャンネルは専用のデコーダ**（ハッシュテーブル、
a7 リスト、平均もチャンネルごとの §2.2 のハンドル）を持ち、`mfsk_iq_add_channel` に渡した `MfskParams` と
`MfskExtras` から開かれます（NULL ならモードの既定値）。

```c
MfskStatus st;
/* 14.200 MHz を中心にした CF32 の 768 kS/s、iq_swap = 0 */
MfskIqReceiver *rx = mfsk_iq_open(768000, 14200000.0, MFSK_IQ_FORMAT_CF32, 0, &st);
/* チャンネルが多いときは 1 つのポリフェーズフィルタバンクを共有する（下の「選択度」を参照）:
   mfsk_iq_open_with(768000, 14200000.0, MFSK_IQ_FORMAT_CF32, 0, MFSK_IQ_CHANNELIZER_PFB, &st); */

MfskParams p;  memset(&p, 0, sizeof p);  p.size = sizeof p;
mfsk_params_init(MFSK_MODE_FT8, &p);                        /* チャンネルごとのオプション。mfsk_decoder_open と同じ */
strcpy(p.mycall, "JL1NIE");

uint32_t ft8, ft4;
mfsk_iq_add_channel(rx, 14074000.0, MFSK_MODE_FT8, &p, NULL, &ft8);  /* INVALID_ARG: 窓に DC が入る、または帯域外 */
mfsk_iq_add_channel(rx, 14080000.0, MFSK_MODE_FT4, NULL, NULL, &ft4);
mfsk_iq_set_time(rx, utc_ns_now, mfsk_iq_samples_in(rx), NULL);  /* 読み取り無し: グリッドはサンプル 0 から自走 */

for (SDR からの各ブロック) {
    mfsk_iq_push(rx, bytes, n_bytes);                       /* 完了したスロットをデコードしてから返る */
    MfskIqDecode d = {0};                                   /* d.size = sizeof d、または 0 */
    while (mfsk_iq_poll(rx, &d) == 1)                       /* 1 = 行を書いた、0 = 無い、<0 = MfskStatus */
        show(d.channel, d.text, d.abs_freq_hz, d.snr_db);
}
mfsk_iq_retune(rx, new_center_hz, &paused, &resumed);       /* チューナが動いた */
mfsk_iq_gap(rx, lost_samples);                              /* サンプルが届かなかった */
mfsk_iq_close(rx);
```

`format` は `MFSK_IQ_FORMAT_CF32`、`_CS16`、`_CS8`（HackRF）、`_CU8`（RTL-SDR、128 = ゼロ）、
`_CS24` のいずれかで、リトルエンディアン、I の次に Q の順です。`mfsk_iq_push` はその形式のバイト列を
受け取り、2 回の呼び出しにまたがって分割されたサンプルは引き継がれます。`iq_swap` が 0 でなければ I と Q
を入れ替えます（サウンドカードの IQ でしばしば必要）。12 000 以上で、12 kHz との比が小さな分数になる
整数レートを受け付け、そうでないものには `mfsk_iq_open` が `INVALID_ARG` の NULL を返します。

チャンネルは FT8、FT4、FST4 の 5 周期のどれか、WSPR、JT9、JT65、Q65 のサブモード（`MfskMode`）を
載せられます。MSK144、JTTY、uvpacket は `INVALID_ARG`、モードに無いオプションは `UNSUPPORTED`、
コンパイルされていないモードは `UNKNOWN_PROTOCOL` です。**使える音声はおよそ 200 Hz から**
（フロントエンドがダイヤルより下の側波帯を落とす必要があるため）です。各スロットは周期番号つきで
デコードされるので、チャンネルの params が有効にしていれば a7 と平均は連続した周期を見ます。
ギャップや再チューンで失われたスロットがあると、その連続は途切れます。

**チャンネルのデコーダは借り物です。** `mfsk_iq_channel_decoder(rx, ch)` はそれを、デコーダを設定する
呼び出し — `mfsk_decoder_set_params`、`_set_extras`、`_add_callsign`、`_unpack77`、`_last_error`、
`_set_q65_callers`、`_clear` — 用の `MfskDecoder*` として返します。スキマーがスロットの合間にチャンネルの
帯域、depth、DX コールを変えるのにこれを使います。**close してはいけません**: チャンネルが削除されるか
受信器が閉じるまで生きていて、デコードは受信器が行うので、その `decode_*` は呼ばないでください。
`mfsk_iq_channel_state(rx, ch)` は `MFSK_IQ_CHANNEL_ACTIVE` 0、`MFSK_IQ_CHANNEL_PAUSED` 1（retune を参照）、
そのチャンネルが無ければ -1 を返します。

`MfskIqDecode` は他の行と同じくサイズ版管理です。`channel`（`add_channel` が返した値）、具体的な
`mode`、`text`、`freq_hz`（音声）、`abs_freq_hz`（ダイヤルにそれを足した値、`double`）、`dt_sec`、`snr_db`、
`period`（モードの UTC グリッド上でのスロット番号。時計が無ければサンプル 0 から数える）、
`slot_start_sample`（IQ ストリームへのインデックス）、`slot_start_utc_ns` を持ち、`has_utc` が時計の
読み取りが設定されたかを示します。`text` の後ろには、`MfskDecode`（§2.4）と同じ名前・同じ意味で
行の詳細が追加されています: `sync_score`、`sync_cv`、`hard_errors`（それぞれ対応する
`MFSK_DECODE_FLAG_HAS_*` ビットが立っているときに有効）、`delivery`、`pass`、`flags`、`key_bits`、`key`。
IQ の行はテキストではなく `key` と `freq_hz` で比べてください。

**時間と不連続**: サンプル数が時計で、ライブラリは時刻源を読みません。
`mfsk_iq_set_time(rx, utc_ns, at_sample, &change)` は、複素サンプル `at_sample`（`mfsk_iq_samples_in` が返す
カウント）が UTC の `utc_ns` だったと伝えます。読み取りを得るたびに呼んでください。受信器は最大 400 ppm
でその読み取りに追従するので、水晶やホスト時計のドリフトはスロット境界をミリ秒単位で動かすだけで、
スロットは失われません。1 秒を超える跳び（`change` が `MFSK_CLOCK_STEPPED`。他は `MFSK_CLOCK_FIRST` /
`_SLEWED`）だけが、それをまたぐスロットを捨てます。スロットは全部届いてからデコードされ、ストリームが
途中から始まったときの部分スロットはデコードされません。`mfsk_iq_gap` は開いているスロットをすべて
捨て（欠落をまたぐ音声はスロットではないため）、時計は進め続けます。`mfsk_iq_retune` もそれらを捨てます。
新しい帯域に音声窓が収まらなくなったチャンネルは、呼び出しを失敗させずに**一時停止**します — ダイヤルと
デコーダは保たれ、後の retune で帯域内に戻れば再開します — 呼び出しは停止したチャンネル数と再開した
チャンネル数を報告します。録音には、ライブのストリームと同じように、終端の後に少し余白が要ります。
スロットの最後の音声サンプルは、それを運ぶ最後の IQ サンプルの数フィルタ長後に出てくるからです。

**早期デコードは既定で有効（#601）**: デコーダにチェックポイントがあるチャンネル（FT8 の Normal
または Deep）は、`mfsk_iq_push` の中で約 11.8 s の時点でもデコードされます。その行はスロットが
揃う前に、チャンネルのデコーダの `mfsk_decoder_set_on_decode` コールバックと `mfsk_iq_poll` に
`stage == MFSK_STAGE_EARLY`（`MfskIqDecode::stage`、末尾に追加）付きで届きます。スロット全体の
デコードは残りの行を積み、早期の行は再び積みません。コールバックも各行を 1 回だけ受け取ります。
代わりに、周期の後半で `<...>` が解決された早期の行は、キューでは未解決のまま残ります。解決済みの
テキストが要るときは `delivery` で対にしてください。デコーダはハンドルが持っているので、変わるのは
行が届く*時刻*だけで、WSJT-X がチェックポイント A の行を表示するのと同じです。ポイントは push の
たびにチャンネルのデコーダの設定から読むので、借り物のデコーダへの `mfsk_decoder_set_params` は
次のスロットから効きます。`mfsk_iq_set_early(rx, channel, false)` でチャンネルごとに無効にでき、
その場合は以前どおりスロット全体だけで、行は `MFSK_STAGE_NONE` です。

**スレッド**: デコードは `mfsk_iq_push` の中で、呼び出しスレッド上、および `mfsk_runtime_configure` が
設定したプール上で走ります（混んだ FT8 のスロットで数百ミリ秒）。UI スレッドや SDR 自身のコールバック
スレッドではなく、ワーカーから push してください。行はハンドル内のキューで待ち、`mfsk_iq_poll` が
取り出します（最大 4096 件で、poll しない呼び出し側は古いものから失います）。コールバックでなく poll なのは
JTTY と同じ理由です。境界をまたぐユーザーデータの契約が要らず、Kotlin、Swift、C# のラッパーも poll の方が
簡単です。ハンドルはスレッドセーフではなく、一度に 1 スレッドです。

選択度は、チャンネルの窓の外で 120 dB です。`mfsk_iq_open` は Direct 経路で、各チャンネルが入力レートで
混合するため、コストはチャンネル数に比例します（768 kS/s で 1 チャンネルあたり 1 コアの 0.92 %）。
チャンネルが多いときは `mfsk_iq_open_with(..., MFSK_IQ_CHANNELIZER_PFB, &status)` で、1 つのポリフェーズ
フィルタバンクを全チャンネルで共有します。768 kS/s で 1 チャンネル 2.7 %、32 チャンネル 10 %、
128 チャンネル 35 % で、Direct の 32 チャンネル 30 % に対して軽くなります。損益分岐は約 4 チャンネルです。
バンクには 40 kS/s 以上が必要で（それ未満は `INVALID_ARG`）、行の形式と他の呼び出しは両方で同じです。

### 2.9 メッセージ

```c
MfskStatus mfsk_pack77(const char *call1, const char *call2, const char *report,
                       uint8_t *out_message77);
MfskStatus mfsk_pack77_type1(const char *call1, const char *call2, const char *grid,
                             uint8_t *out_message77);
MfskStatus mfsk_pack77_type4(const char *nonstd_call, const char *std_call,
                             const char *report, bool is_cq, uint8_t *out_message77);
MfskStatus mfsk_pack77_free_text(const char *text, uint8_t *out_message77);
MfskStatus mfsk_unpack77(const uint8_t *message77, char *out, size_t cap, size_t *out_len);
MfskStatus mfsk_decoder_unpack77(const MfskDecoder *dec, const uint8_t *message77,
                                 char *out, size_t cap, size_t *out_len);
```

`out_message77` はいずれの場合も呼び出し側所有の 77 バイトバッファで、
どれも確保を行わない。`mfsk_pack77_free_text` は名前に反して何も解放せず、
**13文字までのフリーテキストを pack する**関数である。`mfsk_unpack77` は
`<...>` ハッシュ参照を未解決のままにし、`mfsk_decoder_unpack77` はそれを、
そのデコーダ自身のテーブル（デコードが埋めたもの）で解決する。どちらも
`cap` が足りなければ必要なサイズを報告する。

### 2.10 スレッドとランタイム

```c
MfskStatus mfsk_runtime_configure(const MfskRuntimeConfig *cfg);
uint32_t   mfsk_runtime_thread_count(void);
```

* **デコーダはシングルスレッドである。** デコードのたびに自分のハッシュ
  テーブルと平均を変更する。同時実行するスレッドごとに1つ持つこと。別デコーダ
  同士の並行デコードは支援されており、かつ安価である。
* `parallel` が有効でも、それ以外のデコードは rayon の**グローバル**プールを
  使う — `num_cpus` 本・2 MiB スタックのスレッドが初回デコードで遅延生成され、
  join されない。Android ではこれらは ART にアタッチされていないので、
  そこからのコールバックは `JNIEnv` に触れない。iOS では GCD の QoS クラスの
  外に居てオーディオレンダースレッドと競合する。どちらでもアプリを
  バックグラウンドにしても走り続ける。
* `on_thread_start` / `on_thread_stop` は rayon の `start_handler` /
  `exit_handler` に直結し、これが JNI から `AttachCurrentThread` /
  `DetachCurrentThread` を可能にする — すなわちワーカースレッドからの
  デコードコールバックを合法にする。`num_threads = 1` は逐次デコードを
  強制し、これは `parallel` 無しビルドの挙動と同じである。
* **初回デコードより前に、一度だけ呼ぶこと。** 2回目の呼び出しは黙って
  無視されるのではなく `MFSK_STATUS_UNSUPPORTED` を返す。rayon は自分の
  スレッドが park しているかもしれないプールを作り直せないからである。

### 2.11 エラーとメモリ規則

1. **ハンドル**: `mfsk_decoder_open` / `mfsk_decoder_close`、
   `mfsk_stream_open` / `mfsk_stream_close`、
   `mfsk_jtty_open` / `_close`、`mfsk_iq_open` / `_close`、
   `mfsk_q65_history_new` / `_free`、`mfsk_q65_callers_new` / `_free`。
   close と free は `NULL` に対して冪等である。チャンネルのデコーダ
   （`mfsk_iq_channel_decoder`）は受信器から借りたもので、close しない。
2. **結果行・音声・テキスト**は呼び出し側所有のバッファへ入る。返り値で
   解放が必要なものは無い。`const char*` を返す2つ
   （`mfsk_last_error`、`mfsk_decoder_last_error`）は借用ポインタであって
   確保ではなく、`mfsk_mode_name` の文字列は static である。デコード
   コールバックが受け取る行はその呼び出しの間だけ有効で、残したいものは
   コピーすること。
3. **エラー**: `MFSK_STATUS_OK` 以外が返ったら、デコーダ呼び出しなら
   `mfsk_decoder_last_error(d)`、自由関数なら（まだハンドルが無い、失敗した
   `_open` も含めて）`mfsk_last_error()` を**同じスレッドで**呼ぶ。グローバルは
   スレッドローカルなので、ステータスとメッセージの間にスレッドを乗り換える
   Kotlin のコルーチンや Swift の `async` の呼び出し側は NULL を読む。ハンドルごとの
   枠はそのためにある。返るポインタはそのスレッド（またはそのハンドル）で次に
   失敗しうる呼び出しを行うまで有効。

`MfskStatus`: `OK = 0`、`NULL_POINTER = -1`、`INVALID_ARG = -2`、
`UNKNOWN_PROTOCOL = -3`（このビルドに無い、またはそのモードにデコーダが無い）、
`DECODE_FAILED = -4`、`INTERNAL = -5`（常にバグ）、`UNSUPPORTED = -6`（モードは
在るが要求されたものを提供しない）。

### 2.12 シンボル索引

エクスポートされる関数は 98 個:

| 群 | シンボル |
|---|---|
| decoder (22) | `mfsk_params_init` `mfsk_extras_init` `mfsk_decoder_open` `mfsk_decoder_close` `mfsk_decoder_last_error` `mfsk_decoder_set_params` `mfsk_decoder_set_extras` `mfsk_decoder_set_q65_callers` `mfsk_decoder_clear` `mfsk_decoder_add_callsign` `mfsk_decoder_set_on_decode` `mfsk_decoder_set_budget` `mfsk_decoder_last_budget` `mfsk_decoder_delivery_is_exact` `mfsk_decoder_decode_i16` `mfsk_decoder_decode_f32` `mfsk_decoder_decode_prefix_i16` `mfsk_decoder_decode_prefix_f32` `mfsk_decoder_copy_info` `mfsk_decoder_decode_stream` `mfsk_decoder_prefix_points` `mfsk_decoder_unpack77` |
| streaming (12) | `mfsk_stream_open` `mfsk_stream_close` `mfsk_stream_push_i16` `mfsk_stream_push_f32` `mfsk_stream_position` `mfsk_stream_set_time` `mfsk_stream_slot_ready` `mfsk_stream_slot_is_whole` `mfsk_stream_set_prefix_points` `mfsk_stream_dropped` `mfsk_stream_take_slot_i16` `mfsk_stream_clear` |
| introspection (8) | `mfsk_mode_count` `mfsk_mode_at` `mfsk_mode_name` `mfsk_mode_from_name` `mfsk_mode_info` `mfsk_mode_caps` `mfsk_abi_version` `mfsk_version` |
| 送信 (13) | `mfsk_encode_ft8` `mfsk_encode_ft4` `mfsk_encode_fst4s60` `mfsk_encode_wspr` `mfsk_encode_jt9` `mfsk_encode_jt65` `mfsk_encode_q65` `mfsk_encode_q65_flagged` `mfsk_symbol_count` `mfsk_synth_output_len` `mfsk_message_to_tones` `mfsk_tones_to_i16` `mfsk_tones_to_f32` |
| Q65 リスト (13) | `mfsk_q65_history_new` `mfsk_q65_history_free` `mfsk_q65_history_push` `mfsk_q65_history_record` `mfsk_q65_history_len` `mfsk_q65_history_lookup` `mfsk_q65_callers_new` `mfsk_q65_callers_free` `mfsk_q65_callers_record` `mfsk_q65_callers_expire` `mfsk_q65_callers_remove` `mfsk_q65_callers_len` `mfsk_q65_callers_get` |
| メッセージ (5) | `mfsk_pack77` `mfsk_pack77_type1` `mfsk_pack77_free_text` `mfsk_pack77_type4` `mfsk_unpack77` |
| JTTY (15) | `mfsk_jtty_params_init` `mfsk_jtty_open` `mfsk_jtty_close` `mfsk_jtty_set_params` `mfsk_jtty_push_i16` `mfsk_jtty_push_f32` `mfsk_jtty_finish` `mfsk_jtty_reset` `mfsk_jtty_pending` `mfsk_jtty_poll` `mfsk_jtty_encode_tones` `mfsk_jtty_encode_tones_ex` `mfsk_jtty_synth_len` `mfsk_jtty_tones_to_i16` `mfsk_jtty_tones_to_f32` |
| IQ (15) | `mfsk_iq_open` `mfsk_iq_open_with` `mfsk_iq_close` `mfsk_iq_add_channel` `mfsk_iq_channel_decoder` `mfsk_iq_channel_state` `mfsk_iq_set_early` `mfsk_iq_remove_channel` `mfsk_iq_set_time` `mfsk_iq_retune` `mfsk_iq_gap` `mfsk_iq_push` `mfsk_iq_samples_in` `mfsk_iq_pending` `mfsk_iq_poll` |
| ランタイム (3) | `mfsk_last_error` `mfsk_runtime_configure` `mfsk_runtime_thread_count` |

---

## 3. 移行

### 3.1 0.13 ABI から

0.14.0 は ABI バージョン 3 のままである。どの構造体も末尾にフィールドを足しただけで、
シグネチャが変わった関数は無いので、0.13 のプログラムはそのままリンクして動く。
C の呼び出し側が確認すべきなのは挙動である。

| 領域 | 0.13 | 0.14 |
|---|---|---|
| 結果配列の間隔 | 行はライブラリ自身の `sizeof(MfskDecode)` 間隔で書かれた。古い短いヘッダでビルドしたプログラムでは 2 行目以降が誤った位置に入り、バッファの外にも書いていた | 行は呼び出し側の `out[0].size` 間隔。呼び出しの前に `sizeof(MfskDecode)`（このヘッダのものなら 0）を設定する。構造体の大きさになり得ない `size` は `MFSK_STATUS_INVALID_ARG` で、何も書かない（[#607](https://github.com/jl1nie/mfsk-core/issues/607)、§2.4） |
| デコード後の各行の `size` | 書いたバイト数。新しいヘッダの配列ではライブラリの `sizeof` に縮んでいた | 呼び出し側の間隔。配列をそのまま `mfsk_q65_history_record` や次のデコードに渡せる。ライブラリが書いた量は行ではなくヘッダから読む（[#635](https://github.com/jl1nie/mfsk-core/issues/635)） |
| `out` が短くて断られた呼び出しの再試行 | `mfsk_decoder_decode_stream` はスロットを既に取り出していたので、再試行は `MFSK_STATUS_UNSUPPORTED` になった。ほかの `decode_*` はデコードし直し、コールバックが 2 回届き、Q65 の平均が 2 回進んだ | 余裕のあるバッファでの同じ呼び出しには、すでに見つかった行から答え、デコードし直さない（[#633](https://github.com/jl1nie/mfsk-core/issues/633)） |
| 12 kHz 以外のレートの `float` | 呼び出しごとにリサンプルし、16 ビットへピーク正規化していた | レベルを保ってリサンプルし、12 kHz と同じく渡す。ゲイン（RMS）はデコーダが周期ごとに 1 回決める（[#634](https://github.com/jl1nie/mfsk-core/issues/634)） |
| `sync_score`、`sync_cv`、`hard_errors` | WSPR、JT9、JT65、Q65 では `0` | そこでは今も `0`。その行のモードがどれを測ったかは `flags` のビット 2〜4（`MFSK_DECODE_FLAG_HAS_*`）が示す（[#594](https://github.com/jl1nie/mfsk-core/issues/594)） |
| `info_bits` / `mfsk_decoder_copy_info` | FT8、FT4、FST4 のみ | 全モード: WSPR 50 ビット、JT9 と JT65 72、Q65 77。行は `key` も持つ（[#592](https://github.com/jl1nie/mfsk-core/issues/592)） |
| WSPR、JT9、JT65、Q65 での `mfsk_decoder_set_budget` | `MFSK_STATUS_UNSUPPORTED` | 受け付ける。`MFSK_CAP_BUDGET` を公開し、候補ごとに 1 回確認する（[#593](https://github.com/jl1nie/mfsk-core/issues/593)） |
| `mfsk_iq_push` の FT8 チャンネル | スロット全体のみ | **既定で早期デコード**: チェックポイント A の行が約 11.8 s に `stage == MFSK_STAGE_EARLY` で届き、残りは終わりに、繰り返さずに届く。`mfsk_iq_set_early(rx, channel, false)` でスロット全体に戻る（[#601](https://github.com/jl1nie/mfsk-core/issues/601)、§2.8.2） |
| `MfskStream` | スロット全体 | `mfsk_decoder_prefix_points` → `mfsk_stream_set_prefix_points` で選ばない限り変わらない。選んだら、すべての配信を `mfsk_decoder_decode_stream` でデコードする（§2.5） |

新規（すべて追加のみ）: `mfsk_decoder_decode_prefix_i16` / `_f32`、
`mfsk_decoder_prefix_points`、`mfsk_decoder_delivery_is_exact`、
`mfsk_stream_set_prefix_points`、`mfsk_stream_slot_is_whole`、
`mfsk_iq_set_early`。`MfskDecode` に `key_bits`、`key`、`delivery`、
`stage`。`MfskIqDecode` に `MfskDecode` の詳細（`sync_score`、
`sync_cv`、`hard_errors`、`pass`、`flags`、`key_bits`、`key`、`delivery`、
`stage`）。`MfskBudgetReport` に `rows_subtracted`。デコード結果は
`LIBRARY.md` §1.3 の一覧どおりに動く（コールサインのプレフィックスを検査しなくなった、
OSD の行ごとの移植、JT9 のビン以下の補正）。

**Kotlin と Swift** は同じ ABI に従い、行の型はソース互換でなくなる:
`syncScore`、`syncCv` / `syncCV`、`hardErrors` は nullable になり（モードが測らない
ところでは `null` / `nil`）、行に `key`、`keyBits`、`delivery`、`stage` が加わる。
新規: `decodePrefix`、`prefixPoints`、`deliveryIsExact`、ストリームの
`setPrefixPoints` と `slotIsWhole` / `isSlotWhole`、IQ 受信器の `setEarly`、
予算レポートの `rowsSubtracted`。Kotlin の貸し出しチャンネルデコーダは `onDecode` と
`setBudget` を受け付ける。C と同じく、IQ 受信器の FT8 チャンネルは既定で早期デコードする。

### 3.2 0.12 ABI から

0.13.0 で C 側のデコード surface が置き換わった（ABI バージョン 2 → 3）。
`mfsk-ffi` は `publish = false` で、リポジトリ内の C++ ドライバ、Kotlin
バインディング、Swift パッケージも一緒に移ったので、C の消費者はデコード
呼び出しを書き直し、残り（送信、メッセージ、ストリーミング取り込みの push 側、
JTTY、Q65 リスト、ランタイム）はそのまま使える。

| 0.12 | 0.13 |
|---|---|
| `MfskDecodeSession`、`mfsk_session_open` / `_close` | `MfskDecoder`、`mfsk_decoder_open` / `_close` — 2つは別の Rust 値を所有するので意図的に別の型 |
| `MfskDecodeParams` + `mfsk_decode_params_init` | 2つの構造体: `MfskParams`（WSJT-X のパラメータブロック、`mfsk_params_init`）と `MfskExtras`（ライブラリ独自のオプション、`mfsk_extras_init`）。`freq_min_hz` / `freq_max_hz` → `band_lo_hz` / `band_hi_hz`、`freq_hint_hz` → `rx_freq_hz`（+ `tol_hz`）、`tx_freq_hz` はそのまま、`depth` / `strictness` / `eq_mode` / `sync_min` / `max_cand` / `sic_*` / `single_pass` / `search_hz` → `depth` と `MfskExtras` の同名フィールド（`strategy`、`sniper_hz`）、`has_ap_hint`・`ap_*` → `MfskExtras`、`nb_*` → `MfskExtras` |
| 呼び出しごとの `params` を取る `mfsk_session_decode_i16` / `_f32` | `period` を取る `mfsk_decoder_decode_i16` / `_f32`。ブロックの変更は `mfsk_decoder_set_params` / `_set_extras` |
| `mfsk_session_decode_stream(…, params, …, double *utc)` | `mfsk_decoder_decode_stream(…, int64_t *period, int64_t *utc_ns)` |
| `mfsk_session_set_on_decode` / `_set_budget` / `_last_budget` / `_add_callsign` / `_copy_info` / `_last_error` | 名前はそのまま、接頭辞が `mfsk_decoder_` |
| `mfsk_session_keep_known` / `_known_count` / `_keep_fft_cache` | 廃止。デコーダは上流が持つ状態を持つ。結果を持ち越す代わりに FT8 の a7（`MfskExtras::a7`、`period` つき） |
| `mfsk_stream_set_epoch(s, double utc_s)`、`_buffered`、`_take_slot_i16(…, double *utc)` | `mfsk_stream_set_time(s, utc_ns, at_sample, &change)` と `mfsk_stream_position`、`_dropped`、`_take_slot_i16(…, int64_t *period, int64_t *utc_ns)`。ストリームはエポックを受け取る代わりに読み取りに追従する（400 ppm） |
| `mfsk_wspr_decode`、`mfsk_jt9_decode_at`、`mfsk_jt65_decode_at` | `mfsk_decoder_open(MFSK_MODE_WSPR / _JT9 / _JT65, …)` と `mfsk_decoder_decode_*`。帯域は `band_lo_hz` / `band_hi_hz`、フレーム窓は `t_early_s` / `t_late_s` |
| `mfsk_q65_decode`、`_with_ap`、`_fading`、`_with_ap_list`、`_decode_ex`、`mfsk_q65_params_init`、`MfskQ65Params` | Q65 デコーダ1本: `pileup`、`max_drift`、`fading_*`、AP ヒントは `MfskExtras`。`eme_delay` と平均は `MfskParams::flags`。AP リストは QSO 文脈から。コンテストの呼び出し局は `mfsk_decoder_set_q65_callers` |
| `mfsk_callsign_hash_table_*`、`MfskCallsignHashTable*` 引数 | 廃止。デコーダがそれぞれ自分のテーブルを持ち、`mfsk_decoder_add_callsign` で種を入れる |
| `mfsk_mode_defaults`、`MfskDecodeDefaults` | 廃止。`mfsk_params_init` が既定値を書く |
| `mfsk_unpack77(session, …)` | `mfsk_unpack77(…)` と `mfsk_decoder_unpack77(dec, …)` |
| `mfsk_iq_add_channel(rx, dial, mode, &ch)` | `mfsk_iq_add_channel(rx, dial, mode, params, extras, &ch)`（従来の挙動なら NULL, NULL）。`mfsk_iq_channel_decoder`、`mfsk_iq_channel_state` は新規 |
| `mfsk_iq_set_time_anchor(rx, utc_ns_at_sample_0)` | `mfsk_iq_set_time(rx, utc_ns, at_sample, &change)`、繰り返し呼べる |
| `INVALID_ARG` で失敗する `mfsk_iq_retune(rx, hz)` | `mfsk_iq_retune(rx, hz, &paused, &resumed)` は収まらなくなったチャンネルを一時停止する |
| 「セッションが開く」意味の `MFSK_CAP_DECODE_HANDLE` | 「77ビットのスロットファミリ」。スロット系のモードはすべてデコーダが開く |
| FT8 の `previous_cycle` は「未公開」（#496） | `period` 引数つきの `MfskExtras::a7` |

今も大事な点: すべての構造体に `size` を設定する（または `_init` する）こと、
ブロックに触れる前に `mfsk_params_init` を呼ぶこと、デコーダ呼び出しの後は
グローバルではなく `mfsk_decoder_last_error` を読むこと。

---

## 4. Kotlin / Android

`bindings/kotlin/` は保守されているバインディングで、ソース変更のたびに
CI がデスクトップ JVM 上でビルド・実行している。pre-v2 ABI 向けに書かれ、
結果をパイプ区切り文字列で受け渡し、何にもビルドされていなかった旧
`mfsk-ffi/examples/kotlin_jni/` スキャフォルドを置き換えたもの。

```kotlin
import io.github.mfskcore.*

// 一覧をハードコードせず、ビルドに何があるか訊く。
val ft8 = Mfsk.modes().first { Mfsk.modeName(it) == "FT8" }

// Android では初回デコード前に一度呼ぶ — 下記参照。
Mfsk.configureRuntime(threads = 2)

MfskDecoder.open(ft8).use { dec ->
    for (r in dec.decode(pcm, period = periodIndex)) {
        Log.i("ft8", "${r.freqHz} Hz  ${r.snrDb} dB  ${r.text}")
    }
}
```

**構成。** `Mfsk` がイントロスペクションと送信を持ち、`MfskDecoder` が
全スロットモード共通のデコードハンドル（§2.1）で `AutoCloseable` なので
`.use { }` が解放する。`MfskDecode` は `data class` — ハンドルではなく値である。
ABI が呼び出し側所有のメモリに行を書くからで、解放すべきものも、デコーダより
長生きしうるものも無い。

**`Mfsk.configureRuntime` が Android 固有の部分。** これが無いと
デコードは rayon のグローバルプールで走り、そのスレッドは VM が一度も
アタッチしていない素の pthread なので、そこから `JNIEnv` に触れず、
ワーカースレッドからのデコードコールバックは非推奨どころか不正である。
シムのスレッドフックが `AttachCurrentThreadAsDaemon` と
`DetachCurrentThread` を呼ぶことでそれが合法になる。同時に、join されない
`num_cpus` × 2 MiB スタックからプールを外す役目も持つ。

**素の attach ではなく `AsDaemon` であることが要である。** 自前でシムを
書く消費者が最も間違えやすい点でもある — 非デーモンのアタッチ済み
スレッドは JVM を生かし続け、rayon のワーカーは join されないので、
通常の `AttachCurrentThread` では他が全部成功した後にプロセスが終了時に
ハングする。

**デコーダはシングルスレッド。** デコードのたびに変更するコールサイン
ハッシュテーブルを所有する。スレッドごとに1つ。別デコーダ同士の並行
デコードは支援されている。

**パラメータは2つの data class。** `MfskParams` がパラメータブロック（§2.3）、
`MfskExtras` がライブラリ独自のオプション（§2.3.1）である。
`Mfsk.defaultParams(mode)` から始めて、変えたい所だけ `copy` する。
`MfskParams` の帯域にコンストラクタの既定値は意図的に無い — ABI に init 呼び出しが
あるのと同じ理由で、0 埋めはモードの既定値と等価ではない（帯域が 0 だと何も
デコードしない）。`MfskExtras()` は `mfsk_extras_init` が書くもの、つまりすべての
オプションが未設定の状態:

```kotlin
val p = Mfsk.defaultParams(ft8).copy(
    rxFreqHz = 1500f,
    txFreqHz = 1500f,                       // FT8 の nftx。CAP_TX_FREQ が必要
    station = MfskStation("JL1NIE", "PM95"),
    qso = MfskQso("K1JT", "FN20", MfskQsoProgress.REPLYING),
)
val e = MfskExtras(
    a7 = true,                              // FT8 の a7。デコードごとに period が要る
    apHint = MfskApHint("K1JT", "HA0DU"),   // メッセージのフィールド順
)
MfskDecoder.open(ft8, p, e).use { dec -> dec.decode(pcm, period) }
```

`rxFreqHz`・`tolHz`・`txFreqHz` は nullable（C では NaN。0 Hz も周波数なので）。
`depth` は `MfskDepth`、`averaging` / `deepSearch` / `emeDelay` は `flags`、`ap` は
`MfskApMode`、`contest` は `MfskContest`。extras のうち `strategy` は
`MfskStrategy.SinglePass`・`.SicRounds(n)`・`.SicEarly`、`strictness` は
`MfskStrictness`、`noiseBlanker` は `MfskNoiseBlanker.Percent(n)` か
`.Sweep(step, toleranceHz)`（FST4）、`fading = MfskQ65Fading(b90Ts)` と `pileup`・
`maxDrift` は Q65 のものである。C 層と同じく、モードが持たないオプションは丸めずに
**extras を適用する時点で失敗する** — `MfskDecoder.open` か `setExtras` が、
フィールド名を含む `MfskUnsupportedException` を投げる。範囲外の値は
`MfskInvalidArgException`、ビルドに無いモードは `MfskUnknownModeException`。
`dec.setParams(…)` と `dec.setExtras(…)` は周期の間にブロックを変え、状態は保つ。
`dec.clear()` は状態を忘れ、`dec.addCallsign("JL1NIE")` はハッシュテーブルに種を
入れる。パラメータは構造体ごとに3本のフラット配列で JNI を渡り、並びは
`mfsk_jni.c` の `read_params` と `read_extras` に書いてある。JVM テストは各スロットを、
それを名指しする拒否メッセージで確かめている。

**デコード。** `dec.decode(pcm, period = …)` は `ShortArray` か `FloatArray`（任意の
レベル）を取り、`sampleRate` が 12 000 以外ならリサンプルされ、`period` は UTC
グリッドの番号または null である。`dec.copyInfo(i)` は行 `i` の FEC ビット。UTC
グリッドで切り出すストリームは、スロットをコピーして出し入れせずにデコードする:

```kotlin
MfskStream.open(ft8).use { s ->
    s.push(chunk)                                   // 任意のサイズ
    s.setTime(utcNs, atSample = s.position)         // 読み取りを得るたびに
    dec.decodeStream(s)?.let { r -> show(r.period, r.slotStartUtcNs, r.rows) }  // null: まだスロットが無い
}
```

`MfskStream.takeSlot()` は代わりにスロットをコピーして取り出し、`slotReady` と
`dropped` は C の呼び出しに対応し、`setTime` は `MfskClockChange`
（`FIRST`、`SLEWED`、`STEPPED`）を返す。

**Q65 も同じデコーダ。** Q65 のモードを開き、`MfskParams.emeDelay` / `averaging`
と `MfskExtras.pileup`・`maxDrift`・`fading` を設定する — `decodeQ65` はもう無い。
行は `copiedLastTx`（Pileup の `#`）を返し、`Mfsk.synthesizeQ65(…, copiedLastTx)` が
1 つ送る。`MfskQ65History`（`q65_hist`: `push`、`record(rows)`、
`lookup(rxFreqHz)`）と `MfskQ65Callers`（`q65_hist2`: `record(freqHz, text, now)`、
`expire(now)`、`remove(call)`、`callers`）は呼び出し側が所有する `AutoCloseable` の
ハンドルで、`dec.setQ65Callers(callers)` がコンテストリストをデコーダへ渡す。

**`dec.setBudget { … }`、`dec.lastBudget`** は budget（§2.2。デコーダを持つ全モード、
`lastBudget.rowsSubtracted` も含む）。述語は候補ごとに JNI を1往復するので、捕捉したデッドラインとの
`System.nanoTime()` 比較程度に留めること。それより重いものは JVM 側が既に
計算した boolean の裏に置く。

**`dec.decodePrefix(pcm, period)`** は早期デコード（§2.2、#572）で、音声が届くたびに
その周期でここまでの全部を渡して呼ぶ。FT8 は 141 696 サンプルでチェックポイント A の行を
`stage == MfskStage.EARLY` 付きで返し、周期全体で完全な集合を返す。ストリームでは
`stream.setPrefixPoints(dec.prefixPoints)` の後、用意されたスロットごとに
`dec.decodeStream(stream)` を呼ぶ（区別は `stream.slotIsWhole`）。IQ では既定で有効で、
`MfskIqDecode.stage` で早期の行が分かり、`rx.setEarly(ch, false)` で無効にできる（#601）。

**行**は `MfskDecode` である。`syncScore`、`syncCv`、`hardErrors` は nullable で、モードが
報告しないもの（C の行で `MFSK_DECODE_FLAG_HAS_*` ビットが落ちているもの）は null。`key` は
詰めたメッセージキーの16進文字列で `keyBits` と組、`delivery` は C の行のもので `-1` は null。
`MfskIqDecode` も同じ詳細を持つ。

**`dec.onDecode { row -> … }`** は `decode` が返すリストに加えて、見つかった順に
行を配信する — 長いスロットが終わる前に画面に何か出したい UI 向け
（`decode(…, onRow = …)` なら1回の呼び出しだけ）。リスナは rayon ワーカーから
呼ばれるので並行安全である必要があり、Android でビューに触れるものはメイン
ルーパへ post しなければならない。`configureRuntime` を先に呼ぶ必要は**無い** —
シムは VM が見たことの無いワーカーを自分で（デーモンとして）アタッチし、リスナの
メソッド ID をラムダの生成クラスではなく*インタフェース*から取る。リスナが投げた
例外は表示のうえクリアされ（rayon ワーカーには伝播先が無い）、デコードは続行する。
`dec.deliveryIsExact` は、それらの行が返される行とちょうど同じ順で一致するかを言う。どちらの
場合も、ストリームの行と返された形は `delivery` で対にする。

**IQ** は `MfskIqReceiver`（§2.8.2）: `MfskIqReceiver.open(sampleRate, centerHz,
format, iqSwap, channelizer)`、チャンネル ID を返す `addChannel(dialHz, mode, params,
extras)`、`push(bytes)`、`MfskIqDecode` を返す `poll()`、`setTime`、`retune`（停止した
チャンネル数と再開した数を返す）、`gap`、そして**借り物**の `MfskDecoder` を返す
`channelDecoder(ch)` — その `setParams`・`setExtras`・`addCallsign` が動作中の
チャンネルを再設定し、`onDecode` と `setBudget` は `push` が行うデコード（早期の行を含む）を
受け取り、上限を付ける（チャンネルごとに同じオブジェクトで、受信器はチャンネルを解放する前に
リスナーを外す）。`push` は UI スレッドの外で呼ぶこと。

**JTTY** は `MfskJttyReceiver`（§2.8.1）: `MfskJttyReceiver.open(sampleRate,
MfskJttyParams())` のあと `for (u in rx.push(chunk)) …` — `push` は生じた更新
（メッセージごとに 1 件、最新のテキスト。`id` は不変）を返し、`finish()` は
未完の最終行を返し、`close()` でハンドルを解放する。`push` は UI スレッドの外で
呼ぶこと。機能ビットは `Mfsk.CAP_STREAM_RECEIVER`。JVM テストは同梱の upstream
録音を 4096 サンプルのチャンクで（および 24 kHz のリサンプル経由でも）流し、
録音のメッセージが出ることを確認する。送信は `MfskJtty.tones(text, profile)`、
`MfskJtty.synthesize(tones)`、`MfskJtty.encode(text)`。

**シムが Rust + `jni` ではなく C なのは意図的。** 生成された `mfsk.h` を
`#include` するので、シムのビルド自体が「もう一つのコンパイラがそのヘッダを
本物の翻訳単位として読む」検査になる。これは既に元を取っている:
シムを書いたことで `MFSK_DECODE_FLAG_HASH_RESOLVED` がヘッダに無いことが
判明した（定数が、cbindgen が出力できない依存クレート側に居たため）。
Rust シムならクレートに直接リンクするので何も見えなかった。

ビルドとテスト: `bindings/kotlin/build.sh`（`JAVA_HOME` と `kotlinc` が要る）。
Android 向けには
`cargo ndk -t arm64-v8a build -p mfsk-ffi --release --no-default-features
--features mobile` で `libmfsk.so` をビルドし、同じヘッダに対して NDK の
clang でシムをコンパイルする。`.cargo/config.toml` には Android 15 端末が
要求する 16 KB ページサイズのリンクフラグが既に入っている。

---

## 5. Swift / Apple

`bindings/swift/` は同じ ABI の上の SwiftPM パッケージ — 写して使う
スキャフォルドではなく、依存先にするパッケージである。module map が
`mfsk-ffi/include/mfsk.h` を**その場で** include するので、コピーを持たず
ヘッダに追従する。

**単一デコーダ化の書き換え以降ビルドされていない。** パッケージは Swift
ツールチェーンの無い環境で `mfsk_decoder_*` に移された。ここにあるものは
Apple のハードウェア上でコンパイルも実行もされておらず、98件の XCTest
（`grep -rc 'func test' bindings/swift/Tests` で数えた件数）は書かれてはいるが
通っていない。以下のどの行も、Mac で `bindings/swift/scripts/test.sh` を走らせて
から頼ること。

```swift
import MfskCore

let slot = try Mode.ft8.synthesiseSlot(call1: "CQ", call2: "JL1NIE", report: "PM95",
                                       frequencyHz: 1500)
let decoder = try Decoder(mode: .ft8)
for row in try decoder.decode(slot) {
    print(row.frequencyHz, row.snrDB, row.text)
}
```

* `Mode` / `ModeInfo` / `Capabilities` がイントロスペクション族を包むので、
  ピッカーはハードコードした一覧ではなくビルドから埋まる。
* `Decoder` は全スロットモード共通のデコードハンドル（§2.1）で、WSPR、JT9、
  JT65、Q65 の10サブモードも含む: `Decoder(mode:params:extras:)`。`DecodeParams` が
  パラメータブロック（`try DecodeParams(mode: .ft8)` がモードの既定値を返し、
  そこから `bandHz`・`rxFrequencyHz`・`depth`・`ap`・`station`・`qso` …）、
  `Extras` がライブラリ独自のオプション（`strategy`・`apHint`・`a7`・
  `sniperHalfWidthHz`・`noiseBlanker`・`pileup`・`maxDrift`・`fading` …）である。
  モードが持たないオプションは `init` / `setExtras` で、名指しした `MfskError`
  （コード `.unsupported`）を throw する。`setParams`・`setExtras`・`clear()`・
  `addCallsign(_:)` は周期の間に作用し、`decode(_:sampleRate:period:handler:)` は
  `[Int16]` か `[Float]` と `period`（単発の録音なら nil）を取る。
* `CaptureStream` は UTC グリッドでスロットを切り出す取り込みの入れ物
  （`setTime(utcNanoseconds:)`、`position`、`isSlotReady`、`droppedSlots`、
  `takeSlot()`）で、`decoder.decode(stream)` がスロットを出し入れするコピーを
  避ける融合デコードである。スロットがまだ無ければ nil を返す。
* `decoder.setBudget { … }` は呼び出し側が時計を読む述語で探索を区切り
  （ライブラリは読まない）、`decoder.lastBudget` が打ち切りの残した仕事を
  — スキップした中で最良の候補の質と `rowsSubtracted` も含めて — 告げる。
  デコーダを持つ全モードが `Capabilities.budget` を持つ。
* `Decode.syncScore`、`syncCV`、`hardErrors` は Optional で、モードが報告しないものは nil。
  `key` / `keyBits` はメッセージキー、`delivery` は `onDecode` に渡った行と返された形を
  対にする（`decoder.deliveryIsExact` は両者が同じリストかを言う）。`IQDecode` も同じ詳細を持つ。
* `decoder.decodePrefix(pcm, period:)` は早期デコード（§2.2、#572）で、その周期で
  ここまでの全サンプルを渡す。FT8 は 141 696 サンプルでチェックポイント A の行
  （`stage == .early`）を、周期全体で完全な集合を返す。`CaptureStream` では
  `try stream.setPrefixPoints(decoder.prefixPoints)` の後、用意されたスロットごとに
  `decoder.decode(stream)` を呼ぶ（区別は `stream.isSlotWhole`）。IQ では既定で有効で、
  `IQDecode.stage` で分かり、`rx.setEarly(false, forChannel:)` で無効にできる（#601）。
* `decoder.onDecode { row in … }` は呼び出しが返す配列と並行して、
  見つかった順に行を流す。`desktop` ビルドではクロージャは rayon ワーカー上で
  （場合により並行に）走り、`mobile` では候補順に単一スレッドで走る。
  クロージャは差し替えかデコーダ解放まで保持され、ハンドルを閉じる前に
  クリアされる。
* 失敗は `MfskError` を throw する。ステータスコードと理由文字列の両方を
  持ち、ハンドル自身のエラースロットを先に、スレッドローカルのグローバルを
  後に読む。
* **Q65 も同じデコーダ。** Pileup、Max Drift、高速フェージングのメトリックは
  `Extras`（`pileup`・`maxDrift`・`fading = Extras.Fading(…)`）、EME 遅延と平均は
  `DecodeParams.emeDelay` / `averaging`、`Decode.copiedLastTx` は Pileup の `#`。
  `Q65` は送信側: `Q65.encode(subMode:…)`、`Q65.encode(…, copiedLastTx:)`、
  それに `Q65SubMode`（**独自の番号体系**で `a15` が 6、`.mode` で `Mode` へ橋渡し）
  と `Q65FadingModel`。`Q65History`（`q65_hist`）と `Q65Callers`（`q65_hist2`）は
  WSJT-X が持つ 2 つのリストで、呼び出し側が所有するクラス。コンテストリストは
  `decoder.setQ65Callers(_:)` で渡す。
* AP ヒント（`Extras.APHint`）のフィールドは**メッセージのフィールドをその順に**
  並べたもので — CQ なら `call1` は `"CQ"`、送信局ではない — 探索を誘導するので
  はなくメッセージのビットを固定するため、順序を誤ったヒントは誤ったヒントに
  なる。デコードは各候補をまず AP なしで試す（#555 以降、`jt9 -3` と同じ）ので、
  きれいな信号はどちらでもデコードされ、ヒントを必要とした弱い信号は失われる。
* `Message.text(resolvedBy: decoder)` はパックされたメッセージを、そのデコーダ自身
  のハッシュテーブルで展開する。
* **`IQReceiver`**（§2.8.2）: `IQReceiver(sampleRate:centerHz:format:iqSwap:channelizer:)`、
  `addChannel(dialHz:mode:params:extras:)`、`push(_:)`、`IQDecode` を返す `poll()` /
  `drain()`、`setTime(utcNanoseconds:atSample:)`、`retune`、`gap`、
  `state(ofChannel:)`、そして動作中のチャンネルを再設定できる**借り物**の `Decoder`
  を返す `decoder(forChannel:)`（これにデコードさせてはならない）。

**JTTY** は `JttyReceiver`（§2.8.1）: `try JttyReceiver(sampleRate:params:)` のあと
`try receiver.push(samples)` が生じた `[JttyUpdate]`（メッセージごとに 1 件、最新の
テキスト。`id` は不変、`isComplete`・`frequencyHz`・`startSeconds`）を返し、
`finish()` は未完の最終行を返す。`Mode.jtty` は `.decodeHandle` ではなく
`Capabilities.streamReceiver` を報告する。`JttyParams` は受信周波数・許容幅・
同期下限・帯域・減算の有無を持つ。一度に 1 スレッドで、メインアクターの外から
呼ぶこと — `push` は戻る前にデコードする。`JttyReceiverTests` は同梱の upstream
録音（`#filePath` で位置を求める）と自前のループバックを流す。`Jtty.tones(for:profile:)`
（テキストパッカー）、`Jtty.synthesise(_:)`、`Jtty.audio(for:)` がテキストを音声にする。

`bindings/swift/scripts/test.sh` が `libmfsk` をビルドしてテストを走らせる。
実アプリからのリンク（および iOS ビルドが `mobile` feature セットを選ぶべき理由）は
`bindings/swift/README.md` が扱う。
`bindings/swift/scripts/build-xcframework.sh` は iOS 実機向けとシミュレータ向けの
ビルドを `target/xcframework/Mfsk.xcframework`（モジュール名 `CMfsk`）にまとめ、
成功を報告する前に両スライスへのリンクを確認する。

CI は同じスクリプトを `macos-latest` 上で走らせ（`Swift binding (macOS) +
iOS build`）、そこが `aarch64-apple-ios` のクロスコンパイル場所でもある。
XCTest は Command Line Tools ではなく Xcode に同梱され、iOS SDK も Xcode の
ものなので、1つのランナーで両方を賄う。ローカルでは `xcode-select` が CLT を
指している場合、スクリプトが `DEVELOPER_DIR` を Xcode に向ける。
