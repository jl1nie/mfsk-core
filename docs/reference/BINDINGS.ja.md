# mfsk-core — C / Kotlin / Swift バインディング

> **English:** [BINDINGS.md](BINDINGS.md)

Rust 以外から mfsk-core を利用するための文書。Rust ホスト API は
[`LIBRARY.md`](LIBRARY.ja.md)、`no_std` / 組込ターゲットは
[`EMBEDDED.md`](EMBEDDED.ja.md) を参照。

| バインディング | 場所 | ビルド・テスト |
|---|---|---|
| **C / C++** | `mfsk-ffi/`、ヘッダ `mfsk-ffi/include/mfsk.h` | CI `ffi` ジョブ — 両 feature セットでの Rust テストに加え、実在の C++ ドライバ `examples/cpp_smoke/`（マルチスレッド負荷試験を含む） |
| **Kotlin / Android** | `bindings/kotlin/`（C シム + `Mfsk.kt`） | CI `kotlin` ジョブ、デスクトップ JVM 上 |
| **Swift / Apple** | `bindings/swift/`（SwiftPM パッケージ `MfskCore`） | CI `swift` ジョブ、`macos-latest` 上 — XCTest 75件と `aarch64-apple-ios` クロスビルド |

3つとも同一の C ABI の上に載っている。`mfsk.h` は cbindgen 生成でリポジトリに
コミットされており、そのドキュメントコメントがシンボル単位の正本である。
本書はその地図であって、置き換えではない。

## 目次

- [1. 生成物とリンク](#1-生成物とリンク)
- [2. C ABI](#2-c-abi)
  - [2.1 形: セッションと、呼び出し側が所有するメモリ](#21-形-セッションと呼び出し側が所有するメモリ)
  - [2.2 スロットをデコードする](#22-スロットをデコードする)
  - [2.3 `MfskDecodeParams` — 探索の指定](#23-mfskdecodeparams--探索の指定)
  - [2.4 `MfskDecode` — 結果1行](#24-mfskdecode--結果1行)
  - [2.5 ストリーミング取り込み](#25-ストリーミング取り込み)
  - [2.6 送信](#26-送信)
  - [2.7 イントロスペクション](#27-イントロスペクション)
  - [2.8 独自エントリポイントを持つモード](#28-独自エントリポイントを持つモード)
  - [2.9 メッセージ](#29-メッセージ)
  - [2.10 スレッドとランタイム](#210-スレッドとランタイム)
  - [2.11 エラーとメモリ規則](#211-エラーとメモリ規則)
  - [2.12 シンボル索引](#212-シンボル索引)
- [3. pre-v2 ABI からの移行](#3-pre-v2-abi-からの移行)
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
プレフィクスを置けるが、MSVC が要求する「戻り値型と関数名の間」には置けない。

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

### 2.1 形: セッションと、呼び出し側が所有するメモリ

ほぼ全体を2つの規則が覆う。

1. **セッションがデコードハンドルである。** `mfsk_session_open` → 設定 →
   1つ以上のスロットをデコード → `mfsk_session_close`。セッションは
   コールサインハッシュテーブルと、任意で前スロットの結果および FFT を
   所有する — いずれも複数回の呼び出しをまたいで初めて意味を持つもの。
2. **確保済みメモリは境界を越えない。** 結果行・合成音声・展開テキストは
   すべて呼び出し側がサイズを決めて所有するバッファに書かれる。解放すべき
   ポインタが存在しないので、呼び出しと解放の間で例外が巻き戻ったときに
   ラッパがリークする、というカテゴリ自体が消える。

ハンドルは `MfskDecodeSession*`、`MfskStream*`、
`MfskCallsignHashTable*` の3つだけで、それぞれに `_open`/`_new` と
`_close`/`_free` がある。互いに別の不完全型なので、取り違えは未定義動作では
なく C の型エラーになる。

### 2.2 スロットをデコードする

最小の流れ。これは `mfsk-ffi/examples/cpp_smoke/main.cpp` が CI で実際に
走らせている形である:

```c
#include "mfsk.h"

MfskStatus st = MFSK_STATUS_INTERNAL;
MfskDecodeSession* s = mfsk_session_open(MFSK_MODE_FT8, NULL, &st);
if (s == NULL) { /* 理由は mfsk_last_error() */ }

MfskDecode rows[16];
size_t n = 0;
if (mfsk_session_decode_i16(s, pcm, n_pcm, 12000, NULL,
                            rows, 16, &n) == MFSK_STATUS_OK) {
    for (size_t i = 0; i < n; ++i) {
        printf("%.1f Hz  %.0f dB  %s\n",
               rows[i].freq_hz, rows[i].snr_db, rows[i].text);
    }
}
mfsk_session_close(s);
```

params に `NULL` を渡すとそのモードの既定値が使われる。`out_cap` は
受け取る意思のある行数の上限、`out_len` に実際に書かれた行数が返る。

```c
MfskDecodeSession *mfsk_session_open(uint32_t mode, const MfskDecodeParams *params,
                                     MfskStatus *out_status);
void               mfsk_session_close(MfskDecodeSession *s);

MfskStatus mfsk_session_decode_i16(MfskDecodeSession *s, const int16_t *samples,
                                   size_t n_samples, uint32_t sample_rate,
                                   const MfskDecodeParams *params,
                                   MfskDecode *out, size_t out_cap, size_t *out_len);
MfskStatus mfsk_session_decode_f32(MfskDecodeSession *s, const float *samples, ...);
```

セッション単位の戦略設定。一度設定すれば以降の全デコードに効く:

| 呼び出し | 効果 |
|---|---|
| `mfsk_session_set_on_decode(s, cb, user)` | 見つかった順に行を配信する（戻り値の配列に加えて） |
| `mfsk_session_set_budget(s, check, user)` | 呼び出し側の述語をポーリングし、`false` で打ち切る |
| `mfsk_session_last_budget(s, &report)` | 打ち切りが何を残したか — スキップした候補数、実行したステージ数、スキップした中で最良の候補の質 |
| `mfsk_session_keep_known(s, true)` | 今回の結果を既知信号として次回へ持ち越す |
| `mfsk_session_known_count(s)` | 現在持ち越している数 |
| `mfsk_session_keep_fft_cache(s, true)` | 同じ音声への2回目のパスでスロット変換を再利用 |
| `mfsk_session_add_callsign(s, "JL1NIE")` | `<...>` 参照が解決できるようハッシュテーブルへ投入 |
| `mfsk_session_copy_info(s, i, out, cap, &len)` | 行 `i` の背後にある FEC 情報ビット |

budget 述語は候補ごとにポーリングされ、**ライブラリ側は一切時計を読まない** —
期限は述語が何と比較するか次第である。これが wasm から、また
スロット途中でバックグラウンドに回された端末から使える理由になっている。

### 2.3 `MfskDecodeParams` — 探索の指定

構造体をゼロ埋めし `size` を設定したうえで、まずモードの既定値を
ライブラリに埋めさせ、それから必要な項目だけ上書きする:

```c
MfskDecodeParams p;
memset(&p, 0, sizeof p);
p.size = sizeof p;
mfsk_decode_params_init(MFSK_MODE_FT8, &p);
p.freq_max_hz = 2600.0f;
```

| フィールド | 意味 |
|---|---|
| `freq_min_hz` / `freq_max_hz` | 探索帯域の両端 |
| `sync_min` | sync 閾値 — **モード間で比較不能**。`MfskDecodeDefaults::sync_scale` を見よ |
| `max_cand` | 候補数の上限 |
| `depth` | `MfskDecodeDepth` — コスト／再現率の段 |
| `strictness` | `MfskStrictness` — 採否閾値のプロファイル |
| `eq_mode` | `MfskEqMode`。**入力音声の性質**であって探索の性質ではない（アナログフィルタが傾けた通過帯域を平坦化する） |
| `freq_hint_hz` | この周波数付近の候補を優先。`NaN`（`_init` が書く値）は未設定。a-priori パスの QSO 周波数でもある: 両方の呼出符号を固定する AP ヒント（`ap_call1` と `ap_call2`）は、この周波数の 50 Hz 以内にだけ試し、未設定なら一切試さない。送信周波数は下の `tx_freq_hz` |
| `sic_rounds` | 逐次干渉除去の回数、0 で無効。`MFSK_CAP_SIC_ROUNDS` が必要 |
| `sic_early` | チェックポイント模倣の早期デコード。`MFSK_CAP_SIC_EARLY` が必要 |
| `has_ap_hint`, `ap_call1`, `ap_call2`, `ap_grid` | 事前情報ヒント。`MFSK_CAP_AP_WIDEBAND`（狭帯域呼び出しでは `_AP_NARROW`）が必要 |
| `search_hz` | 狭帯域探索の半値幅。0 でモード既定。`MFSK_CAP_SNIPER` のときのみ意味を持つ |
| `tx_freq_hz` | 運用者の送信周波数（WSJT-X の `nftx`）。`NaN`（`_init` が書く値）は未設定。FT8 は、両方の呼出符号を固定する AP 仮説を `freq_hint_hz` の 50 Hz 以内に加えて、この周波数の 50 Hz 以内でも試す（`ft8b.f90`）。`MFSK_CAP_TX_FREQ` が必要で、**FT8 のみ**。狭帯域呼び出し（`search_hz`）では読まれないので併用不可。他のモードは `mfsk_session_open` で拒否する |
| `nb_percent` | インパルスノイズブランカ: スロット全体の変換の前に、大きい方から `n` パーセントのサンプルを消す（`0..=25`、WSJT-X の **NB** 設定）。既定の 0 は何も消さない。25 を超える値は丸めずに拒否する。0 以外のとき `MFSK_CAP_NOISE_BLANKER` が必要 |
| `nb_sweep_step` | 0 以外なら、ブランキングを 1 段でなく `0, step, 2*step, .. 20` パーセントの各段で 1 回ずつデコードする（5・2・1。GUI が出すのは 5 と 2）。`nb_percent` より優先。0 より大きい各段は `freq_hint_hz` の `nb_ftol_hz` 以内だけを探すので、**ヒントが無ければ 0 % の段しか走らない**。最大 21 回のデコード。`MFSK_CAP_NOISE_BLANKER` が必要 |
| `nb_ftol_hz` | スイープの探索窓の半値幅（Hz、WSJT-X の F Tol）。`nb_sweep_step` のときだけ読まれ、その場合は正の値が必要。引き継ぐ既定値が無いので `_init` は 0 を書く |

`MfskDecodeParams` は公開後も伸びてきた。`tx_freq_hz` と 3 つの `nb_*` が最新で、
`search_hz` の後ろに追加している。古いヘッダでビルドした呼び出し側は短い `size` を
渡し、ライブラリはその前半だけを読み、残りは既定値（未設定 / オフ）のままにする。

**AP フィールドはメッセージのフィールドをその順に並べたもの**である —
CQ の場合 `ap_call1` は `"CQ"` であって送信局ではない。これらは探索を
誘導するのではなくメッセージのビットを固定するので、順序を誤ると
0.1 dB 単位の損ではなくデコードそのものが消える。

### 2.4 `MfskDecode` — 結果1行

平坦・固定長で、呼び出し側の配列に書かれる。`text` はインラインの
`char[MFSK_DECODE_TEXT_LEN]`、NUL 終端。

| フィールド | 意味 |
|---|---|
| `size` | 呼び出し側が理解している `sizeof(MfskDecode)` |
| `mode` | **具体的なサブモード**であってファミリではない。FST4 の5周期はそれぞれ別に報告される |
| `text` | 復号メッセージ |
| `freq_hz`, `dt_sec`, `snr_db` | キャリア、スロットの `dt = 0` からの時間オフセット、2500 Hz 基準帯域での SNR |
| `sync_score` | この復号の sync 相関 |
| `sync_cv` | ブロックごとの sync 電力の変動係数 — 安定なチャネルでは 0 近傍、QSB で上昇。この行が持つ唯一のフェージング指標 |
| `hard_errors` | FEC が訂正した硬判定誤り数 |
| `info_bits` | FEC 情報ブロック幅、91（CRC-14）または 101（CRC-24） |
| `pass` | どのパスがこの行を生んだか。**プロトコル私的** — 診断用であってロジック用ではない |
| `flags` | bit 0 = `MFSK_DECODE_FLAG_HASH_RESOLVED`、`<...>` 参照の解決にハッシュテーブルを要した |

### 2.5 ストリーミング取り込み

音声を押し込む1スロット分のリング。モード自身の `slot_samples_12k` から
サイズが決まるので、FST4-300 の 360万サンプルのスロットも FT4 の
9万サンプルと同じ扱いになる。

```c
MfskStream *mfsk_stream_open(uint32_t mode, uint32_t sample_rate, MfskStatus *out);
MfskStatus  mfsk_stream_push_i16(MfskStream *s, const int16_t *samples, size_t n);
MfskStatus  mfsk_stream_push_f32(MfskStream *s, const float *samples, size_t n);
void        mfsk_stream_set_epoch(MfskStream *s, double utc_seconds_of_next_sample);
bool        mfsk_stream_slot_ready(const MfskStream *s);
size_t      mfsk_stream_buffered(const MfskStream *s);
size_t      mfsk_stream_take_slot_i16(MfskStream *s, int16_t *out, size_t cap,
                                      double *out_slot_start_utc);
void        mfsk_stream_clear(MfskStream *s);
void        mfsk_stream_close(MfskStream *s);

/* 融合版: リングから直接デコードする */
MfskStatus  mfsk_session_decode_stream(MfskDecodeSession *s, MfskStream *stream,
                                       const MfskDecodeParams *params,
                                       MfskDecode *out, size_t out_cap, size_t *out_len,
                                       double *out_slot_start_utc);
```

**`Instant` も `SystemTime` も、いかなる時計も使わない。** 次のサンプルが
どの UTC 秒に属するかをホストが告げ、グリッドは算術をするだけである。
epoch 未設定ならグリッドは最初のサンプルから自走する — 録音を再生する場合は
それが正しい。

take してから decode するより `mfsk_session_decode_stream` を使うこと。
FST4-300 のスロットを取り出して渡し直すのは 7 MB を無駄に動かすだけである。

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
MfskStatus  mfsk_mode_defaults(uint32_t mode, MfskDecodeDefaults *out);
uint32_t    mfsk_abi_version(void);
uint32_t    mfsk_version(void);
```

**`MfskMode` は全モードを指し、その discriminant は ABI である。**
レジストリ項目ごとに1つ＋ MSK144 と JTTY で、一度割り当てたら並べ替えない —
レジストリの登録は feature で変わるため、`q65` 無しのビルドではそれ以降の
インデックスが全部ずれる。ゆえに意図的にレジストリのインデックスではない。
このビルドに実際どれがあるかは `mfsk_mode_count` / `mfsk_mode_at` が答える。

**ケーパビリティは推測ではなく公開される。** 要となるのは
`MFSK_CAP_DECODE_HANDLE` で、`mfsk_session_decode_i16` 系がそもそも
適用できるかを示す。Q65 は公称開始サンプルと時間許容量を取り、
WSPR / JT9 / JT65 にはビルダが無い。劣っているのではなく形が違うのであり、
それを呼び出し側が「知っていなければならない事実」ではなく
「読めるビット」にしたのがこの surface である。

| bit | 定数 | 意味 |
|---|---|---|
| 0 | `MFSK_CAP_DECODE_HANDLE` | セッションのデコード呼び出しが適用できる |
| 1 | `MFSK_CAP_SNIPER` | 狭帯域単一目標探索。**設計上 FT8 のみ** |
| 2 | `MFSK_CAP_AP_NARROW` | 目標指定探索での AP ヒント |
| 3 | `MFSK_CAP_AP_WIDEBAND` | 広帯域探索での AP ヒント |
| 4 | `MFSK_CAP_SIC_ROUNDS` | 平坦な逐次干渉除去 |
| 5 | `MFSK_CAP_SIC_EARLY` | チェックポイント模倣の早期デコード。FT8 のみ |
| 6 | `MFSK_CAP_OSD` | OSD の*スイッチ*が効く。無い場合は「切れない」であって「無い」ではない |
| 7 | `MFSK_CAP_EQ_MODE` | イコライズがデコーダまで届く |
| 8 | `MFSK_CAP_STRICTNESS` | strictness プロファイルが受理して捨てられるのではなく効く |
| 9 | `MFSK_CAP_BUDGET` | 呼び出し側の budget 述語がポーリングされる |
| 10–15 | `MFSK_CAP_KNOWN_FILTER` … `MFSK_CAP_STREAM_RECEIVER` | `mfsk.h` を参照 |
| 16 | `MFSK_CAP_NOISE_BLANKER` | WSJT-X のインパルスノイズブランカ（`nb_percent`、`nb_sweep_step`）。**FST4 の全サブモードのみ** |
| 17 | `MFSK_CAP_TX_FREQ` | 送信周波数（`tx_freq_hz`）が a-priori 探索に効く。**FT8 のみ** |

これらのビットは `mfsk_core::registry::caps` を写したもので、
`mfsk-core/tests/registry_caps.rs` が双方向にトレイト実装と結び付けている —
トレイトを持たないプロトコルを名指しすればそこで*コンパイル*エラーになり、
実装してビットを立て忘れれば実行時に失敗する。最後の環は
`mfsk-ffi/tests/mode_introspection.rs` が各 `MFSK_CAP_*` を対応する
レジストリ定数と突き合わせて閉じている。手書きのケーパビリティ表は
2リリース以内に嘘になるからである。

**`mfsk_mode_defaults` は この ABI 最悪の罠を取り除く。** 既定値はデータであり、
`MfskDecodeDefaults::sync_scale` が「2つのモードの数値がそもそも比較可能か」を
告げる:

```c
MfskDecodeDefaults d = {0};
d.size = sizeof d;
mfsk_mode_defaults(MFSK_MODE_FT4, &d);
/* d.sync_min == 1.18, d.sync_scale == MFSK_SYNC_SCALE_BASELINE_NORMALISED */
```

FT4 はスコアリング前にスペクトルをフィット済みベースラインで割るため、
雑音は**構成上** 1.0 付近に来る。WSJT-X 自身の 1.18
（`ft4_decode.f90:195`）は好みではなく下限である。FT8 と FST4 のそれは
絶対的な Costas スコア。WSPR・JT9・JT65・Q65 は sync を「sync と雑音の和に
対する sync の割合」で測り（`MFSK_SYNC_SCALE_SYNC_FRACTION`）、0‥1 の値で
共通の既定値は 0.1。モードをまたいでコピーするのは誤りで、
このフィールドが出来るまでそれを告げるものが無かった。

#413 以降、これは探索を持つ全モードについてライブラリ自身の既定値であり、
`MFSK_CAP_DECODE_HANDLE` を持たないモードも含む。そのため JT9 と JT65 は、
C の入口が既知キャリアでの点デコードであるにもかかわらず Rust 側の走査帯域を
公開し、Q65 は `mfsk_q65_*` 系が実際に行う広い走査より狭いライブラリ既定値を
公開する。

**`MfskModeInfo::decode_fft1_size` は見積り前に読むべきフィールド。**
デコーダがスロット全体に対して取る前方 FFT のサイズで、FT4 は 92 160 点、
FST4-300 は **4 194 304** 点 — 他のどのフィールドも示唆しない 45 倍差であり、
「全モードで呼び出しの形は1つ」がメモリの話としては誤りである理由でもある。

**サイズバージョニング。** `MfskModeInfo`、`MfskDecodeDefaults`、
`MfskDecodeParams`、`MfskDecode` はいずれも先頭が `size`。自分の `sizeof` を
設定する（あるいは構造体をゼロ埋めすればライブラリが埋める）。ヘッダより
新しいライブラリは、呼び出し側が宣言した前半部分だけを書き、`size` を
実際に書いた量へ書き換える。

`mfsk_abi_version()` を `mfsk_version()` と分けてあるのは意図的である。
クレートのバージョンは境界と無関係な理由で動くからである。

### 2.8 独自エントリポイントを持つモード

`MFSK_CAP_DECODE_HANDLE` を持たないモードは直接呼ぶ:

```c
MfskStatus mfsk_wspr_decode(const int16_t *samples, size_t n, uint32_t rate,
                            MfskDecode *out, size_t cap, size_t *out_len);
MfskStatus mfsk_jt9_decode_at (const int16_t *samples, size_t n, uint32_t rate,
                               float freq_hz, MfskDecode *out, size_t cap, size_t *out_len);
MfskStatus mfsk_jt65_decode_at(/* jt9 と同形 */);
```

Q65 は「何を手掛かりとして与えるか」で分かれる4つの族を持つ。いずれも
`submode` と任意の `MfskCallsignHashTable*` を取る:

| 呼び出し | 追加で取るもの |
|---|---|
| `mfsk_q65_decode` | — |
| `mfsk_q65_decode_with_ap` | `ap_call1`, `ap_call2`, `ap_grid`, `ap_report` |
| `mfsk_q65_decode_fading` | `b90_ts`, `fading_model`（`MfskQ65FadingModel`） |
| `mfsk_q65_decode_with_ap_list` | `my_call`, `his_call`, `his_grid` — QSO 状態の仮説リスト |

`MfskQ65SubMode` は**独自の番号体系**を持ち、`a15` は 6 である。
一致を仮定せず `MfskMode` へ橋渡しすること。

ハッシュテーブルはセッションではなく呼び出し側が所有する唯一のハンドル:
`mfsk_callsign_hash_table_new` / `_insert` / `_free`。

**WSJT-X 3.2 の Q65 設定は `mfsk_q65_decode_ex` 1 本で扱う。** 上の 4 つは
戦略を名前で選び、決め打ちの広い窓を走査する。Pileup・Max Drift・EME delay・q3
リストデコードはそれらの組み合わせなので、組み合わせごとに位置引数の関数を
増やすのではなく、1 つの呼び出しとサイズバージョン付きの構造体にした:

```c
MfskQ65Params p;
memset(&p, 0, sizeof p);
p.size = sizeof p;
mfsk_q65_params_init(MFSK_MODE_Q65A30, &p);   // ライブラリの既定値
p.max_drift = 10;
MfskStatus st = mfsk_q65_decode_ex(MFSK_MODE_Q65A30, pcm, n, 12000, &p,
                                   /*callers*/ NULL, /*hash_table*/ NULL,
                                   rows, cap, &n_rows);
```

`MfskQ65SubMode` ではなく **`MfskMode`**（Q65 のもの）を取り、`dt_sec` は WSJT-X の
DT 列と同じく `nominal_start_s` からの値で返す（従来の関数は
`start_sample / 12000` を返す）。`mfsk_q65_params_init` が書くのはライブラリ自身の
既定値（200–3000 Hz、±1 s、しきい値 0.1、候補 8）で、従来の関数の広い窓ではない。

| フィールド | 意味 |
|---|---|
| `nominal_start_s` | バッファ内で `dt = 0` の位置: モードの `tx_start_offset_s`（0.5 s、Q65-120 以降は 1.0 s）。スロット境界から始まるバッファならこれが正しい。Max Drift が正規化する周期と q3 の slot start もこれで決まるので、便宜ではなく正しい値が必要 |
| `t_early_s`, `t_late_s` | フレーム開始がそこからどれだけ前後にずれてよいか |
| `pileup` | **Q65 Pileup**: 両方の呼出符号だけを名指しする AP ヒントは予備の 78 ビット目を空けるので、「copied last Tx」フラグ付きの返信も一致する。`has_ap_hint` が必要 |
| `eme_delay` | **EME delay**（「Decode at 52 s」）: 遅い側の端が +5.5 s（Q65-15 は +4.0 s）まで届く |
| `max_drift` | **Max Drift**、スペクトルビン `0..=50`: フレーム全体での線形なトーンドリフトを探索して除く。通常探索の `2*bins+1` 倍のコスト。upstream と同様に帯域を `nfqso ± ntol` に絞ること |
| `rx_freq_hz`, `ftol_hz` | q3 デコードが見る Rx 周波数（NaN で未設定）と F Tol（既定 10 Hz）。`ap_list` が必要 |
| `ap_list` | 0 なし。1 は `list_my_call` / `list_his_call` / `list_his_grid` の標準 QSO リスト。2 は `list_my_call` と、呼び出しに渡す `MfskQ65Callers*` による**コンテストリスト**。`rx_freq_hz` があれば WSJT-X の **q3** デコードで Rx 周波数で最初に走り、無ければ粗い候補ごとのテンプレート照合 |
| `fading_b90_ts`, `fading_model` | 高速フェージング用メトリック（NaN、既定は通常の AWGN）。0 は Gaussian、1 は Lorentzian |
| `has_ap_hint`, `ap_call1`, `ap_call2`, `ap_grid`, `ap_report` | a-priori ヒント。メッセージのフィールド順 |

フラグはすべて `uint32_t`、省略可能な float は不在なら NaN なので、C からの異常値は
Rust の不正な `bool` ではなく拒否になる。**エンジンが黙って無視する組み合わせは
拒否**し、理由を `mfsk_last_error()` に入れる: `ap_list` と `fading_b90_ts`（両者を
組み合わせる WSJT-X の経路は無い）、`ap_list` の無い `rx_freq_hz`、AP ヒントの無い
`pileup`、フェージング併用や Rx 周波数の無いリストデコードでの `max_drift`。Pileup の
返信は `MFSK_DECODE_FLAG_COPIED_LAST_TX`（`MfskDecode::flags` のビット 1、WSJT-X では
`#`）付きで返り、従来の Q65 呼び出しも立てる。送る側は `mfsk_encode_q65_flagged` が
`copied_last_tx` 付きの `mfsk_encode_q65`。

**WSJT-X が持つ 2 つのリストは、呼び出し側が所有するハンドル**にした。デコーダは
状態を持たず、時計はアプリケーションのものだけだから。`MfskQ65History`
（`q65_hist`、直近 100 件のデコード。`_push`、行配列をまとめて入れる `_record`、
DX 局が未入力のときの "Decode Again" が読む DX 呼出符号とグリッドを返す
`_lookup(rx_freq, &dx)`）と `MfskQ65Callers`（`q65_hist2`、グリッド付きで呼んできた
最大 50 局。`_record(freq, text, now)`、`_expire(now)`、`_remove(call)`、`_len`、
`_get`）。時刻は呼び出し側が渡す Unix 秒で、ライブラリは時計を読まない。どちらも
スレッドセーフではない。

### 2.8.1 JTTY — スロット呼び出しではなく受信器ハンドル

JTTY（WSJT-X 3.2 のキーボードモード）にはスロットが無い。フレームは送信側が
好きなときに始まり、メッセージは複数フレームからなるので、受信器が状態を持ち、
出力はメッセージの*更新*になる。モードは `MFSK_MODE_JTTY`、
`MFSK_CAP_STREAM_RECEIVER` を公開し（`MFSK_CAP_DECODE_HANDLE` は持たない）、
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

**送信**は他のモードと同じ 3 段で、77 ビットメッセージの代わりにテキストが入る:

```c
size_t n = 0;
mfsk_jtty_encode_tones("CQ K1ABC CQ", /*profile*/ 0, NULL, 0, &n);   /* サイズ問い合わせ: n = 59 */
uint8_t tones[16 * 59];
mfsk_jtty_encode_tones("CQ K1ABC CQ", 0, tones, sizeof tones, &n);   /* upstream の pack_jtty + genjtty */
int16_t pcm[16 * 59 * 384 + 4096];  size_t m;                        /* 必要量は mfsk_jtty_synth_len(n) */
mfsk_jtty_tones_to_i16(tones, n, 1500.0f, 8000.0f, pcm, sizeof pcm / 2, &m);
```

`profile` は 0 = 不明、1 = Field Day、2 = RTTY Roundup。整形が変わるのは RTTY Roundup
だけ（シリアル番号と州の候補、`599 5` → `599 005`）。パッカーはフレーム数が最小に
なるように選ぶ: コールサイン・グリッド・レポート・制御フレーズは 1 フレーム、
その他のテキストは 5 文字で 1 フレーム。80 文字超、16 フレーム超、RTTY のシリアルが
収まらない場合は `MFSK_STATUS_INVALID_ARG`（理由は `mfsk_last_error`）。空メッセージは
`OK` で `*out_len = 0`。WSJT-X が `pack_jtty` の外側に持つ F キーテンプレートと N1MM
タグはホスト側の方針であり、ライブラリには無い（線引きは #463）。

### 2.9 メッセージ

```c
MfskStatus mfsk_pack77(const char *call1, const char *call2, const char *report,
                       uint8_t *out_message77);
MfskStatus mfsk_pack77_type1(const char *call1, const char *call2, const char *grid,
                             uint8_t *out_message77);
MfskStatus mfsk_pack77_type4(const char *nonstd_call, const char *std_call,
                             const char *report, bool is_cq, uint8_t *out_message77);
MfskStatus mfsk_pack77_free_text(const char *text, uint8_t *out_message77);
MfskStatus mfsk_unpack77(const MfskDecodeSession *session, const uint8_t *message77,
                         char *out, size_t cap, size_t *out_len);
```

`out_message77` はいずれの場合も呼び出し側所有の 77 バイトバッファで、
どれも確保を行わない。`mfsk_pack77_free_text` は名前に反して何も解放せず、
**13文字までのフリーテキストを pack する**関数である。`mfsk_unpack77` が
セッションを取るのは `<...>` ハッシュ参照をそのテーブルで解決するためで、
持っていなければ `NULL` を渡す。

### 2.10 スレッドとランタイム

```c
MfskStatus mfsk_runtime_configure(const MfskRuntimeConfig *cfg);
uint32_t   mfsk_runtime_thread_count(void);
```

* **セッションはシングルスレッドである。** デコードのたびに自分のハッシュ
  テーブルを変更する。同時実行するスレッドごとに1つ持つこと。別セッション
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

1. **ハンドル**: `mfsk_session_open` / `mfsk_session_close`、
   `mfsk_stream_open` / `mfsk_stream_close`、
   `mfsk_callsign_hash_table_new` / `_free`。close と free は `NULL` に対して
   冪等である。
2. **結果行・音声・テキスト**は呼び出し側所有のバッファへ入る。返り値で
   解放が必要なものは無い。`const char*` を返す2つ
   （`mfsk_last_error`、`mfsk_session_last_error`）は借用ポインタであって
   確保ではなく、`mfsk_mode_name` の文字列は static である。
3. **エラー**: `MFSK_STATUS_OK` 以外が返ったら、セッション呼び出しなら
   `mfsk_session_last_error(s)`、自由関数なら `mfsk_last_error()` を
   **同じスレッドで**呼ぶ。返るポインタはそのスレッドで次に失敗しうる
   呼び出しを行うまで有効。

`MfskStatus`: `OK = 0`、`NULL_POINTER = -1`、`INVALID_ARG = -2`、
`UNKNOWN_PROTOCOL = -3`（このビルドに無い）、`DECODE_FAILED = -4`、
`INTERNAL = -5`（常にバグ）、`UNSUPPORTED = -6`（モードは在るが要求された
ものを提供しない）。

### 2.12 シンボル索引

エクスポートされる関数は 77 個:

| 群 | シンボル |
|---|---|
| session (14) | `mfsk_session_open` `mfsk_session_close` `mfsk_session_decode_i16` `mfsk_session_decode_f32` `mfsk_session_decode_stream` `mfsk_session_set_on_decode` `mfsk_session_set_budget` `mfsk_session_last_budget` `mfsk_session_keep_known` `mfsk_session_known_count` `mfsk_session_keep_fft_cache` `mfsk_session_add_callsign` `mfsk_session_copy_info` `mfsk_session_last_error` |
| streaming (9) | `mfsk_stream_open` `mfsk_stream_close` `mfsk_stream_push_i16` `mfsk_stream_push_f32` `mfsk_stream_buffered` `mfsk_stream_set_epoch` `mfsk_stream_slot_ready` `mfsk_stream_take_slot_i16` `mfsk_stream_clear` |
| introspection (10) | `mfsk_mode_count` `mfsk_mode_at` `mfsk_mode_name` `mfsk_mode_from_name` `mfsk_mode_info` `mfsk_mode_caps` `mfsk_mode_defaults` `mfsk_decode_params_init` `mfsk_abi_version` `mfsk_version` |
| 専用デコード (7) | `mfsk_wspr_decode` `mfsk_jt9_decode_at` `mfsk_jt65_decode_at` `mfsk_q65_decode` `mfsk_q65_decode_with_ap` `mfsk_q65_decode_fading` `mfsk_q65_decode_with_ap_list` |
| 送信 (12) | `mfsk_encode_ft8` `mfsk_encode_ft4` `mfsk_encode_fst4s60` `mfsk_encode_wspr` `mfsk_encode_jt9` `mfsk_encode_jt65` `mfsk_encode_q65` `mfsk_message_to_tones` `mfsk_tones_to_i16` `mfsk_tones_to_f32` `mfsk_symbol_count` `mfsk_synth_output_len` |
| メッセージ (5) | `mfsk_pack77` `mfsk_pack77_type1` `mfsk_pack77_type4` `mfsk_pack77_free_text` `mfsk_unpack77` |
| JTTY (14) | `mfsk_jtty_params_init` `mfsk_jtty_open` `mfsk_jtty_close` `mfsk_jtty_set_params` `mfsk_jtty_push_i16` `mfsk_jtty_push_f32` `mfsk_jtty_finish` `mfsk_jtty_reset` `mfsk_jtty_pending` `mfsk_jtty_poll` `mfsk_jtty_encode_tones` `mfsk_jtty_synth_len` `mfsk_jtty_tones_to_i16` `mfsk_jtty_tones_to_f32` |
| ハッシュテーブル (3) | `mfsk_callsign_hash_table_new` `mfsk_callsign_hash_table_insert` `mfsk_callsign_hash_table_free` |
| ランタイム (3) | `mfsk_runtime_configure` `mfsk_runtime_thread_count` `mfsk_last_error` |

---

## 3. pre-v2 ABI からの移行

0.11.0 で C 側のデコード surface は丸ごと置き換わった。`mfsk-ffi` は
`publish = false` で、実際に動いていた消費者はリポジトリ内の C++ ドライバ
だけなので影響範囲は差分の見た目より小さい — ただし C の消費者は
調整ではなく書き直しになる。

| pre-v2 | v2 |
|---|---|
| `MfskProtocol` enum | `MfskMode` — レジストリ項目ごとに discriminant があり、FST4 の5サブモード全てを指せる。`mfsk_mode_count` / `mfsk_mode_at` で列挙 |
| `mfsk_decoder_new` / `_free` | `mfsk_session_open` / `mfsk_session_close` |
| `MfskDecodeOptions*` と8つの `mfsk_decode_options_set_*` | `MfskDecodeParams`、ただの size 付き構造体。`mfsk_decode_params_init` で初期化 |
| `MfskResultList` と `mfsk_result_list_free` | `MfskDecode out[]`、呼び出し側所有の配列。解放不要 |
| `MfskSamples` と `mfsk_samples_free` | `mfsk_symbol_count` / `mfsk_synth_output_len` でサイズを決めた呼び出し側バッファ |
| `mfsk_decode_{i16,f32}_sniper` | 通常のデコードで `MfskDecodeParams::search_hz` を使う。`MFSK_CAP_SNIPER` があるモード（FT8 のみ） |
| ヒープを返す7つの `mfsk_encode_*` | 同名のまま、`out` / `cap` / `out_len` に書く形へ |
| 何でも `mfsk_last_error()` | セッション呼び出しは `mfsk_session_last_error(s)`。自由関数は引き続き `mfsk_last_error()` |
| モード幾何のハードコード | `mfsk_mode_info` / `mfsk_mode_caps` / `mfsk_mode_defaults` |
| `mfsk-ffi-ft8`（FT8 専用の組込クレート） | 退役。`mfsk-ffi` を使うか、Rust staticlib シムから `mfsk-core` を呼ぶ — [`EMBEDDED.md`](EMBEDDED.ja.md) 参照 |

`MfskDecodeSession` を pre-v2 の `MfskDecoder` と**別の型**にしたのは意図的で
ある。2つは別の Rust 値を所有しており、「どの関数に渡すかで意味が変わる
ハンドル」こそがこの再設計の終わらせようとした失敗様式だからである。

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

MfskSession.open(ft8).use { s ->
    for (r in s.decode(pcm, sampleRate = 12_000)) {
        Log.i("ft8", "${r.freqHz} Hz  ${r.snrDb} dB  ${r.text}")
    }
}
```

**構成。** `Mfsk` がイントロスペクションと送信を持ち、`MfskSession` が
デコードハンドルで `AutoCloseable` なので `.use { }` が解放する。
`MfskDecode` は `data class` — ハンドルではなく値である。ABI が
呼び出し側所有のメモリに行を書くからで、解放すべきものも、セッションより
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

**セッションはシングルスレッド。** デコードのたびに変更するコールサイン
ハッシュテーブルを所有する。スレッドごとに1つ。別セッション同士の並行
デコードは支援されている。

**デコードのパラメータは data class の `MfskDecodeParams`。**
`Mfsk.defaultParams(mode)` から始めて、変えたい所だけ `copy` する。
コンストラクタの既定値は意図的に無い — ABI に init 呼び出しがあるのと同じ
理由で、0 埋めはモードの既定値と等価ではない（`maxCand` が 0 だと何もデコード
しない）:

```kotlin
val p = Mfsk.defaultParams(ft8).copy(
    freqHintHz = 1500f,
    txFreqHz = 1500f,                       // FT8 の nftx。CAP_TX_FREQ が必要
    apHint = MfskApHint("K1JT", "HA0DU"),   // メッセージのフィールド順
)
MfskSession.open(ft8, p).use { s -> s.decode(pcm) }
```

`freqHintHz` と `txFreqHz` は nullable（C では NaN。0 Hz も周波数なので）。
`noiseBlanker` は `MfskNoiseBlanker.Percent(n)` か
`.Sweep(step, toleranceHz)`（FST4、`CAP_NOISE_BLANKER` が必要）。C 層と同じく、
モードが持たないパラメータは丸めずに **`open` で失敗し**、フィールド名と
能力ビットを含むメッセージを返す。`session.decode(pcm, params = …)` はその
呼び出しだけを上書きする。パラメータは3本のフラット配列で JNI を渡り、並びは
`mfsk_jni.c` の `read_params` に書いてある。JVM テストは各スロットを、それを
名指しする拒否メッセージで確かめ、AP ヒント・QSO 周波数・送信周波数は弱い
信号で端から端まで確かめている。

**Q65 にはデコードハンドルが無いので `Mfsk.decodeQ65`** — `mfsk_q65_decode_ex`（§2.8）
に対応する、WSJT-X 3.2 の設定を扱う 1 本の呼び出し。`MfskQ65Params` は
`Mfsk.q65DefaultParams(mode)` から始めて `copy` し、`pileup`・`emeDelay`・
`maxDrift`・`rxFreqHz` / `ftolHz`・`fading = MfskQ65Fading(…)`・
`list = MfskQ65List.Standard(…)` または `.Contest(…)`・`apHint` を持つ。行は
`nominalStartS` からの `dtSec` と `copiedLastTx` を返す:

```kotlin
val q65 = Mfsk.modes().first { Mfsk.modeName(it) == "Q65-30A" }
val p = Mfsk.q65DefaultParams(q65).copy(
    freqMinHz = 1450f, freqMaxHz = 1550f, maxDrift = 10,   // Max Drift、帯域は nfqso ± ntol に絞る
)
val rows = Mfsk.decodeQ65(q65, pcmFloat, p)
```

`MfskQ65History`（`q65_hist`: `push`、`record(rows)`、`lookup(rxFreqHz)`）と
`MfskQ65Callers`（`q65_hist2`: `record(freqHz, text, now)`、`expire(now)`、
`remove(call)`、`callers`）は呼び出し側が所有する `AutoCloseable` のハンドルで、
コンテストリストは `decodeQ65` に渡した callers を読む。`Mfsk.synthesizeQ65(…,
copiedLastTx)` は Pileup の返信を送る。JVM テストは、EME delay で 3 秒遅れのフレーム、
Pileup 下のフラグ付き返信、他に何も無い窓での q3 によるリストメッセージをデコードし、
配列マーシャリングの各スロットを、それを名指しする拒否メッセージで確かめている。

**`session.setBudget { … }`、`keepKnown`、`keepFftCache`** は同じ3戦略の
セッション単位版。budget 述語は候補ごとに JNI を1往復するので、捕捉した
デッドラインとの `System.nanoTime()` 比較程度に留めること。それより重い
ものは JVM 側が既に計算した boolean の裏に置く。

**`session.onDecode { row -> … }`** は `decode` が返すリストに加えて、
見つかった順に行を配信する — 長いスロットが終わる前に画面に何か出したい
UI 向け。リスナは rayon ワーカーから呼ばれるので並行安全である必要があり、
Android でビューに触れるものはメインルーパへ post しなければならない。
`configureRuntime` を先に呼ぶ必要は**無い** — シムは VM が見たことの無い
ワーカーを自分で（デーモンとして）アタッチし、リスナのメソッド ID を
ラムダの生成クラスではなく*インタフェース*から取る。リスナが投げた例外は
表示のうえクリアされ（rayon ワーカーには伝播先が無い）、デコードは続行する。

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

```swift
import MfskCore

let slot = try Mode.ft8.synthesiseSlot(call1: "CQ", call2: "JL1NIE", report: "PM95",
                                       frequencyHz: 1500)
let session = try DecodeSession(mode: .ft8)
for row in try session.decode(slot) {
    print(row.frequencyHz, row.snrDB, row.text)
}
```

* `Mode` / `ModeInfo` / `Capabilities` がイントロスペクション族を包むので、
  ピッカーはハードコードした一覧ではなくビルドから埋まる。
* `DecodeSession` は `MFSK_CAP_DECODE_HANDLE` を持つ全モードを覆う。
  `WSPR`、`JT9`、`JT65` は C と同様それぞれ独自のエントリポイントを持つ。
* `CaptureStream` が1スロット分の取り込みリングで、`session.decode(stream)` が
  スロットを出し入れするコピーを避ける融合デコードである。
* `session.setBudget { … }` は呼び出し側が時計を読む述語で探索を区切り
  （ライブラリは読まない）、`session.lastBudget` が打ち切りの残した仕事を
  — スキップした中で最良の候補の質も含めて — 告げる。`keepKnown(_:)` は
  あるデコードの結果を次回へ既知信号として持ち越し、`keepFFTCache(_:)` は
  同じ音声への2回目のパスでスロット変換を再利用する。
* `session.onDecode { row in … }` は呼び出しが返す配列と並行して、
  見つかった順に行を流す。`desktop` ビルドではクロージャは rayon ワーカー上で
  （場合により並行に）走り、`mobile` では候補順に単一スレッドで走る。
  クロージャは差し替えかセッション解放まで保持され、ハンドルを閉じる前に
  クリアされる。
* 失敗は `MfskError` を throw する。ステータスコードと理由文字列の両方を
  持ち、ハンドル自身のエラースロットを先に、スレッドローカルのグローバルを
  後に読む — `mfsk_session_copy_info` がハンドルを `const*` で取るため
  後者しか書けないからである。
* `Q65` は族全体を持つ — 4つのデコード戦略（通常、事前情報、高速フェージング、
  AP リスト）、`Q65SubMode`（**独自の番号体系**で `a15` が 6、`.mode` で
  `Mode` へ橋渡し）、`Q65FadingModel`、そしてセッションではなく呼び出し側が
  所有する唯一のハンドル `CallsignHashTable`。これらの enum が Swift に
  届くのは `cbindgen.toml` が `mfsk.h` へ出力するようになったからで、
  それ以前はラッパ側が 0…9 をハードコードするしかなかった。
* AP ヒントのフィールドは**メッセージのフィールドをその順に**並べたもので
  — CQ なら `call1` は `"CQ"`、送信局ではない — 探索を誘導するのではなく
  メッセージのビットを固定するため、順序を誤ると 0.1 dB 単位の損ではなく
  デコードが消える。両方向とも `Q65Tests` で固定されている。
* **`Q65.decode(_:mode:params:callers:)`** は `mfsk_q65_decode_ex`（§2.8）— WSJT-X 3.2
  の Q65 設定を扱う 1 本の呼び出し。`try Q65.Params(mode: .q65a30)` が既定値を
  返し、そこから `pileup`・`emeDelay`・`maxDrift`・`rxFrequencyHz` / `ftolHz`・
  `fading = Q65.Fading(…)`・`list = .standard(…)` または `.contest(…)`・`apHint` を
  変える。`Q65SubMode` ではなく `Mode` を取り、`Decode.dtSeconds` は
  `nominalStartSeconds` からの値。`Decode.copiedLastTx` は Pileup の `#`、
  `Q65.encode(…, copiedLastTx:)` で送る。`Q65History`（`q65_hist`）と
  `Q65Callers`（`q65_hist2`）は WSJT-X が持つ 2 つのリストで、呼び出し側が所有する
  クラス。`Q65ExtendedTests` は C と Kotlin のテストに対応している。
  **Swift ツールチェーンの無い環境で書いた — Mac で
  `bindings/swift/scripts/test.sh` を走らせること。**

**JTTY** は `JttyReceiver`（§2.8.1）: `try JttyReceiver(sampleRate:params:)` のあと
`try receiver.push(samples)` が生じた `[JttyUpdate]`（メッセージごとに 1 件、最新の
テキスト。`id` は不変、`isComplete`・`frequencyHz`・`startSeconds`）を返し、
`finish()` は未完の最終行を返す。`Mode.jtty` は `.decodeHandle` ではなく
`Capabilities.streamReceiver` を報告する。`JttyParams` は受信周波数・許容幅・
同期下限・帯域・減算の有無を持つ。一度に 1 スレッドで、メインアクターの外から
呼ぶこと — `push` は戻る前にデコードする。`JttyReceiverTests` は同梱の upstream
録音（`#filePath` で位置を求める）と自前のループバックを流す。`Jtty.tones(for:profile:)`
（テキストパッカー）、`Jtty.synthesise(_:)`、`Jtty.audio(for:)` がテキストを音声にする。

`bindings/swift/scripts/test.sh` が `libmfsk` をビルドして 75 件のテストを
走らせる。実アプリからのリンク（および iOS ビルドが `mobile` feature セットを
選ぶべき理由）は `bindings/swift/README.md` が扱う。

CI は同じスクリプトを `macos-latest` 上で走らせ（`Swift binding (macOS) +
iOS build`）、そこが `aarch64-apple-ios` のクロスコンパイル場所でもある。
XCTest は Command Line Tools ではなく Xcode に同梱され、iOS SDK も Xcode の
ものなので、1つのランナーで両方を賄う。ローカルでは `xcode-select` が CLT を
指している場合、スクリプトが `DEVELOPER_DIR` を Xcode に向ける。
