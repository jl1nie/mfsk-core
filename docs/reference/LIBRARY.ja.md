# mfsk-core — Rust API リファレンス

> **English:** [LIBRARY.md](LIBRARY.md)

WSJT-X の微弱信号デコーダ群 — FT8・FT4・FST4・WSPR・JT9・JT65・Q65・
MSK144・JTTY — に加え、実験的で WSJT 由来ではない `uvpacket` モードを、
1つの汎用コアの上に純 Rust で再実装したもの。コア
(`engine` / `fec` / `msg`) はプロトコル非依存で、各プロトコルは FEC
コーデック・メッセージコーデック・sync モードをそこへ挿す zero-sized
type である。配線済みの全プロトコルが同じ受信フロー
`coarse-sync → refine → LLR → FEC decode → message unpack` を通り、
その上にプロトコル毎の戦略が重なり、WSJT-X 自身のモデルに沿った 1 つのデコード API が
それを駆動する: モードごとに永続する `Decoder<P>` を、WSJT-X のパラメータブロックで
設定する（[§2](#2-デコード-api)）。MSK144 と JTTY はこのコアの中ではなく
横に置かれている。どちらもスロットを持たないので `Protocol` 型を持たない
([§3.1](#31-プロトコル毎の汎用-vs-専用))。

本書は Rust ホスト API を扱う。他の読者は:

| 目的 | 参照先 |
|---|---|
| C・C++・Kotlin・Swift から呼ぶ | [`BINDINGS.md`](BINDINGS.ja.md) |
| `no_std` / 固定小数点 / MCU | [`EMBEDDED.md`](EMBEDDED.ja.md) |
| 見つかった順にデコード結果を受け取る | [`STREAMING.md`](STREAMING.ja.md) |
| バッジと 20 行のクイックスタート | [`README.md`](../../README.md) |
| ある設計が*なぜ*そうなったか | [`../notes/DESIGN_RATIONALE.md`](../notes/DESIGN_RATIONALE.md) |

## 目次

- [1. クイックスタート](#1-クイックスタート)
  - [1.1 使い方の場面ごとに: デコーダは持ち続けるもの](#11-使い方の場面ごとに-デコーダは持ち続けるもの)
  - [1.2 0.12 からの移行](#12-012-からの移行)
- [2. デコード API](#2-デコード-api)
  - [2.1 `Decoder<P>`](#21-decoderp)
  - [2.2 `DecodeParams` と `Depth`](#22-decodeparams-と-depth)
  - [2.3 計算予算](#23-計算予算)
  - [2.4 ストリーミング配信](#24-ストリーミング配信)
  - [2.5 Extras と `Decoder` の外にあるプロトコル](#25-extras-と-decoder-の外にあるプロトコル)
  - [2.6 メッセージの受理](#26-メッセージの受理)
  - [2.7 広帯域 IQ 入力](#27-広帯域-iq-入力)
  - [2.8 非標準・複合・接尾辞付きコールサイン](#28-非標準複合接尾辞付きコールサイン)
- [3. プロトコル](#3-プロトコル)
  - [3.1 プロトコル毎の汎用 vs 専用](#31-プロトコル毎の汎用-vs-専用)
  - [3.2 諸元](#32-諸元)
  - [3.3 プロトコル別の注記](#33-プロトコル別の注記)
  - [3.4 デコード戦略](#34-デコード戦略)
- [4. モジュールとクレートの地図](#4-モジュールとクレートの地図)
- [5. `Protocol` トレイト階層](#5-protocol-トレイト階層)
- [6. engine プリミティブ](#6-engine-プリミティブ)
- [7. Feature フラグ](#7-feature-フラグ)
- [8. ランタイムレジストリと trait 面の検証](#8-ランタイムレジストリと-trait-面の検証)
- [ライセンス](#ライセンス)

---

## 1. クイックスタート

```toml
[dependencies]
mfsk-core = { version = "0.13", features = ["ft8", "ft4", "wspr"] }
```

必要なプロトコル feature だけを入れる。以下の例は説明のために複数を
有効にしている。

**FT8 スロットをデコードする。** フレームを合成してから復号する:

```rust
use mfsk_core::decoder::{DecodeParams, Decoder, SlotInput};
use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
use mfsk_core::ft8::Ft8;
use mfsk_core::msg::wsjt77::pack77;

// 1. FT8 フレームを合成し、15 秒スロットに詰める。
let msg77 = pack77("CQ", "JA1ABC", "PM95").unwrap();
let tones = message_to_tones::<Ft8>(&msg77);
let frame = synthesize_i16::<Ft8>(&tones, 12_000, /* freq */ 1500.0, /* amp */ 20_000);

let mut audio = vec![0i16; 180_000]; // 15 s @ 12 kHz
let start = (0.5 * 12_000.0) as usize;
for (i, &s) in frame.iter().enumerate() {
    if start + i < audio.len() { audio[start + i] = s; }
}

// 2. デコードする。モードごとにデコーダを 1 つ持つ。パラメータブロックは
// WSJT-X のもの（帯域・depth・自局…）。`Depth::Deep` が既定。
let mut decoder = Decoder::<Ft8>::new(DecodeParams::for_band((100.0, 3_000.0)));
let result = decoder.decode(&SlotInput::i16(&audio));
assert!(!result.rows.is_empty(), "roundtrip must decode");
for row in &result.rows {
    let d = &row.decoded;
    println!("{:7.1} Hz  dt={:+.2} s  SNR={:+.0} dB  {}",
             d.freq_hz, d.dt_sec, d.snr_db, d.text);
}
```

実音声は、名目開始位置から始まる 12 kHz の 1 周期として届く。他のサンプルレートは
`engine::dsp::resample` で変換する。`SlotInput` は `&[i16]` と `&[f32]` を取る。
`Decoder` は周期をまたいで持ち続けること。コールサインのハッシュ表はそこにある
（[§2.1](#21-decoderp)）。`Decoder::<Ft8>::with_defaults()` は、帯域を自分で指定する
代わりに WSJT-X の GUI が起動時に持つブロックから始める
（[§2.2](#22-decodeparams-と-depth)）。

### 1.1 使い方の場面ごとに: デコーダは持ち続けるもの

0.13 より前、デコードは関数呼び出しだった。音声を渡せば行が返り、周期をまたいで残すべきもの
（コールサインのハッシュ表、前の周期のデコード結果、Q65 の平均）は呼び出し側が持ち、毎回
渡し直していた。0.13 のデコーダは、一度作って持ち続けるオブジェクトである。WSJT-X が
セッションの間モードごとに 1 つのデコーダを動かし続けるのと同じである。設定は WSJT-X の
GUI と同じく周期の合間に変え、WSJT-X が覚えていることをデコーダが覚えている。以下は
よくある使い方と、それぞれ API がその形になっている理由である。例が従うルールは
[§2](#2-デコード-api) の冒頭にまとめてある。

**ライブ受信: デコーダを持ち続け、周期に番号を付ける。** API はこの使い方を中心に
作られている。デコーダは一度だけ作り、周期が終わるたびに渡し、周期には UTC の格子上の
番号（`t / T`）を付ける。番号は、どの周期が連続しているかをデコーダに伝える。FT8 の a7 と
Q65 の平均はこれを必要とする。コールサイン表はデコーダの中にあるので、ある周期で聞いた
コールが、次の周期のハッシュ化された `<...>` を解決する:

```rust
use mfsk_core::decoder::{Decoder, SlotInput};
use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
use mfsk_core::ft8::Ft8;
use mfsk_core::msg::wsjt77::{pack77, pack77_type4};

/// FT8 フレームを 1 つ含む 15 秒の周期。受信機が渡すものと同じ形。
fn period(msg77: &[u8; 77]) -> Vec<i16> {
    let tones = message_to_tones::<Ft8>(msg77);
    let frame = synthesize_i16::<Ft8>(&tones, 12_000, 1_500.0, 20_000);
    let mut audio = vec![0i16; 180_000];
    audio[6_000..6_000 + frame.len()].copy_from_slice(&frame);
    audio
}

// 一度だけ作り、セッションの間ずっと持つ。
let mut rx = Decoder::<Ft8>::with_defaults();

// 周期 0: JA1ABC が CQ を出す。デコーダがそのコールを覚える。
let p0 = period(&pack77("CQ", "JA1ABC", "PM95").unwrap());
rx.decode(&SlotInput::i16(&p0).period(0));

// 周期 1: JA1ABC を 12 ビットのハッシュでしか名乗らない応答。
let p1 = period(&pack77_type4("JL1NIE/1", "JA1ABC", "RR73", false).unwrap());
let heard = rx.decode(&SlotInput::i16(&p1).period(1)).rows;
assert!(heard.iter().any(|r| r.decoded.text.contains("<JA1ABC>")));

// 同じ音声を、周期 0 を聞いていないデコーダに通すと解決できない。
let mut fresh = Decoder::<Ft8>::with_defaults();
let blind = fresh.decode(&SlotInput::i16(&p1).period(1)).rows;
assert!(blind.iter().all(|r| !r.decoded.text.contains("<JA1ABC>")));
```

*理由:* これは `jt9` のプロセスがしていることで、状態を本家と同じ場所に置けば、
呼び出し側がそれを失くしたり取り違えたりできない。0.12 では呼び出し側がみな表を
持ち回り、このクレート自身の IQ 受信器は `<...>` を一度も解決できなかった（0.12.0 は
それで yank された）。

**QSO を進める: 運用者が知っていることをデコーダに伝える。** 自局のコール、相手局、
QSO がどこまで進んだかは、GUI が埋めるのと同じくパラメータブロックに入れる。デコーダは
本家の表に従い、そこから a-priori の仮説を作る。レポートを待っている間は
`MyCall DxCall ???`、こちらがレポートを送った後は `... RRR` / `73` / `RR73` である。
周期の合間に `params_mut()` で書き換えれば、それ以外はデコーダが保持する。

```rust
use mfsk_core::decoder::{
    ApMode, DecodeParams, Decoder, QsoContext, QsoProgress, SlotInput,
};
use mfsk_core::ft8::Ft8;

let mut rx = Decoder::<Ft8>::new(
    DecodeParams::for_band((200.0, 4_000.0))
        .station("JL1NIE", "PM95") // MyCall、MyGrid
        .rx_freq(1_500.0)          // 相手局がいる周波数
        .tx_freq(1_500.0)          // 自局が送信する周波数
        .ap(ApMode::Full),         // 「Enable AP」
);
let period = vec![0i16; 180_000]; // 無線機からの 1 周期

// JA1ABC の CQ に応答した。
rx.params_mut().qso = QsoContext {
    his_call: "JA1ABC".into(),
    his_grid: "PM95".into(),
    progress: QsoProgress::Replying,
};
rx.decode(&SlotInput::i16(&period).period(100));

// 次の周期: レポートを送ったので、RRR / 73 / RR73 を待つ。
rx.params_mut().qso.progress = QsoProgress::RogerReport;
rx.decode(&SlotInput::i16(&period).period(101));
```

*理由:* これが、待っている弱い応答を WSJT-X が拾う仕組みである。QSO の文脈を設定すると、
FT8 の弱い応答は 30 回中 20 回デコードされ、AP 無しでは 30 回中 0 回だった。`station` を
設定するまでは何も動かない。また FT8 の既定は、GUI の「Enable AP」の初期状態と同じく
AP オフである。

**複数のバンドやモードを同時に: それぞれにデコーダを 1 つ。** スキマーや、FT8 と FT4 を
同時に見る受信機は、チャンネルごとにデコーダを持つ。`AnyDecoder` は実行時にモードを選び、
デコーダは `Send` なので、それぞれを別のスレッドで動かせる:

```rust
use mfsk_core::Mode;
use mfsk_core::decoder::AnyDecoder;

let channels = [Mode::Ft8, Mode::Ft4];
let workers: Vec<_> = channels
    .into_iter()
    .map(|mode| {
        std::thread::spawn(move || {
            let mut rx = AnyDecoder::with_defaults(mode);
            let slot = vec![0i16; mode.meta().slot_samples_12k as usize];
            rx.decode_i16(&slot, Some(0)).rows.len()
        })
    })
    .collect();
for w in workers {
    w.join().unwrap();
}
```

*理由:* WSJT-X を 2 つ動かせば表は 2 つあり、デコーダ 2 つも同じである。20 m で聞いた
コールが 40 m のハッシュを解決することはなく、2 つのスレッドが 1 つの表を取り合うことも
ない。新しいチャンネルに先に覚えさせたいコールは `learn_callsign` で入れる。

**締め切り: 間に合う範囲でデコードする。** ライブラリは時計を読まない。続けるかどうかを
答える関数を渡すと、候補の合間にそれが呼ばれる。締め切りで探索が打ち切られたかは、
結果の報告で分かる:

```rust
use std::time::{Duration, Instant};
use mfsk_core::decoder::{Decoder, SlotInput};
use mfsk_core::ft8::Ft8;

let mut rx = Decoder::<Ft8>::with_defaults();
let period = vec![0i16; 180_000];
let deadline = Instant::now() + Duration::from_millis(500);
let keep_going = || Instant::now() < deadline;

let result = rx.decode(&SlotInput::i16(&period).budget(&keep_going));
if result.budget.exhausted {
    println!("打ち切り: 候補 {} 件をスキップ", result.budget.candidates_skipped);
}
```

*理由:* 同じコードがデスクトップ、wasm、MCU で動き、その 3 つは時計が違う。また
プロセスがスロットの途中で止められることもある。FT8・FT4・FST4 が対応している
（[§2.3](#23-計算予算)）。

**見つかった行をすぐ画面に出す。** `decode_with` は、行がデコードされるたびにすぐ
コールバックに渡し、最後に全部をまとめて返す:

```rust
use mfsk_core::decoder::{Decoder, SlotInput};
use mfsk_core::ft8::Ft8;

let mut rx = Decoder::<Ft8>::with_defaults();
let period = vec![0i16; 180_000];
let all = rx.decode_with(&SlotInput::i16(&period).period(7), &|row| {
    println!("{:+.1} s {:6.0} Hz  {}", row.decoded.dt_sec, row.decoded.freq_hz, row.decoded.text);
});
println!("この周期で {} 行", all.rows.len());
```

*理由:* FT8 の深い探索には時間がかかり、GUI はその全部を待つべきではない。配信の順序と
重複除去の約束は [`STREAMING.md`](STREAMING.md) にある。

**GUI と違う探索をする。** `Depth` は、WSJT-X の Fast・Normal・Deep と同じ探索を与える。
それ以外（マイコンの計算量に合わせる、計測で 1 つのつまみを固定する、0.12 の探索に
戻す）は、モードの `Tuning` extra で設定する。設定した項目だけが上書きされる:

```rust
use mfsk_core::decoder::{Decoder, Ft8Strategy};
use mfsk_core::ft8::Ft8;

let mut rx = Decoder::<Ft8>::with_defaults();
let tuning = &mut rx.extras_mut().tuning;
tuning.sync_min = Some(0.8); // 0.12 の FT8 の探索: 低い sync の下限、
tuning.max_cand = Some(60); // 少ない候補数、
tuning.strategy = Some(Ft8Strategy::SinglePass); // 1 パスで引き算なし
```

*理由:* WSJT-X がすることは `DecodeParams` にあり、このクレートが足したものは `Extras` に
ある。そのため、WSJT-X のものではない設定は、コードの上でもそれと分かる。

**別のサンプルレートの音声。** デコーダは `jt9` と同じく 12 kHz の音声を受け取る。
他のレートは `engine::dsp::resample::resample_to_12k`（とその `f32` 版・ストリーム版）で
変換する。12 kHz より上では、6 kHz より上の成分が帯域に折り返さないよう、まず元のレートで
低域通過フィルタを通す。48 kHz では WSJT-X 自身のフィルタ、つまり `Detector.cpp` が
サウンドカードの音声に通す 49 タップの `fil4` を使い、他のレートでは同じ仕様で作った
低域通過フィルタを使う。その後で線形補間する。0.13.0 まではフィルタが無く、48 kHz は
単に間引かれていた。全帯域に雑音がある入力（マイクや雑音の多いサウンドカード）では、
FT8 で約 6 dB 損をしていた（#576）。

```rust
use mfsk_core::engine::dsp::resample::resample_to_12k;

let at_48k = vec![0i16; 48_000 * 15]; // 48 kHz のサウンドカードからの 15 秒
let at_12k = resample_to_12k(&at_48k, 48_000);
assert_eq!(at_12k.len(), 180_000);
```

**録音ファイル 1 つ: デコーダ 1 つと呼び出し 1 回。** ファイルはライブ受信の特殊な場合で、
上のクイックスタートのとおり `Decoder::new(params)` と `decode` 1 回である。一発用の別の
API は無い。一発の呼び出しにしかできないことが無いからである。

周期は名目開始位置から始まる周期全体である。それより短いバッファも長いバッファも受け付け
（`tests/decoder_input_length.rs` は全モードを空のバッファから 1.5 周期まで流す）、短いものは
含まれている分だけを返す。0.13.0 までは、FT8 や FT4 のフレームより短いバッファで panic する
ことがあった（#567）。

### 1.2 0.12 からの移行

0.13 はデコード API を拡張したのではなく、置き換えた。ファミリごとのリクエストビルダーは
無くなり、WSJT-X 自身のパラメータブロックで動く、モードごとに 1 つのデコーダがその代わりに
なった（[§2](#2-デコード-api)）。0.12.0 は yank 済みで、0.12.1 は無い。なぜこの形にしたのか、
そのために何を手放したのかは
[`DESIGN_RATIONALE.md` §6](../notes/DESIGN_RATIONALE.md#6-the-decode-api-follows-wsjt-xs-decoder-013)
にある。

**変わるのはコードだけでなく結果も。** 既定値が WSJT-X の GUI のものになった
（[§2.2](#22-decodeparams-と-depth)）ので、オプションを何も設定しない同じ音声でも、
同じ結果は返らない:

| | 0.12 | 0.13 | 測定 |
|---|---|---|---|
| FT8 の探索 | sync 0.8、候補 60 | `Depth::Deep`: sync 1.3、候補 1000、`SicEarly` | `qso3_busy.wav` で `jt9 -8 -d 3` と同じ 21 メッセージ。本家との比較で `jt9` の 3.8〜4.0 倍の速度（[`BENCHMARKS.md`](../notes/BENCHMARKS.md)） |
| 帯域 | 100〜3000 Hz | FT8 / FT4 は 200〜4000 Hz、FST4 は 600〜1400 Hz | — |
| AP | ヒントがあれば常にオン | FT8 と JT65 はオフ（「Enable AP」の初期状態と同じ）。`station` を設定すれば QSO 文脈 AP | FT8 の弱い応答: AP オフで 30 中 0、オンで 30 中 20 をデコード |
| JT9 / JT65 | 無音から全ゼロの符号語（`000AAA 000AAA RA90`）が返っていた | 本家と同じく捨てる | — |
| WSPR | `wsprd` に近い | `wsprd` 自身の数値 | WSJT-X のゴールデンで、計測用に改造した `wsprd` と SNR が 0.02 dB 以内、DT は小数 3 桁まで一致 |

0.12 の探索を保ちたければ、フレーム系の `Tuning` extra を設定する
（[§2.5](#25-extras-と-decoder-の外にあるプロトコル)）。QSO 無しで AP を使いたければ、
`station` か `ap_hint` extra を設定する。

**コード。** 0.12 の呼び出し側が変更すべきこと（全一覧は `CHANGELOG.md` の
`## 0.13.0`）:

| 領域 | 0.12 | 0.13 |
|---|---|---|
| デコード入口、FT8 / FT4 / FST4 | `msg::decode_request::DecodeRequest<P>` / `SniperRequest<P>` とそのビルダー | `Decoder::<P>::new(DecodeParams)` ＋ `decode(&SlotInput)`。オプションは `P::Extras`。リクエスト型は `pub(crate)`（`internal-testing` で開く） |
| デコード入口、WSPR / JT9 / JT65 / Q65 | `wspr::`、`jt9::`、`jt65::`、`q65::DecodeRequest`。Q65 の `SniperRequest`、`MultiPeriodRequest` | 同じ `Decoder<P>`。広帯域リクエストは `pub(crate)`。`SniperRequest`（既知の位置でのデコード）は公開のまま。Q65 の平均は `averaging` ＋ `SlotInput::period` |
| オプション | リクエストごとのビルダーメソッド（`.osd()`、`.strictness()`、`.eq_mode()`、`.ap_hint()`、`.sic_*()`、`.contest()`、`.tx_freq()`…） | `DecodeParams`（WSJT-X のブロック: 帯域、`rx_freq_hz`、`tx_freq_hz`、`depth`、`station`、`qso`、`ap`、`contest`、`eme_delay`…）とモード別の `Extras`（`Tuning`、`ap_hint`、`eq`、`filter`、`a7`、`sniper`、`noise_blanker`、Q65 のもの） |
| 周期をまたぐ状態 | `.previous_cycle()`、`.hash_table(Arc)`、WSPR の `.table(&mut)` / `.confirmed()`、`MultiPeriodRequest` | デコーダの状態: `Decoder::clear()`、`learn_callsign`、`unpack77`。ハッシュ表はデコーダごとで、共有されない |
| `wsjtx_depth` | `WsjtxDepth::{D1, D2, D3}` を取る FT8 のコンストラクタ | `DecodeParams::depth`、`Depth::{Fast, Normal, Deep}`: モードごとに、`ndepth` と同様に**探索設定の全てを決める** |
| 既定値 | FT8 は sync 0.8 / 候補 60、ヒントがあれば AP オン、帯域 100〜3000 | depth のもの: `Deep`（FT8 は sync 1.3、候補 1000）、FT8 と JT65 は AP オフ、FT8 / FT4 は帯域 200〜4000、FST4 は 600〜1400（`default_params`） |
| AP | 自由形式の `ApHint` だけ（QSO のコードワードを持つのは Q65 のみ） | FT8・FT4・FST4 は `station` ＋ `qso` ＋ `ap` と本家の `naptypes` による QSO 文脈 AP。`ApHint` は `ap_hint` extra として残る |
| JT9 / JT65 | 全ゼロの符号語を報告していた | 報告しない |
| 戻り値 | `DecodeOutcome { results, fft_cache, budget }` | `SlotResult { rows: Vec<Row { decoded, detail, native }>, budget }`。`fft_cache` は無い |
| ストリーミング | 各リクエストの `.on_result(cb)` | `Decoder::decode_with(&slot, on_row)`。行はデコーダの表で解決済み |
| 予算 | `.budget(check)` | `SlotInput::budget(check)` |
| 音声 | `&[i16]`（フレーム系）、`&[f32]`（それ以外） | 全モードで `SlotInput::i16` / `SlotInput::f32` |
| 実行時のモード | `iq::IqMode` | `registry::Mode` と `AnyDecoder` |
| IQ | `IqReceiver` は凍結した既定値で `push_*` の中でデコードし、`on_decode`、`set_time_anchor`、`IqDecode` 行を持っていた | プル型: `push_*(.., &mut Vec<CompletedSlot>)`、`set_time(utc_ns, at_sample)` → `ClockChange`、`retune` → `RetuneReport`。デコードはチャンネルごとの `AnyDecoder` で行う |
| 時刻 | 受信器ごとのスロット算術 | `slotgrid::{SlotGrid, SampleClock, SlotCutter}` |

**無くなったもの、と代わりの方法。**

| 0.12 | 代わり |
|---|---|
| `.hash_table(Arc)` — 複数のリクエストで共有する 1 つの表 | デコーダごとに自分の表を持つ（本家のプロセスごとと同じ）。同じモードの 2 チャンネルは別々に学習する。`learn_callsign` で種を入れられる |
| `.previous_cycle()`、`MultiPeriodRequest` — 呼び出しごとに渡す状態 | デコーダが保持する。どれが連続した周期かが分かるよう、`SlotInput::period` で周期に番号を付ける |
| `.known(list)` — 前のパスで既にデコードした信号を、探索の前に引き算する | 対応するものは無い。一度デコードしたスロットへの 2 回目のパス（WebFT8 の 2 段デコード）は API に含まれない。本家での形は早期デコードで、[#572](https://github.com/jl1nie/mfsk-core/issues/572) |
| `.message_filter(closure)` / `.also_accept(closure)` | `MessageFilter::Only(f)` / `AlsoAccept(f)`。`f` は関数ポインタで、extras が `Clone + 'static` のままでいられるようにしている。何も取り込まないクロージャは変換される。データが要る判定は `static` から読む（[§2.6](#26-メッセージの受理)） |
| `.fft_cache()` / 結果の `fft_cache` — 2 回目のパスに渡すスロットの FFT | 対応するものは無い。理由は `.known()` と同じ |
| `wsjtx_depth(WsjtxDepth::D1…D3)` | `DecodeParams` の `Depth::Fast` / `Normal` / `Deep`。全モードに効く |

0.12 の合成 API（`engine::tx`）、公称開始位置からの `dt_sec`、`engine::search` は変わらない。

---

## 2. デコード API

mfsk-core は WSJT-X と同じやり方でデコードする。WSJT-X はモードごとに 1 つのデコーダ
（`jt9 -s`）を走らせ、GUI は周期ごとにパラメータブロック（`lib/jt9com.f90`）を埋めてから
デコーダに読ませ、デコーダが周期をまたいで保持するのは SAVE 変数とモジュール変数に
あるものだけである。`mfsk_core::decoder` がそのモデルである:

| WSJT-X | mfsk-core |
|---|---|
| 1 モード分のデコーダプロセス | `Decoder<P>` |
| `params` ブロック（`nfa`、`nfb`、`nfqso`、`ndepth`、`mycall`、`hiscall`、`lft8apon`…） | `DecodeParams`。周期の合間に `params_mut()` で変える |
| 1 周期分の音声（`id2`） | `SlotInput` |
| SAVE 変数が保持するもの（ハッシュ表、a7、平均スペクトル） | `P::State`。デコーダごとに、初回使用時に確保 |
| 無し: 本家にそのオプションは無い | `P::Extras`。モードごとに型付けされ、そのモードに無いオプションはコンパイルできない |

リクエストごとのオプションオブジェクトも、その横の一発 API も無い。録音ファイルは
`Decoder::<P>::new(params)` と `decode` 1 回である。その下の engine 関数
（`decode_frame`、`process_candidate_basic`、`GenericPipelineProtocol` トレイト、
ファミリ別の `DecodeRequest` 型）は `pub(crate)` で、下流がデコーダを迂回できない
ようにしてある。既定外の `internal-testing` feature がクレート自身の統合テスト向けに
それを開ける。`Decoder<P>` があるのは、スロットでデコードする 7 ファミリ（FT8・FT4・FST4・
WSPR・JT9・JT65・Q65）である。uvpacket・MSK144・JTTY は独自のエントリポイントを持つ
（[§2.5](#25-extras-と-decoder-の外にあるプロトコル)）。

**この API が従うルール。** どれも本家の振る舞いを意図して残したもので、これを知って
いれば以下の大半は予測できる。

1. **モードごとに 1 つのデコーダで、本家が保持するものを保持する。** ハッシュ表、FT8 の
   a7 の行、Q65 と JT65 の平均は `Decoder` の中にあり、デコーダ間で共有されず、
   `clear()` まで残る（[§2.1](#21-decoderp)）。
2. **パラメータブロックは WSJT-X のもので、各モードは本家のデコーダが読むものだけを
   読む。** モードが読まない項目は、`jt9` と同じく無視され、拒否はされない。
   [§2.2](#22-decodeparams-と-depth) の「読むモード」の列で確かめること。
3. **探索は `ndepth` と同様に `Depth` が決める。** ライブラリ独自のつまみ（`Tuning`）は、
   設定したときだけ設定を上書きする（[§2.5](#25-extras-と-decoder-の外にあるプロトコル)）。
4. **本家に無いものは、モードごとに型付けされた `Extras` の項目になる。** そのモードに
   無いオプションはコンパイルできない。`AnyDecoder` や C ABI 経由では `Unsupported`
   エラーになり、黙って無視されることはない。
5. **既定値は WSJT-X の GUI のもの**なので、何も設定しないデコーダは、何も設定しない
   WSJT-X と同じことをする。
6. **デコーダを迂回する道は無い。** その下の engine は `pub(crate)` なので、どの呼び出し側も
   同じ状態の扱いと同じ既定値を使う。

### 2.1 `Decoder<P>`

```text
pub struct Decoder<P: Decodable> { params, extras, state }
```

`Decodable` は、スロットでデコードする全 ZST が実装する。モード（`const MODE: Mode`）、
`type State`（本家が周期をまたいで保持するもの）、`type Extras`（本ライブラリが足すもの）、
`type Row`（そのモード固有の結果）を束ねる。

| メソッド | 効果 |
|---|---|
| `Decoder::new(params)` | そのブロックを持つデコーダ。最初のデコードまで何も確保しない |
| `Decoder::with_defaults()` | GUI でそのモードが持つブロック（[§2.2](#22-decodeparams-と-depth)） |
| `params()` / `params_mut()` | ブロック。GUI が書き換えるのと同様、周期の合間に変える。状態は保たれる |
| `extras()` / `extras_mut()` / `with_extras(e)` | そのモードのライブラリ独自オプション（[§2.5](#25-extras-と-decoder-の外にあるプロトコル)） |
| `decode(&SlotInput)` | 1 周期をデコード → `SlotResult<P::Row>` |
| `decode_with(&SlotInput, on_row)` | 同じ。見つかるたびに各行を `on_row` へ渡す（[§2.4](#24-ストリーミング配信)） |
| `unpack77(&[u8])` | パック済み 77 ビットメッセージをテキストにする。`<...>` は**このデコーダの**表で解決 |
| `learn_callsign(&str)` | このデコーダの表にコールサインを教える（`save_hash_call`）。ハッシュ呼出符号を持たないモードでは `false` |
| `clear()` | 周期をまたいで持っているものを全て忘れる（WSJT-X の "Clear Avg" と `ndepth & 128`） |

`Decoder<P>` は `Send` である（`Sync` は要求されない）: ワーカースレッドへ移して使い、
チャンネルごとに 1 つ持たせる。

**1 周期を入れて、行が出てくる。** `SlotInput` は、名目開始位置から始まる 1 周期分の
音声である:

| フィールド / コンストラクタ | 意味 |
|---|---|
| `SlotInput::i16(&[i16])`、`SlotInput::f32(&[f32])` | 音声。`Audio::I16`（`jt9` が `id2` として読むもの）または `Audio::F32`。フレーム系（FT8・FT4・FST4）は WSJT-X と同じく 16 ビット音声を取るので、`F32` はまず固定 RMS（`decoder::F32_TO_I16_RMS`）に揃えられる。WSPR・JT9・JT65・Q65 は `f32` で動くので、`I16` は 32768 で割られる。呼び出し側がレベルを選ぶことはない |
| `.period(n)` | 周期の UTC グリッド上の番号（`t / T`）。連続した周期を要する状態（FT8 の a7、Q65 の平均）は、これが分かっているときだけ使われる。無ければ、単発の録音はその状態に触れない |
| `.budget(check)` | 締切の述語、[§2.3](#23-計算予算) |

**段階的・早期デコードのエントリポイントは無い**: `SlotInput` は周期全体である。
（WSJT-X の nzhsym 41/47/50 の早期デコードはこの API の一部ではない。ボードは
低レベルの項目の上で独自の先頭部分パスを走らせる。）

`SlotResult<R>` は `rows: Vec<Row<R>>`（見つかった順）と `budget: BudgetReport` である。
`Row<R>` は 1 つのデコードの 3 つの見え方を持つ:

| フィールド | 内容 |
|---|---|
| `decoded: Decoded` | モード共通の行: `text`（デコーダのハッシュ表で解決済み）、`freq_hz`、`dt_sec`、`snr_db`、`protocol` |
| `detail: RowDetail` | それ以外でモード間に共通するもの: `sync_score`、`sync_cv`、`hard_errors`、`pass`、`info`、`hash_resolved`（`<...>` の解決に表が要った）、`copied_last_tx`（Q65 Pileup）。持たないモードは既定値のまま。WSPR・JT9・JT65 はどれも埋めない |
| `native: R` | そのモード固有の結果: `DecodeResult`（FT8・FT4・FST4）、`WsprResult`、`Jt9Result`、`Jt65Result`、`Q65Result` |

**デコーダが周期をまたいで持つもの**は、本家のデコーダが持つものだけである。デコーダごと
であり、デコーダ間で共有されることはない（本家の表がプロセスごとなのと同じ）: 同じ
モードの 2 チャンネルは 2 つの表を持つ。

| モード | `State` | 本家 |
|---|---|---|
| FT8・FT4・FST4 | `FrameState`: コールサインのハッシュ表。`a7` extra を有効にした FT8 は直近 2 周期分の復号結果も | `packjt77`、`ft8_a7.f90` |
| Q65 | `Q65State`: ハッシュ表、シンボルスペクトルの移動平均（`s1a`、`navg`）と直近の周期番号 | `packjt77`、`q65.f90` の SAVE |
| WSPR | `WsprState`: OSD が、Fano が既に聞いた局を確認できるようにするコールサイン表（上限なし） | wsprd の `hashtable.txt` |
| JT9 | `()` — 72 ビットメッセージはハッシュ呼出符号を運ばない | — |
| JT65 | `Averager`: `avg65` が合算する周期（最大 64、各 63 × 64 のシンボル電力。1 つ 16 KB で、来た分だけ確保） | `jt65_decode.f90` の `avg65` |

ハッシュは候補ループの**後**に、単一スレッドで、デコード順に解決・学習される
（`unpack77_learn`）: メッセージは、自分が導入する呼出符号で自身のハッシュを解決
しないし、周期 *n* で聞いた呼出符号は**同じ**デコーダの周期 *n*+1 の `<...>` を解決する。
表は最初の挿入時に 1 ブロックとして遅延確保されるので、`Decoder::new` は組込みヒープ
では何も消費しない。

```rust
use mfsk_core::decoder::{DecodeParams, Decoder};
use mfsk_core::ft8::Ft8;
use mfsk_core::msg::wsjt77::pack77_type4;

// "<JA1ABC> JL1NIE/1 RR73": 標準コールサインは 12 ビットのハッシュで運ばれる。
let msg77 = pack77_type4("JL1NIE/1", "JA1ABC", "RR73", false).unwrap();

// 新しいデコーダには、そのハッシュが誰のものか分からない ...
let mut decoder = Decoder::<Ft8>::new(DecodeParams::for_band((200.0, 3_000.0)));
let blind = decoder.unpack77(&msg77).unwrap();
assert!(blind.contains("<...>"), "{blind}");

// ... その呼出符号を聞いたデコーダには分かるが、別のデコーダには分からないまま。
assert!(decoder.learn_callsign("JA1ABC"));
assert!(decoder.unpack77(&msg77).unwrap().contains("<JA1ABC>"));
let other = Decoder::<Ft8>::new(DecodeParams::for_band((200.0, 3_000.0)));
assert!(other.unpack77(&msg77).unwrap().contains("<...>"));
```

**実行時にモードを選ぶ: `AnyDecoder`。** `AnyDecoder::new(Mode, DecodeParams)` は、
このビルドが持つ `registry::Mode`（`Mode::ALL`、名前からの検索は `Mode::from_name`）ごとに
1 つのバリアントを持つ enum で、`match` で振り分けられる: `Box<dyn>` も、デコード経路での
確保も無い。モードをデータとして持つコード向けにある: IQ レシーバの呼び出し側
（[§2.7](#27-広帯域-iq-入力)）、C ABI、GUI。メソッドは `Decoder` のものと対応し、結果は
モード非依存の `AnySlotResult { rows: Vec<Decoded>, details: Vec<RowDetail>, budget }`
（モード固有の結果が欲しいコードは型付きの `Decoder<P>` を持つ）。`extras_mut()` は
`match` するための `AnyExtras` を返し、`set_ap_hint` は自由形式の AP ヒントを、それを取る
モードに設定して、取らないモードでは `Err(Unsupported { mode, option })` を返す。
`decode_i16(audio, period)` は短縮形である。`AnyDecoder` は、プロトコル feature が少なくとも
1 つあるときだけ存在する。

```rust
# #[cfg(all(feature = "ft8", feature = "wspr"))] {
use mfsk_core::Mode;
use mfsk_core::decoder::{AnyDecoder, AnyExtras, DecodeParams};
use mfsk_core::msg::ApHint;

let mut dec = AnyDecoder::new(Mode::Ft8, DecodeParams::for_band((200.0, 3_000.0)));
dec.set_ap_hint(Some(ApHint::new().with_call1("CQ").with_call2("JA1ABC"))).unwrap();
if let AnyExtras::Ft8(e) = dec.extras_mut() {
    e.a7 = true; // そのモード固有のオプション。型付き
}
assert_eq!(dec.mode(), Mode::Ft8);

// WSPR に AP ヒントは無い: 不一致は no-op ではなくエラー値になる。
let mut wspr = AnyDecoder::with_defaults(Mode::Wspr);
assert!(wspr.set_ap_hint(None).is_err());
# }
```

### 2.2 `DecodeParams` と `Depth`

`DecodeParams` は `lib/jt9com.f90` の `params` ブロックである（`#[non_exhaustive]`。
`DecodeParams::for_band((lo, hi))` とチェーンするセッターで作る）。**各モードは、自分の
本家デコーダが読むものを読み、残りは無視する（`jt9` と同じ）**: フィールドがあるからと
いって、そのモードにオプションが存在することにはならない。

| フィールド | 本家 | 読むモード |
|---|---|---|
| `band_hz` | `nfa`、`nfb` | 全モード |
| `rx_freq_hz` | `nfqso` | FT8（10 Hz 以内の候補を先にデコード、相手を名指す仮説はこの周波数の 50 Hz 以内だけ、a8、sniper の中心）、FT4 と FST4（同じ AP 規則）、JT9（Rx 周波数パス）、Q65（q3 リスト復号） |
| `tol_hz` | `ntol` | JT9（既定 50 Hz）、Q65（F Tol、既定 10 Hz） |
| `tx_freq_hz` | `nftx` | FT8: この周波数の 50 Hz 以内で両コールサインの仮説 |
| `depth` | `ndepth & 7` | 全モード。下の表 |
| `averaging` | `ndepth & 16` | Q65 と JT65（どちらも `SlotInput::period` が要る。JT65 のは `jt65::averaging`、`avg65`） |
| `deep_search` | `ndepth & 32` | JT65 の本家フラグ。まだ読まない |
| `station` | `mycall`、`mygrid` | FT8・FT4・FST4（AP）、Q65（AP リスト） |
| `qso` | `hiscall`、`hisgrid`、`nQSOProgress` | 同上 |
| `ap` | `lft8apon`、`lapcqonly` | 同上: `ApMode::{Off, CqOnly, Full}` |
| `contest` | `ncontest` | FT8（`/R` と `TU; ` のメッセージを残す、[§2.6](#26-メッセージの受理)。コンテストの `CQ` トークン。Fox）、FT4 と FST4（`CQ` トークン）、Q65（コーラーのリスト） |
| `eme_delay` | `emedelay` | FT8（報告する `dt` が 2 秒遅くなる）、Q65（探索窓の遅い側の端が +5.5 s、Q65-15 では +4.0 s に動く） |

**既定値は GUI に従う。** `Decoder::with_defaults()` と `decoder::default_params(mode)` は
`Depth::Deep`（GUI の `NDepth` 既定）、**FT8 と JT65 は AP オフ**（"Enable AP" の
チェックボックスが未チェックで始まる。他のモードにはボックスが無い）、GUI が暗に持つ帯域
を返す: GUI 自身は帯域を持たない（`nfa` はウォーターフォールの左端、`nfb` は右端）ので、
FT8 と FT4 は `jt9` のコマンドラインの 200〜4000 Hz、FST4 は GUI 自身の F Low / F High
である 600〜1400 Hz、他のモードはレジストリの帯域になる。`DecodeParams::for_band` は素の
ブロックである: `Deep`、AP は `Full`、自局も QSO も無し。自局のコールサインが無いと残るのは
盲目の `CQ` 仮説だけなので、`ApMode::Off` である `default_params(Mode::Ft8)` とは同じ
ではない。

**`Depth` は、`ndepth` と同様に、探索設定の全てを決める。** ただしモードの `Tuning`
extra で設定したものは除く（[§2.5](#25-extras-と-decoder-の外にあるプロトコル)）。 `Fast`・`Normal`・`Deep` は
`ndepth` の 1・2・3 で、モードごとに、そのモードの本家デコーダがそれらに対して設定する
ものを、行単位で設定する（引用は各 `Decodable` 実装にある。出典は v3.2.0-rc1 の export で、
2.7 のツリーではない）:

| モード | Fast | Normal | Deep | 本家 |
|---|---|---|---|---|
| FT8 | sync 2.1、候補 1000、OSD なし、フラット SIC 2 ラウンド、nsync 下限 8 | sync 2.1、OSD、`SicEarly`、下限 8 | sync 1.3、OSD、`SicEarly`、下限 6 / 7 | `ft8_decode.f90:175-181`、`ft8b.f90:178-180,430-437` |
| FT4 | sync 1.18、候補 200、1 パス、OSD なし、AP なし | 3 パス、OSD なし | 3 パス、OSD あり | `ft4_decode.f90:31,192-203,323-324` |
| FST4 | minsync 1.20（FST4-15 は 1.15）、候補 200、OSD あり、`i0 ± 1` タイミング再試行なし、AP なし | + 再試行（`jittermax`） | 同じ | `fst4_decode.f90:53,234-248,308-309,421-423` |
| WSPR | `wsprd -qB`: 2 パス、ジッタなし | `-C 500 -o 4`: 3 パス、OSD あり | `+ -d`: 候補が増える | `wsprd.c:819-900`、GUI の `mainwindow.cpp:2824-2826` |
| JT9 | Fano limit 5000 | 10000 | 30000 | `jt9_decode.f90:83-100` |
| JT65 | 2 パス、`nvec` 100 | 2 パス、1000 | 4 パス、1000 | `jt65_decode.f90:110-119` |
| Q65 | `maxiters` 40、`(idf, idt, maxdist)` (1, 1, 4) | 60、(3, 3, 5) | 100、(5, 5, 5) | `q65_decode.f90:183-188`、`q65_loops.f90:27-40` |

**QSO 文脈の AP（FT8・FT4・FST4）。** AP の仮説は、本家の `naptypes` 表
（`ft8b.f90:55-70`、`ft4_decode.f90:132-137`、`fst4_decode.f90:133-138`）を通して
`station`・`qso`・`ap` から導かれる。`station` が設定されていなければ何も走らない。
`nQSOProgress` ごとに 1 周期で試される `iaptype`（1 = `CQ ??? ???`、
2 = `MyCall ??? ???`、3 = `MyCall DxCall ???`、4 / 5 / 6 = タイプ 3 の末尾が
`RRR` / `73` / `RR73`）:

| `qso.progress` | FT8 | FT4、FST4 |
|---|---|---|
| `Calling` | 1, 2 | 1, 2 |
| `Replying`、`Report` | 2, 3 | 2, 3 |
| `RogerReport`、`Rogers` | 3, 4, 5, 6 | 3, 6 |
| `Signoff` | 3, 1, 2 | 3, 1, 2 |

`MyCall` を名指すタイプには標準の `station.call` が、`DxCall` を名指すタイプには標準の
`qso.his_call` が要る（`PJ4/K1ABC` のような非標準の呼出符号は、それらのタイプを除外する）。
タイプ 3 以上は、Rx または Tx 周波数の 50 Hz 以内でだけ走る。`ApMode::CqOnly` はタイプ 1 を
残し、`Off` は何も残さない。FT4 と FST4 は `Fast` では AP を走らせず、Fox（と FT4 の Hound）
も走らせない。盲目の `CQ` はコンテストのトークン（`CQ TEST`、`CQ FD`、`CQ RU`）を使う。
Hound の「950 Hz 未満のみ」は適用していない。自由形式の `ap_hint` extra
（[§2.5](#25-extras-と-decoder-の外にあるプロトコル)）は、設定されるとこの導出を**置き換える**。
Q65 は同じフィールドから代わりにコードワードのリストを導く（`standard_qso_codewords`、
またはコンテスト用のリスト）。`rx_freq_hz` が設定されていれば、それが q3 リストとして使われる。
FT8 の a8 は、ヒントが MyCall、DxCall、相手のグリッドを持ち、Rx 周波数があるときに走る。
a7 は `a7` extra である。

**戦略拡張はファントムデコードの出どころである。** この suite が出荷した偽デコードの
バグ2件はどちらも減算経路にあった（#243 は `__staged_sic`、#253 は `.sic_early()`）。
そのため新しい戦略は、同じ PR で精度ガードと一緒に出荷する。

### 2.3 計算予算

`SlotInput::budget(check)` は、候補の間で呼ばれる呼び出し側の述語
（`&(dyn Fn() -> bool + Sync)`）を取る。**ライブラリは自前の時計を読まない** — 締切は
述語が何と比較するかで決まり、これによって wasm や、スロットの途中で一時停止された
プロセスからも使える。

`SlotResult::budget` は `BudgetReport` で、打ち切りが何をやり残したかを伝える:
スキップした候補数、実行したステージ数、スキップした最良候補の質。これにより呼び出し側は
「何も無かった」と「有望な候補をキューに残したまま時間切れになった」を区別できる。
`rows_subtracted` は、予算が候補以外に費やされる唯一の場所 — FT8 `SicEarly` の
チェックポイント B・C の減算ループ — を表す。`exhausted` が立っていて行数より小さければ、
打ち切りは探索中ではなく後片付けの最中に起きた。減算される行には候補のランキングが
付かないので、上の各フィールドではそれを言えない。

FT8・FT4・全 FST4 サブモードが対応する（`MFSK_CAP_BUDGET` は同じ事実を C へ公開したもの）。
WSPR・JT9・JT65・Q65 は周期全体をデコードし、空のレポートを返す。

### 2.4 ストリーミング配信

`Decoder::decode_with(&slot, &|row: &Row<_>| …)` は、見つかるたびに各行を、呼び出しが
返す `SlotResult` とは別に配信する — 長いスロットが終わる前に何かを画面に出したい UI 向けである。
`AnyDecoder::decode_with` は `&(dyn Fn(&Decoded, &RowDetail) + Sync)` を取る。

配信順と重複排除の契約は**ここでは繰り返さない**: [`STREAMING.md`](STREAMING.ja.md) が
正式な説明である。一行でいえば: 逐次戦略は呼び出しが返す行をそのまま同じ順序で
配信し、並列戦略は完了順に配信し、返される行では既に重複排除済みの一時的な重複を
見せることがある。コールバックに渡される行は、周期の開始時点のハッシュ表で解決されたもの
で、返される行は同じ周期内で先に学習された呼出符号も見る。

全モードが同じメソッドで同じ形を提供する。WSPR のそれは正確な契約ではなく並列の契約で
ある — [`STREAMING.md`](STREAMING.ja.md) §3b を参照。JTTY には `Decoder` が無く、音声呼び出しの
内側からコールバックで配信する: `jtty::rx::Stream::push(samples, &mut |update| …)`（と
`finish`）が呼び出し側のスレッドでそれを呼ぶ —
[§2.5](#25-extras-と-decoder-の外にあるプロトコル)。

### 2.5 Extras と `Decoder` の外にあるプロトコル

**Extras** は、本家に無く本ライブラリが足すものである。各モードの `Decodable::Extras` は、
そのモードが持つオプションだけを保持するので、そのモードに無いオプションは、実行時の
拒否ではなくコンパイルエラーになる（C ABI と `AnyExtras` は、同じ不一致を実行時に
`Unsupported` で返す）。Extras は `Clone + Default` で、`extras_mut()` または
`with_extras(..)` で設定し、周期の合間に変えてよい。

*フレーム系*（`Ft8Extras`、`Ft4Extras`、`Fst4Extras`）は次を共有する:

| フィールド | 型 | 既定 | 効果 |
|---|---|---|---|
| `tuning` | `Tuning<S>` | 全て `None` | ライブラリ独自の探索設定。`Depth` が決めたものを、**設定したときだけ**上書きする: `sync_min`、`max_cand`、`osd`、`strictness`（`DecodeStrictness`、[§6](#6-engine-プリミティブ)）、`strategy`。組込み（候補 15、1 パス）と tier-C スイープが使う |
| `ap_hint` | `Option<ApHint>` | なし | 本家の QSO 文脈 AP とは別の、自由形式の a-priori ヒント: skimmer の「DX を 1 局狙う」場合で、本家は QSO 文脈でしか表現できない。設定すると導出されたヒントを置き換える。両コールサインを固定する仮説は `rx_freq_hz` の 50 Hz 以内の候補にだけ走る（`ft4_decode.f90` / `ft8b.f90` は常に `nfqso` を持つ） |
| `eq` | `EqMode` | `Off` | `Off` / `Local`。探索ではなく**入力音声**の性質である |
| `filter` | `MessageFilter` | `Default` | メッセージの受理、[§2.6](#26-メッセージの受理) |

さらにモードごとに:

| extra | 対象 | 効果 |
|---|---|---|
| `Tuning::strategy` | FT8: `Ft8Strategy::{SinglePass, SicRounds(n), SicEarly}`、FT4: `Ft4Strategy::{SinglePass, SicRounds(n)}`、FST4: `Fst4Strategy::SinglePass` | `match` で振り分け、モードごとに monomorphize される enum なので、選ばれない戦略はコストがゼロ。`SicRounds(n)` はフラットな逐次干渉除去（n は 1..=3 に丸める）、`SicEarly` はチェックポイントエミュレーション（`jt9 -d2/-d3`）で、3 チェックポイント固定の構造。FT4 に `SicEarly` は無く、FST4 に減算は無い（本家に無いことをそのまま写している） |
| `a7` | FT8（`bool`、オフ） | WSJT-X の **a7** リスト復号（`ft8_a7.f90`）。このデコーダ自身の周期 *n* − 2（同じ系列の 1 周期前）の復号結果を入力にする（`SlotInput::period` が要る）。その各組が次に送りうるメッセージを、前回の周波数と DT で照合する（pass id 30） |
| `sniper` | FT8（`Option<Sniper { search_hz }>`、250 Hz） | 下記の roofing filter モード。`rx_freq_hz` が要る |
| `noise_blanker` | FST4（`Option<NoiseBlanker>`） | WSJT-X の **NB**（`blanker.f90`）: スロット FFT の前に最も大きいサンプルをゼロにする。`Percent(n)` は `n` % を消す（0..=25）。`Sweep { step, ftol_hz }` は 0、step、… 20 % でデコードし、0 より上のレベルは `rx_freq_hz` の `ftol_hz` 以内でだけ行う（最大 21 回のデコード）。WSJT-X の既定の NB 0 % と同じくオフ |

**sniper モードを sniper モードにしているのはヒントではなく窓である。** `Sniper` は探索を
`rx_freq_hz` の ±`search_hz` に絞る。これが存在するのは、オペレータが送受信機の
*アナログ* roofing filter を絞り — Yaesu FTDX101MP と FTDX10 が約 500 Hz の代表例 — 搬送波
が既知の局に向けたからである。届く音声は既に帯域制限されており、デコーダはハードウェアに
合わせているだけである。`eq: EqMode::Local` は、そのフィルタのスカートが通過帯域に付ける
傾きを平坦にする。**FT8 だけ**であり、汎用の「既知の 1 局を狙う」便宜機能ではない: それは
広帯域経路の `ap_hint` で、FT8・FT4・全 FST4 サブモードが持つ。FT4 と FST4 の sniper
エントリポイントは 2026-09-13 まで存在したが削除した: 広帯域経路がここでの全モードの
主経路であり、sniper 無しで WSJT-X に忠実でないなら、それは広帯域経路のバグである。
計測を含む全経緯は [`DESIGN_RATIONALE.md`](../notes/DESIGN_RATIONALE.md)。

```rust
use mfsk_core::decoder::{Decoder, DecodeParams, Sniper};
use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
use mfsk_core::ft8::Ft8;
use mfsk_core::msg::ApHint;
use mfsk_core::msg::wsjt77::pack77;

let msg77 = pack77("CQ", "JA1ABC", "PM95").unwrap();
let tones = message_to_tones::<Ft8>(&msg77);
let frame = synthesize_i16::<Ft8>(&tones, 12_000, /* freq */ 1000.0, /* amp */ 20_000);
let mut audio = vec![0i16; 180_000]; // 15 s @ 12 kHz
let start = (0.5 * 12_000.0) as usize;
audio[start..start + frame.len()].copy_from_slice(&frame);

let mut decoder = Decoder::<Ft8>::new(DecodeParams::for_band((200.0, 3_000.0)).rx_freq(1000.0));
let extras = decoder.extras_mut();
extras.sniper = Some(Sniper { search_hz: 250.0 });
extras.eq = EqMode::Local;
extras.ap_hint = Some(ApHint::new().with_call1("CQ").with_call2("JA1ABC"));

let result = decoder.decode(&mfsk_core::decoder::SlotInput::i16(&audio));
assert!(!result.rows.is_empty(), "roundtrip must decode");
for row in &result.rows {
    println!("{:7.1} Hz  {}", row.decoded.freq_hz, row.decoded.text);
}
```

戦略拡張はファントムデコードの出どころである（[§2.2](#22-decodeparams-と-depth)）。
`Tuning::strategy` と `a7` は既定外のコード経路である。

*WSPR・JT9・JT65* は `WsprExtras`、`Jt9Extras`、`Jt65Extras` を取る。3 つとも
`search: SearchTuning`（`time_tolerance_early_sec`、`time_tolerance_late_sec`、
`score_threshold`、`max_candidates`。いずれも `Option` で、ブロックの帯域を載せた
モードの `default_search_params()` の上に重なる）を持つ。WSPR は depth のものを上書きする
`max_cycles_per_bit` を足す（10000 は `wsprd` 自身の既定で、GUI の Normal と Deep は速度の
ため 500 に下げる。実測: WSJT-X の golden では 500 だと −23 dB の G8VDQ を失い、10000 だと
復号する）。JT65 は `chase: Option<ChaseParams>` を足す（既定では Chase デコーダの試行回数は
depth の `nvec`）。

**WSPR** はスロットを wsprd と同じ 375 Hz ベースバンドにデシメートし、そこで wsprd 自身の
粗探索と 3 回のデコードパスを走らせる。共有の FT 系パイプラインとはステージ構成が異なる。
ただし内部で使っている FEC (`ConvFano`) とメッセージコーデック (`Wspr50Message`) は
`Wspr: Protocol` の関連型として宣言済みで、抽象の枠組みからは外れていない — スロット
レベルのデコーダだけが違う。デコーダの表は、前のスロットの Fano 復号が確認した局を OSD が
再び見つけられるようにする。これが `wsprd` が自身のサンプルファイルで −25 dB の W3BI に
届く仕組みである。

```rust
# #[cfg(feature = "wspr")] {
use mfsk_core::decoder::{DecodeParams, Decoder, SlotInput};
use mfsk_core::msg::WsprMessage;
use mfsk_core::wspr::Wspr;
use mfsk_core::wspr::tx::synthesize_type1;

// WSPR Type 1 フレームを合成する（120 s @ 12 kHz スロット）。
let samples_f32 = synthesize_type1("K1ABC", "FN42", 37, 12_000, 1500.0, 0.3)
    .expect("valid message");

let mut decoder = Decoder::<Wspr>::new(DecodeParams::for_band((1400.0, 1600.0)));
let result = decoder.decode(&SlotInput::f32(&samples_f32));
assert!(!result.rows.is_empty(), "roundtrip must decode");
for row in result.rows {
    let d = row.native; // WsprResult
    match d.message {
        WsprMessage::Type1 { callsign, grid, power_dbm } => {
            println!("{:7.2} Hz  {:+.0} dB  {} {} {}dBm", d.freq_hz, d.snr_db, callsign, grid, power_dbm);
        }
        WsprMessage::Type2 { callsign, power_dbm } => {
            println!("{:7.2} Hz  {:+.0} dB  {} {}dBm", d.freq_hz, d.snr_db, callsign, power_dbm);
        }
        WsprMessage::Type3 { callsign_hash, grid6, power_dbm } => {
            println!("{:7.2} Hz  {:+.0} dB  <#{:05x}> {} {}dBm",
                     d.freq_hz, d.snr_db, callsign_hash, grid6, power_dbm);
        }
    }
}
# }
```

**JT9** と **JT65** は同じ形である: `Decoder::<Jt9>` と `Decoder::<Jt65>` に、60 秒周期の
`SlotInput::f32`（または `i16`）を渡す。JT9 は `rx_freq_hz` があれば、本家の Rx 周波数パスも
`tol_hz`（既定 50 Hz）の範囲で走らせる。どちらも 72 ビットメッセージをテキストで返す。
0.13 以降、どちらも全ゼロの符号語（`000AAA 000AAA RA90`）を報告しない。これは無音を
Reed-Solomon または Fano で復号すると出てくるものである。

```rust
# #[cfg(all(feature = "jt9", feature = "jt65"))] {
use mfsk_core::decoder::{DecodeParams, Decoder, Depth, SlotInput};
use mfsk_core::{Jt65, Jt9};

let jt9_audio = mfsk_core::jt9::tx::synthesize_standard("CQ", "K1ABC", "FN42", 12_000, 1500.0, 0.3)
    .expect("pack + synth");
let mut jt9 = Decoder::<Jt9>::new(DecodeParams::for_band((200.0, 4_000.0)).depth(Depth::Deep));
assert!(!jt9.decode(&SlotInput::f32(&jt9_audio)).rows.is_empty(), "roundtrip must decode");

let jt65_audio = mfsk_core::jt65::tx::synthesize_standard("CQ", "K1ABC", "FN42", 12_000, 1270.0, 0.3)
    .expect("pack + synth");
let mut jt65 = Decoder::<Jt65>::new(DecodeParams::for_band((200.0, 4_000.0)));
let result = jt65.decode(&SlotInput::f32(&jt65_audio));
assert!(!result.rows.is_empty(), "roundtrip must decode");
for row in result.rows {
    println!("{:7.2} Hz  {:+.0} dB  {}", row.decoded.freq_hz, row.decoded.snr_db, row.decoded.text);
}
# }
```

JT65 の Chase 探索（`jt65::chase`、issue #169）は WSJT-X の stochastic Chase デコーダ `ftrsdap`
の忠実な移植（マジックナンバーも含む）。AWGN スイープでは 50% 交差を −22.5 dB から −23.5 dB に
下げ、その代わり即座に復号できない候補ごとに最大 `ChaseParams::max_trials` 回の RS 試行を払う。

**JT65 の averaging**（`params.averaging`、`ndepth & 16`。`jt65::averaging`、
`jt65_decode.f90` の `avg65` の移植）。単一周期で復号できなかった候補は保存される: 周期、DT、
周波数、63 × 64 のシンボル電力を、デコーダごとに最大 64 周期。同じ偶奇の保存済み周期のうち、
DT が 0.2 秒以内、周波数が `tol_hz`（既定 50 Hz）以内のものを合算し、2 周期以上あれば、
その和を 1 周期の場合と同じに復号する（Chase 探索。確率は `s1/psum` なので、和を
再スケールする必要はない）。毎回の呼び出しで `SlotInput::period` が要る（無ければ平均しない）。
`clear()` で保存した周期を忘れる。σ = 2.0 の雑音では、単一周期はどれも復号できず、
6 周期目の和が復号する
（`tests/decoder_depth.rs::jt65_averaging_decodes_what_no_single_period_does`）。
未移植: JT65B/C の平滑化ループ（`ismo`）と `nflip`。`deep_search`（`ndepth & 32`、
呼出符号データベースとの相関 `hint65`）は保持するが読まない。

**既知の位置でのデコード。** `wspr::SniperRequest`、`jt9::SniperRequest`、
`jt65::SniperRequest`、`q65::SniperRequest` は公開のまま残る: 呼び出し側が既に持っている
`(start_sample, frequency)` でのデコードは探索ではなく、WSPR ボード（`embedded-shared`）
がその上に作られているからである。`SniperRequest::new(audio, rate, start_sample, freq_hz)`。
WSPR の `::baseband(idat, qdat, …)` は、呼び出し側が既にデシメート済みのベースバンドで同じ
ことをし、`.drift()`、`.nblocks()`、`.confirmed(&WsprCallsignTable)`、`.refine_drift()` を
取る — スキャン自身のパスが候補ごとに設定するつまみである（CoreS3 の WSPR 受信機が自前の
候補ループから駆動するのがこれ）。JT65 のそれは `.chase(..)` または
`.erasures(&[0, 8, 16, 24, 32])` を取る（後から呼んだ方が勝つ）。

**Q65** は 10 個のサブモード ZST で、それぞれ `Decoder<Q65a30>` などになり、最も豊富な
extras（`Q65Extras`）を持つ:

| フィールド | 既定 | 効果 |
|---|---|---|
| `search` | — | 上と同じ `SearchTuning` |
| `ap_hint` | なし | フレーム系と同じ自由形式ヒント。**各候補をまず AP なしで試し**、次にヒント付きで試す。`q65_decode.f90` の `ipass` ループと同じ順で、1 回のヒント付き走査が、plain 走査とヒント付き走査を合わせた結果を返す（`Q65Result::ap` がどちらでデコードしたかを示す。#555 以降） |
| `ap_list` | 空 | AP リスト復号のための候補コードワード（`Vec<[i32; 63]>`）。`station` と `qso` からデコーダが作るリストの代わりになる |
| `callers` | なし | `Q65Callers`、聞こえたコンテスト局（`q65_hist2`）。`Contest::GridExchange` のときリストに加わる |
| `pileup` | `false` | Q65 Pileup（WSJT-X 3.2）: Pileup モードの局は、相手の直前の送信を受信できたことを予備の 78 ビット目で知らせる。`Q65Result::copied_last_tx` がそれを報告し（WSJT-X は行に `#` を付ける）、`encode_channel_symbols_flagged` / `synthesize_standard_flagged_for` で送れる。`pileup` を有効にすると、両方の呼出符号を指定した `ap_hint` がこのビットを 0 に固定せず自由にするので、フラグ付きの応答にも一致する。`ap_list` のテンプレートは `q65_set_list.f90` と同じくビットを立てない |
| `max_drift` | `0` | WSJT-X の Max Drift（0..50 ビン）。同期探索で、フレーム全体にわたる最大 `max_drift` ビンの直線的なトーンのドリフトを試し（`q65_ccf_22`）、グリッドデコードで見つかったドリフトを取り除く（`q65_loops` の `twkfreq`）。周波数ビンあたりの探索コストは通常の `2*max_drift+1` 倍。WSJT-X は有効な間、探索窓を Rx ± F Tol に絞るので、`band_hz` も同じように絞ること。通常と `ap_hint` のスキャンだけ |
| `fading` | なし | `(FadingModel, b90_ts)`: 呼び出し側が選ぶモデルによる高速フェージング指標。ドップラー拡散チャネル（マイクロ波 EME、10 Hz 以上の拡散）向け |

どのフロントエンドが走るか（`q65/decode_request.rs`。`.decode()` は
`ap_list > fading (+ ap_hint) > ap_hint > plain` の順で解決し、`ap_list` と `fading` は
エンジン内で排他）。Q65 の `Decoder` には、ブロックによって選ばれる 3 つの経路がある:

| 状況 | 戦略 | 方法 | 閾値の利得 |
|---|---|---|---|
| 既定のスキャン | `(Δf,Δt,b90)` グリッド + Lorentzian フェージング BP、労力は [§2.2](#22-decodeparams-と-depth) の depth | 何も設定しない | WSJT-X 忠実な既定 |
| コールサインやレポートが既知、地上波 | AP ヒント BP | `extras.ap_hint` | 約 2 dB |
| ドップラー拡散チャネル | 高速フェージング指標 + BP | `extras.fading` | 拡散チャネルで 5–8 dB |
| コールサイン対は既知、QSO 状態は無し | AP リストのテンプレート照合 | `extras.ap_list` | 約 3 dB |
| コールサイン対と受信周波数が既知（WSJT-X の q3） | 受信周波数付近にあるリストの全メッセージの 85 シンボル sync を取り、そのあとリスト復号 | `station`（＋ `qso`）と `rx_freq_hz`（＋ `tol_hz`）、または `rx_freq_hz` と `extras.ap_list` | `q65sim` Q65-30A、−24 / −26 / −28 / −30 dB の各レベル 20 ファイル: 20 / 20 / 7 / 2、`jt9 -3 -d 1` も同じファイルで同じ |
| 複数周期にまたがる微弱・電離層散乱信号 | シンボルスペクトルの移動平均（`averaging`） | `params.averaging = true` と連続する `SlotInput::period` | 単一周期のどの戦略でも取れない信号を拾う |

**Q65 の平均**は、別個のリクエスト（0.12 の `MultiPeriodRequest`）ではなくデコーダの状態
である: `averaging` を有効にすると、各周期が移動平均（`s1a`、重み `1/min(navg, 4)`、3 段
カスケード。直近 `decoder::MAX_AVERAGED_PERIODS` = 8 周期を保持し、それより古いものの重みは
`0.75^8` ≈ 10 % 以下）に加えられ、`period` に欠落があるか `None` だと平均はやり直しになる。
1 周期につき結果は 1 つで、q3 が当たった周期ではラダーを飛ばす（本家は続けて候補ループに入る）。
平均経路が取るのは探索チューニングと AP リスト / q3 で、`ap_hint`・`fading`・`pileup`・
`max_drift` は取らない。

**Q65 の q3 リスト復号。** `rx_freq_hz` とリスト（`station` と `qso` から作るか、
`extras.ap_list`）があるとき、`tol_hz`（既定は `jt9` CLI と同じ 10 Hz）は WSJT-X の q3
デコードである。受信周波数の F Tol 以内で、リストの各メッセージの 85 シンボル全部を使って
同期を取り（`q65_ccf_85`）、高速フェージング指標で `b90` を掃引しながらリスト復号する
（`q65_dec_q3`）。これを最初に実行し、その後で帯域の残りをスキャンする。`max_drift` 50 の
ときは、受信周波数で何も復号できなければ、そこで見つかったドリフトを取り除いたスペクトル
でもう一度実行する（"w3sz" の段階 5）。**`ap_list` が現れるすべての箇所が 1:1 移植というわけ
ではない**（issue #522）: 本家のリスト復号は常に受信周波数を条件とする q3 としてしか動かない
ので、`rx_freq_hz` を付けない `extras.ap_list` は crate 独自の候補ごとの AWGN 指標による
テンプレート照合で、本家に対応物は無い — 意図的な拡張であり忠実性のギャップではない。

`q65::Q65History` は WSJT-X の `q65_hist` で、アプリケーションが保持する。
デコードのたびに `.record(&result)` で記録し（最新 100 件を保持）、
`.lookup(rx_freq_hz)` は 10 Hz 以内の最新のデコードから DX コールを返す
（メッセージにグリッドがあればグリッドも返す）。WSJT-X は DX コール未入力で
手動の Decode Again を行ったときにこれを使い、オペレータがコールを入力しなくても
フル AP リスト（`standard_qso_codewords`。`extras.ap_list` に渡す）を作る。
`q65::Q65Callers` と `contest_codewords` はコンテストモード版である
（`q65_hist2` / `q65_set_list2`）。グリッド付きで呼んできた局を最大 50 局、
アプリケーションが保持する（`record(freq, msg, now)`、`expire(now)`）。
そこから、各局について `MyCall Caller Grid` / `R Grid` / `RRR` / `RR73` /
`73` を 78 ビット目なしとありの両方で作ったフル AP リストを作る。各フロントエンドが実際に何をして
いるか、既定のスキャンがなぜ素の Bessel パスではないのかは
[`DESIGN_RATIONALE.md` §4](../notes/DESIGN_RATIONALE.md#4-q65s-decoder-strategies-and-what-each-is-for)。

**Q65 の時間窓と EME 遅延。** `default_search_params()` は公称開始の
-1.0 .. +1.0 s を探索する。WSJT-X の GUI と同じである（`q65.f90` の
`lag1`/`lag2`）。`eme_delay` は "Decode at 52 s" の EME 遅延に当たり、月面反射の往復分
として後ろ側の端を +5.5 s（Q65-15 は +4.0 s）に広げる。`dt_sec` は公称開始からの値である。

```rust
# #[cfg(feature = "q65")] {
use mfsk_core::decoder::{DecodeParams, Decoder, Depth};
use mfsk_core::q65::Q65a30;

// QSO 中の Q65-30A 局: Rx 周波数と許容幅が q3 リスト復号の条件になり、
// 平均は毎回の呼び出しで `SlotInput::period` を要する。
let params = DecodeParams::for_band((200.0, 3_000.0))
    .depth(Depth::Deep)
    .rx_freq(1_000.0)
    .tol(10.0)
    .station("K1ABC", "FN42")
    .qso("JA1XYZ", "PM95", mfsk_core::decoder::QsoProgress::Report)
    .averaging(true);
let mut decoder = Decoder::<Q65a30>::new(params);
decoder.extras_mut().max_drift = 0;
assert!(decoder.params().averaging);
# }
```

**`dt_sec`、`SearchParams`、`SyncCandidate`。** WSPR・JT9・JT65・Q65 では、
結果の `dt_sec` は**公称開始位置** — モードの `tx_start_offset_s`
（WSPR は固定の 1.0 s `TX_START_OFFSET_S`） — から測り、符号付きなので、
早く始まったフレームは負になる（#397。以前は Q65 と JT65 でバッファ先頭から
測っており、0.5 s 早いフレームが −0.013 s と +0.442 s になっていた）。
`Q65Result` / `Jt65Result` / `Jt9Result` の `to_decoded` はこのフィールドを読む。
スキャン系のモードは
`engine::search` にある 1 つの粗探索の語彙を共有する（#394）:
`SearchParams { freq_min_hz, freq_max_hz, time_tolerance_early_sec,
time_tolerance_late_sec, score_threshold, max_candidates }`（窓は Q65 のそれが
非対称なので**秒**の early/late の組である。`SearchParams::symmetric(..)` は
両方を設定する）と `SyncCandidate { start_sample,
freq_hz, score }`。`freq_hz` は tone 0 で、`.dt_sec(nominal, rate)` が
変換する。`SearchParams::default()` は無い: 既定値はモード固有のデータなので、
各モードが `search::default_search_params()` を持つ
（Q65: 200-3000 Hz、±1.0 s、8 候補、threshold 0.1）— `SearchTuning` が上書きするのはこれである。
FT8・FT4・FST4 は、
`start_sample` の代わりに `dt_sec` を持つ `engine::sync::SyncCandidate` を
意図的にそのまま使う。

**uvpacket** は `Decoder` の外に独自の送信器と受信器（`uvpacket::tx`、`uvpacket::rx`）を
持つ: FEC の母符号だけを再利用する、WSJT 由来ではない応用例である。詳細は
[`UVPACKET.md`](UVPACKET.ja.md) にある。

**MSK144** も設計上 `Decoder` の外にある: FSK ではなく、`msk144::decode::decode_slot` は
`engine::pipeline` を迂回する。T/R 周期全体を走査してピングを探す。

**JTTY**（WSJT-X 3.2.0-rc1 の微弱信号キーボードチャット用モード。`lib/jtty/` の
移植で、#477 と `docs/notes/JTTY_UPSTREAM.md` で管理）は、ここで唯一 **スロットを持たない**
モードです。31.25 ボーの 4-GFSK、1.888 秒のフレーム（59 シンボル: 同期 13 + データ 46）が
いつでも始まり、メッセージは複数フレームからなり、受信側が組み立てます。C・Kotlin・Swift からは
受信器のハンドルで、`mfsk_jtty_*`・`MfskJttyReceiver`・`JttyReceiver` が
[`BINDINGS.md` §2.8.1](BINDINGS.ja.md#281-jtty--スロット呼び出しではなく受信器ハンドル) にあります。そのため MSK144 と同様 `Protocol` と
`PROTOCOLS` の外にあり、受信器は *逐次入力* です。`jtty::rx::Stream` は任意サイズの
チャンクで音声を受け取り、メッセージ更新を `push` の内部で呼び出し元スレッド上の
コールバックへ渡します（`parallel` 有効時は rayon のプールも使います。結果はスレッド数に
依存しません）。背後の `Receiver` は不変テーブルだけを持ち `Arc` で共有できるので、
音声チャンネルごとに `Stream` を作ってもバッファ 1 つ分のコストです。

```rust
# #[cfg(all(feature = "jtty", feature = "fft-rustfft"))] {
use std::sync::Arc;
use mfsk_core::jtty::rx::{Params, Receiver, Stream};
use mfsk_core::jtty::source::{Atom, CallAction};
use mfsk_core::jtty::tx;

// 1 フレーム "CQ K1ABC"。送信の最後のフレームには end-of-message が立つ
let atoms = [Atom::call(CallAction::Cq, "K1ABC")];
let tones = tx::tones(&atoms).expect("encodable");
let mut audio: Vec<i16> = vec![0; 12_000];                 // 頭に 1 秒の無音
audio.extend(tx::synth_f32(&tones, 1500.0, 3000.0).iter().map(|&x| x as i16));
audio.extend(std::iter::repeat(0).take(4 * 12_000));       // 終わるまでの余白

let mut stream = Stream::new(Arc::new(Receiver::new()), Params::default());
let mut updates = Vec::new();
for chunk in audio.chunks(4096) {                          // チャンクサイズは任意
    stream.push(chunk, &mut |u| updates.push(u));
}
stream.finish(&mut |u| updates.push(u));                   // 未完のものを吐き出す
assert!(updates.iter().any(|u| u.text.contains("CQ K1ABC")));
# }
```

**送信**はテキストから始まる。`jtty::pack::pack(text, profile)` は upstream の
`pack_jtty` で、テキストを正規化し（大文字化、`~` と NUL は空白、空白は 1 個に畳む、
64 文字のアルファベット外は `#`）、動的計画法で**フレーム数が最小**になるよう選ぶ —
アクション付きのコールサイン（`CQ K1ABC CQ`）、制御フレーズ（`TU`）、数値、グリッド、
`599 <地名>`、Field Day の `3A EMA` は 1 フレーム、それ以外は 5 文字で 1 フレーム。
`ExchangeProfile::RttyRoundup` はシリアル番号と州の候補を加える（`599 5` は `599 005`
として送られる）。他のプロファイルは同じ結果になる。切り詰めずに拒否する: 80 文字超、
16 フレーム超、収まらないシリアルは `PackError`。結果は `jtty::tx::tones` と
`synth_f32` が取る形。upstream 自身の `pack_jtty` と 3 525 メッセージ × 交換プロファイルで
突き合わせている（`tests/jtty_pack.rs`）。毎回同じフレームになる。WSJT-X の GUI がその外側に持つもの — F キーテンプレート、
N1MM タグ、運用状況からのプロファイル判定 — はホスト側の方針でライブラリには無い
（QSO 状態に依存するデコーダについて #463 が同じ線を引いている）。

```rust
# #[cfg(feature = "jtty")] {
use mfsk_core::jtty::pack::{self, ExchangeProfile};

let tones = pack::tones("cq k1abc cq", ExchangeProfile::Unknown)
    .expect("packs")
    .expect("not empty");
assert_eq!(tones.len(), 59);                       // 1 フレーム
assert_eq!(pack::pack("HELLO WORLD", ExchangeProfile::Unknown).unwrap().len(), 3);
# }
```

`Params` は `rjtty` が取るものを持ちます。運用者の受信周波数と許容幅（チャンネル 0）、
同期の下限 `smin_db`、チャンネル 1・2 が監視する帯域（運用周波数外の局を
1350 ± 150 Hz と 1650 ± 150 Hz で探します）。`.subtract`（既定 on）はデコードした
フレームを信号から引き、それ以前のウィンドウを探し直します。強い局の下の弱い局が
取れるのはこのためで、off にすると単一信号の受信器になります。`Receiver::scan` /
`scan_messages` は録音全体を一度に処理する形で、同じサンプルを逐次に流した結果と
完全に一致します。`MessageUpdate` はメッセージが伸びるたびに 1 回、完了（`complete`）
または打ち切りで最後に 1 回出ます。`id` はメッセージの存続中変わりません。基準機での
実測は 0.47 秒ウィンドウあたり 1 スレッドで 11.7 ms（実時間の 43 倍）。既知の限界
（upstream の受信器と共通、#488）: 約 12〜16 Hz/s を超える周波数ドリフトは追従せず、
低軌道衛星の最接近付近ではこれを超えうる。

**SNR の比較可能性。** `Jt65Result::snr_db` と Q65 のそれは WSJT-X の
2500 Hz 基準帯域に換算されている。`Jt9Result::snr_db` は**されていない** —
JT9 の多段 AGC/IFFT/コヒーレント加算パイプラインは単純な帯域幅オフセットに
還元できないため、相対値専用である（JT9 のデコード同士で比べること。
他プロトコルの `snr_db` と比べないこと）。WSPR のそれは `wsprd` 較正の
候補 SNR で、`wsprd` 自身がスポットの隣に表示するのと同じ数値である。

### 2.6 メッセージの受理

この層より下は全て誤り*検出*である — LDPC のパリティ検査、そして CRC。
どちらも、出てきたものが誰かが実際に送ったメッセージかどうかについては
何も言わない。CRC-14 の偽陽性とは、デコーダが収束した符号語が送信された
ものではなかった場合であり、その 77 情報ビットは実質的に一様で、その半分
以上が構文的に妥当なメッセージに展開される。

`MessageCodec::is_plausible` がそれを拒否する。**本家に対応物は無い** —
`ft8b.f90` は `nbadcrc` と `nharderrors <= 36` だけで受理しており、本
クレートの上限もその同じ 36 である。つまりこれは移植ではなく判断であり、
判断はバンドを知っている側のものである。

**判定はテキストではなくフィールドに対して行う。** `unpack` は
`Wsjt77Fields`（復号済みフィールドとしてのメッセージ）を返し、判定は
*コールサイン欄*を見る。レンダリング済み文字列に問いを立てると、トークンに
割り直して「どれがコールサインだったか」を推測することになる。
`JA1ABC 3Y0Z 6A EMA` をその方式で判定すると `6A` と `EMA` をコールサイン
文法に掛け、`JA1ABC PM95 20` なら `PM95` と `20` を掛ける。issue #383 まで
実際にそうなっており、ARRL RTTY Roundup・free text・telemetry を丸ごと
拒否し続けていた。テキスト規則が検査しそうな他のもの（ARRL セクション番号、
グリッドの境界、RTTY 交換の範囲）は、既に復号時に強制済みである。

コールサインを持たない型が2つあり、名指しで扱う。free text と telemetry は
冗長性が皆無（ほぼ全ビット列が妥当な値）なので受理し、EU VHF contest は
ハッシュ2つしか持たないので、どちらかが解決したときだけ受理する。

extra は 1 つ、`Ft8Extras`・`Ft4Extras`・`Fst4Extras` の `filter: MessageFilter` — つまり
**フレーム系の全モード** — で、値は 4 つある:

| 値 | 判定 |
|---|---|
| `MessageFilter::Default` | プロトコルが既定で codec の判定を走らせるなら（FT8 と FT4。FST4 は走らせない）それ、走らせないなら判定なし |
| `MessageFilter::Codec` | codec の判定のみ — 既定でオフのプロトコルで有効化する一行の手段 |
| `MessageFilter::AlsoAccept(f)` | codec の判定 **＋** `f` が受理するもの。判定を広げるだけで、減らすことはない |
| `MessageFilter::Only(f)` | 判定を `f` で丸ごと置き換える |

`f` は `fn(&Wsjt77Fields) -> bool` — 関数ポインタなので、extras は `Clone` かつ `'static`
のままである（何もキャプチャしないクロージャはこれに型強制される）。

```rust
use mfsk_core::decoder::{DecodeParams, Decoder, MessageFilter, SlotInput};
use mfsk_core::ft8::Ft8;
use mfsk_core::msg::wsjt77::Wsjt77Fields;

/// 配備先が知っていて ITU 許可リストが知らないもの。メッセージは
/// フィールドとして見え、`callsigns()` は厳密にコールサイン欄だけで
/// あり、grid や report が紛れ込むことはない。
fn special_event_only(m: &Wsjt77Fields) -> bool {
    m.callsigns().all(|c| c.starts_with("8J"))
}

let audio = vec![0i16; 180_000]; // 15 s @ 12 kHz
let params = DecodeParams::for_band((200.0, 3_000.0));

// codec の判定 ＋ それが知らないコールサイン。
let mut widened = Decoder::<Ft8>::new(params.clone());
widened.extras_mut().filter = MessageFilter::AlsoAccept(special_event_only);

// 一切の判断をしない — CRC を通ったメッセージは全部、ファントム込みで。
// これが本家の受理規則そのものである。
let mut unfiltered = Decoder::<Ft8>::new(params);
unfiltered.extras_mut().filter = MessageFilter::Only(|_| true);

// 無音には実信号も CRC 生存者も無いので、フィルタ無しの方も空で返る。
assert!(widened.decode(&SlotInput::i16(&audio)).rows.is_empty());
assert!(unfiltered.decode(&SlotInput::i16(&audio)).rows.is_empty());
```

`MessageFilter::Only(f)` が置き換える側の判定は、そこに到達した CRC 生存者のおよそ 2/3 を
落としているので、緩い `f` はファントム行を表に出す。

**FT8 は、ポリシーに関わらず `/R` と `TU; ` のメッセージも落とす。** #439 以降、
FT8 は `ft8b.f90`（WSJT-X 3.0 以降）が CRC の直後にすることと同じことをする:
コンテスト中でなければ、`/R` を含む、または `TU; ` で始まる標準または RTTY
Roundup のメッセージは捨てられ、そのパスは次へ進む。これはポリシーより前に
あるので、`MessageFilter::Only(|_| true)` でもそれらの行は戻らない。戻すのは
ブロックの `contest` が `Contest::None` 以外であることで、`CALL1/R CALL2` や `TU; CALL1 CALL2` が実トラフィック
になるコンテストではこれが正しい設定である。

**既定でオンなのは FT8 と FT4。理由は減算である。** 受理したものを減算する
経路では、誤ったデコードは表示上の問題では済まない。`SicRounds` と
`SicEarly` は、次に探す前に復号した波形を音声から取り除く。
`qso3_busy.wav` での実測では、判定を切ると `SicEarly` がファントム
`CQ G47OXF RD84` を受理して減算し、その下にいた実信号 `CQ EA2BFM IN83` を
失う — 18/18 が 17/18 になる。単一パス経路では、同じ判定が `max_cand = 200`
で 2 行のゴミを落とし、**実機が使う深さでは 1 件も落とさない**。

FT4 も同じ CRC-14 と同じ SIC 経路を持つので、自前の実測が揃った時点で
同じ扱いになった — `ft4sim` コーパスの閾値帯（−21..−13 dB、ITU-R 4チャネル）
720 スロットで、ファントム行 **7 → 2**、golden 行 **353 → 354**、
50% 交差 SNR は4チャネルとも **0.00 dB 変化なし**。動いた唯一の recall セルは
*増える*向きで、拒否されても候補ラダーが止まらないため。このコーパスが
試せないのは許可リスト自身のリスク（全スロットが同一コールサイン）なので、
珍しいプレフィクスを受ける運用は `MessageFilter::AlsoAccept` で広げる。

**FST4 はオフのまま。** 測っていないからではない: CRC-24 により偽陽性率が
他の2つより 512 倍低く、判定が落とすものがほとんど無い一方、recall を失う
可能性だけは同じだからである。

**未使用時のコストはゼロ。** ポリシーは行コールバックや予算のような `&dyn Fn` ではなく、
エンジンの型パラメータである。`MessageFilter` の各バリアントがそれぞれ自前の monomorphize
されたコピーを選び、`Default` はゼロサイズ型 `DefaultPolicy` を持ち、既定でフィルタしない
プロトコルでは**メッセージの復号すら行われない** — どちらの条件もそのコピーの中ではコンパイル時
定数である。行コールバックは*デコード*ごとに1回発火するが、こちらはメッセージ段に到達した
候補ごとに発火する。型パラメータにする価値があるのはそのため。

---

### 2.7 広帯域 IQ 入力

`mfsk_core::iq` は、これまでのデコーダが 12 kHz の実数音声を受け取るところへ、広帯域の複素 IQ
ストリーム（SDR、IQ 録音）を受け取ります。ライブラリの範囲は DSP とデコードまでで、デバイス制御、
UI、スポット送信は含みません。**信号を探すことはしません**。どのダイヤル周波数がどのモードかは
呼び出し側が指定し、チャンネル内の音声は、デコーダが自分の `DecodeParams` の帯域で探索します。

**1 チャンネル: `IqToAudio`**。12 kHz 以上の任意の整数レートの IQ から、そのダイヤル周波数について
送受信機の USB 出力が運んだはずの音声を作ります。

```rust
use mfsk_core::iq::{IqError, IqSampleFormat, IqStream, IqToAudio};

let stream = IqStream::new(768_000, 14_200_000.0, IqSampleFormat::Cf32);
// 14.290 MHz は中心から 90 kHz 上: 帯域内で、DC から離れている。
let mut ch = IqToAudio::new(stream, 14_290_000.0).unwrap();
let mut audio = Vec::new();
ch.push_cf32(&[0.0; 2 * 1024], &mut audio);   // I/Q インターリーブ。12 kHz の f32 音声を追記する
assert_eq!(ch.samples_in(), 1024);
// チャンネルの 0〜6 kHz の窓の中に DC があれば拒否される。
assert_eq!(IqToAudio::new(stream, 14_199_000.0).err(), Some(IqError::TooCloseToDc));
```

配置は最初に検査されます。チャンネルの音声 0〜6 kHz の窓が `±Fs/2` の内側にあること
（`IqError::OutsideBand`）、ストリームの DC を含まないこと（`TooCloseToDc`）、レートが小さな有理比で
12 kHz に届くこと（例えば 999 983 Hz は `UnsupportedRate`、12 kHz 未満は `RateTooLow`）です。
経路は次のとおりです。チャンネルの音声 3 kHz を DC に混合し、短い FIR デシメータのカスケードで
24〜48 kS/s まで下げ、ポリフェーズの `L/M` リサンプラで正確に 12 kHz の複素信号にし、そこで鋭い
ローパスを 1 段かけ、元の位置へシフトし戻して、実部を取ります。

**使える音声はおよそ 200 Hz から。** 実部を取ると、ダイヤルより*下*の側波帯が必要な側波帯に折り返します。
そのためローパスは、ナイキストではなく音声 0 Hz のところで鋭くなければなりません。通過は音声
200〜5800 Hz、阻止は −200 Hz です。0〜200 Hz の信号は減衰し、確実には復号できません。SSB 受信機自身の
フィルタもこの辺りから始まります。複素入力をデコーダへ直接渡す方式ならこの制約は無くなりますが、
このフロントエンドよりはるかに大きな変更になります（issue #534）。

**N チャンネル、UTC 基準: `IqReceiver`**（FFT バックエンドが必要）。チャネライズとスロット切り出し
だけを行い、**何もデコードしない**。どのダイヤル周波数がどのモードかは呼び出し側が指定し、
ストリームの UTC を伝え、完了したスロットを所有権付きの `CompletedSlot` として引き出す。
デコードは呼び出し側の仕事で、チャンネルごとに 1 つの `AnyDecoder` を使う — チャンネルごとに
独自のオプションと独自のコールサイン表を持ち、デコードは任意のスレッドで走らせられる
（スロットは所有権付きで `Send`）。

```rust
# #[cfg(all(feature = "ft8", feature = "ft4", feature = "fft-rustfft"))] {
use std::collections::HashMap;
use mfsk_core::Mode;
use mfsk_core::decoder::AnyDecoder;
use mfsk_core::iq::{IqReceiver, IqSampleFormat, IqStream};

let mut rx = IqReceiver::new(IqStream::new(768_000, 14_200_000.0, IqSampleFormat::Cf32));
let ft8 = rx.add_channel(14_074_000.0, Mode::Ft8).unwrap(); // 窓に DC が入る、または帯域外なら Err
let ft4 = rx.add_channel(14_080_000.0, Mode::Ft4).unwrap();
// チャンネルが多いときは 1 つのポリフェーズフィルタバンクを共有する（下の「2 つのチャネライザ」）:
//   IqReceiver::with_channelizer(stream, Channelizer::Pfb)?
let mut decoders = HashMap::from([
    (ft8, AnyDecoder::with_defaults(Mode::Ft8)), // チャンネルごと: 独自のオプションとハッシュ表
    (ft4, AnyDecoder::with_defaults(Mode::Ft4)),
]);

// rx.set_time(utc_ns, at_sample);   // 時刻の読みが得られるたびに
let mut slots = Vec::new();
rx.push_cf32(&vec![0.0f32; 2 * 4096], &mut slots); // I/Q インターリーブ。push_cs16 / push_bytes もある
for slot in slots {
    let out = decoders.get_mut(&slot.channel).unwrap().decode(&slot.input());
    for d in &out.rows {
        println!("{:.1} Hz  {}", slot.abs_freq_hz(d.freq_hz), d.text);
    }
}
let report = rx.retune(14_201_000.0); // RetuneReport { paused, resumed }
assert!(report.paused.is_empty());
rx.gap(1_000); // サンプルの欠落
# }
```

`Mode` は `registry::Mode`（0.12 の `iq::IqMode` を置き換える）で、そのビルドが持つ範囲の
FT8、FT4、FST4 の 5 周期、WSPR、JT9、JT65、Q65 の 10 サブモードである。`CompletedSlot` は
`channel`、`mode`、`dial_hz`、`period`（グリッド上のスロット番号）、`start_sample`、
`utc_ns`（時計があるとき）、`audio: Vec<f32>` を持つ — スロットの名目開始位置からの 12 kHz
音声なので、`dt` は WAV のときと同じ読みになる。`slot.input()` は `SlotInput`（音声と
`period` で、平均と a7 が連続するスロットを見られる）、`slot.abs_freq_hz(audio_hz)` は
ダイヤル + 音声周波数である。公開の IQ 型は `#[non_exhaustive]` である。

*時間*: サンプル数が時計で、与えない限り時刻源は読まない。チャンネルの音声インデックス `k` は
サンプル 0 から `k/12000` 秒後で、周期 `T` のモードのスロット `j` は UTC の `[j·T, (j+1)·T)`
を覆う（整数で計算）。`set_time(utc_ns, at_sample)` は時計の観測値（Unix エポックからの
ナノ秒。`at_sample` は入力サンプル数で、`samples_in()` と同じ数え方）を受け取り、
`slotgrid::SampleClock` を通じて上限付きの速度で追従するので、水晶やホストの時計がドリフト
してもスロット境界はミリ秒単位で動くだけで、スロットは失われない。観測が引き起こした
`ClockChange` を返す: `First`（時計が設定された）、`Slewed { by_ns }`（範囲内。前回の観測
からの時間の最大 400 ppm、FT8 のスロット 1 つで 6 ms 動く）、`Stepped { by_ns }`（1 秒より
離れている: ドリフトではなく時計が設定し直されたもの。開いているスロットはその跳びを
またぐので捨てられる）。観測が無ければグリッドはサンプル 0 から自走し、録音の再生に向く。
スロットは常に自身の境界から始まる: ストリームが時計よりわずかに速いときは、最初の数サンプルが
前のスロットの最後のサンプルになる。スロットは全部届いてから完了し、ストリームが途中から
始まったときの部分スロットは完了しない。スロットの最後の音声サンプルは、それを運ぶ最後の IQ
サンプルの数フィルタ長後に出てくるので、録音には、ライブのストリームと同じように、終端の後に
少し余白が要る。

`mfsk_core::slotgrid` はその算術だけを取り出したもので、整数のみ、`std`・確保・アトミック
なしなので、組込みボードや C ABI の音声ストリームにも収まる: `SlotGrid::new(period_ns, rate_hz)`
（`start_of`、`next_start`、`follow`）、`SampleClock`（`observe`、`utc_of`、
`with_max_slew_ppm`、`with_step_ns`）、`SlotCutter<T>`（グリッドのレートでサンプルを
与えると、時計のアンカーに追従しながら各スロットを自身の境界で返す）。+13 ppm で 1 日、観測
値に 3〜11 ms のジッタを乗せたシミュレーションでも、スロットは 1 つも失われない
（`slotgrid::tests`）。

*再チューンと欠落*: `retune(center_hz)` は新しい中心周波数に収まるチャンネルをすべて動かし、
収まらないものを**一時停止**して、`RetuneReport { paused, resumed }` を返す。一時停止した
チャンネルはダイヤルを保ち、呼び出し側はデコーダとその表を保ち、後の再チューンでチャンネルが
帯域内に戻れば再開する（`channel_state(id)`: `Active` または `Paused(IqError)`）。`retune` と
`gap(lost)` はどちらも開いているスロットをすべて捨てる — 中心の変更やサンプルの欠落を
またぐ音声はスロットではない — そしてサンプル時計は続く。

*スレッド*: スロットはそれを完了させた `push_*` 呼び出しから返され、呼び出し側の好きな場所で
デコードされる — 混んだ FT8 では数百ミリ秒かかるので、ブロックできない呼び出し側はワーカーに
渡す。各スロットはデコーダに渡す前に固定の RMS に揃えられる。デコーダはスケールに依存せず、
IQ 自身のレベルを引き継ぐ理由がないためで、無音または NaN のスロットは返されない。

*形式*: `Cf32` と `Cs16` は型付きでもバイト列でも、`Cs8`（HackRF）、`Cu8`（RTL-SDR、128 = ゼロ）、
`Cs24` は `push_bytes` のバイト列で受け取ります。呼び出しをまたいで分割されたサンプルは引き継がれます。

*選択度*: チャンネルの音声 −200〜6200 Hz の外なら、帯域内のどこでも 120 dB（`iq::REJECT_DB`）。
すべてのフィルタをこの値の Kaiser 設計にしており、768 kS/s の理想 16 bit ADC の 2500 Hz 帯域での
ノイズフロアに相当します。各デシメータの折り返し端を狙った点を含め、レートごとに約 1150 点の妨害波位置で
実測した最悪値は、`Direct` 経路で 192 kS/s が −124.0 dB、768 k が −121.0、2.4 M が −122.0、`Pfb` 経路で
5 レート（窓をサブバンド全体に渡って動かし、各約 1000 点）とも −124.3〜−124.4 dB です。
`tests/iq_front_end.rs` と `iq::pfb` のユニットテストがこれを検査します。

*2 つのチャネライザ*: `IqReceiver::new` は `Channelizer::Direct` を使います。チャンネルごとに
`IqToAudio` を 1 つ置き、それぞれが入力レートから混合・間引きするので、コストはチャンネル数に比例します。
`IqReceiver::with_channelizer(stream, Channelizer::Pfb)` は、代わりに 1 つのポリフェーズフィルタバンク
（`iq::PfbChannelizer`）を全チャンネルで共有します。2 倍オーバーサンプルで、サブバンドは約 24 kHz 間隔、
各チャンネルの後段は、窓に最も近いサブバンド上の `IqToAudio` です。どちらもデコーダには同じ選択度の同じ
音声を渡し、IQ のデコードテストはすべて両方を通って WAV 経路と同じ集合になります。実測（1 スレッド、
release、`Cf32` 入力、1 コア比 %）:

| チャンネル数 | 768 kS/s Direct | 768 kS/s Pfb | 2.4 MS/s Direct | 2.4 MS/s Pfb |
|---:|---:|---:|---:|---:|
| 1 | 0.92 | 2.66 | 2.38 | 9.08 |
| 4 | 3.68 | 3.34 | 9.51 | 9.78 |
| 8 | 7.40 | 4.32 | 19.13 | 10.77 |
| 32 | 29.89 | 10.22 | 76.51 | 16.64 |
| 128 | — | 34.94 | — | 41.15 |

損益分岐はどちらのレートでも約 4 チャンネルです。アマチュアバンドの数モードなら `Direct` で足り、帯域全体の
スキマーには `Pfb` が向きます。バンクには 40 kS/s 以上のレートが必要です（それ未満は `UnsupportedRate`）。
設計と、マスク付き FFT（ビン間で −71 dB 漏れた）ではなくポリフェーズバンクにした理由は
`docs/notes/IQ_CHANNELIZER.md` にあります。

*根拠*: `tests/iq_front_end.rs` と `tests/iq_receiver.rs` は、実録音を両側波帯の IQ として（下側波帯が
漏れれば余分な復号として現れる）48 k、192 k、250 k、768 k、2.4 MS/s、I/Q 反転、DC から外した配置で
置き、WAV 経路と同じ復号集合を要求します。FT8 16/16 と 14/14、FT4 11/11 で、余分な復号は無く、周波数は
2 Hz 以内、DT は 0.05 秒以内です。`tests/iq_receiver_modes.rs` は WSPR（9/9）、JT9（5/5）、JT65、
Q65-120D と -300A について同じことを確認します。C ABI は
[`mfsk_iq_*`](BINDINGS.ja.md#282-広帯域-iq--sdr-ストリーム用の受信器ハンドル) です。

### 2.8 非標準・複合・接尾辞付きコールサイン

77 ビットのメッセージが持てる完全な非標準コールサインは高々 1 つで、残りはハッシュで運ばれる。このクレートは、そうしたメッセージを WSJT-X v3.2.0-rc1 と
まったく同じにアンパックする。issue #568 で、WS（旧 WSJT-X Improved）が非標準・複合・接尾辞付きコールサインの QSO のために組み立てる 168 通りのメッセージを、
テーブルの 2 つの状態で確かめた（`tests/ws_77bit_extension.rs`）。このトラフィックについて、呼び出し側が知っておくべきこと:

- **`<...>` は「このデコーダがまだ学んでいないハッシュ」を意味し、エラーではない。** 第三者の局には、メッセージのハッシュ部分がこう見える。完全なコールを見る前
  （型 4 のメッセージや CQ の前）のデコーダにも、こう見える。行に推測が入ることはない。テキストがテーブルを必要としたとき `RowDetail::hash_resolved` が立つ。
  `Decoder` はモードごとに 1 つを、セッションの間ずっと使い続けること。ある周期で聞いたコールが、次の周期で解決される。
- **レポートに入るコールは基本コールのことがある。** `DG2YCB/QRP` との QSO のレポートと R レポートに、`DG2YCB` を送るプログラムがある。最初のメッセージと RR73/73 には
  完全なコールが入る。メッセージのどこにも、両者が同じ局だとは書かれていない。相手の照合は WSJT-X と同じく基本コールで行い、ログに書くコールは、完全な形で運んだメッセージから取ること。
- **型 4 にはレポートもグリッドもない。** 平文の非標準コールを含むメッセージは、`<相手> 自局`、`... RRR`、`... RR73`、`... 73` になる。
  WSJT-X は、2 つのコールが標準メッセージに収まるときレポートを送る。標準コール 2 つ（`W1XYZ/P DG2YCB -05` は型 2）、または接尾辞のない標準コールと、もう一方を 22 ビットハッシュにしたもの
  （`W9XYZ <PJ4/K1ABC> -11`、型 1）である。一方が平文の非標準コールのとき、またはハッシュと `/R`・`/P` 付きのコールを組み合わせるとき（`W1XYZ/P <DG2YCB/MM> -05`）は送れない。
  この 2 つの入れ物は型 4 しかなく、型 4 にはレポートがないので、rc1 はそのパックを拒否する。形式としては、両コールをハッシュにした型 1
  （`<W250USA> <DG123YCB> -05`。rc1 はパックもアンパックもできる）でレポートを運べる。両方のコールが既知になってからである。WS はこの形のメッセージを組み立てるが、WSJT-X は組み立てない。
- **ハッシュは、同じハッシュを持つ保存済みコールに解決される。** 12 ビットのハッシュ（型 4）の値は 4,096 通りなので、500 個の異なるコールを持つデコーダは、出会う未知の
  12 ビットハッシュの約 11% を、何かの保存済みコールに解決する。22 ビットのハッシュで保存が 1,000 個なら約 0.024%（テーブルの大きさからの算術で、測定ではない）。
  解決された `<CALL>` は証拠として扱い、交信をログに書くときは、完全なコールを待つこと。
- **`/P` と `/R` はハッシュされるコールの一部である。** コールは接尾辞付きでハッシュされ、学習される（WSJT-X と同じ）。`learn_callsign("W1XYZ/P")` とし、`"W1XYZ"` とはしない。
- **QSO の進行制御はこのクレートの対象外。** こうしたメッセージに対する WSJT-X 自身の扱い（たとえば第 3 語のないメッセージをレポート 0 と数える）は、アプリケーション側のロジックで、移植していない。

詳細、表、根拠: [`docs/notes/WS_77BIT_EXTENSION.md`](../notes/WS_77BIT_EXTENSION.md)。

## 3. プロトコル

### 3.1 プロトコル毎の汎用 vs 専用

最初に読むべき地図がこれ。各行が 1 プロトコルで、各セルはその層が
**汎用**（汎用コアからそのまま再利用） か **専用** (そのプロトコル
自身のモジュールにあるコード) かを示す。

| プロトコル | FEC コーデック | メッセージコーデック | Sync mode | デコード入口 |
|-----------|---------------|--------------------|-----------|-------------|
| **FT8**  | 汎用 `Ldpc174_91` | 汎用 `Wsjt77Message` (77 bit) | `Block` — 3×Costas-7 | `Decoder<Ft8>`、内部は FT8 専用 `ft8::decode_block` エンジン [^ft8] |
| **FT4**  | 汎用 `Ldpc174_91` | 汎用 `Wsjt77Message` (77 bit) | `Block` — 4×Costas-4 | `Decoder<Ft4>`、汎用 `engine::pipeline` の上 |
| **FST4** | 汎用 `Ldpc240_101` | 汎用 `Wsjt77Message` (77 bit) | `Block` — 5×Costas-8 | `Decoder<P>`、汎用 `engine::pipeline` の上 |
| **WSPR** | 専用 `ConvFano` (畳み込み r=½ K=32 + Fano) | 専用 `Wspr50Message` (50 bit) | 専用 `Interleaved` [^wspr] | `Decoder<Wspr>`、専用 `wspr::decode` の上 |
| **JT9**  | 専用 `ConvFano232` (畳み込み、206 bit 枠) | 汎用 `Jt72Codec` (72 bit) | `Block` (長さ 1 スロット) | `Decoder<Jt9>`、専用 `jt9` 入口の上 |
| **JT65** | 専用 `Rs63_12` (RS GF(2⁶)、消失対応) | 汎用 `Jt72Codec` (72 bit) | `Block` (長さ 1 スロット) | `Decoder<Jt65>`、専用 `jt65` 入口の上 |
| **Q65**  | 専用 `Q65Fec` + GF(64) 上の QRA コーデック [^q65] | 専用 `Q65Message` (77 bit) | `Block` | `Decoder<P>`、専用 `q65::rx` の上 |
| **uvpacket** | 汎用 `Ldpc240_101` (punctured) | 専用 `UvPacketRawMessage` (バイトパイプ) | `Block` — Costas-4 [^uv] | 専用 `uvpacket::rx` |
| **MSK144** | 汎用 `Ldpc128_90` + CRC-13 | 汎用 `msg::wsjt77` (77 bit) | **なし — `Protocol` を実装しない** [^msk] | 専用 `msk144::decode::decode_slot` |
| **JTTY** | 専用 tail-biting 畳み込み r=½ K=10 (`jtty::tbcc`、list-WAVA は `jtty::trellis`) + CRC-12 | 専用 32 bit `jtty::source` 文法 (`Atom`)、1 メッセージが複数フレーム | **なし — `Protocol` を実装しない** [^jtty]、全フレームの先頭に 13 トーンの sync | 専用 `jtty::rx::{Receiver, Stream}` |

この表が可視化するパターン:

- **FT8 / FT4 / FST4** は「安い」追加 — LDPC + 77 bit メッセージ +
  ブロック Costas 同期で、ほぼ全部が汎用。
- **WSPR** は *FEC 系統*・*メッセージ長*・*sync mode* の 3 つを独立に
  差し替える — これらの軸が本当に直交している証拠。
- **Q65** は第 3 の FEC 系統 (GF(64) 上の非二進 QRA)、1 マクロから
  10 sub-mode、そしてデコード戦略の一群（§3.4）を、いずれも同じ
  `Protocol` super-trait の内側で加える。
- **uvpacket** は非 WSJT の応用例で、FEC マザーコードだけを再利用し
  汎用 TX/RX パイプラインは迂回する。詳細は
  [`UVPACKET.md`](UVPACKET.ja.md)。
- **MSK144** は唯一、trait 面そのものから外れるプロトコルだが、それでも
  FEC 層とメッセージ層は再利用する。
- **JTTY** も外れ、DSP より上は何も共有しない: FEC・メッセージ文法・受信器は
  すべて独自（`jtty::*`）で、受信器が逐次入力なのはこのモードだけである —
  [§2.5](#25-extras-と-decoder-の外にあるプロトコル)。

> この表は `mfsk-core/tests/common_selftest.rs` のコード共有ラチェット、
> `README.md` の共有率パラグラフ、`lib.rs` 自身のドキュメントが揃って
> 辿り着く先の正本である。ここを変えるならそれらも変わる。

[^ft8]: FT8 は FT4/FST4 と同じ `Decoder<P>` で駆動されるが、
    内部では `engine::pipeline` ではなく手調整された専用エンジン
    `ft8::decode_block` (ホスト・組込み共用) を通る。
    [§6](#6-engine-プリミティブ) を参照。

[^wspr]: `SyncMode::Interleaved` — チャネルシンボルすべての LSB に
    固定 162 bit sync vector の 1 bit を載せる形式で、ブロック Costas
    ではない。このバリアントを使うのは WSPR のみ。

[^q65]: `Q65Fec::decode_soft` は**設計上 `None` を返す** — 実デコードは
    bit-LLR ではなく GF(64) の確率ベクトル上で QRA コーデック
    (`fec::qra` + `fec::qra15_65_64`) が行う。`NTONES = 65` かつ
    `BITS_PER_SYMBOL = 6` (tone 0 は同期専用) が `GRAY_MAP` 長の契約を
    `[2^BITS_PER_SYMBOL, NTONES]` に緩めた事例。

[^uv]: uvpacket は汎用パイプラインを迂回するため、`ModulationParams`
    定数のいくつかは装飾的 — trait と不変条件テストを満たすためだけに
    存在する。

[^msk]: MSK144 (issue #25) は連続位相の二値 MSK を offset-QPSK として
    送信し、864 サンプルのフレームを固定スロット内の既知オフセットに
    置くのではなく T/R 期間全体で繰り返す — したがって
    `ModulationParams`/`FrameLayout` も `engine::pipeline` も合わず、
    `Protocol` を実装する ZST も存在しない。独自の
    `msk144::decode::decode_slot` ドライバが `msk144::spd`/`msk144::sync`
    でピングを走査する。それでも 77 bit `msg::wsjt77` コーデックと汎用
    LDPC BP/OSD エンジン (`fec::ldpc_128_90`) は再利用する。WSJT-X `samples/MSK144/*.wav` に対する
    ゴールデン WAV recall は 3/3 (`tests/msk144_wsjtx_samples.rs`)。

[^jtty]: JTTY (WSJT-X 3.2.0-rc1、#477) は 31.25 ボーの 4-GFSK (`NSPS`
    384、GFSK BT 2、変調指数 1 なのでトーン間隔 = ボーレート) で、
    いつでも始まりうる自己完結した 1.888 s のフレーム — 同期 13 + データ 46
    トーン — からなる。T/R スロットが無いので `ModulationParams` /
    `FrameLayout` が言うべきことは無く、`Protocol` を実装する ZST も存在しない。
    `PROTOCOLS` にも載らない（モード番号 `MFSK_MODE_JTTY` を与えるのは
    `mfsk-ffi` だけ）。`jtty::rx::Receiver` は窓を、`Stream` はライブの
    入力をデコードし、どちらも `jtty_mdecode`（信号減算、遡及再スイープ、
    メッセージ組み立てを含む）を移植したもので、`jtty::pack` /
    `jtty::tx` がテキストを送信する。upstream の `rjtty` に対する recall:
    AWGN/フェージングスイープの 18 セルすべてで同一。tier C の 50 % 交差は
    −16.20 dB (AWGN) と −15.25 dB (中程度のフェージング)、360 ファイルで
    想定外のデコードは 0 件 (`tests/jtty_sweep.rs`)。


### 3.2 諸元

配線済み ZST は 24 個 — WSJT 系のプロトコルとサブモードが 20、
`uvpacket` のサブモードが 4。MSK144 と JTTY は参考として最終行に挙げてあるが
この 24 には**含まれない** — どちらも `Protocol` を実装しないため、レジストリ
にも `tests/protocol_invariants.rs` にも現れない。

| プロトコル       | スロット   | トーン | シンボル | トーン Δf  | FEC                   | Msg   | Sync          | 備考 |
|------------------|------------|--------|----------|------------|-----------------------|-------|---------------|------|
| FT8              | 15 s       | 8      | 79       | 6.25 Hz    | LDPC(174, 91)         | 77 b  | 3×Costas-7    | |
| FT4              | 7.5 s      | 4      | 103      | 20.833 Hz  | LDPC(174, 91)         | 77 b  | 4×Costas-4    | |
| FST4-15          | 15 s       | 4      | 160      | 16.667 Hz  | LDPC(240, 101)        | 77 b  | 5×Costas-8    | 最速 FST4、閾値約-20.7dB |
| FST4-30          | 30 s       | 4      | 160      | 7.143 Hz   | LDPC(240, 101)        | 77 b  | 5×Costas-8    | 閾値約-24.2dB |
| FST4-60A         | 60 s       | 4      | 160      | 3.0864 Hz  | LDPC(240, 101)        | 77 b  | 5×Costas-8    | 地上波主力サブモード、閾値約-28.1dB |
| FST4-120         | 120 s      | 4      | 160      | 1.4634 Hz  | LDPC(240, 101)        | 77 b  | 5×Costas-8    | 閾値約-31.3dB |
| FST4-300         | 300 s      | 4      | 160      | 0.5580 Hz  | LDPC(240, 101)        | 77 b  | 5×Costas-8    | 閾値約-35.3dB、実装済み中最深 |
| WSPR             | 120 s      | 4      | 162      | 1.465 Hz   | conv r=½ K=32 + Fano  | 50 b  | シンボル毎 LSB (npr3) | |
| JT9              | 60 s       | 9      | 85       | 1.736 Hz   | conv r=½ K=32 + Fano  | 72 b  | 16 分散位置   | |
| JT65             | 60 s       | 65     | 126      | 2.69 Hz    | RS(63, 12) GF(2⁶)     | 72 b  | 63 分散位置   | |
| Q65-15A          | 15 s       | 65     | 85       | 6.667 Hz   | QRA(15, 65) GF(2⁶) + CRC-12 | 77 b | 22 分散位置 | |
| Q65-30A          | 30 s       | 65     | 85       | 3.333 Hz   | (同 QRA codec) | 77 b | (同) | |
| Q65-60A          | 60 s       | 65     | 85       | 1.667 Hz   | (同 QRA codec)        | 77 b  | (同)          | 6 m EME |
| Q65-60B          | 60 s       | 65     | 85       | 3.333 Hz   | (同 QRA codec)        | 77 b  | (同)          | 70 cm / 23 cm EME |
| Q65-60C          | 60 s       | 65     | 85       | 6.667 Hz   | (同 QRA codec)        | 77 b  | (同)          | ~3 GHz EME |
| Q65-60D          | 60 s       | 65     | 85       | 13.33 Hz   | (同 QRA codec)        | 77 b  | (同)          | 5.7 / 10 GHz EME |
| Q65-60E          | 60 s       | 65     | 85       | 26.67 Hz   | (同 QRA codec)        | 77 b  | (同)          | 24 GHz+、強拡散 |
| Q65-120D         | 120 s      | 65     | 85       | 6.0 Hz     | (同 QRA codec)        | 77 b  | (同)          | 10GHz レインスキャッター/対流圏散乱 |
| Q65-120E         | 120 s      | 65     | 85       | 12.0 Hz    | (同 QRA codec)        | 77 b  | (同)          | 6m イオノスキャッター |
| Q65-300A         | 300 s      | 65     | 85       | 0.289 Hz   | (同 QRA codec)        | 77 b  | (同)          | 光散乱、最深AWGN |
| MSK144 | T/R 周期 | 2 (MSK) | 144 | — | LDPC(128, 90) + CRC-13 | 77 b | バースト走査 | `Protocol` 実装ではない |
| JTTY | なし (任意の開始) | 4 | 59 (同期 13 + データ 46) | 31.25 Hz | tail-biting conv r=½ K=10 + CRC-12 | 32 b ソース + フラグ 2 b | 先頭 13 トーン sync | 1.888 s フレーム、NSPS 384 (31.25 ボー)、`Protocol` 実装ではない |

### 3.3 プロトコル別の注記

- **FT8 / FT4 — WSJT-X 3.x で変わったこと、そしてこのクレートが従うもの。**
  `v2.7.0` と `v3.0.0` のタグで読んだ（3.2.0-rc1 も同じ値を持つ）。FT8 は
  SNR の下限と `xsnr2` の打ち切りが **−25 dB**（`FT8_SNR_FLOOR_DB`、以前は −24）、
  `Ft8::AP_MAG_SCALE` が **1.1**（以前は 1.01）、`mlag` が **13**、3 パスで
  パス 2 と 3 は二乗した `|cs|²` メトリック、5 つ目の LLR 変種 `llre`、
  nsync の下限（`> 6`、二乗メトリックのパスでは `> 7`、`Depth::Fast` と `Depth::Normal` では、
  `ndepth <= 2` と同様に `> 8`）を持ち、コンテスト外では `/R` と `TU; ` のメッセージを捨てる
  （#438、#439、§2.6）。FT4 の公開既定値は `sync_min` 1.18、`max_cand` 200
  （#440。`ft4_decode.f90` は 1.2 / 100 から移った）。メッセージパッカは
  `pack77_1` に従う: `RR73` はグリッド `RR73`（フィールド 32373、
  `MAXGRID4 + 3` ではない）として送られ、−35..−31 dB のレポートは upstream と
  同じく 101 で折り返し、`/P` / `/R` 付きのコールもパックされる（どちらかが
  `/P` を持てば Type 2）（#464）。
  **OSD は `decode174_91` と同じやり方で走る（#456）。** FT4（と FT8 の AP の段）
  では、BP のあと、BP の 1 反復後と 2 反復後の和に対して OSD をかけ、固定した
  ビットを反転させるテストパターンは飛ばし、CRC は勝者に対して 1 回だけ検査する
  （`bp_llr_zsum_ap_with_scratch` 上の `osd_decode_npre1_masked`）。以前は生の
  LLR を探索して全候補で CRC を検査していた: iid ガウス LLR では `osd_depth` 2 の
  呼び出しの **22.6 %** が CRC を通り、`decode174_91` の 5.8e-5 とは桁違いだった
  （修正後は 9.7e-5）。`rx_freq_hz` の 50 Hz 以内では FT4 は 3 つ目の OSD
  スナップショットも取る（`FecOpts::osd_snapshots`、`maxosd = 3`）: スイープ
  20 800 ファイルで 41 件増え、失ったものは無い。OSD 後の `osd_max_errors`
  ゲートは無くなった（[§6](#6-engine-プリミティブ)）。
- **FST4** — LDPC(240, 101) + 24 bit CRC (`fec::ldpc240_101`)。BP/OSD
  のコードは LDPC サイズが変わっても同じなので、新規なのはパリティ
  検査行列・生成行列と符号寸法だけ。実装済みの 5 sub-mode
  (FST4-15/30/60A/120/300) は `NSPS` / `SYMBOL_DT` / `TONE_SPACING_HZ`
  のみが異なり (FST4-15 だけ `TX_START_OFFSET_S` も 1.0 s ではなく 0.5 s)、
  `fst4_submode!` マクロが生成する。
  FST4-900 / FST4-1800 は未実装 (需要なし)。FST4W (WSPR 型片方向
  50 bit ビーコン、LDPC(240, 74)) は別の
  メッセージ形式で対象外 — issue #23 参照。**OSD は (240, 91) 部分符号を
  探索する**。`fst4_decode.f90:478`（`decode240_101(llr, Keff=91, …)`）と同じで、
  メッセージと先頭 14 個の CRC ビットだけが自由で、最後の 10 個の CRC ビットは
  符号にカスケードされる（`ldpc240_101::FST4_KEFF = 91`、`osd::PartialCrc`）。
  upstream 自身の `decode240_101` で、1 点あたり 4000 回の BPSK/AWGN では、
  振幅 1.0 / 0.9 / 0.8 で 3432 / 2071 / 593 を回収し、101 ビット全部が自由な
  場合の 2903 / 1231 / 211 を上回る。約 0.5 dB である。CRC-24 は OSD の勝者に
  対して 1 回だけ検査する（`osd240_101.f90:285`）。検出に使えるのが 14 ビット
  なので、呼び出しあたりの誤符号語率は FT8 の 2⁻¹⁴ であり、そのため
  `FrameDecodable::REQUIRES_UNPACK`（FST4 は true）は、77 ビットがアンパック
  できないデコードを、`fst4_decode.f90:570` と同じく拒否する。`jt9 -7 -d3` に
  対する tier C、20 グループ: 交差は −0.07 dB（このクレートから `jt9` を引いた
  値。以前は +0.18）、想定外のデコードは 27 件（以前は 99 件、`jt9` は 7 件）
  （#456）。
  `noise_blanker` extra は WSJT-X の **NB** である（[§2.5](#25-extras-と-decoder-の外にあるプロトコル)）:
  1 秒に 20 回のフルスケールのクリックを入れた FST4-15 の 50 スロットで、
  これ無しでは 0 件、ここでは 29 件、`jt9` は NB 2 % で 27 件（#469）。
- **WSPR** — `ConvFano` は WSJT-X `lib/wsprd/fano.c` の移植、
  `Wspr50Message` は Type 1 / 2 / 3 を実装。`wspr` モジュールは
  120 s スロットの coarse search を妥当な時間で回すため四半シンボル
  粒度のスペクトログラムを追加する。
- **JT9 / JT65** — JT9 の `ConvFano232` は WSPR の `ConvFano` と
  206 bit 符号語フレーミングだけが異なり、いずれも 72 bit `Jt72Codec`
  に接続する。JT65 の `Rs63_12` は
  Karn の Berlekamp-Massey による消失対応復号を提供する。
- **Q65** — GF(64) 上の QRA (`fec::qra::QraCode` + 具象コード
  `fec::qra15_65_64::QRA15_65_64_IRR_E23`)。アプリケーション層は 13
  情報シンボルに CRC-12 を付与し、65 シンボルの符号語から CRC 2
  シンボルを puncture して 63 チャネルシンボルを実送信する。10
  sub-mode は `NSPS` とトーン間隔 (×1…×16) のみが異なり、すべてのデコード戦略
  は同じ QRA codec を共有する。デコーダのメトリックは
  `q65_init` と同じく puncture 後の符号化率 13/63 を使う（以前は 15/65 で 12 %
  高かった）。またすべてのリスト復号は `q65_dec1` と同じく `plog > PLOG_MIN`
  （−242）と非ゼロのメッセージを要求する。
- **JTTY** — [§3.1](#31-プロトコル毎の汎用-vs-専用) の脚注と
  [§2.5](#25-extras-と-decoder-の外にあるプロトコル) を参照。定数は trait 定数
  ではなく `jtty` にある（`NSPS`、`SYNC_SYMBOLS`、`FRAME_SYMBOLS`、
  `MAX_FRAMES` = 16）。GFSK パルスは `engine::dsp::gfsk` ではなく `jtty::tx` に
  ある独自の 1 始まりのものを持つ: このパルスは #482 まで 1 サンプル早く、
  JTTY の波形チェック（upstream に対して 4.9e-2）がそれを暴いた。

### 3.4 デコード戦略

どのプロトコルも同じ基本フローを走るが、その周りを包む*戦略*が異なり、`Depth`
（[§2.2](#22-decodeparams-と-depth)）が `ndepth` と同様にそれを選ぶ。
大半は単一パスである。1つの FEC フレームに対して複数の並列受信系を
持つのは Q65 だけで、MSK144 はスロットモデル自体をバースト走査に
置き換え、JTTY は逐次入力の受信器に置き換えている。

| プロトコル | `Depth` ごとの戦略 | 任意（extras とパラメータ） |
|----------|-----------|-----------|
| **FT8** | `Fast` はフラット SIC 2 ラウンド・OSD なし、`Normal` / `Deep` は `SicEarly` | `Tuning::strategy`（`SinglePass`、`SicRounds(n)`、`SicEarly`）、QSO 文脈または `ap_hint` からの AP iaptype ループ (1–12)、**a7 / a8 リストデコーダ**（pass id 30 / 31。FT8 の全戦略の最後に走る。a7 は `a7` extra と `SlotInput::period`、a8 は MyCall・HisCall・HisGrid と `rx_freq_hz`）、`sniper` |
| **FT4** | `Fast` は単一パス・OSD なし・AP なし、`Normal` は `SicRounds(3)`、`Deep` は OSD 付きの `SicRounds(3)` | `Tuning::strategy`（`SinglePass`、`SicRounds(n)`）、フルスロット・コヒーレント sync (`sync2d`) |
| **FST4** | 全 depth で単一パス BP + OSD、`Normal` から `i0 ± 1` のタイミング再試行 | フルスロット2段コヒーレント sync 探索、`noise_blanker`（固定 % またはスイープ） |
| **WSPR** | 四半シンボル・スペクトログラム走査の上で、wsprd の `-qB` / `-C 500 -o 4` / `+ -d` の各パス | `max_cycles_per_bit` |
| **JT9** | 単一の専用パス、depth ごとの Fano limit。`rx_freq_hz` では Rx 周波数パス | — |
| **JT65** | 減算付きの 2 / 2 / 4 パス。depth の `nvec` 回の試行を持つ確率的 Chase デコーダ | `chase`（RS 消失復号は `jt65::SniperRequest::erasures`） |
| **Q65** | `(Δf,Δt,b90)` グリッド + Lorentzian フェージング BP（スキャン）、グリッドの労力は depth で決まる | AP ヒント、明示的な高速フェージング、AP リスト、平均、**q3** リスト復号、Max Drift、Pileup、EME 遅延（[§2.5](#25-extras-と-decoder-の外にあるプロトコル)） |
| **MSK144** | T/R 周期全体のバースト走査 | — |
| **JTTY** | ストリーミング: sync サーフェス、候補、4 段の list-WAVA ラダー、ゲート。デコードしたフレームを減算し、遡及再スイープし、フレームをメッセージに組み立てる | `Params::subtract` をオフ（単一信号の受信器） |

**事前情報デコード (AP) は sniper の機能ではなく一般の選択肢である。**
AP は候補ごとの ladder の最後の一段である — FT4 と FST4 全サブモードでは
`process_candidate_basic` の、FT8 では FT8 自身の ladder の。
`msg::pipeline_ap` は仮説生成だけで自前のエンジンを持たない。かつて偶然 sniper と結合しており、それがデコードの
大半を失わせていた — 実測は
[`DESIGN_RATIONALE.md`](../notes/DESIGN_RATIONALE.md) にある。

**Q65 の戦略** — どのブロックでどれが走るか、q3 リスト復号、平均、Pileup、Max Drift、時間窓と
EME 遅延、履歴とコーラーのヘルパ — は
[§2.5](#25-extras-と-decoder-の外にあるプロトコル)にある。

---

## 4. モジュールとクレートの地図

```text
mfsk_core
├── engine/           Protocol trait 群、DSP、sync、LLR、equaliser、pipeline
│   ├── protocol.rs     ModulationParams / FrameLayout / Protocol / FecCodec / MessageCodec
│   ├── dsp/            resample · downsample · gfsk · cpfsk · envelope · subtract ·
│   │                   msk · analytic · ddc · fir_decimate · polyphase · dotprod ·
│   │                   symbol_fft · blanker · 固定小数点 FFT カーネル
│   ├── fft.rs          FftPlanner トレイトと extern factory (EMBEDDED.md 参照)。スレッドごとのプランナ `with_planner`
│   ├── scalar.rs       Q-format 固定小数点スカラ型
│   ├── sync.rs         coarse_sync / refine_candidate
│   ├── sync2d.rs       FT4 / FST4 フルスロット・コヒーレント sync 探索
│   ├── search.rs       SearchParams / SyncCandidate / SearchWindow — WSPR、JT9、
│   │                   JT65、Q65 が共有する粗探索 (#394)
│   ├── gray.rs         gray / inv_gray、幅をパラメータ化 (`igray.c`) — JT65、JT9
│   ├── ft4_coarse.rs   FT4 の粗候補生成
│   ├── baseline.rs     スペクトルのベースラインフィット (FT4 / FST4 正規化)
│   ├── llr.rs          symbol_spectra / compute_llr / sync_quality
│   ├── equalize.rs     equalize_local (トーン毎 Wiener)
│   ├── spectrogram.rs  Spectrogram 構築/スコアリングカーネル — JT9, JT65, Q65
│   │                   (WSPR は独自実装を維持 — fixed-point FFT backend +
│   │                   baseline-fit 正規化という実質的な差異があるため)
│   ├── interleave.rs   bit-reversal interleave_bitrev/deinterleave_bitrev
│   │                   — WSPR, JT9 (JT65 は別アルゴリズム — 7×9 行列転置
│   │                   — のため jt65/ に独自実装を維持)
│   ├── tx.rs           message_to_tones / info_to_tones、FskWaveform、および
│   │                   synthesize / synthesize_into / synthesize_i16 / synth_len
│   └── pipeline.rs     decode_frame / decode_frame_subtract / process_candidate_basic
│                       (pub(crate) 内部実装 — 呼び出しは
│                       decoder::Decoder 経由)
├── decoder/          公開デコード API — §2
│   ├── mod.rs          Decoder<P> / Decodable / SlotInput / Audio / Row / RowDetail / SlotResult
│   ├── params.rs       DecodeParams / Depth / Station / QsoContext / ApMode / Contest / SearchTuning
│   ├── frame.rs        FT8 / FT4 / FST4: Ft8Extras · Ft4Extras · Fst4Extras、Tuning、戦略、QSO 文脈 AP
│   ├── slow.rs         WSPR / JT9 / JT65: それぞれの extras と状態
│   ├── q65.rs          Q65Extras / Q65State（平均）
│   └── any.rs          AnyDecoder / AnyExtras / Unsupported
├── slotgrid.rs       SlotGrid · SampleClock · ClockChange · SlotCutter — UTC スロット算術、整数のみ、no_std
├── fec/              FecCodec 実装群
│   ├── ldpc/           LDPC(174, 91)  — FT8, FT4 (bp.rs / osd.rs / params.rs / tables.rs)
│   ├── ldpc240_101/    LDPC(240, 101) — FST4、uvpacket (punctured)
│   ├── ldpc_128_90/    LDPC(128, 90)  — MSK144
│   ├── conv/           ConvFano r=½ K=32 — WSPR、ConvFano232 — JT9 (fano.rs)
│   ├── rs/             RS(63, 12) GF(2⁶) — JT65
│   ├── qra/            Q-ary RA codec ファミリ — Q65
│   │   ├── code.rs       汎用 QRA エンコーダ + 非二進 BP デコーダ
│   │   ├── q65.rs        Q65 ラッパー (CRC-12 + puncturing) + リストデコード
│   │   ├── fast_fading.rs ドップラー拡散対応 intrinsic metric
│   │   ├── fading_tables.rs Gaussian / Lorentzian キャリブレーション表
│   │   ├── npfwht.rs      非二進 Walsh-Hadamard 変換ヘルパ
│   │   └── pdmath.rs      確率領域 BP 数値計算ヘルパ
│   └── qra15_65_64/    QRA15_65_64_IRR_E23 の符号インスタンス
├── msg/              メッセージコーデックと公開の出力行
│   ├── decode_request.rs フレーム系のリクエストビルダー — crate 非公開（`internal-testing` で開く）。`decoder` が駆動する
│   ├── decoded.rs      Decoded — モード共通の出力行
│   ├── wsjt77.rs       77 bit WSJT メッセージ — FT8, FT4, FST4, Q65, MSK144
│   ├── wspr.rs         50 bit WSPR Types 1 / 2 / 3
│   ├── jt72.rs         72 bit JT メッセージ — JT9, JT65
│   ├── callsign28.rs   共有 base-37/36/10/27³ コールサイン pack/unpack
│   ├── q65.rs          77 bit <-> 13×GF(64) symbol パッキング (QRA codec 用)
│   ├── ap.rs           ApHint — a-priori ヒントビルダー
│   ├── pipeline_ap.rs  AP 仮説生成 (77-bit 系プロトコル)
│   ├── packet_bytes.rs PacketBytesMessage — バイトペイロード例示コーデック
│   └── hash_table.rs   コールサインハッシュテーブル
├── registry.rs       PROTOCOLS 静的配列 + ProtocolMeta + by_id / by_name
├── iq/               広帯域 IQ 入力 — §2.7: IqToAudio（1 チャンネル → 12 kHz USB 音声、120 dB）、IqReceiver（N チャンネルを
│                     UTC の CompletedSlot に切る。デコードはしない。`Channelizer::Direct` か `Pfb`）、PfbChannelizer（ポリフェーズフィルタバンク）
├── ft8/              FT8 ZST + decode + decode_block + wave_gen
│   ├── list_decode.rs  WSJT-X の a7 / a8 リストデコーダ (pass id 30 / 31)
│   └── acquire.rs      実電波の音声からの cold スロット位相取得 (#356)
├── ft4/              FT4 ZST + decode
├── fst4/             FST4 ファミリ — 5 sub-mode ZST (15/30/60A/120/300) + decode
├── wspr/             WSPR ZST + decode + synth + spectrogram search + ddc
├── jt9/              JT9 ZST + decode
├── jt65/             JT65 ZST + decode (+ 消失対応 RS、chase)
├── q65/              Q65 ファミリ — 10 sub-mode ZST + decode + synth
│   ├── decode_request.rs 広帯域・マルチ周期のリクエスト（crate 非公開）、SniperRequest（公開）— §2.5
│   ├── search.rs       default_search_params、eme_delay_late_sec
│   ├── ap_list.rs      full-AP 符号語リスト (`q65_set_list`)
│   ├── hist.rs         Q65History (`q65_hist`)
│   ├── contest.rs      Q65Callers、contest_codewords (`q65_hist2` / `q65_set_list2`)
│   └── q3.rs           q3 リスト復号 (`q65_dec0` のリスト分岐。クレート内部用)
├── msk144/           MSK144 — Protocol 実装なし、独立トップレベルドライバ
├── jtty/             JTTY — Protocol 実装なし (スロットが無い)。ワイヤ形式、フレームデコーダ、逐次受信器
│   ├── source.rs · crc.rs · tbcc.rs   32 bit 文法、CRC-12、tail-biting エンコーダ
│   ├── pack.rs · tx.rs                テキスト → 最少の atom → トーン → 音声
│   ├── trellis.rs · correlate.rs · ladder.rs · subtract.rs   list-WAVA デコーダ、相関器、ラダー、減算
│   └── dsp.rs · rx.rs · assemble.rs   (FFT feature) 窓デコーダ、`Stream`、フレーム → メッセージ
└── uvpacket/         非 WSJT 応用例 — 4 sub-mode ZST、独自 tx/rx
```

各プロトコルモジュールは同名のフィーチャーフラグで gate されている。`engine`、`fec`、`msg`、`registry` は常時利用可能。

### ワークスペースのクレート

ルート `Cargo.toml` の `[workspace] members`:

| クレート | 責務 | 公開 |
|---------|------|------|
| `mfsk-core` | ライブラリ本体。ホスト (rustfft) または差し替え可能な FFT バックエンドで `no_std` + alloc。他はすべてこれの消費者。 | **する**（crates.io） |
| `mfsk-ffi` | 全プロトコルを覆う C ABI: `libmfsk.{so,a,dylib}` とコミット済みの `mfsk.h`。[`BINDINGS.md`](BINDINGS.ja.md) 参照 | しない |
| `mfsk-ffi-abi` | `mfsk-ffi` が再輸出する共有の `#[repr(C)]` mode / status / params / row 型（issue #205） | しない |
| `hosttest/mfsk-app-shared` | `embedded-poc/mfsk-app-shared` のホストで検査可能な部分を通常の `cargo test` で走らせる | しない |

4 つとも `workspace.package.version` が唯一の版の出所である。

ホストの `cargo build` が stable ツールチェーンでコンパイルしようとして
失敗するため、意図的にワークスペース**外**に置いてあるもの:
`embedded-poc/`（M5Stack ESP32 ボード向けの独立した Cargo プロジェクト群。
いずれも `mfsk-core` に path 依存する — [`EMBEDDED.md`](EMBEDDED.ja.md)）と
`bench/wasm/`（wasm-bindgen ハーネス、issue #208）。

`bindings/kotlin/` と `bindings/swift/` は Rust ではないのでどちらの
リストにも無い — [`BINDINGS.md`](BINDINGS.ja.md) 参照。

FT8 専用の小さな組込 C ABI だった `mfsk-ffi-ft8` は 0.11.0 で**退役**した。
ESP-IDF の消費者はどのみち Rust の staticlib シムを必要とし（純粋な C は
`extern "Rust"` の FFT planner シンボルを定義できない）、どうせ Rust を
書くなら `mfsk-core` を直接呼ぶ方が単純だからである。

#### `FecCodec` はシンボル粒度から独立

`FecCodec` trait の表面 (`engine/protocol.rs`) は **bit** で語る:
`&[u8]` info / codeword、`&[f32]` bit-LLR、`K`・`N` も bit 単位。
FEC の系統のうち 2 つ — JT65 の Reed-Solomon over
GF(2⁶) と Q65 の QRA over GF(2⁶) — は非二進符号で、bit 単位の
trait API を満たすために `encode` の中で bit ↔ シンボル変換を
内製している。それぞれの本来のシンボル単位デコードは
`decode_soft` の外側に置かれていて、`Q65Fec::decode_soft` は仕様
として `None` を返し、実際の Q65 デコードは GF(64) 確率ベクトル上で
`fec::qra::Q65Codec` を介して実行される。`K` / `N` を bit で数えて
おくことで、二進・非二進どちらの符号にも
`FecCodec::N ≤ N_DATA × BITS_PER_SYMBOL` という横断的不変条件
（[§8](#8-ランタイムレジストリと-trait-面の検証)）が同じ式で成り立つ。

---

## 5. `Protocol` トレイト階層

このクレートはボトムアップに読むとよい。どのプロトコルにも依存しない
**汎用コア**があり、各プロトコルはそのコアのどの部品を使うかを選ぶだけの
薄いプラグインである。

1. **`engine/`** — プロトコル非依存の DSP・同期・LLR・イコライザ・復号
   パイプライン。ここの関数はすべて `P: Protocol` に対して汎用で、
   プロトコルの定数を読むだけ。プロトコル毎の分岐は一切持たない。
2. **`fec/`** — 前方誤り訂正コーデック群。それぞれ `FecCodec` の実装。
3. **`msg/`** — メッセージコーデック群 (それぞれ `MessageCodec` の実装)
   と、パイプラインを駆動する crate 非公開のリクエストビルダー。
   **`decoder/`** はその上にある: `Decoder<P>`（§2）が WSJT-X のパラメータブロックを
   それらのビルダーに対応づけ、周期をまたぐ状態を持つ。
4. **プロトコル**は 3 つの合成可能な trait を実装する zero-sized type で、
   持つのは定数と 2 つの関連型の選択 — `type Fec` と `type Msg` — および
   `SYNC_MODE` だけである。プロトコルを追加するという行為はそれで全部である。

デコード時、これらの層は 1 つの受信フローとして実行され、実装済みの
すべてのプロトコルが共有する:

```text
┌─────────┐  coarse_sync   ┌──────────────┐  refine_candidate  ┌──────────┐
│ samples │ ─────────────▶ │  candidates  │ ─────────────────▶ │ candidate│
│ i16/f32 │  (FFT/Costas)  │ (f, dt, snr) │   (fine sync)      │ refined  │
└─────────┘                └──────────────┘                    └────┬─────┘
                                                                    │  symbol_spectra
                                                                    ▼
                  ┌─────────────┐  compute_llr  ┌──────────────┐  equalize_local
                  │   LLR vec   │ ◀───────────  │     cs[]     │ ◀──────────┐
                  │  (4 vars)   │   (per WSJT)  │   Complex    │ (per-tone  │
                  └──────┬──────┘               │  per-symbol  │  Wiener)   │
                         │                      └──────────────┘            │
                         │  P::Fec::decode_soft  (LDPC BP / Fano / RS /     │
                         │                        QRA-symbol-level)         │
                         ▼                                                  │
                  ┌─────────────┐                                           │
                  │ info bits   │                                           │
                  └──────┬──────┘                                           │
                         │  P::Msg::unpack                                  │
                         ▼                                                  │
                  ┌─────────────┐                                           │
                  │ message txt │ ──── (subtract for next iter) ────────────┘
                  └─────────────┘
```

トレイト:

<!-- 非コンパイル: 同名 trait をここで再宣言しても実際の定義との
     整合性チェックにはならない (下の worked example は実物の
     trait を import して impl するのでドリフトすれば壊れる)。
     `engine/protocol.rs` を変更したら手動で追随させること。 -->

```rust,ignore
pub trait ModulationParams: Copy + Default + 'static {
    const NTONES: u32;
    const BITS_PER_SYMBOL: u32;
    const NSPS: u32;              // samples/symbol @ 12 kHz
    const SYMBOL_DT: f32;
    const TONE_SPACING_HZ: f32;
    const GRAY_MAP: &'static [u8];
    const GFSK_BT: f32;
    const GFSK_HMOD: f32;
    const NFFT_PER_SYMBOL_FACTOR: u32;
    const NSTEP_PER_SYMBOL: u32;
    const NDOWN: u32;
    // Defaulted knobs — a protocol overrides only what its WSJT-X
    // counterpart does differently (`engine/protocol.rs`):
    const LLR_SCALE: f32 = 2.83;
    const LLR_NSYM_MAX: u32 = 3;                    // FT4 4, FST4 8
    const LLR_NSYM_MID: Option<u32> = None;         // FST4 Some(4): the nsym=4 rung
    const INFO_SCRAMBLE_RVEC: Option<&'static [u8]> = None;  // FT4, FST4: the 77-element rvec
    const SPECTRUM_WINDOW: SpectrumWindow = SpectrumWindow::Rectangular;  // FT4 Nuttall4
}

pub trait FrameLayout: Copy + Default + 'static {
    const N_DATA: u32;
    const N_SYNC: u32;
    const N_SYMBOLS: u32;
    const N_RAMP: u32;
    const SYNC_MODE: SyncMode;  // Block(&[SyncBlock]) または Interleaved { .. }
    const T_SLOT_S: f32;
    const TX_START_OFFSET_S: f32;
    const CODEWORD_INTERLEAVE: Option<&'static [u16]> = None;  // no wired protocol sets it
}

pub enum SyncMode {
    /// ブロック型 Costas / pilot 配列が固定シンボル位置に置かれる。
    /// FT8 / FT4 / FST4 が利用。
    Block(&'static [SyncBlock]),
    /// シンボル毎ビット埋込型: 既知の sync vector の 1 ビットが
    /// 各チャネルシンボルのトーン index の `sync_bit_pos` に埋め込まれる。
    /// WSPR が利用 (symbol = 2·data + sync_bit)。
    Interleaved {
        sync_bit_pos: u8,
        vector: &'static [u8],
    },
}

pub trait Protocol: ModulationParams + FrameLayout + 'static {
    type Fec: FecCodec;
    type Msg: MessageCodec;
    type SyncPhasors: SyncPhasors;   // what the Δt search precomputes; `()` except FT4
    const ID: ProtocolId;
    const AP_MAG_SCALE: f32 = 1.01;  // apmag = max|llr| * this; FT8 1.1, FT4 1.1, FST4 1.1 (3.0 onward)
    const DECODE_FFT1_SIZE: u32 = 0; // forward-FFT length over the slot; 0 = no shared downsampler
}
```

### `Decodable`: プロトコルを `Decoder<P>` の背後に置く

<!-- Not compiled: the decode hooks are `#[doc(hidden)]` and crate-internal. -->

```rust,ignore
pub trait Decodable: Sized {
    const MODE: Mode;                        // the registry::Mode this ZST decodes
    type State: Default + Send;              // what upstream keeps across periods
    type Extras: Clone + Default + Send;     // what this library adds, typed per mode
    type Row: Clone + Send;                  // the mode's native result
    // plus hidden hooks: the decode itself, 77-bit unpack against State, learn
}
```

スロットでデコードする全 ZST（20 の WSJT 系モード。`uvpacket`・MSK144・JTTY は含まない）が
実装する。公開 API が汎用化されている対象はこれで、置き換えられた `FrameDecodable` と
ファミリ別のリクエスト型は crate 非公開である。新しいモードは、ここへの impl、レジストリの
`modes!` リストへの 1 行、`any.rs` の `any_decoder!` へのバリアント 1 つを加える。

### トレイト合成の実例

2 つの具体例で、3 つの trait が実際の ZST でどう組み合わさるかを示す。
これらは実物の import した trait に対してコンパイルされるので、trait 面が
ドリフトすれば壊れる。

**FT4** — 標準的なブロック Costas 系。`Fec` と `Msg` は FT8 と共有する:

```rust
use mfsk_core::engine::{
    FrameLayout, ModulationParams, Protocol, ProtocolId, SyncBlock, SyncMode,
};
use mfsk_core::fec::Ldpc174_91; // fec::ldpc から re-export
use mfsk_core::msg::Wsjt77Message;

#[derive(Copy, Clone, Debug, Default)]
pub struct Ft4;

impl ModulationParams for Ft4 {
    const NTONES: u32 = 4;
    const BITS_PER_SYMBOL: u32 = 2;
    const NSPS: u32 = 576;          // 48 ms @ 12 kHz
    const SYMBOL_DT: f32 = 0.048;
    const TONE_SPACING_HZ: f32 = 20.833;
    const GRAY_MAP: &'static [u8] = &[0, 1, 3, 2];
    const GFSK_BT: f32 = 1.0;
    const GFSK_HMOD: f32 = 1.0;
    const NFFT_PER_SYMBOL_FACTOR: u32 = 4;
    const NSTEP_PER_SYMBOL: u32 = 2;
    const NDOWN: u32 = 18;
    // (LLR_NSYM_MAX / INFO_SCRAMBLE_RVEC 等はデフォルト値のままの
    // recall チューニング用パラメータ — 実際の FT4 側の上書き値は
    // `ft4::Ft4` を参照)
}

impl FrameLayout for Ft4 {
    const N_DATA: u32 = 87;
    const N_SYNC: u32 = 16;
    const N_SYMBOLS: u32 = 103;
    const N_RAMP: u32 = 2;
    const SYNC_MODE: SyncMode = SyncMode::Block(&FT4_SYNC_BLOCKS);
    const T_SLOT_S: f32 = 7.5;
    const TX_START_OFFSET_S: f32 = 0.5;
}

impl Protocol for Ft4 {
    type Fec = Ldpc174_91;          // FT8 と共有
    type Msg = Wsjt77Message;       // FT8 と共有
    // Δt 探索が事前計算するもの。通常は `()`。FT4 本体は
    // `Ft4CoarsePhasors` を持つので、この例とは異なる。
    type SyncPhasors = ();
    const ID: ProtocolId = ProtocolId::Ft4;
}

const FT4_SYNC_BLOCKS: [SyncBlock; 4] = [
    SyncBlock { start_symbol:  0, pattern: &[0, 1, 3, 2] },
    SyncBlock { start_symbol: 33, pattern: &[1, 0, 2, 3] },
    SyncBlock { start_symbol: 66, pattern: &[2, 3, 1, 0] },
    SyncBlock { start_symbol: 99, pattern: &[3, 2, 0, 1] },
];
```

**WSPR** — 3 軸すべてが FT 系と異なる例。`Fec` / `Msg` を新規型に
差し替え、同期は `Interleaved` バリアントで表現する:

```rust
use mfsk_core::engine::{FrameLayout, ModulationParams, Protocol, ProtocolId, SyncMode};
use mfsk_core::fec::conv::ConvFano;
use mfsk_core::msg::wspr::Wspr50Message;

#[derive(Copy, Clone, Debug, Default)]
pub struct Wspr;

impl ModulationParams for Wspr {
    const NTONES: u32 = 4;
    const BITS_PER_SYMBOL: u32 = 2;
    const NSPS: u32 = 8192;                  // 約 683 ms @ 12 kHz
    const SYMBOL_DT: f32 = 8192.0 / 12_000.0;
    const TONE_SPACING_HZ: f32 = 12_000.0 / 8192.0;  // ≈ 1.4648
    const GRAY_MAP: &'static [u8] = &[0, 1, 2, 3];
    const GFSK_BT: f32 = 1.0;
    const GFSK_HMOD: f32 = 1.0;
    const NFFT_PER_SYMBOL_FACTOR: u32 = 1;
    const NSTEP_PER_SYMBOL: u32 = 16;
    const NDOWN: u32 = 32;
}

impl FrameLayout for Wspr {
    const N_DATA: u32 = 162;
    const N_SYNC: u32 = 0;                   // sync はデータシンボルに埋込
    const N_SYMBOLS: u32 = 162;
    const N_RAMP: u32 = 0;
    const SYNC_MODE: SyncMode = SyncMode::Interleaved {
        sync_bit_pos: 0,                     // トーン index の LSB に埋込
        vector: &WSPR_SYNC_VECTOR,           // 162 bit 既知列 (npr3)
    };
    const T_SLOT_S: f32 = 120.0;
    const TX_START_OFFSET_S: f32 = 1.0;
}

impl Protocol for Wspr {
    type Fec = ConvFano;                     // 畳み込み符号 + Fano
    type Msg = Wspr50Message;                // 50 bit メッセージ
    type SyncPhasors = ();                   // Δt 探索の表は持たない
    const ID: ProtocolId = ProtocolId::Wspr;
}

// 説明用のダミー値 — 実際の 162 bit npr3 ベクトルは
// `wspr::decode` 内部の非公開 sync テーブルにある。
const WSPR_SYNC_VECTOR: [u8; 162] = [0u8; 162];
```

### 送信：`FskWaveform`

FSK で送信するプロトコルは `engine::tx::FskWaveform` も実装する。
WSJT-X の2つの送信方式のどちらに属するかを示す定数が1つあるだけ（#391）:

```rust,ignore
impl FskWaveform for Wspr {
    const WAVEFORM: Waveform = Waveform::Cpfsk;          // WSPR, JT9, JT65, Q65
}
impl FskWaveform for Ft8 {
    const WAVEFORM: Waveform = Waveform::Gfsk(FT8_GFSK); // FT8, FT4, FST4
}
```

`engine::tx` に必要なのはこれだけで、`message_to_tones::<P>` が 77 bit
メッセージをトーン列にし（FT8 / FT4 / FST4）、`synthesize::<P>` /
`synthesize_into` / `synthesize_i16` / `synth_len` が上の全モードについて
任意のサンプルレートでトーン列を音声にする。`Cpfsk` は
`TONE_SPACING_HZ` と `SYMBOL_DT` による素の連続位相 FSK で、WSJT-X が
変調器の中で生成するもの。`Gfsk` は `gen_ft8wave.f90` などが事前計算する
整形済み波形。各 `Gfsk` 設定はテストでプロトコル自身の `NSPS`・`GFSK_BT`・
`GFSK_HMOD` と照合しているので、両者がずれることはない。送信が FSK で
ないプロトコル（例として置いている `uvpacket` は π/4-DQPSK）は単に
実装しない。

### Monomorphization がこれを無料にしている

ホットパスの関数はすべて `P: Protocol` を**コンパイル時型パラメータ**として受け取る。rustc が
具象プロトコルごとに 1 コピーずつ monomorphize し、LLVM は完全特殊化
された関数として trait 定数を即値にインライン化する。抽象化のコストは
ゼロ — 生成される FT8 コードは本ライブラリが fork する前の FT8 専用
ハンドコードとバイト単位で同一で、FT4 は共通関数に加えた
マイクロ最適化すべての恩恵を自動的に受ける。

`dyn Trait` はコールドパス専用: FFI 境界と、復号したテキストを
アンパックする `MessageCodec`（候補ごとではなく、デコード成功ごとに 1 回）。

### プロトコルを追加する

`CONTRIBUTING.md` に手順がある。要点を言えば、どれだけの作業になるかは
どれだけ再利用できるかで決まる:

| ケース | 作業 |
|---|---|
| 既存モードと同じ FEC とメッセージ（別の FST4 サブモード） | 数値定数だけが異なる新しい ZST。`Fec`/`Msg` は型エイリアス。汎用パイプライン全体がそのまま動き、`Decodable` の実装（その `State`・`Extras`・`Row`）が `Decoder<P>` の背後に置く |
| FEC が新しく、メッセージは同じ（別サイズの LDPC） | `fec/` にモジュールを追加し `FecCodec` を実装する。BP/OSD/systematic エンコードは LDPC のサイズをまたいで一般化されるので、実際の変更はテーブルと寸法である。`fec::ldpc240_101` が例 |
| どちらも新しい（WSPR） | FEC を追加し、メッセージコーデックを追加し、sync 構造が本当に異なるなら `SyncMode` を拡張する |
| 既存プロトコルのサブモード | `q65_submode!` / `fst4_submode!` マクロが、異なる定数から ZST とその 3 つの trait 実装を生成する。`tests/protocol_invariants.rs` に 1 行足せば拾われる |

FST4-60A は共有コードに一切触れずに追加できた。

---

## 6. engine プリミティブ

### DSP (`mfsk_core::engine::dsp`)

| モジュール | 役割 |
|---|---|
| `resample` | 12 kHz への線形リサンプラ |
| `downsample` | FFT ベース複素デシメーション (`DownsampleCfg`) |
| `gfsk` | GFSK トーン→PCM 波形合成 (`GfskCfg`, `GfskStream`)。3 シンボルのガウスパルスは #482 以降 **1 始まり**で、`gen_ft8wave.f90` / `gen_ft4wave` / `gen_fst4wave` が作るとおり (`tt=(i-1.5*nsps)/nsps`、`i=1..3*nsps`)。以前は `i=0` から走って 1 サンプル早く、FT8 / FT4 / FST4 のあらゆる波形がずれていた（位相誤差は最大 2π·Δf/fs、FT8 で 0.023 rad）。`ft8sim`（v3.2.0-rc1、SNR 99）に対する正規化サンプル誤差の最悪値: 1.3e-2 → 1.2e-4 (`tests/gfsk_vs_wsjtx.rs`) |
| `cpfsk` | 素の連続位相 FSK 合成器 — WSJT-X の正の `toneSpacing` の送信ループ、WSPR / JT9 / JT65 / Q65 (`synth_f32`, `synth_f32_into`)。4 つの `tx.rs` がそれぞれ持っていたものを 1 つにした |
| `envelope` | WSJT-X の変調器がその 4 モードの送信に掛ける raised-cosine ランプ (`ramp_samples`, `apply_ramp`; #259) |
| `symbol_fft` | `SymbolFft`: フレームの各シンボルで再利用する 1 本の `nsps` 点 FFT。`engine::fft` 経由で計画する — JT9、JT65、Q65 の復調器 (#390) |
| `blanker` | `blanker(audio, nz, ndropmax, npct)`: `blanker.f90` のインパルスノイズブランカで、FST4 の `.noise_blanker()` の背後にある |
| `subtract` | 位相連続最小二乗 SIC (`SubtractCfg`) |
| `ddc` | ストリーミング・デジタルダウンコンバータ (WSPR の組込チャネライザ) |
| `fir_decimate` | `FirStage`（複素 I/Q に対するストリーミング FIR + 間引き）とローパスの設計関数: `design_lowpass`（Blackman、約 74 dB）、および選択度を指定する場合の `kaiser_order` + `design_lowpass_kaiser`（Kaiser、`f64`、`no_std`）。`FirStage::from_taps` は設計済みのプロトタイプを受け取る（#534） |
| `polyphase` | `PolyphaseResampler`（複素 I/Q に対するストリーミング有理比 `L/M` リサンプラ）。`from_prototype` は設計済みのプロトタイプを受け取る（#534） |
| `dotprod` | extern フックが置き換えるドット積カーネル |

いずれもランタイム `*Cfg` 構造体を引数に取る (`<P>` ではない) のは、
FFT サイズなどチューニングが trait 定数だけから単純派生できない
ためで、プロトコルモジュールがモジュールレベル定数を公開している:
`ft8::downsample::FT8_CFG`、`ft4::decode::FT4_DOWNSAMPLE` など。

**Gray code。** `engine::gray::{gray, inv_gray}(n, bits)` は幅 1..=8 に
対する `igray.c` である（`u32` で計算する: C のループのシフトは 4 ビット
から `u8` をあふれさせる）。JT65（6 ビット）と JT9（3）が使い、
FT8 / FT4 / FST4 はプロトコル毎の `GRAY_MAP` テーブルを使う。

### Sync (`mfsk_core::engine::sync`)

* `coarse_sync::<P>(audio: AudioSource, freq_min, freq_max, sync_min,
  freq_hint: Option<f32>, max_cand, grid: RxGrid)` — UTC 整列 2D
  ピーク探索、`P::SYNC_MODE.blocks()` を走査 (FT8 以外向け)。
  `AudioSource` は `Real(&[i16])` か `Complex(&[f32], &[f32])` で、
  `RxGrid` と組にする（通常の PCM なら `RxGrid::real(12_000.0)`）。
* `refine_candidate::<P>(cd0, cand, search_steps)` — 整数サンプル
  スキャン + 放物線サブサンプル補間
* `make_costas_ref` / `score_costas_block` —
  診断・カスタムパイプライン用の生相関ヘルパー
* `sync_power_cv(per_block)` — `DecodeResult::sync_cv` の背後にある母集団の
  変動係数。#414 以降、全プロトコルで定義が 1 つである（FT8 のものは、同じ
  チャネルで FT4 や FST4 の √3 倍だった。これに閾値をかけているものは無い）。

**FT8 は `ft8::decode_block::coarse_sync` のみを経由する。**
`engine::sync::coarse_sync::<Ft8>` を直接呼ぶのは、手組みの非既定用途では
今も正しい経路だが、`Decoder<Ft8>` は内部で
`decode_block::coarse_sync` を経由する。そのため FT8 の coarse-sync の変更が
FST4 の感度曲線を動かすことはない: FST4 は代わりに
`engine::sync::coarse_sync` と `engine::sync2d::fst4_sync_search` を通って
sync に至る。

### Sync2D (`mfsk_core::engine::sync2d`)

WSJT-X から移植したプロトコル専用のフルスロット・コヒーレント探索が
2 系統ある。いずれも symbol 境界で位相をリセットせず 8-symbol
ブロック全体で連続的に位相を積算する **位相連続** Costas 参照信号
(`make_costas_ref_continuous`) を用いて `score_flat_coherent` でスコア化
する — 非コヒーレントな `Σ|z_k|²` パワー和に比べ、sync スコアの SNR 弁別力が
~3 dB 改善する:

* `ft4_sync_search::<P>` と窓指定版の `ft4_sync_search_window::<P>` —
  **FT4 専用**。coarse-sync 候補自身の (しばしば外れる) Δt 推定
  周辺のローカル窓ではなく、スロット全体のダウンサンプル・サンプル
  範囲にわたるコヒーレントな Δt 探索 (`ft4_decode.f90` の
  `isync=1`/`isync=2` ループ、`sync4d.f90` のスコア関数)。
* `fst4_sync_search::<P>` — WSJT-X の
  `fst4_decode.f90:657-925` に対応する FST4 専用の 2 段階フル
  スロット探索。coarse パスはスロット全体 (±1.5 s、step 4、
  周波数 ±12 step × 0.1·baud)、fine パスは ±7 step × 0.02·baud ×
  ±4 サンプル。FST4 の AWGN 感度ギャップを WSJT-X 公称値に対して
  ~0.3 dB まで縮小した (issue #146)。

`engine::sync::coarse_sync::<P>` にも FST4 専用の拡張がある: bin は、既存の
short-time Costas グリッドしきい値を通るか、WSJT-X の
`get_candidates_fst4` を模したフルスロット非コヒーレント 4-tone パワー
チェックをクリアすることで、候補リストに入れる
(`P::ID == ProtocolId::Fst4` でゲート、FT8/FT4 はバイト完全一致のまま)。
単一信号の AWGN sweep では no-op として計測されたが、混雑した広帯域
スキャンでは WSJT-X 準拠のカバレッジ改善として意味がある。

### LLR (`mfsk_core::engine::llr`)

* `symbol_spectra::<P>(cd0, i_start)` — シンボル単位 FFT bin
  (FT8 では中間 `cd0` を割り当てない
  `ft8::decode_block::fill_symbol_spectra` を推奨)
* `compute_llr::<P, T>(cs)`（`T: LlrScalar`、`LlrSet<T>` を返す） — WSJT 式 4 バリアント LLR (a/b/c/d)。
  `nsym ∈ {1, 2, P::LLR_NSYM_MAX}` の相関ラダー仮説から構築され、
  プロトコルが `P::LLR_NSYM_MID` を設定していればその `nsym` での `llre`
  （FST4: 4）も加わる。
  `LLR_NSYM_MAX` のデフォルトは 3、FT4 は 4、FST4 は 8
  に上書き — どちらも自身の WSJT-X bit-metric コード
  (`get_ft4_bitmetrics.f90` / `get_fst4_bitmetrics.f90`) に合わせた値
* `sync_quality::<P>(cs)` — 硬判定 sync シンボル一致数

### Equalise (`mfsk_core::engine::equalize`)

* `equalize_local::<P>(cs)` — `P::SYNC_MODE.blocks()` pilot 観測から
  トーン毎 Wiener equalizer を推定、Costas が訪問しないトーンは
  線形外挿でカバー

### Pipeline (`mfsk_core::engine::pipeline`)

`decode_frame::<P>` (coarse sync → 並列 `process_candidate` →
dedupe)、`decode_frame_subtract::<P>` (SIC ドライバ)、
`process_candidate_basic::<P>` (候補単体の BP+OSD) は engine の生関数
である。これらは **`pub(crate)`** で、`pub` になるのは `internal-testing`
の下だけ。`Decoder<P>` を使うこと。

**`DecodeStrictness` (`Strict`/`Normal`/`Deep`) は全プロトコルに等しく
届くわけではない** — `.strictness(...)` が呼び出しに何かをするかどうかを
思い込む前に、どのノブが効くかを確かめること:

| メソッド | 有効な対象 |
|---|---|
| `osd_max_errors()` — OSD 後の硬判定エラー上限、`osd_depth` 別 | **どのデコード経路でもない。** `process_candidate_basic` はもはやどのプロトコルにも適用しない: FST4 が最初に外し (#146)、CRC-24 とアンパック成功 (`REQUIRES_UNPACK = true`、`fst4_decode.f90:570` と同じ) で受理する。FT4 は #456 で続いた。OSD が雑音候補の 22.6 % で誤った符号語を返さなくなったためである (`ft4_decode.f90` にそのようなゲートは無い)。FT8 は一度も呼んだことがない。メソッドは公開 API として、また旧ラダーを写す診断のために残っている |
| `ap_max_errors(locked_bits)` — AP 付きの上限、locked-bit 数で段階化 | FT8 の per-candidate AP ループと、汎用ラダーの AP の段（FT4、FST4 全サブモード）。数値は統一されている (issue #191)。`Normal` は一律 **36** で、`ft8b.f90` の上限（`ft4_decode.f90` には無い）。36 は従来の 30 / 25 に比べ、FT8 スイープで 544 件多くヒットし（+6.2 %、失ったものは無い）、12 800 ファイルあたり 1〜4 件の phantom が増えた (#456)。`Strict` は locked が 55 ビット以上で 20、それ以外は 24。`Deep` は 55 以上で 30、それ以外は 36 |
| `ft8_nharderrors_max()` — FT8 自身の flat（段階化しない）上限、非 AP の BP staircase と OSD フォールバック向け | FT8 (`ft8::decode_block::process_candidates`/`osd_strategy`)。`Normal` は 36 を返し、これは WSJT-X 自身の `ft8b.f90:422` の上限である。`Strict = 22` は issue #72 の先行事例を再利用し、`Deep = 37` は #253 で FT8 スイープ (`MFSK_FT8_SWEEP_STRICTNESS`、AWGN/CCIR 16 セル、レベルと戦略ごとに 320 試行) により 40 から調整し直した値: golden の recall は 37 で既に飽和しており（単一パス 105/320、`.sic_early()` 108/320、40 まで同一）、一方で false accept は増え続けていた（15 → 16、20 → 21） |

### FT8 ブロックデコーダのエントリ (`mfsk_core::ft8::decode_block`)

FT8 モジュールは共有パイプラインの上に並列のエントリ群を持ち、
ホスト・組込で同じ `process_one_candidate_inner` 本体を共有する。
入力は同じで、内側のどのステップを有効にするかが
違うだけ:

* `decode_block` / `decode_block_tuned` — pass-1 BP のみ
* `decode_block_with_ap` / `decode_block_with_ap_tuned` — pass-1 BP
  に続き、`q_thresh` を超える sync quality の候補に対して WSJT-X
  AP iaptype ループ (1–12) を回す。
* `decode_block_into[_tuned]` — 組込 fixed-point エントリポイント
  (`fixed-point` feature)。`decode_block[_tuned]` と同じ形だが、
  `embedded-shared::dual_core` との API 安定性のため
  別名を維持。
* `coarse_sync` / `coarse_sync_with_allsum` — FT8 sync grid 本体
* `fill_symbol_spectra` / `fill_symbol_spectra_goertzel` — 音声から
  直接シンボル毎 FFT を抽出

FT8 には `ft8::list_decode`（a7 / a8 リストデコーダ。全戦略の最後に走る —
[§3.4](#34-デコード戦略)）と `ft8::acquire`（`acquire_slot_phase`: 時計を
持たない受信機のための cold スロット位相取得。より長い録音から 5 s 間隔の
±2.5 s 窓 3 つを取り、`circular_dt_medoid` で 1 つにまとめる; #356）もある。

---

## 7. Feature フラグ

**正本は `mfsk-core/Cargo.toml` の表**であり、ドキュメントとして読む
価値がある — 各フラグにそれを選ばせた実測が併記されている。ここでは
Rust ホスト消費者に関係する分だけをまとめる。`no_std` と固定小数点側は
[`EMBEDDED.md`](EMBEDDED.ja.md)。

`default = ["std", "ft8", "ft4", "parallel", "fft-rustfft"]`。

| Feature | 既定 | 効果 |
|---|---|---|
| `std` | on | 標準ライブラリ。off にすると `alloc` と extern FFT バックエンドが要る |
| `alloc` | — | アロケータ付き `no_std` |
| `ft8` | on | FT8 の ZST・decode・wave_gen |
| `ft4` | on | FT4 の ZST・decode |
| `fst4` | off | FST4-15/30/60A/120/300 の ZST・decode。**ホスト専用ではない** — backend 非依存の engine を完全に通り、`alloc,fst4,fft-extern` で型検査が通る（issue #306） |
| `wspr` | off | WSPR の ZST・decode・synth・スペクトログラム探索 |
| `jt9` / `jt65` / `q65` | off | **#390 以降ホスト専用ではない** — バックエンドを強制せず、`alloc,<mode>,fft-extern` で `std`/`rustfft` を一切引かずに型検査が通る（`ci.yml` と `scripts/pre-push-check.sh` の feature matrix に 1 行ずつ）。デコーダのみで、組込アプリからの利用はまだ無い |
| `msk144` | off | MSK144 — `Protocol` の ZST は無く、独自のトップレベルドライバを持つ |
| `jtty` | off | JTTY — `Protocol` の ZST は無い。FFT を使わないモジュール（`source`・`crc`・`tbcc`・`tx`・`pack`・`trellis`・`correlate`・`ladder`・`subtract`）はどこでもビルドでき、`dsp`・`rx`・`assemble` はホスト FFT（`fft-rustfft` か `fft-extern`）が要る。feature matrix に `alloc,jtty` と `alloc,jtty,fft-extern` の行がある |
| `uvpacket` | off | 非 WSJT の応用例、4 サブモード ZST。`fst4` を引き、かつ **`std` を明示的に宣言する**（`std::f32::consts::PI` を使うため） |
| `packet-bytes` | off | `PacketBytesMessage` — バイトペイロードの `MessageCodec` 例 |
| `full` | off | 全プロトコル + `uvpacket` + `packet-bytes` + `serde` + `parallel` + ホスト FFT |
| `parallel` | on | パイプラインでの rayon `par_iter`（wasm では no-op） |
| `fft-rustfft` | on | ホストの FFT バックエンド |
| `fft-extern` | off | 最終バイナリが `mfsk_core_make_default_fft_planner` を供給する。`engine::fft` 参照 |
| `dotprod-extern` | off | 同様に `mfsk_core_dotprod_f32` |
| `fixed-point` | off | 組込が出荷する数値パス。**`nstep-half` を含意し、そうでなければならない** |
| `serde` | off | 公開結果型への `Serialize`/`Deserialize` |
| `internal-testing` | off | クレート自身の統合テスト向けに `pub(crate)` の engine 内部を開ける。意図的に `full` に**含まれない** |

嵌まりやすいフラグ:

- **クレート全体を対象とするコマンドでは `internal-testing` は必須。**
  無いと `cargo clippy --all-targets --features full` が `fst4_sweep` /
  `ft4_sweep` / `fst4_wsjtx_samples` で `E0603`（private item）を報告する。
  これはフラグ不足であって、あなたが壊した回帰ではない。
- **`fixed-point` は `nstep-half` を含意する。** 切り離すと、ホストの
  固定小数点が組込（NSPS/2）とは違う時間グリッド（NSTEP=NSPS/4）で走り、
  同じ WAV に対して候補の順位が全く変わってしまった — `qso3_busy` の
  単一パスで 4 対 7 decode。ホスト固定小数点の存在意義は組込を忠実に
  模擬することである。
- **`wspr-ddc` と `wspr-ddc-cascade` は排他**である（`decode_scan_inner`
  の `compile_error!`）。WSPR のチャネライザを参照実装の全スロット FFT から
  ストリーミングダウンコンバータ（単段 / 2 段カスケード）へ差し替える。
  ホストの既定は厳密な参照実装のまま。
- **`wspr-fano-cap-fast` は上げ放題のノブではない。** サイクル予算を
  参照実装より上げ始めると phantom decode を作り出す。掃引した表は
  `wspr::decode` にある。

---

## 8. ランタイムレジストリと trait 面の検証

### 8.1 `PROTOCOLS` レジストリ

`mfsk_core::PROTOCOLS` は `&'static [ProtocolMeta]` で、各
`Protocol` 実装 ZST の関連定数からコンパイル時に組み立てられる。
「このビルドは何をサポートするか」を尋ねる消費側は、自前のリストを
ハードコードする必要が無い:

```rust
use mfsk_core::PROTOCOLS;

for p in PROTOCOLS {
    println!(
        "{:10}  {:>3}-tone  {:>4} bits/sym  {:>5.1} s slot  ID={:?}",
        p.name, p.ntones, p.bits_per_symbol, p.t_slot_s, p.id,
    );
}
```

各 `ProtocolMeta` は protocol の `id` (`ProtocolId`、
ファミリレベル)、表示名 `name`、および trait 面が公開する全定数を
保持する — 変調 (`ntones`, `bits_per_symbol`, `nsps`, `symbol_dt`,
`tone_spacing_hz`, `gfsk_bt`, `gfsk_hmod`)、フレーム (`n_data`,
`n_sync`, `n_symbols`, `t_slot_s`)、コーデック (`fec_k`, `fec_n`,
`payload_bits`)。この幾何情報に加えて、ホストが他に尋ねる手段の無いものを公開する:

* `tx_start_offset_s` — スロットバッファの先頭から最初のシンボルまでの秒数で、
  `dt = 0` の基準。FT8、FT4、FST4-15、Q65 の 15 / 30 s で 0.5、他の FST4
  サブモードと Q65 の 60 s 以上で 1.0（Q65 は `q65.f90:130-131` と同じく
  `nsps` に従う。#399 まではどのサブモードも 1.0 を公開していた）。
* `slot_samples_12k` — サンプル数で表したスロット（FT4 90 000、FT8 180 000、
  FST4-300 3 600 000）と、`decode_fft1_size` — デコーダがスロット全体に
  かける前方 FFT（`Protocol::DECODE_FFT1_SIZE`。FT4 92 160、FT8 192 000、
  **FST4-300 4 194 304**、独自のフロントエンドを持つモードは 0）。後者は、
  「どのモードでも呼び出しの形は 1 つ」がメモリの話としては間違いになる数字である。
* `profile: DecodeProfile { caps, defaults, sync_scale, sniper_max_cand_cap }`。

`caps` は `registry::caps` のビットからなる `u32` で、17 個ある。
`tests/registry_caps.rs` が実際の trait 実装と双方向に突き合わせている
（trait が無いのにビットを主張すれば失敗し、ビットが無いのに trait が
あっても失敗する）: `DECODE_HANDLE` 0、`SNIPER` 1、`AP_NARROW` 2、
`AP_WIDEBAND` 3、`SIC_ROUNDS` 4、`SIC_EARLY` 5、`OSD` 6、`EQ_MODE` 7、
`STRICTNESS` 8、`BUDGET` 9、`KNOWN_FILTER` 10、`KNOWN_SUBTRACT` 11、
`FFT_CACHE` 12、`ON_RESULT` 13、`ENCODE` 14、そして `NOISE_BLANKER` = 1 << 16
（FST4 の全サブモード）と `TX_FREQ` = 1 << 17（FT8。マーカ trait が無いので
主張リストが固定する）。**ビット 15 は意図的に飛ばしてある**: これは
`MFSK_CAP_STREAM_RECEIVER` で、`mfsk-ffi` だけが定義する（JTTY にはそれを
主張するレジストリ entry が無い）。
`sync_scale` は `sync_min` の読み方を示す。スケールは同じ数値ではないからだ:
`CostasAbsolute`（FT8。雑音に固定値は無い）、`BaselineNormalised`
（FT4、および #554 以降の FST4: スペクトルをフィットしたベースラインで割るので、
雑音は ~1.0 にあり、それ未満の閾値はあらゆるピークを通す）、`SyncFraction`（WSPR、JT9、JT65、
Q65: sync の電力を sync と雑音の和で割った 0‥1 の値で、既定は
`DEFAULT_SCORE_THRESHOLD` = 0.1）。`sniper_max_cand_cap` は sniper 経路が
`max_cand` に黙って適用する上限である（FT4: 15）。

`profile.defaults` はレジストリが公開するホスト探索値で、C ABI の `mfsk_mode_defaults` が返すもの
でもある。**`Decoder<P>` はこれを読まない**: `sync_min` と候補数は `Depth` が決め
（[§2.2](#22-decodeparams-と-depth)）、帯域は `decoder::default_params(mode)` のものである。
この表はレジストリ自身のデータ（UI が「このビルドの既定の探索は？」と尋ねるためのもの）で、
スキャン系モードの時間窓・スコア閾値・候補数の上限は今も各自の `default_search_params()`
から始まる:

| エントリ | 帯域 (Hz) | `sync_min` | `max_cand` | 出所 |
|---|---|---|---|---|
| FT8 | 100-3000 | 0.8 | 60 | `FT8_PROFILE` |
| FT4 | 300-2700 | 1.18 | 200 | WSJT-X 3.x `ft4_decode.f90` の `syncmin` / `MAXCAND`（#440。以前は 1.2 / 100） |
| FST4（5 つすべて） | 100-3000 | 1.20（FST4-15: 1.15） | 200 | WSJT-X `fst4_decode.f90` の `minsync` と候補配列（#554。以前は Costas スケールで 0.8 / 50） |
| WSPR | 1400-1600 | 0.1 | 200 | `wspr::search::default_search_params()`、±5.46 s（8 シンボル） |
| JT9 | 200-4000 | 0.1 | 8 | `jt9::search::default_search_params()`、±1.728 s |
| JT65 | 1000-2000 | 0.1 | 8 | `jt65::search::default_search_params()`、±7.62 s |
| Q65（10 個すべて） | 200-3000 | 0.1 | 8 | `q65::search::default_search_params()`、±1.0 s（#399、#413） |
| uvpacket | — | 0 | 0 | 探索の仕組みはどれも当てはまらない |

#413 以降、WSPR・JT9・JT65・Q65 は、手で持っていたコピー（4 つすべてで
ずれていた: WSPR / JT9 / JT65 は帯域を公開せず、Q65 は `mfsk-ffi` の広い EME
スキャン、32 候補で 0.05 を公開していた）ではなく、ライブラリ自身の
`default_search_params()` から自分の行を読む。FFT バックエンドが無いと
その 4 つは `search` モジュールを持たず、空の帯域を公開する。

ルックアップ:

* `by_id(ProtocolId::Q65)` — ファミリ id を共有する*すべての* entry。
  Q65 は 10 件、FST4 は 5 件、その他は 1 件。
* `by_name("Q65-60D")` — 名前の厳密一致検索。
* `for_protocol_id(id)` — 同じ id を共有する最初の entry。
  「ファミリ毎に 1 mode」のケースで便利。

family / sub-mode の区別が最も効くのは Q65 である: 10 sub-mode すべてが
`ProtocolId::Q65` を共有しつつ、NSPS・トーン間隔・スロット長が異なる
ため、別々の registry entry として存在する。

レジストリ本体は `mfsk-core/src/registry.rs` 内部の
`protocol_meta!` マクロで構築される。新しいプロトコルの追加は
ZST + 表示名で 1 行ずつ。

### 8.2 汎用 trait 面検査

`tests/protocol_invariants.rs` は 1 つの汎用
`assert_protocol_invariants::<P>` を配線済みの全 ZST に対して実行する —
24 個: WSJT 系 20 に加えて `uvpacket` の 4 つ — そして ~25 個の trait レベルの
不変条件を pin する。その中に `FecCodec::N ≤ N_DATA × BITS_PER_SYMBOL` と
`GRAY_MAP` の長さの契約 `[2^BITS_PER_SYMBOL, NTONES]` がある。

新しいプロトコルはここに 1 行を得る。MSK144 と JTTY は現れない。どちらも
`Protocol` を実装しないからで、これは設計上の決定であって、埋めるべき穴では
ない。

---

## ライセンス

リポジトリルートの [`LICENSE`](../../LICENSE) を参照。ここにある
アルゴリズムはすべて WSJT-X（Joe Taylor K1JT ら）に由来し、各ソース
ファイルが移植元の `lib/*.f90` / `lib/*.c` を明記している。
