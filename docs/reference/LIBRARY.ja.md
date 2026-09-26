# mfsk-core — Rust API リファレンス

> **English:** [LIBRARY.md](LIBRARY.md)

WSJT-X の微弱信号デコーダ群 — FT8・FT4・FST4・WSPR・JT9・JT65・Q65・
MSK144・JTTY — に加え、実験的で WSJT 由来ではない `uvpacket` モードを、
1つの汎用コアの上に純 Rust で再実装したもの。コア
(`engine` / `fec` / `msg`) はプロトコル非依存で、各プロトコルは FEC
コーデック・メッセージコーデック・sync モードをそこへ挿す zero-sized
type である。配線済みの全プロトコルが同じ受信フロー
`coarse-sync → refine → LLR → FEC decode → message unpack` を通り、
その上にプロトコル毎の戦略が重なる。MSK144 と JTTY はこのコアの中ではなく
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
- [2. デコード API](#2-デコード-api)
  - [2.1 `DecodeRequest<P>`](#21-decoderequestp)
  - [2.2 `SniperRequest<P>`](#22-sniperrequestp)
  - [2.3 計算予算](#23-計算予算)
  - [2.4 ストリーミング配信](#24-ストリーミング配信)
  - [2.5 独自エントリポイントを持つプロトコル](#25-独自エントリポイントを持つプロトコル)
  - [2.6 メッセージの受理](#26-メッセージの受理)
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
mfsk-core = { version = "0.12", features = ["ft8", "ft4", "wspr"] }
```

必要なプロトコル feature だけを入れる。以下の例は説明のために複数を
有効にしている。

**FT8 スロットをデコードする。** フレームを合成してから復号する:

```rust
use mfsk_core::ft8::Ft8;
use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::{pack77, unpack77};

// 1. FT8 フレームを合成し、15 秒スロットに詰める。
let msg77 = pack77("CQ", "JA1ABC", "PM95").unwrap();
let tones = message_to_tones::<Ft8>(&msg77);
let frame = synthesize_i16::<Ft8>(&tones, 12_000, /* freq */ 1500.0, /* amp */ 20_000);

let mut audio = vec![0i16; 180_000]; // 15 s @ 12 kHz
let start = (0.5 * 12_000.0) as usize;
for (i, &s) in frame.iter().enumerate() {
    if start + i < audio.len() { audio[start + i] = s; }
}

// 2. デコードする。new(audio, freq_min, freq_max, sync_min, max_cand)。
// OSDはデフォルトでon。BP-onlyの軽量デコードにしたければ `.osd(false)`。
let results = DecodeRequest::<Ft8>::new(&audio, 100.0, 3_000.0, 1.0, 50)
    .decode()
    .results;
for r in &results {
    if let Some(text) = unpack77(r.message77()) {
        println!("{:7.1} Hz  dt={:+.2} s  SNR={:+.0} dB  {}",
                 r.freq_hz, r.dt_sec, r.snr_db, text);
    }
}
```

実音声は 12 kHz の 15 秒スロットとして届く。他のサンプルレートは
`engine::dsp::resample` で変換する。`DecodeRequest` は `&[i16]` を取る。

---

## 2. デコード API

`mfsk_core::msg::decode_request` の `DecodeRequest` と `SniperRequest`
が、FT8・FT4・FST4 全サブモード（`FrameDecodable` マーカトレイト）の
**公開**デコードエントリポイントである。

その下にある engine 関数（`decode_frame`、`process_candidate_basic`、
`GenericPipelineProtocol` トレイト）は issue #191/#203 以降
`pub(crate)` で、下流がリクエスト型を迂回できないようにしてある。
既定外の `internal-testing` feature がクレート自身の統合テスト向けに
それを開ける。

Q65・WSPR・JT65・JT9・uvpacket・JTTY は独自のエントリポイントを持つ —
[§2.5](#25-独自エントリポイントを持つプロトコル)
(MSK144 のそれは `msk144::decode::decode_slot`)。

### 2.1 `DecodeRequest<P>`

`freq_min..freq_max` を広帯域探索する。構築し、連鎖し、デコードする:

```text
DecodeRequest::<P>::new(audio, freq_min, freq_max, sync_min, max_cand)
    .osd(true)
    .decode()          // -> DecodeOutcome<P>
```

`DecodeOutcome<P>` は `.results: Vec<P::DecodeResult>`、後続呼び出し用の
`.fft_cache`、`.budget: BudgetReport` を持つ。

| メソッド | 引数 | 既定 | 対応 | 効果 |
|---|---|---|---|---|
| `new` | `(audio, freq_min, freq_max, sync_min, max_cand)` | — | 全部 | 広帯域探索 |
| `.freq_hint(hz)` | `f32` | 未設定 | 全部 | この周波数付近の候補を優先。**a-priori パスの QSO 周波数（`nfqso`）でもある:** `.ap_hint()` が両方の呼出符号を固定するとき、その仮説はこの周波数の 50 Hz 以内の候補にだけ試し、未設定なら一切試さない（`ft4_decode.f90` / `ft8b.f90` は常に `nfqso` を持つ）。FT4 はこの窓の候補を、OSD の 3 つ目のスナップショット（`maxosd = 3`）でも復号する。CQ と MyCall だけの仮説は、どこでも試す |
| `.previous_cycle(&[..])` | 復号結果 | 空 | FT8 | 同じ系列の 1 周期前（30 秒前のスロット）の復号結果。WSJT-X の **a7** リスト復号を有効にする: その各組が次に送りうるメッセージを、前回の周波数と DT で照合する（pass id 30）。**a8** は呼び出し不要: `.ap_hint()` に MyCall、HisCall、HisGrid があり `.freq_hint()` があれば動く（pass id 31） |
| `.tx_freq(hz)` | `f32` | 未設定 | FT8 | 送信周波数（`nftx`）: FT8 は両方の呼出符号を固定する仮説を、この周波数の 50 Hz 以内でも試す。他のモードは無視する（`ft4_decode.f90` に `nftx` はない） |
| `.osd(bool)` | `bool` | `true` | 全部 | BP の階段が失敗したときの OSD フォールバック。ホストデコードでは `LlrEffort` は常に `Full` |
| `.strictness(s)` | `DecodeStrictness` | `Normal` | 全部 | 採否閾値のプロファイル。どのノブがどのプロトコルに実際に届くかは [§6](#6-engine-プリミティブ) |
| `.eq_mode(m)` | `EqMode` | `Off` | 全部 | `Off` / `Local`。**入力音声**の性質であって探索の性質ではない |
| `.known(&[..])` | 復号済みの行 | 空 | 全部 | 前パスで見つかったメッセージをスキップまたは減算する |
| `.fft_cache(c)` | 前回の `DecodeOutcome` のキャッシュ | 無し | 全部 | 同じ音声への前方 FFT を再利用 |
| `.noise_blanker(nb)` | `NoiseBlanker` | オフ | `SupportsNoiseBlanker` — **FST4 の全サブモード** | WSJT-X の **NB**（`blanker.f90`）: スロット FFT の前に振幅の大きい標本を 0 にする。`Percent(n)` は `n` %（0..=25）を消す。`Sweep { step, ftol_hz }` は 0, step, … 20 % の各レベルでデコードし、0 より上のレベルは `.freq_hint()` の `ftol_hz` 以内だけを試す（最大 21 回のデコード）。既定はオフで、WSJT-X の既定 NB 0 % と同じ |
| `.ap_hint(&ApHint)` | `&ApHint` | 無し | `SupportsWideBandAp` — **FT8・FT4・FST4 全サブモード** | 事前仮説からメッセージビットを固定 |
| `.sic_rounds(n)` | `usize`、`1..=3` にクランプ | 無し | `SupportsSicRounds` — **FT8, FT4** | 平坦な逐次干渉除去 |
| `.sic_early()` | — | 無し | `SupportsSicEarly` — **FT8** | チェックポイント模倣の早期デコード（3 チェックポイント固定構造） |
| `.also_accept(f)` | `Fn(&Wsjt77Fields) -> bool` | 無し | `SupportsMessageFilter` — **FT8・FT4・FST4 全サブモード** | codec が通すもの **＋** `f` が通すもの — [§2.6](#26-メッセージの受理) |
| `.message_filter(f)` | `Fn(&Wsjt77Fields) -> bool` | 無し | `SupportsMessageFilter` — **FT8・FT4・FST4 全サブモード** | codec の判定を `f` で置き換える — [§2.6](#26-メッセージの受理) |
| `.codec_filter()` | — | FT8 は on、他は off | `SupportsMessageFilter` — **FT8・FT4・FST4 全サブモード** | 既定で判定しないプロトコルで codec 自身の判定を適用する — [§2.6](#26-メッセージの受理) |
| `.contest(on)` | `bool` | `false` | **FT8** | WSJT-X の `ncontest != 0`。FT8 が CRC 後に落とす `/R`・`TU; ` のメッセージを残す — [§2.6](#26-メッセージの受理) |
| `.on_result(cb)` | `FnMut(&Row)` | 無し | 全部 | 見つかった順に行を配信 — [§2.4](#24-ストリーミング配信) |
| `.budget(check)` | `FnMut() -> bool` | 無し | 全部 | 呼び出し側の締切述語 — [§2.3](#23-計算予算) |
| `.sniper(...)` | `(audio, target_hz, max_cand)` | — | `SupportsSniper` — **FT8** | 代わりに `SniperRequest` を作る |
| `.decode()` | — | — | 全部 | 実行 |

**`DecodeRequest::<Ft8>::wsjtx_depth(audio, freq_min, freq_max, sync_min,
max_cand, tier, ap)`** は 2 つ目の*コンストラクタ*（メソッドではない）で、
FT8 専用、`ft8::decode` にある。(OSD, SIC 戦略, AP) の組が
`jt9 -d1/-d2/-d3` と対応するリクエストを作る: `WsjtxDepth::D1` は OSD
オフに `.sic_rounds(2)`、`D2` は OSD に `.sic_early()`、`D3` はさらに
`.ap_hint(ap)` を加える（`ap: Option<&ApHint>` は `D3` でだけ読まれ、
`D1`/`D2` では `jt9` 自身の depth と AP の結合どおり無視される）。`D1` と
`D2` は、WSJT-X 3.0 の `ndepth <= 2` と同じく、候補が越えねばならない
ハード sync (nsync) の下限も 6（二乗メトリックのパスでは 7）から 8 に
引き上げる（#439）。`sync_min` は呼び出し側のもののままである:
busy-band コーパスでは 1.3 が `.sic_early()` の recall を保ったまま
想定外のデコードを 22 件（0.8 のとき）から 6 件に減らし、WSJT-X 3.x が
`-d1/-d2` に使う 2.1 は、そこで recall を 3〜4 ポイント失う
（[`BENCHMARKS.md`](../notes/BENCHMARKS.md) の "The busy-band corpus"）。

**戦略の拡張こそが phantom decode の出所である。** このスイートが
出荷してしまった false-decode バグは2件とも減算パスにあった
（`__staged_sic` の #243、`.sic_early()` の #253）。新しい戦略は
同じ PR で精度ガードとともに出荷すること。

### 2.2 `SniperRequest<P>`

狭帯域・単一目標の探索。`SupportsSniper` で gate されており、
**`Ft8` にのみ実装されている**。

```rust
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::{EqMode, ApHint};
use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
use mfsk_core::msg::decode_request::SniperRequest;
use mfsk_core::msg::wsjt77::{pack77, unpack77};

let msg77 = pack77("CQ", "JA1ABC", "PM95").unwrap();
let tones = message_to_tones::<Ft8>(&msg77);
let frame = synthesize_i16::<Ft8>(&tones, 12_000, /* freq */ 1000.0, /* amp */ 20_000);
let mut audio = vec![0i16; 180_000]; // 15 秒 @ 12 kHz
let start = (0.5 * 12_000.0) as usize;
audio[start..start + frame.len()].copy_from_slice(&frame);

let ap = ApHint::new().with_call1("CQ").with_call2("JA1ABC");
let results = SniperRequest::<Ft8>::new(&audio, /*target_hz*/ 1000.0, /*max_cand*/ 15)
    .eq_mode(EqMode::Local)
    .ap_hint(&ap)
    .decode()
    .results;
assert!(!results.is_empty(), "ラウンドトリップは復号できるはず");
for r in &results {
    let text = unpack77(r.message77()).unwrap();
    println!("{:7.1} Hz  {}", r.freq_hz, text);
}
```

| メソッド | 既定 | 効果 |
|---|---|---|
| `new(audio, target_hz, max_cand)` | — | `target_hz` の ±250 Hz |
| `.search_hz(w)` | 250 Hz | 窓を広げる / 狭める |
| `.sync_min(v)` | モード既定 | sync 閾値 |
| `.osd(bool)` | `true` | `DecodeRequest` と同じ |
| `.strictness(s)` | `Normal` | 同上 |
| `.eq_mode(m)` | `Off` | 同上 |
| `.ap_hint(&h)` | 無し | 同上 |
| `.also_accept(f)` | 無し | 同上 |
| `.message_filter(f)` | 無し | 同上 |
| `.codec_filter()` | on（FT8） | 同上 |
| `.on_result(cb)` | 無し | 同上 |
| `.budget(check)` | 無し | 同上 |
| `.decode()` | — | 同じ `DecodeOutcome<P>` |

SIC 版は無い — sniper 探索は本質的に単一候補である。

**これを sniper たらしめているのは窓であって、ヒントではない。**
±250 Hz の探索が存在するのは、運用者がトランシーバの*アナログ*
ルーフィングフィルタを絞った（~500 Hz を持つ機種として Yaesu FTDX101MP
と FTDX10 が代表例）うえで、キャリア周波数が既知の局に向けているから
である。届く音声は既に帯域制限されており、デコーダはハードウェアに
合わせているだけである。「相手が分かっているときに使う便利機能」では
**ない**。

**`.ap_hint()` はここでも使える** — 上の表にあるとおりで、
`SniperRequest` の `.decode()` はそれをデコーダまで渡す。ただし AP は
窓と直交する機能であって、sniper が*何であるか*の一部ではない。同じ
ヒントは広帯域の `DecodeRequest` にも、FT8・FT4・FST4 全サブモードに
届く。かつて両者は結合していた — AP は候補ループを `if has_ap` で
抜けるエンジン経由でしか使えず、*ヒントを持つこと*が探索を単一目標に
していた — 取り除かれたのはその結合であって、ここでの AP の利用可否
ではない。

FT4 と FST4 の sniper エントリポイントは 2026-09-13 まで存在したが
削除された。広帯域パスがこのクレートの全モードにとっての本線であり、
sniper 無しで WSJT-X 忠実でないなら、それは広帯域パス側のバグである。
測定を含む詳細は
[`DESIGN_RATIONALE.md`](../notes/DESIGN_RATIONALE.md)。

### 2.3 計算予算

`.budget(check)` は候補の合間にポーリングされる述語を取る。
**ライブラリ側は一切時計を読まない** — 締切は述語が何と比較するか次第で
あり、これが wasm から、またスロット途中でサスペンドされたプロセスから
使える理由になっている。

`DecodeOutcome::budget` は `BudgetReport` で、打ち切りが何を残したかを
告げる: スキップした候補数、実行したステージ数、そしてスキップした中で
最良の候補がどれだけ良かったか — 「何も無かった」のか「有望な候補を
残したまま時間切れになった」のかを呼び出し側が区別できる。

FT8・FT4・FST4 全サブモードが対応する（C 側に公開されている
`MFSK_CAP_BUDGET` と同じ事実）。

### 2.4 ストリーミング配信

`.on_result(cb)` は、呼び出しが返す `Vec` に加えて、見つかった順に各行を
配信する — 長いスロットが終わる前に画面へ何か出したい UI 向け。

配信順と重複除去の契約は**ここでは繰り返さない**。
[`STREAMING.md`](STREAMING.ja.md) が正式な説明で、規範的なのは
`DecodeRequest::on_result` 自身の doc comment である。一行で言えば:
逐次デコードは呼び出しが返す行をその順で正確に配信し、並列デコードは
完了順に配信するため、返り値の `Vec` では既に除去済みの重複が一時的に
見えることがある。

各プロトコルが同じ形を独自のエントリポイントで提供している:
`wspr::DecodeRequest`・`jt9::DecodeRequest`・`jt65::DecodeRequest`・
`q65::{DecodeRequest, SniperRequest, MultiPeriodRequest}` の
`.on_result(cb)`。WSPR は完全一致ではなく並列側の契約になる ——
[`STREAMING.ja.md`](STREAMING.ja.md) §3b を参照。JTTY にはリクエスト
ビルダが無く、音声呼び出しの内側からコールバックで配信する:
`jtty::rx::Stream::push(samples, &mut |update| …)`（および `finish`）が、
呼び出し側のスレッドでそれを呼ぶ —
[§2.5](#25-独自エントリポイントを持つプロトコル)。

### 2.5 独自エントリポイントを持つプロトコル

WSPR はスロットを wsprd と同じ 375 Hz ベースバンドにデシメートし、
そこで wsprd 自身の粗探索と 3 回のデコードパスを走らせる。共有の FT 系
パイプラインとはステージ構成が異なるため、`wspr` モジュールが独自の
エントリポイント `wspr::DecodeRequest` / `wspr::SniperRequest`
（issue #403、14 個のフリー関数を置き換えた）を用意している。ただし内部で使っている FEC (`ConvFano`) と
メッセージコーデック (`Wspr50Message`) は `Wspr: Protocol` の
関連型として宣言済みで、抽象の枠組みからは外れていない。

```rust
# #[cfg(feature = "wspr")] {
use mfsk_core::wspr::DecodeRequest;
use mfsk_core::wspr::tx::synthesize_type1;
use mfsk_core::msg::WsprMessage;

// WSPR Type 1 フレームを合成 (120 秒 @ 12 kHz スロット)。
let samples_f32 = synthesize_type1("K1ABC", "FN42", 37, 12_000, 1500.0, 0.3)
    .expect("valid message");

let decodes = DecodeRequest::new(&samples_f32, /*sample_rate*/ 12_000).decode();
assert!(!decodes.is_empty(), "ラウンドトリップは復号できるはず");
for d in decodes {
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

`DecodeRequest::new` が粗同期 (周波数×時刻×ドリフト探索) を込みで
スロット全体をスキャンする。`.nominal_start()`・`.params()`・
`.on_result()`・`.table(&mut WsprCallsignTable)` も取る。スロットを
またいで持ち回したテーブルがあると、前のスロットの Fano デコードで
確認済みの局を OSD が再発見できる —— wsprd が自身のサンプルファイルで
W3BI を −25 dB で拾えるのはこの仕組みによる。

周波数・開始サンプルが既知の場合は
`DecodeRequest::sniper(samples, rate, start_sample, freq_hz).decode()`
で粗同期を省略できる。`SniperRequest::baseband(idat, qdat, …)` は
呼び出し側でデシメート済みのベースバンドに対して同じことを行い、
`.drift()`・`.nblocks()`・`.confirmed()`・`.refine_drift()` ——
スキャン自身の各パスが候補ごとに設定するつまみ —— を取る。CoreS3 の
WSPR 受信機は自前の候補ループからこれを駆動している。



**JT9** のビルダーは `jt9::DecodeRequest` ひとつ（issue #403）。形は
下の Q65 と同じだが、JT9 はサブモードがひとつなのでジェネリックではない。
`DecodeRequest::new(audio, sample_rate)` は
`jt9::search::default_search_params()` でバッファ全体を探索し、
`.nominal_start()`・`.params()`・`.depth(Jt9Depth)`・`.on_result()` で
調整する。既知のアラインメントでの単点デコードは
`DecodeRequest::sniper(audio, rate, start_sample, freq_hz).decode()`。
こちらは `Jt72Message` だけを返す — この経路には同期探索・AFC・SNR 推定が
なく、報告できるものがないため。置き換えられた `decode_scan*` /
`decode_at` の 6 つのフリー関数は削除した。

```rust
# #[cfg(feature = "jt9")] {
use mfsk_core::jt9::{DecodeRequest, Jt9Depth};
use mfsk_core::jt9::tx::synthesize_standard;

let audio_f32 = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, 1500.0, 0.3)
    .expect("pack + synth");
let decodes = DecodeRequest::new(&audio_f32, 12_000)
    .depth(Jt9Depth::Deep)
    .decode();
assert!(!decodes.is_empty(), "ラウンドトリップは復号できるはず");
# }
```

**JT65** も同じ組 `jt65::DecodeRequest` と `jt65::SniperRequest` を持ち
（issue #403）、9 つのフリー関数を置き換えた。JT65 固有の軸は
Reed-Solomon の走らせ方で、既定は硬判定、どちらのビルダーでも
`.chase(ChaseParams)` で stochastic Chase 探索、sniper では
`.erasures(&[0, 8, 16, 24, 32])` で決定的な消失ラダーになる。sniper で
両方を呼んだ場合は後から呼んだ方が有効。

```rust
# #[cfg(feature = "jt65")] {
use mfsk_core::jt65::DecodeRequest;
use mfsk_core::jt65::tx::synthesize_standard;

let audio_f32 = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, 1270.0, 0.3)
    .expect("pack + synth");
let decodes = DecodeRequest::new(&audio_f32, 12_000).decode();
assert!(!decodes.is_empty(), "ラウンドトリップは復号できるはず");
for d in decodes {
    println!("{:7.2} Hz  {:+.0} dB  {}", d.freq_hz, d.snr_db, d.message);
}
# }
```

Chase 探索（`jt65::chase`、issue #169）は WSJT-X の stochastic Chase
デコーダ `ftrsdap` の忠実な移植（マジックナンバーも含む）。AWGN スイープでは 50% 交差を −22.5 dB から
−23.5 dB に下げ、その代わり即座に復号できない候補ごとに最大
`ChaseParams::max_trials` 回の RS 試行を払う。

**Q65** は `mfsk_core::q65::decode_request` に3つの汎用ビルダを持ち、
`msg::decode_request` と同じ形で、10 サブモード ZST すべてに実装された
sealed な `Q65SubMode` マーカを介して汎用化されている:
`DecodeRequest<P>`（広帯域スキャン）、`SniperRequest<P>`（既知の
`(start_sample, base_freq_hz)`）、`MultiPeriodRequest<P>`（複数スロット
平均）。`.ap_hint()`・`.ap_list()`・`.fading()` は capability gate された
マーカトレイトではなく素の inherent メソッドである — Q65 は全サブモードが
全機能を一様に持つため。下層の `q65::rx` 関数群は `pub(crate)`。どのビルダが
どのメソッドを取るか（`q65/decode_request.rs`。ビルダに無いメソッドは、黙って
何もしないのではなくコンパイルエラーになる）:

| メソッド | `DecodeRequest` | `SniperRequest` | `MultiPeriodRequest` |
|---|---|---|---|
| `.ap_hint(&ApHint)` | yes | yes | no |
| `.ap_list(&codewords)` | yes | yes | yes |
| `.fading(model, b90_ts)` | yes | yes | no |
| `.pileup(bool)` | yes | yes | no |
| `.max_drift(bins)` | yes | no | no |
| `.eme_delay(bool)` | yes | no | yes |
| `.rx_freq(hz)` / `.ftol(hz)` | yes | no | no |
| `.hash_table(Arc<CallsignHashTable>)` | yes | yes | yes |
| `.on_result(cb)` | yes | yes | yes |
| `.decode()` の戻り値 | `Vec<Q65Result>` | `Option<Q65Result>` | `Vec<Q65Result>` |

`.hash_table()` は、`<...>` というハッシュ化コールサイン（Type 4）の
プレースホルダを解決するセッションの `CallsignHashTable` である。未設定なら
未解決のまま残る。`Arc` で共有されるので、セッション中の全 `decode()` に同じ
テーブルを渡してもリファレンスカウントを増やすだけで済み、テーブルは呼び出し側が
所有して育てる。

**`dt_sec`、`SearchParams`、`SyncCandidate`。** WSPR・JT9・JT65・Q65 では、
結果の `dt_sec` は**公称開始位置** — リクエストの `nominal_start` サンプル
（WSPR は固定の 1.0 s `TX_START_OFFSET_S`） — から測り、符号付きなので、
早く始まったフレームは負になる（#397。以前は Q65 と JT65 でバッファ先頭から
測っており、0.5 s 早いフレームが −0.013 s と +0.442 s になっていた）。
`Q65Result` / `Jt65Result` / `Jt9Result` の `to_decoded` はこのフィールドを読み、
もはや `(sample_rate, nominal_start_sample)` を取らない。`Jt9Result` は
`dt_sec` を得た。`msg::decoded::dt_from_samples` は無くなった。
**0.12 での破壊的変更**で、残りは
[0.12 での破壊的変更](#012-での破壊的変更)にある。スキャン系のモードは
`engine::search` にある 1 つの粗探索の語彙を共有する（#394）:
`SearchParams { freq_min_hz, freq_max_hz, time_tolerance_early_sec,
time_tolerance_late_sec, score_threshold, max_candidates }`（窓は Q65 のそれが
非対称なので**秒**の early/late の組である。`SearchParams::symmetric(..)` は
両方を設定する）と `SyncCandidate { start_sample,
freq_hz, score }`。`freq_hz` は tone 0 で、`.dt_sec(nominal, rate)` が
変換する。`SearchParams::default()` は無い: 既定値はモード固有のデータなので、
各モードが `search::default_search_params()` を持つ
（Q65: 200-3000 Hz、±1.0 s、8 候補、threshold 0.1）。FT8・FT4・FST4 は、
`start_sample` の代わりに `dt_sec` を持つ `engine::sync::SyncCandidate` を
意図的にそのまま使う。

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

ビルダーは3つ。いずれも `SupportsMessageFilter` 上にあり、対象は
**`Ft8` / `Ft4` / 全 FST4 サブモード**である:

```rust
use mfsk_core::ft8::Ft8;
use mfsk_core::msg::decode_request::DecodeRequest;

/// 配備先が知っていて ITU 許可リストが知らないもの。
fn is_special_event_call(call: &str) -> bool {
    call.starts_with("8J")
}

let audio = vec![0i16; 180_000]; // 15 s @ 12 kHz

// codec の判定 ＋ それが知らないコールサイン。クロージャは復号済み
// メッセージを受け取るので、`callsigns()` は厳密にコールサイン欄だけで
// あり、grid や report が紛れ込むことはない。
let widened = DecodeRequest::<Ft8>::new(&audio, 200.0, 3000.0, 1.5, 20)
    .also_accept(|m| m.callsigns().all(is_special_event_call))
    .decode();

// 一切の判断をしない — CRC を通ったメッセージは全部、ファントム込みで。
// これが本家の受理規則そのものである。
let unfiltered = DecodeRequest::<Ft8>::new(&audio, 200.0, 3000.0, 1.5, 20)
    .message_filter(|_| true)
    .decode();

// 無音には実信号も CRC 生存者も無いので、フィルタ無しの方も空で返る。
assert!(widened.results.is_empty());
assert!(unfiltered.results.is_empty());
```

`.also_accept(f)` は判定を広げるだけで、減らすことはない。
`.codec_filter()` は判定のみを適用する — 既定でオフのプロトコルで
有効化する一行の手段である。`.message_filter(f)` は判定を丸ごと置き換え、
置き換えられる側はそこに到達した CRC 生存者のおよそ 2/3 を落としている
ので、緩い `f` はファントム行を表に出す。

**FT8 は、ポリシーに関わらず `/R` と `TU; ` のメッセージも落とす。** #439 以降、
FT8 は `ft8b.f90`（WSJT-X 3.0 以降）が CRC の直後にすることと同じことをする:
コンテスト中でなければ、`/R` を含む、または `TU; ` で始まる標準または RTTY
Roundup のメッセージは捨てられ、そのパスは次へ進む。これはポリシーより前に
あるので、`.message_filter(|_| true)` でもそれらの行は戻らない。戻すのは
`.contest(true)` で、`CALL1/R CALL2` や `TU; CALL1 CALL2` が実トラフィック
になるコンテストではこれが正しい設定である。

**既定でオンなのは FT8 と FT4。理由は減算である。** 受理したものを減算する
経路では、誤ったデコードは表示上の問題では済まない。`.sic_rounds()` と
`.sic_early()` は、次に探す前に復号した波形を音声から取り除く。
`qso3_busy.wav` での実測では、判定を切ると `.sic_early()` がファントム
`CQ G47OXF RD84` を受理して減算し、その下にいた実信号 `CQ EA2BFM IN83` を
失う — 18/18 が 17/18 になる。単一パス経路では、同じ判定が `max_cand = 200`
で 2 行のゴミを落とし、**実機が使う深さでは 1 件も落とさない**。

FT4 も同じ CRC-14 と同じ SIC 経路を持つので、自前の実測が揃った時点で
同じ扱いになった — `ft4sim` コーパスの閾値帯（−21..−13 dB、ITU-R 4チャネル）
720 スロットで、ファントム行 **7 → 2**、golden 行 **353 → 354**、
50% 交差 SNR は4チャネルとも **0.00 dB 変化なし**。動いた唯一の recall セルは
*増える*向きで、拒否されても候補ラダーが止まらないため。このコーパスが
試せないのは許可リスト自身のリスク（全スロットが同一コールサイン）なので、
珍しいプレフィクスを受ける運用は `.also_accept()` で広げる。

**FST4 はオフのまま。** 測っていないからではない: CRC-24 により偽陽性率が
他の2つより 512 倍低く、判定が落とすものがほとんど無い一方、recall を失う
可能性だけは同じだからである。

**未使用時のコストはゼロ。** ポリシーは `.on_result()` / `.budget()` の
ような `&dyn Fn` ではなく型パラメータである。3つとも呼ばない request は
ゼロサイズ型 `DefaultPolicy` を持ち、既定でフィルタしないプロトコルでは
**メッセージの復号すら行われない** — どちらの条件もコンパイル時定数である。
前者2つのフックは*デコード*ごとに1回発火するが、こちらはメッセージ段に
到達した候補ごとに発火する。型パラメータにする価値があるのはそのため。

---

## 3. プロトコル

### 3.1 プロトコル毎の汎用 vs 専用

最初に読むべき地図がこれ。各行が 1 プロトコルで、各セルはその層が
**汎用**（汎用コアからそのまま再利用） か **専用** (そのプロトコル
自身のモジュールにあるコード) かを示す。

| プロトコル | FEC コーデック | メッセージコーデック | Sync mode | デコード入口 |
|-----------|---------------|--------------------|-----------|-------------|
| **FT8**  | 汎用 `Ldpc174_91` | 汎用 `Wsjt77Message` (77 bit) | `Block` — 3×Costas-7 | 汎用 `DecodeRequest`、内部は FT8 専用 `ft8::decode_block` エンジン [^ft8] |
| **FT4**  | 汎用 `Ldpc174_91` | 汎用 `Wsjt77Message` (77 bit) | `Block` — 4×Costas-4 | 汎用 `DecodeRequest` / `engine::pipeline` |
| **FST4** | 汎用 `Ldpc240_101` | 汎用 `Wsjt77Message` (77 bit) | `Block` — 5×Costas-8 | 汎用 `DecodeRequest` / `engine::pipeline` |
| **WSPR** | 専用 `ConvFano` (畳み込み r=½ K=32 + Fano) | 専用 `Wspr50Message` (50 bit) | 専用 `Interleaved` [^wspr] | 専用 `wspr::decode` |
| **JT9**  | 専用 `ConvFano232` (畳み込み、206 bit 枠) | 汎用 `Jt72Codec` (72 bit) | `Block` (長さ 1 スロット) | 専用 `jt9` 入口 |
| **JT65** | 専用 `Rs63_12` (RS GF(2⁶)、消失対応) | 汎用 `Jt72Codec` (72 bit) | `Block` (長さ 1 スロット) | 専用 `jt65` 入口 |
| **Q65**  | 専用 `Q65Fec` + GF(64) 上の QRA コーデック [^q65] | 専用 `Q65Message` (77 bit) | `Block` | 専用 `q65::rx` + Q65 ローカル `DecodeRequest` |
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
  [§2.5](#25-独自エントリポイントを持つプロトコル)。

> この表は `mfsk-core/tests/common_selftest.rs` のコード共有ラチェット、
> `README.md` の共有率パラグラフ、`lib.rs` 自身のドキュメントが揃って
> 辿り着く先の正本である。ここを変えるならそれらも変わる。

[^ft8]: FT8 は FT4/FST4 と同じく汎用 `DecodeRequest` ビルダーを使うが、
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
  nsync の下限（`> 6`、二乗メトリックのパスでは `> 7`、`WsjtxDepth::D1/D2` では
  `> 8`）を持ち、コンテスト外では `/R` と `TU; ` のメッセージを捨てる
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
  （修正後は 9.7e-5）。`.freq_hint()` の 50 Hz 以内では FT4 は 3 つ目の OSD
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
  `.noise_blanker()` は WSJT-X の **NB** である（[§2.1](#21-decoderequestp)）:
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
  [§2.5](#25-独自エントリポイントを持つプロトコル) を参照。定数は trait 定数
  ではなく `jtty` にある（`NSPS`、`SYNC_SYMBOLS`、`FRAME_SYMBOLS`、
  `MAX_FRAMES` = 16）。GFSK パルスは `engine::dsp::gfsk` ではなく `jtty::tx` に
  ある独自の 1 始まりのものを持つ: このパルスは #482 まで 1 サンプル早く、
  JTTY の波形チェック（upstream に対して 4.9e-2）がそれを暴いた。

### 3.4 デコード戦略

どのプロトコルも同じ基本フローを走るが、その周りを包む*戦略*が異なる。
大半は単一パスである。1つの FEC フレームに対して複数の並列受信系を
持つのは Q65 だけで、MSK144 はスロットモデル自体をバースト走査に
置き換え、JTTY は逐次入力の受信器に置き換えている。

| プロトコル | 既定の戦略 | 任意の戦略 |
|----------|-----------|-----------|
| **FT8** | 単一パス BP + OSD | AP iaptype ループ (1–12)、SIC 1–3 ラウンド、`.sic_early()`、sniper、**a7 / a8 リストデコーダ**（pass id 30 / 31。FT8 の全戦略の最後に走る。a7 は `.previous_cycle()`、a8 は MyCall・HisCall・HisGrid を持つ `.ap_hint()` と `.freq_hint()`）、`wsjtx_depth(…)` プリセット |
| **FT4** | 単一パス BP + OSD | SIC 1–3 ラウンド、フルスロット・コヒーレント sync (`sync2d`) |
| **FST4** | 単一パス BP + OSD | フルスロット2段コヒーレント sync 探索、ノイズブランカ（`.noise_blanker()`、固定 % またはスイープ） |
| **WSPR** | 単一の専用パス（四半シンボル・スペクトログラム走査） | — |
| **JT9** | 単一の専用パス | — |
| **JT65** | 単一の専用パス | RS 消失復号、確率的 Chase デコーダ |
| **Q65** | `(Δf,Δt,b90)` グリッド + Lorentzian フェージング BP（スキャン） | AP ヒント、明示的な高速フェージング、AP リスト、マルチ周期、**q3** リスト復号（`.ap_list().rx_freq()`）、Max Drift、Pileup、EME 遅延 |
| **MSK144** | T/R 周期全体のバースト走査 | — |
| **JTTY** | ストリーミング: sync サーフェス、候補、4 段の list-WAVA ラダー、ゲート。デコードしたフレームを減算し、遡及再スイープし、フレームをメッセージに組み立てる | `Params::subtract` をオフ（単一信号の受信器） |

**事前情報デコード (AP) は sniper の機能ではなく一般の選択肢である。**
AP は候補ごとの ladder の最後の一段である — FT4 と FST4 全サブモードでは
`process_candidate_basic` の、FT8 では FT8 自身の ladder の。
`msg::pipeline_ap` は仮説生成だけで自前のエンジンを持たない。かつて偶然 sniper と結合しており、それがデコードの
大半を失わせていた — 実測は
[`DESIGN_RATIONALE.md`](../notes/DESIGN_RATIONALE.md) にある。

**Q65 の戦略の選び方:**

| 状況 | 戦略 | ビルダ呼び出し | 閾値の利得 |
|---|---|---|---|
| 候補1点が既知、内容は未知 | AWGN Bessel + BP（点デコードのみ） | `SniperRequest::<P>::new(...).decode()` | ベースライン |
| 既定のスキャン — チャネルも内容も未知 | `(Δf,Δt,b90)` グリッド探索 + Lorentzian フェージング BP | `DecodeRequest::<P>::new(...).decode()` | WSJT-X 忠実な既定 |
| コールサインやレポートが既知、地上波 | AP ヒント BP | どちらかのビルダで `.ap_hint(&ap)` | 約 2 dB |
| ドップラー拡散、モデルを明示（マイクロ波 EME、10 Hz 以上の拡散） | 高速フェージング metric + BP、`(b90_ts, FadingModel)` は呼び出し側が指定 | どちらかのビルダで `.fading(model, b90_ts)` | 拡散チャネルで 5–8 dB |
| コールサイン対は既知、QSO 状態は無し、地上波 | AP リストのテンプレート照合 | どちらかのビルダで `.ap_list(&candidates)` | 約 3 dB |
| コールサイン対と受信周波数が既知（WSJT-X の q3） | 受信周波数付近にあるリストの全メッセージの 85 シンボル sync を取り、そのあとリスト復号 | `DecodeRequest` で `.ap_list(&codewords).rx_freq(hz)`（＋ `.ftol(hz)`） | `q65sim` Q65-30A、−24 / −26 / −28 / −30 dB の各レベル 20 ファイル: 20 / 20 / 7 / 2、`jt9 -3 -d 1` も同じファイルで同じ |
| 複数 T/R 周期にまたがる微弱・電離層散乱信号 | マルチ周期 EMA 平均（3 段カスケード） | `MultiPeriodRequest::<P>::new(...).decode()` | 単一周期のどの戦略でも取れない信号を拾う |

`.ap_list()` と `.fading()` は下層エンジンでは排他であり、`.decode()` は
`ap_list > fading (+ ap_hint) > ap_hint > plain` の順で解決する。
`q65::Q65History` は WSJT-X の `q65_hist` で、アプリケーションが保持する。
デコードのたびに `.record(&result)` で記録し（最新 100 件を保持）、
`.lookup(rx_freq_hz)` は 10 Hz 以内の最新のデコードから DX コールを返す
（メッセージにグリッドがあればグリッドも返す）。WSJT-X は DX コール未入力で
手動の Decode Again を行ったときにこれを使い、オペレータがコールを入力しなくても
フル AP リスト（`standard_qso_codewords`）を作る。
`q65::Q65Callers` と `contest_codewords` はコンテストモード版である
（`q65_hist2` / `q65_set_list2`）。グリッド付きで呼んできた局を最大 50 局、
アプリケーションが保持する（`record(freq, msg, now)`、`expire(now)`）。
そこから、各局について `MyCall Caller Grid` / `R Grid` / `RRR` / `RR73` /
`73` を 78 ビット目なしとありの両方で作ったフル AP リストを作り、
`.ap_list()` に渡す。
`MultiPeriodRequest` は T/R スロットごとに1本の `&[&[f32]]` を取り、
Rust 専用である（C ABI には無い）。各フロントエンドが実際に何をして
いるか、既定のスキャンがなぜ素の Bessel パスではないのかは
[`DESIGN_RATIONALE.md` §4](../notes/DESIGN_RATIONALE.md#4-q65s-decoder-strategies-and-what-each-is-for)。

**Q65 Pileup（WSJT-X 3.2）。** Pileup モードの局は、相手の直前の送信を
受信できたことを Q65 の予備の 78 ビット目で知らせる。
`Q65Result::copied_last_tx` がそれを報告し（WSJT-X は行に `#` を付ける）、
`encode_channel_symbols_flagged` / `synthesize_standard_flagged_for` で送れる。
どちらのビルダーでも `.pileup(true)` を指定すると上流のこのモードの AP 方針に
なる: 両方の呼出符号だけを指定した `.ap_hint()` は、このビットを 0 に固定せず
自由にするので、フラグ付きの応答にも一致する。指定しなければ、そのような
ヒントはフラグ付きの応答を受け付けない。Pileup 以外の WSJT-X と同じである。
`.ap_list()` のテンプレートは `q65_set_list.f90` と同じくビットを立てない。

**Q65 Max Drift。** `q65::DecodeRequest` の `.max_drift(bins)` は
WSJT-X の Max Drift 設定（0..50、既定はオフ）である。同期探索で、フレーム
全体にわたる最大 `bins` ビン（1 ビン = 1 ボー）の直線的なトーンのドリフトを
試し（`q65_ccf_22`）、グリッドデコードで見つかったドリフトを取り除く
（`q65_loops` の `twkfreq`）。周波数ビンあたりの探索コストは通常の
`2*bins+1` 倍になる。WSJT-X は有効な間、探索窓を受信周波数 ± F Tol に
絞るので、`SearchParams` も同じように絞ること。通常のスキャンと
`.ap_hint()` のスキャンに効く。

**Q65 の時間窓と EME 遅延。** `default_search_params()` は公称開始の
-1.0 .. +1.0 s を探索する。WSJT-X の GUI と同じである（`q65.f90` の
`lag1`/`lag2`）。`q65::DecodeRequest` と `MultiPeriodRequest` の
`.eme_delay(true)` は "Decode at 52 s" の EME 遅延に当たり、月面反射の往復分
として後ろ側の端を +5.5 s（Q65-15 は +4.0 s）に広げる。`dt_sec` はどちらの
リクエストでも公称開始からの値である。公称開始を持たない `SniperRequest` は
バッファ先頭からの値を返す。

**Q65 の q3 リスト復号。** `.ap_list(&codewords).rx_freq(hz)`（`.ftol(hz)` 付き、
既定は `jt9` CLI と同じ 10 Hz）は WSJT-X の q3 デコードである。受信周波数の
F Tol 以内で、リストの各メッセージの 85 シンボル全部を使って同期を取り
（`q65_ccf_85`）、高速フェージング指標で `b90` を掃引しながらリスト復号する
（`q65_dec_q3`）。これを最初に実行し、その後で帯域の残りをスキャンする。
`.max_drift(50)` のときは、受信周波数で何も復号できなければ、そこで見つかった
ドリフトを取り除いたスペクトルでもう一度実行する（"w3sz" の段階 5）。
`.rx_freq()` がない場合の `.ap_list()` は、スキャンの代わりに候補ごとに
テンプレートを照合する crate 独自の方式である。

---

## 4. モジュールとクレートの地図

```text
mfsk_core
├── engine/           Protocol trait 群、DSP、sync、LLR、equaliser、pipeline
│   ├── protocol.rs     ModulationParams / FrameLayout / Protocol / FecCodec / MessageCodec
│   ├── dsp/            resample · downsample · gfsk · cpfsk · envelope · subtract ·
│   │                   msk · analytic · ddc · fir_decimate · polyphase · dotprod ·
│   │                   symbol_fft · blanker · 固定小数点 FFT カーネル
│   ├── fft.rs          FftPlanner トレイトと extern factory (EMBEDDED.md 参照)
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
│                       msg::decode_request::DecodeRequest/SniperRequest 経由)
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
├── msg/              メッセージコーデックと公開デコード API
│   ├── decode_request.rs DecodeRequest / SniperRequest — §2
│   ├── decoded.rs      Decoded — 公開の出力行
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
├── ft8/              FT8 ZST + decode + decode_block + wave_gen
│   ├── list_decode.rs  WSJT-X の a7 / a8 リストデコーダ (pass id 30 / 31)
│   └── acquire.rs      実電波の音声からの cold スロット位相取得 (#356)
├── ft4/              FT4 ZST + decode
├── fst4/             FST4 ファミリ — 5 sub-mode ZST (15/30/60A/120/300) + decode
├── wspr/             WSPR ZST + decode + synth + spectrogram search + ddc
├── jt9/              JT9 ZST + decode
├── jt65/             JT65 ZST + decode (+ 消失対応 RS、chase)
├── q65/              Q65 ファミリ — 10 sub-mode ZST + decode + synth
│   ├── decode_request.rs DecodeRequest / SniperRequest / MultiPeriodRequest (§2.5)
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
   と、パイプライン全体を駆動する汎用 `DecodeRequest`/`SniperRequest`
   ビルダー。
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
| 既存モードと同じ FEC とメッセージ（別の FST4 サブモード） | 数値定数だけが異なる新しい ZST。`Fec`/`Msg` は型エイリアス。`DecodeRequest::<P>` パイプライン全体がそのまま動く |
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
| `fir` / `dotprod` | ポリフェーズ FIR と、extern フックが置き換えるドット積カーネル |

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
今も正しい経路だが、`DecodeRequest::<Ft8>` と `SniperRequest::<Ft8>` は内部で
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
の下だけ。`DecodeRequest`/`SniperRequest` を使うこと。

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

### 0.12 での破壊的変更

0.11 の呼び出し側が変更すべきこと（全文と移行表: `CHANGELOG.md` の
`## 0.12.0`）。特記しない限り出力はビット単位で同一である。

| 領域 | 0.11 | 0.12 |
|---|---|---|
| デコード入口、JT9 (#403) | `decode_scan*`、`decode_at`（6 関数） | `jt9::DecodeRequest::new(..).decode()`、`::sniper(..)` |
| デコード入口、JT65 (#403) | `decode_scan*`、`decode_scan_chase*`、`decode_at*`、`chase::decode_at_with_chase`（9 個） | `jt65::DecodeRequest` / `SniperRequest`、`.chase(..)`、`.erasures(..)` |
| デコード入口、WSPR (#403) | 14 関数、`decode_at_baseband_nblocks_gated_drift` とその仲間。`decode_scan_subtract*` は public | `wspr::DecodeRequest` / `SniperRequest`。SIC の組は `internal-testing` の背後だけ |
| 合成 (#391) | `ft8::wave_gen::tones_to_*`、`ft4::encode::*`、`fst4::encode::*`、`wspr::tx::synthesize_audio`、`q65::synthesize_audio_for` … | `FskWaveform` 上の `engine::tx::synthesize::<P>` / `synthesize_into` / `synthesize_i16[_into]` / `synth_len` |
| トーン (#391) | モード毎の `message_to_tones`、FT8 のものは `&[u8]` → `[u8; 79]` | `engine::tx::message_to_tones::<P>(&[u8; 77]) -> Vec<u8>`。`DecodeResult::message77()` は `&[u8; 77]` を返す（`*r.message77() == m77` で比較する） |
| Gray code (#391) | `jt65::{gray6, inv_gray6}` | `engine::gray::{gray, inv_gray}(n, bits)`。`fst4::encode::append_crc24` → `fec::ldpc240_101::append_crc24` |
| JT65 復調器 (#390) | タプルを返す 4 つの `demodulate_aligned*` 関数 | `jt65::demodulate_aligned(..)?` は `Jt65Demod`（`.symbols`、`.conf`、`.second_symbols`、`.rel`、`.raw_pwr`、`.snr_db`）を返す |
| `dt_sec` (#397) | Q65 / JT65 はバッファ先頭から。JT9 には無かった | どこでも公称開始から。`to_decoded` は引数を取らない。`Jt9Result::dt_sec` が新設。`dt_from_samples` は無くなった |
| 探索型 (#394) | 4 つの `SearchParams` / `SyncCandidate`、`SearchParams::default()`、`time_tolerance_sec`、WSPR の `time_tolerance_symbols` | `engine::search` の re-export。モード毎の `default_search_params()`。`time_tolerance_early_sec` / `_late_sec` は秒単位 |
| Q65 の窓 | `default_search_params()` は −1.0 … +5.5 s | `q65.f90:127-130` と同じ −1.0 … +1.0 s。`.eme_delay(true)` で遅い側の到達範囲を復元する |
| LDPC BP (#417) | `fec::ldpc::bp::bp_decode_nms`、`bp_decode_nms_q11`、`llr_f32_to_q11` | `bp_decode_nms_with_scratch`、または `bp_decode_generic_nms::<Ldpc174_91Params, T>`。`Q11i16::from_f32(x).0`。カーネルごとに本体は 1 つ |
| `sync_cv` (#414) | FT8 のものは二乗和の平方根 | 全プロトコルで母集団の CV。したがって FT8 の値は以前の 1/√3 になる |
| FST4 OSD (#456) | 101 ビット全部を探索した | `osd_decode_npre_generic(.., partial_crc: Option<PartialCrc>)`。FST4 は (240, 91) 部分符号を渡す |
| 既定の挙動 | FT4 の `sync_min` 1.2 / `max_cand` 100。FT4 のメッセージポリシーはオフ | 1.18 / 200 (#440)。オン (#383) |

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
`CostasAbsolute`（FT8、FST4。雑音に固定値は無い）、`BaselineNormalised`
（FT4: スペクトルをフィットしたベースラインで割るので、雑音は ~1.0 にあり、
それ未満の閾値はあらゆるピークを通す）、`SyncFraction`（WSPR、JT9、JT65、
Q65: sync の電力を sync と雑音の和で割った 0‥1 の値で、既定は
`DEFAULT_SCORE_THRESHOLD` = 0.1）。`sniper_max_cand_cap` は sniper 経路が
`max_cand` に黙って適用する上限である（FT4: 15）。

`profile.defaults` は、自分で選ばない呼び出し側が得るもの
（このクレートのホスト設定値で、C ABI の `mfsk_mode_defaults` も同じものを返す）:

| エントリ | 帯域 (Hz) | `sync_min` | `max_cand` | 出所 |
|---|---|---|---|---|
| FT8 | 100-3000 | 0.8 | 60 | `FT8_PROFILE` |
| FT4 | 300-2700 | 1.18 | 200 | WSJT-X 3.x `ft4_decode.f90` の `syncmin` / `MAXCAND`（#440。以前は 1.2 / 100） |
| FST4（5 つすべて） | 100-3000 | 0.8 | 50 | `FST4_PROFILE` |
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
