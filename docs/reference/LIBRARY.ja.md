# mfsk-core — Rust API リファレンス

> **English:** [LIBRARY.md](LIBRARY.md)

WSJT-X の微弱信号デコーダ群 — FT8・FT4・FST4・WSPR・JT9・JT65・Q65・
MSK144 — を1つの汎用コアの上に純 Rust で再実装したもの。コア
(`engine` / `fec` / `msg`) はプロトコル非依存で、各プロトコルは FEC
コーデック・メッセージコーデック・sync モードをそこへ挿す zero-sized
type である。配線済みの全プロトコルが同じ受信フロー
`coarse-sync → refine → LLR → FEC decode → message unpack` を通り、
その上にプロトコル毎の戦略が重なる。

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
mfsk-core = { version = "0.11", features = ["ft8", "ft4", "wspr"] }
```

必要なプロトコル feature だけを入れる。以下の例は説明のために複数を
有効にしている。

**FT8 スロットをデコードする。** フレームを合成してから復号する:


```rust
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::wave_gen::{message_to_tones, tones_to_i16};
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::{pack77, unpack77};

// 1. FT8 フレームを合成し、15 秒スロットに詰める。
let msg77 = pack77("CQ", "JA1ABC", "PM95").unwrap();
let tones = message_to_tones(&msg77);
let frame = tones_to_i16(&tones, /* freq */ 1500.0, /* amp */ 20_000);

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

Q65・WSPR・JT65・JT9・uvpacket は独自のエントリポイントを持つ —
[§2.5](#25-独自エントリポイントを持つプロトコル)。

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
| `.freq_hint(hz)` | `f32` | 未設定 | 全部 | この周波数付近の候補を優先 |
| `.osd(bool)` | `bool` | `true` | 全部 | BP の階段が失敗したときの OSD フォールバック。ホストデコードでは `LlrEffort` は常に `Full` |
| `.strictness(s)` | `DecodeStrictness` | `Normal` | 全部 | 採否閾値のプロファイル。どのノブがどのプロトコルに実際に届くかは [§6](#6-engine-プリミティブ) |
| `.eq_mode(m)` | `EqMode` | `Off` | 全部 | `Off` / `Local`。**入力音声**の性質であって探索の性質ではない |
| `.known(&[..])` | 復号済みの行 | 空 | 全部 | 前パスで見つかったメッセージをスキップまたは減算する |
| `.fft_cache(c)` | 前回の `DecodeOutcome` のキャッシュ | 無し | 全部 | 同じ音声への前方 FFT を再利用 |
| `.ap_hint(&ApHint)` | `&ApHint` | 無し | `SupportsWideBandAp` — **FT8・FT4・FST4 全サブモード** | 事前仮説からメッセージビットを固定 |
| `.sic_rounds(n)` | `usize`、`1..=3` にクランプ | 無し | `SupportsSicRounds` — **FT8, FT4** | 平坦な逐次干渉除去 |
| `.sic_early()` | — | 無し | `SupportsSicEarly` — **FT8** | チェックポイント模倣の早期デコード（3 チェックポイント固定構造） |
| `.also_accept(f)` | `Fn(&Wsjt77Fields) -> bool` | 無し | `SupportsMessageFilter` — **FT8・FT4・FST4 全サブモード** | codec が通すもの **＋** `f` が通すもの — [§2.6](#26-メッセージの受理) |
| `.message_filter(f)` | `Fn(&Wsjt77Fields) -> bool` | 無し | `SupportsMessageFilter` — **FT8・FT4・FST4 全サブモード** | codec の判定を `f` で置き換える — [§2.6](#26-メッセージの受理) |
| `.codec_filter()` | — | FT8 は on、他は off | `SupportsMessageFilter` — **FT8・FT4・FST4 全サブモード** | 既定で判定しないプロトコルで codec 自身の判定を適用する — [§2.6](#26-メッセージの受理) |
| `.on_result(cb)` | `FnMut(&Row)` | 無し | 全部 | 見つかった順に行を配信 — [§2.4](#24-ストリーミング配信) |
| `.budget(check)` | `FnMut() -> bool` | 無し | 全部 | 呼び出し側の締切述語 — [§2.3](#23-計算予算) |
| `.sniper(...)` | `(audio, target_hz, max_cand)` | — | `SupportsSniper` — **FT8** | 代わりに `SniperRequest` を作る |
| `.decode()` | — | — | 全部 | 実行 |

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
use mfsk_core::ft8::wave_gen::{message_to_tones, tones_to_i16};
use mfsk_core::msg::decode_request::SniperRequest;
use mfsk_core::msg::wsjt77::{pack77, unpack77};

let msg77 = pack77("CQ", "JA1ABC", "PM95").unwrap();
let tones = message_to_tones(&msg77);
let frame = tones_to_i16(&tones, /* freq */ 1000.0, /* amp */ 20_000);
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
`wspr::decode::{decode_scan_streaming, decode_scan_subtract_streaming}`、
`jt9::decode_scan_streaming`、`jt65::decode_scan_streaming`、および
`q65::{DecodeRequest, SniperRequest, MultiPeriodRequest}` の
`.on_result(cb)`。

### 2.5 独自エントリポイントを持つプロトコル


WSPR は 12 kHz で直接シンボル長 (8192 サンプル) の FFT を取る方式で、
FT 系の「ダウンサンプリングしてからシンボル同期」という流れと
ステージ構成が異なる。そのため `wspr` モジュールが独自のエントリ
ポイントを用意している。ただし内部で使っている FEC (`ConvFano`) と
メッセージコーデック (`Wspr50Message`) は `Wspr: Protocol` の
関連型として宣言済みで、抽象の枠組みからは外れていない。

```rust
# #[cfg(feature = "wspr")] {
use mfsk_core::wspr::decode::decode_scan_default;
use mfsk_core::wspr::tx::synthesize_type1;
use mfsk_core::msg::WsprMessage;

// WSPR Type 1 フレームを合成 (120 秒 @ 12 kHz スロット)。
let samples_f32 = synthesize_type1("K1ABC", "FN42", 37, 12_000, 1500.0, 0.3)
    .expect("valid message");

let decodes = decode_scan_default(&samples_f32, /*sample_rate*/ 12_000);
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
            // ハッシュは過去の Type-1 受信から解決できる場合がある
            println!("{:7.2} Hz  {:+.0} dB  <#{:05x}> {} {}dBm",
                     d.freq_hz, d.snr_db, callsign_hash, grid6, power_dbm);
        }
    }
}
# }
```

`snr_db` は粗同期の段階で計算済みの wsprd 準拠 SNR (dB, 2500 Hz
基準) — wsprd 自身がスポットに添えて報告する値と同じもの。

`decode_scan_default` が粗同期 (周波数×時刻探索) を込みでスロット全体を
スキャンする。周波数・開始サンプルが既知の場合は
`wspr::decode::decode_at(samples, rate, start_sample, freq_hz)` を
直接呼べば粗同期を省略できる。



JT9 と JT65 は同じ scan + 単点デコードのパターンを提供する:

```rust
# #[cfg(feature = "jt65")] {
use mfsk_core::jt65::decode_scan_default;
use mfsk_core::jt65::tx::synthesize_standard;

let audio_f32 = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, 1270.0, 0.3)
    .expect("pack + synth");
let decodes = decode_scan_default(&audio_f32, 12_000);
assert!(!decodes.is_empty(), "ラウンドトリップは復号できるはず");
for d in decodes {
    println!("{:7.2} Hz  {:+.0} dB  {}", d.freq_hz, d.snr_db, d.message);
}
# }
```

JT65 はさらに `decode_at_with_erasures` を提供しており、
低 SNR 環境で RS 消失復号が通常デコーダでは落とすフレームを
回復できる。さらに深い SNR 向けに `decode_at_with_chase` /
`decode_scan_chase*`（`jt65::chase`、issue #169）も用意している —
WSJT-X の stochastic Chase デコーダ `ftrsdap` の忠実な移植（アルゴリズム
形状だけでなく、消失確率テーブル・`getpp` スペクトル電力による候補
ランキング・受理ゲート定数などマジックナンバーも含む）。呼び出し形は
通常の `decode_scan` 系と同じで `&ChaseParams` 引数が増えるだけ。
同日、もう一つ独立した修正も入った：`search`/`rx` に周波数のサブビン
精緻化 + NCO 補正を追加し、FFT の「scalloping loss」を解消——これは
`decode_at_with_chase` だけでなく JT65 の全デコード経路に効く
（`decode_at_with_erasures` 自体もコード変更ゼロのまま同程度に改善）。
chase アルゴリズムの詳細は `chase` モジュールの doc コメント、実測結果
の全体像（この2つの修正を合わせ、従来の ~7-8 dB ギャップをこのcrate の
AWGN コーパス上でほぼ解消——WSJT-X比較の方法論に関する留保も含めて）は
`docs/notes/BENCHMARKS.md` の JT65 節を参照。

`Jt65Result::snr_db` と JT9 の `Jt9Result::snr_db` はどちらも、
各シンボルで復号されたトーンの電力と他トーンの電力比から算出する
decode 側の推定値。JT65 側は Q65 と同じ方法で WSJT-X の 2500 Hz
基準帯域に変換しているが、JT9 側は変換していない —
`jt9::softsym` の downsam9 → peakdt9 → symspec2 パイプラインは
AGC スケーリング・非正規化 IFFT・コヒーレント和を経ており、
JT65/Q65 で成立する帯域幅オフセットがそのまま適用できないため。
`Jt9Result::snr_db` は相対値としてのみ扱うこと (JT9 同士の比較には
使えるが、他プロトコルの `snr_db` とは比較不可)。


**Q65** は `mfsk_core::q65::decode_request` に3つの汎用ビルダを持ち、
`msg::decode_request` と同じ形で、10 サブモード ZST すべてに実装された
sealed な `Q65SubMode` マーカを介して汎用化されている:
`DecodeRequest<P>`（広帯域スキャン）、`SniperRequest<P>`（既知の
`(start_sample, base_freq_hz)`）、`MultiPeriodRequest<P>`（複数スロット
平均）。`.ap_hint()`・`.ap_list()`・`.fading()` は capability gate された
マーカトレイトではなく素の inherent メソッドである — Q65 は全サブモードが
全機能を一様に持つため。下層の `q65::rx` 関数群は `pub(crate)`。

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
自身のモジュールにあるコード) かを示す。ここでの「汎用/専用」は文字どおり
の意味で、汎用セルはそのプロトコルにとって著述コストゼロ、専用セルは
そのプロトコルが持ち込まねばならなかった作業である。

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

この表が可視化するパターン:

- **FT8 / FT4 / FST4** は「安い」追加 — LDPC + 77 bit メッセージ +
  ブロック Costas 同期で、ほぼ全部が汎用。共通コードが多いのは構造が
  共通だからであって、抽象化を試す材料にはなりにくい。
- **WSPR** は *FEC 系統*・*メッセージ長*・*sync mode* の 3 つを独立に
  差し替える — これらの軸が本当に直交している証拠。
- **Q65** は第 3 の FEC 系統 (GF(64) 上の非二進 QRA)、1 マクロから
  10 sub-mode、そして 5 つの並列デコード戦略（[§3.4](#34-デコード戦略)）を、いずれも同じ
  `Protocol` super-trait の内側で加える。
- **uvpacket** は非 WSJT の応用例で、FEC マザーコードだけを再利用し
  汎用 TX/RX パイプラインは迂回する（[UVPACKET.md](UVPACKET.ja.md)）。
- **MSK144** は唯一、trait 面そのものから外れるプロトコルだが、それでも
  FEC 層とメッセージ層は再利用する。

[§3.4](#34-デコード戦略) が Q65 のデコード戦略を、[§8](#8-ランタイムレジストリと-trait-面の検証) が `PROTOCOLS` レジストリと汎用
`tests/protocol_invariants.rs` 検査機構 (実装される 24 ZST — WSJT
ファミリ 20 + uvpacket 4 — すべての列挙・検証) を扱う。

[^ft8]: FT8 は FT4/FST4 と同じく汎用 `DecodeRequest` ビルダーを使うが、
    内部では `engine::pipeline` ではなく手調整された専用エンジン
    `ft8::decode_block` (ホスト・組込み共用) を通る。
    [§6](#6-engine-プリミティブ)「FT8 ブロックデコーダのエントリ」を参照。

[^wspr]: `SyncMode::Interleaved` — チャネルシンボルすべての LSB に
    固定 162 bit sync vector の 1 bit を載せる形式で、ブロック Costas
    ではない。このバリアントを使うのは WSPR のみ。

[^q65]: `Q65Fec::decode_soft` は**設計上 `None` を返す** — 実デコードは
    bit-LLR ではなく GF(64) の確率ベクトル上で QRA コーデック
    (`fec::qra` + `fec::qra15_65_64`) が行う。`NTONES = 65` かつ
    `BITS_PER_SYMBOL = 6` (tone 0 は同期専用) が `GRAY_MAP` 長の契約を
    `[2^BITS_PER_SYMBOL, NTONES]` に緩めた事例。
    [§4](#4-モジュールとクレートの地図)「`FecCodec` はシンボル粒度から
    独立」と [§3.4](#34-デコード戦略) を参照。

[^uv]: uvpacket は汎用パイプラインを迂回するため、`ModulationParams`
    定数のいくつかは装飾的 — trait と不変条件テストを満たすためだけに
    存在する。[UVPACKET.md](UVPACKET.ja.md) を参照。

[^msk]: MSK144 (issue #25) は連続位相の二値 MSK を offset-QPSK として
    送信し、864 サンプルのフレームを固定スロット内の既知オフセットに
    置くのではなく T/R 期間全体で繰り返す — したがって
    `ModulationParams`/`FrameLayout` も `engine::pipeline` も合わず、
    `Protocol` を実装する ZST も存在しない。独自の
    `msk144::decode::decode_slot` ドライバが `msk144::spd`/`msk144::sync`
    でピングを走査する。それでも 77 bit `msg::wsjt77` コーデックと汎用
    LDPC BP/OSD エンジン (`fec::ldpc_128_90`、FST4 の `Ldpc240_101` と
    同じ手順で追加) は再利用する。WSJT-X `samples/MSK144/*.wav` に対する
    ゴールデン WAV recall は 3/3 (`tests/msk144_wsjtx_samples.rs`)。


> この表は `mfsk-core/tests/common_selftest.rs` のコード共有ラチェット、
> `README.md` の共有率パラグラフ、`lib.rs` 自身のドキュメントが揃って
> 辿り着く先の正本である。ここを変えるならそれらも変わる。


### 3.2 諸元

配線済み ZST は 24 個 — WSJT 系のプロトコルとサブモードが 20、
`uvpacket` のサブモードが 4。MSK144 は参考として最終行に挙げてあるが
この 24 には**含まれない** — `Protocol` を実装しないため、レジストリ
にも `tests/protocol_invariants.rs` にも現れない。

| プロトコル       | スロット   | トーン | シンボル | トーン Δf  | FEC                   | Msg   | Sync          | 状態 |
|------------------|------------|--------|----------|------------|-----------------------|-------|---------------|------|
| FT8              | 15 s       | 8      | 79       | 6.25 Hz    | LDPC(174, 91)         | 77 b  | 3×Costas-7    | 実装済 |
| FT4              | 7.5 s      | 4      | 103      | 20.833 Hz  | LDPC(174, 91)         | 77 b  | 4×Costas-4    | 実装済 |
| FST4-15          | 15 s       | 4      | 160      | 16.667 Hz  | LDPC(240, 101)        | 77 b  | 5×Costas-8    | 実装済 (最速 FST4、閾値約-20.7dB) |
| FST4-30          | 30 s       | 4      | 160      | 7.143 Hz   | LDPC(240, 101)        | 77 b  | 5×Costas-8    | 実装済 (閾値約-24.2dB) |
| FST4-60A         | 60 s       | 4      | 160      | 3.0864 Hz  | LDPC(240, 101)        | 77 b  | 5×Costas-8    | 実装済 (地上波主力サブモード、閾値約-28.1dB) |
| FST4-120         | 120 s      | 4      | 160      | 1.4634 Hz  | LDPC(240, 101)        | 77 b  | 5×Costas-8    | 実装済 (閾値約-31.3dB) |
| FST4-300         | 300 s      | 4      | 160      | 0.5580 Hz  | LDPC(240, 101)        | 77 b  | 5×Costas-8    | 実装済 (閾値約-35.3dB、実装済み中最深) |
| WSPR             | 120 s      | 4      | 162      | 1.465 Hz   | conv r=½ K=32 + Fano  | 50 b  | シンボル毎 LSB (npr3) | 実装済 |
| JT9              | 60 s       | 9      | 85       | 1.736 Hz   | conv r=½ K=32 + Fano  | 72 b  | 16 分散位置   | 実装済 |
| JT65             | 60 s       | 65     | 126      | 2.69 Hz    | RS(63, 12) GF(2⁶)     | 72 b  | 63 分散位置   | 実装済 |
| Q65-15A          | 15 s       | 65     | 85       | 6.667 Hz   | QRA(15, 65) GF(2⁶) + CRC-12 | 77 b | 22 分散位置 | 実装済 |
| Q65-30A          | 30 s       | 65     | 85       | 3.333 Hz   | (同 QRA codec) | 77 b | (同) | 実装済 |
| Q65-60A          | 60 s       | 65     | 85       | 1.667 Hz   | (同 QRA codec)        | 77 b  | (同)          | 実装済 (6 m EME) |
| Q65-60B          | 60 s       | 65     | 85       | 3.333 Hz   | (同 QRA codec)        | 77 b  | (同)          | 実装済 (70 cm / 23 cm EME) |
| Q65-60C          | 60 s       | 65     | 85       | 6.667 Hz   | (同 QRA codec)        | 77 b  | (同)          | 実装済 (~3 GHz EME) |
| Q65-60D          | 60 s       | 65     | 85       | 13.33 Hz   | (同 QRA codec)        | 77 b  | (同)          | 実装済 (5.7 / 10 GHz EME) |
| Q65-60E          | 60 s       | 65     | 85       | 26.67 Hz   | (同 QRA codec)        | 77 b  | (同)          | 実装済 (24 GHz+、強拡散) |
| Q65-120D         | 120 s      | 65     | 85       | 6.0 Hz     | (同 QRA codec)        | 77 b  | (同)          | 実装済 (10GHz レインスキャッター/対流圏散乱) |
| Q65-120E         | 120 s      | 65     | 85       | 12.0 Hz    | (同 QRA codec)        | 77 b  | (同)          | 実装済 (6m イオノスキャッター) |
| Q65-300A         | 300 s      | 65     | 85       | 0.289 Hz   | (同 QRA codec)        | 77 b  | (同)          | 実装済 (光散乱、最深AWGN) |

### 3.3 プロトコル別の注記

汎用 vs 専用 の分類は [§3.1](#31-プロトコル毎の汎用-vs-専用) の表に
まとめてある。以下の注記は、その表に載せきれないプロトコル固有の事実
だけを補う。

- **FST4** — LDPC(240, 101) + 24 bit CRC (`fec::ldpc240_101`)。BP/OSD
  のコードは LDPC サイズが変わっても同じなので、新規なのはパリティ
  検査行列・生成行列と符号寸法だけ。実装済みの 5 sub-mode
  (FST4-15/30/60A/120/300) は `NSPS` / `SYMBOL_DT` / `TONE_SPACING_HZ`
  のみが異なり (FST4-15 だけ `TX_START_OFFSET_S` も 0.5 s)、
  `q65_submode!` と同じパターンの `fst4_submode!` マクロが生成する。
  FST4-900 / FST4-1800 は未実装 (需要なし)。FST4W (WSPR 型片方向
  50 bit ビーコン、LDPC(240, 74)、周期 120/300/900/1800 s) は別の
  メッセージ形式で対象外 — issue #23 参照。
- **WSPR** — `ConvFano` は WSJT-X `lib/wsprd/fano.c` の移植、
  `Wspr50Message` は Type 1 / 2 / 3 を実装。`wspr` モジュールは
  120 s スロットの coarse search を妥当な時間で回すため四半シンボル
  粒度のスペクトログラムを追加する。
- **JT9 / JT65** — JT9 の `ConvFano232` は WSPR の `ConvFano` と
  206 bit 符号語フレーミングだけが異なり、いずれも 72 bit `Jt72Codec`
  に接続する。JT65 の `Rs63_12` (`fec::Rs63_12` として re-export) は
  Karn の Berlekamp-Massey による消失対応復号を提供する。
- **Q65** — GF(64) 上の QRA (`fec::qra::QraCode` + 具象コード
  `fec::qra15_65_64::QRA15_65_64_IRR_E23`)。アプリケーション層は 13
  情報シンボルに CRC-12 を付与し、65 シンボルの符号語から CRC 2
  シンボルを puncture して 63 チャネルシンボルを実送信する。10
  sub-mode は `NSPS` とトーン間隔 (×1…×16) のみが異なり、5 戦略（[§3.4](#34-デコード戦略)）
  はすべて同じ QRA codec を共有する。

### 3.4 デコード戦略

どのプロトコルも同じ基本フローを走るが、その周りを包む*戦略*が異なる。
大半は単一パスである。1つの FEC フレームに対して複数の並列受信系を
持つのは Q65 だけで、MSK144 はスロットモデル自体をバースト走査に
置き換えている。

| プロトコル | 既定の戦略 | 任意の戦略 |
|----------|-----------|-----------|
| **FT8** | 単一パス BP + OSD | AP iaptype ループ (1–12)、SIC 1–3 ラウンド、`.sic_early()`、sniper |
| **FT4** | 単一パス BP + OSD | SIC 1–3 ラウンド、フルスロット・コヒーレント sync (`sync2d`) |
| **FST4** | 単一パス BP + OSD | フルスロット2段コヒーレント sync 探索 |
| **WSPR** | 単一の専用パス（四半シンボル・スペクトログラム走査） | — |
| **JT9** | 単一の専用パス | — |
| **JT65** | 単一の専用パス | RS 消失復号、確率的 Chase デコーダ |
| **Q65** | `(Δf,Δt,b90)` グリッド + Lorentzian フェージング BP（スキャン） | AP ヒント、明示的な高速フェージング、AP リスト、マルチ周期 |
| **MSK144** | T/R 周期全体のバースト走査 | — |

**事前情報デコード (AP) は sniper の機能ではなく一般の選択肢である。**
AP は `process_candidate_basic` 自身の ladder の一段であり、FT8・FT4・
FST4 全サブモードに届く。`msg::pipeline_ap` は仮説生成だけで自前の
エンジンを持たない。かつて偶然 sniper と結合しており、それがデコードの
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
| 複数 T/R 周期にまたがる微弱・電離層散乱信号 | マルチ周期 EMA 平均（3 段カスケード） | `MultiPeriodRequest::<P>::new(...).decode()` | 単一周期のどの戦略でも取れない信号を拾う |

`.ap_list()` と `.fading()` は下層エンジンでは排他であり、`.decode()` は
`ap_list > fading (+ ap_hint) > ap_hint > plain` の順で解決する。
`MultiPeriodRequest` は T/R スロットごとに1本の `&[&[f32]]` を取り、
Rust 専用である（C ABI には無い）。各フロントエンドが実際に何をして
いるか、既定のスキャンがなぜ素の Bessel パスではないのかは
[`DESIGN_RATIONALE.md` §4](../notes/DESIGN_RATIONALE.md#4-q65s-decoder-strategies-and-what-each-is-for)。

---
## 4. モジュールとクレートの地図

```text
mfsk_core
├── engine/           Protocol trait 群、DSP、sync、LLR、equaliser、pipeline
│   ├── protocol.rs     ModulationParams / FrameLayout / Protocol / FecCodec / MessageCodec
│   ├── dsp/            resample · downsample · gfsk · subtract · msk · analytic ·
│   │                   ddc · fir · dotprod · 固定小数点 FFT カーネル
│   ├── fft.rs          FftPlanner トレイトと extern factory (EMBEDDED.md 参照)
│   ├── scalar.rs       Q-format 固定小数点スカラ型
│   ├── sync.rs         coarse_sync / refine_candidate
│   ├── sync2d.rs       FT4 / FST4 フルスロット・コヒーレント sync 探索
│   ├── ft4_coarse.rs   FT4 の粗候補生成
│   ├── baseline.rs     スペクトルのベースラインフィット (FT4 / FST4 正規化)
│   ├── tx.rs           送信側の共有ヘルパ
│   ├── llr.rs          symbol_spectra / compute_llr / sync_quality
│   ├── equalize.rs     equalize_local (トーン毎 Wiener)
│   ├── spectrogram.rs  Spectrogram 構築/スコアリングカーネル — JT9, JT65, Q65
│   │                   (WSPR は独自実装を維持 — fixed-point FFT backend +
│   │                   baseline-fit 正規化という実質的な差異があるため)
│   ├── interleave.rs   bit-reversal interleave_bitrev/deinterleave_bitrev
│   │                   — WSPR, JT9 (JT65 は別アルゴリズム — 7×9 行列転置
│   │                   — のため jt65/ に独自実装を維持)
│   └── pipeline.rs     decode_frame / decode_frame_subtract / process_candidate_basic
│                       (pub(crate) 内部実装 — 呼び出しは
│                       msg::decode_request::DecodeRequest/SniperRequest 経由)
├── fec/              FecCodec 実装群
│   ├── ldpc/           LDPC(174, 91)  — FT8, FT4 (bp.rs / osd.rs / params.rs / tables.rs)
│   ├── ldpc240_101/    LDPC(240, 101) — FST4
│   ├── ldpc_128_90/    LDPC(128, 90)  — MSK144
│   ├── conv/           ConvFano r=½ K=32 — WSPR、ConvFano232 — JT9 (fano.rs)
│   ├── rs/             RS(63, 12) GF(2⁶) — JT65
│   └── qra/            Q-ary RA codec ファミリ — Q65
│       ├── code.rs       汎用 QRA エンコーダ + 非二進 BP デコーダ
│       ├── q65.rs        Q65 アプリケーション層 (CRC-12 + puncturing) +
│       │                 リストデコード関数 (check_codeword_llh,
│       │                 decode_with_codeword_list)
│       ├── fast_fading.rs ドップラー拡散対応 intrinsic metric
│       ├── fading_tables.rs Gaussian / Lorentzian キャリブレーション表
│       ├── npfwht.rs      非二進 Walsh-Hadamard 変換ヘルパ
│       └── pdmath.rs      確率領域 BP 数値計算ヘルパ
├── msg/              メッセージコーデック
│   ├── decode_request.rs DecodeRequest / SniperRequest — FT8/FT4/FST4 の
│   │                     公開デコードエントリポイント (§2。0.8.0 以前の
│   │                     decode_frame*/decode_sniper* 系を置換)
│   ├── wsjt77.rs       77 bit WSJT メッセージ (pack / unpack) — FT8, FT4, FST4, Q65, MSK144
│   ├── wspr.rs         50 bit WSPR Types 1 / 2 / 3
│   ├── jt72.rs         72 bit JT メッセージ — JT9, JT65
│   ├── callsign28.rs   共有 base-37/36/10/27³ コールサイン pack/unpack
│   │                   コア — jt72 (JT9/JT65) と wspr の両方がラップ。
│   │                   上流の単一ルーチン (`packjt.f90` の
│   │                   packcall/unpackcall) に由来
│   ├── q65.rs          77 bit <-> 13×GF(64) symbol パッキング (QRA codec 用)
│   ├── ap.rs           ApHint — a-priori ヒントビルダー (with_call1/call2/grid/report)
│   ├── pipeline_ap.rs  AP 対応マルチパス decode pipeline (77-bit 系プロトコル)
│   ├── packet_bytes.rs PacketBytesMessage — バイトペイロード例示コーデック
│   └── hash_table.rs   コールサインハッシュテーブル
├── registry.rs       PROTOCOLS 静的配列 + ProtocolMeta + by_id / by_name
├── ft8/              FT8 ZST + decode + wave_gen
├── ft4/              FT4 ZST + decode
├── fst4/             FST4 ファミリ — 5 sub-mode ZST (15/30/60A/120/300) + decode
├── wspr/             WSPR ZST + decode + synth + spectrogram search
├── jt9/              JT9 ZST + decode
├── jt65/             JT65 ZST + decode (+ 消失対応 RS)
├── q65/              Q65 ファミリ — 10 sub-mode ZST + decode + synth
│   ├── protocol.rs     q65_submode! マクロ (Q65a15..Q65a300 ZST 生成)
│   ├── rx.rs           5 つのデコード戦略 (AWGN / AP-hint / fast-fading / AP-list / multi-period)、§3.4 参照
│   ├── ap_list.rs      standard_qso_codewords — full AP-list 候補生成
│   ├── tx.rs           65-FSK 合成器 (sub-mode 対応)
│   ├── search.rs       22 シンボル Costas-block coarse 検索
│   └── sync_pattern.rs Q65 分散同期配置
├── msk144/           MSK144 — Protocol 実装なし、独立トップレベルドライバ (§3.1)
│   ├── tx.rs           codeword -> 864 サンプル複素 OQPSK フレーム
│   ├── sync.rs         (CFO, タイミング) 同時整合フィルタ探索
│   ├── spd.rs          バースト候補検出 + short-ping デコードループ
│   ├── frame_decode.rs sync ゲート -> LLR -> LDPC -> メッセージ
│   └── decode.rs       decode_slot(): スライディングウィンドウ型トップレベルドライバ
└── uvpacket/         非 WSJT 応用例 — 4 sub-mode ZST、独自 tx/rx (UVPACKET.md)
    ├── protocol.rs     ModulationParams/FrameLayout 実装 (一部は装飾的、UVPACKET.md 参照)
    ├── framing.rs      可変長バーストフレーミング
    ├── sync_pattern.rs 4 バリアント 127-chip BPSK m-sequence プリアンブル
    ├── interleaver.rs  bit インターリーバ
    ├── puncture.rs     ヘッダブロック用 LDPC240_101 puncturing
    ├── message.rs      byte-pipe (app_type) メッセージ層
    ├── tx.rs           π/4-DQPSK + RRC 合成器
    └── rx.rs           LMS イコライザ + differential demod + decode
```

各プロトコルモジュールはフィーチャーフラグ (`ft8`、`ft4`、`fst4`、
`wspr`、`jt9`、`jt65`、`q65`、`msk144`、`packet-bytes`、`uvpacket`)
で gate されている。`engine`、`fec`、`msg`、`registry` は常時利用可能。

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
上記の 4 系統の FEC のうち 2 系統 — JT65 の Reed-Solomon over
GF(2⁶) と Q65 の QRA over GF(2⁶) — は非二進符号で、bit 単位の
trait API を満たすために `encode` の中で bit ↔ シンボル変換を
内製している。それぞれの本来のシンボル単位デコードは
`decode_soft` の外側に置かれていて、`Q65Fec::decode_soft` は仕様
として `None` を返し、実際の Q65 デコードは GF(64) 確率ベクトル上の
非二進 BP として `fec::qra::Q65Codec` で実行され、エントリポイントは
`q65::rx::decode_at_for` になっている。`K` / `N` を bit で数えて
おくことで、二進・非二進どちらの符号にも
`FecCodec::N ≤ N_DATA × BITS_PER_SYMBOL` という横断的不変条件
（[§8.2](#82-汎用-trait-面検査)）が同じ式で成り立つ。

---

## 5. `Protocol` トレイト階層


このクレートはボトムアップに読むとよい。どのプロトコルにも依存しない
**汎用コア**があり、各プロトコルはそのコアのどの部品を使うかを選ぶだけの
薄いプラグインである。

1. **`engine/`** — プロトコル非依存の DSP・同期・LLR・イコライザ・復号
   パイプライン。ここの関数はすべて `P: Protocol` に対して汎用で、
   プロトコルの定数を読むだけ。プロトコル毎の分岐は一切持たない。
2. **`fec/`** — 前方誤り訂正コーデック群。それぞれ `FecCodec` の実装で、
   LDPC (3 サイズ) + BP/OSD、畳み込み + Fano、Reed-Solomon、Q-ary の
   QRA コーデックがある。
3. **`msg/`** — メッセージコーデック群 (それぞれ `MessageCodec` の実装)
   と、パイプライン全体を駆動する汎用 `DecodeRequest`/`SniperRequest`
   ビルダー（[§2](#2-デコード-api)）。
4. **プロトコル**は 3 つの合成可能な trait を実装する ZST (zero-sized
   type; `ModulationParams` + `FrameLayout` → `Protocol`)。持つのは
   定数と 2 つの関連型の選択 (`type Fec` と `type Msg`)、そして
   `SYNC_MODE` だけ。「プロトコルを追加する」とは、FEC を選び、
   メッセージコーデックを選び、sync mode を選び、数値を宣言する—これで
   全部である。


デコード時、これらの層は 1 つの受信フローとして実行される。すべての
実装済みプロトコルが共有し、`engine` 内の `P: Protocol` に対して汎用な
自由関数の連なりである（関数レベルの注記は [§6](#6-engine-プリミティブ)）:

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

`P: Protocol` は**コンパイル時**の型パラメータなので、monomorphize が
プロトコルごとに完全特殊化されたコピーを生成する—抽象化にランタイム
コストはない（下記「Monomorphization がこれを無料にしている」）。直接の帰結: 同一の
アルゴリズムが Native Rust / WASM / Android / C・C++ のいずれでも動く。
共通経路 (たとえば LDPC BP) の改善はそれを使う全プロトコルに波及する。
プロトコル追加の変更範囲はそのプラグインに閉じる。C ABI の分岐は
`protocol_id` 一段のみで、その先は既に特殊化済み。




トレイト定義:

対応するすべてのモードは、3 つの合成可能な trait を実装する
**Zero-Sized Type (ZST)** で記述される:

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
    const LLR_SCALE: f32 = 2.83;
}

pub trait FrameLayout: Copy + Default + 'static {
    const N_DATA: u32;
    const N_SYNC: u32;
    const N_SYMBOLS: u32;
    const N_RAMP: u32;
    const SYNC_MODE: SyncMode;  // Block(&[SyncBlock]) または Interleaved { .. }
    const T_SLOT_S: f32;
    const TX_START_OFFSET_S: f32;
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
    const ID: ProtocolId;
}
```


### トレイト合成の実例

上記 3 つのトレイトがどう組み合わさるかを、既存プロトコルの
ZST 定義で示す。

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
    const ID: ProtocolId = ProtocolId::Wspr;
}

// 説明用のダミー値 — 実際の 162 bit npr3 ベクトルは
// `wspr::decode` 内部の非公開 sync テーブルにある。
const WSPR_SYNC_VECTOR: [u8; 162] = [0u8; 162];
```

呼び出し側のパイプラインは `DecodeRequest::<Ft4>::new(...).decode()`
（[§1](#1-クイックスタート)）または WSPR 専用の
`wspr::decode::decode_scan_default(...)` のように型引数でプロトコルを
指定するだけで済み、合成の結果として選ばれた FEC・メッセージ
コーデック・同期方式が自動的に使われる。


### Monomorphization がこれを無料にしている

ホットパス (`engine::sync::coarse_sync::<P>`、
`engine::llr::compute_llr::<P>`、
`engine::pipeline::process_candidate_basic::<P>`、…) はすべて
`P: Protocol` を**コンパイル時型パラメータ**として受け取る。rustc が
具象プロトコルごとに 1 コピーずつ monomorphize し、LLVM は完全特殊化
された関数として trait 定数を即値にインライン化する。抽象化のコストは
ゼロ — 生成される FT8 コードは本ライブラリが fork する前の FT8 専用
ハンドコードとバイト単位で同一で、FT4 は共通関数に加えた
マイクロ最適化すべての恩恵を自動的に受ける。

`dyn Trait` はコールドパス専用: FFI 境界、JS 側のプロトコル切替、
デコード後 1 回のみ実行される `MessageCodec::unpack` など。

### プロトコルを追加する

既存資産をどこまで再利用できるかによって、追加作業は大きく 3 段階に
分かれる。

1. **FEC とメッセージが既存のものと同じ場合** (例: FT2、あるいは
   FST4 の他サブモード) — 新しい ZST を定義し、数値定数 (`NTONES`、
   `NSPS`、`TONE_SPACING_HZ`、`SYNC_MODE` など) と同期パターンを
   入れ替えるだけで済む。`Fec` と `Msg` は既存実装の型エイリアスで
   構わず、`DecodeRequest::<P>` パイプライン全体がそのまま動く。

2. **FEC が新しく、メッセージは既存と同じ場合** (例: 異なるサイズの
   LDPC) — `fec/` にコーデックのモジュールを追加し、`FecCodec`
   トレイトを実装する。BP / OSD / systematic エンコードの
   アルゴリズムは LDPC のサイズが変わっても構造的に同じなので、
   変更箇所はパリティ検査行列・生成行列と符号寸法 (N, K) にとどまる。
   実例として `fec::ldpc240_101` が参考になる。

3. **FEC とメッセージのどちらも新しい場合** (例: WSPR) — FEC 実装と
   メッセージコーデックを追加し、さらに同期構造が従来と大きく異なる
   ときは `SyncMode` に新しいバリアントを足す。WSPR はこの経路で
   追加しており、`ConvFano` + `Wspr50Message` + `SyncMode::Interleaved`
   の 3 点を新設しつつ、coarse search / spectrogram / 候補重複除去 /
   CRC 検査 / メッセージ unpack といったパイプライン側の仕組みは
   従来のまま利用している。

4. **既存プロトコルの sub-mode 追加** (例: Q65-60A〜E が Q65-30A と
   NSPS とトーン間隔以外を共有) — `q65_submode!` マクロが差分定数を
   受け取り、ZST と 3 つの trait 実装を 1 行で生成する。新規テストや
   パイプライン変更は不要 — `tests/protocol_invariants.rs` に
   1 行追加するだけで構造健全性チェックが自動的に走る。


---


## 6. engine プリミティブ

### DSP (`mfsk_core::engine::dsp`)

| モジュール      | 役割                                                        |
|-----------------|-------------------------------------------------------------|
| `resample`      | 12 kHz への線形リサンプラ                                   |
| `downsample`    | FFT ベース複素デシメーション (`DownsampleCfg`)              |
| `gfsk`          | GFSK トーン→PCM 波形合成 (`GfskCfg`)                        |
| `subtract`      | 位相連続最小二乗 SIC (`SubtractCfg`)                        |

いずれもランタイム `*Cfg` 構造体を引数に取る (`<P>` ではない) のは、
FFT サイズなどチューニングが trait 定数だけから単純派生できない
ためで、プロトコルモジュールがモジュールレベル定数を公開している:
`ft8::downsample::FT8_CFG`、`ft4::decode::FT4_DOWNSAMPLE` など。

### Sync (`mfsk_core::engine::sync`)

* `coarse_sync::<P>(audio, freq_min, freq_max, …)` — UTC 整列 2D
  ピーク探索、`P::SYNC_MODE.blocks()` を走査 (FT8 以外向け)
* `refine_candidate::<P>(cd0, cand, search_steps)` — 整数サンプル
  スキャン + 放物線サブサンプル補間
* `make_costas_ref(pattern, ds_spb)` / `score_costas_block(...)` —
  診断・カスタムパイプライン用の生相関ヘルパー

> **FT8 は `decode_block::coarse_sync` のみを経由する。**
> 0.6.0 以降、FT8 ホストパイプラインは
> `mfsk_core::ft8::decode_block::coarse_sync` (`compute_spectrogram`
> とともに公開 API に昇格) を使う。旧 `ft8::sync::coarse_sync` の
> 薄ラッパは削除済。`engine::sync::coarse_sync::<Ft8>` を直接呼び
> 出すパスは残しているが、`DecodeRequest::<Ft8>`/`SniperRequest::<Ft8>`
> ([§2](#2-デコード-api)) は内部で `decode_block::coarse_sync` を経由する。

### Sync2D — FT4 / FST4 フルスロット・コヒーレント sync 探索 (`mfsk_core::engine::sync2d`)

WSJT-X から移植したプロトコル専用のフルスロット・コヒーレント探索が
2 系統ここにある。いずれも symbol 境界で位相をリセットせず 8-symbol
ブロック全体で連続的に位相を積算する Costas 参照信号
(`make_costas_ref_continuous`) を、非コヒーレントな `Σ|z_k|²` パワー
和ではなくコヒーレントな単一内積 (`score_flat_coherent`、振幅 `|z|`)
でスコア化する — sync スコアの SNR 弁別力が ~3 dB 改善する:

* `ft4_sync_search::<P>(cd0, candidate)` / 窓指定版の
  `ft4_sync_search_window::<P>(cd0, candidate, ib_min, ib_max)` —
  **FT4 専用**。coarse-sync 候補自身の (しばしば外れる) Δt 推定
  周辺のローカル窓ではなく、スロット全体のダウンサンプル・サンプル
  範囲にわたるコヒーレントな Δt 探索 (`ft4_decode.f90` の
  `isync=1`/`isync=2` ループ、`sync4d.f90` のスコア関数)。
* `fst4_sync_search::<P>(cd0, cand)` — WSJT-X の
  `fst4_decode.f90:657-925` に対応する FST4 専用の 2 段階フル
  スロット探索。coarse パスはスロット全体 (±1.5 s、step 4、
  周波数 ±12 step × 0.1·baud)、fine パスは ±7 step × 0.02·baud ×
  ±4 サンプル。FST4 の AWGN 感度ギャップを WSJT-X 公称値に対して
  ~0.3 dB まで縮小した (issue #146)。

両者はかつて共有していたローカル (Δf, Δt) refine
(`sync2d_refine` / `Sync2dConfig`) を置き換えたもので、その旧実装は
**削除済み** (2026-07-20、呼び出し箇所ゼロ)。FT4 (issue #72) と
FST4 (issue #146) がそれぞれフルスロット探索を必要とするようになった
ため — coarse-sync 候補の位置を中心とするローカル窓では、その
非コヒーレントな Δt 推定が窓の探索半径を超えて外れているケースを
回復できなかった。

同じ作業で `engine::sync::coarse_sync::<P>` にも FST4 専用の拡張が
入った: 既存の short-time Costas グリッドしきい値に加えて、
WSJT-X の `get_candidates_fst4` を模したフルスロット非コヒーレント
4-tone パワーチェックをクリアした bin も候補リストに追加できる
ようになった (`P::ID == ProtocolId::Fst4` でゲート、FT8/FT4 は
バイト完全一致のまま)。単一信号の AWGN sweep では no-op として
計測された (この sweep では正解候補が元々リストから漏れる状況では
なかった) が、混雑帯でリストサイズが固定のまま多数の co-channel
候補が競合する実運用シナリオでは、WSJT-X 準拠のカバレッジ改善と
して意味がある。

### LLR (`mfsk_core::engine::llr`)

* `symbol_spectra::<P>(cd0, i_start)` — シンボル単位 FFT bin
  (汎用パス。FT8 では中間 `cd0` を割り当てない
  `ft8::decode_block::fill_symbol_spectra` を推奨)
* `compute_llr::<P>(cs)` — WSJT 式 4 バリアント LLR (a/b/c/d)。
  `nsym ∈ {1, 2, P::LLR_NSYM_MAX}` の相関ラダー仮説から構築される。
  `LLR_NSYM_MAX` のデフォルトは 3 (FT8 較正値)、FT4 は 4、FST4 は 8
  に上書き — それぞれ自身の WSJT-X bit-metric コード
  (`get_ft4_bitmetrics.f90` / `get_fst4_bitmetrics.f90`) に合わせた
  値で、FT8 のデフォルトを無自覚に流用しているわけではない (FST4 の
  上書きは 0.7.1 で追加。それまでは上書きが無く FT8 のデフォルトに
  フォールバックしていた。issue #146)
* `sync_quality::<P>(cs)` — 硬判定 sync シンボル一致数

### Equalise (`mfsk_core::engine::equalize`)

* `equalize_local::<P>(cs)` — `P::SYNC_MODE.blocks()` pilot 観測から
  トーン毎 Wiener equalizer を推定、Costas が訪問しないトーンは
  線形外挿でカバー

### Pipeline (`mfsk_core::engine::pipeline`)

`decode_frame::<P>` (coarse sync → 並列 process_candidate →
dedupe)、`decode_frame_subtract::<P>` (3-pass SIC ドライバ)、
`process_candidate_basic::<P>` (候補単体の BP+OSD) は pipeline の
下にある engine 生関数だが、issue #191/#203 以降 **`pub(crate)`**
(または非デフォルトの `internal-testing` feature 下でのみ `pub`。
クレート自身のテストバイナリが使用) である。アプリケーションから
直接呼び出すべきではなく、代わりに
`msg::decode_request::DecodeRequest`/`SniperRequest` (上記参照) を
使う — これらの関数を builder で包んでいる。`decode_frame_subtract`
は 0.6.2 以降 `subtract_signal_lpf` (WSJT-X 式 channel-aware
subtract) を使用。旧 `subtract_signal_weighted` /
`qsb_partial_gain` 系は削除済。

`DecodeStrictness` (`Strict`/`Normal`/`Deep`) は 3 つのメソッドを持つ
(4 つ目の `osd_score_min()` — OSD 実行前の coarse-sync スコアゲート
— は issue #230 で完全に削除済み: FST4・FT4 両方でバイパスされてお
り、どのプロトコルにも生きた呼び出し元が残っていなかったため)。
プロトコルごとに「実際に効くか」が異なる — `.strictness(...)` が何
かを変えるかどうかは呼び出し先次第なので注意:

* `osd_max_errors()` — OSD 後の硬判定エラー上限ゲート (`osd_depth`
  別)。**実質 FT4 専用。** FST4 ではバイパス済み
  (`engine/pipeline.rs` の `is_fst4` — FST4 は WSJT-X 自身の FST4
  受理判定 `fst4_decode.f90:570`: `nharderrors >= 0 &&
  unpk77_success` に合わせて CRC-24 のみを信頼、そのようなゲートは
  無し) だが FT4 では**生きている** — 実際の `ft4sim` AWGN/CCIR
  sweep で再較正済み (issue #72、2026-07-18)。もはや FT8 較正値の
  プレースホルダーではない。**名前に反して、FT8 はこのメソッドを
  これまで一度も呼んでいなかった** — FT8 自身の OSD dispatch は
  hardcoded 定数を使っていた (下記 `ft8_nharderrors_max` 参照)。旧版
  の本ドキュメントが「FT8 較正値」と誤って説明していた箇所を訂正。
* `ap_max_errors(locked_bits)` — AP 付き decode の硬判定エラー上限、
  locked-bit 数で段階化。FT8 の per-candidate AP loop と FT4/FST4 の
  AP sniper (`msg::pipeline_ap`) 双方で生きている — 両呼び出し箇所で
  数値統一済み (issue #191)。
* `ft8_nharderrors_max()` — FT8 自身の flat (`osd_depth` 段階なし)
  硬判定エラー上限、**非 AP** の BP staircase と OSD fallback 向け
  (`ft8::decode_block::process_candidates`/`osd_strategy`)。issue
  #221 で追加: それまで `.strictness(...)` は FT8 の非 AP 経路では
  何もしないダミーだった — hardcoded `36` (WSJT-X 自身の
  `ft8b.f90:422` の上限) が無条件に走っており、issue #188 で
  strictness 段階版を消費していたコードが削除されて以来 dead code
  化していた。`Normal` は今も同じ 36 を返す (デフォルト挙動は無変
  更)。`Strict`/`Deep` は新規の生きた knob — `Strict = 22` は issue
  #72 の調査で実際に使われた値の再利用、`Deep = 40` は探索的な値で
  フェージングコーパスでの sweep はまだ未実施。

AP 対応版は `msg::pipeline_ap` に配置 (AP hint 構築が
77-bit 形式に依存するため)。

### FT8 ブロックデコーダのエントリ (`mfsk_core::ft8::decode_block`)

FT8 モジュールは共有パイプラインの上に並列のエントリ群を持ち、
ホスト・組込で同じ `process_one_candidate_inner` 本体を共有する
(0.6.1 で導入)。入力は同じで、内側のどのステップを有効にするかが
違うだけ:

* `decode_block` / `decode_block_tuned` — pass-1 BP のみ
* `decode_block_with_ap` / `decode_block_with_ap_tuned` — pass-1 BP
  に続き、`q_thresh` を超える sync quality の候補に対して WSJT-X
  AP iaptype ループ (1–12) を回す。0.6.1 新規
* `decode_block_into[_tuned]` — 組込 fixed-point エントリポイント
  (`fixed-point` feature)。`decode_block[_tuned]` と同じ形だが、
  `embedded-shared::dual_core` との API 安定性のため
  別名を維持。0.8.0 以前は呼出側提供の BASIS scratch も受け取って
  いたが、Goertzel fill path 移行で scratch が不要になったため削除
  (issue #162)
* `coarse_sync` / `coarse_sync_with_allsum` — FT8 sync grid 本体
  (0.6.0 で公開 API 昇格)
* `fill_symbol_spectra` / `fill_symbol_spectra_goertzel` — 音声から
  直接シンボル毎 FFT を抽出 (旧コードの cd0 +
  `engine::llr::symbol_spectra` 二段経路を置換)


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
| `jt9` / `jt65` / `q65` | off | **ホスト専用** — `rustfft` を直接呼ぶため `fft-rustfft` を、したがって `std` を引く |
| `msk144` | off | MSK144 — `Protocol` の ZST は無く、独自のトップレベルドライバを持つ |
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
「このビルドは何をサポートするか」を列挙したい消費側 (UI 層、
FFI ブリッジ、自動検出 probe) は、自前のリストをハードコード
する必要が無い:

```rust
use mfsk_core::PROTOCOLS;

for p in PROTOCOLS {
    println!(
        "{:10}  {:>3}-tone  {:>4} bits/sym  {:>5.1} s slot  ID={:?}",
        p.name, p.ntones, p.bits_per_symbol, p.t_slot_s, p.id,
    );
}
```

各 `ProtocolMeta` は protocol の `id` (`ProtocolId` enum、
ファミリレベル)、表示名 `name`、および trait 面が公開する全定数を
保持する — 変調 (`ntones`, `bits_per_symbol`, `nsps`, `symbol_dt`,
`tone_spacing_hz`, `gfsk_bt`, `gfsk_hmod`)、フレーム (`n_data`,
`n_sync`, `n_symbols`, `t_slot_s`)、コーデック (`fec_k`, `fec_n`,
`payload_bits`)。

参照ヘルパー:

* `mfsk_core::by_id(ProtocolId::Q65)` — 同じファミリ id を持つ
  全 entry を返す。Q65 は 10 件 (sub-mode 毎)、その他は 1 件。
* `mfsk_core::by_name("Q65-60D")` — 表示名による厳密一致検索。
* `mfsk_core::for_protocol_id(id)` — 同じ id を持つ最初の entry。
  「ファミリ毎に 1 mode」のケースで便利。

Q65 は registry 上で family / sub-mode 区別が最も顕在化する例:
10 sub-mode 全てが `ProtocolId::Q65` を共有 (FFI tag が family
レベルである故) しつつ、NSPS / トーン間隔 / スロット長が異なる
ため独立した entry になる。同じ形が FST4 にも小規模に現れる —
`by_id(ProtocolId::Fst4)` は 5 件 (T/R 周期 sub-mode 毎) を返す。

レジストリ本体は `mfsk-core/src/registry.rs` 内部の
`protocol_meta!` マクロで構築される。新しいプロトコルの追加は
ZST + 表示名で 1 行ずつ。

### 8.2 汎用 trait 面検査

`tests/protocol_invariants.rs` は `assert_protocol_invariants::<P:
Protocol>(name)` の単一の generic 関数をすべての実装 ZST に対して
実行する。本体は FT8、FT4、5 sub-mode の FST4、WSPR、JT9、JT65、
10 sub-mode の Q65、4 sub-mode の uvpacket — 24 invocation × 1 実装。
3 つのヘルパー関数が合計 17 個の不変条件を pin する:

* **`assert_modulation_invariants<P: ModulationParams>`** —
  `2^BITS_PER_SYMBOL ≤ NTONES`、`SYMBOL_DT × 12000 == NSPS`、
  `TONE_SPACING_HZ`, `NDOWN`, `NSTEP_PER_SYMBOL`,
  `NFFT_PER_SYMBOL_FACTOR`, `GFSK_HMOD > 0`、`GFSK_BT ≥ 0`、
  `GRAY_MAP.len() ∈ [2^BITS_PER_SYMBOL, NTONES]`、map エントリは
  unique かつ tone index 範囲内。
* **`assert_frame_layout_invariants<P>`** —
  `N_SYMBOLS == N_DATA + N_SYNC`、正の `T_SLOT_S`、非負の
  `TX_START_OFFSET_S`。`SyncMode::Block` ではパターン長の総和が
  `N_SYNC` と一致しブロックがフレームに収まる; `SyncMode::Interleaved`
  では sync vector 長が `N_SYMBOLS` と一致し
  `sync_bit_pos < BITS_PER_SYMBOL`。
* **`assert_codec_consistency<P: Protocol>`** —
  `MessageCodec::PAYLOAD_BITS > 0`、`FecCodec::K > 0`、
  `FecCodec::N > K`、`FecCodec::K ≥ PAYLOAD_BITS` (FEC 容量が
  メッセージを保持する)、`FecCodec::N ≤ N_DATA × BITS_PER_SYMBOL`
  (符号語がチャネルシンボルに収まる)。

別のテストでは各 registry entry を ZST と**異なる経路**でクロス
検査する (名前検索 → 直接 trait 定数読み取り)。`protocol_meta!`
マクロ内のフィールド typo は `cargo build` を通してしまうが、
このクロスパスチェックで捕捉される。

これにより Q65 作業時に trait 面のドリフトを抑止できた —
`GRAY_MAP` の既存 doc 契約 `len() == NTONES` が JT9 (data tone
のみ 8 個に絞っている) で成立しない事実が顕在化し、契約を
`[2^BITS_PER_SYMBOL, NTONES]` に緩める変更を同じ pass で
入れることができた (誰かが trait ファイルを再読する記憶力に
依存せずに済んだ)。

新しい `Protocol` 実装の追加は機械的:

1. 新しい ZST に trait を実装する。
2. `registry.rs` の `PROTOCOLS` に `protocol_meta!("表示名",
   MyProtocolZst)` を 1 行追加。
3. `tests/protocol_invariants.rs` に対応する
   `assert_protocol_invariants::<MyProtocolZst>(...)` を 1 行追加。

新プロトコル固有のデコードテストを書く前に、構造的不整合は
CI で先に表面化する。

---

## ライセンス

リポジトリルートの [`LICENSE`](../../LICENSE) を参照。ここにある
アルゴリズムはすべて WSJT-X（Joe Taylor K1JT ら）に由来し、各ソース
ファイルが移植元の `lib/*.f90` / `lib/*.c` を明記している。
