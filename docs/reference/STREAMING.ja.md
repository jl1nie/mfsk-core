# mfsk-core — ストリーミングデコードインターフェイス

> **English:** [STREAMING.md](STREAMING.md)

本ドキュメントは mfsk-core の**ストリーミング配信**インターフェイス
を解説する: `DecodeRequest` / `SniperRequest` ファミリの
`.on_result(cb)` コールバックとプロトコルごとの兄弟関数、その配信契約
が保証するもの、**なぜ `async fn` / `Future` / チャネルベースの API で
はなく素の同期コールバックなのか**、そして Tokio 非同期クライアントへ
橋渡しする完全な実例。

ライブラリ全体（トレイト階層、DSP プリミティブ、C ABI）については
[LIBRARY.ja.md](LIBRARY.ja.md) を参照。本ドキュメントはそのうち §4 の
「ストリーミング配信」を深掘りし、非同期橋渡しの実例を追加したもの。

---

## 1. ここでの「ストリーミング」とは

デコードは**すでに手元にある 1 スロット分の音声**を対象とする —
FT8 なら 15 秒、WSPR なら 110.6 秒、といった単位である。開きっぱなし
のソケット読み込みではない。したがって「ストリーミング」は*サンプルを
逐次流し込む*という意味ではない（スロット全体を参照で渡す）。逆方向、
つまり**結果が逐次流れ出す**ことを指す。スロット全体のデコードが完了
してから単一の `Vec` として返すのではなく、デコーダがメッセージを見つ
けるたびに、受理された 1 件ごとにコールバックが 1 回発火する。

バッチ API とストリーミング API は**同じ呼び出し**である。ストリーミ
ングは純粋に加算的:

```rust
use mfsk_core::ft8::Ft8;
use mfsk_core::msg::decode_request::DecodeRequest;

// バッチ: 最後にまとめて受け取る。
let outcome = DecodeRequest::<Ft8>::new(&audio, 100.0, 3000.0, 1.5, 100)
    .decode();
for r in &outcome.results { /* ... */ }

// ストリーミング: 同じデコードに、途中で結果ごとに発火するコールバック
// を足すだけ。終了後も `outcome.results` はバッチ全体を保持している。
let on_result = |r: &mfsk_core::ft8::decode::DecodeResult| {
    // 候補が受理されるたびに発火
};
let outcome = DecodeRequest::<Ft8>::new(&audio, 100.0, 3000.0, 1.5, 100)
    .on_result(&on_result)
    .decode();
```

`.on_result()` を呼ばない呼び出し側にとっては、この機能が存在しなかっ
た頃と挙動上の差はまったくない。

### なぜストリーミングするのか

1 スロットのデコードは瞬時ではない。混雑した FT8 バンドでは OSD エスカ
レーションを伴うフルデプスの広帯域探索がホストでも体感できる程度に時間
を要し、組込みターゲットではさらに長い。ストリーミングにより、最初の
（通常は最も強い）デコードが得られた瞬間に UI に描画でき、最も高価な最
後の候補が深い OSD を抜けきるまでスピナーを眺めずに済む。ログ記録、QSO
状態機械、スポットのアップロードといった下流段も、後続の結果を計算中で
あっても早い結果に対して処理を開始できる。

---

## 2. プロトコルごとのエントリポイント

プロトコル横断の**共有コールバック型は存在しない** —
`DecodeResult`（FT8/FT4/FST4）、`Q65Result`、`WsprResult`、
`Jt65Result`、`Jt9Result` は構造的に別物である。よってこれは 1 つのトレ
イトで抽象化するものではなく、各プロトコル自身の API ファミリ内で同じ
名前・同じ意味論に従う一貫した*パターン*である。

| プロトコル           | エントリポイント                                                        | コールバック型                          |
|----------------------|------------------------------------------------------------------------|-----------------------------------------|
| FT8 / FT4 / FST4     | `DecodeRequest<P>` / `SniperRequest<P>` — `.on_result(cb)`             | `&(dyn Fn(&DecodeResult) + Sync)`       |
| Q65                  | `DecodeRequest`/`SniperRequest`/`MultiPeriodRequest` — `.on_result(cb)` | `&(dyn Fn(&Q65Result) + Sync)`          |
| WSPR                 | `decode_scan_streaming` / `decode_scan_subtract_streaming`             | `&(dyn Fn(&WsprResult) + Sync)`         |
| JT65                 | `decode_scan_streaming`                                                | `&(dyn Fn(&Jt65Result) + Sync)`         |
| JT9                  | `decode_scan_streaming`                                                | `&(dyn Fn(&Jt9Result) + Sync)`          |
| FT8（`ft8::decode_block`） | `ft8::decode_block::decode_block_streaming`                      | `&mut dyn FnMut(&DecodeResult)`         |

補足:

- **ビルダ（FT8/FT4/FST4/Q65）**は `.on_result(cb)` をもう 1 つの連鎖可
  能なメソッドとして持つ。返される `DecodeOutcome` は引き続きバッチ全体
  を保持する。
- **WSPR/JT65/JT9 にはビルダがない**ため、既存の `decode_scan` に対する
  `decode_scan_streaming` *兄弟関数*が追加された（WSPR には
  `decode_scan_subtract` に対する `decode_scan_subtract_streaming` もあ
  る）。兄弟関数は同じ引数に末尾の `on_result` 参照を足したもので、非ス
  トリーミング版と同じ `Vec` を返す。
- **`ft8::decode_block::decode_block_streaming`** は 2 つの feature 分岐
  版どちらでも `&dyn Fn + Sync` ではなく `&mut dyn FnMut` を取る:
  組込み（`not(fft-rustfft)`）の単一パスパイプラインは厳密に逐次
  （no_std では rayon なし）であり、ホスト（`fft-rustfft`）のマルチパス/
  減算ドライバ（`decode_block_multipass`）も同様に常に逐次（`DecodeRequest`
  の単一パス/sniper 戦略と異なり内部に rayon を持たない）なので、捕捉し
  た状態を変更する `FnMut` クロージャはどちらでも安全であり、`Sync`
  境界はどちらにも不要。issue #243 以前はホスト版でここにコールバックを
  安全に出せなかった: `xsnr2` SNR 妥当性ゲート（`ft8b.f90:456`）が減算が
  終わった*後*に事後バッチとして走っており、結果が*すでにストリームさ
  れた後*にそれを破棄・変更しうる一方、このコールバックには修正/撤回イ
  ベントがなかったため。現在はこのゲートを候補ごとに即座に（その候補の
  信号を減算した直後、受理する前に）インラインで走らせるようにしたため、
  両方の feature 分岐が下記 §3a の完全一致契約を共有する。ホスト側の完
  全一致検証は `tests/ft8_decode_block_streaming_host.rs` を参照（組込
  み側の `tests/ft8_decode_block_streaming.rs` と対をなす）。

---

## 3. 配信契約

契約はちょうど**2 種類**あり、戦略が逐次で走るか `rayon`
（`feature = "parallel"`）下で走るかで決まる。正典は
`DecodeRequest::on_result` の doc コメントで、ここではその要約を示す。

### 3a. 逐次 — 完全一致

`cb` は**返される `Vec` に最終的に載る結果ごとに、同じ順序でちょうど
1 回**発火する。ストリームした内容とバッチ返り値の間に乖離はゼロ。

対象: `.sic_rounds(n)` と `.sic_early()`（FT8/FT4 の SIC 戦略）、
`ft8::decode_block::decode_block_streaming`（issue #243 以降、組込み・
ホスト `fft-rustfft` 両分岐とも）、JT65 と JT9 の `decode_scan_streaming`、
Q65 の全ビルダ、WSPR の `decode_scan_subtract_streaming`（自身の外側
SIC パスの受理点でのみ発火）。Q65 の `MultiPeriodRequest` は逐次形の一
変種で、候補ごとではなく受理デコードを生む**スロットごと**に 1 回発火
する（複数周期 EME / 電離層散乱の平均化における自然なストリーミング単
位）。

### 3b. 並列 — 完了順、一時的な重複がありうる

`cb` は**その候補をデコードしたスレッドから、完了順**（候補探索順では
ない）に、そして最終的なクロス候補デデュープパスの**前**に発火する。
2 つの同期候補が同じメッセージに収束する稀なケースでは、`Vec` に残るの
は 1 件だけでも `cb` は両方に対して発火しうる。バッチと厳密に一致させた
い呼び出し側は自分の側で `.message77()` によるデデュープを行うこと —
クレート自身のデデュープが使うのと同じキーである。

対象: デフォルトの単一パス戦略と `SniperRequest`（FT8/FT4）、WSPR の
`decode_scan_streaming`（両方の粗探索パスが `rayon::par_iter()` 下で走
る）。

**これが並列パスのコールバックに `Sync` が必要な理由である** — 複数の
rayon ワーカスレッドから並行して呼ばれうる。

### 配信順は強い信号を優先するか

両ファミリで、そうなる傾向はある: `coarse_sync` は候補を Costas 同期ス
コアの降順で返し、逐次ループも並列スイープもそのリストを順に処理するた
め、スコアの高い（同期スコアは SNR と相関するので通常はより強い）候補
が先に現れやすい。ただし**相関であって保証ではない** — 同期スコアはデ
モジュレーション前の相関電力の測定値であり、デモジュレーション後の
BP/OSD コストの予測子ではない。よってスコアの高い候補がフル OSD エスカ
レーションを要する一方、スコアの低い候補が 1 回の BP パスで収束すること
もある。逐次戦略に限っては、深い OSD を要するリスト前方の候補が後続すべ
ての候補をブロックする（単一スレッド）; 並列戦略にこのヘッドオブライン
ブロッキングはない。

---

## 4. なぜ同期コールバックで、`async` / Tokio / チャネルではないのか

これは未完成ではなく意図的な設計判断である。要約すると: **mfsk-core は
ランタイム非依存を保ち、各コンシューマ（ランタイムをまったく持てないも
のを含む）が自身の並行モデルを選べるようにする。そして端で Tokio に橋
渡しするのは自明（§5）なので、コアから async を排しても失うものはな
い。**

理由を重み順に:

### 4a. 移植性 — コアは `std`・エグゼキュータ非依存でなければならない

mfsk-core が掲げる目標は「複数のランタイム（ネイティブ Rust、
WebAssembly、Android JNI、C ABI）から同一に消費される」単一クレートで
ある。`engine` とプロトコル層が `no_std` クリーンなのは、ESP32 組込み
ターゲット（`embedded-poc/m5stack-*-app`）が**まさにこのデコードパスの
一級コンシューマ**だからだ — 同じ `decode_block` ストリーミングコール
バックが、アロケータ付きエグゼキュータも `std` も持たない Xtensa LX7 上
で走る。

`async fn` / `Future` を返す API — あるいは既定でチャネルやエグゼキュー
タを要求する何か — は、`std` とランタイムをコールグラフ*全体*に引き込
み、それらの `no_std` ターゲット、`wasm32-unknown-unknown` ビルド、
C ABI（`libmfsk.so`）、JNI スキャフォールドを壊す。同期 `Fn` コールバッ
クはそのすべてで無変更でコンパイルされる。

### 4b. `await` するものがない

async は**I/O バウンド**で中断点の多い処理 — ソケット・タイマ・ディス
ク待ち — に適した道具である。デコードはその正反対で、**固定のインメモ
リバッファに対する CPU バウンドな計算**を最初から最後まで行い、譲るべ
き外部イベントを持たない。これを `async` にしても、ランタイム機構と関数
色の伝播を増やすだけで、レイテンシもスループットも**まったく**得られな
い — リアクタが代わりに有用な仕事をできる await 点が存在しないからだ。

### 4c. ランタイム選択を強制しない（関数の色付けがない）

`async` なデコード API はその上のコールスタック全体を色付けする: すべて
の呼び出し側も `async` になり、*何らかの*エグゼキュータを走らせねばな
らない。同期コールバックはそれを一切強制しない。呼び出し側がモデルを選
ぶ — Tokio、`async-std`、素の `std::thread`、GUI イベントループ、素の
組込みスーパーループ、あるいは何も無し — そして mfsk-core はどれかを知
る必要がない。`tokio::sync::mpsc`（や任意のチャネル型）を焼き込めば、ラ
ンタイムを使えないコンシューマに特定のランタイムを強制し、呼び出し側が
1 行でできる仕事のためにクレートが重いオプション依存を背負うことにな
る。

### 4d. コードベース内の前例

素のコールバックのイディオムはすでにここにある:
`process_candidates_with_ap` は充填クロージャ
（`F: FnMut(&mut [[Cmplx<f32>;8];79], &SyncCandidate, SymMask)`）を取
る。`.on_result()` は 2 つ目の async 風パターンを隣に導入するのではな
く、同じ形に従う。

### 帰結

mfsk-core は*あなたが*供給するクロージャを通じて結果を配信するので、そ
れがどうスレッドやランタイムを越えるかはあなたが握る。Tokio チャネルに
乗せたい? クロージャに `Sender` を入れる。GUI スレッドに乗せたい? クロー
ジャからイベントループへポストする。スロット跨ぎのバックグラウンド継続
（WSJT-X「Fast」モード型 — 次スロットのキャプチャ開始後もデコードを続け
る）が欲しい? `std::thread::spawn` してそこで `.decode()` を呼ぶ。どれ
もコアライブラリの支援を要さず、すべてアプリケーションの端で組み上が
る。

---

## 5. 実例: Tokio 非同期クライアントから呼ぶ

目標: 1 つの 15 秒 FT8 スロットを**非同期ランタイムをブロックせず**にデ
コードし、各メッセージを最後にまとめてではなく**デコードされた瞬間に**
`async` ループで受け取る。

形を決めるのは 2 つの事実:

1. **デコードはブロッキングな CPU バウンド処理である。** Tokio のワーカ
   スレッド上で走らせてはならない（リアクタを止めてしまう）。
   `tokio::task::spawn_blocking` で走らせる。
2. **コールバックが橋渡しである。** `tokio::sync::mpsc::Sender` を捕捉
   し、借用された各 `DecodeResult` を所有権を持つ `Send` な値に変換して
   送る。mfsk-core はチャネルもランタイムも `async` も一切見ない。

### `Cargo.toml`

```toml
[dependencies]
# `Decoded` + `to_decoded` は 0.9 で導入。デコード行を JSON 化したいなら
# `features = ["serde"]` を足す。
mfsk-core = "0.9"
tokio = { version = "1", features = ["rt-multi-thread", "macros", "sync"] }
# 任意、§5.3 の Stream アダプタ用のみ:
tokio-stream = "0.1"
```

### 5.1 橋渡し

```rust
use mfsk_core::engine::protocol::ProtocolId;
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::DecodeResult;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::decoded::Decoded; // クレート提供の所有・Send な UI 行

use tokio::sync::mpsc;

/// 1 つの 15 秒 FT8 スロット（12 kHz モノラル i16 PCM）をブロッキングワーカ
/// 上でデコードし、受理されたメッセージを届き次第 async 呼び出し側へ流す。
///
/// チャネルの受信側を即座に返し、デコードは spawn_blocking 上で走る。
/// デコード終了時にチャネルが閉じる。
pub fn decode_slot_stream(audio: Vec<i16>) -> mpsc::Receiver<Decoded> {
    // 有界にして、遅いコンシューマに対しメモリを無限に伸ばす代わりに
    // バックプレッシャをかける。1 スロットが 64 件に届くことはまずないので、
    // ここで生産側がブロックすることは実際上ほぼない。
    let (tx, rx) = mpsc::channel::<Decoded>(64);

    tokio::task::spawn_blocking(move || {
        // このクロージャが mfsk-core の同期世界と Tokio の async 世界を
        // つなぐ橋渡しのすべて。`Fn`（`&self` メソッドの
        // `Sender::blocking_send` のみ）かつ `Sync` で、`.on_result` の
        // `&(dyn Fn(&DecodeResult) + Sync)` 境界を満たす —— FT8 の既定の
        // 広帯域戦略が候補を rayon ワーカスレッドへ分配し、これを並行に
        // 呼びうるため必須。
        let on_result = move |r: &DecodeResult| {
            // `DecodeResult::to_decoded` が 77 ビットペイロードを unpack し、
            // 所有・Send な `Decoded` 行（text + freq + dt + snr + protocol）を返す
            // —— まさにチャネル越しに move したいもの。`None` は unpack 不能を意味し、
            // ゴミ行を送らずスキップする。（クレートが `Decoded` を出す前は、この
            // クロージャは所有構造体を手組みし `unpack77` を自前で呼んでいた。今は1呼び出し。）
            if let Some(d) = r.to_decoded(ProtocolId::Ft8, None) {
                // ここでの `blocking_send` は正しい: これは spawn_blocking スレッド
                // （並列戦略では rayon ワーカでもありうる）上で走り、Tokio ランタイム
                // ワーカでは決してない —— よって async コンテキスト内で `blocking_send`
                // が起こすようなパニックはしない。送信エラーは受信側が drop された
                // ことを意味し、それ以上することはない。
                let _ = tx.blocking_send(d);
            }
        };

        // 広帯域探索 100–3000 Hz、sync_min 1.5、最大 100 候補。
        // `.on_result` は加算的 —— 返る DecodeOutcome は引き続きバッチ全体を
        // 保持するが、ここでは全件ストリームしたので破棄する。
        let _outcome = DecodeRequest::<Ft8>::new(&audio, 100.0, 3000.0, 1.5, 100)
            .on_result(&on_result)
            .decode();
        // タスク復帰時にここで `on_result`（と捕捉した `tx`）が drop され、
        // チャネルが閉じてコンシューマのループが終わる。
    });

    rx
}
```

> `Decoded`（`mfsk_core::msg::decoded::Decoded`）はクレートの統一・所有
> デコード行 —— `text` / `freq_hz` / `dt_sec` / `snr_db` / `protocol`、
> `Clone` + `Send`、`--features serde` で `Serialize`/`Deserialize`。
> 各プロトコルのネイティブ結果に `to_decoded(..)` 変換があり
> （WSPR/Q65/JT65/JT9 も）、同じ橋渡し形が全モードで使える。
> [LIBRARY.ja.md](LIBRARY.ja.md) と `docs/notes/DECODED_ROW.md` を参照。

### 5.2 ストリームを消費する

```rust
#[tokio::main]
async fn main() {
    // あなたのキャプチャパイプラインが供給する: スロット境界に整列した
    // 12 kHz モノラル i16 PCM の 15 秒スロット 1 つ（約 180,000 サンプル）。
    let audio: Vec<i16> = load_one_ft8_slot();

    let mut rx = decode_slot_stream(audio);

    // 各メッセージは、末尾でまとめてではなくデコーダが受理した瞬間にここへ
    // 届く。デコードタスクが終わって Sender を drop するとループが抜ける。
    while let Some(msg) = rx.recv().await {
        println!(
            "{:+5.1} dB  {:7.1} Hz  dt={:+.2}s  {}",
            msg.snr_db, msg.freq_hz, msg.dt_sec, msg.text,
        );
        // ...あるいは `msg` を QSO 状態機械、スポットアップローダ、
        // websocket、DB 書き込みへ転送 —— すべてここから `.await` 可能。
    }

    println!("スロットのデコード完了");
}

# fn load_one_ft8_slot() -> Vec<i16> { Vec::new() }
```

### 5.3 任意: `Stream` として公開する

コンビネータベースのコンシューマ
（`while let Some(x) = stream.next().await`、`.map()`、`.filter()`）へ
渡したい場合は、受信側をラップする:

```rust
use tokio_stream::wrappers::ReceiverStream;
use tokio_stream::StreamExt;

let stream = ReceiverStream::new(decode_slot_stream(audio));
tokio::pin!(stream);
while let Some(msg) = stream.next().await {
    // §5.2 と同じ。ただし StreamExt のコンビネータと合成可能
}
```

### 5.4 よくあるバリエーション

- **ブロックしない生産側。** 遅いコンシューマでデコードスレッドを決して
  ブロックさせず、むしろ結果を落としたい場合は、`blocking_send` を
  `try_send` に替えて `Err(TrySendError::Full)` を処理する（例: 落とし
  た件数を数える）。有界チャネルと `blocking_send` の組み合わせでは、詰
  まったコンシューマは代わりにバックプレッシャをかけてデコードを遅らせ
  る —— 正しさが重要な UI では通常こちらが望ましい。
- **逐次・完全一致配信。** §3a のより強い契約（コールバック順 == バッチ
  順、一時的重複なし）が欲しければ、既定の広帯域パスではなく逐次戦略 ——
  例えば FT8 の `.sic_rounds(3)` や `.sic_early()` —— を使う。橋渡しコー
  ドは同一で、変わるのはビルダのメソッドだけ。
- **他プロトコル。** WSPR/JT65/JT9 では、同じ `spawn_blocking` の殻の中
  でフリー関数 `decode_scan_streaming(&audio, sample_rate,
  nominal_start_sample, &params, &on_result)` を呼ぶ。クロージャは同じ
  `Sender` を捕捉する。Q65 では上の FT8 とまったく同様に
  `DecodeRequest`/`SniperRequest`/`MultiPeriodRequest` ビルダを使う。
- **キャンセル。** `Receiver` を drop すると、クロージャ内の次の
  `blocking_send` が `Err` を返すので早期に止められる —— ただしデコード
  自体に内部キャンセル点はないため、`spawn_blocking` タスクは何であれ完
  了まで走る。ハードなキャンセルには、より短い単位（候補ごとの
  `SniperRequest` 呼び出し）でデコードし、その間でフラグを確認する。

---

## 6. 関連

- [LIBRARY.ja.md](LIBRARY.ja.md) §4 —— ライブラリ API リファレンス内の
  ストリーミング節、およびそれが属する `DecodeRequest` /
  `SniperRequest` ビルダ面。
- `DecodeRequest::on_result` の doc コメント
  （`mfsk-core/src/msg/decode_request.rs`）—— 正典であり常に最新の配信
  契約。
- `mfsk-core/tests/ft8_decode_block_streaming.rs` —— 組込み
  `decode_block_streaming` の完全一致テスト。
- `mfsk-core/tests/ft8_decode_block_streaming_host.rs` —— ホスト
  `fft-rustfft` 版 `decode_block_streaming` の完全一致テスト（issue #243）。
- `mfsk-core/tests/wspr_wsjtx_samples.rs` —— 実信号に対する WSPR
  `decode_scan_streaming` / `decode_scan_subtract_streaming`。
- [EMBEDDED.ja.md](EMBEDDED.ja.md) —— `mfsk-ffi-ft8` FFI 成果物向けの
  C ABI ストリーミングチュートリアル（同じ移植性の理由で、C 境界越しに
  も同じストリーミングの考え方をコールバックベースで実現）。
