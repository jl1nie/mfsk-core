# mfsk-core — ストリーミングデコードインターフェイス

> **English:** [STREAMING.md](STREAMING.md)

本ドキュメントは mfsk-core の**ストリーミング配信**インターフェイス
を解説する: `Decoder::decode_with` とその行コールバック、その配信契約
が保証するもの、**なぜ `async fn` / `Future` / チャネルベースの API で
はなく素の同期コールバックなのか**、そして Tokio 非同期クライアントへ
橋渡しする完全な実例。

ライブラリ全体（`Decoder` モデル、トレイト階層、DSP プリミティブ）は
[LIBRARY.ja.md](LIBRARY.ja.md)、C ABI は
[BINDINGS.ja.md](BINDINGS.ja.md) を参照。本ドキュメントは
LIBRARY.ja.md の §2.4「ストリーミング配信」を深掘りし、非同期橋渡しの
実例を追加したもの。

---

## 1. ここでの「ストリーミング」とは

デコードは**すでに手元にある 1 スロット分の音声**を対象とする —
FT8 なら 15 秒、WSPR なら 110.6 秒、といった単位である。開きっぱなし
のソケット読み込みではない。したがって「ストリーミング」は*サンプルを
逐次流し込む*という意味ではない（スロット全体を参照で渡す）。逆方向、
つまり**結果が逐次流れ出す**ことを指す。スロット全体のデコードが完了
してから単一の `SlotResult` として返すのではなく、デコーダがメッセージを見つ
けるたびに、受理された 1 件ごとにコールバックが 1 回発火する。

バッチ API とストリーミング API は**同じデコーダ**である。ストリーミ
ングは純粋に加算的:

```rust
use mfsk_core::decoder::{DecodeParams, Decoder, Row, SlotInput};
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::DecodeResult;

let mut decoder = Decoder::<Ft8>::new(DecodeParams::for_band((100.0, 3000.0)));
let slot = SlotInput::i16(&audio);

// バッチ: 最後にまとめて受け取る。
let result = decoder.decode(&slot);
for row in &result.rows { /* ... */ }

// ストリーミング: 同じデコードに、途中で行ごとに発火するコールバック
// を足すだけ。終了後も `result.rows` はバッチ全体を保持している。
let on_row = |row: &Row<DecodeResult>| {
    // 候補が受理されるたびに発火。row.decoded はモード共通の行
};
let result = decoder.decode_with(&slot, &on_row);
```

`decode_with` を使わない呼び出し側にとっては、`decode` と挙動上の差はまったくない。

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

行の型は `Row<R>` である: モード共通の `decoded: Decoded`（テキスト・周波数・dt・SNR・
プロトコル）、`detail: RowDetail`、そしてモード固有の結果 `native: R` —— `DecodeResult`
（FT8/FT4/FST4）、`Q65Result`、`WsprResult`、`Jt65Result`、`Jt9Result` は構造的に別物
である。よってコールバック型は、モードごとの `R` を埋めた 1 つの汎用の形
`&(dyn Fn(&Row<R>) + Sync)`（`decoder::OnRow`）である。

| プロトコル           | エントリポイント                                  | コールバック型                          |
|----------------------|---------------------------------------------------|-----------------------------------------|
| FT8 / FT4 / FST4     | `Decoder<P>::decode_with(&slot, on_row)`          | `&(dyn Fn(&Row<DecodeResult>) + Sync)`  |
| Q65                  | `Decoder<Q65…>::decode_with`                      | `&(dyn Fn(&Row<Q65Result>) + Sync)`     |
| WSPR                 | `Decoder<Wspr>::decode_with`                      | `&(dyn Fn(&Row<WsprResult>) + Sync)`    |
| JT65                 | `Decoder<Jt65>::decode_with`                      | `&(dyn Fn(&Row<Jt65Result>) + Sync)`    |
| JT9                  | `Decoder<Jt9>::decode_with`                       | `&(dyn Fn(&Row<Jt9Result>) + Sync)`     |
| 実行時に選ぶ任意のモード | `AnyDecoder::decode_with`                      | `&(dyn Fn(&Decoded, &RowDetail) + Sync)` |
| FT8（`ft8::decode_block`） | `ft8::decode_block::decode_block_streaming`  | `&mut dyn FnMut(&DecodeResult)`         |
| JTTY                 | `jtty::rx::Stream::push(samples, &mut cb)`        | `&mut dyn FnMut(MessageUpdate)`         |

補足:

- **すべての `Decoder`** は `decode` の隣に `decode_with` を持つ。返される
  `SlotResult` は引き続きバッチ全体を保持する。
- **行は配信時に解決される。** コールバックが見る行のテキストは、周期の開始時点の
  デコーダのコールサインハッシュ表で解決されたものである。`decode_with` が返す行は
  同じ周期内で先に学習された呼出符号も見る（ハッシュは候補ループの後に、デコード順に
  学習される）。そのため、ストリームされた行の `<...>` が、返された行では解決済みに
  なることがある。ストリームされた行と返された行を対応づけるには
  `RowDetail::delivery`（#592）を使う。ストリームされた行はその周期の配信の中での自分の
  位置を持ち、返された行は自分だった配信の位置（ビット・周波数・時刻が同じ最初の配信、
  厳密比較）を持つので、キーも丸めも要らない。コールバックが見なかった返却行と、
  `std` 無しのビルドでは `None`。デコーダや周期をまたいで比べるときは、テキストではなく
  メッセージのビットを比べること。型付きの行なら `row.native.message77()`、どの行でも
  `RowDetail::info`（全モードが埋める）。同じメッセージが 2 つの周波数にあってもキーは
  1 つなので、周波数を足すこと。ペイロードが
  unpack できない候補は、どちらでも配信されない。
- **平均を使う Q65**（`averaging` が有効で `SlotInput::period` あり）は 1 周期につき
  高々 1 つの結果を出すので、コールバックは周期ごとに 1 回発火する。
- **WSPR/JT65/JT9 にはかつてビルダがなく**、それぞれフリー関数の
  `decode_scan_streaming` *兄弟関数*を生やしていた —— 軸が 1 つ増える
  たびにこの形を繰り返した結果、3 モードで公開 `decode_*` 関数が 32 個に
  なった。issue #403 でモードごとのリクエスト型に置き換え、0.13 で
  `Decoder<P>` に置き換えて、ストリーミングは他と同じくメソッド 1 つになった。
- **`ft8::decode_block::decode_block_streaming`** は 2 つの feature 分岐
  版どちらでも `&dyn Fn + Sync` ではなく `&mut dyn FnMut` を取る:
  組込み（`not(fft-rustfft)`）の単一パスパイプラインは厳密に逐次
  （no_std では rayon なし）であり、ホスト（`fft-rustfft`）のマルチパス/
  減算ドライバ（`decode_block_multipass`）も同様に常に逐次（単一パス/sniper
  戦略と異なり内部に rayon を持たない）なので、捕捉し
  た状態を変更する `FnMut` クロージャはどちらでも安全であり、`Sync`
  境界はどちらにも不要。issue #243 以前はホスト版でここにコールバックを
  安全に出せなかった: `xsnr2` SNR 妥当性ゲート（`ft8b.f90:483`）が減算が
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
（`feature = "parallel"`）下で走るかで決まる。正典はエンジンの行コールバックの
doc コメント（`DecodeRequest::on_result`。今は crate 非公開で、`Decoder::decode_with` が
それを包む）で、ここではその要約を示す。どの戦略が走るかは `Depth` が決める
（[LIBRARY.ja.md](LIBRARY.ja.md) §2.2）ので、契約は depth に従う:

### 3a. 逐次 — 完全一致

`cb` は**返される `SlotResult` に最終的に載る行ごとに、同じ順序でちょうど
1 回**発火する。ストリームした内容とバッチ返り値の間に乖離はゼロ
（§2 のハッシュ解決を除く: テキストではなくメッセージのビットで比べること）。

対象: FT8 の `SicRounds(n)` と `SicEarly`、FT4 の `SicRounds(n)` —— FT8 と FT4 で
`Depth::Normal` と `Depth::Deep` が走らせるもので、既定ブロックの depth は `Deep`、
`ft8::decode_block::decode_block_streaming`（issue #243 以降、組込み・
ホスト `fft-rustfft` 両分岐とも）、JT65 と JT9 のデコーダ、Q65 のスキャン。
`averaging` を有効にした Q65 は逐次形の一
変種で、候補ごとではなく受理デコードを生む**周期ごと**に 1 回発火
する（複数周期 EME / 電離層散乱の平均化における自然なストリーミング単
位）。

### 3b. 並列 — 完了順、一時的な重複がありうる

`cb` は**その候補をデコードしたスレッドから、完了順**（候補探索順では
ない）に、そして最終的なクロス候補デデュープパスの**前**に発火する。
2 つの同期候補が同じメッセージに収束する稀なケースでは、返される行に残るの
は 1 件だけでも `cb` は両方に対して発火しうる。バッチと厳密に一致させた
い呼び出し側は自分の側でメッセージのビットによるデデュープを行うこと —
型付きの行なら `.message77()`、`AnyDecoder` や C ABI の行なら `RowDetail::info`。
クレート自身のデデュープが使うのと同じキーである。

`AnyDecoder::delivery_is_exact()`（と `Decoder::delivery_is_exact()`、C の `mfsk_decoder_delivery_is_exact`、Kotlin と Swift の `deliveryIsExact`）で、現在の
モード・depth・extras がどちらの契約で動くか分かる。`true` なら §3a、`false` は
「§3b、保証なし」で、`true` が返る呼び出し側はガードを省略できる。depth や extras を
変えたら問い直すこと。

対象: 単一パス戦略（FT4 の `Fast` depth である `SinglePass` と、FST4 の唯一の戦略）と
FT8 の `sniper` モード、WSPR（パス 1・パス 2 の候補ループが `rayon::par_iter()`
下で走る）。

**これがコールバックに `Sync` が必要な理由である** — 複数の
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

### 監査済み: 「revoke-less retract」の欠陥はどこにも存在しない（2026-08-09）

*revoke-less retract*（取り消しイベントなしの遡及的除外）は上記のどち
らの契約とも異なる特有の失敗モードである: `cb` がある候補に対して発火
した後、発火点より別の後処理ステップによって、その候補が最終的に返さ
れる `Vec` に一切現れない、というもの——§3b の「完了順で重複しうる」
という文書化済みの挙動ではなく、コールバックが配信を確定した後にゲー
トやフィルタが走った結果として起きる、修正/撤回イベントの一切ない黙示
的な除外である。これはどちらの契約よりも悪い: §3b の弱い保証（バッチ
の結果は必ず1回以上発火する）すら暗黙のうちに破りうる——呼び出し側が
「ストリームされた=本物」に依存し始めた途端、配信集合とバッチ `Vec` が
食い違う。

この形の不具合は実際に2回、FT8/FT4/FST4（`.known(...)` というフェー
ズ横断dedupビルダメソッドを持っていた唯一のプロトコル群——WSPR/Q65/JT65/JT9
にはこの概念自体が存在しないため、そもそも晒されようがなかった）で発生
した。0.13 は `.known()` を公開 API から外した（その役目は今はデコーダの状態）ので、
`Decoder` の呼び出し側にはこの組み合わせはもう存在しない。エンジンのリクエストは
`internal-testing` の背後にこれを残しており、下の表が監査するのはその修正である:

1. **FT8ホストのマルチパスドライバ**（issue #243）—— `xsnr2` SNR妥当
   性ゲートが、1パス全体（あるいは修正初期段階では3パス全体）が終
   わった後にバッチとして走っており、その時点ですでに候補に対して
   `on_result` が発火済みだった。候補ごとに、受理される前にインライ
   ンでゲートを走らせるよう修正した。
2. **FT8の`SicEarly`、FT4の`SicRounds`/単一パス、FST4の単
   一パス**—— `.known(...)` は入力オーディオから事前に減算されていた
   （これ自体は正しい）が、実際の候補ごとのdedupには一切組み込まれ
   ておらず、呼び出し側レベルの事後フィルタが `known`
   に一致するものを黙って落としていた——`on_result` がすでに発火した
   後に。`known` による判定をコールバック発火点より前でアトミックに
   行うよう修正した（FT8: 既存の候補ごとdedupに `known` を組み込み;
   FT4/FST4: 下層の共有ジェネリックエンジンには `known` を通すパラメー
   タが存在しないため、`pipeline::known_filtered_on_result` というラッ
   パで対応）。

どちらも机上の検討だけでなく、実信号に対する再現実験で見つけたもの
——常設の回帰テストとして
`tests/ft8_streaming_sic_early_with_known_matches_batch_exactly` と
`tests/ft4_streaming_sic_rounds_with_known_matches_batch_exactly` を参
照。

両修正の後、クレート内の全ての `on_result`/`cb` 呼び出し箇所を（上
記修正からの推測ではなく）直接再監査した。いずれも、`cb` が発火する値/
集合と、その後返されるコレクションにコミットされる値/集合が完全に一致
し、両者の間にフィルタリングステップが一切存在しない——多くは
`if let Some(cb) = on_result { cb(&r); } vec.push(r);` という単一ブロッ
ク、まれに（FT4/FST4の `decode_frame_subtract`）`for r in &deduped {
cb(r); }` ループの直後に何も挟まず `all_results.extend(deduped)` が続
く形。どちらの形でも同じ保証が得られる: コールバックがすでに発火した値
は、その後いかなる手段によっても集合から取り除かれえない。

0.13 の `Decoder` はそれらの箇所の上に 1 層を足すが、保証は保たれる: そのラッパは
エンジンの結果を `Row` に変換し（ペイロードをハッシュ表で unpack する）、unpack できない
結果を、コールバックからも返される行からも同様に落とす（`decoder/frame.rs` の
`frame_decode`）。

| 箇所（エンジン。行番号はコミット `1a4cbda` 時点。このファイルがそれ以降変化していたら要再確認） | 場所 |
|---|---|
| FT8 `decode_block_multipass`/`decode_block_streaming` | `ft8/decode_block/process_candidates.rs:486` |
| FT8 `sic_inner_passes_with_cache`（`SicRounds`/`SicEarly` を担当） | `ft8/decode.rs:630` |
| FT8 `decode_frame_inner`（並列/逐次の単一パス） | `ft8/decode.rs:374,387` |
| FT8 `decode_sniper_inner`（並列/逐次のsniper） | `ft8/decode.rs:996,1016` |
| FT4/FST4 `decode_frame`（並列/逐次の単一パス、ジェネリックエンジン） | `engine/pipeline.rs:994,1014` |
| FT4/FST4 `decode_frame_subtract`（`SicRounds`、ジェネリックエンジン） | `engine/pipeline.rs:1239` |
| WSPR スキャン（`decode_scan_inner`、パス 1 / パス 2） | `wspr/decode.rs` |
| Q65 スキャン | `q65/decode_request.rs:374` |
| Q65 平均経路（`decode_multi_period_for`） | `q65/rx.rs:1345` |
| Q65内部スキャンヘルパ（`decode_scan_fading_for`、`decode_scan_with_ap_list_for`、`decode_scan_inner`） | `q65/rx.rs:511,610,700` |
| JT65 スキャン（`decode_scan_inner`、#403 以降は通常と Chase で 1 本のループ） | `jt65/mod.rs` |
| JT9 スキャン（`decode_scan_inner`） | `jt9/mod.rs` |

新しく `_streaming` 兄弟関数や行コールバックのフックをあるプロトコルに追加する際、
そのプロトコルがフェーズ横断dedupパラメータも持つ（または将来持つ）なら、その組み合わせこそ
がこのバグクラスの温床である——コールバックが、上記の全行がそうしてい
るように、返すコレクションへのコミットと同じ分岐から発火しているか
を確認すること。「戻り値だけをフィルタしているから無害」と決めつけて
はいけない。

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
る。``decode_with` の行コールバックは 2 つ目の async 風パターンを隣に導入するのではな
く、同じ形に従う。

### 帰結

mfsk-core は*あなたが*供給するクロージャを通じて結果を配信するので、そ
れがどうスレッドやランタイムを越えるかはあなたが握る。Tokio チャネルに
乗せたい? クロージャに `Sender` を入れる。GUI スレッドに乗せたい? クロー
ジャからイベントループへポストする。スロット跨ぎのバックグラウンド継続
（WSJT-X「Fast」モード型 — 次スロットのキャプチャ開始後もデコードを続け
る）が欲しい? `Decoder` を `std::thread::spawn` に move してそこで `decode()` を呼ぶ
（`Decoder<P>` は `Send`）。どれもコアライブラリの支援を要さず、すべてアプリケーションの
端で組み上がる。

---

## 5. 実例: Tokio 非同期クライアントから呼ぶ

目標: FT8 スロットを**非同期ランタイムをブロックせず**にデコードし、各メッセージを
スロット末尾でまとめてではなく**デコードされた瞬間に** `async` ループで受け取る。

形を決めるのは 3 つの事実:

1. **デコードはブロッキングな CPU バウンド処理である。** Tokio のワーカ
   スレッド上で走らせてはならない（リアクタを止めてしまう）。
   `tokio::task::spawn_blocking` で走らせる。
2. **デコーダは状態を持つ。** `Decoder` はコールサインのハッシュ表（と FT8 の a7 リスト）を
   スロットからスロットへ保持するので、1 つのブロッキングタスクがチャンネルの存続中 1 つの
   デコーダを所有し、スロットを供給される。スロットごとに新しいデコーダを作ると、ハッシュ化
   された呼出符号を全て忘れる。
3. **コールバックが橋渡しである。** `tokio::sync::mpsc::Sender` を捕捉
   し、借用された各行が持つ所有権付きの `Decoded` を clone して送る。
   mfsk-core はチャネルもランタイムも `async` も一切見ない。

### `Cargo.toml`

```toml
[dependencies]
# デコード行を JSON 化したいなら `features = ["serde"]` を足す。
mfsk-core = "0.13"
tokio = { version = "1", features = ["rt-multi-thread", "macros", "sync"] }
# 任意、§5.3 の Stream アダプタ用のみ:
tokio-stream = "0.1"
```

### 5.1 橋渡し

```rust
use mfsk_core::decoder::{DecodeParams, Decoder, Row, SlotInput};
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::DecodeResult;
use mfsk_core::msg::decoded::Decoded; // クレート提供の所有・Send な UI 行

use tokio::sync::mpsc;

/// 1 つの 15 秒 FT8 スロット（12 kHz モノラル i16 PCM）と、その UTC グリッド上の番号。
pub struct Slot {
    pub period: i64,
    pub audio: Vec<i16>,
}

/// ブロッキングワーカ上で FT8 デコーダを起動する。スロットを送ると、受理された
/// メッセージが届き次第、返されるチャネルから戻ってくる。
///
/// ワーカは `slots` が drop されるまで `Decoder`（とそのハッシュ表）を所有し、
/// 終了すると返されるチャネルが閉じる。
pub fn spawn_ft8_worker(
    params: DecodeParams,
) -> (std::sync::mpsc::Sender<Slot>, mpsc::Receiver<Decoded>) {
    let (slot_tx, slot_rx) = std::sync::mpsc::channel::<Slot>();
    // 有界にして、遅いコンシューマに対しメモリを無限に伸ばす代わりに
    // バックプレッシャをかける。1 スロットが 64 件に届くことはまずないので、
    // ここで生産側がブロックすることは実際上ほぼない。
    let (tx, rx) = mpsc::channel::<Decoded>(64);

    tokio::task::spawn_blocking(move || {
        let mut decoder = Decoder::<Ft8>::new(params);
        while let Ok(slot) = slot_rx.recv() {
            // このクロージャが mfsk-core の同期世界と Tokio の async 世界を
            // つなぐ橋渡しのすべて。`Fn`（`&self` メソッドの
            // `Sender::blocking_send` のみ）かつ `Sync` で、`decode_with` の
            // `&(dyn Fn(&Row<_>) + Sync)` 境界を満たす —— 単一パス戦略が
            // 候補を rayon ワーカスレッドへ分配し、これを並行に呼びうるため必須。
            let on_row = |row: &Row<DecodeResult>| {
                // `row.decoded` は既に所有・Send な `Decoded`（text、freq、dt、snr、
                // protocol）で、このデコーダのコールサイン表で解決済み —— まさに
                // チャネル越しに move したいもの。ここでの `blocking_send` は正しい:
                // これは spawn_blocking スレッド（並列戦略では rayon ワーカでもありうる）
                // 上で走り、Tokio ランタイムワーカでは決してない —— よって async
                // コンテキスト内で `blocking_send` が起こすようなパニックはしない。
                // 送信エラーは受信側が drop されたことを意味し、それ以上することはない。
                let _ = tx.blocking_send(row.decoded.clone());
            };
            // `period` により a7 と平均が連続するスロットを見られる。返される
            // `SlotResult` はストリームしたものの繰り返しなので捨てる。
            let _ = decoder.decode_with(&SlotInput::i16(&slot.audio).period(slot.period), &on_row);
        }
        // タスク復帰時にここで `tx` が drop され、チャネルが閉じて
        // コンシューマのループが終わる。
    });

    (slot_tx, rx)
}
```

> `Decoded`（`mfsk_core::msg::decoded::Decoded`）はクレートの統一・所有
> デコード行 —— `text` / `freq_hz` / `dt_sec` / `snr_db` / `protocol`、
> `Clone` + `Send`、`--features serde` で `Serialize`/`Deserialize`。
> 全ての `Row` がモードを問わず `.decoded` にこれを持つので、同じ橋渡し形が
> どの `Decoder<P>` でも使える。`AnyDecoder` 上のワーカは、コールバックに
> `Decoded` が直接渡される `AnyDecoder::decode_with` を使う。
> [LIBRARY.ja.md](LIBRARY.ja.md) と `docs/notes/DECODED_ROW.md` を参照。

### 5.2 ストリームを消費する

```rust
#[tokio::main]
async fn main() {
    let (slots, mut rx) = spawn_ft8_worker(DecodeParams::for_band((200.0, 3000.0)));

    // あなたのキャプチャパイプラインが供給する: スロット境界に整列した
    // 12 kHz モノラル i16 PCM の 15 秒スロット 1 つ（約 180 000 サンプル）。
    slots.send(Slot { period: 0, audio: load_one_ft8_slot() }).unwrap();
    drop(slots); // もうスロットは無い: ワーカが終わり、チャネルが閉じる

    // 各メッセージは、末尾でまとめてではなくデコーダが受理した瞬間にここへ
    // 届く。ワーカが終わって Sender を drop するとループが抜ける。
    while let Some(msg) = rx.recv().await {
        println!(
            "{:+5.1} dB  {:7.1} Hz  dt={:+.2}s  {}",
            msg.snr_db, msg.freq_hz, msg.dt_sec, msg.text,
        );
        // ...あるいは `msg` を QSO 状態機械、スポットアップローダ、
        // websocket、DB 書き込みへ転送 —— すべてここから `.await` 可能。
    }

    println!("slot decode complete");
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

let (slots, rx) = spawn_ft8_worker(params);
let stream = ReceiverStream::new(rx);
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
  順、一時的重複なし）が欲しければ、単一パスではなく逐次戦略 —— FT8 と FT4 の
  `Depth::Normal` または `Deep`（既定ブロックは `Deep`）、あるいは
  `Ft8Extras::tuning.strategy = Some(Ft8Strategy::SicRounds(3))` —— を使う。橋渡しコー
  ドは同一で、変わるのはデコーダの設定だけ。
- **他プロトコル。** どの `Decoder<P>`（WSPR・JT65・JT9・Q65・FT4・FST4）にも同じ
  `decode_with` がある。同じ `spawn_blocking` の殻の中で、上の FT8 とまったく同様に
  `Decoder::<P>::new(..)` を作ればよい。クロージャは同じ `Sender` を捕捉する。
- **複数チャンネル。** チャンネルごとにワーカ 1 つ、`Decoder` 1 つ、ハッシュ表 1 つ —— IQ レシーバの
  呼び出し側が `ChannelId` ごとに 1 つの `AnyDecoder` を持つのも同じである
  （[LIBRARY.ja.md](LIBRARY.ja.md) §2.7）。
- **キャンセル。** `Receiver` を drop すると、クロージャ内の次の
  `blocking_send` が `Err` を返すので早期に止められる —— ただしデコード
  自体に内部キャンセル点はないため、`spawn_blocking` タスクは何であれ完
  了まで走る。ハードなキャンセルには、`SlotInput::budget(..)` にフラグを読む述語を渡し
  （全モードが候補ごとに 1 回それを呼ぶ。[LIBRARY.ja.md](LIBRARY.ja.md)
  §2.3）、より短い単位でデコードする。

---

## 6. 関連

- [LIBRARY.ja.md](LIBRARY.ja.md) §2.4 —— ライブラリ API リファレンス内の
  ストリーミング節、およびそれが属する `Decoder` / `DecodeParams` / extras の面。
- `Decoder::decode_with` の doc コメント（`mfsk-core/src/decoder/mod.rs`）と、
  エンジンの行コールバックの doc（`DecodeRequest::on_result`、
  `mfsk-core/src/msg/decode_request.rs`）—— 正典であり常に最新の配信契約。
- `mfsk-core/tests/ft8_decode_block_streaming.rs` —— 組込み
  `decode_block_streaming` の完全一致テスト。
- `mfsk-core/tests/ft8_decode_block_streaming_host.rs` —— ホスト
  `fft-rustfft` 版 `decode_block_streaming` の完全一致テスト（issue #243）。
- `mfsk-core/tests/wspr_wsjtx_samples.rs` —— 実信号に対する WSPR の
  行コールバック。
- [BINDINGS.ja.md](BINDINGS.ja.md) —— C 境界越しの同じ考え方:
  `mfsk_decoder_set_on_decode` と `mfsk_stream_*` のリング。同じ移植性の
  理由からコールバックベースになっている。
