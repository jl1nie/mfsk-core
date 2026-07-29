# mfsk-core — ライブラリアーキテクチャ & API リファレンス

> **English:** [LIBRARY.md](LIBRARY.md)

mfsk-core を組み込む開発者向けリファレンス。Rust クレートとして使う場合、
C/C++ から `libmfsk.so` をリンクする場合、Kotlin/Android で JNI 雛形を
使う場合すべて対象。

クイックスタート (バッジ・依存設定・最小コード例) は
[README.md](../../README.md) を参照。本ドキュメントでは設計の *なぜ* と
*どう動くか* に踏み込む。

## 0. はじめに

### 0.1 背景

FT8 / FT4 / FST4 / WSPR / JT9 / JT65 をはじめとする弱信号デジタル
通信モードは K1JT Joe Taylor 氏を中心とするチームにより WSJT-X として
開発されており、同プロジェクトが事実上のリファレンス実装である。
本ライブラリが扱うアルゴリズム (同期相関、LLR 計算、LDPC の BP / OSD
復号、畳み込み符号の Fano 逐次復号、Reed-Solomon 消失復号、各プロトコルの
メッセージ符号化など) はすべて WSJT-X に由来し、各ソースファイルの
docstring では対応する `lib/ft8/`、`lib/ft4/`、`lib/fst4/`、
`lib/wsprd/`、`lib/jt9_*`、`lib/jt65_*` 等のファイル名を明示している。

WSJT-X は C++ と Fortran で書かれたデスクトップアプリケーションとして
長く進化してきた経緯があり、その形で完成度を高めてきた。一方で
ブラウザ PWA として動かしたい、Android 単体アプリに組み込みたい、
あるいは他の Rust / C++ プロジェクトからライブラリとして呼び出したい、
といったデスクトップの外側の用途では、プラットフォームごとに相応の
書き直しが必要になる。

### 0.2 目的

mfsk-core は WSJT-X のアルゴリズムを Rust で再実装し、それを複数の
実行環境 (Native Rust / WebAssembly / Android JNI / C ABI) から
同じ形で利用できる単一クレートとして整理することを目的とする。
本家の C++/Fortran コードとアルゴリズム上等価であることを保ちつつ、
配布形態を広げることに主眼を置いている。

### 0.3 設計方針

プロトコル非依存のアルゴリズム (DSP、同期、LLR、イコライザ、LDPC の
BP / OSD、畳み込み符号の Fano 復号、Reed-Solomon 消失復号、メッセージ
コーデックの共通部) は `engine`、`fec`、`msg` モジュールにまとめ、
各プロトコルは固有定数と採用する FEC / メッセージコーデックを宣言する
比較的小さな ZST (zero-sized type) として表現する。パイプラインは
`DecodeRequest::<P>` (§4) 経由で駆動され、`P: Protocol` をコンパイル時
型パラメータとして受け取り、monomorphize によってプロトコルごとに
特殊化されたコードが生成されるので、抽象化のためにランタイムコストが
増えることはない。

この方針から直接得られるのは次のような性質である。

- 同一のアルゴリズム実装が Native Rust / WASM / Android / C や C++ の
  いずれの環境でも使える。
- 共通経路 (たとえば LDPC BP) に加えた改善は、その経路を使うすべての
  プロトコルに自動的に波及する。
- 新しいプロトコルを追加する際、変更範囲は当該プロトコル固有の部分に
  閉じ込めやすい (具体的な指針は §2 にまとめた)。
- C ABI (`mfsk-ffi`) の分岐は `match protocol_id` 一段のみで、その先は
  既に特殊化されたコードに入る。

### 0.4 現在対応しているプロトコル

| プロトコル   | スロット | FEC                        | メッセージ | 同期              | 出典           |
|--------------|---------|----------------------------|-----------|-------------------|----------------|
| FT8          | 15 s    | LDPC(174, 91) + CRC-14     | 77 bit    | 3×Costas-7        | `lib/ft8/`     |
| FT4          | 7.5 s   | LDPC(174, 91) + CRC-14     | 77 bit    | 4×Costas-4        | `lib/ft4/`     |
| FST4-60A     | 60 s    | LDPC(240, 101) + CRC-24    | 77 bit    | 5×Costas-8        | `lib/fst4/`    |
| WSPR         | 120 s   | 畳み込み r=½ K=32 + Fano   | 50 bit    | シンボル毎 LSB    | `lib/wsprd/`   |
| JT9          | 60 s    | 畳み込み r=½ K=32 + Fano   | 72 bit    | 16 分散位置       | `lib/jt9_decode.f90`, `lib/conv232.f90` |
| JT65         | 60 s    | Reed-Solomon(63, 12) GF(2⁶) | 72 bit   | 63 分散位置 (擬似乱数) | `lib/jt65_decode.f90`, `lib/wrapkarn.c` |
| Q65-15A      | 15 s    | QRA(15, 65) GF(2⁶) + CRC-12 | 77 bit   | 22 分散位置       | `lib/qra/q65/` |
| Q65-30A      | 30 s    | (同 QRA codec)              | 77 bit   | (同 sync 配置)    | `lib/qra/q65/` |
| Q65-60A‥E    | 60 s    | (同 QRA codec)              | 77 bit    | (同 sync 配置)    | `lib/qra/q65/` |
| Q65-120D‥E   | 120 s   | (同 QRA codec)              | 77 bit    | (同 sync 配置)    | `lib/qra/q65/` |
| Q65-300A     | 300 s   | (同 QRA codec)              | 77 bit    | (同 sync 配置)    | `lib/qra/q65/` |

Q65 は 10 個の sub-mode 構成で実装している — 地上波向け Q65-15A/30A
2 つ、EME 帯向けの 60 秒スロット 5 種 (Q65-60A〜Q65-60E、トーン間隔
倍率 ×1, ×2, ×4, ×8, ×16)、そして長周期スキャッター系 3 種
(Q65-120D 10GHz レインスキャッター/対流圏散乱、Q65-120E 6m
イオノスキャッター、Q65-300A 光散乱 — 実装済み中最深の約-34dB
AWGN閾値)。これらは FEC、メッセージコーデック、同期配置、共通の
trait 実装ブロックを共有しており、NSPS とトーン間隔のみが
sub-mode ごとに異なる。

FST4 も同様に 5 個の T/R 周期 sub-mode (FST4-15 / FST4-30 /
FST4-60A / FST4-120 / FST4-300) を実装済みで、LDPC(240, 101)・
メッセージコーデック・5×Costas-8 同期・GFSK 整形 (BT=2.0) を共有し、
`NSPS` / `NDOWN` / `SYMBOL_DT` / `TONE_SPACING_HZ` のみが異なる
(FST4-15 のみ `TX_START_OFFSET_S` も異なる)。FST4-900 / FST4-1800 は
意図的に未実装 (現時点で需要なし)。FST4W (WSPR 型の片方向 50 bit
ビーコン、LDPC(240, 74)、周期 120/300/900/1800 s) は別のメッセージ
形式であり本書の対象外 — issue #23 参照。

**MSK144** はこの表に意図的に含めていない — `Protocol` を実装する
ZST が存在しないため。§0.6 を参照。

### 0.5 設計が機能していることの確認 — WSPR と Q65 という 2 つのストレステスト

FT8 / FT4 / FST4 はいずれも LDPC + 77 bit メッセージ + ブロック Costas
同期という共通点が多く、共通化の恩恵が大きい一方、共通点の多さが
抽象化の良し悪しを測る材料にはなりにくい。**WSPR** と **Q65** は
それぞれ独立した軸で trait 面を試す題材であり、いずれも FT 系の
コード経路には手を入れず吸収できた。

#### WSPR — FT 系から独立した 3 軸

1. **FEC の系統**: LDPC ではなく畳み込み符号 (r=½、拘束長 32) と
   Fano 逐次復号。`mfsk_core::fec::conv::ConvFano` として追加した。
2. **メッセージ長**: 77 bit ではなく 50 bit。Type 1 / 2 / 3 の
   メッセージ形式を `mfsk_core::msg::wspr::Wspr50Message` で実装。
3. **同期構造**: ブロック Costas ではなく、チャネルシンボル
   すべての LSB に 162 bit の sync vector を埋め込む形式。これを
   表現するために `FrameLayout::SYNC_MODE` に `Interleaved` バリアントを
   追加した。

#### Q65 — 第 3 の FEC 系統 + 5 戦略 × 10 sub-mode

Q65 は WSPR では試されなかった軸で trait 面を試す材料になった:

1. **さらに別の FEC 系統**: GF(64) 上の Q-ary repeat-accumulate 符号。
   非二進 Walsh-Hadamard メッセージにより確率ベクトル領域で belief
   propagation を実行する。`mfsk_core::fec::qra` (汎用 QRA codec) と
   `mfsk_core::fec::qra15_65_64` (Q65 固有のコードインスタンス) として
   追加した。
2. **65 トーン FSK + 同期専用トーン**: `NTONES = 65` だが
   `BITS_PER_SYMBOL = 6`。データアルファベットは GF(64) で、
   tone 0 は同期専用。これが `GRAY_MAP` 長に関する trait doc の
   不整合を露呈した — 全プロトコルで `== NTONES` も
   `== 2^BITS_PER_SYMBOL` も一様には成立しないため、契約を
   `[2^BITS_PER_SYMBOL, NTONES]` に緩めた。
3. **10 sub-mode を 1 個の impl ブロックで共有**: 地上波向け
   Q65-15A/30A、EME 用の Q65-60A〜E (6 m / 70 cm / 23 cm /
   5.7 GHz / 10 GHz / 24 GHz+)、長周期スキャッター用の
   Q65-120D/120E/300A。NSPS とトーン間隔のみが異なる。
   `q65_submode!` マクロが各 sub-mode の ZST と trait 実装を 1 行で
   生成するため、sub-mode ごとのコード重複は無い。
4. **5 つの並列なデコード戦略**: 同一 FEC フレームに対し正当に
   選び得る受信経路が複数通り存在するのは Q65 が初めて — plain AWGN
   BP、AP-hint BP、fast-fading metric、AP-list テンプレート照合、
   multi-period EMA averaging。§3 で詳述する。各戦略は
   `DecodeRequest`/`SniperRequest`/`MultiPeriodRequest` 上の builder
   メソッドの組み合わせとして表現され、sub-mode ZST に対して
   generic であり、内部の FEC とメッセージコーデックは共有する。

WSPR が「**FEC 系統 + メッセージ長 + 同期形式**」を独立に差し替え
られることを示したのに対し、Q65 は「第 3 の FEC 系統 + 10 sub-mode
+ 5 戦略」がすべて `Protocol` super-trait の中に収まり、専用の
配管を増やすことなく実現できることを示している。§3 でデコード戦略を
詳述し、§7 で `PROTOCOLS` レジストリ (実装される 24 ZST —
WSJT ファミリ 20 + uvpacket 4、§10.1 — すべての列挙) と
`tests/protocol_invariants.rs` の汎用検査機構を扱う。

### 0.6 MSK144 — 抽象化を使わないプロトコル

§0.4 の全プロトコルは、互いの違いの下に一つの共通性質を持つ:
coarse-sync → LLR → FEC という pipeline が、既知のおおよその
オフセットに 0..N 個のフレームが並ぶ、1 つの固定長スロットの上で
動く、という前提である。**MSK144** (issue #25) はこの前提の両方を
同時に破る。無理に型に押し込む代わりに、`msk144::decode` は
それ自身が独立したトップレベルドライバであり、`engine::pipeline` /
`ModulationParams` / `FrameLayout` のいずれにも触れない。
`Protocol` を実装する ZST も存在しない。これは WSPR や Q65 (§0.5)
よりもさらに一歩踏み込んだ話で、あちらは実デコードロジックが
共有 pipeline を迂回する場合でも trait 面自体は採用している ——
MSK144 は trait 面そのものから外れている。

1. **M 元 FSK ではない**: MSK144 は連続位相の二値 MSK で、
   offset-QPSK として送信される — bit は I/Q レールに割り当てられ、
   それぞれ半正弦パルスで整形される。`ModulationParams` の
   トーン番号・Gray map・GFSK 整形というモデルには、有用な
   写像先がない。変調器と整合フィルタ復調器は `engine::dsp::msk` に
   trait 実装ではなく単なる関数として存在する。
2. **静的スロットではない**: 他の全プロトコルはフレームが固定長
   スロットバッファ内の既知のおおよそのオフセットに位置すると
   仮定している。MSK144 は 864 サンプル (72 ms) のフレームを
   15 秒 (または 30 秒) の T/R 期間全体にわたって連続的に繰り返す
   ——実際の送信は同一内容を期間全体でバックトゥバックにループし続け、
   受信側は電離飛跡のピングがどこに来るか分からないまま走査する
   必要がある。`msk144::spd::detect_burst_candidates` (二乗信号の
   2 トーン スペクトル走査) と `msk144::sync::msk144_sync`
   (CFO/シンボルタイミングの同時整合フィルタ相関) がその走査を担う
   ——WSJT-X の `msk144spd.f90`/`msk144sync.f90` からの移植であり、
   `engine::sync::coarse_sync` からではない。

一方、ライブラリの他部分と共有しているものもある: 77 bit の
`pack77`/`unpack77` メッセージペイロード (`msg::wsjt77`、完全に
無変更 — MSK144 の実際の WSJT-X エンコーダは FT8/FT4/FST4 と
同じ関数を呼んでいる) と、汎用 LDPC belief-propagation/OSD
エンジン (`fec::ldpc::bp`/`osd`) を LDPC(128, 90) + CRC-13
(`fec::ldpc_128_90`) 用に 4 番目の `LdpcParams` 実装として
パラメータ化したもの ——これは FST4 用に `Ldpc240_101Params` を
追加したときと全く同じ、その trait 自身が文書化している拡張手順
どおりに追加した。つまり FEC・メッセージ層はこれまでの追加と
同じ形で拡張されており、変調・フレームタイミング層だけが対象外
になっている理由は、`ModulationParams`/`FrameLayout` に
過渡バースト・非 FSK なプロトコルが差し込める場所がそもそも
存在しないからである。

WSJT-X `samples/MSK144/*.wav` の 2 ファイルに対するゴールデン
WAV recall は 3/3 (`tests/msk144_wsjtx_samples.rs`) —
アーキテクチャが分岐していても recall は犠牲になっていない。

## 1. モジュール構成

```text
mfsk_core
├── engine/           Protocol trait 群、DSP、sync、LLR、equaliser、pipeline
│   ├── protocol.rs     ModulationParams / FrameLayout / Protocol / FecCodec / MessageCodec
│   ├── dsp/            resample · downsample · gfsk · subtract · msk · analytic
│   ├── sync.rs         coarse_sync / refine_candidate
│   ├── llr.rs          symbol_spectra / compute_llr / sync_quality
│   ├── equalize.rs     equalize_local (トーン毎 Wiener)
│   └── pipeline.rs     decode_frame / decode_frame_subtract / process_candidate_basic
│                       (pub(crate) 内部実装 — §4 参照。呼び出しは
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
│   │                     公開デコードエントリポイント (§4。0.8.0 以前の
│   │                     decode_frame*/decode_sniper* 系を置換)
│   ├── wsjt77.rs       77 bit WSJT メッセージ (pack / unpack) — FT8, FT4, FST4, Q65, MSK144
│   ├── wspr.rs         50 bit WSPR Types 1 / 2 / 3
│   ├── jt72.rs         72 bit JT メッセージ — JT9, JT65
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
│   ├── rx.rs           5 つのデコード戦略 (AWGN / AP-hint / fast-fading / AP-list / multi-period)、§3 参照
│   ├── ap_list.rs      standard_qso_codewords — full AP-list 候補生成
│   ├── tx.rs           65-FSK 合成器 (sub-mode 対応)
│   ├── search.rs       22 シンボル Costas-block coarse 検索
│   └── sync_pattern.rs Q65 分散同期配置
├── msk144/           MSK144 — Protocol 実装なし、独立トップレベルドライバ (§0.6)
│   ├── tx.rs           codeword -> 864 サンプル複素 OQPSK フレーム
│   ├── sync.rs         (CFO, タイミング) 同時整合フィルタ探索
│   ├── spd.rs          バースト候補検出 + short-ping デコードループ
│   ├── frame_decode.rs sync ゲート -> LLR -> LDPC -> メッセージ
│   └── decode.rs       decode_slot(): スライディングウィンドウ型トップレベルドライバ
└── uvpacket/         非 WSJT 応用例 — 4 sub-mode ZST、独自 tx/rx (§10.1)
    ├── protocol.rs     ModulationParams/FrameLayout 実装 (一部は装飾的、§10.1 参照)
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

ワークスペースの兄弟クレート `mfsk-ffi` が同じクレートの上に C ABI
共有ライブラリ (`libmfsk.{so,a,dylib}` + `mfsk.h`) を構築する。

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
(§7.2) が同じ式で成り立つ。

## 2. Protocol トレイト階層

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
(§4、§6.2) または WSPR 専用の
`wspr::decode::decode_scan_default(...)` のように型引数でプロトコルを
指定するだけで済み、合成の結果として選ばれた FEC・メッセージ
コーデック・同期方式が自動的に使われる。

### Monomorphization とゼロコスト抽象

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

### 新しいプロトコルを追加する場合

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

## 3. デコード戦略 (Q65 ケーススタディ)

このライブラリの大半のプロトコルはデコード経路が 1 通りしかない
(FT 系列の `DecodeRequest::<P>` (§4「パブリックデコードエントリ
ポイント」参照)、WSPR の `wspr::decode::decode_scan_default` など)。
Q65 は同一 FEC フレームに対して**正当に選び得る受信経路が複数通り**
ある最初の実装プロトコルで、各経路が異なる種類のチャネル劣化に
対して計算コストとデコード閾値のトレードオフを実現している。

issue #204 以降、これらは `mfsk_core::q65::decode_request` 内の
3 つの generic builder として公開されている — `DecodeRequest<P>`
(wide-band scan)、`SniperRequest<P>` (既知の
`(start_sample, base_freq_hz)` を指定する単一候補探索、
`DecodeRequest::sniper` または `SniperRequest::new` 直接呼び出しで
構築)、`MultiPeriodRequest<P>` (複数スロット平均化デコード) —
`msg::decode_request` の FT8/FT4/FST4 向け builder (§4) と同じ形を
とり、10 sub-mode ZST すべてに実装された sealed `Q65SubMode`
マーカーに対して generic である。本節がかつて直接参照していた
`q65::rx` の各関数 (`decode_at_for`, `decode_scan_for`, …) は
`pub(crate)` になっている — 公開エントリポイントは builder 側。
`.ap_hint()` / `.ap_list()` / `.fading()` は (`SupportsWideBandAp`
のような capability-gated marker trait ではなく) 素朴な inherent
method である。Q65 は全 sub-mode が全 capability を一様にサポート
するため。

先に一点注意しておく: capability を何も設定しない
`SniperRequest::decode()` (内部で `decode_at_for` を呼ぶ) は文字通り
plain Bessel-I0 metric — point-decode のみの基準経路である。しかし
capability を何も設定しない `DecodeRequest::decode()` (内部で
`decode_scan_for` を呼ぶ) — 実際のほぼ全呼び出し元が使う形 — は、
各 coarse-search 候補を、`q65_loops.f90` / `q65_dec_q012` から移植した
WSJT-X 忠実な `(Δf, Δt, b90)` grid search を `FadingModel::Lorentzian`
の fast-fading metric で常に実行する経路に通す。これは WSJT-X 自身の
自動デコーダの挙動と一致する — WSJT-X もデフォルト scan で
plain-AWGN-only な Bessel パスを走らせることは無い。つまり「AWGN」と
「fast-fading」はチャネル種別で選ぶ 2 つの独立した front end という
より、(sniper 用 / scan 用という) 2 つの異なるエントリポイント群と
捉える方が正確で、scan 経路はデフォルトで既にある程度の fading を
仮定しており、`.fading()` は呼び出し側が明示的に `(b90_ts, model)`
を指定したい場合のために存在する。

| 状況                                                  | 戦略                                              | Builder 呼び出し                                                     | 閾値ゲイン       |
|-------------------------------------------------------|-----------------------------------------------------|-------------------------------------------------------------------------|------------------|
| 単一候補既知、メッセージ未知 (point-decode のみ)      | AWGN Bessel + BP                                    | `SniperRequest::<P>::new(...).decode()`                                  | 基準             |
| デフォルト scan — チャネル特性 / メッセージ未知       | `(Δf,Δt,b90)` grid search + Lorentzian fading BP   | `DecodeRequest::<P>::new(...).decode()`                                  | WSJT-X 忠実デフォルト |
| コールサイン or レポート既知、地上波チャネル          | AP-hint BP                                          | いずれかの builder に `.ap_hint(&ap)`                                    | ~2 dB            |
| ドップラー拡散チャネル、model 明示指定 (≥10 Hz、microwave EME) | Fast-fading metric + BP、呼び出し側が `(b90_ts, FadingModel)` を指定 | いずれかの builder に `.fading(model, b90_ts)` | 拡散時 5–8 dB    |
| コールペア既知、QSO 文脈無し、地上波                  | AP-list テンプレート照合                            | いずれかの builder に `.ap_list(&candidates)`                            | ~3 dB |
| 複数 T/R 期間にまたがる弱信号 / ionoscatter           | Multi-period EMA averaging (3 段カスケード)         | `MultiPeriodRequest::<P>::new(...).decode()`                             | 単一期間のどの戦略でも復号できない信号を回収 |

**AWGN Bessel + BP** (`SniperRequest` に capability 未設定) は教科書
通りの単発経路: シンボル毎 FFT エネルギーを Bessel-I0 metric で
確率ベクトル化し、QRA 符号上で非二進 belief propagation を実行する。
加法ガウスノイズに近いチャネルで安全に動く — ただしこれは
*point-decode* の形のみであり、scan ファミリがこの front end に
留まらない理由は上の注意点を参照。

**AP-hint BP** (`.ap_hint(&ap)`) は BP 開始前に既知の情報ビット位置で
intrinsic 確率ベクトルをクランプする。正しい hint は BP の収束点を
真値側に寄せ、誤った hint は誤デコードよりも収束失敗を引き起こす
傾向がある (CRC が残りを捕捉する)。`mfsk_core::msg::ApHint` の
builder (`with_call1`, `with_call2`, `with_grid`, `with_report`) で
構築する。

**Fast-fading metric** (`.fading(model, b90_ts)`) は Bessel front end
を、呼び出し側が選ぶ `FadingModel::Gaussian` / `FadingModel::Lorentzian`
形状に対してキャリブレーションされた拡散対応 metric に置き換える。
トーンが 10–60 Hz に拡散する microwave EME で必須: リファレンス
録音 `samples/Q65/60D_EME_10GHz/` (10 GHz EME) はこの経路で復号
できるが、plain Bessel front end では 0 件。`b90_ts` は拡散帯域 ×
シンボル周期 (代表値: 0.05 = ほぼ AWGN、1.0 = 中程度、5.0+ = 強拡散)。

**AP-list テンプレート照合** (`.ap_list(&candidates)`) は BP を
**走らせない**。代わりに生成器
`q65::ap_list::standard_qso_codewords(my_call, his_call, his_grid)`
が WSJT-X "full AP list" — 既知のコールペアが正当に生成し得る
標準交換 206 件 (`MYCALL HISCALL`、`MYCALL HISCALL RRR/RR73/73`、
`CQ HISCALL grid`、加えて 200 件の SNR ladder) — を事前に符号化する。
デコーダは soft observation との対数尤度がリスト規模調整済み
閾値を超える候補を選ぶか、そうでなければ `None` を返す。
SNR −25 dB (公開閾値より 1 dB 低い) のスイープ試験では plain BP が
0/6 失敗するのに対し AP-list は 6/6 復号する。`.ap_list()` と
`.fading()` は内部エンジン上で排他 — `.decode()` の優先順位は
ap_list > fading (+ap_hint) > ap_hint > plain。

**Multi-period EMA averaging** (`MultiPeriodRequest`) は
`q65_decode.f90` の WSJT-X `iavg=1`/`iavg=2` 平均化デコードを
移植したもの — 上記のどの単一期間戦略でも復号できない
ionoscatter / 弱 EME 信号を救う最後の戦略である。単一のオーディオ
バッファではなく `&[&[f32]]` (T/R スロットごとに 1 バッファ) を
受け取る。連続する T/R 期間にわたって per-slot spectrogram の
指数移動平均 (時定数 `min(navg, 4)`) を維持し、各スロットで
平均化済みエネルギーに対して 3 段のデコードラダーを試す:
(1) `.ap_list()` が設定されていれば AP-list、(2) `b90·Ts ∈ {3, 8, 15}`
× `{Gaussian, Lorentzian}` を掃く fast-fading BP、(3) 最終手段としての
plain Bessel BP (AWGN fallback、この builder には別途
`.fading()`/`.ap_hint()` は無く、このラダーが常に走る)。スロット
あたり最大 1 件を返し、`(message, ±4 Hz freq)` で重複排除する。
`mfsk-ffi` にはまだ未公開 — 現時点では Rust API のみ。

C ABI には上記のうち 4 戦略が `mfsk_q65_decode`、
`mfsk_q65_decode_with_ap`、`mfsk_q65_decode_fading`、
`mfsk_q65_decode_with_ap_list` として 1 対 1 で公開されており、
いずれも `MfskQ65SubMode` 引数を取って 10 sub-mode のいずれにも
アクセス可能。`mfsk_q65_decode_fading` はさらに `MfskQ65FadingModel`
(`Gaussian` / `Lorentzian`) 引数を取る (§8)。Multi-period averaging は
まだ C ABI に含まれていない。

## 4. 共有プリミティブ (`engine`)

### 受信パイプライン概観

任意の wired プロトコルにおける受信フロー全体 — 生オーディオ
サンプルから復号メッセージ文字列まで — は、以下に挙げる `engine`
サブモジュール内のフリー関数群を `P: Protocol` でパラメタライズ
して鎖状に呼び出した形になっている:

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

`Demodulator` や `Receiver` という trait はない。受信経路は
`engine::sync` / `engine::llr` / `engine::equalize` / `engine::pipeline` の
フリー関数群として実現され、それぞれ `P: Protocol` で generic に
なっている。Monomorphization により、手書きのプロトコル別デコーダ
と同等のコードが生成されつつ、各プロトコルに n-method な受信
trait の実装を強制しないで済む。Soft demap は
`engine::llr::compute_llr<P>` で、`Protocol::demap()` メソッドではなく
フリー関数になっているのは、スペクトル抽出 (`symbol_spectra`)・
WSJT 式 4 バリアント LLR (a/b/c/d)・equalizer がデータとして
組み合わさるためで、trait 合成では表現が冗長になる。Sync /
equalize / pipeline ドライバも同じスタイルで、プロトコル型を
パラメータとして受け取り `P` の関連定数 (`NTONES`、`NSPS`、
`SYNC_MODE` など) を直接参照する。

### パブリックデコードエントリポイント: `DecodeRequest` / `SniperRequest`

上図の engine 関数群 (`coarse_sync`、`decode_frame`、
`process_candidate_basic`、…) は issue #191/#203 以降内部実装
(`pub(crate)`) である。アプリケーションはこれらを
`mfsk_core::msg::decode_request` にある 2 つの generic builder 経由で
駆動する。実装対象は `Ft8`、`Ft4`、および全 FST4 sub-mode
(`FrameDecodable` マーカー trait。Q65/WSPR/JT65/JT9/uvpacket は
それぞれ独自のエントリポイントを維持、§6.3/§6.5):

* **`DecodeRequest<P>`** — `freq_min..freq_max` に対する wide-band
  探索。`DecodeRequest::<P>::new(audio, freq_min, freq_max, sync_min,
  max_cand)` の後、`.osd(bool)` (デフォルト `true`。BP staircase が
  失敗した際のOSDフォールバックの有無を切り替える — hostのdecodeでは
  `LlrEffort`は常に`Full`固定。詳細は`.osd`自体のdoc comment参照)、
  `.strictness(...)`、`.eq_mode(...)`、
  `.known(...)` (前パスで既に復号済みのメッセージをスキップ/減算)、
  `.fft_cache(...)` (直前呼び出しの forward FFT を再利用)、
  プロトコルが `SupportsWideBandAp` を実装していれば `.ap_hint(...)`
  (FT8 のみ)、さらにプロトコルが対応していれば
  `.sic_rounds(n)` / `.sic_early()` のいずれかで successive-
  interference-cancellation 戦略を選択 (`SupportsSicRounds`:
  FT8+FT4、`n`は1..=3にクランプ。`SupportsSicEarly`: FT8のみ、
  チェックポイント構造は3固定) をチェーンする。`.decode()` で
  `DecodeOutcome<P>` (`.results: Vec<P::DecodeResult>`、後続呼び出し用の
  `.fft_cache` も含む) を得る。
* **`SniperRequest<P>`** — narrow-band、単一ターゲット探索。
  `DecodeRequest::<P>::sniper(audio, target_freq_hz, max_cand)` または
  `SniperRequest::<P>::new(...)` を直接呼ぶ。`.osd(bool)`、
  `.strictness(...)`、`.eq_mode(...)`、プロトコルのメッセージ
  コーデックが `WsjtApCompatible` を実装していれば `.ap_hint(...)`
  (SIC バリアントは無し — sniper モードは元々単一候補)。
  `.decode()` は同じ `DecodeOutcome<P>` 形状を返す。

これにより FT8 の `decode_frame*`/`decode_frame_subtract*`/
`decode_sniper*` 系 (公開関数 15 個) と FT4/FST4 独自の同種の
suffix 展開を置き換えた — §6.2/§6.4 に実例がある。

`DecodeDepth` (`llr_effort`/`osd`) という型自体は残っている ——
`decode_block`/`decode_block_into` (embedded/host共通のプレーン関数
FT8 API、§10) が位置引数として受け取る型。ただし `DecodeRequest`/
`SniperRequest` はもう直接は公開しない — hostで`LlrEffort::Minimal`
を必要とした呼び出しは一つも無かった (`decode_block_into`のESP32
電力予算のためだけに存在する variant) ので、builder側は`Full`固定に
し、`osd`トグルだけを公開する。

**`WsjtxDepth`** (`mfsk_core::ft8::decode::WsjtxDepth`、
`DecodeRequest::<Ft8>::wsjtx_depth(...)`) は `.osd(...)` +
`.sic_rounds(n)`/`.sic_early()` + `.ap_hint()` を、実際のWSJT-Xの
`jt9 -d 1/2/3` CLIフラグに対応する3段階の named tier (`D1`/`D2`/`D3`)
にまとめたもの——実際の`jt9`ビルドとのベンチマーク比較用。tierと
builderメソッドの正確な対応、既知の限界（OSD強度がjt9と厳密には
一致しない点）は型自身のdoc commentを参照。

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
> (§4) は内部で `decode_block::coarse_sync` を経由する。詳細は §10。

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

`DecodeStrictness` (`Strict`/`Normal`/`Deep`) は 4 つのメソッドを持つ
が、プロトコルごとに「実際に効くか」が異なる — `.strictness(...)`
が何かを変えるかどうかは呼び出し先次第なので注意:

* `osd_score_min()` / `osd_max_errors()` — OSD 実行前の coarse-sync
  スコアしきい値と、OSD 後の硬判定エラー上限ゲート (`osd_depth` 別)。
  **実質 FT4 専用。** `osd_score_min` は FST4・FT4 両方でバイパス済み
  (`engine/pipeline.rs` の `bypass_osd_score_min` — FST4 は WSJT-X 自
  身の FST4 受理判定 `fst4_decode.f90:570`: `nharderrors >= 0 &&
  unpk77_success` に合わせて CRC-24 のみを信頼、スコア事前フィルタ
  なし。FT4 も同じ症状に独立して行き当たり同じバイパスを適用)。
  `osd_max_errors` は FST4 バイパスのままだが FT4 では**生きている**
  — 実際の `ft4sim` AWGN/CCIR sweep で再較正済み (issue #72、
  2026-07-18)。もはや FT8 較正値のプレースホルダーではない。**名前
  に反して、FT8 はこの 2 メソッドをこれまで一度も呼んでいなかった**
  — FT8 自身の OSD dispatch は hardcoded 定数を使っていた (下記
  `ft8_nharderrors_max` 参照)。旧版の本ドキュメントが「FT8 較正値」
  と誤って説明していた箇所を訂正。
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
  `mfsk-ffi-ft8` / `embedded-shared::dual_core` との API 安定性のため
  別名を維持。0.8.0 以前は呼出側提供の BASIS scratch も受け取って
  いたが、Goertzel fill path 移行で scratch が不要になったため削除
  (issue #162)
* `coarse_sync` / `coarse_sync_with_allsum` — FT8 sync grid 本体
  (0.6.0 で公開 API 昇格)
* `fill_symbol_spectra` / `fill_symbol_spectra_goertzel` — 音声から
  直接シンボル毎 FFT を抽出 (旧コードの cd0 +
  `engine::llr::symbol_spectra` 二段経路を置換)

## 5. Feature flags

| フィーチャー      | デフォルト | 効果                                                         |
|-------------------|------------|--------------------------------------------------------------|
| `ft8`             | on         | FT8 ZST、decode、wave_gen                                   |
| `ft4`             | on         | FT4 ZST、decode                                             |
| `fst4`            | off        | FST4-15/30/60A/120/300 ZST、decode                          |
| `wspr`            | off        | WSPR ZST、decode、synth、spectrogram search                 |
| `jt9`             | off        | JT9 ZST、decode                                             |
| `jt65`            | off        | JT65 ZST、decode (+ 消失対応 RS)                            |
| `q65`             | off        | Q65-15A/30A + Q65-60A‥E + Q65-120D/E/300A ZST、5 デコード戦略 (§3)、synth |
| `msk144`          | off        | MSK144 — `Protocol` ZST 無し、独立トップレベルドライバ (§0.6) |
| `packet-bytes`    | off        | `PacketBytesMessage` — バイトペイロード例示 `MessageCodec`    |
| `uvpacket`        | off        | uvpacket — 非 WSJT 応用例、4 sub-mode ZST (§10.1)、`fst4` を要求 |
| `full`            | off        | 上記全プロトコル系フィーチャーの集約                          |
| `parallel`        | on         | パイプラインで rayon `par_iter` (WASM は無効化)              |

## 6. Rust から利用する

### 6.1 依存関係

```toml
[dependencies]
mfsk-core = { version = "0.7", features = ["ft8", "ft4", "wspr"] }
```

必要なプロトコルのフィーチャーだけ有効にすれば十分。以下では複数を
有効にした例を示す。

### 6.2 FT8 デコード — 最小例

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

### 6.3 WSPR — 別系統の復調 (abstraction と両立する形で)

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
            println!("{:7.2} Hz  {} {} {}dBm", d.freq_hz, callsign, grid, power_dbm);
        }
        WsprMessage::Type2 { callsign, power_dbm } => {
            println!("{:7.2} Hz  {} {}dBm", d.freq_hz, callsign, power_dbm);
        }
        WsprMessage::Type3 { callsign_hash, grid6, power_dbm } => {
            // ハッシュは過去の Type-1 受信から解決できる場合がある
            println!("{:7.2} Hz  <#{:05x}> {} {}dBm",
                     d.freq_hz, callsign_hash, grid6, power_dbm);
        }
    }
}
# }
```

`decode_scan_default` が粗同期 (周波数×時刻探索) を込みでスロット全体を
スキャンする。周波数・開始サンプルが既知の場合は
`wspr::decode::decode_at(samples, rate, start_sample, freq_hz)` を
直接呼べば粗同期を省略できる。

### 6.4 Sniper モード + AP hint

既知のターゲット周波数を中心に ±250 Hz に絞って AP hint を与えると、
より弱い信号まで引き出せる — 500 Hz ハードウェア BPF の後段や、
既知の 1 局を狙う場合の利用を想定:

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

`EqMode` は 0.7.0 以降 `Off` / `Local` の 2 種類のみ — 従来の
`Adaptive` (EQ あり→無しの 2 パス試行) は、その効果
(-18 dB で追加復号 ~1/20) が候補あたり 2 倍のコストに見合わなく
なったため同リリースで廃止された (issue #73)。旧 2 パス挙動が
欲しい呼び出し元は `Local` → `Off` の順に明示的に 2 回呼べばよい。

`SniperRequest::<Ft4>` も同じ形で使える (`FrameDecodable` は両方に
実装されている)。

### 6.5 JT9 / JT65

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
    println!("{:7.2} Hz  {}", d.freq_hz, d.message);
}
# }
```

JT65 はさらに `decode_at_with_erasures` を提供しており、
低 SNR 環境で RS 消失復号が通常デコーダでは落とすフレームを
回復できる。

## 7. ランタイム registry と trait 面の検証

ライブラリを自己記述的・自己検証的に保つための構造的インフラが
2 つあり、いずれもプロトコル毎の保守を要しない。

### 7.1 `PROTOCOLS` レジストリ

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

### 7.2 汎用 trait 面検査

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

## 8. C / C++ — `mfsk-ffi`

### 生成物

`cargo build -p mfsk-ffi --release` で:

* `target/release/libmfsk.so`  (Linux / Android 共有オブジェクト)
* `target/release/libmfsk.a`   (static、組み込み向け)
* `mfsk-ffi/include/mfsk.h`    (cbindgen 生成、コミット済)

### API

正確な宣言は `mfsk-ffi/include/mfsk.h` 参照。サマリ:

```c
enum MfskProtocol {
    MFSK_PROTOCOL_FT8     = 0,
    MFSK_PROTOCOL_FT4     = 1,
    MFSK_PROTOCOL_WSPR    = 2,
    MFSK_PROTOCOL_JT9     = 3,
    MFSK_PROTOCOL_JT65    = 4,
    MFSK_PROTOCOL_FST4S60 = 5,  // FFI で公開されているのは FST4-60A のみ
                                // (他の 4 sub-mode: 15/30/120/300 は
                                // Rust API 限定、§0.4)
    MFSK_PROTOCOL_Q65A30  = 6,
};

// Q65 sub-mode は family enum とは別の専用 enum で識別 (全 10 sub-mode):
enum MfskQ65SubMode { A30=0, A60=1, B60=2, C60=3, D60=4, E60=5,
                       A15=6, D120=7, E120=8, A300=9 };
// mfsk_q65_decode_fading の model 引数 (§3):
enum MfskQ65FadingModel { Gaussian=0, Lorentzian=1 };

uint32_t          mfsk_version(void);           // major<<16 | minor<<8 | patch
MfskDecoder*      mfsk_decoder_new(MfskProtocol protocol);
void              mfsk_decoder_free(MfskDecoder* dec);

// `options` は NULL 可。NULL の場合はプロトコルごとの既定の探索範囲
// / しきい値 / depth を使う。mfsk_decode_options_new(...) のハンドル
// を渡すと一律に上書きできる。
MfskDecodeOptions* mfsk_decode_options_new(float freq_min_hz, float freq_max_hz,
                                  float sync_min, int max_cand,
                                  MfskDecodeDepth depth);
void              mfsk_decode_options_free(MfskDecodeOptions* opts);

MfskStatus        mfsk_decode_i16(MfskDecoder*, const int16_t* samples,
                                  size_t n, uint32_t sample_rate,
                                  const MfskDecodeOptions* options,
                                  MfskResultList* out);
MfskStatus        mfsk_decode_f32(MfskDecoder*, const float*,  size_t,
                                  uint32_t, const MfskDecodeOptions*,
                                  MfskResultList* out);

MfskStatus        mfsk_encode_ft8(const char* call1, const char* call2,
                                  const char* report, float freq_hz,
                                  MfskSamples* out);
MfskStatus        mfsk_encode_ft4(...);      // 同形
MfskStatus        mfsk_encode_fst4s60(...);  // 同形
MfskStatus        mfsk_encode_wspr(const char* call, const char* grid,
                                   int32_t power_dbm, float freq_hz,
                                   MfskSamples* out);
MfskStatus        mfsk_encode_jt9(...);      // ft8 と同形
MfskStatus        mfsk_encode_jt65(...);     // ft8 と同形
MfskStatus        mfsk_encode_q65(MfskQ65SubMode submode,
                                  const char* call1, const char* call2,
                                  const char* report, float freq_hz,
                                  MfskSamples* out);

// Q65 専用 4 戦略 (sub-mode 引数で 10 sub-mode のいずれにも適用。
// multi-period averaging はまだ C ABI 未公開、§3 参照):
MfskStatus        mfsk_q65_decode(MfskQ65SubMode, ...);              // AWGN
MfskStatus        mfsk_q65_decode_with_ap(MfskQ65SubMode, ...,
                                  const char* ap_call1, ap_call2,
                                  ap_grid, ap_report, ...);          // AP-hint BP
MfskStatus        mfsk_q65_decode_fading(MfskQ65SubMode, ...,
                                  float b90_ts,
                                  MfskQ65FadingModel, ...);          // fast-fading
MfskStatus        mfsk_q65_decode_with_ap_list(MfskQ65SubMode, ...,
                                  const char* my_call,
                                  const char* his_call,
                                  const char* his_grid, ...);        // AP-list

void              mfsk_result_list_free(MfskResultList* list);
void              mfsk_samples_free(MfskSamples* s);
const char*       mfsk_last_error(void);
```

`MfskResultList` は呼び出し元が確保するストレージで、デコードが
中身を埋める。各 `MfskResult::text` は固定長のインラインバッファ
(ヒープポインタではない) — list 全体が 1 回のアロケーションで
`mfsk_result_list_free` により 1 回で解放される (issue #205 以前は
`text` はメッセージごとのヒープ `CString*` だった。`mfsk-ffi-ft8` は
元々この固定バッファ方式で、`mfsk-ffi` も揃えて両クレートで ABI
形状を共有するようにした)。

`MfskSamples` は呼び出し元が確保するストレージで、エンコードが
中身を埋める。12 kHz f32 PCM を保持し `mfsk_samples_free` で解放。

最小 E2E デモは `mfsk-ffi/examples/cpp_smoke/` 参照。

### メモリルール

1. **ハンドル**: `mfsk_decoder_new` で確保、`mfsk_decoder_free` で解放。
   スレッドあたり 1 ハンドル。NULL に対する free は no-op。
2. **結果リスト**: `MfskResultList` をスタック上でゼロ初期化、
   そのアドレスを decode に渡し、読み終わったら
   `mfsk_result_list_free` で解放。`text` は固定長インラインバッファ
   なので個別に free するポインタはない。
3. **サンプルバッファ**: `MfskSamples` をゼロ初期化、encode に渡し、
   `mfsk_samples_free` で解放。
4. **デコードオプション**: `mfsk_decode_options_new` で確保する
   任意の `MfskDecodeOptions*` ハンドル、`mfsk_decode_options_free`
   で解放。NULL は常に有効 (プロトコル既定値を使う)。
5. **エラー**: `MfskStatus` が非ゼロの場合、**同じスレッド** で
   `mfsk_last_error` を呼ぶと診断メッセージが得られる。返される
   ポインタは次の fallible 呼び出しまで有効。

### スレッド安全性

* `MfskDecoder` は `!Sync`: 並行スレッドごとに 1 ハンドル
* デコーダはキャッシュとエラー報告にスレッドローカルを使うので、
  複数スレッドそれぞれが自分のハンドルを持つコストは小さい

## 9. Kotlin / Android

`mfsk-ffi/examples/kotlin_jni/` にそのまま使える雛形:

```kotlin
package io.github.mfskcore

Mfsk.open(Mfsk.Protocol.FT4).use { dec ->
    val pcm: ShortArray = /* 取得した音声 */
    for (m in dec.decode(pcm, sampleRate = 12_000)) {
        Log.i("ft4", "${m.freqHz} Hz  ${m.snrDb} dB  ${m.text}")
    }
}
```

* `libmfsk.so` は `cargo build --target aarch64-linux-android -p mfsk-ffi` で生成
* `libmfsk_jni.so` は約 115 行の C shim、`ShortArray` ↔
  `MfskResultList` を変換
* `Mfsk.kt` は `AutoCloseable` な Kotlin クラス。`.use { }` で確実
  に解放

詳細は `mfsk-ffi/examples/kotlin_jni/README.md` 参照。

## 10. プロトコル対応状況

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

FST4 は FT8 の LDPC(174, 91) ではなく LDPC(240, 101) + 24 bit CRC を
用いる別の符号系で、`fec::ldpc240_101` として実装している。
BP / OSD のアルゴリズムは LDPC サイズが変わっても構造的に同じなので、
新たに用意したのはパリティ検査行列・生成行列と符号寸法だけである。
実装済みの 5 sub-mode (FST4-15/30/60A/120/300) は全経路が動作する
状態で揃っており、異なるのは `NSPS` / `SYMBOL_DT` /
`TONE_SPACING_HZ` のみ (FST4-15 だけは `TX_START_OFFSET_S` も
0.5 s で他と異なる) — `q65_submode!` と同じパターンの
`fst4_submode!` マクロが各 ZST を生成する。FST4-900 / FST4-1800 は
未実装のまま (現時点で需要なし)。FST4W (WSPR 型片方向 50 bit
ビーコン、LDPC(240, 74)、周期 120/300/900/1800 s) は別のメッセージ
形式であり本書の対象外 — issue #23 参照。

WSPR はこれまでの FT モードと構造的に異なる。LDPC ではなく畳み込み符号
(`fec::conv::ConvFano`、WSJT-X `lib/wsprd/fano.c` を移植)、
77 bit ではなく 50 bit のメッセージ (`msg::wspr::Wspr50Message`
で Type 1 / 2 / 3 を実装)、ブロック Costas ではなくシンボル毎の
interleaved sync (`SyncMode::Interleaved`) を用いる。`wspr` モジュール
自身は TX 合成・RX 復調・四半シンボル粒度のスペクトログラムによる
coarse search を提供しており、120 s スロット全体の探索を妥当な時間で
実行できるように構成している。

JT9 は WSPR と同じ畳み込み FEC 系統を使うが、`ConvFano` をそのまま
共有するのではなく独自の `ConvFano232` 型 (`fec::conv`) として実装
している — JT9 の 206 bit 符号語フレーミングが WSPR とは異なるため。
加えて 72 bit JT メッセージコーデック (`msg::jt72::Jt72Codec`) を
使う。JT65 は Reed-Solomon `fec::rs::Rs63_12` (`fec::Rs63_12` として
re-export) を用い、Karn の Berlekamp-Massey アルゴリズムに基づく
消失対応復号を提供する。

Q65 は第 3 の FEC 系統を導入する: GF(64) 上の Q-ary
repeat-accumulate 符号で、Walsh-Hadamard メッセージにより確率
ベクトル領域で非二進 belief propagation を実行する
(`fec::qra::QraCode` と具象コードインスタンス
`fec::qra15_65_64::QRA15_65_64_IRR_E23`)。アプリケーション層は
13 個のユーザ情報シンボルに CRC-12 を付与した上で 65 シンボルの
符号語から CRC 2 シンボルを puncture することで 63 シンボルを
実送信する。実装した 10 sub-mode は同じ FEC + 同期配置 + 77 bit
メッセージを共有し、`NSPS` (15-s / 30-s / 60-s / 120-s / 300-s
スロット) とトーン間隔 (×1, ×2, ×4, ×8, ×16) のみが異なる。§3 の
5 つの並列デコード戦略 (AWGN BP, AP-hint BP, fast-fading metric,
AP-list テンプレート照合, multi-period EMA averaging) はすべて
同じ QRA codec を利用する。

### 10.1 スコープ境界: 応用例としての `uvpacket`

`uvpacket` は in-tree ですが WSJT 系の**外側**に位置します。FEC
基盤 (`Ldpc240_101`、BP、OSD-2/3) を WSJT の変調・同期・メッセージ
コーデック・スロット仕様を一切共有しないプロトコルで再利用する
**応用例**です。具体的には狭帯域 FM 音声チャンネル (HT/モバイル、
~3 kHz 音声帯域) 向け 4 モードパケットプロトコルで、単一搬送波
**π/4-DQPSK + LMS イコライザ** + RRC パルス、4 種の 127 chip BPSK
m-sequence プリアンブル (モード符号化済み)、差動復調 (搬送波
位相追跡器なし)、バイトパイプ API を採用します。

WSJT 系との共有は FEC 親コードに留まります:

| Layer | WSJT 系 | uvpacket |
|---|---|---|
| 変調 | M-ary tone FSK / GFSK | 単一搬送波 π/4-DQPSK + RRC |
| 復調 | 非コヒーレントシンボル電力検出 | LMS イコライザ + 1 シンボル差動 |
| スロット | 固定 7.5 / 15 / 60 / 120 s | 可変長バースト |
| 同期 | tone index Costas ブロック | 4 変種 127 chip BPSK m-sequence (モード符号化) |
| メッセージ | 構造化 (callsign + grid) | バイトパイプ (`app_type` タグ) |
| パイプライン | 汎用 `mfsk-core` TX/RX | 専用 `uvpacket::{tx,rx}` |
| FEC | (モード固有) | `Ldpc240_101` (FST4 と共有) + 専用 unpunctured ヘッダブロック |

uvpacket が汎用 TX/RX パイプラインを bypass するため、
`ModulationParams` trait 定数のうち `NTONES = 4`、`GFSK_BT`、
`TONE_SPACING_HZ`、`GFSK_HMOD` は **decorative** — trait signature
と `protocol_invariants` テストを満たすためだけに存在し、
`tx::encode` や `rx::decode_known_layout` から参照されません。
このトレードオフは
[`mfsk-core/src/uvpacket/protocol.rs`](../../mfsk-core/src/uvpacket/protocol.rs)
で明文化されており、非 WSJT プロトコルを sibling crate として切り
出さず in-tree に残した自然な帰結です。

実際の受信パイプラインを bypass しているにもかかわらず、uvpacket の
4 つの sub-mode ZST (`UvRobust`, `UvStandard`, `UvUltraRobust`,
`UvExpress`) は実際に `Protocol` を実装しており、`PROTOCOLS`
レジストリ (§7.1) と `tests/protocol_invariants.rs` の両方に
配線されています — `tx::encode`/`rx::decode_known_layout` がその
大半を読まなくても、列挙・不変条件検査の目的では trait 面が満たされ
ています。

#### Trait scope を 2 方向から probe する見立て

0.4.0 リリースは trait 抽象に対して **正反対の 2 方向のストレス
テスト** を同時に出荷しました:

- **Q65 ファミリ拡張 — *positive* probe**。trait 表面を非バイナリ
  符号 (QRA over GF(2⁶)) と 4 並列デコード戦略 (AWGN / AP-hint /
  fast-fading / AP-list) まで押し広げて、形が崩れないことを確認
  した。`Protocol` / `ModulationParams` / `FrameLayout` /
  `FecCodec` / `MessageCodec` の各層が、1 つのマクロから生やした
  6 サブモードに対してそのまま伸びた。
- **`uvpacket` — *negative* probe**。WSJT 系の外側に変調・同期・
  メッセージ形式・スロット政策のすべての軸で踏み出し、抽象が
  どこで自然に剥離するかを観測した。FEC + DSP + チャンネル
  テスト基盤は持ち越せた一方、汎用 TX/RX パイプラインと
  message-codec / AP-compat 系 trait は剥離した。

この剥離は trait 抽象の**不足**ではなく、trait 表面が **WSJT 系
プロトコルファミリのために適切に scope されている**ことの根拠
です。汎用 PHY フレームワークだったら `SYNC_MODE` を「Costas でも
m-sequence でも」と一般化する代償に WSJT 系のコードが余分な
indirection を払うことになる。同じ視点を応用例側から書いた
1 セクションが [`docs/reference/UVPACKET.ja.md`](UVPACKET.ja.md) §0 にあり、
本節と対のペアです。

uvpacket の完全な設計ナラティブ、AX.25 / M17 / D-STAR / DMR /
VARA との比較、特性測定曲線については
[`docs/reference/UVPACKET.ja.md`](UVPACKET.ja.md) を参照。代表的な WAV
サンプルは `audio_samples/uvpacket/` 配下。

## ライセンス

ライブラリコードは GPL-3.0-or-later。WSJT-X のリファレンス
アルゴリズム由来。
