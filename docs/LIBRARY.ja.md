# mfsk-core — ライブラリアーキテクチャ & API リファレンス

> **English:** [LIBRARY.md](LIBRARY.md)

mfsk-core を組み込む開発者向けリファレンス。Rust クレートとして使う場合、
C/C++ から `libmfsk.so` をリンクする場合、Kotlin/Android で JNI 雛形を
使う場合すべて対象。

クイックスタート (バッジ・依存設定・最小コード例) は
[README.md](../README.md) を参照。本ドキュメントでは設計の *なぜ* と
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
コーデックの共通部) は `core`、`fec`、`msg` モジュールにまとめ、
各プロトコルは固有定数と採用する FEC / メッセージコーデックを宣言する
比較的小さな ZST (zero-sized type) として表現する。パイプラインは
`decode_frame::<P>()` の形で `P: Protocol` をコンパイル時型パラメータ
として受け取り、monomorphize によってプロトコルごとに特殊化された
コードが生成されるので、抽象化のためにランタイムコストが増えることは
ない。

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

### 0.5 設計が機能していることの確認 — WSPR を例に

FT8 / FT4 / FST4 はいずれも LDPC + 77 bit メッセージ + ブロック Costas
同期という共通点が多く、共通化の恩恵が大きい一方、共通点の多さが
抽象化の良し悪しを測る材料にはなりにくい。その意味で WSPR は次の
3 点で FT 系と構造的に異なり、抽象化の妥当性を確認する題材になる。

1. **FEC の系統**: LDPC ではなく畳み込み符号 (r=1/2、拘束長 32) と
   Fano 逐次復号。`mfsk_core::fec::conv::ConvFano` として追加した。
2. **メッセージ長**: 77 bit ではなく 50 bit。Type 1 / 2 / 3 の
   メッセージ形式を `mfsk_core::msg::wspr::Wspr50Message` で実装。
3. **同期構造**: ブロック Costas ではなく、チャネルシンボル
   すべての LSB に 162 bit の sync vector を埋め込む形式。これを
   表現するために `FrameLayout::SYNC_MODE` に `Interleaved` バリアントを
   追加した。

これら 3 点はいずれも trait 面の別々の軸に変更を加えるものだったが、
それぞれ新しい実装・バリアントを追加することで吸収でき、
FT8 / FT4 / FST4 のコード経路には手を入れていない。実際、3 モードの
trait 実装は引き続き `SyncMode::Block` を使用し、以前と同じバイト列を
生成する。

## 1. モジュール構成

```
mfsk_core
├── core/             Protocol trait 群、DSP、sync、LLR、equaliser、pipeline
│   ├── protocol.rs     ModulationParams / FrameLayout / Protocol / FecCodec / MessageCodec
│   ├── dsp/            resample · downsample · gfsk · subtract
│   ├── sync.rs         coarse_sync / refine_candidate
│   ├── llr.rs          symbol_spectra / compute_llr / sync_quality
│   ├── equalize.rs     equalize_local (トーン毎 Wiener)
│   └── pipeline.rs     decode_frame / decode_frame_subtract / process_candidate_basic
├── fec/              FecCodec 実装群
│   ├── ldpc174_91.rs   LDPC(174, 91)  — FT8, FT4
│   ├── ldpc240_101.rs  LDPC(240, 101) — FST4
│   ├── conv.rs         ConvFano r=½ K=32 — WSPR, JT9
│   └── rs63_12.rs      RS(63, 12) GF(2⁶) — JT65
├── msg/              メッセージコーデック
│   ├── wsjt77.rs       77 bit WSJT メッセージ (pack / unpack) — FT8, FT4, FST4
│   ├── wspr.rs         50 bit WSPR Types 1 / 2 / 3
│   ├── jt72.rs         72 bit JT メッセージ — JT9, JT65
│   └── hash.rs         コールサインハッシュテーブル
├── ft8/              FT8 ZST + decode + wave_gen
├── ft4/              FT4 ZST + decode
├── fst4/             FST4-60A ZST + decode
├── wspr/             WSPR ZST + decode + synth + spectrogram search
├── jt9/              JT9 ZST + decode
└── jt65/             JT65 ZST + decode (+ 消失対応 RS)
```

各プロトコルモジュールはフィーチャーフラグ (`ft8`、`ft4`、`fst4`、
`wspr`、`jt9`、`jt65`) で gate されている。`core`、`fec`、`msg` は
常時利用可能。

ワークスペースの兄弟クレート `mfsk-ffi` が同じクレートの上に C ABI
共有ライブラリ (`libmfsk.{so,a,dylib}` + `mfsk.h`) を構築する。

## 2. Protocol トレイト階層

対応するすべてのモードは、3 つの合成可能な trait を実装する
**Zero-Sized Type (ZST)** で記述される:

```rust
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
use mfsk_core::core::protocol::*;
use mfsk_core::fec::ldpc174_91::Ldpc174_91;
use mfsk_core::msg::wsjt77::Wsjt77Message;

pub struct Ft4;

impl ModulationParams for Ft4 {
    const NTONES: u32 = 4;
    const BITS_PER_SYMBOL: u32 = 2;
    const NSPS: u32 = 576;          // 48 ms @ 12 kHz
    const TONE_SPACING_HZ: f32 = 20.833;
    const GRAY_MAP: &'static [u8] = &[0, 1, 3, 2];
    // … (GFSK_BT、NFFT_PER_SYMBOL_FACTOR、NDOWN などの定数)
}

impl FrameLayout for Ft4 {
    const N_DATA: u32 = 87;
    const N_SYNC: u32 = 16;
    const N_SYMBOLS: u32 = 103;
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
use mfsk_core::fec::conv::ConvFano;
use mfsk_core::msg::wspr::Wspr50Message;

pub struct Wspr;

impl ModulationParams for Wspr {
    const NTONES: u32 = 4;
    const BITS_PER_SYMBOL: u32 = 2;
    const NSPS: u32 = 8192;                  // 約 683 ms @ 12 kHz
    const TONE_SPACING_HZ: f32 = 12_000.0 / 8192.0;  // ≈ 1.4648
    const GRAY_MAP: &'static [u8] = &[0, 1, 2, 3];
    // …
}

impl FrameLayout for Wspr {
    const N_DATA: u32 = 162;
    const N_SYNC: u32 = 0;                   // sync はデータシンボルに埋込
    const N_SYMBOLS: u32 = 162;
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
```

呼び出し側のパイプラインは `decode_frame::<Ft4>(...)` または
WSPR 専用の `wspr::decode::decode_scan_default(...)` のように
型引数でプロトコルを指定するだけで済み、合成の結果として選ばれた
FEC・メッセージコーデック・同期方式が自動的に使われる。

### Monomorphization とゼロコスト抽象

ホットパス (`core::sync::coarse_sync::<P>`、
`core::llr::compute_llr::<P>`、
`core::pipeline::process_candidate_basic::<P>`、…) はすべて
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
   構わず、`decode_frame::<P>()` パイプライン全体がそのまま動く。

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

## 3. 共有プリミティブ (`core`)

### DSP (`mfsk_core::core::dsp`)

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

### Sync (`mfsk_core::core::sync`)

* `coarse_sync::<P>(audio, freq_min, freq_max, …)` — UTC 整列 2D
  ピーク探索、`P::SYNC_MODE.blocks()` を走査
* `refine_candidate::<P>(cd0, cand, search_steps)` — 整数サンプル
  スキャン + 放物線サブサンプル補間
* `make_costas_ref(pattern, ds_spb)` / `score_costas_block(...)` —
  診断・カスタムパイプライン用の生相関ヘルパー

### LLR (`mfsk_core::core::llr`)

* `symbol_spectra::<P>(cd0, i_start)` — シンボル単位 FFT bin
* `compute_llr::<P>(cs)` — WSJT 式 4 バリアント LLR (a/b/c/d)
* `sync_quality::<P>(cs)` — 硬判定 sync シンボル一致数

### Equalise (`mfsk_core::core::equalize`)

* `equalize_local::<P>(cs)` — `P::SYNC_MODE.blocks()` pilot 観測から
  トーン毎 Wiener equalizer を推定、Costas が訪問しないトーンは
  線形外挿でカバー

### Pipeline (`mfsk_core::core::pipeline`)

* `decode_frame::<P>(...)` — coarse sync → 並列 process_candidate → dedupe
* `decode_frame_subtract::<P>(...)` — 3-pass SIC ドライバ
* `process_candidate_basic::<P>(...)` — 候補単体の BP+OSD

AP 対応版は `msg::pipeline_ap` に配置 (AP hint 構築が
77-bit 形式に依存するため)。

## 4. Feature flags

| フィーチャー  | デフォルト | 効果                                                         |
|---------------|------------|--------------------------------------------------------------|
| `ft8`         | on         | FT8 ZST、decode、wave_gen                                   |
| `ft4`         | on         | FT4 ZST、decode                                             |
| `fst4`        | off        | FST4-60A ZST、decode                                        |
| `wspr`        | off        | WSPR ZST、decode、synth、spectrogram search                 |
| `jt9`         | off        | JT9 ZST、decode                                             |
| `jt65`        | off        | JT65 ZST、decode (+ 消失対応 RS)                            |
| `full`        | off        | 全 6 プロトコルの集約                                        |
| `parallel`    | on         | パイプラインで rayon `par_iter` (WASM は無効化)              |
| `osd-deep`    | off        | AP ≥55 bit ロック時に OSD-3 フォールバック追加              |
| `eq-fallback` | off        | `EqMode::Adaptive` が EQ 失敗時に非 EQ にフォールバック      |

`osd-deep` + `eq-fallback` は重い: FT4 −18 dB 成功率を 5/10 → 6/10 に
引き上げる代償としてデコード時間が約 10× 増える。WASM の 7.5 s スロット
予算内に収まるよう **既定 off**、CPU 余裕のあるデスクトップでのみ
有効化する想定。

## 5. Rust から利用する

### 5.1 依存関係

```toml
[dependencies]
mfsk-core = { version = "0.1", features = ["ft8", "ft4", "wspr"] }
```

必要なプロトコルのフィーチャーだけ有効にすれば十分。以下では複数を
有効にした例を示す。

### 5.2 FT8 デコード — 最小例

```rust
use mfsk_core::ft8::{
    decode::{decode_frame, DecodeDepth},
    wave_gen::{message_to_tones, tones_to_i16},
};
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

// 2. デコードする。
for r in decode_frame(&audio, 100.0, 3_000.0, 1.0, None,
                      DecodeDepth::BpAllOsd, 50) {
    if let Some(text) = unpack77(&r.message77) {
        println!("{:7.1} Hz  dt={:+.2} s  SNR={:+.0} dB  {}",
                 r.freq_hz, r.dt_sec, r.snr_db, text);
    }
}
```

### 5.3 WSPR — 別系統の復調 (abstraction と両立する形で)

WSPR は 12 kHz で直接シンボル長 (8192 サンプル) の FFT を取る方式で、
FT 系の「ダウンサンプリングしてからシンボル同期」という流れと
ステージ構成が異なる。そのため `wspr` モジュールが独自のエントリ
ポイントを用意している。ただし内部で使っている FEC (`ConvFano`) と
メッセージコーデック (`Wspr50Message`) は `Wspr: Protocol` の
関連型として宣言済みで、抽象の枠組みからは外れていない。

```rust
use mfsk_core::wspr::decode::decode_scan_default;
use mfsk_core::msg::WsprMessage;

let samples_f32: Vec<f32> = /* 120 秒 × 12 kHz の f32 サンプル */;

let decodes = decode_scan_default(&samples_f32, /*sample_rate*/ 12_000);
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
```

`decode_scan_default` が粗同期 (周波数×時刻探索) を込みでスロット全体を
スキャンする。周波数・開始サンプルが既知の場合は
`wspr::decode::decode_at(samples, rate, start_sample, freq_hz)` を
直接呼べば粗同期を省略できる。

### 5.4 Sniper モード + AP hint

狭帯域 (±500 Hz 程度) に絞って AP hint を与えると、より弱い信号まで
引き出せる:

```rust
use mfsk_core::ft8::decode::{decode_sniper_ap, DecodeDepth, ApHint};
use mfsk_core::core::equalize::EqMode;

let ap = ApHint::new().with_call1("CQ").with_call2("JA1ABC");
for r in decode_sniper_ap(&audio, /*target_hz*/ 1000.0,
                          DecodeDepth::BpAllOsd, /*max_cand*/ 15,
                          EqMode::Adaptive, Some(&ap)) {
    // …
}
```

FT4 側も `ft4::decode::decode_sniper_ap` として同じ形で提供して
いる。

### 5.5 JT9 / JT65

JT9 と JT65 は同じ scan + 単点デコードのパターンを提供する:

```rust
use mfsk_core::jt65::decode_scan_default;

let audio_f32: Vec<f32> = /* 60 秒 × 12 kHz の f32 サンプル */;
for d in decode_scan_default(&audio_f32, 12_000) {
    println!("{:7.2} Hz  {}", d.freq_hz, d.message);
}
```

JT65 はさらに `decode_at_with_erasures` を提供しており、
低 SNR 環境で RS 消失復号が通常デコーダでは落とすフレームを
回復できる。

## 6. C / C++ — `mfsk-ffi`

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
    MFSK_PROTOCOL_FST4S60 = 5,
};

uint32_t          mfsk_version(void);           // major<<16 | minor<<8 | patch
MfskDecoder*      mfsk_decoder_new(MfskProtocol protocol);
void              mfsk_decoder_free(MfskDecoder* dec);

MfskStatus        mfsk_decode_i16(MfskDecoder*, const int16_t* samples,
                                  size_t n, uint32_t sample_rate,
                                  MfskMessageList* out);
MfskStatus        mfsk_decode_f32(MfskDecoder*, const float*,  size_t,
                                  uint32_t, MfskMessageList* out);

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

void              mfsk_message_list_free(MfskMessageList* list);
void              mfsk_samples_free(MfskSamples* s);
const char*       mfsk_last_error(void);
```

`MfskMessageList` は呼び出し元が確保するストレージで、デコードが
中身を埋める。text フィールドは UTF-8 NUL 終端 `char*`、list が所有し
`mfsk_message_list_free` で解放される。

`MfskSamples` は呼び出し元が確保するストレージで、エンコードが
中身を埋める。12 kHz f32 PCM を保持し `mfsk_samples_free` で解放。

最小 E2E デモは `mfsk-ffi/examples/cpp_smoke/` 参照。

### メモリルール

1. **ハンドル**: `mfsk_decoder_new` で確保、`mfsk_decoder_free` で解放。
   スレッドあたり 1 ハンドル。NULL に対する free は no-op。
2. **メッセージリスト**: `MfskMessageList` をスタック上でゼロ初期化、
   そのアドレスを decode に渡し、読み終わったら
   `mfsk_message_list_free` で解放。個別 `text` ポインタを手動で
   free してはいけない。
3. **サンプルバッファ**: `MfskSamples` をゼロ初期化、encode に渡し、
   `mfsk_samples_free` で解放。
4. **エラー**: `MfskStatus` が非ゼロの場合、**同じスレッド** で
   `mfsk_last_error` を呼ぶと診断メッセージが得られる。返される
   ポインタは次の fallible 呼び出しまで有効。

### スレッド安全性

* `MfskDecoder` は `!Sync`: 並行スレッドごとに 1 ハンドル
* デコーダはキャッシュとエラー報告にスレッドローカルを使うので、
  複数スレッドそれぞれが自分のハンドルを持つコストは小さい

## 7. Kotlin / Android

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
  `MfskMessageList` を変換
* `Mfsk.kt` は `AutoCloseable` な Kotlin クラス。`.use { }` で確実
  に解放

詳細は `mfsk-ffi/examples/kotlin_jni/README.md` 参照。

## 8. プロトコル対応状況

| プロトコル       | スロット   | トーン | シンボル | トーン Δf  | FEC                   | Msg   | Sync          | 状態 |
|------------------|------------|--------|----------|------------|-----------------------|-------|---------------|------|
| FT8              | 15 s       | 8      | 79       | 6.25 Hz    | LDPC(174, 91)         | 77 b  | 3×Costas-7    | 実装済 |
| FT4              | 7.5 s      | 4      | 103      | 20.833 Hz  | LDPC(174, 91)         | 77 b  | 4×Costas-4    | 実装済 |
| FST4-60A         | 60 s       | 4      | 160      | 3.125 Hz   | LDPC(240, 101)        | 77 b  | 5×Costas-8    | 実装済 |
| FST4 他サブモード | 15–1800 s  | 4      | 可変     | 可変       | LDPC(240, 101)        | 77 b  | 5×Costas-8    | ZST 1 つ/サブモード |
| WSPR             | 120 s      | 4      | 162      | 1.465 Hz   | conv r=½ K=32 + Fano  | 50 b  | シンボル毎 LSB (npr3) | 実装済 |
| JT9              | 60 s       | 9      | 85       | 1.736 Hz   | conv r=½ K=32 + Fano  | 72 b  | 16 分散位置   | 実装済 |
| JT65             | 60 s       | 65     | 126      | 2.69 Hz    | RS(63, 12) GF(2⁶)     | 72 b  | 63 分散位置   | 実装済 |

FST4 は FT8 の LDPC(174, 91) ではなく LDPC(240, 101) + 24 bit CRC を
用いる別の符号系で、`fec::ldpc240_101` として実装している。
BP / OSD のアルゴリズムは LDPC サイズが変わっても構造的に同じなので、
新たに用意したのはパリティ検査行列・生成行列と符号寸法だけである。
FST4-60A は全経路が動作する状態でまとめている。他のサブモード
(-15 / -30 / -120 / -300 / -900 / -1800) は `NSPS` / `SYMBOL_DT` /
`TONE_SPACING_HZ` のみが異なるため、それぞれ短い ZST を追加して
同じ FEC・同期・DSP を再利用すれば対応できる。

WSPR はこれまでの FT モードと構造的に異なる。LDPC ではなく畳み込み符号
(`fec::conv::ConvFano`、WSJT-X `lib/wsprd/fano.c` を移植)、
77 bit ではなく 50 bit のメッセージ (`msg::wspr::Wspr50Message`
で Type 1 / 2 / 3 を実装)、ブロック Costas ではなくシンボル毎の
interleaved sync (`SyncMode::Interleaved`) を用いる。`wspr` モジュール
自身は TX 合成・RX 復調・四半シンボル粒度のスペクトログラムによる
coarse search を提供しており、120 s スロット全体の探索を妥当な時間で
実行できるように構成している。

JT9 は畳み込み FEC (`ConvFano`) と 72 bit JT メッセージコーデック
(`msg::jt72::Jt72Codec`) を再利用する。JT65 は Reed-Solomon
`fec::rs63_12::Rs63_12` を用い、Karn の Berlekamp-Massey アルゴリズム
に基づく消失対応復号を提供する。

## ライセンス

ライブラリコードは GPL-3.0-or-later。WSJT-X のリファレンス
アルゴリズム由来。
