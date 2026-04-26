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
| Q65-30A      | 30 s    | QRA(15, 65) GF(2⁶) + CRC-12 | 77 bit   | 22 分散位置       | `lib/qra/q65/` |
| Q65-60A‥E    | 60 s    | (同 QRA codec)              | 77 bit    | (同 sync 配置)    | `lib/qra/q65/` |

Q65 は 6 個の sub-mode 構成で実装している — 地上波向け Q65-30A
1 つに加え、EME 帯向けの 60 秒スロット 5 種 (Q65-60A〜Q65-60E、
トーン間隔倍率 ×1, ×2, ×4, ×8, ×16)。これらは FEC、メッセージ
コーデック、同期配置、共通の trait 実装ブロックを共有しており、
NSPS とトーン間隔のみが sub-mode ごとに異なる。

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

#### Q65 — 第 3 の FEC 系統 + 4 戦略 × 6 sub-mode

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
3. **6 sub-mode を 1 個の impl ブロックで共有**: 地上波向け
   Q65-30A に加え、EME 用の Q65-60A〜E (6 m / 70 cm / 23 cm /
   5.7 GHz / 10 GHz / 24 GHz+)。NSPS とトーン間隔のみが異なる。
   `q65_submode!` マクロが各 sub-mode の ZST と trait 実装を 1 行で
   生成するため、sub-mode ごとのコード重複は無い。
4. **4 つの並列なデコード戦略**: 同一 FEC フレームに対し正当に
   選び得る受信経路が 4 通り存在するのは Q65 が初めて — plain AWGN
   BP、AP-biased BP、fast-fading metric、AP-list テンプレート照合。
   §3 で詳述する。各戦略は sub-mode ZST に対して generic な
   別個のエントリポイント関数として共存し、内部の FEC と
   メッセージコーデックは共有する。

WSPR が「**FEC 系統 + メッセージ長 + 同期形式**」を独立に差し替え
られることを示したのに対し、Q65 は「第 3 の FEC 系統 + 6 sub-mode
+ 4 戦略」がすべて `Protocol` super-trait の中に収まり、専用の
配管を増やすことなく実現できることを示している。§3 で 4 戦略を
詳述し、§7 で `PROTOCOLS` レジストリ (実装される 11 ZST すべての
列挙) と `tests/protocol_invariants.rs` の汎用検査機構を扱う。

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
│   ├── rs63_12.rs      RS(63, 12) GF(2⁶) — JT65
│   └── qra/            Q-ary RA codec ファミリ — Q65
│       ├── code.rs       汎用 QRA エンコーダ + 非二進 BP デコーダ
│       ├── q65.rs        Q65 アプリケーション層 (CRC-12 + puncturing) +
│       │                 リストデコード関数 (check_codeword_llh,
│       │                 decode_with_codeword_list)
│       ├── fast_fading.rs ドップラー拡散対応 intrinsic metric
│       └── fading_tables.rs Gaussian / Lorentzian キャリブレーション表
├── msg/              メッセージコーデック
│   ├── wsjt77.rs       77 bit WSJT メッセージ (pack / unpack) — FT8, FT4, FST4, Q65
│   ├── wspr.rs         50 bit WSPR Types 1 / 2 / 3
│   ├── jt72.rs         72 bit JT メッセージ — JT9, JT65
│   └── hash.rs         コールサインハッシュテーブル
├── registry.rs       PROTOCOLS 静的配列 + ProtocolMeta + by_id / by_name
├── ft8/              FT8 ZST + decode + wave_gen
├── ft4/              FT4 ZST + decode
├── fst4/             FST4-60A ZST + decode
├── wspr/             WSPR ZST + decode + synth + spectrogram search
├── jt9/              JT9 ZST + decode
├── jt65/             JT65 ZST + decode (+ 消失対応 RS)
└── q65/              Q65 ファミリ — 6 sub-mode ZST + decode + synth
    ├── protocol.rs     q65_submode! マクロ (Q65a30..Q65e60 ZST 生成)
    ├── rx.rs           4 つのデコード戦略 (AWGN / AP / fast-fading / AP-list)
    ├── ap_list.rs      standard_qso_codewords — full AP-list 候補生成
    ├── tx.rs           65-FSK 合成器 (sub-mode 対応)
    ├── search.rs       22 シンボル Costas-block coarse 検索
    └── sync_pattern.rs Q65 分散同期配置
```

各プロトコルモジュールはフィーチャーフラグ (`ft8`、`ft4`、`fst4`、
`wspr`、`jt9`、`jt65`、`q65`) で gate されている。`core`、`fec`、
`msg`、`registry` は常時利用可能。

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

4. **既存プロトコルの sub-mode 追加** (例: Q65-60A〜E が Q65-30A と
   NSPS とトーン間隔以外を共有) — `q65_submode!` マクロが差分定数を
   受け取り、ZST と 3 つの trait 実装を 1 行で生成する。新規テストや
   パイプライン変更は不要 — `tests/protocol_invariants.rs` に
   1 行追加するだけで構造健全性チェックが自動的に走る。

## 3. デコード戦略 (Q65 ケーススタディ)

このライブラリの大半のプロトコルはデコード経路が 1 通りしかない
(FT 系列の `decode_frame::<P>`、WSPR の
`wspr::decode::decode_scan_default` など)。Q65 は同一 FEC フレームに
対して**正当に選び得る受信経路が 4 通り**ある最初の実装プロトコル
で、各経路が異なる種類のチャネル劣化に対して計算コストとデコード
閾値のトレードオフを実現している。これらは `mfsk_core::q65::rx`
内で並列な関数ファミリとして共存し、すべて sub-mode ZST に対して
generic である。

| 状況                                                  | 戦略                       | エントリポイント                                              | 閾値ゲイン       |
|-------------------------------------------------------|----------------------------|---------------------------------------------------------------|------------------|
| デフォルト — チャネル特性 / メッセージ未知            | AWGN Bessel + BP           | `decode_at_for<P>` / `decode_scan_for<P>`                     | 基準             |
| コールサイン or レポート既知、地上波チャネル          | AP-biased BP               | `decode_at_with_ap_for<P>` / `decode_scan_with_ap_for<P>`     | ~2 dB            |
| ドップラー拡散チャネル (≥10 Hz、microwave EME)        | Fast-fading metric + BP    | `decode_at_fading_for<P>` / `decode_scan_fading_for<P>`       | 拡散時 5–8 dB    |
| コールペア既知、QSO 文脈無し、地上波                  | AP-list テンプレート照合   | `decode_at_with_ap_list_for<P>` / `decode_scan_with_ap_list_for<P>` | ~3 dB |

**AWGN Bessel + BP** は教科書通りの経路: シンボル毎 FFT エネルギーを
Bessel-I0 metric で確率ベクトル化し、QRA 符号上で非二進 belief
propagation を実行する。加法ガウスノイズに近いチャネルで安全に動く。

**AP-biased BP** は BP 開始前に既知の情報ビット位置で intrinsic
確率ベクトルをクランプする。正しい hint は BP の収束点を真値側に
寄せ、誤った hint は誤デコードよりも収束失敗を引き起こす傾向が
ある (CRC が残りを捕捉する)。`mfsk_core::msg::ApHint` の builder
(`with_call1`, `with_call2`, `with_grid`, `with_report`) で構築する。

**Fast-fading metric** は Bessel front end を、Gaussian / Lorentzian
fading 形状に対してキャリブレーションされた拡散対応 metric に
置き換える。トーンが 10–60 Hz に拡散する microwave EME で必須:
リファレンス録音 `samples/Q65/60D_EME_10GHz/` (10 GHz EME) は
この経路で復号できるが、AWGN front end では 0 件。`b90_ts` は
拡散帯域 × シンボル周期 (代表値: 0.05 = ほぼ AWGN、1.0 = 中程度、
5.0+ = 強拡散)。

**AP-list テンプレート照合**は BP を**走らせない**。代わりに生成器
`q65::ap_list::standard_qso_codewords(my_call, his_call, his_grid)`
が WSJT-X "full AP list" — 既知のコールペアが正当に生成し得る
標準交換 206 件 (`MYCALL HISCALL`、`MYCALL HISCALL RRR/RR73/73`、
`CQ HISCALL grid`、加えて 200 件の SNR ladder) — を事前に符号化する。
デコーダは soft observation との対数尤度がリスト規模調整済み
閾値を超える候補を選ぶか、そうでなければ `None` を返す。
SNR −25 dB (公開閾値より 1 dB 低い) のスイープ試験では plain BP が
0/6 失敗するのに対し AP-list は 6/6 復号する。

**戦略の合成**: fast-fading と AP-list は合成可能で、fast-fading の
intrinsic ベクトルをそのまま `Q65Codec::decode_with_codeword_list`
に渡せる。専用の `decode_at_fading_with_ap_list_for<P>` を配線する
のは小さな additive な作業; 現状は呼び出し側が低レベル primitive
(`intrinsics_fast_fading` + `Q65Codec`) で明示的に合成する。

C ABI には同 4 戦略が `mfsk_q65_decode`、`mfsk_q65_decode_with_ap`、
`mfsk_q65_decode_fading`、`mfsk_q65_decode_with_ap_list` として
1 対 1 で公開されており、いずれも `MfskQ65SubMode` 引数を取って
6 sub-mode のいずれにもアクセス可能。

## 4. 共有プリミティブ (`core`)

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

## 5. Feature flags

| フィーチャー  | デフォルト | 効果                                                         |
|---------------|------------|--------------------------------------------------------------|
| `ft8`         | on         | FT8 ZST、decode、wave_gen                                   |
| `ft4`         | on         | FT4 ZST、decode                                             |
| `fst4`        | off        | FST4-60A ZST、decode                                        |
| `wspr`        | off        | WSPR ZST、decode、synth、spectrogram search                 |
| `jt9`         | off        | JT9 ZST、decode                                             |
| `jt65`        | off        | JT65 ZST、decode (+ 消失対応 RS)                            |
| `q65`         | off        | Q65-30A + Q65-60A‥E ZST、4 デコード戦略、synth              |
| `full`        | off        | 全 7 プロトコルの集約                                        |
| `parallel`    | on         | パイプラインで rayon `par_iter` (WASM は無効化)              |
| `osd-deep`    | off        | AP ≥55 bit ロック時に OSD-3 フォールバック追加              |
| `eq-fallback` | off        | `EqMode::Adaptive` が EQ 失敗時に非 EQ にフォールバック      |

`osd-deep` + `eq-fallback` は重い: FT4 −18 dB 成功率を 5/10 → 6/10 に
引き上げる代償としてデコード時間が約 10× 増える。WASM の 7.5 s スロット
予算内に収まるよう **既定 off**、CPU 余裕のあるデスクトップでのみ
有効化する想定。

## 6. Rust から利用する

### 6.1 依存関係

```toml
[dependencies]
mfsk-core = { version = "0.1", features = ["ft8", "ft4", "wspr"] }
```

必要なプロトコルのフィーチャーだけ有効にすれば十分。以下では複数を
有効にした例を示す。

### 6.2 FT8 デコード — 最小例

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

### 6.3 WSPR — 別系統の復調 (abstraction と両立する形で)

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

### 6.4 Sniper モード + AP hint

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

### 6.5 JT9 / JT65

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
  全 entry を返す。Q65 は 6 件 (sub-mode 毎)、その他は 1 件。
* `mfsk_core::by_name("Q65-60D")` — 表示名による厳密一致検索。
* `mfsk_core::for_protocol_id(id)` — 同じ id を持つ最初の entry。
  「ファミリ毎に 1 mode」のケースで便利。

Q65 は registry 上で family / sub-mode 区別が顕在化する例:
6 sub-mode 全てが `ProtocolId::Q65` を共有 (FFI tag が family
レベルである故) しつつ、NSPS / トーン間隔 / スロット長が異なる
ため独立した entry になる。

レジストリ本体は `mfsk-core/src/registry.rs` 内部の
`protocol_meta!` マクロで構築される。新しいプロトコルの追加は
ZST + 表示名で 1 行ずつ。

### 7.2 汎用 trait 面検査

`tests/protocol_invariants.rs` は `assert_protocol_invariants::<P:
Protocol>(name)` の単一の generic 関数をすべての実装 ZST に対して
実行する。本体は FT8、FT4、FST4、WSPR、JT9、JT65 と 6 sub-mode の
Q65 — 11 invocation × 1 実装。3 つのヘルパー関数が合計 17 個の
不変条件を pin する:

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
    MFSK_PROTOCOL_FST4S60 = 5,
    MFSK_PROTOCOL_Q65A30  = 6,
};

// Q65 sub-mode は family enum とは別の専用 enum で識別:
enum MfskQ65SubMode { A30=0, A60=1, B60=2, C60=3, D60=4, E60=5 };
enum MfskQ65FadingModel { Gaussian=0, Lorentzian=1 };

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
MfskStatus        mfsk_encode_q65(MfskQ65SubMode submode,
                                  const char* call1, const char* call2,
                                  const char* report, float freq_hz,
                                  MfskSamples* out);

// Q65 専用 4 戦略 (sub-mode 引数で 6 sub-mode のいずれにも適用):
MfskStatus        mfsk_q65_decode(MfskQ65SubMode, ...);              // AWGN
MfskStatus        mfsk_q65_decode_with_ap(MfskQ65SubMode, ...,
                                  const char* ap_call1, ap_call2,
                                  ap_grid, ap_report, ...);          // AP-biased BP
MfskStatus        mfsk_q65_decode_fading(MfskQ65SubMode, ...,
                                  float b90_ts,
                                  MfskQ65FadingModel, ...);          // fast-fading
MfskStatus        mfsk_q65_decode_with_ap_list(MfskQ65SubMode, ...,
                                  const char* my_call,
                                  const char* his_call,
                                  const char* his_grid, ...);        // AP-list

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
  `MfskMessageList` を変換
* `Mfsk.kt` は `AutoCloseable` な Kotlin クラス。`.use { }` で確実
  に解放

詳細は `mfsk-ffi/examples/kotlin_jni/README.md` 参照。

## 10. プロトコル対応状況

| プロトコル       | スロット   | トーン | シンボル | トーン Δf  | FEC                   | Msg   | Sync          | 状態 |
|------------------|------------|--------|----------|------------|-----------------------|-------|---------------|------|
| FT8              | 15 s       | 8      | 79       | 6.25 Hz    | LDPC(174, 91)         | 77 b  | 3×Costas-7    | 実装済 |
| FT4              | 7.5 s      | 4      | 103      | 20.833 Hz  | LDPC(174, 91)         | 77 b  | 4×Costas-4    | 実装済 |
| FST4-60A         | 60 s       | 4      | 160      | 3.125 Hz   | LDPC(240, 101)        | 77 b  | 5×Costas-8    | 実装済 |
| FST4 他サブモード | 15–1800 s  | 4      | 可変     | 可変       | LDPC(240, 101)        | 77 b  | 5×Costas-8    | ZST 1 つ/サブモード |
| WSPR             | 120 s      | 4      | 162      | 1.465 Hz   | conv r=½ K=32 + Fano  | 50 b  | シンボル毎 LSB (npr3) | 実装済 |
| JT9              | 60 s       | 9      | 85       | 1.736 Hz   | conv r=½ K=32 + Fano  | 72 b  | 16 分散位置   | 実装済 |
| JT65             | 60 s       | 65     | 126      | 2.69 Hz    | RS(63, 12) GF(2⁶)     | 72 b  | 63 分散位置   | 実装済 |
| Q65-30A          | 30 s       | 65     | 85       | 3.333 Hz   | QRA(15, 65) GF(2⁶) + CRC-12 | 77 b | 22 分散位置 | 実装済 |
| Q65-60A          | 60 s       | 65     | 85       | 1.667 Hz   | (同 QRA codec)        | 77 b  | (同)          | 実装済 (6 m EME) |
| Q65-60B          | 60 s       | 65     | 85       | 3.333 Hz   | (同 QRA codec)        | 77 b  | (同)          | 実装済 (70 cm / 23 cm EME) |
| Q65-60C          | 60 s       | 65     | 85       | 6.667 Hz   | (同 QRA codec)        | 77 b  | (同)          | 実装済 (~3 GHz EME) |
| Q65-60D          | 60 s       | 65     | 85       | 13.33 Hz   | (同 QRA codec)        | 77 b  | (同)          | 実装済 (5.7 / 10 GHz EME) |
| Q65-60E          | 60 s       | 65     | 85       | 26.67 Hz   | (同 QRA codec)        | 77 b  | (同)          | 実装済 (24 GHz+、強拡散) |

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

Q65 は第 3 の FEC 系統を導入する: GF(64) 上の Q-ary
repeat-accumulate 符号で、Walsh-Hadamard メッセージにより確率
ベクトル領域で非二進 belief propagation を実行する
(`fec::qra::QraCode` と具象コードインスタンス
`fec::qra15_65_64::QRA15_65_64_IRR_E23`)。アプリケーション層は
13 個のユーザ情報シンボルに CRC-12 を付与した上で 65 シンボルの
符号語から CRC 2 シンボルを puncture することで 63 シンボルを
実送信する。実装した 6 sub-mode は同じ FEC + 同期配置 + 77 bit
メッセージを共有し、`NSPS` (30 s vs 60 s スロット) と
トーン間隔 (×1, ×2, ×4, ×8, ×16) のみが異なる。§3 の 4 つの
並列デコード戦略 (AWGN BP, AP-biased BP, fast-fading metric,
AP-list テンプレート照合) はすべて同じ QRA codec を利用する。

## ライセンス

ライブラリコードは GPL-3.0-or-later。WSJT-X のリファレンス
アルゴリズム由来。
