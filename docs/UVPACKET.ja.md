# uvpacket — 設計と実装ディープダイブ

本書は `uvpacket` プロトコル系統の技術リファレンスである。高レベルな
モチベーションは [`LIBRARY.ja.md` §11](LIBRARY.ja.md#11-uvpacket--モチベーション例)
を参照のこと。ここではアーキテクチャガイドに置くには細かすぎる
内容 — インターリーバの数学、同期配置の導出、チャネルモデル
導出、フェージングテスト方法論、レートラダーの天井 — を扱う。

## チャネルモデル — 帯域フラット時間選択性 Rayleigh フェージング

uvpacket の対象は U/VHF アマチュア移動 / ポータブル運用。
144 / 433 MHz、移動速度 10–150 km/h でのチャネル劣化は以下に
分解できる:

- **マルチパス遅延広がり**: 郊外 / 開地で 0.1–1 μs、市街地で
  ≤ 5 μs 程度。コヒーレンス帯域は遅延広がりの逆数なので最悪でも
  ~1 MHz 程度。3 kHz 音声 passband に対して **3 桁以上フラット
  フェージング** — 帯域全体が 1 つの複素ゲイン `h(t)` で fade する。
- **Doppler 広がり**: 車速 / 波長。例えば 50 km/h × 144 MHz で
  6.7 Hz Doppler。コヒーレンス時間は `9 / (16π fd)` ≈ 27 ms
  だが、実際の fade null 持続時間は **複数コヒーレンス時間** に
  およぶ (Rayleigh エンベロープが I/Q 両象限の小振幅交差点で
  発生するため)。50–200 ms の deep null は日常茶飯事。
- **AWGN フロア**: 受信機 NF 1–5 dB、環境 EMI は中程度。動作 SNR
  は 0–20 dB 範囲が興味の対象。

帯域フラット時間選択性 Rayleigh フェージングは、独立な低域通過
フィルタ済 Gaussian 過程 2 本 (I, Q) を遮断周波数 = Doppler に設定
してモデル化できる。`tests/uvpacket_roundtrip.rs` の試験足場が
このモデルそのものを実装している — `apply_rayleigh_fade` の
1 次 IIR 構成と分散スケーリングは単位電力チャネル `E|h|² = 1`
を保つよう設計されている。

## インターリーバ — 174 ビット上の stride-25 多項式

TX 側は channel-bit 位置 `j` に符号語 bit `INTERLEAVE[j]` を書き込み、
RX 側は逆置換を行う。表はコンパイル時生成:

```rust
const UVPACKET_INTERLEAVE: [u16; 174] = {
    let mut arr = [0u16; 174];
    let mut j = 0usize;
    while j < 174 {
        arr[j] = ((j * 7) % 174) as u16;
        j += 1;
    }
    arr
};
```

### なぜ stride 25 (逆数 7)?

求める置換 π : {0..174} → {0..174} の条件:

1. **全射 (置換)** — 各符号語 bit が channel stream にちょうど 1 回
   現れる。等価に gcd(stride, 174) = 1。
2. **離散度** — 連続符号語 bit (i, i+1) が channel 上で十分離れて
   配置されること。連続 B bit のチャネル損失が符号語上では sparse
   な損失パターンになる。
3. **逆計算が安価** — TX/RX どちらも `(stride · index) mod 174` で
   置換を計算したい。stride の mod 174 における逆元も小さい整数で
   ある必要がある。

候補 (174 = 2 · 3 · 29 と互いに素なもの):

| stride | mod 174 の逆元 | 備考                                           |
|-------:|--------------:|------------------------------------------------|
|     5  |            35  | 逆元が大きく毎回反復で高コスト                 |
|     7  |            25  | stride 25 と対称ペア                           |
|    11  |            95  | 逆元が非常に大きい                             |
|    25  |             7  | **採用** — 大 stride、小逆元                  |
|    31  |           129  | 逆元が非常に大きい                             |

stride 25 は連続符号語 bit を channel 位置で 25 離す。4-FSK
(2 bits/symbol) では約 12 シンボル相当の隔離。RX 側で deinterleave
した後、連続 `B` channel-bit 損失は符号語上で stride 7 で散らばる
(25 × 7 ≡ 1 mod 174 なので)。

### バースト損失耐性、具体的な計算

サブモードの NSPS (12 kHz 音声でのサンプル/シンボル) と fade null
持続時間 `T` ms に対して、損失 channel bit 数は:

```
B = floor(T · 12000 / NSPS) · 2     // 4-FSK は 2 bits/symbol
```

deinterleave 後、これら `B` 個の損失は 174 bit 符号語上の `B` 個の
位置に散らばる。サンプル値:

| サブモード   | fade T | NSPS | 失うシンボル | チャネル bit | 影響を受ける符号語 bit |
|--------------|-------:|-----:|-------------:|-------------:|-----------------------:|
| UvPacket150  |  50 ms |   80 |          7.5 |           14 |                     14 |
| UvPacket150  | 200 ms |   80 |           30 |           60 |                     60 |
| UvPacket300  |  50 ms |   40 |           15 |           30 |                     30 |
| UvPacket600  |  50 ms |   20 |           30 |           60 |                     60 |
| UvPacket1200 |  50 ms |   10 |           60 |          120 |                    120 |

LDPC(174, 91) のソフト判定 BP は閾値 SNR 付近でランダム 25–35
ビット消失を訂正する能力がある。14, 30 bit ケースは余裕で訂正
範囲内、60 bit はギリギリ、120 bit (UvPacket1200 50 ms fade) は
範囲外 — ただし UvPacket1200 の 50 ms fade は *フレームのほぼ全部*
(83 ms フレーム中) なので、デコード失敗が正解。

## 同期配置 — 頭 + 中央 Costas-4

`SYNC_BLOCKS` 定数は 2 個の同期ブロックを配置する:

```rust
pub const UVPACKET_SYNC_BLOCKS: [SyncBlock; 2] = [
    SyncBlock { start_symbol: 0,  pattern: &UVPACKET_SYNC_PATTERN },
    SyncBlock { start_symbol: 47, pattern: &UVPACKET_SYNC_PATTERN },
];
```

### なぜ 3 つではなく 2 つ?

3 ブロック (頭 + 中央 + 末尾) なら対称カバレッジになり、頭 + 中央
を straddle する fade に対して末尾ブロックが追加の sync vote を
提供できる。コストは +4 channel symbol で、UvPacket150 では +27 ms
フレーム時間 (+4.2%)。

現設計が 2 ブロックを選んだ理由は **短フレームでの非対称性が
重要** だから: UvPacket1200 では全 95 シンボルで 79 ms、3 ブロック
化すると 86 ms — コヒーレンス時間 1 個分に相当する。経験的には
2 ブロック配置で `tests/uvpacket_roundtrip.rs` のフェージング
シナリオは全て通過する。将来の deployment で「頭 + 中央」の
組み合わせが厳しい fading (高速移動、強反射地形) で相関を失う
ようなら、3 番目のブロックを末尾に追加するのは
`UVPACKET_SYNC_BLOCKS` の 1 行変更で済む。

### なぜ `[0, 1, 3, 2]` Costas パターン?

これは FT4 が最初の同期ブロックで使う `FT4_COSTAS_A` と同じ
パターン。長さ 4 の Costas array は研究済 — 自己相関は単一ピーク
に対して副ローブが ≤ 1、これが `coarse_sync` が利用する性質。
実績あるパターンを再利用することで自己相関特性の再導出を回避。
両ブロックでパターンを共有: 両方クリーンなら相関が同位相で
合算され、片方フェードしても独立に機能する。

## レートラダー — なぜ 1200 baud が天井?

12 kHz 音声パイプラインでの M-FSK の実用上限:

- **シンボルレート上限** = `12000 / NSPS`。NSPS = 10 で 1200 baud。
  NSPS = 5 で 2400 baud だが 4-FSK のトーン拡がりは
  `4 × 2400 = 9600 Hz`、1500 Hz 中心キャリアでは最高トーンが
  9000 Hz — 6 kHz Nyquist 超過。NSPS = 5 では aliasing で
  UvPacket2400 は信頼性が低い。
- **トーン間隔上限** = `Nyquist / NTONES` (マージン考慮)。
  4-FSK では最高トーン周波数が `中心 + 3 × Δf`。1500 Hz 中心で
  ≤ 5500 Hz 制約から Δf ≤ 1333 Hz — UvPacket1200 の 1200 Hz は
  入るが UvPacket1500 は入らない。
- **スペクトル効率上限** = M-FSK 系統。1.2 kbps 正味の天井を
  破るには変調方式そのものを変える必要がある。同 baud rate で
  PSK / QAM なら 2–6× の bits/symbol が得られるが coherent
  receiver が必要で、別プロトコル系統となる。OFDM は 3 kHz で
  ≥ 5 kbps を狙う別案 (VARA 系)。

「数 kbps」の正味レートを目指すなら別プロジェクトで以下のいずれか
が必要:

1. 広サンプルレート (24 / 48 kHz) — `mfsk-core/src/core` 配下
   のすべての DSP モジュールを触る大改修。uvpacket では未着手。
2. 高次変調 (8-PSK, 16-QAM) — coherent demodulator と完全に新しい
   TX/RX パスが必要。`core::protocol` のトレイト面に拡張が要る。

uvpacket は 12 kHz 音声パイプライン下で M-FSK が出せる実用上限の
ところに位置している。

## テスト方法論

`mfsk-core/tests/uvpacket_roundtrip.rs` は各サブモードで 4 つの
チャネルシナリオを回す:

1. **クリーンベースバンド** — pad-to-buffer、無劣化。エンコーダ・
   インターリーバ・同期配置・RX パイプライン・`PacketBytesMessage`
   の CRC-7 が byte-identical で噛み合うことの sanity check。
2. **AWGN 適度な SNR** — additive white Gaussian noise、UvPacket300
   に +5 dB、UvPacket1200 に +10 dB (BW 広めの分ノイズ積分が大きい
   ので底上げ)。LDPC が散発エラーを訂正することを確認。
3. **時間選択性 Rayleigh + AWGN** — 主役シナリオ。UvPacket300 を
   5 Hz Doppler / +12 dB SNR、UvPacket150 を 2 Hz Doppler / +10 dB
   SNR で典型的移動条件を網羅。Rayleigh エンベロープは独立な
   1 次 IIR フィルタ済 Gaussian 過程 2 本で生成、平均チャネル電力を
   1 に保つよう分散スケーリング (√2 envelope-magnitude 補正のため
   `mean_gain = 1.4`)。
4. **バーストヌル sanity** — 音声先頭 50 ms をハードゼロ化して
   頭 Costas + 数シンボル分の data を消失させる。中央 Costas +
   インターリーバがフレームを救済できることを検証。

フェージングシナリオは LCG 種で決定論的なので、テストは run 間で
再現可能。インターリーバ・同期配置・LLR deinterleave 経路を触る
PR は 4 シナリオ全てを通過させる必要がある。

## 用途と制約

uvpacket の現状は以下に向く:

- **データビーコン** — 1–10 byte ペイロード (テレメトリ、位置、
  ステータス) を適切なサブモードで定期送信。
- **短いアプリメッセージ** — `chain.rs` と組み合わせて ~512 byte
  まで (8 byte app-payload × 64 frames)。
- **マルチ局共有チャネル** — レシーバは `coarse_sync` の
  sliding-window 相関に基づくので、異なる音声中心周波数の複数
  同時送信が自動デコードされる (チャネル全体の SNR 予算次第)。

向かない用途:

- **リアルタイム音声 / 映像** — 1.2 kbps 正味では会話用 codec に
  はるかに足りない。
- **大量ファイル転送** — 1.2 kbps では 1 MB が ~111 分。VARA HF
  または 9k6 packet を使うべき。
- **非フラットフェージング** — 長遅延広がり (HF skywave 電離層
  multipath) は範囲外。FST4 / WSPR / Q65 のより長い同期構造が
  そこでは正解。
