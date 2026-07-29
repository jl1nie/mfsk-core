# uvpacket — 応用例: NFM 音声チャンネル向けパケットプロトコル

> **English:** [UVPACKET.md](UVPACKET.md)

`uvpacket` は `mfsk-core` の FEC 基盤（`Ldpc240_101`、BP、OSD-2/3）
を WSJT-X 系の外で再利用する **応用例** として in-tree に置かれて
いるモジュールです。WSJT-X 系のメンバーでは**ありません**。設計
対象は別 — 狭帯域 FM 音声チャンネル（HT/モバイル、~3 kHz 音声
帯域）でのプライベートグループ向けアマチュア無線メッセージング
（署名付き QSL 交換、短文、位置レポート）です。

このドキュメントでは設計上の選択、特性測定結果、既知の modem
実装損失をまとめます。API は in-source rustdoc を参照。

## 0. なぜこの応用例があるのか — Q65 と対をなす境界 probe

`uvpacket` は単独の応用例ではなく、`mfsk-core` の trait 抽象の
**境界を意図的に外側から probe する** ための実験です。0.4.0 で
shipped した Q65 ファミリ拡張と対になる: Q65 は trait が
WSJT 系の内側でどこまで素直に伸びるかを確かめる **positive probe**、
`uvpacket` は WSJT 系の外に出ると trait がどこで自然に途切れるかを
確かめる **negative probe**。両方の結果が揃ってはじめて trait 設計
の妥当性が立体的に裏付けられる、という発想です。

### Q65 で確認できたこと (positive)

`Protocol` / `ModulationParams` / `FrameLayout` / `FecCodec` /
`MessageCodec` の **全層** が、以下を含めてもクリーンに乗りました:

- 非バイナリ符号 (QRA over GF(2⁶)) を `FecCodec` の bit 単位 API に
  シンボル変換でラップする。
- 6 サブモードを単一の `q65_submode!` マクロから生やし、すべて
  `tests/protocol_invariants.rs` を通る。
- AWGN / AP-hint / fast-fading / AP-list の 4 並列デコード戦略が
  `decode_at_for::<P>` の generic コードを共有する。

これは「trait が WSJT 系の自然な拡張に対しては強い」という positive
な示唆です。

### `uvpacket` で確認したかったこと (negative)

WSJT 系の外側に意図的に出ます:

- 単一搬送波 **π/4-DQPSK + LMS イコライザ** (M-ary tone FSK ではない)
- **バイトパイプ API** (構造化 callsign + grid メッセージではない)
- 自前のヘッダ LDPC ブロック + **可変長バースト** (固定スロットではない)
- 4 種の **127 chip BPSK プリアンブル** (Costas tone-index ではない)

仮説: 「ここで trait が破綻するなら WSJT 系内部では問題にならない
が trait の表現力に穴がある。逆に綺麗に剥離するなら、抽象が
WSJT 系に対して適切な scope を持っている」。

### 観測された境界

| 項目 | 結果 |
|---|---|
| `Ldpc240_101` 親コード + BP / OSD-2/3 | **そのまま借りた** |
| テストチャンネル基盤 (AWGN / Rayleigh / SSB realistic / FM realistic) | **そのまま借りた** |
| DSP 部品 (RRC、相関器、LMS 解) | **そのまま借りた** |
| `MessageCodec` | **意図的に bypass** (バイトパイプ — 構造化 codec 不要) |
| `WsjtApCompatible` | **意図的に bypass** (uvpacket には AP の概念がない) |
| `Protocol::ID` / `ModulationParams` 定数 | **decorative** (自前 TX/RX パイプラインからは参照されない) |
| 汎用 `engine::pipeline::decode_frame::<P>` | **使っていない** (専用 `uvpacket::rx`) |

### 結論 — trait は WSJT 系のために意図的に specific

`mfsk-core` の trait 抽象は **WSJT 系プロトコルファミリ向けに
最適化された具体的な抽象**であって、汎用 PHY フレームワークでは
ありません。これは正しい設計判断です:

- 抽象を WSJT 系に絞ったからこそ、`Protocol` 1 つの `<P>` パラメータ
  から `coarse_sync::<P>` / `compute_llr::<P>` / `decode_frame::<P>`
  が monomorphize でき、Q65 のような新モードがゼロコストで生える。
- 汎用 PHY framework として作っていたら、`SYNC_MODE` を「Costas
  でも m-sequence でも何でも」と一般化する代償に trait 階層が
  階段状に増え、WSJT 系のコードが冗長になっていたはず。

`uvpacket` はその境界の外側で WSJT 用 trait 抽象が自然に剥離する
ことを実証していて、抽象の**不足**を示しているのではなく、抽象の
**適切な scope を確認している**。WSJT 系外のプロトコルに対しては
`mfsk-core` を「FEC + DSP + チャンネルテスト基盤を持つ薄い
ライブラリ」として、自前の TX/RX パイプラインを書く側から使うのが
自然な使い方です。`uvpacket` はその使い方の働く実例でもあります。

以下、§1 以降は `uvpacket` 単体の設計と特性測定を扱います。trait
抽象との関係についての要約は LIBRARY.ja.md §10.1 にも 1 段落あり、
そちらと本節は対のペアです。

## 1. スコープ

### 1.1 これが何か

NFM 音声帯域に収まる **4 モードのパケット modem**。FST4 由来の
hand-tuned irregular LDPC を親コードとして使用。**両端で同じ
ソフトウェア**を動かすプライベートグループ向け — 公的な互換
プロトコル置換ではなく、既存 TNC とも互換性なし。

### 1.2 これは「ない」もの

- 相互運用モードではない。標準化なし、TNC サポートなし。
- 音声モードではない。データ専用。
- 広帯域モードではない。NFM 音声 (~3 kHz) に収まり、ネット
  スループット 1–1.8 kbps。M17 / D-STAR / DMR / VARA FM とは
  別の土俵。
- 弱信号モードではない。FM 閾値（CNR ≥ +9–10 dB）より上の運用
  envelope を狙うもので、それ以下では FM 検波系のどんな modem
  でも崩壊する不可避フロアがチャンネル側にある。

### 1.3 立ち位置

3 kHz 音声帯域での U/VHF 私的グループメッセージング (~1 kbps
クラス) における事実ベースのピア比較:

| | チャンネル | FEC | Net bps | 90 % PER 閾値 (AWGN) | OSS |
|---|---|---|---:|---:|:-:|
| AX.25 / AFSK 1200 | NFM | なし | 1200 | +10 dB SNR_3kHz | ✓ |
| PSK31 | SSB | なし | ~50 | −10 dB SNR_2.5kHz | ✓ |
| Olivia 4/250 | SSB | conv + interleave | ~50 | −13 dB SNR_500Hz | ✓ |
| **uvpacket UltraRobust** | **NFM または SSB** | **LDPC + OSD、半 baud** | **504** | **−3.7 dB SNR_3kHz** | ✓ |
| uvpacket Robust | NFM または SSB | LDPC + OSD | 1008 | +1.3 dB SNR_3kHz | ✓ |
| uvpacket Standard | NFM または SSB | LDPC + OSD | 1200 | +4.0 dB SNR_3kHz | ✓ |
| uvpacket Express | NFM または SSB | LDPC + OSD | 1800 | +7.8 dB SNR_3kHz | ✓ |
| M17 4-FSK | 9 kHz | conv | 4800 | +5–7 dB SNR | ✓ |
| VARA FM | 12.5 kHz | プロプラ | ~25000 | +10 dB SNR | ✗ |
| VARA HF | 2.4 kHz SSB | プロプラ | 5000–25000 | 条件次第 | ✗ |
| D-STAR DV | 6.25 kHz | Golay | 4800 voice + 1200 data | ~+10 dB CNR | 部分的 |
| DMR / NXDN | 6.25–12.5 kHz | BCH | 4-FSK voice + data | ~+7–8 dB CNR | (商用) |

設計空間で最も近い peer は AX.25 と PSK31。太字行から右側はすべて
別の設計空間 (より広い channel、voice 主体、または proprietary)。

NFM での AX.25 比: スループット 16 % 減と引き換えに ~14 dB 良い
SNR 閾値、本格 FEC とブロックインターリーバーで fade burst 耐性。
SSB での PSK31 比: 同じ passband で 20 倍速いが ~6 dB 閾値で
不利 — 同パスバンドでの異なるニッチ。

uvpacket は VARA / M17 / D-STAR クラスの広帯域や音声併用プロト
コルとは速度競争をしない。FT8 / JS8 / Olivia 等の極弱信号モード
とも競合しない (それらは桁違いに遅い)。

成果物: **3 kHz NFM / SSB voice passband に収まり**、両方で動作
する、サブ秒バースト + 4段階の機会的スループットラダーを持つ
オープンソース FEC 符号化パケット modem。

## 2. 設計

### 2.1 変調

単一搬送波 **π/4-shifted DQPSK**、ルートレイズドコサインパルス
(α = 0.35、span 6 sym)、音声中心 1500 Hz、サンプリング 12 kHz。
シンボルレートは **1200 baud**（Robust / Standard / Express）
または **600 baud**（UltraRobust、§2.3 参照）。情報ビットは
コンステレーション回転 Δφ ∈ {±π/4, ±3π/4} に差動エンコードされ、
RX は 1 シンボル共役積 → −π/4 derot → Gray demap で復元。

π/4-DQPSK + 差動復調の組合せは絶対搬送波位相基準を不要にする —
シンボル周期に対して遅い位相劣化（LO walk、AFC 範囲内のクラリ
ファイア offset、群遅延傾斜）はすべて 1 シンボル積で打ち消される。
これが 0.3 のコヒーレント QPSK パイプライン（AWGN bench は通った
が over-the-air で死んだ）からの主要な変更点。

### 2.2 プリアンブル — モード符号化 m-sequence + イコライザ fit

フレーム頭は **127 chip BPSK m-sequence**（Fibonacci LFSR、長さ 7、
[`Mode`] が 4 つの原始多項式から 1 つを選択）。127 chip × 1 sym/chip
= 1200 baud で 106 ms（UltraRobust 600 baud では 212 ms）。多項式の
選択がプリアンブル自身に**モードを符号化**しており、0.3 時代の
LDPC レイアウト総当たり試行を置き換える — RX は 4 本の相互相関を
取って勝者を選ぶだけで、デコードコストが mode 混乱に依存せず
1+n_blocks に確定する。

長さ 127 m-sequence の巡回自己相関サイドローブは 1/127 ≈ −22 dB
で抑えられ、これは**9 タップ T 間隔 LMS イコライザ**を既知プリ
アンブルに対する閉形式最小二乗で fit するのにも十分綺麗。残差
回転と pre-rotation timing 補正も同じプリアンブル区間で取られる。
ペイロードに**パイロットシンボルは存在しない** — イコライザと
1 シンボル差動復調がフレーム全期間の残留チャンネルを吸収する。

### 2.3 FEC

FST4 由来の [`Ldpc240_101`] をレート 0.42 の親コードとして再利用
（情報 101 bit → channel 240 bit / block）。4 つのサブモードは
**kSR-greedy puncture set 選択**（Ha–McLaughlin）で 139 parity
bit にパンクチャを適用:

| サブモード | Baud | rate | パンクチャ | Net bps | 想定姿勢 |
|---|---:|---:|---:|---:|---|
| UltraRobust | 600 | 0.42 | 0 % | 504 | 弱信号 / マラソン QSL 用 (半 baud) |
| Robust | 1200 | 0.42 | 0 % | 1008 | フル baud での最大マージン |
| Standard | 1200 | 0.50 | 30 % | 1200 | フェージング有の典型的 NFM |
| Express | 1200 | 0.75 | 76 % | 1800 | 強信号での最速（OSD-3 必須） |

UltraRobust は Robust と同 FEC レートながらシンボル周期を倍にする
ため、シンボルあたりエネルギー倍 + フェージング平均化向上 +
タップ遅延マルチパスがシンボル周期に対して相対的に短くなる。
4 段モードラダーで end-to-end ~12 dB SNR_3kHz 範囲をカバー (§3)。

kSR-greedy は深い rate で uniform-spread に対し ~1–3 dB の Eb/N0
gain を出し、これが Express をそもそも成立させている。

**専用ヘッダ LDPC ブロック**（Ldpc240_101 unpunctured、Robust /
UltraRobust と同レート）が 4 byte フレームヘッダをペイロードと
分離して運ぶ — ヘッダ復元はペイロードのデコード順や puncture 深度
に依存しない。

### 2.4 フレーム構造

- 可変長: フレームあたり 1–32 LDPC ブロック。
- **ヘッダ LDPC ブロック**（Ldpc240_101 unpunctured）が 4 byte
  フレームヘッダを運ぶ。モードは**プリアンブル多項式で識別**
  されるためヘッダフィールドには含まれない — ヘッダは
  block_count (5b) + app_type (4b) + sequence (5b) + reserved (2b)
  + CRC-16 (16b)。
- 各ペイロード LDPC ブロックは 96 情報 bit (12 byte) を運び、
  FEC の 101 bit 入力にパディング。
- フレーム内全 codeword をまたぐ **ブロックインターリーバー**が
  fade burst の erasure を全 codeword に拡散。

### 2.5 アプリケーション API

バイトパイプ — `mfsk-core` の `MessageCodec` をバイパス。呼び出し
側は raw bytes と 4 bit `app_type` タグを渡す。modem は中身を解釈
しない。推奨割り当て:

| `app_type` | 用途 |
|---:|---|
| 0 | raw / 実験 |
| 1 | 署名付き QSL 交換 |
| 2 | 位置ビーコン |
| 3 | 短文 |
| 4 | ARQ ACK |
| 5–15 | ユーザー定義 |

公開エントリポイント (詳細は in-source rustdoc):

- `tx::encode(header, payload, audio_centre_hz) -> Result<Vec<f32>, PackError>`
  — Vec を確保するシンプルなラッパ。
- `tx::encode_into(out, header, payload, audio_centre_hz) -> Result<(), PackError>`
  + `tx::encode_output_len(mode, n_payload_blocks)` — 0.4.1 (embedded
  ポート) で追加された caller-buffer TX。I2S DMA 等で per-burst
  Vec 確保を避けたい用途向け。
- `rx::decode_known_layout(audio, sample_offset, audio_centre_hz, mode, &fec_opts)`
  — 基本デコード。`default_fec_opts()` で OSD-2 / bp_max_iter = 50、
  独自の `FecOpts` を組めば OSD-3 や caller-side AP マスクも可能。
- `rx::decode_known_layout_with_afc(.., &afc_opts)` — 上記に
  `±afc_opts.search_hz` (デフォルト ±200 Hz) の AFC sweep を前置。
- `rx::decode(audio, audio_centre_hz) -> Vec<DecodedFrame>` —
  自動検出: passband を走査し、4 種の preamble バリアントいずれかの
  sync ピークごとに、勝った preamble が示すモードでデコード。
- `rx::decode_multichannel(audio, &mc_opts, &fec_opts)` /
  `rx::measure_slot_energies(audio, &mc_opts, slot_spacing_hz)` —
  passband 全体のスキャン + LBT 用スロットエネルギー survey
  (§3.10 参照)。

## 3. 特性測定

### 3.1 モード位置付けサマリ

`tests/uvpacket_per_modes_sweep.rs` (`#[ignore]`、
`cargo test --release --test uvpacket_per_modes_sweep <name>
-- --ignored --nocapture` で実行)、cell ごとに 30 trials、
4 ブロックフレーム、16 byte ペイロード、π/4-DQPSK + LMS
イコライザ + OSD-2。

**90 % PER 閾値 (≥ 27/30 復号)、Eb/N0_info / SNR_3kHz dB:**

| モード (net bps) | AWGN | Rayleigh fd=5 Hz | SSB realistic | FM realistic | Multipath 3-tap |
|---|---:|---:|---:|---:|---:|
| **UltraRobust** (504) | **+4 / −3.7** | **+8 / +0.3** | **+4 / −3.7** | **+6 / −1.7** | **+6 / −1.7** |
| Robust (1008) | +6 / +1.3 | +12 / +7.3 | +8 / +3.3 | +10 / +5.3 | +8 / +3.3 |
| Standard (1200) | +8 / +4.0 | +12 / +8.0 | +8 / +4.0 | +10 / +6.0 | +10 / +6.0 |
| Express (1800) | +10 / +7.8 | +20 / +17.8 | >+15 / >+12.8 | +20 / +17.8 | **fail** |

(SNR_3kHz = Eb/N0_info + 10·log₁₀(R_info / 3000); モード別
R_info: 504 / 1008 / 1200 / 1800 bps → −7.74 / −4.74 / −3.98 /
−2.22 dB)

UltraRobust は全フェージングチャンネル (Rayleigh, SSB, FM) で
Robust に対し均一に 4 dB マージン、AWGN / multipath で 2 dB
マージンを持つ。Express は +20 dB Eb/N0 でもマルチタップ
multipath で崩壊する — イコライザの 9 タップは 1200 baud で
~7.5 ms しかカバーできず 15 ms タップを解像できない。一方
600 baud の UltraRobust では ~13 ms 相当の T 間隔リーチが
ある。

### 3.2 LDPC レイヤー（modem バイパス参照）

`tests/uvpacket_ldpc_direct.rs` は Gaussian noise の LLR を直接
LDPC デコーダに食わせる（channel bit 単位の `Eb/N0_info` で校正）。
これで FEC を modem から分離し、modem end-to-end が目指す**理論的
上限**を出す。

50 % PER 閾値: UltraRobust / Robust ≈ +0.5 dB（同 FEC）、
Standard ≈ +0.7 dB、Express ≈ +1.5 dB。親コードの設計レートは
0.42 なので Robust / UltraRobust が FEC レイヤーで ~1 dB のリード
を保つ。§3.1 の π/4-DQPSK end-to-end 閾値はこの LDPC のみの上限
から ~3 dB 上に位置する — この 3 dB が差動復調の**非コヒーレント
対コヒーレント**の不可避ギャップで、over-the-air の位相劣化
スタックを生き延びるために払った代価 (§4)。

### 3.3 AWGN sweep

```text
mode         Eb/N0 (dB)  -2   0   2   4   6   8  10
─────────────────────────────────────────────────────
UltraRobust               0   0   6  29  30  30  30
Robust                    0   0   0   1  25  30  30
Standard                  0   0   0   0  19  30  30
Express                   0   0   0   0   0  21  30
```

90 % PER 閾値: UltraRobust ≈ +4 dB、Robust ≈ +6 dB、
Standard ≈ +8 dB、Express ≈ +10 dB。教科書通りの rate ordering
(低 rate ほど低閾値) + UltraRobust の 2 dB 半 baud ボーナス。

### 3.4 Rayleigh フラットフェージング

```text
mode         fd (Hz)  +4   +8  +12  +16  +20  +25   (Eb/N0_info dB)
────────────────────────────────────────────────────────
UltraRobust    1       4   22   29   30   30   30
UltraRobust    5       3   29   30   30   30   30
UltraRobust   10       4   29   30   30   30   30
Robust         1       0   10   26   30   30   30
Robust         5       0    7   29   30   30   30
Robust        10       0   10   30   30   30   30
Standard       1       1   11   24   30   30   30
Standard       5       0    6   26   30   30   30
Standard      10       0    7   30   30   30   30
Express        1       0    3   10   20   27   28
Express        5       0    0    3   18   29   30
Express       10       0    0    2   22   28   29
```

≥ 90 % PER 閾値: UltraRobust ≈ +8 dB（全 Doppler）、
Robust / Standard ≈ +12 dB、Express ≈ +20 dB。Doppler 依存性は
下位 3 モードでは穏やか（差動復調が遅い位相ドリフトを吸収）、
Express だけが 1 Hz vs 10 Hz で閾値が動く。

### 3.5 SSB realistic — クラリファイア offset + LO walk + 軽 multipath

チャンネル: BPF (300, 2700) Hz with 100 Hz transition、
クラリファイア offset 100 Hz (AFC 範囲内)、LO 位相 walk
2 rad/√s、5 ms multipath タップ 1 本 −10 dB。

```text
mode         Eb/N0 (dB)  +4   +6   +8  +10  +12  +15
─────────────────────────────────────────────────────
UltraRobust              21   30   30   30   30   30
Robust                    0    7   28   30   30   30
Standard                  0    1   23   30   30   30
Express                   0    0    1    7   17   27
```

UltraRobust は AWGN 閾値 (+4 dB) と**ほぼ同等**で動作 — 半 baud
シンボル周期がイコライザに multipath タップを吸収する時間を
与え、差動復調が LO walk に不感である。

### 3.6 FM realistic — de-emphasis + 識別器ドリフト + Rician

チャンネル: 75 µs de-emphasis、識別器 DC ドリフト 50 Hz、LO walk
1 rad/√s、Rician K = 10 dB、5 ms multipath タップ 1 本 −10 dB。

```text
mode         Eb/N0 (dB)  +6   +8  +10  +12  +15  +20
─────────────────────────────────────────────────────
UltraRobust              27   30   30   30   30   30
Robust                    0    9   28   30   30   30
Standard                  0    2   24   30   30   30
Express                   0    0    1    5   17   26
```

UltraRobust が再び Robust に対し ~4 dB マージン。Express は
de-emphasis 傾斜 × multipath で FM ではほぼ実用に耐えない。

### 3.7 純マルチタップ multipath (3 + 8 + 15 ms)

その他の劣化なし、AWGN のみのマルチパス耐性試験。イコライザの
リーチを単離する。

```text
mode         Eb/N0 (dB)  +6   +8  +10  +12  +15  +20
─────────────────────────────────────────────────────
UltraRobust              30   30   30   30   30   30
Robust                    0   20   30   30   30   30
Standard                  0   19   28   30   30   30
Express                   0    0    0    0    6    8
```

UltraRobust は +6 dB Eb/N0 でも**フロア張り付き** — 600 baud
シンボル周期 (~1.67 ms) で 9 タップイコライザが ~13 ms (T 間隔)
をカバーでき、15 ms タップ末尾も余裕で含む。Express は完全崩壊:
1200 baud × 9 タップで ~7.5 ms しかカバーできず長いタップを
解像できない。

### 3.8 FM 閾値フロア — そして modem 実装損失が運用上不可視な理由

modem は FM 検波の上に乗る。CNR ≈ +9–10 dB を下回ると FM
discriminator 出力はインパルスノイズ支配となり、**どんな**
audio-domain modem も壊滅的に失敗する。上の音声領域 Eb/N0 数値
は FM 閾値より上でのみ意味を持つ。

**FM 閾値の地点**で、検波後の音声 SNR (3 kHz パスバンド換算) は
おおよそ `CNR_threshold + FM_SNR_improvement ≈ +9 +
10·log₁₀(B_IF/B_audio · 3) ≈ +9 + 11 ≈ +20 dB SNR_3kHz`。

uvpacket UltraRobust の 90 % PER 閾値 (+4 dB Eb/N0_info) を同じ
単位に換算:

```text
SNR_3kHz_UltraRobust = +4 + 10·log₁₀(504 / 3000) = −3.7 dB
```

FM 閾値フロアから UltraRobust modem 閾値までのマージン:
**~+24 dB**。残り実装損失は運用上**不可視** — チャンネル側
の不可避 CNR フロアより遥か下で、そこではどんな audio modem も
復号しない。

FM 閾値が NFM 音声チャンネルの拘束条件。

### 3.9 SSB 互換性 — と AFC

modem は audio 領域の π/4-DQPSK + RRC プロセッサ（信号は
α = 0.35 で 1500 Hz 中心の ~1600 Hz、典型的な SSB パスバンドに
余裕で収まる）。SSB では FM-threshold floor が消えるので modem
は本来の限界 ~−3.7 dB SNR_3kHz UltraRobust 閾値で動作可能 —
特に HF で有用な weak-signal data envelope。

**AFC エントリポイント**。SSB 用途には
[`decode_known_layout_with_afc(audio, .., &AfcOpts)`](https://docs.rs/mfsk-core/latest/mfsk_core/uvpacket/rx/fn.decode_known_layout_with_afc.html)
を使う。デフォルトの `decode_known_layout` は
`audio_centre_hz` を厳密に既知と仮定する (NFM では TX/RX が同じ
audio centre を共有するので妥当)。

AFC アルゴリズムは `audio_centre_hz + Δf_test` を 25 Hz 刻みで
`[−search_hz, +search_hz]` (デフォルト ±200 Hz) を sweep し、
各候補で matched filter を走らせ、preamble correlation の
ピークを与える Δf を選ぶ。3点 coarse-grid magnitude の放物線
リファインで grid 間隔の数分の1 まで Δf を絞る。コストは
single-decode の ~17倍（release で ~50–100 ms）。下流の LMS
位相フィットが grid 残差を吸収する。

素朴な FFT-over-chip-rate アプローチ（軽量だが不正確）は失敗:
integer-sample preamble correlator が拾う `best_off` 自身が
`sinc(δ · 31 / 1200)` でロールオフするため、`|δ| ≳ 20 Hz` で
noise sample に落ちる。周波数グリッド探索は correlator
magnitude 自身が正しい Δf でピークを作るのでこれを回避できる。

NFM 利用者は `decode_known_layout` をそのまま使ってよい (静的
VFO チャンネルでは AFC は単なるオーバーヘッド)。

### 3.10 マルチチャンネル SSB + slotted-ALOHA TX

uvpacket 信号 1 つの占有帯域は `R_s · (1+α) = 1200 · 1.5 =
1800 Hz` (RRC ロールオフ込みの全幅、−3 dB 主ローブは ~600 Hz)。
隣接スロット間隔 < −20 dB の干渉抑制で **1200 Hz**: この間隔で
一方の信号スペクトラムが対向の RRC ロールオフのゼロ点に乗る。
2.4 kHz SSB パスバンドには **2 つ**の uvpacket フレームが同時
収納可能 (典型: 800 Hz と 2000 Hz の audio 中心)。

`mfsk-core::uvpacket::rx` は stateless な primitives を 2 つ提供:

```rust
# #[cfg(feature = "uvpacket")] {
use mfsk_core::uvpacket::framing::FrameHeader;
use mfsk_core::uvpacket::rx::{
    MultiChannelOpts, decode_multichannel, default_fec_opts, measure_slot_energies,
};
use mfsk_core::uvpacket::{Mode, tx};

// RX: passband 全体で全フレーム復号、検出された audio centre と
// 一緒に返す
let header = FrameHeader { mode: Mode::Robust, block_count: 4, app_type: 0, sequence: 1 };
let payload: Vec<u8> = (0..20).map(|i| (i ^ 0x5A) as u8).collect();
let audio = tx::encode(&header, &payload, /* audio_centre_hz */ 800.0).unwrap();

let mc_opts = MultiChannelOpts::default();
let fec_opts = default_fec_opts();
let frames = decode_multichannel(&audio, &mc_opts, &fec_opts);
assert!(!frames.is_empty(), "ラウンドトリップは復号できるはず");
for (centre_hz, frame) in &frames {
    println!("{:7.1} Hz  {} バイトのペイロード", centre_hz, frame.payload.len());
}

// TX 側 LBT: スロットごとの平均 MF magnitude survey
let energies = measure_slot_energies(&audio, &mc_opts, /*slot_spacing_hz*/ 1200.0);
for e in &energies {
    println!("{:7.1} Hz  mean|MF|^2={:.4}", e.audio_centre_hz, e.mean_mf_magnitude);
}
# }
```

`decode_multichannel` は coarse-grid 周波数 sweep (デフォルト
25 Hz 刻み、300–2700 Hz)、各候補で preamble correlation peak、
周波数軸 NMS (`nms_radius_hz` デフォルト 600 Hz = half slot
spacing) で隣接 grid duplicate を除去、各 survivor を
`decode_known_layout` で復号 (内部 AFC 不要 — coarse
grid の ≤ 12.5 Hz 残差は LMS 位相フィットが吸収)。

`measure_slot_energies` は各 `slot_spacing_hz` 間隔のスロット
中心で matched filter |output|² 平均を返す。policy-free —
呼び出し側が独自の閾値で free vs busy を判定し、free slot から
uniform random で 1 つ選んで TX。

TX 側は既存の `tx::encode(&header, &payload, audio_centre_hz)`
そのまま。動作: 各 TX で LBT → free slot から random pick → 送信
→ アプリ層 ARQ ACK 待機。衝突したら新たな random pick で retry。
**音声周波数軸の slotted ALOHA**。

これはアマチュア無線の自然な所作「周波数をワッチして空いてる
ところで出る」を modem 内で formalize したもの。CSMA/**CD** 本来
は半二重 SSB に適用不能 (送信中の信頼ある受信ができない)。
slotted ALOHA + LBT + アプリ層 ARQ で同等の挙動をはるかに
少ないメカニズムで実現。

mfsk-core は RNG 依存なし、state なし — アプリが乱数源
(`rand::Rng`, browser `crypto.getRandomValues`, …) と ARQ /
retry ポリシーを持つ。

実測: 800/2000 Hz 中心の 2 同時フレームは検出 audio centre 真値
±50 Hz 以内で復号; +8 dB Eb/N0_info AWGN で同様; busy slot 1 つ
の survey で busy mag > free mag × 5。

## 4. modem 実装損失

§3.2 の LDPC のみ閾値と §3.3 の π/4-DQPSK end-to-end 閾値の
ギャップは AWGN で **~3 dB** — これは差動復調の不可避
**非コヒーレント対コヒーレント** ギャップであり、エンジニアリング
スラックではない。0.4 の再設計はこのギャップを、0.3 のコヒー
レント QPSK パイプラインを破壊した over-the-air 位相劣化スタック
を生き延びる代価として受け入れた。

現在の rx 実装:

- **127 chip BPSK プリアンブル × 4 多項式 相互相関** — モード
  (UltraRobust / Robust / Standard / Express) ごとに 1 本の相互
  相関。勝った多項式がモードを識別するため、LDPC レイアウトの
  総当たりが不要になり、デコードコストは mode 混乱に依存せず
  1+n_blocks に確定する。
- **9 タップ T 間隔 LMS イコライザ**を既知プリアンブルに対する
  閉形式最小二乗で fit (反復適応なし)。リーチは 1200 baud で
  ~7.5 ms、600 baud で ~13 ms — 後者は 1200 baud モードでは
  解像できない典型的マルチタップ multipath をカバーする。
- プリアンブルの複素符号付き平均から**残差回転推定**、−π/4
  derot 前に適用。シンボル単位 PLL なしで AFC 範囲内のクラリ
  ファイア offset を吸収。
- **1 シンボル差動復調**: r_diff[n] = r[n] · r[n−1]*。−π/4 derot
  後に Gray demap で bit 復元。ペイロードに pilot 挿入なし、
  搬送波位相追跡器なし。
- 差動サンプル上の magnitude ベース σ²_n 推定器による
  **σ-aware LLR スケーリング**。
- デフォルトは **OSD-2** (`default_fec_opts()`)。`decode_known_layout`
  は `&FecOpts` を受け取るので、OSD-3 を選びたい呼び出し側
  （~30× 遅いが高 rate モードの閾値近傍で ~10–15 % PER 改善）や
  caller-side AP マスクに対応する。
- **専用ヘッダ LDPC ブロック** (Ldpc240_101 unpunctured) で、
  ヘッダ復元はペイロード puncture 深度に依存しない。

これらの選択は ~3 dB の AWGN ヘッドルームを「絶対位相基準なしで
動作可能」に交換するものである。§3.4–3.7 で測定した全フェー
ジング・位相 walk チャンネルでこのトレードは正解 — 0.3 のコヒー
レントパイプラインはこれらどのチャンネルもフィールドリアルな
設定では生き延びなかった。

## 5. 変調ピボットの経緯

- **0.3.0**: 正直な AFSK1200 / AX.25 比較で「クリーンチャンネルで
  ~5–10 倍速い」と判明し、署名付き QSL ペイロードに対する設計を
  放棄。214 B ペイロード (JSON 126 B + Base64 署名 88 B) のオンエア
  時間:

  | 伝送              | チャンネル速度 | 正味速度   | 214 B 送出時間 |
  |-------------------|------------:|----------:|--------------:|
  | AX.25 / AFSK 1200 |     1200 bps | ~1100 bps | ~2 秒 + TXDELAY 0.5 秒 |
  | UvPacket300       |      600 bps |  ~200 bps | ~10 秒 |
  | UvPacket150       |      300 bps |  ~100 bps | ~18 秒 |

  AFSK 1200 が運べないチャンネル (Rayleigh フェード / 山岳 / 携行
  弱 SSB) でのみオンエア時間のペナルティが見合う。「AFSK 置換」
  というフレーミング自体が誤りだった。
- **0.3.1 Phase 1**: h = 0.5 の 4-GFSK。Phase 2 で `sinc(0.5)
  ≈ 0.637` が非コヒーレント検波下で隣接 tone を非直交にする
  ことが判明（教科書条件は h ≥ 1）。
- **0.3.1 Phase 2 → 0.3.3**: コヒーレント QPSK + RRC + LMS
  位相追跡器にピボット。AWGN / Rayleigh sim はベンチパスしたが、
  AFC、coherence-ratio gate、1-shot AFC を繰り返し追加しても
  SSB / FM 音声経路の over-the-air で死んだ。
- **0.4.0**: コヒーレント QPSK を **π/4-DQPSK + LMS イコライザ
  + 4 変種 127 chip プリアンブル + 専用ヘッダブロック +
  UltraRobust 半 baud モード** に置換。3 dB の非コヒーレント
  ギャップは構造的だが、置換した 5–10 dB の位相劣化損失は遥か
  に大きい。経緯は `docs/0.3.1_PLAN.md` と 0.4 直前のプランファイル。

`tests/common/channel.rs` の σ 公式も Phase 2'a で per-burst の
測定信号電力を取るよう再校正したので、表示 Eb/N0_info は変調間
で比較可能。

## 6. 音声サンプル

リポジトリの `audio_samples/uvpacket/` に耳での確認用 WAV を
配置。すべて 12 kHz mono 16-bit PCM、200 ms の前後無音付き:

| ファイル | モード | チャンネル | 復号 |
|---|---|---|:-:|
| `uv_robust_clean.wav` | Robust, 4 blocks, 20 B | clean | ✓ |
| `uv_robust_awgn_+8db.wav` | Robust | AWGN +8 dB Eb/N0 | ✓ |
| `uv_robust_awgn_+4db.wav` | Robust | AWGN +4 dB Eb/N0 | ✓ (LMS 後 97% per-frame) |
| `uv_robust_awgn_+2db.wav` | Robust | AWGN +2 dB Eb/N0 | ✓ (53 % per-frame 統計; この seed は sub-sample timing で OK 側) |
| `uv_robust_rayleigh_5hz_+15db.wav` | Robust | 5 Hz Rayleigh, +15 dB | ✓ |
| `uv_express_clean.wav` | Express, 4 blocks, 20 B | clean | ✓ |

再生成は:

```sh
cargo run --release --features uvpacket --example uvpacket_samples
```

クリーンな Robust バーストは ~440 ms、Express は ~270 ms。可聴
キャラクターは「狭帯域データバズ」 — RRC pulse が各 QPSK シンボル
を複数 tone にまたがって広げるので、スペクトラムは
`[1500 − 600, 1500 + 600] Hz` でほぼ平坦、レイズドコサインの
shoulder 付き。

## 7. 実装ポインタ

| Layer | File |
|---|---|
| Protocol ZST / サブモードパラメータ | [`mfsk-core/src/uvpacket/protocol.rs`](../../mfsk-core/src/uvpacket/protocol.rs) |
| フレームヘッダ + CRC + bit packing | [`mfsk-core/src/uvpacket/framing.rs`](../../mfsk-core/src/uvpacket/framing.rs) |
| Puncture sets (kSR-greedy) | [`mfsk-core/src/uvpacket/puncture.rs`](../../mfsk-core/src/uvpacket/puncture.rs) |
| ブロックインターリーバー | [`mfsk-core/src/uvpacket/interleaver.rs`](../../mfsk-core/src/uvpacket/interleaver.rs) |
| プリアンブル多項式 (4 変種) | [`mfsk-core/src/uvpacket/sync_pattern.rs`](../../mfsk-core/src/uvpacket/sync_pattern.rs) |
| TX (bytes → audio) | [`mfsk-core/src/uvpacket/tx.rs`](../../mfsk-core/src/uvpacket/tx.rs) |
| RX (audio → bytes)、イコライザ | [`mfsk-core/src/uvpacket/rx.rs`](../../mfsk-core/src/uvpacket/rx.rs) |
| AWGN + Rayleigh ハーネス | [`mfsk-core/tests/common/channel.rs`](../../mfsk-core/tests/common/channel.rs) |
| SSB / FM realistic チャンネル sim | [`mfsk-core/tests/common/air_channel.rs`](../../mfsk-core/tests/common/air_channel.rs) |
| LDPC のみ sweep (modem バイパス) | [`mfsk-core/tests/uvpacket_ldpc_direct.rs`](../../mfsk-core/tests/uvpacket_ldpc_direct.rs) |
| Modem TX/RX 診断 | [`mfsk-core/tests/uvpacket_modem_diag.rs`](../../mfsk-core/tests/uvpacket_modem_diag.rs) |
| 4 モード × 5 チャンネル PER sweep | [`mfsk-core/tests/uvpacket_per_modes_sweep.rs`](../../mfsk-core/tests/uvpacket_per_modes_sweep.rs) |

## 8. ライセンス

GPL-3.0-or-later、`mfsk-core` の他と同じ。LDPC 親コードは WSJT-X
(`lib/fst4/`) からの派生。
