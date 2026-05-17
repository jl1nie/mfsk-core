# M5StickS3 FT8 controller — ユーザーマニュアル

M5StickS3 ハードウェア上に構築した手持ち式 FT8 デコーダ + QSO
コントローラ。Xtensa LX7 上で `mfsk-core` Rust デコーダを実行。
IC-705 (または互換 BLE CI-V / 音声ケーブル対応無線機) と連携し、
スロット駆動デコード + 自動 QSO シーケンスを行う。

本マニュアルは日常運用 (build / flash / 設定 / 操作) を対象とする。
内部アーキテクチャ (FFT パイプライン、Goertzel カーネル、同期
アルゴリズム) は `mfsk-core/` のソースを参照。

---

## 1. 何ができるか

同一ファームウェアが **7 つの mode** のいずれかで起動する (NVS 永続
の `BootMode` で選択 — §5 参照)。実運用の 2 mode：

- **Acoustic** — 内蔵 ES8311 マイクで無線機の音声を音響キャプチャ
  (Stick を IC-705 のスピーカーグリルに向ける)。デコーダ常時稼働、
  デコード結果が LCD に表示、operator が station を選ぶと BLE CI-V
  経由で IC-705 の PTT 制御。無線機との配線不要。
- **Qso** — Acoustic と同じだが bidirectional I2S 経由で Stick の
  スピーカーから合成 TX 音声を出力する (IC-705 内蔵マイクを介する
  PTT-only ではなく、Stick が音源)。Stick の TRRS ジャックと無線機の
  PHONE / SP IN を音声ケーブルで接続するとき使う。

5 つの開発 / 診断 mode：

- **Decode** — `wav_sim` が baked `qso3_busy.wav` をループ再生、音源
  接続なしでデコーダ動作を確認する用途。
- **Wifi** — WiFi STA + UDP log sink 起動、デコードなし。`cfg.toml`
  の AP 認証情報検証用。
- **CivTest** — IC-705 への BLE CI-V リンクを動作確認 (周波数読み出し、
  PTT toggle、mode 読み出し)。ペアリング診断用。
- **TxTest** — I2S TX から 1500 Hz FT8 トーンを合成、スピーカー / 音声
  ケーブル経路をリアル信号なしで検証。
- **Uac** — USB-Host Audio Class。**M5StickS3 ハードウェアでは動作不可**
  (基板に VBUS source 回路と ID pin 配線がない)。後継 CoreS3 への移行
  予定は `docs/EMBEDDED.ja.md` 参照。共有検証作業のため code は残置。

---

## 2. ハードウェア

### 必須

| 項目 | 用途 |
|---|---|
| **M5StickS3** | ホスト基板 (ESP32-S3 LX7、8 MB Octal PSRAM、ES8311 codec、ST7789P3 135×240 LCD、KEY1 / KEY2) |
| **USB-C ケーブル** | 給電 + シリアルコンソール + ファーム書込 |
| **音声入力可能なトランシーバ** | IC-705 強く推奨 (BLE CI-V 標準対応) |

### Optional

| 項目 | 用途 |
|---|---|
| **3.5 mm TRRS ケーブル** | Stick TRRS ジャック ↔ 無線機 PHONE / SP IN。**Qso mode の TX には必須**。Acoustic mode では不要 |
| **WiFi AP** | `log::info!` を UDP で PC にストリーム (LCD scroll より格段に追いやすい) |
| **`udp-log-listen.sh` 稼働 PC** | UDP log stream 受信 |

### ピンアウト参照

M5StickS3 GPIO map は `embedded-poc/m5stack-s3-app/src/board.rs` 参照 (KEY1 = GPIO 11、KEY2 =
GPIO 12、ES8311 I2C = GPIO 38/39、LCD SPI = GPIO 13/14/15/17/18 等)。
M5Stack 公式 pinout PDF と byte-for-byte 一致。

---

## 3. 初回 setup

### 3.1. ツールチェイン

ファームは Rust-on-Xtensa-LX7。`esp` rust toolchain + ESP-IDF v5.x が
必要 (後者は `embuild` が自動管理)。

```sh
# 一度だけ: espup 管理の Xtensa toolchain install
curl -L https://github.com/esp-rs/espup/releases/latest/download/espup-x86_64-unknown-linux-gnu -o espup
chmod +x espup
./espup install
source ~/export-esp.sh

# 確認: embedded crate に cd すると `esp` toolchain が自動選択される
# (rust-toolchain.toml 経由)
cd embedded-poc/m5stack-s3-app
rustup show active-toolchain   # → "esp"
```

`~/export-esp.sh` は Xtensa toolchain 用に `PATH` + `LIBCLANG_PATH` を
export する。**シェルを開く度 source すること** — さもないと `bindgen`
が `libclang` を見つけられず build が混乱するエラーで止まる。

### 3.2. Build

```sh
cd embedded-poc/m5stack-s3-app
source ~/export-esp.sh        # §3.1 参照
cargo build --release
```

初回 build は ~10 分 (ESP-IDF + esp-dsp + esp-nimble コンパイル)。
2 回目以降は ~30 秒。

### 3.3. Flash + monitor

**必ず wrapper スクリプトを使う** こと、`espflash flash --monitor` を
直接叩いてはいけない：

```sh
../scripts/flash-monitor.sh \
    target/xtensa-esp32s3-espidf/release/mfsk-core-m5stack-s3-app \
    logs/first-boot-$(date +%Y-%m-%d).log \
    90    # capture 秒数 (bringup シーケンスには ≥ 60)
```

wrapper を使う理由：
- `espflash monitor` は `--before default-reset` がデフォルトで、S3
  USB-OTG 基板では flash 後に chip を DOWNLOAD mode に落とす
  ("rst:0x15 USB_UART_CHIP_RESET … waiting for download")。wrapper は
  `--before no-reset --after no-reset` を渡す。
- 同一 ELF の再 flash は "Segment … has not changed, skipping write"
  と出て **実際は書き込まれない** — chip は旧バイナリを動かし続ける。
  wrapper はこれを明示ログするので不可解な挙動を追わずに済む。これが
  出たらソースファイルを 1 つ触る (`log::info!` を 1 行ずらす等) と
  rebuild が走る。
- 実 flash は S3 factory partition で 15-25 秒。その後 monitor が
  指定秒数 (上記例は `90`) chip stdout を log path にキャプチャ。

### 3.4. `cfg.toml`

gitignore 対象テンプレートをコピーして実値を入れる：

```sh
cp cfg-sample.toml cfg.toml
# cfg.toml を編集 — §4 参照
```

`cfg.toml` は `build.rs` がコンパイル時に読み、値は
`cargo:rustc-env=` 経由で injection されるため、**変更は必ず rebuild
が必要**。これは意図的：秘密が git に入らない、認証情報変更を
operator が rebuild で必ず明示確認する。

---

## 4. `cfg.toml` リファレンス

```toml
[wifi]
ssid = "your-ssid-here"        # 空 → WiFi init skip
psk  = "your-passphrase-here"
pc_ip = "auto"                  # "auto" / "255.255.255.255" / "192.168.x.y"
port  = 9999                    # UDP log 送信先 port

[station]
call = "JL1NIE"                 # operator callsign — 空 → QSO FSM idle
grid = "PM95"                   # 4 文字 Maidenhead grid
```

### `[wifi]`

- **`ssid` / `psk`**: 自宅 WiFi 認証情報。`ssid = ""` (または `[wifi]`
  セクション丸ごと削除) で WiFi init をスキップ — ファームは起動する
  が UDP log なし。携帯用途に便利。
- **`pc_ip`**:
  - `"auto"` (default): DHCP 取得 IP からサブネット directed
    broadcast を計算 (例: `192.168.1.42/24` → `192.168.1.255`)。最も
    確実: AP は limited broadcast (`255.255.255.255`) を drop することが
    多いが directed broadcast は通す。
  - `"255.255.255.255"`: limited broadcast。単純 LAN なら動く。
  - `"192.168.x.y"`: 固定 PC IP への unicast。PC IP が安定なら最も
    確実。
- **`port`**: UDP datagram port (default 9999)。PC 側:
  `embedded-poc/scripts/udp-log-listen.sh 9999`。

### `[station]`

- **`call`**: operator callsign。空 → QSO FSM は Idle のまま、auto-CQ
  メニュー toggle は no-op。入力 → CQ メッセージのアドレス部に使われる。
- **`grid`**: CQ メッセージ用 4 文字 Maidenhead grid。
- **デバイス上での編集は意図的にサポートしない** — 再編集 + rebuild が
  正規ワークフロー。2 ボタンで callsign 全入力はエラーが多く、誤入力は
  wrong-call QRM の原因。

### WSL2 PC listener

WSL2 上 (Windows 上の Linux) の UDP log: inbound UDP 9999 を許可する
Hyper-V ファイアウォール規則が必要 (mirrored-mode default はブロック
する)。`git log --grep "Phase 0.6"` で `New-NetFirewallHyperVRule`
レシピを検索。

---

## 5. `BootMode` — mode の選択と切替

選択された `BootMode` は NVS (`mfsk` namespace、key `boot_mode`) に保存され、
電源 cycle を跨いで persist する。**build 時の choice ではない** — 同じ
flash バイナリでどの mode も実行可能。

### Mode 巡回

```
Decode → Wifi → Acoustic → CivTest → TxTest → Qso → Uac → Decode → …
```

### Mode 変更 2 通り

| タイミング | 方法 | 効果 |
|---|---|---|
| **boot 時** | **KEY1** (前面ボタン) を押しながら電源投入 / リセット | 次の mode へ巡回 (上記順 1 ステップ)、NVS に persist、その mode で起動 |
| **稼働中** | **KEY2** (上面ボタン) を ≥ 1 秒長押し | 同じく次の mode へ巡回、persist、`esp_restart()` |

現在の mode は以下で表示：
- boot シリアルログ (`boot_mode: QSO`, `boot_mode: ACOUSTIC` 等)
- LCD status bar (上段右側)

### 日常運用での推奨 mode

- **Acoustic**: 大半の operator はこれ。Stick を無線機スピーカーに
  向けるだけ。BLE CI-V は optional だが推奨 (QSO 時の auto PTT)。
- **Qso**: TRRS ケーブルがあり Stick から実 TX したい場合 (無線機内蔵
  マイク経由ではなく)。

他の mode は診断用 — 日常利用では不要なことがほとんど。

---

## 6. デコードパイプライン構成 (Phase 1.7.7 以降)

音声入力 → デコード結果出力、FT8 slot 終了から ~1.5–2 秒。

```
ES8311 mic / TRRS in   ← 48 kHz stereo I2S RX (1 ch 使用)
        │
        │  audio::capture_tx_thread  (LX7 PRO_CPU、prio 5)
        │  ─ 48k → 12k Q32 resampler (ch 毎)
        │  ─ 12_000 × 15 = 180_000 sample capture で slot-end signal
        ▼
chunk_q  ← ChunkMsg::{Samples(Vec<i16>), SlotEnd { wav_idx, total_samples }}
        │
        │  stage1_inc::worker  (APP_CPU、prio 6 — dsp_worker を preempt)
        │  ─ 音声蓄積、1 秒経過で auto-gain shift 確定
        │  ─ NSPS/2 = 960 sample 毎に 3840-pt sc16 FFT (per-pair)
        │  ─ 矩形窓 (NFFT=3840 で tone_step_bins = 2.0 exact)
        │  ─ two-for-one real-FFT trick (1 complex FFT = 2 frame)
        │  ─ mag² saturate u16 → spec (PSRAM, ~360 KB)
        │  ─ half ごとの allsum (head 100-1550 Hz / tail 1550-3000 Hz)
        ▼
spec_q   ← SpecBundle { spec, allsum_head, allsum_tail }  (~SlotEnd の 200 ms 前)
slot_q   ← Slot { audio, wav_idx }                        (SlotEnd 時)
        │
        │  decode_pipeline (PRO_CPU、prio 6)
        │  ┌─ 2. coarse_sync_split_with_allsum → top 30 candidate
        │  │   (head + tail を dual_core dispatch で並列)
        │  ├─ 3a. pass2 (refine、sync_quality_block0 で top 15)
        │  │   Goertzel per-symbol DFT (scratch ゼロ ← Phase 1.7.7)
        │  └─ 3b. stage3 (candidate 毎に LLR + BP/OSD)
        │      full 21-sync + 58-data cs 構築も Goertzel
        ▼
results  → [(freq_hz, dt_sec, message77, hard_errors, snr_db)]
        │
        │  qso::QsoManager FSM
        ▼
LCD UI ← decoded_list + waterfall + status bar
BLE   ← operator が callsign 選択 → CI-V PTT toggle (Acoustic/Qso)
I2S TX ← FT8 音声合成 (Qso mode のみ)
WiFi  → UDP log (`log::info!` 各行)
```

### なぜ Goertzel か (Phase 1.7.7-Stick)

旧パイプラインは per-symbol DFT 用に 4 × 30 KB の internal-DRAM
`BASIS` sin/cos スクラッチを確保していた (60 KB × 2 re/im × 2
dual_core worker = 120 KB)。Qso mode で I2S bidir DMA descriptor が
内部 DRAM の連続大ブロックを確保できず、
`i2s_alloc_dma_desc: allocate DMA buffer failed` で起動失敗していた。

Phase 1.7.7 で BASIS dot-product を一般化 Goertzel 再帰に置換 — (sym,
tone) ごとに状態 3 値をスタックに置いて return 時に捨てる。同じ
DFT 数学、**scratch ゼロ**、+0.16..+0.63 dB SNR 改善 (f32 Goertzel が
旧 Q15 BASIS dot product より精度高い)。M5StickS3 Qso mode は I2S
bidir DMA alloc が初回で成功して clean に起動。

詳細は project memory `phase177-goertzel` と PR #103 参照。

---

## 7. UI と操作

### LCD レイアウト (135 × 240 縦)

```
┌─────────────────────────────────┐
│ status bar (mode / RX gain / …) │
├─────────────────────────────────┤
│                                 │
│   waterfall (200-2700 Hz band)  │
│   ~80 行                        │
│                                 │
├─────────────────────────────────┤
│   decoded list (callsign + DT)  │
│   ハイライト = 応答対象選択中   │
├─────────────────────────────────┤
│   TX strip (次の送信メッセージ) │
└─────────────────────────────────┘
```

### ボタン

| Button | 短押し | 長押し |
|---|---|---|
| **KEY1** (前面) | decoded list の選択を巡回 | 現スロットの median DT を新同期基準として mark (Phase 1.7.3+) |
| **KEY2** (上面) | メニュー開閉 | `BootMode` 巡回 + restart (§5 参照) |
| **Reset 側面** | 再起動 | — |

### メニュー (KEY2 短押し)

モーダルメニューを開く (Phase 1.7-Stick):

- **Band picker** — HF 11 band プリセット (160 m → 6 m); BLE CI-V で
  IC-705 周波数を変更。
- **Mode picker** — USB / LSB / DATA-U / DATA-L。
- **Auto-CQ toggle** — ON 時、`QsoState == Idle` の slot で毎回 CQ を
  出す。
- **Auto-DF toggle** — ON 時、decoded stream から作る
  `tx_picker::OccupancyMap` を使って空いている audio DF slot を選ぶ
  ので TX が他局を QRM しない。

---

## 8. QSO ワークフロー (Acoustic / Qso mode)

1. IC-705 を BLE ペアリング (一度のみ — ペアリングしない場合は §9
   troubleshooting 参照)。
2. Stick を起動。`auto_cq` ON で TX slot 毎に CQ 発射。
3. 応答局があると、その callsign が decoded list に `[YOUR-CALL]`
   ハイライト付きで表示 (`decode_pipeline` の filter が宛先メッセージを
   認識)。
4. **KEY1 短押し** で応答者を選択。
5. ファームが QSO FSM 進行 (`Cq → InCallSelected → WaitingReport →
   WaitingR73`)、各 TX slot 境界で BLE CI-V 経由 IC-705 PTT を keying、
   次のメッセージを送信。
6. R-73 受信後 FSM は `Cq` に復帰、auto-CQ が再開。

完全な状態機械は `embedded-poc/mfsk-app-shared/src/qso.rs` 参照。

---

## 9. Troubleshooting

| 症状 | 原因 | 対処 |
|---|---|---|
| build 時 `bindgen / unable to find libclang` | `~/export-esp.sh` source 忘れ | `cargo` 前に `source ~/export-esp.sh` |
| flash が ~5 秒で完了 "Segment … has not changed, skipping write" | 同一 ELF 再 flash | ソース 1 行触って rebuild |
| flash 後 chip が DOWNLOAD mode へ (`rst:0x15 USB_UART_CHIP_RESET`) | `espflash monitor` 直接利用 + default reset | 必ず `embedded-poc/scripts/flash-monitor.sh` を使う |
| Qso mode で `i2s_alloc_dma_desc: allocate DMA buffer failed` | DRAM fragmentation (Phase 1.7.7 以前) | Phase 1.7.7 (Goertzel) 以降のファームへ更新 |
| BLE CI-V が IC-705 とペアリングしない | IC-705 が CI-V mode だが BLE advertising していない | IC-705 で `MENU → SET → Bluetooth → Bluetooth function = ON`、`Pairing/Reception = Pairing reception`。Stick を再起動 |
| UDP log が PC に届かない | router が subnet broadcast を drop、または firewall | `cfg.toml` で `pc_ip = "192.168.x.y"` (PC unicast)。PC firewall で UDP 9999 開放。WSL2 は §4 参照 |
| 数分でコンソールフリーズ | USB-CDC host disconnect (chip は `println!` 継続) | ファームで既に修正済 (commit `8b46f4e` が `usb_serial_jtag_is_connected` で `println!` を gating)。fresh build で起きる場合は tooling 古い |
| デコーダ動いてるが 0 decode | 音声 level 過小 / 過大、または IC-705 の band 違い | `audio capture+tx tick: NNN B/s rx` ログ行を見る — 健全な 48 kHz stereo I2S RX なら 192–196 kB/s 持続。無音なら mic gain (`embedded-poc/m5stack-s3-app/src/audio.rs` `mic_gain_db`) や無線機出力 level 確認 |

### ログキャプチャ規約

毎回 `logs/<bin>_<tag>_$(date +%Y-%m-%d).log` に出力するとセッション
診断が予測可能に積み上がる。`flash-monitor.sh` の log path 引数に
従って統一すること。

PC 側 UDP log:

```sh
embedded-poc/scripts/udp-log-listen.sh 9999 \
    | tee logs/udp_$(date +%Y-%m-%d_%H%M).log
```

---

## 10. 参照

- [`docs/EMBEDDED.ja.md`](EMBEDDED.ja.md) — `mfsk-core` 組込統合リファレンス (scalar アーキテクチャ、FFT-extern 契約、Goertzel per-symbol DFT、Q-format リファレンス、`mfsk-ffi-ft8` C ABI チュートリアル、性能ベンチ、streaming pipeline、binary footprint)
- [`docs/ROADMAP.md`](ROADMAP.md) — リリースマイルストーン + Phase B-Stick / B-Core プラン
- [`embedded-poc/CLAUDE.md`](../embedded-poc/CLAUDE.md) — 共通組込ツールチェインメモ (cross-board)
- [`embedded-poc/m5stack-s3-app/CLAUDE.md`](../embedded-poc/m5stack-s3-app/CLAUDE.md) — この crate での AI agent 向けボード固有メモ
- `mfsk-core/` — 実体のデコーダライブラリ (host + embedded shared、crates.io 公開)
- `embedded-poc/mfsk-app-shared/` — ボード非依存アプリロジック (QSO FSM、UI primitive、WiFi、log fanout)
