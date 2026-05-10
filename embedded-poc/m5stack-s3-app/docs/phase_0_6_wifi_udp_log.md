# Phase 0.6 — WiFi UDP log streaming

> **Phase 1 (USB UAC) 着手の前提条件**。実装済 (2026-05-10、未実機検証)。
> このドキュメントを次セッション (リモート機) で読んで cfg.toml 作成 →
> build → flash → log 受信を確認する手順をまとめる。

## なぜ必要か

ESP32-S3 USB-OTG ポートは 1 個だけで、Phase 1 で IC-705 を USB host で
接続する作業に入った瞬間、`espflash flash --monitor` (USB-CDC log) の
経路が物理的に塞がる。`log::info!` の UART 出力が見えなくなり、UAC
descriptor parse 失敗 / endpoint stream エラー / panic を診断できなく
なる = 泥沼確定。

WiFi STA + UDP datagram で **USB-OTG 状態と完全独立な log 経路** を
boot 時に立ち上げ、PC 側で `nc -lu 9999` 等で受信できれば、IC-705 接続
中でも runtime log が PC に流れ続ける。

## アーキテクチャ

`log_sink::FanoutLogger` が 4 系統に分岐:

```
log::info!(...) ──► [FanoutLogger]
                      ├─ UART0 / USB-CDC      (espflash monitor で見える)
                      ├─ LcdPanel             (M5StickS3 LCD 内蔵 12 行)
                      ├─ FlashLog             (LittleFS、現状 placeholder)
                      └─ UdpLogSink (Phase 0.6)  ─ WiFi STA → 255.255.255.255:9999
```

WiFi associate までの 2-3 秒は既存 `staging` Deque (32 行) で受け、
UDP sink 登録時に `LogFanout::drain_staging_to_udp()` で吐き出す。
WiFi 接続失敗時は `log::warn!` を出して残り 3 系統で継続 — boot 完走
> log 経路の優先順位。

## ファイル構成

| Path | 役割 |
|---|---|
| `src/wifi.rs` | `BlockingWifi<EspWifi>` の STA 接続。wifikey2 から薄く port (AP fallback / NVS 永続化なし) |
| `src/udp_log.rs` | `UdpSocket` ベースの sink。non-blocking + best-effort |
| `src/log_sink.rs` | `LogFanout::udp` フィールド追加、`drain_staging_to_udp()` |
| `src/main.rs` | boot 時に WiFi → UDP sink → staging drain の順で初期化、modem を WiFi に渡し残り peripherals を `display::run_log_panel` に |
| `cfg-sample.toml` | 資格情報テンプレ (commit) |
| `cfg.toml` | **gitignored**、user が手書きで作る |
| `.gitignore` | `cfg.toml` / `cfg.toml-*` を除外 |
| `build.rs` | cfg.toml を読み `cargo:rustc-env` で `WIFI_SSID` / `WIFI_PSK` / `UDP_LOG_TARGET` / `UDP_LOG_PORT` を注入 |
| `sdkconfig.defaults` | `CONFIG_ESP_WIFI_ENABLED=y` + `CONFIG_LWIP_TCP_ENABLED=y` + `CONFIG_LWIP_UDP_ENABLED=y` |
| `../scripts/udp-log-listen.sh` | PC 側 capture wrapper |

## セットアップ手順 (リモート機初回)

### 1. cfg.toml を作る

```sh
cd embedded-poc/m5stack-s3-app
cp cfg-sample.toml cfg.toml
```

`cfg.toml` を編集:

```toml
[wifi]
ssid = "<家 / dev 環境の SSID>"
psk = "<password>"
pc_ip = "auto"   # DHCP で付与された subnet broadcast (例 192.168.1.255) を runtime 計算
port = 9999
```

`pc_ip` の値:
- `"auto"` (推奨): DHCP の IP/mask から directed subnet broadcast を runtime
  計算 (limited broadcast `255.255.255.255` より router/AP で drop されにくい)
- `"255.255.255.255"`: limited broadcast (シンプルな LAN なら動く)
- `"192.168.x.y"`: PC 個別 unicast (最も確実、ただし PC IP 固定が前提)

`git status` で `cfg.toml` が **untracked** として出ることを必ず確認
(tracked になっていたら `.gitignore` が効いてない)。

### 2. PC 側 listener

```sh
embedded-poc/scripts/udp-log-listen.sh 9999 /tmp/m5s3.log
# または直接:
nc -lu 9999 | tee /tmp/m5s3.log
```

ファイアウォール: PC が UDP 9999 を受信するブロックがないこと。Linux
で ufw 等使ってる場合は `sudo ufw allow 9999/udp`。

### 3. ビルド + flash

```sh
source ~/export-esp.sh
cd embedded-poc/m5stack-s3-app
cargo build --release
../scripts/flash-monitor.sh \
    target/xtensa-esp32s3-espidf/release/mfsk-core-m5stack-s3-app \
    logs/phase_0_6_first_$(date +%Y-%m-%d).log \
    60
```

### 4. 動作確認

期待される log 系列 (USB-CDC / PC UDP の両方、Phase 0.6 mode):

```
=== mfsk-core-m5stack-s3-app boot ===
phase 4 + 0.6: QSO FSM + WiFi UDP log
WiFi STA up: 192.168.x.x/24 (gw 192.168.x.1, bcast 192.168.x.255) ch=Some(N)
UDP log sink up → 192.168.x.255:9999
decode_pipeline skipped (Phase 0.6 mode: WiFi takes DRAM that decoder needs)
alive tick=0 free_heap=8514004
alive tick=50 free_heap=8513884
alive tick=100 free_heap=8513688
...
```

(`WIFI_SSID` 設定有り = Phase 0.6 mode。decode_pipeline は走らないが、
`alive tick` heartbeat が 5 sec 間隔で UDP に流れる。WSL2 host で
`nc -lu 9999` で受信できれば成功。)

USB ケーブルを抜く (= IC-705 USB host 接続を simulate) → PC 側 UDP
listener にログが**流れ続ける** ことを確認 (mirrored networking 設定済の
WSL2 にも届く)。

## 合否基準 (Phase 0.6 mode = WiFi 有効)

- [x] WiFi STA IP が log の "WiFi STA up:" 行に表示される (実機 192.168.1.38/24 で確認済 2026-05-11)
- [x] UDP sink up ログが出る (192.168.1.255:9999 で確認済)
- [x] panic / abort ゼロ、heap 安定 (~8.5 MB free 維持)
- [ ] **WSL2 mirrored networking 設定後**、`nc -lu 9999` で boot 行 +
      `alive tick=...` 5 sec heartbeat が PC に到達
- [ ] USB ケーブル disconnect 後も UDP log 継続 (= Phase 1 UAC で IC-705
      が USB-OTG を占有しても log 経路が生きる証明)
- [ ] `cfg.toml` が `git status` で untracked のまま (tracked にならない)

(BLE coexistence と FT8 decode は Phase 0.6 では verify しない —
DRAM 不足で同居不可、Phase 0.7+ で memory architecture 見直し時に再検証)

## 重要: WiFi 有効時は decode_pipeline を skip する

**実機検証で判明 (2026-05-11)**: ESP32-S3 の内部 DRAM (~161 KB) は
WiFi static buffers (~80 KB) + decode_pipeline thread tree (main 32 KB
+ dsp_worker + stage1_inc 16 KB + wav_sim) + 静的 `BASIS_RE/IM`
(60 KB×2) を同時に詰めると `xTaskCreatePinnedToCore failed: -1`
(stage1_inc spawn 失敗) で panic する。BASIS は cache/DMA 制約で PSRAM
に移せない (decoder が動かなくなる)。

`main.rs` の挙動:
- `WIFI_SSID` 空 (cfg.toml 不在) → Phase 4 demo: decode_pipeline 起動、
  qso3 WAV ループ + auto-CQ FSM
- `WIFI_SSID` 設定あり → Phase 0.6 mode: decode_pipeline skip、WiFi +
  UDP log + display heartbeat のみ。FT8 decode は走らない

これは「**WiFi UDP log** が動くこと」を Phase 0.6 で先に確立する判断。
両者 coexist は Phase 0.7+ で memory architecture の設計見直しが要る
(static BASIS を thread-local 動的確保 + WiFi static buffer を sdkconfig
で削るなど)。

## sdkconfig: BT / USB Host も一時無効化

DRAM 確保の余地を作るため `sdkconfig.defaults` で:
- `CONFIG_BT_ENABLED=n` (NimBLE central は Phase 2 で再有効化)
- `CONFIG_USB_HOST_ENABLED=n` (Phase 1 UAC で再有効化)

Phase 1 着手時に上記を `=y` に戻して UAC 実装、coexistence は実機で
要再検証。BT は Phase 2 で。

## 既知の限界 (Phase 0.7+ で対処)

1. **Boot-time gap (2-3 秒)**: WiFi associate するまでの log は
   `staging` Deque (32 行) に溜まる。32 行を超えると古いものが落ちる。
   **救出策**: `flash_log.rs` placeholder を LittleFS ring buffer 化、
   次起動時に dump
2. **UDP packet loss**: best-effort なので burst log 中に落ちる可能性。
   通常運用では問題ない。重要 log はあえて遅延 (`std::thread::sleep`)
   入れる手も
3. **broadcast に依存**: Wi-Fi AP の "broadcast 抑制" (一部 Mesh 機能)
   が有効だと届かない。その場合は `pc_ip` を unicast (PC の IP) に
4. **WiFi credential が `cfg.toml` に平文**: gitignore 任せ、紛失時に
   commit リスクあり。Phase 6 (button + UI) で NVS storage + AP fallback
   web 設定 (wifikey2 `config.rs` 参照) 検討
5. **WiFi+BLE coex 未検証**: esp-idf default の software coex で動く
   想定だが、実機で `CONFIG_ESP_COEX_*` 調整が要るかも

## 参照

- `/home/minoru/src/wifikey2/wifikey/src/wifi.rs` (211 行) — 同 owner の
  WiFi STA テンプレ。本実装はここから AP / NVS / reconnect を削った薄い port
- `/home/minoru/.cargo/git/checkouts/esp-idf-svc-*/050b120/examples/wifi.rs` —
  esp-idf-svc 0.52 の正しい呼出順 (NVS は `None` で OK)
- `/home/minoru/.claude/plans/humming-drifting-alpaca.md` — Phase 0.6 の
  実装計画 (本ドキュメントの設計判断ロジック)

## Phase 1 着手前のチェックリスト

Phase 0.6 が次セッションで実機検証できたら、以下を満たした状態で UAC
着手:

- [ ] PC UDP listener が log を受け続ける (USB 抜いても止まらない)
- [ ] BLE が WiFi 同時動作中も panic しない
- [ ] `cfg.toml` が確実に gitignored
- [ ] LCD scroll panel 表示も従来通り (regression なし)

そのまま `uac.rs` の実装 (recipe.c port + USB Host stack init) に進む。
