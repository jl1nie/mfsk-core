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
pc_ip = "255.255.255.255"   # broadcast (LAN 全 PC が listener で受け取れる)
port = 9999
```

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

期待される log 系列 (USB-CDC / PC UDP の両方):

```
=== mfsk-core-m5stack-s3-app boot ===
phase 4 + 0.6: QSO FSM + WiFi UDP log
WiFi STA up: 192.168.x.x ch=Some(N)
UDP log sink up → 255.255.255.255:9999
[QSO] CALL: CQ JL1NIE PM95
WAV[0] p1=30 dec=7
...
```

USB ケーブルを抜く (= IC-705 USB host 接続を simulate) → PC 側 UDP
listener にログが**流れ続ける** ことを確認。

## 合否基準

- [ ] WiFi STA IP が log の "WiFi STA up:" 行に表示される
- [ ] decode pipeline / FSM ログが UDP 経路で PC に到達 (USB-CDC 切断
      しても継続)
- [ ] BLE NimBLE が同時動作可能 (WiFi 起動後も BLE 関連 panic 無し)。
      現状の `civ.rs` は scaffold だが crate は依存に入っているので
      coex の効果はこの段階で確認できる
- [ ] `cfg.toml` が `git status` で untracked のまま (tracked にならない)
- [ ] `WIFI_SSID` を空にして build → boot しても crash せず "WIFI_SSID
      empty" log だけ出て継続

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
