//! ADIF logger backed by LittleFS (Phase 5).
//!
//! 参照: `/home/ubuntu/src/rs-ft8n/docs/qso-log.js::toAdif`。
//! ストレージ: partitions.csv の `littlefs` パーティション (1 MB)。
//! `esp_idf_svc` の VFS 経由で `/littlefs/qso.adi` に append-only。
//! USB-CDC コマンド `dump_adif` でホストへ全文転送する API も Phase 5 で。
//!
//! Phase 0 ではプレースホルダのみ。
