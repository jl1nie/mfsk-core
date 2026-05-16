//! `/littlefs/run.log` append-only writer (Phase 0.5).
//!
//! `esp-idf-svc` 0.52 系の `vfs::EspLittleFs` を使って partitions.csv の
//! `littlefs` 1 MB パーティションを `/littlefs/` にマウント。`run.log`
//! をオープンしっぱなしで `log_sink::FlashLog` 実装を提供する。
//!
//! ローテーション: 1 MB 上限を超えたら `run.log` → `run.log.1` に rename
//! (1 段のみ、それ以上は破棄)。
//!
//! Boot 時に前回 run の末尾を CDC へ吐き出す `dump_tail(n)` も提供。
//!
//! Phase 0.5 では型と TODO のみ。`vfs::EspLittleFs` の正確な API 名は
//! 0.52 の docs.rs で再確認してから埋める (mount path, partition label,
//! format on first boot 等の引数)。

#![allow(dead_code)]

use crate::log_sink::FlashLog;

pub struct LittleFsLog {
    // TODO: hold std::fs::File handle once VFS mount is up.
    // Phase 0.5 plumbing: keep this struct compileable with a
    // placeholder so log_sink can be wired without blocking on the
    // exact esp-idf-svc API spelling.
    _placeholder: (),
}

impl LittleFsLog {
    pub fn open() -> Option<Self> {
        // TODO: mount /littlefs and open append-only.
        // For Phase 0.5 boot bring-up, return None — Fanout falls back
        // to LCD + EspLogger only.
        None
    }

    pub fn dump_tail(&self, _max_lines: usize) {
        // TODO: read /littlefs/run.log.1 then run.log, emit to log::info!
    }
}

impl FlashLog for LittleFsLog {
    fn write_line(&mut self, _line: &str) {
        // TODO: write + '\n', fsync, rotate at 1 MB.
    }
}
