//! 固定長の1行スロット。確保もブロックもしない。
//!
//! **`std::sync::Mutex` を static に置いてはいけない。** あれは初回
//! ロック時に pthread ミューテックスをヒープ確保する
//! (`OnceBox<Mutex>::initialize` → `pthread_create`)。ホストモードで
//! USB デバイスが3台ぶら下がると内部 DRAM は 38 KB まで落ちるので、
//! その確保は失敗しうる。実際、クラッシュ診断のために置いたスロットが
//! そのまま abort の原因になり、`display::run_log_panel` →
//! `esp_log_bridge::last_error_line` → `pthread_create` という
//! バックトレースを残した。診断が診断対象を壊していた形。
//!
//! しかもこのスロットは `vprintf` フックからも書かれる — つまり USB
//! やドライバのタスク文脈から。そこで確保やブロックを起こしうるものを
//! 使う理由はない。
//!
//! Refs #163.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

/// 1行の最大長。パネル幅 30 文字 × 数行ぶん。
const CAP: usize = 96;

pub struct LogSlot {
    lock: AtomicBool,
    len: AtomicUsize,
    buf: UnsafeCell<[u8; CAP]>,
}

// SAFETY: 中身へのアクセスは `lock` のスピンロックで直列化する。
unsafe impl Sync for LogSlot {}

impl LogSlot {
    pub const fn new() -> Self {
        Self {
            lock: AtomicBool::new(false),
            len: AtomicUsize::new(0),
            buf: UnsafeCell::new([0; CAP]),
        }
    }

    fn acquire(&self) -> bool {
        // 一度だけ試す。競合したら諦める — ログの1行のために
        // 割り込み文脈で回るより、落とすほうがましな類の情報。
        self.lock
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_ok()
    }

    fn release(&self) {
        self.lock.store(false, Ordering::Release);
    }

    /// 空のときだけ書く (最初の1本を残す用)。
    pub fn store_first(&self, text: &str) {
        if self.len.load(Ordering::Relaxed) != 0 {
            return;
        }
        self.store(text);
    }

    pub fn store(&self, text: &str) {
        if !self.acquire() {
            return;
        }
        let n = text.len().min(CAP);
        // UTF-8 境界を割らない。
        let mut n = n;
        while n > 0 && !text.is_char_boundary(n) {
            n -= 1;
        }
        // SAFETY: ロック保持中。`n <= CAP`。
        unsafe {
            let dst = &mut *self.buf.get();
            dst[..n].copy_from_slice(&text.as_bytes()[..n]);
        }
        self.len.store(n, Ordering::Release);
        self.release();
    }

    /// 呼び出し側のバッファへコピーして返す。
    pub fn read(&self) -> heapless::String<CAP> {
        let mut out: heapless::String<CAP> = heapless::String::new();
        if !self.acquire() {
            return out;
        }
        let n = self.len.load(Ordering::Acquire).min(CAP);
        // SAFETY: ロック保持中。書き込み側が UTF-8 境界を守っている。
        let src = unsafe { &*self.buf.get() };
        if let Ok(s) = core::str::from_utf8(&src[..n]) {
            let _ = out.push_str(s);
        }
        self.release();
        out
    }

    pub fn is_empty(&self) -> bool {
        self.len.load(Ordering::Relaxed) == 0
    }
}
