//! Log fanout: console + LCD scroll panel + flash file + UDP datagram.
//!
//! 動機: USB-OTG host モード中 (= IC-705 接続中) はシリアルコンソールが
//! 物理的に使えない。電源を切って USB ケーブルを母艦 PC に挿し直さない
//! と `espflash --monitor` は読めない。そこで:
//!
//!   log::info!(...) ──► [Fanout]
//!                         ├─ EspLogger (USB-CDC、生きていれば)
//!                         ├─ LcdPanel  (mipidsi に常時 12 行の scroll)
//!                         ├─ FlashLog  (/littlefs/run.log に append)
//!                         └─ UdpLog    (Phase 0.6、WiFi UDP 経由 PC へ)
//!
//! どれか落ちても残りは継続。LCD/Flash が初期化前に呼ばれた `log::info!`
//! は内部 staging buffer に貯めておき、初期化後に flush する。UDP も
//! WiFi associate が終わるまでは sink 不在 (= staging に貯まる)、関連
//! 後に `drain_staging_to_udp` で吐き出す。

use core::fmt::Write as _;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use heapless::{Deque, String};

/// 1 ログ行の最大長。
///
/// **160 に拡大 (2026-08-22)。** 元は 80 — LCD の 6x10 フォント 22 文字幅
/// を基準にした値だが、同じ型が **UDP 送信**にも使われている。USB host を
/// 入れるとシリアルコンソールが消えるため、UDP はその時点で唯一の観測手段
/// になる (issue #163)。実測: `uac: rx tick` の音声統計行はちょうど `rms`
/// の直前で切られていた — 読みたい数値そのものが届かない状態だった。
///
/// LCD 側は描画時にどのみち幅で切れるので、拡大による表示上の影響はない。
/// コストは `LcdPanel` 12 行 + staging 32 行 = +3.5 KB。
pub const LINE_MAX: usize = 160;
/// LCD scroll panel に保持する行数。120 px / 10 px = 12 行。
pub const LCD_LINES: usize = 12;
/// 起動直後のステージング (UDP sink 確立前) 行数。
///
/// **32 → 128 → 32 (2026-08-22/23).** 一度 128 に増やしたのは、WiFi
/// 接続までの約30秒ぶんのログが 32 行では溢れて消え、USB 列挙まわりが
/// 読めなかったから。だがこの配列は `.bss` に居る — `heapless::String
/// <160>` × 128 で約 20 KB の内部 DRAM で、ホストモードではそれが直接
/// 効いてくる。USB デバイス3台がぶら下がった状態の空き内部 DRAM は
/// 14.6 KB (最大ブロック 7680 B) まで落ち、UAC のアイソクロナス転送
/// バッファが確保できず `uac_host_device_start` が失敗した。
///
/// 32 に戻す。増やした理由のほうは、より良い手段で解決済み: C 側ログを
/// fanout に流す `esp_log_bridge`、画面の常設パネル、そしてフラッシュ
/// コアダンプ。起動直後の行を大量に溜め込む必要はもう無い。
/// Refs #163.
pub const STAGING_LINES: usize = 32;

pub type LogLine = String<LINE_MAX>;

/// [`LINE_MAX`] に収まるよう切り詰める。**UTF-8 境界を割らない**。
///
/// 元は `&line[..LINE_MAX]` — 境界がマルチバイト文字の途中に落ちると
/// **panic する**。このリポジトリのログ行は em-dash (`—`, 3 バイト) を
/// 日常的に含むので、これは理論上の話ではない: `LINE_MAX` を 80 から 160
/// に変えるだけで、どの行のどの位置で切れるかが変わり、以前は無害だった行
/// が新たに踏みうる。しかも踏む場所がロガー自身なので、落ちたことを報告
/// する手段ごと失う (issue #163)。
///
/// 切り詰めたときは末尾に `~` を付ける。数値が途中で切れた行を、完全な行
/// と読み違えないため — これも #163 で避けたい「曖昧な結果」の一種。
fn clip(line: &str) -> &str {
    if line.len() <= LINE_MAX {
        return line;
    }
    let mut end = LINE_MAX - 1;
    while end > 0 && !line.is_char_boundary(end) {
        end -= 1;
    }
    &line[..end]
}

/// LCD scroll panel — `Deque` で末尾追加、容量超過で先頭削除。
pub struct LcdPanel {
    lines: Deque<LogLine, LCD_LINES>,
    dirty: bool,
}

impl LcdPanel {
    pub const fn new() -> Self {
        Self {
            lines: Deque::new(),
            dirty: false,
        }
    }

    pub fn push(&mut self, line: &str) {
        let mut s: LogLine = String::new();
        let truncated = clip(line);
        let _ = s.push_str(truncated);
        if truncated.len() < line.len() {
            let _ = s.push('~');
        }
        if self.lines.is_full() {
            let _ = self.lines.pop_front();
        }
        let _ = self.lines.push_back(s);
        self.dirty = true;
    }

    pub fn drain_dirty(&mut self) -> Option<&Deque<LogLine, LCD_LINES>> {
        if !self.dirty {
            return None;
        }
        self.dirty = false;
        Some(&self.lines)
    }

    /// 最古 → 最新の順でイテレート (描画時に上から下へ流せるように).
    pub fn iter_chronological(&self) -> impl Iterator<Item = &LogLine> {
        self.lines.iter()
    }

    pub fn len(&self) -> usize {
        self.lines.len()
    }

    pub fn is_empty(&self) -> bool {
        self.lines.is_empty()
    }
}

/// Append-only flash log writer (LittleFS).
///
/// Phase 0.5 の LittleFS bring-up で `/littlefs/run.log` を open する。
/// `write_line` は ASCII 1 行 + `\n` を fsync 付き append。1 MB を超えた
/// ら `run.log` → `run.log.1` にローテート。
pub trait FlashLog: Send {
    fn write_line(&mut self, line: &str);
}

/// Fanout sink — `log::Log` 実装。
pub struct LogFanout {
    pub lcd: Mutex<CriticalSectionRawMutex, LcdPanel>,
    /// LCD/Flash/UDP 未初期化期間のステージング。WiFi associate が
    /// 完了するまでの 2-3 秒に呼ばれた `log::info!` を貯めておき、
    /// `drain_staging_to_udp` で UDP に吐き出す経路で使う。
    pub staging: Mutex<CriticalSectionRawMutex, Deque<LogLine, STAGING_LINES>>,
    /// Flash sink (Phase 5 に近い形で確立)。`Option` で boot 時に late-bind。
    pub flash: Mutex<CriticalSectionRawMutex, Option<&'static mut dyn FlashLog>>,
    /// UDP sink (Phase 0.6)。WiFi associate 後に
    /// `*FANOUT.udp.lock() = Some(UdpLogSink::new(..)?)` で登録。
    pub udp: Mutex<CriticalSectionRawMutex, Option<crate::udp_log::UdpLogSink>>,
}

impl LogFanout {
    pub const fn new() -> Self {
        Self {
            lcd: Mutex::new(LcdPanel::new()),
            staging: Mutex::new(Deque::new()),
            flash: Mutex::new(None),
            udp: Mutex::new(None),
        }
    }

    /// 1 行投函 (caller は format 済み &str を渡す)。
    pub fn push(&self, line: &str) {
        // LCD: lock 取れれば push、取れなければ staging に積む
        if let Ok(mut lcd) = self.lcd.try_lock() {
            lcd.push(line);
        } else if let Ok(mut staging) = self.staging.try_lock() {
            self.staging_push(&mut staging, line);
        }
        // Flash: 同様。Mutex を握れない最悪ケースは drop (boot シーケンス
        // 中の竞合のみ想定で、運用時は lock 競合は起きない)。
        if let Ok(mut flash) = self.flash.try_lock() {
            if let Some(sink) = flash.as_deref_mut() {
                sink.write_line(line);
            }
        }
        // UDP: best-effort datagram。sink 未登録 (WiFi 未接続) なら
        // staging に積んで後で `drain_staging_to_udp` で吐く。lock を
        // 取れない場合は黙って落とす (= 排他で詰まらせない)。
        match self.udp.try_lock() {
            Ok(udp) => match udp.as_ref() {
                Some(sink) => {
                    // Anything staged while the lock was busy goes out
                    // first, so order survives — but only a few per
                    // call.
                    //
                    // This runs in whichever task called `log!`, and
                    // that includes lwIP's own. Draining the whole ring
                    // there meant one log line from `tiT` could turn
                    // into 128 synchronous datagrams, each needing a
                    // pbuf, which is how the network stack ran itself
                    // out of memory and aborted. A handful per call
                    // still empties the backlog quickly, because log
                    // lines keep coming.
                    const FLUSH_PER_CALL: usize = 4;
                    if let Ok(mut staging) = self.staging.try_lock() {
                        for _ in 0..FLUSH_PER_CALL {
                            match staging.pop_front() {
                                Some(staged) => sink.send_line(&staged),
                                None => break,
                            }
                        }
                    }
                    sink.send_line(line);
                }
                None => {
                    if let Ok(mut staging) = self.staging.try_lock() {
                        self.staging_push(&mut staging, line);
                    }
                }
            },
            // Lock busy — stage it rather than drop it.
            //
            // This used to be a silent drop, and it lost exactly the
            // lines worth having. The contention window is boot: the
            // sink-install retry loop and the host-mode wait both poll
            // this mutex while every other thread is logging its
            // startup, so PMIC state, the LCD result and the previous
            // boot's coredump summary all fell into it. The symptom is
            // a UDP capture that shows the first three lines of a boot
            // and then jumps twenty seconds forward — which reads like
            // a board that went quiet, not like a logger that threw
            // the lines away. Refs #163.
            Err(_) => {
                if let Ok(mut staging) = self.staging.try_lock() {
                    self.staging_push(&mut staging, line);
                }
            }
        }
    }

    fn staging_push(&self, staging: &mut Deque<LogLine, STAGING_LINES>, line: &str) {
        let mut s: LogLine = String::new();
        let truncated = clip(line);
        let _ = s.push_str(truncated);
        if truncated.len() < line.len() {
            let _ = s.push('~');
        }
        if staging.is_full() {
            let _ = staging.pop_front();
        }
        let _ = staging.push_back(s);
    }

    /// WiFi 関連後に呼ぶ。staging の蓄積行を UDP に流す。staging は
    /// LCD-pre-init とも兼用 (= UDP-pre-init で積まれた行が混じる)
    /// が、頻度的に問題にならないので分けない。
    pub fn drain_staging_to_udp(&self) {
        let Ok(udp_guard) = self.udp.try_lock() else {
            return;
        };
        let Some(sink) = udp_guard.as_ref() else {
            return;
        };
        let Ok(mut staging) = self.staging.try_lock() else {
            return;
        };
        while let Some(line) = staging.pop_front() {
            sink.send_line(&line);
        }
    }
}

/// `log::Log` 実装。`init()` で `log::set_logger` する。
pub struct FanoutLogger {
    inner: &'static LogFanout,
    level: log::LevelFilter,
}

impl FanoutLogger {
    pub const fn new(inner: &'static LogFanout, level: log::LevelFilter) -> Self {
        Self { inner, level }
    }

    pub fn install(&'static self) {
        let _ = log::set_logger(self);
        log::set_max_level(self.level);
    }
}

impl log::Log for FanoutLogger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        metadata.level() <= self.level
    }

    fn log(&self, record: &log::Record) {
        if !self.enabled(record.metadata()) {
            return;
        }
        // LCD/flash 用の短い行 (target prefix を捨てる)。
        let mut line: LogLine = String::new();
        // `write!` into a full `heapless::String` stops silently — the
        // formatter's fragment simply fails to push and the `Err` is
        // the only trace. Mark it, so a line that lost its tail cannot
        // be misread as a complete one. That distinction is the whole
        // point during a session where this is the only console
        // (issue #163): a truncated number read as a whole one is
        // worse than no number.
        let complete = write!(
            &mut line,
            "{} {}",
            level_short(record.level()),
            record.args()
        )
        .is_ok();
        if !complete {
            // Make room for the marker on a char boundary.
            while !line.is_empty() && line.len() + 1 > LINE_MAX {
                line.pop();
            }
            let _ = line.push('~');
        }
        self.inner.push(&line);

        // UART にも吐き出す (EspLogger を init していないので自前で)。
        // C-side ESP_LOG のタイムスタンプ付きフォーマットには合わせず、
        // Rust 側ログは簡素に。
        //
        // USB-CDC (USB-Serial-JTAG) host が切れている間に println! すると
        // VFS 層で TX FIFO 待ちに永久 block して FanoutLogger 自体が
        // freeze する (Phase 0.6 検証で確認、2026-05-11)。Phase 1 (USB host
        // モード) でも CDC 端末が消えるので必須対策。SOF が来てる間だけ
        // stdout に流す。
        //
        // `usb_serial_jtag_is_connected` は S3 の native USB-Serial-JTAG
        // 専用 API — 古典 ESP32 (Core2) には存在しない。Core2 は外付け
        // CP2104 UART bridge を使うので feature flag で無効化し、
        // unconditional println に fallback する (失敗モードが S3 とは
        // 別物 — CP2104 切断時の TX block は ESP32 から見えない)。
        #[cfg(feature = "usb-serial-jtag")]
        let usb_ok = unsafe { esp_idf_svc::sys::usb_serial_jtag_is_connected() };
        #[cfg(not(feature = "usb-serial-jtag"))]
        let usb_ok = true;
        if usb_ok {
            println!(
                "{} {}: {}",
                level_short(record.level()),
                record.target(),
                record.args()
            );
        }
    }

    fn flush(&self) {}
}

const fn level_short(l: log::Level) -> &'static str {
    match l {
        log::Level::Error => "E",
        log::Level::Warn => "W",
        log::Level::Info => "I",
        log::Level::Debug => "D",
        log::Level::Trace => "T",
    }
}
