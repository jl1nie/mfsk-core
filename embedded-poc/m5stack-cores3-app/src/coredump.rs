//! 前回のクラッシュをフラッシュのコアダンプから読み出してログに流す。
//!
//! ホストモードでは USB ドライバが USB-Serial-JTAG を持っていくので、
//! ESP-IDF のパニックハンドラが書き込むコンソールが存在しない。
//! しかもハンドラは `esp_rom_printf` を直接叩くため、fanout にも
//! 乗らない。結果、クラッシュは「理由も痕跡もない突然の再起動」に
//! なる — スタックトレースが出ないのはそのためで、出力が失われて
//! いるのではなく、出力先が無い。
//!
//! コアダンプはリセットを跨いでフラッシュに残るので、次の起動で
//! 読み出して、生きているログ経路 (UDP / LCD) に載せ直す。
//!
//! Refs #163.

use esp_idf_svc::sys;

/// 前回が異常リセットだったか。表示ループが「読める時間だけ止まる」
/// 判断に使う。
static ABNORMAL: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

/// 前回が異常リセット (パニック / WDT / ブラウンアウト) だったか。
pub fn was_abnormal_reset() -> bool {
    ABNORMAL.load(core::sync::atomic::Ordering::Acquire)
}

/// 直近のクラッシュ要約 (画面用、無ければ空)。
static SUMMARY: crate::log_slot::LogSlot = crate::log_slot::LogSlot::new();

/// 画面用: 直近のクラッシュ要約。
pub fn last_crash() -> heapless::String<96> {
    SUMMARY.read()
}

/// Xtensa の EXCCAUSE を人が読める名前に。よく出るものだけ。
fn cause_name(cause: u32) -> &'static str {
    match cause {
        0 => "IllegalInstruction",
        2 => "InstrFetchError",
        3 => "LoadStoreError",
        6 => "IntegerDivideByZero",
        8 => "Privileged",
        9 => "LoadStoreAlignment",
        20 => "InstrFetchProhibited",
        28 => "LoadProhibited",
        29 => "StoreProhibited",
        _ => "other",
    }
}

/// 起動時に1回呼ぶ。ダンプがあれば要約をログに流す。
pub fn report_previous_crash() {
    // Reset reason first. A coredump only exists for a panic or an
    // abort; a brownout or a watchdog leaves nothing behind, so
    // "no dump stored" on its own cannot tell a clean boot from a
    // board whose supply sagged. On battery, powering a radio's VBUS
    // through the boost converter, that is not a hypothetical.
    // Refs #163.
    // SAFETY: 引数なし、副作用なし。
    let reason = unsafe { sys::esp_reset_reason() };
    let name = match reason {
        sys::esp_reset_reason_t_ESP_RST_POWERON => "POWERON",
        sys::esp_reset_reason_t_ESP_RST_EXT => "EXT",
        sys::esp_reset_reason_t_ESP_RST_SW => "SW",
        sys::esp_reset_reason_t_ESP_RST_PANIC => "PANIC",
        sys::esp_reset_reason_t_ESP_RST_INT_WDT => "INT_WDT",
        sys::esp_reset_reason_t_ESP_RST_TASK_WDT => "TASK_WDT",
        sys::esp_reset_reason_t_ESP_RST_WDT => "WDT",
        sys::esp_reset_reason_t_ESP_RST_DEEPSLEEP => "DEEPSLEEP",
        sys::esp_reset_reason_t_ESP_RST_BROWNOUT => "BROWNOUT",
        sys::esp_reset_reason_t_ESP_RST_SDIO => "SDIO",
        _ => "unknown",
    };
    log::warn!("reset reason: {name} ({reason})");
    SUMMARY.store(&format!("rst={name}"));

    // Remember whether this was an abnormal reset. The display loop
    // holds on it once the panel is actually on screen — pausing here
    // would pause in front of a dark LCD, which helps nobody.
    ABNORMAL.store(
        !matches!(
            reason,
            sys::esp_reset_reason_t_ESP_RST_POWERON | sys::esp_reset_reason_t_ESP_RST_EXT
        ),
        core::sync::atomic::Ordering::Release,
    );

    // SAFETY: どちらも引数なし / 出力専用。
    let has_dump = unsafe { sys::esp_core_dump_image_check() } == sys::ESP_OK;
    if !has_dump {
        log::info!("coredump: none stored — last boot was clean");
        return;
    }

    let mut summary = sys::esp_core_dump_summary_t::default();
    // SAFETY: `summary` は正しい型・サイズで確保済み。
    let err = unsafe { sys::esp_core_dump_get_summary(&mut summary) };
    if err != sys::ESP_OK {
        log::error!("coredump: image present but summary read failed (err={err:#x})");
        return;
    }

    let task = {
        let bytes: Vec<u8> = summary
            .exc_task
            .iter()
            .take_while(|&&c| c != 0)
            .map(|&c| c as u8)
            .collect();
        String::from_utf8_lossy(&bytes).into_owned()
    };
    let cause = summary.ex_info.exc_cause;

    log::error!(
        "coredump: PREVIOUS CRASH — task '{task}' pc=0x{:08x} cause={cause} ({}) vaddr=0x{:08x}",
        summary.exc_pc,
        cause_name(cause),
        summary.ex_info.exc_vaddr,
    );

    let depth = (summary.exc_bt_info.depth as usize).min(summary.exc_bt_info.bt.len());
    if depth > 0 {
        let mut bt = String::new();
        for pc in &summary.exc_bt_info.bt[..depth] {
            bt.push_str(&format!("0x{pc:08x} "));
        }
        log::error!(
            "coredump: backtrace{} {bt}",
            if summary.exc_bt_info.corrupted {
                " (corrupted)"
            } else {
                ""
            }
        );
    }

    // 画面用は1行に圧縮 (パネル幅は30文字なので、タスク名と原因と PC だけ)。
    SUMMARY.store(&format!(
        "{task} {} pc={:08x}",
        cause_name(cause),
        summary.exc_pc
    ));

    // 次のクラッシュと区別できるよう消しておく。
    // SAFETY: 引数なし。
    let err = unsafe { sys::esp_core_dump_image_erase() };
    if err != sys::ESP_OK {
        log::warn!("coredump: erase failed (err={err:#x}) — next boot may report this same crash");
    }
}
