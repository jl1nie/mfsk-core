//! ESP-IDF の C 側ログ (`ESP_LOGx`) を Rust 側の [`LogFanout`] に流す。
//!
//! `FanoutLogger` が拾うのは `log` クレート経由の Rust 側ログだけで、
//! `HUB` / `EXT_PORT` / `ENUM` / `USBH` — つまり USB 列挙について
//! 知りたいことのほぼ全部 — は C 側から出ている。通常はそれで困らない:
//! C 側ログは UART / USB-Serial-JTAG に出るし、そこを読めばいい。
//!
//! ホストモードだけが例外で、`usb_host_install()` は USB-Serial-JTAG を
//! 巻き取ってしまうため、列挙が始まる時点でコンソールは既に無い
//! (過去の host モードのログが例外なく `Broken pipe` で終わっているのは
//! これ)。VBUS ゲートの都合でホストモードのボードは PC に繋がっても
//! いないので、UDP 以外に窓が無い。そこで、そのときだけ出力先を
//! 差し替える。
//!
//! **ペリフェラルモードでは呼ばないこと。** 元の vprintf は
//! `va_list` を消費してしまう都合で連鎖させていないので、入れた時点で
//! シリアルからは C 側ログが消える。生きているコンソールを潰す価値は
//! ない。
//!
//! Refs #163.

use core::ffi::{c_char, c_int};
use std::cell::Cell;

use esp_idf_svc::sys::va_list;

unsafe extern "C" {
    fn vsnprintf(s: *mut c_char, n: usize, format: *const c_char, arg: va_list) -> c_int;
}

/// C 側ログ1行の最大長。溢れた分は切り捨て。
const BUF: usize = 240;

thread_local! {
    /// 再入ガード。`FANOUT.push` は UDP 送信まで行くので、lwIP が
    /// その中で `ESP_LOGx` を叩くと無限再帰になる。フックは複数タスクから
    /// 並行に呼ばれる (`esp_log_set_vprintf` の doc が明記している) ので、
    /// グローバルなフラグだと他タスクの正当な行まで落ちる。
    static IN_HOOK: Cell<bool> = const { Cell::new(false) };
}

unsafe extern "C" fn hook(fmt: *const c_char, args: va_list) -> c_int {
    let reentered = IN_HOOK.with(|f| f.replace(true));
    if reentered {
        return 0;
    }

    let mut buf = [0u8; BUF];
    // SAFETY: `fmt` / `args` は C 側ロガーが用意したもの。`args` は
    // ここで一度だけ消費する (元の vprintf には渡さない)。
    let n = unsafe { vsnprintf(buf.as_mut_ptr() as *mut c_char, BUF, fmt, args) };

    if n > 0 {
        let len = (n as usize).min(BUF - 1);
        if let Ok(s) = core::str::from_utf8(&buf[..len]) {
            // C 側ログは行末に改行を持つ。fanout は1行1エントリなので剥がす。
            crate::FANOUT.push(s.trim_end_matches(['\r', '\n']));
        }
    }

    IN_HOOK.with(|f| f.set(false));
    n
}

/// C 側ログの出力先を fanout に差し替える。
pub fn install() {
    // SAFETY: 署名は `vprintf_like_t` と一致。
    unsafe {
        esp_idf_svc::sys::esp_log_set_vprintf(Some(hook));
    }
    log::info!("esp-idf C-side log output redirected into the fanout (UDP/LCD)");
}
