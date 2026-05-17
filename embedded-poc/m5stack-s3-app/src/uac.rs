//! USB Audio Class host capture — Phase 1 host install (#30).
//!
//! `start_host()` installs the ESP-IDF USB host stack + the
//! `espressif/usb_host_uac` class driver, registers a driver event
//! callback that logs hot-plug events, and spawns a dedicated thread
//! that pumps `usb_host_lib_handle_events()`. Opening a device and
//! pulling iso IN packets lands in #31; the resample + push to the
//! decoder ringbuf lands in #32.
//!
//! ## 接続方式 (確定済)
//!
//! - Component: `espressif/usb_host_uac@^1.4` を `Cargo.toml` の
//!   `extra_components` 経由で managed component として取得。
//!   bindings は `esp_idf_svc::sys::uac::*` に生成。
//! - IC-705 USB Audio は **固定 48 kHz / stereo / 16-bit** (16 kHz は
//!   selectable ではない)。S3 側で stereo → mono (L ch 抽出 / R ch 破棄)
//!   + 48 kHz → 12 kHz の 4:1 decimation を `embedded_shared` 経由で
//!   行う (#32)。
//! - Reference example は esp-usb 上流の
//!   `host/class/uac/usb_host_uac/examples/audio_player/main/main.c`。
//!   init 順序 (`usb_host_install` → events task spawn →
//!   `uac_host_install` → `RX_CONNECTED` 通知で device_open →
//!   device_start) をそのまま Rust に移植する。
//!
//! ## OTG 排他
//!
//! `usb_host_install()` を呼んだ瞬間に USB-Serial-JTAG endpoint が
//! detach されるため、`BootMode::Uac` では WiFi STA + UDP log を必ず
//! 起動させる (`main.rs` dispatch arm で強制)。
//!
//! ## 進捗
//!
//! - [x] managed component + bindings (#29)
//! - [x] host install + hot-plug callback (#30, このファイル)
//! - [ ] iso IN streaming (#31)
//! - [ ] 48 kHz stereo → 12 kHz mono resampler + ringbuf → decode (#32)
//! - [ ] verification on hardware (#33-#34)
//! - [ ] disconnect/reconnect polish (#35)

use anyhow::{anyhow, Result};
use esp_idf_svc::sys;

/// USB host event-pump task stack — modest budget; the loop just
/// blocks on `usb_host_lib_handle_events` and dispatches flags.
const USB_EVENTS_TASK_STACK: usize = 4096;

/// UAC class-driver background-task config. 1.4.x supports
/// `tskNO_AFFINITY` but pinning to core 0 (PRO_CPU) matches the upstream
/// audio_player example and keeps the decoder's core 1 (APP_CPU) free.
const UAC_DRIVER_TASK_STACK: usize = 4096;
const UAC_DRIVER_TASK_PRIORITY: usize = 5;
const UAC_DRIVER_TASK_CORE: sys::BaseType_t = 0;

/// Driver event callback. Invoked by the UAC class-driver background
/// task on every `RX_CONNECTED` / `TX_CONNECTED` notification (i.e.
/// every time an audio streaming interface enumerates).
///
/// #30 scope: just log the event so we can see IC-705 enumeration over
/// UDP. #31 will replace the body with "push a Connect message onto
/// the app channel so the app task can `uac_host_device_open` +
/// `uac_host_device_start`".
extern "C" fn driver_event_cb(
    addr: u8,
    iface_num: u8,
    event: sys::uac::uac_host_driver_event_t,
    _arg: *mut core::ffi::c_void,
) {
    let kind = match event {
        sys::uac::uac_host_driver_event_t_UAC_HOST_DRIVER_EVENT_RX_CONNECTED => "RX_CONNECTED",
        sys::uac::uac_host_driver_event_t_UAC_HOST_DRIVER_EVENT_TX_CONNECTED => "TX_CONNECTED",
        other => {
            log::warn!("uac: unknown driver event addr={addr} iface={iface_num} raw={other}");
            return;
        }
    };
    log::info!("uac: driver event addr={addr} iface={iface_num} kind={kind}");
}

/// USB host event-pump body. Blocks indefinitely on
/// `usb_host_lib_handle_events`. Returned `event_flags` are
/// intentionally ignored: `NO_CLIENTS` only fires when all class
/// drivers (= our UAC driver) have been uninstalled — never in our
/// sticky-install model — and `ALL_FREE` is only useful when
/// preparing to uninstall the host library, which we also never do.
/// Device hot-plug events surface through the class driver's own
/// callback ([`driver_event_cb`]), not these flags.
fn usb_events_task() {
    // `portMAX_DELAY` is a C macro (not bound by bindgen). FreeRTOS on
    // ESP-IDF defines `TickType_t` as `u32` so the all-ones value =
    // "block indefinitely until an event arrives" — exactly what we want.
    const FOREVER: sys::TickType_t = sys::TickType_t::MAX;
    loop {
        let mut event_flags: u32 = 0;
        let err = unsafe {
            sys::usb_host_lib_handle_events(FOREVER, &mut event_flags as *mut u32)
        };
        if err != sys::ESP_OK as sys::esp_err_t {
            // ESP_ERR_TIMEOUT は portMAX_DELAY では発生しないはず。
            // INVALID_STATE = host stack 未インストール → loop 即抜けるべき。
            log::error!("uac: usb_host_lib_handle_events err={err:#x}");
            if err == sys::ESP_ERR_INVALID_STATE as sys::esp_err_t {
                break;
            }
            // 他の error は理屈上発生しないが、起きた時 0 delay で tight
            // loop すると UDP log を埋め尽くす + lower-prio task を starve
            // する。50 ms sleep を挟んで recoverable 系を吸収する。
            esp_idf_svc::hal::delay::FreeRtos::delay_ms(50);
            continue;
        }
        let _ = event_flags;
    }
    log::error!("uac: usb_events_task exiting — host stack gone");
}

/// Install the USB host stack + the UAC class driver. Returns once
/// both are running in their respective background tasks; the caller
/// (main.rs UAC dispatch arm) falls through to the display loop.
pub fn start_host() -> Result<()> {
    log::info!("uac: installing USB host stack");
    let host_config = sys::usb_host_config_t {
        skip_phy_setup: false,
        root_port_unpowered: false,
        intr_flags: sys::ESP_INTR_FLAG_LEVEL1 as i32,
        enum_filter_cb: None,
        // peripheral_map = 0 → default peripheral (S3 has only one
        // USB-OTG controller so this is unambiguous).
        peripheral_map: 0,
        // Zero-init custom FIFO triggers the kconfig-bias default
        // (RX-heavy for host class drivers). Sufficient for iso IN
        // capture; we are not a high-bandwidth bulk consumer.
        fifo_settings_custom: sys::usb_host_config_t__bindgen_ty_1 {
            nptx_fifo_lines: 0,
            ptx_fifo_lines: 0,
            rx_fifo_lines: 0,
        },
    };
    let err = unsafe { sys::usb_host_install(&host_config as *const _) };
    if err != sys::ESP_OK as sys::esp_err_t {
        return Err(anyhow!("usb_host_install failed (err={err:#x})"));
    }

    // Spawn the events pump. std::thread runs FreeRTOS task underneath
    // via esp-idf-hal pthread shim; 4 KB stack covers the trivial loop.
    // If spawn fails after `usb_host_install` succeeded the host stack
    // is installed but no one is pumping `usb_host_lib_handle_events`,
    // leaving the controller in a half-up state. Roll back with
    // `usb_host_uninstall` so a subsequent BootMode flip (or even
    // re-entry on retry) finds a clean baseline.
    if let Err(e) = std::thread::Builder::new()
        .stack_size(USB_EVENTS_TASK_STACK)
        .name("usb_events".into())
        .spawn(usb_events_task)
    {
        let uninstall_err = unsafe { sys::usb_host_uninstall() };
        if uninstall_err != sys::ESP_OK as sys::esp_err_t {
            log::error!(
                "uac: usb_host_uninstall after spawn failure also failed (err={uninstall_err:#x}); host stack left in inconsistent state"
            );
        } else {
            log::info!("uac: rolled back usb_host_install after spawn failure");
        }
        return Err(anyhow!("usb_events_task spawn failed: {e}"));
    }
    log::info!("uac: usb_events_task spawned (stack={USB_EVENTS_TASK_STACK} B)");

    // UAC class driver: background-task mode (driver pumps its own
    // event queue + invokes our callback). The callback is light —
    // logs only in #30 — so the default priority is fine.
    log::info!("uac: installing UAC class driver");
    let uac_config = sys::uac::uac_host_driver_config_t {
        create_background_task: true,
        task_priority: UAC_DRIVER_TASK_PRIORITY,
        stack_size: UAC_DRIVER_TASK_STACK,
        core_id: UAC_DRIVER_TASK_CORE,
        callback: Some(driver_event_cb),
        callback_arg: core::ptr::null_mut(),
    };
    let err = unsafe { sys::uac::uac_host_install(&uac_config as *const _) };
    if err != sys::ESP_OK as sys::esp_err_t {
        // Roll back the host stack so the controller doesn't sit half-
        // up. The events pump task is already spawned and blocked in
        // `usb_host_lib_handle_events(FOREVER, ..)` at this point, so
        // calling `usb_host_uninstall` straight away tends to trip
        // `ESP_ERR_INVALID_STATE`. Call `usb_host_lib_unblock` first
        // to wake the pump (which then sees INVALID_STATE on its next
        // handle_events call and exits cleanly) before the uninstall.
        // Mirror the spawn-failure rollback's err-checked logging so a
        // "host stack left in inconsistent state" condition surfaces.
        let unblock_err = unsafe { sys::usb_host_lib_unblock() };
        if unblock_err != sys::ESP_OK as sys::esp_err_t {
            log::warn!(
                "uac: usb_host_lib_unblock before rollback returned err={unblock_err:#x}"
            );
        }
        let uninstall_err = unsafe { sys::usb_host_uninstall() };
        if uninstall_err != sys::ESP_OK as sys::esp_err_t {
            log::error!(
                "uac: usb_host_uninstall after uac_host_install failure also failed (err={uninstall_err:#x}); host stack left in inconsistent state"
            );
        } else {
            log::info!("uac: rolled back usb_host_install after uac_host_install failure");
        }
        return Err(anyhow!("uac_host_install failed (err={err:#x})"));
    }

    log::info!(
        "uac: host + class driver up — waiting for IC-705 enumeration (driver task core={UAC_DRIVER_TASK_CORE}, prio={UAC_DRIVER_TASK_PRIORITY})"
    );
    Ok(())
}
