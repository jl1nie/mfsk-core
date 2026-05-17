// usb_host_uac bindgen header. Only m5stack-s3-app pulls this component
// in (Core2 has no USB-OTG host). The `bindings_module = "uac"` block in
// m5stack-s3-app/Cargo.toml's `extra_components` points here so the
// generated `esp_idf_svc::sys::uac::*` module contains *only* UAC class-
// driver symbols (cf. PR #91 review).
#include "usb/uac_host.h"
