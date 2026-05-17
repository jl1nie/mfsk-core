// esp-dsp bindgen header. Only the `bindings_module = "esp_dsp"` block
// in each crate's Cargo.toml `extra_components` points here, so the
// generated module contains *only* esp-dsp symbols — no cross-pollination
// with other managed components (cf. PR #91 review: the previous shared
// `bindings.h` caused both `esp_dsp` and `uac` modules to contain the
// full union of bindings, ballooning compile time + bindings.rs size).
#include "esp_dsp.h"
