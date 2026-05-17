// Extra bindgen header for the esp-idf-sys build. Identical to the
// ESP32-S3 PoC — esp-dsp is the same managed component on both chips.
#if defined(ESP_IDF_COMP_ESPRESSIF__ESP_DSP_ENABLED)
#include "esp_dsp.h"
#endif

// USB UAC host driver — only m5stack-s3-app pulls this component in
// (Core2 has no USB-OTG host). Guarded on the component-manager
// macro so Core2 builds don't try to include a header that isn't on
// the include path.
#if defined(ESP_IDF_COMP_ESPRESSIF__USB_HOST_UAC_ENABLED)
#include "usb/uac_host.h"
#endif
