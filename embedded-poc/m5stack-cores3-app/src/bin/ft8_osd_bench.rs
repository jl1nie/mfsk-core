//! FT8's OSD on the CoreS3, the host's port against the embedded one: thin shim over `embedded_shared::apps::osd_bench`, whose module doc has the predictions. Logs only through
//! `embedded-poc/scripts/flash-monitor.sh` / `capture.sh`.
//!
//! Build: `cargo build --release --features ft8-osd-bench --bin ft8-osd-bench`.

const INPUTS: &[u8] = include_bytes!("../../../assets/osd_micro_in.bin");

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    embedded_shared::apps::compute_bench::init_logger_once();
    embedded_shared::apps::osd_bench::run("m5stack-cores3", INPUTS)
}
