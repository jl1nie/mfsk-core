//! What two cores of the CoreS3 share: thin shim over `embedded_shared::apps::mem_bench`, whose module doc has the predictions. Logs only through
//! `embedded-poc/scripts/flash-monitor.sh` / `capture.sh`.
//!
//! Build: `cargo build --release --features ft8-mem-bench --bin ft8-mem-bench`.

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    embedded_shared::apps::compute_bench::init_logger_once();
    embedded_shared::apps::mem_bench::run()
}
