use std::env;
use std::path::PathBuf;

fn main() {
    embuild::espidf::sysenv::output();

    // Re-generate sdkconfig.gen.defaults so the partition CSV path is
    // anchored to *this* checkout's CARGO_MANIFEST_DIR (CMake resolves
    // CONFIG_PARTITION_TABLE_CUSTOM_FILENAME relative to its build dir,
    // so a relative path silently looks under `out/`). On a fresh clone
    // the file is missing and esp-idf-sys's build script — which runs
    // before this one — will fail; run `./bootstrap-sdkconfig.sh` once.
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let gen_path = manifest_dir.join("sdkconfig.gen.defaults");
    let partitions_csv = manifest_dir.join("partitions.csv");
    let body = format!(
        "CONFIG_PARTITION_TABLE_CUSTOM_FILENAME=\"{}\"\n",
        partitions_csv.display()
    );
    if std::fs::read_to_string(&gen_path).ok().as_deref() != Some(body.as_str()) {
        std::fs::write(&gen_path, &body)
            .unwrap_or_else(|e| panic!("write {}: {}", gen_path.display(), e));
    }
}
