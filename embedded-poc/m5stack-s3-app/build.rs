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

    // Phase 0.6: read cfg.toml (gitignored) for WiFi credentials and
    // UDP log target, expose them to the binary via env! macro at
    // compile time. Absence yields empty SSID → WiFi init is skipped
    // at runtime, so the build stays valid on machines without a
    // cfg.toml (CI / bootstrap).
    let cfg_path = manifest_dir.join("cfg.toml");
    println!("cargo:rerun-if-changed={}", cfg_path.display());

    // cfg.toml schema (extended Phase 1.7 with [station] section for
    // operator callsign + grid; build still succeeds with empty values).
    let (ssid, psk, pc_ip, port, my_call, my_grid) = if cfg_path.exists() {
        let txt = std::fs::read_to_string(&cfg_path)
            .unwrap_or_else(|e| panic!("read {}: {}", cfg_path.display(), e));
        let v: toml::Value = toml::from_str(&txt)
            .unwrap_or_else(|e| panic!("parse {}: {}", cfg_path.display(), e));
        let wifi = v
            .get("wifi")
            .and_then(|t| t.as_table())
            .unwrap_or_else(|| panic!("{} missing [wifi] section", cfg_path.display()));
        let ssid = wifi
            .get("ssid")
            .and_then(|s| s.as_str())
            .unwrap_or("")
            .to_string();
        let psk = wifi
            .get("psk")
            .and_then(|s| s.as_str())
            .unwrap_or("")
            .to_string();
        let pc_ip = wifi
            .get("pc_ip")
            .and_then(|s| s.as_str())
            .unwrap_or("255.255.255.255")
            .to_string();
        let port = wifi
            .get("port")
            .and_then(|p| p.as_integer())
            .map(|p| p as u16)
            .unwrap_or(9999);
        // [station] section is optional — empty defaults mean the QSO
        // FSM stays Idle (won't TX) until a real call/grid is provided.
        let station = v.get("station").and_then(|t| t.as_table());
        let my_call = station
            .and_then(|s| s.get("call"))
            .and_then(|s| s.as_str())
            .unwrap_or("")
            .to_string();
        let my_grid = station
            .and_then(|s| s.get("grid"))
            .and_then(|s| s.as_str())
            .unwrap_or("")
            .to_string();
        (ssid, psk, pc_ip, port, my_call, my_grid)
    } else {
        (
            String::new(),
            String::new(),
            "255.255.255.255".to_string(),
            9999u16,
            String::new(),
            String::new(),
        )
    };

    println!("cargo:rustc-env=WIFI_SSID={ssid}");
    println!("cargo:rustc-env=WIFI_PSK={psk}");
    println!("cargo:rustc-env=UDP_LOG_TARGET={pc_ip}");
    println!("cargo:rustc-env=UDP_LOG_PORT={port}");
    println!("cargo:rustc-env=MY_CALL={my_call}");
    println!("cargo:rustc-env=MY_GRID={my_grid}");
}
