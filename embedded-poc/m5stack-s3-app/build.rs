use std::env;
use std::path::PathBuf;

fn main() {
    embuild::espidf::sysenv::output();

    // Phase 0.6: read cfg.toml (gitignored) for WiFi credentials and
    // UDP log target, expose them to the binary via env! macro at
    // compile time. Absence yields empty SSID → WiFi init is skipped
    // at runtime, so the build stays valid on machines without a
    // cfg.toml (CI / bootstrap).
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let cfg_path = manifest_dir.join("cfg.toml");
    println!("cargo:rerun-if-changed={}", cfg_path.display());

    let (ssid, psk, pc_ip, port) = if cfg_path.exists() {
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
        (ssid, psk, pc_ip, port)
    } else {
        (
            String::new(),
            String::new(),
            "255.255.255.255".to_string(),
            9999u16,
        )
    };

    println!("cargo:rustc-env=WIFI_SSID={ssid}");
    println!("cargo:rustc-env=WIFI_PSK={psk}");
    println!("cargo:rustc-env=UDP_LOG_TARGET={pc_ip}");
    println!("cargo:rustc-env=UDP_LOG_PORT={port}");
}
