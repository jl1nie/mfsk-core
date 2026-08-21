use std::env;
use std::path::PathBuf;

fn main() {
    embuild::espidf::sysenv::output();

    // Anchor CONFIG_PARTITION_TABLE_CUSTOM_FILENAME to this checkout's
    // absolute path — same rationale as the sibling s3-app build.rs.
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

    // cfg.toml (gitignored) carries WiFi credentials + UDP log target.
    // Absent → empty SSID → WiFi init skipped, binary still boots.
    let cfg_path = manifest_dir.join("cfg.toml");
    println!("cargo:rerun-if-changed={}", cfg_path.display());

    let (ssid, psk, pc_ip, port, boot_mode) = if cfg_path.exists() {
        let txt = std::fs::read_to_string(&cfg_path)
            .unwrap_or_else(|e| panic!("read {}: {}", cfg_path.display(), e));
        let v: toml::Value =
            toml::from_str(&txt).unwrap_or_else(|e| panic!("parse {}: {}", cfg_path.display(), e));
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
        let boot_mode = v
            .get("app")
            .and_then(|t| t.as_table())
            .and_then(|t| t.get("boot_mode"))
            .and_then(|s| s.as_str())
            .unwrap_or("")
            .to_string();
        (ssid, psk, pc_ip, port, boot_mode)
    } else {
        (
            String::new(),
            String::new(),
            "255.255.255.255".to_string(),
            9999u16,
            String::new(),
        )
    };

    println!("cargo:rustc-env=WIFI_SSID={ssid}");
    println!("cargo:rustc-env=WIFI_PSK={psk}");
    println!("cargo:rustc-env=UDP_LOG_TARGET={pc_ip}");
    println!("cargo:rustc-env=UDP_LOG_PORT={port}");
    println!("cargo:rustc-env=BOOT_MODE_DEFAULT={boot_mode}");
}
