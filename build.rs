use std::env;
use std::fs::{self, File};
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");

    // WiFi credentials from gitignored wifi.toml; fall back to the checked-in
    // example so a fresh checkout (which has no real credentials) still
    // compiles. With placeholders the chip simply never joins at runtime.
    println!("cargo:rerun-if-changed=wifi.toml");
    println!("cargo:rerun-if-changed=wifi.toml.example");
    let wifi_raw = match fs::read_to_string("wifi.toml") {
        Ok(s) => s,
        Err(_) => {
            println!(
                "cargo:warning=wifi.toml not found; using wifi.toml.example placeholders (wifi will not connect at runtime)"
            );
            fs::read_to_string("wifi.toml.example")
                .expect("neither wifi.toml nor wifi.toml.example exists")
        }
    };

    let mut ssid: Option<String> = None;
    let mut password: Option<String> = None;
    for line in wifi_raw.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some((k, v)) = line.split_once('=') {
            let v = v.trim().trim_matches('"').to_string();
            match k.trim() {
                "ssid" => ssid = Some(v),
                "password" => password = Some(v),
                _ => {}
            }
        }
    }
    let ssid = ssid.expect("wifi.toml missing `ssid` key");
    let password = password.expect("wifi.toml missing `password` key");

    let wifi_rs = format!(
        "pub const WIFI_NETWORK: &str = {:?};\npub const WIFI_PASSWORD: &str = {:?};\n",
        ssid, password,
    );
    fs::write(out.join("wifi.rs"), wifi_rs).unwrap();
}
