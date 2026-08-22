use std::env;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    let manifest = env::var("CARGO_MANIFEST_DIR").unwrap();
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());

    println!("cargo:rustc-link-search=native={manifest}");
    println!("cargo:rerun-if-changed={manifest}/memory.x");

    // fw version string: "vM.m.p_<git6>" (same format as the C build, tools/gen_version)
    let root = PathBuf::from(&manifest).parent().unwrap().parent().unwrap().to_path_buf();
    let ver_raw = std::fs::read_to_string(root.join("VERSION")).unwrap_or_else(|_| "0.0.0 dev\n".into());
    let ver: String = ver_raw.split_whitespace().next().unwrap_or("0.0.0").to_string();
    let git = Command::new("git")
        .arg("rev-parse")
        .arg("--short=6")
        .current_dir(&root)
        .output();
    let git = match git {
        Ok(o) if o.status.success() => String::from_utf8_lossy(&o.stdout).trim().to_string(),
        _ => "000000".to_string(),
    };
    let fw_version = format!("v{ver}_{git}");
    // single-digit components for the Modbus version register (maj<<12|min<<8|patch)
    let parts: Vec<u32> = ver.split('.').map(|p| p.parse::<u32>().unwrap_or(0)).collect();
    let (maj, min, pat) = (
        *parts.first().unwrap_or(&0),
        *parts.get(1).unwrap_or(&0),
        *parts.get(2).unwrap_or(&0),
    );
    std::fs::write(
        out.join("fw_version.rs"),
        format!(
            "pub const FW_VERSION: &str = \"{fw_version}\";\n\
             pub const FW_MAJOR: u8 = {maj};\n\
             pub const FW_MINOR: u8 = {min};\n\
             pub const FW_PATCH: u8 = {pat};\n\
             pub const FW_GIT: &[u8; 6] = b\"{git}\";\n"
        ),
    )
    .unwrap();
    println!("cargo:rerun-if-changed={}", root.join("VERSION").display());
    println!("cargo:rerun-if-env-changed=FW_GIT_DIR");
}
