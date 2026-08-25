use std::env;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    let manifest = env::var("CARGO_MANIFEST_DIR").unwrap();
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());

    println!("cargo:rustc-link-search=native={manifest}");
    println!("cargo:rerun-if-changed={manifest}/memory.x");
    println!("cargo:rerun-if-changed={manifest}/ccram.x");

    // version banner: "vM.m.p_<git6>" like the firmware build
    let root = PathBuf::from(&manifest)
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf();
    let ver = std::fs::read_to_string(root.join("VERSION")).unwrap_or_else(|_| "0.0.0\n".into());
    let ver: String = ver.split_whitespace().next().unwrap_or("0.0.0").into();
    let git = match Command::new("git")
        .arg("rev-parse")
        .arg("--short=6")
        .arg("HEAD")
        .current_dir(&root)
        .output()
    {
        Ok(o) if o.status.success() => String::from_utf8_lossy(&o.stdout).trim().to_string(),
        _ => "000000".to_string(),
    };
    std::fs::write(
        out.join("boot_version.rs"),
        format!("pub const BOOT_BANNER: &str = \"io-edge-hub boot v{ver}_{git}\";\n"),
    )
    .unwrap();
    println!("cargo:rerun-if-changed={}", root.join("VERSION").display());
    println!(
        "cargo:rerun-if-changed={}",
        root.join(".git").join("refs").display()
    );
}
