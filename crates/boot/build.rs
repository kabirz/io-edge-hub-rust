use std::env;
use std::path::PathBuf;

fn main() {
    let manifest = env::var("CARGO_MANIFEST_DIR").unwrap();
    println!("cargo:rustc-link-search=native={manifest}");
    println!("cargo:rerun-if-changed={manifest}/memory.x");
    println!("cargo:rerun-if-changed={manifest}/ccram.x");
    // fold linker-script sizes into a generated constant included by main.rs:
    // editing memory.x then changes it, forcing a relink (cargo does not
    // track -T scripts and a stale link is a silent boot killer)
    let memx =
        std::fs::read_to_string(PathBuf::from(&manifest).join("memory.x")).unwrap_or_default();
    let ccmx =
        std::fs::read_to_string(PathBuf::from(&manifest).join("ccram.x")).unwrap_or_default();
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    std::fs::write(
        out.join("boot_layout.rs"),
        format!(
            "/// (memory.x bytes, ccram.x bytes) — content hash to force relinks.\n\
             #[allow(dead_code)]\npub const LINK_SCRIPTS: (usize, usize) = ({}, {});\n",
            memx.len(),
            ccmx.len()
        ),
    )
    .unwrap();
}
