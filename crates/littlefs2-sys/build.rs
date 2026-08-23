use std::env;
use std::path::PathBuf;

fn main() {
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    let littlefs_path = "littlefs";

    let mut builder = cc::Build::new();
    let builder = builder
        .flag("-std=c99")
        .flag("-DLFS_NO_DEBUG")
        .flag("-DLFS_NO_WARN")
        .flag("-DLFS_NO_ERROR")
        .flag("-DLFS_NO_ASSERT")
        .flag("-DLFS_NO_MALLOC")
        .include(littlefs_path)
        .include(&out_path)
        .file(format!("{littlefs_path}/lfs.c"))
        .file(format!("{littlefs_path}/lfs_util.c"));

    // the Rust crate always enables this: adds lfs_config.disk_version
    builder.flag("-DLFS_MULTIVERSION");

    builder.compile("lfs-sys");

    println!("cargo::rerun-if-changed={littlefs_path}/lfs.c");
    println!("cargo::rerun-if-changed={littlefs_path}/lfs.h");
    println!("cargo::rerun-if-changed={littlefs_path}/lfs_util.c");
    println!("cargo::rerun-if-changed={littlefs_path}/lfs_util.h");
}
