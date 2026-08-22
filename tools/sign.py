#!/usr/bin/env python3
"""Build pipeline: objcopy -> imgtool sign -> merge full image.

Usage: python tools/sign.py [--release]
Produces build/app.signed.bin (flash at 0x08010000) and build/full.bin/hex
(flash at 0x08000000, includes unchanged MCUboot bootloader).

Signing parameters are byte-identical to the C repo pipeline (CMakeLists.txt
add_custom_command, imgtool sign): RSA-2048, --header-size 512 --pad-header
--align 8 --slot-size 0x70000 --max-sectors 120 --erased-val 0xff.
"""
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMGTOOL = os.environ.get(
    "IMGTOOL", r"C:\Users\jxwaz\code\io-edge-hub-freertos\deps\mcuboot\scripts\imgtool.py")
KEY = os.path.join(ROOT, "keys", "root-rsa2048.pem")
BIN = os.path.join(ROOT, "target", "thumbv7em-none-eabihf", "release", "io-edge-hub-fw.bin")
OBJCOPY = os.path.join(os.path.expanduser("~"), ".cargo", "bin", "rust-objcopy.exe")


def main():
    if not os.path.isfile(KEY):
        sys.exit(f"missing signing key {KEY} (copy it from the C repo, never commit it)")

    out_dir = os.path.join(ROOT, "build")
    os.makedirs(out_dir, exist_ok=True)
    app_bin = os.path.join(out_dir, "app.bin")
    signed = os.path.join(out_dir, "app.signed.bin")

    subprocess.run([
        "cargo", "objcopy", "--release", "--bin", "io-edge-hub-fw",
        "--", "-O", "binary", app_bin,
    ], check=True, cwd=ROOT)

    version = open(os.path.join(ROOT, "VERSION")).read().split()[0]
    subprocess.run([
        sys.executable, IMGTOOL, "sign", app_bin, signed,
        "--key", KEY,
        "--header-size", "512", "--pad-header",
        "--align", "8", "--version", version,
        "--slot-size", "0x70000", "--max-sectors", "120",
        "--erased-val", "0xff",
    ], check=True)

    full_bin = os.path.join(out_dir, "full.bin")
    subprocess.run([
        sys.executable, os.path.join(ROOT, "tools", "merge_image.py"),
        os.path.join(ROOT, "assets", "boot.bin"), signed, "65536", full_bin,
    ], check=True)
    write_ihex(full_bin, 0x08000000, os.path.join(out_dir, "full.hex"))
    print(f"ok: {signed} ({os.path.getsize(signed)} B), {full_bin} ({os.path.getsize(full_bin)} B)")


def write_ihex(bin_path, base, hex_path):
    """Minimal Intel HEX writer (binary -> contiguous records at `base`)."""
    data = open(bin_path, "rb").read()
    lines = []
    addr_hi = None
    for off in range(0, len(data), 32):
        chunk = data[off:off + 32]
        addr = base + off
        hi = addr >> 16
        if hi != addr_hi:
            addr_hi = hi
            rec = bytes([0x02, 0x00, 0x00, 0x04]) + hi.to_bytes(2, "big")
            lines.append(":" + rec.hex().upper() + f"{(-sum(rec)) & 0xFF:02X}")
        lo = addr & 0xFFFF
        rec = bytes([len(chunk), lo >> 8, lo & 0xFF, 0x00]) + chunk
        lines.append(":" + rec.hex().upper() + f"{(-sum(rec)) & 0xFF:02X}")
    lines.append(":00000001FF")
    with open(hex_path, "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
