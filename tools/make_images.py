#!/usr/bin/env python3
"""Produce flashable images from a release build (embassy-boot layout).

Replaces the MCUboot-era sign.py: images are plain binaries, no imgtool,
no signing step (integrity = CRC16 readback verify in-app; trial boot +
revert protects against bad swaps).

Outputs into build/:
  boot.bin  bootloader          -> flash at 0x08000000
  app.bin   application         -> flash at 0x08020000
  full.bin  boot + pad + app    -> one-shot factory image at 0x08000000
"""

import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TARGET = os.path.join(ROOT, "target", "thumbv7em-none-eabihf", "release")
BUILD = os.path.join(ROOT, "build")

OBJCOPY = os.environ.get(
    "RUST_OBJCOPY", os.path.join(os.path.expanduser("~"), ".cargo", "bin", "rust-objcopy")
)

BOOT_BASE = 0x08000000
ACTIVE_BASE = 0x08020000
BOOT_LEN = ACTIVE_BASE - BOOT_BASE   # 128 KiB reservation
ACTIVE_LEN = 0x08080000 - ACTIVE_BASE  # 384 KiB (three 128K sectors)


def objcopy(elf: str, bin_path: str) -> None:
    subprocess.run([OBJCOPY, "-O", "binary", elf, bin_path], check=True)


def main() -> int:
    if not os.path.exists(OBJCOPY):
        print(f"rust-objcopy not found at {OBJCOPY} (set RUST_OBJCOPY)", file=sys.stderr)
        return 1
    os.makedirs(BUILD, exist_ok=True)

    boot_elf = os.path.join(TARGET, "io-edge-hub-boot")
    fw_elf = os.path.join(TARGET, "io-edge-hub-fw")
    for elf in (boot_elf, fw_elf):
        if not os.path.exists(elf):
            print(f"missing {elf} — run `cargo build --release` first", file=sys.stderr)
            return 1

    boot_bin = os.path.join(BUILD, "boot.bin")
    app_bin = os.path.join(BUILD, "app.bin")
    objcopy(boot_elf, boot_bin)
    objcopy(fw_elf, app_bin)

    boot_sz = os.path.getsize(boot_bin)
    app_sz = os.path.getsize(app_bin)
    if boot_sz > BOOT_LEN:
        print(f"boot image {boot_sz} B exceeds {BOOT_LEN} B partition", file=sys.stderr)
        return 1
    if app_sz > ACTIVE_LEN:
        print(f"app image {app_sz} B exceeds {ACTIVE_LEN} B ACTIVE slot", file=sys.stderr)
        return 1

    full_bin = os.path.join(BUILD, "full.bin")
    with open(full_bin, "wb") as out:
        with open(boot_bin, "rb") as f:
            out.write(f.read())
        out.write(b"\xFF" * (BOOT_LEN - boot_sz))
        with open(app_bin, "rb") as f:
            out.write(f.read())

    print(f"boot {boot_sz} B / {BOOT_LEN}   -> probe-rs download --base-address 0x{BOOT_BASE:08X} build/boot.bin")
    print(f"app  {app_sz} B / {ACTIVE_LEN}   -> probe-rs download --base-address 0x{ACTIVE_BASE:08X} build/app.bin")
    print(f"full {os.path.getsize(full_bin)} B -> probe-rs download --base-address 0x{BOOT_BASE:08X} build/full.bin")
    return 0


if __name__ == "__main__":
    sys.exit(main())
