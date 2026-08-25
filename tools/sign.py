#!/usr/bin/env python3
"""Build pipeline: objcopy bootloader + app -> ed25519 sign -> merge.

Usage: python tools/sign.py
Produces in build/:

  boot.bin      raw bootloader binary (embassy-boot, links at 0x08000000)
  app.bin       raw application binary (active partition @0x08020000)
  app.dfu.bin   UPDATE PAYLOAD for UDP/WS/CAN: app.bin + 64-byte ed25519
                signature of SHA-512(app.bin) (embassy-boot/salty scheme)
  full.bin/hex  boot.bin padded with 0xFF to 0x08020000 + app.bin
                (flash at 0x08000000; manufacturing image)

The signing key is keys/ed25519.key (gitignored; generate with
tools/gen_ed25519.py). The public key and its SHA-256 (the keyhash every
upgrade channel checks) are compiled into the firmware
(proto::fw_upg::FW_PUBKEY / FW_KEYHASH) — if you regenerate the keypair,
update those constants and rebuild both binaries.
"""

import hashlib
import os
import subprocess
import sys

from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KEY = os.path.join(ROOT, "keys", "ed25519.key")
OUT = os.path.join(ROOT, "build")

# active partition start; boot.bin is padded with 0xFF up to here
APP_BASE = 0x08020000


def objcopy(bin_name, package):
    out = os.path.join(OUT, bin_name)
    subprocess.run(
        ["cargo", "objcopy", "--release", "--bin", package, "--", "-O", "binary", out],
        check=True,
        cwd=ROOT,
    )
    return out


def sign(app_bin_path, out_path):
    key = Ed25519PrivateKey.from_private_bytes(
        bytes.fromhex(open(KEY).read().strip()))
    image = open(app_bin_path, "rb").read()
    # pure Ed25519 over the 64-byte SHA-512 digest — what embassy-boot's
    # salty verifier checks on the device (verify(message = sha512(fw)))
    digest = hashlib.sha512(image).digest()
    sig = key.sign(digest)
    assert len(sig) == 64
    with open(out_path, "wb") as f:
        f.write(image)
        f.write(sig)
    pub = key.public_key().public_bytes(
        serialization.Encoding.Raw, serialization.PublicFormat.Raw)
    keyhash = hashlib.sha256(pub).hexdigest()
    print(f"ok: {out_path} ({os.path.getsize(out_path)} B = {len(image)} image + 64 sig)")
    print(f"    pubkey {pub.hex()}")
    print(f"    keyhash (must equal proto::fw_upg::FW_KEYHASH): {keyhash}")


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


def main():
    if not os.path.isfile(KEY):
        sys.exit(f"missing signing key {KEY} (generate with tools/gen_ed25519.py, never commit it)")
    os.makedirs(OUT, exist_ok=True)

    boot_bin = objcopy("boot.bin", "io-edge-hub-boot")
    app_bin = objcopy("app.bin", "io-edge-hub-fw")

    dfu_bin = os.path.join(OUT, "app.dfu.bin")
    sign(app_bin, dfu_bin)

    full_bin = os.path.join(OUT, "full.bin")
    subprocess.run(
        [sys.executable, os.path.join(ROOT, "tools", "merge_image.py"),
         boot_bin, app_bin, hex(APP_BASE - 0x08000000), full_bin],
        check=True,
    )
    write_ihex(full_bin, 0x08000000, os.path.join(OUT, "full.hex"))
    print(f"ok: {full_bin} ({os.path.getsize(full_bin)} B)")


if __name__ == "__main__":
    main()
