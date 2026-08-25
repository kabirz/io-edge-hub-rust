#!/usr/bin/env python3
"""Flash build/full.bin (bootloader + signed app) via cargo-flash.

    python tools/flash.py            # flash the existing build/full.bin
    python tools/flash.py --build    # cargo build --release + tools/sign.py first
    python tools/flash.py --dry-run  # print the commands only

Executes:

    cargo flash --chip STM32F407VETx --path build/full.bin \
        --binary-format bin --base-address 0x8000000

full.bin = boot(64K) + signed app (slot0), so this rewrites the WHOLE
internal flash including the MCUboot bootloader (manufacturing image). For
an app-only update flash build/app.signed.bin at 0x08010000 instead.

Requires cargo-flash (cargo install cargo-flash) and the ST-Link attached
to the machine running this script — on the bench that is the Linux box,
not the Windows build host.
"""

import argparse
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BIN = os.path.join("build", "full.bin")
CMD = [
    "cargo", "flash", "--chip", "STM32F407VETx", "--path", BIN,
    "--binary-format", "bin", "--base-address", "0x8000000",
]


def run(cmd):
    print("+", " ".join(cmd), flush=True)
    return subprocess.run(cmd, cwd=ROOT).returncode


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--build", action="store_true",
                    help="run cargo build --release + tools/sign.py first")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the commands without executing")
    args = ap.parse_args()

    if args.build:
        steps = [
            ["cargo", "build", "--release"],
            [sys.executable, os.path.join("tools", "sign.py")],
        ]
        for c in steps:
            if args.dry_run:
                print("+", " ".join(c))
                continue
            if run(c) != 0:
                sys.exit(f"step failed: {' '.join(c)}")

    if args.dry_run:
        print("+", " ".join(CMD))
        return
    if not os.path.isfile(os.path.join(ROOT, BIN)):
        sys.exit(f"{BIN} not found — run with --build, or tools/sign.py first")
    sys.exit(run(CMD))


if __name__ == "__main__":
    main()
