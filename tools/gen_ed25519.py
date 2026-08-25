#!/usr/bin/env python3
"""Generate the firmware-update ed25519 signing keypair (if missing).

Creates keys/ed25519.key (seed hex, private - gitignored, never commit),
keys/ed25519.pub (raw 32-byte public key, safe to publish) and
keys/ed25519.keyhash (32-byte sha256(pubkey) — copy next to the host-tool
exe for CAN upgrades; UDP/web hosts fetch the keyhash from the device).
Prints:

  PUBKEY_HEX   - raw public key, baked into proto::fw_upg::FW_PUBKEY
  KEYHASH_HEX  - sha256(public key), baked into proto::fw_upg::FW_KEYHASH
"""
import hashlib
import os
import sys

from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KEY = os.path.join(ROOT, "keys", "ed25519.key")
PUB = os.path.join(ROOT, "keys", "ed25519.pub")
KEYHASH = os.path.join(ROOT, "keys", "ed25519.keyhash")


def main():
    os.makedirs(os.path.join(ROOT, "keys"), exist_ok=True)
    if os.path.isfile(KEY):
        priv = Ed25519PrivateKey.from_private_bytes(bytes.fromhex(open(KEY).read().strip()))
    else:
        priv = Ed25519PrivateKey.generate()
        with open(KEY, "w") as f:
            f.write(priv.private_bytes(
                serialization.Encoding.Raw,
                serialization.PrivateFormat.Raw,
                serialization.NoEncryption(),
            ).hex())
        os.chmod(KEY, 0o600)
    pub = priv.public_key().public_bytes(
        serialization.Encoding.Raw, serialization.PublicFormat.Raw)
    with open(PUB, "wb") as f:
        f.write(pub)
    keyhash = hashlib.sha256(pub).digest()
    with open(KEYHASH, "wb") as f:
        f.write(keyhash)
    print("PUBKEY_HEX  ", pub.hex())
    print("KEYHASH_HEX ", keyhash.hex())
    print(f"keyhash file: {KEYHASH} (copy next to the host-tool exe for CAN)")


if __name__ == "__main__":
    sys.exit(main())
