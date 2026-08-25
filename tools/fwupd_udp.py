#!/usr/bin/env python3
"""Standalone UDP firmware-upgrade client (no test-suite dependencies).

    python tools/fwupd_udp.py [ip] [build/app.dfu.bin]

Runs the full cycle against the device: START(size+keyhash) -> DATA_V2
go-back-N windows -> END(crc16 + ed25519 verified on device) -> REBOOT,
then polls GET_VERSION until the box finished the embassy-boot swap and is
back online.

The keyhash is fetched from the device itself over this same UDP channel
(cmd 0x15 GET_KEYHASH: SHA-256 of the ed25519 public key baked into the
firmware), so rotating the signing key only touches the firmware. Offline
fallback: keys/ed25519.pub next to the repo, hashed on the fly.

Requires only Python 3 (no third-party packages). Run it from a host on
the device's /24 — the upgrade commands are dropped cross-subnet.
"""
import hashlib
import os
import socket
import struct
import sys
import time

V2_WINDOW = 8
V2_ACK_TMO = 1.0
V2_MAX_RETRIES = 8
PORT = 8600

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def resolve_keyhash(ip, sock):
    """keyhash the DEVICE checks — ask it via UDP 0x15 (same channel the
    upgrade runs on); fall back to hashing the repo public key."""
    try:
        sock.settimeout(2.0)
        sock.sendto(b"\x15", (ip, PORT))
        r, _ = sock.recvfrom(64)
        if len(r) >= 33 and r[0] == 0x15:
            return bytes(r[1:33]), "device UDP 0x15"
    except OSError:
        pass
    pub = os.path.join(ROOT, "keys", "ed25519.pub")
    if os.path.isfile(pub):
        return hashlib.sha256(open(pub, "rb").read()).digest(), "keys/ed25519.pub"
    raise SystemExit("cannot resolve keyhash: device not answering 0x15 and keys/ed25519.pub missing")


def crc16_ccitt(data):
    """Reflected poly-0x1021/init-0 (Zephyr sys/crc.h crc16_ccitt)."""
    seed = 0
    for b in data:
        e = (seed ^ b) & 0xFF
        f = (e ^ (e << 4)) & 0xFF
        seed = ((seed >> 8) ^ (f << 8) ^ (f << 3) ^ (f >> 4)) & 0xFFFF
    return seed


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.12.101"
    path = sys.argv[2] if len(sys.argv) > 2 else "build/app.dfu.bin"
    img = open(path, "rb").read()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst = (ip, PORT)

    keyhash, src = resolve_keyhash(ip, s)
    print(f"image: {path} ({len(img)} B incl 64B sig), keyhash {keyhash.hex()[:16]}... ({src})")

    def xfer(payload, tmo):
        s.settimeout(tmo)
        s.sendto(payload, dst)
        return s.recvfrom(2048)[0]

    t0 = time.time()
    r = xfer(bytes([0x01]) + struct.pack("<I", len(img)) + keyhash, 30.0)
    status, chunk = r[1], struct.unpack("<H", r[2:4])[0]
    assert status == 1, f"START rejected status={status} (1=ok 2=keyhash 0=other)"
    print(f"START ok, v2 chunk={chunk} B (DFU erase done)")

    total, off, retries = len(img), 0, 0
    while off < total:
        win_end = min(off + V2_WINDOW * chunk, total)
        w = off
        while w < win_end:
            n = min(chunk, total - w)
            s.sendto(bytes([0x06]) + struct.pack("<I", w) + img[w:w + n], dst)
            w += n
        deadline = time.time() + V2_ACK_TMO
        confirmed = off
        while confirmed < win_end and time.time() < deadline:
            try:
                s.settimeout(max(0.05, deadline - time.time()))
                r, _ = s.recvfrom(64)
            except socket.timeout:
                break
            if len(r) >= 5 and r[0] == 0x06:
                roff = struct.unpack("<I", r[1:5])[0]
                confirmed = max(confirmed, min(roff, total))
        assert confirmed > off or retries < V2_MAX_RETRIES, "window stalled"
        if confirmed >= win_end:
            off, retries = confirmed, 0
            continue
        retries += 1
        off = confirmed
        print(f"  retry at {off}/{total} (#{retries})")
    print(f"DATA_V2 done: {off}/{total} B in {time.time() - t0:.1f}s")

    t1 = time.time()
    r = xfer(bytes([0x03, 0]) + struct.pack("<H", crc16_ccitt(img)), 30.0)
    print(f"END -> ok={r[1]} (CRC readback + ed25519 verify {time.time() - t1:.1f}s)")
    assert r[1] == 1, "END rejected (crc/signature)"

    try:
        r = xfer(bytes([0x05]), 3.0)
        print("REBOOT ->", r.hex())
    except socket.timeout:
        print("REBOOT: already rebooting")

    print("waiting for the swap to finish (~30 s measured)...")
    deadline = time.time() + 150
    while time.time() < deadline:
        try:
            s.settimeout(2.0)
            s.sendto(b"\x04", dst)
            r, _ = s.recvfrom(256)
            # fixed 14-byte reply: [04]["v0.3.0_"][git 6B ascii]
            ver = r[1:14].decode(errors="replace")
            print(f"ONLINE after {time.time() - t0:.1f}s total, version {ver}")
            return 0
        except (socket.timeout, OSError):
            time.sleep(1.0)
    print("ERROR: not back online within 150 s")
    return 1


if __name__ == "__main__":
    sys.exit(main())
