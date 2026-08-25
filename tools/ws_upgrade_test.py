#!/usr/bin/env python3
"""WS firmware-upgrade regression (stdlib only, no third-party deps).

    python tools/ws_upgrade_test.py [ip] [build/app.dfu.bin]

Simulates exactly what the web page's 升级 button does: a minimal
WebSocket client performs the handshake, sends fw_start(size, keyhash)
with the keyhash the device reports via /api/info (so a key rotation only
touches the firmware), streams the payload in 10240-byte binary frames,
sends fw_end, then polls /api/info until the device finished the
embassy-boot swap. Note: triggers a real (same-image, auto-confirmed) swap.
"""
import base64
import hashlib
import json
import os
import socket
import struct
import sys
import time
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.12.101"
PATH = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, "build", "app.dfu.bin")


def resolve_keyhash(ip):
    """Device-reported keyhash first; repo public key as offline fallback."""
    try:
        info = json.load(urllib.request.urlopen(f"http://{ip}/api/info", timeout=3))
        kh = info.get("keyhash")
        if isinstance(kh, str) and len(kh) == 64:
            return bytes.fromhex(kh), "device /api/info"
    except Exception:
        pass
    pub = os.path.join(ROOT, "keys", "ed25519.pub")
    if os.path.isfile(pub):
        return hashlib.sha256(open(pub, "rb").read()).digest(), "keys/ed25519.pub"
    raise SystemExit("cannot resolve keyhash")


class Ws:
    def __init__(self, host):
        self.s = socket.create_connection((host, 80), timeout=10)
        key = base64.b64encode(os.urandom(16)).decode()
        req = (f"GET /ws HTTP/1.1\r\nHost: {host}\r\nUpgrade: websocket\r\n"
               f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
               "Sec-WebSocket-Version: 13\r\n\r\n")
        self.s.sendall(req.encode())
        buf = b""
        while b"\r\n\r\n" not in buf:
            buf += self.s.recv(4096)
        head, _, rest = buf.partition(b"\r\n\r\n")
        assert b"101" in head.split(b"\r\n")[0], head
        expect = base64.b64encode(hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest())
        assert expect in head, "handshake accept mismatch"
        self.buf = rest

    def _read(self, n):
        while len(self.buf) < n:
            self.buf += self.s.recv(65536)
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def recv_text(self, timeout=20):
        self.s.settimeout(timeout)
        while True:
            h = self._read(2)
            fin_op = h[0]
            ln = h[1] & 0x7F
            if ln == 126:
                ln = struct.unpack(">H", self._read(2))[0]
            elif ln == 127:
                ln = struct.unpack(">Q", self._read(8))[0]
            payload = self._read(ln)
            op = fin_op & 0x0F
            if op == 0x1:
                return json.loads(payload.decode())
            # io/regs pushes (text) etc: keep looping for our reply

    def send(self, payload, binary=False):
        op = 0x2 if binary else 0x1
        mask = os.urandom(4)
        ln = len(payload)
        if ln < 126:
            hdr = struct.pack(">BB", 0x80 | op, 0x80 | ln)
        elif ln < 65536:
            hdr = struct.pack(">BBH", 0x80 | op, 0x80 | 126, ln)
        else:
            hdr = struct.pack(">BBQ", 0x80 | op, 0x80 | 127, ln)
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        self.s.sendall(hdr + mask + masked)

    def close(self):
        try:
            self.s.close()
        except OSError:
            pass


def main():
    img = open(PATH, "rb").read()
    keyhash, src = resolve_keyhash(IP)
    print(f"payload {len(img)} B, keyhash {keyhash.hex()[:16]}... ({src})")
    ws = Ws(IP)
    t0 = time.time()

    ws.send(json.dumps({"cmd": "fw_start", "size": len(img),
                        "keyhash": base64.b64encode(keyhash).decode()}).encode())
    # fw_start reply comes after the erase; skip push frames until "t" absent
    while True:
        r = ws.recv_text(timeout=25)
        if "t" not in r:
            break
    print(f"fw_start -> {r}")
    assert r.get("ok") is True, r

    off = 0
    while off < len(img):
        ws.send(img[off:off + 10240], binary=True)
        off += 10240
    print(f"sent {off} B in {time.time() - t0:.1f}s")

    ws.send(json.dumps({"cmd": "fw_end"}).encode())
    while True:
        r = ws.recv_text(timeout=25)
        if "cmd" not in r and "t" not in r:
            break
    print(f"fw_end -> {r}")
    assert r.get("ok") is True, r
    ws.close()

    print("waiting for reboot + swap (~35s)...")
    deadline = time.time() + 120
    while time.time() < deadline:
        try:
            info = json.load(urllib.request.urlopen(f"http://{IP}/api/info", timeout=2))
            if info["uptime_ms"] < 60000:
                print(f"ONLINE: {info['version']} uptime {info['uptime_ms']}ms -> PASS")
                return 0
        except Exception:
            pass
        time.sleep(2)
    print("not back online")
    return 1


if __name__ == "__main__":
    sys.exit(main())
