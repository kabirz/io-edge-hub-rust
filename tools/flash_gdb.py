#!/usr/bin/env python3
"""Flash firmware over a remote GDB server (st-util / OpenOCD) without gdb.

Speaks just enough GDB Remote Serial Protocol to do what
`gdb -ex 'target remote HOST:PORT' -ex 'load'` does:

    halt -> vFlashErase (per flash sector) -> vFlashWrite -> vFlashDone
    -> read-back verify -> SYSRESETREQ (reboot through the bootloader)

Every phase is spot-checked (post-erase 0xFF, post-first-write read-back) so
a silently no-op'd flash op is caught immediately, and the target is ALWAYS
reset on the way out — success or failure — so the board never stays halted.

Usage:
    python tools/flash_gdb.py 10.84.9.190 4242 build/app.signed.bin [base]
    (base defaults to 0x08010000, the MCUboot slot0 app address)

Note: plain `st-util` serves one connection then exits — run it with --multi
(or restart it) between flashes.

CAVEAT: the st-util 1.8.0-121-g8c34a4e build on 10.84.9.190 (2026-03) writes
vFlashWrite payloads LITERALLY instead of hex-decoding them (probed: payload
"11223344" lands in flash as 8 ASCII bytes), so this script corrupted slot0
there before the problem was identified. On that box flash with st-flash
over ssh instead:

    scp build/app.signed.bin zhp@10.84.9.190:/tmp/
    ssh zhp@10.84.9.190 "st-flash erase 0x08010000 0x70000 && \
        st-flash write /tmp/app.signed.bin 0x08010000 && \
        st-flash read /tmp/rb.bin 0x08010000 <size> && md5sum /tmp/rb.bin && \
        st-flash reset"
"""
import socket
import sys
import time

# STM32F407 flash sector map: 4x16K, 1x64K, 7x128K from 0x08000000
F4_SECTORS = [(0x08000000 + 0x4000 * i, 0x4000) for i in range(4)] + [
    (0x08010000, 0x10000),
] + [(0x08020000 + 0x20000 * i, 0x20000) for i in range(7)]

AIRCR = 0xE000ED0C
SYSRESETREQ = 0x05FA0000 | (1 << 2)  # VECTKEY | SYSRESETREQ


def sectors_covering(base, size):
    out = []
    for addr, length in F4_SECTORS:
        if addr + length <= base or addr >= base + size:
            continue
        out.append((addr, length))
    return out


class Rsp:
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=5)
        self.sock.settimeout(5)
        self.buf = b""

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass

    def _fill(self, n):
        while len(self.buf) < n:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("gdb server closed the connection")
            self.buf += chunk

    def _byte(self):
        self._fill(1)
        b, self.buf = self.buf[:1], self.buf[1:]
        return b

    def read_packet(self, timeout=5):
        """Read one $data#cs packet; ack it; return data bytes."""
        self.sock.settimeout(timeout)
        while True:
            b = self._byte()
            if b == b"$":
                break
            # '+'/'-' acks and stray output are skipped
        data = b""
        while True:
            b = self._byte()
            data += b
            if data.endswith(b"#") and len(data) > 1:
                self._fill(2)
                csum = self.buf[:2]
                self.buf = self.buf[2:]
                body = data[:-1]
                if sum(body) & 0xFF == int(csum, 16):
                    self.sock.sendall(b"+")
                    return body
                self.sock.sendall(b"-")
                data = b""  # bad checksum: resync on next '$'

    def send_packet(self, data, timeout=5):
        """Send $data#cs, retry on '-', return the reply body."""
        frame = b"$" + data + b"#" + b"%02x" % (sum(data) & 0xFF)
        for attempt in range(3):
            self.sock.settimeout(timeout)
            self.sock.sendall(frame)
            ack = self._byte()
            if ack == b"+":
                return self.read_packet(timeout)
            if ack == b"$":
                # server skipped the ack and went straight to the reply
                # (seen right after an async stop/reset notification)
                self.buf = ack + self.buf
                return self.read_packet(timeout)
            if ack != b"-":
                raise ConnectionError(f"unexpected byte {ack!r} waiting for ack")
        raise ConnectionError("gdb server kept naking the packet")

    def read_mem(self, addr, n, timeout=10):
        r = self.send_packet(b"m%x,%x" % (addr, n), timeout=timeout)
        if r.startswith(b"E") or len(r) != 2 * n:
            raise ConnectionError(f"read 0x{addr:x}+{n} failed: {r!r}")
        return bytes.fromhex(r.decode())

    def sysreset(self):
        """AIRCR SYSRESETREQ while halted -> core resets and runs (no
        reset-vector catch armed, so MCUboot boots whatever is in slot0)."""
        try:
            r = self.send_packet(
                b"M%x,4:%s" % (AIRCR, SYSRESETREQ.to_bytes(4, "big").hex().encode()),
                timeout=10,
            )
            print(f"reset (AIRCR SYSRESETREQ): {r!r}")
            return True
        except (OSError, ConnectionError) as e:
            print(f"reset failed: {e}")
            return False


def hexs(b):
    return b.hex().encode()


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "10.84.9.190"
    port = int(sys.argv[2] if len(sys.argv) > 2 else 4242)
    path = sys.argv[3] if len(sys.argv) > 3 else "build/app.signed.bin"
    base = int(sys.argv[4], 0) if len(sys.argv) > 4 else 0x08010000

    image = open(path, "rb").read()
    print(f"flashing {path} ({len(image)} B) to 0x{base:08X} via {host}:{port}")

    rsp = Rsp(host, port)
    ok = False
    try:
        sup = rsp.send_packet(b"qSupported:multiprocess+", timeout=5)
        pkt_size = 2048
        if b"PacketSize=" in sup:
            pkt_size = int(sup.split(b"PacketSize=")[1].split(b";")[0], 16)
        chunk = max(256, min(1024, (pkt_size - 64) // 2))
        print(f"negotiated: packet {pkt_size} B -> write chunk {chunk} B")

        state = rsp.send_packet(b"?", timeout=5)
        print(f"target state: {state!r}")

        # halt (raw ETX break; reply should be a stop packet)
        rsp.sock.sendall(b"\x03")
        try:
            stopped = rsp.read_packet(timeout=10)
            print(f"halted: {stopped!r}")
        except (TimeoutError, socket.timeout):
            print("halt: no stop reply (already halted?) — continuing")

        before = rsp.read_mem(base, 16)
        print(f"flash @0x{base:08X} before: {before.hex()} (erased={before == b'\\xff' * 16})")

        secs = sectors_covering(base, len(image))
        for addr, length in secs:
            t0 = time.time()
            r = rsp.send_packet(b"vFlashErase:%x,%x" % (addr, length), timeout=90)
            if not r.startswith(b"OK"):
                raise RuntimeError(f"vFlashErase @0x{addr:x} failed: {r!r}")
            check = rsp.read_mem(addr, 16)
            mark = "OK" if check == b"\xff" * 16 else f"NOT ERASED ({check.hex()})"
            print(f"erase 0x{addr:08X} +0x{length:X} ({time.time() - t0:.1f}s) {mark}")

        t0 = time.time()
        for off in range(0, len(image), chunk):
            part = image[off : off + chunk]
            r = rsp.send_packet(b"vFlashWrite:%x:" % (base + off) + hexs(part), timeout=30)
            if not r.startswith(b"OK"):
                raise RuntimeError(f"vFlashWrite @0x{base + off:x} failed: {r!r}")
            if off == 0:  # first chunk: prove writes actually land
                got = rsp.read_mem(base, len(part))
                if got != part:
                    raise RuntimeError(
                        f"first write did not land: flash={got[:16].hex()} want={part[:16].hex()}"
                    )
                print("first chunk verified in place")
            done = off + len(part)
            if done % (64 * 1024) < chunk:
                print(f"write {done * 100 // len(image):3d}% ({done} B)")
        r = rsp.send_packet(b"vFlashDone", timeout=30)
        if not r.startswith(b"OK"):
            raise RuntimeError(f"vFlashDone failed: {r!r}")
        print(f"write done ({time.time() - t0:.1f}s), verifying read-back...")

        for off in range(0, len(image), 1024):
            n = min(1024, len(image) - off)
            if rsp.read_mem(base + off, n, timeout=15) != image[off : off + n]:
                raise RuntimeError(f"VERIFY MISMATCH @0x{base + off:x}")
        print("verify OK")
        ok = True
    except Exception as e:
        print(f"FLASH FAILED: {e}")
    finally:
        if ok:
            rsp.sysreset()
            print("FLASH OK — target rebooting")
        else:
            # never leave the core halted on a failed run: reset it so the
            # bootloader at least gets a chance to run the previous image
            print("attempting best-effort reset so the board is not left halted...")
            rsp.sysreset()
        rsp.close()
        sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
