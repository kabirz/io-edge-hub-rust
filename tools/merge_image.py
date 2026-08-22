#!/usr/bin/env python3
"""merge boot.bin + padding + app.signed.bin -> full.bin"""
import sys
boot = open(sys.argv[1], 'rb').read()
app = open(sys.argv[2], 'rb').read()
slot0 = int(sys.argv[3])
out = sys.argv[4]
pad = slot0 - len(boot)
assert pad >= 0, 'boot.bin > slot0 start'
with open(out, 'wb') as f:
    f.write(boot)
    f.write(b'\xff' * pad)
    f.write(app)
total = len(boot) + pad + len(app)
print('full.bin: %d bytes (boot %d + app %d)' % (total, len(boot), len(app)))
