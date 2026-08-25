#!/usr/bin/env python3
"""merge boot.bin + 0xFF padding + app image -> full.bin

    merge_image.py boot.bin app.bin <app-base-offset, e.g. 0x20000> full.bin
"""
import sys
boot = open(sys.argv[1], 'rb').read()
app = open(sys.argv[2], 'rb').read()
app_off = int(sys.argv[3], 0)
out = sys.argv[4]
pad = app_off - len(boot)
assert pad >= 0, 'boot.bin larger than the app base offset'
with open(out, 'wb') as f:
    f.write(boot)
    f.write(b'\xff' * pad)
    f.write(app)
total = len(boot) + pad + len(app)
print('full.bin: %d bytes (boot %d + pad %d + app %d)' % (total, len(boot), pad, len(app)))
