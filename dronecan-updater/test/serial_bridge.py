#!/usr/bin/env python3
"""Relays bytes between a serial port and stdin/stdout, standing in for Web Serial in hw-update.mjs.
Exits as soon as either side closes, so a stale bridge never keeps the port open."""
import os
import sys
import threading

import serial

port = serial.Serial(sys.argv[1], 115200, timeout=0.01)
port.dtr = port.rts = True


def to_stdout():
    out = sys.stdout.buffer
    try:
        while True:
            data = port.read(4096)
            if data:
                out.write(data)
                out.flush()
    except Exception:
        pass
    os._exit(0)


threading.Thread(target=to_stdout, daemon=True).start()
fd = sys.stdin.fileno()
try:
    while True:
        data = os.read(fd, 4096)
        if not data:
            break
        port.write(data)
except Exception:
    pass
os._exit(0)
