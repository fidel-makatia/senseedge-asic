#!/usr/bin/env python3
"""Host side of the SenseEdge Zybo protocol (sw/senseedge_zybo.c, host mode).

    link = open_link(args)            # --port /dev/cu.usbserial-XXXX [--baud 115200] | --mock
    link.identify()                   # {'id': 0x5E5E0002, 'protocol': 1, 'sw': 0, 'host_mode': 1}
    link.load_weights(w212)           # -> (read_back_errors, read_back np.int8[212])
    link.config(div=9, threshold=100, n_consec=3)
    link.load_record(codes[n, 64])    # into the emulator RAM, n <= 64 windows
    link.run(n, feats=True, bins=False)  # -> (status, [ {cls, conf, alarm, feat, bins}, ... ])
    link.peek(off); link.poke(off, val); link.reset(); link.quit()

Frames:  host -> board  A5 cmd len16 payload crc;  board -> host  5A status len16 payload crc,
crc = XOR of everything between the start byte and the crc.  --mock runs the firmware against
sw/mock/mock_chip.c in a subprocess (no board needed) for protocol tests.
"""
import os
import select
import struct
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
F_CLK = 20_000_000


class SeLinkError(Exception):
    pass


class SerialTransport:
    def __init__(self, port, baud):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.05)
        time.sleep(0.2)
        self.ser.reset_input_buffer()

    def write(self, b):
        self.ser.write(b); self.ser.flush()

    def read(self, n, timeout):
        out = b""; t_end = time.time() + timeout
        while len(out) < n and time.time() < t_end:
            out += self.ser.read(n - len(out))
        return out

    def close(self):
        self.ser.close()


class PipeTransport:
    """The mock firmware as a subprocess; its stdin/stdout are the UART."""
    def __init__(self, proc):
        self.proc = proc

    def write(self, b):
        self.proc.stdin.write(b); self.proc.stdin.flush()

    def read(self, n, timeout):
        out = b""; t_end = time.time() + timeout
        fd = self.proc.stdout.fileno()
        while len(out) < n:
            left = t_end - time.time()
            if left <= 0:
                break
            r, _, _ = select.select([fd], [], [], left)
            if not r:
                break
            chunk = os.read(fd, n - len(out))
            if not chunk:
                break
            out += chunk
        return out

    def close(self):
        try:
            self.proc.stdin.close(); self.proc.terminate()
        except Exception:
            pass


class SeLink:
    def __init__(self, transport, timeout=5.0):
        self.t = transport
        self.timeout = timeout

    # ---- framing
    def cmd(self, c, payload=b"", timeout=None):
        payload = bytes(payload)
        hdr = bytes([ord(c), len(payload) & 0xFF, len(payload) >> 8])
        crc = 0
        for b in hdr + payload:
            crc ^= b
        self.t.write(b"\xA5" + hdr + payload + bytes([crc]))
        timeout = self.timeout if timeout is None else timeout
        t_end = time.time() + timeout
        # resynchronise on the start byte (text-mode output may precede the first reply)
        while True:
            b = self.t.read(1, max(0.0, t_end - time.time()))
            if not b:
                raise SeLinkError(f"no reply to '{c}' within {timeout:.1f} s")
            if b == b"\x5A":
                break
        h = self.t.read(3, max(0.0, t_end - time.time()))
        if len(h) < 3:
            raise SeLinkError("short reply header")
        status, n = h[0], h[1] | (h[2] << 8)
        body = self.t.read(n + 1, max(0.5, t_end - time.time()))
        if len(body) < n + 1:
            raise SeLinkError(f"short reply payload ({len(body)} of {n + 1} bytes)")
        crc = 0
        for b in h + body[:-1]:
            crc ^= b
        if crc != body[-1]:
            raise SeLinkError("reply crc mismatch")
        if status in (0xFF, 0xFE, 0xFD, 0xFC):
            raise SeLinkError({0xFF: "unknown command", 0xFE: "payload too long", 0xFD: "crc rejected",
                               0xFC: "bad payload length"}[status] + f" ('{c}')")
        return status, body[:-1]

    # ---- commands
    def identify(self):
        _, p = self.cmd("I")
        i, v, sw = struct.unpack("<III", p[:12])
        return dict(id=i, protocol=v, sw=sw, host_mode=p[12])

    def load_weights(self, w):
        w = np.asarray(w).astype(np.int64)
        assert w.shape == (212,) and w.min() >= -128 and w.max() <= 127
        st, p = self.cmd("W", bytes(int(v) & 0xFF for v in w))
        return st, np.frombuffer(p, np.int8).copy()

    def config(self, div, threshold, n_consec):
        self.cmd("C", struct.pack("<HH", div, ((n_consec & 0xF) << 8) | (threshold & 0xFF)))

    def reset(self):
        st, _ = self.cmd("X")
        return st

    def load_record(self, codes):
        codes = np.asarray(codes, np.int64)
        assert codes.ndim == 2 and codes.shape[1] == 64 and 1 <= len(codes) <= 64 and codes.min() >= 0 and codes.max() <= 4095
        self.cmd("R", bytes([len(codes)]) + codes.astype("<u2").tobytes())

    def run(self, n, feats=True, bins=False, div=9, timeout=None):
        flags = (1 if feats else 0) | (2 if bins else 0)
        if timeout is None:
            timeout = 5.0 + 3.0 * n * 64 * 35 * (div + 1) / F_CLK + 0.02 * n
        st, p = self.cmd("G", bytes([n, flags]), timeout=timeout)
        done = p[0]
        rec = 3 + (8 if feats else 0) + (64 if bins else 0)
        out = []
        for k in range(done):
            q = p[1 + k * rec: 1 + (k + 1) * rec]
            d = dict(cls=q[0], conf=q[1], alarm=q[2], feat=None, bins=None)
            pos = 3
            if feats:
                d["feat"] = list(q[pos:pos + 8]); pos += 8
            if bins:
                d["bins"] = list(np.frombuffer(q[pos:pos + 64], "<u2").astype(int))
            out.append(d)
        return st, out

    def peek(self, off):
        _, p = self.cmd("D", struct.pack("<H", off))
        return struct.unpack("<I", p)[0]

    def poke(self, off, val):
        self.cmd("P", struct.pack("<HI", off, val & 0xFFFFFFFF))

    def quit(self):
        self.cmd("Q")

    def close(self):
        self.t.close()


def add_link_args(ap):
    ap.add_argument("--port", help="serial port of the Zybo USB-UART, e.g. /dev/cu.usbserial-210279A0B or COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mock", action="store_true", help="run against the mock firmware (no board)")


def open_link(args, timeout=5.0):
    if args.mock:
        exe = HERE.parent / "build" / "mock_fw"
        src = [HERE.parent / "sw" / "senseedge_zybo.c", HERE.parent / "sw" / "mock" / "mock_chip.c"]
        if not exe.exists() or exe.stat().st_mtime < max(s.stat().st_mtime for s in src):
            subprocess.run(["sh", str(HERE.parent / "sw" / "mock" / "build.sh")], check=True)
        proc = subprocess.Popen([str(exe)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=sys.stderr)
        return SeLink(PipeTransport(proc), timeout)
    if not args.port:
        sys.exit("give --port (the Zybo's USB-UART) or --mock")
    return SeLink(SerialTransport(args.port, args.baud), timeout)
