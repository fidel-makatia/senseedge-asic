#!/usr/bin/env python3
"""Log the Zybo's text-mode UART output (the banner, the self-test verdict, CLASS lines).

  uart_log.py --port COM10 [--baud 115200] [--seconds 20] [--out results/selftest_uart.txt]
              [--until "SELF-TEST"] [--lines N]
Stops after --seconds, or once a line containing --until has been seen and --lines further
lines have followed (default 6).  Every line is time-stamped relative to the first byte.
"""
import argparse
import sys
import time
from pathlib import Path

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True); ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=20.0); ap.add_argument("--out")
    ap.add_argument("--until", default=None); ap.add_argument("--lines", type=int, default=6)
    a = ap.parse_args()
    ser = serial.Serial(a.port, a.baud, timeout=0.1)
    ser.reset_input_buffer()
    t0 = None; buf = b""; out = []; seen = False; after = 0
    t_end = time.time() + a.seconds
    while time.time() < t_end:
        chunk = ser.read(256)
        if chunk:
            if t0 is None:
                t0 = time.time()
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("ascii", "backslashreplace").rstrip("\r")   # console-safe on Windows
                stamp = f"{time.time() - t0:8.3f}  {text}"
                print(stamp, flush=True); out.append(stamp)
                if a.until and a.until in text:
                    seen = True
                elif seen:
                    after += 1
            if seen and after >= a.lines:
                break
    ser.close()
    if a.out:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text("\n".join(out) + "\n")
    if not out:
        print("no UART output received", file=sys.stderr)
        sys.exit(2)


if __name__ == "__main__":
    main()
