#!/usr/bin/env python3
"""Protocol test of sw/senseedge_zybo.c against the mock chip (sw/mock), no board needed:
every command, framing, crc, lengths, the weight round trip, the record path into the
emulator RAM, the per-window record layout of 'G' (with and without features and bins), the
reset-and-reload sequence, peek/poke, and the return to text mode.
Run:  python3 test_protocol_mock.py
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from se_link import open_link  # noqa: E402


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--mock", action="store_true", default=True)
    ap.add_argument("--port"); ap.add_argument("--baud", type=int, default=115200)
    a = ap.parse_args()
    rng = np.random.RandomState(3)
    link = open_link(a)
    fails = 0

    def check(name, ok):
        nonlocal fails
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
        fails += 0 if ok else 1

    info = link.identify()
    check("identify: id 0x5E5E0002, protocol 1", info["id"] == 0x5E5E0002 and info["protocol"] == 1)

    w = rng.randint(-128, 128, size=212)
    st, rb = link.load_weights(w)
    check("weights: 212 bytes, 0 read-back errors, read-back equal", st == 0 and (rb.astype(np.int64) == w).all())

    link.config(div=9, threshold=100, n_consec=3)
    check("config: CLK_DIV reads 9, ALARM_CFG reads (3<<8)|100", link.peek(0x1C) == 9 and link.peek(0x0C) == (3 << 8) | 100)

    codes = rng.randint(0, 4096, size=(5, 64))
    link.load_record(codes)
    ram = np.array([link.peek(0x2000 + 4 * i) for i in range(5 * 64)])
    check("record: 5 windows land in the emulator RAM, EMU_LEN = 320, emulator selected",
          (ram == codes.flatten()).all() and link.peek(0x1004) == 320 and (link.peek(0x1000) & 1) == 1)

    st, res = link.run(5, feats=True, bins=True, div=9)
    ok = st == 0 and len(res) == 5
    for k, d in enumerate(res):
        exp_feat = [int(codes.flatten()[64 * k + i]) & 0xFF for i in range(8)]
        exp_bins = [(int(codes.flatten()[64 * k + i]) ^ i) & 0xFFFF for i in range(32)]
        ok &= d["cls"] == (k & 3) and d["conf"] == 100 + (k & 127) and d["alarm"] == ((k >> 2) & 1)
        ok &= d["feat"] == exp_feat and d["bins"] == exp_bins
    check("run: 5 windows with features and bins decoded as the mock chip produced them", ok)
    check("run: weights reloaded after the reset inside 'G'", (np.array([link.peek(0x20 + 4 * i) for i in range(53)]) ==
          np.frombuffer(bytes(int(v) & 0xFF for v in w), "<u4")).all())

    st, res = link.run(3, feats=False, bins=False, div=9)
    check("run: 3 windows, decisions only, restart from decision 0 after the reset",
          st == 0 and [(d["cls"], d["conf"]) for d in res] == [(0, 100), (1, 101), (2, 102)] and res[0]["feat"] is None)

    link.poke(0x0C, (2 << 8) | 77)
    check("poke/peek: ALARM_CFG", link.peek(0x0C) == (2 << 8) | 77)
    st = link.reset()
    check("reset: weights and config re-applied (ALARM_CFG back to the kept value, CLK_DIV 9)",
          st == 0 and link.peek(0x0C) == (3 << 8) | 100 and link.peek(0x1C) == 9 and link.peek(0x20) == int(np.frombuffer(bytes(int(v) & 0xFF for v in w[:4]), "<u4")[0]))

    try:
        link.cmd("Z")
        check("unknown command rejected", False)
    except Exception as e:
        check(f"unknown command rejected ({e})", True)
    link.quit()
    check("quit: chip re-enabled", link.peek(0x00) == 1)
    link.close()
    print("PROTOCOL MOCK TEST:", "PASS" if fails == 0 else f"FAIL ({fails})")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
