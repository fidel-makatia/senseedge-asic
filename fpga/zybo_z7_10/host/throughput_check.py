#!/usr/bin/env python3
"""Window period and classification throughput measured on the board with the Zynq global
timer (firmware command 'T'): after a reset, a reload and the first decision, the elapsed time
over the next n decisions at each divider D, against the prediction 2240 (D+1) clocks per
window (64 samples x 35 clocks per SPI frame) at 20 MHz.  The decision latency inside a
window (728 clocks after the buffer is full, from the RTL audit) is included in the period.

  throughput_check.py --port COM10 --baud 921600 [--divs 0,1,4,9,70,249] [--n 63]
                      [--records records/cwru_d249_g512.npz] [--out results/throughput.json]
"""
import argparse
import json
import struct
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from se_link import F_CLK, add_link_args, open_link  # noqa: E402


def main():
    ap = argparse.ArgumentParser(); add_link_args(ap)
    ap.add_argument("--divs", default="0,1,4,9,70,249"); ap.add_argument("--n", type=int, default=63)
    ap.add_argument("--records", default=str(HERE / "records" / "cwru_d249_g512.npz"))
    ap.add_argument("--out", default=str(HERE / "results" / "throughput.json"))
    a = ap.parse_args()
    d = np.load(a.records)
    codes = d["codes"][:64].astype(np.int64)
    w = np.zeros(212, np.int64)
    for k in range(4):
        w[9 * k] = 127; w[144 + 17 * k] = 127
    link = open_link(a)
    info = link.identify()
    link.load_weights(w)
    link.load_record(codes)
    rows = []
    for div in [int(v) for v in a.divs.split(",")]:
        link.config(div, 100, 3)
        st, p = link.cmd("T", bytes([a.n]), timeout=10 + 3 * a.n * 2240 * (div + 1) / F_CLK)
        ticks = struct.unpack("<Q", p[:8])[0]; cps = struct.unpack("<I", p[8:12])[0]; done = p[12]
        if st or done != a.n:
            sys.exit(f"D={div}: status {st}, {done} of {a.n} decisions")
        period = ticks / cps / a.n
        pred = 2240 * (div + 1) / F_CLK
        rows.append(dict(div=div, fs=F_CLK / (35 * (div + 1)), n=a.n, ticks=ticks, counts_per_second=cps,
                         period_s=period, period_pred_s=pred, period_clocks=period * F_CLK, windows_per_s=1 / period,
                         error_pct=100 * (period / pred - 1)))
        print(f"D = {div:3d}: f_s = {F_CLK / (35 * (div + 1)):9.1f} S/s; window {1e6 * period:9.2f} us measured "
              f"({period * F_CLK:8.1f} clocks) vs {1e6 * pred:9.2f} us predicted ({2240 * (div + 1)} clocks): "
              f"{100 * (period / pred - 1):+.3f} %; {1 / period:8.1f} classifications/s")
    link.quit(); link.close()
    out = dict(board_id=f"0x{info['id']:08X}", f_clk=F_CLK, rows=rows)
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(out, indent=1))
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
