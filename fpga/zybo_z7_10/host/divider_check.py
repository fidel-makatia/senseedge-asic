#!/usr/bin/env python3
"""Divider independence on the board: one block of windows replayed at several sample-rate
dividers must give identical bins, features and decisions (the datapath does not depend on
D; only the replay time does), and the time per window must follow 64 x 35 (D+1) / 20 MHz.

  divider_check.py --records x.npz --weights w.npz --port ... [--divs 249,9,4] [--n 32]
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[2] / "verilog" / "dv" / "audit"))
from evaluate import load_weights  # noqa: E402
from ref_models import chain  # noqa: E402
from se_link import F_CLK, add_link_args, open_link  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    add_link_args(ap)
    ap.add_argument("--records", required=True); ap.add_argument("--weights", required=True)
    ap.add_argument("--divs", default="249,9,4"); ap.add_argument("--n", type=int, default=32)
    ap.add_argument("--out", help="write the measurements as JSON (for paper_figures.py)")
    a = ap.parse_args()
    d = np.load(a.records)
    rng = np.random.RandomState(0)
    idx = np.sort(rng.choice(len(d["codes"]), size=a.n, replace=False))
    codes = d["codes"][idx].astype(np.int64)
    W = load_weights(a.weights)
    exp = [chain(c, W) for c in codes]

    link = open_link(a)
    link.load_weights(W)
    link.load_record(codes)
    ok_all = True
    runs = []
    for div in [int(v) for v in a.divs.split(",")]:
        link.config(div, 100, 3)
        t0 = time.time()
        st, res = link.run(len(codes), feats=True, bins=True, div=div)
        dt = time.time() - t0
        ok = st == 0 and all(r["bins"] == e[0] and r["feat"] == e[1] and (r["cls"], r["conf"]) == (e[2], e[3]) for r, e in zip(res, exp))
        ok_all &= ok
        t_win = 64 * 35 * (div + 1) / F_CLK
        print(f"D={div:3d}: f_s = {F_CLK / (35 * (div + 1)):8.1f} S/s, window {1e3 * t_win:7.2f} ms; {len(res)} windows in {dt:.2f} s "
              f"({1e3 * dt / len(res):.2f} ms each incl. transfer); bins/features/decisions {'identical to the reference' if ok else 'MISMATCH'}")
        runs.append(dict(div=div, fs=F_CLK / (35 * (div + 1)), ms_window_pred=1e3 * t_win, ms_per_window=1e3 * dt / max(1, len(res)),
                         seconds=dt, windows=len(res), ok=bool(ok)))
    link.quit(); link.close()
    print("DIVIDER CHECK:", "PASS" if ok_all else "FAIL")
    if a.out:
        import json
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text(json.dumps(dict(n=len(codes), ok=bool(ok_all), records=str(a.records), weights=str(a.weights),
                                               baud=a.baud, runs=runs), indent=1))


if __name__ == "__main__":
    main()
