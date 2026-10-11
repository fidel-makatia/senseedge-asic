#!/usr/bin/env python3
"""The self-test of sw/senseedge_zybo.c, driven over the host protocol so its numbers are recorded:
the bin-8 record is played through the emulator under the diagonal weight image, and the 32
bins, 8 features, class and confidence read back from the board are compared with the Python
reference chain.  Writes results/selftest_spectrum.json for paper_figures.py.

  selftest_capture.py --port COM10 [--out results/selftest_spectrum.json]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[2] / "verilog" / "dv" / "audit"))
from ref_models import chain  # noqa: E402
from se_link import add_link_args, open_link  # noqa: E402

REC8 = [2048, 2402, 2548, 2402, 2048, 1694, 1548, 1694] * 8


def diagonal_weights():
    w = np.zeros(212, np.int64)
    for k in range(4):
        w[9 * k] = 127          # hidden k <- input k
        w[144 + 17 * k] = 127   # class k <- hidden k
    return w


def main():
    ap = argparse.ArgumentParser(); add_link_args(ap)
    ap.add_argument("--out", default=str(HERE / "results" / "selftest_spectrum.json"))
    a = ap.parse_args()
    W = diagonal_weights()
    codes = np.array([REC8], np.int64)
    m, f, c, cf = chain(codes[0], W)

    link = open_link(a)
    info = link.identify()
    st, rb = link.load_weights(W)
    link.config(4, 100, 3)
    link.load_record(codes)
    st2, res = link.run(1, feats=True, bins=True, div=4)
    link.quit(); link.close()
    r = res[0]
    ok = st == 0 and (rb.astype(np.int64) == W).all() and st2 == 0 and r["bins"] == m and r["feat"] == f and (r["cls"], r["conf"]) == (c, cf)
    ints = lambda v: [int(x) for x in v]
    out = dict(board_id=f"0x{info['id']:08X}", readback_errors=int(st), board=ints(r["bins"]), reference=ints(m), features=ints(r["feat"]),
               features_ref=ints(f), cls=int(r["cls"]), conf=int(r["conf"]), cls_ref=int(c), conf_ref=int(cf), ok=bool(ok))
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(out, indent=1))
    print(f"bins board {r['bins']}\nbins ref   {m}\nfeatures {r['feat']} (ref {f}); class {r['cls']} conf {r['conf']} (ref {c}, {cf})")
    print("SELF-TEST SPECTRUM:", "PASS" if ok else "FAIL")


if __name__ == "__main__":
    main()
