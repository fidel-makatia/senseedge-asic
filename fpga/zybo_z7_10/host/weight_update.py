#!/usr/bin/env python3
"""Register-map model update on the board (test run 4): the same windows are classified under
two weight sets loaded over the bus alone, and every decision and the alarm are checked
against the reference chain.

Set A is the trained set; set B is A with the output layer rotated by one class (class k of B
reads the accumulator of class k+1 of A), so every decision moves to a known other class and,
for a fault sequence, the alarm that asserts under A does not assert under B.

  weight_update.py --records x.npz --weights w.npz --port ... [--class 1] [--n 16] [--div 9]
"""
import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[2] / "verilog" / "dv" / "audit"))
from evaluate import load_weights  # noqa: E402
from ref_models import alarm_model, chain, unpack  # noqa: E402
from se_link import add_link_args, open_link  # noqa: E402


def rotate_outputs(params):
    W1, b1, W2, b2 = unpack(params)
    W2r = np.roll(W2, -1, axis=0); b2r = np.roll(b2, -1)
    return np.concatenate([W1.flatten(), b1, W2r.flatten(), b2r]).astype(np.int64)


def main():
    ap = argparse.ArgumentParser()
    add_link_args(ap)
    ap.add_argument("--records", required=True); ap.add_argument("--weights", required=True)
    ap.add_argument("--class", dest="cls", type=int, default=1); ap.add_argument("--n", type=int, default=16)
    ap.add_argument("--div", type=int, default=9); ap.add_argument("--thr", type=int, default=100); ap.add_argument("--ncons", type=int, default=3)
    ap.add_argument("--out", help="write the decisions and alarm flags as JSON (for paper_figures.py)")
    a = ap.parse_args()
    d = np.load(a.records)
    seq, label, split = d["seq"], d["label"], d["seq_split"]
    cand = [s for s in np.unique(seq) if split[s] == "test" and label[seq == s][0] == a.cls]
    if not cand:
        sys.exit("no test sequence of that class")
    idx = np.flatnonzero(seq == cand[0])[: a.n]
    codes = d["codes"][idx].astype(np.int64)
    A = load_weights(a.weights); B = rotate_outputs(A)

    link = open_link(a)
    link.config(a.div, a.thr, a.ncons)
    link.load_record(codes)
    report = {}
    for name, W in (("A", A), ("B", B), ("A again", A)):
        st, rb = link.load_weights(W)
        st2, res = link.run(len(codes), feats=False, bins=False, div=a.div)
        exp = [chain(c, W)[2:] for c in codes]
        exp_alarm = alarm_model(exp, a.thr, a.ncons)
        got = [(r["cls"], r["conf"]) for r in res]
        ok = st == 0 and (rb.astype(np.int64) == W).all() and st2 == 0 and got == exp and [r["alarm"] for r in res] == exp_alarm
        report[name] = dict(readback_errors=st, decisions=[g[0] for g in got], alarm=[r["alarm"] for r in res], ok=ok)
        print(f"set {name}: read-back errors {st}; classes {[g[0] for g in got]}; alarm {[r['alarm'] for r in res]}; "
              f"{'matches the reference' if ok else 'MISMATCH'}")
    link.quit(); link.close()
    changed = sum(1 for x, y in zip(report["A"]["decisions"], report["B"]["decisions"]) if x != y)
    print(f"decisions changed by the register-map write alone: {changed} of {len(codes)}; "
          f"alarm under A {max(report['A']['alarm'])}, under B {max(report['B']['alarm'])}; "
          f"RESULT: {'PASS' if all(v['ok'] for v in report.values()) else 'FAIL'}")
    if a.out:
        import json
        report.update(dict(changed=changed, n=len(codes), sequence=int(cand[0]), label=int(a.cls), div=a.div, threshold=a.thr,
                           n_consec=a.ncons, ok=all(report[k]["ok"] for k in ("A", "B", "A again"))))
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()
