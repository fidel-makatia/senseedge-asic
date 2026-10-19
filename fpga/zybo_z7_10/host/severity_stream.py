#!/usr/bin/env python3
"""Severity ladder on the board: the chip watches a machine whose fault grows step by step.

A healthy test sequence is followed by one test sequence of each condition in --ladder (in
that order), --per windows of each, replayed at the deployment divider.  Per window the chip's
8 features, decision and confidence are recorded (bins too), and checked bit for bit against
the reference chain.  The figure (paper_figures.py: fig_<name>_severity) shows the fault
energy the chip computes and its decision/confidence climbing with severity: the monitoring
signal a maintenance system would trend.

  severity_stream.py --records records/mafaulda_d249.npz --weights ../../../ml/records/mafaulda_d249_weights.npz
                     --port COM10 --baud 921600 --healthy normal
                     --ladder imbalance/6g,imbalance/10g,imbalance/15g,imbalance/20g,imbalance/25g,imbalance/30g,imbalance/35g
                     [--per 32] [--div 249] [--out results/severity_mafaulda_imbalance]
"""
import argparse
import json
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
from stream_demo import run_blocks, seq_of_cond  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    add_link_args(ap)
    ap.add_argument("--records", required=True); ap.add_argument("--weights", required=True)
    ap.add_argument("--healthy", required=True); ap.add_argument("--ladder", required=True)
    ap.add_argument("--per", type=int, default=32); ap.add_argument("--div", type=int, default=249)
    ap.add_argument("--speed", type=float, default=None, help="pick, per condition, the test sequence closest to this shaft speed (Hz)")
    ap.add_argument("--thr", type=int, default=100); ap.add_argument("--ncons", type=int, default=3)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    d = np.load(a.records); meta = json.loads(str(d["meta"]))
    params = load_weights(a.weights)
    codes, seq = d["codes"], d["seq"]
    steps = [a.healthy] + [c for c in a.ladder.split(",") if c]
    tracked = meta.get("policy") == "tracked"
    segs = []
    for cond in steps:
        cands = [int(s) for s in np.unique(seq) if d["seq_cond"][s] == cond and d["seq_split"][s] == "test"]
        if not cands:
            sys.exit(f"no test sequence with condition {cond}")
        s = min(cands, key=lambda x: abs(float(d["seq_speed_hz"][x]) - a.speed)) if a.speed else cands[0]
        w = codes[seq == s].astype(np.int64)
        pick = w[len(w) // 2 - a.per // 2: len(w) // 2 - a.per // 2 + a.per]      # the middle of the sequence
        segs.append(dict(cond=cond, seq=int(s), speed_hz=float(d["seq_speed_hz"][s]), windows=len(pick), codes=pick,
                         div=int(d["seq_div"][s]) if tracked else a.div))
    fs = F_CLK / (35 * (a.div + 1)); t_win = 64 / fs

    link = open_link(a)
    info = link.identify()
    st, rb = link.load_weights(params)
    if st or not (rb.astype(np.int64) == params).all():
        sys.exit("weight load failed")
    chk = dict(n=0, mismatch=0)
    t0 = time.time()
    res = []
    for sg in segs:   # one block per segment, at the segment's own divider when the records are tracked
        res += run_blocks(link, sg["codes"], params, sg["div"], a.thr, a.ncons, bins=True, check=chk)
    link.quit(); link.close()
    el = time.time() - t0
    stream = np.concatenate([sg["codes"] for sg in segs])

    pos = 0
    for s in segs:
        r = res[pos:pos + s["windows"]]; pos += s["windows"]
        cls = [w["cls"] for w in r]; conf = [w["conf"] for w in r]; b0 = [w["feat"][0] for w in r]
        s["decisions"] = cls; s["mean_conf"] = float(np.mean(conf)); s["mean_band0"] = float(np.mean(b0))
        s["mean_feat"] = [float(np.mean([w["feat"][k] for w in r])) for k in range(8)]
        s["fault_fraction"] = float(np.mean([c != 0 for c in cls]))
        s.pop("codes")
        s["fs"] = F_CLK / (35 * (s["div"] + 1))
        print(f"{s['cond']:28s} seq {s['seq']:4d} {s['speed_hz']:5.1f} Hz D={s['div']:4d}: fault decisions {100 * s['fault_fraction']:5.1f} %, "
              f"mean confidence {s['mean_conf']:5.1f}, band 1-4 {s['mean_band0']:5.1f}, decisions {cls}")
    out = dict(board_id=f"0x{info['id']:08X}", records=str(a.records), weights=str(a.weights), class_names=meta["class_names"],
               div=a.div, fs=fs, t_window_s=t_win, tracked=tracked, speed_target=a.speed, threshold=a.thr, n_consec=a.ncons, per=a.per, segments=segs,
               windows=[dict(cls=w["cls"], conf=w["conf"], feat=w["feat"], bins=w["bins"]) for w in res],
               codes_rms=[float(np.sqrt(np.mean((w - 2048.0) ** 2))) for w in stream],
               checked=chk["n"], mismatches=chk["mismatch"], elapsed_s=el)
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).with_suffix(".json").write_text(json.dumps(out))
    print(f"{len(res)} windows ({len(res) * t_win:.1f} s of machine time) in {el:.1f} s; bit-exact {chk['n'] - chk['mismatch']}/{chk['n']}; wrote {Path(a.out).with_suffix('.json')}")


if __name__ == "__main__":
    main()
