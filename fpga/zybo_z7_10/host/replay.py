#!/usr/bin/env python3
"""Replay dataset records through SenseEdge on the Zybo Z7-10 and check every window.

For every sequence of the chosen split, the windows go to the board in blocks of up to 64:
the block is written into the emulator RAM, the chip is reset, the weights reloaded, and the
block run; the board returns class, confidence, alarm and the 8 features of every window (and
the 32 bins of every `--bins-every`-th block).  Each window is compared with the Python
reference chain (bit for bit: bins, features, class, confidence) and the alarm with the alarm
model, so the run is both the accuracy measurement and a bit-exactness test of the hardware.

  replay.py --records records/mafaulda_d249.npz --weights ../../../ml/records/mafaulda_weights.npz
            --port /dev/cu.usbserial-XXXX [--baud 115200] [--split test] [--div 9] [--thr 100]
            [--ncons 3] [--bins-every 20] [--max-seqs N] --out results/mafaulda_fpga
Writes <out>.csv (one row per window), <out>.json (metrics, mismatches, timing) and <out>.md.
--mock runs against the mock firmware (protocol check only; its "chip" is not SenseEdge).
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
from evaluate import FIELDS, load_weights, markdown, metrics, write_csv  # noqa: E402
from ref_models import alarm_model, chain  # noqa: E402
from se_link import add_link_args, open_link  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_link_args(ap)
    ap.add_argument("--records", required=True); ap.add_argument("--weights", required=True)
    ap.add_argument("--split", default="test"); ap.add_argument("--div", type=int, default=9)
    ap.add_argument("--thr", type=int, default=100); ap.add_argument("--ncons", type=int, default=3)
    ap.add_argument("--bins-every", type=int, default=20); ap.add_argument("--max-seqs", type=int)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    d = np.load(a.records)
    meta = json.loads(str(d["meta"]))
    codes, seq, win, label = d["codes"], d["seq"], d["win"], d["label"]
    seq_split, seq_cond, seq_div = d["seq_split"], d["seq_cond"], d["seq_div"]
    params = load_weights(a.weights)
    seqs = [s for s in np.unique(seq) if a.split == "all" or seq_split[s] == a.split]
    if a.max_seqs:
        seqs = seqs[: a.max_seqs]
    tracked = meta.get("policy") == "tracked"

    link = open_link(a)
    info = link.identify()
    print(f"board: id 0x{info['id']:08X}, protocol {info['protocol']}; {len(seqs)} sequences of split '{a.split}', "
          f"{int(np.isin(seq, seqs).sum())} windows, div {'per sequence' if tracked else a.div}")
    st, rb = link.load_weights(params)
    if st != 0 or not (rb.astype(np.int64) == params).all():
        sys.exit(f"weight load: {st} read-back errors")
    link.config(a.div, a.thr, a.ncons)

    rows = []
    n_bins_checked = n_mis_bins = n_mis_feat = n_mis_cls = n_mis_alarm = 0
    blocks = 0
    t0 = time.time()
    for si, s in enumerate(seqs):
        idx = np.flatnonzero(seq == s)
        div = int(seq_div[s]) if tracked else a.div
        if tracked:
            link.config(div, a.thr, a.ncons)
        for b0 in range(0, len(idx), 64):
            blk = idx[b0:b0 + 64]
            want_bins = (blocks % a.bins_every) == 0
            link.load_record(codes[blk])
            st, res = link.run(len(blk), feats=True, bins=want_bins, div=div)
            if st != 0 or len(res) != len(blk):
                sys.exit(f"run failed on sequence {s} block {b0 // 64}: status {st}, {len(res)} of {len(blk)} windows")
            dec = []
            for k, i in enumerate(blk):
                m, f, c, cf = chain(codes[i].astype(np.int64), params)
                r = res[k]
                exact = 1
                if r["feat"] != f: n_mis_feat += 1; exact = 0
                if (r["cls"], r["conf"]) != (c, cf): n_mis_cls += 1; exact = 0
                if want_bins:
                    n_bins_checked += 1
                    if r["bins"] != m: n_mis_bins += 1; exact = 0
                dec.append((c, cf))
                rows.append([int(s), int(win[i]), int(label[i]), str(seq_cond[s]), str(seq_split[s]), int(r["cls"]), int(r["conf"]), int(r["alarm"]), exact])
            exp_alarm = alarm_model(dec, a.thr, a.ncons)
            for k in range(len(blk)):
                if res[k]["alarm"] != exp_alarm[k]:
                    n_mis_alarm += 1; rows[-len(blk) + k][-1] = 0
            blocks += 1
        if (si + 1) % 10 == 0 or si == len(seqs) - 1:
            el = time.time() - t0
            print(f"  {si + 1}/{len(seqs)} sequences, {len(rows)} windows, {len(rows) / el:.0f} windows/s, "
                  f"mismatches feat {n_mis_feat} cls {n_mis_cls} bins {n_mis_bins}/{n_bins_checked} alarm {n_mis_alarm}", flush=True)
    elapsed = time.time() - t0
    link.quit(); link.close()

    out = Path(a.out); out.parent.mkdir(parents=True, exist_ok=True)
    write_csv(out.with_suffix(".csv"), rows)
    r = {k: np.array([row[i] for row in rows]) for i, k in enumerate(FIELDS)}
    m = metrics(r["label"], r["cls"], r["seq"], r["cond"], class_names=meta["class_names"])
    m.update(dict(board_id=f"0x{info['id']:08X}", div=a.div if not tracked else "tracked", threshold=a.thr, n_consec=a.ncons,
                  blocks=blocks, bins_checked=n_bins_checked, mismatch_bins=n_mis_bins, mismatch_features=n_mis_feat,
                  mismatch_class_conf=n_mis_cls, mismatch_alarm=n_mis_alarm, elapsed_s=round(elapsed, 1),
                  windows_per_s=round(len(rows) / elapsed, 1), records=str(a.records), weights=str(a.weights),
                  baud=a.baud, mock=bool(a.mock)))
    out.with_suffix(".json").write_text(json.dumps(m, indent=1))
    md = markdown(m)
    out.with_suffix(".md").write_text(md)
    print(md)
    print(f"bit-exact: features {len(rows) - n_mis_feat}/{len(rows)}, class+confidence {len(rows) - n_mis_cls}/{len(rows)}, "
          f"bins {n_bins_checked - n_mis_bins}/{n_bins_checked}, alarm {len(rows) - n_mis_alarm}/{len(rows)}; {elapsed:.0f} s")


if __name__ == "__main__":
    main()
