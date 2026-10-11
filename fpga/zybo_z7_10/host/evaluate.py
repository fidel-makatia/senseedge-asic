#!/usr/bin/env python3
"""Metrics for SenseEdge dataset results, on the board (replay.py) or on the host model.

Rows (CSV): seq, win, label, cond, split, cls, conf, alarm, exact
  window accuracy, balanced accuracy, per-class recall and precision, 4x4 confusion matrix,
  sequence accuracy (majority vote over the windows of a sequence), bootstrap 95 % intervals
  with the sequence as the resampling unit, per-condition accuracy, mismatch count.

  evaluate.py --csv results.csv --out results.json [--md results.md]
  evaluate.py --host --records x.npz --weights w.npz [--split test] --csv-out host.csv --out host.json
The --host form computes the same rows with the Python reference chain, which is what the
board must reproduce bit for bit; it gives the expected numbers before a board is touched.
"""
import argparse
import csv
import json
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[2] / "verilog" / "dv" / "audit"))
from ref_models import chain  # noqa: E402

FIELDS = ["seq", "win", "label", "cond", "split", "cls", "conf", "alarm", "exact"]


def load_weights(path):
    path = Path(path)
    if path.suffix == ".npz":
        return np.load(path)["all_weights"].astype(np.int64)
    if path.suffix == ".hex":
        return np.array([int(l, 16) for l in path.read_text().split()]).astype(np.int8).astype(np.int64)
    if path.suffix == ".h":
        body = path.read_text().split("{", 1)[1].split("}", 1)[0]
        return np.array([int(v) for v in body.replace("\n", " ").split(",") if v.strip()], np.int64)
    raise ValueError(path)


def metrics(label, cls, seq, cond=None, n_boot=1000, seed=0, class_names=None):
    label = np.asarray(label, np.int64); cls = np.asarray(cls, np.int64); seq = np.asarray(seq)
    n = len(label)
    if n == 0:
        raise ValueError("metrics of an empty set")
    conf = np.zeros((4, 4), np.int64)
    for t, p in zip(label, cls):
        conf[t, p] += 1
    recall = [float(conf[c, c] / conf[c].sum()) if conf[c].sum() else float("nan") for c in range(4)]
    precision = [float(conf[c, c] / conf[:, c].sum()) if conf[:, c].sum() else float("nan") for c in range(4)]
    win_acc = float((label == cls).mean())
    bal_acc = float(np.nanmean(recall))
    # sequence majority vote
    seqs = np.unique(seq)
    seq_true, seq_pred = [], []
    for s in seqs:
        m = seq == s
        seq_true.append(int(label[m][0]))
        seq_pred.append(int(np.bincount(cls[m], minlength=4).argmax()))
    seq_true = np.array(seq_true); seq_pred = np.array(seq_pred)
    seq_acc = float((seq_true == seq_pred).mean())
    # bootstrap over sequences
    rng = np.random.RandomState(seed)
    per_seq_correct = np.array([float((label[seq == s] == cls[seq == s]).mean()) for s in seqs])
    per_seq_n = np.array([int((seq == s).sum()) for s in seqs])
    wa, sa = [], []
    for _ in range(n_boot):
        pick = rng.randint(0, len(seqs), len(seqs))
        wa.append(float((per_seq_correct[pick] * per_seq_n[pick]).sum() / per_seq_n[pick].sum()))
        sa.append(float((seq_true[pick] == seq_pred[pick]).mean()))
    ci = lambda v: [float(np.percentile(v, 2.5)), float(np.percentile(v, 97.5))] if len(v) else [float("nan")] * 2
    out = dict(windows=int(n), sequences=int(len(seqs)), window_accuracy=win_acc,
               window_accuracy_ci95=ci(wa), balanced_accuracy=bal_acc, sequence_accuracy=seq_acc,
               sequence_accuracy_ci95=ci(sa),
               recall=recall, precision=precision, confusion=conf.tolist(), class_names=class_names)
    if cond is not None:
        cond = np.asarray(cond)
        out["per_condition"] = {str(c): dict(windows=int((cond == c).sum()), accuracy=float((label[cond == c] == cls[cond == c]).mean()))
                                for c in sorted(set(cond.tolist()))}
    return out


def read_csv(path):
    rows = {k: [] for k in FIELDS}
    with open(path) as f:
        for r in csv.DictReader(f):
            for k in FIELDS:
                rows[k].append(r[k])
    for k in ("seq", "win", "label", "cls", "conf", "alarm", "exact"):
        rows[k] = np.array([int(v) for v in rows[k]])
    return rows


def write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f); w.writerow(FIELDS)
        for r in rows:
            w.writerow(r)


def _host_window(args):
    codes, params = args
    _, _, c, cf = chain(codes, params)
    return c, cf


def host_rows(records, params, split="test", jobs=8, max_seqs=None):
    d = np.load(records)
    codes, seq, win, label = d["codes"], d["seq"], d["win"], d["label"]
    sel = np.flatnonzero(d["seq_split"][seq] == split) if split != "all" else np.arange(len(codes))
    if max_seqs:
        keep = np.unique(seq[sel])[:max_seqs]
        sel = sel[np.isin(seq[sel], keep)]
    with Pool(jobs) as pool:
        dec = pool.map(_host_window, [(codes[i].astype(np.int64), params) for i in sel], chunksize=256)
    rows = []
    for i, (c, cf) in zip(sel, dec):
        rows.append([int(seq[i]), int(win[i]), int(label[i]), str(d["seq_cond"][seq[i]]), str(d["seq_split"][seq[i]]), c, cf, 0, 1])
    return rows, json.loads(str(d["meta"]))


def markdown(m):
    names = m.get("class_names") or ["0", "1", "2", "3"]
    L = [f"windows {m['windows']}, sequences {m['sequences']}",
         f"window accuracy {100 * m['window_accuracy']:.2f} % (95 % CI {100 * m['window_accuracy_ci95'][0]:.2f} to {100 * m['window_accuracy_ci95'][1]:.2f}), "
         f"balanced {100 * m['balanced_accuracy']:.2f} %, sequence majority {100 * m['sequence_accuracy']:.2f} % "
         f"(CI {100 * m['sequence_accuracy_ci95'][0]:.1f} to {100 * m['sequence_accuracy_ci95'][1]:.1f})", "",
         "| true \\ pred | " + " | ".join(names) + " | recall |", "|---|" + "---|" * 5]
    for c in range(4):
        L.append(f"| {names[c]} | " + " | ".join(str(v) for v in m["confusion"][c]) + f" | {100 * m['recall'][c]:.1f} % |")
    if "per_condition" in m:
        L += ["", "| condition | windows | accuracy |", "|---|---|---|"]
        for k, v in m["per_condition"].items():
            L.append(f"| {k} | {v['windows']} | {100 * v['accuracy']:.1f} % |")
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv"); ap.add_argument("--host", action="store_true")
    ap.add_argument("--records"); ap.add_argument("--weights"); ap.add_argument("--split", default="test")
    ap.add_argument("--csv-out"); ap.add_argument("--out", required=True); ap.add_argument("--md")
    ap.add_argument("--jobs", type=int, default=8); ap.add_argument("--max-seqs", type=int)
    a = ap.parse_args()
    names = None
    if a.host:
        rows, meta = host_rows(a.records, load_weights(a.weights), a.split, a.jobs, a.max_seqs)
        names = meta["class_names"]
        if a.csv_out:
            write_csv(a.csv_out, rows)
        r = {k: np.array([row[i] for row in rows]) for i, k in enumerate(FIELDS)}
    else:
        r = read_csv(a.csv)
        if a.records:
            names = json.loads(str(np.load(a.records)["meta"]))["class_names"]
    m = metrics(r["label"], r["cls"], r["seq"], r["cond"], class_names=names)
    m["mismatches"] = int((np.asarray(r["exact"]).astype(int) == 0).sum())
    Path(a.out).write_text(json.dumps(m, indent=1))
    md = markdown(m)
    if a.md:
        Path(a.md).write_text(md)
    print(md)


if __name__ == "__main__":
    main()
