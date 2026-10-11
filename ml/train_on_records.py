#!/usr/bin/env python3
"""Train the 8-16-4 INT8 network on replay records of a public dataset, on the integer grid of
the corrected nn_engine.v, with the features computed by the bit-exact reference chain
(verilog/dv/audit/ref_models.py: capture -> fft64 -> features).

Protocol
  - features are computed for every window once and cached next to the records,
  - the records' 'train' sequences are split again by sequence into fit (80 %) and val (20 %),
    stratified by class and condition; the 'test' split is never touched until the end,
  - class-weighted cross entropy (the datasets are imbalanced; HEALTHY is the smallest class),
  - several seeds; the seed with the best balanced val accuracy is kept,
  - the kept weights are checked bit-exact against nn_engine.v in Icarus on 400 test vectors,
  - outputs: <out>/<name>_weights.{npz,hex,h} (the .h is also copied to fpga/zybo_z7_10/sw/)
    and <out>/<name>_train.{json,md} with fit/val/test metrics from host/evaluate.py.

  train_on_records.py --records fpga/zybo_z7_10/host/records/mafaulda_d249.npz --name mafaulda
                      [--seeds 8] [--steps 6000] [--jobs 8] [--no-rtl] [--out ml/records]
"""
import argparse
import json
import shutil
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
sys.path.insert(0, str(REPO / "verilog" / "dv" / "audit"))
sys.path.insert(0, str(REPO / "fpga" / "zybo_z7_10" / "host"))
sys.path.insert(0, str(HERE))
from ref_models import chain, wrap  # noqa: E402
from evaluate import markdown, metrics  # noqa: E402
from train_corrected_exact import model_int, pack, project, run_rtl  # noqa: E402


def _feat(codes):
    return chain(codes)[1]


def features_cached(records, jobs):
    cache = Path(str(records).replace(".npz", ".feats.npz"))
    d = np.load(records)
    if cache.exists():
        f = np.load(cache)
        if len(f["X"]) == len(d["codes"]):
            return f["X"].astype(np.int64)
    t0 = time.time()
    with Pool(jobs) as pool:
        X = pool.map(_feat, [c.astype(np.int64) for c in d["codes"]], chunksize=512)
    X = np.array(X, np.uint8)
    np.savez_compressed(cache, X=X)
    print(f"features of {len(X)} windows computed in {time.time() - t0:.0f} s -> {cache.name}")
    return X.astype(np.int64)


def train(X, y, sw, seed, steps=6000, c1=4000.0, c2=60.0, headroom=30000.0):
    """train_corrected_exact.train with per-sample weights sw (sum 1)."""
    rng = np.random.RandomState(seed)
    Xe = X.astype(np.float64)
    W1 = rng.randn(16, 8) * 12; b1 = np.zeros(16)
    W2 = rng.randn(4, 16) * 12; b2 = np.zeros(4)
    project(W1, b1, W2, b2)
    params = [W1, b1, W2, b2]
    m = [np.zeros_like(p) for p in params]; v = [np.zeros_like(p) for p in params]
    lr = [0.6, 2.0, 0.6, 2.0]
    onehot = np.eye(4)[y]
    for t in range(1, steps + 1):
        acc1 = Xe @ W1.T + b1
        h = np.maximum(0, acc1) / c1
        z = (h @ W2.T) / c2 * 8 + b2 / (c1 * c2) * 8
        z -= z.max(1, keepdims=True); p = np.exp(z); p /= p.sum(1, keepdims=True)
        dz = (p - onehot) * sw[:, None]
        gW2 = dz.T @ h / c2 * 8; gb2 = dz.sum(0) / (c1 * c2) * 8
        dh = dz @ W2 / c2 * 8
        over = np.clip(np.abs(acc1) - headroom, 0, None) * np.sign(acc1) * 1e-7
        dacc = dh * (acc1 > 0) / c1 + over * sw[:, None]
        gW1 = dacc.T @ Xe; gb1 = dacc.sum(0)
        for i, (p_, g) in enumerate(zip(params, [gW1, gb1, gW2, gb2])):
            m[i] = 0.9 * m[i] + 0.1 * g; v[i] = 0.999 * v[i] + 0.001 * g * g
            p_ -= lr[i] * (m[i] / (1 - 0.9 ** t)) / (np.sqrt(v[i] / (1 - 0.999 ** t)) + 1e-12)
        project(*params)
    return [np.round(p_).astype(np.int64) for p_ in params]


def c_header(name, params):
    vals = ", ".join(str(int(v)) for v in params)
    return (f"/* SenseEdge 8-16-4 INT8 weights trained on {name} replay records (ml/train_on_records.py);\n"
            f"   layout: W1[16][8], b1[16], W2[4][16], b2[4]; load as 53 little-endian words at 0x20. */\n"
            f"#include <stdint.h>\nstatic const int8_t weights_{name}[212] = {{ {vals} }};\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--records", required=True, type=Path); ap.add_argument("--name", required=True)
    ap.add_argument("--seeds", type=int, default=8); ap.add_argument("--steps", type=int, default=6000)
    ap.add_argument("--jobs", type=int, default=8); ap.add_argument("--no-rtl", action="store_true")
    ap.add_argument("--out", type=Path, default=HERE / "records"); ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    d = np.load(a.records)
    meta = json.loads(str(d["meta"])); names = meta["class_names"]
    X = features_cached(a.records, a.jobs)
    y = d["label"].astype(np.int64); seq = d["seq"]; seq_split = d["seq_split"]; seq_cond = d["seq_cond"]
    train_seqs = [s for s in np.unique(seq) if seq_split[s] == "train"]
    rng = np.random.RandomState(a.seed)
    groups = {}                      # inner fit/val split: by sequence, stratified by class
    for s in train_seqs:
        groups.setdefault(int(y[seq == s][0]), []).append(s)
    val = set()
    for g in groups.values():
        g = sorted(g); rng.shuffle(g)
        if len(g) >= 2:
            val.update(g[: max(1, int(round(0.2 * len(g))))])
    fit_m = np.isin(seq, [s for s in train_seqs if s not in val]); val_m = np.isin(seq, list(val)); test_m = seq_split[seq] == "test"
    Xf, yf = X[fit_m], y[fit_m]
    cw = len(yf) / (4.0 * np.maximum(1, np.bincount(yf, minlength=4)))
    sw = cw[yf]; sw = sw / sw.sum()
    print(f"{a.name}: fit {fit_m.sum()} windows ({len(train_seqs) - len(val)} seq), val {val_m.sum()} ({len(val)} seq), "
          f"test {test_m.sum()} ({int((seq_split == 'test').sum())} seq); class weights {np.round(cw, 2).tolist()}")

    best = None
    for s in range(a.seeds):
        t0 = time.time()
        q = train(Xf, yf, sw, s, a.steps)
        mv = metrics(y[val_m], model_int(X[val_m], *q)[2], seq[val_m], n_boot=0)
        print(f"  seed {s}: balanced val {100 * mv['balanced_accuracy']:.1f} %, window val {100 * mv['window_accuracy']:.1f} % ({time.time() - t0:.0f} s)")
        if best is None or mv["balanced_accuracy"] > best[0]:
            best = (mv["balanced_accuracy"], s, q)
    _, s, q = best
    params = pack(*q)
    rep = {}
    for part, msk in (("fit", fit_m), ("val", val_m), ("test", test_m)):
        rep[part] = metrics(y[msk], model_int(X[msk], *q)[2], seq[msk], seq_cond[seq[msk]], class_names=names)
    if not a.no_rtl:
        ti = np.flatnonzero(test_m); pick = np.random.RandomState(1).choice(ti, size=min(400, len(ti)), replace=False)
        r = run_rtl(params, X[pick], REPO / "verilog" / "dv" / "audit" / "build" / "records")
        hid, logit, cls = model_int(X[pick], *q)
        exact = int(((hid == r["hid"]).all(1) & (logit == r["logit"]).all(1) & (cls == r["cls"])).sum())
        rep["rtl_bit_exact"] = f"{exact}/{len(pick)}"
        assert exact == len(pick), "RTL does not match the integer model"
        print(f"RTL: {exact}/{len(pick)} test vectors bit-exact in nn_engine.v")
    a.out.mkdir(parents=True, exist_ok=True)
    W1, b1, W2, b2 = q
    np.savez(a.out / f"{a.name}_weights.npz", layer1_weights=W1.astype(np.int8), layer1_biases=b1.astype(np.int8),
             layer2_weights=W2.astype(np.int8), layer2_biases=b2.astype(np.int8), all_weights=params.astype(np.int8),
             input_domain=np.array(["unsigned_u8_corrected_rtl"]), dataset=np.array([a.name]))
    (a.out / f"{a.name}_weights.hex").write_text("".join(f"{int(v) & 0xFF:02x}\n" for v in params))
    h = a.out / f"{a.name}_weights.h"; h.write_text(c_header(a.name, params))
    shutil.copy(h, REPO / "fpga" / "zybo_z7_10" / "sw" / h.name)
    rep.update(dict(name=a.name, records=str(a.records), seed=s, steps=a.steps, meta=meta))
    (a.out / f"{a.name}_train.json").write_text(json.dumps(rep, indent=1))
    md = f"# {a.name}: 8-16-4 INT8 on the corrected arithmetic, seed {s}\n\n## test split (host model, what the board must reproduce)\n" + markdown(rep["test"]) + \
         "\n## val split\n" + markdown(rep["val"]) + "\n## fit split\n" + markdown(rep["fit"])
    (a.out / f"{a.name}_train.md").write_text(md)
    t = rep["test"]
    print(f"TEST: window accuracy {100 * t['window_accuracy']:.2f} % (CI {100 * t['window_accuracy_ci95'][0]:.1f}-{100 * t['window_accuracy_ci95'][1]:.1f}), "
          f"balanced {100 * t['balanced_accuracy']:.2f} %, sequence majority {100 * t['sequence_accuracy']:.2f} %; recall {np.round(100 * np.array(t['recall']), 1).tolist()}")
    print(f"saved {a.out / (a.name + '_weights.npz')}, .hex, .h (copied to fpga/zybo_z7_10/sw/), {a.name}_train.json/.md")


if __name__ == "__main__":
    main()
