#!/usr/bin/env python3
"""Train the 8-16-4 INT8 network directly on the integer grid of the corrected nn_engine.v
(branch fix/audit-defects) and prove the result bit-exact against the RTL.

Model: verilog/dv/audit/ref_models.py::nn_infer (unsigned inputs, signed weights, every
product signed, 16-bit wrap then ReLU, 24-bit outputs, first index of the maximum).
Data:  the synthetic feature generator of train_senseedge_fixed.py (same split as the
       pre-silicon audit).  Replace gen_data with recorded chip features for a machine.

Writes senseedge_weights.npz (same keys as the other trainers) and senseedge_weights.hex,
then run export_weights.py to regenerate firmware/nn_weights.h.

Usage:  python3 train_corrected_exact.py [--seeds 8] [--no-rtl]
"""
import argparse
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
AUDIT = HERE.parent / "verilog" / "dv" / "audit"
RTL = HERE.parent / "verilog" / "rtl"
sys.path.insert(0, str(AUDIT))
from ref_models import nn_infer, wrap  # noqa: E402

import train_senseedge_fixed as T  # noqa: E402


def model_int(X, W1, b1, W2, b2):
    """Vectorised nn_infer: returns (hidden, logits, class)."""
    hid = np.maximum(wrap(X @ W1.T + b1, 16), 0)
    out = wrap(hid @ W2.T + b2, 24)
    return hid, out, out.argmax(1)


def project(W1, b1, W2, b2):
    np.clip(W1, -127, 127, out=W1); np.clip(b1, -128, 127, out=b1)
    np.clip(W2, -127, 127, out=W2); np.clip(b2, -128, 127, out=b2)


def train(X, y, seed, steps=6000, c1=4000.0, c2=60.0, headroom=30000.0):
    """Adam on a float copy of the integer parameters, projected to the grid every step.
    The hidden pre-activations are penalised beyond `headroom` so that the 16-bit wrap of
    the hardware is never reached on the training data."""
    rng = np.random.RandomState(seed)
    Xe = X.astype(np.float64)
    W1 = rng.randn(16, 8) * 12; b1 = np.zeros(16)
    W2 = rng.randn(4, 16) * 12; b2 = np.zeros(4)
    project(W1, b1, W2, b2)
    params = [W1, b1, W2, b2]
    m = [np.zeros_like(p) for p in params]; v = [np.zeros_like(p) for p in params]
    lr = [0.6, 2.0, 0.6, 2.0]
    n = len(y); onehot = np.eye(4)[y]
    for t in range(1, steps + 1):
        acc1 = Xe @ W1.T + b1
        h = np.maximum(0, acc1) / c1
        z = (h @ W2.T) / c2 * 8 + b2 / (c1 * c2) * 8
        z -= z.max(1, keepdims=True); p = np.exp(z); p /= p.sum(1, keepdims=True)
        dz = (p - onehot) / n
        gW2 = dz.T @ h / c2 * 8; gb2 = dz.sum(0) / (c1 * c2) * 8
        dh = dz @ W2 / c2 * 8
        over = np.clip(np.abs(acc1) - headroom, 0, None) * np.sign(acc1) * 1e-7
        dacc = dh * (acc1 > 0) / c1 + over / n
        gW1 = dacc.T @ Xe; gb1 = dacc.sum(0)
        for i, (p_, g) in enumerate(zip(params, [gW1, gb1, gW2, gb2])):
            m[i] = 0.9 * m[i] + 0.1 * g; v[i] = 0.999 * v[i] + 0.001 * g * g
            p_ -= lr[i] * (m[i] / (1 - 0.9 ** t)) / (np.sqrt(v[i] / (1 - 0.999 ** t)) + 1e-12)
        project(*params)
    return [np.round(p_).astype(np.int64) for p_ in params]


def pack(W1, b1, W2, b2):
    return np.concatenate([W1.flatten(), b1, W2.flatten(), b2]).astype(np.int64)


def run_rtl(params, X, build):
    """nn_engine.v in Icarus Verilog through the audit testbench."""
    build.mkdir(parents=True, exist_ok=True)
    vvp = build / "tb_nn_vectors.vvp"
    subprocess.run(["iverilog", "-g2012", "-o", str(vvp), str(AUDIT / "tb_nn_vectors.v"), str(RTL / "nn_engine.v")],
                   check=True)
    (build / "weights.hex").write_text("".join(f"{int(v) & 0xFF:02x}\n" for v in params))
    (build / "vectors.hex").write_text("".join(f"{int(v) & 0xFF:02x}\n" for v in np.asarray(X).flatten()))
    subprocess.run(["vvp", str(vvp), f"+nvec={len(X)}"], check=True, capture_output=True, cwd=build)
    out = np.loadtxt(build / "nn_out.txt", dtype=np.int64).reshape(len(X), -1)
    return dict(cls=out[:, 0], conf=out[:, 1], hid=out[:, 2:18], logit=out[:, 18:22])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=8)
    ap.add_argument("--samples", type=int, default=500)
    ap.add_argument("--no-rtl", action="store_true")
    a = ap.parse_args()

    Xu, y = T.gen_data(a.samples, 42)
    X = np.clip(np.round(Xu), 0, 255).astype(np.int64)
    idx = np.random.RandomState(0).permutation(len(y)); ntr = int(0.8 * len(y)); tr, va = idx[:ntr], idx[ntr:]

    best = None
    for seed in range(a.seeds):
        q = train(X[tr], y[tr], seed)
        acc = (model_int(X[tr], *q)[2] == y[tr]).mean()
        print(f"seed {seed}: train acc {acc*100:.1f}%")
        if best is None or acc > best[0]:
            best = (acc, seed, q)
    tr_acc, seed, q = best
    hid, logit, cls = model_int(X[va], *q)
    va_acc = (cls == y[va]).mean()
    print(f"best seed {seed}: train {tr_acc*100:.1f}%, val {va_acc*100:.1f}% on the integer model")

    params = pack(*q)
    if not a.no_rtl:
        r = run_rtl(params, X[va], AUDIT / "build")
        exact = int(((hid == r["hid"]).all(1) & (logit == r["logit"]).all(1) & (cls == r["cls"])).sum())
        print(f"RTL: {exact}/{len(va)} validation vectors bit-exact (hidden, logits, class); "
              f"RTL val acc {(r['cls'] == y[va]).mean()*100:.1f}%")
        assert exact == len(va), "RTL does not match the integer model"

    W1, b1, W2, b2 = q
    np.savez(HERE / "senseedge_weights.npz",
             layer1_weights=W1.astype(np.int8), layer1_biases=b1.astype(np.int8),
             layer2_weights=W2.astype(np.int8), layer2_biases=b2.astype(np.int8),
             all_weights=params.astype(np.int8), input_domain=np.array(["unsigned_u8_corrected_rtl"]))
    (HERE / "senseedge_weights.hex").write_text("".join(f"{int(v) & 0xFF:02x}\n" for v in params))
    print("saved senseedge_weights.npz and senseedge_weights.hex (212 params); now run export_weights.py")


if __name__ == "__main__":
    main()
