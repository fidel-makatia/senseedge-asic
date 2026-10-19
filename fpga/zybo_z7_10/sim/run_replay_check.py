#!/usr/bin/env python3
"""Simulate a dataset replay through the Zybo wrapper (tb_replay.v) and check every window
against the Python reference chain: 32 bins, 8 features, class, confidence, and the alarm
flag, block by block exactly as the host-mode protocol runs it on the board.

Usage:
  run_replay_check.py [--records file.npz] [--n 256] [--div 9] [--blk 64]
                      [--weights file.npz|file.hex] [--thr 100] [--ncons 3] [--seed 1] [--split test]
Without --records the stimulus is tones in random bins; without --weights the diagonal image
of the self-test is used (class k <- band k).
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(REPO / "verilog" / "dv" / "audit"))
from ref_models import alarm_model, chain  # noqa: E402

RTL = REPO / "verilog" / "rtl"
FILES = [HERE / "tb_replay.v", HERE.parent / "rtl" / "axil_senseedge.v", HERE.parent / "rtl" / "mcp3201_emu.v"] + \
        [RTL / f for f in ("spi_adc_if.v", "fft_engine.v", "feature_extract.v", "nn_engine.v", "wb_interface.v",
                           "alarm_logic.v", "senseedge_top.v")]
BUILD = HERE / "build"


def load_weights(path):
    if path is None:
        w = np.zeros(212, np.int64)
        for k in range(4):
            w[9 * k] = 127          # hidden k <- input k (W1 row k, column k)
            w[144 + 17 * k] = 127   # class k <- hidden k (W2 row k, column k)
        return w
    if path.suffix == ".npz":
        return np.load(path)["all_weights"].astype(np.int64)
    return np.array([int(l, 16) for l in path.read_text().split()], np.int64).astype(np.int8).astype(np.int64)


def pick_windows(a):
    rng = np.random.RandomState(a.seed)
    if a.records:
        d = np.load(a.records)
        codes, seq, split = d["codes"], d["seq"], d["seq_split"]
        ok = np.flatnonzero(split[seq] == a.split) if a.split != "all" else np.arange(len(codes))
        idx = np.sort(rng.choice(ok, size=min(a.n, len(ok)), replace=False))
        return codes[idx].astype(np.int64), d["label"][idx], idx
    t = np.arange(64)
    wins, labels = [], []
    for i in range(a.n):
        k = rng.randint(1, 32); amp = rng.randint(50, 1500); ph = rng.uniform(0, 2 * np.pi)
        wins.append(np.clip(np.round(2048 + amp * np.sin(2 * np.pi * k * t / 64 + ph)), 0, 4095).astype(np.int64))
        labels.append(-1)
    return np.array(wins), np.array(labels), np.arange(a.n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--records", type=Path)
    ap.add_argument("--split", default="test")
    ap.add_argument("--n", type=int, default=256)
    ap.add_argument("--div", type=int, default=9)
    ap.add_argument("--blk", type=int, default=64)
    ap.add_argument("--weights", type=Path)
    ap.add_argument("--thr", type=int, default=100)
    ap.add_argument("--ncons", type=int, default=3)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    assert a.n <= 1024 and 1 <= a.blk <= 64
    wins, labels, idx = pick_windows(a)
    params = load_weights(a.weights)

    BUILD.mkdir(exist_ok=True)
    (BUILD / "codes.hex").write_text("".join(f"{int(v) & 0xFFF:03x}\n" for v in wins.flatten()))
    (BUILD / "weights.hex").write_text("".join(f"{int(v) & 0xFF:02x}\n" for v in params))
    vvp = BUILD / "tb_replay.vvp"
    subprocess.run(["iverilog", "-g2012", "-o", str(vvp), *map(str, FILES)], check=True)
    t0 = time.time()
    r = subprocess.run(["vvp", str(vvp), f"+nwin={len(wins)}", f"+div={a.div}", f"+blk={a.blk}", f"+thr={a.thr}",
                        f"+ncons={a.ncons}"], check=True, capture_output=True, text=True, cwd=BUILD)
    t_sim = time.time() - t0
    if "FAIL" in r.stdout or "TIMEOUT" in r.stdout:
        print(r.stdout)

    out = {}
    werr = 0; blocks = 0
    for line in (BUILD / "replay_out.txt").read_text().splitlines():
        tag, *v = line.split()
        if tag == "B":
            blocks += 1; werr += int(v[2])
        elif tag == "W":
            out.setdefault(int(v[0]), {})["dec"] = (int(v[1]), int(v[2]), int(v[3]))
        elif tag == "F":
            out.setdefault(int(v[0]), {})["feat"] = [int(x) for x in v[1:]]
        elif tag == "M":
            out.setdefault(int(v[0]), {})["bins"] = [int(x) for x in v[1:]]

    bad_bins = bad_feat = bad_cls = bad_alarm = 0
    exp_dec = []
    for g in range(len(wins)):
        m, f, c, cf = chain(wins[g], params)
        exp_dec.append((c, cf))
        o = out.get(g, {})
        if o.get("bins") != m: bad_bins += 1
        if o.get("feat") != f: bad_feat += 1
        if o.get("dec", (None, None, None))[:2] != (c, cf): bad_cls += 1
    # alarm: the chip is reset at every block start, so the model restarts per block
    for g0 in range(0, len(wins), a.blk):
        exp_al = alarm_model(exp_dec[g0:g0 + a.blk], a.thr, a.ncons)
        for k, al in enumerate(exp_al):
            if out.get(g0 + k, {}).get("dec", (None, None, None))[2] != al:
                bad_alarm += 1
    n = len(wins)
    acc = None
    if (labels >= 0).all():
        pred = np.array([exp_dec[g][0] for g in range(n)])
        acc = float((pred == labels).mean())
    summary = dict(windows=n, blocks=blocks, div=a.div, weight_readback_errors=werr, bins_mismatch=bad_bins,
                   feature_mismatch=bad_feat, class_or_conf_mismatch=bad_cls, alarm_mismatch=bad_alarm,
                   sim_seconds=round(t_sim, 1), window_accuracy_on_this_subset=acc,
                   records=str(a.records) if a.records else "tones", weights=str(a.weights) if a.weights else "diagonal")
    (BUILD / "replay_check.json").write_text(json.dumps(summary, indent=1))
    ok = werr == 0 and bad_bins == 0 and bad_feat == 0 and bad_cls == 0 and bad_alarm == 0 and len(out) == n
    print(f"ZYBO REPLAY SIM: {'PASS' if ok else 'FAIL'}: {n} windows in {blocks} blocks at div={a.div}; "
          f"weight read-back errors {werr}; mismatches: bins {bad_bins}, features {bad_feat}, class/conf {bad_cls}, "
          f"alarm {bad_alarm}" + (f"; accuracy on these windows {100 * acc:.1f} %" if acc is not None else "") +
          f"; {t_sim:.0f} s")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
