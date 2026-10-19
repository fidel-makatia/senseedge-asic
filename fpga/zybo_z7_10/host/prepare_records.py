#!/usr/bin/env python3
"""Turn a public vibration dataset into SenseEdge replay records: 12-bit MCP3201 codes in
64-sample windows at the chip's sample rate, with sequence-level labels and a fixed split.

Datasets
  mafaulda  MaFaulDa (UFRJ), 1951 sequences of 5 s at 50 kHz, 8 columns:
            0 tachometer, 1-3 underhang accelerometer (axial, radial, tangential),
            4-6 overhang accelerometer (axial, radial, tangential), 7 microphone.
            Classes mapped to the chip's four outputs:
              normal -> 0 HEALTHY, underhang/* and overhang/* -> 1 BEARING_WEAR,
              imbalance -> 2 IMBALANCE, horizontal-/vertical-misalignment -> 3 MISALIGNMENT.
            Default channel 2 (underhang radial): one single-axis sensor for every class, as on
            the SenseEdge node.  The file stem is the rotation frequency in Hz.
  cwru      CWRU bearing data, drive-end accelerometer; normal 97-100 (48 kHz) and 12 kHz
            drive-end faults IR/B/OR at 0.007, 0.014, 0.021 in, loads 0-3 hp:
              normal -> 0, inner race -> 1, ball -> 2, outer race (6 o'clock) -> 3.
            Default split: load 3 hp held out (unseen-load protocol).

Signal path (identical for every sequence; nothing is fitted per sequence except its mean)
  1. one channel, mean removed (the node's AC coupling),
  2. polyphase resampling from the source rate to f_s = 20 MHz / (35 (D+1)) with the exact
     rational ratio (anti-aliasing included),
  3. one global gain, fixed from the HEALTHY class only: the 99.9th percentile of |x| maps to
     `--gain-target` codes (default 1024, a quarter of the ADC range),
  4. codes = clip(round(2048 + gain x), 0, 4095); the first and last 64 samples are dropped
     (filter edges) and the rest cut into non-overlapping 64-sample windows.

Sample-rate policy
  fixed    one divider for all sequences (default D=249: 2285.7 S/s, the chip's reset value)
  tracked  per-sequence divider so that the shaft line sits in bin 2 (f_s = 32 f_rot); the
           replay script programs CLK_DIV per sequence (only the replay speed changes; the
           datapath does not depend on D).

Output: an .npz with
  codes      uint16 [N, 64]   N windows
  seq, win   int32  [N]       sequence index and window index within the sequence
  label      int8   [N]       class 0..3
  seq_path, seq_cond, seq_split   str [S]     per sequence
  seq_class  int8 [S]; seq_speed_hz, seq_fs float32 [S]; seq_div int32 [S]
  meta       json string: dataset, channel, policy, div, gain, percentile, clipping per class, ...

Usage
  prepare_records.py mafaulda --root ~/datasets/mafaulda/extracted --out records/mafaulda_d249.npz
  prepare_records.py cwru --root ~/datasets/cwru/raw --out records/cwru_d249.npz
  options: --div 249 | --policy tracked, --channel 2, --split seq --test-frac 0.3 --seed 0,
           --split cond:<regex> (sequences whose condition matches go to the test set),
           --max-per-class N (quick runs), --jobs 16
"""
import argparse
import json
import re
import sys
import time
from datetime import date
from fractions import Fraction
from multiprocessing import Pool
from pathlib import Path

import numpy as np
from scipy.signal import resample_poly

F_CLK = 20_000_000
CLASS_NAMES = {"mafaulda": ["HEALTHY", "BEARING_WEAR", "IMBALANCE", "MISALIGNMENT"],
               "cwru": ["NORMAL", "INNER_RACE", "BALL", "OUTER_RACE"]}
CWRU_FILES = {  # file number: (class, condition)
    97: (0, "normal_0hp"), 98: (0, "normal_1hp"), 99: (0, "normal_2hp"), 100: (0, "normal_3hp"),
    105: (1, "IR007_0hp"), 106: (1, "IR007_1hp"), 107: (1, "IR007_2hp"), 108: (1, "IR007_3hp"),
    118: (2, "B007_0hp"), 119: (2, "B007_1hp"), 120: (2, "B007_2hp"), 121: (2, "B007_3hp"),
    130: (3, "OR007_0hp"), 131: (3, "OR007_1hp"), 132: (3, "OR007_2hp"), 133: (3, "OR007_3hp"),
    169: (1, "IR014_0hp"), 170: (1, "IR014_1hp"), 171: (1, "IR014_2hp"), 172: (1, "IR014_3hp"),
    185: (2, "B014_0hp"), 186: (2, "B014_1hp"), 187: (2, "B014_2hp"), 188: (2, "B014_3hp"),
    197: (3, "OR014_0hp"), 198: (3, "OR014_1hp"), 199: (3, "OR014_2hp"), 200: (3, "OR014_3hp"),
    209: (1, "IR021_0hp"), 210: (1, "IR021_1hp"), 211: (1, "IR021_2hp"), 212: (1, "IR021_3hp"),
    222: (2, "B021_0hp"), 223: (2, "B021_1hp"), 224: (2, "B021_2hp"), 225: (2, "B021_3hp"),
    234: (3, "OR021_0hp"), 235: (3, "OR021_1hp"), 236: (3, "OR021_2hp"), 237: (3, "OR021_3hp"),
}


def fs_of_div(d):
    return F_CLK / (35 * (d + 1))


def div_for_fs(fs):
    return max(0, int(round(F_CLK / (35 * fs))) - 1)


def mafaulda_class(top):
    return {"normal": 0, "underhang": 1, "overhang": 1, "imbalance": 2,
            "horizontal-misalignment": 3, "vertical-misalignment": 3}[top]


def list_mafaulda(root, channel):
    seqs = []
    for f in sorted(root.rglob("*.csv")):
        rel = f.relative_to(root)
        top = rel.parts[0]
        if top not in ("normal", "underhang", "overhang", "imbalance", "horizontal-misalignment", "vertical-misalignment"):
            continue
        try:
            speed = float(f.stem)
        except ValueError:
            speed = float("nan")
        seqs.append(dict(path=str(f), rel=str(rel), cls=mafaulda_class(top), cond="/".join(rel.parts[:-1]),
                         speed=speed, fs_in=50000, channel=channel, kind="csv"))
    return seqs


def list_cwru(root, channel):
    seqs = []
    for n, (cls, cond) in CWRU_FILES.items():
        f = root / f"{n}.mat"
        if not f.exists():
            print(f"warning: {f} missing", file=sys.stderr)
            continue
        seqs.append(dict(path=str(f), rel=f.name, cls=cls, cond=cond, speed=float("nan"),
                         fs_in=48000 if cls == 0 else 12000, channel=channel, kind=f"mat:{n}"))
    return seqs


def read_signal(s):
    if s["kind"] == "csv":
        try:
            import pandas as pd
            x = pd.read_csv(s["path"], header=None, usecols=[s["channel"]], dtype=np.float64).values[:, 0]
            if np.isnan(s["speed"]):
                tach = pd.read_csv(s["path"], header=None, usecols=[0], dtype=np.float64).values[:, 0]
                s["speed"] = tach_speed(tach, s["fs_in"])
        except ImportError:
            x = np.loadtxt(s["path"], delimiter=",", usecols=(s["channel"],))
        return x
    import scipy.io
    n = int(s["kind"].split(":")[1])
    m = scipy.io.loadmat(s["path"])
    x = m[f"X{n:03d}_DE_time"][:, 0].astype(np.float64)
    rpm = m.get(f"X{n:03d}RPM")
    s["speed"] = float(rpm[0, 0]) / 60.0 if rpm is not None else float("nan")
    return x


def tach_speed(tach, fs):
    """Rotation frequency from the tachometer pulses (rising edges through the mid level)."""
    mid = 0.5 * (tach.max() + tach.min())
    up = np.flatnonzero((tach[1:] > mid) & (tach[:-1] <= mid))
    return float(len(up) / (len(tach) / fs)) if len(up) > 2 else float("nan")


def process(args):
    s, policy, div_fixed = args
    x = read_signal(s)
    x = x - x.mean()
    if policy == "tracked" and np.isfinite(s["speed"]) and s["speed"] > 0:
        d = div_for_fs(32.0 * s["speed"])
    else:
        d = div_fixed
    r = Fraction(F_CLK, int(s["fs_in"]) * 35 * (d + 1))
    y = resample_poly(x, r.numerator, r.denominator)
    y = y[64:-64]
    y = y[: (len(y) // 64) * 64]
    s = dict(s); s["div"] = d; s["fs"] = fs_of_div(d)
    return s, y.astype(np.float32)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset", choices=["mafaulda", "cwru"])
    ap.add_argument("--root", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--channel", type=int, default=None, help="mafaulda column (default 2); cwru uses the DE channel")
    ap.add_argument("--policy", choices=["fixed", "tracked"], default="fixed")
    ap.add_argument("--div", type=int, default=249)
    ap.add_argument("--gain-percentile", type=float, default=99.9)
    ap.add_argument("--gain-target", type=float, default=1024.0)
    ap.add_argument("--split", default=None, help="'seq' (stratified random by sequence) or 'cond:<regex>'")
    ap.add_argument("--test-frac", type=float, default=0.3)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--max-per-class", type=int, default=None)
    ap.add_argument("--jobs", type=int, default=8)
    a = ap.parse_args()

    channel = 2 if a.channel is None else a.channel
    seqs = list_mafaulda(a.root, channel) if a.dataset == "mafaulda" else list_cwru(a.root, channel)
    if not seqs:
        sys.exit(f"no sequences found under {a.root}")
    split_mode = a.split or ("seq" if a.dataset == "mafaulda" else "cond:_3hp$")
    rng = np.random.RandomState(a.seed)
    if a.max_per_class:
        keep = []
        for c in range(4):
            idx = [i for i, s in enumerate(seqs) if s["cls"] == c]
            rng.shuffle(idx)
            keep += idx[: a.max_per_class]
        seqs = [seqs[i] for i in sorted(keep)]
    print(f"{len(seqs)} sequences; reading and resampling with {a.jobs} workers ...", flush=True)
    t0 = time.time()
    with Pool(a.jobs) as pool:
        done = pool.map(process, [(s, a.policy, a.div) for s in seqs], chunksize=4)
    seqs = [d[0] for d in done]
    sigs = [d[1] for d in done]
    print(f"  {time.time() - t0:.0f} s", flush=True)

    # one global gain from the healthy class
    healthy = np.concatenate([y for s, y in zip(seqs, sigs) if s["cls"] == 0])
    p = float(np.percentile(np.abs(healthy), a.gain_percentile))
    gain = a.gain_target / p
    # split
    if split_mode == "seq":
        groups = {}
        for i, s in enumerate(seqs):
            groups.setdefault((s["cls"], s["cond"]), []).append(i)
        test = set()
        for g in groups.values():
            g = list(g); rng.shuffle(g)
            n_test = int(round(a.test_frac * len(g)))
            if len(g) >= 2 and n_test == 0:
                n_test = 1
            test.update(g[:n_test])
    else:
        pat = re.compile(split_mode.split(":", 1)[1])
        test = {i for i, s in enumerate(seqs) if pat.search(s["cond"])}
    for i, s in enumerate(seqs):
        s["split"] = "test" if i in test else "train"

    codes, seq_i, win_i, label = [], [], [], []
    clip = np.zeros(4); total = np.zeros(4)
    for i, (s, y) in enumerate(zip(seqs, sigs)):
        c = np.round(2048.0 + gain * y.astype(np.float64))
        clip[s["cls"]] += int(((c < 0) | (c > 4095)).sum()); total[s["cls"]] += len(c)
        c = np.clip(c, 0, 4095).astype(np.uint16).reshape(-1, 64)
        codes.append(c); seq_i.append(np.full(len(c), i, np.int32)); win_i.append(np.arange(len(c), dtype=np.int32))
        label.append(np.full(len(c), s["cls"], np.int8))
    codes = np.concatenate(codes); seq_i = np.concatenate(seq_i); win_i = np.concatenate(win_i); label = np.concatenate(label)
    meta = dict(dataset=a.dataset, root=str(a.root), channel=channel, policy=a.policy, div=a.div,
                fs=fs_of_div(a.div), gain=gain, percentile=a.gain_percentile, percentile_value=p,
                gain_target=a.gain_target, split=split_mode, test_frac=a.test_frac, seed=a.seed,
                class_names=CLASS_NAMES[a.dataset],
                clip_fraction={CLASS_NAMES[a.dataset][c]: float(clip[c] / max(1, total[c])) for c in range(4)},
                sequences=len(seqs), windows=int(len(codes)), created=str(date.today()))
    a.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(a.out, codes=codes, seq=seq_i, win=win_i, label=label,
                        seq_path=np.array([s["rel"] for s in seqs]), seq_cond=np.array([s["cond"] for s in seqs]),
                        seq_split=np.array([s["split"] for s in seqs]), seq_class=np.array([s["cls"] for s in seqs], np.int8),
                        seq_speed_hz=np.array([s["speed"] for s in seqs], np.float32),
                        seq_fs=np.array([s["fs"] for s in seqs], np.float32), seq_div=np.array([s["div"] for s in seqs], np.int32),
                        meta=json.dumps(meta))
    names = CLASS_NAMES[a.dataset]
    print(f"wrote {a.out}: {len(codes)} windows from {len(seqs)} sequences; gain {gain:.4g} "
          f"(p{a.gain_percentile} of healthy |x| = {p:.4g} -> {a.gain_target:g} codes)")
    for c in range(4):
        ns = sum(1 for s in seqs if s["cls"] == c); nt = sum(1 for s in seqs if s["cls"] == c and s["split"] == "test")
        print(f"  {names[c]:13s} {ns:5d} sequences ({nt} test), {int((label == c).sum()):7d} windows, "
              f"clipped samples {100 * clip[c] / max(1, total[c]):.3f} %")


if __name__ == "__main__":
    main()
