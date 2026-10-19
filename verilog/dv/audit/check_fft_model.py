#!/usr/bin/env python3
"""Prove ref_models.fft64 (and chain) bit-exact against fft_engine.v inside senseedge_top.

Drives tb_chain.v (the audit's acquisition -> FFT -> features harness with the data-sheet ADC
model) with windows of 12-bit codes and compares every magnitude bin and every feature with
the Python model.  Stimulus, by default 1024 windows:
  - random full-range codes (the arithmetic at the limits of the 24-bit store),
  - one tone per bin at three amplitudes, with random phase and offset,
  - corner cases: all 0, all 4095, alternating 0/4095, impulses, mid-scale constant,
  - optionally windows from a records file made by fpga/zybo_z7_10/host/prepare_records.py.

Usage:  python3 check_fft_model.py [--n 1024] [--records file.npz] [--div 4] [--seed 1]
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from ref_models import chain  # noqa: E402

RTL = HERE.parents[1] / "rtl"
RTL_FILES = [RTL / f for f in ("defines.v", "spi_adc_if.v", "fft_engine.v", "feature_extract.v",
                                "nn_engine.v", "wb_interface.v", "alarm_logic.v", "senseedge_top.v")]
BUILD = HERE / "build" / "fftcheck"


def stimulus(n, seed, records=None):
    rng = np.random.RandomState(seed)
    wins = []
    if records is not None:
        codes = np.load(records)["codes"]
        pick = rng.choice(len(codes), size=min(n, len(codes)), replace=False)
        wins = [codes[i].astype(np.int64) for i in sorted(pick)]
        return np.array(wins[:n])
    t = np.arange(64)
    # corner cases
    wins.append(np.zeros(64, np.int64)); wins.append(np.full(64, 4095, np.int64))
    wins.append(np.where(t % 2 == 0, 0, 4095)); wins.append(np.full(64, 2048, np.int64))
    for k in (0, 1, 63):
        w = np.full(64, 2048, np.int64); w[k] = 4095; wins.append(w.copy()); w[k] = 0; wins.append(w.copy())
    # one tone per bin, three amplitudes
    for amp in (100, 1000, 2047):
        for k in range(1, 32):
            ph = rng.uniform(0, 2 * np.pi); off = rng.randint(-40, 41)
            w = np.clip(np.round(2048 + off + amp * np.sin(2 * np.pi * k * t / 64 + ph)), 0, 4095)
            wins.append(w.astype(np.int64))
    # random full range
    while len(wins) < n:
        wins.append(rng.randint(0, 4096, size=64).astype(np.int64))
    return np.array(wins[:n])


def run_rtl(wins, div):
    BUILD.mkdir(parents=True, exist_ok=True)
    vvp = BUILD / "tb_chain.vvp"
    subprocess.run(["iverilog", "-g2012", "-o", str(vvp), str(HERE / "tb_chain.v"), *map(str, RTL_FILES)], check=True)
    (BUILD / "codes.hex").write_text("".join(f"{int(v) & 0xFFF:03x}\n" for v in wins.flatten()))
    subprocess.run(["vvp", str(vvp), f"+nwin={len(wins)}", f"+div={div}"], check=True, capture_output=True, cwd=BUILD)
    bins, feats = {}, {}
    for line in (BUILD / "chain_out.txt").read_text().splitlines():
        tag, *vals = line.split()
        if tag == "M":
            bins[int(vals[0])] = [int(v) for v in vals[1:]]
        elif tag == "F":
            feats[int(vals[0])] = [int(v) for v in vals[1:]]
    return bins, feats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=1024)
    ap.add_argument("--records", type=Path)
    ap.add_argument("--div", type=int, default=4)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    assert a.n <= 1024, "tb_chain holds 1024 windows"
    wins = stimulus(a.n, a.seed, a.records)
    t0 = time.time()
    bins, feats = run_rtl(wins, a.div)
    t_rtl = time.time() - t0
    bad = 0
    for w in range(len(wins)):
        m, f, _, _ = chain(wins[w])
        if bins.get(w) != m or feats.get(w) != f:
            bad += 1
            if bad <= 5:
                print(f"MISMATCH window {w}: rtl bins {bins.get(w)}\n  model {m}\n  rtl feats {feats.get(w)} model {f}")
    src = f"records {a.records.name}" if a.records else "random, tones, corners"
    print(f"fft64/chain vs fft_engine.v: {len(wins) - bad}/{len(wins)} windows bit-exact "
          f"(32 bins and 8 features each), div={a.div}, {src}, RTL run {t_rtl:.0f} s")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
