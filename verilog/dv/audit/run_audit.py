#!/usr/bin/env python3
"""Verify the corrected RTL (branch fix/audit-defects) bit for bit against ref_models.py.

  A  nn_engine.v through its weight port: random weights and inputs, and feature-like inputs
  B  capture transfer: every ADC code 0..4095 through the SPI model, sample == code - 2048
  C  tone sweep: a tone in each bin; features == features(bins), bands as designed, peak == bin
  D  Wishbone only, as the Caravel core sees it: 53 weight words written and read back,
     partial byte select, write above the window; then windows: class/confidence on the bus
     == nn_infer(features(bins on the bus))
  E  latency and per-state cycle counts from a full buffer to a decision

Run from this directory:  python3 run_audit.py      (needs iverilog, vvp, numpy)
Writes build/audit_results.json and prints a PASS/FAIL table.
"""
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

from ref_models import capture, features, nn_infer, to_u8

HERE = Path(__file__).resolve().parent
RTL = HERE.parents[1] / "rtl"
BUILD = HERE / "build"
BUILD.mkdir(exist_ok=True)
RTL_FILES = [RTL / f for f in ("defines.v", "spi_adc_if.v", "fft_engine.v", "feature_extract.v",
                                "nn_engine.v", "wb_interface.v", "alarm_logic.v", "senseedge_top.v")]
MID = 2048
results = {}


def sh(cmd):
    return subprocess.run([str(c) for c in cmd], check=True, capture_output=True, text=True, cwd=BUILD)


def compile_tb(name, extra):
    sh(["iverilog", "-g2012", "-o", f"{name}.vvp", HERE / f"{name}.v", *extra])


def write_hex(path, values, width):
    Path(path).write_text("".join(f"{int(v) & ((1 << (4 * width)) - 1):0{width}x}\n" for v in values))


def parse(path):
    out = {}
    for line in Path(path).read_text().splitlines():
        tag, *vals = line.split()
        out.setdefault(tag, []).append([int(v) for v in vals])
    return out


def report(key, ok, detail):
    results[key] = dict(ok=bool(ok), detail=detail)
    print(f"  [{'PASS' if ok else 'FAIL'}] {key}: {detail}")


# ---------------------------------------------------------------- A: network
def exp_nn():
    print("A  nn_engine against nn_infer")
    compile_tb("tb_nn_vectors", [RTL / "nn_engine.v"])
    rng = np.random.RandomState(1)
    total = exact = 0
    worst = None
    runs = []
    for r in range(5):                                   # 5 weight sets x 400 random vectors
        params = rng.randint(-128, 128, 212)
        X = rng.randint(0, 256, (400, 8))
        runs.append((params, X))
    params = rng.randint(-40, 41, 212)                   # small weights, feature-like inputs
    X = np.concatenate([rng.randint(0, 256, (200, 8)), rng.randint(0, 64, (100, 8)),
                        rng.randint(192, 256, (100, 8))])
    runs.append((params, X))
    for params, X in runs:
        write_hex(BUILD / "weights.hex", params, 2)
        write_hex(BUILD / "vectors.hex", X.flatten(), 2)
        sh(["vvp", "tb_nn_vectors.vvp", f"+nvec={len(X)}"])
        out = np.loadtxt(BUILD / "nn_out.txt", dtype=np.int64).reshape(len(X), -1)
        for n in range(len(X)):
            cls, conf, hid, logit = nn_infer(X[n], params)
            total += 1
            ok = (out[n, 0] == cls and out[n, 1] == conf and np.array_equal(out[n, 2:18], hid)
                  and np.array_equal(out[n, 18:22], logit))
            exact += ok
            if not ok and worst is None:
                worst = dict(x=X[n].tolist(), rtl=out[n].tolist(), ref=[cls, conf, hid.tolist(), logit.tolist()])
    report("A.nn_bit_exact", exact == total, f"{exact}/{total} vectors exact (class, confidence, 16 hidden, 4 logits)")
    if worst:
        print("     first mismatch:", worst)
    results["A.nn_bit_exact"]["vectors"] = total


# ---------------------------------------------------------------- chain helpers
def run_chain(codes, div=4):
    codes = np.asarray(codes, dtype=int)
    write_hex(BUILD / "codes.hex", codes.flatten(), 3)
    sh(["vvp", "tb_chain.vvp", f"+nwin={len(codes)}", f"+div={div}"])
    return parse(BUILD / "chain_out.txt")


def tone(k, amp, phase=0.0, n=64):
    return amp * np.sin(2 * np.pi * k * np.arange(n) / n + phase)


# ---------------------------------------------------------------- B: capture
def exp_transfer():
    print("B  capture transfer, all 4096 codes")
    compile_tb("tb_chain", RTL_FILES)
    codes = np.arange(4096).reshape(64, 64)
    r = run_chain(codes)
    samples = np.array([row[1:] for row in r["S"]]).flatten()
    ref = np.array([capture(c) for c in codes.flatten()])
    bad = int((samples != ref).sum())
    report("B.capture_transfer", bad == 0, f"{4096 - bad}/4096 codes give sample = code - 2048")
    results["B.capture_transfer"]["range"] = [int(samples.min()), int(samples.max())]


# ---------------------------------------------------------------- C: tone sweep
BAND_OF = {k: b for b, (lo, hi) in enumerate([(1, 4), (5, 10), (11, 20), (21, 31)]) for k in range(lo, hi + 1)}


def exp_tonesweep():
    print("C  tone sweep, one tone per bin")
    codes = np.array([np.clip(np.round(MID + 50 + tone(k, 800)), 0, 4095).astype(int) for k in range(32)])
    r = run_chain(codes)
    bins = np.array([row[1:] for row in r["M"]])
    feats = np.array([row[1:] for row in r["F"]])
    exact = sum(np.array_equal(feats[k], features(bins[k])) for k in range(32))
    report("C.features_bit_exact", exact == 32, f"{exact}/32 windows: RTL features == features(RTL bins)")
    peak_ok = sum(int(feats[k][4] == 8 * k) for k in range(32))
    report("C.peak_bin", peak_ok == 32, f"{peak_ok}/32 windows report the stimulus bin as the peak")
    band_ok = 0
    for k in range(1, 32):
        nz = [b for b in range(4) if feats[k][b] != 0]
        band_ok += int(nz == [BAND_OF[k]])
    report("C.bands_as_designed", band_ok == 31, f"{band_ok}/31 tones land only in their designed band "
           "(1-4, 5-10, 11-20, 21-31)")
    dc = feats[0]
    report("C.dc_not_in_bands", all(dc[b] == 0 for b in range(4)), f"DC-only window: band features {dc[:4].tolist()}")


# ---------------------------------------------------------------- D: bus
def exp_bus():
    print("D  Wishbone only, as the management core sees it")
    compile_tb("tb_bus", RTL_FILES)
    rng = np.random.RandomState(3)
    params = rng.randint(-128, 128, 212)
    write_hex(BUILD / "weights.hex", params, 2)
    nwin = 8
    codes = []
    for w in range(nwin):
        ks = rng.uniform(1, 31, 3)
        x = sum(tone(k, rng.uniform(100, 500), rng.uniform(0, 6.3)) for k in ks) + rng.normal(0, 6, 64)
        codes.append(np.clip(np.round(MID + rng.randint(-60, 61) + x), 0, 4095).astype(int))
    write_hex(BUILD / "codes.hex", np.array(codes).flatten(), 3)
    sh(["vvp", "tb_bus.vvp", f"+nwin={nwin}", "+div=4"])
    r = parse(BUILD / "bus_out.txt")
    words = [sum(to_u8(params[4 * w + k]) << (8 * k) for k in range(4)) for w in range(53)]
    got = [row[1] for row in r["W"]]
    report("D.weights_readback", got == words, f"{sum(a == b for a, b in zip(got, words))}/53 words read back "
           "equal to what was written (all 212 weights reachable)")
    expect_p = (words[0] & ~0xFF00) | 0xA500
    report("D.partial_select", r["P"][0][0] == expect_p, f"byte-lane-1 write changed only lane 1: "
           f"read 0x{r['P'][0][0]:08x}, expected 0x{expect_p:08x}")
    report("D.above_window", r["X"][0] == [0, words[52]], f"write at 0xF4 ignored: reads {r['X'][0][0]}, "
           f"word 52 still 0x{r['X'][0][1]:08x}")
    bins = np.array([row[1:] for row in r["M"]])
    feats = np.array([row[1:] for row in r["F"]])
    cls = [row[1:] for row in r["C"]]
    f_ok = c_ok = 0
    for w in range(nwin):
        f = features(bins[w])
        f_ok += int(np.array_equal(feats[w], f))
        c, conf, _, _ = nn_infer(f, params)
        c_ok += int(cls[w] == [c, conf])
    report("D.features_on_bus", f_ok == nwin, f"{f_ok}/{nwin} windows: FEATURE_DATA == features(FFT_DATA)")
    report("D.class_on_bus", c_ok == nwin, f"{c_ok}/{nwin} windows: CLASS register == nn_infer(features) "
           "with weights loaded over the bus")


# ---------------------------------------------------------------- E: latency
def exp_latency():
    print("E  latency")
    codes = np.clip(np.round(MID + tone(8, 600)), 0, 4095).astype(int)[None, :]
    r = run_chain(codes)
    lat = r["LAT"][0][0]
    fft, fe, nn = r["FFT"][0], r["FE"][0], r["NN"][0]
    results["E.latency"] = dict(ok=True, buffer_full_to_decision=lat, fft_states=fft, fe_states=fe, nn_states=nn,
                                detail=f"{lat} clocks from full buffer to decision (fabricated part: 729)")
    print(f"  [INFO] E.latency: {lat} clocks; FFT states {fft}, FE states {fe}, NN states {nn}")


if __name__ == "__main__":
    for fn in (exp_nn, exp_transfer, exp_tonesweep, exp_bus, exp_latency):
        try:
            fn()
        except subprocess.CalledProcessError as e:
            print("  simulator error:", e.stderr[-2000:] or e.stdout[-2000:])
            results[fn.__name__] = dict(ok=False, detail="simulator error")
    (BUILD / "audit_results.json").write_text(json.dumps(results, indent=1))
    failed = [k for k, v in results.items() if not v["ok"]]
    print("\nRESULT:", "ALL PASS" if not failed else f"FAILED {failed}")
    sys.exit(1 if failed else 0)
