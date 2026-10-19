#!/usr/bin/env python3
"""Gate-level and stress simulations of the corrected RTL, before any FPGA.

  G1  nn_engine.v synthesised by Yosys (generic gates): 2400 vectors, class and confidence
  G2  senseedge_top synthesised by Yosys (generic gates), observed through the bus only:
      same 8 windows as run_audit D; bins, features and class must equal the RTL run and
      the reference
  G3  senseedge_top mapped to sky130_fd_sc_hd (Yosys dfflibmap + abc with the PDK liberty),
      simulated with the PDK's own Verilog cell models (functional, unit delay): same check
  S   stress scenarios of tb_stress.v on RTL, on the generic netlist and on the sky130
      netlist: every scenario must give the reference bins, features and class, the IRQ
      flag must set and clear, and the sim must not stall

Run from this directory after run_audit.py:  python3 run_gl.py
Writes build/gl_results.json.
"""
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

from ref_models import features, nn_infer, to_u8

HERE = Path(__file__).resolve().parent
RTL = HERE.parents[1] / "rtl"
BUILD = HERE / "build" / "gl"          # own directory: never shared with run_audit.py or manual runs
RTL_FILES = [RTL / f for f in ("defines.v", "spi_adc_if.v", "fft_engine.v", "feature_extract.v",
                                "nn_engine.v", "wb_interface.v", "alarm_logic.v", "senseedge_top.v")]
YOSYS_DAT = Path(subprocess.check_output(["yosys-config", "--datdir"], text=True).strip())
SIMCELLS = [YOSYS_DAT / "simcells.v", YOSYS_DAT / "simlib.v"]
PDK = Path(os.environ.get("PDK_ROOT", Path.home() / "chipfoundry/senseedge/dependencies/pdks/ciel/sky130/versions/"
                          "0fe599b2afb6708d281543108caf8310912f54af")) / "sky130A/libs.ref/sky130_fd_sc_hd"
LIB = PDK / "lib" / "sky130_fd_sc_hd__tt_025C_1v80.lib"
PDK_V = [PDK / "verilog" / "primitives.v", PDK / "verilog" / "sky130_fd_sc_hd.v"]
MID = 2048
results = {}


def sh(cmd, **kw):
    return subprocess.run([str(c) for c in cmd], check=True, capture_output=True, text=True, cwd=BUILD, **kw)


def write_hex(path, values, width):
    Path(path).write_text("".join(f"{int(v) & ((1 << (4 * width)) - 1):0{width}x}\n" for v in values))


def parse(path):
    out = {}
    for line in Path(path).read_text().splitlines():
        tag, *vals = line.split()
        out.setdefault(tag, []).append(vals if tag == "E" else [int(v) for v in vals])
    return out


def report(key, ok, detail):
    results[key] = dict(ok=bool(ok), detail=detail)
    print(f"  [{'PASS' if ok else 'FAIL'}] {key}: {detail}")


def tone(k, amp, phase=0.0, n=64):
    return amp * np.sin(2 * np.pi * k * np.arange(n) / n + phase)


def synth(out, top, files, sky130=False):
    files = " ".join(str(f) for f in files)
    if sky130:
        script = (f"read_verilog {files}; synth -top {top}; dfflibmap -liberty {LIB}; abc -liberty {LIB}; "
                  f"opt_clean -purge; write_verilog -noattr {out}")
    else:
        script = f"read_verilog {files}; synth -top {top}; opt_clean -purge; write_verilog -noattr {out}"
    sh(["yosys", "-q", "-p", script])


def bus_windows(seed=3, nwin=8):
    """The same stimulus as run_audit.py experiment D."""
    rng = np.random.RandomState(seed)
    params = rng.randint(-128, 128, 212)
    codes = []
    for w in range(nwin):
        ks = rng.uniform(1, 31, 3)
        x = sum(tone(k, rng.uniform(100, 500), rng.uniform(0, 6.3)) for k in ks) + rng.normal(0, 6, 64)
        codes.append(np.clip(np.round(MID + rng.randint(-60, 61) + x), 0, 4095).astype(int))
    return params, np.array(codes)


def run_bus(tag, dut_files, defines=(), nwin=8):
    sh(["iverilog", "-g2012", "-o", f"tb_bus_{tag}.vvp", *[f"-D{d}" for d in defines], HERE / "tb_bus.v", *dut_files])
    sh(["vvp", f"tb_bus_{tag}.vvp", f"+nwin={nwin}", "+div=4"])
    return parse(BUILD / "bus_out.txt")


def check_bus(tag, r, params, ref):
    bins = np.array([row[1:] for row in r["M"]]); feats = np.array([row[1:] for row in r["F"]]); cls = [row[1:] for row in r["C"]]
    words = [sum(to_u8(params[4 * w + k]) << (8 * k) for k in range(4)) for w in range(53)]
    ok_w = [row[1] for row in r["W"]] == words
    f_ok = sum(int(np.array_equal(feats[w], features(bins[w]))) for w in range(len(bins)))
    c_ok = sum(int(cls[w] == list(nn_infer(features(bins[w]), params)[:2])) for w in range(len(bins)))
    same = ref is not None and np.array_equal(bins, ref["bins"]) and np.array_equal(feats, ref["feats"]) and cls == ref["cls"]
    report(f"{tag}.weights_readback", ok_w, "53/53 weight words read back" if ok_w else "weight read-back differs")
    report(f"{tag}.features_and_class", f_ok == len(bins) and c_ok == len(bins),
           f"{f_ok}/{len(bins)} features and {c_ok}/{len(bins)} classes equal the reference on the bus")
    if ref is not None:
        report(f"{tag}.equals_rtl", same, "bins, features and classes identical to the RTL run" if same
               else "differs from the RTL run")
    return dict(bins=bins, feats=feats, cls=cls)


def run_stress(tag, dut_files, params, codes, scenarios=range(7), defines=()):
    sh(["iverilog", "-g2012", "-o", f"tb_stress_{tag}.vvp", *[f"-D{d}" for d in defines], HERE / "tb_stress.v", *dut_files])
    write_hex(BUILD / "weights.hex", params, 2)
    write_hex(BUILD / "codes.hex", codes[0], 3)
    want_f = features  # evaluated per scenario from its own bins
    cls_ref = None
    passed = []
    for scn in scenarios:
        sh(["vvp", f"tb_stress_{tag}.vvp", f"+scn={scn}"])
        r = parse(BUILD / "stress_out.txt")
        if "M" not in r:
            passed.append((scn, False, "no window completed (stall or timeout)")); continue
        bins = np.array(r["M"][0][1:]); feats = np.array(r["F"][0][1:]); cls = r["C"][0][1:]
        f_ok = np.array_equal(feats, want_f(bins))
        exp = list(nn_infer(want_f(bins), params)[:2])
        if scn == 0:
            cls_ref = (bins.tolist(), feats.tolist(), cls)
        same = scn == 0 or (bins.tolist(), feats.tolist(), cls) == cls_ref
        irq_ok = r["I"][0] == [1, 0]
        err = r.get("E", [])
        ok = f_ok and cls == exp and same and irq_ok and not err
        passed.append((scn, ok, f"T={r['T'][0][0]} clocks, features {'ok' if f_ok else 'BAD'}, class {cls}"
                       f"{'' if cls == exp else ' != ' + str(exp)}, {'same as baseline' if same else 'DIFFERS'}, "
                       f"irq {'set/cleared' if irq_ok else 'BAD'}{', errors ' + str(err) if err else ''}"))
    allok = all(p[1] for p in passed)
    report(f"{tag}.stress", allok, "; ".join(f"scn{p[0]} {'ok' if p[1] else 'FAIL'}" for p in passed))
    for p in passed:
        print(f"       scn {p[0]}: {p[2]}")


def main():
    BUILD.mkdir(parents=True, exist_ok=True)
    params, codes = bus_windows()
    write_hex(BUILD / "weights.hex", params, 2)
    write_hex(BUILD / "codes.hex", codes.flatten(), 3)

    print("RTL reference run of the bus windows")
    ref = check_bus("RTL", run_bus("rtl", RTL_FILES), params, None)

    print("G1  nn_engine, generic gate netlist, 2400 vectors")
    synth(BUILD / "gl_nn_generic.v", "nn_engine", [RTL / "nn_engine.v"])
    sh(["iverilog", "-g2012", "-DNETLIST", "-o", "tb_nn_gl.vvp", HERE / "tb_nn_vectors.v", BUILD / "gl_nn_generic.v", *SIMCELLS])
    rng = np.random.RandomState(1); total = exact = 0
    for r in range(6):
        p = rng.randint(-128, 128, 212); X = rng.randint(0, 256, (400, 8))
        write_hex(BUILD / "weights.hex", p, 2); write_hex(BUILD / "vectors.hex", X.flatten(), 2)
        sh(["vvp", "tb_nn_gl.vvp", f"+nvec={len(X)}"])
        out = np.loadtxt(BUILD / "nn_out.txt", dtype=np.int64).reshape(len(X), -1)
        for n in range(len(X)):
            c, conf, _, _ = nn_infer(X[n], p); total += 1; exact += int(out[n, 0] == c and out[n, 1] == conf)
    report("G1.nn_netlist", exact == total, f"{exact}/{total} vectors: class and confidence exact on the synthesised network")
    write_hex(BUILD / "weights.hex", params, 2)

    print("G2  senseedge_top, generic gate netlist, through the bus")
    synth(BUILD / "gl_top_generic.v", "senseedge_top", RTL_FILES)
    check_bus("G2", run_bus("gl", [BUILD / "gl_top_generic.v", *SIMCELLS]), params, ref)

    print("G3  senseedge_top mapped to sky130_fd_sc_hd, PDK cell models")
    if LIB.exists():
        synth(BUILD / "gl_top_sky130.v", "senseedge_top", RTL_FILES, sky130=True)
        check_bus("G3", run_bus("sky", [BUILD / "gl_top_sky130.v", *PDK_V], defines=("FUNCTIONAL", "UNIT_DELAY=#1")), params, ref)
    else:
        print("  [SKIP] PDK liberty not found at", LIB)

    print("S   stress scenarios on RTL")
    run_stress("S.rtl", RTL_FILES, params, codes)
    print("S   stress scenarios on the generic netlist")
    run_stress("S.gl", [BUILD / "gl_top_generic.v", *SIMCELLS], params, codes)
    if LIB.exists():
        print("S   stress scenarios on the sky130 netlist")
        run_stress("S.sky", [BUILD / "gl_top_sky130.v", *PDK_V], params, codes, defines=("FUNCTIONAL", "UNIT_DELAY=#1"))

    (BUILD / "gl_results.json").write_text(json.dumps(results, indent=1))
    failed = [k for k, v in results.items() if not v["ok"]]
    print("\nRESULT:", "ALL PASS" if not failed else f"FAILED {failed}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as e:
        print("simulator/synthesis error:\n", (e.stderr or e.stdout)[-3000:])
        sys.exit(2)
