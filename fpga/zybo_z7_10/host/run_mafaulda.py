#!/usr/bin/env python3
"""MaFaulDa end to end (TEST_RUNS.md run 5): archives -> records -> weights -> board replay.

  run_mafaulda.py --zips <dir with the six class .zip> --root <extract dir> --port COM10 [--baud 921600]
                  [--variants d249,d70,tracked] [--seeds 16] [--skip-extract] [--skip-records]
                  [--skip-train] [--skip-replay] [--no-rtl] [--python <interpreter>]

  1 extract   the six archives into <root>/<class>/...            (skipped when present)
  2 records   prepare_records.py mafaulda for each variant         records/mafaulda_<v>.npz
  3 weights   ml/train_on_records.py, best of --seeds              ml/records/mafaulda_<v>_weights.npz
  4 replay    replay.py on the board, test split                   results/fpga_mafaulda_<v>.{csv,json,md}
Every step is logged to results/mafaulda_session.log by the caller; the first failure stops the run.
"""
import argparse
import subprocess
import sys
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
CLASSES = ["normal", "horizontal-misalignment", "vertical-misalignment", "imbalance", "underhang", "overhang"]
VARIANTS = {"d249": ["--div", "249"], "d70": ["--div", "70"], "tracked": ["--policy", "tracked"]}


def run(label, cmd, cwd=HERE, must_contain=None):
    print(f"\n=== {label}\n    {' '.join(str(c) for c in cmd)}", flush=True)
    p = subprocess.run([str(c) for c in cmd], cwd=cwd, text=True)
    ok = p.returncode == 0
    print(f"=== {label}: {'OK' if ok else 'FAILED'}", flush=True)
    if not ok:
        sys.exit(f"stopped at: {label}")


def extract(zips, root):
    root.mkdir(parents=True, exist_ok=True)
    for c in CLASSES:
        z = zips / f"{c}.zip"
        if not z.exists():
            sys.exit(f"missing archive {z}")
        if (root / c).is_dir() and any((root / c).rglob("*.csv")):
            print(f"  {c}: already extracted", flush=True); continue
        print(f"  extracting {z.name} ...", flush=True)
        with zipfile.ZipFile(z) as zf:
            names = zf.namelist()
            top = {n.split("/")[0] for n in names if "/" in n}
            # archives unpack as <class>/... ; guard against ones that unpack flat
            dest = root if top == {c} else root / c
            zf.extractall(dest)
        n = sum(1 for _ in (root / c).rglob("*.csv"))
        print(f"  {c}: {n} csv files", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zips", type=Path, required=True); ap.add_argument("--root", type=Path, required=True)
    ap.add_argument("--port"); ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--variants", default="d249,d70,tracked"); ap.add_argument("--seeds", type=int, default=16)
    ap.add_argument("--jobs", type=int, default=8); ap.add_argument("--no-rtl", action="store_true")
    for s in ("extract", "records", "train", "replay"):
        ap.add_argument(f"--skip-{s}", action="store_true")
    ap.add_argument("--python", default=sys.executable)
    a = ap.parse_args()
    py = [a.python, "-I"]
    variants = [v for v in a.variants.split(",") if v]
    (HERE / "results").mkdir(exist_ok=True); (HERE / "records").mkdir(exist_ok=True)

    if not a.skip_extract:
        print("=== 1 extract", flush=True); extract(a.zips, a.root)
    if not a.skip_records:
        for v in variants:
            run(f"2 records {v}", py + ["prepare_records.py", "mafaulda", "--root", a.root, "--out", f"records/mafaulda_{v}.npz",
                                        "--gain-target", "512", "--jobs", str(a.jobs)] + VARIANTS[v])
    if not a.skip_train:
        for v in variants:
            run(f"3 weights {v}", py + [REPO / "ml" / "train_on_records.py", "--records", HERE / "records" / f"mafaulda_{v}.npz",
                                        "--name", f"mafaulda_{v}", "--seeds", str(a.seeds), "--jobs", str(a.jobs)] + (["--no-rtl"] if a.no_rtl else []),
                cwd=REPO)
    if not a.skip_replay:
        if not a.port:
            sys.exit("--port is needed for the replay")
        for v in variants:
            run(f"4 replay {v}", py + ["replay.py", "--records", f"records/mafaulda_{v}.npz", "--weights", REPO / "ml" / "records" / f"mafaulda_{v}_weights.npz",
                                       "--split", "test", "--out", f"results/fpga_mafaulda_{v}", "--port", a.port, "--baud", str(a.baud)])


if __name__ == "__main__":
    main()
