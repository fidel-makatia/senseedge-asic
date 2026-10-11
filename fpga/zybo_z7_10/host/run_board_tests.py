#!/usr/bin/env python3
"""Runs the whole Zybo test session of TEST_RUNS.md in order and writes every result under
results/ (JSON, CSV, MD), then the paper figures.  Stops at the first failed gate.

  run_board_tests.py --port COM10 [--baud 115200] [--skip-replay] [--python <interpreter>]

  1 text-mode UART: banner and self-test verdict           results/selftest_uart.txt
  2 self-test spectrum over the protocol (identify, weight round trip, record, run, read-back)
                                                           results/selftest_spectrum.json
  3 CWRU replay at D = 249, 142, 70 (test split)           results/fpga_cwru_d*_g512.{csv,json,md}
  4 divider independence                                   results/divider_check.json
  5 register-map weight update                             results/weight_update.json
  6 figures                                                results/figures/
(test_protocol_mock.py is not run here: its expected values are the mock chip's, not SenseEdge's.)
"""
import argparse
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ML = HERE.parents[2] / "ml" / "records"


def run(label, cmd, must_contain=None):
    print(f"\n=== {label}\n    {' '.join(cmd)}", flush=True)
    p = subprocess.run(cmd, cwd=HERE, text=True, capture_output=True)
    sys.stdout.write(p.stdout); sys.stderr.write(p.stderr)
    ok = p.returncode == 0 and (must_contain is None or must_contain in p.stdout)
    print(f"=== {label}: {'OK' if ok else 'FAILED'}", flush=True)
    if not ok:
        sys.exit(f"stopped at: {label}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True); ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--skip-replay", action="store_true"); ap.add_argument("--python", default=sys.executable)
    ap.add_argument("--uart-seconds", type=float, default=25.0)
    ap.add_argument("--skip-uart", action="store_true", help="the banner was already captured (results/selftest_uart.txt)")
    a = ap.parse_args()
    py = [a.python, "-I"]
    port = ["--port", a.port, "--baud", str(a.baud)]
    (HERE / "results").mkdir(exist_ok=True)

    if not a.skip_uart:
        run("1 text-mode UART (press the board's reset or power-cycle if nothing arrives)",
            py + ["uart_log.py", "--port", a.port, "--baud", str(a.baud), "--seconds", str(a.uart_seconds),
                  "--until", "SELF-TEST", "--lines", "3", "--out", "results/selftest_uart.txt"], must_contain="SELF-TEST PASSED")
    run("2 self-test spectrum", py + ["selftest_capture.py"] + port + ["--out", "results/selftest_spectrum.json"],
        must_contain="SELF-TEST SPECTRUM: PASS")
    if not a.skip_replay:
        for d in (249, 142, 70):
            run(f"3 CWRU replay D={d}", py + ["replay.py", "--records", f"records/cwru_d{d}_g512.npz",
                                              "--weights", str(ML / f"cwru_d{d}_g512_weights.npz"), "--split", "test",
                                              "--out", f"results/fpga_cwru_d{d}_g512"] + port)
    run("4 divider independence", py + ["divider_check.py", "--records", "records/cwru_d249_g512.npz",
                                        "--weights", str(ML / "cwru_d249_g512_weights.npz"), "--out", "results/divider_check.json"] + port,
        must_contain="DIVIDER CHECK: PASS")
    run("5 register-map weight update", py + ["weight_update.py", "--records", "records/cwru_d249_g512.npz",
                                              "--weights", str(ML / "cwru_d249_g512_weights.npz"), "--out", "results/weight_update.json"] + port,
        must_contain="RESULT: PASS")
    run("6 figures", py + ["paper_figures.py", "--prefix", "fpga"])


if __name__ == "__main__":
    main()
