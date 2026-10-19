#!/usr/bin/env python3
"""LaTeX tables (booktabs) from the Zybo results, for the TVLSI manuscript.

  paper_tables.py [--results results] [--build ../build] [--prefix fpga_cwru] [--out results/tables]

  tab_bitexact.tex     windows, blocks, bins checked and mismatches per sample rate (Table VI row)
  tab_accuracy.tex     window / balanced / sequence accuracy with 95 % intervals and per-class recall
  tab_fpga.tex         utilization and timing of the Zybo implementation (Table VII FPGA column)
  tab_divider.tex      time per window against D
"""
import argparse
import json
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
DATASETS = {"cwru": [("cwru_d249_g512", "2.29", "249"), ("cwru_d142_g512", "4.0", "142"), ("cwru_d70_g512", "8.0", "70")],
            "mafaulda": [("mafaulda_d249", "2.29", "249"), ("mafaulda_d70", "8.0", "70"), ("mafaulda_tracked", "tracked", "per seq.")]}


def pct(v):
    return f"{100 * v:.1f}"


def load(results, prefix, ds):
    out = []
    for stem, fs, d in DATASETS[ds]:
        j = results / f"{prefix}_{stem}.json"
        if j.exists():
            out.append((fs, d, json.loads(j.read_text())))
    return out


def tab_bitexact(runs):
    L = [r"\begin{tabular}{@{}lrrrrrrr@{}}", r"\toprule",
         r"$f_s$ (kS/s) & $D$ & Windows & Blocks & Features & Class, conf. & Bins (windows) & Alarm \\", r"\midrule"]
    for fs, d, m in runs:
        n = m["windows"]
        L.append(f"{fs} & {d} & {n:,} & {m['blocks']} & {n - m['mismatch_features']:,}/{n:,} & {n - m['mismatch_class_conf']:,}/{n:,} & "
                 f"{m['bins_checked'] - m['mismatch_bins']:,}/{m['bins_checked']:,} & {n - m['mismatch_alarm']:,}/{n:,} \\\\")
    L += [r"\bottomrule", r"\end{tabular}"]
    return "\n".join(L) + "\n"


def tab_accuracy(runs):
    names = runs[0][2]["class_names"]
    L = [r"\begin{tabular}{@{}lrrrrrrrr@{}}", r"\toprule"]
    hdr = r"$f_s$ (kS/s) & Window (\%) & 95\,\% CI & Balanced (\%) & Seq. majority (\%)"
    hdr += "".join(f" & {n.replace('_', ' ').title()}" for n in names) + r" \\"
    L += [hdr, r"\midrule"]
    for fs, d, m in runs:
        L.append(f"{fs} & {pct(m['window_accuracy'])} & {pct(m['window_accuracy_ci95'][0])}--{pct(m['window_accuracy_ci95'][1])} & "
                 f"{pct(m['balanced_accuracy'])} & {pct(m['sequence_accuracy'])}" + "".join(f" & {pct(r)}" for r in m["recall"]) + r" \\")
    L += [r"\bottomrule", r"\end{tabular}", "% last four columns: per-class recall on the held-out split"]
    return "\n".join(L) + "\n"


def parse_utilization(path):
    rows = {}
    for line in path.read_text(errors="replace").splitlines():
        # | Site Type | Used | Fixed | Prohibited | Available | Util% |
        m = re.match(r"^\|\s*([A-Za-z0-9 /_*().-]+?)\s*\|\s*([\d.]+)\s*\|\s*\d+\s*\|\s*\d+\s*\|\s*(\d+)\s*\|\s*([\d.<]+)\s*\|", line)
        if m:
            name = m.group(1).strip()
            if name not in rows:
                used = float(m.group(2)); used = int(used) if used.is_integer() else used
                rows[name] = (used, int(m.group(3)), m.group(4))
    return rows


def parse_timing(path):
    txt = path.read_text(errors="replace")
    m = re.search(r"WNS\(ns\)\s+TNS\(ns\).*?\n.*?\n\s*([-\d.]+)\s+([-\d.]+)\s+(\d+)\s+(\d+)\s+([-\d.]+)\s+([-\d.]+)", txt, re.S)
    return dict(wns=float(m.group(1)), tns=float(m.group(2)), whs=float(m.group(5)), ths=float(m.group(6))) if m else {}


def parse_hier(path):
    """Per-instance rows of report_utilization -hierarchical: name -> (LUT, FF, BRAM, DSP)."""
    rows = {}
    for line in path.read_text(errors="replace").splitlines():
        c = [x.strip() for x in line.split("|")]
        if len(c) > 11 and c[1] and re.match(r"^\d+$", c[3]):
            rows.setdefault(c[1], (int(c[3]), int(c[7]), float(c[8]), int(c[10])))
    return rows


def parse_power(path):
    txt = path.read_text(errors="replace")
    out = {}
    for key, pat in (("total", r"Total On-Chip Power \(W\)\s*\|\s*([\d.]+)"), ("ps7", r"\|\s*PS7\s*\|\s*([\d.]+)"),
                     ("pl", r"\|\s*axil_senseedge_0\s*\|\s*([\d.]+)")):
        m = re.search(pat, txt)
        if m:
            out[key] = float(m.group(1))
    return out


def tab_fpga(build):
    u = parse_utilization(build / "utilization.txt") if (build / "utilization.txt").exists() else {}
    t = parse_timing(build / "timing.txt") if (build / "timing.txt").exists() else {}
    h = parse_hier(build / "utilization_hier.txt") if (build / "utilization_hier.txt").exists() else {}
    p = parse_power(build / "power.txt") if (build / "power.txt").exists() else {}
    want = [("Slice LUTs", "LUT"), ("Slice Registers", "Flip-flops"), ("Block RAM Tile", "BRAM (36 Kb tiles)"), ("DSPs", "DSP48E1")]
    L = [r"\begin{tabular}{@{}lrrr@{}}", r"\toprule", r"Resource (whole design) & Used & Available & Util. (\%) \\", r"\midrule"]
    for key, label in want:
        k = next((n for n in u if n.startswith(key)), None)
        if k:
            L.append(f"{label} & {u[k][0]:,} & {u[k][1]:,} & {u[k][2]} \\\\")
    if t:
        L += [r"\midrule", f"Setup slack at 20 MHz (WNS) & \\multicolumn{{3}}{{r}}{{{t['wns']:+.3f} ns}} \\\\",
              f"Hold slack (WHS) & \\multicolumn{{3}}{{r}}{{{t['whs']:+.3f} ns}} \\\\"]
    if p:
        L += [f"Power estimate, PL (SenseEdge wrapper) & \\multicolumn{{3}}{{r}}{{{1e3 * p.get('pl', 0):.0f} mW}} \\\\",
              f"Power estimate, PS7 & \\multicolumn{{3}}{{r}}{{{p.get('ps7', 0):.2f} W}} \\\\"]
    L += [r"\bottomrule", r"\end{tabular}", "% xc7z010clg400-1, whole design incl. Zynq PS wrapper and AXI interconnect; power from report_power (vectorless)"]
    if h:
        blocks = [("u_se", "senseedge\\_top (whole macro)"), ("u_fft", "FFT engine"), ("u_nn", "NN engine"), ("u_wb", "Wishbone + registers"),
                  ("u_feature", "Feature extraction"), ("u_spi_adc", "SPI ADC capture"), ("u_alarm", "Alarm logic")]
        L += ["", r"\begin{tabular}{@{}lrrrr@{}}", r"\toprule", r"Block & LUT & FF & BRAM & DSP \\", r"\midrule"]
        for key, label in blocks:
            if key in h:
                lut, ff, bram, dsp = h[key]
                L.append(f"{label} & {lut:,} & {ff:,} & {bram:g} & {dsp} \\\\")
        L += [r"\bottomrule", r"\end{tabular}", "% report_utilization -hierarchical on the implemented design"]
    return "\n".join(L) + "\n"


def tab_divider(results):
    j = results / "divider_check.json"
    if not j.exists():
        return ""
    m = json.loads(j.read_text())
    L = [r"\begin{tabular}{@{}rrrrl@{}}", r"\toprule", r"$D$ & $f_s$ (S/s) & $64\cdot35(D{+}1)/f_{clk}$ (ms) & Measured per window (ms) & Result \\", r"\midrule"]
    for r in m["runs"]:
        L.append(f"{r['div']} & {r['fs']:.1f} & {r['ms_window_pred']:.2f} & {r['ms_per_window']:.2f} & {'identical' if r['ok'] else 'mismatch'} \\\\")
    L += [r"\bottomrule", r"\end{tabular}", f"% {m['n']} windows per D; measured time includes the UART transfer at {m['baud']} baud"]
    return "\n".join(L) + "\n"


def parse_power_hier(path):
    """report_power hierarchy section: instance name -> watts (first occurrence)."""
    out = {}
    if not path.exists():
        return out
    txt = path.read_text(errors="replace")
    sec = txt.split("Hierarchy", 1)[-1] if "Hierarchy" in txt else txt
    for line in sec.splitlines():
        m = re.match(r"^\|\s*([A-Za-z0-9_\[\]\.]+)\s*\|\s*([\d.<]+)\s*\|", line)
        if m and m.group(1) not in out:
            try:
                out[m.group(1)] = float(m.group(2).lstrip("<"))
            except ValueError:
                pass
    return out


def tab_metrics(results, build):
    """One table of the implementation and run-time metrics measured or estimated on the Zybo."""
    F = 20e6
    t = parse_timing(build / "timing.txt") if (build / "timing.txt").exists() else {}
    h = parse_hier(build / "utilization_hier.txt") if (build / "utilization_hier.txt").exists() else {}
    pv = parse_power(build / "power.txt") if (build / "power.txt").exists() else {}
    L = [r"\begin{tabular}{@{}llr@{}}", r"\toprule", r"Metric & Source & Value \\", r"\midrule"]
    if "u_se" in h:
        lut, ff, bram, dsp = h["u_se"]
        L.append(f"Macro resources (xc7z010) & implementation & {lut:,} LUT, {ff:,} FF, {dsp} DSP \\\\")
    if t:
        L.append(f"Setup / hold slack at 20 MHz & STA & {t['wns']:+.2f} / {t['whs']:+.3f} ns \\\\")
        L.append(r"$f_\mathrm{max}$ on the FPGA & STA, $1/(50\,\mathrm{ns} - \mathrm{WNS})$ & " + f"{1e3 / (50 - t['wns']):.1f}" + r" MHz \\")
    L.append(r"Decision latency, buffer full $\to$ class & RTL audit E, reproduced & 728 clocks = 36.4 $\mu$s \\")
    j = results / "throughput.json"
    if j.exists():
        m = json.loads(j.read_text())
        for r in m["rows"]:
            if r["div"] in (0, 249):
                L.append(f"Window period at $D={r['div']}$ & board, global timer & {1e6 * r['period_s']:.1f} " + r"$\mu$s "
                         f"({r['period_clocks']:.0f} clocks; predicted {2304 if r['div'] == 0 else 2240 * (r['div'] + 1)}) \\\\")
        r0 = m["rows"][0]
        L.append(f"Peak classification rate ($D=0$) & board & {r0['windows_per_s']:,.0f} per second \\\\")
    if pv:
        L.append(r"PL power, vectorless & report\_power & " + f"{1e3 * pv.get('pl', 0):.0f}" + r" mW \\")
    if pv and j.exists():
        for div in (0, 249):
            rr = next((r for r in m["rows"] if r["div"] == div), None)
            if rr:
                e = pv.get("pl", 0) * rr["period_s"]
                L.append(f"Energy per classification, $D={div}$ (estimate) & 27 mW $" + r"\times$" + f" window period & {1e6 * e:.2f} " + r"$\mu$J \\")
    for div in (249, 4):
        ph = parse_power_hier(build / "power" / f"power_saif_d{div}.txt")
        k = next((n for n in ph if n.startswith("axil_senseedge")), None)
        if k:
            L.append(f"PL power, SAIF-annotated, $D={div}$ & xsim activity + " + r"report\_power & " + f"{1e3 * ph[k]:.0f}" + r" mW \\")
            if j.exists():
                rr = next((r for r in m["rows"] if r["div"] == div), None)
                if rr:
                    L.append(f"Energy per classification, $D={div}$ & power " + r"$	imes$ window period & " + f"{1e6 * ph[k] * rr['period_s']:.2f} " + r"$\mu$J \\")
    L += [r"\bottomrule", r"\end{tabular}", "% PL = the SenseEdge wrapper on the FPGA; the Zynq PS (1.4 W) is the test host, not the device"]
    return "\n".join(L) + "\n"


def tab_stream(results):
    runs = sorted(results.glob("stream_*.json"))
    if not runs:
        return ""
    L = [r"\begin{tabular}{@{}lrrrrrr@{}}", r"\toprule",
         r"Stream & $f_s$ (kS/s) & Healthy ok & Fault ok & Delay at $N{=}3$ & False-alarm blocks, $N{=}0/1/2$ & Bit-exact \\", r"\midrule"]
    for j in runs:
        m = json.loads(j.read_text()); tl = m["timeline"]["windows"]; h = m["healthy"]["windows"]
        cls = [w["cls"] for w in tl]
        ok_h = sum(1 for c in cls[:h] if c == 0); ok_f = sum(1 for c in cls[h:] if c != 0)
        d = {x["n"]: x for x in m["alarm_depth"]}
        d3 = d[3]["delay_windows"] if 3 in d else None
        fa = "/".join(f"{d[n]['false_alarm_blocks']}" for n in (0, 1, 2) if n in d) + f" of {d[0]['healthy_blocks']}"
        L.append(f"{m['healthy']['cond'].replace('_', ' ')} $\\to$ {m['fault']['cond'].replace('_', ' ')} & {m['fs'] / 1e3:.2f} & {ok_h}/{h} & {ok_f}/{len(tl) - h} & "
                 f"{d3} win. = {1e3 * d3 * m['t_window_s']:.0f} ms & {fa} & {m['checked'] - m['mismatches']}/{m['checked']} \\\\")
    L += [r"\bottomrule", r"\end{tabular}", "% stream_demo.py: hardware decisions and alarm, replayed at the deployment divider; threshold 100"]
    return "\n".join(L) + "\n"

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", type=Path, default=HERE / "results"); ap.add_argument("--build", type=Path, default=HERE.parent / "build")
    ap.add_argument("--prefix", default="fpga"); ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args()
    out = a.out or (a.results / "tables"); out.mkdir(parents=True, exist_ok=True)
    tables = {"tab_fpga.tex": tab_fpga(a.build), "tab_divider.tex": tab_divider(a.results), "tab_stream.tex": tab_stream(a.results),
              "tab_metrics.tex": tab_metrics(a.results, a.build)}
    for ds in DATASETS:
        runs = load(a.results, a.prefix, ds)
        if runs:
            tables[f"tab_{ds}_bitexact.tex"] = tab_bitexact(runs); tables[f"tab_{ds}_accuracy.tex"] = tab_accuracy(runs)
    for name, body in tables.items():
        if body:
            (out / name).write_text(body); print(f"  {name}"); print(body)


if __name__ == "__main__":
    main()
