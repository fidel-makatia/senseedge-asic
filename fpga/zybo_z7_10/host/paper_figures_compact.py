#!/usr/bin/env python3
"""Compact variants of three paper figures, for the TVLSI page budget.

  fig_<stream>_onset_short      the 4-panel onset figure at 3.9 in instead of 4.6 in
  fig_<stream>_onset_compact    codes, chip bins, decisions + alarm (no feature panel), 2.7 in
  fig_<name>_severity_compact   features with the fault share per step, decisions, 2.3 in

Same data, same style as paper_figures.py (imported from this directory).  The manuscript uses
fig_mafaulda_d249_imbalance_onset_short, fig_cwru_d70_onset_compact and
fig_mafaulda_imbalance_severity_compact, copied to paper/tvlsi/figs/fig_fpga_*.pdf.

    python3 paper_figures_compact.py [--results results] [--out results/figures]
"""
import argparse, json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import paper_figures as pf

HERE = Path(__file__).resolve().parent


def onset_short(results, out, name, height=3.9):
    """paper_figures.fig_stream_onset with a shorter figure; the function's figsize is fixed, so patch subplots."""
    orig = plt.subplots
    def subplots(*a, **k):
        if k.get("figsize") == (pf.COL2, 4.6):
            k["figsize"] = (pf.COL2, height)
        return orig(*a, **k)
    plt.subplots = subplots
    try:
        orig_save = pf.save
        pf.save = lambda fig, o, n: orig_save(fig, o, n.replace("_onset", "_onset_short"))
        pf.fig_stream_onset(results, out, name)
    finally:
        plt.subplots = orig; pf.save = orig_save


def onset_compact(results, out, name):
    j = results / f"stream_{name}.json"
    if not j.exists():
        return
    m = json.loads(j.read_text()); on = m["onset"]["windows"]; codes = np.array(m["onset"]["codes"], float)
    fs, tw, names = m["fs"], m["t_window_s"], m["class_names"]; onset_t = m["pre"] * tw
    fig, axes = plt.subplots(3, 1, figsize=(pf.COL2, 2.7), sharex=True, gridspec_kw=dict(height_ratios=[1.0, 1.4, 0.55]))
    sig = (codes - 2048.0).flatten(); t_s = np.arange(len(sig)) / fs
    axes[0].plot(t_s, sig, lw=0.35, color=pf.SERIES[0]); axes[0].set_ylabel("ADC code - 2048"); axes[0].axvline(onset_t, color=pf.INK, lw=0.6)
    BOX = dict(boxstyle="square,pad=0.15", fc="#fcfcfb", ec="none", alpha=0.85)
    axes[0].text(onset_t, axes[0].get_ylim()[1], f"  {m['fault']['cond']} begins", fontsize=6, color=pf.INK2, va="top", bbox=BOX)
    axes[0].text(0.01, 0.95, f"{m['healthy']['cond']}", transform=axes[0].transAxes, fontsize=6, color=pf.INK2, va="top", bbox=BOX)
    pf._spectrogram(axes[1], on, fs, tw, onset_t)
    axes[1].text(0.01, 0.93, "32-bin FFT read back from the chip (log magnitude)", transform=axes[1].transAxes, fontsize=6, color="white", va="top")
    pf._decision_strip(axes[2], on, tw, [w["alarm"] for w in on], names, onset_t)
    axes[2].legend(ncol=4, fontsize=6, loc="upper center", bbox_to_anchor=(0.5, -0.9), handlelength=0.9, columnspacing=1.0)
    axes[2].set_xlabel(f"Machine time (s) at {fs / 1e3:.2f} kS/s, {1e3 * tw:.0f} ms per window; alarm: N = {m['n_consec']}, threshold {m['threshold']}")
    al = [w["alarm"] for w in on]
    if 1 in al:
        d = al.index(1) - m["pre"] + 1
        axes[2].text(0.99, 1.02, f"hardware alarm {d} windows ({1e3 * d * tw:.0f} ms) after the onset", transform=axes[2].transAxes, fontsize=6, color=pf.INK2, ha="right", va="bottom")
    fig.subplots_adjust(hspace=0.12, left=0.09, right=0.99, top=0.98, bottom=0.26)
    pf.save(fig, out, f"fig_{name}_onset_compact")


def severity_compact(results, out, name):
    j = results / f"severity_{name}.json"
    if not j.exists():
        return
    m = json.loads(j.read_text()); segs = m["segments"]; W = m["windows"]; names = m["class_names"]
    n = len(W); edges = np.cumsum([0] + [s["windows"] for s in segs]); x = np.arange(n) + 0.5
    fig, axes = plt.subplots(2, 1, figsize=(pf.COL2, 2.3), sharex=True, gridspec_kw=dict(height_ratios=[1.0, 0.42]))
    axes[0].plot(x, [w["feat"][0] for w in W], lw=1.0, color=pf.SERIES[0], label="Band 1-4 energy (chip feature 0)")
    axes[0].plot(x, [w["feat"][7] for w in W], lw=1.0, color=pf.SERIES[2], label="Total energy (chip feature 7)")
    axes[0].set_ylabel("Feature (0-255)"); axes[0].set_ylim(0, 330); axes[0].set_yticks([0, 100, 200, 255]); axes[0].yaxis.grid(True); axes[0].set_axisbelow(True)
    axes[0].legend(loc="upper left", fontsize=6, handlelength=1.2, ncol=2, columnspacing=1.0, borderaxespad=0.1)
    xs = [(edges[i] + edges[i + 1]) / 2 for i in range(len(segs))]
    for i, s in enumerate(segs):
        axes[1].text(xs[i], 0.5, f"{100 * s['fault_fraction']:.0f} % fault", fontsize=5.5, color="white", ha="center", va="center", zorder=5)
    cls = np.array([w["cls"] for w in W])
    for c in range(4):
        mm = cls == c
        if mm.any():
            axes[1].bar(x[mm], np.ones(mm.sum()), 1.0, color=pf.CLASS_COL[c], label=names[c].replace("_", " ").title(), zorder=3)
    axes[1].set_yticks([]); axes[1].set_ylim(0, 1.0)
    for s in axes[1].spines.values():
        s.set_visible(False)
    for ax in axes:
        for e in edges[1:-1]:
            ax.axvline(e, color=pf.AXIS, lw=0.6)
    axes[1].set_xticks(xs); axes[1].set_xticklabels([s['cond'].split('/')[-1] + chr(10) + f"{s['speed_hz']:.0f} Hz" for s in segs], fontsize=6)
    axes[1].set_xlim(0, n)
    axes[1].set_xlabel(f"{m['per']} windows per step at {m['fs'] / 1e3:.2f} kS/s; left: {segs[0]['cond']}; then {segs[1]['cond'].split('/')[0]} with increasing mass (shaft speed under each step)", fontsize=6.5)
    axes[1].legend(ncol=4, fontsize=6, loc="upper center", bbox_to_anchor=(0.5, -1.05), handlelength=0.9, columnspacing=1.0)
    fig.subplots_adjust(hspace=0.1, left=0.08, right=0.99, top=0.97, bottom=0.33)
    pf.save(fig, out, f"fig_{name}_severity_compact")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", type=Path, default=HERE / "results"); ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args(); out = a.out or (a.results / "figures")
    pf.style()
    for j in sorted(a.results.glob("stream_*.json")):
        name = j.stem[len("stream_"):]
        onset_short(a.results, out, name); onset_compact(a.results, out, name); print(f"stream {name}: onset_short, onset_compact")
    for j in sorted(a.results.glob("severity_*.json")):
        name = j.stem[len("severity_"):]
        severity_compact(a.results, out, name); print(f"severity {name}: severity_compact")


if __name__ == "__main__":
    main()
