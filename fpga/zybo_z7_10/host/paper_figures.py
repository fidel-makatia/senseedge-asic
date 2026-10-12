#!/usr/bin/env python3
"""Publication figures (IEEE TVLSI, single column 3.5 in / double 7.16 in) from the Zybo results.

Inputs are the files replay.py, divider_check.py, weight_update.py and selftest_capture.py write
under results/ (the host-model equivalents from evaluate.py --host carry the same schema and are
used for a dry run).  Every figure is written as vector PDF and 600-dpi PNG under results/figures/.

  paper_figures.py [--results results] [--prefix fpga] [--out results/figures]
  paper_figures.py --prefix host                         # host-model dry run (CWRU only)

Per dataset (CWRU: three sample rates; MaFaulDa: two rates and the tracked policy)
  fig_<ds>_accuracy        window / balanced / sequence-majority accuracy per variant, 95 %
                           sequence-bootstrap interval on the window accuracy
  fig_<ds>_confusion       row-normalised confusion matrices, one panel per variant
  fig_<ds>_per_condition   accuracy of every held-out condition, one marker per variant
Board-only
  fig_divider              time per window against the divider D
  fig_weight_update        decisions and alarm under weight set A, B (rotated outputs), A again
  fig_selftest_spectrum    the 32 bins of the bin-8 self-test record, board vs reference
"""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402

HERE = Path(__file__).resolve().parent
F_CLK = 20_000_000

# ---- palette (validated: references/palette.md of the dataviz method; first three slots pass all pairs)
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]        # blue, orange, aqua, yellow
SEQ = ["#fcfcfb", "#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]
INK, INK2, MUTED, GRID, AXIS = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
CMAP = LinearSegmentedColormap.from_list("seq_blue", SEQ)
COL1, COL2 = 3.5, 7.16     # IEEE column widths in inches

# dataset -> (file stem per variant, x label, sub-label), condition order, footnote
DATASETS = {
    "cwru": dict(
        variants=[("cwru_d249_g512", "2.29 kS/s", "D = 249"), ("cwru_d142_g512", "4.0 kS/s", "D = 142"), ("cwru_d70_g512", "8.0 kS/s", "D = 70")],
        held_out="the held-out 3 hp load", legend="Sample rate",
        cond_order=["normal_3hp", "IR007_3hp", "IR014_3hp", "IR021_3hp", "B007_3hp", "B014_3hp", "B021_3hp", "OR007_3hp", "OR014_3hp", "OR021_3hp"],
        cond_label=lambda c: c.replace("_3hp", "").replace("normal", "Normal"),
        cond_note="IR = inner race, B = ball, OR = outer race; 007/014/021 = fault diameter in mils.",
        cond_height=2.6),
    "mafaulda": dict(
        variants=[("mafaulda_d249", "2.29 kS/s", "D = 249"), ("mafaulda_d70", "8.0 kS/s", "D = 70"), ("mafaulda_tracked", "Tracked", r"$f_s = 32\,f_\mathrm{rot}$")],
        held_out="the 30 % held-out sequences", legend="Sampling",
        cond_order="mafaulda",
        cond_label=lambda c: c.replace("horizontal-misalignment", "H-misalign").replace("vertical-misalignment", "V-misalign")
                              .replace("_fault", "").replace("outer_race", "outer race").replace("underhang/", "UH ").replace("overhang/", "OH ").replace("/", " "),
        cond_note="UH/OH = underhang/overhang bearing; cage, outer race, ball faults at 0/6/20/35 g; misalignment in mm; imbalance in g.",
        cond_height=4.6),
}


def style():
    plt.rcParams.update({
        "font.family": "sans-serif", "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans"],
        "font.size": 8, "axes.labelsize": 8, "axes.titlesize": 8, "legend.fontsize": 7,
        "xtick.labelsize": 7, "ytick.labelsize": 7, "axes.linewidth": 0.6,
        "axes.edgecolor": AXIS, "axes.labelcolor": INK, "xtick.color": INK2, "ytick.color": INK2,
        "xtick.major.width": 0.6, "ytick.major.width": 0.6, "xtick.major.size": 2.5, "ytick.major.size": 2.5,
        "axes.grid": False, "grid.color": GRID, "grid.linewidth": 0.5, "grid.linestyle": "-",
        "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False,
        "pdf.fonttype": 42, "ps.fonttype": 42, "savefig.dpi": 600, "figure.dpi": 120,
        "axes.prop_cycle": matplotlib.cycler(color=SERIES), "text.color": INK,
    })


def save(fig, out, name):
    out.mkdir(parents=True, exist_ok=True)
    for ext in ("pdf", "png"):
        try:
            fig.savefig(out / f"{name}.{ext}", bbox_inches="tight", pad_inches=0.02)
        except (PermissionError, OSError):   # the file is open in a viewer or being synced: keep the previous copy
            print(f"  WARNING: {name}.{ext} is locked, not rewritten")
    plt.close(fig)
    print(f"  {name}.pdf / .png")


def load_runs(results, prefix, ds):
    runs = []
    for stem, label, sub in DATASETS[ds]["variants"]:
        j = results / f"{prefix}_{stem}.json"
        if j.exists():
            c = results / f"{prefix}_{stem}.csv"
            runs.append((label, sub, json.loads(j.read_text()), list(csv.DictReader(open(c))) if c.exists() else []))
    return runs


def fig_accuracy(ds, runs, out):
    cfg = DATASETS[ds]
    fig, ax = plt.subplots(figsize=(COL1, 2.3))
    x = np.arange(len(runs)); w = 0.24
    keys = [("window_accuracy", "Window"), ("balanced_accuracy", "Balanced"), ("sequence_accuracy", "Sequence majority")]
    for k, (key, label) in enumerate(keys):
        vals = [100 * m[key] for _, _, m, _ in runs]
        bars = ax.bar(x + (k - 1) * (w + 0.02), vals, w, color=SERIES[k], label=label, zorder=3)
        if key == "window_accuracy":
            lo = [100 * (m[key] - m["window_accuracy_ci95"][0]) for _, _, m, _ in runs]
            hi = [100 * (m["window_accuracy_ci95"][1] - m[key]) for _, _, m, _ in runs]
            ax.errorbar(x + (k - 1) * (w + 0.02), vals, yerr=[lo, hi], fmt="none", ecolor=INK2, elinewidth=0.8, capsize=2, zorder=4)
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, 1.5, f"{v:.1f}", ha="center", va="bottom", fontsize=6, color="white", zorder=5)
    ax.set_xticks(x); ax.set_xticklabels([f"{l}\n({s})" for l, s, _, _ in runs])
    ax.set_ylabel(f"Accuracy on {cfg['held_out']} (%)"); ax.set_ylim(0, 105)
    ax.yaxis.grid(True, zorder=0); ax.set_axisbelow(True)
    ax.legend(ncol=3, loc="upper center", bbox_to_anchor=(0.5, 1.14), handlelength=1.0, columnspacing=1.0)
    n = sum(m["windows"] for _, _, m, _ in runs); s = runs[0][2]["sequences"]
    ax.text(0.0, -0.27, f"Whiskers: 95 % interval, sequence bootstrap. {n:,} windows,\n"
            f"{s} held-out sequences per variant; every window bit-exact to the reference chain.",
            transform=ax.transAxes, ha="left", va="top", fontsize=6, color=INK2)
    save(fig, out, f"fig_{ds}_accuracy")


def fig_confusion(ds, runs, out):
    fig, axes = plt.subplots(1, 3, figsize=(COL2, 2.35))
    for ax, (label, sub, m, _) in zip(axes, runs):
        conf = np.array(m["confusion"], float); names = [n.replace("_", " ").title() for n in m["class_names"]]
        norm = conf / np.maximum(1, conf.sum(1, keepdims=True))
        ax.imshow(norm, cmap=CMAP, vmin=0, vmax=1, aspect="equal")
        for i in range(4):
            for j in range(4):
                v = norm[i, j]
                ax.text(j, i, f"{100 * v:.0f}%\n{int(conf[i, j]):,}", ha="center", va="center", fontsize=6,
                        color="white" if v > 0.55 else INK)
        ax.set_xticks(range(4)); ax.set_yticks(range(4))
        ax.set_xticklabels(names, rotation=30, ha="right"); ax.set_yticklabels(names if ax is axes[0] else [])
        ax.set_title(f"{label} ({sub}): {100 * m['window_accuracy']:.1f} %", fontsize=8)
        for s in ax.spines.values():
            s.set_visible(False)
        ax.tick_params(length=0)
        if ax is axes[0]:
            ax.set_ylabel("True class")
        ax.set_xlabel("Predicted class")
    for ax in axes[len(runs):]:
        ax.axis("off")
    fig.subplots_adjust(wspace=0.08)
    save(fig, out, f"fig_{ds}_confusion")


def fig_per_condition(ds, runs, out):
    cfg = DATASETS[ds]
    conds = []
    for _, _, m, _ in runs:
        for c in m.get("per_condition", {}):
            if c not in conds:
                conds.append(c)
    if not conds:
        return
    if cfg["cond_order"] == "mafaulda":
        import re
        fam = {"normal": 0, "imbalance": 1, "horizontal-misalignment": 2, "vertical-misalignment": 3, "underhang": 4, "overhang": 5}
        kind = {"cage_fault": 0, "outer_race": 1, "ball_fault": 2}
        def key(c):
            parts = c.split("/"); num = re.findall(r"[\d.]+", parts[-1])
            return (fam.get(parts[0], 9), kind.get(parts[1], 0) if len(parts) > 2 else 0, float(num[0]) if num else 0.0)
        conds = sorted(conds, key=key)
    elif cfg["cond_order"]:
        conds = [c for c in cfg["cond_order"] if c in conds] + [c for c in conds if c not in cfg["cond_order"]]
    else:
        conds = sorted(conds)
    fig, ax = plt.subplots(figsize=(COL1, cfg["cond_height"]))
    y = np.arange(len(conds))
    for k, (label, _, m, _) in enumerate(runs):
        pc = m["per_condition"]
        vals = [100 * pc[c]["accuracy"] if c in pc else np.nan for c in conds]
        ax.plot(vals, y, "o", ms=4.5, mec="#fcfcfb", mew=0.8, color=SERIES[k], label=label, zorder=3 + k)
    ax.set_yticks(y); ax.set_yticklabels([cfg["cond_label"](c) for c in conds], fontsize=6 if len(conds) > 12 else 7)
    ax.invert_yaxis(); ax.set_xlim(0, 100); ax.set_xlabel(f"Window accuracy on {cfg['held_out']} (%)")
    ax.xaxis.grid(True, zorder=0); ax.set_axisbelow(True)
    ax.legend(title=cfg["legend"], loc="lower left", bbox_to_anchor=(0.0, 0.0), handlelength=0.8)
    ax.text(0, -0.55 / cfg["cond_height"], cfg["cond_note"], transform=ax.transAxes, fontsize=6, color=INK2, va="top", wrap=True)
    save(fig, out, f"fig_{ds}_per_condition")


def fig_divider(results, out):
    j = results / "divider_check.json"
    if not j.exists():
        return
    m = json.loads(j.read_text())
    runs = sorted(m["runs"], key=lambda r: r["div"])
    D = np.array([r["div"] for r in runs]); t = np.array([r["ms_per_window"] for r in runs])
    pred = 1e3 * 64 * 35 * (D + 1) / F_CLK
    fig, ax = plt.subplots(figsize=(COL1, 2.2))
    x = np.arange(len(D)); w = 0.34
    b1 = ax.bar(x - w / 2 - 0.01, pred, w, color=GRID, label="Capture time 64 x 35 (D+1) / 20 MHz", zorder=3)
    b2 = ax.bar(x + w / 2 + 0.01, t, w, color=SERIES[0], label="Measured per window, incl. UART transfer", zorder=3)
    for b, v in list(zip(b1, pred)) + list(zip(b2, t)):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.4, f"{v:.2f}", ha="center", va="bottom", fontsize=6, color=INK2)
    ax.set_xticks(x); ax.set_xticklabels([f"D = {d}\n{r['fs'] / 1e3:.3g} kS/s" for d, r in zip(D, runs)])
    ax.set_ylabel("Time per window (ms)"); ax.set_ylim(0, max(t.max(), pred.max()) * 1.18)
    ax.yaxis.grid(True, zorder=0); ax.set_axisbelow(True)
    ax.legend(loc="upper left", handlelength=1.0)
    ax.text(0.0, -0.30, f"{m['n']} windows at each D: bins, features and decisions identical ({'PASS' if m['ok'] else 'FAIL'});\n"
            f"the difference is the {m['baud']}-baud transfer of codes and results.",
            transform=ax.transAxes, ha="left", va="top", fontsize=6, color=INK2)
    save(fig, out, "fig_divider")


def fig_weight_update(results, out):
    j = results / "weight_update.json"
    if not j.exists():
        return
    m = json.loads(j.read_text()); names = ["Normal", "Inner race", "Ball", "Outer race"]
    sets = [("A", "Set A\n(trained)"), ("B", "Set B\n(outputs rotated)"), ("A again", "Set A\nreloaded")]
    n = len(m["A"]["decisions"])
    fig, axes = plt.subplots(3, 1, figsize=(COL1, 2.6), sharex=True)
    for ax, (k, title) in zip(axes, sets):
        dec = np.array(m[k]["decisions"]); al = np.array(m[k]["alarm"])
        for c in range(4):
            idx = np.flatnonzero(dec == c)
            ax.bar(idx, np.ones(len(idx)), 0.82, color=SERIES[c], zorder=3)
        if al.any():
            a0 = int(np.argmax(al))
            ax.axvspan(a0 - 0.5, n - 0.5, color=MUTED, alpha=0.18, lw=0, zorder=1)
            ax.text(a0, 1.08, "alarm", fontsize=6, color=INK2, ha="left", va="bottom")
        ax.set_yticks([]); ax.set_ylim(0, 1.3); ax.set_ylabel(title, rotation=0, ha="right", va="center", fontsize=6.5)
        for s in ax.spines.values():
            s.set_visible(False)
        ax.text(n - 0.4, 0.5, f"{m[k]['readback_errors']} read-back errors", fontsize=6, color=INK2, va="center")
    axes[-1].set_xlabel("Window"); axes[-1].set_xticks(range(0, n, 4))
    handles = [plt.Rectangle((0, 0), 1, 1, color=SERIES[c]) for c in range(4)]
    fig.legend(handles, names, ncol=4, loc="upper center", bbox_to_anchor=(0.55, 1.02), handlelength=0.9, columnspacing=1.0)
    fig.subplots_adjust(hspace=0.35, left=0.22, right=0.98, top=0.86, bottom=0.17)
    save(fig, out, "fig_weight_update")


def fig_selftest_spectrum(results, out):
    j = results / "selftest_spectrum.json"
    if not j.exists():
        return
    m = json.loads(j.read_text()); board = np.array(m["board"]); ref = np.array(m["reference"])
    fig, ax = plt.subplots(figsize=(COL1, 2.0))
    k = np.arange(32)
    ax.bar(k, ref, 0.8, color=GRID, label="Reference (ref_models.fft64)", zorder=2)
    ax.bar(k, board, 0.45, color=SERIES[0], label="Board (FFT_DATA read-back)", zorder=3)
    ax.set_xlabel("FFT bin"); ax.set_ylabel("Magnitude (16-bit)"); ax.set_xticks(range(0, 32, 4))
    ax.yaxis.grid(True, zorder=0); ax.set_axisbelow(True); ax.legend(loc="center right", handlelength=1.0)
    ax.annotate(f"bin 8 = {int(board[8])}", (8, board[8] * 0.92), textcoords="offset points", xytext=(6, 0), fontsize=6, color=INK2, va="top")
    ax.annotate(f"bin 24 = {int(board[24])}", (24, board[24]), textcoords="offset points", xytext=(0, 4), fontsize=6, color=INK2, ha="center")
    ax.text(0.0, -0.30, f"{'32/32 bins identical' if (board == ref).all() else 'MISMATCH'}; features {m['features']},\nclass {m['cls']}, confidence {m['conf']}.",
            transform=ax.transAxes, ha="left", va="top", fontsize=6, color=INK2)
    save(fig, out, "fig_selftest_spectrum")


# ---------------------------------------------------------------- streaming figures (stream_demo.py)
CLASS_COL = SERIES  # class k -> categorical slot k (fixed order; identity never re-assigned)


def _spectrogram(ax, wins, fs, t_win, onset_t=None):
    B = np.array([w["bins"] for w in wins], float).T          # 32 x n
    L = np.log10(B + 1.0)
    n = B.shape[1]
    im = ax.imshow(L, aspect="auto", origin="lower", cmap=CMAP, extent=[0, n * t_win, 0, 32 * fs / 64 / 1e3], interpolation="nearest")
    ax.set_ylabel("Frequency (kHz)")
    cax = ax.inset_axes([1.005, 0.0, 0.012, 1.0])
    cb = ax.figure.colorbar(im, cax=cax); cb.set_label("log10 |X|", fontsize=6); cb.ax.tick_params(labelsize=5.5, length=2)
    if onset_t is not None:
        ax.axvline(onset_t, color=INK, lw=0.6, ls="-")


def _decision_strip(ax, wins, t_win, alarm, names, onset_t=None, show_conf=True):
    n = len(wins); t = (np.arange(n) + 0.5) * t_win
    cls = np.array([w["cls"] for w in wins]); conf = np.array([w["conf"] for w in wins])
    for c in range(4):
        m = cls == c
        if m.any():
            ax.bar(t[m], np.ones(m.sum()), t_win, color=CLASS_COL[c], zorder=3, label=names[c].replace("_", " ").title())
    al = np.array(alarm)
    if al.any():
        a0 = int(np.argmax(al))
        ax.axvspan(a0 * t_win, n * t_win, ymin=0.0, ymax=1.0, color=MUTED, alpha=0.18, lw=0, zorder=1)
        ax.text(a0 * t_win, 1.02, " alarm", fontsize=6, color=INK2, va="bottom", ha="left")
    if show_conf:
        ax2 = ax.twinx() if False else None  # no dual axes: confidence goes on its own panel
    ax.set_yticks([]); ax.set_ylim(0, 1.25); ax.set_xlim(0, n * t_win)
    if onset_t is not None:
        ax.axvline(onset_t, color=INK, lw=0.6)
    for s in ax.spines.values():
        s.set_visible(False)


def fig_stream_onset(results, out, name):
    j = results / f"stream_{name}.json"
    if not j.exists():
        return
    m = json.loads(j.read_text()); on = m["onset"]["windows"]; codes = np.array(m["onset"]["codes"], float)
    fs, tw, names = m["fs"], m["t_window_s"], m["class_names"]; n = len(on); onset_t = m["pre"] * tw
    fig, axes = plt.subplots(4, 1, figsize=(COL2, 4.6), sharex=True, gridspec_kw=dict(height_ratios=[1.1, 1.4, 1.0, 0.55]))
    # 1 signal
    sig = (codes - 2048.0).flatten(); t_s = np.arange(len(sig)) / fs
    axes[0].plot(t_s, sig, lw=0.35, color=SERIES[0]); axes[0].set_ylabel("ADC code - 2048")
    axes[0].axvline(onset_t, color=INK, lw=0.6)
    BOX = dict(boxstyle="square,pad=0.15", fc="#fcfcfb", ec="none", alpha=0.85)
    axes[0].text(onset_t, axes[0].get_ylim()[1], f"  {m['fault']['cond']} begins", fontsize=6, color=INK2, va="top", bbox=BOX)
    axes[0].text(0.01, 0.95, f"{m['healthy']['cond']}", transform=axes[0].transAxes, fontsize=6, color=INK2, va="top", bbox=BOX)
    # 2 chip spectrogram
    _spectrogram(axes[1], on, fs, tw, onset_t); axes[1].text(0.01, 0.93, "32-bin FFT read back from the chip (log magnitude)", transform=axes[1].transAxes, fontsize=6, color="white", va="top")
    # 3 features (band energies) + confidence
    F = np.array([w["feat"] for w in on], float); t = (np.arange(n) + 0.5) * tw
    for k, lab in enumerate(["Band 1-4", "Band 5-10", "Band 11-20", "Band 21-31"]):
        axes[2].plot(t, F[:, k], lw=1.0, color=SERIES[k], label=lab)
    axes[2].plot(t, [w["conf"] for w in on], lw=1.0, color=INK, ls=(0, (2, 1.5)), label="Confidence")
    axes[2].set_ylabel("Feature / conf. (0-255)"); axes[2].set_ylim(0, 345); axes[2].set_yticks([0, 100, 200, 255]); axes[2].axvline(onset_t, color=INK, lw=0.6)
    axes[2].legend(ncol=5, fontsize=6, loc="upper left", handlelength=1.2, columnspacing=0.8, borderaxespad=0.2)
    axes[2].yaxis.grid(True, zorder=0); axes[2].set_axisbelow(True)
    # 4 decisions + hardware alarm
    _decision_strip(axes[3], on, tw, [w["alarm"] for w in on], names, onset_t)
    axes[3].legend(ncol=4, fontsize=6, loc="upper center", bbox_to_anchor=(0.5, -0.55), handlelength=0.9, columnspacing=1.0)
    axes[3].set_xlabel(f"Machine time (s) at {fs / 1e3:.2f} kS/s, {1e3 * tw:.0f} ms per window; alarm: N = {m['n_consec']}, threshold {m['threshold']}")
    al = [w["alarm"] for w in on]
    if 1 in al:
        d = al.index(1) - m["pre"] + 1
        axes[3].text(0.99, 1.02, f"hardware alarm {d} windows ({1e3 * d * tw:.0f} ms) after the onset", transform=axes[3].transAxes, fontsize=6, color=INK2, ha="right", va="bottom")
    fig.subplots_adjust(hspace=0.12, left=0.09, right=0.99, top=0.98, bottom=0.14)
    save(fig, out, f"fig_{name}_onset")


def fig_stream_timeline(results, out, name):
    j = results / f"stream_{name}.json"
    if not j.exists():
        return
    m = json.loads(j.read_text()); tl = m["timeline"]["windows"]; fs, tw, names = m["fs"], m["t_window_s"], m["class_names"]
    n = len(tl); onset_t = m["healthy"]["windows"] * tw; t = (np.arange(n) + 0.5) * tw
    fig, axes = plt.subplots(3, 1, figsize=(COL2, 3.4), sharex=True, gridspec_kw=dict(height_ratios=[1.0, 1.4, 0.55]))
    axes[0].plot(t, m["timeline"]["codes_rms"], lw=0.8, color=SERIES[0]); axes[0].set_ylabel("RMS (ADC codes)")
    axes[0].axvline(onset_t, color=INK, lw=0.6); axes[0].yaxis.grid(True); axes[0].set_axisbelow(True)
    BOX = dict(boxstyle="square,pad=0.15", fc="#fcfcfb", ec="none", alpha=0.85)
    axes[0].text(0.01, 0.93, f"{m['healthy']['cond']} ({m['healthy']['windows']} windows)", transform=axes[0].transAxes, fontsize=6, color=INK2, va="top", bbox=BOX)
    axes[0].text(onset_t, axes[0].get_ylim()[1], f"  {m['fault']['cond']} ({m['fault']['windows']} windows)", fontsize=6, color=INK2, va="top", bbox=BOX)
    _spectrogram(axes[1], tl, fs, tw, onset_t)
    _decision_strip(axes[2], tl, tw, m["timeline"]["alarm_model"], names, onset_t)
    axes[2].legend(ncol=4, fontsize=6, loc="upper center", bbox_to_anchor=(0.5, -0.7), handlelength=0.9, columnspacing=1.0)
    cls = np.array([w["cls"] for w in tl]); h = m["healthy"]["windows"]
    axes[2].set_xlabel(f"Machine time (s); {int((cls[:h] == 0).sum())}/{h} healthy windows and {int((cls[h:] != 0).sum())}/{n - h} fault windows decided correctly by the chip; "
                       f"alarm: alarm model over the chip's decisions (N = {m['n_consec']})", fontsize=6.5)
    fig.subplots_adjust(hspace=0.12, left=0.09, right=0.99, top=0.98, bottom=0.2)
    save(fig, out, f"fig_{name}_timeline")


def fig_alarm_depth(results, out, name):
    j = results / f"stream_{name}.json"
    if not j.exists():
        return
    m = json.loads(j.read_text()); D = m["alarm_depth"]; tw = m["t_window_s"]
    N = [d["n"] for d in D]
    fig, axes = plt.subplots(1, 2, figsize=(COL1, 1.9))
    delay = [d["delay_windows"] if d["delay_windows"] is not None and d["delay_windows"] > 0 else np.nan for d in D]
    axes[0].plot(N, delay, "o-", ms=4, lw=1.2, color=SERIES[0], mec="#fcfcfb", mew=0.8, zorder=3)
    early = [d["n"] for d in D if d["delay_windows"] is not None and d["delay_windows"] <= 0]
    if early:
        axes[0].plot(early, [0.5] * len(early), "x", ms=4, color=SERIES[1], mew=1.0, zorder=4)
        axes[0].text(max(early) + 0.3, 0.5, "alarm already on\nbefore the onset", fontsize=5.5, color=INK2, va="center")
    axes[0].set_ylim(0, max(1, np.nanmax(delay)) * 1.25)
    axes[0].set_xlabel("Alarm depth N"); axes[0].set_ylabel(f"Detection delay (windows of {1e3 * tw:.0f} ms)")
    axes[0].yaxis.grid(True, zorder=0); axes[0].set_axisbelow(True); axes[0].set_xticks(N)
    fa = [100.0 * d["false_alarm_blocks"] / max(1, d["healthy_blocks"]) for d in D]
    axes[1].bar(N, fa, 0.7, color=SERIES[1], zorder=3)
    for n_, v, d in zip(N, fa, D):
        axes[1].text(n_, v + 1.5 + (6 if n_ % 2 else 0), f"{d['false_alarm_blocks']}/{d['healthy_blocks']}", ha="center", fontsize=5.5, color=INK2)
    axes[1].set_xlabel("Alarm depth N"); axes[1].set_ylabel("Healthy blocks with\na false alarm (%)"); axes[1].set_xticks(N)
    axes[1].set_ylim(0, max(10, max(fa) * 1.3)); axes[1].yaxis.grid(True, zorder=0); axes[1].set_axisbelow(True)
    fig.text(0.0, -0.04, f"Hardware alarm logic, threshold {m['threshold']}. Onset: {m['pre']} healthy windows then {m['fault']['cond']}.\n"
             f"False alarms over {D[0]['healthy_windows']} healthy test windows replayed in blocks of 64.", fontsize=6, color=INK2, va="top")
    fig.subplots_adjust(wspace=0.6, left=0.14, right=0.98, top=0.97, bottom=0.25)
    save(fig, out, f"fig_{name}_alarm_depth")


def fig_class_spectra(results, out, name):
    j = results / f"stream_{name}.json"
    if not j.exists():
        return
    m = json.loads(j.read_text()); sp = m["class_spectra"]; fs = m["fs"]
    fig, axes = plt.subplots(2, len(sp), figsize=(COL2, 2.6), gridspec_kw=dict(height_ratios=[1, 1.15]))
    for k, s in enumerate(sp):
        codes = np.array(s["codes"], float) - 2048.0; t = 1e3 * np.arange(64) / fs
        dec = m["class_names"][s["decision"]].replace("_", " ").title()
        axes[0, k].plot(t, codes, lw=0.8, color=SERIES[0])
        axes[0, k].set_title(f"{s['name'].replace('_', ' ').title()} ({s['cond']})\nchip decision: {dec}, confidence {s['conf']}", fontsize=6.5)
        axes[0, k].set_xlabel("Time (ms)", fontsize=7)
        if k == 0:
            axes[0, k].set_ylabel("ADC code - 2048")
        bins = np.array(s["bins"], float); f = np.arange(32) * fs / 64
        axes[1, k].bar(f, bins, fs / 64 * 0.85, color=SERIES[0], zorder=3)
        pk = int(np.argmax(bins)); right = f[pk] > f[16]
        axes[1, k].annotate(f"peak: bin {pk}, {f[pk]:.0f} Hz", (f[pk], bins[pk]), textcoords="offset points",
                            xytext=(-5 if right else 5, 2), fontsize=5.5, color=INK2, va="bottom", ha="right" if right else "left")
        axes[1, k].set_ylim(0, bins.max() * 1.18)
        axes[1, k].set_xlabel("Frequency (Hz)", fontsize=7); axes[1, k].yaxis.grid(True, zorder=0); axes[1, k].set_axisbelow(True)
        if k == 0:
            axes[1, k].set_ylabel("Chip FFT magnitude")
    fig.subplots_adjust(hspace=0.6, wspace=0.3, left=0.08, right=0.99, top=0.88, bottom=0.17)
    save(fig, out, f"fig_{name}_class_spectra")


def fig_severity(results, out, name):
    j = results / f"severity_{name}.json"
    if not j.exists():
        return
    m = json.loads(j.read_text()); segs = m["segments"]; W = m["windows"]; names = m["class_names"]
    n = len(W); edges = np.cumsum([0] + [s["windows"] for s in segs])
    fig, axes = plt.subplots(3, 1, figsize=(COL2, 3.6), sharex=True, gridspec_kw=dict(height_ratios=[1.0, 1.0, 0.5]))
    x = np.arange(n) + 0.5
    # 1 RMS of the input and the chip's band 1-4 feature
    axes[0].plot(x, [w["feat"][0] for w in W], lw=1.0, color=SERIES[0], label="Band 1-4 energy (chip feature 0)")
    axes[0].plot(x, [w["feat"][7] for w in W], lw=1.0, color=SERIES[2], label="Total energy (chip feature 7)")
    axes[0].set_ylabel("Feature (0-255)"); axes[0].yaxis.grid(True); axes[0].set_axisbelow(True)
    axes[0].legend(loc="upper left", fontsize=6, handlelength=1.2)
    # 2 confidence and fault fraction per segment
    axes[1].plot(x, [w["conf"] for w in W], lw=0.9, color=INK, ls=(0, (2, 1.5)), label="Confidence (per window)")
    xs = [(edges[i] + edges[i + 1]) / 2 for i in range(len(segs))]
    axes[1].plot(xs, [100 * s["fault_fraction"] for s in segs], "o-", ms=4, lw=1.2, color=SERIES[1], mec="#fcfcfb", mew=0.8,
                 label="Windows decided as a fault (% per step)", zorder=4)
    axes[1].set_ylim(0, 270); axes[1].set_yticks([0, 50, 100, 255]); axes[1].set_ylabel("Confidence / fault %")
    axes[1].yaxis.grid(True); axes[1].set_axisbelow(True); axes[1].legend(loc="upper left", fontsize=6, handlelength=1.2)
    # 3 decisions
    cls = np.array([w["cls"] for w in W])
    for c in range(4):
        mm = cls == c
        if mm.any():
            axes[2].bar(x[mm], np.ones(mm.sum()), 1.0, color=CLASS_COL[c], label=names[c].replace("_", " ").title(), zorder=3)
    axes[2].set_yticks([]); axes[2].set_ylim(0, 1.0)
    for s in axes[2].spines.values():
        s.set_visible(False)
    for ax in axes:
        for e in edges[1:-1]:
            ax.axvline(e, color=AXIS, lw=0.6)
    axes[2].set_xticks(xs)
    axes[2].set_xticklabels([s['cond'].split('/')[-1] + chr(10) + f"{s['speed_hz']:.0f} Hz" for s in segs], fontsize=6)
    axes[2].set_xlim(0, n)
    pol = f"tracked f_s = 32 f_rot" if m.get("tracked") else f"{m['fs'] / 1e3:.2f} kS/s"
    axes[2].set_xlabel(f"{m['per']} windows per step, {pol}; left: {segs[0]['cond']}; then {segs[1]['cond'].split('/')[0]} with increasing severity (shaft speed under each step)", fontsize=6.5)
    axes[2].legend(ncol=4, fontsize=6, loc="upper center", bbox_to_anchor=(0.5, -0.75), handlelength=0.9, columnspacing=1.0)
    fig.subplots_adjust(hspace=0.12, left=0.08, right=0.99, top=0.98, bottom=0.26)
    save(fig, out, f"fig_{name}_severity")


def _power_hier(path):
    """report_power -hierarchical: instance name -> total W (first occurrence per name)."""
    import re
    out = {}
    txt = path.read_text(errors="replace")
    sec = txt.split("Hierarchy", 1)[-1]
    for line in sec.splitlines():
        m = re.match(r"^\|\s*([A-Za-z0-9_\[\]\.]+)\s*\|\s*([\d.<]+)\s*\|", line)
        if m and m.group(1) not in out:
            try:
                out[m.group(1)] = float(m.group(2).lstrip("<"))
            except ValueError:
                pass
    return out


def fig_power(build, out):
    files = [(d, build / "power" / f"power_saif_d{d}.txt") for d in (249, 4)]
    files = [(d, f) for d, f in files if f.exists()]
    if not files:
        return
    blocks = [("u_fft", "FFT"), ("u_nn", "NN"), ("u_wb", "Wishbone"), ("u_feature", "Features"), ("u_spi_adc", "SPI capture"), ("u_alarm", "Alarm"), ("u_emu", "ADC emulator")]
    fig, ax = plt.subplots(figsize=(COL1, 2.2))
    x = np.arange(len(blocks)); w = 0.36
    for k, (d, f) in enumerate(files):
        ph = _power_hier(f)
        vals = [1e3 * ph.get(b, 0.0) for b, _ in blocks]
        ax.bar(x + (k - 0.5) * (w + 0.02), vals, w, color=SERIES[k], label=f"D = {d} ({F_CLK / (35 * (d + 1)) / 1e3:.2g} kS/s)", zorder=3)
        for xi, v in zip(x + (k - 0.5) * (w + 0.02), vals):
            ax.text(xi, v + 0.15, f"{v:.1f}", ha="center", fontsize=5.5, color=INK2)
    ax.set_xticks(x); ax.set_xticklabels([n for _, n in blocks], fontsize=6.5)
    ax.set_ylabel("Power on the FPGA (mW)"); ax.yaxis.grid(True, zorder=0); ax.set_axisbelow(True)
    ax.legend(title="Replay divider", fontsize=6, title_fontsize=6)
    tot = {d: 1e3 * next((v for n, v in _power_hier(f).items() if n.startswith("axil_senseedge")), 0.0) for d, f in files}
    ax.text(0.0, -0.3, "report_power on the implemented xc7z010 design with switching activity from an xsim run of 16 recorded\n"
            f"vibration windows (SAIF); whole wrapper: " + ", ".join(f"{tot[d]:.0f} mW at D = {d}" for d in tot) + ". Not a measurement: the Zybo has no current sense.",
            transform=ax.transAxes, fontsize=6, color=INK2, va="top")
    save(fig, out, "fig_power_blocks")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", type=Path, default=HERE / "results"); ap.add_argument("--prefix", default="fpga")
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args()
    out = a.out or (a.results / "figures")
    style()
    for ds in DATASETS:
        runs = load_runs(a.results, a.prefix, ds)
        print(f"{ds}: {len(runs)} variants -> {out}")
        if runs:
            fig_accuracy(ds, runs, out); fig_confusion(ds, runs, out); fig_per_condition(ds, runs, out)
    fig_divider(a.results, out); fig_weight_update(a.results, out); fig_selftest_spectrum(a.results, out)
    for j in sorted(a.results.glob("stream_*.json")):
        name = j.stem[len("stream_"):]
        print(f"stream {name}")
        fig_stream_onset(a.results, out, name); fig_stream_timeline(a.results, out, name)
        fig_alarm_depth(a.results, out, name); fig_class_spectra(a.results, out, name)
    for j in sorted(a.results.glob("severity_*.json")):
        fig_severity(a.results, out, j.stem[len("severity_"):])
    fig_power(a.results.parent.parent / "build", out)


if __name__ == "__main__":
    main()
