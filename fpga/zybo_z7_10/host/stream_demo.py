#!/usr/bin/env python3
"""Stream recorded vibration through SenseEdge on the board and record what the chip does, window
by window, for the deployment figures of the paper:

  onset        one 64-window block: `--pre` healthy windows followed by fault windows, replayed at
               the deployment divider; per window the chip's 32 FFT bins, 8 features, class,
               confidence and the hardware alarm flag (N = --ncons)
  timeline     the whole healthy sequence followed by the whole fault sequence (several blocks);
               per window bins, features, class, confidence; the alarm over the continuous stream
               is the alarm model (alarm_logic.v) applied to the chip's decisions, because the
               replay protocol resets the chip between blocks
  alarm_depth  the onset block and every healthy block of the test split replayed with
               N = 0..--max-n in hardware: detection delay after the onset and false alarms
  class_spectra  one representative test window per class: codes and the chip's spectrum

Every bin, feature and decision is also checked against the reference chain (count reported).

  stream_demo.py --records records/cwru_d249_g512.npz --weights ../../../ml/records/cwru_d249_g512_weights.npz
                 --port COM10 --baud 921600 --healthy normal_3hp --fault IR007_3hp [--div 249]
                 [--pre 24] [--ncons 3] [--thr 100] [--max-n 7] --out results/stream_cwru_d249
Writes <out>.json.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[2] / "verilog" / "dv" / "audit"))
from evaluate import load_weights  # noqa: E402
from ref_models import alarm_model, chain  # noqa: E402
from se_link import F_CLK, add_link_args, open_link  # noqa: E402


def seq_of_cond(d, cond, split="test"):
    for s in np.unique(d["seq"]):
        if d["seq_cond"][s] == cond and (split == "any" or d["seq_split"][s] == split):
            return int(s)
    sys.exit(f"no {split} sequence with condition {cond}")


def run_blocks(link, codes, params, div, thr, ncons, bins=True, check=None):
    """Replay codes (n, 64) in blocks of 64; returns per-window dicts and the mismatch counter."""
    out = []
    for b0 in range(0, len(codes), 64):
        blk = codes[b0:b0 + 64]
        link.config(div, thr, ncons)
        link.load_record(blk)
        st, res = link.run(len(blk), feats=True, bins=bins, div=div)
        if st != 0 or len(res) != len(blk):
            sys.exit(f"run failed: status {st}, {len(res)} of {len(blk)} windows")
        for k, r in enumerate(res):
            if check is not None:
                m, f, c, cf = chain(blk[k].astype(np.int64), params)
                check["n"] += 1
                if (bins and r["bins"] != m) or r["feat"] != f or (r["cls"], r["conf"]) != (c, cf):
                    check["mismatch"] += 1
            out.append(dict(cls=int(r["cls"]), conf=int(r["conf"]), alarm=int(r["alarm"]),
                            feat=[int(v) for v in r["feat"]], bins=[int(v) for v in r["bins"]] if bins else None))
    return out


def main():
    ap = argparse.ArgumentParser()
    add_link_args(ap)
    ap.add_argument("--records", required=True); ap.add_argument("--weights", required=True)
    ap.add_argument("--healthy", required=True); ap.add_argument("--fault", required=True)
    ap.add_argument("--div", type=int, default=249); ap.add_argument("--pre", type=int, default=24)
    ap.add_argument("--thr", type=int, default=100); ap.add_argument("--ncons", type=int, default=3)
    ap.add_argument("--max-n", type=int, default=7); ap.add_argument("--out", required=True)
    a = ap.parse_args()

    d = np.load(a.records); meta = json.loads(str(d["meta"]))
    params = load_weights(a.weights)
    codes, seq, label = d["codes"], d["seq"], d["label"]
    sh, sf = seq_of_cond(d, a.healthy), seq_of_cond(d, a.fault)
    H, Fc = codes[seq == sh].astype(np.int64), codes[seq == sf].astype(np.int64)
    fs = F_CLK / (35 * (a.div + 1)); t_win = 64 / fs
    chk = dict(n=0, mismatch=0)

    link = open_link(a)
    info = link.identify()
    st, rb = link.load_weights(params)
    if st or not (rb.astype(np.int64) == params).all():
        sys.exit("weight load failed")
    print(f"board 0x{info['id']:08X}; healthy {a.healthy} (seq {sh}, {len(H)} windows), fault {a.fault} (seq {sf}, {len(Fc)} windows); "
          f"D = {a.div} ({fs:.1f} S/s, {1e3 * t_win:.1f} ms per window)", flush=True)

    # 1. onset block (hardware alarm, N = ncons)
    onset = np.concatenate([H[:a.pre], Fc[:64 - a.pre]])
    t0 = time.time()
    on = run_blocks(link, onset, params, a.div, a.thr, a.ncons, bins=True, check=chk)
    print(f"onset block: {len(on)} windows in {time.time() - t0:.2f} s; decisions {[w['cls'] for w in on]}; "
          f"alarm {[w['alarm'] for w in on]}", flush=True)

    # 2. long timeline (decisions + chip spectrogram; alarm from the model over the continuous stream)
    stream = np.concatenate([H, Fc])
    t0 = time.time()
    tl = run_blocks(link, stream, params, a.div, a.thr, a.ncons, bins=True, check=chk)
    alarm_cont = alarm_model([(w["cls"], w["conf"]) for w in tl], a.thr, a.ncons)
    print(f"timeline: {len(tl)} windows ({len(tl) * t_win:.1f} s of machine time) in {time.time() - t0:.1f} s; "
          f"{sum(w['cls'] != 0 for w in tl[:len(H)])} fault decisions before the onset, {sum(w['cls'] == 0 for w in tl[len(H):])} healthy after", flush=True)

    # 3. alarm depth sweep in hardware
    healthy_test = codes[(label == 0) & (d["seq_split"][seq] == "test")].astype(np.int64)
    depth = []
    for n in range(0, a.max_n + 1):
        o = run_blocks(link, onset, params, a.div, a.thr, n, bins=False)
        al = [w["alarm"] for w in o]
        delay = (al.index(1) - a.pre + 1) if 1 in al else None       # windows after the onset until the alarm asserts
        hb = run_blocks(link, healthy_test, params, a.div, a.thr, n, bins=False)
        fa_blocks = 0; fa_windows = 0
        for b0 in range(0, len(hb), 64):
            flags = [w["alarm"] for w in hb[b0:b0 + 64]]
            fa_blocks += 1 if any(flags) else 0; fa_windows += sum(flags)
        depth.append(dict(n=n, delay_windows=delay, delay_s=None if delay is None else delay * t_win,
                          healthy_windows=len(hb), healthy_blocks=(len(hb) + 63) // 64, false_alarm_blocks=fa_blocks,
                          false_alarm_windows=fa_windows))
        print(f"N = {n}: delay {delay} windows; false alarms in {fa_blocks}/{(len(hb) + 63) // 64} healthy blocks ({fa_windows} windows)", flush=True)

    # 4. one representative window per class (highest-confidence correct test window)
    spectra = []
    names = meta["class_names"]
    test_m = d["seq_split"][seq] == "test"
    for c in range(4):
        idx = np.flatnonzero((label == c) & test_m)
        if len(idx) == 0:
            continue
        pick = idx[np.random.RandomState(c).choice(len(idx), size=min(32, len(idx)), replace=False)]
        res = run_blocks(link, codes[pick].astype(np.int64), params, a.div, a.thr, a.ncons, bins=True, check=chk)
        best = max(range(len(res)), key=lambda k: (res[k]["cls"] == c, res[k]["conf"]))
        spectra.append(dict(cls=c, name=names[c], window=int(pick[best]), cond=str(d["seq_cond"][seq[pick[best]]]),
                            codes=[int(v) for v in codes[pick[best]]], bins=res[best]["bins"], feat=res[best]["feat"],
                            decision=res[best]["cls"], conf=res[best]["conf"]))
    link.quit(); link.close()

    out = dict(board_id=f"0x{info['id']:08X}", records=str(a.records), weights=str(a.weights), class_names=names,
               div=a.div, fs=fs, t_window_s=t_win, threshold=a.thr, n_consec=a.ncons, pre=a.pre,
               healthy=dict(cond=a.healthy, seq=sh, windows=len(H)), fault=dict(cond=a.fault, seq=sf, windows=len(Fc)),
               onset=dict(codes=[[int(v) for v in w] for w in onset], windows=on),
               timeline=dict(codes_rms=[float(np.sqrt(np.mean((w - 2048.0) ** 2))) for w in stream],
                             windows=[dict(cls=w["cls"], conf=w["conf"], feat=w["feat"], bins=w["bins"]) for w in tl],
                             alarm_model=alarm_cont),
               alarm_depth=depth, class_spectra=spectra, checked=chk["n"], mismatches=chk["mismatch"])
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).with_suffix(".json").write_text(json.dumps(out))
    print(f"bit-exact against the reference chain: {chk['n'] - chk['mismatch']}/{chk['n']} windows; wrote {Path(a.out).with_suffix('.json')}")


if __name__ == "__main__":
    main()
