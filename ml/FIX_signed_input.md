# Critical fix: signed-INT8 input domain for the NN

## The defect (weights only — silicon is unaffected)

`nn_engine.v` declares the feature inputs as `reg signed [7:0]`, so the MAC
reads every 8-bit feature as a **signed** value in `[-128, +127]`. But
`feature_extract.v` emits **unsigned** features in `[0, 255]`, and the original
`train_senseedge.py` trained on unsigned inputs. Every feature byte `>= 128`
(the common case for the dominant band of each fault class) is therefore
multiplied with the **wrong sign** in silicon. The deployed weights computed a
different function than the hardware, so out-of-box classification on real
signals would be wrong.

This was not caught by `tb_nn_engine.v` because its stimulus uses only small
feature values (`8, 10, 120`) — all `< 128`, where signed and unsigned agree.

## Why no re-spin is needed

The datapath is self-consistent (`signed × signed`), and the network weights
are **runtime-loadable over Wishbone** (`ADDR_NN_WEIGHTS`). The fix is entirely
in software: train in the hardware's exact numeric domain and reload the 212
bytes. The fabricated die is correct as-is.

## The fix

`train_senseedge_fixed.py` replaces the training/quantization/eval path with a
**bit-exact model of `nn_engine.v`**:

- inputs reinterpreted as `int8` (the value the MAC actually sees);
- L1 MAC in 24-bit, bias sign-extended from int8 (exactly as RTL);
- hidden activation stored into a **signed 16-bit** register (int16 wrap
  modeled) then ReLU;
- L2 MAC in 24-bit, argmax decision;
- L1 weights uniformly down-shifted if the worst-case activation would exceed
  int16 — an **argmax-preserving** scaling (ReLU is positive-homogeneous and
  the decision is an argmax), so accuracy is not affected.

The reported accuracy is measured on the **hardware-exact** datapath, so it is
what the silicon will produce:

```
$ python3 ml/train_senseedge_fixed.py
float(signed-domain) val acc = 91.8%
HARDWARE-EXACT val acc       = 90.5%   (L1 down-shift=1)
saved ml/senseedge_weights.npz and ml/senseedge_weights.hex (212 params, signed-int8 input domain)
```

Outputs:
- `senseedge_weights.npz` — regenerated weights, signed-int8 input domain.
- `senseedge_weights.hex` — 212-byte `$readmemh` image for gate-level / bench
  reload and for the firmware weight-load routine.

## Bring-up validation (do on first silicon)

Load `senseedge_weights.hex` over Wishbone, drive a known feature vector, and
compare `class_id`/`confidence` bit-exactly against `hw_infer()` in the fixed
script (and against the RTL golden). Retrain on real CWRU/field features in the
same signed domain before recording deployment accuracy for publication.

> The original `train_senseedge.py` is retained for history; **do not use its
> weights on silicon.** Use `train_senseedge_fixed.py`.
