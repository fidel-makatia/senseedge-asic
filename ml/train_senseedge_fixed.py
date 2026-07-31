#!/usr/bin/env python3
"""SenseEdge NN training — HARDWARE-EXACT, signed-INT8 input domain.

Why this file exists
--------------------
`nn_engine.v` declares the feature inputs as `reg signed [7:0]`, so the MAC
interprets each 8-bit feature as a *signed* value in [-128, +127]. The
feature extractor and the original training script produced *unsigned*
features in [0, 255]. Any feature >= 128 was therefore multiplied with the
wrong sign in silicon, and the deployed weights (trained on unsigned inputs)
computed a different function than the hardware. Fabricated silicon cannot be
changed, but the weights are runtime-loadable over Wishbone, so the fix lives
entirely here: train in the hardware's exact numeric domain and reload.

What this script models bit-accurately (matches nn_engine.v):
  * inputs      : int8  (feature byte reinterpreted as signed)
  * L1 MAC      : acc24 = sum(int8 x int8) + sign_extend(int8 bias)
  * hidden      : acc truncated to signed 16-bit (int16 wrap), then ReLU
  * L2 MAC      : acc24 = sum(int16 x int8) + sign_extend(int8 bias)
  * decision    : argmax over 4 outputs
Because the layer-1 activation is stored in 16 bits with no requant shift,
L1 weights are scaled so the worst-case activation stays within int16; a
uniform L1 scale is argmax-preserving (ReLU is positive-homogeneous and the
final decision is an argmax), so accuracy is not affected by that scaling.
"""
import argparse
import os
import numpy as np

INT16_MAX = 32767
INT16_MIN = -32768


# --------------------------------------------------------------------------
# synthetic feature generator — features are UNSIGNED bytes [0,255] exactly as
# feature_extract.v emits them; the signed reinterpretation happens at the
# hardware boundary (to_hw_int8), matching silicon.
# --------------------------------------------------------------------------
def gen_data(n_per_class=500, seed=42):
    rng = np.random.RandomState(seed)
    X, y = [], []
    def clip(a): return np.clip(a, 0, 255)
    # feature order: band_low, band_midlow, band_midhi, band_high,
    #                peak_bin<<3, peak_mag, centroid, total_energy
    specs = [
        # healthy: low broadband, no dominant band
        lambda n: np.stack([clip(rng.uniform(20, 90, n)),  clip(rng.uniform(20, 90, n)),
                            clip(rng.uniform(20, 90, n)),  clip(rng.uniform(20, 90, n)),
                            clip(rng.uniform(40, 210, n)), clip(rng.uniform(30, 110, n)),
                            clip(rng.uniform(60, 190, n)), clip(rng.uniform(40, 160, n))], 1),
        # bearing wear: dominant high band + high peak mag
        lambda n: np.stack([clip(rng.uniform(20, 90, n)),  clip(rng.uniform(30, 120, n)),
                            clip(rng.uniform(90, 200, n)), clip(rng.uniform(170, 255, n)),
                            clip(rng.uniform(180, 248, n)),clip(rng.uniform(180, 255, n)),
                            clip(rng.uniform(150, 240, n)),clip(rng.uniform(160, 255, n))], 1),
        # imbalance: dominant low band, low peak bin
        lambda n: np.stack([clip(rng.uniform(160, 255, n)),clip(rng.uniform(60, 150, n)),
                            clip(rng.uniform(20, 90, n)),  clip(rng.uniform(20, 80, n)),
                            clip(rng.uniform(8, 64, n)),   clip(rng.uniform(150, 255, n)),
                            clip(rng.uniform(40, 120, n)), clip(rng.uniform(150, 240, n))], 1),
        # misalignment: dominant mid bands, 1x/2x structure
        lambda n: np.stack([clip(rng.uniform(120, 220, n)),clip(rng.uniform(150, 255, n)),
                            clip(rng.uniform(140, 240, n)),clip(rng.uniform(40, 120, n)),
                            clip(rng.uniform(64, 160, n)), clip(rng.uniform(150, 255, n)),
                            clip(rng.uniform(90, 190, n)), clip(rng.uniform(160, 255, n))], 1),
    ]
    for cls, gen in enumerate(specs):
        X.append(gen(n_per_class)); y.append(np.full(n_per_class, cls))
    X = np.vstack(X).astype(np.float64)
    y = np.concatenate(y)
    return X, y


def to_hw_int8(X_uint8):
    """Reinterpret unsigned feature bytes as the signed values the MAC sees."""
    return np.int8(np.clip(np.round(X_uint8), 0, 255).astype(np.uint8)).astype(np.float64)


# --------------------------------------------------------------------------
# float training (on the signed-int8 input domain)
# --------------------------------------------------------------------------
def relu(x): return np.maximum(0, x)
def softmax(z):
    z = z - z.max(1, keepdims=True); e = np.exp(z); return e / e.sum(1, keepdims=True)

def fwd(Xs, W1, b1, W2, b2):
    z1 = Xs @ W1.T + b1; h1 = relu(z1); z2 = h1 @ W2.T + b2
    return z1, h1, z2, softmax(z2)

def train(Xs, y, epochs=300, lr=0.02, seed=123):
    rng = np.random.RandomState(seed)
    W1 = rng.randn(16, 8) * np.sqrt(2/8); b1 = np.zeros(16)
    W2 = rng.randn(4, 16) * np.sqrt(2/16); b2 = np.zeros(4)
    # scale inputs into a unit-ish range for stable float training; the learned
    # W1 is rescaled to the int8 grid afterward, so this is only conditioning.
    Xn = Xs / 128.0
    best = None; best_acc = -1
    for ep in range(epochs):
        p = rng.permutation(len(y)); Xb, yb = Xn[p], y[p]
        z1, h1, z2, pr = fwd(Xb, W1, b1, W2, b2)
        n = len(yb)
        dz2 = pr.copy(); dz2[np.arange(n), yb] -= 1; dz2 /= n
        dW2 = dz2.T @ h1; db2 = dz2.sum(0)
        dz1 = (dz2 @ W2) * (z1 > 0); dW1 = dz1.T @ Xb; db1 = dz1.sum(0)
        W1 -= lr*dW1; b1 -= lr*db1; W2 -= lr*dW2; b2 -= lr*db2
        acc = (np.argmax(fwd(Xn, W1, b1, W2, b2)[3], 1) == y).mean()
        if acc > best_acc: best_acc, best = acc, (W1.copy()/128.0, b1.copy(), W2.copy(), b2.copy())
    return best  # W1 folded back to the raw (un-normalized) input scale


# --------------------------------------------------------------------------
# quantization to the exact hardware grid
# --------------------------------------------------------------------------
def q_int8(w):
    m = np.max(np.abs(w))
    if m == 0: return np.zeros_like(w, np.int64), 1.0
    s = 127.0 / m
    return np.clip(np.round(w * s), -128, 127).astype(np.int64), s

def quantize(W1, b1, W2, b2, Xs):
    # L1 weights to int8; then check int16 activation headroom on the data and
    # uniformly down-scale L1 (weights+bias together) if it would overflow.
    W1q, s1 = q_int8(W1)
    b1q = np.clip(np.round(b1 * s1), -128, 127).astype(np.int64)  # bias on weight grid, int8
    # worst observed L1 pre-activation with these int8 params
    acc1 = Xs.astype(np.int64) @ W1q.T + b1q
    peak = np.max(np.abs(acc1)) if acc1.size else 0
    shift = 0
    while peak >> shift > INT16_MAX:
        shift += 1
    if shift:
        W1q = (W1q >> shift); b1q = (b1q >> shift)  # argmax-preserving downscale
    W2q, s2 = q_int8(W2)
    b2q = np.clip(np.round(b2 * s2), -128, 127).astype(np.int64)
    return (W1q.astype(np.int8), b1q.astype(np.int8),
            W2q.astype(np.int8), b2q.astype(np.int8), shift)


# --------------------------------------------------------------------------
# BIT-EXACT hardware datapath (mirrors nn_engine.v)
# --------------------------------------------------------------------------
def to_int16(x):
    return ((x + 32768) & 0xFFFF) - 32768   # signed 16-bit wrap, like acc[15:0]

def hw_infer(X_uint8, W1q, b1q, W2q, b2q):
    Xi = np.int8(np.clip(np.round(X_uint8),0,255).astype(np.uint8)).astype(np.int64)  # signed input
    preds = np.empty(len(Xi), np.int64)
    for i, x in enumerate(Xi):
        acc1 = x @ W1q.T.astype(np.int64) + b1q.astype(np.int64)     # 24-bit acc (numpy int64 ok)
        hid = to_int16(acc1)                                          # store into signed[15:0]
        hid = np.maximum(0, hid)                                      # ReLU
        acc2 = hid @ W2q.T.astype(np.int64) + b2q.astype(np.int64)
        preds[i] = int(np.argmax(acc2))
    return preds


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", type=int, default=500)
    ap.add_argument("--epochs", type=int, default=300)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--output", default=os.path.join(os.path.dirname(__file__), "senseedge_weights.npz"))
    ap.add_argument("--hex", default=os.path.join(os.path.dirname(__file__), "senseedge_weights.hex"))
    a = ap.parse_args()

    Xu, y = gen_data(a.samples, a.seed)                 # unsigned features (as HW emits)
    Xs = to_hw_int8(Xu)                                 # signed domain (as MAC reads)
    # split
    rng = np.random.RandomState(0); idx = rng.permutation(len(y))
    ntr = int(0.8*len(y)); tr, va = idx[:ntr], idx[ntr:]
    W1, b1, W2, b2 = train(Xs[tr], y[tr], a.epochs, seed=123)
    fa = (np.argmax(fwd(Xs[va]/128.0, W1*128.0, b1, W2, b2)[3],1) == y[va]).mean()
    W1q, b1q, W2q, b2q, shift = quantize(W1, b1, W2, b2, Xs[tr])
    # accuracy of the EXACT hardware datapath on held-out data:
    hw_acc = (hw_infer(Xu[va], W1q, b1q, W2q, b2q) == y[va]).mean()
    print(f"float(signed-domain) val acc = {fa*100:.1f}%")
    print(f"HARDWARE-EXACT val acc       = {hw_acc*100:.1f}%   (L1 down-shift={shift})")

    packed = np.concatenate([W1q.flatten(), b1q.flatten(),
                             W2q.flatten(), b2q.flatten()]).astype(np.int8)
    assert packed.size == 212
    np.savez(a.output, layer1_weights=W1q, layer1_biases=b1q,
             layer2_weights=W2q, layer2_biases=b2q, all_weights=packed,
             input_domain=np.array(["signed_int8"]))
    with open(a.hex, "w") as f:                          # $readmemh, two-hex bytes
        for v in packed.astype(np.uint8): f.write(f"{v:02x}\n")
    print(f"saved {a.output} and {a.hex}  ({packed.size} params, signed-int8 input domain)")

if __name__ == "__main__":
    main()
