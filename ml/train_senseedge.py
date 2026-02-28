# SPDX-License-Identifier: Apache-2.0
# SenseEdge ML Training Pipeline
# Trains a tiny neural network (8->16->4) for vibration fault classification
# Produces INT8 quantized weights compatible with nn_engine.v

"""
Train a small fully-connected neural network for the SenseEdge ASIC.

Architecture: 8 inputs -> 16 hidden (ReLU) -> 4 outputs (argmax)
All weights quantized to INT8 [-128, 127] for hardware inference.
Total parameters: (8*16) + 16 + (16*4) + 4 = 212

Features match the hardware feature_extract.v module:
  [0] Band energy low     (FFT bins 1-4)
  [1] Band energy mid-low (FFT bins 5-10)
  [2] Band energy mid-hi  (FFT bins 11-20)
  [3] Band energy high    (FFT bins 21-31)
  [4] Peak frequency      (bin index of max)
  [5] Peak magnitude      (value at peak bin)
  [6] Spectral centroid   (weighted avg frequency)
  [7] Total energy        (sum all bins)

All features are normalized to [0, 255] (uint8) to match hardware.
"""

import argparse
import os
import sys
import numpy as np


# ---------------------------------------------------------------------------
# Synthetic data generation
# ---------------------------------------------------------------------------

def generate_synthetic_data(n_samples_per_class=500, seed=42):
    """Generate synthetic vibration feature data for 4 fault classes.

    Each sample is 8 features in [0, 255] (uint8 range) that mimic the
    spectral characteristics produced by feature_extract.v.

    Classes:
      0 - Healthy:      energy concentrated in low band, low total energy
      1 - Bearing Wear: energy in mid-low band, high peak magnitude
      2 - Imbalance:    energy in mid-high band
      3 - Misalignment: energy in high band

    Returns:
        X : ndarray (N, 8) float64 in [0, 255]
        y : ndarray (N,) int, class labels 0-3
    """
    rng = np.random.RandomState(seed)
    n = n_samples_per_class
    X_all, y_all = [], []

    # --- Class 0: Healthy ---
    band_low     = rng.uniform(140, 255, n)
    band_midlow  = rng.uniform(20, 80, n)
    band_midhi   = rng.uniform(5, 40, n)
    band_high    = rng.uniform(0, 25, n)
    peak_freq    = rng.uniform(10, 60, n)        # low bin index * 8
    peak_mag     = rng.uniform(30, 100, n)
    centroid     = rng.uniform(10, 50, n)         # low spectral centroid
    total_energy = rng.uniform(40, 120, n)        # moderate-low
    X_all.append(np.column_stack([
        band_low, band_midlow, band_midhi, band_high,
        peak_freq, peak_mag, centroid, total_energy]))
    y_all.append(np.zeros(n, dtype=int))

    # --- Class 1: Bearing Wear ---
    band_low     = rng.uniform(40, 100, n)
    band_midlow  = rng.uniform(150, 255, n)       # dominant mid-low
    band_midhi   = rng.uniform(30, 80, n)
    band_high    = rng.uniform(10, 50, n)
    peak_freq    = rng.uniform(50, 110, n)         # mid-range peak
    peak_mag     = rng.uniform(180, 255, n)        # high peak magnitude
    centroid     = rng.uniform(50, 100, n)
    total_energy = rng.uniform(120, 200, n)
    X_all.append(np.column_stack([
        band_low, band_midlow, band_midhi, band_high,
        peak_freq, peak_mag, centroid, total_energy]))
    y_all.append(np.ones(n, dtype=int))

    # --- Class 2: Imbalance ---
    band_low     = rng.uniform(20, 70, n)
    band_midlow  = rng.uniform(30, 80, n)
    band_midhi   = rng.uniform(160, 255, n)        # dominant mid-high
    band_high    = rng.uniform(20, 70, n)
    peak_freq    = rng.uniform(100, 170, n)         # higher peak bin
    peak_mag     = rng.uniform(100, 180, n)
    centroid     = rng.uniform(100, 160, n)
    total_energy = rng.uniform(100, 180, n)
    X_all.append(np.column_stack([
        band_low, band_midlow, band_midhi, band_high,
        peak_freq, peak_mag, centroid, total_energy]))
    y_all.append(np.full(n, 2, dtype=int))

    # --- Class 3: Misalignment ---
    band_low     = rng.uniform(10, 50, n)
    band_midlow  = rng.uniform(15, 60, n)
    band_midhi   = rng.uniform(40, 100, n)
    band_high    = rng.uniform(170, 255, n)         # dominant high band
    peak_freq    = rng.uniform(170, 248, n)          # high peak bin
    peak_mag     = rng.uniform(120, 210, n)
    centroid     = rng.uniform(160, 230, n)          # high centroid
    total_energy = rng.uniform(130, 220, n)
    X_all.append(np.column_stack([
        band_low, band_midlow, band_midhi, band_high,
        peak_freq, peak_mag, centroid, total_energy]))
    y_all.append(np.full(n, 3, dtype=int))

    X = np.vstack(X_all)
    y = np.concatenate(y_all)

    # Shuffle
    idx = rng.permutation(len(y))
    return X[idx], y[idx]


# ---------------------------------------------------------------------------
# Real CWRU data loading (optional, requires scipy)
# ---------------------------------------------------------------------------

def load_cwru_data(data_dir):
    """Load real CWRU bearing dataset .mat files and extract 8 features.

    Expected directory layout (standard CWRU filenames):
      data_dir/Normal.mat          -> class 0
      data_dir/B007.mat            -> class 1 (bearing inner race)
      data_dir/IR007.mat           -> class 2 (imbalance/inner race)
      data_dir/OR007.mat           -> class 3 (outer race / misalignment)

    Each .mat file should contain a drive-end accelerometer signal key
    ending in '_DE_time'.

    Returns:
        X : ndarray (N, 8) float64 in [0, 255]
        y : ndarray (N,) int
    """
    try:
        from scipy.io import loadmat
    except ImportError:
        print("ERROR: scipy is required to load .mat files.")
        print("       Install with: pip install scipy")
        sys.exit(1)

    file_class_map = {
        "Normal.mat": 0,
        "B007.mat":   1,
        "IR007.mat":  2,
        "OR007.mat":  3,
    }

    X_all, y_all = [], []
    window_size = 1024  # samples per window (gives 512 FFT bins, we use 32)
    hop = 512

    for fname, label in file_class_map.items():
        fpath = os.path.join(data_dir, fname)
        if not os.path.isfile(fpath):
            print(f"  WARNING: {fpath} not found, skipping class {label}")
            continue

        mat = loadmat(fpath)
        # Find the drive-end accelerometer key
        de_key = None
        for k in mat.keys():
            if k.endswith("_DE_time"):
                de_key = k
                break
        if de_key is None:
            print(f"  WARNING: No DE_time key in {fname}, skipping")
            continue

        signal = mat[de_key].flatten()
        n_windows = (len(signal) - window_size) // hop

        for i in range(n_windows):
            seg = signal[i * hop: i * hop + window_size]
            features = _extract_features_from_signal(seg)
            X_all.append(features)
            y_all.append(label)

    if not X_all:
        print("ERROR: No data loaded. Check data_dir and filenames.")
        sys.exit(1)

    X = np.array(X_all, dtype=np.float64)
    y = np.array(y_all, dtype=int)

    # Normalize each feature to [0, 255]
    for j in range(8):
        col = X[:, j]
        cmin, cmax = col.min(), col.max()
        if cmax > cmin:
            X[:, j] = (col - cmin) / (cmax - cmin) * 255.0
        else:
            X[:, j] = 0.0

    idx = np.random.permutation(len(y))
    return X[idx], y[idx]


def _extract_features_from_signal(segment):
    """Compute 8 features from a time-domain segment, mirroring
    the hardware feature_extract.v module.

    Uses a 64-point FFT and takes the first 32 magnitude bins.
    """
    n_fft = 64
    fft_vals = np.fft.rfft(segment[:n_fft])
    mag = np.abs(fft_vals)[:32]  # bins 0..31

    band_low    = np.sum(mag[1:5])
    band_midlow = np.sum(mag[5:11])
    band_midhi  = np.sum(mag[11:21])
    band_high   = np.sum(mag[21:32])

    peak_bin = np.argmax(mag)
    peak_mag_val = mag[peak_bin]

    total = np.sum(mag)
    if total > 0:
        centroid = np.sum(np.arange(32) * mag) / total
    else:
        centroid = 0.0

    return np.array([
        band_low, band_midlow, band_midhi, band_high,
        peak_bin * 8.0,       # scale like hardware: peak_bin << 3
        peak_mag_val,
        centroid * 8.0,       # scale to approximate hardware range
        total,
    ])


# ---------------------------------------------------------------------------
# Neural network forward / backward (pure numpy)
# ---------------------------------------------------------------------------

def softmax(logits):
    """Numerically stable softmax."""
    e = np.exp(logits - logits.max(axis=1, keepdims=True))
    return e / e.sum(axis=1, keepdims=True)


def relu(x):
    return np.maximum(0, x)


def relu_grad(x):
    return (x > 0).astype(np.float64)


def cross_entropy_loss(probs, labels):
    """Mean cross-entropy loss over batch."""
    n = len(labels)
    log_probs = -np.log(probs[np.arange(n), labels] + 1e-12)
    return log_probs.mean()


def forward(X, W1, b1, W2, b2):
    """Forward pass.

    Args:
        X:  (batch, 8)   inputs in [0, 255]
        W1: (16, 8)      layer 1 weights
        b1: (16,)        layer 1 biases
        W2: (4, 16)      layer 2 weights
        b2: (4,)         layer 2 biases

    Returns:
        z1, h1, z2, probs
    """
    z1 = X @ W1.T + b1           # (batch, 16)
    h1 = relu(z1)                 # (batch, 16)
    z2 = h1 @ W2.T + b2          # (batch, 4)
    probs = softmax(z2)           # (batch, 4)
    return z1, h1, z2, probs


def backward(X, z1, h1, probs, labels, W2):
    """Backward pass with cross-entropy + softmax gradient.

    Returns:
        dW1, db1, dW2, db2
    """
    n = X.shape[0]

    # Output layer gradient: softmax + cross-entropy
    dz2 = probs.copy()
    dz2[np.arange(n), labels] -= 1.0
    dz2 /= n

    dW2 = dz2.T @ h1             # (4, 16)
    db2 = dz2.sum(axis=0)        # (4,)

    # Hidden layer gradient
    dh1 = dz2 @ W2               # (batch, 16)
    dz1 = dh1 * relu_grad(z1)    # (batch, 16)

    dW1 = dz1.T @ X              # (16, 8)
    db1 = dz1.sum(axis=0)        # (16,)

    return dW1, db1, dW2, db2


# ---------------------------------------------------------------------------
# INT8 quantization
# ---------------------------------------------------------------------------

def quantize_int8(weights):
    """Quantize float weights to INT8 [-128, 127].

    Uses symmetric min-max quantization.
    Returns quantized weights (int8 ndarray) and the scale factor.
    """
    w_max = np.max(np.abs(weights))
    if w_max == 0:
        return np.zeros_like(weights, dtype=np.int8), 1.0
    scale = 127.0 / w_max
    q = np.round(weights * scale).astype(np.int64)
    q = np.clip(q, -128, 127).astype(np.int8)
    return q, scale


# ---------------------------------------------------------------------------
# Training loop
# ---------------------------------------------------------------------------

def train(X_train, y_train, X_val, y_val,
          epochs=200, batch_size=64, lr=0.01, lr_decay=0.995, seed=123):
    """Train the 8->16->4 network using mini-batch SGD.

    Returns:
        W1, b1, W2, b2 (float64 weights)
    """
    rng = np.random.RandomState(seed)

    # Xavier initialization
    W1 = rng.randn(16, 8) * np.sqrt(2.0 / 8)
    b1 = np.zeros(16)
    W2 = rng.randn(4, 16) * np.sqrt(2.0 / 16)
    b2 = np.zeros(4)

    n_train = X_train.shape[0]
    best_val_acc = 0.0
    best_params = (W1.copy(), b1.copy(), W2.copy(), b2.copy())

    print(f"Training: {n_train} samples, {epochs} epochs, "
          f"batch_size={batch_size}, lr={lr}")
    print("-" * 65)

    for epoch in range(epochs):
        # Shuffle training data
        perm = rng.permutation(n_train)
        X_shuf = X_train[perm]
        y_shuf = y_train[perm]

        epoch_loss = 0.0
        n_batches = 0

        for start in range(0, n_train, batch_size):
            end = min(start + batch_size, n_train)
            Xb = X_shuf[start:end]
            yb = y_shuf[start:end]

            z1, h1, z2, probs = forward(Xb, W1, b1, W2, b2)
            loss = cross_entropy_loss(probs, yb)
            epoch_loss += loss
            n_batches += 1

            dW1, db1, dW2, db2 = backward(Xb, z1, h1, probs, yb, W2)

            W1 -= lr * dW1
            b1 -= lr * db1
            W2 -= lr * dW2
            b2 -= lr * db2

        lr *= lr_decay
        epoch_loss /= n_batches

        # Validation accuracy
        _, _, _, val_probs = forward(X_val, W1, b1, W2, b2)
        val_preds = np.argmax(val_probs, axis=1)
        val_acc = np.mean(val_preds == y_val)

        if val_acc > best_val_acc:
            best_val_acc = val_acc
            best_params = (W1.copy(), b1.copy(), W2.copy(), b2.copy())

        if (epoch + 1) % 20 == 0 or epoch == 0:
            print(f"  Epoch {epoch+1:4d}/{epochs}  "
                  f"loss={epoch_loss:.4f}  "
                  f"val_acc={val_acc*100:.1f}%  "
                  f"lr={lr:.6f}")

    print("-" * 65)
    print(f"Best validation accuracy: {best_val_acc*100:.1f}%")
    return best_params


# ---------------------------------------------------------------------------
# Evaluate quantized model
# ---------------------------------------------------------------------------

def evaluate_quantized(X, y, W1_q, b1_q, W2_q, b2_q):
    """Run inference with INT8 weights (cast to float for numpy matmul)
    and report accuracy.  This simulates what the hardware will compute."""
    _, _, _, probs = forward(
        X,
        W1_q.astype(np.float64), b1_q.astype(np.float64),
        W2_q.astype(np.float64), b2_q.astype(np.float64),
    )
    preds = np.argmax(probs, axis=1)
    acc = np.mean(preds == y)
    return acc


# ---------------------------------------------------------------------------
# Save weights
# ---------------------------------------------------------------------------

def save_weights(path, W1_q, b1_q, W2_q, b2_q, scales):
    """Save INT8 quantized weights to .npz file.

    Memory layout matches nn_engine.v:
      [0..127]   Layer 1 weights (16 neurons x 8 inputs, row-major)
      [128..143]  Layer 1 biases
      [144..207]  Layer 2 weights (4 neurons x 16 inputs, row-major)
      [208..211]  Layer 2 biases
    """
    # Build flat array matching hardware memory layout
    all_weights = np.concatenate([
        W1_q.flatten(),   # [0..127]   16x8 row-major
        b1_q.flatten(),   # [128..143]
        W2_q.flatten(),   # [144..207] 4x16 row-major
        b2_q.flatten(),   # [208..211]
    ]).astype(np.int8)

    assert len(all_weights) == 212, \
        f"Expected 212 parameters, got {len(all_weights)}"

    np.savez(path,
             layer1_weights=W1_q,
             layer1_biases=b1_q,
             layer2_weights=W2_q,
             layer2_biases=b2_q,
             all_weights=all_weights,
             scales=np.array(scales))

    print(f"Saved INT8 weights to {path}")
    print(f"  Layer 1 weights: {W1_q.shape}  range [{W1_q.min()}, {W1_q.max()}]")
    print(f"  Layer 1 biases:  {b1_q.shape}  range [{b1_q.min()}, {b1_q.max()}]")
    print(f"  Layer 2 weights: {W2_q.shape}  range [{W2_q.min()}, {W2_q.max()}]")
    print(f"  Layer 2 biases:  {b2_q.shape}  range [{b2_q.min()}, {b2_q.max()}]")
    print(f"  Total: {len(all_weights)} parameters")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Train SenseEdge vibration classifier (8->16->4 INT8)")
    parser.add_argument("--cwru-dir", type=str, default=None,
                        help="Path to CWRU .mat files (omit for synthetic data)")
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--lr", type=float, default=0.01)
    parser.add_argument("--samples", type=int, default=500,
                        help="Samples per class for synthetic data")
    parser.add_argument("--output", type=str, default=None,
                        help="Output .npz path (default: ml/senseedge_weights.npz)")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    if args.output is None:
        args.output = os.path.join(os.path.dirname(__file__),
                                   "senseedge_weights.npz")

    print("=" * 65)
    print("SenseEdge ML Training Pipeline")
    print("Network: 8 inputs -> 16 hidden (ReLU) -> 4 outputs (argmax)")
    print("Quantization: INT8 [-128, 127]")
    print("Total parameters: 212")
    print("=" * 65)

    # --- Load data ---
    if args.cwru_dir is not None:
        print(f"\nLoading CWRU data from {args.cwru_dir} ...")
        X, y = load_cwru_data(args.cwru_dir)
    else:
        print("\nGenerating synthetic training data ...")
        X, y = generate_synthetic_data(n_samples_per_class=args.samples,
                                       seed=args.seed)

    # Clip to uint8 range (hardware operates on [0, 255])
    X = np.clip(X, 0, 255)

    print(f"  Dataset: {X.shape[0]} samples, {X.shape[1]} features, "
          f"{len(np.unique(y))} classes")
    for c in range(4):
        n_c = np.sum(y == c)
        print(f"    Class {c}: {n_c} samples")

    # --- Train/validation split (80/20) ---
    n = len(y)
    split = int(0.8 * n)
    X_train, y_train = X[:split], y[:split]
    X_val, y_val = X[split:], y[split:]

    print(f"\n  Train: {len(y_train)} samples")
    print(f"  Val:   {len(y_val)} samples\n")

    # --- Train ---
    W1, b1, W2, b2 = train(X_train, y_train, X_val, y_val,
                            epochs=args.epochs,
                            batch_size=args.batch_size,
                            lr=args.lr,
                            seed=args.seed)

    # --- Float32 accuracy ---
    _, _, _, probs = forward(X_val, W1, b1, W2, b2)
    preds = np.argmax(probs, axis=1)
    float_acc = np.mean(preds == y_val)
    print(f"\nFloat accuracy (validation): {float_acc*100:.1f}%")

    # --- Quantize to INT8 ---
    print("\nQuantizing to INT8 ...")
    W1_q, s1w = quantize_int8(W1)
    b1_q, s1b = quantize_int8(b1)
    W2_q, s2w = quantize_int8(W2)
    b2_q, s2b = quantize_int8(b2)

    int8_acc = evaluate_quantized(X_val, y_val, W1_q, b1_q, W2_q, b2_q)
    print(f"INT8 accuracy (validation):  {int8_acc*100:.1f}%")

    if int8_acc < 0.90:
        print("WARNING: INT8 accuracy below 90% target.")
        print("         Consider increasing training epochs or adjusting lr.")

    # --- Per-class accuracy ---
    _, _, _, probs_q = forward(
        X_val,
        W1_q.astype(np.float64), b1_q.astype(np.float64),
        W2_q.astype(np.float64), b2_q.astype(np.float64),
    )
    preds_q = np.argmax(probs_q, axis=1)
    class_names = ["Healthy", "Bearing Wear", "Imbalance", "Misalignment"]
    print("\nPer-class accuracy (INT8):")
    for c in range(4):
        mask = y_val == c
        if mask.sum() > 0:
            cacc = np.mean(preds_q[mask] == c)
            print(f"  Class {c} ({class_names[c]:>14s}): {cacc*100:.1f}%")

    # --- Save ---
    scales = [s1w, s1b, s2w, s2b]
    save_weights(args.output, W1_q, b1_q, W2_q, b2_q, scales)

    print("\nDone.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
