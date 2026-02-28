# SenseEdge ML Training Pipeline

Train and export INT8 neural network weights for the SenseEdge vibration classifier ASIC.

**Network:** 8 inputs -> 16 hidden (ReLU) -> 4 outputs (argmax)
**Parameters:** 212 total (128 + 16 + 64 + 4), INT8 quantized

## Setup

```
pip install -r requirements.txt
```

## Train

Using synthetic data (no external files needed):

```
python train_senseedge.py
```

Using real CWRU bearing dataset `.mat` files (requires scipy):

```
python train_senseedge.py --cwru-dir /path/to/cwru/mat/files
```

Options:

| Flag | Default | Description |
|------|---------|-------------|
| `--cwru-dir` | None | Path to CWRU .mat files; omit for synthetic data |
| `--epochs` | 200 | Training epochs |
| `--batch-size` | 64 | Mini-batch size |
| `--lr` | 0.01 | Initial learning rate |
| `--samples` | 500 | Samples per class (synthetic mode) |
| `--output` | `ml/senseedge_weights.npz` | Output weight file |
| `--seed` | 42 | Random seed |

## Export to C Header

```
python export_weights.py
```

This reads `senseedge_weights.npz` and writes `firmware/nn_weights.h`.

Options:

| Flag | Default | Description |
|------|---------|-------------|
| `--input` | `ml/senseedge_weights.npz` | Input .npz weight file |
| `--output` | `firmware/nn_weights.h` | Output C header path |

## Load Weights into Hardware

```c
#include "nn_weights.h"
#include "senseedge_regs.h"

// Sequential load via Wishbone
for (int i = 0; i < 212; i++)
    reg_write(SE_NN_WEIGHTS, NN_WEIGHT(i, all_weights[i]));
```

## Weight Memory Layout

Matches `nn_engine.v` weight storage:

| Address | Content | Count |
|---------|---------|-------|
| 0..127 | Layer 1 weights (16 neurons x 8 inputs, row-major) | 128 |
| 128..143 | Layer 1 biases | 16 |
| 144..207 | Layer 2 weights (4 neurons x 16 inputs, row-major) | 64 |
| 208..211 | Layer 2 biases | 4 |
| **Total** | | **212** |

## Fault Classes

| Class | Label | Spectral Signature |
|-------|-------|--------------------|
| 0 | Healthy | Energy in low band, low total energy |
| 1 | Bearing Wear | Energy in mid-low band, high peak magnitude |
| 2 | Imbalance | Energy in mid-high band |
| 3 | Misalignment | Energy in high band |
