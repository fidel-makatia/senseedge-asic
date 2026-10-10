"""Integer reference models of the corrected SenseEdge RTL (branch fix/audit-defects).

Every function here is the specification the RTL is checked against, bit for bit:
  capture(code)          spi_adc_if.v     straight-binary ADC code -> signed sample
  features(mag)          feature_extract.v 32 magnitude bins -> 8 unsigned feature bytes
  nn_infer(x, params)    nn_engine.v      8 feature bytes, 212 INT8 parameters -> class, confidence
"""
import numpy as np


def wrap(v, bits):
    m = 1 << bits
    return ((np.asarray(v, np.int64) + (m >> 1)) % m) - (m >> 1)


def capture(code):
    """Sample delivered to the FFT for ADC code c in [0, 4095]."""
    return int(code) - 2048


def features(mag):
    """feature_extract.v on 32 unsigned 16-bit magnitudes.  Bands 1-4, 5-10, 11-20, 21-31;
    peak (first index of the maximum), centroid and total over bins 0-31; saturating bit
    selection to 8 bits exactly as the RTL does it."""
    mag = np.asarray(mag, np.int64)
    assert mag.shape == (32,) and mag.min() >= 0 and mag.max() <= 0xFFFF

    def sat24(v):
        v = int(v) & 0xFFFFFF
        return 255 if (v >> 16) != 0 else (v >> 8) & 0xFF

    bands = [mag[1:5].sum(), mag[5:11].sum(), mag[11:21].sum(), mag[21:32].sum()]
    peak_bin = int(np.argmax(mag))
    peak_mag = int(mag[peak_bin])
    total = int(mag.sum()) & 0xFFFFFF
    wsum = int((mag * np.arange(32)).sum()) & 0xFFFFFFFF
    return [sat24(bands[0]), sat24(bands[1]), sat24(bands[2]), sat24(bands[3]),
            (peak_bin << 3) & 0xFF, (peak_mag >> 8) & 0xFF,
            0 if total == 0 else (wsum >> 16) & 0xFF, sat24(total)]


def unpack(params):
    p = np.asarray(params, np.int64)
    assert p.shape == (212,) and p.min() >= -128 and p.max() <= 127
    return p[:128].reshape(16, 8), p[128:144], p[144:208].reshape(4, 16), p[208:212]


def nn_infer(x, params):
    """nn_engine.v: unsigned inputs, signed weights, every product signed, 16-bit wrap then
    ReLU on the hidden layer, 24-bit wrap on the outputs, first index of the maximum,
    confidence = winning accumulator clamped to [0, 255].
    Returns (class, confidence, hidden[16] after ReLU, logits[4])."""
    x = np.asarray(x, np.int64)
    assert x.shape == (8,) and x.min() >= 0 and x.max() <= 255
    W1, b1, W2, b2 = unpack(params)
    hid = np.maximum(wrap(W1 @ x + b1, 16), 0)
    out = wrap(W2 @ hid + b2, 24)
    cls = int(np.argmax(out))           # first occurrence of the maximum
    m = int(out[cls])
    conf = 0 if m < 0 else (255 if m > 255 else m)
    return cls, conf, hid, out


def to_u8(v):
    return int(v) & 0xFF
