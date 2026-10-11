"""Integer reference models of the corrected SenseEdge RTL (branch fix/audit-defects).

Every function here is the specification the RTL is checked against, bit for bit:
  capture(code)          spi_adc_if.v     straight-binary ADC code -> signed sample
  fft64(samples)         fft_engine.v     64 signed samples -> 32 unsigned 16-bit magnitudes
  features(mag)          feature_extract.v 32 magnitude bins -> 8 unsigned feature bytes
  nn_infer(x, params)    nn_engine.v      8 feature bytes, 212 INT8 parameters -> class, confidence
  chain(codes, params)   the whole macro: 64 ADC codes -> bins, features, class, confidence
  alarm_model(decisions, threshold, n)  alarm_logic.v over a run of decisions
fft64 is checked against the RTL by check_fft_model.py (random, tone, corner-case and
recorded-vibration windows); the others by run_audit.py.
"""
import numpy as np


def wrap(v, bits):
    m = 1 << bits
    return ((np.asarray(v, np.int64) + (m >> 1)) % m) - (m >> 1)


def capture(code):
    """Sample delivered to the FFT for ADC code c in [0, 4095]."""
    return int(code) - 2048


# Q1.14 twiddles of fft_engine.v: W(k,64) = cos(2 pi k/64) - j sin(2 pi k/64), scaled by 16384
TW_RE = [16384, 16305, 16069, 15679, 15137, 14449, 13623, 12665, 11585, 10394, 9102, 7723, 6270,
         4756, 3196, 1608, 0, -1608, -3196, -4756, -6270, -7723, -9102, -10394, -11585, -12665,
         -13623, -14449, -15137, -15679, -16069, -16305]
TW_IM = [0, -1608, -3196, -4756, -6270, -7723, -9102, -10394, -11585, -12665, -13623, -14449,
         -15137, -15679, -16069, -16305, -16384, -16305, -16069, -15679, -15137, -14449, -13623,
         -12665, -11585, -10394, -9102, -7723, -6270, -4756, -3196, -1608]


def bitrev6(i):
    return int(f"{i:06b}"[::-1], 2)


def fft64(samples):
    """fft_engine.v, bit for bit: 64 signed 16-bit samples (sample_buf) -> 32 magnitudes.
    Load in bit-reversed order into a 24-bit store; six radix-2 DIT stages with one
    time-multiplexed butterfly; the 40-bit complex product is taken as bits [37:14] (an
    arithmetic shift by 14 and a 24-bit wrap); sums and differences wrap at 24 bits; the
    magnitude is max(|re|,|im|) + (min(|re|,|im|) >> 1) on 16 bits, saturated to 0xFFFF when
    the larger part has any of bits 23:16 set, and otherwise wrapped at 16 bits."""
    x = [int(s) for s in samples]
    assert len(x) == 64 and min(x) >= -32768 and max(x) <= 32767
    re = [x[bitrev6(i)] for i in range(64)]
    im = [0] * 64
    for s in range(6):
        half = 1 << s
        for grp in range(64 >> (s + 1)):
            for j in range(half):
                p = grp * 2 * half + j
                q = p + half
                wr, wi = TW_RE[j * (32 >> s)], TW_IM[j * (32 >> s)]
                br, bi = re[q], im[q]
                tr = int(wrap((br * wr - bi * wi) >> 14, 24))
                ti = int(wrap((br * wi + bi * wr) >> 14, 24))
                ar, ai = re[p], im[p]
                re[p], im[p] = int(wrap(ar + tr, 24)), int(wrap(ai + ti, 24))
                re[q], im[q] = int(wrap(ar - tr, 24)), int(wrap(ai - ti, 24))
    mags = []
    for k in range(32):
        a = (-re[k]) & 0xFFFFFF if re[k] < 0 else re[k]
        b = (-im[k]) & 0xFFFFFF if im[k] < 0 else im[k]
        mx, mn = (a, b) if a > b else (b, a)
        mags.append(0xFFFF if (mx >> 16) != 0 else (mx + ((mn & 0xFFFF) >> 1)) & 0xFFFF)
    return mags


def chain(codes, params=None):
    """The macro on one window: 64 ADC codes -> (bins[32], features[8], class, confidence).
    class and confidence are None when params is None."""
    mags = fft64([capture(c) for c in codes])
    feat = features(mags)
    if params is None:
        return mags, feat, None, None
    cls, conf, _, _ = nn_infer(feat, params)
    return mags, feat, cls, conf


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


def alarm_model(decisions, threshold, n_consec):
    """alarm_logic.v over a sequence of (class, confidence) decisions after a reset: returns
    the alarm_active flag after each decision.  A decision counts as a fault when its class is
    not 0 and its confidence reaches the threshold; the alarm asserts on the fault that finds
    the consecutive counter already at n_consec (the (n_consec+1)-th in a row), the counter
    saturates at 15, any other decision clears the counter, and a healthy decision at or above
    the threshold clears the alarm."""
    count, alarm, out = 0, 0, []
    for cls, conf in decisions:
        if cls != 0 and conf >= threshold:
            if count >= n_consec and not alarm:
                alarm = 1
            count = min(count + 1, 15)
        else:
            count = 0
            if cls == 0 and conf >= threshold:
                alarm = 0
        out.append(alarm)
    return out


def to_u8(v):
    return int(v) & 0xFF
