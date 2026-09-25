"""Noise scenarios and acoustic level helpers (A-weighting, SPL, 1/3 octave)."""
from __future__ import annotations

import numpy as np
from scipy import signal

P_REF = 20e-6  # Pa


def a_weighting_sos(fs):
    """IEC 61672 A-weighting as a digital SOS filter (bilinear, pre-warped poles)."""
    f1, f2, f3, f4 = 20.598997, 107.65265, 737.86223, 12194.217
    a1000 = 1.9997
    z = [0, 0, 0, 0]
    p = [-2 * np.pi * f1] * 2 + [-2 * np.pi * f2, -2 * np.pi * f3] + [-2 * np.pi * f4] * 2
    k = (2 * np.pi * f4) ** 2 * 10 ** (a1000 / 20)
    zd, pd, kd = signal.bilinear_zpk(z, p, k, fs)
    return signal.zpk2sos(zd, pd, kd)


def spl(x):
    return 20 * np.log10(np.sqrt(np.mean(x ** 2)) / P_REF + 1e-30)


def spl_a(x, fs):
    return spl(signal.sosfilt(a_weighting_sos(fs), x))


def scale_to_dba(x, fs, target_dba):
    return x * 10 ** ((target_dba - spl_a(x, fs)) / 20)


def pink(n, rng):
    X = np.fft.rfft(rng.standard_normal(n))
    f = np.arange(len(X))
    f[0] = 1
    X /= np.sqrt(f)
    y = np.fft.irfft(X, n)
    return y / np.std(y)


def industrial(n, fs, rng, kind="motor"):
    """Synthetic industrial noise.

    motor      : 2-pole induction motor + fan + transformer hum over pink bed (pulp-mill drive, pump house)
    compressor : strong low-frequency periodic content + broadband
    pink       : pure broadband pink noise (worst case for ANC)
    """
    t = np.arange(n) / fs
    bed = pink(n, rng)
    if kind == "pink":
        return bed
    if kind == "motor":
        f_rot = 49.4                     # 2-pole motor at rated slip
        tones = [(f_rot, 1.0), (2 * f_rot, 0.8), (100.0, 1.2), (200.0, 0.5), (300.0, 0.3),
                 (6 * f_rot * 1.0, 0.9),  # 6-blade fan blade-pass ~296 Hz
                 (12 * f_rot, 0.35)]
        tonal = np.zeros(n)
        for f, a in tones:
            # slow frequency wander, like a real machine under load
            wander = 1 + 0.0005 * np.sin(2 * np.pi * 0.1 * t + rng.uniform(0, 6))  # +-0.05 % slip wander
            ph = 2 * np.pi * np.cumsum(f * wander) / fs + rng.uniform(0, 2 * np.pi)
            tonal += a * np.sin(ph)
        tonal /= np.std(tonal)
        return 1.3 * tonal + 0.7 * bed
    if kind == "compressor":
        f0 = 24.5
        tonal = sum((1 / k) * np.sin(2 * np.pi * k * f0 * t + rng.uniform(0, 6)) for k in range(1, 30))
        tonal /= np.std(tonal)
        return tonal + 0.6 * bed
    raise ValueError(kind)


THIRD_OCT = np.array([25, 31.5, 40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500, 630,
                      800, 1000, 1250, 1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000, 10000, 12500])


def third_octave_levels(x, fs):
    f, pxx = signal.welch(x, fs, nperseg=16384)
    out = []
    for fc in THIRD_OCT:
        lo, hi = fc / 2 ** (1 / 6), fc * 2 ** (1 / 6)
        m = (f >= lo) & (f < hi)
        pw = np.trapezoid(pxx[m], f[m]) if m.any() else 0.0
        out.append(10 * np.log10(pw / P_REF ** 2 + 1e-30))
    return np.array(out)
