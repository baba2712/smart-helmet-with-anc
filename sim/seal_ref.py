"""Reference implementation of the in-use seal monitor (the firmware's
dsp_misc.c seal_* functions are tested against this: same filters, same
per-second arithmetic, same decision logic).

Per ear, every sample (ISR):
    x_c   = x - F_hat*y      outside noise, speaker leakage removed   (from the ANC core)
    d_hat = e - S_hat*y      noise that got through the cup, ANC removed
    octave bands 250, 500, 1000, 2000 Hz (2nd-order Butterworth band-pass, 2 biquads)
    accumulate band mean squares of x_c and d_hat over 1 s

Every second (main loop):
    IL_b   = L(x_c, b) - L(d_hat, b)              per-band passive attenuation (dB)
    active = bands with L(x_c, b) >= max_b L(x_c) - GATE_DB  and  >= MIN_BAND_DB
    drop   = mean over active bands of (baseline_b - IL_b)
    drop_avg <- drop_avg + (drop - drop_avg) / AVG_S          (only on valid seconds)
    flag  after HOLD_S consecutive valid seconds with drop_avg >= FLAG_DB,
    clear when drop_avg < FLAG_DB - HYST_DB

Per-band attenuation is a property of the cup and its seal, not of the noise,
so one factory baseline (reference head / fixture, any broadband noise) holds
for every noise - and a bad fit is caught at power-on, not only a change.
"""
from __future__ import annotations

import numpy as np
from scipy import signal

FS = 32_000
BANDS = (250.0, 500.0, 1000.0, 2000.0)
GATE_DB = 15.0        # a band counts if its outside level is within this of the loudest band
MIN_BAND_DB = 55.0    # ... and loud enough that the in-cup band is above the mic/ADC floor
FLAG_DB = 3.0
HYST_DB = 1.0
AVG_S = 4.0
HOLD_S = 3


def band_sos():
    """(len(BANDS), 2, 6) second-order sections, one 2nd-order band-pass per octave band."""
    return np.array([signal.butter(2, [b / np.sqrt(2), b * np.sqrt(2)], "bandpass", fs=FS, output="sos")
                     for b in BANDS])


def db(ms):
    return 10.0 * np.log10(np.asarray(ms) / 4e-10 + 1e-12)


def band_ms_per_second(x, sos=None):
    """(seconds, bands) mean square of x in each band over consecutive 1 s blocks."""
    sos = band_sos() if sos is None else sos
    n = len(x) // FS * FS
    out = []
    for s in sos:
        y = signal.sosfilt(s, x[:n])
        out.append(np.mean(y.reshape(-1, FS) ** 2, axis=1))
    return np.array(out).T


def per_second(ms_x, ms_d, base):
    """One second of the decision arithmetic. Returns (valid, drop, il[bands])."""
    lx, ld = db(ms_x), db(ms_d)
    il = lx - ld
    active = (lx >= lx.max() - GATE_DB) & (lx >= MIN_BAND_DB)
    if not active.any():
        return False, 0.0, il
    return True, float(np.mean((np.asarray(base) - il)[active])), il


class SealMonitor:
    def __init__(self, base):
        self.base = np.asarray(base, float)
        self.avg = 0.0
        self.n_valid = 0
        self.run = 0
        self.flag = False

    def add_second(self, ms_x, ms_d):
        valid, drop, il = per_second(ms_x, ms_d, self.base)
        if valid:
            self.avg = drop if self.n_valid == 0 else self.avg + (drop - self.avg) / AVG_S
            self.n_valid += 1
            self.run = self.run + 1 if self.avg >= FLAG_DB else 0
            if self.run >= HOLD_S:
                self.flag = True
            elif self.avg < FLAG_DB - HYST_DB:
                self.flag = False
        return valid, drop, il


def learn_baseline(ms_x_seconds, ms_d_seconds):
    """Factory baseline: energy-mean per-band attenuation over the learning seconds."""
    return db(np.mean(ms_x_seconds, axis=0)) - db(np.mean(ms_d_seconds, axis=0))
