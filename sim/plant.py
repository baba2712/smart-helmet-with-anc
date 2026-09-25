"""Acoustic + electrical plant models for the ANC helmet earcup.

Every path is built from its continuous-time frequency response (including
exact fractional delays) and converted to a causal FIR at the controller
sample rate by frequency sampling. That keeps latency modelling honest, which
is the single thing that decides whether ANC works.

Paths (per ear):
    P  primary      : outside pressure at ref mic -> pressure at error mic (through the cup)
    S  secondary    : controller output sample -> error-mic ADC sample (compute + DAC + recon + amp + driver + AA)
    F  feedback     : controller output sample -> ref-mic ADC sample (speaker leaking out through the cup)
    Ar ref chain    : outside pressure -> ref ADC sample (mic + preamp + anti-alias filter)
All pressures are in pascal; ADC-domain signals are kept in pascal-equivalent
units (i.e. the mic sensitivity and gains are normalised out), which is what the
firmware does too after calibration.
"""
from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field

FS = 32_000


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------
def butter_lp_s(f, fc, order=2):
    """Analog Butterworth low-pass evaluated at frequencies f (Hz)."""
    s = 1j * f / fc
    if order == 1:
        return 1 / (1 + s)
    if order == 2:
        return 1 / (s * s + np.sqrt(2) * s + 1)
    if order == 3:
        return 1 / ((1 + s) * (s * s + s + 1))
    raise ValueError(order)


def hp1_s(f, fc):
    s = 1j * f / fc
    return s / (1 + s)


def resonance_lp_s(f, f0, q):
    s = 1j * f / f0
    return 1 / (s * s + s / q + 1)


def delay_s(f, tau):
    return np.exp(-2j * np.pi * f * tau)


def zoh_s(f, fs=FS):
    """Zero-order hold of the DAC: sinc magnitude + half-sample delay."""
    x = f / fs
    return np.sinc(x) * np.exp(-1j * np.pi * x)


def min_phase_from_mag_db(f, mag_db, n_fft=8192):
    """Minimum-phase complex response from a magnitude table (real cepstrum)."""
    grid = np.linspace(0, FS / 2, n_fft // 2 + 1)
    m = np.interp(grid, f, mag_db)
    logmag = m / 20 * np.log(10)
    full = np.concatenate([logmag, logmag[-2:0:-1]])
    cep = np.fft.ifft(full).real
    fold = np.zeros_like(cep)
    fold[0] = cep[0]
    fold[1:n_fft // 2] = 2 * cep[1:n_fft // 2]
    fold[n_fft // 2] = cep[n_fft // 2]
    H = np.exp(np.fft.fft(fold))[: n_fft // 2 + 1]
    return grid, H


def fir_from_response(H_func, n_taps=256, n_fft=8192, window=True):
    """Causal FIR (at FS) whose response matches H_func(f) up to Nyquist."""
    f = np.linspace(0, FS / 2, n_fft // 2 + 1)
    H = H_func(f)
    H[-1] = H[-1].real  # Nyquist bin must be real
    h = np.fft.irfft(H, n_fft)[:n_taps]
    if window:
        # gentle half-Hann on the tail only so the head (delay) is untouched
        tail = n_taps // 4
        w = np.ones(n_taps)
        w[-tail:] = 0.5 * (1 + np.cos(np.linspace(0, np.pi, tail)))
        h = h * w
    return h


# --------------------------------------------------------------------------
# earmuff passive insertion loss (typical SNR~27 dB helmet-mount earmuff)
# --------------------------------------------------------------------------
PASSIVE_IL_F = np.array([0, 31.5, 63, 125, 250, 500, 1000, 2000, 4000, 8000, 16000])
PASSIVE_IL_DB = np.array([3, 4, 6, 10, 18, 27, 32, 35, 38, 38, 38])


@dataclass
class PlantConfig:
    fs: int = FS
    compute_delay_samples: float = 1.0      # sample in -> DAC write happens next sample edge
    aa_fc: float = 10_000.0                 # anti-alias corner (Hz), 3rd order total
    recon_fc: float = 9_000.0               # DAC reconstruction corner (Hz), 2nd order
    driver_f0: float = 900.0                # driver+cup resonance (Hz)
    driver_q: float = 1.2
    leak_hp: float = 35.0                   # cushion leak high-pass (Hz)
    tau_driver_to_err: float = 30e-6        # ~1 cm
    tau_ref_to_ear: float = 70e-6           # outside ref mic to ear through shell (~2.4 cm)
    passive_il_scale: float = 1.0           # 1.0 = nominal cup seal; <1 = worse seal
    feedback_extra_db: float = 6.0          # extra loss speaker -> outside ref mic
    extra_delay_samples: float = 0.0        # for robustness sweeps
    n_taps: int = 256


@dataclass
class Plant:
    cfg: PlantConfig = field(default_factory=PlantConfig)

    def __post_init__(self):
        c = self.cfg
        il_db = -PASSIVE_IL_DB * c.passive_il_scale
        g, Hcup = min_phase_from_mag_db(PASSIVE_IL_F, il_db)
        self._cup = lambda f: np.interp(f, g, Hcup.real) + 1j * np.interp(f, g, Hcup.imag)

        aa = lambda f: butter_lp_s(f, c.aa_fc, 3)
        ref_chain = lambda f: aa(f) * hp1_s(f, 10.0)          # mic LF corner ~10 Hz
        err_chain = ref_chain
        out_chain = lambda f: (delay_s(f, (c.compute_delay_samples + c.extra_delay_samples) / c.fs)
                               * zoh_s(f, c.fs) * butter_lp_s(f, c.recon_fc, 2))
        driver = lambda f: hp1_s(f, c.leak_hp) * resonance_lp_s(f, c.driver_f0, c.driver_q)

        self.h_ref = fir_from_response(ref_chain, c.n_taps)
        self.h_P = fir_from_response(lambda f: delay_s(f, c.tau_ref_to_ear) * self._cup(f) * err_chain(f), c.n_taps)
        # normalise so controller output units are "Pa at the ear at 200 Hz" -
        # the firmware applies the same calibration (DAC counts -> Pa via driver sensitivity)
        s_raw = lambda f: out_chain(f) * driver(f) * delay_s(f, c.tau_driver_to_err) * err_chain(f)
        self.out_gain = 1.0 / abs(s_raw(np.array([200.0]))[0])
        g_out = self.out_gain
        self.h_S = fir_from_response(lambda f: g_out * s_raw(f), c.n_taps)
        fb_gain = 10 ** (-c.feedback_extra_db / 20)
        self.h_F = fir_from_response(lambda f: g_out * fb_gain * out_chain(f) * driver(f) * self._cup(f)
                                     * delay_s(f, c.tau_driver_to_err) * ref_chain(f), c.n_taps)
        # "open ear" reference: what the ear would get with no helmet at all
        self.h_open = fir_from_response(lambda f: delay_s(f, c.tau_ref_to_ear) * err_chain(f), c.n_taps)

    # group delay at low frequency (samples) - useful for reports
    def s_delay_samples(self):
        f = np.linspace(20, 500, 64)
        H = np.fft.rfft(self.h_S, 8192)
        fg = np.linspace(0, FS / 2, len(H))
        ph = np.unwrap(np.angle(H))
        gd = -np.gradient(ph, 2 * np.pi * fg) * FS
        return float(np.interp(200.0, fg, gd))
