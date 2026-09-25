"""Reference (float64, numba) implementation of the helmet's hybrid FxLMS controller.

This is the *specification* the firmware DSP core (firmware/dsp/anc_core.c) is
tested against. Keep the two in lock-step: same state, same update order.

Controller per ear, per sample n:
    x_c    = x_adc - F_hat * y          (remove speaker leakage from the reference mic)
    d_hat  = e_adc - S_hat * y          (IMC: estimate of the disturbance at the ear)
    y      = W_ff * x_c + W_fb * d_hat  (feedforward + adaptive feedback)
    y      = soft_limit(y)
    x'     = S_hat * x_c,   d' = S_hat * d_hat          (filtered references)
    W_ff  <- (1-g) W_ff - mu_ff * e * x'  / (eps + |x'|^2)
    W_fb  <- (1-g) W_fb - mu_fb * e * d'  / (eps + |d'|^2)
"""
from __future__ import annotations

import numpy as np
from numba import njit
from dataclasses import dataclass


@dataclass
class AncParams:
    l_ff: int = 128          # feedforward taps
    l_fb: int = 64           # feedback taps
    l_s: int = 64            # secondary path model taps
    l_f: int = 128           # speaker->ref-mic leakage model taps
    w_hp_hz: float = 150.0   # error/filtered-ref weighting: low-shelf corner (0 = off)
    w_floor_db: float = -20.0  # weighting gain below the corner (never -inf: LF must stay observable)
    ref_hp_hz: float = 30.0  # 2nd-order HP on x_c and d_hat: never chase infrasound the driver can't make
    clip_trip: float = 0.05  # watchdog: fraction of clipped output samples (100 ms window) that trips
    mu_ff: float = 0.004     # normalised step sizes
    mu_fb: float = 0.002
    leak: float = 1e-5       # leakage per sample
    eps: float = 1e-6        # NLMS regulariser (Pa^2 * taps)
    y_max: float = 20.0      # output limit in Pa-at-ear equivalent (~120 dB peak)
    wd_ratio_db: float = 6.0 # watchdog: e power this much above d_hat power ...
    wd_hold_ms: float = 200  # ... for this long => reset + passive
    use_ff: bool = True
    use_fb: bool = True


@njit(cache=True)
def _run(d, xr, h_S, h_F, s_hat, f_hat, noise_e, noise_x,
         l_ff, l_fb, mu_ff, mu_fb, leak, eps, y_max, use_ff, use_fb,
         wd_ratio, wd_hold, adapt_start, wb0, wb1, wa1, clip_trip, hb, ha):
    n = d.shape[0]
    Ls = s_hat.shape[0]
    Lf = f_hat.shape[0]
    Lp = h_S.shape[0]
    # weighting filter states (1st order: y = b0 x + b1 x1 - a1 y1)
    we_x1 = 0.0
    we_y1 = 0.0
    wx_x1 = 0.0
    wx_y1 = 0.0
    wd_x1 = 0.0
    wd_y1 = 0.0
    e_out = np.zeros(n)
    y_out = np.zeros(n)
    trips = 0
    ybuf = np.zeros(Lp)       # ybuf[k] = y[n-1-k]  (plant history, previous outputs)
    xbuf = np.zeros(max(l_ff, Ls))
    dbuf = np.zeros(max(l_fb, Ls))
    xfbuf = np.zeros(l_ff)
    dfbuf = np.zeros(l_fb)
    w_ff = np.zeros(l_ff)
    w_fb = np.zeros(l_fb)
    p_xf = 0.0
    p_df = 0.0
    pe = 1e-12
    pd = 1e-12
    bad = 0
    clipf = 0.0
    hx = np.zeros(4)   # biquad DF1 states for the reference high-pass: x1 x2 y1 y2
    hd = np.zeros(4)
    a_s = 1.0 / (0.02 * 32000)   # 20 ms power smoothers
    a_c = 1.0 / (0.1 * 32000)    # 100 ms clip-rate smoother
    for i in range(n):
        # ---- plant: what the mics see this sample (depends only on past outputs) ----
        sy = 0.0
        fy = 0.0
        for k in range(Lp - 1):
            sy += h_S[k + 1] * ybuf[k]
            fy += h_F[k + 1] * ybuf[k]
        e = d[i] + sy + noise_e[i]
        x = xr[i] + fy + noise_x[i]
        # ---- controller ----
        sh = 0.0
        fh = 0.0
        for k in range(Ls - 1):
            sh += s_hat[k + 1] * ybuf[k]
        for k in range(Lf - 1):
            fh += f_hat[k + 1] * ybuf[k]
        xc_raw = x - fh
        dh = e - sh            # un-filtered d_hat for the watchdog
        xc = hb[0] * xc_raw + hb[1] * hx[0] + hb[2] * hx[1] - ha[1] * hx[2] - ha[2] * hx[3]
        hx[1] = hx[0]
        hx[0] = xc_raw
        hx[3] = hx[2]
        hx[2] = xc
        dhf = hb[0] * dh + hb[1] * hd[0] + hb[2] * hd[1] - ha[1] * hd[2] - ha[2] * hd[3]
        hd[1] = hd[0]
        hd[0] = dh
        hd[3] = hd[2]
        hd[2] = dhf
        # shift reference buffers
        for k in range(xbuf.shape[0] - 1, 0, -1):
            xbuf[k] = xbuf[k - 1]
        xbuf[0] = xc
        for k in range(dbuf.shape[0] - 1, 0, -1):
            dbuf[k] = dbuf[k - 1]
        dbuf[0] = dhf
        y = 0.0
        if use_ff:
            for k in range(l_ff):
                y += w_ff[k] * xbuf[k]
        if use_fb:
            for k in range(l_fb):
                y += w_fb[k] * dbuf[k]
        # soft limiter (tanh-like, cheap rational form, identical in C)
        clipped = 0.0
        if y > y_max or y < -y_max:
            y = y_max if y > 0 else -y_max
            clipped = 1.0
        clipf += a_c * (clipped - clipf)
        # filtered references
        xf = 0.0
        df = 0.0
        for k in range(Ls):
            xf += s_hat[k] * xbuf[k]
            df += s_hat[k] * dbuf[k]
        # frequency weighting of the filtered references (same filter as on e)
        t = wb0 * xf + wb1 * wx_x1 - wa1 * wx_y1
        wx_x1 = xf
        wx_y1 = t
        xf = t
        t = wb0 * df + wb1 * wd_x1 - wa1 * wd_y1
        wd_x1 = df
        wd_y1 = t
        df = t
        p_xf += xf * xf - xfbuf[l_ff - 1] * xfbuf[l_ff - 1]
        p_df += df * df - dfbuf[l_fb - 1] * dfbuf[l_fb - 1]
        if p_xf < 0.0:
            p_xf = 0.0
        if p_df < 0.0:
            p_df = 0.0
        for k in range(l_ff - 1, 0, -1):
            xfbuf[k] = xfbuf[k - 1]
        xfbuf[0] = xf
        for k in range(l_fb - 1, 0, -1):
            dfbuf[k] = dfbuf[k - 1]
        dfbuf[0] = df
        # weighted error
        ew = wb0 * e + wb1 * we_x1 - wa1 * we_y1
        we_x1 = e
        we_y1 = ew
        # adaptation
        if i >= adapt_start:
            if use_ff:
                g = mu_ff * ew / (eps + p_xf)
                for k in range(l_ff):
                    w_ff[k] = w_ff[k] * (1.0 - leak) - g * xfbuf[k]
            if use_fb:
                g = mu_fb * ew / (eps + p_df)
                for k in range(l_fb):
                    w_fb[k] = w_fb[k] * (1.0 - leak) - g * dfbuf[k]
        # divergence watchdog
        pe += a_s * (e * e - pe)
        pd += a_s * (dh * dh - pd)
        if pe > wd_ratio * pd:
            bad += 1
        else:
            bad = 0
        if bad > wd_hold or clipf > clip_trip:
            w_ff[:] = 0.0
            w_fb[:] = 0.0
            bad = 0
            clipf = 0.0
            trips += 1
        # plant output history
        for k in range(Lp - 1, 0, -1):
            ybuf[k] = ybuf[k - 1]
        ybuf[0] = y
        e_out[i] = e
        y_out[i] = y
    return e_out, y_out, w_ff, w_fb, trips


def run_anc(d, xr, plant, s_hat, f_hat, p: AncParams, rng, sensor_noise_pa=6e-4,
            adapt_start=0, fs=32000):
    """d  : primary disturbance at the error mic (Pa, already through P and err chain)
    xr : outside noise as seen by the ref ADC (Pa, through ref chain)
    sensor_noise_pa: rms self-noise of mic+preamp (6e-4 Pa ~ 30 dB SPL)"""
    n = len(d)
    ne = rng.standard_normal(n) * sensor_noise_pa
    nx = rng.standard_normal(n) * sensor_noise_pa
    wd_ratio = 10 ** (p.wd_ratio_db / 10)
    wd_hold = int(p.wd_hold_ms * 1e-3 * fs)
    wb0, wb1, wa1 = weighting_coeffs(p.w_hp_hz, fs, p.w_floor_db)
    hb, ha = ref_hp_coeffs(p.ref_hp_hz, fs)
    return _run(d.astype(np.float64), xr.astype(np.float64), plant.h_S, plant.h_F,
                s_hat.astype(np.float64), f_hat.astype(np.float64), ne, nx,
                p.l_ff, p.l_fb, p.mu_ff, p.mu_fb, p.leak, p.eps, p.y_max,
                p.use_ff, p.use_fb, wd_ratio, wd_hold, adapt_start, wb0, wb1, wa1, p.clip_trip, hb, ha)


def ref_hp_coeffs(fc, fs=32000):
    """2nd-order Butterworth high-pass biquad (b[3], a[3] with a[0]=1)."""
    from scipy import signal
    if fc <= 0:
        return np.array([1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])
    b, a = signal.butter(2, fc, 'highpass', fs=fs)
    return b.astype(np.float64), a.astype(np.float64)


def weighting_coeffs(fc, fs=32000, floor_db=-20.0):
    """1st-order low-shelf (bilinear, pre-warped): gain 10^(floor/20) at DC, 1 above fc.
    De-emphasises the huge LF tones in the gradient so mid-band noise is not traded
    away for a fraction of a dB at 50 Hz. fc=0 -> pass-through."""
    if fc <= 0:
        return 1.0, 0.0, 0.0
    a = 10 ** (floor_db / 20)
    wc = 2 * np.pi * fc
    K = wc / np.tan(wc / (2 * fs))
    den = K + wc
    return (K + a * wc) / den, (a * wc - K) / den, (wc - K) / den


@njit(cache=True)
def _identify(d, xr, h_S, h_F, probe, s0, f0, mu):
    """Online NLMS identification of S and F with a white probe, ambient noise present.
    Starts from s0/f0 (zeros, or the factory calibration stored in flash)."""
    n = probe.shape[0]
    Lp = h_S.shape[0]
    Ls = s0.shape[0]
    Lf = f0.shape[0]
    L = max(Ls, Lf)
    pbuf = np.zeros(max(L, Lp))
    s = s0.copy()
    f = f0.copy()
    for i in range(n):
        for k in range(pbuf.shape[0] - 1, 0, -1):
            pbuf[k] = pbuf[k - 1]
        pbuf[0] = probe[i]
        e = d[i]
        x = xr[i]
        for k in range(1, Lp):
            e += h_S[k] * pbuf[k]
            x += h_F[k] * pbuf[k]
        es = e
        ef = x
        ps = 1e-9
        pf = 1e-9
        for k in range(Ls):
            es -= s[k] * pbuf[k]
            ps += pbuf[k] * pbuf[k]
        for k in range(Lf):
            ef -= f[k] * pbuf[k]
            pf += pbuf[k] * pbuf[k]
        g1 = mu * es / ps
        g2 = mu * ef / pf
        for k in range(Ls):
            s[k] += g1 * pbuf[k]
        for k in range(Lf):
            f[k] += g2 * pbuf[k]
    return s, f


def probe_gain(plant):
    """Output-units gain so a unit-variance white probe gives 1 Pa rms at the ear."""
    return 1.0 / np.linalg.norm(plant.h_S)


def identify_paths(plant, d, xr, l_s=64, l_f=128, probe_pa=0.3, seconds=2.0, mu=0.01,
                   fs=32000, rng=None, s0=None, f0=None):
    """Secondary/leakage path ID as the firmware does it.
    probe_pa: rms probe level at the ear (0.3 Pa ~ 83 dB SPL)."""
    rng = rng or np.random.default_rng(1)
    n = int(seconds * fs)
    probe = rng.standard_normal(n) * probe_pa * probe_gain(plant)
    s0 = np.zeros(l_s) if s0 is None else s0.astype(np.float64)
    f0 = np.zeros(l_f) if f0 is None else f0.astype(np.float64)
    return _identify(d[:n].astype(np.float64), xr[:n].astype(np.float64), plant.h_S, plant.h_F,
                     probe, s0, f0, mu)
