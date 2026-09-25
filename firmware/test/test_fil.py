#!/usr/bin/env python3
"""Firmware-in-the-loop tests: the real firmware DSP (C, float32) vs the
simulation reference (Python, float64) in the same plant.

    python3 firmware/test/test_fil.py        (builds the harness with host gcc)

Pass criteria
  * C and Python ANC agree within 0.5 dB(A) at the ear on every noise type
  * first 20 ms of the error signal match to float32 precision (same algorithm, same order)
  * C path identification reaches >= 12 dB fit (ceiling ~17 dB: 64-tap model vs longer tail) in a quiet room and < 5 % model error
  * dosimeter: 94 dB 1 kHz tone reads 94.0 +/- 0.2 dB(A); 85 dB(A) for 8 h = 100 % dose (NIOSH)
  * seal monitor: C per-band attenuation within 0.05 dB of sim/seal_ref.py every second, identical
    flag decisions; a good seal never flags, a thick-glasses-temple leak does
"""
import ctypes as C
import os
import subprocess
import sys

import numpy as np
from scipy import signal

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(FW, "..", "sim"))
from plant import Plant, FS                                # noqa: E402
from noise import industrial, scale_to_dba, spl_a          # noqa: E402
from anc_ref import run_anc, identify_paths                # noqa: E402
from run_all import TUNED                                  # noqa: E402
import seal_ref                                            # noqa: E402
from seal_monitor import leaky_plant, past_only            # noqa: E402

BUILD = os.path.join(HERE, "build")
os.makedirs(BUILD, exist_ok=True)
LIB = os.path.join(BUILD, "libfil.so")
subprocess.check_call(["gcc", "-O2", "-std=c99", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC",
                       "-I" + os.path.join(FW, "inc"), "-I" + os.path.join(FW, "dsp"),
                       os.path.join(FW, "dsp", "anc_core.c"), os.path.join(FW, "dsp", "dsp_misc.c"),
                       os.path.join(HERE, "fil_harness.c"), "-o", LIB, "-lm"])
lib = C.CDLL(LIB)
dp = np.ctypeslib.ndpointer(np.float64, flags="C")
fp = np.ctypeslib.ndpointer(np.float32, flags="C")
lib.run_fil.argtypes = [C.c_int, dp, dp, dp, dp, C.c_int, fp, fp, dp, dp, dp, dp, C.c_int, C.c_int]
lib.run_fil.restype = C.c_int
lib.run_sysid.argtypes = [C.c_int, dp, dp, dp, dp, C.c_int, C.c_float, C.c_float, fp, fp]
lib.run_sysid.restype = C.c_float
lib.run_dosi.argtypes = [C.c_int, fp, C.c_float, C.c_float, C.POINTER(C.c_double), C.POINTER(C.c_double)]
ip = np.ctypeslib.ndpointer(np.int32, flags="C")
lib.run_seal.argtypes = [C.c_int, fp, fp, fp, fp, fp, fp, ip, ip]
lib.run_seal.restype = C.c_int
lib.run_seal_learn.argtypes = [C.c_int, fp, fp, fp]
lib.run_seal_learn.restype = C.c_int
lib.run_tone.argtypes = [dp, C.c_int, C.c_float, C.c_float, dp, C.c_int]
lib.run_tone.restype = C.c_float

failures = []


def check(cond, msg):
    print(("PASS " if cond else "FAIL ") + msg)
    if not cond:
        failures.append(msg)


pl = Plant()
z = np.zeros(3 * FS)
s_hat, f_hat = identify_paths(pl, z, z, l_s=TUNED.l_s, l_f=TUNED.l_f, seconds=2.0, mu=0.01)
s32, f32 = s_hat.astype(np.float32), f_hat.astype(np.float32)

# ---- 1) closed-loop ANC: C vs Python ----
n = 12 * FS
last = slice(n - 4 * FS, n)
for kind in ["motor", "compressor", "pink"]:
    rng = np.random.default_rng(11)
    out = scale_to_dba(industrial(n, FS, rng, kind), FS, 100)
    d = signal.lfilter(pl.h_P, 1, out)
    xr = signal.lfilter(pl.h_ref, 1, out)
    rng_py = np.random.default_rng(5)
    e_py, y_py, *_ = run_anc(d, xr, pl, s32.astype(np.float64), f32.astype(np.float64), TUNED, rng_py)
    rng_n = np.random.default_rng(5)
    ne = rng_n.standard_normal(n) * 6e-4
    nx = rng_n.standard_normal(n) * 6e-4
    e_c = np.zeros(n)
    y_c = np.zeros(n)
    trips = lib.run_fil(n, d, xr, pl.h_S, pl.h_F, len(pl.h_S), s32, f32, ne, nx, e_c, y_c, 1, 1)
    a_py, a_c, a_pass = spl_a(e_py[last], FS), spl_a(e_c[last], FS), spl_a(d[last], FS)
    check(abs(a_py - a_c) < 0.5,
          f"{kind:10s} passive {a_pass:.1f} | python ANC {a_py:.1f} | firmware ANC {a_c:.1f} dB(A) | trips {trips}")
    early = slice(0, int(0.02 * FS))
    rel = np.max(np.abs(y_c[early] - y_py[early])) / (np.max(np.abs(y_py[early])) + 1e-12)
    check(rel < 1e-3, f"{kind:10s} first 20 ms output matches reference (max rel diff {rel:.1e})")

# ---- 2) path identification in C ----
s_c = np.zeros(TUNED.l_s, np.float32)
f_c = np.zeros(TUNED.l_f, np.float32)
fit = lib.run_sysid(3 * FS, z, z, pl.h_S, pl.h_F, len(pl.h_S), C.c_float(0.3 / np.linalg.norm(pl.h_S)),
                    C.c_float(0.01), s_c, f_c)
err = np.linalg.norm(s_c - pl.h_S[:TUNED.l_s]) / np.linalg.norm(pl.h_S[:TUNED.l_s])
check(fit > 12 and err < 0.05, f"sysid quiet room: fit {fit:.1f} dB, S model error {err * 100:.1f} %")

# ---- 3) dosimeter ----
t = np.arange(10 * FS) / FS
tone = (np.sqrt(2) * 1.0 * np.sin(2 * np.pi * 1000 * t)).astype(np.float32)   # 1 Pa rms = 94 dB
laeq, dose = C.c_double(), C.c_double()
lib.run_dosi(len(tone), tone, 85.0, 3.0, C.byref(laeq), C.byref(dose))
check(abs(laeq.value - 93.98) < 0.2, f"dosimeter 94 dB 1 kHz calibrator tone -> {laeq.value:.2f} dB(A)")
# 85 dB(A) for 10 s -> dose fraction 10 s / 8 h -> scale to 8 h
p85 = (tone * 10 ** ((85 - 93.98) / 20)).astype(np.float32)
lib.run_dosi(len(p85), p85, 85.0, 3.0, C.byref(laeq), C.byref(dose))
dose_8h = dose.value * (8 * 3600 / 10)
check(abs(dose_8h - 100) < 5, f"dosimeter 85 dB(A) extrapolated to 8 h -> {dose_8h:.1f} % dose (NIOSH 85/3)")
p100 = (tone * 10 ** ((100 - 93.98) / 20)).astype(np.float32)
lib.run_dosi(len(p100), p100, 85.0, 3.0, C.byref(laeq), C.byref(dose))
mins = 8 * 60 * 100 / (dose.value * 8 * 3600 / 10) if dose.value > 0 else 0
check(abs(mins - 15) < 1.5, f"dosimeter 100 dB(A): allowed time {mins:.1f} min (NIOSH says 15 min)")

# ---- 4) seal monitor ----
def seal_signals(seal, kind, seconds=12):
    """x_c and d_hat exactly as the ANC core forms them, ANC running, on a (leaky) plant."""
    nn = seconds * FS
    out = scale_to_dba(industrial(nn, FS, np.random.default_rng(3), kind), FS, 100)
    p = leaky_plant(seal)
    d = signal.lfilter(p.h_P, 1, out)
    xr = signal.lfilter(p.h_ref, 1, out)
    e, y, *_ = run_anc(d, xr, p, s_hat, f_hat, TUNED, np.random.default_rng(7))
    xc = xr + signal.lfilter(past_only(p.h_F), 1, y) - signal.lfilter(past_only(f_hat), 1, y)
    dh = e - signal.lfilter(past_only(s_hat), 1, y)
    return xc.astype(np.float32), dh.astype(np.float32)


xc0, dh0 = seal_signals(1.0, "pink")
# learn after the ANC has converged (from 4 s), filters starting cold in both implementations
base_py = seal_ref.learn_baseline(seal_ref.band_ms_per_second(xc0[4 * FS:].astype(np.float64)),
                                  seal_ref.band_ms_per_second(dh0[4 * FS:].astype(np.float64)))
base_c = np.zeros(len(seal_ref.BANDS), np.float32)
nl = lib.run_seal_learn(len(xc0) - 4 * FS, xc0[4 * FS:], dh0[4 * FS:], base_c)
check(nl == 8 and np.max(np.abs(base_c - base_py)) < 0.05,
      f"seal baseline learning: C {np.round(base_c, 2)} vs python {np.round(base_py, 2)} dB")
for seal, kind, want in ((1.0, "motor", False), (1.0, "compressor", False), (0.7, "motor", True), (0.7, "pink", True)):
    xc, dh = seal_signals(seal, kind)
    ns = len(xc) // FS
    il_c = np.zeros(ns * 4, np.float32); drop_c = np.zeros(ns, np.float32); avg_c = np.zeros(ns, np.float32)
    flag_c = np.zeros(ns, np.int32); valid_c = np.zeros(ns, np.int32)
    got = lib.run_seal(len(xc), xc, dh, base_py.astype(np.float32), il_c, drop_c, avg_c, flag_c, valid_c)
    mon = seal_ref.SealMonitor(base_py)
    bx, bd = seal_ref.band_ms_per_second(xc.astype(np.float64)), seal_ref.band_ms_per_second(dh.astype(np.float64))
    il_py, flag_py = [], []
    for k in range(ns):
        _, _, il = mon.add_second(bx[k], bd[k])
        il_py.append(il)
        flag_py.append(mon.flag)
    dil = np.max(np.abs(il_c.reshape(ns, 4) - np.array(il_py)))
    same = list(flag_c.astype(bool)) == flag_py
    check(got == ns and dil < 0.05 and same and bool(flag_c[-1]) == want,
          f"seal {seal:.1f} {kind:10s} C vs python band IL max diff {dil:.3f} dB, flags identical {same}, "
          f"loss {avg_c[-1]:.1f} dB -> {'LEAK' if flag_c[-1] else 'ok'} (expect {'LEAK' if want else 'ok'})")

# ---- 5) driver acceptance tone + lock-in ----
noise = np.random.default_rng(9).standard_normal(4 * FS) * 6e-4
for f_hz in (63.0, 1000.0):
    want = abs(np.sum(pl.h_S[1:] * np.exp(-2j * np.pi * f_hz * np.arange(1, len(pl.h_S)) / FS)))
    got = lib.run_tone(pl.h_S, len(pl.h_S), C.c_float(f_hz), C.c_float(0.5), noise, len(noise))
    check(abs(got / (0.5 * want) - 1) < 0.01,
          f"tone lock-in {f_hz:.0f} Hz: measured {got:.4f} Pa vs path gain x amplitude {0.5 * want:.4f} Pa")

print()
print("ALL PASS" if not failures else f"{len(failures)} FAILURE(S)")
sys.exit(1 if failures else 0)
