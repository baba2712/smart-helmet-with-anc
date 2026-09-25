#!/usr/bin/env python3
"""Run the full ANC helmet simulation study.

Outputs (sim/results/):
    paths.png            plant responses (P, S, F, passive IL)
    spectra_<noise>.png  1/3-octave levels: open ear / passive / passive+ANC
    convergence.png      dB(A) at the ear vs time after switch-on
    latency_sweep.png    ANC attenuation vs extra latency
    robustness.png       ANC attenuation vs model error / seal quality
    summary.md, summary.json

Also exports the tuned parameters to firmware/inc/anc_tuning.h and the
factory-calibrated path models + plant to firmware/test/vectors/ for the
firmware-in-the-loop test.

usage: python3 run_all.py [--quick] [--wav recording.wav]
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import asdict, replace

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import signal

sys.path.insert(0, os.path.dirname(__file__))
from plant import Plant, PlantConfig, FS, PASSIVE_IL_F, PASSIVE_IL_DB  # noqa: E402
from noise import industrial, scale_to_dba, spl_a, spl, third_octave_levels, THIRD_OCT  # noqa: E402
from anc_ref import AncParams, run_anc, identify_paths, weighting_coeffs, ref_hp_coeffs  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "results")
FW = os.path.join(HERE, "..", "firmware")

# ---- the tuned parameter set (single source of truth, exported to firmware) ----
TUNED = AncParams(l_ff=128, l_fb=64, l_s=64, l_f=128, mu_ff=0.001, mu_fb=0.0005, leak=1e-5,
                  eps=1e-6, y_max=10.0, w_hp_hz=300.0, w_floor_db=-20.0, ref_hp_hz=30.0)
AMBIENT_DBA = 100.0

plt.rcParams.update({"figure.dpi": 110, "axes.grid": True, "grid.alpha": 0.3, "font.size": 9})
C_OPEN, C_PASS, C_ANC = "#8c8c8c", "#3b75af", "#d1495b"


def load_wav(path, n):
    import soundfile as sf
    x, fs = sf.read(path, always_2d=True)
    x = x[:, 0]
    if fs != FS:
        x = signal.resample_poly(x, FS, fs)
    reps = int(np.ceil(n / len(x)))
    return np.tile(x, reps)[:n]


def scenario(pl, kind, n, rng, wav=None):
    out = load_wav(wav, n) if kind == "recorded" else industrial(n, FS, rng, kind)
    out = scale_to_dba(out, FS, AMBIENT_DBA)
    d = signal.lfilter(pl.h_P, 1, out)
    xr = signal.lfilter(pl.h_ref, 1, out)
    op = signal.lfilter(pl.h_open, 1, out)
    return out, d, xr, op


def factory_cal(pl, p=TUNED):
    z = np.zeros(3 * FS)
    return identify_paths(pl, z, z, l_s=p.l_s, l_f=p.l_f, seconds=2.0, mu=0.01)


def fig_paths(pl):
    f = np.fft.rfftfreq(8192, 1 / FS)
    fig, ax = plt.subplots(2, 1, figsize=(7, 5.5), sharex=True)
    for h, lab, c in [(pl.h_S, "S: controller -> error mic", C_ANC), (pl.h_P, "P: outside -> ear (passive cup)", C_PASS),
                      (pl.h_F, "F: speaker -> outside ref mic", "#6a994e")]:
        H = np.fft.rfft(h, 8192)
        ax[0].semilogx(f[1:], 20 * np.log10(np.abs(H[1:]) + 1e-12), label=lab, color=c)
        ax[1].semilogx(f[1:], np.unwrap(np.angle(H[1:])) * 180 / np.pi, color=c)
    ax[0].semilogx(PASSIVE_IL_F[1:], -PASSIVE_IL_DB[1:], "k--", lw=0.8, label="earmuff IL table")
    ax[0].set_ylabel("magnitude (dB)")
    ax[0].set_ylim(-60, 15)
    ax[0].legend(fontsize=8)
    ax[1].set_ylabel("phase (deg)")
    ax[1].set_xlabel("frequency (Hz)")
    ax[1].set_xlim(20, 16000)
    fig.suptitle(f"Plant models (S low-frequency group delay = {pl.s_delay_samples():.1f} samples @ {FS} Hz)")
    fig.tight_layout()
    fig.savefig(os.path.join(RES, "paths.png"))
    plt.close(fig)


def run_case(pl, s, f, kind, n, p=TUNED, seed=7, wav=None):
    rng = np.random.default_rng(seed)
    out, d, xr, op = scenario(pl, kind, n, rng, wav)
    e, y, wff, wfb, trips = run_anc(d, xr, pl, s, f, p, rng)
    last = slice(n - 5 * FS, n)
    return dict(kind=kind, open_dba=spl_a(op[last], FS), passive_dba=spl_a(d[last], FS),
                anc_dba=spl_a(e[last], FS), open_dbz=spl(op[last]), passive_dbz=spl(d[last]),
                anc_dbz=spl(e[last]), trips=int(trips), y_rms=float(np.std(y[last])),
                third_open=third_octave_levels(op[last], FS), third_pass=third_octave_levels(d[last], FS),
                third_anc=third_octave_levels(e[last], FS), e=e, d=d)


def fig_spectrum(r):
    fig, ax = plt.subplots(figsize=(7.5, 3.6))
    x = np.arange(len(THIRD_OCT))
    ax.plot(x, r["third_open"], "o-", color=C_OPEN, ms=3, label=f"open ear  {r['open_dba']:.1f} dB(A)")
    ax.plot(x, r["third_pass"], "o-", color=C_PASS, ms=3, label=f"passive cup  {r['passive_dba']:.1f} dB(A)")
    ax.plot(x, r["third_anc"], "o-", color=C_ANC, ms=3, label=f"passive + ANC  {r['anc_dba']:.1f} dB(A)")
    ax.set_xticks(x[::2], [f"{v:g}" for v in THIRD_OCT[::2]], rotation=45)
    ax.set_xlabel("1/3-octave band (Hz)")
    ax.set_ylabel("SPL (dB re 20 µPa)")
    ax.set_title(f"'{r['kind']}' noise at {AMBIENT_DBA:.0f} dB(A) ambient")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(RES, f"spectra_{r['kind']}.png"))
    plt.close(fig)


def fig_convergence(results):
    fig, ax = plt.subplots(figsize=(7, 3.4))
    blk = FS // 4
    aw = None
    from noise import a_weighting_sos
    sos = a_weighting_sos(FS)
    for r, c in zip(results, [C_ANC, "#edae49", "#00798c", "#6a994e"]):
        ea = signal.sosfilt(sos, r["e"])
        da = signal.sosfilt(sos, r["d"])
        nb = len(ea) // blk
        t = (np.arange(nb) + 0.5) * blk / FS
        lv = [20 * np.log10(np.std(ea[i * blk:(i + 1) * blk]) / 20e-6) for i in range(nb)]
        lp = [20 * np.log10(np.std(da[i * blk:(i + 1) * blk]) / 20e-6) for i in range(nb)]
        ax.plot(t, lv, color=c, label=f"{r['kind']} (ANC)")
        ax.plot(t, lp, color=c, ls=":", lw=0.8)
    ax.set_xlabel("time after ANC on (s)")
    ax.set_ylabel("dB(A) at ear, 250 ms blocks")
    ax.set_title("Convergence (dotted = passive only)")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(RES, "convergence.png"))
    plt.close(fig)


def latency_sweep(n):
    rows = []
    for extra in [0, 1, 2, 3, 4, 6]:
        pl = Plant(PlantConfig(extra_delay_samples=extra))
        s, f = factory_cal(pl)
        rr = [run_case(pl, s, f, k, n) for k in ("motor", "pink")]
        rows.append((extra, [r["passive_dba"] - r["anc_dba"] for r in rr]))
    fig, ax = plt.subplots(figsize=(6, 3.2))
    xs = [r[0] * 1e6 / FS for r in rows]
    ax.plot(xs, [r[1][0] for r in rows], "o-", color=C_ANC, label="motor")
    ax.plot(xs, [r[1][1] for r in rows], "o-", color="#00798c", label="pink (broadband)")
    ax.set_xlabel("extra electrical latency (µs)")
    ax.set_ylabel("ANC attenuation (dB(A))")
    ax.set_title("Why latency is everything")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(RES, "latency_sweep.png"))
    plt.close(fig)
    return rows


def robustness(n):
    base = Plant()
    s0, f0 = factory_cal(base)
    cases = []
    # 1) model gain error: true plant louder/quieter than the stored model
    for g_db in [-6, -3, 3, 6]:
        s = s0 * 10 ** (g_db / 20)
        r = run_case(base, s, f0, "motor", n)
        cases.append((f"S_hat gain {g_db:+d} dB", r["passive_dba"] - r["anc_dba"], r["trips"]))
    # 2) model delay error (fit changed, driver moved)
    for dly in [-1, 1, 2]:
        s = np.roll(s0, dly)
        if dly > 0:
            s[:dly] = 0
        else:
            s[dly:] = 0
        r = run_case(base, s, f0, "motor", n)
        cases.append((f"S_hat delay {dly:+d} sample", r["passive_dba"] - r["anc_dba"], r["trips"]))
    # 3) poor seal (glasses, bad fit) - plant changes, model is the good-seal factory one
    for sc in [0.7, 0.5]:
        pl = Plant(PlantConfig(passive_il_scale=sc))
        r = run_case(pl, s0, f0, "motor", n)
        cases.append((f"seal {int(sc*100)}% of nominal", r["passive_dba"] - r["anc_dba"], r["trips"]))
    # 4) boot-time refinement of the factory model under 100 dB(A) noise
    rng = np.random.default_rng(3)
    _, d, xr, _ = scenario(base, "motor", 4 * FS, rng)
    s_ref, f_ref = identify_paths(base, d, xr, l_s=TUNED.l_s, l_f=TUNED.l_f, seconds=3.0, mu=0.0005,
                                  s0=s0, f0=f0)
    err = np.linalg.norm(s_ref - base.h_S[:TUNED.l_s]) / np.linalg.norm(base.h_S[:TUNED.l_s])
    r = run_case(base, s_ref, f_ref, "motor", n)
    cases.append((f"boot refine in 100 dB(A) (S err {err*100:.0f}%)", r["passive_dba"] - r["anc_dba"], r["trips"]))
    fig, ax = plt.subplots(figsize=(7, 3.6))
    ax.barh([c[0] for c in cases][::-1], [c[1] for c in cases][::-1], color=C_ANC)
    ax.set_xlabel("ANC attenuation on motor noise (dB(A))")
    ax.set_title("Robustness to model error and fit")
    fig.tight_layout()
    fig.savefig(os.path.join(RES, "robustness.png"))
    plt.close(fig)
    return cases


def cf(v):
    """C float literal that always has a decimal point or exponent."""
    t = f"{float(v):.9g}"
    if not any(c in t for c in ".en"):
        t += ".0"
    return t + "f"


def export_firmware(pl, s, f, p=None):
    p = p or TUNED
    inc = os.path.join(FW, "inc")
    vec = os.path.join(FW, "test", "vectors")
    os.makedirs(inc, exist_ok=True)
    os.makedirs(vec, exist_ok=True)
    wb0, wb1, wa1 = weighting_coeffs(p.w_hp_hz, FS, p.w_floor_db)
    hb, ha = ref_hp_coeffs(p.ref_hp_hz, FS)
    with open(os.path.join(inc, "anc_tuning.h"), "w") as fh:
        fh.write(f"""/* AUTO-GENERATED by sim/run_all.py - do not edit by hand.
 * Tuned hybrid-FxLMS parameters (see sim/results/summary.md). */
#ifndef ANC_TUNING_H
#define ANC_TUNING_H

#define ANC_FS_HZ          {FS}u
#define ANC_L_FF           {p.l_ff}u   /* feedforward taps */
#define ANC_L_FB           {p.l_fb}u   /* adaptive feedback (IMC) taps */
#define ANC_L_S            {p.l_s}u   /* secondary-path model taps */
#define ANC_L_F            {p.l_f}u   /* speaker->ref-mic leakage model taps */
#define ANC_MU_FF          {cf(p.mu_ff)}
#define ANC_MU_FB          {cf(p.mu_fb)}
#define ANC_LEAK           {cf(p.leak)}
#define ANC_EPS            {cf(p.eps)}
#define ANC_Y_MAX_PA       {cf(p.y_max)}   /* output clamp, Pa at the ear */
#define ANC_WD_RATIO       {cf(10 ** (p.wd_ratio_db / 10))}
#define ANC_WD_HOLD        {int(p.wd_hold_ms * 1e-3 * FS)}u
#define ANC_CLIP_TRIP      {cf(p.clip_trip)}

/* error/filtered-reference weighting: 1st-order low-shelf {p.w_hp_hz:g} Hz, {p.w_floor_db:g} dB floor */
#define ANC_W_B0           {cf(wb0)}
#define ANC_W_B1           {cf(wb1)}
#define ANC_W_A1           {cf(wa1)}

/* 2nd-order Butterworth high-pass {p.ref_hp_hz:g} Hz on x_c and d_hat */
#define ANC_HP_B0          {cf(hb[0])}
#define ANC_HP_B1          {cf(hb[1])}
#define ANC_HP_B2          {cf(hb[2])}
#define ANC_HP_A1          {cf(ha[1])}
#define ANC_HP_A2          {cf(ha[2])}

/* boot-time path identification */
#define ANC_ID_PROBE_PA    0.3f        /* ~83 dB SPL at the ear */
#define ANC_ID_SECONDS     3u
#define ANC_ID_MU_FACTORY  0.01f       /* quiet-room factory calibration */
#define ANC_ID_MU_REFINE   0.0005f     /* in-field refinement from the stored model */

#endif
""")
    # fixed filters used by the dosimeter and hear-through (generated here so they match the sim)
    from noise import a_weighting_sos
    sos_a = a_weighting_sos(FS)
    sos_ht = np.vstack([signal.butter(2, 300, "highpass", fs=FS, output="sos"),
                        signal.butter(2, 4000, "lowpass", fs=FS, output="sos")])
    def sos_c(name, sos):
        rows = ",\n".join("    {" + ", ".join(cf(v) for v in (r[0], r[1], r[2], r[4], r[5])) + "}" for r in sos)
        return f"#define {name}_N {len(sos)}u\nstatic const float {name}[{len(sos)}][5] = {{\n{rows}\n}};\n"
    with open(os.path.join(inc, "dsp_coeffs.h"), "w") as fh:
        fh.write("/* AUTO-GENERATED by sim/run_all.py - biquads as {b0, b1, b2, a1, a2} (a0 = 1). */\n"
                 "#ifndef DSP_COEFFS_H\n#define DSP_COEFFS_H\n\n"
                 "/* IEC 61672 A-weighting at 32 kHz (dosimeter) */\n" + sos_c("SOS_AWEIGHT", sos_a) +
                 "\n/* hear-through speech band 300 Hz - 4 kHz */\n" + sos_c("SOS_HEARTHRU", sos_ht) +
                 "\n#endif\n")
    np.asarray(s, np.float32).tofile(os.path.join(vec, "s_hat.f32"))
    np.asarray(f, np.float32).tofile(os.path.join(vec, "f_hat.f32"))
    np.asarray(pl.h_S, np.float32).tofile(os.path.join(vec, "plant_S.f32"))
    np.asarray(pl.h_F, np.float32).tofile(os.path.join(vec, "plant_F.f32"))
    np.asarray(pl.h_P, np.float32).tofile(os.path.join(vec, "plant_P.f32"))
    np.asarray(pl.h_ref, np.float32).tofile(os.path.join(vec, "plant_ref.f32"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="shorter runs, skip sweeps")
    ap.add_argument("--wav", help="recorded noise WAV (any rate), scaled to 100 dB(A)")
    a = ap.parse_args()
    os.makedirs(RES, exist_ok=True)
    n = (8 if a.quick else 20) * FS

    pl = Plant()
    s, f = factory_cal(pl)
    fig_paths(pl)
    kinds = ["motor", "compressor", "pink"] + (["recorded"] if a.wav else [])
    results = [run_case(pl, s, f, k, n, wav=a.wav) for k in kinds]
    for r in results:
        fig_spectrum(r)
    fig_convergence(results)
    lat = [] if a.quick else latency_sweep(12 * FS)
    rob = [] if a.quick else robustness(12 * FS)
    export_firmware(pl, s, f)

    summ = dict(fs=FS, ambient_dba=AMBIENT_DBA, params=asdict(TUNED),
                s_group_delay_samples=pl.s_delay_samples(),
                cases=[{k: (float(v) if isinstance(v, (float, np.floating)) else v)
                        for k, v in r.items() if not k.startswith(("third", "e", "d"))} for r in results],
                latency_sweep=[{"extra_us": x * 1e6 / FS, "motor_db": y[0], "pink_db": y[1]} for x, y in lat],
                robustness=[{"case": c, "anc_db": v, "trips": t} for c, v, t in rob])
    with open(os.path.join(RES, "summary.json"), "w") as fh:
        json.dump(summ, fh, indent=2, default=float)

    L = ["# Simulation results", "",
         f"Ambient {AMBIENT_DBA:.0f} dB(A) outside the helmet, fs = {FS} Hz, secondary-path low-frequency group delay "
         f"{pl.s_delay_samples():.1f} samples. Levels at the ear, last 5 s of a {n // FS} s run.", "",
         "| Noise | Open ear dB(A) | Passive dB(A) | Passive + ANC dB(A) | ANC adds | Total reduction | Watchdog trips |",
         "| --- | --- | --- | --- | --- | --- | --- |"]
    for r in results:
        L.append(f"| {r['kind']} | {r['open_dba']:.1f} | {r['passive_dba']:.1f} | {r['anc_dba']:.1f} | "
                 f"{r['passive_dba'] - r['anc_dba']:.1f} dB | **{r['open_dba'] - r['anc_dba']:.1f} dB** | {r['trips']} |")
    if lat:
        L += ["", "## Latency sweep (ANC attenuation, dB(A))", "", "| Extra latency (µs) | Motor | Pink |", "| --- | --- | --- |"]
        L += [f"| {x * 1e6 / FS:.0f} | {y[0]:.1f} | {y[1]:.1f} |" for x, y in lat]
    if rob:
        L += ["", "## Robustness (motor noise, ANC attenuation)", "", "| Case | ANC dB(A) | Trips |", "| --- | --- | --- |"]
        L += [f"| {c} | {v:.1f} | {t} |" for c, v, t in rob]
    i63 = [i for i, v in enumerate(THIRD_OCT) if v <= 63]
    lf = lambda r, key: 10 * np.log10(np.sum(10 ** (r[key][i63] / 10)))
    i63 = [i for i, v in enumerate(THIRD_OCT) if v <= 63]
    lf = lambda r, key: 10 * np.log10(np.sum(10 ** (r[key][i63] / 10)))
    L += ["", "## Known limitation: sub-63 Hz rumble", "",
          "The update is weighted toward 150 Hz-1 kHz, where the A-weighted (hearing-damage) dose is. "
          "Energy below 63 Hz is left roughly as the passive cup leaves it:", "",
          "| Noise | Passive <=63 Hz (dB, unweighted) | Passive + ANC <=63 Hz |", "| --- | --- | --- |"]
    L += [f"| {r['kind']} | {lf(r, 'third_pass'):.1f} | {lf(r, 'third_anc'):.1f} |" for r in results]
    L += ["", "Figures: `paths.png`, `spectra_*.png`, `convergence.png`, `latency_sweep.png`, `robustness.png`."]
    with open(os.path.join(RES, "summary.md"), "w") as fh:
        fh.write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main()
