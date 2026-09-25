#!/usr/bin/env python3
"""Seal-aware protection: the helmet measures its own attenuation while worn.

Idea (the product's differentiator): a hearing-protector's real attenuation
drops 3-12 dB when a safety-glasses temple, hair or a tilted helmet breaks the
cushion seal, and nothing tells the wearer or the safety officer. Compliance
programs assume the label rating. The ANC hardware already has an outside
(reference) mic and an inside (error) mic, and the controller already forms

    x_c   = x - F_hat * y     outside noise without the speaker's own leakage
    d_hat = e - S_hat * y     the noise that got *through the cup* (ANC removed)

so, every second, with no extra sensor or test signal:

    per-octave-band attenuation  IL_b = L_b(x_c) - L_b(d_hat)  -> seal quality (noise-independent)
    total protection     PAR_A = L_A(x_c) - L_A(e)              -> what the wearer actually gets
    verified exposure          = L_A(e) (+ the mic's cup-to-eardrum offset)

and the seal is flagged when the band attenuation falls a set margin below
ONE factory baseline (seal_ref.py: the same decision logic as the firmware). This study checks that the estimates track the truth in the plant
model, across synthetic and real recorded noise, and shows how wrong a
label-based dose estimate becomes with a leak.

    python3 sim/seal_monitor.py [--dir ../noise-data]
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
from scipy import signal

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from plant import Plant, PlantConfig, FS  # noqa: E402
from noise import industrial, scale_to_dba, spl_a  # noqa: E402
from anc_ref import run_anc  # noqa: E402
from run_all import TUNED, AMBIENT_DBA, factory_cal, RES  # noqa: E402
import seal_ref  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# seal conditions: passive insertion-loss scale (1.0 = factory fit)
SEALS = [(1.0, "good seal"), (0.85, "thin glasses temple"), (0.7, "thick glasses temple"),
         (0.55, "poor fit / hair")]
FLAG_DB = seal_ref.FLAG_DB   # flag when band attenuation drops this far below the factory baseline
WIN = FS               # 1 s estimation window
SECONDS = 16


def a_level_blocks(x):
    """A-weighted level of each 1 s block (dB re 20 uPa)."""
    return np.array([spl_a(x[i:i + WIN], FS) for i in range(0, len(x) - WIN + 1, WIN)])


def past_only(h):
    """The controller forms S_hat*y from *previous* outputs (tap 0 unused)."""
    g = h.copy()
    g[0] = 0.0
    return g


def leaky_plant(seal):
    """A leak lowers the passive insertion loss *and* lets the driver's low-frequency
    pressure escape (the secondary path's leak corner rises), so the factory S_hat
    no longer matches the plant - the estimator has to live with that."""
    return Plant(PlantConfig(passive_il_scale=seal, leak_hp=35.0 / seal ** 2))


def run_case(seal, out, s_hat, f_hat):
    pl = leaky_plant(seal)
    d = signal.lfilter(pl.h_P, 1, out)
    xr = signal.lfilter(pl.h_ref, 1, out)
    e, y, *_, trips = run_anc(d, xr, pl, s_hat, f_hat, TUNED, np.random.default_rng(7))
    # what the firmware sees / computes (sensor noise is irrelevant at these levels)
    x_meas = xr + signal.lfilter(past_only(pl.h_F), 1, y)
    x_c = x_meas - signal.lfilter(past_only(f_hat), 1, y)
    d_hat = e - signal.lfilter(past_only(s_hat), 1, y)
    lx, ld, le = a_level_blocks(x_c), a_level_blocks(d_hat), a_level_blocks(e)
    bx, bd = seal_ref.band_ms_per_second(x_c), seal_ref.band_ms_per_second(d_hat)
    # ground truth from the plant: passive IL on this noise, total protection
    true_il = a_level_blocks(xr) - a_level_blocks(d)
    skip = 4                                   # let the ANC converge
    return dict(seal=seal, trips=int(trips), band_x=bx, band_d=bd,
                il_est=lx - ld, il_true=true_il, par_est=lx - le,
                at_ear=le, passive_ear=a_level_blocks(d),
                il_est_mean=float(np.mean((lx - ld)[skip:])), il_true_mean=float(np.mean(true_il[skip:])),
                par_mean=float(np.mean((lx - le)[skip:])), at_ear_mean=float(np.mean(le[skip:])))


def noise_set(n, data_dir):
    sets = [(k, scale_to_dba(industrial(n, FS, np.random.default_rng(7), k), FS, AMBIENT_DBA))
            for k in ("motor", "compressor", "pink")]
    if data_dir and os.path.isdir(data_dir):
        from real_audio import load_category, CATEGORIES
        for c in CATEGORIES:
            p = os.path.join(data_dir, c)
            if os.path.isdir(p):
                sets.append((c, scale_to_dba(load_category(p, n), FS, AMBIENT_DBA)))
    return sets


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.join(HERE, "..", "..", "noise-data"))
    a = ap.parse_args()
    n = SECONDS * FS
    s_hat, f_hat = factory_cal(Plant())
    sets = noise_set(n, a.dir)

    runs = {(name, seal): run_case(seal, out, s_hat, f_hat) for name, out in sets for seal, _ in SEALS}
    # ONE factory baseline: good seal, pink noise, energy mean after convergence
    ref = runs[("pink", 1.0)]
    base = seal_ref.learn_baseline(ref["band_x"][4:], ref["band_d"][4:])
    rows = []
    for name, _ in sets:
        for seal, label in SEALS:
            r = runs[(name, seal)]
            mon = seal_ref.SealMonitor(base)
            t_flag = None
            for k in range(len(r["band_x"])):
                mon.add_second(r["band_x"][k], r["band_d"][k])
                if mon.flag and t_flag is None:
                    t_flag = k + 1
            true_drop = runs[(name, 1.0)]["il_true_mean"] - r["il_true_mean"]
            r.update(noise=name, label=label, drop=mon.avg, flagged=bool(mon.flag), t_flag=t_flag,
                     true_drop=true_drop, est_err=r["il_est_mean"] - r["il_true_mean"])
            rows.append(r)
            print(f"{name:16s} {label:22s} IL true {r['il_true_mean']:5.1f}  est {r['il_est_mean']:5.1f}  "
                  f"band drop {mon.avg:4.1f} (true {true_drop:4.1f})  flag {r['flagged']!s:5s} "
                  f"t {t_flag}  at ear {r['at_ear_mean']:5.1f}  trips {r['trips']}", flush=True)

    # ---- summary numbers ----
    errs = np.array([r["est_err"] for r in rows])
    leaks = [r for r in rows if r["seal"] < 1.0]
    real_leaks = [r for r in leaks if r["true_drop"] >= FLAG_DB]
    t_flags = [r["t_flag"] for r in real_leaks if r["t_flag"]]
    tp = sum(r["flagged"] for r in real_leaks)
    fp = sum(r["flagged"] for r in rows if r["seal"] == 1.0)
    # a fit test at the start of the shift certifies the good-seal level; a leak later in the
    # shift raises the real dose with nobody knowing. Dose ratio = 2^(dL / 3) (NIOSH 3 dB exchange)
    good = {r["noise"]: r["at_ear_mean"] for r in rows if r["seal"] == 1.0}
    for r in rows:
        r["dose_x"] = float(2 ** ((r["at_ear_mean"] - good[r["noise"]]) / 3.0))
        r["anc_adds"] = float(np.mean(r["passive_ear"][4:]) - r["at_ear_mean"])
    thick = [r for r in rows if r["label"] == "thick glasses temple"]
    poor = [r for r in rows if r["label"] == "poor fit / hair"]
    anc_by_seal = {seal: float(np.mean([r["anc_adds"] for r in rows if r["seal"] == seal])) for seal, _ in SEALS}
    trips_by_seal = {seal: int(sum(r["trips"] for r in rows if r["seal"] == seal)) for seal, _ in SEALS}

    os.makedirs(RES, exist_ok=True)
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.8))
    for r in rows:
        c = {1.0: "#3b75af", 0.85: "#6a994e", 0.7: "#e9a03b", 0.55: "#d1495b"}[r["seal"]]
        ax[0].scatter(r["il_true_mean"], r["il_est_mean"], color=c, s=18)
    lim = [min(min(r["il_true_mean"], r["il_est_mean"]) for r in rows) - 1,
           max(max(r["il_true_mean"], r["il_est_mean"]) for r in rows) + 1]
    ax[0].plot(lim, lim, "k--", lw=0.8)
    ax[0].set_xlabel("true passive attenuation on this noise (dB(A))")
    ax[0].set_ylabel("in-use estimate (dB(A))")
    ax[0].set_title("Self-measured seal vs truth")
    for (seal, label), c in zip(SEALS, ["#3b75af", "#6a994e", "#e9a03b", "#d1495b"]):
        ax[0].scatter([], [], color=c, label=label)
    ax[0].legend(fontsize=7)
    names = [nm for nm, _ in sets]
    x = np.arange(len(names))
    for j, ((seal, label), c) in enumerate(zip(SEALS, ["#3b75af", "#6a994e", "#e9a03b", "#d1495b"])):
        v = [[r for r in rows if r["noise"] == nm and r["seal"] == seal][0]["at_ear_mean"] for nm in names]
        ax[1].bar(x + (j - 1.5) * 0.2, v, 0.2, color=c, label=label)
    ax[1].axhline(85, color="#999", ls=":", lw=0.8, label="85 dB(A) limit")
    ax[1].set_xticks(x)
    ax[1].set_xticklabels(names, rotation=35, ha="right", fontsize=7)
    ax[1].set_ylabel("verified level at the ear, ANC on (dB(A))")
    ax[1].set_ylim(55, 90)
    ax[1].legend(fontsize=6, ncol=2)
    ax[1].set_title("What the wearer actually gets")
    fig.tight_layout()
    fig.savefig(os.path.join(RES, "seal_monitor.png"))
    plt.close(fig)

    summ = dict(ambient_dba=AMBIENT_DBA, flag_db=FLAG_DB, window_s=WIN / FS,
                est_error_mean_db=float(np.mean(errs)), est_error_max_abs_db=float(np.max(np.abs(errs))),
                leaks_detected=f"{tp}/{len(real_leaks)}", false_alarms=f"{fp}/{len(sets)}",
                baseline_db=[float(v) for v in base], time_to_flag_s=[min(t_flags), max(t_flags)],
                dose_x_thick_temple=[min(r["dose_x"] for r in thick), max(r["dose_x"] for r in thick)],
                dose_x_poor_fit=[min(r["dose_x"] for r in poor), max(r["dose_x"] for r in poor)],
                cases=[{k: (float(v) if isinstance(v, (np.floating, float)) else v) for k, v in r.items()
                        if not isinstance(v, np.ndarray)} for r in rows])
    with open(os.path.join(RES, "seal_monitor.json"), "w") as fh:
        json.dump(summ, fh, indent=2, default=float)

    L = ["# Seal-aware protection: self-measured attenuation while worn", "",
         f"Ambient {AMBIENT_DBA:.0f} dB(A). Four seal conditions (passive insertion loss scaled to "
         + ", ".join(f"{int(s * 100)} %" for s, _ in SEALS) + ") x " + f"{len(sets)} noises "
         "(3 synthetic + real ESC-50 recordings). The leak also raises the driver's low-frequency leak corner "
         "(35 Hz / seal^2: " + ", ".join(f"{35 / s ** 2:.0f}" for s, _ in SEALS) + " Hz), so the factory "
         "secondary-path model is wrong under a leak, as it would be on a real head. Estimates use only signals the firmware already has "
         f"(outside mic, error mic, controller output, factory S_hat/F_hat), {WIN // FS} s windows. "
         "Detection uses the firmware's logic (`seal_ref.py`): octave bands 250 Hz-2 kHz against **one** "
         "factory baseline measured once (good seal, pink noise: "
         + ", ".join(f"{b:.0f} Hz {v:.1f} dB" for b, v in zip(seal_ref.BANDS, base)) + "), used for every noise.", "",
         f"- **Passive-attenuation estimate error:** mean {np.mean(errs):+.2f} dB, worst {np.max(np.abs(errs)):.2f} dB "
         "(estimate vs. the plant's true attenuation on the same noise).",
         f"- **Leak detection** (band attenuation {FLAG_DB:.0f} dB below the factory baseline): {tp} of "
         f"{len(real_leaks)} leaks that really cost >= {FLAG_DB:.0f} dB flagged, {fp} false alarms on "
         f"{len(sets)} good-seal runs, flagged {min(t_flags)}-{max(t_flags)} s after switch-on.",
         "- **What a one-time fit test misses:** it certifies the good-seal level. A thick glasses temple "
         f"later in the shift raises the real dose {min(r['dose_x'] for r in thick):.1f}-"
         f"{max(r['dose_x'] for r in thick):.1f}x, a poor fit {min(r['dose_x'] for r in poor):.1f}-"
         f"{max(r['dose_x'] for r in poor):.1f}x (NIOSH 3 dB exchange), with ANC running. "
         "Here it is measured continuously (1 s windows) and flagged.",
         "- **ANC cannot make up for a leak:** its average gain falls from "
         + " -> ".join(f"{anc_by_seal[s_]:.1f}" for s_, _ in SEALS) + " dB(A) as the seal worsens, and watchdog "
         "resets rise (" + " -> ".join(str(trips_by_seal[s_]) for s_, _ in SEALS) + " across all noises), because "
         "the driver's own low-frequency pressure leaks out too. Re-identifying the secondary path on the head "
         "was tested and does not restore it on tonal noise. The only fix is the wearer reseating the cup, "
         "so the product's response to a flag is an alert (tone + LED + log), not silent compensation.", "",
         "| Noise | Seal | True passive IL (A) | Estimated IL (A) | True loss | Band drop vs factory | Flagged (s) | At ear, ANC on | ANC adds | Measured PAR | Dose vs fit |",
         "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for r in rows:
        L.append(f"| {r['noise']} | {r['label']} | {r['il_true_mean']:.1f} | {r['il_est_mean']:.1f} | "
                 f"{r['true_drop']:.1f} | {r['drop']:.1f} | {('yes (' + str(r['t_flag']) + ')') if r['flagged'] else 'no'} | {r['at_ear_mean']:.1f} | {r['anc_adds']:.1f} | {r['par_mean']:.1f} | {r['dose_x']:.1f}x |")
    L += ["", "Levels in dB(A). IL = passive attenuation on that noise (ANC contribution removed); "
          "PAR = total protection with ANC. The error mic sits in the cup, not at the eardrum: a fixed "
          "per-design offset has to be calibrated on a head-and-torso simulator before these become "
          "compliance numbers.", "", "Figure: `seal_monitor.png`."]
    with open(os.path.join(RES, "seal_monitor.md"), "w") as fh:
        fh.write("\n".join(L) + "\n")
    print("\n".join(L[:9]))


if __name__ == "__main__":
    main()
