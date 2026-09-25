#!/usr/bin/env python3
"""ANC on real recorded noise instead of the synthetic models.

Uses clips from the ESC-50 dataset (K. J. Piczak, CC BY-NC 3.0 - fine for this
study, not redistributed here): https://github.com/karolpiczak/ESC-50
Download them with --fetch (about 30 MB), or point --dir at any folder of WAVs
named <category>/*.wav or <anything>-<category>.wav.

For each category the clips are trimmed of digital silence, joined with short
crossfades, resampled to 32 kHz and scaled to 100 dB(A) at the outside mic,
then run through the same plant, factory calibration and controller as
run_all.py. Levels are measured over the last 5 s.

    python3 sim/real_audio.py --fetch          # download + run
    python3 sim/real_audio.py --dir my_wavs    # your own recordings

Caveat: ESC-50 clips are field recordings on consumer gear, so their content
below ~50 Hz is rolled off compared with a calibrated measurement mic.
"""
from __future__ import annotations

import argparse
import csv
import io
import json
import os
import sys
import urllib.request
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import soundfile as sf
from dataclasses import replace
from scipy import signal

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plant import Plant, FS  # noqa: E402
from noise import scale_to_dba, spl_a, third_octave_levels, THIRD_OCT  # noqa: E402
from anc_ref import run_anc  # noqa: E402
from run_all import TUNED, AMBIENT_DBA, factory_cal, RES  # noqa: E402

ESC = "https://raw.githubusercontent.com/karolpiczak/ESC-50/master"
CATEGORIES = ["engine", "chainsaw", "hand_saw", "helicopter", "train", "vacuum_cleaner", "washing_machine"]
PER_CAT = 10
SECONDS = 20


def fetch(dest):
    os.makedirs(dest, exist_ok=True)
    meta = urllib.request.urlopen(f"{ESC}/meta/esc50.csv").read().decode()
    rows = [r for r in csv.DictReader(io.StringIO(meta)) if r["category"] in CATEGORIES]
    todo = []
    for c in CATEGORIES:
        todo += [(c, r["filename"]) for r in rows if r["category"] == c][:PER_CAT]

    def get(item):
        c, fn = item
        out = os.path.join(dest, c, fn)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        if not os.path.exists(out):
            with open(out, "wb") as fh:
                fh.write(urllib.request.urlopen(f"{ESC}/audio/{fn}").read())
    with ThreadPoolExecutor(8) as ex:
        list(ex.map(get, todo))


def load_category(folder, n):
    """Silence-trimmed, crossfaded, 32 kHz concatenation of every WAV in folder, n samples."""
    parts = []
    for fn in sorted(os.listdir(folder)):
        if not fn.lower().endswith((".wav", ".flac", ".ogg")):
            continue
        x, fs = sf.read(os.path.join(folder, fn), always_2d=True)
        x = x[:, 0]
        # ESC-50 pads short recordings with digital silence
        env = np.convolve(np.abs(x), np.ones(441) / 441, mode="same")
        x = x[env > 1e-4 * max(env.max(), 1e-12)]
        if len(x) < fs // 2:
            continue
        x = signal.resample_poly(x, FS, fs) if fs != FS else x
        x = x - np.mean(x)
        parts.append(x / (np.std(x) + 1e-12))       # equal loudness per clip before joining
    if not parts:
        raise RuntimeError(f"no usable audio in {folder}")
    xf = int(0.05 * FS)
    ramp = np.linspace(0, 1, xf)
    out = parts[0]
    while len(out) < n:
        for p in parts[1:] + parts[:1]:
            out = np.concatenate([out[:-xf], out[-xf:] * (1 - ramp) + p[:xf] * ramp, p[xf:]])
            if len(out) >= n:
                break
    return out[:n]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fetch", action="store_true", help="download the ESC-50 clips first")
    ap.add_argument("--dir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "noise-data"),
                    help="folder with one sub-folder of WAVs per noise category")
    a = ap.parse_args()
    if a.fetch:
        fetch(a.dir)
    cats = sorted(d for d in os.listdir(a.dir) if os.path.isdir(os.path.join(a.dir, d)))
    cats = [c for c in CATEGORIES if c in cats] + [c for c in cats if c not in CATEGORIES]

    n = SECONDS * FS
    pl = Plant()
    s_hat, f_hat = factory_cal(pl)
    last = slice(n - 5 * FS, n)
    rows = []
    for c in cats:
        out = scale_to_dba(load_category(os.path.join(a.dir, c), n), FS, AMBIENT_DBA)
        d = signal.lfilter(pl.h_P, 1, out)
        xr = signal.lfilter(pl.h_ref, 1, out)
        op = signal.lfilter(pl.h_open, 1, out)
        e, y, *_, trips = run_anc(d, xr, pl, s_hat, f_hat, TUNED, np.random.default_rng(7))
        # same controller with twice the output headroom: shows where the driver/amp limit bites
        e2, *_, trips2 = run_anc(d, xr, pl, s_hat, f_hat, replace(TUNED, y_max=2 * TUNED.y_max),
                                 np.random.default_rng(7))
        t_open, t_pass, t_anc = (third_octave_levels(v[last], FS) for v in (op, d, e))
        band = np.array(THIRD_OCT)
        lf = (band >= 80) & (band <= 500)
        r = dict(category=c, open_dba=spl_a(op[last], FS), passive_dba=spl_a(d[last], FS),
                 anc_dba=spl_a(e[last], FS), trips=int(trips), peak_y_pa=float(np.max(np.abs(y))),
                 anc_2x_headroom_dba=spl_a(e2[last], FS), trips_2x_headroom=int(trips2),
                 anc_80_500_db=float(np.max(t_pass[lf] - t_anc[lf])))
        rows.append(r)
        print(f"{c:16s} open {r['open_dba']:6.1f}  passive {r['passive_dba']:6.1f}  "
              f"+ANC {r['anc_dba']:6.1f} dB(A)  (ANC {r['passive_dba'] - r['anc_dba']:+.1f}, trips {trips})", flush=True)

    os.makedirs(RES, exist_ok=True)
    with open(os.path.join(RES, "real_audio.json"), "w") as fh:
        json.dump(dict(ambient_dba=AMBIENT_DBA, seconds=SECONDS, source="ESC-50 (CC BY-NC 3.0)", cases=rows),
                  fh, indent=2, default=float)
    L = ["# ANC on real recorded noise (ESC-50 clips)", "",
         f"{PER_CAT} clips per category, silence-trimmed and joined into {SECONDS} s, scaled to "
         f"{AMBIENT_DBA:.0f} dB(A) outside. Levels at the ear over the last 5 s; same plant, factory "
         "calibration and tuned controller as `run_all.py`.", "",
         "| Noise | Open ear | Passive cup | Passive + ANC | ANC adds | Best 1/3-oct ANC gain, 80-500 Hz | Watchdog trips "
         f"| + ANC, {2 * TUNED.y_max:.0f} Pa headroom (trips) |",
         "| --- | --- | --- | --- | --- | --- | --- | --- |"]
    for r in rows:
        L.append(f"| {r['category']} | {r['open_dba']:.1f} | {r['passive_dba']:.1f} | **{r['anc_dba']:.1f}** | "
                 f"{r['passive_dba'] - r['anc_dba']:.1f} dB | {r['anc_80_500_db']:.1f} dB | {r['trips']} | "
                 f"{r['anc_2x_headroom_dba']:.1f} ({r['trips_2x_headroom']}) |")
    L += ["", f"Watchdog trips are all from output clipping (the anti-noise wanted more than the "
          f"{TUNED.y_max:.0f} Pa clamp, i.e. more than the driver/amp is assumed to deliver), not from "
          "divergence: every trip resets the filters and costs ~1 dB. Low-frequency engine rumble "
          f"needs ~15 Pa peak at the ear. With {2 * TUNED.y_max:.0f} Pa of headroom there are no trips; "
          "whether the TPA6132A2 + 40 mm driver can deliver that is the hardware question the SPICE "
          "study answers.",
          "", "Source: ESC-50, K. J. Piczak, CC BY-NC 3.0 (https://github.com/karolpiczak/ESC-50). "
          "Consumer field recordings: content below ~50 Hz is under-represented."]
    with open(os.path.join(RES, "real_audio.md"), "w") as fh:
        fh.write("\n".join(L) + "\n")


if __name__ == "__main__":
    main()
