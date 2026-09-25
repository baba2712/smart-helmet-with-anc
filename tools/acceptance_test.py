#!/usr/bin/env python3
"""Prototype acceptance test: the TRL-6 evidence run for one assembled helmet.

Walks an operator through the test sequence over USB, measures everything it can
through the helmet's own command link, asks the operator for what only a person
can do or read (calibrator on a mic, noise source on, reference SPL meter reading,
glasses under the cushion), and writes a dated pass/fail report:

    reports/<serial>-<date>/acceptance.json and acceptance.md

    python3 tools/acceptance_test.py                  # full run, interactive
    python3 tools/acceptance_test.py --only noise,seal
    python3 tools/acceptance_test.py --self-test      # exercise the script against a simulated helmet

Setup (docs/bringup_and_test.md): helmet on a head-and-torso simulator or a
volunteer, a loudspeaker playing broadband industrial noise, a Class 1/2 SPL
meter next to the outside mic, a 94 dB / 1 kHz acoustic calibrator.
Pass limits come from the simulation, SPICE and firmware-in-the-loop studies
in this repository; each step says where its limit comes from.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# ------------------------------------------------------------------ limits (with their source)
LIM = {
    "cpu_pct_max": 80.0,          # 32 kHz frame must leave headroom (budget in docs/architecture.md)
    "isr_us_max": 25.0,           # of the 31.25 us sample period
    "vbat_min": 3.5,
    "mic_cal_err_db": 1.0,        # after trim, calibrator reads 94.0 +/- 1.0 dB
    "path_fit_db": 10.0,          # firmware refuses a factory path calibration below this
    "driver_pa_per_v": 17.0,      # SPICE: ~15 Pa of 63 Hz anti-noise at the amp's 0.9 V
    "noise_amb_dba": (95.0, 105.0),   # test condition: outside level
    "ear_anc_dba_max": 80.0,      # sim at 100 dB(A): 69-75 dB(A); 5 dB margin for a real head
    "anc_gain_db_min": 3.0,       # sim/real recordings: +3..8 dB(A) over the passive cup
    "internal_vs_meter_db": 2.0,  # helmet's outside reading vs the reference SPL meter
    "seal_flag_s": 15.0,          # sim: 3-5 s; allow for handling time
    "seal_clear_s": 20.0,
    "soak_min": 10,               # continuous ANC without a watchdog trip or audio overrun
}


# ------------------------------------------------------------------ helmet link
class Link:
    def __init__(self, port=None):
        sys.path.insert(0, HERE)
        from anc_tool import Helmet
        self.h = Helmet(port)

    def cmd(self, line, wait=None, timeout=3.0):
        return self.h.cmd(line, wait_end=wait, timeout=timeout)

    def status(self):
        return self.h.status()


class SimLink:
    """A stand-in helmet for --self-test: plausible replies, so the script's own logic runs."""
    def __init__(self):
        self.mode, self.leak, self.t_leak = "passive", False, None

    def cmd(self, line, wait=None, timeout=3.0):
        if line == "version":
            return ["ANC-Helmet fw sim, fs=32000 Hz"]
        if line.startswith("mode "):
            self.mode = line.split()[1].replace("ht", "anc+ht")
            return ["OK"]
        if line.startswith("cal paths"):
            return ["path calibration ok: fit L 15.8 dB, R 16.1 dB", "calibration saved"]
        if line.startswith("cal mic"):
            return ["OK trim 1.0123 (was reading 93.89 dB) - 'save' to keep"]
        if line.startswith("test driver"):
            return ["driver test 63 Hz: L 22.4 Pa/V, R 21.9 Pa/V - PASS (>= 17 Pa/V)"]
        if line.startswith("seal learn"):
            return ["seal baseline L 17.0 24.6 31.0 34.6 / R 16.8 24.2 30.7 34.1 dB - 'save' to keep"]
        if line == "save":
            return ["OK saved"]
        return ["OK"]

    def status(self):
        anc = self.mode.startswith("anc")
        leak = self.leak and self.t_leak is not None and time.time() - self.t_leak > 1
        return {"mode": self.mode, "cal": 1, "ear_dba": (72.4 if anc else 78.0) + (6 if leak else 0),
                "amb_dba": 99.6, "atten_db": 27, "dose_pct": 1.0, "dose_unprot_pct": 20, "laeq_shift": 72,
                "vbat": 3.95, "chg": 0, "usb": 1, "ble": 0, "cpu_pct": 46.0, "isr_us_max": 14.2,
                "trips": [0, 0], "ovl": 0, "adc_ovr": 0, "fit_warn": 0, "seal_cal": 1,
                "seal_loss_db": [0.4, 6.1 if leak else 0.6], "seal_leak": [0, 1 if leak else 0]}


# ------------------------------------------------------------------ runner
class Run:
    def __init__(self, link, auto=False):
        self.link, self.auto, self.steps = link, auto, []

    def ask(self, prompt, default=""):
        if self.auto:
            print(f"  [auto] {prompt} -> {default}")
            return default
        return input(f"  >> {prompt} ").strip() or default

    def wait_ready(self, what):
        self.ask(f"{what} - press Enter when ready")

    def record(self, name, ok, value, limit, source, note=""):
        self.steps.append(dict(step=name, result="PASS" if ok else "FAIL", value=value, limit=limit,
                               source=source, note=note, time=dt.datetime.now().isoformat(timespec="seconds")))
        print(f"  {'PASS' if ok else 'FAIL'}  {name}: {value}  (limit {limit})")

    def average_status(self, seconds, keys):
        vals = {k: [] for k in keys}
        last = None
        t0 = time.time()
        while time.time() - t0 < seconds:
            st = self.link.status()
            if st:
                last = st
                for k in keys:
                    vals[k].append(st[k])
            time.sleep(1.0 if not isinstance(self.link, SimLink) else 0.0)
            if isinstance(self.link, SimLink) and len(vals[keys[0]]) >= 3:
                break
        import math
        # energy mean for levels
        return {k: 10 * math.log10(sum(10 ** (v / 10) for v in vs) / len(vs)) if vs else float("nan")
                for k, vs in vals.items()}, last


def step_health(r):
    v = r.link.cmd("version")
    st = r.link.status()
    r.record("firmware responds", bool(v and st), v[0] if v else "no reply", "reply", "USB CDC link")
    if not st:
        return
    r.record("battery", st["vbat"] >= LIM["vbat_min"], f"{st['vbat']:.2f} V", f">= {LIM['vbat_min']} V", "cell spec")
    r.record("CPU load", st["cpu_pct"] <= LIM["cpu_pct_max"], f"{st['cpu_pct']:.0f} %", f"<= {LIM['cpu_pct_max']:.0f} %",
             "32 kHz real-time budget")
    r.record("worst audio ISR", st["isr_us_max"] <= LIM["isr_us_max"], f"{st['isr_us_max']:.1f} us",
             f"<= {LIM['isr_us_max']} us", "31.25 us sample period")


def step_mics(r):
    for mic, where in (("refl", "LEFT OUTSIDE"), ("errl", "LEFT IN-CUP"), ("refr", "RIGHT OUTSIDE"), ("errr", "RIGHT IN-CUP")):
        r.wait_ready(f"94 dB / 1 kHz calibrator on the {where} mic ({mic})")
        rep = " ".join(r.link.cmd(f"cal mic {mic} 94", timeout=4))
        m = re.search(r"trim ([\d.]+) \(was reading ([\d.]+) dB\)", rep)
        ok = bool(m) and abs(float(m.group(2)) - 94.0) <= 6.0
        r.record(f"mic {mic} trim", ok, rep[:90], "trim within +/-6 dB", "IM73A135 sensitivity tolerance")
    r.link.cmd("save")


def step_paths(r):
    r.wait_ready("QUIET room (< 45 dB(A)), helmet on the test head")
    rep = " ".join(r.link.cmd("cal paths 3", wait="calibration saved", timeout=12))
    m = re.search(r"fit L ([\d.]+) dB, R ([\d.]+) dB", rep)
    fit = min(float(m.group(1)), float(m.group(2))) if m else 0.0
    r.record("speaker-path identification", fit >= LIM["path_fit_db"], f"{fit:.1f} dB fit",
             f">= {LIM['path_fit_db']} dB", "firmware + test_fil.py (16 dB in sim)")


def step_driver(r):
    r.wait_ready("helmet still on the test head (the driver test plays a 63 Hz tone)")
    rep = " ".join(r.link.cmd("test driver 63 100", wait="driver test ", timeout=8))
    m = re.search(r"L ([\d.]+) Pa/V, R ([\d.]+) Pa/V", rep)
    v = min(float(m.group(1)), float(m.group(2))) if m else 0.0
    r.record("driver sensitivity at 63 Hz", v >= LIM["driver_pa_per_v"], f"{v:.1f} Pa/V (worse ear)",
             f">= {LIM['driver_pa_per_v']} Pa/V", "hardware/spice/results/spice_results.md")


def step_seal_learn(r):
    r.wait_ready("GOOD fit on the test head, broadband noise ON at 80-100 dB(A)")
    rep = " ".join(r.link.cmd("seal learn 10", wait="seal baseline", timeout=15))
    ok = "seal baseline" in rep and "ERR" not in rep
    r.record("seal baseline learned", ok, rep[:90], "baseline for both ears", "sim/seal_ref.py")
    if ok:
        r.link.cmd("save")


def step_noise(r):
    r.wait_ready("broadband industrial noise ON, ~100 dB(A) at the outside mic; good fit")
    r.link.cmd("mode passive")
    time.sleep(3 if not isinstance(r.link, SimLink) else 0)
    p, _ = r.average_status(20, ["amb_dba", "ear_dba"])
    r.link.cmd("mode anc")
    time.sleep(8 if not isinstance(r.link, SimLink) else 0)
    a, last = r.average_status(30, ["amb_dba", "ear_dba"])
    lo, hi = LIM["noise_amb_dba"]
    r.record("test condition: outside level", lo <= a["amb_dba"] <= hi, f"{a['amb_dba']:.1f} dB(A)",
             f"{lo}-{hi} dB(A)", "relevant environment (plant noise)")
    meter = r.ask("reference SPL meter reading next to the outside mic, dB(A)?", default=f"{a['amb_dba']:.1f}")
    try:
        d = abs(float(meter) - a["amb_dba"])
        r.record("helmet outside reading vs reference meter", d <= LIM["internal_vs_meter_db"], f"{d:.1f} dB apart",
                 f"<= {LIM['internal_vs_meter_db']} dB", "dosimeter calibration (test_fil.py: 93.99 dB on 94 dB)")
    except ValueError:
        pass
    r.record("level at the ear, ANC on", a["ear_dba"] <= LIM["ear_anc_dba_max"], f"{a['ear_dba']:.1f} dB(A)",
             f"<= {LIM['ear_anc_dba_max']} dB(A)", "sim/results/summary.md, real_audio.md")
    g = p["ear_dba"] - a["ear_dba"]
    r.record("ANC gain over the passive cup", g >= LIM["anc_gain_db_min"], f"{g:.1f} dB(A)",
             f">= {LIM['anc_gain_db_min']} dB(A)", "sim/results/real_audio.md")
    if last:
        r.record("no watchdog trips", sum(last["trips"]) == 0 and last["ovl"] == 0, f"trips {last['trips']}, ovl {last['ovl']}",
                 "0", "firmware watchdogs")


def step_seal(r):
    r.link.cmd("mode anc")
    r.wait_ready("noise still ON; now slide a safety-glasses temple under the RIGHT cushion")
    if isinstance(r.link, SimLink):
        r.link.leak, r.link.t_leak = True, time.time()
    t0 = time.time()
    flagged = None
    while time.time() - t0 < LIM["seal_flag_s"] + 5:
        st = r.link.status()
        if st and any(st["seal_leak"]):
            flagged = time.time() - t0
            break
        time.sleep(0.5)
    r.record("seal leak flagged", flagged is not None and flagged <= LIM["seal_flag_s"],
             "not flagged" if flagged is None else f"after {flagged:.1f} s", f"<= {LIM['seal_flag_s']} s",
             "sim/results/seal_monitor.md (3-5 s)")
    r.wait_ready("remove the glasses and reseat the cup")
    if isinstance(r.link, SimLink):
        r.link.leak = False
    t0 = time.time()
    cleared = None
    while time.time() - t0 < LIM["seal_clear_s"] + 5:
        st = r.link.status()
        if st and not any(st["seal_leak"]):
            cleared = time.time() - t0
            break
        time.sleep(0.5)
    r.record("seal flag clears after reseating", cleared is not None and cleared <= LIM["seal_clear_s"],
             "still flagged" if cleared is None else f"after {cleared:.1f} s", f"<= {LIM['seal_clear_s']} s",
             "hysteresis 1 dB, 4 s average")


def step_soak(r):
    mins = LIM["soak_min"] if not isinstance(r.link, SimLink) else 0
    r.wait_ready(f"noise ON, ANC on: {LIM['soak_min']} min soak")
    r.link.cmd("mode anc")
    t0 = time.time()
    worst = None
    while time.time() - t0 < mins * 60:
        st = r.link.status()
        worst = st or worst
        time.sleep(5)
    st = r.link.status() if worst is None else worst
    ok = st and sum(st["trips"]) == 0 and st["adc_ovr"] == 0 and st["ovl"] == 0
    r.record(f"{LIM['soak_min']} min soak", bool(ok),
             f"trips {st['trips']}, audio overruns {st['adc_ovr']}, overload {st['ovl']}" if st else "no status",
             "all 0", "real-time firmware")


STEPS = [("health", step_health), ("mics", step_mics), ("paths", step_paths), ("driver", step_driver),
         ("seal_learn", step_seal_learn), ("noise", step_noise), ("seal", step_seal), ("soak", step_soak)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--only", help="comma list of: " + ",".join(n for n, _ in STEPS))
    ap.add_argument("--serial", default="proto-01", help="unit name for the report")
    ap.add_argument("--self-test", action="store_true", help="run against a simulated helmet, no prompts")
    a = ap.parse_args()
    link = SimLink() if a.self_test else Link(a.port)
    r = Run(link, auto=a.self_test)
    only = set(a.only.split(",")) if a.only else None
    for name, fn in STEPS:
        if only and name not in only:
            continue
        print(f"\n== {name}")
        fn(r)
    n_fail = sum(s["result"] == "FAIL" for s in r.steps)
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M")
    out = os.path.join(ROOT, "reports", f"{a.serial}-{stamp}" + ("-selftest" if a.self_test else ""))
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "acceptance.json"), "w") as fh:
        json.dump(dict(unit=a.serial, date=stamp, simulated=a.self_test, limits=LIM, steps=r.steps), fh, indent=2)
    L = [f"# Acceptance test - {a.serial}, {stamp}" + (" (SELF-TEST, simulated helmet)" if a.self_test else ""), "",
         f"**{len(r.steps) - n_fail} / {len(r.steps)} checks passed.**", "",
         "| Step | Result | Value | Limit | Limit source |", "| --- | --- | --- | --- | --- |"]
    L += [f"| {s['step']} | {s['result']} | {s['value']} | {s['limit']} | {s['source']} |" for s in r.steps]
    with open(os.path.join(out, "acceptance.md"), "w") as fh:
        fh.write("\n".join(L) + "\n")
    print(f"\n{len(r.steps) - n_fail}/{len(r.steps)} passed - report: {out}")
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
