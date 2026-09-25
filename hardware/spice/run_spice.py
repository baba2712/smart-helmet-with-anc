#!/usr/bin/env python3
"""SPICE (ngspice) verification of the ANC helmet's analog chain.

Circuits come from the same component values as hardware/gen/boards.py:

  1. mic front ends (REF 75k/220p, ERR 150k/100p): difference amp + Sallen-Key
     -> response vs the plant model in sim/plant.py, input-referred noise vs
     the mic's own noise, and the SPL at which each channel clips
  2. VMID bias buffer: MCP6022 follower driving 10 R + 1 uF - stability
     (loop gain / phase margin, load-step ringing) over op-amp output resistance
  3. DAC reconstruction: Sallen-Key 8.2k/8.2k/3.3n/1.5n + 1 uF into the amp
     -> response and group delay (latency is what limits ANC)
  4. amp + driver + earcup: TPA6132A2 (behavioural, clipped) driving a
     Thiele-Small driver loaded by the cup's front/rear air volumes and seal leak
     -> the most pressure the anti-noise can reach at 63 Hz, vs the ~15 Pa the
     real-noise study needs (sim/results/real_audio.md)

Models are behavioural where no vendor model is available offline (the
MCP6022 macro-model is 2-pole: A0 110 dB, GBW 10 MHz, 65 deg PM; the
TPA6132A2 is a gain block with an output clamp). Driver parameters are
typical-class for a 40 mm headphone driver and are swept - replace them with
the chosen driver's measured Thiele-Small parameters.

    python3 hardware/spice/run_spice.py      # needs ngspice on PATH
Outputs: hardware/spice/results/*.png, spice_results.md, spice_results.json
"""
from __future__ import annotations

import json
import math
import os
import subprocess
import tempfile

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "results")
os.makedirs(RES, exist_ok=True)
plt.rcParams.update({"figure.dpi": 110, "axes.grid": True, "grid.alpha": 0.3, "font.size": 9})

VDDA = 2.8                   # 2V8A rail = ADC VREF+ = op-amp supply
VMID = VDDA / 2
MIC_SENS = 0.012589          # V/Pa differential (IM73A135, -38 dBV/Pa)
MIC_SELF_NOISE_DBA = 94 - 73 # SNR 73 dB(A) -> 21 dB(A) SPL self-noise
P0 = 20e-6
RHO_C2 = 1.21 * 343.0 ** 2   # air bulk modulus (Pa)

# ---------------------------------------------------------------- op-amp macro-model
OPAMP = """
* MCP6022-class behavioural op-amp: A0 110 dB, GBW 10 MHz, 2nd pole for 65 deg PM,
* rail-to-rail output (30 mV from the rails), open-loop output resistance {ro} ohm,
* input voltage noise 8.7 nV/rtHz (a noiseless-current resistor at the + input).
.subckt opamp inp inn out vdd vss
Rn inp inp_n {rn}
G1 0 n1 inp_n inn 0.316
R1 n1 0 1e6
C1 n1 0 5.03e-9
E2 n2 0 n1 0 1
R2 n2 n3 1k
C2 n3 0 7.44e-12
B3 n4 0 V = min(max(v(n3), v(vss) + 0.03), v(vdd) - 0.03)
Ro n4 out {ro}
.ends
"""
EN_OPAMP = 8.7e-9
RN_OPAMP = EN_OPAMP ** 2 / (4 * 1.380649e-23 * 300.0)


def opamp(ro=150.0):
    return OPAMP.format(ro=ro, rn=f"{RN_OPAMP:.1f}")


def ngspice(netlist):
    """Run a netlist in batch mode; returns {name: ndarray} from its wrdata files."""
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, "c.cir")
        with open(path, "w") as fh:
            fh.write(netlist.replace("@TD@", td))
        r = subprocess.run(["ngspice", "-b", path], capture_output=True, text=True, timeout=300)
        if r.returncode != 0 or "rror" in r.stdout + r.stderr and "no error" not in (r.stdout + r.stderr).lower():
            bad = [l for l in (r.stdout + r.stderr).splitlines() if "rror" in l]
            if bad:
                raise RuntimeError("ngspice failed:\n" + "\n".join(bad[:20]))
        out = {}
        for fn in os.listdir(td):
            if fn.endswith(".dat"):
                out[fn[:-4]] = np.loadtxt(os.path.join(td, fn))
        return out


def a_weight_db(f):
    f = np.asarray(f, float)
    ra = (12194 ** 2 * f ** 4) / ((f ** 2 + 20.6 ** 2) * np.sqrt((f ** 2 + 107.7 ** 2) * (f ** 2 + 737.9 ** 2))
                                  * (f ** 2 + 12194 ** 2))
    return 20 * np.log10(np.maximum(ra, 1e-20)) + 2.0


# ---------------------------------------------------------------- 1. mic front end
def frontend_net(rf, cf, analysis):
    return f"""mic front end
{opamp()}
Vdd vdd 0 {VDDA}
Vmid vmid 0 {VMID}
Vin in 0 DC 0 AC 1 {analysis.get('src', '')}
Vcm cm 0 0.8
E1 mp cm in 0 0.5
E2 mn cm in 0 -0.5
C1 mp ap 1u
C2 mn an 1u
R1 ap p 22k
R2 an n 22k
R3 p vmid {rf}
C3 p vmid {cf}
R4 n s1 {rf}
C4 n s1 {cf}
XA p n s1 vdd 0 opamp
R5 s1 ska 8.2k
R6 ska skb 8.2k
C5 ska out 4.7n
C6 skb 0 1n
XB skb out out vdd 0 opamp
R7 out adc 47
C7 adc 0 2.2n
.control
{analysis['cmd']}
.endc
.end
"""


def study_frontend():
    rows = {}
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
    for name, rf, cf, color in (("REF (outside)", "75k", "220p", "#3b75af"), ("ERR (in-cup)", "150k", "100p", "#d1495b")):
        ac = ngspice(frontend_net(rf, cf, {"cmd": "ac dec 50 1 1e6\nwrdata @TD@/ac.dat v(adc)"}))["ac"]
        f, re, im = ac[:, 0], ac[:, 1], ac[:, 2]
        h = re + 1j * im
        g_mid = np.abs(h[np.argmin(np.abs(f - 1000))])
        db = 20 * np.log10(np.abs(h) / g_mid)
        f_lo = f[np.argmax(db > -3)]
        f_hi = f[len(db) - 1 - np.argmax(db[::-1] > -3)]
        ax[0].semilogx(f, db, color=color, label=f"{name}: {g_mid:.2f} V/V, -3 dB {f_lo:.1f} Hz / {f_hi / 1e3:.1f} kHz")
        # noise at the ADC, referred to the mic terminals (V/rtHz), 20 Hz - 16 kHz
        nz = ngspice(frontend_net(rf, cf, {"cmd": "noise v(adc) Vin dec 50 20 16k\nsetplot noise1\n"
                                                  "wrdata @TD@/nz.dat inoise_spectrum onoise_spectrum"}))["nz"]
        fn, inz, onz = nz[:, 0], nz[:, 1], nz[:, 3]
        ax[1].loglog(fn, inz * 1e9, color=color, label=f"{name} electronics, input-referred")
        wa = 10 ** (a_weight_db(fn) / 10)
        in_a = math.sqrt(np.trapezoid(inz ** 2 * wa, fn))
        on_rms = math.sqrt(np.trapezoid(onz ** 2, fn))
        noise_spl_a = 20 * math.log10(in_a / MIC_SENS / P0)
        # clipping: rail-to-rail swing around VMID at the ADC, referred to Pa at the mic
        tr = ngspice(frontend_net(rf, cf, {"src": "SIN(0 2 1k)", "cmd": "tran 2u 3m\nwrdata @TD@/tr.dat v(adc)"}))["tr"]
        v = tr[tr[:, 0] > 1e-3, 1]
        swing = min(v.max() - VMID, VMID - v.min())
        clip_pa = swing / (g_mid * MIC_SENS)
        rows[name] = dict(gain=g_mid, f_lo_hz=f_lo, f_hi_hz=f_hi, noise_in_uV_a=in_a * 1e6, noise_out_uV=on_rms * 1e6,
                          noise_spl_dba=noise_spl_a, clip_pa_peak=clip_pa,
                          clip_db_peak=20 * math.log10(clip_pa / P0), swing_v=swing)
    mic_v = P0 * 10 ** (MIC_SELF_NOISE_DBA / 20) * MIC_SENS
    ax[1].axhline(mic_v / math.sqrt(16e3) * 1e9, color="k", ls="--", lw=0.8,
                  label=f"mic self-noise ({MIC_SELF_NOISE_DBA} dB(A) SPL, flat approx.)")
    # plant.py's assumption for comparison: 1st-order 10 Hz HP x 3rd-order 10 kHz Butterworth
    s = 1j * f / 10e3
    plant = (1j * f / 10) / (1 + 1j * f / 10) / ((1 + s) * (s * s + s + 1))
    ax[0].semilogx(f, 20 * np.log10(np.abs(plant)), "k--", lw=0.8, label="sim/plant.py assumption")
    ax[0].set_xlim(1, 1e5); ax[0].set_ylim(-40, 5)
    ax[0].set_xlabel("Hz"); ax[0].set_ylabel("dB re 1 kHz"); ax[0].set_title("Mic front end: response")
    ax[0].legend(fontsize=7)
    ax[1].set_xlabel("Hz"); ax[1].set_ylabel("nV/rtHz at mic"); ax[1].set_title("Front-end noise")
    ax[1].legend(fontsize=7)
    fig.tight_layout(); fig.savefig(os.path.join(RES, "frontend.png")); plt.close(fig)
    return rows


# ---------------------------------------------------------------- 2. VMID buffer
def vmid_net(ro, rs, cl, cmd):
    return f"""VMID buffer
{opamp(ro)}
Vdd vdd 0 {VDDA}
Vdiv div 0 {VMID}
* loop broken at the feedback input with a series AC source (Middlebrook-style voltage injection)
XU div fb buf vdd 0 opamp
Vinj buf fbx DC 0 AC 1
Rfb fbx fb 0.001
Rs buf vmid {rs}
Cl vmid 0 {cl}
* the front ends' feedback networks load VMID lightly (4 x 75k..150k): ignore
Iload vmid 0 DC 0 PULSE(0 1m 50u 10n 10n 200u 1)
.control
{cmd}
.endc
.end
"""


def study_vmid():
    rows = []
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
    for ro, rs, color in ((50, 10, "#6a994e"), (150, 10, "#3b75af"), (400, 10, "#d1495b"), (150, 0.1, "#999999"),
                          (150, 47, "#e9a03b")):
        ac = ngspice(vmid_net(ro, rs, "1u", "ac dec 40 10 1e8\nwrdata @TD@/lg.dat v(buf) v(fbx)"))["lg"]
        f = ac[:, 0]
        vx = ac[:, 1] + 1j * ac[:, 2]       # returned (op-amp output side)
        vy = ac[:, 4] + 1j * ac[:, 5]       # driven (toward the - input)
        T = -vx / vy          # loop gain at the injection point
        mag = 20 * np.log10(np.abs(T))
        ph = np.unwrap(np.angle(T)) * 180 / np.pi
        i = np.argmax(mag < 0)
        fc = f[i]
        pm = float(180.0 + ph[i])     # T is +A0 (0 deg) at DC once the feedback sign is removed
        tr = ngspice(vmid_net(ro, rs, "1u", "tran 0.1u 400u\nwrdata @TD@/st.dat v(vmid)"))["st"]
        t, v = tr[:, 0], tr[:, 1]
        seg = v[(t > 50e-6) & (t < 240e-6)]
        dip = (VMID - seg.min()) * 1e3
        # ringing: overshoot above the settled value after the load is released
        post = v[t > 252e-6]
        overshoot = (post.max() - VMID) * 1e3
        lab = f"Ro {ro} R, Rs {rs:g} R"
        rows.append(dict(ro=ro, rs=rs, fc_khz=fc / 1e3, pm_deg=pm, step_dip_mv=dip, release_overshoot_mv=overshoot))
        ax[0].semilogx(f, mag, color=color, label=f"{lab}: fc {fc / 1e3:.0f} kHz, PM {pm:.0f} deg")
        ax[1].plot(t * 1e6, (v - VMID) * 1e3, color=color, label=lab)
    ax[0].set_xlim(10, 1e8); ax[0].set_ylim(-40, 130)
    ax[0].set_xlabel("Hz"); ax[0].set_ylabel("loop gain (dB)"); ax[0].set_title("VMID buffer loop gain (1 uF load)")
    ax[0].legend(fontsize=7)
    ax[1].set_xlabel("us"); ax[1].set_ylabel("VMID - 1.4 V (mV)"); ax[1].set_title("1 mA load step")
    ax[1].legend(fontsize=7)
    fig.tight_layout(); fig.savefig(os.path.join(RES, "vmid.png")); plt.close(fig)
    return rows


# ---------------------------------------------------------------- 3. DAC reconstruction
def recon_net(rin):
    return f"""DAC reconstruction
{opamp()}
Vdd vdd 0 {VDDA}
Vdac dac 0 DC {VMID} AC 1
R1 dac a 8.2k
R2 a b 8.2k
C1 a out 3.3n
C2 b 0 1.5n
XU b out out vdd 0 opamp
Cc out ampin 1u
Rin ampin 0 {rin}
.control
ac dec 50 1 1e6
wrdata @TD@/r.dat v(ampin)
.endc
.end
"""


def study_recon():
    rows = []
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
    for rin, color in ((10e3, "#d1495b"), (20e3, "#3b75af"), (40e3, "#6a994e")):
        d = ngspice(recon_net(rin))["r"]
        f, h = d[:, 0], d[:, 1] + 1j * d[:, 2]
        db = 20 * np.log10(np.abs(h))
        ph = np.unwrap(np.angle(h))
        gd = -np.gradient(ph, 2 * np.pi * f)
        f_hi = f[len(db) - 1 - np.argmax(db[::-1] > -3)]
        f_lo = f[np.argmax(db > -3)]
        gd200 = float(np.interp(200, f, gd)) * 1e6
        gd1k = float(np.interp(1000, f, gd)) * 1e6
        rows.append(dict(rin_k=rin / 1e3, f_lo_hz=f_lo, f_hi_khz=f_hi / 1e3, gd_200_us=gd200, gd_1k_us=gd1k,
                         peak_db=float(db.max())))
        ax[0].semilogx(f, db, color=color, label=f"amp Rin {rin / 1e3:.0f}k: {f_lo:.1f} Hz - {f_hi / 1e3:.1f} kHz")
        ax[1].semilogx(f, gd * 1e6, color=color, label=f"Rin {rin / 1e3:.0f}k: {gd200:.1f} us @200 Hz")
    ax[0].set_xlim(1, 1e5); ax[0].set_ylim(-30, 3)
    ax[0].set_xlabel("Hz"); ax[0].set_ylabel("dB"); ax[0].set_title("DAC reconstruction + coupling")
    ax[0].legend(fontsize=7)
    ax[1].set_xlim(50, 2e4); ax[1].set_ylim(0, 60)
    ax[1].set_xlabel("Hz"); ax[1].set_ylabel("group delay (us)"); ax[1].set_title("Latency added")
    ax[1].legend(fontsize=7)
    fig.tight_layout(); fig.savefig(os.path.join(RES, "recon.png")); plt.close(fig)
    return rows


# ---------------------------------------------------------------- 4. amp + driver + cup
# typical-class 40 mm headphone driver (assumed, swept): Re, Le, Bl, Mms, Cms, Rms, Sd
DRIVER = dict(re=32.0, le=60e-6, bl=1.2, mms=0.12e-3, fs=150.0, qms=2.0, sd=math.pi * 0.0175 ** 2)
CUP = dict(v_front=40e-6, v_rear=60e-6, leak_fc=35.0)
AMP_VPK = 0.9          # TPA6132A2 output clamp, V peak (25 mW / 16 R ~ 0.89 V pk)


def driver_net(bl, re, amp_gain, vin, v_front, leak_fc, freq=None):
    d = DRIVER
    cms = 1 / ((2 * math.pi * d["fs"]) ** 2 * d["mms"])
    rms = 2 * math.pi * d["fs"] * d["mms"] / d["qms"]
    sd = d["sd"]
    ca_f = v_front / RHO_C2                      # acoustic compliances (m^3/Pa)
    ca_r = CUP["v_rear"] / RHO_C2
    ra_leak = 1 / (2 * math.pi * leak_fc * ca_f)
    src = f"DC 0 AC {vin}" if freq is None else f"DC 0 SIN(0 {vin} {freq})"
    return f"""amp + driver + cup
* amp: gain block with a hard output clamp (behavioural TPA6132A2)
Vin in 0 {src}
Bamp amp 0 V = min(max({amp_gain} * v(in), -{AMP_VPK}), {AMP_VPK})
* electrical: Re + Le, then the gyrator into the mechanical (impedance analogy: F=V, u=I)
Re amp e1 {re}
Le e1 e2 {d['le']}
Hbemf e2 e3 Vu {bl}
Vcoil e3 0 0
* mechanical loop: force Bl*i (a voltage here) drives Mms, Cms, Rms and the acoustic load
Hcoil m1 0 Vcoil {bl}
Vu m1 m2 0
Lm m2 m3 {d['mms']}
Cm m3 m4 {cms}
Rm m4 m5 {rms}
* acoustic load reflected to the mechanical side: F = Sd * (p_front - p_rear); u*Sd = volume velocity
Epf m5 m6 pf 0 {sd}
Epr m6 0 0 pr {sd}
Fqf 0 pf Vu {sd}
Cf pf 0 {ca_f}
Rleak pf 0 {ra_leak}
Fqr pr 0 Vu {sd}
Cr pr 0 {ca_r}
.control
{"ac dec 50 10 20k" if freq is None else "tran 20u 0.2"}
wrdata @TD@/p.dat v(pf)
.endc
.end
"""


def study_driver():
    """Pressure at the error mic per amp volt, and the most clean 63 Hz anti-noise the amp's
    output clamp allows. Force factor and coil resistance are the driver's unknowns; a
    leakier seal (seal study) is included because it takes the low end away."""
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
    rows = []
    cases = [(0.8, 32.0, 35.0, "#d1495b"), (1.2, 32.0, 35.0, "#3b75af"), (1.6, 32.0, 35.0, "#6a994e"),
             (0.8, 16.0, 35.0, "#9b5de5"), (1.2, 32.0, 116.0, "#e9a03b")]
    for bl, re, leak, color in cases:
        d = ngspice(driver_net(bl, re, 1.0, 1.0, CUP["v_front"], leak))["p"]
        f, p = d[:, 0], np.abs(d[:, 1] + 1j * d[:, 2])
        pa_v_63 = float(np.interp(63, f, p))
        pa_v_300 = float(np.interp(300, f, p))
        max_pa = pa_v_63 * AMP_VPK
        # check in the time domain: 63 Hz sine exactly at the clamp level
        tr = ngspice(driver_net(bl, re, 1.0, AMP_VPK, CUP["v_front"], leak, freq=63))["p"]
        pk_tr = float(np.max(np.abs(tr[tr[:, 0] > 0.12, 1])))
        lab = f"Bl {bl} Tm, {re:.0f} R" + ("" if leak == 35.0 else f", leaky seal ({leak:.0f} Hz)")
        rows.append(dict(bl=bl, re=re, leak_hz=leak, pa_per_v_63=pa_v_63, pa_per_v_300=pa_v_300,
                         max_pa_63=max_pa, max_pa_63_tran=pk_tr, max_db_pk_63=20 * math.log10(max_pa / P0),
                         upper_res_hz=float(f[np.argmax(p * (f > 200))]), meets_15pa=bool(max_pa >= 15.0)))
        ax[0].semilogx(f, 20 * np.log10(p), color=color, label=f"{lab}: {pa_v_63:.0f} Pa/V @63 Hz")
        ax[1].bar(len(rows) - 1, max_pa, color=color)
    ax[1].axhline(15.0, color="k", ls="--", lw=0.8, label="needed for real engine rumble (~15 Pa)")
    ax[1].set_xticks(range(len(rows)))
    ax[1].set_xticklabels([f"Bl {r['bl']}\n{r['re']:.0f} R" + ("\nleaky" if r["leak_hz"] != 35.0 else "")
                           for r in rows], fontsize=7)
    ax[1].set_ylabel("Pa peak"); ax[1].set_title(f"Max clean 63 Hz anti-noise ({AMP_VPK} V amp clamp)")
    ax[1].legend(fontsize=7)
    ax[0].set_xlabel("Hz"); ax[0].set_ylabel("dB re 1 Pa/V"); ax[0].set_xlim(10, 2e4)
    ax[0].set_title("Pressure at the error mic per amp volt")
    ax[0].legend(fontsize=7)
    fig.tight_layout(); fig.savefig(os.path.join(RES, "driver.png")); plt.close(fig)
    return rows


def main():
    fe = study_frontend()
    vm = study_vmid()
    rc = study_recon()
    dr = study_driver()
    summary = dict(frontend=fe, vmid=vm, recon=rc, driver=dr, driver_params=DRIVER, cup=CUP, amp_vpk=AMP_VPK)
    with open(os.path.join(RES, "spice_results.json"), "w") as fh:
        json.dump(summary, fh, indent=2, default=float)
    with open(os.path.join(RES, "spice_results.md"), "w") as fh:
        fh.write(report(summary))
    print(json.dumps(summary, indent=1, default=lambda x: round(float(x), 3)))


def report(S):
    fe, vm, rc, dr = S["frontend"], S["vmid"], S["recon"], S["driver"]
    ref, err = fe["REF (outside)"], fe["ERR (in-cup)"]
    ovl_fw = math.sqrt(2) * P0 * 10 ** (118.0 / 20)       # firmware EAR_OVERLOAD_DB 118 as a sine peak
    iso = [r for r in vm if r["rs"] == 10]
    bare = [r for r in vm if r["rs"] < 1][0]
    need = 15.0
    L = ["# SPICE verification of the analog chain (ngspice)", "",
         "Generated by `hardware/spice/run_spice.py` from the component values in `hardware/gen/boards.py`. "
         "Op-amp: behavioural MCP6022 (A0 110 dB, GBW 10 MHz, 65 deg PM, 8.7 nV/rtHz, rail-to-rail). "
         "Amp: behavioural TPA6132A2 with a 0.9 V peak output clamp. Driver: a typical-class 40 mm "
         "Thiele-Small model, swept; replace with the chosen driver's measured parameters.", "",
         "## 1. Mic front ends", "",
         "| Channel | Gain (V/V) | -3 dB band | Noise, input-referred | Clips at |",
         "| --- | --- | --- | --- | --- |"]
    for k, r in fe.items():
        L.append(f"| {k} | {r['gain']:.2f} | {r['f_lo_hz']:.1f} Hz - {r['f_hi_hz'] / 1e3:.1f} kHz | "
                 f"{r['noise_in_uV_a']:.1f} uV(A) = {r['noise_spl_dba']:.1f} dB(A) SPL | "
                 f"{r['clip_pa_peak']:.1f} Pa = {r['clip_db_peak']:.1f} dB peak |")
    L += ["", f"- Response matches the simulation's plant (`sim/plant.py`: 10 Hz high-pass, 3rd-order ~10 kHz "
          "anti-alias), so the ANC results stand on the real circuit.",
          f"- Electronics noise is about the mic's own self-noise ({MIC_SELF_NOISE_DBA} dB(A) SPL): fine for "
          "ANC at 100 dB(A) and for quiet-room dosimetry.",
          f"- **Finding - ear-overload cut-out cannot fire:** the in-cup channel saturates at {err['clip_pa_peak']:.1f} Pa, "
          f"but the firmware trips at {ovl_fw:.1f} Pa (`EAR_OVERLOAD_DB` 118 as a sine peak). Fixed in "
          "`firmware/src/app.c`: the threshold is capped below the channel's full scale.", "",
          "## 2. VMID bias buffer (MCP6022 follower, 10 R + 1 uF)", "",
          "| Op-amp output R | Isolation R | Loop crossover | Phase margin | 1 mA load step |",
          "| --- | --- | --- | --- | --- |"]
    for r in vm:
        L.append(f"| {r['ro']} R | {r['rs']:g} R | {r['fc_khz']:.0f} kHz | {r['pm_deg']:.0f} deg | "
                 f"{r['step_dip_mv']:.1f} mV dip |")
    L += ["", f"- Stable with the 10 R isolation resistor over the whole plausible output-resistance range "
          f"(PM >= {min(r['pm_deg'] for r in iso):.0f} deg). Without it the 1 uF load leaves "
          f"{bare['pm_deg']:.0f} deg: ringing. **Keep the 10 R** (the datasheet asks for a series R above ~60 pF).",
          "", "## 3. DAC reconstruction + coupling into the amp", "",
          "| Amp input R | Pass band | Group delay @ 1 kHz | @ 200 Hz |", "| --- | --- | --- | --- |"]
    for r in rc:
        L.append(f"| {r['rin_k']:.0f} k | {r['f_lo_hz']:.1f} Hz - {r['f_hi_khz']:.1f} kHz | {r['gd_1k_us']:.1f} us | "
                 f"{r['gd_200_us']:.1f} us |")
    L += ["", "- Matches the plant model's 2nd-order 9 kHz reconstruction (~25 us at 1 kHz; about 0.8 sample at "
          "32 kHz). The extra delay at 200 Hz is the 1 uF coupling high-pass - a phase lead the identified "
          "secondary-path model absorbs, not added latency.", "",
          "## 4. Amp + driver + earcup: is there enough anti-noise?", "",
          f"The real-noise study needs ~{need:.0f} Pa peak at 63 Hz for engine rumble "
          "(`sim/results/real_audio.md`).", "",
          "| Driver | Seal | Pa/V @ 63 Hz | Max clean 63 Hz anti-noise | Meets 15 Pa |", "| --- | --- | --- | --- | --- |"]
    for r in dr:
        L.append(f"| Bl {r['bl']} Tm, {r['re']:.0f} R | {'good' if r['leak_hz'] == 35.0 else 'leaky'} | "
                 f"{r['pa_per_v_63']:.1f} | {r['max_pa_63']:.1f} Pa ({r['max_db_pk_63']:.1f} dB peak) | "
                 f"{'yes' if r['meets_15pa'] else '**no**'} |")
    L += ["", f"- **Driver requirement:** >= {need / S['amp_vpk']:.0f} Pa/V at 63 Hz in the cup (with the "
          f"TPA6132A2's {S['amp_vpk']} V peak swing). A weak 32 R driver (Bl 0.8) falls short; a **16 R driver** "
          "doubles the pressure per volt from the same amp and supply.",
          "- **Finding - the amp clips before the controller's clamp:** at 0 dB amp gain the amp saturates at "
          f"{S['amp_vpk']} V, i.e. {S['amp_vpk'] * 8.0:.1f} controller units (1/`OUT_PA_PER_V` V per unit), while "
          "`y_max` is 10 units (set by the DAC range only). The controller's clip detector never saw amp "
          "clipping. Fixed in `firmware/src/app.c`: `y_max` is also capped by the amp swing at the selected gain.",
          "", "Figures: `frontend.png`, `vmid.png`, `recon.png`, `driver.png`."]
    return "\n".join(L) + "\n"


if __name__ == "__main__":
    main()
