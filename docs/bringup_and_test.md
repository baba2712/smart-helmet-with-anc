# Build, bring-up and TRL-6 demonstration

What it takes to go from this repository to a demonstrated prototype. TRL 6 means the
system prototype works in a relevant environment, so for this helmet that means real
industrial-level noise, a real or simulated head, and measured results.
Everything up to "Order" is done in the repository. The rest needs hands and hardware.

## 1. Order the boards (JLCPCB)

| Board | Files | Settings |
| --- | --- | --- |
| main (×1 per helmet) | `hardware/main-board/fab/main-board-gerbers-jlcpcb.zip`, `-bom-jlcpcb.csv`, `-cpl-jlcpcb.csv` | 4 layers, 1.6 mm, JLC04161H-7628 stack-up, **ENIG** (LQFP-100 and 0.5 mm WQFN pads), min track/gap 0.127 mm, min via 0.45/0.2 mm |
| satellite (×1) | `hardware/satellite-board/fab/…` | 2 layers, 1.6 mm, HASL lead-free is fine |
| mic (×4) | `hardware/mic-board/fab/…` | 2 layers, **1.0 mm** so the MEMS sound port is short; ENIG |

Assembly: upload the BOM and CPL. The BOM has values, dielectrics and exact MPNs but no
LCSC numbers on purpose. Let JLCPCB's matcher propose parts, and check each proposal
against the value, package and MPN column before you confirm. A wrong LCSC number means a
wrong part soldered. Parts on the bottom side (D4–D6 status LEDs, SW1 power button) need
two-sided assembly, or solder them by hand.

Hand-solder: the JST connectors if the matcher has no stock, the battery lead and the drivers.

## 2. Buy

| Item | Qty | Notes |
| --- | --- | --- |
| 40 mm headphone drivers, 16 Ω preferred | 2-3 candidate models × 2 | Qualified in the cup with `test driver` (step 5): need ≥ 17 Pa/V at 63 Hz. |
| 103450 Li-ion pouch, 2000 mAh, with protection PCB | 1 | 50 × 34 × 10.5 mm max; JST PH 2.0 lead |
| Helmet-mount earmuffs (SNR ~27 dB) | 1 pair | The passive cup the ANC adds to |
| Industrial safety helmet with earmuff slots | 1 | |
| JST SH 1.0 mm cables (2, 4, 6 pin) | set | mics, speakers, left-right cable |
| M2 heat-set inserts + M2 × 4 screws | 5 + 5 | |
| ST-Link V2/V3 + 2×5 1.27 mm cable | 1 | SWD flashing |
| Class 1/2 SPL meter, 94 dB 1 kHz calibrator | 1 each | measurement reference |

## 3. Mechanical

1. Remove the earmuff's foam insert and measure the cup inside with calipers: long axis,
   short axis, depth, and wall thickness where the outside mic goes.
2. Put the numbers in `mechanical/params.scad`, then run `python3 mechanical/check_fit.py`.
   It must pass. The main board has only 0.45 mm of clearance in the nominal cup.
3. Run `mechanical/export_stl.sh`. Print in PETG or ASA with 0.2 mm layers and 3 perimeters.
   The light pipe is clear PETG, the button plunger TPU.

## 4. Bring-up (before the battery)

1. Look for solder bridges on U5 (LQFP-100), U10 (WQFN-16) and U3 (QFN).
2. Put a bench supply on the battery connector J2: **3.7 V, 100 mA current limit**.
   Current should settle below about 60 mA with no firmware.
3. Rails: 3V3 = 3.30 V ± 3 %, 2V8A = 2.80 V ± 2 %, VMID = 1.40 V ± 20 mV.
4. Flash over SWD: `cd firmware && make flash`, or `make dfu` over USB with BOOT held.
5. Connect USB-C. The charge LED should light, and `python3 tools/anc_tool.py status` should
   print one JSON line.
6. Connect the battery and check that `vbat` reads within 50 mV of a multimeter.

## 5. Calibrate and accept (per helmet)

```bash
python3 tools/acceptance_test.py --serial proto-01
```

It runs, prompting the operator where needed:

| Step | What | Pass |
| --- | --- | --- |
| health | firmware reply, battery, CPU load, worst ISR time | CPU ≤ 80 %, ISR ≤ 25 µs |
| mics | 94 dB calibrator on each of the 4 mics, trims stored | trim within ± 6 dB |
| paths | factory speaker-path identification, quiet room | fit ≥ 10 dB |
| driver | 63 Hz tone, in-cup Pa/V per ear | ≥ 17 Pa/V |
| seal_learn | factory seal baseline, good fit, broadband noise | baseline stored |
| noise | ~100 dB(A) industrial noise: passive vs ANC | ear ≤ 80 dB(A), ANC ≥ 3 dB(A), helmet vs meter ≤ 2 dB |
| seal | glasses temple under a cushion, then reseat | flagged ≤ 15 s, cleared ≤ 20 s |
| soak | 10 min ANC in noise | no trips, overruns or overloads |

The report goes to `reports/<serial>-<date>/acceptance.md` and `.json`. Every limit names
the study it comes from.

## 6. TRL-6 demonstration

**Relevant environment:** broadband industrial noise at 95–105 dB(A). Use a recording of real
plant noise (motors, compressors, a press line) through a PA speaker in a small room, or run
it on site. **Test head:** an acoustic test fixture (a head-and-torso simulator, or a DIY head
with an ear-simulator mic) rather than a person. Hearing protection is the device under test,
so nobody should rely on it at 100 dB(A) before it has passed.

Evidence pack:
1. Acceptance reports for two helmets (repeatability).
2. Ear level with ANC on and off, from the helmet's error mic **and** from the fixture's
   ear mic, for three noise types.
3. Seal monitor with three glasses frames (thin, medium, thick): time to flag, and the
   measured drop against the fixture's measured loss.
4. A 2-hour run on battery: dose log download (`anc_tool.py log`) against a reference dosimeter.
5. Photos and video of the setup, the firmware version and the git commit.

**After TRL 6** (for PPE certification, TRL 7+): lab testing of the passive cup and then the
active system to EN 352-4/-5 / ANSI S12.42 / IS 9167, and the electrical safety and EMC
tests for a battery product.
