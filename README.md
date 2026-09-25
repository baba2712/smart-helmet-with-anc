# ANC Safety Helmet

An industrial safety helmet with **hybrid active noise cancellation** (feed-forward + adaptive feedback FxLMS) on an STM32H743. Target: take ~100 dB(A) plant noise down to **≤ 75 dB(A) at the ear** (passive earcup + ANC), below the 85 dB(A) 8-hour limit. Also a built-in **noise dosimeter**, **hear-through** for speech and alarms, a **seal monitor** that measures the cup's attenuation while it is worn and warns when the seal leaks (glasses, hair, a tilted helmet), and an optional **BLE** link.

> Status: **prototype design, not certified PPE.** Nothing has been built or measured on real hardware yet. See [Status](#status).

## Simulated results (100 dB(A) ambient)

| Noise | Open ear | Passive cup | Passive + ANC | Total reduction |
| --- | --- | --- | --- | --- |
| Motor / fan (tonal) | 100.0 | 80.7 | **72.3 dB(A)** | 27.7 dB |
| Compressor | 99.9 | 77.3 | **73.6 dB(A)** | 26.4 dB |
| Broadband (pink) | 99.9 | 72.4 | **69.1 dB(A)** | 30.8 dB |

ANC adds 3–8.5 dB(A) on top of the passive cup; most gain is on tonal machine noise (80–400 Hz). Known limitation: energy below 63 Hz is not reduced. Full report: [`sim/results/summary.md`](sim/results/summary.md).

On **70 real field recordings** (engine, chainsaw, hand saw, helicopter, train, vacuum cleaner, washing machine; ESC-50) the level at the ear ends at 65–75 dB(A): [`sim/results/real_audio.md`](sim/results/real_audio.md). Engine rumble needs ~15 Pa of anti-noise at 63 Hz, which sets a driver requirement (below).

**Seal monitor** (10 noises × 4 seal conditions, one factory baseline): 23/23 leaks that cost ≥ 3 dB flagged in 3–5 s, 0 false alarms: [`sim/results/seal_monitor.md`](sim/results/seal_monitor.md).

## Repository

| Folder | What's in it |
| --- | --- |
| `sim/` | Python plant + noise models, the reference hybrid-FxLMS controller, and the full study (`run_all.py`). Also generates firmware tuning headers. `real_audio.py` runs it on real recordings; `seal_ref.py` / `seal_monitor.py` are the seal monitor's reference and study. |
| `firmware/` | Bare-metal C for STM32H743: 32 kHz ADC→DSP→DAC path, FxLMS core, path identification, dosimeter, hear-through, USB CDC (TinyUSB), BLE UART, flash storage, power/UI. `test/` holds firmware-in-the-loop tests against the sim. |
| `hardware/` | KiCad 7 projects (open fine in KiCad 8/9) for three boards, generated from `hardware/gen/` (Python circuit descriptions → schematic → PCB → Freerouting → fab files). `spice/` verifies the analog chain in ngspice. |
| `mechanical/` | OpenSCAD earcup inserts, driver baffle, outside-mic pod, light pipe / button plunger. Parametric; **measure your earmuff** and edit `params.scad`. `check_fit.py` cross-checks the parts against the PCBs and the cup. |
| `tools/` | `anc_tool.py`: host tool for the USB/BLE command link (status, tuning, calibration, driver test, dose log). `acceptance_test.py`: the per-helmet acceptance / TRL-6 evidence run. |
| `docs/` | Architecture notes; [`bringup_and_test.md`](docs/bringup_and_test.md): order, build, bring-up, calibration and the TRL-6 demonstration. |

## Boards

| Board | Layers | Size | State |
| --- | --- | --- | --- |
| `main-board`: MCU, power, charger, left-ear analog, amp | 4 | 60 × 50 mm, 14 mm corners | Routed; DRC 0 unconnected, 0 electrical errors; fab files in `fab/` |
| `satellite-board`: right-ear mic front ends | 2 | 34 × 26 mm | Routed; DRC 0 errors (silk warnings only); fab files in `fab/` |
| `mic-board`: IM73A135 MEMS mic (×4 per helmet) | 2 | 11 × 13.2 mm | Routed; DRC clean; fab files in `fab/` |

## Build and test

```bash
git clone --recursive <this repo>
pip install -r sim/requirements.txt

python3 sim/run_all.py                 # simulation study + regenerates firmware/inc/anc_tuning.h
python3 sim/real_audio.py --fetch      # ANC on real recordings (downloads ~30 MB of ESC-50 clips)
python3 sim/seal_monitor.py            # seal-monitor study (uses the recordings if present)
cd firmware && make                    # needs arm-none-eabi-gcc; -> build/anc_helmet.bin (~79 KB)
python3 test/test_fil.py               # firmware DSP vs simulation (all pass)

cd .. && python3 hardware/spice/run_spice.py   # analog chain in ngspice
python3 mechanical/check_fit.py                # parts vs PCBs vs cup

cd hardware/gen && python3 build.py sch                           # regenerate schematics
python3 pcb_build.py place main-board && python3 pcb_build.py route main-board
python3 pcb_build.py finish main-board && python3 pcb_build.py tie main-board
python3 pcb_build.py fab main-board
```

`pcb_build.py` needs KiCad 7's Python module (Ubuntu: `apt install kicad kicad-footprints kicad-symbols`, run with the system `python3`) and Freerouting 2.x at `../../tools-dl/freerouting2.jar` (or `FREEROUTING_JAR`).

Verified so far:
- Firmware compiles with `-Werror`: 78.8 KB flash, 38 KB DTCM, 4 KB ITCM.
- Firmware-in-the-loop tests all pass. C matches the Python reference to 0.0 dB(A) on all three noise types, and the output matches to about 1e-4 relative. The seal monitor matches its reference to < 0.001 dB with identical decisions.
- Path identification in a quiet room: 16 dB fit, 0.6% model error.
- Dosimeter: a 94 dB calibrator tone reads 93.99 dB(A); NIOSH dose math checks out.
- SPICE ([`hardware/spice/results/spice_results.md`](hardware/spice/results/spice_results.md)): mic front ends match the simulation's plant, noise ~23 dB(A) SPL; VMID buffer 85–87° phase margin (keep the 10 Ω isolation resistor); DAC reconstruction ~26 µs at 1 kHz.
- Mechanical fit: all checks pass; the main board clears the cup wall by only 0.45 mm (measure your earmuff).

## Status

Done:
- [x] Simulation study and tuned parameters
- [x] Firmware (compiles; DSP verified in the loop against the sim)
- [x] Schematics for all three boards
- [x] Satellite and mic boards routed, with Gerber, BOM and CPL files for JLCPCB
- [x] Real-recording ANC study, SPICE verification of the analog chain, mechanical fit check
- [x] Seal monitor (reference, study, firmware, FIL test)
- [x] Main board fully routed (Freerouting + `hardware/gen/finish_route.py`), fab files for all three boards
- [x] Printable STLs (`mechanical/stl/`, nominal earmuff), driver acceptance test in firmware, acceptance-test runner

Remaining:
- [ ] **Order and build** following [`docs/bringup_and_test.md`](docs/bringup_and_test.md).
- [ ] **Driver:** buy 2-3 candidate 40 mm drivers (16 Ω preferred) and keep one that passes `anc_tool.py driver-test` (≥ 17 Pa/V at 63 Hz in the cup).
- [ ] Open all three boards in KiCad 8/9 for a last ERC/DRC and 3D look before ordering (KiCad 7 DRC here is clean apart from two reviewed connector-courtyard overlaps).
- [ ] Measure the real earmuff, update `mechanical/params.scad`, re-run `check_fit.py`, export STLs.
- [ ] Calibrate and accept each helmet: `python3 tools/acceptance_test.py`, then the TRL-6 run (docs/bringup_and_test.md §6).
- [ ] Bring-up and acoustic test on real hardware: SPL meter plus a 94 dB calibrator.

## Safety

This is a research prototype. It is **not certified hearing protection.** Real PPE needs testing to EN 352 / ANSI S3.19 (or IS 9167 in India). The firmware fails safe: it drops to passive-only on divergence, output clipping, fault or low battery. The passive cup still has to protect on its own.

## License

MIT, © Khaja Baba Shaik.
