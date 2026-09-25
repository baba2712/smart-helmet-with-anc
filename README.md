# ANC Safety Helmet

An industrial safety helmet with **hybrid active noise cancellation** (feed-forward + adaptive feedback FxLMS) on an STM32H743. Target: take ~100 dB(A) plant noise down to **≤ 75 dB(A) at the ear** (passive earcup + ANC), below the 85 dB(A) 8-hour limit. Also a built-in **noise dosimeter**, **hear-through** for speech and alarms, and an optional **BLE** link.

> Status: **prototype design, not certified PPE.** Nothing has been built or measured on real hardware yet. See [Status](#status).

## Simulated results (100 dB(A) ambient)

| Noise | Open ear | Passive cup | Passive + ANC | Total reduction |
| --- | --- | --- | --- | --- |
| Motor / fan (tonal) | 100.0 | 80.7 | **72.3 dB(A)** | 27.7 dB |
| Compressor | 99.9 | 77.3 | **73.6 dB(A)** | 26.4 dB |
| Broadband (pink) | 99.9 | 72.4 | **69.1 dB(A)** | 30.8 dB |

ANC adds 3–8.5 dB(A) on top of the passive cup; most gain is on tonal machine noise (80–400 Hz). Known limitation: energy below 63 Hz is not reduced. Full report: [`sim/results/summary.md`](sim/results/summary.md).

## Repository

| Folder | What's in it |
| --- | --- |
| `sim/` | Python plant + noise models, the reference hybrid-FxLMS controller, and the full study (`run_all.py`). Also generates firmware tuning headers. |
| `firmware/` | Bare-metal C for STM32H743: 32 kHz ADC→DSP→DAC path, FxLMS core, path identification, dosimeter, hear-through, USB CDC (TinyUSB), BLE UART, flash storage, power/UI. `test/` holds firmware-in-the-loop tests against the sim. |
| `hardware/` | KiCad 7 projects (open fine in KiCad 8/9) for three boards, generated from `hardware/gen/` (Python circuit descriptions → schematic → PCB → Freerouting → fab files). |
| `mechanical/` | OpenSCAD earcup inserts, driver baffle, outside-mic pod, light pipe / button plunger. Parametric; **measure your earmuff** and edit `params.scad`. |
| `tools/` | `anc_tool.py`: host tool for the USB/BLE command link (status, tuning, calibration, dose log). |
| `docs/` | Architecture notes. |

## Boards

| Board | Layers | Size | State |
| --- | --- | --- | --- |
| `main-board`: MCU, power, charger, left-ear analog, amp | 4 | 60 × 50 mm, 14 mm corners | Schematic done; **placed, not yet routed** |
| `satellite-board`: right-ear mic front ends | 2 | 34 × 26 mm | Routed; DRC 0 errors (silk warnings only); fab files in `fab/` |
| `mic-board`: IM73A135 MEMS mic (×4 per helmet) | 2 | 11 × 13.2 mm | Routed; DRC clean; fab files in `fab/` |

## Build and test

```bash
git clone --recursive <this repo>
pip install -r sim/requirements.txt

python3 sim/run_all.py                 # simulation study + regenerates firmware/inc/anc_tuning.h
cd firmware && make                    # needs arm-none-eabi-gcc; -> build/anc_helmet.bin (~76 KB)
python3 test/test_fil.py               # firmware DSP vs simulation (all pass)

cd ../hardware/gen && python3 build.py sch                       # regenerate schematics
python3 pcb_build.py place main-board && python3 pcb_build.py route main-board
python3 pcb_build.py finish main-board && python3 pcb_build.py fab main-board
```

Verified so far:
- Firmware compiles with `-Werror`: 75.6 KB flash, 36 KB DTCM, 3.5 KB ITCM.
- Firmware-in-the-loop tests all pass. C matches the Python reference to 0.0 dB(A) on all three noise types, and the output matches to about 1e-4 relative.
- Path identification in a quiet room: 16 dB fit, 0.6% model error.
- Dosimeter: a 94 dB calibrator tone reads 93.99 dB(A); NIOSH dose math checks out.

## Status

Done:
- [x] Simulation study and tuned parameters
- [x] Firmware (compiles; DSP verified in the loop against the sim)
- [x] Schematics for all three boards
- [x] Satellite and mic boards routed, with Gerber, BOM and CPL files for JLCPCB

Remaining:
- [ ] **Main board routing:** run the `route` + `finish` + `fab` steps above, or route it in KiCad. Placement is fixed for the elliptical cup.
- [ ] Open all three boards in KiCad 8/9 and run full ERC/DRC plus a 3D fit check before ordering.
- [ ] `mechanical/check_fit.py` (board-vs-cup fit check); regenerate STLs after measuring the real earmuff.
- [ ] Bring-up and acoustic test on real hardware: SPL meter plus a 94 dB calibrator.

## Safety

This is a research prototype. It is **not certified hearing protection.** Real PPE needs testing to EN 352 / ANSI S3.19 (or IS 9167 in India). The firmware fails safe: it drops to passive-only on divergence, output clipping, fault or low battery. The passive cup still has to protect on its own.

## License

MIT, © Khaja Baba Shaik.
