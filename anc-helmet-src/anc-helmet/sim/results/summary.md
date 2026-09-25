# Simulation results

Ambient 100 dB(A) outside the helmet, fs = 32000 Hz, secondary-path low-frequency group delay 9.0 samples. Levels at the ear, last 5 s of a 8 s run.

| Noise | Open ear dB(A) | Passive dB(A) | Passive + ANC dB(A) | ANC adds | Total reduction | Watchdog trips |
| --- | --- | --- | --- | --- | --- | --- |
| motor | 99.9 | 80.5 | 72.1 | 8.4 dB | **27.8 dB** | 0 |
| compressor | 99.9 | 77.0 | 74.0 | 3.0 dB | **25.9 dB** | 0 |
| pink | 99.9 | 72.3 | 69.0 | 3.3 dB | **30.9 dB** | 0 |

## Known limitation: sub-63 Hz rumble

The update is weighted toward 150 Hz-1 kHz, where the A-weighted (hearing-damage) dose is. Energy below 63 Hz is left roughly as the passive cup leaves it:

| Noise | Passive <=63 Hz (dB, unweighted) | Passive + ANC <=63 Hz |
| --- | --- | --- |
| motor | 95.7 | 96.4 |
| compressor | 100.7 | 102.4 |
| pink | 88.3 | 90.7 |

Figures: `paths.png`, `spectra_*.png`, `convergence.png`, `latency_sweep.png`, `robustness.png`.
