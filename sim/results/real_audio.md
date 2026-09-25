# ANC on real recorded noise (ESC-50 clips)

10 clips per category, silence-trimmed and joined into 20 s, scaled to 100 dB(A) outside. Levels at the ear over the last 5 s; same plant, factory calibration and tuned controller as `run_all.py`.

| Noise | Open ear | Passive cup | Passive + ANC | ANC adds | Best 1/3-oct ANC gain, 80-500 Hz | Watchdog trips | + ANC, 20 Pa headroom (trips) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| engine | 99.7 | 78.3 | **74.8** | 3.6 dB | 8.9 dB | 6 | 73.9 (0) |
| chainsaw | 100.6 | 73.4 | **69.0** | 4.3 dB | 17.5 dB | 0 | 69.0 (0) |
| hand_saw | 99.8 | 69.5 | **68.8** | 0.8 dB | 12.5 dB | 0 | 68.8 (0) |
| helicopter | 97.3 | 72.7 | **69.9** | 2.7 dB | 9.9 dB | 0 | 69.9 (0) |
| train | 101.3 | 78.6 | **71.8** | 6.7 dB | 17.3 dB | 2 | 71.8 (0) |
| vacuum_cleaner | 98.0 | 73.7 | **65.3** | 8.4 dB | 27.0 dB | 0 | 65.3 (0) |
| washing_machine | 102.1 | 74.9 | **68.4** | 6.5 dB | 27.9 dB | 0 | 68.4 (0) |

Watchdog trips are all from output clipping (the anti-noise wanted more than the 10 Pa clamp, i.e. more than the driver/amp is assumed to deliver), not from divergence: every trip resets the filters and costs ~1 dB. Low-frequency engine rumble needs ~15 Pa peak at the ear. With 20 Pa of headroom there are no trips; whether the TPA6132A2 + 40 mm driver can deliver that is the hardware question the SPICE study answers.

Source: ESC-50, K. J. Piczak, CC BY-NC 3.0 (https://github.com/karolpiczak/ESC-50). Consumer field recordings: content below ~50 Hz is under-represented.
