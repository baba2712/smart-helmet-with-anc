# Seal-aware protection: self-measured attenuation while worn

Ambient 100 dB(A). Four seal conditions (passive insertion loss scaled to 100 %, 85 %, 70 %, 55 %) x 10 noises (3 synthetic + real ESC-50 recordings). The leak also raises the driver's low-frequency leak corner (35 Hz / seal^2: 35, 48, 71, 116 Hz), so the factory secondary-path model is wrong under a leak, as it would be on a real head. Estimates use only signals the firmware already has (outside mic, error mic, controller output, factory S_hat/F_hat), 1 s windows. Detection uses the firmware's logic (`seal_ref.py`): octave bands 250 Hz-2 kHz against **one** factory baseline measured once (good seal, pink noise: 250 Hz 17.0 dB, 500 Hz 24.6 dB, 1000 Hz 31.0 dB, 2000 Hz 34.7 dB), used for every noise.

- **Passive-attenuation estimate error:** mean +0.24 dB, worst 1.04 dB (estimate vs. the plant's true attenuation on the same noise).
- **Leak detection** (band attenuation 3 dB below the factory baseline): 23 of 23 leaks that really cost >= 3 dB flagged, 0 false alarms on 10 good-seal runs, flagged 3-5 s after switch-on.
- **What a one-time fit test misses:** it certifies the good-seal level. A thick glasses temple later in the shift raises the real dose 3.9-8.7x, a poor fit 9.2-26.5x (NIOSH 3 dB exchange), with ANC running. Here it is measured continuously (1 s windows) and flagged.
- **ANC cannot make up for a leak:** its average gain falls from 4.3 -> 3.3 -> 2.1 -> 1.4 dB(A) as the seal worsens, and watchdog resets rise (4 -> 24 -> 57 -> 108 across all noises), because the driver's own low-frequency pressure leaks out too. Re-identifying the secondary path on the head was tested and does not restore it on tonal noise. The only fix is the wearer reseating the cup, so the product's response to a flag is an alert (tone + LED + log), not silent compensation.

| Noise | Seal | True passive IL (A) | Estimated IL (A) | True loss | Band drop vs factory | Flagged (s) | At ear, ANC on | ANC adds | Measured PAR | Dose vs fit |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| motor | good seal | 19.4 | 20.5 | 0.0 | 1.1 | no | 72.4 | 8.2 | 27.6 | 1.0x |
| motor | thin glasses temple | 17.4 | 18.0 | 2.0 | 4.7 | yes (3) | 75.6 | 7.0 | 24.4 | 2.1x |
| motor | thick glasses temple | 15.1 | 15.3 | 4.3 | 8.3 | yes (3) | 81.5 | 3.4 | 18.5 | 8.2x |
| motor | poor fit / hair | 12.6 | 12.5 | 6.8 | 12.0 | yes (3) | 85.5 | 1.9 | 14.5 | 20.5x |
| compressor | good seal | 22.8 | 23.0 | 0.0 | 0.6 | no | 74.3 | 2.9 | 25.7 | 1.0x |
| compressor | thin glasses temple | 20.7 | 20.8 | 2.2 | 4.3 | yes (3) | 76.5 | 2.8 | 23.5 | 1.7x |
| compressor | thick glasses temple | 18.1 | 18.0 | 4.7 | 8.0 | yes (3) | 80.2 | 1.6 | 19.8 | 3.9x |
| compressor | poor fit / hair | 15.1 | 14.9 | 7.7 | 11.8 | yes (3) | 84.2 | 0.6 | 15.8 | 9.9x |
| pink | good seal | 27.5 | 27.9 | 0.0 | -0.0 | no | 69.2 | 3.2 | 30.7 | 1.0x |
| pink | thin glasses temple | 24.5 | 24.8 | 2.9 | 3.8 | yes (3) | 73.2 | 2.2 | 26.7 | 2.5x |
| pink | thick glasses temple | 21.1 | 21.3 | 6.3 | 7.6 | yes (3) | 77.6 | 1.2 | 22.3 | 6.9x |
| pink | poor fit / hair | 17.2 | 17.4 | 10.3 | 11.4 | yes (3) | 82.2 | 0.5 | 17.7 | 20.2x |
| engine | good seal | 23.9 | 23.2 | 0.0 | 0.3 | no | 75.6 | -1.0 | 23.0 | 1.0x |
| engine | thin glasses temple | 21.7 | 20.8 | 2.2 | 4.1 | yes (3) | 80.0 | -3.1 | 18.6 | 2.7x |
| engine | thick glasses temple | 19.0 | 18.1 | 4.9 | 7.9 | yes (3) | 82.6 | -3.1 | 15.9 | 5.1x |
| engine | poor fit / hair | 15.7 | 15.1 | 8.3 | 11.8 | yes (3) | 85.2 | -2.3 | 13.4 | 9.2x |
| chainsaw | good seal | 26.1 | 26.4 | 0.0 | -0.1 | no | 68.7 | 4.6 | 30.6 | 1.0x |
| chainsaw | thin glasses temple | 22.9 | 23.1 | 3.2 | 3.8 | yes (3) | 73.1 | 3.3 | 26.2 | 2.8x |
| chainsaw | thick glasses temple | 19.4 | 19.7 | 6.6 | 7.6 | yes (3) | 77.6 | 2.2 | 21.7 | 7.9x |
| chainsaw | poor fit / hair | 15.7 | 16.2 | 10.3 | 11.4 | yes (3) | 82.1 | 1.5 | 17.2 | 22.3x |
| hand_saw | good seal | 32.0 | 32.0 | 0.0 | -1.1 | no | 66.9 | 0.4 | 32.4 | 1.0x |
| hand_saw | thin glasses temple | 27.5 | 27.6 | 4.5 | 2.9 | yes (5) | 71.4 | 0.3 | 27.8 | 2.8x |
| hand_saw | thick glasses temple | 22.9 | 23.0 | 9.1 | 7.0 | yes (3) | 76.0 | 0.3 | 23.2 | 8.3x |
| hand_saw | poor fit / hair | 18.2 | 18.3 | 13.8 | 11.0 | yes (3) | 80.8 | 0.3 | 18.5 | 24.8x |
| helicopter | good seal | 22.2 | 23.2 | 0.0 | 0.9 | no | 68.7 | 7.8 | 30.0 | 1.0x |
| helicopter | thin glasses temple | 20.0 | 20.8 | 2.2 | 4.5 | yes (3) | 71.8 | 6.8 | 26.8 | 2.1x |
| helicopter | thick glasses temple | 17.6 | 18.2 | 4.6 | 8.1 | yes (3) | 75.7 | 5.3 | 22.9 | 5.1x |
| helicopter | poor fit / hair | 14.7 | 15.2 | 7.5 | 11.7 | yes (3) | 80.3 | 3.6 | 18.3 | 14.7x |
| train | good seal | 20.7 | 21.4 | 0.0 | 0.8 | no | 70.5 | 7.9 | 28.6 | 1.0x |
| train | thin glasses temple | 18.2 | 18.8 | 2.5 | 4.2 | yes (3) | 74.2 | 6.6 | 24.8 | 2.4x |
| train | thick glasses temple | 15.5 | 16.1 | 5.1 | 7.6 | yes (3) | 78.0 | 5.5 | 21.1 | 5.7x |
| train | poor fit / hair | 12.7 | 13.3 | 8.0 | 11.0 | yes (3) | 82.3 | 4.1 | 16.8 | 15.3x |
| vacuum_cleaner | good seal | 30.3 | 30.5 | 0.0 | 0.1 | no | 65.8 | 3.5 | 33.8 | 1.0x |
| vacuum_cleaner | thin glasses temple | 26.6 | 26.8 | 3.7 | 3.9 | yes (3) | 70.4 | 2.6 | 29.2 | 2.9x |
| vacuum_cleaner | thick glasses temple | 22.6 | 22.7 | 7.7 | 7.7 | yes (3) | 75.2 | 1.9 | 24.4 | 8.7x |
| vacuum_cleaner | poor fit / hair | 18.1 | 18.3 | 12.2 | 11.5 | yes (3) | 80.0 | 1.5 | 19.6 | 26.5x |
| washing_machine | good seal | 25.0 | 25.7 | 0.0 | 0.4 | no | 69.5 | 6.0 | 31.0 | 1.0x |
| washing_machine | thin glasses temple | 22.3 | 22.9 | 2.7 | 4.1 | yes (3) | 73.8 | 4.5 | 26.7 | 2.7x |
| washing_machine | thick glasses temple | 19.2 | 19.7 | 5.8 | 7.9 | yes (3) | 78.2 | 3.1 | 22.3 | 7.5x |
| washing_machine | poor fit / hair | 15.7 | 16.2 | 9.2 | 11.6 | yes (3) | 82.8 | 2.0 | 17.8 | 21.4x |

Levels in dB(A). IL = passive attenuation on that noise (ANC contribution removed); PAR = total protection with ANC. The error mic sits in the cup, not at the eardrum: a fixed per-design offset has to be calibrated on a head-and-torso simulator before these become compliance numbers.

Figure: `seal_monitor.png`.
