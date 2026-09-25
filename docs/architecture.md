# Architecture

## What the system does

A safety helmet with helmet-mount earmuffs. Each cup has two MEMS microphones and a
40 mm driver. One STM32H743 runs a **hybrid FxLMS** controller for both ears at
32 kHz. It adds anti-noise at the ear, keeps an exposure log (noise dosimeter),
and can pass speech and alarms through at a capped level.

```
 outside noise ~100 dB(A)
        |
        v
  [REF mic] ---+                          (outside the shell, per cup)
               |    [passive cup: -6 dB @63 Hz ... -35 dB @2 kHz]
               v
  +----- preamp + AA filter ----+        ADC1/ADC2 simultaneous, 16-bit, TIM6 @ 32 kHz
  |                             v
  |     x_c = x - F^ * u   (speaker leakage removed)
  |     d^  = e - S^ * u   (IMC disturbance estimate)
  |     y   = W_ff * x_c + W_fb * d^          <- hybrid FxLMS, weighted update
  |     u   = ramp * y + hear_through(x_c)    -> DAC1 -> recon filter -> TPA6132A2 -> driver
  |                             ^
  +----- preamp + AA filter ----+
               ^
  [ERR mic] ---+                          (in the cup, ~8 mm from the ear canal)
```

## Signal chain and latency budget

The controller can only cancel what it can predict. Every microsecond between the
sound reaching the reference mic and the anti-noise leaving the driver has to be
paid for by prediction. So the design avoids anything that buffers.

| Stage | Delay | Why this choice |
| --- | --- | --- |
| Mic + difference preamp | ~0 | Differential MEMS output: common-mode noise on the cable cancels |
| Anti-alias filter (1st-order pole + 2nd-order Sallen-Key, ~9 kHz) | ~30 µs | Kept low-order on purpose. Each pole costs delay |
| SAR ADC (not sigma-delta) | ~1-2 µs | An audio codec's decimation filter would add 0.5-1 ms and kill feed-forward ANC |
| DMA -> ISR -> both outputs computed | ~5 µs | Hot code in ITCM, state in DTCM (zero wait-state), no RTOS in the audio path |
| DAC (unbuffered write, no trigger) | 1 APB clock | Output updates right after the write |
| Zero-order hold + reconstruction filter | ~16 µs + ~25 µs | |
| Amp + driver + acoustic path to error mic | ~30 µs + driver phase | |

The simulation models every one of these as an exact fractional delay
(`sim/plant.py`). The latency sweep in `sim/results/summary.md` shows how much
attenuation each extra 30 µs costs.

## Control algorithm

Hybrid FxLMS, per ear. The files are `firmware/dsp/anc_core.c`, which mirrors
`sim/anc_ref.py` line for line.

* **Feed-forward** (128 taps): the outside mic as reference. Handles tonal machine
  noise (motors, fans, transformer hum) very well, because tonal noise is
  predictable, so causality doesn't limit it.
* **Adaptive feedback, IMC form** (64 taps): the reference is the disturbance
  estimated from the error mic, `d^ = e - S^*u`. It catches what the outside mic
  doesn't see.
* **Secondary-path model S^** (64 taps) and **leakage model F^** (128 taps):
  identified with a white-noise probe. This happens at the factory (quiet room)
  and is then *refined* at every power-on (2 s, ~83 dB SPL at the ear). The
  refinement doubles as a **fit check**: a lifted cup or a dead driver shows up
  as a big model change, and the helmet warns instead of misbehaving.
* **Normalised LMS with leakage**: step size is independent of noise level. Leakage
  (1e-5) keeps the filters bounded.
* **Frequency-weighted update**: a low-shelf (−20 dB below 300 Hz) on the error and
  filtered references. Without it, huge 50/100 Hz tones dominate the gradient,
  and the controller trades a fraction of a dB there for +8 to +20 dB of mid-band
  amplification (found in simulation, see the commit history).
* **30 Hz high-pass on both references**: the driver can't make infrasound. Chasing
  it drives the output into clipping, and the distortion is audible.

Tuned values live in one place, `sim/run_all.py::TUNED`. They are exported to
`firmware/inc/anc_tuning.h`.

## Safety logic (firmware)

| Guard | Trigger | Action |
| --- | --- | --- |
| Divergence watchdog | Error power > 6 dB above the disturbance estimate for 200 ms | Filters reset to zero |
| Clip watchdog | > 5 % of output samples clipped over 100 ms | Filters reset to zero |
| Ear overload | \|p\| at the error mic > 118 dB (sine peak), capped at 90 % of the channel's full scale (~14 Pa), for 50 ms | Forced passive + red LED |
| Output ceiling | Always | Anti-noise clamp at the lower of the DAC range and the amp's 0.9 V swing at the selected gain; hear-through hard-limited to 82 dB SPL |
| Mode changes | Always | 50 ms fades (no clicks) |
| No calibration | `paths_valid == 0` | ANC refuses to run; passive + magenta LED |
| Fit check failure at power-on | Model change > 6 dB or poor fit | Keeps the stored model; amber LED |
| Seal leak (in use) | Per-band cup attenuation >= 3 dB below the factory baseline for 3 s | Amber LED, `seal_leak` in status/BLE, log flag bit 4 |
| HardFault / watchdog reset | Any | DAC to mid-scale and amp muted *before* reset |
| Low battery | < 3.2 V | Saves the log record and powers off |

In every failure the helmet falls back to a **passive earmuff**. That is the safe state.

## Dosimeter

* A-weighted (IEC 61672 biquads at 32 kHz) level at the ear (energy mean of both
  error mics) and outside (reference mic), 1 s LAeq.
* Dose per NIOSH/ISO (85 dB(A), 3 dB exchange) by default. It is configurable for
  OSHA / the Indian Factories Act (90 dB(A), 5 dB exchange).
* A minute-by-minute record (at-ear and outside LAeq, running dose, battery, mode,
  flags) goes into a flash ring in bank 2. That is about 136 h of history, and it
  survives power cycles. The shift dose continues across power-offs until `dose reset`.
* The firmware test checks it against the standard: 94.0 dB at 1 kHz reads
  93.99 dB(A), and 100 dB(A) gives exactly NIOSH's 15 min allowance.

## Seal monitor (in-use protection check)

A cushion leak (safety-glasses temple, hair, a tilted helmet) costs 3-12 dB of
protection, and ANC cannot win it back: the driver's own low-frequency pressure
leaks out too. So the helmet measures its seal continuously and tells the wearer.

It needs no extra sensor or test signal. The ANC core already forms
`x_c = x - F_hat*y` (outside noise without the speaker's leakage) and
`d_hat = e - S_hat*y` (noise that came through the cup, ANC removed). Per ear,
the ISR accumulates octave-band (250 Hz-2 kHz) mean squares of both; once a
second the main loop computes the per-band attenuation `L(x_c) - L(d_hat)`.
Per-band attenuation belongs to the cup and its seal, not to the noise, so one
factory baseline (`seal learn`, reference head, broadband noise) holds for any
noise. Bands within 15 dB of the loudest outside band and above 55 dB count;
their mean shortfall vs the baseline, smoothed over ~4 s, raises the flag at
3 dB (1 dB hysteresis).

Simulation over 10 noises (3 synthetic, 7 real recordings) x 4 seal conditions:
23/23 real leaks flagged in 3-5 s, 0 false alarms (`sim/results/seal_monitor.md`).
C and Python agree to < 0.001 dB (`firmware/test/test_fil.py`). Reference:
`sim/seal_ref.py`; firmware: `seal_*` in `dsp/dsp_misc.c`.

## Boards

| Board | Where | What |
| --- | --- | --- |
| Main (60 × 50 mm, 4-layer) | Left cup | STM32H743VIT6, USB-C, Li-ion charger + power path, 3.3 V buck-boost, 2.8 V analog LDO, left mic front-ends, 2 × DAC reconstruction, TPA6132A2 stereo amp, BLE header, SWD, buttons/LEDs |
| Satellite (34 × 26 mm, 2-layer) | Right cup | Right mic front-ends (so the analog signal crosses the headband cable at line level), speaker connector |
| Mic (11 × 12.5 mm, 2-layer, × 4) | Outside shell and in front of each driver | IM73A135 differential MEMS mic (bottom port) + decoupling |

The pin map lives in one table (`hardware/gen/pinmap.py`). That table generates both
the schematic nets and `firmware/inc/board_pins.h`.
