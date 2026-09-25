/*
 * board.h - electrical constants of the ANC helmet main board (rev A).
 * Pin assignments live in board_pins.h (generated from hardware/gen/pinmap.py).
 * Values marked [CAL] are nominal and get replaced by per-unit calibration
 * stored in flash (see docs/calibration.md).
 */
#ifndef BOARD_H
#define BOARD_H

#include "board_pins.h"

#define BOARD_NAME           "ANC-Helmet rev A"
#define FW_VERSION           "1.0.0"

/* clocks */
#define HSE_HZ               25000000u
#define SYSCLK_HZ            400000000u   /* VOS1: valid on every H743 silicon revision */
#define HCLK_HZ              200000000u
#define APB1_TIMER_HZ        200000000u
#define ADC_KER_HZ           36000000u    /* PLL2P */

/* analog front end */
#define VREF_V               2.8f         /* VREF+ = VDDA = 2V8A rail (LP5907-2.8) */
#define ADC_FULL_SCALE       65536.0f     /* 16-bit */
#define ADC_MID              32768.0f     /* preamps are biased at VREF/2 */
#define MIC_SENS_V_PER_PA    0.012589f    /* IM73A135: -38 dBV/Pa differential [CAL] */
#define GAIN_REF             3.409f       /* 75k/22k difference amp (outside mics) */
#define GAIN_ERR             6.818f       /* 150k/22k difference amp (in-cup mics) */

/* output: DAC (12-bit, 0..VREF) -> 2nd-order recon filter -> TPA6132A2 */
#define DAC_MID              2048.0f
#define DAC_MAX_CODE         4095.0f
/* nominal anti-noise scale: pascal at the ear per DAC volt at amp gain 0 dB.
 * Only sets the unit of the controller output; the identified S_hat absorbs
 * the real value. [CAL] */
#define OUT_PA_PER_V         8.0f
#define AMP_GAIN_DEFAULT     1u           /* TPA6132A2 G1:G0 -> 0=-6 dB 1=0 dB 2=+3 dB 3=+6 dB */

/* battery: VBAT -> 100k/100k -> ADC3 */
#define VBAT_DIV             2.0f
#define VBAT_LOW_V           3.40f        /* warn */
#define VBAT_CUTOFF_V        3.20f        /* save state and power off */

/* safety */
#define HT_LIMIT_DB_SPL      82.0f        /* hear-through can never exceed this at the ear */
#define EAR_OVERLOAD_DB      118.0f       /* sustained unweighted peak at the error mic -> passive */

#endif
