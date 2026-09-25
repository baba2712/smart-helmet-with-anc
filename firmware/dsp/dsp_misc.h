/*
 * Small DSP blocks around the ANC core (portable C99, float32):
 *   biquad     - DF1 biquad cascade
 *   sysid      - speaker->mic path identification (boot / factory calibration)
 *   dosimeter  - A-weighted exposure (LAeq, dose %, peak) at ear and outside
 *   hearthru   - speech-band pass-through with a hard level limiter
 *   seal       - in-use seal monitor: per-band passive attenuation vs a factory baseline
 */
#ifndef DSP_MISC_H
#define DSP_MISC_H

#include <stdint.h>
#include <stdbool.h>
#include "anc_tuning.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ biquad */
typedef struct { float x1, x2, y1, y2; } biquad_state_t;

static inline float biquad_run(const float c[5], biquad_state_t *s, float x)
{
    const float y = c[0] * x + c[1] * s->x1 + c[2] * s->x2 - c[3] * s->y1 - c[4] * s->y2;
    s->x2 = s->x1; s->x1 = x; s->y2 = s->y1; s->y1 = y;
    return y;
}

/* ------------------------------------------------------------------ sysid */
#define SYSID_LP  ((ANC_L_S > ANC_L_F) ? ANC_L_S : ANC_L_F)

typedef struct {
    float s[ANC_L_S];        /* estimate: output -> error mic */
    float f[ANC_L_F];        /* estimate: output -> reference mic */
    float pbuf[2 * SYSID_LP];
    uint32_t pos;
    uint32_t rng;
    float amp;               /* uniform probe half-range (output units) */
    float mu;
    float pe, pr;            /* smoothed |e|^2 and |residual|^2 for the fit metric */
    uint32_t n_left;
} sysid_t;

/* start from s0/f0 (NULL = zeros); probe_rms in output units; seconds of probing */
void  sysid_start(sysid_t *id, const float *s0, const float *f0, float probe_rms, float mu, float seconds);
float sysid_next_probe(sysid_t *id);                 /* call first each sample, write result to DAC */
void  sysid_update(sysid_t *id, float x_adc, float e_adc);
static inline bool sysid_done(const sysid_t *id) { return id->n_left == 0u; }
float sysid_fit_db(const sysid_t *id);               /* how much of e the model explains (dB, higher = better) */
float sysid_energy(const float *h, uint32_t n);

/* ------------------------------------------------------------------ dosimeter */
typedef struct {
    biquad_state_t aw[3][3];     /* 3 channels x 3 sections: ear L, ear R, ambient */
    float acc[3];                /* running sum of A-weighted p^2 */
    float peak[3];               /* unweighted |p| max */
    uint32_t n;
    /* double-buffered 1-second results handed to the main loop */
    volatile bool ready;
    float sec_ms[3];             /* mean square (Pa^2) over the last second */
    float sec_peak[3];
} dosi_rt_t;

void dosi_rt_init(dosi_rt_t *d);
void dosi_rt_sample(dosi_rt_t *d, float ear_l, float ear_r, float ambient);   /* ISR, every sample */

typedef struct {
    float lc_db;        /* criterion level (85 NIOSH/ISO, 90 OSHA/India Factories Act) */
    float q_db;         /* exchange rate (3 ISO/NIOSH, 5 OSHA) */
    float thr_db;       /* levels below this don't count (0 = count everything) */
    float t_crit_s;     /* criterion duration, 8 h */
} dose_cfg_t;

typedef struct {
    dose_cfg_t cfg;
    double dose_pct;         /* worn (at-ear) dose, % of allowed */
    double dose_amb_pct;     /* what the dose would have been without the helmet */
    double e_ear, e_amb;     /* energy sums for LAeq over the shift */
    uint32_t seconds;
    float laeq_1s_ear, laeq_1s_amb, lzpk_ear;
    float max_1s_ear;
} dose_t;

void  dose_init(dose_t *d, const dose_cfg_t *cfg);
void  dose_add_second(dose_t *d, float ms_ear, float ms_amb, float peak_ear);   /* main loop, 1 Hz */
float dose_laeq(double e_sum, uint32_t seconds);
float dose_pa2_to_db(float ms);

/* ------------------------------------------------------------------ hear-through */
typedef struct {
    biquad_state_t bp[2];
    float gain;          /* linear, 0 = off */
    float limit_pk;      /* Pa peak at the ear, hard ceiling */
    float env;           /* peak envelope */
} hearthru_t;

void  hearthru_init(hearthru_t *h, float gain, float limit_db_spl);
float hearthru_run(hearthru_t *h, float ref_pa);

/* ------------------------------------------------------------------ seal monitor
 * Reference: sim/seal_ref.py (filters, per-second arithmetic and decisions match).
 * ISR: octave-band mean squares of x_c (outside, speaker leakage removed) and
 * d_hat (noise that came through the cup, ANC removed), per ear, over 1 s.
 * Main loop (1 Hz): per-band attenuation vs the factory baseline -> drop, flag. */
#define SEAL_MAX_BANDS 4u

typedef struct {
    biquad_state_t bq[2][2][SEAL_MAX_BANDS][2];   /* ear, signal (x_c, d_hat), band, section */
    float acc[2][2][SEAL_MAX_BANDS];
    uint32_t n;
    volatile bool ready;
    float sec_ms[2][2][SEAL_MAX_BANDS];          /* Pa^2, last complete second */
} seal_rt_t;

void seal_rt_init(seal_rt_t *s);
void seal_rt_sample(seal_rt_t *s, const float xc[2], const float dh[2]);     /* ISR, every sample */

typedef struct {
    float base[SEAL_MAX_BANDS];    /* factory per-band attenuation (dB) */
    float il[SEAL_MAX_BANDS];      /* last second's per-band attenuation (dB) */
    float drop;                    /* last valid second: mean(base - il) over active bands */
    float avg;                     /* smoothed drop (dB) - the reported seal loss */
    uint32_t n_valid, run;
    bool valid;                    /* last second had enough noise to judge */
    bool flag;                     /* seal leak */
} seal_t;

void seal_init(seal_t *m, const float base[SEAL_MAX_BANDS]);
bool seal_add_second(seal_t *m, const float ms_x[SEAL_MAX_BANDS], const float ms_d[SEAL_MAX_BANDS]);

/* factory baseline: energy-mean attenuation per band over the learning seconds */
typedef struct { double ax[SEAL_MAX_BANDS], ad[SEAL_MAX_BANDS]; uint32_t n; } seal_learn_t;
void seal_learn_add(seal_learn_t *l, const float ms_x[SEAL_MAX_BANDS], const float ms_d[SEAL_MAX_BANDS]);
bool seal_learn_result(const seal_learn_t *l, float base[SEAL_MAX_BANDS]);    /* false if no data */

#ifdef __cplusplus
}
#endif
#endif
