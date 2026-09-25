/*
 * anc_core - hybrid FxLMS controller for one earcup (portable C99, float32).
 *
 * This is the firmware twin of sim/anc_ref.py::_run. Same state, same update
 * order; firmware/test/ checks the two against each other in closed loop.
 *
 * Per sample:
 *     y = anc_output(ear, x_adc, e_adc)       -- as early as possible, write the DAC
 *     anc_commit(ear, u_total, ht)            -- u_total = what actually went to the DAC
 *                                                (y + hear-through ht), then adapt
 *
 * Units: x_adc/e_adc in pascal at the mics (after mic calibration), y in
 * "pascal at the ear at 200 Hz" (the output calibration makes S_hat unity
 * there). Nothing here touches hardware.
 */
#ifndef ANC_CORE_H
#define ANC_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "anc_tuning.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ANC_LY  ((ANC_L_S > ANC_L_F) ? ANC_L_S : ANC_L_F)          /* output history */
#define ANC_LX  ((ANC_L_FF > ANC_L_S) ? ANC_L_FF : ANC_L_S)        /* reference history */
#define ANC_LD  ((ANC_L_FB > ANC_L_S) ? ANC_L_FB : ANC_L_S)        /* d_hat history */

typedef struct {
    float mu_ff, mu_fb, leak, eps, y_max;
    float wd_ratio, clip_trip;
    uint32_t wd_hold;
    bool use_ff, use_fb, adapt;
} anc_params_t;

typedef struct {
    uint32_t trips;          /* watchdog resets since init */
    float    pe, pd;         /* smoothed error / disturbance power (Pa^2) */
    float    clip_rate;      /* smoothed fraction of clipped output samples */
    float    last_y;
} anc_stats_t;

/* Doubled circular buffer: buf[pos + k] == sample from k steps ago, contiguous. */
typedef struct { uint32_t pos; } anc_ring_t;

typedef struct {
    anc_params_t p;
    anc_stats_t  st;

    float w_ff[ANC_L_FF];
    float w_fb[ANC_L_FB];
    float s_hat[ANC_L_S];
    float f_hat[ANC_L_F];

    float ybuf[2 * ANC_LY];  anc_ring_t ry;   /* u[n-1-k] */
    float xbuf[2 * ANC_LX];  anc_ring_t rx;   /* x_c filtered, newest first */
    float dbuf[2 * ANC_LD];  anc_ring_t rd;   /* d_hat filtered */
    float xfb[2 * ANC_L_FF]; anc_ring_t rxf;  /* weighted filtered reference */
    float dfb[2 * ANC_L_FB]; anc_ring_t rdf;
    float hbuf[2 * ANC_L_S]; anc_ring_t rh;   /* hear-through history (for e_adapt) */

    float p_xf, p_df;                          /* running |x'|^2, |d'|^2 */
    float hx[4], hd[4];                        /* ref high-pass DF1 states */
    float wx1, wy1, wdx1, wdy1, we_x1, we_y1;  /* weighting filter states */
    uint32_t bad;

    /* scratch carried from output() to commit() */
    float e, dh;
    float xc;              /* reference with speaker leakage removed (for hear-through) */
} anc_t;

void  anc_default_params(anc_params_t *p);
void  anc_init(anc_t *a, const anc_params_t *p, const float *s_hat, const float *f_hat);
void  anc_reset_weights(anc_t *a);
float anc_output(anc_t *a, float x_adc, float e_adc);
void  anc_commit(anc_t *a, float u_total, float ht);

#ifdef __cplusplus
}
#endif
#endif
