/*
 * anc_core.c - hybrid FxLMS controller for one earcup. See anc_core.h.
 *
 * Mirrors sim/anc_ref.py::_run line for line (the sim is the specification).
 * Hot path: kept branch-light and with fixed-length loops so GCC unrolls them;
 * on the STM32H7 the whole anc_t lives in DTCM (zero wait state) and this file
 * is linked into ITCM (see ld/stm32h743vi.ld, section .itcm).
 */
#include "anc_core.h"
#include <string.h>

#if defined(__arm__)
#define ANC_FAST __attribute__((section(".itcm"), noinline))
#else
#define ANC_FAST
#endif

static inline void ring_push(float *buf, anc_ring_t *r, uint32_t n, float v)
{
    r->pos = (r->pos == 0u) ? (n - 1u) : (r->pos - 1u);
    buf[r->pos] = v;
    buf[r->pos + n] = v;
}

/* newest-first view: view[k] = sample from k steps ago */
#define VIEW(buf, ring) (&(buf)[(ring).pos])

void anc_default_params(anc_params_t *p)
{
    p->mu_ff = ANC_MU_FF;
    p->mu_fb = ANC_MU_FB;
    p->leak = ANC_LEAK;
    p->eps = ANC_EPS;
    p->y_max = ANC_Y_MAX_PA;
    p->wd_ratio = ANC_WD_RATIO;
    p->wd_hold = ANC_WD_HOLD;
    p->clip_trip = ANC_CLIP_TRIP;
    p->use_ff = true;
    p->use_fb = true;
    p->adapt = true;
}

void ANC_FAST anc_reset_weights(anc_t *a)
{
    for (uint32_t k = 0; k < ANC_L_FF; k++) a->w_ff[k] = 0.0f;
    for (uint32_t k = 0; k < ANC_L_FB; k++) a->w_fb[k] = 0.0f;
}

void anc_init(anc_t *a, const anc_params_t *p, const float *s_hat, const float *f_hat)
{
    memset(a, 0, sizeof *a);
    a->p = *p;
    if (s_hat) memcpy(a->s_hat, s_hat, sizeof a->s_hat);
    if (f_hat) memcpy(a->f_hat, f_hat, sizeof a->f_hat);
    a->st.pe = 1e-12f;
    a->st.pd = 1e-12f;
}

float ANC_FAST anc_output(anc_t *a, float x_adc, float e_adc)
{
    const float *yv = VIEW(a->ybuf, a->ry);    /* yv[k] = u[n-1-k] */
    const float *hv = VIEW(a->hbuf, a->rh);    /* hv[k] = ht[n-1-k] */

    /* model the speaker's contribution at both mics */
    float sh = 0.0f, fh = 0.0f, shh = 0.0f;
    for (uint32_t k = 0; k < ANC_L_S - 1u; k++) {
        sh += a->s_hat[k + 1u] * yv[k];
        shh += a->s_hat[k + 1u] * hv[k];
    }
    for (uint32_t k = 0; k < ANC_L_F - 1u; k++)
        fh += a->f_hat[k + 1u] * yv[k];

    const float xc_raw = x_adc - fh;           /* reference with speaker leakage removed */
    const float dh = e_adc - sh;               /* IMC disturbance estimate */
    a->e = e_adc - shh;                        /* error without our own hear-through */
    a->dh = dh;
    a->xc = xc_raw;

    /* 2nd-order high-pass on both references: never chase infrasound */
    float *h = a->hx;
    const float xc = ANC_HP_B0 * xc_raw + ANC_HP_B1 * h[0] + ANC_HP_B2 * h[1] - ANC_HP_A1 * h[2] - ANC_HP_A2 * h[3];
    h[1] = h[0]; h[0] = xc_raw; h[3] = h[2]; h[2] = xc;
    h = a->hd;
    const float dhf = ANC_HP_B0 * dh + ANC_HP_B1 * h[0] + ANC_HP_B2 * h[1] - ANC_HP_A1 * h[2] - ANC_HP_A2 * h[3];
    h[1] = h[0]; h[0] = dh; h[3] = h[2]; h[2] = dhf;

    ring_push(a->xbuf, &a->rx, ANC_LX, xc);
    ring_push(a->dbuf, &a->rd, ANC_LD, dhf);

    const float *xv = VIEW(a->xbuf, a->rx);
    const float *dv = VIEW(a->dbuf, a->rd);
    float y = 0.0f;
    if (a->p.use_ff)
        for (uint32_t k = 0; k < ANC_L_FF; k++) y += a->w_ff[k] * xv[k];
    if (a->p.use_fb)
        for (uint32_t k = 0; k < ANC_L_FB; k++) y += a->w_fb[k] * dv[k];

    float clipped = 0.0f;
    if (y > a->p.y_max)       { y = a->p.y_max;  clipped = 1.0f; }
    else if (y < -a->p.y_max) { y = -a->p.y_max; clipped = 1.0f; }
    a->st.clip_rate += (1.0f / (0.1f * (float)ANC_FS_HZ)) * (clipped - a->st.clip_rate);
    a->st.last_y = y;
    return y;
}

void ANC_FAST anc_commit(anc_t *a, float u_total, float ht)
{
    const float *xv = VIEW(a->xbuf, a->rx);
    const float *dv = VIEW(a->dbuf, a->rd);

    /* filtered references x' = S_hat * x_c, d' = S_hat * d_hat */
    float xf = 0.0f, df = 0.0f;
    for (uint32_t k = 0; k < ANC_L_S; k++) {
        xf += a->s_hat[k] * xv[k];
        df += a->s_hat[k] * dv[k];
    }
    /* frequency weighting (low-shelf) on filtered refs and error */
    float t = ANC_W_B0 * xf + ANC_W_B1 * a->wx1 - ANC_W_A1 * a->wy1;
    a->wx1 = xf; a->wy1 = t; xf = t;
    t = ANC_W_B0 * df + ANC_W_B1 * a->wdx1 - ANC_W_A1 * a->wdy1;
    a->wdx1 = df; a->wdy1 = t; df = t;

    const float old_xf = a->xfb[a->rxf.pos + ANC_L_FF - 1u];
    const float old_df = a->dfb[a->rdf.pos + ANC_L_FB - 1u];
    a->p_xf += xf * xf - old_xf * old_xf;
    a->p_df += df * df - old_df * old_df;
    if (a->p_xf < 0.0f) a->p_xf = 0.0f;
    if (a->p_df < 0.0f) a->p_df = 0.0f;
    ring_push(a->xfb, &a->rxf, ANC_L_FF, xf);
    ring_push(a->dfb, &a->rdf, ANC_L_FB, df);

    const float e = a->e;
    const float ew = ANC_W_B0 * e + ANC_W_B1 * a->we_x1 - ANC_W_A1 * a->we_y1;
    a->we_x1 = e; a->we_y1 = ew;

    if (a->p.adapt) {
        const float keep = 1.0f - a->p.leak;
        if (a->p.use_ff) {
            const float g = a->p.mu_ff * ew / (a->p.eps + a->p_xf);
            const float *v = VIEW(a->xfb, a->rxf);
            for (uint32_t k = 0; k < ANC_L_FF; k++) a->w_ff[k] = a->w_ff[k] * keep - g * v[k];
        }
        if (a->p.use_fb) {
            const float g = a->p.mu_fb * ew / (a->p.eps + a->p_df);
            const float *v = VIEW(a->dfb, a->rdf);
            for (uint32_t k = 0; k < ANC_L_FB; k++) a->w_fb[k] = a->w_fb[k] * keep - g * v[k];
        }
    }

    /* divergence watchdog: error well above the disturbance estimate, or
     * sustained output clipping => zero the filters (the caller also sees
     * st.trips change and can drop to passive / flag the user) */
    const float as = 1.0f / (0.02f * (float)ANC_FS_HZ);
    a->st.pe += as * (e * e - a->st.pe);
    a->st.pd += as * (a->dh * a->dh - a->st.pd);
    a->bad = (a->st.pe > a->p.wd_ratio * a->st.pd) ? a->bad + 1u : 0u;
    if (a->bad > a->p.wd_hold || a->st.clip_rate > a->p.clip_trip) {
        anc_reset_weights(a);
        a->bad = 0u;
        a->st.clip_rate = 0.0f;
        a->st.trips++;
    }

    ring_push(a->ybuf, &a->ry, ANC_LY, u_total);
    ring_push(a->hbuf, &a->rh, ANC_L_S, ht);
}
