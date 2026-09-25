/*
 * Firmware-in-the-loop harness: runs the real firmware DSP (anc_core.c,
 * dsp_misc.c) inside the same discrete plant as sim/anc_ref.py::_run.
 * Built as a shared library and driven from test/test_fil.py.
 */
#include <stdlib.h>
#include <string.h>
#include "anc_core.h"
#include "dsp_misc.h"

/* returns number of watchdog trips; e_out/y_out length n */
int run_fil(int n, const double *d, const double *xr, const double *h_S, const double *h_F, int lp,
            const float *s_hat, const float *f_hat, const double *noise_e, const double *noise_x,
            double *e_out, double *y_out, int use_ff, int use_fb)
{
    static anc_t a;
    anc_params_t p;
    anc_default_params(&p);
    p.use_ff = use_ff; p.use_fb = use_fb;
    anc_init(&a, &p, s_hat, f_hat);
    double *yb = calloc((size_t)lp, sizeof(double));      /* yb[k] = y[n-1-k] */
    for (int i = 0; i < n; i++) {
        double sy = 0, fy = 0;
        for (int k = 0; k < lp - 1; k++) { sy += h_S[k + 1] * yb[k]; fy += h_F[k + 1] * yb[k]; }
        const float e = (float)(d[i] + sy + noise_e[i]);
        const float x = (float)(xr[i] + fy + noise_x[i]);
        const float y = anc_output(&a, x, e);
        anc_commit(&a, y, 0.0f);
        memmove(yb + 1, yb, (size_t)(lp - 1) * sizeof(double));
        yb[0] = y;
        e_out[i] = e; y_out[i] = y;
    }
    free(yb);
    return (int)a.st.trips;
}

/* path identification in the same plant; returns fit in dB */
float run_sysid(int n, const double *d, const double *xr, const double *h_S, const double *h_F, int lp,
                float probe_rms, float mu, float *s_out, float *f_out)
{
    static sysid_t id;
    sysid_start(&id, NULL, NULL, probe_rms, mu, (float)n / (float)ANC_FS_HZ);
    double *pb = calloc((size_t)lp, sizeof(double));      /* pb[k] = probe[n-1-k] */
    for (int i = 0; i < n; i++) {
        double e = d[i], x = xr[i];
        for (int k = 0; k < lp - 1; k++) { e += h_S[k + 1] * pb[k]; x += h_F[k + 1] * pb[k]; }
        const float p = sysid_next_probe(&id);
        sysid_update(&id, (float)x, (float)e);
        memmove(pb + 1, pb, (size_t)(lp - 1) * sizeof(double));
        pb[0] = p;
    }
    free(pb);
    memcpy(s_out, id.s, sizeof id.s);
    memcpy(f_out, id.f, sizeof id.f);
    return sysid_fit_db(&id);
}

/* dosimeter: feed a pressure signal to all three channels for `seconds`, return LAeq + dose */
void run_dosi(int n, const float *p, float lc, float q, double *laeq, double *dose_pct)
{
    static dosi_rt_t rt;
    static dose_t ds;
    dose_cfg_t cfg = { lc, q, 0.0f, 8.0f * 3600.0f };
    dosi_rt_init(&rt);
    dose_init(&ds, &cfg);
    for (int i = 0; i < n; i++) {
        dosi_rt_sample(&rt, p[i], p[i], p[i]);
        if (rt.ready) {
            rt.ready = false;
            dose_add_second(&ds, rt.sec_ms[0], rt.sec_ms[2], rt.sec_peak[0]);
        }
    }
    *laeq = dose_laeq(ds.e_ear, ds.seconds);
    *dose_pct = ds.dose_pct;
}

/* seal monitor on one ear's x_c / d_hat (the right ear gets the same signals).
 * Per second: il_out[s*4+b], drop_out[s], avg_out[s], flag_out[s], valid_out[s]. */
int run_seal(int n, const float *xc, const float *dh, const float *base,
             float *il_out, float *drop_out, float *avg_out, int *flag_out, int *valid_out)
{
    static seal_rt_t rt;
    seal_t m;
    seal_rt_init(&rt);
    seal_init(&m, base);
    int sec = 0;
    for (int i = 0; i < n; i++) {
        const float x2[2] = { xc[i], xc[i] }, d2[2] = { dh[i], dh[i] };
        seal_rt_sample(&rt, x2, d2);
        if (rt.ready) {
            rt.ready = false;
            valid_out[sec] = seal_add_second(&m, rt.sec_ms[0][0], rt.sec_ms[0][1]);
            for (unsigned b = 0; b < SEAL_MAX_BANDS; b++) il_out[sec * (int)SEAL_MAX_BANDS + (int)b] = m.il[b];
            drop_out[sec] = m.drop;
            avg_out[sec] = m.avg;
            flag_out[sec] = m.flag;
            sec++;
        }
    }
    return sec;
}

/* factory baseline learning over every complete second of xc / dh */
int run_seal_learn(int n, const float *xc, const float *dh, float *base_out)
{
    static seal_rt_t rt;
    seal_learn_t l;
    memset(&l, 0, sizeof l);
    seal_rt_init(&rt);
    for (int i = 0; i < n; i++) {
        const float x2[2] = { xc[i], xc[i] }, d2[2] = { dh[i], dh[i] };
        seal_rt_sample(&rt, x2, d2);
        if (rt.ready) { rt.ready = false; seal_learn_add(&l, rt.sec_ms[0][0], rt.sec_ms[0][1]); }
    }
    return seal_learn_result(&l, base_out) ? (int)l.n : 0;
}

/* tone + lock-in through the plant's secondary path (both "ears" see the same path);
 * returns the measured amplitude at the error mic (Pa peak) */
float run_tone(const double *h_S, int lp, float freq, float amp, const double *noise, int max_n)
{
    static tone_t t;
    tone_start(&t, freq, amp, 0.3f, 2.0f);
    double *yb = calloc((size_t)lp, sizeof(double));
    for (int i = 0; i < max_n && !tone_done(&t); i++) {
        double sy = 0;
        for (int k = 0; k < lp - 1; k++) sy += h_S[k + 1] * yb[k];
        const float e[2] = { (float)(sy + noise[i]), (float)(sy + noise[i]) };
        const float y = tone_next(&t);
        tone_update(&t, e);
        memmove(yb + 1, yb, (size_t)(lp - 1) * sizeof(double));
        yb[0] = y;
    }
    free(yb);
    return tone_amplitude(&t, 0);
}
