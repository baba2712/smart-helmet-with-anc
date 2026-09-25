/* dsp_misc.c - see dsp_misc.h */
#include "dsp_misc.h"
#include "dsp_coeffs.h"
#include <math.h>
#include <string.h>

#if defined(__arm__)
#define DSP_FAST __attribute__((section(".itcm"), noinline))
#else
#define DSP_FAST
#endif

#define P_REF 20e-6f

/* ================================================================ sysid */
void sysid_start(sysid_t *id, const float *s0, const float *f0, float probe_rms, float mu, float seconds)
{
    memset(id, 0, sizeof *id);
    if (s0) memcpy(id->s, s0, sizeof id->s);
    if (f0) memcpy(id->f, f0, sizeof id->f);
    id->rng = 0x2545F491u;
    id->amp = probe_rms * 1.7320508f;      /* uniform [-a, a] has rms a/sqrt(3) */
    id->mu = mu;
    id->n_left = (uint32_t)(seconds * (float)ANC_FS_HZ);
    id->pe = id->pr = 1e-12f;
}

float DSP_FAST sysid_next_probe(sysid_t *id)
{
    /* xorshift32 -> uniform [-1, 1) */
    uint32_t r = id->rng;
    r ^= r << 13; r ^= r >> 17; r ^= r << 5;
    id->rng = r;
    const float v = id->amp * ((float)(int32_t)r * (1.0f / 2147483648.0f));
    id->pos = (id->pos == 0u) ? (SYSID_LP - 1u) : (id->pos - 1u);
    id->pbuf[id->pos] = v;
    id->pbuf[id->pos + SYSID_LP] = v;
    return v;
}

void DSP_FAST sysid_update(sysid_t *id, float x_adc, float e_adc)
{
    if (id->n_left == 0u) return;
    id->n_left--;
    const float *p = &id->pbuf[id->pos];     /* p[0] = probe just generated (not yet heard) */
    float es = e_adc, ef = x_adc, ps = 1e-9f, pf = 1e-9f;
    for (uint32_t k = 0; k < ANC_L_S; k++) { es -= id->s[k] * p[k]; ps += p[k] * p[k]; }
    for (uint32_t k = 0; k < ANC_L_F; k++) { ef -= id->f[k] * p[k]; pf += p[k] * p[k]; }
    const float g1 = id->mu * es / ps, g2 = id->mu * ef / pf;
    for (uint32_t k = 0; k < ANC_L_S; k++) id->s[k] += g1 * p[k];
    for (uint32_t k = 0; k < ANC_L_F; k++) id->f[k] += g2 * p[k];
    const float a = 1.0f / (0.25f * (float)ANC_FS_HZ);
    id->pe += a * (e_adc * e_adc - id->pe);
    id->pr += a * (es * es - id->pr);
}

float sysid_fit_db(const sysid_t *id) { return 10.0f * log10f(id->pe / id->pr); }

float sysid_energy(const float *h, uint32_t n)
{
    float s = 0.0f;
    for (uint32_t k = 0; k < n; k++) s += h[k] * h[k];
    return s;
}

/* ================================================================ dosimeter */
void dosi_rt_init(dosi_rt_t *d) { memset(d, 0, sizeof *d); }

void DSP_FAST dosi_rt_sample(dosi_rt_t *d, float ear_l, float ear_r, float ambient)
{
    const float in[3] = { ear_l, ear_r, ambient };
    for (int c = 0; c < 3; c++) {
        float v = in[c];
        const float a = fabsf(v);
        if (a > d->peak[c]) d->peak[c] = a;
        for (unsigned s = 0; s < SOS_AWEIGHT_N; s++) v = biquad_run(SOS_AWEIGHT[s], &d->aw[c][s], v);
        d->acc[c] += v * v;
    }
    if (++d->n >= ANC_FS_HZ) {
        for (int c = 0; c < 3; c++) {
            d->sec_ms[c] = d->acc[c] * (1.0f / (float)ANC_FS_HZ);
            d->sec_peak[c] = d->peak[c];
            d->acc[c] = 0.0f;
            d->peak[c] = 0.0f;
        }
        d->n = 0u;
        d->ready = true;
    }
}

float dose_pa2_to_db(float ms) { return 10.0f * log10f(ms / (P_REF * P_REF) + 1e-12f); }

float dose_laeq(double e_sum, uint32_t seconds)
{
    if (seconds == 0u) return 0.0f;
    return 10.0f * (float)log10(e_sum / (double)seconds + 1e-30);
}

void dose_init(dose_t *d, const dose_cfg_t *cfg)
{
    memset(d, 0, sizeof *d);
    d->cfg = *cfg;
}

static double dose_increment(const dose_cfg_t *c, float l_db)
{
    if (c->thr_db > 0.0f && l_db < c->thr_db) return 0.0;
    /* allowed time at level L: T = Tc * 2^((Lc - L)/q) ; 1 s uses 1/T of the allowance */
    const double t_allowed = (double)c->t_crit_s * pow(2.0, ((double)c->lc_db - (double)l_db) / (double)c->q_db);
    return 100.0 / t_allowed;
}

void dose_add_second(dose_t *d, float ms_ear, float ms_amb, float peak_ear)
{
    const float le = dose_pa2_to_db(ms_ear), la = dose_pa2_to_db(ms_amb);
    d->laeq_1s_ear = le;
    d->laeq_1s_amb = la;
    d->lzpk_ear = 20.0f * log10f(peak_ear / P_REF + 1e-12f);
    if (le > d->max_1s_ear) d->max_1s_ear = le;
    d->dose_pct += dose_increment(&d->cfg, le);
    d->dose_amb_pct += dose_increment(&d->cfg, la);
    d->e_ear += pow(10.0, (double)le / 10.0);
    d->e_amb += pow(10.0, (double)la / 10.0);
    d->seconds++;
}

/* ================================================================ hear-through */
void hearthru_init(hearthru_t *h, float gain, float limit_db_spl)
{
    memset(h, 0, sizeof *h);
    h->gain = gain;
    h->limit_pk = 1.4142f * P_REF * powf(10.0f, limit_db_spl / 20.0f);
}

float DSP_FAST hearthru_run(hearthru_t *h, float ref_pa)
{
    if (h->gain <= 0.0f) return 0.0f;
    float v = ref_pa;
    for (unsigned s = 0; s < SOS_HEARTHRU_N; s++) v = biquad_run(SOS_HEARTHRU[s], &h->bp[s], v);
    v *= h->gain;
    /* peak envelope: 1 ms attack, 150 ms release */
    const float a = fabsf(v);
    const float k = (a > h->env) ? (1.0f / (0.001f * (float)ANC_FS_HZ)) : (1.0f / (0.15f * (float)ANC_FS_HZ));
    h->env += k * (a - h->env);
    if (h->env > h->limit_pk) v *= h->limit_pk / h->env;         /* smooth gain reduction */
    if (v > h->limit_pk) v = h->limit_pk;                        /* and a hard ceiling for transients */
    else if (v < -h->limit_pk) v = -h->limit_pk;
    return v;
}
