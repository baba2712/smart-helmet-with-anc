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

/* ------------------------------------------------------------------ seal monitor */
_Static_assert(SEAL_NB == SEAL_MAX_BANDS, "seal band count: regenerate dsp_coeffs.h or change SEAL_MAX_BANDS");
_Static_assert(SOS_SEALBAND_N == 2u * SEAL_NB, "seal monitor uses two sections per band");

void seal_rt_init(seal_rt_t *s) { memset(s, 0, sizeof *s); }

void DSP_FAST seal_rt_sample(seal_rt_t *s, const float xc[2], const float dh[2])
{
    for (int e = 0; e < 2; e++) {
        const float in[2] = { xc[e], dh[e] };
        for (int c = 0; c < 2; c++) {
            for (unsigned b = 0; b < SEAL_NB; b++) {
                float v = biquad_run(SOS_SEALBAND[2u * b], &s->bq[e][c][b][0], in[c]);
                v = biquad_run(SOS_SEALBAND[2u * b + 1u], &s->bq[e][c][b][1], v);
                s->acc[e][c][b] += v * v;
            }
        }
    }
    if (++s->n >= ANC_FS_HZ) {
        for (int e = 0; e < 2; e++)
            for (int c = 0; c < 2; c++)
                for (unsigned b = 0; b < SEAL_NB; b++) {
                    s->sec_ms[e][c][b] = s->acc[e][c][b] * (1.0f / (float)ANC_FS_HZ);
                    s->acc[e][c][b] = 0.0f;
                }
        s->n = 0u;
        s->ready = true;
    }
}

void seal_init(seal_t *m, const float base[SEAL_MAX_BANDS])
{
    memset(m, 0, sizeof *m);
    memcpy(m->base, base, sizeof m->base);
}

bool seal_add_second(seal_t *m, const float ms_x[SEAL_MAX_BANDS], const float ms_d[SEAL_MAX_BANDS])
{
    float lx[SEAL_MAX_BANDS], lmax = -1e9f;
    for (unsigned b = 0; b < SEAL_NB; b++) {
        lx[b] = dose_pa2_to_db(ms_x[b]);
        m->il[b] = lx[b] - dose_pa2_to_db(ms_d[b]);
        if (lx[b] > lmax) lmax = lx[b];
    }
    float sum = 0.0f;
    unsigned na = 0;
    for (unsigned b = 0; b < SEAL_NB; b++) {
        if (lx[b] >= lmax - SEAL_GATE_DB && lx[b] >= SEAL_MIN_BAND_DB) {
            sum += m->base[b] - m->il[b];
            na++;
        }
    }
    m->valid = (na > 0u);
    if (!m->valid) return false;
    m->drop = sum / (float)na;
    m->avg = (m->n_valid == 0u) ? m->drop : m->avg + (m->drop - m->avg) * (1.0f / SEAL_AVG_S);
    m->n_valid++;
    m->run = (m->avg >= SEAL_FLAG_DB) ? m->run + 1u : 0u;
    if (m->run >= SEAL_HOLD_S) m->flag = true;
    else if (m->avg < SEAL_FLAG_DB - SEAL_HYST_DB) m->flag = false;
    return true;
}

void seal_learn_add(seal_learn_t *l, const float ms_x[SEAL_MAX_BANDS], const float ms_d[SEAL_MAX_BANDS])
{
    for (unsigned b = 0; b < SEAL_NB; b++) { l->ax[b] += (double)ms_x[b]; l->ad[b] += (double)ms_d[b]; }
    l->n++;
}

bool seal_learn_result(const seal_learn_t *l, float base[SEAL_MAX_BANDS])
{
    if (l->n == 0u) return false;
    for (unsigned b = 0; b < SEAL_NB; b++)
        base[b] = dose_pa2_to_db((float)(l->ax[b] / l->n)) - dose_pa2_to_db((float)(l->ad[b] / l->n));
    return true;
}

/* ------------------------------------------------------------------ tone + lock-in */
void tone_start(tone_t *t, float freq_hz, float amp, float settle_s, float meas_s)
{
    memset(t, 0, sizeof *t);
    const float w = 6.28318531f * freq_hz / (float)ANC_FS_HZ;
    t->wc = cosf(w);
    t->ws = sinf(w);
    t->c = 1.0f;
    t->s = 0.0f;
    t->amp = amp;
    t->n_settle = (uint32_t)(settle_s * (float)ANC_FS_HZ);
    /* whole periods only, so the lock-in has no leakage from the DC offset */
    const uint32_t per = (uint32_t)((float)ANC_FS_HZ / freq_hz + 0.5f);
    const uint32_t nm = (uint32_t)(meas_s * (float)ANC_FS_HZ);
    t->n_total = t->n_settle + (per ? (nm / per) * per : nm);
    t->active = true;
}

float DSP_FAST tone_next(tone_t *t)
{
    return t->active ? t->amp * t->s : 0.0f;
}

void DSP_FAST tone_update(tone_t *t, const float e[2])
{
    if (!t->active) return;
    if (t->n >= t->n_settle) {
        for (int k = 0; k < 2; k++) {
            t->i_acc[k] += (double)(e[k] * t->s);
            t->q_acc[k] += (double)(e[k] * t->c);
        }
    }
    /* advance the rotator; renormalise so float error never grows */
    const float c = t->c * t->wc - t->s * t->ws;
    const float s = t->s * t->wc + t->c * t->ws;
    const float g = 1.5f - 0.5f * (c * c + s * s);
    t->c = c * g;
    t->s = s * g;
    if (++t->n >= t->n_total) t->active = false;
}

float tone_amplitude(const tone_t *t, int ear)
{
    const uint32_t nm = t->n_total - t->n_settle;
    if (nm == 0u) return 0.0f;
    const double i = t->i_acc[ear] / nm, q = t->q_acc[ear] / nm;
    return (float)(2.0 * sqrt(i * i + q * q));
}
