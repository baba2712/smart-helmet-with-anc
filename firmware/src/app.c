/*
 * app.c - real-time audio frame (32 kHz ISR) + mode/calibration logic.
 *
 * Signal flow per frame, per ear:
 *     codes -> pascal (mic calibration)
 *     ANC:  y  = anc_output(x_ref, e_err)              (hybrid FxLMS)
 *     HT:   ht = hearthru_run(x_ref minus speaker leakage)
 *     out:  u  = ramp * y + ht   -> DAC   (written for both ears before any adaptation)
 *     then: anc_commit(u, ht), dosimeter, overload guard
 */
#include "app.h"
#include "audio.h"
#include "board.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

app_cal_t   g_cal;
app_state_t g_app;
anc_t       g_anc[2];
sysid_t     g_id[2];
dosi_rt_t   g_dosi;
hearthru_t  g_ht[2];
seal_rt_t   g_seal_rt;
meter_t     g_meter;

static float k_in[4];          /* ADC code -> Pa: ref L, err L, ref R, err R */
static float dac_per_unit;     /* DAC codes per controller output unit */
static float ovl_pk;           /* Pa, overload threshold at the error mic */
static uint32_t ovl_run[2];

static const char *const mode_names[MODE_COUNT] = { "passive", "anc", "anc+ht", "id", "test" };
static tone_t g_tone;
static float  test_vpk;                 /* amp output volts per tone unit of amplitude */
static app_mode_t mode_after_test;
static const float amp_gain_lin[4] = { 0.5012f, 1.0f, 1.4125f, 1.9953f };   /* -6, 0, +3, +6 dB */
const char *app_mode_name(app_mode_t m) { return (m < MODE_COUNT) ? mode_names[m] : "?"; }

void app_defaults(app_cal_t *c)
{
    memset(c, 0, sizeof *c);
    for (int i = 0; i < 4; i++) c->mic_gain[i] = 1.0f;
    c->mu_ff = ANC_MU_FF;
    c->mu_fb = ANC_MU_FB;
    c->ht_gain = 1.0f;
    c->dose_lc = 85.0f;            /* NIOSH REL / ISO 1999; set 90 & q=5 for OSHA / Factories Act */
    c->dose_q = 3.0f;
    c->dose_thr = 0.0f;
    c->default_mode = MODE_ANC;
    c->boot_refine = 1;
    c->amp_gain = AMP_GAIN_DEFAULT;
}

static void compute_scales(void)
{
    const float v_per_code = VREF_V / ADC_FULL_SCALE;
    k_in[0] = v_per_code / (GAIN_REF * MIC_SENS_V_PER_PA) * g_cal.mic_gain[0];
    k_in[1] = v_per_code / (GAIN_ERR * MIC_SENS_V_PER_PA) * g_cal.mic_gain[1];
    k_in[2] = v_per_code / (GAIN_REF * MIC_SENS_V_PER_PA) * g_cal.mic_gain[2];
    k_in[3] = v_per_code / (GAIN_ERR * MIC_SENS_V_PER_PA) * g_cal.mic_gain[3];
    dac_per_unit = (4096.0f / VREF_V) / OUT_PA_PER_V;
    ovl_pk = 1.4142f * 20e-6f * powf(10.0f, EAR_OVERLOAD_DB / 20.0f);
    /* the in-cup channel saturates at ~16 Pa (SPICE): a threshold above full scale
     * could never trip, so cap it at 90 % of the smaller error channel's full scale */
    const float err_fs = (ADC_FULL_SCALE / 2.0f) * fminf(k_in[1], k_in[3]);
    if (ovl_pk > 0.9f * err_fs) ovl_pk = 0.9f * err_fs;
}

void app_apply_params(void)
{
    anc_params_t p;
    anc_default_params(&p);
    p.mu_ff = g_cal.mu_ff;
    p.mu_fb = g_cal.mu_fb;
    /* never ask for more than the DAC can give ... */
    const float y_dac_max = 0.95f * (DAC_MID - 1.0f) / dac_per_unit;
    if (p.y_max > y_dac_max) p.y_max = y_dac_max;
    /* ... or than the amp can swing at the selected gain: otherwise the amp clips
     * first and the controller's clip detector never sees it */
    const float v_dac_per_unit = dac_per_unit * VREF_V / 4096.0f;
    const float y_amp_max = 0.95f * AMP_OUT_VPK / (amp_gain_lin[g_cal.amp_gain & 3u] * v_dac_per_unit);
    if (p.y_max > y_amp_max) p.y_max = y_amp_max;
    for (int e = 0; e < 2; e++) {
        g_anc[e].p.mu_ff = p.mu_ff;
        g_anc[e].p.mu_fb = p.mu_fb;
        g_anc[e].p.y_max = p.y_max;
        g_ht[e].gain = g_cal.ht_gain;
    }
}

static void init_cores(void)
{
    anc_params_t p;
    anc_default_params(&p);
    for (int e = 0; e < 2; e++) {
        anc_init(&g_anc[e], &p, g_cal.s_hat[e], g_cal.f_hat[e]);
        hearthru_init(&g_ht[e], g_cal.ht_gain, HT_LIMIT_DB_SPL);
    }
    app_apply_params();
}

void app_init(void)
{
    seal_rt_init(&g_seal_rt);
    compute_scales();
    init_cores();
    dosi_rt_init(&g_dosi);
    g_app.mode = MODE_PASSIVE;
    g_app.out_gain = 0.0f;
    g_app.out_target = 0.0f;
}

void app_set_mode(app_mode_t m)
{
    if (m == MODE_ID || m == MODE_TEST || m >= MODE_COUNT) return;
    if ((m == MODE_ANC || m == MODE_ANC_HT) && !g_cal.paths_valid) m = MODE_PASSIVE;   /* never run blind */
    if (g_app.mode == MODE_ID || g_app.mode == MODE_TEST) return;                       /* finish ID / test first */
    g_app.overload = false;
    g_app.mode = m;
}

bool app_id_running(void) { return g_app.mode == MODE_ID; }

bool app_start_test(float freq_hz, float amp_vpk, float seconds)
{
    if (g_app.mode == MODE_ID || g_app.mode == MODE_TEST) return false;
    if (freq_hz < 20.0f || freq_hz > 4000.0f || amp_vpk <= 0.0f || amp_vpk > AMP_OUT_VPK) return false;
    const float v_per_unit = dac_per_unit * VREF_V / 4096.0f * amp_gain_lin[g_cal.amp_gain & 3u];
    test_vpk = v_per_unit;
    mode_after_test = g_app.mode;
    g_app.out_gain = 0.0f;
    tone_start(&g_tone, freq_hz, amp_vpk / v_per_unit, 0.3f, seconds);
    __DSB();
    g_app.mode = MODE_TEST;
    return true;
}

bool app_test_done(void)
{
    if (g_app.mode != MODE_TEST || !tone_done(&g_tone)) return false;
    __disable_irq();
    init_cores();                       /* restart the controllers cleanly after the tone */
    g_app.mode = mode_after_test;
    __enable_irq();
    return true;
}

float app_test_result(int ear)
{
    const float v = g_tone.amp * test_vpk;             /* amp output, V peak */
    return (v > 0.0f) ? tone_amplitude(&g_tone, ear) / v : 0.0f;
}

bool app_start_id(bool refine, float seconds)
{
    if (g_app.mode == MODE_ID) return false;
    if (refine && !g_cal.paths_valid) return false;
    g_app.mode_after_id = (g_app.mode == MODE_ID) ? MODE_PASSIVE : g_app.mode;
    g_app.out_gain = 0.0f;
    for (int e = 0; e < 2; e++) {
        sysid_start(&g_id[e], refine ? g_cal.s_hat[e] : NULL, refine ? g_cal.f_hat[e] : NULL,
                    ANC_ID_PROBE_PA, refine ? ANC_ID_MU_REFINE : ANC_ID_MU_FACTORY, seconds);
    }
    g_id[EAR_R].rng ^= 0x9E3779B9u;          /* independent probes per ear */
    g_app.id_refine = refine;
    __DSB();
    g_app.mode = MODE_ID;
    return true;
}

int app_finish_id(char *msg, int len)
{
    int rc = 0;
    for (int e = 0; e < 2; e++) g_app.id_fit_db[e] = sysid_fit_db(&g_id[e]);
    const float fit = fminf(g_app.id_fit_db[0], g_app.id_fit_db[1]);
    if (g_app.id_refine) {
        float worst = 0.0f;
        for (int e = 0; e < 2; e++) {
            const float r = 10.0f * log10f(sysid_energy(g_id[e].s, ANC_L_S) / (sysid_energy(g_cal.s_hat[e], ANC_L_S) + 1e-20f));
            if (fabsf(r) > fabsf(worst)) worst = r;
        }
        if (fit < 6.0f || fabsf(worst) > 6.0f) {
            snprintf(msg, (size_t)len, "fit check failed (fit %.1f dB, change %+.1f dB): helmet not worn, cup lifted or driver fault - kept stored model", (double)fit, (double)worst);
            rc = 1;
        } else {
            for (int e = 0; e < 2; e++) {
                memcpy(g_cal.s_hat[e], g_id[e].s, sizeof g_cal.s_hat[e]);
                memcpy(g_cal.f_hat[e], g_id[e].f, sizeof g_cal.f_hat[e]);
            }
            snprintf(msg, (size_t)len, "fit ok: L %.1f dB, R %.1f dB, change %+.1f dB", (double)g_app.id_fit_db[0], (double)g_app.id_fit_db[1], (double)worst);
        }
    } else {
        if (fit < 10.0f) {
            snprintf(msg, (size_t)len, "path calibration FAILED (fit L %.1f / R %.1f dB, need >= 10): check driver + mic wiring, quiet room", (double)g_app.id_fit_db[0], (double)g_app.id_fit_db[1]);
            rc = 2;
        } else {
            for (int e = 0; e < 2; e++) {
                memcpy(g_cal.s_hat[e], g_id[e].s, sizeof g_cal.s_hat[e]);
                memcpy(g_cal.f_hat[e], g_id[e].f, sizeof g_cal.f_hat[e]);
            }
            g_cal.paths_valid = 1;
            snprintf(msg, (size_t)len, "path calibration ok: fit L %.1f dB, R %.1f dB", (double)g_app.id_fit_db[0], (double)g_app.id_fit_db[1]);
        }
    }
    /* restart the controllers on whatever model is now installed */
    __disable_irq();
    init_cores();
    g_app.mode = (g_cal.paths_valid) ? g_app.mode_after_id : MODE_PASSIVE;
    if (g_app.mode == MODE_ID) g_app.mode = MODE_PASSIVE;
    __enable_irq();
    return rc;
}

static inline uint32_t to_dac(float u)
{
    float c = DAC_MID + u * dac_per_unit;
    if (c < 0.0f) c = 0.0f;
    if (c > DAC_MAX_CODE) c = DAC_MAX_CODE;
    return (uint32_t)c;
}

__attribute__((section(".itcm"))) void app_audio_frame(const audio_in_t *in)
{
    const float x[2] = { ((float)in->ref_l - ADC_MID) * k_in[0], ((float)in->ref_r - ADC_MID) * k_in[2] };
    const float e[2] = { ((float)in->err_l - ADC_MID) * k_in[1], ((float)in->err_r - ADC_MID) * k_in[3] };
    const app_mode_t mode = g_app.mode;

    if (mode == MODE_TEST) {
        const float u = tone_next(&g_tone);
        audio_dac_write(to_dac(u), to_dac(u));
        tone_update(&g_tone, e);
        if (tone_done(&g_tone)) audio_dac_write((uint32_t)DAC_MID, (uint32_t)DAC_MID);
    } else if (mode == MODE_ID) {
        float p[2];
        for (int k = 0; k < 2; k++) p[k] = sysid_done(&g_id[k]) ? 0.0f : sysid_next_probe(&g_id[k]);
        audio_dac_write(to_dac(p[0]), to_dac(p[1]));
        for (int k = 0; k < 2; k++) sysid_update(&g_id[k], x[k], e[k]);
    } else {
        const bool want = (mode == MODE_ANC || mode == MODE_ANC_HT) && !g_app.overload;
        const float target = want ? 1.0f : 0.0f;
        float g = g_app.out_gain;
        const float step = 1.0f / (0.05f * (float)ANC_FS_HZ);      /* 50 ms fades */
        g = (g < target) ? fminf(g + step, target) : fmaxf(g - step, target);
        g_app.out_gain = g;

        /* seal monitor inputs: outside noise without the speaker's leakage, and the noise
         * that came through the cup with the anti-noise removed (plain mics when idle) */
        float sx[2] = { x[0], x[1] }, sd[2] = { e[0], e[1] };
        if (g > 0.0f) {
            float y[2], ht[2], u[2];
            for (int k = 0; k < 2; k++) {
                y[k] = anc_output(&g_anc[k], x[k], e[k]);
                ht[k] = (mode == MODE_ANC_HT) ? hearthru_run(&g_ht[k], g_anc[k].xc) : 0.0f;
                u[k] = g * y[k] + ht[k];
            }
            audio_dac_write(to_dac(u[0]), to_dac(u[1]));
            for (int k = 0; k < 2; k++) {
                anc_commit(&g_anc[k], u[k], ht[k]);
                g_app.trips[k] = g_anc[k].st.trips;
                sx[k] = g_anc[k].xc;
                sd[k] = g_anc[k].dh;
            }
        } else {
            audio_dac_write((uint32_t)DAC_MID, (uint32_t)DAC_MID);
        }
        seal_rt_sample(&g_seal_rt, sx, sd);

        /* safety: extreme sustained level at the ear (e.g. driver fault, feedback howl) -> passive */
        for (int k = 0; k < 2; k++) {
            ovl_run[k] = (fabsf(e[k]) > ovl_pk) ? ovl_run[k] + 1u : (ovl_run[k] > 0u ? ovl_run[k] - 1u : 0u);
            if (ovl_run[k] > ANC_FS_HZ / 20u && g_app.out_gain > 0.0f) {
                g_app.overload = true;
                g_app.overload_count++;
                anc_reset_weights(&g_anc[k]);
                ovl_run[k] = 0u;
            }
        }
    }
    /* dose at the ear = energy mean of both ears; ambient from the left outside mic */
    dosi_rt_sample(&g_dosi, e[0], e[1], x[0]);

    g_meter.acc[0] += x[0] * x[0];
    g_meter.acc[1] += e[0] * e[0];
    g_meter.acc[2] += x[1] * x[1];
    g_meter.acc[3] += e[1] * e[1];
    if (++g_meter.n >= ANC_FS_HZ) {
        for (int k = 0; k < 4; k++) { g_meter.ms[k] = g_meter.acc[k] * (1.0f / (float)ANC_FS_HZ); g_meter.acc[k] = 0.0f; }
        g_meter.n = 0u;
        g_meter.ready = true;
    }
}
