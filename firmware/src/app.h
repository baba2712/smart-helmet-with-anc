/* app.h - operating modes, calibration data and the real-time audio frame. */
#ifndef APP_H
#define APP_H

#include <stdint.h>
#include <stdbool.h>
#include "anc_core.h"
#include "dsp_misc.h"

typedef enum {
    MODE_PASSIVE = 0,    /* outputs muted, dosimeter still running */
    MODE_ANC,            /* hybrid ANC */
    MODE_ANC_HT,         /* ANC + speech/alarm hear-through */
    MODE_ID,             /* speaker-path identification running (probe noise) */
    MODE_COUNT
} app_mode_t;

enum { EAR_L = 0, EAR_R = 1 };

/* persistent calibration + settings (stored in flash, see store.c) */
typedef struct {
    float s_hat[2][ANC_L_S];
    float f_hat[2][ANC_L_F];
    float mic_gain[4];             /* per-mic trim vs nominal: ref L, err L, ref R, err R */
    float mu_ff, mu_fb;
    float ht_gain;                 /* linear, hear-through */
    float dose_lc, dose_q, dose_thr;
    uint8_t paths_valid;           /* factory path calibration done */
    uint8_t default_mode;          /* mode after power-on */
    uint8_t boot_refine;           /* 2 s path check at power-on */
    uint8_t amp_gain;              /* TPA6132A2 G1:G0 */
    uint32_t serial;
    /* --- v3: seal monitor (new fields go at the end: store.c migrates older blobs) --- */
    float seal_base[2][SEAL_MAX_BANDS];   /* factory per-band passive attenuation, dB */
    uint8_t seal_valid;                   /* 'seal learn' done */
    uint8_t pad_[3];
} app_cal_t;

typedef struct {
    volatile app_mode_t mode;
    volatile app_mode_t mode_after_id;
    volatile bool id_refine;        /* ID is a refinement of stored paths (vs factory from zero) */
    volatile uint32_t trips[2];
    volatile float out_gain;        /* ramped 0..1 to avoid clicks on mode changes */
    volatile float out_target;
    volatile bool overload;         /* error mic saw sustained extreme level -> forced passive */
    volatile uint32_t overload_count;
    float id_fit_db[2];
} app_state_t;

extern app_cal_t   g_cal;
extern app_state_t g_app;
extern anc_t       g_anc[2];
extern sysid_t     g_id[2];
extern dosi_rt_t   g_dosi;
extern hearthru_t  g_ht[2];
extern seal_rt_t   g_seal_rt;

/* raw (unweighted) 1-second mean square of each mic, for calibration and status */
typedef struct {
    float acc[4];
    uint32_t n;
    volatile bool ready;
    float ms[4];              /* Pa^2: ref L, err L, ref R, err R */
} meter_t;
extern meter_t g_meter;

void app_defaults(app_cal_t *c);
void app_init(void);
void app_set_mode(app_mode_t m);
bool app_start_id(bool refine, float seconds);   /* returns false if not allowed */
bool app_id_running(void);
int  app_finish_id(char *msg, int len);          /* call when ID done: validates, installs; 0 = ok */
void app_apply_params(void);
const char *app_mode_name(app_mode_t m);

#endif
