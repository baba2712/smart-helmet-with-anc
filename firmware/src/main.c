/*
 * main.c - boot sequence, main loop, command line (USB CDC and BLE share it).
 *
 * Boot:  hold power -> clocks -> load calibration -> audio up (muted) -> amp on
 *        -> if calibrated: 2 s fit check (probe noise) -> default mode (ANC)
 *        -> if not calibrated: passive + magenta LED, waits for "cal paths"
 * Loop:  USB/BLE commands, button, LED, 1 Hz dose/battery, 1/min log record, watchdog.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "hw.h"
#include "audio.h"
#include "app.h"
#include "store.h"
#include "comms.h"
#include "power_ui.h"

/* survives a soft reset: request to enter the ST DFU bootloader */
static volatile uint32_t boot_magic __attribute__((section(".noinit")));
#define BOOT_MAGIC_DFU 0xDF00B007u

static dose_t   g_dose;
static uint32_t minutes_on, sec_in_min;
static double   min_e_ear, min_e_amb;
static uint8_t  min_flags;
static bool     stream[2];
static comms_link_t id_requester = LINK_ALL;
static int      mic_cal_ch = -1;
static float    mic_cal_db;
static comms_link_t mic_cal_link;
static bool     fit_warning;

extern uint32_t _sitcm, _itcm_start, _itcm_end;    /* linker */
extern uint32_t _saxi, _eaxi;

/* ------------------------------------------------------------------ output helpers */
static void out(comms_link_t l, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void out(comms_link_t l, const char *fmt, ...)
{
    char b[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b, sizeof b - 2, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof b - 3) n = (int)sizeof b - 3;
    b[n++] = '\r'; b[n++] = '\n';
    comms_write(l, b, (uint32_t)n);
}

static float cpu_load_pct(void)
{
    return 100.0f * (float)audio_isr_cycles * (float)ANC_FS_HZ / (float)SystemCoreClock;
}

static void print_status(comms_link_t l)
{
    out(l, "{\"mode\":\"%s\",\"cal\":%u,\"ear_dba\":%.1f,\"amb_dba\":%.1f,\"atten_db\":%.1f,\"dose_pct\":%.2f,"
           "\"dose_unprot_pct\":%.1f,\"laeq_shift\":%.1f,\"vbat\":%.2f,\"chg\":%u,\"usb\":%u,\"ble\":%u,\"cpu_pct\":%.1f,"
           "\"isr_us_max\":%.1f,\"trips\":[%lu,%lu],\"ovl\":%lu,\"adc_ovr\":%lu,\"fit_warn\":%u}",
        app_mode_name(g_app.mode), g_cal.paths_valid, (double)g_dose.laeq_1s_ear, (double)g_dose.laeq_1s_amb,
        (double)(g_dose.laeq_1s_amb - g_dose.laeq_1s_ear), g_dose.dose_pct, g_dose.dose_amb_pct,
        (double)dose_laeq(g_dose.e_ear, g_dose.seconds), (double)power_vbat(), power_charging(),
        power_usb_present(), comms_ble_connected(), (double)cpu_load_pct(),
        (double)((float)audio_isr_cycles_max * 1e6f / (float)SystemCoreClock),
        (unsigned long)g_app.trips[0], (unsigned long)g_app.trips[1], (unsigned long)g_app.overload_count,
        (unsigned long)audio_overruns, fit_warning);
}

static void update_led(void)
{
    if (power_vbat() < VBAT_LOW_V && !power_usb_present()) { led_set(LED_LOW_BATT); return; }
    if (g_app.overload) { led_set(LED_FAULT); return; }
    switch (g_app.mode) {
    case MODE_ID:      led_set(LED_ID); break;
    case MODE_ANC:     led_set(fit_warning ? LED_FIT_WARN : LED_ANC); break;
    case MODE_ANC_HT:  led_set(fit_warning ? LED_FIT_WARN : LED_ANC_HT); break;
    default:           led_set(g_cal.paths_valid ? LED_PASSIVE : LED_NEEDS_CAL); break;
    }
}

static void new_shift(void)
{
    dose_cfg_t dc = { g_cal.dose_lc, g_cal.dose_q, g_cal.dose_thr, 8.0f * 3600.0f };
    dose_init(&g_dose, &dc);
    minutes_on = 0;
    min_flags |= 8u;
}

static void save_state_record(void)
{
    store_rec_t r;
    memset(&r, 0, sizeof r);
    const uint32_t n = sec_in_min ? sec_in_min : 1u;
    r.minutes_on = minutes_on;
    r.laeq_ear = 10.0f * log10f((float)(min_e_ear / n) + 1e-12f);
    r.laeq_amb = 10.0f * log10f((float)(min_e_amb / n) + 1e-12f);
    r.dose_pct = (float)g_dose.dose_pct;
    r.dose_amb_pct = (float)g_dose.dose_amb_pct;
    r.vbat_mv = (uint16_t)(power_vbat() * 1000.0f);
    r.mode = (uint8_t)g_app.mode;
    r.flags = min_flags | (power_charging() ? 4u : 0u);
    store_log_append(&r);
    min_e_ear = min_e_amb = 0.0;
    sec_in_min = 0;
    min_flags = 0;
}

static void shutdown(void)
{
    audio_stop();
    amp_enable(false);
    save_state_record();
    power_off();
}

/* ------------------------------------------------------------------ command line */
static const char *HELP[] = {
    "status                      one JSON line of live values",
    "stream on|off               status every second on this link",
    "mode passive|anc|ht         operating mode",
    "cal paths [s]               FACTORY: speaker-path calibration, quiet room, helmet on head/dummy (default 3 s)",
    "cal check                   in-field fit check (refines the stored model)",
    "cal mic refl|errl|refr|errr [dB]   mic trim with a 1 kHz calibrator (default 94 dB)",
    "get                         all settings",
    "set <key> <value>           mu_ff mu_fb ht_gain amp_gain dose_lc dose_q dose_thr default_mode boot_check",
    "save                        write settings + calibration to flash",
    "dose [reset]                shift exposure summary / start new shift",
    "log [erase]                 dump minute log as CSV",
    "defaults                    factory settings (keeps path + mic calibration)",
    "version | reboot | dfu | off",
};

static void cmd_get(comms_link_t l)
{
    out(l, "mu_ff=%g mu_fb=%g ht_gain=%g amp_gain=%u dose_lc=%g dose_q=%g dose_thr=%g default_mode=%s boot_check=%u",
        (double)g_cal.mu_ff, (double)g_cal.mu_fb, (double)g_cal.ht_gain, g_cal.amp_gain, (double)g_cal.dose_lc,
        (double)g_cal.dose_q, (double)g_cal.dose_thr, app_mode_name((app_mode_t)g_cal.default_mode), g_cal.boot_refine);
    out(l, "mic_gain refl=%.4f errl=%.4f refr=%.4f errr=%.4f paths_valid=%u serial=%lu",
        (double)g_cal.mic_gain[0], (double)g_cal.mic_gain[1], (double)g_cal.mic_gain[2], (double)g_cal.mic_gain[3],
        g_cal.paths_valid, (unsigned long)g_cal.serial);
}

static bool parse_mode(const char *s, app_mode_t *m)
{
    if (!strcmp(s, "passive")) { *m = MODE_PASSIVE; return true; }
    if (!strcmp(s, "anc"))     { *m = MODE_ANC; return true; }
    if (!strcmp(s, "ht") || !strcmp(s, "anc+ht")) { *m = MODE_ANC_HT; return true; }
    return false;
}

static void cmd_set(comms_link_t l, const char *k, const char *v)
{
    const float f = strtof(v, NULL);
    if (!strcmp(k, "mu_ff") && f > 0 && f < 0.05f) g_cal.mu_ff = f;
    else if (!strcmp(k, "mu_fb") && f > 0 && f < 0.05f) g_cal.mu_fb = f;
    else if (!strcmp(k, "ht_gain") && f >= 0 && f <= 4.0f) g_cal.ht_gain = f;
    else if (!strcmp(k, "amp_gain") && f >= 0 && f <= 3) { g_cal.amp_gain = (uint8_t)f; amp_set_gain(g_cal.amp_gain); }
    else if (!strcmp(k, "dose_lc") && f >= 70 && f <= 100) g_cal.dose_lc = f;
    else if (!strcmp(k, "dose_q") && (f == 3.0f || f == 4.0f || f == 5.0f)) g_cal.dose_q = f;
    else if (!strcmp(k, "dose_thr") && f >= 0 && f <= 90) g_cal.dose_thr = f;
    else if (!strcmp(k, "boot_check") && (f == 0 || f == 1)) g_cal.boot_refine = (uint8_t)f;
    else if (!strcmp(k, "default_mode")) {
        app_mode_t m;
        if (!parse_mode(v, &m)) { out(l, "ERR mode"); return; }
        g_cal.default_mode = (uint8_t)m;
    } else { out(l, "ERR unknown key or value out of range"); return; }
    app_apply_params();
    g_dose.cfg.lc_db = g_cal.dose_lc; g_dose.cfg.q_db = g_cal.dose_q; g_dose.cfg.thr_db = g_cal.dose_thr;
    out(l, "OK (not saved - send 'save')");
}

static void cmd_log(comms_link_t l)
{
    const uint32_t n = store_log_count();
    out(l, "seq,minutes_on,laeq_ear_dba,laeq_amb_dba,dose_pct,dose_unprotected_pct,vbat_mv,mode,flags");
    for (uint32_t i = 0; i < n; i++) {
        store_rec_t r;
        if (!store_log_get(i, &r)) continue;
        out(l, "%lu,%lu,%.1f,%.1f,%.2f,%.1f,%u,%s,%u", (unsigned long)r.seq, (unsigned long)r.minutes_on,
            (double)r.laeq_ear, (double)r.laeq_amb, (double)r.dose_pct, (double)r.dose_amb_pct, r.vbat_mv,
            app_mode_name((app_mode_t)r.mode), r.flags);
        hw_watchdog_kick();
    }
    out(l, "END %lu records", (unsigned long)n);
}

static void on_line(comms_link_t l, char *s)
{
    char *argv[6];
    int argc = 0;
    for (char *t = strtok(s, " \t"); t && argc < 6; t = strtok(NULL, " \t")) argv[argc++] = t;
    if (!argc) return;
    const char *c = argv[0];

    if (!strcmp(c, "help") || !strcmp(c, "?")) {
        for (unsigned i = 0; i < sizeof HELP / sizeof HELP[0]; i++) out(l, "%s", HELP[i]);
    } else if (!strcmp(c, "version")) {
        out(l, "%s fw %s, %s, fs=%u Hz, taps ff/fb/s/f=%u/%u/%u/%u, silicon rev %s", BOARD_NAME, FW_VERSION,
            __DATE__, (unsigned)ANC_FS_HZ, (unsigned)ANC_L_FF, (unsigned)ANC_L_FB, (unsigned)ANC_L_S,
            (unsigned)ANC_L_F, hw_is_rev_v() ? "V" : "Y");
    } else if (!strcmp(c, "status")) {
        print_status(l);
    } else if (!strcmp(c, "stream") && argc > 1) {
        stream[l == LINK_BLE] = !strcmp(argv[1], "on");
        out(l, "OK");
    } else if (!strcmp(c, "mode") && argc > 1) {
        app_mode_t m;
        if (!parse_mode(argv[1], &m)) { out(l, "ERR mode passive|anc|ht"); return; }
        if (m != MODE_PASSIVE && !g_cal.paths_valid) { out(l, "ERR not calibrated - run 'cal paths' first"); return; }
        app_set_mode(m);
        fit_warning = false;
        out(l, "OK %s", app_mode_name(g_app.mode));
    } else if (!strcmp(c, "cal") && argc > 1 && !strcmp(argv[1], "paths")) {
        const float secs = (argc > 2) ? strtof(argv[2], NULL) : 3.0f;
        if (secs < 1.0f || secs > 20.0f) { out(l, "ERR seconds 1..20"); return; }
        id_requester = l;
        if (!app_start_id(false, secs)) { out(l, "ERR busy"); return; }
        out(l, "calibrating speaker paths for %.0f s - keep quiet, helmet on", (double)secs);
    } else if (!strcmp(c, "cal") && argc > 1 && !strcmp(argv[1], "check")) {
        id_requester = l;
        if (!app_start_id(true, 2.0f)) { out(l, "ERR not calibrated or busy"); return; }
        out(l, "fit check (2 s)...");
    } else if (!strcmp(c, "cal") && argc > 2 && !strcmp(argv[1], "mic")) {
        static const char *names[4] = { "refl", "errl", "refr", "errr" };
        mic_cal_ch = -1;
        for (int i = 0; i < 4; i++) if (!strcmp(argv[2], names[i])) mic_cal_ch = i;
        if (mic_cal_ch < 0) { out(l, "ERR mic refl|errl|refr|errr"); return; }
        mic_cal_db = (argc > 3) ? strtof(argv[3], NULL) : 94.0f;
        mic_cal_link = l;
        g_meter.ready = false;
        out(l, "measuring %s for 1 s at %.1f dB...", argv[2], (double)mic_cal_db);
    } else if (!strcmp(c, "get")) {
        cmd_get(l);
    } else if (!strcmp(c, "set") && argc > 2) {
        cmd_set(l, argv[1], argv[2]);
    } else if (!strcmp(c, "save")) {
        out(l, store_save_cal(&g_cal) ? "OK saved" : "ERR flash write failed");
    } else if (!strcmp(c, "dose")) {
        if (argc > 1 && !strcmp(argv[1], "reset")) { new_shift(); out(l, "OK new shift"); return; }
        out(l, "shift: %lu s, LAeq ear %.1f dB(A), LAeq outside %.1f dB(A), dose %.2f %% (would be %.1f %% unprotected), "
               "criterion %.0f dB / %.0f dB exchange, max 1 s at ear %.1f dB(A)",
            (unsigned long)g_dose.seconds, (double)dose_laeq(g_dose.e_ear, g_dose.seconds),
            (double)dose_laeq(g_dose.e_amb, g_dose.seconds), g_dose.dose_pct, g_dose.dose_amb_pct,
            (double)g_dose.cfg.lc_db, (double)g_dose.cfg.q_db, (double)g_dose.max_1s_ear);
    } else if (!strcmp(c, "log")) {
        if (argc > 1 && !strcmp(argv[1], "erase")) { store_log_erase_all(); out(l, "OK log erased"); return; }
        cmd_log(l);
    } else if (!strcmp(c, "defaults")) {
        app_cal_t keep = g_cal;
        app_defaults(&g_cal);
        memcpy(g_cal.s_hat, keep.s_hat, sizeof g_cal.s_hat);
        memcpy(g_cal.f_hat, keep.f_hat, sizeof g_cal.f_hat);
        memcpy(g_cal.mic_gain, keep.mic_gain, sizeof g_cal.mic_gain);
        g_cal.paths_valid = keep.paths_valid;
        g_cal.serial = keep.serial;
        app_apply_params();
        out(l, "OK defaults (not saved)");
    } else if (!strcmp(c, "reboot")) {
        out(l, "rebooting"); hw_delay_ms(50); NVIC_SystemReset();
    } else if (!strcmp(c, "dfu")) {
        out(l, "entering ST DFU bootloader"); hw_delay_ms(50);
        boot_magic = BOOT_MAGIC_DFU; NVIC_SystemReset();
    } else if (!strcmp(c, "off")) {
        out(l, "powering off"); hw_delay_ms(50); shutdown();
    } else {
        out(l, "ERR unknown command - try 'help'");
    }
}

/* ------------------------------------------------------------------ periodic work */
static void every_second(void)
{
    const float ms_ear = 0.5f * (g_dosi.sec_ms[0] + g_dosi.sec_ms[1]);
    const float pk = fmaxf(g_dosi.sec_peak[0], g_dosi.sec_peak[1]);
    dose_add_second(&g_dose, ms_ear, g_dosi.sec_ms[2], pk);
    min_e_ear += pow(10.0, (double)g_dose.laeq_1s_ear / 10.0);
    min_e_amb += pow(10.0, (double)g_dose.laeq_1s_amb / 10.0);
    if (g_app.overload) min_flags |= 1u;
    if (++sec_in_min >= 60u) { minutes_on++; save_state_record(); }

    power_poll();
    if (power_vbat() < VBAT_CUTOFF_V && !power_usb_present()) shutdown();
    update_led();
    for (int k = 0; k < 2; k++) if (stream[k]) print_status((comms_link_t)k);
}

static void finish_id_if_done(void)
{
    if (g_app.mode != MODE_ID || !sysid_done(&g_id[0]) || !sysid_done(&g_id[1])) return;
    char msg[200];
    const bool factory = !g_app.id_refine;
    const int rc = app_finish_id(msg, sizeof msg);
    out(id_requester, "%s", msg);
    if (rc == 0 && factory) out(id_requester, store_save_cal(&g_cal) ? "calibration saved" : "ERR flash write failed");
    fit_warning = (rc == 1);
    if (rc == 1) min_flags |= 2u;
    if (rc == 2) g_app.mode = MODE_PASSIVE;
    id_requester = LINK_ALL;
    update_led();
}

static void mic_cal_poll(void)
{
    if (mic_cal_ch < 0 || !g_meter.ready) return;
    g_meter.ready = false;
    const float want = 20e-6f * powf(10.0f, mic_cal_db / 20.0f);
    const float have = sqrtf(g_meter.ms[mic_cal_ch]);
    const float trim = want / (have + 1e-12f);
    if (trim < 0.5f || trim > 2.0f) {
        out(mic_cal_link, "ERR measured %.1f dB - trim %.2f out of range (calibrator on the right mic?)",
            (double)(20.0f * log10f(have / 20e-6f)), (double)trim);
    } else {
        g_cal.mic_gain[mic_cal_ch] *= trim;
        app_init();                         /* rebuild scales */
        out(mic_cal_link, "OK trim %.4f (was reading %.2f dB) - 'save' to keep", (double)trim,
            (double)(20.0f * log10f(have / 20e-6f)));
    }
    mic_cal_ch = -1;
}

/* ------------------------------------------------------------------ main */
static void copy_ram_sections(void)
{
    uint32_t *src = &_sitcm, *dst = &_itcm_start;
    while (dst < &_itcm_end) *dst++ = *src++;
    for (uint32_t *p = &_saxi; p < &_eaxi; p++) *p = 0;
    __DSB(); __ISB();
}

int main(void)
{
    if (boot_magic == BOOT_MAGIC_DFU) { boot_magic = 0; hw_jump_to_bootloader(); }
    power_hold();                           /* before anything else: keep the regulator on */
    copy_ram_sections();
    hw_clock_init();
    hw_systick_init();
    hw_watchdog_init(2000);

    led_init();
    led_set(LED_BOOT);
    button_init();
    amp_init();
    power_init();

    app_defaults(&g_cal);
    const bool have_cal = store_load_cal(&g_cal);
    if (!have_cal) app_defaults(&g_cal);
    store_log_init();
    new_shift();
    store_rec_t last;
    if (store_log_last(&last) && !(last.flags & 8u)) {          /* continue the running shift */
        g_dose.dose_pct = last.dose_pct;
        g_dose.dose_amb_pct = last.dose_amb_pct;
        minutes_on = last.minutes_on;
        min_flags &= ~8u;
    }

    app_init();
    audio_init();
    comms_init();
    audio_start();
    hw_delay_ms(100);                        /* let the DAC settle at mid-scale before the amp wakes */
    amp_set_gain(g_cal.amp_gain);
    amp_enable(true);
    hw_delay_ms(50);

    if (g_cal.paths_valid) {
        g_app.mode = (app_mode_t)g_cal.default_mode;
        if (g_cal.boot_refine) app_start_id(true, 2.0f);
    } else {
        g_app.mode = MODE_PASSIVE;
    }
    update_led();

    uint32_t t_sec = hw_millis(), t_10 = hw_millis();
    for (;;) {
        hw_watchdog_kick();
        comms_poll(on_line);
        finish_id_if_done();
        mic_cal_poll();

        if (g_dosi.ready) { g_dosi.ready = false; every_second(); t_sec = hw_millis(); }
        else if (hw_millis() - t_sec > 3000u) { update_led(); t_sec = hw_millis(); }   /* audio stalled? */

        if (hw_millis() - t_10 >= 10u) {
            t_10 = hw_millis();
            led_poll();
            switch (button_poll()) {
            case BTN_SHORT:
                if (!g_cal.paths_valid) break;
                if (g_app.mode == MODE_ANC) app_set_mode(MODE_ANC_HT);
                else if (g_app.mode == MODE_ANC_HT) app_set_mode(MODE_PASSIVE);
                else if (g_app.mode == MODE_PASSIVE) app_set_mode(MODE_ANC);
                fit_warning = false;
                update_led();
                break;
            case BTN_DOUBLE:                     /* re-check the fit, e.g. after adjusting the helmet */
                if (g_cal.paths_valid) app_start_id(true, 2.0f);
                update_led();
                break;
            case BTN_LONG:
                shutdown();
                break;
            default: break;
            }
        }
        __WFI();                                 /* woken by the 32 kHz audio ISR at the latest */
    }
}

/* any fault: mute the speakers first, then reset */
void HardFault_Handler(void)
{
    DAC1->DHR12RD = (2048u << DAC_DHR12RD_DACC2DHR_Pos) | 2048u;
    GPIOD->BSRR = 1u << (PIN_AMP_EN_PIN + 16u);
    NVIC_SystemReset();
}
void MemManage_Handler(void) { HardFault_Handler(); }
void BusFault_Handler(void) { HardFault_Handler(); }
void UsageFault_Handler(void) { HardFault_Handler(); }
