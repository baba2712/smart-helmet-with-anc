/* power_ui.h - soft power latch, battery/charger monitoring, headphone amp, button, RGB LED. */
#ifndef POWER_UI_H
#define POWER_UI_H

#include <stdint.h>
#include <stdbool.h>

/* ---- power ---- */
void  power_hold(void);              /* call first thing: keeps the regulator on after the button is released */
void  power_off(void);               /* mutes, releases the latch; returns only if USB keeps us alive */
void  power_init(void);              /* ADC3 for VBAT, charger status, VBUS sense */
float power_vbat(void);              /* volts, filtered */
bool  power_charging(void);
bool  power_usb_present(void);
void  power_poll(void);              /* ~10 Hz */

/* ---- amp ---- */
void amp_init(void);
void amp_enable(bool on);
void amp_set_gain(uint8_t g);        /* 0..3 = -6, 0, +3, +6 dB */

/* ---- button ---- */
typedef enum { BTN_NONE = 0, BTN_SHORT, BTN_DOUBLE, BTN_LONG } btn_event_t;
void        button_init(void);
btn_event_t button_poll(void);       /* call every 1..10 ms */
bool        button_held(void);

/* ---- LED ---- */
typedef enum {
    LED_OFF, LED_BOOT, LED_PASSIVE, LED_ANC, LED_ANC_HT, LED_ID, LED_NEEDS_CAL,
    LED_FIT_WARN, LED_FAULT, LED_LOW_BATT, LED_CHARGING, LED_CHARGED
} led_pattern_t;
void led_init(void);
void led_set(led_pattern_t p);
void led_poll(void);                 /* animates blinking patterns, call every ~10 ms */

#endif
