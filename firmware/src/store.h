/* store.h - calibration/settings and the exposure log in flash bank 2 (dual-bank RWW:
 * erasing/programming bank 2 never stalls code running from bank 1 or ITCM). */
#ifndef STORE_H
#define STORE_H

#include <stdint.h>
#include <stdbool.h>
#include "app.h"

/* one 32-byte flash word per minute */
typedef struct {
    uint32_t seq;            /* monotonically increasing record number (0xFFFFFFFF = erased) */
    uint32_t minutes_on;     /* power-on minutes since shift reset */
    float    laeq_ear;       /* dB(A), this minute, at the ear */
    float    laeq_amb;       /* dB(A), this minute, outside */
    float    dose_pct;       /* shift dose so far, at the ear */
    float    dose_amb_pct;   /* shift dose so far, unprotected */
    uint16_t vbat_mv;
    uint8_t  mode;
    uint8_t  flags;          /* bit0 overload, bit1 watchdog trip, bit2 charging, bit3 shift start */
    uint32_t crc;
} store_rec_t;

_Static_assert(sizeof(store_rec_t) == 32, "log record must be one 256-bit flash word");

bool store_load_cal(app_cal_t *c);          /* false -> defaults used */
bool store_save_cal(const app_cal_t *c);
void store_log_init(void);                  /* finds the write position, loads the last record */
bool store_log_append(store_rec_t *r);      /* fills seq + crc */
bool store_log_last(store_rec_t *r);
uint32_t store_log_count(void);
bool store_log_get(uint32_t idx_from_oldest, store_rec_t *r);
void store_log_erase_all(void);
uint32_t store_crc32(const void *p, uint32_t n);

#endif
