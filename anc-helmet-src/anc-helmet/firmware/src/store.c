/*
 * store.c - flash bank 2 layout (STM32H743, 128 KB sectors, 256-bit flash words):
 *   sector 5 (0x081A0000)  calibration + settings (app_cal_t + header + CRC)
 *   sector 6 (0x081C0000)  exposure log ring, half A
 *   sector 7 (0x081E0000)  exposure log ring, half B   -> 2 x 4096 minutes (~136 h) of history
 * The application image is linked into bank 1 only (see ld/stm32h743vi.ld).
 */
#include "store.h"
#include "hw.h"
#include <string.h>

#define CAL_SECTOR     5u
#define LOG_SECTOR_A   6u
#define LOG_SECTOR_B   7u
#define SECTOR_SIZE    (128u * 1024u)
#define SECT_ADDR(s)   (FLASH_BANK2_BASE + (s) * SECTOR_SIZE)
#define REC_PER_SECT   (SECTOR_SIZE / sizeof(store_rec_t))
#define CAL_MAGIC      0x414E4331u    /* "ANC1" */
#define CAL_VERSION    2u

typedef struct {
    uint32_t magic, version, size, reserved;
    app_cal_t cal;
} cal_blob_t;

static uint32_t log_next_seq, log_count, log_oldest_seq;
static uint32_t log_wr_sector, log_wr_index;

uint32_t store_crc32(const void *p, uint32_t n)
{
    const uint8_t *b = p;
    uint32_t c = 0xFFFFFFFFu;
    while (n--) {
        c ^= *b++;
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

/* ---------------------------------------------------------------- low-level bank 2 */
static void fl_unlock(void)
{
    if (FLASH->CR2 & FLASH_CR_LOCK) { FLASH->KEYR2 = 0x45670123u; FLASH->KEYR2 = 0xCDEF89ABu; }
}
static void fl_lock(void) { FLASH->CR2 |= FLASH_CR_LOCK; }
static bool fl_wait(void)
{
    while (FLASH->SR2 & (FLASH_SR_BSY | FLASH_SR_QW | FLASH_SR_WBNE)) hw_watchdog_kick();
    const uint32_t err = FLASH->SR2 & (FLASH_SR_WRPERR | FLASH_SR_PGSERR | FLASH_SR_STRBERR | FLASH_SR_INCERR
                                     | FLASH_SR_OPERR | FLASH_SR_RDPERR | FLASH_SR_RDSERR | FLASH_SR_SNECCERR
                                     | FLASH_SR_DBECCERR);
    FLASH->CCR2 = 0x0FFF0000u;           /* clear all flags */
    return err == 0u;
}

static bool fl_erase(uint32_t sector)
{
    fl_unlock();
    fl_wait();
    FLASH->CR2 = (FLASH->CR2 & ~(FLASH_CR_PSIZE | FLASH_CR_SNB)) | (3u << FLASH_CR_PSIZE_Pos)
               | (sector << FLASH_CR_SNB_Pos) | FLASH_CR_SER;
    FLASH->CR2 |= FLASH_CR_START;
    const bool ok = fl_wait();
    FLASH->CR2 &= ~FLASH_CR_SER;
    fl_lock();
    return ok;
}

/* program n bytes (multiple of 32) at a 32-byte aligned address */
static bool fl_program(uint32_t addr, const void *src, uint32_t n)
{
    const uint32_t *s = src;
    bool ok = true;
    fl_unlock();
    fl_wait();
    FLASH->CR2 = (FLASH->CR2 & ~FLASH_CR_PSIZE) | (3u << FLASH_CR_PSIZE_Pos) | FLASH_CR_PG;
    for (uint32_t off = 0; off < n && ok; off += 32u) {
        volatile uint32_t *d = (volatile uint32_t *)(addr + off);
        for (int w = 0; w < 8; w++) d[w] = s[(off >> 2) + (uint32_t)w];
        __DSB();
        ok = fl_wait();
    }
    FLASH->CR2 &= ~FLASH_CR_PG;
    fl_lock();
    return ok;
}

/* ---------------------------------------------------------------- calibration */
bool store_load_cal(app_cal_t *c)
{
    const cal_blob_t *b = (const cal_blob_t *)SECT_ADDR(CAL_SECTOR);
    const uint32_t *crc = (const uint32_t *)((const uint8_t *)b + sizeof(cal_blob_t));
    if (b->magic != CAL_MAGIC || b->version != CAL_VERSION || b->size != sizeof(app_cal_t)) return false;
    if (*crc != store_crc32(b, sizeof(cal_blob_t))) return false;
    memcpy(c, &b->cal, sizeof *c);
    return true;
}

bool store_save_cal(const app_cal_t *c)
{
    static uint8_t buf[(sizeof(cal_blob_t) + 4u + 31u) & ~31u] __attribute__((aligned(4)));
    memset(buf, 0xFF, sizeof buf);
    cal_blob_t *b = (cal_blob_t *)buf;
    b->magic = CAL_MAGIC;
    b->version = CAL_VERSION;
    b->size = sizeof(app_cal_t);
    b->reserved = 0;
    memcpy(&b->cal, c, sizeof *c);
    const uint32_t crc = store_crc32(b, sizeof(cal_blob_t));
    memcpy(buf + sizeof(cal_blob_t), &crc, 4);
    if (!fl_erase(CAL_SECTOR)) return false;
    if (!fl_program(SECT_ADDR(CAL_SECTOR), buf, sizeof buf)) return false;
    app_cal_t chk;
    return store_load_cal(&chk) && memcmp(&chk, c, sizeof chk) == 0;
}

/* ---------------------------------------------------------------- exposure log */
static const store_rec_t *rec_at(uint32_t sector, uint32_t idx)
{
    return (const store_rec_t *)(SECT_ADDR(sector) + idx * sizeof(store_rec_t));
}
static bool rec_valid(const store_rec_t *r)
{
    return r->seq != 0xFFFFFFFFu && r->crc == store_crc32(r, sizeof *r - 4u);
}

/* number of written records at the start of a sector and the highest seq among them */
static uint32_t scan_sector(uint32_t sector, uint32_t *max_seq, uint32_t *min_seq)
{
    uint32_t n = 0;
    *max_seq = 0; *min_seq = 0xFFFFFFFFu;
    for (; n < REC_PER_SECT; n++) {
        const store_rec_t *r = rec_at(sector, n);
        if (r->seq == 0xFFFFFFFFu) break;
        if (rec_valid(r)) {
            if (r->seq > *max_seq) *max_seq = r->seq;
            if (r->seq < *min_seq) *min_seq = r->seq;
        }
    }
    return n;
}

void store_log_init(void)
{
    uint32_t maxa, mina, maxb, minb;
    const uint32_t na = scan_sector(LOG_SECTOR_A, &maxa, &mina);
    const uint32_t nb = scan_sector(LOG_SECTOR_B, &maxb, &minb);
    if (na == 0u && nb == 0u) {
        log_wr_sector = LOG_SECTOR_A; log_wr_index = 0; log_next_seq = 1; log_count = 0; log_oldest_seq = 1;
        return;
    }
    const bool a_newer = (na > 0u) && (nb == 0u || maxa > maxb);
    log_wr_sector = a_newer ? LOG_SECTOR_A : LOG_SECTOR_B;
    log_wr_index = a_newer ? na : nb;
    log_next_seq = (a_newer ? maxa : maxb) + 1u;
    log_count = na + nb;
    log_oldest_seq = (na && nb) ? ((mina < minb) ? mina : minb) : (na ? mina : minb);
}

bool store_log_append(store_rec_t *r)
{
    if (log_wr_index >= REC_PER_SECT) {
        const uint32_t other = (log_wr_sector == LOG_SECTOR_A) ? LOG_SECTOR_B : LOG_SECTOR_A;
        uint32_t mx, mn;
        const uint32_t dropped = scan_sector(other, &mx, &mn);
        if (!fl_erase(other)) return false;
        log_count -= dropped;
        log_oldest_seq = log_next_seq - log_count;
        log_wr_sector = other;
        log_wr_index = 0;
    }
    r->seq = log_next_seq;
    r->crc = store_crc32(r, sizeof *r - 4u);
    const uint32_t addr = (uint32_t)rec_at(log_wr_sector, log_wr_index);
    if (!fl_program(addr, r, sizeof *r)) return false;
    log_wr_index++;
    log_next_seq++;
    log_count++;
    return true;
}

bool store_log_last(store_rec_t *r)
{
    if (log_count == 0u) return false;
    uint32_t sector = log_wr_sector, idx = log_wr_index;
    if (idx == 0u) { sector = (sector == LOG_SECTOR_A) ? LOG_SECTOR_B : LOG_SECTOR_A; idx = REC_PER_SECT; }
    const store_rec_t *p = rec_at(sector, idx - 1u);
    if (!rec_valid(p)) return false;
    *r = *p;
    return true;
}

uint32_t store_log_count(void) { return log_count; }

bool store_log_get(uint32_t i, store_rec_t *r)
{
    if (i >= log_count) return false;
    const uint32_t want = log_oldest_seq + i;
    for (uint32_t s = LOG_SECTOR_A; s <= LOG_SECTOR_B; s++) {
        const store_rec_t *first = rec_at(s, 0);
        if (!rec_valid(first) || want < first->seq) continue;
        const uint32_t idx = want - first->seq;
        if (idx < REC_PER_SECT) {
            const store_rec_t *p = rec_at(s, idx);
            if (rec_valid(p) && p->seq == want) { *r = *p; return true; }
        }
    }
    return false;
}

void store_log_erase_all(void)
{
    fl_erase(LOG_SECTOR_A);
    fl_erase(LOG_SECTOR_B);
    store_log_init();
}
