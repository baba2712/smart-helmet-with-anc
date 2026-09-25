/* hw.h - low-level helpers: clocks, GPIO, time base, watchdog. */
#ifndef HW_H
#define HW_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx.h"
#include "board.h"

enum { GPIO_IN = 0, GPIO_OUT = 1, GPIO_AF = 2, GPIO_AN = 3 };
enum { PULL_NONE = 0, PULL_UP = 1, PULL_DOWN = 2 };

void hw_clock_init(void);            /* LDO, VOS1, HSE, PLL1 400 MHz, PLL2P 36 MHz ADC, HSI48 USB */
void hw_gpio(GPIO_TypeDef *port, uint32_t pin, uint32_t mode, uint32_t pull, uint32_t af);
static inline void hw_pin_write(GPIO_TypeDef *p, uint32_t pin, bool v) { p->BSRR = v ? (1u << pin) : (1u << (pin + 16u)); }
static inline bool hw_pin_read(GPIO_TypeDef *p, uint32_t pin) { return (p->IDR >> pin) & 1u; }

void     hw_systick_init(void);
uint32_t hw_millis(void);
void     hw_delay_ms(uint32_t ms);
void     hw_delay_us(uint32_t us);
static inline uint32_t hw_cycles(void) { return DWT->CYCCNT; }

void hw_watchdog_init(uint32_t timeout_ms);
void hw_watchdog_kick(void);
bool hw_is_rev_v(void);              /* silicon revision V (0x2003) vs Y (0x1003) */
void hw_jump_to_bootloader(void);    /* ST system DFU bootloader */

#endif
