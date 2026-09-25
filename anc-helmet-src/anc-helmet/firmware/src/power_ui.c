/*
 * power_ui.c
 *
 * Soft power: the button pulls the buck-boost EN high through a diode; firmware
 * then drives PWR_HOLD (second diode) to keep it on. power_off() releases it and
 * the rail collapses when the button is let go.
 * Button sense: NPN inverter -> PC13 active low (works from 3.0 V battery to 5 V USB).
 * LED: common-anode RGB on TIM1 CH1..3, outputs configured active-low.
 */
#include "power_ui.h"
#include "hw.h"
#include <math.h>

/* ================================================================ power */
static float vbat_f = 3.7f;

void power_hold(void)
{
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;
    (void)RCC->AHB4ENR;
    hw_gpio(PIN_PWR_HOLD_PORT, PIN_PWR_HOLD_PIN, GPIO_OUT, PULL_NONE, 0);
    hw_pin_write(PIN_PWR_HOLD_PORT, PIN_PWR_HOLD_PIN, true);
}

static uint16_t adc3_read(uint32_t ch)
{
    ADC3->SQR1 = (0u << ADC_SQR1_L_Pos) | (ch << ADC_SQR1_SQ1_Pos);
    ADC3->ISR = ADC_ISR_EOC;
    ADC3->CR |= ADC_CR_ADSTART;
    uint32_t t = 100000u;
    while (!(ADC3->ISR & ADC_ISR_EOC) && --t) {}
    return (uint16_t)ADC3->DR;
}

void power_init(void)
{
    hw_gpio(PIN_VBAT_SNS_PORT, PIN_VBAT_SNS_PIN, GPIO_AN, PULL_NONE, 0);
    hw_gpio(PIN_CHG_STAT_PORT, PIN_CHG_STAT_PIN, GPIO_IN, PULL_UP, 0);
    hw_gpio(PIN_VBUS_SNS_PORT, PIN_VBUS_SNS_PIN, GPIO_IN, PULL_NONE, 0);

    RCC->AHB4ENR |= RCC_AHB4ENR_ADC3EN;
    (void)RCC->AHB4ENR;
    ADC3_COMMON->CCR = 0;                        /* async kernel clock (PLL2P) */
    ADC3->CR &= ~ADC_CR_DEEPPWD;
    ADC3->CR |= ADC_CR_ADVREGEN;
    hw_delay_us(50);
    ADC3->CR = (ADC3->CR & ~ADC_CR_BOOST) | (hw_is_rev_v() ? ADC_CR_BOOST_1 : ADC_CR_BOOST_0);
    ADC3->CR |= ADC_CR_ADCALLIN;
    ADC3->CR |= ADC_CR_ADCAL;
    while (ADC3->CR & ADC_CR_ADCAL) {}
    const uint32_t ch = 10u;                     /* PC0 = ADC3_INP10 */
    ADC3->PCSEL |= 1u << ch;
    ADC3->SMPR2 = (ADC3->SMPR2 & ~(7u << (3u * (ch - 10u)))) | (6u << (3u * (ch - 10u)));   /* 387.5 cycles */
    ADC3->CFGR = ADC_CFGR_OVRMOD;                /* 16-bit, software trigger */
    ADC3->ISR = ADC_ISR_ADRDY;
    ADC3->CR |= ADC_CR_ADEN;
    while (!(ADC3->ISR & ADC_ISR_ADRDY)) {}
    vbat_f = (float)adc3_read(ch) * (VREF_V / ADC_FULL_SCALE) * VBAT_DIV;
}

void power_poll(void)
{
    const float v = (float)adc3_read(10u) * (VREF_V / ADC_FULL_SCALE) * VBAT_DIV;
    vbat_f += 0.1f * (v - vbat_f);
}

float power_vbat(void) { return vbat_f; }
bool  power_charging(void) { return !hw_pin_read(PIN_CHG_STAT_PORT, PIN_CHG_STAT_PIN); }
bool  power_usb_present(void) { return hw_pin_read(PIN_VBUS_SNS_PORT, PIN_VBUS_SNS_PIN); }

void power_off(void)
{
    amp_enable(false);
    led_set(LED_OFF);
    hw_pin_write(PIN_PWR_HOLD_PORT, PIN_PWR_HOLD_PIN, false);
    /* the rail drops once the button is released; keep the watchdog quiet meanwhile */
    for (;;) { hw_watchdog_kick(); }
}

/* ================================================================ amp */
void amp_init(void)
{
    hw_gpio(PIN_AMP_EN_PORT, PIN_AMP_EN_PIN, GPIO_OUT, PULL_NONE, 0);
    hw_gpio(PIN_AMP_G0_PORT, PIN_AMP_G0_PIN, GPIO_OUT, PULL_NONE, 0);
    hw_gpio(PIN_AMP_G1_PORT, PIN_AMP_G1_PIN, GPIO_OUT, PULL_NONE, 0);
    amp_enable(false);
}
void amp_enable(bool on) { hw_pin_write(PIN_AMP_EN_PORT, PIN_AMP_EN_PIN, on); }
void amp_set_gain(uint8_t g)
{
    hw_pin_write(PIN_AMP_G0_PORT, PIN_AMP_G0_PIN, g & 1u);
    hw_pin_write(PIN_AMP_G1_PORT, PIN_AMP_G1_PIN, (g >> 1) & 1u);
}

/* ================================================================ button */
static uint32_t btn_t_down, btn_t_up, btn_debounce;
static bool btn_state, btn_long_sent, btn_pending_short;

void button_init(void) { hw_gpio(PIN_BTN_PORT, PIN_BTN_PIN, GPIO_IN, PULL_UP, 0); }
bool button_held(void) { return btn_state; }

btn_event_t button_poll(void)
{
    const uint32_t now = hw_millis();
    const bool raw = !hw_pin_read(PIN_BTN_PORT, PIN_BTN_PIN);     /* active low */
    if (raw != btn_state) {
        if (btn_debounce == 0u) btn_debounce = now;
        if (now - btn_debounce >= 20u) {
            btn_state = raw;
            btn_debounce = 0u;
            if (raw) { btn_t_down = now; btn_long_sent = false; }
            else {
                btn_t_up = now;
                if (!btn_long_sent) {
                    if (btn_pending_short && (btn_t_up - btn_t_down) < 600u) { btn_pending_short = false; return BTN_DOUBLE; }
                    btn_pending_short = true;
                }
            }
        }
    } else btn_debounce = 0u;
    if (btn_state && !btn_long_sent && (now - btn_t_down) >= 2000u) { btn_long_sent = true; btn_pending_short = false; return BTN_LONG; }
    if (!btn_state && btn_pending_short && (now - btn_t_up) > 350u) { btn_pending_short = false; return BTN_SHORT; }
    return BTN_NONE;
}

/* ================================================================ LED */
static led_pattern_t led_pat = LED_OFF;

static void led_rgb(uint16_t r, uint16_t g, uint16_t b)
{
    TIM1->CCR1 = r; TIM1->CCR2 = g; TIM1->CCR3 = b;
}

void led_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    (void)RCC->APB2ENR;
    hw_gpio(PIN_LED_R_PORT, PIN_LED_R_PIN, GPIO_AF, PULL_NONE, 1);
    hw_gpio(PIN_LED_G_PORT, PIN_LED_G_PIN, GPIO_AF, PULL_NONE, 1);
    hw_gpio(PIN_LED_B_PORT, PIN_LED_B_PIN, GPIO_AF, PULL_NONE, 1);
    TIM1->PSC = 199u;                           /* 200 MHz -> 1 MHz */
    TIM1->ARR = 999u;                           /* 1 kHz PWM, duty 0..1000 */
    TIM1->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = (6u << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
    /* common-anode LED: active low outputs */
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC3E | TIM_CCER_CC3P;
    TIM1->BDTR = TIM_BDTR_MOE;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
    led_rgb(0, 0, 0);
}

void led_set(led_pattern_t p) { led_pat = p; }

void led_poll(void)
{
    const uint32_t t = hw_millis();
    const bool blink = (t / 250u) & 1u, slow = (t / 1000u) & 1u;
    const float breathe = 0.5f + 0.5f * sinf((float)(t % 3000u) * (6.2831853f / 3000.0f));
    const uint16_t br = (uint16_t)(60.0f + 240.0f * breathe);
    switch (led_pat) {
    case LED_OFF:        led_rgb(0, 0, 0); break;
    case LED_BOOT:       led_rgb(300, 300, 300); break;
    case LED_PASSIVE:    led_rgb(slow ? 120 : 0, slow ? 120 : 0, slow ? 120 : 0); break;
    case LED_ANC:        led_rgb(0, br, 0); break;
    case LED_ANC_HT:     led_rgb(0, br / 2u, br); break;
    case LED_ID:         led_rgb(blink ? 400 : 0, 0, blink ? 400 : 0); break;
    case LED_NEEDS_CAL:  led_rgb(slow ? 400 : 0, 0, slow ? 400 : 0); break;
    case LED_FIT_WARN:   led_rgb(blink ? 500 : 0, blink ? 250 : 0, 0); break;
    case LED_FAULT:      led_rgb(((t / 100u) & 1u) ? 800 : 0, 0, 0); break;
    case LED_LOW_BATT:   led_rgb(slow ? 500 : 0, 0, 0); break;
    case LED_CHARGING:   led_rgb(br, br / 3u, 0); break;
    case LED_CHARGED:    led_rgb(0, 300, 0); break;
    }
}
