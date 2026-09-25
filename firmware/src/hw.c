/*
 * hw.c - clock tree, GPIO, SysTick time base, IWDG.
 *
 * Clock plan (HSE 25 MHz crystal):
 *   PLL1: /5 -> 5 MHz ref, x160 -> 800 MHz VCO, /2 -> 400 MHz SYSCLK   (VOS1, every silicon rev)
 *   AHB /2 = 200 MHz, APB1..4 /2 = 100 MHz (timers 200 MHz)
 *   PLL2: /5 -> 5 MHz, x144 -> 720 MHz VCO, P /20 -> 36 MHz ADC kernel clock
 *   HSI48 + CRS (synced to USB SOF) -> USB OTG FS
 */
#include "hw.h"

static volatile uint32_t g_ms;

bool hw_is_rev_v(void) { return (DBGMCU->IDCODE >> 16) == 0x2003u; }

void hw_clock_init(void)
{
    /* supply: internal LDO only (no SMPS on H743) */
    PWR->CR3 = (PWR->CR3 & ~(PWR_CR3_SCUEN | PWR_CR3_BYPASS)) | PWR_CR3_LDOEN;
    while (!(PWR->CSR1 & PWR_CSR1_ACTVOSRDY)) {}

    /* VOS1 (0b11) */
    PWR->D3CR = (PWR->D3CR & ~PWR_D3CR_VOS) | (3u << PWR_D3CR_VOS_Pos);
    while (!(PWR->D3CR & PWR_D3CR_VOSRDY)) {}

    /* HSE crystal */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) {}

    /* PLL sources and pre-dividers */
    RCC->PLLCKSELR = RCC_PLLCKSELR_PLLSRC_HSE | (5u << RCC_PLLCKSELR_DIVM1_Pos) | (5u << RCC_PLLCKSELR_DIVM2_Pos)
                   | (0u << RCC_PLLCKSELR_DIVM3_Pos);
    RCC->PLLCFGR = (2u << RCC_PLLCFGR_PLL1RGE_Pos) | (2u << RCC_PLLCFGR_PLL2RGE_Pos)      /* 4-8 MHz inputs */
                 | RCC_PLLCFGR_DIVP1EN | RCC_PLLCFGR_DIVQ1EN | RCC_PLLCFGR_DIVR1EN | RCC_PLLCFGR_DIVP2EN;
    RCC->PLL1DIVR = ((160u - 1u) << RCC_PLL1DIVR_N1_Pos) | ((2u - 1u) << RCC_PLL1DIVR_P1_Pos)
                  | ((8u - 1u) << RCC_PLL1DIVR_Q1_Pos) | ((2u - 1u) << RCC_PLL1DIVR_R1_Pos);
    RCC->PLL2DIVR = ((144u - 1u) << RCC_PLL2DIVR_N2_Pos) | ((20u - 1u) << RCC_PLL2DIVR_P2_Pos)
                  | ((2u - 1u) << RCC_PLL2DIVR_Q2_Pos) | ((2u - 1u) << RCC_PLL2DIVR_R2_Pos);
    RCC->CR |= RCC_CR_PLL1ON | RCC_CR_PLL2ON;
    while ((RCC->CR & (RCC_CR_PLL1RDY | RCC_CR_PLL2RDY)) != (RCC_CR_PLL1RDY | RCC_CR_PLL2RDY)) {}

    /* bus prescalers: HPRE /2 (0b1000), all APB /2 (0b100) */
    RCC->D1CFGR = (0u << RCC_D1CFGR_D1CPRE_Pos) | (8u << RCC_D1CFGR_HPRE_Pos) | (4u << RCC_D1CFGR_D1PPRE_Pos);
    RCC->D2CFGR = (4u << RCC_D2CFGR_D2PPRE1_Pos) | (4u << RCC_D2CFGR_D2PPRE2_Pos);
    RCC->D3CFGR = (4u << RCC_D3CFGR_D3PPRE_Pos);

    /* flash: 2 wait states + WRHIGHFREQ=2 for 200 MHz AXI at VOS1 */
    FLASH->ACR = (2u << FLASH_ACR_LATENCY_Pos) | (2u << FLASH_ACR_WRHIGHFREQ_Pos);
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != (2u << FLASH_ACR_LATENCY_Pos)) {}

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL1;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL1) {}

    /* kernel clocks: ADC <- PLL2P, USB <- HSI48, USART3 <- PCLK1 */
    RCC->D3CCIPR = (RCC->D3CCIPR & ~RCC_D3CCIPR_ADCSEL) | (0u << RCC_D3CCIPR_ADCSEL_Pos);
    RCC->CR |= RCC_CR_HSI48ON;
    while (!(RCC->CR & RCC_CR_HSI48RDY)) {}
    RCC->D2CCIP2R = (RCC->D2CCIP2R & ~(RCC_D2CCIP2R_USBSEL | RCC_D2CCIP2R_USART28SEL))
                  | (3u << RCC_D2CCIP2R_USBSEL_Pos);

    /* CRS trims HSI48 against USB start-of-frame */
    RCC->APB1HENR |= RCC_APB1HENR_CRSEN;
    CRS->CFGR = (CRS->CFGR & ~CRS_CFGR_SYNCSRC) | (2u << CRS_CFGR_SYNCSRC_Pos);
    CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;

    /* GPIO ports, SYSCFG, AXI/D2 SRAM */
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN | RCC_AHB4ENR_GPIODEN
                  | RCC_AHB4ENR_GPIOEEN | RCC_AHB4ENR_GPIOHEN;
    RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
    (void)RCC->APB4ENR;

    SystemCoreClockUpdate();

    /* instruction cache on; data cache stays OFF: DSP state is in DTCM (never cached)
     * and DMA buffers in AXI SRAM stay coherent without maintenance */
    SCB_EnableICache();

    /* cycle counter for CPU-load measurement */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void hw_gpio(GPIO_TypeDef *port, uint32_t pin, uint32_t mode, uint32_t pull, uint32_t af)
{
    const uint32_t s2 = pin * 2u;
    port->MODER = (port->MODER & ~(3u << s2)) | (mode << s2);
    port->PUPDR = (port->PUPDR & ~(3u << s2)) | (pull << s2);
    port->OSPEEDR = (port->OSPEEDR & ~(3u << s2)) | (((mode == GPIO_AF) ? 2u : 0u) << s2);
    port->OTYPER &= ~(1u << pin);
    if (mode == GPIO_AF) {
        volatile uint32_t *afr = &port->AFR[pin >> 3];
        const uint32_t s4 = (pin & 7u) * 4u;
        *afr = (*afr & ~(0xFu << s4)) | (af << s4);
    }
}

void hw_systick_init(void)
{
    SysTick_Config(SystemCoreClock / 1000u);
    NVIC_SetPriority(SysTick_IRQn, 6);
}

void SysTick_Handler(void) { g_ms++; }
uint32_t hw_millis(void) { return g_ms; }

void hw_delay_ms(uint32_t ms)
{
    const uint32_t t0 = g_ms;
    while ((g_ms - t0) < ms) { hw_watchdog_kick(); }
}

void hw_delay_us(uint32_t us)
{
    const uint32_t c0 = DWT->CYCCNT, n = us * (SystemCoreClock / 1000000u);
    while ((DWT->CYCCNT - c0) < n) {}
}

void hw_watchdog_init(uint32_t timeout_ms)
{
    /* LSI ~32 kHz, prescaler /64 -> 2 ms per count */
    IWDG1->KR = 0xCCCCu;
    IWDG1->KR = 0x5555u;
    IWDG1->PR = 4u;
    uint32_t rl = timeout_ms / 2u;
    IWDG1->RLR = (rl > 0xFFFu) ? 0xFFFu : rl;
    while (IWDG1->SR) {}
    IWDG1->KR = 0xAAAAu;
}

void hw_watchdog_kick(void) { IWDG1->KR = 0xAAAAu; }

void hw_jump_to_bootloader(void)
{
    /* H743 system memory (ST DFU bootloader over USB on PA11/PA12) */
    const uint32_t sysmem = 0x1FF09800u;
    __disable_irq();
    SysTick->CTRL = 0;
    for (uint32_t i = 0; i < 8u; i++) { NVIC->ICER[i] = 0xFFFFFFFFu; NVIC->ICPR[i] = 0xFFFFFFFFu; }
    SCB_DisableICache();
    RCC->CFGR &= ~RCC_CFGR_SW;                 /* back to HSI */
    while ((RCC->CFGR & RCC_CFGR_SWS) != 0u) {}
    SCB->VTOR = sysmem;
    __set_MSP(*(volatile uint32_t *)sysmem);
    __enable_irq();
    ((void (*)(void))(*(volatile uint32_t *)(sysmem + 4u)))();
    for (;;) {}
}
