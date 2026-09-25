/*
 * audio.c - the latency-critical signal path.
 *
 *   TIM6 update @ 32 kHz --TRGO--> ADC1 (master) + ADC2 (slave), regular simultaneous,
 *   2 conversions each:   ADC1: REF_L (INP3), ERR_L (INP4)
 *                         ADC2: REF_R (INP9), ERR_R (INP8)
 *   ADC12 common data register (DAMDF=10: slave<<16 | master) -> DMA1 Stream0 (2 words, circular)
 *   DMA transfer-complete ISR (priority 0) -> app_audio_frame() -> DAC1 DHR12RD (no trigger:
 *   the output updates one APB clock after the write).
 *
 * Latency budget, trigger -> DAC pin: ~2 us conversions + ~0.3 us DMA/ISR entry
 * + output computation (~3 us) = ~6 us, then the analog filters. See docs/firmware.md.
 */
#include "audio.h"
#include "hw.h"

#define FS_HZ           32000u
#define ADC_CH_REF_L    3u
#define ADC_CH_ERR_L    4u
#define ADC_CH_REF_R    9u
#define ADC_CH_ERR_R    8u
#define ADC_SMP_8C5     2u          /* 8.5 ADC clock cycles sampling */
#define EXTSEL_TIM6     13u         /* adc_ext_trg13 = tim6_trgo */
#define DMAMUX_ADC1     9u

/* DMA can't reach DTCM: this lives in AXI SRAM (D-cache is off, so no maintenance needed) */
static volatile uint32_t dma_buf[2] __attribute__((section(".axisram"), aligned(32)));

volatile uint32_t audio_isr_cycles, audio_isr_cycles_max, audio_frames, audio_overruns;

static void adc_power_up(ADC_TypeDef *adc)
{
    adc->CR &= ~ADC_CR_DEEPPWD;
    adc->CR |= ADC_CR_ADVREGEN;
    if (hw_is_rev_v()) { while (!(adc->ISR & ADC_ISR_LDORDY)) {} }
    else hw_delay_us(20);
    /* BOOST for the kernel clock: rev V divides it by 2 internally (18 MHz -> 12.5..25 range),
     * rev Y runs it straight (36 MHz -> single BOOST bit) */
    adc->CR = (adc->CR & ~ADC_CR_BOOST) | (hw_is_rev_v() ? ADC_CR_BOOST_1 : ADC_CR_BOOST_0);
    /* single-ended offset + linearity calibration */
    adc->CR &= ~ADC_CR_ADCALDIF;
    adc->CR |= ADC_CR_ADCALLIN;
    adc->CR |= ADC_CR_ADCAL;
    while (adc->CR & ADC_CR_ADCAL) {}
}

static void adc_channels(ADC_TypeDef *adc, uint32_t ch1, uint32_t ch2, uint32_t cfgr)
{
    adc->PCSEL |= (1u << ch1) | (1u << ch2);
    adc->SMPR1 = (adc->SMPR1 & ~((7u << (3u * ch1)) | (7u << (3u * ch2))))
               | (ADC_SMP_8C5 << (3u * ch1)) | (ADC_SMP_8C5 << (3u * ch2));
    adc->SQR1 = (1u << ADC_SQR1_L_Pos) | (ch1 << ADC_SQR1_SQ1_Pos) | (ch2 << ADC_SQR1_SQ2_Pos);
    adc->CFGR = cfgr;
    adc->CFGR2 = 0;                                   /* no oversampling: every us counts */
}

static void adc_enable(ADC_TypeDef *adc)
{
    adc->ISR = ADC_ISR_ADRDY;
    adc->CR |= ADC_CR_ADEN;
    while (!(adc->ISR & ADC_ISR_ADRDY)) {}
}

void audio_init(void)
{
    /* analog pins */
    hw_gpio(PIN_REF_L_PORT, PIN_REF_L_PIN, GPIO_AN, PULL_NONE, 0);
    hw_gpio(PIN_ERR_L_PORT, PIN_ERR_L_PIN, GPIO_AN, PULL_NONE, 0);
    hw_gpio(PIN_REF_R_PORT, PIN_REF_R_PIN, GPIO_AN, PULL_NONE, 0);
    hw_gpio(PIN_ERR_R_PORT, PIN_ERR_R_PIN, GPIO_AN, PULL_NONE, 0);
    hw_gpio(PIN_DAC_L_PORT, PIN_DAC_L_PIN, GPIO_AN, PULL_NONE, 0);
    hw_gpio(PIN_DAC_R_PORT, PIN_DAC_R_PIN, GPIO_AN, PULL_NONE, 0);
    hw_gpio(PIN_DBG_TP_PORT, PIN_DBG_TP_PIN, GPIO_OUT, PULL_NONE, 0);

    RCC->AHB1ENR |= RCC_AHB1ENR_ADC12EN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1LENR |= RCC_APB1LENR_TIM6EN | RCC_APB1LENR_DAC12EN;
    (void)RCC->APB1LENR;

    /* ---- DAC1: both channels, buffered, no trigger ---- */
    DAC1->MCR = 0;
    DAC1->DHR12RD = (2048u << DAC_DHR12RD_DACC2DHR_Pos) | 2048u;
    DAC1->CR = DAC_CR_EN1 | DAC_CR_EN2;

    /* ---- ADC1 master / ADC2 slave ---- */
    ADC12_COMMON->CCR = (0u << ADC_CCR_CKMODE_Pos) | (0u << ADC_CCR_PRESC_Pos);
    adc_power_up(ADC1);
    adc_power_up(ADC2);
    const uint32_t cfg_master = (0u << ADC_CFGR_RES_Pos) | (1u << ADC_CFGR_EXTEN_Pos) | (EXTSEL_TIM6 << ADC_CFGR_EXTSEL_Pos)
                              | ADC_CFGR_OVRMOD | (3u << ADC_CFGR_DMNGT_Pos);     /* DMA circular */
    const uint32_t cfg_slave  = (0u << ADC_CFGR_RES_Pos) | ADC_CFGR_OVRMOD;
    adc_channels(ADC1, ADC_CH_REF_L, ADC_CH_ERR_L, cfg_master);
    adc_channels(ADC2, ADC_CH_REF_R, ADC_CH_ERR_R, cfg_slave);
    /* dual regular simultaneous (0b00110), 32-bit packed data (DAMDF=0b10) */
    ADC12_COMMON->CCR |= (6u << ADC_CCR_DUAL_Pos) | (2u << ADC_CCR_DAMDF_Pos);
    adc_enable(ADC1);
    adc_enable(ADC2);

    /* ---- DMA1 Stream0 <- ADC12 CDR ---- */
    DMA1_Stream0->CR = 0;
    while (DMA1_Stream0->CR & DMA_SxCR_EN) {}
    DMAMUX1_Channel0->CCR = DMAMUX_ADC1;
    DMA1_Stream0->PAR = (uint32_t)&ADC12_COMMON->CDR;
    DMA1_Stream0->M0AR = (uint32_t)dma_buf;
    DMA1_Stream0->NDTR = 2;
    DMA1_Stream0->FCR = 0;                             /* direct mode */
    DMA1_Stream0->CR = (3u << DMA_SxCR_PL_Pos) | (2u << DMA_SxCR_MSIZE_Pos) | (2u << DMA_SxCR_PSIZE_Pos)
                     | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_TCIE;
    NVIC_SetPriority(DMA1_Stream0_IRQn, 0);
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);

    /* ---- TIM6: 200 MHz / 6250 = 32 kHz, TRGO on update ---- */
    TIM6->PSC = 0;
    TIM6->ARR = (APB1_TIMER_HZ / FS_HZ) - 1u;
    TIM6->CR2 = (2u << TIM_CR2_MMS_Pos);
    TIM6->EGR = TIM_EGR_UG;
}

void audio_start(void)
{
    DMA1->LIFCR = 0x3Fu;                                 /* clear stream0 flags */
    DMA1_Stream0->NDTR = 2;
    DMA1_Stream0->CR |= DMA_SxCR_EN;
    ADC1->CR |= ADC_CR_ADSTART;                          /* master start arms both ADCs */
    TIM6->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

void audio_stop(void)
{
    TIM6->CR1 &= ~TIM_CR1_CEN;
    ADC1->CR |= ADC_CR_ADSTP;
    while (ADC1->CR & ADC_CR_ADSTART) {}
    DMA1_Stream0->CR &= ~DMA_SxCR_EN;
    DAC1->DHR12RD = (2048u << DAC_DHR12RD_DACC2DHR_Pos) | 2048u;
}

__attribute__((section(".itcm"))) void DMA1_Stream0_IRQHandler(void)
{
    const uint32_t c0 = DWT->CYCCNT;
    PIN_DBG_TP_PORT->BSRR = 1u << PIN_DBG_TP_PIN;
    DMA1->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0;

    const uint32_t w0 = dma_buf[0], w1 = dma_buf[1];
    const audio_in_t in = {
        .ref_l = (uint16_t)(w0 & 0xFFFFu), .ref_r = (uint16_t)(w0 >> 16),
        .err_l = (uint16_t)(w1 & 0xFFFFu), .err_r = (uint16_t)(w1 >> 16),
    };
    app_audio_frame(&in);                      /* writes the DAC as early as it can, then adapts */

    if (ADC1->ISR & ADC_ISR_OVR) { ADC1->ISR = ADC_ISR_OVR; audio_overruns++; }
    audio_frames++;
    const uint32_t dt = DWT->CYCCNT - c0;
    audio_isr_cycles = dt;
    if (dt > audio_isr_cycles_max) audio_isr_cycles_max = dt;
    PIN_DBG_TP_PORT->BSRR = 1u << (PIN_DBG_TP_PIN + 16u);
}
