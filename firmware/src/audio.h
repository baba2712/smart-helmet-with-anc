/* audio.h - 32 kHz sample-synchronous I/O: TIM6 -> ADC1+ADC2 (dual simultaneous) -> DMA -> ISR -> DAC1 */
#ifndef AUDIO_H
#define AUDIO_H

#include <stdint.h>
#include "stm32h7xx.h"

/* raw 16-bit ADC codes for one frame */
typedef struct { uint16_t ref_l, err_l, ref_r, err_r; } audio_in_t;

/* implemented by the application (app.c); runs inside the DMA ISR at 32 kHz.
 * It must call audio_dac_write() as soon as the outputs are known (before adapting). */
void app_audio_frame(const audio_in_t *in);

/* both DAC channels in one bus write; the pins update one APB clock later */
static inline void audio_dac_write(uint32_t code_l, uint32_t code_r)
{
    DAC1->DHR12RD = (code_r << DAC_DHR12RD_DACC2DHR_Pos) | code_l;
}

void audio_init(void);
void audio_start(void);
void audio_stop(void);                         /* DAC to mid-scale, conversions stopped */

extern volatile uint32_t audio_isr_cycles;     /* last ISR duration (CPU cycles) */
extern volatile uint32_t audio_isr_cycles_max;
extern volatile uint32_t audio_frames;
extern volatile uint32_t audio_overruns;

#endif
