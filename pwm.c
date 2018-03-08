#include <ch.h>
#include <hal.h>
#include <stdnoreturn.h>
#include <math.h>
#include "globalVar.h"
#include "stdutil.h"
#include "pwm.h"


/* #define PATTERN_SIZE            8 */
/* #define STM32_TIM_CR2_CCDS      (1U << 3) */

/* // PWM configuration Timer1 */
/* static PWMConfig pwmcfg1 = { */
/*   84000000, /\* PWM clock frequency *\/ */
/*   42, /\* PWM period *\/ */
/*   NULL,  /\* No callback *\/ */
/*   { */
/*     {PWM_OUTPUT_DISABLED, NULL}, */
/*     {PWM_OUTPUT_DISABLED, NULL}, */
/*     {PWM_OUTPUT_DISABLED, NULL}, */
/*     {PWM_OUTPUT_ACTIVE_HIGH, NULL}, */
/*   }, */
/*   STM32_TIM_CR2_CCDS, /\* CR2 register initialization*\/ */
/*   #if STM32_PWM_USE_ADVANCED */
/*   0, /\* TIM BDTR (break & dead-time) register initialization *\/ */
/*   #endif */
/*   STM32_TIM_DIER_UDE /\* TIM DIER register initialization, is ignored by OS *\/ */
/* }; */

/* /\* */
/*  * Application entry point. */
/*  *\/ */
/* int main(void) { */

/*   uint16_t pattern[PATTERN_SIZE] = {0x0101,0x0202,0x0404,0x0808, */
/*                                       0x1010,0x2020,0x4040,0x8080}; */
                            
/*   halInit(); */
/*   chSysInit(); */

/*   // configure the timer in up/down to reload after 84 timer clock tics */
/*   // The clock is running at 84,000,000Hz, */
/*   // so 84,000,000 / 42 /2 = 1.0 MHz in up/down mode */
/*   pwmStart(&PWMD1, &pwmcfg1); */
/*   pwmEnableChannel(&PWMD1, 3, 20); */

/*   // customize timer setup to center aligned mode */
/*   PWMD1.tim->CR1 |= STM32_TIM_CR1_CMS(0x01); */
/*   // center aligned mode would generate update events (aka DMA-requests) */
/*   // on overflow and underflow situations. Repetion counter waits */
/*   // N+1 events before next interrupt/DMA request is generated. */
/*   // So now DMA is only triggered on underflow with pwm-pulse in between. */
/*   PWMD1.tim->RCR = 0x01; */

/*   // OS resets UDE-bit, so set it here again */
/*   PWMD1.tim->DIER |= STM32_TIM_DIER_UDE; */

/*   // Allocate the stream */
/*   dmaStreamAllocate( STM32_DMA2_STREAM5, 0, NULL, NULL ); */

/*   // Set the source Address */
/*   dmaStreamSetPeripheral( STM32_DMA2_STREAM5, &GPIOF->ODR ); */

/*   // Set the destination address */
/*   dmaStreamSetMemory0( STM32_DMA2_STREAM5, pattern ); */

/*   // set the size of the output buffer */
/*   dmaStreamSetTransactionSize( STM32_DMA2_STREAM5, PATTERN_SIZE ); */

/*   // config 16-bit HWORD transfers, memory to peripheral, */
/*   // inc source address, fixed dest address, */
/*   // circular mode, select channel 6 */
/*   dmaStreamSetMode( STM32_DMA2_STREAM5, */
/*                     STM32_DMA_CR_PL(0) | STM32_DMA_CR_PSIZE_HWORD | */
/*                     STM32_DMA_CR_MSIZE_HWORD | STM32_DMA_CR_DIR_M2P | */
/*                     STM32_DMA_CR_MINC | STM32_DMA_CR_CIRC | */
/*                     STM32_DMA_CR_CHSEL(6)); */
 
/*   // init FIFO to 0 aka only one element */
/*   dmaStreamSetFIFO(STM32_DMA2_STREAM5, STM32_DMA_FCR_FTH_1Q); */
/*   dmaStreamEnable(STM32_DMA2_STREAM5); */
 
/*   while (TRUE) { */
/*       chThdSleepMilliseconds(500); */
/*   } */
/* } */





// pour les parties 1 (persistance) et 2 (servo) et 3 (il variation continue du pwm)
// n'y a pas besoin de modifier  la structure de configuration.
// Il n'y a que dans la partie Utilisation des 4 canaux
// que le champ .channels de cette structure devra être modifié
static PWMConfig pwmcfg = {	// pwm d'une frequence d'un hz et 10000 pas de quantification
  .frequency = 10000,        // TickFreq : PwmFreq(100) * ticksPerPeriod(100)  
  .period    = 100       , //   tickPerPeriod (100)
  .callback  = NULL,	     //   pas de callback de passage à l'etat actif
  .channels  = {
    // sortie active, polarité normale, pas de callback
    {.mode = PWM_OUTPUT_DISABLED, .callback = NULL},
    // sortie inactive
    {.mode = PWM_OUTPUT_DISABLED, .callback = NULL},
    // sortie inactive
    {.mode = PWM_OUTPUT_DISABLED, .callback = NULL},
    // sortie inactive
    {.mode = PWM_OUTPUT_DISABLED, .callback = NULL}
  },
  .cr2  =  STM32_TIM_CR2_CCDS, 
  .dier =  STM32_TIM_DIER_UDE
};


void launchPwm (const uint16_t pwmFreq, const uint16_t tickPerPeriod)
{
  pwmcfg.frequency = pwmFreq * tickPerPeriod;
  pwmcfg.period = tickPerPeriod;

  pwmStart(&PWMD1, &pwmcfg);
  pwmEnableChannel(&PWMD1, 0, tickPerPeriod/2);

  // customize timer setup to center aligned mode
  //  PWMD1.tim->CR1 |= STM32_TIM_CR1_CMS(0x01);

  // So now DMA is only triggered on underflow with pwm-pulse in between.
  //PWMD1.tim->RCR = 0x01;
}




