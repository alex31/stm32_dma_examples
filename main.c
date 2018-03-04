/*
  Nom(s), prénom(s) du ou des élèves : 
 */

#include <ch.h>
#include <hal.h>
#include <stdnoreturn.h>
#include <math.h>
#include "globalVar.h"
#include "stdutil.h"
#include "ttyConsole.h"
#include "potentiometre.h"
#include "pwm.h"



/*

  ° connecter A04 (ADC1_IN4) sur le potentiomètre à bouton
  ° laisser le jumper entre +3.3V et TOPs pour ce potentiomètre
  ° connecter A15 sur led8 
  ° connecter A15 sur A08 : rebouclage PWM vers ICU
  ° connecter B06 (uart1_tx) sur ftdi rx
  ° connecter B07 (uart1_rx) sur ftdi tx
  ° connecter C00 sur led1 
 */


#define ICU1_CH1_DMA_STREAM     STM32_DMA_STREAM(STM32_DMA_STREAM_ID(2, 6))
#define ICU1_CH1_DMA_PRIORITY	6
#define ICU1_CH1_DMA_CHANNEL	0
#define ICU1_CHANNEL		ICU_CHANNEL_1




static volatile bool dmaIsrHasFired = false;

static bool dmaStart(void);
static void dmaStop(void);
static void dmaStartAcquisition(uint16_t *widthOrPeriod,
				size_t depth);

static void icu_lld_serve_rx_interrupt(ICUDriver *icup, uint32_t flags);


static const ICUConfig icu1ch1_cfg = {
  .mode = ICU_INPUT_ACTIVE_HIGH,
  .frequency = 1000000,                                    /* 10kHz ICU clock frequency.   */
  .width_cb = NULL,
  .period_cb = NULL,
  .overflow_cb = NULL,
  .channel = ICU1_CHANNEL,
  .dier = TIM_DIER_CC1DE | TIM_DIER_TDE
};




static THD_WORKING_AREA(waBlinker, 512);
static noreturn void blinker (void *arg);

static uint16_t samples[128] = {0}; // took 0.0128 seconds to fill

int main(void) {

    /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */

  halInit();
  chSysInit();
  initHeap();

  consoleInit();

  initPotentiometre();
  launchPwm(10000, 100);
  
  chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, blinker, NULL);
  dmaStart();
  icuStart(&ICUD1, &icu1ch1_cfg);
  dmaStartAcquisition(samples, ARRAY_LEN(samples));
  icuStartCapture(&ICUD1);
  //icuEnableNotifications(&ICUD1);
  
  consoleLaunch();  
  chThdSleep(TIME_INFINITE); 
}


static noreturn void blinker (void *arg)
{

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    for (size_t i=0; i< 4; i++) {
      DebugTrace ("samples[%ul] = %u ISR has %s fired", i, samples[i], dmaIsrHasFired ? "" : "NOT");
    }
    dmaIsrHasFired = false;
    palToggleLine(LINE_C00_LED1); 	
    chThdSleepMilliseconds(500);
  }
}



static bool dmaStart(void)
{
  // state to be managed
  bool b;

  osalSysLock();
  b = dmaStreamAllocate(ICU1_CH1_DMA_STREAM,
			ICU1_CH1_DMA_PRIORITY,
			(stm32_dmaisr_t)icu_lld_serve_rx_interrupt,
			(void *) &ICUD1);
  osalDbgAssert(!b, "stream already allocated");
  dmaStreamSetPeripheral(ICU1_CH1_DMA_STREAM, &ICUD1.tim->CCR[ICU1_CHANNEL+1]);
  osalSysUnlock();
  
  return b;
}

static void dmaStop(void)
{
  osalSysLock();
  dmaStreamRelease(ICU1_CH1_DMA_STREAM);
  osalSysUnlock();
}

static void dmaStartAcquisition(uint16_t *widthsAndPeriods,
				const size_t depth)
{
  (void) widthsAndPeriods;
  (void) depth;
  const stm32_dma_stream_t  *dmastream = ICU1_CH1_DMA_STREAM;

  uint32_t dmamode = STM32_DMA_CR_CHSEL(ICU1_CH1_DMA_CHANNEL) |
    STM32_DMA_CR_PL(ICU1_CH1_DMA_PRIORITY) |              // ** Half transfert ISR (HTIE) missing ?
    STM32_DMA_CR_DIR_P2M | // Peripheral to memory
    STM32_DMA_CR_MSIZE_HWORD | STM32_DMA_CR_PSIZE_HWORD | // 16 bits to 16 bits transfert
    STM32_DMA_CR_MINC        | STM32_DMA_CR_TCIE        | // memory increment, transfert complete ISR enable
    STM32_DMA_CR_DMEIE       | STM32_DMA_CR_TEIE; // direct mode error and transfert error ISR enable

  if (depth > 1) {
    dmamode |= (STM32_DMA_CR_CIRC | STM32_DMA_CR_HTIE);
  }
  
  dmaStreamSetMemory0(dmastream, widthsAndPeriods);
  dmaStreamSetTransactionSize(dmastream, depth);
  dmaStreamSetMode(dmastream, dmamode);
  dmaStreamEnable(dmastream);


  osalSysLock();
  

  osalSysUnlock();
}



static void icu_lld_serve_rx_interrupt(ICUDriver *icup, uint32_t flags)
{
  (void) icup;
  dmaIsrHasFired = true;
  /* DMA errors handling.*/
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
    /* DMA, this could help only if the DMA tries to access an unmapped
       address space or violates alignment rules.*/
    chSysHalt ("icu_lld_serve_rx_interrupt dma error");
  }
  else {
    /* /\* It is possible that the conversion group has already be reset by the */
    /*    ICU error handler, in this case this interrupt is spurious.*\/ */
    /* if (icup->grpp != NULL) { */

    /*   if ((flags & STM32_DMA_ISR_TCIF) != 0) { */
    /*     /\* Transfer complete processing.*\/ */
    /*     _icu_isr_full_code(icup); */
    /*   } */
    /*   else if ((flags & STM32_DMA_ISR_HTIF) != 0) { */
    /*     /\* Half transfer processing.*\/ */
    /*     _icu_isr_half_code(icup); */
    /*   } */
    /* } */
  }
}
