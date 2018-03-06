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
#include "hal_dma.h"


/*

  ° connecter A04 (ADC1_IN4) sur le potentiomètre à bouton
  ° laisser le jumper entre +3.3V et TOPs pour ce potentiomètre
  ° connecter A15 sur led8 
  ° connecter A15 sur A08 : rebouclage PWM vers ICU
  ° connecter B06 (uart1_tx) sur ftdi rx
  ° connecter B07 (uart1_rx) sur ftdi tx
  ° connecter C00 sur led1 
 */


#define ICU1_CH1_DMA_CONTROLER		2
#define ICU1_CH1_DMA_STREAM		6
#define ICU1_CH1_DMA_PRIORITY	6
#define ICU1_CH1_DMA_CHANNEL	0
#define ICU1_CHANNEL		ICU_CHANNEL_1
#define DCR_DBL			(1 << 8) // 2 transfert
// first register to get is CCR1
#define DCR_DBA			(((uint32_t *) (&ICUD1.tim->CCR) - ((uint32_t *) ICUD1.tim))) 

static void icu_lld_serve_rx_interrupt(void *icup, uint32_t flags);
static bool dmaStart(void);
static void dmaStop(void);
static void dmaStartAcquisition(uint16_t *widthOrPeriod,
				size_t depth);

static volatile bool dmaIsrHasFired = false;

static const DMAConfig dmaConfig = {
  .controller = ICU1_CH1_DMA_CONTROLER,
  .stream = ICU1_CH1_DMA_STREAM,
  .channel = ICU1_CH1_DMA_CHANNEL,
  .priority = ICU1_CH1_DMA_PRIORITY,
  .serve_dma_isr = &icu_lld_serve_rx_interrupt,
  .serve_dma_isr_arg = &ICUD1,
  //.periph_addr = &ICUD1.tim->DMAR, : not a constant, should have to use cmsis definition
  .periph_addr = &TIM1->DMAR,
  .direction = STM32_DMA_CR_DIR_P2M,
  .psize = 2,
  .msize = 2,
  .inc_peripheral_addr = false,
  .inc_memory_addr = true,
  .circular = true,
  .isr_flags = STM32_DMA_CR_TCIE | STM32_DMA_CR_HTIE | STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE,
  .pburst = 4,
  .mburst = 4,
  .fifo = 2
};

static DMADriver dmad;

static const ICUConfig icu1ch1_cfg = {
  .mode = ICU_INPUT_ACTIVE_HIGH,
  .frequency = 1000000,                                    /* 1Mhz ICU clock frequency.   */
  .width_cb = NULL,
  .period_cb = NULL,
  .overflow_cb = NULL,
  .channel = ICU1_CHANNEL,
  .dier = TIM_DIER_CC1DE | TIM_DIER_TDE
};




static THD_WORKING_AREA(waBlinker, 512);
static noreturn void blinker (void *arg);

static uint16_t samples[128] __attribute__((aligned(4))) = {0}; // took 0.0128 seconds to fill

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
  ICUD1.tim->DCR = DCR_DBL | DCR_DBA;
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
    for (size_t i=0; i< 2; i++) {
      DebugTrace ("samples[%ul] = %u ISR has %s fired", i, samples[i], dmaIsrHasFired ? "" : "NOT");
    }
    for (size_t i=ARRAY_LEN(samples)-2; i<ARRAY_LEN(samples) ; i++) {
      DebugTrace ("samples[%ul] = %u", i, samples[i]);
    }
    dmaIsrHasFired = false;
    palToggleLine(LINE_C00_LED1); 	
    chThdSleepMilliseconds(500);
  }
}



static bool dmaStart(void)
{
 
  return dma_start(&dmad, &dmaConfig);
}

static void dmaStop(void)
{
   dma_stop(&dmad);
}

static void dmaStartAcquisition(uint16_t *widthsAndPeriods,
				const size_t depth)
{
  dma_start_ptransfert(&dmad, widthsAndPeriods, depth);
}



static void icu_lld_serve_rx_interrupt(void *icup, uint32_t flags)
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
