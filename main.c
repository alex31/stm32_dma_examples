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
#include "hal_stm32_dma.h"


/*
  ° connecter A04 (ADC1_IN4) sur le potentiomètre à bouton
  ° laisser le jumper entre +3.3V et TOPs pour ce potentiomètre
  ° connecter A15 sur led8 
  ° connecter A15 sur A08 : rebouclage PWM vers ICU
  ° connecter B06 (uart1_tx) sur ftdi rx
  ° connecter B07 (uart1_rx) sur ftdi tx
  ° connecter C00 sur led1 
 */


#define DCR_DBL			(1 << 8) // 2 transfert
// first register to get is CCR1
#define DCR_DBA			(((uint32_t *) (&ICUD1.tim->CCR) - ((uint32_t *) ICUD1.tim))) 

static void initDma(void);
static bool startDma(void);
static void startDmaAcquisition(uint16_t *widthOrPeriod,
				size_t depth);
static void error_cb(DMADriver *dmap, dmaerrormask_t err);
static void end_cb(DMADriver *dmap, void *buffer, const size_t num);

static volatile dmaerrormask_t last_err = 0;
static volatile uint16_t *last_half_buffer = NULL;
static volatile size_t last_num = 0;
static volatile uint16_t *last_half_buffer_after_tmout = NULL;
static volatile size_t last_num_after_tmout = 0;
static volatile systime_t timestamp = 0;

static const DMAConfig dmaConfig = {
  .stream = STM32_ICU1_CH1_DMA_STREAM,
  .channel = STM32_ICU1_CH1_DMA_CHANNEL,
  .dma_priority = STM32_ICU1_CH1_DMA_PRIORITY,
  .irq_priority = STM32_ICU1_CH1_DMA_IRQ_PRIORITY,
  //.periph_addr = &ICUD1.tim->DMAR, : not a constant, should have to use cmsis definition
  //  .periph_addr = &TIM1->DMAR,
  .direction = DMA_DIR_P2M,
  .psize = 2,
  .msize = 2,
  .inc_peripheral_addr = false,
  .inc_memory_addr = true,
  .circular = true,
  .error_cb = &error_cb,
#if STM32_DMA_USE_ASYNC_TIMOUT
  .timeout = TIME_MS2I(100),
#endif
  .end_cb = &end_cb,
  .mburst = 8,
  .fifo = 4
};

static DMADriver dmap;

static const ICUConfig icu1ch1_cfg = {
  .mode = ICU_INPUT_ACTIVE_HIGH,
  .frequency = 1000000,                                    /* 1Mhz ICU clock frequency.   */
  .width_cb = NULL,
  .period_cb = NULL,
  .overflow_cb = NULL,
  .channel = ICU_CHANNEL_1,
  .dier = TIM_DIER_CC1DE | TIM_DIER_TDE
};




static THD_WORKING_AREA(waBlinker, 512);
static noreturn void blinker (void *arg);
#define DMA_DATA_LEN 128

static uint16_t samples[DMA_DATA_LEN] __attribute__((aligned(16))) = {0}; // took 0.0128 seconds to fill

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
  initDma();
  
  consoleInit();

  initPotentiometre();

  launchPwm(10000, 100);
  
  chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, blinker, NULL);
  startDma();
  icuStart(&ICUD1, &icu1ch1_cfg);
  ICUD1.tim->DCR = DCR_DBL | DCR_DBA;
  startDmaAcquisition(samples, DMA_DATA_LEN);
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
    if (last_half_buffer == NULL) {
      DebugTrace ("cd not yet called");
    } else {
      for (size_t i=0; i< 2; i++) {
	DebugTrace ("samples[%u] = %u half_buffer=%p half_index=%u last_num=%u", i, samples[i],
		    last_half_buffer, last_half_buffer-samples, last_num);
      }
      for (size_t i=DMA_DATA_LEN-2; i<DMA_DATA_LEN ; i++) {
	DebugTrace ("samples[%u] = %u half_buffer_after_tmout=%p last_num_after_tmout=%u", i, samples[i],
		    last_half_buffer_after_tmout, last_num_after_tmout);
      }
      DebugTrace ("was called %lu ms ago", chVTGetSystemTime()-timestamp);
    }
    palToggleLine(LINE_C00_LED1); 	
    chThdSleepMilliseconds(1000);
  }
}


static void initDma(void)
{
   dmaObjectInit(&dmap);
}

static bool startDma(void)
{
   return dmaStart(&dmap, &dmaConfig);
}


static void startDmaAcquisition(uint16_t *widthsAndPeriods,
				const size_t depth)
{
  dmaAcquireBus(&dmap);
  dmaStartTransfert(&dmap, &TIM1->DMAR, widthsAndPeriods, depth);
}



static void error_cb(DMADriver *_dmap, dmaerrormask_t err)
{
  (void) _dmap;
  last_err = err;
}

static void end_cb(DMADriver *_dmap, void *buffer, size_t num)
{
  (void) _dmap;
  static bool last_call_was_from_tmout = false;

  chSysLockFromISR();
  timestamp = chVTGetSystemTimeX();
  chSysUnlockFromISR();
  last_half_buffer = (uint16_t *) buffer;
  last_num = num;
  if (num != DMA_DATA_LEN/2) {
    if (last_call_was_from_tmout) {
      last_half_buffer_after_tmout = last_half_buffer;
      last_num_after_tmout = last_num;
    } else {
      last_call_was_from_tmout = true;
    }
  } else {
    last_call_was_from_tmout = false;
  }
}
