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



#define ICU1_CHANNEL		ICU_CHANNEL_1
#define DCR_DBL			(1 << 8) // 2 transfert
// first register to get is CCR1
#define DCR_DBA			(((uint32_t *) (&ICUD1.tim->CCR) - ((uint32_t *) ICUD1.tim))) 


static void error_cb(DMADriver *dmap, dmaerrormask_t err);
static void end_cb(DMADriver *dmap, void *buffer, const size_t num);

static volatile dmaerrormask_t last_err = 0;
static volatile uint16_t *last_half_buffer = NULL;
static volatile size_t last_num = 0;
static volatile size_t cb_count = 0;

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
  .circular = false,
  .error_cb = &error_cb,
  .end_cb = &end_cb,
  .pburst = 0,
  .mburst = 0,
  .fifo = 0
};

static DMADriver dmap;

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

static THD_WORKING_AREA(waSyncDmaTest, 512);
static noreturn void syncDmaTest (void *arg);

static uint16_t widthAndPeriods[48] __attribute__((aligned(4))) = {0}; // took 0.0128 seconds to fill

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
  dmaObjectInit(&dmap);
  
  consoleInit();

  chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, blinker, NULL);
  dmaStart(&dmap, &dmaConfig);
  icuStart(&ICUD1, &icu1ch1_cfg);
  ICUD1.tim->DCR = DCR_DBL | DCR_DBA;
  icuStartCapture(&ICUD1);
  chThdCreateStatic(waSyncDmaTest, sizeof(waSyncDmaTest), NORMALPRIO, syncDmaTest, NULL);
  
  consoleLaunch();  
  chThdSleep(TIME_INFINITE); 
}


static noreturn void blinker (void *arg)
{

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palToggleLine(LINE_C00_LED1); 	
    chThdSleepMilliseconds(500);
  }
}

static noreturn void syncDmaTest (void *arg)
{

  (void)arg;
  chRegSetThreadName("syncDmaTest");
  while (true) {
    msg_t status = dmaTransfertTimeout(&dmap, &TIM1->DMAR, widthAndPeriods, ARRAY_LEN(widthAndPeriods),
				       TIME_MS2I(100));
     if (status != MSG_OK) {
       DebugTrace ("DMA Time OUT");
     } else {
     }

     chThdSleepMilliseconds(1000);
  }
}



static void error_cb(DMADriver *_dmap, dmaerrormask_t err)
{
  (void) _dmap;
  last_err = err;
}

static void end_cb(DMADriver *_dmap, void *buffer, size_t num)
{
  (void) _dmap;

  if (buffer != widthAndPeriods)
    last_half_buffer = (uint16_t *) buffer;

  last_num = num;
  cb_count++;
}
