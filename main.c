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
#include "pwm.h"
#include "hal_dma.h"


/*

  ° connecter A04 (ADC1_IN4) sur le potentiomètre à bouton
  ° laisser le jumper entre +3.3V et TOPs pour ce potentiomètre
  ° connecter B06 (uart1_tx) sur ftdi rx
  ° connecter B07 (uart1_rx) sur ftdi tx
  ° connecter C00 sur led1 
 */


#define TIM1_CH1_DMA_CONTROLER		2
#define TIM1_CH1_DMA_STREAM		5
#define TIM1_CH1_DMA_IRQ_PRIORITY	6
#define TIM1_CH1_DMA_PRIORITY		2
#define TIM1_CH1_DMA_CHANNEL		6

static const DMAConfig dmaConfig = {
  .controller = TIM1_CH1_DMA_CONTROLER,
  .stream = TIM1_CH1_DMA_STREAM,
  .channel = TIM1_CH1_DMA_CHANNEL,
  .dma_priority =  TIM1_CH1_DMA_PRIORITY,
  .irq_priority = TIM1_CH1_DMA_IRQ_PRIORITY,
  //.periph_addr = &ICUD1.tim->DMAR, : not a constant, should have to use cmsis definition
  //  .periph_addr = &TIM1->DMAR,
  .direction = DMA_DIR_M2P,
  .psize = 2,
  .msize = 2,
  .inc_peripheral_addr = false,
  .inc_memory_addr = true,
  .circular = true,
  .fifo = 1
};

static DMADriver dmap;


static THD_WORKING_AREA(waBlinker, 512);
static noreturn void blinker (void *arg);
static void initPattern(void);

static uint16_t pattern[4] __attribute__((aligned(4))) = {0}; 

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
  
  initPattern();
  consoleInit();

  launchPwm(1, 10000);
  
  chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, blinker, NULL);
  dmaStart(&dmap, &dmaConfig);
  dmaStartTransfert(&dmap, &GPIOB->ODR, pattern, ARRAY_LEN(pattern));
  
  //chSysHalt("test dma");
  consoleLaunch();
  chThdSleep(TIME_INFINITE); 
}


static noreturn void blinker (void *arg)
{

  (void)arg;
  chRegSetThreadName("blinker");

  while (true) {
    palToggleLine(LINE_C00_LED1); 	
    chThdSleepMilliseconds(1000);
  }
}



static void initPattern(void)
{
  for (size_t i=0; i<ARRAY_LEN(pattern); i++) {
    pattern[i] = 1<<i;
  }
}
