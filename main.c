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


#define M2M_DMA_CONTROLER	2
#define M2M_DMA_STREAM		6
#define M2M_DMA_IRQ_PRIORITY	6
#define M2M_DMA_PRIORITY	2
#define M2M_DMA_CHANNEL		0


static const DMAConfig dmaConfig = {
  .controller = M2M_DMA_CONTROLER,
  .stream = M2M_DMA_STREAM,
  .channel = M2M_DMA_CHANNEL,
  .dma_priority =  M2M_DMA_PRIORITY,
  .irq_priority = M2M_DMA_IRQ_PRIORITY,
  .direction = DMA_DIR_M2M,
  .psize = 4,
  .msize = 4,
  .inc_peripheral_addr = true,
  .inc_memory_addr = true,
  .circular = false,
  .mburst = 4,
  .fifo = 4
};

static DMADriver dmap;

static THD_WORKING_AREA(waBlinker, 512);
static noreturn void blinker (void *arg);

static uint32_t bufferFrom[1024];
static uint32_t bufferTo[1024];

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
  dmaStart(&dmap, &dmaConfig);


  for (size_t i=0; i<ARRAY_LEN(bufferFrom); i++) {
    bufferFrom[i]=i;
  }
  
  consoleInit();

  chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, blinker, NULL);
  
  consoleLaunch();  
  chThdSleep(TIME_INFINITE); 
}


static noreturn void blinker (void *arg)
{

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    dmaTransfert(&dmap, bufferFrom, bufferTo, ARRAY_LEN(bufferFrom));
    for (size_t i=0; i<ARRAY_LEN(bufferTo); i+=50) {
      DebugTrace ("To[%u] = %lu", i, bufferTo[i]);
    }

    palToggleLine(LINE_C00_LED1);
    chThdSleepMilliseconds(500);
  }
}



