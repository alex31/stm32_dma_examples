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
#include "hal_stm32_dma.h"
#include "bitband.h"


/*

  ° connecter A08 sur analyseur logique
  ° connecter B06 (uart1_tx) sur ftdi rx
  ° connecter B07 (uart1_rx) sur ftdi tx
  ° connecter C00 sur led1 

*/

#define WIDTHS_SIZE	 1024
#define PWM_FREQ         500
#define TICKS_PER_PERIOD 22400 // we can also use 1e4 notation

#define TICK_FREQ (PWM_FREQ * TICKS_PER_PERIOD)

typedef uint16_t timer_reg_t;
timer_reg_t widths[WIDTHS_SIZE] __attribute__((aligned(16))); 

 
static const DMAConfig dmaConfig = {
  .stream = STM32_PWM1_UP_DMA_STREAM,
  .channel = STM32_PWM1_UP_DMA_CHANNEL,
  .dma_priority = STM32_PWM1_UP_DMA_PRIORITY,
  .irq_priority = STM32_PWM1_UP_DMA_IRQ_PRIORITY,
  .direction = DMA_DIR_M2P,
  .psize = sizeof(timer_reg_t), // if we change for a 32 bit timer just have to change
  .msize = sizeof(timer_reg_t), // type of width array
  .inc_peripheral_addr = false,
  .inc_memory_addr = true,
  .circular = true,
  .error_cb = NULL,
  .end_cb = NULL,
  .pburst = 0,
  .mburst = 0,
  .fifo = 0
};

static DMADriver dmap;


// pour les parties 1 (persistance) et 2 (servo) et 3 (il variation continue du pwm)
// n'y a pas besoin de modifier  la structure de configuration.
// Il n'y a que dans la partie Utilisation des 4 canaux
// que le champ .channels de cette structure devra être modifié
static PWMConfig pwmcfg = {     // pwm d'une frequence d'un hz et 10000 pas de quantification
  .frequency = TICK_FREQ,        // TickFreq : PwmFreq(1) * ticksPerPeriod(10000)  
  .period    = TICKS_PER_PERIOD, //   tickPerPeriod (10000)
  .callback  = NULL,             //   pas de callback de passage à l'etat actif
  .channels  = {
    // sortie active, polarité normale, pas de callback
    {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = NULL},
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



static THD_WORKING_AREA(waBlinker, 512);
static noreturn void blinker (void *arg);


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


  for (size_t i=0; i< WIDTHS_SIZE; i++) {
    widths[i] = i*(1.0f*TICKS_PER_PERIOD/WIDTHS_SIZE);
  }
  
  consoleInit();

  chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, blinker, NULL);
  dmaStart(&dmap, &dmaConfig);
  pwmStart(&PWMD1, &pwmcfg);
  pwmEnableChannel(&PWMD1, 0, 1); 
  dmaStartTransfert(&dmap, &TIM1->CCR1, widths, WIDTHS_SIZE);

  
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

