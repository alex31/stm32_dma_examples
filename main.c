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

  ° connecter A04 (ADC1_IN11) sur le potentiomètre à bouton
  ° laisser le jumper entre +3.3V et TOPs pour ce potentiomètre
  ° connecter A15 sur led8 
  ° connecter A15 sur A08 : rebouclage PWM vers ICU
  ° connecter B06 (uart1_tx) sur ftdi rx
  ° connecter B07 (uart1_rx) sur ftdi tx
  ° connecter C00 sur led1 
 */
static void icuOverflowCb(ICUDriver *icup);
static bool hasOverflood = false;


static const ICUConfig icucfg = {
  .mode = ICU_INPUT_ACTIVE_HIGH,
  .frequency = 10000,                                    /* 10kHz ICU clock frequency.   */
  .width_cb = NULL,
  .period_cb = NULL,
  .overflow_cb = icuOverflowCb,
  .channel = ICU_CHANNEL_1,
  .dier = 0
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

  consoleInit();

  initPotentiometre();
  launchPwm(100, 100);
  
  chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, blinker, NULL);
  icuStart(&ICUD1, &icucfg);
  icuStartCapture(&ICUD1);
  icuEnableNotifications(&ICUD1);
  
  consoleLaunch();  
  chThdSleep(TIME_INFINITE); 
}


static noreturn void blinker (void *arg)
{

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    if (hasOverflood) {
      hasOverflood = false;
      DebugTrace ("** overflow **");
    } else {
      DebugTrace("period = %lu, width = %lu", icuGetPeriodX(&ICUD1), icuGetWidthX(&ICUD1));
    }
    palToggleLine(LINE_C00_LED1); 	
    chThdSleepMilliseconds(500);
  }
}

static void icuOverflowCb(ICUDriver *icup)
{
  (void) icup;
  hasOverflood = true;
}
