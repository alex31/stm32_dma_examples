/*
  Nom(s), prénom(s) du ou des élèves : 
 */

#include <ch.h>
#include <hal.h>
#include <stdnoreturn.h>
#include <math.h>
#include <string.h>
#include "globalVar.h"
#include "stdutil.h"
#include "ttyConsole.h"
#include "esc_dshot.h"
#include "simpleSerialMessage.h"


/*

  ° connecter A03 (uart2_rx) sur sorties TLM des controleurs flyduino KISS24 ou KISS32 (en //)
  ° connecter A08, A09, A10, A11 sur entrée PWM des controleurs flyduino KISS24 ou KISS32
  ° connecter (cavalier)  B06 (uart1_tx) sur sonde rx : interface graphique de commande
  ° connecter (cavalier)  B07 (uart1_rx) sur sonde tx : interface graphique de commande
  ° connecter B10 (uart3_tx) sur ftdi rx :  shell
  ° connecter B11 (uart3_rx) sur ftdi tx :  shell
  ° connecter C00 sur led1 
*/

#define BENCH_TELEMETRY_BAUD		115200

typedef enum  {PWM_ORDER=0, CALIBRATE} IncomingMessageId;

typedef struct {
  uint16_t msgId;
  int16_t  escIdx;
  int16_t  duty;
} TelemetryDownMsg;

typedef struct {
  uint32_t msgId;
  float	   voltage;
  float	   current;
  float	   consumption;
  float	   rpm;
  float	   temperature;
  uint32_t escIdx;
} CommandUpMsg;




static const SerialConfig  hostcfg =  {
  .speed = BENCH_TELEMETRY_BAUD,
  .cr1 = 0,                                      // pas de parité
  .cr2 = USART_CR2_STOP1_BITS | USART_CR2_LINEN, // 1 bit de stop
  .cr3 = 0                                       // pas de controle de flux hardware (CTS, RTS)
};


static const DSHOTConfig dshotConfig = {
  .dma_stream = STM32_PWM1_UP_DMA_STREAM,
  .dma_channel = STM32_PWM1_UP_DMA_CHANNEL,
  .pwmp = &PWMD1,
  .tlm_sd = &SD2
};

DSHOTDriver dshotd;

static volatile uint16_t throttle;
static volatile dshot_special_commands_t specialCommand = DSHOT_CMD_MAX;

static THD_WORKING_AREA(waBlinker, 512);
static noreturn void blinker (void *arg);
static THD_WORKING_AREA(waSendTelemetry, 512);
static noreturn void sendTelemetryThd (void *arg);
static void telemetryReceive_cb(const uint8_t *buffer, const size_t len,  void * const userData);


int main(void)
{
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
  
  sdStart(&SD1, &hostcfg);
  dshotStart(&dshotd, &dshotConfig);
  consoleInit();
  consoleLaunch();
  simpleMsgBind ((BaseSequentialStream *) &SD1, telemetryReceive_cb,
		 NULL, NULL);
  
  
  chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, blinker, NULL);
  chThdCreateStatic(waSendTelemetry, sizeof(waSendTelemetry), NORMALPRIO, &sendTelemetryThd, NULL);
  
  
  while (true) {
    if (specialCommand == DSHOT_CMD_MAX) {
      dshotSetThrottle(&dshotd, 0, throttle);
      dshotSendFrame(&dshotd);
    } else {
      dshotSendSpecialCommand(&dshotd, 0, specialCommand);
      specialCommand = DSHOT_CMD_MAX;
    }
    chThdSleepMicroseconds(1000); // 1khz
  }
}




/*
#                                _    _    _                      _            
#                               | |  | |  | |                    | |           
#                  ___    __ _  | |  | |  | |__     __ _    ___  | | _         
#                 / __|  / _` | | |  | |  | '_ \   / _` |  / __| | |/ /        
#                | (__  | (_| | | |  | |  | |_) | | (_| | | (__  |   <         
#                 \___|  \__,_| |_|  |_|  |_.__/   \__,_|  \___| |_|\_\        
*/


static void telemetryReceive_cb(const uint8_t *buffer, const size_t len,  void * const userData)
{
  (void) userData;

  

  if (len != sizeof(TelemetryDownMsg)) {
    DebugTrace ("Msg len error : rec %u instead of waited %u", len, sizeof(TelemetryDownMsg));
  } else {
    const TelemetryDownMsg *msg = (TelemetryDownMsg *) buffer;
    if (msg->escIdx != 0)
      return;
    switch (msg->msgId) {
    case PWM_ORDER : {
      const uint32_t rawThrottle = msg->duty;
      if (rawThrottle >= 48) { 
	throttle = rawThrottle < 2047 ? rawThrottle : 2047;
      } else {
	specialCommand = msg->duty;
      }
    }
      break;
    case CALIBRATE : DebugTrace ("Calibrate not yet implemented");
      break;
    default : DebugTrace ("message not yet implemented");
      break;
    }
  }
}



/*
#                 _      _                                 _                 
#                | |    | |                               | |                
#                | |_   | |__    _ __    ___    __ _    __| |   ___          
#                | __|  | '_ \  | '__|  / _ \  / _` |  / _` |  / __|         
#                \ |_   | | | | | |    |  __/ | (_| | | (_| |  \__ \         
#                 \__|  |_| |_| |_|     \___|  \__,_|  \__,_|  |___/         
*/
static noreturn void blinker (void *arg)
{

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palToggleLine(LINE_C00_LED1); 	
    chThdSleepMilliseconds(500);
  }
}



 /* my ($msgId, $bat_voltage, $current, $consumption,		    */
 /* 	$rpm, $temperature, $channel) = unpack ('Lf5L', $$bufferRef); */

static void sendTelemetryThd (void *arg)
{
  (void)arg;
  chRegSetThreadName("telemetry");

  while (true) {
    for (uint32_t idx=0; idx < DSHOT_CHANNELS; idx++) {
      const DshotTelemetry *tlm = dshotGetTelemetry(&dshotd, idx);
      if (tlm->temp != 0) {
	const CommandUpMsg upMsg = (CommandUpMsg) {.msgId = 0,
						   .voltage = tlm->voltage/100.0,
						   .current = tlm->current/100.0,
						   .consumption = tlm->consumption,
						   .rpm = tlm->rpm*100,
						   .temperature = tlm->temp,
						   .escIdx = idx
	};
	simpleMsgSend((BaseSequentialStream *) &SD1, (uint8_t *) &upMsg, sizeof(upMsg));
      }
      chThdSleepMilliseconds(100);
    }
  }
}


