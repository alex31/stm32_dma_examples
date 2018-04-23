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
#include "simpleSerialMessage.h"


/*

  ° connecter A08, A09, A10, A11 sur des controleurs flyduino KISS24 ou KISS32
  ° connecter B06 (uart1_tx) sur ftdi rx
  ° connecter B07 (uart1_rx) sur ftdi tx
  ° connecter C00 sur led1 

*/


/*
#                                         __   _     __ _         
#                                        / _| (_)   / _` |        
#                  ___    ___    _ __   | |_   _   | (_| |        
#                 / __|  / _ \  | '_ \  |  _| | |   \__, |        
#                | (__  | (_) | | | | | | |   | |    __/ |        
#                 \___|  \___/  |_| |_| |_|   |_|   |___/         
*/
#define DSHOT_SPEED_KHZ			600
#define DSHOT_FRAME_SILENT_SYNC_BITS	4
#define DSHOT_CHANNELS			4
#define DSHOT_TELEMETRY_BAUD		115200

#define BENCH_TELEMETRY_BAUD	        115200


/*
#                     _            __   _            _    _      _                          
#                    | |          / _| (_)          (_)  | |    (_)                         
#                  __| |    ___  | |_   _    _ __    _   | |_    _     ___    _ __          
#                 / _` |   / _ \ |  _| | |  | '_ \  | |  | __|  | |   / _ \  | '_ \         
#                | (_| |  |  __/ | |   | |  | | | | | |  \ |_   | |  | (_) | | | | |        
#                 \__,_|   \___| |_|   |_|  |_| |_| |_|   \__|  |_|   \___/  |_| |_|        
*/
#define DSHOT_BIT_WIDTHS 16
#define DSHOT_DMA_BUFFER_SIZE	 (DSHOT_BIT_WIDTHS+DSHOT_FRAME_SILENT_SYNC_BITS)
#define PWM_FREQ         168000 // the timer will beat @84Mhz
#define TICKS_PER_PERIOD 1000  // that let use any timer :
			       // does not care if linked to PCLK1 or PCLK2
			       // tick_per_period will be dynamically calculated 
			       // after pwm init
#define DSHOT_SPEED (DSHOT_SPEED_KHZ*1000)
#define TICK_FREQ (PWM_FREQ * TICKS_PER_PERIOD)
#define DSHOT_PWM_PERIOD (TICK_FREQ/DSHOT_SPEED)
#define DSHOT_BIT0_DUTY (DSHOT_PWM_PERIOD * 373 / 1000)
//#define DSHOT_BIT1_DUTY (DSHOT_PWM_PERIOD * 684 / 1000)
#define DSHOT_BIT1_DUTY (DSHOT_BIT0_DUTY*2)
#define    DCR_DBL              ((4-1) << 8) // 2 transfert
// first register to get is CCR1
#define DCR_DBA                 (((uint32_t *) (&PWMD1.tim->CCR) - ((uint32_t *) PWMD1.tim))) 


typedef uint16_t timer_reg_t;

typedef union {
  struct {
    uint16_t crc:4;
    uint16_t telemetryRequest:1;
    uint16_t throttle:11;
  };
  uint16_t rawFrame;
}  DshotPacket;

typedef struct {
  DshotPacket dp[DSHOT_CHANNELS];
} DshotPackets;

typedef struct {
  timer_reg_t widths[DSHOT_DMA_BUFFER_SIZE][DSHOT_CHANNELS] __attribute__((aligned(16)));
} DshotDmaBuffer;

typedef enum  {PWM_ORDER=0, CALIBRATE} IncomingMessageId;
typedef struct {
  uint16_t msgId;
  int16_t  escIdx;
  int16_t  duty;
} TelemetryDownMsg;



/*
#                 _ __                   _              _      _   _    _ __                 
#                | '_ \                 | |            | |    | | | |  | '_ \                
#                | |_) |  _ __    ___   | |_     ___   | |_   | |_| |  | |_) |   ___         
#                | .__/  | '__|  / _ \  | __|   / _ \  | __|   \__, |  | .__/   / _ \        
#                | |     | |    | (_) | \ |_   | (_) | \ |_     __/ |  | |     |  __/        
#                |_|     |_|     \___/   \__|   \___/   \__|   |___/   |_|      \___|        
*/



DshotPacket makeDshotPacket(const uint16_t throttle, bool tlmRequest);
void        buildDshotDmaBuffer(const DshotPackets * const dsp,  DshotDmaBuffer * const dma);
static void telemetryReceive_cb(const uint8_t *buffer, const size_t len,  void * const userData);
//static noreturn void sendTelemetryThd (void *arg);
static noreturn void blinker (void *arg);


/*
#                  __ _   _            _               _          
#                 / _` | | |          | |             | |         
#                | (_| | | |    ___   | |__     __ _  | |         
#                 \__, | | |   / _ \  | '_ \   / _` | | |         
#                  __/ | | |  | (_) | | |_) | | (_| | | |         
#                 |___/  |_|   \___/  |_.__/   \__,_| |_|         
*/
//static THD_WORKING_AREA(waSendTelemetry, 1024);
static THD_WORKING_AREA(waBlinker, 512);

static const SerialConfig  hostcfg =  {
  .speed = BENCH_TELEMETRY_BAUD,
  .cr1 = 0,                                      // pas de parité
  .cr2 = USART_CR2_STOP1_BITS | USART_CR2_LINEN, // 1 bit de stop, detection d'erreur de trame avancée
  .cr3 = 0                                       // pas de controle de flux hardware (CTS, RTS)
};

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
  .circular = false,
  .error_cb = NULL,
  .end_cb = NULL,
  .pburst = 0,
  .mburst = 4,
  .fifo = 0
};

static DMADriver dmap;
static DshotPackets dshotMotors;




static PWMConfig pwmcfg = {     
  .frequency = TICK_FREQ,       
  .period    = TICKS_PER_PERIOD,
  .callback  = NULL,            
  .channels  = {
    {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = NULL},
    {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = NULL},
    {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = NULL},
    {.mode = PWM_OUTPUT_ACTIVE_HIGH, .callback = NULL},
  },
  .cr2  =  STM32_TIM_CR2_CCDS, 
  .dier =  STM32_TIM_DIER_UDE
};


  
static DshotDmaBuffer dsdb;

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
  sdStart(&SD1, &hostcfg);
  //consoleInit();

  chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, blinker, NULL);
  dmaStart(&dmap, &dmaConfig);
  pwmStart(&PWMD1, &pwmcfg);
  PWMD1.tim->DCR = DCR_DBL | DCR_DBA; // enable bloc register DMA transaction
  pwmChangePeriod(&PWMD1, DSHOT_PWM_PERIOD);

  for (size_t j=0;j<4;j++)
    pwmEnableChannel(&PWMD1, j, 1);
  
  //  consoleLaunch();
  simpleMsgBind ((BaseSequentialStream *) &SD1, telemetryReceive_cb,
		 NULL, NULL);

  dshotMotors.dp[0] = dshotMotors.dp[1] = makeDshotPacket(0,0);
  dshotMotors.dp[2] = dshotMotors.dp[3] = makeDshotPacket(1046,0);
  
  buildDshotDmaBuffer(&dshotMotors, &dsdb);
  
  
  while (true) {
    if (dmap.state == DMA_READY) {
      dmaStartTransfert(&dmap, &TIM1->DMAR, &dsdb, DSHOT_DMA_BUFFER_SIZE * DSHOT_CHANNELS);
    } else {
      DebugTrace ("DMA Not Ready");
    }
    chThdSleepMicroseconds(1000); // 1khz
  }
}



DshotPacket makeDshotPacket(const uint16_t throttle, bool tlmRequest)
{
  DshotPacket dp = {.throttle = throttle,
		    .telemetryRequest =  (tlmRequest ? 1 : 0),
		    .crc = 0};
  
  // compute checksum
  uint16_t csum = (throttle << 1) | (tlmRequest ? 1 : 0);
  for (int i = 0; i < 3; i++) {
    dp.crc ^=  csum;   // xor data by nibbles
    csum >>= 4;
  }
  
  return dp;
}

void buildDshotDmaBuffer(const DshotPackets * const dsp,  DshotDmaBuffer * const dma)
{
  for (size_t chanIdx=0; chanIdx < DSHOT_CHANNELS; chanIdx++) {
    for (size_t bitIdx=0; bitIdx < DSHOT_BIT_WIDTHS; bitIdx++) {
      dma->widths[bitIdx][chanIdx] = dsp->dp[chanIdx].rawFrame &
	(1 << ((DSHOT_BIT_WIDTHS -1) - bitIdx)) ?
	DSHOT_BIT1_DUTY : DSHOT_BIT0_DUTY;
    }

    // silence for sync in case of continous sending
    for (size_t bitIdx=DSHOT_BIT_WIDTHS; bitIdx < DSHOT_DMA_BUFFER_SIZE; bitIdx++) {
      dma->widths[bitIdx][chanIdx] = 0;
    }
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
    switch (msg->msgId) {
    case PWM_ORDER : {
      const uint32_t rawThrottle = msg->duty;
      int throttle=0;
      if (rawThrottle) 
	throttle = rawThrottle < 2047 ? rawThrottle : 2047;
      
      dshotMotors.dp[msg->escIdx] = makeDshotPacket(throttle, 0);
      buildDshotDmaBuffer(&dshotMotors, &dsdb);
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
