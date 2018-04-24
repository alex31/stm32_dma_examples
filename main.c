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

  ° connecter A03 (uart2_rx) sur sorties TLM des controleurs flyduino KISS24 ou KISS32 (en //)
  ° connecter A08, A09, A10, A11 sur entrée PWM des controleurs flyduino KISS24 ou KISS32
  ° connecter (cavalier)  B06 (uart1_tx) sur sonde rx : interface graphique de commande
  ° connecter (cavalier)  B07 (uart1_rx) sur sonde tx : interface graphique de commande
  ° connecter B10 (uart3_tx) sur ftdi rx :  shell
  ° connecter B11 (uart3_rx) sur ftdi tx :  shell
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
#define PWM_FREQ         (STM32_SYSCLK/2000) // the timer will beat @84Mhz on STM32F4
#define TICKS_PER_PERIOD 1000             // that let use any timer :
			                  // does not care if linked to PCLK1 or PCLK2
			                  // tick_per_period will be dynamically calculated 
			                  // after pwm init
#define DSHOT_SPEED (DSHOT_SPEED_KHZ*1000)
#define TICK_FREQ (PWM_FREQ * TICKS_PER_PERIOD)
#define DSHOT_PWM_PERIOD (TICK_FREQ/DSHOT_SPEED)
#define DSHOT_BIT0_DUTY (DSHOT_PWM_PERIOD * 373 / 1000)
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
  union {
    struct {
      uint8_t  temp;
      uint16_t voltage;
      uint16_t current;
      uint16_t consumption;
      uint16_t rpm;
    } __attribute__ ((__packed__, scalar_storage_order ("big-endian")));
    uint8_t rawData[9];
  };
  uint8_t  crc8;
}  __attribute__ ((__packed__)) DshotTelemetry ;

typedef struct {
  DshotPacket       dp[DSHOT_CHANNELS];
  DshotTelemetry    dt[DSHOT_CHANNELS];
  volatile uint8_t  currentTlmQry;
  volatile bool	    onGoingQry;
} DshotPackets;

typedef struct {
  // alignment to satisfy dma requirement
  timer_reg_t widths[DSHOT_DMA_BUFFER_SIZE][DSHOT_CHANNELS] __attribute__((aligned(16)));
} DshotDmaBuffer;

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
static noreturn void sendTelemetryThd (void *arg);
static noreturn void blinker (void *arg);
static noreturn void dshotTlmRec (void *arg);
static uint8_t updateCrc8(uint8_t crc, uint8_t crc_seed);
uint8_t calculateCrc8(const uint8_t *Buf, const uint8_t BufLen);

/*
#                  __ _   _            _               _          
#                 / _` | | |          | |             | |         
#                | (_| | | |    ___   | |__     __ _  | |         
#                 \__, | | |   / _ \  | '_ \   / _` | | |         
#                  __/ | | |  | (_) | | |_) | | (_| | | |         
#                 |___/  |_|   \___/  |_.__/   \__,_| |_|         
*/
static msg_t  _mbBuf[1];
static MAILBOX_DECL(mb, _mbBuf, sizeof(_mbBuf)/sizeof(_mbBuf[0])); 

static THD_WORKING_AREA(waSendTelemetry, 1024);
static THD_WORKING_AREA(waBlinker, 512);
static THD_WORKING_AREA(waDshotTlmRec, 512);
static volatile int throttle=0;


static const SerialConfig  hostcfg =  {
  .speed = BENCH_TELEMETRY_BAUD,
  .cr1 = 0,                                      // pas de parité
  .cr2 = USART_CR2_STOP1_BITS | USART_CR2_LINEN, // 1 bit de stop
  .cr3 = 0                                       // pas de controle de flux hardware (CTS, RTS)
};

static const SerialConfig  tlmcfg =  {
  .speed = DSHOT_TELEMETRY_BAUD,
  .cr1 = 0,                                      // pas de parité
  .cr2 = USART_CR2_STOP1_BITS,			 // 1 bit de stop
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
  sdStart(&SD2, &tlmcfg);
  consoleInit();

  chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, blinker, NULL);
  chThdCreateStatic(waDshotTlmRec, sizeof(waDshotTlmRec), NORMALPRIO, dshotTlmRec, NULL);
  chThdCreateStatic(waSendTelemetry, sizeof(waSendTelemetry), NORMALPRIO, &sendTelemetryThd, NULL);
  
  dmaStart(&dmap, &dmaConfig);
  pwmStart(&PWMD1, &pwmcfg);
  PWMD1.tim->DCR = DCR_DBL | DCR_DBA; // enable bloc register DMA transaction
  pwmChangePeriod(&PWMD1, DSHOT_PWM_PERIOD);

  for (size_t j=0;j<4;j++)
    pwmEnableChannel(&PWMD1, j, 1);
  
  consoleLaunch();
  simpleMsgBind ((BaseSequentialStream *) &SD1, telemetryReceive_cb,
		 NULL, NULL);

  dshotMotors.dp[0] = dshotMotors.dp[1] = makeDshotPacket(0,0);
  dshotMotors.dp[2] = dshotMotors.dp[3] = makeDshotPacket(1046,0);
  
  buildDshotDmaBuffer(&dshotMotors, &dsdb);
  
  int count = 0;
  while (true) {
    if (dmap.state == DMA_READY) {
      if (count++ >= 100) {
	dshotMotors.dp[0] = makeDshotPacket(throttle, 1);
	chMBPostTimeout(&mb, 0, TIME_IMMEDIATE);
	count = 0;
      } else {
	dshotMotors.dp[0] = makeDshotPacket(throttle, 0);
      }
      buildDshotDmaBuffer(&dshotMotors, &dsdb);
      dmaStartTransfert(&dmap, &TIM1->DMAR, &dsdb, DSHOT_DMA_BUFFER_SIZE * DSHOT_CHANNELS);
    } else {
      DebugTrace ("DMA Not Ready");
    }
    chThdSleepMicroseconds(500); // 1khz
  }
}



DshotPacket makeDshotPacket(const uint16_t _throttle, bool tlmRequest)
{
  DshotPacket dp = {.throttle = _throttle,
		    .telemetryRequest =  (tlmRequest ? 1 : 0),
		    .crc = 0};
  
  // compute checksum
  uint16_t csum = (_throttle << 1) | (tlmRequest ? 1 : 0);
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

static uint8_t updateCrc8(uint8_t crc, uint8_t crc_seed)
{
    uint8_t crc_u = crc;
    crc_u ^= crc_seed;

    for (int i=0; i<8; i++) {
        crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
    }

    return (crc_u);
}

uint8_t calculateCrc8(const uint8_t *Buf, const uint8_t BufLen)
{
    uint8_t crc = 0;
    for (int i = 0; i < BufLen; i++) {
        crc = updateCrc8(Buf[i], crc);
    }

    return crc;
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
      if (rawThrottle) 
	throttle = rawThrottle < 2047 ? rawThrottle : 2047;
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




static noreturn void dshotTlmRec (void *arg)
{
  (void)arg;
  uint32_t escIdx=0;
  
  chRegSetThreadName("dshotTlmRec");
  while (true) {
    chMBFetchTimeout(&mb, (msg_t *) &escIdx, TIME_INFINITE);
    const uint32_t idx = escIdx;
    dshotMotors.onGoingQry = true;
    sdRead(&SD2, dshotMotors.dt[idx].rawData, sizeof(DshotTelemetry));
    dshotMotors.onGoingQry = false;

     if (calculateCrc8(dshotMotors.dt[idx].rawData, 
		       sizeof(dshotMotors.dt[idx].rawData)) != dshotMotors.dt[idx].crc8) {
       DebugTrace ("Dshot Telemetry error");
       // empty buffer to resync
       while (sdGetTimeout(&SD2, TIME_IMMEDIATE) >= 0) {};
     } else {
       /* DebugTrace("[%lu] temp=%u volt=%.2f current=%.2f consum=%u mA rpm=%u", */
       /* 		  idx, */
       /* 		  dshotMotors.dt[idx].temp, */
       /* 		  dshotMotors.dt[idx].voltage/100.0, */
       /* 		  dshotMotors.dt[idx].current/100.0, */
       /* 		  dshotMotors.dt[idx].consumption, */
       /* 		  dshotMotors.dt[idx].rpm*100); */
     }
  }
}

 /* my ($msgId, $bat_voltage, $current, $consumption,		    */
 /* 	$rpm, $temperature, $channel) = unpack ('Lf5L', $$bufferRef); */

static void sendTelemetryThd (void *arg)
{
  (void)arg;
  chRegSetThreadName("telemetry");

  while (true) {
    for (int idx=0; idx<2; idx++) {
      const CommandUpMsg upMsg = (CommandUpMsg) {.msgId = 0,
			    .voltage = dshotMotors.dt[idx].voltage/100.0,
			    .current = dshotMotors.dt[idx].current/100.0,
			    .consumption = dshotMotors.dt[idx].consumption,
			    .rpm = dshotMotors.dt[idx].rpm*100,
			    .temperature = dshotMotors.dt[idx].temp,
			    .escIdx = idx
      };
      simpleMsgSend((BaseSequentialStream *) &SD1, (uint8_t *) &upMsg, sizeof(upMsg)); 
    }
    
    chThdSleepMilliseconds(100);
  }
}


