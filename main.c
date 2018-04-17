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
#include "math.h"


/*

  ° connecter A08, A09, A10, A11 sur des leds
  ° connecter B06 (uart1_tx) sur ftdi rx
  ° connecter B07 (uart1_rx) sur ftdi tx
  ° connecter C00 sur led1 

*/

#define DSHOT_SPEED_KHZ	 150

#define DSHOT_BIT_WIDTHS 16
#define DSHOT_FRAME_SILENT_SYNC_BITS 4
#define DSHOT_DMA_BUFFER_SIZE	 (DSHOT_BIT_WIDTHS+DSHOT_FRAME_SILENT_SYNC_BITS)
#define CHANNELS	 4
#define PWM_FREQ         84000 // the timer will beat @84Mhz
#define TICKS_PER_PERIOD 1000  // that let use any timer :
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
    uint16_t throttle:11;
    uint16_t telemetryRequest:1;
    uint16_t crc:4;
  };
  uint16_t rawFrame;
}  DshotPacket;

typedef struct {
  DshotPacket dp[CHANNELS];
} DshotPackets;

typedef struct {
  timer_reg_t widths[DSHOT_DMA_BUFFER_SIZE][CHANNELS] __attribute__((aligned(16)));
} DshotDmaBuffer;

DshotPackets dshotMotors;


DshotPacket makeDshotPacket(const uint16_t throttle, bool tlmRequest);
void        buildDshotDmaBuffer(const DshotPackets * const dsp,  DshotDmaBuffer * const dma);


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
  .mburst = 0,
  .fifo = 0
};

static DMADriver dmap;



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

  consoleInit();

  chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, blinker, NULL);
  dmaStart(&dmap, &dmaConfig);
  pwmStart(&PWMD1, &pwmcfg);
  PWMD1.tim->DCR = DCR_DBL | DCR_DBA; // enable bloc register DMA transaction
  pwmChangePeriod(&PWMD1, DSHOT_PWM_PERIOD);

  for (size_t j=0;j<4;j++)
    pwmEnableChannel(&PWMD1, j, 1);
  
  consoleLaunch();

  dshotMotors.dp[0] = makeDshotPacket(1025,0);
  dshotMotors.dp[1] = makeDshotPacket(514,0);
  dshotMotors.dp[2] = makeDshotPacket(260,1);
  dshotMotors.dp[3] = makeDshotPacket(1046,0);
  
  DshotDmaBuffer dsdb;
  buildDshotDmaBuffer(&dshotMotors, &dsdb);
  
  while (true) {
    if (dmap.state == DMA_READY) {
      dmaStartTransfert(&dmap, &TIM1->DMAR, &dsdb, DSHOT_DMA_BUFFER_SIZE * CHANNELS);
    } else {
      DebugTrace ("DMA Not Ready");
    }
    chThdSleepMilliseconds(1);
  }
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
  for (size_t chanIdx=0; chanIdx < CHANNELS; chanIdx++) {
    for (size_t bitIdx=0; bitIdx < DSHOT_BIT_WIDTHS; bitIdx++) {
      dma->widths[bitIdx][chanIdx] = dsp->dp[chanIdx].rawFrame & (1 << bitIdx) ?
	DSHOT_BIT1_DUTY : DSHOT_BIT0_DUTY;
    }
    for (size_t bitIdx=DSHOT_BIT_WIDTHS; bitIdx < DSHOT_DMA_BUFFER_SIZE; bitIdx++) {
      dma->widths[bitIdx][chanIdx] = 0;
    }
  }
}
