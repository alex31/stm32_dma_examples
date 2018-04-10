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

  ° alimenter DHT22
  ° connecter DATA DHT22 sur A08 
  ° connecter B06 (uart1_tx) sur ftdi rx
  ° connecter B07 (uart1_rx) sur ftdi tx
  ° connecter C00 sur led1 

 */

/*===========================================================================*/
/* DHT11 related defines                                                     */
/*===========================================================================*/
/*
 * Width are in useconds
 */
#define    MCU_REQUEST_WIDTH                     18000
#define    DHT_ERROR_WIDTH                         200
#define    DHT_START_BIT_WIDTH                      80
#define    DHT_INTER_BIT_WIDTH                      50

/*===========================================================================*/
/* ICU related code                                                          */
/*===========================================================================*/

#define    ICU_TIM_FREQ         1e6 // 1Mhz
#define    ICU1_CHANNEL		ICU_CHANNEL_1


static msg_t  mbBuf[256];
static MAILBOX_DECL(mb, mbBuf, sizeof(mbBuf)/sizeof(mbBuf[0])); 


static void error_cb(DMADriver *dmap, dmaerrormask_t err);
static void end_cb(DMADriver *dmap, void *buffer, const size_t n);
static volatile dmaerrormask_t last_err = 0;

typedef union {
  struct {
    uint16_t humidity;
    uint16_t temp;
    uint8_t  chksum; 
  } __attribute__ ((scalar_storage_order ("big-endian"))) ; // DHT halfword are big_endian
							    // but MCU is little_endian
    uint8_t raw [5];
}   DhtData;

// icucnt_t is 32 bits wide for 32bits timer, but we use TIM1 which is 16 bits timer
// let's optimize
typedef  uint16_t timer_reg_t;

// 48 is bigger than needed 40, but burst mode implies size of buffer to be multiple of size of
// burst data (4*2 = 8) 
volatile timer_reg_t widths[256] __attribute__((aligned(16))); 

 
static const DMAConfig dmaConfig = {
  .stream = STM32_ICU1_CH1_DMA_STREAM,
  .channel = STM32_ICU1_CH1_DMA_CHANNEL,
  .dma_priority = STM32_ICU1_CH1_DMA_PRIORITY,
  .irq_priority = STM32_ICU1_CH1_DMA_IRQ_PRIORITY,
  //.periph_addr = &ICUD1.tim->DMAR, : not a constant, should have to use cmsis definition
  //  .periph_addr = &TIM1->DMAR,
  .direction = DMA_DIR_P2M,
  .psize = sizeof(timer_reg_t), // if we change for a 32 bit timer just have to change
  .msize = sizeof(timer_reg_t), // type of width array
  .inc_peripheral_addr = false,
  .inc_memory_addr = true,
  .circular = true,
  .end_cb = &end_cb,
  .error_cb = &error_cb,
  .timeout = TIME_MS2I(200),
  .pburst = 4,
  .mburst = 4,
  .fifo = 0
};

static DMADriver dmap;

static const ICUConfig icu1ch1_cfg = {
  .mode = ICU_INPUT_ACTIVE_HIGH,
  .frequency = ICU_TIM_FREQ,
  .width_cb = NULL,
  .period_cb = NULL,
  .overflow_cb = NULL,
  .channel = ICU1_CHANNEL,
  .dier = TIM_DIER_CC1DE | TIM_DIER_TDE
};


static THD_WORKING_AREA(waBlinker, 512);
static noreturn void blinker (void *arg);

static THD_WORKING_AREA(waDht22Acquisition, 512);
static noreturn void dht22Acquisition (void *arg);

static THD_WORKING_AREA(waDht22sendStartPulse, 512);
static noreturn void dht22sendStartPulse (void *arg);

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
  icuStartCapture(&ICUD1);
  // launch a DMA transfert in continuous mode
  dmaStartTransfert(&dmap, &ICUD1.tim->CCR[1], (timer_reg_t *) widths, ARRAY_LEN(widths));

  chThdCreateStatic(waDht22Acquisition, sizeof(waDht22Acquisition), NORMALPRIO, dht22Acquisition, NULL);
  chThdCreateStatic(waDht22sendStartPulse, sizeof(waDht22sendStartPulse), NORMALPRIO, dht22sendStartPulse, NULL);
  
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


static noreturn void dht22sendStartPulse (void *arg)
{
  (void)arg;
  chRegSetThreadName("dht22sendStartPulse");

  // when the pin is in gpio mode, should be forced LOW
  palClearLine(LINE_A08_ICU_IN);
  
  while (true) {

    // do the interrogation pulse on the pin
    palSetLineMode(LINE_A08_ICU_IN, PAL_MODE_OUTPUT_OPENDRAIN);
    chThdSleepMicroseconds(MCU_REQUEST_WIDTH);

    // revert pin to input capture mode
    palSetLineMode(LINE_A08_ICU_IN, PAL_MODE_ALTERNATE(AF_LINE_A08_ICU_IN)
		   | PAL_STM32_PUPDR_PULLUP);

    chThdSleepMilliseconds(1000);
  }
}



static noreturn void dht22Acquisition (void *arg)
{
  (void)arg;
  chRegSetThreadName("dht22Acquisition");

  static DhtData dhtData;

  // use bitbanding capability of STM32F4 (see ref manuel RM090)
  volatile uint32_t * const fhtRegBB = bb_sramp(&dhtData, 0);

  
  // generate binary stream using pulse width registered by ICU+DMA
  uint8_t bit_counter=0;
  while (true) {
    timer_reg_t width;
    //    if (chMBFetchTimeout (&mb, (msg_t *) &width, TIME_MS2I(500)) != MSG_OK) {
    //      DebugTrace ("***** TIMEOUT @ %u *******",  bit_counter);
    chMBFetchTimeout (&mb, (msg_t *) &width, TIME_INFINITE);
      //    }
    //    DebugTrace ("width[%u] = %u", bit_counter, width);
    if (width >= DHT_START_BIT_WIDTH) {
      /* starting bit resetting the bit counter */
      bit_counter = 0;
    } else  {
      fhtRegBB[bit_counter++] = width >= DHT_INTER_BIT_WIDTH;
    }
    // if frame is completely acquired, exit loop 
    if (bit_counter == 40) {
  
      // decode binary stream and generate numeric field

      // DHT most significant bit is bit0, but MCU  most significant bit is bit7
      // revbit reverse the bits order in a byte/halfword,word
      for (uint i=0; i< sizeof(dhtData.raw); i++) {
	dhtData.raw[i] = revbit(dhtData.raw[i]);
      }
    
      // compute and compare checksum
      if (dhtData.raw[4] == ((dhtData.raw[0] + dhtData.raw[1] +
			      dhtData.raw[2] + dhtData.raw[3]) & 0xff)) {

	const float humidity = dhtData.humidity / 10.0f;
	const float temp = (dhtData.temp & 0x7fff) / ((dhtData.temp & 0x8000) ? - 10.0f : 10.0f);
	chprintf(chp, "Temperature: %.2f C, Humidity Rate: %.2f %% \n\r",
		 temp, humidity);
      } else {
	chprintf(chp, "Checksum FAILED!\n\r");
      }
      bit_counter = 0;
    }
  }
}




static void error_cb(DMADriver *_dmap, dmaerrormask_t err)
{
  (void) _dmap;
  last_err |= err;
}

static void end_cb(DMADriver *_dmap, void *buffer, const size_t n)
{
  (void) _dmap;
  
  chSysLockFromISR();
  volatile timer_reg_t * const bArray = (volatile timer_reg_t *) buffer;
  for (size_t i=0; i<n; i++) {
    chMBPostI(&mb, (msg_t) bArray[i]);
  }
  chSysUnlockFromISR();
}

