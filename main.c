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

#define    ICU_TIM_FREQ         1000000
#define    ICU1_CHANNEL		ICU_CHANNEL_1
#define    DCR_DBL		(1 << 8) // 2 transfert
// first register to get is CCR1
#define DCR_DBA			(((uint32_t *) (&ICUD1.tim->CCR) - ((uint32_t *) ICUD1.tim))) 


static void error_cb(DMADriver *dmap, dmaerrormask_t err);
static void end_cb(DMADriver *dmap, void *buffer, const size_t num);

static volatile dmaerrormask_t last_err = 0;
static volatile size_t	       cb_count = 0;

typedef union {
  struct {
    uint16_t humidity;
    uint16_t temp;
    uint8_t  chksum;
  } __attribute__ ((scalar_storage_order ("big-endian"))) ;
    uint8_t raw [5];
}   DhtData;

static const DMAConfig dmaConfig = {
  .stream = STM32_ICU1_CH1_DMA_STREAM,
  .channel = STM32_ICU1_CH1_DMA_CHANNEL,
  .dma_priority = STM32_ICU1_CH1_DMA_PRIORITY,
  .irq_priority = STM32_ICU1_CH1_DMA_IRQ_PRIORITY,
  //.periph_addr = &ICUD1.tim->DMAR, : not a constant, should have to use cmsis definition
  //  .periph_addr = &TIM1->DMAR,
  .direction = DMA_DIR_P2M,
  .psize = 4,
  .msize = 4,
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
  .frequency = ICU_TIM_FREQ,
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

  DhtData dhtData;
  icucnt_t widthAndPeriods[84] __attribute__((aligned(4))) = {0xff}; // took 0.0128 seconds to fill


  while (true) {
    palSetLineMode(LINE_A08_ICU_IN, PAL_MODE_OUTPUT_PUSHPULL);
    palClearLine(LINE_A08_ICU_IN);
    chThdSleepMicroseconds(MCU_REQUEST_WIDTH);
    palSetLine(LINE_A08_ICU_IN);
    
    
    /*
     * Initializes the ICU driver 1.
     * GPIOA8 is the ICU input.
     */
    palSetLineMode(LINE_A08_ICU_IN, PAL_MODE_ALTERNATE(AF_LINE_A08_ICU_IN)
		   | PAL_STM32_PUPDR_PULLUP);
    chThdSleepMilliseconds(5);

    cb_count =0;
    msg_t status = dmaTransfertTimeout(&dmap, &TIM1->DMAR, widthAndPeriods, ARRAY_LEN(widthAndPeriods),
				       TIME_MS2I(1000));
    if (status != MSG_OK) {
      DebugTrace ("DMA Time OUT");
    }

    DebugTrace ("cb_count = %d", cb_count);
    //chprintf(chp, "\033[2J\033[1;1H");

    // decodage trame binaire DHT

    uint8_t bit_counter=0;
    volatile uint32_t * const fhtRegBB = bb_sramp(&dhtData, 0);
    for (size_t i=0; i<ARRAY_LEN(widthAndPeriods)/2; i++) {
      const icucnt_t width = widthAndPeriods[1+(bit_counter*2)];
      DebugTrace ("width[%u] = %lu", bit_counter, width);
      if (width >= DHT_START_BIT_WIDTH)
	/* starting bit resetting the bit counter */
	bit_counter = 0;
      else 
	fhtRegBB[bit_counter++] = width >= DHT_INTER_BIT_WIDTH;
    }
  }
  
  // decodage champs DHT
  for (uint i=0; i< sizeof(dhtData.raw); i++) {
    dhtData.raw[i] = revbit(dhtData.raw[i]);
  }
  
  const float humidity = dhtData.humidity / 10.0f;
  const float temp = (dhtData.temp & 0x7fff) / ((dhtData.temp & 0x8000) ? - 10.0f : 10.0f);
  
  if (dhtData.raw[4] == ((dhtData.raw[0]+dhtData.raw[1]+dhtData.raw[2]+dhtData.raw[3]) & 0xff)) {
    chprintf(chp, "Temperature: %.2f C, Humidity Rate: %.2f %% \n\r",
	     temp, humidity);
  } else {
    chprintf(chp, "Checksum FAILED!\n\r");
  }
  
  chThdSleepMilliseconds(1000);
}




static void error_cb(DMADriver *_dmap, dmaerrormask_t err)
{
  (void) _dmap;
  last_err = err;
}

static void end_cb(DMADriver *_dmap, void *buffer, size_t num)
{
  (void) _dmap;
  (void) num;
  (void) buffer;
  
  cb_count++;
}
