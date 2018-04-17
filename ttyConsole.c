#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "ch.h"
#include "hal.h"
#include "microrl/microrlShell.h"
#include "ttyConsole.h"
#include "stdutil.h"
#include "globalVar.h"
#include "rtcAccess.h"
#include "printf.h"
#include "portage.h"


/*===========================================================================*/
/* START OF EDITABLE SECTION                                           */
/*===========================================================================*/


static void cmd_mem(BaseSequentialStream *lchp, int argc,const char * const argv[]);
static void cmd_threads(BaseSequentialStream *lchp, int argc,const char * const argv[]);
static void cmd_rtc(BaseSequentialStream *lchp, int argc,const char * const argv[]);
static void cmd_uid(BaseSequentialStream *lchp, int argc,const char * const argv[]);

static const ShellCommand commands[] = {
  {"mem", cmd_mem},
  {"threads", cmd_threads},
  {"rtc", cmd_rtc},
  {"uid", cmd_uid},
  {NULL, NULL}
};


/*===========================================================================*/
/* START OF PRIVATE SECTION  : DO NOT CHANGE ANYTHING BELOW THIS LINE        */
/*===========================================================================*/

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/


#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(4096)

#if CONSOLE_DEV_USB == 0
static const SerialConfig ftdiConfig =  {
  115200,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};
#endif


#define MAX_CPU_INFO_ENTRIES 20
typedef struct _ThreadCpuInfo {
  float    ticks[MAX_CPU_INFO_ENTRIES];
  float    cpu[MAX_CPU_INFO_ENTRIES];
  float    totalTicks;
} ThreadCpuInfo ;

static void stampThreadCpuInfo (ThreadCpuInfo *ti);
static float stampThreadGetCpuPercent (const ThreadCpuInfo *ti, const uint32_t idx);

static void cmd_uid(BaseSequentialStream *lchp, int argc,const char * const argv[]) {
  (void)argv;
  if (argc > 0) {
    chprintf (lchp, "Usage: uid\r\n");
    return;
  }

  /* for (uint32_t i=0; i<2000; i++) { */
  /*   bkpram[i] = (uint16_t) i; */
  /* } */
  
  /* for (uint32_t i=0; i<2000; i++) { */
  /*   if (bkpram[i] != (uint16_t) i) { */
  /*     DebugTrace ("bkpram error"); */
  /*   } */
  /* } */

  chprintf (lchp, "uniq id : ");
  for (uint32_t i=0; i< UniqProcessorIdLen; i++)
    chprintf (lchp, "[%x] ", UniqProcessorId[i]);
  chprintf (lchp, "\r\n");
}

static void cmd_rtc(BaseSequentialStream *lchp, int argc,const char * const argv[])
{
  if ((argc != 0) && (argc != 2) && (argc != 6)) {
     DebugTrace ("Usage: rtc [Hour Minute Second Year monTh Day day_of_Week Adjust] value or");
     DebugTrace ("Usage: rtc  Hour Minute Second Year monTh Day");
     return;
  }
 
  if (argc == 2) {
    const char timeVar = (char) tolower ((int) *(argv[0]));
    const int32_t varVal = atoi (argv[1]);
    
    switch (timeVar) {
    case 'h':
      setHour ((uint32_t)(varVal));
      break;
      
    case 'm':
       setMinute ((uint32_t)(varVal));
      break;
      
    case 's':
      setSecond ((uint32_t)(varVal));
      break;
      
    case 'y':
       setYear ((uint32_t)(varVal));
      break;
      
    case 't':
       setMonth ((uint32_t)(varVal));
      break;
      
    case 'd':
       setMonthDay ((uint32_t)(varVal));
      break;

    case 'w':
       setWeekDay ((uint32_t)(varVal));
      break;

    case 'a':
      {
	int32_t newSec =(int)(getSecond()) + varVal;
	if (newSec > 59) {
	  int32_t newMin =(int)(getMinute()) + (newSec/60);
	  if (newMin > 59) {
	    setHour ((getHour()+((uint32_t)(newMin/60))) % 24);
	    newMin %= 60;
	  }
	  setMinute ((uint32_t)newMin);
	}
	if (newSec < 0) {
	  int32_t newMin =(int)getMinute() + (newSec/60)-1;
	  if (newMin < 0) {
	    setHour ((getHour()-((uint32_t)newMin/60)-1) % 24);
	    newMin %= 60;
	  }
	  setMinute ((uint32_t)newMin);
	}
	setSecond ((uint32_t)newSec % 60);
      }
      break;
      
    default:
      DebugTrace ("Usage: rtc [Hour Minute Second Year monTh Day Weekday Adjust] value");
    }
  } else if (argc == 6) {
    setYear ((uint32_t) atoi(argv[3]));
    setMonth ((uint32_t) atoi(argv[4]));
    setMonthDay ((uint32_t) atoi(argv[5]));
    setHour ((uint32_t) atoi(argv[0]));
    setMinute ((uint32_t) atoi(argv[1]));
    setSecond ((uint32_t) atoi(argv[2]));
  }

  chprintf (lchp, "RTC : %s %.02lu/%.02lu/%.04lu  %.02lu:%.02lu:%.02lu\r\n",
	    getWeekDayAscii(), getMonthDay(), getMonth(), getYear(),
	    getHour(), getMinute(), getSecond());
}


static void cmd_mem(BaseSequentialStream *lchp, int argc,const char * const argv[]) {
  (void)argv;
  if (argc > 0) {
    chprintf (lchp, "Usage: mem\r\n");
    return;
  }

  chprintf (lchp, "core free memory : %u bytes\r\n", chCoreStatus());
  chprintf (lchp, "heap free memory : %u bytes\r\n", getHeapFree());

  void * ptr1 = malloc_m (100);
  void * ptr2 = malloc_m (100);

  chprintf (lchp, "(2x) malloc_m(1000) = %p ;; %p\r\n", ptr1, ptr2);
  chprintf (lchp, "heap free memory : %d bytes\r\n", getHeapFree());

  free_m (ptr1);
  free_m (ptr2);
}




static void cmd_threads(BaseSequentialStream *lchp, int argc,const char * const argv[]) {
  static const char *states[] = {THD_STATE_NAMES};
  Thread *tp = chRegFirstThread();
  (void)argv;
  (void)argc;
  float totalTicks=0;
  float idleTicks=0;

  static ThreadCpuInfo threadCpuInfo = {
    .ticks = {[0 ... MAX_CPU_INFO_ENTRIES-1] = 0.f}, 
    .cpu =   {[0 ... MAX_CPU_INFO_ENTRIES-1] =-1.f}, 
    .totalTicks = 0.f
  };
  
  stampThreadCpuInfo (&threadCpuInfo);
  
  chprintf (lchp, "    addr    stack  frestk prio refs  state        time \t percent        name\r\n");
  uint32_t idx=0;
  do {
    chprintf (lchp, "%.8lx %.8lx %6lu %4lu %4lu %9s %9lu   %.1f    \t%s\r\n",
	      (uint32_t)tp, (uint32_t)tp->ctx.sp,
	      get_stack_free (tp),
	      (uint32_t)tp->prio, (uint32_t)(tp->refs - 1),
	      states[tp->state], (uint32_t)tp->time, 
	      stampThreadGetCpuPercent (&threadCpuInfo, idx),
	      chRegGetThreadName(tp));
    totalTicks+= (float) tp->time;
    if (strcmp (chRegGetThreadName(tp), "idle") == 0)
      idleTicks =  (float) tp->time;
    tp = chRegNextThread ((Thread *)tp);
    idx++;
  } while (tp != NULL);

  const float idlePercent = (idleTicks*100.f)/totalTicks;
  const float cpuPercent = 100.f - idlePercent;
  chprintf (lchp, "\r\ncpu load = %.2f%%\r\n", cpuPercent);
}


static const ShellConfig shell_cfg1 = {
#if CONSOLE_DEV_USB == 0
  (BaseSequentialStream *) &CONSOLE_DEV_SD,
#else
  (BaseSequentialStream *) &SDU1,
#endif
  commands
};



void consoleInit (void)
{
  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * USBD1 : FS, USBD2 : HS
   */

#if CONSOLE_DEV_USB != 0
  usbSerialInit(&SDU1, &USBDRIVER); 
  chp = (BaseSequentialStream *) &SDU1;
#else
  sdStart(&CONSOLE_DEV_SD, &ftdiConfig);
  chp = (BaseSequentialStream *) &CONSOLE_DEV_SD;
#endif
  /*
   * Shell manager initialization.
   */
  shellInit();
}


void consoleLaunch (void)
{
  Thread *shelltp = NULL;

  if (!shelltp) {
    //       palSetPad (BOARD_LED3_P, BOARD_LED3);
    shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
  } else if (chThdTerminated(shelltp)) {
    chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
    shelltp = NULL;           /* Triggers spawning of a new shell.        */
    //       palClearPad (BOARD_LED3_P, BOARD_LED3);
  }
  chThdSleepMilliseconds(100);
}

static void stampThreadCpuInfo (ThreadCpuInfo *ti)
{
  const Thread *tp =  chRegFirstThread();
  uint32_t idx=0;
  
  float totalTicks =0;
  do {
    totalTicks+= (float) tp->time;
    ti->cpu[idx] = (float) tp->time - ti->ticks[idx];;
    ti->ticks[idx] = (float) tp->time;
    tp = chRegNextThread ((Thread *)tp);
    idx++;
  } while ((tp != NULL) && (idx < MAX_CPU_INFO_ENTRIES));
  
  const float diffTotal = totalTicks- ti->totalTicks;
  ti->totalTicks = totalTicks;
  
  tp =  chRegFirstThread();
  idx=0;
  do {
    ti->cpu[idx] =  (ti->cpu[idx]*100.f)/diffTotal;
    tp = chRegNextThread ((Thread *)tp);
    idx++;
  } while ((tp != NULL) && (idx < MAX_CPU_INFO_ENTRIES));
}


static float stampThreadGetCpuPercent (const ThreadCpuInfo *ti, const uint32_t idx)
{
  if (idx >= MAX_CPU_INFO_ENTRIES) 
    return -1.f;

  return ti->cpu[idx];
}
