#include <ch.h>
#include <hal.h>
#include <stdnoreturn.h>
#include "globalVar.h"
#include "stdutil.h"

#define ADC_GRP1_NUM_CHANNELS   1
#define ADC_GRP1_BUF_DEPTH      1
static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 1 samples of 2 channels, SW triggered.
 * Channels:    IN4
 */
static const ADCConversionGroup adcgrpcfg1 = {
  .circular	= TRUE,
  .num_channels = ADC_GRP1_NUM_CHANNELS,
  .end_cb	= NULL, // adc complete callback
  .error_cb	= NULL, // adc error callback
  .cr1		= 0,                        /* CR1 */
  .cr2		= ADC_CR2_SWSTART,          /* CR2 */
  .smpr1	= 0,
  .smpr2	= ADC_SMPR2_SMP_AN4(ADC_SAMPLE_480),
  .sqr1		= ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),  /* SQR1 */
  .sqr2		= 0,					   /* SQR2 */
  .sqr3		= ADC_SQR3_SQ1_N(ADC_CHANNEL_IN4) 	   /* SQR3 */
};




void initPotentiometre(void)
{
  adcStart(&ADCD1, NULL);
  adcStartConversion (&ADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);
}


float getPotValue (void)
{
  return (samples1[0] / 4095.0f);
}
