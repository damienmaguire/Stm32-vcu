#ifndef ANAIN_PRJ_H_INCLUDED
#define ANAIN_PRJ_H_INCLUDED

#include "hwdefs.h"

#define NUM_SAMPLES 12
#define SAMPLE_TIME ADC_SMPR_SMP_7DOT5CYC

#define ANA_IN_LIST \
   ANA_IN_ENTRY(throttle1, GPIOA, 5) \
   ANA_IN_ENTRY(throttle2, GPIOA, 6) \
   ANA_IN_ENTRY(uaux,      GPIOC, 2) \
   ANA_IN_ENTRY(MG1_Temp,  GPIOB, 1) \
   ANA_IN_ENTRY(MG2_Temp,  GPIOC, 0) \

#endif // ANAIN_PRJ_H_INCLUDED
