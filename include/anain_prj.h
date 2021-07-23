#ifndef ANAIN_PRJ_H_INCLUDED
#define ANAIN_PRJ_H_INCLUDED

#include "hwdefs.h"

#define NUM_SAMPLES 12
#define SAMPLE_TIME ADC_SMPR_SMP_7DOT5CYC

#define ANA_IN_LIST \
    ANA_IN_ENTRY(throttle1, GPIOC, 0) \
   ANA_IN_ENTRY(throttle2, GPIOC, 1) \
   ANA_IN_ENTRY(uaux,      GPIOB, 1) \
   ANA_IN_ENTRY(GP_analog1,GPIOC, 2) \
   ANA_IN_ENTRY(GP_analog2,GPIOC, 3) \
   ANA_IN_ENTRY(MG1_Temp,  GPIOC, 4) \
   ANA_IN_ENTRY(MG2_Temp,  GPIOC, 5) \

#endif // ANAIN_PRJ_H_INCLUDED
