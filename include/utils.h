#ifndef UTILS_H
#define UTILS_H

#include "BMW_E65.h"
#include "my_fp.h"
#include "my_math.h"
#include "errormessage.h"
#include "params.h"
#include "digio.h"
#include <libopencm3/stm32/rtc.h>
#include "canhardware.h"
#include "anain.h"
#include "throttle.h"
#include "isa_shunt.h"
#include "bmw_sbox.h"
#include "vag_sbox.h"
#include "vehicle.h"
#include "shifter.h"
#include <libopencm3/stm32/timer.h>
#include "iomatrix.h"

namespace utils
{
    int32_t change(int32_t, int32_t, int32_t, int32_t, int32_t);
    float GetUserThrottleCommand(CanHardware*);
    float ProcessThrottle(int);
    float ProcessUdc(int);
    void CalcSOC();
    void GetDigInputs(CanHardware*);
    void PostErrorIfRunning(ERROR_MESSAGE_NUM);
    void SelectDirection(Vehicle* , Shifter*);
    void displayThrottle();
    void ProcessCruiseControlButtons();
    void CpSpoofOutput();
}

#endif
