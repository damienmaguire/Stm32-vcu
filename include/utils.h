#ifndef UTILS_H
#define UTILS_H

#include "BMW_E65.h"
#include "my_fp.h"
#include "my_math.h"
#include "errormessage.h"
#include "params.h"
#include "digio.h"
#include <libopencm3/stm32/rtc.h>
#include "stm32_can.h"
#include "anain.h"
#include "throttle.h"
#include "isa_shunt.h"

namespace utils
{
    int32_t change(int32_t, int32_t, int32_t, int32_t, int32_t);
    int GetUserThrottleCommand(Can*);
    s32fp ProcessThrottle(int, Can*);
    s32fp ProcessUdc(uint32_t, int);
    void GetDigInputs(Can*);
    void PostErrorIfRunning(ERROR_MESSAGE_NUM);
    void SelectDirection(_vehmodes, BMW_E65Class);
    void displayThrottle();
}

#endif
