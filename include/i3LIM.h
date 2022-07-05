#ifndef i3LIM_h
#define i3LIM_h

/*  This library supports the BMW i3 LIM charge interface module.

*/

#include <stdint.h>
#include "my_fp.h"
#include "params.h"
#include "stm32_can.h"

enum class i3LIMChargingState
{
    No_Chg,
    AC_Chg,
    DC_Chg
};

class i3LIMClass
{

public:

static void handle3B4(uint32_t data[2]);
static void handle29E(uint32_t data[2]);
static void handle2B2(uint32_t data[2]);
static void handle2EF(uint32_t data[2]);
static void handle272(uint32_t data[2]);
static void Send200msMessages(CanHardware* can);
static void Send100msMessages(CanHardware* can);
static void Send10msMessages(CanHardware* can);
static i3LIMChargingState Control_Charge(bool RunCh);
private:
static void CCS_Pwr_Con();
static void Chg_Timers();
};

#endif /* i3LIM_h */
