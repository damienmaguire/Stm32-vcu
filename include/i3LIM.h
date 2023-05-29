#ifndef i3LIM_h
#define i3LIM_h

/*  This library supports the BMW i3 LIM charge interface module.

*/

#include <stdint.h>
#include "my_fp.h"
#include "params.h"
#include "stm32_can.h"
#include "chargerint.h"
#include "my_math.h"

class i3LIMClass: public Chargerint
{

public:
      void SetCanInterface(CanHardware* c);
      void DecodeCAN(int id, uint32_t data[2]);
      void Task10Ms();
      void Task100Ms();
      void Task200Ms();
      bool DCFCRequest(bool RunCh);
      bool ACRequest(bool RunCh);

private:
static void handle3B4(uint32_t data[2]);
static void handle29E(uint32_t data[2]);
static void handle2B2(uint32_t data[2]);
static void handle2EF(uint32_t data[2]);
static void handle272(uint32_t data[2]);
static void CCS_Pwr_Con();
static void Chg_Timers();
};

#endif /* i3LIM_h */
