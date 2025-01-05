#ifndef Foccci_h
#define Foccci_h

/*  This library supports the Foccci
https://github.com/uhi22/foccci
https://github.com/uhi22/ccs32clara
	2024 - Tom de Bree
*/

#include <stdint.h>
#include "my_fp.h"
#include "params.h"
#include "stm32_can.h"
#include "chargerint.h"
#include "my_math.h"

#define NODEID 22

class FoccciClass: public Chargerint
{

public:
      void SetCanInterface(CanHardware* c);
      void DecodeCAN(int id, uint32_t data[2]);
      void Task10Ms();
      void Task100Ms();
      void Task200Ms();
      void ConfigCan();
      bool DCFCRequest(bool RunCh);
      bool ACRequest(bool RunCh);
      void CCS_Pwr_Con();

private:
static void handle357(uint32_t data[2]);
static void handle109(uint32_t data[2]);
static void handle596(uint32_t data[2]);
static void Chg_Timers();
};

#endif /* CPC_h */
