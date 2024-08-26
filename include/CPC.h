#ifndef CPC_h
#define CPC_h

/*  This library supports the Charge Port Controller CAN interface
	2024 - Tom de Bree
*/

#include <stdint.h>
#include "my_fp.h"
#include "params.h"
#include "stm32_can.h"
#include "chargerint.h"
#include "my_math.h"

class CPCClass: public Chargerint
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
static void handle357(uint32_t data[2]);

static void Chg_Timers();
};

#endif /* CPC_h */
