#ifndef Focci_h
#define Focci_h

/*  This library supports the Focci
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

class FocciClass: public Chargerint
{

public:
      void SetCanInterface(CanHardware* c);
      void DecodeCAN(int id, const uint8_t bytes[8]) override;
      void Task10Ms();
      void Task100Ms();
      void Task200Ms();
      void ConfigCan();
      bool DCFCRequest(bool RunCh);
      bool ACRequest(bool RunCh);
      void CCS_Pwr_Con();

private:
static void handle357(const uint8_t bytes[8]);
static void handle109(const uint8_t bytes[8]);
static void handle596(const uint8_t bytes[8]);
static void Chg_Timers();
};

#endif /* CPC_h */
