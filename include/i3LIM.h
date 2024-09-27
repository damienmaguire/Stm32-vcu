#ifndef i3LIM_h
#define i3LIM_h

/*  This library supports the BMW i3 LIM charge interface module.

*/

#include "chargerint.h"
#include "canhardware.h"
#include <stdint.h>

class i3LIMClass: public Chargerint
{
public:
      void SetCanInterface(CanHardware* c) override;
      void DecodeCAN(int id, uint32_t* data) override;
      void Task10Ms() override;
      void Task100Ms() override;
      void Task200Ms() override;
      bool DCFCRequest(bool RunCh) override;
      bool ACRequest(bool RunCh) override;

private:
      void handle3B4(uint32_t data[2]);
      void handle29E(uint32_t data[2]);
      void handle2B2(uint32_t data[2]);
      void handle2EF(uint32_t data[2]);
      void handle272(uint32_t data[2]);
      void CCS_Pwr_Con();
      void Chg_Timers();
};

#endif /* i3LIM_h */
