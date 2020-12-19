#ifndef FOC_H
#define FOC_H

#include <stdint.h>
#include "my_fp.h"

class FOC
{
   public:
      static void ParkClarke(s32fp il1, s32fp il2, uint16_t angle);
      static int32_t GetQLimit(int32_t maxVd);
      static int32_t GetTotalVoltage(int32_t ud, int32_t uq);
      static void InvParkClarke(int32_t ud, int32_t uq, uint16_t angle);
      static void Mtpa(int32_t is, int32_t& idref, int32_t& iqref);
      static int32_t GetMaximumModulationIndex();
      static s32fp id;
      static s32fp iq;
      static int32_t DutyCycles[3];

   protected:
   private:
      static uint32_t sqrt(uint32_t rad);
      static u32fp fpsqrt(u32fp rad);
};

#endif // FOC_H
