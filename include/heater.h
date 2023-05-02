#ifndef HEATER_H
#define HEATER_H


#include <stdint.h>
#include "my_math.h"
#include "my_fp.h"
#include "CANSPI.h"
#include "digio.h"
#include "utils.h"

class Heater
{
public:
   virtual void DecodeCAN(int, uint32_t*) {};
   virtual float GetTemperature() { return 0; }
   virtual void SetTargetTemperature(float temp) = 0; //target temperature in °C
   virtual void SetPower(uint16_t power, bool HeatReq) = 0; //Must be called cyclically with power in watts
   virtual void DeInit() {} //called when switching to another heater, similar to a destructor
   virtual void SetCanInterface(CanHardware* c) { can = c; }

protected:
   CanHardware* can;
};


#endif // HEATER_H
