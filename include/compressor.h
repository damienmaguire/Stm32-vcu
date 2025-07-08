#ifndef COMPRESSOR_H
#define COMPRESSOR_H


#include <stdint.h>
#include "my_math.h"
#include "my_fp.h"
#include "CANSPI.h"
#include "digio.h"
#include "utils.h"

class Compressor
{
public:
   virtual void DecodeCAN(int, uint32_t*) {};
   virtual void DeInit() {} //called when switching to another heater, similar to a destructor
   virtual void SetCanInterface(CanHardware* c) { can = c; }
   virtual void Task100Ms() {};
   virtual void Task10Ms() {};

protected:
   CanHardware* can;
};


#endif // COMPRESSOR_H
