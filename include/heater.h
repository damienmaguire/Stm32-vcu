#ifndef HEATER_H
#define HEATER_H


#include <stdint.h>
#include "my_math.h"
#include "my_fp.h"
#include "CANSPI.h"
#include "digio.h"
#include "utils.h"

class AmperaHeater
{
   public:
    static void sendWakeup();
    static void controlPower(uint16_t heatPwr);
    protected:

    private:


};

class VWHeater
{
   public:


    protected:

    private:


};
#endif // HEATER_H
