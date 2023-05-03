#ifndef NOHEATER_H
#define NOHEATER_H

#include <heater.h>


class noHeater: public Heater
{

public:
      void SetTargetTemperature(float temp) { (void)temp; } //Not supported (yet)?
      void SetPower(uint16_t, bool){};

private:

};

#endif // NOHEATER_H
