#ifndef NOHEATER_H
#define NOHEATER_H

#include <heater.h>


class noHeater: public Heater
{

public:
      void SetTargetTemperature(float temp) { (void)temp; } //Not supported (yet)?
      void SetPower(float power){};

private:

};

#endif // NOHEATER_H
