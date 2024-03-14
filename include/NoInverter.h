#ifndef NoInverter_h
#define NoInverter_h

/*  Dummy library for when no Inverter interface is used.

*/
#include <inverter.h>

class NoInverterClass: public Inverter
{

public:
   void SetTorque(float torquePercent) {torquePercent = torquePercent;}
   float GetMotorTemperature(){return(0);}
   float GetInverterTemperature() {return(0);}
   float GetInverterVoltage() {return(0);}
   float GetMotorSpeed(){return(0);}
   int GetInverterState() {return(0);}
private:

};

#endif
