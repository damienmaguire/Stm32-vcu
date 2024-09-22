#ifndef BMW_E65_h
#define BMW_E65_h

/*  This library supports the Powertrain CAN messages for the BMW E65 for driving dash gauges, putting out malf lights etc
    Also reads gear lever, brake lights etc

*/

#include <stdint.h>
#include "vehicle.h"
#include "my_math.h"

class BMW_E65: public Vehicle
{
public:
   BMW_E65() : terminal15On(false), dashInit(false), gear(PARK) { }
   void SetCanInterface(CanHardware*);
   void Task10Ms();
   void Task100Ms();
   void Task200Ms();
   void SetRevCounter(int speed) { revCounter = speed; }
   void SetTemperatureGauge(float temp) { temperature = temp; }
   void DecodeCAN(int, uint32_t* data);
   bool Ready() { return terminal15On; }
   bool Start() { return terminal15On; }
   bool GetGear(Vehicle::gear& outGear);
   void DashOff();
   void handle130(uint32_t data[2]);
   void handle192(uint32_t data[2]);
   void handle2FC(uint32_t data[2]);
   void handle480(uint32_t data[2]);
   void SetE90(bool e90) { isE90 = e90; }
   void Engine_Data();

private:
   void SendAbsDscMessages(bool Brake_In);

   bool terminal15On;
   bool terminalROn;
   bool terminal50On;
   bool dashInit;
   Vehicle::gear gear;
   int revCounter;
   float temperature;
   bool  CANWake;
   bool  StartButt;
   bool isE90;
};

#endif /* BMW_E65_h */

