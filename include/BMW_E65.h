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
   void DecodeCAN(int id, const uint8_t bytes[8]) override;
   bool Ready() { return terminal15On; }
   bool Start() { return terminal15On; }
   void DashOff();
   void handle130(const uint8_t bytes[8]);
   void handle1A0(const uint8_t bytes[8]);
   void handle2FC(const uint8_t bytes[8]);
   void handle480(const uint8_t bytes[8]);
   void SetE90(bool e90) { isE90 = e90; }
   void Engine_Data();
   void SetFuelGauge(float level);

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

