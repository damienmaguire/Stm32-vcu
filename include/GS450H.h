#ifndef GS450H_h
#define GS450H_h

/*  Communication with Lexus GS450H and potentiall any Toyota Hybrid inverter using sync serial

*/

#include <stdint.h>
#include "my_fp.h"
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include "digio.h"
#include "params.h"
#include "inverter.h"

#define MG2MAXSPEED 10000
#define MAX_COMMAND_SIZE 200

class GS450HClass: public Inverter
{

public:
   GS450HClass()
   {
       scaledTorqueTarget = 0;
       timerIsRunning = false;
   }
   void Task1Ms();
   void Task100Ms();
   void SetTorque(float torquePercent);
   float GetMotorTemperature();
   float GetInverterTemperature() { return temp_inv_water; }
   float GetInverterVoltage() { return dc_bus_voltage; }
   float GetMotorSpeed() { return mg2_speed; }
   int GetInverterState();
   void DeInit() { setTimerState(false); } //called when switching to another inverter, similar to a destructor


   //Lexus/Toyota specific functions
   void SetPrius();
   void SetGS450H();
   void SetGS300H();
   void SetGear(int16_t g) { gear = g; }
   void SetOil(int16_t o) { oil = o; }

private:
   int16_t dc_bus_voltage, mg1_speed, mg2_speed, gear, oil;
   float temp_inv_water, temp_inv_inductor;
   bool timerIsRunning;
   int scaledTorqueTarget;
   uint8_t VerifyMTHChecksum(uint16_t );
   void CalcHTMChecksum(uint16_t);
   void setTimerState(bool);
   void GS450Hgear();
};

#endif /* GS450H_h */
