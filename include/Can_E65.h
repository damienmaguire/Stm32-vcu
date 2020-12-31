#ifndef Can_E65_h
#define Can_E65_h

/*  This library supports the Powertrain CAN messages for the BMW E65 for driving dash gauges, putting out malf lights etc
    Also reads gear lever, brake lights etc

*/

#include <stdint.h>
#include "my_fp.h"

class Can_E65
{

public:
    static		bool Cas(int id, uint32_t data[2], uint32_t time);
    static		void Gear(int id, uint32_t data[2], uint32_t time);
    static		void Tacho(uint16_t speed);
    static		void absdsc(bool Brake_In);
    static		void DashOn();
    static		void GDis();
    static      uint8_t Gear_E65();


private:









};

#endif /* Can_E65_h */

