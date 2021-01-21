#ifndef BMW_E65_h
#define BMW_E65_h

/*  This library supports the Powertrain CAN messages for the BMW E65 for driving dash gauges, putting out malf lights etc
    Also reads gear lever, brake lights etc

*/

#include <stdint.h>
#include "my_fp.h"

class BMW_E65Class
{

public:
    static		void Tacho(int16_t speed);
    static		void absdsc(bool Brake_In);
    static		void GDis();
    void Cas(int id, uint32_t data[2], uint32_t time);
    void DashOn();
    void DashOff();
    void Gear(int id, uint32_t data[2], uint32_t time);
    uint8_t getGear();
    void setGear();
    bool getTerminal15();
    void setTerminal15(bool);
    BMW_E65Class()
    {
        Terminal15On = false;
        dashInit = false;
        gear = 0;
    }

private:
    bool Terminal15On;
    bool dashInit;
    uint8_t gear;
};

#endif /* BMW_E65_h */

