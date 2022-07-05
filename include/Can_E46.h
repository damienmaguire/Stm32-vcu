#ifndef Can_E46_h
#define Can_E46_h

/*  This library supports the Powertrain CAN messages for the BMW E46 for driving dash gauges, putting out malf lights etc

*/

#include <stdint.h>
#include "my_fp.h"
#include "canhardware.h"

class Can_E46
{

public:
    static		void Msg316(uint16_t outRPM, CanHardware* can);
    static		void Msg329(uint16_t tempValue, CanHardware* can);
    static		void Msg43F(int8_t gear, CanHardware* can);
    static		void Msg545(CanHardware* can);

private:

};

#endif /* Can_E46_h */
