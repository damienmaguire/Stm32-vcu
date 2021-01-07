#ifndef Can_E39_h
#define Can_E39_h

/*  This library supports the Powertrain CAN messages for the BMW E39 for driving dash gauges, putting out malf lights etc

*/

#include <stdint.h>
#include "my_fp.h"
#include "stm32_can.h"

class Can_E39
{

public:
    static		void SendE39(uint16_t outRPM , uint16_t tempValue);


private:









};

#endif /* Can_E39_h */

