#ifndef Can_E46_h
#define Can_E46_h

/*  This library supports the Powertrain CAN messages for the BMW E46 for driving dash gauges, putting out malf lights etc

*/

#include <stdint.h>
#include "my_fp.h"

class Can_E46
{

public:
    static		void Msg316();
    static		void Msg329();
    static		void Msg545();

private:









};

#endif /* Can_E46_h */
