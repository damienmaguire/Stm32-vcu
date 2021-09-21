#ifndef charger_h
#define charger_h

/*  This library supports the various OEM chargers. eg Chevy volt.

*/

#include <stdint.h>
#include "my_fp.h"
#include "params.h"
#include "stm32_can.h"

class chargerClass
{

public:

static void handle108(uint32_t data[2]);
static bool HVreq;
static void Send100msMessages(bool ChRun);
private:

};

#endif /* charger_h */
