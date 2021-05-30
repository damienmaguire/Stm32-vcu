#ifndef i3LIM_h
#define i3LIM_h

/*  This library supports the BMW i3 LIM charge interface module.

*/

#include <stdint.h>
#include "my_fp.h"
#include "params.h"
#include "stm32_can.h"
#include "digio.h"

class i3LIMClass
{

public:

static void handle3B4(uint32_t data[2]);
static void Send200msMessages();
static void Send100msMessages();
static void Send10msMessages();
private:

};

#endif /* i3LIM_h */
