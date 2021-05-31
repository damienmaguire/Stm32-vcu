#ifndef i3LIM_h
#define i3LIM_h

/*  This library supports the BMW i3 LIM charge interface module.

*/

#include <stdint.h>
#include "my_fp.h"
#include "params.h"
#include "stm32_can.h"

class i3LIMClass
{

public:

static void handle3B4(uint32_t data[2]);
static void handle29E(uint32_t data[2]);
static void handle2B2(uint32_t data[2]);
static void handle2EF(uint32_t data[2]);
static void handle272(uint32_t data[2]);
static void Send200msMessages();
static void Send100msMessages();
static void Send10msMessages();
static uint8_t Control_Charge();
private:

};

#endif /* i3LIM_h */
