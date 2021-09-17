#include <stdint.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include "stm32_can.h"
#include "terminal.h"
#include "params.h"
#include "hwdefs.h"
#include "digio.h"
#include "hwinit.h"
#include "anain.h"
#include "temp_meas.h"
#include "param_save.h"
#include "my_math.h"
#include "errormessage.h"
#include "printf.h"
#include "stm32scheduler.h"
#include "leafinv.h"
#include "isa_shunt.h"
#include "Can_E39.h"
#include "Can_E46.h"
#include "BMW_E65.h"
#include "Can_OI.h"
#include "Can_VAG.h"
#include "GS450H.h"
#include "throttle.h"
#include "utils.h"
#include "charger.h"
#include "i3LIM.h"
#include "CANSPI.h"
#include "chademo.h"
#include "heater.h"

/*
typedef union {
    struct {
        uint8_t idType;
        uint32_t id;
        uint8_t dlc;
        uint8_t data0;
        uint8_t data1;
        uint8_t data2;
        uint8_t data3;
        uint8_t data4;
        uint8_t data5;
        uint8_t data6;
        uint8_t data7;
    } frame;
    uint8_t array[14];
} CAN3_Msg;
*/

typedef union {
    struct {
        uint32_t data[2];
        uint8_t datab[8];

    } frame;
    //uint8_t array[14];
} CAN3_Msg;
