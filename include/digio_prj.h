#ifndef PinMode_PRJ_H_INCLUDED
#define PinMode_PRJ_H_INCLUDED

#include "hwdefs.h"

#define DIG_IO_LIST \
    DIG_IO_ENTRY(cruise_in,   GPIOB, GPIO9,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(start_in,    GPIOB, GPIO4,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(brake_in,    GPIOC, GPIO1,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(fwd_in,      GPIOB, GPIO5,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(rev_in,      GPIOB, GPIO4,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(bms_in,      GPIOC, GPIO3,  PinMode::INPUT_PD)   \
    DIG_IO_ENTRY(dcsw_out,    GPIOC, GPIO7, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(led_out,     GPIOC, GPIO13, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(err_out,     GPIOB, GPIO10, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(brk_out,     GPIOC, GPIO5,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(prec_out,     GPIOC, GPIO8,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(inv_out,     GPIOC, GPIO9,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(SL1_out,     GPIOC, GPIO6,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(SL2_out,     GPIOB, GPIO15,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(SP_out,     GPIOB, GPIO14,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(gear1_in,   GPIOC, GPIO12,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(gear2_in,   GPIOD, GPIO2,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(gear3_in,   GPIOB, GPIO3,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(req_out,     GPIOA, GPIO4,  PinMode::OUTPUT)      \



#endif // PinMode_PRJ_H_INCLUDED
