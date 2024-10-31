#ifndef SimpleISA_h
#define SimpleISA_h

/*  This library supports the ISA Scale IVT Modular current/voltage sensor device.  These devices measure current, up to three voltages, and provide temperature compensation.
    This library was written by Jack Rickard of EVtv - http://www.evtv.me copyright 2016
    You are licensed to use this library for any purpose, commercial or private,
    without restriction.

*/

#include <stdint.h>
#include "my_fp.h"
#include "canhardware.h"

class ISA
{

    ISA();
    ~ISA();



public:
    static void RegisterCanMessages(CanHardware* can);
    static void initialize(CanHardware* can);
    static void initCurrent(CanHardware* can);
    static void sendSTORE(CanHardware* can);
    static void STOP(CanHardware* can);
    static void START(CanHardware* can);
    static void RESTART(CanHardware* can);
    static void deFAULT(CanHardware* can);
    static void DecodeCAN(int id, const uint8_t bytes[8]);

    static int32_t Voltage;
    static int32_t Voltage2;
    static int32_t Voltage3;
    static int32_t Temperature;
    static int32_t Amperes;   // Floating point with current in Amperes
    static int32_t KW;
    static int32_t KWh;
    static int32_t Ah;


private:
    static void handle521(const uint8_t bytes[8]);
    static void handle522(const uint8_t bytes[8]);
    static void handle523(const uint8_t bytes[8]);
    static void handle524(const uint8_t bytes[8]);
    static void handle525(const uint8_t bytes[8]);
    static void handle526(const uint8_t bytes[8]);
    static void handle527(const uint8_t bytes[8]);
    static void handle528(const uint8_t bytes[8]);
};

#endif /* SimpleISA_h */
