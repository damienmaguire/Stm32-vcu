#ifndef bmw_sbox_h
#define bmw_sbox_h

/*  BMW PHEV Battery "SBOX" control routines.
    See : https://github.com/damienmaguire/BMW_SBox
*/

#include <stdint.h>
#include "my_fp.h"
#include "canhardware.h"
#include "my_math.h"
#include "stm32_can.h"
#include "params.h"

class SBOX
{

    SBOX();
    ~SBOX();



public:
    static void RegisterCanMessages(CanHardware* can);
    static void DecodeCAN(int id, const uint8_t bytes[8]);
    static void ControlContactors(int opmode, CanHardware* can);

    static int32_t Voltage;
    static int32_t Voltage2;
    static int32_t Temperature;
    static int32_t Amperes;   // Floating point with current in Amperes
    static int32_t KW;
    static int32_t KWh;
    static int32_t Ah;


private:


    static void handle200(const uint8_t bytes[8]);
    static void handle210(const uint8_t bytes[8]);
    static void handle220(const uint8_t bytes[8]);

};

#endif /* bmw_sbox_h */


