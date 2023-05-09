#ifndef vag_sbox_h
#define vag_sbox_h

/*  VW PHEV Battery contactor box control routines.
    See : https://github.com/jamiejones85/VW-GTE-ContactorBox/tree/main
*/

#include <stdint.h>
#include "my_fp.h"
#include "canhardware.h"
#include "my_math.h"
#include "stm32_can.h"
#include "params.h"

class VWBOX
{

    VWBOX();
    ~VWBOX();



public:
    static void RegisterCanMessages(CanHardware* can);
    static void DecodeCAN(int id, uint32_t data[2]);
    static void ControlContactors(int opmode, CanHardware* can);

    static float Voltage;
    static float Voltage2;
    static int32_t Temperature;
    static int16_t Amperes;   // Floating point with current in Amperes
    static int32_t KW;
    static int32_t KWh;
    static int32_t Ah;


private:


    static void handle0BB(uint32_t data[2]);
    static uint8_t vw_crc_calc(uint8_t *data);

};

#endif /* vag_sbox_h */



