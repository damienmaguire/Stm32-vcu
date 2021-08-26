/*
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *                      Damien Maguire <info@evbmw.com>
 * Yes I'm really writing software now........run.....run away.......
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "leafinv.h"
#include "my_fp.h"
#include "my_math.h"
#include "stm32_can.h"
#include "params.h"




uint8_t LeafINV::run10ms = 0;
uint8_t LeafINV::run100ms = 0;
uint32_t LeafINV::lastRecv = 0;
uint16_t LeafINV::voltage;
int16_t LeafINV::speed;
bool LeafINV::error=false;
int16_t LeafINV::inv_temp;
int16_t LeafINV::motor_temp;
int16_t LeafINV::final_torque_request;
static uint8_t counter_1db=0;
static uint8_t counter_11a_d6=0;
static uint8_t counter_1d4=0;


/*Info on running Leaf Gen 2 PDM
IDs required :
0x1D4
0x1DB
0x1DC
0x1F2
0x50B
0x55B
0x59E
0x5BC

PDM sends:
0x390
0x393
0x679 on evse plug insert

*/


void LeafINV::DecodeCAN(int id, uint32_t data[2])
{

    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)

    if (id == 0x1DA)// THIS MSG CONTAINS INV VOLTAGE, MOTOR SPEED AND ERROR STATE
    {
        voltage = (bytes[0] << 2) | (bytes[1] >> 6);//MEASURED VOLTAGE FROM LEAF INVERTER

        int16_t parsed_speed = (bytes[4] << 8) | bytes[5];
        speed = (parsed_speed == 0x7fff ? 0 : parsed_speed);//LEAF MOTOR RPM

        error = (bytes[6] & 0xb0) != 0x00;//INVERTER ERROR STATE

    }
    else if (id == 0x55A)// THIS MSG CONTAINS INV TEMP AND MOTOR TEMP
    {
        inv_temp = fahrenheit_to_celsius(bytes[2]);//INVERTER TEMP
        motor_temp = fahrenheit_to_celsius(bytes[1]);//MOTOR TEMP
    }

}



void LeafINV::SetTorque(int8_t gear, int16_t torque)
{
    if(gear==0) final_torque_request=0;//Neutral
    if(gear==32) final_torque_request=torque;//Drive
    if(gear==-32) final_torque_request=torque*-1;;//Reverse

    Param::SetInt(Param::torque,final_torque_request);//post processed final torue value sent to inv to web interface

}





void LeafINV::Send10msMessages()
{


    uint8_t bytes[8];


    // Data taken from a gen1 inFrame where the car is starting to
    // move at about 10% throttle: 4E400055 0000017D

    // All possible gen1 values: 00 01 0D 11 1D 2D 2E 3D 3E 4D 4E
    // MSB nibble: Selected gear (gen1/LeafLogs)
    //   0: some kind of non-gear before driving
    //      0: Park in Gen 2. byte 0 = 0x01 when in park and charging
    //   1: some kind of non-gear after driving
    //   2: R
    //   3: N
    //   4: D
    // LSB nibble: ? (LeafLogs)
    //   0: sometimes at startup, not always; never when the
    //      inverted is powered on (0.06%)
    //   1: this is the usual value (55% of the time in LeafLogs)
    //   D: seems to occur for ~90ms when changing gears (0.2%)
    //   E: this also is a usual value, but never occurs with the
    //      non-gears 0 and 1 (44% of the time in LeafLogs)


//byte 0 determines motor rotation direction
    bytes[0] = 0x4E;//this will need to be pulled from the for/rev pins on the vcu but just leave as fwd for testing

    // 0x40 when car is ON, 0x80 when OFF, 0x50 when ECO. Car must be off when charing 0x80
    bytes[1] = 0x40;

    // Usually 0x00, sometimes 0x80 (LeafLogs), 0x04 seen by canmsgs
    bytes[2] = 0x00;

    // Weird value at D3:4 that goes along with the counter
    // NOTE: Not actually needed, you can just send constant AA C0
    const static uint8_t weird_d34_values[4][2] = {
      {0xaa, 0xc0},
      {0x55, 0x00},
      {0x55, 0x40},
      {0xaa, 0x80},
    };



    bytes[3] = weird_d34_values[counter_11a_d6][0];//0xAA;
    bytes[4] = weird_d34_values[counter_11a_d6][1];//0xC0;

    // Always 0x00 (LeafLogs, canmsgs)
    bytes[5] = 0x00;

    // A 2-bit counter
    bytes[6] = counter_11a_d6;

    counter_11a_d6++;
    if(counter_11a_d6 >= 4)
    {
        counter_11a_d6 = 0;
    }


    // Extra CRC
    nissan_crc(bytes, 0x85);//not sure if this is really working or just making me look like a muppet.



    Can::GetInterface(0)->Send(0x11A, (uint32_t*)bytes,8);//send 0x11a
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Send target motor torque signal
    ////////////////////////////////////////////////////

    // Data taken from a gen1 inFrame where the car is starting to
    // move at about 10% throttle: F70700E0C74430D4

    // Usually F7, but can have values between 9A...F7 (gen1)
    bytes[0] = 0xF7;
    // 2016: 6E
    // outFrame.data.bytes[0] = 0x6E;

    // Usually 07, but can have values between 07...70 (gen1)
    bytes[1] = 0x07;
    // 2016: 6E
    //outFrame.data.bytes[1] = 0x6E;

    // Requested torque (signed 12-bit value + always 0x0 in low nibble)
    static int16_t last_logged_final_torque_request = 0;
    if(final_torque_request != last_logged_final_torque_request)
    {
        last_logged_final_torque_request = final_torque_request;

    }
    if(final_torque_request >= -2048 && final_torque_request <= 2047)
    {
        bytes[2] = ((final_torque_request < 0) ? 0x80 : 0) |((final_torque_request >> 4) & 0x7f);
        bytes[3] = (final_torque_request << 4) & 0xf0;
    }
    else
    {
        bytes[2] = 0x00;
        bytes[3] = 0x00;
    }

    // MSB nibble: Runs through the sequence 0, 4, 8, C
    // LSB nibble: Precharge report (precedes actual precharge
    //             control)
    //   0: Discharging (5%)
    //   2: Precharge not started (1.4%)
    //   3: Precharging (0.4%)
    //   5: Starting discharge (3x10ms) (2.0%)
    //   7: Precharged (93%)
    bytes[4] = 0x07 | (counter_1d4 << 6);
    //bytes[4] = 0x02 | (counter_1d4 << 6);
    //Bit 2 is HV status. 0x00 No HV, 0x01 HV On.

    counter_1d4++;
    if(counter_1d4 >= 4) counter_1d4 = 0;

    // MSB nibble:
    //   0: 35-40ms at startup when gear is 0, then at shutdown 40ms
    //      after the car has been shut off (6% total)
    //   4: Otherwise (94%)
    // LSB nibble:
    //   0: ~100ms when changing gear, along with 11A D0 b3:0 value
    //      D (0.3%)
    //   2: Reverse gear related (13%)
    //   4: Forward gear related (21%)
    //   6: Occurs always when gear 11A D0 is 01 or 11 (66%)
    //outFrame.data.bytes[5] = 0x44;
    //outFrame.data.bytes[5] = 0x46;

    // 2016 drive cycle: 06, 46, precharge, 44, drive, 46, discharge, 06
    // 0x46 requires ~25 torque to start
    //outFrame.data.bytes[5] = 0x46;
    // 0x44 requires ~8 torque to start
    bytes[5] = 0x44;
    //bit 6 is Main contactor status. 0x00 Not on, 0x01 on.

    // MSB nibble:
    //   In a drive cycle, this slowly changes between values (gen1):
    //     leaf_on_off.txt:
    //       5 7 3 2 0 1 3 7
    //     leaf_on_rev_off.txt:
    //       5 7 3 2 0 6
    //     leaf_on_Dx3.txt:
    //       5 7 3 2 0 2 3 2 0 2 3 2 0 2 3 7
    //     leaf_on_stat_DRDRDR.txt:
    //       0 1 3 7
    //     leaf_on_Driveincircle_off.txt:
    //       5 3 2 0 8 B 3 2 0 8 A B 3 2 0 8 A B A 8 0 2 3 7
    //     leaf_on_wotind_off.txt:
    //       3 2 0 8 A B 3 7
    //     leaf_on_wotinr_off.txt:
    //       5 7 3 2 0 8 A B 3 7
    //     leaf_ac_charge.txt:
    //       4 6 E 6
    //   Possibly some kind of control flags, try to figure out
    //   using:
    //     grep 000001D4 leaf_on_wotind_off.txt | cut -d' ' -f10 | uniq | ~/projects/leaf_tools/util/hex_to_ascii_binary.py
    //   2016:
    //     Has different values!
    // LSB nibble:
    //   0: Always (gen1)
    //   1:  (2016)

    // 2016 drive cycle:
    //   E0: to 0.15s
    //   E1: 2 messages
    //   61: to 2.06s (inverter is powered up and precharge
    //                 starts and completes during this)
    //   21: to 13.9s
    //   01: to 17.9s
    //   81: to 19.5s
    //   A1: to 26.8s
    //   21: to 31.0s
    //   01: to 33.9s
    //   81: to 48.8s
    //   A1: to 53.0s
    //   21: to 55.5s
    //   61: 2 messages
    //   60: to 55.9s
    //   E0: to end of capture (discharge starts during this)

    // This value has been chosen at the end of the hardest
    // acceleration in the wide-open-throttle pull, with full-ish
    // torque still being requested, in
    //   LeafLogs/leaf_on_wotind_off.txt
    //outFrame.data.bytes[6] = 0x00;

    // This value has been chosen for being seen most of the time
    // when, and before, applying throttle in the wide-open-throttle
    // pull, in
    //   LeafLogs/leaf_on_wotind_off.txt
    bytes[6] = 0x30;    //brake applied heavilly.
    //In Gen 2 byte 6 is Charge status.
    //0x8C Charging interrupted
    //0xE0 Charging

    // Value chosen from a 2016 log
    //outFrame.data.bytes[6] = 0x61;

    // Value chosen from a 2016 log
    // 2016-24kWh-ev-on-drive-park-off.pcap #12101 / 15.63s
    // outFrame.data.bytes[6] = 0x01;
    //byte 6 brake signal

    // Extra CRC
    nissan_crc(bytes, 0x85);

    Can::GetInterface(0)->Send(0x1D4, (uint32_t*)bytes,8);//send on can1
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//We need to send 0x1db here with voltage measured by inverter
//Zero seems to work also on my gen1
////////////////////////////////////////////////////////////////

    bytes[0]=0x00;
    bytes[1]=0x00;
    bytes[2]=0x00;
    bytes[3]=0x00;
    bytes[4]=0x00;
    bytes[5]=0x00;
    bytes[6]=counter_1db;
    // Extra CRC in byte 7
    nissan_crc(bytes, 0x85);


    counter_1db++;
    if(counter_1db >= 4) counter_1db = 0;

    Can::GetInterface(0)->Send(0x1DB, (uint32_t*)bytes,8);
//////////////////////////////////////////////////////////////////////////////////////////
    // Statistics from 2016 capture:
    //     10 00000000000000
    //     21 000002c0000000
    //    122 000000c0000000
    //    513 000006c0000000

    // Let's just send the most common one all the time
    // FIXME: This is a very sloppy implementation. Thanks. I try:)
    //  hex_to_data(outFrame.data.bytes, "00,00,06,c0,00,00,00");
    bytes[0]=0x00;
    bytes[1]=0x00;
    bytes[2]=0x06;
    bytes[3]=0xc0;
    bytes[4]=0x00;
    bytes[5]=0x00;
    bytes[6]=0x00;

    Can::GetInterface(0)->Send(0x50B, (uint32_t*)bytes,7);//possible problem here as 0x50B is DLC 7....

    //Need to add 0x1f2 for charge mode
    //Byte 2 bits 5 and 5 charge status transition request. 0x00 other,0x01 Normal charge,0x02 DCFC,0x03 Stop request.
    //Byte 2 bit 1. Connector detect. 0=other,1=v2h.
}

void LeafINV::Send100msMessages()
{
    // FIXME: Temporarily commenting out to suppress warnings while data send is commented out.
    // FIXME: uint32_t canData[2] = { 0, 0 };

    // Can::GetInterface(0)->Send(0x390, canData);


    // Can::GetInterface(0)->Send(0x50C, canData);


    // Can::GetInterface(0)->Send(0x54C, canData);

    run100ms = (run100ms + 1) & 3;
}


int8_t LeafINV::fahrenheit_to_celsius(uint16_t fahrenheit)
{
    int16_t result = ((int16_t)fahrenheit - 32) * 5 / 9;
    if(result < -128)
        return -128;
    if(result > 127)
        return 127;
    return result;
}



void LeafINV::nissan_crc(uint8_t *data, uint8_t polynomial)
{
    // We want to process 8 bytes with the 8th byte being zero
    data[7] = 0;
    uint8_t crc = 0;
    for(int b=0; b<8; b++)
    {
        for(int i=7; i>=0; i--)
        {
            uint8_t bit = ((data[b] &(1 << i)) > 0) ? 1 : 0;
            if(crc >= 0x80)
                crc = (uint8_t)(((crc << 1) + bit) ^ polynomial);
            else
                crc = (uint8_t)((crc << 1) + bit);
        }
    }
    data[7] = crc;
}




