#include <heater.h>

//This class handles cabin heaters such as the Ampera heater (swcan on CAN3) or VW heater (LIN).

/*
Ampera heater info from : https://leafdriveblog.wordpress.com/2018/12/05/5kw-electric-heater/
http://s80ev.blogspot.com/2016/12/the-heat-is-on.html
https://github.com/neuweiler/GEVCUExtension/blob/develop/EberspaecherHeater.cpp

 * The heater communicates using J1939 protocol. It has to be "woken up" one time with a 0x100 message and then
 * must see a "keep alive" to stay active, which is the 0x621 message. The message repetition rate is between
 * 25 and 100ms intervals.

LV connector has SW CAN, ENABLE and GND pins.

Temperature sensor is 3.2Kohm at 21degC. It is NTC type.

The Eberspacher CAN version of their PTC liquid heater used in the Chevrolet Volt will work when used with a 33.33Kb SWCAN.
The data below is what we have found to be the minimum required to turn on and operate this heater.
This capture will operate the heater at approximately 33% of full power.
To command higher power, increase the value of message 0x10720099 byte 1 (it begins with byte 0) which is 3E below.
We saw full power heat when 85 was used as the value for byte 1 and that value will vary based upon inlet temperature.
The data below contains an entry for “Bus” and that refers to which CAN Bus of the 2 Buses on the CANDue recorded the original event.

ID,Extended,Bus,LEN,D0,D1,D2,D3,D4,D5,D6,D7
0x100,False,1,0,0,0,0,0,0,0,0,0
0x621,False,1,8,0,40,0,0,0,0,0,0
0x10720099,True,1,5,2,3E,0,0,0,0,0,0
0x13FFE060,True,1,0,0,0,0,0,0,0,0,0
0x10720099,True,1,5,2,3E,0,0,0,0,0,0
0x102CC040,True,1,8,1,1,CF,0F,0,51,46,60
0x10242040,True,1,1,0,0,0,0,0,0,0,0
0x102CC040,True,1,8,1,1,CF,0F,0,51,46,60
0x102CC040,True,1,8,1,1,CF,0F,0,51,46,60
0x102CC040,True,1,8,1,1,CF,0F,0,51,46,60
0x102740CB,True,1,3,2D,0,0,0,0,0,0,0
0x102740CB,True,1,3,19,0,0,0,0,0,0,0
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Info on VW heater from : https://openinverter.org/forum/viewtopic.php?f=24&p=31718&sid=37fd4c47b77d59aa95c9af42f46d8b31#p31718

I found the LIN messages for the water heater. The feedback comes on ID48. Byte 0 is power. 13 for 770W, 26 for 1540W.
ID28 is sent for control. Byte 0 is power, last bit of byte 1 starts and stops.

connectors are:
HVA 280 KEY_E from TE +HV near to the body, -HV far from the body
1J0973714 1= +12V , 2= GND, 4= LIN

uint8_t checksum (uint8_t * pdata, uint8_t len, uint8_t mode)
{
uint16_t tmp;
if (mode == CLASSIC) tmp = 0;
else tmp = pdata[0];
for (uint8_t i=1; i<=len; i++)
{
tmp = tmp + pdata[ i ];
if (tmp>255) tmp = tmp-255;
}
//tmp = pdata[3]+11;
tmp = 255-tmp;
return tmp;
}

*/

uCAN_MSG txMessage_Ampera;

/*
 * Wake up all SW-CAN devices by switching the transceiver to HV mode and
 * sending the command 0x100 and switching the HV mode off again.
 */
void AmperaHeater::sendWakeup()
{
    DigIo::sw_mode0.Clear();
    DigIo::sw_mode1.Set();  // set HV mode

    // 0x100, False, 0, 00,00,00,00,00,00,00,00
    txMessage_Ampera.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    txMessage_Ampera.frame.id = 0x100;
    txMessage_Ampera.frame.dlc = 8;
    txMessage_Ampera.frame.data0 = 0x00;
    txMessage_Ampera.frame.data1 = 0x00;
    txMessage_Ampera.frame.data2 = 0x00;
    txMessage_Ampera.frame.data3 = 0x00;
    txMessage_Ampera.frame.data4 = 0x00;
    txMessage_Ampera.frame.data5 = 0x00;
    txMessage_Ampera.frame.data6 = 0x00;
    txMessage_Ampera.frame.data7 = 0x00;
    CANSPI_Transmit(&txMessage_Ampera);
    //may need dealy here
    DigIo::sw_mode0.Set();
    DigIo::sw_mode1.Set();  // set normal mode
}
