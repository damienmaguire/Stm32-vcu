#include <charger.h>





/*
//From SimpBMS by Tom DeBree:

  if (settings.chargertype == ChevyVolt)
  {
    msg.id  = 0x30E;
    msg.len = 1;
    msg.buf[0] = 0x02; //only HV charging , 0x03 hv and 12V charging
    Can0.write(msg);

    msg.id  = 0x304;
    msg.len = 4;
    msg.buf[0] = 0x40; //fixed
    if ((chargecurrent * 2) > 255)
    {
      msg.buf[1] = 255;
    }
    else
    {
      msg.buf[1] = (chargecurrent * 2);
    }
    if ((settings.ChargeVsetpoint * settings.Scells ) > 200)
    {
      msg.buf[2] = highByte(uint16_t((settings.ChargeVsetpoint * settings.Scells ) * 2));
      msg.buf[3] = lowByte(uint16_t((settings.ChargeVsetpoint * settings.Scells ) * 2));
    }
    else
    {
      msg.buf[2] = highByte( 400);
      msg.buf[3] = lowByte( 400);
    }
    Can0.write(msg);
  }

  if (settings.chargertype == Coda)
  {
    msg.id  = 0x050;
    msg.len = 8;
    msg.buf[0] = 0x00;
    msg.buf[1] = 0xDC;
    if ((settings.ChargeVsetpoint * settings.Scells ) > 200)
    {
      msg.buf[2] = highByte(uint16_t((settings.ChargeVsetpoint * settings.Scells ) * 10));
      msg.buf[3] = lowByte(uint16_t((settings.ChargeVsetpoint * settings.Scells ) * 10));
    }
    else
    {
      msg.buf[2] = highByte( 400);
      msg.buf[3] = lowByte( 400);
    }
    msg.buf[4] = 0x00;
    if ((settings.ChargeVsetpoint * settings.Scells)*chargecurrent < 3300)
    {
      msg.buf[5] = highByte(uint16_t(((settings.ChargeVsetpoint * settings.Scells) * chargecurrent) / 240));
      msg.buf[6] = highByte(uint16_t(((settings.ChargeVsetpoint * settings.Scells) * chargecurrent) / 240));
    }
    else //15 A AC limit
    {
      msg.buf[5] = 0x00;
      msg.buf[6] = 0x96;
    }
    msg.buf[7] = 0x01; //HV charging
    Can0.write(msg);
  }
}

//From Arber Blog:

The Lear operates CAN at 500Kbps.
Select which of the charger modules you whish to operate. Only one msg is sent for heartbeat, do not send all three messages!
Address DLC Data0 Data1 Data2 Data3
0x30E     1     0x01                                                Turn on aux12vdc
0x30E     1     0x02                                                Turn on HV Charging
0x30E     1     0x03                                                Turn on aux12vdc and turn on HV Charging
This message is sent at 30ms interval
Then send command line.
0x304     4       0x40   0xA0   0x03   0x20           Charge at 8amps to 400.0vdc
This is sent at 500ms interval
Address 0x304 Data0 is an unknown at present, but seem to be 40 or 48 in the Volt.
Address 0x304 Data1 is the current commanded, convert to decimal and divide by 20.
Address 0x304 Data2 first 2 bits are MSB of the voltage command.
Address 0x304 Data3 byte is the LSB of the voltage command. Then MSB LSB is divided by 2.
Example:
Data1 is A0(hex) which is 160 decimal. Divided by 20 is 8 and that is the commanded current.
Data2 is 03(hex) and Data3 is 20(hex) which is 0320(hex) equals 800(decimal) divided by 2 is 400vdc.
Charging at 10A and to 410V
0x304     4       0x40   0xC8   0x03   0x34
Current measurement is in returned message 0x212 data byte0 and data byte1 as a 16 bit integer, then divide that by 158 to read actual output current.

Charger really needs liquid cooling as it gets hot quickly when charging at 8A. 01C0

Charging max at 12.6A to 224Vdc needs the following signal then:

ID 0x304 MSG 0x40 0xFC 0x01 0xC0 transmitted at 500ms

*/

