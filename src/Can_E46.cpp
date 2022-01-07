
#include "Can_E46.h"
#include "stm32_can.h"

static uint8_t counter_329 = 0;
static uint8_t ABSMsg = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////

//these messages go out on vehicle can and are specific to driving the E46 instrument cluster etc.

//////////////////////DME Messages //////////////////////////////////////////////////////////
void Can_E46::Msg316(uint16_t speed_input)
{
    // Limit tachometer range from 750 RPMs - 7000 RPMs at max.
    // These limits ensure the vehicle thinks engine is alive and within the
    // max allowable RPM.
    // FIXME: Verify if this is true, or is the Terminal 15 bit which is
    //        more important.
    // Comparison uses ternary operator.
    speed_input = (speed_input < 750) ? 750 : speed_input;
    speed_input = (speed_input > 7000) ? 7000 : speed_input;

    uint8_t rpm_to_can_mult = 64;
    uint8_t rpm_to_can_div = 10;

    uint8_t canRPMlo = ((speed_input * rpm_to_can_mult) / rpm_to_can_div) & 0xFF;
    uint8_t canRPMhi = ((speed_input * rpm_to_can_mult) / rpm_to_can_div) >> 8;

    // Declare data frame array.
    uint8_t bytes[8];

    // Byte 0 - Status - 0x01 is Terminal 15 Status, 0x04 is Traction Control OK
    bytes[0]=0x05;
    // Byte 1 - Torque with all interventions
    bytes[1]=0x00;
    // Byte 2 / 3 "Engine" RPM, RPM * 6.4. 16 bits, Intel LSB (LSB,MSB)
    bytes[2]=canRPMlo;  //RPM LSB
    bytes[3]=canRPMhi;  //RPM MSB
    // Byte 4 - Driver Desired Torque
    bytes[4]=0x00;
    // Byte 5 - Friction Torque
    bytes[5]=0x00;
    // Byte 6 - Various other status bits
    bytes[6]=0x00;
    // Byte 7 - Torque with internal interventions only
    bytes[7]=0x00;

    Can::GetInterface(1)->Send(0x316, (uint32_t*)bytes,8); //Send on CAN2
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Can_E46::Msg329(uint16_t tempValue)
{
    //********************temp sense  *******************************
    //  tempValue=analogRead(tempIN); //read Analog pin voltage
    //  The sensor and gauge are not linear.  So if the sensed
    //  Voltage is less than the mid point the Map function
    //  is used to map the input values to output values For the gauge
    //  output values (in decimal):
    //  86 = First visible movment of needle starts
    //  93 = Begining of Blue section
    //  128 = End of Blue Section
    //  169 = Begin Straight up
    //  193 = Midpoint of needle straight up values
    //  219 = Needle begins to move from center
    //  230 = Beginning of Red section
    //  254 = needle pegged to the right
    //  MAP program statement: map(value, fromLow, fromHigh, toLow, toHigh)
    //  if(tempValue < 964){  //if pin voltage < mid point value
    //      tempValue= inverter_status.inverter_temperature;  //read temp from leaf inverter can.
    //      tempValue= map(tempValue,15,80,88,254); //Map to e46 temp gauge
    //  }
    //  else
    //  {
    //      tempValue= map(tempValue,964,1014,219,254); //Map upper half of range
    //  }

    //Can bus data packet values to be sent
    // MSG 0x329

    // Declare data frame array.
    uint8_t bytes[8];

    // Byte 0 - Bits 6-7 Multiplexer ID, Bits 0-5 Data
    bytes[0]=ABSMsg;  //needs to cycle 11,86,d9
    // Byte 1 - Coolant Temperature
    bytes[1]=tempValue; //temp bit tdata
    // Byte 2 - Atmospheric Pressure in mbar
    bytes[2]=0xc5;
    // Byte 3 - Status bits - 0x10 Engine running,
    //          0x08 Coolant Temp > 60 deg C (Warmup bit), Other bits include
    //          cruise control bits.
    bytes[3]=0x00;
    // Byte 4 - Cruise control desired relative torque, 0-99.6% (0-254)
    bytes[4]=0x00;
    // Byte 5 - Driver desired relative torque, 0-99.6% (0-254)
    //          Max of driver or cruise desired value.
    bytes[5]=0x00;
    // Byte 6 - 0x01 Brake Pedal pressed, 0x02 Brake Light Switch error,
    //          0x04 Kickdown, 0x08 Cruise Enabled
    bytes[6]=0x00;
    // Byte 7 - Zero, or -1 (255)
    bytes[7]=0x00;

    counter_329++;
    if(counter_329 >= 22) counter_329 = 0;
    if(counter_329==0) ABSMsg=0x11;
    if(counter_329==8) ABSMsg=0x86;
    if(counter_329==15) ABSMsg=0xd9;

    Can::GetInterface(1)->Send(0x329, (uint32_t*)bytes,8); //Send on CAN2
}

void Can_E46::Msg43F(int8_t gear)
{
    //Can bus data packet values to be sent
    uint8_t bytes[8];
    // Source: https://www.bimmerforums.com/forum/showthread.php?1887229-E46-Can-bus-project&p=30055342#post30055342
    // byte 0 = 0x81 //doesn't do anything to the ike
    bytes[0] = 0x81;
    // byte 1 = 0x01 where;
    // 01 = first gear
    // 02= second gear
    // 03 = third gear
    // 04 = fourth gear
    // 05 = D
    // 06 = N
    // 07 = R
    // 08 = P
    // 09 = 5
    // 0A = 6
    switch (gear)
    {
    case -1 /* Reverse */:
      bytes[1] = 0x07;
      break;
    case 0 /* Neutral */:
      bytes[1] = 0x06;
      break;
    case 1 /* Drive */:
      bytes[1] = 0x05;
      break;
    default:
      bytes[1] = 0x08;
      break;
    }

    // byte 2 = 0xFF where;
    // FF = no display
    // 00 = E
    // 39 = M
    // 40 = S
    bytes[2] = 0xFF;

    // byte 3 = 0xFF //doesn't do anything to the ike
    bytes[3] = 0xFF;

    // byte 4 = 0x00 //doesn't do anything to the ike
    bytes[4] = 0x00;

    // byte 5 = 0x80 where;
    // 80 = clears the gear warning picture - all other values bring it on
    bytes[5] = 0x80;

    // byte 6 = 0xFF //doesn't do anything to the ike
    bytes[6] = 0xFF;

    // byte 7 = 0x00 //doesn't do anything to the ike
    bytes[7] = 0xFF;

    Can::GetInterface(1)->Send(0x43F, (uint32_t*)bytes,8); //Send on CAN2
}

void Can_E46::Msg545()
{
    // int z = 0x60; // + y;  higher value lower MPG

    // Data sent to instrument cluster. Status and slow moving data.
    // Fuel consumption is fuel usage (since start) in uL % 65536.

    //MSG 0x545
    //Can bus data packet values to be sent
    uint8_t bytes[8];

    // Byte 0 - 2 Check Engine, 8 Cruise Enabled , 0x10 EML, 0x40 Gas Cap
    bytes[0]=0x00;
    // Byte 1 - Fuel consumption LSB
    bytes[1]=0x00;
    // Byte 2 - Fuel consumption MSB
    bytes[2]=0x00;
    // Byte 3 - 0x08 Overheat, Yellow Oil Level 0x02, M3 cluster shift lights
    bytes[3]=0x00;
    // Byte 4 - Oil Temperature
    bytes[4]=0x7E;
    // Byte 5 - Battery light, 0x01.
    bytes[5]=0x10;
    // Byte 6 - Unused
    bytes[6]=0x00;
    // Byte 7 - 0x80 Oil Pressure (Red Oil light), Idle set speed
    bytes[7]=0x18;

    Can::GetInterface(1)->Send(0x545, (uint32_t*)bytes,8); //Send on CAN2
}

