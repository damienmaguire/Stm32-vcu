#include <BMW_E65.h>
#include "stm32_can.h"
#include "params.h"

uint8_t  Gcount; //gear display counter byte
uint8_t shiftPos=0xe1; //contains byte to display gear position on dash.default to park
uint8_t gear_BA=0x03; //set to park as initial condition
uint8_t mthCnt;
uint8_t C1D00 = 0x00;  //0x1D0 counter
uint8_t C1D01 = 0x00;  //0x1D0 counter
uint8_t A80=0xbe;//0x0A8 first counter byte
uint8_t A81=0x00;//0x0A8 second counter byte
uint8_t A90=0xe9;//0x0A9 first counter byte
uint8_t A91=0x00;//0x0A9 second counter byte
uint8_t BA5=0x4d;//0x0BA first counter byte(byte 5)
uint8_t BA6=0x80;//0x0BA second counter byte(byte 6)
uint8_t AA1=0x00;//0x0AA First counter byte


void BMW_E65::SetCanInterface(CanHardware* c)
{
    can = c;

    can->RegisterUserMessage(0x130);//E65 CAS
    can->RegisterUserMessage(0x192);//E65 Shifter
    can->RegisterUserMessage(0x480);//Network Management
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////Handle incomming pt can messages from the car here
////////////////////////////////////////////////////////////////////////////////////////////////////
void BMW_E65::DecodeCAN(int id, uint32_t* data)
{

    switch (id)
    {
    case 0x130:
        BMW_E65::handle130(data);
        break;

    case 0x480:
        BMW_E65::handle480(data);
        break;

    default:
        break;
    }
}

void BMW_E65::handle130(uint32_t data[2])
{
    uint8_t* bytes = (uint8_t*)data;
    /*
        if ((bytes[0] == 0x45) || (bytes[0] == 0x55))
        {
            // 0x45 is run, 0x55 is engine crank request
            terminal15On = true;
        }
        else
        {
            terminal15On = false;
        }
      */
    if ((bytes[0] & 0x01) > 0)
    {
        terminalROn = true;
    }
    else
    {
        terminalROn = false;
    }

    if ((bytes[0] & 0x04) > 0)
    {
        terminal15On = true;
    }
    else
    {
        terminal15On = false;
    }

    if ((bytes[0] & 0x10) > 0)
    {
        terminal50On = true;
    }
    else
    {
        terminal50On = false;
    }

    if ((bytes[2] & 0x08) > 0)
    {
        StartButt = true;
    }
    else
    {
        StartButt = false;
    }
}


void BMW_E65::handle480(uint32_t data[2])
{
    uint8_t* bytes = (uint8_t*)data;

    if (bytes[1] == 0x32)
    {
        CANWake = false;
    }
    else
    {
        CANWake = true;
    }
}

void BMW_E65::Task10Ms()
{
    if(CANWake)
    {
        SendAbsDscMessages(Param::GetBool(Param::din_brake));
    }
}

void BMW_E65::Task100Ms()
{
    if(CANWake)
    {
        uint32_t data[2];

        data[0] = 0x8261; //sets max rpm on tach (temp thing)

        /////////////////this can id must be sent once at T15 on to fire up the instrument cluster/////////////////////////
        if (!this->dashInit)
        {
            for (int i = 0; i < 3; i++)
            {
                can->Send(0x332, data, 2); //Send on CAN2
            }
            this->dashInit=true;
        }

        BMW_E65::Engine_Data();
    }
}

void BMW_E65::Task200Ms()
{
    if(CANWake)
    {
        //update shitPos
        int selectedDir = Param::GetInt(Param::dir);

        if (selectedDir == 0)
        {
            //neutral/park
            this->gear = PARK;
            gear_BA = 0x03;
            shiftPos = 0xe1;
        }
        else if (selectedDir == -1)
        {
            //reverse
            this->gear = REVERSE;
            gear_BA = 0x02;
            shiftPos = 0xd2;
        }
        else if (selectedDir == 1)
        {
            //forward
            this->gear = DRIVE;
            gear_BA = 0x08;
            shiftPos = 0x78;
        }

        uint8_t bytes[8];
///////////////////////////////////////////////////////////////////////////////////////////////////
        bytes[0]=shiftPos;  //e1=P  78=D  d2=R  b4=N
        bytes[1]=0x0c;
        bytes[2]=0x8f;
        bytes[3]=Gcount;
        bytes[4]=0xf0;


        can->Send(0x1D2, (uint32_t*)bytes,5); //Send on CAN2
        ///////////////////////////
        //Byte 3 is a counter running from 0D through to ED and then back to 0D///
        //////////////////////////////////////////////

        Gcount=Gcount+0x10;
        if (Gcount==0xED)
        {
            Gcount=0x0D;
        }
    }
}

void BMW_E65::DashOff()
{
    this->dashInit=false;
}

void BMW_E65::SendAbsDscMessages(bool Brake_In)
{

//////////send abs/dsc messages////////////////////////
    uint8_t bytes[8];

    uint16_t RPM_A = 0;
    if (Ready())
    {
        RPM_A = 750 * 4;
        bytes[1] = 0x50 | AA1;  //Counter for 0xAA Byte 0
        bytes[2] = 0x07;
        bytes[6] = 0x94;
        bytes[7] = 0x00;
    }
    else
    {
        bytes[1] = 0x30 | AA1;  //Counter for 0xAA Byte 0
        bytes[2] = 0xFE;
        bytes[6] = 0x84;
        bytes[7] = 0x00;
    }
    bytes[3] = 0x00;             //Pedal position 0-255
    bytes[4] = RPM_A & 0xff;   //lowByte(RPM_A);
    bytes[5] = RPM_A>>8 & 0xff;;  //highByte(RPM_A);


    ///Check sum math for 0x0AA///
    int16_t check_AA = (bytes[1] + bytes[2] + bytes[3] + bytes[4] + bytes[5] + bytes[6] + bytes[7] + 0xAA);
    check_AA = (check_AA / 0x100) + (check_AA & 0xff);
    check_AA = check_AA & 0xff;
    bytes[0] = check_AA;  //checksum

    can->Send(0x0AA, bytes, 8); //Send on CAN


    uint8_t a8_brake;


    if(Brake_In)
    {
        a8_brake=0x64;
    }

    else
    {
        a8_brake=0x04;
    }

    int16_t check_A8 = (A81+0x21+0xe0+0x21+0x1f+0x0f+a8_brake+0xa8);
    check_A8 = (check_A8 / 0x100)+ (check_A8 & 0xff);
    check_A8 = check_A8 & 0xff;

    bytes[0]=check_A8;  //checksum
    bytes[1]=A81; //counter byte
    bytes[2]=0x21;
    bytes[3]=0xe0;
    bytes[4]=0x21;
    bytes[5]=0x1f;
    bytes[6]=0x0f;
    bytes[7]=a8_brake;  //brake off =0x04 , brake on = 0x64.

    can->Send(0x0A8, bytes, 8); //Send on CAN

    bytes[0]=A90; //first counter byte
    bytes[1]=A91; //second counter byte
    bytes[2]=0x79;
    bytes[3]=0xdf;
    bytes[4]=0x1d;
    bytes[5]=0xc7;
    bytes[6]=0xe0;
    bytes[7]=0x21;

    can->Send(0x0A9, bytes, 8); //Send on CAN

    int16_t check_BA = (gear_BA+0xff+0x0f+BA6+0x0ba);
    check_BA = (check_BA / 0x100)+ (check_BA & 0xff);
    check_BA = check_BA & 0xff;

    bytes[0]=gear_BA; //was just 0x03
    bytes[1]=0xff;
    bytes[2]=0x0f;
    bytes[3]=0x00;
    bytes[4]=0x00;
    bytes[5]=check_BA; //BA5; //counter byte 5
    bytes[6]=BA6; //counter byte 6

    can->Send(0x0BA, bytes, 7); //Send on CAN2

////////////////////////////////////////
////here we increment the abs/dsc msg counters

    AA1++;
    A80++;
    A81++;
    A90++;
    A91++;
    BA5++;
    BA6++;

    if (BA5==0x5C) //reload initial condition
    {
        AA1 = 0x0;  //0x0AA second counter byte
        A80=0xbe;//0x0A8 first counter byte
        A81=0x00;//0x0A8 second counter byte
        A90=0xe9;//0x0A9 first counter byte
        A91=0x00;//0x0A9 second counter byte
        BA5=0x4d;//0x0BA first counter byte(byte 5)
        BA6=0x80;//0x0BA second counter byte(byte 6)
    }

}

void BMW_E65::Engine_Data()
{
    uint8_t bytes[8];

    uint8_t EngRun = 0x00;

    if (Param::GetInt(Param::opmode) == MOD_RUN)
    {
        EngRun = 0x60;
        bytes[4] = 0x9C;
        bytes[5] = 0x9E;
    }
    else
    {
        bytes[4] = 0x00;
        bytes[5] = 0x00;
    }

    bytes[0] = 0x3C;            //Engine Coolant Temp
    bytes[1] = 0xFF;            //Engine Oil Temp
    bytes[2] = EngRun | C1D00;  //Counter
    bytes[3] = 0xC3;

    bytes[6] = 0xCD;
    bytes[7] = 0x82;  //Idle Traget

    can->Send(0x1D0,bytes,8); //Send on CAN2

    if (C1D00 == C1D01)
    {
        C1D00++;
        if (C1D00 == 0x0F)
        {
            C1D00 = 0x00;
        }
    }
    else
    {
        C1D01 = C1D00;
    }

}

bool BMW_E65::GetGear(Vehicle::gear& outGear)
{
    return false;
}
