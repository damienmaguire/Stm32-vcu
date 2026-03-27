/*
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
 *
 *New implementation as of V2.02. See :
 *https://openinverter.org/wiki/CAN_communication
 */

#include "Can_OI.h"
#include "params.h"
#include <libopencm3/stm32/crc.h>

uint16_t Can_OI::voltage;
int16_t Can_OI::speed;
int16_t Can_OI::inv_temp;
int16_t Can_OI::motor_temp;
int16_t Can_OI::final_torque_request;
static bool statusInv = 0;
uint8_t Inv_Opmode = 0;
int opmode;
uint16_t InvStartTimeout = 0;

static bool RespondReq = 0;
static bool EmptyField = 0;
static bool ActionSuccess = 0;
static bool CanField = 0;
static uint8_t ConfigStep = 0;
static uint8_t IntStep = 0;
static uint8_t EmptyCount = 0;

static uint16_t xarr = 0;

static uint8_t CanConfigTxArr[5][11] = {
    {0x90, 0x01, 0x00, 0x00, 0xD0, 0x07, 0x00, 0x08, 0xE8, 0x03, 0x00},
    {0x90, 0x01, 0x00, 0x00, 0xD1, 0x07, 0x08, 0x10, 0x10, 0x27, 0x00},
    {0x90, 0x01, 0x00, 0x00, 0xDC, 0x07, 0x18, 0x10, 0xE8, 0x03, 0x00},
    {0x90, 0x01, 0x00, 0x00, 0xE3, 0x07, 0x28, 0x08, 0xE8, 0x03, 0x1E},
    {0x90, 0x01, 0x00, 0x00, 0xE4, 0x07, 0x30, 0x08, 0xE8, 0x03, 0x1E}};

void Can_OI::SetCanInterface(CanHardware *c) {
  can = c;

  can->RegisterUserMessage(0x190); // Open Inv Msg. Dec 400 for RPM.
  // can->RegisterUserMessage(0x19A);//Open Inv Msg. Dec 410 for temps
  // can->RegisterUserMessage(0x1A4);//Open Inv Msg. Dec 420 for Voltage.
  // can->RegisterUserMessage(0x1AE);//Open Inv Msg. Dec 430 for Opmode.
  can->RegisterUserMessage(
      0x580 + NODEIDCANOI); // For config responses to check if it conflicts
                            // with FOCCI NODE 22 Response
}

void Can_OI::DecodeCAN(int id, uint32_t *data) {
  // 0x1A4 bits 0-15 inverter voltage x10
  // 0x190 bits 0-15 motor rpm x1
  // 0x19A bits 0-15 heatsink temp x10

  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
                       // See comments are useful:)

  if (id == 0x190) // Configured message
  {
    Inv_Opmode = bytes[0]; // INVERTER OPMODE
    // 0=Off, 1=Run, 2=ManualRun, 3=Boost, 4=Buck, 5=Sine, 6=AcHeat
    voltage = ((bytes[2] << 8) | (bytes[1])) / 10;
    speed = ((bytes[4] << 8) | (bytes[3]));
    inv_temp = bytes[5] - 30;   // INVERTER TEMP
    motor_temp = bytes[6] - 30; // MOTOR TEMP
  } else if (id ==
             0x580 + NODEIDCANOI) // Response on config Id 1 would be 0x581
  {
    handleSDO(data);
  }
}

void Can_OI::handleSDO(uint32_t data[2]) // SDO responses handling
{
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
                       // See comments are useful:)
  if (RespondReq ==
      1) // only look at this if we have sent a message looking for a response
  {
    if (bytes[0] == 0x80) {
      EmptyField = true;
      EmptyCount++;
    } else if (bytes[0] == 0x43) {
      CanField = true;
      EmptyCount = 0;
    } else if (bytes[0] == 0x60) {
      ActionSuccess = true;
      EmptyCount = 0;
    } else {
      EmptyCount = 0;
    }
    RespondReq = 0; // recieved message so clear respond request

    if (EmptyCount > 4) // 4 empty fields/Aborts recieved stop configuring
    {
      ConfigStep = 5;
    }
  }
}

void Can_OI::SetTorque(float torquePercent) {
  // Not used
  final_torque_request = torquePercent * 10;
  Param::SetInt(
      Param::torque,
      final_torque_request); //!!! NOT USED BY IO inverter post processed final
                             //! torue value sent to inv to web interface
  ////

  int opmode = Param::GetInt(Param::opmode);
  int dir = Param::GetInt(Param::dir);

  uint8_t tempIO = 0;

  if (opmode == MOD_RUN) // No need to send CAN any time but run mode
  {
    if (dir == 1) // Forward
    {
      tempIO += 8;        // Forward, only send direction data if in run mode
    } else if (dir == -1) // Reverse
    {
      tempIO += 16; // Reverse, only send direction data if in run mode
    }

    if (Param::GetBool(Param::din_brake))
      tempIO += 4;
    // if(Param::GetBool(Param::din_start)) tempIO+=2;
    if (InvStartTimeout !=
        0) // Set the start signal true for 3 seconds on vcu entering run mode.
    {
      InvStartTimeout--;
      tempIO += 2; // inv start signal on for 3 secs once enter vcu run mode
    }

    /*
    pot[0:11]
    pot2[12:23]
    canio[24:29]
    cruise[24]
    start[25]
    brake[26]
    forward[27]
    reverse[28]
    bms[29]
    canrun1[30:31]
    cruisespeed[32:45]
    canrun2[46:47]
    regenpreset[48:55]
    cancrc[56:63]
    */

    uint32_t data[2];
    uint32_t pot = Param::GetInt(Param::pot) & 0xFFF;
    uint32_t pot2 = Param::GetInt(Param::pot2) & 0xFFF;
    uint32_t canio = tempIO & 0x3F;
    uint32_t ctr = Param::GetInt(Param::canctr) & 0x3;
    uint32_t cruise =
        0; // Param::GetInt(Param::cruisespeed) & 0x3FFF; // Cruise disabled
    uint32_t regen = 100 & 0x7F; // Param::GetInt(Param::potbrake) & 0x7F; //
                                 // Fixed for now 100% regen allowed

    data[0] = pot | (pot2 << 12) | (canio << 24) | (ctr << 30);
    data[1] = cruise | (ctr << 14) | (regen << 16);

    crc_reset();
    uint32_t crc = crc_calculate_block(data, 2) & 0xFF;
    data[1] |= crc << 24;

    can->Send(0x3F, data); // send 0x3F
  }

  if (opmode == MOD_OFF) {
    InvStartTimeout = 300;
  }
}

int Can_OI::GetInverterState() {
  if (Inv_Opmode == 0)
    statusInv = 0;
  if (Inv_Opmode == 1)
    statusInv = 1;
  if (opmode == MOD_OFF)
    statusInv = 0; // Resort to off if vcu is in off state.
  return statusInv;
}

void Can_OI::Task100Ms() {
  opmode = Param::GetInt(Param::opmode);
  if (opmode == MOD_OFF) {
    voltage = 0; // Clear all vals when in off mode to ensure real vals are
                 // displayed on next start
    speed = 0;
    inv_temp = 0;
    motor_temp = 0;
    Inv_Opmode = 0;
    final_torque_request = 0;
  }

  if (Param::GetInt(Param::ConfigCANOI) == 1) // do the config shit
  {
    ConfigCan();
  }
}

void Can_OI::ConfigCan() {

  // CAN sending//
  uint8_t bytes[8];

  if (ConfigStep == 0) // Clear all TX CAN configs
  {
    if (RespondReq == 0 && EmptyField == 0) {
      bytes[0] = 0x23; // write request general
      bytes[1] = 0x00; // Can TX message index
      bytes[2] = 0x31; // Can TX message
      bytes[3] = 0x00; // sub index
      bytes[4] = 0x00;
      bytes[5] = 0x00;
      bytes[6] = 0x00;
      bytes[7] = 0x00;
      can->Send(0x600 + NODEIDCANOI, (uint32_t *)bytes, 8); //

      RespondReq = 1;
    }
    if (EmptyField == 1) {
      EmptyField = 0;
      ConfigStep++;
    }
  } else if (ConfigStep == 1) // Clear all RX CAN configs
  {
    if (RespondReq == 0 && EmptyField == 0) {
      bytes[0] = 0x23; // write request general
      bytes[1] = 0x80; // Can RX message index
      bytes[2] = 0x31; // Can message
      bytes[3] = 0x00; // sub index
      bytes[4] = 0x00;
      bytes[5] = 0x00;
      bytes[6] = 0x00;
      bytes[7] = 0x00;
      can->Send(0x600 + NODEIDCANOI, (uint32_t *)bytes, 8); //

      RespondReq = 1;
    }
    if (EmptyField == 1) {
      EmptyField = 0;
      ConfigStep++;
      IntStep = 0;
      ActionSuccess = false;
    }
  } else if (ConfigStep == 2) // Config TX CAN frames
  {
    if (xarr < 6) {
      if (IntStep == 0) {
        if (ActionSuccess == true) {
          IntStep = 1;
          ActionSuccess = false;
        } else if (RespondReq == 0) {
          bytes[0] = 0x23;                    // write request general
          bytes[1] = 0x00;                    // write request general
          bytes[2] = 0x30;                    // write request can tx
          bytes[3] = 0x00;                    // write request can tx index
          bytes[4] = CanConfigTxArr[xarr][0]; // CAN ID
          bytes[5] = CanConfigTxArr[xarr][1]; // CAN ID
          bytes[6] = CanConfigTxArr[xarr][2]; // CAN ID
          bytes[7] = CanConfigTxArr[xarr][3]; // CAN ID

          can->Send(0x600 + NODEIDCANOI, (uint32_t *)bytes, 8); //

          RespondReq = 1;
        }
      }

      if (IntStep == 1) {
        if (ActionSuccess == true) {
          IntStep = 2;
          ActionSuccess = false;
        } else if (RespondReq == 0) {
          bytes[0] = 0x23;                    // write request general
          bytes[1] = 0x00;                    // write request general
          bytes[2] = 0x30;                    // write request can tx
          bytes[3] = 0x01;                    // write request can tx index
          bytes[4] = CanConfigTxArr[xarr][4]; // Param ID
          bytes[5] = CanConfigTxArr[xarr][5]; // Param ID
          bytes[6] = CanConfigTxArr[xarr][6]; // Start Bit
          bytes[7] = CanConfigTxArr[xarr][7]; // Length in Bit

          can->Send(0x600 + NODEIDCANOI, (uint32_t *)bytes, 8); //
          RespondReq = 1;
        }
      }

      if (IntStep == 2) {
        if (ActionSuccess == true) {
          IntStep = 0;
          ActionSuccess = false;
          xarr++;
        } else if (RespondReq == 0) {
          bytes[0] = 0x23;                    // write request general
          bytes[1] = 0x00;                    // write request general
          bytes[2] = 0x30;                    // write request can tx
          bytes[3] = 0x02;                    // write request can tx index
          bytes[4] = CanConfigTxArr[xarr][8]; // Gain LSB
          bytes[5] = CanConfigTxArr[xarr][9]; // Gain MSB
          bytes[6] = 0x00; // Offset LSB - No change for mapping
          bytes[7] = CanConfigTxArr[xarr][10];
          ;                                                     // Offset MSB
          can->Send(0x600 + NODEIDCANOI, (uint32_t *)bytes, 8); //

          RespondReq = 1;
        }
      }
    } else {
      RespondReq = 0;
      IntStep = 0;
      xarr = 0;
      ConfigStep++;
    }
  } else if (ConfigStep == 3) // Config RX CAN frames
  {
    /* //NO RX FRAMES needed
    if(xarr < 7)
    {
        if(IntStep == 0)
        {
            if(ActionSuccess == true)
            {
                IntStep= 1;
                ActionSuccess = false;
            }
            else if(RespondReq == 0)
            {
                bytes[0] = 0x23; //write request general
                bytes[1] = 0x01; //write request can rx
                bytes[2] = 0x30; //write request can rx
                bytes[3] = 0x00; //write request can rx index
                bytes[4] = CanConfigRxArr[xarr][0]; //CAN ID
                bytes[5] = CanConfigRxArr[xarr][1];  //CAN ID
                bytes[6] = CanConfigRxArr[xarr][2];  //CAN ID
                bytes[7] = CanConfigRxArr[xarr][3];  //CAN ID

                can->Send(0x600 + NODEIDCANOI, (uint32_t*)bytes,8); //
                RespondReq = 1;
            }
        }

        if(IntStep == 1)
        {
            if(ActionSuccess == true)
            {
                IntStep= 2;
                ActionSuccess = false;
            }
            else if(RespondReq == 0)
            {
                bytes[0] = 0x23; //write request general
                bytes[1] = 0x01; //write request can rx
                bytes[2] = 0x30; //write request can rx
                bytes[3] = 0x01; //write request can tx index
                bytes[4] = CanConfigRxArr[xarr][4];  //Param ID
                bytes[5] = CanConfigRxArr[xarr][5];  //Param ID
                bytes[6] = CanConfigRxArr[xarr][6];  //Start Bit
                bytes[7] = CanConfigRxArr[xarr][7];  //Length in Bit

                can->Send(0x600 + NODEIDCANOI, (uint32_t*)bytes,8); //
                RespondReq = 1;
            }
        }

        if(IntStep == 2)
        {
            if(ActionSuccess == true)
            {
                IntStep= 0;
                ActionSuccess = false;
                xarr++;
            }
            else if(RespondReq == 0)
            {
                bytes[0] = 0x23; //write request general
                bytes[1] = 0x01; //write request can rx
                bytes[2] = 0x30; //write request can rx
                bytes[3] = 0x02; //write request can tx index
                bytes[4] = 0xE8; //Gain LSB - No change for mapping
                bytes[5] = 0x03; //Gain MSB - No change for mapping
                bytes[6] = 0x00; //Offset LSB - No change for mapping
                bytes[7] = 0x00; //Offset MSB - No change for mapping

                can->Send(0x600 + NODEIDCANOI, (uint32_t*)bytes,8); //
                RespondReq = 1;
            }
        }
    }
    else
    {
        RespondReq = 0;
        IntStep = 0;
        xarr = 0;
        ActionSuccess = false;
        ConfigStep++;
    }
    */
    RespondReq = 0;
    IntStep = 0;
    xarr = 0;
    ActionSuccess = false;
    ConfigStep++;
  } else if (ConfigStep == 4) // Save CAN Mapping to flash
  {
    if (RespondReq == 0 && ActionSuccess == false) {
      bytes[0] = 0x23; // write request general
      bytes[1] = 0x02;
      bytes[2] = 0x50; // Save all to flash
      bytes[3] = 0x00;
      bytes[4] = 0x00;
      bytes[5] = 0x00;
      bytes[6] = 0x00;
      bytes[7] = 0x00;

      can->Send(0x600 + NODEIDCANOI, (uint32_t *)bytes, 8); //
      ActionSuccess = false;
      RespondReq = true;
    } else if (ActionSuccess == true) {
      ConfigStep++;
    }
  } else if (ConfigStep == 5) // Config completed
  {
    Param::SetInt(Param::ConfigCANOI, 0);
  }
}
