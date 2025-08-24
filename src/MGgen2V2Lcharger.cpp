/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2021-2025  Johannes Huebner <dev@johanneshuebner.com>
 * 	                        Damien Maguire <info@evbmw.com>
 *                          Ben Bament
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

#include <MGgen2V2Lcharger.h>

/* Control of the MG Gen 2 V2L charger. */

uint8_t MGgen2V2Lcharger::chgStatus;
uint8_t MGgen2V2Lcharger::evseDuty;
float MGgen2V2Lcharger::dcBusV;
float MGgen2V2Lcharger::temp_1;
float MGgen2V2Lcharger::temp_2;
float MGgen2V2Lcharger::ACVolts;
float MGgen2V2Lcharger::ACAmps;
float MGgen2V2Lcharger::DCAmps;
float MGgen2V2Lcharger::LV_Volts;
float MGgen2V2Lcharger::LV_Amps;
uint16_t MGgen2V2Lcharger::batteryVolts;
static uint8_t PlugStat = 0;

bool MGgen2V2Lcharger::ControlCharge(bool RunCh, bool ACReq) {
  int chgmode = Param::GetInt(Param::interface);
  switch (chgmode) {
  case Unused:
    if (PlugStat == 1 && ACReq) {
      clearToStart = true;
      return true;
    } else {
      clearToStart = false;
      return false;
    }

    break;

  case i3LIM:
    if (RunCh &&
        ACReq) // we have a startup request to AC charge from a charge interface
    {
      clearToStart = true;
      return true;
    } else {
      clearToStart = false;
      return false;
    }
    break;

  case CPC:
    if (RunCh &&
        ACReq) // we have a startup request to AC charge from a charge interface
    {
      clearToStart = true;
      return true;
    } else {
      clearToStart = false;
      return false;
    }
    break;

  case Foccci:
    if (RunCh &&
        ACReq) // we have a startup request to AC charge from a charge interface
    {
      clearToStart = true;
      return true;
    } else {
      clearToStart = false;
      return false;
    }
    break;

  case Chademo:
    if (RunCh && ACReq) {
      clearToStart = true;
      return true;
    } else {
      clearToStart = false;
      return false;
    }

    break;
  }
  return false;
}

void MGgen2V2Lcharger::SetCanInterface(CanHardware *c) {

  can = c;
  can->RegisterUserMessage(0x324);
  can->RegisterUserMessage(0x39F);
  can->RegisterUserMessage(0x323);
  // can->RegisterUserMessage(0x326);//test
}

void MGgen2V2Lcharger::DecodeCAN(int id, uint32_t data[2]) {
  switch (id) {
  case 0x324:
    MGgen2V2Lcharger::handle324(data);
    break;
  case 0x39F:
    MGgen2V2Lcharger::handle39F(data);
    break;
  case 0x323:
    MGgen2V2Lcharger::handle323(data);
    break;
    // case 0x38A:
    //    MGgen2V2Lcharger::handle38A(data);
    //    break;
  }
}

void MGgen2V2Lcharger::Task100Ms() {
  int opmode = Param::GetInt(Param::opmode);
  uint8_t bytes[8];
  if (opmode == MOD_RUN) // do some DC-DC stuff
  {
    bytes[0] = 0x06;
    bytes[1] = 0xA0;
    bytes[2] = 0x26; // 26 for on, 06 for off
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x7F;
    can->Send(0x19C, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x03; // 01 is stand by, 03 is driving, 06 is AC charging, 07 is
                     // CCS charging
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x20; // 20 to wake up charger.
    bytes[7] = 0x00;
    can->Send(0x297, (uint32_t *)bytes, 8); // 297 is BMS state

    bytes[0] = 0x0E; // 0E to wake up
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x20;
    bytes[7] = 0x00;
    can->Send(0x1F1, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x28;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x46;                        // 46 start V2l? or 48
    can->Send(0x33F, (uint32_t *)bytes, 8); // V2L

    bytes[0] = 0x3B;
    bytes[1] = 0xCA;
    bytes[2] = 0x86;
    bytes[3] = 0x02;
    bytes[4] = 0xFD;
    bytes[5] = 0x60;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x29B, (uint32_t *)bytes, 8);
  }

  if (opmode == MOD_CHARGE) {

    setVolts = Param::GetInt(Param::Voltspnt) * 10;
    actVolts = Param::GetInt(Param::udc);
    /*
    bytes[0] = 0x06;
    bytes[1] = 0xA0;
    bytes[2] = 0x26; //26 for on, 06 for off
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x7F;
    can->Send(0x19C, (uint32_t*)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x06; // 01 is stand by, 03 is driving, 06 is AC charging, 07 is
    CCS charging bytes[2] = 0x00; bytes[3] = 0x00; bytes[4] = 0x00; bytes[5] =
    0x00; bytes[6] = 0x20; // 20 to wake up charger. bytes[7] = 0x00;
    can->Send(0x297, (uint32_t*)bytes, 8); // 297 is BMS state
*/
    bytes[0] = 0x0E; // 0E to wake up
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x20;
    bytes[7] = 0x00;
    can->Send(0x1F1, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x06; // 01 is stand by, 03 is driving, 06 is AC charging, 07 is
                     // CCS charging
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x20;
    bytes[6] = 0x20; // 20 to wake up charger.
    bytes[7] = 0x00;
    can->Send(0x297, (uint32_t *)bytes, 8);

    bytes[0] = 0x3B;
    bytes[1] = 0xCA;
    bytes[2] = 0x86;
    bytes[3] = 0x00;
    bytes[4] = 0xFD;
    bytes[5] = 0x60;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x29B, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x32E, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x343, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x348, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = currentRamp;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = setVolts >> 8;
    bytes[5] = setVolts & 0xff;
    bytes[6] = 0x00;
    bytes[7] = 0x00;

    can->Send(0x394, (uint32_t *)bytes, 8);

    bytes[0] = 0x44;
    bytes[1] = 0x6E;
    bytes[2] = 0xB4;
    bytes[3] = 0x28;
    bytes[4] = 0x80;
    bytes[5] = 0x4E;
    bytes[6] = 0x4E;
    bytes[7] = 0x4D;
    can->Send(0x396, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x43;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0xCD;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x39A, (uint32_t *)bytes, 8);

    bytes[0] = 0x44;
    bytes[1] = 0x43;
    bytes[2] = 0x9D;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x39B, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x28;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x41;
    can->Send(0x33F, (uint32_t *)bytes, 8);

    bytes[0] = 0x06;
    bytes[1] = 0xA0;
    bytes[2] = 0x26;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x7F;
    can->Send(0x19C, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x20;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x322, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x28;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x10;
    bytes[5] = 0x43;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x394, (uint32_t *)bytes, 8);
  }
  if (clearToStart) {
    if (actVolts < Param::GetInt(Param::Voltspnt))
      currentRamp++;
    if (actVolts >= Param::GetInt(Param::Voltspnt))
      currentRamp--;
    if (currentRamp >= 0x28)
      currentRamp = 0x28; // clamp to max of 40A

    bytes[0] = 0x28;
    bytes[1] = 0x89;
    bytes[2] = 0x07;
    bytes[3] = 0xFE;
    bytes[4] = 0x00;
    bytes[5] = 0xDC;
    bytes[6] = 0x57;
    bytes[7] = 0x12;
    can->Send(0x29C, (uint32_t *)bytes, 8);

  } else {

    bytes[0] = 0x28;
    bytes[1] = 0xFF;
    bytes[2] = 0x83;
    bytes[3] = 0xFF;
    bytes[4] = 0x00;
    bytes[5] = 0xFF;
    bytes[6] = 0x7F;
    bytes[7] = 0xFF;
    can->Send(0x29C, (uint32_t *)bytes, 8);
  }
}

void MGgen2V2Lcharger::Off() {
  uint8_t bytes[8];
  bytes[0] = 0x26;
  bytes[1] = 0xA0;
  bytes[2] = 0x06; // 26 for on, 06 for off
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x7F;
  can->Send(0x19C, (uint32_t *)bytes, 8);

  bytes[0] = 0x00;
  bytes[1] = 0x01;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00; // 20 to wake up charger.
  bytes[7] = 0x00;
  can->Send(0x297, (uint32_t *)bytes, 8);

  bytes[0] = 0x28;
  bytes[1] = 0xFF;
  bytes[2] = 0x83;
  bytes[3] = 0xFF;
  bytes[4] = 0x00;
  bytes[5] = 0xFF;
  bytes[6] = 0x7F;
  bytes[7] = 0xFF;
  can->Send(0x29C, (uint32_t *)bytes, 8);

  bytes[0] = 0x00; // 0E to wake up
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x20;
  bytes[7] = 0x00;
  can->Send(0x1F1, (uint32_t *)bytes, 8);

  bytes[0] = 0x00;
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x28;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;                        //
  can->Send(0x33F, (uint32_t *)bytes, 8); // V2L

  bytes[0] = 0x00;
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x20;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x322, (uint32_t *)bytes, 8);

  bytes[0] = 0x00;
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x28;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x41;
  can->Send(0x33F, (uint32_t *)bytes, 8);

  bytes[0] = 0x44;
  bytes[1] = 0x43;
  bytes[2] = 0x9D;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x39B, (uint32_t *)bytes, 8);

  bytes[0] = 0x00;
  bytes[1] = 0x43;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0xCD;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x39A, (uint32_t *)bytes, 8);

  bytes[0] = 0x44;
  bytes[1] = 0x6E;
  bytes[2] = 0xB4;
  bytes[3] = 0x28;
  bytes[4] = 0x80;
  bytes[5] = 0x4E;
  bytes[6] = 0x4E;
  bytes[7] = 0x4D;
  can->Send(0x396, (uint32_t *)bytes, 8);

  bytes[0] = 0x00;
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x343, (uint32_t *)bytes, 8);

  bytes[0] = 0x00;
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x32E, (uint32_t *)bytes, 8);

  bytes[0] = 0x3B;
  bytes[1] = 0xCA;
  bytes[2] = 0x86;
  bytes[3] = 0x00;
  bytes[4] = 0xFD;
  bytes[5] = 0x60;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x29B, (uint32_t *)bytes, 8);
}

void MGgen2V2Lcharger::handle324(uint32_t data[2])

{
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
                       // See comments are useful:)
  batteryVolts = ((bytes[1] << 8) | (bytes[2])) * 0.02;
  ;

  if (Param::GetInt(Param::ShuntType) == 0 &&
      Param::GetInt(Param::Inverter) !=
          1) // Only populate if no shunt is used and not using Leaf inverter
             // !!!look to clean up
  {
    // Param::SetFloat(Param::udc, batteryVolts);
  }
}

void MGgen2V2Lcharger::handle39F(uint32_t data[2])

{
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
                       // See comments are useful:)
  LV_Volts = bytes[1] / 8;
  LV_Amps = bytes[4];
  Param::SetFloat(Param::U12V, LV_Volts);
  Param::SetFloat(Param::I12V, LV_Amps);
}

void MGgen2V2Lcharger::handle323(uint32_t data[2]) {

  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
                       // See comments are useful:
  PlugStat = bytes[5];
}
/*
void MGgen2V2Lcharger::handle38A(uint32_t data[2])

{

}

*/
