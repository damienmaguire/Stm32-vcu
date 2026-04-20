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

#include "InverterACP.h"
#include "my_math.h"
#include "params.h"

#define ACP_DRIVE_ENABLE 0x2
#define ACP_CONT_CLOSED 0x4
#define ACP_MODE_TORQUE 0x8
#define ACP_STATE_DRIVE 0x3

void InverterACP::SetCanInterface(CanHardware *c) {
  can = c;

  can->RegisterUserMessage(0x201);
  can->RegisterUserMessage(0x221);
  can->RegisterUserMessage(0x281);
}

void InverterACP::DecodeCAN(int id, uint32_t *wdata) {
  uint8_t *data = (uint8_t *)wdata;
  if (id == 0x201) {
    inv_flags = data[0] + (data[1] << 8);
    voltage = (float)((data[2] << 8) + data[3]);
  } else if (id == 0x221) {
    speed = (int16_t)((data[4] << 8) + data[5]);
  } else if (id == 0x281) {
    int16_t mtemp1 = ((int16_t)data[0]) - 40;
    int16_t mtemp2 = ((int16_t)data[1]) - 40;
    motor_temp = MAX(mtemp1, mtemp2);
    int16_t invtemp1 = ((int16_t)data[2]) - 40;
    int16_t invtemp2 = ((int16_t)data[3]) - 40;
    int16_t invtemp3 = ((int16_t)data[4]) - 40;
    inv_temp = MAX(invtemp1, MAX(invtemp2, invtemp3));
  }
}

void InverterACP::SetTorque(float torquePercent) {
  static bool toggleBit = false;
  uint8_t data[8];
  int opmode = Param::GetInt(Param::opmode);
  uint16_t udc = Param::GetInt(Param::udc);

  data[0] = toggleBit;
  data[6] = udc >> 8;
  data[7] = udc & 0xFF;
  toggleBit = !toggleBit;

  if (opmode == MOD_RUN) {
    int16_t torque = (int16_t)(10 * torquePercent);
    data[0] |= ACP_DRIVE_ENABLE | ACP_CONT_CLOSED | ACP_MODE_TORQUE;
    data[1] = torque >> 8;
    data[2] = torque & 0xFF; // Big endian
  }

  can->Send(0x100, data, 8);
}

int InverterACP::GetInverterState() {
  return (inv_flags & 0xF) == ACP_STATE_DRIVE;
}

void InverterACP::Task100Ms() {}
