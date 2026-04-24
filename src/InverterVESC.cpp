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

#include "InverterVESC.h"
#include "my_math.h"
#include "params.h"

void InverterVESC::SetCanInterface(CanHardware *c) {
  can = c;

  can->RegisterUserMessage(0x901);//ID 1 Status 1
  can->RegisterUserMessage(0x1001);//ID 1 Status 4
  can->RegisterUserMessage(0x1B01);//ID 1 Status 5
  can->RegisterUserMessage(0x281);//ID 1 Status 6
}

void InverterVESC::DecodeCAN(int id, uint32_t *wdata) {
  uint8_t *data = (uint8_t *)wdata;
  if (id == 0x901) {
    inv_dc = (int16_t)((data[6] << 8) + data[7]);
    inv_cur = (int16_t)((data[4] << 8) + data[5]);
    speed =
      ((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3]));
  } else if (id == 0x1001) {
    motor_temp = (int16_t)((data[0] << 8) + data[1]) * 0.1;
    inv_temp = (int16_t)((data[2] << 8) + data[3]) * 0.1;
  } else if (id == 0x1B01) {
    voltage = (int16_t)((data[4] << 8) + data[5]) * 0.1;
  }
}

void InverterVESC::SetTorque(float torquePercent) {
  uint8_t data[8];
  int opmode = Param::GetInt(Param::opmode);

  if (opmode == MOD_RUN) {
    int32_t torque = torquePercent * 1000;
    data[0] = (torque >> 24) & 0xFF;
    data[1] = (torque >> 16) & 0xFF;
    data[2] = (torque >> 8) & 0xFF; // Big endian
    data[3] = torque & 0xFF; // Big endian

    Param::SetInt(Param::torque,torque); // post processed final torque value sent
                                         // to inv to web interface

    can->Send(0x1, data, 4, true); //ID 1
  }
}

void InverterVESC::Task100Ms() {}
