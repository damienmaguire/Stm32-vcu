/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2023 Damien Maguire
 * Ben Bament
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

#include <ElconDCDC.h>

// This is an interface for Elcon Chargers that require DC-DC messages, based
// heavily on the Tesla DCDC module.

void ElconDCDC::SetCanInterface(CanHardware *c) {
  can = c;
  can->RegisterUserMessage(0x1801D08F);
}

// Process voltage , current and temperature message from the Elcon
// Charger/DCDC.
void ElconDCDC::DecodeCAN(int id, uint8_t *data) {
  if (id == 0x1801D08F) {
    Param::SetFloat(
        Param::U12V,
        (data[0] * 256 + data[1]) *
            0.1); // Display 12v system voltage as read from the dcdc
    Param::SetFloat(
        Param::I12V,
        (data[2] * 256 + data[3]) *
            0.1); // Display 12v system current as read from the dcdc
    Param::SetFloat(Param::ChgTemp, data[7] - 40); // Display dcdc coolant temp
  }
}

void ElconDCDC::Task100Ms() {

  int opmode = Param::GetInt(Param::opmode);
  uint8_t bytes[8];

  if (opmode == MOD_RUN || opmode == MOD_CHARGE) {
    timer200++;
    if (timer200 == 2) {

      bytes[0] = 0x01; // 1 to activate DC-DC
      can->Send(0x18008FD0, bytes, 8);
      timer200 = 0;
    }
  }
}
