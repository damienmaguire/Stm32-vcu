/*
 * This file is part of the Zombieverter VCU project.
 *
 * Copyright (C) 2024 Tom de Bree <tom@voltinflux.com>
 * Copyright (C) 2018 Johannes Huebner <dev@johanneshuebner.com>
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
 * Controls the VW LIN based heater as :
 * https://openinverter.org/wiki/VAG_PTC_Air_Heater
 *
 * Based on reverse engineer work by Tom de Bree
 *
 */

#include <VWAirHeater.h>

static uint8_t processedPower = 0;
static uint8_t TenCount = 0;

void vwAirHeater::SetLinInterface(LinBus *l) {
  lin = l;
  DigIo::lin_wake.Clear(); // Not used on TJA1027
  DigIo::lin_nslp.Set();   // Wakes the device
}

void vwAirHeater::SetPower(uint16_t power, bool HeatReq) {
  TenCount++;
  if (TenCount == 5) // slow down to 50ms as this is called in 10ms task.
  {
    TenCount = 0;
    HeatReq = HeatReq;
    // going to ignore heatreq here, already used to determine to run this
    // sequence.

    if (power >= 255)
      power = 255;          // Constrain power to max for VW heater.
    processedPower = power; // 10;

    static bool read = true;

    if (lin->HasReceived(19, 8)) // 0x13 hex address
    {
      uint8_t *data = lin->GetReceivedBytes();

      Param::SetFloat(Param::tmpheater,
                      data[5] * 0.25); // Looks like the temp feedback 1
      Param::SetFloat(Param::udcheater, data[1] * 2); // HV Voltage feedback
      Param::SetFloat(Param::powerheater,
                      data[0] *
                          data[1]); // HV current is Byte 0 in 0.5A per/1 and HV
                                    // voltage is byte 1 2V/1 so they cancel
    }

    if (read) {
      lin->Request(19, 0, 0); // 0x13 hex address
    }

    else {
      uint8_t PowReq = Param::GetInt(
          Param::HeatPercnt); // VW heater uses a % setting as opposed to a set
                              // power val. Regulates its temps to this.
      uint8_t lindata[8];
      lindata[0] = 0x39;   // fixed
      lindata[1] = 0xC5;   // fixed
      lindata[2] = PowReq; // Left Request 0-100, tested even at 120 to confirm
                           // max values
      lindata[3] = PowReq; // Right Request 0-100, tested even at 120 to confirm
                           // max values
      lindata[4] = 0;      // fixed
      lindata[5] = 0xAA;   // fixed
      lindata[6] = 0x78;   // Power Limit? 0x50 and 0x78 tested
      lindata[7] = 0;      // fixed
      lin->Request(38, lindata, sizeof(lindata)); // 0x26 hex address
    }

    read = !read;
  }
}
