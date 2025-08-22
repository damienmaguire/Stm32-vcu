/*
 * This file is part of the Zombieverter VCU project.
 *
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
 * https://openinverter.org/wiki/Volkswagen_Heater
 *
 * ID : 28 (dec) Command. Length 2. Byte 0 : Power 0-2530W scale x10. , Byte 1 :
 * Last bit: Start/Stop
 *
 * ID : 48 (dec) Response. Length 8. Byte 0 : Power 13=770W , 26 = 1540W. Byte 6
 * Temp in , Byte 7 Temp out.
 *
 * Observed return data on 0x30 : 0x00 , 0xFF, 0xE7, 0x80, 0x8D, 0x39, 0x34,
 * 0x34 8 bytes Command was : 0x5A, 0x01, 0x00 , 0x00 Observed data with heater
 * running approx 4kw : 0x32, 0xFC, 0x80, 0x80, 0x8D, 0x5D, 0x7A, 0x77
 */

#include <VWheater.h>

static uint8_t processedPower = 0;
static uint8_t TenCount = 0;

void vwHeater::SetLinInterface(LinBus *l) {
  lin = l;
  DigIo::lin_wake.Clear(); // Not used on TJA1027
  DigIo::lin_nslp.Set();   // Wakes the device
  // Johannes for president!
}

void vwHeater::SetPower(uint16_t power, bool HeatReq) {
  TenCount++;
  if (TenCount == 5) // slow down to 50ms as this is called in 10ms task.
  {
    TenCount = 0;
    HeatReq = HeatReq;
    // going to ignore heatreq just for test.

    if (power >= 255)
      power = 255;          // Constrain power to max for VW heater.
    processedPower = power; // 10;

    static bool read = true;

    if (lin->HasReceived(48, 8)) // 0x30 hex address
    {
      uint8_t *data = lin->GetReceivedBytes();

      Param::SetFloat(Param::tmpheater,
                      data[6] -
                          47); // Looks like the temp val has an offset prob for
                               // neg vals. Need to get it more accurate
      Param::SetFloat(
          Param::udcheater,
          data[4] / 10); // 0x8D = 141 dec /10 =14.1V ? 12v system voltage.
      Param::SetFloat(Param::powerheater,
                      data[0] *
                          80); // actual power used by heater. again, needs some
                               // more experiments to get more accurate
    }

    if (read) {
      lin->Request(48, 0, 0); // 0x30 hex address
    } else {
      uint8_t lindata[4];
      lindata[0] = Param::GetInt(
          Param::HeatPercnt); // VW heater uses a % setting as opposed to a set
                              // power val. Regulates its temps to this.
      lindata[1] = 1;         // Always on for test. Can use heatreq here.
      lindata[2] = 0;
      lindata[3] = 0;
      lin->Request(28, lindata, sizeof(lindata)); // 0x1C hex address
    }

    read = !read;
  }
}
