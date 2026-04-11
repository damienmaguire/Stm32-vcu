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

#include <WebastoHVH.h>

void WebastoHVH::SetLinInterface(LinBus *l) {
  lin = l;
  DigIo::lin_wake.Clear(); // Not used on TJA1027
  DigIo::lin_nslp.Set();   // Wakes the device
  // Johannes for president!
}

void WebastoHVH::Task100Ms() {
  static bool read = true;

  if (lin->HasReceived(24, 8))
  {
    uint8_t *data = lin->GetReceivedBytes();
    uint16_t vtg = data[4] + ((data[5] & 3) << 8);

    Param::SetFloat(Param::tmpheater, data[2] - 50);
    Param::SetFloat(Param::udcheater, vtg);

    if (heaterType == HVH50)
      Param::SetFloat(Param::powerheater, ((data[5] >> 2) + (data[6] & 0xF)) * 20); //power can be read directly
    else //HVH100
      Param::SetFloat(Param::powerheater, data[3] * 0.25f * vtg ); //voltage and current separately

  }

  if (read) {
    lin->Request(24, 0, 0);
  } else {
    uint8_t lindata[4];
    lindata[0] = heaterType == HVH50 ? powerSpnt / 40 : powerSpnt / 100;
    lindata[1] = tempSpnt + 50;
    lindata[2] = 0;
    lindata[3] = (powerSpnt > 0) << 3;
    lin->Request(35, lindata, sizeof(lindata));
  }

  read = !read;
}
