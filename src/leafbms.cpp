/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2018 Johannes Huebner <dev@johanneshuebner.com>
 *               2024 Daniel Öster <info@dalasevrepair.fi>
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

#include "leafbms.h"
#include "my_fp.h"
#include "my_math.h"

#define ZE0_BATTERY 0  // 2011-2013 ZE0
#define AZE0_BATTERY 1 // 2013-2017 AZE0
#define ZE1_BATTERY 2  // 2018+ ZE1
static uint8_t LEAF_battery_Type = ZE0_BATTERY;
static int temperature = 0;

void LeafBMS::SetCanInterface(CanHardware *can) {
  can->RegisterUserMessage(0x1DB); // Leaf BMS message 10ms
  can->RegisterUserMessage(0x1DC); // Leaf BMS message 10ms
  can->RegisterUserMessage(0x55B); // Leaf BMS message 100ms
  can->RegisterUserMessage(0x5BC); // Leaf BMS message 100ms (500ms on ZE0)
  // can->RegisterUserMessage(0x5C0);//Leaf BMS message 500ms
  // can->RegisterUserMessage(0x59E);//Leaf BMS message 500ms (Only on AZE0)
  can->RegisterUserMessage(0x1C2); // Leaf BMS message 10ms (ZE1)
  can->RegisterUserMessage(0x1ED); // Leaf BMS message 10ms (ZE1, only on 62kWh)
}

void LeafBMS::DecodeCAN(int id, uint8_t *data) {
  uint8_t *bytes = (uint8_t *)data;

  switch (id) {
  case 0x1DB: {
    if (isMessageCorrupt(bytes)) {
      // Message content malformed, abort reading data from it! Raise flag!
      break;
    }
    float cur = uint16_t(bytes[0] << 3) + uint16_t(bytes[1] >> 5);
    if (cur > 1023)
      cur -= 2047; // check if negative
    uint16_t udc = uint16_t(bytes[2] << 2) + uint16_t(bytes[3] >> 6);
    // bool interlock = (bytes[3] & (1 << 3)) >> 3;
    // bool full = (bytes[3] & (1 << 4)) >> 4;

    if (Param::GetInt(Param::ShuntType) ==
        0) // Only populate if no shunt is used
    {
      float BattCur = cur / 2;
      float BattVoltage = udc / 2;
      Param::SetFloat(Param::idc, BattCur);
      if (BattVoltage < 450)
        Param::SetFloat(Param::udc2, BattVoltage);
      if (BattVoltage > 200)
        Param::SetFloat(Param::udcsw,
                        BattVoltage -
                            20); // Set for precharging based on actual voltage
      float kw =
          (BattVoltage * BattCur) /
          1000; // get power from isa sensor and post to parameter database
      Param::SetFloat(Param::power, kw);
    }
    break;
  }
  case 0x1DC: {
    if (isMessageCorrupt(bytes)) {
      // Message content malformed, abort reading data from it! Raise flag!
      break;
    }
    float dislimit = uint16_t(bytes[0] << 2) + uint16_t(bytes[1] >> 6);
    dislimit = dislimit * 0.25; // Kw discharge limit
    float chglimit = uint16_t((bytes[1] & 0x3F) << 4) + uint16_t(bytes[2] >> 4);
    chglimit = chglimit * 0.25; // Kw charge limit
    float chargelimit =
        uint16_t((bytes[2] & 0x0F) << 6) + uint16_t(bytes[3] >> 2);
    chargelimit = chargelimit * 0.1; // Kw charger limit

    chargelimit = chargelimit * 1000 /
                  Param::GetFloat(Param::udc2); // Transform into Amps
    // Param::SetFixed(Param::dislim, dislimit / 4);

    Param::SetFloat(Param::BMS_ChargeLim, chargelimit);
    Param::SetInt(Param::BMS_MaxInput, chglimit);
    Param::SetInt(Param::BMS_MaxOutput, dislimit);
    break;
  }
  case 0x55B: {
    if (isMessageCorrupt(bytes)) {
      // Message content malformed, abort reading data from it! Raise flag!
      break;
    }
    float soc = uint16_t(bytes[0] << 2) + uint16_t(bytes[1] >> 6);
    if (Param::GetInt(Param::ShuntType) ==
        0) // Only populate if no shunt is used
    {
      soc = soc * 0.1;
      Param::SetFloat(Param::SOC, soc);
    }

    uint16_t IsoTemp = uint16_t(bytes[4] << 2) + uint16_t(bytes[5] >> 6);

    Param::SetInt(Param::BMS_IsoMeas, IsoTemp);
    break;
  }
  case 0x5BC: {
    /*
    int soh = bytes[4] >> 1;
    int cond = (bytes[6] >> 5) + ((bytes[5] & 0x3) << 3);
    /nt limres = bytes[5] >> 5;

    //Param::SetInt(Param::limreason, limres);

    //Only acquire quick charge remaining time
    if (cond == 0)
    {
        int time = bytes[7] + ((bytes[6] & 0x1F) << 8);

        //Param::SetInt(Param::chgtime, time);
    }


    //Param::SetInt(Param::soh, soh);
    */
    // 0x5BC only contains average battery temperature on ZE0
    if (LEAF_battery_Type == ZE0_BATTERY) {
      temperature = (bytes[3] - 40);
      Param::SetInt(Param::BMS_Tavg, temperature);
    }
    break;
  }
  case 0x5C0: {
    // This temperature only works for 2013-2017 AZE0 LEAF packs, the mux is
    // different on other generations
    if (LEAF_battery_Type == AZE0_BATTERY) {
      if ((bytes[0] >> 6) == 1) { // Mux signalling MAX value
        temperature =
            ((bytes[2] / 2) - 40); // Effectively has only 7-bit precision,
                                   // bottom bit is always 0
        Param::SetInt(Param::BMS_Tavg, temperature);
      }
    }
    break;
  }
  case 0x59E: {
    // AZE0 2013-2017 or ZE1 2018-2023 battery detected
    // Only detect as AZE0 if not already set as ZE1
    if (LEAF_battery_Type != ZE1_BATTERY) {
      LEAF_battery_Type = AZE0_BATTERY;
    }
    break;
  }
  case 0x1C2: {
    // ZE1 2018-2023 battery detected!
    LEAF_battery_Type = ZE1_BATTERY;
    break;
  }
  case 0x1ED: {
    // ZE1 62kWh battery detected!
    LEAF_battery_Type = ZE1_BATTERY;
    break;
  }
  default:
    break;
  }
}

bool LeafBMS::isMessageCorrupt(uint8_t *data) {
  uint8_t crc = 0;
  uint8_t polynomial = 0x85;

  for (int b = 0; b < 8; b++) {
    uint8_t byte =
        (b == 7) ? 0 : data[b]; // Treat 8th byte as 0 during calculation.
    for (int i = 7; i >= 0; i--) {
      uint8_t bit = ((byte & (1 << i)) > 0) ? 1 : 0;
      if (crc >= 0x80)
        crc = (uint8_t)(((crc << 1) + bit) ^ polynomial);
      else
        crc = (uint8_t)((crc << 1) + bit);
    }
  }
  return crc != data[7];
}