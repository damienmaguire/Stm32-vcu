/*
 * This file is part of the Zombieverter project.
 *
 * Copyright (C) 2024 Mitch Elliott
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

#include "Gear_9x1.h"

uint8_t Gear9x1LeverPos = 0x78; // Gear Lever Position on Init
uint8_t Gear9x1TipSport = 0; // Tip +/- Sport Postion
uint8_t Gear9x1Checksum = 0; // Checksum Value for Plausibilty
uint8_t Gear9x1Counter = 0; // Counter Value

void Gear_9x1::SetCanInterface(CanHardware* c)
{
   can = c;
   can->RegisterUserMessage(0x133);// 9x1 based shifter CAN message: WHL_01
}


void Gear_9x1::DecodeCAN(int id, uint32_t* data)
{
  uint8_t *bytes = (uint8_t *)data;
  if (id == 0x133)
  {
    Gear9x1LeverPos = bytes[0];
    Gear9x1Counter = (bytes[1] >> 4);       // Mapping for later to determine signal validity
    Gear9x1TipSport = (bytes[1] & (0x0FU)); // +/- sport movement
    Gear9x1Checksum = bytes[6];             // Mapping for later to determine signal validity

    switch (Gear9x1LeverPos)
    {
    case 0x78: // Park
    {
      this->gear = PARK;
    }
    break;
    case 0x87: // Reverse
    {
      this->gear = REVERSE;
    }
    break;
    case 0x96: // Neutral
    {
      this->gear = NEUTRAL;
    }
    break;
    case 0xA5: // Drive
    {
      this->gear = DRIVE;
    }
    break;
    case 0x1E: // Manual/ Tip
    {
      this->gear = DRIVE;
    }
    break;
    }
  }
}

 void Gear_9x1::Task10Ms()
 {

 }

 void Gear_9x1::Task100Ms()
 {

 }

 bool Gear_9x1::GetGear(Shifter::Sgear& outGear)
{
   outGear = gear;    //send the shifter pos
   return true; //Let caller know we set a valid gear
}
