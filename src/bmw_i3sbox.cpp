/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2026 Damien Maguire <info@evbmw.com>
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

#include "bmw_i3sbox.h"

/*
 * BMW i3 SBOX Local-CAN at 500 kbit/s.
 * 0x100 pack V    u16le(B0,B1) mV
 * 0x110 output V  u16le(B0,B1) mV   (bench still caps ~17.5 V — confirm)
 * 0x130 current   i16le(B0,B1) mA, discard if B5 == 0x80
 * B4 high nibble is the 0-F alive counter on the 2 ms frames.
 * No TX. Contactors are GPIO.
 */

int32_t I3SBOX::Voltage = 0;
int32_t I3SBOX::Voltage2 = 0;
int32_t I3SBOX::Amperes = 0;
uint8_t I3SBOX::Alive = 0;
uint8_t I3SBOX::Flags = 0;
bool I3SBOX::ValidCurrent = false;

void I3SBOX::RegisterCanMessages(CanHardware *can) {
  can->RegisterUserMessage(0x100);
  can->RegisterUserMessage(0x110);
  can->RegisterUserMessage(0x130);
}

void I3SBOX::DecodeCAN(int id, uint32_t data[2]) {
  switch (id) {
  case 0x100:
    handle100(data);
    break;
  case 0x110:
    handle110(data);
    break;
  case 0x130:
    handle130(data);
    break;
  default:
    break;
  }
}

void I3SBOX::handle100(uint32_t data[2]) {
  uint8_t *bytes = (uint8_t *)data;
  Voltage = (int32_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
  Alive = (uint8_t)(bytes[4] >> 4);
}

void I3SBOX::handle110(uint32_t data[2]) {
  uint8_t *bytes = (uint8_t *)data;
  Voltage2 = (int32_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

void I3SBOX::handle130(uint32_t data[2]) {
  uint8_t *bytes = (uint8_t *)data;
  Flags = bytes[5];
  if (Flags == 0x80) {
    ValidCurrent = false;
    return;
  }
  int16_t raw = (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
  Amperes = (int32_t)raw;
  ValidCurrent = (Flags == 0x00);
}
