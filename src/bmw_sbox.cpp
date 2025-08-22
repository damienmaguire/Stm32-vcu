/*   * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2019-2022 Damien Maguire <info@evbmw.com>
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

#include <bmw_sbox.h>

/*
 * Implements control of the contactors in the BMW PHEV battery box "SBOX" unit.
 * Reads batter parameters from SBOX.
 * See : https://github.com/damienmaguire/BMW_SBox
 */

int32_t SBOX::Amperes;
int32_t SBOX::Ah;
int32_t SBOX::KW;
int32_t SBOX::KWh;
int32_t SBOX::Voltage = 0;
int32_t SBOX::Voltage2 = 0;
int32_t SBOX::Temperature;
uint8_t canCtr100 = 0;
uint8_t CCByte = 0;
uint8_t CRCByte = 0;
uint8_t Timer20ms = 0;

// BMW(Maxim crc poly 0x31) CRC 8 lookup table
uint8_t crc_array[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20,
    0xa3, 0xfd, 0x1f, 0x41, 0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23, 0x7d, 0x9f, 0xc1,
    0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e,
    0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67, 0x39,
    0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45,
    0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 0x8c, 0xd2, 0x30, 0x6e,
    0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31,
    0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
    0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e, 0xd0,
    0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea,
    0x69, 0x37, 0xd5, 0x8b, 0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b,
    0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54,
    0xd7, 0x89, 0x6b, 0x35,
};

uint8_t BMW_crc8(const uint8_t *data, const uint16_t size) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < size; ++i) {
    crc = crc_array[data[i] ^ crc];
  }
  return crc;
}

void SBOX::RegisterCanMessages(CanHardware *can) {
  can->RegisterUserMessage(0x200); // SBOX MSG
  can->RegisterUserMessage(0x210); // SBOX MSG
  can->RegisterUserMessage(0x220); // SBOX MSG
}

void SBOX::DecodeCAN(int id, uint32_t data[2]) {
  switch (id) {
  case 0x200:
    SBOX::handle200(data); // SBOX CAN MESSAGE
    break;
  case 0x210:
    SBOX::handle210(data); // SBOX CAN MESSAGE
    break;
  case 0x220:
    SBOX::handle220(data); // SBOX CAN MESSAGE
    break;
  }
}

void SBOX::handle200(uint32_t data[2]) // SBOX Current

{
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
                       // See comments are useful:)
  Amperes = ((bytes[2] << 16) | (bytes[1] << 8) | (bytes[0]));
  Amperes = (Amperes << 8) >> 8; // extend sign bit as its a 24 bit signed value
                                 // in a 32bit int! AAAHHHHHH!
}

void SBOX::handle210(uint32_t data[2]) // SBOX battery voltage

{
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
                       // See comments are useful:)
  Voltage = ((bytes[2] << 16) | (bytes[1] << 8) | (bytes[0]));
  Voltage = (Voltage << 8) >> 8; // extend sign bit as its a 24 bit signed value
                                 // in a 32bit int! AAAHHHHHH!
}

void SBOX::handle220(uint32_t data[2]) // SBOX Output voltage

{
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
                       // See comments are useful:)
  Voltage2 = ((bytes[2] << 16) | (bytes[1] << 8) | (bytes[0]));
  Voltage2 = (Voltage2 << 8) >> 8; // extend sign bit as its a 24 bit signed
                                   // value in a 32bit int! AAAHHHHHH!
}

void SBOX::ControlContactors(int opmode, CanHardware *can) {
  uint8_t bytes[8];
  bytes[0] = 0xFF; // sems to control the iso relay
  bytes[1] = 0xFE; // needs to be 0xFE to enable contactors.
  bytes[2] = 0xFF;
  bytes[3] = 0xFF;
  can->Send(0x300, (uint32_t *)bytes, 4);

  bytes[0] = CCByte;
  bytes[1] = (canCtr100 << 4 | 0x1);
  bytes[2] = 0xFF;
  bytes[3] = 0x00; // crc
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;

  canCtr100++;
  if (canCtr100 > 0xE)
    canCtr100 = 0;

  switch (opmode) {
  case 0:
    CCByte = 0x00; // all contactors off
    break;
  case 2:          // Precharge
    CCByte = 0xA6; // Prech and Neg contactors activated
    break;

  case 1:          // Run
    CCByte = 0xAA; // All contactors activated
    break;

  case 4:          // Charge
    CCByte = 0xAA; // All contactors activated
    break;

  case 3:          // Precharge fail
    CCByte = 0x00; // all contactors off
    break;
  }
  CRCByte = BMW_crc8(bytes, 8);
  bytes[3] = CRCByte; // crc
  can->Send(0x100, (uint32_t *)bytes, 4);
}
