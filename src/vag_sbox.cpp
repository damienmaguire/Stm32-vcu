/*
 * This file is part of the ZombieVerter project.
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

 *Implements control of the contactors in the VW PHEV battery box unit.
 *Reads batter parameters from contactor box.
 *See : https://github.com/jamiejones85/VW-GTE-ContactorBox/tree/main
*/

#include <vag_sbox.h>

int16_t VWBOX::Amperes;
int32_t VWBOX::Ah;
int32_t VWBOX::KW;
int32_t VWBOX::KWh;
float VWBOX::Voltage = 0;
float VWBOX::Voltage2 = 0;
int32_t VWBOX::Temperature;
uint8_t Sec1tmr = 0;
uint8_t vag_cnt0ba = 0;
int16_t q12;

void VWBOX::RegisterCanMessages(CanHardware *can) {
  can->RegisterUserMessage(0x0BB); // VWBOX MSG
}

void VWBOX::DecodeCAN(int id, uint32_t data[2]) {
  switch (id) {
  case 0x0BB:
    VWBOX::handle0BB(data); // VWBOX CAN MESSAGE
    break;
  }
}

void VWBOX::handle0BB(uint32_t data[2]) // VWBOX Current and voltages

{
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
                       // See comments are useful:)
  q12 = (((bytes[2]) << 4) | ((bytes[1]) >> 4));
  Amperes = (q12 & 0x0800) ? (q12 | 0xF800)
                           : q12; // Step1: Copy , Step2: Paste , Step3: Profit!
  Voltage = ((bytes[5] << 4) | ((bytes[4] >> 4) & 0xF)); // output voltage
  Voltage2 = (((bytes[4] & 0xF) << 8) | (bytes[3]));     // battery voltage
}

void VWBOX::ControlContactors(int opmode, CanHardware *can) {
  uint8_t bytes[8];
  Sec1tmr++;
  if (Sec1tmr == 100) {
    bytes[0] = 0x00;
    bytes[1] = 0x00;
    can->Send(0x1BFFDA19, (uint32_t *)bytes, 2);
    Sec1tmr = 0;
  }

  bytes[0] = 0x00;
  bytes[1] = vag_cnt0ba;
  bytes[2] = 0x28;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x26;

  switch (opmode) {
  case 0:
    bytes[2] = 0x28; // all contactors off
    bytes[1] = bytes[1] | 0x00;
    break;
  case 2: // Precharge
    bytes[2] = bytes[2] | 0x1;
    ;                           // Neg on
    bytes[1] = bytes[1] | 0x10; // Prech on
    break;

  case 1: // Run
    bytes[2] = bytes[2] | 0x1;
    ;                           // Neg on
    bytes[1] = bytes[1] | 0x50; // main on and prech on
    break;

  case 4: // Charge
    bytes[2] = bytes[2] | 0x1;
    ;                           // Neg on
    bytes[1] = bytes[1] | 0x50; // main on and prech on
    break;

  case 3:            // Precharge fail
    bytes[2] = 0x28; // all contactors off
    bytes[1] = bytes[1] | 0x00;
    break;
  }
  bytes[0] = vw_crc_calc(bytes);
  can->Send(0x0BA, (uint32_t *)bytes, 8);
  vag_cnt0ba++;
  if (vag_cnt0ba > 0x0F)
    vag_cnt0ba = 0x00;
}

uint8_t VWBOX::vw_crc_calc(
    uint8_t *data) // just works on 0x0ba here. will expand it to others TODO.
{

  const uint8_t poly = 0x2F;
  const uint8_t xor_output = 0xFF;
  // VAG Magic Bytes
  const uint8_t MB00BA[16] = {0x6C, 0xAA, 0x01, 0xCF, 0x39, 0x38, 0xDF, 0x4F,
                              0x13, 0x2A, 0x73, 0x8C, 0xF1, 0x76, 0xF6, 0x70};

  uint8_t crc = 0xFF;
  uint8_t magicByte = 0x00;
  uint8_t counter =
      data[1] & 0x0F; // only the low byte of the couner is relevant

  magicByte = MB00BA[counter];

  for (uint8_t i = 1; i < 8 + 1; i++) {
    // We skip the empty CRC position and start at the timer
    // The last element is the VAG magic byte for address 0x187 depending on the
    // counter value.
    if (i < 8)
      crc ^= data[i];
    else
      crc ^= magicByte;

    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ poly;
      else
        crc = (crc << 1);
    }
  }

  crc ^= xor_output;

  return crc; // set the CRC checksum directly in the output bytes
}
