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

#ifndef BMW_I3SBOX_H
#define BMW_I3SBOX_H

/* BMW i3 pack-internal SBOX (PN 61278648904) Local-CAN decode.
 * Measurement only. Coils stay on VCU GPIO.
 * Voltages are i32le millivolts on B0-B3 (B2 ticks at 65.536 V).
 * Not the PHEV SBOX (ShuntType=2, IDs 0x100/0x300 as coil commands).
 * https://github.com/damienmaguire/BMW-i3-SBOX
 */

#include "canhardware.h"
#include <stdint.h>

class I3SBOX {
  I3SBOX();
  ~I3SBOX();

public:
  static void RegisterCanMessages(CanHardware *can);
  static void DecodeCAN(int id, uint32_t data[2]);

  static int32_t Voltage;  // pack / battery-side, mV (0x100 i32le)
  static int32_t Voltage2; // vehicle / output-side, mV (0x110 i32le)
  static int32_t Amperes;  // pack current, mA (0x130), last valid frame
  static uint8_t Alive;    // 0x100 B4 high nibble
  static uint8_t Flags;    // last 0x130 B5
  static bool ValidCurrent;

private:
  static void handle100(uint32_t data[2]);
  static void handle110(uint32_t data[2]);
  static void handle130(uint32_t data[2]);
};

#endif
