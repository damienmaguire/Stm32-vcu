/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2026 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef C5PTECAN_H
#define C5PTECAN_H

#include <stdint.h>

namespace C5PTECAN {
constexpr int BIT_INDEX_MASK = 0x7;

inline int NextMotorolaLsbBit(int bit) {
  return (bit & BIT_INDEX_MASK) == BIT_INDEX_MASK ? bit - 15 : bit + 1;
}

inline void PackMotorolaLsb(uint8_t *bytes, int startBit, int length,
                            uint32_t value) {
  int bit = startBit;

  for (int i = 0; i < length; i++) {
    const uint8_t mask = 1U << (bit & BIT_INDEX_MASK);

    if (value & (1UL << i))
      bytes[bit / 8] |= mask;
    else
      bytes[bit / 8] &= (uint8_t)~mask;

    bit = NextMotorolaLsbBit(bit);
  }
}

inline uint32_t UnpackMotorolaLsb(const uint8_t *bytes, int startBit,
                                  int length) {
  uint32_t value = 0;
  int bit = startBit;

  for (int i = 0; i < length; i++) {
    const uint8_t mask = 1U << (bit & BIT_INDEX_MASK);

    if (bytes[bit / 8] & mask)
      value |= 1UL << i;

    bit = NextMotorolaLsbBit(bit);
  }

  return value;
}

// SAE J1850 CRC-8 variant with polynomial 0x1D, init 0x00, final xor 0x00.
inline uint8_t Crc8SaeJ1850Zero(const uint8_t *data, int length) {
  uint8_t crc = 0x00;

  for (int i = 0; i < length; i++) {
    crc ^= data[i];

    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x80)
        crc = (uint8_t)((crc << 1) ^ 0x1D);
      else
        crc <<= 1;
    }
  }

  return crc;
}

inline void FinalizeE2EFrame(uint8_t *bytes, uint8_t counter, int counterBit) {
  PackMotorolaLsb(bytes, counterBit, 4, counter & 0xF);
  bytes[0] = Crc8SaeJ1850Zero(bytes + 1, 7);
}

} // namespace C5PTECAN

#endif // C5PTECAN_H
