/*
 * This file is part of the Zombieverter project.
 *
 * Copyright (C) 2026 ZF_8HP contributor
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The CRC8/0x1D table and algorithm are isomorphic to F30_Lever's
 * implementation (Damien Maguire, 2023). Per-ID xorout 0x04 was
 * determined by brute-forcing all (poly, init^xorout) pairs against
 * 553 captured 0x3F9 frames; 0x1D + 0x04 is the unique combination
 * that validates 100% of frames over bytes [1..7].
 *
 * The four observed gear-byte values map as:
 *   0x33 = Park    (confirmed: sole value in poweroff and idrive-buttons captures)
 *   0x31 = Neutral (transient between every other gear during rotation)
 *   0x32 = Reverse (hypothesis - needs ground-truth confirmation)
 *   0x35 = Drive   (hypothesis - needs ground-truth confirmation)
 *   0x34 = Sport?  (never observed; reserved)
 */

#include "iX4_Lever.h"

iX4_Lever::iX4_Lever()
    : gear(PARK), rawGearByte(0), leverEncoder(0),
      counterLow(0), prevCounter(0), prevCounterValid(false),
      crcFails(0), ctrFails(0)
{
    BuildCrcTable();
}

void iX4_Lever::BuildCrcTable()
{
    for (int dividend = 0; dividend < 256; ++dividend) {
        uint8_t r = (uint8_t)dividend;
        for (uint8_t bit = 8; bit > 0; --bit) {
            r = (r & 0x80) ? (uint8_t)((r << 1) ^ IX4_LEVER_CRC_POLY)
                           : (uint8_t)(r << 1);
        }
        crcTable[dividend] = r;
    }
}

uint8_t iX4_Lever::ComputeCrc8(const uint8_t bytes[8])
{
    /* Static-context calculation for unit tests; rebuilds table inline.
     * Production calls go through the instance crcTable for speed. */
    static uint8_t table[256];
    static bool inited = false;
    if (!inited) {
        for (int d = 0; d < 256; ++d) {
            uint8_t r = (uint8_t)d;
            for (int b = 0; b < 8; ++b)
                r = (r & 0x80) ? (uint8_t)((r << 1) ^ IX4_LEVER_CRC_POLY)
                               : (uint8_t)(r << 1);
            table[d] = r;
        }
        inited = true;
    }
    uint8_t rem = 0x00;
    for (int i = 1; i < 8; ++i)
        rem = table[bytes[i] ^ rem];
    return (uint8_t)(rem ^ IX4_LEVER_CRC_XOROUT);
}

bool iX4_Lever::ValidateFrame(const uint8_t bytes[8])
{
    return ComputeCrc8(bytes) == bytes[0];
}

void iX4_Lever::SetCanInterface(CanHardware *c)
{
    can = c;
    can->RegisterUserMessage(IX4_LEVER_CAN_ID);
}

void iX4_Lever::DecodeCAN(int id, uint32_t *data)
{
    if (id != IX4_LEVER_CAN_ID) return;

    const uint8_t *b = reinterpret_cast<const uint8_t *>(data);

    /* CRC check: poly 0x1D, init 0x00, xorout 0x04, bytes 1..7. */
    uint8_t rem = 0x00;
    for (int i = 1; i < 8; ++i)
        rem = crcTable[b[i] ^ rem];
    if ((uint8_t)(rem ^ IX4_LEVER_CRC_XOROUT) != b[0]) {
        ++crcFails;
        return; /* Drop bad frame; hold last gear. */
    }

    /* Counter sanity: low nibble of byte 1. High nibble is always 0xF
     * across 553 observed frames; treat anything else as malformed.
     * Low nibble cycles 0..14 (0xFF/0x*F unobserved as a "counter=15"). */
    if ((b[1] & 0xF0) != 0xF0) {
        ++ctrFails;
        return;
    }
    counterLow = (uint8_t)(b[1] & 0x0F);
    if (prevCounterValid) {
        uint8_t expected = (uint8_t)((prevCounter + 1) & 0x0F);
        /* Allow expected, expected+1 (one missed frame), or wrap to 0 from 14.
         * 0x0F appears unused; if seen, treat as a soft fault but still
         * advance the chain so we don't deadlock. */
        if (counterLow != expected && counterLow != ((expected + 1) & 0x0F))
            ++ctrFails;
    }
    prevCounter = counterLow;
    prevCounterValid = true;

    /* Latched gear in byte 6. */
    rawGearByte  = b[6];
    leverEncoder = b[4];

    switch (rawGearByte) {
    case 0x33: gear = PARK;    break;
    case 0x31: gear = NEUTRAL; break;
    case 0x32: gear = REVERSE; break; /* hypothesis */
    case 0x35: gear = DRIVE;   break; /* hypothesis */
    /* 0x34 (Sport) and unknown values: hold last good gear. */
    default:                   break;
    }
}

bool iX4_Lever::GetGear(Shifter::Sgear &outGear)
{
    outGear = gear;
    return true;
}
