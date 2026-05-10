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
 * Reverse engineered from passive CAN capture of a 2022 BMW i4 G26 Gran
 * Coupe AWD shared by Damien Maguire on openinverter.org forum thread
 * #7028 (Mar 21 2026). CRC8 polynomial reuses the F-chassis F30_Lever
 * scheme; per-ID xorout determined empirically from 553 captured frames.
 */

#ifndef IX4_LEVER_H
#define IX4_LEVER_H

#include "shifter.h"
#include <stdint.h>

#define IX4_LEVER_CAN_ID       0x3F9
#define IX4_LEVER_CRC_POLY     0x1D
#define IX4_LEVER_CRC_XOROUT   0x04

class iX4_Lever : public Shifter {
public:
    iX4_Lever();
    void DecodeCAN(int id, uint32_t *data) override;
    bool GetGear(Shifter::Sgear &outGear) override;
    void SetCanInterface(CanHardware *c) override;

    /* Exposed for unit-test access. */
    static uint8_t ComputeCrc8(const uint8_t bytes[8]);
    static bool    ValidateFrame(const uint8_t bytes[8]);

    /* Telemetry that may be exposed as VCU params. */
    uint8_t  GetRawGearByte() const   { return rawGearByte; }
    uint8_t  GetLeverPosition() const { return leverEncoder; }
    uint8_t  GetCounter() const       { return counterLow; }
    uint16_t GetCrcFailCount() const  { return crcFails; }
    uint16_t GetCounterFailCount() const { return ctrFails; }

private:
    Shifter::Sgear gear;
    uint8_t  rawGearByte;
    uint8_t  leverEncoder;
    uint8_t  counterLow;
    uint8_t  prevCounter;
    bool     prevCounterValid;
    uint16_t crcFails;
    uint16_t ctrFails;

    void BuildCrcTable();
    uint8_t crcTable[256];
};

#endif // IX4_LEVER_H
