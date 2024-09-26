/*
 * This file is part of the tumanako_vc project.
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
 */
#ifndef LEAFBMS_H
#define LEAFBMS_H
#include "bms.h"

class LeafBMS: public BMS
{
public:
    virtual void SetCanInterface(CanHardware* can);
    void DecodeCAN(int id, uint8_t * data);
    static void RequestNextFrame(CanHardware* can);
    static uint16_t GetCellVoltage(int idx);
    static int GetCellStatus(int idx);
    static const int NUMCELLS = 96;
    static float Voltage;
    static float Voltage2;
    static int32_t Temperature;
    static int32_t Amperes;   // Floating point with current in Amperes
    static int32_t SOC;
    static int32_t KW;
    static int32_t KWh;


private:
    static uint8_t Crc8ForHCM(int n, uint8_t *msg);
    static int bmsGrp;
    static int bmsGrpIndex;
    static uint8_t voltBytes[NUMCELLS * 2];
    static uint8_t statusBits[NUMCELLS / 4];

};

#endif // LEAFBMS_H
