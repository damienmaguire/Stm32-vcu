/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2012-2020 Johannes Huebner <dev@johanneshuebner.com> 
 *               2021-2022 Damien Maguire <info@evbmw.com>
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

#ifndef THROTTLE_H
#define THROTTLE_H

#include "my_fp.h"

class Throttle
{
public:
    static bool CheckAndLimitRange(int* potval, int potIdx);
    static float NormalizeThrottle(int potval, int potIdx);
    static float CalcThrottle(int potval, int potIdx, bool brkpedal);
    static float CalcIdleSpeed(int speed);
    static float CalcCruiseSpeed(int speed);
    static bool TemperatureDerate(float tmp, float tmpMax, float& finalSpnt);
    static void UdcLimitCommand(float& finalSpnt, float udc);
    static void IdcLimitCommand(float& finalSpnt, float idc);
    static void SpeedLimitCommand(float& finalSpnt, int speed);
    static float RampThrottle(float finalSpnt);
    static void RegenRampDown(float& finalSpnt, int speed);
    static int potmin[2];
    static int potmax[2];
    static float regenTravel;
    static float brknompedal;
    static float regenmax;
    static float brkcruise;
    static float throtmax;
    static float throtmin;
    static float throtdead;
    static int idleSpeed;
    static int cruiseSpeed;
    static float speedkp;
    static int speedflt;
    static float idleThrotLim;
    static float regenRamp;
    static float throttleRamp;
    static int bmslimhigh;
    static int bmslimlow;
    static int accelmax;
    static int accelflt;
    static float udcmin;
    static float udcmax;
    static float idcmin;
    static float idcmax;
    static int speedLimit;

private:
    static int speedFiltered;
    static float potnomFiltered;
    static float brkRamped;
};

#endif // THROTTLE_H
