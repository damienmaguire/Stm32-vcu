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

#include "throttle.h"
#include "my_math.h"

#define POT_SLACK 200

int Throttle::potmin[2];
int Throttle::potmax[2];
float Throttle::regenRpm;
float Throttle::regenendRpm;
float Throttle::brknompedal;
float Throttle::regenmax;
float Throttle::regenBrake;
float Throttle::brkcruise;
int Throttle::idleSpeed;
int Throttle::cruiseSpeed;
float Throttle::speedkp;
int Throttle::speedflt;
int Throttle::speedFiltered;
float Throttle::idleThrotLim;
float Throttle::potnomFiltered;
float Throttle::throtmax;
float Throttle::throtmaxRev;
float Throttle::throtmin;
float Throttle::throtdead;
float Throttle::regenRamp;
float Throttle::throttleRamp;
float Throttle::udcmin;
float Throttle::udcmax;
float Throttle::idcmin;
float Throttle::idcmax;
int Throttle::speedLimit;
float Throttle::ThrotRpmFilt;
bool Throttle::noregenreq;
float UDCres;
float IDCres;
float UDCprevspnt = 0;
float IDCprevspnt = 0;

// internal variable, reused every time the function is called
static float throttleRamped = 0.0;
static float SpeedFiltered = 0.0;

static float regenlim =0;

#define PedalPosArrLen 50
static float PedalPos;
static float LastPedalPos;
static float PedalChange =0;
static float PedalPosTot =0;
static float PedalPosArr[PedalPosArrLen];
static uint8_t PedalPosIdx = 0;
static int8_t PedalReq = 0; //positive is accel negative is decell

/**
 * @brief Check the throttle input for sanity and limit the range to min/max values
 *
 * @param potval Pointer to the throttle input array, range should be [potMin, potMax].
 * @param potIdx Index of the throttle input array, range is [0, 1].
 * @return true if the throttle input was within bounds (accounting for POT_SLACK).
 * @return false the throttle was too far out of bounds. Setting potval to the minimum.
 */
bool Throttle::CheckAndLimitRange(int* potval, int potIdx)
{
    // The range check accounts for inverted throttle pedals, where the minimum
    // value is higher than the maximum. To accomodate for that, the potMin and potMax
    // variables are set for internal use.
    int potMin = potmax[potIdx] > potmin[potIdx] ? potmin[potIdx] : potmax[potIdx];
    int potMax = potmax[potIdx] > potmin[potIdx] ? potmax[potIdx] : potmin[potIdx];

    if (((*potval + POT_SLACK) < potMin) || (*potval > (potMax + POT_SLACK)))
    {
        *potval = potMin;
        return false;
    }
    else if (*potval < potMin)
    {
        *potval = potMin;
    }
    else if (*potval > potMax)
    {
        *potval = potMax;
    }

    return true;
}

/**
 * @brief Normalize the throttle input value to the min-max scale.
 *
 * Returns 0.0% for illegal indices and if potmin and potmax are equal.
 *
 * @param potval Throttle input value, range is [potmin[potIdx], potmax[potIdx]], not checked!
 * @param potIdx Index of the throttle input, should be [0, 1].
 * @return Normalized throttle value output with range [0.0, 100.0] with correct input.
 */
float Throttle::NormalizeThrottle(int potval, int potIdx)
{
    if(potIdx < 0 || potIdx > 1)
        return 0.0f;

    if(potmin[potIdx] == potmax[potIdx])
        return 0.0f;


    return 100.0f * ((float)(potval - potmin[potIdx]) / (float)(potmax[potIdx] - potmin[potIdx]));
}

/**
 * @brief Calculate a throttle percentage from the potval input.
 *
 * After the previous range checks, the throttle input potval lies within the
 * range of [potmin[0], potmax[0]]. From this range, the input is converted to
 * a percent range of [-100.0, -100.0].
 *
 * TODO: No regen implemented. Commanding 0 throttle while braking, otherwise direct output.
 *
 * @param potval
 * @param idx Index of the throttle input that should be used for calculation.
 * @param brkpedal Brake pedal input (true for brake pedal pressed, false otherwise).
 * @return float
 */
float Throttle::CalcThrottle(int potval, int potIdx, bool brkpedal)
{
    int speed = Param::GetInt(Param::speed);
    int dir = Param::GetInt(Param::dir);
    float potnom = 0.0f;  // normalize potval against the potmin and potmax values

    if(speed< 0)//make sure speed is not negative
    {
        speed *= -1;
    }

    //limiting speed change rate
    if(ABS(speed-SpeedFiltered)>ThrotRpmFilt)
    {
        if(speed > SpeedFiltered)
        {
            SpeedFiltered +=  ThrotRpmFilt;
        }
        else
        {
            SpeedFiltered -=  ThrotRpmFilt;
        }
    }
    else
    {
        SpeedFiltered = speed;
    }

    speed = SpeedFiltered;

    ///////////////////////

    if(dir == 0)//neutral no torque command
    {
        return 0;
    }

    if (brkpedal)
    {
        if(speed < 100 || speed < regenendRpm)
        {
            return 0;
        }
        else if (speed < regenRpm)
        {
            potnom = utils::change(speed, regenendRpm, regenRpm, 0, regenBrake);//taper regen according to speed
            return potnom;
        }
        else
        {
            potnom =  regenBrake;
            return potnom;
        }
    }

    // substract offset, bring potval to the potmin-potmax scale and make a percentage
    potnom = NormalizeThrottle(potval, potIdx);

    // Apply the deadzone parameter. To avoid that we lose the range between
    // 0 and throtdead, the scale of potnom is mapped from the [0.0, 100.0] scale
    // to the [throtdead, 100.0] scale.
    if(potnom < throtdead)
    {
        potnom = 0.0f;
    }
    else
    {
        potnom = (potnom - throtdead) * (100.0f / (100.0f - throtdead));
    }

//!! pedal command intent coding

    PedalPos = potnom; //save comparison next time to check if pedal had moved

    float TempAvgPos = AveragePos(PedalPos); //get an rolling average pedal position over the last 50 measurements for smoothing

    PedalChange = PedalPos - TempAvgPos; //current pedal position compared to average


    if(PedalChange < -1.0 )//Check pedal is release compared to last time
    {
        PedalReq = -1; //pedal is released enough - Commanding regen or slowing
    }
    else if(PedalChange > 1.0 )//Check pedal is increased compared to last time
    {
        PedalReq = 1; //pedal pressed - Commanding accelerating - thus always more power
    }
    else//pedal not changed
    {
        potnom = TempAvgPos; //use the averaged pedal
    }


    //Do clever bits for regen and such.


    if (noregenreq>0) //If forced to have no regen, like clutch pedal pressed
    {
        regenlim = 0;
    }
    else
    {
        if(speed < 100 || speed <regenendRpm)//No regen under 100 rpm or speed under regenendRpm
        {
            regenlim = 0;
        }
        else if(speed < regenRpm)
        {
            regenlim = utils::change(speed, regenendRpm, regenRpm, 0, regenmax);//taper regen according to speed
        }
        else
        {
            regenlim = regenmax;
        }
    }



    //!!!potnom is throttle position up to this point//

    if(dir == 1)//Forward
    {
        //change limits to uint32, multiply by 10 then 0.1 to add a decimal to remove the hard edges
        potnom = utils::change(potnom,0,100,regenlim*10,throtmax*10);
        potnom *= 0.1;
    }
    else //Reverse, as neutral already exited function
    {
        if(Param::GetInt(Param::revRegen) == 0)//If regen in reverse is to be off
        {
            regenlim = 0;
        }
        potnom = utils::change(potnom,0,100,regenlim*10,throtmaxRev*10);
        potnom *= 0.1;
    }

    LastPedalPos = PedalPos; //Save current pedal position for next loop.
    return potnom;
}

/**
 * @brief Apply the throttle ramping parameters for ramping up and down.
 *
 * @param potnom Normalized throttle command in percent, range [-100.0, 100.0].
 * @return float Ramped throttle command in percent, range [-100.0, 100.0].
 */
float Throttle::RampThrottle(float potnom)
{
    // make sure potnom is within the boundaries of [throtmin, throtmax]
    potnom = MIN(potnom, throtmax);
    potnom = MAX(potnom, throtmin);

    if (potnom >= throttleRamped) // higher throttle command than currently applied
    {
        if(potnom > 0)
        {
            throttleRamped = RAMPUP(throttleRamped, potnom, throttleRamp);
            potnom = throttleRamped;
        }
        else
        {
            throttleRamped = RAMPUP(throttleRamped, potnom, regenRamp);
            potnom = throttleRamped;
        }
    }
    else //(potnom < throttleRamped) // lower throttle command than currently applied
    {
        if(potnom >= 0)
        {
            throttleRamped = potnom; //No ramping from high throttle to low throttle
        }
        else
        {
            if(throttleRamped > 0)
            {
                throttleRamped = 0;
            }
            throttleRamped = RAMPDOWN(throttleRamped, potnom, regenRamp);
            potnom = throttleRamped;
        }
    }

    return potnom;
}

float Throttle::CalcIdleSpeed(int speed)
{
    int speederr = idleSpeed - speed;
    return MIN(idleThrotLim, speedkp * speederr);
}

float Throttle::CalcCruiseSpeed(int speed)
{
    speedFiltered = IIRFILTER(speedFiltered, speed, speedflt);
    int speederr = cruiseSpeed - speedFiltered;

    float potnom = speedkp * speederr;
    potnom = MIN(100, potnom);
    potnom = MAX(brkcruise, potnom);

    return potnom;
}

bool Throttle::TemperatureDerate(float temp, float tempMax, float& finalSpnt)
{
    uint16_t DerateReason = Param::GetInt(Param::TorqDerate);
    float limit = 0;

    if (temp <= tempMax)
    {
        limit = 100.0f;
    }
    else if (temp < (tempMax + 2.0f))
    {
        limit = 50.0f;
        DerateReason |= 16;
        Param::SetInt(Param::TorqDerate,DerateReason);
    }

    if (finalSpnt >= 0)
        finalSpnt = MIN(finalSpnt, limit);
    else
        finalSpnt = MAX(finalSpnt, -limit);

    return limit < 100.0f;
}

void Throttle::UdcLimitCommand(float& finalSpnt, float udc)
{
    udcmin = Param::GetFloat(Param::udcmin);//Made dynamic
    udcmax = Param::GetFloat(Param::udclim);

    uint16_t DerateReason = Param::GetInt(Param::TorqDerate);

    if(udcmin>0)    //ignore if set to zero. useful for bench testing without isa shunt
    {
        if (finalSpnt >= 0) //if we are requesting torque
        {
            float udcErr = udc - udcmin;
            UDCres = udcErr * 3.5; // error multiplied by
            UDCres = MAX(0, UDCres); // only allow positive UDCres limit
            if(finalSpnt > UDCres)
            {
                DerateReason |= 1;
            }
            finalSpnt = MIN(finalSpnt, UDCres);
        }
        else
        {
            float udcErr = udc - udcmax;
            UDCres = udcErr * 3.5;
            UDCres = MIN(0, UDCres);
            if(finalSpnt < UDCres)
            {
                DerateReason |= 2;
            }
            finalSpnt = MAX(finalSpnt, UDCres);

        }
    }
    else
    {
        finalSpnt = UDCprevspnt;
        finalSpnt = finalSpnt;
    }
}

void Throttle::IdcLimitCommand(float& finalSpnt, float idc)
{
    static float idcFiltered = 0;
    idcFiltered = IIRFILTERF(idcFiltered, idc, 4);

    idcmax = Param::GetFloat(Param::idcmax);//Made dynamic
    idcmin = Param::GetFloat(Param::idcmin);

    uint16_t DerateReason = Param::GetInt(Param::TorqDerate);

    if(idcmax>0)    //ignore if set to zero. useful for bench testing without isa shunt
    {
        if (finalSpnt >= 0)
        {
            float idcerr = idcmax - idcFiltered;
            IDCres = idcerr * 1;//gain needs tuning
            IDCres = MAX(0, IDCres);
            if(finalSpnt < IDCres)
            {
                DerateReason |= 8;
            }
            finalSpnt = MIN(finalSpnt, IDCres);
        }
        else
        {

            float idcerr = idcmin + idcFiltered;
            IDCres = idcerr * 1;//gain needs tuning
            IDCres = MIN(0, IDCres);
            if(finalSpnt > IDCres)
            {
                DerateReason |= 4;
            }
            finalSpnt = MAX(finalSpnt, IDCres);
        }
    }
    else
    {
        finalSpnt = finalSpnt;
    }
}


void Throttle::SpeedLimitCommand(float& finalSpnt, int speed)
{
    static int speedFiltered = 0;

    speedFiltered = IIRFILTER(speedFiltered, speed, 4);

    if (finalSpnt > 0)
    {
        int speederr = speedLimit - speedFiltered;
        int res = speederr / 4;

        res = MAX(0, res);
        finalSpnt = MIN(res, finalSpnt);
    }
}

float Throttle::AveragePos(float Pos)
{
    PedalPosIdx++; //next average arrray positon
    if(PedalPosIdx >= PedalPosArrLen)
    {
        PedalPosIdx = 0;
    }
    PedalPosTot -= PedalPosArr[PedalPosIdx];
    PedalPosTot += Pos;
    PedalPosArr[PedalPosIdx] = Pos;

    return PedalPosTot/PedalPosArrLen;
}
