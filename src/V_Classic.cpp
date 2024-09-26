/*
 * This file is part of the Zombieverter project.
 *
 * Copyright (C) 2023 Damien Maguire
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
#include "V_Classic.h"
#include "hwinit.h"
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include "my_math.h"


//We use this as an init function
void V_Classic::SetCanInterface(CanHardware* c)//Abusing the SetCanInterface as a initializer function on start up
{
    can = c;
    utils::SpeedoStart();
}


void V_Classic::SetRevCounter(int speed)
{
    uint16_t speed_input = speed;
    speed_input = MAX(750, speed_input);//
    speed_input = MIN(7500, speed_input);

    utils::SpeedoSet(speed_input);//Moved pwm control into Utils
}


void V_Classic::SetTemperatureGauge(float temp)
{
    float dc = temp * 10; //TODO find right factor for value like 0..0.5 or so
    //Would like to use digi pots here
    dc = dc;
}

void V_Classic::Task1Ms()
{

}


void V_Classic::Task10Ms()
{

}


void V_Classic::Task100Ms()
{
    if (!Param::GetInt(Param::T15Stat))
    {
        utils::SpeedoSet(0);//set speedo off
    }
}


bool V_Classic::Ready()
{
    return DigIo::t15_digi.Get();
}

bool V_Classic::Start()
{
    return Param::GetBool(Param::din_start);
}
