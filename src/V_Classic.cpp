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
#include "my_math.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

// We use this as an init function
void V_Classic::SetCanInterface(
    CanHardware
        *c) // Abusing the SetCanInterface as a initializer function on start up
{
  can = c;
  utils::SpeedoStart();
}

void V_Classic::SetRevCounter(int speed) {
  uint16_t speed_input = speed;
  if (Param::GetInt(Param::PumpPWM) == 1) // If Pump PWM out is set to Tacho
  {
    speed_input = MAX(750, speed_input); //
    speed_input = MIN(7500, speed_input);
  }
  if (Param::GetInt(Param::PumpPWM) == 2) // If Pump PWM out is set to Speedo
  {
    speed_input = MIN(13000, speed_input);
    if (speed_input < 200)
      speed_input = 0; // to be verified if this is okay
  }

  utils::SpeedoSet(speed_input); // Moved pwm control into Utils
}

void V_Classic::SetTemperatureGauge(float temp) {
  temp = temp; // We aint doing anything here old code!
}

void V_Classic::Task1Ms() {}

void V_Classic::Task10Ms() {}

void V_Classic::Task100Ms() {
  if (!Param::GetInt(Param::T15Stat)) {
    utils::SpeedoSet(0);       // set speedo off
    utils::SetTempgaugePWM(0); // turn off temp gauge
    utils::SetSocgaugePWM(0);  // turn off soc gauge
  } else {
    utils::SetTempgaugePWM(1); // turn on temp gauge
    utils::SetSocgaugePWM(1);  // turn on soc gauge
  }
}

bool V_Classic::Ready() { return DigIo::t15_digi.Get(); }

bool V_Classic::Start() { return Param::GetBool(Param::din_start); }
