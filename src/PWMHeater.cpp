/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2018 Johannes Huebner <dev@johanneshuebner.com>
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
#include <PWMHeater.h>
#include <iomatrix.h>
#include <libopencm3/stm32/timer.h>

PWMHeater::PWMHeater() {}

void PWMHeater::SetPower(uint16_t power, bool HeatReq) {
  int32_t PowerDC = 0;
  if (HeatReq && (power > 0)) {
    Param::SetInt(Param::PWMHeatOn, 1);
    // Use the HeatPercent as the duty cycle for the PWM signal.
    PowerDC = utils::change(Param::GetInt(Param::HeatPercnt), 0, 100, 0,
                            Param::GetInt(Param::Tim3_Period));
  } else {
    Param::SetInt(Param::PWMHeatOn, 0);
  }
  if (Param::GetInt(Param::PWM1Func) == IOMatrix::PWMHEATER) {
    timer_set_oc_value(TIM3, TIM_OC1, PowerDC);
  }
  if (Param::GetInt(Param::PWM2Func) == IOMatrix::PWMHEATER) {
    timer_set_oc_value(TIM3, TIM_OC2, PowerDC);
  }
  if (Param::GetInt(Param::PWM3Func) == IOMatrix::PWMHEATER) {
    timer_set_oc_value(TIM3, TIM_OC3, PowerDC);
  }
}
