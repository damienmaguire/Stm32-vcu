/*
 * This file is part of the ZombieVeter project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *               2021-2022 Damien Maguire <info@evbmw.com>
 * Yes I'm really writing software now........run.....run away.......
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

#ifndef notused_h
#define notused_h

/*  Dummy library for unused devices. */

#include "chargerint.h"
#include <stdint.h>

class notused :
   public Chargerint,
   public BMS,
   public Chargerhw,
   public Shifter,
   public Heater,
   public Compressor,
   public Inverter,
   public Vehicle {

public:
  bool ACRequest(bool RunCh) { return RunCh; };
  void SetTargetTemperature(float temp) { (void)temp; }
  void SetPower(uint16_t power, bool HeatReq) { (void)power; (void)HeatReq; }
  void SetTorque(float torquePercent) { (void)torquePercent; }
  float GetMotorTemperature() { return 0; }
  float GetInverterTemperature() { return 0; }
  float GetInverterVoltage() { return 0; }
  float GetMotorSpeed() { return 0; }
  int GetInverterState() { return 0; }
  void SetRevCounter(int speed) { speed = speed; }
  void SetTemperatureGauge(float temp) { temp = temp; }
  bool Ready() { return DigIo::t15_digi.Get(); }
  bool Start() { return Param::GetBool(Param::din_start); }

private:
};

#endif /* notused_h */
