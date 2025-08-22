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

#ifndef NoInverter_h
#define NoInverter_h

/*  Dummy library for when no Inverter interface is used.

*/
#include <inverter.h>

class NoInverterClass : public Inverter {

public:
  void SetTorque(float torquePercent) { torquePercent = torquePercent; }
  float GetMotorTemperature() { return (0); }
  float GetInverterTemperature() { return (0); }
  float GetInverterVoltage() { return (0); }
  float GetMotorSpeed() { return (0); }
  int GetInverterState() { return (0); }

private:
};

#endif
