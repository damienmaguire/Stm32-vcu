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

#ifndef NOVEHICLE_H_INCLUDED
#define NOVEHICLE_H_INCLUDED

#include <vehicle.h>

class NoVehicle : public Vehicle {
public:
  void SetRevCounter(int speed) { speed = speed; }
  void SetTemperatureGauge(float temp) { temp = temp; }
  bool Ready() { return DigIo::t15_digi.Get(); }
  bool Start() { return Param::GetBool(Param::din_start); }

protected:
};

#endif // VEHICLE_H_INCLUDED
