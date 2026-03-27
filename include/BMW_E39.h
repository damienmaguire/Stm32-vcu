/*
 * This file is part of the stm32-vcu project.
 *
 * Copyright (C) 2021 Damien Maguire
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

#ifndef BMW_E39_h
#define BMW_E39_h

/*  This library supports the Powertrain CAN messages for the BMW E39 for
   driving dash gauges, putting out malf lights etc

*/

#include "vehicle.h"
#include <stdint.h>

class BMW_E39 : public Vehicle {

public:
  void SetCanInterface(CanHardware *c);
  void Task10Ms();
  void Task100Ms();
  void SetRevCounter(int s) { speed = s; }
  void SetTemperatureGauge(float temp);
  void DecodeCAN(int id, uint32_t *data);
  bool Ready();
  bool Start();
  void SetE46(bool e46) { isE46 = e46; }

private:
  void Msg316();
  void Msg329();
  void Msg545();
  void Msg43F(int8_t gear);
  void Msg43B();

  uint16_t speed;
  bool isE46;
  bool AbsCANalive;
  bool SendCAN;
};

#endif /* Can_E39_h */
