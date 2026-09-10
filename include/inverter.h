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

#ifndef INVERTER_H_INCLUDED
#define INVERTER_H_INCLUDED

#include "canhardware.h"
#include "params.h"

class Inverter {
public:
  virtual void Task1Ms() {}   // Default does nothing
  virtual void Task10Ms() {}  // Default does nothing
  virtual void Task100Ms() {} // Default does nothing
  virtual void DecodeCAN(int, uint32_t *) {};
  virtual void SetTorque(float torquePercent) = 0;
  virtual float GetMotorTemperature() = 0;
  virtual float GetInverterTemperature() = 0;
  virtual float GetInverterVoltage() = 0;
  virtual float GetMotorSpeed() = 0;
  virtual int GetInverterState() = 0;
  virtual void DeInit() {
  } // called when switching to another inverter, similar to a destructor
  virtual void SetCanInterface(CanHardware *c) { can = c; }
  // Which Param the "post processed final torque request" telemetry value is
  // published to. Defaults to Param::torque; a second, simultaneously active
  // inverter instance is pointed at Param::torque2 instead so the two don't
  // overwrite each other's display value.
  virtual void SetTorqueParam(Param::PARAM_NUM p) { torqueParam = p; }
  // Which Param drivers that support a reverse-wired motor (e.g.
  // RearOutlanderInverter, LeafINV) read to flip torque sign. Defaults to
  // Param::reversemotor; a second, simultaneously active inverter instance
  // is pointed at Param::reversemotor2 so the two can be wired oppositely
  // without fighting over one flag.
  virtual void SetReverseParam(Param::PARAM_NUM p) { reverseParam = p; }

protected:
  CanHardware *can;
  Param::PARAM_NUM torqueParam = Param::torque;
  Param::PARAM_NUM reverseParam = Param::reversemotor;
};

#endif // INVERTER_H_INCLUDED
