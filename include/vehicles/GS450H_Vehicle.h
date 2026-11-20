/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2026 Damien Maguire <info@evbmw.com>
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

#ifndef GS450H_VEHICLE_H
#define GS450H_VEHICLE_H

/* Stand-in for the Lexus GS450h HV-ECU on the one CAN that stays in
 * the car. That is VehicleCan. The other OEM link only joined the
 * engine ECM to the HV-ECU, and both of those ECUs are removed, so
 * this class does not open a second bus and does not listen for ECM
 * frames.
 *
 * What it sends are the HV frames logged on firmware bus 2 / logger
 * bus 0, the bus the remaining modules (brake, steering, body) are
 * still on. See GS450H_Vehicle.cpp.
 */

#include "vehicles/vehicle.h"
#include <stdint.h>

class GS450H_Vehicle : public Vehicle {
public:
  void SetCanInterface(CanHardware *c);
  void Task1Ms();
  void SetRevCounter(int speed) { motorRpm = speed; }
  void SetTemperatureGauge(float temp) { (void)temp; }
  bool Ready();

private:
  uint32_t tick = 0;
  int16_t motorRpm = 0;
};

#endif /* GS450H_VEHICLE_H */
