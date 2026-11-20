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

/* Stand-in for the Lexus GS450h HV-ECU on VehicleCan, which is the
 * chassis bus. The ECM and the original HV-ECU are both removed.
 * Gear on 0x540 and 0x120 comes from Param::dir. Wheel speed, vehicle
 * speed and the brake switch are read back off the same bus.
 */

#include "vehicles/vehicle.h"
#include <stdint.h>

class GS450H_Vehicle : public Vehicle {
public:
  void SetCanInterface(CanHardware *c);
  void Task1Ms();
  void Task10Ms();
  void Task100Ms();
  void DecodeCAN(int id, uint32_t *data);
  void SetRevCounter(int speed) { motorRpm = speed; }
  void SetTemperatureGauge(float temp) { (void)temp; }
  bool Ready();

private:
  uint32_t nowUs = 0;
  uint32_t ignAt = 0;
  uint32_t busAt = 0;
  uint32_t txOrigin = 0;
  uint32_t dueUs[6] = {};
  int16_t motorRpm = 0;
  int lastGear = 0;
  uint16_t age230 = 10000;
  uint16_t age320 = 10000;
  bool ignWas = false;
  bool busSeen = false;
  bool txOn = false;
  bool haveGear = false;
  bool next540A5 = true;
  bool sw230 = false;
  bool sw320 = false;

  int GearCode() const;
  void ApplyGear(uint16_t id, uint8_t *b) const;
  void ApplyBrake() const;
  void NoteBus();
  void ResetTx();
  void SendFrame(int slot, bool event540);
};

#endif /* GS450H_VEHICLE_H */
