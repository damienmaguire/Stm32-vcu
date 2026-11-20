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

/* Stand-in for the Lexus GS450h HV-ECU (89983-30200) on the car CAN.
 * VehicleCan is firmware bus 1 (logger bus 1, ECM/body). Gs450Bus is
 * firmware bus 2 (logger bus 0, chassis). Frames the OEM sends on both
 * are sent on both when the two controllers differ.
 *
 * GsEcm bits, from the ECM frames this class receives:
 *   b0  0x039 byte1 bit1   engine stopped
 *   b1  0x039 byte2 0x77   engine-off sentinel
 *   b2  0x3C8 byte0 bit7   crank request
 *   b3  0x3C8 byte4 bit2   engine running
 *   b4  0x3CF byte0 bit4   ECM init done
 *   b5  a checksum-good ECM frame has been seen
 *   b6  0x484 seen recently (no checksum on that frame)
 */

#include "vehicles/vehicle.h"
#include <stdint.h>

class GS450H_Vehicle : public Vehicle {
public:
  void SetCanInterface(CanHardware *c);
  void SetChassisCan(CanHardware *c);
  void Task1Ms();
  void Task100Ms();
  void DecodeCAN(int id, uint32_t *data);
  void SetRevCounter(int speed) { motorRpm = speed; }
  void SetTemperatureGauge(float temp) { (void)temp; }
  bool Ready();

private:
  CanHardware *chassis = 0;
  uint32_t tick = 0;
  int16_t motorRpm = 0;
  uint16_t age039 = 10000;
  uint16_t age3C8 = 10000;
  uint16_t age3CF = 10000;
  uint16_t age484 = 10000;
  uint8_t ecmFlags = 0;

  void SendOn(CanHardware *bus, uint32_t id, uint8_t *b, uint8_t dlc);
  void Dispatch(uint8_t which, uint32_t id, uint8_t *b, uint8_t dlc);
  void AgeEcm();
};

#endif /* GS450H_VEHICLE_H */
