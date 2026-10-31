/*
 * This file is part of the stm32-vcu project.
 *
 * Copyright (C) 2026 Damien Maguire
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

#ifndef BMW_F11_h
#define BMW_F11_h

/* BMW F11 (5 Series Touring, N47). The diesel ECU and the ZF 8HP are
   gone. This class stands in for the DDE on PT-CAN (500 kbit/s) so the
   chassis ECUs still see an engine. FlexRay is a separate pico board.

   Phase 1 transmits only while 0x12F shows the bus is already awake.
   Nothing is sent on PT-CAN2. 0x137, 0x3FB, 0x580, 0x5C0, 0x380, 0x351
   and the 0x592 immobiliser sub-frames (0x80 / 0x85) are never sent.

   Untested on the car.
*/

#include "vehicles/vehicle.h"
#include <stdint.h>

class BMW_F11 : public Vehicle {
public:
  BMW_F11();
  void SetCanInterface(CanHardware *c);
  void Task10Ms();
  void Task100Ms();
  void SetRevCounter(int s) { motorRpm = s; }
  void SetTemperatureGauge(float temp) { coolantC = temp; }
  void DecodeCAN(int id, uint32_t *data);
  bool Ready();
  bool Start();

  // Driver demand in Nm, after the DSC ceiling. With no pico frame, or
  // once that frame is stale, the ceiling is "no limit" and driverNm is
  // returned unchanged.
  float FinalTorqueNm(float driverNm) const;

private:
  void SendFastFrames();
  void Send3F9(bool ready);
  void Send100MsFrames();
  void SendMilOff();
  void NoteBusState();
  void OnBusWake();
  void OnBusSleep();
  bool BusAwake() const { return busQuiet100 < 20; }
  uint16_t RpmTx() const;
  float DriverDemandNm() const;
  void AgeDscLimit();
  void HandleDscTorqueCeiling(uint32_t *data);
  // TODO: a latched ZombieVerter fault. Stays false, so MIL ON is never
  // sent. The ON payload would be 40 22 00 15 FF FF FF FF.
  bool MilFaultLatched() const { return false; }

  static uint8_t Crc8_1d(const uint8_t *data, int len, uint8_t crc);
  static void E2ESeal(uint8_t *frame, int dlc, uint16_t dataId, uint8_t *alive);

  int motorRpm;
  float coolantC;
  uint8_t busQuiet100; // 100 ms ticks since the last 0x12F. 20 = 2 s.
  bool busWasAwake;
  uint16_t tick10;

  uint8_t aliveA5;
  uint8_t aliveD9;
  uint8_t aliveDc;
  uint8_t aliveF3;
  uint8_t alive8F;
  uint8_t alive3F9;

  bool seen3f9;
  uint8_t age3f910;
  uint8_t last3f9[6];

  bool milOffSent;

  // DSC torque ceiling. The DSC talks to the DDE on FlexRay only. The
  // pico will forward that limit on CAN. The ID and the byte layout are
  // not defined, so dscTorqueCanId stays 0 and no filter is registered.
  bool dscLimitFresh;
  uint8_t dscLimitAge10ms;
  float dscLimitNm;
  uint32_t dscTorqueCanId;
};

#endif /* BMW_F11_h */
