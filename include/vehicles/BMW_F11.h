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

// BMW F11 (5 Series Touring, 2010-2017) with the diesel engine and the
// ZF 8-speed gearbox removed.
//
// The DDE was the diesel engine ECU. The dash, DSC (stability control),
// steering and heater still expect it. This class takes the DDE's place
// on PT-CAN, the powertrain CAN bus at 500 kbit/s. Plug that bus into
// whichever ZombieVerter port is selected as "Vehicle CAN".
//
// We listen on PT-CAN for ignition and the brake pedal, and we send the
// engine frames (rpm, temperatures, "engine running", check-engine light
// off). We send nothing on PT-CAN2. That second bus belonged to the
// gearbox.
//
// FlexRay is the car's other fast network. A separate Pico board
// emulates the DDE there. This class does not speak FlexRay and does
// not send anything to the Pico.
//
// Words used below:
//   KL15  ignition on
//   KL50  start / crank
//   MIL   the check-engine light
//   E2E   the checksum on important frames. Byte 0 is a CRC and the low
//         half of byte 1 is a counter that steps 0..14. If either is
//         wrong the receiving ECU throws the frame away.
//
// We only transmit after 0x12F or 0x173 has been heard in the last 2
// seconds, so a silent car is never woken by us.
//
// Not done yet, and not tried on the car:
//   real motor torque in Nm (we report 0 Nm)
//   a DSC torque limit from the Pico (no CAN id chosen)
//   turning the MIL on for a real fault
//   which brake-switch bit is the brake-light switch
//   the ignition-off variants of a few engine frames

#include "vehicles/vehicle.h"
#include <stdint.h>

class BMW_F11 : public Vehicle {
public:
  BMW_F11();
  // Choose the Vehicle CAN port and ask to be told about 0x12F and 0x173.
  void SetCanInterface(CanHardware *c);
  // 10 ms tick: fast engine frames, temperatures, MIL off.
  void Task10Ms();
  // 100 ms tick: fuel counter, 12 V, and the 2 second bus-sleep timer.
  void Task100Ms();
  // Inverter rpm. While driving, the dash is shown at least 750 rpm.
  void SetRevCounter(int s) { motorRpm = s; }
  // Inverter coolant temperature, degrees C. Shown on 0x3F9.
  void SetTemperatureGauge(float temp) { coolantC = temp; }
  // Incoming PT-CAN: ignition (0x12F) and brake pedal (0x173).
  void DecodeCAN(int id, uint32_t *data);
  // True when KL15 (ignition) is on. The rest of the VCU treats this
  // the way an E46 treats the terminal-15 pin.
  bool Ready();
  // True for the start button, or for KL50 (crank) on 0x12F.
  bool Start();

  // Driver demand in Nm, limited by the DSC torque ceiling. With no
  // Pico frame the ceiling is "no limit" and driverNm comes back as-is.
  float FinalTorqueNm(float driverNm) const;

private:
  // 0x0A5, 0x0D9, 0x0DC and 0x0F3, every 20 ms.
  void SendFastFrames();
  // 0x3F9 temperatures and engine-running, every 1 s and on change.
  void Send3F9(bool ready);
  // 0x2C4 fuel used and 0x281 12 V, every 100 ms.
  void Send100MsFrames();
  // One check-engine-light off frame on 0x592.
  void SendMilOff();
  // Notice PT-CAN waking or going to sleep.
  void NoteBusState();
  void OnBusWake();
  void OnBusSleep();
  // True until 2 seconds have passed with no 0x12F and no 0x173.
  bool BusAwake() const { return busQuiet100 < 20; }
  // rpm for the dash. At least 750 while the inverter is running.
  uint16_t RpmTx() const;
  // Driver torque in Nm. Currently 0 until a real Nm source exists.
  float DriverDemandNm() const;
  // Drop a DSC torque cut that has not been refreshed for 500 ms.
  void AgeDscLimit();
  // Decode ignition and start from 0x12F.
  void Handle12F(uint8_t *bytes);
  // Decode the brake pedal from 0x173.
  void Handle173(uint8_t *bytes);
  // OR the CAN brake pedal into the normal brake input.
  void ApplyBrake();
  // TODO: Pico frame that carries the DSC torque ceiling. No id yet.
  void HandleDscTorqueCeiling(uint32_t *data);
  // TODO: return true for a latched ZombieVerter fault. While this
  // stays false the check-engine light (MIL) is never turned on.
  // The ON frame would be 40 22 00 15 FF FF FF FF.
  bool MilFaultLatched() const { return false; }

  static uint8_t Crc8_1d(const uint8_t *data, int len, uint8_t crc);
  static void E2ESeal(uint8_t *frame, int dlc, uint16_t dataId, uint8_t *alive);

  int motorRpm;
  float coolantC;
  // 100 ms ticks since the last 0x12F or 0x173. 20 ticks = 2 seconds,
  // which means PT-CAN has gone to sleep and we must stop talking.
  uint8_t busQuiet100;
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

  // From 0x12F byte 2, low nibble. 6 = off, 9 = accessory (holds KL15
  // only if it was already on), >= 0xA = ignition on, >= 0xD = crank.
  // 0xE/0xF leave these alone. See Handle12F.
  bool kl15On;
  bool kl50On;

  // From 0x173 byte 7 bits 0 and 1. Either switch means the pedal is
  // down. Which bit is the brake-light switch is still unconfirmed.
  bool brakePressed;
  bool brakeSeen;
  // 10 ms ticks since the last 0x173. Past 50 (500 ms) we let go.
  uint8_t brakeAge10ms;

  // DSC torque ceiling. The DSC talks to the DDE on FlexRay only. The
  // pico will forward that limit on CAN. The ID and the byte layout are
  // not defined, so dscTorqueCanId stays 0 and no filter is registered.
  bool dscLimitFresh;
  uint8_t dscLimitAge10ms;
  float dscLimitNm;
  uint32_t dscTorqueCanId;
};

#endif /* BMW_F11_h */
