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

#ifndef BMW_E92_h
#define BMW_E92_h

/*  BMW E92 (N43, Bosch MSD80 DME) with the engine removed.
    This class stands in for that DME on PT-CAN at 500 kbit/s.
    Plug that bus into whichever port is selected as Vehicle CAN.

    It is shaped like the E46/E39 class: one vehicle object, selected
    by the Vehicle parameter, sending DME frames from the 10 ms task
    and reading ignition and road speed back.

    Frames sent (rates from the MSD80 logs):
      0x0A8, 0x0A9, 0x0AA   10 ms    four torques, rpm, pedal, engine state
      0x1B6                 60 ms    heat / load, logged running and off bytes
      0x1D0                100 ms    coolant, intake/ambient, pressure
      0x200                200 ms    constant 00 80 C0 00 00 00 00 00
      0x3B3                200 ms    power-available ramp (log median 201 ms)
      0x492                460 ms    network management, wake then 17 52
      0x2F3               1000 ms    slow alive frame
      0x308                500 ms    alive and nibble checksum, KL15 or RUN
      0x3B4               1800 ms    12 V battery voltage
    The log median for 0x3B4 is 1799.9 ms. A "~200 ms" note in the
    task summary loses to that measurement.

    Not sent, on purpose:
      0x80, 0x85, 0x351, 0x380, 0x580, 0x5C0 and any challenge burst
        (immobiliser / EWS / CAS).
      0x592, 0x612, 0x7E8 and every other 0x6xx id (diagnostic, OBD,
        and the 0x612 network frame that sits in that range).
      0x0AC, 0x0B4, 0x175, 0x38E: in the firmware, never seen on the car.
      0x332, 0x337, 0x381, 0x383, 0x38B, 0x3EF: real DME frames
        whose bytes are undecoded RAM mirrors. They are outside the
        set the dash needs, and a made-up value could look like a fault.
*/

#include "vehicles/BMW_E92_codec.h"
#include "vehicles/vehicle.h"
#include <stdint.h>

class BMW_E92 : public Vehicle {
public:
  BMW_E92();
  void SetCanInterface(CanHardware *c);
  void Task10Ms();
  void Task100Ms();
  void SetRevCounter(int s) { motorRpm = s; }
  void SetTemperatureGauge(float temp) { coolantC = temp; }
  void DecodeCAN(int id, uint32_t *data);
  // Ignition on: terminal 15 on the wiring pin, or KL15 in 0x130.
  bool Ready();
  // Start request: the wired start input, or KL50 (crank) in 0x130.
  bool Start();

private:
  void Send0A8();
  void Send0A9();
  void Send0AA();
  void Send1B6();
  void Send1D0();
  void Send200();
  void Send2F3();
  void Send3B3();
  void Send3B4();
  void Send308();
  void Send492();
  E92Engine Picture();
  // Ignition on and the VCU actually in RUN. KL15 alone is not enough.
  bool MotorRunning();

  int motorRpm;
  float coolantC;
  bool sendOn;
  bool sawCar;
  bool saw130;
  bool kl15;
  bool kl50;
  uint8_t awakeHold;
  uint8_t lost130;
  uint32_t tick;
  uint8_t aliveA8;
  uint8_t aliveA9;
  uint8_t aliveAA;
  uint8_t alive1D0;
  uint8_t alive2F3;
  uint8_t alive308;
  uint16_t fuel;
  // Set once the motor has been in RUN this wake. Cleared at sleep.
  bool engineEverRun;
  // 0x492 sequence. Restarts each time we begin transmitting.
  bool busLive;
  int nmPhase;
  int nmStatusFrames;
  uint8_t nmWait;
  // Milliseconds the motor has been in RUN, for the 0x3B3 ramp.
  bool motorWasRun;
  uint32_t runMs;
};

#endif /* BMW_E92_h */
