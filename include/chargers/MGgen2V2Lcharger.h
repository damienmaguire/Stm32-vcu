/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2021-2023  Johannes Huebner <dev@johanneshuebner.com>
 * 	                        Damien Maguire <info@evbmw.com>
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
 *
 */

#ifndef MGgen2V2Lcharger_H
#define MGgen2V2Lcharger_H

/* Control of the MG Gen2 on board charger (OBC) and DCDC converter. */

#include "chargers/chargerhw.h"
#include "hwinit.h"
#include "my_fp.h"
#include "my_math.h"
#include "params.h"
#include <stdint.h>

class MGgen2V2Lcharger : public Chargerhw {

public:
  void DecodeCAN(int id, uint32_t data[2]);
  void Task100Ms();
  void Off();
  bool ControlCharge(bool RunCh, bool ACReq);
  void SetCanInterface(CanHardware *c);

private:
  enum ChargeState : uint8_t {
    CHG_IDLE,     // not charging
    CHG_RUNNING,  // charging, current limit ramp and CV taper
    CHG_STOPPING, // 0 A, waiting for the OBC to stop
    CHG_DONE      // finished by the driver: no restart until unplugged
  };

  bool clearToStart = false;
  uint8_t offTick = 0;     // Off() calls left to skip before the next send
  uint16_t limitRaw = 0;   // 0x29C current limit sent, 0 = not charging
  float cvLimitA = 0;      // CV taper limit in A
  bool stopCharge = false; // hard stop or lost HV reading: 0 A until the end
  ChargeState chargeState = CHG_IDLE;
  bool latchAfterStop = false; // stop ends in CHG_DONE rather than CHG_IDLE
  bool completeSent = false;   // charge complete frames go out until OFF
  uint8_t stopTicks = 0;       // 100 ms ticks since the stop began
  uint16_t cvLowTicks = 0;     // 100 ms ticks with the CV limit at its end
  static uint8_t chgStatus, evseDuty;
  static float dcBusV, temp_1, temp_2, ACVolts, DCAmps, ACAmps;
  static float LV_Volts, LV_Amps;

  static bool PlugPresent();
  void UpdateCvLimit();
  void UpdateChargeState(int opmode);
  void StartStop(bool latch);

  static void handle324(uint32_t data[2]);
  static void handle323(uint32_t data[2]);
  static void handle33B(uint32_t data[2]);
  static void handle33D(uint32_t data[2]);
  static void handle39F(uint32_t data[2]);
  // static void handle38A(uint32_t data[2]);
};

#endif // MGgen2V2Lcharger_H
