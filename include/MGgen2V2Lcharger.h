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

/* Control of the Mitsubishi Outlander PHEV on board charger (OBC) and DCDC
 * Converter. */

#include "chargerhw.h"
#include "hwinit.h"
#include "my_fp.h"
#include "my_math.h"
#include "params.h"
#include <libopencm3/stm32/timer.h>
#include <stdint.h>

class MGgen2V2Lcharger : public Chargerhw {

public:
  void DecodeCAN(int id, uint32_t data[2]);
  void Task100Ms();
  void Off();
  bool ControlCharge(bool RunCh, bool ACReq);
  void SetCanInterface(CanHardware *c);

private:
  int opmode;
  uint16_t setVolts, actVolts, termAmps;
  int16_t actAmps;
  uint8_t currentRamp;
  bool clearToStart = false, shutDownReq = false, pwmON = false;
  static uint8_t chgStatus, evseDuty;
  static float dcBusV, temp_1, temp_2, ACVolts, DCAmps, ACAmps;
  static float LV_Volts, LV_Amps;
  static uint16_t batteryVolts;

  static void handle324(uint32_t data[2]);
  static void handle323(uint32_t data[2]);
  static void handle39F(uint32_t data[2]);
  // static void handle38A(uint32_t data[2]);
};

#endif // MGgen2V2Lcharger_H
