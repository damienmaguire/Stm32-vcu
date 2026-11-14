/*
 * This file is part of the ZombieVeter project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *               2021-2022 Damien Maguire <info@evbmw.com>
 * Yes I'm really writing software now........run.....run away.......
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

#include "utils.h"

#include "bmw_sbox.h"
#include "hwinit.h"
#include "iomatrix.h"
#include "isa_shunt.h"
#include "my_math.h"
#include "throttle.h"
#include "vag_sbox.h"
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/timer.h>

namespace utils {

#define CAN_TIMEOUT 1 // 1000ms

float SOCVal = 0;
int32_t NetWh = 0;

bool Timer1Run = false;

void PostErrorIfRunning(ERROR_MESSAGE_NUM err) {
  if (Param::GetInt(Param::opmode) == MOD_RUN) {
    ErrorMessage::Post(err);
  }
}

void GetDigInputs(CanHardware *can) {
  static bool canIoActive = false;
  int canio = Param::GetInt(Param::canio);

  canIoActive |= canio != 0;

  if ((rtc_get_counter_val() - can->GetLastRxTimestamp()) >= CAN_TIMEOUT &&
      canIoActive) {
    canio = 0;
    Param::SetInt(Param::canio, 0);
    ErrorMessage::Post(ERR_CANTIMEOUT);
  }

  Param::SetInt(Param::din_HVreq, DigIo::HV_req.Get());
  Param::SetInt(Param::din_start,
                DigIo::start_in.Get() | ((canio & CAN_IO_START) != 0));
  Param::SetInt(Param::din_brake,
                DigIo::brake_in.Get() | ((canio & CAN_IO_BRAKE) != 0));
  Param::SetInt(Param::din_forward,
                DigIo::fwd_in.Get() | ((canio & CAN_IO_FWD) != 0));
  Param::SetInt(Param::din_reverse,
                DigIo::rev_in.Get() | ((canio & CAN_IO_REV) != 0));
  Param::SetInt(Param::din_bms, (canio & CAN_IO_BMS) != 0);
  Param::SetInt(Param::din_12Vgp, DigIo::gp_12Vin.Get());
}

float ProcessUdc(int motorSpeed) {
  float udc = Param::GetFloat(Param::udc);

  if (Param::GetInt(Param::ShuntType) == 0) {
    if (Param::GetInt(Param::opmode) == MOD_OFF) {
      udc = 0;
      Param::SetFloat(Param::udc, udc);
    }
  } else if (Param::GetInt(Param::ShuntType) == 1 ||
             Param::GetInt(Param::ShuntType) == 4) {
    float udc2 = ((float)ISA::Voltage2) / 1000;
    Param::SetFloat(Param::udc2, udc2);
    if (Param::GetInt(Param::ShuntType) == 4) {
      if (udc2 > Param::GetFloat(Param::udcmin)) {
        Param::SetFloat(Param::udcsw, udc2 - 20);
      }
    }
  } else if (Param::GetInt(Param::ShuntType) == 2) {
    if (Param::GetInt(Param::opmode) != MOD_OFF) {
      float udc2 = ((float)SBOX::Voltage) / 1000;
      Param::SetFloat(Param::udc2, udc2);
      if (udc2 > Param::GetFloat(Param::udcmin)) {
        Param::SetFloat(Param::udcsw, udc2 - 20);
      }
    }
  }
  return udc;
}

} // namespace utils
