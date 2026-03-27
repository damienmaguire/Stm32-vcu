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

#ifndef i3LIM_h
#define i3LIM_h

/*  This library supports the BMW i3 LIM charge interface module. */

#include "canhardware.h"
#include "chargerint.h"
#include <stdint.h>

class i3LIMClass : public Chargerint {
public:
  void SetCanInterface(CanHardware *c) override;
  void DecodeCAN(int id, uint32_t *data) override;
  void Task10Ms() override;
  void Task100Ms() override;
  void Task200Ms() override;
  bool DCFCRequest(bool RunCh) override;
  bool ACRequest(bool RunCh) override;

private:
  void handle3B4(uint32_t data[2]);
  void handle29E(uint32_t data[2]);
  void handle2B2(uint32_t data[2]);
  void handle2EF(uint32_t data[2]);
  void handle272(uint32_t data[2]);
  void CCS_Pwr_Con();
  void Chg_Timers();
};

#endif /* i3LIM_h */
