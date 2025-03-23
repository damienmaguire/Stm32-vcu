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

#ifndef amperacharger_h
#define amperacharger_h

/*  TODO: Add can info for control of Chevy Volt , Opel/Vauxhaul Ampera charger here. */

#include <stdint.h>
#include "my_fp.h"
#include "chargerhw.h"

class amperaCharger: public Chargerhw
{
public:
  bool ControlCharge(bool RunCh, bool ACreq);
};

#endif /* amperacharger_h */



