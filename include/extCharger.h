
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

#ifndef EXTCHARGER_H
#define EXTCHARGER_H

/*  This library supports the various opensource tesla charger controllers e.g.
 * PCS , Gen2/3 etc. */

#include "chargerhw.h"
#include "digio.h"
#include "iomatrix.h"
#include <stdint.h>

class extCharger : public Chargerhw {

public:
  bool ControlCharge(bool RunCh, bool ACReq);

private:
};

#endif /* EXTCHARGER_H */
