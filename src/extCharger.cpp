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

#include <extCharger.h>

static bool chargeAllow=false;

//this mode accepts a request for HV via a 12v inputfrom a charger controller e.g. Tesla Gen2/3 M3 PCS etc.



bool extCharger::ControlCharge(bool RunCh ,bool ACReq)
{
    bool dummy=RunCh;
    dummy=0;
    dummy=dummy;

    if(ACReq)
        {
            chargeAllow = IOMatrix::GetPin(IOMatrix::HVREQ)->Get();
            //enable charger digital line.
            IOMatrix::GetPin(IOMatrix::OBCENABLE)->Set();
            return chargeAllow;
        }
    else
        {
            //disable charger digital line when requested by timer or webui.
            IOMatrix::GetPin(IOMatrix::OBCENABLE)->Clear();
            return false;
        }
return false;
}


