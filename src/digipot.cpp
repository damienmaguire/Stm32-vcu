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

#include "digipot.h"

void DigiPot::SetPot1Step() {
    DigIo::pot1_cs.Clear();
    spi_xfer(SPI3, Param::GetInt(Param::DigiPot1Step));
    DigIo::pot1_cs.Set();
}

void DigiPot::SetPot2Step() {
    DigIo::pot2_cs.Clear();
    spi_xfer(SPI3, Param::GetInt(Param::DigiPot2Step));
    DigIo::pot2_cs.Set();
}