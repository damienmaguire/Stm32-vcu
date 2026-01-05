/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2023 Damien Maguire
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

#ifndef DIGIPOT_H
#define DIGIPOT_H
#include "digio.h"
#include "params.h"
#include <libopencm3/stm32/spi.h>

namespace DigiPot {
void SetPot1Step();
void SetPot2Step();
} // namespace DigiPot
#endif // DIGIPOT_H
