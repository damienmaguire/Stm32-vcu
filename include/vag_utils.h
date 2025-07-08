/*
 * This file is part of the ZombieVeter project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *               2021-2022 Damien Maguire <info@evbmw.com>
 *               2024 Mitch Elliott
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

 #ifndef VAG_UTILS_H
 #define VAG_UTILS_H
 
 #include <stdint.h>
 #include "my_fp.h"
 #include "my_math.h"
 
 namespace vag_utils
 {
    uint8_t vw_crc_calc(uint8_t* inputBytes, uint8_t length, uint16_t address);
    uint8_t vw_crc_calc_MQB(uint8_t* inputBytes, uint8_t length, uint16_t address);
 }
 
 #endif
 