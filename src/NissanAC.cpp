/*
 * This file is part of the Zombieverter VCU project.
 *
 * Copyright (C) 2018 Johannes Huebner <dev@johanneshuebner.com>
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
 * Controls the Nissan / Denso ES27C Airconditioning compressor.
 * Nissan part number : 92600-3NF0A.
 *
 *Command ID 0x3B , 8 bytes. Compressor power in bytes 0 an 1.
 *
 * uint8_t data[8] = { 0xb2, 0x00, 0x00, 0x90, 0xff, 0x00, 0x00, 0x00 };
 * b[1] = 0x05 -> 1kw,  b[1] = 0x12 -> 2kw, b[1] = 0x16 -> 3kw
 *
 * if (compCmd > 0)
 *  {
 *    data[0] = 0xb3;
 *
 *    data[1] = compCmd;
 *  }
 *
 *
 */

 #include <NissanAC.h>
