/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2026 Tom de Bree <tom@voltinflux.com>
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
#ifndef TOYOTA_FIXTURES_H_INCLUDED
#define TOYOTA_FIXTURES_H_INCLUDED

/* Toyota sync serial frames captured from real inverters and HV ECUs, used as
 * test fixtures. Each comment says which capture a frame comes from. Every
 * frame passes the 16-bit checksum. */

#include <stdint.h>

// GS450h OEM HTM, "pulled from peak accel observed", from the inverterControl.h
// of an early Arduino Due GS450h gateway controller
static const uint8_t GS450H_HTM_PEAK_ACCEL[80] = {
    52,  0,   0,   0,   0,   96, 255, 44, 0,   247, 243, 96,  255, 0, 0,   2,
    2,   0,   144, 247, 112, 8,  2,   7,  124, 255, 219, 6,   42,  0, 64,  12,
    219, 6,   0,   0,   1,   2,  0,   0,  0,   0,   0,   0,   0,   0, 0,   0,
    0,   0,   0,   0,   0,   0,  0,   0,  0,   0,   0,   0,   0,   0, 24,  182,
    246, 155, 9,   0,   0,   0,  137, 0,  14,  1,   0,   157, 2,   0, 105, 14};

// GS450h OEM HTM, "seed for motor control", same source
static const uint8_t GS450H_HTM_SEED[80] = {
    163, 0,  0, 0, 0,   116, 254, 41, 0,   247, 243, 116, 254, 0, 0,   2,
    2,   0,  2, 2, 104, 4,   31,  5,  96,  254, 104, 4,   37,  0, 6,   34,
    104, 4,  0, 0, 1,   2,   0,   0,  0,   0,   0,   0,   0,   0, 0,   0,
    0,   0,  0, 0, 0,   0,   0,   0,  0,   0,   0,   0,   0,   0, 22,  139,
    246, 55, 9, 0, 0,   0,   137, 0,  221, 0,   0,   126, 6,   0, 121, 12};

// Prius Gen3 bench HTM, logic analyser capture (60 V HV, torque), frame 0
static const uint8_t PRIUS_HTM_BENCH_INIT[100] = {
    0, 14, 0, 0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0, 0,  4, 0, 0,  0,
    0, 0,  0, 0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0, 0,  0, 0, 0,  0,
    0, 4,  0, 0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0, 0,  0, 0, 0,  0,
    0, 0,  0, 0, 0, 0, 0,   4, 0, 0, 0,   0, 0, 4, 0, 25, 0, 0, 0,  0,
    0, 0,  0, 0, 0, 0, 136, 0, 0, 0, 160, 0, 0, 0, 0, 0,  0, 0, 95, 1};

// Auris MTH, drive log, valid frame 1800 (MG2 257 rpm)
static const uint8_t AURIS_MTH_1800[120] = {
    0,   12,  0,  0,   0,   0,  0,  0,   0, 0,   0,   255, 0,   0,   0,
    0,   68,  13, 0,   0,   15, 0,  48,  0, 210, 17,  1,   0,   12,  0,
    0,   48,  0,  0,   0,   4,  0,  0,   1, 1,   253, 0,   228, 255, 88,
    255, 240, 5,  0,   0,   17, 13, 123, 4, 0,   0,   27,  32,  32,  255,
    64,  136, 17, 0,   0,   0,  0,  0,   0, 0,   0,   0,   0,   0,   0,
    0,   0,   0,  0,   0,   0,  0,  0,   0, 0,   0,   0,   0,   0,   0,
    128, 0,   0,  2,   255, 1,  15, 0,   0, 0,   225, 1,   224, 1,   15,
    15,  0,   0,  225, 1,   0,  16, 0,   0, 206, 0,   0,   122, 105, 16};

// Auris MTH, same drive log, valid frame 2700 (MG2 -219 rpm)
static const uint8_t AURIS_MTH_2700[120] = {
    0,   12,  0,  0,   0,   0,   0,   255, 207, 0,   221, 0,   0,   0,   0,
    0,   4,   6,  0,   0,   15,  0,   80,  0,   184, 17,  1,   0,   12,  0,
    0,   48,  0,  0,   0,   239, 207, 0,   37,  255, 37,  255, 39,  255, 119,
    0,   64,  8,  0,   0,   17,  13,  195, 2,   0,   0,   25,  32,  32,  255,
    64,  184, 17, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    128, 0,   0,  2,   255, 1,   15,  0,   0,   0,   225, 1,   222, 1,   15,
    15,  0,   0,  225, 1,   0,   16,  0,   0,   206, 0,   0,   60,  193, 18};

// Prius Gen3 PHV (ZVW35) OEM HTM, car capture start-D-N-P-stop, frame 0 (init)
static const uint8_t PHV_HTM_INIT[105] = {
    0, 14, 0, 0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0, 0,  4, 0, 0, 0,  0,
    0, 0,  0, 0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0, 0,  0, 0, 0, 0,  0,
    4, 0,  0, 0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0, 0,  0, 0, 0, 0,  0,
    0, 0,  0, 0, 0, 0, 4,   0, 0, 0, 0,   0, 0, 4, 0, 25, 0, 0, 0, 0,  0,
    0, 0,  0, 0, 0, 0, 136, 0, 0, 0, 160, 0, 0, 0, 0, 0,  0, 0, 0, 95, 1};

// Prius PHV OEM HTM, same capture, frame 400 (ready, P)
static const uint8_t PHV_HTM_READY[105] = {
    0,   14, 0,   2,  0,  0,   0,   0,   0,   0,   0,  0, 0, 23,  0, 97, 0, 0,
    0,   0,  0,   0,  0,  136, 249, 120, 6,   0,   0,  0, 0, 0,   0, 22, 0, 0,
    0,   0,  0,   23, 0,  1,   0,   0,   0,   0,   0,  0, 0, 0,   0, 0,  0, 0,
    0,   0,  0,   0,  0,  0,   0,   0,   0,   0,   0,  0, 0, 0,   0, 4,  0, 0,
    0,   0,  0,   23, 0,  69,  22,  246, 251, 150, 14, 0, 0, 23,  0, 0,  0, 0,
    201, 0,  218, 0,  16, 0,   0,   0,   200, 0,   23, 0, 0, 105, 8};

// Prius PHV OEM HTM, same capture, frame 3500 (D, MG2 torque 131)
static const uint8_t PHV_HTM_DRIVE[105] = {
    0,   14,  0,   2,  0,  0,   0,   0,  0,   0,   0,  0,   0, 23,  0, 97, 0, 0,
    0,   0,   0,   0,  0,  248, 254, 8,  1,   131, 0,  131, 0, 131, 0, 22, 0, 0,
    0,   131, 0,   24, 0,  3,   128, 0,  0,   0,   0,  0,   0, 0,   0, 0,  0, 0,
    0,   0,   0,   0,  0,  0,   0,   0,  0,   0,   0,  0,   0, 0,   0, 4,  0, 0,
    0,   0,   0,   23, 0,  75,  22,  94, 250, 184, 14, 0,   0, 23,  0, 0,  0, 0,
    201, 0,   218, 0,  16, 0,   0,   0,  214, 0,   0,  0,   0, 126, 10};

// Prius PHV OEM MTH, same capture, frame 3100 (D, MG2 moving)
static const uint8_t PHV_MTH_DRIVE[140] = {
    0,   12, 0,   0,   0,   0,   0,  0,  0,   0,   22,  0,   0,   0,   0,   0,
    184, 9,  0,   0,   18,  0,   48, 0,  184, 17,  0,   0,   0,   0,   0,   0,
    0,   12, 0,   0,   48,  0,   0,  0,  0,   0,   0,   45,  255, 114, 255, 0,
    0,   0,  0,   152, 7,   0,   0,  19, 23,  178, 253, 0,   0,   6,   0,   32,
    0,   64, 184, 17,  0,   0,   0,  0,  0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,  0,   0,   0,   0,   0,  0,  0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,  0,   0,   128, 0,   0,  0,  0,   0,   0,   0,   2,   255, 1,   15,
    0,   0,  0,   209, 1,   202, 1,  17, 17,  0,   0,   209, 1,   0,   16,  0,
    0,   0,  0,   0,   206, 0,   0,  14, 0,   0,   124, 13};

#endif // TOYOTA_FIXTURES_H_INCLUDED
