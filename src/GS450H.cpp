/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2019-2022 Damien Maguire <info@evbmw.com>
 * Changes by Tom de Bree <tom@voltinflux.com> 2024
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

#include "GS450H.h"
#include "anain.h"
#include "hwinit.h"
#include "my_math.h"
#include "temp_meas.h"
#include "utils.h"
#include <libopencm3/stm32/timer.h>

#define LOW_Gear 0
#define HIGH_Gear 1
#define AUTO_Gear 2

#define GS450H 1
#define PRIUS 2
#define IS300H 3

static uint8_t DriveType = 0;
volatile int received = 0;
static uint8_t htm_state = 0;
static uint8_t inv_status = 1; // must be 1 for gs450h and gs300h
uint16_t counter;
// static uint16_t htm_checksum; //superseded by Checksum function
static uint8_t frame_count;
static uint8_t GearSW;
static int16_t mg1_torque, mg2_torque, speedSum;
static bool statusInv = 0;
static bool TorqueCut, ShiftInit = 0;
static int8_t TorqueShiftRamp = 0;
static uint8_t speedSum2;
static uint8_t gearAct, gearReq, gearStep = 0;

static void dma_read(uint8_t *data, int size);
static void dma_write(const uint8_t *data, int size);

// 80 bytes out and 100 bytes back in (with offset of 8 bytes.
static uint8_t mth_data[140];
static const uint8_t htm_data_setup[100] = {
    0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0,   0, 4, 0, 0,  0,
    0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 4, 0,  0,
    0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0,  0,
    4, 0, 25, 0, 0, 0, 0, 0, 0, 0, 128, 0, 0, 0, 128, 0, 0, 0, 37, 1};
static uint8_t htm_data[105] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static const uint8_t htm_data_GS300H[105] = {
    0,   14, 0,   2,  0,  0,   0,   0,  0,   0,   0,  0, 0, 23, 0, 97, 0, 0,
    0,   0,  0,   0,  0,  248, 254, 8,  1,   0,   0,  0, 0, 0,  0, 22, 0, 0,
    0,   0,  0,   23, 0,  1,   0,   0,  0,   0,   0,  0, 0, 0,  0, 0,  0, 0,
    0,   0,  0,   0,  0,  0,   0,   0,  0,   0,   0,  0, 0, 0,  0, 4,  0, 0,
    0,   0,  0,   23, 0,  75,  22,  47, 250, 137, 14, 0, 0, 23, 0, 0,  0, 0,
    201, 0,  218, 0,  16, 0,   0,   0,  29,  0,   0,  0, 0, 0,  0};
static const uint8_t htm_data_Prius[100] = {
    0, 30,  0,  2,   0,  0,   0,   55,  0,  128, 254, 0,  0,   40, 0,   97, 0,
    0, 0,   0,  0,   0,  136, 249, 120, 6,  143, 255, 50, 255, 48, 255, 49, 0,
    0, 0,   48, 255, 43, 0,   1,   128, 0,  0,   0,   0,  0,   0,  0,   0,  0,
    0, 0,   0,  0,   0,  0,   0,   0,   0,  0,   0,   0,  0,   0,  0,   0,  4,
    0, 0,   0,  0,   40, 0,   75,  33,  68, 246, 51,  8,  0,   0,  40,  0,  0,
    0, 201, 0,  222, 0,  16,  0,   0,   0,  211, 40,  0,  0,   62, 15};

#if 0
// Not currently used
static uint8_t htm_data_setup_auris[100]= {0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0x00, 0x00, 0x00, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5F, 0x01};
#endif
/*
uint8_t htm_data_init[7][100]=
{
    {0,14,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,4,0,25,0,0,0,0,0,0,0,0,0,0,136,0,0,0,160,0,0,0,0,0,0,0,95,1},
    {0,14,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,4,0,25,0,0,0,0,0,0,0,0,0,0,136,0,0,0,160,0,0,0,0,0,0,0,95,1},
    {0,30,0,0,0,0,0,18,0,154,250,0,0,0,0,97,4,0,0,0,0,0,173,255,82,0,0,0,0,0,0,0,16,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,4,75,25,60,246,52,8,0,0,0,0,0,0,138,0,0,0,168,0,0,0,1,0,0,0,72,7},
    {0,30,0,0,0,0,0,18,0,154,250,0,0,0,0,97,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,16,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,4,75,25,60,246,52,8,0,0,0,0,0,0,138,0,0,0,168,0,0,0,2,0,0,0,75,5},
    {0,30,0,0,0,0,0,18,0,154,250,0,0,0,0,97,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,16,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,4,75,25,60,246,52,8,0,0,0,0,0,0,138,0,0,0,168,0,0,0,2,0,0,0,75,5},
    {0,30,0,0,0,0,0,18,0,154,250,0,0,255,0,97,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,16,0,0,0,0,0,255,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,255,4,73,25,60,246,52,8,0,0,255,0,0,0,138,0,0,0,168,0,0,0,3,0,0,0,70,9},
    {0,30,0,2,0,0,0,18,0,154,250,0,0,16,0,97,0,0,0,0,0,0,200,249,56,6,165,0,136,0,63,0,16,0,0,0,63,0,16,0,3,128,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,16,0,75,12,45,248,21,6,0,0,16,0,0,0,202,0,211,0,16,0,0,0,134,16,0,0,130,10}
};
*/
uint8_t htm_data_init[7][100] = {
    {0, 14, 0, 0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0, 0,  4, 0, 0,  0,
     0, 0,  0, 0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0, 0,  0, 0, 0,  0,
     0, 4,  0, 0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0, 0,  0, 0, 0,  0,
     0, 0,  0, 0, 0, 0, 0,   4, 0, 0, 0,   0, 0, 4, 0, 25, 0, 0, 0,  0,
     0, 0,  0, 0, 0, 0, 136, 0, 0, 0, 160, 0, 0, 0, 0, 0,  0, 0, 95, 1},
    {0, 14, 0, 0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0, 0,  4, 0, 0,  0,
     0, 0,  0, 0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0, 0,  0, 0, 0,  0,
     0, 4,  0, 0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0, 0,  0, 0, 0,  0,
     0, 0,  0, 0, 0, 0, 0,   4, 0, 0, 0,   0, 0, 4, 0, 25, 0, 0, 0,  0,
     0, 0,  0, 0, 0, 0, 136, 0, 0, 0, 160, 0, 0, 0, 0, 0,  0, 0, 95, 1},
    {0, 30,  0, 0, 0, 0,   0,   18, 0,  154, 250, 0, 0, 0,  0, 97, 4,
     0, 0,   0, 0, 0, 173, 255, 82, 0,  0,   0,   0, 0, 0,  0, 16, 0,
     0, 0,   0, 0, 0, 0,   0,   4,  0,  0,   0,   0, 0, 0,  0, 0,  0,
     0, 0,   0, 0, 0, 0,   0,   0,  0,  0,   0,   0, 0, 0,  0, 0,  4,
     0, 0,   0, 0, 0, 4,   75,  12, 60, 251, 52,  4, 0, 0,  0, 0,  0,
     0, 138, 0, 0, 0, 168, 0,   0,  0,  1,   0,   0, 0, 72, 7},
    {0, 30, 0, 0, 0, 0, 0,   18, 0, 154, 250, 0, 0,  0, 0,  97, 4,  0,   0,  0,
     0, 0,  0, 0, 0, 0, 0,   0,  0, 0,   0,   0, 16, 0, 0,  0,  0,  0,   0,  0,
     0, 4,  0, 0, 0, 0, 0,   0,  0, 0,   0,   0, 0,  0, 0,  0,  0,  0,   0,  0,
     0, 0,  0, 0, 0, 0, 0,   4,  0, 0,   0,   0, 0,  4, 75, 12, 60, 251, 52, 4,
     0, 0,  0, 0, 0, 0, 138, 0,  0, 0,   168, 0, 0,  0, 2,  0,  0,  0,   75, 5},
    {0, 30, 0, 0, 0, 0, 0,   18, 0, 154, 250, 0, 0,  0, 0,  97, 4,  0,   0,  0,
     0, 0,  0, 0, 0, 0, 0,   0,  0, 0,   0,   0, 16, 0, 0,  0,  0,  0,   0,  0,
     0, 4,  0, 0, 0, 0, 0,   0,  0, 0,   0,   0, 0,  0, 0,  0,  0,  0,   0,  0,
     0, 0,  0, 0, 0, 0, 0,   4,  0, 0,   0,   0, 0,  4, 75, 12, 60, 251, 52, 4,
     0, 0,  0, 0, 0, 0, 138, 0,  0, 0,   168, 0, 0,  0, 2,  0,  0,  0,   75, 5},
    {0, 30,  0, 0, 0,   0,   0,  18, 0,  154, 250, 0, 0, 255, 0,   97, 4,
     0, 0,   0, 0, 0,   0,   0,  0,  0,  0,   0,   0, 0, 0,   0,   16, 0,
     0, 0,   0, 0, 255, 0,   0,  4,  0,  0,   0,   0, 0, 0,   0,   0,  0,
     0, 0,   0, 0, 0,   0,   0,  0,  0,  0,   0,   0, 0, 0,   0,   0,  4,
     0, 0,   0, 0, 255, 4,   73, 12, 60, 251, 52,  4, 0, 0,   255, 0,  0,
     0, 138, 0, 0, 0,   168, 0,  0,  0,  3,   0,   0, 0, 70,  9},
    {0, 30,  0,  2,   0,  0,   0,   18,  0,  154, 250, 0,   0, 16,  0,  97, 0,
     0, 0,   0,  0,   0,  200, 249, 56,  6,  165, 0,   136, 0, 63,  0,  16, 0,
     0, 0,   63, 0,   16, 0,   3,   128, 0,  0,   0,   0,   0, 0,   0,  0,  0,
     0, 0,   0,  0,   0,  0,   0,   0,   0,  0,   0,   0,   0, 0,   0,  0,  4,
     0, 0,   0,  0,   16, 0,   75,  12,  45, 251, 21,  4,   0, 0,   16, 0,  0,
     0, 202, 0,  211, 0,  16,  0,   0,   0,  134, 16,  0,   0, 130, 10}};

uint8_t htm_data_Init_GS300H[6][105] = {
    {0, 14, 0, 0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0, 0,  4, 0, 0, 0,  0,
     0, 0,  0, 0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0, 0,  0, 0, 0, 0,  0,
     4, 0,  0, 0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0, 0,  0, 0, 0, 0,  0,
     0, 0,  0, 0, 0, 0, 4,   0, 0, 0, 0,   0, 0, 4, 0, 25, 0, 0, 0, 0,  0,
     0, 0,  0, 0, 0, 0, 136, 0, 0, 0, 160, 0, 0, 0, 0, 0,  0, 0, 0, 95, 1},
    {0, 14, 0, 0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0, 0,  4, 0, 0, 0,  0,
     0, 0,  0, 0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0, 0,  0, 0, 0, 0,  0,
     4, 0,  0, 0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0, 0,  0, 0, 0, 0,  0,
     0, 0,  0, 0, 0, 0, 4,   0, 0, 0, 0,   0, 0, 4, 0, 25, 0, 0, 0, 0,  0,
     0, 0,  0, 0, 0, 0, 136, 0, 0, 0, 160, 0, 0, 0, 0, 0,  0, 0, 0, 95, 1},
    {0,   14, 0, 0, 0,   0,   0,   0,   0,   0,   0,  0, 0, 0,   0, 97, 4, 0,
     0,   0,  0, 0, 0,   173, 255, 82,  0,   0,   0,  0, 0, 0,   0, 22, 0, 0,
     0,   0,  0, 0, 0,   0,   4,   0,   0,   0,   0,  0, 0, 0,   0, 0,  0, 0,
     0,   0,  0, 0, 0,   0,   0,   0,   0,   0,   0,  0, 0, 0,   0, 4,  0, 0,
     0,   0,  0, 0, 4,   75,  25,  212, 254, 210, 15, 0, 0, 0,   0, 0,  0, 0,
     137, 0,  0, 0, 168, 0,   0,   0,   1,   0,   0,  0, 0, 220, 6},
    {0,   14, 0, 0, 0,   0,   0,   0,   0,   0,   0,  0, 0, 0,   0, 97, 4, 0,
     0,   0,  0, 0, 0,   173, 255, 82,  0,   0,   0,  0, 0, 0,   0, 22, 0, 0,
     0,   0,  0, 0, 0,   0,   4,   0,   0,   0,   0,  0, 0, 0,   0, 0,  0, 0,
     0,   0,  0, 0, 0,   0,   0,   0,   0,   0,   0,  0, 0, 0,   0, 4,  0, 0,
     0,   0,  0, 0, 4,   75,  25,  212, 254, 210, 15, 0, 0, 0,   0, 0,  0, 0,
     137, 0,  0, 0, 168, 0,   0,   0,   1,   0,   0,  0, 0, 220, 6},
    {0,   14, 0, 0, 0,   0,  0,  0,   0,   0,   0,  0, 0, 0,   0, 97, 4, 0,
     0,   0,  0, 0, 0,   0,  0,  0,   0,   0,   0,  0, 0, 0,   0, 22, 0, 0,
     0,   0,  0, 0, 0,   0,  4,  0,   0,   0,   0,  0, 0, 0,   0, 0,  0, 0,
     0,   0,  0, 0, 0,   0,  0,  0,   0,   0,   0,  0, 0, 0,   0, 4,  0, 0,
     0,   0,  0, 0, 4,   75, 25, 212, 254, 190, 15, 0, 0, 0,   0, 0,  0, 0,
     137, 0,  0, 0, 168, 0,  0,  0,   2,   0,   0,  0, 0, 203, 4},
    {0,   14, 0, 0, 0,   0,  0,  0,   0,   0,   0,  0, 0, 0,   0, 97, 4, 0,
     0,   0,  0, 0, 0,   0,  0,  0,   0,   0,   0,  0, 0, 0,   0, 22, 0, 0,
     0,   0,  0, 0, 0,   0,  4,  0,   0,   0,   0,  0, 0, 0,   0, 0,  0, 0,
     0,   0,  0, 0, 0,   0,  0,  0,   0,   0,   0,  0, 0, 0,   0, 4,  0, 0,
     0,   0,  0, 0, 4,   75, 25, 212, 254, 190, 15, 0, 0, 0,   0, 0,  0, 0,
     137, 0,  0, 0, 168, 0,  0,  0,   2,   0,   0,  0, 0, 203, 4}};

void GS450HClass::SetTorque(float torquePercent) {
  uint8_t MotorActive = Param::GetInt(Param::MotActive);
  if (DriveType == GS450H) {
    GS450Hgear(); // check if we need to shift - can modify torque limits so
                  // needs to ran before calculating requests

    if (!TorqueCut) // Cut torque only when shifting for now
    {
      mg1_torque =
          (torquePercent * 4375) /
          100.0f; // mg1 does not need shifting !!!verify max allowed request

      torquePercent = TorqueShiftRamp * torquePercent *
                      0.01; // multiply by the torque ramp for when shifting
      scaledTorqueTarget =
          (torquePercent * 3500) / 100.0f; // !!!verify max allowed request
      mg2_torque = this->scaledTorqueTarget;
      // mg1_torque = ((mg2_torque*5)/4); //no need to shift

      if (ShiftInit == true && TorqueShiftRamp > 0) {
        TorqueShiftRamp -=
            (5 * Param::GetFloat(Param::throtramp)); // ramp down 5 x throtramp
        if (TorqueShiftRamp < 0) {
          TorqueShiftRamp = 0; // if we go below 0 force it to zero to signify
                               // finishing ramp down
        }
      }

      if (TorqueShiftRamp < 100 &&
          ShiftInit == false) // ramp torque back in after shifting - Note this
                              // also runs on first power on so theoretically
                              // reduced throttle on start
      {
        TorqueShiftRamp += Param::GetFloat(
            Param::throtramp); // ramp back in 5% every time this is ran, every
                               // 10ms - Increased from 10.
        if (TorqueShiftRamp > 100) {
          TorqueShiftRamp = 100; // keep it limited to 100
        }
      }

      if (gear == 0) //!!!Low gear
      {
        if (torquePercent < 0) {
          mg2_torque *= 0.5;
          mg1_torque *= 0.5;
        }
      }
    } else {
      mg2_torque = 0;
      mg1_torque = 0;
    }

    if (MotorActive == 0) // Both motors
    {
      if (scaledTorqueTarget < 0)
        mg1_torque = 0;
    } else if (MotorActive == 1) // Only Mg1 active
    {
      mg2_torque = 0;
    } else if (MotorActive == 2) // Only Mg2 active
    {
      mg1_torque = 0;
    } else if (MotorActive == 3) // MG1 only at high torque
    {
      mg1_torque = 0;
      if (torquePercent > 50) // only have MG1 active above 50%
      {
        mg1_torque = utils::change(torquePercent, 50, 100, 0, 4375);
      }
    }
  } else if (DriveType == PRIUS) {
    if (!TorqueCut) // should not trigger at all, only used for shifting GS450h
    {
      scaledTorqueTarget = (torquePercent * 3500) / 100.0f;
      mg2_torque = this->scaledTorqueTarget;
      mg1_torque = ((mg2_torque * 5) / 4);

      if (MotorActive == 0) // Both motors
      {
        if (scaledTorqueTarget < 0)
          mg1_torque = 0;
      } else if (MotorActive == 1) // Only Mg1 active
      {
        mg2_torque = 0;
      } else if (MotorActive == 2) // Only Mg2 active
      {
        mg1_torque = 0;
      }
    } else {
      mg2_torque = 0;
      mg1_torque = 0;
    }
  } else if (DriveType == IS300H) {
    if (!TorqueCut) // should not trigger at all, only used for shifting GS450h
    {
      scaledTorqueTarget = (torquePercent * 3500) / 100.0f;
      mg2_torque = this->scaledTorqueTarget;
      mg1_torque = ((mg2_torque * 5) / 4);

      if (MotorActive == 0) // Both motors
      {
        if (scaledTorqueTarget < 0)
          mg1_torque = 0;
      } else if (MotorActive == 1) // Only Mg1 active
      {
        mg2_torque = 0;
      } else if (MotorActive == 2) // Only Mg2 active
      {
        mg1_torque = 0;
      } else if (MotorActive == 3) // MG1 only at high torque
      {
        mg1_torque = 0;
        if (torquePercent > 50) // only have MG1 active above 50%
        {
          mg1_torque = utils::change(torquePercent, 50, 100, 0, 4375);
        }
      }
    } else {
      mg2_torque = 0;
      mg1_torque = 0;
    }
  }
}

float GS450HClass::GetMotorTemperature() {
  int tmpmg1 =
      AnaIn::MG1_Temp.Get(); // in the gs450h case we must read the analog temp
                             // values from sensors in the gearbox
  int tmpmg2 = AnaIn::MG2_Temp.Get();

  float t1 = (tmpmg1 * (-0.02058758)) +
             56.56512898; // Trying a best fit line approach.
  float t2 = (tmpmg2 * (-0.02058758)) + 56.56512898;
  ;
  float tmpm = MAX(t1, t2); // which ever is the hottest gets displayed

  return float(tmpm);
}

// 100 ms code
void GS450HClass::Task100Ms() {
  if (DriveType == GS450H) {
    GS450Houtput();
  }
}

void GS450HClass::GS450Hgear() //!!! should be ran every 10ms - ran before
                               //! calculating torque request
{
  // Param::SetInt(Param::InvStat, GS450HClass::statusFB()); //update inverter
  // status on web interface
  gear = (Param::GetInt(Param::Gear));

  if (gear == 2) //!!!Auto Shifting always start in low gear when powered on
  {
    if (gearAct == 0 &&
        mg2_speed > 7000) // Shift up when in low gear and mg2 is over 7000rpm
    {
      gearReq = 1;                               // request high gear
    } else if (gearAct == 1 && mg2_speed < 2000) // Shift down when in high gear
                                                 // and mg2 is under 8000rpm
    {
      gearReq = 0; // request high gear
    }

    if (gearAct != gearReq) // check if we need to shift gears
    {
      // TorqueCut = true; //Cut Torque to motor
      // TorqueShiftRamp = 0; //Zero torque Limiter
      ShiftInit = true;
      if (TorqueShiftRamp == 0) {
        gearStep++; // increase gearStep by 1 adds 10ms delay before shifting
      }
      if (gearStep == 4) // wait some cycles before changing
      {
        gear = gearReq; // change the outputs
      } else if (gearStep == 5) {
        gearAct = gearReq; // we have now shifted gear
        gearStep = 0;      // reset shift loop
        // TorqueCut = false;//allow torque again
        ShiftInit = false; // Allow troque ramping we are done shifting
      }
    } else // no shifting needed so gear should be gearAct
    {
      gear = gearAct;
      ShiftInit = false; // always force it into false when not trying to shift.
                         // In case of exiting shifting boundries during process
    }
  }

  if (gear == 1) //!!!High gear
  {
    DigIo::SP_out.Clear();
    DigIo::SL1_out.Clear();
    DigIo::SL2_out.Clear();
  }

  if (gear == 0) //!!!Low gear
  {
    DigIo::SP_out.Clear();
    DigIo::SL1_out.Set();
    DigIo::SL2_out.Set();
  }

  if (gear == 3) //!!!High in FWD and Low in REV - Jamie Jones special
  {
    int dir = Param::GetInt(Param::dir);
    if (dir == -1) // reverse go low
    {
      DigIo::SP_out.Clear();
      DigIo::SL1_out.Set();
      DigIo::SL2_out.Set();
    } else // go high
    {
      DigIo::SP_out.Clear();
      DigIo::SL1_out.Clear();
      DigIo::SL2_out.Clear();
    }
  }
}

void GS450HClass::GS450Houtput() //!!! should be ran every 10ms
{
  if (Param::GetInt(Param::opmode) == MOD_OFF) {
    utils::GS450hOilPump(0);
  }

  if (Param::GetInt(Param::opmode) == MOD_RUN) {
    Param::SetInt(
        Param::Gear1,
        DigIo::gear1_in.Get()); // update web interface with status of gearbox
                                // PB feedbacks for diag purposes.
    Param::SetInt(Param::Gear2, DigIo::gear2_in.Get());
    Param::SetInt(Param::Gear3, DigIo::gear3_in.Get());
    GearSW = ((!DigIo::gear3_in.Get() << 2) | (!DigIo::gear2_in.Get() << 1) |
              (!DigIo::gear1_in.Get()));
    if (GearSW == 6)
      Param::SetInt(Param::GearFB, LOW_Gear); // set low gear
    if (GearSW == 5)
      Param::SetInt(Param::GearFB, HIGH_Gear); // set high gear
    utils::GS450hOilPump(Param::GetInt(
        Param::OilPump)); // toyota hybrid oil pump pwm to run set point
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Dilbert's code here
//////////////////////////////////////////////////////////////////////////////////////////////////////

void GS450HClass::SetPrius() {
  setTimerState(true); // start toyota timers
  if (htm_state < 5) {
    htm_state = 5;
    inv_status = 0; // must be 0 for prius
  }
  for (int i = 0; i < 100; i++)
    htm_data[i] = htm_data_Prius[i];
  DriveType = PRIUS;
}

void GS450HClass::SetGS450H() {
  setTimerState(true); // start toyota timers
  if (htm_state > 4) {
    htm_state = 0;
    inv_status = 1; // must be 1 for gs450h
  }
  DriveType = GS450H;
}

void GS450HClass::SetGS300H() {
  setTimerState(true); // start toyota timers
  if (htm_state < 10) {
    htm_state = 10;
  }
  inv_status = 0; // must be 0 for gs300h
  for (int i = 0; i < 105; i++)
    htm_data[i] = htm_data_GS300H[i];
  DriveType = IS300H;
}

uint8_t GS450HClass::VerifyMTHChecksum(uint16_t len) {

  uint16_t mth_checksum = 0;

  for (int i = 0; i < (len - 2); i++)
    mth_checksum += mth_data[i];

  if (mth_checksum == (mth_data[len - 2] | (mth_data[len - 1] << 8)))
    return 1;
  else
    return 0;
}

void GS450HClass::CalcHTMChecksum(uint16_t len) {
  uint16_t htm_checksum = 0;

  for (int i = 0; i < (len - 2); i++)
    htm_checksum += htm_data[i];
  htm_data[len - 2] = htm_checksum & 0xFF;
  htm_data[len - 1] = htm_checksum >> 8;
}

void GS450HClass::Task1Ms() {

  switch (htm_state) {
  case 0:
    dma_read(mth_data, 100); // read in mth data via dma. Probably need some
                             // kind of check dma complete flag here
    DigIo::req_out
        .Clear(); // HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin, 0);
    htm_state++;
    break;
  case 1:
    DigIo::req_out
        .Set(); // HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin, 1);

    if (inv_status == 0) {
      if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL7,
                                 DMA_TCIF)) // if the transfer complete flag is
                                            // set then send another packet
      {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL7,
                                  DMA_TCIF); // clear the flag.
        dma_write(htm_data, 80); // HAL_UART_Transmit_IT(&huart2, htm_data, 80);
      }

    } else {
      dma_write(htm_data_setup,
                80); // HAL_UART_Transmit_IT(&huart2, htm_data_setup, 80);
      if (mth_data[1] != 0)
        inv_status--;
      if (mth_data[1] == 0)
        inv_status = 1;
    }
    htm_state++;
    break;
  case 2:
    htm_state++;
    break;
  case 3:
    if (VerifyMTHChecksum(100) == 0 ||
        dma_get_interrupt_flag(DMA1, DMA_CHANNEL6, DMA_TCIF) == 0) {
      statusInv = 0;
      // set speeds to 0 to prevent dynamic throttle/regen issues
      mg1_speed = 0;
      mg2_speed = 0;
      // disable cruise
      Param::SetInt(Param::cruisespeed, 0);
    } else {
      // exchange data and prepare next HTM frame
      dma_clear_interrupt_flags(DMA1, DMA_CHANNEL6, DMA_TCIF);
      statusInv = 1;
      dc_bus_voltage = (((mth_data[82] | mth_data[83] << 8) - 5) / 2);
      temp_inv_water = int8_t(mth_data[42]);
      temp_inv_inductor = int8_t(mth_data[86]);
      mg1_speed = mth_data[6] | mth_data[7] << 8;
      mg2_speed = mth_data[31] | mth_data[32] << 8;
    }

    mth_data[98] = 0;
    mth_data[99] = 0;

    htm_state++;
    break;
  case 4:
    // -3500 (reverse) to 3500 (forward)
    Param::SetInt(Param::torque,
                  mg2_torque); // post processed final torue value sent to inv
                               // to web interface

    // speed feedback
    speedSum = mg2_speed + mg1_speed;
    speedSum /= 113;
    speedSum2 = speedSum;
    htm_data[0] = speedSum2;
    htm_data[75] = (mg1_torque * 4) & 0xFF;
    htm_data[76] = ((mg1_torque * 4) >> 8) & 0xFF;

    // mg1
    htm_data[5] = (-mg1_torque) & 0xFF; // negative is forward
    htm_data[6] = ((-mg1_torque) >> 8);
    htm_data[11] = htm_data[5];
    htm_data[12] = htm_data[6];

    // mg2
    htm_data[26] = (mg2_torque) & 0xFF; // positive is forward
    htm_data[27] = ((mg2_torque) >> 8) & 0xFF;
    htm_data[32] = htm_data[26];
    htm_data[33] = htm_data[27];

    htm_data[63] = (-5000) & 0xFF; // regen ability of battery
    htm_data[64] = ((-5000) >> 8);

    htm_data[65] = (27500) & 0xFF; // discharge ability of battery
    htm_data[66] = ((27500) >> 8);

    //!!moved to checksum function.
    /*
    htm_checksum=0;

    for (int i = 0; i < 78; i++)
        htm_checksum += htm_data[i];

    htm_data[78]=htm_checksum&0xFF;
    htm_data[79]=htm_checksum>>8;
    */
    CalcHTMChecksum(80);

    if (counter > 100) {
      // HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin );
      counter = 0;
    } else {
      counter++;
    }

    htm_state = 0;
    break;

  /***** Demo code for Gen3 Prius/Auris direct communications! */
  case 5:
    dma_read(mth_data, 120); // read in mth data via dma. Probably need some
                             // kind of check dma complete flag here
    DigIo::req_out
        .Clear(); // HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin, 0);
    htm_state++;
    break;
  case 6:
    DigIo::req_out
        .Set(); // HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin, 1);

    if (inv_status > 5) {
      if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL7,
                                 DMA_TCIF)) // if the transfer complete flag is
                                            // set then send another packet
      {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL7,
                                  DMA_TCIF); // clear the flag.
        dma_write(htm_data, 100); // HAL_UART_Transmit_IT(&huart2, htm_data,
                                  // 80);
      }
    } else {
      dma_write(&htm_data_init[inv_status][0],
                100); // HAL_UART_Transmit_IT(&huart2, htm_data_setup, 80);

      inv_status++;
      if (inv_status == 6) {
        // memcpy(htm_data, &htm_data_init[ inv_status ][0], 100);
      }
    }
    htm_state++;
    break;
  case 7:
    htm_state++;
    break;
  case 8:
    if (VerifyMTHChecksum(120) == 0 ||
        dma_get_interrupt_flag(DMA1, DMA_CHANNEL6, DMA_TCIF) == 0) {

      statusInv = 0;
      // set speeds to 0 to prevent dynamic throttle/regen issues
      mg1_speed = 0;
      mg2_speed = 0;
      // disable cruise
      Param::SetInt(Param::cruisespeed, 0);
    } else {

      // exchange data and prepare next HTM frame
      dma_clear_interrupt_flags(DMA1, DMA_CHANNEL6, DMA_TCIF);
      statusInv = 1;
      dc_bus_voltage = (((mth_data[100] | mth_data[101] << 8) - 5) / 2);
      temp_inv_water = int8_t(mth_data[20]); // from 300h
      temp_inv_inductor = (mth_data[86] | mth_data[87] << 8);
      mg1_speed = mth_data[6] | mth_data[7] << 8;
      mg2_speed = mth_data[38] | mth_data[39] << 8;
    }

    mth_data[98] = 0;
    mth_data[99] = 0;

    htm_state++;
    break;
  case 9:
    Param::SetInt(Param::torque,
                  mg2_torque); // post processed final torue value sent to inv
                               // to web interface

    // speed feedback
    speedSum = mg2_speed + mg1_speed;
    speedSum /= 113;
    // Possibly not needed
    // uint8_t speedSum2=speedSum;
    // htm_data[0]=speedSum2;

    // these bytes are used, and seem to be MG1 for startup, but can't work out
    // the relatino to the bytes earlier in the stream, possibly the byte order
    // has been flipped on these 2 bytes could be a software bug ?
    // htm_data[76]=(mg1_torque*4) & 0xFF; //Possibly wrong
    // htm_data[75]=((mg1_torque*4)>>8) & 0xFF; //Possibly wrong

    // mg1
    htm_data[5] = (mg1_torque) & 0xFF; // negative is forward
    htm_data[6] = ((mg1_torque) >> 8);
    htm_data[11] = htm_data[5];
    htm_data[12] = htm_data[6];

    // mg2 the MG2 values are now beside each other!
    htm_data[30] = (mg2_torque) & 0xFF; // positive is forward
    htm_data[31] = ((mg2_torque) >> 8) & 0xFF;

    if (scaledTorqueTarget > 0) {
      // forward direction these bytes should match
      htm_data[26] = htm_data[30];
      htm_data[27] = htm_data[31];
      htm_data[28] = (mg2_torque / 2) & 0xFF; // positive is forward
      htm_data[29] = ((mg2_torque / 2) >> 8) & 0xFF;
    }

    if (scaledTorqueTarget < 0) {
      // reverse direction these bytes should match
      htm_data[28] = htm_data[30];
      htm_data[29] = htm_data[31];
      htm_data[26] = (mg2_torque / 2) & 0xFF; // positive is forward
      htm_data[27] = ((mg2_torque / 2) >> 8) & 0xFF;
    }

    // Battery Limits
    /*
            htm_data[85]=(-5000)&0xFF;  // regen ability of battery !!!increased
            htm_data[86]=((-5000)>>8);

            htm_data[87]=(-10000)&0xFF;  // discharge ability of battery
       !!!Remove negative and increased htm_data[88]=((-10000)>>8);
    */
    // Battery Limits = forced zero
    // 40	4	75	12	60	251	52	4

    htm_data[72] = 0x40;
    htm_data[73] = 0x04;
    htm_data[74] = 0x75;
    htm_data[75] = 0x12;
    htm_data[76] = 0x60;
    htm_data[77] = 0x25;
    htm_data[78] = 0x52;
    htm_data[79] = 0x04;

    htm_data[86] = 137; // from start up
    htm_data[88] = 137; // 221 on start

    // checksum
    if (++frame_count & 0x01) {
      // b94_count++;
      htm_data[94]++;
    }

    CalcHTMChecksum(100);

    htm_state = 5;
    break;

  /***** Code for Lexus GS300H */
  case 10:
    if (Param::GetInt(Param::opmode) != MOD_RUN)
      inv_status = 0;
    dma_read(mth_data, 140); // read in mth data via dma.
    DigIo::req_out
        .Clear(); // HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin, 0);
    htm_state++;
    break;
  case 11:
    DigIo::req_out
        .Set(); // HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin, 1);
    if (inv_status > 6) {
      if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL7,
                                 DMA_TCIF)) // if the transfer complete flag is
                                            // set then send another packet
      {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL7,
                                  DMA_TCIF); // clear the flag.
        dma_write(htm_data, 105); // HAL_UART_Transmit_IT(&huart2, htm_data,
                                  // 80);
      }
    } else {
      dma_write(&htm_data_Init_GS300H[inv_status][0],
                105); // HAL_UART_Transmit_IT(&huart2, htm_data_setup, 80);

      inv_status++;
    }
    htm_state++;
    break;
  case 12:
    htm_state++;
    break;
  case 13:
    if (VerifyMTHChecksum(140) == 0 ||
        dma_get_interrupt_flag(DMA1, DMA_CHANNEL6, DMA_TCIF) == 0) {

      statusInv = 0;
      // inv_status=0; Stop reinit of inverter
      /* PART OF 2.20A Changes that stopped working
      //inv_status=0; Stop reinit of inverter
      //set speeds to 0 to prevent dynamic throttle/regen issues
      mg1_speed=0;
      mg2_speed=0;
      //disable cruise
      Param::SetInt(Param::cruisespeed, 0);
      */
    } else {

      // exchange data and prepare next HTM frame
      dma_clear_interrupt_flags(DMA1, DMA_CHANNEL6, DMA_TCIF);
      statusInv = 1;
      dc_bus_voltage = (((mth_data[117] | mth_data[118] << 8)) / 2);
      temp_inv_water = int8_t(mth_data[20]);
      temp_inv_inductor = (mth_data[25] | mth_data[26] << 8);
      mg1_speed = mth_data[10] | mth_data[11] << 8;
      mg2_speed = mth_data[43] | mth_data[44] << 8;
    }

    // mth_data[98]=0;
    // mth_data[99]=0;

    htm_state++;
    break;
  case 14:
    Param::SetInt(Param::torque,
                  mg2_torque); // post processed final torue value sent to inv
                               // to web interface

    // speed feedback
    speedSum = mg2_speed + mg1_speed;
    speedSum /= 113;

    // mg1
    htm_data[5] = (mg1_torque * -1) & 0xFF; // negative is forward
    htm_data[6] = ((mg1_torque * -1) >> 8);
    htm_data[11] = htm_data[5];
    htm_data[12] = htm_data[6];

    // mg2
    htm_data[31] = (mg2_torque) & 0xFF; // positive is forward
    htm_data[32] = ((mg2_torque) >> 8);
    htm_data[37] = htm_data[26];
    htm_data[38] = htm_data[27];

    // Battery Limits

    htm_data[79] = (-5000) & 0xFF; // regen ability of battery
    htm_data[80] = ((-5000) >> 8);

    htm_data[81] = (10000) & 0xFF; // discharge ability of battery
    htm_data[82] = ((10000) >> 8);
    CalcHTMChecksum(105);

    htm_state = 10;
    break;
  }
}

void GS450HClass::setTimerState(bool desiredTimerState) {
  if (desiredTimerState != this->timerIsRunning) {
    if (desiredTimerState) {
      if (Param::GetInt(Param::PumpPWM) ==
          0) // If Pump PWM out is set to Oil Pump
      {
        tim_setup(); // trigger pump pwm out setup
      }
      tim2_setup();                // TOYOTA HYBRID INVERTER INTERFACE CLOCK
      this->timerIsRunning = true; // timers are now running
    } else {
      // These are only used with the Totoa hybrid option.
      timer_disable_counter(TIM2);  // TOYOTA HYBRID INVERTER INTERFACE CLOCK
      this->timerIsRunning = false; // timers are now stopped
    }
  }
}

int GS450HClass::GetInverterState() { return statusInv; }
//////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Usart 2 DMA Transmitt and Receive Section
//////////////////////////////////////////////////////////////////////////

static void dma_write(const uint8_t *data, int size) {
  /*
   * Using channel 7 for USART2_TX
   */

  /* Reset DMA channel*/
  dma_channel_reset(DMA1, DMA_CHANNEL7);

  dma_set_peripheral_address(DMA1, DMA_CHANNEL7, (uint32_t)&USART2_DR);
  dma_set_memory_address(DMA1, DMA_CHANNEL7, (uint32_t)data);
  dma_set_number_of_data(DMA1, DMA_CHANNEL7, size);
  dma_set_read_from_memory(DMA1, DMA_CHANNEL7);
  dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL7);
  dma_set_peripheral_size(DMA1, DMA_CHANNEL7, DMA_CCR_PSIZE_8BIT);
  dma_set_memory_size(DMA1, DMA_CHANNEL7, DMA_CCR_MSIZE_8BIT);
  dma_set_priority(DMA1, DMA_CHANNEL7, DMA_CCR_PL_MEDIUM);

  dma_enable_channel(DMA1, DMA_CHANNEL7);

  usart_enable_tx_dma(USART2);
}

static void dma_read(uint8_t *data, int size) {
  /*
   * Using channel 6 for USART2_RX
   */

  /* Reset DMA channel*/
  dma_channel_reset(DMA1, DMA_CHANNEL6);

  dma_set_peripheral_address(DMA1, DMA_CHANNEL6, (uint32_t)&USART2_DR);
  dma_set_memory_address(DMA1, DMA_CHANNEL6, (uint32_t)data);
  dma_set_number_of_data(DMA1, DMA_CHANNEL6, size);
  dma_set_read_from_peripheral(DMA1, DMA_CHANNEL6);
  dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL6);
  dma_set_peripheral_size(DMA1, DMA_CHANNEL6, DMA_CCR_PSIZE_8BIT);
  dma_set_memory_size(DMA1, DMA_CHANNEL6, DMA_CCR_MSIZE_8BIT);
  dma_set_priority(DMA1, DMA_CHANNEL6, DMA_CCR_PL_LOW);

  dma_enable_channel(DMA1, DMA_CHANNEL6);

  usart_enable_rx_dma(USART2);
}
