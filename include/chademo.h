/*
 * This file is part of the tumanako_vc project.
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
 */

#ifndef CHADEMO_H
#define CHADEMO_H
#include <stdint.h>
#include "my_math.h"
#include "my_fp.h"
#include "stm32_can.h"

class ChaDeMo
{
   public:
      static void Process108Message(uint32_t data[2]);
      static void Process109Message(uint32_t data[2]);
      /** Must be called every 100ms */
      static void SendMessages(Can* can);

      static void SetTargetBatteryVoltage(uint16_t vtg) { targetBatteryVoltage = vtg; }
      static void SetChargeCurrent(uint8_t cur);
      static void SetEnabled(bool enabled);
      /** Set vehicle in parking position, true=yes, 0=false */
      static void SetParkPosition(bool pos) { parkingPosition = !pos; }
      static void SetContactor(bool state) { contactorOpen = !state; }
      static void SetGeneralFault() { fault = true; }
      /** Set current state of charge */
      static void SetSoC(s32fp soC) { soc = soC >> (FRAC_DIGITS - 1); }
      static int GetChargerOutputVoltage() { return chargerOutputVoltage; }
      static int GetChargerOutputCurrent() { return chargerOutputCurrent; }
      static int GetChargerMaxCurrent() { return chargerMaxCurrent; }
      static int GetChargerStatus() { return chargerStatus; }
      static bool ConnectorLocked() { return (chargerStatus & 0x4) != 0; }
      static bool ChargerStopRequest() { return (chargerStatus & 0x2A) != 0; }
      static uint8_t GetRampedCurrentRequest() { return rampedCurReq; }
      /** Must be called every 100 ms */
      static void CheckSensorDeviation(uint16_t internalVoltage);
   protected:

   private:
      static bool chargeEnabled;
      static bool parkingPosition;
      static bool fault;
      static bool contactorOpen;
      static uint8_t chargerStatus;
      static uint8_t chargerMaxCurrent;
      static uint8_t chargeCurrentRequest;
      static uint32_t rampedCurReq;
      static uint16_t targetBatteryVoltage;
      static uint16_t chargerOutputVoltage;
      static uint8_t chargerOutputCurrent;
      static uint8_t soc;
      static uint32_t vtgTimeout;
      static uint32_t curTimeout;
};

#endif // CHADEMO_H
