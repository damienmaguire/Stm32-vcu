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
 */

#ifndef OUTLANDERCANHEATER_H
#define OUTLANDERCANHEATER_H

//#include <libopencm3/stm32/usart.h>
#include <heater.h>


class OutlanderCanHeater : public Heater
{
   public:
      void SetTargetTemperature(float temp);
      void SetCanInterface(CanHardware* c);
      void DecodeCAN(int id, uint32_t data[2]);
      void SetPower(uint16_t power, bool HeatReq);
      void Task100Ms();

   private:
      int8_t currentTemperature;
      int8_t desiredTemperature;

      bool shouldHeat;
      static void handle398(uint32_t data[2]);

};

#endif // OUTLANDERCANHEATER_H
