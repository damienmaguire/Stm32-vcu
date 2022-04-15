
/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *                      Damien Maguire <info@evbmw.com>
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
#ifndef CAN_OI_H
#define CAN_OI_H
#include <stdint.h>
#include "my_fp.h"
#include "inverter.h"

class Can_OI: public Inverter
{
public:
   void Task100Ms();
   void DecodeCAN(int, uint32_t*);
   void SetTorque(float torquePercent);
   float GetMotorTemperature() { return motor_temp; }
   float GetInverterTemperature() { return inv_temp; }
   float GetInverterVoltage() { return voltage; }
   float GetMotorSpeed() { return speed; }
   int GetInverterState() { return error; }

private:
   static int16_t speed;
   static int16_t inv_temp;
   static int16_t motor_temp;
   static bool error;
   static uint16_t voltage;

   static uint8_t run100ms;
   static uint32_t lastRecv;
   static int16_t final_torque_request;
};

#endif // CAN_OI_H
