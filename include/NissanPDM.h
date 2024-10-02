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
#ifndef NISSANPDM_H
#define NISSANPDM_H

#include <stdint.h>
#include "my_fp.h"
#include "chargerhw.h"

class NissanPDM: public Chargerhw
{
public:
   void DecodeCAN(int id, const uint8_t bytes[8]) override;
   void Task10Ms();
   void Task100Ms();
   bool ControlCharge(bool RunCh, bool ACReq);
//   void SetTorque(float torque);
//   float GetMotorTemperature() { return motor_temp; }
//   float GetInverterTemperature() { return inv_temp; }
//   float GetInverterVoltage() { return voltage / 2; }
//   float GetMotorSpeed() { return speed / 2; }
//   int GetInverterState() { return error; }
   void SetCanInterface(CanHardware* c);

private:
   static void nissan_crc(uint8_t *data, uint8_t polynomial);
   static int8_t fahrenheit_to_celsius(uint16_t fahrenheit);
//   uint32_t lastRecv;
//   int16_t speed;
//   int16_t inv_temp;
//   int16_t motor_temp;
//   bool error;
//   uint16_t voltage;
//   int16_t final_torque_request;
};

#endif // NISSANPDM_H

