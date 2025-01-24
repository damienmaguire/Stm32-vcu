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

#ifndef GS450H_h
#define GS450H_h

/*  Communication with Lexus GS450H and potentiall any Toyota Hybrid inverter using sync serial */

#include <stdint.h>
#include "my_fp.h"
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include "digio.h"
#include "params.h"
#include "inverter.h"

#define MG2MAXSPEED 10000
#define MAX_COMMAND_SIZE 200

class GS450HClass: public Inverter
{

public:
   GS450HClass()
   {
       scaledTorqueTarget = 0;
       timerIsRunning = false;
   }
   void Task1Ms();
   void Task100Ms();
   void SetTorque(float torquePercent);
   float GetMotorTemperature();
   float GetInverterTemperature() { return temp_inv_water; }
   float GetInverterVoltage() { return dc_bus_voltage; }
   float GetMotorSpeed() { return mg2_speed; }
   int GetInverterState();
   void DeInit() { setTimerState(false); } //called when switching to another inverter, similar to a destructor


   //Lexus/Toyota specific functions
   void SetPrius();
   void SetGS450H();
   void SetGS300H();
   void SetGear(int16_t g) { gear = g; }
   void SetOil(int16_t o) { oil = o; }

private:
   int16_t dc_bus_voltage, mg1_speed, mg2_speed, gear, oil;
   float temp_inv_water, temp_inv_inductor;
   bool timerIsRunning;
   int scaledTorqueTarget;
   uint8_t VerifyMTHChecksum(uint16_t );
   void CalcHTMChecksum(uint16_t);
   void setTimerState(bool);
   void GS450Hgear();
   void GS450Houtput();
};

#endif /* GS450H_h */
