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
#ifndef SUBARUVEHICLE_H
#define SUBARUVEHICLE_H

#include <vehicle.h>


class SubaruVehicle : public Vehicle
{
   public:
      /** Default constructor */
      SubaruVehicle();
      void SetCanInterface(CanHardware* c);
      bool GetGear(gear& gear);
      bool Ready() { return true; }
      void SetRevCounter(int speed);
      void SetTemperatureGauge(float temp);
      void SetFuelGauge(float level);
      float GetFrontRearBalance();
      int GetCruiseState();
      bool EnableTractionControl();

   protected:

   private:
      gear lastGear;
      int timerPeriod;
      float frontRearBalance;
      bool tcOn;
      bool ccOn;
};

#endif // SUBARUVEHICLE_H
