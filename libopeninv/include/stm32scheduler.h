/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2017 Johannes Huebner <contact@johanneshuebner.com>
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
#ifndef STM32SCHEDULER_H
#define STM32SCHEDULER_H
#include <stdint.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>

#define MAX_TASKS 4

/** @brief Schedules up to 4 tasks using a timer peripheral */
class Stm32Scheduler
{
   public:
      /** @brief construct a new scheduler using given timer
       * @pre Timer clock and NVIC interrupt must be enabled
       * @param timer Address of timer peripheral to use
       */
      Stm32Scheduler(uint32_t timer);

      /** @brief Add a periodic task, can be called up to 4 times
       * @param function the task function
       * @param period The calling period in 100*ms
       */
      void AddTask(void (*function)(void), uint16_t period);

      /** @brief Run the scheduler, must be called by the scheduler timer ISR */
      void Run();

      /** @brief Return CPU load caused by scheduler tasks
       * @return load in 0.1%
       */
      int GetCpuLoad();

   protected:
   private:
      static void nofunc(void);
      static const enum tim_oc_id ocMap[MAX_TASKS];
      void (*functions[MAX_TASKS]) (void);
      uint16_t periods[MAX_TASKS];
      uint16_t execTicks[MAX_TASKS];
      uint32_t timer;
      int nextTask;
};

#endif // STM32SCHEDULER_H
