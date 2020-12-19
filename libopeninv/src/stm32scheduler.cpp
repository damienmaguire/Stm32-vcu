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
#include "stm32scheduler.h"

/* return CCRc of TIMt */
#define TIM_CCR(t,c) (*(volatile uint32_t *)(&TIM_CCR1(t) + (c)))

const enum tim_oc_id Stm32Scheduler::ocMap[MAX_TASKS] = { TIM_OC1, TIM_OC2, TIM_OC3, TIM_OC4 };

Stm32Scheduler::Stm32Scheduler(uint32_t timer)
{
   this->timer = timer;
   /* Setup timers upcounting and auto preload enable */
   timer_enable_preload(timer);
   timer_direction_up(timer);
   /* Set prescaler to count at 100 kHz = 72 MHz/7200 - 1 */
   timer_set_prescaler(timer, 719);
   /* Maximum counter value */
   timer_set_period(timer, 0xFFFF);

   for (int i = 0; i < MAX_TASKS; i++)
   {
      functions[i] = nofunc;
      periods[i] = 0xFFFF;
   }

   nextTask = 0;
}

void Stm32Scheduler::AddTask(void (*function)(void), uint16_t period)
{
   if (nextTask >= MAX_TASKS) return;
   /* Disable timer */
   timer_disable_counter(timer);

   timer_set_oc_mode(timer, ocMap[nextTask], TIM_OCM_ACTIVE);
   timer_set_oc_value(timer, ocMap[nextTask], 0);

   /* Assign task function and period */
   functions[nextTask] = function;
   periods  [nextTask] = period * 100;

   /* Enable interrupt for that channel */
   timer_enable_irq(timer, TIM_DIER_CC1IE << nextTask);

   /* Reset counter */
   timer_set_counter(timer, 0);

   /* Enable timer */
   timer_enable_counter(timer);

   nextTask++;
}

void Stm32Scheduler::Run()
{
   for (int i = 0; i < MAX_TASKS; i++)
   {
      if (timer_get_flag(timer, TIM_SR_CC1IF << i))
      {
         uint16_t start = timer_get_counter(timer);
         timer_clear_flag(timer, TIM_SR_CC1IF << i);
         TIM_CCR(timer, i) += periods[i];
         functions[i]();
         execTicks[i] = timer_get_counter(timer) - start;
      }
   }
}

int Stm32Scheduler::GetCpuLoad()
{
   int totalLoad = 0;
   for (int i = 0; i < MAX_TASKS; i++)
   {
      int load = (10 * execTicks[i]) / periods[i];
      totalLoad += load;
   }
   return totalLoad;
}

void Stm32Scheduler::nofunc()
{
}
