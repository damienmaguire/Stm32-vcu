/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
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

#ifndef HWINIT_H_INCLUDED
#define HWINIT_H_INCLUDED


#ifdef __cplusplus
extern "C"
{
#endif

void clock_setup(void);
void usart_setup(void);
void usart2_setup(void);
void nvic_setup(void);
void rtc_setup(void);
void tim_setup(void);
void tim2_setup(void);
void tim3_setup(void);
void spi2_setup(void);
void spi3_setup(void);

#ifdef __cplusplus
}
#endif

#endif // HWINIT_H_INCLUDED
