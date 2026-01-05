/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2019-2022 Damien Maguire <info@evbmw.com>
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

#ifndef HWDEFS_H_INCLUDED
#define HWDEFS_H_INCLUDED

// Common for any config

// Maximum value for over current limit timer
#define GAUGEMAX 4096
#define USART_BAUDRATE 115200
// Maximum PWM frequency is 36MHz/2^MIN_PWM_DIGITS
#define MIN_PWM_DIGITS 11
#define PERIPH_CLK ((uint32_t)36000000)

#define RCC_CLOCK_SETUP rcc_clock_setup_in_hse_8mhz_out_72mhz

#define PWM_TIMER TIM1
#define PWM_TIMRST RST_TIM1
#define PWM_TIMER_IRQ NVIC_TIM1_UP_IRQ
#define pwm_timer_isr tim1_up_isr

#define FUELGAUGE_TIMER TIM4

#define TERM_USART USART3
#define TERM_USART_TXPIN GPIO_USART3_PR_TX
// #define TERM_USART_TXPORT  GPIOC
#define TERM_USART_TXPORT GPIOB // gpio B for usart 3 on 32F107 Zombie
#define TERM_USART_DMARX DMA_CHANNEL3
#define TERM_USART_DMATX                                                       \
  DMA_CHANNEL2 // this means we can not use it on rev1 hardware (TIM3_CH3)
#define TERM_USART_DR USART3_DR
#define TERM_BUFSIZE 128

// Address of parameter block in flash for 105
#define FLASH_PAGE_SIZE 2048
#define PARAM_BLKNUM 1
#define PARAM_BLKSIZE FLASH_PAGE_SIZE
#define CAN1_BLKNUM 2
#define CAN2_BLKNUM 4

#endif // HWDEFS_H_INCLUDED
