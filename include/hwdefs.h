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

#define PWM_TIMER TIM1
#define PWM_TIMRST RST_TIM1

#define FUELGAUGE_TIMER TIM4

#define TERM_USART USART3
#define TERM_USART_TXPORT GPIOB // gpio B for usart 3 on 32F107 Zombie
#define TERM_USART_DR USART3_DR
#define TERM_BUFSIZE 128

/* Parameter block size. F1 stores this as a flash page. F4 stores the
 * same image at PARAM_ADDRESS and erases the whole sector around it.
 */
#define FLASH_PAGE_SIZE 2048
#define PARAM_BLKNUM 1
#define PARAM_BLKSIZE FLASH_PAGE_SIZE
#define CAN1_BLKNUM 2
#define CAN2_BLKNUM 4

#if defined(STM32F4)
/* 168 MHz from the F407 3.3 V profile in libopencm3.
 * HCLK 168 MHz, APB1 42 MHz, APB2 84 MHz.
 * APB prescalers are not 1, so the timer clocks are twice the bus clocks.
 * CAN is on APB1. 500 kbit/s is exact at 42 MHz, which is what this VCU
 * asks for. libopeninv's F4 defaults (CAN_PERIPH_SPEED 42 and
 * DELAY_CYCLES_PER_US 28) match this clock, so they are not overridden.
 *
 * The crystal is not confirmed on the new board. The F107 firmware
 * assumes 8 MHz (RCC_CLOCK_HSE8_72MHZ). Pass VCU_HSE_HZ=25000000 if the
 * F407 board is measured with a 25 MHz crystal. Both profiles produce
 * the clocks above. 180 MHz profiles are not used.
 */
#define PERIPH_CLK ((uint32_t)42000000)
#define VCU_HCLK_HZ 168000000u
#define VCU_PCLK1_HZ 42000000u
#define VCU_PCLK2_HZ 84000000u
#define VCU_TIMCLK_APB1_HZ (VCU_PCLK1_HZ * 2u)
#define VCU_TIMCLK_APB2_HZ (VCU_PCLK2_HZ * 2u)

/* TIM1 update shares its vector with TIM10 on the F407. */
#define PWM_TIMER_IRQ NVIC_TIM1_UP_TIM10_IRQ
#define pwm_timer_isr tim1_up_tim10_isr

/* USART3 terminal DMA is chosen inside libopeninv. Kept here so an old
 * reader does not pick up the F1 channel numbers: RX is DMA1 stream 1,
 * TX is DMA1 stream 3, request channel 4.
 */
#define TERM_USART_DMARX DMA_STREAM1
#define TERM_USART_DMATX DMA_STREAM3

/* One store per sector. Saving erases that whole sector.
 * Sector 0 (16 KiB at 0x08000000) is left for a bootloader this tree
 * does not contain. The application image itself is sectors 4 to 7.
 * That range exists on both the 512 KiB VET6 and the 1 MiB VGT6.
 * Sectors 8 to 11 exist only on the 1 MiB part and are not used.
 * FLASH_APP_END is one past the last byte the linker may fill.
 */
#define PARAM_ADDRESS 0x08004000u
#define PARAM_SECTOR 1u
#define CANMAP_ADDRESS 0x08008000u
#define CANMAP_SECTOR 2u
#define PINDEF_ADDRESS 0x0800C000u
#define PINDEF_SECTOR 3u
#define FLASH_APP_START 0x08010000u
#define FLASH_APP_END 0x08080000u

#if (PINDEF_SECTOR == PARAM_SECTOR) || (PINDEF_SECTOR == CANMAP_SECTOR) ||    \
    (PARAM_SECTOR == CANMAP_SECTOR)
#error Each F407 flash store needs its own sector.
#endif
#else
#define PERIPH_CLK ((uint32_t)36000000)

#define PWM_TIMER_IRQ NVIC_TIM1_UP_IRQ
#define pwm_timer_isr tim1_up_isr

#define TERM_USART_TXPIN GPIO_USART3_PR_TX
#define TERM_USART_DMARX DMA_CHANNEL3
#define TERM_USART_DMATX                                                       \
  DMA_CHANNEL2 // this means we can not use it on rev1 hardware (TIM3_CH3)
#endif

#endif // HWDEFS_H_INCLUDED
