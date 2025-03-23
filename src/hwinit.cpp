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

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/rtc.h>
#include "hwdefs.h"
#include "hwinit.h"
#include "params.h"
#include "iomatrix.h"
#include "digio.h"

/**
* Start clocks of all needed peripherals
*/
void clock_setup(void)
{
    RCC_CLOCK_SETUP();

    rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6);

    //The reset value for PRIGROUP (=0) is not actually a defined
    //value. Explicitly set 16 preemtion priorities
    SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_PRIGROUP_GROUP16_NOSUB;

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_GPIOE);
    rcc_periph_clock_enable(RCC_USART3);
    rcc_periph_clock_enable(RCC_USART2);//GS450H Inverter Comms
    rcc_periph_clock_enable(RCC_USART1);//LIN Comms
    rcc_periph_clock_enable(RCC_TIM1); //GS450H oil pump pwm
    rcc_periph_clock_enable(RCC_TIM2); //GS450H 500khz usart clock
    rcc_periph_clock_enable(RCC_TIM3); //PWM outputs
    rcc_periph_clock_enable(RCC_TIM4); //Scheduler
    rcc_periph_clock_enable(RCC_DMA1);  //ADC, and UARTS
    // rcc_periph_clock_enable(RCC_DMA2);
    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_CRC);
    rcc_periph_clock_enable(RCC_AFIO); //CAN AND USART3
    rcc_periph_clock_enable(RCC_CAN1); //CAN1
    rcc_periph_clock_enable(RCC_CAN2); //CAN2
    rcc_periph_clock_enable(RCC_SPI2);  //CAN3
    rcc_periph_clock_enable(RCC_SPI3);  //Digital POTS
}


void spi2_setup()   //spi 2 used for CAN3
{

    spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_32, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    spi_set_standard_mode(SPI2,0);//set mode 0

    spi_enable_software_slave_management(SPI2);
    //spi_enable_ss_output(SPI2);
    spi_set_nss_high(SPI2);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO15 | GPIO13);//MOSI , CLK
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO14);//MISO
    spi_enable(SPI2);
}

void spi3_setup()   //spi3 used for digi pots (fuel gauge etc)
{
    gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_SPI3_REMAP);

    spi_init_master(SPI3, SPI_CR1_BAUDRATE_FPCLK_DIV_32, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

    spi_enable_software_slave_management(SPI3);
    spi_set_nss_high(SPI3);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO12 | GPIO10);//MOSI , CLK
    gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO11);//MISO
    spi_enable(SPI3);
}

/**
* Setup USART1 for LINbus
*/

void usart1_setup(void)
{
    /* Setup GPIO pin GPIO_USART1_TX and GPIO_USART1_RX. */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
    usart_set_baudrate(USART1, 19200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable(USART1);
}

/**
* Setup USART2 500000 8N1 for Toyota inverter comms
*/
void usart2_setup(void)
{
    /* Setup GPIO pin GPIO_USART2_TX and GPIO_USART2_RX. */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);
    usart_set_baudrate(USART2, 500000);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_enable(USART2);
}

/**
* Enable Timer refresh and break interrupts
*/
void nvic_setup(void)
{
    nvic_enable_irq(NVIC_DMA1_CHANNEL7_IRQ);
    nvic_set_priority(NVIC_DMA1_CHANNEL7_IRQ, 0xf0);//usart2_TX

    nvic_enable_irq(NVIC_DMA1_CHANNEL6_IRQ);
    nvic_set_priority(NVIC_DMA1_CHANNEL6_IRQ, 0xf0);//usart2_RX low priority int

    // nvic_enable_irq(NVIC_DMA1_CHANNEL3_IRQ);
    //nvic_set_priority(NVIC_DMA1_CHANNEL3_IRQ, 0x20);//usart3_RX high priority int

    nvic_enable_irq(NVIC_TIM4_IRQ); //Scheduler on tim4
    nvic_set_priority(NVIC_TIM4_IRQ, 0); //Highest priority

    nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ); //CAN RX
    nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 0xe << 4); //second lowest priority

    nvic_enable_irq(NVIC_USB_HP_CAN_TX_IRQ); //CAN TX
    nvic_set_priority(NVIC_USB_HP_CAN_TX_IRQ, 0xe << 4); //second lowest priority

    /* Enable MCP2526 IRQ on PE15 */
    nvic_enable_irq(NVIC_EXTI15_10_IRQ);
    exti_enable_request(EXTI15);
    exti_set_trigger(EXTI15, EXTI_TRIGGER_FALLING);
    exti_select_source(EXTI15,GPIOE);
    /* Without this the RTC interrupt routine will never be called. */
    nvic_enable_irq(NVIC_RTC_IRQ);
    nvic_set_priority(NVIC_RTC_IRQ, 0x20);

}

void rtc_setup()
{
    //Base clock is HSE/128 = 8MHz/128 = 62.5kHz
    //62.5kHz / (62499 + 1) = 1Hz
    rtc_auto_awake(RCC_HSE, 62499); //1s tick
    rtc_set_counter_val(0);
    //* Enable the RTC interrupt to occur off the SEC flag.
    rtc_clear_flag(RTC_SEC);
    rtc_interrupt_enable(RTC_SEC);
}

void tim_setup()
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Code for timer 1
    //oil pump control pwm for Toyota hybrid gearbox Needs to be 1khz
    //Variable frequency output
    ////////////////////////////////////////////////////////////////////////
    gpio_set_mode(GPIOE,GPIO_MODE_OUTPUT_2_MHZ,	// Low speed (only need 1khz)
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO9);	// GPIOE9=TIM1.CH1 Alt

    timer_disable_counter(TIM1);
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1,
                   TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM1,32); //16
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM2);
    timer_enable_oc_output(TIM1, TIM_OC1);
    timer_enable_break_main_output(TIM1);
    timer_set_oc_value(TIM1, TIM_OC1, 400);//duty. 1000 = 52% , 500 = 76% , 1500=28%
    timer_set_period(TIM1, 1088);//frequency
    timer_enable_counter(TIM1);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

}

void tim2_setup()
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Code for timer 2 to create usart 2 clock for Toyota hybrid comms
    ////////////////////////////////////////////////////////////////////////
    gpio_set_mode(GPIOA,GPIO_MODE_OUTPUT_50_MHZ,	// High speed
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO1);	// GPIOA1=TIM2.CH2 clock for usart2

    // TIM2:
    timer_disable_counter(TIM2);
//	timer_reset(TIM2);

    timer_set_mode(TIM2,
                   TIM_CR1_CKD_CK_INT,
                   TIM_CR1_CMS_EDGE,
                   TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM2,0);
    timer_enable_preload(TIM2);
    timer_continuous_mode(TIM2);
    timer_set_period(TIM2,143);//500khz

    timer_disable_oc_output(TIM2,TIM_OC2);
    timer_set_oc_mode(TIM2,TIM_OC2,TIM_OCM_PWM1);
    timer_enable_oc_output(TIM2,TIM_OC2);

    timer_set_oc_value(TIM2,TIM_OC2,72);//50% duty
    timer_enable_counter(TIM2);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
}


void tim3_setup()
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Setup all 3 PWM ports to output a 1khz 50% duty cycle PWM signal
    // General purpose pwm output. Push/pull driven to +12v/gnd. Timer 3 Chan 3 PB0.
    // General purpose pwm output. Push/pull driven to +12v/gnd. Timer 3 Chan 2 PA7.
    // General purpose pwm output. Push/pull driven to +12v/gnd. Timer 3 Chan 1 PA6.
    ////////////////////////////////////////////////////////////////////////
    bool Fixed1K = 0;
    ///PWM3
    if (Param::GetInt(Param::PWM3Func) == IOMatrix::PWM_TIM3)
    {
        gpio_set_mode(GPIOB,GPIO_MODE_OUTPUT_2_MHZ,	// Low speed (only need 1khz)
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO0);	// GPIOB0=TIM3.CH3
    }
    else if (Param::GetInt(Param::PWM3Func) == IOMatrix::CP_SPOOF || Param::GetInt(Param::PWM3Func) == IOMatrix::GS450HOIL || Param::GetInt(Param::PWM3Func) == IOMatrix::PWMTEMPGAUGE || Param::GetInt(Param::PWM3Func) == IOMatrix::PWMSOCGAUGE)
    {
        gpio_set_mode(GPIOB,GPIO_MODE_OUTPUT_2_MHZ,	// Low speed (only need 1khz)
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO0);	// GPIOB0=TIM3.CH3
        Fixed1K = 1; //Running 1kHz Fixed
    }
    else
    {
        DigIo::PWM3.Configure(GPIOB,GPIO0,PinMode::OUTPUT);
    }

    ///PWM2
    if (Param::GetInt(Param::PWM2Func) == IOMatrix::PWM_TIM3)
    {
        gpio_set_mode(GPIOA,GPIO_MODE_OUTPUT_2_MHZ,	// Low speed (only need 1khz)
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO7);	// GPIOE9=TIM3.CH2
    }
    else if (Param::GetInt(Param::PWM2Func) == IOMatrix::CP_SPOOF || Param::GetInt(Param::PWM2Func) == IOMatrix::GS450HOIL || Param::GetInt(Param::PWM2Func) == IOMatrix::PWMTEMPGAUGE || Param::GetInt(Param::PWM2Func) == IOMatrix::PWMSOCGAUGE)
    {
        gpio_set_mode(GPIOA,GPIO_MODE_OUTPUT_2_MHZ,	// Low speed (only need 1khz)
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO7);	// GPIOE9=TIM3.CH2
        Fixed1K = 1; //Running 1kHz Fixed
    }
    else
    {
        DigIo::PWM2.Configure(GPIOA,GPIO7,PinMode::OUTPUT);
    }

    ///PWM1
    if (Param::GetInt(Param::PWM1Func) == IOMatrix::PWM_TIM3)
    {
        gpio_set_mode(GPIOA,GPIO_MODE_OUTPUT_2_MHZ,	// Low speed (only need 1khz)
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO6);	// GPIOA6=TIM3.CH1
    }
    else if (Param::GetInt(Param::PWM1Func) == IOMatrix::CP_SPOOF || Param::GetInt(Param::PWM1Func) == IOMatrix::GS450HOIL || Param::GetInt(Param::PWM1Func) == IOMatrix::PWMTEMPGAUGE || Param::GetInt(Param::PWM1Func) == IOMatrix::PWMSOCGAUGE)
    {
        gpio_set_mode(GPIOA,GPIO_MODE_OUTPUT_2_MHZ,	// Low speed (only need 1khz)
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO6);	// GPIOA6=TIM3.CH1
        Fixed1K = 1; //Running 1kHz Fixed
    }
    else
    {
        DigIo::PWM1.Configure(GPIOA,GPIO6,PinMode::OUTPUT);
    }

    timer_disable_counter(TIM3);
    //edge aligned PWM
    timer_set_alignment(TIM3, TIM_CR1_CMS_EDGE);
    timer_enable_preload(TIM3);
    /* PWM mode 1 and preload enable */
    timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_PWM1);
    timer_enable_oc_preload(TIM3, TIM_OC1);
    timer_enable_oc_preload(TIM3, TIM_OC2);
    timer_enable_oc_preload(TIM3, TIM_OC3);

    timer_set_oc_polarity_high(TIM3, TIM_OC1);
    timer_set_oc_polarity_high(TIM3, TIM_OC2);
    timer_set_oc_polarity_high(TIM3, TIM_OC3);
    timer_enable_oc_output(TIM3, TIM_OC1);
    timer_enable_oc_output(TIM3, TIM_OC2);
    timer_enable_oc_output(TIM3, TIM_OC3);

    if (Fixed1K == 0) //No CP Spoof Selected or GS450h Oil pump or Gauges
    {
        timer_set_period(TIM3, Param::GetInt(Param::Tim3_Period));
        timer_set_oc_value(TIM3, TIM_OC1, Param::GetInt(Param::Tim3_1_OC));
        timer_set_oc_value(TIM3, TIM_OC2, Param::GetInt(Param::Tim3_2_OC));
        timer_set_oc_value(TIM3, TIM_OC3, Param::GetInt(Param::Tim3_3_OC));
        timer_generate_event(TIM3, TIM_EGR_UG);
        timer_set_prescaler(TIM3,Param::GetInt(Param::Tim3_Presc));
        timer_enable_counter(TIM3);
    }
    else//No CP Spoof or GS450h Oil pump output selected locks TIM3 Force 1khz clock (996khz)
    {
        timer_set_period(TIM3, 3999);
        timer_set_oc_value(TIM3, TIM_OC1, 1);//No duty set here
        timer_set_oc_value(TIM3, TIM_OC2, 1);//No duty set here
        timer_set_oc_value(TIM3, TIM_OC3, 1);//No duty set here
        timer_generate_event(TIM3, TIM_EGR_UG);
        timer_set_prescaler(TIM3,17);
        timer_enable_counter(TIM3);
    }
}
