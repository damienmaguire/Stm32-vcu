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
    rcc_periph_clock_enable(RCC_TIM1); //GS450H oil pump pwm
    rcc_periph_clock_enable(RCC_TIM2); //GS450H 500khz usart clock
    rcc_periph_clock_enable(RCC_TIM3); //Scheduler
    rcc_periph_clock_enable(RCC_TIM4); //
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

static bool is_floating(uint32_t port, uint16_t pin)
{
    //A pin is considered floating if its state can be controlled with the internal 30k pull-up/down resistor
    bool isFloating = false;
    gpio_set_mode(port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, pin);
    gpio_set(port, pin); //pull up with internal ~30k
    for (volatile int i = 0; i < 80000; i++);
    isFloating = gpio_get(port, pin);
    gpio_clear(port, pin);
    for (volatile int i = 0; i < 80000; i++);
    isFloating &= gpio_get(port, pin) == 0;
    return isFloating;
}

HWREV detect_hw()
{
    if (is_floating(GPIOC, GPIO9)) //Desat pin is floating
        return HW_REV1;
    else if (is_floating(GPIOA, GPIO0)) //uvlo/button pin is floating
        return HW_REV3;
    else if (is_floating(GPIOC, GPIO8)) //bms input is output for mux and floating
        return HW_TESLA;
    else
        return HW_REV2;
}

/**
* Setup UART3 115200 8N1
*/
void usart_setup(void)
{
    gpio_set_mode(TERM_USART_TXPORT, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, TERM_USART_TXPIN);
    usart_set_baudrate(TERM_USART, USART_BAUDRATE);
    usart_set_databits(TERM_USART, 8);
    usart_set_stopbits(TERM_USART, USART_STOPBITS_2);
    usart_set_mode(TERM_USART, USART_MODE_TX_RX);
    usart_set_parity(TERM_USART, USART_PARITY_NONE);
    usart_set_flow_control(TERM_USART, USART_FLOWCONTROL_NONE);
    usart_enable_rx_dma(TERM_USART);

    usart_enable_tx_dma(TERM_USART);
    dma_channel_reset(DMA1, TERM_USART_DMATX);
    dma_set_read_from_memory(DMA1, TERM_USART_DMATX);
    dma_set_peripheral_address(DMA1, TERM_USART_DMATX, (uint32_t)&TERM_USART_DR);
    dma_set_peripheral_size(DMA1, TERM_USART_DMATX, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, TERM_USART_DMATX, DMA_CCR_MSIZE_8BIT);
    dma_enable_memory_increment_mode(DMA1, TERM_USART_DMATX);
    dma_set_priority(DMA1, TERM_USART_DMATX, DMA_CCR_PL_VERY_HIGH);

    dma_channel_reset(DMA1, TERM_USART_DMARX);
    dma_set_peripheral_address(DMA1, TERM_USART_DMARX, (uint32_t)&TERM_USART_DR);
    dma_set_peripheral_size(DMA1, TERM_USART_DMARX, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, TERM_USART_DMARX, DMA_CCR_MSIZE_8BIT);
    dma_enable_memory_increment_mode(DMA1, TERM_USART_DMARX);
    dma_set_priority(DMA1, TERM_USART_DMARX, DMA_CCR_PL_VERY_HIGH);
    // dma_enable_transfer_complete_interrupt(DMA1, TERM_USART_DMARX);
    dma_enable_channel(DMA1, TERM_USART_DMARX);

    usart_enable(TERM_USART);
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


    nvic_enable_irq(NVIC_TIM3_IRQ); //Scheduler on tim3
    nvic_set_priority(NVIC_TIM3_IRQ, 0); //Highest priority


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
    //62.5kHz / (624 + 1) = 100Hz
    rtc_auto_awake(RCC_HSE, 624); //10ms tick
    rtc_set_counter_val(0);
    //* Enable the RTC interrupt to occur off the SEC flag.
    rtc_clear_flag(RTC_SEC);
	rtc_interrupt_enable(RTC_SEC);
}

void tim_setup()
{
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Code for timer 1 to create oil pump control pwm for Toyota hybrid gearbox
    //Needs to be 1khz
    ////////////////////////////////////////////////////////////////////////
    gpio_set_mode(GPIOA,GPIO_MODE_OUTPUT_50_MHZ,	// High speed
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO8);	// GPIOA8=TIM1.CH1

    timer_disable_counter(TIM1);
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1,
                   TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM1,16);
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM2);
    timer_enable_oc_output(TIM1, TIM_OC1);
    timer_enable_break_main_output(TIM1);
    timer_set_oc_value(TIM1, TIM_OC1, 1000);//duty. 1000 = 52% , 500 = 76% , 1500=28%
    timer_set_period(TIM1, 2100);
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

