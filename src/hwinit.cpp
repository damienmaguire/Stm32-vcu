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

#include "hwinit.h"
#include "digio.h"
#include "errormessage.h"
#include "hwdefs.h"
#include "iomatrix.h"
#include "params.h"
#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#if defined(STM32F4)
#include <libopencm3/cm3/vector.h>
#endif

#if defined(STM32F4)
/* The F407 reset vector is at the start of flash. This image is linked
 * later, so interrupts have to use its own table. libopencm3 turns the
 * FPU on in pre_main before this runs. Called again from clock_setup
 * in case a debugger starts at main.
 */
static void vcu_relocate_vector_table(void) {
  SCB_VTOR = (uint32_t)&vector_table;
}

using VcuInitFn = void (*)(void);
__attribute__((section(".preinit_array"), used)) static const VcuInitFn
    vcu_preinit_vtor = vcu_relocate_vector_table;

/* Alternate function, push-pull. speed is a GPIO_OSPEED_* value.
 * F4 inputs of a peripheral are alternate-function pins as well.
 */
static void af_out(uint32_t port, uint16_t pins, uint8_t af, uint8_t speed) {
  gpio_set_af(port, af, pins);
  gpio_set_output_options(port, GPIO_OTYPE_PP, speed, pins);
  gpio_mode_setup(port, GPIO_MODE_AF, GPIO_PUPD_NONE, pins);
}

static void af_in(uint32_t port, uint16_t pins, uint8_t af) {
  gpio_set_af(port, af, pins);
  gpio_mode_setup(port, GPIO_MODE_AF, GPIO_PUPD_NONE, pins);
}

#ifndef VCU_HSE_HZ
#define VCU_HSE_HZ 8000000
#endif

/* Counter clock stays equal to the F107 so the oil-pump duty numbers
 * and the speedo formula in utils.cpp do not have to change.
 * F107 TIM1 clock is 72 MHz and the prescaler is 32, so 72 MHz / 33.
 */
static_assert(static_cast<unsigned long long>(VCU_TIMCLK_APB2_HZ) * 33ull ==
                  72000000ull * 77ull,
              "F407 TIM1 prescaler no longer matches the F107 counter clock");
static_assert(VCU_TIMCLK_APB1_HZ / 168u == 500000u, "TIM2 is not 500 kHz");
static_assert(VCU_TIMCLK_APB1_HZ / (21u * 4000u) == 1000u, "TIM3 is not 1 kHz");
static_assert(VCU_TIMCLK_APB1_HZ % 10000u == 0u, "TIM5 cannot tick at 1 Hz");
#endif

/**
 * Start clocks of all needed peripherals
 */
void clock_setup(void) {
#if defined(STM32F4)
  vcu_relocate_vector_table();
  /* libopencm3 sets the 3.3 V voltage scale and 5 flash wait states.
   * Both crystals below land on HCLK 168, APB1 42, APB2 84.
   */
#if VCU_HSE_HZ == 8000000
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
#elif VCU_HSE_HZ == 25000000
  rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
#else
#error "F407 clock needs VCU_HSE_HZ of 8000000 or 25000000"
#endif
#else
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

  rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6);
#endif

  // The reset value for PRIGROUP (=0) is not actually a defined
  // value. Explicitly set 16 preemtion priorities
  SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_PRIGROUP_GROUP16_NOSUB;

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_GPIOD);
  rcc_periph_clock_enable(RCC_GPIOE);
  /* LQFP100 pins 49 and 73 are VCAP_1 and VCAP_2 on the F407. They have
   * no GPIO port or bit, so there is nothing to configure. Do not drive
   * them. PH0 and PH1 stay with the crystal.
   */
  rcc_periph_clock_enable(RCC_USART3);
  rcc_periph_clock_enable(RCC_USART2); // GS450H Inverter Comms
  rcc_periph_clock_enable(RCC_USART1); // LIN Comms
  rcc_periph_clock_enable(RCC_TIM1);   // GS450H oil pump pwm
  rcc_periph_clock_enable(RCC_TIM2);   // GS450H 500khz usart clock
  rcc_periph_clock_enable(RCC_TIM3);   // PWM outputs
  rcc_periph_clock_enable(RCC_TIM4);   // Scheduler
  rcc_periph_clock_enable(RCC_DMA1);   // ADC, and UARTS
#if defined(STM32F4)
  rcc_periph_clock_enable(RCC_DMA2); // ADC1 is DMA2 stream 0 on the F407
  rcc_periph_clock_enable(RCC_TIM5); // 1 Hz seconds tick
#else
  // rcc_periph_clock_enable(RCC_DMA2);
#endif
  rcc_periph_clock_enable(RCC_ADC1);
#if defined(STM32F4)
  /* PCLK2 is 84 MHz. Divide by 4 for a 21 MHz ADC clock.
   * Divide by 2 would be 42 MHz, above the F407 ADC limit.
   */
  adc_set_clk_prescale(ADC_CCR_ADCPRE_BY4);
#endif
  rcc_periph_clock_enable(RCC_CRC);
#if defined(STM32F4)
  rcc_periph_clock_enable(RCC_SYSCFG); // EXTI routing. The F407 has no AFIO.
#else
  rcc_periph_clock_enable(RCC_AFIO); // CAN AND USART3
#endif
  /* CAN2 filters live in the CAN1 block, so CAN2 needs both clocks. */
  rcc_periph_clock_enable(RCC_CAN1); // CAN1
  rcc_periph_clock_enable(RCC_CAN2); // CAN2
  rcc_periph_clock_enable(RCC_SPI2); // CAN3
  rcc_periph_clock_enable(RCC_SPI3); // Digital POTS
}

void spi2_setup() // spi 2 used for CAN3
{

  spi_init_master(
      SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_32, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
      SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
  spi_set_standard_mode(SPI2, 0); // set mode 0

  spi_enable_software_slave_management(SPI2);
  // spi_enable_ss_output(SPI2);
  spi_set_nss_high(SPI2);
#if defined(STM32F4)
  /* SPI2 on PB13/PB14/PB15 is AF5. Chip select stays a GPIO. */
  af_out(GPIOB, GPIO15 | GPIO13, GPIO_AF5, GPIO_OSPEED_50MHZ); // MOSI, CLK
  af_in(GPIOB, GPIO14, GPIO_AF5);                              // MISO
#else
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO15 | GPIO13); // MOSI , CLK
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO14); // MISO
#endif
  spi_enable(SPI2);
}

void spi3_setup() // spi3 used for digi pots (fuel gauge etc)
{
#if defined(STM32F4)
  /* PC10/PC11/PC12 are SPI3 AF6. No AFIO remap. PA15, PB3 and PB4 leave
   * JTAG when DigIo sets them as GPIO inputs.
   */
#else
  gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_SPI3_REMAP);
#endif

  spi_init_master(
      SPI3, SPI_CR1_BAUDRATE_FPCLK_DIV_32, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
      SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

  spi_enable_software_slave_management(SPI3);
  spi_set_nss_high(SPI3);
#if defined(STM32F4)
  af_out(GPIOC, GPIO12 | GPIO10, GPIO_AF6, GPIO_OSPEED_50MHZ); // MOSI, CLK
  af_in(GPIOC, GPIO11, GPIO_AF6);                              // MISO
#else
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO12 | GPIO10); // MOSI , CLK
  gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO11); // MISO
#endif
  spi_enable(SPI3);
}

/**
 * Setup USART2 500000 8N1 for Toyota inverter comms
 */
void usart2_setup(void) {
  /* Setup GPIO pin GPIO_USART2_TX and GPIO_USART2_RX. */
#if defined(STM32F4)
  /* PA2 TX and PA3 RX are USART2 AF7. */
  af_out(GPIOA, GPIO2, GPIO_AF7, GPIO_OSPEED_50MHZ);
  af_in(GPIOA, GPIO3, GPIO_AF7);
#else
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_USART2_TX);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);
#endif
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
void nvic_setup(void) {
#if defined(STM32F4)
  /* USART2 TX is DMA1 stream 6, RX is stream 5. Not the F1 channel numbers. */
  nvic_enable_irq(NVIC_DMA1_STREAM6_IRQ);
  nvic_set_priority(NVIC_DMA1_STREAM6_IRQ, 0xf0); // usart2_TX

  nvic_enable_irq(NVIC_DMA1_STREAM5_IRQ);
  nvic_set_priority(NVIC_DMA1_STREAM5_IRQ, 0xf0); // usart2_RX low priority int
#else
  nvic_enable_irq(NVIC_DMA1_CHANNEL7_IRQ);
  nvic_set_priority(NVIC_DMA1_CHANNEL7_IRQ, 0xf0); // usart2_TX

  nvic_enable_irq(NVIC_DMA1_CHANNEL6_IRQ);
  nvic_set_priority(NVIC_DMA1_CHANNEL6_IRQ, 0xf0); // usart2_RX low priority int
#endif

  // nvic_enable_irq(NVIC_DMA1_CHANNEL3_IRQ);
  // nvic_set_priority(NVIC_DMA1_CHANNEL3_IRQ, 0x20);//usart3_RX high priority
  // int

  nvic_enable_irq(NVIC_TIM4_IRQ);      // Scheduler on tim4
  nvic_set_priority(NVIC_TIM4_IRQ, 0); // Highest priority

#if defined(STM32F4)
  nvic_enable_irq(NVIC_CAN1_RX0_IRQ);             // CAN RX
  nvic_set_priority(NVIC_CAN1_RX0_IRQ, 0xe << 4); // second lowest priority

  nvic_enable_irq(NVIC_CAN1_TX_IRQ);             // CAN TX
  nvic_set_priority(NVIC_CAN1_TX_IRQ, 0xe << 4); // second lowest priority
#else
  nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);             // CAN RX
  nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 0xe << 4); // second lowest
                                                        // priority

  nvic_enable_irq(NVIC_USB_HP_CAN_TX_IRQ);             // CAN TX
  nvic_set_priority(NVIC_USB_HP_CAN_TX_IRQ, 0xe << 4); // second lowest priority
#endif

  /* Enable MCP2526 IRQ on PE15 */
#if defined(STM32F4)
  gpio_mode_setup(GPIOE, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO15);
#endif
  nvic_enable_irq(NVIC_EXTI15_10_IRQ);
  exti_enable_request(EXTI15);
  exti_set_trigger(EXTI15, EXTI_TRIGGER_FALLING);
  exti_select_source(EXTI15, GPIOE);
#if defined(STM32F4)
  /* 1 Hz tick. Same priority the F107 RTC second interrupt used. */
  nvic_enable_irq(NVIC_TIM5_IRQ);
  nvic_set_priority(NVIC_TIM5_IRQ, 0x20);
#else
  /* Without this the RTC interrupt routine will never be called. */
  nvic_enable_irq(NVIC_RTC_IRQ);
  nvic_set_priority(NVIC_RTC_IRQ, 0x20);
#endif
}

void rtc_setup() {
#if defined(STM32F4)
  /* The F407 RTC is a calendar, not the F1 seconds counter. TIM5 is a
   * 32-bit timer that this board does not use for a pin. Its update
   * event runs at 1 Hz and tim5_isr counts the seconds. The count
   * starts at zero on every boot, as the F107 counter was cleared.
   * APB1 timers clock at 84 MHz here: 84 MHz / 8400 / 10000 = 1 Hz.
   */
  timer_disable_counter(TIM5);
  timer_set_mode(TIM5, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_prescaler(TIM5, (VCU_TIMCLK_APB1_HZ / 10000u) - 1u);
  timer_set_period(TIM5, 10000u - 1u);
  timer_clear_flag(TIM5, TIM_SR_UIF);
  timer_enable_irq(TIM5, TIM_DIER_UIE);
  timer_set_counter(TIM5, 0);
  timer_enable_counter(TIM5);
#else
  // Base clock is HSE/128 = 8MHz/128 = 62.5kHz
  // 62.5kHz / (62499 + 1) = 1Hz
  rtc_auto_awake(RCC_HSE, 62499); // 1s tick
  rtc_set_counter_val(0);
  //* Enable the RTC interrupt to occur off the SEC flag.
  rtc_clear_flag(RTC_SEC);
  rtc_interrupt_enable(RTC_SEC);
#endif
}

void tim_setup() {
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Code for timer 1
  // oil pump control pwm for Toyota hybrid gearbox Needs to be 1khz
  // Variable frequency output
  ////////////////////////////////////////////////////////////////////////
#if defined(STM32F4)
  /* PE9 is TIM1 CH1, AF1. The F107 full remap is what put TIM1 there. */
  af_out(GPIOE, GPIO9, GPIO_AF1, GPIO_OSPEED_2MHZ);
#else
  gpio_set_mode(GPIOE, GPIO_MODE_OUTPUT_2_MHZ, // Low speed (only need 1khz)
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9); // GPIOE9=TIM1.CH1 Alt
#endif

  timer_disable_counter(TIM1);
  timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1,
                 TIM_CR1_DIR_UP);
#if defined(STM32F4)
  /* TIM1 clocks at 168 MHz. Prescaler 76 gives the same counter clock
   * the F107 gets from 72 MHz / 33, so period 1088 is still about 1 kHz
   * centre-aligned and the oil-pump compare values keep their duty.
   */
  timer_set_prescaler(TIM1, 76);
#else
  timer_set_prescaler(TIM1, 32); // 16
#endif
  timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM2);
  timer_enable_oc_output(TIM1, TIM_OC1);
  timer_enable_break_main_output(TIM1);
  timer_set_oc_value(TIM1, TIM_OC1,
                     400);      // duty. 1000 = 52% , 500 = 76% , 1500=28%
  timer_set_period(TIM1, 1088); // frequency
  timer_enable_counter(TIM1);
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void tim2_setup() {
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Code for timer 2 to create usart 2 clock for Toyota hybrid comms
  ////////////////////////////////////////////////////////////////////////
#if defined(STM32F4)
  /* PA1 is TIM2 CH2, AF1. This is the 500 kHz clock to the inverter. */
  af_out(GPIOA, GPIO1, GPIO_AF1, GPIO_OSPEED_50MHZ);
#else
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, // High speed
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO1); // GPIOA1=TIM2.CH2 clock for usart2
#endif

  // TIM2:
  timer_disable_counter(TIM2);
  //	timer_reset(TIM2);

  timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_prescaler(TIM2, 0);
  timer_enable_preload(TIM2);
  timer_continuous_mode(TIM2);
#if defined(STM32F4)
  /* TIM2 clocks at 84 MHz. 84 MHz / 168 = 500 kHz, half of that is 50%. */
  timer_set_period(TIM2, 167);
#else
  timer_set_period(TIM2, 143); // 500khz
#endif

  timer_disable_oc_output(TIM2, TIM_OC2);
  timer_set_oc_mode(TIM2, TIM_OC2, TIM_OCM_PWM1);
  timer_enable_oc_output(TIM2, TIM_OC2);

#if defined(STM32F4)
  timer_set_oc_value(TIM2, TIM_OC2, 84); // 50% duty
#else
  timer_set_oc_value(TIM2, TIM_OC2, 72); // 50% duty
#endif
  timer_enable_counter(TIM2);
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void tim3_setup() {
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Setup all 3 PWM ports to output a 1khz 50% duty cycle PWM signal
  // General purpose pwm output. Push/pull driven to +12v/gnd. Timer 3 Chan 3
  // PB0. General purpose pwm output. Push/pull driven to +12v/gnd. Timer 3 Chan
  // 2 PA7. General purpose pwm output. Push/pull driven to +12v/gnd. Timer 3
  // Chan 1 PA6.
  ////////////////////////////////////////////////////////////////////////
  bool Fixed1K = false;
  /// PWM3
  if (Param::GetInt(Param::PWM3Func) == IOMatrix::PWM_TIM3) {
#if defined(STM32F4)
    af_out(GPIOB, GPIO0, GPIO_AF2, GPIO_OSPEED_2MHZ); // PB0 = TIM3 CH3
#else
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, // Low speed (only need 1khz)
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO0); // GPIOB0=TIM3.CH3
#endif
  } else if (Param::GetInt(Param::PWM3Func) == IOMatrix::CP_SPOOF ||
             Param::GetInt(Param::PWM3Func) == IOMatrix::GS450HOIL ||
             Param::GetInt(Param::PWM3Func) == IOMatrix::PWMTEMPGAUGE ||
             Param::GetInt(Param::PWM3Func) == IOMatrix::PWMSOCGAUGE) {
#if defined(STM32F4)
    af_out(GPIOB, GPIO0, GPIO_AF2, GPIO_OSPEED_2MHZ);
#else
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, // Low speed (only need 1khz)
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO0); // GPIOB0=TIM3.CH3
#endif
    Fixed1K = true; // Running 1kHz Fixed
  } else {
    DigIo::PWM3.Configure(GPIOB, GPIO0, PinMode::OUTPUT);
  }

  /// PWM2
  if (Param::GetInt(Param::PWM2Func) == IOMatrix::PWM_TIM3) {
#if defined(STM32F4)
    af_out(GPIOA, GPIO7, GPIO_AF2, GPIO_OSPEED_2MHZ); // PA7 = TIM3 CH2
#else
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, // Low speed (only need 1khz)
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7); // GPIOE9=TIM3.CH2
#endif
  } else if (Param::GetInt(Param::PWM2Func) == IOMatrix::CP_SPOOF ||
             Param::GetInt(Param::PWM2Func) == IOMatrix::GS450HOIL ||
             Param::GetInt(Param::PWM2Func) == IOMatrix::PWMTEMPGAUGE ||
             Param::GetInt(Param::PWM2Func) == IOMatrix::PWMSOCGAUGE) {
#if defined(STM32F4)
    af_out(GPIOA, GPIO7, GPIO_AF2, GPIO_OSPEED_2MHZ);
#else
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, // Low speed (only need 1khz)
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7); // GPIOE9=TIM3.CH2
#endif
    Fixed1K = true; // Running 1kHz Fixed
  } else {
    DigIo::PWM2.Configure(GPIOA, GPIO7, PinMode::OUTPUT);
  }

  /// PWM1
  if (Param::GetInt(Param::PWM1Func) == IOMatrix::PWM_TIM3) {
#if defined(STM32F4)
    af_out(GPIOA, GPIO6, GPIO_AF2, GPIO_OSPEED_2MHZ); // PA6 = TIM3 CH1
#else
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, // Low speed (only need 1khz)
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO6); // GPIOA6=TIM3.CH1
#endif
  } else if (Param::GetInt(Param::PWM1Func) == IOMatrix::CP_SPOOF ||
             Param::GetInt(Param::PWM1Func) == IOMatrix::GS450HOIL ||
             Param::GetInt(Param::PWM1Func) == IOMatrix::PWMTEMPGAUGE ||
             Param::GetInt(Param::PWM1Func) == IOMatrix::PWMSOCGAUGE) {
#if defined(STM32F4)
    af_out(GPIOA, GPIO6, GPIO_AF2, GPIO_OSPEED_2MHZ);
#else
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, // Low speed (only need 1khz)
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO6); // GPIOA6=TIM3.CH1
#endif
    Fixed1K = true; // Running 1kHz Fixed
  } else {
    DigIo::PWM1.Configure(GPIOA, GPIO6, PinMode::OUTPUT);
  }

  timer_disable_counter(TIM3);
  // edge aligned PWM
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

  // Fixed1K uses are incompatible with PWM Heater, throw an error.
  if (Fixed1K && Param::GetInt(Param::Heater) == HeatType::PWM) {
    ErrorMessage::Post(ERR_PWMCONFIGERROR);
  }

  if (!Fixed1K) // No CP Spoof Selected or GS450h Oil pump or Gauges
  {
    /* Tim3_Presc and Tim3_Period are written straight into the timer.
     * APB1 timers clock at 72 MHz on the F107 and 84 MHz on this F407
     * setup, so a value chosen on the F107 runs faster on the F407.
     * The fixed 1 kHz path below does not use these parameters.
     */
    timer_set_period(TIM3, Param::GetInt(Param::Tim3_Period));
    timer_set_oc_value(TIM3, TIM_OC1, Param::GetInt(Param::Tim3_1_OC));
    timer_set_oc_value(TIM3, TIM_OC2, Param::GetInt(Param::Tim3_2_OC));
    timer_set_oc_value(TIM3, TIM_OC3, Param::GetInt(Param::Tim3_3_OC));
    timer_generate_event(TIM3, TIM_EGR_UG);
    timer_set_prescaler(TIM3, Param::GetInt(Param::Tim3_Presc));
    timer_enable_counter(TIM3);
  } else // No CP Spoof or GS450h Oil pump output selected locks TIM3 Force 1khz
         // clock (996khz)
  {
    // Set the params to indicate that custom params are being ignored.
    Param::SetInt(Param::Tim3_Period, 3999);
    timer_set_period(TIM3, 3999);
    Param::SetInt(Param::Tim3_1_OC, 1);
    timer_set_oc_value(TIM3, TIM_OC1, 1); // No duty set here
    Param::SetInt(Param::Tim3_2_OC, 1);
    timer_set_oc_value(TIM3, TIM_OC2, 1); // No duty set here
    Param::SetInt(Param::Tim3_3_OC, 1);
    timer_set_oc_value(TIM3, TIM_OC3, 1); // No duty set here
    timer_generate_event(TIM3, TIM_EGR_UG);
#if defined(STM32F4)
    /* 84 MHz / (21 * 4000) = 1 kHz. The period stays 3999 so the gauge
     * and pilot duty values keep the same meaning.
     */
    Param::SetInt(Param::Tim3_Presc, 20);
    timer_set_prescaler(TIM3, 20);
#else
    Param::SetInt(Param::Tim3_Presc, 17);
    timer_set_prescaler(TIM3, 17);
#endif
    timer_enable_counter(TIM3);
  }
}
