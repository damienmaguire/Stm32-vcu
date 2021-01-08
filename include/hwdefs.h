#ifndef HWDEFS_H_INCLUDED
#define HWDEFS_H_INCLUDED


//Common for any config

//Maximum value for over current limit timer
#define GAUGEMAX 4096
#define USART_BAUDRATE 115200
//Maximum PWM frequency is 36MHz/2^MIN_PWM_DIGITS
#define MIN_PWM_DIGITS 11
#define PERIPH_CLK      ((uint32_t)36000000)

#define RCC_CLOCK_SETUP rcc_clock_setup_in_hse_8mhz_out_72mhz

#define PWM_TIMER     TIM1
#define PWM_TIMRST    RST_TIM1
#define PWM_TIMER_IRQ NVIC_TIM1_UP_IRQ
#define pwm_timer_isr tim1_up_isr

//#define REV_CNT_RCC_ENR    RCC_APB1ENR_TIM3EN
//#define rev_timer_isr      tim3_isr
//#define REV_CNT_TIMER      TIM3
//#define REV_CNT_TIMRST     RST_TIM3
#define FUELGAUGE_TIMER TIM4

#define TERM_USART         USART3
#define TERM_USART_TXPIN   GPIO_USART3_PR_TX
#define TERM_USART_TXPORT  GPIOC
#define TERM_USART_DMARX   DMA_CHANNEL3
#define TERM_USART_DMATX   DMA_CHANNEL2 //this means we can not use it on rev1 hardware (TIM3_CH3)
#define TERM_USART_DR      USART3_DR
#define TERM_BUFSIZE       128
//Address of parameter block in flash for 103
//#define FLASH_PAGE_SIZE 1024
//#define PARAM_ADDRESS 0x0801FC00
//#define PARAM_BLKSIZE FLASH_PAGE_SIZE
//#define CANMAP_ADDRESS 0x0801F800

//Address of parameter block in flash for 105
#define FLASH_PAGE_SIZE 2048
#define PARAM_ADDRESS 0x0801F800
#define PARAM_BLKSIZE FLASH_PAGE_SIZE
#define CANMAP_ADDRESS 0x0801F000


//#define REV_CNT_IC         hwRev == HW_REV1 ? TIM_IC3 : TIM_IC1
//#define REV_CNT_CCR        hwRev == HW_REV1 ? TIM3_CCR3 : TIM3_CCR1
//#define REV_CNT_CCR_PTR    hwRev == HW_REV1 ? (uint32_t)&TIM3_CCR3 : (uint32_t)&TIM3_CCR1
//#define REV_CNT_CCER       hwRev == HW_REV1 ? TIM_CCER_CC3P : TIM_CCER_CC1P
//#define REV_CNT_SR         hwRev == HW_REV1 ? TIM_SR_CC3IF : TIM_SR_CC1IF
//#define REV_CNT_DMAEN      hwRev == HW_REV1 ? TIM_DIER_CC3DE : TIM_DIER_CC1DE
//#define REV_CNT_DMACHAN    hwRev == HW_REV1 ? DMA_CHANNEL2 : DMA_CHANNEL6
//#define REV_CNT_DMA_CNDTR  hwRev == HW_REV1 ? DMA1_CNDTR2 : DMA1_CNDTR6

typedef enum
{
    HW_REV1, HW_REV2, HW_REV3, HW_TESLA
} HWREV;

extern HWREV hwRev;

#endif // HWDEFS_H_INCLUDED
