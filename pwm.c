/* pwm.c - 第5题：TIM4 PWM 背光调光实现 */

#include "pwm.h"

/* ==================== 模块私有全局变量 ==================== */
/* 这些变量在 TIM7_IRQHandler（main.c）中也会被访问，因此通过 pwm.h 导出为 extern。*/
volatile uint16_t current_duty = BACKLIGHT_DIM;  /* 当前占空比（CCR2 值） */
volatile uint8_t  fade_active  = 0;              /* 淡出进行中 */
volatile uint32_t fade_last    = 0;              /* 上次步进时刻（ms_ticks） */

/* ==================== Timer4_PWM_Init ==================== */
/*
 * 初始化步骤：
 *   1. 使能 TIM4 时钟（GPIOD 时钟已由 LEDs_InitPorts 使能，无需重复）
 *   2. 配置 PD13 为复用功能模式（AF2 = TIM4_CH2）
 *   3. 设置 TIM4 预分频和自动重装载值
 *   4. 配置通道2为 PWM 模式1，使能预装载
 *   5. 写入初始占空比，启动定时器
 */
void Timer4_PWM_Init(void)
{
    /* 使能 TIM4 时钟（GPIOD 已在 LEDs_InitPorts 中使能） */
    // RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; /* 已在 LEDs_InitPorts() 中使能，无需重复 */
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    /* PD13 → 复用功能模式（MODER[27:26] = 10b） */
    GPIOD->MODER &= ~(3U << (2 * 13));  /* U：无符号字面量后缀，避免符号位移位溢出 */
    GPIOD->MODER |=  (2U << (2 * 13));

    /* 设置复用功能：AFRH（AFR[1]）中 Pin13 对应位段 [23:20]
     * = (13-8)*4 = 20，写入 AF2（0010）*/
    GPIOD->AFR[1] &= ~(0xFU << ((13 - 8) * 4));
    GPIOD->AFR[1] |=  (2U   << ((13 - 8) * 4));

    /* TIM4 预分频与计数周期 */
    TIM4->PSC = 559;       /* 84 MHz / 560 = 150 kHz 计数时钟 */
    TIM4->ARR = PWM_ARR;   /* ARR = 999 → 150 Hz PWM 频率 */

    /* 通道2：PWM 模式1（OC2M = 110b）+ 预装载使能（OC2PE = 1）
     * PWM 模式1：CNT < CCR2 时输出高电平，否则输出低电平 */
    TIM4->CCMR1 &= ~(TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE);
    TIM4->CCMR1 |=  (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;

    /* 初始占空比：低亮度 */
    TIM4->CCR2 = BACKLIGHT_DIM;
    current_duty = BACKLIGHT_DIM;

    /* 使能通道2输出（CCER：CC2E = 1） */
    TIM4->CCER |= TIM_CCER_CC2E;

    /* 使能自动重装载预装载 */
    TIM4->CR1 |= TIM_CR1_ARPE;

    /* 强制产生更新事件，使 PSC/ARR 立即生效 */
    TIM4->EGR |= TIM_EGR_UG;
    TIM4->SR   = 0;

    /* 启动 TIM4 */
    TIM4->CR1 |= TIM_CR1_CEN;
}

/* ==================== Set_Backlight_Duty ==================== */
/* 直接写入 CCR2（可在 ISR 中安全调用）。超出上限时截断。*/
void Set_Backlight_Duty(uint16_t duty)
{
    if (duty > BACKLIGHT_FULL) duty = BACKLIGHT_FULL;
    current_duty = duty;
    TIM4->CCR2   = duty;
}
