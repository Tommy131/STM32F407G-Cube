/* pwm.h - 第5题：TIM4 PWM 背光调光 */
/*
 * 本模块封装了实验第5题所需的 PWM 背光控制：
 *   - TIM4_CH2（PD13）产生 150 Hz PWM 信号
 *   - 支持动态占空比调节（0%~100%）
 *   - 提供淡出所需的共享状态变量（由 TIM7 中断使用）
 *
 * 硬件连接：
 *   PD13 = TIM4_CH2（AF2，参见 STM32F407 数据手册表9，第60页）
 *
 * PWM 频率计算：
 *   TIM4 时钟 = 84 MHz（APB1 定时器时钟 = APB1 × 2）
 *   PSC = 559  →  计数时钟 = 84 MHz / 560 = 150 kHz
 *   ARR = 999  →  PWM 频率 = 150 kHz / 1000 = 150 Hz
 */

#ifndef PWM_H
#define PWM_H

#include "stm32f4xx.h"

/* ==================== PWM 亮度参数 ==================== */
#define PWM_ARR        999    /* TIM4 ARR：决定 PWM 频率分辨率（1000 级） */
#define BACKLIGHT_DIM  200    /* 低亮度：CCR2 = 200 → 占空比 20% */
#define BACKLIGHT_FULL 1000   /* 全亮度：CCR2 = 1000 → 占空比 100% */

/* 淡出参数：
 *   每 FADE_INTERVAL ms 将占空比减少 FADE_STEP
 *   总步数 = (1000 - 200) / 4 = 200 步
 *   总时长 = 200 × 10 ms = 2000 ms ≈ 2 秒 */
#define FADE_STEP      4      /* 每步减少的 CCR2 值 */
#define FADE_INTERVAL  10     /* 淡出步进间隔（ms） */

/* ==================== 共享状态变量（TIM7 中断中访问） ==================== */
extern volatile uint16_t current_duty;  /* 当前 CCR2 值（亮度） */
extern volatile uint8_t  fade_active;   /* 淡出进行中标志 */
extern volatile uint32_t fade_last;     /* 上次淡出步进时刻（ms_ticks） */

/* ==================== 函数声明 ==================== */

/**
 * Timer4_PWM_Init - 初始化 TIM4 通道2 PWM 输出（PD13，150 Hz）
 *
 * 将 PD13 配置为 AF2（TIM4_CH2），并以低亮度（BACKLIGHT_DIM）启动 PWM。
 * 调用此函数前须先调用 LEDs_InitPorts()（GPIOD 时钟已在其中使能）。
 */
void Timer4_PWM_Init(void);

/**
 * Set_Backlight_Duty - 设置 PWM 占空比（背光亮度）
 * @duty: 目标 CCR2 值（0 = 全灭，1000 = 全亮）
 *
 * 超过 BACKLIGHT_FULL 时自动截断。可在 ISR 中安全调用。
 */
void Set_Backlight_Duty(uint16_t duty);

#endif /* PWM_H */
