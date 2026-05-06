/* freq_measure.h - 第6题：TIM12 输入捕获频率测量 */
/*
 * 本模块实现基于 TIM12 输入捕获的精确频率测量：
 *   - 捕获引脚：PB14 = TIM12_CH1（AF9）
 *   - 上升沿触发，测量相邻两次上升沿之间的 Tick 数
 *   - TIM12 为 16 位定时器，通过溢出计数器扩展为虚拟 32 位计数器
 *   - 频率计算：cap_freq_hz = TIM12_CLOCK_HZ / cap_ticks（纯整数运算）
 *
 * 时钟配置：
 *   TIM12 时钟 = APB1 定时器时钟 = 84 MHz
 *   PSC = 0  →  1 Tick ≈ 11.9 ns
 *   ARR = 0xFFFF  →  溢出周期 ≈ 780 µs
 *
 * 分辨率说明：
 *   84 MHz 时钟下，78 kHz 信号对应约 1075 个 Tick，量化误差 < 0.03%。
 *   若使用 1 MHz 时钟（PSC=83），仅 13 个 Tick，误差约 6%。
 *
 * 中断向量：
 *   TIM8_BRK_TIM12_IRQn（TIM12 与 TIM8_BRK 在 STM32F407 上共用）
 */

#ifndef FREQ_MEASURE_H
#define FREQ_MEASURE_H

#include "stm32f4xx.h"

/* ==================== 配置常量 ==================== */
#define TIM12_CLOCK_HZ  84000000UL  /* TIM12 计数时钟（PSC=0，84 MHz） */
#define CAP_TIMEOUT_MS  2000UL      /* 无信号超时时间：2 秒后显示 "--" */
/* TIM12 输入捕获预分频：
 * IC1PSC=11b 时每 8 个上升沿才产生一次捕获中断，显著降低高频时 ISR 压力。 */
#define CAP_EDGE_DIV    8UL

/* ==================== 测量结果（ISR 写入，主循环读取） ==================== */
extern volatile uint32_t cap_ticks;    /* 两次上升沿之间的 Tick 数 */
extern volatile uint32_t cap_freq_hz;  /* 计算所得频率（Hz，整数） */
extern volatile uint32_t cap_last_ms;  /* 最近一次有效测量的时间戳（ms_ticks） */
extern volatile uint8_t  cap_valid;    /* 1 = 至少已完成一次有效测量 */

/* ==================== 函数声明 ==================== */

/**
 * Timer12_Capture_Init - 初始化 TIM12 输入捕获（PB14，上升沿触发）
 *
 * 配置 PB14 为 AF9（TIM12_CH1），设置 TIM12 为 84 MHz 无预分频模式，
 * 使能捕获中断（CC1IE）和溢出中断（UIE），并以高优先级启动。
 */
void Timer12_Capture_Init(void);

#endif /* FREQ_MEASURE_H */
