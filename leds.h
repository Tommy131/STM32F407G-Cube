/* leds.h - 第1/2题：LED 跑马灯与 GPIO 工具函数 */
/*
 * 本模块封装了实验第1、2题所需的全部底层 GPIO 操作：
 *   - 通用 GPIO 输出配置（set_output）
 *   - 微秒忙等延时（u_delay）
 *   - LED 与 LCD 数据总线端口初始化（LEDs_InitPorts）
 *   - 16 位并行数据输出至 LCD 数据总线（LCD_Output16BitWord）
 *   - 16 路 LED 写入（LEDs_Write）
 *
 * 引脚分配（控制信号，均位于 GPIOD）：
 *   PD5  = WR（写使能，低有效）
 *   PD7  = CS（片选，低有效）
 *   PD11 = SEL（选择 LED / LCD 数据总线）
 */

#ifndef LEDS_H
#define LEDS_H

#include "stm32f4xx.h"

/* ==================== 控制引脚定义 ==================== */
#define LED_WR_PIN  5   /* PD5：写使能 */
#define LED_CS_PIN  7   /* PD7：片选   */
#define LED_SEL_PIN 11  /* PD11：总线选择 */

/* ==================== 函数声明 ==================== */

/**
 * set_output - 将指定 GPIO 引脚配置为推挽输出模式
 * @port: GPIO 端口指针（如 GPIOD）
 * @pin:  引脚编号（0~15）
 */
void set_output(GPIO_TypeDef *port, int pin);

/**
 * u_delay - 微秒级忙等延时
 * @usec: 延时时长（微秒）
 *
 * 基于 CPU 空转循环实现，精度受编译器优化影响。
 * 系统时钟 = 168 MHz，每次循环约 100 ns。
 */
void u_delay(uint32_t usec);

/**
 * LEDs_InitPorts - 初始化 LED 和 LCD 数据总线所用的全部 GPIO 端口
 *
 * 使能 GPIOD + GPIOE 时钟，将相关引脚设为输出模式，
 * 并将控制信号（WR/CS/SEL）初始化为高电平（非激活状态）。
 */
void LEDs_InitPorts(void);

/**
 * LCD_Output16BitWord - 将 16 位数据写入 GPIOD/GPIOE 数据总线
 * @data: 待输出的 16 位数据
 *
 * 数据位映射：
 *   Bit[1:0]  → PD[1:0]
 *   Bit[4:2]  → PD[10:8]
 *   Bit[15:13]→ PD[15:14]（注：位顺序见实现）
 *   Bit[12:4] → PE[15:7]
 */
void LCD_Output16BitWord(uint16_t data);

/**
 * LEDs_Write - 将 16 位数据写入 LED 移位寄存器
 * @data: 16 路 LED 状态（bit0 = LED0, bit15 = LED15）
 *
 * 时序：SEL=1 选择 LED 总线 → CS=0 使能 → 输出数据 →
 *        WR 产生上升沿锁存 → CS=1 释放。
 */
void LEDs_Write(uint16_t data);

#endif /* LEDS_H */
