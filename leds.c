/* leds.c - 第1/2题：LED 跑马灯与 GPIO 工具函数实现 */

#include "leds.h"

/* ==================== set_output ==================== */
/* 将指定引脚的 MODER[2n+1:2n] 设为 01b（通用输出模式）。*/
void set_output(GPIO_TypeDef *port, int pin)
{
    port->MODER &= ~(3 << (2 * pin));
    port->MODER |=  (1 << (2 * pin));
}

/* ==================== u_delay ==================== */
/* 系统时钟 168 MHz，每次循环约 10 个时钟周期 ≈ 59.5 ns；
 * 循环次数 = usec × 10 ≈ 1 µs/次（误差受编译优化影响）。 */
void u_delay(uint32_t usec)
{
    for (volatile uint32_t i = 0; i < (usec * 10); i++);
}

/* ==================== LEDs_InitPorts ==================== */
/*
 * 初始化 LED 数据总线所用 GPIO 引脚：
 *
 *   GPIOD：PD0, PD1（数据低位）
 *          PD8, PD9, PD10（数据中位）
 *          PD14, PD15（数据高位）
 *          PD5（WR）, PD7（CS）, PD11（SEL）——控制信号
 *
 *   GPIOE：PE7 ~ PE15（9 位数据高字节）
 *
 * 控制信号初始电平：均拉高（非激活状态）。
 */
void LEDs_InitPorts(void)
{
    /* 使能 GPIOD 和 GPIOE 时钟 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;

    /* GPIOD 数据/控制引脚设为输出 */
    set_output(GPIOD, 0);  set_output(GPIOD, 1);
    set_output(GPIOD, 8);  set_output(GPIOD, 9);  set_output(GPIOD, 10);
    set_output(GPIOD, 14); set_output(GPIOD, 15);

    /* GPIOE PE7~PE15 设为输出 */
    for (int i = 7; i <= 15; i++)
        set_output(GPIOE, i);

    /* 控制信号引脚设为输出 */
    set_output(GPIOD, LED_WR_PIN);
    set_output(GPIOD, LED_CS_PIN);
    set_output(GPIOD, LED_SEL_PIN);

    /* 控制信号初始电平：高（非激活） */
    GPIOD->ODR |= (1 << LED_WR_PIN)
               |  (1 << LED_CS_PIN)
               |  (1 << LED_SEL_PIN);
}

/* ==================== LCD_Output16BitWord ==================== */
/*
 * 将 16 位字拆分为 GPIOD 和 GPIOE 的对应位段并输出。
 *
 * 位映射（硬件走线决定）：
 *   data[1:0]   → GPIOD[1:0]    (PD1, PD0)
 *   data[4:2]   → GPIOD[10:8]   (PD10, PD9, PD8)
 *   data[15:13] → GPIOD[15:14]  (PD15, PD14)  ← 仅取高3位中的低2位
 *   data[12:4]  → GPIOE[15:7]   (PE15..PE7)
 *
 * 注意：先清除目标位段，再写入新值，避免影响其余引脚。
 */
void LCD_Output16BitWord(uint16_t data)
{
    uint16_t d = 0, e = 0;

    d |= ((data & 0x3) << 14);           /* data[1:0]  → PD[15:14] */
    d |= ((data >> 2) & 0x3);            /* data[3:2]  → PD[1:0]   */
    d |= ((data >> 13) & 0x7) << 8;      /* data[15:13]→ PD[10:8]  */
    e |= ((data >> 4) & 0x1FF) << 7;     /* data[12:4] → PE[15:7]  */

    GPIOD->ODR &= ~((0x3 << 14) | 0x3 | (0x7 << 8));
    GPIOE->ODR &= ~(0x1FF << 7);

    GPIOD->ODR |= d;
    GPIOE->ODR |= e;
}

/* ==================== LEDs_Write ==================== */
/*
 * 通过移位寄存器将 16 位数据锁存到 LED 阵列。
 *
 * 时序：
 *   SEL = 1  →  选择 LED 总线（非 LCD）
 *   CS  = 0  →  使能移位寄存器
 *   输出数据至数据总线
 *   WR  上升沿（0→1）→  锁存数据
 *   CS  = 1  →  释放总线
 */
void LEDs_Write(uint16_t data)
{
    GPIOD->ODR |=  (1 << LED_SEL_PIN);   /* SEL = 1：选 LED 总线 */
    GPIOD->ODR &= ~(1 << LED_CS_PIN);    /* CS  = 0：使能 */

    LCD_Output16BitWord(data);            /* 输出 16 位数据 */

    GPIOD->ODR |=  (1 << LED_WR_PIN);    /* WR = 1（准备产生上升沿） */
    GPIOD->ODR &= ~(1 << LED_WR_PIN);    /* WR = 0 */
    GPIOD->ODR |=  (1 << LED_WR_PIN);    /* WR = 1：上升沿锁存数据 */

    GPIOD->ODR |=  (1 << LED_CS_PIN);    /* CS = 1：释放总线 */
}
