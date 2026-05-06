/* leds.c - 第1/2题：LED 跑马灯与 GPIO 工具函数实现 */

/*
 * 寄存器缩写速查（本文件涉及）
 * ─────────────────────────────────────────────────────────────────────────
 * RCC        Reset and Clock Control               复位与时钟控制器
 * AHB1ENR    AHB1 peripheral clock Enable Register  AHB1总线外设时钟使能寄存器
 *              GPIO 端口均挂在 AHB1 总线上，使用前必须在此寄存器中置位使能
 *
 * MODER      GPIO Mode Register                    GPIO引脚模式寄存器
 *              每引脚占 2 位：00=输入  01=通用输出  10=复用功能  11=模拟
 *              本模块只使用 01（通用推挽输出）
 *
 * ODR        Output Data Register                  GPIO输出数据寄存器
 *              每 bit 对应一个引脚，写 1 → 引脚输出高电平，写 0 → 低电平
 *              可用 |= 置位（不影响其他引脚）或 &= ~ 清位
 *
 * IDR        Input Data Register                   GPIO输入数据寄存器
 *              只读，每 bit 反映对应引脚当前实际电平（在 main.c 中读取按键用）
 * ─────────────────────────────────────────────────────────────────────────
 */

#include "leds.h"

/* ==================== set_output ==================== */
/*
 * 将指定 GPIO 引脚配置为"通用输出模式"（General Purpose Output）。
 *
 * MODER（GPIO Mode Register，GPIO模式寄存器）：
 *   每个引脚对应 2 位，Pin N 对应 Bit[2N+1 : 2N]。
 *   先用 &= ~ 清零目标 2 位（避免残留值干扰），再用 |= 写入 01b = 输出模式。
 */
void set_output(GPIO_TypeDef *port, int pin)
{
    port->MODER &= ~(3 << (2 * pin));  /* MODER：清零 Pin N 的 2 位模式字段 */
    port->MODER |=  (1 << (2 * pin));  /* MODER：写入 01b = 通用输出模式    */
}

/* ==================== u_delay ==================== */
/*
 * 微秒级忙等延时（Busy-wait delay）。
 * 系统时钟 168 MHz，空转循环每次约 10 个时钟周期 ≈ 59.5 ns，
 * 循环次数 = usec × 10 ≈ 1 µs/次（实际精度受编译器优化影响）。
 *
 * volatile 关键字防止编译器将空循环优化掉。
 */
void u_delay(uint32_t usec)
{
    for (volatile uint32_t i = 0; i < (usec * 10); i++);
}

/* ==================== LEDs_InitPorts ==================== */
/*
 * 初始化 LED 数据总线与控制信号所用的全部 GPIO 引脚。
 *
 * 硬件接线（由电路板决定，不可更改）：
 *   GPIOD：PD0, PD1          → 数据总线 D[1:0]（低2位）
 *          PD8, PD9, PD10    → 数据总线 D[10:8]（经 LCD_Output16BitWord 映射）
 *          PD14, PD15        → 数据总线高位
 *          PD5（WR）          → 写使能，低有效
 *          PD7（CS）          → 片选，低有效
 *          PD11（SEL）        → LED/LCD 总线选择
 *   GPIOE：PE7 ~ PE15        → 数据总线 D[15:7]（9位，连接移位寄存器）
 */
void LEDs_InitPorts(void)
{
    /* RCC->AHB1ENR（AHB1 peripheral clock Enable Register）：
     * AHB1总线外设时钟使能寄存器。
     * GPIO 端口时钟默认关闭，必须先在此置位才能访问 GPIO 寄存器，
     * 否则读写 GPIO 寄存器将无任何效果甚至导致总线错误。
     * GPIODEN = GPIOD 时钟使能位，GPIOEEN = GPIOE 时钟使能位。*/
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN   /* 使能 GPIOD 时钟 */
                 |  RCC_AHB1ENR_GPIOEEN;  /* 使能 GPIOE 时钟 */

    /* ---- GPIOD：数据引脚设为输出 ---- */
    /* MODER：写入 01b = 通用推挽输出模式（每次调用 set_output 处理一个引脚）*/
    set_output(GPIOD, 0);   /* PD0：数据总线 D0  */
    set_output(GPIOD, 1);   /* PD1：数据总线 D1  */
    set_output(GPIOD, 8);   /* PD8：数据总线 D8  */
    set_output(GPIOD, 9);   /* PD9：数据总线 D9  */
    set_output(GPIOD, 10);  /* PD10：数据总线 D10 */
    set_output(GPIOD, 14);  /* PD14：数据总线高位 */
    set_output(GPIOD, 15);  /* PD15：数据总线高位 */

    /* ---- GPIOE：PE7~PE15 全部设为输出（9位数据总线高字节） ---- */
    for (int i = 7; i <= 15; i++)
        set_output(GPIOE, i);

    /* ---- 控制信号引脚：WR / CS / SEL 设为输出 ---- */
    set_output(GPIOD, LED_WR_PIN);   /* PD5（WR） */
    set_output(GPIOD, LED_CS_PIN);   /* PD7（CS） */
    set_output(GPIOD, LED_SEL_PIN);  /* PD11（SEL）*/

    /* ODR（Output Data Register，GPIO输出数据寄存器）：
     * 控制信号初始化为高电平（非激活状态）：
     *   WR = 1：写使能未触发（低电平有效）
     *   CS = 1：片选未使能（低电平有效）
     *   SEL = 1：总线默认选择 LED 侧（按具体硬件定义）*/
    GPIOD->ODR |= (1 << LED_WR_PIN)   /* ODR：WR  = 1（高电平，非激活） */
               |  (1 << LED_CS_PIN)   /* ODR：CS  = 1（高电平，非激活） */
               |  (1 << LED_SEL_PIN); /* ODR：SEL = 1（高电平，初始状态）*/
}

/* ==================== LCD_Output16BitWord ==================== */
/*
 * 将 16 位数据字拆分并输出到 GPIOD 和 GPIOE 上的数据总线引脚。
 *
 * 由于硬件走线的限制，16 条数据线并非按顺序连接到 GPIO 的连续引脚上，
 * 而是分散在 GPIOD 和 GPIOE 的不同位段，因此需要进行位域重组：
 *
 *   data[1:0]   → GPIOD[1:0]    (PD1, PD0)
 *   data[4:2]   → GPIOD[10:8]   (PD10, PD9, PD8)
 *   data[15:13] → GPIOD[15:14]  (PD15, PD14)
 *   data[12:4]  → GPIOE[15:7]   (PE15..PE7)
 *
 * 操作步骤：
 *   1. 计算目标位的新值（d 对应 GPIOD，e 对应 GPIOE）
 *   2. 先用 &= ~ 清除目标位段（保留其他引脚不变）
 *   3. 再用 |= 写入新值
 *
 * ODR（Output Data Register）：向目标位写 1 → 高电平，写 0 → 低电平。
 */
void LCD_Output16BitWord(uint16_t data)
{
    uint16_t d = 0, e = 0;

    /* 位域重组：提取 data 各字段并移到目标 GPIO 位置 */
    d |= ((data & 0x3) << 14);       /* data[1:0]   → GPIOD[15:14] */
    d |= ((data >> 2) & 0x3);        /* data[3:2]   → GPIOD[1:0]   */
    d |= ((data >> 13) & 0x7) << 8;  /* data[15:13] → GPIOD[10:8]  */
    e |= ((data >> 4) & 0x1FF) << 7; /* data[12:4]  → GPIOE[15:7]  */

    /* ODR（Output Data Register）：先清除目标位段，再写入新值（Read-Modify-Write）*/
    GPIOD->ODR &= ~((0x3 << 14) | 0x3 | (0x7 << 8));  /* 清除 PD[15:14], PD[1:0], PD[10:8] */
    GPIOE->ODR &= ~(0x1FF << 7);                       /* 清除 PE[15:7] */

    GPIOD->ODR |= d;  /* 写入 GPIOD 数据位 */
    GPIOE->ODR |= e;  /* 写入 GPIOE 数据位 */
}

/* ==================== LEDs_Write ==================== */
/*
 * 通过移位寄存器将 16 位 LED 状态锁存到 LED 阵列。
 *
 * 时序（由移位寄存器芯片的控制逻辑决定）：
 *   SEL = 1  →  选择 LED 数据总线（区分 LED 与 LCD 的共用总线）
 *   CS  = 0  →  使能移位寄存器（片选低有效）
 *   输出 16 位数据到总线
 *   WR  下降沿（1→0→1）产生上升沿 →  移位寄存器锁存当前总线数据
 *   CS  = 1  →  释放总线（片选恢复高电平）
 *
 * ODR（Output Data Register）：通过 |= 置位或 &= ~ 清位单独控制每个引脚。
 */
void LEDs_Write(uint16_t data)
{
    /* ODR：SEL = 1，选择 LED 总线（与 LCD 共用同一组数据引脚，SEL 区分两者）*/
    GPIOD->ODR |=  (1 << LED_SEL_PIN);

    /* ODR：CS = 0，使能移位寄存器（低电平有效）*/
    GPIOD->ODR &= ~(1 << LED_CS_PIN);

    /* 将 16 位数据输出到总线（通过 GPIOD/GPIOE 的 ODR）*/
    LCD_Output16BitWord(data);

    /* ODR：产生 WR 上升沿（1→0→1）以锁存数据到移位寄存器 */
    GPIOD->ODR |=  (1 << LED_WR_PIN);   /* WR = 1（高，准备下降沿） */
    GPIOD->ODR &= ~(1 << LED_WR_PIN);   /* WR = 0（下降沿） */
    GPIOD->ODR |=  (1 << LED_WR_PIN);   /* WR = 1（上升沿：锁存数据）*/

    /* ODR：CS = 1，释放总线 */
    GPIOD->ODR |=  (1 << LED_CS_PIN);
}
