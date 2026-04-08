/* Mikrocontroller-Programmierung Start-Projekt */
/* U. Margull, 29.2.2024 (V1.5)                 */

#include "stm32f4xx.h"
#include "_mcpr_stm32f407.h"
#include <inttypes.h>

/* 控制引脚定义 */
#define LED_WR_PIN 5   // PD5
#define LED_CS_PIN 7   // PD7 (NE1)
#define LED_SEL_PIN 11 // PD11 (A16)

/* 函数声明 */
void u_delay(uint32_t usec);
void set_output(GPIO_TypeDef *port, int pin);
void LCD_Output16BitWord(uint16_t data);
void LEDs_InitPorts();
void LEDs_Write(uint16_t data);

int main(void)
{
    mcpr_SetSystemCoreClock();

    LEDs_InitPorts();

    /* GPIOA & GPIOD 时钟 */
    RCC->AHB1ENR |= (1 << 0) | (1 << 3);

    /* PA0 输入（按键） */
    GPIOA->MODER &= ~(0x00000003);

    /* PD12 输出（板载LED） */
    GPIOD->MODER |= 0x01000000;

    while (1)
    {
        // 当按钮被激活，循环闪烁LED灯
        if ((GPIOA->IDR & 1) != 0)
        {
            GPIOD->ODR |= 1 << 12;
            u_delay(500000);
            GPIOD->ODR &= ~(1 << 12);
            u_delay(500000);
        }
        else
        {
            GPIOD->ODR &= ~(1 << 12); // ~(1<<12);
        }

        /* -------- Lauflicht 正向 (只开一盏灯) -------- */
        for (int i = 0; i < 16; i++)
        {
            LEDs_Write(1 << i);
            u_delay(200000);
        }

        /* -------- Lauflicht 正向 (全开, 只关一盏灯) -------- */
        for (int i = 0; i < 16; i++)
        {
            LEDs_Write(~(1 << i));
            u_delay(200000);
        }

        /* -------- Lauflicht 反向 -------- */
        /* for (int i = 15; i >= 0; i--)
        {
            LEDs_Write(1 << i);
            u_delay(200000);
        } */
    }
}

/* 延时 */
void u_delay(uint32_t usec)
{
    for (volatile uint32_t i = 0; i < (usec * 10); i++)
        ;
}

/* 设置GPIO为输出 */
void set_output(GPIO_TypeDef *port, int pin)
{
    port->MODER &= ~(3 << (2 * pin));
    port->MODER |= (1 << (2 * pin));
}

/* 初始化端口 */
void LEDs_InitPorts()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

    /* -------- 数据引脚 -------- */

    set_output(GPIOD, 0);
    set_output(GPIOD, 1);

    set_output(GPIOD, 8);
    set_output(GPIOD, 9);
    set_output(GPIOD, 10);

    set_output(GPIOD, 14);
    set_output(GPIOD, 15);

    for (int i = 7; i <= 15; i++)
    {
        set_output(GPIOE, i);
    }

    /* -------- 控制引脚 -------- */

    set_output(GPIOD, LED_WR_PIN);
    set_output(GPIOD, LED_CS_PIN);
    set_output(GPIOD, LED_SEL_PIN);

    /* 初始状态 */
    GPIOD->ODR |= (1 << LED_WR_PIN);  // WR = HIGH
    GPIOD->ODR |= (1 << LED_CS_PIN);  // CS = HIGH（未选中）
    GPIOD->ODR |= (1 << LED_SEL_PIN); // SEL = HIGH（选LED）
}

/* 输出16位数据 */
void LCD_Output16BitWord(uint16_t data)
{
    uint16_t d = 0;
    uint16_t e = 0;

    /* 数据映射 */

    // bit0-1 -> PD14-15
    d |= ((data & 0x3) << 14);

    // bit2-3 -> PD0-1
    d |= ((data >> 2) & 0x3);

    // bit13-15 -> PD8-10
    d |= ((data >> 13) & 0x7) << 8;

    // bit4-12 -> PE7-15
    e |= ((data >> 4) & 0x1FF) << 7;

    /* 清零相关位 */
    GPIOD->ODR &= ~(
        (0x3 << 14) |
        (0x3) |
        (0x7 << 8));

    GPIOE->ODR &= ~(0x1FF << 7);

    /* 写入 */
    GPIOD->ODR |= d;
    GPIOE->ODR |= e;
}

/* 写入LED（核心函数） */
void LEDs_Write(uint16_t data)
{
    /* 1. 选择 LED 芯片 */
    GPIOD->ODR |= (1 << LED_SEL_PIN); // PD11 = HIGH → LED
    GPIOD->ODR &= ~(1 << LED_CS_PIN); // PD7  = LOW  → 选中

    /* 2. 输出数据 */
    LCD_Output16BitWord(data);

    /* 3. WR 写脉冲 */
    GPIOD->ODR |= (1 << LED_WR_PIN); // 先拉高脉冲
    GPIOD->ODR &= ~(1 << LED_WR_PIN); // 拉低脉冲
    GPIOD->ODR |= (1 << LED_WR_PIN); // 再拉高脉冲
    // HIGH → LOW → HIGH: WS (Write pulse脉冲)
    // 时刻	动作
    // WR 高	空闲
    // WR 下降沿（高→低）	准备写
    // WR 上升沿（低→高）	真正写入数据

    /* 4. 取消选择 */
    GPIOD->ODR |= (1 << LED_CS_PIN);
}