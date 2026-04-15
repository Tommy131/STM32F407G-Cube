#include "stm32f4xx.h"
#include "_mcpr_stm32f407.h"
#include <stdio.h>
#include <inttypes.h>
#include "display.h"

/* 控制引脚定义 */
#define LED_WR_PIN 5
#define LED_CS_PIN 7
#define LED_SEL_PIN 11

/* ===== 时间变量 ===== */
volatile uint32_t ms_ticks = 0;
volatile uint8_t flag_1s = 0;

volatile int counter = 0;

/* ===== 按键 ===== */
uint8_t key_now = 0;
uint8_t key_last = 0;

/* ===== 背光 ===== */
uint32_t release_time = 0;
uint8_t backlight_on = 1;

/* ===== 倒计时 ===== */
uint8_t countdown_active = 0;
uint8_t last_displayed = 255;

/* ===== 函数声明 ===== */
void set_output(GPIO_TypeDef *port, int pin);
void LEDs_InitPorts();
void LCD_Output16BitWord(uint16_t data);
void LEDs_Write(uint16_t data);
void Timer7_Init(void);
void Reset_Counter();
void My_WriteToLCD_Int(int value);

/* ===== Timer7 ===== */
void Timer7_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

    TIM7->CR1 = 0;

    TIM7->PSC = 8399;
    TIM7->ARR = 9;

    TIM7->CR1 |= TIM_CR1_ARPE;
    TIM7->CR1 |= TIM_CR1_URS;

    TIM7->EGR |= TIM_EGR_UG;

    TIM7->SR = 0;

    TIM7->DIER |= TIM_DIER_UIE;

    NVIC_SetPriority(TIM7_IRQn, 1);
    NVIC_EnableIRQ(TIM7_IRQn);

    TIM7->CR1 |= TIM_CR1_CEN;
}

/* ===== 中断 ===== */
void TIM7_IRQHandler(void)
{
    if (TIM7->SR & TIM_SR_UIF)
    {
        TIM7->SR &= ~TIM_SR_UIF;

        ms_ticks++;

        if (ms_ticks % 1000 == 0)
        {
            flag_1s = 1;
        }
    }
}

/* ===== 主函数 ===== */
int main(void)
{
    mcpr_SetSystemCoreClock();

    LEDs_InitPorts();

    LCD_Init();
    LCD_ClearDisplay(0xFE00);

    RCC->AHB1ENR |= (1 << 0) | (1 << 3);

    /* PA0 输入 */
    GPIOA->MODER &= ~(0x3);

    /* PD12 LED */
    set_output(GPIOD, 12);

    /* PD13 背光 */
    set_output(GPIOD, 13);

    GPIOD->ODR |= (1 << 12);
    GPIOD->ODR |= (1 << 13);

    Timer7_Init();

    uint32_t last_time = 0;
    uint8_t led_state = 0;

    while (1)
    {
        if (ms_ticks - last_time >= 50)
        {
            last_time = ms_ticks;

            /* ===== 读取按键 ===== */
            key_now = (GPIOA->IDR & 1);

            /* ===== 按下 ===== */
            if (key_now)
            {
                /* 停止倒计时 */
                countdown_active = 0;
								Reset_Counter();

                /* 背光开 */
                GPIOD->ODR |= (1 << 13);
                backlight_on = 1;

                /* 1Hz闪烁 */
                if (flag_1s)
                {
                    flag_1s = 0;
                    led_state ^= 1;

                    if (led_state)
                        GPIOD->ODR |= (1 << 12);
                    else
                        GPIOD->ODR &= ~(1 << 12);
                }
            }
            else
            {
                /* LED常亮 */
                GPIOD->ODR |= (1 << 12);
            }

            /* ===== 松开 ===== */
            if (key_last == 1 && key_now == 0)
            {
                release_time = ms_ticks;
                countdown_active = 1;
                last_displayed = 255;
            }

            /* ===== 倒计时逻辑 ===== */
            if (countdown_active)
            {
                uint32_t elapsed = ms_ticks - release_time;

                if (elapsed >= 10000)
                {
                    countdown_active = 0;

                    /* 关闭背光 */
                    if (backlight_on)
                    {
                        GPIOD->ODR &= ~(1 << 13);
                        backlight_on = 0;
                    }

                    LEDs_Write(0);        // 清空LED
                    Reset_Counter();         // 重置倒计时
                    LCD_ClearDisplay(0xFE00);
                }
                else
                {
                    uint8_t remaining = 10 - (elapsed / 1000);

                    if (remaining != last_displayed)
                    {
                        last_displayed = remaining;

                        /* 用LED显示倒计时 */
                        if (remaining < 16)
                        {
                            LEDs_Write(1 << remaining);
                            My_WriteToLCD_Int(counter++);
                        }
                    }
                }
            }

            key_last = key_now;
        }
    }
}

/* ===== GPIO ===== */
void set_output(GPIO_TypeDef *port, int pin)
{
    port->MODER &= ~(3 << (2 * pin));
    port->MODER |= (1 << (2 * pin));
}

/* ===== LED接口 ===== */
void LEDs_InitPorts()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

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

    set_output(GPIOD, LED_WR_PIN);
    set_output(GPIOD, LED_CS_PIN);
    set_output(GPIOD, LED_SEL_PIN);

    GPIOD->ODR |= (1 << LED_WR_PIN);
    GPIOD->ODR |= (1 << LED_CS_PIN);
    GPIOD->ODR |= (1 << LED_SEL_PIN);
}

void LCD_Output16BitWord(uint16_t data)
{
    uint16_t d = 0;
    uint16_t e = 0;

    d |= ((data & 0x3) << 14);
    d |= ((data >> 2) & 0x3);
    d |= ((data >> 13) & 0x7) << 8;
    e |= ((data >> 4) & 0x1FF) << 7;

    GPIOD->ODR &= ~((0x3 << 14) | (0x3) | (0x7 << 8));
    GPIOE->ODR &= ~(0x1FF << 7);

    GPIOD->ODR |= d;
    GPIOE->ODR |= e;
}

void LEDs_Write(uint16_t data)
{
    GPIOD->ODR |= (1 << LED_SEL_PIN);
    GPIOD->ODR &= ~(1 << LED_CS_PIN);

    LCD_Output16BitWord(data);

    GPIOD->ODR |= (1 << LED_WR_PIN);
    GPIOD->ODR &= ~(1 << LED_WR_PIN);
    GPIOD->ODR |= (1 << LED_WR_PIN);

    GPIOD->ODR |= (1 << LED_CS_PIN);
}

void Reset_Counter()
{
	 counter = 0;
}

void My_WriteToLCD_Int(int value)
{
    char buffer[4];
		sprintf(buffer, "%d", value);

    LCD_WriteString(10, 10, 0xFFFF, 0x0000, buffer);
}
