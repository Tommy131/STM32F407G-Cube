/* Mikrocontroller-Programmierung - 最终融合版本 */
/* 特性：跑马灯持续运行 + PD12严格1Hz（中断控制）+ 10秒倒计时关闭显示屏 */

#include "stm32f4xx.h"
#include "_mcpr_stm32f407.h"
#include <stdio.h>
#include <inttypes.h>
#include "display.h"

/* ==================== 引脚定义 ==================== */
#define LED_WR_PIN  5
#define LED_CS_PIN  7
#define LED_SEL_PIN 11

/* ==================== 全局状态变量 ==================== */
volatile uint32_t ms_ticks        = 0;
volatile uint8_t  button_pressed  = 0;    // 当前按键是否按下
volatile uint8_t  key_last        = 0;    // 上一次按键状态（用于边沿检测）

volatile uint8_t  countdown_active = 0;   // 倒计时是否进行中
volatile uint32_t release_time     = 0;   // 松开时刻 (ms)
volatile uint8_t  last_displayed   = 255; // 上次显示的剩余秒数

volatile uint8_t  backlight_on     = 1;   // 背光状态
volatile int      counter          = 0;   // LCD 显示计数器

/* ==================== PD12 闪烁控制（仅在中断中使用） ==================== */
static volatile uint32_t led_toggle_last = 0;  // 上次翻转时刻
static volatile uint8_t  led_state       = 0;  // 当前LED状态

/* ==================== 函数声明 ==================== */
void set_output(GPIO_TypeDef *port, int pin);
void LEDs_InitPorts(void);
void LCD_Output16BitWord(uint16_t data);
void LEDs_Write(uint16_t data);
void Timer7_Init(void);
void Reset_Counter(void);
void My_WriteToLCD_Int(int value);
void u_delay(uint32_t usec);

/* ==================== Timer7 初始化（1ms 中断） ==================== */
void Timer7_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

    TIM7->CR1  = 0;
    TIM7->PSC  = 8399;   // 84MHz / 8400 = 10kHz
    TIM7->ARR  = 9;      // 10kHz / 10   = 1kHz → 周期 1ms

    TIM7->CR1  |= TIM_CR1_ARPE | TIM_CR1_URS;
    TIM7->EGR  |= TIM_EGR_UG;
    TIM7->SR    = 0;
    TIM7->DIER |= TIM_DIER_UIE;

    NVIC_SetPriority(TIM7_IRQn, 1);
    NVIC_EnableIRQ(TIM7_IRQn);

    TIM7->CR1  |= TIM_CR1_CEN;
}

/* ==================== Timer7 中断服务函数（核心调度） ==================== */
/*
   所有对时序有严格要求的逻辑都在这里处理：
   - 按键边沿检测
   - PD12 1Hz 精确闪烁
   - 倒计时逻辑
   - LCD 刷新
*/
void TIM7_IRQHandler(void)
{
    if (!(TIM7->SR & TIM_SR_UIF)) return;
    TIM7->SR &= ~TIM_SR_UIF;   // 清除中断标志

    ms_ticks++;

    /* -------------------------------------------------- */
    /* 1. 按键检测（每 1ms 采样，每 50ms 做一次边沿判断）  */
    /* -------------------------------------------------- */
    static uint32_t key_check_last = 0;

    if (ms_ticks - key_check_last >= 50)
    {
        key_check_last = ms_ticks;

        uint8_t key_now = (GPIOA->IDR & 1) != 0;

        /* 上升沿：按下 */
        if (key_now && !key_last)
        {
            button_pressed   = 1;
            countdown_active = 0;           // 取消倒计时
            Reset_Counter();

            /* 背光打开 */
            if (!backlight_on)
            {
                GPIOD->ODR |= (1 << 13);
                backlight_on = 1;
            }
        }

        /* 下降沿：松开 */
        if (!key_now && key_last)
        {
            button_pressed   = 0;
            release_time     = ms_ticks;
            countdown_active = 1;
            last_displayed   = 255;

            /* 松开后 PD12 立即常亮 */
            led_state = 1;
            GPIOD->ODR |= (1 << 12);
        }

        key_last = key_now;
    }

    /* -------------------------------------------------- */
    /* 2. PD12 精确 1Hz 闪烁（仅按键按下时有效）           */
    /*    用 ms_ticks 直接判断，严格 500ms 翻转一次         */
    /* -------------------------------------------------- */
    if (button_pressed)
    {
        if (ms_ticks - led_toggle_last >= 500)
        {
            led_toggle_last = ms_ticks;
            led_state ^= 1;

            if (led_state)
                GPIOD->ODR |=  (1 << 12);
            else
                GPIOD->ODR &= ~(1 << 12);
        }
    }

    /* -------------------------------------------------- */
    /* 3. 倒计时逻辑（松开后 10 秒）                       */
    /* -------------------------------------------------- */
    if (countdown_active)
    {
        uint32_t elapsed   = ms_ticks - release_time;
        uint8_t  remaining = (elapsed < 10000)
                             ? (uint8_t)(10 - elapsed / 1000)
                             : 0;

        /* 每秒刷新一次 */
        if (remaining != last_displayed)
        {
            last_displayed = remaining;

            // if (remaining < 16) LEDs_Write(1 << remaining);  // 外部LED显示剩余秒数

            My_WriteToLCD_Int(counter++);    // LCD显示计数
        }

        /* 10秒到期 */
        if (elapsed >= 10000)
        {
            countdown_active = 0;

            /* 关闭背光 */
            GPIOD->ODR &= ~(1 << 13);
            backlight_on = 0;

            LCD_ClearDisplay(0xFE00);        // 清屏
            LEDs_Write(0x0000);              // 外部LED全灭
            Reset_Counter();
        }
    }
}

/* ==================== 主函数 ==================== */
int main(void)
{
    mcpr_SetSystemCoreClock();

    /* ---- 外设初始化 ---- */
    LEDs_InitPorts();

    LCD_Init();
    LCD_ClearDisplay(0xFE00);

    /* ---- GPIO 时钟 ---- */
    RCC->AHB1ENR |= (1 << 0) | (1 << 3);   // GPIOA + GPIOD

    /* ---- PA0：USER 键输入 ---- */
    GPIOA->MODER &= ~(0x00000003);

    /* ---- PD12：板载 LED 输出，初始常亮 ---- */
    set_output(GPIOD, 12);
    GPIOD->ODR |= (1 << 12);
    led_state = 1;

    /* ---- PD13：背光输出，初始打开 ---- */
    set_output(GPIOD, 13);
    GPIOD->ODR |= (1 << 13);

    /* ---- 启动 Timer7 ---- */
    Timer7_Init();

    /* ==================== 主循环：持续跑马灯 ==================== */
    /*
       跑马灯完全在主循环中运行。
       PD12 闪烁、按键检测、倒计时全部由 TIM7 中断处理。
       两者完全解耦，互不干扰。
    */
    while (1)
    {
        /* ---- 正向单灯跑马灯 ---- */
        for (int i = 0; i < 16; i++)
        {
            LEDs_Write(1 << i);
            u_delay(120000);   // 约 120ms，可调整
        }

        /* ---- 全亮逐个熄灭 ---- */
        for (int i = 0; i < 16; i++)
        {
            LEDs_Write(~(uint16_t)(1 << i));
            u_delay(120000);
        }
    }
}

/* ==================== 辅助函数（与最新代码保持一致） ==================== */

void u_delay(uint32_t usec)
{
    for (volatile uint32_t i = 0; i < (usec * 10); i++);
}

void set_output(GPIO_TypeDef *port, int pin)
{
    port->MODER &= ~(3 << (2 * pin));
    port->MODER |= (1 << (2 * pin));
}

void LEDs_InitPorts(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;

    set_output(GPIOD, 0);  set_output(GPIOD, 1);
    set_output(GPIOD, 8);  set_output(GPIOD, 9);  set_output(GPIOD, 10);
    set_output(GPIOD, 14); set_output(GPIOD, 15);

    for (int i = 7; i <= 15; i++)
        set_output(GPIOE, i);

    set_output(GPIOD, LED_WR_PIN);
    set_output(GPIOD, LED_CS_PIN);
    set_output(GPIOD, LED_SEL_PIN);

    GPIOD->ODR |= (1 << LED_WR_PIN)
               |  (1 << LED_CS_PIN)
               |  (1 << LED_SEL_PIN);
}

void LCD_Output16BitWord(uint16_t data)
{
    uint16_t d = 0, e = 0;

    d |= ((data & 0x3) << 14);
    d |= ((data >> 2) & 0x3);
    d |= ((data >> 13) & 0x7) << 8;
    e |= ((data >> 4) & 0x1FF) << 7;

    GPIOD->ODR &= ~((0x3 << 14) | 0x3 | (0x7 << 8));
    GPIOE->ODR &= ~(0x1FF << 7);

    GPIOD->ODR |= d;
    GPIOE->ODR |= e;
}

void LEDs_Write(uint16_t data)
{
    GPIOD->ODR |=  (1 << LED_SEL_PIN);
    GPIOD->ODR &= ~(1 << LED_CS_PIN);

    LCD_Output16BitWord(data);

    GPIOD->ODR |=  (1 << LED_WR_PIN);
    GPIOD->ODR &= ~(1 << LED_WR_PIN);
    GPIOD->ODR |=  (1 << LED_WR_PIN);

    GPIOD->ODR |=  (1 << LED_CS_PIN);
}

void Reset_Counter(void)
{
    counter = 0;
}

void My_WriteToLCD_Int(int value)
{
    char buffer[8];
    sprintf(buffer, "%d", value);
    LCD_WriteString(10, 10, 0xFFFF, 0x0000, buffer);
}