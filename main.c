/* main.c - 微控制器编程实验主文件 */
/*
 * 本文件负责系统初始化、主循环和 TIM7 1ms 心跳中断。
 * 各外设功能已拆分至独立模块：
 *
 *   leds.c / leds.h        - 第1/2题：LED 跑马灯与 GPIO 工具函数
 *   display.c / display.h  - 第3题：ILI9325 LCD 初始化与绘图
 *   pwm.c / pwm.h          - 第5题：TIM4 PWM 背光调光
 *   freq_measure.c / .h    - 第6题：TIM12 输入捕获频率测量
 *
 * 本文件包含：
 *   - 第4题：按键检测、10秒倒计时、PD12 LED 闪烁（均在 TIM7 中断中处理）
 *   - TIM7 初始化（1 ms 节拍）
 *   - LCD 显示刷新（LCD_UpdateDisplay）
 *   - 主循环（跑马灯 + 定期 LCD 刷新）
 */

#include "stm32f4xx.h"
#include "_mcpr_stm32f407.h"
#include <stdio.h>
#include <inttypes.h>
#include "display.h"
#include "leds.h"
#include "pwm.h"
#include "freq_measure.h"

/* ==================== 应用级全局状态变量 ==================== */
volatile uint32_t ms_ticks        = 0;   /* TIM7 1ms 心跳计数 */
volatile uint8_t  button_pressed  = 0;   /* 当前按键是否处于按下状态 */
volatile uint8_t  key_last        = 0;   /* 上次采样的按键状态（边沿检测） */

volatile uint8_t  countdown_active = 0;  /* 10 秒倒计时进行中 */
volatile uint32_t release_time     = 0;  /* 按键松开时刻（ms_ticks） */

volatile int      counter          = 0;  /* LCD 秒计数器（每秒递增） */

/* LCD 刷新请求标志：ISR/其他模块置位，主循环执行实际刷新
 * （避免在中断上下文中执行耗时 GPIO 操作） */
volatile uint8_t  lcd_update_pending = 0;

/* PD12 LED 闪烁控制（仅在 TIM7_IRQHandler 中使用） */
static volatile uint32_t led_toggle_last = 0;
static volatile uint8_t  led_state       = 0;

/* ==================== 函数声明 ==================== */
void Timer7_Init(void);
void LCD_UpdateDisplay(void);

/* ==================== Timer7 初始化（1 ms 中断） ==================== */
/*
 * 定时器配置：
 *   TIM7 时钟 = 84 MHz（APB1 定时器时钟）
 *   PSC = 8399  →  计数时钟 = 84 MHz / 8400 = 10 kHz
 *   ARR = 9     →  中断频率 = 10 kHz / 10 = 1 kHz → 周期 1 ms
 */
void Timer7_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

    TIM7->CR1  = 0;
    TIM7->PSC  = 8399;   /* 84 MHz / 8400 = 10 kHz */
    TIM7->ARR  = 9;      /* 10 kHz / 10   = 1 kHz → 周期 1 ms */

    TIM7->CR1  |= TIM_CR1_ARPE | TIM_CR1_URS;
    TIM7->EGR  |= TIM_EGR_UG;
    TIM7->SR    = 0;
    TIM7->DIER |= TIM_DIER_UIE;

    NVIC_SetPriority(TIM7_IRQn, 1);
    NVIC_EnableIRQ(TIM7_IRQn);

    TIM7->CR1  |= TIM_CR1_CEN;
}

/* ==================== TIM7 中断服务函数（1 ms 心跳） ==================== */
/*
 * 每 1 ms 触发一次，集中处理所有时序逻辑：
 *   1. 按键边沿检测（每 50 ms 采样，消抖）
 *   2. PD12 精确 1 Hz 闪烁（按键按下期间）
 *   3. LCD 秒计数器递增与刷新请求
 *   4. 10 秒倒计时（按键松开后触发背光淡出）
 *   5. 背光亮度淡出（由 pwm.c 提供的状态变量驱动）
 */
void TIM7_IRQHandler(void)
{
    if (!(TIM7->SR & TIM_SR_UIF)) return;
    TIM7->SR &= ~TIM_SR_UIF;   /* 清除中断标志 */

    ms_ticks++;

    /* ------------------------------------------------------------------ */
    /* 1. 按键边沿检测（消抖：每 50 ms 采样一次）                           */
    /* ------------------------------------------------------------------ */
    static uint32_t key_check_last = 0;

    if (ms_ticks - key_check_last >= 50)
    {
        key_check_last = ms_ticks;

        uint8_t key_now = (GPIOA->IDR & 1) != 0;

        /* 上升沿：按键按下 → 立即全亮，取消倒计时和淡出 */
        if (key_now && !key_last)
        {
            button_pressed   = 1;
            countdown_active = 0;
            fade_active      = 0;    /* 取消正在进行的淡出 */

            /* 全亮度：CCR2 = 1000（100%） */
            Set_Backlight_Duty(BACKLIGHT_FULL);
        }

        /* 下降沿：按键松开 → 启动 10 秒倒计时（亮度保持全亮）*/
        if (!key_now && key_last)
        {
            button_pressed   = 0;
            release_time     = ms_ticks;
            countdown_active = 1;

            /* PD12 松开后立即常亮 */
            led_state = 1;
            GPIOD->ODR |= (1 << 12);
        }

        key_last = key_now;
    }

    /* ------------------------------------------------------------------ */
    /* 2. PD12 精确 1 Hz 闪烁（仅按键按下期间，每 500 ms 翻转一次）        */
    /* ------------------------------------------------------------------ */
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

    /* ------------------------------------------------------------------ */
    /* 3. 秒计数器递增与 LCD 刷新请求（始终运行，与按键状态无关）           */
    /* ------------------------------------------------------------------ */
    static uint32_t counter_last = 0;

    if (ms_ticks - counter_last >= 1000)
    {
        counter_last = ms_ticks;
        counter++;
        lcd_update_pending = 1;   /* 通知主循环刷新 LCD（不在中断中执行） */
    }

    /* ------------------------------------------------------------------ */
    /* 4. 10 秒倒计时（按键松开后触发背光淡出）                             */
    /* ------------------------------------------------------------------ */
    if (countdown_active)
    {
        uint32_t elapsed = ms_ticks - release_time;

        /* 10 秒到期：关闭倒计时，启动背光淡出，清除倒计时行 */
        if (elapsed >= 10000)
        {
            countdown_active = 0;

            fade_active = 1;
            fade_last   = ms_ticks;

            LEDs_Write(0x0000);
            lcd_update_pending = 1;   /* 通知主循环清除倒计时行 */
        }
    }

    /* ------------------------------------------------------------------ */
    /* 5. 背光亮度淡出（倒计时结束后，每 FADE_INTERVAL ms 减少 FADE_STEP）  */
    /*    总时长：(1000-200)/4 × 10ms = 2000ms ≈ 2 秒                     */
    /* ------------------------------------------------------------------ */
    if (fade_active)
    {
        if (ms_ticks - fade_last >= FADE_INTERVAL)
        {
            fade_last = ms_ticks;

            if (current_duty > BACKLIGHT_DIM + FADE_STEP)
            {
                /* 每步降低亮度 */
                current_duty -= FADE_STEP;
                TIM4->CCR2    = current_duty;
            }
            else
            {
                /* 达到目标亮度，淡出结束 */
                current_duty = BACKLIGHT_DIM;
                TIM4->CCR2   = BACKLIGHT_DIM;
                fade_active  = 0;
            }
        }
    }
}

/* ==================== 主函数 ==================== */
int main(void)
{
    mcpr_SetSystemCoreClock();

    /* ---- 外设初始化 ---- */
    LEDs_InitPorts();          /* 第1/2题：LED GPIO 初始化（同时使能 GPIOD） */

    LCD_Init();                /* 第3题：LCD 控制器初始化 */
    LCD_ClearDisplay(0xFE00); /* 以背景色清屏 */

    /* ---- GPIO 时钟（GPIOA + GPIOD） ---- */
    RCC->AHB1ENR |= (1 << 0) | (1 << 3);

    /* ---- PA0：USER 按键输入（默认已为输入模式，无需额外配置） ---- */
    GPIOA->MODER &= ~(0x00000003);

    /* ---- PD12：板载 LED 输出，初始常亮 ---- */
    set_output(GPIOD, 12);
    GPIOD->ODR |= (1 << 12);
    led_state = 1;

    /* ---- 第5题：PD13 背光 PWM 初始化（TIM4_CH2，150 Hz，低亮度启动） ---- */
    Timer4_PWM_Init();

    /* ---- 第4题：启动 TIM7（1 ms 心跳，驱动所有时序逻辑） ---- */
    Timer7_Init();

    /* ---- 第6题：启动 TIM12 输入捕获频率测量 ---- */
    Timer12_Capture_Init();

    /* ---- 上电后立即显示初始内容 ---- */
    LCD_UpdateDisplay();

    /* ==================== 主循环：持续跑马灯 ==================== */
    /*
     * 跑马灯完全在主循环中运行（正向单灯 + 全亮逐个熄灭）。
     * PD12 闪烁、按键检测、倒计时、PWM 淡出均由 TIM7 中断处理。
     *
     * 每步循环前检查 lcd_update_pending，约每 100 ms 轮询一次，
     * 避免整轮跑马灯（约 3~4 秒）结束后才处理 LCD 刷新请求。
     */
    while (1)
    {
        /* 正向单灯跑马灯（LED0 → LED15） */
        for (int i = 0; i < 16; i++)
        {
            if (lcd_update_pending)
            {
                lcd_update_pending = 0;
                LCD_UpdateDisplay();
            }
            LEDs_Write(1 << i);
            u_delay(100000);
        }

        /* 全亮逐个熄灭（LED0 先灭 → LED15 最后灭） */
        for (int i = 0; i < 16; i++)
        {
            if (lcd_update_pending)
            {
                lcd_update_pending = 0;
                LCD_UpdateDisplay();
            }
            LEDs_Write(~(uint16_t)(1 << i));
            u_delay(100000);
        }
    }
}

/* ==================== LCD_UpdateDisplay ==================== */
/*
 * 刷新全部四行 LCD 显示内容：
 *
 *   第1行（y=10）："Time Ticker: n"   – 秒计数器，始终显示
 *   第2行（y=30）："countdown: N s"   – 仅在 10 秒倒计时期间显示
 *                  倒计时结束后用背景色覆盖清除
 *   第3行（y=55）："Ticks: xxxxxxx"   – 两次上升沿间的 Tick 数
 *                  信号超时（>2s）时显示 "--"
 *   第4行（y=75）："Freq:  xxxxx Hz"  – 计算所得频率（Hz）
 *                  信号超时时显示 "--"
 *
 * 背景色 0xFE00 与 LCD_ClearDisplay 保持一致，可覆盖旧数字。
 */
void LCD_UpdateDisplay(void)
{
    char buf[32];

    /* ---- 第1行：秒计数器（始终可见） ---- */
    sprintf(buf, "Time Ticker: %-6d", counter);
    LCD_WriteString(10, 10, WHITE, 0xFE00, buf);

    /* ---- 第2行：10秒倒计时（仅倒计时激活时显示） ---- */
    if (countdown_active)
    {
        uint32_t elapsed   = ms_ticks - release_time;
        uint8_t  remaining = (elapsed < 10000) ? (uint8_t)(10 - elapsed / 1000) : 0;
        sprintf(buf, "countdown: %-2d s  ", remaining);
        LCD_WriteString(10, 30, WHITE, 0xFE00, buf);
    }
    else
    {
        /* 用背景色覆盖旧文字 */
        LCD_WriteString(10, 30, 0xFE00, 0xFE00, "                  ");
    }

    /* ---- 第3、4行：频率测量结果（第6题） ---- */
    /* 超时判断：从未测量过，或自上次测量起超过 CAP_TIMEOUT_MS → 显示 "--" */
    uint8_t timeout = (!cap_valid) ||
                      ((ms_ticks - cap_last_ms) >= CAP_TIMEOUT_MS);

    if (timeout)
    {
        LCD_WriteString(10, 55, WHITE, 0xFE00, "Ticks: --        ");
        LCD_WriteString(10, 75, WHITE, 0xFE00, "Freq:  --        ");
    }
    else
    {
        sprintf(buf, "Ticks: %-9" PRIu32, cap_ticks);
        LCD_WriteString(10, 55, WHITE, 0xFE00, buf);

        sprintf(buf, "Freq:  %-7" PRIu32 " Hz", cap_freq_hz);
        LCD_WriteString(10, 75, WHITE, 0xFE00, buf);
    }
}
