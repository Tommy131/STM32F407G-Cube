/* 微控制器编程 - 第5题：显示屏 PWM 调光 */
/*
 * 目标：通过 TIM4_CH2 对背光（PD13）进行 PWM 调光。
 *   - 正常模式：低亮度（20% 占空比）
 *   - 按键按下：全亮（100%）持续 10 秒
 *   - 之后：缓慢恢复至低亮度（约 2 秒）
 *
 * PWM 配置：
 *   PD13 = TIM4_CH2（AF2，参见数据手册表9，第60页）
 *   TIM4 时钟 = 84 MHz（APB1 预分频=4，定时器时钟 = APB1 × 2）
 *   PSC=559，ARR=999  →  84 MHz / 560 / 1000 = 150 Hz
 */

#include "stm32f4xx.h"
#include "_mcpr_stm32f407.h"
#include <stdio.h>
#include <inttypes.h>
#include "display.h"

/* ==================== 引脚定义 ==================== */
#define LED_WR_PIN  5
#define LED_CS_PIN  7
#define LED_SEL_PIN 11

/* ==================== 第6题：频率测量（TIM12 输入捕获） ==================== */
/*
 * PB14 = TIM12_CH1（AF9，参见数据手册表9）
 * TIM12 时钟 = 84 MHz / (83+1) = 1 MHz  →  1 Tick = 1 µs
 * 频率（Hz）= 1.000.000 / 两次上升沿之间的 Tick 数
 *
 * TIM12 为 16 位定时器；溢出次数被记录并与捕获值合并为
 * 虚拟 32 位计数器 → 极低频率（<15 Hz）也可测量。
 *
 * 中断向量：TIM8_BRK_TIM12_IRQn
 *   （在 STM32F407 上，TIM12 与 TIM8_BRK 共用该向量）
 */
#define TIM12_CLOCK_HZ  1000000UL   /* 1 MHz 计数时钟 */
#define CAP_TIMEOUT_MS  2000UL      /* 无信号 → 2 秒后显示 "--" */

/* 测量数据：在 ISR 中写入，在主循环中读取 */
volatile uint32_t cap_ticks    = 0;   /* 两次上升沿之间的 Tick 数 */
volatile uint32_t cap_freq_hz  = 0;   /* 计算所得频率（Hz，整数） */
volatile uint32_t cap_last_ms  = 0;   /* 最近一次测量的 ms_ticks 时间戳 */
volatile uint8_t  cap_valid    = 0;   /* 1 = 至少已有一次有效测量 */

/* ISR 内部状态变量（仅在 TIM8_BRK_TIM12_IRQHandler 中使用） */
static volatile uint32_t tim12_overflow   = 0;  /* 16 位溢出次数 */
static volatile uint32_t tim12_last_cap32 = 0;  /* 上一次 32 位捕获值 */
static volatile uint8_t  tim12_first      = 1;  /* 尚未接收到第一个上升沿 */

/* ==================== PWM 亮度参数 ==================== */
/* PWM分辨率：ARR=999，即CCR2范围 0~1000 */
#define PWM_ARR        999

/* 正常亮度：20% 占空比 → CCR2 = 200 */
#define BACKLIGHT_DIM  200

/* 全亮度：100% 占空比 → CCR2 = 1000 */
#define BACKLIGHT_FULL 1000

/* 淡出参数：每 FADE_INTERVAL ms 减少 FADE_STEP
 * 总步数 = (1000-200)/4 = 200 步 × 10ms = 2000ms ≈ 2 秒缓慢淡出 */
#define FADE_STEP      4
#define FADE_INTERVAL  10

/* ==================== 全局状态变量 ==================== */
volatile uint32_t ms_ticks        = 0;
volatile uint8_t  button_pressed  = 0;   /* 当前按键是否按下 */
volatile uint8_t  key_last        = 0;   /* 上次按键状态（边沿检测用） */

volatile uint8_t  countdown_active = 0;  /* 10秒倒计时进行中 */
volatile uint32_t release_time     = 0;  /* 按键松开时刻 (ms) */

volatile int      counter          = 0;  /* LCD 计数器（始终递增）*/

/* LCD 刷新请求标志：ISR 置位，主循环负责实际刷新，避免在中断中执行耗时的 GPIO 操作 */
volatile uint8_t  lcd_update_pending = 0;

/* PWM 亮度控制 */
volatile uint16_t current_duty = BACKLIGHT_DIM; /* 当前 CCR2 值 */
volatile uint8_t  fade_active  = 0;             /* 淡出进行中 */
volatile uint32_t fade_last    = 0;             /* 上次淡出时刻 */

/* PD12 闪烁控制（仅在中断中使用） */
static volatile uint32_t led_toggle_last = 0;
static volatile uint8_t  led_state       = 0;

/* ==================== 函数声明 ==================== */
void set_output(GPIO_TypeDef *port, int pin);
void LEDs_InitPorts(void);
void LCD_Output16BitWord(uint16_t data);
void LEDs_Write(uint16_t data);
void Timer7_Init(void);
void Timer4_PWM_Init(void);
void Set_Backlight_Duty(uint16_t duty);
void Timer12_Capture_Init(void);
void LCD_UpdateDisplay(void);
void u_delay(uint32_t usec);

/* ==================== Timer4 PWM 初始化（PD13 = TIM4_CH2，150 Hz） ==================== */
/*
 * PD13 切换为复用功能模式（AF2 = TIM4）。
 * TIM4 通道 2 产生 150 Hz 的 PWM 信号。
 *
 * 频率计算：
 *   TIM4 时钟  = 84 MHz
 *   PSC        = 559  →  计数时钟 = 84 MHz / (559+1) = 150 kHz
 *   ARR        = 999  →  PWM 频率 = 150 kHz / (999+1) = 150 Hz
 */
void Timer4_PWM_Init(void)
{
    /* 使能 GPIOD 和 TIM4 时钟 */
    // RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; // 已在 LEDs_InitPorts() 中使能，无需重复
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    /* PD13 → 复用功能模式（MODER[27:26] = 10b） */
    GPIOD->MODER &= ~(3U << (2 * 13)); /* U：无符号字面量后缀，避免符号位移位溢出 */
    GPIOD->MODER |=  (2U << (2 * 13));

    /* 设置复用功能：AFRH（AFR[1]）中 Pin13 对应的位段 [23:20]
     * = (13-8)*4 = 20，写入 AF2（0010）*/
    GPIOD->AFR[1] &= ~(0xFU << ((13 - 8) * 4));
    GPIOD->AFR[1] |=  (2U   << ((13 - 8) * 4));

    /* TIM4 预分频与计数周期 */
    TIM4->PSC = 559;       /* 84 MHz / 560 = 150 kHz 计数时钟 */
    TIM4->ARR = PWM_ARR;   /* ARR = 999 → 150 Hz PWM */

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

/* ==================== 设置 PWM 亮度 ==================== */
/* 直接写入 CCR2（可在 ISR 中安全调用）。 */
void Set_Backlight_Duty(uint16_t duty)
{
    if (duty > BACKLIGHT_FULL) duty = BACKLIGHT_FULL;
    current_duty   = duty;
    TIM4->CCR2     = duty;
}

/* ==================== Timer7 初始化（1ms 中断） ==================== */
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

/* ==================== Timer7 中断服务函数（1 ms 节拍） ==================== */
/*
 * 所有时序逻辑均在此处理：
 *   1. 按键边沿检测（每 50 ms 采样）
 *   2. PD12 精确 1 Hz 闪烁
 *   3. 独立计时器：每秒刷新 LCD（始终运行）
 *   4. 10 秒倒计时（仅控制背光淡出时机）
 *   5. 背光亮度淡出（倒计时结束后缓慢降至低亮度）
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

            /* 全亮度：CCR2 = 1000 (100%) */
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
    /* 2. PD12 精确 1 Hz 闪烁（仅按键按下期间）                            */
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
    /* 3. 独立计时器：每秒递增计数器并刷新 LCD（始终运行，与按键无关）      */
    /* ------------------------------------------------------------------ */
    static uint32_t counter_last = 0;

    if (ms_ticks - counter_last >= 1000)
    {
        counter_last = ms_ticks;
        counter++;
        lcd_update_pending = 1;   /* 通知主循环刷新 LCD，不在中断中执行 */
    }

    /* ------------------------------------------------------------------ */
    /* 4. 10 秒倒计时（按键松开后，仅控制背光淡出时机）                     */
    /* ------------------------------------------------------------------ */
    if (countdown_active)
    {
        uint32_t elapsed = ms_ticks - release_time;

        /* 10 秒到期：关闭倒计时，开始背光淡出，立即清除倒计时行 */
        if (elapsed >= 10000)
        {
            countdown_active = 0;

            fade_active = 1;
            fade_last   = ms_ticks;

            LEDs_Write(0x0000);
            lcd_update_pending = 1;   /* 通知主循环清除 countdown 行 */
        }
    }

    /* ------------------------------------------------------------------ */
    /* 5. 背光淡出（倒计时结束后缓慢降低亮度）                              */
    /*    每 FADE_INTERVAL(10) ms 将 CCR2 减少 FADE_STEP(4)                */
    /*    总时间：(1000-200)/4 × 10ms = 2000ms ≈ 2 秒                     */
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

/* ==================== 第6题：TIM12 输入捕获初始化 ==================== */
/*
 * PB14 配置为 TIM12_CH1（AF9）。
 *
 * 计数时钟计算：
 *   TIM12 时钟 = APB1 定时器时钟 = 84 MHz
 *   PSC = 83  →  计数时钟 = 84 MHz / (83+1) = 1 MHz  →  1 Tick = 1 µs
 *   ARR = 0xFFFF（65535）→ 最大 16 位计数周期
 *
 * 中断：
 *   CC1IE  = 捕获中断（PB14 上升沿）
 *   UIE    = 更新中断（16 位计数器溢出）
 *   向量   = TIM8_BRK_TIM12_IRQn（与 STM32F407 上 TIM8_BRK 共用）
 */
void Timer12_Capture_Init(void)
{
    /* ---- 使能 GPIOB 时钟和 TIM12 时钟 ---- */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;

    /* ---- PB14：设置为复用功能模式（MODER[29:28] = 10b） ---- */
    GPIOB->MODER &= ~(3U << (2 * 14));
    GPIOB->MODER |=  (2U << (2 * 14));

    /* ---- AF9（TIM12_CH1）写入 AFRH（AFR[1]）中 Pin14 对应位段
     *      位 [27:24] = (14-8)*4 = 24，写入 1001（9） ---- */
    GPIOB->AFR[1] &= ~(0xFU << ((14 - 8) * 4));
    GPIOB->AFR[1] |=  (9U   << ((14 - 8) * 4));

    /* ---- TIM12 基本配置 ---- */
    TIM12->CR1  = 0;
    TIM12->PSC  = 83;       /* 84 MHz / 84 = 1 MHz → 1 Tick = 1 µs        */
    TIM12->ARR  = 0xFFFF;   /* 最大 16 位计数值                             */

    /* ---- 通道1：输入捕获，无滤波，无预分频
     *      CC1S = 01b → TI1（直接输入）
     *      IC1PSC = 00b → 无预分频（每个边沿均触发）
     *      IC1F   = 0000b → 无数字滤波器                           ---- */
    TIM12->CCMR1 = (1U << TIM_CCMR1_CC1S_Pos);  /* CC1S = 01 */

    /* ---- 上升沿触发，使能捕获（CCER：CC1P=0，CC1E=1） ---- */
    TIM12->CCER  = TIM_CCER_CC1E;   /* CC1P=0 → 上升沿                      */

    /* ---- 使能中断：捕获（CC1IE）+ 溢出（UIE） ---- */
    TIM12->DIER  = TIM_DIER_CC1IE | TIM_DIER_UIE;

    /* ---- NVIC：TIM12 与 TIM8_BRK 共用 TIM8_BRK_TIM12_IRQn 向量
     *      优先级高于 TIM7（0 < 1），确保捕获时间戳精确 ---- */
    NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 0);
    NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);

    /* ---- 强制更新事件，清除标志，启动定时器 ---- */
    TIM12->EGR |= TIM_EGR_UG;
    TIM12->SR   = 0;
    TIM12->CR1 |= TIM_CR1_CEN;
}

/* ==================== 第6题：TIM12 中断服务函数 ==================== */
/*
 * 该处理函数响应两类事件：
 *   1. UIF（更新/溢出）：TIM12 计数器超过 0xFFFF → 溢出计数器加一
 *   2. CC1IF（捕获）：  PB14 出现上升沿 → 读取时间戳
 *
 * 竞争条件处理（两个标志同时置位）：
 *   CCR1 < 0x8000 → 上升沿发生在溢出之后
 *     → 当前溢出计数已正确递增，无需修正
 *   CCR1 >= 0x8000 → 上升沿发生在溢出之前
 *     → 溢出计数被错误递增 → 将溢出计数减一修正
 *
 * 频率计算（整数运算，无浮点）：
 *   cap_freq_hz = TIM12_CLOCK_HZ / cap_ticks
 *               = 1.000.000 / 两次上升沿间的 Tick 差值
 */
void TIM8_BRK_TIM12_IRQHandler(void)
{
    uint32_t sr = TIM12->SR;

    /* ------------------------------------------------------------------ */
    /* 1. 溢出中断（UIF）：溢出计数器加一                                   */
    /* ------------------------------------------------------------------ */
    if (sr & TIM_SR_UIF)
    {
        TIM12->SR &= ~TIM_SR_UIF;   /* 清除标志 */
        tim12_overflow++;
    }

    /* ------------------------------------------------------------------ */
    /* 2. 捕获中断（CC1IF）：解析时间戳                                     */
    /* ------------------------------------------------------------------ */
    if (sr & TIM_SR_CC1IF)
    {
        TIM12->SR &= ~TIM_SR_CC1IF;   /* 清除标志 */

        /* 读取 CCR1（硬件同时清除 CC1IF） */
        uint16_t raw16 = (uint16_t)TIM12->CCR1;

        /* UIF 与 CC1IF 同时置位时的溢出修正：
         * 若 CCR1 较大（≥ 0x8000），说明上升沿发生在溢出之前
         * → 溢出计数已被多加一次，需要减一补偿 */
        uint32_t ov = tim12_overflow;
        if ((sr & TIM_SR_UIF) && (raw16 >= 0x8000U))
            ov--;   /* 修正：上升沿发生在溢出之前 */

        /* 合并为 32 位捕获值 */
        uint32_t cap32 = (ov << 16) | raw16;

        if (tim12_first)
        {
            /* 第一个上升沿：仅保存参考值，暂不计算 */
            tim12_last_cap32 = cap32;
            tim12_first      = 0;
        }
        else
        {
            /* 差值 = 两次上升沿之间的 Tick 数 */
            uint32_t delta = cap32 - tim12_last_cap32;
            tim12_last_cap32 = cap32;

            if (delta > 0)
            {
                cap_ticks   = delta;
                cap_freq_hz = TIM12_CLOCK_HZ / delta;  /* Hz，整数 */
                cap_last_ms = ms_ticks;
                cap_valid   = 1;

                /* 最多每 200 ms 请求一次 LCD 刷新 */
                static uint32_t last_lcd_req = 0;
                if (ms_ticks - last_lcd_req >= 200)
                {
                    last_lcd_req      = ms_ticks;
                    lcd_update_pending = 1;
                }
            }
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

    /* ---- GPIO 时钟（GPIOA + GPIOD） ---- */
    RCC->AHB1ENR |= (1 << 0) | (1 << 3);

    /* ---- PA0：USER 键输入（默认输入模式，无需额外配置） ---- */
    GPIOA->MODER &= ~(0x00000003);

    /* ---- PD12：板载 LED 输出，初始常亮 ---- */
    set_output(GPIOD, 12);
    GPIOD->ODR |= (1 << 12);
    led_state = 1;

    /* ---- PD13：背光 PWM 初始化（TIM4_CH2，150 Hz，低亮度启动） ---- */
    Timer4_PWM_Init();

    /* ---- 启动 Timer7（1 ms 节拍） ---- */
    Timer7_Init();

    /* ---- 第6题：启动 TIM12 输入捕获频率测量 ---- */
    Timer12_Capture_Init();

    /* ---- 上电后立即显示初始内容 ---- */
    LCD_UpdateDisplay();

    /* ==================== 主循环：持续跑马灯 ==================== */
    /*
     * 跑马灯完全在主循环中运行。
     * PD12 闪烁、按键检测、倒计时、PWM 淡出均由 TIM7 中断处理。
     */
    while (1)
    {
        /* 正向单灯跑马灯
         * 每步前检查 LCD 刷新标志（约每 100 ms 一次），
         * 避免 lcd_update_pending 在整轮跑马灯（约 3-4 秒）结束后才处理。 */
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

        /* 全亮逐个熄灭 */
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

/* ==================== 辅助函数 ==================== */

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

/*
 * LCD_UpdateDisplay - 更新全部四行显示内容。
 *
 * 第1行（y=10）："Time Ticker: n"    – 始终显示
 * 第2行（y=30）："countdown: N s"    – 仅在 10 秒倒计时期间显示
 *                （倒计时结束时清除该行）
 * 第3行（y=55）："Ticks: xxxxxxx"    – 两次上升沿间的微秒数
 *                超时时显示 "--"
 * 第4行（y=75）："Freq:  xxxxx Hz"   – 计算所得频率（Hz）
 *                超时时显示 "--"
 *
 * 背景色 0xFE00 与 LCD_ClearDisplay 一致，用于覆盖旧数字。
 */
void LCD_UpdateDisplay(void)
{
    char buf[32];

    /* ---- 第1行：秒计数器（始终可见） ---- */
    sprintf(buf, "Time Ticker: %-6d", counter);
    LCD_WriteString(10, 10, WHITE, 0xFE00, buf);

    /* ---- 第2行：倒计时（仅在倒计时激活时显示） ---- */
    if (countdown_active)
    {
        uint32_t elapsed   = ms_ticks - release_time;
        uint8_t  remaining = (elapsed < 10000) ? (uint8_t)(10 - elapsed / 1000) : 0;
        sprintf(buf, "countdown: %-2d s  ", remaining);
        LCD_WriteString(10, 30, WHITE, 0xFE00, buf);
    }
    else
    {
        /* 清空该行（用背景色覆盖旧文字） */
        LCD_WriteString(10, 30, 0xFE00, 0xFE00, "                  ");
    }

    /* ---- 第3、4行：频率测量（第6题） ---- */
    /* 超时判断：自上次有效测量起超过 CAP_TIMEOUT_MS → 显示 "--" */
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
