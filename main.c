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
 *   - 第4题：按键检测、10秒倒计时、PD12 LED 闪烁（在主循环中处理）
 *   - TIM7 初始化（1 ms 节拍）
 *   - LCD 显示刷新（LCD_UpdateDisplay）
 *   - 主循环（跑马灯 + 定期 LCD 刷新）
 */

/*
 * 寄存器缩写速查（本文件涉及）
 * ─────────────────────────────────────────────────────────────────────────
 * RCC        Reset and Clock Control               复位与时钟控制器
 * AHB1ENR    AHB1 peripheral clock Enable Register  AHB1总线外设时钟使能寄存器
 *              GPIO 端口均挂在 AHB1 总线，使用前必须在此置位使能
 * APB1ENR    APB1 peripheral clock Enable Register  APB1总线外设时钟使能寄存器
 *              TIM7 挂在 APB1 总线，使用前必须在此置位使能
 *
 * MODER      GPIO Mode Register                    GPIO引脚模式寄存器
 *              每引脚占 2 位：00=输入  01=通用输出  10=复用功能  11=模拟
 * ODR        Output Data Register                  GPIO输出数据寄存器
 *              每 bit 对应一个引脚，写 1 → 高电平，写 0 → 低电平
 *              |= 置位单个引脚，&= ~ 清零单个引脚（不影响其他引脚）
 * IDR        Input Data Register                   GPIO输入数据寄存器
 *              只读，每 bit 反映对应引脚当前实际电平（用于读取按键）
 *
 * CR1        Timer Control Register 1              定时器控制寄存器1
 *   CEN        Counter Enable                      计数器使能位（1=启动计数）
 *   ARPE       Auto-Reload Preload Enable          自动重装载预装载使能
 *              = 1 → 对 ARR 的修改在下次更新事件后生效，避免计数跳变
 *   URS        Update Request Source               更新请求源选择
 *              = 1 → 只有计数器溢出才触发更新中断（软件 UG 不触发）
 * PSC        Prescaler Register                    预分频寄存器
 *              计数时钟 = 输入时钟 / (PSC + 1)
 * ARR        Auto-Reload Register                  自动重装载寄存器
 *              计数器从 0 数到 ARR 后溢出并触发更新中断
 * EGR        Event Generation Register             事件生成寄存器
 *   UG         Update Generation                   软件触发更新事件
 *              写 1 → 使 PSC/ARR 配置立即生效并复位计数器
 * DIER       DMA/Interrupt Enable Register         DMA与中断使能寄存器
 *   UIE        Update Interrupt Enable             更新（溢出）中断使能位
 * SR         Status Register                       状态寄存器（中断标志集合）
 *   UIF        Update Interrupt Flag               更新（溢出）中断标志
 *              硬件溢出时置位，必须软件写 0 清除，否则中断持续触发
 *
 * CCR2       Capture/Compare Register 2            TIM4 通道2比较寄存器
 *              直接写入新值即可修改 PWM 占空比（在淡出逻辑中使用）
 *
 * NVIC       Nested Vectored Interrupt Controller  嵌套向量中断控制器
 *              管理所有外设中断的优先级（0=最高）与全局使能
 * ─────────────────────────────────────────────────────────────────────────
 */

#include "stm32f4xx.h"
#include "_mcpr_stm32f407.h"
#include <stdio.h>
#include <inttypes.h>
#include "display.h"
#include "leds.h"
#include "pwm.h"
#include "freq_measure.h"

/* ==================== Aufgabe 3 测试开关 ==================== */
/* 置 1 可启用组合测试：Basic -> 等待 USER 键 -> Efficiency */
#define ENABLE_AUFGABE3_TEST 1

/* ==================== 应用级全局状态变量 ==================== */
volatile uint32_t ms_ticks        = 0;   /* TIM7 1ms 心跳计数（毫秒时间戳） */
volatile uint8_t  button_pressed  = 0;   /* 当前按键是否处于按下状态 */
volatile uint8_t  key_last        = 0;   /* 上次采样的按键状态（边沿检测用） */

volatile uint8_t  countdown_active = 0;  /* 10 秒倒计时进行中 */
volatile uint32_t release_time     = 0;  /* 按键松开时刻（ms_ticks 快照） */

volatile int      counter          = 0;  /* LCD 秒计数器（每秒递增） */

/* LCD 刷新请求标志：ISR/其他模块置位，主循环执行实际刷新
 * （避免在中断上下文中执行耗时 GPIO 操作） */
volatile uint8_t  lcd_update_pending = 0;

/* PD12 LED 闪烁与按键采样节拍控制（仅在 App_ProcessTickTasks 中使用） */
static volatile uint32_t led_toggle_last = 0;
static volatile uint8_t  led_state       = 0;
static volatile uint32_t key_check_last  = 0;
static volatile uint32_t counter_last    = 0;

/* ==================== 函数声明 ==================== */
void Timer7_Init(void);
void LCD_UpdateDisplay(void);
void App_ProcessTickTasks(void);
void App_DelayWithTasks(uint32_t total_us);

/* ==================== Timer7 初始化（1 ms 中断） ==================== */
/*
 * 配置 TIM7 产生精确 1 ms 周期中断，作为系统心跳（ms_ticks 时间基准）。
 *
 * 频率计算：
 *   TIM7 时钟 = APB1 定时器时钟 = 84 MHz
 *   PSC = 8399  →  计数时钟 = 84 MHz / (8399+1) = 10 kHz
 *   ARR = 9     →  中断频率 = 10 kHz / (9+1)    = 1 kHz → 周期 1 ms
 */
void Timer7_Init(void)
{
    /* ────────────────────────────────────────────────────────────────────
     * APB1ENR（APB1 peripheral clock Enable Register）：
     * APB1总线外设时钟使能寄存器。TIM7 挂在 APB1 总线，
     * 时钟默认关闭，必须先使能才能访问 TIM7 的寄存器。
     * ──────────────────────────────────────────────────────────────────── */
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;  /* APB1ENR：使能 TIM7 时钟 */

    /* ────────────────────────────────────────────────────────────────────
     * CR1（Control Register 1，定时器控制寄存器1）：
     * 先清零，确保定时器处于已知初始状态，后续再按需置位各控制位。
     * ──────────────────────────────────────────────────────────────────── */
    TIM7->CR1 = 0;  /* CR1：清零重置，暂停计数器 */

    /* ────────────────────────────────────────────────────────────────────
     * PSC（Prescaler，预分频寄存器）：
     *   计数时钟 = 84 MHz / (8399+1) = 10 kHz，每个 Tick = 0.1 ms
     * ARR（Auto-Reload Register，自动重装载寄存器）：
     *   计数器从 0 数到 9（共 10 个 Tick）后溢出，产生更新中断
     *   中断周期 = 10 × 0.1 ms = 1 ms → 1 kHz 中断频率
     * ──────────────────────────────────────────────────────────────────── */
    TIM7->PSC = 8399;  /* PSC：Prescaler，84MHz / 8400 = 10kHz 计数时钟 */
    TIM7->ARR = 9;     /* ARR：Auto-Reload Register，溢出周期 = 1ms     */

    /* ────────────────────────────────────────────────────────────────────
     * CR1 位域配置：
     *   ARPE（Auto-Reload Preload Enable，自动重装载预装载使能）：
     *     = 1 → ARR 修改通过预装载寄存器缓冲，下次溢出后生效（防止跳变）
     *   URS（Update Request Source，更新请求源选择）：
     *     = 1 → 只有计数器溢出（overflow）才触发更新中断
     *     = 0（默认）时 UG 位写 1 也会触发更新中断，可能造成 EGR.UG 操作后
     *     立即误触发中断，因此设为 1 过滤掉软件触发的更新事件。
     * ──────────────────────────────────────────────────────────────────── */
    TIM7->CR1 |= TIM_CR1_ARPE  /* ARPE：ARR 预装载使能 */
              |  TIM_CR1_URS;  /* URS ：只有溢出才触发更新中断（屏蔽软件UG触发）*/

    /* ────────────────────────────────────────────────────────────────────
     * EGR（Event Generation Register，事件生成寄存器）
     * UG（Update Generation，软件触发更新位）：
     *   写 1 → 立即将 PSC/ARR 装入影子寄存器并复位计数器（使配置生效）。
     *   由于 URS=1，此操作不会触发更新中断。
     * SR（Status Register，状态寄存器）写 0 清除所有标志（保险起见）。
     * ──────────────────────────────────────────────────────────────────── */
    TIM7->EGR |= TIM_EGR_UG;  /* EGR.UG：Update Generation，使 PSC/ARR 立即生效 */
    TIM7->SR   = 0;            /* SR：清除所有中断标志，防止启动时误触发          */

    /* ────────────────────────────────────────────────────────────────────
     * DIER（DMA/Interrupt Enable Register，DMA与中断使能寄存器）
     * UIE（Update Interrupt Enable，更新中断使能位）：
     *   = 1 → 计数器每次溢出时产生中断请求（触发 TIM7_IRQHandler）
     * ──────────────────────────────────────────────────────────────────── */
    TIM7->DIER |= TIM_DIER_UIE;  /* DIER.UIE：使能更新（溢出）中断 */

    /* ────────────────────────────────────────────────────────────────────
     * NVIC（Nested Vectored Interrupt Controller，嵌套向量中断控制器）：
     *   优先级 1 → 低于 TIM12（优先级 0），确保频率捕获时间戳不被 TIM7 延迟。
     * ──────────────────────────────────────────────────────────────────── */
    NVIC_SetPriority(TIM7_IRQn, 1);  /* NVIC：TIM7 中断优先级 = 1（次高）*/
    NVIC_EnableIRQ(TIM7_IRQn);       /* NVIC：在中断控制器中使能 TIM7 向量 */

    /* ────────────────────────────────────────────────────────────────────
     * CR1.CEN（Counter Enable，计数器使能位）：
     *   写 1 → 定时器正式开始计数，1ms 后第一次触发中断。
     * ──────────────────────────────────────────────────────────────────── */
    TIM7->CR1 |= TIM_CR1_CEN;  /* CR1.CEN：Counter Enable，启动 TIM7 */
}

/* ==================== TIM7 中断服务函数（1 ms 心跳） ==================== */
/*
 * 每 1 ms 由硬件自动调用一次。
 * 仅维护毫秒时间基准 ms_ticks，所有业务逻辑迁移到主循环的 App_ProcessTickTasks()，
 * 以降低中断上下文的执行时间，减少对高优先级 TIM12 捕获的干扰。
 */
void TIM7_IRQHandler(void)
{
    /* SR（Status Register，状态寄存器）
     * UIF（Update Interrupt Flag，更新中断标志）：
     *   硬件在计数器溢出时自动置位。先检查该标志确认确实是溢出中断
     *   （而非其他来源），避免错误处理。*/
    if (!(TIM7->SR & TIM_SR_UIF))  /* SR.UIF：Update Interrupt Flag，溢出中断标志 */
        return;

    TIM7->SR &= ~TIM_SR_UIF;  /* 写 0 清除 UIF（必须手动清除，否则中断持续重入）*/

    ms_ticks++;  /* 毫秒计数器递增（所有时序逻辑的时间基准）*/
}

/* ==================== 基于 ms_ticks 的时序任务 ==================== */
/*
 * 将原来在 TIM7 中断中处理的所有时序业务集中到此函数，
 * 在主循环中轮询调用，避免在 ISR 中执行耗时操作。
 *
 * 任务列表：
 *   1. 按键边沿检测（消抖：每 50 ms 采样一次）
 *   2. PD12 精确 1 Hz 闪烁（按键按下期间）
 *   3. 秒计数器递增与 LCD 刷新请求
 *   4. 10 秒倒计时（按键松开后触发）
 *   5. 背光亮度淡出（倒计时结束后缓慢降至低亮度）
 */
void App_ProcessTickTasks(void)
{
    /* ------------------------------------------------------------------ */
    /* 1. 按键边沿检测（消抖：每 50 ms 采样一次）                           */
    /* ------------------------------------------------------------------ */
    if (ms_ticks - key_check_last >= 50)
    {
        key_check_last = ms_ticks;

        /* IDR（Input Data Register，GPIO输入数据寄存器）：
         * 只读寄存器，每 bit 反映对应引脚实际电平。
         * PA0 = USER 按键，按下时被外部上拉至高电平（IDR bit0 = 1）。*/
        uint8_t key_now = (GPIOA->IDR & 1) != 0;  /* IDR：读取 PA0 当前电平 */

        /* 上升沿（0→1）：按键按下 → 立即全亮，取消倒计时和淡出 */
        if (key_now && !key_last)
        {
            button_pressed   = 1;
            countdown_active = 0;
            fade_active      = 0;  /* 取消正在进行的淡出 */

            /* Set_Backlight_Duty() 内部写 TIM4->CCR2
             * CCR2（Capture/Compare Register 2）：TIM4 通道2比较值，控制 PWM 占空比 */
            Set_Backlight_Duty(BACKLIGHT_FULL);  /* CCR2 = 1000 → 100% 全亮 */
        }

        /* 下降沿（1→0）：按键松开 → 启动 10 秒倒计时（亮度保持全亮）*/
        if (!key_now && key_last)
        {
            button_pressed   = 0;
            release_time     = ms_ticks;
            countdown_active = 1;

            /* ODR（Output Data Register，GPIO输出数据寄存器）：
             * |= 写 1 置高 PD12，点亮板载 LED（松开按键后立即常亮）*/
            led_state = 1;
            GPIOD->ODR |= (1 << 12);  /* ODR：PD12 = 1，LED 常亮 */
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

            /* ODR（Output Data Register）：
             * |= 置位（LED 亮）或 &= ~ 清位（LED 灭），只影响 PD12 这一个 bit */
            if (led_state)
                GPIOD->ODR |=  (1 << 12);  /* ODR：PD12 = 1，LED 亮 */
            else
                GPIOD->ODR &= ~(1 << 12);  /* ODR：PD12 = 0，LED 灭 */
        }
    }

    /* ------------------------------------------------------------------ */
    /* 3. 秒计数器递增与 LCD 刷新请求（始终运行，与按键状态无关）           */
    /* ------------------------------------------------------------------ */
    if (ms_ticks - counter_last >= 1000)
    {
        counter_last = ms_ticks;
        counter++;
        lcd_update_pending = 1;  /* 通知主循环刷新 LCD（不在此处直接调用，避免长时间占用）*/
    }

    /* ------------------------------------------------------------------ */
    /* 4. 10 秒倒计时（按键松开后，到期时触发背光淡出）                     */
    /* ------------------------------------------------------------------ */
    if (countdown_active)
    {
        uint32_t elapsed = ms_ticks - release_time;

        /* 10 秒到期：关闭倒计时，启动背光淡出，清空倒计时行 */
        if (elapsed >= 10000)
        {
            countdown_active = 0;

            fade_active = 1;       /* 启动淡出（fade_active 定义于 pwm.c，pwm.h 中 extern）*/
            fade_last   = ms_ticks;

            LEDs_Write(0x0000);    /* 关闭所有 LED */
            lcd_update_pending = 1;
        }
    }

    /* ------------------------------------------------------------------ */
    /* 5. 背光亮度淡出（每 FADE_INTERVAL ms 减少 FADE_STEP）                */
    /*    总时长：(1000-200)/4 × 10 ms = 2000 ms ≈ 2 秒                   */
    /* ------------------------------------------------------------------ */
    if (fade_active)
    {
        if (ms_ticks - fade_last >= FADE_INTERVAL)
        {
            fade_last = ms_ticks;

            if (current_duty > BACKLIGHT_DIM + FADE_STEP)
            {
                /* 每步降低亮度：直接写 TIM4->CCR2
                 * CCR2（Capture/Compare Register 2，通道2比较寄存器）：
                 * 减小 CCR2 → 占空比降低 → 背光变暗（预装载使能，下周期生效）*/
                current_duty -= FADE_STEP;
                TIM4->CCR2    = current_duty;  /* CCR2：写入新占空比，背光降低一步 */
            }
            else
            {
                /* 达到目标低亮度，淡出完成 */
                current_duty = BACKLIGHT_DIM;
                TIM4->CCR2   = BACKLIGHT_DIM;  /* CCR2：设置为最终低亮度值 */
                fade_active  = 0;
            }
        }
    }
}

/* ==================== App_DelayWithTasks ==================== */
/*
 * 以 1 ms 时间片切片执行阻塞延时，每片结束后调用 App_ProcessTickTasks()。
 * 相比直接调用 u_delay(total_us)，可确保时序任务（按键/LED/淡出）不会因
 * 长延时而"掉帧"（最大响应延迟从 ~3200 ms 缩短至 ~1 ms）。
 *
 * @param total_us  总延时时长（微秒）
 */
void App_DelayWithTasks(uint32_t total_us)
{
    const uint32_t slice_us = 1000;  /* 每次忙等 1 ms，然后处理一次时序任务 */

    while (total_us >= slice_us)
    {
        u_delay(slice_us);
        App_ProcessTickTasks();
        total_us -= slice_us;
    }

    if (total_us > 0)
    {
        u_delay(total_us);
        App_ProcessTickTasks();
    }
}

/* ==================== 主函数 ==================== */
int main(void)
{
    mcpr_SetSystemCoreClock();

    /* ─── 第1/2题：LED GPIO 初始化（同时使能 GPIOD 时钟） ─── */
    LEDs_InitPorts();

    /* ─── 第3题：LCD 控制器初始化 ─── */
    LCD_Init();

    /* ────────────────────────────────────────────────────────────────────
     * AHB1ENR（AHB1 peripheral clock Enable Register）：
     * 使能 GPIOA（bit0）和 GPIOD（bit3）时钟。
     * GPIOD 时钟已在 LEDs_InitPorts() 中使能，此处 |= 重复写入无害（幂等）。
     * GPIOA 用于读取 PA0（USER 按键）。
     * ──────────────────────────────────────────────────────────────────── */
    RCC->AHB1ENR |= (1 << 0)   /* AHB1ENR bit0：使能 GPIOA 时钟（PA0 按键）*/
                 |  (1 << 3);  /* AHB1ENR bit3：使能 GPIOD 时钟（PD12 LED）*/

    /* ────────────────────────────────────────────────────────────────────
     * MODER（GPIO Mode Register，GPIO模式寄存器）：
     * PA0 默认为输入模式（MODER[1:0] = 00b），显式清零以确保状态确定。
     * 输入模式下 CPU 可通过 IDR 读取引脚电平，外部按键决定 PA0 高/低。
     * ──────────────────────────────────────────────────────────────────── */
    GPIOA->MODER &= ~(0x00000003);  /* MODER：PA0 强制设为输入模式（00b）*/

    /* ─── PD12：板载 LED 输出，初始常亮 ─── */
    set_output(GPIOD, 12);          /* MODER：PD12 设为通用输出模式（01b）*/
    GPIOD->ODR |= (1 << 12);       /* ODR：PD12 = 1，LED 初始常亮         */
    led_state = 1;

    /* ─── 第5题：PD13 背光 PWM（TIM4_CH2，150 Hz，低亮度启动） ─── */
    Timer4_PWM_Init();

    /* ─── 第4题：启动 TIM7（1 ms 心跳，ms_ticks 时间基准） ─── */
    Timer7_Init();

    /* ─── 第6题：启动 TIM12 输入捕获频率测量 ─── */
    Timer12_Capture_Init();

    /* ─── 上电后立即显示初始内容 ─── */
    LCD_UpdateDisplay();

#if ENABLE_AUFGABE3_TEST
    Aufgabe3_RunCombinedTest();
#endif

    LCD_ClearDisplay(0xFE00);  /* 以背景色清屏 */

    /* ==================== 主循环：持续跑马灯 ==================== */
    /*
     * 跑马灯完全在主循环中运行（正向单灯 + 全亮逐个熄灭）。
     * 每步延时拆分为 1 ms 时间片，穿插调用 App_ProcessTickTasks()，
     * 确保按键/LED/倒计时/淡出逻辑的响应延迟 ≤ 1 ms。
     */
    while (1)
    {
        App_ProcessTickTasks();  /* 每轮循环开始先处理一次时序任务 */

        /* 正向单灯跑马灯（LED0 → LED15） */
        for (int i = 0; i < 16; i++)
        {
            App_ProcessTickTasks();
            if (lcd_update_pending)
            {
                lcd_update_pending = 0;
                LCD_UpdateDisplay();
            }
            LEDs_Write(1 << i);        /* 点亮第 i 个 LED */
            App_DelayWithTasks(100000); /* 延时 100 ms，期间持续轮询时序任务 */
        }

        /* 全亮逐个熄灭（LED0 先灭 → LED15 最后灭） */
        for (int i = 0; i < 16; i++)
        {
            App_ProcessTickTasks();
            if (lcd_update_pending)
            {
                lcd_update_pending = 0;
                LCD_UpdateDisplay();
            }
            LEDs_Write(~(uint16_t)(1 << i));  /* 熄灭第 i 个 LED，其余全亮 */
            App_DelayWithTasks(100000);
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
        /* 用背景色覆盖旧文字（与背景色相同的"空白字符串"） */
        LCD_WriteString(10, 30, 0xFE00, 0xFE00, "                  ");
    }

    /* ---- 第3、4行：频率测量结果（第6题） ---- */
    /* 超时判断：从未测量过，或自上次有效测量起超过 CAP_TIMEOUT_MS → 显示 "--" */
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
