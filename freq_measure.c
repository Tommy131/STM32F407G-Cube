/* freq_measure.c - 第6题：TIM12 输入捕获频率测量实现 */

#include "freq_measure.h"

/* ---- 跨模块依赖：ms_ticks 和 lcd_update_pending 由 main.c 定义 ---- */
extern volatile uint32_t ms_ticks;
extern volatile uint8_t  lcd_update_pending;

/* ==================== 测量结果变量（本模块定义，头文件中声明为 extern） ==================== */
volatile uint32_t cap_ticks   = 0;   /* 两次上升沿之间的 Tick 数 */
volatile uint32_t cap_freq_hz = 0;   /* 计算所得频率（Hz，整数） */
volatile uint32_t cap_last_ms = 0;   /* 最近一次测量的 ms_ticks 时间戳 */
volatile uint8_t  cap_valid   = 0;   /* 1 = 至少已有一次有效测量 */

/* ==================== ISR 内部状态（模块私有） ==================== */
static volatile uint32_t tim12_overflow   = 0;  /* 16 位溢出次数（用于扩展为虚拟 32 位） */
static volatile uint32_t tim12_last_cap32 = 0;  /* 上一次 32 位捕获值（差值计算用） */
static volatile uint8_t  tim12_first      = 1;  /* 尚未接收到第一个上升沿 */

/* ==================== Timer12_Capture_Init ==================== */
/*
 * 初始化步骤：
 *   1. 使能 GPIOB 和 TIM12 时钟
 *   2. 配置 PB14 为复用功能模式（AF9 = TIM12_CH1）
 *   3. 配置 TIM12：PSC=0（84 MHz），ARR=0xFFFF，上升沿捕获
 *   4. 使能 CC1IE（捕获中断）+ UIE（溢出中断）
 *   5. 设置 NVIC 优先级（高于 TIM7，确保时间戳精确）并启动
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
    TIM12->PSC  = 0;        /* 84 MHz / 1 = 84 MHz → 1 Tick ≈ 11.9 ns   */
    TIM12->ARR  = 0xFFFF;   /* 最大 16 位计数值，溢出周期 ≈ 780 µs       */

    /* ---- 通道1：输入捕获，无滤波，无预分频
     *      CC1S = 01b → TI1（直接输入）
     *      IC1PSC = 00b → 无预分频（每个边沿均触发）
     *      IC1F   = 0000b → 无数字滤波器                           ---- */
    TIM12->CCMR1 = (1U << TIM_CCMR1_CC1S_Pos);  /* CC1S = 01 */

    /* ---- 上升沿触发，使能捕获（CCER：CC1P=0，CC1E=1） ---- */
    TIM12->CCER  = TIM_CCER_CC1E;   /* CC1P=0 → 上升沿 */

    /* ---- 使能中断：捕获（CC1IE）+ 溢出（UIE） ---- */
    TIM12->DIER  = TIM_DIER_CC1IE | TIM_DIER_UIE;

    /* ---- NVIC：TIM12 与 TIM8_BRK 共用 TIM8_BRK_TIM12_IRQn 向量
     *      优先级 0（高于 TIM7 的优先级 1），确保捕获时间戳精确 ---- */
    NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 0);
    NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);

    /* ---- 强制更新事件，清除标志，启动定时器 ---- */
    TIM12->EGR |= TIM_EGR_UG;
    TIM12->SR   = 0;
    TIM12->CR1 |= TIM_CR1_CEN;
}

/* ==================== TIM8_BRK_TIM12_IRQHandler ==================== */
/*
 * 响应两类事件：
 *
 * 1. UIF（溢出）：16 位计数器超过 0xFFFF → 溢出计数器加一
 *
 * 2. CC1IF（捕获）：PB14 出现上升沿 → 读取时间戳，计算频率
 *
 * 竞争条件处理（UIF 与 CC1IF 同时置位）：
 *   CCR1 ≥ 0x8000：上升沿发生在溢出之前 → 溢出计数被多加，需减一修正
 *   CCR1 < 0x8000：上升沿发生在溢出之后 → 溢出计数正确，无需修正
 *
 * 频率计算（纯整数，无浮点）：
 *   cap_freq_hz = TIM12_CLOCK_HZ / delta_ticks
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
    /* 2. 捕获中断（CC1IF）：解析时间戳并计算频率                           */
    /* ------------------------------------------------------------------ */
    if (sr & TIM_SR_CC1IF)
    {
        TIM12->SR &= ~TIM_SR_CC1IF;   /* 清除标志 */

        /* 读取 CCR1（硬件会同时清除 CC1IF） */
        uint16_t raw16 = (uint16_t)TIM12->CCR1;

        /* UIF 与 CC1IF 同时置位时的竞争条件修正：
         * 若 CCR1 ≥ 0x8000，说明上升沿发生在溢出之前
         * → tim12_overflow 已被多加一次，需减一补偿 */
        uint32_t ov = tim12_overflow;
        if ((sr & TIM_SR_UIF) && (raw16 >= 0x8000U))
            ov--;   /* 修正：上升沿在溢出之前 */

        /* 合并为虚拟 32 位捕获值 */
        uint32_t cap32 = (ov << 16) | raw16;

        if (tim12_first)
        {
            /* 第一个上升沿：仅保存参考值，暂不计算频率 */
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
                cap_freq_hz = TIM12_CLOCK_HZ / delta;  /* Hz，整数运算 */
                cap_last_ms = ms_ticks;
                cap_valid   = 1;

                /* LCD 刷新限速：最多每 200 ms 请求一次，避免频繁刷新影响显示 */
                static uint32_t last_lcd_req = 0;
                if (ms_ticks - last_lcd_req >= 200)
                {
                    last_lcd_req       = ms_ticks;
                    lcd_update_pending = 1;
                }
            }
        }
    }
}
