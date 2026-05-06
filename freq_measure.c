/* freq_measure.c - 第6题：TIM12 输入捕获频率测量实现 */

/*
 * 寄存器缩写速查（本文件涉及）
 * ─────────────────────────────────────────────────────────────────────────
 * RCC        Reset and Clock Control               复位与时钟控制器
 * AHB1ENR    AHB1 peripheral clock Enable Register  AHB1总线外设时钟使能寄存器
 * APB1ENR    APB1 peripheral clock Enable Register  APB1总线外设时钟使能寄存器
 *
 * MODER      GPIO Mode Register                    GPIO引脚模式寄存器
 *              00=输入  01=通用输出  10=复用功能  11=模拟
 * AFR[1]     Alternate Function Register High      复用功能寄存器高位（Pin8~15）
 *              每4位对应一个引脚，写入 AF0~AF15 选择具体外设
 *
 * CR1        Timer Control Register 1              定时器控制寄存器1
 *   CEN        Counter Enable                      计数器使能位（1=启动）
 * PSC        Prescaler Register                    预分频寄存器
 *              计数时钟 = 输入时钟 / (PSC + 1)
 * ARR        Auto-Reload Register                  自动重装载寄存器
 *              计数器溢出值（达到 ARR+1 后归零并触发更新事件）
 * CCMR1      Capture/Compare Mode Register 1       捕获/比较模式寄存器1（通道1/2）
 *   CC1S       Capture/Compare 1 channel Selection 通道1方向选择
 *              00=输出  01=输入(TI1直接)  10=输入(TI2交叉)  11=输入(TRC)
 *   IC1PSC     Input Capture 1 PreSCaler           输入捕获1预分频
 *              00=不分频(每沿捕获) 01=2分频 10=4分频 11=8分频
 *   IC1F       Input Capture 1 digital Filter      输入捕获1数字滤波器
 *              0000=无滤波，其余值越大滤波越强（抑制毛刺）
 * CCER       Capture/Compare Enable Register       捕获/比较使能寄存器
 *   CC1E       Capture/Compare 1 output Enable     通道1捕获使能位（1=使能）
 *   CC1P       Capture/Compare 1 output Polarity   通道1触发极性
 *              0=上升沿触发  1=下降沿触发（CC1P+CC1NP=11 → 双边沿）
 * DIER       DMA/Interrupt Enable Register         DMA与中断使能寄存器
 *   UIE        Update Interrupt Enable             更新（溢出）中断使能位
 *   CC1IE      Capture/Compare 1 Interrupt Enable  通道1捕获中断使能位
 * SR         Status Register                       状态寄存器（中断标志位集合）
 *   UIF        Update Interrupt Flag               更新（溢出）中断标志（写0清除）
 *   CC1IF      Capture/Compare 1 Interrupt Flag    通道1捕获中断标志（写0清除）
 * CCR1       Capture/Compare Register 1            通道1捕获/比较寄存器
 *              发生捕获时，硬件自动将当前计数值锁存到此寄存器
 * EGR        Event Generation Register             事件生成寄存器
 *   UG         Update Generation                   软件触发更新事件
 *              写1后 PSC/ARR 立即生效，同时置位 UIF（需手动清除）
 *
 * NVIC       Nested Vectored Interrupt Controller  嵌套向量中断控制器
 * ─────────────────────────────────────────────────────────────────────────
 */

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
    /* ────────────────────────────────────────────────────────────────────
     * 1. 时钟使能
     *    RCC->AHB1ENR：AHB1总线外设时钟使能寄存器，控制各 GPIO 端口的供电
     *    RCC->APB1ENR：APB1总线外设时钟使能寄存器，控制定时器、UART等外设供电
     *    STM32 上电后所有外设时钟默认关闭（省电），必须在使用前手动开启。
     * ──────────────────────────────────────────────────────────────────── */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  /* 使能 GPIOB 时钟（PB14 所在端口） */
    RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;  /* 使能 TIM12 时钟 */

    /* ────────────────────────────────────────────────────────────────────
     * 2. PB14 引脚模式配置
     *    MODER（GPIO Mode Register，GPIO模式寄存器）：
     *      每个引脚占 2 位，Pin14 对应 Bit[29:28]。
     *      写入 10b = 复用功能模式（Alternate Function），
     *      此模式下引脚的输入/输出由片内外设（TIM12）直接驱动，而非 CPU 控制。
     * ──────────────────────────────────────────────────────────────────── */
    GPIOB->MODER &= ~(3U << (2 * 14));  /* 先清除 Bit[29:28]（避免部分写入错误） */
    GPIOB->MODER |=  (2U << (2 * 14));  /* 写入 10b → 复用功能模式 */

    /* ────────────────────────────────────────────────────────────────────
     *    AFR[1]（Alternate Function Register High，复用功能寄存器高位）：
     *      控制 Pin8~Pin15 各引脚所连接的外设，每引脚占 4 位（AF0~AF15）。
     *      Pin14 在 AFR[1] 中的偏移 = (14-8)*4 = 24 位，写入 9 = AF9。
     *      查 STM32F407 数据手册 Table 9：AF9 对应 TIM12_CH1（PB14）。
     * ──────────────────────────────────────────────────────────────────── */
    GPIOB->AFR[1] &= ~(0xFU << ((14 - 8) * 4));  /* 清除 Pin14 的 AF 选择位 */
    GPIOB->AFR[1] |=  (9U   << ((14 - 8) * 4));  /* 写入 AF9 = TIM12_CH1 */

    /* ────────────────────────────────────────────────────────────────────
     * 3. TIM12 基本参数配置
     *    CR1（Control Register 1，控制寄存器1）：先清零，后续按需设位。
     *    PSC（Prescaler，预分频寄存器）：
     *      计数时钟 = TIM12输入时钟 / (PSC + 1) = 84 MHz / 1 = 84 MHz
     *      每个 Tick ≈ 11.9 ns，分辨率极高（与 PSC=83 相比误差缩小84倍）。
     *    ARR（Auto-Reload Register，自动重装载寄存器）：
     *      TIM12 为 16 位定时器，ARR 最大为 0xFFFF = 65535。
     *      溢出周期 = 65536 / 84,000,000 ≈ 780 µs，即约每 780 µs 溢出一次。
     * ──────────────────────────────────────────────────────────────────── */
    TIM12->CR1 = 0;         /* CR1：Control Register 1，定时器总控制（清零重置） */
    TIM12->PSC = 0;         /* PSC：Prescaler，预分频 = 0 → 84 MHz 计数时钟    */
    TIM12->ARR = 0xFFFF;    /* ARR：Auto-Reload Register，最大16位值 = 65535   */

    /* ────────────────────────────────────────────────────────────────────
     * 4. CCMR1 - 通道1配置为输入捕获模式
     *    CCMR1（Capture/Compare Mode Register 1，捕获/比较模式寄存器1）
     *    控制通道1和通道2的工作模式（本函数仅用通道1）：
     *
     *    CC1S（Capture/Compare 1 channel Selection，通道1方向选择）：
     *      位[1:0] = 01b → 通道1为输入模式，连接到 TI1（即 PB14 引脚信号）
     *      00=输出（PWM等）  01=输入直连TI1  10=输入交叉TI2  11=输入TRC触发
     *
     *    IC1PSC（Input Capture 1 PreSCaler，输入捕获1预分频）：
     *      默认 00b → 不分频，PB14 每次上升沿都触发一次捕获。
     *
     *    IC1F（Input Capture 1 digital Filter，输入捕获1数字滤波器）：
     *      默认 0000b → 无滤波，信号直接进入捕获单元（适合干净的方波）。
     * ──────────────────────────────────────────────────────────────────── */
    TIM12->CCMR1 = (1U << TIM_CCMR1_CC1S_Pos);
    /*             ↑ CC1S=01：通道1 = 输入模式，连接 TI1（PB14）
     *               IC1PSC=00（默认）：每个上升沿均捕获
     *               IC1F=0000（默认）：不启用数字滤波器                      */

    /* ────────────────────────────────────────────────────────────────────
     * 5. CCER - 捕获极性与使能
     *    CCER（Capture/Compare Enable Register，捕获/比较使能寄存器）
     *    每个通道有 CC1E / CC1P / CC1NP 三位：
     *
     *    CC1E（Capture/Compare 1 output Enable，通道1捕获使能）：
     *      = 1 → 使能通道1捕获功能
     *
     *    CC1P（Capture/Compare 1 output Polarity，通道1极性）：
     *      = 0（保持默认）→ 上升沿触发捕获
     *      若需下降沿：CC1P=1；若需双边沿：CC1P=1 且 CC1NP=1
     * ──────────────────────────────────────────────────────────────────── */
    TIM12->CCER = TIM_CCER_CC1E;
    /*            ↑ CC1E=1：使能通道1捕获
     *              CC1P=0（默认）：上升沿触发                               */

    /* ────────────────────────────────────────────────────────────────────
     * 6. DIER - 中断使能
     *    DIER（DMA/Interrupt Enable Register，DMA与中断使能寄存器）
     *
     *    CC1IE（Capture/Compare 1 Interrupt Enable，通道1捕获中断使能）：
     *      = 1 → PB14 出现上升沿时，硬件置位 SR.CC1IF 并触发中断
     *
     *    UIE（Update Interrupt Enable，更新中断使能）：
     *      = 1 → 计数器溢出时（CNT 从 0xFFFF 归零），硬件置位 SR.UIF 并触发中断
     *      必须使能，否则无法实现 32 位计数器扩展（溢出计数器 tim12_overflow）
     * ──────────────────────────────────────────────────────────────────── */
    TIM12->DIER = TIM_DIER_CC1IE  /* CC1IE：通道1捕获中断使能 */
                | TIM_DIER_UIE;   /* UIE  ：溢出更新中断使能  */

    /* ────────────────────────────────────────────────────────────────────
     * 7. NVIC 中断控制器配置
     *    NVIC（Nested Vectored Interrupt Controller，嵌套向量中断控制器）
     *    负责管理所有外设中断的优先级和使能。
     *
     *    注意：STM32F407 上 TIM12 无独立向量，与 TIM8 刹车事件共用
     *          TIM8_BRK_TIM12_IRQn 向量。
     *
     *    优先级设为 0（最高），高于 TIM7 的优先级 1，确保捕获时间戳精确，
     *    不被 TIM7 中断打断（否则可能产生 ~1 µs 的时间戳误差）。
     * ──────────────────────────────────────────────────────────────────── */
    NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 0);  /* 优先级 0 = 最高 */
    NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);       /* 在 NVIC 中使能该中断向量 */

    /* ────────────────────────────────────────────────────────────────────
     * 8. 软件触发更新事件，清除标志，启动计数器
     *    EGR（Event Generation Register，事件生成寄存器）
     *    UG（Update Generation，软件更新触发位）：
     *      写 1 → 立即将 PSC/ARR 的值装入影子寄存器（使配置生效），
     *      同时将计数器复位为 0，并置位 SR.UIF（需手动清除，防止启动时误触发）。
     *
     *    SR（Status Register，状态寄存器）写 0 清除所有中断标志。
     *
     *    CR1.CEN（Counter Enable，计数器使能）：
     *      写 1 → 正式启动定时器，开始计数。
     * ──────────────────────────────────────────────────────────────────── */
    TIM12->EGR  |= TIM_EGR_UG;   /* EGR.UG：Update Generation，触发更新使配置生效 */
    TIM12->SR    = 0;             /* SR：Status Register，清除 UG 产生的 UIF 标志   */
    TIM12->CR1  |= TIM_CR1_CEN;  /* CR1.CEN：Counter Enable，启动计数器             */
}

/* ==================== TIM8_BRK_TIM12_IRQHandler ==================== */
/*
 * 响应两类事件（通过读取 SR 状态寄存器区分）：
 *
 * ① UIF（Update Interrupt Flag，更新中断标志）：
 *      计数器从 0xFFFF 溢出归零时由硬件置位。
 *      → 溢出计数器 tim12_overflow 加一（用于虚拟 32 位扩展）。
 *
 * ② CC1IF（Capture/Compare 1 Interrupt Flag，通道1捕获中断标志）：
 *      PB14 出现上升沿时由硬件置位，同时将计数器当前值锁存到 CCR1。
 *      → 读取 CCR1，合并溢出计数，计算两次上升沿之间的 Tick 差值，求频率。
 *
 * 竞争条件处理（UIF 与 CC1IF 同时置位）：
 *   CCR1 ≥ 0x8000：上升沿发生在溢出之前 → 溢出计数已被多加，需减一修正
 *   CCR1 < 0x8000：上升沿发生在溢出之后 → 溢出计数正确，无需修正
 *
 * 频率计算（纯整数，无浮点）：
 *   cap_freq_hz = TIM12_CLOCK_HZ / delta_ticks
 */
void TIM8_BRK_TIM12_IRQHandler(void)
{
    /* 一次性读取 SR（Status Register，状态寄存器），保存快照。
     * 不直接多次读 TIM12->SR，防止第二次读时硬件已清除某些标志位（竞态）。*/
    uint32_t sr = TIM12->SR;

    /* ------------------------------------------------------------------ */
    /* ① 溢出中断：SR.UIF（Update Interrupt Flag，更新中断标志）           */
    /*   计数器从 ARR(0xFFFF) 归零时由硬件自动置位                          */
    /* ------------------------------------------------------------------ */
    if (sr & TIM_SR_UIF)                   /* TIM_SR_UIF：UIF 位掩码 */
    {
        TIM12->SR &= ~TIM_SR_UIF;          /* 写 0 清除 UIF 标志（必须手动清除，否则持续中断） */
        tim12_overflow++;                  /* 溢出计数 +1，用于虚拟 32 位扩展 */
    }

    /* ------------------------------------------------------------------ */
    /* ② 捕获中断：SR.CC1IF（Capture/Compare 1 Interrupt Flag）            */
    /*   PB14 出现上升沿时由硬件置位，同时将 CNT 锁存到 CCR1               */
    /* ------------------------------------------------------------------ */
    if (sr & TIM_SR_CC1IF)                 /* TIM_SR_CC1IF：CC1IF 位掩码 */
    {
        TIM12->SR &= ~TIM_SR_CC1IF;        /* 写 0 清除 CC1IF 标志 */

        /* CCR1（Capture/Compare Register 1，通道1捕获寄存器）：
         * 读取时硬件已将捕获发生瞬间的计数器值锁存于此。
         * TIM12 为 16 位定时器，只取低 16 位。                             */
        uint16_t raw16 = (uint16_t)TIM12->CCR1;

        /* 竞争条件修正：
         * 若 UIF 和 CC1IF 同时被置位，说明溢出与捕获几乎同时发生。
         * 此时 tim12_overflow 已递增，但需判断上升沿究竟在溢出前还是后：
         *   raw16 ≥ 0x8000（计数器值较大）→ 上升沿在溢出之前发生
         *     → overflow 被多加了一次，需减一修正                          */
        uint32_t ov = tim12_overflow;
        if ((sr & TIM_SR_UIF) && (raw16 >= 0x8000U))
            ov--;   /* 修正：上升沿在溢出之前 */

        /* 将 16 位 overflow 计数和 16 位 raw16 合并为虚拟 32 位时间戳 */
        uint32_t cap32 = (ov << 16) | raw16;

        if (tim12_first)
        {
            /* 第一个上升沿：仅记录参考时间戳，差值计算从第二个沿开始 */
            tim12_last_cap32 = cap32;
            tim12_first      = 0;
        }
        else
        {
            /* delta = 相邻两次上升沿之间经过的 Tick 数 */
            uint32_t delta = cap32 - tim12_last_cap32;
            tim12_last_cap32 = cap32;

            if (delta > 0)
            {
                cap_ticks   = delta;
                cap_freq_hz = TIM12_CLOCK_HZ / delta;  /* Hz = 84MHz / Ticks，纯整数 */
                cap_last_ms = ms_ticks;
                cap_valid   = 1;

                /* LCD 刷新限速：最多每 200 ms 请求一次，避免高频信号造成显示抖动 */
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
