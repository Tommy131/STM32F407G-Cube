/* pwm.c - 第5题：TIM4 PWM 背光调光实现 */

/*
 * 寄存器缩写速查（本文件涉及）
 * ─────────────────────────────────────────────────────────────────────────
 * RCC        Reset and Clock Control               复位与时钟控制器
 * APB1ENR    APB1 peripheral clock Enable Register  APB1总线外设时钟使能寄存器
 *              TIM4 挂在 APB1 总线上，使用前必须在此寄存器中使能时钟
 *
 * MODER      GPIO Mode Register                    GPIO引脚模式寄存器
 *              10b = 复用功能模式（AF），引脚由外设（TIM4）驱动
 * AFR[1]     Alternate Function Register High      复用功能寄存器高位（Pin8~15）
 *              Pin13 对应偏移 (13-8)*4 = 20 位，写入 AF2 → TIM4_CH2
 *
 * PSC        Prescaler Register                    预分频寄存器
 *              计数时钟 = TIM4输入时钟 / (PSC + 1)
 * ARR        Auto-Reload Register                  自动重装载寄存器
 *              PWM 周期 = (ARR + 1) 个计数时钟；分辨率 = ARR + 1 级
 * CCMR1      Capture/Compare Mode Register 1       捕获/比较模式寄存器1（通道1/2）
 *   OC2M       Output Compare 2 Mode               通道2输出比较模式
 *              110b = PWM 模式1：CNT < CCR2 输出高电平，否则低电平
 *   OC2PE      Output Compare 2 Preload Enable     通道2预装载使能
 *              = 1 → 写入 CCR2 的值在下次更新事件后才生效（避免毛刺）
 * CCR2       Capture/Compare Register 2            通道2捕获/比较寄存器
 *              PWM 占空比 = CCR2 / (ARR + 1)，范围 0（全灭）~ ARR+1（全亮）
 * CCER       Capture/Compare Enable Register       捕获/比较使能寄存器
 *   CC2E       Capture/Compare 2 output Enable     通道2输出使能位
 *              = 1 → 使能通道2的 PWM 输出到引脚
 * CR1        Timer Control Register 1              定时器控制寄存器1
 *   ARPE       Auto-Reload Preload Enable          自动重装载预装载使能
 *              = 1 → ARR 的新值在更新事件后才生效（避免修改时的计数跳变）
 *   CEN        Counter Enable                      计数器使能位（1=启动计数）
 * EGR        Event Generation Register             事件生成寄存器
 *   UG         Update Generation                   软件触发更新事件
 *              写 1 → 使 PSC/ARR 立即装入，并置位 SR.UIF（需手动清除）
 * SR         Status Register                       状态寄存器（中断标志集合）
 *              写 0 清除标志，本模块写 SR=0 清除 EGR.UG 触发的 UIF 标志
 * ─────────────────────────────────────────────────────────────────────────
 */

#include "pwm.h"

/* ==================== 模块私有全局变量 ==================== */
/* 这些变量在 TIM7_IRQHandler（main.c）中也会访问，因此通过 pwm.h 导出为 extern。*/
volatile uint16_t current_duty = BACKLIGHT_DIM;  /* 当前占空比（CCR2 值，0~1000） */
volatile uint8_t  fade_active  = 0;              /* 淡出进行中标志 */
volatile uint32_t fade_last    = 0;              /* 上次淡出步进时刻（ms_ticks） */

/* ==================== Timer4_PWM_Init ==================== */
/*
 * 配置 TIM4 通道2 在 PD13 上输出 150 Hz PWM 信号。
 *
 * 频率计算：
 *   TIM4 时钟 = 84 MHz（APB1 × 2，因 APB1 预分频 = 4，定时器时钟 ×2 补偿）
 *   PSC = 559  →  计数时钟 = 84 MHz / (559+1) = 150 kHz
 *   ARR = 999  →  PWM 频率  = 150 kHz / (999+1) = 150 Hz
 *   分辨率 = 1000 级（CCR2 范围 0~1000，对应 0%~100% 占空比）
 */
void Timer4_PWM_Init(void)
{
    /* ────────────────────────────────────────────────────────────────────
     * 1. 时钟使能
     *    APB1ENR（APB1 peripheral clock Enable Register）：
     *    TIM4 挂在 APB1 总线上。GPIOD 时钟已由 LEDs_InitPorts() 使能，无需重复。
     * ──────────────────────────────────────────────────────────────────── */
    // RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; /* 已在 LEDs_InitPorts() 中使能，无需重复 */
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;   /* APB1ENR：使能 TIM4 时钟 */

    /* ────────────────────────────────────────────────────────────────────
     * 2. PD13 引脚配置为复用功能模式
     *    MODER（GPIO Mode Register，GPIO模式寄存器）：
     *      Pin13 对应 Bit[27:26]，写入 10b = 复用功能模式（AF）。
     *      复用功能模式下，引脚的高/低电平由 TIM4 PWM 逻辑自动驱动，
     *      CPU 不再通过 ODR 控制该引脚。
     * ──────────────────────────────────────────────────────────────────── */
    GPIOD->MODER &= ~(3U << (2 * 13));  /* MODER：清除 Pin13 的 2 位模式字段 */
    GPIOD->MODER |=  (2U << (2 * 13));  /* MODER：写入 10b = 复用功能模式    */

    /* ────────────────────────────────────────────────────────────────────
     *    AFR[1]（Alternate Function Register High，复用功能寄存器高位）：
     *      管理 Pin8~Pin15 的复用功能选择，每引脚 4 位（AF0~AF15）。
     *      Pin13 偏移 = (13-8)*4 = 20 位，写入 2 = AF2。
     *      查 STM32F407 数据手册 Table 9：AF2 对应 TIM4_CH2（PD13）。
     * ──────────────────────────────────────────────────────────────────── */
    GPIOD->AFR[1] &= ~(0xFU << ((13 - 8) * 4));  /* AFR[1]：清除 Pin13 的 AF 选择位 */
    GPIOD->AFR[1] |=  (2U   << ((13 - 8) * 4));  /* AFR[1]：写入 AF2 = TIM4_CH2    */

    /* ────────────────────────────────────────────────────────────────────
     * 3. TIM4 时基配置
     *    PSC（Prescaler，预分频寄存器）：
     *      计数时钟 = 84 MHz / (559 + 1) = 150 kHz
     *    ARR（Auto-Reload Register，自动重装载寄存器）：
     *      计数器从 0 数到 ARR 后溢出，PWM 频率 = 150 kHz / (999+1) = 150 Hz
     *      ARR = 999 意味着 CCR2 有效范围为 0~1000，共 1000 级分辨率。
     * ──────────────────────────────────────────────────────────────────── */
    TIM4->PSC = 559;      /* PSC：Prescaler，84MHz / 560 = 150kHz 计数时钟 */
    TIM4->ARR = PWM_ARR;  /* ARR：Auto-Reload Register，999 → 150Hz PWM    */

    /* ────────────────────────────────────────────────────────────────────
     * 4. CCMR1 - 通道2配置为 PWM 输出模式
     *    CCMR1（Capture/Compare Mode Register 1，捕获/比较模式寄存器1）
     *    用于配置通道1（低8位）和通道2（高8位）的工作模式。
     *
     *    OC2M（Output Compare 2 Mode，通道2输出比较模式）：
     *      位[14:12] = 110b = PWM 模式1
     *      PWM模式1：当 CNT < CCR2 时输出高电平（亮），否则输出低电平（暗）。
     *      占空比 = CCR2 / (ARR+1) = CCR2 / 1000
     *
     *    OC2PE（Output Compare 2 Preload Enable，通道2预装载使能）：
     *      = 1 → 对 CCR2 的写入暂存于预装载寄存器，在下次更新事件时才生效。
     *      避免在 PWM 周期中途修改 CCR2 时产生脉冲宽度突变（电压毛刺）。
     * ──────────────────────────────────────────────────────────────────── */
    TIM4->CCMR1 &= ~(TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE);  /* 清除通道2模式位 */
    TIM4->CCMR1 |=  (6U << TIM_CCMR1_OC2M_Pos)           /* OC2M = 110b：PWM模式1 */
                 |  TIM_CCMR1_OC2PE;                      /* OC2PE = 1：预装载使能 */

    /* ────────────────────────────────────────────────────────────────────
     * 5. CCR2 - 初始占空比
     *    CCR2（Capture/Compare Register 2，通道2比较寄存器）：
     *      存储 PWM 的比较阈值，占空比 = CCR2 / (ARR+1)。
     *      初始值 = BACKLIGHT_DIM = 200，即 200/1000 = 20% 占空比（低亮度）。
     * ──────────────────────────────────────────────────────────────────── */
    TIM4->CCR2   = BACKLIGHT_DIM;  /* CCR2：初始占空比 20%（低亮度启动）*/
    current_duty = BACKLIGHT_DIM;  /* 同步更新软件状态变量 */

    /* ────────────────────────────────────────────────────────────────────
     * 6. CCER - 使能通道2输出
     *    CCER（Capture/Compare Enable Register，捕获/比较使能寄存器）
     *    CC2E（Capture/Compare 2 output Enable，通道2输出使能）：
     *      = 1 → 允许通道2将 PWM 信号输出到 PD13 引脚。
     *      未使能时，PD13 不受 TIM4 控制，即使定时器在运行也无 PWM 输出。
     * ──────────────────────────────────────────────────────────────────── */
    TIM4->CCER |= TIM_CCER_CC2E;  /* CCER.CC2E = 1：使能通道2 PWM 输出 */

    /* ────────────────────────────────────────────────────────────────────
     * 7. CR1 - 使能预装载，准备启动
     *    CR1（Control Register 1，定时器控制寄存器1）
     *    ARPE（Auto-Reload Preload Enable，自动重装载预装载使能）：
     *      = 1 → ARR 的修改同样通过预装载机制生效，与 OC2PE 配合保证波形完整性。
     * ──────────────────────────────────────────────────────────────────── */
    TIM4->CR1 |= TIM_CR1_ARPE;  /* CR1.ARPE：ARR 预装载使能 */

    /* ────────────────────────────────────────────────────────────────────
     * 8. EGR.UG - 软件触发更新事件，使配置立即生效
     *    EGR（Event Generation Register，事件生成寄存器）
     *    UG（Update Generation，软件更新触发位）：
     *      写 1 → 将 PSC、ARR、CCR2 的预装载值立即复制到影子寄存器，
     *      并将计数器复位为 0，同时置位 SR.UIF（需清除防止误中断）。
     *    SR（Status Register）写 0 清除 UIF 标志。
     * ──────────────────────────────────────────────────────────────────── */
    TIM4->EGR |= TIM_EGR_UG;  /* EGR.UG：Update Generation，使 PSC/ARR/CCR2 立即生效 */
    TIM4->SR   = 0;            /* SR：清除 UG 产生的 UIF 标志，防止立即触发更新中断 */

    /* ────────────────────────────────────────────────────────────────────
     * 9. CR1.CEN - 启动定时器
     *    CEN（Counter Enable，计数器使能位）：
     *      写 1 → 定时器开始计数，PWM 信号从此刻起输出到 PD13。
     * ──────────────────────────────────────────────────────────────────── */
    TIM4->CR1 |= TIM_CR1_CEN;  /* CR1.CEN：Counter Enable，启动 TIM4 */
}

/* ==================== Set_Backlight_Duty ==================== */
/*
 * 动态更新 PWM 占空比（背光亮度）。
 *
 * CCR2（Capture/Compare Register 2）：
 *   直接写入新的比较值，由于 OC2PE=1（预装载使能），
 *   新值会在下一个 PWM 周期（约 6.7 ms）开始时平滑生效，不产生毛刺。
 *   可在 ISR 中安全调用（单次寄存器写入是原子操作）。
 */
void Set_Backlight_Duty(uint16_t duty)
{
    if (duty > BACKLIGHT_FULL) duty = BACKLIGHT_FULL;  /* 超出上限时截断 */
    current_duty = duty;    /* 更新软件状态变量（供淡出逻辑读取）*/
    TIM4->CCR2   = duty;    /* CCR2：写入新占空比，预装载后下周期生效 */
}
