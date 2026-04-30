/* Mikrocontroller-Programmierung - Aufgabe 5: PWM-Dimmen des Displays */
/*
 * Ziel: Hintergrundbeleuchtung (PD13) über PWM mit TIM4_CH2 dimmen.
 *   - Normalbetrieb: reduzierte Helligkeit (20% Duty Cycle)
 *   - Tastendruck:   volle Helligkeit (100%) für 10 Sekunden
 *   - Danach:        langsame Rückkehr zur reduzierten Helligkeit (ca. 2 s)
 *
 * PWM-Konfiguration:
 *   PD13 = TIM4_CH2 (AF2, laut Datenblatt Tabelle 9, Seite 60)
 *   TIM4-Takt = 84 MHz (APB1-Prescaler=4, Timertakt = APB1 × 2)
 *   PSC=559, ARR=999  →  84 MHz / 560 / 1000 = 150 Hz
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

/* ==================== Aufgabe 6: Frequenzmessung (TIM12 Input Capture) ==================== */
/*
 * PB14 = TIM12_CH1 (AF9, laut Datenblatt Tabelle 9)
 * TIM12-Takt = 84 MHz / (83+1) = 1 MHz  →  1 Tick = 1 µs
 * Frequenz (Hz) = 1.000.000 / Ticks_zwischen_zwei_Flanken
 *
 * TIM12 ist 16-bit; Überläufe werden gezählt und zu einem
 * virtuellen 32-Bit-Zähler kombiniert → auch sehr niedrige
 * Frequenzen (<15 Hz) sind messbar.
 *
 * Interrupt-Vektor: TIM8_BRK_TIM12_IRQn
 *   (TIM12 teilt sich den Vektor mit TIM8_BRK auf STM32F407)
 */
#define TIM12_CLOCK_HZ  1000000UL   /* 1 MHz Zähltakt */
#define CAP_TIMEOUT_MS  2000UL      /* Kein Signal → Anzeige "--" nach 2 s */

/* Messdaten: in der ISR geschrieben, in der Hauptschleife gelesen */
volatile uint32_t cap_ticks    = 0;   /* Ticks zwischen zwei steigenden Flanken */
volatile uint32_t cap_freq_hz  = 0;   /* Berechnete Frequenz in Hz (ganzzahlig) */
volatile uint32_t cap_last_ms  = 0;   /* ms_ticks-Zeitstempel der letzten Messung */
volatile uint8_t  cap_valid    = 0;   /* 1 = mindestens eine gültige Messung vorhanden */

/* ISR-interne Zustandsvariablen (nur in TIM8_BRK_TIM12_IRQHandler verwendet) */
static volatile uint32_t tim12_overflow   = 0;  /* Anzahl der 16-Bit-Überläufe */
static volatile uint32_t tim12_last_cap32 = 0;  /* Letzter 32-Bit-Capture-Wert */
static volatile uint8_t  tim12_first      = 1;  /* Noch keine erste Flanke empfangen */

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
 * PD13 wird auf AF-Modus (AF2 = TIM4) umgeschaltet.
 * TIM4 Channel 2 erzeugt ein 150-Hz-PWM-Signal.
 *
 * Berechnung:
 *   TIM4-Takt  = 84 MHz
 *   PSC        = 559  →  Zähltakt = 84 MHz / (559+1) = 150 kHz
 *   ARR        = 999  →  PWM-Frequenz = 150 kHz / (999+1) = 150 Hz
 */
void Timer4_PWM_Init(void)
{
    /* 使能 GPIOD 和 TIM4 时钟 */
    // RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; // steht schon in LEDs_InitPorts() an Zeil 350
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    /* PD13 → AF-Modus (MODER[27:26] = 10b) */
    GPIOD->MODER &= ~(3U << (2 * 13)); // U: unsigned literal suffix (positive)
    GPIOD->MODER |=  (2U << (2 * 13));

    /* Alternate Function: AF2 (TIM4) für Pin 13 in AFRH (AFR[1])
     * AFRH bits [23:20] (= (13-8)*4 = 20) auf 0010 setzen */
    GPIOD->AFR[1] &= ~(0xFU << ((13 - 8) * 4));
    GPIOD->AFR[1] |=  (2U   << ((13 - 8) * 4));

    /* TIM4 Vorteiler und Periode */
    TIM4->PSC = 559;       /* 84 MHz / 560 = 150 kHz Zähltakt */
    TIM4->ARR = PWM_ARR;   /* ARR = 999 → 150 Hz PWM */

    /* CH2: PWM-Modus 1 (OC2M = 110b) + Preload-Enable (OC2PE = 1)
     * PWM-Modus 1: Ausgang HIGH wenn CNT < CCR2, LOW sonst */
    TIM4->CCMR1 &= ~(TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE);
    TIM4->CCMR1 |=  (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;

    /* Anfangs-Duty-Cycle: reduzierte Helligkeit */
    TIM4->CCR2 = BACKLIGHT_DIM;
    current_duty = BACKLIGHT_DIM;

    /* CH2-Ausgang aktivieren (CCER: CC2E = 1) */
    TIM4->CCER |= TIM_CCER_CC2E;

    /* Auto-Reload-Preload aktivieren */
    TIM4->CR1 |= TIM_CR1_ARPE;

    /* Update-Event erzwingen, damit PSC/ARR sofort geladen werden */
    TIM4->EGR |= TIM_EGR_UG;
    TIM4->SR   = 0;

    /* TIM4 starten */
    TIM4->CR1 |= TIM_CR1_CEN;
}

/* ==================== PWM-Helligkeit setzen ==================== */
/* Setzt CCR2 direkt (sicher auch aus ISR heraus aufrufbar). */
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
    TIM7->SR &= ~TIM_SR_UIF;   /* Interrupt-Flag löschen */

    ms_ticks++;

    /* ------------------------------------------------------------------ */
    /* 1. Tastenerkennung mit Entprellung (alle 50 ms)                     */
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
    /* 4. 背光淡出（倒计时结束后缓慢降低亮度）                              */
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

/* ==================== Aufgabe 6: TIM12 Input-Capture-Initialisierung ==================== */
/*
 * PB14 wird als TIM12_CH1 (AF9) konfiguriert.
 *
 * Zähltakt-Berechnung:
 *   TIM12-Takt = APB1-Timertakt = 84 MHz
 *   PSC = 83  →  Zähltakt = 84 MHz / (83+1) = 1 MHz  →  1 Tick = 1 µs
 *   ARR = 0xFFFF (65535) → maximale Zählperiode ohne Prescaler-Anpassung
 *
 * Interrupts:
 *   CC1IE  = Capture-Interrupt (steig. Flanke auf PB14)
 *   UIE    = Update-Interrupt  (Überlauf des 16-Bit-Zählers)
 *   Vektor = TIM8_BRK_TIM12_IRQn (geteilt auf STM32F407)
 */
void Timer12_Capture_Init(void)
{
    /* ---- GPIOB-Takt und TIM12-Takt aktivieren ---- */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;

    /* ---- PB14: Alternate-Function-Modus (MODER[29:28] = 10b) ---- */
    GPIOB->MODER &= ~(3U << (2 * 14));
    GPIOB->MODER |=  (2U << (2 * 14));

    /* ---- AF9 (TIM12_CH1) für Pin 14 in AFRH (AFR[1])
     *      Bits [27:24] = (14-8)*4 = 24 → auf 1001 (9) setzen ---- */
    GPIOB->AFR[1] &= ~(0xFU << ((14 - 8) * 4));
    GPIOB->AFR[1] |=  (9U   << ((14 - 8) * 4));

    /* ---- TIM12-Grundkonfiguration ---- */
    TIM12->CR1  = 0;
    TIM12->PSC  = 83;       /* 84 MHz / 84 = 1 MHz → 1 Tick = 1 µs        */
    TIM12->ARR  = 0xFFFF;   /* Maximaler 16-Bit-Zählerstand                */

    /* ---- Kanal 1: Input Capture, ungefiltert, kein Prescaler
     *      CC1S = 01b → TI1 (direkter Eingang)
     *      IC1PSC = 00b → kein Prescaler (jede Flanke auslösen)
     *      IC1F   = 0000b → kein digitaler Filter                 ---- */
    TIM12->CCMR1 = (1U << TIM_CCMR1_CC1S_Pos);  /* CC1S = 01 */

    /* ---- Steigende Flanke, Capture aktivieren (CCER: CC1P=0, CC1E=1) ---- */
    TIM12->CCER  = TIM_CCER_CC1E;   /* CC1P=0 → steigende Flanke            */

    /* ---- Interrupts: Capture (CC1IE) + Überlauf (UIE) aktivieren ---- */
    TIM12->DIER  = TIM_DIER_CC1IE | TIM_DIER_UIE;

    /* ---- NVIC: TIM12 teilt Vektor TIM8_BRK_TIM12_IRQn mit TIM8_BRK
     *      Priorität höher als TIM7 (0 < 1), damit Captures exakt erfasst werden ---- */
    NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 0);
    NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);

    /* ---- Update-Event erzwingen, Flags löschen, Timer starten ---- */
    TIM12->EGR |= TIM_EGR_UG;
    TIM12->SR   = 0;
    TIM12->CR1 |= TIM_CR1_CEN;
}

/* ==================== Aufgabe 6: TIM12 Interrupt-Serviceroutine ==================== */
/*
 * Dieser Handler wird für zwei Ereignisse aufgerufen:
 *   1. UIF  (Update/Overflow): TIM12-Zähler hat 0xFFFF überschritten → Überlaufzähler ++
 *   2. CC1IF (Capture):        Steigende Flanke auf PB14 → Zeitstempel auslesen
 *
 * Race-Condition-Behandlung (beide Flags gleichzeitig gesetzt):
 *   Wenn CCR1 < 0x8000 → Flanke kam kurz NACH dem Überlauf
 *     → aktueller overflow-Wert ist bereits korrekt hochgezählt (kein Abzug)
 *   Wenn CCR1 >= 0x8000 → Flanke kam kurz VOR dem Überlauf
 *     → overflow wurde fälschlich inkrementiert → overflow um 1 reduzieren
 *
 * Frequenzberechnung (ganzzahlig, kein Float):
 *   cap_freq_hz = TIM12_CLOCK_HZ / cap_ticks
 *               = 1.000.000 / delta_ticks
 */
void TIM8_BRK_TIM12_IRQHandler(void)
{
    uint32_t sr = TIM12->SR;

    /* ------------------------------------------------------------------ */
    /* 1. Überlauf-Interrupt (UIF): Überlaufzähler inkrementieren          */
    /* ------------------------------------------------------------------ */
    if (sr & TIM_SR_UIF)
    {
        TIM12->SR &= ~TIM_SR_UIF;   /* Flag löschen */
        tim12_overflow++;
    }

    /* ------------------------------------------------------------------ */
    /* 2. Capture-Interrupt (CC1IF): Zeitstempel auswerten                 */
    /* ------------------------------------------------------------------ */
    if (sr & TIM_SR_CC1IF)
    {
        TIM12->SR &= ~TIM_SR_CC1IF;   /* Flag löschen */

        /* CCR1 lesen (löscht CC1IF hardwareseitig ebenfalls) */
        uint16_t raw16 = (uint16_t)TIM12->CCR1;

        /* Überlaufkorrektur bei gleichzeitigem UIF + CC1IF:
         * Wenn CCR1 groß (≥ 0x8000), kam die Flanke VOR dem Überlauf
         * → der Überlaufzähler wurde bereits inkrementiert, muss korrigiert werden */
        uint32_t ov = tim12_overflow;
        if ((sr & TIM_SR_UIF) && (raw16 >= 0x8000U))
            ov--;   /* Korrektur: Flanke war vor dem Überlauf */

        /* 32-Bit-Capture-Wert zusammensetzen */
        uint32_t cap32 = (ov << 16) | raw16;

        if (tim12_first)
        {
            /* Erste Flanke: nur Referenz speichern, noch keine Messung */
            tim12_last_cap32 = cap32;
            tim12_first      = 0;
        }
        else
        {
            /* Differenz = Ticks zwischen zwei steigenden Flanken */
            uint32_t delta = cap32 - tim12_last_cap32;
            tim12_last_cap32 = cap32;

            if (delta > 0)
            {
                cap_ticks   = delta;
                cap_freq_hz = TIM12_CLOCK_HZ / delta;  /* Hz, ganzzahlig */
                cap_last_ms = ms_ticks;
                cap_valid   = 1;

                /* LCD-Update maximal alle 200 ms anfordern */
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

    /* ---- Aufgabe 6: TIM12 Input Capture für Frequenzmessung starten ---- */
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
        /* LCD 刷新在主循环中执行，避免阻塞中断（PD12 1Hz 精度依赖于此）*/
        if (lcd_update_pending)
        {
            lcd_update_pending = 0;
            LCD_UpdateDisplay();
        }

        /* 正向单灯跑马灯 */
        for (int i = 0; i < 16; i++)
        {
            LEDs_Write(1 << i);
            u_delay(100000);
        }

        /* 全亮逐个熄灭 */
        for (int i = 0; i < 16; i++)
        {
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
 * LCD_UpdateDisplay - Aktualisiert beide Anzeigezeilen.
 *
 * Zeile 1 (y=10): "Time Ticker: n"   – läuft immer
 * Zeile 2 (y=30): "countdown: N s"   – nur während des 10-s-Countdowns
 *                 (Zeile wird beim Ablauf des Countdowns gelöscht)
 *
 * Hintergrundfarbe 0xFE00 entspricht LCD_ClearDisplay-Farbe,
 * damit alte Ziffern sauber überschrieben werden.
 */
/*
 * LCD_UpdateDisplay - Aktualisiert alle vier Anzeigezeilen.
 *
 * Zeile 1 (y=10): "Time Ticker: n"     – läuft immer
 * Zeile 2 (y=30): "countdown: N s"     – nur während des 10-s-Countdowns
 *                 (Zeile wird beim Ablauf des Countdowns gelöscht)
 * Zeile 3 (y=55): "Ticks: xxxxxxx"     – Messwert in Mikrosekunden
 *                 "--      " bei Timeout (kein Signal)
 * Zeile 4 (y=75): "Freq:  xxxxx Hz"    – berechnete Frequenz in Hz
 *                 "--      " bei Timeout (kein Signal)
 *
 * Hintergrundfarbe 0xFE00 entspricht LCD_ClearDisplay-Farbe,
 * damit alte Ziffern sauber überschrieben werden.
 */
void LCD_UpdateDisplay(void)
{
    char buf[32];

    /* ---- Zeile 1: Sekundenzähler (immer sichtbar) ---- */
    sprintf(buf, "Time Ticker: %-6d", counter);
    LCD_WriteString(10, 10, WHITE, 0xFE00, buf);

    /* ---- Zeile 2: Countdown (nur während aktivem Countdown) ---- */
    if (countdown_active)
    {
        uint32_t elapsed   = ms_ticks - release_time;
        uint8_t  remaining = (elapsed < 10000) ? (uint8_t)(10 - elapsed / 1000) : 0;
        sprintf(buf, "countdown: %-2d s  ", remaining);
        LCD_WriteString(10, 30, WHITE, 0xFE00, buf);
    }
    else
    {
        LCD_WriteString(10, 30, 0xFE00, 0xFE00, "                  ");
    }

    /* ---- Zeilen 3+4: Frequenzmessung (Aufgabe 6) ---- */
    /* Timeout-Prüfung: kein gültiges Signal seit CAP_TIMEOUT_MS → "--" anzeigen */
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
