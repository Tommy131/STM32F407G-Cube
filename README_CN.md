# STM32F407 微控制器编程实验 – 技术文档

**学校：** 英戈尔施塔特应用技术大学（THI）  
**课程：** 信息学 2 – 微控制器编程（Mikrocontroller-Programmierung）  
**目标硬件：** STM32F407VGTx Discovery Board（Cortex-M4，168 MHz）  
**开发环境：** Keil MDK，ARM Clang V6.24  

---

## 目录

1. [整体设计思路与架构](#1-整体设计思路与架构)
2. [第1/2题 – LED 跑马灯（GPIO 控制）](#2-第12题--led-跑马灯gpio-控制)
3. [第3题 – LCD 显示屏（SSD2119 驱动）](#3-第3题--lcd-显示屏ssd2119-驱动)
4. [第4题 – 按键检测与定时器（TIM7）](#4-第4题--按键检测与定时器tim7)
5. [第5题 – PWM 背光调光（TIM4）](#5-第5题--pwm-背光调光tim4)
6. [第6题 – 频率测量（TIM12 输入捕获）](#6-第6题--频率测量tim12-输入捕获)
7. [模块化代码结构](#7-模块化代码结构)

---

## 1. 整体设计思路与架构

### 1.1 核心设计原则

本项目采用**渐进式开发策略**，每道题在前一题的基础上添加新功能，不破坏已有的实现。整个开发过程遵循以下几条核心原则：

#### 原则一：中断服务函数（ISR）保持最短

ISR 的执行时间直接影响系统实时性。本项目的 ISR 只做最轻量级的工作——维护毫秒计数器：

```c
void TIM7_IRQHandler(void)
{
    if (!(TIM7->SR & TIM_SR_UIF)) return;
    TIM7->SR &= ~TIM_SR_UIF;
    ms_ticks++;  // ISR 只做这一件事
}
```

所有业务逻辑（按键检测、LED 闪烁、倒计时、背光淡出）均移至主循环的 `App_ProcessTickTasks()` 函数中。这样做的好处是：
- 避免在 ISR 中执行耗时的 GPIO 操作
- 便于调试（主循环代码可以用断点调试，ISR 不行）
- 降低嵌套中断带来的复杂性

#### 原则二：时间片化延时，避免"阻塞盲区"

传统的 `u_delay(3200000)` 会让 CPU 盲目空转 3.2 秒，期间所有业务逻辑都无法响应。本项目将延时切分为 1 ms 的时间片：

```c
void App_DelayWithTasks(uint32_t total_us)
{
    const uint32_t slice_us = 1000;  // 每次只延时 1 ms

    while (total_us >= slice_us) {
        u_delay(slice_us);
        App_ProcessTickTasks();  // 延时间隙处理所有时序任务
        total_us -= slice_us;
    }
    if (total_us > 0) {
        u_delay(total_us);
        App_ProcessTickTasks();
    }
}
```

这样，即使在 100 ms 的 LED 步进延时中，按键检测每 1 ms 也能被响应一次，最大响应延迟从 ~3200 ms 降至 ~1 ms。

#### 原则三：标志位驱动的 LCD 刷新

LCD 更新函数每次调用需要执行数百次 GPIO 操作（每个字符 192 次像素写入），绝对不能在 ISR 中调用。因此采用"标志位 + 主循环轮询"的模式：

```c
// 在 ISR 或其他模块中：仅设置标志位
lcd_update_pending = 1;

// 在主循环中：检测标志位后执行实际更新
for (int i = 0; i < 16; i++) {
    App_ProcessTickTasks();
    if (lcd_update_pending) {
        lcd_update_pending = 0;
        LCD_UpdateDisplay();  // 只在主循环中调用
    }
    LEDs_Write(1 << i);
    App_DelayWithTasks(100000);
}
```

### 1.2 STM32F407 时钟树

理解时钟分配对所有定时器配置都至关重要：

```
外部晶振 / 内部 RC 振荡器
    │
    └─ PLL 倍频 → SYSCLK = 168 MHz
                    │
                    ├─ AHB1总线（÷1）= 168 MHz → GPIO、DMA 等
                    │
                    └─ APB1总线（÷4）= 42 MHz
                            │
                            └─ APB1定时器时钟（×2补偿）= 84 MHz
                               → TIM4、TIM7、TIM12 均使用此时钟
```

**关键点：** APB1 外设时钟为 42 MHz，但 APB1 **定时器**时钟为 84 MHz（当 APB1 预分频 ≠ 1 时，定时器时钟自动 ×2 补偿）。因此 PSC 计算时必须以 84 MHz 为基准。

### 1.3 各题依赖关系

```
第1/2题：GPIO 初始化、LED 总线驱动
    └── 第3题：LCD 驱动（复用 LCD_Output16BitWord）
    └── 第5题：PWM 背光（GPIOD 已由第1/2题初始化）
第4题：TIM7 心跳 → 第5题倒计时触发 → 第6题超时判断
第6题：TIM12 输入捕获 → 测量结果写入 LCD（第3题）
```

---

## 2. 第1/2题 – LED 跑马灯（GPIO 控制）

### 2.1 硬件拓扑：共享数据总线

本实验板的 16 个 LED **并非**每个 LED 独占一个 GPIO 引脚。LED 阵列通过**移位寄存器（Shift Register）**与 LCD 共享同一组 16 位并行数据总线（GPIOD + GPIOE）。由 SEL 信号决定数据发往哪个设备：

```
CPU
 │
 ├─ GPIOD/GPIOE（16根数据线）──────┬──── LCD（当 SEL=0 时接收）
 │                               └──── 移位寄存器（当 SEL=1 时接收）
 │                                           │
 ├─ PD5（WR）：写时钟脉冲                       └─ 16个LED
 ├─ PD7（CS）：片选（低有效）
 └─ PD11（SEL）：设备选择
```

这种设计节省了大量 GPIO 引脚，但代价是数据总线的时序控制必须非常精确。

### 2.2 位重映射：最核心的技术挑战

16 根数据线分布在 GPIOD 和 GPIOE 的**非连续位段**上，因此不能简单地将 16 位数据写入一个寄存器。必须将数据拆分并重新映射：

| 数据位 | 目标 GPIO 引脚 |
|--------|--------------|
| `data[1:0]` | `PD[1:0]`（PD1、PD0） |
| `data[3:2]` | `PD[10:8]`（PD10、PD9、PD8 中的低2位）|
| `data[15:13]` | `PD[15:14]`（PD15、PD14）|
| `data[12:4]` | `PE[15:7]`（PE15~PE7，共9位）|

```c
void LCD_Output16BitWord(uint16_t data)
{
    uint16_t d = 0, e = 0;

    // 第一步：计算各字段应写入的目标位
    d |= ((data & 0x3) << 14);        // data[1:0]   → GPIOD[15:14]
    d |= ((data >> 2) & 0x3);         // data[3:2]   → GPIOD[1:0]
    d |= ((data >> 13) & 0x7) << 8;   // data[15:13] → GPIOD[10:8]
    e |= ((data >> 4) & 0x1FF) << 7;  // data[12:4]  → GPIOE[15:7]

    // 第二步：先清除目标位段（保留其他引脚不变）
    GPIOD->ODR &= ~((0x3 << 14) | 0x3 | (0x7 << 8));
    GPIOE->ODR &= ~(0x1FF << 7);

    // 第三步：写入新值
    GPIOD->ODR |= d;
    GPIOE->ODR |= e;
}
```

**为什么要先清除再写入（Read-Modify-Write）？**  
因为 ODR 寄存器同时控制该端口的所有 16 个引脚。例如 PD12（板载 LED）、PD13（PWM 背光）也在 GPIOD 中，如果直接赋值 `GPIOD->ODR = d`，这些引脚的状态会被覆盖。使用 `&= ~mask` + `|= value` 只修改目标位，其余位保持不变。

### 2.3 GPIO 引脚初始化

所有数据总线引脚必须配置为**推挽输出模式**（通用输出，MODER = 01b）：

```c
void set_output(GPIO_TypeDef *port, int pin)
{
    // MODER：每引脚占 2 位，Pin N 对应 Bit[2N+1 : 2N]
    port->MODER &= ~(3 << (2 * pin));  // 先清零
    port->MODER |=  (1 << (2 * pin));  // 写入 01b = 通用输出
}

void LEDs_InitPorts(void)
{
    // AHB1ENR：使能 GPIOD 和 GPIOE 的时钟（上电后默认关闭）
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;

    // 数据引脚：全部设为输出
    set_output(GPIOD, 0);  set_output(GPIOD, 1);
    set_output(GPIOD, 8);  set_output(GPIOD, 9);  set_output(GPIOD, 10);
    set_output(GPIOD, 14); set_output(GPIOD, 15);
    for (int i = 7; i <= 15; i++)
        set_output(GPIOE, i);

    // 控制引脚：WR / CS / SEL
    set_output(GPIOD, LED_WR_PIN);   // PD5
    set_output(GPIOD, LED_CS_PIN);   // PD7
    set_output(GPIOD, LED_SEL_PIN);  // PD11

    // ODR：控制信号初始化为高电平（非激活状态）
    GPIOD->ODR |= (1 << LED_WR_PIN) | (1 << LED_CS_PIN) | (1 << LED_SEL_PIN);
}
```

### 2.4 LED 写入时序

移位寄存器芯片的锁存信号需要满足特定时序：

```
SEL ─── 1 ──────────────────────────────── 1
CS  ─────── 0 ─────────────────────── 1 ───
DATA    （在总线上保持稳定）
WR  ────── 1 ─── 0 ─── 1 ───────────────────
                     ↑
               上升沿锁存数据
```

```c
void LEDs_Write(uint16_t data)
{
    GPIOD->ODR |=  (1 << LED_SEL_PIN);  // SEL=1：选择 LED 总线
    GPIOD->ODR &= ~(1 << LED_CS_PIN);   // CS=0：使能移位寄存器

    LCD_Output16BitWord(data);           // 将 16 位数据输出到总线

    GPIOD->ODR |=  (1 << LED_WR_PIN);   // WR=1（准备产生下降沿）
    GPIOD->ODR &= ~(1 << LED_WR_PIN);   // WR=0（下降沿）
    GPIOD->ODR |=  (1 << LED_WR_PIN);   // WR=1（上升沿 → 锁存数据）

    GPIOD->ODR |=  (1 << LED_CS_PIN);   // CS=1：释放总线
}
```

### 2.5 跑马灯实现

**正向单灯移动（LED 0→15 依次点亮）：**
```c
for (int i = 0; i < 16; i++) {
    LEDs_Write(1 << i);          // 只有第 i 个 bit 为 1
    App_DelayWithTasks(100000);  // 等待 100 ms
}
```

**逐个熄灭（全亮后依次关闭）：**
```c
for (int i = 0; i < 16; i++) {
    LEDs_Write(~(uint16_t)(1 << i));  // 第 i 个 bit 为 0，其余全 1
    App_DelayWithTasks(100000);
}
```

### 2.6 实现难点

**难点1：长延时期间任务响应丢失**  
完整一轮跑马灯需要 32 × 100 ms = 3.2 秒。如果使用 `u_delay(3200000)` 直接阻塞，LCD 更新、按键检测、LED 闪烁都将暂停 3.2 秒。解决方案是使用 `App_DelayWithTasks()`，将延时切分为 1 ms 时间片，每片后执行一次任务处理。

**难点2：GPIO 位操作的正确性**  
由于 ODR 同时控制多个功能引脚（LED、PWM、WR/CS/SEL 等），任何全寄存器赋值操作都可能破坏其他引脚的状态。必须使用 Read-Modify-Write（`&= ~mask` 清位，`|= value` 置位）。

---

## 3. 第3题 – LCD 显示屏（SSD2119 驱动）

### 3.1 8080 并行总线接口

SSD2119 使用 Intel 8080 兼容的并行总线协议，支持同步 16 位数据传输：

| 信号 | 引脚 | 功能 |
|------|------|------|
| CS | PD7 | 片选（低有效，选中 LCD） |
| DC | PE3 | 0=命令模式，1=数据模式 |
| WR | PD5 | 写脉冲（上升沿锁存数据） |
| RD | PD4 | 读脉冲（本实验仅写，保持高电平）|
| RST | PD3 | 硬件复位（低电平脉冲）|
| D[15:0] | GPIOD/E | 16位数据总线 |

**基本写操作时序：**
```
CS  ─── 0 ──────────────────────── 1 ───
DC  ─── （0=命令 / 1=数据）──────────────
DATA    （16位数据稳定在总线上）
WR  ─── 0 ─── 1 ────────────────────────
              ↑ 上升沿 → 控制器锁存数据
```

```c
// 发送命令（寄存器地址）
void LCD_WriteCommand(uint16_t cmd)
{
    LCD_CS_ENABLE();     // PD11=0：OR门使能 CS 路径
    LCD_DC_CMD();        // PE3=0：进入命令模式
    LCD_CS_LOW();        // PD7=0：选中 LCD
    LCD_Output16BitWord(cmd);
    LCD_WR_LOW();        // PD5=0：建立数据
    LCD_WR_HIGH();       // PD5=1：上升沿锁存
    LCD_CS_HIGH();       // PD7=1：取消选中
}

// 发送数据（像素颜色或寄存器值）
void LCD_WriteData(uint16_t data)
{
    LCD_CS_ENABLE();
    LCD_DC_DATA();       // PE3=1：数据模式
    LCD_CS_LOW();
    LCD_Output16BitWord(data);
    LCD_WR_LOW();
    LCD_WR_HIGH();
    LCD_CS_HIGH();
}
```

### 3.2 RGB565 颜色格式

SSD2119 使用 RGB565 格式：每个像素 16 位，其中 5 位红色、6 位绿色、5 位蓝色：

```
位域：[15:11]=R(5位)  [10:5]=G(6位)  [4:0]=B(5位)

示例：
白色：0xFFFF = 11111_111111_11111
黑色：0x0000 = 00000_000000_00000
红色：0xF800 = 11111_000000_00000
绿色：0x07E0 = 00000_111111_00000
蓝色：0x001F = 00000_000000_11111
```

**注意：** 头文件中 `RGB565(r, b, g)` 宏的参数顺序为 `(红, 蓝, 绿)`，第二和第三参数交换，这是因为底层位映射的特殊性。

### 3.3 GRAM 光标定位与像素写入

SSD2119 内部有一个图形 RAM（GRAM），通过 X/Y 坐标寄存器定位后连续写入像素数据：

```c
// 将光标移至 (x, y)
void LCD_SetCursor(uint32_t x, uint32_t y)
{
    LCD_WriteReg(SSD2119_X_RAM_ADDR_REG, (uint16_t)x);  // 0x004E：X坐标
    LCD_WriteReg(SSD2119_Y_RAM_ADDR_REG, (uint16_t)y);  // 0x004F：Y坐标
}

// 在当前光标位置写入一个像素（控制器自动递增内部地址）
void LCD_DrawPixel(uint16_t color)
{
    LCD_WriteCommand(SSD2119_RAM_DATA_REG);  // 0x0022：GRAM 写入寄存器
    LCD_WriteData(color);
}
```

**清屏性能优化：**  
`LCD_ClearDisplay()` 需要写入 320×240 = 76800 个像素。若每个像素都单独操作 CS 和 DC，开销极大。优化方案是在整个传输过程中保持 CS 和 DC 状态不变，仅改变 WR 信号：

```c
void LCD_ClearDisplay(uint16_t color)
{
    LCD_SetCursor(0, 0);
    LCD_WriteCommand(SSD2119_RAM_DATA_REG);
    LCD_CS_ENABLE();
    LCD_DC_DATA();
    LCD_CS_LOW();                          // CS 保持低电平（不反复切换）
    for (uint32_t i = 0; i < 320*240; i++) {
        LCD_Output16BitWord(color);
        LCD_WR_LOW();
        LCD_WR_HIGH();                     // 只有 WR 脉冲，其余信号不动
    }
    LCD_CS_HIGH();
}
```

### 3.4 字符渲染：12×16 点阵字体

每个 ASCII 字符以 12×16 像素的点阵形式存储在 `console_font_12x16[]` 数组中：

- 每个字符占 32 字节（16行 × 2字节/行）
- 每行 2 字节（16位），高 12 位有效（bit15 = 最左列像素）
- 字符 `c` 的起始偏移 = `(uint8_t)c * 32`

```c
void LCD_WriteLetter(uint16_t x, uint16_t y,
                     uint16_t colFG, uint16_t colBG, char c)
{
    uint16_t offset = (uint8_t)c * 32;  // 每字符 32 字节

    for (uint8_t row = 0; row < 16; row++) {
        // 将两个字节合并为 16 位像素行掩码
        uint16_t pixel_row =
            ((uint16_t)console_font_12x16[offset + row * 2] << 8) |
             (uint16_t)console_font_12x16[offset + row * 2 + 1];

        LCD_SetCursor(x, y + row);
        LCD_WriteCommand(SSD2119_RAM_DATA_REG);

        LCD_CS_ENABLE(); LCD_DC_DATA(); LCD_CS_LOW();  // 一行只切换一次
        for (uint8_t col = 0; col < 12; col++) {
            // 0x8000U >> col：从左到右依次检查每一位
            uint16_t color = (pixel_row & (0x8000U >> col)) ? colFG : colBG;
            LCD_Output16BitWord(color);
            LCD_WR_LOW();
            LCD_WR_HIGH();
        }
        LCD_CS_HIGH();
    }
}
```

### 3.5 图形算法（第3题扩展）

**Bresenham 直线算法（纯整数实现）：**  
核心思想是用整数误差项 `err` 来判断下一个像素应该水平移动还是斜向移动，完全避免浮点运算：

```c
static void A3_DrawLine(int x0, int y0, int x1, int y1, uint16_t color)
{
    int dx =  abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;  // 初始误差

    while (1) {
        A3_DrawPixelXY(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }  // 水平步进
        if (e2 <= dx) { err += dx; y0 += sy; }  // 垂直步进
    }
}
```

**中点圆算法（Midpoint Circle Algorithm）：**  
利用圆的 8 重对称性，每次只计算 1/8 圆弧，同时绘制对称的 8 个点：

```c
static void A3_DrawCircle(uint16_t cx, uint16_t cy, uint16_t r, uint16_t color)
{
    int x = r, y = 0, err = 1 - x;
    while (x >= y) {
        // 8 重对称：一次计算画出 8 个点
        A3_DrawPixelXY(cx+x, cy+y, color); A3_DrawPixelXY(cx+y, cy+x, color);
        A3_DrawPixelXY(cx-y, cy+x, color); A3_DrawPixelXY(cx-x, cy+y, color);
        A3_DrawPixelXY(cx-x, cy-y, color); A3_DrawPixelXY(cx-y, cy-x, color);
        A3_DrawPixelXY(cx+y, cy-x, color); A3_DrawPixelXY(cx+x, cy-y, color);
        y++;
        err += (err < 0) ? (2*y + 1) : (2*(y - --x) + 1);
    }
}
```

### 3.6 实现难点

**难点1：旧数字残留问题**  
当显示的数字从"1234"变为"99"时，"34"两位会残留在屏幕上。解决方案：使用左对齐格式化字符串，并用空格填充至固定宽度：

```c
sprintf(buf, "Freq:  %-7" PRIu32 " Hz", cap_freq_hz);
// 输出："Freq:  78140   Hz"（后面有空格覆盖旧字符）
```

**难点2：LCD 初始化时序**  
SSD2119 需要复杂的初始化序列（几十条寄存器写操作），且每步之间有严格的时序要求。初始化失败会导致屏幕花屏或全白，调试时只能逐步排查。

**难点3：LCD_Output16BitWord 被两个模块共用**  
LED 总线和 LCD 总线共享同一函数。调用前必须通过 SEL 信号正确选择目标设备，否则 LED 数据会被写入 LCD，或 LCD 数据被写入 LED。

---

## 4. 第4题 – 按键检测与定时器（TIM7）

### 4.1 TIM7 定时器配置（1 ms 精确中断）

TIM7 产生精确的 1 ms 周期中断，作为整个系统的时间基准：

```
TIM7 时钟 = APB1 定时器时钟 = 84 MHz
PSC = 8399  →  计数时钟 = 84 MHz / (8399+1) = 10 kHz
ARR = 9     →  中断频率 = 10 kHz / (9+1) = 1 kHz → 周期 1 ms
```

```c
void Timer7_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;  // 使能 TIM7 时钟

    TIM7->CR1 = 0;           // 清零重置
    TIM7->PSC = 8399;        // 预分频：84MHz / 8400 = 10kHz
    TIM7->ARR = 9;           // 自动重装载：周期 = 1ms

    TIM7->CR1 |= TIM_CR1_ARPE  // ARPE=1：ARR 预装载使能
              |  TIM_CR1_URS;  // URS=1：只有溢出才触发中断（不响应软件UG）
    TIM7->EGR |= TIM_EGR_UG;  // UG=1：软件触发更新，使 PSC/ARR 立即生效
    TIM7->SR   = 0;            // 清除 UG 产生的 UIF 标志

    TIM7->DIER |= TIM_DIER_UIE;      // UIE=1：使能溢出中断
    NVIC_SetPriority(TIM7_IRQn, 1);  // 优先级 1（低于 TIM12 的 0）
    NVIC_EnableIRQ(TIM7_IRQn);
    TIM7->CR1  |= TIM_CR1_CEN;      // CEN=1：启动计数器
}
```

**为什么需要 CR1.URS = 1？**  
`EGR.UG = 1` 软件触发更新事件时，如果没有 `URS = 1`，这个操作本身也会产生一次更新中断（UIF 置位）。而此时定时器还没有真正溢出，会产生一次虚假中断。设置 `URS = 1` 后，只有计数器真正溢出才会触发中断，软件触发的更新事件被忽略。

### 4.2 为什么必须手动清除 UIF 标志

```c
void TIM7_IRQHandler(void)
{
    if (!(TIM7->SR & TIM_SR_UIF)) return;  // 防止无关中断
    TIM7->SR &= ~TIM_SR_UIF;              // 必须手动清除！
    ms_ticks++;
}
```

STM32 的定时器 ISR **不会**自动清除 UIF 标志（不同于某些其他微控制器）。如果不清除，处理器返回后发现 UIF 仍然置位，会立即再次进入 ISR，陷入无限循环，导致系统卡死。

### 4.3 软件消抖与边沿检测

机械按键按下和松开时会产生几毫秒的"抖动"（电平在高低之间快速切换）。若每 1 ms 采样一次，会读到很多虚假的跳变。消抖方案：**每 50 ms 只采样一次**：

```c
if (ms_ticks - key_check_last >= 50) {
    key_check_last = ms_ticks;

    // IDR：Input Data Register，只读，反映 PA0 当前实际电平
    uint8_t key_now = (GPIOA->IDR & 1) != 0;

    // 上升沿检测（0→1）：按键按下
    if (key_now && !key_last) {
        button_pressed   = 1;
        countdown_active = 0;
        fade_active      = 0;           // 取消正在进行的淡出
        Set_Backlight_Duty(BACKLIGHT_FULL);  // 背光立即全亮
    }

    // 下降沿检测（1→0）：按键松开
    if (!key_now && key_last) {
        button_pressed   = 0;
        release_time     = ms_ticks;   // 记录松开时刻
        countdown_active = 1;          // 启动 10 秒倒计时
        led_state = 1;
        GPIOD->ODR |= (1 << 12);      // PD12 立即常亮
    }

    key_last = key_now;  // 保存当前状态，下次用于边沿检测
}
```

**边沿检测原理：** `key_last` 保存的是**上一次**采样的按键状态，`key_now` 是**本次**采样结果。只有当两次结果不同时，才发生了状态跳变。

### 4.4 10 秒倒计时与背光淡出逻辑

整个逻辑是一个状态机：

```
状态0（空闲）: 低亮度
    │ 按键按下
    ▼
状态1（按下）: 全亮 + PD12 闪烁
    │ 按键松开
    ▼
状态2（倒计时）: 全亮 + 倒计时显示
    │ 10 秒到期
    ▼
状态3（淡出）: 亮度从 1000 逐步降至 200（共 2 秒）
    │ 淡出完成
    ▼
状态0（空闲）: 低亮度（循环）
```

**倒计时实现：**
```c
if (countdown_active) {
    uint32_t elapsed = ms_ticks - release_time;
    if (elapsed >= 10000) {             // 10000 ms = 10 秒
        countdown_active = 0;
        fade_active = 1;
        fade_last   = ms_ticks;
        LEDs_Write(0x0000);             // 关闭所有 LED
        lcd_update_pending = 1;
    }
}
```

**淡出实现：**
```c
if (fade_active) {
    if (ms_ticks - fade_last >= FADE_INTERVAL) {  // 每 10 ms 执行一步
        fade_last = ms_ticks;
        if (current_duty > BACKLIGHT_DIM + FADE_STEP) {
            current_duty -= FADE_STEP;    // 每步减少 4（CCR2 单位）
            TIM4->CCR2 = current_duty;
        } else {
            current_duty = BACKLIGHT_DIM; // 达到目标低亮度
            TIM4->CCR2   = BACKLIGHT_DIM;
            fade_active  = 0;
        }
    }
}
// 总步数 = (1000-200)/4 = 200 步
// 总时间 = 200 × 10 ms = 2000 ms ≈ 2 秒
```

### 4.5 实现难点

**难点：LCD 刷新延迟 4 秒的 Bug**  
早期版本中，`lcd_update_pending` 只在主循环最外层检查一次。由于跑马灯一轮需要 3.2 秒，`counter` 虽然每秒递增，但 LCD 上的显示却每 ~4 秒才更新一次。

根本原因：`u_delay(100000)` × 32 次 ≈ 3.2 秒的阻塞，期间 LCD 检查代码完全没有机会执行。

解决方案：将检查语句移入 `for` 循环内部，并用 `App_DelayWithTasks()` 替换 `u_delay()`：

```c
for (int i = 0; i < 16; i++) {
    App_ProcessTickTasks();     // ← 每 100 ms 检查一次，而非每 3.2 秒
    if (lcd_update_pending) {
        lcd_update_pending = 0;
        LCD_UpdateDisplay();
    }
    LEDs_Write(1 << i);
    App_DelayWithTasks(100000); // ← 内部每 1 ms 调用 App_ProcessTickTasks()
}
```

---

## 5. 第5题 – PWM 背光调光（TIM4）

### 5.1 PWM 基本原理

PWM（脉冲宽度调制）通过改变方波的**占空比**（高电平时间占总周期的比例）来控制平均电压，从而调节 LED 亮度：

```
CCR2=200（占空比 20%，低亮度）：
───┐  ┌─────────────────────────────────┐  ┌─
   │  │← 200 →│←          800          →│  │
   └──┘                                 └──┘
   0               ARR=999              0

CCR2=1000（占空比 100%，全亮度）：
───────────────────────────────────────────────
```

**计算公式：**
```
TIM4 时钟 = 84 MHz
计数时钟  = 84 MHz / (PSC+1) = 84 MHz / 560 = 150 kHz
PWM 频率  = 150 kHz / (ARR+1) = 150 kHz / 1000 = 150 Hz
占空比    = CCR2 / (ARR+1) = CCR2 / 1000
```

### 5.2 TIM4 初始化：完整步骤

```c
void Timer4_PWM_Init(void)
{
    // 步骤1：使能 TIM4 时钟（GPIOD 已在 LEDs_InitPorts 中使能）
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // 步骤2：PD13 配置为复用功能模式（MODER = 10b）
    GPIOD->MODER &= ~(3U << (2 * 13));
    GPIOD->MODER |=  (2U << (2 * 13));  // 复用功能模式

    // 步骤3：选择 AF2（TIM4_CH2）
    // AFR[1] 控制 Pin8~15，Pin13 偏移 = (13-8)*4 = 20 位
    GPIOD->AFR[1] &= ~(0xFU << ((13-8)*4));
    GPIOD->AFR[1] |=  (2U   << ((13-8)*4));  // 写入 AF2

    // 步骤4：配置时基
    TIM4->PSC = 559;   // 84MHz / 560 = 150kHz
    TIM4->ARR = 999;   // 150kHz / 1000 = 150Hz

    // 步骤5：CCMR1 配置通道2为 PWM 模式1 + 预装载使能
    TIM4->CCMR1 &= ~(TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE);
    TIM4->CCMR1 |=  (6U << TIM_CCMR1_OC2M_Pos)  // OC2M=110b：PWM模式1
                 |  TIM_CCMR1_OC2PE;             // OC2PE=1：预装载使能

    // 步骤6：写入初始占空比
    TIM4->CCR2 = 200;   // 20% 低亮度
    current_duty = 200;

    // 步骤7：使能通道2输出
    TIM4->CCER |= TIM_CCER_CC2E;  // CC2E=1：通道2输出使能

    // 步骤8：使能 ARR 预装载
    TIM4->CR1 |= TIM_CR1_ARPE;

    // 步骤9：软件触发更新，使配置立即生效
    TIM4->EGR |= TIM_EGR_UG;
    TIM4->SR   = 0;

    // 步骤10：启动定时器
    TIM4->CR1 |= TIM_CR1_CEN;
}
```

### 5.3 复用功能模式（AF）详解

**为什么 PD13 的电平不由 CPU 的 ODR 控制，而由 TIM4 自动驱动？**

GPIO 有四种工作模式：

| MODER 值 | 模式 | 谁控制引脚？ |
|----------|------|------------|
| 00b | 输入 | 外部信号 |
| 01b | 通用输出 | CPU（ODR 寄存器）|
| **10b** | **复用功能（AF）** | **片内外设（TIM4）** |
| 11b | 模拟 | ADC/DAC |

配置为 AF 模式后，PD13 的高/低电平完全由 TIM4 的 PWM 逻辑自动生成，CPU 不需要任何干预——TIM4 会在 `CNT < CCR2` 时自动输出高电平，否则输出低电平。

**AFR 寄存器：选择具体哪个外设**

一个引脚可能对应多个外设（如 PD13 可以连接 TIM4 或 DAC 等），通过 AFR 寄存器中的 4 位字段（AF0~AF15）选择：

```c
// STM32F407 数据手册 Table 9：PD13 AF2 = TIM4_CH2
GPIOD->AFR[1] |= (2U << ((13-8)*4));  // 写入 2 = AF2
```

### 5.4 PWM 模式1 的工作原理

```
CNT 计数过程：    0──────────── CCR2(200) ────────────── ARR(999) ──> 0（溢出）
输出电平：        ← ─ HIGH(亮)─→│←────────── LOW(灭)──────────────→│
```

- 当 `CNT < CCR2` 时：输出高电平（LED 亮）
- 当 `CNT ≥ CCR2` 时：输出低电平（LED 灭）
- 占空比 = CCR2 / 1000，例如 CCR2=200 → 占空比 20% → 平均亮度 20%

### 5.5 OC2PE 预装载的重要性

不启用预装载时，写入 CCR2 的新值立即生效：

```
无预装载（OC2PE=0）：
时刻t：写 CCR2=600
效果：当前 PWM 脉冲可能立即被截断，产生毛刺脉冲

有预装载（OC2PE=1）：
时刻t：写 CCR2=600（暂存在预装载寄存器）
下一个 ARR 溢出：新值生效，过渡平滑
```

### 5.6 实现难点

**难点：为什么不用软件模拟 PWM（Bit-Banging）？**  
软件 PWM 需要定时切换引脚电平，必须在每个 PWM 周期内精确执行两次 GPIO 操作。对于 150 Hz 的 PWM，每 6.67 ms 需要切换一次，但同时还需要运行跑马灯、LCD 更新等逻辑，很容易产生时序冲突。硬件 PWM 完全由 TIM4 自主运行，CPU 的负载为零。

---

## 6. 第6题 – 频率测量（TIM12 输入捕获）

### 6.1 输入捕获工作原理

**核心思想：** 不是产生信号，而是"捕获"外部信号的时间戳。每当 PB14 出现一次上升沿，TIM12 硬件会自动将当前计数器值锁存到 CCR1 寄存器，并触发中断。相邻两次捕获值的差即为信号周期（以 Tick 为单位），频率可由此计算：

```
信号波形：───────┐    ┌────┐    ┌────┐    ┌─────
               │    │    │    │    │    │
               └────┘    └────┘    └────┘
捕获时刻：     T1        T2        T3

周期（Tick数）= T2 - T1 = Δ
频率（Hz）= 84,000,000 / Δ
```

### 6.2 TIM12 硬件配置

```
TIM12 时钟 = 84 MHz
PSC = 0  →  计数时钟 = 84 MHz / (0+1) = 84 MHz → 1 Tick ≈ 11.9 ns
ARR = 0xFFFF = 65535  →  溢出周期 = 65536 / 84,000,000 ≈ 780 µs
```

**PB14 引脚配置（AF9 = TIM12_CH1）：**

```c
void Timer12_Capture_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;

    // PB14：复用功能模式（AF9 = TIM12_CH1）
    GPIOB->MODER &= ~(3U << (2 * 14));
    GPIOB->MODER |=  (2U << (2 * 14));
    GPIOB->AFR[1] &= ~(0xFU << ((14-8)*4));
    GPIOB->AFR[1] |=  (9U   << ((14-8)*4));  // AF9

    // TIM12 时基
    TIM12->CR1  = 0;
    TIM12->PSC  = 0;
    TIM12->ARR  = 0xFFFF;

    // CCMR1：通道1配置为输入捕获
    // CC1S=01：通道1为输入，连接 TI1（PB14）
    // IC1PSC=00（默认）：每个上升沿都捕获
    // IC1F=0000（默认）：无数字滤波器
    TIM12->CCMR1 = (1U << TIM_CCMR1_CC1S_Pos);

    // CCER：上升沿触发，使能捕获
    // CC1E=1：使能通道1捕获
    // CC1P=0（默认）：上升沿触发
    TIM12->CCER = TIM_CCER_CC1E;

    // DIER：使能捕获中断 + 溢出中断
    TIM12->DIER = TIM_DIER_CC1IE | TIM_DIER_UIE;

    // NVIC：优先级 0（最高），高于 TIM7 的优先级 1
    NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 0);
    NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);

    TIM12->EGR |= TIM_EGR_UG;
    TIM12->SR   = 0;
    TIM12->CR1 |= TIM_CR1_CEN;
}
```

### 6.3 虚拟 32 位计数器扩展

TIM12 是 16 位定时器，最大计数值 65535，溢出周期约 780 µs。若被测信号频率低于 1281 Hz，两次上升沿之间计数器可能溢出多次，直接相减会得到错误结果。

**解决方案：用溢出计数器将 16 位扩展为 32 位**

```c
// 全局溢出计数器（ISR 私有）
static volatile uint32_t tim12_overflow = 0;

// 溢出中断：每次溢出将计数器扩展为高 16 位
if (sr & TIM_SR_UIF) {
    TIM12->SR &= ~TIM_SR_UIF;
    tim12_overflow++;
}

// 捕获时：合并为 32 位虚拟时间戳
uint16_t raw16 = (uint16_t)TIM12->CCR1;
uint32_t cap32 = (tim12_overflow << 16) | raw16;
//               ↑ 高 16 位（溢出次数）  ↑ 低 16 位（CCR1）
```

有了 32 位时间戳，理论上可以测量最低约 0.02 Hz 的信号（32 位计数器约 51 秒才溢出一次）。

### 6.4 竞争条件（Race Condition）的处理

当 UIE（溢出）和 CC1IF（捕获）同时发生时，ISR 需要判断：捕获发生在溢出**之前**还是**之后**？

```
情况A：上升沿 → 溢出（正常）
时间轴：  ……[A]……[溢出]……
CCR1 值：较小（刚从 0 开始计数不久）→ CCR1 < 0x8000
overflow 已正确递增，无需修正

情况B：溢出 → 上升沿（需要修正！）
时间轴：  ……[溢出]……[A]……
CCR1 值：较大（接近溢出时捕获）→ CCR1 ≥ 0x8000
overflow 被多加了一次（溢出在先），需要减一补偿
```

**修正代码：**

```c
uint32_t ov = tim12_overflow;
if ((sr & TIM_SR_UIF) && (raw16 >= 0x8000U)) {
    ov--;  // 修正：捕获发生在溢出之前，溢出计数多加了一次
}
uint32_t cap32 = (ov << 16) | raw16;
```

**直觉理解：**  
如果 CCR1 ≥ 0x8000（计数器值处于上半区，接近溢出），说明上升沿发生时计数器已经很大了，紧接着发生了溢出。即上升沿在溢出之前，`tim12_overflow` 被错误地多加了一次，需要补偿。

### 6.5 量化误差与精度优化

**使用 PSC=83（1 MHz 计数时钟）时的误差：**

```
以 78 kHz 信号为例：
周期 = 1 / 78,000 ≈ 12.82 µs
1 MHz 时钟下：12.82 个 Tick → 取整为 12 或 13

取 12：f = 1,000,000 / 12 = 83,333 Hz（误差 +6.8%）
取 13：f = 1,000,000 / 13 = 76,923 Hz（误差 -1.4%）
→ 实测结果总在 76,923 Hz 附近徘徊，与真值 78.12 kHz 偏差显著
```

**使用 PSC=0（84 MHz 计数时钟）时的误差：**

```
同样以 78 kHz 信号：
周期 = 12.82 µs，84 MHz 时钟下：1075.7 个 Tick → 取整为 1075 或 1076

取 1075：f = 84,000,000 / 1075 = 78,140 Hz（误差 +0.026%）
取 1076：f = 84,000,000 / 1076 = 78,066 Hz（误差 -0.069%）
→ 精度提升约 84 倍（Tick 数增加 84 倍，量化误差等比缩小）
```

**量化误差的本质：** 频率计算用的是整数除法，误差来源于 Tick 数的取整。Tick 数越多，取整引入的相对误差越小：

```
误差 ≈ ±1 Tick / 总Tick数 × 100%
1 MHz 下：±1/13 ≈ ±7.7%
84 MHz 下：±1/1075 ≈ ±0.093%
```

### 6.6 完整的 ISR 实现

```c
void TIM8_BRK_TIM12_IRQHandler(void)
{
    // 一次性读取 SR 状态寄存器（快照），避免多次读取时硬件已清除标志
    uint32_t sr = TIM12->SR;

    // ── 处理溢出（UIE）──────────────────────────────────────────────────
    if (sr & TIM_SR_UIF) {
        TIM12->SR &= ~TIM_SR_UIF;  // 清除 UIF 标志（必须手动清除）
        tim12_overflow++;
    }

    // ── 处理捕获（CC1IF）───────────────────────────────────────────────
    if (sr & TIM_SR_CC1IF) {
        TIM12->SR &= ~TIM_SR_CC1IF;

        // CCR1：读取捕获到的 16 位计数器快照
        uint16_t raw16 = (uint16_t)TIM12->CCR1;

        // 竞争条件修正
        uint32_t ov = tim12_overflow;
        if ((sr & TIM_SR_UIF) && (raw16 >= 0x8000U))
            ov--;

        // 合并为 32 位虚拟时间戳
        uint32_t cap32 = (ov << 16) | raw16;

        if (tim12_first) {
            // 第一个上升沿：保存参考值，从第二个沿开始计算
            tim12_last_cap32 = cap32;
            tim12_first      = 0;
        } else {
            uint32_t delta = cap32 - tim12_last_cap32;
            tim12_last_cap32 = cap32;

            if (delta > 0) {
                cap_ticks   = delta;
                cap_freq_hz = 84000000UL / delta;  // 整数除法，无浮点
                cap_last_ms = ms_ticks;
                cap_valid   = 1;

                // LCD 刷新限速：最多 200 ms 更新一次
                static uint32_t last_lcd_req = 0;
                if (ms_ticks - last_lcd_req >= 200) {
                    last_lcd_req       = ms_ticks;
                    lcd_update_pending = 1;
                }
            }
        }
    }
}
```

### 6.7 为何使用 TIM8_BRK_TIM12_IRQn 向量？

在 STM32F407 上，TIM12 **没有独立的中断向量**，它与 TIM8 的刹车（BRK）事件共用 `TIM8_BRK_TIM12_IRQn` 这一个向量。因此中断处理函数必须命名为 `TIM8_BRK_TIM12_IRQHandler`，而不是 `TIM12_IRQHandler`（后者在 STM32F407 上不存在）。进入 ISR 后通过读取 `TIM12->SR` 来区分捕获事件和溢出事件。

### 6.8 实现难点

**难点1：高频信号下 LCD 被频繁刷新**  
78 kHz 的信号每秒产生约 78,000 次捕获中断。若每次中断都设置 `lcd_update_pending = 1`，LCD 更新函数会被以极高频率调用，占满 CPU，其他任何任务都无法执行。

**解决方案：LCD 刷新限速（Rate Limiting）**

```c
static uint32_t last_lcd_req = 0;
if (ms_ticks - last_lcd_req >= 200) {  // 每 200 ms 最多刷新一次
    last_lcd_req       = ms_ticks;
    lcd_update_pending = 1;
}
```

**难点2：超时显示"--"**  
当没有信号输入（或信号消失）时，`cap_valid` 不会被更新。需要通过检测"最近一次有效测量距今的时间"来判断超时：

```c
uint8_t timeout = (!cap_valid) ||
                  ((ms_ticks - cap_last_ms) >= 2000);  // 2 秒无信号 → 超时

if (timeout) {
    LCD_WriteString(10, 55, WHITE, 0xFE00, "Ticks: --        ");
    LCD_WriteString(10, 75, WHITE, 0xFE00, "Freq:  --        ");
}
```

**难点3：竞争条件调试困难**  
Race Condition 是一种间歇性问题，只在特定时序下出现（UIE 和 CC1IF 同时置位）。调试时无法用断点重现，只能通过逻辑分析仪观察 CCR1 值的分布规律，推断边界情况并加以处理。

---

## 7. 模块化代码结构

### 7.1 文件拆分方案

原始的 641 行单文件 `main.c` 按任务拆分为 6 个模块：

| 文件 | 对应任务 | 主要内容 |
|------|----------|---------|
| `leds.c/.h` | 1/2 | `set_output`, `u_delay`, `LEDs_InitPorts`, `LCD_Output16BitWord`, `LEDs_Write` |
| `display.c/.h` | 3 | SSD2119 驱动，Bresenham 直线，中点圆，字符渲染 |
| `main.c` | 4 + 入口 | TIM7 初始化，`App_ProcessTickTasks`，主循环，`LCD_UpdateDisplay` |
| `pwm.c/.h` | 5 | `Timer4_PWM_Init`, `Set_Backlight_Duty`，PWM 状态变量 |
| `freq_measure.c/.h` | 6 | `Timer12_Capture_Init`，`TIM8_BRK_TIM12_IRQHandler`，测量结果变量 |

### 7.2 跨模块数据共享：extern 机制

C 语言中，一个全局变量只能在一个 `.c` 文件中**定义**，在其他文件中通过 `extern` **声明**后使用：

```c
// pwm.c（定义，分配内存）：
volatile uint16_t current_duty = 200;
volatile uint8_t  fade_active  = 0;
volatile uint32_t fade_last    = 0;

// pwm.h（声明，供其他模块使用）：
extern volatile uint16_t current_duty;
extern volatile uint8_t  fade_active;
extern volatile uint32_t fade_last;

// main.c（使用，需 #include "pwm.h"）：
fade_active = 1;     // 直接使用，链接器会找到 pwm.c 中的定义
TIM4->CCR2 = current_duty - FADE_STEP;
```

```c
// freq_measure.c（需要 main.c 的变量，避免循环包含）：
extern volatile uint32_t ms_ticks;          // main.c 中定义
extern volatile uint8_t  lcd_update_pending; // main.c 中定义
```

### 7.3 中断优先级设计

| 定时器 | 中断向量 | 优先级 | 设计理由 |
|--------|----------|--------|----------|
| TIM12 | TIM8_BRK_TIM12_IRQn | **0（最高）** | 捕获时间戳精度最重要，不能被任何中断打断 |
| TIM7 | TIM7_IRQn | 1 | 系统时钟基准，允许被 TIM12 抢占 |

若 TIM12 优先级低于 TIM7，当 TIM7 中断正在执行时，TIM12 的捕获事件会被延迟约 1~几 µs。对于 84 MHz 时钟（1 Tick = 11.9 ns），这意味着可能损失 100 个以上的 Tick，造成严重的频率测量误差。

### 7.4 volatile 关键字的必要性

所有在 ISR 和主循环之间共享的变量都必须声明为 `volatile`：

```c
volatile uint32_t ms_ticks = 0;
volatile uint8_t  lcd_update_pending = 0;
volatile uint32_t cap_freq_hz = 0;
```

**为什么必须用 volatile？**  
编译器可能会将变量"优化"进 CPU 寄存器，主循环中多次读取同一变量时只读一次（缓存优化）。但 ISR 可能在任意时刻修改内存中的变量值，若编译器从寄存器缓存读值，就会读到过时数据。`volatile` 告诉编译器：每次使用该变量时，都必须从内存中重新读取，不允许缓存。

### 7.5 Keil uvprojx 项目文件更新

三个编译目标（Flash、RAM、仿真）均已添加新的源文件：

```xml
<File>
  <FileName>leds.c</FileName>
  <FileType>1</FileType>    <!-- FileType 1 = C 源文件 -->
  <FilePath>.\leds.c</FilePath>
</File>
<File>
  <FileName>pwm.c</FileName>
  <FileType>1</FileType>
  <FilePath>.\pwm.c</FilePath>
</File>
<File>
  <FileName>freq_measure.c</FileName>
  <FileType>1</FileType>
  <FilePath>.\freq_measure.c</FilePath>
</File>
<File>
  <FileName>display.c</FileName>
  <FileType>1</FileType>    <!-- 从 FileType 3（.o）改为 FileType 1（.c）-->
  <FilePath>.\display.c</FilePath>
</File>
```

---

*本文档为 THI 英戈尔施塔特 SoSe 2026 微控制器编程实验（MC-Praktikum）技术总结。*
