# STM32F407 Discovery FreeRTOS Demo

THI 课程 *Integrationsplattform — Automotive Funktionen*（WiSe 2025/26）的实践工程。目标板是 STM32F407G-DISC1，芯片 STM32F407VGT6。程序用 STM32CubeIDE 生成，操作系统是 FreeRTOS，应用代码通过 CMSIS-RTOS2 调用它。

各题的实现按文件拆开。`main.c` 只做硬件初始化和启动调度器。CubeMX 重新生成代码时，写在 `USER CODE BEGIN` / `USER CODE END` 之间的调用会保留。

## 硬件

- 板卡：STM32F407G-DISC1
- 调试器：板载 ST-LINK
- 晶振：8 MHz HSE
- 系统时钟：168 MHz
- 液晶屏：SSD2119，320×240，16 位 8080 并行接口
- 电位器：接到 PA1。板上没有现成电位器，不接时读数会停在某个附近

时钟树：

```text
8 MHz / 8 × 336 / 2 = 168 MHz 系统时钟
APB1 = 168 / 4 = 42 MHz，定时器时钟再乘 2 = 84 MHz
APB2 = 168 / 2 = 84 MHz
```

HAL 时基是 TIM6。FreeRTOS 节拍是 1000 Hz，所以 `osDelay(1)` 约等于 1 ms。

## 构建和下载

1. 用 STM32CubeIDE 打开本目录，工程名是 `demo`。
2. 选择 Debug 配置，执行 Project → Build Project。
3. 用板载 ST-LINK 下载并调试。

`Debug/` 里的 makefile 是 CubeIDE 生成的。新增 `Core/Src` 下的 `.c` 文件后，在 IDE 里重新编译一次，IDE 会把新文件收进编译列表。

应用代码放在 `Core/Src/a2_*.c` 和 `Core/Src/a3.c`。不要把作业逻辑直接写进 CubeMX 生成函数的生成区。

## 目录

| 路径 | 内容 |
|---|---|
| `Core/Src/main.c` | 时钟、外设初始化、启动各题 |
| `Core/Src/a2_1_1.c` | 按住 User 键时翻转绿色灯 |
| `Core/Src/a2_1_2.c` | 100 ms 软件定时器 |
| `Core/Src/a2_1_3.c` | TIM7 的 1 ms 中断 |
| `Core/Src/a2_1_5.c` | PD13 的 PWM 正弦调光 |
| `Core/Src/a2_2.c` | 生产者 / 消费者求平均 |
| `Core/Src/a2_3.c` | 液晶数据线初始化和问候语 |
| `Core/Src/a3.c` | 加分题：五个通信任务 |
| `Core/Src/display.c` | SSD2119 驱动 |
| `Core/Inc/fonts.h` | 12×16 ASCII 字模 |
| `demo.ioc` | CubeMX 配置 |

## 启动顺序

`main()` 按这个顺序启动：

1. `HAL_Init()`、系统时钟、GPIO、TIM7、TIM4、RNG。
2. `A2_3_Init()`：配置液晶数据线，初始化屏幕，清成红色并写出 `Hallo Kurs!`。这一步在 `osKernelStart()` 之前。
3. `osKernelInitialize()`。
4. 创建软件定时器、PWM 调光任务、A2.2 的两个任务、A3 的五个任务。
5. 启动 TIM7 中断。
6. `osKernelStart()`。调度器接管后，`defaultTask` 再初始化 USB Host，并进入按键闪灯循环。

## 引脚

| 引脚 | 功能 |
|---|---|
| PA0 | User 键，按下为高电平 |
| PA1 | 电位器，ADC1 通道 1 |
| PD12 | 绿色灯 LD4。按住 User 键时翻转 |
| PD13 | TIM4 通道 2 PWM。橙色灯，也是背光 |
| PD14 | 红色灯 LD5，同时是液晶数据位 0 |
| PD15 | 蓝色灯 LD6，同时是液晶数据位 1 |
| PD0、PD1、PD8–PD10 | 液晶数据位 |
| PE7–PE15 | 液晶数据位 |
| PD3 | 液晶复位，低有效 |
| PD4 | 液晶读使能，写模式下保持高 |
| PD5 | 液晶写使能，低有效 |
| PD7 | 液晶片选，低有效 |
| PD11 | 液晶片选使能，访问屏幕时为低 |
| PE3 | 液晶数据/命令线。1 为数据，0 为命令 |

PD13 不能用 TIM7 做 PWM。TIM6 和 TIM7 是基本定时器，没有输出通道。PD13 的定时器复用是 TIM4 通道 2，复用号 `GPIO_AF2_TIM4`。

## 题目对照

### A2.1.1 绿色灯

文件：`a2_1_1.c`。

`defaultTask` 调用 `A2_1_1_Task()`。按住 PA0 时，任务延时约 100 ms 后翻转 PD12，松开后熄灭。循环末尾还有 `osDelay(10)`，否则这个高优先级任务会一直占住 CPU，软件定时器服务任务得不到运行。

### A2.1.2 软件定时器

文件：`a2_1_2.c`。

`osTimerNew()` 创建周期定时器，`osTimerStart(..., 100)` 启动。回调翻转 PD14。回调跑在定时器服务任务里，不跑在 SysTick 里。

### A2.1.3 硬件定时器

文件：`a2_1_3.c`。

TIM7 输入时钟 84 MHz。预分频 83，自动重装 999，中断周期 1 ms。`HAL_TIM_Base_Start_IT()` 启动中断。`HAL_TIM_PeriodElapsedCallback()` 里若实例是 TIM7，就翻转 PD15。TIM6 的回调仍负责 `HAL_IncTick()`，不要改掉。

### A2.1.5 背光 PWM

文件：`a2_1_5.c`。

TIM4 通道 2 输出 PWM，载波约 1.28 kHz（预分频 0，周期 65535）。`dimmTask` 每 20 ms 改一次比较值。亮度按 `(1 - cos(2πt / 2s)) / 2` 变化：第 1 秒变亮，第 2 秒变暗。

### A2.2 任务通信求平均

文件：`a2_2.c`。

两个同优先级任务：

- Producer 依次发送 `1` 到 `10`，最后发送 `0`。
- Consumer 累加这些整数。`0` 不加进总和。收到 `0` 后把整数平均值送回。`1` 到 `10` 的结果是 `5`。

`numberQueue` 深度 16，一条消息是一个 `int32_t`。`averageQueue` 深度 1，只回传平均值。`emptySlots` 和 `filledSlots` 是计数信号量，和 `numberQueue` 的 16 个位置对应。发送前先占空位，取出后再还空位。调试器里观察 `lastAverage`。

### A2.3 液晶输出

文件：`a2_3.c`、`display.c`、`fonts.h`。

`A2_3_Init()` 把液晶数据线配成推挽输出，调用 `LCD_Init()`，清成红色，并在 (10, 10) 写出 `Hallo Kurs!`。16 位数据总线的位重排在 `display.c` 的 `LCD_Output16BitWord()`。字模只含 ASCII，不能直接写中文。

### A3 加分题

文件：`a3.c`。

五个任务在调度器启动前由 `A3_Init()` 创建：

| 任务 | 作用 |
|---|---|
| `buttonTask` | 每 20 ms 读 PA0。状态变化时把 0 或 1 放进 `displayQueue`，并释放 `buttonSignal` |
| `randomTask` | 每秒用 `HAL_RNG_GenerateRandomNumber()` 产生一个 32 位随机数，放进 `randomQueue` |
| `evalTask` | 每收满 5 个随机数，把整数平均值放进 `displayQueue` |
| `potTask` | 每 200 ms 读 PA1 的 12 位 ADC 值，放进 `displayQueue` |
| `displayTask` | 取出消息，持有 `lcdMutex` 后写屏 |

屏幕上的三行：

- y = 40：`Button: pressed` 或 `Button: released`
- y = 64：`Average: ` 加上 5 个数的平均值
- y = 88：`Pot: ` 加上 0 到 4095 的 ADC 值

`displayQueue` 的消息带类型字段，深度 8。`randomQueue` 只连接随机数任务和求平均任务。画屏前获取 `lcdMutex`，因为液晶驱动不能被两个任务同时调用。`buttonSignal` 是计数信号量，每个按键变化释放一个令牌，显示任务处理按键消息时取走它。

## 标签

| 标签 | 内容 |
|---|---|
| `A2.1.3` | 软件定时器和 TIM7 不再抢同一根脚 |
| `A2.1.5` | PD13 的 PWM 正弦调光 |
| `A2.2` | 生产者 / 消费者求平均，前面的题目拆到独立文件 |
| `A2.3` | 液晶初始化并显示问候语 |
| `A3` | 五个任务把按键、平均值和电位器显示到屏幕上 |

这些标签都签了名。仓库配置了 `commit.gpgsign`。本机 Git 若找不到 `gpg.exe`，提交时要把程序指到 `C:\Program Files\Git\usr\bin\gpg.exe`。

## 已知限制

- PD14、PD15 同时被灯任务和液晶数据线使用。`a2_1_2.c` 每 100 ms 翻转 PD14，`a2_1_3.c` 每 1 ms 翻转 PD15。写屏时这两处翻转会把数据线打乱。要看稳定画面，先停掉这两处翻转。
- 工程没有带入 `stm32f4xx_hal_adc.c`。电位器在 `a3.c` 里直接写 ADC1 寄存器，通道固定为 PA1。
- `display.c` 里的 `A3_RunBasicTest()` 是空函数。上学期的图形测试没有随驱动导入。`Aufgabe3_RunCombinedTest()` 目前不会画出那些图形。
- A2.2 的平均值是整数除法。A3 的平均值是 5 个 32 位随机数的整数平均，数值会很大。
- USB Host 由 CubeMX 生成并在 `defaultTask` 里初始化。本作业不使用网络，也不依赖 U 盘。
