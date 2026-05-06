/*
 * display.c - ILI9325 LCD 显示驱动（16位 8080 并行接口）
 *
 * 第3题：STM32F407 MCPraktikum（THI）显示屏驱动
 *
 * -----------------------------------------------------------------------
 * 硬件引脚分配（FSMC 数据引脚复用为 GPIO，加控制线）
 * -----------------------------------------------------------------------
 * 数据线（16位，分布于 GPIOD 和 GPIOE）：
 *   LCD位  0 → PD14       LCD位  8 → PE11
 *   LCD位  1 → PD15       LCD位  9 → PE12
 *   LCD位  2 → PD0        LCD位 10 → PE13
 *   LCD位  3 → PD1        LCD位 11 → PE14
 *   LCD位  4 → PE7        LCD位 12 → PE15
 *   LCD位  5 → PE8        LCD位 13 → PD8
 *   LCD位  6 → PE9        LCD位 14 → PD9
 *   LCD位  7 → PE10       LCD位 15 → PD10
 *   （位重排由 main.c 中的 LCD_Output16BitWord 完成）
 *
 * 控制线：
 *   PD4  = LCD_RD  （FSMC_NOE，读选通，低有效，平时保持高电平）
 *   PD5  = LCD_WR  （FSMC_NWE，写选通，低有效；与 LED_WR 共用）
 *   PD6  = LCD_CS  （片选，低有效；与 LED_CS=PD7 分离）
 *   PD11 = LCD_DC  （数据/命令：1=数据，0=命令；与 LED_SEL 共用）
 *
 * -----------------------------------------------------------------------
 * 显示控制器：ILI9325 / ILI9320 兼容，240×320 QVGA
 * -----------------------------------------------------------------------
 */

#include "stm32f4xx.h"
#include "display.h"
#include "fonts.h"

/* ==================== 控制线宏定义 ==================== */
/* 每次操作只影响对应位，其余位保持不变。 */
#define LCD_CS_LOW()   (GPIOD->ODR &= ~(1U << 6))   /* CS 低  → 选中 LCD   */
#define LCD_CS_HIGH()  (GPIOD->ODR |=  (1U << 6))   /* CS 高  → 取消选中   */
#define LCD_DC_CMD()   (GPIOD->ODR &= ~(1U << 11))  /* DC=0   → 命令模式   */
#define LCD_DC_DATA()  (GPIOD->ODR |=  (1U << 11))  /* DC=1   → 数据模式   */
#define LCD_WR_LOW()   (GPIOD->ODR &= ~(1U << 5))   /* WR 低  → 建立数据   */
#define LCD_WR_HIGH()  (GPIOD->ODR |=  (1U << 5))   /* WR 高  → 锁存数据   */
#define LCD_RD_HIGH()  (GPIOD->ODR |=  (1U << 4))   /* RD 高  → 不执行读取 */

/* ==================== 显示屏尺寸 ==================== */
#define LCD_WIDTH   240   /* 水平像素数 */
#define LCD_HEIGHT  320   /* 垂直像素数 */

/* ==================== ILI9325 寄存器定义 ==================== */
#define ILI9325_ENTRY_MODE  0x0003   /* 扫描方向与颜色格式         */
#define ILI9325_GRAM_HADDR  0x0020   /* GRAM X 地址计数器          */
#define ILI9325_GRAM_VADDR  0x0021   /* GRAM Y 地址计数器          */
#define ILI9325_GRAM_DATA   0x0022   /* GRAM 数据访问（写像素）     */
#define ILI9325_H_WIN_START 0x0050   /* 水平窗口起始地址           */
#define ILI9325_H_WIN_END   0x0051   /* 水平窗口结束地址           */
#define ILI9325_V_WIN_START 0x0052   /* 垂直窗口起始地址           */
#define ILI9325_V_WIN_END   0x0053   /* 垂直窗口结束地址           */

/* ==================== 软件延时 ==================== */
static void lcd_delay(volatile uint32_t n)
{
    while (n--);
}

/* ==================== 基础写入函数 ==================== */

/*
 * LCD_WriteCommand - 向显示屏发送一条命令（寄存器地址）。
 * 8080 总线时序：CS↓，DC=0，建立数据，WR 脉冲（↓↑），CS↑
 */
void LCD_WriteCommand(uint16_t cmd)
{
    LCD_DC_CMD();          /* DC = 0：命令模式                  */
    LCD_CS_LOW();          /* CS 低：选中显示屏                 */
    LCD_Output16BitWord(cmd);
    LCD_WR_LOW();          /* WR 低：建立数据                   */
    LCD_WR_HIGH();         /* WR 高：锁存数据（上升沿）         */
    LCD_CS_HIGH();         /* CS 高：取消选中显示屏             */
}

/*
 * LCD_WriteData - 向显示屏发送一个数据值。
 * 8080 总线时序：CS↓，DC=1，建立数据，WR 脉冲（↓↑），CS↑
 */
void LCD_WriteData(uint16_t data)
{
    LCD_DC_DATA();         /* DC = 1：数据模式                  */
    LCD_CS_LOW();          /* CS 低：选中显示屏                 */
    LCD_Output16BitWord(data);
    LCD_WR_LOW();          /* WR 低：建立数据                   */
    LCD_WR_HIGH();         /* WR 高：锁存数据                   */
    LCD_CS_HIGH();         /* CS 高：取消选中显示屏             */
}

/*
 * LCD_WriteReg - 组合命令与数据的寄存器写入。
 * 先写寄存器地址（命令），再写寄存器值（数据）。
 * 用于初始化序列及光标移动。
 */
static void LCD_WriteReg(uint16_t reg, uint16_t data)
{
    LCD_WriteCommand(reg);
    LCD_WriteData(data);
}

/* ==================== LCD_SetCursor ==================== */
/*
 * 将 GRAM 光标定位到像素坐标 (x, y)。
 * x：0 至 LCD_WIDTH-1（0~239）
 * y：0 至 LCD_HEIGHT-1（0~319）
 *
 * 调用后可用 LCD_DrawPixel 写入像素。
 * 控制器在每次写入后自动递增内部地址计数器。
 */
void LCD_SetCursor(uint32_t x, uint32_t y)
{
    LCD_WriteReg(ILI9325_GRAM_HADDR, (uint16_t)x);
    LCD_WriteReg(ILI9325_GRAM_VADDR, (uint16_t)y);
}

/* ==================== LCD_DrawPixel ==================== */
/*
 * 在当前光标位置写入一个指定颜色（RGB565）的像素。
 * 写入后控制器自动移动光标（取决于 Entry Mode AM=0 → X 方向）。
 *
 * 调用前必须先调用 LCD_SetCursor。
 */
void LCD_DrawPixel(uint16_t color)
{
    LCD_WriteCommand(ILI9325_GRAM_DATA);
    LCD_WriteData(color);
}

/* ==================== LCD_ClearDisplay ==================== */
/*
 * 用指定颜色填充整个屏幕（240×320 = 76800 像素）。
 * 为提升速度，整个像素输出期间 CS 持续保持低电平，DC 保持数据模式。
 */
void LCD_ClearDisplay(uint16_t color)
{
    /* 光标归零，启动 GRAM 写入 */
    LCD_SetCursor(0, 0);
    LCD_WriteCommand(ILI9325_GRAM_DATA);

    /* 批量传输优化：所有像素期间 CS 和 DC 保持不变 */
    LCD_DC_DATA();
    LCD_CS_LOW();
    for (uint32_t i = 0; i < (uint32_t)LCD_WIDTH * LCD_HEIGHT; i++)
    {
        LCD_Output16BitWord(color);
        LCD_WR_LOW();
        LCD_WR_HIGH();
    }
    LCD_CS_HIGH();
}

/* ==================== LCD_WriteLetter ==================== */
/*
 * 在位置 (x, y) 绘制一个 ASCII 字符（12×16 像素）。
 *
 * 参数：
 *   x, y          - 字符左上角的像素坐标
 *   colForeground - 前景色（RGB565），对应置位的像素
 *   colBackground - 背景色（RGB565），对应未置位的像素
 *   c             - ASCII 字符
 *
 * 字体格式（console_font_12x16）：
 *   每个字符 = 16 行 × 2 字节 = 32 字节
 *   数组偏移 = ASCII 码 × 32
 *   每行 2 字节 → 16 位值，高 12 位为像素（Bit15=最左侧）
 */
void LCD_WriteLetter(uint16_t x, uint16_t y,
                     uint16_t colForeground, uint16_t colBackground,
                     char c)
{
    /* 字体表中该字符的起始偏移 */
    uint16_t offset = (uint8_t)c * 32;

    for (uint8_t row = 0; row < 16; row++)
    {
        /* 将每行的两个字节拼合为 16 位像素掩码 */
        uint16_t pixel_row =
            ((uint16_t)console_font_12x16[offset + row * 2]     << 8) |
             (uint16_t)console_font_12x16[offset + row * 2 + 1];

        /* 将光标移至行首，然后依次输出 12 个像素 */
        LCD_SetCursor(x, y + row);
        LCD_WriteCommand(ILI9325_GRAM_DATA);

        /* 行内批量传输优化：CS 和 DC 保持不变 */
        LCD_DC_DATA();
        LCD_CS_LOW();
        for (uint8_t col = 0; col < 12; col++)
        {
            /* Bit15 = 最左侧像素，Bit4 = 最右侧像素 */
            uint16_t color = (pixel_row & (0x8000U >> col))
                             ? colForeground
                             : colBackground;
            LCD_Output16BitWord(color);
            LCD_WR_LOW();
            LCD_WR_HIGH();
        }
        LCD_CS_HIGH();
    }
}

/* ==================== LCD_WriteString ==================== */
/*
 * 从位置 (x, y) 开始输出一个 C 字符串。
 * 每个字符宽 12 像素，输出后光标向右移动 12 像素。
 * 不支持自动换行。
 */
void LCD_WriteString(uint16_t x, uint16_t y,
                     uint16_t colForeground, uint16_t colBackground,
                     char *text)
{
    while (*text)
    {
        LCD_WriteLetter(x, y, colForeground, colBackground, *text);
        x += 12;   /* 字符宽度：12 像素 */
        text++;
    }
}

/* ==================== LCD_Init ==================== */
/*
 * 初始化 LCD 显示屏（兼容 ILI9325 / ILI9320）。
 *
 * 步骤：
 *  1. 配置 GPIO 控制线（PD4、PD5、PD6 设为输出）
 *  2. 将控制线置于确定的空闲状态
 *  3. 写入 ILI9325 寄存器初始化序列（电源、时序、Gamma、窗口）
 *  4. 开启显示（寄存器 0x0007 = 0x0133）
 *
 * 数据引脚（GPIOD 和 GPIOE）已在 main.c 的 LEDs_InitPorts() 中配置，
 * 此处无需重复设置。
 */
void LCD_Init(void)
{
    /* ---- 确保 GPIOD 时钟已使能 ---- */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

    /* ---- PD4：LCD_RD 配置为输出，初始高电平（不读取） ---- */
    GPIOD->MODER &= ~(3U << (2 * 4));
    GPIOD->MODER |=  (1U << (2 * 4));
    LCD_RD_HIGH();

    /* ---- PD5：LCD_WR（与 LED_WR 共用）——已在 LEDs_InitPorts 中配置。
     *          此处再次配置为输出，确保状态正确。 ---- */
    GPIOD->MODER &= ~(3U << (2 * 5));
    GPIOD->MODER |=  (1U << (2 * 5));
    LCD_WR_HIGH();

    /* ---- PD6：LCD_CS 配置为输出，初始高电平（不选中） ---- */
    GPIOD->MODER &= ~(3U << (2 * 6));
    GPIOD->MODER |=  (1U << (2 * 6));
    LCD_CS_HIGH();

    /* ---- PD11：LCD_DC（与 LED_SEL 共用）——已配置。
     *           设为数据模式。 ---- */
    LCD_DC_DATA();

    /* ---- 复位等待（若有硬件复位则等待复位完成，否则软件延时） ---- */
    lcd_delay(500000);

    /* ---- ILI9325 初始化序列 ---- */

    /* 内部振荡器设置（按厂商推荐值） */
    LCD_WriteReg(0x00E3, 0x3008);
    LCD_WriteReg(0x00E7, 0x0012);
    LCD_WriteReg(0x00EF, 0x1231);

    /* 基本配置 */
    LCD_WriteReg(0x0001, 0x0100);  /* 驱动输出控制：SS=1                    */
    LCD_WriteReg(0x0002, 0x0700);  /* LCD 驱动控制：BC=1                    */

    /* 扫描模式：BGR=1，ID1=1（Y++），ID0=1（X++），AM=0（X 方向）          */
    /* BGR=1 → 硬件交换 R 与 B → RGB565 颜色正确显示                       */
    LCD_WriteReg(ILI9325_ENTRY_MODE, 0x1030);

    LCD_WriteReg(0x0004, 0x0000);  /* 缩放：不缩放                          */
    LCD_WriteReg(0x0008, 0x0207);  /* 显示控制2：前/后沿时序                */
    LCD_WriteReg(0x0009, 0x0000);
    LCD_WriteReg(0x000A, 0x0000);
    LCD_WriteReg(0x000C, 0x0001);  /* RGB 接口：16位                        */
    LCD_WriteReg(0x000D, 0x0000);
    LCD_WriteReg(0x000F, 0x0000);

    /* 上电序列（按 ILI9325 应用说明） */
    LCD_WriteReg(0x0010, 0x0000);
    LCD_WriteReg(0x0011, 0x0007);
    LCD_WriteReg(0x0012, 0x0000);
    LCD_WriteReg(0x0013, 0x0000);
    lcd_delay(500000);             /* 等待各路电源稳定                       */

    LCD_WriteReg(0x0010, 0x1590);
    LCD_WriteReg(0x0011, 0x0227);
    lcd_delay(200000);

    LCD_WriteReg(0x0012, 0x009C);
    lcd_delay(200000);

    LCD_WriteReg(0x0013, 0x1900);
    LCD_WriteReg(0x0029, 0x0023);  /* VCOMac                                */
    LCD_WriteReg(0x002B, 0x000E);  /* 帧率                                  */
    lcd_delay(200000);

    /* GRAM 起始地址 */
    LCD_WriteReg(ILI9325_GRAM_HADDR, 0x0000);
    LCD_WriteReg(ILI9325_GRAM_VADDR, 0x0000);
    lcd_delay(200000);

    /* Gamma 校正 */
    LCD_WriteReg(0x0030, 0x0007);
    LCD_WriteReg(0x0031, 0x0707);
    LCD_WriteReg(0x0032, 0x0006);
    LCD_WriteReg(0x0035, 0x0704);
    LCD_WriteReg(0x0036, 0x1F04);
    LCD_WriteReg(0x0037, 0x0004);
    LCD_WriteReg(0x0038, 0x0000);
    LCD_WriteReg(0x0039, 0x0706);
    LCD_WriteReg(0x003C, 0x0701);
    LCD_WriteReg(0x003D, 0x000F);

    /* 显示窗口：整个屏幕（水平 0~239，垂直 0~319） */
    LCD_WriteReg(ILI9325_H_WIN_START, 0x0000);
    LCD_WriteReg(ILI9325_H_WIN_END,   LCD_WIDTH  - 1);  /* 0x00EF = 239 */
    LCD_WriteReg(ILI9325_V_WIN_START, 0x0000);
    LCD_WriteReg(ILI9325_V_WIN_END,   LCD_HEIGHT - 1);  /* 0x013F = 319 */

    /* 栅极扫描配置 */
    LCD_WriteReg(0x0060, 0xA700);
    LCD_WriteReg(0x0061, 0x0001);
    LCD_WriteReg(0x006A, 0x0000);  /* 垂直滚动：不滚动                      */

    /* 局部显示：关闭 */
    LCD_WriteReg(0x0080, 0x0000);
    LCD_WriteReg(0x0081, 0x0000);
    LCD_WriteReg(0x0082, 0x0000);
    LCD_WriteReg(0x0083, 0x0000);
    LCD_WriteReg(0x0084, 0x0000);
    LCD_WriteReg(0x0085, 0x0000);

    /* 面板接口设置 */
    LCD_WriteReg(0x0090, 0x0010);
    LCD_WriteReg(0x0092, 0x0600);

    /* 开启显示（D1=1 D0=1 REV=1 GON=1 = 0x0133） */
    LCD_WriteReg(0x0007, 0x0133);
    lcd_delay(200000);
}
