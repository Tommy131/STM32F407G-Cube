/*
 * display.c - SSD2119 LCD 显示驱动（16位 8080 并行接口）
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
 * 控制线（按 Praktikum 文档）：
 *   PD4  = LCD_RD    （低有效，写模式下始终保持高）
 *   PD5  = LCD_WR    （低有效写脉冲）
 *   PD3  = LCD_RESET （低有效，初始化前给低脉冲）
 *   PD7  = LCD_CS    （低有效）
 *   PE3  = LCD_DC    （1=数据，0=命令）
 *   PD11 = LCD_CS_EN （与 PD7 进行 OR 以生成 LCD CS，访问 LCD 时需置 0）
 *
 * -----------------------------------------------------------------------
 * 显示控制器：SSD2119，320×240 QVGA
 * -----------------------------------------------------------------------
 */

#include "stm32f4xx.h"
#include "display.h"
#include "fonts.h"

/* ==================== 控制线宏定义 ==================== */
/* 每次操作只影响对应位，其余位保持不变。 */
#define LCD_CS_LOW() (GPIOD->ODR &= ~(1U << 7))     /* CS 低  → 选中 LCD   */
#define LCD_CS_HIGH() (GPIOD->ODR |= (1U << 7))     /* CS 高  → 取消选中   */
#define LCD_DC_CMD() (GPIOE->ODR &= ~(1U << 3))     /* DC=0   → 命令模式   */
#define LCD_DC_DATA() (GPIOE->ODR |= (1U << 3))     /* DC=1   → 数据模式   */
#define LCD_WR_LOW() (GPIOD->ODR &= ~(1U << 5))     /* WR 低  → 建立数据   */
#define LCD_WR_HIGH() (GPIOD->ODR |= (1U << 5))     /* WR 高  → 锁存数据   */
#define LCD_RD_HIGH() (GPIOD->ODR |= (1U << 4))     /* RD 高  → 不执行读取 */
#define LCD_RST_LOW() (GPIOD->ODR &= ~(1U << 3))    /* RESET 低            */
#define LCD_RST_HIGH() (GPIOD->ODR |= (1U << 3))    /* RESET 高            */
#define LCD_CS_ENABLE() (GPIOD->ODR &= ~(1U << 11)) /* 使能 LCD CS OR 门 */
#define LCD_CS_DISABLE() (GPIOD->ODR |= (1U << 11)) /* 关闭 LCD CS OR 门 */

/* ==================== 显示屏尺寸 ==================== */
#define LCD_WIDTH 320  /* 水平像素数 */
#define LCD_HEIGHT 240 /* 垂直像素数 */

/* ==================== SSD2119 寄存器定义 ==================== */
#define SSD2119_RAM_DATA_REG 0x0022
#define SSD2119_X_RAM_ADDR_REG 0x004E
#define SSD2119_Y_RAM_ADDR_REG 0x004F

/* ==================== 软件延时 ==================== */
static void lcd_delay(volatile uint32_t n)
{
    while (n--)
        ;
}

/* ==================== 基础写入函数 ==================== */

/*
 * LCD_WriteCommand - 向显示屏发送一条命令（寄存器地址）。
 * 8080 总线时序：CS↓，DC=0，建立数据，WR 脉冲（↓↑），CS↑
 */
void LCD_WriteCommand(uint16_t cmd)
{
    LCD_CS_ENABLE();
    LCD_DC_CMD(); /* DC = 0：命令模式                  */
    LCD_CS_LOW(); /* CS 低：选中显示屏                 */
    LCD_Output16BitWord(cmd);
    LCD_WR_LOW();  /* WR 低：建立数据                   */
    LCD_WR_HIGH(); /* WR 高：锁存数据（上升沿）         */
    LCD_CS_HIGH(); /* CS 高：取消选中显示屏             */
}

/*
 * LCD_WriteData - 向显示屏发送一个数据值。
 * 8080 总线时序：CS↓，DC=1，建立数据，WR 脉冲（↓↑），CS↑
 */
void LCD_WriteData(uint16_t data)
{
    LCD_CS_ENABLE();
    LCD_DC_DATA(); /* DC = 1：数据模式                  */
    LCD_CS_LOW();  /* CS 低：选中显示屏                 */
    LCD_Output16BitWord(data);
    LCD_WR_LOW();  /* WR 低：建立数据                   */
    LCD_WR_HIGH(); /* WR 高：锁存数据                   */
    LCD_CS_HIGH(); /* CS 高：取消选中显示屏             */
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
 * x：0 至 LCD_WIDTH-1（0~319）
 * y：0 至 LCD_HEIGHT-1（0~239）
 *
 * 调用后可用 LCD_DrawPixel 写入像素。
 * 控制器在每次写入后自动递增内部地址计数器。
 */
void LCD_SetCursor(uint32_t x, uint32_t y)
{
    LCD_WriteReg(SSD2119_X_RAM_ADDR_REG, (uint16_t)x);
    LCD_WriteReg(SSD2119_Y_RAM_ADDR_REG, (uint16_t)y);
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
    LCD_WriteCommand(SSD2119_RAM_DATA_REG);
    LCD_WriteData(color);
}

/* ==================== LCD_ClearDisplay ==================== */
/*
 * 用指定颜色填充整个屏幕（320×240 = 76800 像素）。
 * 为提升速度，整个像素输出期间 CS 持续保持低电平，DC 保持数据模式。
 */
void LCD_ClearDisplay(uint16_t color)
{
    /* 光标归零，启动 GRAM 写入 */
    LCD_SetCursor(0, 0);
    LCD_WriteCommand(SSD2119_RAM_DATA_REG);

    /* 批量传输优化：所有像素期间 CS 和 DC 保持不变 */
    LCD_CS_ENABLE();
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
            ((uint16_t)console_font_12x16[offset + row * 2] << 8) |
            (uint16_t)console_font_12x16[offset + row * 2 + 1];

        /* 将光标移至行首，然后依次输出 12 个像素 */
        LCD_SetCursor(x, y + row);
        LCD_WriteCommand(SSD2119_RAM_DATA_REG);

        /* 行内批量传输优化：CS 和 DC 保持不变 */
        LCD_CS_ENABLE();
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
        x += 12; /* 字符宽度：12 像素 */
        text++;
    }
}

/* ==================== Aufgabe 3 测试实现 ==================== */
static void A3_DrawPixelXY(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT)
        return;
    LCD_SetCursor(x, y);
    LCD_DrawPixel(color);
}

/* Bresenham 直线算法（整数实现） */
static void A3_DrawLine(int x0, int y0, int x1, int y1, uint16_t color)
{
    int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int sx = (x0 < x1) ? 1 : -1;
    int dy = -((y1 > y0) ? (y1 - y0) : (y0 - y1));
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (1)
    {
        if (x0 >= 0 && x0 < LCD_WIDTH && y0 >= 0 && y0 < LCD_HEIGHT)
            A3_DrawPixelXY((uint16_t)x0, (uint16_t)y0, color);

        if (x0 == x1 && y0 == y1)
            break;

        int e2 = 2 * err;
        if (e2 >= dy)
        {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

static void A3_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (w < 2 || h < 2)
        return;
    A3_DrawLine(x, y, x + w - 1, y, color);
    A3_DrawLine(x, y + h - 1, x + w - 1, y + h - 1, color);
    A3_DrawLine(x, y, x, y + h - 1, color);
    A3_DrawLine(x + w - 1, y, x + w - 1, y + h - 1, color);
}

/* Midpoint circle algorithm（整数实现） */
static void A3_DrawCircle(uint16_t cx, uint16_t cy, uint16_t r, uint16_t color)
{
    int x = (int)r;
    int y = 0;
    int err = 1 - x;

    while (x >= y)
    {
        A3_DrawPixelXY((uint16_t)(cx + x), (uint16_t)(cy + y), color);
        A3_DrawPixelXY((uint16_t)(cx + y), (uint16_t)(cy + x), color);
        A3_DrawPixelXY((uint16_t)(cx - y), (uint16_t)(cy + x), color);
        A3_DrawPixelXY((uint16_t)(cx - x), (uint16_t)(cy + y), color);
        A3_DrawPixelXY((uint16_t)(cx - x), (uint16_t)(cy - y), color);
        A3_DrawPixelXY((uint16_t)(cx - y), (uint16_t)(cy - x), color);
        A3_DrawPixelXY((uint16_t)(cx + y), (uint16_t)(cy - x), color);
        A3_DrawPixelXY((uint16_t)(cx + x), (uint16_t)(cy - y), color);

        y++;
        if (err < 0)
        {
            err += 2 * y + 1;
        }
        else
        {
            x--;
            err += 2 * (y - x) + 1;
        }
    }
}

static void A3_RunBasicTest(void)
{
    LCD_ClearDisplay(BLACK);

    /* 四角像素测试（验证坐标与颜色写入） */
    LCD_SetCursor(0, 0);
    LCD_DrawPixel(RED);
    LCD_SetCursor(LCD_WIDTH - 1, 0);
    LCD_DrawPixel(GREEN);
    LCD_SetCursor(0, LCD_HEIGHT - 1);
    LCD_DrawPixel(BLUE);
    LCD_SetCursor(LCD_WIDTH - 1, LCD_HEIGHT - 1);
    LCD_DrawPixel(WHITE);

    LCD_WriteString(20, 20, WHITE, BLACK, "Aufgabe 3: Pixel OK");
    LCD_WriteString(20, 40, GREEN, BLACK, "Text Ausgabe OK");

    /* 文本显示完成后先暂停，按下 USER 键再进入图形绘制页面 */
    LCD_WriteString(10, 210, WHITE, BLACK, "Press USER key for graphics");
    while (GPIOA->IDR & 1U)
    {
    }
    while (!(GPIOA->IDR & 1U))
    {
    }
    lcd_delay(150000);
    while (GPIOA->IDR & 1U)
    {
    }

    LCD_ClearDisplay(BLACK);
    LCD_WriteString(10, 10, WHITE, BLACK, "Complex Graphics Demo");

    for (uint16_t x = 0; x < LCD_WIDTH; x += 20)
        A3_DrawLine(x, 0, x, LCD_HEIGHT - 1, GREY);
    for (uint16_t y = 0; y < LCD_HEIGHT; y += 20)
        A3_DrawLine(0, y, LCD_WIDTH - 1, y, GREY);

    A3_DrawLine(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1, RED);
    A3_DrawLine(LCD_WIDTH - 1, 0, 0, LCD_HEIGHT - 1, BLUE);
    A3_DrawRect(30, 30, 260, 180, GREEN);
    A3_DrawRect(55, 50, 210, 140, WHITE);
    A3_DrawCircle(LCD_WIDTH / 2, LCD_HEIGHT / 2, 70, 0xFFE0); /* Yellow */
    A3_DrawCircle(LCD_WIDTH / 2, LCD_HEIGHT / 2, 40, 0xF81F); /* Magenta */
}

static void A3_WaitForUserKey(void)
{
    LCD_WriteString(10, 210, WHITE, BLACK, "Press USER key to continue");

    /* 等待按键从松开到按下，再等待释放（简易消抖） */
    while (GPIOA->IDR & 1U)
    {
    }
    while (!(GPIOA->IDR & 1U))
    {
    }
    lcd_delay(150000);
    while (GPIOA->IDR & 1U)
    {
    }
}

static void A3_RunEfficiencyTest(void)
{
    uint16_t colorbg = 0xF800; /* Rot */
    uint16_t colorfg = 0x07E0; /* Gruen */

    while (1)
    {
        GPIOD->ODR ^= (1U << 12); /* toggle Pin PD12 */
        LCD_ClearDisplay(colorbg);
        LCD_WriteString(20, 220, colorfg, colorbg, "Hallo Welt!");
        LCD_WriteString(20, 200, colorfg, colorbg, "Press USER to exit");

        uint16_t tmp = colorfg;
        colorfg = colorbg;
        colorbg = (uint16_t)~tmp;

        /* 按下 USER 键后退出效率测试，继续后续程序流程 */
        if (GPIOA->IDR & 1U)
        {
            lcd_delay(150000);
            while (GPIOA->IDR & 1U)
            {
            }
            break;
        }
    }
}

void Aufgabe3_RunCombinedTest(void)
{
    A3_RunBasicTest();
    A3_WaitForUserKey();
    A3_RunEfficiencyTest();
}

/* ==================== LCD_Init ==================== */
/*
 * 初始化 LCD 显示屏（SSD2119）。
 *
 * 步骤：
 *  1. 配置 GPIO 控制线（PD3、PD4、PD5、PD7、PD11、PE3 设为输出）
 *  2. 将控制线置于确定的空闲状态
 *  3. 写入 SSD2119 寄存器初始化序列
 *  4. 开启显示（寄存器 0x0007 = 0x0033）
 *
 * 数据引脚（GPIOD 和 GPIOE）已在 main.c 的 LEDs_InitPorts() 中配置，
 * 此处无需重复设置。
 */
void LCD_Init(void)
{
    /* ---- 确保 GPIOD/GPIOE 时钟已使能 ---- */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;

    /* ---- PD3：LCD_RESET 输出 ---- */
    GPIOD->MODER &= ~(3U << (2 * 3));
    GPIOD->MODER |= (1U << (2 * 3));
    LCD_RST_HIGH();

    /* ---- PD4：LCD_RD 配置为输出，初始高电平（不读取） ---- */
    GPIOD->MODER &= ~(3U << (2 * 4));
    GPIOD->MODER |= (1U << (2 * 4));
    LCD_RD_HIGH();

    /* ---- PD5：LCD_WR（与 LED_WR 共用）——已在 LEDs_InitPorts 中配置。
     *          此处再次配置为输出，确保状态正确。 ---- */
    GPIOD->MODER &= ~(3U << (2 * 5));
    GPIOD->MODER |= (1U << (2 * 5));
    LCD_WR_HIGH();

    /* ---- PD7：LCD_CS 配置为输出，初始高电平（不选中） ---- */
    GPIOD->MODER &= ~(3U << (2 * 7));
    GPIOD->MODER |= (1U << (2 * 7));
    LCD_CS_HIGH();

    /* ---- PD11：LCD_CS_EN（与 LED_SEL 共用）输出 ---- */
    GPIOD->MODER &= ~(3U << (2 * 11));
    GPIOD->MODER |= (1U << (2 * 11));
    LCD_CS_DISABLE();

    /* ---- PE3：LCD_DC 输出 ---- */
    GPIOE->MODER &= ~(3U << (2 * 3));
    GPIOE->MODER |= (1U << (2 * 3));
    LCD_DC_DATA();

    /* ---- 复位脉冲：低电平至少 15us ---- */
    LCD_RST_LOW();
    lcd_delay(12000);
    LCD_RST_HIGH();
    lcd_delay(12000);

    /* 访问 LCD 前先使能 OR 门（PD11=0） */
    LCD_CS_ENABLE();

    /* ---- SSD2119 初始化序列（来自 Praktikum 文档） ---- */
    LCD_WriteReg(0x0010, 0x0001);
    LCD_WriteReg(0x001E, 0x00B2);
    LCD_WriteReg(0x0028, 0x0006);
    LCD_WriteReg(0x0000, 0x0001);
    LCD_WriteReg(0x0001, 0x72EF);
    LCD_WriteReg(0x0002, 0x0600);
    LCD_WriteReg(0x0010, 0x0000);
    lcd_delay(3000000);
    LCD_WriteReg(0x0011, 0x6870);
    LCD_WriteReg(0x0012, 0x0999);
    LCD_WriteReg(0x0026, 0x3800);
    LCD_WriteReg(0x0007, 0x0033);
    LCD_WriteReg(0x000C, 0x0005);
    LCD_WriteReg(0x000D, 0x000A);
    LCD_WriteReg(0x000E, 0x2E00);

    /* 全屏窗口与初始光标 */
    LCD_WriteReg(0x0044, (uint16_t)((LCD_HEIGHT - 1U) << 8)); /* 0..239 */
    LCD_WriteReg(0x0045, 0x0000);
    LCD_WriteReg(0x0046, (uint16_t)(LCD_WIDTH - 1U)); /* 0..319 */
    LCD_WriteReg(SSD2119_X_RAM_ADDR_REG, 0x0000);
    LCD_WriteReg(SSD2119_Y_RAM_ADDR_REG, 0x0000);
}
