/*
 * display.c - LCD-Display-Treiber für ILI9325 (16-bit 8080-Parallelinterface)
 *
 * Aufgabe 3: Display-Ansteuerung für STM32F407 MCPraktikum (THI)
 *
 * -----------------------------------------------------------------------
 * Hardware-Pinbelegung (FSMC-Datenpins als GPIO, Steuerleitungen)
 * -----------------------------------------------------------------------
 * Datenleitungen (16-bit, verteilt auf GPIOD und GPIOE):
 *   LCD-Bit  0 → PD14       LCD-Bit  8 → PE11
 *   LCD-Bit  1 → PD15       LCD-Bit  9 → PE12
 *   LCD-Bit  2 → PD0        LCD-Bit 10 → PE13
 *   LCD-Bit  3 → PD1        LCD-Bit 11 → PE14
 *   LCD-Bit  4 → PE7        LCD-Bit 12 → PE15
 *   LCD-Bit  5 → PE8        LCD-Bit 13 → PD8
 *   LCD-Bit  6 → PE9        LCD-Bit 14 → PD9
 *   LCD-Bit  7 → PE10       LCD-Bit 15 → PD10
 *   (Umsortierung erfolgt über LCD_Output16BitWord in main.c)
 *
 * Steuerleitungen:
 *   PD4  = LCD_RD  (FSMC_NOE, Read-Strobe, aktiv low, normalerweise high)
 *   PD5  = LCD_WR  (FSMC_NWE, Write-Strobe, aktiv low; geteilt mit LED_WR)
 *   PD6  = LCD_CS  (Chip-Select, aktiv low; getrennt von LED_CS=PD7)
 *   PD11 = LCD_DC  (Data/Command: 1=Daten, 0=Kommando; geteilt mit LED_SEL)
 *
 * -----------------------------------------------------------------------
 * Display-Controller: ILI9325 / ILI9320 kompatibel, 240×320 QVGA
 * -----------------------------------------------------------------------
 */

#include "stm32f4xx.h"
#include "display.h"
#include "fonts.h"

/* ==================== Steuerleitungs-Makros ==================== */
/* Jeder Zugriff nur auf das betroffene Bit; andere Bits unverändert. */
#define LCD_CS_LOW()   (GPIOD->ODR &= ~(1U << 6))   /* CS low  → LCD auswählen   */
#define LCD_CS_HIGH()  (GPIOD->ODR |=  (1U << 6))   /* CS high → LCD abwählen    */
#define LCD_DC_CMD()   (GPIOD->ODR &= ~(1U << 11))  /* DC=0    → Kommando-Modus  */
#define LCD_DC_DATA()  (GPIOD->ODR |=  (1U << 11))  /* DC=1    → Daten-Modus     */
#define LCD_WR_LOW()   (GPIOD->ODR &= ~(1U << 5))   /* WR low  → Daten anlegen   */
#define LCD_WR_HIGH()  (GPIOD->ODR |=  (1U << 5))   /* WR high → Daten übernehmen*/
#define LCD_RD_HIGH()  (GPIOD->ODR |=  (1U << 4))   /* RD high → kein Lesevorgang*/

/* ==================== Displaygröße ==================== */
#define LCD_WIDTH   240   /* Pixel horizontal */
#define LCD_HEIGHT  320   /* Pixel vertikal   */

/* ==================== ILI9325 Registerdefinitionen ==================== */
#define ILI9325_ENTRY_MODE  0x0003   /* Scanrichtung und Farbformat          */
#define ILI9325_GRAM_HADDR  0x0020   /* GRAM-X-Adresszähler                  */
#define ILI9325_GRAM_VADDR  0x0021   /* GRAM-Y-Adresszähler                  */
#define ILI9325_GRAM_DATA   0x0022   /* GRAM-Datenzugriff (Pixel schreiben)  */
#define ILI9325_H_WIN_START 0x0050   /* Horizontales Fenster Start           */
#define ILI9325_H_WIN_END   0x0051   /* Horizontales Fenster Ende            */
#define ILI9325_V_WIN_START 0x0052   /* Vertikales Fenster Start             */
#define ILI9325_V_WIN_END   0x0053   /* Vertikales Fenster Ende              */

/* ==================== Helper: Software-Delay ==================== */
static void lcd_delay(volatile uint32_t n)
{
    while (n--);
}

/* ==================== Grundlegende Schreibfunktionen ==================== */

/*
 * LCD_WriteCommand - Sendet ein Kommando (Registeradresse) an das Display.
 * 8080-Bustiming: CS↓, DC=0, Daten anlegen, WR-Puls (↓↑), CS↑
 */
void LCD_WriteCommand(uint16_t cmd)
{
    LCD_DC_CMD();          /* DC = 0: Kommando-Modus                */
    LCD_CS_LOW();          /* CS low: Display auswählen             */
    LCD_Output16BitWord(cmd);
    LCD_WR_LOW();          /* WR low: Daten anlegen                 */
    LCD_WR_HIGH();         /* WR high: Daten übernehmen (steigende Flanke) */
    LCD_CS_HIGH();         /* CS high: Display abwählen             */
}

/*
 * LCD_WriteData - Sendet einen Datenwert an das Display.
 * 8080-Bustiming: CS↓, DC=1, Daten anlegen, WR-Puls (↓↑), CS↑
 */
void LCD_WriteData(uint16_t data)
{
    LCD_DC_DATA();         /* DC = 1: Daten-Modus                   */
    LCD_CS_LOW();          /* CS low: Display auswählen             */
    LCD_Output16BitWord(data);
    LCD_WR_LOW();          /* WR low: Daten anlegen                 */
    LCD_WR_HIGH();         /* WR high: Daten übernehmen             */
    LCD_CS_HIGH();         /* CS high: Display abwählen             */
}

/*
 * LCD_WriteReg - Kombinierter Kommando- und Datenzugriff.
 * Schreibt zunächst die Registeradresse (Kommando), dann den Wert (Daten).
 * Wird sowohl für die Initialisierung als auch für Cursorbewegungen genutzt.
 */
static void LCD_WriteReg(uint16_t reg, uint16_t data)
{
    LCD_WriteCommand(reg);
    LCD_WriteData(data);
}

/* ==================== LCD_SetCursor ==================== */
/*
 * Setzt den GRAM-Cursor auf die Pixelkoordinate (x, y).
 * x: 0 bis LCD_WIDTH-1 (0–239)
 * y: 0 bis LCD_HEIGHT-1 (0–319)
 *
 * Nach dieser Funktion kann mit LCD_DrawPixel ein Pixel geschrieben werden.
 * Der interne Adresszähler inkrementiert nach jedem Schreibvorgang automatisch.
 */
void LCD_SetCursor(uint32_t x, uint32_t y)
{
    LCD_WriteReg(ILI9325_GRAM_HADDR, (uint16_t)x);
    LCD_WriteReg(ILI9325_GRAM_VADDR, (uint16_t)y);
}

/* ==================== LCD_DrawPixel ==================== */
/*
 * Schreibt ein Pixel in der angegebenen Farbe (RGB565) an die aktuelle
 * Cursorposition. Der Cursor wird danach vom Controller automatisch weiter-
 * geschaltet (abhängig von Entry Mode AM=0 → X-Richtung).
 *
 * Vor dem Aufruf muss LCD_SetCursor aufgerufen worden sein.
 */
void LCD_DrawPixel(uint16_t color)
{
    LCD_WriteCommand(ILI9325_GRAM_DATA);
    LCD_WriteData(color);
}

/* ==================== LCD_ClearDisplay ==================== */
/*
 * Füllt das gesamte Display (240×320 = 76.800 Pixel) mit einer Farbe.
 * Für maximale Geschwindigkeit bleibt CS dauerhaft low und DC=1 (Datenmodus)
 * während der gesamten Pixelausgabe.
 */
void LCD_ClearDisplay(uint16_t color)
{
    /* Cursor auf Ursprung setzen, dann GRAM-Schreiben starten */
    LCD_SetCursor(0, 0);
    LCD_WriteCommand(ILI9325_GRAM_DATA);

    /* Optimierter Burst: CS und DC bleiben für alle Pixel konstant */
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
 * Zeichnet einen einzelnen ASCII-Buchstaben (12×16 Pixel) an Position (x, y).
 *
 * Parameter:
 *   x, y          - Pixelkoordinate der linken oberen Ecke des Zeichens
 *   colForeground - Vordergrundfarbe (RGB565) für gesetzte Bits
 *   colBackground - Hintergrundfarbe (RGB565) für nicht gesetzte Bits
 *   c             - ASCII-Zeichen
 *
 * Schriftformat (console_font_12x16):
 *   Jedes Zeichen = 16 Zeilen × 2 Bytes = 32 Bytes
 *   Offset im Array = ASCII-Code × 32
 *   Pro Zeile: 2 Bytes → 16-Bit-Wert, obere 12 Bits = Pixel (Bit15=links)
 */
void LCD_WriteLetter(uint16_t x, uint16_t y,
                     uint16_t colForeground, uint16_t colBackground,
                     char c)
{
    /* Startadresse des Zeichens in der Fonttabelle */
    uint16_t offset = (uint8_t)c * 32;

    for (uint8_t row = 0; row < 16; row++)
    {
        /* Zwei Bytes pro Zeile zu einem 16-Bit-Pixelmuster zusammensetzen */
        uint16_t pixel_row =
            ((uint16_t)console_font_12x16[offset + row * 2]     << 8) |
             (uint16_t)console_font_12x16[offset + row * 2 + 1];

        /* Cursor auf den Zeilenanfang setzen, dann 12 Pixel sequenziell */
        LCD_SetCursor(x, y + row);
        LCD_WriteCommand(ILI9325_GRAM_DATA);

        /* Optimierter Zeilenburst: CS und DC bleiben konstant */
        LCD_DC_DATA();
        LCD_CS_LOW();
        for (uint8_t col = 0; col < 12; col++)
        {
            /* Bit 15 = linkster Pixel, Bit 4 = rechtster Pixel */
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
 * Gibt einen C-String ab Position (x, y) aus.
 * Jeder Buchstabe ist 12 Pixel breit. Der Cursor rückt nach jedem
 * Zeichen um 12 Pixel nach rechts. Kein automatischer Zeilenumbruch.
 */
void LCD_WriteString(uint16_t x, uint16_t y,
                     uint16_t colForeground, uint16_t colBackground,
                     char *text)
{
    while (*text)
    {
        LCD_WriteLetter(x, y, colForeground, colBackground, *text);
        x += 12;   /* Zeichenbreite: 12 Pixel */
        text++;
    }
}

/* ==================== LCD_Init ==================== */
/*
 * Initialisiert das LCD-Display (ILI9325 / ILI9320 kompatibel).
 *
 * Ablauf:
 *  1. GPIO-Steuerleitungen konfigurieren (PD4, PD5, PD6 als Ausgänge)
 *  2. Steuerleitungen in definierten Ruhezustand bringen
 *  3. ILI9325-Registersequenz schreiben (Power, Timing, Gamma, Fenster)
 *  4. Display einschalten (Register 0x0007 = 0x0133)
 *
 * Die Datenpins (GPIOD und GPIOE) werden in LEDs_InitPorts() in main.c
 * konfiguriert und müssen daher hier nicht erneut gesetzt werden.
 */
void LCD_Init(void)
{
    /* ---- GPIO-Takt sicherstellen ---- */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

    /* ---- PD4: LCD_RD als Ausgang, inaktiv high ---- */
    GPIOD->MODER &= ~(3U << (2 * 4));
    GPIOD->MODER |=  (1U << (2 * 4));
    LCD_RD_HIGH();

    /* ---- PD5: LCD_WR (geteilt mit LED_WR) – bereits in LEDs_InitPorts.
     *          Sicherheitshalber nochmals als Ausgang konfigurieren. ---- */
    GPIOD->MODER &= ~(3U << (2 * 5));
    GPIOD->MODER |=  (1U << (2 * 5));
    LCD_WR_HIGH();

    /* ---- PD6: LCD_CS als Ausgang, inaktiv high ---- */
    GPIOD->MODER &= ~(3U << (2 * 6));
    GPIOD->MODER |=  (1U << (2 * 6));
    LCD_CS_HIGH();

    /* ---- PD11: LCD_DC (geteilt mit LED_SEL) – bereits konfiguriert.
     *           Auf Data-Modus setzen. ---- */
    LCD_DC_DATA();

    /* ---- Reset-Pause (Hardware-Reset falls vorhanden, sonst Software-Delay) ---- */
    lcd_delay(500000);

    /* ---- ILI9325 Initialisierungssequenz ---- */

    /* Interne Oscillator-Einstellungen (Herstellerempfehlung) */
    LCD_WriteReg(0x00E3, 0x3008);
    LCD_WriteReg(0x00E7, 0x0012);
    LCD_WriteReg(0x00EF, 0x1231);

    /* Grundkonfiguration */
    LCD_WriteReg(0x0001, 0x0100);  /* Driver Output Control: SS=1           */
    LCD_WriteReg(0x0002, 0x0700);  /* LCD Driving Control: BC=1             */

    /* Entry Mode: BGR=1, ID1=1(Y++), ID0=1(X++), AM=0(X-Richtung)         */
    /* BGR=1 → Hardware tauscht R und B → RGB565 wird korrekt dargestellt   */
    LCD_WriteReg(ILI9325_ENTRY_MODE, 0x1030);

    LCD_WriteReg(0x0004, 0x0000);  /* Resize: kein Scaling                  */
    LCD_WriteReg(0x0008, 0x0207);  /* Display Control 2: front/back porch   */
    LCD_WriteReg(0x0009, 0x0000);
    LCD_WriteReg(0x000A, 0x0000);
    LCD_WriteReg(0x000C, 0x0001);  /* RGB Interface: 16-bit                 */
    LCD_WriteReg(0x000D, 0x0000);
    LCD_WriteReg(0x000F, 0x0000);

    /* Power-Sequenz (laut ILI9325-Applikationshinweis) */
    LCD_WriteReg(0x0010, 0x0000);
    LCD_WriteReg(0x0011, 0x0007);
    LCD_WriteReg(0x0012, 0x0000);
    LCD_WriteReg(0x0013, 0x0000);
    lcd_delay(500000);             /* Spannungen stabilisieren lassen        */

    LCD_WriteReg(0x0010, 0x1590);
    LCD_WriteReg(0x0011, 0x0227);
    lcd_delay(200000);

    LCD_WriteReg(0x0012, 0x009C);
    lcd_delay(200000);

    LCD_WriteReg(0x0013, 0x1900);
    LCD_WriteReg(0x0029, 0x0023);  /* VCOMac                                */
    LCD_WriteReg(0x002B, 0x000E);  /* Frame rate                            */
    lcd_delay(200000);

    /* GRAM-Startadresse */
    LCD_WriteReg(ILI9325_GRAM_HADDR, 0x0000);
    LCD_WriteReg(ILI9325_GRAM_VADDR, 0x0000);
    lcd_delay(200000);

    /* Gamma-Korrektur */
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

    /* Ausgabefenster: gesamtes Display (0–239 horizontal, 0–319 vertikal) */
    LCD_WriteReg(ILI9325_H_WIN_START, 0x0000);
    LCD_WriteReg(ILI9325_H_WIN_END,   LCD_WIDTH  - 1);  /* 0x00EF = 239 */
    LCD_WriteReg(ILI9325_V_WIN_START, 0x0000);
    LCD_WriteReg(ILI9325_V_WIN_END,   LCD_HEIGHT - 1);  /* 0x013F = 319 */

    /* Gate-Scan-Konfiguration */
    LCD_WriteReg(0x0060, 0xA700);
    LCD_WriteReg(0x0061, 0x0001);
    LCD_WriteReg(0x006A, 0x0000);  /* Vertical Scroll: kein Scroll          */

    /* Partial Display: deaktiviert */
    LCD_WriteReg(0x0080, 0x0000);
    LCD_WriteReg(0x0081, 0x0000);
    LCD_WriteReg(0x0082, 0x0000);
    LCD_WriteReg(0x0083, 0x0000);
    LCD_WriteReg(0x0084, 0x0000);
    LCD_WriteReg(0x0085, 0x0000);

    /* Panel Interface */
    LCD_WriteReg(0x0090, 0x0010);
    LCD_WriteReg(0x0092, 0x0600);

    /* Display einschalten (D1=1 D0=1 REV=1 GON=1 = 0x0133) */
    LCD_WriteReg(0x0007, 0x0133);
    lcd_delay(200000);
}
