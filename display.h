#ifndef DISPLAY_H
#define DISPLAY_H
#include <inttypes.h>

/* Farbdefinition in RGB565 (Achtung: Parameterreihenfolge r, b, g) */
#define RGB565(r, b, g) ((((r)&0x1F)<<11)|(((g)&0x3F)<<5)|((b)&0x1F))

/* Vordefinierte Farben */
#define WHITE  0xFFFF
#define BLACK  0x0000
#define RED    RGB565(31,  0,  0)
#define GREEN  RGB565( 0, 63,  0)
#define BLUE   RGB565( 0,  0, 31)
#define GREY   RGB565(16, 32, 16)

/* ------------------------------------------------------------------ */
/* Low-Level-Schreibfunktionen (8080-Parallelinterface)               */
/* ------------------------------------------------------------------ */

/* Sendet eine Registeradresse (Kommando) an das Display */
void LCD_WriteCommand(uint16_t cmd);

/* Sendet einen Datenwert an das Display */
void LCD_WriteData(uint16_t data);

/* ------------------------------------------------------------------ */
/* Grundlegende Display-Funktionen                                    */
/* ------------------------------------------------------------------ */

/* Initialisiert das Display (muss als erstes aufgerufen werden) */
void LCD_Init(void);

/* Löscht das gesamte Display (füllt es mit der angegebenen Farbe) */
void LCD_ClearDisplay(uint16_t color);

/* Setzt den GRAM-Cursor auf (x, y) für nachfolgende LCD_DrawPixel-Aufrufe */
void LCD_SetCursor(uint32_t x, uint32_t y);

/* Schreibt ein Pixel in der angegebenen Farbe an die aktuelle Cursorposition */
void LCD_DrawPixel(uint16_t color);

/* ------------------------------------------------------------------ */
/* Textausgabe                                                         */
/* ------------------------------------------------------------------ */

/* Gibt einen einzelnen ASCII-Buchstaben (12×16 Pixel) an Position (x, y) aus */
void LCD_WriteLetter(uint16_t x, uint16_t y,
                     uint16_t colForeground, uint16_t colBackground,
                     char c);

/* Gibt einen C-String ab Position (x, y) aus */
void LCD_WriteString(uint16_t x, uint16_t y,
                     uint16_t colForeground, uint16_t colBackground,
                     char *text);

/* ------------------------------------------------------------------ */
/* Hinweis: Diese Funktion muss in main.c bereitgestellt werden!       */
/* Sie legt ein 16-Bit-Datenwort auf die (verteilten) GPIO-Pins.       */
/* ------------------------------------------------------------------ */
extern void LCD_Output16BitWord(uint16_t data);

#endif /* DISPLAY_H */
