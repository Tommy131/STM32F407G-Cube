#ifndef DISPLAY_H
#define DISPLAY_H
#include <inttypes.h>

// Makro zur Farbdefinition in RGB
#define RGB565(r, b, g ) ((((r)&0x1F)<<11)|(((g)&0x3F)<<5)|((b)&0x1F))

// ein paar vordefinierte Farben
#define WHITE		0xFFFF
#define BLACK 	0x0000
#define RED			RGB565(31,0,0)
#define	GREEN		RGB565(0,63,0)
#define BLUE		RGB565(0,0,31)
#define GREY		RGB565(16,32,16)


// Init-Funktion: muss am Anfang aufgerufen werden
void LCD_Init(void);

// Löschen des Displays (in der angegebenen Farbe)
void LCD_ClearDisplay(uint16_t color);

// Ausgabe eines Textes an der vorbezeichneten Stelle am Display
// Parameter:
// x, y:           Position am Bildschirm
// colForeground:  Farbe in der der Text geschrieben wird
// colBackground:  Farbe des Hintergrundes (es werden immer 12x16 Pixel gemalt)
// text:           Zeiger auf einen C String
void LCD_WriteString( uint16_t x, uint16_t y, uint16_t colForeground, uint16_t colBackground, char * text );

// Wichtig: die folgende Funktion muss zur Verfügung gestellt werden 
// (sollte von der LED-Lauflicht-Aufgabe bekannt sein!
extern void LCD_Output16BitWord(uint16_t data);  // bitte entsprechende Funktion bereitstellen

#endif
