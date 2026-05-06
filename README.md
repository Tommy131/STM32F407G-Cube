# STM32F407 Mikrocontroller-Praktikum – Technische Dokumentation

**Hochschule:** Technische Hochschule Ingolstadt (THI)  
**Modul:** Informatik 2 – Mikrocontroller-Programmierung  
**Zielplattform:** STM32F407VGTx Discovery Board (Cortex-M4, 168 MHz)  
**Entwicklungsumgebung:** Keil MDK, ARM Clang V6.24  

---

## Inhaltsverzeichnis

1. [Gesamtkonzept und Architektur](#1-gesamtkonzept-und-architektur)
2. [Aufgabe 1/2 – LED-Lauflicht (GPIO-Steuerung)](#2-aufgabe-12--led-lauflicht-gpio-steuerung)
3. [Aufgabe 3 – LCD-Anzeige (SSD2119-Treiber)](#3-aufgabe-3--lcd-anzeige-ssd2119-treiber)
4. [Aufgabe 4 – Tastenerkennung und Zeitgeber (TIM7)](#4-aufgabe-4--tastenerkennung-und-zeitgeber-tim7)
5. [Aufgabe 5 – PWM-Hintergrundbeleuchtung (TIM4)](#5-aufgabe-5--pwm-hintergrundbeleuchtung-tim4)
6. [Aufgabe 6 – Frequenzmessung (TIM12 Input Capture)](#6-aufgabe-6--frequenzmessung-tim12-input-capture)
7. [Modulare Codestruktur](#7-modulare-codestruktur)

---

## 1. Gesamtkonzept und Architektur

### 1.1 Grundlegende Entwurfsprinzipien

Das Projekt wurde schrittweise nach dem Baukastenprinzip entwickelt. Jede neue Aufgabe baut auf der vorherigen auf, ohne bestehende Funktionalität zu beeinträchtigen. Dabei galten folgende Leitlinien:

**Trennung von Interrupt-Kontext und Hauptschleife:**  
Zeitkritische Operationen (Zeitstempel erfassen, Zähler inkrementieren) werden im Interrupt Service Routine (ISR) erledigt. Alle rechenintensiven Operationen (LCD-Ausgabe, Frequenzberechnung, printf) laufen in der Hauptschleife, um die ISR kurz zu halten und Prioritätsinversionen zu vermeiden.

**Zeitscheibenbasierte Verzögerung:**  
Anstatt mit `u_delay(3.200.000)` den Prozessor für 3,2 Sekunden zu blockieren, wird die Verzögerung in 1-ms-Scheiben aufgeteilt. Nach jeder Scheibe werden alle zeitkritischen Aufgaben abgearbeitet:

```c
void App_DelayWithTasks(uint32_t total_us)
{
    const uint32_t slice_us = 1000;  // 1 ms pro Scheibe
    while (total_us >= slice_us) {
        u_delay(slice_us);
        App_ProcessTickTasks();   // Tasten, LED, Countdown, Fade
        total_us -= slice_us;
    }
    if (total_us > 0) {
        u_delay(total_us);
        App_ProcessTickTasks();
    }
}
```

**Polling-basierte LCD-Aktualisierung:**  
Das Flag `lcd_update_pending` wird von ISRs oder anderen Modulen gesetzt. Die Hauptschleife prüft dieses Flag bei jedem Schleifendurchlauf und führt die eigentliche LCD-Aktualisierung durch:

```c
for (int i = 0; i < 16; i++) {
    App_ProcessTickTasks();
    if (lcd_update_pending) {
        lcd_update_pending = 0;
        LCD_UpdateDisplay();     // Nur hier, nie im ISR
    }
    LEDs_Write(1 << i);
    App_DelayWithTasks(100000);
}
```

### 1.2 Taktverteilung des STM32F407

Das Verständnis der Taktverteilung ist fundamental für alle Timer-Konfigurationen:

```
HSE/HSI (12 MHz)
    │
    └─ PLL → SYSCLK = 168 MHz
                │
                ├─ AHB1 (÷1) = 168 MHz → GPIO, DMA
                │
                └─ APB1 (÷4) = 42 MHz
                        │
                        └─ APB1-Timer (×2) = 84 MHz → TIM4, TIM7, TIM12
```

**Wichtig:** APB1-Peripheriegeräte laufen mit 42 MHz, aber APB1-Timer bekommen den doppelten Takt (84 MHz), weil der Prescaler ≠ 1 ist. Dieser Faktor muss bei allen PSC-Berechnungen berücksichtigt werden.

### 1.3 Dateistruktur

```
mcpr-jic4873/
├── main.c           # Aufgabe 4 + Einstiegspunkt + LCD_UpdateDisplay
├── leds.c/.h        # Aufgabe 1/2: GPIO, LED-Steuerung
├── pwm.c/.h         # Aufgabe 5: TIM4 PWM-Hintergrundbeleuchtung
├── freq_measure.c/.h# Aufgabe 6: TIM12 Input Capture
├── display.c/.h     # Aufgabe 3: SSD2119 LCD-Treiber
└── MC_Praktikum.uvprojx
```

---

## 2. Aufgabe 1/2 – LED-Lauflicht (GPIO-Steuerung)

### 2.1 Hardwarekonzept: Gemeinsamer Datenbus

Die 16 LEDs sind **nicht** direkt an je einen GPIO-Pin angeschlossen. Stattdessen teilen sich LEDs und LCD-Display einen gemeinsamen 16-Bit-Parallelbus (GPIOD + GPIOE). Ein Schieberegister (74HC595 oder äquivalent) trennt die beiden Geräte. Durch das **SEL**-Signal wählt man aus, wer die Daten erhält.

```
CPU
 │
 ├─ GPIOD/GPIOE (16 Datenleitungen) ──────┬──── LCD (wenn SEL=0)
 │                                        └──── Schieberegister (wenn SEL=1)
 │                                                    │
 ├─ PD5  (WR)   ── Schreibtakt                        └─ 16 LEDs
 ├─ PD7  (CS)   ── Chip-Select (Low = aktiv)
 └─ PD11 (SEL)  ── Geräteauswahl (1=LEDs, 0=LCD)
```

### 2.2 Bit-Remapping: Das zentrale Problem

Die 16 Datenleitungen sind auf GPIOD und GPIOE verteilt, aber **nicht** in linearer Reihenfolge. Deshalb muss `LCD_Output16BitWord()` das 16-Bit-Datenwort auf verschiedene GPIO-Bitfelder aufteilen:

| Datenbits | Ziel-GPIO |
|-----------|-----------|
| `data[1:0]` | `GPIOD[1:0]` (PD1, PD0) |
| `data[4:2]` | `GPIOD[10:8]` (PD10–PD8) |
| `data[15:13]` | `GPIOD[15:14]` (PD15–PD14) |
| `data[12:4]` | `GPIOE[15:7]` (PE15–PE7) |

```c
void LCD_Output16BitWord(uint16_t data)
{
    uint16_t d = 0, e = 0;

    d |= ((data & 0x3) << 14);       // data[1:0]   → GPIOD[15:14]
    d |= ((data >> 2) & 0x3);        // data[3:2]   → GPIOD[1:0]
    d |= ((data >> 13) & 0x7) << 8;  // data[15:13] → GPIOD[10:8]
    e |= ((data >> 4) & 0x1FF) << 7; // data[12:4]  → GPIOE[15:7]

    GPIOD->ODR &= ~((0x3 << 14) | 0x3 | (0x7 << 8)); // Zielbits löschen
    GPIOE->ODR &= ~(0x1FF << 7);

    GPIOD->ODR |= d;  // neue Werte setzen
    GPIOE->ODR |= e;
}
```

Das Read-Modify-Write-Muster (`&= ~mask` dann `|= value`) ist zwingend erforderlich, damit andere GPIO-Pins (z.B. PD12 = LED, PD13 = PWM) nicht verändert werden.

### 2.3 Schreibprotokoll (Timing-Sequenz)

Das Schieberegister erwartet folgendes Protokoll:

```
SEL  ──────── 1 ──────────────────────────────  1
CS   ──── 0 ──────────────────────────────  1 ────
DATA    (stabil auf Bus gehalten)
WR   ──── 1 ─── 0 ─── 1 ──────────────────────────
              ↑ fallende Flanke lädt Daten ins SR
```

```c
void LEDs_Write(uint16_t data)
{
    GPIOD->ODR |=  (1 << LED_SEL_PIN);   // SEL=1: LED-Bus wählen
    GPIOD->ODR &= ~(1 << LED_CS_PIN);    // CS=0:  Schieberegister aktivieren
    LCD_Output16BitWord(data);            // Daten anlegen
    GPIOD->ODR |=  (1 << LED_WR_PIN);    // WR=1 (vorbereiten)
    GPIOD->ODR &= ~(1 << LED_WR_PIN);    // WR=0 (fallende Flanke)
    GPIOD->ODR |=  (1 << LED_WR_PIN);    // WR=1 (steigende Flanke → Übernahme)
    GPIOD->ODR |=  (1 << LED_CS_PIN);    // CS=1: Bus freigeben
}
```

### 2.4 Lauflicht-Algorithmus

**Vorwärts (einzelne LED wandert):**
```c
for (int i = 0; i < 16; i++) {
    LEDs_Write(1 << i);          // Bit i gesetzt → LED i an
    App_DelayWithTasks(100000);  // 100 ms warten
}
```

**Rückwärts (LEDs nacheinander löschen):**
```c
for (int i = 0; i < 16; i++) {
    LEDs_Write(~(uint16_t)(1 << i));  // alle LEDs außer i
    App_DelayWithTasks(100000);
}
```

### 2.5 GPIO-Initialisierung: MODER-Register

Jeder GPIO-Pin wird durch 2 Bits im MODER-Register konfiguriert:

```c
void set_output(GPIO_TypeDef *port, int pin)
{
    port->MODER &= ~(3 << (2 * pin)); // Bits 2n+1:2n löschen
    port->MODER |=  (1 << (2 * pin)); // 01b = Ausgang
}
```

### 2.6 Implementierungsherausforderungen

**Problem 1: Read-Modify-Write bei GPIO-ODR**  
Da mehrere Pins gleichzeitig in ODR leben (PD12 = LED, PD13 = PWM, PD5 = WR, etc.), darf niemals das gesamte ODR überschrieben werden. Nur die gewünschten Bits dürfen geändert werden.

**Problem 2: Taktversatz durch langen Lauflicht-Zyklus**  
Ein vollständiger Umlauf dauert 32 × 100 ms = 3,2 Sekunden. Ohne die `App_DelayWithTasks()`-Funktion würde in dieser Zeit keine Tastenentprellung, kein LED-Blinken und kein LCD-Update stattfinden.

---

## 3. Aufgabe 3 – LCD-Anzeige (SSD2119-Treiber)

### 3.1 Hardware-Interface: 8080-Parallelbus

Das SSD2119-Display wird über ein Intel-8080-kompatibles Parallelinterface angesteuert. Dieses Interface verwendet:

| Signal | Pin | Bedeutung |
|--------|-----|-----------|
| CS | PD7 | Chip-Select (Low = aktiv) |
| DC | PE3 | 0 = Kommando, 1 = Daten |
| WR | PD5 | Schreib-Takt (fallende Flanke) |
| RD | PD4 | Lese-Takt (hier nicht genutzt) |
| RST | PD3 | Hardware-Reset (Low-Puls) |
| D[15:0] | GPIOD/E | Datenbus |

**Grundlegendes Schreibprotokoll:**
```
CS  ──── 0 ───────────────────── 1 ────
DC  ──── (0=CMD / 1=DATA) ──────────────
DATA    (16 Bit auf Bus)
WR  ──── 0 ─── 1 ──────────────────────
              ↑ steigende Flanke = Daten gültig
```

```c
void LCD_WriteCommand(uint16_t cmd)
{
    LCD_CS_ENABLE();         // PD11=0 (OR-Gate für CS)
    LCD_DC_CMD();            // PE3=0: Kommando-Modus
    LCD_CS_LOW();            // PD7=0: Chip auswählen
    LCD_Output16BitWord(cmd);
    LCD_WR_LOW();            // PD5=0: Daten anlegen
    LCD_WR_HIGH();           // PD5=1: steigende Flanke → Übernahme
    LCD_CS_HIGH();           // PD7=1: Chip deselektieren
}
```

### 3.2 RGB565-Farbformat

Das Display verwendet das RGB565-Format: 5 Bits Rot, 6 Bits Grün, 5 Bits Blau in einem 16-Bit-Wort:

```
Bit: 15 14 13 12 11 | 10  9  8  7  6  5 |  4  3  2  1  0
     R4 R3 R2 R1 R0 | G5 G4 G3 G2 G1 G0 | B4 B3 B2 B1 B0
```

```c
#define RGB565(r, b, g) ((((r)&0x1F)<<11)|(((g)&0x3F)<<5)|((b)&0x1F))
#define WHITE  0xFFFF
#define BLACK  0x0000
#define RED    RGB565(31,  0,  0)
```

**Achtung:** Die Parameterreihenfolge ist `(r, b, g)` – die Blau- und Grün-Parameter sind vertauscht, was die hardwareseitige Bitanordnung des Displays widerspiegelt.

### 3.3 GRAM-Cursor und Pixelausgabe

Der Grafikcontroller verfügt über einen internen Speicher (GRAM). Um einen Pixel zu schreiben, muss man zuerst die X/Y-Position setzen, dann den Datenwert senden:

```c
void LCD_SetCursor(uint32_t x, uint32_t y)
{
    LCD_WriteReg(SSD2119_X_RAM_ADDR_REG, (uint16_t)x);
    LCD_WriteReg(SSD2119_Y_RAM_ADDR_REG, (uint16_t)y);
}

void LCD_DrawPixel(uint16_t color)
{
    LCD_WriteCommand(SSD2119_RAM_DATA_REG);
    LCD_WriteData(color);
}
```

**Performance-Optimierung bei `LCD_ClearDisplay`:**  
Anstatt für jeden der 76.800 Pixel (320×240) CS und DC zu toggeln, werden diese Signale während der gesamten Übertragung auf konstantem Pegel gehalten:

```c
void LCD_ClearDisplay(uint16_t color)
{
    LCD_SetCursor(0, 0);
    LCD_WriteCommand(SSD2119_RAM_DATA_REG);
    LCD_CS_ENABLE();
    LCD_DC_DATA();
    LCD_CS_LOW();
    for (uint32_t i = 0; i < (uint32_t)320 * 240; i++) {
        LCD_Output16BitWord(color);
        LCD_WR_LOW();
        LCD_WR_HIGH();
    }
    LCD_CS_HIGH();
}
```

### 3.4 Zeichendarstellung: 12×16-Pixel-Font

Jeder ASCII-Charakter wird als 12×16-Pixel-Bitmap aus einer vordefinierten Fonttabelle `console_font_12x16[]` gelesen:

```c
void LCD_WriteLetter(uint16_t x, uint16_t y,
                     uint16_t colFG, uint16_t colBG, char c)
{
    uint16_t offset = (uint8_t)c * 32; // 32 Bytes pro Zeichen (16 Zeilen × 2 Bytes)

    for (uint8_t row = 0; row < 16; row++) {
        uint16_t pixel_row =
            ((uint16_t)console_font_12x16[offset + row * 2] << 8) |
            (uint16_t)console_font_12x16[offset + row * 2 + 1];

        LCD_SetCursor(x, y + row);
        LCD_WriteCommand(SSD2119_RAM_DATA_REG);

        LCD_CS_ENABLE(); LCD_DC_DATA(); LCD_CS_LOW(); // einmal pro Zeile
        for (uint8_t col = 0; col < 12; col++) {
            uint16_t color = (pixel_row & (0x8000U >> col)) ? colFG : colBG;
            LCD_Output16BitWord(color);
            LCD_WR_LOW();
            LCD_WR_HIGH();
        }
        LCD_CS_HIGH();
    }
}
```

Das Bit 15 (MSB) jeder Zeile entspricht dem linkesten Pixel. Mit `0x8000U >> col` wird pro Spalte ein Schiebemuster erzeugt.

### 3.5 Grafik-Algorithmen (Aufgabe 3 Zusatz)

**Bresenham-Geradenz ug (Integer-Arithmetik):**  
Der Bresenham-Algorithmus zeichnet Linien ausschließlich mit ganzzahligen Operationen (kein Sinus/Cosinus, kein Floating-Point):

```c
static void A3_DrawLine(int x0, int y0, int x1, int y1, uint16_t color)
{
    int dx =  abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (1) {
        A3_DrawPixelXY(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}
```

**Midpoint-Kreisalgorithmus:**  
Nutzt die 8-fache Symmetrie eines Kreises – für jeden berechneten Punkt werden 8 symmetrische Punkte gezeichnet:

```c
static void A3_DrawCircle(uint16_t cx, uint16_t cy, uint16_t r, uint16_t color)
{
    int x = r, y = 0, err = 1 - x;
    while (x >= y) {
        // 8 Oktanten gleichzeitig zeichnen
        A3_DrawPixelXY(cx+x, cy+y, color); A3_DrawPixelXY(cx+y, cy+x, color);
        A3_DrawPixelXY(cx-y, cy+x, color); A3_DrawPixelXY(cx-x, cy+y, color);
        A3_DrawPixelXY(cx-x, cy-y, color); A3_DrawPixelXY(cx-y, cy-x, color);
        A3_DrawPixelXY(cx+y, cy-x, color); A3_DrawPixelXY(cx+x, cy-y, color);
        y++;
        err += (err < 0) ? (2*y + 1) : (2*(y - --x) + 1);
    }
}
```

### 3.6 Implementierungsherausforderungen

**Problem 1: Gemeinsamer Datenbus mit LEDs**  
`LCD_Output16BitWord()` wird sowohl für den LCD- als auch für den LED-Bus verwendet. Das SEL-Signal entscheidet, wer die Daten empfängt.

**Problem 2: Langsame Bildschirmaktualisierung**  
`LCD_UpdateDisplay()` schreibt vier Textzeilen, jede bestehend aus mehreren `LCD_WriteLetter()`-Aufrufen. Jeder Buchstabe erfordert 16 × 12 = 192 Pixelschreibvorgänge. Diese Funktion darf daher **niemals im ISR** aufgerufen werden.

**Problem 3: Überschreiben alter Ziffern**  
Wenn eine Zahl von "1234" auf "99" schrumpft, bleiben die alten Ziffern sichtbar. Lösung: Links-ausgerichtete Format-Strings mit Leerzeichen-Padding:

```c
sprintf(buf, "Freq:  %-7" PRIu32 " Hz", cap_freq_hz);
// Ausgabe: "Freq:  78140   Hz"  (mit Leerzeichen am Ende)
```

---

## 4. Aufgabe 4 – Tastenerkennung und Zeitgeber (TIM7)

### 4.1 TIM7-Konfiguration (1 ms Interrupt)

TIM7 erzeugt einen präzisen 1-ms-Interrupt als Systemtakt:

```
TIM7-Takt = 84 MHz
PSC = 8399  →  Zähltakt = 84 MHz / 8400 = 10 kHz
ARR = 9     →  Interrupt = 10 kHz / 10 = 1 kHz → Periode 1 ms
```

```c
void Timer7_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7->CR1  = 0;
    TIM7->PSC  = 8399;
    TIM7->ARR  = 9;
    TIM7->CR1  |= TIM_CR1_ARPE | TIM_CR1_URS;
    // ARPE: ARR-Preload aktivieren (kein Zählsprung bei Änderung)
    // URS:  Nur Überlauf löst Update-IRQ aus (kein UG-Trigger)
    TIM7->EGR  |= TIM_EGR_UG;  // PSC/ARR sofort laden
    TIM7->SR    = 0;            // UIF-Flag löschen
    TIM7->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM7_IRQn, 1);
    NVIC_EnableIRQ(TIM7_IRQn);
    TIM7->CR1  |= TIM_CR1_CEN;
}
```

**Wichtig: CR1.URS-Bit**  
Ohne URS=1 würde das Schreiben von EGR.UG=1 ebenfalls einen Update-Interrupt auslösen, obwohl der Zähler noch gar nicht gestartet hat. URS=1 filtert diesen Sonderfall heraus.

### 4.2 ISR-Minimalismus

Der TIM7-ISR macht nur das Nötigste – er inkrementiert den Millisekunden-Zähler:

```c
void TIM7_IRQHandler(void)
{
    if (!(TIM7->SR & TIM_SR_UIF)) return; // falschen Interrupt ablehnen
    TIM7->SR &= ~TIM_SR_UIF;             // UIF-Flag manuell löschen (PFLICHT!)
    ms_ticks++;
}
```

**Warum das UIF-Flag manuell löschen?**  
Im Gegensatz zu manchen anderen Peripheriegeräten löscht der STM32-Timer das UIF-Flag **nicht automatisch** beim Eintritt in die ISR. Bleibt es gesetzt, wird sofort erneut ein Interrupt ausgelöst – der Prozessor hängt in einer Endlosschleife innerhalb der ISR.

### 4.3 Tastenentprellung und Flankenerkennung

Mechanische Taster prellen beim Drücken und Loslassen mehrere Millisekunden. Daher wird der Tastenzustand nur alle 50 ms abgetastet:

```c
if (ms_ticks - key_check_last >= 50) {
    key_check_last = ms_ticks;
    uint8_t key_now = (GPIOA->IDR & 1) != 0; // PA0 lesen

    if (key_now && !key_last)  { /* steigende Flanke: Taste gedrückt */  }
    if (!key_now && key_last)  { /* fallende Flanke: Taste losgelassen */ }

    key_last = key_now;
}
```

**Flankenerkennung durch Vorwert-Vergleich:**  
`key_last` speichert den Zustand vom letzten Abtastzeitpunkt. Ein Übergang `0→1` zeigt das Drücken, ein Übergang `1→0` das Loslassen an.

### 4.4 10-Sekunden-Countdown mit Hintergrundbeleuchtungs-Fading

```
Taste gedrückt  → Helligkeit sofort 100%, Countdown gestoppt
Taste losgelassen → release_time = ms_ticks, countdown_active = 1
10 Sekunden      → countdown_active = 0, fade_active = 1
Fading: alle 10 ms, CCR2 -= 4 bis CCR2 = 200 (ca. 2 Sekunden)
```

```c
if (countdown_active) {
    uint32_t elapsed = ms_ticks - release_time;
    if (elapsed >= 10000) {
        countdown_active = 0;
        fade_active = 1;
        fade_last   = ms_ticks;
    }
}

if (fade_active) {
    if (ms_ticks - fade_last >= 10) {  // alle 10 ms
        fade_last = ms_ticks;
        if (current_duty > 200 + 4) {
            current_duty -= 4;
            TIM4->CCR2 = current_duty;
        } else {
            current_duty = 200;
            TIM4->CCR2 = 200;
            fade_active = 0;
        }
    }
}
```

### 4.5 PD12-LED Präzisionsblinkung (1 Hz)

Während die Taste gedrückt ist, blinkt PD12 genau mit 1 Hz (500 ms ein, 500 ms aus):

```c
if (button_pressed) {
    if (ms_ticks - led_toggle_last >= 500) {
        led_toggle_last = ms_ticks;
        led_state ^= 1;
        if (led_state) GPIOD->ODR |=  (1 << 12);
        else           GPIOD->ODR &= ~(1 << 12);
    }
}
```

### 4.6 Implementierungsherausforderungen

**Problem: LCD-Aktualisierungsverzögerung (4-Sekunden-Bug)**  
Ursprünglich wurde `lcd_update_pending` nur einmal pro Hauptschleifen-Iteration geprüft. Da ein Umlauf des Lauflichts 3,2 Sekunden dauert, wurde der Zähler auf dem Display nur alle ~4 Sekunden aktualisiert, obwohl er jede Sekunde inkrementiert wurde.

**Lösung:** Das Flag wird innerhalb beider `for`-Schleifen geprüft (jede 100 ms), und `App_DelayWithTasks()` verarbeitet Tasks in 1-ms-Scheiben.

---

## 5. Aufgabe 5 – PWM-Hintergrundbeleuchtung (TIM4)

### 5.1 PWM-Grundprinzip

PWM (Pulse Width Modulation) erzeugt ein Rechtecksignal mit variablem Tastverhältnis. Die Helligkeit der Hintergrundbeleuchtung ist proportional zur durchschnittlichen Spannung, die dem Tastverhältnis entspricht:

```
Tastverhältnis = CCR2 / (ARR + 1) = CCR2 / 1000
CCR2 = 200  →  20% Tastverhältnis → Dunkelmodus
CCR2 = 1000 →  100% Tastverhältnis → Vollhelligkeit
```

```
PWM-Signal:
───┐    ┌────────────────────────────┐    ┌─
   │    │← CCR2 (hell) →│← ARR-CCR2 →│    │
   └────┘                            └────┘
   0                ARR              0
```

### 5.2 TIM4-Konfiguration

```
TIM4-Takt = 84 MHz
PSC = 559  →  Zähltakt = 84 MHz / 560 = 150 kHz
ARR = 999  →  PWM-Frequenz = 150 kHz / 1000 = 150 Hz
```

```c
void Timer4_PWM_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // PD13 → Alternate Function 2 (TIM4_CH2)
    GPIOD->MODER &= ~(3U << (2 * 13));
    GPIOD->MODER |=  (2U << (2 * 13));       // 10b = AF-Modus
    GPIOD->AFR[1] &= ~(0xFU << ((13-8)*4));
    GPIOD->AFR[1] |=  (2U   << ((13-8)*4)); // AF2 = TIM4_CH2

    TIM4->PSC = 559;
    TIM4->ARR = 999;  // PWM_ARR

    // PWM-Modus 1, Preload aktivieren
    TIM4->CCMR1 &= ~(TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE);
    TIM4->CCMR1 |=  (6U << TIM_CCMR1_OC2M_Pos) // OC2M=110: PWM-Modus 1
                 |  TIM_CCMR1_OC2PE;            // Preload aktivieren

    TIM4->CCR2   = 200;    // 20% Tastverhältnis beim Start
    TIM4->CCER  |= TIM_CCER_CC2E; // Kanal 2 Ausgang aktivieren
    TIM4->CR1   |= TIM_CR1_ARPE;  // ARR-Preload
    TIM4->EGR   |= TIM_EGR_UG;
    TIM4->SR     = 0;
    TIM4->CR1   |= TIM_CR1_CEN;
}
```

### 5.3 PWM-Modus 1 vs. Modus 2

| Modus | CNT < CCR2 | CNT ≥ CCR2 |
|-------|------------|------------|
| PWM-Modus 1 (110b) | Ausgang = 1 (HIGH) | Ausgang = 0 (LOW) |
| PWM-Modus 2 (111b) | Ausgang = 0 (LOW) | Ausgang = 1 (HIGH) |

Bei Modus 1 mit CCR2=200 und ARR=999: 200/1000 der Zeit ist der Ausgang HIGH → 20% Helligkeit.

### 5.4 Alternate Function: Warum PD13 durch TIM4 gesteuert wird

Im normalen Ausgabemodus steuert die CPU durch Schreiben in ODR den GPIO-Pegel. Im **Alternate Function-Modus** übernimmt ein internes Peripheriegerät (hier TIM4) die direkte Steuerung des Pin-Pegels. Die CPU muss nichts tun – TIM4 erzeugt selbstständig das PWM-Signal.

```c
// GPIO konfiguriert als AF
GPIOD->MODER |= (2U << (2 * 13)); // 10b = Alternate Function

// Welche AF? → AFR-Register
GPIOD->AFR[1] |= (2U << ((13-8)*4)); // AF2 = TIM4_CH2 (lt. Datenblatt Tabelle 9)
```

### 5.5 OC2PE (Output Compare Preload Enable)

Das Preload-Register verhindert Glitches beim Ändern des Tastverhältnisses während eines laufenden PWM-Zyklus:

```
Ohne Preload:  CCR2 sofort wirksam → möglicher Glitch mitten im Puls
Mit Preload:   CCR2 erst beim nächsten Update-Ereignis wirksam → glatter Übergang
```

```c
TIM4->CCMR1 |= TIM_CCMR1_OC2PE; // OC2PE=1: Preload aktiviert
// Jetzt: Schreiben in CCR2 → Wert gepuffert, wirksam beim nächsten ARR-Überlauf
TIM4->CCR2 = new_value;          // wird erst beim nächsten Zyklus aktiv
```

### 5.6 Sanftes Fading (Helligkeitsübergang)

Beim Sanftübergang von 100% auf 20% wird CCR2 in kleinen Schritten reduziert:

```
Schrittgröße  = FADE_STEP     = 4 (CCR2-Einheiten pro Schritt)
Intervall      = FADE_INTERVAL = 10 ms
Schritte       = (1000 - 200) / 4 = 200 Schritte
Gesamtdauer   = 200 × 10 ms = 2000 ms ≈ 2 Sekunden
```

### 5.7 Implementierungsherausforderungen

**Problem: Wieso nicht einfach GPIO mit Software-PWM?**  
Software-PWM (Bit-Banging) würde den CPU zu 100% blockieren und andere Tasks unmöglich machen. Die Hardware-PWM von TIM4 läuft vollständig autonom – der CPU-Overhead ist null.

**Problem: Auswahl der PWM-Frequenz**  
150 Hz ist der Kompromiss: Niedrig genug für die LED-Ansteuerung, hoch genug um für das menschliche Auge kein Flimmern zu erzeugen (>50 Hz Grenze).

---

## 6. Aufgabe 6 – Frequenzmessung (TIM12 Input Capture)

### 6.1 Messprinzip: Input Capture

**Grundidee:** Anstatt die Frequenz zu erzeugen, "fängt" der Timer den Zeitstempel jeder steigenden Flanke des eingehenden Signals auf PB14 ein. Die Frequenz ergibt sich aus der Differenz zweier aufeinanderfolgender Zeitstempel:

```
Signal: ──-───┐    ┌────┐    ┌────┐    ┌────
              │    │    │    │    │    │
              └────┘    └────┘    └────┘
Capture:      T1        T2        T3

Periode = T2 - T1 = delta Ticks
Frequenz = TIM12_CLOCK_HZ / delta = 84.000.000 / delta
```

### 6.2 TIM12-Konfiguration

```
TIM12-Takt = 84 MHz
PSC = 0  →  Zähltakt = 84 MHz / 1 = 84 MHz → 1 Tick = 11,9 ns
ARR = 0xFFFF (65535)  →  Überlauf alle 65536 / 84.000.000 ≈ 780 µs
```

**PB14 → Alternate Function 9 (TIM12_CH1):**

```c
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;

GPIOB->MODER &= ~(3U << (2 * 14));
GPIOB->MODER |=  (2U << (2 * 14));   // AF-Modus

GPIOB->AFR[1] &= ~(0xFU << ((14-8)*4));
GPIOB->AFR[1] |=  (9U   << ((14-8)*4)); // AF9 = TIM12_CH1

TIM12->CR1   = 0;
TIM12->PSC   = 0;
TIM12->ARR   = 0xFFFF;
TIM12->CCMR1 = (1U << TIM_CCMR1_CC1S_Pos); // CC1S=01: Eingangskanal TI1
TIM12->CCER  = TIM_CCER_CC1E;              // CC1E=1: Capture aktivieren (CC1P=0: steigende Flanke)
TIM12->DIER  = TIM_DIER_CC1IE             // Capture-Interrupt
             | TIM_DIER_UIE;              // Überlauf-Interrupt
```

### 6.3 Das 16→32-Bit Erweiterungsproblem

TIM12 ist ein 16-Bit-Timer (max. 65535 Ticks). Bei einer Signalfrequenz unter 1281 Hz würde der Timer zwischen zwei Flanken überlaufen. Lösung: Software-seitige 32-Bit-Erweiterung:

```c
static volatile uint32_t tim12_overflow = 0; // Überlaufzähler

// Im UIE-Interrupt (Überlauf):
tim12_overflow++;

// Virtueller 32-Bit-Zeitstempel:
uint32_t cap32 = (tim12_overflow << 16) | raw16;
//               ↑ obere 16 Bit                ↑ Hardware-CCR1
```

Mit 32-Bit-Erweiterung können bis zu ca. 2^32 / 84.000.000 ≈ 51 Sekunden zwischen zwei Flanken liegen, was einer Minimalfrequenz von ca. 0,02 Hz entspricht.

### 6.4 Die kritische Race Condition

Wenn UIE und CC1IF **gleichzeitig** gesetzt werden, gibt es eine Mehrdeutigkeit: War die Flanke *vor* oder *nach* dem Überlauf?

```
Fall A: Flanke → dann Überlauf (korrekt)
  CCR1 = kleiner Wert (z.B. 0x0010)
  → tim12_overflow ist bereits korrekt erhöht

Fall B: Überlauf → dann Flanke (Problem!)
  CCR1 = großer Wert (z.B. 0xFFF0)
  → tim12_overflow wurde schon erhöht, obwohl die Flanke vor dem Überlauf war
  → overflow muss um 1 verringert werden
```

**Lösung durch Schwellenwertprüfung:**

```c
uint32_t ov = tim12_overflow;
if ((sr & TIM_SR_UIF) && (raw16 >= 0x8000U)) {
    ov--;  // Korrektur: Flanke war vor dem Überlauf
}
uint32_t cap32 = (ov << 16) | raw16;
```

**Intuition:** Wenn UIE und CC1IF gleichzeitig auftreten und CCR1 ≥ 0x8000 (obere Hälfte des Zählbereichs), dann muss die Flanke kurz *vor* dem Überlauf stattgefunden haben. Der Überlaufzähler wurde zu früh erhöht.

### 6.5 Auflösungsproblem und Lösung

**Das Problem mit PSC=83 (1 MHz Zähltakt):**

```
Signal: 78 kHz → Periode = 1/78.000 s ≈ 12,82 µs
Bei 1 MHz: 12,82 Ticks → gerundet auf 12 oder 13 Ticks

Bei 13 Ticks: f = 1.000.000 / 13 = 76.923 Hz (Fehler: -1,4%)
Bei 12 Ticks: f = 1.000.000 / 12 = 83.333 Hz (Fehler: +6,7%)
```

**Die Lösung mit PSC=0 (84 MHz Zähltakt):**

```
Signal: 78 kHz → Periode = 12,82 µs
Bei 84 MHz: 1075,7 Ticks → gerundet auf 1075 oder 1076 Ticks

Bei 1075 Ticks: f = 84.000.000 / 1075 = 78.140 Hz (Fehler: <0,03%)
Bei 1076 Ticks: f = 84.000.000 / 1076 = 78.066 Hz (Fehler: <0,07%)
```

Der Quantisierungsfehler skaliert umgekehrt proportional zur Anzahl der Ticks pro Periode. Mehr Ticks = weniger Fehler.

### 6.6 Vollständiger ISR

```c
void TIM8_BRK_TIM12_IRQHandler(void)
{
    uint32_t sr = TIM12->SR; // Snapshot (einmal lesen)

    // 1. Überlauf behandeln
    if (sr & TIM_SR_UIF) {
        TIM12->SR &= ~TIM_SR_UIF;
        tim12_overflow++;
    }

    // 2. Capture behandeln
    if (sr & TIM_SR_CC1IF) {
        TIM12->SR &= ~TIM_SR_CC1IF;

        uint16_t raw16 = (uint16_t)TIM12->CCR1; // Zeitstempel auslesen

        // Race Condition korrigieren
        uint32_t ov = tim12_overflow;
        if ((sr & TIM_SR_UIF) && (raw16 >= 0x8000U))
            ov--;

        uint32_t cap32 = (ov << 16) | raw16;

        if (tim12_first) {
            tim12_last_cap32 = cap32;
            tim12_first = 0;
        } else {
            uint32_t delta = cap32 - tim12_last_cap32;
            tim12_last_cap32 = cap32;

            if (delta > 0) {
                cap_ticks   = delta;
                cap_freq_hz = 84000000UL / delta; // Hz (Integer)
                cap_last_ms = ms_ticks;
                cap_valid   = 1;

                // LCD maximal alle 200 ms aktualisieren
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

### 6.7 Warum TIM8_BRK_TIM12_IRQn?

Auf dem STM32F407 teilen sich TIM12 und der TIM8-Break-Event einen gemeinsamen Interrupt-Vektor. Der Handler muss daher `TIM12_BRK_TIM12_IRQn` heißen – es gibt **keinen** separaten `TIM12_IRQn`. Beim Eintritt in den Handler prüft man TIM12->SR, um Capture- von Überlauf-Ereignissen zu unterscheiden.

### 6.8 Implementierungsherausforderungen

**Problem: LCD-Blockierung bei hoher Eingangsfrequenz**  
Bei 78 kHz werden ca. 78.000 Capture-Interrupts pro Sekunde ausgelöst. Ohne LCD-Ratenbegrenzung würde `lcd_update_pending` 78.000 Mal pro Sekunde gesetzt und `LCD_UpdateDisplay()` würde die gesamte CPU-Zeit beanspruchen.  
**Lösung:** `lcd_update_pending` wird maximal alle 200 ms gesetzt.

**Problem: Timeout-Anzeige**  
Wenn kein Signal vorhanden ist, soll das Display `--` anzeigen:

```c
uint8_t timeout = (!cap_valid) ||
                  ((ms_ticks - cap_last_ms) >= 2000); // 2 Sekunden Timeout

if (timeout) {
    LCD_WriteString(10, 55, WHITE, 0xFE00, "Ticks: --        ");
    LCD_WriteString(10, 75, WHITE, 0xFE00, "Freq:  --        ");
}
```

---

## 7. Modulare Codestruktur

### 7.1 Dateiaufteilung

Die ursprünglich monolithische `main.c` (641 Zeilen) wurde in 6 Module aufgeteilt:

| Datei | Aufgabe | Inhalt |
|-------|---------|--------|
| `leds.c/.h` | 1/2 | `set_output`, `u_delay`, `LEDs_InitPorts`, `LCD_Output16BitWord`, `LEDs_Write` |
| `display.c/.h` | 3 | SSD2119-Treiber, Grafik, Textausgabe |
| `main.c` | 4 + Einstieg | TIM7, `App_ProcessTickTasks`, Hauptschleife, `LCD_UpdateDisplay` |
| `pwm.c/.h` | 5 | `Timer4_PWM_Init`, `Set_Backlight_Duty`, PWM-Globals |
| `freq_measure.c/.h` | 6 | `Timer12_Capture_Init`, ISR, Capture-Globals |

### 7.2 Datenaustausch über extern-Deklarationen

Globale Variablen werden einmal definiert und in anderen Modulen als `extern` deklariert:

```c
// pwm.c (Definition):
volatile uint16_t current_duty = 200;
volatile uint8_t  fade_active  = 0;
volatile uint32_t fade_last    = 0;

// pwm.h (Deklaration für andere Module):
extern volatile uint16_t current_duty;
extern volatile uint8_t  fade_active;
extern volatile uint32_t fade_last;
```

```c
// freq_measure.c (benötigt ms_ticks aus main.c):
extern volatile uint32_t ms_ticks;
extern volatile uint8_t  lcd_update_pending;
```

### 7.3 Interrupt-Prioritäten

| Timer | IRQn | Priorität | Begründung |
|-------|------|-----------|------------|
| TIM12 | TIM8_BRK_TIM12_IRQn | 0 (höchste) | Zeitstempel-Präzision |
| TIM7  | TIM7_IRQn | 1 | Systemtakt-Basis |

TIM12 hat höhere Priorität als TIM7, damit eine mögliche TIM7-ISR die Capture-Zeitstempel nicht verzögert.

### 7.4 Keil-Projektdatei (uvprojx)

Alle drei Kompilierungsziele (Flash, RAM, Simulation) wurden aktualisiert, um die neuen `.c`-Dateien zu beinhalten:

```xml
<File>
  <FileName>leds.c</FileName>
  <FileType>1</FileType>   <!-- 1 = C-Quelldatei -->
  <FilePath>.\leds.c</FilePath>
</File>
```

---

*Erstellt für das MC-Praktikum SoSe 2026, THI Ingolstadt.*
