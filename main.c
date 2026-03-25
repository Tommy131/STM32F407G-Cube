/* Mikrocontroller-Programmierung Start-Projekt */
/* U. Margull, 29.2.2024 (V1.5)                 */

#include "stm32f4xx.h"
#include "_mcpr_stm32f407.h"
#include <inttypes.h>

/* Funktionsdeklaration fuer die Verzoegerungsfunktion */
void u_delay(uint32_t usec);

int main(void)
{
    /* Systemtakt initialisieren */
    mcpr_SetSystemCoreClock();

    /* Takt fuer GPIOA (Bit 0) und GPIOD (Bit 3) ueber RCC einschalten */
    RCC->AHB1ENR |= (1 << 0) | (1 << 3);

    /* PA0 (USER-Taster) als Eingang konfigurieren:
       MODER Bits 1:0 auf 00 setzen ? Eingangsmodus */
    GPIOA->MODER &= ~(0x00000003);

    /* PD12 (gruene LED) als Ausgang konfigurieren:
       MODER Bits 25:24 auf 01 setzen ? Ausgangsmodus */
    GPIOD->MODER |= 0x01000000;

    /* Hauptschleife */
    while (1)
    {
        /* Pruefen ob USER-Taster (PA0) gedrueckt ist (IDR Bit 0 == 1) */
        if ((GPIOA->IDR & 1) != 0)
        {
            /* LED einschalten: PD12 im ODR auf 1 setzen */
            GPIOD->ODR |= (1 << 12);

            /* 0,5 Sekunden warten */
            u_delay(500000);

            /* LED ausschalten: PD12 im ODR auf 0 setzen */
            GPIOD->ODR &= ~(1 << 12);

            /* 0,5 Sekunden warten */
            u_delay(500000);
        }
        else
        {
            /* Taster nicht gedrueckt ? LED dauerhaft ausschalten */
            GPIOD->ODR &= ~(1 << 12);
        }
    }
}

/* Verzoegerungsfunktion (Busy-Waiting)
   Parameter: usec ? gewuenschte Wartezeit in Mikrosekunden (ungefaehr)
   Die Schleife laeuft (usec * 10) mal, um die Verzoegerung zu erzeugen.
   volatile verhindert, dass der Compiler die Schleife wegoptimiert. */
void u_delay(uint32_t usec)
{
    for (volatile uint32_t i = 0; i < (usec * 10); i++)
    {
    }
}