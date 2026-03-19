/* Mikrocontroller-Programmierung Start-Projekt */
/* U. Margull, 29.2.2024 (V1.5)                 */

#include "stm32f4xx.h"
#include "_mcpr_stm32f407.h"
#include <inttypes.h>


int main(void)
{
	uint32_t i = 0;
	
	mcpr_SetSystemCoreClock();
	
	// Peripheral GPIOD einschalten
	RCC->AHB1ENR |= 1<<3|1;

	// Orange LED (Port D13) auf Ausgang schalten
	GPIOD->MODER |= 0x04000000; //1<<26;
	GPIOD->ODR |= 1<<13;
	
	while( 1 ) {
		if( (GPIOA->IDR & 1) != 0) { 
			GPIOD->ODR |= 1<<13; 
		} else { 
			GPIOD->ODR &= 0xDFFF; // ~(1<<13); 
		}
	}
		
}

