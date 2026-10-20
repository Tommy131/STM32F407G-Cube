#include "a2_3.h"

#include "display.h"
#include "main.h"

/*
 * A2.3：硬件初始化之后、启动内核之前初始化液晶屏，再清屏并写一行字。
 * 显示接口在 Core/Inc/display.h，字模在 Core/Inc/fonts.h。
 *
 * PD14、PD15 同时是液晶数据线和当前的红灯、蓝灯。
 * a2_1_2.c 与 a2_1_3.c 仍在翻转这两根脚，写屏时会干扰数据线。
 */

static void SetOutput(GPIO_TypeDef *port, uint32_t pin)
{
  port->MODER &= ~(3U << (2U * pin));
  port->MODER |= (1U << (2U * pin));
}

static void InitDataPins(void)
{
  uint32_t pin;

  /* display.c 只配置控制线。数据线要先变成推挽输出。 */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;

  SetOutput(GPIOD, 0);
  SetOutput(GPIOD, 1);
  SetOutput(GPIOD, 8);
  SetOutput(GPIOD, 9);
  SetOutput(GPIOD, 10);
  SetOutput(GPIOD, 14);
  SetOutput(GPIOD, 15);
  for (pin = 7; pin <= 15; pin++) {
    SetOutput(GPIOE, pin);
  }
}

void A2_3_Init(void)
{
  InitDataPins();
  LCD_Init();
  LCD_ClearDisplay(RED);
  LCD_WriteString(10, 10, BLACK, RED, "Hallo Kurs!");
}
