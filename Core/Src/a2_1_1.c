#include "a2_1_1.h"

#include "main.h"
#include "cmsis_os.h"

void A2_1_1_Task(void *argument)
{
  (void)argument;

  for (;;)
  {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
      osDelay(100);
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    } else {
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    }
    /* Block so the lower-priority timer service task can run. */
    osDelay(10);
  }
}
