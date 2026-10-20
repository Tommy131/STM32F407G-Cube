#include "a2_1_3.h"

#include "main.h"

extern TIM_HandleTypeDef htim7;

void A2_1_3_Start(void)
{
  HAL_TIM_Base_Start_IT(&htim7);
}

void A2_1_3_OnPeriodElapsed(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM7) {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
  }
}
