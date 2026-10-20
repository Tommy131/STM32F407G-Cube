#include "a2_1_5.h"

#include "main.h"
#include "cmsis_os.h"

#include <math.h>

extern TIM_HandleTypeDef htim4;

static void DimmTask(void *argument)
{
  const uint32_t periodMs = 2000;
  const uint32_t stepMs = 20;
  uint32_t timeMs = 0;
  uint32_t maxCompare = htim4.Init.Period;

  (void)argument;
  for (;;)
  {
    float phase = 2.0f * 3.14159265f * (float)timeMs / (float)periodMs;
    float duty = (1.0f - cosf(phase)) * 0.5f;
    uint32_t compare = (uint32_t)(duty * (float)maxCompare + 0.5f);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, compare);
    osDelay(stepMs);
    timeMs += stepMs;
    if (timeMs >= periodMs) {
      timeMs = 0;
    }
  }
}

static const osThreadAttr_t dimmTask_attributes = {
  .name = "dimmTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

void A2_1_5_Start(void)
{
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 32768);
  osThreadNew(DimmTask, NULL, &dimmTask_attributes);
}
