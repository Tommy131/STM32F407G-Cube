#include "a2_1_2.h"

#include "main.h"
#include "cmsis_os.h"

static void OrangeCallback(void *argument)
{
  (void)argument;
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
}

void A2_1_2_Start(void)
{
  osTimerId_t orangeTimerHandle;

  orangeTimerHandle = osTimerNew(OrangeCallback, osTimerPeriodic, NULL, NULL);
  if (orangeTimerHandle != NULL) {
    osTimerStart(orangeTimerHandle, 100);
  }
}
