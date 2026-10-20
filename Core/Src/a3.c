#include "a3.h"

#include "display.h"
#include "main.h"
#include "cmsis_os.h"

/*
 * A3 加分题。五个任务通过两条队列通信，液晶屏由互斥量保护。
 *
 *   buttonTask  读 User 键，状态变化后发给 displayTask
 *   randomTask  每秒用片上随机数发生器产生一个数，发给 evalTask
 *   evalTask    每 5 个数算一次平均值，发给 displayTask
 *   potTask     读 PA1（ADC1 通道 1）上的电位器，发给 displayTask
 *   displayTask 把这三项画到液晶屏上
 *
 * 软件定时器在 a2_1_2.c，硬件定时器在 a2_1_3.c。
 * PD14 和 PD15 是液晶数据线，不要再拿它们闪灯。
 *
 * 系统调用是 CMSIS-RTOS2，原型在 cmsis_os2.h。
 * 片上随机数文档：stm32f4xx_hal_rng.h，课件 UM1725 第 56.2.1 节。
 * 本工程没有带入 ADC 的 HAL 源文件，电位器直接读 ADC1 寄存器。
 * 电位器请接到 PA1。没有接线时，读数会漂在某个固定值附近。
 */

#define A3_DISPLAY_QUEUE_DEPTH 8
#define A3_RANDOM_QUEUE_DEPTH 8
#define A3_SAMPLE_COUNT 5
#define A3_LINE_CHARS 24

typedef enum {
  A3_MSG_BUTTON = 1,
  A3_MSG_AVERAGE,
  A3_MSG_POTENTIOMETER
} A3_MessageType;

typedef struct {
  A3_MessageType type;
  uint32_t value;
} A3_Message;

extern RNG_HandleTypeDef hrng;

static osMessageQueueId_t displayQueue;
static osMessageQueueId_t randomQueue;
static osMutexId_t lcdMutex;
static osSemaphoreId_t buttonSignal;

static void ButtonTask(void *argument);
static void RandomTask(void *argument);
static void EvalTask(void *argument);
static void PotTask(void *argument);
static void DisplayTask(void *argument);
static void SendDisplay(A3_MessageType type, uint32_t value);
static void InitPotentiometer(void);
static uint16_t ReadPotentiometer(void);

static const osThreadAttr_t buttonTask_attributes = {
  .name = "button",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

static const osThreadAttr_t randomTask_attributes = {
  .name = "random",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

static const osThreadAttr_t evalTask_attributes = {
  .name = "eval",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

static const osThreadAttr_t potTask_attributes = {
  .name = "pot",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

static const osThreadAttr_t displayTask_attributes = {
  .name = "display",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

void A3_Init(void)
{
  displayQueue = osMessageQueueNew(A3_DISPLAY_QUEUE_DEPTH, sizeof(A3_Message), NULL);
  randomQueue = osMessageQueueNew(A3_RANDOM_QUEUE_DEPTH, sizeof(uint32_t), NULL);
  lcdMutex = osMutexNew(NULL);
  /* 每个按键事件一个令牌，深度与显示队列相同，避免连续按键丢信号。 */
  buttonSignal = osSemaphoreNew(A3_DISPLAY_QUEUE_DEPTH, 0, NULL);

  if (displayQueue == NULL || randomQueue == NULL ||
      lcdMutex == NULL || buttonSignal == NULL) {
    Error_Handler();
  }

  InitPotentiometer();

  osThreadNew(ButtonTask, NULL, &buttonTask_attributes);
  osThreadNew(RandomTask, NULL, &randomTask_attributes);
  osThreadNew(EvalTask, NULL, &evalTask_attributes);
  osThreadNew(PotTask, NULL, &potTask_attributes);
  osThreadNew(DisplayTask, NULL, &displayTask_attributes);
}

static void CheckStatus(osStatus_t status)
{
  if (status != osOK) {
    Error_Handler();
  }
}

static void SendDisplay(A3_MessageType type, uint32_t value)
{
  A3_Message message;

  message.type = type;
  message.value = value;
  CheckStatus(osMessageQueuePut(displayQueue, &message, 0, osWaitForever));
}

static void InitPotentiometer(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  /* PA1 = ADC1 通道 1，模拟输入。 */
  GPIOA->MODER |= (3U << (1U * 2U));
  GPIOA->PUPDR &= ~(3U << (1U * 2U));

  /* APB2 是 84 MHz，四分频后 ADC 时钟是 21 MHz。 */
  ADC123_COMMON->CCR &= ~ADC_CCR_ADCPRE;
  ADC123_COMMON->CCR |= ADC_CCR_ADCPRE_0;

  ADC1->CR1 = 0;
  ADC1->CR2 = 0;
  ADC1->SQR1 = 0;
  ADC1->SQR3 = 1;
  ADC1->SMPR2 &= ~(7U << 3);
  ADC1->SMPR2 |= (4U << 3);
  ADC1->CR2 |= ADC_CR2_ADON;
}

static uint16_t ReadPotentiometer(void)
{
  uint32_t timeout = 100000U;

  ADC1->SR &= ~ADC_SR_EOC;
  ADC1->CR2 |= ADC_CR2_SWSTART;
  while (((ADC1->SR & ADC_SR_EOC) == 0U) && (timeout > 0U)) {
    timeout--;
  }
  if (timeout == 0U) {
    return 0;
  }
  return (uint16_t)ADC1->DR;
}

static void ButtonTask(void *argument)
{
  GPIO_PinState last = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

  (void)argument;
  SendDisplay(A3_MSG_BUTTON, (last == GPIO_PIN_SET) ? 1U : 0U);
  osSemaphoreRelease(buttonSignal);

  for (;;)
  {
    GPIO_PinState now = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

    if (now != last) {
      last = now;
      SendDisplay(A3_MSG_BUTTON, (now == GPIO_PIN_SET) ? 1U : 0U);
      /* 二进制令牌已满时 Release 会返回资源错误，下一轮仍会重发。 */
      osSemaphoreRelease(buttonSignal);
    }
    osDelay(20);
  }
}

static void RandomTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    uint32_t value = 0;

    if (HAL_RNG_GenerateRandomNumber(&hrng, &value) == HAL_OK) {
      CheckStatus(osMessageQueuePut(randomQueue, &value, 0, osWaitForever));
    }
    osDelay(1000);
  }
}

static void EvalTask(void *argument)
{
  uint64_t sum = 0;
  uint32_t count = 0;

  (void)argument;

  for (;;)
  {
    uint32_t value = 0;

    CheckStatus(osMessageQueueGet(randomQueue, &value, NULL, osWaitForever));
    sum += value;
    count++;
    if (count >= A3_SAMPLE_COUNT) {
      SendDisplay(A3_MSG_AVERAGE, (uint32_t)(sum / A3_SAMPLE_COUNT));
      sum = 0;
      count = 0;
    }
  }
}

static void PotTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    SendDisplay(A3_MSG_POTENTIOMETER, ReadPotentiometer());
    osDelay(200);
  }
}

static void AppendText(char *dst, uint32_t *index, const char *text)
{
  while (*text != '\0' && *index < (A3_LINE_CHARS - 1U)) {
    dst[*index] = *text;
    (*index)++;
    text++;
  }
}

static void AppendU32(char *dst, uint32_t *index, uint32_t value)
{
  char digits[10];
  uint32_t count = 0;

  if (value == 0U) {
    digits[count++] = '0';
  } else {
    while (value > 0U && count < 10U) {
      digits[count++] = (char)('0' + (value % 10U));
      value /= 10U;
    }
  }
  while (count > 0U && *index < (A3_LINE_CHARS - 1U)) {
    count--;
    dst[*index] = digits[count];
    (*index)++;
  }
}

static void DrawLine(uint16_t y, const char *label, uint32_t value, uint8_t asNumber)
{
  char line[A3_LINE_CHARS];
  uint32_t index = 0;

  AppendText(line, &index, label);
  if (asNumber != 0U) {
    AppendU32(line, &index, value);
  }
  while (index < (A3_LINE_CHARS - 1U)) {
    line[index++] = ' ';
  }
  line[index] = '\0';

  CheckStatus(osMutexAcquire(lcdMutex, osWaitForever));
  LCD_WriteString(10, y, BLACK, WHITE, line);
  CheckStatus(osMutexRelease(lcdMutex));
}

static void DisplayTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    A3_Message message;

    CheckStatus(osMessageQueueGet(displayQueue, &message, NULL, osWaitForever));
    if (message.type == A3_MSG_BUTTON) {
      /* 取走按键令牌。超时为 0：令牌还没到时也不卡住后续显示。 */
      osSemaphoreAcquire(buttonSignal, 0);
      DrawLine(40, (message.value != 0U) ? "Button: pressed" : "Button: released", 0, 0);
    } else if (message.type == A3_MSG_AVERAGE) {
      DrawLine(64, "Average: ", message.value, 1);
    } else if (message.type == A3_MSG_POTENTIOMETER) {
      DrawLine(88, "Pot: ", message.value, 1);
    }
  }
}
