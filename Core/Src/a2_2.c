#include "a2_2.h"

#include "main.h"
#include "cmsis_os.h"

/*
 * A2.2: Producer 发整数，Consumer 求平均后再送回来。
 *
 * 本工程调用的是 CMSIS-RTOS2，不是 FreeRTOS 原生函数名。
 * CubeMX 选择了 Middleware -> FreeRTOS -> Interface CMSIS_V2，
 * 所以任务、队列、信号量都用 osXxx()。osXxx() 在 cmsis_os2.c 里
 * 转成 xTaskCreate、xQueueCreate、xSemaphoreCreateCounting 等。
 *
 * 查文档时看两层：
 *
 * 1. 你实际调用的 CMSIS-RTOS2
 *    本地原型和参数说明：
 *      Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.h
 *    在线手册：
 *      https://arm-software.github.io/CMSIS_5/RTOS2/html/group__CMSIS__RTOS.html
 *
 * 2. 底下的 FreeRTOS 原生 API
 *    本地包装：
 *      Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
 *    本地头文件：
 *      Middlewares/Third_Party/FreeRTOS/Source/include/task.h
 *      Middlewares/Third_Party/FreeRTOS/Source/include/queue.h
 *      Middlewares/Third_Party/FreeRTOS/Source/include/semphr.h
 *    在线手册：
 *      https://www.freertos.org/Documentation/02-Kernel/04-API-references
 *
 * 时间单位是内核 tick。本工程 configTICK_RATE_HZ 为 1000，所以 1 tick = 1 ms。
 * osWaitForever 在 cmsis_os2.h 里是 0xFFFFFFFF，表示一直等到成功。
 */

/* 一条消息是一个 int32_t。16 能放下 1..10 再加结束标记 0。 */
#define QUEUE_DEPTH 16

/*
 * RNG 已由 main() 里的 MX_RNG_Init() 初始化。
 * 只有选择硬件随机数时才用它。HAL 文档不在 FreeRTOS 里：
 *   Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rng.h
 *   函数 HAL_RNG_GenerateRandomNumber()
 * 课件指向的说明是 UM1725，第 56.2.1 节。
 */
extern RNG_HandleTypeDef hrng;

/*
 * numberQueue：Producer -> Consumer，元素是待平均的整数，最后一条是 0。
 * averageQueue：Consumer -> Producer，只回传一个平均值，所以深度是 1。
 */
static osMessageQueueId_t numberQueue;
static osMessageQueueId_t averageQueue;

/*
 * 计数信号量，和 numberQueue 的 16 个位置一一对应。
 * emptySlots 的令牌数 = 还可以放几个数。
 * filledSlots 的令牌数 = 已经放下、还没取走的数。
 * averageQueue 只有一个写者和一个读者，不用这两个信号量。
 */
static osSemaphoreId_t emptySlots;
static osSemaphoreId_t filledSlots;

static void ProducerTask(void *argument);
static void ConsumerTask(void *argument);
static void SendNumber(int32_t value);
static void ReceiveNumber(int32_t *value);

/* 调试器里观察这一轮的整数平均值。1..10 的结果是 5。 */
static volatile int32_t lastAverage;

/*
 * osThreadAttr_t 的字段说明在 cmsis_os2.h 的 osThreadAttr_t。
 * stack_size 单位是字节。256 * 4 = 1024 字节，够整数运算和一次 HAL 调用。
 * 两个任务用同一优先级。它们会在队列或信号量上阻塞，不会互相饿死。
 */
static const osThreadAttr_t producerTask_attributes = {
  .name = "producer",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

static const osThreadAttr_t consumerTask_attributes = {
  .name = "consumer",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

void A2_2_Init(void)
{
  /*
   * 创建整数队列。必须在 osKernelInitialize() 之后、osKernelStart() 之前调用。
   * main() 的 USER CODE RTOS_THREADS 满足这个顺序。
   *
   * osMessageQueueNew(消息条数, 每条字节数, 属性)
   *   属性传 NULL，使用 CMSIS 默认内存分配。
   *   失败时返回 NULL。
   * 文档：cmsis_os2.h，章节 "Message Queue Management Functions"
   *       https://arm-software.github.io/CMSIS_5/RTOS2/html/group__CMSIS__RTOS__Message.html
   * 对应 FreeRTOS：cmsis_os2.c 里的 osMessageQueueNew() 调用 xQueueCreate()
   *       queue.h；https://www.freertos.org/a00116.html
   */
  numberQueue = osMessageQueueNew(QUEUE_DEPTH, sizeof(int32_t), NULL);

  /* 平均值邮箱。深度 1：旧平均值没被取走时，下一次 Put 会阻塞。 */
  averageQueue = osMessageQueueNew(1, sizeof(int32_t), NULL);

  /*
   * osSemaphoreNew(最大令牌数, 初始令牌数, 属性)
   * emptySlots 一开始是满的：16 个位置都能放。
   * filledSlots 一开始是 0：还没有任何数可读。
   * 文档：cmsis_os2.h，章节 "Semaphore Management Functions"
   *       https://arm-software.github.io/CMSIS_5/RTOS2/html/group__CMSIS__RTOS__SemaphoreMgmt.html
   * 对应 FreeRTOS：xSemaphoreCreateCounting()
   *       semphr.h；https://www.freertos.org/xSemaphoreCreateCounting.html
   */
  emptySlots = osSemaphoreNew(QUEUE_DEPTH, QUEUE_DEPTH, NULL);
  filledSlots = osSemaphoreNew(QUEUE_DEPTH, 0, NULL);

  /* 任一对象创建失败就停在 Error_Handler()，避免后面用空句柄。 */
  if (numberQueue == NULL || averageQueue == NULL ||
      emptySlots == NULL || filledSlots == NULL) {
    Error_Handler();
  }

  /*
   * osThreadNew(任务函数, 传给任务的参数, 属性)
   * 这里不需要给任务传参数，所以第二个参数是 NULL。
   * 任务要等 osKernelStart() 之后才会真正运行。
   * 文档：cmsis_os2.h，章节 "Thread Management Functions"
   *       https://arm-software.github.io/CMSIS_5/RTOS2/html/group__CMSIS__RTOS__ThreadMgmt.html
   * 对应 FreeRTOS：xTaskCreate()
   *       task.h；https://www.freertos.org/a00125.html
   */
  osThreadNew(ProducerTask, NULL, &producerTask_attributes);
  osThreadNew(ConsumerTask, NULL, &consumerTask_attributes);
}

static void CheckStatus(osStatus_t status)
{
  if (status != osOK) {
    Error_Handler();
  }
}

static void SendNumber(int32_t value)
{
  /* 先占一个空位，再放入队列，最后通知对方有新数据。 */
  CheckStatus(osSemaphoreAcquire(emptySlots, osWaitForever));
  CheckStatus(osMessageQueuePut(numberQueue, &value, 0, 0));
  CheckStatus(osSemaphoreRelease(filledSlots));
}

static void ReceiveNumber(int32_t *value)
{
  /* 顺序与发送相反：等一个已放入的数，取出，再还回空位。 */
  CheckStatus(osSemaphoreAcquire(filledSlots, osWaitForever));
  CheckStatus(osMessageQueueGet(numberQueue, value, NULL, 0));
  CheckStatus(osSemaphoreRelease(emptySlots));
}

static void ProducerTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    int32_t number;
    int32_t average;

    for (number = 1; number <= 10; number++) {
      SendNumber(number);
    }
    /* 0 只表示这一串结束，Consumer 不把它加进总和。 */
    SendNumber(0);

    CheckStatus(osMessageQueueGet(averageQueue, &average, NULL, osWaitForever));
    lastAverage = average;

    /* 停 1 秒再发下一轮，方便在调试器里看 lastAverage。 */
    osDelay(1000);
  }
}

static void ConsumerTask(void *argument)
{
  int32_t sum = 0;
  int32_t count = 0;

  (void)argument;

  for (;;)
  {
    int32_t value;
    int32_t average;

    ReceiveNumber(&value);
    if (value != 0) {
      sum += value;
      count++;
      continue;
    }

    average = (count == 0) ? 0 : (sum / count);
    CheckStatus(osMessageQueuePut(averageQueue, &average, 0, osWaitForever));
    sum = 0;
    count = 0;
  }
}
