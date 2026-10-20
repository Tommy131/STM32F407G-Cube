#ifndef A2_3_H
#define A2_3_H

#include <stdint.h>

/*
 * A2.3 只负责把已经导入的 display.c 接到启动流程上。
 * 清屏和写字符串留在 A2_3_Init() 的 TODO 里。
 */
void A2_3_Init(void);

#endif
