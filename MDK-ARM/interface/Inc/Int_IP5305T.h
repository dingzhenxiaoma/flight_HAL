#ifndef __INT_IP5305T_H__
#define __INT_IP5305T_H__

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

//启动IP5305T电源
void Int_IP5305T_start(void);

// 关闭IP5305T电源
void Int_IP5305T_shutdown(void);

#endif
