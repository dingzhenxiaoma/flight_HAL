#ifndef __COM_DEBUG_H__
#define __COM_DEBUG_H__

#include "usart.h"
#include <stdio.h>
#include <string.h>
#include "stdarg.h"

//飞机飞行的时候要关闭打印功能
#define DEBUG_LOG_ENABLE 1

#ifdef DEBUG_LOG_ENABLE

//提取文件名，去掉路径前缀（兼容 / 和 \ 两种路径分隔符）
#define _DBG_STRIP_SLASH (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define _DBG_STRIP_BSLASH (strrchr(_DBG_STRIP_SLASH, '\\') ? strrchr(_DBG_STRIP_SLASH, '\\') + 1 : _DBG_STRIP_SLASH)

//添加文件名和行号
#define debug_printf(format, ...) printf("[%s:%d] " format, _DBG_STRIP_BSLASH, __LINE__, ##__VA_ARGS__)

#else
#define debug_printf(format, ...)

#endif

#endif // __COM_DEBUG_H__
