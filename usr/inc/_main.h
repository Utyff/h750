#ifndef MY_MAIN_H
#define MY_MAIN_H

#ifdef STM32F303xC
#include "stm32f3xx_hal.h"
#endif
#ifdef STM32F407xx
#include "stm32f4xx_hal.h"
#endif
#if defined (STM32F765xx) || defined(STM32F746xx)
#include "stm32f7xx_hal.h"
#endif
#if defined(STM32H743xx) ||  defined(STM32H750xx)
#include "stm32h7xx_hal.h"
#endif

#include "main.h"

typedef int32_t s32;
typedef int16_t s16;
typedef int8_t s8;

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;

typedef __IO uint32_t vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t vu8;

extern UART_HandleTypeDef huart1;

void mainInitialize();

void mainCycle();

#define DEBUG_TRACE_UART

#if defined(DEBUG_TRACE_SWO)

void SWO_Trace(uint8_t* msg);
#define DBG_Trace(msg) SWO_Trace((uint8_t*)(msg))

#elif defined(DEBUG_TRACE_UART)

#include "string.h"
#include "stdio.h"

#define DBG_Trace(msg) HAL_UART_Transmit(&huart1, (uint8_t*)(msg), (uint16_t)strlen((char*)(msg)), 0xFFFF)

#elif defined(DEBUG_TRACE_NONE)

#define DBG_Trace(msg)

#else
#error Define DUBUG_TRACE destination
#endif

#endif //MY_MAIN_H
