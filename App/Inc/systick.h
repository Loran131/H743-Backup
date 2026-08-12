/**
 ******************************************************************************
 * @file    systick.h
 * @brief   SysTick utility — non-blocking delay and software timers
 * @note    Relies on HAL_GetTick() (1 ms SysTick). Correctly handles 32-bit
 *          wraparound via unsigned arithmetic (wraps after ~49.7 days).
 ******************************************************************************
 */

#ifndef __SYSTICK_H__
#define __SYSTICK_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Max number of software timers ----------------------------------------------*/
#define SYSTICK_MAX_TIMERS    8

/* Exported functions --------------------------------------------------------*/

/* --- Core time functions --- */
void     SysTick_Init(void);
uint32_t SysTick_Get(void);
uint32_t SysTick_Elapsed(uint32_t start_tick);
uint32_t SysTick_Uptime(void);

/* --- Non-blocking delay --- */
void     SysTick_DelayMs(uint32_t ms);
uint8_t  SysTick_CheckDelay(void);

/* --- Software timers (up to 8) --- */
int8_t   SysTick_TimerCreate(uint32_t period_ms, uint8_t oneshot);
void     SysTick_TimerDelete(int8_t id);
uint8_t  SysTick_TimerExpired(int8_t id);
void     SysTick_TimerReset(int8_t id);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTICK_H__ */
