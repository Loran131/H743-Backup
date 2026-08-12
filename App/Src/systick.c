/**
 ******************************************************************************
 * @file    systick.c
 * @brief   SysTick utility implementation
 *          - Up to 8 software timers with periodic or one-shot modes
 *          - Non-blocking delay (starter + poll in main loop)
 *          - Elapsed-time and uptime helpers
 *          - All timing via HAL_GetTick(); unsigned arithmetic handles
 *            32-bit wraparound correctly (49.7-day rollover).
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "systick.h"
#include "main.h"

/* Private types --------------------------------------------------------------*/
typedef struct {
    uint32_t period_ms;      /* Timer period, 0 = inactive                        */
    uint32_t next_expiry;    /* Absolute tick when next expiry fires               */
    uint8_t  oneshot;        /* 1 = one-shot, 0 = periodic (auto-reload)           */
} SysTick_Timer_t;

/* Private variables ---------------------------------------------------------*/
static uint32_t      g_startup_tick;                           /* Recorded in SysTick_Init()          */
static uint32_t      g_delay_target;                           /* Target tick for non-blocking delay  */
static uint8_t       g_delay_active;                           /* 1 = delay running, 0 = idle         */
static SysTick_Timer_t g_timers[SYSTICK_MAX_TIMERS];           /* Software timer pool                 */

/* -------------------------------------------------------------------------- */
/*                         Core time functions                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Record the startup tick for uptime calculation.
 * @note   Call once near the start of main(), after HAL_Init().
 */
void SysTick_Init(void)
{
    g_startup_tick = HAL_GetTick();
}

/**
 * @brief  Get the current system tick in milliseconds.
 * @retval Current HAL tick value (wraps after ~49.7 days).
 */
uint32_t SysTick_Get(void)
{
    return HAL_GetTick();
}

/**
 * @brief  Calculate milliseconds elapsed since a given start tick.
 * @param  start_tick: the tick value captured at the start of the interval.
 * @retval Milliseconds elapsed.  Handles 32-bit wraparound correctly via
 *         unsigned arithmetic.
 */
uint32_t SysTick_Elapsed(uint32_t start_tick)
{
    return HAL_GetTick() - start_tick;
}

/**
 * @brief  Get system uptime in whole seconds since SysTick_Init().
 * @retval Seconds elapsed since boot (wraps after ~136 years, safe).
 */
uint32_t SysTick_Uptime(void)
{
    return (HAL_GetTick() - g_startup_tick) / 1000u;
}

/* -------------------------------------------------------------------------- */
/*                      Non-blocking delay                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Start a non-blocking delay.
 * @param  ms: delay duration in milliseconds.
 * @note   Call SysTick_CheckDelay() in the main loop to poll completion.
 */
void SysTick_DelayMs(uint32_t ms)
{
    g_delay_target = HAL_GetTick() + ms;
    g_delay_active = 1;
}

/**
 * @brief  Poll whether a non-blocking delay started by SysTick_DelayMs() has
 *         expired.
 * @retval 1 if delay has expired or was never started; 0 if still running.
 * @note   Call this in the main loop.  Once it returns 1, g_delay_active is
 *         cleared so repeated calls return 1 until the next SysTick_DelayMs().
 */
uint8_t SysTick_CheckDelay(void)
{
    if (g_delay_active == 0) {
        return 1;                /* Idle — nothing pending               */
    }

    /* Unsigned subtraction handles wraparound: target "in the past" iff
       (now - target) does not underflow, i.e. now >= target.            */
    if ((int32_t)(HAL_GetTick() - g_delay_target) >= 0) {
        g_delay_active = 0;
        return 1;                /* Expired                              */
    }

    return 0;                    /* Still running                        */
}

/* -------------------------------------------------------------------------- */
/*                        Software timers                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Create a software timer.
 * @param  period_ms: period in milliseconds (must be > 0).
 * @param  oneshot:   1 = fire once and deactivate; 0 = periodic auto-reload.
 * @retval Timer ID (0 … SYSTICK_MAX_TIMERS-1) on success, or -1 if the pool
 *         is full.
 * @note   Timers are evaluated in SysTick_TimerExpired(), which must be called
 *         regularly from the main loop.
 */
int8_t SysTick_TimerCreate(uint32_t period_ms, uint8_t oneshot)
{
    if (period_ms == 0) {
        return -1;
    }

    for (int8_t i = 0; i < SYSTICK_MAX_TIMERS; i++) {
        if (g_timers[i].period_ms == 0) {
            g_timers[i].period_ms  = period_ms;
            g_timers[i].next_expiry = HAL_GetTick() + period_ms;
            g_timers[i].oneshot    = (oneshot != 0) ? 1 : 0;
            return i;
        }
    }

    return -1;  /* Pool full */
}

/**
 * @brief  Delete a software timer, freeing its slot.
 * @param  id: timer ID returned by SysTick_TimerCreate().
 */
void SysTick_TimerDelete(int8_t id)
{
    if (id >= 0 && id < SYSTICK_MAX_TIMERS) {
        g_timers[id].period_ms = 0;
    }
}

/**
 * @brief  Check whether a timer has expired (call from main loop).
 * @param  id: timer ID returned by SysTick_TimerCreate().
 * @retval 1 if the timer expired (and was auto-reloaded if periodic);
 *         0 otherwise.
 * @note   For periodic timers, the next expiry is scheduled automatically.
 *         For one-shot timers, the slot is deactivated after expiring.
 *         Must be called regularly — the function not only checks but also
 *         drives timer state transitions.
 */
uint8_t SysTick_TimerExpired(int8_t id)
{
    if (id < 0 || id >= SYSTICK_MAX_TIMERS) {
        return 0;
    }
    if (g_timers[id].period_ms == 0) {
        return 0;  /* Inactive */
    }

    /* Unsigned wraparound-safe check: (now - next_expiry) >= 0 means expired */
    if ((int32_t)(HAL_GetTick() - g_timers[id].next_expiry) >= 0) {
        if (g_timers[id].oneshot) {
            g_timers[id].period_ms = 0;   /* Deactivate after firing */
            return 1;
        } else {
            /* Periodic: advance next_expiry by period to avoid drift */
            g_timers[id].next_expiry += g_timers[id].period_ms;
            return 1;
        }
    }

    return 0;
}

/**
 * @brief  Reset a timer's countdown to its original period from now.
 * @param  id: timer ID returned by SysTick_TimerCreate().
 */
void SysTick_TimerReset(int8_t id)
{
    if (id >= 0 && id < SYSTICK_MAX_TIMERS && g_timers[id].period_ms != 0) {
        g_timers[id].next_expiry = HAL_GetTick() + g_timers[id].period_ms;
    }
}
