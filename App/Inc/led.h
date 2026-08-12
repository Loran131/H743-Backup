/**
 ******************************************************************************
 * @file    led.h
 * @brief   LED driver API — open-drain, active-low (ON = pin LOW)
 ******************************************************************************
 */

#ifndef __LED_H__
#define __LED_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* LED selection constants ---------------------------------------------------*/
#define LED_ID_0  0   /* PB1, board LED3 */
#define LED_ID_1  1   /* PB0, board LED2 */
#define LED_ID_2  2   /* PA15, board LED1 */

/* Exported functions --------------------------------------------------------*/
void LED_Init(void);
void LED_On(uint8_t led_id);
void LED_Off(uint8_t led_id);
void LED_Toggle(uint8_t led_id);
void LED_Heartbeat(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __LED_H__ */
