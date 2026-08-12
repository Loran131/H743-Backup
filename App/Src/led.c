/**
 ******************************************************************************
 * @file    led.c
 * @brief   LED driver — open-drain, active-low (ON = pin LOW, OFF = pin HIGH)
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "led.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void LED_Init(void)
{
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
}

void LED_On(uint8_t led_id)
{
  switch (led_id) {
    case LED_ID_0:
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
      break;
    case LED_ID_1:
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
      break;
    case LED_ID_2:
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
      break;
    default:
      break;
  }
}

void LED_Off(uint8_t led_id)
{
  switch (led_id) {
    case LED_ID_0:
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
      break;
    case LED_ID_1:
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
      break;
    case LED_ID_2:
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
      break;
    default:
      break;
  }
}

void LED_Toggle(uint8_t led_id)
{
  switch (led_id) {
    case LED_ID_0:
      HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
      break;
    case LED_ID_1:
      HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
      break;
    case LED_ID_2:
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
      break;
    default:
      break;
  }
}

void LED_Heartbeat(void)
{
  static uint32_t last_tick = 0;
  uint32_t now = HAL_GetTick();
  if ((now - last_tick) >= 500u) {
    last_tick = now;
    HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
